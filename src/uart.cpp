#include "proto/uart.hpp"

#include <algorithm>

#include "proto/crc.hpp"

namespace proto {

size_t uartEncodeFrame(const uint8_t* payload, size_t len, uint8_t* out, size_t outCap) {
    if (len > kUartMaxPayload || (len > 0 && payload == nullptr) || out == nullptr) return 0;
    const size_t total = len + kUartOverhead;
    if (outCap < total) return 0;
    out[0] = kUartSof;
    out[1] = static_cast<uint8_t>(len);
    if (len) std::copy(payload, payload + len, out + 2);
    const uint16_t crc = crc16Ccitt(out + 1, len + 1);
    out[2 + len] = static_cast<uint8_t>(crc >> 8);
    out[3 + len] = static_cast<uint8_t>(crc & 0xFF);
    return total;
}

void UartFrameParser::feed(uint8_t b) {
    switch (state_) {
        case State::WaitSof:
            if (b == kUartSof) state_ = State::Len;
            else ++stats_.bytesDiscarded;
            break;
        case State::Len:
            buf_[0] = b;
            len_ = b;
            idx_ = 0;
            state_ = len_ ? State::Payload : State::CrcHi;
            break;
        case State::Payload:
            buf_[1 + idx_++] = b;
            if (idx_ == len_) state_ = State::CrcHi;
            break;
        case State::CrcHi:
            rxCrc_ = static_cast<uint16_t>(b << 8);
            state_ = State::CrcLo;
            break;
        case State::CrcLo:
            rxCrc_ |= b;
            state_ = State::WaitSof;
            if (crc16Ccitt(buf_.data(), len_ + 1) == rxCrc_) {
                ++stats_.framesOk;
                if (handler_) handler_(buf_.data() + 1, len_);
            } else {
                ++stats_.crcErrors;
            }
            break;
    }
}

Status UartLink::send(const uint8_t* payload, size_t len) {
    std::array<uint8_t, kUartMaxPayload + kUartOverhead> tx{};
    const size_t n = uartEncodeFrame(payload, len, tx.data(), tx.size());
    if (n == 0) return Status::InvalidArg;
    return port_.write(tx.data(), n) == n ? Status::Ok : Status::BusError;
}

size_t UartLink::poll(size_t maxBytes) {
    size_t n = 0;
    uint8_t b;
    while (n < maxBytes && port_.readByte(b)) {
        parser_.feed(b);
        ++n;
    }
    return n;
}

}  // namespace proto
