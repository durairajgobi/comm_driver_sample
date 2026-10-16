#pragma once
// Simple robust UART packet framing:
//
//   [0xA5 SOF] [LEN] [PAYLOAD x LEN] [CRC16 hi] [CRC16 lo]
//
// CRC-16/CCITT-FALSE over LEN + PAYLOAD. The byte-wise parser resynchronises on
// noise, truncated packets and CRC errors, so it can be fed straight from an
// RX interrupt or DMA ring buffer.

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "proto/hal.hpp"

namespace proto {

constexpr uint8_t kUartSof = 0xA5;
constexpr size_t kUartMaxPayload = 255;
constexpr size_t kUartOverhead = 4;

// Returns encoded size, or 0 if `out` is too small / payload too long.
size_t uartEncodeFrame(const uint8_t* payload, size_t len, uint8_t* out, size_t outCap);

class UartFrameParser {
public:
    using Handler = std::function<void(const uint8_t* payload, size_t len)>;
    struct Stats {
        uint32_t framesOk = 0;
        uint32_t crcErrors = 0;
        uint32_t bytesDiscarded = 0;
    };

    explicit UartFrameParser(Handler h) : handler_(std::move(h)) {}

    void feed(uint8_t byte);
    void feed(const uint8_t* data, size_t len) { for (size_t i = 0; i < len; ++i) feed(data[i]); }
    void reset() { state_ = State::WaitSof; }
    const Stats& stats() const { return stats_; }

private:
    enum class State { WaitSof, Len, Payload, CrcHi, CrcLo };
    Handler handler_;
    State state_ = State::WaitSof;
    std::array<uint8_t, kUartMaxPayload + 1> buf_{};  // [LEN, payload...]
    size_t len_ = 0;
    size_t idx_ = 0;
    uint16_t rxCrc_ = 0;
    Stats stats_;
};

// Convenience link: encode + write, and poll the port into the parser.
class UartLink {
public:
    UartLink(IUartPort& port, UartFrameParser::Handler h) : port_(port), parser_(std::move(h)) {}
    Status send(const uint8_t* payload, size_t len);
    size_t poll(size_t maxBytes = 512);  // returns number of bytes consumed
    const UartFrameParser::Stats& stats() const { return parser_.stats(); }

private:
    IUartPort& port_;
    UartFrameParser parser_;
};

}  // namespace proto
