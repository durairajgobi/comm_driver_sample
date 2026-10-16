#include "proto/isotp.hpp"

#include <algorithm>

namespace proto {

// ------------------------------------------------------------- sender ---

CanFrame IsoTpSender::blank() const {
    CanFrame f;
    f.id = txId_;
    f.dlc = 8;
    f.data.fill(kIsoTpPadding);
    return f;
}

Status IsoTpSender::start(const uint8_t* data, size_t len) {
    if (state_ == State::WaitFlowControl || state_ == State::SendConsecutive) return Status::Busy;
    if (data == nullptr || len == 0) return Status::InvalidArg;
    if (len > kIsoTpMaxPayload) return Status::Overflow;
    buf_.assign(data, data + len);
    offset_ = 0;
    sn_ = 1;
    sentInBlock_ = 0;
    state_ = State::Idle;
    return Status::Ok;
}

bool IsoTpSender::firstFrame(CanFrame& out) {
    if (buf_.empty() || state_ != State::Idle) return false;
    out = blank();
    if (buf_.size() <= 7) {
        out.data[0] = static_cast<uint8_t>(buf_.size());
        std::copy(buf_.begin(), buf_.end(), out.data.begin() + 1);
        offset_ = buf_.size();
        state_ = State::Done;
    } else {
        out.data[0] = static_cast<uint8_t>(0x10 | ((buf_.size() >> 8) & 0x0F));
        out.data[1] = static_cast<uint8_t>(buf_.size() & 0xFF);
        std::copy(buf_.begin(), buf_.begin() + 6, out.data.begin() + 2);
        offset_ = 6;
        state_ = State::WaitFlowControl;
    }
    return true;
}

Status IsoTpSender::onFlowControl(const CanFrame& fc) {
    if (state_ != State::WaitFlowControl) return Status::Busy;
    if (fc.dlc < 3 || (fc.data[0] & 0xF0) != 0x30) return Status::ProtocolError;
    switch (fc.data[0] & 0x0F) {
        case 0:  // ContinueToSend
            blockSize_ = fc.data[1];
            stMin_ = fc.data[2];
            sentInBlock_ = 0;
            state_ = State::SendConsecutive;
            return Status::Ok;
        case 1:  // Wait
            return Status::Ok;
        case 2:  // Overflow: receiver can't take this message
            state_ = State::Error;
            return Status::Overflow;
        default:
            state_ = State::Error;
            return Status::ProtocolError;
    }
}

bool IsoTpSender::nextFrame(CanFrame& out) {
    if (state_ != State::SendConsecutive) return false;
    out = blank();
    out.data[0] = static_cast<uint8_t>(0x20 | (sn_ & 0x0F));
    const size_t n = std::min<size_t>(7, buf_.size() - offset_);
    std::copy(buf_.begin() + static_cast<long>(offset_),
              buf_.begin() + static_cast<long>(offset_ + n), out.data.begin() + 1);
    offset_ += n;
    sn_ = static_cast<uint8_t>((sn_ + 1) & 0x0F);
    if (offset_ >= buf_.size()) {
        state_ = State::Done;
    } else if (blockSize_ != 0 && ++sentInBlock_ >= blockSize_) {
        state_ = State::WaitFlowControl;
    }
    return true;
}

// ----------------------------------------------------------- receiver ---

CanFrame IsoTpReceiver::makeFc(uint8_t flowStatus) const {
    CanFrame f;
    f.id = fcId_;
    f.dlc = 8;
    f.data.fill(kIsoTpPadding);
    f.data[0] = static_cast<uint8_t>(0x30 | flowStatus);
    f.data[1] = blockSize_;
    f.data[2] = stMin_;
    return f;
}

IsoTpReceiver::Result IsoTpReceiver::onFrame(const CanFrame& f, CanFrame& fcOut) {
    if (f.dlc == 0) return Result::Error;
    const uint8_t pci = f.data[0] >> 4;

    if (pci == 0x0) {  // Single Frame (a new SF aborts any transfer in progress)
        const uint8_t len = f.data[0] & 0x0F;
        if (len == 0 || len > 7 || len + 1u > f.dlc) return Result::Error;
        buf_.assign(f.data.begin() + 1, f.data.begin() + 1 + len);
        inProgress_ = false;
        return Result::Complete;
    }
    if (pci == 0x1) {  // First Frame
        if (f.dlc < 8) return Result::Error;
        expected_ = (static_cast<size_t>(f.data[0] & 0x0F) << 8) | f.data[1];
        if (expected_ <= 7) return Result::Error;
        buf_.assign(f.data.begin() + 2, f.data.begin() + 8);
        nextSn_ = 1;
        cfInBlock_ = 0;
        inProgress_ = true;
        fcOut = makeFc(0);  // CTS
        return Result::SendFlowControl;
    }
    if (pci == 0x2) {  // Consecutive Frame
        if (!inProgress_) return Result::None;  // stray CF, ignore
        if ((f.data[0] & 0x0F) != nextSn_) {
            inProgress_ = false;
            return Result::Error;  // lost/duplicated frame
        }
        nextSn_ = static_cast<uint8_t>((nextSn_ + 1) & 0x0F);
        const size_t n = std::min<size_t>(7, expected_ - buf_.size());
        if (n + 1 > f.dlc) { inProgress_ = false; return Result::Error; }
        buf_.insert(buf_.end(), f.data.begin() + 1, f.data.begin() + 1 + static_cast<long>(n));
        if (buf_.size() >= expected_) {
            inProgress_ = false;
            return Result::Complete;
        }
        if (blockSize_ != 0 && ++cfInBlock_ >= blockSize_) {
            cfInBlock_ = 0;
            fcOut = makeFc(0);
            return Result::SendFlowControl;
        }
        return Result::None;
    }
    return Result::None;  // FC frames are for the sender side
}

}  // namespace proto
