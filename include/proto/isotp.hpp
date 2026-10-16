#pragma once
// ISO 15765-2 (ISO-TP) transport over classic CAN, normal addressing.
// Used by UDS (ISO 14229) / OBD-II diagnostics to move up to 4095 bytes.
//
// PCI types: 0x0 Single Frame, 0x1 First Frame, 0x2 Consecutive Frame, 0x3 Flow Control.
// Timing (N_Bs/N_Cr timeouts, STmin pacing) is left to the caller's scheduler:
// the sender exposes stMin() and the state, the caller decides when to call next.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "proto/hal.hpp"

namespace proto {

constexpr size_t kIsoTpMaxPayload = 4095;
constexpr uint8_t kIsoTpPadding = 0xCC;

class IsoTpSender {
public:
    enum class State { Idle, WaitFlowControl, SendConsecutive, Done, Error };

    explicit IsoTpSender(uint32_t txId) : txId_(txId) {}

    Status start(const uint8_t* data, size_t len);
    // First frame to transmit (SF or FF). After an SF the transfer is Done.
    bool firstFrame(CanFrame& out);
    // Feed a received Flow Control frame.
    Status onFlowControl(const CanFrame& fc);
    // Next Consecutive Frame, if allowed right now.
    bool nextFrame(CanFrame& out);

    State state() const { return state_; }
    uint8_t stMin() const { return stMin_; }

private:
    CanFrame blank() const;
    uint32_t txId_;
    std::vector<uint8_t> buf_;
    size_t offset_ = 0;
    uint8_t sn_ = 0;
    uint8_t blockSize_ = 0;
    uint8_t sentInBlock_ = 0;
    uint8_t stMin_ = 0;
    State state_ = State::Idle;
};

class IsoTpReceiver {
public:
    enum class Result { None, Complete, SendFlowControl, Error };

    IsoTpReceiver(uint32_t fcTxId, uint8_t blockSize = 0, uint8_t stMin = 0)
        : fcId_(fcTxId), blockSize_(blockSize), stMin_(stMin) {}

    // fcOut is filled when the result is SendFlowControl.
    Result onFrame(const CanFrame& f, CanFrame& fcOut);
    const std::vector<uint8_t>& payload() const { return buf_; }

private:
    CanFrame makeFc(uint8_t flowStatus) const;
    uint32_t fcId_;
    uint8_t blockSize_;
    uint8_t stMin_;
    std::vector<uint8_t> buf_;
    size_t expected_ = 0;
    uint8_t nextSn_ = 0;
    uint8_t cfInBlock_ = 0;
    bool inProgress_ = false;
};

}  // namespace proto
