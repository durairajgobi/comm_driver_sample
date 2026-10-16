#include "proto/spi.hpp"

#include <algorithm>

namespace proto {

// ------------------------------------------------------------- SoftSpi ---

SoftSpi::SoftSpi(ISpiPins& pins, SpiMode mode, BitOrder order)
    : pins_(pins), cpol_(spiCpol(mode)), cpha_(spiCpha(mode)), order_(order) {
    pins_.setCs(false);
    pins_.setSck(cpol_);  // idle clock level
}

void SoftSpi::select(bool active) {
    pins_.setSck(cpol_);
    pins_.setCs(active);
}

uint8_t SoftSpi::transferByte(uint8_t out) {
    uint8_t in = 0;
    for (int i = 0; i < 8; ++i) {
        const int bitPos = (order_ == BitOrder::MsbFirst) ? 7 - i : i;
        const bool outBit = (out >> bitPos) & 1;
        bool inBit;
        if (!cpha_) {
            // CPHA=0: data valid before the leading edge, sampled on leading edge.
            pins_.setMosi(outBit);
            pins_.halfPeriodDelay();
            pins_.setSck(!cpol_);  // leading edge -> sample
            inBit = pins_.getMiso();
            pins_.halfPeriodDelay();
            pins_.setSck(cpol_);   // trailing edge -> slave shifts
        } else {
            // CPHA=1: data changes on leading edge, sampled on trailing edge.
            pins_.setSck(!cpol_);  // leading edge -> shift
            pins_.setMosi(outBit);
            pins_.halfPeriodDelay();
            pins_.setSck(cpol_);   // trailing edge -> sample
            inBit = pins_.getMiso();
            pins_.halfPeriodDelay();
        }
        if (inBit) in |= static_cast<uint8_t>(1u << bitPos);
    }
    return in;
}

Status SoftSpi::transfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        const uint8_t r = transferByte(tx ? tx[i] : 0xFF);
        if (rx) rx[i] = r;
    }
    return Status::Ok;
}

// ------------------------------------------------------------ SpiFlash ---

bool SpiFlash::inRange(uint32_t addr, size_t len) const {
    if (addr > 0xFFFFFF) return false;
    if (cfg_.capacityBytes == 0) return true;
    return static_cast<uint64_t>(addr) + len <= cfg_.capacityBytes;
}

Status SpiFlash::command(const uint8_t* hdr, size_t hdrLen, const uint8_t* tx, uint8_t* rx,
                         size_t len) {
    bus_.select(true);
    Status s = bus_.transfer(hdr, nullptr, hdrLen);
    if (s == Status::Ok && len > 0) s = bus_.transfer(tx, rx, len);
    bus_.select(false);  // always release CS, even on error
    return s;
}

Status SpiFlash::readJedecId(JedecId& id) {
    const uint8_t cmd = kJedecId;
    uint8_t r[3] = {};
    Status s = command(&cmd, 1, nullptr, r, 3);
    if (s != Status::Ok) return s;
    if ((r[0] == 0x00 && r[1] == 0x00 && r[2] == 0x00) ||
        (r[0] == 0xFF && r[1] == 0xFF && r[2] == 0xFF)) {
        return Status::BusError;  // MISO stuck low/high: no chip
    }
    id.manufacturer = r[0];
    id.memoryType = r[1];
    id.capacityCode = r[2];
    return Status::Ok;
}

Status SpiFlash::readStatus(uint8_t& status) {
    const uint8_t cmd = kReadStatus1;
    return command(&cmd, 1, nullptr, &status, 1);
}

Status SpiFlash::waitReady() {
    for (uint32_t i = 0; i < cfg_.maxBusyPolls; ++i) {
        uint8_t st = 0;
        Status s = readStatus(st);
        if (s != Status::Ok) return s;
        if ((st & kStatusBusy) == 0) return Status::Ok;
        delay_.delayMs(1);
    }
    return Status::Timeout;
}

Status SpiFlash::writeEnable() {
    const uint8_t cmd = kWriteEnable;
    Status s = command(&cmd, 1, nullptr, nullptr, 0);
    if (s != Status::Ok) return s;
    uint8_t st = 0;
    s = readStatus(st);
    if (s != Status::Ok) return s;
    return (st & kStatusWel) ? Status::Ok : Status::ProtocolError;  // write-protected?
}

Status SpiFlash::read(uint32_t addr, uint8_t* data, size_t len) {
    if (data == nullptr || len == 0 || !inRange(addr, len)) return Status::InvalidArg;
    const uint8_t hdr[4] = {kReadData, static_cast<uint8_t>(addr >> 16),
                            static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr)};
    return command(hdr, 4, nullptr, data, len);
}

Status SpiFlash::write(uint32_t addr, const uint8_t* data, size_t len) {
    if (data == nullptr || len == 0 || !inRange(addr, len)) return Status::InvalidArg;
    size_t done = 0;
    while (done < len) {
        const uint32_t a = addr + static_cast<uint32_t>(done);
        const size_t chunk = std::min<size_t>(cfg_.pageSize - (a % cfg_.pageSize), len - done);
        Status s = writeEnable();
        if (s != Status::Ok) return s;
        const uint8_t hdr[4] = {kPageProgram, static_cast<uint8_t>(a >> 16),
                                static_cast<uint8_t>(a >> 8), static_cast<uint8_t>(a)};
        s = command(hdr, 4, data + done, nullptr, chunk);
        if (s != Status::Ok) return s;
        s = waitReady();
        if (s != Status::Ok) return s;
        done += chunk;
    }
    return Status::Ok;
}

Status SpiFlash::eraseSector(uint32_t addr) {
    if (!inRange(addr, 1)) return Status::InvalidArg;
    addr &= ~(kSectorSize - 1);
    Status s = writeEnable();
    if (s != Status::Ok) return s;
    const uint8_t hdr[4] = {kSectorErase4K, static_cast<uint8_t>(addr >> 16),
                            static_cast<uint8_t>(addr >> 8), static_cast<uint8_t>(addr)};
    s = command(hdr, 4, nullptr, nullptr, 0);
    if (s != Status::Ok) return s;
    return waitReady();
}

}  // namespace proto
