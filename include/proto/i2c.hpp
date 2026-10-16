#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "proto/hal.hpp"

namespace proto {

// Generic 8-bit-register I2C peripheral (sensors, PMICs, IO expanders, RTCs...).
class I2cRegisterDevice {
public:
    static constexpr size_t kMaxBurst = 32;

    I2cRegisterDevice(II2cBus& bus, uint8_t addr7) : bus_(bus), addr_(addr7) {}

    uint8_t address() const { return addr_; }

    Status writeReg(uint8_t reg, uint8_t value);
    Status readReg(uint8_t reg, uint8_t& value);
    Status writeRegs(uint8_t reg, const uint8_t* data, size_t len);
    Status readRegs(uint8_t reg, uint8_t* data, size_t len);
    // Read-modify-write: only bits in `mask` are changed. Skips the write if unchanged.
    Status updateBits(uint8_t reg, uint8_t mask, uint8_t value);
    // Two consecutive registers as a big-endian 16-bit value (MSB at `reg`).
    Status readReg16BE(uint8_t reg, uint16_t& value);

private:
    bool validAddr() const { return addr_ <= 0x7F; }
    II2cBus& bus_;
    uint8_t addr_;
};

// Probes 7-bit addresses 0x08..0x77 with a zero-length write.
std::vector<uint8_t> i2cScan(II2cBus& bus);

// 24xx-series EEPROM with 16-bit word address (24LC64 .. 24LC512, AT24C256, ...).
class Eeprom24 {
public:
    static constexpr size_t kMaxPageSize = 256;

    struct Config {
        uint8_t addr7 = 0x50;
        uint32_t capacityBytes = 32768;  // 24LC256
        uint16_t pageSize = 64;
        uint32_t maxAckPolls = 20;      // ~1 ms each, tWR is typically 5 ms
    };

    Eeprom24(II2cBus& bus, IDelay& delay, Config cfg) : bus_(bus), delay_(delay), cfg_(cfg) {}

    Status read(uint32_t memAddr, uint8_t* data, size_t len);
    // Splits writes at page boundaries and ACK-polls after each page.
    Status write(uint32_t memAddr, const uint8_t* data, size_t len);

private:
    Status waitWriteComplete();
    II2cBus& bus_;
    IDelay& delay_;
    Config cfg_;
};

}  // namespace proto
