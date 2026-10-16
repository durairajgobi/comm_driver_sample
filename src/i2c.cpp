#include "proto/i2c.hpp"

#include <algorithm>
#include <array>

namespace proto {

Status I2cRegisterDevice::writeReg(uint8_t reg, uint8_t value) {
    if (!validAddr()) return Status::InvalidArg;
    const uint8_t buf[2] = {reg, value};
    return bus_.write(addr_, buf, sizeof(buf), true);
}

Status I2cRegisterDevice::readReg(uint8_t reg, uint8_t& value) {
    return readRegs(reg, &value, 1);
}

Status I2cRegisterDevice::writeRegs(uint8_t reg, const uint8_t* data, size_t len) {
    if (!validAddr() || data == nullptr || len == 0) return Status::InvalidArg;
    if (len > kMaxBurst) return Status::Overflow;
    std::array<uint8_t, kMaxBurst + 1> buf{};
    buf[0] = reg;
    std::copy(data, data + len, buf.begin() + 1);
    return bus_.write(addr_, buf.data(), len + 1, true);
}

Status I2cRegisterDevice::readRegs(uint8_t reg, uint8_t* data, size_t len) {
    if (!validAddr() || data == nullptr || len == 0) return Status::InvalidArg;
    // Write register pointer without STOP, then repeated-START read.
    Status s = bus_.write(addr_, &reg, 1, false);
    if (s != Status::Ok) return s;
    return bus_.read(addr_, data, len);
}

Status I2cRegisterDevice::updateBits(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t cur = 0;
    Status s = readReg(reg, cur);
    if (s != Status::Ok) return s;
    const uint8_t next = static_cast<uint8_t>((cur & ~mask) | (value & mask));
    if (next == cur) return Status::Ok;
    return writeReg(reg, next);
}

Status I2cRegisterDevice::readReg16BE(uint8_t reg, uint16_t& value) {
    uint8_t buf[2] = {};
    Status s = readRegs(reg, buf, 2);
    if (s == Status::Ok) value = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
    return s;
}

std::vector<uint8_t> i2cScan(II2cBus& bus) {
    std::vector<uint8_t> found;
    for (uint8_t a = 0x08; a <= 0x77; ++a) {
        if (bus.write(a, nullptr, 0, true) == Status::Ok) found.push_back(a);
    }
    return found;
}

// ------------------------------------------------------------- EEPROM ---

Status Eeprom24::read(uint32_t memAddr, uint8_t* data, size_t len) {
    if (data == nullptr || len == 0) return Status::InvalidArg;
    if (memAddr + len > cfg_.capacityBytes) return Status::InvalidArg;
    const uint8_t a[2] = {static_cast<uint8_t>(memAddr >> 8), static_cast<uint8_t>(memAddr)};
    Status s = bus_.write(cfg_.addr7, a, 2, false);
    if (s != Status::Ok) return s;
    return bus_.read(cfg_.addr7, data, len);  // sequential read, address auto-increments
}

Status Eeprom24::write(uint32_t memAddr, const uint8_t* data, size_t len) {
    if (data == nullptr || len == 0) return Status::InvalidArg;
    if (cfg_.pageSize == 0 || cfg_.pageSize > kMaxPageSize) return Status::InvalidArg;
    if (memAddr + len > cfg_.capacityBytes) return Status::InvalidArg;

    std::array<uint8_t, kMaxPageSize + 2> buf{};
    size_t done = 0;
    while (done < len) {
        const uint32_t addr = memAddr + static_cast<uint32_t>(done);
        const size_t roomInPage = cfg_.pageSize - (addr % cfg_.pageSize);
        const size_t chunk = std::min(roomInPage, len - done);

        buf[0] = static_cast<uint8_t>(addr >> 8);
        buf[1] = static_cast<uint8_t>(addr);
        std::copy(data + done, data + done + chunk, buf.begin() + 2);

        Status s = bus_.write(cfg_.addr7, buf.data(), chunk + 2, true);
        if (s != Status::Ok) return s;
        s = waitWriteComplete();
        if (s != Status::Ok) return s;
        done += chunk;
    }
    return Status::Ok;
}

Status Eeprom24::waitWriteComplete() {
    // During the internal write cycle the device NACKs its address.
    for (uint32_t i = 0; i < cfg_.maxAckPolls; ++i) {
        if (bus_.write(cfg_.addr7, nullptr, 0, true) == Status::Ok) return Status::Ok;
        delay_.delayMs(1);
    }
    return Status::Timeout;
}

}  // namespace proto
