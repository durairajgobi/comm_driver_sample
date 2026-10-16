#include "proto/modbus.hpp"

#include <cstring>

#include "proto/crc.hpp"

namespace proto {

static size_t appendCrc(uint8_t* out, size_t len) {
    const uint16_t crc = crc16Modbus(out, len);
    out[len] = static_cast<uint8_t>(crc & 0xFF);  // Modbus: CRC low byte first
    out[len + 1] = static_cast<uint8_t>(crc >> 8);
    return len + 2;
}

bool mbCheckCrc(const uint8_t* f, size_t len) {
    if (f == nullptr || len < 4) return false;
    const uint16_t crc = crc16Modbus(f, len - 2);
    return f[len - 2] == (crc & 0xFF) && f[len - 1] == (crc >> 8);
}

size_t mbBuildReadRegisters(uint8_t slave, uint8_t fn, uint16_t start, uint16_t qty, uint8_t* out,
                            size_t cap) {
    if (out == nullptr || cap < 8 || slave == 0 || slave > 247) return 0;
    if (fn != kMbReadHoldingRegs && fn != kMbReadInputRegs) return 0;
    if (qty == 0 || qty > kMbMaxReadRegs) return 0;
    out[0] = slave;
    out[1] = fn;
    out[2] = static_cast<uint8_t>(start >> 8);
    out[3] = static_cast<uint8_t>(start);
    out[4] = static_cast<uint8_t>(qty >> 8);
    out[5] = static_cast<uint8_t>(qty);
    return appendCrc(out, 6);
}

size_t mbBuildWriteSingle(uint8_t slave, uint16_t addr, uint16_t value, uint8_t* out, size_t cap) {
    if (out == nullptr || cap < 8 || slave > 247) return 0;  // slave 0 = broadcast allowed
    out[0] = slave;
    out[1] = kMbWriteSingleReg;
    out[2] = static_cast<uint8_t>(addr >> 8);
    out[3] = static_cast<uint8_t>(addr);
    out[4] = static_cast<uint8_t>(value >> 8);
    out[5] = static_cast<uint8_t>(value);
    return appendCrc(out, 6);
}

size_t mbBuildWriteMultiple(uint8_t slave, uint16_t start, const uint16_t* values, uint16_t qty,
                            uint8_t* out, size_t cap) {
    if (out == nullptr || values == nullptr || slave > 247) return 0;
    if (qty == 0 || qty > kMbMaxWriteRegs) return 0;
    const size_t total = 7 + 2u * qty + 2;
    if (cap < total) return 0;
    out[0] = slave;
    out[1] = kMbWriteMultipleRegs;
    out[2] = static_cast<uint8_t>(start >> 8);
    out[3] = static_cast<uint8_t>(start);
    out[4] = static_cast<uint8_t>(qty >> 8);
    out[5] = static_cast<uint8_t>(qty);
    out[6] = static_cast<uint8_t>(2 * qty);
    for (uint16_t i = 0; i < qty; ++i) {
        out[7 + 2 * i] = static_cast<uint8_t>(values[i] >> 8);
        out[8 + 2 * i] = static_cast<uint8_t>(values[i]);
    }
    return appendCrc(out, 7 + 2u * qty);
}

Status mbParseReadRegisters(const uint8_t* f, size_t len, uint8_t slave, uint8_t fn,
                            uint16_t* regs, uint16_t qty, uint8_t& exceptionCode) {
    exceptionCode = 0;
    if (f == nullptr || regs == nullptr) return Status::InvalidArg;
    if (!mbCheckCrc(f, len)) return Status::CrcError;
    if (f[0] != slave) return Status::ProtocolError;
    if (f[1] == (fn | 0x80)) {
        if (len != 5) return Status::ProtocolError;
        exceptionCode = f[2];
        return Status::DeviceException;
    }
    if (f[1] != fn) return Status::ProtocolError;
    const size_t byteCount = f[2];
    if (byteCount != 2u * qty || len != 5 + byteCount) return Status::ProtocolError;
    for (uint16_t i = 0; i < qty; ++i) {
        regs[i] = static_cast<uint16_t>((f[3 + 2 * i] << 8) | f[4 + 2 * i]);
    }
    return Status::Ok;
}

// ------------------------------------------------------------- master ---

Status ModbusRtuMaster::receive(uint8_t* buf, size_t expected, size_t& got) {
    got = 0;
    while (got < expected) {
        if (!port_.readByte(buf[got])) return got == 0 ? Status::Timeout : Status::ProtocolError;
        ++got;
        if (got == 5 && (buf[1] & 0x80)) return Status::Ok;  // exception response
    }
    return Status::Ok;
}

Status ModbusRtuMaster::readHoldingRegisters(uint8_t slave, uint16_t start, uint16_t qty,
                                             uint16_t* regs) {
    uint8_t req[8];
    const size_t n = mbBuildReadRegisters(slave, kMbReadHoldingRegs, start, qty, req, sizeof(req));
    if (n == 0 || regs == nullptr) return Status::InvalidArg;
    if (port_.write(req, n) != n) return Status::BusError;

    uint8_t rsp[kMbMaxAdu];
    size_t got = 0;
    Status s = receive(rsp, 5 + 2u * qty, got);
    if (s != Status::Ok) return s;
    s = mbParseReadRegisters(rsp, got, slave, kMbReadHoldingRegs, regs, qty, lastException_);
    return s;
}

Status ModbusRtuMaster::transactEcho(const uint8_t* req, size_t len, uint8_t slave) {
    if (port_.write(req, len) != len) return Status::BusError;
    if (slave == 0) return Status::Ok;  // broadcast: no reply
    uint8_t rsp[8];
    size_t got = 0;
    Status s = receive(rsp, 8, got);
    if (s != Status::Ok) return s;
    if (!mbCheckCrc(rsp, got)) return Status::CrcError;
    if (got == 5) {
        lastException_ = rsp[2];
        return Status::DeviceException;
    }
    // Both FC06 and FC16 echo the first 6 bytes of the request.
    return std::memcmp(rsp, req, 6) == 0 ? Status::Ok : Status::ProtocolError;
}

Status ModbusRtuMaster::writeSingleRegister(uint8_t slave, uint16_t addr, uint16_t value) {
    uint8_t req[8];
    const size_t n = mbBuildWriteSingle(slave, addr, value, req, sizeof(req));
    if (n == 0) return Status::InvalidArg;
    return transactEcho(req, n, slave);
}

Status ModbusRtuMaster::writeMultipleRegisters(uint8_t slave, uint16_t start,
                                               const uint16_t* values, uint16_t qty) {
    uint8_t req[kMbMaxAdu];
    const size_t n = mbBuildWriteMultiple(slave, start, values, qty, req, sizeof(req));
    if (n == 0) return Status::InvalidArg;
    return transactEcho(req, n, slave);
}

}  // namespace proto
