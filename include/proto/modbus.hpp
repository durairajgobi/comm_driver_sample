#pragma once
// Modbus RTU (over RS-485 / UART) master: request builders, response parsers,
// and a blocking master on top of IUartPort.

#include <cstddef>
#include <cstdint>

#include "proto/hal.hpp"

namespace proto {

enum ModbusFn : uint8_t {
    kMbReadCoils = 0x01,
    kMbReadHoldingRegs = 0x03,
    kMbReadInputRegs = 0x04,
    kMbWriteSingleReg = 0x06,
    kMbWriteMultipleRegs = 0x10,
};

constexpr size_t kMbMaxAdu = 256;
constexpr uint16_t kMbMaxReadRegs = 125;
constexpr uint16_t kMbMaxWriteRegs = 123;

// All builders return frame length (CRC included) or 0 on bad arguments.
size_t mbBuildReadRegisters(uint8_t slave, uint8_t fn, uint16_t start, uint16_t qty,
                            uint8_t* out, size_t cap);
size_t mbBuildWriteSingle(uint8_t slave, uint16_t addr, uint16_t value, uint8_t* out, size_t cap);
size_t mbBuildWriteMultiple(uint8_t slave, uint16_t start, const uint16_t* values, uint16_t qty,
                            uint8_t* out, size_t cap);

bool mbCheckCrc(const uint8_t* frame, size_t len);

// Parses a read-registers response. On a Modbus exception returns DeviceException
// and sets `exceptionCode`.
Status mbParseReadRegisters(const uint8_t* frame, size_t len, uint8_t slave, uint8_t fn,
                            uint16_t* regs, uint16_t qty, uint8_t& exceptionCode);

class ModbusRtuMaster {
public:
    explicit ModbusRtuMaster(IUartPort& port) : port_(port) {}

    Status readHoldingRegisters(uint8_t slave, uint16_t start, uint16_t qty, uint16_t* regs);
    Status writeSingleRegister(uint8_t slave, uint16_t addr, uint16_t value);
    Status writeMultipleRegisters(uint8_t slave, uint16_t start, const uint16_t* values, uint16_t qty);
    uint8_t lastException() const { return lastException_; }

private:
    // Reads a response; stops early on an exception frame (5 bytes).
    Status receive(uint8_t* buf, size_t expected, size_t& got);
    Status transactEcho(const uint8_t* req, size_t len, uint8_t slave);
    IUartPort& port_;
    uint8_t lastException_ = 0;
};

}  // namespace proto
