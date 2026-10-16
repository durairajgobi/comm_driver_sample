#pragma once
// Hardware abstraction layer.
//
// Every driver in this library talks to hardware ONLY through these small
// interfaces. On target you implement them on top of your MCU vendor HAL
// (STM32 HAL, ESP-IDF, Zephyr, Linux /dev/i2c-*, spidev, SocketCAN, ...).
// In unit tests they are replaced by GoogleMock mocks or simulated devices.

#include <array>
#include <cstddef>
#include <cstdint>

#include "proto/status.hpp"

namespace proto {

// ---------------------------------------------------------------- timing ---
class IDelay {
public:
    virtual ~IDelay() = default;
    virtual void delayMs(uint32_t ms) = 0;
};

// ------------------------------------------------------------------- I2C ---
class II2cBus {
public:
    virtual ~II2cBus() = default;
    // START + addr(W) + data + [STOP]. sendStop=false leaves the bus open for
    // a repeated START (needed for register reads). len==0 is an address probe.
    virtual Status write(uint8_t addr7, const uint8_t* data, size_t len, bool sendStop = true) = 0;
    // (Repeated) START + addr(R) + len bytes + STOP.
    virtual Status read(uint8_t addr7, uint8_t* data, size_t len) = 0;
};

// ------------------------------------------------------------------- SPI ---
class ISpiBus {
public:
    virtual ~ISpiBus() = default;
    virtual void select(bool active) = 0;  // drive chip-select (active = asserted)
    // Full-duplex transfer. tx may be nullptr (send 0xFF), rx may be nullptr (discard).
    virtual Status transfer(const uint8_t* tx, uint8_t* rx, size_t len) = 0;
};

// Raw pins for the bit-banged SPI master (SoftSpi).
class ISpiPins {
public:
    virtual ~ISpiPins() = default;
    virtual void setCs(bool active) = 0;
    virtual void setSck(bool level) = 0;
    virtual void setMosi(bool level) = 0;
    virtual bool getMiso() = 0;
    virtual void halfPeriodDelay() {}
};

// ------------------------------------------------------------------- CAN ---
struct CanFrame {
    uint32_t id = 0;
    bool extended = false;  // 29-bit identifier
    bool rtr = false;       // remote transmission request
    uint8_t dlc = 0;        // 0..8
    std::array<uint8_t, 8> data{};
};

class ICanController {
public:
    virtual ~ICanController() = default;
    virtual Status send(const CanFrame& frame) = 0;
    virtual bool receive(CanFrame& frame) = 0;  // false = nothing pending / timeout
};

// ------------------------------------------------------------------ UART ---
class IUartPort {
public:
    virtual ~IUartPort() = default;
    virtual size_t write(const uint8_t* data, size_t len) = 0;
    // Returns false when no byte arrived within the port's inter-byte timeout.
    virtual bool readByte(uint8_t& byte) = 0;
};

// ---------------------------------------------------------------- 1-Wire ---
class IOneWireBus {
public:
    virtual ~IOneWireBus() = default;
    virtual bool reset() = 0;  // true = presence pulse detected
    virtual void writeBit(bool bit) = 0;
    virtual bool readBit() = 0;
};

}  // namespace proto
