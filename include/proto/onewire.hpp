#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "proto/hal.hpp"

namespace proto {

using RomCode = std::array<uint8_t, 8>;  // [family][serial x6][crc8]

class OneWire {
public:
    enum RomCmd : uint8_t { kSearchRom = 0xF0, kReadRom = 0x33, kMatchRom = 0x55, kSkipRom = 0xCC };

    explicit OneWire(IOneWireBus& bus) : bus_(bus) {}

    bool reset() { return bus_.reset(); }
    void writeByte(uint8_t b);   // LSB first
    uint8_t readByte();          // LSB first
    void write(const uint8_t* d, size_t n) { for (size_t i = 0; i < n; ++i) writeByte(d[i]); }
    void read(uint8_t* d, size_t n) { for (size_t i = 0; i < n; ++i) d[i] = readByte(); }

    // Only valid with a single device on the bus.
    Status readRom(RomCode& rom);
    Status select(const RomCode& rom);  // reset + MATCH ROM
    Status skip();                      // reset + SKIP ROM

    // Maxim AN187 ROM search. Call searchReset() then searchNext() until false.
    void searchReset();
    bool searchNext(RomCode& rom);
    std::vector<RomCode> searchAll(size_t maxDevices = 16);

private:
    IOneWireBus& bus_;
    RomCode romNo_{};
    int lastDiscrepancy_ = 0;
    bool lastDevice_ = false;
};

bool oneWireRomValid(const RomCode& rom);

// DS18B20 digital thermometer.
class Ds18b20 {
public:
    static constexpr uint8_t kFamily = 0x28;
    enum Cmd : uint8_t { kConvertT = 0x44, kReadScratchpad = 0xBE };

    explicit Ds18b20(OneWire& ow) : ow_(ow) {}

    // Addresses a specific sensor if `rom` is given, otherwise SKIP ROM (single drop).
    Status startConversion(const RomCode* rom = nullptr);
    Status readTemperature(float& celsius, const RomCode* rom = nullptr);

    static float rawToCelsius(int16_t raw) { return static_cast<float>(raw) / 16.0f; }

private:
    Status address(const RomCode* rom);
    OneWire& ow_;
};

}  // namespace proto
