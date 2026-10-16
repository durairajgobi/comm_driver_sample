#include "proto/onewire.hpp"

#include "proto/crc.hpp"

namespace proto {

bool oneWireRomValid(const RomCode& rom) {
    if (rom[0] == 0x00) return false;
    return crc8Maxim(rom.data(), 7) == rom[7];
}

void OneWire::writeByte(uint8_t b) {
    for (int i = 0; i < 8; ++i) bus_.writeBit((b >> i) & 1);
}

uint8_t OneWire::readByte() {
    uint8_t b = 0;
    for (int i = 0; i < 8; ++i) {
        if (bus_.readBit()) b |= static_cast<uint8_t>(1u << i);
    }
    return b;
}

Status OneWire::readRom(RomCode& rom) {
    if (!bus_.reset()) return Status::Timeout;  // no presence pulse
    writeByte(kReadRom);
    read(rom.data(), rom.size());
    return oneWireRomValid(rom) ? Status::Ok : Status::CrcError;
}

Status OneWire::select(const RomCode& rom) {
    if (!bus_.reset()) return Status::Timeout;
    writeByte(kMatchRom);
    write(rom.data(), rom.size());
    return Status::Ok;
}

Status OneWire::skip() {
    if (!bus_.reset()) return Status::Timeout;
    writeByte(kSkipRom);
    return Status::Ok;
}

void OneWire::searchReset() {
    romNo_.fill(0);
    lastDiscrepancy_ = 0;
    lastDevice_ = false;
}

bool OneWire::searchNext(RomCode& rom) {
    if (lastDevice_) return false;
    if (!bus_.reset()) {
        searchReset();
        return false;
    }
    writeByte(kSearchRom);

    int lastZero = 0;
    int idBitNumber = 1;
    for (int byteIdx = 0; byteIdx < 8; ++byteIdx) {
        for (uint8_t mask = 1; mask != 0; mask = static_cast<uint8_t>(mask << 1), ++idBitNumber) {
            const bool idBit = bus_.readBit();
            const bool cmpBit = bus_.readBit();
            if (idBit && cmpBit) {  // no device responded
                searchReset();
                return false;
            }
            bool dir;
            if (idBit != cmpBit) {
                dir = idBit;  // all remaining devices agree on this bit
            } else {
                // Discrepancy: devices with 0 and 1 here.
                if (idBitNumber < lastDiscrepancy_) dir = (romNo_[byteIdx] & mask) != 0;
                else dir = (idBitNumber == lastDiscrepancy_);
                if (!dir) lastZero = idBitNumber;
            }
            if (dir) romNo_[byteIdx] |= mask;
            else romNo_[byteIdx] &= static_cast<uint8_t>(~mask);
            bus_.writeBit(dir);
        }
    }

    lastDiscrepancy_ = lastZero;
    if (lastDiscrepancy_ == 0) lastDevice_ = true;
    if (!oneWireRomValid(romNo_)) {
        searchReset();
        return false;
    }
    rom = romNo_;
    return true;
}

std::vector<RomCode> OneWire::searchAll(size_t maxDevices) {
    std::vector<RomCode> out;
    searchReset();
    RomCode rom;
    while (out.size() < maxDevices && searchNext(rom)) out.push_back(rom);
    return out;
}

// ------------------------------------------------------------- DS18B20 ---

Status Ds18b20::address(const RomCode* rom) { return rom ? ow_.select(*rom) : ow_.skip(); }

Status Ds18b20::startConversion(const RomCode* rom) {
    Status s = address(rom);
    if (s != Status::Ok) return s;
    ow_.writeByte(kConvertT);
    return Status::Ok;
}

Status Ds18b20::readTemperature(float& celsius, const RomCode* rom) {
    Status s = address(rom);
    if (s != Status::Ok) return s;
    ow_.writeByte(kReadScratchpad);
    uint8_t sp[9] = {};
    ow_.read(sp, sizeof(sp));
    if (crc8Maxim(sp, 8) != sp[8]) return Status::CrcError;
    const int16_t raw = static_cast<int16_t>(static_cast<uint16_t>(sp[1] << 8) | sp[0]);
    celsius = rawToCelsius(raw);
    return Status::Ok;
}

}  // namespace proto
