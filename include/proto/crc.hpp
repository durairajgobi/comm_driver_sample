#pragma once
#include <cstddef>
#include <cstdint>

namespace proto {

// Dallas/Maxim 1-Wire CRC-8 (poly 0x31 reflected = 0x8C, init 0x00).
uint8_t crc8Maxim(const uint8_t* data, size_t len, uint8_t init = 0x00);

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection). Used for UART framing.
uint16_t crc16Ccitt(const uint8_t* data, size_t len, uint16_t init = 0xFFFF);

// CRC-16/MODBUS (poly 0x8005 reflected = 0xA001, init 0xFFFF). Transmitted low byte first.
uint16_t crc16Modbus(const uint8_t* data, size_t len, uint16_t init = 0xFFFF);

}  // namespace proto
