#include <gtest/gtest.h>

#include <cstring>

#include "proto/crc.hpp"

using namespace proto;

namespace {
const uint8_t* check() { return reinterpret_cast<const uint8_t*>("123456789"); }
}  // namespace

// Standard "check" values from the CRC catalogue (reveng.sourceforge.io).
TEST(Crc, Crc8MaximCheckValue) { EXPECT_EQ(crc8Maxim(check(), 9), 0xA1); }
TEST(Crc, Crc16CcittFalseCheckValue) { EXPECT_EQ(crc16Ccitt(check(), 9), 0x29B1); }
TEST(Crc, Crc16ModbusCheckValue) { EXPECT_EQ(crc16Modbus(check(), 9), 0x4B37); }

TEST(Crc, EmptyInputReturnsInit) {
    EXPECT_EQ(crc8Maxim(nullptr, 0), 0x00);
    EXPECT_EQ(crc16Ccitt(nullptr, 0), 0xFFFF);
    EXPECT_EQ(crc16Modbus(nullptr, 0), 0xFFFF);
}

TEST(Crc, IncrementalEqualsOneShot) {
    const uint8_t* d = check();
    EXPECT_EQ(crc16Ccitt(d + 4, 5, crc16Ccitt(d, 4)), crc16Ccitt(d, 9));
    EXPECT_EQ(crc16Modbus(d + 4, 5, crc16Modbus(d, 4)), crc16Modbus(d, 9));
    EXPECT_EQ(crc8Maxim(d + 4, 5, crc8Maxim(d, 4)), crc8Maxim(d, 9));
}

TEST(Crc, Crc8AppendedGivesZeroResidue) {
    uint8_t buf[10];
    std::memcpy(buf, check(), 9);
    buf[9] = crc8Maxim(buf, 9);
    EXPECT_EQ(crc8Maxim(buf, 10), 0x00);  // property used by 1-Wire ROM/scratchpad checks
}

TEST(Crc, SingleBitFlipAlwaysDetected) {
    uint8_t buf[9];
    std::memcpy(buf, check(), 9);
    const uint16_t ref = crc16Ccitt(buf, 9);
    for (int byte = 0; byte < 9; ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            buf[byte] ^= static_cast<uint8_t>(1 << bit);
            EXPECT_NE(crc16Ccitt(buf, 9), ref) << "byte " << byte << " bit " << bit;
            buf[byte] ^= static_cast<uint8_t>(1 << bit);
        }
    }
}
