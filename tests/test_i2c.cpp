#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <numeric>

#include "mocks.hpp"
#include "proto/i2c.hpp"

using namespace proto;
using namespace proto::test;
using ::testing::_;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::NiceMock;
using ::testing::Return;
using ::testing::SetArrayArgument;

// =========================================================================
// I2cRegisterDevice -- interaction tests with GoogleMock
// =========================================================================
class I2cRegisterDeviceTest : public ::testing::Test {
protected:
    MockI2cBus bus;
    I2cRegisterDevice dev{bus, 0x48};
};

TEST_F(I2cRegisterDeviceTest, WriteRegSendsRegisterThenValueWithStop) {
    EXPECT_CALL(bus, write(0x48, _, 2, true)).With(I2cBytes(V({0x01, 0x60}))).WillOnce(Return(Status::Ok));
    EXPECT_EQ(dev.writeReg(0x01, 0x60), Status::Ok);
}

TEST_F(I2cRegisterDeviceTest, ReadRegUsesRepeatedStart) {
    const uint8_t value = 0xA7;
    InSequence seq;
    EXPECT_CALL(bus, write(0x48, _, 1, false)).With(I2cBytes(V({0x0F}))).WillOnce(Return(Status::Ok));
    EXPECT_CALL(bus, read(0x48, _, 1))
        .WillOnce(DoAll(SetArrayArgument<1>(&value, &value + 1), Return(Status::Ok)));
    uint8_t out = 0;
    EXPECT_EQ(dev.readReg(0x0F, out), Status::Ok);
    EXPECT_EQ(out, 0xA7);
}

TEST_F(I2cRegisterDeviceTest, ReadReg16BigEndian) {
    const uint8_t raw[2] = {0x1A, 0x2B};
    EXPECT_CALL(bus, write(0x48, _, 1, false)).WillOnce(Return(Status::Ok));
    EXPECT_CALL(bus, read(0x48, _, 2))
        .WillOnce(DoAll(SetArrayArgument<1>(raw, raw + 2), Return(Status::Ok)));
    uint16_t v = 0;
    EXPECT_EQ(dev.readReg16BE(0x00, v), Status::Ok);
    EXPECT_EQ(v, 0x1A2B);
}

TEST_F(I2cRegisterDeviceTest, NackOnAddressPhaseAbortsRead) {
    EXPECT_CALL(bus, write(_, _, _, _)).WillOnce(Return(Status::Nack));
    EXPECT_CALL(bus, read(_, _, _)).Times(0);
    uint8_t v;
    EXPECT_EQ(dev.readReg(0x00, v), Status::Nack);
}

TEST_F(I2cRegisterDeviceTest, UpdateBitsOnlyTouchesMaskedBits) {
    const uint8_t cur = 0b1010'1010;
    EXPECT_CALL(bus, write(0x48, _, 1, false)).WillOnce(Return(Status::Ok));
    EXPECT_CALL(bus, read(0x48, _, 1))
        .WillOnce(DoAll(SetArrayArgument<1>(&cur, &cur + 1), Return(Status::Ok)));
    EXPECT_CALL(bus, write(0x48, _, 2, true)).With(I2cBytes(V({0x20, 0b1010'0101})))
        .WillOnce(Return(Status::Ok));
    EXPECT_EQ(dev.updateBits(0x20, 0x0F, 0x05), Status::Ok);
}

TEST_F(I2cRegisterDeviceTest, UpdateBitsSkipsWriteWhenUnchanged) {
    const uint8_t cur = 0x35;
    EXPECT_CALL(bus, write(0x48, _, 1, false)).WillOnce(Return(Status::Ok));
    EXPECT_CALL(bus, read(0x48, _, 1))
        .WillOnce(DoAll(SetArrayArgument<1>(&cur, &cur + 1), Return(Status::Ok)));
    EXPECT_CALL(bus, write(_, _, 2, true)).Times(0);
    EXPECT_EQ(dev.updateBits(0x20, 0x0F, 0x05), Status::Ok);
}

TEST_F(I2cRegisterDeviceTest, BurstWriteBuildsSingleTransaction) {
    const uint8_t data[3] = {1, 2, 3};
    EXPECT_CALL(bus, write(0x48, _, 4, true)).With(I2cBytes(V({0x10, 1, 2, 3}))).WillOnce(Return(Status::Ok));
    EXPECT_EQ(dev.writeRegs(0x10, data, 3), Status::Ok);
}

TEST_F(I2cRegisterDeviceTest, BurstWriteRejectsOversize) {
    uint8_t big[I2cRegisterDevice::kMaxBurst + 1] = {};
    EXPECT_CALL(bus, write(_, _, _, _)).Times(0);
    EXPECT_EQ(dev.writeRegs(0x10, big, sizeof(big)), Status::Overflow);
}

TEST(I2cRegisterDevice, RejectsEightBitAddress) {
    MockI2cBus bus;
    I2cRegisterDevice dev(bus, 0x90);  // common mistake: 8-bit address
    EXPECT_CALL(bus, write(_, _, _, _)).Times(0);
    EXPECT_EQ(dev.writeReg(0, 0), Status::InvalidArg);
}

TEST(I2cScan, ReportsOnlyAckingAddresses) {
    NiceMock<MockI2cBus> bus;
    ON_CALL(bus, write(_, nullptr, 0, true)).WillByDefault(Return(Status::Nack));
    ON_CALL(bus, write(0x3C, nullptr, 0, true)).WillByDefault(Return(Status::Ok));
    ON_CALL(bus, write(0x68, nullptr, 0, true)).WillByDefault(Return(Status::Ok));
    EXPECT_CALL(bus, write(_, nullptr, 0, true)).Times(0x77 - 0x08 + 1);
    EXPECT_EQ(i2cScan(bus), V({0x3C, 0x68}));
}

// =========================================================================
// Eeprom24 -- behavioural tests against a simulated 24LC256
// =========================================================================
class FakeEeprom : public II2cBus {
public:
    FakeEeprom(uint8_t addr, size_t size, size_t page, int busyPolls)
        : addr_(addr), mem(size, 0xFF), page_(page), busyPolls_(busyPolls) {}

    Status write(uint8_t a, const uint8_t* d, size_t len, bool) override {
        if (a != addr_) return Status::Nack;
        if (busy_ > 0) { --busy_; ++nackedPolls; return Status::Nack; }
        if (len == 0) return Status::Ok;           // ACK poll succeeded
        if (len == 1) return Status::ProtocolError;
        ptr_ = ((d[0] << 8) | d[1]) % mem.size();
        if (len > 2) {
            // Real devices wrap inside the current page if you overrun it.
            const size_t base = ptr_ - ptr_ % page_;
            for (size_t i = 2; i < len; ++i) mem[base + (ptr_ - base + i - 2) % page_] = d[i];
            ++pageWrites;
            busy_ = stuckBusy ? 1'000'000 : busyPolls_;
        }
        return Status::Ok;
    }
    Status read(uint8_t a, uint8_t* d, size_t len) override {
        if (a != addr_ || busy_ > 0) return Status::Nack;
        for (size_t i = 0; i < len; ++i) d[i] = mem[(ptr_ + i) % mem.size()];
        ptr_ = (ptr_ + len) % mem.size();
        return Status::Ok;
    }

    uint8_t addr_;
    std::vector<uint8_t> mem;
    size_t page_;
    int busyPolls_;
    int busy_ = 0;
    size_t ptr_ = 0;
    int pageWrites = 0;
    int nackedPolls = 0;
    bool stuckBusy = false;
};

class Eeprom24Test : public ::testing::Test {
protected:
    FakeEeprom chip{0x50, 32768, 64, 3};
    NiceMock<MockDelay> delay;
    Eeprom24 eeprom{chip, delay, Eeprom24::Config{0x50, 32768, 64, 20}};
};

TEST_F(Eeprom24Test, WriteThenReadBackAcrossPageBoundaries) {
    std::vector<uint8_t> data(200);
    std::iota(data.begin(), data.end(), 0);
    const uint32_t start = 50;  // not page aligned: 14 + 64 + 64 + 58 bytes
    ASSERT_EQ(eeprom.write(start, data.data(), data.size()), Status::Ok);
    EXPECT_EQ(chip.pageWrites, 4);

    std::vector<uint8_t> back(data.size());
    ASSERT_EQ(eeprom.read(start, back.data(), back.size()), Status::Ok);
    EXPECT_EQ(back, data);
    EXPECT_EQ(chip.mem[start - 1], 0xFF);  // neighbours untouched
    EXPECT_EQ(chip.mem[start + data.size()], 0xFF);
}

TEST_F(Eeprom24Test, AckPollsWhileBusyAndDelaysBetweenPolls) {
    EXPECT_CALL(delay, delayMs(1)).Times(3);
    const uint8_t b = 0x42;
    EXPECT_EQ(eeprom.write(0, &b, 1), Status::Ok);
    EXPECT_EQ(chip.nackedPolls, 3);
}

TEST_F(Eeprom24Test, TimesOutWhenDeviceNeverFinishes) {
    chip.stuckBusy = true;
    const uint8_t b = 0x42;
    EXPECT_EQ(eeprom.write(0, &b, 1), Status::Timeout);
}

TEST_F(Eeprom24Test, RejectsOutOfRange) {
    uint8_t buf[4] = {};
    EXPECT_EQ(eeprom.read(32766, buf, 4), Status::InvalidArg);
    EXPECT_EQ(eeprom.write(32766, buf, 4), Status::InvalidArg);
}

TEST(Eeprom24, WrongAddressReportsNack) {
    FakeEeprom chip(0x50, 32768, 64, 0);
    NiceMock<MockDelay> delay;
    Eeprom24 eeprom(chip, delay, Eeprom24::Config{0x51, 32768, 64, 5});
    uint8_t b;
    EXPECT_EQ(eeprom.read(0, &b, 1), Status::Nack);
}
