#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <numeric>

#include "mocks.hpp"
#include "proto/spi.hpp"

using namespace proto;
using namespace proto::test;
using ::testing::_;
using ::testing::InSequence;
using ::testing::NiceMock;
using ::testing::Return;

// =========================================================================
// SoftSpi -- verified against a cycle-level simulated SPI slave, all 4 modes
// =========================================================================
class SimSpiSlave : public ISpiPins {
public:
    SimSpiSlave(SpiMode m, std::vector<uint8_t> tx)
        : cpol_(spiCpol(m)), cpha_(spiCpha(m)), tx_(std::move(tx)), sck_(cpol_) {}

    void setCs(bool active) override {
        cs_ = active;
        if (active) {
            outPos_ = 0;
            bits_ = 0;
            if (!cpha_) present(outPos_);  // CPHA=0: first bit valid on CS assert
        }
    }
    void setSck(bool level) override {
        if (level == sck_) return;
        const bool leading = (sck_ == cpol_);
        sck_ = level;
        ++edges;
        if (!cs_) return;
        const bool sampleEdge = cpha_ ? !leading : leading;
        if (sampleEdge) {
            cur_ = static_cast<uint8_t>((cur_ << 1) | (mosi_ ? 1 : 0));
            if (++bits_ % 8 == 0) { rx.push_back(cur_); cur_ = 0; }
        } else if (cpha_) {
            present(outPos_++);  // CPHA=1: shift out on leading edge
        } else {
            present(++outPos_);  // CPHA=0: shift out on trailing edge
        }
    }
    void setMosi(bool l) override { mosi_ = l; }
    bool getMiso() override { return cs_ ? miso_ : true; }

    bool sckLevel() const { return sck_; }
    std::vector<uint8_t> rx;
    int edges = 0;

private:
    void present(size_t pos) {
        const size_t byte = pos / 8;
        miso_ = byte < tx_.size() ? ((tx_[byte] >> (7 - pos % 8)) & 1) : true;
    }
    bool cpol_, cpha_;
    std::vector<uint8_t> tx_;
    bool sck_;
    bool cs_ = false, mosi_ = false, miso_ = true;
    size_t outPos_ = 0;
    int bits_ = 0;
    uint8_t cur_ = 0;
};

class SoftSpiModeTest : public ::testing::TestWithParam<SpiMode> {};

TEST_P(SoftSpiModeTest, FullDuplexExchangeMatchesSlave) {
    const std::vector<uint8_t> masterTx = {0x9F, 0x3C, 0xA5, 0x00, 0xFF};
    const std::vector<uint8_t> slaveTx = {0x5A, 0xC3, 0x01, 0x80, 0x7E};
    SimSpiSlave slave(GetParam(), slaveTx);
    SoftSpi spi(slave, GetParam());

    std::vector<uint8_t> rx(masterTx.size());
    spi.select(true);
    ASSERT_EQ(spi.transfer(masterTx.data(), rx.data(), rx.size()), Status::Ok);
    spi.select(false);

    EXPECT_EQ(rx, slaveTx);
    EXPECT_EQ(slave.rx, masterTx);
    EXPECT_EQ(slave.sckLevel(), spiCpol(GetParam())) << "clock must return to idle (CPOL)";
    EXPECT_EQ(slave.edges, 2 * 8 * static_cast<int>(masterTx.size()));
}

INSTANTIATE_TEST_SUITE_P(AllModes, SoftSpiModeTest,
                         ::testing::Values(SpiMode::Mode0, SpiMode::Mode1, SpiMode::Mode2,
                                           SpiMode::Mode3),
                         [](const auto& info) {
                             return "Mode" + std::to_string(static_cast<int>(info.param));
                         });

TEST(SoftSpi, LsbFirstReversesBitOrderOnTheWire) {
    SimSpiSlave slave(SpiMode::Mode0, {0x80});  // slave sends MSB-first 1000'0000
    SoftSpi spi(slave, SpiMode::Mode0, BitOrder::LsbFirst);
    spi.select(true);
    const uint8_t got = spi.transferByte(0x01);
    spi.select(false);
    EXPECT_EQ(slave.rx, V({0x80}));  // 0x01 LSB-first == 0x80 MSB-first
    EXPECT_EQ(got, 0x01);
}

TEST(SoftSpi, NullTxSendsOnesNullRxDiscards) {
    SimSpiSlave slave(SpiMode::Mode3, {0x12, 0x34});
    SoftSpi spi(slave, SpiMode::Mode3);
    spi.select(true);
    EXPECT_EQ(spi.transfer(nullptr, nullptr, 2), Status::Ok);
    spi.select(false);
    EXPECT_EQ(slave.rx, V({0xFF, 0xFF}));
}

// =========================================================================
// SpiFlash -- behavioural tests against a simulated W25Q-style NOR flash
// =========================================================================
class FakeSpiFlash : public ISpiBus {
public:
    explicit FakeSpiFlash(size_t size = 65536, int busyReads = 2) : mem(size, 0xFF), busyReads_(busyReads) {}

    void select(bool active) override {
        if (active) { cs_ = true; pos_ = 0; addr_ = 0; return; }
        if (cs_ && pos_ > 0) {
            if ((cmd_ == 0x02 || cmd_ == 0x20) && wel_) {
                if (cmd_ == 0x20) {
                    const size_t base = addr_ & ~size_t{0xFFF};
                    std::fill(mem.begin() + base, mem.begin() + base + 4096, 0xFF);
                    ++erases;
                } else {
                    ++programs;
                }
                busy_ = busyReads_;
                wel_ = false;
            }
            if (cmd_ == 0x06 && !writeProtected) wel_ = true;
        }
        cs_ = false;
    }

    Status transfer(const uint8_t* tx, uint8_t* rx, size_t len) override {
        if (!cs_) { ++transfersWithoutCs; return Status::BusError; }
        for (size_t i = 0; i < len; ++i) {
            const uint8_t out = clock(tx ? tx[i] : 0xFF);
            if (rx) rx[i] = out;
        }
        return Status::Ok;
    }

    std::vector<uint8_t> mem;
    int programs = 0, erases = 0, transfersWithoutCs = 0;
    bool writeProtected = false;
    uint8_t id[3] = {0xEF, 0x40, 0x16};  // Winbond W25Q32

private:
    uint8_t clock(uint8_t in) {
        const size_t p = pos_++;
        if (p == 0) { cmd_ = in; return 0xFF; }
        switch (cmd_) {
            case 0x9F: return p <= 3 ? id[p - 1] : 0xFF;
            case 0x05: {
                const uint8_t st = static_cast<uint8_t>((busy_ > 0 ? 1 : 0) | (wel_ ? 2 : 0));
                if (busy_ > 0) --busy_;
                return st;
            }
            case 0x03: case 0x02: case 0x20:
                if (p <= 3) { addr_ = (addr_ << 8) | in; return 0xFF; }
                if (cmd_ == 0x03) return mem[(addr_ + (p - 4)) % mem.size()];
                if (cmd_ == 0x02 && wel_) {
                    const size_t base = addr_ & ~size_t{0xFF};
                    mem[base + ((addr_ + (p - 4)) & 0xFF)] &= in;  // NOR: 1 -> 0 only, wraps in page
                }
                return 0xFF;
            default: return 0xFF;
        }
    }
    bool cs_ = false, wel_ = false;
    uint8_t cmd_ = 0;
    size_t pos_ = 0, addr_ = 0;
    int busy_ = 0, busyReads_;
};

class SpiFlashTest : public ::testing::Test {
protected:
    FakeSpiFlash chip;
    NiceMock<MockDelay> delay;
    SpiFlash flash{chip, delay, SpiFlash::Config{256, 100, 65536}};
};

TEST_F(SpiFlashTest, ReadsJedecId) {
    JedecId id;
    ASSERT_EQ(flash.readJedecId(id), Status::Ok);
    EXPECT_EQ(id.manufacturer, 0xEF);
    EXPECT_EQ(id.memoryType, 0x40);
    EXPECT_EQ(id.capacityBytes(), 4u * 1024 * 1024);
}

TEST_F(SpiFlashTest, MissingChipDetectedFromFloatingMiso) {
    chip.id[0] = chip.id[1] = chip.id[2] = 0xFF;
    JedecId id;
    EXPECT_EQ(flash.readJedecId(id), Status::BusError);
}

TEST_F(SpiFlashTest, ProgramSplitsAtPageBoundaries) {
    std::vector<uint8_t> data(600);
    std::iota(data.begin(), data.end(), 0);
    ASSERT_EQ(flash.write(0x1F0, data.data(), data.size()), Status::Ok);
    EXPECT_EQ(chip.programs, 4);  // 16 + 256 + 256 + 72

    std::vector<uint8_t> back(data.size());
    ASSERT_EQ(flash.read(0x1F0, back.data(), back.size()), Status::Ok);
    EXPECT_EQ(back, data);
    EXPECT_EQ(chip.transfersWithoutCs, 0);
}

TEST_F(SpiFlashTest, EraseSectorRestoresOnesAndAligns) {
    const uint8_t z[4] = {0, 0, 0, 0};
    ASSERT_EQ(flash.write(0x2010, z, 4), Status::Ok);
    ASSERT_EQ(flash.eraseSector(0x2FFF), Status::Ok);  // any address inside the sector
    EXPECT_EQ(chip.erases, 1);
    EXPECT_EQ(chip.mem[0x2010], 0xFF);
}

TEST_F(SpiFlashTest, WaitsForBusyFlagWithDelays) {
    EXPECT_CALL(delay, delayMs(1)).Times(2);  // fake reports busy for 2 status reads
    const uint8_t b = 0x00;
    EXPECT_EQ(flash.write(0, &b, 1), Status::Ok);
}

TEST_F(SpiFlashTest, WriteProtectedChipFailsWriteEnable) {
    chip.writeProtected = true;
    const uint8_t b = 0x00;
    EXPECT_EQ(flash.write(0, &b, 1), Status::ProtocolError);
    EXPECT_EQ(chip.mem[0], 0xFF);
}

TEST_F(SpiFlashTest, RejectsOutOfRange) {
    uint8_t b[2];
    EXPECT_EQ(flash.read(65535, b, 2), Status::InvalidArg);
}

TEST(SpiFlash, ChipSelectReleasedEvenWhenTransferFails) {
    MockSpiBus bus;
    NiceMock<MockDelay> delay;
    SpiFlash flash(bus, delay);
    InSequence seq;
    EXPECT_CALL(bus, select(true));
    EXPECT_CALL(bus, transfer(_, _, _)).WillOnce(Return(Status::BusError));
    EXPECT_CALL(bus, select(false));
    uint8_t b;
    EXPECT_EQ(flash.read(0, &b, 1), Status::BusError);
}

TEST(SpiFlash, ReadCommandHeaderIsOpcodePlus24BitAddress) {
    MockSpiBus bus;
    NiceMock<MockDelay> delay;
    SpiFlash flash(bus, delay);
    EXPECT_CALL(bus, select(_)).Times(2);
    EXPECT_CALL(bus, transfer(_, nullptr, 4)).With(SpiTxBytes(V({0x03, 0x12, 0x34, 0x56})))
        .WillOnce(Return(Status::Ok));
    EXPECT_CALL(bus, transfer(nullptr, _, 8)).WillOnce(Return(Status::Ok));
    uint8_t buf[8];
    EXPECT_EQ(flash.read(0x123456, buf, 8), Status::Ok);
}
