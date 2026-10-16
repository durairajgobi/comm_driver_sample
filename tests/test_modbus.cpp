#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <deque>
#include <map>

#include "mocks.hpp"
#include "proto/crc.hpp"
#include "proto/modbus.hpp"

using namespace proto;
using namespace proto::test;
using ::testing::_;
using ::testing::Return;

// --------------------------------------------- frames vs. spec examples ---
TEST(ModbusFrames, ReadHoldingRegistersRequest) {
    uint8_t f[8];
    ASSERT_EQ(mbBuildReadRegisters(0x01, kMbReadHoldingRegs, 0x0000, 10, f, sizeof(f)), 8u);
    EXPECT_EQ(std::vector<uint8_t>(f, f + 8), V({0x01, 0x03, 0x00, 0x00, 0x00, 0x0A, 0xC5, 0xCD}));
}

TEST(ModbusFrames, WriteSingleRegisterRequest) {
    uint8_t f[8];
    ASSERT_EQ(mbBuildWriteSingle(0x01, 0x0001, 0x0003, f, sizeof(f)), 8u);
    EXPECT_EQ(std::vector<uint8_t>(f, f + 8), V({0x01, 0x06, 0x00, 0x01, 0x00, 0x03, 0x98, 0x0B}));
}

TEST(ModbusFrames, WriteMultipleRegistersLayout) {
    const uint16_t vals[2] = {0x000A, 0x0102};
    uint8_t f[32];
    const size_t n = mbBuildWriteMultiple(0x11, 0x0001, vals, 2, f, sizeof(f));
    ASSERT_EQ(n, 13u);
    EXPECT_EQ(std::vector<uint8_t>(f, f + 11),
              V({0x11, 0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02}));
    EXPECT_TRUE(mbCheckCrc(f, n));
}

TEST(ModbusFrames, BuildersValidateArguments) {
    uint8_t f[300];
    EXPECT_EQ(mbBuildReadRegisters(0, kMbReadHoldingRegs, 0, 1, f, 300), 0u);    // broadcast read
    EXPECT_EQ(mbBuildReadRegisters(248, kMbReadHoldingRegs, 0, 1, f, 300), 0u);  // reserved address
    EXPECT_EQ(mbBuildReadRegisters(1, kMbReadHoldingRegs, 0, 0, f, 300), 0u);
    EXPECT_EQ(mbBuildReadRegisters(1, kMbReadHoldingRegs, 0, 126, f, 300), 0u);
    EXPECT_EQ(mbBuildReadRegisters(1, kMbWriteSingleReg, 0, 1, f, 300), 0u);
    EXPECT_EQ(mbBuildReadRegisters(1, kMbReadHoldingRegs, 0, 1, f, 7), 0u);
    uint16_t v[124] = {};
    EXPECT_EQ(mbBuildWriteMultiple(1, 0, v, 124, f, 300), 0u);
}

TEST(ModbusParse, SpecExampleResponse) {
    // Modbus Application Protocol spec, FC03 example (slave 0x11, 3 registers)
    const auto rsp = V({0x11, 0x03, 0x06, 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40, 0x49, 0xAD});
    uint16_t regs[3];
    uint8_t ex = 0xFF;
    ASSERT_EQ(mbParseReadRegisters(rsp.data(), rsp.size(), 0x11, kMbReadHoldingRegs, regs, 3, ex), Status::Ok);
    EXPECT_EQ(regs[0], 0xAE41);
    EXPECT_EQ(regs[1], 0x5652);
    EXPECT_EQ(regs[2], 0x4340);
    EXPECT_EQ(ex, 0);
}

TEST(ModbusParse, ExceptionResponse) {
    const auto rsp = V({0x01, 0x83, 0x02, 0xC0, 0xF1});  // illegal data address
    uint16_t regs[1];
    uint8_t ex = 0;
    EXPECT_EQ(mbParseReadRegisters(rsp.data(), rsp.size(), 1, kMbReadHoldingRegs, regs, 1, ex),
              Status::DeviceException);
    EXPECT_EQ(ex, 0x02);
}

TEST(ModbusParse, DetectsWrongSlaveCrcAndLength) {
    auto rsp = V({0x11, 0x03, 0x06, 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40, 0x49, 0xAD});
    uint16_t regs[3];
    uint8_t ex;
    EXPECT_EQ(mbParseReadRegisters(rsp.data(), rsp.size(), 0x12, 3, regs, 3, ex), Status::ProtocolError);
    EXPECT_EQ(mbParseReadRegisters(rsp.data(), rsp.size(), 0x11, 3, regs, 2, ex), Status::ProtocolError);
    rsp[4] ^= 0x01;
    EXPECT_EQ(mbParseReadRegisters(rsp.data(), rsp.size(), 0x11, 3, regs, 3, ex), Status::CrcError);
}

// ================================================================
// ModbusRtuMaster against a simulated RS-485 slave
// ================================================================
class FakeModbusSlave : public IUartPort {
public:
    explicit FakeModbusSlave(uint8_t id) : id_(id) {}
    std::map<uint16_t, uint16_t> regs;
    bool silent = false, corruptCrc = false;
    int requests = 0;

    size_t write(const uint8_t* d, size_t n) override {
        ++requests;
        rx_.clear();
        if (silent || !mbCheckCrc(d, n)) return n;
        const uint8_t addr = d[0];
        if (addr != id_ && addr != 0) return n;
        const uint8_t fn = d[1];
        const uint16_t a = static_cast<uint16_t>((d[2] << 8) | d[3]);
        const uint16_t q = static_cast<uint16_t>((d[4] << 8) | d[5]);
        std::vector<uint8_t> out = {id_, fn};
        if (fn == kMbReadHoldingRegs) {
            for (uint16_t i = 0; i < q; ++i) if (!regs.count(a + i)) { exception(fn, 0x02); return n; }
            out.push_back(static_cast<uint8_t>(2 * q));
            for (uint16_t i = 0; i < q; ++i) {
                out.push_back(static_cast<uint8_t>(regs[a + i] >> 8));
                out.push_back(static_cast<uint8_t>(regs[a + i]));
            }
        } else if (fn == kMbWriteSingleReg) {
            if (!regs.count(a)) { exception(fn, 0x02); return n; }
            regs[a] = q;
            out.assign(d, d + 6);
        } else if (fn == kMbWriteMultipleRegs) {
            for (uint16_t i = 0; i < q; ++i) {
                if (!regs.count(a + i)) { exception(fn, 0x02); return n; }
                regs[a + i] = static_cast<uint16_t>((d[7 + 2 * i] << 8) | d[8 + 2 * i]);
            }
            out.assign(d, d + 6);
        } else {
            { exception(fn, 0x01); return n; }
        }
        if (addr != 0) queue(out);
        return n;
    }
    bool readByte(uint8_t& b) override {
        if (rx_.empty()) return false;  // models the inter-frame timeout
        b = rx_.front();
        rx_.pop_front();
        return true;
    }

private:
    void exception(uint8_t fn, uint8_t code) {
        queue({id_, static_cast<uint8_t>(fn | 0x80), code});
    }
    void queue(std::vector<uint8_t> f) {
        const uint16_t crc = crc16Modbus(f.data(), f.size());
        f.push_back(static_cast<uint8_t>(crc & 0xFF));
        f.push_back(static_cast<uint8_t>((crc >> 8) ^ (corruptCrc ? 0x01 : 0)));
        rx_.assign(f.begin(), f.end());
    }
    uint8_t id_;
    std::deque<uint8_t> rx_;
};

class ModbusMasterTest : public ::testing::Test {
protected:
    void SetUp() override {
        for (uint16_t a = 100; a < 110; ++a) slave.regs[a] = static_cast<uint16_t>(a * 3);
    }
    FakeModbusSlave slave{0x07};
    ModbusRtuMaster master{slave};
};

TEST_F(ModbusMasterTest, ReadHoldingRegisters) {
    uint16_t r[4] = {};
    ASSERT_EQ(master.readHoldingRegisters(0x07, 102, 4, r), Status::Ok);
    EXPECT_EQ(r[0], 306);
    EXPECT_EQ(r[3], 315);
}

TEST_F(ModbusMasterTest, WriteSingleThenReadBack) {
    ASSERT_EQ(master.writeSingleRegister(0x07, 105, 0xBEEF), Status::Ok);
    uint16_t r = 0;
    ASSERT_EQ(master.readHoldingRegisters(0x07, 105, 1, &r), Status::Ok);
    EXPECT_EQ(r, 0xBEEF);
}

TEST_F(ModbusMasterTest, WriteMultiple) {
    const uint16_t v[3] = {1, 2, 3};
    ASSERT_EQ(master.writeMultipleRegisters(0x07, 100, v, 3), Status::Ok);
    EXPECT_EQ(slave.regs[100], 1);
    EXPECT_EQ(slave.regs[102], 3);
    EXPECT_EQ(slave.regs[103], 309);
}

TEST_F(ModbusMasterTest, IllegalAddressReturnsException) {
    uint16_t r[2];
    EXPECT_EQ(master.readHoldingRegisters(0x07, 109, 2, r), Status::DeviceException);
    EXPECT_EQ(master.lastException(), 0x02);
    EXPECT_EQ(master.writeSingleRegister(0x07, 999, 1), Status::DeviceException);
}

TEST_F(ModbusMasterTest, NoResponseIsTimeout) {
    uint16_t r;
    EXPECT_EQ(master.readHoldingRegisters(0x08, 100, 1, &r), Status::Timeout);  // wrong slave id
    slave.silent = true;
    EXPECT_EQ(master.writeSingleRegister(0x07, 100, 1), Status::Timeout);
}

TEST_F(ModbusMasterTest, CorruptedResponseIsCrcError) {
    slave.corruptCrc = true;
    uint16_t r;
    EXPECT_EQ(master.readHoldingRegisters(0x07, 100, 1, &r), Status::CrcError);
}

TEST_F(ModbusMasterTest, BroadcastWriteDoesNotWaitForReply) {
    EXPECT_EQ(master.writeSingleRegister(0, 101, 77), Status::Ok);
    EXPECT_EQ(slave.regs[101], 77);
}

TEST(ModbusMaster, TruncatedResponseIsProtocolError) {
    MockUartPort port;
    ModbusRtuMaster master(port);
    EXPECT_CALL(port, write(_, 8)).WillOnce(Return(8));
    uint8_t seq[] = {0x01, 0x03, 0x02};
    int i = 0;
    EXPECT_CALL(port, readByte(_)).WillRepeatedly([&](uint8_t& b) {
        if (i >= 3) return false;
        b = seq[i++];
        return true;
    });
    uint16_t r;
    EXPECT_EQ(master.readHoldingRegisters(1, 0, 1, &r), Status::ProtocolError);
}

TEST(ModbusMaster, UartWriteFailureIsBusError) {
    MockUartPort port;
    ModbusRtuMaster master(port);
    EXPECT_CALL(port, write(_, _)).WillOnce(Return(0));
    EXPECT_CALL(port, readByte(_)).Times(0);
    uint16_t r;
    EXPECT_EQ(master.readHoldingRegisters(1, 0, 1, &r), Status::BusError);
}
