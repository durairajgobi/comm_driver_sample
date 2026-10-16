#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks.hpp"
#include "proto/can.hpp"

using namespace proto;
using namespace proto::test;
using ::testing::_;
using ::testing::DoAll;
using ::testing::Field;
using ::testing::Return;
using ::testing::SetArgReferee;

// ------------------------------------------------------------ frames ---
TEST(CanFrame, ValidatesIdRangeAndDlc) {
    CanFrame f;
    f.id = 0x7FF;
    EXPECT_TRUE(canIsValid(f));
    f.id = 0x800;
    EXPECT_FALSE(canIsValid(f));
    f.extended = true;
    EXPECT_TRUE(canIsValid(f));
    f.id = 0x20000000;
    EXPECT_FALSE(canIsValid(f));
    f.id = 0x100;
    f.dlc = 9;
    EXPECT_FALSE(canIsValid(f));
}

TEST(CanFrame, MakeFrameClampsDlc) {
    const uint8_t d[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    const CanFrame f = canMakeFrame(0x123, d, 10);
    EXPECT_EQ(f.dlc, 8);
    EXPECT_EQ(f.data[7], 8);
}

TEST(CanFilter, MaskSelectsIdRange) {
    const CanFilter f{0x100, 0x7F0, false};  // accepts 0x100..0x10F
    EXPECT_TRUE(f.matches(canMakeFrame(0x105, nullptr, 0)));
    EXPECT_FALSE(f.matches(canMakeFrame(0x110, nullptr, 0)));
    EXPECT_FALSE(f.matches(canMakeFrame(0x105, nullptr, 0, true)));  // wrong ID type
}

// ----------------------------------------------------- signal codec ---
TEST(CanSignal, IntelExtractAcrossBytes) {
    const uint8_t d[8] = {0x34, 0x12};
    const CanFrame f = canMakeFrame(1, d, 8);
    EXPECT_EQ(canExtractRaw(f, {0, 16, ByteOrder::Intel}), 0x1234u);
    EXPECT_EQ(canExtractRaw(f, {4, 8, ByteOrder::Intel}), 0x23u);
}

TEST(CanSignal, MotorolaExtractUsesSawtoothNumbering) {
    const uint8_t d[8] = {0x12, 0x34};
    const CanFrame f = canMakeFrame(1, d, 8);
    EXPECT_EQ(canExtractRaw(f, {7, 16, ByteOrder::Motorola}), 0x1234u);  // DBC: 7|16@0+
    EXPECT_EQ(canExtractRaw(f, {3, 8, ByteOrder::Motorola}), 0x23u);     // nibble-straddling
}

TEST(CanSignal, EngineSpeedExampleWithFactor) {
    // J1939 EEC1 engine speed: start 24, 16 bit, Intel, 0.125 rpm/bit
    const CanSignal rpm{24, 16, ByteOrder::Intel, false, 0.125, 0.0};
    CanFrame f;
    f.dlc = 8;
    ASSERT_TRUE(canEncode(f, rpm, 1500.0));
    EXPECT_EQ(f.data[3], 0xE0);  // 12000 = 0x2EE0
    EXPECT_EQ(f.data[4], 0x2E);
    EXPECT_DOUBLE_EQ(canDecode(f, rpm), 1500.0);
}

TEST(CanSignal, SignedWithOffsetRoundTrip) {
    const CanSignal temp{8, 12, ByteOrder::Motorola, true, 0.1, -20.0};
    CanFrame f;
    for (double v : {-224.8, -20.0, 0.0, 36.6, 184.7}) {
        ASSERT_TRUE(canEncode(f, temp, v)) << v;
        EXPECT_NEAR(canDecode(f, temp), v, 1e-9) << v;
    }
}

TEST(CanSignal, EncodeRejectsOutOfRangeAndLeavesFrameUntouched) {
    const CanSignal s{0, 8, ByteOrder::Intel, false, 1.0, 0.0};
    CanFrame f;
    f.data[0] = 0x55;
    EXPECT_FALSE(canEncode(f, s, 256));
    EXPECT_FALSE(canEncode(f, s, -1));
    EXPECT_EQ(f.data[0], 0x55);
    const CanSignal ss{0, 8, ByteOrder::Intel, true, 1.0, 0.0};
    EXPECT_TRUE(canEncode(f, ss, -128));
    EXPECT_FALSE(canEncode(f, ss, 128));
}

TEST(CanSignal, InsertDoesNotDisturbNeighbourSignals) {
    CanFrame f;
    f.data.fill(0xFF);
    canInsertRaw(f, {10, 5, ByteOrder::Intel}, 0);
    EXPECT_EQ(f.data[0], 0xFF);
    EXPECT_EQ(f.data[1], 0x83);  // bits 10..14 cleared
    EXPECT_EQ(f.data[2], 0xFF);
}

TEST(CanSignal, SixtyFourBitSignal) {
    CanFrame f;
    const CanSignal s{0, 64, ByteOrder::Intel};
    canInsertRaw(f, s, 0x0123456789ABCDEFull);
    EXPECT_EQ(canExtractRaw(f, s), 0x0123456789ABCDEFull);
    EXPECT_EQ(f.data[0], 0xEF);
}

// ------------------------------------------------------- bit timing ---
struct TimingCase { uint32_t clock, bitrate; double sp; };
class CanBitTimingTest : public ::testing::TestWithParam<TimingCase> {};

TEST_P(CanBitTimingTest, ProducesExactBitrateWithinConstraints) {
    const auto& c = GetParam();
    CanBitTiming t;
    ASSERT_TRUE(canComputeBitTiming(c.clock, c.bitrate, c.sp, t));
    const uint32_t tq = 1u + t.tseg1 + t.tseg2;
    EXPECT_EQ(c.clock / (t.prescaler * tq), c.bitrate);
    EXPECT_EQ(t.actualBitrate, c.bitrate);
    EXPECT_GE(tq, 8u);
    EXPECT_LE(tq, 25u);
    EXPECT_LE(t.tseg1, 16);
    EXPECT_LE(t.tseg2, 8);
    EXPECT_LE(t.sjw, t.tseg2);
    EXPECT_NEAR(t.samplePoint, c.sp, 0.03);
}

INSTANTIATE_TEST_SUITE_P(
    CommonClocks, CanBitTimingTest,
    ::testing::Values(TimingCase{36'000'000, 500'000, 0.875},   // STM32F1 APB1
                      TimingCase{42'000'000, 500'000, 0.875},   // STM32F4 APB1
                      TimingCase{80'000'000, 1'000'000, 0.80},  // ESP32 TWAI
                      TimingCase{16'000'000, 250'000, 0.875},
                      TimingCase{8'000'000, 125'000, 0.875}));

TEST(CanBitTiming, ImpossibleCombinationFails) {
    CanBitTiming t;
    EXPECT_FALSE(canComputeBitTiming(1'000'000, 1'000'000, 0.875, t));  // needs >= 8 TQ
    EXPECT_FALSE(canComputeBitTiming(36'000'000, 500'000, 0.3, t));     // bad sample point
}

// ------------------------------------------------------------ CanNode ---
TEST(CanNode, SendRejectsInvalidFrameWithoutTouchingHardware) {
    MockCanController ctrl;
    CanNode node(ctrl);
    EXPECT_CALL(ctrl, send(_)).Times(0);
    EXPECT_EQ(node.send(canMakeFrame(0x900, nullptr, 0)), Status::InvalidArg);
}

TEST(CanNode, SendForwardsValidFrame) {
    MockCanController ctrl;
    CanNode node(ctrl);
    EXPECT_CALL(ctrl, send(Field(&CanFrame::id, 0x321))).WillOnce(Return(Status::Ok));
    EXPECT_EQ(node.send(canMakeFrame(0x321, nullptr, 0)), Status::Ok);
}

TEST(CanNode, ReceiveSkipsFramesRejectedByFilters) {
    MockCanController ctrl;
    CanNode node(ctrl);
    node.addFilter({0x7E8, 0x7FF, false});
    EXPECT_CALL(ctrl, receive(_))
        .WillOnce(DoAll(SetArgReferee<0>(canMakeFrame(0x100, nullptr, 0)), Return(true)))
        .WillOnce(DoAll(SetArgReferee<0>(canMakeFrame(0x7E8, nullptr, 0)), Return(true)));
    CanFrame got;
    ASSERT_TRUE(node.receive(got));
    EXPECT_EQ(got.id, 0x7E8u);
}

TEST(CanNode, ReceiveReturnsFalseWhenQueueDrained) {
    MockCanController ctrl;
    CanNode node(ctrl);
    node.addFilter({0x7E8, 0x7FF, false});
    EXPECT_CALL(ctrl, receive(_))
        .WillOnce(DoAll(SetArgReferee<0>(canMakeFrame(0x100, nullptr, 0)), Return(true)))
        .WillOnce(Return(false));
    CanFrame got;
    EXPECT_FALSE(node.receive(got));
}
