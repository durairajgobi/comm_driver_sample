#include <gtest/gtest.h>

#include <numeric>

#include "mocks.hpp"
#include "proto/isotp.hpp"

using namespace proto;
using namespace proto::test;
using State = IsoTpSender::State;
using Result = IsoTpReceiver::Result;

static std::vector<uint8_t> pattern(size_t n) {
    std::vector<uint8_t> v(n);
    std::iota(v.begin(), v.end(), 0);
    return v;
}

TEST(IsoTpSender, ShortPayloadIsSingleFrame) {
    IsoTpSender tx(0x7E0);
    const auto req = V({0x22, 0xF1, 0x90});  // UDS ReadDataByIdentifier VIN
    ASSERT_EQ(tx.start(req.data(), req.size()), Status::Ok);
    CanFrame f;
    ASSERT_TRUE(tx.firstFrame(f));
    EXPECT_EQ(f.id, 0x7E0u);
    EXPECT_EQ(f.dlc, 8);
    EXPECT_EQ(f.data[0], 0x03);
    EXPECT_EQ(f.data[1], 0x22);
    EXPECT_EQ(f.data[4], kIsoTpPadding);
    EXPECT_EQ(tx.state(), State::Done);
}

TEST(IsoTpSender, LongPayloadUsesFirstFrameAndWaitsForFlowControl) {
    IsoTpSender tx(0x7E8);
    const auto data = pattern(20);
    ASSERT_EQ(tx.start(data.data(), data.size()), Status::Ok);
    CanFrame f;
    ASSERT_TRUE(tx.firstFrame(f));
    EXPECT_EQ(f.data[0], 0x10);
    EXPECT_EQ(f.data[1], 20);
    EXPECT_EQ(f.data[2], 0);
    EXPECT_EQ(f.data[7], 5);
    EXPECT_EQ(tx.state(), State::WaitFlowControl);
    EXPECT_FALSE(tx.nextFrame(f)) << "must not send CF before FC";

    CanFrame fc;
    fc.dlc = 3;
    fc.data[0] = 0x30; fc.data[1] = 0; fc.data[2] = 10;
    ASSERT_EQ(tx.onFlowControl(fc), Status::Ok);
    EXPECT_EQ(tx.stMin(), 10);

    ASSERT_TRUE(tx.nextFrame(f));
    EXPECT_EQ(f.data[0], 0x21);
    EXPECT_EQ(f.data[1], 6);
    ASSERT_TRUE(tx.nextFrame(f));
    EXPECT_EQ(f.data[0], 0x22);
    EXPECT_EQ(f.data[1], 13);
    EXPECT_EQ(f.data[7], 19);
    EXPECT_EQ(tx.state(), State::Done);
}

TEST(IsoTpSender, FlowControlWaitAndOverflow) {
    IsoTpSender tx(1);
    const auto data = pattern(30);
    tx.start(data.data(), data.size());
    CanFrame f, fc;
    tx.firstFrame(f);
    fc.dlc = 3;
    fc.data[0] = 0x31;  // WAIT
    EXPECT_EQ(tx.onFlowControl(fc), Status::Ok);
    EXPECT_EQ(tx.state(), State::WaitFlowControl);
    fc.data[0] = 0x32;  // OVERFLOW
    EXPECT_EQ(tx.onFlowControl(fc), Status::Overflow);
    EXPECT_EQ(tx.state(), State::Error);
}

TEST(IsoTpSender, RejectsBadArguments) {
    IsoTpSender tx(1);
    std::vector<uint8_t> big(kIsoTpMaxPayload + 1);
    EXPECT_EQ(tx.start(big.data(), big.size()), Status::Overflow);
    EXPECT_EQ(tx.start(big.data(), 0), Status::InvalidArg);
    tx.start(big.data(), 100);
    CanFrame f;
    tx.firstFrame(f);
    EXPECT_EQ(tx.start(big.data(), 5), Status::Busy);
}

TEST(IsoTpReceiver, SingleFrameCompletesImmediately) {
    IsoTpReceiver rx(0x7E0);
    CanFrame f, fc;
    f.dlc = 8;
    f.data = {0x02, 0x10, 0x03, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};
    EXPECT_EQ(rx.onFrame(f, fc), Result::Complete);
    EXPECT_EQ(rx.payload(), V({0x10, 0x03}));
}

TEST(IsoTpReceiver, WrongSequenceNumberIsError) {
    IsoTpReceiver rx(1);
    CanFrame ff, cf, fc;
    ff.dlc = 8;
    ff.data = {0x10, 0x14, 0, 1, 2, 3, 4, 5};
    ASSERT_EQ(rx.onFrame(ff, fc), Result::SendFlowControl);
    EXPECT_EQ(fc.data[0], 0x30);
    cf.dlc = 8;
    cf.data = {0x22, 6, 7, 8, 9, 10, 11, 12};  // expected SN 1
    EXPECT_EQ(rx.onFrame(cf, fc), Result::Error);
}

TEST(IsoTpReceiver, StrayConsecutiveFrameIgnored) {
    IsoTpReceiver rx(1);
    CanFrame cf, fc;
    cf.dlc = 8;
    cf.data[0] = 0x21;
    EXPECT_EQ(rx.onFrame(cf, fc), Result::None);
}

// End-to-end: sender <-> receiver over a simulated bus, with block size.
class IsoTpLoopback : public ::testing::TestWithParam<std::tuple<size_t, uint8_t>> {};

TEST_P(IsoTpLoopback, TransfersPayloadIntact) {
    const auto [len, blockSize] = GetParam();
    const auto data = pattern(len);
    IsoTpSender tx(0x7E8);
    IsoTpReceiver rx(0x7E0, blockSize, 0);

    ASSERT_EQ(tx.start(data.data(), data.size()), Status::Ok);
    CanFrame f, fc;
    ASSERT_TRUE(tx.firstFrame(f));
    int fcCount = 0, frames = 1;
    Result r = rx.onFrame(f, fc);
    while (r != Result::Complete) {
        ASSERT_NE(r, Result::Error);
        if (r == Result::SendFlowControl) {
            ++fcCount;
            ASSERT_EQ(tx.onFlowControl(fc), Status::Ok);
        }
        ASSERT_TRUE(tx.nextFrame(f));
        ++frames;
        r = rx.onFrame(f, fc);
    }
    EXPECT_EQ(rx.payload(), data);
    EXPECT_EQ(tx.state(), State::Done);

    const size_t expectedFrames = len <= 7 ? 1 : 1 + (len - 6 + 6) / 7;
    EXPECT_EQ(static_cast<size_t>(frames), expectedFrames);
    if (len > 7) {
        const size_t cfs = expectedFrames - 1;
        const size_t expectFc = blockSize == 0 ? 1 : 1 + (cfs - 1) / blockSize;
        EXPECT_EQ(static_cast<size_t>(fcCount), expectFc);
    }
}

INSTANTIATE_TEST_SUITE_P(Sizes, IsoTpLoopback,
                         ::testing::Combine(::testing::Values(1, 7, 8, 13, 14, 62, 111, 200, 4095),
                                            ::testing::Values(0, 1, 4)));
