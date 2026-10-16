#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <deque>

#include "mocks.hpp"
#include "proto/crc.hpp"
#include "proto/uart.hpp"

using namespace proto;
using namespace proto::test;
using ::testing::_;
using ::testing::Return;

static std::vector<uint8_t> encode(const std::vector<uint8_t>& p) {
    std::vector<uint8_t> out(p.size() + kUartOverhead);
    out.resize(uartEncodeFrame(p.data(), p.size(), out.data(), out.size()));
    return out;
}

class UartParserTest : public ::testing::Test {
protected:
    std::vector<std::vector<uint8_t>> got;
    UartFrameParser parser{[this](const uint8_t* d, size_t n) { got.emplace_back(d, d + n); }};
};

TEST(UartEncode, LayoutIsSofLenPayloadCrcBigEndian) {
    const auto f = encode(V({0x01, 0x02}));
    ASSERT_EQ(f.size(), 6u);
    EXPECT_EQ(f[0], kUartSof);
    EXPECT_EQ(f[1], 2);
    const uint16_t crc = crc16Ccitt(f.data() + 1, 3);
    EXPECT_EQ(f[4], crc >> 8);
    EXPECT_EQ(f[5], crc & 0xFF);
}

TEST(UartEncode, RejectsSmallBufferAndOversizePayload) {
    uint8_t out[8];
    const uint8_t p[5] = {};
    EXPECT_EQ(uartEncodeFrame(p, 5, out, sizeof(out)), 0u);  // needs 9
    std::vector<uint8_t> big(256), bigOut(300);
    EXPECT_EQ(uartEncodeFrame(big.data(), big.size(), bigOut.data(), bigOut.size()), 0u);
}

TEST_F(UartParserTest, RoundTripsIncludingEmptyAndMaxPayload) {
    std::vector<uint8_t> max(kUartMaxPayload);
    for (size_t i = 0; i < max.size(); ++i) max[i] = static_cast<uint8_t>(i * 7);
    for (const auto& p : {V({}), V({0xA5, 0xA5}), max}) {
        const auto f = encode(p);
        parser.feed(f.data(), f.size());
    }
    ASSERT_EQ(got.size(), 3u);
    EXPECT_TRUE(got[0].empty());
    EXPECT_EQ(got[1], V({0xA5, 0xA5}));  // SOF value inside payload is fine
    EXPECT_EQ(got[2], max);
    EXPECT_EQ(parser.stats().framesOk, 3u);
}

TEST_F(UartParserTest, SkipsLeadingNoise) {
    const auto f = encode(V({0x10, 0x20}));
    parser.feed(V({0x00, 0x13, 0x37}).data(), 3);
    parser.feed(f.data(), f.size());
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(parser.stats().bytesDiscarded, 3u);
}

TEST_F(UartParserTest, CorruptedFrameDroppedAndNextFrameStillReceived) {
    auto bad = encode(V({1, 2, 3}));
    bad[3] ^= 0x01;
    const auto good = encode(V({4, 5, 6}));
    parser.feed(bad.data(), bad.size());
    parser.feed(good.data(), good.size());
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0], V({4, 5, 6}));
    EXPECT_EQ(parser.stats().crcErrors, 1u);
}

TEST_F(UartParserTest, ByteAtATimeFeedingWorks) {
    const auto f = encode(V({9, 8, 7, 6}));
    for (uint8_t b : f) parser.feed(b);
    ASSERT_EQ(got.size(), 1u);
}

TEST_F(UartParserTest, EverySingleBitErrorIsRejected) {
    const auto ref = encode(V({0xDE, 0xAD, 0xBE, 0xEF}));
    for (size_t byte = 2; byte < ref.size(); ++byte) {  // payload + CRC bytes
        for (int bit = 0; bit < 8; ++bit) {
            auto f = ref;
            f[byte] ^= static_cast<uint8_t>(1 << bit);
            parser.reset();
            parser.feed(f.data(), f.size());
        }
    }
    EXPECT_TRUE(got.empty());
}

// ------------------------------------------------------------ UartLink ---
TEST(UartLink, SendWritesEncodedFrame) {
    MockUartPort port;
    UartLink link(port, nullptr);
    const auto expected = encode(V({0x42}));
    EXPECT_CALL(port, write(_, _)).With(UartBytes(expected)).WillOnce(Return(expected.size()));
    EXPECT_EQ(link.send(V({0x42}).data(), 1), Status::Ok);
}

TEST(UartLink, ShortWriteIsBusError) {
    MockUartPort port;
    UartLink link(port, nullptr);
    EXPECT_CALL(port, write(_, _)).WillOnce(Return(2));
    const uint8_t p = 0;
    EXPECT_EQ(link.send(&p, 1), Status::BusError);
}

class FakeUart : public IUartPort {
public:
    size_t write(const uint8_t*, size_t len) override { return len; }
    bool readByte(uint8_t& b) override {
        if (rx.empty()) return false;
        b = rx.front();
        rx.pop_front();
        return true;
    }
    std::deque<uint8_t> rx;
};

TEST(UartLink, PollDeliversFramesFromPort) {
    FakeUart port;
    int frames = 0;
    UartLink link(port, [&](const uint8_t*, size_t n) { ++frames; EXPECT_EQ(n, 2u); });
    for (int i = 0; i < 3; ++i)
        for (uint8_t b : encode(V({1, 2}))) port.rx.push_back(b);
    EXPECT_EQ(link.poll(), 18u);
    EXPECT_EQ(frames, 3);
}
