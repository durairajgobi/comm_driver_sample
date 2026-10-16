#include <gtest/gtest.h>

#include "proto/lin.hpp"

using namespace proto;

// Reference PIDs from the LIN 2.2A specification table.
struct PidCase { uint8_t id, pid; };
class LinPidTest : public ::testing::TestWithParam<PidCase> {};

TEST_P(LinPidTest, MatchesSpecTable) {
    EXPECT_EQ(linProtectedId(GetParam().id), GetParam().pid);
}

INSTANTIATE_TEST_SUITE_P(SpecTable, LinPidTest,
                         ::testing::Values(PidCase{0x00, 0x80}, PidCase{0x01, 0xC1},
                                           PidCase{0x10, 0x50}, PidCase{0x20, 0x20},
                                           PidCase{0x3C, 0x3C}, PidCase{0x3D, 0x7D},
                                           PidCase{0x3F, 0xBF}));

TEST(LinPid, AllSixtyFourIdsRoundTripAndAllOtherPidsRejected) {
    int valid = 0;
    for (int pid = 0; pid < 256; ++pid) {
        uint8_t id = 0xFF;
        if (linParsePid(static_cast<uint8_t>(pid), id)) {
            ++valid;
            EXPECT_EQ(linProtectedId(id), pid);
        }
    }
    EXPECT_EQ(valid, 64);
}

TEST(LinPid, UpperBitsOfIdIgnored) { EXPECT_EQ(linProtectedId(0xC1), linProtectedId(0x01)); }

TEST(LinPid, DefaultLengths) {
    EXPECT_EQ(linDefaultLength(0x00), 2);
    EXPECT_EQ(linDefaultLength(0x20), 4);
    EXPECT_EQ(linDefaultLength(0x3C), 8);
}

TEST(LinChecksum, ClassicSimpleSum) {
    const uint8_t d[] = {0x01, 0x02, 0x03};
    EXPECT_EQ(linChecksum(0x00, d, 3, LinChecksum::Classic), 0xF9);
}

TEST(LinChecksum, CarryIsWrappedIntoLsb) {
    const uint8_t d[] = {0xFF, 0x01};  // 0x100 -> 0x01 -> ~ = 0xFE
    EXPECT_EQ(linChecksum(0x00, d, 2, LinChecksum::Classic), 0xFE);
    const uint8_t all[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    EXPECT_EQ(linChecksum(0x00, all, 8, LinChecksum::Classic), 0x00);
}

TEST(LinChecksum, EnhancedIncludesPid) {
    const uint8_t d[] = {0x10, 0x20};
    const uint8_t pid = linProtectedId(0x10);
    EXPECT_NE(linChecksum(pid, d, 2, LinChecksum::Enhanced), linChecksum(pid, d, 2, LinChecksum::Classic));
    // Sum including PID plus checksum must be 0xFF (spec property).
    uint16_t sum = pid;
    for (uint8_t b : d) { sum += b; if (sum > 0xFF) sum -= 0xFF; }
    sum += linChecksum(pid, d, 2, LinChecksum::Enhanced);
    if (sum > 0xFF) sum -= 0xFF;
    EXPECT_EQ(sum, 0xFF);
}

TEST(LinResponse, BuildAndVerifyRoundTrip) {
    const uint8_t data[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t frame[9];
    const size_t n = linBuildResponse(0x22, data, 4, LinChecksum::Enhanced, frame, sizeof(frame));
    ASSERT_EQ(n, 5u);
    EXPECT_EQ(linVerifyResponse(linProtectedId(0x22), frame, n, LinChecksum::Enhanced), Status::Ok);
    frame[1] ^= 0x10;
    EXPECT_EQ(linVerifyResponse(linProtectedId(0x22), frame, n, LinChecksum::Enhanced), Status::CrcError);
}

TEST(LinResponse, DiagnosticFramesForceClassicChecksum) {
    const uint8_t req[8] = {0x7F, 0x06, 0xB2, 0x00, 0xFF, 0x7F, 0xFF, 0xFF};
    uint8_t frame[9];
    ASSERT_EQ(linBuildResponse(kLinMasterReqId, req, 8, LinChecksum::Enhanced, frame, 9), 9u);
    EXPECT_EQ(frame[8], linChecksum(0, req, 8, LinChecksum::Classic));
}

TEST(LinResponse, RejectsBadPidAndLengths) {
    uint8_t frame[3] = {1, 2, 0};
    EXPECT_EQ(linVerifyResponse(0x01, frame, 3, LinChecksum::Classic), Status::ProtocolError);  // bad parity
    uint8_t out[9];
    EXPECT_EQ(linBuildResponse(0x01, frame, 9, LinChecksum::Classic, out, 9), 0u);  // > 8 data bytes
    EXPECT_EQ(linBuildResponse(0x01, frame, 2, LinChecksum::Classic, out, 2), 0u);  // no room for checksum
}
