#include "proto/lin.hpp"

#include <algorithm>

namespace proto {

uint8_t linProtectedId(uint8_t id) {
    id &= 0x3F;
    auto bit = [id](int n) { return (id >> n) & 1; };
    const uint8_t p0 = static_cast<uint8_t>(bit(0) ^ bit(1) ^ bit(2) ^ bit(4));
    const uint8_t p1 = static_cast<uint8_t>(~(bit(1) ^ bit(3) ^ bit(4) ^ bit(5)) & 1);
    return static_cast<uint8_t>(id | (p0 << 6) | (p1 << 7));
}

bool linParsePid(uint8_t pid, uint8_t& id6) {
    const uint8_t id = pid & 0x3F;
    if (linProtectedId(id) != pid) return false;
    id6 = id;
    return true;
}

uint8_t linDefaultLength(uint8_t id6) {
    id6 &= 0x3F;
    if (id6 < 32) return 2;
    if (id6 < 48) return 4;
    return 8;
}

uint8_t linChecksum(uint8_t pid, const uint8_t* data, size_t len, LinChecksum type) {
    uint16_t sum = (type == LinChecksum::Enhanced) ? pid : 0;
    for (size_t i = 0; i < len; ++i) {
        sum = static_cast<uint16_t>(sum + data[i]);
        if (sum > 0xFF) sum = static_cast<uint16_t>(sum - 0xFF);  // add-with-carry
    }
    return static_cast<uint8_t>(~sum);
}

LinChecksum linChecksumFor(uint8_t id6, LinChecksum type) {
    id6 &= 0x3F;
    return (id6 == kLinMasterReqId || id6 == kLinSlaveRespId) ? LinChecksum::Classic : type;
}

size_t linBuildResponse(uint8_t id6, const uint8_t* data, size_t len, LinChecksum type,
                        uint8_t* out, size_t outCap) {
    if (len == 0 || len > 8 || data == nullptr || out == nullptr || outCap < len + 1) return 0;
    const uint8_t pid = linProtectedId(id6);
    std::copy(data, data + len, out);
    out[len] = linChecksum(pid, data, len, linChecksumFor(id6, type));
    return len + 1;
}

Status linVerifyResponse(uint8_t pid, const uint8_t* frame, size_t len, LinChecksum type) {
    uint8_t id6 = 0;
    if (!linParsePid(pid, id6)) return Status::ProtocolError;
    if (frame == nullptr || len < 2 || len > 9) return Status::InvalidArg;
    const size_t n = len - 1;
    const uint8_t expect = linChecksum(pid, frame, n, linChecksumFor(id6, type));
    return expect == frame[n] ? Status::Ok : Status::CrcError;
}

}  // namespace proto
