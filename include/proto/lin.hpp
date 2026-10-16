#pragma once
// LIN 2.x frame helpers (protected identifier, checksums, response framing).
// The break/sync field generation is UART-hardware specific (LIN break
// detection / send-break bit) and belongs in your HAL.

#include <cstddef>
#include <cstdint>

#include "proto/status.hpp"

namespace proto {

enum class LinChecksum : uint8_t { Classic, Enhanced };

constexpr uint8_t kLinSync = 0x55;
constexpr uint8_t kLinMasterReqId = 0x3C;  // diagnostic frames: always classic checksum
constexpr uint8_t kLinSlaveRespId = 0x3D;

// 6-bit frame ID -> PID with parity bits P0 (bit6) and P1 (bit7).
uint8_t linProtectedId(uint8_t id6);
// Validates parity; on success writes the 6-bit ID.
bool linParsePid(uint8_t pid, uint8_t& id6);
// Data length implied by the ID for LIN 1.x style configuration (IDs 0-31:2, 32-47:4, 48-63:8).
uint8_t linDefaultLength(uint8_t id6);

uint8_t linChecksum(uint8_t pid, const uint8_t* data, size_t len, LinChecksum type);

// Picks Classic for diagnostic IDs 0x3C/0x3D, else `type`.
LinChecksum linChecksumFor(uint8_t id6, LinChecksum type);

// Builds a response: data bytes + checksum. Returns bytes written or 0 on error.
size_t linBuildResponse(uint8_t id6, const uint8_t* data, size_t len, LinChecksum type,
                        uint8_t* out, size_t outCap);
// Verifies a received response (data + trailing checksum byte) for a given PID.
Status linVerifyResponse(uint8_t pid, const uint8_t* frame, size_t len, LinChecksum type);

}  // namespace proto
