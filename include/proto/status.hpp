#pragma once
#include <cstdint>

namespace proto {

enum class Status : uint8_t {
    Ok = 0,
    Nack,            // I2C address/data not acknowledged
    Timeout,         // device did not respond / stayed busy too long
    BusError,        // low-level bus failure
    InvalidArg,      // caller passed something out of range
    CrcError,        // checksum / CRC mismatch
    Busy,            // state machine not ready for this call
    Overflow,        // buffer too small / message too long
    ProtocolError,   // malformed or unexpected frame
    DeviceException, // device answered with an error (e.g. Modbus exception)
};

inline const char* toString(Status s) {
    switch (s) {
        case Status::Ok: return "Ok";
        case Status::Nack: return "Nack";
        case Status::Timeout: return "Timeout";
        case Status::BusError: return "BusError";
        case Status::InvalidArg: return "InvalidArg";
        case Status::CrcError: return "CrcError";
        case Status::Busy: return "Busy";
        case Status::Overflow: return "Overflow";
        case Status::ProtocolError: return "ProtocolError";
        case Status::DeviceException: return "DeviceException";
    }
    return "Unknown";
}

}  // namespace proto
