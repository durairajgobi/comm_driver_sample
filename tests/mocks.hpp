#pragma once
// GoogleMock mocks for every HAL interface + small shared helpers.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "proto/hal.hpp"

namespace proto::test {

class MockI2cBus : public II2cBus {
public:
    MOCK_METHOD(Status, write, (uint8_t addr7, const uint8_t* data, size_t len, bool sendStop), (override));
    MOCK_METHOD(Status, read, (uint8_t addr7, uint8_t* data, size_t len), (override));
};

class MockSpiBus : public ISpiBus {
public:
    MOCK_METHOD(void, select, (bool active), (override));
    MOCK_METHOD(Status, transfer, (const uint8_t* tx, uint8_t* rx, size_t len), (override));
};

class MockCanController : public ICanController {
public:
    MOCK_METHOD(Status, send, (const CanFrame& frame), (override));
    MOCK_METHOD(bool, receive, (CanFrame & frame), (override));
};

class MockUartPort : public IUartPort {
public:
    MOCK_METHOD(size_t, write, (const uint8_t* data, size_t len), (override));
    MOCK_METHOD(bool, readByte, (uint8_t & byte), (override));
};

class MockDelay : public IDelay {
public:
    MOCK_METHOD(void, delayMs, (uint32_t ms), (override));
};

// Buffer matchers for (pointer, length) argument pairs. They see the real
// length, so a mismatched-size call can never be read out of bounds.
// Usage: EXPECT_CALL(bus, write(0x48, _, _, true)).With(I2cBytes(V({...})));
inline auto I2cBytes(const std::vector<uint8_t>& v) {      // write(addr, data, len, stop)
    return ::testing::Args<1, 2>(::testing::ElementsAreArray(v));
}
inline auto SpiTxBytes(const std::vector<uint8_t>& v) {    // transfer(tx, rx, len)
    return ::testing::Args<0, 2>(::testing::ElementsAreArray(v));
}
inline auto UartBytes(const std::vector<uint8_t>& v) {     // write(data, len)
    return ::testing::Args<0, 1>(::testing::ElementsAreArray(v));
}

inline std::vector<uint8_t> V(std::initializer_list<int> l) {
    std::vector<uint8_t> v;
    for (int x : l) v.push_back(static_cast<uint8_t>(x));
    return v;
}

}  // namespace proto::test
