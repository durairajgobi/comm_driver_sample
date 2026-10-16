#include <gtest/gtest.h>

#include <algorithm>
#include <set>

#include "proto/crc.hpp"
#include "proto/onewire.hpp"

using namespace proto;

// =========================================================================
// Bit-level simulation of a 1-Wire bus with several slaves (wired-AND).
// Implements SEARCH ROM, READ ROM, MATCH ROM, SKIP ROM and DS18B20
// CONVERT T / READ SCRATCHPAD.
// =========================================================================
struct SimDevice {
    RomCode rom;
    std::array<uint8_t, 9> scratchpad{};
};

class SimOneWireBus : public IOneWireBus {
public:
    std::vector<SimDevice> devices;
    int conversions = 0;
    bool corruptReads = false;

    bool reset() override {
        active_.clear();
        for (size_t i = 0; i < devices.size(); ++i) active_.push_back(i);
        state_ = St::RomCmd;
        bitCnt_ = 0;
        acc_ = 0;
        return !devices.empty();
    }

    void writeBit(bool b) override {
        switch (state_) {
            case St::RomCmd:
                if (b) acc_ |= 1u << bitCnt_;
                if (++bitCnt_ == 8) onRomCmd(static_cast<uint8_t>(acc_));
                break;
            case St::Search:
                dropIf([&](const SimDevice& d) { return romBit(d, bitCnt_) != b; });
                ++bitCnt_;
                phase_ = 0;
                break;
            case St::Match:
                dropIf([&](const SimDevice& d) { return romBit(d, bitCnt_) != b; });
                if (++bitCnt_ == 64) { state_ = St::FnCmd; bitCnt_ = 0; acc_ = 0; }
                break;
            case St::FnCmd:
                if (b) acc_ |= 1u << bitCnt_;
                if (++bitCnt_ == 8) onFnCmd(static_cast<uint8_t>(acc_));
                break;
            default: break;
        }
    }

    bool readBit() override {
        bool v = true;  // pull-up: idle bus reads 1
        switch (state_) {
            case St::Search:
                for (size_t i : active_) {
                    const bool bit = romBit(devices[i], bitCnt_);
                    v = v && (phase_ == 0 ? bit : !bit);
                }
                ++phase_;
                return v;
            case St::ReadRom:
                for (size_t i : active_) v = v && romBit(devices[i], bitCnt_);
                ++bitCnt_;
                return v;
            case St::ReadData:
                for (size_t i : active_) {
                    v = v && ((devices[i].scratchpad[bitCnt_ / 8] >> (bitCnt_ % 8)) & 1);
                }
                if (corruptReads && bitCnt_ == 3) v = !v;
                ++bitCnt_;
                return v;
            default:
                return v;
        }
    }

private:
    enum class St { Idle, RomCmd, Search, ReadRom, Match, FnCmd, ReadData };
    static bool romBit(const SimDevice& d, int n) { return (d.rom[n / 8] >> (n % 8)) & 1; }
    template <class Pred> void dropIf(Pred p) {
        active_.erase(std::remove_if(active_.begin(), active_.end(),
                                     [&](size_t i) { return p(devices[i]); }),
                      active_.end());
    }
    void onRomCmd(uint8_t c) {
        bitCnt_ = 0; acc_ = 0; phase_ = 0;
        switch (c) {
            case OneWire::kSearchRom: state_ = St::Search; break;
            case OneWire::kReadRom: state_ = St::ReadRom; break;
            case OneWire::kMatchRom: state_ = St::Match; break;
            case OneWire::kSkipRom: state_ = St::FnCmd; break;
            default: state_ = St::Idle;
        }
    }
    void onFnCmd(uint8_t c) {
        bitCnt_ = 0;
        if (c == Ds18b20::kConvertT) { conversions += static_cast<int>(active_.size()); state_ = St::Idle; }
        else if (c == Ds18b20::kReadScratchpad) state_ = St::ReadData;
        else state_ = St::Idle;
    }
    St state_ = St::Idle;
    std::vector<size_t> active_;
    int bitCnt_ = 0, phase_ = 0;
    uint32_t acc_ = 0;
};

static RomCode makeRom(uint8_t family, uint64_t serial) {
    RomCode r{};
    r[0] = family;
    for (int i = 0; i < 6; ++i) r[1 + i] = static_cast<uint8_t>(serial >> (8 * i));
    r[7] = crc8Maxim(r.data(), 7);
    return r;
}

static std::array<uint8_t, 9> scratch(int16_t raw) {
    std::array<uint8_t, 9> s = {static_cast<uint8_t>(raw & 0xFF), static_cast<uint8_t>((raw >> 8) & 0xFF),
                                0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10, 0};
    s[8] = crc8Maxim(s.data(), 8);
    return s;
}

// ------------------------------------------------------------- ROM ---
TEST(OneWire, ReadRomFromSingleDevice) {
    SimOneWireBus bus;
    const RomCode rom = makeRom(0x28, 0x0000123456789ABCull);
    bus.devices.push_back({rom, {}});
    OneWire ow(bus);
    RomCode got{};
    ASSERT_EQ(ow.readRom(got), Status::Ok);
    EXPECT_EQ(got, rom);
}

TEST(OneWire, NoPresencePulseIsTimeout) {
    SimOneWireBus bus;
    OneWire ow(bus);
    RomCode got{};
    EXPECT_EQ(ow.readRom(got), Status::Timeout);
}

TEST(OneWire, ReadRomWithTwoDevicesCollidesAndFailsCrc) {
    SimOneWireBus bus;
    bus.devices.push_back({makeRom(0x28, 1), {}});
    bus.devices.push_back({makeRom(0x28, 2), {}});
    OneWire ow(bus);
    RomCode got{};
    EXPECT_EQ(ow.readRom(got), Status::CrcError);
}

TEST(OneWire, RomValidation) {
    RomCode r = makeRom(0x10, 42);
    EXPECT_TRUE(oneWireRomValid(r));
    r[3] ^= 1;
    EXPECT_FALSE(oneWireRomValid(r));
    EXPECT_FALSE(oneWireRomValid(RomCode{}));  // family 0 = no device
}

// ----------------------------------------------------------- search ---
class OneWireSearch : public ::testing::TestWithParam<std::vector<uint64_t>> {};

TEST_P(OneWireSearch, FindsEveryDeviceExactlyOnce) {
    SimOneWireBus bus;
    std::set<RomCode> expected;
    for (uint64_t s : GetParam()) {
        const RomCode r = makeRom(0x28, s);
        bus.devices.push_back({r, {}});
        expected.insert(r);
    }
    OneWire ow(bus);
    const auto found = ow.searchAll();
    EXPECT_EQ(found.size(), expected.size());
    EXPECT_EQ(std::set<RomCode>(found.begin(), found.end()), expected);
}

INSTANTIATE_TEST_SUITE_P(
    Topologies, OneWireSearch,
    ::testing::Values(std::vector<uint64_t>{0xABCDEF},
                      std::vector<uint64_t>{1, 2},                      // differ in low bits
                      std::vector<uint64_t>{0x800000000000, 0x000000000001},  // differ at both ends
                      std::vector<uint64_t>{10, 11, 12, 13, 14, 15, 16, 17},
                      std::vector<uint64_t>{0x111111, 0x222222, 0x333333, 0x7FFFFFFFFFFF, 0}));

TEST(OneWireSearchMixed, DifferentFamiliesOnOneBus) {
    SimOneWireBus bus;
    bus.devices.push_back({makeRom(0x28, 5), {}});  // DS18B20
    bus.devices.push_back({makeRom(0x10, 5), {}});  // DS18S20
    bus.devices.push_back({makeRom(0x01, 5), {}});  // DS2401 silicon serial
    OneWire ow(bus);
    EXPECT_EQ(ow.searchAll().size(), 3u);
}

TEST(OneWireSearchMixed, EmptyBusFindsNothing) {
    SimOneWireBus bus;
    OneWire ow(bus);
    EXPECT_TRUE(ow.searchAll().empty());
}

// ---------------------------------------------------------- DS18B20 ---
struct TempCase { int16_t raw; float celsius; };
class Ds18b20Conversion : public ::testing::TestWithParam<TempCase> {};

// Values from the DS18B20 datasheet, table 1.
TEST_P(Ds18b20Conversion, DatasheetTable) {
    EXPECT_FLOAT_EQ(Ds18b20::rawToCelsius(GetParam().raw), GetParam().celsius);
}

INSTANTIATE_TEST_SUITE_P(Datasheet, Ds18b20Conversion,
                         ::testing::Values(TempCase{0x07D0, 125.0f}, TempCase{0x0550, 85.0f},
                                           TempCase{0x0191, 25.0625f}, TempCase{0x00A2, 10.125f},
                                           TempCase{0x0008, 0.5f}, TempCase{0x0000, 0.0f},
                                           TempCase{static_cast<int16_t>(0xFFF8), -0.5f},
                                           TempCase{static_cast<int16_t>(0xFF5E), -10.125f},
                                           TempCase{static_cast<int16_t>(0xFE6F), -25.0625f},
                                           TempCase{static_cast<int16_t>(0xFC90), -55.0f}));

TEST(Ds18b20, SkipRomReadOnSingleDropBus) {
    SimOneWireBus bus;
    bus.devices.push_back({makeRom(0x28, 1), scratch(0x0191)});
    OneWire ow(bus);
    Ds18b20 sensor(ow);
    ASSERT_EQ(sensor.startConversion(), Status::Ok);
    EXPECT_EQ(bus.conversions, 1);
    float t = 0;
    ASSERT_EQ(sensor.readTemperature(t), Status::Ok);
    EXPECT_FLOAT_EQ(t, 25.0625f);
}

TEST(Ds18b20, MatchRomAddressesTheRightSensor) {
    SimOneWireBus bus;
    const RomCode a = makeRom(0x28, 0xAAAA), b = makeRom(0x28, 0xBBBB);
    bus.devices.push_back({a, scratch(0x0550)});
    bus.devices.push_back({b, scratch(static_cast<int16_t>(0xFF5E))});
    OneWire ow(bus);
    Ds18b20 sensor(ow);

    ASSERT_EQ(sensor.startConversion(), Status::Ok);  // SKIP ROM: all sensors convert
    EXPECT_EQ(bus.conversions, 2);
    float ta = 0, tb = 0;
    ASSERT_EQ(sensor.readTemperature(ta, &a), Status::Ok);
    ASSERT_EQ(sensor.readTemperature(tb, &b), Status::Ok);
    EXPECT_FLOAT_EQ(ta, 85.0f);
    EXPECT_FLOAT_EQ(tb, -10.125f);
}

TEST(Ds18b20, CorruptScratchpadRejected) {
    SimOneWireBus bus;
    bus.devices.push_back({makeRom(0x28, 1), scratch(0x0191)});
    bus.corruptReads = true;
    OneWire ow(bus);
    Ds18b20 sensor(ow);
    float t = 123;
    EXPECT_EQ(sensor.readTemperature(t), Status::CrcError);
    EXPECT_FLOAT_EQ(t, 123);  // output untouched on error
}

TEST(Ds18b20, NoSensorIsTimeout) {
    SimOneWireBus bus;
    OneWire ow(bus);
    Ds18b20 sensor(ow);
    float t;
    EXPECT_EQ(sensor.readTemperature(t), Status::Timeout);
}
