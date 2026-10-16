#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "proto/hal.hpp"

namespace proto {

constexpr uint32_t kCanStdIdMax = 0x7FF;
constexpr uint32_t kCanExtIdMax = 0x1FFFFFFF;

bool canIsValid(const CanFrame& f);
CanFrame canMakeFrame(uint32_t id, const uint8_t* data, uint8_t dlc, bool extended = false);

// Hardware-style acceptance filter: frame passes if (frame.id & mask) == (id & mask).
struct CanFilter {
    uint32_t id = 0;
    uint32_t mask = 0;
    bool extended = false;
    bool matches(const CanFrame& f) const {
        return f.extended == extended && (f.id & mask) == (id & mask);
    }
};

// ------------------------------------------------ DBC-style signal codec ---
enum class ByteOrder : uint8_t { Intel, Motorola };  // little-endian / big-endian (DBC @1 / @0)

struct CanSignal {
    uint16_t startBit = 0;  // DBC start bit (Intel: LSB position, Motorola: MSB position)
    uint8_t length = 1;     // 1..64
    ByteOrder order = ByteOrder::Intel;
    bool isSigned = false;
    double factor = 1.0;
    double offset = 0.0;
};

uint64_t canExtractRaw(const CanFrame& f, const CanSignal& sig);
void canInsertRaw(CanFrame& f, const CanSignal& sig, uint64_t raw);
double canDecode(const CanFrame& f, const CanSignal& sig);
// Returns false (frame untouched) if the physical value doesn't fit in the signal.
bool canEncode(CanFrame& f, const CanSignal& sig, double physical);

// ------------------------------------------------- bit timing calculator ---
// Classic bxCAN/FDCAN-nominal constraints: TQ count 8..25, TSEG1 1..16, TSEG2 1..8.
struct CanBitTiming {
    uint16_t prescaler = 0;
    uint8_t tseg1 = 0;  // PROP_SEG + PHASE_SEG1
    uint8_t tseg2 = 0;  // PHASE_SEG2
    uint8_t sjw = 0;
    uint32_t actualBitrate = 0;
    double samplePoint = 0.0;  // 0..1
};
bool canComputeBitTiming(uint32_t clockHz, uint32_t bitrate, double targetSamplePoint,
                         CanBitTiming& out);

// ------------------------------------------------------ simple CAN node ---
// Thin wrapper that applies software filters on top of an ICanController.
class CanNode {
public:
    explicit CanNode(ICanController& ctrl) : ctrl_(ctrl) {}
    void addFilter(const CanFilter& f) { filters_.push_back(f); }
    Status send(const CanFrame& f) { return canIsValid(f) ? ctrl_.send(f) : Status::InvalidArg; }
    // Drains the controller until a frame passes the filters (or none left).
    bool receive(CanFrame& out);

private:
    ICanController& ctrl_;
    std::vector<CanFilter> filters_;
};

}  // namespace proto
