#include "proto/can.hpp"

#include <cmath>

namespace proto {

bool canIsValid(const CanFrame& f) {
    if (f.dlc > 8) return false;
    return f.extended ? f.id <= kCanExtIdMax : f.id <= kCanStdIdMax;
}

CanFrame canMakeFrame(uint32_t id, const uint8_t* data, uint8_t dlc, bool extended) {
    CanFrame f;
    f.id = id;
    f.extended = extended;
    f.dlc = dlc > 8 ? 8 : dlc;
    for (uint8_t i = 0; i < f.dlc && data; ++i) f.data[i] = data[i];
    return f;
}

// --- bit addressing helpers --------------------------------------------------
static bool getBit(const CanFrame& f, unsigned pos) { return (f.data[pos / 8] >> (pos % 8)) & 1; }
static void setBit(CanFrame& f, unsigned pos, bool v) {
    if (v) f.data[pos / 8] |= static_cast<uint8_t>(1u << (pos % 8));
    else   f.data[pos / 8] &= static_cast<uint8_t>(~(1u << (pos % 8)));
}
// Motorola "sawtooth": moving to the next less-significant bit.
static unsigned motorolaNext(unsigned pos) { return (pos % 8 == 0) ? pos + 15 : pos - 1; }

uint64_t canExtractRaw(const CanFrame& f, const CanSignal& sig) {
    uint64_t raw = 0;
    if (sig.order == ByteOrder::Intel) {
        for (int i = sig.length - 1; i >= 0; --i) {
            const unsigned pos = sig.startBit + static_cast<unsigned>(i);
            if (pos >= 64) return 0;
            raw = (raw << 1) | (getBit(f, pos) ? 1u : 0u);
        }
    } else {
        unsigned pos = sig.startBit;
        for (int i = 0; i < sig.length; ++i) {
            if (pos >= 64) return 0;
            raw = (raw << 1) | (getBit(f, pos) ? 1u : 0u);
            pos = motorolaNext(pos);
        }
    }
    return raw;
}

void canInsertRaw(CanFrame& f, const CanSignal& sig, uint64_t raw) {
    if (sig.order == ByteOrder::Intel) {
        for (unsigned i = 0; i < sig.length; ++i) {
            const unsigned pos = sig.startBit + i;
            if (pos >= 64) return;
            setBit(f, pos, (raw >> i) & 1);
        }
    } else {
        unsigned pos = sig.startBit;
        for (int i = sig.length - 1; i >= 0; --i) {
            if (pos >= 64) return;
            setBit(f, pos, (raw >> i) & 1);
            pos = motorolaNext(pos);
        }
    }
}

double canDecode(const CanFrame& f, const CanSignal& sig) {
    const uint64_t raw = canExtractRaw(f, sig);
    double v;
    if (sig.isSigned && sig.length < 64 && (raw >> (sig.length - 1)) & 1) {
        const int64_t s = static_cast<int64_t>(raw) - (int64_t{1} << sig.length);
        v = static_cast<double>(s);
    } else if (sig.isSigned && sig.length == 64) {
        v = static_cast<double>(static_cast<int64_t>(raw));
    } else {
        v = static_cast<double>(raw);
    }
    return v * sig.factor + sig.offset;
}

bool canEncode(CanFrame& f, const CanSignal& sig, double physical) {
    if (sig.length == 0 || sig.length > 64 || sig.factor == 0.0) return false;
    const double scaled = std::llround((physical - sig.offset) / sig.factor);
    uint64_t raw;
    if (sig.isSigned) {
        const double lo = -std::ldexp(1.0, sig.length - 1);
        const double hi = std::ldexp(1.0, sig.length - 1) - 1;
        if (scaled < lo || scaled > hi) return false;
        raw = static_cast<uint64_t>(static_cast<int64_t>(scaled));
    } else {
        const double hi = std::ldexp(1.0, sig.length) - 1;
        if (scaled < 0 || scaled > hi) return false;
        raw = static_cast<uint64_t>(scaled);
    }
    if (sig.length < 64) raw &= (uint64_t{1} << sig.length) - 1;
    canInsertRaw(f, sig, raw);
    return true;
}

bool canComputeBitTiming(uint32_t clockHz, uint32_t bitrate, double targetSp, CanBitTiming& out) {
    if (clockHz == 0 || bitrate == 0 || targetSp <= 0.5 || targetSp >= 1.0) return false;
    bool found = false;
    double bestErr = 1e9;
    for (uint32_t presc = 1; presc <= 1024; ++presc) {
        if (clockHz % (presc * bitrate) != 0) continue;  // exact bitrate only
        const uint32_t tq = clockHz / (presc * bitrate);
        if (tq < 8 || tq > 25) continue;
        uint32_t tseg2 = static_cast<uint32_t>(std::lround(tq * (1.0 - targetSp)));
        if (tseg2 < 1) tseg2 = 1;
        if (tseg2 > 8) tseg2 = 8;
        const uint32_t tseg1 = tq - 1 - tseg2;
        if (tseg1 < 1 || tseg1 > 16) continue;
        const double sp = static_cast<double>(1 + tseg1) / tq;
        const double err = std::fabs(sp - targetSp);
        // Prefer smaller error; on tie prefer more time quanta (finer resync).
        if (!found || err < bestErr - 1e-9 ||
            (std::fabs(err - bestErr) < 1e-9 && tq > 1u + out.tseg1 + out.tseg2)) {
            found = true;
            bestErr = err;
            out.prescaler = static_cast<uint16_t>(presc);
            out.tseg1 = static_cast<uint8_t>(tseg1);
            out.tseg2 = static_cast<uint8_t>(tseg2);
            out.sjw = static_cast<uint8_t>(tseg2 < 4 ? tseg2 : 4);
            out.actualBitrate = clockHz / (presc * tq);
            out.samplePoint = sp;
        }
    }
    return found;
}

bool CanNode::receive(CanFrame& out) {
    CanFrame f;
    while (ctrl_.receive(f)) {
        if (filters_.empty()) { out = f; return true; }
        for (const auto& flt : filters_) {
            if (flt.matches(f)) { out = f; return true; }
        }
    }
    return false;
}

}  // namespace proto
