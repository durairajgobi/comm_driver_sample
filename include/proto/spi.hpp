#pragma once
#include <cstddef>
#include <cstdint>

#include "proto/hal.hpp"

namespace proto {

// ------------------------------------------------- bit-banged SPI master ---
enum class SpiMode : uint8_t { Mode0 = 0, Mode1 = 1, Mode2 = 2, Mode3 = 3 };
enum class BitOrder : uint8_t { MsbFirst, LsbFirst };

inline bool spiCpol(SpiMode m) { return (static_cast<uint8_t>(m) & 0x2) != 0; }
inline bool spiCpha(SpiMode m) { return (static_cast<uint8_t>(m) & 0x1) != 0; }

// Implements ISpiBus on top of GPIO pins, so any SPI driver (e.g. SpiFlash)
// can run over bit-banged pins.
class SoftSpi : public ISpiBus {
public:
    SoftSpi(ISpiPins& pins, SpiMode mode = SpiMode::Mode0, BitOrder order = BitOrder::MsbFirst);

    void select(bool active) override;
    Status transfer(const uint8_t* tx, uint8_t* rx, size_t len) override;
    uint8_t transferByte(uint8_t out);

private:
    ISpiPins& pins_;
    bool cpol_;
    bool cpha_;
    BitOrder order_;
};

// ------------------------------------------ JEDEC serial NOR flash driver ---
// Works with W25Qxx, MX25Lxx, IS25LPxx, AT25SFxx and similar (24-bit addressing).
struct JedecId {
    uint8_t manufacturer = 0;
    uint8_t memoryType = 0;
    uint8_t capacityCode = 0;
    uint32_t capacityBytes() const { return capacityCode < 32 ? (1u << capacityCode) : 0; }
};

class SpiFlash {
public:
    enum Cmd : uint8_t {
        kWriteEnable = 0x06,
        kReadStatus1 = 0x05,
        kReadData = 0x03,
        kPageProgram = 0x02,
        kSectorErase4K = 0x20,
        kChipErase = 0xC7,
        kJedecId = 0x9F,
    };
    static constexpr uint8_t kStatusBusy = 0x01;
    static constexpr uint8_t kStatusWel = 0x02;
    static constexpr uint32_t kSectorSize = 4096;

    struct Config {
        uint32_t pageSize = 256;
        uint32_t maxBusyPolls = 5000;
        uint32_t capacityBytes = 0;  // 0 = don't range-check
    };

    SpiFlash(ISpiBus& bus, IDelay& delay) : SpiFlash(bus, delay, Config{}) {}
    SpiFlash(ISpiBus& bus, IDelay& delay, Config cfg) : bus_(bus), delay_(delay), cfg_(cfg) {}

    Status readJedecId(JedecId& id);
    Status readStatus(uint8_t& status);
    Status read(uint32_t addr, uint8_t* data, size_t len);
    Status write(uint32_t addr, const uint8_t* data, size_t len);  // area must be erased
    Status eraseSector(uint32_t addr);
    Status waitReady();

private:
    Status writeEnable();
    Status command(const uint8_t* hdr, size_t hdrLen, const uint8_t* tx, uint8_t* rx, size_t len);
    bool inRange(uint32_t addr, size_t len) const;

    ISpiBus& bus_;
    IDelay& delay_;
    Config cfg_;
};

}  // namespace proto
