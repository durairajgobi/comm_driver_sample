# proto — embedded protocol drivers with GoogleTest

Portable C++17 implementations of common embedded/automotive/industrial
protocols, each with a GoogleTest + GoogleMock suite that runs on your PC —
no hardware needed.

| Protocol | Implementation | What the tests check |
|---|---|---|
| **I2C** | `I2cRegisterDevice` (read/write/burst/RMW), `i2cScan`, `Eeprom24` (24LCxx page writes + ACK polling) | exact bus bytes, repeated-START reads, NACK handling, page-boundary splitting against a simulated 24LC256 |
| **SPI** | `SoftSpi` bit-bang master (modes 0–3, MSB/LSB first), `SpiFlash` JEDEC NOR driver (W25Q/MX25L…) | cycle-accurate simulated slave in all 4 modes, page programming, sector erase, busy polling, CS released on error |
| **CAN** | frame validation, acceptance filters, DBC signal codec (Intel/Motorola, signed, factor/offset), bit-timing calculator, `CanNode` | DBC vectors, J1939 example, range checks, timing for STM32/ESP32 clocks |
| **ISO-TP** (ISO 15765-2) | `IsoTpSender` / `IsoTpReceiver` (SF/FF/CF/FC, block size, STmin) | sender↔receiver loopback for 1–4095 bytes × block sizes, sequence-number errors, FC wait/overflow |
| **UART** | SOF/LEN/payload/CRC16 framing, resynchronising byte parser, `UartLink` | noise, corruption, every single-bit error rejected, byte-at-a-time feeds |
| **LIN 2.x** | protected ID, classic/enhanced checksum, response build/verify | spec PID table, all 256 PIDs, carry handling, diagnostic frames forced to classic |
| **1-Wire** | reset/byte I/O, READ/MATCH/SKIP ROM, Maxim AN187 ROM search, `Ds18b20` | bit-level wired-AND bus simulation with multiple devices, datasheet temperature table |
| **Modbus RTU** | request builders, response parser, `ModbusRtuMaster` | spec frame examples, exceptions, timeout, CRC errors, broadcast, against a simulated slave |
| **CRC** | CRC-8/Maxim, CRC-16/CCITT-FALSE, CRC-16/MODBUS | catalogue check values, incremental use |

## Build and run tests

```bash
cmake -S . -B build          # fetches GoogleTest v1.15.2 if not installed
cmake --build build -j
ctest --test-dir build --output-on-failure      # or ./build/proto_tests
```

Options: `-DPROTO_SANITIZE=ON` (ASan + UBSan), `-DPROTO_COVERAGE=ON` (gcov),
`-DPROTO_BUILD_TESTS=OFF` (library only, e.g. for cross-compiling).

Run one protocol: `./build/proto_tests --gtest_filter='*Spi*'`

## Architecture

```
include/proto/hal.hpp      <- the ONLY place hardware is touched (tiny interfaces)
   II2cBus  ISpiBus  ISpiPins  ICanController  IUartPort  IOneWireBus  IDelay
        ^                                   ^
        |  on target: your HAL port         |  in tests: gmock mocks / simulated devices
        |  (STM32 HAL, ESP-IDF, Zephyr,     |  (tests/mocks.hpp, FakeEeprom,
        |   Linux i2c-dev/spidev/SocketCAN) |   FakeSpiFlash, SimSpiSlave, ...)
include/proto/*.hpp + src/*.cpp  <- protocol logic, 100% host-testable
```

Two test styles are used deliberately:

* **Interaction tests (GoogleMock)** — assert the exact bytes and call order
  on the bus, e.g. "register read = write(reg, no STOP) then read".
* **Behavioural tests (simulated devices)** — run the driver against a fake
  chip and check the end result, e.g. "600 bytes written across page boundaries
  read back identically". These survive refactors that change call patterns.

## Porting to hardware (STM32 HAL example)

```cpp
class Stm32I2c : public proto::II2cBus {
public:
    explicit Stm32I2c(I2C_HandleTypeDef* h) : h_(h) {}
    proto::Status write(uint8_t a, const uint8_t* d, size_t n, bool stop) override {
        if (n == 0) return HAL_I2C_IsDeviceReady(h_, a << 1, 1, 10) == HAL_OK
                         ? proto::Status::Ok : proto::Status::Nack;
        auto opt = stop ? I2C_FIRST_AND_LAST_FRAME : I2C_FIRST_FRAME;
        return HAL_I2C_Master_Seq_Transmit_IT(h_, a << 1, const_cast<uint8_t*>(d), n, opt) == HAL_OK
                   ? waitDone() : proto::Status::Nack;
    }
    proto::Status read(uint8_t a, uint8_t* d, size_t n) override {
        return HAL_I2C_Master_Seq_Receive_IT(h_, a << 1, d, n, I2C_LAST_FRAME) == HAL_OK
                   ? waitDone() : proto::Status::Nack;
    }
private:
    proto::Status waitDone();  // wait for the HAL callback / check error code
    I2C_HandleTypeDef* h_;
};

Stm32I2c bus(&hi2c1);
proto::I2cRegisterDevice tmp117(bus, 0x48);
uint16_t raw;
tmp117.readReg16BE(0x00, raw);
```

The other interfaces port the same way: `ISpiBus` → `HAL_SPI_TransmitReceive` +
a CS GPIO; `ICanController` → `HAL_CAN_AddTxMessage` / RX FIFO; `IUartPort` →
UART + ring buffer; `IOneWireBus` → open-drain GPIO with µs timing or a UART
trick; `IDelay` → `HAL_Delay`.

## Design notes

* No exceptions, no RTTI needed; heap only in `IsoTp*` buffers, `i2cScan`,
  `OneWire::searchAll` and `CanNode` filters (swap for fixed arrays if required).
* All functions return `proto::Status`; CS / bus state is always restored on error.
* Out of scope (good next additions): CAN-FD (DLC 9–15 → 12–64 bytes),
  SMBus PEC, SPI SD-card mode, J1939 transport (BAM/CMDT), Modbus slave side,
  USB / Ethernet.
