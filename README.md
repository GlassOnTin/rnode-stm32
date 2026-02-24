# rnode-stm32

Minimal [RNode](https://reticulum.network/hardware.html)-compatible firmware for the **EByte E22P-868MBH-SC** evaluation board (STM32F103C8T6 + SX1262). Works as a LoRa radio interface with [Reticulum](https://reticulum.network/) via `RNodeInterface`.

## Hardware

- **MCU**: STM32F103C8T6 (64KB flash, 20KB RAM)
- **Radio**: Semtech SX1262 via SPI, with external PA (up to +30dBm / 1W)
- **USB**: USB-C, CDC ACM (Virtual COM Port)

See [docs/hardware.md](docs/hardware.md) for pin mapping and board details.

## Quick start

Requires [PlatformIO](https://platformio.org/).

```bash
pio run                  # build
./upload.sh              # flash (Linux) — or: pio run -t upload
```

Flash usage: ~55KB / 62KB available (2KB reserved for bootloader). RAM: ~8KB / 20KB.

See [docs/flashing.md](docs/flashing.md) for remote flashing, SWD bootloader setup, and udev rules.

### Reticulum configuration

Add to `~/.reticulum/config`:

```ini
[[RNode LoRa]]
  type = RNodeInterface
  interface_enabled = True
  port = /dev/ttyACM0
  frequency = 868000000
  bandwidth = 125000
  txpower = 2
  spreadingfactor = 7
  codingrate = 5
```

`txpower` is in dBm. The SX1262 supports -9 to +22 dBm; with the E22P's external PA, effective radiated power is higher. Start low (2 dBm) for bench testing.

Verify with:

```bash
rnodeconf /dev/ttyACM0 --info    # Should show firmware version 1.62
rnstatus                          # Should show RNode LoRa interface as Up
```

## Features

- USB CDC serial with KISS frame parser/builder
- Detection handshake and radio parameter configuration (passes `validateRadioState()`)
- Non-blocking TX via DIO1 ISR; p-persistent CSMA/CA
- TX guards: rejects oversized packets and back-to-back TX
- RSSI/SNR reporting, airtime tracking and enforcement
- Channel statistics, PHY parameter reporting, CPU temperature
- LED indicators (TX/RX)

See [docs/protocol.md](docs/protocol.md) for the full features list, KISS protocol details, error handling, and USB CDC notes.

## Architecture

A clean-room implementation (~770 lines of application code, excluding RadioLib). Not a fork of [RNode_Firmware](https://github.com/markqvist/RNode_Firmware) — the KISS protocol is simple enough to implement from scratch, avoiding the ESP32/NRF52/AVR complexity of the upstream project.

| File | Purpose |
|------|---------|
| `src/kiss.h` | KISS constants and parser/builder API |
| `src/kiss.cpp` | KISS state machine and frame escaping |
| `src/main.cpp` | USB CDC, radio init, command handlers, CSMA, TX/RX, diagnostics |
| `src/diag_main.cpp` | Standalone diagnostic firmware (separate build target `diag_e22p`) |
| `platformio.ini` | Build configuration and pin definitions |
| `upload.sh` | Build, flash, and rebind cdc_acm (Linux) |
| `remote_flash.sh` | Build, scp, and flash to remote host over SSH |

## License

MIT
