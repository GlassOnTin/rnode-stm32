# rnode-stm32

Minimal [RNode](https://reticulum.network/hardware.html)-compatible firmware for the **EByte E22P-868MBH-SC** evaluation board (STM32F103C8T6 + SX1262).

Implements the KISS protocol over USB CDC, allowing the board to work as a LoRa radio interface with [Reticulum](https://reticulum.network/) via `RNodeInterface`.

## Hardware

- **MCU**: STM32F103C8T6 (64KB flash, 20KB RAM)
- **Radio**: Semtech SX1262 via SPI, with external PA (up to +30dBm / 1W)
- **TCXO**: 1.6V via DIO3
- **USB**: USB-C, CDC ACM (Virtual COM Port)
- **RF switch**: TXEN (PB12), RXEN (PB13)

### Pin mapping

| Function | Pin |
|----------|-----|
| SPI NSS  | PA4 |
| SPI SCK  | PA5 |
| SPI MISO | PA6 |
| SPI MOSI | PA7 |
| DIO1     | PA3 |
| RESET    | PB0 |
| BUSY     | PB1 |
| TXEN     | PB12 |
| RXEN     | PB13 |
| LED TX   | PA15 |
| LED RX   | PB8 |

Pin mapping verified from [MeshCore](https://github.com/rfrancis/MeshCore) `ebyte_e22p_f103` variant and board schematic.

## Building

Requires [PlatformIO](https://platformio.org/).

```bash
pio run
```

Flash usage: ~53KB / 64KB. RAM: ~7KB / 20KB.

## Flashing

The E22P board has SWD pads (SWDIO, SWCLK) accessible via CON2. The STM32's NRST is also accessible nearby. A J-Link or ST-Link programmer is required.

```bash
# Using PlatformIO (J-Link)
pio run -t upload

# Using OpenOCD (J-Link)
openocd -f interface/jlink.cfg -c "transport select swd; adapter speed 100" \
  -f target/stm32f1x.cfg \
  -c "init; reset halt; flash write_image erase .pio/build/rnode_e22p/firmware.bin 0x08000000; reset run; shutdown"
```

### Notes

- The factory firmware has **read protection (RDP)** enabled. You must unlock it first, which triggers a mass erase:
  ```bash
  openocd -f interface/jlink.cfg -c "transport select swd; adapter speed 100" \
    -f target/stm32f1x.cfg \
    -c "init; reset halt; flash protect 0 0 last off; reset halt; flash write_image erase firmware.bin 0x08000000; reset run; shutdown"
  ```
- Connecting **NRST** to the programmer is recommended — without it, the debugger may fail to halt the CPU.
- An **nRF5340-DK** (or any board with an onboard J-Link) can be used as an SWD programmer by routing its debug-out header to the E22P.

## Reticulum configuration

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

Verify with:

```bash
rnodeconf /dev/ttyACM0 --info    # Should show firmware version 1.60
rnstatus                          # Should show RNode LoRa interface as Up
```

## Implemented features

- USB CDC (Virtual COM Port)
- KISS frame parser and builder with escaping
- Detection handshake (`rnodeconf --info` works)
- Radio parameter configuration: frequency, bandwidth, TX power, spreading factor, coding rate
- Parameter echo-back (passes `validateRadioState()`)
- CMD_DATA TX and RX over LoRa
- RSSI and SNR reporting
- Flow control (CMD_READY)
- CMD_RANDOM, CMD_BLINK, CMD_RESET

## Not yet implemented

- CSMA/CA carrier sense before TX
- Airtime limiting (CMD_ST_ALOCK / CMD_LT_ALOCK tracking)
- LED activity indicators
- Persistent configuration in STM32 flash
- Channel statistics (CMD_STAT_CHTM, CMD_STAT_PHYPRM)

## Architecture

This is a clean-room implementation (~300 lines of application logic), not a fork of [RNode_Firmware](https://github.com/markqvist/RNode_Firmware). The KISS protocol is simple enough to implement from scratch, avoiding the ESP32/NRF52/AVR complexity of the upstream project.

| File | Purpose |
|------|---------|
| `src/kiss.h` | KISS constants and parser/builder API |
| `src/kiss.cpp` | KISS state machine and frame escaping |
| `src/main.cpp` | USB CDC, radio init, command handlers, TX/RX |
| `platformio.ini` | Build configuration |

## License

MIT
