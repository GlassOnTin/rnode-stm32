# rnode-stm32

Minimal [RNode](https://reticulum.network/hardware.html)-compatible firmware for the **EByte E22P-868MBH-SC** evaluation board (STM32F103C8T6 + SX1262).

The host communicates over USB serial using KISS framing: each command or data packet is wrapped in `FEND` delimiters with a command byte (frequency, bandwidth, TX power, etc.). The firmware parses these frames, configures the SX1262 radio, and relays packets between the USB serial link and the LoRa air interface. This allows the board to work as a LoRa radio interface with [Reticulum](https://reticulum.network/) via `RNodeInterface`.

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

Flash usage: ~55KB / 62KB available (2KB reserved for bootloader). RAM: ~7KB / 20KB.

## Flashing

Firmware uploads go over the existing USB-C cable using a 2KB [HID bootloader](https://github.com/Serasidis/STM32_HID_Bootloader) (driverless on all platforms). SWD is only needed once to flash the bootloader itself.

```bash
pio run -t upload
```

### One-time bootloader setup (SWD)

The E22P board has SWD pads (SWDIO, SWCLK) accessible via CON2. Flash the HID bootloader once:

```bash
# Download and extract bootloader (STM32F103 low/medium density, LED on PC13)
wget -O /tmp/stm32_binaries.zip \
  https://github.com/Serasidis/STM32_HID_Bootloader/releases/download/2.2.2/stm32_binaries.zip
unzip -j /tmp/stm32_binaries.zip \
  "stm32_binaries/F103/low_and_medium_density/hid_generic_pc13.bin" -d /tmp

# Flash bootloader at 0x08000000
openocd -f interface/jlink.cfg -c "transport select swd" \
  -f target/stm32f1x.cfg -c "reset_config none separate" \
  -c "init" -c "reset halt" \
  -c "flash write_image erase /tmp/hid_generic_pc13.bin 0x08000000" \
  -c "reset run" -c "shutdown"

# Flash firmware at 0x08000800 (first time only, subsequent uploads use USB)
openocd -f interface/jlink.cfg -c "transport select swd" \
  -f target/stm32f1x.cfg -c "reset_config none separate" \
  -c "init" -c "reset halt" \
  -c "flash write_image erase .pio/build/rnode_e22p/firmware.bin 0x08000800" \
  -c "reset run" -c "shutdown"
```

The `hid-flash` CLI tool is required on the host. PlatformIO does not bundle it. Build from [source](https://github.com/Serasidis/STM32_HID_Bootloader/tree/master/cli) (`make` in the `cli/` directory) and install to PATH.

### Flash layout

```
0x08000000  HID Bootloader (2 KB)
0x08000800  RNode Firmware  (62 KB available)
0x08010000  End of flash
```

### Notes

- **JLink via PlatformIO often fails** on this board because the nRF5340-DK's onboard J-Link cannot assert the STM32's NRST pin over the SWD wires alone. Use the **openocd commands above** with `reset_config none separate` which avoids the reset pin dependency.
- The factory firmware has **read protection (RDP)** enabled. You must unlock it first, which triggers a mass erase:
  ```bash
  openocd -f interface/jlink.cfg -c "transport select swd" \
    -f target/stm32f1x.cfg -c "reset_config none separate" \
    -c "init; reset halt; flash protect 0 0 last off; reset halt; flash write_image erase firmware.bin 0x08000000; reset run; shutdown"
  ```
- An **nRF5340-DK** (or any board with an onboard J-Link) can be used as an SWD programmer by routing its debug-out header to the E22P.
- SWD can still flash firmware directly to `0x08000800` as a fallback if USB upload is unavailable.

### Linux udev rules

The STM32's USB CDC implementation requires `hupcl` to be disabled on the tty, otherwise the kernel drops DTR on port close and the tty becomes unresponsive until rebound. Install these rules to fix this automatically:

```bash
# HID bootloader permissions (for upload without sudo)
sudo tee /etc/udev/rules.d/99-stm32-hid-bootloader.rules << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="beba", MODE="0666"
EOF

# CDC serial port fix (disable hupcl to prevent stale tty)
sudo tee /etc/udev/rules.d/99-stm32-cdc.rules << 'EOF'
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", RUN+="/bin/stty -F /dev/%k -hupcl"
EOF

sudo udevadm control --reload-rules
```

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

`txpower` is in dBm. The SX1262 supports -9 to +22 dBm; with the E22P's external PA, effective radiated power is higher. Start low (2 dBm) for bench testing.

Verify with:

```bash
rnodeconf /dev/ttyACM0 --info    # Should show firmware version 1.60
rnstatus                          # Should show RNode LoRa interface as Up
```

## Features

**Protocol**
- USB CDC serial with KISS frame parser/builder
- Detection handshake (`rnodeconf --info`)
- Radio parameter configuration and echo-back (passes `validateRadioState()`)
- Flow control (CMD_READY after each TX completion)

**Radio**
- Non-blocking TX via `startTransmit()` + DIO1 ISR (USB serial stays responsive during TX)
- p-persistent CSMA/CA (p=0.5) with carrier sense before transmit
- RSSI and SNR reporting per received packet
- CMD_RANDOM, CMD_BLINK, CMD_RESET

**Diagnostics**
- Airtime tracking and enforcement (CMD_ST_ALOCK / CMD_LT_ALOCK, 15s and 1h windows)
- Channel statistics (CMD_STAT_CHTM: airtime %, channel load %, current RSSI, noise floor)
- PHY parameter reporting (CMD_STAT_PHYPRM: symbol time/rate, preamble, CSMA timing)
- CPU temperature (CMD_STAT_TEMP via STM32 internal ADC)
- LED indicators (TX LED during transmit, RX LED pulse on receive)

## Error handling

- **Radio init failure**: both LEDs blink continuously; USB CDC still enumerates but no commands are processed.
- **TX failure** (`startTransmit` returns error): CMD_ERROR with `ERROR_TXFAILED` (0x02) is sent to the host, radio returns to RX.
- **Airtime limit exceeded**: CMD_ERROR with `ERROR_QUEUE_FULL` (0x04), packet is dropped.
- **Malformed KISS frames**: the parser silently discards invalid escape sequences and oversized frames.
- **CSMA timeout**: after 32 busy-channel retries, the packet is transmitted anyway to avoid indefinite blocking.

## Architecture

A clean-room implementation in `kiss.cpp`, `kiss.h`, and `main.cpp` (~600 lines of application code, excluding RadioLib). Not a fork of [RNode_Firmware](https://github.com/markqvist/RNode_Firmware) — the KISS protocol is simple enough to implement from scratch, avoiding the ESP32/NRF52/AVR complexity of the upstream project.

| File | Purpose |
|------|---------|
| `src/kiss.h` | KISS constants and parser/builder API |
| `src/kiss.cpp` | KISS state machine and frame escaping |
| `src/main.cpp` | USB CDC, radio init, command handlers, CSMA, TX/RX, diagnostics |
| `platformio.ini` | Build configuration and pin definitions |

## License

MIT
