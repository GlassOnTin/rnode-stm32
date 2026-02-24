# Protocol

## KISS framing

The host communicates over USB serial using KISS framing: each command or data packet is wrapped in `FEND` delimiters with a command byte (frequency, bandwidth, TX power, etc.). The firmware parses these frames, configures the SX1262 radio, and relays packets between the USB serial link and the LoRa air interface. This allows the board to work as a LoRa radio interface with [Reticulum](https://reticulum.network/) via `RNodeInterface`.

## Features

**Protocol**
- USB CDC serial with KISS frame parser/builder
- Detection handshake (`rnodeconf --info`)
- Radio parameter configuration and echo-back (passes `validateRadioState()`)
- Flow control (CMD_READY after each TX completion)

**Radio**
- Non-blocking TX via `startTransmit()` + DIO1 ISR (USB serial stays responsive during TX)
- p-persistent CSMA/CA (p=0.5) with carrier sense before transmit
- TX guards: rejects packets >255 bytes (SX1262 LoRa max) and back-to-back TX while CSMA pending
- DIO2 RF switch disabled — E22P uses MCU-driven TXEN/RXEN, not SX1262 DIO2
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
- **Oversized packet** (>255 bytes): CMD_ERROR with `ERROR_TXFAILED` (0x02), packet is rejected before reaching CSMA.
- **Back-to-back TX**: CMD_ERROR with `ERROR_QUEUE_FULL` (0x04) if a packet arrives while CSMA is already pending.
- **Airtime limit exceeded**: CMD_ERROR with `ERROR_QUEUE_FULL` (0x04), packet is dropped.
- **Malformed KISS frames**: the parser silently discards invalid escape sequences and oversized frames.
- **CSMA timeout**: after 32 busy-channel retries, the packet is transmitted anyway to avoid indefinite blocking.

## USB CDC notes

The STM32 Arduino USB CDC driver has a receive queue that defaults to 3 USB packets (192 bytes). When the host sends large KISS frames back-to-back (as `rnsd` does without flow control), this queue overflows and causes a hard fault. The build sets `CDC_RECEIVE_QUEUE_BUFFER_PACKET_NUMBER=16` (1024 bytes) to prevent this.

The `CDC_unstick_tx()` function (added to the framework's `usbd_cdc_if.c`) periodically clears stale USB IN transfer state. On STM32F103 CDC-ACM, the host may be slow to poll the IN endpoint, leaving `TxState=1` indefinitely and blocking all `Serial.write()` calls.

## Architecture

A clean-room implementation in `kiss.cpp`, `kiss.h`, and `main.cpp` (~770 lines of application code, excluding RadioLib). Not a fork of [RNode_Firmware](https://github.com/markqvist/RNode_Firmware) — the KISS protocol is simple enough to implement from scratch, avoiding the ESP32/NRF52/AVR complexity of the upstream project.

| File | Purpose |
|------|---------|
| `src/kiss.h` | KISS constants and parser/builder API |
| `src/kiss.cpp` | KISS state machine and frame escaping |
| `src/main.cpp` | USB CDC, radio init, command handlers, CSMA, TX/RX, diagnostics |
| `src/diag_main.cpp` | Standalone diagnostic firmware (separate build target `diag_e22p`) |
| `platformio.ini` | Build configuration and pin definitions |
| `upload.sh` | Build, flash, and rebind cdc_acm (Linux) |
| `remote_flash.sh` | Build, scp, and flash to remote host over SSH |
