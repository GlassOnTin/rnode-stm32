# Hardware

## Specs

- **MCU**: STM32F103C8T6 (64KB flash, 20KB RAM)
- **Radio**: Semtech SX1262 via SPI, with external PA (up to +30dBm / 1W)
- **TCXO**: 1.6V via DIO3
- **USB**: USB-C, CDC ACM (Virtual COM Port)
- **RF switch**: TXEN (PB12), RXEN (PB13)

## Pin mapping

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
