#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "kiss.h"

// ---------------------------------------------------------------------------
// Hardware setup
// ---------------------------------------------------------------------------
static SPIClass spi(P_LORA_MOSI, P_LORA_MISO, P_LORA_SCLK);
static SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static KissParser parser;
static uint8_t tx_frame_buf[KISS_BUF_SIZE];
static bool radio_ready = false;
static bool rx_active = false;
static volatile bool dio1_flag = false;  // set by ISR on DIO1 rising edge
static bool transmitting = false;

// Current radio parameters (echoed back to host)
static uint32_t cfg_frequency = 0;
static uint32_t cfg_bandwidth = 0;
static uint8_t  cfg_txpower   = 0;
static uint8_t  cfg_sf        = 0;
static uint8_t  cfg_cr        = 0;
static uint8_t  cfg_state     = RADIO_STATE_OFF;

// ---------------------------------------------------------------------------
// Send a KISS frame over USB CDC
// ---------------------------------------------------------------------------
static void kiss_send(uint8_t cmd, const uint8_t *data, size_t len) {
    size_t n = kiss_build_frame(tx_frame_buf, sizeof(tx_frame_buf), cmd, data, len);
    if (n > 0) {
        Serial.write(tx_frame_buf, n);
    }
}

static void kiss_send_byte(uint8_t cmd, uint8_t val) {
    kiss_send(cmd, &val, 1);
}

static void kiss_send_u16(uint8_t cmd, uint16_t val) {
    uint8_t buf[2] = {
        (uint8_t)(val >> 8),
        (uint8_t)(val & 0xFF)
    };
    kiss_send(cmd, buf, 2);
}

static void kiss_send_u32(uint8_t cmd, uint32_t val) {
    uint8_t buf[4] = {
        (uint8_t)(val >> 24),
        (uint8_t)((val >> 16) & 0xFF),
        (uint8_t)((val >> 8) & 0xFF),
        (uint8_t)(val & 0xFF)
    };
    kiss_send(cmd, buf, 4);
}

// ---------------------------------------------------------------------------
// DIO1 ISR — packet received or TX done
// ---------------------------------------------------------------------------
static void dio1_isr() {
    dio1_flag = true;
}

// ---------------------------------------------------------------------------
// Radio init — reuses proven sequence from MeshCore target.cpp
// ---------------------------------------------------------------------------
static bool init_radio() {
    spi.begin();

    float tcxo = SX126X_DIO3_TCXO_VOLTAGE;

    // begin() with defaults — will be overridden by host commands
    // Use 868.0 MHz, 125kHz BW, SF7, CR5 as safe initial values
    int status = radio.begin(868.0, 125.0, 7, 5,
                             RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 2, 16, tcxo);

    if (status == RADIOLIB_ERR_SPI_CMD_FAILED || status == RADIOLIB_ERR_SPI_CMD_INVALID) {
        // TCXO fallback: try without
        status = radio.begin(868.0, 125.0, 7, 5,
                             RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 2, 16, 0.0f);
    }

    if (status != RADIOLIB_ERR_NONE) {
        return false;
    }

    radio.setCRC(1);
    radio.setRxBoostedGainMode(true);
    radio.setRfSwitchPins(SX126X_RXEN, SX126X_TXEN);
    radio.setDio1Action(dio1_isr);

    return true;
}

// ---------------------------------------------------------------------------
// Start continuous RX
// ---------------------------------------------------------------------------
static void start_rx() {
    radio.startReceive();
    rx_active = true;
}

// ---------------------------------------------------------------------------
// Handle received packet
// ---------------------------------------------------------------------------
static void handle_rx_packet() {
    size_t len = radio.getPacketLength();
    if (len == 0 || len > HW_MTU) {
        start_rx();
        return;
    }

    uint8_t buf[HW_MTU];
    int status = radio.readData(buf, len);
    if (status != RADIOLIB_ERR_NONE) {
        start_rx();
        return;
    }

    // Report RSSI (value + offset)
    float rssi = radio.getRSSI();
    int rssi_byte = (int)rssi + RSSI_OFFSET;
    if (rssi_byte < 0) rssi_byte = 0;
    if (rssi_byte > 255) rssi_byte = 255;
    kiss_send_byte(CMD_STAT_RSSI, (uint8_t)rssi_byte);

    // Report SNR (signed, quarter-dB units)
    float snr = radio.getSNR();
    int8_t snr_byte = (int8_t)(snr * 4.0f);
    kiss_send_byte(CMD_STAT_SNR, (uint8_t)snr_byte);

    // Send packet data
    kiss_send(CMD_DATA, buf, len);

    // Return to RX
    start_rx();
}

// ---------------------------------------------------------------------------
// Handle transmitted packet complete
// ---------------------------------------------------------------------------
static void handle_tx_done() {
    transmitting = false;

    // Signal ready for next packet
    kiss_send_byte(CMD_READY, 0x01);

    // Return to RX if radio is on
    if (cfg_state == RADIO_STATE_ON) {
        start_rx();
    }
}

// ---------------------------------------------------------------------------
// KISS frame handler — called when a complete frame arrives from host
// ---------------------------------------------------------------------------
static void on_kiss_frame(uint8_t cmd, const uint8_t *data, size_t len) {
    switch (cmd) {

    case CMD_DETECT:
        if (len >= 1 && data[0] == DETECT_REQ) {
            kiss_send_byte(CMD_DETECT, DETECT_RESP);
        }
        break;

    case CMD_FW_VERSION: {
        uint8_t ver[2] = { FW_MAJ, FW_MIN };
        kiss_send(CMD_FW_VERSION, ver, 2);
        break;
    }

    case CMD_PLATFORM:
        kiss_send_byte(CMD_PLATFORM, PRODUCT_HMBRW);
        break;

    case CMD_MCU:
        kiss_send_byte(CMD_MCU, MCU_STM32);
        break;

    case CMD_FREQUENCY:
        if (len >= 4) {
            cfg_frequency = ((uint32_t)data[0] << 24) |
                            ((uint32_t)data[1] << 16) |
                            ((uint32_t)data[2] << 8)  |
                            (uint32_t)data[3];
            if (radio_ready) {
                radio.setFrequency((float)cfg_frequency / 1000000.0f);
            }
            kiss_send_u32(CMD_FREQUENCY, cfg_frequency);
        }
        break;

    case CMD_BANDWIDTH:
        if (len >= 4) {
            cfg_bandwidth = ((uint32_t)data[0] << 24) |
                            ((uint32_t)data[1] << 16) |
                            ((uint32_t)data[2] << 8)  |
                            (uint32_t)data[3];
            if (radio_ready) {
                radio.setBandwidth((float)cfg_bandwidth / 1000.0f);
            }
            kiss_send_u32(CMD_BANDWIDTH, cfg_bandwidth);
        }
        break;

    case CMD_TXPOWER:
        if (len >= 1) {
            cfg_txpower = data[0];
            if (radio_ready) {
                radio.setOutputPower((int8_t)cfg_txpower);
            }
            kiss_send_byte(CMD_TXPOWER, cfg_txpower);
        }
        break;

    case CMD_SF:
        if (len >= 1) {
            cfg_sf = data[0];
            if (radio_ready) {
                radio.setSpreadingFactor(cfg_sf);
            }
            kiss_send_byte(CMD_SF, cfg_sf);
        }
        break;

    case CMD_CR:
        if (len >= 1) {
            cfg_cr = data[0];
            if (radio_ready) {
                radio.setCodingRate(cfg_cr);
            }
            kiss_send_byte(CMD_CR, cfg_cr);
        }
        break;

    case CMD_RADIO_STATE:
        if (len >= 1) {
            cfg_state = data[0];
            if (cfg_state == RADIO_STATE_ON && radio_ready) {
                start_rx();
            } else {
                radio.standby();
                rx_active = false;
            }
            kiss_send_byte(CMD_RADIO_STATE, cfg_state);
        }
        break;

    case CMD_ST_ALOCK:
        if (len >= 2) {
            kiss_send(CMD_ST_ALOCK, data, 2);
        }
        break;

    case CMD_LT_ALOCK:
        if (len >= 2) {
            kiss_send(CMD_LT_ALOCK, data, 2);
        }
        break;

    case CMD_DATA:
        if (len > 0 && len <= HW_MTU && radio_ready) {
            transmitting = true;
            rx_active = false;
            int status = radio.transmit((uint8_t *)data, len);
            if (status != RADIOLIB_ERR_NONE) {
                kiss_send_byte(CMD_ERROR, ERROR_TXFAILED);
                transmitting = false;
                if (cfg_state == RADIO_STATE_ON) {
                    start_rx();
                }
            } else {
                handle_tx_done();
            }
        }
        break;

    case CMD_RANDOM: {
        if (radio_ready) {
            uint8_t r = (uint8_t)(radio.random(256) & 0xFF);
            kiss_send_byte(CMD_RANDOM, r);
        }
        break;
    }

    case CMD_BLINK:
        // Blink TX LED briefly
        digitalWrite(LED_TX, HIGH);
        delay(100);
        digitalWrite(LED_TX, LOW);
        break;

    case CMD_RESET:
        NVIC_SystemReset();
        break;

    case CMD_LEAVE:
        cfg_state = RADIO_STATE_OFF;
        radio.standby();
        rx_active = false;
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    // LED pins
    pinMode(LED_TX, OUTPUT);
    pinMode(LED_RX, OUTPUT);
    digitalWrite(LED_TX, LOW);
    digitalWrite(LED_RX, LOW);

    // USB CDC
    Serial.begin(115200);

    // Init radio
    radio_ready = init_radio();
    if (!radio_ready) {
        // Blink both LEDs to indicate radio init failure
        while (true) {
            digitalWrite(LED_TX, HIGH);
            digitalWrite(LED_RX, HIGH);
            delay(200);
            digitalWrite(LED_TX, LOW);
            digitalWrite(LED_RX, LOW);
            delay(200);
        }
    }

    // Init KISS parser
    kiss_init(&parser, on_kiss_frame);

    // Brief LED flash to indicate ready
    digitalWrite(LED_TX, HIGH);
    delay(100);
    digitalWrite(LED_TX, LOW);
}

// ---------------------------------------------------------------------------
// Arduino main loop
// ---------------------------------------------------------------------------
void loop() {
    // Process incoming USB data
    while (Serial.available()) {
        kiss_process_byte(&parser, (uint8_t)Serial.read());
    }

    // Check DIO1 interrupt flag
    if (dio1_flag) {
        dio1_flag = false;
        if (rx_active && !transmitting) {
            handle_rx_packet();
        }
    }
}
