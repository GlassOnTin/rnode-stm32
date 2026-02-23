#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "kiss.h"

extern "C" void CDC_reset_transmit(void);

// ---------------------------------------------------------------------------
// Hardware setup
// ---------------------------------------------------------------------------
static SPIClass spi(P_LORA_MOSI, P_LORA_MISO, P_LORA_SCLK);
static SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);

// ---------------------------------------------------------------------------
// CSMA constants
// ---------------------------------------------------------------------------
#define CSMA_P           128     // p=0.5 out of 256
#define CSMA_MAX_RETRIES 32
#define CSMA_RSSI_FLOOR  (-90)   // default threshold dBm
#define CSMA_SENSE_MARGIN 15.0f  // dB above measured noise floor = "busy"

// Timing constants
#define ST_WINDOW_MS     15000UL
#define LT_WINDOW_MS     3600000UL
#define CHTM_INTERVAL_MS 1000UL
#define RSSI_SAMPLE_MS   100UL
#define TEMP_INTERVAL_MS 5000UL
#define LED_RX_ON_MS     50UL

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static KissParser parser;
static uint8_t tx_frame_buf[KISS_BUF_SIZE];
static bool radio_ready = false;
static bool rx_active = false;
static volatile bool dio1_flag = false;
static bool transmitting = false;
static bool host_active = false;
static uint32_t last_host_rx_ms = 0;

// Current radio parameters
static uint32_t cfg_frequency = 0;
static uint32_t cfg_bandwidth = 0;
static uint8_t  cfg_txpower   = 0;
static uint8_t  cfg_sf        = 0;
static uint8_t  cfg_cr        = 0;
static uint8_t  cfg_state     = RADIO_STATE_OFF;

// 32-bit magic guard: random corruption is vanishingly unlikely to produce
// this exact value, so we gate all radio-dependent periodic behavior on it
// rather than on cfg_state (which is a single byte easily corrupted to 0x01).
#define RADIO_MAGIC 0xA5B4C3D2UL
static uint32_t radio_on_magic = 0;

// CSMA state — 64-byte guard zone after buffer to absorb any overflow.
// Memory corruption was observed writing to addresses immediately after
// csma_buf, corrupting cfg_state and other radio config variables.
static struct {
    uint8_t data[HW_MTU];
    uint8_t guard[64];
} csma_storage;
#define csma_buf csma_storage.data
static size_t  csma_len = 0;
static bool    csma_pending = false;
static uint32_t csma_next_ms = 0;
static uint8_t  csma_retries = 0;
static uint16_t csma_slot_ms = 10;
static uint16_t csma_difs_ms = 30;

// Airtime tracking
static uint32_t tx_start_ms = 0;
static uint32_t st_window_start = 0;
static uint32_t st_tx_ms = 0;
static uint32_t lt_window_start = 0;
static uint32_t lt_tx_ms = 0;
static uint16_t st_alock_limit = 0;
static uint16_t lt_alock_limit = 0;

// Channel utilisation
static uint32_t ch_st_samples = 0;
static uint32_t ch_st_busy = 0;
static uint32_t ch_lt_samples = 0;
static uint32_t ch_lt_busy = 0;
static float    noise_floor_f = -120.0f;

// Periodic timers
static uint32_t last_chtm_ms = 0;
static uint32_t last_rssi_ms = 0;
static uint32_t last_temp_ms = 0;

// LED RX auto-off
static uint32_t led_rx_off_ms = 0;

// RX diagnostics
static uint32_t diag_dio1_count = 0;
static uint32_t diag_rx_pkt_count = 0;
static uint32_t diag_rx_start_fail = 0;
static uint32_t diag_crc_err_count = 0;
static uint32_t diag_hdr_count = 0;

// Simple PRNG (xorshift16)
static uint16_t prng_s = 1;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline float csma_threshold() {
    return noise_floor_f + CSMA_SENSE_MARGIN;
}

static uint8_t prng_next() {
    prng_s ^= prng_s << 7;
    prng_s ^= prng_s >> 9;
    prng_s ^= prng_s << 8;
    return (uint8_t)(prng_s & 0xFF);
}

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
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    kiss_send(cmd, buf, 2);
}

static void kiss_send_u32(uint8_t cmd, uint32_t val) {
    uint8_t buf[4] = {
        (uint8_t)(val >> 24), (uint8_t)((val >> 16) & 0xFF),
        (uint8_t)((val >> 8) & 0xFF), (uint8_t)(val & 0xFF)
    };
    kiss_send(cmd, buf, 4);
}

// ---------------------------------------------------------------------------
// DIO1 ISR
// ---------------------------------------------------------------------------
static void dio1_isr() {
    dio1_flag = true;
    diag_dio1_count++;
}

// ---------------------------------------------------------------------------
// Radio init
// ---------------------------------------------------------------------------
static bool init_radio() {
    spi.begin();

    float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
    int status = radio.begin(868.0, 125.0, 7, 5,
                             RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 2, 16, tcxo);

    if (status == RADIOLIB_ERR_SPI_CMD_FAILED || status == RADIOLIB_ERR_SPI_CMD_INVALID) {
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
// Compute CSMA timing from radio config
// ---------------------------------------------------------------------------
static void compute_csma_params() {
    if (cfg_sf == 0 || cfg_bandwidth == 0) return;
    // symbol_time_us = 2^SF * 1e6 / BW_Hz
    uint32_t sym_us = ((uint32_t)1 << cfg_sf) * 1000000UL / cfg_bandwidth;
    uint32_t slot = (sym_us + 999) / 1000; // round up to ms
    if (slot < 1) slot = 1;
    if (slot > 1000) slot = 1000;
    csma_slot_ms = (uint16_t)slot;
    csma_difs_ms = csma_slot_ms * 3;
}

// ---------------------------------------------------------------------------
// Send CMD_STAT_PHYPRM
// ---------------------------------------------------------------------------
static void send_phyprm() {
    if (cfg_sf == 0 || cfg_bandwidth == 0) return;

    uint32_t sym_us = ((uint32_t)1 << cfg_sf) * 1000000UL / cfg_bandwidth;
    uint16_t sym_time = (sym_us > 65535) ? 65535 : (uint16_t)sym_us;
    uint16_t sym_rate = (uint16_t)(cfg_bandwidth / ((uint32_t)1 << cfg_sf));
    uint16_t pre_sym  = 16;
    uint32_t pre_us   = (uint32_t)pre_sym * sym_us;
    uint16_t pre_ms   = (uint16_t)((pre_us + 999) / 1000);

    uint8_t buf[12] = {
        (uint8_t)(sym_time >> 8),      (uint8_t)(sym_time & 0xFF),
        (uint8_t)(sym_rate >> 8),      (uint8_t)(sym_rate & 0xFF),
        (uint8_t)(pre_sym >> 8),       (uint8_t)(pre_sym & 0xFF),
        (uint8_t)(pre_ms >> 8),        (uint8_t)(pre_ms & 0xFF),
        (uint8_t)(csma_slot_ms >> 8),  (uint8_t)(csma_slot_ms & 0xFF),
        (uint8_t)(csma_difs_ms >> 8),  (uint8_t)(csma_difs_ms & 0xFF),
    };
    kiss_send(CMD_STAT_PHYPRM, buf, 12);
}

// ---------------------------------------------------------------------------
// Start continuous RX
// ---------------------------------------------------------------------------
static void start_rx() {
    int16_t st = radio.startReceive();
    if (st != RADIOLIB_ERR_NONE) {
        diag_rx_start_fail++;
        rx_active = false;
        return;
    }
    rx_active = true;
}

// ---------------------------------------------------------------------------
// Perform the actual transmit (called from CSMA or direct)
// ---------------------------------------------------------------------------
static void do_transmit() {
    csma_pending = false;
    transmitting = true;
    rx_active = false;
    tx_start_ms = millis();
    digitalWrite(LED_TX, HIGH);

    int status = radio.startTransmit(csma_buf, csma_len);
    if (status != RADIOLIB_ERR_NONE) {
        transmitting = false;
        digitalWrite(LED_TX, LOW);
        kiss_send_byte(CMD_ERROR, ERROR_TXFAILED);
        if (cfg_state == RADIO_STATE_ON) {
            start_rx();
        }
    }
    // On success, DIO1 fires when TX completes
}

// ---------------------------------------------------------------------------
// CSMA check — called from loop()
// ---------------------------------------------------------------------------
static void csma_check() {
    if (!csma_pending || transmitting) return;

    uint32_t now = millis();
    if ((int32_t)(now - csma_next_ms) < 0) return; // not time yet

    // Read instantaneous RSSI while in RX
    float rssi = radio.getRSSI(false);

    if (rssi < csma_threshold()) {
        // Channel clear — p-persistent
        if (prng_next() < CSMA_P) {
            do_transmit();
        } else {
            csma_next_ms = now + csma_slot_ms;
        }
    } else {
        // Channel busy
        csma_retries++;
        if (csma_retries >= CSMA_MAX_RETRIES) {
            do_transmit(); // give up waiting, transmit anyway
        } else {
            csma_next_ms = now + csma_slot_ms;
        }
    }
}

// ---------------------------------------------------------------------------
// Handle received packet
// ---------------------------------------------------------------------------
static void handle_rx_packet() {
    diag_rx_pkt_count++;
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

    // LED RX on briefly
    digitalWrite(LED_RX, HIGH);
    led_rx_off_ms = millis() + LED_RX_ON_MS;

    // Report RSSI
    float rssi = radio.getRSSI();
    int rssi_byte = (int)rssi + RSSI_OFFSET;
    if (rssi_byte < 0) rssi_byte = 0;
    if (rssi_byte > 255) rssi_byte = 255;
    kiss_send_byte(CMD_STAT_RSSI, (uint8_t)rssi_byte);

    // Report SNR
    float snr = radio.getSNR();
    int8_t snr_byte = (int8_t)(snr * 4.0f);
    kiss_send_byte(CMD_STAT_SNR, (uint8_t)snr_byte);

    // Send packet data
    kiss_send(CMD_DATA, buf, len);

    start_rx();
}

// ---------------------------------------------------------------------------
// Handle TX complete (called from DIO1 path)
// ---------------------------------------------------------------------------
static void handle_tx_done() {
    transmitting = false;
    digitalWrite(LED_TX, LOW);

    // Airtime accounting
    uint32_t dur = millis() - tx_start_ms;
    st_tx_ms += dur;
    lt_tx_ms += dur;

    kiss_send_byte(CMD_READY, 0x01);

    if (cfg_state == RADIO_STATE_ON) {
        start_rx();
    }
}

// ---------------------------------------------------------------------------
// Airtime window management
// ---------------------------------------------------------------------------
static void update_airtime_windows(uint32_t now) {
    if (now - st_window_start >= ST_WINDOW_MS) {
        st_tx_ms = 0;
        ch_st_samples = 0;
        ch_st_busy = 0;
        noise_floor_f = -120.0f;
        st_window_start = now;
    }
    if (now - lt_window_start >= LT_WINDOW_MS) {
        lt_tx_ms = 0;
        ch_lt_samples = 0;
        ch_lt_busy = 0;
        lt_window_start = now;
    }
}

// ---------------------------------------------------------------------------
// Check if airtime limit exceeded
// ---------------------------------------------------------------------------
static bool airtime_limited(uint32_t now) {
    if (st_alock_limit > 0) {
        uint32_t elapsed = now - st_window_start;
        if (elapsed > 0 && (st_tx_ms * 10000UL / elapsed) >= st_alock_limit) {
            return true;
        }
    }
    if (lt_alock_limit > 0) {
        uint32_t elapsed = now - lt_window_start;
        if (elapsed > 0 && (lt_tx_ms * 10000UL / elapsed) >= lt_alock_limit) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Sample RSSI for channel utilisation
// ---------------------------------------------------------------------------
static void sample_channel_rssi() {
    if (!rx_active || transmitting || csma_pending) return;

    float rssi = radio.getRSSI(false);

    // Track noise floor: use first sample or lower values
    if (ch_st_samples == 0 || rssi < noise_floor_f) {
        noise_floor_f = rssi;
    }

    bool busy = (rssi >= csma_threshold());
    ch_st_samples++;
    ch_lt_samples++;
    if (busy) {
        ch_st_busy++;
        ch_lt_busy++;
    }
}

// ---------------------------------------------------------------------------
// Send CMD_STAT_CHTM
// ---------------------------------------------------------------------------
static void send_chtm(uint32_t now) {
    uint32_t st_el = now - st_window_start;
    uint32_t lt_el = now - lt_window_start;

    uint16_t air_st = (st_el > 0) ? (uint16_t)(st_tx_ms * 10000UL / st_el) : 0;
    uint16_t air_lt = (lt_el > 0) ? (uint16_t)(lt_tx_ms * 10000UL / lt_el) : 0;
    uint16_t ch_st  = (ch_st_samples > 0) ? (uint16_t)(ch_st_busy * 10000UL / ch_st_samples) : 0;
    uint16_t ch_lt  = (ch_lt_samples > 0) ? (uint16_t)(ch_lt_busy * 10000UL / ch_lt_samples) : 0;

    // Current RSSI + noise floor (only if in RX idle)
    uint8_t cur_rssi  = 0;
    uint8_t noise_byte = 0;
    if (rx_active && !transmitting && !csma_pending) {
        float r = radio.getRSSI(false);
        int rv = (int)r + RSSI_OFFSET;
        cur_rssi = (rv < 0) ? 0 : (rv > 255) ? 255 : (uint8_t)rv;

        int nv = (int)noise_floor_f + RSSI_OFFSET;
        noise_byte = (nv < 0) ? 0 : (nv > 255) ? 255 : (uint8_t)nv;
    }

    uint8_t buf[11] = {
        (uint8_t)(air_st >> 8), (uint8_t)(air_st & 0xFF),
        (uint8_t)(air_lt >> 8), (uint8_t)(air_lt & 0xFF),
        (uint8_t)(ch_st >> 8),  (uint8_t)(ch_st & 0xFF),
        (uint8_t)(ch_lt >> 8),  (uint8_t)(ch_lt & 0xFF),
        cur_rssi, noise_byte, 0xFF
    };
    kiss_send(CMD_STAT_CHTM, buf, 11);
}

// ---------------------------------------------------------------------------
// Read STM32 internal temperature sensor
// ---------------------------------------------------------------------------
static bool temp_init_done = false;
static void read_temperature() {
    if (!temp_init_done) {
        __HAL_RCC_ADC1_CLK_ENABLE();
        // Power on ADC1
        ADC1->CR2 |= ADC_CR2_ADON;
        delay(2);
        // Calibrate
        ADC1->CR2 |= ADC_CR2_CAL;
        while (ADC1->CR2 & ADC_CR2_CAL);
        // Enable temp sensor + vrefint
        ADC1->CR2 |= ADC_CR2_TSVREFE;
        // 239.5 cycle sampling for channel 16
        ADC1->SMPR1 |= (0x7UL << 18);
        delay(2);
        temp_init_done = true;
    }
    // Single conversion on channel 16 (internal temp sensor)
    ADC1->SQR1 = 0;           // 1 conversion in sequence
    ADC1->SQR3 = 16;          // channel 16 first
    ADC1->CR2 |= ADC_CR2_ADON; // start conversion
    while (!(ADC1->SR & ADC_SR_EOC));
    uint32_t raw = ADC1->DR & 0xFFF;

    // STM32F103: V_25=1.43V, slope=4.3mV/C, 12-bit ADC, Vref=3.3V
    uint32_t mv = raw * 3300UL / 4096UL;
    int32_t temp = ((int32_t)1430 - (int32_t)mv) * 10 / 43 + 25;

    int val = (int)(temp + 120);
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    kiss_send_byte(CMD_STAT_TEMP, (uint8_t)val);
}

// ---------------------------------------------------------------------------
// KISS frame handler
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
            radio_on_magic = (cfg_state == RADIO_STATE_ON) ? RADIO_MAGIC : 0;
            if (cfg_state == RADIO_STATE_ON && radio_ready) {
                compute_csma_params();
                start_rx();
                send_phyprm();
            } else {
                radio.standby();
                rx_active = false;
                csma_pending = false;
            }
            kiss_send_byte(CMD_RADIO_STATE, cfg_state);
        }
        break;

    case CMD_ST_ALOCK:
        if (len >= 2) {
            st_alock_limit = ((uint16_t)data[0] << 8) | data[1];
            kiss_send(CMD_ST_ALOCK, data, 2);
        }
        break;

    case CMD_LT_ALOCK:
        if (len >= 2) {
            lt_alock_limit = ((uint16_t)data[0] << 8) | data[1];
            kiss_send(CMD_LT_ALOCK, data, 2);
        }
        break;

    case CMD_DATA:
        if (len > 0 && len <= HW_MTU && radio_ready
            && cfg_state == RADIO_STATE_ON && !transmitting)
        {
            uint32_t now = millis();
            if (airtime_limited(now)) {
                kiss_send_byte(CMD_ERROR, ERROR_QUEUE_FULL);
                break;
            }
            memcpy(csma_buf, data, len);
            csma_len = len;
            csma_pending = true;
            csma_retries = 0;
            csma_next_ms = now + csma_difs_ms;
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
        digitalWrite(LED_TX, HIGH);
        delay(100);
        digitalWrite(LED_TX, LOW);
        break;

    case CMD_RESET:
        NVIC_SystemReset();
        break;

    case CMD_LEAVE:
        cfg_state = RADIO_STATE_OFF;
        radio_on_magic = 0;
        radio.standby();
        rx_active = false;
        csma_pending = false;
        break;

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    pinMode(LED_TX, OUTPUT);
    pinMode(LED_RX, OUTPUT);
    digitalWrite(LED_TX, LOW);
    digitalWrite(LED_RX, LOW);

    Serial.begin(115200);

    radio_ready = init_radio();
    if (!radio_ready) {
        while (true) {
            digitalWrite(LED_TX, HIGH);
            digitalWrite(LED_RX, HIGH);
            delay(200);
            digitalWrite(LED_TX, LOW);
            digitalWrite(LED_RX, LOW);
            delay(200);
        }
    }

    // Seed PRNG from radio noise
    prng_s = (uint16_t)(radio.random(256) | (radio.random(256) << 8));
    if (prng_s == 0) prng_s = 1;

    // Init timers
    uint32_t now = millis();
    st_window_start = now;
    lt_window_start = now;
    last_chtm_ms    = now;
    last_rssi_ms    = now;
    last_temp_ms    = now;

    kiss_init(&parser, on_kiss_frame);
    memset(csma_storage.guard, 0xAA, sizeof(csma_storage.guard));

    // Brief LED flash to indicate ready
    digitalWrite(LED_TX, HIGH);
    delay(100);
    digitalWrite(LED_TX, LOW);
}

// ---------------------------------------------------------------------------
// Arduino main loop
// ---------------------------------------------------------------------------
void loop() {
    // Detect new host session: data arrived after > 2s silence.
    // Reset transmit state BEFORE processing any data so the first
    // response in the new session isn't blocked by stale TxState
    // or queued behind old data.
    if (Serial.available() && millis() - last_host_rx_ms >= 2000UL) {
        CDC_reset_transmit();
        kiss_init(&parser, on_kiss_frame);
        uint32_t rx_now = millis();
        last_chtm_ms = rx_now;
        last_temp_ms = rx_now;
    }

    // Process incoming USB data
    while (Serial.available()) {
        kiss_process_byte(&parser, (uint8_t)Serial.read());
        last_host_rx_ms = millis();
        host_active = true;
    }

    uint32_t now = millis();

    // Consider host gone if no data received for 5 seconds
    if (host_active && (now - last_host_rx_ms >= 5000UL)) {
        host_active = false;
    }

    // If cfg_state got corrupted but radio_on_magic doesn't match,
    // silently fix cfg_state.  The 32-bit magic makes false positives
    // from random corruption vanishingly unlikely.
    if (cfg_state == RADIO_STATE_ON && radio_on_magic != RADIO_MAGIC) {
        cfg_state = RADIO_STATE_OFF;
        radio.standby();
        rx_active = false;
    }

    // DIO1 interrupt: TX complete or RX packet
    if (dio1_flag) {
        dio1_flag = false;
        if (transmitting) {
            handle_tx_done();
        } else if (rx_active) {
            handle_rx_packet();
        }
    }

    // CSMA state machine
    csma_check();

    // LED RX auto-off
    if (led_rx_off_ms && (int32_t)(now - led_rx_off_ms) >= 0) {
        digitalWrite(LED_RX, LOW);
        led_rx_off_ms = 0;
    }

    // Airtime window management
    update_airtime_windows(now);

    // Periodic RSSI sampling for channel utilisation
    if (now - last_rssi_ms >= RSSI_SAMPLE_MS) {
        last_rssi_ms = now;
        sample_channel_rssi();
    }

    // Poll SX1262 IRQ status for CRC errors and headers (not routed to DIO1)
    {
        static uint32_t last_diag_ms = 0;
        if (now - last_diag_ms >= 500UL && rx_active && !transmitting) {
            last_diag_ms = now;
            uint16_t irq = (uint16_t)radio.getIrqFlags();
            if (irq & RADIOLIB_SX126X_IRQ_CRC_ERR) {
                diag_crc_err_count++;
            }
            if (irq & RADIOLIB_SX126X_IRQ_HEADER_VALID) {
                diag_hdr_count++;
            }
        }
    }

    // Periodic channel/airtime report (only when radio legitimately on)
    if (host_active && radio_on_magic == RADIO_MAGIC) {
        if (now - last_chtm_ms >= CHTM_INTERVAL_MS) {
            last_chtm_ms = now;
            send_chtm(now);

            // RX diagnostics report
            // status byte: bit0=rx_active, bit1=transmitting, bit2=csma_pending
            {
                uint8_t sw_status = (rx_active ? 1 : 0)
                                  | (transmitting ? 2 : 0)
                                  | (csma_pending ? 4 : 0);
                uint8_t diag[11];
                diag[0]  = (uint8_t)(diag_dio1_count >> 8);
                diag[1]  = (uint8_t)(diag_dio1_count);
                diag[2]  = (uint8_t)(diag_rx_pkt_count >> 8);
                diag[3]  = (uint8_t)(diag_rx_pkt_count);
                diag[4]  = (uint8_t)(diag_rx_start_fail >> 8);
                diag[5]  = (uint8_t)(diag_rx_start_fail);
                diag[6]  = (uint8_t)(diag_crc_err_count >> 8);
                diag[7]  = (uint8_t)(diag_crc_err_count);
                diag[8]  = (uint8_t)(diag_hdr_count >> 8);
                diag[9]  = (uint8_t)(diag_hdr_count);
                diag[10] = sw_status;
                kiss_send(CMD_STAT_CSMA, diag, 11);
            }
        }
    }

    // Periodic temperature report
    if (host_active && radio_on_magic == RADIO_MAGIC) {
        if (now - last_temp_ms >= TEMP_INTERVAL_MS) {
            last_temp_ms = now;
            read_temperature();
        }
    }
}
