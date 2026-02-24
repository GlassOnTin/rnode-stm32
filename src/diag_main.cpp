// Standalone RX diagnostic firmware for E22P.
// Tests blocking receive, then continuous monitoring.

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

class DiagSX1262 : public SX1262 {
public:
    DiagSX1262(Module* mod) : SX1262(mod) {}
    uint8_t diagGetStatus() { return getStatus(); }
    uint16_t diagGetDeviceErrors() { return getDeviceErrors(); }
    void diagClearIrq() { clearIrqStatus(); }
};

static SPIClass spi(P_LORA_MOSI, P_LORA_MISO, P_LORA_SCLK);
static DiagSX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);

static volatile uint32_t dio1_count = 0;
static volatile bool dio1_flag = false;
static void dio1_isr() { dio1_count++; dio1_flag = true; }

static uint32_t rx_count = 0;
static uint32_t last_print = 0;

// Boot status
static int boot_begin_status = -999;
static float boot_rssi_sb = 0, boot_rssi_rx = 0;
static int blocking_rx_result = -999;

void setup() {
    pinMode(LED_TX, OUTPUT);
    pinMode(LED_RX, OUTPUT);
    digitalWrite(LED_TX, LOW);
    digitalWrite(LED_RX, LOW);

    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("=== E22P RX Diagnostic v5 ===");

    spi.begin();

    float tcxo = SX126X_DIO3_TCXO_VOLTAGE;
    int status = radio.begin(868.0, 125.0, 7, 5,
                             RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 2, 16, tcxo);
    Serial.print("begin(tcxo=");
    Serial.print(tcxo, 1);
    Serial.print("): ");
    Serial.println(status);

    if (status == RADIOLIB_ERR_SPI_CMD_FAILED || status == RADIOLIB_ERR_SPI_CMD_INVALID) {
        Serial.println("TCXO fail, retrying without...");
        status = radio.begin(868.0, 125.0, 7, 5,
                             RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 2, 16, 0.0f);
        Serial.print("begin(tcxo=0): ");
        Serial.println(status);
    }
    boot_begin_status = status;

    if (status != RADIOLIB_ERR_NONE) {
        Serial.println("RADIO INIT FAILED");
        while (true) { digitalWrite(LED_TX, !digitalRead(LED_TX)); delay(200); }
    }

    radio.setCRC(1);
    radio.setRxBoostedGainMode(true);
    radio.setRfSwitchPins(SX126X_RXEN, SX126X_TXEN);
    radio.setDio1Action(dio1_isr);

    // === Mimic main firmware init: radio.random() calls ===
    uint16_t prng_s = (uint16_t)(radio.random(256) | (radio.random(256) << 8));
    Serial.print("random seed: 0x");
    Serial.println(prng_s, HEX);

    // === Mimic main firmware: reconfigure params via setters (like KISS commands) ===
    radio.setFrequency(868.0);
    radio.setBandwidth(125.0);
    radio.setSpreadingFactor(7);
    radio.setCodingRate(5);
    radio.setOutputPower(14);
    Serial.println("Re-applied params via setters (like KISS)");

    // Verify mode transitions via RSSI
    boot_rssi_sb = radio.getRSSI(false);
    Serial.print("RSSI standby: ");
    Serial.println(boot_rssi_sb, 1);

    // === Enter continuous RX using no-arg startReceive (like main firmware) ===
    Serial.println();
    Serial.println("=== Phase A: startReceive() no args (main FW style) ===");

    dio1_count = 0;
    int16_t rxst = radio.startReceive();
    Serial.print("startReceive(): ");
    Serial.println(rxst);
    delay(100);
    boot_rssi_rx = radio.getRSSI(false);
    Serial.print("RSSI in RX: ");
    Serial.println(boot_rssi_rx, 1);

    Serial.println();
    Serial.println("sec  | rssi    | irq    | dio1 | rx | usb    | RXEN TXEN DIO1");
    Serial.println("-----|---------|--------|------|----|--------|---------------");

    last_print = millis();
}

// USB serial stats
static uint32_t usb_bytes_rx = 0;
static uint32_t usb_reads = 0;

void loop() {
    uint32_t now = millis();

    // === Mimic main firmware: read USB serial (same as KISS loop) ===
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        usb_bytes_rx++;
        usb_reads++;
        (void)b;  // discard — we just want the USB read activity
    }

    if (dio1_flag) {
        dio1_flag = false;
        uint16_t irq = (uint16_t)radio.getIrqFlags();
        if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {
            size_t len = radio.getPacketLength();
            if (len > 0 && len <= 508) {
                uint8_t buf[508];
                radio.readData(buf, len);
                rx_count++;
                digitalWrite(LED_RX, HIGH);
                Serial.print(">>> PKT #");
                Serial.print(rx_count);
                Serial.print(" len=");
                Serial.print(len);
                Serial.print(" rssi=");
                Serial.print(radio.getRSSI(), 1);
                Serial.print(" snr=");
                Serial.println(radio.getSNR(), 1);
            }
            radio.startReceive();
        } else {
            Serial.print(">>> DIO1 irq=0x");
            Serial.println(irq, HEX);
        }
    }

    // LED auto-off
    static uint32_t led_on_ms = 0;
    if (digitalRead(LED_RX)) {
        if (led_on_ms == 0) led_on_ms = now;
        if (now - led_on_ms >= 50) { digitalWrite(LED_RX, LOW); led_on_ms = 0; }
    }

    // Fast RSSI sampling
    static float rssi_min = 0, rssi_max = -200;
    static uint32_t rssi_samples = 0;
    static uint32_t last_rssi_ms = 0;
    if (now - last_rssi_ms >= 10) {
        last_rssi_ms = now;
        float r = radio.getRSSI(false);
        if (rssi_samples == 0 || r < rssi_min) rssi_min = r;
        if (rssi_samples == 0 || r > rssi_max) rssi_max = r;
        rssi_samples++;
    }

    // Periodic output
    if (now - last_print >= 1000) {
        last_print = now;
        uint16_t irq = (uint16_t)radio.getIrqFlags();
        radio.diagClearIrq();

        Serial.print(now / 1000);
        Serial.print("    | ");
        Serial.print(rssi_min, 0);
        Serial.print("/");
        Serial.print(rssi_max, 0);
        rssi_min = 0; rssi_max = -200; rssi_samples = 0;
        Serial.print(" | 0x");
        if (irq < 0x1000) Serial.print("0");
        if (irq < 0x100) Serial.print("0");
        if (irq < 0x10) Serial.print("0");
        Serial.print(irq, HEX);
        Serial.print(" | ");
        Serial.print(dio1_count);
        Serial.print("    | ");
        Serial.print(rx_count);
        Serial.print("  | ");
        Serial.print(usb_bytes_rx);
        Serial.print(" | ");
        Serial.print(digitalRead(SX126X_RXEN));
        Serial.print("    ");
        Serial.print(digitalRead(SX126X_TXEN));
        Serial.print("    ");
        Serial.println(digitalRead(P_LORA_DIO_1));

        if (irq) {
            if (irq & RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED) Serial.println("  ^ PREAMBLE");
            if (irq & RADIOLIB_SX126X_IRQ_HEADER_VALID) Serial.println("  ^ HEADER_VALID");
            if (irq & RADIOLIB_SX126X_IRQ_CRC_ERR)      Serial.println("  ^ CRC_ERR");
            if (irq & RADIOLIB_SX126X_IRQ_RX_DONE)       Serial.println("  ^ RX_DONE");
            if (irq & RADIOLIB_SX126X_IRQ_TIMEOUT)        Serial.println("  ^ TIMEOUT");
        }

        // Every 15s, print summary
        if ((now / 1000) % 15 == 0) {
            Serial.print("  INFO: rx=");
            Serial.print(rx_count);
            Serial.print(" usb_bytes=");
            Serial.print(usb_bytes_rx);
            Serial.print(" usb_reads=");
            Serial.println(usb_reads);
        }
    }
}
