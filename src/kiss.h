#pragma once
#include <stdint.h>
#include <stddef.h>

// KISS framing
#define FEND  0xC0
#define FESC  0xDB
#define TFEND 0xDC
#define TFESC 0xDD

// KISS commands
#define CMD_DATA        0x00
#define CMD_FREQUENCY   0x01
#define CMD_BANDWIDTH   0x02
#define CMD_TXPOWER     0x03
#define CMD_SF          0x04
#define CMD_CR          0x05
#define CMD_RADIO_STATE 0x06
#define CMD_RADIO_LOCK  0x07
#define CMD_DETECT      0x08
#define CMD_LEAVE       0x0A
#define CMD_ST_ALOCK    0x0B
#define CMD_LT_ALOCK    0x0C
#define CMD_READY       0x0F
#define CMD_STAT_RX     0x21
#define CMD_STAT_TX     0x22
#define CMD_STAT_RSSI   0x23
#define CMD_STAT_SNR    0x24
#define CMD_STAT_CHTM   0x25
#define CMD_STAT_PHYPRM 0x26
#define CMD_STAT_BAT    0x27
#define CMD_STAT_CSMA   0x28
#define CMD_STAT_TEMP   0x29
#define CMD_BLINK       0x30
#define CMD_RANDOM      0x40
#define CMD_FB_EXT      0x41
#define CMD_FB_READ     0x42
#define CMD_FB_WRITE    0x43
#define CMD_DISP_READ   0x66
#define CMD_PLATFORM    0x48
#define CMD_MCU         0x49
#define CMD_FW_VERSION  0x50
#define CMD_ROM_READ    0x51
#define CMD_RESET       0x55
#define CMD_ERROR       0x90

// Detection
#define DETECT_REQ  0x73
#define DETECT_RESP 0x46

// Radio state
#define RADIO_STATE_OFF 0x00
#define RADIO_STATE_ON  0x01

// Platform/MCU identifiers
#define PRODUCT_HMBRW 0xF0
#define MCU_STM32     0x81

// Firmware version
#define FW_MAJ 1
#define FW_MIN 62

// Error codes
#define ERROR_INITRADIO     0x01
#define ERROR_TXFAILED      0x02
#define ERROR_QUEUE_FULL    0x04

// Protocol limits
#define HW_MTU       508
#define RSSI_OFFSET  157
#define KISS_BUF_SIZE 600  // HW_MTU + overhead for escaping

// KISS frame parser states
enum KissState {
    KISS_WAIT_FEND,
    KISS_WAIT_CMD,
    KISS_IN_FRAME,
    KISS_ESCAPE
};

// Parsed KISS frame callback
typedef void (*kiss_frame_cb)(uint8_t cmd, const uint8_t *data, size_t len);

// Parser state
struct KissParser {
    KissState state;
    uint8_t cmd;
    uint8_t buf[KISS_BUF_SIZE];
    size_t pos;
    kiss_frame_cb callback;
};

void kiss_init(KissParser *p, kiss_frame_cb cb);
void kiss_process_byte(KissParser *p, uint8_t b);

// Frame builder — writes KISS-escaped frame to output buffer.
// Returns number of bytes written.
size_t kiss_build_frame(uint8_t *out, size_t out_size,
                        uint8_t cmd, const uint8_t *data, size_t len);
