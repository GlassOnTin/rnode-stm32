#include "kiss.h"

void kiss_init(KissParser *p, kiss_frame_cb cb) {
    p->state = KISS_WAIT_FEND;
    p->cmd = 0;
    p->pos = 0;
    p->callback = cb;
}

void kiss_process_byte(KissParser *p, uint8_t b) {
    switch (p->state) {
    case KISS_WAIT_FEND:
        if (b == FEND) {
            p->state = KISS_WAIT_CMD;
        }
        break;

    case KISS_WAIT_CMD:
        if (b == FEND) {
            // consecutive FENDs, stay in this state
            break;
        }
        p->cmd = b;
        p->pos = 0;
        p->state = KISS_IN_FRAME;
        break;

    case KISS_IN_FRAME:
        if (b == FEND) {
            // frame complete
            if (p->callback) {
                p->callback(p->cmd, p->buf, p->pos);
            }
            p->state = KISS_WAIT_CMD;
        } else if (b == FESC) {
            p->state = KISS_ESCAPE;
        } else {
            if (p->pos < KISS_BUF_SIZE) {
                p->buf[p->pos++] = b;
            }
        }
        break;

    case KISS_ESCAPE:
        if (b == TFEND) {
            if (p->pos < KISS_BUF_SIZE) {
                p->buf[p->pos++] = FEND;
            }
        } else if (b == TFESC) {
            if (p->pos < KISS_BUF_SIZE) {
                p->buf[p->pos++] = FESC;
            }
        }
        // any other byte after FESC is a protocol error; drop it
        p->state = KISS_IN_FRAME;
        break;
    }
}

size_t kiss_build_frame(uint8_t *out, size_t out_size,
                        uint8_t cmd, const uint8_t *data, size_t len) {
    size_t i = 0;

    if (out_size < 3) return 0;  // minimum: FEND CMD FEND

    out[i++] = FEND;
    out[i++] = cmd;

    for (size_t j = 0; j < len && i < out_size - 1; j++) {
        uint8_t b = data[j];
        if (b == FEND) {
            if (i + 2 > out_size - 1) break;
            out[i++] = FESC;
            out[i++] = TFEND;
        } else if (b == FESC) {
            if (i + 2 > out_size - 1) break;
            out[i++] = FESC;
            out[i++] = TFESC;
        } else {
            out[i++] = b;
        }
    }

    out[i++] = FEND;
    return i;
}
