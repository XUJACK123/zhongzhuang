#ifndef DBUS_PROTOCOL_H
#define DBUS_PROTOCOL_H
#include <stdint.h>
#define DBUS_UP 1U
#define DBUS_DOWN 2U
#define DBUS_MIDDLE 3U
typedef struct {
    int16_t channel[5];
    uint8_t right_switch, left_switch;
    uint32_t valid, frame_count, last_rx_ms;
} DBUS_State;
static inline int DBUS_Decode(const uint8_t b[18], DBUS_State *s) {
    uint16_t ch[5];
    ch[0] = (b[0] | ((uint16_t)b[1] << 8)) & 0x7FFU;
    ch[1] = ((b[1] >> 3) | ((uint16_t)b[2] << 5)) & 0x7FFU;
    ch[2] = ((b[2] >> 6) | ((uint16_t)b[3] << 2) | ((uint16_t)b[4] << 10)) & 0x7FFU;
    ch[3] = ((b[4] >> 1) | ((uint16_t)b[5] << 7)) & 0x7FFU;
    ch[4] = b[16] | ((uint16_t)b[17] << 8);
    uint8_t right = (b[5] >> 4) & 3U, left = (b[5] >> 6) & 3U;
    if (!right || !left || b[12] > 1U || b[13] > 1U) return 0;
    for (unsigned i = 0; i < 5; ++i) {
        if (ch[i] < 364U || ch[i] > 1684U) return 0;
    }
    for (unsigned i = 0; i < 5; ++i) s->channel[i] = (int16_t)ch[i] - 1024;
    s->right_switch = right;
    s->left_switch = left;
    return 1;
}
#endif
