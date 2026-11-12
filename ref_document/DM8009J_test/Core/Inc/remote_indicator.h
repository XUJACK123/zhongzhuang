#ifndef REMOTE_INDICATOR_H
#define REMOTE_INDICATOR_H
#include "dbus_protocol.h"
#include "dm8009_step_config.h"
/* Low-brightness GRB. Blue also denotes mid/mismatched switches: no permission.
 * LED only reports remote state, never motor readiness or successful movement. */
#define REMOTE_LED_GREEN 0x100000U
#define REMOTE_LED_BLUE  0x000010U
#define REMOTE_LED_RED   0x001000U
static inline uint32_t DM_RemoteColor(const DBUS_State *r, uint32_t now) {
    if (!r->valid || now-r->last_rx_ms > DM_RX_TIMEOUT_MS) return REMOTE_LED_GREEN;
    if (r->right_switch==DBUS_UP && r->left_switch==DBUS_UP) return REMOTE_LED_RED;
    return REMOTE_LED_BLUE;
}
#endif
