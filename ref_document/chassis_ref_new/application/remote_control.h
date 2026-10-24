#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_protocol.h"

#define RC_FRAME_LENGTH 18U
#define RC_RX_BUFFER_LENGTH RC_FRAME_LENGTH

#define RC_SWITCH_UP 1U
#define RC_SWITCH_DOWN 2U
#define RC_SWITCH_MID 3U

typedef struct
{
    chassis_dbus_frame_t frame;
    uint32_t update_time_ms;
    bool valid;
} remote_control_snapshot_t;

void remote_control_init(void);
void remote_control_get_snapshot(remote_control_snapshot_t *snapshot);

#endif
