#ifndef DIAG_CAN_H
#define DIAG_CAN_H
#include <stdint.h>
typedef struct {
    uint32_t groups_tx,fc_tx,fd_tx,active_groups_tx,active_fc_tx,active_gap_max_us,event_loss_flags_seen;
    uint32_t foreign_group_rx,foreign_dji_rx;
    uint8_t last_foreign_group[8],last_foreign_dji[8];
} DiagCanMonitor;
extern volatile DiagCanMonitor g_diag_can;
#endif
