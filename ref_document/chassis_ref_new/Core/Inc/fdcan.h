#ifndef FDCAN_H
#define FDCAN_H
#include "main.h"
#include <stdbool.h>
extern FDCAN_HandleTypeDef hfdcan2;
bool can2_init(void);
bool can2_send(uint16_t id, const uint8_t data[8]);
void can2_cancel_pending(void);
bool can2_healthy(void);
extern volatile uint32_t g_can_tx_fail, g_can_rx_overflow;
typedef struct {
    uint32_t id, received, last_ms;
    uint8_t data[8];
} CanMonitorNode;
typedef struct {
    uint32_t received, other, tx_error, rx_error, bus_off, last_error;
    uint32_t tx_queued, nonzero_torque_frames, enable_commands, status_samples;
    CanMonitorNode node[6];
} CanMonitor;
extern volatile CanMonitor g_can_monitor;
#endif
