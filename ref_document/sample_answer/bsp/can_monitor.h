#ifndef CAN_MONITOR_H
#define CAN_MONITOR_H
#include <stdint.h>

typedef struct {
    uint32_t count;
    uint32_t tick_ms;
    uint32_t id;
    uint32_t extended;
    uint32_t remote;
    uint32_t dlc;
    uint8_t data[8];
} CanObservedFrame;

typedef struct {
    uint32_t total;
    uint32_t fifo_overruns;
    uint32_t last_error;
    uint32_t c620_seen_mask;
    CanObservedFrame latest;
    CanObservedFrame c620[8];
    uint32_t standard_id_seen[64];
} CanObservedBus;

extern volatile CanObservedBus can_observed[2];
void CanMonitorInit(void);
void CanMonitorPoll(void);
#endif
