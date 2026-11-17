#ifndef STAND_ENTRY_H
#define STAND_ENTRY_H
#include <stdbool.h>
#include <stdint.h>

/* Separate debug symbol: do not change the existing ChassisDebug/RAM ABI. */
typedef struct {
    uint32_t since, stable_ms, tracking, ready, last_tick;
} StandEntry;
void stand_entry_step(StandEntry *s, uint32_t now, bool active, bool eligible);
#endif
