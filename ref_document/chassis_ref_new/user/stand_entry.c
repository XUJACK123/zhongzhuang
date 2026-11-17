#include "stand_entry.h"
#include "robot_config.h"
#include <string.h>

void stand_entry_step(StandEntry *s, uint32_t now, bool active, bool eligible) {
    /* Once balancing, normal chassis faults own stopping; pose may change to drive. */
    if (active) return;
    if (s->tracking && now-s->last_tick>ROBOT_CONTROL_MS+1U)
        memset(s,0,sizeof(*s));
    if (!eligible) { memset(s,0,sizeof(*s)); return; }
    if (!s->tracking) { s->tracking=1; s->since=now; }
    s->last_tick=now;
    uint32_t elapsed=now-s->since;
    s->stable_ms=elapsed>ROBOT_STAND_ENTRY_DWELL_MS ? ROBOT_STAND_ENTRY_DWELL_MS : elapsed;
    s->ready=elapsed>=ROBOT_STAND_ENTRY_DWELL_MS;
}
