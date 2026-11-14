#include "control_state.h"
#include "robot_config.h"
/* Minimal operator interface: the lowest S1 position is zero current for every
 * mode. Any other valid S1 position runs once S1 has been seen down after boot,
 * a remote loss or a fault. Hardware health still forces FAULT. */
void control_state_step(ControlState *s, uint32_t now, bool remote_ok, uint8_t s1,
                        uint8_t s2, bool centered, bool healthy, bool enabled) {
    (void)s2; (void)centered;
    if (s1==2) { s->mode=CONTROL_STOP; s->down_seen=true; return; }
    if (!remote_ok) { s->mode=CONTROL_STOP; s->down_seen=false; return; }
    if (!healthy) { if (s->mode!=CONTROL_STOP) s->mode=CONTROL_FAULT; s->down_seen=false; return; }
    if (s->mode==CONTROL_FAULT) return;
    if (!s->down_seen) { s->mode=CONTROL_STOP; return; }
    if (s->mode!=CONTROL_ENABLING && s->mode!=CONTROL_ACTIVE) {
        s->mode=CONTROL_ENABLING; s->since=now; return;
    }
    if (s->mode==CONTROL_ENABLING) {
        if (enabled) s->mode=CONTROL_ACTIVE;
        else if (now-s->since>ROBOT_ENABLE_TIMEOUT_MS) s->mode=CONTROL_FAULT;
    } else if (!enabled) s->mode=CONTROL_FAULT;
}
