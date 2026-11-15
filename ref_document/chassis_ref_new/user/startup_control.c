#include "startup_control.h"
#include <string.h>
void startup_step(StartupControl *s, uint32_t now, bool active, bool ready, bool balance_safe, bool actor_available) {
    if (!active) { memset(s,0,sizeof(*s)); return; }
    if (s->phase==STARTUP_FAILED) return;
    if (s->phase==STARTUP_STOP) {
        s->since=now;
        if (ready) { s->phase=STARTUP_BALANCE; s->balance_since=now; }
        else if (actor_available) s->phase=STARTUP_GETUP;
        else { s->failure=STARTUP_MODEL_MISSING; s->phase=STARTUP_FAILED; }
    }
    if (s->phase==STARTUP_GETUP) {
        if (now-s->since>5000) { s->phase=STARTUP_FAILED; s->failure=STARTUP_TIMEOUT; }
        else if (ready) {
            if (!s->stable) { s->stable=true; s->stable_since=now; }
            if (now-s->stable_since>=1000) { s->phase=STARTUP_HANDOFF; s->handoff_since=now; }
        } else s->stable=false;
    }
    if (s->phase==STARTUP_HANDOFF) {
        if (!ready || !balance_safe) { s->phase=STARTUP_FAILED; s->failure=STARTUP_UNSTABLE; }
        else {
            float u=(float)(now-s->handoff_since)/300;
            if (u>=1) { s->phase=STARTUP_BALANCE; s->balance_since=now; }
            else s->balance_weight=u*u*(3-2*u);
        }
    }
    if (s->phase==STARTUP_BALANCE) {
        s->balance_weight=1;
        s->completed=now-s->balance_since>=10000;
        if (!balance_safe) { s->phase=STARTUP_FAILED; s->failure=STARTUP_UNSTABLE; }
    }
    if (s->phase==STARTUP_FAILED) { s->torque_scale=0; s->balance_weight=0; s->completed=false; return; }
    s->torque_scale=(float)(now-s->since)/200;
    if (s->torque_scale>1) s->torque_scale=1;
}
