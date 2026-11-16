#include "limited_current.h"
#include <math.h>
#include <string.h>
void ident_stop(LimitedCurrent *c, uint32_t reason) {
    c->phase=DONE; c->reason=reason; c->down_seen=false;
}
void ident_step(LimitedCurrent *c, const IdentInput *in, float *current_a) {
    *current_a=0;
    bool remote=in->remote_valid && in->remote_age<=100 && in->s1>=1 && in->s1<=3;
    bool active=c->phase==ENABLING || c->phase==RUNNING || c->phase==COOLDOWN;
    if (!remote || in->s1==2) {
        if (active) ident_stop(c,REMOTE_STOP);
        if (remote && in->s1==2) c->down_seen=true;
        if (!remote) c->down_seen=false;
    }
    bool finite=true;
    for (unsigned i=0;i<6;++i) finite=finite && isfinite(in->arm[i]) && isfinite(in->velocity[i]);
    if (!finite || !in->healthy) {
        if (c->phase==ARMED || active) ident_stop(c,finite ? FEEDBACK_STOP : INVALID_INPUT);
        return;
    }
    if ((c->phase==IDLE || c->phase==DONE) && remote && in->s1==2 &&
        c->down_seen && in->token && in->token!=c->last_token) {
        c->last_token=in->token; c->phase=ARMED; c->reason=REASON_NONE;
        c->trial_index=0;
        memcpy(c->origin,in->arm,sizeof(c->origin));
        return;
    }
    if (c->phase==ARMED && remote && in->s1!=2) {
        if (!c->down_seen) return;
        memcpy(c->origin,in->arm,sizeof(c->origin)); /* reference the trigger, not minutes of operator wait */
        c->phase=ENABLING; c->since=in->now;
    }
    if (c->phase!=ENABLING && c->phase!=RUNNING && c->phase!=COOLDOWN) return;
    for (unsigned i=0;i<6;++i) {
        bool wheel=i==2 || i==5;
        float delta_limit=wheel ? IDENT_WHEEL_MAX_DELTA_RAD : IDENT_MAX_DELTA_RAD;
        float speed_limit=wheel ? IDENT_WHEEL_MAX_SPEED_RAD_S : IDENT_MAX_SPEED_RAD_S;
        if (fabsf(in->arm[i]-c->origin[i])>delta_limit ||
            fabsf(in->velocity[i])>speed_limit) {
            ident_stop(c,MOVEMENT_STOP); return;
        }
    }
    if (c->phase==COOLDOWN) {
        if ((uint32_t)(in->now-c->since)<IDENT_COOLDOWN_MS) return;
        c->phase=ENABLING;c->since=in->now;
    }
    if (c->phase==ENABLING) {
        if ((uint32_t)(in->now-c->since)<IDENT_ENABLE_MS) return;
        c->phase=RUNNING; c->since=in->now;
    }
    uint32_t elapsed=in->now-c->since;
    if (elapsed>=IDENT_RUN_MS) {
        if (++c->trial_index<IDENT_TRIAL_COUNT) {c->phase=COOLDOWN;c->since=in->now;}
        else ident_stop(c,COMPLETE);
        return;
    }
    const float levels[IDENT_TRIAL_COUNT]=IDENT_CURRENT_LEVELS_A;
    if (c->trial_index>=IDENT_TRIAL_COUNT) {ident_stop(c,INVALID_INPUT);return;}
    *current_a=IDENT_CURRENT_DIRECTION*levels[c->trial_index]*(float)elapsed/(float)IDENT_RUN_MS;
}
