#include "limited_current.h"
#include <math.h>
#include <string.h>
void ident_stop(LimitedCurrent *c, uint32_t reason) {
    c->phase=DONE;c->reason=reason;
}
void ident_step(LimitedCurrent *c, const IdentInput *in, float *current_a) {
    *current_a=0;
    bool pending=c->phase==ARMED || c->phase==ENABLING || c->phase==RUNNING || c->phase==COOLDOWN;
    bool remote=in->remote_valid && in->remote_age<=100 && in->s1>=1 && in->s1<=3;
    /* Down/stale remote consumes and cancels requests, even before arming. */
    if (!remote || in->s1==2) {
        if(in->token)c->last_token=in->token;
        if(pending)ident_stop(c,REMOTE_STOP);
        return;
    }
    bool finite=true;
    for(unsigned i=0;i<6;++i)finite=finite && isfinite(in->arm[i]) && isfinite(in->velocity[i]);
    if(!finite || !in->healthy) {
        if(in->token)c->last_token=in->token;
        if(pending)ident_stop(c,finite ? FEEDBACK_STOP : INVALID_INPUT);
        return;
    }
    if((c->phase==IDLE || c->phase==DONE) && in->token && in->token!=c->last_token) {
        c->last_token=in->token;c->phase=ARMED;c->reason=REASON_NONE;c->since=in->now;c->trial_index=0;
        memcpy(c->origin,in->arm,sizeof(c->origin));
        return;
    }
    if(!pending)return;
    for(unsigned i=0;i<6;++i) {
        bool wheel=i==2 || i==5;
        float delta_limit=wheel ? IDENT_WHEEL_MAX_DELTA_RAD : IDENT_MAX_DELTA_RAD;
        float speed_limit=wheel ? IDENT_WHEEL_MAX_SPEED_RAD_S : IDENT_MAX_SPEED_RAD_S;
        if(fabsf(in->arm[i]-c->origin[i])>delta_limit || fabsf(in->velocity[i])>speed_limit) {
            ident_stop(c,MOVEMENT_STOP);return;
        }
    }
    if(c->phase==ARMED) {
        if((uint32_t)(in->now-c->since)<IDENT_ARM_DELAY_MS)return;
        c->phase=ENABLING;c->since=in->now;
    }
    if(c->phase==COOLDOWN) {
        if((uint32_t)(in->now-c->since)<IDENT_COOLDOWN_MS)return;
        c->phase=ENABLING;c->since=in->now;
    }
    if(c->phase==ENABLING) {
        if((uint32_t)(in->now-c->since)<IDENT_ENABLE_MS)return;
        c->phase=RUNNING;c->since=in->now;
    }
    uint32_t elapsed=in->now-c->since;
    if(elapsed>=IDENT_RUN_MS) {
        if(++c->trial_index<IDENT_TRIAL_COUNT) {c->phase=COOLDOWN;c->since=in->now;}
        else ident_stop(c,COMPLETE);
        return;
    }
    const float levels[IDENT_TRIAL_COUNT]=IDENT_CURRENT_LEVELS_A;
    if(c->trial_index>=IDENT_TRIAL_COUNT) {ident_stop(c,INVALID_INPUT);return;}
    *current_a=IDENT_CURRENT_DIRECTION*levels[c->trial_index]*(float)elapsed/(float)IDENT_RUN_MS;
}
