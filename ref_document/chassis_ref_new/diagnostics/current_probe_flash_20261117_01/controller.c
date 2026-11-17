#include "controller.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>
volatile ProbeConfig g_probe_config={4,0.5f};
static const float signs[6]=ROBOT_ENCODER_SIGNS;
bool probe_configuration_valid(void) {
    uint32_t i=g_probe_config.index; float nm=g_probe_config.raw_nominal_nm;
    return (i==0 || i==1 || i==3 || i==4) && isfinite(nm) && fabsf(nm)>=0.1f && fabsf(nm)<=PROBE_CAP_NM;
}
void retraction_begin(Retraction *s,uint32_t now,const float q[6]) {
    memset(s,0,sizeof(*s));s->started=now;
    memcpy(s->origin,q,sizeof(s->origin));memcpy(s->target,q,sizeof(s->target));
    if (!probe_configuration_valid()) {s->reason=4;return;}
    s->index=g_probe_config.index;s->raw_nominal_nm=g_probe_config.raw_nominal_nm;s->running=1;
}
bool probe_guard(Retraction *s,const float q[6],const float qd[6]) {
    for(unsigned i=0;i<6;++i) {
        bool wheel=i==2 || i==5;
        if(!isfinite(q[i]) || !isfinite(qd[i])) s->reason=4;
        else if(fabsf(q[i]-s->origin[i])>(wheel ? 0.087266463f : PROBE_TRAVEL_RAD)) s->reason=2;
        else if(fabsf(qd[i])>(wheel ? 1.0f : PROBE_SPEED_RAD_S)) s->reason=3;
    }
    if(s->reason) {s->running=0;memset(s->torque,0,sizeof(s->torque));return false;}
    return true;
}
bool retraction_step(Retraction *s,uint32_t now,const float q[6],const float qd[6],float dt) {
    if(!s->running) return false;
    if(!isfinite(dt) || dt<=0 || dt>0.0031f) s->reason=7;
    if(!probe_guard(s,q,qd)) return false;
    uint32_t elapsed=now-s->started;
    if(elapsed>=RETRACT_TOTAL_MS) {s->reason=1;s->running=0;memset(s->torque,0,sizeof(s->torque));return false;}
    float fraction=1;
    if(elapsed<RETRACT_RAMP_MS) fraction=(float)elapsed/RETRACT_RAMP_MS;
    else if(elapsed>=RETRACT_RAMP_MS+RETRACT_HOLD_MS) fraction=1-(float)(elapsed-RETRACT_RAMP_MS-RETRACT_HOLD_MS)/RETRACT_RELEASE_MS;
    memset(s->torque,0,sizeof(s->torque));
    /* motor_encode multiplies by signs again: the requested polarity is the
     * raw DM current polarity, not a possibly wrong joint-space convention. */
    s->torque[s->index]=signs[s->index]*s->raw_nominal_nm*fraction;
    return true;
}
