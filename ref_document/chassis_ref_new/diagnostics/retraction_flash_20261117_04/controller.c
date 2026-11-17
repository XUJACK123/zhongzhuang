#include "controller.h"
#include "algorithm/rl.h"
#include "algorithm/model/model.h"
#include <math.h>
#include <string.h>
void retraction_begin(Retraction *s,uint32_t now,const float q[6]) {
    memset(s,0,sizeof(*s));s->started=now;s->running=1;
    memcpy(s->origin,q,sizeof(s->origin));memcpy(s->target,q,sizeof(s->target));
}
bool retraction_step(Retraction *s,uint32_t now,const float q[6],const float qd[6],float dt) {
    uint32_t elapsed=now-s->started;
    if (!s->running) return false;
    if (!isfinite(dt) || dt<=0 || dt>0.0031f) s->reason=7;
    for (unsigned i=0;i<6;++i) {
        bool wheel=i==2 || i==5;
        if (!isfinite(q[i]) || !isfinite(qd[i])) s->reason=4;
        else if (fabsf(q[i]-s->origin[i])>(wheel ? 0.7f : 0.174532925f)) s->reason=2;
        else if (fabsf(qd[i])>(wheel ? 2.0f : 0.8f)) s->reason=3;
    }
    if (q[1]<rl_physical_knee_limits[0] || q[1]>rl_physical_knee_limits[1] ||
        q[4]<rl_physical_knee_limits[2] || q[4]>rl_physical_knee_limits[3] ||
        s->origin[4]+RETRACT_DELTA_RAD>rl_physical_knee_limits[3]) s->reason=4;
    if (elapsed>=RETRACT_RAMP_MS+RETRACT_HOLD_MS+RETRACT_RELEASE_MS) s->reason=1;
    if (s->reason) {s->running=0;memset(s->torque,0,sizeof(s->torque));return false;}
    float u=fminf(1,(float)elapsed/RETRACT_RAMP_MS);
    /* Smooth target: left knee folds by 5 degrees; left hip holds its entry angle.
     * Right leg and both wheels receive zero active torque. */
    s->target[4]=s->origin[4]+RETRACT_DELTA_RAD*u*u*(3-2*u);
    float joint[6]={0},trial_integral[6]={0},motor[6],speed[6];
    for (unsigned i=3;i<=4;++i) {
        float error=s->target[i]-q[i];
        trial_integral[i]=rl_clip(s->integral[i]+80.0f*error*dt,-25,25);
        joint[i]=300.0f*error-8.0f*qd[i]+trial_integral[i];
    }
    if (!rl_joint_to_motor(q,qd,joint,motor,speed)) {
        s->reason=4;s->running=0;memset(s->torque,0,sizeof(s->torque));return false;
    }
    bool saturated=fabsf(motor[3])>RETRACT_CAP_NM || fabsf(motor[4])>RETRACT_CAP_NM;
    if (!saturated) memcpy(s->integral,trial_integral,sizeof(s->integral));
    bool release=elapsed>=RETRACT_RAMP_MS+RETRACT_HOLD_MS;
    if (release && !s->releasing) {
        memcpy(s->release_torque,s->torque,sizeof(s->torque));s->releasing=1;
    }
    for(unsigned i=0;i<6;++i) {
        float target=(i==3 || i==4) ? rl_clip(motor[i],-RETRACT_CAP_NM,RETRACT_CAP_NM) : 0;
        if (release) s->torque[i]=s->release_torque[i]*(1-(float)(elapsed-RETRACT_RAMP_MS-RETRACT_HOLD_MS)/RETRACT_RELEASE_MS);
        else s->torque[i]+=rl_clip(target-s->torque[i],-15.0f*dt,15.0f*dt);
    }
    return true;
}
