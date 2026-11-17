#include "supported_trial.h"
#include "robot_config.h"
#include "algorithm/rl.h"
#include <math.h>
#include <string.h>
SupportedTrial g_supported_trial;
volatile SupportedSession g_supported_session;
bool supported_session_step(uint32_t now,bool remote_ok,uint8_t s1,bool centered,bool ready,
                            bool stopped,bool bounded_stop,bool other_fault,bool trace_ready) {
    volatile SupportedSession *s=&g_supported_session;
    if (remote_ok && s1==2) s->down_seen=1;
    if (!remote_ok || other_fault) s->down_seen=0;
    if (s->permit && (!remote_ok || s1!=3 || !centered || bounded_stop || other_fault)) {
        s->permit=0; s->last_stop=now; s->has_stopped=1;
    }
    uint32_t token=s->request;
    if (token!=s->ack) {
        s->ack=token;
        if (s->permit) s->status=3;
        else if (s->has_stopped && now-s->last_stop<2000U) s->status=2;
        else if (!remote_ok || s1!=3 || !centered || !ready || !trace_ready || !s->down_seen ||
                 (!stopped && !bounded_stop) || other_fault) s->status=1;
        else if (!isfinite(s->requested_wheel_cap) || s->requested_wheel_cap<0 ||
                 s->requested_wheel_cap>ROBOT_WHEEL_TORQUE_CAP_NM) s->status=4;
        else { s->applied_wheel_cap=s->requested_wheel_cap; s->permit=1; ++s->accepted; s->status=0; }
    }
    return s->permit!=0;
}
bool supported_trial_stop(uint32_t now, bool active, const float q[6], const float speed[6]) {
#if ROBOT_SUPPORTED_TRIAL_MS == 0
    (void)now; (void)active; (void)q; (void)speed; return false;
#else
    SupportedTrial *s=&g_supported_trial;
    if (!active) { s->running=0; return false; }
    if (!s->running) {
        s->started=now; s->running=1; s->reason=0;
        memcpy(s->origin,q,sizeof(s->origin));
    }
    if (now-s->started>=ROBOT_SUPPORTED_TRIAL_MS) s->reason=1;
    for (unsigned i=0;i<6;++i) {
        bool wheel=i==2 || i==5;
        float delta=q[i]-s->origin[i];
        if (i==0 || i==3) delta=rl_wrap(delta);
        if (!isfinite(q[i]) || !isfinite(speed[i])) s->reason=5;
        else if (fabsf(delta)>(wheel ? 0.70f : 0.14f)) s->reason=wheel ? 3 : 2;
        else if (wheel && fabsf(speed[i])>6.0f) s->reason=4;
    }
    return s->reason!=0;
#endif
}
