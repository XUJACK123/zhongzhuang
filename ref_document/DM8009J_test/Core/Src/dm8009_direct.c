#include "dm8009_direct.h"
#include <string.h>
static float Abs(float x) {return x<0?-x:x;}
void DM_DirectTick(DM_DirectTest *t,const DM_App *a,uint32_t now,
                   uint32_t start,uint32_t abort,DBUS_State *remote) {
    float position=DM_USE_MULTI?a->multi_position_deg:a->position_deg;
    float speed=DM_USE_MULTI?a->multi_speed_deg_s:a->speed_deg_s;
    memset(remote,0,sizeof(*remote));
    remote->valid=1;remote->last_rx_ms=now;
    remote->right_switch=remote->left_switch=DBUS_DOWN;
    if(t->phase==0 && start) {
        t->precheck_flags=(a->stage!=DM_READY?1U:0U) |
            (!DM_USE_MULTI && !a->feedback_fresh?2U:0U) | (!DM_USE_MULTI && a->feedback.state>1U?4U:0U) |
            (a->step.active?8U:0U) |
            (!(Abs(speed)<(DM_USE_MULTI?3.0f:1.5f)*DM_DIRECT_START_SPEED_THRESHOLD_SCALE)?16U:0U) |
            (DM_USE_MULTI && (!a->multi_count || now-a->multi_ms>DM_RX_TIMEOUT_MS || a->multi_state!=1U)?32U:0U) |
            ((DM_USE_MULTI ? a->multi_temperature>=DM_STEP_MAX_TEMPERATURE :
             (a->feedback.mos_temperature>=DM_STEP_MAX_TEMPERATURE ||
              a->feedback.rotor_temperature>=DM_STEP_MAX_TEMPERATURE))?64U:0U);
        if(t->precheck_flags) {
            t->max_speed=Abs(speed);t->phase=3;t->result=6;return;
        }
        t->phase=1;t->start_ms=now;t->start_deg=position;
        t->initial_steps=a->step.step_count;t->initial_completed=a->step.completed_count;
    }
    if(t->phase==0 || t->phase==3)return;
    t->elapsed_ms=now-t->start_ms;t->final_deg=position;
    t->delta_deg=t->final_deg-t->start_deg;
    if(Abs(speed)>t->max_speed)t->max_speed=Abs(speed);
    if(Abs(a->torque_quantized_Nm)>t->peak_torque)t->peak_torque=Abs(a->torque_quantized_Nm);
    if(t->sample_count<128U && (!t->sample_count || now-t->last_sample_ms>=(DM_STEP_TIMEOUT_MS+119U)/120U)) {
        t->samples[t->sample_count++]=(DM_DirectSample){t->elapsed_ms,position,speed,a->torque_quantized_Nm};
        t->last_sample_ms=now;
    }
    uint32_t result=abort?5U:0U;
    if(!result && a->stage!=DM_READY)result=3;
    if(!result && a->step.fault)result=4;
    if(!result && a->step.completed_count>t->initial_completed)result=1;
    if(!result && t->elapsed_ms>=DM_STEP_TIMEOUT_MS)result=2;
    if(!result && Abs(t->delta_deg)>DM_STEP_TRAVEL_LIMIT_DEG)result=4;
    if(result) {
        t->phase=(DM_STEP_HOLD_AT_TARGET && result==1U)?4U:3U;
        t->result=result;t->stop_step_fault=a->step.fault;
        if(t->phase==4U)remote->right_switch=remote->left_switch=DBUS_UP;
        return;
    }
    if(t->phase==1 && t->elapsed_ms>=150U)t->phase=2;
    if(t->phase==2)remote->right_switch=remote->left_switch=DBUS_UP;
}
