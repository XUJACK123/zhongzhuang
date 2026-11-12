#include "dm8009_step.h"
#include "dbus_protocol.h"
static float Abs(float x) { return x < 0 ? -x : x; }
static float Clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static void Stop(DM_StepController *c) {
    c->active = 0; c->torque_Nm = 0; c->integral = 0; c->torque_cap_Nm = 0; c->holding = 0;
    c->settling = 0; c->desired_speed_deg_s = 0;
}
static void Fault(DM_StepController *c, uint32_t code) {
    Stop(c); c->fault = code; c->armed = 0;
    c->upper_latched = c->up_timing = c->down_timing = c->release_timing = 0;
}
void DM_StepUpdate(DM_StepController *c, const DM_StepInput *in) {
    uint32_t now = in->now_ms;
    int first_update=!c->initialized;
    uint32_t elapsed = c->initialized ? now - c->last_ms : DM_TX_PERIOD_MS;
    c->initialized = 1; c->last_ms = now;
    float speed = in->speed_deg_s;
    if (!in->remote_ok) { Fault(c, 1); return; }
    if (!in->motor_ok) { Fault(c, 2); return; }
    if (!in->bus_ok) { Fault(c, 3); return; }
    if (in->temperature >= DM_STEP_MAX_TEMPERATURE) { Fault(c, 4); return; }
    if (!(Abs(in->position_deg) < 1e8f && Abs(speed) < DM_STEP_OVERSPEED_DEG_S)) { Fault(c, 5); return; }
    if (c->active && elapsed > 20U) { Fault(c, 6); return; }
    /* Filtering prevents velocity noise from kicking the current command.
     * Raw speed above is still used for the independent overspeed stop. */
    if(first_update)c->filtered_speed_deg_s=speed;
    else c->filtered_speed_deg_s+=(speed-c->filtered_speed_deg_s)*
                                  ((float)elapsed/(12.0f+(float)elapsed));
    int both_down = in->right_switch == DBUS_DOWN && in->left_switch == DBUS_DOWN;
    int both_up = in->right_switch == DBUS_UP && in->left_switch == DBUS_UP;
    if (both_down) {
        Stop(c); c->up_timing = 0;
        if (!c->down_timing) { c->down_timing = 1; c->down_since = now; }
        if (now - c->down_since >= DM_STEP_ARM_MS) {
            c->armed = 1; c->fault = 0; c->upper_latched = 0;
        }
    } else c->down_timing = 0;
    if (!both_up) {
        Stop(c); c->up_timing = 0;
        if (!c->release_timing) { c->release_timing = 1; c->release_since = now; }
        if (now - c->release_since >= DM_STEP_DEBOUNCE_MS) c->upper_latched = 0;
        return;
    }
    c->release_timing = 0;
    if (!c->armed || c->fault) { Stop(c); return; }
    if (!c->up_timing) { c->up_timing = 1; c->up_since = now; }
    if (!c->upper_latched && now - c->up_since >= DM_STEP_DEBOUNCE_MS) {
        c->upper_latched = 1; c->active = 1;
        c->start_deg = in->position_deg;
        c->target_deg = DM_STEP_FIXED_TARGET ? DM_STEP_DEGREES : in->position_deg + DM_STEP_DEGREES;
        if(Abs(c->target_deg-c->start_deg)>DM_STEP_TRAVEL_LIMIT_DEG) {Fault(c,8);return;}
        c->move_start_ms = now; c->integral = 0; c->settling = 0; c->torque_cap_Nm = 0; c->holding = 0;
        ++c->step_count;
    }
    if (!c->active) { c->torque_Nm = 0; return; }
    if (!c->holding && now - c->move_start_ms > DM_STEP_TIMEOUT_MS) { Fault(c, 7); return; }
    if (Abs(in->position_deg - c->start_deg) > DM_STEP_TRAVEL_LIMIT_DEG) { Fault(c, 8); return; }
    /* No integral accumulation while the enable handshake is pending. */
    if (!in->drive_enabled) { c->torque_Nm = 0; c->integral = 0; c->torque_cap_Nm = 0; return; }
    c->error_deg = c->target_deg - in->position_deg;
    if (Abs(c->error_deg) <= DM_STEP_POSITION_TOL_DEG && Abs(c->filtered_speed_deg_s) <= 1.5f) {
        if (!c->settling) { c->settling = 1; c->settle_since = now; }
        if (!c->holding && now - c->settle_since >= DM_STEP_SETTLE_MS) {
            ++c->completed_count;
            if(DM_STEP_HOLD_AT_TARGET)c->holding=1;
            else {Stop(c);return;}
        }
    } else c->settling = 0;
    /* Retain load-compensating current THROUGH the settling interval. */
    c->desired_speed_deg_s = Clamp(c->error_deg * DM_STEP_POSITION_KP, -DM_STEP_SPEED_DEG_S, DM_STEP_SPEED_DEG_S);
    float control_speed=c->filtered_speed_deg_s;
    /* React early to acceleration in the commanded direction, while a
     * negative noise spike cannot suddenly increase positive current. */
    if(c->desired_speed_deg_s>0 && speed-0.5f>control_speed)control_speed=speed-0.5f;
    if(c->desired_speed_deg_s<0 && speed+0.5f<control_speed)control_speed=speed+0.5f;
    float error = c->desired_speed_deg_s - control_speed;
    float dt = Clamp((float)elapsed * 0.001f, 0.0f, 0.02f);
    c->torque_cap_Nm=Clamp(c->torque_cap_Nm+DM_STEP_TORQUE_SLEW_NM_S*dt,
                            0,DM_STEP_TORQUE_LIMIT_NM);
    float cap=c->torque_cap_Nm;
    float next_i = Clamp(c->integral + error * DM_STEP_SPEED_KI * dt,-cap,cap);
    float raw = error * DM_STEP_SPEED_KP + next_i;
    /* Anti-windup follows the CURRENT envelope, not the eventual maximum. */
    if ((raw <= cap && raw >= -cap) ||
        (raw > cap && error < 0) || (raw < -cap && error > 0))
        c->integral = next_i;
    /* Slowly raise available current, but let speed feedback brake/recover
     * promptly inside that envelope. Re-ramping every PI correction caused
     * stick/slip and prevented the speed loop from regulating breakaway. */
    c->torque_Nm=Clamp(error*DM_STEP_SPEED_KP+c->integral,-cap,cap);

}
