#include "m3508_step.h"
#include "dbus_protocol.h"
#include <math.h>
static float Abs(float x) { return x < 0 ? -x : x; }
static float Clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static void Stop(M3508_StepController *c) {
    c->active = 0; c->current = 0; c->integral = 0;
    c->settling = 0; c->desired_speed_deg_s = 0;
}
static void Fault(M3508_StepController *c, uint32_t code) {
    Stop(c); c->fault = code; c->armed = 0;
    c->upper_latched = c->up_timing = c->down_timing = c->release_timing = 0;
}
void M3508_StepUpdate(M3508_StepController *c, const M3508_StepInput *in) {
    uint32_t now = in->now_ms;
    uint32_t elapsed = c->initialized ? now - c->last_ms : 2U;
    c->initialized = 1; c->last_ms = now;
    float speed = in->speed_deg_s;
    if (!in->remote_ok) { Fault(c, 1); return; }
    if (!in->motor_ok) { Fault(c, 2); return; }
    if (!in->bus_ok) { Fault(c, 3); return; }
    if (in->temperature >= STEP_MAX_TEMPERATURE) { Fault(c, 4); return; }
    if (!(Abs(in->position_deg) < 1e8f && Abs(speed) < STEP_OVERSPEED_DEG_S)) { Fault(c, 5); return; }
    if (c->active && elapsed > 20U) { Fault(c, 6); return; }
    int both_down = in->right_switch == DBUS_DOWN && in->left_switch == DBUS_DOWN;
    int both_up = in->right_switch == DBUS_UP && in->left_switch == DBUS_UP;
    if (both_down) {
        Stop(c); c->up_timing = 0;
        if (!c->down_timing) { c->down_timing = 1; c->down_since = now; }
        if (now - c->down_since >= STEP_ARM_DOWN_MS) {
            c->armed = 1; c->fault = 0; c->upper_latched = 0;
        }
    } else c->down_timing = 0;
    if (!both_up) {
        Stop(c); c->up_timing = 0;
        if (!c->release_timing) { c->release_timing = 1; c->release_since = now; }
        if (now - c->release_since >= STEP_SWITCH_DEBOUNCE_MS) c->upper_latched = 0;
        return;
    }
    c->release_timing = 0;
    if (!c->armed || c->fault) { Stop(c); return; }
    if (!c->up_timing) { c->up_timing = 1; c->up_since = now; }
    if (!c->upper_latched && now - c->up_since >= STEP_SWITCH_DEBOUNCE_MS) {
        c->upper_latched = 1; c->active = 1;
        c->target_deg = in->position_deg + STEP_DEGREES;
        c->move_start_ms = now; c->integral = 0; c->settling = 0;
        ++c->step_count;
    }
    if (!c->active) { c->current = 0; return; }
    if (now - c->move_start_ms > STEP_TIMEOUT_MS) { Fault(c, 7); return; }
    c->error_deg = c->target_deg - in->position_deg;
    if (Abs(c->error_deg) <= STEP_POSITION_TOL_DEG && Abs(speed) <= 0.4f) {
        c->current = 0; c->integral = 0;
        if (!c->settling) { c->settling = 1; c->settle_since = now; }
        if (now - c->settle_since >= STEP_SETTLE_MS) { ++c->completed_count; Stop(c); }
        return;
    }
    c->settling = 0;
    float dt = Clamp((float)elapsed * 0.001f, 0.0f, 0.02f);
    float stopping_speed = sqrtf(2.0f * STEP_ACCEL_DEG_S2 * Abs(c->error_deg));
    float bound = stopping_speed < STEP_SPEED_DEG_S ? stopping_speed : STEP_SPEED_DEG_S;
    float requested_speed = Clamp(c->error_deg * STEP_POSITION_KP, -bound, bound);
    c->desired_speed_deg_s += Clamp(requested_speed - c->desired_speed_deg_s,
                                    -STEP_ACCEL_DEG_S2 * dt, STEP_ACCEL_DEG_S2 * dt);
    float speed_error = c->desired_speed_deg_s - speed;
    float next_i = Clamp(c->integral + speed_error * STEP_SPEED_KI * dt,
                         -STEP_CURRENT_LIMIT, STEP_CURRENT_LIMIT);
    float raw = speed_error * STEP_SPEED_KP + next_i;
    if ((raw <= STEP_CURRENT_LIMIT && raw >= -STEP_CURRENT_LIMIT) ||
        (raw > STEP_CURRENT_LIMIT && speed_error < 0) || (raw < -STEP_CURRENT_LIMIT && speed_error > 0))
        c->integral = next_i;
    c->current = (int16_t)Clamp(speed_error * STEP_SPEED_KP + c->integral,
                               -STEP_CURRENT_LIMIT, STEP_CURRENT_LIMIT);
}
