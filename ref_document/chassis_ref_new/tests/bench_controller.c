#include "bench_controller.h"
#include <math.h>
#include <string.h>
#include "chassis_motor_config.h"

static float clip(float value, float limit)
{
    return fminf(fmaxf(value, -limit), limit);
}

void bench_controller_reset(bench_controller_t *controller)
{
    if (controller != NULL) memset(controller, 0, sizeof(*controller));
}

/* 按真实时间求梯形/三角速度轨迹，不通过补跑周期累加位置。 */
void bench_trajectory(float time_s, float target_rad, float *position, float *velocity)
{
    const float distance = fabsf(target_rad);
    const float sign = target_rad < 0.0f ? -1.0f : 1.0f;
    const float ramp = fminf(BENCH_SPEED_RAD_S / BENCH_ACCEL_RAD_S2,
                             sqrtf(distance / BENCH_ACCEL_RAD_S2));
    const float peak = ramp * BENCH_ACCEL_RAD_S2;
    const float cruise = peak > 0.0f ? fmaxf(0.0f, distance / peak - ramp) : 0.0f;
    const float end = 2.0f * ramp + cruise;
    const float t = fmaxf(time_s, 0.0f);
    if (t >= end)
    {
        *position = target_rad;
        *velocity = 0.0f;
    }
    else if (t < ramp)
    {
        *position = sign * 0.5f * BENCH_ACCEL_RAD_S2 * t * t;
        *velocity = sign * BENCH_ACCEL_RAD_S2 * t;
    }
    else if (t < ramp + cruise)
    {
        *position = sign * (0.5f * peak * ramp + peak * (t - ramp));
        *velocity = sign * peak;
    }
    else
    {
        const float remaining = end - t;
        *position = sign * (distance - 0.5f * BENCH_ACCEL_RAD_S2 * remaining * remaining);
        *velocity = sign * BENCH_ACCEL_RAD_S2 * remaining;
    }
}

float bench_controller_step(bench_controller_t *c, uint32_t now_ms,
                            bool permit, bool reset_requested, bool is_wheel,
                            float position_rad, float speed_rad_s)
{
    if (c == NULL) return 0.0f;
    if (reset_requested) bench_controller_reset(c);
    if (!permit || reset_requested || c->faults != BENCH_FAULT_NONE)
    {
        c->active = false;
        c->torque_nm = 0.0f;
        return 0.0f;
    }
    if (!isfinite(position_rad) || !isfinite(speed_rad_s) ||
        !isfinite(BENCH_TARGET_RAD) || fabsf(BENCH_TARGET_RAD) > 0.2617994f)
        c->faults |= BENCH_FAULT_DATA; /* 台架目标不超过 15 度。 */
    if (fabsf(speed_rad_s) > BENCH_OVERSPEED_RAD_S)
        c->faults |= BENCH_FAULT_OVERSPEED;
    if (c->faults != 0U) goto stopped;
    if (!c->active)
    {
        bench_controller_reset(c);
        c->active = true;
        c->start_ms = now_ms;
        c->previous_ms = now_ms;
        c->previous_position = position_rad;
        c->speed_rad_s = speed_rad_s;
        return 0.0f;
    }
    const uint32_t elapsed_ms = now_ms - c->previous_ms;
    if (elapsed_ms > BENCH_MAX_GAP_MS)
    {
        c->faults |= BENCH_FAULT_TIMING;
        goto stopped;
    }
    if (elapsed_ms == 0U) return c->torque_nm;
    const float dt = (float)elapsed_ms * 0.001f;
    float delta = position_rad - c->previous_position;
    if (!is_wheel) delta = remainderf(delta, 2.0f * CHASSIS_DM_POSITION_MAX);
    /* 小行程中跳变视为异常，不选择另一条整圈路径追赶。 */
    if (fabsf(delta) > 0.05f)
    {
        c->faults |= BENCH_FAULT_DATA;
        goto stopped;
    }
    c->position_rad += delta;
    c->previous_position = position_rad;
    c->previous_ms = now_ms;
    c->speed_rad_s = speed_rad_s;
    if (fabsf(c->position_rad) > fabsf(BENCH_TARGET_RAD) + BENCH_TRAVEL_MARGIN_RAD)
    {
        c->faults |= BENCH_FAULT_TRAVEL;
        goto stopped;
    }
    bench_trajectory((float)(now_ms - c->start_ms) * 0.001f, BENCH_TARGET_RAD,
                      &c->reference_rad, &c->reference_speed);
    if (fabsf(BENCH_TARGET_RAD - c->position_rad) < 0.017453293f && fabsf(speed_rad_s) < 0.05f)
        c->settled_ms += elapsed_ms;
    else
        c->settled_ms = 0U;
    if (c->settled_ms >= 300U) c->reached = true;
    if (!c->reached && now_ms - c->start_ms > 5000U)
    {
        c->faults |= BENCH_FAULT_NOT_REACHED;
        goto stopped;
    }
    const float kp = is_wheel ? 0.50f : 5.0f;
    const float kd = is_wheel ? 0.02f : 0.20f;
    const float limit = is_wheel ? CHASSIS_BENCH_WHEEL_LIMIT_NM : CHASSIS_BENCH_JOINT_LIMIT_NM;
    const float slew = is_wheel ? 0.50f : 2.0f;
    const float requested = clip(kp * (c->reference_rad - c->position_rad) +
                                 kd * (c->reference_speed - speed_rad_s), limit);
    c->torque_nm += clip(requested - c->torque_nm, slew * dt);
    return c->torque_nm;
stopped:
    c->active = false;
    c->torque_nm = 0.0f; /* 停止直接清零，不走缓降。 */
    return 0.0f;
}
