#include "bench_controller.h"
#include <math.h>
#include <string.h>
#include "chassis_protocol.h"

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
    if (!permit || reset_requested)
    {
        bench_controller_reset(c);
        return 0.0f;
    }
    /* 3508 的位置/速度口径是“转子 / 13.94”，而台架的目标按**电机输出轴**（可见轴）计算：
     * 输出轴角度 = 转子 / 19.2 = 反馈值 × 13.94 / 19.2。 */
    const float gear = is_wheel ? (CHASSIS_WHEEL_GEAR_RATIO / CHASSIS_WHEEL_MOTOR_GEAR_RATIO) : 1.0f;
    const float motor_position = position_rad * gear;
    const float motor_speed = speed_rad_s * gear;

    c->faults = BENCH_FAULT_NONE;
    if (!isfinite(motor_position) || !isfinite(motor_speed) || !isfinite(BENCH_TARGET_RAD))
    {
        c->faults = BENCH_FAULT_DATA;
        goto stopped;
    }
    if (!c->active)
    {
        bench_controller_reset(c);
        c->active = true;
        c->start_ms = now_ms;
        c->previous_ms = now_ms;
        c->previous_position = motor_position;
        c->speed_rad_s = motor_speed;
        return 0.0f;
    }
    const uint32_t elapsed_ms = now_ms - c->previous_ms;
    if (elapsed_ms == 0U) return c->torque_nm;
    const float dt = (float)elapsed_ms * 0.001f;
    float delta = motor_position - c->previous_position;
    if (!is_wheel) delta = remainderf(delta, 2.0f * CHASSIS_DM_POSITION_MAX);
    c->position_rad += delta;
    c->previous_position = motor_position;
    c->previous_ms = now_ms;
    c->speed_rad_s = motor_speed;

    /* 一次上档只走一段：到位或超时后保持零力矩，直到下一次下档复位。 */
    if (c->reached || c->aborted)
    {
        c->torque_nm = 0.0f;
        return 0.0f;
    }
    bench_trajectory((float)(now_ms - c->start_ms) * 0.001f, BENCH_TARGET_RAD,
                      &c->reference_rad, &c->reference_speed);
    if (fabsf(BENCH_TARGET_RAD - c->reference_rad) < 1e-6f &&
        fabsf(BENCH_TARGET_RAD - c->position_rad) < 0.017453293f && fabsf(motor_speed) < 0.05f)
        c->settled_ms += elapsed_ms;
    else
        c->settled_ms = 0U;
    if (c->settled_ms >= BENCH_SETTLE_MS)
    {
        c->reached = true;
        c->torque_nm = 0.0f;
        return 0.0f;
    }
    if (now_ms - c->start_ms > BENCH_TIMEOUT_MS)
    {
        c->aborted = true;
        c->torque_nm = 0.0f;
        return 0.0f;
    }

    const float kp = is_wheel ? BENCH_WHEEL_KP : BENCH_JOINT_KP;
    const float ki = is_wheel ? BENCH_WHEEL_KI : BENCH_JOINT_KI;
    const float kd = is_wheel ? BENCH_WHEEL_KD : BENCH_JOINT_KD;
    const float limit = is_wheel ? CHASSIS_BENCH_WHEEL_LIMIT_NM : CHASSIS_BENCH_JOINT_LIMIT_NM;
    const float slew = is_wheel ? BENCH_WHEEL_SLEW_NM_S : BENCH_JOINT_SLEW_NM_S;
    /* 常规位置 PID */
    const float err = c->reference_rad - c->position_rad;
    const float derr = (err - c->previous_error) / dt;
    c->previous_error = err;
    const float p_term = kp * err;
    const float d_term = kd * derr;
    /* 抗积分饱和：输出已经顶到限幅且误差同向时，不再继续累加。 */
    const float unsaturated = p_term + c->integral_nm + d_term;
    const bool saturated = (unsaturated >= limit && err > 0.0f) ||
                           (unsaturated <= -limit && err < 0.0f);
    if (!saturated)
        c->integral_nm = clip(c->integral_nm + ki * err * dt, limit);
    const float requested = clip(p_term + c->integral_nm + d_term, limit);
    c->torque_nm += clip(requested - c->torque_nm, slew * dt);
    return c->torque_nm;
stopped:
    c->active = false;
    c->torque_nm = 0.0f; /* 停止直接清零，不走缓降。 */
    return 0.0f;
}
