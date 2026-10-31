#ifndef BENCH_CONTROLLER_H
#define BENCH_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
#include "bench_config.h"
#define BENCH_PERIOD_MS 2U
#define BENCH_SPEED_RAD_S 0.0349065850f  /* 2 deg/s，台架要慢 */
#define BENCH_ACCEL_RAD_S2 0.0349065850f /* 2 deg/s^2 */
#define BENCH_TARGET_RAD ((BENCH_TARGET_DEG) * 0.01745329252f)
/* 常规位置 PID：力矩 = Kp·e + Ki·∫e·dt + Kd·de/dt，输出限幅 ±CHASSIS_BENCH_*_LIMIT_NM。
 * 单位：Kp = N·m/rad，Ki = N·m/(rad·s)，Kd = N·m·s/rad。 */
#define BENCH_SETTLE_MS 300U    /* 到位判定：轨迹走完 + 误差 <1° + 速度 <0.05 rad/s 持续这么久 */
#define BENCH_TIMEOUT_MS 20000U /* 单次移动超时后停住，等下一次下档复位 */
#define BENCH_JOINT_KP 5.0f
#define BENCH_JOINT_KI 3.0f
#define BENCH_JOINT_KD 0.30f
#define BENCH_JOINT_SLEW_NM_S 2.0f
#define BENCH_WHEEL_KP 3.0f
#define BENCH_WHEEL_KI 0.0f
#define BENCH_WHEEL_KD 1.0f
#define BENCH_WHEEL_SLEW_NM_S 4.0f
enum
{
    BENCH_FAULT_NONE = 0U,
    BENCH_FAULT_DATA = 1U << 0
};
typedef struct
{
    uint32_t faults, start_ms, previous_ms, settled_ms;
    bool active, reached, aborted;
    float previous_position;
    float position_rad; /* 从本轮起点累积的相对位置，单位 rad；3508 为电机输出轴角度。 */
    float reference_rad, reference_speed, speed_rad_s, torque_nm;
    float integral_nm;  /* 积分项：Ki·∫e·dt，用来消掉静摩擦造成的稳态误差。 */
    float previous_error; /* 上一次的位置误差，用于计算微分项。 */
} bench_controller_t;
void bench_controller_reset(bench_controller_t *controller);
void bench_trajectory(float time_s, float target_rad, float *position, float *velocity);
float bench_controller_step(bench_controller_t *controller, uint32_t now_ms,
                            bool permit, bool reset_requested, bool is_wheel,
                            float position_rad, float speed_rad_s);
#endif
