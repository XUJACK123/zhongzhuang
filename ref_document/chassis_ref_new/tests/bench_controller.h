#ifndef BENCH_CONTROLLER_H
#define BENCH_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
#ifndef BENCH_TARGET_DEG
#define BENCH_TARGET_DEG 5.0f
#endif
#define BENCH_PERIOD_MS 2U
#define BENCH_MAX_GAP_MS 10U
#define BENCH_SPEED_RAD_S 0.0872664626f  /* 5 deg/s */
#define BENCH_ACCEL_RAD_S2 0.1745329252f /* 10 deg/s^2 */
#define BENCH_OVERSPEED_RAD_S 0.5f
#define BENCH_TRAVEL_MARGIN_RAD 0.0872664626f
#define BENCH_TARGET_RAD ((BENCH_TARGET_DEG) * 0.01745329252f)
enum
{
    BENCH_FAULT_NONE = 0U,
    BENCH_FAULT_DATA = 1U << 0,
    BENCH_FAULT_TIMING = 1U << 1,
    BENCH_FAULT_OVERSPEED = 1U << 2,
    BENCH_FAULT_TRAVEL = 1U << 3,
    BENCH_FAULT_NOT_REACHED = 1U << 4
};
typedef struct
{
    uint32_t faults, start_ms, previous_ms, settled_ms;
    bool active, reached;
    float previous_position;
    float position_rad; /* 从本轮起点累积的相对位置。 */
    float reference_rad, reference_speed, speed_rad_s, torque_nm;
} bench_controller_t;
void bench_controller_reset(bench_controller_t *controller);
void bench_trajectory(float time_s, float target_rad, float *position, float *velocity);
float bench_controller_step(bench_controller_t *controller, uint32_t now_ms,
                            bool permit, bool reset_requested, bool is_wheel,
                            float position_rad, float speed_rad_s);
#endif
