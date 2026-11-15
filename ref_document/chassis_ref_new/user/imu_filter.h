#ifndef IMU_FILTER_H
#define IMU_FILTER_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    float previous_accel[3], acceleration_weight, direction_residual, jerk;
    float stationary_seconds;
    bool initialized, stationary_allowed;
} ImuFusionState;
typedef struct {
    volatile uint32_t edge_cycles, edge_tick;
    uint32_t consumed_cycles, missed_edges;
    float sample_dt_s;
} ImuTiming;
extern ImuFusionState g_imu_fusion;
extern ImuTiming g_imu_timing;
float imu_acceleration_weight(ImuFusionState *s, const float acceleration[3], const float up[3], float dt);
void imu_track_bias(ImuFusionState *s, const float raw_gyro[3], float bias[3], float norm, float dt);
#endif
