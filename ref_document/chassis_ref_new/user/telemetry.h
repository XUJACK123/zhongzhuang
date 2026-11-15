#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stdint.h>
#define TELEMETRY_CAPACITY 32U
#define TELEMETRY_SCHEMA 2U
typedef struct {
    uint32_t begin, tick, cycle, control_version, model_version, getup_model_version;
    uint32_t mode, phase, failure, inhibit, fault, policy_age_ms, submitted_mask;
    uint32_t motor_tick[6], motor_received[6], imu_tick, remote_tick;
    uint32_t motor_raw[6], unwrap_ambiguous[6], imu_missed_edges;
    float arm[6], motor_velocity[6], quaternion[4], omega[3], acceleration[3], bias[3], xy[2], velocity[2];
    float q[6], qd[6], obs[31], action[6], target_q[6], target_qd[6];
    float requested_motor_torque[6], limited_motor_torque[6], feedback[6];
    float odometry_confidence, wheel_residual, accel_weight, imu_dt_s;
    float torque_scale, balance_weight;
    uint8_t dm[8], dji[8];
    uint32_t end;
} TelemetryFrame;
typedef struct {
    uint32_t schema, frame_bytes, capacity, published, overwritten, frozen;
    TelemetryFrame frame[TELEMETRY_CAPACITY];
} TelemetryRing;
extern volatile TelemetryRing g_telemetry;
void telemetry_reset(void);
void telemetry_publish(const TelemetryFrame *frame);
void telemetry_freeze(void);
#endif
