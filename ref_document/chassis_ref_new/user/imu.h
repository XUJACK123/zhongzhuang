#ifndef IMU_H
#define IMU_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    float quaternion[4], omega[3], accel[3], gravity[3], yaw;
    float bias[3], bias_sum[3], accel_sum[3];
    uint32_t tick, samples, calibration_samples, errors;
    bool initialized, calibrated;
    uint32_t motion_resets, cadence_resets, max_gap_ms;
    volatile uint32_t drdy_edges;
} Imu;
extern Imu g_imu;
bool imu_init(void);
void imu_service(uint32_t now);
#endif
