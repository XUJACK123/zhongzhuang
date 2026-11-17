#ifndef RETRACTION_MEASUREMENT_H
#define RETRACTION_MEASUREMENT_H
#include <stdint.h>
#include <stdbool.h>
#define MEASURE_VALUES 20U
typedef struct {
    uint32_t request,ack,state,started,ended,count,invalid_count,inhibit_or;
    uint32_t max_step_ms,step_gaps,max_motor_age_ms,max_imu_age_ms,max_remote_age_ms;
    uint32_t tx_fail_start,tx_fail_end,rx_overflow_start,rx_overflow_end;
    uint32_t drdy_start,drdy_end,imu_missed_start,imu_missed_end;
    float mean[MEASURE_VALUES],m2[MEASURE_VALUES],min[MEASURE_VALUES],max[MEASURE_VALUES];
} StaticMeasurement;
extern StaticMeasurement g_static_measurement;
void static_measurement_begin(uint32_t now);
void static_measurement_sample(uint32_t now,uint32_t step,bool allowed,uint32_t inhibit,const float values[MEASURE_VALUES]);
typedef struct {
    uint32_t valid,tick,fault,reason,inhibit;
    float q[6],qd[6],torque[6],origin[6],motor_velocity[6];
} StopEvent;
extern StopEvent g_stop_event;
#endif
