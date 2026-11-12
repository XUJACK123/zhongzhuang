#ifndef DM8009_STEP_H
#define DM8009_STEP_H
#include <stdint.h>
#include "dm8009_step_config.h"
typedef struct {
    uint32_t initialized, last_ms, armed, active, upper_latched;
    uint32_t down_timing, down_since, up_timing, up_since, release_timing, release_since;
    uint32_t settling, settle_since, move_start_ms;
    uint32_t step_count, completed_count, fault;
    float start_deg, target_deg, error_deg, integral, desired_speed_deg_s, torque_Nm;
    float torque_cap_Nm; /* slowly rising envelope; inner speed loop remains responsive */
    float filtered_speed_deg_s;
    uint32_t holding;
} DM_StepController;
typedef struct {
    uint32_t now_ms, remote_ok, motor_ok, bus_ok, drive_enabled;
    uint8_t right_switch, left_switch, temperature;
    float position_deg, speed_deg_s;
} DM_StepInput;
void DM_StepUpdate(DM_StepController *c, const DM_StepInput *in);
#endif
