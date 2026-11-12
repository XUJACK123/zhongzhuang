#ifndef M3508_STEP_H
#define M3508_STEP_H
#include <stdint.h>
#include "m3508_step_config.h"
typedef struct {
    uint32_t initialized, last_ms, armed, active, upper_latched;
    uint32_t down_timing, down_since, up_timing, up_since, release_timing, release_since;
    uint32_t settling, settle_since, move_start_ms;
    uint32_t step_count, completed_count, fault;
    float target_deg, error_deg, integral, desired_speed_deg_s;
    int16_t current;
} M3508_StepController;
typedef struct {
    uint32_t now_ms, remote_ok, motor_ok, bus_ok;
    uint8_t right_switch, left_switch, temperature;
    float position_deg, speed_deg_s;
} M3508_StepInput;
void M3508_StepUpdate(M3508_StepController *c, const M3508_StepInput *in);
static inline float M3508_CountsToDegrees(int64_t counts) {
    return (float)counts * (360.0f / (8192.0f * M3508_FEEDBACK_RATIO));
}
static inline int32_t M3508_EncoderDelta(uint16_t previous, uint16_t next) {
    int32_t delta = (int32_t)next - previous;
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    return delta;
}
#endif
