#ifndef RETRACTION_CONTROLLER_H
#define RETRACTION_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
/* Independent suspended-leg position experiment; never invokes an RL actor. */
#define RETRACT_RAMP_MS 1500U
#define RETRACT_HOLD_MS 5000U
#define RETRACT_RELEASE_MS 1000U
#define RETRACT_DELTA_RAD 0.0872664626f
#define RETRACT_CAP_NM 20.0f
#define RETRACT_RECORD_MS 20U
#define RETRACT_TOTAL_MS (RETRACT_RAMP_MS+RETRACT_HOLD_MS+RETRACT_RELEASE_MS)
#define RETRACT_CONTROL_VERSION 0xD2611172U
typedef struct {
    uint32_t started,reason,running;
    float origin[6],target[6],integral[6],torque[6],release_torque[6];
    uint32_t releasing;
} Retraction;
void retraction_begin(Retraction *s,uint32_t now,const float q[6]);
bool retraction_step(Retraction *s,uint32_t now,const float q[6],const float qd[6],float dt);
#endif
