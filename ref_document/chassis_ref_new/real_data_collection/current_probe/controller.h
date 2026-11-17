#ifndef CURRENT_PROBE_CONTROLLER_H
#define CURRENT_PROBE_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
#define RETRACT_CONTROL_VERSION 0xC2611171U
#define RETRACT_RAMP_MS 1000U
#define RETRACT_HOLD_MS 500U
#define RETRACT_RELEASE_MS 500U
#define RETRACT_TOTAL_MS 2000U
#define RETRACT_RECORD_MS 10U
#define PROBE_CAP_NM 3.0f
#define PROBE_TRAVEL_RAD 0.034906585f
#define PROBE_SPEED_RAD_S 0.15f
typedef struct {uint32_t index; float raw_nominal_nm;} ProbeConfig;
extern volatile ProbeConfig g_probe_config;
typedef struct {
    uint32_t started,reason,running;
    float origin[6],target[6],integral[6],torque[6],release_torque[6];
    uint32_t releasing;
    uint32_t index;
    float raw_nominal_nm;
} Retraction;
extern Retraction g_retraction;
bool probe_configuration_valid(void);
bool probe_guard(Retraction *s,const float q[6],const float qd[6]);
void retraction_begin(Retraction *s,uint32_t now,const float q[6]);
bool retraction_step(Retraction *s,uint32_t now,const float q[6],const float qd[6],float dt);
#endif
