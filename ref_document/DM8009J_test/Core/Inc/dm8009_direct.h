#ifndef DM8009_DIRECT_H
#define DM8009_DIRECT_H
#include "dm8009_app.h"
typedef struct { uint32_t ms; float deg, speed, torque; } DM_DirectSample;
typedef struct {
    uint32_t phase, result, start_ms, elapsed_ms, initial_steps, initial_completed;
    uint32_t sample_count, last_sample_ms;
    float start_deg, final_deg, delta_deg, max_speed, peak_torque;
    DM_DirectSample samples[128];
    uint32_t precheck_flags; /* stage,MIT age,state,active,speed,multi age/state,temp */
    uint32_t stop_step_fault;
} DM_DirectTest;
/* phase 0 idle,1 down-arm,2 run,3 done,4 reached and holding. Result 1 complete,2 timeout,
 * 3 app fault,4 step fault,5 host abort,6 not ready. One request per reset. */
void DM_DirectTick(DM_DirectTest *t,const DM_App *a,uint32_t now,
                   uint32_t start,uint32_t abort,DBUS_State *remote);
#endif
