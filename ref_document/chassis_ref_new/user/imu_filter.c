#include "imu_filter.h"
#include <math.h>
ImuFusionState g_imu_fusion;
ImuTiming g_imu_timing;
float imu_acceleration_weight(ImuFusionState *s, const float a[3], const float up[3], float dt) {
    float norm=0,dot=0,change=0;
    if (!(dt>0 && dt<=0.005f)) return 0;
    for (unsigned i=0;i<3;++i) { norm+=a[i]*a[i]; change+=(a[i]-s->previous_accel[i])*(a[i]-s->previous_accel[i]); }
    norm=sqrtf(norm);
    if (!isfinite(norm) || norm<0.1f) return 0;
    for (unsigned i=0;i<3;++i) { dot+=a[i]*up[i]/norm; s->previous_accel[i]=a[i]; }
    s->jerk=s->initialized ? sqrtf(change)/dt : 0;
    s->direction_residual=acosf(fmaxf(-1,fminf(1,dot)));
    float magnitude=(norm-9.81f)/0.5f, direction=s->direction_residual/0.05f, jerk=s->jerk/20;
    s->acceleration_weight=1/(1+magnitude*magnitude+direction*direction+jerk*jerk);
    s->initialized=true;
    return s->acceleration_weight;
}
void imu_track_bias(ImuFusionState *s, const float gyro[3], float bias[3], float norm, float dt) {
    bool still=s->stationary_allowed && fabsf(norm-9.81f)<0.15f && s->jerk<2 && dt>0 && dt<=0.005f;
    for (unsigned i=0;i<3;++i) if (!isfinite(gyro[i]) || fabsf(gyro[i]-bias[i])>0.02f) still=false;
    s->stationary_seconds=still ? s->stationary_seconds+dt : 0;
    if (s->stationary_seconds<2) return;
    float gain=dt/30.0f;
    for (unsigned i=0;i<3;++i) bias[i]+=gain*(gyro[i]-bias[i]);
}
