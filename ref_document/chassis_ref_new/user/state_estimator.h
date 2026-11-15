#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H
#include "algorithm/rl.h"
typedef struct { float xy[2], last_q[6], held_qd[6]; bool velocity_valid; uint32_t velocity_tick; } StateEstimator;
typedef struct {
    float wheel_velocity_world[2][3], weight[2], residual, confidence;
    float predicted_world[3];
    uint32_t age_ms[6], degraded_ms;
    bool initialized, reference_realign;
} EstimatorQuality;
bool estimator_update_qualified(StateEstimator *s, EstimatorQuality *quality,
    RlMeasurement *m, const float quaternion[4], const float arm[6], const float speed[6],
    const float omega[3], const float acceleration[3], const uint32_t feedback_tick[6], uint32_t now, float dt);
void estimator_reset(StateEstimator *s);
bool estimator_update(StateEstimator *s, RlMeasurement *m, const float quaternion[4],
                      const float arm[6], const float motor_speed[6],
                      const float omega[3], uint32_t now, float dt);
#endif
