#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H
#include "algorithm/rl.h"
typedef struct { float xy[2], last_q[6], held_qd[6]; bool velocity_valid; uint32_t velocity_tick; } StateEstimator;
void estimator_reset(StateEstimator *s);
bool estimator_update(StateEstimator *s, RlMeasurement *m, const float quaternion[4],
                      const float arm[6], const float motor_speed[6],
                      const float omega[3], uint32_t now, float dt);
#endif
