#ifndef RL_H
#define RL_H
#include <stdbool.h>
#include <stdint.h>

#define RL_OBS 31
#define RL_ACT 6
#define RL_PI 3.14159265358979323846f
typedef struct {
    float q[6], qd[6]; /* URDF generalized coordinates, right hip/knee/wheel first */
    float omega[3], gravity[3]; /* body rad/s; body R^T*(0,0,-1) */
    float xy[2], yaw, velocity[2]; /* root origin in world; velocity in body */
} RlMeasurement;
typedef struct {
    float hidden1[128], hidden2[64], hidden3[32];
} RlNetwork;
typedef struct {
    RlNetwork network;
    float obs[31], action[6], command[3], reference_xy[2], reference_yaw;
    float target_q[6], target_qd[6], joint_torque[6], motor_torque[6];
} RlController;

float rl_clip(float x, float lo, float hi);
float rl_wrap(float x);
bool rl_actor(RlNetwork *net, const float obs[31], float action[6]);
void rl_reset(RlController *c, const RlMeasurement *m);
bool rl_policy_step(RlController *c, const RlMeasurement *m,
                    const float request[3], float dt);
bool rl_torque_step(RlController *c, const RlMeasurement *m);
bool rl_motor_to_joint(const float arm[6], const float speed[6],
                       float q[6], float qd[6]);
bool rl_joint_to_motor(const float q[6], const float qd[6], const float tau[6],
                       float motor[6], float motor_speed[6]);
#endif
