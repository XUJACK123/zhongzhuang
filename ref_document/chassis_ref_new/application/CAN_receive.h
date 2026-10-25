#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_protocol.h"

typedef struct
{
    motor_feedback_t motor[CHASSIS_MOTOR_COUNT];
} motor_can_snapshot_t;

bool motor_can_init(void);
void motor_can_get_snapshot(motor_can_snapshot_t *snapshot);
bool motor_can_send_torques(const float requested_nm[CHASSIS_MOTOR_COUNT],
                            float applied_nm[CHASSIS_MOTOR_COUNT]);
bool motor_can_send_zero(void);
void motor_can_joint_init_reset(void);
bool motor_can_joint_init_step(uint32_t now_ms);
bool motor_can_required_joints_enabled(void);

#endif
