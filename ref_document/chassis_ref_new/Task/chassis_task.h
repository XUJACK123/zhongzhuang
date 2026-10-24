#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "CAN_receive.h"
#include "INS_task.h"
#include "chassis_protocol.h"
#include "chassis_safety.h"
#include "remote_control.h"

typedef struct
{
    motor_feedback_t motor[CHASSIS_MOTOR_COUNT];
    ins_snapshot_t imu;
    remote_control_snapshot_t remote;
    float applied_torque_nm[CHASSIS_MOTOR_COUNT];
    uint32_t command_time_ms;
    uint32_t sample_time_ms;
    uint32_t fault_flags;
    chassis_control_state_t state;
} chassis_feedback_t;

/*
 * 六维力矩顺序固定为：左前关节、左后关节、右前关节、右后关节、左轮、右轮。
 * 单位为 N·m，方向采用各电机驱动器的原生正方向。
 */
bool chassis_submit_torque(const float torque_nm[CHASSIS_MOTOR_COUNT]);
bool chassis_get_feedback(chassis_feedback_t *feedback);
void chassis_stop(void);
void chassis_task(void const *argument);

#endif
