#include "chassis_safety.h"

#include <stddef.h>

#define DBUS_SWITCH_UP 1U
#define DBUS_SWITCH_DOWN 2U
#define DBUS_SWITCH_MID 3U

void chassis_safety_init(chassis_safety_t *safety)
{
    if (safety == NULL)
    {
        return;
    }
    safety->state = CHASSIS_STATE_LOCKED;
    safety->fault_flags = CHASSIS_FAULT_RECOVERY_REQUIRED;
    safety->up_entry_command_sequence = 0U;
    safety->down_seen = false;
    safety->up_seen = false;
}

bool chassis_safety_step(chassis_safety_t *safety, const chassis_safety_input_t *input)
{
    uint32_t health_faults = CHASSIS_FAULT_NONE;

    if (safety == NULL || input == NULL)
    {
        return false;
    }

    if (!input->dbus_valid || input->dbus_age_ms > CHASSIS_DBUS_TIMEOUT_MS)
    {
        health_faults |= CHASSIS_FAULT_DBUS_TIMEOUT;
    }
    if (!input->motors_valid || input->motor_max_age_ms > CHASSIS_MOTOR_TIMEOUT_MS)
    {
        health_faults |= CHASSIS_FAULT_MOTOR_TIMEOUT;
    }
    if (!input->imu_valid || input->imu_age_ms > CHASSIS_IMU_TIMEOUT_MS)
    {
        health_faults |= CHASSIS_FAULT_IMU_TIMEOUT;
    }
    if (!input->can_tx_ok)
    {
        health_faults |= CHASSIS_FAULT_CAN_TX;
    }
    if (!input->joints_enabled)
    {
        health_faults |= CHASSIS_FAULT_JOINT_STATUS;
    }
    if (input->command_invalid)
    {
        health_faults |= CHASSIS_FAULT_COMMAND_INVALID;
    }

    if (health_faults != CHASSIS_FAULT_NONE)
    {
        safety->state = CHASSIS_STATE_LOCKED;
        safety->fault_flags |= health_faults | CHASSIS_FAULT_RECOVERY_REQUIRED;
        safety->down_seen = false;
        safety->up_seen = false;
        return false;
    }

    if (input->switch_left == DBUS_SWITCH_DOWN)
    {
        safety->state = CHASSIS_STATE_STOPPED;
        safety->fault_flags = CHASSIS_FAULT_NONE;
        safety->down_seen = true;
        safety->up_seen = false;
        return false;
    }

    if (input->switch_left == DBUS_SWITCH_MID)
    {
        safety->state = safety->down_seen ? CHASSIS_STATE_STANDBY : CHASSIS_STATE_LOCKED;
        safety->up_seen = false;
        if (!safety->down_seen)
        {
            safety->fault_flags |= CHASSIS_FAULT_RECOVERY_REQUIRED;
        }
        return false;
    }

    if (input->switch_left != DBUS_SWITCH_UP || !safety->down_seen)
    {
        safety->state = CHASSIS_STATE_LOCKED;
        safety->fault_flags |= CHASSIS_FAULT_RECOVERY_REQUIRED;
        return false;
    }

    if (!safety->up_seen)
    {
        /* 进入上档后才开始等待新命令，禁止复用拨档前的旧力矩。 */
        safety->up_entry_command_sequence = input->command_sequence;
        safety->up_seen = true;
        safety->state = CHASSIS_STATE_STANDBY;
        safety->fault_flags = CHASSIS_FAULT_NONE;
        return false;
    }

    if (input->command_sequence == safety->up_entry_command_sequence || !input->command_valid)
    {
        safety->state = CHASSIS_STATE_STANDBY;
        safety->fault_flags = CHASSIS_FAULT_NONE;
        return false;
    }

    if (input->command_age_ms > CHASSIS_COMMAND_TIMEOUT_MS)
    {
        safety->state = CHASSIS_STATE_LOCKED;
        safety->fault_flags |= CHASSIS_FAULT_COMMAND_TIMEOUT | CHASSIS_FAULT_RECOVERY_REQUIRED;
        safety->down_seen = false;
        safety->up_seen = false;
        return false;
    }

    safety->state = CHASSIS_STATE_ARMED;
    safety->fault_flags = CHASSIS_FAULT_NONE;
    return true;
}
