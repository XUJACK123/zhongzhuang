#include "chassis_task.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#define CHASSIS_TASK_PERIOD_TICKS pdMS_TO_TICKS(2U)

typedef struct
{
    float torque_nm[CHASSIS_MOTOR_COUNT];
    uint32_t update_time_ms;
    uint32_t sequence;
    bool valid;
    bool invalid_latched;
} chassis_command_t;

static chassis_command_t latest_command;
static chassis_feedback_t latest_feedback;

static uint32_t age_ms(uint32_t now, uint32_t then)
{
    return now - then;
}

bool chassis_submit_torque(const float torque_nm[CHASSIS_MOTOR_COUNT])
{
    if (!chassis_torque_vector_valid(torque_nm))
    {
        taskENTER_CRITICAL();
        memset(latest_command.torque_nm, 0, sizeof(latest_command.torque_nm));
        latest_command.valid = false;
        latest_command.invalid_latched = true;
        latest_command.update_time_ms = HAL_GetTick();
        taskEXIT_CRITICAL();
        return false;
    }

    taskENTER_CRITICAL();
    memcpy(latest_command.torque_nm, torque_nm, sizeof(latest_command.torque_nm));
    latest_command.update_time_ms = HAL_GetTick();
    ++latest_command.sequence;
    latest_command.valid = true;
    taskEXIT_CRITICAL();
    return true;
}

void chassis_stop(void)
{
    taskENTER_CRITICAL();
    memset(latest_command.torque_nm, 0, sizeof(latest_command.torque_nm));
    latest_command.update_time_ms = HAL_GetTick();
    latest_command.valid = false;
    latest_command.invalid_latched = false;
    taskEXIT_CRITICAL();
}

bool chassis_get_feedback(chassis_feedback_t *feedback)
{
    if (feedback == NULL)
    {
        return false;
    }
    taskENTER_CRITICAL();
    *feedback = latest_feedback;
    taskEXIT_CRITICAL();
    return true;
}

static void read_command(chassis_command_t *command)
{
    taskENTER_CRITICAL();
    *command = latest_command;
    taskEXIT_CRITICAL();
}

static void publish_feedback(const motor_can_snapshot_t *motors,
                             const ins_snapshot_t *imu,
                             const remote_control_snapshot_t *remote,
                             const float applied_nm[CHASSIS_MOTOR_COUNT],
                             const chassis_command_t *command,
                             const chassis_safety_t *safety,
                             uint32_t now_ms)
{
    taskENTER_CRITICAL();
    memcpy(latest_feedback.motor, motors->motor, sizeof(latest_feedback.motor));
    latest_feedback.imu = *imu;
    latest_feedback.remote = *remote;
    memcpy(latest_feedback.applied_torque_nm, applied_nm, sizeof(latest_feedback.applied_torque_nm));
    latest_feedback.command_time_ms = command->update_time_ms;
    latest_feedback.sample_time_ms = now_ms;
    latest_feedback.fault_flags = safety->fault_flags;
    latest_feedback.state = safety->state;
    taskEXIT_CRITICAL();
}

void chassis_task(void const *argument)
{
    TickType_t wake_time;
    chassis_safety_t safety;
    bool can_tx_ok;
    bool down_was_active = false;
    float applied_nm[CHASSIS_MOTOR_COUNT] = {0};

    (void)argument;
    memset(&latest_command, 0, sizeof(latest_command));
    memset(&latest_feedback, 0, sizeof(latest_feedback));
    chassis_safety_init(&safety);
    can_tx_ok = motor_can_init();
    wake_time = xTaskGetTickCount();

    for (;;)
    {
        motor_can_snapshot_t motors;
        ins_snapshot_t imu;
        remote_control_snapshot_t remote;
        chassis_command_t command;
        chassis_safety_input_t input = {0};
        uint32_t motor_max_age = 0U;
        bool joints_enabled;

        motor_can_get_snapshot(&motors);
        INS_get_snapshot(&imu);
        remote_control_get_snapshot(&remote);
        const uint32_t now_ms = HAL_GetTick(); /* 采样之后取时间，避免 ISR 更新造成负年龄。 */

        const bool down_active = remote.valid &&
                                 age_ms(now_ms, remote.update_time_ms) <= CHASSIS_DBUS_TIMEOUT_MS &&
                                 remote.frame.switch_left == RC_SWITCH_DOWN;
        if (down_active)
        {
            if (!down_was_active)
            {
                /* 每次由操作者重新进入下档时，只开启一轮有限次数的初始化。 */
                motor_can_joint_init_reset();
            }
            chassis_stop();
            (void)motor_can_joint_init_step(now_ms);
        }
        down_was_active = down_active;
        read_command(&command);

        const bool motors_valid = chassis_feedback_health(motors.motor, now_ms,
                                                           &motor_max_age, &joints_enabled);

        input.now_ms = now_ms;
        input.dbus_valid = remote.valid;
        input.dbus_age_ms = age_ms(now_ms, remote.update_time_ms);
        input.motors_valid = motors_valid;
        input.motor_max_age_ms = motor_max_age;
        input.imu_valid = imu.valid;
        input.imu_age_ms = age_ms(now_ms, imu.update_time_ms);
        input.command_valid = command.valid;
        input.command_invalid = command.invalid_latched;
        input.command_age_ms = age_ms(now_ms, command.update_time_ms);
        input.command_sequence = command.sequence;
        input.switch_left = remote.frame.switch_left;
        input.can_tx_ok = can_tx_ok;
        input.joints_enabled = joints_enabled;

        if (chassis_safety_step(&safety, &input))
        {
            if (!motor_can_send_torques(command.torque_nm, applied_nm))
            {
                memset(applied_nm, 0, sizeof(applied_nm));
                (void)motor_can_send_zero();
                can_tx_ok = false;
            }
            else
            {
                can_tx_ok = true;
            }
        }
        else
        {
            memset(applied_nm, 0, sizeof(applied_nm));
            can_tx_ok = motor_can_send_zero();
        }

        publish_feedback(&motors, &imu, &remote, applied_nm, &command, &safety, now_ms);
        vTaskDelayUntil(&wake_time, CHASSIS_TASK_PERIOD_TICKS);
    }
}
