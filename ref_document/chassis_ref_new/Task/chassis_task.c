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
    bool init_was_active = false;
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

        const bool remote_online = remote.valid &&
            age_ms(now_ms, remote.update_time_ms) <= CHASSIS_DBUS_TIMEOUT_MS;
#if TEST_MOTOR_ENABLE
        /* 台架上档即开始初始化，不再要求先下档解锁。 */
        const bool init_active = remote_online && remote.frame.switch_right == RC_SWITCH_UP;
#else
        const bool init_active = remote_online && remote.frame.switch_right == RC_SWITCH_DOWN;
#endif
        if (init_active)
        {
            if (!init_was_active) motor_can_joint_init_reset();
#if !TEST_MOTOR_ENABLE
            chassis_stop();
#endif
            (void)motor_can_joint_init_step(now_ms);
        }
        init_was_active = init_active;
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
        input.switch_left = remote.frame.switch_right;
        input.can_tx_ok = can_tx_ok;
        input.joints_enabled = joints_enabled;

        /* 最外层输出许可：直接核对遥控快照，不依赖测试任务提交零命令。 */
        bool remote_permit = remote.valid &&
            age_ms(HAL_GetTick(), remote.update_time_ms) <= CHASSIS_DBUS_TIMEOUT_MS &&
            remote.frame.switch_right == RC_SWITCH_UP;
#if TEST_MOTOR_ENABLE
        /* 简化台架：只保留闭环所需的反馈、命令有效性，不锁存恢复流程。
         * 不检查 IMU、左拨杆；遥控恢复上档后会从当前位置重新开始。 */
        safety.fault_flags = CHASSIS_FAULT_NONE;
        if (!remote_online) safety.fault_flags |= CHASSIS_FAULT_DBUS_TIMEOUT;
        if (!motors_valid || motor_max_age > CHASSIS_MOTOR_TIMEOUT_MS)
            safety.fault_flags |= CHASSIS_FAULT_MOTOR_TIMEOUT;
        if (!joints_enabled) safety.fault_flags |= CHASSIS_FAULT_JOINT_STATUS;
        if (!command.valid || input.command_age_ms > CHASSIS_COMMAND_TIMEOUT_MS)
            safety.fault_flags |= CHASSIS_FAULT_COMMAND_TIMEOUT;
        const bool safety_permit = safety.fault_flags == CHASSIS_FAULT_NONE;
        safety.state = !remote_online ? CHASSIS_STATE_LOCKED :
                       !remote_permit ? CHASSIS_STATE_STOPPED :
                       safety_permit ? CHASSIS_STATE_ARMED : CHASSIS_STATE_STANDBY;
#else
        const bool safety_permit = chassis_safety_step(&safety, &input);
#endif
        if (!remote_permit || !safety_permit)
        {
            if (!remote_permit)
            {
                /* 丢弃残留运动命令；非法命令故障仍需由下档明确复位。 */
                taskENTER_CRITICAL();
                memset(latest_command.torque_nm, 0, sizeof(latest_command.torque_nm));
                latest_command.valid = false;
                taskEXIT_CRITICAL();
            }
            memset(applied_nm, 0, sizeof(applied_nm));
            can_tx_ok = motor_can_send_zero(); /* 每个周期发零，不只是停止发帧。 */
        }
        else
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

        publish_feedback(&motors, &imu, &remote, applied_nm, &command, &safety, now_ms);
        vTaskDelayUntil(&wake_time, CHASSIS_TASK_PERIOD_TICKS);
    }
}
