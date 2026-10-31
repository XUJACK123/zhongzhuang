/** 台架控制器：只生成所选电机的力矩，不直接访问 CAN。 */
#include "bench_motor_test.h"
#include "FreeRTOS.h"
#include "task.h"
#include "chassis_task.h"

volatile bench_test_status_t bench_test_status;

void bench_test_get_status(bench_test_status_t *status)
{
    if (status == NULL) return;
    taskENTER_CRITICAL();
    *status = bench_test_status;
    taskEXIT_CRITICAL();
}

void bench_test_task(void const *argument)
{
    bench_test_status_t status = {0};
    chassis_motor_route_t route;
    (void)argument;
    (void)chassis_motor_route(BENCH_MOTOR_INDEX, &route);
    status.motor_index = BENCH_MOTOR_INDEX;
    status.bus = route.bus;
    status.tx_id = route.tx_id;
    status.rx_id = route.rx_id;
    vTaskDelay(pdMS_TO_TICKS(500U));
    TickType_t wake_time = xTaskGetTickCount(); /* 等待后建立周期基准。 */
    for (;;)
    {
        chassis_feedback_t feedback;
        float torque[CHASSIS_MOTOR_COUNT] = {0};
        (void)chassis_get_feedback(&feedback);
        const uint32_t now = HAL_GetTick();
        const bool remote_ok = feedback.remote.valid &&
            now - feedback.remote.update_time_ms <= CHASSIS_DBUS_TIMEOUT_MS;
        const bool motor_ok = feedback.motor[BENCH_MOTOR_INDEX].valid &&
            now - feedback.motor[BENCH_MOTOR_INDEX].update_time_ms <= CHASSIS_MOTOR_TIMEOUT_MS;
        const bool all_up = remote_ok && feedback.remote.frame.switch_left == RC_SWITCH_UP &&
            feedback.remote.frame.switch_right == RC_SWITCH_UP;
        const bool reset = remote_ok && feedback.remote.frame.switch_left == RC_SWITCH_DOWN;
        const bool permit = all_up && motor_ok && feedback.state == CHASSIS_STATE_ARMED &&
            feedback.fault_flags == 0U && now - feedback.sample_time_ms <= BENCH_MAX_GAP_MS;
        torque[BENCH_MOTOR_INDEX] = bench_controller_step(&status.controller, now,
            permit, reset, route.is_wheel, feedback.motor[BENCH_MOTOR_INDEX].position_rad,
            feedback.motor[BENCH_MOTOR_INDEX].speed_rad_s);
        /* 待机提交零命令，完成“上档后新命令”的握手。 */
        (void)chassis_submit_torque(torque);
        status.chassis_faults = feedback.fault_flags;
        status.chassis_state = feedback.state;
        taskENTER_CRITICAL();
        bench_test_status = status;
        taskEXIT_CRITICAL();
        /* 超期不补跑旧周期积累力矩。 */
        const TickType_t current = xTaskGetTickCount();
        if (current - wake_time >= pdMS_TO_TICKS(BENCH_PERIOD_MS)) wake_time = current;
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(BENCH_PERIOD_MS));
    }
}
