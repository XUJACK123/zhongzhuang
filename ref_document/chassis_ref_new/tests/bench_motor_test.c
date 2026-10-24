/**
 ******************************************************************************
 * @file    bench_motor_test.c
 * @brief   临时台架测试：双拨杆全部上档时，让指定电机转动 +90 度。
 *
 * @note    仅用于检查电机连接和方向，不属于正式控制逻辑。
 *          其他 5 个电机始终提交 0 N·m。
 *
 *          使用方式：
 *            make clean
 *            make TEST_MOTOR=1
 *
 *          测试电机由 BENCH_CAN_BUS + BENCH_CAN_ID 指定。
 ******************************************************************************
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "chassis_protocol.h"
#include "chassis_task.h"
#include "main.h"
#include "remote_control.h"

/* ==========================================================================
 * 指定测试电机：CAN 总线 + 反馈 ID
 *
 * CAN1: 0x003 左前关节 | 0x004 左后关节 | 0x201 左轮
 * CAN2: 0x002 右前关节 | 0x001 右后关节 | 0x202 右轮
 * CAN3: 当前工程未初始化 FDCAN3，不能使用
 *
 * 注意：这里填的是“反馈 ID”。
 *       DM8009 的反馈 ID 与命令 ID 相同；
 *       3508 轮子的反馈 ID 是 0x201/0x202，命令统一发在 0x200。
 * ========================================================================== */
#ifndef BENCH_CAN_BUS
#define BENCH_CAN_BUS 1
#endif

#ifndef BENCH_CAN_ID
#define BENCH_CAN_ID 0x003U
#endif

#if (BENCH_CAN_BUS == 3)
#error "当前工程只初始化了 FDCAN1/FDCAN2，BENCH_CAN_BUS 只能为 1 或 2"
#endif

/* ==========================================================================
 * 测试参数
 * ========================================================================== */
#define BENCH_TARGET_RAD (1.57079632679f) /* +90 deg */

#define BENCH_JOINT_KP 5.0f
#define BENCH_JOINT_KD 0.20f
#define BENCH_JOINT_MAX_TORQUE_NM 5.0f

#define BENCH_WHEEL_KP 0.50f
#define BENCH_WHEEL_KD 0.02f
#define BENCH_WHEEL_MAX_TORQUE_NM 1.0f

#define BENCH_TORQUE_SLEW_NM_PER_S 30.0f
#define BENCH_PERIOD_MS 2U

/* ==========================================================================
 * 绿色 LED / 蜂鸣器配置
 *
 * 当前工程 .ioc 没有定义 LED 和蜂鸣器引脚，默认关闭。
 * 请按实际原理图修改端口、引脚、有效电平，并把 ENABLE 改成 1。
 * ========================================================================== */
#define BENCH_LED_ENABLE 0
#define BENCH_LED_PORT GPIOB
#define BENCH_LED_PIN GPIO_PIN_0
#define BENCH_LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BENCH_LED_ON() \
    HAL_GPIO_WritePin(BENCH_LED_PORT, BENCH_LED_PIN, GPIO_PIN_SET)
#define BENCH_LED_OFF() \
    HAL_GPIO_WritePin(BENCH_LED_PORT, BENCH_LED_PIN, GPIO_PIN_RESET)

/* 默认按“有源蜂鸣器”处理：GPIO 高电平响。无源蜂鸣器需要改成 PWM。 */
#define BENCH_BUZZER_ENABLE 0
#define BENCH_BUZZER_PORT GPIOB
#define BENCH_BUZZER_PIN GPIO_PIN_1
#define BENCH_BUZZER_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BENCH_BUZZER_ON() \
    HAL_GPIO_WritePin(BENCH_BUZZER_PORT, BENCH_BUZZER_PIN, GPIO_PIN_SET)
#define BENCH_BUZZER_OFF() \
    HAL_GPIO_WritePin(BENCH_BUZZER_PORT, BENCH_BUZZER_PIN, GPIO_PIN_RESET)

/* ==========================================================================
 * 内部实现
 * ========================================================================== */

static float bench_wrap_pi(float angle)
{
    while (angle > 3.14159265358979323846f)
    {
        angle -= 6.28318530717958647692f;
    }
    while (angle < -3.14159265358979323846f)
    {
        angle += 6.28318530717958647692f;
    }
    return angle;
}

/*
 * 把 (CAN 总线, 反馈 ID) 映射到 chassis_submit_torque() 使用的数组下标。
 * 返回 -1 表示当前工程不支持该组合。
 */
static int bench_resolve_motor_index(uint8_t bus, uint32_t id)
{
    if (bus == 1U)
    {
        if (id == 0x003U) return CHASSIS_MOTOR_LEFT_FRONT;
        if (id == 0x004U) return CHASSIS_MOTOR_LEFT_REAR;
        if (id == 0x201U) return CHASSIS_MOTOR_LEFT_WHEEL;
    }
    else if (bus == 2U)
    {
        if (id == 0x002U) return CHASSIS_MOTOR_RIGHT_FRONT;
        if (id == 0x001U) return CHASSIS_MOTOR_RIGHT_REAR;
        if (id == 0x202U) return CHASSIS_MOTOR_RIGHT_WHEEL;
    }
    return -1;
}

#if BENCH_LED_ENABLE
static void bench_led_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    BENCH_LED_GPIO_CLK_ENABLE();
    gpio.Pin = BENCH_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BENCH_LED_PORT, &gpio);
    BENCH_LED_ON();
}
#else
static void bench_led_init(void)
{
}
#endif

#if BENCH_BUZZER_ENABLE
static void bench_buzzer_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    BENCH_BUZZER_GPIO_CLK_ENABLE();
    gpio.Pin = BENCH_BUZZER_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BENCH_BUZZER_PORT, &gpio);
    BENCH_BUZZER_OFF();
}

static void bench_buzzer_beep_three_times(void)
{
    for (uint32_t i = 0U; i < 3U; ++i)
    {
        BENCH_BUZZER_ON();
        vTaskDelay(pdMS_TO_TICKS(100U));
        BENCH_BUZZER_OFF();
        vTaskDelay(pdMS_TO_TICKS(150U));
    }
}
#else
static void bench_buzzer_init(void)
{
}

static void bench_buzzer_beep_three_times(void)
{
}
#endif

void bench_test_task(void const *argument)
{
    TickType_t wake_time = xTaskGetTickCount();
    bool latched = false;
    float target_rad = 0.0f;
    float last_torque_nm = 0.0f;

    (void)argument;

    const int resolved_index =
        bench_resolve_motor_index(BENCH_CAN_BUS, BENCH_CAN_ID);

    if (resolved_index < 0)
    {
        /* 不支持的 CAN 总线/ID 组合：不发送任何非零力矩。 */
        for (;;)
        {
            float zero[CHASSIS_MOTOR_COUNT] = {0};
            (void)chassis_submit_torque(zero);
            vTaskDelay(pdMS_TO_TICKS(100U));
        }
    }

    const size_t motor_index = (size_t)resolved_index;

    bench_led_init();
    bench_buzzer_init();
    bench_buzzer_beep_three_times();

    /* 等 chassis_task 完成 CAN 初始化和第一轮反馈。 */
    vTaskDelay(pdMS_TO_TICKS(500U));

    for (;;)
    {
        remote_control_snapshot_t remote;
        chassis_feedback_t feedback;
        float torque_nm[CHASSIS_MOTOR_COUNT] = {0};

        remote_control_get_snapshot(&remote);

        const bool all_up = remote.valid &&
                            remote.frame.switch_left == RC_SWITCH_UP &&
                            remote.frame.switch_right == RC_SWITCH_UP;

        if (!all_up)
        {
            latched = false;
            last_torque_nm = 0.0f;
        }
        else if (chassis_get_feedback(&feedback))
        {
            const bool motor_valid = feedback.motor[motor_index].valid;

            if (!motor_valid)
            {
                latched = false;
                last_torque_nm = 0.0f;
            }
            else
            {
                if (!latched)
                {
                    /* 以“双拨杆刚全部上档”时的位置为起点，目标 +90 度。 */
                    target_rad = feedback.motor[motor_index].position_rad +
                                 BENCH_TARGET_RAD;
                    latched = true;
                }

                const bool is_wheel =
                    motor_index >= CHASSIS_MOTOR_LEFT_WHEEL;
                const float kp = is_wheel ? BENCH_WHEEL_KP : BENCH_JOINT_KP;
                const float kd = is_wheel ? BENCH_WHEEL_KD : BENCH_JOINT_KD;
                const float max_torque_nm =
                    is_wheel ? BENCH_WHEEL_MAX_TORQUE_NM
                             : BENCH_JOINT_MAX_TORQUE_NM;

                const float position_rad =
                    feedback.motor[motor_index].position_rad;
                const float speed_rad_s =
                    feedback.motor[motor_index].speed_rad_s;

                float error_rad = target_rad - position_rad;
                if (!is_wheel)
                {
                    /* 关节反馈在 ±π 内，取最短路径。 */
                    error_rad = bench_wrap_pi(error_rad);
                }

                float command_nm = kp * error_rad - kd * speed_rad_s;

                if (command_nm > max_torque_nm)
                {
                    command_nm = max_torque_nm;
                }
                else if (command_nm < -max_torque_nm)
                {
                    command_nm = -max_torque_nm;
                }

                /* 限制力矩变化率，避免一上档就阶跃。 */
                const float max_step =
                    BENCH_TORQUE_SLEW_NM_PER_S *
                    ((float)BENCH_PERIOD_MS * 0.001f);

                if (command_nm > last_torque_nm + max_step)
                {
                    command_nm = last_torque_nm + max_step;
                }
                else if (command_nm < last_torque_nm - max_step)
                {
                    command_nm = last_torque_nm - max_step;
                }

                torque_nm[motor_index] = command_nm;
                last_torque_nm = command_nm;
            }
        }

        (void)chassis_submit_torque(torque_nm);
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(BENCH_PERIOD_MS));
    }
}
