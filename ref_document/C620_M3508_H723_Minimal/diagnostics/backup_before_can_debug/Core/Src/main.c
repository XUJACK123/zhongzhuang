/**
 * @file main.c
 * @brief STM32H723VG + C620 + M3508，直接电流控制，无 PID / RTOS。
 */
#include "main.h"
#include "c620_can.h"

/* 只改这里：范围 -16384~16384，对应约 -20~20 A。
 * 1000 约为 +1.22 A；-1000 反向；0 为零电流。
 * volatile 允许调试时直接修改此变量。上电后会持续发送此设定值。
 */
volatile int16_t g_c620_current = 1000;

volatile uint32_t g_can_tx_queued = 0;
volatile uint32_t g_can_tx_failed = 0;
volatile uint32_t g_can_bus_off = 0;
volatile uint32_t g_error = 0;

int main(void) {
    SCB_EnableICache();
    HAL_Init();
    SystemClock_Config();
    FDCAN2_Clock_Config();
    MX_FDCAN2_Init();

    if (C620_CAN_Start() != HAL_OK) {
        Error_Handler();
    }

    while (1) {
        /* 发送最新电流值；失败时计数，下一轮继续尝试。 */
        if (C620_SendCurrent(C620_MOTOR_ID, g_c620_current) == HAL_OK) {
            g_can_tx_queued++;
        } else {
            g_can_tx_failed++;
        }
        C620_PollFeedback();

        FDCAN_ProtocolStatusTypeDef status = {0};
        if (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &status) == HAL_OK) {
            g_can_bus_off = status.BusOff;
        }
        HAL_Delay(2); /* 约每 2 ms 发送一次，沿用原底盘的发送周期。 */
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}

void Error_Handler(void) {
    g_error = 1;
    __disable_irq();
    while (1) {
    }
}
