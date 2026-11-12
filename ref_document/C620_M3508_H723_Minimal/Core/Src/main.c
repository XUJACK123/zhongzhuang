/**
 * @file main.c
 * @brief DBUS both-up edge -> one +10 degree M3508 output-shaft step.
 */
#include "main.h"
#include "c620_can.h"
#include "dbus_remote.h"
#include "m3508_step.h"

/* Read-only diagnostic output; the step controller owns motor current. */
volatile int16_t g_c620_current = 0;
volatile M3508_StepController g_step;
volatile float g_m3508_position_deg, g_m3508_speed_deg_s;
volatile uint32_t g_remote_fresh, g_motor_fresh;

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
    if (!DBUS_Init()) Error_Handler();

    if (C620_CAN_Start() != HAL_OK) {
        Error_Handler();
    }

    M3508_StepController controller = {0};
    int16_t previous_current = 0;
    uint32_t speed_frame = 0, speed_tick = 0;
    int64_t speed_counts = 0;
    while (1) {
        C620_PollFeedback();
        DBUS_State remote = DBUS_Snapshot();
        uint32_t now = HAL_GetTick();
        FDCAN_ProtocolStatusTypeDef status = {0};
        int bus_ok = HAL_FDCAN_GetProtocolStatus(&hfdcan2, &status) == HAL_OK && !status.BusOff;
        g_can_bus_off = status.BusOff;
        g_remote_fresh = remote.valid && now - remote.last_rx_ms <= STEP_REMOTE_TIMEOUT_MS;
        g_motor_fresh = g_c620_feedback.frame_count && now - g_c620_feedback.last_rx_ms <= STEP_MOTOR_TIMEOUT_MS;
        g_m3508_position_deg = M3508_CountsToDegrees(g_c620_feedback.total_encoder);
        /* Integer RPM is too coarse at 3deg/s. Differentiate encoder counts
         * only when feedback updates, then low-pass the quantized increments. */
        if (!g_motor_fresh) { speed_frame = 0; g_m3508_speed_deg_s = 0; }
        else if (g_c620_feedback.frame_count != speed_frame) {
            uint32_t sample_tick = g_c620_feedback.last_rx_ms;
            uint32_t sample_ms = sample_tick - speed_tick;
            if (speed_frame && sample_ms && sample_ms <= STEP_MOTOR_TIMEOUT_MS) {
                float seconds = (float)sample_ms * 0.001f;
                float measured = M3508_CountsToDegrees(g_c620_feedback.total_encoder - speed_counts) / seconds;
                float alpha = seconds / (STEP_SPEED_FILTER_SECONDS + seconds);
                g_m3508_speed_deg_s += alpha * (measured - g_m3508_speed_deg_s);
            }
            speed_frame = g_c620_feedback.frame_count;
            speed_tick = sample_tick;
            speed_counts = g_c620_feedback.total_encoder;
        }
        M3508_StepInput input = {
            .now_ms = now, .remote_ok = g_remote_fresh, .motor_ok = g_motor_fresh, .bus_ok = bus_ok,
            .right_switch = remote.right_switch, .left_switch = remote.left_switch,
            .temperature = g_c620_feedback.temperature,
            .position_deg = g_m3508_position_deg, .speed_deg_s = g_m3508_speed_deg_s
        };
        M3508_StepUpdate(&controller, &input);
        g_step = controller;
        g_c620_current = controller.current;
        /* Drop pending nonzero frames when permission is removed or a step ends. */
        if (!g_c620_current && previous_current)
            (void)HAL_FDCAN_AbortTxRequest(&hfdcan2, 0xFFFFU);
        if (C620_SendCurrent(C620_MOTOR_ID, g_c620_current) == HAL_OK) {
            g_can_tx_queued++;
        } else {
            g_can_tx_failed++;
        }
        previous_current = g_c620_current;
        HAL_Delay(2);
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
