#include "remote_control.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#ifndef DBUS_RX_INVERTED
#define DBUS_RX_INVERTED 0
#endif

/* DMA1 无法访问 DTCM，因此接收缓冲区固定放在 AXI SRAM。 */
__attribute__((section(".AXI_SRAM"), aligned(32))) static uint8_t dbus_rx_buffer[RC_RX_BUFFER_LENGTH];
static volatile remote_control_snapshot_t remote_snapshot;
volatile remote_control_debug_t remote_debug;

static bool remote_control_start_dma(void)
{
    ++remote_debug.dma_start_count;
    const HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dbus_rx_buffer, RC_RX_BUFFER_LENGTH);
    remote_debug.last_dma_status = (uint32_t)status;
    if (status != HAL_OK)
    {
        ++remote_debug.dma_start_fail_count;
        remote_debug.recovery_pending = true;
        return false;
    }
    /* 36 字节缓冲的半传输恰为 18 字节；必须关闭 HT，等 IDLE 确认帧结束。 */
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    return true;
}

void remote_control_init(void)
{
    memset((void *)&remote_snapshot, 0, sizeof(remote_snapshot));
    memset((void *)&remote_debug, 0, sizeof(remote_debug));

#if DBUS_RX_INVERTED
    huart5.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
    huart5.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
    (void)HAL_UART_Init(&huart5);
#endif

    (void)remote_control_start_dma();
}

/* 与 HKU 一样把错误恢复移到任务上下文，最多尝试 5 次。
 * 只屏蔽 UART5 及其 DMA IRQ，HAL 时基继续运行，避免临界区内等待超时死锁。 */
void remote_control_service(void)
{
    if (!remote_debug.recovery_pending) return;

    HAL_NVIC_DisableIRQ(UART5_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    if (remote_debug.recovery_attempt_count < RC_MAX_RECOVERY_ATTEMPTS)
    {
        remote_debug.recovery_pending = false;
        remote_snapshot.valid = false;
        ++remote_debug.recovery_attempt_count;
        if (HAL_UART_AbortReceive(&huart5) == HAL_OK)
        {
            __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                 UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_IDLEF);
            __HAL_UART_SEND_REQ(&huart5, UART_RXDATA_FLUSH_REQUEST);
            if (remote_control_start_dma()) ++remote_debug.recovery_success_count;
        }
        else
        {
            remote_debug.recovery_pending = true;
        }
    }
    else
    {
        remote_debug.recovery_pending = false;
        remote_debug.recovery_exhausted = true;
        remote_snapshot.valid = false;
    }
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

void remote_control_get_snapshot(remote_control_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = remote_snapshot;
    taskEXIT_CRITICAL();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    chassis_dbus_frame_t decoded;

    if (huart != &huart5)
    {
        return;
    }

    /* HT 不终止 DMA，不能在这里重启；本工程已关闭 HT，此处再作防御。 */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) return;
    if (remote_debug.recovery_pending || remote_debug.recovery_exhausted) return;

    ++remote_debug.rx_event_count;
    remote_debug.last_rx_time_ms = HAL_GetTick();
    remote_debug.last_rx_size = size;
    remote_debug.last_rx_event = (uint32_t)HAL_UARTEx_GetRxEventType(huart);
    for (uint16_t i = 0U; i < RC_FRAME_LENGTH; ++i)
        remote_debug.last_raw[i] = i < size ? dbus_rx_buffer[i] : 0U;

    if (size == RC_FRAME_LENGTH && huart->RxEventType == HAL_UART_RXEVENT_IDLE)
    {
        const bool valid = chassis_dbus_decode(dbus_rx_buffer, &decoded);
        remote_debug.last_decoded = decoded;
        if (valid)
        {
            remote_snapshot.frame = decoded;
            remote_snapshot.update_time_ms = HAL_GetTick();
            remote_snapshot.valid = true;
            remote_debug.last_valid_time_ms = remote_snapshot.update_time_ms;
            ++remote_debug.valid_frame_count;
        }
        else
        {
            ++remote_debug.decode_error_count;
            remote_snapshot.valid = false;
        }
    }
    else
    {
        ++remote_debug.length_error_count;
        remote_snapshot.valid = false;
    }

    if (!remote_control_start_dma())
    {
        remote_snapshot.valid = false;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5)
    {
        ++remote_debug.uart_error_count;
        remote_debug.last_uart_error = huart->ErrorCode;
        remote_debug.uart_error_bits |= huart->ErrorCode;
        remote_snapshot.valid = false;
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                             UART_CLEAR_NEF | UART_CLEAR_OREF);
        remote_debug.recovery_pending = true;
    }
}
