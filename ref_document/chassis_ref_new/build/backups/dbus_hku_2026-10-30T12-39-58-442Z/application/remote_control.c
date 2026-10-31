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
        return false;
    }
    /* 一帧正好 18 字节，不需要半传输回调。 */
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

    ++remote_debug.rx_event_count;
    remote_debug.last_rx_time_ms = HAL_GetTick();
    remote_debug.last_rx_size = size;
    remote_debug.last_rx_event = (uint32_t)HAL_UARTEx_GetRxEventType(huart);
    for (uint16_t i = 0U; i < RC_FRAME_LENGTH; ++i)
        remote_debug.last_raw[i] = i < size ? dbus_rx_buffer[i] : 0U;

    if (size == RC_FRAME_LENGTH)
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
        }
    }
    else
    {
        ++remote_debug.length_error_count;
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
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)HAL_UART_AbortReceive(huart);
        (void)remote_control_start_dma();
    }
}
