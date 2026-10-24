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

static bool remote_control_start_dma(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, dbus_rx_buffer, RC_RX_BUFFER_LENGTH) != HAL_OK)
    {
        return false;
    }
    /* 一帧正好 18 字节，不需要半传输回调。 */
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    return true;
}

void remote_control_init(void)
{
    memset((void *)&remote_snapshot, 0, sizeof(remote_snapshot));

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

    if (size == RC_FRAME_LENGTH && chassis_dbus_decode(dbus_rx_buffer, &decoded))
    {
        remote_snapshot.frame = decoded;
        remote_snapshot.update_time_ms = HAL_GetTick();
        remote_snapshot.valid = true;
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
        remote_snapshot.valid = false;
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)HAL_UART_AbortReceive(huart);
        (void)remote_control_start_dma();
    }
}
