#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "remote_control.h"
#include "usart.h"

static DMA_HandleTypeDef dma;
UART_HandleTypeDef huart5 = {.hdmarx = &dma};
static uint8_t *dma_buffer;
static uint16_t dma_size;
static uint32_t now_ms, starts, aborts;
static unsigned int irq_mask;
static bool receiving;
static HAL_StatusTypeDef start_status = HAL_OK, abort_status = HAL_OK;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *buf, uint16_t size)
{
    assert(uart == &huart5);
    ++starts;
    if (start_status != HAL_OK) return start_status;
    if (receiving) return HAL_BUSY;
    dma_buffer = buf;
    dma_size = size;
    receiving = true;
    uart->ErrorCode = 0U; /* HAL 启动会清错误；测试诊断是否预先保留。 */
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    assert(uart == &huart5 && irq_mask == 3U);
    ++aborts;
    if (abort_status == HAL_OK) receiving = false;
    return abort_status;
}
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(const UART_HandleTypeDef *uart)
{ return uart->RxEventType; }
uint32_t HAL_GetTick(void) { return now_ms; }
void HAL_NVIC_DisableIRQ(int irq) { irq_mask |= 1U << irq; }
void HAL_NVIC_EnableIRQ(int irq) { irq_mask &= ~(1U << irq); }

static void reset(void)
{
    receiving = false;
    start_status = abort_status = HAL_OK;
    starts = aborts = irq_mask = 0U;
    now_ms = 100U;
    dma.disabled_it = 0U;
    remote_control_init();
}
static void fill_frame(uint8_t right, uint8_t left)
{
    memset(dma_buffer, 0, dma_size);
    uint64_t bits = ((uint64_t)right << 44) | ((uint64_t)left << 46);
    for (unsigned int i = 0U; i < 4U; ++i) bits |= (uint64_t)1024U << (11U * i);
    for (unsigned int i = 0U; i < 6U; ++i) dma_buffer[i] = (uint8_t)(bits >> (8U * i));
    /* 第五通道留零：此前 +/-660 校验会把整帧丢弃。 */
}
static void event(HAL_UART_RxEventTypeTypeDef type, uint16_t size)
{
    huart5.RxEventType = type;
    if (type != HAL_UART_RXEVENT_HT) receiving = false;
    HAL_UARTEx_RxEventCallback(&huart5, size);
}
static remote_control_snapshot_t snapshot(void)
{
    remote_control_snapshot_t value;
    remote_control_get_snapshot(&value);
    return value;
}

int main(void)
{
    reset();
    assert(dma_size == 36U && (dma.disabled_it & DMA_IT_HT));
    assert(!snapshot().valid);
    fill_frame(1U, 3U);
    event(HAL_UART_RXEVENT_HT, 18U);
    assert(starts == 1U && !snapshot().valid); /* HT 不停止接收，也不作为完整帧。 */
    event(HAL_UART_RXEVENT_IDLE, 18U);
    assert(snapshot().valid && snapshot().frame.switch_right == 1U);
    assert(snapshot().frame.switch_left == 3U && snapshot().frame.channel[4] == -1024);
    assert(remote_debug.valid_frame_count == 1U && starts == 2U);
    assert(snapshot().update_time_ms == now_ms);

    event(HAL_UART_RXEVENT_IDLE, 7U); /* 中途启动的残帧，下一完整帧应恢复。 */
    assert(!snapshot().valid && remote_debug.length_error_count == 1U);
    fill_frame(2U, 1U);
    now_ms += 10U;
    event(HAL_UART_RXEVENT_IDLE, 18U);
    assert(snapshot().valid && snapshot().frame.switch_right == 2U);
    event(HAL_UART_RXEVENT_TC, 36U); /* 无有效空闲边界的满缓冲不能伪装成一帧。 */
    assert(!snapshot().valid && remote_debug.length_error_count == 2U);
    fill_frame(0U, 1U);
    event(HAL_UART_RXEVENT_IDLE, 18U);
    assert(!snapshot().valid && remote_debug.decode_error_count == 1U);

    uint32_t before = starts;
    huart5.ErrorCode = 9U;
    receiving = false; /* HAL 的 DMA 错误路径已终止接收。 */
    HAL_UART_ErrorCallback(&huart5);
    assert(starts == before && aborts == 0U && remote_debug.recovery_pending);
    assert(remote_debug.last_uart_error == 9U && !snapshot().valid);
    remote_control_service();
    assert(irq_mask == 0U && receiving && starts == before + 1U && aborts == 1U);
    assert(remote_debug.recovery_success_count == 1U && !remote_debug.recovery_pending);
    assert(remote_debug.last_uart_error == 9U && huart5.ErrorCode == 0U);
    fill_frame(3U, 2U);
    event(HAL_UART_RXEVENT_IDLE, 18U);
    assert(snapshot().valid);

    /* 初次启动失败也必须进入任务恢复；连续失败最多 5 次。 */
    receiving = false;
    start_status = HAL_ERROR;
    remote_control_init();
    assert(remote_debug.dma_start_fail_count == 1U && remote_debug.recovery_pending);
    for (unsigned int i = 0U; i < 10U; ++i) remote_control_service();
    assert(remote_debug.recovery_attempt_count == 5U && remote_debug.recovery_exhausted);
    assert(remote_debug.dma_start_fail_count == 6U && !snapshot().valid && irq_mask == 0U);

    reset();
    start_status = HAL_BUSY;
    fill_frame(1U, 2U);
    event(HAL_UART_RXEVENT_IDLE, 18U); /* 数据有效但重启失败，不能继续报告在线。 */
    assert(!snapshot().valid && remote_debug.recovery_pending);
    start_status = HAL_OK;
    remote_control_service();
    assert(!remote_debug.recovery_pending && remote_debug.recovery_success_count == 1U);
    assert(!snapshot().valid); /* 直到新的有效帧到来才恢复在线。 */

    UART_HandleTypeDef other = {0};
    before = remote_debug.rx_event_count;
    HAL_UARTEx_RxEventCallback(&other, 18U);
    HAL_UART_ErrorCallback(&other);
    assert(remote_debug.rx_event_count == before && remote_debug.uart_error_count == 0U);
    puts("remote receive tests passed (mock HAL; no hardware timing validation)");
    return 0;
}
