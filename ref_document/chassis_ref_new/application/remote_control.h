#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_protocol.h"

#define RC_FRAME_LENGTH 18U
#define RC_RX_BUFFER_LENGTH (2U * RC_FRAME_LENGTH) /* HKU SBUS_RX_BUF_NUM = 36 */
#define RC_MAX_RECOVERY_ATTEMPTS 5U /* HKU UART_MAX_DMA_RESTARTS */

#define RC_SWITCH_UP 1U
#define RC_SWITCH_DOWN 2U
#define RC_SWITCH_MID 3U

typedef struct
{
    chassis_dbus_frame_t frame;
    uint32_t update_time_ms;
    bool valid;
} remote_control_snapshot_t;

/* 调试观测值，不用于控制判定；运行中读取多个字段可能跨越一次中断更新。 */
typedef struct
{
    uint32_t dma_start_count;
    uint32_t dma_start_fail_count;
    uint32_t last_dma_status; /* HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 */
    uint32_t rx_event_count;
    uint32_t valid_frame_count;
    uint32_t length_error_count;
    uint32_t decode_error_count;
    uint32_t uart_error_count;
    uint32_t last_uart_error; /* 在 HAL 恢复清零前保存错误码。 */
    uint32_t uart_error_bits; /* 自启动以来所有 UART 错误位的并集。 */
    uint32_t recovery_attempt_count;
    uint32_t recovery_success_count;
    bool recovery_pending;
    bool recovery_exhausted;
    uint32_t last_rx_time_ms;
    uint32_t last_valid_time_ms;
    uint32_t last_rx_event; /* HAL_UART_RXEVENT_TC=0, HT=1, IDLE=2 */
    uint16_t last_rx_size;
    uint8_t last_raw[RC_FRAME_LENGTH];
    chassis_dbus_frame_t last_decoded; /* 最近一份 18 字节数据的解码值，即使校验失败。 */
} remote_control_debug_t;

extern volatile remote_control_debug_t remote_debug;

void remote_control_init(void);
void remote_control_service(void); /* 每 10 ms 在任务中处理待恢复的 UART/DMA。 */
void remote_control_get_snapshot(remote_control_snapshot_t *snapshot);

#endif
