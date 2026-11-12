/**
 * @file c620_can.c
 * @brief 从 HKU_Balancing_2026 的 CAN_3508_2006_CMD / BSP_FDCAN_SendStd 精简。
 *        保留 DJI 大端打包方式和 STM32 HAL 发送接口。
 */
#include "c620_can.h"
#include "m3508_step.h"

volatile C620_Feedback_t g_c620_feedback = {0};
FDCAN_TxHeaderTypeDef tx = {0};

HAL_StatusTypeDef C620_CAN_Start(void) {
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x200U + C620_MOTOR_ID; /* 1 号反馈 ID = 0x201 */
    filter.FilterID2 = 0x7FFU;               /* 精确匹配 11 位标准 ID */
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
        return HAL_ERROR;
    }
    /* 最小版使用轮询接收，不开启 CAN 中断。 */
    return HAL_FDCAN_Start(&hfdcan2);
}

HAL_StatusTypeDef C620_SendCurrent(uint8_t motor_id, int16_t current) {
    if (motor_id < 1U || motor_id > 8U) {
        return HAL_ERROR;
    }
    if (current > 16384) {
        current = 16384;
    } else if (current < -16384) {
        current = -16384;
    }

    uint8_t data[8] = {0};
    uint8_t slot = (uint8_t)((motor_id - 1U) % 4U);
    uint16_t raw = (uint16_t)current;
    data[2U * slot] = (uint8_t)(raw >> 8);       /* 高字节 */
    data[2U * slot + 1U] = (uint8_t)(raw & 0xFFU); /* 低字节 */

    tx.Identifier = (motor_id <= 4U) ? 0x200U : 0x1FFU;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U) {
        return HAL_BUSY;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx, data);
}

void C620_PollFeedback(void) {
    /* 只处理进入此函数时已有的帧，避免接收影响发送周期。 */
    uint32_t pending = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0);
    for (uint32_t i = 0; i < pending; ++i) {
        FDCAN_RxHeaderTypeDef rx = {0};
        uint8_t data[8] = {0};
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
            break;
        }
        if (rx.IdType != FDCAN_STANDARD_ID || rx.RxFrameType != FDCAN_DATA_FRAME ||
            rx.Identifier != 0x200U + C620_MOTOR_ID || rx.DataLength != FDCAN_DLC_BYTES_8) {
            continue;
        }
        uint16_t encoder = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
        if (encoder >= 8192U) continue;
        uint32_t now = HAL_GetTick();
        if (g_c620_feedback.frame_count && now - g_c620_feedback.last_rx_ms <= STEP_MOTOR_TIMEOUT_MS)
            g_c620_feedback.total_encoder += M3508_EncoderDelta(g_c620_feedback.encoder, encoder);
        g_c620_feedback.encoder = encoder;
        g_c620_feedback.speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        g_c620_feedback.measured_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
        g_c620_feedback.temperature = data[6];
        g_c620_feedback.last_rx_ms = now;
        g_c620_feedback.frame_count++;
    }
}
