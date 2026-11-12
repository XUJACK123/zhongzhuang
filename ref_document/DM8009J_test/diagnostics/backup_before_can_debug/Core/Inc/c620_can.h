#ifndef C620_CAN_H
#define C620_CAN_H

#include "main.h"

/* C620 上设置的电机编号：1~8。默认控制 1 号。 */
#define C620_MOTOR_ID 1U
#if C620_MOTOR_ID < 1U || C620_MOTOR_ID > 8U
#error "C620_MOTOR_ID must be 1..8"
#endif

typedef struct {
    uint16_t encoder;         /* 0~8191，电机转子编码器 */
    int16_t speed_rpm;        /* 电机转子转速 */
    int16_t measured_current; /* 电调反馈的原始电流值 */
    uint8_t temperature;
    uint32_t frame_count;
    uint32_t last_rx_ms;
} C620_Feedback_t;

extern volatile C620_Feedback_t g_c620_feedback;

HAL_StatusTypeDef C620_CAN_Start(void);
/* 单电机电流控制，其所在组内另外三个槽位发送 0。 */
HAL_StatusTypeDef C620_SendCurrent(uint8_t motor_id, int16_t current);
void C620_PollFeedback(void);

#endif
