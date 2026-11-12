#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "can.h"

typedef struct _
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id(反馈的报文头)
    uint8_t rx_len;                // 接收长度,可能为0-8
} CANInstance;

void CANServiceInit(void);
CANInstance *CANRegister(uint32_t tx_id, uint32_t rx_id);
void CANTransmit(CANInstance *_instance);
void CANSetMotorCurrent(CANInstance *_instance, uint8_t motor_id, int16_t current);
#endif