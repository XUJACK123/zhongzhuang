#include "bsp_can.h"
#include <string.h>

/* This example owns CAN1 and the 0x200 group (motors 1..4). */
static CANInstance motor_group;
static volatile MotorFeedback feedback;
CAN_RxHeaderTypeDef header;

void CANServiceInit(void)
{
    if (HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();
}

CANInstance *CANRegister(uint32_t tx_id, uint32_t rx_id)
{
    CAN_FilterTypeDef filter = {0};
    memset(&motor_group, 0, sizeof(motor_group));
    memset((void *)&feedback, 0, sizeof(feedback));
    motor_group.can_handle = &hcan1;
    motor_group.tx_id = tx_id;
    motor_group.rx_id = rx_id;
    motor_group.txconf.StdId = tx_id;
    motor_group.txconf.IDE = CAN_ID_STD;
    motor_group.txconf.RTR = CAN_RTR_DATA;
    motor_group.txconf.DLC = 8;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = rx_id << 5;
    filter.FilterMaskIdHigh = 0x7FFU << 5;
    filter.FilterMaskIdLow = 0x06U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) Error_Handler();
    return &motor_group;
}

HAL_StatusTypeDef CANTransmit(CANInstance *instance)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(instance->can_handle) == 0) return HAL_BUSY;
    return HAL_CAN_AddTxMessage(instance->can_handle, &instance->txconf,
                                instance->tx_buff, &instance->tx_mailbox);
}

void CANSetMotorCurrent(CANInstance *instance, uint8_t id, int16_t current)
{
    if (id < 1 || id > 4) return;
    if (current > 16384) current = 16384;
    if (current < -16384) current = -16384;
    instance->tx_buff[(id - 1) * 2] = (uint16_t)current >> 8;
    instance->tx_buff[(id - 1) * 2 + 1] = (uint16_t)current & 0xFF;
}

void CANGetMotorFeedback(MotorFeedback *out)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *out = feedback;
    __set_PRIMASK(mask);
}

static void CANFIFOxCallback(CAN_HandleTypeDef *hcan, uint32_t fifo)
{
    uint8_t data[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) != 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK) break;
        if (hcan != &hcan1 || header.IDE != CAN_ID_STD ||
            header.RTR != CAN_RTR_DATA || header.DLC != 8 ||
            header.StdId != motor_group.rx_id) continue;
        uint16_t encoder = ((uint16_t)data[0] << 8) | data[1];
        if (encoder > 8191) continue;
        if (feedback.received)
        {
            int32_t delta = (int32_t)encoder - feedback.encoder;
            if (delta > 4096) delta -= 8192;
            if (delta < -4096) delta += 8192;
            feedback.total_counts += delta;
        }
        feedback.encoder = encoder;
        feedback.speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
        feedback.current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
        feedback.temperature = data[6];
        feedback.last_rx_ms = HAL_GetTick();
        feedback.received = 1;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}
