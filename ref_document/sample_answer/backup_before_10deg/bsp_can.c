#include "main.h"
#include "bsp_can.h"
#include <stm32f4xx_hal_can.h>
#include <stdlib.h>
#include "string.h"

void CANServiceInit()
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CANAddFilter(CANInstance *_instance)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;                                                       // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // 使用16位id模式,即只有低16位有效
    can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;              // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
    can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
    can_filter_conf.FilterIdLow = _instance->rx_id << 5;                                                      // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
    can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后自增
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器

    HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
}

CANInstance *CANRegister(uint32_t tx_id, uint32_t rx_id)
{
    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    memset(instance, 0, sizeof(CANInstance));

    instance->can_handle = &hcan1;
    instance->tx_id = tx_id;
    instance->rx_id = rx_id;

    instance->txconf.StdId = tx_id;     // 对于 C620/C610 控制 1-4 号电机，此处为 0x200
    instance->txconf.IDE = CAN_ID_STD;
    instance->txconf.RTR = CAN_RTR_DATA;
    instance->txconf.DLC = 0x08;        // 发送 8 字节

    CANAddFilter(instance);
    return instance;
}

void CANTransmit(CANInstance *_instance)
{
    HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox);
}

void CANSetMotorCurrent(CANInstance *_instance, uint8_t canid, int16_t current)
{
    if (canid < 1 || canid > 4) return;

    uint8_t index = (canid - 1) * 2;
    _instance->tx_buff[index]     = (current >> 8) & 0xFF; // 高 8 位
    _instance->tx_buff[index + 1] = current & 0xFF;        // 低 8 位
}

static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf; // 同上
    static uint8_t RxData[8];     //用于保存接收到的信息
    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, RxData); // 从FIFO中获取数据
        if (_hcan == &hcan1 && rxconf.StdId == 0x201)
        {
            uint16_t encoder = (RxData[0] << 8) | RxData[1];        // 编码器值 0~8191                                     
            int16_t  speed   = (int16_t)((RxData[2] << 8) | RxData[3]);  // 转速 RPM                                       
            int16_t  current = (int16_t)((RxData[4] << 8) | RxData[5]);  // 实际电流 mA                                    
            uint8_t  temp    = RxData[6];                           // 温度 °C  
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}                                                                                                                    
