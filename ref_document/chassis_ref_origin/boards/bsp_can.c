#include "bsp_can.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern FDCAN_TxHeaderTypeDef Header;

void can_filter_init(void)
{
  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
  FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID; 				// 过滤ID类型选择 标准ID
  FDCAN1_FilterConfig.FilterIndex = 0;          			  	//当前FDCAN过滤器编号，可以设置多个过滤器过滤不同的ID 依次类推0、1、2....
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK; 			//过滤器Mask模式 关乎到下面ID1、ID2的配置
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;	//选择哪个FIFO区接收，根据CubeMX的配置来，FIFO1就改成FDCAN_FILTER_TO_RXFIFO1
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; 					// 这个都行，只要ID2配置0x00000000就不会过滤调任何ID
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; 					//理由如上
  HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig); 		//将上述配置到CAN1
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); //开启CAN1的全局过滤，就是开启过滤器
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);//打开FIFO0区的新数据接收中断
  HAL_FDCAN_Start(&hfdcan1);									//使能CAN1
 	
  FDCAN_FilterTypeDef FDCAN2_FilterConfig;
  FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN2_FilterConfig.FilterIndex = 0;
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN2_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN2_FilterConfig.FilterID2 = 0x00000000; 
  HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan2);
	
  FDCAN_FilterTypeDef FDCAN3_FilterConfig;
  FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN3_FilterConfig.FilterIndex = 0;
  FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN3_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN3_FilterConfig.FilterID2 = 0x00000000; 
  HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan3);
	
}
