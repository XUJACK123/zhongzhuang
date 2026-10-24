#include "RS485_Receive.h"
#include "detect_task.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern void My_UART5_IRQHandler(void);
extern void My_UART7_IRQHandler(void);
uint8_t unitree_usart_buf[2][UNITREE_RX_BUF_NUM];
A1_Send_t A1_Send_2;
A1_Send_t A1_Send_3;
Joint_motor_t Joint_motor[4];
/**
  * @brief		串口初始化
  * @param[out]	none
  * @param[in]	none
  * @retval		none	
  * @attention  其实就是开启串口接收而已
*/
void RS485_Receive_Init(void)
{
	//开启对应串口对A1电机的数据进行接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,unitree_usart_buf[0],UNITREE_RX_BUF_NUM);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,unitree_usart_buf[1],UNITREE_RX_BUF_NUM);
}

/**
  * @brief		串口空闲中断回调函数
  * @param[out]	串口句柄：        huart
  * @param[in]	接收到的字节数：  Size
  * @retval		none	
  * @attention  所有的串口接收空闲中断，都在这里集中处理
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)    
{ 
	if(huart->RxEventType == HAL_UART_RXEVENT_IDLE || huart->RxEventType == HAL_UART_RXEVENT_TC)//防止DMA接收过半事件的干扰
	{
		if(huart == &huart2)
		{
			
			if (unitree_usart_buf[0][2]==0)
			{
				Joint_motor[0].Current_Temp=unitree_usart_buf[0][6];
				Joint_motor[0].Current_Error=unitree_usart_buf[0][7];
				Joint_motor[0].Current_Torque=(float)((int16_t)(unitree_usart_buf[0][13]<<8|unitree_usart_buf[0][12]))/256.0f*9.1f;
				Joint_motor[0].Current_Speed=(float)((int16_t)(unitree_usart_buf[0][15]<<8|unitree_usart_buf[0][14]))/128.0f/9.1f;
				Joint_motor[0].Current_Position=(float)(unitree_usart_buf[0][33]<<24|unitree_usart_buf[0][32]<<16|unitree_usart_buf[0][31]<<8|unitree_usart_buf[0][30]<<0)/16384.0f*3.1415926f*2.0f;
				detect_hook(CHASSIS_MOTOR3_TOE);
			}
			else if (unitree_usart_buf[0][2]==1)
			{
				Joint_motor[1].Current_Temp=unitree_usart_buf[0][6];
				Joint_motor[1].Current_Error=unitree_usart_buf[0][7];
				Joint_motor[1].Current_Torque=(float)((int16_t)(unitree_usart_buf[0][13]<<8|unitree_usart_buf[0][12]))/256.0f*9.1f;
				Joint_motor[1].Current_Speed=(float)((int16_t)(unitree_usart_buf[0][15]<<8|unitree_usart_buf[0][14]))/128.0f/9.1f;
				Joint_motor[1].Current_Position=(float)(unitree_usart_buf[0][33]<<24|unitree_usart_buf[0][32]<<16|unitree_usart_buf[0][31]<<8|unitree_usart_buf[0][30]<<0)/16384.0f*3.1415926f*2.0f;
				detect_hook(CHASSIS_MOTOR4_TOE);
			}
		}
		else if(huart==&huart3)
		{
			
			if (unitree_usart_buf[1][2]==0)
			{
				Joint_motor[2].Current_Temp=unitree_usart_buf[1][6];
				Joint_motor[2].Current_Error=unitree_usart_buf[1][7];
				Joint_motor[2].Current_Torque=(float)((int16_t)(unitree_usart_buf[1][13]<<8|unitree_usart_buf[1][12]))/256.0f*9.1f;
				Joint_motor[2].Current_Speed=(float)((int16_t)(unitree_usart_buf[1][15]<<8|unitree_usart_buf[1][14]))/128.0f/9.1f;
				Joint_motor[2].Current_Position=(float)(unitree_usart_buf[1][33]<<24|unitree_usart_buf[1][32]<<16|unitree_usart_buf[1][31]<<8|unitree_usart_buf[1][30]<<0)/16384.0f*3.1415926f*2.0f;
				detect_hook(CHASSIS_MOTOR5_TOE);
			}
			else if (unitree_usart_buf[1][2]==1)
			{
				Joint_motor[3].Current_Temp=unitree_usart_buf[1][6];
				Joint_motor[3].Current_Error=unitree_usart_buf[1][7];
				Joint_motor[3].Current_Torque=(float)((int16_t)(unitree_usart_buf[1][13]<<8|unitree_usart_buf[1][12]))/256.0f*9.1f;
				Joint_motor[3].Current_Speed=(float)((int16_t)(unitree_usart_buf[1][15]<<8|unitree_usart_buf[1][14]))/128.0f/9.1f;
				Joint_motor[3].Current_Position=(float)(unitree_usart_buf[1][33]<<24|unitree_usart_buf[1][32]<<16|unitree_usart_buf[1][31]<<8|unitree_usart_buf[1][30]<<0)/16384.0f*3.1415926f*2.0f;
				detect_hook(CHASSIS_MOTOR6_TOE);
			}
		}
		else if (huart==&huart5)
		{
			My_UART5_IRQHandler();
		}
		else if (huart==&huart7)
		{
			My_UART7_IRQHandler();
		}
	}
}

/**
  * @brief		串口错误中断回调函数
  * @param[out]	串口句柄  huart
  * @param[in]	none
  * @retval		none	
  * @attention  这里的串口错误回调函数是为了应对：串口接收数据过程中出现错误（比如接口接触不良，波特率异常，有设备
				在传输数据中突然掉线等）时，防止串口在一轮接收结束后，串口空闲中断与DMA均被关闭，而使得两者无法通过在回调函数中的
				HAL_UARTEx_ReceiveToIdle_DMA()来重启（这就导致发生错误时，就无法无法接收后续的新数据）所以若发生了错误我们需要在
				HAL_UART_ErrorCallback()中再重新开启接收，不然错误一旦发生，串口就再也不会继续接收数据了。
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,unitree_usart_buf[0],UNITREE_RX_BUF_NUM);//再次开启对应串口接收数据                                                         
	}
	else if(huart == &huart3)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,unitree_usart_buf[1],UNITREE_RX_BUF_NUM);//再次开启对应串口接收数据                                                         
	}
}

/**
  * @brief		A1电机通信需要的CRC校验
  * @param[out]	CRC数据
  * @param[in]	数据以及数据长度
  * @retval		none	
*/
static inline  uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = ((uint32_t)1 << 31);
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

/**
  * @brief		USART2的扭矩发送函数
  * @param[in]	扭矩和ID
  * @retval		none	
*/
void USART2_cmd_current(int16_t Motor_T,uint8_t ID)
{
	A1_Send_2.start_1=0xFE;
	A1_Send_2.start_2=0xEE;
	A1_Send_2.Motor_ID=ID;
	A1_Send_2.Reserved_1=0;
	
	A1_Send_2.mode=10;
	A1_Send_2.ModifyBit=0xFF;
	A1_Send_2.ReadBit=0;
	A1_Send_2.Reserved_2=0;
	A1_Send_2.Modify=0;
	A1_Send_2.Motor_T=Motor_T;//单位：Nm,|T|<128
	A1_Send_2.Motor_W=0;//单位：rad/s,|W|<256
	A1_Send_2.Motor_P=0;
	A1_Send_2.Motor_P_K=0;
	A1_Send_2.Motor_W_K=0;
	A1_Send_2.LowHzMotorCmdIndex=0;
	A1_Send_2.LowHzMotorCmdByte=0;
	A1_Send_2.Reserved_3=0;
	
	A1_Send_2.CRC_Data=crc32_core((uint32_t*)(&(A1_Send_2)),7);

	HAL_UART_Transmit_DMA( &huart2, (uint8_t*)&A1_Send_2, UNITREE_SEND_FRAME_LENGTH);

}

/**
  * @brief		USART3的扭矩发送函数
  * @param[in]	扭矩和ID
  * @retval		none	
*/
void USART3_cmd_current(int16_t Motor_T,uint8_t ID)
{
	A1_Send_3.start_1=0xFE;
	A1_Send_3.start_2=0xEE;
	A1_Send_3.Motor_ID=ID;
	A1_Send_3.Reserved_1=0;
			
	A1_Send_3.mode=10;
	A1_Send_3.ModifyBit=0xFF;
	A1_Send_3.ReadBit=0;
	A1_Send_3.Reserved_2=0;
	A1_Send_3.Modify=0;
	A1_Send_3.Motor_T=Motor_T;//单位：Nm,|T|<128
	A1_Send_3.Motor_W=0;//单位：rad/s,|W|<256
	A1_Send_3.Motor_P=0;
	A1_Send_3.Motor_P_K=0;
	A1_Send_3.Motor_W_K=0;
	A1_Send_3.LowHzMotorCmdIndex=0;
	A1_Send_3.LowHzMotorCmdByte=0;
	A1_Send_3.Reserved_3=0;
	
	A1_Send_3.CRC_Data=crc32_core((uint32_t*)(&(A1_Send_3)),7);

	HAL_UART_Transmit_DMA( &huart3, (uint8_t*)&A1_Send_3, UNITREE_SEND_FRAME_LENGTH);

}

const Joint_motor_t *get_joint_left_ahead_measure_point(void)
{
  return &Joint_motor[0];
}

const Joint_motor_t *get_joint_left_back_measure_point(void)
{
  return &Joint_motor[1];
}

const Joint_motor_t *get_joint_right_ahead_measure_point(void)
{
  return &Joint_motor[2];
}

const Joint_motor_t *get_joint_right_back_measure_point(void)
{
  return &Joint_motor[3];
}
