#include "IMU_Receive.h"
#include "bsp_imu.h"

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;

uint8_t wit_buf[2][WIT_BUF_LENGTH];
WIT_data_t IMU_Data;

void WIT_imu_init(void)
{
	UART7_DMA_init( wit_buf[0], wit_buf[1], WIT_BUF_LENGTH);
}


void My_UART7_IRQHandler(void)
{
       static uint16_t this_time_rx_len = 0;
        if ((((DMA_Stream_TypeDef *)huart7.hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_uart7_rx);

            // get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = WIT_BUF_LENGTH - ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->NDTR;

            // reset set_data_lenght
            //重新设定数据长度
            ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->NDTR = WIT_BUF_LENGTH;

            // set memory buffer 1
            //设定缓冲区1
            ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->CR |= DMA_SxCR_CT;

            // enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_uart7_rx);

            if (this_time_rx_len == WIT_DATA_LENGTH)
            {
				if( wit_buf[0][0] == 0x55 )
				{
					uart_to_INS( wit_buf[0] );
				}
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_uart7_rx);

            // get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = WIT_BUF_LENGTH - ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->NDTR;

            // reset set_data_lenght
            //重新设定数据长度
            ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->NDTR = WIT_BUF_LENGTH;

            // set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_uart7_rx);

            if (this_time_rx_len == WIT_DATA_LENGTH)
            {
                if( wit_buf[1][0] == 0x55 )
				{
					uart_to_INS( wit_buf[1] );
				}
            }
        }
}

void uart_to_INS(uint8_t buf[])
{
	//当前设置回传了四组数据
	for( uint8_t j = 0; j<4 ; j++ )
	{
		uint8_t i;
		uint8_t data_shifting = j*11;
		
		if( !(buf[data_shifting] == 0x55) )
		{
			//帧头识别失败
			break;
		}else
		{
			//判别该组数据类型
			switch(buf[data_shifting+1])
			{
				case wit_acc_head:
				{
					IMU_Data.iAcc[0] = (buf[data_shifting+2] | ( buf[data_shifting+3] << 8 )) ;
					IMU_Data.iAcc[1] = (buf[data_shifting+4] | ( buf[data_shifting+5] << 8 )) ;
					IMU_Data.iAcc[2] = (buf[data_shifting+6] | ( buf[data_shifting+7] << 8 )) ;
					IMU_Data.temperature = (buf[data_shifting+8] | ( buf[data_shifting+9] << 8 )) / 100.0;
					
					for(i=0 ; i<3 ; i++) IMU_Data.fAcc[i] = IMU_Data.iAcc[i] * acc_u16_to_float;
					break;
				}
				case wit_gyro_head:
				{
					IMU_Data.iGyro[0] = (buf[data_shifting+2] | ( buf[data_shifting+3] << 8 )) ;
					IMU_Data.iGyro[1] = (buf[data_shifting+4] | ( buf[data_shifting+5] << 8 )) ;
					IMU_Data.iGyro[2] = (buf[data_shifting+6] | ( buf[data_shifting+7] << 8 )) ;
		//			WIT_data.temperature = buf[8] | ( buf[9] << 8 );
					
					for(i=0 ; i<3 ; i++)
					{
						IMU_Data.fGyro[i] = IMU_Data.iGyro[i] * gyro_u16_to_float;
						IMU_Data.fGyro_rad[i] = IMU_Data.fGyro[i] * degree_to_rad;
					}
					break;
				}
				case wit_ang_head:
				{
					IMU_Data.iAngle[0] = (buf[data_shifting+2] | ( buf[data_shifting+3] << 8 )) ;
					IMU_Data.iAngle[1] = (buf[data_shifting+4] | ( buf[data_shifting+5] << 8 )) ;
					IMU_Data.iAngle[2] = (buf[data_shifting+6] | ( buf[data_shifting+7] << 8 )) ;
					IMU_Data.version = buf[data_shifting+8] | ( buf[data_shifting+9] << 8 );
					
					for(i=0 ; i<3 ; i++)
					{
						IMU_Data.fAngle[i] = IMU_Data.iAngle[i] * ang_u16_to_float;
						IMU_Data.fAngle_rad[i] = IMU_Data.fAngle[i] * degree_to_rad;
					}
					
					break;
				}
				case wit_meg_head:
				{
					IMU_Data.iMeg[0] = buf[data_shifting+2] | ( buf[data_shifting+3] << 8 );
					IMU_Data.iMeg[1] = buf[data_shifting+4] | ( buf[data_shifting+5] << 8 );
					IMU_Data.iMeg[2] = buf[data_shifting+6] | ( buf[data_shifting+7] << 8 );
		//			WIT_data.temperature = buf[8] | ( buf[9] << 8 );
					
					for(i=0 ; i<3 ; i++) IMU_Data.fMeg[i] = IMU_Data.iMeg[i] / 10.0;
					break;
				}
				
				default:
					break;
			}
		}
	}
}
