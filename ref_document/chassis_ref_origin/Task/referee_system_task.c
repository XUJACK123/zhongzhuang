//但是因为时间原因，来不及弄这个了，当个demo之后可以修改为上控制板

#include "referee_system_task.h"
#include "bsp_usart10.h"
#include "detect_task.h"
#include "Freertos.h"
#include "task.h"
#include "referee_packet.h"
#include <string.h>

extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;

__attribute__((section (".AXI_SRAM"))) uint8_t usart10_buf[2][USART_RX_BUF_LENGHT];
RefereeSystemFrame_t receive_referee_system_data;
ReceiveReferee_t ReceiveReferee;
/**
  * @brief          裁判系统任务,用于给裁判系统发消息，主要是画UI
  * @retval         none
  */
void referee_usart_task(void const *argument)
{

    while (1)
    {
		vTaskDelay(1);
    }
}

void referee_pocket_solve(uint8_t *buf,uint16_t len)
{
    static uint16_t data_length;//实际数据内容长度
	static uint16_t data_id;//实际数据的ID
	data_length=(buf[2]<<8|buf[1]);//根据数据，总长-帧头-ID-帧尾，得到实际内容数据长度
	if(buf[0]==0xA5 && Verify_CRC16_Check_Sum(&buf[0],data_length+9)==1)//通过帧头和CRC16判断数据对不对进而解包,实际内容数据长度+其他内容9=总长(用于CRC校验)
	{
		data_id=(buf[5]<<8|buf[6]);//根据协议，得到当前数据的ID
		switch (data_id) 
		{
		case 0x0202:
			ReceiveReferee.buffer_energy=(buf[6+8]<<8|buf[6+9]);//6是0~6的7个帧头等，后面的数据根据裁判系统写
			ReceiveReferee.shooter_17mm_1_barrel_heat=(buf[6+10]<<8|buf[6+11]);
			break;
        default:
            break;
		}
	}
}

void My_UART10_IRQHandler(void)
{
	static uint16_t this_time_rx_len = 0;
	if ((((DMA_Stream_TypeDef *)huart10.hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
	{
		__HAL_DMA_DISABLE(&hdma_usart10_rx);
		this_time_rx_len = REFEREE_FIFO_BUF_LENGTH - ((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->NDTR;
		((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->NDTR = REFEREE_FIFO_BUF_LENGTH;
		((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->CR |= DMA_SxCR_CT;
		__HAL_DMA_ENABLE(&hdma_usart10_rx);
		referee_pocket_solve(usart10_buf[0],this_time_rx_len);
		memset(usart10_buf[0],0,USART_RX_BUF_LENGHT);
		detect_hook(REFEREE_TOE);
	}
	else
	{
		__HAL_DMA_DISABLE(&hdma_usart10_rx);
		this_time_rx_len = REFEREE_FIFO_BUF_LENGTH - ((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->NDTR;
		((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->NDTR = REFEREE_FIFO_BUF_LENGTH;
		((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->CR &= ~(DMA_SxCR_CT);
		__HAL_DMA_ENABLE(&hdma_usart10_rx);
		referee_pocket_solve(usart10_buf[1],this_time_rx_len);
		memset(usart10_buf[1],0,USART_RX_BUF_LENGHT);
		detect_hook(REFEREE_TOE);
	}
}

void referee_usart_init(void)
{
	UART10_DMA_init(usart10_buf[0],usart10_buf[1],USART_RX_BUF_LENGHT);
}
