//之前给裁判系统图传链路的旧图传发消息，没有用到，暂时不用了

#include "Image_Trans_Receive.h"
#include "bsp_usart1.h"
#include "usart.h"

#define SET_IMAGETRANSRECEIVE_LENGTH 20
#define RECEIVE_IMAGETRANSRECEIVE_LENGTH 10

__attribute__((section (".AXI_SRAM"))) uint8_t Receive_ImageTrans_data[2][SET_IMAGETRANSRECEIVE_LENGTH];

void ImageTransReceive_SET(void);

void ImageTransReceive_Init(void)
{
	UART1_DMA_init(Receive_ImageTrans_data[0],Receive_ImageTrans_data[1],SET_IMAGETRANSRECEIVE_LENGTH);
	ImageTransReceive_SET();
	ImageTransReceive_Read();
}

void My_UART1_IRQHandler(void)
{
//	static uint16_t this_time_rx_len = 0;
	if ((((DMA_Stream_TypeDef *)huart1.hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
	{
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
//		this_time_rx_len = SET_IMAGETRANSRECEIVE_LENGTH - ((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->NDTR;
		((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->NDTR = SET_IMAGETRANSRECEIVE_LENGTH;
		((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->CR |= DMA_SxCR_CT;
		__HAL_DMA_ENABLE(&hdma_usart1_rx);
	}
	else
	{
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
//		this_time_rx_len = SET_IMAGETRANSRECEIVE_LENGTH - ((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->NDTR;
		((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->NDTR = SET_IMAGETRANSRECEIVE_LENGTH;
		((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->CR &= ~(DMA_SxCR_CT);
		__HAL_DMA_ENABLE(&hdma_usart1_rx);
	}
}

void ImageTransReceive_SET(void)
{
	__attribute__((section (".AXI_SRAM"))) static uint8_t tx_buf[10];
	
	tx_buf[0]=0xA5;
	tx_buf[1]=0x01;
	tx_buf[2]=0x00;
	tx_buf[3]=0x00;
	tx_buf[4]=0x68;
	tx_buf[5]=0x01;
	tx_buf[6]=0x0F;
	tx_buf[7]=0x01;
	tx_buf[8]=0x8B;
	tx_buf[9]=0xBD;
	HAL_UART_Transmit_DMA(&huart1,tx_buf,10);

}

void ImageTransReceive_Read(void)
{
	__attribute__((section (".AXI_SRAM"))) static uint8_t tx_buf[9];
	
	tx_buf[0]=0xA5;
	tx_buf[1]=0x00;
	tx_buf[2]=0x00;
	tx_buf[3]=0x0A;
	tx_buf[4]=0xBD;
	tx_buf[5]=0x02;
	tx_buf[6]=0x0F;
	tx_buf[7]=0x66;
	tx_buf[8]=0x04;
	HAL_UART_Transmit_DMA(&huart1,tx_buf,9);

}
