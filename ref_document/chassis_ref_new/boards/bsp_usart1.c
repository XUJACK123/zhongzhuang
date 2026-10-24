#include "bsp_usart1.h"
#include "usart.h"

void UART1_DMA_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	huart1.ReceptionType=HAL_UART_RECEPTION_TOIDLE;
	huart1.RxXferSize=dma_buf_num*2;
	
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(((DMA_Stream_TypeDef  *)hdma_usart1_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }
    ((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->PAR = (uint32_t) &huart1.Instance->RDR;
    //memory buffer 1
    //内存缓冲区1
    ((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    ((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    ((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef *)hdma_usart1_rx.Instance)->CR, DMA_SxCR_DBM);
    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

}
