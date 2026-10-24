#include "bsp_usart10.h"

extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;

void UART10_DMA_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	huart10.ReceptionType=HAL_UART_RECEPTION_TOIDLE;
	huart10.RxXferSize=dma_buf_num*2;
	
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart10.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart10, UART_IT_IDLE);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart10_rx);
    while(((DMA_Stream_TypeDef  *)hdma_usart10_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart10_rx);
    }
    ((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->PAR = (uint32_t) &huart10.Instance->RDR;
    //memory buffer 1
    //内存缓冲区1
    ((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    ((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    ((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef *)hdma_usart10_rx.Instance)->CR, DMA_SxCR_DBM);
    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart10_rx);

}
