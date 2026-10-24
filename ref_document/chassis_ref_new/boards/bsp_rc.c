#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	huart5.ReceptionType=HAL_UART_RECEPTION_TOIDLE;
	huart5.RxXferSize=dma_buf_num*2;
	
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart5_rx);
    while(((DMA_Stream_TypeDef  *)hdma_uart5_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart5_rx);
    }
    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->PAR = (uint32_t) &huart5.Instance->RDR;
    //memory buffer 1
    //内存缓冲区1
    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR, DMA_SxCR_DBM);
    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_uart5_rx);
}
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart5);
}
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart5);
    __HAL_DMA_DISABLE(&hdma_uart5_rx);

    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_uart5_rx);
    __HAL_UART_ENABLE(&huart5);
}
