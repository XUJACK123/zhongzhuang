#include "bsp_imu.h"
#include "main.h"

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;


void UART7_DMA_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	huart7.ReceptionType=HAL_UART_RECEPTION_TOIDLE;
	huart7.RxXferSize=dma_buf_num*2;
	
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart7.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart7_rx);
    while(((DMA_Stream_TypeDef  *)hdma_uart7_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart7_rx);
    }
    ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->PAR = (uint32_t) &huart7.Instance->RDR;
    //memory buffer 1
    //内存缓冲区1
    ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    ((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef *)hdma_uart7_rx.Instance)->CR, DMA_SxCR_DBM);
    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_uart7_rx);

}
