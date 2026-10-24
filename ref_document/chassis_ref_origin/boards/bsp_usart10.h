#ifndef BSP_USART10_H
#define BSP_USART10_H

#include "main.h"

extern void UART10_DMA_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif
