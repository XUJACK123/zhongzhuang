#ifndef BSP_USART7_H
#define BSP_USART7_H

#include "main.h"

extern void UART7_DMA_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif
