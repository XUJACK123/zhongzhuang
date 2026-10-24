#ifndef IMAGE_TRANS_RECEIVE_H
#define IMAGE_TRANS_RECEIVE_H

#include "main.h"

extern void ImageTransReceive_Init(void);
extern void My_UART1_IRQHandler(void);
extern void ImageTransReceive_SET(void);
extern void ImageTransReceive_Read(void);
#endif
