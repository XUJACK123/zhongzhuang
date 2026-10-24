#ifndef REFEREE_SYSTEM_TASK_H
#define REFEREE_SYSTEM_TASK_H

#include "main.h"

#define USART_RX_BUF_LENGHT     128
#define REFEREE_FIFO_BUF_LENGTH 256

void referee_usart_task(void const *argument);
void My_UART10_IRQHandler(void);
void referee_usart_init(void);
void referee_usart_task_receive(void const *argument);
#endif
