#ifndef USART7_RECEIVE_H
#define USART7_RECEIVE_H

#include "main.h"

void My_UART7_IRQHandler(void);
void USART7_receive_init(void);
void user_send_data(void);

typedef struct
{
  float data1;
  float data2;
  float data3;
  float data4;
  float data5;
  float data6;
  float data7;
  float data8;
  float data9;
  float data10;
  float data11;
  char end1;
  char end2;
  char end3;
  char end4;
} user_send_t;

typedef struct
{
  int16_t data1;

} user_receive_t;

#endif
