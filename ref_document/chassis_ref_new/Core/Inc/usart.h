/**
  ******************************************************************************
  * @file    usart.h
  * @brief   UART5 DBUS and UART7 telemetry declarations.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef USART_H
#define USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;

void MX_UART5_Init(void);
void MX_UART7_Init(void);

#ifdef __cplusplus
}
#endif

#endif
