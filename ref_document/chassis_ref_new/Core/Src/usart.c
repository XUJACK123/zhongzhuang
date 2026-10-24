/**
  ******************************************************************************
  * @file    usart.c
  * @brief   UART5 DBUS input and UART7 telemetry output configuration.
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

#include "usart.h"

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart7_tx;

void MX_UART5_Init(void)
{
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 100000;
    /* 8 个数据位加 1 个偶校验位，在 STM32 HAL 中使用 9B 字长。 */
    huart5.Init.WordLength = UART_WORDLENGTH_9B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_EVEN;
    huart5.Init.Mode = UART_MODE_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart5) != HAL_OK ||
        HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_UART7_Init(void)
{
    huart7.Instance = UART7;
    huart7.Init.BaudRate = 115200;
    huart7.Init.WordLength = UART_WORDLENGTH_8B;
    huart7.Init.StopBits = UART_STOPBITS_1;
    huart7.Init.Parity = UART_PARITY_NONE;
    huart7.Init.Mode = UART_MODE_TX;
    huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart7.Init.OverSampling = UART_OVERSAMPLING_16;
    huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart7) != HAL_OK ||
        HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    GPIO_InitTypeDef gpio = {0};

    if (uart->Instance == UART5)
    {
        __HAL_RCC_UART5_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_2;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOD, &gpio);

        hdma_uart5_rx.Instance = DMA1_Stream0;
        hdma_uart5_rx.Init.Request = DMA_REQUEST_UART5_RX;
        hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart5_rx.Init.Mode = DMA_NORMAL;
        hdma_uart5_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
        hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
        {
            Error_Handler();
        }
        __HAL_LINKDMA(uart, hdmarx, hdma_uart5_rx);

        HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    }
    else if (uart->Instance == UART7)
    {
        __HAL_RCC_UART7_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_8;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF7_UART7;
        HAL_GPIO_Init(GPIOE, &gpio);

        hdma_uart7_tx.Instance = DMA1_Stream5;
        hdma_uart7_tx.Init.Request = DMA_REQUEST_UART7_TX;
        hdma_uart7_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_uart7_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart7_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart7_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart7_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_uart7_tx.Init.Mode = DMA_NORMAL;
        hdma_uart7_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_uart7_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_uart7_tx) != HAL_OK)
        {
            Error_Handler();
        }
        __HAL_LINKDMA(uart, hdmatx, hdma_uart7_tx);

        /* DMA 完成后 HAL 仍需 UART TC 中断结束发送状态。 */
        HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART7_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uart)
{
    if (uart->Instance == UART5)
    {
        __HAL_RCC_UART5_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
        HAL_DMA_DeInit(uart->hdmarx);
        HAL_NVIC_DisableIRQ(UART5_IRQn);
    }
    else if (uart->Instance == UART7)
    {
        __HAL_RCC_UART7_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_8);
        HAL_DMA_DeInit(uart->hdmatx);
        HAL_NVIC_DisableIRQ(UART7_IRQn);
    }
}
