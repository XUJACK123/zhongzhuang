/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   FDCAN1 and FDCAN2 configuration for the six chassis motors.
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

#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

static uint32_t fdcan_clock_users;

static void fdcan_common_init(FDCAN_HandleTypeDef *fdcan,
                              FDCAN_GlobalTypeDef *instance,
                              uint32_t message_ram_offset)
{
    fdcan->Instance = instance;
    fdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    fdcan->Init.Mode = FDCAN_MODE_NORMAL;
    fdcan->Init.AutoRetransmission = DISABLE;
    fdcan->Init.TransmitPause = DISABLE;
    fdcan->Init.ProtocolException = DISABLE;
    fdcan->Init.NominalPrescaler = 3;
    fdcan->Init.NominalSyncJumpWidth = 1;
    fdcan->Init.NominalTimeSeg1 = 23;
    fdcan->Init.NominalTimeSeg2 = 8;
    fdcan->Init.DataPrescaler = 3;
    fdcan->Init.DataSyncJumpWidth = 10;
    fdcan->Init.DataTimeSeg1 = 23;
    fdcan->Init.DataTimeSeg2 = 8;
    fdcan->Init.MessageRAMOffset = message_ram_offset;
    fdcan->Init.StdFiltersNbr = 1;
    fdcan->Init.ExtFiltersNbr = 0;
    fdcan->Init.RxFifo0ElmtsNbr = 8;
    fdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    fdcan->Init.RxFifo1ElmtsNbr = 0;
    fdcan->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    fdcan->Init.RxBuffersNbr = 0;
    fdcan->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    fdcan->Init.TxEventsNbr = 0;
    /* 本工程只使用发送 FIFO，不使用按索引访问的专用发送缓冲区。 */
    fdcan->Init.TxBuffersNbr = 0;
    /*
     * 普通周期每条总线发送 3 帧；关节非阻塞初始化周期还会先发送
     * 2 帧特殊命令，因此按最坏的 5 帧突发留出余量。
     */
    fdcan->Init.TxFifoQueueElmtsNbr = 8;
    fdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    fdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(fdcan) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_FDCAN1_Init(void)
{
    fdcan_common_init(&hfdcan1, FDCAN1, 0U);
}

void MX_FDCAN2_Init(void)
{
    fdcan_common_init(&hfdcan2, FDCAN2, 853U);
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcan)
{
    GPIO_InitTypeDef gpio = {0};

    if (fdcan->Instance != FDCAN1 && fdcan->Instance != FDCAN2)
    {
        return;
    }

    ++fdcan_clock_users;
    if (fdcan_clock_users == 1U)
    {
        __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    if (fdcan->Instance == FDCAN1)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOD, &gpio);
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
    else
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF9_FDCAN2;
        HAL_GPIO_Init(GPIOB, &gpio);
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcan)
{
    if (fdcan->Instance == FDCAN1)
    {
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    }
    else if (fdcan->Instance == FDCAN2)
    {
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6);
        HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    }
    else
    {
        return;
    }

    if (fdcan_clock_users > 0U)
    {
        --fdcan_clock_users;
        if (fdcan_clock_users == 0U)
        {
            __HAL_RCC_FDCAN_CLK_DISABLE();
        }
    }
}
