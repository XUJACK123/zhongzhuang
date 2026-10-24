/**
  ******************************************************************************
  * @file    stm32h7xx_it.h
  * @brief   Interrupt handler declarations used by the torque base.
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

#ifndef STM32H7XX_IT_H
#define STM32H7XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void FDCAN1_IT0_IRQHandler(void);
void FDCAN2_IT0_IRQHandler(void);
void TIM2_IRQHandler(void);
void SPI2_IRQHandler(void);
void UART5_IRQHandler(void);
void UART7_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
