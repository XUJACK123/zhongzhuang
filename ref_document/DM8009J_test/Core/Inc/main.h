#ifndef MAIN_H
#define MAIN_H

#include "stm32h7xx_hal.h"

extern FDCAN_HandleTypeDef hfdcan2;
void SystemClock_Config(void);
void FDCAN2_Clock_Config(void);
void MX_FDCAN2_Init(void);
void Error_Handler(void);

#endif
