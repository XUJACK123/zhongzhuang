/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   FDCAN1 and FDCAN2 declarations.
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

#ifndef FDCAN_H
#define FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);

#ifdef __cplusplus
}
#endif

#endif
