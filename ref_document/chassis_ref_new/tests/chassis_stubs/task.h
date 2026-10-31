#ifndef CHASSIS_TEST_TASK_H
#define CHASSIS_TEST_TASK_H
#include "FreeRTOS.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *wake_time, TickType_t period);
#endif
