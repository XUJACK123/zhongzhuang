#ifndef FDCAN_H
#define FDCAN_H

#include <stdbool.h>

#include "main.h"

/* 一个候选的 CAN 引脚组合：TX 与 RX 位于同一端口。 */
typedef struct
{
    FDCAN_GlobalTypeDef *instance;
    GPIO_TypeDef *port;
    uint16_t tx_pin;
    uint16_t rx_pin;
    uint32_t alternate;
    uint32_t prescaler; /* 仲裁段预分频：3=1Mbps，6=500kbps，12=250kbps */
} can_pin_option_t;

/* 复用一个句柄，逐个候选组合切换 Instance。 */
extern FDCAN_HandleTypeDef hfdcan1;

bool can_probe_open(const can_pin_option_t *option, uint32_t mode);
void can_probe_close(void);

#endif
