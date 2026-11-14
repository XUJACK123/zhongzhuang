#ifndef FDCAN_H
#define FDCAN_H
#include <stdbool.h>
#include <stdint.h>
extern volatile uint32_t g_can_tx_fail,g_can_rx_overflow;
bool can2_send(uint16_t id,const uint8_t data[8]);
void can2_cancel_pending(void);
bool can2_healthy(void);
#endif
