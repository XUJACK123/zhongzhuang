#ifndef BOARD_H
#define BOARD_H
#include "main.h"
#include "remote_control.h"
#include <stdbool.h>
extern UART_HandleTypeDef huart5;
extern RemoteControl g_remote;
typedef struct {
    uint32_t init_stage, rx_starts, rx_start_failures, rx_events, uart_errors;
    uint32_t irq_count, cr1, isr, uart_clock, rx_state;
} BoardDebug;
extern volatile BoardDebug g_board_debug;
void board_uart_irq(void);
bool board_init(void);
void board_remote_snapshot(RemoteControl *out);
void board_watchdog_start(void);
void board_watchdog_feed(void);
uint32_t board_cycles(void);
#endif
