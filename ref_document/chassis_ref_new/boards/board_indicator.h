#ifndef BOARD_INDICATOR_H
#define BOARD_INDICATOR_H
#include <stdbool.h>
#include <stdint.h>
/* DM-MC02：PA7/SPI6 为 WS2812，PB15/TIM12_CH2 为无源蜂鸣器。 */
bool board_indicator_init(void); /* 调度前，仅初始化并点亮绿色。 */
void board_indicator_service(uint32_t now_ms); /* 低优先级任务推进三声启动音。 */
#endif
