#ifndef BOARD_INDICATOR_H
#define BOARD_INDICATOR_H
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    uint32_t init_stage; /* 1=clock, 2=SPI, 3=PWM init, 4=PWM config, 5=PWM start, 6=ready */
    uint32_t last_hal_status;
    uint32_t tx_attempt_count;
    uint32_t tx_success_count;
    uint32_t tx_failure_count;
    uint32_t last_spi_error;
    uint32_t last_spi_sr;
    uint32_t last_tx_ms;
    uint32_t tx_stage; /* 1=TXP, 2=TXC, 3=SUSP, 4=complete */
} board_indicator_debug_t;
extern volatile board_indicator_debug_t board_indicator_debug;

/* DM-MC02：PA7/SPI6 为 WS2812，PB15/TIM12_CH2 为无源蜂鸣器。 */
bool board_indicator_init(void); /* 调度前仅配置外设；首次灯色发送由任务执行。 */
void board_indicator_service(uint32_t now_ms); /* 低优先级任务更新遥控颜色并推进三声启动音。 */
#endif
