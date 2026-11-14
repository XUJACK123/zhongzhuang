#include "board.h"
UART_HandleTypeDef huart5;
RemoteControl g_remote;
volatile BoardDebug g_board_debug;
static IWDG_HandleTypeDef watchdog;
static uint8_t remote_rx[36]; /* interrupt RX, no D-cache/DMA coherence assumption */
static void remote_start(void) {
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart5,remote_rx,sizeof(remote_rx))!=HAL_OK) {
        g_remote.valid=false; ++g_board_debug.rx_start_failures;
    } else ++g_board_debug.rx_starts;
}
bool board_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g={0};
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14|GPIO_PIN_15,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
    g.Pin=GPIO_PIN_14|GPIO_PIN_15; g.Mode=GPIO_MODE_OUTPUT_PP; g.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC,&g); g.Pin=GPIO_PIN_0; HAL_GPIO_Init(GPIOA,&g);
    RCC_PeriphCLKInitTypeDef clk={0};
    clk.PeriphClockSelection=RCC_PERIPHCLK_UART5;
    clk.Usart234578ClockSelection=RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&clk)!=HAL_OK) return false;
    __HAL_RCC_UART5_CLK_ENABLE();
    g.Pin=GPIO_PIN_2; g.Mode=GPIO_MODE_AF_PP; g.Pull=GPIO_PULLUP;
    g.Speed=GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate=GPIO_AF8_UART5; HAL_GPIO_Init(GPIOD,&g);
    g_board_debug.init_stage=2;
    huart5.Instance=UART5; huart5.Init.BaudRate=100000;
    huart5.Init.WordLength=UART_WORDLENGTH_9B; /* parity consumes bit 9: payload 8 bits */
    huart5.Init.StopBits=UART_STOPBITS_1; huart5.Init.Parity=UART_PARITY_EVEN;
    huart5.Init.Mode=UART_MODE_RX; huart5.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart5.Init.OverSampling=UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling=UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler=UART_PRESCALER_DIV1;
    if (HAL_UART_Init(&huart5)!=HAL_OK || HAL_UARTEx_DisableFifoMode(&huart5)!=HAL_OK) return false;
    g_board_debug.init_stage=3;
    HAL_NVIC_SetPriority(UART5_IRQn,4,0); HAL_NVIC_EnableIRQ(UART5_IRQn);
    remote_start();
    g_board_debug.init_stage=4;
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0; DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    return true;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size) {
    if (h->Instance!=UART5) return;
    ++g_board_debug.rx_events;
    if (size==18 || size==36) {
        for (unsigned i=0;i<size;i+=18)
            if (!remote_decode(&g_remote,remote_rx+i,18,HAL_GetTick())) break;
    } else { g_remote.valid=false; ++g_remote.rejected; }
    remote_start();
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
    if (h->Instance!=UART5) return;
    ++g_board_debug.uart_errors;
    g_remote.valid=false; ++g_remote.rejected;
    (void)HAL_UART_AbortReceive(h); remote_start();
}
void board_remote_snapshot(RemoteControl *out) {
    g_board_debug.cr1=UART5->CR1; g_board_debug.isr=UART5->ISR;
    g_board_debug.uart_clock=RCC->APB1LENR & RCC_APB1LENR_UART5EN;
    g_board_debug.rx_state=huart5.RxState;
    uint32_t mask=__get_PRIMASK(); __disable_irq(); *out=g_remote; __set_PRIMASK(mask);
}
void board_uart_irq(void) { ++g_board_debug.irq_count; HAL_UART_IRQHandler(&huart5); }
void board_watchdog_start(void) {
    watchdog.Instance=IWDG1; watchdog.Init.Prescaler=IWDG_PRESCALER_32;
    watchdog.Init.Reload=99; watchdog.Init.Window=IWDG_WINDOW_DISABLE; /* ~100ms at 32kHz LSI */
    if (HAL_IWDG_Init(&watchdog)!=HAL_OK) Error_Handler();
}
void board_watchdog_feed(void) { (void)HAL_IWDG_Refresh(&watchdog); }
uint32_t board_cycles(void) { return DWT->CYCCNT; }
