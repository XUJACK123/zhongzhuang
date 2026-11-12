/* DM-MC02 P6 DBUS -> board inversion -> UART5 RX PD2 AF8.
 * 100000 baud, 8 data + even parity + 1 stop. IRQ+IDLE framing, no DMA/cache dependency. */
#include "main.h"
#include "dbus_remote.h"
volatile DBUS_State g_remote;
volatile uint32_t g_remote_uart_errors, g_remote_bad_frames, g_remote_rx_bytes;
static uint8_t bytes[18];
static uint32_t used, bad;

int DBUS_Init(void) {
    RCC_PeriphCLKInitTypeDef clock = {0};
    clock.PeriphClockSelection = RCC_PERIPHCLK_UART5;
    clock.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) return 0;
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_UART5_FORCE_RESET();
    __HAL_RCC_UART5_RELEASE_RESET();
    GPIO_InitTypeDef pin = {0};
    pin.Pin = GPIO_PIN_2;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_PULLUP;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    pin.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &pin);
    UART5->CR1 = 0;
    UART5->CR2 = 0; /* one stop bit; board Q1 already inverts the physical DBUS signal */
    UART5->CR3 = USART_CR3_EIE;
    UART5->PRESC = 0;
    UART5->BRR = (HAL_RCC_GetPCLK1Freq() + 50000U) / 100000U;
    UART5->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF |
                 USART_ICR_ORECF | USART_ICR_IDLECF;
    /* M0=1 gives 9 total bits including parity, hence 8 payload bits. */
    UART5->CR1 = USART_CR1_M0 | USART_CR1_PCE | USART_CR1_RE |
                 USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_PEIE | USART_CR1_UE;
    uint32_t start = HAL_GetTick();
    while (!(UART5->ISR & USART_ISR_REACK)) {
        if (HAL_GetTick() - start > 10U) return 0;
    }
    HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
    HAL_NVIC_ClearPendingIRQ(UART5_IRQn);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
    return 1;
}
void UART5_IRQHandler(void) {
    uint32_t status = UART5->ISR;
    if (status & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) {
        ++g_remote_uart_errors;
        g_remote.valid = 0;
        bad = 1;
        UART5->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
    }
    if (status & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)UART5->RDR;
        ++g_remote_rx_bytes;
        if (used < sizeof(bytes)) bytes[used++] = byte;
        else bad = 1;
    }
    if (status & USART_ISR_IDLE) {
        UART5->ICR = USART_ICR_IDLECF;
        DBUS_State next = {0};
        if (!bad && used == sizeof(bytes) && DBUS_Decode(bytes, &next)) {
            next.frame_count = g_remote.frame_count + 1U;
            next.last_rx_ms = HAL_GetTick();
            next.valid = 1;
            g_remote = next;
        } else if (used || bad) {
            ++g_remote_bad_frames;
            g_remote.valid = 0;
        }
        used = bad = 0;
    }
}
DBUS_State DBUS_Snapshot(void) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    DBUS_State snapshot = g_remote;
    __set_PRIMASK(mask);
    return snapshot;
}
