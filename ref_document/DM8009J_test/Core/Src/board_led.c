/* DM-MC02 onboard WS2812: PA7/SPI6 MOSI, HSE 24 MHz /4 = 6 MHz.
 * GRB encoding follows dmBots/DM-MC02 WS2812 example (0x60=0, 0x78=1).
 * Polling only: no DMA memory-region/cache dependency; send only on change. */
#include "main.h"
#include "board_led.h"
static SPI_HandleTypeDef led_spi;
static uint32_t current_grb = 0xFFFFFFFFU;
volatile uint32_t g_led_grb, g_led_errors, g_led_last_sr, g_led_bytes_sent;
static int Transmit(const uint8_t *pixels, uint32_t count) {
    uint32_t start=HAL_GetTick();
    /* Use an unbounded SPI stream (TSIZE=0), then explicitly drain TXC and
     * suspend. This dedicated simplex LED transport does not depend on EOT. */
    CLEAR_BIT(SPI6->CR1,SPI_CR1_SPE);
    SPI6->CR2=0;
    SPI6->IFCR=SPI_IFCR_EOTC|SPI_IFCR_TXTFC|SPI_IFCR_SUSPC;
    SET_BIT(SPI6->CR1,SPI_CR1_SPE);
    g_led_bytes_sent=0;
    for(uint32_t i=0;i<count;++i) {
        while(!(SPI6->SR&SPI_SR_TXP)) {
            if(HAL_GetTick()-start>=2U)goto failed;
        }
        *(__IO uint8_t *)&SPI6->TXDR=pixels[i];
        if(!i)SET_BIT(SPI6->CR1,SPI_CR1_CSTART);
        ++g_led_bytes_sent;
    }
    __DSB();
    while(!(SPI6->SR&SPI_SR_TXC)) {
        if(HAL_GetTick()-start>=2U)goto failed;
    }
    SET_BIT(SPI6->CR1,SPI_CR1_CSUSP);
    while(!(SPI6->SR&SPI_SR_SUSP)) {
        if(HAL_GetTick()-start>=2U)goto failed;
    }
    g_led_last_sr=SPI6->SR;
    CLEAR_BIT(SPI6->CR1,SPI_CR1_SPE);
    SPI6->IFCR=SPI_IFCR_EOTC|SPI_IFCR_TXTFC|SPI_IFCR_SUSPC;
    return 1;
failed:
    g_led_last_sr=SPI6->SR;
    CLEAR_BIT(SPI6->CR1,SPI_CR1_SPE);
    ++g_led_errors;return 0;
}
static int SetColor(uint32_t grb) {
    static uint8_t pixels[224U+24U+224U]; /* > 280 us LOW before/after data */
    for (uint32_t bit=0;bit<24U;++bit)
        pixels[224U+bit] = (grb & (1UL<<(23U-bit))) ? 0x78U : 0x60U;
    if(!Transmit(pixels,sizeof(pixels)))return 0;
    current_grb=g_led_grb=grb; return 1;
}
int BoardLED_Init(void) {
    RCC_PeriphCLKInitTypeDef clock={0};
    clock.PeriphClockSelection=RCC_PERIPHCLK_SPI6;
    clock.Spi6ClockSelection=RCC_SPI6CLKSOURCE_HSE;
    if(HAL_RCCEx_PeriphCLKConfig(&clock)!=HAL_OK)return 0;
    __HAL_RCC_SPI6_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef pin={0};
    pin.Pin=GPIO_PIN_7;pin.Mode=GPIO_MODE_AF_PP;pin.Pull=GPIO_NOPULL;
    pin.Speed=GPIO_SPEED_FREQ_VERY_HIGH;pin.Alternate=GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOA,&pin);
    led_spi.Instance=SPI6;
    led_spi.Init.Mode=SPI_MODE_MASTER;
    led_spi.Init.Direction=SPI_DIRECTION_2LINES_TXONLY;
    led_spi.Init.DataSize=SPI_DATASIZE_8BIT;
    led_spi.Init.CLKPolarity=SPI_POLARITY_LOW;
    led_spi.Init.CLKPhase=SPI_PHASE_1EDGE;
    led_spi.Init.NSS=SPI_NSS_SOFT;
    led_spi.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_4;
    led_spi.Init.FirstBit=SPI_FIRSTBIT_MSB;
    led_spi.Init.TIMode=SPI_TIMODE_DISABLE;
    led_spi.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
    led_spi.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;
    led_spi.Init.FifoThreshold=SPI_FIFO_THRESHOLD_01DATA;
    led_spi.Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;
    if(HAL_SPI_Init(&led_spi)!=HAL_OK)return 0;
    return SetColor(REMOTE_LED_GREEN);
}
void BoardLED_Update(const DBUS_State *remote,uint32_t now) {
    uint32_t grb=DM_RemoteColor(remote,now);
    if(grb!=current_grb)(void)SetColor(grb);
}
