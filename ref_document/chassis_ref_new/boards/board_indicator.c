#include "board_indicator.h"
#include "main.h"

static SPI_HandleTypeDef indicator_spi;
static TIM_HandleTypeDef buzzer_timer;
static bool ready;
static bool beep_started;
static uint32_t beep_start_ms;

bool board_indicator_init(void)
{
    RCC_PeriphCLKInitTypeDef clocks = {0};
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef pwm = {0};
    /* 厂家 WS2812 例程使用 HSE 24 MHz / 4 = 6 MHz。 */
    clocks.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    clocks.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&clocks) != HAL_OK) return false;
    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_7; /* 只复用 MOSI，不占用外接接口上的 PA5/PA6。 */
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOA, &gpio);
    indicator_spi.Instance = SPI6;
    indicator_spi.Init.Mode = SPI_MODE_MASTER;
    indicator_spi.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    indicator_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    indicator_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    indicator_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    indicator_spi.Init.NSS = SPI_NSS_SOFT;
    indicator_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    indicator_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    indicator_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    indicator_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    indicator_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    indicator_spi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    indicator_spi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    if (HAL_SPI_Init(&indicator_spi) != HAL_OK) return false;
    /* GRB，每个 LED 数据位占一个 SPI 字节。前后各 300 us 低电平复位。 */
    uint8_t pixels[224U + 24U + 224U] = {0};
    const uint32_t green = 0x100000UL; /* G=16，低亮度绿灯常亮。 */
    for (uint32_t bit = 0U; bit < 24U; ++bit)
        pixels[224U + bit] = (green & (1UL << (23U - bit))) ? 0x78U : 0x60U;
    if (HAL_SPI_Transmit(&indicator_spi, pixels, sizeof(pixels), 10U) != HAL_OK) return false;

    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_15;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM12;
    HAL_GPIO_Init(GPIOB, &gpio);
    /* 当前 APB1=/2、TIMPRE=0，定时器时钟为 PCLK1 的两倍。 */
    const uint32_t timer_clock = 2U * HAL_RCC_GetPCLK1Freq();
    buzzer_timer.Instance = TIM12;
    buzzer_timer.Init.Prescaler = timer_clock / 1000000U - 1U;
    buzzer_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    buzzer_timer.Init.Period = 250U - 1U; /* 4 kHz */
    buzzer_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    buzzer_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&buzzer_timer) != HAL_OK) return false;
    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.Pulse = 0U;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&buzzer_timer, &pwm, TIM_CHANNEL_2) != HAL_OK) return false;
    ready = HAL_TIM_PWM_Start(&buzzer_timer, TIM_CHANNEL_2) == HAL_OK;
    return ready;
}

void board_indicator_service(uint32_t now_ms)
{
    if (!ready) return;
    if (!beep_started)
    {
        beep_start_ms = now_ms;
        beep_started = true;
    }
    const uint32_t elapsed = now_ms - beep_start_ms;
    /* 三个 100 ms 音，间隔 150 ms；之后保持静音，不阻塞电机任务。 */
    const bool on = elapsed < 750U && elapsed % 250U < 100U;
    __HAL_TIM_SET_COMPARE(&buzzer_timer, TIM_CHANNEL_2, on ? 125U : 0U);
    if (elapsed >= 750U) ready = false; /* 防止时钟回绕后再次响起。 */
}
