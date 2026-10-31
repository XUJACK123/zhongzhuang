#include "board_indicator.h"
#include "main.h"
#include "remote_control.h"
#include "chassis_safety.h"

static SPI_HandleTypeDef indicator_spi;
static TIM_HandleTypeDef buzzer_timer;
static bool ready;
static bool led_ready;
static uint32_t current_grb;
static bool beep_started;
static uint32_t beep_start_ms;
remote_control_snapshot_t remote;

/* GRB：每个颜色分量为 16，保持低亮度；只在颜色变化时发送。 */
static bool indicator_set_color(uint32_t grb)
{
    /* 静态缓冲避免占用低优先级任务的栈；仅初始化和该任务调用。 */
    static uint8_t pixels[224U + 24U + 224U];
    for (uint32_t bit = 0U; bit < 24U; ++bit)
        pixels[224U + bit] = (grb & (1UL << (23U - bit))) ? 0x78U : 0x60U;
    if (HAL_SPI_Transmit(&indicator_spi, pixels, sizeof(pixels), 10U) != HAL_OK)
        return false;
    current_grb = grb;
    return true;
}

static void indicator_update_remote(void)
{
    
    remote_control_get_snapshot(&remote);
    /* 在快照之后取时间，避免刚收到的新帧时间戳晚于本次 service 入参。 */
    const uint32_t now_ms = HAL_GetTick();
    uint32_t grb = 0x101010UL; /* 未连接、超时或非法档位：白色。 */
    if (remote.valid && now_ms - remote.update_time_ms <= CHASSIS_DBUS_TIMEOUT_MS)
    {
        switch (remote.frame.switch_left) /* 协议 S1 三档拨杆。 */
        {
        case RC_SWITCH_UP:   grb = 0x100000UL; break; /* 绿色 */
        case RC_SWITCH_MID:  grb = 0x001000UL; break; /* 红色 */
        case RC_SWITCH_DOWN: grb = 0x000010UL; break; /* 蓝色 */
        default: break;
        }
    }
    if (grb != current_grb) (void)indicator_set_color(grb);
}

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
    /* 上电尚未收到遥控数据：先显示白色。前后低电平用于 WS2812 复位。 */
    if (!indicator_set_color(0x101010UL)) return false;
    led_ready = true;

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
    if (led_ready) indicator_update_remote();
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
