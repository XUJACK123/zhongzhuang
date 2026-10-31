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
static bool last_tx_failed;
volatile board_indicator_debug_t board_indicator_debug;
static uint32_t beep_start_ms;
remote_control_snapshot_t remote;

/* SPI6 专用 TX-only：TSIZE=0，不等待本板已实测失败的 EOT。
 * 全部字节入 FIFO 后，等 TXC（FIFO 和移位寄存器均发送完成），再挂起并关闭。
 * 参考 RM0468 SPI 主机结束流程及 ES0491 的 TSIZE=0 方式；不改公共 HAL。 */
static bool indicator_wait_flag(uint32_t flag, uint32_t started_ms)
{
    for (;;)
    {
        const uint32_t sr = indicator_spi.Instance->SR;
        if (sr & (SPI_SR_MODF | SPI_SR_UDR | SPI_SR_TIFRE))
        {
            if (sr & SPI_SR_MODF) indicator_spi.ErrorCode |= HAL_SPI_ERROR_MODF;
            if (sr & SPI_SR_UDR) indicator_spi.ErrorCode |= HAL_SPI_ERROR_UDR;
            if (sr & SPI_SR_TIFRE) indicator_spi.ErrorCode |= HAL_SPI_ERROR_FRE;
            return false;
        }
        if (__HAL_SPI_GET_FLAG(&indicator_spi, flag)) return true;
        if ((uint32_t)(HAL_GetTick() - started_ms) >= 10U)
        {
            indicator_spi.ErrorCode |= HAL_SPI_ERROR_TIMEOUT;
            return false;
        }
    }
}

static bool indicator_set_color(uint32_t grb)
{
    static uint8_t pixels[224U + 24U + 224U];
    const uint32_t started_ms = HAL_GetTick();
    bool success = false;
    ++board_indicator_debug.tx_attempt_count;
    board_indicator_debug.last_tx_ms = started_ms;
    for (uint32_t bit = 0U; bit < 24U; ++bit)
        pixels[224U + bit] = (grb & (1UL << (23U - bit))) ? 0x78U : 0x60U;

    indicator_spi.ErrorCode = HAL_SPI_ERROR_NONE;
    indicator_spi.State = HAL_SPI_STATE_BUSY_TX;
    __HAL_SPI_DISABLE(&indicator_spi);
    indicator_spi.Instance->IER = 0U; /* 此专用发送使用轮询，不启用 TXC/TXP 中断。 */
    indicator_spi.Instance->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_SUSPC |
                                  SPI_IFCR_MODFC | SPI_IFCR_UDRC | SPI_IFCR_TIFREC;
    indicator_spi.Instance->CR2 = 0U; /* TSIZE=0, TSER=0 */
    __HAL_SPI_ENABLE(&indicator_spi);
    SET_BIT(indicator_spi.Instance->CR1, SPI_CR1_CSTART);
    board_indicator_debug.tx_stage = 1U;
    for (uint32_t i = 0U; i < sizeof(pixels); ++i)
    {
        if (!indicator_wait_flag(SPI_FLAG_TXP, started_ms)) goto finished;
        *(__IO uint8_t *)&indicator_spi.Instance->TXDR = pixels[i];
    }
    __DSB();
    board_indicator_debug.tx_stage = 2U;
    if (!indicator_wait_flag(SPI_FLAG_TXC, started_ms)) goto finished;
    board_indicator_debug.tx_stage = 3U;
    SET_BIT(indicator_spi.Instance->CR1, SPI_CR1_CSUSP);
    if (!indicator_wait_flag(SPI_FLAG_SUSP, started_ms)) goto finished;
    success = true;

finished:
    board_indicator_debug.last_spi_sr = indicator_spi.Instance->SR;
    board_indicator_debug.last_spi_error = indicator_spi.ErrorCode;
    __HAL_SPI_DISABLE(&indicator_spi); /* 超时也停止 SPI 并清空 FIFO，下次从完整帧重试。 */
    indicator_spi.Instance->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_SUSPC |
                                  SPI_IFCR_MODFC | SPI_IFCR_UDRC | SPI_IFCR_TIFREC;
    indicator_spi.State = HAL_SPI_STATE_READY;
    last_tx_failed = !success;
    if (!success)
    {
        ++board_indicator_debug.tx_failure_count;
        return false;
    }
    board_indicator_debug.tx_stage = 4U;
    ++board_indicator_debug.tx_success_count;
    current_grb = grb; /* 只有确认完成，才把颜色记为已发送。 */
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
        switch (remote.frame.switch_right) /* 协议 S1 / HKU s[0]：右拨杆。 */
        {
        case RC_SWITCH_UP:   grb = 0x100000UL; break; /* 绿色 */
        case RC_SWITCH_MID:  grb = 0x001000UL; break; /* 红色 */
        case RC_SWITCH_DOWN: grb = 0x000010UL; break; /* 蓝色 */
        default: break;
        }
    }
    if (last_tx_failed && (uint32_t)(now_ms - board_indicator_debug.last_tx_ms) < 100U) return;
    if (grb != current_grb) (void)indicator_set_color(grb);
}

bool board_indicator_init(void)
{
    RCC_PeriphCLKInitTypeDef clocks = {0};
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef pwm = {0};
    /* 厂家 WS2812 例程使用 HSE 24 MHz / 4 = 6 MHz。 */
    board_indicator_debug.init_stage = 1U;
    clocks.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    clocks.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE;
    board_indicator_debug.last_hal_status = HAL_RCCEx_PeriphCLKConfig(&clocks);
    if (board_indicator_debug.last_hal_status != HAL_OK) return false;
    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_SPI6_FORCE_RESET();
    __HAL_RCC_SPI6_RELEASE_RESET();
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
    board_indicator_debug.init_stage = 2U;
    board_indicator_debug.last_hal_status = HAL_SPI_Init(&indicator_spi);
    if (board_indicator_debug.last_hal_status != HAL_OK) return false;
    /* 与 HKU 一样不在调度前发送；非法缓存色保证任务首次一定写灯。 */
    current_grb = UINT32_MAX;
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
    board_indicator_debug.init_stage = 3U;
    board_indicator_debug.last_hal_status = HAL_TIM_PWM_Init(&buzzer_timer);
    if (board_indicator_debug.last_hal_status != HAL_OK) return false;
    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.Pulse = 0U;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;
    board_indicator_debug.init_stage = 4U;
    board_indicator_debug.last_hal_status = HAL_TIM_PWM_ConfigChannel(&buzzer_timer, &pwm, TIM_CHANNEL_2);
    if (board_indicator_debug.last_hal_status != HAL_OK) return false;
    board_indicator_debug.init_stage = 5U;
    board_indicator_debug.last_hal_status = HAL_TIM_PWM_Start(&buzzer_timer, TIM_CHANNEL_2);
    ready = board_indicator_debug.last_hal_status == HAL_OK;
    if (ready) board_indicator_debug.init_stage = 6U;
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
