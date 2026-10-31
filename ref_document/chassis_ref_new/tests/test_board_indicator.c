#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "board_indicator.h"
#include "remote_control.h"

SPI_TypeDef test_spi;
static remote_control_snapshot_t input;
static uint32_t tick, blocked_flag, pwm_value;
static unsigned int writes, transfers;
static uint8_t captured[472];
static bool last_byte_pending;
void remote_control_get_snapshot(remote_control_snapshot_t *s) { *s = input; }
uint32_t HAL_GetTick(void) { return tick++; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return 137500000U; }
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *c) { (void)c; return HAL_OK; }
void HAL_GPIO_Init(void *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *s) { s->State = HAL_SPI_STATE_READY; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *t) { (void)t; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *t, TIM_OC_InitTypeDef *c, uint32_t ch)
{ (void)t; (void)c; (void)ch; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t, uint32_t ch)
{ (void)t; (void)ch; return HAL_OK; }
void test_pwm(uint32_t v) { pwm_value = v; }
void test_spi_disable(SPI_HandleTypeDef *s) { s->Instance->CR1 = 0U; last_byte_pending = false; }
void test_spi_enable(SPI_HandleTypeDef *s)
{
    assert(s->Instance->CR2 == 0U && s->Instance->IER == 0U);
    writes = 0U;
    ++transfers;
    last_byte_pending = false;
}
bool test_spi_flag(SPI_HandleTypeDef *s, uint32_t f)
{
    /* 在下一次 TXP/TXC 检查时捕获上一轮真正写入 TXDR 的字节。 */
    if (last_byte_pending)
    {
        assert(writes < sizeof(captured));
        captured[writes++] = (uint8_t)s->Instance->TXDR;
        last_byte_pending = false;
    }
    if (f == blocked_flag) return false;
    if (f == SPI_FLAG_TXP) last_byte_pending = true;
    if (f == SPI_FLAG_TXC) assert(writes == sizeof(captured));
    if (f == SPI_FLAG_SUSP) assert(s->Instance->CR1 & SPI_CR1_CSUSP);
    return true;
}
static void expect_color(uint32_t grb)
{
    assert(writes == sizeof(captured));
    for (unsigned int i = 0U; i < sizeof(captured); ++i)
    {
        uint8_t expected = 0U;
        if (i >= 224U && i < 248U) expected = grb & (1UL << (247U - i)) ? 0x78U : 0x60U;
        assert(captured[i] == expected);
    }
}
static void service(void) { board_indicator_service(tick); }
int main(void)
{
    assert(board_indicator_init());
    assert(transfers == 0U && board_indicator_debug.init_stage == 6U); /* 初始化不发送。 */
    service();
    assert(board_indicator_debug.tx_success_count == 1U);
    expect_color(0x101010U);
    service();
    assert(transfers == 1U); /* 未变化时不重复发送。 */
    input.valid = true;
    input.frame.switch_right = RC_SWITCH_UP;
    input.update_time_ms = tick;
    service();
    expect_color(0x100000U);
    input.frame.switch_right = RC_SWITCH_MID;
    service();
    expect_color(0x001000U);
    input.frame.switch_right = RC_SWITCH_DOWN;
    service();
    expect_color(0x000010U);
    tick += 101U;
    service();
    expect_color(0x101010U);

    input.valid = true;
    input.update_time_ms = tick;
    input.frame.switch_right = RC_SWITCH_UP;
    blocked_flag = SPI_FLAG_TXC;
    uint32_t good = board_indicator_debug.tx_success_count;
    service();
    assert(board_indicator_debug.tx_failure_count == 1U);
    assert(board_indicator_debug.last_spi_error == HAL_SPI_ERROR_TIMEOUT);
    assert(board_indicator_debug.tx_stage == 2U && board_indicator_debug.tx_success_count == good);
    unsigned int attempts = transfers;
    service();
    assert(transfers == attempts); /* 超时后 100 ms 内不反复占用任务。 */
    blocked_flag = 0U;
    tick += 101U;
    input.update_time_ms = tick;
    service();
    assert(board_indicator_debug.tx_success_count == good + 1U);
    expect_color(0x100000U); /* 失败未错误缓存为已发送，稍后可以恢复。 */
    tick += 1000U;
    service();
    assert(pwm_value == 0U);
    puts("indicator tests passed (mock registers; waveform requires hardware validation)");
    return 0;
}
