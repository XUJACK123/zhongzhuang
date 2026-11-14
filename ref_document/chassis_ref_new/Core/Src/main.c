/* STM32H723 DM-MC02 RL wheel-leg firmware. Makefile is authoritative. */
#include "main.h"
#include "fdcan.h"
#include "board.h"
#include "imu.h"
#include "chassis.h"

#define HSE_MHZ 24U

#if HSE_MHZ == 24
/* 24 MHz / 3 = 8 MHz，VCO 8 x 68.75 = 550 MHz，FDCAN 12 x 16 / 2 = 96 MHz */
#define PLL1_M 3U
#define PLL1_N 68U
#define PLL1_FRACN 6144U
#define PLL1_VCI RCC_PLL1VCIRANGE_3
#define PLL2_M 2U
#define PLL2_N 16U
#define PLL2_VCI RCC_PLL2VCIRANGE_3
#elif HSE_MHZ == 25
/* 25 MHz / 25 = 1 MHz，VCO 1 x 550 = 550 MHz，FDCAN 1 x 192 / 2 = 96 MHz */
#define PLL1_M 25U
#define PLL1_N 550U
#define PLL1_FRACN 0U
#define PLL1_VCI RCC_PLL1VCIRANGE_0
#define PLL2_M 25U
#define PLL2_N 192U
#define PLL2_VCI RCC_PLL2VCIRANGE_0
#else
#error "HSE_MHZ must be 24 or 25"
#endif

static void SystemClock_Config(void);
static void PeriphCommonClock_Config(void);

int main(void)
{
    SCB_EnableICache();
    HAL_Init();
    SystemClock_Config();
    PeriphCommonClock_Config();
    if (!board_init()) Error_Handler();
    HAL_Delay(100);
    if (!can2_init()) Error_Handler();
    const uint8_t zero[8]={0};
    (void)can2_send(0x3fe,zero);
    (void)can2_send(0x200,zero);
    /* Init failure is visible as IMU inhibit; continue zero-output service. */
    (void)imu_init();
    chassis_init();
    board_watchdog_start();
    uint32_t sampled=HAL_GetTick()-1U;
    for (;;) {
        uint32_t now=HAL_GetTick();
        if (now!=sampled) { sampled=now; imu_service(now); }
        chassis_service(HAL_GetTick());
        __WFI();
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = PLL1_M;
    oscillator.PLL.PLLN = PLL1_N;
    oscillator.PLL.PLLP = 1;
    oscillator.PLL.PLLQ = 2;
    oscillator.PLL.PLLR = 2;
    oscillator.PLL.PLLRGE = PLL1_VCI;
    oscillator.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    oscillator.PLL.PLLFRACN = PLL1_FRACN;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                       RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clocks.AHBCLKDivider = RCC_HCLK_DIV2;
    clocks.APB3CLKDivider = RCC_APB3_DIV2;
    clocks.APB1CLKDivider = RCC_APB1_DIV2;
    clocks.APB2CLKDivider = RCC_APB2_DIV2;
    clocks.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

static void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef peripheral_clock = {0};

    peripheral_clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_SPI2;
    peripheral_clock.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    peripheral_clock.PLL2.PLL2M = PLL2_M;
    peripheral_clock.PLL2.PLL2N = PLL2_N;
    peripheral_clock.PLL2.PLL2P = 2;
    peripheral_clock.PLL2.PLL2Q = 2;
    peripheral_clock.PLL2.PLL2R = 2;
    peripheral_clock.PLL2.PLL2RGE = PLL2_VCI;
    peripheral_clock.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    peripheral_clock.PLL2.PLL2FRACN = 0;
    peripheral_clock.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clock) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
