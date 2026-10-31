#ifndef TEST_INDICATOR_MAIN_H
#define TEST_INDICATOR_MAIN_H
#include <stdint.h>
#include <stdbool.h>
#define __IO volatile
#define __DSB() ((void)0)
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t CR1, CR2, SR, IER, IFCR, TXDR; } SPI_TypeDef;
extern SPI_TypeDef test_spi;
#define SPI6 (&test_spi)
#define TIM12 ((void *)12)
#define GPIOA ((void *)1)
#define GPIOB ((void *)2)
typedef struct { uint32_t Mode; uint32_t Direction; uint32_t DataSize; uint32_t CLKPolarity; uint32_t CLKPhase; uint32_t NSS; uint32_t BaudRatePrescaler; uint32_t FirstBit; uint32_t TIMode; uint32_t CRCCalculation; uint32_t NSSPMode; uint32_t FifoThreshold; uint32_t MasterKeepIOState; uint32_t Prescaler; uint32_t CounterMode; uint32_t Period; uint32_t ClockDivision; uint32_t AutoReloadPreload; } TestInit;
typedef struct { SPI_TypeDef *Instance; TestInit Init; uint32_t ErrorCode, State; } SPI_HandleTypeDef;
typedef struct { void *Instance; TestInit Init; } TIM_HandleTypeDef;
typedef struct { uint32_t PeriphClockSelection, Spi6ClockSelection; } RCC_PeriphCLKInitTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t OCMode, Pulse, OCPolarity, OCFastMode; } TIM_OC_InitTypeDef;
#define SPI_SR_MODF 0x200U
#define SPI_SR_UDR 0x20U
#define SPI_SR_TIFRE 0x100U
#define SPI_FLAG_TXP 2U
#define SPI_FLAG_TXC 0x1000U
#define SPI_FLAG_SUSP 0x800U
#define SPI_IFCR_EOTC 8U
#define SPI_IFCR_TXTFC 16U
#define SPI_IFCR_SUSPC 0x800U
#define SPI_IFCR_MODFC 0x200U
#define SPI_IFCR_UDRC 0x20U
#define SPI_IFCR_TIFREC 0x100U
#define SPI_CR1_CSTART 0x200U
#define SPI_CR1_CSUSP 0x400U
#define HAL_SPI_ERROR_NONE 0U
#define HAL_SPI_ERROR_MODF 1U
#define HAL_SPI_ERROR_UDR 0x80U
#define HAL_SPI_ERROR_FRE 8U
#define HAL_SPI_ERROR_TIMEOUT 0x100U
#define HAL_SPI_STATE_BUSY_TX 3U
#define HAL_SPI_STATE_READY 1U
#define SET_BIT(reg, bit) ((reg) |= (bit))
#define __HAL_SPI_DISABLE(h) test_spi_disable(h)
#define __HAL_SPI_ENABLE(h) test_spi_enable(h)
#define __HAL_SPI_GET_FLAG(h, f) test_spi_flag(h, f)
#define __HAL_TIM_SET_COMPARE(h, ch, val) test_pwm(val)
#define RCC_PERIPHCLK_SPI6 100U
#define RCC_SPI6CLKSOURCE_HSE 101U
#define GPIO_PIN_7 102U
#define GPIO_MODE_AF_PP 103U
#define GPIO_NOPULL 104U
#define GPIO_SPEED_FREQ_VERY_HIGH 105U
#define GPIO_AF8_SPI6 106U
#define SPI_MODE_MASTER 107U
#define SPI_DIRECTION_2LINES_TXONLY 108U
#define SPI_DATASIZE_8BIT 109U
#define SPI_POLARITY_LOW 110U
#define SPI_PHASE_1EDGE 111U
#define SPI_NSS_SOFT 112U
#define SPI_BAUDRATEPRESCALER_4 113U
#define SPI_FIRSTBIT_MSB 114U
#define SPI_TIMODE_DISABLE 115U
#define SPI_CRCCALCULATION_DISABLE 116U
#define SPI_NSS_PULSE_DISABLE 117U
#define SPI_FIFO_THRESHOLD_01DATA 118U
#define SPI_MASTER_KEEP_IO_STATE_ENABLE 119U
#define GPIO_PIN_15 120U
#define GPIO_SPEED_FREQ_LOW 121U
#define GPIO_AF2_TIM12 122U
#define TIM_COUNTERMODE_UP 123U
#define TIM_CLOCKDIVISION_DIV1 124U
#define TIM_AUTORELOAD_PRELOAD_DISABLE 125U
#define TIM_OCMODE_PWM1 126U
#define TIM_OCPOLARITY_HIGH 127U
#define TIM_OCFAST_DISABLE 128U
#define TIM_CHANNEL_2 129U
#define __HAL_RCC_SPI6_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI6_FORCE_RESET() ((void)0)
#define __HAL_RCC_SPI6_RELEASE_RESET() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM12_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
void test_spi_disable(SPI_HandleTypeDef *);
void test_spi_enable(SPI_HandleTypeDef *);
bool test_spi_flag(SPI_HandleTypeDef *, uint32_t);
void test_pwm(uint32_t);
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *);
void HAL_GPIO_Init(void *, GPIO_InitTypeDef *);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *, TIM_OC_InitTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
#endif
