#include "main.h"
#include "stm32h7xx_it.h"
#include "fdcan.h"
#include "board.h"

/* Retained across watchdog/software reset; not valid across power loss.
 * magic, exception kind, CFSR, HFSR, MMFAR, BFAR, ICSR, tick. */
volatile uint32_t g_exception_record[8] __attribute__((section(".FAULT_RETAIN")));
static void record_exception(uint32_t kind) {
    __disable_irq();
    g_exception_record[0]=0;
    g_exception_record[1]=kind; g_exception_record[2]=SCB->CFSR;
    g_exception_record[3]=SCB->HFSR; g_exception_record[4]=SCB->MMFAR;
    g_exception_record[5]=SCB->BFAR; g_exception_record[6]=SCB->ICSR;
    g_exception_record[7]=HAL_GetTick();
    __DSB(); g_exception_record[0]=0x45584331U; __DSB();
    while (1) {} /* watchdog still stops/restarts a stuck control path */
}

void NMI_Handler(void)
{
    while (1)
    {
    }
}

void HardFault_Handler(void)
{
    record_exception(3);
}

void MemManage_Handler(void)
{
    record_exception(4);
}

void BusFault_Handler(void)
{
    record_exception(5);
}

void UsageFault_Handler(void)
{
    record_exception(6);
}

void DebugMon_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void FDCAN1_IT0_IRQHandler(void)
{
}

void FDCAN2_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan2);
}

void FDCAN3_IT0_IRQHandler(void)
{
}

void UART5_IRQHandler(void) { board_uart_irq(); }
void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12); }
