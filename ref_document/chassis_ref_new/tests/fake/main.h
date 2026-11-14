#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
typedef struct { unsigned unused; } UART_HandleTypeDef;
extern uint32_t SystemCoreClock;
uint32_t HAL_GetTick(void);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
#endif
