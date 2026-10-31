#ifndef REMOTE_TEST_USART_H
#define REMOTE_TEST_USART_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_IDLE } HAL_UART_RxEventTypeTypeDef;
typedef struct { uint32_t disabled_it; } DMA_HandleTypeDef;
typedef struct {
    DMA_HandleTypeDef *hdmarx;
    uint32_t ErrorCode;
    HAL_UART_RxEventTypeTypeDef RxEventType;
} UART_HandleTypeDef;
extern UART_HandleTypeDef huart5;
enum { UART5_IRQn, DMA1_Stream0_IRQn };
#define DMA_IT_HT 1U
#define UART_CLEAR_PEF 1U
#define UART_CLEAR_FEF 2U
#define UART_CLEAR_NEF 4U
#define UART_CLEAR_OREF 8U
#define UART_CLEAR_IDLEF 16U
#define UART_RXDATA_FLUSH_REQUEST 1U
#define __HAL_DMA_DISABLE_IT(dma, it) ((dma)->disabled_it |= (it))
#define __HAL_UART_CLEAR_FLAG(uart, flags) ((void)(uart), (void)(flags))
#define __HAL_UART_SEND_REQ(uart, req) ((void)(uart), (void)(req))
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(const UART_HandleTypeDef *);
uint32_t HAL_GetTick(void);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_NVIC_EnableIRQ(int irq);
#endif
