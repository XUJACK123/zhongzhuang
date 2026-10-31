#include "send_task.h"

#include "FreeRTOS.h"
#include "USART7_Receive.h"
#include "task.h"
#include "board_indicator.h"
#include "main.h"
#include "remote_control.h"

void send_task(void const *argument)
{
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(100U));
    for (;;)
    {
        remote_control_service();
        board_indicator_service(HAL_GetTick());
        user_send_data();
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
