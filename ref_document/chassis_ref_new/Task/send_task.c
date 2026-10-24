#include "send_task.h"

#include "FreeRTOS.h"
#include "USART7_Receive.h"
#include "task.h"

void send_task(void const *argument)
{
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(100U));
    for (;;)
    {
        user_send_data();
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
