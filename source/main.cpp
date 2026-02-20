#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "btHelper.h"




int main()
{
    stdio_init_all();

    // xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(bt_task, "BT_Task", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while(1){};
}
