#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led.h"

void app_main(void)
{
    
    led_init();

    while (1)
    {

        led_set_rgb(1, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        led_set_rgb(0, 1, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        led_set_rgb(0, 0, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        led_clear();

        //printf("Hello world!\n");
        esp_log_write(ESP_LOG_INFO, "main", "Hello world!\n");
        
        vTaskDelay(pdMS_TO_TICKS(100));

    }
}
