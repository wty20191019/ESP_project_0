#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


void app_main(void)
{
    
    
    
    
    while (1)
    {
        
        //printf("Hello world!\n");
        esp_log_write(ESP_LOG_INFO, "main", "Hello world!\n");
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
}
