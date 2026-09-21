#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "hoverboard_ros skeleton, UDP port %d", CONFIG_HB_UDP_PORT);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
