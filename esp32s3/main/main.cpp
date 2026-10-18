#include "bridge.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "log_forward.h"
#include "net_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#if defined(CONFIG_HB_ENABLE_IMU) && !defined(CONFIG_ESP_CONSOLE_NONE)
#error "The IMU uses UART0: set Component config > ESP System Settings > Channel for console output = None"
#endif

static const char* TAG = "main";

extern "C" void app_main(void)
{
    log_forward_install();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#ifndef CONFIG_HB_WIFI_MODE_OFF
    ESP_ERROR_CHECK(wifi_start());
#endif

    ESP_LOGI(TAG, "hoverboard_ros started, UDP port %d", CONFIG_HB_UDP_PORT);
    bridge_run();
}
