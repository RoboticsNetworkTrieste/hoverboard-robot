#include "net_wifi.h"

#include <cstring>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "sdkconfig.h"

static const char* TAG = "wifi";

#ifdef CONFIG_HB_WIFI_STA_STATIC_IP
// Replaces the DHCP client with the Kconfig address. No DNS server is set: the bridge never resolves names.
static esp_err_t setStaticIp(esp_netif_t* sta)
{
    esp_netif_ip_info_t info = {};
    if (esp_netif_str_to_ip4(CONFIG_HB_WIFI_STA_IP, &info.ip) != ESP_OK ||
        esp_netif_str_to_ip4(CONFIG_HB_WIFI_STA_NETMASK, &info.netmask) != ESP_OK ||
        esp_netif_str_to_ip4(CONFIG_HB_WIFI_STA_GATEWAY, &info.gw) != ESP_OK) {
        ESP_LOGE(TAG, "Invalid static IP settings in menuconfig");
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = esp_netif_dhcpc_stop(sta);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        return err;
    }
    return esp_netif_set_ip_info(sta, &info);
}
#endif

static void onWifiEvent(void*, esp_event_base_t base, int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Disconnected from %s, retrying", CONFIG_HB_WIFI_SSID);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(data);
        ESP_LOGI(TAG, "Station IP " IPSTR, IP2STR(&event->ip_info.ip));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Client joined access point %s", CONFIG_HB_WIFI_SSID);
    }
}

esp_err_t wifi_start()
{
    if (std::strlen(CONFIG_HB_WIFI_PASSWORD) < 8) {
        ESP_LOGE(TAG, "WiFi password must have at least 8 characters");
        return ESP_ERR_INVALID_ARG;
    }

#ifdef CONFIG_HB_WIFI_MODE_STA
    esp_netif_t* sta = esp_netif_create_default_wifi_sta();
#ifdef CONFIG_HB_WIFI_STA_STATIC_IP
    ESP_RETURN_ON_ERROR(setStaticIp(sta), TAG, "static IP");
#endif
#else
    esp_netif_t* ap = esp_netif_create_default_wifi_ap();  // 192.168.4.1 with DHCP server
    uint8_t offer = 0;                                       // Do not offer a default route or DNS
    esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET, ESP_NETIF_ROUTER_SOLICITATION_ADDRESS, &offer, sizeof(offer));
#endif

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "esp_wifi_init");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &onWifiEvent, nullptr), TAG,
                        "wifi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &onWifiEvent, nullptr), TAG,
                        "ip handler");

    wifi_config_t cfg = {};
#ifdef CONFIG_HB_WIFI_MODE_STA
    strlcpy((char*)cfg.sta.ssid, CONFIG_HB_WIFI_SSID, sizeof(cfg.sta.ssid));
    strlcpy((char*)cfg.sta.password, CONFIG_HB_WIFI_PASSWORD, sizeof(cfg.sta.password));
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set_mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &cfg), TAG, "set_config");
#else
    strlcpy((char*)cfg.ap.ssid, CONFIG_HB_WIFI_SSID, sizeof(cfg.ap.ssid));
    strlcpy((char*)cfg.ap.password, CONFIG_HB_WIFI_PASSWORD, sizeof(cfg.ap.password));
    cfg.ap.ssid_len = (uint8_t)std::strlen(CONFIG_HB_WIFI_SSID);
    cfg.ap.channel = CONFIG_HB_WIFI_AP_CHANNEL;
    cfg.ap.max_connection = 4;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "set_mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &cfg), TAG, "set_config");
#endif

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start");
    esp_wifi_set_ps(WIFI_PS_NONE);  // Lowest latency

    ESP_RETURN_ON_ERROR(mdns_init(), TAG, "mdns_init");
    mdns_hostname_set(CONFIG_HB_HOSTNAME);
    ESP_LOGI(TAG, "WiFi started, mDNS name %s.local", CONFIG_HB_HOSTNAME);
    return ESP_OK;
}
