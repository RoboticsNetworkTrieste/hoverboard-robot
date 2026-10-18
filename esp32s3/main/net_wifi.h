#pragma once

#include "esp_err.h"

// Starts WiFi in the Kconfig-selected mode (station or access point) and mDNS.
// In station mode the IP is static (HB_WIFI_STA_STATIC_IP) or assigned by DHCP.
esp_err_t wifi_start();
