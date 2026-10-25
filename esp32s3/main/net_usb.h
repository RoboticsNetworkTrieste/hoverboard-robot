#pragma once

#include "esp_err.h"

// Starts TinyUSB CDC-NCM and a lwIP interface at 192.168.7.1 with a DHCP server.
esp_err_t usb_net_start();
