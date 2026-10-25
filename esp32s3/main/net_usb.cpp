#include "net_usb.h"

#include <cstdlib>
#include <cstring>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "tinyusb.h"
#include "tinyusb_net.h"

static const char* TAG = "usb_net";

static const uint8_t kHostSideMac[6] = {0x02, 0x48, 0x42, 0x00, 0x07, 0x02};  // Seen by the PC
static const uint8_t kEspSideMac[6] = {0x02, 0x48, 0x42, 0x00, 0x07, 0x01};   // lwIP interface

static esp_netif_t* s_netif = nullptr;

// Returns ESP_ERR_INVALID_STATE at once when no host is connected, so sends never block unplugged.
static esp_err_t netifTransmit(void*, void* buffer, size_t len)
{
    return tinyusb_net_send_sync(buffer, (uint16_t)len, nullptr, pdMS_TO_TICKS(100));
}

static void netifFreeRx(void*, void* buffer)
{
    free(buffer);
}

// USB frames are copied because TinyUSB reuses its buffer after the callback; lwIP frees the copy.
static esp_err_t usbReceive(void* buffer, uint16_t len, void*)
{
    if (s_netif == nullptr) return ESP_OK;
    void* copy = malloc(len);
    if (copy == nullptr) return ESP_ERR_NO_MEM;
    std::memcpy(copy, buffer, len);
    return esp_netif_receive(s_netif, copy, len, nullptr);
}

esp_err_t usb_net_start()
{
    esp_netif_ip_info_t ip = {};
    ip.ip.addr = ESP_IP4TOADDR(192, 168, 7, 1);
    ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
    ip.gw.addr = 0;

    esp_netif_inherent_config_t base = {};
    base.flags = (esp_netif_flags_t)(ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP);
    std::memcpy(base.mac, kEspSideMac, sizeof(kEspSideMac));
    base.ip_info = &ip;
    base.if_key = "USB_NCM";
    base.if_desc = "usb ncm";
    base.route_prio = 10;

    esp_netif_driver_ifconfig_t driver = {};
    driver.handle = (void*)1;  // Non-null handle required by esp_netif
    driver.transmit = netifTransmit;
    driver.driver_free_rx_buffer = netifFreeRx;

    esp_netif_config_t cfg = {};
    cfg.base = &base;
    cfg.driver = &driver;
    cfg.stack = ESP_NETIF_NETSTACK_DEFAULT_ETH;

    s_netif = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(s_netif != nullptr, ESP_FAIL, TAG, "esp_netif_new");

    uint8_t offer = 0;  // Do not offer a default route or DNS: the PC keeps its internet route
    esp_netif_dhcps_option(s_netif, ESP_NETIF_OP_SET, ESP_NETIF_ROUTER_SOLICITATION_ADDRESS, &offer, sizeof(offer));
    esp_netif_action_start(s_netif, nullptr, 0, nullptr);

    const tinyusb_config_t tusb_cfg = {};  // Default descriptors from Kconfig (NCM enabled)
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "tinyusb_driver_install");

    tinyusb_net_config_t net_cfg = {};
    std::memcpy(net_cfg.mac_addr, kHostSideMac, sizeof(kHostSideMac));
    net_cfg.on_recv_callback = usbReceive;
    ESP_RETURN_ON_ERROR(tinyusb_net_init(TINYUSB_USBDEV_0, &net_cfg), TAG, "tinyusb_net_init");

    ESP_LOGI(TAG, "USB tethering at 192.168.7.1");
    return ESP_OK;
}
