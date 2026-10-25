idf_component_register(
    SRCS "main.cpp" "net_wifi.cpp" "net_usb.cpp" "udp_link.cpp" "uart_port.cpp" "log_forward.cpp" "bridge.cpp"
    INCLUDE_DIRS "."
    REQUIRES hover_core esp_wifi esp_netif esp_event nvs_flash esp_driver_uart esp_timer lwip)
