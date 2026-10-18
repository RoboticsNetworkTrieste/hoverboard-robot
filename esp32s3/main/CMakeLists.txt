idf_component_register(
    SRCS "main.cpp" "net_wifi.cpp"
    INCLUDE_DIRS "."
    REQUIRES hover_core esp_wifi esp_netif esp_event nvs_flash)
