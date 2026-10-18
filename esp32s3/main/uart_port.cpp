#include "uart_port.h"

#include "esp_check.h"

static const char* TAG = "uart";

esp_err_t UartPort::open(uart_port_t port, int rx_gpio, int tx_gpio, int baud, size_t rx_buffer)
{
    uart_config_t cfg = {};
    cfg.baud_rate = baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_RETURN_ON_ERROR(uart_param_config(port, &cfg), TAG, "param_config %d", port);
    ESP_RETURN_ON_ERROR(uart_set_pin(port, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG,
                        "set_pin %d", port);
    ESP_RETURN_ON_ERROR(uart_driver_install(port, (int)rx_buffer, 0, 0, nullptr, 0), TAG, "driver_install %d", port);
    port_ = port;
    open_ = true;
    return ESP_OK;
}

size_t UartPort::read(uint8_t* buf, size_t max)
{
    if (!open_) return 0;
    int n = uart_read_bytes(port_, buf, (uint32_t)max, 0);
    return n > 0 ? (size_t)n : 0;
}

void UartPort::write(const uint8_t* data, size_t len)
{
    if (open_) uart_write_bytes(port_, data, len);
}
