#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/uart.h"
#include "esp_err.h"

// Non-blocking UART access with a driver-side RX ring buffer.
class UartPort {
public:
    esp_err_t open(uart_port_t port, int rx_gpio, int tx_gpio, int baud, size_t rx_buffer);
    size_t read(uint8_t* buf, size_t max);
    void write(const uint8_t* data, size_t len);
    bool isOpen() const { return open_; }

private:
    uart_port_t port_ = UART_NUM_MAX;
    bool open_ = false;
};
