#pragma once

#include <cstddef>
#include <cstdint>

namespace util {

// LDROBOT command frames for the STL-50B2. See docs/lidar/STL-50B2.md.
inline constexpr size_t kLidarFrameSize = 8;
inline constexpr uint8_t kLidarStart[kLidarFrameSize] = {0x54, 0xA0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E};
inline constexpr uint8_t kLidarStop[kLidarFrameSize] = {0x54, 0xA1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x4A};

// Keeps the LIDAR streaming by resending the start command while the line is silent.
// A start sent before the sensor has booted is lost, and the LIDAR then stays silent
// forever, so one shot at power-up is not enough.
class LidarStarter {
public:
    explicit LidarStarter(uint32_t retry_ms) : retry_ms_(retry_ms) {}

    // Call whenever bytes arrive from the LIDAR.
    void onBytes(uint32_t now_ms)
    {
        last_rx_ms_ = now_ms;
        streaming_ = true;
    }

    // True when the caller must write kLidarStart to the LIDAR UART.
    bool tick(uint32_t now_ms)
    {
        if (!sent_) {  // First call: start straight away
            sent_ = true;
            last_tx_ms_ = now_ms;
            return true;
        }
        if (streaming_ && (now_ms - last_rx_ms_) < retry_ms_) return false;  // Data is flowing
        if ((now_ms - last_tx_ms_) < retry_ms_) return false;                // Last command still fresh
        streaming_ = false;
        last_tx_ms_ = now_ms;
        return true;
    }

private:
    uint32_t retry_ms_;
    uint32_t last_rx_ms_ = 0;
    uint32_t last_tx_ms_ = 0;
    bool streaming_ = false;
    bool sent_ = false;
};

}  // namespace util
