#pragma once

#include <cstddef>
#include <cstdint>

namespace wt901 {

struct Sample {
    int16_t acc[3];    // Raw, 32768 = 16 g
    int16_t gyro[3];   // Raw, 32768 = 2000 deg/s
    int16_t angle[3];  // Raw roll/pitch/yaw, 32768 = 180 deg
};

// Parses WitMotion 11-byte packets: 0x55, type, 8 data bytes, sum of the first 10 bytes.
class Parser {
public:
    // Returns true when an angle packet completes an acc/gyro/angle group.
    bool push(uint8_t byte);
    const Sample& sample() const { return sample_; }
    uint32_t badCount() const { return bad_; }

private:
    uint8_t buf_[11];
    size_t idx_ = 0;
    Sample sample_{};
    uint32_t bad_ = 0;
};

}  // namespace wt901
