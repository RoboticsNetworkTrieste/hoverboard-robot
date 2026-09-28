#pragma once

#include <cstddef>
#include <cstdint>

namespace hover {

constexpr uint16_t kStartFrame = 0xABCD;
constexpr int16_t kOdomWrap = 9000;  // Firmware tick counters wrap at this value

struct __attribute__((packed)) Command {
    uint16_t start;
    int16_t steer;  // TANK_STEERING: left wheel
    int16_t speed;  // TANK_STEERING: right wheel
    uint16_t checksum;
};

struct __attribute__((packed)) Feedback {
    uint16_t start;
    int16_t cmd1;
    int16_t cmd2;
    int16_t speedR_meas;  // [rpm]
    int16_t speedL_meas;  // [rpm]
    int16_t wheelR_cnt;   // [ticks] 0..kOdomWrap-1
    int16_t wheelL_cnt;   // [ticks] 0..kOdomWrap-1
    int16_t batVoltage;   // [V*100]
    int16_t boardTemp;    // [degC*10]
    uint16_t cmdLed;
    uint16_t checksum;
};

Command makeCommand(int16_t left, int16_t right);

// Byte-stream parser for feedback packets; resynchronizes on every start frame.
class FeedbackParser {
public:
    // Returns true when a packet with a valid checksum completes.
    bool push(uint8_t byte);
    const Feedback& last() const { return last_; }
    uint32_t okCount() const { return ok_; }
    uint32_t badCount() const { return bad_; }

private:
    Feedback building_{};
    Feedback last_{};
    size_t idx_ = 0;
    uint8_t prev_ = 0;
    uint32_t ok_ = 0;
    uint32_t bad_ = 0;
};

// Turns the wrapping firmware counter into a cumulative tick count.
class TickUnwrapper {
public:
    int32_t update(int16_t raw);
    int32_t total() const { return total_; }

private:
    bool init_ = false;
    int16_t prev_ = 0;
    int32_t total_ = 0;
};

}  // namespace hover
