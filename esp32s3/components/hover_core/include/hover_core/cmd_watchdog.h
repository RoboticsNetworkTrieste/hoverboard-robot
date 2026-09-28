#pragma once

#include <cstdint>

namespace util {

// Expires when no command was received within the timeout (or never).
class CmdWatchdog {
public:
    explicit CmdWatchdog(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}
    void feed(uint32_t now_ms)
    {
        last_ms_ = now_ms;
        fed_ = true;
    }
    bool expired(uint32_t now_ms) const { return !fed_ || (now_ms - last_ms_) > timeout_ms_; }

private:
    uint32_t timeout_ms_;
    uint32_t last_ms_ = 0;
    bool fed_ = false;
};

}  // namespace util
