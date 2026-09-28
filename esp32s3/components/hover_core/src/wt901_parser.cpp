#include "hover_core/wt901_parser.h"

#include <cstring>

namespace wt901 {

bool Parser::push(uint8_t byte)
{
    if (idx_ == 0 && byte != 0x55) return false;
    if (idx_ == 1 && (byte & 0xF0) != 0x50) {  // Not a packet type: restart the search
        idx_ = (byte == 0x55) ? 1 : 0;
        return false;
    }
    buf_[idx_++] = byte;
    if (idx_ < sizeof(buf_)) return false;
    idx_ = 0;

    uint8_t sum = 0;
    for (size_t i = 0; i < 10; i++) sum += buf_[i];
    if (sum != buf_[10]) {
        bad_++;
        return false;
    }

    int16_t v[3];
    for (int i = 0; i < 3; i++) v[i] = (int16_t)(buf_[2 + 2 * i] | (buf_[3 + 2 * i] << 8));

    switch (buf_[1]) {
    case 0x51:
        std::memcpy(sample_.acc, v, sizeof(v));
        return false;
    case 0x52:
        std::memcpy(sample_.gyro, v, sizeof(v));
        return false;
    case 0x53:
        std::memcpy(sample_.angle, v, sizeof(v));
        return true;
    default:
        return false;
    }
}

}  // namespace wt901
