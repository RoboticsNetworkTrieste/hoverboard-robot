#include "hover_core/lidar_chunker.h"

#include <cstring>

namespace util {

LidarChunker::LidarChunker(size_t max_bytes, uint32_t max_age_ms)
    : max_bytes_(max_bytes > kCapacity ? kCapacity : (max_bytes == 0 ? 1 : max_bytes)),
      max_age_ms_(max_age_ms)
{
}

size_t LidarChunker::push(const uint8_t* data, size_t len, uint32_t now_ms)
{
    size_t room = max_bytes_ - size_;
    size_t n = len < room ? len : room;
    if (n == 0) return 0;
    if (size_ == 0) first_ms_ = now_ms;
    std::memcpy(buf_ + size_, data, n);
    size_ += n;
    return n;
}

bool LidarChunker::ready(uint32_t now_ms) const
{
    if (size_ == 0) return false;
    return size_ >= max_bytes_ || (now_ms - first_ms_) >= max_age_ms_;
}

}  // namespace util
