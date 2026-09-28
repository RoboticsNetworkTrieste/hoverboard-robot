#pragma once

#include <cstddef>
#include <cstdint>

namespace util {

// Groups LIDAR bytes into datagram-sized chunks, flushed by size or age.
class LidarChunker {
public:
    static constexpr size_t kCapacity = 512;

    LidarChunker(size_t max_bytes, uint32_t max_age_ms);
    // Appends as many bytes as fit and returns how many were taken.
    size_t push(const uint8_t* data, size_t len, uint32_t now_ms);
    bool ready(uint32_t now_ms) const;
    const uint8_t* data() const { return buf_; }
    size_t size() const { return size_; }
    void clear() { size_ = 0; }

private:
    uint8_t buf_[kCapacity];
    size_t max_bytes_;
    uint32_t max_age_ms_;
    size_t size_ = 0;
    uint32_t first_ms_ = 0;
};

}  // namespace util
