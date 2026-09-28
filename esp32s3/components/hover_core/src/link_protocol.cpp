#include "hover_core/link_protocol.h"

#include <cstring>

namespace proto {

static_assert(sizeof(Header) == kHeaderSize, "Header size");
static_assert(sizeof(CmdWheels) == 4, "CmdWheels size");
static_assert(sizeof(WheelState) == 25, "WheelState size");
static_assert(sizeof(ImuState) == 18, "ImuState size");

size_t encode(MsgType type, uint16_t seq, uint32_t stamp_ms, const void* body, size_t body_len,
              uint8_t* out, size_t out_cap)
{
    size_t total = kHeaderSize + body_len;
    if (total > out_cap) return 0;
    Header h;
    h.type = type;
    h.seq = seq;
    h.stamp_ms = stamp_ms;
    std::memcpy(out, &h, kHeaderSize);
    if (body_len > 0) std::memmove(out + kHeaderSize, body, body_len);
    return total;
}

bool decode(const uint8_t* data, size_t len, Header& header, const uint8_t*& body, size_t& body_len)
{
    if (len < kHeaderSize) return false;
    std::memcpy(&header, data, kHeaderSize);
    body = data + kHeaderSize;
    body_len = len - kHeaderSize;
    return true;
}

}  // namespace proto
