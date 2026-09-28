#include "hover_core/hover_protocol.h"

namespace hover {

static uint16_t feedbackChecksum(const Feedback& f)
{
    return (uint16_t)(f.start ^ f.cmd1 ^ f.cmd2 ^ f.speedR_meas ^ f.speedL_meas ^ f.wheelR_cnt ^
                      f.wheelL_cnt ^ f.batVoltage ^ f.boardTemp ^ f.cmdLed);
}

Command makeCommand(int16_t left, int16_t right)
{
    Command c;
    c.start = kStartFrame;
    c.steer = left;
    c.speed = right;
    c.checksum = (uint16_t)(c.start ^ c.steer ^ c.speed);
    return c;
}

bool FeedbackParser::push(uint8_t byte)
{
    auto* buf = reinterpret_cast<uint8_t*>(&building_);
    uint16_t frame = (uint16_t)((byte << 8) | prev_);
    prev_ = byte;

    if (frame == kStartFrame) {
        buf[0] = (uint8_t)(kStartFrame & 0xFF);
        buf[1] = (uint8_t)(kStartFrame >> 8);
        idx_ = 2;
        return false;
    }
    if (idx_ < 2) return false;

    buf[idx_++] = byte;
    if (idx_ < sizeof(Feedback)) return false;

    idx_ = 0;
    if (feedbackChecksum(building_) != building_.checksum) {
        bad_++;
        return false;
    }
    last_ = building_;
    ok_++;
    return true;
}

int32_t TickUnwrapper::update(int16_t raw)
{
    if (!init_) {
        prev_ = raw;
        init_ = true;
        return total_;
    }
    int32_t d = (int32_t)raw - prev_;
    if (d > kOdomWrap / 2) d -= kOdomWrap;
    if (d < -kOdomWrap / 2) d += kOdomWrap;
    total_ += d;
    prev_ = raw;
    return total_;
}

}  // namespace hover
