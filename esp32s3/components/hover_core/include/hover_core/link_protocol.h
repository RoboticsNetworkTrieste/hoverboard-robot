#pragma once

#include <cstddef>
#include <cstdint>

// UDP message format shared with ros2/hoverboard_bridge/hoverboard_bridge/link_protocol.py.
namespace proto {

constexpr size_t kHeaderSize = 7;
constexpr size_t kMaxDatagram = 1024;

enum MsgType : uint8_t {
    kCmdWheels = 0x01,   // PC -> ESP
    kLidarTx = 0x02,     // PC -> ESP, raw bytes for the LIDAR
    kWheelState = 0x10,  // ESP -> PC
    kImu = 0x11,         // ESP -> PC
    kLidarRx = 0x12,     // ESP -> PC, raw bytes from the LIDAR
    kLog = 0x13,         // ESP -> PC, one log line
};

enum StatusBits : uint8_t {
    kStatusHoverOk = 0x01,
    kStatusCmdTimeout = 0x02,
    kStatusImuOk = 0x04,
};

struct __attribute__((packed)) Header {
    uint8_t type;
    uint16_t seq;
    uint32_t stamp_ms;
};

struct __attribute__((packed)) CmdWheels {
    int16_t left;   // Firmware units, 1000 = N_MOT_MAX rpm
    int16_t right;
};

struct __attribute__((packed)) WheelState {
    int32_t ticks_l;   // Unwrapped hall ticks
    int32_t ticks_r;
    int16_t rpm_l;
    int16_t rpm_r;
    int16_t bat_cv;    // [V*100]
    int16_t temp_dc;   // [degC*10]
    uint8_t status;    // StatusBits
    uint32_t hover_ok;
    uint32_t hover_bad;
};

struct __attribute__((packed)) ImuState {
    int16_t acc[3];    // Raw WT901 units
    int16_t gyro[3];
    int16_t angle[3];
};

// Writes header + body into out. Returns the datagram size, or 0 if it does not fit.
size_t encode(MsgType type, uint16_t seq, uint32_t stamp_ms, const void* body, size_t body_len,
              uint8_t* out, size_t out_cap);

// Splits a datagram into header and body. Returns false if shorter than the header.
bool decode(const uint8_t* data, size_t len, Header& header, const uint8_t*& body, size_t& body_len);

}  // namespace proto
