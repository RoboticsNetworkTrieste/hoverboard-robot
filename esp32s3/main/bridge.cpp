#include "bridge.h"

#include <algorithm>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hover_core/cmd_watchdog.h"
#include "hover_core/hover_protocol.h"
#include "hover_core/lidar_chunker.h"
#include "hover_core/lidar_starter.h"
#include "hover_core/link_protocol.h"
#include "hover_core/wt901_parser.h"
#include "log_forward.h"
#include "sdkconfig.h"
#include "uart_port.h"
#include "udp_link.h"

namespace {

const char* TAG = "bridge";

constexpr uint32_t kHoverSendPeriodMs = 50;  // Firmware serial timeout is ~800 ms
constexpr uint32_t kStatePeriodMs = 20;      // 50 Hz telemetry
constexpr uint32_t kFreshMs = 500;           // Feedback/IMU age for status bits
constexpr size_t kLidarChunkBytes = 400;
constexpr uint32_t kLidarFlushMs = 10;
constexpr int kLogLinesPerTick = 4;  // Larger bursts get dropped by the WiFi TX queue

uint32_t nowMs()
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

class Bridge {
public:
    void run();

private:
    void openPorts();
    void pollUdp(uint32_t now);
    void handleDatagram(const uint8_t* data, size_t len, uint32_t now);
    void pollHover(uint32_t now);
    void pollImu(uint32_t now);
    void pollLidar(uint32_t now);
    void flushLidar(uint32_t now);
    void startLidar(uint32_t now);
    void sendHoverCommand(uint32_t now);
    void sendWheelState(uint32_t now);
    void sendLogs(uint32_t now);
    void sendMessage(proto::MsgType type, const void* body, size_t len, uint32_t now);

    UdpLink udp_;
    UartPort hoverUart_;
    UartPort imuUart_;
    UartPort lidarUart_;

    hover::FeedbackParser hoverParser_;
    hover::TickUnwrapper ticksL_;
    hover::TickUnwrapper ticksR_;
    wt901::Parser imuParser_;
    util::CmdWatchdog watchdog_{CONFIG_HB_CMD_TIMEOUT_MS};
    util::LidarChunker lidarChunk_{kLidarChunkBytes, kLidarFlushMs};
#ifdef CONFIG_HB_LIDAR_AUTOSTART
    util::LidarStarter lidarStarter_{CONFIG_HB_LIDAR_START_RETRY_MS};
#endif

    int16_t cmdLeft_ = 0;
    int16_t cmdRight_ = 0;
    bool feedbackSeen_ = false;
    bool imuSeen_ = false;
    uint32_t lastFeedbackMs_ = 0;
    uint32_t lastImuMs_ = 0;
    uint16_t seq_ = 0;

    uint8_t rx_[proto::kMaxDatagram];
    uint8_t tx_[proto::kMaxDatagram];
};

void Bridge::openPorts()
{
    udp_.open(CONFIG_HB_UDP_PORT);
    hoverUart_.open(UART_NUM_1, CONFIG_HB_HOVER_RX_GPIO, CONFIG_HB_HOVER_TX_GPIO, CONFIG_HB_HOVER_BAUD, 1024);
#ifdef CONFIG_HB_ENABLE_IMU
    imuUart_.open(UART_NUM_0, CONFIG_HB_IMU_RX_GPIO, CONFIG_HB_IMU_TX_GPIO, CONFIG_HB_IMU_BAUD, 512);
#endif
#ifdef CONFIG_HB_ENABLE_LIDAR
    lidarUart_.open(UART_NUM_2, CONFIG_HB_LIDAR_RX_GPIO, CONFIG_HB_LIDAR_TX_GPIO, CONFIG_HB_LIDAR_BAUD, 4096);
#endif
}

void Bridge::run()
{
    openPorts();
    ESP_LOGI(TAG, "Bridge running");
    uint32_t tHover = 0;
    uint32_t tState = 0;
    for (;;) {
        uint32_t now = nowMs();
        pollUdp(now);
        pollHover(now);
        pollImu(now);
        pollLidar(now);
        startLidar(now);
        if (now - tHover >= kHoverSendPeriodMs) {
            tHover = now;
            sendHoverCommand(now);
        }
        if (now - tState >= kStatePeriodMs) {
            tState = now;
            sendWheelState(now);
            sendLogs(now);
        }
        vTaskDelay(1);  // 1 ms with CONFIG_FREERTOS_HZ=1000
    }
}

void Bridge::pollUdp(uint32_t now)
{
    size_t n;
    while ((n = udp_.poll(rx_, sizeof(rx_))) > 0) handleDatagram(rx_, n, now);
}

void Bridge::handleDatagram(const uint8_t* data, size_t len, uint32_t now)
{
    proto::Header header;
    const uint8_t* body = nullptr;
    size_t bodyLen = 0;
    if (!proto::decode(data, len, header, body, bodyLen)) return;

    switch (header.type) {
    case proto::kCmdWheels: {
        if (bodyLen != sizeof(proto::CmdWheels)) return;
        proto::CmdWheels cmd;
        std::memcpy(&cmd, body, sizeof(cmd));
        cmdLeft_ = (int16_t)std::clamp<int>(cmd.left, -CONFIG_HB_CMD_LIMIT, CONFIG_HB_CMD_LIMIT);
        cmdRight_ = (int16_t)std::clamp<int>(cmd.right, -CONFIG_HB_CMD_LIMIT, CONFIG_HB_CMD_LIMIT);
        watchdog_.feed(now);
        udp_.commitPeer();
        break;
    }
    case proto::kLidarTx:
        lidarUart_.write(body, bodyLen);
        break;
    default:
        break;
    }
}

void Bridge::pollHover(uint32_t now)
{
    uint8_t buf[128];
    size_t n;
    while ((n = hoverUart_.read(buf, sizeof(buf))) > 0) {
        for (size_t i = 0; i < n; i++) {
            if (!hoverParser_.push(buf[i])) continue;
            const hover::Feedback& f = hoverParser_.last();
            ticksL_.update(f.wheelL_cnt);
            ticksR_.update(f.wheelR_cnt);
            feedbackSeen_ = true;
            lastFeedbackMs_ = now;
        }
    }
}

void Bridge::pollImu(uint32_t now)
{
    uint8_t buf[128];
    size_t n;
    while ((n = imuUart_.read(buf, sizeof(buf))) > 0) {
        for (size_t i = 0; i < n; i++) {
            if (!imuParser_.push(buf[i])) continue;
            const wt901::Sample& s = imuParser_.sample();
            proto::ImuState msg;
            std::memcpy(msg.acc, s.acc, sizeof(msg.acc));
            std::memcpy(msg.gyro, s.gyro, sizeof(msg.gyro));
            std::memcpy(msg.angle, s.angle, sizeof(msg.angle));
            sendMessage(proto::kImu, &msg, sizeof(msg), now);
            imuSeen_ = true;
            lastImuMs_ = now;
        }
    }
}

void Bridge::startLidar(uint32_t now)
{
#ifdef CONFIG_HB_LIDAR_AUTOSTART
    if (lidarStarter_.tick(now)) {
        lidarUart_.write(util::kLidarStart, util::kLidarFrameSize);
        ESP_LOGI(TAG, "LIDAR start command sent");
    }
#else
    (void)now;
#endif
}

void Bridge::pollLidar(uint32_t now)
{
    uint8_t buf[512];
    size_t n;
    while ((n = lidarUart_.read(buf, sizeof(buf))) > 0) {
#ifdef CONFIG_HB_LIDAR_AUTOSTART
        lidarStarter_.onBytes(now);
#endif
        size_t off = 0;
        while (off < n) {
            off += lidarChunk_.push(buf + off, n - off, now);
            if (lidarChunk_.ready(now)) flushLidar(now);
        }
    }
    if (lidarChunk_.ready(now)) flushLidar(now);
}

void Bridge::flushLidar(uint32_t now)
{
    sendMessage(proto::kLidarRx, lidarChunk_.data(), lidarChunk_.size(), now);
    lidarChunk_.clear();
}

void Bridge::sendHoverCommand(uint32_t now)
{
    bool stop = watchdog_.expired(now);
    hover::Command c = hover::makeCommand(stop ? 0 : cmdLeft_, stop ? 0 : cmdRight_);
    hoverUart_.write(reinterpret_cast<const uint8_t*>(&c), sizeof(c));
}

void Bridge::sendWheelState(uint32_t now)
{
    const hover::Feedback& f = hoverParser_.last();
    proto::WheelState s{};
    s.ticks_l = ticksL_.total();
    s.ticks_r = ticksR_.total();
    s.rpm_l = f.speedL_meas;
    s.rpm_r = f.speedR_meas;
    s.bat_cv = f.batVoltage;
    s.temp_dc = f.boardTemp;
    uint8_t status = 0;
    if (feedbackSeen_ && now - lastFeedbackMs_ < kFreshMs) status |= proto::kStatusHoverOk;
    if (watchdog_.expired(now)) status |= proto::kStatusCmdTimeout;
    if (imuSeen_ && now - lastImuMs_ < kFreshMs) status |= proto::kStatusImuOk;
    s.status = status;
    s.hover_ok = hoverParser_.okCount();
    s.hover_bad = hoverParser_.badCount();
    sendMessage(proto::kWheelState, &s, sizeof(s), now);
}

void Bridge::sendLogs(uint32_t now)
{
    if (!udp_.hasPeer()) return;  // Keep queued lines until a PC is listening
    char line[kLogLineMax];
    size_t n;
    for (int i = 0; i < kLogLinesPerTick && (n = log_forward_read(line, sizeof(line))) > 0; i++) {
        sendMessage(proto::kLog, line, n, now);
    }
}

void Bridge::sendMessage(proto::MsgType type, const void* body, size_t len, uint32_t now)
{
    size_t n = proto::encode(type, seq_++, now, body, len, tx_, sizeof(tx_));
    if (n > 0) udp_.send(tx_, n);
}

}  // namespace

void bridge_run()
{
    static Bridge bridge;  // Static: buffers are too large for the task stack
    bridge.run();
}
