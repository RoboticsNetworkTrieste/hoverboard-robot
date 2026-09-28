// Host tests for hover_core. Build and run with test_host/run_tests.sh.
#include <cstdio>
#include <cstring>

#include "hover_core/hover_protocol.h"
#include "hover_core/cmd_watchdog.h"
#include "hover_core/lidar_chunker.h"
#include "hover_core/lidar_starter.h"
#include "hover_core/link_protocol.h"
#include "hover_core/wt901_parser.h"

static int g_failures = 0;
#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
            g_failures++;                                                      \
        }                                                                      \
    } while (0)

static void testHoverProtocol()
{
    // makeCommand golden vector: left=100, right=-100
    const uint8_t expectedCmd[] = {0xCD, 0xAB, 0x64, 0x00, 0x9C, 0xFF, 0x35, 0x54};
    hover::Command cmd = hover::makeCommand(100, -100);
    CHECK(sizeof(cmd) == sizeof(expectedCmd));
    CHECK(std::memcmp(&cmd, expectedCmd, sizeof(expectedCmd)) == 0);

    // Feedback golden vector preceded by noise
    const uint8_t stream[] = {0x12, 0x34,
                              0xCD, 0xAB, 0x0A, 0x00, 0xF6, 0xFF, 0xCE, 0xFF, 0x32, 0x00, 0x1E,
                              0x23, 0x05, 0x00, 0x0A, 0x0F, 0x45, 0x01, 0x00, 0x00, 0x99, 0x86};
    hover::FeedbackParser parser;
    int packets = 0;
    for (uint8_t b : stream) packets += parser.push(b) ? 1 : 0;
    CHECK(packets == 1);
    CHECK(parser.okCount() == 1);
    CHECK(parser.last().speedL_meas == 50);
    CHECK(parser.last().speedR_meas == -50);
    CHECK(parser.last().wheelR_cnt == 8990);
    CHECK(parser.last().batVoltage == 3850);

    // Corrupted checksum is counted as bad
    uint8_t bad[sizeof(stream) - 2];
    std::memcpy(bad, stream + 2, sizeof(bad));
    bad[sizeof(bad) - 1] ^= 0xFF;
    for (uint8_t b : bad) parser.push(b);
    CHECK(parser.badCount() == 1);

    // Tick unwrapping across the 9000 wrap in both directions
    hover::TickUnwrapper unwrap;
    CHECK(unwrap.update(8990) == 0);
    CHECK(unwrap.update(5) == 15);
    CHECK(unwrap.update(8995) == 5);
}

static void testLinkProtocol()
{
    // CMD_WHEELS golden vector: seq=1, stamp=0, left=100, right=-100
    const uint8_t expected[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x9C, 0xFF};
    proto::CmdWheels cmd{100, -100};
    uint8_t out[proto::kMaxDatagram];
    size_t n = proto::encode(proto::kCmdWheels, 1, 0, &cmd, sizeof(cmd), out, sizeof(out));
    CHECK(n == sizeof(expected));
    CHECK(std::memcmp(out, expected, sizeof(expected)) == 0);

    proto::Header h;
    const uint8_t* body = nullptr;
    size_t bodyLen = 0;
    CHECK(proto::decode(expected, sizeof(expected), h, body, bodyLen));
    CHECK(h.type == proto::kCmdWheels && h.seq == 1 && bodyLen == sizeof(proto::CmdWheels));
    CHECK(!proto::decode(expected, 6, h, body, bodyLen));

    // WHEEL_STATE golden vector: seq=7, stamp=123456
    const uint8_t expectedState[] = {0x10, 0x07, 0x00, 0x40, 0xE2, 0x01, 0x00, 0x18, 0xFC, 0xFF, 0xFF,
                                     0xD0, 0x07, 0x00, 0x00, 0x32, 0x00, 0xCE, 0xFF, 0x0A, 0x0F, 0x45,
                                     0x01, 0x05, 0x64, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
    proto::WheelState s{};
    s.ticks_l = -1000;
    s.ticks_r = 2000;
    s.rpm_l = 50;
    s.rpm_r = -50;
    s.bat_cv = 3850;
    s.temp_dc = 325;
    s.status = 0x05;
    s.hover_ok = 100;
    s.hover_bad = 2;
    n = proto::encode(proto::kWheelState, 7, 123456, &s, sizeof(s), out, sizeof(out));
    CHECK(n == sizeof(expectedState));
    CHECK(std::memcmp(out, expectedState, sizeof(expectedState)) == 0);

    // Too large for the output buffer
    CHECK(proto::encode(proto::kLog, 0, 0, out, 20, out, 10) == 0);
}

static void testWatchdogAndChunker()
{
    util::CmdWatchdog wd(300);
    CHECK(wd.expired(0));  // Never fed
    wd.feed(100);
    CHECK(!wd.expired(400));
    CHECK(wd.expired(401));

    util::LidarChunker chunk(4, 10);
    const uint8_t bytes[] = {1, 2, 3, 4, 5, 6};
    CHECK(chunk.push(bytes, 3, 0) == 3);
    CHECK(!chunk.ready(5));
    CHECK(chunk.ready(10));  // Age flush
    chunk.clear();
    CHECK(chunk.push(bytes, 6, 20) == 4);  // Size limit
    CHECK(chunk.ready(20));
    CHECK(chunk.size() == 4 && chunk.data()[3] == 4);
}

static void testLidarStarter()
{
    util::LidarStarter starter(1000);
    CHECK(starter.tick(0));       // Start command on the very first call
    CHECK(!starter.tick(500));    // Retry period not elapsed yet
    CHECK(starter.tick(1000));    // Still silent: resend

    starter.onBytes(1200);        // The LIDAR is streaming
    CHECK(!starter.tick(1300));   // Nothing to do while data flows
    CHECK(!starter.tick(2100));   // Stream stalled, but the last command is still recent
    CHECK(starter.tick(2300));    // Stream dead and retry elapsed: resend

    CHECK(util::kLidarStart[1] == 0xA0 && util::kLidarStart[7] == 0x5E);
    CHECK(util::kLidarStop[1] == 0xA1 && util::kLidarStop[7] == 0x4A);
}

static void testWt901Parser()
{
    const uint8_t stream[] = {
        0x00,                                                                    // Noise
        0x55, 0x51, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xB8, 0x0B, 0x71,        // acc (0, 0, 2048)
        0x55, 0x52, 0x00, 0x40, 0x00, 0x00, 0x00, 0xC0, 0xB8, 0x0B, 0x6A,        // gyro (16384, 0, -16384)
        0x55, 0x53, 0x00, 0x00, 0x00, 0x20, 0x00, 0xC0, 0x00, 0x00, 0x88,        // angle (0, 8192, -16384)
    };
    wt901::Parser parser;
    int samples = 0;
    for (uint8_t b : stream) samples += parser.push(b) ? 1 : 0;
    CHECK(samples == 1);
    CHECK(parser.sample().acc[2] == 2048);
    CHECK(parser.sample().gyro[0] == 16384 && parser.sample().gyro[2] == -16384);
    CHECK(parser.sample().angle[1] == 8192);
    CHECK(parser.badCount() == 0);

    uint8_t bad[11];
    std::memcpy(bad, stream + 1, sizeof(bad));
    bad[10] ^= 0xFF;
    for (uint8_t b : bad) parser.push(b);
    CHECK(parser.badCount() == 1);
}

int main()
{
    testHoverProtocol();
    testLinkProtocol();
    testWatchdogAndChunker();
    testLidarStarter();
    testWt901Parser();
    if (g_failures == 0) std::printf("All host tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
