# Hoverboard ROS2 UDP Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **One machine, ROS2 in Docker.** Everything runs on the Ubuntu 24.04 development PC. ROS2 Humble is not installed on the host: it runs in the Docker container defined in `tools/docker-humble` (Task 5). The host has ROS2 Jazzy installed; do not use it for this project. See "Where each step runs" below before executing any step.

**Goal:** An ESP32-S3 firmware that bridges the hoverboard mainboard, a WT901C IMU and a 2D LIDAR to a ROS2 Humble PC using UDP, carried over WiFi (station or access point) or USB tethering, plus the ROS2 Python node on the PC.

**Architecture:** The ESP32-S3 runs one single-threaded bridge loop that polls three UARTs and one UDP socket bound to all network interfaces (WiFi and USB CDC-NCM). Every message, in both directions, is a UDP datagram with a 7-byte header. The ESP replies to the address of the last PC that sent a valid wheel command, so the PC chooses the medium by the IP it targets. On the PC, an `rclpy` node converts `/cmd_vel` into wheel commands and publishes odometry, TF, joint states, IMU, battery and temperature. It also exposes the raw LIDAR byte stream on a pseudo terminal for a serial LIDAR driver.

**Tech Stack:** ESP-IDF v5.4.x (C++17), `espressif/esp_tinyusb` 1.x (CDC-NCM), `espressif/mdns`, FreeRTOS, lwIP sockets. ROS2 Humble (Ubuntu 22.04, Python 3.10) inside a Docker container, `rclpy`, `ament_python`, pytest. Host C++ tests are built with the host g++.

**Spec:** No separate spec document. The design decisions from the conversation are summarized in "Design Summary" below. Executors must read that section first.

## Status and open issues (2026-10-18)

Bench result: Tasks 1-7, 9 and 10 are implemented. Over the WiFi access point, all wiring is confirmed:
hoverboard feedback and commands, WT901C IMU and LIDAR data reach the PC as UDP datagrams.
The configuration used for that run is now the default (`Kconfig.projbuild`, `sdkconfig.defaults`):
access point mode with placeholder SSID/password, pins as in "UART assignment", IMU enabled at 115200 baud,
console output None, WPA3-SAE on (ESP-IDF default).

Open issues:
- **Packet loss.** Many datagrams were lost during the bench tests. Not investigated yet. The `seq` field is
  shared by all ESP->PC messages, so gaps in it measure loss directly. Suspects: `UdpLink::send` drops silently
  when the WiFi TX queue is full (one datagram per IMU packet plus LIDAR chunks plus 50 Hz state), RF conditions,
  PC-side receive loop.
- **Station mode does not connect.** The ESP no longer joins the router; earlier it connected after a few
  disconnects. Access point mode was used as a workaround. Not investigated yet. The disconnect log does not print
  the reason code, which would be the first clue.
- Task 7 Step 5 (station checks), Task 9 Step 7 watchdog checks and Task 10 Step 5 (README sensor notes) still
  to be confirmed or done; then Tasks 11-14.

## Design Summary

### Why ESP-IDF and not Arduino
USB tethering requires the ESP32-S3 to act as a USB network card (CDC-NCM). The Arduino ESP32 core only provides CDC serial, HID and MSC USB classes. NCM networking is available only through ESP-IDF's `esp_tinyusb` component (`tinyusb_net_*` API). The existing Arduino sketch `tools/arduino_hoverboard_test` stays as the UART bench tool.

### Transports (all carry the same UDP datagrams)
| Medium | Kconfig | ESP IP | PC reaches ESP at |
|---|---|---|---|
| WiFi station | `HB_WIFI_MODE_STA` + `HB_WIFI_SSID`/`HB_WIFI_PASSWORD` | Static `HB_WIFI_STA_IP` (default `192.168.1.50`), or DHCP from the router if `HB_WIFI_STA_STATIC_IP` is off | the static IP; `hoverboard.local` (mDNS) also works from the host |
| WiFi access point | `HB_WIFI_MODE_AP` + `HB_WIFI_SSID`/`HB_WIFI_PASSWORD` | `192.168.4.1` (DHCP server) | `192.168.4.1` |
| USB tethering (CDC-NCM, optional) | `HB_USB_NET=y` | `192.168.7.1` (DHCP server gives the PC `192.168.7.2`, no default route) | `192.168.7.1` |

WiFi (in one of its modes) and USB can be enabled together. The UDP socket listens on `0.0.0.0:HB_UDP_PORT` (default 8888). Telemetry goes to the last sender of a valid `CMD_WHEELS` message.

### UART assignment (ESP32-S3 has exactly 3 UARTs)
| UART | Device | ESP pins (Kconfig defaults) | Baud |
|---|---|---|---|
| UART1 | Hoverboard (right sensor cable, USART3) | RX=GPIO3, TX=GPIO4 | 115200 |
| UART2 | LIDAR STL-50B2 | RX=GPIO7, TX=GPIO8 | 230400 (confirmed on the bench, see `docs/lidar/STL-50B2.md`) |
| UART0 | WT901C IMU | RX=GPIO5, TX=GPIO6 | 115200 (this unit; WT901C factory default is 9600) |

### Board and power
The board is a Waveshare ESP32-S3-Zero. Only its edge pins are used: 5V, GND, 3V3, GPIO1-13 and GPIO43/44 (TX/RX). GPIO14-18 and GPIO38-45 are solder pads on the top and bottom faces and are not used. GPIO21 drives the on-board RGB LED.

Power and wiring are described in "Electrical wiring" below.

The board has a single USB-C port, wired to the native USB. When the firmware enables USB tethering, that port becomes a CDC-NCM network device and no serial port or console is available on it. Flashing then needs download mode: hold BOOT, press and release RESET, release BOOT.

Using UART0 for the IMU requires the ESP-IDF console to be disabled (`CONFIG_ESP_CONSOLE_NONE`). ESP logs are always forwarded as `LOG` datagrams, so they appear in the PC tools anyway.

### Electrical wiring
The hoverboard keeps its original mainboard, motor controllers, hall sensors and motors. Everything added for the robot runs from one 5V rail:

```
Hoverboard battery
  -> original mainboard
       -> 12V output (the line that used to feed a Bluetooth speaker, 12V measured)
            -> DSN-mini-360 step-down (MP2307, 1.8A), output trimmed to 5.0V
                 -> ESP32-S3-Zero 5V pin
                 -> LIDAR STL-50B2 5V
                 -> WT901C VCC
                 -> level shifter HV
ESP32-S3-Zero 3V3 pin -> level shifter LV
All grounds are common: mainboard, step-down output, ESP, level shifter, LIDAR, IMU.
```

Expected load on the 5V rail is about 1A at most (ESP with WiFi up to ~0.5A peaks, LIDAR motor start-up, IMU), so about 0.5A on the 12V line. The speaker that line used to supply drew more.

The ESP32-S3 GPIOs are not 5V tolerant. The LIDAR and the IMU run at 5V, so their UART lines go through a 4-channel MH level converter, all four channels used, (BSS138 MOSFETs with 10k pull-ups on both sides, https://wiki.liutyi.info/display/ARDUINO/MH+Level+Converter). The hoverboard UART is 3.3V (STM32F103; USART3 pins PB10/PB11 are also 5V tolerant) and is wired directly.

| Signal | Direction | Level | Path |
|---|---|---|---|
| Hoverboard TX -> GPIO3 (UART1 RX) | mainboard -> ESP | 3.3V | direct |
| GPIO4 (UART1 TX) -> Hoverboard RX | ESP -> mainboard | 3.3V | direct |
| LIDAR TX -> GPIO7 (UART2 RX) | LIDAR -> ESP | 5V (to verify) | shifter HV1 -> LV1 |
| GPIO8 (UART2 TX) -> LIDAR RX | ESP -> LIDAR | 3.3V -> 5V | shifter LV2 -> HV2 |
| WT901C TX -> GPIO5 (UART0 RX) | IMU -> ESP | 5V (to verify) | shifter HV3 -> LV3 |
| GPIO6 (UART0 TX) -> WT901C RX | ESP -> IMU | 3.3V -> 5V | shifter LV4 -> HV4 |

Rules:
- Trim the DSN-mini-360 output to 5.0V with a multimeter before connecting any load; the module does not ship set to 5V.
- The 5V UART levels of the LIDAR and the IMU come from the manufacturer sheets and are not measured yet. Measure each TX pin at idle (an idle UART line sits high) and write the result here. A 3.3V reading is still safe through the shifter.
- Never connect the red 15V wire of the hoverboard sensor cable.
- The IMU RX line is wired but the firmware sends nothing on it yet; it idles high, so the WT901C ignores it. It is there for future configuration commands (calibration, output rate, baud).
- WiFi is the normal link. Before plugging the USB-C cable (flashing, USB tethering), disconnect the step-down output from the ESP 5V pin, otherwise the 5V rail can feed back into the PC USB port. LIDAR, IMU and the shifter HV side can stay on the step-down; grounds stay common.

### LIDAR start-up
The STL-50B2 belongs to the LDROBOT family: it stays silent until it receives a start
command, and the `ldlidar_stl_ros2` driver never sends one. The ESP therefore starts it,
writing one of two hardcoded frames. No CRC code is needed in the firmware.

| Frame | Bytes |
|---|---|
| Start motor and scanning | `54 A0 04 00 00 00 00 5E` |
| Stop motor | `54 A1 04 00 00 00 00 4A` |

The start command is repeated, not sent once. The ESP and the LIDAR come up from the same
DC-DC converter, so a start sent before the LIDAR MCU is ready is lost, and the line then
stays silent forever - a failure indistinguishable from bad wiring. `util::LidarStarter`
resends the frame every `HB_LIDAR_START_RETRY_MS` (default 1000 ms) until LIDAR bytes
arrive, and resumes if the stream stops, which also recovers from a sensor brown-out.

The LIDAR spins for as long as the ESP is powered; there is no stop when idle. The
`LIDAR_TX` channel stays available so the PC can still send commands by hand, such as the
set-speed command `0xA2`.

Protocol, CRC8 and bench measurements: `docs/lidar/STL-50B2.md`.

### UDP message format (little endian)
Header, 7 bytes: `uint8 type, uint16 seq, uint32 stamp_ms`. The sender's millisecond clock goes in `stamp_ms`; the PC sends 0.

| Type | Id | Direction | Body |
|---|---|---|---|
| `CMD_WHEELS` | 0x01 | PC→ESP | `int16 left, int16 right` (firmware units, ±1000 = ±N_MOT_MAX rpm) |
| `LIDAR_TX` | 0x02 | PC→ESP | raw bytes written to the LIDAR UART (the ESP starts the LIDAR on its own; this is for extra commands) |
| `WHEEL_STATE` | 0x10 | ESP→PC, 50 Hz | `int32 ticks_l, int32 ticks_r, int16 rpm_l, int16 rpm_r, int16 bat_cv, int16 temp_dc, uint8 status, uint32 hover_ok, uint32 hover_bad` (25 bytes) |
| `IMU` | 0x11 | ESP→PC, on each WT901 angle packet | `int16 acc[3], int16 gyro[3], int16 angle[3]` raw WT901 units (18 bytes) |
| `LIDAR_RX` | 0x12 | ESP→PC | raw LIDAR bytes, flushed at 400 bytes or 10 ms |
| `LOG` | 0x13 | ESP→PC | ASCII text of one ESP log line |

`status` bits: `0x01` hoverboard feedback fresh (<500 ms), `0x02` command watchdog expired, `0x04` IMU data fresh (<500 ms).

ESP wheel values are raw from the firmware: `ticks_*` are the unwrapped `wheel*_cnt` counters, `rpm_*` are `speed*_meas`. Sign corrections, kinematics and unit conversions are done on the PC, so they can be tuned without reflashing.

### Safety
- ESP watchdog: if no valid `CMD_WHEELS` arrives for `HB_CMD_TIMEOUT_MS` (300 ms), the ESP keeps sending `0 0` to the hoverboard.
- PC node: if no `/cmd_vel` arrives for `cmd_vel_timeout` (0.5 s), it sends `0 0`.
- Hoverboard firmware: stops the motors if the ESP stops talking (~0.8 s).

### Hoverboard firmware facts (from `Inc/config.h`, `Src/main.c`, `Src/bldc.c`)
- `VARIANT_USART`, `CONTROL_SERIAL_USART3`, `FEEDBACK_SERIAL_USART3`, `TANK_STEERING` (`steer` = left wheel, `speed` = right wheel), `SPD_MODE`, `N_MOT_MAX 200`, `ENABLE_ODOMETRY`, `USART3_BAUD 115200`.
- Command packet: `uint16 start=0xABCD, int16 steer, int16 speed, uint16 checksum = start^steer^speed` (8 bytes).
- Feedback packet (22 bytes): `start, cmd1, cmd2, speedR_meas, speedL_meas, wheelR_cnt, wheelL_cnt, batVoltage, boardTemp, cmdLed, checksum`. The checksum is the XOR of all previous fields.
- `wheel*_cnt` counts hall transitions modulo 9000.
- Without `INVERT_*_DIRECTION` the firmware negates the right command internally, so positive commands drive both wheels forward. `speedR_meas` (`n_mot` of the right motor) is negative when moving forward, while `odom_r` is already negated in `Src/bldc.c`, so `wheelR_cnt` counts up when moving forward. Ticks and rpm therefore need separate signs on the PC: defaults `right_ticks_sign = 1`, `right_rpm_sign = -1`. Verified on the bench in Task 9 (command 50/50: both wheels forward, rpm L +10 / R -10, both tick counters increasing).

## Global Constraints

- All code comments, logs and identifiers in English. Comments short and useful; no historical "changed from" comments.
- Version control is handled by the user: tasks contain no git steps and executors run no git write operations. Ask the user for confirmation before flashing any device.
- ESP-IDF v5.4.x, target `esp32s3`. `espressif/esp_tinyusb` major version 1 (the `tinyusb_net_init` / `tinyusb_net_send_sync` API used here is the 1.x API).
- ROS2 Humble, Ubuntu 22.04, Python 3.10, all inside the `tools/docker-humble` image. No third-party Python dependencies beyond the ROS2 Humble desktop install. `link_probe` must also run with the host Python 3.12 (standard library only).
- The hoverboard firmware configuration is not changed by this plan. It is already built and flashed with the settings listed under "Hoverboard firmware facts"; no STM32 work and no reflashing of the mainboard is part of this plan.
- Wire formats must match byte-for-byte between `components/hover_core` (C++) and `hoverboard_bridge/link_protocol.py` (Python). The golden vectors in the tests are the reference.
- WiFi credentials live only in the local `sdkconfig`, which is git-ignored. `sdkconfig.defaults` must not contain real credentials.
- Tests are intentionally minimal (golden vectors and a few behaviours). Every test step must actually be run; report failures plainly rather than claiming tests pass.

### USB tethering compatibility
USB tethering (Task 8) is deferred, not dropped. Until it is implemented, no change may make it harder to add:
- The firmware stays transport-agnostic. The UDP socket binds `0.0.0.0` and telemetry goes to the last sender of a valid `CMD_WHEELS`. Outside `net_wifi.cpp`, no code refers to the WiFi interface (no binding to a netif, no WiFi-only checks before sending).
- `app_main` initialises NVS, `esp_netif` and the default event loop before starting any network, so `usb_net_start()` can be added next to `wifi_start()`.
- The native USB port (GPIO19/20) is reserved for flashing and CDC-NCM. Do not use it as a log channel or for other USB classes, and do not use GPIO19/20 as GPIOs. ESP logs travel as `LOG` datagrams.
- Keep the `HB_USB_NET` menuconfig option, the `espressif/esp_tinyusb` 1.x dependency and `CONFIG_TINYUSB_NET_MODE_NCM` in `sdkconfig.defaults`.
- `192.168.7.0/24` is reserved for USB. The ESP access point and the station network must use other subnets.
- On the PC, the ESP address is a plain parameter (`esp_host`, `--host`); no WiFi-only discovery.

### Where each step runs

| Where | Toolchain | Used by |
|---|---|---|
| Host (Ubuntu 24.04) | ESP-IDF v5.4.4 in `~/esp/esp-idf`, `g++`, Docker, USB flashing of the ESP | Tasks 1–4, 7–10, ESP hardware checks |
| Container (`tools/docker-humble`) | ROS2 Humble, `colcon`, `pytest`, `ldlidar_stl_ros2` | Tasks 6, 11, 12, 13, 14 |

Host steps: run `. ~/esp/esp-idf/export.sh` once per terminal so `idf.py` is on PATH. On this PC the ESP serial port is usually `/dev/ttyACM0` (the user is in the `dialout` group).

Container steps are marked "(container)". Open a shell with `tools/docker-humble/run.sh` (from the repo root), or pass the command directly: `tools/docker-humble/run.sh <command>`. Inside the container the repo folder `ros2/` is mounted at `/ws/src`, so the package lives at `/ws/src/hoverboard_bridge`, and the workspace root is `/ws`. The container uses host networking, so it reaches the ESP over WiFi and USB tethering exactly like the host. It has no mDNS resolver, so it always targets the ESP by IP; in station mode that is the static `HB_WIFI_STA_IP`.

`link_probe` is the exception: it imports only `argparse`, `socket`, `time` and `link_protocol`, so the host Python runs it directly from `ros2/hoverboard_bridge`, without the container.

## File Structure

```
esp32s3/                 ESP-IDF project (ESP32-S3 firmware)
  CMakeLists.txt                      Project definition
  sdkconfig.defaults                  Target, TinyUSB NCM, FreeRTOS tick, stack sizes
  README.md                           Build, configure, flash, wiring
  main/
    CMakeLists.txt                    Main component sources
    idf_component.yml                 Managed deps: esp_tinyusb, mdns
    Kconfig.projbuild                 "Hoverboard ROS bridge" menuconfig menu
    main.cpp                          app_main: init NVS/netif, start networks, run bridge
    net_wifi.h / net_wifi.cpp         WiFi STA (static IP or DHCP) or AP + mDNS
    net_usb.h / net_usb.cpp           USB CDC-NCM network interface with DHCP server (Task 8, deferred)
    udp_link.h / udp_link.cpp         Non-blocking UDP socket, peer tracking
    uart_port.h / uart_port.cpp       Thin non-blocking UART wrapper
    log_forward.h / log_forward.cpp   Captures ESP log lines for LOG datagrams
    bridge.h / bridge.cpp             The bridge loop (UART <-> UDP)
  components/hover_core/              Pure C++ logic, no ESP-IDF includes (host testable)
    CMakeLists.txt
    include/hover_core/hover_protocol.h   Hoverboard UART packets, parser, tick unwrapper
    include/hover_core/link_protocol.h    UDP message header and bodies
    include/hover_core/cmd_watchdog.h     Command timeout
    include/hover_core/lidar_chunker.h    LIDAR byte grouping
    include/hover_core/lidar_starter.h    LIDAR start command with retry
    include/hover_core/wt901_parser.h     WitMotion packet parser
    src/hover_protocol.cpp
    src/link_protocol.cpp
    src/lidar_chunker.cpp
    src/wt901_parser.cpp
  test_host/
    test_core.cpp                     Host tests for hover_core (golden vectors)
    run_tests.sh                      Builds and runs host tests with g++

tools/docker-humble/                  ROS2 Humble development container
  Dockerfile                          Humble desktop image, host-matching user, ldlidar_stl_ros2 underlay
  compose.yaml                        Host network/IPC, X11, ros2/ mounted as /ws/src
  ros_env.sh                          Sources ROS2, the LIDAR underlay and the workspace overlay
  entrypoint.sh                       Sources ros_env.sh, then runs the command
  run.sh                              Opens a shell or runs a command in the container
  README.md                           Usage

ros2/hoverboard_bridge/               ROS2 Humble ament_python package
  package.xml
  setup.py
  setup.cfg
  resource/hoverboard_bridge
  config/bridge.yaml                  Node parameters
  launch/bridge.launch.py             Launch with esp_host argument
  hoverboard_bridge/
    __init__.py
    link_protocol.py                  UDP message encode/decode (mirror of link_protocol.h)
    link_probe.py                     ROS-free bench CLI (commands + telemetry printout)
    kinematics.py                     Diff-drive twist->commands, odometry integration
    imu_convert.py                    WT901 raw -> SI units, Euler -> quaternion
    lidar_pty.py                      Pseudo terminal for the LIDAR driver
    bridge_node.py                    The ROS2 node
  test/
    test_link_protocol.py
    test_kinematics.py
    test_imu_convert.py
    test_lidar_pty.py                 Linux only
```

Execution order note: Tasks 1–6 and 11–13 need no hardware. Tasks 7, 9 and 10 can be written and compiled now; their hardware verification steps wait for the ESP32-S3. Task 14 needs the full robot. Task 8 (USB tethering) is deferred until Task 14 passes over WiFi.

---

### Task 1: ESP-IDF project skeleton and configuration menu

**Files:**
- Create: `esp32s3/CMakeLists.txt`
- Create: `esp32s3/sdkconfig.defaults`
- Create: `esp32s3/main/CMakeLists.txt`
- Create: `esp32s3/main/idf_component.yml`
- Create: `esp32s3/main/Kconfig.projbuild`
- Create: `esp32s3/main/main.cpp`
- Create: `esp32s3/components/hover_core/CMakeLists.txt`
- Create: `esp32s3/README.md`
- Modify: `.gitignore` (append ESP-IDF build outputs and local sdkconfig)

**Interfaces:**
- Produces: `CONFIG_HB_*` Kconfig symbols used by all later ESP tasks: `HB_WIFI_MODE_OFF|STA|AP`, `HB_WIFI_SSID`, `HB_WIFI_PASSWORD`, `HB_WIFI_AP_CHANNEL`, `HB_HOSTNAME`, `HB_USB_NET`, `HB_UDP_PORT`, `HB_HOVER_RX_GPIO`, `HB_HOVER_TX_GPIO`, `HB_HOVER_BAUD`, `HB_CMD_TIMEOUT_MS`, `HB_CMD_LIMIT`, `HB_ENABLE_IMU`, `HB_IMU_RX_GPIO`, `HB_IMU_BAUD`, `HB_ENABLE_LIDAR`, `HB_LIDAR_RX_GPIO`, `HB_LIDAR_TX_GPIO`, `HB_LIDAR_BAUD`, `HB_LIDAR_AUTOSTART`, `HB_LIDAR_START_RETRY_MS`.

- [x] **Step 1: Install ESP-IDF v5.4.x**

ESP-IDF v5.4.4 is installed in `~/esp/esp-idf`. Run `. ~/esp/esp-idf/export.sh` so `idf.py` is on PATH.

Run: `idf.py --version`
Expected: `ESP-IDF v5.4.x`

- [x] **Step 2: Create the project files**

`esp32s3/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hoverboard_ros)
```

`esp32s3/sdkconfig.defaults`:
```
CONFIG_IDF_TARGET="esp32s3"
CONFIG_FREERTOS_HZ=1000
CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192
CONFIG_TINYUSB_NET_MODE_NCM=y
CONFIG_LWIP_UDP_RECVMBOX_SIZE=32
CONFIG_COMPILER_CXX_EXCEPTIONS=n
```

`esp32s3/main/CMakeLists.txt` (Task 1 version, extended in Tasks 7–9):
```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES hover_core)
```

`esp32s3/main/idf_component.yml`:
```yaml
dependencies:
  idf: ">=5.4,<5.5"
  espressif/esp_tinyusb: "^1.7.0"
  espressif/mdns: "^1.8.0"
```

`esp32s3/components/hover_core/CMakeLists.txt` (sources are added in Tasks 2–4; a component without sources is valid):
```cmake
idf_component_register(
    INCLUDE_DIRS "include")
```

Also create the empty directory `esp32s3/components/hover_core/include/hover_core/` with a `.gitkeep` file.

- [x] **Step 3: Write the configuration menu**

`esp32s3/main/Kconfig.projbuild`:
```
menu "Hoverboard ROS bridge"

    choice HB_WIFI_MODE
        prompt "WiFi mode"
        default HB_WIFI_MODE_AP
        config HB_WIFI_MODE_OFF
            bool "Off"
        config HB_WIFI_MODE_STA
            bool "Station (join an existing network)"
        config HB_WIFI_MODE_AP
            bool "Access point (the ESP creates its own network)"
    endchoice

    config HB_WIFI_SSID
        string "WiFi SSID"
        default "hoverboard-robot"
        depends on !HB_WIFI_MODE_OFF
        help
            Station: network to join. Access point: name of the network created by the ESP.

    config HB_WIFI_PASSWORD
        string "WiFi password"
        default "change-me-now"
        depends on !HB_WIFI_MODE_OFF
        help
            Access point mode requires at least 8 characters.

    config HB_WIFI_AP_CHANNEL
        int "Access point channel"
        range 1 13
        default 6
        depends on HB_WIFI_MODE_AP

    config HB_HOSTNAME
        string "mDNS hostname"
        default "hoverboard"

    config HB_USB_NET
        bool "Enable USB tethering (CDC-NCM network, ESP at 192.168.7.1)"
        default n

    config HB_UDP_PORT
        int "UDP port"
        range 1024 65535
        default 8888

    config HB_HOVER_RX_GPIO
        int "Hoverboard UART RX GPIO"
        default 4

    config HB_HOVER_TX_GPIO
        int "Hoverboard UART TX GPIO"
        default 5

    config HB_HOVER_BAUD
        int "Hoverboard UART baud (must match USART3_BAUD in firmware)"
        default 115200

    config HB_CMD_TIMEOUT_MS
        int "Stop wheels if no command for this time [ms]"
        range 50 2000
        default 300

    config HB_CMD_LIMIT
        int "Wheel command clamp (1000 = N_MOT_MAX)"
        range 0 1000
        default 1000

    config HB_ENABLE_IMU
        bool "Enable WT901C IMU on UART0"
        default n
        help
            UART0 is also the default console. Set Component config > ESP System Settings >
            Channel for console output = None when enabling the IMU.

    config HB_IMU_RX_GPIO
        int "IMU UART RX GPIO"
        default 6
        depends on HB_ENABLE_IMU

    config HB_IMU_BAUD
        int "IMU UART baud"
        default 9600
        depends on HB_ENABLE_IMU

    config HB_ENABLE_LIDAR
        bool "Enable LIDAR pass-through on UART2"
        default y

    config HB_LIDAR_RX_GPIO
        int "LIDAR UART RX GPIO"
        default 7
        depends on HB_ENABLE_LIDAR

    config HB_LIDAR_TX_GPIO
        int "LIDAR UART TX GPIO"
        default 8
        depends on HB_ENABLE_LIDAR

    config HB_LIDAR_BAUD
        int "LIDAR UART baud"
        default 230400
        depends on HB_ENABLE_LIDAR

    config HB_LIDAR_AUTOSTART
        bool "Send the LDROBOT start command to the LIDAR"
        default y
        depends on HB_ENABLE_LIDAR
        help
            The STL-50B2 stays silent until it receives 54 A0 04 00 00 00 00 5E.
            Disable only to drive the LIDAR entirely from the PC over LIDAR_TX.

    config HB_LIDAR_START_RETRY_MS
        int "LIDAR start command retry period [ms]"
        default 1000
        depends on HB_LIDAR_AUTOSTART
        help
            The start command is resent this often while no LIDAR bytes arrive,
            so a command lost during power-up does not leave the line silent.

endmenu
```

- [x] **Step 4: Write a minimal main**

`esp32s3/main/main.cpp` (Task 1 version, replaced in Task 9):
```cpp
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "hoverboard_ros skeleton, UDP port %d", CONFIG_HB_UDP_PORT);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

- [x] **Step 5: Ignore build outputs and local configuration**

Append to `.gitignore`:
```
# ESP-IDF project
esp32s3/build/
esp32s3/managed_components/
esp32s3/dependencies.lock
esp32s3/sdkconfig
esp32s3/sdkconfig.old
```

- [x] **Step 6: Write the README**

`esp32s3/README.md`:
````markdown
# hoverboard_ros (ESP32-S3 firmware)

Bridges the hoverboard mainboard, a WT901C IMU and a serial LIDAR to ROS2 over UDP.
UDP travels over WiFi (station or access point) and/or USB tethering (CDC-NCM).

## Build and configure
```
cd esp32s3
idf.py set-target esp32s3      # first time only
idf.py menuconfig              # "Hoverboard ROS bridge" menu: WiFi mode, SSID, password, pins
idf.py build
```
`sdkconfig` holds your WiFi credentials and is git-ignored.

## Flash
The board is a Waveshare ESP32-S3-Zero: it has a single USB-C port, wired to the native USB.
With USB tethering enabled, that port is a USB network device (CDC-NCM), not a serial port.
Before plugging the USB-C cable, disconnect the step-down output from the ESP 5V pin.
To flash, enter download mode: hold BOOT, press and release RESET, release BOOT.
The board then shows up as a serial port again.
```
idf.py -p <PORT> flash
```
Press RESET after flashing to start the firmware.

## Reaching the ESP
| Medium | ESP address |
|---|---|
| USB tethering | 192.168.7.1 (PC gets 192.168.7.2 by DHCP) |
| WiFi access point | 192.168.4.1 |
| WiFi station | the static IP set in menuconfig (default 192.168.1.50), or hoverboard.local |

UDP port: 8888. Quick test without ROS, from any machine with Python 3 on the robot network:
`python3 -m hoverboard_bridge.link_probe --host <ESP address>` (run from `ros2/hoverboard_bridge`).

## Wiring
Power: mainboard 12V output -> DSN-mini-360 step-down trimmed to 5.0V -> ESP 5V pin, LIDAR,
WT901C and level shifter HV. The level shifter LV side runs from the ESP 3V3 pin. All grounds are common.

| Device | Device pin | ESP32-S3 |
|---|---|---|
| Hoverboard right sensor cable (USART3, 3.3V) | GND / TX / RX | GND / GPIO3 / GPIO4 direct. NEVER connect the red 15V wire |
| WT901C (5V) | VCC / TX / RX / GND | 5V rail / shifter HV3-LV3 -> GPIO5 / GPIO6 -> shifter LV4-HV4 / GND |
| LIDAR (5V) | 5V / GND / TX / RX | 5V rail / GND / shifter HV1-LV1 -> GPIO7 / GPIO8 -> shifter LV2-HV2 |
| MH level converter | HV / LV / GND | 5V rail / 3V3 / GND |
| DSN-mini-360 step-down | OUT+ / OUT- | 5V / GND |

Only the edge pins of the ESP32-S3-Zero are used (5V, GND, 3V3, GPIO1-13, GPIO43/44);
the pads on the top and bottom faces are left unsoldered.
The ESP GPIOs are not 5V tolerant: LIDAR and IMU UART lines always go through the shifter.
Before plugging the USB-C cable, disconnect the step-down output from the ESP 5V pin.
````

- [x] **Step 7: Build**

Run (from `esp32s3`): `idf.py set-target esp32s3` then `idf.py build`
Expected: build succeeds. `managed_components/espressif__esp_tinyusb` and `espressif__mdns` are downloaded. If `^1.7.0` of `esp_tinyusb` cannot be resolved, set the highest available `1.x` version in `idf_component.yml`; do not use 2.x.

Run: `idf.py menuconfig`
Expected: "Hoverboard ROS bridge" menu shows all options. Under Component config > TinyUSB Stack, the network driver is NCM.

---

### Task 2: hover_core – hoverboard UART protocol

**Files:**
- Create: `esp32s3/components/hover_core/include/hover_core/hover_protocol.h`
- Create: `esp32s3/components/hover_core/src/hover_protocol.cpp`
- Modify: `esp32s3/components/hover_core/CMakeLists.txt`
- Create: `esp32s3/test_host/test_core.cpp`
- Create: `esp32s3/test_host/run_tests.sh`

**Interfaces:**
- Produces (namespace `hover`):
  - `struct Command` (packed, 8 bytes), `struct Feedback` (packed, 22 bytes), `constexpr uint16_t kStartFrame`, `constexpr int16_t kOdomWrap`
  - `Command makeCommand(int16_t left, int16_t right)`
  - `class FeedbackParser { bool push(uint8_t); const Feedback& last() const; uint32_t okCount() const; uint32_t badCount() const; }`
  - `class TickUnwrapper { int32_t update(int16_t raw); int32_t total() const; }`

- [x] **Step 1: Write the host test harness and failing tests**

`esp32s3/test_host/test_core.cpp`:
```cpp
// Host tests for hover_core. Build and run with test_host/run_tests.sh.
#include <cstdio>
#include <cstring>

#include "hover_core/hover_protocol.h"

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

int main()
{
    testHoverProtocol();
    if (g_failures == 0) std::printf("All host tests passed\n");
    return g_failures == 0 ? 0 : 1;
}
```

`esp32s3/test_host/run_tests.sh`:
```bash
#!/usr/bin/env bash
# Builds hover_core with the host compiler and runs the tests.
set -euo pipefail
cd "$(dirname "$0")/.."
g++ -std=c++17 -Wall -Wextra -I components/hover_core/include \
    components/hover_core/src/*.cpp test_host/test_core.cpp -o /tmp/hover_core_tests
/tmp/hover_core_tests
```

- [x] **Step 2: Run to verify it fails** (skipped: the implementation already existed when the tests were first run)

Run (from `esp32s3`): `bash test_host/run_tests.sh`
Expected: compile error, `hover_core/hover_protocol.h: No such file or directory`.

- [x] **Step 3: Implement the protocol**

`components/hover_core/include/hover_core/hover_protocol.h`:
```cpp
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
```

`components/hover_core/src/hover_protocol.cpp`:
```cpp
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
```

Update `components/hover_core/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "src/hover_protocol.cpp"
    INCLUDE_DIRS "include")
```

- [x] **Step 4: Run tests to verify they pass**

Run: `bash test_host/run_tests.sh`
Expected: `All host tests passed`

---

### Task 3: hover_core – UDP link protocol, command watchdog, LIDAR chunker and starter

**Files:**
- Create: `esp32s3/components/hover_core/include/hover_core/link_protocol.h`
- Create: `esp32s3/components/hover_core/src/link_protocol.cpp`
- Create: `esp32s3/components/hover_core/include/hover_core/cmd_watchdog.h`
- Create: `esp32s3/components/hover_core/include/hover_core/lidar_chunker.h`
- Create: `esp32s3/components/hover_core/src/lidar_chunker.cpp`
- Create: `esp32s3/components/hover_core/include/hover_core/lidar_starter.h`
- Modify: `esp32s3/components/hover_core/CMakeLists.txt`
- Modify: `esp32s3/test_host/test_core.cpp`

**Interfaces:**
- Produces (namespace `proto`):
  - `constexpr size_t kHeaderSize = 7`, `constexpr size_t kMaxDatagram = 1024`
  - `enum MsgType : uint8_t { kCmdWheels=0x01, kLidarTx=0x02, kWheelState=0x10, kImu=0x11, kLidarRx=0x12, kLog=0x13 }`
  - `enum StatusBits : uint8_t { kStatusHoverOk=0x01, kStatusCmdTimeout=0x02, kStatusImuOk=0x04 }`
  - packed structs `Header`, `CmdWheels`, `WheelState`, `ImuState`
  - `size_t encode(MsgType type, uint16_t seq, uint32_t stamp_ms, const void* body, size_t body_len, uint8_t* out, size_t out_cap)`
  - `bool decode(const uint8_t* data, size_t len, Header& header, const uint8_t*& body, size_t& body_len)`
- Produces (namespace `util`):
  - `class CmdWatchdog { explicit CmdWatchdog(uint32_t timeout_ms); void feed(uint32_t now_ms); bool expired(uint32_t now_ms) const; }`
  - `class LidarChunker { LidarChunker(size_t max_bytes, uint32_t max_age_ms); size_t push(const uint8_t*, size_t, uint32_t now_ms); bool ready(uint32_t now_ms) const; const uint8_t* data() const; size_t size() const; void clear(); }`
  - `class LidarStarter { explicit LidarStarter(uint32_t retry_ms); void onBytes(uint32_t now_ms); bool tick(uint32_t now_ms); }` plus `constexpr uint8_t kLidarStart[8]`, `kLidarStop[8]`, `kLidarFrameSize`. Header only, no source file.

- [x] **Step 1: Add failing tests**

In `test_host/test_core.cpp`, add the includes below the existing include:
```cpp
#include "hover_core/cmd_watchdog.h"
#include "hover_core/lidar_chunker.h"
#include "hover_core/lidar_starter.h"
#include "hover_core/link_protocol.h"
```

Add these functions above `main()`:
```cpp
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
```

In `main()`, add after `testHoverProtocol();`:
```cpp
    testLinkProtocol();
    testWatchdogAndChunker();
    testLidarStarter();
```

- [x] **Step 2: Run to verify it fails** (skipped: the implementation already existed when the tests were first run)

Run: `bash test_host/run_tests.sh`
Expected: compile error, `hover_core/cmd_watchdog.h: No such file or directory`.

- [x] **Step 3: Implement**

`components/hover_core/include/hover_core/link_protocol.h`:
```cpp
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
```

`components/hover_core/src/link_protocol.cpp`:
```cpp
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
```

`components/hover_core/include/hover_core/cmd_watchdog.h`:
```cpp
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
```

`components/hover_core/include/hover_core/lidar_chunker.h`:
```cpp
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
```

`components/hover_core/src/lidar_chunker.cpp`:
```cpp
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
```

`components/hover_core/include/hover_core/lidar_starter.h`:
```cpp
#pragma once

#include <cstddef>
#include <cstdint>

namespace util {

// LDROBOT command frames for the STL-50B2. See docs/lidar/STL-50B2.md.
inline constexpr size_t kLidarFrameSize = 8;
inline constexpr uint8_t kLidarStart[kLidarFrameSize] = {0x54, 0xA0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E};
inline constexpr uint8_t kLidarStop[kLidarFrameSize] = {0x54, 0xA1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x4A};

// Keeps the LIDAR streaming by resending the start command while the line is silent.
// A start sent before the sensor has booted is lost, and the LIDAR then stays silent
// forever, so one shot at power-up is not enough.
class LidarStarter {
public:
    explicit LidarStarter(uint32_t retry_ms) : retry_ms_(retry_ms) {}

    // Call whenever bytes arrive from the LIDAR.
    void onBytes(uint32_t now_ms)
    {
        last_rx_ms_ = now_ms;
        streaming_ = true;
    }

    // True when the caller must write kLidarStart to the LIDAR UART.
    bool tick(uint32_t now_ms)
    {
        if (!sent_) {  // First call: start straight away
            sent_ = true;
            last_tx_ms_ = now_ms;
            return true;
        }
        if (streaming_ && (now_ms - last_rx_ms_) < retry_ms_) return false;  // Data is flowing
        if ((now_ms - last_tx_ms_) < retry_ms_) return false;                // Last command still fresh
        streaming_ = false;
        last_tx_ms_ = now_ms;
        return true;
    }

private:
    uint32_t retry_ms_;
    uint32_t last_rx_ms_ = 0;
    uint32_t last_tx_ms_ = 0;
    bool streaming_ = false;
    bool sent_ = false;
};

}  // namespace util
```

`CMakeLists.txt` needs no change for this header: `LidarStarter` is header only.

Update `components/hover_core/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "src/hover_protocol.cpp" "src/link_protocol.cpp" "src/lidar_chunker.cpp"
    INCLUDE_DIRS "include")
```

- [x] **Step 4: Run tests to verify they pass**

Run: `bash test_host/run_tests.sh`
Expected: `All host tests passed`

---

### Task 4: hover_core – WT901 IMU parser

**Files:**
- Create: `esp32s3/components/hover_core/include/hover_core/wt901_parser.h`
- Create: `esp32s3/components/hover_core/src/wt901_parser.cpp`
- Modify: `esp32s3/components/hover_core/CMakeLists.txt`
- Modify: `esp32s3/test_host/test_core.cpp`

**Interfaces:**
- Produces (namespace `wt901`):
  - `struct Sample { int16_t acc[3]; int16_t gyro[3]; int16_t angle[3]; }`
  - `class Parser { bool push(uint8_t byte); const Sample& sample() const; uint32_t badCount() const; }`. `push` returns true when an angle packet (0x53) completes; that packet closes the acc/gyro/angle group.

- [x] **Step 1: Add failing test**

In `test_host/test_core.cpp`, add `#include "hover_core/wt901_parser.h"` with the other includes, then add above `main()`:
```cpp
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
```
Add `testWt901Parser();` in `main()` after the other test calls.

- [x] **Step 2: Run to verify it fails** (skipped: the implementation already existed when the tests were first run)

Run: `bash test_host/run_tests.sh`
Expected: compile error, `hover_core/wt901_parser.h: No such file or directory`.

- [x] **Step 3: Implement**

`components/hover_core/include/hover_core/wt901_parser.h`:
```cpp
#pragma once

#include <cstddef>
#include <cstdint>

namespace wt901 {

struct Sample {
    int16_t acc[3];    // Raw, 32768 = 16 g
    int16_t gyro[3];   // Raw, 32768 = 2000 deg/s
    int16_t angle[3];  // Raw roll/pitch/yaw, 32768 = 180 deg
};

// Parses WitMotion 11-byte packets: 0x55, type, 8 data bytes, sum of the first 10 bytes.
class Parser {
public:
    // Returns true when an angle packet completes an acc/gyro/angle group.
    bool push(uint8_t byte);
    const Sample& sample() const { return sample_; }
    uint32_t badCount() const { return bad_; }

private:
    uint8_t buf_[11];
    size_t idx_ = 0;
    Sample sample_{};
    uint32_t bad_ = 0;
};

}  // namespace wt901
```

`components/hover_core/src/wt901_parser.cpp`:
```cpp
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
```

Update `components/hover_core/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "src/hover_protocol.cpp" "src/link_protocol.cpp" "src/lidar_chunker.cpp" "src/wt901_parser.cpp"
    INCLUDE_DIRS "include")
```

- [x] **Step 4: Run tests to verify they pass**

Run: `bash test_host/run_tests.sh`
Expected: `All host tests passed`

Run (host, from `esp32s3`): `idf.py build`
Expected: build succeeds (hover_core compiles for the ESP32-S3 too).

---

### Task 5: ROS2 Humble development container

**Files:**
- Create: `tools/docker-humble/Dockerfile`
- Create: `tools/docker-humble/compose.yaml`
- Create: `tools/docker-humble/ros_env.sh`
- Create: `tools/docker-humble/entrypoint.sh`
- Create: `tools/docker-humble/run.sh` (executable)
- Create: `tools/docker-humble/README.md`
- Modify: `.gitignore` (append Python test and build artefacts)

**Interfaces:**
- Produces: `tools/docker-humble/run.sh [--build] [command...]`, which opens a shell (or runs a command) in a ROS2 Humble container with:
  - host networking and host IPC (UDP to the ESP, DDS discovery and shared memory)
  - repo `ros2/` mounted at `/ws/src`, with `/ws/build`, `/ws/install` and `/ws/log` in named volumes so the repo stays clean
  - a user with the host UID/GID, so files created in `/ws/src` belong to the host user
  - `ldlidar_stl_ros2` prebuilt in the underlay `/opt/ldlidar_ws`
  - ROS2, the underlay and (when built) the `/ws/install` overlay sourced in every shell and command
  - X11 forwarding for `rviz2`

- [x] **Step 1: Write the container files**

`tools/docker-humble/Dockerfile`:
```dockerfile
# ROS2 Humble environment for the hoverboard_bridge workspace.
FROM osrf/ros:humble-desktop

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ARG LDLIDAR_REF=bf668a8

RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-pytest python3-colcon-common-extensions iproute2 iputils-ping \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME}

# LIDAR driver, built once as an underlay.
RUN mkdir -p /opt/ldlidar_ws/src \
    && git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git /opt/ldlidar_ws/src/ldlidar_stl_ros2 \
    && git -C /opt/ldlidar_ws/src/ldlidar_stl_ros2 checkout ${LDLIDAR_REF} \
    && cd /opt/ldlidar_ws \
    && . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Named volumes inherit this ownership on first use.
RUN mkdir -p /ws/src /ws/build /ws/install /ws/log && chown -R ${USER_UID}:${USER_GID} /ws

COPY ros_env.sh entrypoint.sh /opt/hoverboard/
RUN chmod +x /opt/hoverboard/entrypoint.sh \
    && echo 'source /opt/hoverboard/ros_env.sh' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /ws
ENTRYPOINT ["/opt/hoverboard/entrypoint.sh"]
CMD ["bash"]
```

`tools/docker-humble/ros_env.sh`:
```bash
# Sources ROS2 Humble, the LIDAR driver underlay and the workspace overlay (if built).
source /opt/ros/humble/setup.bash
source /opt/ldlidar_ws/install/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi
```

`tools/docker-humble/entrypoint.sh`:
```bash
#!/usr/bin/env bash
# Runs the container command with the ROS2 environment sourced.
source /opt/hoverboard/ros_env.sh
exec "$@"
```

`tools/docker-humble/compose.yaml`:
```yaml
# ROS2 Humble container for the hoverboard_bridge workspace. Start it with run.sh.
services:
  humble:
    build:
      context: .
      args:
        USER_UID: ${HOST_UID:-1000}
        USER_GID: ${HOST_GID:-1000}
    image: hoverboard-robot/humble
    network_mode: host   # same interfaces as the host: WiFi and USB tethering to the ESP
    ipc: host            # DDS shared memory works between containers on the host network
    environment:
      - DISPLAY=${DISPLAY:-}
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    volumes:
      - ../../ros2:/ws/src
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ws_build:/ws/build
      - ws_install:/ws/install
      - ws_log:/ws/log
    stdin_open: true
    tty: true

volumes:
  ws_build:
  ws_install:
  ws_log:
```

`tools/docker-humble/run.sh`:
```bash
#!/usr/bin/env bash
# Opens a shell, or runs the given command, in the ROS2 Humble container.
# --build rebuilds the image first (needed after editing the Dockerfile).
set -euo pipefail
cd "$(dirname "$0")"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
if [[ "${1:-}" == "--build" ]]; then
    shift
    docker compose build
fi
exec docker compose run --rm humble "$@"
```

`tools/docker-humble/README.md`:
````markdown
# ROS2 Humble container

The ROS2 side of the project targets Humble. This container provides it on any Linux host with Docker.

```
tools/docker-humble/run.sh --build     # first time, or after editing the Dockerfile
tools/docker-humble/run.sh             # interactive shell
tools/docker-humble/run.sh <command>   # one command, e.g. ros2 topic list
```

- `ros2/` from this repo is mounted at `/ws/src`; build with `colcon build --symlink-install` from `/ws`.
  Build outputs live in Docker volumes, not in the repo. `docker compose down -v` (from this folder) wipes them.
- Host networking: the container reaches the ESP at the same addresses as the host
  (192.168.7.1 over USB, 192.168.4.1 on the ESP access point, the static IP from menuconfig in
  station mode). `hoverboard.local` does not resolve inside the container: use the IP.
- `ldlidar_stl_ros2` is prebuilt in the image.
- GUI tools (`rviz2`): run `xhost +local:` on the host once per login session.
- Every shell opened with `run.sh` is a new container; several can run at the same time and see each other's topics.
````

Append to `.gitignore`:
```
.pytest_cache/
*.egg-info/
```

Run: `chmod +x tools/docker-humble/run.sh`

- [x] **Step 2: Build the image**

Run (host, from the repo root): `tools/docker-humble/run.sh --build ros2 pkg prefix ldlidar_stl_ros2`
Expected: the image builds and the command prints `/opt/ldlidar_ws/install/ldlidar_stl_ros2`. If `ldlidar_stl_ros2` fails to compile on Humble, stop and report the error; do not patch the driver.

- [x] **Step 3: Check the environment**

Run: `tools/docker-humble/run.sh bash -c 'echo $ROS_DISTRO; python3 --version; python3 -m pytest --version; id -u'`
Expected: `humble`, `Python 3.10.x`, a pytest version, and the same number as `id -u` on the host.

Run: `tools/docker-humble/run.sh touch /ws/src/.owner_check && ls -n ros2/.owner_check && rm ros2/.owner_check`
Expected: the file is owned by the host UID and GID.

Run: `tools/docker-humble/run.sh ip -br addr`
Expected: the same interfaces as `ip -br addr` on the host.

- [x] **Step 4: Check DDS between two containers**

Run in one terminal: `tools/docker-humble/run.sh ros2 run demo_nodes_cpp talker`
Run in a second terminal: `tools/docker-humble/run.sh ros2 topic echo --once /chatter`
Expected: the second terminal prints one `Hello World: N` message and exits. Stop the talker with Ctrl+C.

- [x] **Step 5: Check the GUI (optional, needs a desktop session)**

Run: `xhost +local:` then `tools/docker-humble/run.sh rviz2`
Expected: the RViz window opens. Close it.

---

### Task 6: ROS2 package skeleton, Python link protocol and bench probe

**Files:**
- Create: `ros2/hoverboard_bridge/package.xml`
- Create: `ros2/hoverboard_bridge/setup.py`
- Create: `ros2/hoverboard_bridge/setup.cfg`
- Create: `ros2/hoverboard_bridge/resource/hoverboard_bridge` (empty file)
- Create: `ros2/hoverboard_bridge/hoverboard_bridge/__init__.py` (empty file)
- Create: `ros2/hoverboard_bridge/hoverboard_bridge/link_protocol.py`
- Create: `ros2/hoverboard_bridge/hoverboard_bridge/link_probe.py`
- Create: `ros2/hoverboard_bridge/test/test_link_protocol.py`

**Interfaces:**
- Consumes: the wire format from Task 3 (`link_protocol.h`).
- Produces (module `hoverboard_bridge.link_protocol`):
  - constants `MSG_CMD_WHEELS, MSG_LIDAR_TX, MSG_WHEEL_STATE, MSG_IMU, MSG_LIDAR_RX, MSG_LOG`, `STATUS_HOVER_OK, STATUS_CMD_TIMEOUT, STATUS_IMU_OK`, `MAX_DATAGRAM`
  - dataclasses `Message(type, seq, stamp_ms, body)`, `WheelState(ticks_l, ticks_r, rpm_l, rpm_r, bat_cv, temp_dc, status, hover_ok, hover_bad)`, `ImuState(acc, gyro, angle)` (tuples of 3 ints)
  - `encode(msg_type: int, seq: int, stamp_ms: int, body: bytes = b'') -> bytes`
  - `decode(datagram: bytes) -> Message | None`
  - `encode_cmd_wheels(seq: int, left: int, right: int) -> bytes`
  - `parse_wheel_state(body: bytes) -> WheelState | None`
  - `parse_imu(body: bytes) -> ImuState | None`
- Produces: console script `link_probe`.

- [x] **Step 1: Write the failing test**

`ros2/hoverboard_bridge/test/test_link_protocol.py`:
```python
from hoverboard_bridge import link_protocol as lp

CMD_GOLDEN = bytes([0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x9C, 0xFF])
STATE_GOLDEN = bytes([0x10, 0x07, 0x00, 0x40, 0xE2, 0x01, 0x00, 0x18, 0xFC, 0xFF, 0xFF,
                      0xD0, 0x07, 0x00, 0x00, 0x32, 0x00, 0xCE, 0xFF, 0x0A, 0x0F, 0x45,
                      0x01, 0x05, 0x64, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00])


def test_cmd_wheels_matches_firmware_bytes():
    assert lp.encode_cmd_wheels(1, 100, -100) == CMD_GOLDEN


def test_wheel_state_decodes_firmware_bytes():
    msg = lp.decode(STATE_GOLDEN)
    assert (msg.type, msg.seq, msg.stamp_ms) == (lp.MSG_WHEEL_STATE, 7, 123456)
    state = lp.parse_wheel_state(msg.body)
    assert state == lp.WheelState(-1000, 2000, 50, -50, 3850, 325, 0x05, 100, 2)


def test_short_inputs_are_rejected():
    assert lp.decode(b'\x10\x00') is None
    assert lp.parse_wheel_state(b'\x00' * 10) is None
    assert lp.parse_imu(b'\x00' * 10) is None
```

- [x] **Step 2: Run to verify it fails**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test/test_link_protocol.py -q`
Expected: FAIL with `ModuleNotFoundError` / `ImportError` for `hoverboard_bridge.link_protocol`.

- [x] **Step 3: Write the package files**

`ros2/hoverboard_bridge/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>hoverboard_bridge</name>
  <version>0.1.0</version>
  <description>UDP bridge between the hoverboard ESP32-S3 firmware and ROS2</description>
  <maintainer email="lorenzodauria97@gmail.com">Lorenzo D'Auria</maintainer>
  <author email="lorenzodauria97@gmail.com">Lorenzo D'Auria</author>
  <license>GPL-3.0-only</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>tf2_ros_py</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`ros2/hoverboard_bridge/setup.py`:
```python
from glob import glob

from setuptools import setup

package_name = 'hoverboard_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author="Lorenzo D'Auria",
    author_email='lorenzodauria97@gmail.com',
    maintainer="Lorenzo D'Auria",
    maintainer_email='lorenzodauria97@gmail.com',
    description='UDP bridge between the hoverboard ESP32-S3 firmware and ROS2',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = hoverboard_bridge.bridge_node:main',
            'link_probe = hoverboard_bridge.link_probe:main',
        ],
    },
)
```

`ros2/hoverboard_bridge/setup.cfg`:
```
[develop]
script_dir=$base/lib/hoverboard_bridge
[install]
install_scripts=$base/lib/hoverboard_bridge
```

Create the empty files `resource/hoverboard_bridge` and `hoverboard_bridge/__init__.py`.

- [x] **Step 4: Implement the protocol module**

`ros2/hoverboard_bridge/hoverboard_bridge/link_protocol.py`:
```python
"""UDP message format shared with the ESP32-S3 firmware (hover_core/link_protocol.h)."""
from dataclasses import dataclass
import struct

HEADER = struct.Struct('<BHI')
CMD_WHEELS = struct.Struct('<hh')
WHEEL_STATE = struct.Struct('<iihhhhBII')
IMU_STATE = struct.Struct('<9h')

MAX_DATAGRAM = 1024

MSG_CMD_WHEELS = 0x01
MSG_LIDAR_TX = 0x02
MSG_WHEEL_STATE = 0x10
MSG_IMU = 0x11
MSG_LIDAR_RX = 0x12
MSG_LOG = 0x13

STATUS_HOVER_OK = 0x01
STATUS_CMD_TIMEOUT = 0x02
STATUS_IMU_OK = 0x04


@dataclass
class Message:
    type: int
    seq: int
    stamp_ms: int
    body: bytes


@dataclass
class WheelState:
    ticks_l: int
    ticks_r: int
    rpm_l: int
    rpm_r: int
    bat_cv: int
    temp_dc: int
    status: int
    hover_ok: int
    hover_bad: int


@dataclass
class ImuState:
    acc: tuple
    gyro: tuple
    angle: tuple


def encode(msg_type, seq, stamp_ms, body=b''):
    return HEADER.pack(msg_type, seq & 0xFFFF, stamp_ms & 0xFFFFFFFF) + bytes(body)


def decode(datagram):
    if len(datagram) < HEADER.size:
        return None
    msg_type, seq, stamp_ms = HEADER.unpack_from(datagram)
    return Message(msg_type, seq, stamp_ms, bytes(datagram[HEADER.size:]))


def encode_cmd_wheels(seq, left, right):
    return encode(MSG_CMD_WHEELS, seq, 0, CMD_WHEELS.pack(left, right))


def parse_wheel_state(body):
    if len(body) != WHEEL_STATE.size:
        return None
    return WheelState(*WHEEL_STATE.unpack(body))


def parse_imu(body):
    if len(body) != IMU_STATE.size:
        return None
    values = IMU_STATE.unpack(body)
    return ImuState(values[0:3], values[3:6], values[6:9])
```

- [x] **Step 5: Run tests to verify they pass**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test/test_link_protocol.py -q`
Expected: `3 passed`

- [x] **Step 6: Write the bench probe**

It runs without ROS: `python3 -m hoverboard_bridge.link_probe` from `ros2/hoverboard_bridge`, with the host Python or in the container.

`ros2/hoverboard_bridge/hoverboard_bridge/link_probe.py`:
```python
"""Bench tool without ROS: sends wheel commands over UDP and prints ESP telemetry."""
import argparse
import socket
import time

from hoverboard_bridge import link_protocol as lp


def format_status(state, imu, lidar_bytes):
    if state is None:
        return 'no telemetry'
    flags = []
    if state.status & lp.STATUS_HOVER_OK:
        flags.append('HOVER_OK')
    if state.status & lp.STATUS_CMD_TIMEOUT:
        flags.append('CMD_TIMEOUT')
    if state.status & lp.STATUS_IMU_OK:
        flags.append('IMU_OK')
    text = (f'ticks L={state.ticks_l:7d} R={state.ticks_r:7d} | rpm L={state.rpm_l:4d} R={state.rpm_r:4d} | '
            f'bat={state.bat_cv / 100:.2f}V temp={state.temp_dc / 10:.1f}C | '
            f'ok={state.hover_ok} bad={state.hover_bad} | {" ".join(flags)}')
    if imu is not None:
        text += f' | imu angle={imu.angle}'
    return text + f' | lidar={lidar_bytes}B'


def print_ticks_per_rev(first, last, elapsed, rpm_sum, rpm_count):
    """Estimates ticks per wheel revolution from tick deltas and mean measured rpm."""
    if first is None or rpm_count == 0 or elapsed <= 0:
        return
    for name, d_ticks, rpm in (('left', last.ticks_l - first.ticks_l, rpm_sum[0] / rpm_count),
                               ('right', last.ticks_r - first.ticks_r, rpm_sum[1] / rpm_count)):
        revs = rpm / 60.0 * elapsed
        estimate = abs(d_ticks / revs) if abs(revs) > 0.5 else float('nan')  # Right rpm and ticks differ in sign
        print(f'{name}: d_ticks={d_ticks} mean_rpm={rpm:.1f} ticks_per_rev~{estimate:.1f}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.50',
                        help='ESP address: WiFi STA static IP (192.168.1.50), WiFi AP 192.168.4.1, USB 192.168.7.1')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--left', type=int, default=0, help='Left wheel command (-1000..1000)')
    parser.add_argument('--right', type=int, default=0, help='Right wheel command (-1000..1000)')
    parser.add_argument('--duration', type=float, default=5.0, help='Seconds before stopping')
    args = parser.parse_args()

    addr = (socket.gethostbyname(args.host), args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    seq = 0
    state = imu = None
    first_state = None
    first_time = last_time = 0.0
    rpm_sum = [0.0, 0.0]
    rpm_count = 0
    lidar_bytes = 0
    t_start = time.monotonic()
    t_cmd = t_print = 0.0

    while time.monotonic() - t_start < args.duration:
        now = time.monotonic()
        if now - t_cmd >= 0.05:
            sock.sendto(lp.encode_cmd_wheels(seq, args.left, args.right), addr)
            seq += 1
            t_cmd = now
        while True:
            try:
                data, _ = sock.recvfrom(lp.MAX_DATAGRAM)
            except (BlockingIOError, ConnectionResetError):  # Windows reports ICMP unreachable as reset
                break
            msg = lp.decode(data)
            if msg is None:
                continue
            if msg.type == lp.MSG_WHEEL_STATE:
                state = lp.parse_wheel_state(msg.body)
                if state is not None and state.status & lp.STATUS_HOVER_OK:
                    if first_state is None:
                        first_state, first_time = state, now
                    last_time = now
                    rpm_sum[0] += state.rpm_l
                    rpm_sum[1] += state.rpm_r
                    rpm_count += 1
            elif msg.type == lp.MSG_IMU:
                imu = lp.parse_imu(msg.body)
            elif msg.type == lp.MSG_LIDAR_RX:
                lidar_bytes += len(msg.body)
            elif msg.type == lp.MSG_LOG:
                print('[esp]', msg.body.decode('ascii', errors='replace').rstrip())
        if now - t_print >= 0.2:
            t_print = now
            print(format_status(state, imu, lidar_bytes))
        time.sleep(0.005)

    for _ in range(5):
        sock.sendto(lp.encode_cmd_wheels(seq, 0, 0), addr)
        seq += 1
        time.sleep(0.05)
    print_ticks_per_rev(first_state, state, last_time - first_time, rpm_sum, rpm_count)


if __name__ == '__main__':
    main()
```

- [x] **Step 7: Smoke test the probe without hardware**

Run (host, from `ros2/hoverboard_bridge`): `python3 -m hoverboard_bridge.link_probe --host 127.0.0.1 --duration 1`
Expected: prints `no telemetry` about 5 times and exits with code 0.

---

### Task 7: ESP networking – WiFi station (static IP) or access point with mDNS

**Files:**
- Modify: `esp32s3/main/Kconfig.projbuild`
- Create: `esp32s3/main/net_wifi.h`
- Create: `esp32s3/main/net_wifi.cpp`
- Modify: `esp32s3/main/main.cpp`
- Modify: `esp32s3/main/CMakeLists.txt`
- Modify: `esp32s3/README.md`

**Interfaces:**
- Consumes: `CONFIG_HB_WIFI_*`, `CONFIG_HB_HOSTNAME` (Task 1).
- Produces: Kconfig symbols `HB_WIFI_STA_STATIC_IP`, `HB_WIFI_STA_IP`, `HB_WIFI_STA_NETMASK`, `HB_WIFI_STA_GATEWAY`.
- Produces: `esp_err_t wifi_start();`. It must be called after `nvs_flash_init()`, `esp_netif_init()` and `esp_event_loop_create_default()`.

- [ ] **Step 1: Add the static IP options**

In `main/Kconfig.projbuild`, insert after the `HB_WIFI_AP_CHANNEL` entry:
```
    config HB_WIFI_STA_STATIC_IP
        bool "Station: use a static IP"
        default y
        depends on HB_WIFI_MODE_STA
        help
            Gives the ESP a fixed address on the router network, so the PC (and the ROS2
            container, which has no mDNS) can reach it without discovery. Choose an address
            in the router subnet but outside its DHCP pool. Off: the router assigns the IP.

    config HB_WIFI_STA_IP
        string "Station static IP"
        default "192.168.1.50"
        depends on HB_WIFI_STA_STATIC_IP

    config HB_WIFI_STA_NETMASK
        string "Station netmask"
        default "255.255.255.0"
        depends on HB_WIFI_STA_STATIC_IP

    config HB_WIFI_STA_GATEWAY
        string "Station gateway"
        default "192.168.1.1"
        depends on HB_WIFI_STA_STATIC_IP
```

In `esp32s3/README.md`, replace the "WiFi station" row of the "Reaching the ESP" table with:
```
| WiFi station | the static IP set in menuconfig (default 192.168.1.50), or hoverboard.local |
```

- [ ] **Step 2: Implement WiFi start**

`main/net_wifi.h`:
```cpp
#pragma once

#include "esp_err.h"

// Starts WiFi in the Kconfig-selected mode (station or access point) and mDNS.
// In station mode the IP is static (HB_WIFI_STA_STATIC_IP) or assigned by DHCP.
esp_err_t wifi_start();
```

`main/net_wifi.cpp`:
```cpp
#include "net_wifi.h"

#include <cstring>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "sdkconfig.h"

static const char* TAG = "wifi";

#ifdef CONFIG_HB_WIFI_STA_STATIC_IP
// Replaces the DHCP client with the Kconfig address. No DNS server is set: the bridge never resolves names.
static esp_err_t setStaticIp(esp_netif_t* sta)
{
    esp_netif_ip_info_t info = {};
    if (esp_netif_str_to_ip4(CONFIG_HB_WIFI_STA_IP, &info.ip) != ESP_OK ||
        esp_netif_str_to_ip4(CONFIG_HB_WIFI_STA_NETMASK, &info.netmask) != ESP_OK ||
        esp_netif_str_to_ip4(CONFIG_HB_WIFI_STA_GATEWAY, &info.gw) != ESP_OK) {
        ESP_LOGE(TAG, "Invalid static IP settings in menuconfig");
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = esp_netif_dhcpc_stop(sta);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        return err;
    }
    return esp_netif_set_ip_info(sta, &info);
}
#endif

static void onWifiEvent(void*, esp_event_base_t base, int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Disconnected from %s, retrying", CONFIG_HB_WIFI_SSID);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(data);
        ESP_LOGI(TAG, "Station IP " IPSTR, IP2STR(&event->ip_info.ip));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Client joined access point %s", CONFIG_HB_WIFI_SSID);
    }
}

esp_err_t wifi_start()
{
    if (std::strlen(CONFIG_HB_WIFI_PASSWORD) < 8) {
        ESP_LOGE(TAG, "WiFi password must have at least 8 characters");
        return ESP_ERR_INVALID_ARG;
    }

#ifdef CONFIG_HB_WIFI_MODE_STA
    esp_netif_t* sta = esp_netif_create_default_wifi_sta();
#ifdef CONFIG_HB_WIFI_STA_STATIC_IP
    ESP_RETURN_ON_ERROR(setStaticIp(sta), TAG, "static IP");
#endif
#else
    esp_netif_t* ap = esp_netif_create_default_wifi_ap();  // 192.168.4.1 with DHCP server
    uint8_t offer = 0;                                       // Do not offer a default route or DNS
    esp_netif_dhcps_option(ap, ESP_NETIF_OP_SET, ESP_NETIF_ROUTER_SOLICITATION_ADDRESS, &offer, sizeof(offer));
#endif

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "esp_wifi_init");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &onWifiEvent, nullptr), TAG,
                        "wifi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &onWifiEvent, nullptr), TAG,
                        "ip handler");

    wifi_config_t cfg = {};
#ifdef CONFIG_HB_WIFI_MODE_STA
    strlcpy((char*)cfg.sta.ssid, CONFIG_HB_WIFI_SSID, sizeof(cfg.sta.ssid));
    strlcpy((char*)cfg.sta.password, CONFIG_HB_WIFI_PASSWORD, sizeof(cfg.sta.password));
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set_mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &cfg), TAG, "set_config");
#else
    strlcpy((char*)cfg.ap.ssid, CONFIG_HB_WIFI_SSID, sizeof(cfg.ap.ssid));
    strlcpy((char*)cfg.ap.password, CONFIG_HB_WIFI_PASSWORD, sizeof(cfg.ap.password));
    cfg.ap.ssid_len = (uint8_t)std::strlen(CONFIG_HB_WIFI_SSID);
    cfg.ap.channel = CONFIG_HB_WIFI_AP_CHANNEL;
    cfg.ap.max_connection = 4;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "set_mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &cfg), TAG, "set_config");
#endif

    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start");
    esp_wifi_set_ps(WIFI_PS_NONE);  // Lowest latency

    ESP_RETURN_ON_ERROR(mdns_init(), TAG, "mdns_init");
    mdns_hostname_set(CONFIG_HB_HOSTNAME);
    ESP_LOGI(TAG, "WiFi started, mDNS name %s.local", CONFIG_HB_HOSTNAME);
    return ESP_OK;
}
```

- [ ] **Step 3: Call it from main**

Replace `main/main.cpp` with:
```cpp
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "net_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char* TAG = "main";

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#ifndef CONFIG_HB_WIFI_MODE_OFF
    ESP_ERROR_CHECK(wifi_start());
#endif

    ESP_LOGI(TAG, "hoverboard_ros networking up");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

Replace `main/CMakeLists.txt` with:
```cmake
idf_component_register(
    SRCS "main.cpp" "net_wifi.cpp"
    INCLUDE_DIRS "."
    REQUIRES hover_core esp_wifi esp_netif esp_event nvs_flash)
```

- [ ] **Step 4: Build**

Run: `idf.py build`
Expected: build succeeds.

Run: `idf.py menuconfig`
Expected: with WiFi mode = Station, the four static IP options appear; with Access point, they are hidden.

- [ ] **Step 5: Hardware check (needs the ESP32-S3; ask the user before flashing)**

AP mode (default): `idf.py -p <PORT> flash monitor`. Connect the PC to SSID `hoverboard-robot`, then run `ping 192.168.4.1`.
Expected: the monitor shows `WiFi started`, ping replies arrive.

STA mode: in `idf.py menuconfig` select Station, set SSID and password, and set the static IP, netmask and gateway for the router network (check the subnet with `ip -br addr` on the host). Flash again, then run `ping <HB_WIFI_STA_IP>` from the host and `tools/docker-humble/run.sh ping -c 3 <HB_WIFI_STA_IP>` from the container.
Expected: the monitor shows `Station IP <HB_WIFI_STA_IP>`, ping replies arrive from both. `ping hoverboard.local` from the host also replies.

Repeat once with `Station: use a static IP` off.
Expected: `Station IP` shows a router-assigned address and ping to it replies.

---

### Task 8 (deferred): USB tethering

Moved to "Deferred: USB tethering" at the end of the plan. It is implemented only after Task 14 passes over WiFi. Until then, follow "USB tethering compatibility" in Global Constraints.

---

### Task 9: ESP bridge loop (UDP ↔ hoverboard, IMU, LIDAR, logs)

**Files:**
- Create: `esp32s3/main/uart_port.h`
- Create: `esp32s3/main/uart_port.cpp`
- Create: `esp32s3/main/udp_link.h`
- Create: `esp32s3/main/udp_link.cpp`
- Create: `esp32s3/main/log_forward.h`
- Create: `esp32s3/main/log_forward.cpp`
- Create: `esp32s3/main/bridge.h`
- Create: `esp32s3/main/bridge.cpp`
- Modify: `esp32s3/main/main.cpp` (final version)
- Modify: `esp32s3/main/CMakeLists.txt` (final version)
- Modify: `esp32s3/main/Kconfig.projbuild` (add `HB_IMU_TX_GPIO`)

**Interfaces:**
- Consumes: `hover::*` (Task 2), `proto::*`, `util::CmdWatchdog`, `util::LidarChunker`, `util::LidarStarter` (Task 3), `wt901::Parser` (Task 4), `wifi_start()` (Task 7).
- Produces:
  - `class UartPort { esp_err_t open(uart_port_t port, int rx_gpio, int tx_gpio, int baud, size_t rx_buffer); size_t read(uint8_t* buf, size_t max); void write(const uint8_t* data, size_t len); bool isOpen() const; }`
  - `class UdpLink { bool open(uint16_t port); size_t poll(uint8_t* buf, size_t cap); void commitPeer(); void send(const uint8_t* data, size_t len); bool hasPeer() const; }`
  - `void log_forward_install(); size_t log_forward_read(char* buf, size_t cap); constexpr size_t kLogLineMax = 128;`
  - `void bridge_run();` (never returns)

- [ ] **Step 0: IMU TX pin option**

In `main/Kconfig.projbuild`, insert after the `HB_IMU_RX_GPIO` entry:
```
    config HB_IMU_TX_GPIO
        int "IMU UART TX GPIO"
        default 9
        depends on HB_ENABLE_IMU
        help
            Wired to the WT901C RX through the level shifter. Nothing is sent yet;
            the line idles high.
```

- [ ] **Step 1: UART wrapper**

`main/uart_port.h`:
```cpp
#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/uart.h"
#include "esp_err.h"

// Non-blocking UART access with a driver-side RX ring buffer.
class UartPort {
public:
    esp_err_t open(uart_port_t port, int rx_gpio, int tx_gpio, int baud, size_t rx_buffer);
    size_t read(uint8_t* buf, size_t max);
    void write(const uint8_t* data, size_t len);
    bool isOpen() const { return open_; }

private:
    uart_port_t port_ = UART_NUM_MAX;
    bool open_ = false;
};
```

`main/uart_port.cpp`:
```cpp
#include "uart_port.h"

#include "esp_check.h"

static const char* TAG = "uart";

esp_err_t UartPort::open(uart_port_t port, int rx_gpio, int tx_gpio, int baud, size_t rx_buffer)
{
    uart_config_t cfg = {};
    cfg.baud_rate = baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_RETURN_ON_ERROR(uart_param_config(port, &cfg), TAG, "param_config %d", port);
    ESP_RETURN_ON_ERROR(uart_set_pin(port, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG,
                        "set_pin %d", port);
    ESP_RETURN_ON_ERROR(uart_driver_install(port, (int)rx_buffer, 0, 0, nullptr, 0), TAG, "driver_install %d", port);
    port_ = port;
    open_ = true;
    return ESP_OK;
}

size_t UartPort::read(uint8_t* buf, size_t max)
{
    if (!open_) return 0;
    int n = uart_read_bytes(port_, buf, (uint32_t)max, 0);
    return n > 0 ? (size_t)n : 0;
}

void UartPort::write(const uint8_t* data, size_t len)
{
    if (open_) uart_write_bytes(port_, data, len);
}
```

- [ ] **Step 2: UDP link**

`main/udp_link.h`:
```cpp
#pragma once

#include <cstddef>
#include <cstdint>

#include "lwip/sockets.h"

// Non-blocking UDP socket on all interfaces. Replies go to the committed peer.
class UdpLink {
public:
    bool open(uint16_t port);
    // Returns the datagram size, or 0 if nothing is pending. Remembers its sender.
    size_t poll(uint8_t* buf, size_t cap);
    // Makes the sender of the last polled datagram the telemetry destination.
    void commitPeer();
    // Sends to the committed peer; dropped if there is none.
    void send(const uint8_t* data, size_t len);
    bool hasPeer() const { return has_peer_; }

private:
    int sock_ = -1;
    sockaddr_in last_from_{};
    sockaddr_in peer_{};
    bool has_peer_ = false;
};
```

`main/udp_link.cpp`:
```cpp
#include "udp_link.h"

#include <fcntl.h>
#include <unistd.h>

#include "esp_log.h"

static const char* TAG = "udp";

bool UdpLink::open(uint16_t port)
{
    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock_ < 0) {
        ESP_LOGE(TAG, "socket() failed");
        return false;
    }
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() to port %u failed", port);
        close(sock_);
        sock_ = -1;
        return false;
    }
    fcntl(sock_, F_SETFL, O_NONBLOCK);
    ESP_LOGI(TAG, "Listening on UDP port %u", port);
    return true;
}

size_t UdpLink::poll(uint8_t* buf, size_t cap)
{
    if (sock_ < 0) return 0;
    socklen_t from_len = sizeof(last_from_);
    int n = recvfrom(sock_, buf, cap, 0, (sockaddr*)&last_from_, &from_len);
    return n > 0 ? (size_t)n : 0;
}

void UdpLink::commitPeer()
{
    bool changed = !has_peer_ || peer_.sin_addr.s_addr != last_from_.sin_addr.s_addr ||
                   peer_.sin_port != last_from_.sin_port;
    peer_ = last_from_;
    has_peer_ = true;
    if (changed) {
        char ip[16];
        inet_ntoa_r(peer_.sin_addr, ip, sizeof(ip));
        ESP_LOGI(TAG, "Telemetry peer %s:%u", ip, ntohs(peer_.sin_port));
    }
}

void UdpLink::send(const uint8_t* data, size_t len)
{
    if (sock_ < 0 || !has_peer_) return;
    sendto(sock_, data, len, 0, (sockaddr*)&peer_, sizeof(peer_));  // Drops silently when buffers are full
}
```

- [ ] **Step 3: Log forwarding**

`main/log_forward.h`:
```cpp
#pragma once

#include <cstddef>

constexpr size_t kLogLineMax = 128;

// Hooks ESP logging so each line can also be sent as a LOG datagram.
void log_forward_install();
// Copies one pending log line into buf. Returns its length, 0 if none. Non-blocking.
size_t log_forward_read(char* buf, size_t cap);
```

`main/log_forward.cpp`:
```cpp
#include "log_forward.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace {

struct LogLine {
    size_t len;
    char text[kLogLineMax];
};

QueueHandle_t s_queue = nullptr;
vprintf_like_t s_previous = nullptr;

int forwardVprintf(const char* fmt, va_list args)
{
    if (s_queue != nullptr) {
        LogLine line;
        va_list copy;
        va_copy(copy, args);
        int n = vsnprintf(line.text, sizeof(line.text), fmt, copy);
        va_end(copy);
        if (n > 0) {
            line.len = (size_t)n < sizeof(line.text) ? (size_t)n : sizeof(line.text) - 1;
            if (xQueueSend(s_queue, &line, 0) != pdTRUE) {
                LogLine oldest;  // Queue full: drop the oldest line so the newest survive
                xQueueReceive(s_queue, &oldest, 0);
                xQueueSend(s_queue, &line, 0);
            }
        }
    }
    return s_previous != nullptr ? s_previous(fmt, args) : 0;
}

}  // namespace

void log_forward_install()
{
    s_queue = xQueueCreate(16, sizeof(LogLine));
    s_previous = esp_log_set_vprintf(forwardVprintf);
}

size_t log_forward_read(char* buf, size_t cap)
{
    LogLine line;
    if (s_queue == nullptr || xQueueReceive(s_queue, &line, 0) != pdTRUE) return 0;
    size_t n = line.len < cap ? line.len : cap;
    std::memcpy(buf, line.text, n);
    return n;
}
```

- [ ] **Step 4: The bridge**

`main/bridge.h`:
```cpp
#pragma once

// Runs the UART <-> UDP bridge loop. Never returns.
void bridge_run();
```

`main/bridge.cpp`:
```cpp
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
```

- [ ] **Step 5: Final main and CMake**

Replace `main/main.cpp` with:
```cpp
#include "bridge.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "log_forward.h"
#include "net_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#if defined(CONFIG_HB_ENABLE_IMU) && !defined(CONFIG_ESP_CONSOLE_NONE)
#error "The IMU uses UART0: set Component config > ESP System Settings > Channel for console output = None"
#endif

static const char* TAG = "main";

extern "C" void app_main(void)
{
    log_forward_install();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#ifndef CONFIG_HB_WIFI_MODE_OFF
    ESP_ERROR_CHECK(wifi_start());
#endif

    ESP_LOGI(TAG, "hoverboard_ros started, UDP port %d", CONFIG_HB_UDP_PORT);
    bridge_run();
}
```

Replace `main/CMakeLists.txt` with:
```cmake
idf_component_register(
    SRCS "main.cpp" "net_wifi.cpp" "udp_link.cpp" "uart_port.cpp" "log_forward.cpp" "bridge.cpp"
    INCLUDE_DIRS "."
    REQUIRES hover_core esp_wifi esp_netif esp_event nvs_flash esp_driver_uart esp_timer lwip)
```

- [ ] **Step 6: Build**

Run: `idf.py build`
Expected: build succeeds with no warnings from `main/`.

- [ ] **Step 7: Hardware check with the hoverboard (ask the user before flashing; wheels lifted)**

In `idf.py menuconfig`, Hoverboard ROS bridge: WiFi mode = Station with SSID, password and static IP for the robot network (as in Task 7), USB tethering = no. The PC joins the same network.

Flash, unplug the USB-C cable, then wire the hoverboard (see "Electrical wiring") and power the ESP from the step-down.

Run (from `ros2/hoverboard_bridge`): `python3 -m hoverboard_bridge.link_probe --host <HB_WIFI_STA_IP> --duration 5`
Expected: `[esp] ... Telemetry peer <PC IP>:...` log line, then status lines with `HOVER_OK`, a plausible battery voltage and `bad` not increasing.

Run: `python3 -m hoverboard_bridge.link_probe --host <HB_WIFI_STA_IP> --left 50 --right 50 --duration 5`
Expected: both wheels spin, `rpm` values are non-zero, and wheels stop after the probe ends.

Watchdog: start the same command and kill the probe with Ctrl+C mid-run.
Expected: wheels stop within about 0.3 s.

Watchdog flag (no hoverboard needed): a new probe cannot see `CMD_TIMEOUT`, because its first command both selects it as telemetry peer and feeds the watchdog. Instead, send commands for 1 s, then stop and keep listening on the same socket. Run from `ros2/hoverboard_bridge`:
```bash
PYTHONPATH=. python3 - <HB_WIFI_STA_IP> <<'EOF'
import socket, sys, time
from hoverboard_bridge import link_protocol as lp
addr = (sys.argv[1], 8888)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setblocking(False)
t0 = time.monotonic(); t_cmd = -1.0; seq = 0; last = None; n = 0
while (now := time.monotonic() - t0) < 2.0:
    if now < 1.0 and now - t_cmd >= 0.05:
        s.sendto(lp.encode_cmd_wheels(seq, 0, 0), addr); seq += 1; t_cmd = now
    try:
        while True:
            m = lp.decode(s.recvfrom(2048)[0])
            if m and m.type == lp.MSG_WHEEL_STATE:
                n += 1
                timeout = bool(lp.parse_wheel_state(m.body).status & lp.STATUS_CMD_TIMEOUT)
                if timeout != last:
                    print(f't={now:.3f}s CMD_TIMEOUT={timeout}, last command at {t_cmd:.3f}s')
                    last = timeout
    except BlockingIOError:
        pass
    time.sleep(0.002)
print(f'WHEEL_STATE received: {n} in 2 s')
EOF
```
Expected: `CMD_TIMEOUT=False` at the start, `CMD_TIMEOUT=True` about 300-320 ms after the last command, about 100 `WHEEL_STATE` messages (50 Hz).

Optional: reflash with WiFi mode = Access point and repeat the first probe command with `--host 192.168.4.1` (PC joined to the ESP access point).
Expected: same telemetry.

---

### Task 10: IMU and LIDAR bring-up on the ESP

**Files:**
- Modify: `ros2/hoverboard_bridge/hoverboard_bridge/link_probe.py`
- Modify: `esp32s3/README.md`

**Interfaces:**
- Consumes: `IMU` and `LIDAR_RX` datagrams from Task 9; `MSG_LIDAR_TX` and `encode` from Task 6.
- Produces: `link_probe` options `--lidar-hex "<hex bytes>"` (sent once to the LIDAR, for extra commands on top of the start the ESP already sends) and a hex dump of the first 64 received LIDAR bytes.

- [ ] **Step 1: Extend the probe for LIDAR identification**

In `link_probe.py`, add after the `--duration` argument:
```python
    parser.add_argument('--lidar-hex', default='',
                        help='Hex bytes sent once to the LIDAR after the first command, '
                             'e.g. "54 A2 04 D8 09 00 00 16" to set 7 Hz')
```

Replace the command-sending block inside the loop:
```python
        if now - t_cmd >= 0.05:
            sock.sendto(lp.encode_cmd_wheels(seq, args.left, args.right), addr)
            seq += 1
            t_cmd = now
```
with:
```python
        if now - t_cmd >= 0.05:
            sock.sendto(lp.encode_cmd_wheels(seq, args.left, args.right), addr)
            seq += 1
            t_cmd = now
            if args.lidar_hex and seq == 1:
                sock.sendto(lp.encode(lp.MSG_LIDAR_TX, seq, 0, bytes.fromhex(args.lidar_hex)), addr)
                seq += 1
```

Replace the LIDAR branch:
```python
            elif msg.type == lp.MSG_LIDAR_RX:
                lidar_bytes += len(msg.body)
```
with:
```python
            elif msg.type == lp.MSG_LIDAR_RX:
                if lidar_bytes < 64:
                    print('[lidar]', msg.body[:64 - lidar_bytes].hex(' '))
                lidar_bytes += len(msg.body)
```

- [ ] **Step 2: Smoke test**

Run: `python3 -m hoverboard_bridge.link_probe --host 127.0.0.1 --duration 1 --lidar-hex "54 A3 04 00 00 00 00 62"`
Expected: exits with code 0 and prints `no telemetry`.

- [ ] **Step 3: IMU hardware check (ask the user before flashing)**

In `idf.py menuconfig`:
- Hoverboard ROS bridge > Enable WT901C IMU on UART0: yes
- Component config > ESP System Settings > Channel for console output: None
- WiFi in station mode, USB tethering off (as in Task 9 Step 7)

Build and flash, then unplug the USB-C cable before powering the ESP from the step-down.

Wire the WT901C as in "Electrical wiring": VCC to the 5V rail, TX through shifter HV3 -> LV3 to GPIO5, GPIO6 through shifter LV4 -> HV4 to RX. Before connecting the shifter, measure the WT901C TX idle voltage and record it in "Electrical wiring".

Run: `python3 -m hoverboard_bridge.link_probe --host <HB_WIFI_STA_IP> --duration 10`
Expected: status lines contain `IMU_OK`, and `imu angle=(...)` changes when the IMU is tilted.

- [ ] **Step 4: LIDAR hardware check**

Wire the LIDAR as in "Electrical wiring": 5V rail, TX through shifter HV1 -> LV1 to GPIO7, GPIO8 through shifter LV2 -> HV2 to RX. Before connecting the shifter, measure the LIDAR TX idle voltage and record it in "Electrical wiring".

The ESP sends the start command by itself, so nothing has to be typed to make the LIDAR spin.

Run: `python3 -m hoverboard_bridge.link_probe --host <HB_WIFI_STA_IP> --duration 5`
Expected: the motor spins up within a second of the ESP booting, `[lidar] 54 2c ...` appears (the LDROBOT data header) and the `lidar=` counter grows at roughly 17 kB/s. The first bytes of the dump also show the command echo `54 a0 04 00 00 00 00 5e` and the health packet `54 1f 00 00 00 41`, whose last-but-one byte is the error mode: `00` means normal.

If the line stays silent, check in this order: TX and RX not swapped, ground shared with the ESP, shifter powered (HV 5V, LV 3.3V), each signal on the matching HV/LV channel pair, LIDAR on a 5V rail that can supply the motor, `HB_LIDAR_AUTOSTART` enabled in menuconfig. The baud is known to be 230400 and the protocol is documented in `docs/lidar/STL-50B2.md`: do not go hunting for either.

- [ ] **Step 5: Document findings**

Append to `esp32s3/README.md`:
````markdown
## Sensor notes
- WT901C: enabling the IMU requires console output = None; ESP logs are then visible only through `link_probe` or the ROS node.
- LIDAR STL-50B2: LDROBOT protocol at 230400 baud. The ESP sends `54 A0 04 00 00 00 00 5E` at
  boot and resends it while the line is silent, so the sensor spins with no action from the PC.
  Protocol notes: `docs/lidar/STL-50B2.md`.
  To send an extra command by hand, for example set the speed to 7 Hz:
  `python3 -m hoverboard_bridge.link_probe --host <ESP address> --lidar-hex "54 A2 04 D8 09 00 00 16"`.
````

---

### Task 11: PC kinematics and IMU conversion

**Files:**
- Create: `ros2/hoverboard_bridge/hoverboard_bridge/kinematics.py`
- Create: `ros2/hoverboard_bridge/hoverboard_bridge/imu_convert.py`
- Create: `ros2/hoverboard_bridge/test/test_kinematics.py`
- Create: `ros2/hoverboard_bridge/test/test_imu_convert.py`

**Interfaces:**
- Produces (module `hoverboard_bridge.kinematics`):
  - `@dataclass DiffDriveParams(wheel_radius: float, wheel_separation: float, ticks_per_rev: int, n_mot_max: float, max_cmd: int = 1000)`
  - `twist_to_commands(p: DiffDriveParams, linear: float, angular: float) -> tuple[int, int]` (left, right firmware units)
  - `rpm_to_rad_s(rpm: float) -> float`
  - `class OdometryIntegrator(p)`: `update(ticks_l: int, ticks_r: int) -> None`, attributes `x, y, theta`, `ticks_to_angle(ticks: int) -> float`
- Produces (module `hoverboard_bridge.imu_convert`):
  - `acc_to_m_s2(raw: int) -> float`, `gyro_to_rad_s(raw: int) -> float`, `angle_to_rad(raw: int) -> float`
  - `euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]` (x, y, z, w)

- [ ] **Step 1: Write failing tests**

`ros2/hoverboard_bridge/test/test_kinematics.py`:
```python
import math

import pytest

from hoverboard_bridge.kinematics import DiffDriveParams, OdometryIntegrator, twist_to_commands

P = DiffDriveParams(wheel_radius=0.0825, wheel_separation=0.40, ticks_per_rev=90, n_mot_max=200.0)


def test_straight_and_rotation_commands():
    assert twist_to_commands(P, 0.5, 0.0) == (289, 289)
    assert twist_to_commands(P, 0.0, 1.0) == (-116, 116)


def test_commands_are_clamped():
    assert twist_to_commands(P, 10.0, 0.0) == (1000, 1000)


def test_odometry_straight_one_revolution():
    odom = OdometryIntegrator(P)
    odom.update(0, 0)
    odom.update(90, 90)
    assert odom.x == pytest.approx(2 * math.pi * 0.0825)
    assert odom.y == pytest.approx(0.0, abs=1e-9)
    assert odom.theta == pytest.approx(0.0, abs=1e-9)


def test_odometry_rotation_in_place():
    odom = OdometryIntegrator(P)
    odom.update(0, 0)
    odom.update(-45, 45)
    assert odom.theta == pytest.approx(2 * math.pi * 0.0825 / 2 * 2 / 0.40)
    assert odom.x == pytest.approx(0.0, abs=1e-9)
```

`ros2/hoverboard_bridge/test/test_imu_convert.py`:
```python
import math

import pytest

from hoverboard_bridge import imu_convert


def test_raw_scaling():
    assert imu_convert.acc_to_m_s2(2048) == pytest.approx(9.80665)
    assert imu_convert.gyro_to_rad_s(16384) == pytest.approx(math.radians(1000.0))
    assert imu_convert.angle_to_rad(-16384) == pytest.approx(-math.pi / 2)


def test_yaw_quaternion():
    x, y, z, w = imu_convert.euler_to_quaternion(0.0, 0.0, math.pi / 2)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert (z, w) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))
```

- [ ] **Step 2: Run to verify they fail**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test/test_kinematics.py test/test_imu_convert.py -q`
Expected: FAIL with `ModuleNotFoundError` for `hoverboard_bridge.kinematics`.

- [ ] **Step 3: Implement**

`ros2/hoverboard_bridge/hoverboard_bridge/kinematics.py`:
```python
"""Differential drive math for the hoverboard (hub motors: motor rpm = wheel rpm)."""
from dataclasses import dataclass
import math


@dataclass
class DiffDriveParams:
    wheel_radius: float      # [m]
    wheel_separation: float  # [m] distance between wheel contact points
    ticks_per_rev: int       # Hall transitions per wheel revolution
    n_mot_max: float         # [rpm] firmware N_MOT_MAX; command 1000 maps to it
    max_cmd: int = 1000


def rpm_to_rad_s(rpm):
    return rpm * 2.0 * math.pi / 60.0


def _speed_to_cmd(p, speed):
    rpm = speed / p.wheel_radius * 60.0 / (2.0 * math.pi)
    cmd = round(rpm * 1000.0 / p.n_mot_max)
    return max(-p.max_cmd, min(p.max_cmd, cmd))


def twist_to_commands(p, linear, angular):
    """Returns (left, right) firmware commands for a body twist."""
    half = angular * p.wheel_separation / 2.0
    return _speed_to_cmd(p, linear - half), _speed_to_cmd(p, linear + half)


class OdometryIntegrator:
    """Integrates cumulative wheel ticks into a planar pose."""

    def __init__(self, p):
        self.p = p
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev = None

    def ticks_to_angle(self, ticks):
        return ticks * 2.0 * math.pi / self.p.ticks_per_rev

    def update(self, ticks_l, ticks_r):
        if self._prev is None:
            self._prev = (ticks_l, ticks_r)
            return
        d_left = self.ticks_to_angle(ticks_l - self._prev[0]) * self.p.wheel_radius
        d_right = self.ticks_to_angle(ticks_r - self._prev[1]) * self.p.wheel_radius
        self._prev = (ticks_l, ticks_r)
        distance = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.p.wheel_separation
        heading = self.theta + d_theta / 2.0
        self.x += distance * math.cos(heading)
        self.y += distance * math.sin(heading)
        self.theta = math.atan2(math.sin(self.theta + d_theta), math.cos(self.theta + d_theta))
```

`ros2/hoverboard_bridge/hoverboard_bridge/imu_convert.py`:
```python
"""WT901 raw values to SI units (default ranges: 16 g, 2000 deg/s)."""
import math

GRAVITY = 9.80665
ACC_RANGE_G = 16.0
GYRO_RANGE_DEG_S = 2000.0


def acc_to_m_s2(raw):
    return raw / 32768.0 * ACC_RANGE_G * GRAVITY


def gyro_to_rad_s(raw):
    return math.radians(raw / 32768.0 * GYRO_RANGE_DEG_S)


def angle_to_rad(raw):
    return math.radians(raw / 32768.0 * 180.0)


def euler_to_quaternion(roll, pitch, yaw):
    """ZYX Euler angles to quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)
```

- [ ] **Step 4: Run tests to verify they pass**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test -q`
Expected: all tests pass (`9 passed`).

---

### Task 12: LIDAR pseudo terminal

**Files:**
- Create: `ros2/hoverboard_bridge/hoverboard_bridge/lidar_pty.py`
- Create: `ros2/hoverboard_bridge/test/test_lidar_pty.py`

**Interfaces:**
- Produces: `class LidarPty(link_path: str)` with `write(data: bytes) -> None` (bytes from the LIDAR, for the driver), `read(max_bytes: int = 512) -> bytes` (bytes written by the driver, for the LIDAR; `b''` if none), `close() -> None`, attribute `slave_name`. Linux only.

- [ ] **Step 1: Write the failing test**

`ros2/hoverboard_bridge/test/test_lidar_pty.py`:
```python
import os
import sys

import pytest

pytestmark = pytest.mark.skipif(not sys.platform.startswith('linux'), reason='pty is Linux only')


def test_bytes_flow_both_ways(tmp_path):
    from hoverboard_bridge.lidar_pty import LidarPty
    link = str(tmp_path / 'ttyLIDAR')
    lidar = LidarPty(link)
    try:
        driver = os.open(link, os.O_RDWR | os.O_NOCTTY)
        lidar.write(b'\x54\x2c')
        assert os.read(driver, 2) == b'\x54\x2c'
        os.write(driver, b'\xa5\x60')
        data = b''
        for _ in range(100):
            data += lidar.read()
            if len(data) >= 2:
                break
        assert data == b'\xa5\x60'
        os.close(driver)
    finally:
        lidar.close()
    assert not os.path.lexists(link)
```

- [ ] **Step 2: Run to verify it fails**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test/test_lidar_pty.py -q`
Expected: FAIL with `ModuleNotFoundError` for `hoverboard_bridge.lidar_pty`.

- [ ] **Step 3: Implement**

`ros2/hoverboard_bridge/hoverboard_bridge/lidar_pty.py`:
```python
"""Pseudo terminal that exposes the LIDAR byte stream to a serial LIDAR driver."""
import os
import pty
import tty


class LidarPty:

    def __init__(self, link_path):
        self._master, self._slave = pty.openpty()
        tty.setraw(self._slave)  # No echo or line editing; the driver may reconfigure it
        os.set_blocking(self._master, False)
        self.slave_name = os.ttyname(self._slave)
        self.link_path = link_path
        if os.path.lexists(link_path):
            os.remove(link_path)
        os.symlink(self.slave_name, link_path)

    def write(self, data):
        """Bytes from the LIDAR; dropped if the driver is not reading."""
        try:
            os.write(self._master, data)
        except (BlockingIOError, OSError):
            pass

    def read(self, max_bytes=512):
        """Bytes written by the driver, to be forwarded to the LIDAR."""
        try:
            return os.read(self._master, max_bytes)
        except (BlockingIOError, OSError):
            return b''

    def close(self):
        if os.path.lexists(self.link_path):
            os.remove(self.link_path)
        os.close(self._master)
        os.close(self._slave)
```

The slave file descriptor stays open for the whole lifetime of the object. Otherwise reads on the master fail with `EIO` whenever the driver closes the port.

- [ ] **Step 4: Run test to verify it passes**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test/test_lidar_pty.py -q`
Expected: `1 passed`

---

### Task 13: ROS2 bridge node, parameters and launch file

**Files:**
- Create: `ros2/hoverboard_bridge/hoverboard_bridge/bridge_node.py`
- Create: `ros2/hoverboard_bridge/config/bridge.yaml`
- Create: `ros2/hoverboard_bridge/launch/bridge.launch.py`

**Interfaces:**
- Consumes: `link_protocol` (Task 6), `kinematics`, `imu_convert` (Task 11), `LidarPty` (Task 12).
- Produces, ROS interfaces of node `hoverboard_bridge`:
  - Subscribes `cmd_vel` (`geometry_msgs/Twist`)
  - Publishes `odom` (`nav_msgs/Odometry`), TF `odom -> base_link`, `joint_states` (`sensor_msgs/JointState`, joints `left_wheel_joint`, `right_wheel_joint`), `imu/data` (`sensor_msgs/Imu`), `hoverboard/battery` (`sensor_msgs/BatteryState`), `hoverboard/temperature` (`sensor_msgs/Temperature`)
  - LIDAR serial stream at `/tmp/ttyLIDAR`
  - Launch argument `esp_host`

- [ ] **Step 1: Write the node**

`ros2/hoverboard_bridge/hoverboard_bridge/bridge_node.py`:
```python
"""ROS2 node bridging /cmd_vel and robot telemetry to the ESP32-S3 over UDP."""
import math
import socket

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, JointState, Temperature
from tf2_ros import TransformBroadcaster

from hoverboard_bridge import imu_convert
from hoverboard_bridge import link_protocol as lp
from hoverboard_bridge.kinematics import DiffDriveParams, OdometryIntegrator, rpm_to_rad_s, twist_to_commands

POSE_COVARIANCE_DIAG = (0.01, 0.01, 1e6, 1e6, 1e6, 0.05)
TWIST_COVARIANCE_DIAG = (0.01, 1e6, 1e6, 1e6, 1e6, 0.05)
IMU_ORIENTATION_VARIANCE = 0.01
IMU_GYRO_VARIANCE = 0.001
IMU_ACC_VARIANCE = 0.05
LIDAR_TX_CHUNK = 512


def diagonal(values, size):
    matrix = [0.0] * (size * size)
    for i, value in enumerate(values):
        matrix[i * (size + 1)] = value
    return matrix


class BridgeNode(Node):

    def __init__(self):
        super().__init__('hoverboard_bridge')
        defaults = {
            'esp_host': '192.168.1.50',
            'esp_port': 8888,
            'wheel_radius': 0.0825,
            'wheel_separation': 0.40,
            'ticks_per_rev': 90,
            'n_mot_max': 200.0,
            'max_cmd': 1000,
            'left_cmd_sign': 1,
            'right_cmd_sign': 1,
            'left_ticks_sign': 1,
            'right_ticks_sign': 1,
            'left_rpm_sign': 1,
            'right_rpm_sign': -1,
            'cmd_rate': 20.0,
            'cmd_vel_timeout': 0.5,
            'link_timeout': 1.0,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'imu_frame': 'imu_link',
            'publish_tf': True,
            'enable_lidar': True,
            'lidar_pty_link': '/tmp/ttyLIDAR',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.kin = DiffDriveParams(self._param('wheel_radius'), self._param('wheel_separation'),
                                   self._param('ticks_per_rev'), self._param('n_mot_max'), self._param('max_cmd'))
        self.odom = OdometryIntegrator(self.kin)
        self.cmd_signs = (self._param('left_cmd_sign'), self._param('right_cmd_sign'))
        self.ticks_signs = (self._param('left_ticks_sign'), self._param('right_ticks_sign'))
        self.rpm_signs = (self._param('left_rpm_sign'), self._param('right_rpm_sign'))
        self.cmd_vel_timeout_ns = int(self._param('cmd_vel_timeout') * 1e9)
        self.link_timeout_ns = int(self._param('link_timeout') * 1e9)
        self.odom_frame = self._param('odom_frame')
        self.base_frame = self._param('base_frame')
        self.imu_frame = self._param('imu_frame')

        host = self._param('esp_host')
        self.esp_addr = (socket.gethostbyname(host), self._param('esp_port'))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

        self.seq = 0
        self.twist = (0.0, 0.0)
        self.last_cmd_vel_time = None
        self.last_rx_time = None
        self.link_up = False
        self.last_status = 0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'hoverboard/battery', 10)
        self.temp_pub = self.create_publisher(Temperature, 'hoverboard/temperature', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self._param('publish_tf') else None
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.lidar = None
        if self._param('enable_lidar'):
            from hoverboard_bridge.lidar_pty import LidarPty
            self.lidar = LidarPty(self._param('lidar_pty_link'))
            self.get_logger().info(f'LIDAR stream at {self.lidar.link_path}')

        self.create_timer(1.0 / self._param('cmd_rate'), self.send_command)
        self.create_timer(0.005, self.poll)
        self.create_timer(1.0, self.check_link)
        self.get_logger().info(f'Bridging to ESP at {host} ({self.esp_addr[0]}:{self.esp_addr[1]})')

    def _param(self, name):
        return self.get_parameter(name).value

    def _send(self, datagram):
        try:
            self.sock.sendto(datagram, self.esp_addr)
        except OSError as err:
            self.get_logger().warn(f'UDP send failed: {err}', throttle_duration_sec=5.0)

    def _next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq

    def on_cmd_vel(self, msg):
        self.twist = (msg.linear.x, msg.angular.z)
        self.last_cmd_vel_time = self.get_clock().now()

    def send_command(self):
        left = right = 0
        now = self.get_clock().now()
        if self.last_cmd_vel_time is not None and \
                (now - self.last_cmd_vel_time).nanoseconds < self.cmd_vel_timeout_ns:
            left, right = twist_to_commands(self.kin, *self.twist)
            left *= self.cmd_signs[0]
            right *= self.cmd_signs[1]
        self._send(lp.encode_cmd_wheels(self._next_seq(), left, right))

    def poll(self):
        for _ in range(200):  # Bounded so the executor stays responsive
            try:
                data, source = self.sock.recvfrom(lp.MAX_DATAGRAM)
            except (BlockingIOError, ConnectionResetError):
                break
            if source[0] != self.esp_addr[0]:
                continue
            msg = lp.decode(data)
            if msg is None:
                continue
            self.last_rx_time = self.get_clock().now()
            if msg.type == lp.MSG_WHEEL_STATE:
                state = lp.parse_wheel_state(msg.body)
                if state is not None:
                    self.publish_wheel_state(state)
            elif msg.type == lp.MSG_IMU:
                imu = lp.parse_imu(msg.body)
                if imu is not None:
                    self.publish_imu(imu)
            elif msg.type == lp.MSG_LIDAR_RX and self.lidar is not None:
                self.lidar.write(msg.body)
            elif msg.type == lp.MSG_LOG:
                self.get_logger().info('[esp] ' + msg.body.decode('ascii', errors='replace').rstrip())

        if self.lidar is not None:
            while True:
                chunk = self.lidar.read(LIDAR_TX_CHUNK)
                if not chunk:
                    break
                self._send(lp.encode(lp.MSG_LIDAR_TX, self._next_seq(), 0, chunk))

    def publish_wheel_state(self, state):
        stamp = self.get_clock().now().to_msg()
        ticks_l = state.ticks_l * self.ticks_signs[0]
        ticks_r = state.ticks_r * self.ticks_signs[1]
        w_left = rpm_to_rad_s(state.rpm_l * self.rpm_signs[0])
        w_right = rpm_to_rad_s(state.rpm_r * self.rpm_signs[1])
        self.odom.update(ticks_l, ticks_r)
        linear = (w_left + w_right) * self.kin.wheel_radius / 2.0
        angular = (w_right - w_left) * self.kin.wheel_radius / self.kin.wheel_separation
        qz, qw = math.sin(self.odom.theta / 2.0), math.cos(self.odom.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom.x
        odom.pose.pose.position.y = self.odom.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = diagonal(POSE_COVARIANCE_DIAG, 6)
        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular
        odom.twist.covariance = diagonal(TWIST_COVARIANCE_DIAG, 6)
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.odom.x
            tf.transform.translation.y = self.odom.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['left_wheel_joint', 'right_wheel_joint']
        joints.position = [self.odom.ticks_to_angle(ticks_l), self.odom.ticks_to_angle(ticks_r)]
        joints.velocity = [w_left, w_right]
        self.joint_pub.publish(joints)

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = state.bat_cv / 100.0
        battery.present = True
        self.battery_pub.publish(battery)

        temperature = Temperature()
        temperature.header.stamp = stamp
        temperature.temperature = state.temp_dc / 10.0
        self.temp_pub.publish(temperature)

        self.report_status_changes(state.status)

    def report_status_changes(self, status):
        changed = status ^ self.last_status
        if changed & lp.STATUS_HOVER_OK:
            if status & lp.STATUS_HOVER_OK:
                self.get_logger().info('Hoverboard feedback OK')
            else:
                self.get_logger().warn('No hoverboard feedback on the ESP (check wiring / power)')
        if changed & lp.STATUS_IMU_OK and status & lp.STATUS_IMU_OK:
            self.get_logger().info('IMU data OK')
        self.last_status = status

    def publish_imu(self, imu):
        roll, pitch, yaw = (imu_convert.angle_to_rad(a) for a in imu.angle)
        x, y, z, w = imu_convert.euler_to_quaternion(roll, pitch, yaw)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = x, y, z, w
        msg.orientation_covariance = diagonal([IMU_ORIENTATION_VARIANCE] * 3, 3)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            imu_convert.gyro_to_rad_s(g) for g in imu.gyro)
        msg.angular_velocity_covariance = diagonal([IMU_GYRO_VARIANCE] * 3, 3)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
            imu_convert.acc_to_m_s2(a) for a in imu.acc)
        msg.linear_acceleration_covariance = diagonal([IMU_ACC_VARIANCE] * 3, 3)
        self.imu_pub.publish(msg)

    def check_link(self):
        now = self.get_clock().now()
        up = self.last_rx_time is not None and (now - self.last_rx_time).nanoseconds < self.link_timeout_ns
        if up != self.link_up:
            if up:
                self.get_logger().info('ESP telemetry received')
            else:
                self.get_logger().warn(f'No telemetry from ESP at {self.esp_addr[0]}:{self.esp_addr[1]}')
            self.link_up = up

    def shutdown(self):
        for _ in range(3):
            self._send(lp.encode_cmd_wheels(self._next_seq(), 0, 0))
        if self.lidar is not None:
            self.lidar.close()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Parameters and launch file**

`ros2/hoverboard_bridge/config/bridge.yaml` (floats must keep a decimal point, ROS2 parameter types are strict):
```yaml
hoverboard_bridge:
  ros__parameters:
    esp_host: "192.168.1.50"       # WiFi STA static IP (192.168.1.50), WiFi AP 192.168.4.1, USB 192.168.7.1
    esp_port: 8888
    wheel_radius: 0.0825           # [m] measure and refine in Task 14
    wheel_separation: 0.40         # [m] measure and refine in Task 14
    ticks_per_rev: 90
    n_mot_max: 200.0               # [rpm] must match N_MOT_MAX in firmware config.h
    max_cmd: 1000
    left_cmd_sign: 1
    right_cmd_sign: 1
    left_ticks_sign: 1             # Firmware tick counters count up when moving forward
    right_ticks_sign: 1
    left_rpm_sign: 1               # Right speedR_meas is negative when moving forward
    right_rpm_sign: -1
    cmd_rate: 20.0
    cmd_vel_timeout: 0.5
    link_timeout: 1.0
    odom_frame: "odom"
    base_frame: "base_link"
    imu_frame: "imu_link"
    publish_tf: true
    enable_lidar: true
    lidar_pty_link: "/tmp/ttyLIDAR"
```

`ros2/hoverboard_bridge/launch/bridge.launch.py`:
```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('hoverboard_bridge'), 'config', 'bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('esp_host', default_value='192.168.1.50',
                              description='WiFi STA static IP (192.168.1.50), WiFi AP 192.168.4.1, USB 192.168.7.1'),
        Node(
            package='hoverboard_bridge',
            executable='bridge_node',
            name='hoverboard_bridge',
            output='screen',
            parameters=[params, {'esp_host': LaunchConfiguration('esp_host')}],
        ),
    ])
```

- [ ] **Step 3: Build the workspace**

Run (container, from `/ws`):
```bash
colcon build --symlink-install --packages-select hoverboard_bridge
source install/setup.bash
```
New container shells source `/ws/install/setup.bash` automatically.
Expected: `Finished <<< hoverboard_bridge`.

- [ ] **Step 4: Run without hardware**

Run (container): `ros2 launch hoverboard_bridge bridge.launch.py esp_host:=127.0.0.1`
Expected: logs `LIDAR stream at /tmp/ttyLIDAR` and `Bridging to ESP at 127.0.0.1`, with no exceptions.

In a second container shell (`tools/docker-humble/run.sh`): `ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"` for a few seconds, then `ros2 node info /hoverboard_bridge`.
Expected: the node lists the subscriber and publishers from the Interfaces block; no errors in the first terminal. Stop with Ctrl+C: clean exit, and `/tmp/ttyLIDAR` is removed.

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test -q`
Expected: all tests pass.

---

### Task 14: Robot bring-up and calibration

**Files:**
- Modify: `ros2/hoverboard_bridge/config/bridge.yaml` (calibrated values)
- Modify: `esp32s3/README.md` (append ROS2 usage)

**Interfaces:**
- Consumes: everything above, with the full robot assembled.

- [ ] **Step 1: Link check over WiFi (wheels lifted)**

ESP in station mode (container): `ros2 launch hoverboard_bridge bridge.launch.py esp_host:=<HB_WIFI_STA_IP>`
Expected: `ESP telemetry received`, `Hoverboard feedback OK`. `ros2 topic echo --once /hoverboard/battery` shows the pack voltage.

Optional: after reflashing in access point mode, repeat with the PC on the ESP access point (`esp_host:=192.168.4.1`). USB tethering is not tested; if it is ever used, disconnect the step-down from the ESP 5V pin before plugging the cable.
Expected: same result.

- [ ] **Step 2: Command signs (wheels lifted)**

Run: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"`
Expected: both wheels turn in the robot's forward direction. For each wheel that turns backwards, flip its `*_cmd_sign` in `bridge.yaml`, rebuild (not needed with `--symlink-install`), relaunch and repeat.

- [ ] **Step 3: Feedback signs (wheels lifted, still commanding forward)**

Run: `ros2 topic echo /joint_states`
Expected: both `velocity` values positive and both `position` values increasing. For each wheel with a negative `velocity`, flip its `*_rpm_sign`; for each wheel with a decreasing `position`, flip its `*_ticks_sign`. The defaults already match the bench result of Task 9, so no change is expected.

- [ ] **Step 4: Ticks per revolution (stop the ROS node first)**

Run (host, from `ros2/hoverboard_bridge`): `python3 -m hoverboard_bridge.link_probe --host <HB_WIFI_STA_IP> --left 100 --right 100 --duration 10`
Expected: the final lines print `ticks_per_rev~N` for both wheels. Set `ticks_per_rev` to the nearest integer (90 is expected for 15 pole-pair hub motors; the Task 9 bench run gave about 0.8 revolutions and 70-73 ticks in 5 s at 10 rpm, consistent with 90).

- [ ] **Step 5: Wheel radius and separation (robot on the floor)**

Radius: mark the start, drive straight with `linear.x: 0.2` for about 2 m, then compare the real distance with `ros2 topic echo --once /odom` position x.
Set `wheel_radius_new = wheel_radius_old * real_distance / odom_distance`.

Separation: rotate in place with `angular.z: 0.5` for about one full turn, compare the real rotation (floor marks) with the odom yaw change (`2 * atan2(qz, qw)`, unwrapped).
Set `wheel_separation_new = wheel_separation_old * odom_angle / real_angle`.

Relaunch and repeat both once. Expected: odom within about 5% of the real distance and angle.

- [ ] **Step 6: LIDAR driver hookup**

Precondition: Task 10 saw LDROBOT frames (`54 2c`).

1. `ldlidar_stl_ros2` is already built in the container image (underlay `/opt/ldlidar_ws`, Task 5).
2. In a second container shell, launch it with `port_name` set to `/tmp/ttyLIDAR` and `port_baudrate` set to `230400`, while `bridge.launch.py` is running. The driver never sends a start command of its own, which is why the ESP does it; the driver only has to parse the stream.
3. Run `ros2 topic hz /scan`. Expected: a steady rate around 7 Hz, the measured rotation speed of this unit.

- [ ] **Step 7: Document ROS2 usage**

Append to `esp32s3/README.md`:
````markdown
## ROS2 (Humble)
```
ros2 launch hoverboard_bridge bridge.launch.py esp_host:=<ESP address>
```
Topics: `/cmd_vel` (in), `/odom`, `/tf`, `/joint_states`, `/imu/data`, `/hoverboard/battery`, `/hoverboard/temperature` (out).
LIDAR raw stream for serial drivers: `/tmp/ttyLIDAR`.
Calibrated parameters live in `ros2/hoverboard_bridge/config/bridge.yaml`.
ROS2 runs in the Humble container: `tools/docker-humble/run.sh` (see `tools/docker-humble/README.md`).
````

---

## Deferred: USB tethering

### Task 8 (deferred): ESP networking – USB tethering (CDC-NCM)

Start only after Task 14 passes over WiFi. Task 9 builds `main.cpp` and `CMakeLists.txt` without USB networking; this task adds it on top. Before plugging the USB-C cable, disconnect the step-down from the ESP 5V pin (see "Electrical wiring").

**Files:**
- Create: `esp32s3/main/net_usb.h`
- Create: `esp32s3/main/net_usb.cpp`
- Modify: `esp32s3/main/main.cpp`
- Modify: `esp32s3/main/CMakeLists.txt`

**Interfaces:**
- Consumes: `CONFIG_HB_USB_NET` (Task 1); `esp_netif_init()` and the default event loop already done in `app_main` (Task 7).
- Produces: `esp_err_t usb_net_start();`. It creates a lwIP interface at `192.168.7.1/24` with a DHCP server that offers no router.

Reference implementations to cross-check if the `esp_tinyusb` 1.x API differs in the resolved version: `esp-idf/examples/peripherals/usb/device/tusb_ncm` and `github.com/esp32-open-source/usb-netif-example`.

- [ ] **Step 1: Implement the USB network interface**

`main/net_usb.h`:
```cpp
#pragma once

#include "esp_err.h"

// Starts TinyUSB CDC-NCM and a lwIP interface at 192.168.7.1 with a DHCP server.
esp_err_t usb_net_start();
```

`main/net_usb.cpp`:
```cpp
#include "net_usb.h"

#include <cstdlib>
#include <cstring>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "tinyusb.h"
#include "tinyusb_net.h"

static const char* TAG = "usb_net";

static const uint8_t kHostSideMac[6] = {0x02, 0x48, 0x42, 0x00, 0x07, 0x02};  // Seen by the PC
static const uint8_t kEspSideMac[6] = {0x02, 0x48, 0x42, 0x00, 0x07, 0x01};   // lwIP interface

static esp_netif_t* s_netif = nullptr;

static esp_err_t netifTransmit(void*, void* buffer, size_t len)
{
    return tinyusb_net_send_sync(buffer, (uint16_t)len, nullptr, pdMS_TO_TICKS(100));
}

static void netifFreeRx(void*, void* buffer)
{
    free(buffer);
}

// USB frames are copied because TinyUSB reuses its buffer after the callback.
static esp_err_t usbReceive(void* buffer, uint16_t len, void*)
{
    if (s_netif == nullptr) return ESP_OK;
    void* copy = malloc(len);
    if (copy == nullptr) return ESP_ERR_NO_MEM;
    std::memcpy(copy, buffer, len);
    return esp_netif_receive(s_netif, copy, len, copy);
}

esp_err_t usb_net_start()
{
    esp_netif_ip_info_t ip = {};
    ip.ip.addr = ESP_IP4TOADDR(192, 168, 7, 1);
    ip.netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0);
    ip.gw.addr = 0;

    esp_netif_inherent_config_t base = {};
    base.flags = (esp_netif_flags_t)(ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP);
    std::memcpy(base.mac, kEspSideMac, sizeof(kEspSideMac));
    base.ip_info = &ip;
    base.if_key = "USB_NCM";
    base.if_desc = "usb ncm";
    base.route_prio = 10;

    esp_netif_driver_ifconfig_t driver = {};
    driver.handle = (void*)1;  // Non-null handle required by esp_netif
    driver.transmit = netifTransmit;
    driver.driver_free_rx_buffer = netifFreeRx;

    esp_netif_config_t cfg = {};
    cfg.base = &base;
    cfg.driver = &driver;
    cfg.stack = ESP_NETIF_NETSTACK_DEFAULT_ETH;

    s_netif = esp_netif_new(&cfg);
    ESP_RETURN_ON_FALSE(s_netif != nullptr, ESP_FAIL, TAG, "esp_netif_new");

    uint8_t offer = 0;  // Do not offer a default route or DNS: the PC keeps its internet route
    esp_netif_dhcps_option(s_netif, ESP_NETIF_OP_SET, ESP_NETIF_ROUTER_SOLICITATION_ADDRESS, &offer, sizeof(offer));
    esp_netif_action_start(s_netif, nullptr, 0, nullptr);

    const tinyusb_config_t tusb_cfg = {};  // Default descriptors from Kconfig (NCM enabled)
    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), TAG, "tinyusb_driver_install");

    tinyusb_net_config_t net_cfg = {};
    std::memcpy(net_cfg.mac_addr, kHostSideMac, sizeof(kHostSideMac));
    net_cfg.on_recv_callback = usbReceive;
    ESP_RETURN_ON_ERROR(tinyusb_net_init(TINYUSB_USBDEV_0, &net_cfg), TAG, "tinyusb_net_init");

    ESP_LOGI(TAG, "USB tethering at 192.168.7.1");
    return ESP_OK;
}
```

- [ ] **Step 2: Call it from main**

In `main/main.cpp` (final version from Task 9) add `#include "net_usb.h"` next to `#include "net_wifi.h"`, and add before the `#ifndef CONFIG_HB_WIFI_MODE_OFF` block:
```cpp
#ifdef CONFIG_HB_USB_NET
    ESP_ERROR_CHECK(usb_net_start());
#endif
```

In `main/CMakeLists.txt` add `"net_usb.cpp"` to `SRCS`.

- [ ] **Step 3: Build**

Run: `idf.py build`
Expected: build succeeds. If `tinyusb_net.h` is not found, run `idf.py menuconfig` and check that Component config > TinyUSB Stack > Network driver is NCM, then build again.

- [ ] **Step 4: Hardware check (needs the ESP32-S3; ask the user before flashing)**

In `idf.py menuconfig` enable Hoverboard ROS bridge > Enable USB tethering (off by default). Disconnect the step-down from the ESP 5V pin, then put the board in download mode (hold BOOT, press and release RESET, release BOOT), run `idf.py -p /dev/ttyACM0 flash`, then press RESET. The same USB-C port now enumerates as a network device; no monitor is available on it.

Run: `ip -br addr` and `ip route`.
Expected: a new interface (`usb0` or `enx0248420007xx`) with `192.168.7.2/24`, and no `default via 192.168.7.1` route.

Run: `ping 192.168.7.1`
Expected: replies.

Run (from `ros2/hoverboard_bridge`): `python3 -m hoverboard_bridge.link_probe --host 192.168.7.1 --duration 5`
Expected: the same telemetry as over WiFi in Task 9 Step 7, with `Telemetry peer 192.168.7.2:...`. With WiFi still enabled, a probe on `<HB_WIFI_STA_IP>` also works.

---

## Future Work (out of scope)

- Runtime WiFi configuration stored in NVS (change SSID and password without reflashing).
- ESP-side timestamps mapped to ROS time for LIDAR and IMU data.
- `ros2_control` hardware interface replacing the Python node (for Nav2 and `diff_drive_controller`).
- Use of the LIDAR TIME sync pin.
