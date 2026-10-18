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
| Hoverboard right sensor cable (USART3, 3.3V) | GND / TX / RX | GND / GPIO4 / GPIO5 direct. NEVER connect the red 15V wire |
| WT901C (5V) | VCC / TX / RX / GND | 5V rail / shifter HV3-LV3 -> GPIO6 / GPIO9 -> shifter LV4-HV4 / GND |
| LIDAR (5V) | 5V / GND / TX / RX | 5V rail / GND / shifter HV1-LV1 -> GPIO7 / GPIO8 -> shifter LV2-HV2 |
| MH level converter | HV / LV / GND | 5V rail / 3V3 / GND |
| DSN-mini-360 step-down | OUT+ / OUT- | 5V / GND |

Only the edge pins of the ESP32-S3-Zero are used (5V, GND, 3V3, GPIO1-13, GPIO43/44);
the pads on the top and bottom faces are left unsoldered.
The ESP GPIOs are not 5V tolerant: LIDAR and IMU UART lines always go through the shifter.
Before plugging the USB-C cable, disconnect the step-down output from the ESP 5V pin.

## Bench tests
Run the probe with the host Python 3 from `ros2/hoverboard_bridge`; no ROS or container needed.
The ROS2 container reaches the same addresses, but it cannot resolve `hoverboard.local`: use the IP there.

### Choose the WiFi mode
menuconfig > "Hoverboard ROS bridge" > WiFi mode, then build and flash:
- Access point: set SSID, password (at least 8 characters) and channel. Join the PC to that SSID and use `192.168.4.1`.
  The ESP offers no default route, so the PC keeps internet only through another interface (Ethernet, second WiFi adapter).
- Station: set the router SSID and password. With "Station: use a static IP" the ESP uses the configured IP
  (default 192.168.1.50, pick one outside the router DHCP pool); without it, read the address from the `Station IP` log line.
  `hoverboard.local` works in both cases.

### Expected boot log
```
wifi: WiFi started, mDNS name hoverboard.local
wifi: Station IP 192.168.1.50                 (station)  /  wifi: Client joined access point ...  (access point)
udp: Listening on UDP port 8888
bridge: Bridge running
bridge: LIDAR start command sent              (every second while no LIDAR data arrives)
udp: Telemetry peer <PC IP>:<port>            (each time a probe starts)
```

### Tests, in order
1. Network: `ping <ESP address>`. The first pings can fail while the ESP joins the router.
2. Telemetry, wheels still (safe, also works without the hoverboard):
   ```
   python3 -m hoverboard_bridge.link_probe --host <ESP address> --duration 5
   ```
   A first `no telemetry` line is normal. `[esp]` lines are ESP logs received over UDP.
   With the hoverboard connected expect `HOVER_OK`, `ok` rising by about 100/s, `bad` steady and a plausible
   battery voltage (about 41.7V when charged). `temp` is meaningless until the firmware temperature calibration is done.
3. Spin (wheels lifted):
   ```
   python3 -m hoverboard_bridge.link_probe --host <ESP address> --left 50 --right 50 --duration 5
   ```
   50 is about 10 rpm and moves in steps (hall sensor resolution); 100 is smoother. Negative values reverse,
   `--right 0` spins the left wheel only. Expected: left rpm positive, right rpm negative (the firmware reports the
   right speed with the opposite sign), both tick counters increasing. The final lines print `ticks_per_rev~N`,
   about 90; use `--left 100 --right 100 --duration 10` for a better estimate.
4. Command watchdog: run test 3 with a longer `--duration` and press Ctrl+C mid-run. The wheels stop within about 0.5s
   (ESP timeout `HB_CMD_TIMEOUT_MS` plus the firmware deceleration). To measure the ESP timeout alone, without the
   hoverboard, use the "Watchdog flag" script in Task 9 Step 7 of `docs/superpowers/plans/2026-09-13-hoverboard-ros-udp-bridge.md`.

### Troubleshooting
| Symptom | Likely cause |
|---|---|
| No ping reply | Wrong address, PC on another network, ESP still connecting, static IP outside the router subnet |
| Ping works, probe shows only `no telemetry` | Wrong `--host` or port. The ESP sends telemetry only to a PC that sends wheel commands |
| No `HOVER_OK`, `ok=0` | Hoverboard TX -> GPIO4 wire, missing common ground, hoverboard off |
| `bad` keeps rising | Noisy line or loose contact on GPIO4, ground |
| Hoverboard repeats 3 low beeps | Serial timeout on the mainboard: GPIO5 -> hoverboard RX wire |
| `HOVER_OK` but wheels do not move | `CMD_TIMEOUT` in the status (commands not reaching the ESP), or command too small |
