# hoverboard_ros (ESP32-S3 firmware)

Bridges the hoverboard mainboard, a WT901C IMU and a serial LIDAR to ROS2 over UDP.
UDP travels over USB tethering (CDC-NCM) or WiFi (station or access point).
WiFi is not usable for driving the robot on this board: see "Link results".

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
| Hoverboard right sensor cable (USART3, 3.3V) | GND / TX / RX | GND / GPIO7 / GPIO8 direct. NEVER connect the red 15V wire |
| WT901C (5V supply, 3.3V UART) | VCC / TX / RX / GND | 5V rail / GPIO9 / GPIO10 direct / GND |
| LIDAR (5V) | 5V / GND / TX / RX | 5V rail / GND / shifter HV3-LV3 -> GPIO5 / GPIO6 -> shifter LV4-HV4 |
| MH level converter | HV / LV / GND | 5V rail / 3V3 / GND |
| DSN-mini-360 step-down | OUT+ / OUT- | 5V / GND |

Only the edge pins of the ESP32-S3-Zero are used (5V, GND, 3V3, GPIO1-13, GPIO43/44);
the pads on the top and bottom faces are left unsoldered.
The ESP GPIOs are not 5V tolerant: the LIDAR UART lines (5V) always go through the shifter.
Before plugging the USB-C cable, disconnect the step-down output from the ESP 5V pin.

Reference multimeter readings against GND, everything powered: HV4 5V, LV4 3.3V, WT901C TX at most 3.3V; HV3 about 2.8V
and LV3 about 1.9V with the LIDAR streaming (averages, as the line toggles; peaks not measured).
LV3 and the WT901C TX must never exceed 3.6V.

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
   (ESP timeout `HB_CMD_TIMEOUT_MS` plus the firmware deceleration).

### Troubleshooting
| Symptom | Likely cause |
|---|---|
| No ping reply | Wrong address, PC on another network, ESP still connecting, static IP outside the router subnet |
| Ping works, probe shows only `no telemetry` | Wrong `--host` or port. The ESP sends telemetry only to a PC that sends wheel commands |
| No `HOVER_OK`, `ok=0` | Hoverboard TX -> GPIO7 wire, missing common ground, hoverboard off |
| `bad` keeps rising | Noisy line or loose contact on GPIO7, ground |
| Hoverboard repeats 3 low beeps | Serial timeout on the mainboard: GPIO8 -> hoverboard RX wire |
| `HOVER_OK` but wheels do not move | `CMD_TIMEOUT` in the status (commands not reaching the ESP), or command too small |

## Sensor notes
- WT901C: UART0 at 115200 baud (this unit; the factory default is 9600, set `HB_IMU_BAUD` to match).
  UART0 is also the default console, so `sdkconfig.defaults` sets console output = None; ESP logs are then
  visible only through `link_probe` or the ROS node. Expect `IMU_OK` in the probe status and |acc| about 1 g at rest.
- LIDAR STL-50B2: LDROBOT protocol at 230400 baud. The ESP sends `54 A0 04 00 00 00 00 5E` at
  boot and resends it while the line is silent, so the sensor spins with no action from the PC.
  Expect `[lidar] 54 2c ...` in the probe output and about 17 kB/s at 7 Hz.
  Protocol notes: `docs/lidar/STL-50B2.md`.
  To send an extra command by hand, for example set the speed to 7 Hz:
  `python3 -m hoverboard_bridge.link_probe --host <ESP address> --lidar-hex "54 A2 04 D8 09 00 00 16"`.

## Link results
**Decision (2026-11-03): WiFi is not usable for this robot on the Waveshare ESP32-S3-Zero. Use USB tethering.**
WiFi station and access point remain in the firmware, but they do not meet the driving criterion: under 1% of
datagrams lost and no `CMD_TIMEOUT`. Going back to WiFi would first need a board with a suitably sized antenna.

Hoverboard, IMU and LIDAR connected, wheels still, zero wheel commands at 20 Hz. Loss comes from gaps in `seq`.

| Date | Link | Duration | Datagrams lost | LIDAR packets missing | CMD_TIMEOUT | Ping lost | Ping RTT |
|---|---|---|---|---|---|---|---|
| 2026-10-25 | USB tethering | 20 s | 0% (0 of 3000) | not measured | 0 | 0% | about 2.3 ms |
| 2026-10-18 | Station, via router | 20 s | 13.5%, in bursts | not measured | 67% of states | 15% | up to 1.7 s |
| 2026-11-03 | Access point, channel 6 | 60 s | 40.4% | 36.2% | 0 | 39.7% | median 264 ms |
| 2026-11-03 | Access point, channel 6 | 60 s | 36.8% | 32.7% | 0 | 36.9% | median 244 ms |
| 2026-11-03 | Access point, channel 6, PC WiFi power save off | 60 s | 38.4% | 34.4% | 0 | 38.7% | median 236 ms |
| 2026-11-03 | Access point, channel 6 | 60 s | 37.2% | 33.7% | 0 | 36.4% | median 243 ms |
| 2026-11-03 | Access point, channel 1 | 60 s | 33.3% | 29.8% | 0 | 37.7% | median 235 ms |

Access point results:
- About 120 datagrams/s of the 180 sent arrive, with little change across channel and PC power save settings.
  `WHEEL_STATE` arrives at about 30 Hz instead of 50 Hz. The received LIDAR data has 0 CRC errors.
- The PC disconnects every few seconds until the ESP starts sending telemetry, on both channels. The PC misses the
  ESP beacons, then drops the link (46 drops in about 5.5 minutes on channel 1, 0 in the 150 s after zero wheel
  commands started). The ESP did not reboot during these drops.
- With USB tethering the same traffic has no loss, so the bridge loop and the send path keep up.

The limit is attributed to the small on-board antenna. This is an engineering judgement: the tests did not isolate
the antenna, and no external antenna, other PC positions, AMPDU off or actual PHY rate were tested.

## ROS2
ROS2 usage (bridge, LIDAR driver, mapping, navigation): [`ros2/README.md`](../ros2/README.md).
