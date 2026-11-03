# Hoverboard Robot Bring-up and Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Finish the bring-up of the hoverboard robot (calibration, LIDAR in ROS2, open firmware issues, link quality) and add a ROS2 navigation stack on top: robot description, fused odometry, SLAM with `slam_toolbox` and autonomous navigation with Nav2.

**Architecture:** The ESP32-S3 bridges the hoverboard mainboard, a WT901C IMU and an STL-50B2 LIDAR to a PC over UDP (WiFi or USB tethering). On the PC, `hoverboard_bridge` turns the datagrams into ROS2 topics and exposes the LIDAR byte stream as a pseudo terminal for `ldlidar_stl_ros2`. A new `hoverboard_bringup` package adds the URDF, an EKF (`robot_localization`) fusing wheel odometry with the IMU gyro, `slam_toolbox` for mapping and Nav2 for navigation. Everything ROS2 runs in the `tools/docker-humble` container.

**Tech Stack:** ESP-IDF v5.4.x (C++17), ROS2 Humble (Python 3.10, `rclpy`), `ldlidar_stl_ros2`, `robot_state_publisher`, `robot_localization`, `slam_toolbox`, Nav2 (`nav2_bringup`, Regulated Pure Pursuit controller), Docker.

This plan replaces `2026-09-13-hoverboard-ros-udp-bridge.md`, whose completed tasks are in the git history. Design details not repeated here (wiring, LIDAR protocol, firmware facts) are in that file's history, in `esp32s3/README.md` and in `docs/lidar/STL-50B2.md`.

---

## Status (2026-11-02)

Done and verified on the hardware:
- Final assembly: all components mounted, all links checked over USB tethering with `link_probe` (0 datagrams lost,
  `HOVER_OK` at 99 frames/s, LIDAR at 7 Hz with 0 CRC errors, motors and hall sensors on both sides).
  ESP pins: hoverboard RX/TX GPIO7/8 direct, WT901C RX/TX GPIO9/10 direct (3.3V UART), LIDAR RX/TX GPIO5/6 through
  shifter channels 3 (LIDAR TX) and 4 (LIDAR RX). The ESP -> WT901C line is wired but unused, so it is untested.
- WT901C output rate set to 50 Hz and saved in the sensor (still 50 Hz after a power cycle, 50 messages/s through the ESP).
- ESP firmware: hoverboard UART protocol, UDP link, command watchdog, WT901C IMU, LIDAR start-up and streaming,
  log forwarding. WiFi access point, WiFi station (static IP or DHCP) with mDNS, USB tethering (CDC-NCM).
- ROS2 package `hoverboard_bridge`: link protocol, kinematics, IMU conversion, LIDAR pseudo terminal, bridge node,
  launch file, `link_probe` bench tool. Host tests (`esp32s3/test_host`) and `pytest` (18 tests) pass.
- Calibration: feedback signs (defaults kept), `ticks_per_rev = 90` (measured 92.9 left, 89.3 right).
- Link: over USB tethering, 0 datagrams lost out of 3000 in 20 s with all sensors connected.

Open issues carried into this plan:
- **Packet loss over WiFi** (station mode: 13.5% of ESP->PC datagrams lost, in bursts; `CMD_TIMEOUT` in 67% of
  `WHEEL_STATE`). The bridge loop keeps up (USB shows no loss), so the loss is in the radio link. Closed on 2026-11-03: access point mode also loses 33-40% on channels 1 and 6, and the user decided that WiFi is not viable on this board without a suitably sized antenna. Results are in `esp32s3/README.md`, "Link results".
- **Station mode does not always connect.** The disconnect log does not print the reason code yet. Closed on 2026-11-03 without a fix: WiFi is no longer pursued on this board.
- **`bridge_node` has no automated tests.** Task 5.
- **Resoldered hoverboard TX -> GPIO7 line.** It was found open circuit after the assembly and then repaired. Closed on 2026-11-03: the user considers the connections sound and stable after the floor runs, so the `bad` counter check was not run.
- **Irregular beep during turns in place.** The upstream `BEEPS_BACKWARD` check in the mainboard firmware (`Src/main.c`) needs a wheel command below -50 and a negative mean speed, and the mean speed is about zero while rotating (at 0.5 rad/s the inner wheel command is about -53). The user wants a steady beep whenever the robot is commanded to move, in any direction: `hoverboard/config.patch` replaces the check with `BEEPS_MOVING`, active when either wheel command magnitude is above `BEEPS_MOVING_THRES` (5, on the -1000 to 1000 scale). Error, timeout and low battery beeps keep priority. The mainboard has not been reflashed yet, so it still beeps the old way. This is the only mainboard firmware change in this plan.
- **`inflation_radius` too small for the footprint.** It is 0.45 m, the front corners are 0.57 m from `base_link`, and the planner only considers `base_link`, so a goal can leave the board front against a wall. Proposed: try 0.6 m, checking that doors stay passable, with a bag of `/scan_fixed`, TF and `/odom` to separate cable and wall warnings. Deferred by the user as a refinement; goes in Task 11 Step 7, still open.
- **EKF diverges when the ESP link is lost.** With the USB cable off, `/odometry/filtered` kept integrating with no input (-4.26 m/s and a position of (3.29, 7.99) with the robot still). After a link loss the stack must be restarted. A goal still active when the link returns would start from a wrong pose. Not fixed yet.

## Context

### Target deployment (decided 2026-11-02)
The final robot carries a Pocophone F1 (6 GB RAM) running postmarketOS, with all the ROS2 nodes in the Ubuntu 22.04 container, connected to the ESP by USB tethering (`192.168.7.1`). The ESP connectivity code stays as it is: WiFi station and access point remain available, but the robot as mounted uses tethering. The phone setup runs in parallel in `2026-11-02-pocophone-onboard-computer.md`; if its gates fail, another on-board computer takes its place and works like the development PC. The ROS2 tasks of this plan run on the development PC, with USB tethering for the floor tests because of the WiFi packet loss listed above.

### Transports and addresses
| Medium | ESP address | Notes |
|---|---|---|
| WiFi station | static `HB_WIFI_STA_IP` (default `192.168.1.50`) or DHCP | `hoverboard.local` resolves on the host only, not in the container |
| WiFi access point | `192.168.4.1` | PC joins the ESP network |
| USB tethering | `192.168.7.1` (PC gets `192.168.7.2`) | Flashing needs download mode: hold BOOT, press RESET, release BOOT |

UDP port 8888. Telemetry goes to the last sender of a valid `CMD_WHEELS`. Before plugging the USB-C cable into the ESP, disconnect the step-down output from the ESP 5V pin.

### UDP messages (little endian, header `uint8 type, uint16 seq, uint32 stamp_ms`)
| Type | Id | Direction | Body |
|---|---|---|---|
| `CMD_WHEELS` | 0x01 | PC->ESP | `int16 left, int16 right` (+-1000 = +-200 rpm) |
| `LIDAR_TX` | 0x02 | PC->ESP | raw bytes to the LIDAR UART |
| `WHEEL_STATE` | 0x10 | ESP->PC, 50 Hz | `int32 ticks_l, ticks_r, int16 rpm_l, rpm_r, bat_cv, temp_dc, uint8 status, uint32 hover_ok, hover_bad` |
| `IMU` | 0x11 | ESP->PC | `int16 acc[3], gyro[3], angle[3]` raw WT901 units |
| `LIDAR_RX` | 0x12 | ESP->PC | raw LIDAR bytes |
| `LOG` | 0x13 | ESP->PC | one ESP log line |

`seq` is one counter shared by all ESP->PC messages, so gaps in it measure datagram loss.

### ROS2 interfaces of `hoverboard_bridge`
In: `/cmd_vel`. Out: `/odom`, `/tf` (`odom` -> `base_link`, parameter `publish_tf`), `/joint_states` (`left_wheel_joint`, `right_wheel_joint`), `/imu/data` (frame `imu_link`), `/hoverboard/battery`, `/hoverboard/temperature`. LIDAR byte stream: pseudo terminal `/tmp/ttyLIDAR`.

### Frames after this plan
```
map -> odom                  slam_toolbox (mapping) or AMCL (localization)
odom -> base_link            EKF (robot_localization), or hoverboard_bridge with use_ekf:=false
base_link -> chassis_link, laser_link, imu_link            robot_state_publisher (fixed joints)
base_link -> left_wheel_link, right_wheel_link             robot_state_publisher (/joint_states)
```
`base_link` lies on the floor, midway between the two wheel contact points, x forward, y left, z up. There is no separate `base_footprint`.

## Global Constraints

- All code comments, logs and identifiers in English. Comments short and useful; no historical "changed from" comments.
- Version control is handled by the user: tasks contain no git steps and executors run no git write operations.
- Ask the user for confirmation before flashing any device. During hardware checks, do not edit code on your own initiative: run the check, report what was seen, propose the next step and wait.
- Package metadata (author, maintainer): "Lorenzo D'Auria" <lorenzodauria97@gmail.com>.
- ESP-IDF v5.4.x, target `esp32s3`. ROS2 Humble, Ubuntu 22.04, Python 3.10, inside the `tools/docker-humble` image. `link_probe` must keep running with the host Python 3.12 (standard library only).
- The hoverboard mainboard firmware is not changed by this plan.
- Wire formats must match byte-for-byte between `components/hover_core` (C++) and `hoverboard_bridge/link_protocol.py`.
- WiFi credentials live only in the local, git-ignored `esp32s3/sdkconfig`.
- Robot speed limits while testing: 0.3 m/s and 1.0 rad/s. Keep a hand near the hoverboard power button during the first runs of every task that moves the robot on the floor.
- Every test step must actually be run; report failures plainly.

### Where each step runs
| Where | Toolchain | Used for |
|---|---|---|
| Host (Ubuntu 24.04) | ESP-IDF v5.4.4 in `~/esp/esp-idf` (`. ~/esp/esp-idf/export.sh` once per terminal), `g++`, Docker | ESP build, host tests, flashing, `link_probe` |
| Container (`tools/docker-humble`) | ROS2 Humble, `colcon`, `pytest` | everything ROS2, marked "(container)" |

Container shells: `tools/docker-humble/run.sh` from the repo root. Every `run.sh` call starts a new container with its own `/tmp`, so the LIDAR driver must run in the same container as the bridge node (same launch, or `docker exec -it <container> bash`). Containers use host networking and see each other's topics. The repo folder `ros2/` is mounted at `/ws/src`; build from `/ws` with `colcon build --symlink-install`. GUI tools need `xhost +local:` on the host once per login session.

## File Structure

```
esp32s3/
  components/hover_core/include/hover_core/hover_protocol.h   modify: TickUnwrapper::reset()
  main/bridge.cpp                                             modify: reset tick reference when feedback resumes
  main/net_wifi.cpp                                           modify: disconnect reason, RSSI and channel in logs
  test_host/test_core.cpp                                     modify: reset() test
  README.md                                                   modify: link results, pointer to ROS2 docs
ros2/
  README.md                                                   rewrite: ROS2 usage
  hoverboard_bridge/
    hoverboard_bridge/link_protocol.py                        modify: SeqLossCounter
    hoverboard_bridge/link_probe.py                           modify: print datagram loss
    hoverboard_bridge/scan_resample.py                        create: fixed-grid LaserScan resampling (pure Python)
    hoverboard_bridge/scan_resampler_node.py                  create: /scan -> /scan_fixed node
    launch/bridge.launch.py                                   modify: publish_tf argument
    config/bridge.yaml                                        modify: calibrated wheel_radius, wheel_separation
    setup.py                                                  modify: scan_resampler entry point
    test/test_link_protocol.py                                modify: SeqLossCounter tests
    test/test_scan_resample.py                                create
    test/test_bridge_node.py                                  create: node against a fake ESP on localhost
  hoverboard_bringup/                                         create (ament_cmake, data only)
    CMakeLists.txt
    package.xml
    urdf/hoverboard.urdf
    config/lidar.yaml          ldlidar_stl_ros2 + scan_resampler
    config/ekf.yaml            robot_localization
    config/slam.yaml           slam_toolbox
    config/nav2.yaml           Nav2
    launch/robot.launch.py     bridge + robot_state_publisher + LIDAR + resampler + EKF
    launch/slam.launch.py      slam_toolbox (mapping)
    launch/nav.launch.py       Nav2, with slam_toolbox or with AMCL on a saved map
    maps/                      saved maps, git-ignored
tools/docker-humble/Dockerfile                                modify: navigation packages
.gitignore                                                    modify: ros2/hoverboard_bringup/maps/
```

---

### Task 1: Robot bring-up and calibration

**Files:**
- Modify: `ros2/hoverboard_bridge/config/bridge.yaml` (calibrated values)

**Interfaces:**
- Consumes: the full robot assembled, ESP at `<ESP>`: USB tethering (`192.168.7.1`) for floor tests, see "Target deployment".

- [x] **Step 1: Command signs (wheels lifted)**

Run (container, terminal 1): `ros2 launch hoverboard_bridge bridge.launch.py esp_host:=<ESP>`
Run (container, terminal 2): `ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"`
Expected: both wheels turn in the robot's forward direction. For each wheel that turns backwards, flip its `*_cmd_sign` in `bridge.yaml` and relaunch (no rebuild needed with `--symlink-install`). The feedback check of 2026-10-18 already gave positive velocities with these signs, so no change is expected.

Result (2026-11-02): both wheels turn forward, no sign changed. Odom over 8 s at 0.1 m/s: x = 0.778 m, y = -0.001 m. `link_probe --left 100 --right 100 --duration 5`: `ticks_per_rev` about 94.6 left and 89.8 right, value 90 kept.

- [x] **Step 2: Measure the UART levels of the LIDAR and the IMU**

With everything powered and idle, measure with a multimeter, against GND, the LIDAR TX on the high-voltage side of the level shifter (HV3), then on the matching low-voltage side (LV3), then the WT901C TX at the ESP (GPIO9, wired directly).
Expected: HV3 about 5V or 3.3V, LV3 and GPIO9 about 3.3V and never above 3.6V. Report the three readings; they go into the "Wiring" section of `esp32s3/README.md` in Task 12. If the LV3 or GPIO9 reading is above 3.6V, stop and disconnect that line: the ESP GPIO is not 5V tolerant.

Result (2026-11-02), multimeter: HV3 about 2.8V, LV3 about 1.9V (LIDAR streaming, so these are averages with the same ratio on both sides), HV4 5V, LV4 3.3V, WT901C TX at most 3.3V. The LV3 peak was not measured (no oscilloscope); the LIDAR runs with 0 CRC errors.

- [x] **Step 3: Wheel radius (robot on the floor)**

Mark the start, then drive straight for about 2 m:
Run (container, terminal 2): `ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"`, stop with Ctrl+C at about 2 m.
Read `ros2 topic echo --once /odom --field pose.pose.position` and measure the real distance with a tape.
Set `wheel_radius_new = wheel_radius_old * real_distance / odom_distance` in `bridge.yaml`, relaunch the bridge (odometry restarts from 0) and repeat once.
Expected: odom within 5% of the real distance.

Result (2026-11-02): with `wheel_radius = 0.0825` the robot moved about 1.6 times faster than commanded (0.974 m odom, 1.56 m real); the wheels are about 27 cm in diameter. New value `wheel_radius = 0.132`, check run 1.536 m odom, 1.50 m real (2.5%). The robot weaves by a few degrees, mostly at start-up; gyro and odometry agree, so the rotations are real. Cause not investigated. The WT901C absolute yaw kept creeping at standstill after the run (about 2.4 deg in 3 s) while the gyro and the odometry stayed still.

- [x] **Step 4: Wheel separation (robot on the floor)**

Mark the robot heading on the floor (tape along the chassis edge). Rotate in place for one full turn:
`ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"`, stop with Ctrl+C when the robot is back on the mark.
Read the yaw with:
```bash
ros2 topic echo --once /odom --field pose.pose.orientation | python3 -c "
import sys, math, yaml
q = yaml.safe_load(sys.stdin.read().split('---')[0])
print(math.degrees(2 * math.atan2(q['z'], q['w'])))"
```
The real rotation is 360 deg; the odom rotation is `360 + printed_angle` (a negative printed angle means odom turned less than a full turn). Restart the bridge before each try so the yaw starts from 0.
Set `wheel_separation_new = wheel_separation_old * odom_angle / real_angle` in `bridge.yaml` and repeat once.
Expected: odom within 5% (18 deg) of the real rotation.

Result (2026-11-02): measured contact point distance 0.585 m, set as `wheel_separation`. One timed turn at 0.5 rad/s: real about 360 deg (no visible angle error, centre moved about 2 cm forward and 1 cm right), odom 367.3 deg (2%), integrated gyro 358.6 deg. The formula gives `wheel_separation = 0.597`, the value now in `bridge.yaml`.

Update the comments of both values in `bridge.yaml`:
```yaml
    wheel_radius: <value>          # [m] calibrated on the floor (2 m straight run)
    wheel_separation: <value>      # [m] calibrated on the floor (one turn in place)
```

- [x] **Step 5: LIDAR driver on the pseudo terminal**

With the bridge running (terminal 1), open a shell in the same container: `docker ps` to get its name, then `docker exec -it <container> bash` and `source /opt/hoverboard/ros_env.sh`. Run:
```bash
ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
  -p product_name:=LDLiDAR_LD19 -p topic_name:=scan -p frame_id:=laser_link \
  -p port_name:=/tmp/ttyLIDAR -p port_baudrate:=230400 -p laser_scan_dir:=true
```
Expected: `ldlidar node start is success`. In a third shell (any container): `ros2 topic hz /scan` gives a steady rate around 7 Hz. If the node does not publish, retry with `product_name:=LDLiDAR_LD06` and report which one works; Task 8 uses it in `config/lidar.yaml`.

Then count the beams per scan (container):
```bash
python3 -c "
import rclpy
from sensor_msgs.msg import LaserScan
rclpy.init()
n = rclpy.create_node('scan_count')
n.create_subscription(LaserScan, 'scan', lambda m: print(len(m.ranges)), 10)
rclpy.spin(n)"
```
Expected: a count that changes from scan to scan (the driver publishes one beam per measured point). This is why Task 7 resamples the scan for `slam_toolbox`. Report the range of counts.

Result (2026-11-02): `product_name:=LDLiDAR_LD19` works, `/scan` at 7.0 Hz (intervals 0.1 to 0.2 s), 617 to 620 beams per scan, 557 to 588 of them valid, ranges 0.29 to 5.39 m with the robot standing still.

---

### Task 2 (dropped): Odometry continuity across a hoverboard reboot (ESP)

Dropped on 2026-11-02, do not execute: all the electronics are powered by the hoverboard, so when it switches off or reboots the rest goes down with it.

When the mainboard reboots while the ESP stays powered, the firmware counters restart from 0 and `hover::TickUnwrapper` takes the jump as a real move. Fix: when hoverboard feedback resumes after a gap of at least `kFreshMs` (500 ms), the next raw value becomes the new reference and the cumulative total is kept.

**Files:**
- Modify: `esp32s3/components/hover_core/include/hover_core/hover_protocol.h`
- Modify: `esp32s3/main/bridge.cpp` (`Bridge::pollHover`)
- Test: `esp32s3/test_host/test_core.cpp`

- [ ] **Step 1: Write the failing test**

In `testHoverProtocol()` of `esp32s3/test_host/test_core.cpp`, after the existing `CHECK(unwrap.update(8995) == 5);`, add:
```cpp
    // reset() keeps the total and takes the next raw value as the new reference
    hover::TickUnwrapper reboot;
    reboot.update(4000);
    reboot.update(4100);
    reboot.reset();
    CHECK(reboot.update(0) == 100);
    CHECK(reboot.update(30) == 130);
```

- [ ] **Step 2: Run to verify it fails**

Run (host): `esp32s3/test_host/run_tests.sh`
Expected: compile error, `TickUnwrapper` has no member named `reset`.

- [ ] **Step 3: Implement**

In `hover_protocol.h`, class `TickUnwrapper`, add below `total()`:
```cpp
    // Next update() becomes the new reference; total() is kept.
    void reset() { init_ = false; }
```

In `esp32s3/main/bridge.cpp`, `Bridge::pollHover`, replace the body of the inner `for` loop:
```cpp
            if (!hoverParser_.push(buf[i])) continue;
            const hover::Feedback& f = hoverParser_.last();
            if (feedbackSeen_ && now - lastFeedbackMs_ >= kFreshMs) {
                // The mainboard may have rebooted and restarted its counters from 0
                ticksL_.reset();
                ticksR_.reset();
                ESP_LOGW(TAG, "Hoverboard feedback resumed, tick reference reset");
            }
            ticksL_.update(f.wheelL_cnt);
            ticksR_.update(f.wheelR_cnt);
            feedbackSeen_ = true;
            lastFeedbackMs_ = now;
```

- [ ] **Step 4: Run tests to verify they pass**

Run (host): `esp32s3/test_host/run_tests.sh`
Expected: `All host tests passed`.

- [ ] **Step 5: Build**

Run (host, from `esp32s3`): `idf.py build`
Expected: build succeeds with no new warnings.

- [ ] **Step 6: Hardware check (ask the user before flashing; wheels lifted)**

Flash as described in `esp32s3/README.md`. Power the ESP from the PC USB port with the step-down output disconnected from the ESP 5V pin, so a hoverboard power cycle does not reboot the ESP. Turn the hoverboard on.
Run (host, from `ros2/hoverboard_bridge`): `python3 -m hoverboard_bridge.link_probe --host <ESP> --left 50 --right 50 --duration 60`
After about 10 s, switch the hoverboard off and on again with its button.
Expected: `[esp] ... Hoverboard feedback resumed, tick reference reset` once the mainboard is back; `ticks L/R` continue from the values before the power cycle instead of jumping by thousands.

---

### Task 3: Link diagnostics (WiFi reason codes, datagram loss in `link_probe`)

**Files:**
- Modify: `esp32s3/main/net_wifi.cpp` (`onWifiEvent`)
- Modify: `ros2/hoverboard_bridge/hoverboard_bridge/link_protocol.py`
- Modify: `ros2/hoverboard_bridge/hoverboard_bridge/link_probe.py`
- Test: `ros2/hoverboard_bridge/test/test_link_protocol.py`

- [x] **Step 1: Write the failing tests**

Append to `ros2/hoverboard_bridge/test/test_link_protocol.py`:
```python
def _count(seqs):
    counter = lp.SeqLossCounter()
    for seq in seqs:
        counter.update(seq)
    return counter


def test_seq_loss_counter_counts_gaps():
    counter = _count([1, 2, 5])
    assert (counter.received, counter.lost) == (3, 2)
    assert counter.loss_ratio() == 2 / 5


def test_seq_loss_counter_handles_wrap_and_duplicates():
    assert _count([65534, 65535, 0, 1]).lost == 0
    assert _count([5, 5, 6]).lost == 0
    assert lp.SeqLossCounter().loss_ratio() == 0.0
```

- [x] **Step 2: Run to verify they fail**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test/test_link_protocol.py -q`
Expected: FAIL with `AttributeError: module 'hoverboard_bridge.link_protocol' has no attribute 'SeqLossCounter'`.

- [x] **Step 3: Implement the counter**

Append to `ros2/hoverboard_bridge/hoverboard_bridge/link_protocol.py`:
```python
class SeqLossCounter:
    """Counts lost ESP->PC datagrams from gaps in the shared 16-bit seq field."""

    def __init__(self):
        self.received = 0
        self.lost = 0
        self._last = None

    def update(self, seq):
        if self._last is not None:
            gap = (seq - self._last - 1) & 0xFFFF
            if gap < 0x8000:  # Larger gaps are duplicates or reordering
                self.lost += gap
        self._last = seq
        self.received += 1

    def loss_ratio(self):
        total = self.received + self.lost
        return self.lost / total if total else 0.0
```

- [x] **Step 4: Run tests to verify they pass**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test -q`
Expected: all tests pass (`20 passed`).

- [x] **Step 5: Print the loss in `link_probe`**

In `ros2/hoverboard_bridge/hoverboard_bridge/link_probe.py`, `main()`:

After `lidar_bytes = 0` add:
```python
    loss = lp.SeqLossCounter()
```
After `if msg is None:` / `continue` in the receive loop add:
```python
            loss.update(msg.seq)
```
After the final `print_ticks_per_rev(...)` call add:
```python
    print(f'datagrams: received={loss.received} lost={loss.lost} ({100 * loss.loss_ratio():.1f}%)')
```

Run (host, from `ros2/hoverboard_bridge`): `python3 -m hoverboard_bridge.link_probe --host 127.0.0.1 --duration 1`
Expected: `no telemetry` lines, then `datagrams: received=0 lost=0 (0.0%)`, no exception.

Result (2026-11-03): the new tests failed with the expected `AttributeError`, then passed. 28 passed in the container (the 20 above predates later tests), 25 passed and 1 skipped module on the host. `link_probe` on `127.0.0.1` prints the expected line. Steps 6 and 7 (ESP firmware) not done yet.

- [ ] **Step 6: Log the WiFi disconnect reason and the signal**

In `esp32s3/main/net_wifi.cpp`, `onWifiEvent`, replace the `WIFI_EVENT_STA_DISCONNECTED` and `IP_EVENT_STA_GOT_IP` branches:
```cpp
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        auto* event = static_cast<wifi_event_sta_disconnected_t*>(data);
        // Reason codes: wifi_err_reason_t in esp_wifi_types.h
        ESP_LOGW(TAG, "Disconnected from %s (reason %d, rssi %d), retrying", CONFIG_HB_WIFI_SSID, event->reason,
                 event->rssi);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(data);
        wifi_ap_record_t ap = {};
        esp_wifi_sta_get_ap_info(&ap);
        ESP_LOGI(TAG, "Station IP " IPSTR ", channel %d, rssi %d", IP2STR(&event->ip_info.ip), ap.primary, ap.rssi);
```

- [ ] **Step 7: Build**

Run (host, from `esp32s3`): `idf.py build`
Expected: build succeeds with no new warnings.

---

### Task 4 (cancelled): WiFi link comparison (hardware)

**Cancelled on 2026-11-03: do not run the steps below.** The user decided that WiFi is not viable on this board without a suitably sized antenna. Tests run before the decision are in `esp32s3/README.md`, "Link results". Rows A-C, E and F and the station reliability test were not run.

Navigation needs a link without bursts of loss: Nav2 stops the robot when `/cmd_vel` or TF data arrive late, and the ESP watchdog stops the wheels after 300 ms without commands. The robot uses USB tethering (see "Target deployment"), so no later task depends on this one. It only measures how usable the WiFi options are for bench work.

**Files:**
- Modify: `esp32s3/README.md` (new section "Link results")

- [ ] **Step 1: Flash the Task 3 firmware (ask the user before flashing)**

Station mode with the usual credentials. Expected: the probe shows `[esp] ... Station IP 192.168.1.50, channel N, rssi R`. The ESP log queue is sent once the PC has sent a command, so the line appears at the start of the first probe run.

- [ ] **Step 2: Measure each configuration**

Hoverboard, IMU and LIDAR connected, wheels still. For each row, run (host, from `ros2/hoverboard_bridge`):
`python3 -m hoverboard_bridge.link_probe --host <ESP> --duration 30 | tail -n 3`
and note the `datagrams:` line, how many status lines show `CMD_TIMEOUT`, and the RSSI. In parallel run `ping -c 30 <ESP>` and note loss and max RTT.

| # | Mode | Position | PC link |
|---|---|---|---|
| A | Station | ESP 1 m from the router | PC on WiFi |
| B | Station | ESP 1 m from the router | PC on Ethernet to the router |
| C | Station | robot where it will drive (farthest room) | PC on Ethernet |
| D | Access point (`HB_WIFI_MODE_AP`, reflash) | PC 1 m from the robot | PC on the ESP network |
| E | Access point | PC at the far end of the driving area | PC on the ESP network |
| F | Best of A-E with AMPDU off (`idf.py menuconfig` -> Component config -> Wi-Fi: disable "WiFi AMPDU TX" and "WiFi AMPDU RX", reflash) | same as that row | same as that row |

Row F targets the repeated `ba-add`/`ba-del` seen in the ESP log: these are block-ack sessions of AMPDU aggregation, which some routers handle poorly.

- [ ] **Step 3: Station mode reliability**

Station mode, best configuration so far. Power cycle the robot 10 times, waiting 20 s each time, and run `python3 -m hoverboard_bridge.link_probe --host <ESP> --duration 3 | grep -E "wifi|Station|telemetry"` after each boot.
Expected: telemetry after every boot. For each failed join, note the `reason` printed by the next successful connection's log backlog, or, with the ESP on the USB serial console (`idf.py monitor`, only with `HB_USB_NET` off and console set back to USB Serial/JTAG in menuconfig), the live disconnect lines.

- [ ] **Step 4: Record the results**

Append to `esp32s3/README.md`:
````markdown
## Link results

Measured with `link_probe --duration 30`, hoverboard, IMU and LIDAR connected, wheels still.

| Configuration | Datagrams lost | CMD_TIMEOUT | RSSI | Ping loss / max RTT |
|---|---|---|---|---|
| USB tethering | 0% | 0 | - | 0% / 2.3 ms |
| <one row per configuration measured in Step 2> | | | | |

Station joins: <successful>/10, disconnect reasons: <codes>.
````

Reference for reading the results: a WiFi configuration is good enough for driving when, at the far end of the driving area, it loses less than 1% of the datagrams and shows no `CMD_TIMEOUT`.

---

### Task 5: Bridge node tests against a fake ESP

**Files:**
- Create: `ros2/hoverboard_bridge/test/test_bridge_node.py`

These tests start the real `BridgeNode` with `esp_host` = `127.0.0.1` and a UDP socket in the test playing the ESP. They need `rclpy`, so they run in the container and are skipped elsewhere.

- [x] **Step 1: Write the tests**

Create `ros2/hoverboard_bridge/test/test_bridge_node.py`:
```python
"""BridgeNode against a fake ESP on localhost (needs rclpy: run in the container)."""
import math
import socket
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

from hoverboard_bridge import link_protocol as lp  # noqa: E402
from hoverboard_bridge.bridge_node import BridgeNode  # noqa: E402

FORWARD_0_5 = (289, 289)  # Default parameters, linear.x = 0.5 m/s (see test_kinematics.py)


@pytest.fixture
def esp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    sock.setblocking(False)
    yield sock
    sock.close()


@pytest.fixture
def bridge(esp):
    rclpy.init(args=['--ros-args', '-p', 'esp_host:=127.0.0.1', '-p', f'esp_port:={esp.getsockname()[1]}',
                     '-p', 'enable_lidar:=false', '-p', 'publish_tf:=false'])
    node = BridgeNode()
    helper = rclpy.create_node('test_helper')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(helper)
    yield helper, executor
    executor.shutdown()
    helper.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


def spin_until(executor, predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return True
    return False


def receive_commands(esp):
    """Returns [((left, right), sender)] for the CMD_WHEELS datagrams waiting on the fake ESP."""
    commands = []
    while True:
        try:
            data, sender = esp.recvfrom(lp.MAX_DATAGRAM)
        except BlockingIOError:
            return commands
        msg = lp.decode(data)
        if msg is not None and msg.type == lp.MSG_CMD_WHEELS:
            commands.append((lp.CMD_WHEELS.unpack(msg.body), sender))


def wheel_state(stamp_ms, ticks_l, ticks_r):
    body = lp.WHEEL_STATE.pack(ticks_l, ticks_r, 0, 0, 4000, 300, lp.STATUS_HOVER_OK, 0, 0)
    return lp.encode(lp.MSG_WHEEL_STATE, 0, stamp_ms, body)


def test_cmd_vel_becomes_wheel_command(bridge, esp):
    helper, executor = bridge
    pub = helper.create_publisher(Twist, 'cmd_vel', 10)
    twist = Twist()
    twist.linear.x = 0.5
    seen = []

    def forward_seen():
        pub.publish(twist)
        seen.extend(cmd for cmd, _ in receive_commands(esp))
        return FORWARD_0_5 in seen

    assert spin_until(executor, forward_seen)


def test_commands_stop_after_cmd_vel_timeout(bridge, esp):
    helper, executor = bridge
    pub = helper.create_publisher(Twist, 'cmd_vel', 10)
    twist = Twist()
    twist.linear.x = 0.5
    pub.publish(twist)
    assert spin_until(executor, lambda: any(cmd == FORWARD_0_5 for cmd, _ in receive_commands(esp)))

    seen = []
    spin_until(executor, lambda: seen.extend(cmd for cmd, _ in receive_commands(esp)) and False, timeout=1.0)
    assert seen and seen[-1] == (0, 0)  # cmd_vel_timeout is 0.5 s


def test_wheel_state_becomes_odometry(bridge, esp):
    helper, executor = bridge
    odoms = []
    helper.create_subscription(Odometry, 'odom', odoms.append, 10)
    senders = []
    assert spin_until(executor, lambda: senders.extend(s for _, s in receive_commands(esp)) or senders)

    esp.sendto(wheel_state(1000, 0, 0), senders[0])
    esp.sendto(wheel_state(1020, 90, 90), senders[0])  # One wheel revolution
    assert spin_until(executor, lambda: odoms and odoms[-1].pose.pose.position.x > 0.0)
    assert odoms[-1].pose.pose.position.x == pytest.approx(2 * math.pi * 0.0825)
```

- [x] **Step 2: Run the tests**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test -q`
Expected: all tests pass (`23 passed`). These tests cover existing behaviour, so they are expected to pass at once; to see that they can fail, temporarily set `'cmd_vel_timeout': 5.0` in the `defaults` of `bridge_node.py`, run again (expected: `test_commands_stop_after_cmd_vel_timeout` fails), then restore `0.5`.

Run (host, from `ros2/hoverboard_bridge`): `python3 -m pytest test -q`
Expected: the bridge node tests are reported as skipped, the others pass.

Result (2026-11-02): 21 passed in the container (18 existing + 3 new, not 23), 18 passed and 1 skipped module on the host. With `cmd_vel_timeout` at 5.0 only `test_commands_stop_after_cmd_vel_timeout` fails. Run these tests with a dedicated `ROS_DOMAIN_ID` (for example `-e ROS_DOMAIN_ID=77`): they publish on `/cmd_vel` and would move the robot if a real bridge shared the domain.

---

### Task 6: Navigation packages in the container

**Files:**
- Modify: `tools/docker-humble/Dockerfile`
- Modify: `tools/docker-humble/README.md`

- [x] **Step 1: Add the packages**

In `tools/docker-humble/Dockerfile`, replace the first `apt-get install` block with:
```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-pytest python3-colcon-common-extensions iproute2 iputils-ping \
        ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
        ros-humble-robot-localization ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*
```

- [x] **Step 2: Rebuild and check**

Run (host): `tools/docker-humble/run.sh --build ros2 pkg list | grep -E "^(nav2_bringup|slam_toolbox|robot_localization|teleop_twist_keyboard)$"`
Expected: the four package names.

- [x] **Step 3: Document**

In `tools/docker-humble/README.md`, replace the line ``- `ldlidar_stl_ros2` is prebuilt in the image.`` with:
```markdown
- `ldlidar_stl_ros2` is prebuilt in the image; Nav2, `slam_toolbox`, `robot_localization` and
  `teleop_twist_keyboard` are installed from the ROS2 apt repository.
```

---

### Task 7: Fixed-grid LaserScan for `slam_toolbox`

`ldlidar_stl_ros2` publishes one beam per measured point, so the beam count changes at every scan (Task 1 Step 5). `slam_toolbox` sizes its laser model on the first scan and drops scans with a different count. `scan_resampler` republishes `/scan` as `/scan_fixed` with a constant number of beams over [-pi, pi), keeping the closest point in each sector.

**Files:**
- Create: `ros2/hoverboard_bridge/hoverboard_bridge/scan_resample.py`
- Create: `ros2/hoverboard_bridge/hoverboard_bridge/scan_resampler_node.py`
- Modify: `ros2/hoverboard_bridge/setup.py`
- Test: `ros2/hoverboard_bridge/test/test_scan_resample.py`

- [x] **Step 1: Write the failing tests**

Create `ros2/hoverboard_bridge/test/test_scan_resample.py`:
```python
import math

from hoverboard_bridge.scan_resample import resample

INF = math.inf


def test_points_land_in_their_sector():
    # Four points a quarter turn apart, four output sectors starting at -pi
    out = resample(0.25, math.pi / 2, [1.0, 2.0, 3.0, 4.0], 0.02, 25.0, beams=4)
    assert out == [3.0, 4.0, 1.0, 2.0]


def test_closest_valid_point_wins_and_invalid_points_are_skipped():
    out = resample(0.1, 0.01, [5.0, 1.5, 0.0, INF, float('nan'), 30.0], 0.02, 25.0, beams=4)
    assert out == [INF, INF, 1.5, INF]


def test_output_size_does_not_depend_on_input_size():
    for count in (600, 640):
        out = resample(0.0, 2 * math.pi / count, [1.0] * count, 0.02, 25.0, beams=360)
        assert len(out) == 360
        assert all(r == 1.0 for r in out)  # Input denser than the output: every sector filled
```

- [x] **Step 2: Run to verify they fail**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test/test_scan_resample.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'hoverboard_bridge.scan_resample'`.

- [x] **Step 3: Implement the resampling**

Create `ros2/hoverboard_bridge/hoverboard_bridge/scan_resample.py`:
```python
"""Resamples a LaserScan with a varying number of beams onto a fixed angular grid."""
import math


def resample(angle_min, angle_increment, ranges, range_min, range_max, beams):
    """Returns `beams` ranges covering [-pi, pi) in steps of 2*pi/beams.

    Each output beam keeps the closest valid input point in its sector; empty sectors are inf.
    """
    out = [math.inf] * beams
    step = 2.0 * math.pi / beams
    for i, r in enumerate(ranges):
        if not range_min <= r <= range_max:  # Also rejects NaN and inf
            continue
        angle = (angle_min + i * angle_increment + math.pi) % (2.0 * math.pi)
        k = int(angle / step) % beams
        out[k] = min(out[k], r)
    return out
```

- [x] **Step 4: Run tests to verify they pass**

Run (container, from `/ws/src/hoverboard_bridge`): `python3 -m pytest test -q`
Expected: all tests pass (`26 passed`).

- [x] **Step 5: Write the node**

Create `ros2/hoverboard_bridge/hoverboard_bridge/scan_resampler_node.py`:
```python
"""Republishes /scan on a fixed angular grid as /scan_fixed (slam_toolbox needs a constant beam count)."""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from hoverboard_bridge.scan_resample import resample


class ScanResampler(Node):

    def __init__(self):
        super().__init__('scan_resampler')
        self.beams = self.declare_parameter('beams', 360).value
        self.pub = self.create_publisher(LaserScan, 'scan_fixed', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)

    def on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_increment = 2.0 * math.pi / self.beams
        out.angle_min = -math.pi
        out.angle_max = math.pi - out.angle_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = resample(msg.angle_min, msg.angle_increment, msg.ranges,
                              msg.range_min, msg.range_max, self.beams)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanResampler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

In `ros2/hoverboard_bridge/setup.py`, extend `console_scripts`:
```python
        'console_scripts': [
            'bridge_node = hoverboard_bridge.bridge_node:main',
            'link_probe = hoverboard_bridge.link_probe:main',
            'scan_resampler = hoverboard_bridge.scan_resampler_node:main',
        ],
```

- [x] **Step 6: Build and smoke test without hardware**

Run (container, from `/ws`): `colcon build --symlink-install --packages-select hoverboard_bridge && source install/setup.bash`
Expected: `Finished <<< hoverboard_bridge`.

Run (container, terminal 1): `ros2 run hoverboard_bridge scan_resampler`
Run (container, terminal 2):
```bash
ros2 topic pub -r 5 /scan sensor_msgs/LaserScan \
  "{header: {frame_id: laser_link}, angle_min: 0.0, angle_increment: 1.0, range_min: 0.02, range_max: 25.0, ranges: [1.0, 2.0, 3.0]}"
```
Run (container, terminal 3):
```bash
python3 -c "
import math, rclpy
from sensor_msgs.msg import LaserScan
rclpy.init()
n = rclpy.create_node('scan_check')
def show(m):
    print(len(m.ranges), sum(math.isfinite(r) for r in m.ranges))
    raise SystemExit
n.create_subscription(LaserScan, 'scan_fixed', show, 10)
rclpy.spin(n)"
```
Expected: `360 3` (360 beams, three of them finite).

Result (2026-11-02): `360 3` as expected. Added beyond this plan: the driver can mask only one angle range, but the wheels occupy two, so `resample()` takes `excluded` sectors and the node has the parameter `excluded_sectors_deg` (flat `[min, max, ...]` in degrees within [-180, 180), LIDAR frame, empty by default). With `[50.0, 60.0]` the smoke test gives `360 2`. 2 extra tests, 26 passed in the container. The robot values `[20.0, 50.0, 130.0, 160.0]` go into `config/lidar.yaml` in Task 8 and are checked on the hardware there.

---

### Task 8: `hoverboard_bringup` package: robot description and robot launch

**Files:**
- Create: `ros2/hoverboard_bringup/CMakeLists.txt`
- Create: `ros2/hoverboard_bringup/package.xml`
- Create: `ros2/hoverboard_bringup/urdf/hoverboard.urdf`
- Create: `ros2/hoverboard_bringup/config/lidar.yaml`
- Create: `ros2/hoverboard_bringup/launch/robot.launch.py`
- Modify: `ros2/hoverboard_bridge/launch/bridge.launch.py`

- [x] **Step 1: Measure the robot**

Measure from `base_link` (floor, midway between the wheel contact points, x forward, y left, z up), in metres:

| Quantity | Used in | Initial value |
|---|---|---|
| Chassis box: length (x), width (y), height of its centre above the floor | URDF `chassis_link` | 0.50, 0.60, 0.10 |
| Footprint: front, rear, left, right extent from `base_link`, including wheels and anything that sticks out | URDF chassis size, `nav2.yaml` `footprint` (Task 11) | 0.25, 0.25, 0.30, 0.30 |
| LIDAR optical centre: x, y, z | URDF `laser_joint` | 0.0, 0.0, 0.20 |
| IMU board centre: x, y, z | URDF `imu_joint` | 0.0, 0.0, 0.10 |

Replace the initial values in the files below with the measured ones. The LIDAR and IMU orientations are checked in Step 6.

Measured (2026-11-02):
- Chassis board, seen from above: length 0.59, width 0.30, centred on y, rear edge at x = -0.12, so front edge at x = 0.47 and centre at x = 0.175. Top surface at z = 0.10. The chassis box is visual only, so its height needs no precision.
- LIDAR: x = 0.225, y = 0, z = 0.21 (mirror 0.11 above the board). Its scan plane passes above the hoverboard body, but the wheels reach into it. Yaw = +90 deg, `rpy="0 0 1.5708"` (LIDAR x points to the robot's left), `laser_scan_dir: true`. Wall test: 1.058 m from the LIDAR against 1.06 m by tape, wall normal at -89.7 deg in the LIDAR frame; a box on the robot's left confirmed the scan is not mirrored. The wheels show up at about 20..50 and 130..160 deg in the LIDAR frame, minimum range 0.29 m.
- IMU: x = 0.10, y = 0, z = 0.11. Board axes: x forward, y left.
- Wheels: `wheel_separation` 0.597 in `bridge.yaml`, 0.66 across the outer sides, diameter about 0.27. In the URDF use radius 0.132 and joints at y = +-0.2985.
- A caster sits under the front of the board, on the centre line.
- Footprint: front 0.47, rear 0.135 (tyre radius, slightly more than the 0.12 of the board), left 0.33, right 0.33.

- [x] **Step 2: Package files**

Create `ros2/hoverboard_bringup/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(hoverboard_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()
```

Create `ros2/hoverboard_bringup/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>hoverboard_bringup</name>
  <version>0.1.0</version>
  <description>Robot description, launch files and navigation configuration for the hoverboard robot</description>
  <maintainer email="lorenzodauria97@gmail.com">Lorenzo D'Auria</maintainer>
  <author email="lorenzodauria97@gmail.com">Lorenzo D'Auria</author>
  <license>GPL-3.0-only</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>hoverboard_bridge</exec_depend>
  <exec_depend>ldlidar_stl_ros2</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>robot_localization</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [x] **Step 3: URDF**

Create `ros2/hoverboard_bringup/urdf/hoverboard.urdf` (wheel radius and separation must match `bridge.yaml`):
```xml
<?xml version="1.0"?>
<!-- base_link: on the floor, midway between the wheel contact points, x forward, z up. -->
<robot name="hoverboard_robot">
  <link name="base_link"/>

  <link name="chassis_link">
    <visual>
      <geometry><box size="0.50 0.60 0.10"/></geometry>
    </visual>
  </link>
  <joint name="chassis_joint" type="fixed">
    <parent link="base_link"/>
    <child link="chassis_link"/>
    <origin xyz="0 0 0.10"/>
  </joint>

  <link name="left_wheel_link">
    <visual>
      <origin rpy="1.5708 0 0"/>
      <geometry><cylinder radius="0.0825" length="0.05"/></geometry>
    </visual>
  </link>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0 0.20 0.0825"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="right_wheel_link">
    <visual>
      <origin rpy="1.5708 0 0"/>
      <geometry><cylinder radius="0.0825" length="0.05"/></geometry>
    </visual>
  </link>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0 -0.20 0.0825"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="laser_link"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0 0 0.20" rpy="0 0 0"/>
  </joint>

  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>
  </joint>
</robot>
```

- [x] **Step 4: LIDAR configuration and launch files**

Create `ros2/hoverboard_bringup/config/lidar.yaml` (use the `product_name` that worked in Task 1 Step 5):
```yaml
ldlidar:
  ros__parameters:
    product_name: "LDLiDAR_LD19"
    topic_name: "scan"
    frame_id: "laser_link"
    port_name: "/tmp/ttyLIDAR"     # Pseudo terminal created by hoverboard_bridge
    port_baudrate: 230400
    laser_scan_dir: true           # Counterclockwise, as ROS expects
    enable_angle_crop_func: false  # Enable to mask parts of the robot seen by the LIDAR
    angle_crop_min: 135.0
    angle_crop_max: 225.0

scan_resampler:
  ros__parameters:
    beams: 360
```

In `ros2/hoverboard_bridge/launch/bridge.launch.py`, add a launch argument after the `esp_host` one:
```python
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Publish odom -> base_link (false when an EKF does it)'),
```
and change the node parameters to:
```python
            parameters=[params, {'esp_host': LaunchConfiguration('esp_host'),
                                 'publish_tf': LaunchConfiguration('publish_tf')}],
```

Create `ros2/hoverboard_bringup/launch/robot.launch.py`:
```python
"""Everything that runs next to the ESP link: bridge, robot description and LIDAR driver."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('hoverboard_bringup')
    bridge_launch = os.path.join(get_package_share_directory('hoverboard_bridge'), 'launch', 'bridge.launch.py')
    lidar_params = os.path.join(bringup, 'config', 'lidar.yaml')
    with open(os.path.join(bringup, 'urdf', 'hoverboard.urdf')) as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('esp_host', default_value='192.168.1.50',
                              description='WiFi STA static IP (192.168.1.50), WiFi AP 192.168.4.1, USB 192.168.7.1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={'esp_host': LaunchConfiguration('esp_host')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # Exits while the bridge has not created the pseudo terminal yet, so it is respawned
        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node', name='ldlidar',
             parameters=[lidar_params], respawn=True, respawn_delay=2.0),
        Node(package='hoverboard_bridge', executable='scan_resampler', parameters=[lidar_params]),
    ])
```

- [x] **Step 5: Build and run without hardware**

Run (container, from `/ws`): `colcon build --symlink-install && source install/setup.bash`
Expected: `Finished <<< hoverboard_bridge` and `Finished <<< hoverboard_bringup`.

Run (container): `ros2 launch hoverboard_bringup robot.launch.py esp_host:=127.0.0.1`
Expected: `Bridging to ESP at 127.0.0.1`, `robot_state_publisher` lists the links, the LIDAR node starts (it waits for data), no exception. In a second container shell: `ros2 run tf2_ros tf2_echo base_link laser_link` prints the URDF offset.

Result (2026-11-02): both packages build, all four nodes run, `tf2_echo` gives the measured offsets (`laser_link` 0.225, 0, 0.21, yaw 90 deg). The URDF and `lidar.yaml` use the Step 1 values, plus `excluded_sectors_deg` for the wheels. Without data the LIDAR node does not wait: it exits about every 5 s with `communication is abnormal` and is respawned. This is harmless with no ESP connected.

- [x] **Step 6: Hardware check: TF tree, LIDAR and IMU orientation (robot on the floor)**

Run (container): `ros2 launch hoverboard_bringup robot.launch.py esp_host:=<ESP>`
Run (container, second shell): `rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz`, set Fixed Frame to `odom`, set the LaserScan display topic to `/scan_fixed`.
Checks:
1. `ros2 run tf2_tools view_frames` (container, writes `frames.pdf`): `odom -> base_link -> {chassis_link, laser_link, imu_link, left_wheel_link, right_wheel_link}`.
2. Place a box about 1 m straight in front of the robot. Expected: the scan shows it on the +x axis of `base_link`. If it appears elsewhere, set the yaw of `laser_joint` (`rpy="0 0 <yaw>"`, radians) to rotate it onto +x; if it appears mirrored (left and right swapped), set `laser_scan_dir: false`. Relaunch and repeat.
3. Points that fall on the robot itself (a ring very close to the LIDAR, or fixed points on chassis parts): mask their angle range with `excluded_sectors_deg` of `scan_resampler` in `config/lidar.yaml` (degrees, in the LIDAR's own frame). The driver crop allows only one range, and the wheels need two.
4. Rotate the robot counterclockwise seen from above (`ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"`) and watch `ros2 topic echo /imu/data --field angular_velocity`. Expected: `z` positive. With the robot still, `linear_acceleration.z` is about +9.8. If not, set the `imu_joint` `rpy` so the IMU axes map onto `base_link` (for example `rpy="3.1416 0 0"` for a board mounted upside down), relaunch and repeat.

Result (2026-11-03), USB tethering: no change needed. TF tree as expected, `odom -> base_link` at 50 Hz. `/scan` and `/scan_fixed` at 7.0 Hz, 360 beams, about 300 valid. Box about 1 m ahead seen at x = 0.97, y = 0.00, bearing -8.5..+8.5 deg, and the RViz view is correct; left/right not repeated, confirmed in Step 1 with the same configuration. `excluded_sectors_deg` covers the wheels (raw points at about 24..48 and 132..155 deg, 0.30 to 0.39 m), closest point in `/scan_fixed` 0.74 m. Standing still: acceleration -0.17, -0.19, +9.81 m/s^2 (tilt about 1.5 deg). 6 s at 0.5 rad/s counterclockwise: gyro z mean +0.516 rad/s (0.39 to 0.71), gyro x and y about 0, integrated gyro 175.1 deg, odom 176.0 deg, real just under 180 deg by eye, centre moved about 2 cm.

---

### Task 9: Fused odometry with `robot_localization`

The wheels slip when the robot turns in place, so the wheel yaw rate is the weakest part of the odometry. The EKF fuses the wheel twist (vx, vy = 0, vyaw) with the IMU gyro z rate and publishes `odom -> base_link`. The IMU orientation is not fused: yaw is not absolute (see `IMU_YAW_VARIANCE` in `bridge_node.py`).

**Files:**
- Create: `ros2/hoverboard_bringup/config/ekf.yaml`
- Modify: `ros2/hoverboard_bringup/launch/robot.launch.py`

- [x] **Step 1: IMU rate**

Run (container): `ros2 launch hoverboard_bringup robot.launch.py esp_host:=<ESP>`, then in a second shell `ros2 topic hz /imu/data`.
Expected: about 50 Hz, the rate saved in the WT901C on 2026-11-02 (measured through the ESP, not yet on `/imu/data`). If it is lower, set it again with a USB-serial adapter at 3.3V on the IMU, with the IMU disconnected from GPIO9/10 (the ESP does not forward IMU commands). At 115200 baud send `FF AA 69 88 B5` (unlock), `FF AA 03 08 00` (50 Hz), `FF AA 00 00 00` (save), then repeat. Report the rate.

Result (2026-11-03): 50.2 Hz on `/imu/data`, counted over 8 s with a subscriber instead of `ros2 topic hz`.

- [x] **Step 2: EKF configuration**

Create `ros2/hoverboard_bringup/config/ekf.yaml`:
```yaml
# Fuses wheel twist and IMU yaw rate; publishes odom -> base_link and /odometry/filtered.
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # Order: x y z, roll pitch yaw, vx vy vz, vroll vpitch vyaw, ax ay az
    odom0: odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]

    imu0: imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
```

- [x] **Step 3: Launch the EKF**

Replace `ros2/hoverboard_bringup/launch/robot.launch.py` with:
```python
"""Everything that runs next to the ESP link: bridge, robot description, LIDAR driver and EKF."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('hoverboard_bringup')
    bridge_launch = os.path.join(get_package_share_directory('hoverboard_bridge'), 'launch', 'bridge.launch.py')
    lidar_params = os.path.join(bringup, 'config', 'lidar.yaml')
    with open(os.path.join(bringup, 'urdf', 'hoverboard.urdf')) as f:
        robot_description = f.read()
    use_ekf = LaunchConfiguration('use_ekf')

    return LaunchDescription([
        DeclareLaunchArgument('esp_host', default_value='192.168.1.50',
                              description='WiFi STA static IP (192.168.1.50), WiFi AP 192.168.4.1, USB 192.168.7.1'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='Fuse wheel odometry and IMU yaw rate; false uses wheel odometry only'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={
                'esp_host': LaunchConfiguration('esp_host'),
                # Only one node may publish odom -> base_link
                'publish_tf': PythonExpression(["'false' if '", use_ekf, "' == 'true' else 'true'"]),
            }.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # Exits while the bridge has not created the pseudo terminal yet, so it is respawned
        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node', name='ldlidar',
             parameters=[lidar_params], respawn=True, respawn_delay=2.0),
        Node(package='hoverboard_bridge', executable='scan_resampler', parameters=[lidar_params]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[os.path.join(bringup, 'config', 'ekf.yaml')], condition=IfCondition(use_ekf)),
    ])
```

- [x] **Step 4: Build and run without hardware**

Run (container, from `/ws`): `colcon build --symlink-install && source install/setup.bash`
Run (container): `ros2 launch hoverboard_bringup robot.launch.py esp_host:=127.0.0.1`
Expected: `ekf_filter_node` starts with no parameter errors. `ros2 param get /hoverboard_bridge publish_tf` gives `False`. With `use_ekf:=false`: no EKF node and `publish_tf` is `True`.

Result (2026-11-03): as expected in both cases.

- [x] **Step 5: Compare wheel and fused odometry (robot on the floor)**

Run (container): `ros2 launch hoverboard_bringup robot.launch.py esp_host:=<ESP>`
Mark the heading on the floor, rotate one full turn in place as in Task 1 Step 4, and read both yaws:
```bash
for topic in /odom /odometry/filtered; do
  ros2 topic echo --once $topic --field pose.pose.orientation | python3 -c "
import sys, math, yaml
q = yaml.safe_load(sys.stdin.read().split('---')[0])
print('$topic', math.degrees(2 * math.atan2(q['z'], q['w'])))"
done
```
Repeat three times (relaunch in between) and once with two turns.
Expected: the `/odometry/filtered` yaw error is not larger than the `/odom` error. Report both. If the filtered yaw is worse, check the sign of the gyro (Task 8 Step 6, check 4) and the IMU rate before going on; navigation can also run with `use_ekf:=false`.

Result (2026-11-03), USB tethering, timed turns at 0.5 rad/s, yaw change per run instead of a relaunch. The real angle comes from the distance between the board front edge (0.47 m from `base_link`) and the tape, so 1 cm is about 1.2 deg and this is the resolution of the comparison.

| Run | Real (deg) | `/odom` (deg) | Error | `/odometry/filtered` (deg) | Error |
|---|---|---|---|---|---|
| 1 turn | 362.4 | 362.6 | +0.2 | 361.4 | -1.0 |
| 1 turn | 364.9 | 367.9 | +3.0 | 365.6 | +0.7 |
| 1 turn | 360 | 360.8 | +0.8 | 359.4 | -0.6 |
| 2 turns | 729.7 | 730.5 | +0.8 | 728.3 | -1.4 |

Mean absolute error: 1.2 deg `/odom`, 0.9 deg filtered. The filtered error is larger in 2 of the 4 runs, always by less than the measurement resolution, so the two are equivalent on this floor. The filtered yaw is 1.2 to 2.3 deg lower than `/odom` in every run. `use_ekf` stays `true`.

---

### Task 10: Mapping with `slam_toolbox`

**Files:**
- Create: `ros2/hoverboard_bringup/config/slam.yaml`
- Create: `ros2/hoverboard_bringup/launch/slam.launch.py`
- Modify: `.gitignore`

Precondition: the robot linked by USB tethering (see "Target deployment").

- [x] **Step 1: Configuration**

Create `ros2/hoverboard_bringup/config/slam.yaml` (the `slam_toolbox` online async defaults, with the frames, the scan topic, the LIDAR range and the update thresholds adapted to this robot):
```yaml
slam_toolbox:
  ros__parameters:
    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan_fixed           # Constant beam count, see scan_resampler
    use_map_saver: true
    mode: mapping

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 2.0
    resolution: 0.05
    min_laser_range: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.2
    transform_timeout: 0.5            # Scans and odometry arrive over the ESP link
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    # General parameters
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Correlation parameters, loop closure
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan matcher parameters
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
    min_pass_through: 2
    occupancy_threshold: 0.1
```

- [x] **Step 2: Launch file**

Create `ros2/hoverboard_bringup/launch/slam.launch.py`:
```python
"""slam_toolbox in online async mapping mode."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('hoverboard_bringup'), 'config', 'slam.yaml')
    return LaunchDescription([
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen', parameters=[params, {'use_sim_time': False}]),
    ])
```

Append to `.gitignore`:
```
# Maps saved by slam_toolbox / map_saver_cli (environment specific)
ros2/hoverboard_bringup/maps/
```

- [x] **Step 3: Build and check without hardware**

Run (container, from `/ws`): `colcon build --symlink-install && source install/setup.bash`
Run (container): `ros2 launch hoverboard_bringup slam.launch.py`
Expected: `slam_toolbox` starts and waits for TF and scans, with no parameter errors.

Result (2026-11-03): builds, `slam_toolbox` starts with the Ceres solver and no parameter errors. Run for 8 s only, without scans or TF.

- [x] **Step 4: Map a room (robot on the floor)**

Run (container, terminal 1): `ros2 launch hoverboard_bringup robot.launch.py esp_host:=<ESP>`
Run (container, terminal 2): `ros2 launch hoverboard_bringup slam.launch.py`
Run (container, terminal 3): `rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz` (Fixed Frame `map`)
Run (container, terminal 4): `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6`
Drive slowly around the room and back to the start; turn slowly (fast turns blur the scans at 7 Hz).
Expected: walls are straight and single (not doubled), the map closes when returning to the start. No `LaserRangeScan contains ... range readings, expected ...` errors in terminal 2.

Result (2026-11-03), USB tethering: driven by keyboard through the whole home and back to the start. Map 8.50 x 11.25 m at 0.05 m, straight and single walls, some 2 to 3 cells thick; the user checked it against the home. No errors or warnings from `slam_toolbox`, the bridge or the EKF. At the start point the robot is 4.8 cm, 4.2 cm and -2.3 deg from the map origin; the fused odometry alone reads about 29 cm and 6 deg, and `map -> odom` is about 34 cm and -8.4 deg. The log does not show whether a loop closure fired.

- [x] **Step 5: Save the map**

Run (container): `mkdir -p /ws/src/hoverboard_bringup/maps && ros2 run nav2_map_server map_saver_cli -f /ws/src/hoverboard_bringup/maps/<name>`
Expected: `/ws/src/hoverboard_bringup/maps/<name>.yaml` and `.pgm` (repo folder `ros2/hoverboard_bringup/maps/`). Open the `.pgm` and check that it matches the room.

Result (2026-11-03): saved as `home_map`, confirmed by the user.

---

### Task 11: Navigation with Nav2

**Files:**
- Create: `ros2/hoverboard_bringup/config/nav2.yaml`
- Create: `ros2/hoverboard_bringup/launch/nav.launch.py`

`nav2_bringup/navigation_launch.py` starts the controller, planner, smoother, behaviours, BT navigator, waypoint follower and velocity smoother; the controller publishes `/cmd_vel_nav` and the velocity smoother turns it into `/cmd_vel` for the bridge. Localization is either `slam_toolbox` (map built while navigating) or AMCL on a map saved in Task 10.

- [x] **Step 1: Nav2 parameters**

Create `ros2/hoverboard_bringup/config/nav2.yaml` (set `footprint` from Task 8 Step 1 as `[[front, left], [front, -right], [-rear, -right], [-rear, left]]`):
```yaml
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 12.0
    laser_min_range: 0.05
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan_fixed

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_assisted_teleop_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node
      - nav2_is_battery_charging_condition_bt_node

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.3
      movement_time_allowance: 10.0
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      stateful: true
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.25
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.3
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 0.8
      transform_tolerance: 0.3
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.1
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 2.0
      allow_reversing: false
      max_robot_pose_search_dist: 10.0
      use_interpolation: false

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      transform_tolerance: 0.3
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      footprint: "[[0.25, 0.30], [0.25, -0.30], [-0.25, -0.30], [-0.25, 0.30]]"
      footprint_padding: 0.02
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan_fixed
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 4.0
          raytrace_min_range: 0.0
          obstacle_max_range: 3.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.45
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      transform_tolerance: 0.3
      resolution: 0.05
      track_unknown_space: true
      footprint: "[[0.25, 0.30], [0.25, -0.30], [-0.25, -0.30], [-0.25, 0.30]]"
      footprint_padding: 0.02
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan_fixed
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 4.0
          raytrace_min_range: 0.0
          obstacle_max_range: 3.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.45
      always_send_full_costmap: true

map_server:
  ros__parameters:
    yaml_filename: ""   # Set by the map launch argument

map_saver:
  ros__parameters:
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: true

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: true

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.3
    simulate_ahead_time: 2.0
    max_rotational_vel: 0.8
    min_rotational_vel: 0.3
    rotational_acc_lim: 2.0

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.3, 0.0, 1.0]
    min_velocity: [-0.3, 0.0, -1.0]
    max_accel: [0.5, 0.0, 1.5]
    max_decel: [-0.8, 0.0, -2.0]
    odom_topic: "odom"
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0
```

- [x] **Step 2: Launch file**

Create `ros2/hoverboard_bringup/launch/nav.launch.py`:
```python
"""Nav2 with slam_toolbox (no map argument) or with AMCL on a saved map."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup = get_package_share_directory('hoverboard_bringup')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    map_file = LaunchConfiguration('map')
    slam_mode = PythonExpression(["'", map_file, "' == ''"])
    common = {
        'params_file': os.path.join(bringup, 'config', 'nav2.yaml'),
        'use_sim_time': 'false',
        'autostart': 'true',
    }

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='',
                              description='Map yaml for AMCL localization; empty runs slam_toolbox instead'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'slam.launch.py')),
            condition=IfCondition(slam_mode)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'localization_launch.py')),
            launch_arguments={**common, 'map': map_file}.items(),
            condition=UnlessCondition(slam_mode)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'navigation_launch.py')),
            launch_arguments=common.items()),
    ])
```

- [x] **Step 3: Build and check without hardware**

Run (container, from `/ws`): `colcon build --symlink-install && source install/setup.bash`
Run (container): `ros2 launch hoverboard_bringup nav.launch.py`
Expected: all Nav2 servers are created and configured with no parameter errors (they wait for TF, so activation stops at `Waiting for transform` messages). Stop it. Run `ros2 launch hoverboard_bringup nav.launch.py map:=/ws/src/hoverboard_bringup/maps/<name>.yaml`: `map_server` loads the map with no error.

Result (2026-11-03): with no map and with `home_map`, all processes start, no `WARN`, `ERROR` or dead process in 18 s, only the expected waits for TF. `map_server` reads `home_map`, 170 x 225 cells at 0.05 m. The `footprint` in `nav2.yaml` uses the Task 8 Step 1 values: front 0.47, rear 0.135, left and right 0.33.

- [x] **Step 4: Low speed response (robot on the floor)**

Nav2 slows down near goals and on curves, so the motors must follow small commands.
Run (container, terminal 1): `ros2 launch hoverboard_bringup robot.launch.py esp_host:=<ESP>`
For `x` in 0.05, 0.1 and 0.2, run for a few seconds `ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: <x>}}"` and read `ros2 topic echo /odom --field twist.twist.linear.x`.
Expected: the robot moves and the measured speed is within about 20% of the command. Report the lowest speed that moves the robot smoothly; if it is above 0.05, raise `min_approach_linear_velocity` and `regulated_linear_scaling_min_speed` in `nav2.yaml` to that value.

Result (2026-11-03): measured mean speed 0.051 at 0.05 m/s, 0.094 at 0.1 m/s, 0.198 at 0.2 m/s; distance from the ticks within 3 cm of the command. `vx` comes from the integer rpm of the mainboard, so it swings widely (0 to 0.117 m/s at 0.05 m/s, 27% of the samples at zero). The motion at 0.05 m/s is not smooth by eye. The user chose 0.2 m/s as the minimum, so both values are 0.2 in `nav2.yaml`.

- [x] **Step 5: Navigate while mapping (robot on the floor)**

Run (container, terminal 1): `ros2 launch hoverboard_bringup robot.launch.py esp_host:=<ESP>`
Run (container, terminal 2): `ros2 launch hoverboard_bringup nav.launch.py`
Run (container, terminal 3): `rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz`
Send a goal 2 m away with "Nav2 Goal" in RViz, then one behind an obstacle.
Expected: a path appears, the robot follows it at up to 0.3 m/s, stops within 0.15 m of the goal, and goes around the obstacle. Pull the USB cable or stop terminal 1 during a run once: the robot stops within 1 s (bridge, ESP and mainboard watchdogs).

Result (2026-11-03), several goals sent by the user from RViz, no contact with anything: 5 goal requests, 2 `Goal succeeded` (one ended 1.2 cm from the target by the map pose), 2 replaced by the user before arrival because the map showed a wall there, 1 `Goal failed`. In the failed one the robot stopped about 6 cm from the target, with the board front edge about 6 cm from a wall, and spin and backup recoveries failed; the final heading could not be reached there. The log has 244 `detected collision ahead`; how many came from the USB cable passing in front of the LIDAR and how many from walls is unknown (no bag recorded). Stop test: the USB cable came off by accident during the saved map session, the robot stopped at once by eye (not timed) and the mainboard gave 3 beeps (serial timeout). The ESP is probably powered by that cable, so its 300 ms watchdog is still untested on the floor.

- [x] **Step 6: Navigate on a saved map (robot on the floor)**

Run (container, terminal 2): `ros2 launch hoverboard_bringup nav.launch.py map:=/ws/src/hoverboard_bringup/maps/<name>.yaml`
In RViz set the robot pose with "2D Pose Estimate", drive a little with teleop until the AMCL particle cloud converges, then send goals.
Expected: the scan stays aligned with the map walls while moving; goals reached as in Step 5.

Result (2026-11-03), `home_map`: initial pose published at the map origin with the robot at the start point, so no teleop was needed for convergence. 97% of the scan points within 15 cm of a map wall at the start, 94% after the first goal. First goal reached 3.9 cm from the target by the AMCL pose, second goal reached around the box without contact. One start-up stalled with `map_server` at `Configuring` (`failed to send response to /map_server/change_state`); a relaunch of Nav2 cleared it, cause not investigated.

- [ ] **Step 7: Tuning record**

Write the values changed while testing (footprint, speeds, inflation, AMCL, `slam_toolbox`) and why into the comments of `nav2.yaml` and `slam.yaml`, one short comment per changed value.

---

### Task 12: Documentation

**Files:**
- Rewrite: `ros2/README.md`
- Modify: `esp32s3/README.md`

Result (2026-11-03): done, extended on request with `scripts/` and the `stack_mapping` and `stack_navigation` launch files. Checked without the robot (ESP address `127.0.0.1`): both stacks start, `save_map.sh` saves the map served by `map_server`, Ctrl-C stops everything. The two whole-stack launches have not run on the robot yet.

- [x] **Step 1: ROS2 README**

Replace `ros2/README.md` with:
````markdown
# ROS2

ROS2 Humble packages for the hoverboard robot. They run in the container from `tools/docker-humble`
(see its README); build from `/ws` with `colcon build --symlink-install`.

| Package | Content |
|---|---|
| `hoverboard_bridge` | UDP bridge to the ESP32-S3 (`bridge_node`), `link_probe` bench tool, `scan_resampler` |
| `hoverboard_bringup` | URDF, LIDAR/EKF/SLAM/Nav2 configuration and launch files |

## Robot
```
ros2 launch hoverboard_bringup robot.launch.py esp_host:=<ESP address>
```
Starts the bridge, `robot_state_publisher`, the LIDAR driver on `/tmp/ttyLIDAR`, `scan_resampler` and the EKF
(`use_ekf:=false` for wheel odometry only). The bridge alone: `ros2 launch hoverboard_bridge bridge.launch.py`.
ESP addresses: WiFi station `192.168.1.50` (default static IP), WiFi access point `192.168.4.1`, USB `192.168.7.1`.
Link used on the robot: USB tethering to the on-board smartphone. WiFi is in the firmware but not usable on this board (see `esp32s3/README.md`, "Link results").

Topics: `/cmd_vel` (in); `/odom`, `/odometry/filtered`, `/tf`, `/joint_states`, `/imu/data`, `/scan`, `/scan_fixed`,
`/hoverboard/battery`, `/hoverboard/temperature` (out).
Calibrated wheel parameters: `hoverboard_bridge/config/bridge.yaml`. Robot geometry: `hoverboard_bringup/urdf/hoverboard.urdf`
(wheel radius and separation must match `bridge.yaml`; the footprint in `config/nav2.yaml` must match the chassis).

## Mapping
```
ros2 launch hoverboard_bringup slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6
ros2 run nav2_map_server map_saver_cli -f /ws/src/hoverboard_bringup/maps/<name>
```
Maps are saved in `ros2/hoverboard_bringup/maps/` (git-ignored).

## Navigation
```
ros2 launch hoverboard_bringup nav.launch.py                               # with slam_toolbox
ros2 launch hoverboard_bringup nav.launch.py map:=/ws/src/hoverboard_bringup/maps/<name>.yaml # with AMCL
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```
With AMCL, set the start pose with "2D Pose Estimate" before sending goals.

## Tests
```
cd /ws/src/hoverboard_bridge && python3 -m pytest test -q
```
````

- [x] **Step 2: ESP README**

In `esp32s3/README.md`, add the UART level readings of Task 1 Step 2 to the wiring section, and append:
```markdown
## ROS2

ROS2 usage (bridge, LIDAR driver, mapping, navigation): `ros2/README.md`.
```

---

## Future Work (out of scope)

- Runtime WiFi configuration stored in NVS (change SSID and password without reflashing).
- ESP-side timestamps mapped to ROS time for LIDAR and IMU data.
- `ros2_control` hardware interface replacing the Python node (`diff_drive_controller`).
- Use of the LIDAR TIME sync pin.
- Magnetometer: the WT901C sends it (packet `0x54`) and the ESP ignores it. Evaluate only if the heading still drifts after Task 9: record the field while turning, with motors off and on (hub motor magnets are the expected problem). Using it needs changes in the firmware, the protocol and the bridge.
- Battery-aware behaviour (low battery warning, docking).
