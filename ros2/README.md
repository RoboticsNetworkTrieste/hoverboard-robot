# ROS2

ROS2 Humble packages for the hoverboard robot. Everything runs in the Docker container from
[`tools/docker-humble`](../tools/docker-humble/README.md), so ROS2 is not needed on the host.

| Package | Content |
|---|---|
| `hoverboard_bridge` | UDP bridge to the ESP32-S3 (`bridge_node`), `link_probe` bench tool, `scan_resampler` |
| `hoverboard_bringup` | URDF, LIDAR/EKF/SLAM/Nav2 configuration and launch files |

## Before the first run
1. Flash the mainboard and the ESP32-S3, and wire the robot (see the [main README](../README.md)).
2. Build the container image, once: `tools/docker-humble/run.sh --build`.
3. Connect the PC to the ESP by USB: disconnect the step-down output from the ESP 5V pin first, then plug in the USB-C
   cable and switch the hoverboard on. Check the link with `ping 192.168.7.1`.

All commands below run on the host, from the repo root. The start scripts build the workspace by themselves.
They use USB tethering by default; to use another ESP address, add it as the last argument.
While the robot moves, keep a hand near the hoverboard power button.

## Build a new map
```
scripts/start_mapping.sh          # terminal 1: whole stack and RViz
scripts/teleop.sh                 # terminal 2: keyboard driving
```
Drive slowly around the area and back to the start, with slow turns: the LIDAR runs at only 7 Hz.
Keep the keyboard focus on the teleop terminal. The robot can also move by itself: "Nav2 Goal" in RViz sends a goal on
the map built so far. Do not use the keyboard and a goal at the same time.

When walls in RViz are straight and not doubled, save the map **before stopping the stack**:
```
scripts/save_map.sh my_map        # terminal 3
```
This writes `my_map.yaml` and `my_map.pgm` in `ros2/hoverboard_bringup/maps/`. The folder is git-ignored, because maps
depend on the environment. An existing map is never overwritten. Then press Ctrl-C in terminals 1 and 2.

## Navigate on a saved map
```
scripts/start_navigation.sh my_map
```
In RViz:
1. Click "2D Pose Estimate" and drag from the real robot position towards its heading.
   The point where mapping started is the map origin. Check that the scan lines up with the map walls.
2. Send goals with "Nav2 Goal". Maximum speed is 0.3 m/s.

If the scan does not line up, drive a little with `scripts/teleop.sh` until the AMCL particle cloud converges.

## What the scripts run
| Script | Runs |
|---|---|
| `scripts/start_mapping.sh [esp_host]` | `ros2 launch hoverboard_bringup stack_mapping.launch.py` |
| `scripts/start_navigation.sh <map name> [esp_host]` | `ros2 launch hoverboard_bringup stack_navigation.launch.py map:=<map yaml>` |
| `scripts/save_map.sh <map name>` | `ros2 run nav2_map_server map_saver_cli -f <path>` |
| `scripts/teleop.sh` | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |

RViz starts when a display is available; `RVIZ=false scripts/start_mapping.sh` disables it.
The launch files of `hoverboard_bringup`, which can also be run from a container shell (`tools/docker-humble/run.sh`):

| Launch file | Starts | Arguments |
|---|---|---|
| `stack_mapping.launch.py` | `robot` + `nav` with `slam_toolbox` + RViz | `esp_host`, `rviz` |
| `stack_navigation.launch.py` | `robot` + `nav` with AMCL + RViz | `map` (required), `esp_host`, `rviz` |
| `robot.launch.py` | Bridge, `robot_state_publisher`, LIDAR driver, `scan_resampler`, EKF | `esp_host` (give it, the default is the WiFi address), `use_ekf` |
| `nav.launch.py` | Nav2, with `slam_toolbox` or with AMCL when `map` is set | `map` |
| `slam.launch.py` | `slam_toolbox` only | |

ESP addresses: USB `192.168.7.1`. WiFi station `192.168.1.50` and access point `192.168.4.1` exist in the firmware, but
WiFi is not usable on this board (see "Link results" in [`esp32s3/README.md`](../esp32s3/README.md)).
The LIDAR driver must run in the same container as the bridge, because it reads the pseudo terminal `/tmp/ttyLIDAR`
created by the bridge. The launch files above already do this.

## Topics and frames
In: `/cmd_vel`. Out: `/odom`, `/odometry/filtered`, `/tf`, `/joint_states`, `/imu/data`, `/scan`, `/scan_fixed`,
`/hoverboard/battery`, `/hoverboard/temperature`.
```
map -> odom                 slam_toolbox (mapping) or AMCL (saved map)
odom -> base_link           EKF, or the bridge with use_ekf:=false
base_link -> chassis_link, laser_link, imu_link, left_wheel_link, right_wheel_link     robot_state_publisher
```
`base_link` lies on the floor, midway between the wheel contact points: x forward, y left, z up.

## Calibration
The values in the repo fit this robot. For a different one, edit these files and restart the stack; no rebuild is needed,
because the workspace uses `--symlink-install`.

| Value | File |
|---|---|
| Wheel radius and separation, signs, `ticks_per_rev` | `hoverboard_bridge/config/bridge.yaml` |
| Robot geometry (wheels must match `bridge.yaml`) | `hoverboard_bringup/urdf/hoverboard.urdf` |
| Footprint | `hoverboard_bringup/config/nav2.yaml` |
| Angles of the wheels seen by the LIDAR (`excluded_sectors_deg`) | `hoverboard_bringup/config/lidar.yaml` |

Start the bridge alone in a container shell, after `colcon build --symlink-install` from `/ws`:
`ros2 launch hoverboard_bridge bridge.launch.py esp_host:=192.168.7.1`. Then send commands from a second shell:
1. Signs, wheels lifted: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"`. Both wheels must
   turn forward; otherwise flip that wheel's `*_cmd_sign`.
2. Wheel radius, on the floor: drive about 2 m with `{linear: {x: 0.2}}`, then compare
   `ros2 topic echo --once /odom --field pose.pose.position` with a tape.
   `new radius = old radius * real distance / odom distance`.
3. Wheel separation, on the floor: restart the bridge, turn in place by one full turn with `{angular: {z: 0.5}}` and read
   `ros2 topic echo --once /odom --field pose.pose.orientation`. Odom angle = `360 + 2 * atan2(z, w)` in degrees.
   `new separation = old separation * odom angle / 360`.

## Tests
```
tools/docker-humble/run.sh bash -c "cd /ws/src/hoverboard_bridge && python3 -m pytest test -q"
```

## Known limitations
- After losing the ESP link, restart the stack: the EKF keeps integrating with no input, so the pose becomes wrong.
- `inflation_radius` (0.45 m) is smaller than the distance of the front corners from `base_link` (0.57 m). A goal close
  to a wall can leave the front of the robot against it, and the final heading may then be unreachable.

## Troubleshooting
| Symptom | Likely cause |
|---|---|
| `ldlidar communication is abnormal` and the driver restarts | No LIDAR data: ESP not reachable or LIDAR not connected |
| `Timed out waiting for transform` messages | Normal until odometry and the first scans arrive; if they continue, the ESP link is down |
| RViz reports no `map` frame on a saved map | Initial pose not set: use "2D Pose Estimate" |
| Start-up stuck with `map_server` at `Configuring` | Seen once, cause unknown: restart the stack |
| Hoverboard repeats 3 low beeps | Serial timeout on the mainboard: see "Troubleshooting" in [`esp32s3/README.md`](../esp32s3/README.md) |
