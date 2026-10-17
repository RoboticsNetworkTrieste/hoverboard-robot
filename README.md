# hoverboard-robot

A simple and cheap ROS2 mobile robot platform built on hoverboard hardware.

An ESP32-S3 connects to the hoverboard mainboard, an IMU and a LIDAR, and bridges them
to ROS2 over UDP (WiFi or USB tethering).

## Repository layout
| Folder | Content |
|---|---|
| [`esp32s3/`](esp32s3/) | ESP-IDF firmware for the ESP32-S3 bridge |
| [`ros2/`](ros2/) | ROS2 packages |
| [`hoverboard/`](hoverboard/) | Mainboard firmware (submodule) and its configuration patch |
| [`tools/`](tools/) | Bench tests: Arduino UART link test, LIDAR viewer |
| [`docs/`](docs/) | Hardware notes and design documents |

## Getting started
```
git clone --recurse-submodules git@github.com:RoboticsNetworkTrieste/hoverboard-robot.git
```
1. Flash the mainboard: see [`hoverboard/README.md`](hoverboard/README.md).
2. Build and flash the ESP32-S3: see [`esp32s3/README.md`](esp32s3/README.md).

## License
GPLv3, see [LICENSE](LICENSE).
