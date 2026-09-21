# hoverboard_ros (ESP32-S3 firmware)

Bridges the hoverboard mainboard, a WT901C IMU and a serial LIDAR to ROS2 over UDP.
UDP travels over WiFi (station or access point) and/or USB tethering (CDC-NCM).

## Build and configure
```
cd esp32/hoverboard_ros
idf.py set-target esp32s3      # first time only
idf.py menuconfig              # "Hoverboard ROS bridge" menu: WiFi mode, SSID, password, pins
idf.py build
```
`sdkconfig` holds your WiFi credentials and is git-ignored.

## Flash
Use the board port wired to the USB-UART bridge (on ESP32-S3-DevKitC-1: the port labelled "UART").
The native USB port ("USB") is used by the firmware as a network device.
```
idf.py -p <PORT> flash
```

## Reaching the ESP
| Medium | ESP address |
|---|---|
| USB tethering | 192.168.7.1 (PC gets 192.168.7.2 by DHCP) |
| WiFi access point | 192.168.4.1 |
| WiFi station | hoverboard.local or the router-assigned IP |

UDP port: 8888. Quick test without ROS, from any machine with Python 3 on the robot network:
`python3 -m hoverboard_bridge.link_probe --host <ESP address>` (run from `ros2/hoverboard_bridge`).

## Wiring
| Device | Device pin | ESP32-S3 |
|---|---|---|
| Hoverboard right sensor cable (USART3) | GND / TX / RX | GND / GPIO4 / GPIO5. NEVER connect the red 15V wire |
| WT901C | VCC / TX / GND (RX unconnected) | 5V / GPIO6 / GND |
| LIDAR | 5V / GND / TX / RX | 5V from DC-DC / GND / GPIO18 / GPIO17 |
