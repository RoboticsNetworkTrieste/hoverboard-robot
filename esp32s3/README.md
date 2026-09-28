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
The board is a Waveshare ESP32-S3-Zero: it has a single USB-C port, wired to the native USB.
Once the firmware runs, that port is a USB network device (CDC-NCM), not a serial port.
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
| WiFi station | hoverboard.local or the router-assigned IP |

UDP port: 8888. Quick test without ROS, from any machine with Python 3 on the robot network:
`python3 -m hoverboard_bridge.link_probe --host <ESP address>` (run from `ros2/hoverboard_bridge`).

## Wiring
| Device | Device pin | ESP32-S3 |
|---|---|---|
| Hoverboard right sensor cable (USART3) | GND / TX / RX | GND / GPIO4 / GPIO5. NEVER connect the red 15V wire |
| WT901C | VCC / TX / GND (RX unconnected) | 3V3 / GPIO6 / GND |
| LIDAR | 5V / GND / TX / RX | 5V from DC-DC / GND / GPIO7 / GPIO8 |
| 5V DC-DC (fed by the hoverboard) | 5V / GND | 5V / GND |

Only the edge pins of the ESP32-S3-Zero are used (5V, GND, 3V3, GPIO1-13, GPIO43/44);
the pads on the top and bottom faces are left unsoldered.
The ESP 5V pin and the LIDAR share the DC-DC output. The IMU runs from the ESP 3V3 pin.
