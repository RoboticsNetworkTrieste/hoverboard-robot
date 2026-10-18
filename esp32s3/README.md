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
