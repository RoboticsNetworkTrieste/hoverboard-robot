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

## Getting the code
```
git clone --recurse-submodules git@github.com:RoboticsNetworkTrieste/hoverboard-robot.git
```
If you already cloned without submodules: `git submodule update --init`.

## PC prerequisites
Tested on Ubuntu 24.04. Commands below assume a Debian/Ubuntu system.

### Hardware
| Item | Used for |
|---|---|
| ST-Link V2 programmer (or clone) | Flashing the hoverboard mainboard through SWD |
| USB-C cable | Flashing the ESP32-S3-Zero |

### System packages
```
sudo apt install git curl python3 python3-pip python3-venv cmake ninja-build ccache \
  wget flex bison gperf libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### USB permissions
Your user must be in the `dialout` and `plugdev` groups, and udev rules must cover the ST-Link
and the ESP32-S3. The PlatformIO rules cover both:
```
sudo usermod -aG dialout,plugdev $USER
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
  | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Log out and back in for the group change to take effect.

### PlatformIO Core (hoverboard firmware)
```
curl -fsSL -o /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
mkdir -p ~/.local/bin
ln -sf ~/.platformio/penv/bin/{platformio,pio} ~/.local/bin/
```
`~/.local/bin` must be on the `PATH` (Ubuntu adds it at login if the folder exists).
Check with `pio --version`. The STM32 platform, the ARM toolchain and OpenOCD are downloaded
automatically by the first build and upload.
The [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) VS Code extension uses the same installation.

### STM32CubeProgrammer (hoverboard unlock, optional GUI flashing)
New hoverboard mainboards are usually read-protected and must be unlocked once.
Download the Linux version from [st.com](https://www.st.com/en/development-tools/stm32cubeprog.html)
(free ST account required), then:
```
unzip en.stm32cubeprg-lin-*.zip -d stm32cubeprog
./stm32cubeprog/SetupSTM32CubeProgrammer-*.linux
```
Start it with `~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32CubeProgrammer`.

### ESP-IDF v5.4 (ESP32-S3 firmware)
The firmware requires ESP-IDF `>=5.4,<5.5` (tested with v5.4.4).
```
mkdir -p ~/esp && cd ~/esp
git clone -b v5.4.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3
```
Load the environment in every new terminal before using `idf.py`:
```
. ~/esp/esp-idf/export.sh
```
The [Espressif Installation Manager (EIM)](https://docs.espressif.com/projects/idf-im-ui/en/latest/)
is an alternative installer; in that case activate with `. ~/.espressif/tools/activate_idf_v5.4.4.sh`.
For VS Code, the ESP-IDF extension can reuse this installation instead of downloading a new one.

### Optional
- ROS2: needed by the packages in [`ros2/`](ros2/) (work in progress).
- Arduino IDE or `arduino-cli`: for [`tools/arduino_hoverboard_test`](tools/arduino_hoverboard_test/).
- Python packages for the LIDAR viewer: `pip install -r tools/lidar/requirements.txt`.

## Hoverboard mainboard firmware
Settings and how to change them: [`hoverboard/README.md`](hoverboard/README.md).

### Build
```
cd hoverboard
./build.sh
```
The script applies `config.patch` to the submodule, builds the `VARIANT_USART` environment and
restores the submodule. Output: `hoverboard-firmware-hack-FOC/.pio/build/VARIANT_USART/firmware.bin`.

### Wiring the ST-Link
Connect the ST-Link to the 4-pin SWD header on the mainboard: GND, SWDIO, SWCLK, 3V3.
Do not connect the battery charger. Keep the hoverboard power button pressed during every
unlock and flash, otherwise the board switches itself off.

### Unlock (once per board)
In STM32CubeProgrammer:
1. Select **ST-LINK**, set Mode to **Under reset** (try **Hot plug** if it fails), press **Connect**.
2. Open the **OB** (Option Bytes) tab, **Read Out Protection**: set it to Level 0 / unprotected, press **Apply**.
   This erases the flash.
3. Disconnect.

### Flash
```
cd hoverboard
./build.sh -t upload
```
Alternatively, in STM32CubeProgrammer, **Erasing & Programming** tab: write `firmware.bin` at address `0x08000000`.

## ESP32-S3 firmware
Configuration, network setup and wiring: [`esp32s3/README.md`](esp32s3/README.md).

### Build
```
. ~/esp/esp-idf/export.sh
cd esp32s3
idf.py set-target esp32s3      # first time only
idf.py menuconfig              # "Hoverboard ROS bridge" menu: WiFi mode, SSID, password, pins
idf.py build
```
The first build downloads the managed components (`esp_tinyusb`, `mdns`).

### Flash
Once the firmware runs, the USB-C port is a network device, not a serial port, so the board
must be put in download mode first: hold BOOT, press and release RESET, release BOOT.
```
ls /dev/ttyACM*                # usually /dev/ttyACM0
idf.py -p /dev/ttyACM0 flash
```
Press RESET after flashing to start the firmware.

## License
GPLv3, see [LICENSE](LICENSE).
