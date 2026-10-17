# Hoverboard mainboard firmware

The firmware is [hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC),
included as a git submodule. This project only stores its configuration as `config.patch`.

Main settings in the patch:
- `VARIANT_USART`, control and feedback on USART3 (right sensor cable)
- `CTRL_MOD_REQ SPD_MODE`, `TANK_STEERING`, `ENABLE_ODOMETRY`
- `N_MOT_MAX 200` rpm, `INACTIVITY_TIMEOUT 30` min

## Build and flash
Requires [PlatformIO](https://platformio.org/) (`pio` on the PATH).
```
git submodule update --init
./build.sh               # build only
./build.sh -t upload     # build and flash through the ST-Link
```
See the upstream README for wiring the ST-Link and unlocking the MCU.

## Changing the configuration
Edit the files inside the submodule, then save the diff and discard the changes:
```
cd hoverboard-firmware-hack-FOC
git apply ../config.patch          # start from the current configuration
# edit Inc/config.h, platformio.ini
git diff > ../config.patch
git checkout .
```
