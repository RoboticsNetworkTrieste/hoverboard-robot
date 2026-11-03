# Pocophone F1 On-board Computer: Feasibility Gates Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Find out, with five pass/fail gates, whether a Pocophone F1 (6 GB RAM) running postmarketOS can be the on-board ROS2 computer linked to the ESP by USB tethering, and decide how to proceed.

**Architecture:** The phone runs a headless postmarketOS. ROS2 runs in the Ubuntu 22.04 container of `tools/docker-humble`, as on the development PC, because postmarketOS is based on Alpine (musl) and has no ROS2 Humble packages. Two scripts from `poco-f1-usb-host` switch the USB port to host mode and can make the phone power it.

**Tech Stack:** postmarketOS edge (renamed Nura on 2026-09-27; tools and packages still use the old name), `pmbootstrap`, `poco-f1-usb-host`, Docker, ROS2 Humble (`ros:humble` arm64), `link_probe`.

This plan runs in parallel with `2026-10-25-hoverboard-bringup-and-navigation.md`, whose floor tests continue on the development PC. If a gate fails with no workaround, another on-board computer takes the phone's place and works like the development PC, so that plan does not depend on this one.

---

## Known facts and risks

Checked on 2026-11-02:

| Fact | Source | Consequence |
|---|---|---|
| The mainline device tree leaves the USB port in peripheral mode with no VBUS source; there is no runtime role switch | `poco-f1-usb-host` README | Task 3 applies a device tree change and a reboot |
| `poco-f1-usb-host` has one documented test, a USB pendrive, by its author; never tested with a CDC-NCM device | Its README | Gate 2 is the main risk |
| The VBUS change keeps 5 V on the port permanently; no external source may be connected while it is active | Its README | Safety rule below |
| A kernel update restores the original device tree | Its README | No `apk upgrade` during the gates |
| An archived kernel config (6.11, `gitlab.com` mirror) has `USB_NET_CDC_NCM`, `REGULATOR_QCOM_USB_VBUS`, `OVERLAY_FS`, `VETH`, `BRIDGE`, `NF_NAT`, namespaces and cgroups | pmaports | Good sign; Task 1 rechecks the current kernel before wiping |
| A community guide reports WiFi problems on this phone, with no details | Headless install gist | Gate 1 tests WiFi; host mode leaves it as the only remote access |
| `osrf/ros:humble-desktop` is amd64 only; `ros:humble` is amd64 and arm64 | Docker Hub manifests | Task 5 makes the base image selectable |
| `ros-humble-nav2-bringup`, `slam-toolbox` and `robot-localization` exist for jammy arm64 | packages.ros.org | Navigation packages are available in the container |

Not verified, checked by the steps below: Docker on this kernel, whether the battery charges in host mode, the current the phone can supply, exact `pmbootstrap init` prompts and package names.

The official device wiki page could not be read while writing this plan. Read `https://wiki.nura.eco/wiki/Xiaomi_Poco_F1_(xiaomi-beryllium)` in a browser in Task 1 and report anything that contradicts this plan.

## Global Constraints

- All code comments, logs and identifiers in English. Comments short and useful.
- Version control is handled by the user: tasks contain no git steps and executors run no git write operations on this repository.
- Ask the user for confirmation before every flash: `pmbootstrap flasher`, `fastboot flash`, `fastboot erase`, and each `poco-usb` or `poco-vbus` command that changes the configuration, because they rewrite the boot partition. Installing postmarketOS wipes Android and all phone data.
- During hardware checks, do not edit code on your own initiative: run the check, report what was seen, propose the next step and wait.
- This plan does not change the ESP firmware. The ESP must already run a build with `HB_USB_NET` enabled.
- Do not run `apk upgrade` during the gates. If a kernel update happens anyway, repeat Task 3 Step 3.
- Every gate is run and its result written in "Gate results", including failures. On a failed gate, stop and report: later gates depend on earlier ones.

### Electrical safety rules

- **Keep the step-down output disconnected from the ESP 5V pin whenever the USB-C cable is plugged into the ESP.** LIDAR, IMU and level shifter HV stay on the step-down rail.
- **Never connect a power bank, charger, Y-cable power input or PC to the phone port while the VBUS change is active** (`poco-vbus status` shows the node as present). Run `sudo poco-vbus revert --reboot` first.
- Before connecting the phone to the PC for fastboot with the VBUS change active, power the phone off completely, then enter fastboot.
- Do not connect the step-down rail to the phone or to the OTG cable. Task 6 measures the current this would need; the follow-up plan decides.

### Power setups

| Setup | Phone configuration | Cabling |
|---|---|---|
| **E** (external) | `poco-usb host` only | Phone -> Y-cable -> ESP; power bank on the Y-cable power input |
| **P** (phone) | `poco-usb host` and `poco-vbus apply` | Phone -> OTG adapter -> ESP; no external source |

### Prerequisite from the bring-up plan

Task 3 Steps 1-5 of `2026-10-25-hoverboard-bringup-and-navigation.md` add the `datagrams:` line to `link_probe`. Gates 3 and 5 need it.

### Hardware needed

| Item | Used for |
|---|---|
| USB-C to USB-A OTG adapter | Setup P |
| USB-C OTG Y-cable with a separate power input, or a powered USB hub behind the OTG adapter | Setup E |
| 5V USB power bank, at least 2 A | Setup E |
| USB-A to USB-C cable for the ESP | Tethering |
| Multimeter, USB power meter | VBUS and current |

### Where each step runs

| Name | Machine |
|---|---|
| (PC) | Development PC, from the repo root unless stated |
| (phone) | Phone, over SSH as user `robot`; `<PHONE>` is its WiFi address |
| (phone container) | `tools/docker-humble/run.sh` on the phone, from `~/hoverboard-robot` |

Services are managed according to the init system chosen by `pmbootstrap`. Check with `test -d /run/systemd/system && echo systemd || echo openrc`, then enable `<service>` with:
- systemd: `sudo systemctl enable --now <service>`
- OpenRC: `sudo rc-update add <service> default && sudo rc-service <service> start`

Results are recorded in "Gate results" at the end of this file.

## File Structure

```
tools/docker-humble/Dockerfile      modify: selectable base image
tools/docker-humble/compose.yaml    modify: pass the base image to the build
tools/docker-humble/README.md       modify: document it
tools/phone/log_health.sh           create: battery, temperature and CPU log for Gate 5
```

---

### Task 1: Prepare the PC and check the kernel before wiping

No gate. Android stays usable until the end of this task.

- [ ] **Step 1: Read the device wiki page**

Open `https://wiki.nura.eco/wiki/Xiaomi_Poco_F1_(xiaomi-beryllium)` in a browser. Report what it says about USB OTG, WiFi, charging and the panel variants, and anything that contradicts this plan.

- [ ] **Step 2: Install the tools (PC)**

```bash
sudo apt install fastboot git openssl python3
mkdir -p ~/phone-images ~/.local/bin && cd ~/phone-images
git clone --depth=1 https://gitlab.nura.eco/nura/pmbootstrap.git
ln -s "$PWD/pmbootstrap/pmbootstrap.py" ~/.local/bin/pmbootstrap
pmbootstrap --version
```
Expected: a `3.x` version. Do not use the `pmbootstrap` package from PyPI (2.1.0, from 2023).

- [ ] **Step 3: Configure pmbootstrap (PC)**

Run: `pmbootstrap init`, with these answers:

| Question | Answer |
|---|---|
| Channel | `edge` (`poco-f1-usb-host` requires it) |
| Vendor / codename | `xiaomi` / `beryllium` |
| Kernel variant | `tianma` (corrected in Task 2 if the phone has the other panel) |
| User | `robot` |
| User interface | `none` |
| Other questions | the default |

Note the init system offered and the kernel package. Expected kernel: `linux-postmarketos-qcom-sdm845`. If it is another package, stop and report: the scripts require this one.

- [ ] **Step 4: Check the current kernel options, without touching the phone (PC)**

```bash
pmbootstrap kconfig check --containers linux-postmarketos-qcom-sdm845
grep -E '^CONFIG_(USB_NET_CDC_NCM|REGULATOR_QCOM_USB_VBUS|OVERLAY_FS|VETH|BRIDGE)=' \
  "$(pmbootstrap config work)"/cache_git/pmaports/device/community/linux-postmarketos-qcom-sdm845/config-*
```
Expected: the check passes and all five options are `y` or `m`. Report the result to the user before going on. If an option is missing, the phone can still be tried, but Gate 2 or Gate 4 is then expected to fail; the user decides. If `--containers` is not accepted, read `pmbootstrap kconfig check --help` and use the equivalent option.

- [ ] **Step 5: Keep a way back to Android**

Copy away any phone data to keep. Install all Android system updates. Download the latest fastboot ROM (`.tgz`) for `beryllium` from Xiaomi and keep it on the PC: Xiaomi's MiFlash tool restores Android with it.

- [ ] **Step 6: Check the bootloader (PC)**

Power off the phone, hold Power and Volume Down until the FASTBOOT screen appears, connect it to the PC.
Run: `fastboot getvar product 2>&1 | head -1` and `fastboot oem device-info`
Expected: `product: beryllium` and `Device unlocked: true`.
If it is locked, unlock it with Xiaomi's Mi Unlock tool (Windows, needs a Mi account, can impose a waiting period; it wipes the phone). This can take days: report it to the user.

---

### Task 2: Gate 1, postmarketOS runs and stays reachable over WiFi

**Pass:** the phone boots postmarketOS, joins the WiFi and stays reachable over WiFi SSH for 30 minutes with no reboot, with ping loss under 5% and no long gap.

- [ ] **Step 1: Build the image (PC)**

All packages go in now because no upgrade is allowed after host mode is set.
Run:
```bash
pmbootstrap install --add bash,docker,docker-cli-compose,dtc,i2c-tools,networkmanager,networkmanager-cli,networkmanager-wifi,python3,rsync,stress-ng,usbutils
```
Set the user password when asked. Expected: `install` finishes with no error. If a package name is not found, find the right name with `pmbootstrap chroot -- apk search <name>`, report it and retry.

- [ ] **Step 2: Flash (PC, ask the user first: this wipes the phone)**

With the phone in fastboot mode:
```bash
pmbootstrap flasher flash_rootfs
pmbootstrap flasher flash_kernel
fastboot erase dtbo
fastboot reboot
```
Expected: the phone reboots. The screen stays blank with UI `none`; the first boot takes 1 to 2 minutes.

- [ ] **Step 3: Log in over USB (PC)**

Keep the phone connected to the PC.
Run: `ping -c 3 172.16.42.1; ssh robot@172.16.42.1 'cat /etc/os-release | head -3; uname -r; test -d /run/systemd/system && echo systemd || echo openrc'`
Expected: replies, then distribution, kernel version and init system. Note them.

- [ ] **Step 4: Check the panel variant (phone)**

Run: `sudo i2cget -y 14 0x38 0x00; tr -d '\0' < /proc/device-tree/model; echo`
An answer from `0x38` means the EBBG variant; no answer means Tianma. If it does not match the kernel variant, run `sudo apk add device-xiaomi-beryllium-kernel-ebbg` (or `-tianma`) and reboot. This needs internet on the phone, so do it after Step 5 if needed. Note the variant.

- [ ] **Step 5: Join the WiFi (phone)**

Enable the `networkmanager` service as in "Where each step runs". Then:
```bash
sudo nmcli --ask device wifi connect "<SSID>"
ip -br addr show wlan0
```
Expected: an address on `wlan0`, which is `<PHONE>`. Reserve it in the router if possible.

- [ ] **Step 6: Key login over WiFi (PC)**

Disconnect the USB cable.
Run: `ssh-copy-id robot@<PHONE> && ssh robot@<PHONE> uptime`
Expected: login without a password. If the connection is refused while WiFi is up, the firewall may block it: over USB, check `sudo nft list ruleset` and report.

- [ ] **Step 7: Check that WiFi returns after a reboot (phone)**

Run: `sudo reboot`, then from the PC, after 2 minutes: `ssh robot@<PHONE> uptime`
Expected: reachable with no intervention. Host mode makes this the only remote access, so do not go on if it fails.

- [ ] **Step 8: Run the gate (PC)**

Run: `ping -i 1 -c 1800 <PHONE> | tail -2; ssh robot@<PHONE> uptime`
Expected: `uptime` longer than 30 minutes, ping loss under 5%. Record the loss, then record Gate 1.

---

### Task 3: Gate 2, the phone is a USB host and sees the ESP network

**Pass:** in setup E or in setup P, the ESP network interface appears on the phone, gets `192.168.7.2` and answers 100 pings with 0% loss; it returns after 5 replugs and after a phone reboot.

Host mode disables the USB login at `172.16.42.1`, so WiFi is the only remote access from here on. The FASTBOOT screen always remains reachable with Power and Volume Down.

- [ ] **Step 1: Save the current boot image (PC)**

The backups made by the scripts stay on the phone, where they are useless if it no longer boots.
```bash
mkdir -p ~/phone-images/boot-backup
ssh -t robot@<PHONE> 'sudo cp /boot/boot.img boot-pre-usb.img && sudo chown robot boot-pre-usb.img && sha256sum boot-pre-usb.img'
scp robot@<PHONE>:boot-pre-usb.img ~/phone-images/boot-backup/
sha256sum ~/phone-images/boot-backup/boot-pre-usb.img
```
Expected: both checksums match. If `/boot/boot.img` does not exist, stop and report: there is no way back from fastboot without this image. The way back is `fastboot flash boot ~/phone-images/boot-backup/boot-pre-usb.img`.

- [ ] **Step 2: Copy the scripts at the reviewed commit (PC)**

The scripts only edit the device tree in `/boot/dtbs/qcom`, run `mkinitfs` and optionally reboot. This was checked at commit `3941b98633`, so use that commit.
```bash
BASE=https://raw.githubusercontent.com/PiotrTopa/poco-f1-usb-host/3941b98633
mkdir -p ~/phone-images/poco-usb-host && cd ~/phone-images/poco-usb-host
curl -fsSO $BASE/poco-usb && curl -fsSO $BASE/poco-vbus
scp poco-usb poco-vbus robot@<PHONE>:
ssh -t robot@<PHONE> 'sudo install -m 0755 poco-usb poco-vbus /usr/local/sbin/ && sudo poco-usb status; sudo poco-vbus status'
```
Expected: the printed DTB matches the variant of Task 2 Step 4, `live dr_mode` is `peripheral`, `DTB node present` is `no`. If the DTB is wrong, prefix the commands with `POCO_DTB=/boot/dtbs/qcom/sdm845-xiaomi-beryllium-<variant>.dtb`.

- [ ] **Step 3: Switch to host mode (phone, ask the user first)**

Run: `sudo poco-usb host --reboot`
After the reboot, run: `sudo poco-usb status`
Expected: `live dr_mode` and `device-tree dr_mode` are `host`, `USB role (live)` is `HOST`. If the phone does not come back within 3 minutes, flash the saved image as in Step 1.

- [ ] **Step 4: Setup E, external power**

ESP on the bench, connected to nothing else. Connect the power bank to the Y-cable power input and the Y-cable to the phone. Before connecting the ESP, measure the voltage on the Y-cable USB port: 4.75 V to 5.25 V. Connect the ESP.
Run (phone):
```bash
lsusb
ip -br link
ip -br addr
```
Expected: an Espressif device (vendor `303a`), a new interface with MAC `02:48:42:00:07:02` (`<ESP_IF>`) and the address `192.168.7.2` from the ESP DHCP server. If the interface exists with no address, run `sudo nmcli device connect <ESP_IF>`. If there is no interface, run `sudo modprobe cdc_ncm; dmesg | tail -30` and report.

- [ ] **Step 5: Ping in setup E (phone)**

Run: `ping -c 100 -i 0.2 192.168.7.1 | tail -2`
Expected: 0% loss. Note the max RTT (2.3 ms on the development PC). Also note `status` and `capacity` in the battery folder of `/sys/class/power_supply`: they show whether the phone charges in this setup.

- [ ] **Step 6: Setup P, phone power (ask the user first)**

Disconnect the Y-cable and the power bank from the phone. Nothing stays connected to the port.
Run (phone): `sudo poco-vbus apply --reboot`
After the reboot, run: `sudo poco-vbus status`
Expected: `DTB node present: yes`, `regulator ... state=enabled`.
Plug the OTG adapter into the phone and measure on its USB port: 4.75 V to 5.25 V. If the voltage is outside this range, do not connect the ESP, and report.
Connect the ESP through the USB power meter, then repeat the commands of Steps 4 and 5.
Expected: the same results as setup E. Note the ESP current. In this setup the phone runs only on its battery.

- [ ] **Step 7: Choose the setup for the next gates**

Use E if it worked, otherwise P. If E is chosen, disconnect the OTG adapter and run `sudo poco-vbus revert --reboot` **before** connecting the Y-cable or anything external.
Run: `sudo poco-usb status; sudo poco-vbus status`
Expected: the state of the chosen setup in "Power setups".

- [ ] **Step 8: Run the gate**

Unplug and replug the ESP cable 5 times, each time running `ping -c 5 192.168.7.1`. Then reboot the phone with the ESP connected and repeat.
Expected: all pings answered once the interface is back, and `poco-usb status` still `host` after the reboot. Record Gate 2 with the result of each setup.

---

### Task 4: Gate 3, the link is as good as on the development PC

**Pass:** in 5 minutes with all sensors connected, less than 0.1% of datagrams lost and no `CMD_TIMEOUT` in the status lines; the wheels stop within about 0.5 s when the link is cut.

Reference on the development PC: 0 datagrams lost out of 3000.

- [ ] **Step 1: Copy the tools (PC)**

Run: `rsync -a --exclude build --exclude install --exclude log --exclude __pycache__ ros2 tools robot@<PHONE>:hoverboard-robot/`

- [ ] **Step 2: Connect the robot**

Phone on the bench next to the robot, wheels lifted. Step-down output disconnected from the ESP 5V pin; LIDAR, IMU and level shifter on the step-down rail. ESP on the setup chosen in Task 3. Turn the hoverboard on.

- [ ] **Step 3: Measure (phone, from `~/hoverboard-robot/ros2/hoverboard_bridge`)**

```bash
python3 -m hoverboard_bridge.link_probe --host 192.168.7.1 --duration 300 > ~/g3.log
grep -c CMD_TIMEOUT ~/g3.log
tail -n 3 ~/g3.log
grep -m3 -E 'HOVER_OK|IMU_OK' ~/g3.log
```
Expected: `0` `CMD_TIMEOUT`; a `datagrams:` line with `lost` under 0.1%; status lines with `HOVER_OK` and `IMU_OK`. If there is no telemetry at all while ping works, check the firewall with `sudo nft list ruleset` and report.

- [ ] **Step 4: Link cut (wheels lifted, hand near the power button)**

Run (phone, shell 1): `python3 -m hoverboard_bridge.link_probe --host 192.168.7.1 --left 100 --right 100 --duration 30`
While the wheels turn, run (phone, shell 2): `sudo ip link set <ESP_IF> down`
Expected: the wheels stop within about 0.5 s. A network error in shell 1 is acceptable.
Run: `sudo ip link set <ESP_IF> up`, then `ping -c 5 192.168.7.1`. Expected: replies. Record Gate 3.

---

### Task 5: Gate 4, the ROS2 container works on the phone

**Pass:** the project image builds on the phone, and `bridge_node` plus the LIDAR driver publish `/odom` and `/scan` at their usual rates.

**Files:**
- Modify: `tools/docker-humble/Dockerfile`
- Modify: `tools/docker-humble/compose.yaml`
- Modify: `tools/docker-humble/README.md`

- [ ] **Step 1: Make the base image selectable**

In `tools/docker-humble/Dockerfile`, replace the `FROM` line with:
```dockerfile
# osrf/ros:humble-desktop is amd64 only; use ros:humble on arm64 (no GUI tools).
ARG BASE_IMAGE=osrf/ros:humble-desktop
FROM ${BASE_IMAGE}
```
In `tools/docker-humble/compose.yaml`, under `build: args:`, add:
```yaml
        BASE_IMAGE: ${ROS_BASE_IMAGE:-osrf/ros:humble-desktop}
```
Append to `tools/docker-humble/README.md`:
```markdown
On arm64 (on-board phone) build from `ros:humble`, which has no GUI tools:
`ROS_BASE_IMAGE=ros:humble tools/docker-humble/run.sh --build`.
```

- [ ] **Step 2: Check the new base image (PC)**

Run: `ROS_BASE_IMAGE=ros:humble tools/docker-humble/run.sh --build ros2 pkg list | grep -E '^(rclpy|tf2_ros|nav_msgs|sensor_msgs|ldlidar_stl_ros2)$'`
Expected: the five package names.

Restore the development image, because both builds use the same tag:
Run: `tools/docker-humble/run.sh --build ros2 pkg list | grep -c rviz2`
Expected: a count of at least 1.

- [ ] **Step 3: Start Docker (phone)**

Enable the `docker` service as in "Where each step runs". Then:
```bash
sudo addgroup robot docker
```
Log out and in again.
Run: `docker compose version; docker run --rm --network host ros:humble ros2 --help | head -3`
Expected: a Compose version and the `ros2` usage text. Note how long the pull takes. If the daemon does not start, read its log (`sudo journalctl -u docker -n 50` or `/var/log/docker.log`) and `sudo nft list ruleset`. Firewall errors may be solved by `postmarketos-config-nftables-docker`, but that package needs `apk add`, which can touch the kernel: report before installing. Record any kernel error: the fix decides between another kernel and running ROS2 outside Docker.

- [ ] **Step 4: Build the image and the workspace**

Run (PC): repeat the `rsync` of Task 4 Step 1.
Run (phone, from `~/hoverboard-robot`): `time ROS_BASE_IMAGE=ros:humble tools/docker-humble/run.sh --build bash -c 'cd /ws && colcon build --symlink-install'`
Expected: `Finished <<< hoverboard_bridge`. Note the time.

- [ ] **Step 5: Run the bridge (phone container, terminal 1)**

Connect the robot as in Task 4 Step 2.
Run: `tools/docker-humble/run.sh`, then in it `ros2 launch hoverboard_bridge bridge.launch.py esp_host:=192.168.7.1`
Expected: `Bridging to ESP at 192.168.7.1`, no exception.

- [ ] **Step 6: Run the LIDAR driver in the same container (terminal 2)**

Run (phone): `docker ps` to get the container name, then `docker exec -it <container> bash` and `source /opt/hoverboard/ros_env.sh`. Use the `product_name` found in Task 1 Step 5 of the bring-up plan, `LDLiDAR_LD19` if that step is not done:
```bash
ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
  -p product_name:=LDLiDAR_LD19 -p topic_name:=scan -p frame_id:=laser_link \
  -p port_name:=/tmp/ttyLIDAR -p port_baudrate:=230400 -p laser_scan_dir:=true
```
Expected: `ldlidar node start is success`.

- [ ] **Step 7: Run the gate (terminal 3, same way as terminal 2)**

Run: `timeout 20 ros2 topic hz /scan; timeout 20 ros2 topic hz /odom; timeout 20 ros2 topic hz /imu/data`
Expected: `/scan` about 7 Hz, `/odom` steady, `/imu/data` steady, with rates close to those on the development PC. Note them. Record Gate 4.

---

### Task 6: Gate 5, sustained load, temperature and power

**Pass:** 30 minutes with four CPU stress workers running, with: `/scan` within 0.5 Hz of its rate in Gate 4; less than 0.1% of datagrams lost and no `CMD_TIMEOUT`; no reboot; no thermal shutdown; `cpu4` frequency at the end at least 70% of its maximum; estimated autonomy of at least 1 hour.

The 1-hour threshold is a starting value: the user can change it. The stress load only stands in for SLAM and Nav2, which do not exist yet. The real load is checked once Task 11 of the bring-up plan is done.

If both power setups passed Gate 2, run Steps 3 to 6 once for each, following the safety rules when switching: setup E shows whether external power can keep the phone charged, setup P shows the autonomy on the battery alone.

**Files:**
- Create: `tools/phone/log_health.sh`

- [ ] **Step 1: Write the log script**

Create `tools/phone/log_health.sh`:
```bash
#!/usr/bin/env bash
# Appends battery, temperature, CPU and memory state to a CSV file every 10 s. Stop with Ctrl+C.
# Usage: log_health.sh <output.csv>
set -euo pipefail

output="$1"

read_value() {
    cat "$1" 2>/dev/null || echo "-"
}

battery=""
for dir in /sys/class/power_supply/*; do
    if [[ -r "$dir/capacity" ]]; then
        battery="$dir"
        break
    fi
done

echo "time,capacity_pct,status,voltage_uv,current_ua,max_temp_mdeg,cpu4_khz,mem_available_kb,load1" >> "$output"
while true; do
    max_temp=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -n 1 || true)
    mem_available=$(awk '/MemAvailable/ {print $2}' /proc/meminfo)
    load1=$(cut -d' ' -f1 /proc/loadavg)
    echo "$(date +%T),$(read_value "$battery/capacity"),$(read_value "$battery/status"),$(read_value "$battery/voltage_now"),$(read_value "$battery/current_now"),${max_temp:--},$(read_value /sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq),$mem_available,$load1" >> "$output"
    sleep 10
done
```
Run: `chmod +x tools/phone/log_health.sh`

- [ ] **Step 2: Check it (PC)**

Run: `rm -f /tmp/health.csv; timeout 25 tools/phone/log_health.sh /tmp/health.csv; cat /tmp/health.csv`
Expected: the header and 3 data lines. Fields that do not exist on the machine show `-`.

Run (PC): repeat the `rsync` of Task 4 Step 1.

- [ ] **Step 3: Start from a known state**

Charge the phone above 80% and note the level. Charge it only with the VBUS change reverted (see the safety rules). Connect the robot as in Task 4 Step 2; in setup E, put the USB power meter between the power bank and the Y-cable; in setup P, between the OTG adapter and the ESP. Start the bridge and the LIDAR driver as in Task 5 Steps 5 and 6.

- [ ] **Step 4: Run 30 minutes under load (phone, three more SSH shells)**

```bash
# Shell A, stop with Ctrl+C when the 30 minutes end
~/hoverboard-robot/tools/phone/log_health.sh ~/g5_load.csv
# Shell B
stress-ng --cpu 4 --timeout 1800 --metrics-brief
# Shell C, in the bridge container as in Task 5 Step 6
PYTHONUNBUFFERED=1 timeout 1800 ros2 topic hz /scan | tee /tmp/scan_hz.log
```
Note the USB power meter current at the start and the end. At the end, run `uptime; cat /sys/devices/system/cpu/cpu4/cpufreq/cpuinfo_max_freq` on the phone.
Expected: the last lines of the `ros2 topic hz` output show the `/scan` rate within 0.5 Hz of Gate 4; `uptime` longer than the test; no shutdown.

- [ ] **Step 5: Link loss under load (phone)**

Stop the ROS2 nodes: the ESP sends telemetry to one peer, so the bridge and `link_probe` must not run at once. Start the stress load again: `stress-ng --cpu 4 --timeout 300 &`
Run (from `~/hoverboard-robot/ros2/hoverboard_bridge`): `python3 -m hoverboard_bridge.link_probe --host 192.168.7.1 --duration 300 > ~/g5.log; grep -c CMD_TIMEOUT ~/g5.log; tail -n 3 ~/g5.log`
Expected: `0` `CMD_TIMEOUT`, `lost` under 0.1%.

- [ ] **Step 6: Autonomy (phone)**

Run: `head -2 ~/g5_load.csv; tail -1 ~/g5_load.csv`
Compute `drop_pct_per_hour = 2 * (capacity_start - capacity_end)` and `autonomy_h = 100 / drop_pct_per_hour`. If the capacity did not fall, the autonomy condition passes as long as the external input stays connected. If `capacity_pct` is `-`, use `voltage_uv` and `status` and report that no percentage is available.
Report for each setup: whether `status` shows `Charging`, the `current_ua` sign, the peak temperature, the `cpu4_khz` at the end against `cpuinfo_max_freq`, and the power meter current. In setup E, this current decides whether the robot step-down can feed the phone in the final setup: compare it with the rating of the DSN-mini-360 module plus the LIDAR and IMU consumption. Record Gate 5.

---

### Task 7: Decide how to proceed

- [ ] **Step 1: Apply the decision table**

| Result | Next step |
|---|---|
| All gates pass | Write the follow-up plan: power from the robot, mounting and cooling, ROS2 autostart at boot, kernel update procedure, full Nav2 load check |
| Gate 1 fails | WiFi or boot problem: try the other kernel variant or a newer image once. Then try Mobian, or replace the phone |
| Gate 2 fails in both setups | Try a powered hub, then the Kali NetHunter Pro procedure on Mobian. Otherwise replace the phone. WiFi to the ESP is not a fallback on this board (`esp32s3/README.md`, "Link results") |
| Gate 3 fails | Same as Gate 2, after trying another cable and the other power setup |
| Gate 4 fails | Docker error: try a kernel with the missing options, or run ROS2 in a plain Ubuntu 22.04 root filesystem in a chroot. Otherwise replace the phone |
| Gate 5 fails | Temperature: add passive cooling and repeat. Autonomy: solve the power supply and repeat. CPU: lower the expected load (LIDAR rate, map resolution) or replace the phone |

"Replace the phone" means another on-board computer that works like the development PC: USB tethering to the ESP and the same container.

- [ ] **Step 2: Report**

Fill "Gate results" and present the results and the proposed next step to the user. The user decides.

---

## Gate results

Fill in while running the plan.

| Gate | Date | Result | Measurements |
|---|---|---|---|
| 1. postmarketOS and WiFi | | | channel, kernel, panel, init system, ping loss |
| 2. USB host and ESP network | | | setup E result, setup P result, max RTT, ESP current, replugs, charging in setup E |
| 3. Link quality | | | setup used, lost / received, `CMD_TIMEOUT`, stop time |
| 4. ROS2 container | | | build time, `/scan`, `/odom`, `/imu/data` rates |
| 5. Load, temperature, power | | | per setup: `/scan` rate, lost, peak temperature, `cpu4` end/max, charging, power meter current, autonomy |

## Out of scope

- Powering the phone from the robot, mounting, cooling.
- Starting ROS2 at boot, RViz on a remote PC over the phone WiFi.
- The procedure for kernel updates after the gates.
- Real SLAM and Nav2 load on the phone.
