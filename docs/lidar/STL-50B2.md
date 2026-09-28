# STL-50B2 LIDAR – Reverse-engineered notes

Findings from bench tests of the "STL-50B2" 360° DTOF LIDAR bought on AliExpress. No official documentation was available: the seller only said that it "requires a command to start". Everything below was verified on the real device unless marked as *unverified*.

**TL;DR:** the sensor speaks the **LDROBOT command protocol** (same family as the LD14P) at **230400 baud**. It stays silent until it receives the start command `54 A0 04 00 00 00 00 5E`. Then it streams standard LDROBOT `54 2C` data packets at about 7 Hz.

## 1. Test setup

| Item | Value |
|---|---|
| Adapter | USB-serial CH340 (Windows, COM4) |
| Power | 5 V |
| Wiring | LIDAR TX → adapter RXD, LIDAR RX → adapter TXD, common GND |
| TIME pin | Not connected |
| Test date | 2026-09-15 |

## 2. Serial parameters

| Parameter | Value |
|---|---|
| Baud rate | **230400** |
| Format | 8 data bits, no parity, 1 stop bit, no flow control |
| Logic level | 3.3 V TTL (LDROBOT family standard; the CH340 adapter was fine) |
| Direction | Bidirectional (the LIDAR needs commands on its RX pin) |

## 3. Behaviour at power-on

- The motor does **not** spin and **no scan data** is sent.
- Every few seconds a short 5-byte status packet appears (see 5.3).
- Listening passively at 9600–2 000 000 baud shows only that sporadic packet, plus occasional line noise when the port is opened.

## 4. Commands

### 4.1 Frame format

| Byte | Field | Value |
|---|---|---|
| 0 | Header | `0x54` |
| 1 | Command code (mode) | see table below |
| 2 | Data length | `0x04` |
| 3–6 | Data buffer | 4 bytes, little endian |
| 7 | CRC8 | over bytes 0–6 (see section 6) |

The LIDAR answers each command by echoing a frame with the same structure.

### 4.2 Command list

| Code | Function | Frame to send | Verified |
|---|---|---|---|
| `0xA0` | Start motor and scanning | `54 A0 04 00 00 00 00 5E` | Yes: echo received, scan data starts |
| `0xA1` | Stop motor | `54 A1 04 00 00 00 00 4A` | Yes: data stops |
| `0xA3` | Query target speed | `54 A3 04 00 00 00 00 62` | Yes: echo received, data bytes `00 00 00 00` |
| `0xA2` | Set target speed [deg/s] | `54 A2 04 <speed LSB> <speed MSB> 00 00 <CRC>` | *Unverified* (from the LD14P manual; the setting is lost at power-off) |

LD14P manual examples for `0xA2`, not yet tested here:
- 7 Hz (2520 deg/s): `54 A2 04 D8 09 00 00 16`
- 6 Hz (2160 deg/s): `54 A2 04 70 08 00 00 A1`

Note: on this unit, the `0xA3` query answered with speed `0` before the start command. The LD14P manual shows the configured target speed instead.

## 5. Packets sent by the LIDAR

### 5.1 Scan data packet (47 bytes)

Standard LDROBOT format, identical to LD06/LD19/LD14P:

| Offset | Size | Field | Unit |
|---|---|---|---|
| 0 | 1 | Header `0x54` | |
| 1 | 1 | `0x2C` (type 1 + 12 points per packet) | |
| 2 | 2 | Rotation speed | deg/s |
| 4 | 2 | Start angle | 0.01 deg |
| 6 | 12 × 3 | Points: distance (2 bytes) + intensity (1 byte) | mm, 0–255 |
| 42 | 2 | End angle | 0.01 deg |
| 44 | 2 | Timestamp | ms, wraps at 30000 |
| 46 | 1 | CRC8 over bytes 0–45 | |

All multi-byte fields are little endian.

The angle of point *i* is obtained by interpolation. Add 36000 to the end angle if it is smaller than the start angle (the scan wrapped past 0°), and take the result modulo 36000:

```
step    = (end_angle - start_angle) / 11
angle_i = (start_angle + step * i) % 36000
```

A distance of 0 means no valid return.

### 5.2 Health packet (6 bytes)

Observed right after the start command: `54 1F 00 00 00 41`.

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | Header `0x54` |
| 1 | 1 | `0x1F` (health information) |
| 2 | 2 | Error code bit field (little endian) |
| 4 | 1 | Error mode: `0x00` normal, `0x01` warning, `0x02` error |
| 5 | 1 | CRC8 |

Error bits, from the LD14P manual and *unverified* on this unit:

| Bit | Meaning |
|---|---|
| 0, 1 | Laser tube abnormal |
| 2 | Receiver board voltage abnormal |
| 3 | Sensor abnormal |
| 4 | Encoder abnormal |
| 5 | Motor error or locked rotor |
| 6 | LIDAR blockage |

### 5.3 Idle status packet (5 bytes, meaning unknown)

Seen sporadically while the motor is stopped:

```
54 A1 27 79 5F
54 A1 8B 79 F0
```

- Structure: header `0x54`, `0xA1`, 2 data bytes, CRC8 (the CRC is valid).
- The data, read as uint16 little endian, was 31015 and 31115 in two captures minutes apart.
- It may be a "motor stopped" state report that shares code `0xA1` with the stop command. It is not needed for operation.

## 6. CRC8

- Polynomial `0x4D`, initial value `0x00`, not reflected, no final XOR.
- The same algorithm validated every command echo, health packet and data packet (1080/1080 packets in the capture).

```python
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

assert crc8(bytes([0x54, 0xA0, 0x04, 0, 0, 0, 0])) == 0x5E
```

The LDROBOT manuals publish the same CRC as a 256-entry lookup table: `crc = CrcTable[(crc ^ byte) & 0xFF]`.

## 7. Measured performance (3 s capture after start)

| Metric | Value |
|---|---|
| Data rate | ≈ 17 kB/s (50 798 bytes in 3 s) |
| Valid data packets | 1080, 0 CRC errors |
| Rotation speed | median 2514 deg/s ≈ **7.0 Hz** (includes the spin-up from standstill) |
| Points | 12 626 in 3 s (≈ 4 200 points/s) |
| Distances seen | 149 mm to 3 898 mm (indoor desk test, not a range test) |
| Angular coverage | Full 360° |
| Health | Error code 0, mode normal |

## 8. Minimal usage example (Python, pyserial)

```python
import serial  # pip install pyserial

START = bytes.fromhex('54 A0 04 00 00 00 00 5E')
STOP = bytes.fromhex('54 A1 04 00 00 00 00 4A')

with serial.Serial('COM4', 230400, timeout=0.1) as port:
    port.write(START)
    try:
        buf = b''
        while True:
            buf += port.read(4096)
            i = buf.find(b'\x54\x2c')
            while i >= 0 and len(buf) - i >= 47:
                pkt = buf[i:i + 47]
                if crc8(pkt[:46]) == pkt[46]:
                    speed = int.from_bytes(pkt[2:4], 'little')
                    start = int.from_bytes(pkt[4:6], 'little') / 100
                    print(f'speed={speed} deg/s start_angle={start:.2f}')
                    buf = buf[i + 47:]
                else:
                    buf = buf[i + 1:]  # False header inside data: resync on the next byte
                i = buf.find(b'\x54\x2c')
    except KeyboardInterrupt:
        pass
    finally:
        port.write(STOP)
```

Replace `COM4` with `/dev/ttyUSB0` on Linux. Always send STOP before closing, or the motor keeps spinning.

### 8.1 Live viewer (debug tool)

[tools/lidar/lidar_viewer.py](../../tools/lidar/lidar_viewer.py) starts the LIDAR and shows each full revolution as a polar plot. Points are colored by intensity, and the title shows the point count and the rotation speed. Closing the window sends STOP.

Setup, once, from `tools/lidar`:

```
python -m venv .venv
.venv\Scripts\activate            (Linux: source .venv/bin/activate)
pip install -r requirements.txt
```

Run:

```
python lidar_viewer.py --port COM4 --max-range 4000
```

| Option | Default | Meaning |
|---|---|---|
| `--port` | `COM4` | Serial port (`/dev/ttyUSB0` on Linux) |
| `--max-range` | `4000` | Plot radius [mm] |

The plot has 0° at the top and angles increasing clockwise, which is the LDROBOT convention. The rotation direction has not been checked on this unit: if the image looks mirrored, remove `ax.set_theta_direction(-1)` from the script.

## 9. Pitfalls met during identification

- **Wrong documentation:** an AI-generated "protocol introduction" from the seller claimed 115200 baud and did not mention the start command. Both are wrong for this unit.
- **LDROBOT ROS2 driver:** `ldlidar_stl_ros2` (LD06/LD19/STL-27L) parses the data format correctly but never sends the start command. On its own it sees a silent line.
- **RPLIDAR/YDLIDAR commands** (`A5 50`, `A5 52`, `A5 60`) have no effect.
- **Silence is not a wiring fault:** with correct wiring the line is quiet until the start command arrives.

## 10. Open questions

- Pinout of the 5-pin connector, in particular the TIME synchronization pin: direction, signal type (pulse per revolution? PPS input?) and voltage level.
- Whether `0xA2` (set speed) works and which speed range is accepted.
- Meaning of the idle `54 A1 xx xx` status packet.
- Maximum range and minimum distance, which need an outdoor or long-hallway test.

## 11. Integration notes for this project

- The hoverboard ROS2 bridge must send `54 A0 …` when it starts using the LIDAR and `54 A1 …` when it stops, through the planned `LIDAR_TX` channel.
- The data stream (≈ 17 kB/s) fits easily in UDP over WiFi or USB.

## Sources

- LDROBOT LD14P Development Manual (Waveshare), sections 2.1–2.3: command frame, command codes, health packet, CRC table. https://files.waveshare.com/upload/9/99/LD14P_Development_Manual.pdf
- LDROBOT LD19 Development Manual V2.3, sections 2–3: data packet and PWM pin. https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf
- LDROBOT ROS2 driver: https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
