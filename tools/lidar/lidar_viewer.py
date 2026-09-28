"""Live viewer for the STL-50B2 LIDAR (LDROBOT protocol, see docs/lidar/STL-50B2.md).

Usage:
    python -m venv .venv
    .venv\Scripts\activate          (Linux: source .venv/bin/activate)
    pip install -r requirements.txt
    python lidar_viewer.py --port COM4
"""

import argparse
import math
import threading

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

BAUD = 230400
START = bytes.fromhex('54 A0 04 00 00 00 00 5E')
STOP = bytes.fromhex('54 A1 04 00 00 00 00 4A')
HEADER = b'\x54\x2c'
PACKET_LEN = 47
POINTS_PER_PACKET = 12


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def parse_packet(pkt: bytes):
    """Return a list of (angle_rad, distance_mm, intensity) for one 47-byte data packet."""
    start = int.from_bytes(pkt[4:6], 'little')
    end = int.from_bytes(pkt[42:44], 'little')
    if end < start:
        end += 36000
    step = (end - start) / (POINTS_PER_PACKET - 1)
    points = []
    for i in range(POINTS_PER_PACKET):
        off = 6 + 3 * i
        dist = int.from_bytes(pkt[off:off + 2], 'little')
        intensity = pkt[off + 2]
        if dist == 0:
            continue  # No valid return
        angle = ((start + step * i) % 36000) / 100
        points.append((math.radians(angle), dist, intensity))
    return points


class LidarReader(threading.Thread):
    """Reads the serial stream and publishes one full revolution at a time."""

    def __init__(self, port: serial.Serial):
        super().__init__(daemon=True)
        self.port = port
        self.lock = threading.Lock()
        self.scan = []  # Last complete revolution
        self.speed = 0  # deg/s
        self.running = True

    def run(self):
        buf = b''
        current = []
        last_start = -1
        while self.running:
            buf += self.port.read(4096)
            i = buf.find(HEADER)
            while i >= 0 and len(buf) - i >= PACKET_LEN:
                pkt = buf[i:i + PACKET_LEN]
                if crc8(pkt[:46]) != pkt[46]:
                    buf = buf[i + 1:]  # False header inside data: resync on the next byte
                    i = buf.find(HEADER)
                    continue
                buf = buf[i + PACKET_LEN:]
                start = int.from_bytes(pkt[4:6], 'little')
                if start < last_start:  # Angle wrapped past 0°: revolution complete
                    with self.lock:
                        self.scan = current
                        self.speed = int.from_bytes(pkt[2:4], 'little')
                    current = []
                last_start = start
                current.extend(parse_packet(pkt))
                i = buf.find(HEADER)
            if i < 0 and len(buf) > 1:
                buf = buf[-1:]  # Keep a possible partial header


def main():
    parser = argparse.ArgumentParser(description='STL-50B2 LIDAR live viewer')
    parser.add_argument('--port', default='COM4', help='Serial port (e.g. COM4 or /dev/ttyUSB0)')
    parser.add_argument('--max-range', type=float, default=4000, help='Plot radius [mm]')
    args = parser.parse_args()

    port = serial.Serial(args.port, BAUD, timeout=0.1)
    port.reset_input_buffer()
    port.write(START)
    reader = LidarReader(port)
    reader.start()

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(projection='polar')
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)  # LIDAR angles increase clockwise
    ax.set_rmax(args.max_range)
    sc = ax.scatter([], [], s=4, c=[], cmap='viridis', vmin=0, vmax=255)
    fig.colorbar(sc, ax=ax, shrink=0.7, label='Intensity')
    title = ax.set_title('Waiting for data...')

    def update(_):
        with reader.lock:
            scan = reader.scan
            speed = reader.speed
        if scan:
            sc.set_offsets([(a, d) for a, d, _ in scan])
            sc.set_array([p[2] for p in scan])
            title.set_text(f'{len(scan)} points  |  {speed / 360:.1f} Hz')
        return sc, title

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        reader.running = False
        reader.join(timeout=1)
        port.write(STOP)  # Otherwise the motor keeps spinning
        port.close()


if __name__ == '__main__':
    main()
