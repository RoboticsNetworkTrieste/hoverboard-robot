"""Bench tool without ROS: sends wheel commands over UDP and prints ESP telemetry."""
import argparse
import socket
import time

from hoverboard_bridge import link_protocol as lp


def format_status(state, imu, lidar_bytes):
    if state is None:
        return 'no telemetry'
    flags = []
    if state.status & lp.STATUS_HOVER_OK:
        flags.append('HOVER_OK')
    if state.status & lp.STATUS_CMD_TIMEOUT:
        flags.append('CMD_TIMEOUT')
    if state.status & lp.STATUS_IMU_OK:
        flags.append('IMU_OK')
    text = (f'ticks L={state.ticks_l:7d} R={state.ticks_r:7d} | rpm L={state.rpm_l:4d} R={state.rpm_r:4d} | '
            f'bat={state.bat_cv / 100:.2f}V temp={state.temp_dc / 10:.1f}C | '
            f'ok={state.hover_ok} bad={state.hover_bad} | {" ".join(flags)}')
    if imu is not None:
        text += f' | imu angle={imu.angle}'
    return text + f' | lidar={lidar_bytes}B'


def print_ticks_per_rev(first, last, elapsed, rpm_sum, rpm_count):
    """Estimates ticks per wheel revolution from tick deltas and mean measured rpm."""
    if first is None or rpm_count == 0 or elapsed <= 0:
        return
    for name, d_ticks, rpm in (('left', last.ticks_l - first.ticks_l, rpm_sum[0] / rpm_count),
                               ('right', last.ticks_r - first.ticks_r, rpm_sum[1] / rpm_count)):
        revs = rpm / 60.0 * elapsed
        estimate = abs(d_ticks / revs) if abs(revs) > 0.5 else float('nan')  # Right rpm and ticks differ in sign
        print(f'{name}: d_ticks={d_ticks} mean_rpm={rpm:.1f} ticks_per_rev~{estimate:.1f}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.50',
                        help='ESP address: WiFi STA static IP (192.168.1.50), WiFi AP 192.168.4.1, USB 192.168.7.1')
    parser.add_argument('--port', type=int, default=8888)
    parser.add_argument('--left', type=int, default=0, help='Left wheel command (-1000..1000)')
    parser.add_argument('--right', type=int, default=0, help='Right wheel command (-1000..1000)')
    parser.add_argument('--duration', type=float, default=5.0, help='Seconds before stopping')
    parser.add_argument('--lidar-hex', default='',
                        help='Hex bytes sent once to the LIDAR after the first command, '
                             'e.g. "54 A2 04 D8 09 00 00 16" to set 7 Hz')
    args = parser.parse_args()

    addr = (socket.gethostbyname(args.host), args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    seq = 0
    state = imu = None
    first_state = None
    first_time = last_time = 0.0
    rpm_sum = [0.0, 0.0]
    rpm_count = 0
    lidar_bytes = 0
    t_start = time.monotonic()
    t_cmd = t_print = 0.0

    while time.monotonic() - t_start < args.duration:
        now = time.monotonic()
        if now - t_cmd >= 0.05:
            sock.sendto(lp.encode_cmd_wheels(seq, args.left, args.right), addr)
            seq += 1
            t_cmd = now
            if args.lidar_hex and seq == 1:
                sock.sendto(lp.encode(lp.MSG_LIDAR_TX, seq, 0, bytes.fromhex(args.lidar_hex)), addr)
                seq += 1
        while True:
            try:
                data, _ = sock.recvfrom(lp.MAX_DATAGRAM)
            except (BlockingIOError, ConnectionResetError):  # Windows reports ICMP unreachable as reset
                break
            msg = lp.decode(data)
            if msg is None:
                continue
            if msg.type == lp.MSG_WHEEL_STATE:
                state = lp.parse_wheel_state(msg.body)
                if state is not None and state.status & lp.STATUS_HOVER_OK:
                    if first_state is None:
                        first_state, first_time = state, now
                    last_time = now
                    rpm_sum[0] += state.rpm_l
                    rpm_sum[1] += state.rpm_r
                    rpm_count += 1
            elif msg.type == lp.MSG_IMU:
                imu = lp.parse_imu(msg.body)
            elif msg.type == lp.MSG_LIDAR_RX:
                if lidar_bytes < 64:
                    print('[lidar]', msg.body[:64 - lidar_bytes].hex(' '))
                lidar_bytes += len(msg.body)
            elif msg.type == lp.MSG_LOG:
                print('[esp]', msg.body.decode('ascii', errors='replace').rstrip())
        if now - t_print >= 0.2:
            t_print = now
            print(format_status(state, imu, lidar_bytes))
        time.sleep(0.005)

    for _ in range(5):
        sock.sendto(lp.encode_cmd_wheels(seq, 0, 0), addr)
        seq += 1
        time.sleep(0.05)
    print_ticks_per_rev(first_state, state, last_time - first_time, rpm_sum, rpm_count)


if __name__ == '__main__':
    main()
