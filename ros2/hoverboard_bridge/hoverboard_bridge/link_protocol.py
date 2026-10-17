"""UDP message format shared with the ESP32-S3 firmware (hover_core/link_protocol.h)."""
from dataclasses import dataclass
import struct

HEADER = struct.Struct('<BHI')
CMD_WHEELS = struct.Struct('<hh')
WHEEL_STATE = struct.Struct('<iihhhhBII')
IMU_STATE = struct.Struct('<9h')

MAX_DATAGRAM = 1024

MSG_CMD_WHEELS = 0x01
MSG_LIDAR_TX = 0x02
MSG_WHEEL_STATE = 0x10
MSG_IMU = 0x11
MSG_LIDAR_RX = 0x12
MSG_LOG = 0x13

STATUS_HOVER_OK = 0x01
STATUS_CMD_TIMEOUT = 0x02
STATUS_IMU_OK = 0x04


@dataclass
class Message:
    type: int
    seq: int
    stamp_ms: int
    body: bytes


@dataclass
class WheelState:
    ticks_l: int
    ticks_r: int
    rpm_l: int
    rpm_r: int
    bat_cv: int
    temp_dc: int
    status: int
    hover_ok: int
    hover_bad: int


@dataclass
class ImuState:
    acc: tuple
    gyro: tuple
    angle: tuple


def encode(msg_type, seq, stamp_ms, body=b''):
    return HEADER.pack(msg_type, seq & 0xFFFF, stamp_ms & 0xFFFFFFFF) + bytes(body)


def decode(datagram):
    if len(datagram) < HEADER.size:
        return None
    msg_type, seq, stamp_ms = HEADER.unpack_from(datagram)
    return Message(msg_type, seq, stamp_ms, bytes(datagram[HEADER.size:]))


def encode_cmd_wheels(seq, left, right):
    return encode(MSG_CMD_WHEELS, seq, 0, CMD_WHEELS.pack(left, right))


def parse_wheel_state(body):
    if len(body) != WHEEL_STATE.size:
        return None
    return WheelState(*WHEEL_STATE.unpack(body))


def parse_imu(body):
    if len(body) != IMU_STATE.size:
        return None
    values = IMU_STATE.unpack(body)
    return ImuState(values[0:3], values[3:6], values[6:9])
