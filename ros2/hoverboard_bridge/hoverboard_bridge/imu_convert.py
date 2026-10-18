"""WT901 raw values to SI units (default ranges: 16 g, 2000 deg/s)."""
import math

GRAVITY = 9.80665
ACC_RANGE_G = 16.0
GYRO_RANGE_DEG_S = 2000.0


def acc_to_m_s2(raw):
    return raw / 32768.0 * ACC_RANGE_G * GRAVITY


def gyro_to_rad_s(raw):
    return math.radians(raw / 32768.0 * GYRO_RANGE_DEG_S)


def angle_to_rad(raw):
    return math.radians(raw / 32768.0 * 180.0)


def euler_to_quaternion(roll, pitch, yaw):
    """ZYX Euler angles to quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)
