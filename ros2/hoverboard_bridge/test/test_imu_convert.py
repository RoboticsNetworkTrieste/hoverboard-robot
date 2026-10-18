import math

import pytest

from hoverboard_bridge import imu_convert


def test_raw_scaling():
    assert imu_convert.acc_to_m_s2(2048) == pytest.approx(9.80665)
    assert imu_convert.gyro_to_rad_s(16384) == pytest.approx(math.radians(1000.0))
    assert imu_convert.angle_to_rad(-16384) == pytest.approx(-math.pi / 2)


def test_yaw_quaternion():
    x, y, z, w = imu_convert.euler_to_quaternion(0.0, 0.0, math.pi / 2)
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert (z, w) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))
