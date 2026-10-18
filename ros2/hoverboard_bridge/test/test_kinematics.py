import math

import pytest

from hoverboard_bridge.kinematics import DiffDriveParams, OdometryIntegrator, TickContinuity, twist_to_commands

P = DiffDriveParams(wheel_radius=0.0825, wheel_separation=0.40, ticks_per_rev=90, n_mot_max=200.0)


def test_straight_and_rotation_commands():
    assert twist_to_commands(P, 0.5, 0.0) == (289, 289)
    assert twist_to_commands(P, 0.0, 1.0) == (-116, 116)


def test_commands_are_clamped():
    assert twist_to_commands(P, 10.0, 0.0) == (1000, 1000)


def test_odometry_straight_one_revolution():
    odom = OdometryIntegrator(P)
    odom.update(0, 0)
    odom.update(90, 90)
    assert odom.x == pytest.approx(2 * math.pi * 0.0825)
    assert odom.y == pytest.approx(0.0, abs=1e-9)
    assert odom.theta == pytest.approx(0.0, abs=1e-9)


def test_odometry_rotation_in_place():
    odom = OdometryIntegrator(P)
    odom.update(0, 0)
    odom.update(-45, 45)
    assert odom.theta == pytest.approx(2 * math.pi * 0.0825 / 2 * 2 / 0.40)
    assert odom.x == pytest.approx(0.0, abs=1e-9)


def test_non_finite_twist_gives_zero_commands():
    assert twist_to_commands(P, float('nan'), 0.0) == (0, 0)
    assert twist_to_commands(P, 0.0, float('inf')) == (0, 0)
    assert twist_to_commands(P, 1e308, 0.0) == (0, 0)


def test_saturation_keeps_curvature():
    left, right = twist_to_commands(P, 2.0, 1.0)
    assert right == 1000
    assert left == round(1000 * 1.8 / 2.2) == 818


def test_invalid_params_are_rejected():
    with pytest.raises(ValueError, match="wheel_radius"):
        DiffDriveParams(wheel_radius=0.0, wheel_separation=0.40, ticks_per_rev=90, n_mot_max=200.0)


def test_tick_continuity_passthrough():
    tc = TickContinuity()
    assert tc.update(1000, 10, 20) == (10, 20, False)
    assert tc.update(1050, 15, 25) == (15, 25, False)


def test_tick_continuity_reboot_keeps_ticks_continuous():
    tc = TickContinuity()
    tc.update(59000, 400, 400)
    assert tc.update(60000, 500, 500) == (500, 500, False)
    assert tc.update(100, 3, 3) == (503, 503, True)
    assert tc.update(150, 5, 4) == (505, 504, False)


def test_tick_continuity_small_backward_step_is_not_reboot():
    tc = TickContinuity()
    tc.update(1000, 10, 10)
    assert tc.update(980, 11, 11) == (11, 11, False)
