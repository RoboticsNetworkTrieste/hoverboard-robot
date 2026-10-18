"""Differential drive math for the hoverboard (hub motors: motor rpm = wheel rpm)."""
from dataclasses import dataclass
import math

CMD_FULL_SCALE = 1000.0  # firmware command that maps to N_MOT_MAX


@dataclass
class DiffDriveParams:
    wheel_radius: float      # [m]
    wheel_separation: float  # [m] distance between wheel contact points
    ticks_per_rev: int       # Hall transitions per wheel revolution
    n_mot_max: float         # [rpm] firmware N_MOT_MAX
    max_cmd: int = 1000

    def __post_init__(self):
        for name in ('wheel_radius', 'wheel_separation', 'ticks_per_rev', 'n_mot_max', 'max_cmd'):
            if not getattr(self, name) > 0:  # Also rejects NaN
                raise ValueError(f"{name} must be > 0")


def rpm_to_rad_s(rpm):
    return rpm * 2.0 * math.pi / 60.0


def _speed_to_cmd(p, speed):
    """Unclamped firmware command (float) for a wheel linear speed [m/s]."""
    rpm = speed / p.wheel_radius * 60.0 / (2.0 * math.pi)
    return rpm * CMD_FULL_SCALE / p.n_mot_max


def twist_to_commands(p, linear, angular):
    """Returns (left, right) firmware commands for a body twist.

    Non-finite input or overflow gives (0, 0). On saturation both wheels are scaled
    together to preserve the requested curvature.
    """
    if not (math.isfinite(linear) and math.isfinite(angular)):
        return 0, 0
    half = angular * p.wheel_separation / 2.0
    left = _speed_to_cmd(p, linear - half)
    right = _speed_to_cmd(p, linear + half)
    peak = max(abs(left), abs(right))
    if not math.isfinite(peak):
        return 0, 0
    if peak > p.max_cmd:
        left *= p.max_cmd / peak
        right *= p.max_cmd / peak
    return round(left), round(right)


class OdometryIntegrator:
    """Integrates cumulative wheel ticks into a planar pose."""

    def __init__(self, p):
        self.p = p
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev = None

    def ticks_to_angle(self, ticks):
        return ticks * 2.0 * math.pi / self.p.ticks_per_rev

    def update(self, ticks_l, ticks_r):
        if self._prev is None:
            self._prev = (ticks_l, ticks_r)
            return
        d_left = self.ticks_to_angle(ticks_l - self._prev[0]) * self.p.wheel_radius
        d_right = self.ticks_to_angle(ticks_r - self._prev[1]) * self.p.wheel_radius
        self._prev = (ticks_l, ticks_r)
        distance = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.p.wheel_separation
        heading = self.theta + d_theta / 2.0
        self.x += distance * math.cos(heading)
        self.y += distance * math.sin(heading)
        self.theta = math.atan2(math.sin(self.theta + d_theta), math.cos(self.theta + d_theta))


class TickContinuity:
    """Keeps cumulative ticks continuous across ESP reboots (ESP clock going backwards)."""

    def __init__(self, reboot_margin_ms=500):
        # stamp_ms wraps after ~49.7 days; a wrap would be taken as a reboot
        self.reboot_margin_ms = reboot_margin_ms
        self._last_stamp_ms = None
        self._last = (0, 0)
        self._offset = (0, 0)

    def update(self, stamp_ms, ticks_l, ticks_r):
        """Returns (continuous_l, continuous_r, rebooted)."""
        rebooted = (self._last_stamp_ms is not None
                    and stamp_ms < self._last_stamp_ms - self.reboot_margin_ms)
        if rebooted:
            self._offset = self._last  # New counters restart from 0
        self._last_stamp_ms = stamp_ms
        self._last = (ticks_l + self._offset[0], ticks_r + self._offset[1])
        return self._last[0], self._last[1], rebooted
