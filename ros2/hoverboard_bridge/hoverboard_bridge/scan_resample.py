"""Resamples a LaserScan with a varying number of beams onto a fixed angular grid."""
import math


def resample(angle_min, angle_increment, ranges, range_min, range_max, beams, excluded=()):
    """Returns `beams` ranges covering [-pi, pi) in steps of 2*pi/beams.

    Each output beam keeps the closest valid input point in its sector; empty sectors are inf.
    Points inside the `excluded` (min, max) sectors, in radians within [-pi, pi), are dropped.
    """
    out = [math.inf] * beams
    step = 2.0 * math.pi / beams
    for i, r in enumerate(ranges):
        if not range_min <= r <= range_max:  # Also rejects NaN and inf
            continue
        angle = (angle_min + i * angle_increment + math.pi) % (2.0 * math.pi)
        if any(low <= angle - math.pi <= high for low, high in excluded):
            continue
        k = int(angle / step) % beams
        out[k] = min(out[k], r)
    return out
