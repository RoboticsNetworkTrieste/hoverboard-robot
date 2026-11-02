import math

from hoverboard_bridge.scan_resample import resample

INF = math.inf


def test_points_land_in_their_sector():
    # Four points a quarter turn apart, four output sectors starting at -pi
    out = resample(0.25, math.pi / 2, [1.0, 2.0, 3.0, 4.0], 0.02, 25.0, beams=4)
    assert out == [3.0, 4.0, 1.0, 2.0]


def test_closest_valid_point_wins_and_invalid_points_are_skipped():
    out = resample(0.1, 0.01, [5.0, 1.5, 0.0, INF, float('nan'), 30.0], 0.02, 25.0, beams=4)
    assert out == [INF, INF, 1.5, INF]


def test_output_size_does_not_depend_on_input_size():
    for count in (600, 640):
        out = resample(0.0, 2 * math.pi / count, [1.0] * count, 0.02, 25.0, beams=360)
        assert len(out) == 360
        assert all(r == 1.0 for r in out)  # Input denser than the output: every sector filled


def test_points_in_excluded_sectors_are_dropped():
    # Same input as the first test: the points at 0.25 and 0.25 + pi/2 rad fall in the excluded sector
    out = resample(0.25, math.pi / 2, [1.0, 2.0, 3.0, 4.0], 0.02, 25.0, beams=4, excluded=[(0.0, 2.0)])
    assert out == [3.0, 4.0, INF, INF]


def test_excluded_sectors_use_the_minus_pi_pi_range():
    # Input angle 3*pi/2 is -pi/2 once normalized
    out = resample(3 * math.pi / 2, 0.01, [1.0], 0.02, 25.0, beams=4, excluded=[(-2.0, -1.0)])
    assert out == [INF, INF, INF, INF]
