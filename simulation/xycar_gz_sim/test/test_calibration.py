import math

import pytest

from xycar_gz_sim.calibration import PiecewiseLinear, XycarCalibration


@pytest.fixture()
def calibration() -> XycarCalibration:
    return XycarCalibration(
        [-0.571108752, -0.427110352, -0.299415242, 0.0, 0.145952652, 0.263676395, 0.408565539],
        [40.0, 30.0, 20.0, 7.0, -20.0, -30.0, -40.0],
        [-40.0, -30.0, -20.0, 7.0, 20.0, 30.0, 40.0],
        [1 / 0.820, 1 / 1.315, 1 / 2.415, 0.0, -1 / 1.150, -1 / 0.780, -1 / 0.5525],
        [4, 5, 6, 7, 8, 9, 10, 12, 15, 20, 25],
        [0.399, 0.508, 0.605, 0.694, 0.761, 0.986, 1.020, 1.247, 1.511, 1.938, 2.222],
        0.355,
        0.250,
    )


def test_piecewise_linear_interpolation_and_clamping() -> None:
    table = PiecewiseLinear([0, 10, 20], [0, 100, 200])
    assert table(-1) == 0
    assert table(5) == 50
    assert table(25) == 200


@pytest.mark.parametrize(
    'team_angle,physical_raw',
    [(-47, -40), (-37, -30), (-27, -20), (0, 7), (13, 20), (23, 30), (33, 40)],
)
def test_team_centered_angle_maps_to_measured_physical_raw(
    team_angle: float, physical_raw: float
) -> None:
    table = PiecewiseLinear(
        [-47, -37, -27, 0, 13, 23, 33],
        [-40, -30, -20, 7, 20, 30, 40],
    )
    assert table(team_angle) == pytest.approx(physical_raw)


def test_logical_zero_maps_to_physical_raw_plus_seven(calibration: XycarCalibration) -> None:
    assert calibration.raw_from_logical(0.0) == pytest.approx(7.0)


@pytest.mark.parametrize('command,speed', [(0, 0.0), (5, 0.508), (10, 1.020), (15, 1.511)])
def test_speed_table(calibration: XycarCalibration, command: float, speed: float) -> None:
    assert calibration.speed_from_command(command) == pytest.approx(speed)


@pytest.mark.parametrize(
    'raw,radius', [(20, 1.150), (30, 0.780), (40, 0.5525), (-20, 2.415), (-30, 1.315), (-40, 0.820)])
def test_turn_radius_table(calibration: XycarCalibration, raw: float, radius: float) -> None:
    assert abs(1.0 / calibration.curvature_from_raw(raw)) == pytest.approx(radius)


def test_ackermann_uses_different_inner_and_outer_angles(calibration: XycarCalibration) -> None:
    right = calibration.ackermann_angles(calibration.curvature_from_raw(20.0))
    assert right.left_rad < 0.0 and right.right_rad < 0.0
    assert abs(right.right_rad) > abs(right.left_rad)
    left = calibration.ackermann_angles(calibration.curvature_from_raw(-20.0))
    assert left.left_rad > 0.0 and left.right_rad > 0.0
    assert left.left_rad > left.right_rad


def test_raw_plus_40_exposes_angle_measurement_conflict(calibration: XycarCalibration) -> None:
    angles = calibration.ackermann_angles(calibration.curvature_from_raw(40.0))
    assert math.degrees(abs(angles.right_rad)) > 39.0
