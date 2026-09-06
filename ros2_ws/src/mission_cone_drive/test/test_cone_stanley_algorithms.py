import math

from mission_cone_drive.cone_stanley_algorithms import (
    closest_path_reference,
    compute_stanley_control,
)


def test_centered_straight_path_produces_zero_steering():
    path = [(0.4, 0.0), (0.8, 0.0), (1.2, 0.0)]

    result = compute_stanley_control(path, speed_mps=0.25)

    assert abs(result.cross_track_error_m) < 1e-6
    assert abs(result.path_heading_rad) < 1e-6
    assert abs(result.servo_command) < 1e-6


def test_path_on_vehicle_left_commands_left_steering():
    path = [(0.4, 0.20), (0.8, 0.20), (1.2, 0.20)]

    result = compute_stanley_control(path, speed_mps=0.25)

    assert result.cross_track_error_m > 0.0
    assert result.servo_command < 0.0


def test_path_on_vehicle_right_commands_right_steering():
    path = [(0.4, -0.20), (0.8, -0.20), (1.2, -0.20)]

    result = compute_stanley_control(path, speed_mps=0.25)

    assert result.cross_track_error_m < 0.0
    assert result.servo_command > 0.0


def test_left_bending_path_adds_left_heading_correction():
    path = [
        (0.4, 0.0),
        (0.7, 0.05),
        (1.0, 0.20),
        (1.3, 0.45),
    ]

    result = compute_stanley_control(
        path,
        speed_mps=0.25,
        heading_window_m=0.40,
    )

    assert result.path_heading_rad > math.radians(5.0)
    assert result.servo_command < 0.0


def test_reference_ignores_duplicate_and_nonfinite_points():
    path = [
        (0.4, 0.0),
        (0.4, 0.0),
        (float('nan'), 0.0),
        (0.8, 0.1),
        (1.2, 0.2),
    ]

    x, y, heading, _ = closest_path_reference(
        path,
        control_x_m=0.33,
    )

    assert math.isfinite(x)
    assert math.isfinite(y)
    assert math.isfinite(heading)
