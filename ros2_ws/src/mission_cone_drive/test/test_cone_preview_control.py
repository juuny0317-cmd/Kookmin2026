from mission_cone_drive.cone_preview_control import (
    compute_cone_preview_control,
    curvature_speed_command,
    limit_raw_angle_for_downstream_trim,
    path_curvature_metrics,
    smooth_steering_command,
)
import pytest


def test_straight_path_preserves_trimmed_center_and_maximum_speed():
    path = [(0.4, 0.0), (0.8, 0.0), (1.2, 0.0)]

    result = compute_cone_preview_control(path, speed_mps=0.48)
    speed = curvature_speed_command(
        result.curvature_risk_per_m,
        result.final_angle_command,
    )

    assert result.raw_angle_command == pytest.approx(0.0)
    assert result.final_angle_command == pytest.approx(-6.0)
    assert speed == pytest.approx(8.0)


def test_trim_aware_limits_reach_both_physical_steering_endpoints():
    negative_raw, negative_final = limit_raw_angle_for_downstream_trim(-100.0)
    positive_raw, positive_final = limit_raw_angle_for_downstream_trim(100.0)

    assert negative_raw == pytest.approx(-36.0)
    assert negative_final == pytest.approx(-42.0)
    assert positive_raw == pytest.approx(48.0)
    assert positive_final == pytest.approx(42.0)


def test_steering_filter_limits_one_frame_reversal():
    first = smooth_steering_command(
        -36.0, 48.0, max_angle_step=12.0, smoothing_alpha=0.65)
    second = smooth_steering_command(
        first, 48.0, max_angle_step=12.0, smoothing_alpha=0.65)

    assert first == pytest.approx(-24.0)
    assert second == pytest.approx(-12.0)


def test_high_curvature_or_full_steering_selects_minimum_speed():
    assert curvature_speed_command(4.0, -42.0) == pytest.approx(4.0)
    assert curvature_speed_command(4.0, 42.0) == pytest.approx(4.0)


def test_preview_curvature_slows_before_a_bend():
    straight = [(0.4, 0.0), (0.8, 0.0), (1.2, 0.0), (1.6, 0.0)]
    bend = [
        (0.4, 0.0),
        (0.7, 0.02),
        (0.95, 0.18),
        (1.12, 0.44),
        (1.20, 0.75),
    ]

    straight_result = compute_cone_preview_control(straight, speed_mps=0.48)
    bend_result = compute_cone_preview_control(bend, speed_mps=0.48)
    straight_speed = curvature_speed_command(
        straight_result.curvature_risk_per_m,
        straight_result.final_angle_command,
    )
    bend_speed = curvature_speed_command(
        bend_result.curvature_risk_per_m,
        bend_result.final_angle_command,
    )

    assert bend_result.curvature_risk_per_m > 0.2
    assert bend_speed < straight_speed
    assert 4.0 <= bend_speed <= 8.0


def test_s_path_contains_opposite_signed_curvature():
    path = [
        (0.35, 0.00),
        (0.55, 0.15),
        (0.78, 0.22),
        (1.00, 0.02),
        (1.20, -0.20),
        (1.42, -0.05),
        (1.62, 0.18),
    ]

    first = path_curvature_metrics(
        path, steering_preview_m=0.25, speed_preview_m=0.50)
    later_path = [(x - 0.90, y) for x, y in path[3:]]
    second = path_curvature_metrics(
        later_path, steering_preview_m=0.25, speed_preview_m=0.50)

    assert first[1] * second[1] < 0.0
