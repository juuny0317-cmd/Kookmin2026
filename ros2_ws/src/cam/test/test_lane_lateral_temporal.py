import math

from cam.Lane_Detector import Lane_Detector
import numpy as np
import pytest
import rclpy
from std_msgs.msg import String


def prepare_node():
    node = Lane_Detector()
    node.bev_img_w = 640
    node.bev_img_h = 120
    node.bev_img_mid = 60
    node.lane = np.asarray([120.0, 320.0, 520.0])
    node.angle = 0.0
    node.prev_angle.clear()
    node.prev_angle.append(0.0)
    node.current_curve_state = 'Straight'
    node.last_curve_curvature_severity = 0.0
    return node


def set_curve_center(node, center_x):
    node.curve_heading_recovering = False
    node.estimated_curve_center_x = None
    node.bev_curve_points = (
        None
        if center_x is None
        else np.asarray([[center_x, 0.0]], dtype=np.float64)
    )


def set_fitted_curve_center(node, center_x):
    node.curve_heading_recovering = False
    node.estimated_curve_center_x = center_x
    node.bev_curve_points = None


def test_straight_rejects_abrupt_center_hypothesis():
    rclpy.init()
    node = prepare_node()
    try:
        predicted = node.lane.copy()

        node.refine_lane_with_candidates_and_prediction(
            [383.0], predicted, source_ns=1_000_000_000)

        assert math.isclose(node.lane[1], 320.0)
        assert node.last_lateral_debug['gate_rejected']
        assert node.last_lateral_debug['selected_candidate'] is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_straight_accepts_temporally_continuous_hypothesis():
    rclpy.init()
    node = prepare_node()
    try:
        predicted = node.lane.copy()

        node.refine_lane_with_candidates_and_prediction(
            [330.0], predicted, source_ns=1_000_000_000)

        assert math.isclose(node.lane[1], 327.0)
        assert not node.last_lateral_debug['gate_rejected']
        assert math.isclose(
            node.last_lateral_debug['selected_candidate'], 330.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_curve_relief_preserves_large_valid_lateral_motion():
    rclpy.init()
    node = prepare_node()
    try:
        node.current_curve_state = 'Curve'
        node.last_curve_curvature_severity = 1.0
        predicted = node.lane.copy()

        node.refine_lane_with_candidates_and_prediction(
            [383.0], predicted, source_ns=1_000_000_000)

        assert math.isclose(node.lane[1], 364.1)
        assert not node.last_lateral_debug['gate_rejected']
        assert not node.last_lateral_debug['rate_limited']
        assert math.isclose(
            node.last_lateral_debug['innovation_gate'],
            node.lateral_innovation_curve,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_center_curve_priority_moves_without_hough_confirmation():
    rclpy.init()
    node = prepare_node()
    try:
        node.estimated_curve_center_x = 365.0
        predicted = node.lane.copy()

        node.refine_lane_with_candidates_and_prediction(
            [383.0], predicted, source_ns=1_000_000_000)

        assert node.last_lateral_debug['gate_rejected']
        assert not node.last_lateral_debug['curve_confirmed']
        assert node.last_lateral_debug['rate_limited']
        assert node.lane[1] > 320.0
        assert node.lane[1] < 365.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_rate_guard_uses_actual_source_dt_on_straight():
    rclpy.init()
    node = prepare_node()
    try:
        node.last_lateral_source_ns = 1_000_000_000
        predicted = node.lane.copy()

        node.refine_lane_with_candidates_and_prediction(
            [350.0], predicted, source_ns=1_050_000_000)

        expected_delta = node.lateral_max_rate_straight * 0.05
        assert math.isclose(node.lane[1], 320.0 + expected_delta)
        assert node.last_lateral_debug['dt_valid']
        assert math.isclose(node.last_lateral_debug['dt_s'], 0.05)
        assert node.last_lateral_debug['rate_limited']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_no_candidates_follows_prediction_instead_of_infinite_hold():
    rclpy.init()
    node = prepare_node()
    try:
        predicted = node.lane + np.asarray([10.0, 10.0, 10.0])

        node.refine_lane_with_candidates_and_prediction(
            [], predicted, source_ns=1_000_000_000)

        assert math.isclose(node.lane[1], 330.0)
        assert not node.last_lateral_debug['gate_rejected']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_invalid_source_stamp_has_bounded_fallback_dt():
    rclpy.init()
    node = prepare_node()
    try:
        predicted = node.lane.copy()

        node.refine_lane_with_candidates_and_prediction(
            [350.0], predicted, source_ns=0)

        assert not node.last_lateral_debug['dt_valid']
        assert math.isclose(node.last_lateral_debug['dt_s'], 1.0 / 12.0)
        assert np.all(np.isfinite(node.lane))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_straight_outer_pair_uses_measured_midpoint():
    rclpy.init()
    node = prepare_node()
    try:
        node.lane += 60.0

        center = node.stabilize_straight_lateral_with_outer_pair(
            [62.0, 311.0, 578.0], temporal_center=380.0)

        assert math.isclose(center, 320.0)
        assert math.isclose(node.lane[1], 320.0)
        assert node.outer_pair_active
        assert not node.outer_pair_held
        assert math.isclose(node.outer_pair_width, 516.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_straight_outer_pair_hold_is_bounded():
    rclpy.init()
    node = prepare_node()
    try:
        node.stabilize_straight_lateral_with_outer_pair(
            [62.0, 311.0, 578.0], temporal_center=320.0)

        for expected_miss_count in range(1, 6):
            node.lane += 10.0
            center = node.stabilize_straight_lateral_with_outer_pair(
                [310.0], temporal_center=node.lane[1])
            assert math.isclose(center, 320.0)
            assert math.isclose(node.lane[1], 320.0)
            assert node.outer_pair_held
            assert node.outer_pair_miss_count == expected_miss_count

        node.lane += 10.0
        center = node.stabilize_straight_lateral_with_outer_pair(
            [310.0], temporal_center=node.lane[1])
        assert center is None
        assert math.isclose(node.lane[1], 330.0)
        assert not node.outer_pair_active
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_curve_bypasses_and_resets_straight_outer_pair():
    rclpy.init()
    node = prepare_node()
    try:
        node.stabilize_straight_lateral_with_outer_pair(
            [62.0, 311.0, 578.0], temporal_center=320.0)
        node.current_curve_state = 'Curve'
        node.lane += 35.0

        center = node.stabilize_straight_lateral_with_outer_pair(
            [62.0, 311.0, 578.0], temporal_center=355.0)

        assert center is None
        assert math.isclose(node.lane[1], 355.0)
        assert node.last_outer_pair_center is None
        assert node.outer_pair_miss_count == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_straight_curve_center_uses_ten_pixel_threshold_and_full_weight():
    rclpy.init()
    node = prepare_node()
    try:
        set_fitted_curve_center(node, 330.0)
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 320.0)

        set_fitted_curve_center(node, 331.0)
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 331.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_center_driving_curve_uses_earlier_stronger_correction():
    rclpy.init()
    node = prepare_node()
    try:
        node.current_curve_state = 'cUrVe'
        set_curve_center(node, 360.0)
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 320.0)

        set_curve_center(node, 380.0)
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 0.85 * 380.0 + 0.15 * 320.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('override_target_lane', ['go_left', 'go_right'])
def test_curve_lane_override_keeps_legacy_threshold_and_weight(
        override_target_lane):
    rclpy.init()
    node = prepare_node()
    try:
        node.current_curve_state = 'Curve'
        node.override_target_lane = override_target_lane
        set_curve_center(node, 380.0)
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 320.0)

        set_curve_center(node, 420.0)
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 0.60 * 420.0 + 0.40 * 320.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_curve_shortcut_keeps_legacy_threshold_and_weight():
    rclpy.init()
    node = prepare_node()
    try:
        node.current_curve_state = 'Curve'
        node.shortcut_path_active = True
        set_curve_center(node, 380.0)
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 320.0)

        set_curve_center(node, 420.0)
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 0.60 * 420.0 + 0.40 * 320.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_curve_center_dropout_holds_three_frames_then_uses_hough_fallback():
    rclpy.init()
    node = prepare_node()
    try:
        node.current_curve_state = 'Curve'
        set_curve_center(node, 420.0)
        node.force_correct_lane_with_curve()
        expected_center = 0.85 * 420.0 + 0.15 * 320.0
        assert math.isclose(node.lane[1], expected_center)
        assert math.isclose(node.last_valid_curve_center_x, 420.0)

        set_curve_center(node, None)
        for expected_miss_count in range(1, 4):
            node.lane = np.asarray([120.0, 320.0, 520.0])
            node.force_correct_lane_with_curve()
            assert math.isclose(node.lane[1], expected_center)
            assert node.curve_center_miss_count == expected_miss_count

        node.lane = np.asarray([120.0, 320.0, 520.0])
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 320.0)
        assert node.curve_center_miss_count == 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_leaving_curve_clears_curve_center_hold_state():
    rclpy.init()
    node = prepare_node()
    try:
        node.current_curve_state = 'Curve'
        set_curve_center(node, 420.0)
        node.force_correct_lane_with_curve()
        set_curve_center(node, None)
        node.force_correct_lane_with_curve()
        assert node.curve_center_miss_count == 1

        node.current_curve_state = 'Straight'
        node.lane = np.asarray([120.0, 320.0, 520.0])
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 320.0)
        assert node.last_valid_curve_center_x is None
        assert node.curve_center_miss_count == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    'invalid_center_x',
    [np.nan, np.inf, -np.inf, -1.0, 640.0],
)
def test_invalid_curve_centers_are_never_stored_or_used(invalid_center_x):
    rclpy.init()
    node = prepare_node()
    try:
        node.current_curve_state = 'Curve'
        set_curve_center(node, invalid_center_x)
        node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 320.0)
        assert node.last_valid_curve_center_x is None
        assert node.curve_center_miss_count == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_curve_center_correction_still_passes_through_lateral_rate_guard():
    rclpy.init()
    node = prepare_node()
    try:
        node.current_curve_state = 'Curve'
        node.last_curve_curvature_severity = 1.0
        node.last_lateral_source_ns = 1_000_000_000
        set_curve_center(node, 420.0)
        predicted = node.lane.copy()

        node.refine_lane_with_candidates_and_prediction(
            [], predicted, source_ns=1_050_000_000)

        expected_allowed_delta = (
            node.lateral_max_rate_straight
            * node.lateral_innovation_curve
            / node.lateral_innovation_straight
            * 0.05
        )
        assert node.last_lateral_debug['rate_limited']
        assert math.isclose(
            node.last_lateral_debug['allowed_delta'],
            expected_allowed_delta,
        )
        assert math.isclose(node.lane[1], 320.0 + expected_allowed_delta)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_valid_straight_yolo_blocks_outer_pair_override():
    rclpy.init()
    node = prepare_node()
    try:
        set_fitted_curve_center(node, 380.0)
        predicted = node.lane.copy()

        node.refine_lane_with_candidates_and_prediction(
            [62.0, 311.0, 578.0],
            predicted,
            source_ns=1_000_000_000,
        )

        expected_delta = node.lateral_max_rate_straight / 12.0
        assert math.isclose(node.lane[1], 320.0 + expected_delta)
        assert not node.outer_pair_active
        assert node.priority_curve_center_valid
        assert node.last_lateral_debug['rate_limited']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_missing_straight_yolo_uses_outer_pair_fallback():
    rclpy.init()
    node = prepare_node()
    try:
        set_fitted_curve_center(node, None)
        node.lane += 60.0
        predicted = node.lane.copy()

        node.refine_lane_with_candidates_and_prediction(
            [62.0, 311.0, 578.0],
            predicted,
            source_ns=1_000_000_000,
        )

        expected_delta = node.lateral_max_rate_straight / 12.0
        assert math.isclose(node.lane[1], 380.0 - expected_delta)
        assert node.outer_pair_active
        assert not node.priority_curve_center_valid
        assert node.last_lateral_debug['rate_limited']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_reset_enables_curve_state_recovery_yolo_priority():
    rclpy.init()
    node = prepare_node()
    try:
        node.current_curve_state = 'Curve'
        node.override_target_lane = 'go_left'
        node.trigger_callback(String(data='reset'))
        set_fitted_curve_center(node, 380.0)

        priority_valid = node.force_correct_lane_with_curve()

        assert node.obstacle_recovery_active()
        assert priority_valid
        assert math.isclose(node.lane[1], 380.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_large_yolo_jump_requires_two_consistent_frames():
    rclpy.init()
    node = prepare_node()
    try:
        set_fitted_curve_center(node, 320.0)
        assert node.force_correct_lane_with_curve()

        set_fitted_curve_center(node, 500.0)
        assert not node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 320.0)

        set_fitted_curve_center(node, 505.0)
        assert node.force_correct_lane_with_curve()
        assert math.isclose(node.lane[1], 505.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
