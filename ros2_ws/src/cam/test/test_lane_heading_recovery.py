from collections import deque
import math

from cam.Lane_Detector import Lane_Detector
from custom_interfaces.msg import Curve
from geometry_msgs.msg import Point
import numpy as np
import rclpy


def make_curve(slope):
    curve = Curve()
    for y in (70.0, 100.0, 130.0, 155.0, 180.0, 210.0):
        point = Point()
        point.x = 160.0 + slope * (y - 160.0)
        point.y = y
        curve.points.append(point)
    return curve


def make_quadratic_curve(curvature_strength):
    curve = Curve()
    for y in (70.0, 100.0, 130.0, 155.0, 180.0, 210.0):
        normalized_y = (y - 160.0) / 240.0
        point = Point()
        point.x = (
            160.0
            + 320.0 * curvature_strength * normalized_y * normalized_y
        )
        point.y = y
        curve.points.append(point)
    return curve


def make_bev_line(angle_deg):
    angle = math.radians(angle_deg)
    bottom_x = 220
    bottom_y = 110
    top_y = 10
    top_x = int(round(bottom_x + math.tan(angle) * (bottom_y - top_y)))
    return np.asarray([[[bottom_x, bottom_y, top_x, top_y]]], dtype=np.int32)


def test_curve_heading_preserves_near_field_direction_sign():
    rclpy.init()
    node = Lane_Detector()
    try:
        node.get_birds_eye_view(np.zeros((240, 320), dtype=np.uint8))
        positive = node.estimate_curve_heading(make_curve(-0.35))
        positive_center = node.estimated_curve_center_x
        negative = node.estimate_curve_heading(make_curve(0.35))

        assert positive is not None and positive > math.radians(5.0)
        assert positive_center is not None
        assert 0.0 <= positive_center < 640.0
        assert negative is not None and negative < math.radians(-5.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_curvature_shortens_heading_preview():
    rclpy.init()
    node = Lane_Detector()
    try:
        node.get_birds_eye_view(np.zeros((240, 320), dtype=np.uint8))
        node.curvature_history.clear()
        straight_heading = node.estimate_curve_heading(make_curve(0.0))
        straight_preview = node.last_heading_preview_ratio

        node.curvature_history.clear()
        curve_heading = node.estimate_curve_heading(
            make_quadratic_curve(2.0))
        curve_preview = node.last_heading_preview_ratio

        assert straight_heading is not None
        assert curve_heading is not None
        assert math.isclose(
            straight_preview,
            node.heading_preview_max_ratio,
            rel_tol=1e-3,
        )
        assert node.heading_preview_min_ratio <= curve_preview
        assert curve_preview < straight_preview
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_curvature_deadzone_ignores_small_curvature_for_heading_preview():
    rclpy.init(args=[
        '--ros-args',
        '-p',
        'heading_curvature_deadzone:=100.0',
    ])
    node = Lane_Detector()
    try:
        node.get_birds_eye_view(np.zeros((240, 320), dtype=np.uint8))
        heading = node.estimate_curve_heading(make_quadratic_curve(2.0))

        assert heading is not None
        assert node.last_curve_curvature > 0.0
        assert math.isclose(node.last_curve_curvature_severity, 0.0)
        assert math.isclose(
            node.last_heading_preview_ratio,
            node.heading_preview_max_ratio,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_tight_curve_uses_curve_only_fit_tolerance():
    rclpy.init()
    node = Lane_Detector()
    try:
        node.get_birds_eye_view(np.zeros((240, 320), dtype=np.uint8))
        curve = Curve()
        curve.state = 'Curve'
        for x, y in (
            (269.0, 239.0),
            (235.0, 194.0),
            (211.0, 162.0),
            (186.0, 138.0),
            (139.0, 126.0),
            (86.0, 122.0),
            (0.0, 116.0),
        ):
            point = Point()
            point.x = x
            point.y = y
            curve.points.append(point)

        heading = node.estimate_curve_heading(curve)

        assert heading is not None
        assert heading < math.radians(-15.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_offscreen_curve_can_supply_recovery_heading():
    rclpy.init()
    node = Lane_Detector()
    try:
        node.get_birds_eye_view(np.zeros((240, 320), dtype=np.uint8))
        curve = Curve()
        curve.state = 'Curve'
        for x, y in (
            (-623.2, 239.0),
            (118.0, 130.0),
            (152.0, 125.0),
            (319.0, 100.44),
        ):
            point = Point()
            point.x = x
            point.y = y
            curve.points.append(point)

        heading = node.estimate_curve_heading(curve)

        assert heading is not None
        assert heading > math.radians(15.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_control_heading_is_smoother_on_straight_and_faster_on_curve():
    rclpy.init()
    node = Lane_Detector()
    try:
        # Keep this below heading_max_step so the test isolates adaptive alpha.
        target = math.radians(10.0)

        node.control_heading = 0.0
        node.control_heading_initialized = True
        node.last_curve_curvature_severity = 0.0
        straight_result = node.update_control_heading(target)

        node.control_heading = 0.0
        node.control_heading_initialized = True
        node.last_curve_curvature_severity = 1.0
        curve_result = node.update_control_heading(target)

        assert math.isclose(
            straight_result,
            target * node.heading_filter_alpha_straight,
        )
        assert math.isclose(
            curve_result,
            target * node.heading_filter_alpha_curve,
        )
        assert curve_result > straight_result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_control_heading_holds_short_curve_dropouts_before_hough_fallback():
    rclpy.init()
    node = Lane_Detector()
    try:
        node.angle = math.radians(-20.0)
        node.control_heading = 0.0
        node.control_heading_initialized = True
        node.last_curve_curvature_severity = 0.5

        valid_heading = math.radians(20.0)
        node.update_control_heading(valid_heading)
        after_valid = node.control_heading

        for _ in range(node.heading_curve_hold_frames):
            node.update_control_heading(None)
            assert node.control_heading_source == 'curve_hold'
            assert node.control_heading >= after_valid
            after_valid = node.control_heading

        node.update_control_heading(None)
        assert node.control_heading_source == 'hough'
        assert node.control_heading < after_valid
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_control_heading_limits_single_frame_direction_reversal():
    rclpy.init()
    node = Lane_Detector()
    try:
        node.control_heading = math.radians(-30.0)
        node.control_heading_initialized = True
        node.last_curve_curvature_severity = 1.0

        result = node.update_control_heading(math.radians(40.0))

        assert math.isclose(
            result,
            math.radians(-18.0),
            abs_tol=1e-9,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_center_curve_unlocks_opposite_hough_direction_after_confirmation():
    rclpy.init()
    node = Lane_Detector()
    try:
        node.bev_img_w = 640
        node.bev_img_h = 120
        node.bev_img_mid = 60
        node.angle = math.radians(-50.0)
        node.prev_angle = deque([node.angle], maxlen=3)
        node.curve_heading_history.clear()
        node.curve_heading_disagreement_count = 0
        node.curve_heading_confirm_frames = 2
        node.curve_heading_disagreement = math.radians(25.0)
        node.curve_heading_recovery_weight = 0.65

        line = make_bev_line(40.0)
        curve_heading = math.radians(40.0)

        first_positions = node.filter_lines_by_angle(
            line, show=False, curve_heading=curve_heading)
        first_angle = node.angle
        second_positions = node.filter_lines_by_angle(
            line, show=False, curve_heading=curve_heading)
        second_angle = node.angle
        node.filter_lines_by_angle(
            line, show=False, curve_heading=curve_heading)
        third_angle = node.angle

        assert first_positions == []
        assert math.isclose(first_angle, math.radians(-50.0))
        assert second_positions
        assert second_angle > 0.0
        assert third_angle > second_angle
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_no_curve_heading_keeps_normal_hough_filter_behavior():
    rclpy.init()
    node = Lane_Detector()
    try:
        node.bev_img_w = 640
        node.bev_img_h = 120
        node.bev_img_mid = 60
        node.angle = 0.0
        node.prev_angle = deque([0.0], maxlen=3)

        positions = node.filter_lines_by_angle(
            make_bev_line(10.0),
            show=False,
            curve_heading=None,
        )

        assert positions
        assert 0.0 < node.angle < math.radians(10.0)
        assert node.curve_heading_disagreement_count == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
