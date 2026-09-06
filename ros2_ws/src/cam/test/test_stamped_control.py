import math

from cam.Integrated_Stanley_Controller import IntegratedStanleyController
from custom_interfaces.msg import XycarState
import rclpy
from std_msgs.msg import Float32, String


def test_stanley_formula_keeps_fixed_softening_denominator():
    rclpy.init()
    node = IntegratedStanleyController()
    try:
        servo, heading, cte_term = node.compute_steering_angle_stanley(
            lane_center_x=420.0,
            lane_angle_rad=0.1,
            k=1.2,
        )
        expected_cte = (420.0 - 320.0) * (1.9 / 650.0)
        expected_term = math.atan2(1.2 * expected_cte, 3.0)
        expected_servo = math.degrees(0.3 * 0.1 + expected_term) * 5.0 / 3.0
        assert math.isclose(heading, 0.1)
        assert math.isclose(cte_term, expected_term)
        assert math.isclose(servo, expected_servo)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_obstacle_profile_is_a_hard_cap_over_curve_speed_policy():
    rclpy.init(args=[
        '--ros-args',
        '-p',
        'speed_straight:=16.0',
        '-p',
        'curve_strong_speed:=12.0',
        '-p',
        'curve_medium_speed:=13.0',
        '-p',
        'curve_mild_speed:=14.0',
    ])
    node = IntegratedStanleyController()
    try:
        node.obstacle_speed_limit_callback(Float32(data=7.0))
        node.current_speed = 16.0
        state = XycarState()
        state.drive_mode = 'center'
        state.target_point.x = 320.0
        state.target_point.z = 0.0
        metadata = node.default_control_metadata()
        metadata.update({
            'curve_state': 'Curve',
            'signed_curvature_hint': 0.8,
            'curvature_severity': 0.8,
        })
        published = []
        node.publish_motor = lambda angle, speed, **kwargs: published.append(
            (angle, speed))

        node.publish_control_from_state(
            state,
            source_ns=1_000_000_000,
            control_metadata=metadata,
        )

        assert published
        assert math.isclose(published[-1][1], 7.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_same_source_stamp_does_not_advance_steering_rate_limit():
    rclpy.init()
    node = IntegratedStanleyController()
    try:
        state = XycarState()
        state.drive_mode = 'center'
        state.target_point.x = 640.0
        state.target_point.z = 0.0

        node.publish_control_from_state(state, source_ns=123)
        first_angle = node.last_steering_deg
        first_speed = node.current_speed
        node.publish_control_from_state(state, source_ns=123)
        repeated_angle = node.last_steering_deg
        repeated_speed = node.current_speed
        node.publish_control_from_state(state, source_ns=124)
        next_source_angle = node.last_steering_deg
        next_source_speed = node.current_speed

        assert math.isclose(first_angle, 15.0)
        assert math.isclose(repeated_angle, first_angle)
        assert math.isclose(repeated_speed, first_speed)
        assert next_source_angle > repeated_angle
        assert next_source_speed > repeated_speed
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_curvature_rate_limit_preserves_clamp_debug_and_speed_path():
    """Keep clamp, debug, and longitudinal calls around the new limiter."""
    rclpy.init(args=[
        '--ros-args',
        '-p',
        'curvature_steering_rate_enabled:=true',
        '-p',
        'nominal_source_rate_hz:=15.0',
    ])
    node = IntegratedStanleyController()
    try:
        state = XycarState()
        state.drive_mode = 'center'
        state.target_point.x = 640.0
        state.target_point.z = 4.0
        metadata = node.default_control_metadata()
        metadata.update({
            'curve_state': 'Curve',
            'heading_source': 'curve',
            'signed_curvature_hint': 0.75,
            'preview_signed_curvature_hint': -0.75,
            'curvature_severity': 0.60,
        })
        node.last_steering_deg = 95.0
        debug_calls = []
        node.publish_debug = lambda **kwargs: debug_calls.append(kwargs)
        node.publish_motor = lambda *args, **kwargs: None

        node.publish_control_from_state(
            state,
            source_ns=1_000_000_000,
            control_metadata=metadata,
        )

        steering_limit = node.curvature_steering_limiter.last_result
        assert steering_limit.max_delta == 20.0
        assert steering_limit.rate_limited
        assert node.last_steering_deg == 100.0
        assert debug_calls[-1]['rate_limited']
        assert (
            debug_calls[-1]['curve_speed_decision'].target_speed
            == debug_calls[-1]['target_speed']
        )
        assert debug_calls[-1]['target_speed'] == 10.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_stop_and_stale_input_reset_curvature_limiter_memory():
    """Reset retained steering authority on STOP and stale input."""
    rclpy.init(args=[
        '--ros-args',
        '-p',
        'curvature_steering_rate_enabled:=true',
    ])
    node = IntegratedStanleyController()
    try:
        node.curvature_steering_limiter.limit(
            raw_angle=80.0,
            last_angle=0.0,
            source_dt_s=1.0 / 15.0,
            current_risk_hint=0.60,
        )
        assert node.curvature_steering_limiter.effective_risk == 0.60

        node.mission_mode_callback(String(data='STOP'))
        assert node.curvature_steering_limiter.effective_risk == 0.0

        node.curvature_steering_limiter.limit(
            raw_angle=80.0,
            last_angle=0.0,
            source_dt_s=1.0 / 15.0,
            current_risk_hint=0.60,
        )
        node.is_active = True
        node.latest_xycar_state = None
        node.run_control_once()
        assert node.curvature_steering_limiter.effective_risk == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cte_gain_schedule_preserves_low_speed_tuning():
    rclpy.init()
    node = IntegratedStanleyController()
    try:
        node.current_speed = 4.0
        node.curve_state = 'Straight'
        ratio = node.update_scheduled_cross_track_gain()

        assert math.isclose(ratio, 0.0)
        assert math.isclose(node.current_k, 1.0)

        node.curve_state = 'Curve'
        node.update_scheduled_cross_track_gain()
        assert math.isclose(node.current_k, 1.20)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cte_gain_schedule_reduces_high_speed_feedback_gain():
    rclpy.init()
    node = IntegratedStanleyController()
    try:
        node.current_speed = 12.0
        node.curve_state = 'Straight'
        ratio = node.update_scheduled_cross_track_gain()

        assert math.isclose(ratio, 1.0)
        assert math.isclose(node.current_k, 0.65)

        node.curve_state = 'Curve'
        node.update_scheduled_cross_track_gain()
        assert math.isclose(node.current_k, 0.90)
        assert math.isclose(node.max_delta_angle, 15.0)
        assert math.isclose(node.heading_weight, 0.30)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_heading_weight_damps_straight_but_preserves_curve_authority():
    rclpy.init()
    node = IntegratedStanleyController()
    try:
        node.current_speed = 12.0
        straight = {
            'curve_state': 'Straight',
            'heading_source': 'curve',
            'curvature_severity': 0.0,
        }
        for _ in range(20):
            node.update_scheduled_heading_weight(straight)
        straight_weight = node.current_heading_weight

        curve = dict(straight)
        curve['curve_state'] = 'Curve'
        first_curve_weight = node.update_scheduled_heading_weight(curve)
        for _ in range(10):
            node.update_scheduled_heading_weight(curve)

        assert math.isclose(straight_weight, 0.18, abs_tol=1e-4)
        assert first_curve_weight > straight_weight
        assert math.isclose(
            node.current_heading_weight, 0.30, abs_tol=1e-4)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_hough_weight_is_reduced_only_outside_confirmed_curve():
    rclpy.init()
    node = IntegratedStanleyController()
    try:
        node.current_speed = 12.0
        node.current_heading_weight = 0.14
        curve_hough = {
            'curve_state': 'Curve',
            'heading_source': 'hough',
            'curvature_severity': 0.0,
        }
        curve_weight = node.update_scheduled_heading_weight(curve_hough)

        node.current_heading_weight = 0.30
        straight_hough = dict(curve_hough)
        straight_hough['curve_state'] = 'Straight'
        for _ in range(30):
            node.update_scheduled_heading_weight(straight_hough)

        assert curve_weight > 0.14
        assert math.isclose(
            node.current_heading_weight, 0.14, abs_tol=1e-4)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_speed_ramp_depends_on_elapsed_time_not_frame_count():
    rclpy.init()
    node = IntegratedStanleyController()
    try:
        node.current_speed = 3.0
        one_frame_speed, _ = node.calculate_final_speed(
            12.0, 1.0 / 12.0)

        node.current_speed = 3.0
        first_half, _ = node.calculate_final_speed(12.0, 1.0 / 24.0)
        node.current_speed = first_half
        two_half_frames_speed, _ = node.calculate_final_speed(
            12.0, 1.0 / 24.0)

        assert math.isclose(one_frame_speed, 4.0)
        assert math.isclose(two_half_frames_speed, one_frame_speed)
    finally:
        node.destroy_node()
        rclpy.shutdown()

