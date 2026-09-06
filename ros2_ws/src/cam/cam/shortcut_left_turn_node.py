"""ROS node that routes the lane curve through a branch-aware left turn."""

from __future__ import annotations

from cam.shortcut_left_turn_logic import (
    BranchObservation,
    DetectionBox,
    LineObservation,
    ShortcutLeftTurnMachine,
    ShortcutTurnConfig,
    ShortcutTurnState,
    blend_paths,
    generate_exit_connection_path,
    generate_left_turn_path,
    observe_crossline,
    observe_curve,
    observe_left_branch,
    observe_outgoing_road,
    resample_path,
)

from custom_interfaces.msg import Curve, Detections, Point2D

import cv2

from cv_bridge import CvBridge, CvBridgeError

import message_filters

import numpy as np

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from sensor_msgs.msg import Image

from std_msgs.msg import Bool, Float32, String

from std_srvs.srv import Trigger


def as_bool(value):
    """Parse ROS launch values that may arrive as strings."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class ShortcutLeftTurnNode(Node):
    """Select a left branch and publish a smooth controller-facing curve."""

    def __init__(self):
        """Configure subscriptions, publishers, services, and turn state."""
        super().__init__('shortcut_left_turn_node')
        self.declare_parameter('image_topic', '/perception/lane/image')
        self.declare_parameter(
            'detections_topic', '/lane_yolo/detections')
        self.declare_parameter('base_curve_topic', '/center_curve/base')
        self.declare_parameter('output_curve_topic', '/center_curve')
        self.declare_parameter(
            'traffic_raw_state_topic', '/traffic_light_raw_state')
        self.declare_parameter('lane_override_topic', '/lane_override_cmd')
        self.declare_parameter(
            'speed_limit_topic', '/shortcut_left_turn/speed_limit')
        self.declare_parameter(
            'path_active_topic', '/shortcut_left_turn/path_active')
        self.declare_parameter(
            'state_topic', '/shortcut_left_turn/state')
        self.declare_parameter(
            'debug_image_topic', '/shortcut_left_turn/debug_image')
        self.declare_parameter('image_qos_reliability', 'reliable')
        self.declare_parameter('sync_queue_size', 20)
        self.declare_parameter('sync_slop', 0.16)
        self.declare_parameter('publish_override', True)
        self.declare_parameter('auto_trigger_after_s', -1.0)
        self.declare_parameter('override_republish_rate_hz', 5.0)
        self.declare_parameter('path_smoothing_alpha', 0.40)
        self.declare_parameter('path_sample_count', 60)
        self.declare_parameter('left_signal_window', 5)
        self.declare_parameter('left_signal_min_matches', 3)
        self.declare_parameter('min_prepare_duration_s', 0.8)
        self.declare_parameter('fallback_commit_s', 5.0)
        self.declare_parameter('strong_gap_ratio', 0.26)
        self.declare_parameter('weak_gap_ratio', 0.18)
        self.declare_parameter('strong_fit_error_ratio', 0.055)
        self.declare_parameter('weak_fit_error_ratio', 0.035)
        self.declare_parameter('strong_confirm_frames', 2)
        self.declare_parameter('weak_confirm_frames', 5)
        self.declare_parameter('prepare_speed_limit', 3.0)
        self.declare_parameter('turn_speed_limit', 3.0)
        self.declare_parameter('cruise_speed_limit', -1.0)
        self.declare_parameter('min_follow_duration_s', 3.0)
        self.declare_parameter('max_follow_duration_s', 12.0)
        self.declare_parameter('branch_loss_timeout_s', 2.0)
        self.declare_parameter('fallback_branch_grace_s', 3.5)
        self.declare_parameter('exit_max_heading_deg', 22.0)
        self.declare_parameter('exit_single_fit_error_ratio', 0.040)
        self.declare_parameter('exit_min_points', 3)
        self.declare_parameter('exit_near_x_min_ratio', 0.08)
        self.declare_parameter('exit_near_x_max_ratio', 0.90)
        self.declare_parameter('exit_confirm_frames', 5)
        self.declare_parameter('entry_alignment_window', 5)
        self.declare_parameter('entry_alignment_min_matches', 3)
        self.declare_parameter('entry_max_heading_deg', 60.0)
        self.declare_parameter('min_cruise_duration_s', 4.0)
        self.declare_parameter('session_timeout_s', 45.0)
        self.declare_parameter('crossline_window', 3)
        self.declare_parameter('crossline_min_matches', 2)
        self.declare_parameter('crossline_max_angle_deg', 22.0)
        self.declare_parameter('crossline_min_x_span_ratio', 0.30)
        self.declare_parameter('crossline_max_y_span_ratio', 0.16)
        self.declare_parameter('crossline_center_y_min_ratio', 0.32)
        self.declare_parameter('crossline_center_y_max_ratio', 0.72)
        self.declare_parameter('exit_min_turn_duration_s', 1.5)
        self.declare_parameter('exit_capture_max_heading_deg', 55.0)
        self.declare_parameter('exit_alignment_window', 5)
        self.declare_parameter('exit_alignment_min_matches', 3)
        self.declare_parameter('exit_handoff_duration_s', 0.8)
        self.declare_parameter('exit_timeout_s', 15.0)
        self.declare_parameter('rearm_non_left_frames', 5)
        self.declare_parameter('auto_trigger_state', 'entry')

        config = ShortcutTurnConfig(
            left_signal_window=int(
                self.get_parameter('left_signal_window').value),
            left_signal_min_matches=int(
                self.get_parameter('left_signal_min_matches').value),
            min_prepare_duration_s=float(
                self.get_parameter('min_prepare_duration_s').value),
            fallback_commit_s=float(
                self.get_parameter('fallback_commit_s').value),
            strong_gap_ratio=float(
                self.get_parameter('strong_gap_ratio').value),
            weak_gap_ratio=float(
                self.get_parameter('weak_gap_ratio').value),
            strong_fit_error_ratio=float(
                self.get_parameter('strong_fit_error_ratio').value),
            weak_fit_error_ratio=float(
                self.get_parameter('weak_fit_error_ratio').value),
            strong_confirm_frames=int(
                self.get_parameter('strong_confirm_frames').value),
            weak_confirm_frames=int(
                self.get_parameter('weak_confirm_frames').value),
            prepare_speed_limit=float(
                self.get_parameter('prepare_speed_limit').value),
            turn_speed_limit=float(
                self.get_parameter('turn_speed_limit').value),
            cruise_speed_limit=float(
                self.get_parameter('cruise_speed_limit').value),
            min_follow_duration_s=float(
                self.get_parameter('min_follow_duration_s').value),
            max_follow_duration_s=float(
                self.get_parameter('max_follow_duration_s').value),
            branch_loss_timeout_s=float(
                self.get_parameter('branch_loss_timeout_s').value),
            fallback_branch_grace_s=float(
                self.get_parameter('fallback_branch_grace_s').value),
            exit_max_heading_deg=float(
                self.get_parameter('exit_max_heading_deg').value),
            exit_single_fit_error_ratio=float(self.get_parameter(
                'exit_single_fit_error_ratio').value),
            exit_min_points=int(
                self.get_parameter('exit_min_points').value),
            exit_near_x_min_ratio=float(
                self.get_parameter('exit_near_x_min_ratio').value),
            exit_near_x_max_ratio=float(
                self.get_parameter('exit_near_x_max_ratio').value),
            exit_confirm_frames=int(
                self.get_parameter('exit_confirm_frames').value),
            entry_alignment_window=int(
                self.get_parameter('entry_alignment_window').value),
            entry_alignment_min_matches=int(self.get_parameter(
                'entry_alignment_min_matches').value),
            entry_max_heading_deg=float(
                self.get_parameter('entry_max_heading_deg').value),
            min_cruise_duration_s=float(
                self.get_parameter('min_cruise_duration_s').value),
            session_timeout_s=float(
                self.get_parameter('session_timeout_s').value),
            crossline_window=int(
                self.get_parameter('crossline_window').value),
            crossline_min_matches=int(
                self.get_parameter('crossline_min_matches').value),
            crossline_max_angle_deg=float(
                self.get_parameter('crossline_max_angle_deg').value),
            crossline_min_x_span_ratio=float(self.get_parameter(
                'crossline_min_x_span_ratio').value),
            crossline_max_y_span_ratio=float(self.get_parameter(
                'crossline_max_y_span_ratio').value),
            crossline_center_y_min_ratio=float(self.get_parameter(
                'crossline_center_y_min_ratio').value),
            crossline_center_y_max_ratio=float(self.get_parameter(
                'crossline_center_y_max_ratio').value),
            exit_min_turn_duration_s=float(
                self.get_parameter('exit_min_turn_duration_s').value),
            exit_capture_max_heading_deg=float(self.get_parameter(
                'exit_capture_max_heading_deg').value),
            exit_alignment_window=int(
                self.get_parameter('exit_alignment_window').value),
            exit_alignment_min_matches=int(self.get_parameter(
                'exit_alignment_min_matches').value),
            exit_handoff_duration_s=float(
                self.get_parameter('exit_handoff_duration_s').value),
            exit_timeout_s=float(
                self.get_parameter('exit_timeout_s').value),
            rearm_non_left_frames=int(
                self.get_parameter('rearm_non_left_frames').value),
        )
        self.machine = ShortcutLeftTurnMachine(config)
        self.bridge = CvBridge()
        self.publish_override = as_bool(
            self.get_parameter('publish_override').value)
        self.auto_trigger_after_s = float(
            self.get_parameter('auto_trigger_after_s').value)
        self.auto_trigger_state = str(
            self.get_parameter('auto_trigger_state').value
        ).strip().lower()
        if self.auto_trigger_state not in ('entry', 'exit'):
            raise ValueError('auto_trigger_state must be entry or exit')
        self.path_smoothing_alpha = float(np.clip(
            self.get_parameter('path_smoothing_alpha').value, 0.0, 1.0))
        self.path_sample_count = max(
            12, int(self.get_parameter('path_sample_count').value))
        self.raw_signal = 'unknown'
        self.first_source_s = None
        self.last_source_s = None
        self.auto_triggered = False
        self.manual_trigger_pending = False
        self.manual_exit_arm_pending = False
        self.last_route = []
        self.last_branch = BranchObservation()
        self.last_crossline = LineObservation()
        self.last_outgoing = LineObservation()
        self.handoff_source_route = []
        self.last_handoff_target = []
        self.exit_capture_started = False
        self.last_status = 'state=IDLE reason=waiting_for_left_signal'

        reliability_name = str(
            self.get_parameter('image_qos_reliability').value
        ).strip().lower()
        image_reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if reliability_name == 'best_effort'
            else ReliabilityPolicy.RELIABLE
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=image_reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter('image_topic').value,
            qos_profile=image_qos,
        )
        self.detections_sub = message_filters.Subscriber(
            self,
            Detections,
            self.get_parameter('detections_topic').value,
            qos_profile=reliable_qos,
        )
        self.base_curve_sub = message_filters.Subscriber(
            self,
            Curve,
            self.get_parameter('base_curve_topic').value,
            qos_profile=reliable_qos,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.detections_sub, self.base_curve_sub],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop').value),
        )
        self.sync.registerCallback(self.synced_callback)
        self.create_subscription(
            String,
            self.get_parameter('traffic_raw_state_topic').value,
            self.traffic_callback,
            10,
        )

        self.curve_pub = self.create_publisher(
            Curve,
            self.get_parameter('output_curve_topic').value,
            reliable_qos,
        )
        self.override_pub = self.create_publisher(
            String, self.get_parameter('lane_override_topic').value, 10)
        self.speed_limit_pub = self.create_publisher(
            Float32, self.get_parameter('speed_limit_topic').value, 10)
        self.path_active_pub = self.create_publisher(
            Bool, self.get_parameter('path_active_topic').value, 10)
        self.state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value, 10)
        self.debug_image_pub = self.create_publisher(
            Image, self.get_parameter('debug_image_topic').value, image_qos)
        self.create_service(
            Trigger,
            '/shortcut_left_turn/trigger',
            self.trigger_callback,
        )
        self.create_service(
            Trigger,
            '/shortcut_left_turn/reset',
            self.reset_callback,
        )
        self.create_service(
            Trigger,
            '/shortcut_left_turn/arm_exit',
            self.arm_exit_callback,
        )
        republish_rate = max(
            0.5,
            float(self.get_parameter('override_republish_rate_hz').value),
        )
        self.create_timer(1.0 / republish_rate, self.republish_override)
        self.get_logger().info(
            'Shortcut left-turn router ready: '
            f'base={self.get_parameter("base_curve_topic").value}, '
            f'output={self.get_parameter("output_curve_topic").value}, '
            f'auto_trigger={self.auto_trigger_after_s:.2f}s, '
            f'auto_state={self.auto_trigger_state}, '
            f'left_signal={config.left_signal_min_matches}-of-'
            f'{config.left_signal_window}, '
            f'publish_override={self.publish_override}'
        )

    @staticmethod
    def stamp_to_seconds(stamp):
        """Convert a ROS timestamp to floating-point seconds."""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def traffic_callback(self, message):
        """Cache the most recent raw traffic-light state."""
        self.raw_signal = str(message.data).strip().lower()
        self.machine.observe_signal(self.raw_signal)

    def trigger_callback(self, _request, response):
        """Queue a complete shortcut session for replay or testing."""
        self.manual_trigger_pending = True
        response.success = True
        response.message = 'Left-turn trigger queued.'
        return response

    def arm_exit_callback(self, _request, response):
        """Queue an exit-only session for a cropped exit replay."""
        self.manual_exit_arm_pending = True
        response.success = True
        response.message = 'Shortcut exit session queued.'
        return response

    def reset_callback(self, _request, response):
        """Clear the shortcut session and release all overrides."""
        self.machine.reset()
        self.last_route = []
        self.last_branch = BranchObservation()
        self.last_crossline = LineObservation()
        self.last_outgoing = LineObservation()
        self.handoff_source_route = []
        self.last_handoff_target = []
        self.exit_capture_started = False
        self.publish_override_command('reset')
        self.speed_limit_pub.publish(Float32(data=-1.0))
        self.path_active_pub.publish(Bool(data=False))
        response.success = True
        response.message = 'Left-turn state reset.'
        return response

    def publish_override_command(self, command):
        """Publish a lane command when override output is enabled."""
        if self.publish_override:
            self.override_pub.publish(String(data=str(command)))

    def republish_override(self):
        """Keep the entry lane command alive while preparing to turn."""
        if self.machine.state == ShortcutTurnState.PREPARE_LEFT:
            self.publish_override_command('go_left')

    def reset_for_source_loop(self, source_s):
        """Reset temporal state when a rosbag loops backward in time."""
        self.machine.reset()
        self.first_source_s = source_s
        self.auto_triggered = False
        self.manual_trigger_pending = False
        self.manual_exit_arm_pending = False
        self.last_route = []
        self.last_branch = BranchObservation()
        self.last_crossline = LineObservation()
        self.last_outgoing = LineObservation()
        self.handoff_source_route = []
        self.last_handoff_target = []
        self.exit_capture_started = False

    @staticmethod
    def detection_boxes(message):
        """Convert center-line detections into ROS-independent boxes."""
        return [
            DetectionBox(
                float(detection.xmin),
                float(detection.ymin),
                float(detection.xmax),
                float(detection.ymax),
                float(detection.confidence),
            )
            for detection in message.detections
            if str(detection.class_name).strip() == 'center_line'
        ]

    def synced_callback(self, image_msg, detections_msg, base_curve_msg):
        """Process a synchronized image, detection set, and base path."""
        source_s = self.stamp_to_seconds(image_msg.header.stamp)
        if source_s <= 0.0:
            source_s = self.get_clock().now().nanoseconds * 1e-9
        if (
            self.last_source_s is not None
            and source_s < self.last_source_s - 0.1
        ):
            self.reset_for_source_loop(source_s)
        self.last_source_s = source_s
        if self.first_source_s is None:
            self.first_source_s = source_s

        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(
                f'Left-turn image conversion failed: {exc}')
            return
        height, width = frame.shape[:2]
        boxes = self.detection_boxes(detections_msg)
        committed = self.machine.state in (
            ShortcutTurnState.FOLLOW_BRANCH,
        )
        branch = observe_left_branch(
            boxes,
            width,
            height,
            committed=committed,
            weak_gap_ratio=self.machine.config.weak_gap_ratio,
            weak_fit_error_ratio=self.machine.config.weak_fit_error_ratio,
        )
        base_observation = observe_curve(
            [(point.x, point.y) for point in base_curve_msg.points],
            width,
            height,
        )
        crossline = observe_crossline(
            boxes,
            width,
            height,
            max_angle_deg=self.machine.config.crossline_max_angle_deg,
            min_x_span_ratio=(
                self.machine.config.crossline_min_x_span_ratio),
            max_y_span_ratio=(
                self.machine.config.crossline_max_y_span_ratio),
            center_y_min_ratio=(
                self.machine.config.crossline_center_y_min_ratio),
            center_y_max_ratio=(
                self.machine.config.crossline_center_y_max_ratio),
        )
        outgoing = observe_outgoing_road(
            boxes,
            width,
            height,
            max_heading_deg=self.machine.config.exit_max_heading_deg,
            min_points=self.machine.config.exit_min_points,
            max_fit_error_ratio=(
                self.machine.config.exit_single_fit_error_ratio),
            near_x_min_ratio=(
                self.machine.config.exit_near_x_min_ratio),
            near_x_max_ratio=(
                self.machine.config.exit_near_x_max_ratio),
        )

        force_trigger = self.manual_trigger_pending
        self.manual_trigger_pending = False
        previous_state = self.machine.state
        if self.manual_exit_arm_pending:
            self.machine.arm_exit(source_s)
            self.manual_exit_arm_pending = False
        if (
            not self.auto_triggered
            and self.auto_trigger_after_s >= 0.0
            and source_s - self.first_source_s >= self.auto_trigger_after_s
        ):
            if self.auto_trigger_state == 'exit':
                self.machine.arm_exit(source_s)
            else:
                force_trigger = True
            self.auto_triggered = True

        decision = self.machine.update(
            source_s,
            self.raw_signal,
            base_observation,
            branch,
            crossline=crossline,
            outgoing=outgoing,
            force_trigger=force_trigger,
            signal_is_new=False,
        )
        if decision.override_command is not None:
            self.publish_override_command(decision.override_command)
        if decision.state != previous_state:
            self.get_logger().warn(
                f'Left-turn transition {previous_state.value} -> '
                f'{decision.state.value}: {decision.reason}'
            )

        route = self.select_route(
            decision,
            branch,
            crossline,
            outgoing,
            base_curve_msg,
            width,
            height,
            previous_state,
        )
        output_curve = self.make_output_curve(
            image_msg,
            base_curve_msg,
            route,
            decision.path_active,
        )
        self.curve_pub.publish(output_curve)
        self.speed_limit_pub.publish(Float32(data=float(decision.speed_limit)))
        self.path_active_pub.publish(Bool(data=decision.path_active))

        status = self.format_status(
            decision,
            branch,
            base_observation,
            crossline,
            outgoing,
            len(route),
        )
        self.last_status = status
        self.state_pub.publish(String(data=status))
        self.publish_debug_image(
            image_msg,
            frame,
            detections_msg,
            base_curve_msg,
            route,
            branch,
            crossline,
            outgoing,
            status,
        )

    def select_route(
        self,
        decision,
        branch,
        crossline,
        outgoing,
        base_curve_msg,
        width,
        height,
        previous_state,
    ):
        """Select and smooth the controller path for the active phase."""
        if not decision.path_active:
            if decision.state in (
                ShortcutTurnState.IDLE,
                ShortcutTurnState.PREPARE_LEFT,
                ShortcutTurnState.SHORTCUT_CRUISE,
                ShortcutTurnState.COMPLETE,
            ):
                self.last_route = []
                self.handoff_source_route = []
                self.last_handoff_target = []
            return []

        if decision.state == ShortcutTurnState.FOLLOW_BRANCH:
            candidate = branch if branch.valid else self.last_branch
            if branch.valid:
                self.last_branch = branch
            fallback = decision.use_fallback_path and not candidate.valid
            if candidate.valid or fallback:
                generated = generate_left_turn_path(
                    candidate,
                    width,
                    height,
                    sample_count=self.path_sample_count,
                    fallback=fallback,
                )
                self.last_route = blend_paths(
                    self.last_route,
                    generated,
                    alpha=self.path_smoothing_alpha,
                )
            elif not self.last_route:
                self.last_route = generate_left_turn_path(
                    BranchObservation(),
                    width,
                    height,
                    sample_count=self.path_sample_count,
                    fallback=True,
                )
            return self.last_route

        if decision.state == ShortcutTurnState.EXIT_LEFT_COMMIT:
            if previous_state != ShortcutTurnState.EXIT_LEFT_COMMIT:
                self.last_route = []
                self.exit_capture_started = False
            if crossline.valid:
                self.last_crossline = crossline
            capture_ready = (
                outgoing.point_count >= self.machine.config.exit_min_points
                and abs(outgoing.heading_deg)
                <= self.machine.config.exit_capture_max_heading_deg
                and outgoing.y_span_ratio >= 0.10
                and outgoing.fit_error_ratio <= 0.060
                and self.machine.config.exit_near_x_min_ratio
                <= outgoing.near_x_ratio
                <= self.machine.config.exit_near_x_max_ratio
            )
            if capture_ready:
                self.exit_capture_started = True
                self.last_outgoing = outgoing
                generated = generate_left_turn_path(
                    BranchObservation(
                        valid=True,
                        points=outgoing.points,
                        point_count=outgoing.point_count,
                        fit_error_ratio=outgoing.fit_error_ratio,
                        near_x_ratio=outgoing.near_x_ratio,
                        heading_deg=outgoing.heading_deg,
                    ),
                    width,
                    height,
                    sample_count=self.path_sample_count,
                )
            elif not self.exit_capture_started:
                candidate = (
                    crossline if crossline.valid else self.last_crossline)
                generated = generate_exit_connection_path(
                    candidate,
                    width,
                    height,
                    sample_count=self.path_sample_count,
                )
            else:
                return self.last_route
            self.last_route = blend_paths(
                self.last_route,
                generated,
                alpha=self.path_smoothing_alpha,
            )
            return self.last_route

        if decision.state == ShortcutTurnState.EXIT_HANDOFF:
            if previous_state != ShortcutTurnState.EXIT_HANDOFF:
                self.handoff_source_route = list(self.last_route)
            base_points = [
                (float(point.x), float(point.y))
                for point in base_curve_msg.points
            ]
            if (
                len(base_points) >= 2
                and base_points[0][1] < base_points[-1][1]
            ):
                base_points.reverse()
            target = resample_path(base_points, self.path_sample_count)
            if outgoing.valid and target:
                self.last_outgoing = outgoing
                self.last_handoff_target = target
            elif self.last_handoff_target:
                target = self.last_handoff_target
            if target and self.handoff_source_route:
                self.last_route = blend_paths(
                    self.handoff_source_route,
                    target,
                    alpha=decision.handoff_progress,
                )
            return self.last_route

        return self.last_route

    @staticmethod
    def make_output_curve(
        image_msg,
        base_curve_msg,
        route,
        path_active,
    ):
        """Build the routed curve while preserving the source header."""
        if not path_active:
            output = Curve()
            output.header = image_msg.header
            output.points = list(base_curve_msg.points)
            output.state = base_curve_msg.state
            return output
        output = Curve()
        output.header = image_msg.header
        output.points = [
            Point2D(x=float(point[0]), y=float(point[1]))
            for point in route
        ]
        output.state = 'Curve'
        return output

    @staticmethod
    def format_status(
        decision,
        branch,
        base,
        crossline,
        outgoing,
        path_points,
    ):
        """Serialize state and geometry fields for logs and overlays."""
        return (
            f'state={decision.state.value} '
            f'session={decision.session_active} '
            f'branch_valid={branch.valid} split={branch.split} '
            f'families={branch.left_count}/{branch.right_count} '
            f'points={branch.point_count} near_x={branch.near_x_ratio:.3f} '
            f'gap={branch.gap_ratio:.3f} fit={branch.fit_error_ratio:.3f} '
            f'branch_heading={branch.heading_deg:+.1f}deg '
            f'base_heading={base.heading_deg:+.1f}deg '
            f'cross_valid={crossline.valid} '
            f'cross_angle={crossline.angle_from_horizontal_deg:.1f}deg '
            f'cross_span={crossline.x_span_ratio:.3f} '
            f'out_valid={outgoing.valid} '
            f'out_heading={outgoing.heading_deg:+.1f}deg '
            f'confirm={decision.branch_confirm_count} '
            f'cross_votes={decision.crossline_confirm_count}/3 '
            f'exit={decision.exit_confirm_count} '
            f'handoff={decision.handoff_progress:.3f} '
            f'path_points={path_points} fallback={decision.use_fallback_path} '
            f'speed_limit={decision.speed_limit:.1f} '
            f'reason={decision.reason}'
        )

    def publish_debug_image(
        self,
        image_msg,
        frame,
        detections_msg,
        base_curve_msg,
        route,
        branch,
        crossline,
        outgoing,
        status,
    ):
        """Publish the annotated perception and command overlay."""
        output = frame.copy()
        for detection in detections_msg.detections:
            if str(detection.class_name).strip() != 'center_line':
                continue
            cv2.rectangle(
                output,
                (int(detection.xmin), int(detection.ymin)),
                (int(detection.xmax), int(detection.ymax)),
                (255, 100, 0),
                1,
            )
        base_points = np.asarray([
            (int(point.x), int(point.y))
            for point in base_curve_msg.points
        ], dtype=np.int32)
        if len(base_points) >= 2:
            cv2.polylines(output, [base_points], False, (130, 130, 130), 2)
        for point in branch.points:
            cv2.circle(
                output,
                (int(point[0]), int(point[1])),
                5,
                (255, 160, 0),
                -1,
            )
        if branch.split and branch.split_x_ratio > 0.0:
            split_x = int(branch.split_x_ratio * output.shape[1])
            cv2.line(
                output, (split_x, 0), (split_x, output.shape[0] - 1),
                (0, 0, 255), 1)
        cross_points = np.asarray(
            sorted(crossline.points, key=lambda point: point[0]),
            dtype=np.int32,
        )
        if len(cross_points) >= 2:
            cv2.polylines(
                output,
                [cross_points],
                False,
                (0, 0, 255),
                2,
            )
        for point in crossline.points:
            cv2.circle(
                output,
                (int(point[0]), int(point[1])),
                5,
                (0, 0, 255),
                -1 if crossline.valid else 1,
            )
        out_points = np.asarray(
            sorted(outgoing.points, key=lambda point: point[1], reverse=True),
            dtype=np.int32,
        )
        if outgoing.valid and len(out_points) >= 2:
            cv2.polylines(
                output,
                [out_points],
                False,
                (255, 255, 0),
                2,
            )
            for point in outgoing.points:
                cv2.circle(
                    output,
                    (int(point[0]), int(point[1])),
                    4,
                    (255, 255, 0),
                    -1,
                )
        route_points = np.asarray(route, dtype=np.int32)
        if len(route_points) >= 2:
            cv2.polylines(output, [route_points], False, (0, 230, 255), 4)
        cv2.circle(
            output,
            (output.shape[1] // 2, int(0.98 * output.shape[0])),
            6,
            (0, 255, 0),
            -1,
        )
        self.draw_status_panel(output, status)
        debug_message = self.bridge.cv2_to_imgmsg(output, 'bgr8')
        debug_message.header = image_msg.header
        self.debug_image_pub.publish(debug_message)

    @staticmethod
    def draw_status_panel(frame, status):
        """Render compact state-machine diagnostics on a debug frame."""
        fields = {}
        for item in status.split():
            if '=' in item:
                key, value = item.split('=', 1)
                fields[key] = value
        panel_height = 94
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (frame.shape[1], panel_height),
                      (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.62, frame, 0.38, 0.0, frame)
        lines = [
            f'{fields.get("state", "IDLE")}  '
            f'session={fields.get("session", "False")}',
            f'cross={fields.get("cross_valid", "False")} '
            f'{fields.get("cross_votes", "0/3")}  '
            f'angle={fields.get("cross_angle", "0deg")}  '
            f'span={fields.get("cross_span", "0")}',
            f'out={fields.get("out_valid", "False")}  '
            f'heading={fields.get("out_heading", "0deg")}  '
            f'align={fields.get("exit", "0")}/5  '
            f'blend={fields.get("handoff", "0")}',
            f'base_h={fields.get("base_heading", "0deg")}  '
            f'speed={fields.get("speed_limit", "-1")}  '
            f'reason={fields.get("reason", "waiting")}',
        ]
        for index, text in enumerate(lines):
            cv2.putText(
                frame,
                text,
                (7, 17 + index * 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.34,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    def destroy_node(self):
        """Release active lane and speed overrides before shutdown."""
        try:
            if rclpy.ok():
                self.publish_override_command('reset')
                self.speed_limit_pub.publish(Float32(data=-1.0))
                self.path_active_pub.publish(Bool(data=False))
        except RCLError:
            # SIGINT can invalidate the context between the ok() check and
            # any one of the final publications.
            pass
        return super().destroy_node()


def main(args=None):
    """Run the shortcut left-turn router."""
    rclpy.init(args=args)
    node = ShortcutLeftTurnNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
