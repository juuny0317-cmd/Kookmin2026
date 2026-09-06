from collections import deque
import threading
import time

from custom_interfaces.msg import (
    Curve,
    LaneControlState,
    LaneControlStateV2,
    LaneControlStateV3,
    ObstacleState,
    PipelineTiming,
    StampedMotorCommand,
    StampedXycarState,
    StanleyDebug,
    StanleyDebugV2,
    StanleyDebugV3,
    StanleyDebugV4,
    XycarState,
)
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, Float32MultiArray, Header, String
from std_srvs.srv import Trigger

from cam.curvature_steering_limiter import (
    CurvatureSteeringLimiter,
    CurvatureSteeringLimiterConfig,
)
from cam.curve_speed_policy import CurveSpeedPolicy, CurveSpeedPolicyConfig


class IntegratedStanleyController(Node):

    def __init__(self):
        super().__init__('integrated_stanley_controller')

        self.control_callback_group = ReentrantCallbackGroup()
        self.state_update_callback_group = ReentrantCallbackGroup()

        self.declare_parameter('start_service_name', '/start_stanley_controller')
        self.declare_parameter('respect_traffic_light', True)
        self.declare_parameter('enable_scene_inputs', True)
        self.declare_parameter(
            'shortcut_speed_limit_topic',
            '/shortcut_left_turn/speed_limit',
        )
        self.declare_parameter('shortcut_speed_limit_timeout_s', 0.8)
        self.declare_parameter(
            'obstacle_speed_limit_topic', '/obstacle_speed_limit')
        self.declare_parameter('obstacle_speed_limit_timeout_s', 0.8)
        self.declare_parameter(
            'shortcut_path_active_topic',
            '/shortcut_left_turn/path_active',
        )
        self.declare_parameter('shortcut_steering_limit', 39.0)
        self.declare_parameter('speed_straight', 20.0)
        self.declare_parameter('curve_strong_speed', 10.0)
        self.declare_parameter('curve_medium_speed', 12.0)
        self.declare_parameter('curve_mild_speed', 14.0)
        self.declare_parameter('straight_recovery_speed', 14.0)
        self.declare_parameter('curve_strong_threshold', 0.60)
        self.declare_parameter('curve_medium_threshold', 0.35)
        self.declare_parameter('straight_enter_cte_px', 45.0)
        self.declare_parameter('straight_exit_cte_px', 60.0)
        self.declare_parameter('straight_enter_heading_deg', 5.0)
        self.declare_parameter('straight_exit_heading_deg', 8.0)
        self.declare_parameter('straight_enter_curvature_risk', 0.20)
        self.declare_parameter('straight_exit_curvature_risk', 0.35)
        self.declare_parameter('straight_confirm_frames', 2)
        self.declare_parameter('curve_confirm_frames', 2)
        self.declare_parameter('curvature_filter_window', 3)
        self.declare_parameter('curvature_scale', 0.50)
        self.declare_parameter('curve_accel_rate_per_s', 6.0)
        self.declare_parameter('straight_accel_rate_per_s', 12.0)
        self.declare_parameter('curve_decel_rate_per_s', 96.0)
        self.declare_parameter('straight_decel_rate_per_s', 36.0)
        self.declare_parameter('state_topic', '/xycar_state')
        self.declare_parameter(
            'stamped_state_topic', '/xycar_state_stamped')
        self.declare_parameter(
            'lane_control_state_topic', '/lane_control_state_stamped')
        self.declare_parameter(
            'lane_control_state_v2_topic', '/lane_control_state_v2')
        self.declare_parameter(
            'lane_control_state_v3_topic', '/lane_control_state_v3')
        self.declare_parameter('use_lane_control_state_v3', False)
        self.declare_parameter('use_lane_control_state_v2', False)
        self.declare_parameter('use_lane_control_state', True)
        self.declare_parameter('use_stamped_state', True)
        self.declare_parameter('motor_topic', '/xycar_motor')
        self.declare_parameter('mission_mode_topic', '/mission_mode')
        self.declare_parameter(
            'stamped_motor_topic', '/lane_motor_cmd_stamped')
        self.declare_parameter('verbose_status', True)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('event_driven_control', False)
        self.declare_parameter('state_timeout_s', 0.80)
        self.declare_parameter('future_stamp_tolerance_s', 0.05)
        self.declare_parameter('hold_last_angle_when_stale', True)
        self.declare_parameter('image_center_x', 320.0)
        self.declare_parameter('gain_schedule_start_speed', 4.0)
        self.declare_parameter('gain_schedule_end_speed', 12.0)
        self.declare_parameter('cross_track_gain_straight_low_speed', 1.0)
        self.declare_parameter('cross_track_gain_straight_high_speed', 0.65)
        self.declare_parameter('cross_track_gain_curve_low_speed', 1.20)
        self.declare_parameter('cross_track_gain_curve_high_speed', 0.90)
        self.declare_parameter('cross_track_gain_obstacle', 1.80)
        self.declare_parameter('adaptive_heading_weight_enabled', True)
        self.declare_parameter('heading_weight_low_speed', 0.30)
        self.declare_parameter('heading_weight_straight_high_speed', 0.18)
        self.declare_parameter('heading_weight_hough_high_speed', 0.14)
        self.declare_parameter('heading_weight_curve_high_speed', 0.30)
        self.declare_parameter('heading_weight_rise_alpha', 0.65)
        self.declare_parameter('heading_weight_fall_alpha', 0.35)
        self.declare_parameter('accel_rate_per_s', 12.0)
        self.declare_parameter('decel_rate_per_s', 36.0)
        self.declare_parameter('nominal_source_rate_hz', 12.0)
        self.declare_parameter('source_dt_min_s', 0.03)
        self.declare_parameter('source_dt_max_s', 0.20)
        self.declare_parameter('curvature_steering_rate_enabled', False)
        self.declare_parameter('steering_rate_straight_per_s', 120.0)
        self.declare_parameter('steering_rate_curve_per_s', 300.0)
        self.declare_parameter('steering_rate_reversal_per_s', 360.0)
        self.declare_parameter('steering_rate_risk_low', 0.15)
        self.declare_parameter('steering_rate_risk_high', 0.60)
        self.declare_parameter('steering_rate_hold_frames', 4)
        self.declare_parameter('debug_topic', '/stanley/debug')
        self.declare_parameter('debug_v2_topic', '/stanley/debug_v2')
        self.declare_parameter('debug_v3_topic', '/stanley/debug_v3')
        self.declare_parameter('debug_v4_topic', '/stanley/debug_v4')
        self.declare_parameter('publish_debug_v2', True)
        self.declare_parameter('publish_debug_v3', True)
        self.declare_parameter('publish_debug_v4', True)
        self.declare_parameter('publish_pipeline_timing', False)
        self.declare_parameter('pipeline_timing_topic', '/pipeline_timing')

        self.respect_traffic_light = self.as_bool(
            self.get_parameter('respect_traffic_light').value)
        self.enable_scene_inputs = self.as_bool(
            self.get_parameter('enable_scene_inputs').value)
        self.shortcut_speed_limit_timeout_s = max(
            0.1,
            float(self.get_parameter(
                'shortcut_speed_limit_timeout_s').value),
        )
        self.obstacle_speed_limit_timeout_s = max(
            0.1,
            float(self.get_parameter(
                'obstacle_speed_limit_timeout_s').value),
        )
        self.shortcut_steering_limit = max(
            0.0,
            float(self.get_parameter('shortcut_steering_limit').value),
        )
        self.verbose_status = self.as_bool(
            self.get_parameter('verbose_status').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.event_driven_control = self.as_bool(
            self.get_parameter('event_driven_control').value)
        self.state_timeout_s = float(self.get_parameter('state_timeout_s').value)
        self.future_stamp_tolerance_s = max(
            0.0,
            float(self.get_parameter('future_stamp_tolerance_s').value),
        )
        self.hold_last_angle_when_stale = self.as_bool(
            self.get_parameter('hold_last_angle_when_stale').value)
        self.image_center_x = float(self.get_parameter('image_center_x').value)
        self.use_stamped_state = self.as_bool(
            self.get_parameter('use_stamped_state').value)
        self.use_lane_control_state = self.as_bool(
            self.get_parameter('use_lane_control_state').value)
        self.use_lane_control_state_v3 = self.as_bool(
            self.get_parameter('use_lane_control_state_v3').value)
        self.use_lane_control_state_v2 = self.as_bool(
            self.get_parameter('use_lane_control_state_v2').value)
        self.gain_schedule_start_speed = float(
            self.get_parameter('gain_schedule_start_speed').value)
        self.gain_schedule_end_speed = max(
            self.gain_schedule_start_speed + 1e-6,
            float(self.get_parameter('gain_schedule_end_speed').value),
        )
        self.k_straight_low_speed = max(0.0, float(
            self.get_parameter(
                'cross_track_gain_straight_low_speed').value))
        self.k_straight_high_speed = max(0.0, float(
            self.get_parameter(
                'cross_track_gain_straight_high_speed').value))
        self.k_curve_low_speed = max(0.0, float(
            self.get_parameter(
                'cross_track_gain_curve_low_speed').value))
        self.k_curve_high_speed = max(0.0, float(
            self.get_parameter(
                'cross_track_gain_curve_high_speed').value))
        self.k_obstacle = max(0.0, float(
            self.get_parameter('cross_track_gain_obstacle').value))
        self.adaptive_heading_weight_enabled = self.as_bool(
            self.get_parameter('adaptive_heading_weight_enabled').value)
        self.heading_weight_low_speed = max(0.0, float(
            self.get_parameter('heading_weight_low_speed').value))
        self.heading_weight_straight_high_speed = max(0.0, float(
            self.get_parameter(
                'heading_weight_straight_high_speed').value))
        self.heading_weight_hough_high_speed = max(0.0, float(
            self.get_parameter('heading_weight_hough_high_speed').value))
        self.heading_weight_curve_high_speed = max(0.0, float(
            self.get_parameter('heading_weight_curve_high_speed').value))
        self.heading_weight_rise_alpha = float(np.clip(
            self.get_parameter('heading_weight_rise_alpha').value,
            0.0,
            1.0,
        ))
        self.heading_weight_fall_alpha = float(np.clip(
            self.get_parameter('heading_weight_fall_alpha').value,
            0.0,
            1.0,
        ))
        self.accel_rate_per_s = max(0.0, float(
            self.get_parameter('accel_rate_per_s').value))
        self.decel_rate_per_s = max(0.0, float(
            self.get_parameter('decel_rate_per_s').value))
        self.nominal_source_dt_s = 1.0 / max(
            float(self.get_parameter('nominal_source_rate_hz').value), 1.0)
        self.source_dt_min_s = max(0.0, float(
            self.get_parameter('source_dt_min_s').value))
        self.source_dt_max_s = max(
            self.source_dt_min_s,
            float(self.get_parameter('source_dt_max_s').value),
        )

        if self.use_lane_control_state_v3:
            self.xycar_state_sub = self.create_subscription(
                LaneControlStateV3,
                self.get_parameter('lane_control_state_v3_topic').value,
                self.lane_control_state_v3_callback,
                10,
                callback_group=self.control_callback_group,
            )
        elif self.use_lane_control_state_v2:
            self.xycar_state_sub = self.create_subscription(
                LaneControlStateV2,
                self.get_parameter('lane_control_state_v2_topic').value,
                self.lane_control_state_v2_callback,
                10,
                callback_group=self.control_callback_group,
            )
        elif self.use_lane_control_state:
            self.xycar_state_sub = self.create_subscription(
                LaneControlState,
                self.get_parameter('lane_control_state_topic').value,
                self.lane_control_state_callback,
                10,
                callback_group=self.control_callback_group,
            )
        elif self.use_stamped_state:
            self.xycar_state_sub = self.create_subscription(
                StampedXycarState,
                self.get_parameter('stamped_state_topic').value,
                self.stamped_xycar_state_callback,
                10,
                callback_group=self.control_callback_group,
            )
        else:
            self.xycar_state_sub = self.create_subscription(
                XycarState,
                self.get_parameter('state_topic').value,
                self.xycar_state_callback,
                10,
                callback_group=self.control_callback_group,
            )
        self.obstacle_state_sub = None
        self.curve_subscription = None
        if not (
            self.use_lane_control_state_v3
            or self.use_lane_control_state_v2
            or self.use_lane_control_state
        ):
            self.curve_subscription = self.create_subscription(
                Curve,
                '/center_curve',
                self.curve_callback,
                10,
                callback_group=self.state_update_callback_group,
            )
        self.traffic_state_subscription = None
        self.sign_color_subscription = None
        if self.enable_scene_inputs:
            self.obstacle_state_sub = self.create_subscription(
                ObstacleState,
                '/obstacle_state',
                self.obstacle_state_callback,
                10,
                callback_group=self.state_update_callback_group,
            )
            self.traffic_state_subscription = self.create_subscription(
                String,
                '/traffic_light_state',
                self.traffic_state_callback,
                10,
                callback_group=self.state_update_callback_group,
            )
            self.sign_color_subscription = self.create_subscription(
                String,
                '/sign_color',
                self.traffic_state_callback,
                10,
                callback_group=self.state_update_callback_group,
            )

        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('motor_topic').value,
            10,
        )
        self.stamped_motor_pub = self.create_publisher(
            StampedMotorCommand,
            self.get_parameter('stamped_motor_topic').value,
            10,
        )
        self.debug_pub = self.create_publisher(
            StanleyDebug,
            self.get_parameter('debug_topic').value,
            10,
        )
        self.debug_v2_pub = None
        if self.as_bool(self.get_parameter('publish_debug_v2').value):
            self.debug_v2_pub = self.create_publisher(
                StanleyDebugV2,
                self.get_parameter('debug_v2_topic').value,
                10,
            )
        self.debug_v3_pub = None
        if self.as_bool(self.get_parameter('publish_debug_v3').value):
            self.debug_v3_pub = self.create_publisher(
                StanleyDebugV3,
                self.get_parameter('debug_v3_topic').value,
                10,
            )
        self.debug_v4_pub = None
        if self.as_bool(self.get_parameter('publish_debug_v4').value):
            self.debug_v4_pub = self.create_publisher(
                StanleyDebugV4,
                self.get_parameter('debug_v4_topic').value,
                10,
            )
        self.pipeline_timing_pub = None
        if self.as_bool(self.get_parameter('publish_pipeline_timing').value):
            self.pipeline_timing_pub = self.create_publisher(
                PipelineTiming,
                self.get_parameter('pipeline_timing_topic').value,
                100,
            )
        self.start_service = self.create_service(
            Trigger,
            self.get_parameter('start_service_name').value,
            self.start_service_callback,
            callback_group=self.state_update_callback_group,
        )

        self.pixel_to_meter = 1.9 / 650.0
        # heading_weight remains as a compatibility alias for formula tests and
        # fixed-weight operation; current_heading_weight is scheduled per frame.
        self.heading_weight = self.heading_weight_low_speed
        self.current_heading_weight = self.heading_weight_low_speed
        self.last_raw_cte_term = 0.0
        self.last_heading_cte_conflict = False
        self.deg_to_servo_scale = 5.0 / 3.0
        self.deg_to_servo_offset = 0.0
        self.max_delta_angle = 15.0
        self.window = 1

        # Compatibility aliases retain the proven low-speed gains.  current_k
        # is selected from the scheduled low/high-speed endpoints per frame.
        self.k_straight = self.k_straight_low_speed
        self.k_curve = self.k_curve_low_speed

        self.speed_straight = float(self.get_parameter('speed_straight').value)
        curve_speed_config = CurveSpeedPolicyConfig(
            straight_speed=self.speed_straight,
            strong_curve_speed=float(self.get_parameter(
                'curve_strong_speed').value),
            medium_curve_speed=float(self.get_parameter(
                'curve_medium_speed').value),
            mild_curve_speed=float(self.get_parameter(
                'curve_mild_speed').value),
            recovery_speed=float(self.get_parameter(
                'straight_recovery_speed').value),
            strong_threshold=float(self.get_parameter(
                'curve_strong_threshold').value),
            medium_threshold=float(self.get_parameter(
                'curve_medium_threshold').value),
            straight_enter_cte_px=float(self.get_parameter(
                'straight_enter_cte_px').value),
            straight_exit_cte_px=float(self.get_parameter(
                'straight_exit_cte_px').value),
            straight_enter_heading_deg=float(self.get_parameter(
                'straight_enter_heading_deg').value),
            straight_exit_heading_deg=float(self.get_parameter(
                'straight_exit_heading_deg').value),
            straight_enter_risk=float(self.get_parameter(
                'straight_enter_curvature_risk').value),
            straight_exit_risk=float(self.get_parameter(
                'straight_exit_curvature_risk').value),
            straight_confirm_frames=max(1, int(self.get_parameter(
                'straight_confirm_frames').value)),
            curve_confirm_frames=max(1, int(self.get_parameter(
                'curve_confirm_frames').value)),
            curvature_filter_window=max(1, int(self.get_parameter(
                'curvature_filter_window').value)),
            curvature_scale=max(1e-6, float(self.get_parameter(
                'curvature_scale').value)),
            curve_accel_rate_per_s=max(0.0, float(self.get_parameter(
                'curve_accel_rate_per_s').value)),
            straight_accel_rate_per_s=max(0.0, float(
                self.get_parameter('straight_accel_rate_per_s').value)),
            curve_decel_rate_per_s=max(0.0, float(self.get_parameter(
                'curve_decel_rate_per_s').value)),
            straight_decel_rate_per_s=max(0.0, float(self.get_parameter(
                'straight_decel_rate_per_s').value)),
        )
        self.curve_speed_policy = CurveSpeedPolicy(curve_speed_config)
        self.MIN_SPEED = float(curve_speed_config.strong_curve_speed)
        steering_limiter_config = CurvatureSteeringLimiterConfig(
            enabled=self.as_bool(self.get_parameter(
                'curvature_steering_rate_enabled').value),
            straight_rate_per_s=max(0.0, float(self.get_parameter(
                'steering_rate_straight_per_s').value)),
            curve_rate_per_s=max(0.0, float(self.get_parameter(
                'steering_rate_curve_per_s').value)),
            reversal_rate_per_s=max(0.0, float(self.get_parameter(
                'steering_rate_reversal_per_s').value)),
            risk_low=float(self.get_parameter(
                'steering_rate_risk_low').value),
            risk_high=float(self.get_parameter(
                'steering_rate_risk_high').value),
            hold_frames=max(0, int(self.get_parameter(
                'steering_rate_hold_frames').value)),
            curvature_filter_window=curve_speed_config.curvature_filter_window,
            curvature_scale=curve_speed_config.curvature_scale,
            legacy_max_delta=self.max_delta_angle,
        )
        self.curvature_steering_limiter = CurvatureSteeringLimiter(
            steering_limiter_config)

        self.TARGET_TIME_GAP = 4.0
        self.OBSTACLE_DETECTION_THRESHOLD = 4.0
        self.CURVE_OVERTAKE_THRESHOLD_M = 1.0
        self.KP_SPEED_CONTROL = 0.5

        self.is_active = False
        self.last_steering_deg = 0.0
        self.angle_buffer = deque(maxlen=self.window)
        self.current_k = self.k_straight
        self.current_mode = 'center'
        self.current_speed = self.MIN_SPEED
        self.latest_xycar_state = None
        self.latest_state_source_header = Header()
        self.latest_state_source_ns = 0
        self.latest_state_received_ns = 0
        self.latest_state_received_monotonic = 0.0
        self.latest_control_metadata = self.default_control_metadata()
        self.last_controlled_source_ns = 0
        self.last_claimed_state_source_ns = 0
        self.last_source_dt_s = self.nominal_source_dt_s
        self.state_lock = threading.Lock()
        self.control_execution_lock = threading.Lock()
        self.last_mission_mode = ''
        mission_mode_qos = QoSProfile(depth=1)
        mission_mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mission_mode_subscription = self.create_subscription(
            String,
            self.get_parameter('mission_mode_topic').value,
            self.mission_mode_callback,
            mission_mode_qos,
            callback_group=self.state_update_callback_group,
        )
        self.state_receive_count = 0
        self.state_update_count = 0
        self.duplicate_source_reject_count = 0
        self.out_of_order_source_reject_count = 0
        self.state_replaced_count = 0
        self.unique_source_control_count = 0
        self.same_source_heartbeat_count = 0
        self.control_output_count = 0

        self.relative_distance = self.OBSTACLE_DETECTION_THRESHOLD + 1.0
        self.obstacle_position = 'none'
        self.detected_vehicle_count = 0
        self.is_obstacle_detected = False
        self.curve_state = 'Straight'
        self.traffic_state = 'unknown'
        self.condition_to_follow = False
        self.shortcut_speed_limit = -1.0
        self.shortcut_speed_limit_received_monotonic = 0.0
        self.obstacle_speed_limit = -1.0
        self.obstacle_speed_limit_received_monotonic = 0.0
        self.shortcut_path_active = False
        self.shortcut_speed_limit_sub = self.create_subscription(
            Float32,
            self.get_parameter('shortcut_speed_limit_topic').value,
            self.shortcut_speed_limit_callback,
            10,
            callback_group=self.state_update_callback_group,
        )
        self.shortcut_path_active_sub = self.create_subscription(
            Bool,
            self.get_parameter('shortcut_path_active_topic').value,
            self.shortcut_path_active_callback,
            10,
            callback_group=self.state_update_callback_group,
        )
        self.obstacle_speed_limit_sub = self.create_subscription(
            Float32,
            self.get_parameter('obstacle_speed_limit_topic').value,
            self.obstacle_speed_limit_callback,
            10,
            callback_group=self.state_update_callback_group,
        )

        self.log_counter = 0
        self.log_interval = 5
        self.stale_log_counter = 0

        timer_period = 1.0 / max(self.control_rate_hz, 1.0)
        self.control_timer = self.create_timer(
            timer_period,
            self.control_timer_callback,
            callback_group=self.control_callback_group,
        )

        if self.use_lane_control_state_v3:
            state_input_name = 'lane_control_v3'
        elif self.use_lane_control_state_v2:
            state_input_name = 'lane_control_v2'
        elif self.use_lane_control_state:
            state_input_name = 'lane_control'
        elif self.use_stamped_state:
            state_input_name = 'stamped'
        else:
            state_input_name = 'legacy'
        self.get_logger().info(
            'Integrated Stanley Controller started. '
            f'straight_speed={self.speed_straight:.1f}, '
            f'control_rate={self.control_rate_hz:.1f}Hz, '
            f'event_driven={self.event_driven_control}, '
            f'state_timeout={self.state_timeout_s:.2f}s, '
            f'state_input={state_input_name}, '
            f'scene_inputs={self.enable_scene_inputs}, '
            f'respect_traffic_light={self.respect_traffic_light}, '
            f'cte_gain_schedule={self.gain_schedule_start_speed:.1f}-'
            f'{self.gain_schedule_end_speed:.1f}, '
            f'heading_weight={self.heading_weight_low_speed:.2f}->'
            f'{self.heading_weight_straight_high_speed:.2f}/'
            f'{self.heading_weight_curve_high_speed:.2f}, '
            'curve_speed_bands='
            f'{curve_speed_config.strong_curve_speed:.1f}-'
            f'{curve_speed_config.medium_curve_speed:.1f}-'
            f'{curve_speed_config.mild_curve_speed:.1f}, '
            f'recovery_speed={curve_speed_config.recovery_speed:.1f}, '
            'lane_decel(straight/curve)='
            f'{curve_speed_config.straight_decel_rate_per_s:.1f}/'
            f'{curve_speed_config.curve_decel_rate_per_s:.1f}, '
            'curvature_steering_rate='
            f'{steering_limiter_config.enabled}/'
            f'{steering_limiter_config.straight_rate_per_s:.0f}-'
            f'{steering_limiter_config.curve_rate_per_s:.0f}-'
            f'{steering_limiter_config.reversal_rate_per_s:.0f}'
        )

    def start_service_callback(self, request, response):
        if not self.is_active:
            self.is_active = True
            response.success = True
            response.message = 'Integrated Stanley Controller started.'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'Already active.'
            self.get_logger().warn(response.message)
        return response

    def mission_mode_callback(self, msg: String):
        new_mode = msg.data.strip().upper()
        if new_mode == 'STOP' and self.last_mission_mode != 'STOP':
            with self.control_execution_lock:
                self.curve_speed_policy.reset()
                self.curvature_steering_limiter.reset()
                self.last_steering_deg = 0.0
                self.angle_buffer.clear()
                self.current_k = self.k_straight
                self.current_heading_weight = self.heading_weight_low_speed
                self.current_mode = 'center'
                self.current_speed = self.MIN_SPEED
                self.last_controlled_source_ns = 0
                self.last_claimed_state_source_ns = 0
                self.last_source_dt_s = self.nominal_source_dt_s
                self.relative_distance = (
                    self.OBSTACLE_DETECTION_THRESHOLD + 1.0)
                self.obstacle_position = 'none'
                self.detected_vehicle_count = 0
                self.is_obstacle_detected = False
                self.condition_to_follow = False
                self.traffic_state = 'unknown'
                self.obstacle_speed_limit = -1.0
                self.obstacle_speed_limit_received_monotonic = 0.0
            self.get_logger().info(
                'Stanley steering/speed planner state reset by drive STOP.')
        self.last_mission_mode = new_mode

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def shortcut_speed_limit_callback(self, message):
        self.shortcut_speed_limit = float(message.data)
        self.shortcut_speed_limit_received_monotonic = time.monotonic()

    def shortcut_path_active_callback(self, message):
        self.shortcut_path_active = bool(message.data)

    def obstacle_speed_limit_callback(self, message):
        self.obstacle_speed_limit = float(message.data)
        self.obstacle_speed_limit_received_monotonic = time.monotonic()

    def current_shortcut_speed_limit(self):
        if self.shortcut_speed_limit < 0.0:
            return None
        age_s = (
            time.monotonic()
            - self.shortcut_speed_limit_received_monotonic
        )
        if age_s > self.shortcut_speed_limit_timeout_s:
            return None
        return max(0.0, float(self.shortcut_speed_limit))

    def current_obstacle_speed_limit(self):
        """Return a fresh obstacle profile cap or None when inactive."""
        if self.obstacle_speed_limit < 0.0:
            return None
        age_s = (
            time.monotonic()
            - self.obstacle_speed_limit_received_monotonic
        )
        if age_s > self.obstacle_speed_limit_timeout_s:
            return None
        return max(0.0, float(self.obstacle_speed_limit))

    @staticmethod
    def clamp_angle(angle):
        return max(min(float(angle), 100.0), -100.0)

    @staticmethod
    def apply_shortcut_steering_limit(angle, active, limit):
        if not active or float(limit) <= 0.0:
            return float(angle)
        return float(np.clip(float(angle), -float(limit), float(limit)))

    @staticmethod
    def default_control_metadata():
        return {
            'curve_state': 'Straight',
            'heading_source': 'legacy',
            'curve_heading_valid': False,
            'signed_curvature_hint': float('nan'),
            'preview_signed_curvature_hint': float('nan'),
            'curvature_severity': 0.0,
            'path_confidence': 0.0,
        }

    def speed_schedule_ratio(self):
        return float(np.clip(
            (
                abs(self.current_speed)
                - self.gain_schedule_start_speed
            ) / (
                self.gain_schedule_end_speed
                - self.gain_schedule_start_speed
            ),
            0.0,
            1.0,
        ))

    def compute_steering_angle_stanley(
            self, lane_center_x, lane_angle_rad, k):
        cte_pixel = float(lane_center_x) - self.image_center_x
        cte = cte_pixel * self.pixel_to_meter
        heading_error = float(lane_angle_rad)
        raw_cte_term = np.arctan2(k * cte, 3.0)
        heading_term = self.current_heading_weight * heading_error
        cte_term = raw_cte_term
        heading_cte_conflict = heading_term * raw_cte_term < 0.0
        self.last_raw_cte_term = float(raw_cte_term)
        self.last_heading_cte_conflict = bool(heading_cte_conflict)
        steering_angle_rad = heading_term + cte_term
        servo_angle = (
            np.degrees(steering_angle_rad) * self.deg_to_servo_scale
            + self.deg_to_servo_offset
        )
        return servo_angle, heading_error, cte_term

    def limit_angle_change(self, current, last):
        delta = float(current) - float(last)
        delta = max(-self.max_delta_angle, min(self.max_delta_angle, delta))
        return float(last) + delta

    def smooth_angle(self, new_angle):
        self.angle_buffer.append(float(new_angle))
        return sum(self.angle_buffer) / len(self.angle_buffer)

    def update_scheduled_cross_track_gain(self):
        """Update Stanley CTE gain without changing its fixed denominator."""
        speed_ratio = self.speed_schedule_ratio()
        if self.is_obstacle_detected:
            self.current_k = self.k_obstacle
        elif self.curve_state.strip().lower() == 'curve':
            self.current_k = float(np.interp(
                speed_ratio,
                [0.0, 1.0],
                [self.k_curve_low_speed, self.k_curve_high_speed],
            ))
        else:
            self.current_k = float(np.interp(
                speed_ratio,
                [0.0, 1.0],
                [self.k_straight_low_speed,
                 self.k_straight_high_speed],
            ))
        return speed_ratio

    def update_scheduled_heading_weight(self, metadata=None):
        """Damp only straight heading noise and recover curve authority fast."""
        if not self.adaptive_heading_weight_enabled:
            self.current_heading_weight = self.heading_weight
            return self.current_heading_weight

        metadata = metadata or self.default_control_metadata()
        speed_ratio = self.speed_schedule_ratio()
        straight_weight = float(np.interp(
            speed_ratio,
            [0.0, 1.0],
            [self.heading_weight_low_speed,
             self.heading_weight_straight_high_speed],
        ))
        hough_weight = float(np.interp(
            speed_ratio,
            [0.0, 1.0],
            [self.heading_weight_low_speed,
             self.heading_weight_hough_high_speed],
        ))
        curve_weight = float(np.interp(
            speed_ratio,
            [0.0, 1.0],
            [self.heading_weight_low_speed,
             self.heading_weight_curve_high_speed],
        ))

        curve_state = str(metadata.get(
            'curve_state', self.curve_state)).strip().lower()
        heading_source = str(metadata.get(
            'heading_source', 'legacy')).strip().lower()
        severity = float(np.clip(
            metadata.get('curvature_severity', 0.0), 0.0, 1.0))
        if curve_state == 'curve':
            # A confirmed curve gets full curve heading authority.
            severity = 1.0
        target_weight = (
            straight_weight
            + (curve_weight - straight_weight) * severity
        )
        if heading_source == 'hough' and curve_state != 'curve':
            target_weight = min(target_weight, hough_weight)
        alpha = (
            self.heading_weight_rise_alpha
            if target_weight > self.current_heading_weight
            else self.heading_weight_fall_alpha
        )
        self.current_heading_weight += alpha * (
            target_weight - self.current_heading_weight)
        return self.current_heading_weight

    def calculate_final_speed(
            self, target_speed, source_dt_s=None,
            accel_rate_per_s=None, decel_rate_per_s=None):
        # Obstacle longitudinal behavior is owned by /obstacle_speed_limit.
        # Keeping the former time-gap branch here would create a second,
        # conflicting speed authority.
        self.condition_to_follow = False
        speed_diff = target_speed - self.current_speed
        source_dt_s = (
            self.nominal_source_dt_s
            if source_dt_s is None
            else float(np.clip(
                source_dt_s,
                self.source_dt_min_s,
                self.source_dt_max_s,
            ))
        )
        if speed_diff > 0:
            accel_rate = (
                self.accel_rate_per_s
                if accel_rate_per_s is None
                else max(0.0, float(accel_rate_per_s))
            )
            accel_step = accel_rate * source_dt_s
            return float(min(
                target_speed,
                self.current_speed + accel_step,
            )), 'ACCELERATING'
        if speed_diff < 0:
            decel_rate = (
                self.decel_rate_per_s
                if decel_rate_per_s is None
                else max(0.0, float(decel_rate_per_s))
            )
            decel_step = decel_rate * source_dt_s
            return float(max(
                target_speed,
                self.current_speed - decel_step,
            )), 'DECELERATING'
        return float(self.current_speed), 'MAINTAINING'

    def curve_callback(self, msg: Curve):
        self.curve_state = msg.state

    def traffic_state_callback(self, msg: String):
        self.traffic_state = msg.data.strip().lower()

    def obstacle_state_callback(self, msg: ObstacleState):
        self.relative_distance = float(msg.distance_m)
        self.obstacle_position = msg.position.strip().lower()
        self.detected_vehicle_count = int(msg.vehicle_count)
        self.is_obstacle_detected = self.relative_distance < self.OBSTACLE_DETECTION_THRESHOLD

    @staticmethod
    def stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def update_latest_state(
            self, state, source_header, source_ns, received_ns,
            received_monotonic, metadata):
        """Install one newer source without allowing stale overwrite."""
        duplicate = False
        out_of_order = False
        with self.state_lock:
            self.state_receive_count += 1
            if source_ns > 0 and self.latest_state_source_ns > 0:
                duplicate = source_ns == self.latest_state_source_ns
                out_of_order = source_ns < self.latest_state_source_ns
                if duplicate:
                    self.duplicate_source_reject_count += 1
                elif out_of_order:
                    self.out_of_order_source_reject_count += 1
            if duplicate or out_of_order:
                accepted = False
            else:
                if (
                    self.latest_state_source_ns > 0
                    and self.latest_state_source_ns
                    != self.last_claimed_state_source_ns
                ):
                    self.state_replaced_count += 1
                self.latest_xycar_state = state
                self.latest_state_source_header = source_header
                self.latest_state_source_ns = source_ns
                self.latest_state_received_ns = received_ns
                self.latest_state_received_monotonic = received_monotonic
                self.latest_control_metadata = dict(metadata)
                self.state_update_count += 1
                accepted = True
        if not accepted:
            self.publish_state_reject_profile(
                source_header=source_header,
                received_ns=received_ns,
                duplicate=duplicate,
            )
        return accepted

    def publish_state_reject_profile(
            self, source_header, received_ns, duplicate):
        """Expose a rejected duplicate/out-of-order callback if profiling."""
        if self.pipeline_timing_pub is None:
            return
        timing = PipelineTiming()
        if source_header is not None:
            timing.header = source_header
        timing.stage = 'stanley_controller_state_reject'
        timing.receive_stamp_ns = int(max(0, received_ns))
        timing.start_stamp_ns = int(max(0, received_ns))
        timing.end_stamp_ns = int(max(0, received_ns))
        timing.queue_wait_ms = 0.0
        timing.compute_ms = 0.0
        timing.received_count = int(self.state_receive_count)
        timing.processed_count = int(self.state_update_count)
        timing.replaced_count = int(
            self.duplicate_source_reject_count
            + self.out_of_order_source_reject_count)
        # True identifies an equal source; False identifies an older source.
        timing.input_reused = bool(duplicate)
        self.pipeline_timing_pub.publish(timing)

    def lane_control_state_callback(self, msg: LaneControlState):
        received_ns = self.get_clock().now().nanoseconds
        received_monotonic = time.monotonic()
        metadata = {
            'curve_state': msg.curve_state,
            'heading_source': msg.heading_source,
            'curve_heading_valid': bool(msg.curve_heading_valid),
            'signed_curvature_hint': float('nan'),
            'preview_signed_curvature_hint': float('nan'),
            'curvature_severity': float(msg.curvature_severity),
            'path_confidence': float(msg.path_confidence),
        }
        accepted = self.update_latest_state(
            state=msg.state,
            source_header=msg.header,
            source_ns=self.stamp_to_ns(msg.header.stamp),
            received_ns=received_ns,
            received_monotonic=received_monotonic,
            metadata=metadata,
        )
        self.trigger_event_control_if_enabled(accepted)

    def lane_control_state_v2_callback(self, msg: LaneControlStateV2):
        received_ns = self.get_clock().now().nanoseconds
        received_monotonic = time.monotonic()
        control = msg.control
        metadata = {
            'curve_state': control.curve_state,
            'heading_source': control.heading_source,
            'curve_heading_valid': bool(control.curve_heading_valid),
            'signed_curvature_hint': float(msg.signed_curvature_hint),
            'preview_signed_curvature_hint': float('nan'),
            'curvature_severity': float(control.curvature_severity),
            'path_confidence': float(control.path_confidence),
        }
        accepted = self.update_latest_state(
            state=control.state,
            source_header=control.header,
            source_ns=self.stamp_to_ns(control.header.stamp),
            received_ns=received_ns,
            received_monotonic=received_monotonic,
            metadata=metadata,
        )
        self.trigger_event_control_if_enabled(accepted)

    def lane_control_state_v3_callback(self, msg: LaneControlStateV3):
        received_ns = self.get_clock().now().nanoseconds
        received_monotonic = time.monotonic()
        extended = msg.control
        control = extended.control
        metadata = {
            'curve_state': control.curve_state,
            'heading_source': control.heading_source,
            'curve_heading_valid': bool(control.curve_heading_valid),
            'signed_curvature_hint': float(
                extended.signed_curvature_hint),
            'preview_signed_curvature_hint': float(
                msg.preview_signed_curvature_hint),
            'curvature_severity': float(control.curvature_severity),
            'path_confidence': float(control.path_confidence),
        }
        accepted = self.update_latest_state(
            state=control.state,
            source_header=control.header,
            source_ns=self.stamp_to_ns(control.header.stamp),
            received_ns=received_ns,
            received_monotonic=received_monotonic,
            metadata=metadata,
        )
        self.trigger_event_control_if_enabled(accepted)

    def stamped_xycar_state_callback(self, msg: StampedXycarState):
        received_ns = self.get_clock().now().nanoseconds
        received_monotonic = time.monotonic()
        metadata = self.default_control_metadata()
        metadata['curve_state'] = self.curve_state
        accepted = self.update_latest_state(
            state=msg.state,
            source_header=msg.header,
            source_ns=self.stamp_to_ns(msg.header.stamp),
            received_ns=received_ns,
            received_monotonic=received_monotonic,
            metadata=metadata,
        )
        self.trigger_event_control_if_enabled(accepted)

    def xycar_state_callback(self, msg: XycarState):
        # A legacy state has no source stamp. Arrival time is retained only for
        # explicitly requested legacy compatibility mode.
        now = self.get_clock().now()
        source_header = Header()
        source_header.stamp = now.to_msg()
        source_header.frame_id = 'legacy_arrival_time'
        metadata = self.default_control_metadata()
        metadata['curve_state'] = self.curve_state
        accepted = self.update_latest_state(
            state=msg,
            source_header=source_header,
            source_ns=now.nanoseconds,
            received_ns=now.nanoseconds,
            received_monotonic=time.monotonic(),
            metadata=metadata,
        )
        self.trigger_event_control_if_enabled(accepted)

    def control_timer_callback(self):
        with self.control_execution_lock:
            self.run_control_once()

    def trigger_event_control_if_enabled(self, state_accepted=True):
        if self.event_driven_control and state_accepted:
            self.control_timer_callback()

    def run_control_once(self):
        if not self.is_active:
            return

        timer_start_monotonic = time.monotonic()
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        with self.state_lock:
            state_msg = self.latest_xycar_state
            source_header = self.latest_state_source_header
            source_ns = self.latest_state_source_ns
            state_received_ns = self.latest_state_received_ns
            state_received_monotonic = (
                self.latest_state_received_monotonic)
            control_metadata = dict(self.latest_control_metadata)
            # Claim the snapshotted source while holding the state lock.  A
            # callback may replace a newer unclaimed slot during computation,
            # but must not count this in-flight source as dropped.
            self.last_claimed_state_source_ns = source_ns
            state_age_s = (
                (now_ns - source_ns) * 1e-9
                if source_ns > 0 else float('inf')
            )

        if self.respect_traffic_light and self.traffic_state == 'red':
            self.publish_motor(
                0.0,
                0.0,
                source_header=source_header,
                state_age_s=state_age_s,
            )
            self.current_speed = 0.0
            return

        source_is_fresh = (
            source_ns > 0
            and state_age_s >= -self.future_stamp_tolerance_s
            and state_age_s <= self.state_timeout_s
        )
        if state_msg is None or not source_is_fresh:
            self.curvature_steering_limiter.reset()
            safe_angle = (
                self.last_steering_deg
                if self.hold_last_angle_when_stale else 0.0
            )
            self.current_speed = 0.0
            self.publish_motor(
                safe_angle,
                0.0,
                source_header=source_header,
                state_age_s=state_age_s,
            )
            self.stale_log_counter += 1
            if self.verbose_status and self.stale_log_counter % 20 == 0:
                self.get_logger().warn(
                    'No fresh /xycar_state. '
                    f'age={state_age_s:.2f}s, publishing stop command.'
                )
            return

        self.stale_log_counter = 0
        self.publish_control_from_state(
            state_msg,
            source_header=source_header,
            source_ns=source_ns,
            state_age_s=state_age_s,
            control_metadata=control_metadata,
            state_received_ns=state_received_ns,
            state_received_monotonic=state_received_monotonic,
            timer_start_ns=now_ns,
            timer_start_monotonic=timer_start_monotonic,
        )

    def publish_control_from_state(
            self, state_msg: XycarState, source_header=None,
            source_ns=0, state_age_s=-1.0, control_metadata=None,
            state_received_ns=0, state_received_monotonic=0.0,
            timer_start_ns=0, timer_start_monotonic=0.0):
        self.current_mode = state_msg.drive_mode.strip().lower()
        lane_center_x = state_msg.target_point.x
        lane_angle_rad = state_msg.target_point.z

        control_metadata = (
            dict(control_metadata)
            if control_metadata is not None
            else self.default_control_metadata()
        )
        if control_metadata.get('heading_source') == 'legacy':
            control_metadata['curve_state'] = self.curve_state
        self.curve_state = str(control_metadata.get(
            'curve_state', self.curve_state))

        is_new_source = (
            source_ns <= 0 or source_ns != self.last_controlled_source_ns
        )
        if not is_new_source:
            # The control timer may run faster than perception. Reusing a state
            # must not advance steering, adaptive gains, or acceleration.
            if self.relative_distance == 0.0:
                self.current_speed = 0.0
            self.publish_motor(
                self.last_steering_deg,
                self.current_speed,
                source_header=source_header,
                state_age_s=state_age_s,
            )
            self.same_source_heartbeat_count += 1
            self.publish_pipeline_profile(
                source_header=source_header,
                state_received_ns=state_received_ns,
                state_received_monotonic=state_received_monotonic,
                timer_start_ns=timer_start_ns,
                timer_start_monotonic=timer_start_monotonic,
                input_reused=True,
            )
            return

        source_dt_s = self.nominal_source_dt_s
        if source_ns > 0 and self.last_controlled_source_ns > 0:
            raw_source_dt_s = (
                source_ns - self.last_controlled_source_ns) * 1e-9
            if raw_source_dt_s > 0.0:
                source_dt_s = float(np.clip(
                    raw_source_dt_s,
                    self.source_dt_min_s,
                    self.source_dt_max_s,
                ))
        self.last_source_dt_s = source_dt_s

        # Use the latest curve state and previous cycle's commanded speed for
        # this frame.  Previously current_k was changed only after steering was
        # computed, so curve transitions always used a one-frame-old gain.
        self.update_scheduled_cross_track_gain()
        self.update_scheduled_heading_weight(control_metadata)
        servo_angle, heading_error_rad, cte_term = self.compute_steering_angle_stanley(
            lane_center_x=lane_center_x,
            lane_angle_rad=lane_angle_rad,
            k=self.current_k,
        )

        steering_limit = self.curvature_steering_limiter.limit(
            raw_angle=servo_angle,
            last_angle=self.last_steering_deg,
            source_dt_s=source_dt_s,
            current_curvature=control_metadata.get(
                'signed_curvature_hint', float('nan')),
            preview_curvature=control_metadata.get(
                'preview_signed_curvature_hint', float('nan')),
            current_risk_hint=control_metadata.get(
                'curvature_severity', 0.0),
        )
        rate_limited = steering_limit.rate_limited
        limited_angle = steering_limit.angle
        smoothed_angle = self.smooth_angle(limited_angle)
        final_angle = self.clamp_angle(smoothed_angle)
        final_angle = self.apply_shortcut_steering_limit(
            final_angle,
            self.shortcut_path_active,
            self.shortcut_steering_limit,
        )
        self.last_steering_deg = final_angle
        if source_ns > 0:
            self.last_controlled_source_ns = source_ns
        self.unique_source_control_count += 1

        curve_speed_decision = self.curve_speed_policy.update(
            curve_state=self.curve_state,
            cte_px=float(lane_center_x) - self.image_center_x,
            heading_error_rad=heading_error_rad,
            signed_curvature=control_metadata.get(
                'signed_curvature_hint', float('nan')),
            preview_signed_curvature=control_metadata.get(
                'preview_signed_curvature_hint', float('nan')),
            curvature_severity=control_metadata.get(
                'curvature_severity', 0.0),
        )
        target_speed = curve_speed_decision.target_speed
        lane_speed_cap = curve_speed_decision.hard_speed_cap
        shortcut_speed_limit = self.current_shortcut_speed_limit()
        if shortcut_speed_limit is not None:
            target_speed = min(target_speed, shortcut_speed_limit)
        obstacle_speed_limit = self.current_obstacle_speed_limit()
        if obstacle_speed_limit is not None:
            target_speed = min(target_speed, obstacle_speed_limit)

        if self.relative_distance == 0.0:
            final_speed = 0.0
            speed_status = 'EMERGENCY STOP'
        else:
            final_speed, speed_status = self.calculate_final_speed(
                target_speed,
                source_dt_s,
                accel_rate_per_s=curve_speed_decision.accel_rate_per_s,
                decel_rate_per_s=curve_speed_decision.decel_rate_per_s,
            )
            if (
                np.isfinite(lane_speed_cap)
                and final_speed > lane_speed_cap
            ):
                # A newly selected curve band is an immediate safety ceiling.
                final_speed = lane_speed_cap
                speed_status = 'CURVE_POLICY_SPEED_CAPPED'
            if (
                shortcut_speed_limit is not None
                and final_speed > shortcut_speed_limit
            ):
                final_speed = shortcut_speed_limit
                speed_status = 'SHORTCUT_SPEED_CAPPED'
            if (
                obstacle_speed_limit is not None
                and final_speed > obstacle_speed_limit
            ):
                final_speed = obstacle_speed_limit
                speed_status = 'OBSTACLE_PROFILE_SPEED_CAPPED'

        self.current_speed = final_speed
        self.publish_motor(
            final_angle,
            final_speed,
            source_header=source_header,
            state_age_s=state_age_s,
        )
        self.publish_pipeline_profile(
            source_header=source_header,
            state_received_ns=state_received_ns,
            state_received_monotonic=state_received_monotonic,
            timer_start_ns=timer_start_ns,
            timer_start_monotonic=timer_start_monotonic,
            input_reused=False,
        )
        self.publish_debug(
            source_header=source_header,
            metadata=control_metadata,
            lane_center_x=lane_center_x,
            heading_error_rad=heading_error_rad,
            cte_term=cte_term,
            raw_servo_angle=servo_angle,
            final_angle=final_angle,
            target_speed=target_speed,
            final_speed=final_speed,
            curve_speed_decision=curve_speed_decision,
            speed_status=speed_status,
            source_dt_s=source_dt_s,
            state_age_s=state_age_s,
            rate_limited=rate_limited,
        )

        self.log_counter += 1
        if (
            self.verbose_status
            and self.log_counter % self.log_interval == 0
        ):
            self.log_status(
                heading_error_rad,
                cte_term,
                self.current_k,
                final_angle,
                target_speed,
                final_speed,
                speed_status,
                rate_limited,
            )

    def publish_pipeline_profile(
            self, source_header, state_received_ns,
            state_received_monotonic, timer_start_ns,
            timer_start_monotonic, input_reused):
        if self.pipeline_timing_pub is None:
            return
        end_monotonic = time.monotonic()
        end_ns = self.get_clock().now().nanoseconds
        self.control_output_count += 1
        timing = PipelineTiming()
        if source_header is not None:
            timing.header = source_header
        timing.stage = 'stanley_controller'
        timing.receive_stamp_ns = int(max(0, state_received_ns))
        timing.start_stamp_ns = int(max(0, timer_start_ns))
        timing.end_stamp_ns = int(end_ns)
        timing.queue_wait_ms = max(
            0.0,
            (timer_start_monotonic - state_received_monotonic) * 1000.0,
        )
        timing.compute_ms = max(
            0.0,
            (end_monotonic - timer_start_monotonic) * 1000.0,
        )
        timing.received_count = int(self.state_receive_count)
        timing.processed_count = int(self.unique_source_control_count)
        timing.replaced_count = int(self.state_replaced_count)
        timing.input_reused = bool(input_reused)
        self.pipeline_timing_pub.publish(timing)

    def publish_debug(
            self, source_header, metadata, lane_center_x,
            heading_error_rad, cte_term, raw_servo_angle, final_angle,
            target_speed, final_speed, curve_speed_decision, speed_status,
            source_dt_s, state_age_s, rate_limited):
        msg = StanleyDebug()
        if source_header is not None:
            msg.header = source_header
        msg.curve_state = str(metadata.get(
            'curve_state', self.curve_state))
        msg.heading_source = str(metadata.get('heading_source', 'legacy'))
        msg.curvature_severity = float(np.clip(
            metadata.get('curvature_severity', 0.0), 0.0, 1.0))
        msg.path_confidence = float(np.clip(
            metadata.get('path_confidence', 0.0), 0.0, 1.0))
        msg.cte_pixels = float(lane_center_x) - self.image_center_x
        msg.cte_m = msg.cte_pixels * self.pixel_to_meter
        msg.cte_gain = float(self.current_k)
        msg.heading_weight = float(self.current_heading_weight)
        msg.heading_error_rad = float(heading_error_rad)
        msg.heading_term_rad = float(
            self.current_heading_weight * heading_error_rad)
        msg.cte_term_rad = float(cte_term)
        msg.raw_servo_angle = float(raw_servo_angle)
        msg.final_servo_angle = float(final_angle)
        msg.target_speed = float(target_speed)
        msg.final_speed = float(final_speed)
        msg.source_dt_s = float(source_dt_s)
        msg.state_age_s = (
            float(state_age_s) if np.isfinite(state_age_s) else -1.0)
        msg.steering_rate_limited = bool(rate_limited)
        self.debug_pub.publish(msg)
        extended = None
        if (
            self.debug_v2_pub is not None
            or self.debug_v3_pub is not None
            or self.debug_v4_pub is not None
        ):
            extended = StanleyDebugV2()
            extended.debug = msg
            extended.raw_cte_term_rad = float(self.last_raw_cte_term)
            if self.debug_v2_pub is not None:
                self.debug_v2_pub.publish(extended)
        planner_debug = None
        if (
            (self.debug_v3_pub is not None or self.debug_v4_pub is not None)
            and extended is not None
        ):
            planner_debug = StanleyDebugV3()
            planner_debug.control = extended
            # This legacy field now carries the direct four-band mode. It never
            # contains lifecycle phases.
            planner_debug.speed_planner_phase = str(
                curve_speed_decision.mode)
            planner_debug.signed_curvature = float(
                curve_speed_decision.signed_curvature)
            planner_debug.preview_signed_curvature = float(
                curve_speed_decision.preview_signed_curvature)
            planner_debug.filtered_curvature_risk = float(
                curve_speed_decision.filtered_curvature_risk)
            planner_debug.curvature_trend = 0.0
            planner_debug.peak_curvature_risk = float(
                curve_speed_decision.filtered_curvature_risk)
            planner_debug.baseline_target_speed = float(
                curve_speed_decision.target_speed)
            planner_debug.planner_target_speed = float(target_speed)
            planner_debug.fast_alignment = bool(
                curve_speed_decision.straight_alignment)
            planner_debug.curve_evidence = bool(
                curve_speed_decision.curve_evidence)
            if self.debug_v3_pub is not None:
                self.debug_v3_pub.publish(planner_debug)
        if self.debug_v4_pub is not None and planner_debug is not None:
            audit = StanleyDebugV4()
            audit.planner = planner_debug
            audit.speed_reason = (
                'SAFETY_STOP'
                if speed_status == 'EMERGENCY STOP'
                else curve_speed_decision.mode
            )
            audit.geometry_target_speed = float(
                curve_speed_decision.target_speed)
            audit.speed_before_rate_limit = float(target_speed)
            audit.speed_after_rate_limit = float(final_speed)
            audit.active_speed_cap = float(
                curve_speed_decision.hard_speed_cap
                if np.isfinite(curve_speed_decision.hard_speed_cap)
                else -1.0
            )
            audit.heading_cte_conflict = bool(
                self.last_heading_cte_conflict)
            self.debug_v4_pub.publish(audit)

    def publish_motor(
            self, angle, speed, source_header=None, state_age_s=-1.0):
        legacy_msg = Float32MultiArray()
        legacy_msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(legacy_msg)

        stamped_msg = StampedMotorCommand()
        if source_header is not None:
            stamped_msg.header = source_header
        stamped_msg.command_stamp = self.get_clock().now().to_msg()
        stamped_msg.state_age_s = (
            float(state_age_s) if np.isfinite(state_age_s) else -1.0
        )
        stamped_msg.angle = float(angle)
        stamped_msg.speed = float(speed)
        self.stamped_motor_pub.publish(stamped_msg)

    def log_status(self, heading_error_rad, cte_term, current_k, final_angle,
                   target_speed, final_speed, speed_status, rate_limited):
        rate_marker = ' RATE_LIMITED' if rate_limited else ''
        steering_limit = self.curvature_steering_limiter.last_result
        reversal_marker = (
            ' REVERSAL' if steering_limit.reversal_applied else '')
        obstacle_status = (
            f'Dist: {self.relative_distance:.2f}m'
            if self.is_obstacle_detected else 'Clear'
        )
        self.get_logger().info(
            '\n[Integrated Stanley Controller]\n'
            f'  Lane Target      : {self.current_mode.upper()}\n'
            f'  Curve State      : {self.curve_state}\n'
            f'  Traffic State    : {self.traffic_state}\n'
            f'  Obstacle Status  : {obstacle_status}\n'
            f'  Stanley K        : {current_k:>6.2f}\n'
            f'  Heading Weight  : {self.current_heading_weight:>6.2f}\n'
            f'  Heading Error    : {np.degrees(heading_error_rad):>6.2f} deg\n'
            f'  Cross Track Term : {np.degrees(cte_term):>6.2f} deg\n'
            f'  Servo Angle      : {final_angle:>6.2f}{rate_marker}\n'
            f'  Steering Risk    : '
            f'{steering_limit.effective_risk:>6.3f}\n'
            f'  Steering Rate    : '
            f'{steering_limit.rate_per_s:>6.1f}/s, '
            f'max_delta={steering_limit.max_delta:.2f}'
            f'{reversal_marker}\n'
            f'  Speed Band       : '
            f'{self.curve_speed_policy.mode}\n'
            f'  Target Speed     : {target_speed:>6.2f}\n'
            f'  Final Speed      : {final_speed:>6.2f} {speed_status}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedStanleyController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
