#! /usr/bin/env python
# -*- coding:utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String
import cv2
import numpy as np
from collections import deque
from custom_interfaces.msg import (
    Curve,
    Detection,
    Detections,
    LaneControlState,
    LaneControlStateV2,
    LaneControlStateV3,
    PipelineTiming,
    StampedXycarState,
    XycarState,
)
import message_filters
from std_srvs.srv import Trigger
from datetime import datetime
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
import time

from cam.perception_utils import (
    scaled_bbox,
    scaled_odd,
    source_is_fresh,
    stamp_to_ns,
)

# ===============================
# 13pixels = 2.5cm (차선 폭)
# 82.5cm (좌우차선 중앙을 끝점으로)
# 41.25cm = 214.5pixels
# ===============================
clicked_points = []


class Lane_Detector(Node):

    def __init__(self):
        super().__init__('lane_detector')
        self.declare_parameter('image_topic', '/perception/lane/image')
        self.declare_parameter(
            'lane_detections_topic', '/lane_yolo/detections')
        self.declare_parameter('scene_detections_topic', '/scene_yolo/detections')
        self.declare_parameter('center_curve_topic', '/center_curve')
        self.declare_parameter('state_topic', '/xycar_state')
        self.declare_parameter('stamped_state_topic', '/xycar_state_stamped')
        self.declare_parameter(
            'lane_control_state_topic', '/lane_control_state_stamped')
        self.declare_parameter(
            'lane_control_state_v2_topic', '/lane_control_state_v2')
        self.declare_parameter(
            'lane_control_state_v3_topic', '/lane_control_state_v3')
        self.declare_parameter('publish_legacy_state', True)
        self.declare_parameter('publish_stamped_state', True)
        self.declare_parameter('publish_lane_control_state', True)
        self.declare_parameter('publish_lane_control_state_v2', True)
        self.declare_parameter('publish_lane_control_state_v3', True)
        self.declare_parameter('lane_override_topic', '/lane_override_cmd')
        self.declare_parameter(
            'shortcut_path_active_topic',
            '/shortcut_left_turn/path_active',
        )
        self.declare_parameter('image_qos_reliability', 'reliable')
        self.declare_parameter('enable_scene_detection_cache', True)
        self.declare_parameter('scene_detection_max_age_s', 0.60)
        self.declare_parameter('scene_detection_source_width', 640)
        self.declare_parameter('scene_detection_source_height', 480)
        self.declare_parameter('base_input_width', 640)
        self.declare_parameter('base_input_height', 480)
        self.declare_parameter('auto_scale_input_pixels', True)
        self.declare_parameter('canny_blur_kernel_px', 7)
        self.declare_parameter('object_grid_cell_size_px', 50)
        self.declare_parameter('publish_performance_stats', False)
        self.declare_parameter('performance_log_period_s', 5.0)
        self.declare_parameter('publish_pipeline_timing', False)
        self.declare_parameter('pipeline_timing_topic', '/pipeline_timing')
        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('sync_slop', 0.5)
        self.declare_parameter('curve_heading_recovery_enabled', True)
        self.declare_parameter(
            'curve_heading_disagreement_deg', 25.0)
        self.declare_parameter('curve_heading_recovery_weight', 0.65)
        self.declare_parameter('curve_heading_confirm_frames', 2)
        self.declare_parameter('curve_heading_min_points', 4)
        self.declare_parameter('curve_heading_min_y_span_ratio', 0.12)
        self.declare_parameter('curve_heading_max_fit_error_ratio', 0.08)
        self.declare_parameter(
            'curve_heading_curve_max_fit_error_ratio', 0.15)
        self.declare_parameter(
            'curve_heading_extrapolation_margin_ratio', 0.75)
        self.declare_parameter('adaptive_heading_enabled', True)
        self.declare_parameter('heading_preview_min_ratio', 0.08)
        self.declare_parameter('heading_preview_max_ratio', 0.16)
        self.declare_parameter('heading_curvature_scale', 0.50)
        self.declare_parameter('heading_curvature_deadzone', 0.0)
        self.declare_parameter('heading_filter_alpha_straight', 0.65)
        self.declare_parameter('heading_filter_alpha_curve', 0.70)
        self.declare_parameter('heading_filter_alpha_fallback', 0.35)
        self.declare_parameter('heading_curve_hold_frames', 3)
        self.declare_parameter('heading_max_step_deg', 12.0)
        # Lateral Hough association parameters are expressed in BEV pixels.
        # The gate is strict on straights and relaxes continuously with curve
        # evidence.  The source-time rate is only a final safety guard.
        self.declare_parameter('lateral_innovation_straight_px', 35.0)
        self.declare_parameter('lateral_innovation_curve_px', 120.0)
        self.declare_parameter('lateral_max_rate_straight_px_s', 320.0)
        self.declare_parameter('lateral_lane_half_width_px', 250.0)
        self.declare_parameter('lateral_outer_pair_tolerance_px', 80.0)
        self.declare_parameter('lateral_outer_pair_hold_frames', 5)
        self.declare_parameter(
            'straight_center_agreement_threshold_px', 10.0)
        self.declare_parameter('straight_center_correction_weight', 1.0)
        self.declare_parameter('straight_center_max_jump_px', 120.0)
        self.declare_parameter('straight_center_jump_confirm_frames', 2)
        self.declare_parameter('obstacle_recovery_duration_s', 3.0)
        self.declare_parameter(
            'curve_center_agreement_threshold_px', 40.0)
        self.declare_parameter('curve_center_correction_weight', 0.85)
        self.declare_parameter('curve_center_hold_frames', 3)
        sync_queue_size = self.get_parameter('sync_queue_size').get_parameter_value().integer_value
        sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value

        reliability_name = str(
            self.get_parameter('image_qos_reliability').value
        ).strip().lower()
        if reliability_name not in ('reliable', 'best_effort'):
            raise ValueError(
                'image_qos_reliability must be reliable or best_effort')
        image_reliability = (
            ReliabilityPolicy.RELIABLE
            if reliability_name == 'reliable'
            else ReliabilityPolicy.BEST_EFFORT
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # --- Subscriber 및 Publisher 설정 ---
        self.trigger_sub = self.create_subscription(
            String,
            self.get_parameter('lane_override_topic').value,
            self.trigger_callback,
            10,
        )
        self.shortcut_path_active = False
        self.shortcut_path_active_sub = self.create_subscription(
            Bool,
            self.get_parameter('shortcut_path_active_topic').value,
            self.shortcut_path_active_callback,
            10,
        )
        latest_image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=image_reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.img_sub = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter('image_topic').value,
            qos_profile=latest_image_qos,
        )
        self.detections_sub = message_filters.Subscriber(
            self,
            Detections,
            self.get_parameter('lane_detections_topic').value,
            qos_profile=reliable_qos,
        )
        self.center_curve_sub = message_filters.Subscriber(
            self,
            Curve,
            self.get_parameter('center_curve_topic').value,
            qos_profile=reliable_qos,
        )
        
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.img_sub, self.detections_sub, self.center_curve_sub],
            queue_size=sync_queue_size, slop=sync_slop
        )
        self.time_synchronizer.registerCallback(self.sync_callback)
        self.xycar_state_pub = None
        if bool(self.get_parameter('publish_legacy_state').value):
            self.xycar_state_pub = self.create_publisher(
                XycarState,
                self.get_parameter('state_topic').value,
                reliable_qos,
            )
        self.stamped_state_pub = None
        if bool(self.get_parameter('publish_stamped_state').value):
            self.stamped_state_pub = self.create_publisher(
                StampedXycarState,
                self.get_parameter('stamped_state_topic').value,
                reliable_qos,
            )
        self.lane_control_state_pub = None
        if bool(self.get_parameter('publish_lane_control_state').value):
            self.lane_control_state_pub = self.create_publisher(
                LaneControlState,
                self.get_parameter('lane_control_state_topic').value,
                reliable_qos,
            )
        self.lane_control_state_v2_pub = None
        if bool(self.get_parameter('publish_lane_control_state_v2').value):
            self.lane_control_state_v2_pub = self.create_publisher(
                LaneControlStateV2,
                self.get_parameter('lane_control_state_v2_topic').value,
                reliable_qos,
            )
        self.lane_control_state_v3_pub = None
        if bool(self.get_parameter('publish_lane_control_state_v3').value):
            self.lane_control_state_v3_pub = self.create_publisher(
                LaneControlStateV3,
                self.get_parameter('lane_control_state_v3_topic').value,
                reliable_qos,
            )
        self.pipeline_timing_pub = None
        if bool(self.get_parameter('publish_pipeline_timing').value):
            timing_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.pipeline_timing_pub = self.create_publisher(
                PipelineTiming,
                self.get_parameter('pipeline_timing_topic').value,
                timing_qos,
            )

        self.enable_scene_detection_cache = bool(
            self.get_parameter('enable_scene_detection_cache').value)
        self.scene_detection_max_age_s = max(
            0.0,
            float(self.get_parameter('scene_detection_max_age_s').value),
        )
        self.scene_source_width = int(
            self.get_parameter('scene_detection_source_width').value)
        self.scene_source_height = int(
            self.get_parameter('scene_detection_source_height').value)
        self.latest_scene_detections = None
        self.scene_frame_mismatch_warned = False
        self.scene_detection_sub = None
        if self.enable_scene_detection_cache:
            scene_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.scene_detection_sub = self.create_subscription(
                Detections,
                self.get_parameter('scene_detections_topic').value,
                self.scene_detection_callback,
                scene_qos,
            )

        self.base_input_width = max(
            1, int(self.get_parameter('base_input_width').value))
        self.base_input_height = max(
            1, int(self.get_parameter('base_input_height').value))
        self.auto_scale_input_pixels = bool(
            self.get_parameter('auto_scale_input_pixels').value)
        self.canny_blur_kernel_px = max(
            3, int(self.get_parameter('canny_blur_kernel_px').value))
        self.publish_performance_stats = bool(
            self.get_parameter('publish_performance_stats').value)
        self.performance_log_period_s = max(
            1.0,
            float(self.get_parameter('performance_log_period_s').value),
        )
        self.sync_count = 0
        self.output_count = 0
        self.max_output_gap_s = 0.0
        self.output_gap_samples_ms = deque(maxlen=512)
        self.last_source_age_ms = float('nan')
        self.last_output_monotonic = None
        self.callback_samples_ms = deque(maxlen=512)
        self.last_performance_monotonic = time.monotonic()
        self.last_performance_counts = (0, 0)
        self.performance_timer = None
        if self.publish_performance_stats:
            self.performance_timer = self.create_timer(
                self.performance_log_period_s,
                self.log_performance,
            )

        # --- 서비스 서버 생성 및 상태 플래그 초기화 ---
        self.declare_parameter('start_service_name', '/start_lane_detection')
        self.is_active = False
        self.start_service = self.create_service(
            Trigger,
            self.get_parameter('start_service_name').value,
            self.start_service_callback
        )
        self.last_sync_time = None
        self.declare_parameter('show_debug_windows', False)
        self.show_debug_windows = self.get_parameter('show_debug_windows').get_parameter_value().bool_value

        # <<< [추가] 상태 표시창 관련 초기화
        self.last_sync_time = None
        self.sync_timeout_duration = 0.5  # seconds
        self.status_timer = self.create_timer(0.05, self.update_status_display) # 20Hz 업데이트
        if not self.show_debug_windows:
            self.destroy_timer(self.status_timer)
            self.status_timer = None
        # >>>

        self.bridge = CvBridge()
        self.get_logger().info('🛣️  Lane_Detector Node Started Successfully❗')
        self.get_logger().info(
            f"Waiting for service call on {self.get_parameter('start_service_name').value} "
            "to begin processing..."
        )

        # 0. 초기값
        self.angle = 0.0
        self.angle = 0.0
        self.lane = np.array([120.0, 320.0, 520.0])
        self.M = None
        self.bev_curve_points = None
        self.roi_src_poly = None
        self.input_image_width = 0
        self.input_image_height = 0

        # Hough angle can become locked to the first half of an S-curve because
        # its normal filter only accepts lines close to the previous angle.
        # A stable local tangent from the YOLO center curve is used only to
        # recover from that disagreement; normal Hough tracking is unchanged.
        self.curve_heading_recovery_enabled = bool(
            self.get_parameter('curve_heading_recovery_enabled').value)
        self.curve_heading_disagreement = np.radians(max(
            0.0,
            float(self.get_parameter(
                'curve_heading_disagreement_deg').value),
        ))
        self.curve_heading_recovery_weight = float(np.clip(
            self.get_parameter('curve_heading_recovery_weight').value,
            0.0,
            1.0,
        ))
        self.curve_heading_confirm_frames = max(
            1,
            int(self.get_parameter('curve_heading_confirm_frames').value),
        )
        self.curve_heading_min_points = max(
            3,
            int(self.get_parameter('curve_heading_min_points').value),
        )
        self.curve_heading_min_y_span_ratio = max(
            0.0,
            float(self.get_parameter('curve_heading_min_y_span_ratio').value),
        )
        self.curve_heading_max_fit_error_ratio = max(
            0.0,
            float(self.get_parameter(
                'curve_heading_max_fit_error_ratio').value),
        )
        self.curve_heading_curve_max_fit_error_ratio = max(
            self.curve_heading_max_fit_error_ratio,
            float(self.get_parameter(
                'curve_heading_curve_max_fit_error_ratio').value),
        )
        self.curve_heading_extrapolation_margin_ratio = max(
            0.0,
            float(self.get_parameter(
                'curve_heading_extrapolation_margin_ratio').value),
        )
        self.curve_heading_history = deque(maxlen=3)
        self.curvature_history = deque(maxlen=5)
        self.curve_heading_disagreement_count = 0
        self.last_curve_heading = None
        self.estimated_curve_center_x = None
        self.curve_heading_recovering = False
        self.adaptive_heading_enabled = bool(
            self.get_parameter('adaptive_heading_enabled').value)
        self.heading_preview_min_ratio = max(
            0.01,
            float(self.get_parameter('heading_preview_min_ratio').value),
        )
        self.heading_preview_max_ratio = max(
            self.heading_preview_min_ratio,
            float(self.get_parameter('heading_preview_max_ratio').value),
        )
        self.heading_curvature_scale = max(
            1e-6,
            float(self.get_parameter('heading_curvature_scale').value),
        )
        self.heading_curvature_deadzone = max(
            0.0,
            float(self.get_parameter('heading_curvature_deadzone').value),
        )
        self.heading_filter_alpha_straight = float(np.clip(
            self.get_parameter('heading_filter_alpha_straight').value,
            0.0,
            1.0,
        ))
        self.heading_filter_alpha_curve = float(np.clip(
            self.get_parameter('heading_filter_alpha_curve').value,
            self.heading_filter_alpha_straight,
            1.0,
        ))
        self.heading_filter_alpha_fallback = float(np.clip(
            self.get_parameter('heading_filter_alpha_fallback').value,
            0.0,
            1.0,
        ))
        self.heading_curve_hold_frames = max(
            0,
            int(self.get_parameter('heading_curve_hold_frames').value),
        )
        self.heading_max_step = np.radians(max(
            0.0,
            float(self.get_parameter('heading_max_step_deg').value),
        ))
        self.control_heading = 0.0
        self.control_heading_initialized = False
        self.control_heading_source = 'hough'
        self.last_curve_curvature = None
        self.last_curve_signed_curvature = None
        self.last_curve_preview_signed_curvature = None
        self.last_curve_curvature_severity = 0.0
        self.last_heading_preview_ratio = None
        self.last_curve_fit_error_ratio = None
        self.last_curve_y_span_ratio = 0.0
        self.curve_heading_valid = False
        self.path_confidence = 0.0
        self.last_valid_curve_heading = None
        self.last_valid_curvature_severity = 0.0
        self.curve_heading_miss_count = 0
        self.current_curve_state = 'Straight'


        # 1. mask_detected_objects
        self.last_seen_map = {}
        self.grid_cell_size = max(
            1, int(self.get_parameter('object_grid_cell_size_px').value))
        self.frame_counter = 0

        # 2. apply_canny
        self.canny_low = 50
        self.canny_high = 150

        # 3. detect_lines_hough 
        self.hough_threshold = 25
        self.min_gap = 1
        self.min_length = 10

        # 4. filter_lines_by_angle
        self.angle_tolerance = np.radians(30)
        self.prev_angle = deque([0.0], maxlen=3)

        # 5. extract_lane_candidates_from_clusters
        self.cluster_threshold = 30

        # 6. predict_lane
        self.lateral_lane_half_width = max(
            1.0,
            float(self.get_parameter('lateral_lane_half_width_px').value),
        )
        self.left_offset = -self.lateral_lane_half_width
        self.right_offset = self.lateral_lane_half_width
        self.angle_correction_gain = 80
        self.min_cos_angle = 0.5
        self.angle_prev_weight = 0.7
        self.angle_new_weight  = 0.3

        # 7. refine_lane_with_candidates_and_prediction
        self.left_to_center_dist = self.lateral_lane_half_width
        self.right_to_center_dist = self.lateral_lane_half_width
        self.outer_lane_dist = self.left_to_center_dist + self.right_to_center_dist
        self.cluster_match_threshold = 70
        self.lane_update_weight = 0.7
        self.prediction_weight = 0.3
        self.max_lane_delta = 80
        self.straight_center_agreement_threshold = max(
            0.0,
            float(self.get_parameter(
                'straight_center_agreement_threshold_px').value),
        )
        self.straight_center_correction_weight = float(np.clip(
            self.get_parameter('straight_center_correction_weight').value,
            0.0,
            1.0,
        ))
        self.straight_center_max_jump = max(
            0.0,
            float(self.get_parameter('straight_center_max_jump_px').value),
        )
        self.straight_center_jump_confirm_frames = max(
            1,
            int(self.get_parameter(
                'straight_center_jump_confirm_frames').value),
        )
        self.obstacle_recovery_duration_s = max(
            0.0,
            float(self.get_parameter('obstacle_recovery_duration_s').value),
        )
        self.lateral_innovation_straight = max(
            1.0,
            float(self.get_parameter(
                'lateral_innovation_straight_px').value),
        )
        self.lateral_innovation_curve = max(
            self.lateral_innovation_straight,
            float(self.get_parameter(
                'lateral_innovation_curve_px').value),
        )
        self.lateral_max_rate_straight = max(
            1.0,
            float(self.get_parameter(
                'lateral_max_rate_straight_px_s').value),
        )
        self.lateral_outer_pair_tolerance = max(
            0.0,
            float(self.get_parameter(
                'lateral_outer_pair_tolerance_px').value),
        )
        self.lateral_outer_pair_hold_frames = max(
            0,
            int(self.get_parameter(
                'lateral_outer_pair_hold_frames').value),
        )
        self.curve_center_agreement_threshold = max(
            0.0,
            float(self.get_parameter(
                'curve_center_agreement_threshold_px').value),
        )
        self.curve_center_correction_weight = float(np.clip(
            self.get_parameter('curve_center_correction_weight').value,
            0.0,
            1.0,
        ))
        self.curve_center_hold_frames = max(
            0,
            int(self.get_parameter('curve_center_hold_frames').value),
        )
        self.last_valid_curve_center_x = None
        self.curve_center_miss_count = 0
        self.last_outer_pair_center = None
        self.outer_pair_miss_count = 0
        self.outer_pair_active = False
        self.outer_pair_held = False
        self.outer_pair_width = None
        self.last_lateral_source_ns = 0
        self.last_lateral_velocity_px_s = 0.0
        self.last_lateral_debug = {}
        self.last_lateral_debug_log_monotonic = 0.0
        self.last_priority_curve_center_x = None
        self.pending_priority_curve_center_x = None
        self.pending_priority_curve_center_count = 0
        self.priority_curve_center_x = None
        self.priority_curve_center_valid = False
        self.obstacle_recovery_until_monotonic = 0.0
        self.last_center_override_log_monotonic = 0.0

        # 8. trigger_callback
        self.override_target_lane = None

        self.get_logger().info(
            'Lane detector topics: '
            f'image={self.get_parameter("image_topic").value}, '
            f'lane_detections={self.get_parameter("lane_detections_topic").value}, '
            f'curve={self.get_parameter("center_curve_topic").value}, '
            f'scene_cache={self.enable_scene_detection_cache}, '
            f'image_qos={reliability_name}/depth1'
        )
        self.get_logger().info(
            'Curve heading recovery: '
            f'enabled={self.curve_heading_recovery_enabled}, '
            f'disagreement={np.degrees(self.curve_heading_disagreement):.1f}deg, '
            f'confirm={self.curve_heading_confirm_frames} frames, '
            f'weight={self.curve_heading_recovery_weight:.2f}'
        )
        self.get_logger().info(
            'Adaptive control heading: '
            f'enabled={self.adaptive_heading_enabled}, '
            f'preview={self.heading_preview_min_ratio:.3f}-'
            f'{self.heading_preview_max_ratio:.3f} image-height, '
            f'curvature_scale={self.heading_curvature_scale:.2f}, '
            f'curvature_deadzone={self.heading_curvature_deadzone:.3f}, '
            f'alpha={self.heading_filter_alpha_straight:.2f}-'
            f'{self.heading_filter_alpha_curve:.2f}, '
            f'hold={self.heading_curve_hold_frames} frames, '
            f'max_step={np.degrees(self.heading_max_step):.1f}deg'
        )
        self.get_logger().info(
            'Temporal lateral association: '
            f'innovation={self.lateral_innovation_straight:.1f}->'
            f'{self.lateral_innovation_curve:.1f}px, '
            f'straight_rate={self.lateral_max_rate_straight:.1f}px/s, '
            f'lane_half_width={self.lateral_lane_half_width:.1f}px, '
            f'outer_pair_tolerance={self.lateral_outer_pair_tolerance:.1f}px, '
            f'outer_pair_hold={self.lateral_outer_pair_hold_frames} frames'
        )
        self.get_logger().info(
            'Straight/recovery center priority: '
            f'threshold={self.straight_center_agreement_threshold:.1f}px, '
            f'weight={self.straight_center_correction_weight:.2f}, '
            f'max_jump={self.straight_center_max_jump:.1f}px, '
            f'confirm={self.straight_center_jump_confirm_frames} frames, '
            f'recovery={self.obstacle_recovery_duration_s:.1f}s'
        )

    def start_service_callback(self, request, response):
        if not self.is_active:
            self.is_active = True
            self.get_logger().info('✅ Service called. Starting lane detection main logic!')
            response.success = True
            response.message = 'Lane detection started.'
        else:
            self.get_logger().warn('⚠️ Main logic is already running.')
            response.success = False
            response.message = 'Already active.'
        return response

    # <<< [추가] 새로운 상태 표시창을 업데이트하는 콜백 함수
    def update_status_display(self):
        # 상태 표시창 크기 및 폰트 설정
        display_height = 100
        display_width = 450
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_color = (255, 255, 255)
        thickness = 1
        
        is_timeout = True
        
        # 마지막 싱크 시간부터 현재까지의 경과 시간 확인
        if self.last_sync_time is not None:
            elapsed_time = (self.get_clock().now() - self.last_sync_time).nanoseconds / 1e9
            if elapsed_time <= self.sync_timeout_duration:
                is_timeout = False

        # 배경색 설정
        if is_timeout:
            status_img = np.full((display_height, display_width, 3), (0, 0, 200), dtype=np.uint8)
            # 타임아웃 발생 시 배경을 빨간색으로
            status_img = np.full((display_height, display_width, 3), (0, 0, 200), dtype=np.uint8)
        else:
            status_img = np.zeros((display_height, display_width, 3), dtype=np.uint8)
            # 정상 상태일 때 배경을 검은색으로
            status_img = np.zeros((display_height, display_width, 3), dtype=np.uint8)
            
        # 현재 시간 표시
        now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        cv2.putText(status_img, f'Current Time: {now_str}', (10, 30), font, font_scale, font_color, thickness, cv2.LINE_AA)
        
        # 마지막 싱크 시간 표시
        if self.last_sync_time is not None:
            last_sync_sec = self.last_sync_time.nanoseconds / 1e9
            last_sync_str = datetime.fromtimestamp(last_sync_sec).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        else:
            last_sync_str = "N/A"
            
        cv2.putText(status_img, f'Last Sync Time: {last_sync_str}', (10, 70), font, font_scale, font_color, thickness, cv2.LINE_AA)

        cv2.imshow('Sync Status', status_img)
        cv2.waitKey(1)
        # 이 창이 독립적으로 업데이트되도록 waitKey를 호출해야 함
        cv2.waitKey(1)
    # >>>

    def get_birds_eye_view(self, image):
        height, width = image.shape[:2]
        self.input_image_width = width
        self.input_image_height = height
        output_width = 640
        output_height = 120
        src = np.float32([
            [width * 0.16, height * 0.64],
            [width * 0.84, height * 0.64],
            [width * 1.00, height * 0.72],
            [width * 0.00, height * 0.72]
        ])
        dst = np.float32([
            [output_width * 0.1, 0],
            [output_width * 0.9, 0],
            [output_width * 0.9, output_height],
            [output_width * 0.1, output_height]
        ])
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.roi_src_poly = src 
        bev = cv2.warpPerspective(image, self.M, (output_width, output_height))
        self.bev_img_w = output_width
        self.bev_img_h = output_height
        self.bev_img_mid = output_height // 2
        return bev, src

    def transform_curve_to_bev(self, curve_msg):
        if self.M is None or curve_msg is None or not curve_msg.points or self.roi_src_poly is None:
            self.bev_curve_points = None
            return
        all_points = curve_msg.points
        points_inside_roi = [p for p in all_points if cv2.pointPolygonTest(self.roi_src_poly, (p.x, p.y), False) >= 0]
        if not points_inside_roi:
            self.bev_curve_points = None
            return
        original_points = np.array([[[p.x, p.y] for p in points_inside_roi]], dtype=np.float32)
        bev_points = cv2.perspectiveTransform(original_points, self.M)
        if bev_points is not None:
            self.bev_curve_points = bev_points[0]
        else:
            self.bev_curve_points = None

    def estimate_curve_heading(self, curve_msg):
        """
        Estimate the center-curve heading with curvature-adaptive preview.

        Only 0-1 published curve points usually fall directly inside the very
        shallow source ROI.  Fit all clean center-curve points in image space,
        choose a longer preview on straight sections and a shorter preview on
        curves, and transform that segment to BEV.
        """
        self.estimated_curve_center_x = None
        self.last_curve_curvature = None
        self.last_curve_signed_curvature = None
        self.last_curve_preview_signed_curvature = None
        self.last_curve_curvature_severity = 0.0
        self.last_heading_preview_ratio = None
        self.last_curve_fit_error_ratio = None
        self.last_curve_y_span_ratio = 0.0
        self.curve_heading_valid = False
        if (
            not (
                self.curve_heading_recovery_enabled
                or self.adaptive_heading_enabled
            )
            or self.M is None
            or self.roi_src_poly is None
            or curve_msg is None
            or len(curve_msg.points) < self.curve_heading_min_points
            or self.input_image_width <= 0
            or self.input_image_height <= 0
        ):
            return None

        points = np.asarray(
            [(point.x, point.y) for point in curve_msg.points],
            dtype=np.float64,
        )
        if points.ndim != 2 or points.shape[1] != 2:
            return None
        finite = np.all(np.isfinite(points), axis=1)
        points = points[finite]
        if len(points) < self.curve_heading_min_points:
            return None

        width = float(self.input_image_width)
        height = float(self.input_image_height)
        normalized_x = points[:, 0] / width
        normalized_y = points[:, 1] / height
        y_span_ratio = float(np.ptp(normalized_y))
        self.last_curve_y_span_ratio = y_span_ratio
        if y_span_ratio < self.curve_heading_min_y_span_ratio:
            return None

        # A quadratic captures the local tangent of the S while avoiding the
        # misleading end-to-end chord through both halves of the curve.
        coefficients = np.polyfit(normalized_y, normalized_x, 2)
        fitted_x = np.polyval(coefficients, normalized_y)
        fit_error = float(np.sqrt(np.mean(
            np.square(normalized_x - fitted_x))))
        self.last_curve_fit_error_ratio = fit_error
        fit_error_limit = self.curve_heading_max_fit_error_ratio
        if str(curve_msg.state).strip().lower() == 'curve':
            # Tight S turns legitimately exceed the straight-section residual
            # limit. Keep the relaxed threshold curve-only so straight noise
            # still receives the original strict rejection.
            fit_error_limit = self.curve_heading_curve_max_fit_error_ratio
        if not np.isfinite(fit_error) or fit_error > fit_error_limit:
            return None

        roi_top_ratio = float(np.min(self.roi_src_poly[:, 1])) / height
        roi_bottom_ratio = float(np.max(self.roi_src_poly[:, 1])) / height
        roi_mid_ratio = 0.5 * (roi_bottom_ratio + roi_top_ratio)

        local_slope = (
            2.0 * coefficients[0] * roi_mid_ratio + coefficients[1])
        second_derivative = 2.0 * coefficients[0]
        signed_curvature = second_derivative / (
            1.0 + local_slope * local_slope) ** 1.5
        if not np.isfinite(signed_curvature):
            return None
        curvature = abs(signed_curvature)
        self.curvature_history.append(float(curvature))
        smoothed_curvature = float(np.median(np.asarray(
            self.curvature_history,
            dtype=np.float64,
        )))
        effective_curvature = max(
            0.0,
            smoothed_curvature - self.heading_curvature_deadzone,
        )
        curvature_severity = (
            effective_curvature
            / (effective_curvature + self.heading_curvature_scale)
        )
        preview_ratio = (
            self.heading_preview_min_ratio
            + (self.heading_preview_max_ratio
               - self.heading_preview_min_ratio)
            * (1.0 - curvature_severity)
        )
        preview_ratio = float(np.clip(
            preview_ratio,
            self.heading_preview_min_ratio,
            self.heading_preview_max_ratio,
        ))

        sample_ratio_y = np.asarray([
            roi_bottom_ratio,
            roi_bottom_ratio - preview_ratio,
        ], dtype=np.float64)
        if sample_ratio_y[1] <= 0.0:
            return None
        sample_ratio_x = np.polyval(coefficients, sample_ratio_y)
        sample_y = sample_ratio_y * height
        sample_x = sample_ratio_x * width
        if not np.all(np.isfinite(sample_x)):
            return None

        # Permit a little more off-screen recovery on an S reversal. The
        # heading and downstream servo command remain bounded independently.
        extrapolation_margin = (
            self.curve_heading_extrapolation_margin_ratio * width)
        if (
            np.any(sample_x < -extrapolation_margin)
            or np.any(sample_x > width + extrapolation_margin)
        ):
            return None

        roi_mid_y = roi_mid_ratio * height
        roi_mid_x = float(np.polyval(coefficients, roi_mid_ratio) * width)
        samples = np.asarray([[
            [sample_x[0], sample_y[0]],
            [sample_x[1], sample_y[1]],
            [roi_mid_x, roi_mid_y],
        ]], dtype=np.float32)
        bev_samples = cv2.perspectiveTransform(samples, self.M)
        if bev_samples is None or not np.all(np.isfinite(bev_samples)):
            return None

        bottom, top = bev_samples[0][:2]
        forward = float(bottom[1] - top[1])
        if forward <= 1.0:
            return None
        heading = float(np.arctan2(top[0] - bottom[0], forward))
        if abs(heading) > np.radians(75.0):
            return None
        self.estimated_curve_center_x = float(bev_samples[0][2][0])
        self.last_curve_curvature = smoothed_curvature
        # Keep the existing public curvature unsigned. This raw signed hint is
        # used only by the separately published Stanley transition extension.
        self.last_curve_signed_curvature = float(signed_curvature)
        # The proven quadratic heading path remains untouched. A separate
        # cubic diagnostic fit estimates far-path curvature for longitudinal
        # preview only; invalid fits fall back to the current signed hint.
        preview_signed_curvature = float(signed_curvature)
        if len(points) >= 6:
            cubic = np.polyfit(normalized_y, normalized_x, 3)
            cubic_x = np.polyval(cubic, normalized_y)
            cubic_error = float(np.sqrt(np.mean(
                np.square(normalized_x - cubic_x))))
            if (
                np.isfinite(cubic_error)
                and cubic_error <= fit_error_limit
            ):
                far_ratio = max(
                    float(np.min(normalized_y)),
                    roi_bottom_ratio - 2.0 * preview_ratio,
                )
                first_derivative = float(np.polyval(
                    np.polyder(cubic, 1), far_ratio))
                second_derivative_far = float(np.polyval(
                    np.polyder(cubic, 2), far_ratio))
                candidate = second_derivative_far / (
                    1.0 + first_derivative * first_derivative
                ) ** 1.5
                if np.isfinite(candidate):
                    preview_signed_curvature = float(candidate)
        self.last_curve_preview_signed_curvature = (
            preview_signed_curvature)
        self.last_curve_curvature_severity = curvature_severity
        self.last_heading_preview_ratio = preview_ratio
        self.curve_heading_valid = True
        return heading

    def update_path_confidence(self):
        """Estimate path quality for diagnostics without changing control."""
        source = self.control_heading_source
        if source == 'curve':
            fit_limit = self.curve_heading_max_fit_error_ratio
            if str(self.current_curve_state).strip().lower() == 'curve':
                fit_limit = self.curve_heading_curve_max_fit_error_ratio
            fit_error = self.last_curve_fit_error_ratio
            fit_score = 0.0
            if fit_error is not None and np.isfinite(fit_error):
                fit_score = 1.0 - float(np.clip(
                    fit_error / max(fit_limit, 1e-6), 0.0, 1.0))
            span_score = float(np.clip(
                self.last_curve_y_span_ratio
                / max(2.0 * self.curve_heading_min_y_span_ratio, 1e-6),
                0.0,
                1.0,
            ))
            self.path_confidence = 0.60 + 0.20 * (
                fit_score + span_score)
        elif source == 'curve_hold':
            hold_fraction = (
                self.curve_heading_miss_count
                / max(self.heading_curve_hold_frames, 1)
            )
            self.path_confidence = 0.60 - 0.20 * float(np.clip(
                hold_fraction, 0.0, 1.0))
        else:
            self.path_confidence = 0.35
        self.path_confidence = float(np.clip(
            self.path_confidence, 0.0, 1.0))
        return self.path_confidence

    def update_control_heading(self, curve_heading):
        """Update the steering heading while keeping CTE at the fixed slice."""
        curve_is_valid = (
            self.adaptive_heading_enabled
            and curve_heading is not None
            and np.isfinite(curve_heading)
        )
        if curve_is_valid:
            target_heading = float(curve_heading)
            severity = float(np.clip(
                self.last_curve_curvature_severity,
                0.0,
                1.0,
            ))
            self.last_valid_curve_heading = target_heading
            self.last_valid_curvature_severity = severity
            self.curve_heading_miss_count = 0
            alpha = (
                self.heading_filter_alpha_straight
                + (self.heading_filter_alpha_curve
                   - self.heading_filter_alpha_straight)
                * severity
            )
            self.control_heading_source = 'curve'
        elif (
            self.adaptive_heading_enabled
            and self.last_valid_curve_heading is not None
            and self.curve_heading_miss_count
            < self.heading_curve_hold_frames
        ):
            self.curve_heading_miss_count += 1
            target_heading = self.last_valid_curve_heading
            severity = self.last_valid_curvature_severity
            alpha = (
                self.heading_filter_alpha_straight
                + (self.heading_filter_alpha_curve
                   - self.heading_filter_alpha_straight)
                * severity
            )
            self.control_heading_source = 'curve_hold'
        else:
            self.curve_heading_miss_count += 1
            target_heading = float(self.angle)
            alpha = self.heading_filter_alpha_fallback
            self.control_heading_source = 'hough'

        self.update_path_confidence()

        if not self.control_heading_initialized:
            self.control_heading = target_heading
            self.control_heading_initialized = True
            return self.control_heading

        delta = np.arctan2(
            np.sin(target_heading - self.control_heading),
            np.cos(target_heading - self.control_heading),
        )
        heading_step = float(alpha) * float(delta)
        max_heading_step = self.heading_max_step
        if max_heading_step > 0.0:
            heading_step = float(np.clip(
                heading_step,
                -max_heading_step,
                max_heading_step,
            ))
        self.control_heading += heading_step
        return self.control_heading
            
    def obstacle_recovery_active(self, now_monotonic=None):
        if now_monotonic is None:
            now_monotonic = time.monotonic()
        return now_monotonic < self.obstacle_recovery_until_monotonic

    def center_curve_priority_active(self):
        state = str(self.current_curve_state).strip().lower()
        return (
            self.override_target_lane is None
            and not self.shortcut_path_active
            and (state == 'straight' or self.obstacle_recovery_active())
        )

    def reset_priority_curve_center_validation(self):
        self.last_priority_curve_center_x = None
        self.pending_priority_curve_center_x = None
        self.pending_priority_curve_center_count = 0
        self.priority_curve_center_x = None
        self.priority_curve_center_valid = False

    def validate_priority_curve_center(self):
        """Accept fitted YOLO centers and confirm implausible one-frame jumps."""
        center = self.estimated_curve_center_x
        valid = (
            center is not None
            and np.isfinite(center)
            and 0.0 <= center < self.bev_img_w
        )
        if not valid:
            self.reset_priority_curve_center_validation()
            return None

        center = float(center)
        previous = self.last_priority_curve_center_x
        if (
            previous is None
            or abs(center - previous) <= self.straight_center_max_jump
        ):
            self.last_priority_curve_center_x = center
            self.pending_priority_curve_center_x = None
            self.pending_priority_curve_center_count = 0
            return center

        pending = self.pending_priority_curve_center_x
        if (
            pending is not None
            and abs(center - pending)
            <= self.straight_center_agreement_threshold
        ):
            self.pending_priority_curve_center_count += 1
        else:
            self.pending_priority_curve_center_x = center
            self.pending_priority_curve_center_count = 1

        if (
            self.pending_priority_curve_center_count
            < self.straight_center_jump_confirm_frames
        ):
            return None

        self.last_priority_curve_center_x = center
        self.pending_priority_curve_center_x = None
        self.pending_priority_curve_center_count = 0
        return center

    def force_correct_lane_with_curve(self):
        """
        Prefer a validated fitted YOLO center on straights and recovery.

        Return True when a valid priority center was available, including when
        it already agreed with the provisional target and no correction moved.
        """
        priority_active = self.center_curve_priority_active()
        enhanced_curve_correction = (
            str(self.current_curve_state).strip().lower() == 'curve'
            and self.override_target_lane is None
            and not self.shortcut_path_active
            and not priority_active
        )
        if not enhanced_curve_correction:
            self.last_valid_curve_center_x = None
            self.curve_center_miss_count = 0

        if not priority_active:
            self.reset_priority_curve_center_validation()

        if priority_active:
            curve_center_x = self.validate_priority_curve_center()
            self.priority_curve_center_x = curve_center_x
            self.priority_curve_center_valid = curve_center_x is not None
            if curve_center_x is None:
                return False
            agreement_threshold = self.straight_center_agreement_threshold
            correction_weight = self.straight_center_correction_weight
        else:
            self.priority_curve_center_x = None
            self.priority_curve_center_valid = False

            if (
                self.curve_heading_recovering
                and self.estimated_curve_center_x is not None
            ):
                curve_center_x = self.estimated_curve_center_x
            elif (
                self.bev_curve_points is not None
                and len(self.bev_curve_points)
            ):
                curve_center_x = np.mean(self.bev_curve_points[:, 0])
            else:
                curve_center_x = None

            curve_center_valid = (
                curve_center_x is not None
                and np.isfinite(curve_center_x)
                and 0 <= curve_center_x < self.bev_img_w
            )
            if enhanced_curve_correction:
                if curve_center_valid:
                    curve_center_x = float(curve_center_x)
                    self.last_valid_curve_center_x = curve_center_x
                    self.curve_center_miss_count = 0
                elif (
                    self.last_valid_curve_center_x is not None
                    and np.isfinite(self.last_valid_curve_center_x)
                    and 0 <= self.last_valid_curve_center_x < self.bev_img_w
                    and self.curve_center_miss_count
                    < self.curve_center_hold_frames
                ):
                    self.curve_center_miss_count += 1
                    curve_center_x = self.last_valid_curve_center_x
                else:
                    return False
                agreement_threshold = self.curve_center_agreement_threshold
                correction_weight = self.curve_center_correction_weight
            else:
                if not curve_center_valid:
                    return False
                agreement_threshold = 80.0
                correction_weight = 0.60
        
        # 1단계 추정치(self.lane[1])와 YOLO 중앙선(curve_center_x)의 거리 차이 계산
        delta = abs(self.lane[1] - curve_center_x)

        # 거리 차이가 설정된 임계값을 초과할 경우에만 보정 수행
        if delta > agreement_threshold:
            now = time.monotonic()
            if now - self.last_center_override_log_monotonic >= 1.0:
                self.get_logger().warn(
                    f'Large deviation detected ({delta:.1f}px). '
                    'Overriding center lane.')
                self.last_center_override_log_monotonic = now
            
            # 부드러운 업데이트를 위해 현재 중앙차선 위치와 가중 평균
            new_center = (correction_weight * curve_center_x) + \
                         ((1 - correction_weight) * self.lane[1])
            
            self.lane[1] = new_center # 중앙 차선 위치 강제 업데이트
            
            # 새로운 중앙 차선을 기준으로 좌/우 차선 위치 재계산
            cos_angle = max(np.cos(self.angle), self.min_cos_angle)
            self.lane[0] = self.lane[1] - self.left_to_center_dist / cos_angle
            self.lane[2] = self.lane[1] + self.right_to_center_dist / cos_angle

        return priority_active

    def scene_detection_callback(self, msg):
        self.latest_scene_detections = msg

    def fresh_scene_targets(self, image_msg, frame_width, frame_height):
        """Return fresh scene boxes scaled into the lane image coordinates."""
        msg = self.latest_scene_detections
        if not self.enable_scene_detection_cache or msg is None:
            self.last_seen_map.clear()
            return []

        image_stamp_ns = stamp_to_ns(image_msg.header.stamp)
        scene_stamp_ns = stamp_to_ns(msg.header.stamp)
        if (
            image_msg.header.frame_id
            and msg.header.frame_id
            and image_msg.header.frame_id != msg.header.frame_id
        ):
            if not self.scene_frame_mismatch_warned:
                self.get_logger().warn(
                    'Ignoring scene detections with mismatched frame_id: '
                    f'lane={image_msg.header.frame_id!r}, '
                    f'scene={msg.header.frame_id!r}'
                )
                self.scene_frame_mismatch_warned = True
            self.last_seen_map.clear()
            return []
        fresh = source_is_fresh(
            image_stamp_ns,
            scene_stamp_ns,
            self.scene_detection_max_age_s,
        )
        if not fresh:
            self.last_seen_map.clear()
            return []

        targets = []
        for source in msg.detections:
            if source.class_name not in (
                    'checkerboard', 'dynamic', 'obstacle_vehicle'):
                continue
            x1, y1, x2, y2 = scaled_bbox(
                (source.xmin, source.ymin, source.xmax, source.ymax),
                self.scene_source_width,
                self.scene_source_height,
                frame_width,
                frame_height,
            )
            detection = Detection()
            detection.class_name = source.class_name
            detection.confidence = source.confidence
            detection.xmin = x1
            detection.ymin = y1
            detection.xmax = x2
            detection.ymax = y2
            targets.append(detection)
        return targets

    def apply_canny(self, img, show=False):
        if len(img.shape) == 3: img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kernel_size = scaled_odd(self.canny_blur_kernel_px, 1.0, minimum=3)
        if self.auto_scale_input_pixels:
            scale = min(
                img.shape[1] / self.base_input_width,
                img.shape[0] / self.base_input_height,
            )
            kernel_size = scaled_odd(
                self.canny_blur_kernel_px, scale, minimum=3)
        img = cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
        img = cv2.Canny(img, self.canny_low, self.canny_high)
        if show: cv2.imshow('canny', img)
        return img

    def mask_detected_objects(
        self,
        canny_img,
        detections_msg,
        scene_targets,
        original_frame,
        show=False,
    ):
        self.frame_counter += 1
        modified_canny = canny_img.copy()
        if show:
            viz_img = original_frame.copy()
            final_mask_for_viz = np.zeros_like(modified_canny)
        active_target_bboxes, current_grid_cells = set(), set()
        current_target_detections = [
            detection for detection in detections_msg.detections
            if detection.class_name in (
                'checkerboard', 'dynamic', 'obstacle_vehicle')
        ]
        current_target_detections.extend(scene_targets)
        lane_detections = [d for d in detections_msg.detections if d.class_name == "center_line"]
        grid_cell_size = self.grid_cell_size
        if self.auto_scale_input_pixels:
            grid_cell_size = max(
                1,
                int(self.grid_cell_size * original_frame.shape[1]
                    / self.base_input_width),
            )
        for det in current_target_detections:
            bbox = (det.xmin, det.ymin, det.xmax, det.ymax)
            active_target_bboxes.add(bbox)
            center_x, center_y = (det.xmin + det.xmax) // 2, (det.ymin + det.ymax) // 2
            grid_cell = (center_x // grid_cell_size, center_y // grid_cell_size)
            current_grid_cells.add(grid_cell)
            self.last_seen_map[grid_cell] = {'last_frame': self.frame_counter, 'bbox': bbox}
        for cell, data in list(self.last_seen_map.items()):
            if self.frame_counter - 1 == data['last_frame'] and cell not in current_grid_cells:
                active_target_bboxes.add(data['bbox'])
            if data['last_frame'] < self.frame_counter - 1:
                del self.last_seen_map[cell]
        for t_bbox in list(active_target_bboxes):
            t_xmin, t_ymin, t_xmax, t_ymax = t_bbox
            overlapping_lanes_bboxes = []
            for lane_det in lane_detections:
                l_xmin, l_ymin, l_xmax, l_ymax = lane_det.xmin, lane_det.ymin, lane_det.xmax, lane_det.ymax
                if not (t_xmax < l_xmin or t_xmin > l_xmax or t_ymax < l_ymin or t_ymin > l_ymax):
                    overlapping_lanes_bboxes.append((l_xmin, l_ymin, l_xmax, l_ymax))
            current_mask = np.zeros_like(modified_canny)
            if not overlapping_lanes_bboxes: cv2.rectangle(current_mask, (t_xmin, t_ymin), (t_xmax, t_ymax), 255, -1)
            else:
                cv2.rectangle(current_mask, (t_xmin, t_ymin), (t_xmax, t_ymax), 255, -1)
                for l_bbox in overlapping_lanes_bboxes: cv2.rectangle(current_mask, (l_bbox[0], l_bbox[1]), (l_bbox[2], l_bbox[3]), 0, -1)
            modified_canny[current_mask == 255] = 0
            if show: final_mask_for_viz = cv2.bitwise_or(final_mask_for_viz, current_mask)
        if show:
            viz_img[final_mask_for_viz == 255] = (0, 0, 0)
            contours, _ = cv2.findContours(final_mask_for_viz, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(viz_img, contours, -1, (0, 0, 255), 2)
            cv2.imshow("Object Masking Visualization", viz_img)
            cv2.imshow("Masked Canny Image", modified_canny)
        return modified_canny

    def detect_lines_hough(self, img, show=False):
        lines = cv2.HoughLinesP(img, 1, np.pi/180, self.hough_threshold, self.min_gap, self.min_length)
        if show:
            hough_img = np.zeros((img.shape[0], img.shape[1], 3))
            if lines is not None:
                for x1, y1, x2, y2 in lines[:, 0]: cv2.line(hough_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.imshow('hough', hough_img)
        return lines

    @staticmethod
    def angle_distance(first, second):
        return abs(np.arctan2(
            np.sin(first - second),
            np.cos(first - second),
        ))

    def filter_lines_by_angle(
        self,
        lines,
        show=True,
        curve_heading=None,
    ):
        candidates = []
        if show: filter_img = np.zeros((self.bev_img_h, self.bev_img_w, 3))
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if y1 == y2: continue
                flag = 1 if y1-y2 > 0 else -1
                theta = np.arctan2(flag * (x2-x1), flag * (y1-y2))
                position = float(
                    (x2-x1)*(self.bev_img_mid-y1)/(y2-y1) + x1)
                candidates.append((theta, position, (x1, y1, x2, y2)))

        selected = [
            candidate for candidate in candidates
            if self.angle_distance(candidate[0], self.angle)
            < self.angle_tolerance
        ]
        previous_angle = self.angle
        self.prev_angle.append(previous_angle)

        recovery_started = False
        smoothed_curve_heading = None
        if curve_heading is not None and np.isfinite(curve_heading):
            self.curve_heading_history.append(float(curve_heading))
            smoothed_curve_heading = float(np.median(
                np.asarray(self.curve_heading_history, dtype=np.float64)))
            self.last_curve_heading = smoothed_curve_heading
            disagreement = self.angle_distance(
                smoothed_curve_heading,
                previous_angle,
            )
            if disagreement > self.curve_heading_disagreement:
                old_count = self.curve_heading_disagreement_count
                self.curve_heading_disagreement_count = min(
                    old_count + 1,
                    self.curve_heading_confirm_frames,
                )
                recovery_started = (
                    old_count < self.curve_heading_confirm_frames
                    and self.curve_heading_disagreement_count
                    >= self.curve_heading_confirm_frames
                )
            else:
                self.curve_heading_disagreement_count = 0
        else:
            self.curve_heading_history.clear()
            self.curve_heading_disagreement_count = 0
            self.last_curve_heading = None

        recovering = (
            smoothed_curve_heading is not None
            and self.curve_heading_disagreement_count
            >= self.curve_heading_confirm_frames
        )
        self.curve_heading_recovering = recovering
        if recovering:
            curve_selected = [
                candidate for candidate in candidates
                if self.angle_distance(
                    candidate[0], smoothed_curve_heading)
                < self.angle_tolerance
            ]
            # Prefer white-line evidence agreeing with the center curve.  If
            # none exists, advance heading from the curve alone and suppress
            # stale opposite-direction lane positions for this frame.
            if curve_selected:
                recovery_target = float(np.mean([
                    candidate[0] for candidate in curve_selected
                ]))
                selected = curve_selected
            else:
                recovery_target = smoothed_curve_heading
                selected = []
            weight = self.curve_heading_recovery_weight
            self.angle = (
                (1.0 - weight) * previous_angle
                + weight * recovery_target
            )
            if recovery_started:
                self.get_logger().warn(
                    'Recovering stale Hough heading from center curve: '
                    f'hough={np.degrees(previous_angle):.1f}deg, '
                    f'curve={np.degrees(smoothed_curve_heading):.1f}deg'
                )
        elif selected:
            new_angle = float(np.mean([
                candidate[0] for candidate in selected
            ]))
            self.angle = (
                self.angle_prev_weight * previous_angle
                + self.angle_new_weight * new_angle
            )

        if show:
            for _, _, (x1, y1, x2, y2) in selected:
                cv2.line(
                    filter_img,
                    (x1, y1),
                    (x2, y2),
                    (0, 0, 255),
                    2,
                )
            cv2.imshow('filtered lines by angle', filter_img)
        return [candidate[1] for candidate in selected]

    def extract_lane_candidates_from_clusters(self, positions, show=False, base_img=None):
        clusters = []
        for position in positions:
            if 0 <= position < self.bev_img_w:
                for cluster in clusters:
                    if abs(np.mean(cluster) - position) < self.cluster_threshold:
                        cluster.append(position)
                        break
                else: clusters.append([position])
        lane_candidates = [np.mean(cluster) for cluster in clusters]
        if show and base_img is not None:
            cluster_img = cv2.cvtColor(base_img.copy(), cv2.COLOR_GRAY2BGR)
            colors = [tuple(map(int, np.random.randint(0, 255, size=3))) for _ in range(len(clusters))]
            for idx, cluster in enumerate(clusters):
                for x in cluster: cv2.circle(cluster_img, (int(x), self.bev_img_mid), 4, colors[idx], -1)
            cv2.imshow('Clustered Lane Positions', cluster_img)
        return lane_candidates

    def predict_lane(self, show=False, base_img=None):
        denom = max(np.cos(self.angle), self.min_cos_angle)
        predicted_lane = self.lane[1] + np.array([self.left_offset / denom, 0, self.right_offset / denom])
        predicted_lane += (self.angle - np.mean(self.prev_angle)) * self.angle_correction_gain
        if show and base_img is not None:
            img = cv2.cvtColor(base_img.copy(), cv2.COLOR_GRAY2BGR) if len(base_img.shape) == 2 else base_img.copy()
            y = self.bev_img_mid
            cv2.circle(img, (int(predicted_lane[0]), y), 4, (255, 0, 255), 2)
            cv2.circle(img, (int(predicted_lane[1]), y), 4, (128, 128, 128), 2)
            cv2.circle(img, (int(predicted_lane[2]), y), 4, (255, 255, 0), 2)
            cv2.putText(img, 'Pred_L', (int(predicted_lane[0])-20, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,255), 1)
            cv2.putText(img, 'Pred_C', (int(predicted_lane[1])-20, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128,128,128), 1)
            cv2.putText(img, 'Pred_R', (int(predicted_lane[2])-20, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,0), 1)
            cv2.imshow('Predicted Lane', img)
        return predicted_lane

    def lateral_source_dt(self, source_ns):
        """Return bounded source-frame dt and whether the stamp was usable."""
        fallback_dt = 1.0 / 12.0
        valid = False
        dt = fallback_dt
        if (
            source_ns > 0
            and self.last_lateral_source_ns > 0
            and source_ns > self.last_lateral_source_ns
        ):
            raw_dt = (source_ns - self.last_lateral_source_ns) * 1e-9
            if 0.0 < raw_dt <= 0.5:
                # A long gap must not make the safety guard effectively vanish.
                dt = float(np.clip(raw_dt, 1.0 / 120.0, 0.20))
                valid = True
        return dt, valid

    def lateral_curve_relief(self):
        """Relax lateral gates only when current geometry supports fast motion."""
        if self.obstacle_recovery_active():
            return 0.0
        severity = float(np.clip(
            self.last_curve_curvature_severity, 0.0, 1.0))
        curve_state = str(self.current_curve_state).strip().lower()
        state_relief = 1.0 if curve_state == 'curve' else 0.0
        previous_angle = float(np.mean(self.prev_angle))
        heading_relief = float(np.clip(
            self.angle_distance(self.angle, previous_angle)
            / np.radians(15.0),
            0.0,
            1.0,
        ))
        return max(
            severity,
            state_relief,
            heading_relief,
        )

    def curve_lateral_reference(self):
        """Return center-curve x for diagnostics, never direct averaging."""
        if self.center_curve_priority_active():
            if self.priority_curve_center_valid:
                return float(self.priority_curve_center_x)
            return None
        if (
            self.estimated_curve_center_x is not None
            and np.isfinite(self.estimated_curve_center_x)
        ):
            return float(self.estimated_curve_center_x)
        if self.bev_curve_points is not None and len(self.bev_curve_points):
            center = float(np.mean(self.bev_curve_points[:, 0]))
            if np.isfinite(center):
                return center
        return None

    def reset_outer_pair_state(self):
        """Discard straight-only outer-lane evidence at mode transitions."""
        self.last_outer_pair_center = None
        self.outer_pair_miss_count = 0
        self.outer_pair_active = False
        self.outer_pair_held = False
        self.outer_pair_width = None

    def stabilize_straight_lateral_with_outer_pair(
        self,
        lane_candidates,
        temporal_center,
    ):
        """Use both outer Hough lines to remove center-role ambiguity.

        The BEV maps the two road boundaries roughly 500 px apart.  Inferring
        the center from either boundary independently can therefore select a
        synthetic center one lane-role away from the visible dashed center.
        When both boundaries exist, their midpoint is geometry-constrained and
        needs no temporal smoothing.  A short hold bridges Hough dropouts only;
        Curve/S frames and explicit lane overrides bypass it immediately.
        """
        self.outer_pair_active = False
        self.outer_pair_held = False
        self.outer_pair_width = None
        if (
            str(self.current_curve_state).strip().lower() != 'straight'
            and not self.obstacle_recovery_active()
        ):
            self.reset_outer_pair_state()
            return None

        visible = sorted(
            float(item) for item in lane_candidates
            if np.isfinite(item) and 0.0 <= item < self.bev_img_w
        )
        expected_width = 2.0 * self.lateral_lane_half_width
        minimum_width = max(
            0.0, expected_width - self.lateral_outer_pair_tolerance)
        maximum_width = expected_width + self.lateral_outer_pair_tolerance
        pairs = [
            (left, right)
            for index, left in enumerate(visible)
            for right in visible[index + 1:]
            if minimum_width <= right - left <= maximum_width
        ]
        robust_center = None
        if pairs:
            reference = self.curve_lateral_reference()
            if reference is None or not np.isfinite(reference):
                reference = float(temporal_center)
            left, right = min(
                pairs,
                key=lambda pair: abs(
                    0.5 * (pair[0] + pair[1]) - reference),
            )
            robust_center = 0.5 * (left + right)
            self.last_outer_pair_center = robust_center
            self.outer_pair_miss_count = 0
            self.outer_pair_active = True
            self.outer_pair_width = right - left
        elif (
            self.last_outer_pair_center is not None
            and self.outer_pair_miss_count
            < self.lateral_outer_pair_hold_frames
        ):
            self.outer_pair_miss_count += 1
            robust_center = self.last_outer_pair_center
            self.outer_pair_active = True
            self.outer_pair_held = True
        else:
            self.outer_pair_miss_count += 1

        if robust_center is not None:
            self.lane += float(robust_center) - self.lane[1]
        return robust_center

    def build_lane_hypotheses(self, lane_candidates):
        """Keep the existing lane-width geometry while forming triplets."""
        possibles = []
        cos_angle = max(np.cos(self.angle), self.min_cos_angle)
        for lane_candidate in lane_candidates:
            index = int(np.argmin(abs(self.lane - lane_candidate)))
            if index == 0:
                estimate = [
                    lane_candidate,
                    lane_candidate + self.left_to_center_dist / cos_angle,
                    lane_candidate + self.outer_lane_dist / cos_angle,
                ]
                center_candidates = [
                    item for item in lane_candidates
                    if abs(item - estimate[1]) < self.cluster_match_threshold
                ] or [estimate[1]]
                right_candidates = [
                    item for item in lane_candidates
                    if abs(item - estimate[2]) < self.cluster_match_threshold
                ] or [estimate[2]]
                for center in center_candidates:
                    possibles.extend([
                        [lane_candidate, center, right]
                        for right in right_candidates
                    ])
            elif index == 1:
                estimate = [
                    lane_candidate - self.left_to_center_dist / cos_angle,
                    lane_candidate,
                    lane_candidate + self.right_to_center_dist / cos_angle,
                ]
                left_candidates = [
                    item for item in lane_candidates
                    if abs(item - estimate[0]) < self.cluster_match_threshold
                ] or [estimate[0]]
                right_candidates = [
                    item for item in lane_candidates
                    if abs(item - estimate[2]) < self.cluster_match_threshold
                ] or [estimate[2]]
                for left in left_candidates:
                    possibles.extend([
                        [left, lane_candidate, right]
                        for right in right_candidates
                    ])
            else:
                estimate = [
                    lane_candidate - self.outer_lane_dist / cos_angle,
                    lane_candidate - self.right_to_center_dist / cos_angle,
                    lane_candidate,
                ]
                left_candidates = [
                    item for item in lane_candidates
                    if abs(item - estimate[0]) < self.cluster_match_threshold
                ] or [estimate[0]]
                center_candidates = [
                    item for item in lane_candidates
                    if abs(item - estimate[1]) < self.cluster_match_threshold
                ] or [estimate[1]]
                for left in left_candidates:
                    possibles.extend([
                        [left, center, lane_candidate]
                        for center in center_candidates
                    ])
        return np.asarray(possibles, dtype=np.float64)

    def apply_lateral_rate_guard(
        self,
        proposed_lane,
        previous_lane,
        dt,
        curve_relief,
    ):
        """Apply a weak source-time center-rate guard after association."""
        curve_rate_scale = (
            self.lateral_innovation_curve
            / self.lateral_innovation_straight
        )
        rate_scale = float(np.interp(
            curve_relief,
            [0.0, 1.0],
            [1.0, curve_rate_scale],
        ))
        allowed_delta = self.lateral_max_rate_straight * rate_scale * dt
        requested_delta = float(proposed_lane[1] - previous_lane[1])
        limited_delta = float(np.clip(
            requested_delta, -allowed_delta, allowed_delta))
        rate_limited = not np.isclose(
            requested_delta, limited_delta, atol=1e-9)
        result = np.asarray(proposed_lane, dtype=np.float64).copy()
        result += previous_lane[1] + limited_delta - result[1]
        return result, rate_limited, allowed_delta

    def record_lateral_debug(
        self,
        lane_candidates,
        selected_lane,
        previous_lane,
        predicted_lane,
        temporal_center,
        innovation,
        innovation_gate,
        gate_rejected,
        curve_confirmed,
        rate_limited,
        allowed_delta,
        source_ns,
        dt,
        dt_valid,
    ):
        """Store detailed association state and emit throttled diagnostics."""
        self.last_lateral_debug = {
            'raw_candidates': [float(item) for item in lane_candidates],
            'selected_candidate': (
                float(selected_lane[1]) if selected_lane is not None else None
            ),
            'previous_lateral': float(previous_lane[1]),
            'predicted_lateral': float(predicted_lane[1]),
            'temporal_lateral': float(temporal_center),
            'curve_reference': self.curve_lateral_reference(),
            'innovation': (
                float(innovation) if innovation is not None else None
            ),
            'innovation_gate': float(innovation_gate),
            'gate_rejected': bool(gate_rejected),
            'curve_confirmed': bool(curve_confirmed),
            'curve_gate_bypass': bool(
                curve_confirmed
                and innovation is not None
                and abs(innovation) > innovation_gate
            ),
            'rate_limited': bool(rate_limited),
            'allowed_delta': float(allowed_delta),
            'outer_pair_active': bool(self.outer_pair_active),
            'outer_pair_held': bool(self.outer_pair_held),
            'outer_pair_center': (
                float(self.last_outer_pair_center)
                if self.last_outer_pair_center is not None else None
            ),
            'outer_pair_width': (
                float(self.outer_pair_width)
                if self.outer_pair_width is not None else None
            ),
            'outer_pair_miss_count': int(self.outer_pair_miss_count),
            'final_lateral': float(self.lane[1]),
            'curve_state': str(self.current_curve_state),
            'source_ns': int(source_ns),
            'dt_s': float(dt),
            'dt_valid': bool(dt_valid),
        }
        now = time.monotonic()
        should_log = (
            (self.publish_performance_stats or self.show_debug_windows)
            and (gate_rejected or rate_limited)
            and now - self.last_lateral_debug_log_monotonic >= 0.5
        )
        if should_log:
            self.get_logger().info(
                'Lateral association: '
                f'candidates={self.last_lateral_debug["raw_candidates"][:8]} '
                f'selected={self.last_lateral_debug["selected_candidate"]} '
                f'previous={previous_lane[1]:.1f} '
                f'predicted={predicted_lane[1]:.1f} '
                f'innovation={self.last_lateral_debug["innovation"]} '
                f'gate={innovation_gate:.1f} reject={gate_rejected} '
                f'curve_confirm={curve_confirmed} '
                f'outer_pair={self.outer_pair_active}/'
                f'held={self.outer_pair_held} '
                f'rate_limit={rate_limited} final={self.lane[1]:.1f} '
                f'state={self.current_curve_state} dt={dt:.3f}s '
                f'source_ns={source_ns}'
            )
            self.last_lateral_debug_log_monotonic = now

    def update_lateral_history(self, previous_center, source_ns, dt):
        """Update source-time state after a final lateral value is selected."""
        measured_velocity = (self.lane[1] - previous_center) / max(dt, 1e-6)
        self.last_lateral_velocity_px_s = (
            0.5 * self.last_lateral_velocity_px_s
            + 0.5 * measured_velocity
        )
        if source_ns > self.last_lateral_source_ns:
            self.last_lateral_source_ns = int(source_ns)

    def refine_lane_with_candidates_and_prediction(
        self,
        lane_candidates,
        predicted_lane,
        source_ns=0,
    ):
        """Associate Hough hypotheses temporally before updating lane[1]."""
        previous_lane = self.lane.copy()
        dt, dt_valid = self.lateral_source_dt(source_ns)
        self.priority_curve_center_x = None
        self.priority_curve_center_valid = False

        # Explicit lane changes are mission commands, not Hough switching.
        # Preserve their proven legacy behavior and keep the new gate center-only.
        if self.override_target_lane == 'go_left':
            self.reset_outer_pair_state()
            cos_angle = max(np.cos(self.angle), self.min_cos_angle)
            left_candidates = [
                item for item in lane_candidates
                if abs(item - predicted_lane[0])
                < self.cluster_match_threshold
            ]
            if left_candidates:
                best_left = min(
                    left_candidates,
                    key=lambda item: abs(item - predicted_lane[0]),
                )
                new_lane = np.array([
                    best_left,
                    best_left + self.left_to_center_dist / cos_angle,
                    best_left + self.outer_lane_dist / cos_angle,
                ])
                delta = np.clip(
                    new_lane - self.lane,
                    -self.max_lane_delta,
                    self.max_lane_delta,
                )
                self.lane += delta
            else:
                self.lane = predicted_lane
            self.force_correct_lane_with_curve()
            self.update_lateral_history(previous_lane[1], source_ns, dt)
            return

        if self.override_target_lane == 'go_right':
            self.reset_outer_pair_state()
            cos_angle = max(np.cos(self.angle), self.min_cos_angle)
            right_candidates = [
                item for item in lane_candidates
                if abs(item - predicted_lane[2])
                < self.cluster_match_threshold
            ]
            if right_candidates:
                best_right = min(
                    right_candidates,
                    key=lambda item: abs(item - predicted_lane[2]),
                )
                new_lane = np.array([
                    best_right - self.outer_lane_dist / cos_angle,
                    best_right - self.right_to_center_dist / cos_angle,
                    best_right,
                ])
                delta = np.clip(
                    new_lane - self.lane,
                    -self.max_lane_delta,
                    self.max_lane_delta,
                )
                self.lane += delta
            else:
                self.lane = predicted_lane
            self.force_correct_lane_with_curve()
            self.update_lateral_history(previous_lane[1], source_ns, dt)
            return

        curve_relief = self.lateral_curve_relief()
        innovation_gate = float(np.interp(
            curve_relief,
            [0.0, 1.0],
            [
                self.lateral_innovation_straight,
                self.lateral_innovation_curve,
            ],
        ))
        velocity_offset = float(np.clip(
            0.25 * self.last_lateral_velocity_px_s * dt,
            -0.25 * innovation_gate,
            0.25 * innovation_gate,
        ))
        temporal_center = float(predicted_lane[1] + velocity_offset)

        selected_lane = None
        selected_innovation = None
        gate_rejected = False
        curve_confirmed = False
        proposed_lane = np.asarray(predicted_lane, dtype=np.float64).copy()
        if lane_candidates:
            hypotheses = self.build_lane_hypotheses(lane_candidates)
            curve_reference = self.curve_lateral_reference()
            scored = []
            for hypothesis in hypotheses:
                residual = hypothesis - predicted_lane
                center_residual = hypothesis[1] - temporal_center
                # Center continuity receives extra weight because lane[1] is
                # the CTE sent directly to Stanley. Outer-lane geometry remains
                # part of the original triplet cost.
                score = (
                    residual[0] * residual[0]
                    + 2.0 * center_residual * center_residual
                    + residual[2] * residual[2]
                )
                innovation = float(hypothesis[1] - predicted_lane[1])
                hypothesis_curve_confirmed = False
                if curve_reference is not None:
                    curve_innovation = (
                        curve_reference - predicted_lane[1])
                    hypothesis_curve_confirmed = (
                        innovation * curve_innovation > 0.0
                        and abs(hypothesis[1] - curve_reference)
                        <= self.lateral_innovation_straight
                    )
                if (
                    abs(innovation) <= innovation_gate
                    or hypothesis_curve_confirmed
                ):
                    scored.append((
                        score,
                        hypothesis,
                        innovation,
                        hypothesis_curve_confirmed,
                    ))
            if scored:
                (
                    _,
                    selected_lane,
                    selected_innovation,
                    curve_confirmed,
                ) = min(
                    scored, key=lambda item: item[0])
                proposed_lane = (
                    self.lane_update_weight * selected_lane
                    + self.prediction_weight * predicted_lane
                )
            else:
                # One bad Hough frame is less trustworthy than the existing
                # heading-aware predictor. This is prediction, not an old-value
                # hold, so genuine curve motion can continue.
                gate_rejected = bool(len(hypotheses))

        delta = np.clip(
            proposed_lane - self.lane,
            -self.max_lane_delta,
            self.max_lane_delta,
        )
        self.lane += delta
        # Preserve the existing center-curve emergency correction, then place
        # the source-time safety guard last so no auxiliary path can bypass it.
        priority_curve_valid = self.force_correct_lane_with_curve()
        if priority_curve_valid:
            self.reset_outer_pair_state()
        else:
            self.stabilize_straight_lateral_with_outer_pair(
                lane_candidates,
                temporal_center,
            )
        guarded_lane, rate_limited, allowed_delta = (
            self.apply_lateral_rate_guard(
                self.lane,
                previous_lane,
                dt,
                curve_relief,
            )
        )
        final_delta = np.clip(
            guarded_lane - previous_lane,
            -self.max_lane_delta,
            self.max_lane_delta,
        )
        self.lane = previous_lane + final_delta
        self.update_lateral_history(previous_lane[1], source_ns, dt)
        self.record_lateral_debug(
            lane_candidates=lane_candidates,
            selected_lane=selected_lane,
            previous_lane=previous_lane,
            predicted_lane=predicted_lane,
            temporal_center=temporal_center,
            innovation=selected_innovation,
            innovation_gate=innovation_gate,
            gate_rejected=gate_rejected,
            curve_confirmed=curve_confirmed,
            rate_limited=rate_limited,
            allowed_delta=allowed_delta,
            source_ns=source_ns,
            dt=dt,
            dt_valid=dt_valid,
        )

    def mark_lane(self, img, lane=None, show=False, curve_points=None):
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if lane is None: lane = self.lane
        l1, l2, l3 = lane
        curve_angle_text = (
            f'{np.degrees(self.last_curve_heading):+.1f}'
            if self.last_curve_heading is not None
            else 'N/A'
        )
        heading_text = (
            f'Track {np.degrees(self.angle):+.1f} | '
            f'Control {np.degrees(self.control_heading):+.1f} | '
            f'Curve {curve_angle_text} deg | '
            f'Recovery {"ON" if self.curve_heading_recovering else "OFF"}'
        )
        cv2.putText(
            img,
            heading_text,
            (10, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        preview_text = (
            f'Preview {self.last_heading_preview_ratio:.3f}H | '
            f'Curvature {self.last_curve_curvature:.3f}'
            if self.last_heading_preview_ratio is not None
            and self.last_curve_curvature is not None
            else f'Preview fallback ({self.control_heading_source})'
        )
        cv2.putText(
            img,
            preview_text,
            (10, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 0),
            1,
            cv2.LINE_AA,
        )
        cv2.circle(img, (int(l1), self.bev_img_mid), 3, (255, 0, 0), 5, cv2.FILLED)
        cv2.circle(img, (int(l2), self.bev_img_mid), 3, (0, 255, 255), 5, cv2.FILLED)
        cv2.circle(img, (int(l3), self.bev_img_mid), 3, (0, 0, 255), 5, cv2.FILLED)

        mid_left, mid_right = (l1 + l2) / 2, (l2 + l3) / 2
        cv2.circle(img, (int(mid_left), self.bev_img_mid), 4, (0, 255, 0), 4, cv2.FILLED)
        cv2.putText(img, 'go_L', (int(mid_left)-20, self.bev_img_mid-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        cv2.circle(img, (int(mid_right), self.bev_img_mid), 4, (0, 255, 0), 4, cv2.FILLED)
        cv2.putText(img, 'go_R', (int(mid_right)-20, self.bev_img_mid-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        if curve_points is not None and len(curve_points) > 0:
            curve_points_int = curve_points.astype(np.int32)
            cv2.polylines(img, [curve_points_int], isClosed=False, color=(255, 0, 255), thickness=2)

        if self.override_target_lane:
            cv2.putText(img, f'Override: {self.override_target_lane}', (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if show:
            cv2.imshow('marked_lane_point', img)

    def visualize_results_on_original_frame(self, original_img, roi_poly, curve_msg, lane_center_bev, curve_center_bev, show=False):
        """
        요청사항을 반영한 새로운 시각화 함수:
        1. 원본 이미지에 ROI 영역(초록색)을 표시합니다.
        2. 원본 이미지에 구독한 중앙선(노란색)을 그립니다.
        3. BEV 공간에서 계산된 차선 검출 중앙값과 YOLO 중앙값 간의 오차를 좌측 상단에 표시합니다.
        """
        if not show:
            return

        display_img = original_img.copy()

        # 1. ROI 영역 표시
        if roi_poly is not None:
            cv2.polylines(display_img, [roi_poly.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)

        # 2. 구독한 curve(YOLO 중앙선) 표시
        if curve_msg is not None and curve_msg.points:
            curve_points = np.array([[[int(p.x), int(p.y)]] for p in curve_msg.points], dtype=np.int32)
            cv2.polylines(display_img, [curve_points], isClosed=False, color=(255, 255, 0), thickness=2)

        # 3. BEV 공간에서의 오차 계산 및 표시
        error_text = "Error: N/A"
        if lane_center_bev is not None and curve_center_bev is not None:
            # "노란 점"(차선 검출 결과)과 "curve"(YOLO 결과)의 거리 오차
            error = abs(lane_center_bev - curve_center_bev)
            error_text = f"Error: {error:.2f} px"
        
        cv2.putText(display_img, error_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        
        # 최종 결과 창 표시
        # cv2.imshow('Result on Original Frame', display_img)

    def show_roi_region(self, img, src, show=False):
        if show:
            cv2.polylines(img, [src.astype(int)], True, (0,255,0), 2, cv2.LINE_AA)
            cv2.imshow('roi region on the undistorted frame',img)

    def sync_callback(self, img_msg, detections_msg, curve_msg):
        callback_start = time.monotonic()
        callback_start_ns = (
            self.get_clock().now().nanoseconds
            if self.pipeline_timing_pub is not None else 0
        )
        self.sync_count += 1
        # <<< [수정] 마지막 싱크 시간 기록
        self.last_sync_time = self.get_clock().now()
        # >>>
        if not self.is_active: return
        source_ns = stamp_to_ns(img_msg.header.stamp)

        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert lane image: {exc}')
            return
        scene_targets = self.fresh_scene_targets(
            img_msg,
            frame.shape[1],
            frame.shape[0],
        )
        canny = self.apply_canny(frame, show=False)
        canny = self.mask_detected_objects(
            canny,
            detections_msg,
            scene_targets,
            frame,
            show=False,
        )
        
        bev, src = self.get_birds_eye_view(canny)
        self.transform_curve_to_bev(curve_msg)
        curve_heading = self.estimate_curve_heading(curve_msg)
        
        # self.show_roi_region(frame, src, show=False) # 새로운 함수로 대체됨
        lines = self.detect_lines_hough(bev, show=False)
        positions = self.filter_lines_by_angle(
            lines,
            show=False,
            curve_heading=curve_heading,
        )
        self.current_curve_state = curve_msg.state
        self.update_control_heading(curve_heading)
        lane_candidates = self.extract_lane_candidates_from_clusters(positions, show=False, base_img=bev)

        predicted_lane = self.predict_lane(show=False, base_img=bev)
        
        self.refine_lane_with_candidates_and_prediction(
            lane_candidates,
            predicted_lane,
            source_ns=source_ns,
        )
        if (
            self.shortcut_path_active
            and self.estimated_curve_center_x is not None
            and np.isfinite(self.estimated_curve_center_x)
            and 0.0 <= self.estimated_curve_center_x < self.bev_img_w
        ):
            # During a committed turn the routed curve is the selected road,
            # while Hough still contains the abandoned straight road. Follow
            # the routed center directly until the new road handoff completes.
            self.lane += self.estimated_curve_center_x - self.lane[1]
        # A lane center outside the BEV has no physical meaning and can create
        # an unnecessarily large CTE after all visual points disappear.  Shift
        # the complete lane triplet so its center stays on the image boundary.
        clipped_center = float(np.clip(
            self.lane[1],
            0.0,
            float(self.bev_img_w - 1),
        ))
        self.lane += clipped_center - self.lane[1]
        
        self.mark_lane(bev, show=self.show_debug_windows, curve_points=self.bev_curve_points)
        
        # --- [변경] 새로운 시각화 함수 호출 ---
        # BEV 공간에서 YOLO 중앙선의 x좌표 평균 계산
        curve_center_bev_x = None
        if self.bev_curve_points is not None and len(self.bev_curve_points) > 0:
            curve_center_bev_x = np.mean(self.bev_curve_points[:, 0])

        # 새로운 시각화 함수 호출 (show=True로 설정하여 활성화)
        self.visualize_results_on_original_frame(
            original_img=frame,
            roi_poly=self.roi_src_poly,
            curve_msg=curve_msg,
            lane_center_bev=self.lane[1],
            curve_center_bev=curve_center_bev_x,
            show=self.show_debug_windows
        )
        # ------------------------------------

        state_msg = XycarState()
        if self.shortcut_path_active:
            state_msg.drive_mode = 'center'
            target_x = self.lane[1]
        elif self.override_target_lane == 'go_right':
            state_msg.drive_mode = 'right'
            target_x = (self.lane[1] + self.lane[2]) / 2
        elif self.override_target_lane == 'go_left':
            state_msg.drive_mode = 'left'
            target_x = (self.lane[0] + self.lane[1]) / 2
        else:
            state_msg.drive_mode = 'center'
            target_x = self.lane[1]
        state_msg.target_point.x = float(target_x)
        state_msg.target_point.y = float(self.bev_img_mid)
        state_msg.target_point.z = self.control_heading
        if self.xycar_state_pub is not None:
            self.xycar_state_pub.publish(state_msg)
        if self.stamped_state_pub is not None:
            stamped_state = StampedXycarState()
            stamped_state.header = img_msg.header
            stamped_state.state = state_msg
            self.stamped_state_pub.publish(stamped_state)
        if (
            self.lane_control_state_pub is not None
            or self.lane_control_state_v2_pub is not None
            or self.lane_control_state_v3_pub is not None
        ):
            control_state = LaneControlState()
            control_state.header = img_msg.header
            control_state.state = state_msg
            control_state.curve_state = str(curve_msg.state)
            control_state.heading_source = self.control_heading_source
            control_state.curve_heading_valid = self.curve_heading_valid
            active_severity = self.last_curve_curvature_severity
            if self.control_heading_source == 'curve_hold':
                active_severity = self.last_valid_curvature_severity
            elif self.control_heading_source == 'hough':
                active_severity = 0.0
            control_state.curvature = float(
                self.last_curve_curvature
                if self.last_curve_curvature is not None
                else float('nan')
            )
            control_state.curvature_severity = float(active_severity)
            control_state.heading_preview_ratio = float(
                self.last_heading_preview_ratio
                if self.last_heading_preview_ratio is not None
                else float('nan')
            )
            control_state.curve_fit_error_ratio = float(
                self.last_curve_fit_error_ratio
                if self.last_curve_fit_error_ratio is not None
                else float('nan')
            )
            control_state.path_confidence = float(self.path_confidence)
            if self.lane_control_state_pub is not None:
                self.lane_control_state_pub.publish(control_state)
            if self.lane_control_state_v2_pub is not None:
                extended_state = LaneControlStateV2()
                extended_state.control = control_state
                extended_state.signed_curvature_hint = float(
                    self.last_curve_signed_curvature
                    if self.last_curve_signed_curvature is not None
                    else float('nan')
                )
                self.lane_control_state_v2_pub.publish(extended_state)
            if self.lane_control_state_v3_pub is not None:
                extended_state_v2 = LaneControlStateV2()
                extended_state_v2.control = control_state
                extended_state_v2.signed_curvature_hint = float(
                    self.last_curve_signed_curvature
                    if self.last_curve_signed_curvature is not None
                    else float('nan')
                )
                extended_state_v3 = LaneControlStateV3()
                extended_state_v3.control = extended_state_v2
                extended_state_v3.preview_signed_curvature_hint = float(
                    self.last_curve_preview_signed_curvature
                    if self.last_curve_preview_signed_curvature is not None
                    else float('nan')
                )
                self.lane_control_state_v3_pub.publish(extended_state_v3)
        self.output_count += 1
        output_now = time.monotonic()
        output_now_ns = (
            self.get_clock().now().nanoseconds
            if self.pipeline_timing_pub is not None else 0
        )
        if self.pipeline_timing_pub is not None:
            timing = PipelineTiming()
            timing.header = img_msg.header
            timing.stage = 'lane_detector_hough'
            timing.receive_stamp_ns = int(callback_start_ns)
            timing.start_stamp_ns = int(callback_start_ns)
            timing.end_stamp_ns = int(output_now_ns)
            timing.queue_wait_ms = 0.0
            timing.compute_ms = (output_now - callback_start) * 1000.0
            timing.received_count = int(self.sync_count)
            timing.processed_count = int(self.output_count)
            timing.replaced_count = 0
            timing.input_reused = False
            self.pipeline_timing_pub.publish(timing)
        source_age_ns = self.get_clock().now().nanoseconds - source_ns
        self.last_source_age_ms = (
            source_age_ns * 1e-6
            if source_ns > 0 and source_age_ns >= 0
            else float('nan')
        )
        if self.last_output_monotonic is not None:
            output_gap_s = output_now - self.last_output_monotonic
            self.max_output_gap_s = max(self.max_output_gap_s, output_gap_s)
            self.output_gap_samples_ms.append(output_gap_s * 1000.0)
        self.last_output_monotonic = output_now
        self.callback_samples_ms.append(
            (output_now - callback_start) * 1000.0)
        if self.show_debug_windows:
            cv2.waitKey(1)

    def log_performance(self):
        now = time.monotonic()
        elapsed = max(now - self.last_performance_monotonic, 1e-6)
        counts = (self.sync_count, self.output_count)
        sync_rate = (counts[0] - self.last_performance_counts[0]) / elapsed
        output_rate = (counts[1] - self.last_performance_counts[1]) / elapsed
        samples = np.asarray(self.callback_samples_ms, dtype=np.float64)
        gap_samples = np.asarray(
            self.output_gap_samples_ms, dtype=np.float64)
        p50 = float(np.percentile(samples, 50)) if samples.size else float('nan')
        p95 = float(np.percentile(samples, 95)) if samples.size else float('nan')
        gap_p95 = (
            float(np.percentile(gap_samples, 95))
            if gap_samples.size else float('nan')
        )
        self.get_logger().info(
            'Lane detector performance: '
            f'sync={sync_rate:.1f}Hz output={output_rate:.1f}Hz '
            f'callback_p50/p95={p50:.1f}/{p95:.1f}ms '
            f'gap_p95/max={gap_p95:.1f}/'
            f'{self.max_output_gap_s * 1000.0:.1f}ms '
            f'source_age={self.last_source_age_ms:.1f}ms'
        )
        self.last_performance_monotonic = now
        self.last_performance_counts = counts
        self.max_output_gap_s = 0.0
        
    def trigger_callback(self, msg):
        command = msg.data.strip().lower()
        if command == 'go_right':
            self.override_target_lane = 'go_right'
            self.obstacle_recovery_until_monotonic = 0.0
            self.reset_priority_curve_center_validation()
        elif command == 'go_left':
            self.override_target_lane = 'go_left'
            self.obstacle_recovery_until_monotonic = 0.0
            self.reset_priority_curve_center_validation()
        elif command == 'reset':
            self.override_target_lane = None
            self.obstacle_recovery_until_monotonic = (
                time.monotonic() + self.obstacle_recovery_duration_s)
            self.reset_priority_curve_center_validation()
        else:
            self.get_logger().warn(
                f'⚠️ [Command Received] unknown: {command}')

    def shortcut_path_active_callback(self, msg):
        self.shortcut_path_active = bool(msg.data)
    
def main(args=None):
    rclpy.init(args=args)
    node = Lane_Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
