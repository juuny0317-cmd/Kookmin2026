#! /usr/bin/env python
# -*- coding:utf-8 -*-
# ==================================================================================
#
#   FILE: centerlane_tracer.py
#
#   DESCRIPTION:
#       YOLOv8 Bounding Box 탐지 결과를 기반으로 중앙 차선을 추적하는 ROS 2 노드.
#       디버그 모드를 통해 실시간으로 파라미터를 조절하며 시각화하거나,
#       Headless 모드로 UI 없이 최적화된 파라미터로 커브 토픽만 발행할 수 있습니다.
#
#   LOGIC FLOW:
#       1. 이미지 토픽('/image_raw')과 YOLO 탐지 토픽('/yolo_detections')을 동기화하여 수신.
#       2. Debug 모드가 켜져 있으면, UI 트랙바에서 실시간으로 파라미터를 읽어와 업데이트.
#          - 꺼져 있으면, 코드에 설정된 기본 파라미터 값을 사용.
#       3. YOLO 탐지 결과 중 'center_line' 클래스이면서 신뢰도가 0.4 이상인 것만 필터링.
#       4. 각 BBox 내에서 이미지 처리(Adaptive Threshold -> Canny Edge -> Hough Line Transform)를
#          수행하여 가장 긴 선분을 찾고, 선분과 BBox의 교차점을 '대표점'으로 추출.
#          - 선분 탐지 실패 시 BBox의 중앙점을 fallback으로 사용.
#       5. 모든 대표점들을 화면 아래쪽부터 위쪽으로 Greedy 알고리즘을 통해 정렬.
#          - 이때 매우 가까운(2px 이내) 점들은 하나의 점으로 통합하여 노이즈를 제거.
#       6. 정렬된 대표점들을 기반으로 'spline' 또는 'linear' 모드로 커브 피팅.
#          - 양 끝점은 이미지 경계까지 연장하여 경로를 생성.
#       7. 최종 생성된 커브(점들의 배열)를 '/center_curve' 토픽으로 발행.
#       8. Debug 모드가 켜져 있으면, 모든 처리 과정과 최종 결과를 화면에 시각화하여 출력.
#
# ==================================================================================


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from custom_interfaces.msg import (
    Curve,
    Detection,
    Detections,
    PipelineTiming,
    Point2D,
)
import message_filters
import math
import threading
import time
from collections import deque
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from scipy.interpolate import splprep, splev

class CenterlaneTracer(Node):
    """
    대표점들에 대해 적응형 곡선 피팅을 수행하고,
    실시간 파라미터 조정을 위한 트랙바와 디버그 창을 제공합니다.
    Debug 모드가 False일 경우, UI 없이 설정된 기본값으로 커브 토픽만 발행합니다.
    """
    def __init__(self):
        super().__init__('centerlane_tracer')

        # =======================================================
        # True로 설정 시 디버그 창 활성화, False로 설정 시 Headless 모드
        self.declare_parameter('debug_view', False)
        self.declare_parameter('verbose_sort_debug', False)
        self.declare_parameter('image_topic', '/perception/lane/image')
        self.declare_parameter(
            'lane_detections_topic', '/lane_yolo/detections')
        self.declare_parameter(
            'scene_detections_topic', '/scene_yolo/detections')
        self.declare_parameter('center_curve_topic', '/center_curve')
        self.declare_parameter('image_qos_reliability', 'reliable')
        self.declare_parameter('scene_detection_max_age_s', 0.6)
        self.declare_parameter('scene_detection_source_width', 640)
        self.declare_parameter('scene_detection_source_height', 480)
        self.declare_parameter('enable_scene_detection_cache', True)
        self.declare_parameter('base_input_width', 640)
        self.declare_parameter('base_input_height', 480)
        self.declare_parameter('height_threshold_px', 220)
        self.declare_parameter('adaptive_block_size_px', 13)
        self.declare_parameter('hough_threshold', 20)
        self.declare_parameter('hough_min_line_length_px', 20)
        self.declare_parameter('hough_max_line_gap_px', 5)
        self.declare_parameter('similarity_threshold_px', 70)
        self.declare_parameter('rmse_threshold_px', 30)
        self.declare_parameter('bottom_box_height_threshold_px', 10)
        self.declare_parameter(
            'bottom_point_height_diff_threshold_px', 5)
        self.declare_parameter('min_bbox_width_px', 5)
        self.declare_parameter('min_bbox_height_px', 5)
        self.declare_parameter('x_distance_threshold_px', 300)
        self.declare_parameter('point_merge_distance_px', 2.0)
        self.declare_parameter('spline_smoothing_px2', 30.0)
        self.declare_parameter('publish_performance_stats', False)
        self.declare_parameter('performance_log_period_s', 5.0)
        self.declare_parameter('publish_pipeline_timing', False)
        self.declare_parameter('pipeline_timing_topic', '/pipeline_timing')
        self.declare_parameter('sync_queue_size', 5)
        self.declare_parameter('sync_slop', 0.1)
        self.debug = bool(self.get_parameter('debug_view').value)
        self.verbose_sort_debug = bool(
            self.get_parameter('verbose_sort_debug').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.lane_detections_topic = str(
            self.get_parameter('lane_detections_topic').value)
        self.scene_detections_topic = str(
            self.get_parameter('scene_detections_topic').value)
        self.center_curve_topic = str(
            self.get_parameter('center_curve_topic').value)
        self.scene_detection_max_age_s = max(
            0.0,
            float(self.get_parameter('scene_detection_max_age_s').value),
        )
        self.scene_detection_source_width = max(
            1,
            int(self.get_parameter('scene_detection_source_width').value),
        )
        self.scene_detection_source_height = max(
            1,
            int(self.get_parameter('scene_detection_source_height').value),
        )
        self.enable_scene_detection_cache = bool(
            self.get_parameter('enable_scene_detection_cache').value)
        self.publish_performance_stats = bool(
            self.get_parameter('publish_performance_stats').value)
        self.performance_log_period_s = max(
            0.1,
            float(self.get_parameter('performance_log_period_s').value),
        )
        # =======================================================

        self.BASE_INPUT_WIDTH = float(max(
            1, int(self.get_parameter('base_input_width').value)))
        self.BASE_INPUT_HEIGHT = float(max(
            1, int(self.get_parameter('base_input_height').value)))
        self.HEIGHT_THRESHOLD = int(
            self.get_parameter('height_threshold_px').value)
        self.POINT_MERGE_DISTANCE = float(
            self.get_parameter('point_merge_distance_px').value)
        self.SPLINE_SMOOTHING = float(
            self.get_parameter('spline_smoothing_px2').value)
        self.bridge = CvBridge()
        self.current_frame = None
        
        self.frame_idx = 0

        self._initialize_parameters()
        
        self.lane_state = "Straight"
        self.state_candidate = "Straight"
        self.state_confidence_counter = 0
        
        self.rmse_history = deque(maxlen=self.params['history_size'])

        self._latest_scene_detections = None
        self._scene_lock = threading.Lock()
        self._scene_frame_mismatch_warned = False
        self._processed_frames = 0
        self._published_frames = 0
        self._last_publish_time = None
        self._max_publish_gap_s = 0.0
        self._callback_durations_ms = deque(maxlen=512)
        self._last_performance_log_time = time.monotonic()
        self._last_performance_processed_frames = 0
        self._last_performance_published_frames = 0

        image_qos_reliability = str(
            self.get_parameter('image_qos_reliability').value
        ).strip().lower()
        if image_qos_reliability not in ('best_effort', 'reliable'):
            self.get_logger().warn(
                'Unknown image_qos_reliability '
                f'"{image_qos_reliability}"; using reliable.')
            image_qos_reliability = 'reliable'
        image_reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if image_qos_reliability == 'best_effort'
            else ReliabilityPolicy.RELIABLE
        )

        latest_image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=image_reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        lane_data_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latest_scene_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_subscription = message_filters.Subscriber(
            self,
            Image,
            self.image_topic,
            qos_profile=latest_image_qos,
        )
        self.yolo_subscription = message_filters.Subscriber(
            self,
            Detections,
            self.lane_detections_topic,
            qos_profile=lane_data_qos,
        )
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_subscription, self.yolo_subscription],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop').value),
        )
        self.time_synchronizer.registerCallback(self.synchronized_callback)

        # Scene detections run at a lower rate than lane perception. Keep them
        # outside the synchronizer so a 4 Hz scene stream cannot throttle the
        # 12 Hz lane path.
        self.scene_subscription = None
        if self.enable_scene_detection_cache:
            self.scene_subscription = self.create_subscription(
                Detections,
                self.scene_detections_topic,
                self.scene_detections_callback,
                latest_scene_qos,
            )

        self.curve_publisher = self.create_publisher(
            Curve,
            self.center_curve_topic,
            lane_data_qos,
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

        self.get_logger().info(
            'Perception I/O: '
            f'image={self.image_topic} '
            f'(KEEP_LAST depth=1, {image_qos_reliability}), '
            f'lane_detections={self.lane_detections_topic} '
            '(RELIABLE depth=5), '
            f'scene_detections={self.scene_detections_topic} '
            f'(enabled={self.enable_scene_detection_cache}, '
            'RELIABLE depth=1, cached), '
            f'curve={self.center_curve_topic} (RELIABLE depth=5)'
        )

        if self.debug:
            self.main_window_name = 'Centerlane Tracer'
            self.debug_window_names = ['Grayscale (AdaptiveThresh)', 'Canny Edges', 'Hough Lines']
            cv2.namedWindow(self.main_window_name, cv2.WINDOW_AUTOSIZE)
            for name in self.debug_window_names:
                cv2.namedWindow(name, cv2.WINDOW_NORMAL)

            cv2.setMouseCallback(self.main_window_name, self.mouse_event_callback)
            
            def nothing(x): pass
            cv2.createTrackbar('Adapt Method', self.main_window_name, self.params['adapt_method_val'], 1, nothing)
            cv2.createTrackbar('Block Size', self.main_window_name, self.params['block_size_val'], 20, nothing)
            cv2.createTrackbar('C Constant', self.main_window_name, self.params['c_constant_val'], 50, nothing)
            cv2.createTrackbar('Canny Thresh1', self.main_window_name, self.params['canny_thresh1'], 255, nothing)
            cv2.createTrackbar('Canny Thresh2', self.main_window_name, self.params['canny_thresh2'], 255, nothing)
            cv2.createTrackbar('Hough Threshold', self.main_window_name, self.params['hough_threshold'], 100, nothing)
            cv2.createTrackbar('Hough MinLineLength', self.main_window_name, self.params['hough_min_len'], 100, nothing)
            cv2.createTrackbar('Hough MaxLineGap', self.main_window_name, self.params['hough_max_gap'], 100, nothing)
            cv2.createTrackbar('Similarity Thresh', self.main_window_name, self.params['similarity_thresh'], 100, nothing)
            cv2.createTrackbar('Slope Thresh x100', self.main_window_name, self.params['slope_thresh_x100'], 1000, nothing)
            cv2.createTrackbar('Fit Mode', self.main_window_name, self.params['fit_mode_val'], 1, nothing)
            
            cv2.createTrackbar('Draw BBox', self.main_window_name, int(self.params['draw_bbox']), 1, nothing)
            cv2.createTrackbar('Draw Points', self.main_window_name, int(self.params['draw_points']), 1, nothing)
            cv2.createTrackbar('Draw Numbers', self.main_window_name, int(self.params['draw_numbers']), 1, nothing)
            cv2.createTrackbar('Draw Hough', self.main_window_name, int(self.params['draw_hough']), 1, nothing)
            cv2.createTrackbar('Draw Curve', self.main_window_name, int(self.params['draw_curve']), 1, nothing)
            
            cv2.createTrackbar('RMSE Thresh', self.main_window_name, self.params['rmse_thresh'], 30, nothing)
            cv2.createTrackbar('History Size', self.main_window_name, self.params['history_size'], 20, nothing)
            cv2.createTrackbar('Confidence Thresh', self.main_window_name, self.params['confidence_thresh'], 20, nothing)

            cv2.createTrackbar('Bottom Box H', self.main_window_name, self.params['bottom_box_height_thresh'], 100, nothing)
            cv2.createTrackbar('Bottom Pt H Diff', self.main_window_name, self.params['bottom_point_height_diff_thresh'], 20, nothing)

            cv2.createTrackbar('Angle Sim', self.main_window_name, int(self.params['angle_similarity_thresh']), 90, nothing)
            cv2.createTrackbar('Length Sim', self.main_window_name, int(self.params['length_similarity_thresh'] * 100), 100, nothing)

            cv2.createTrackbar('Min BBox W', self.main_window_name, self.params['min_bbox_width'], 50, nothing)
            cv2.createTrackbar('Min BBox H', self.main_window_name, self.params['min_bbox_height'], 50, nothing)
            
            cv2.createTrackbar('X Dist Thresh', self.main_window_name, self.params['x_dist_thresh'], 640, nothing)

            self.get_logger().info("Debug mode ON. Trackbars are enabled.")
        else:
            self.get_logger().info("Debug mode OFF. Running in headless mode.")

        self.get_logger().info('CenterlaneTracer has been started.')

    @staticmethod
    def _stamp_to_nanoseconds(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _scaled_int(value, scale, minimum=0):
        scaled = math.floor(float(value) * float(scale) + 0.5)
        return max(minimum, int(scaled))

    @classmethod
    def _scaled_odd_kernel(cls, value, scale, minimum=3):
        scaled = cls._scaled_int(value, scale, minimum=minimum)
        if scaled % 2 == 0:
            scaled += 1
        return scaled

    def _scaled_processing_parameters(self, frame_width, frame_height):
        """Scale only parameters expressed in the input-image coordinates."""
        scale_x = float(frame_width) / self.BASE_INPUT_WIDTH
        scale_y = float(frame_height) / self.BASE_INPUT_HEIGHT
        length_scale = (scale_x + scale_y) * 0.5
        base_block_size = self.params['block_size_val'] * 2 + 3

        return {
            'adapt_method': (
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C
                if self.params['adapt_method_val'] == 1
                else cv2.ADAPTIVE_THRESH_MEAN_C
            ),
            'block_size': self._scaled_odd_kernel(
                base_block_size, length_scale),
            'c_constant': self.params['c_constant_val'] - 25,
            'canny_thresh1': self.params['canny_thresh1'],
            'canny_thresh2': self.params['canny_thresh2'],
            'hough_threshold': self._scaled_int(
                self.params['hough_threshold'], length_scale, minimum=1),
            'hough_min_len': self._scaled_int(
                self.params['hough_min_len'], length_scale, minimum=1),
            'hough_max_gap': self._scaled_int(
                self.params['hough_max_gap'], length_scale, minimum=0),
            'similarity_thresh': (
                self.params['similarity_thresh'] * length_scale),
            'slope_threshold': self.params['slope_thresh_x100'] / 100.0,
            'bottom_box_height_thresh': (
                self.params['bottom_box_height_thresh'] * scale_y),
            'bottom_point_height_diff_thresh': (
                self.params['bottom_point_height_diff_thresh'] * scale_y),
            'angle_similarity_thresh': self.params[
                'angle_similarity_thresh'],
            'length_similarity_thresh': self.params[
                'length_similarity_thresh'],
            'min_bbox_width': self.params['min_bbox_width'] * scale_x,
            'min_bbox_height': self.params['min_bbox_height'] * scale_y,
            'x_dist_thresh': self.params['x_dist_thresh'] * scale_x,
            'height_threshold': self.HEIGHT_THRESHOLD * scale_y,
            'rmse_thresh': self.params['rmse_thresh'] * scale_x,
            'point_merge_distance': max(
                1.0, self.POINT_MERGE_DISTANCE * length_scale),
            # scipy's smoothing value bounds a sum of squared pixel errors.
            'spline_smoothing': (
                self.SPLINE_SMOOTHING * length_scale * length_scale),
        }

    def scene_detections_callback(self, msg):
        with self._scene_lock:
            self._latest_scene_detections = msg

    def _fresh_scaled_scene_vehicles(
            self, image_msg, frame_width, frame_height):
        if not self.enable_scene_detection_cache:
            return [], None

        with self._scene_lock:
            scene_msg = self._latest_scene_detections

        if scene_msg is None:
            return [], None

        image_stamp_ns = self._stamp_to_nanoseconds(image_msg.header.stamp)
        scene_stamp_ns = self._stamp_to_nanoseconds(scene_msg.header.stamp)
        if (
            image_msg.header.frame_id
            and scene_msg.header.frame_id
            and image_msg.header.frame_id != scene_msg.header.frame_id
        ):
            if not self._scene_frame_mismatch_warned:
                self.get_logger().warn(
                    'Ignoring scene detections with mismatched frame_id: '
                    f'lane={image_msg.header.frame_id!r}, '
                    f'scene={scene_msg.header.frame_id!r}'
                )
                self._scene_frame_mismatch_warned = True
            return [], None
        if image_stamp_ns <= 0 or scene_stamp_ns <= 0:
            return [], None

        scene_age_s = (image_stamp_ns - scene_stamp_ns) * 1e-9
        if (
            scene_age_s < 0.0
            or scene_age_s > self.scene_detection_max_age_s
        ):
            return [], scene_age_s

        scale_x = (
            float(frame_width) / self.scene_detection_source_width)
        scale_y = (
            float(frame_height) / self.scene_detection_source_height)
        scaled_vehicles = []
        for detection in scene_msg.detections:
            if detection.class_name != 'obstacle_vehicle':
                continue

            x1 = int(round(detection.xmin * scale_x))
            y1 = int(round(detection.ymin * scale_y))
            x2 = int(round(detection.xmax * scale_x))
            y2 = int(round(detection.ymax * scale_y))
            x1 = max(0, min(frame_width - 1, x1))
            y1 = max(0, min(frame_height - 1, y1))
            x2 = max(x1 + 1, min(frame_width, x2))
            y2 = max(y1 + 1, min(frame_height, y2))

            scaled = Detection()
            scaled.class_name = detection.class_name
            scaled.confidence = detection.confidence
            scaled.xmin = x1
            scaled.ymin = y1
            scaled.xmax = x2
            scaled.ymax = y2
            scaled_vehicles.append(scaled)

        return scaled_vehicles, scene_age_s

    def _record_performance(
            self, callback_start, image_msg, scene_age_s):
        if not self.publish_performance_stats:
            return

        now = time.monotonic()
        callback_ms = (now - callback_start) * 1000.0
        self._callback_durations_ms.append(callback_ms)
        self._processed_frames += 1
        self._published_frames += 1
        if self._last_publish_time is not None:
            self._max_publish_gap_s = max(
                self._max_publish_gap_s,
                now - self._last_publish_time,
            )
        self._last_publish_time = now

        elapsed_s = now - self._last_performance_log_time
        if elapsed_s < self.performance_log_period_s:
            return

        processed_delta = (
            self._processed_frames
            - self._last_performance_processed_frames
        )
        published_delta = (
            self._published_frames
            - self._last_performance_published_frames
        )
        callback_values = np.asarray(
            self._callback_durations_ms, dtype=np.float64)
        callback_p50 = float(np.percentile(callback_values, 50))
        callback_p95 = float(np.percentile(callback_values, 95))

        source_stamp_ns = self._stamp_to_nanoseconds(image_msg.header.stamp)
        source_age_ms = float('nan')
        if source_stamp_ns > 0:
            source_age_ns = self.get_clock().now().nanoseconds - source_stamp_ns
            if source_age_ns >= 0:
                source_age_ms = source_age_ns * 1e-6

        scene_age_text = (
            'none' if scene_age_s is None else f'{scene_age_s * 1000.0:.1f}')
        self.get_logger().info(
            'Centerlane pipeline: '
            f'callback_rate={processed_delta / elapsed_s:.1f}Hz, '
            f'curve_rate={published_delta / elapsed_s:.1f}Hz, '
            f'callback_p50={callback_p50:.1f}ms, '
            f'callback_p95={callback_p95:.1f}ms, '
            f'max_curve_gap={self._max_publish_gap_s * 1000.0:.1f}ms, '
            f'source_age={source_age_ms:.1f}ms, '
            f'scene_age={scene_age_text}ms'
        )
        self._last_performance_processed_frames = self._processed_frames
        self._last_performance_published_frames = self._published_frames
        self._last_performance_log_time = now
        self._max_publish_gap_s = 0.0

    def _initialize_parameters(self):
        adaptive_block_size = max(
            3, int(self.get_parameter('adaptive_block_size_px').value))
        if adaptive_block_size % 2 == 0:
            adaptive_block_size += 1
        self.params = {
            'adapt_method_val': 1,
            'block_size_val': (adaptive_block_size - 3) // 2,
            'c_constant_val': 5,
            'canny_thresh1': 50,
            'canny_thresh2': 150,
            'hough_threshold': int(
                self.get_parameter('hough_threshold').value),
            'hough_min_len': int(
                self.get_parameter('hough_min_line_length_px').value),
            'hough_max_gap': int(
                self.get_parameter('hough_max_line_gap_px').value),
            'similarity_thresh': int(round(float(
                self.get_parameter('similarity_threshold_px').value))),
            'fit_mode_val': 1,
            'slope_thresh_x100': 13,
            'draw_bbox': True,
            'draw_points': True,
            'draw_numbers': True,
            'draw_hough': True,
            'draw_curve': True,

            'rmse_thresh': int(round(float(
                self.get_parameter('rmse_threshold_px').value))),
            'history_size': 5,
            'confidence_thresh': 5,

            'bottom_box_height_thresh': int(round(float(self.get_parameter(
                'bottom_box_height_threshold_px').value))),
            'bottom_point_height_diff_thresh': int(round(float(
                self.get_parameter(
                    'bottom_point_height_diff_threshold_px').value))),
            'angle_similarity_thresh': 20.0,
            'length_similarity_thresh': 0.6,
            'min_bbox_width': int(round(float(
                self.get_parameter('min_bbox_width_px').value))),
            'min_bbox_height': int(round(float(
                self.get_parameter('min_bbox_height_px').value))),
            'x_dist_thresh': int(round(float(
                self.get_parameter('x_distance_threshold_px').value))),
        }
    
    def _update_params_from_trackbars(self):
        self.params['adapt_method_val'] = cv2.getTrackbarPos('Adapt Method', self.main_window_name)
        self.params['block_size_val'] = cv2.getTrackbarPos('Block Size', self.main_window_name)
        self.params['c_constant_val'] = cv2.getTrackbarPos('C Constant', self.main_window_name)
        self.params['canny_thresh1'] = cv2.getTrackbarPos('Canny Thresh1', self.main_window_name)
        self.params['canny_thresh2'] = cv2.getTrackbarPos('Canny Thresh2', self.main_window_name)
        self.params['hough_threshold'] = cv2.getTrackbarPos('Hough Threshold', self.main_window_name)
        self.params['hough_min_len'] = cv2.getTrackbarPos('Hough MinLineLength', self.main_window_name)
        self.params['hough_max_gap'] = cv2.getTrackbarPos('Hough MaxLineGap', self.main_window_name)
        self.params['similarity_thresh'] = cv2.getTrackbarPos('Similarity Thresh', self.main_window_name)
        self.params['slope_thresh_x100'] = cv2.getTrackbarPos('Slope Thresh x100', self.main_window_name)
        self.params['fit_mode_val'] = cv2.getTrackbarPos('Fit Mode', self.main_window_name)
        self.params['draw_bbox'] = cv2.getTrackbarPos('Draw BBox', self.main_window_name) == 1
        self.params['draw_points'] = cv2.getTrackbarPos('Draw Points', self.main_window_name) == 1
        self.params['draw_numbers'] = cv2.getTrackbarPos('Draw Numbers', self.main_window_name) == 1
        self.params['draw_hough'] = cv2.getTrackbarPos('Draw Hough', self.main_window_name) == 1
        self.params['draw_curve'] = cv2.getTrackbarPos('Draw Curve', self.main_window_name) == 1
        
        self.params['rmse_thresh'] = cv2.getTrackbarPos('RMSE Thresh', self.main_window_name)
        new_history_size = cv2.getTrackbarPos('History Size', self.main_window_name)
        if new_history_size > 0 and self.rmse_history.maxlen != new_history_size:
            self.rmse_history = deque(self.rmse_history, maxlen=new_history_size)
        self.params['history_size'] = new_history_size
        self.params['confidence_thresh'] = cv2.getTrackbarPos('Confidence Thresh', self.main_window_name)
        
        self.params['bottom_box_height_thresh'] = cv2.getTrackbarPos('Bottom Box H', self.main_window_name)
        self.params['bottom_point_height_diff_thresh'] = cv2.getTrackbarPos('Bottom Pt H Diff', self.main_window_name)

        self.params['angle_similarity_thresh'] = float(cv2.getTrackbarPos('Angle Sim', self.main_window_name))
        self.params['length_similarity_thresh'] = cv2.getTrackbarPos('Length Sim', self.main_window_name) / 100.0
        
        self.params['min_bbox_width'] = cv2.getTrackbarPos('Min BBox W', self.main_window_name)
        self.params['min_bbox_height'] = cv2.getTrackbarPos('Min BBox H', self.main_window_name)
        
        self.params['x_dist_thresh'] = cv2.getTrackbarPos('X Dist Thresh', self.main_window_name)


    def mouse_event_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.current_frame is not None:
            bgr_pixel = self.current_frame[y, x]
            hsv_pixel = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
            self.get_logger().info(f'Click at ({x}, {y}) | BGR: {bgr_pixel} -> HSV: {hsv_pixel}')

    def synchronized_callback(self, image_msg, yolo_msg):
        callback_start = time.monotonic()
        callback_start_ns = (
            self.get_clock().now().nanoseconds
            if self.pipeline_timing_pub is not None else 0
        )
        self.frame_idx += 1
        
        if self.debug:
            self._update_params_from_trackbars()
        
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            if self.debug:
                self.current_frame = frame.copy()
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        h, w, _ = frame.shape

        proc_params = self._scaled_processing_parameters(w, h)
        fit_mode = 'linear' if self.params['fit_mode_val'] == 1 else 'spline'

        vehicle_detections, scene_age_s = (
            self._fresh_scaled_scene_vehicles(image_msg, w, h)
        )
        
        all_center_lane_detections = [
            det for det in yolo_msg.detections if det.class_name == 'center_line'
        ]

        center_lane_detections = [
            det for det in all_center_lane_detections if det.confidence >= 0.0
        ]
        
        lanes_to_keep = []
        
        for lane_det in center_lane_detections:
            should_filter = False
            for vehicle_det in vehicle_detections:
                v_xmin, v_ymin, v_xmax, v_ymax = vehicle_det.xmin, vehicle_det.ymin, vehicle_det.xmax, vehicle_det.ymax
                l_xmin, l_ymin, l_xmax, l_ymax = lane_det.xmin, lane_det.ymin, lane_det.xmax, lane_det.ymax
                filter_zone_y_max = v_ymin + (v_ymax - v_ymin) * 0.7
                is_x_overlap = (l_xmin < v_xmax) and (l_xmax > v_xmin)
                is_lane_bottom_in_zone = (l_ymax > v_ymin) and (l_ymax < filter_zone_y_max)
                
                if is_x_overlap and is_lane_bottom_in_zone:
                    should_filter = True
                    break
            
            if not should_filter:
                lanes_to_keep.append(lane_det)

        filtered_detections = [
            det for det in lanes_to_keep
            if det.ymax > proc_params['height_threshold']
        ]
        
        all_rep_points_unsorted, lines_to_draw, debug_images = self._extract_representative_points(frame, filtered_detections, proc_params)
        
        sorted_rep_points = self._sort_representative_points(all_rep_points_unsorted, proc_params, self.frame_idx, frame_width=w)

        curve_points_to_draw = self.fit_curve_to_points(
            sorted_rep_points,
            mode=fit_mode, 
            frame_height=h,
            frame_width=w,
            spline_smoothing=proc_params['spline_smoothing'],
        )

        smoothed_rmse = 0.0
        if len(sorted_rep_points) > 2 and curve_points_to_draw:
            current_rmse = self._calculate_rmse_from_line(curve_points_to_draw)
            self.rmse_history.append(current_rmse)
            smoothed_rmse = sum(self.rmse_history) / len(self.rmse_history)
            current_decision = (
                "Curve"
                if smoothed_rmse > proc_params['rmse_thresh']
                else "Straight"
            )
            self._update_lane_state_robust(current_decision)

        curve_msg = Curve()
        curve_msg.header = image_msg.header
        curve_msg.points = [Point2D(x=float(p[0]), y=float(p[1])) for p in curve_points_to_draw] if curve_points_to_draw else []
        curve_msg.state = self.lane_state
        self.curve_publisher.publish(curve_msg)
        callback_end = time.monotonic()
        callback_end_ns = (
            self.get_clock().now().nanoseconds
            if self.pipeline_timing_pub is not None else 0
        )
        if self.pipeline_timing_pub is not None:
            timing = PipelineTiming()
            timing.header = image_msg.header
            timing.stage = 'centerlane_curve'
            timing.receive_stamp_ns = int(callback_start_ns)
            timing.start_stamp_ns = int(callback_start_ns)
            timing.end_stamp_ns = int(callback_end_ns)
            timing.queue_wait_ms = 0.0
            timing.compute_ms = (
                callback_end - callback_start) * 1000.0
            timing.received_count = int(self.frame_idx)
            timing.processed_count = int(self.frame_idx)
            timing.replaced_count = 0
            timing.input_reused = False
            self.pipeline_timing_pub.publish(timing)
        self._record_performance(
            callback_start,
            image_msg,
            scene_age_s,
        )
        
        if self.debug:
            decision_text = f"State: {self.lane_state} (Smoothed RMSE: {smoothed_rmse:.2f})"
            self.visualize_results(
                frame, 
                filtered_detections,
                vehicle_detections, 
                sorted_rep_points, 
                curve_points_to_draw, 
                lines_to_draw, 
                self.params,
                decision_text,
                self.frame_idx,
                all_center_lane_detections 
            )
            for name, img in zip(self.debug_window_names, debug_images):
                if img is not None:
                    cv2.imshow(name, img)
    
    def _calculate_rmse_from_line(self, points):
        if len(points) < 2:
            return 0.0

        points_np = np.array(points)
        x_coords = points_np[:, 0]
        y_coords = points_np[:, 1]
        
        coeffs = np.polyfit(y_coords, x_coords, 1)
        m, c = coeffs
        
        x_predicted = m * y_coords + c
        
        errors = x_coords - x_predicted
        
        rmse = np.sqrt(np.mean(errors**2))
        return rmse

    def _update_lane_state_robust(self, current_decision):
        if current_decision == self.state_candidate:
            self.state_confidence_counter += 1
        else:
            self.state_candidate = current_decision
            self.state_confidence_counter = 1

        if self.state_confidence_counter >= self.params['confidence_thresh']:
            if self.lane_state != self.state_candidate:
                self.get_logger().info(f"Lane state changed from '{self.lane_state}' to '{self.state_candidate}'")
                self.lane_state = self.state_candidate

    def _extract_representative_points(self, frame, detections, params):
        all_rep_points = []
        lines_to_draw = []
        h, w, _ = frame.shape
        debug_gray, debug_canny, debug_hough = None, None, None
        if self.debug:
            debug_gray, debug_canny, debug_hough = np.zeros((h, w), dtype=np.uint8), np.zeros((h, w), dtype=np.uint8), np.zeros((h, w, 3), dtype=np.uint8)

        if not detections:
            return all_rep_points, lines_to_draw, [debug_gray, debug_canny, debug_hough]

        bottom_most_det = max(detections, key=lambda d: d.ymax) if detections else None

        for det in detections:
            x1, y1, x2, y2 = map(int, [det.xmin, det.ymin, det.xmax, det.ymax])
            y1, y2, x1, x2 = max(0, y1), min(h, y2), max(0, x1), min(w, x2)
            
            box_width = x2 - x1
            box_height = y2 - y1
            if box_width < params['min_bbox_width'] or box_height < params['min_bbox_height']:
                continue
            
            if y1 >= y2 or x1 >= x2: continue
            
            roi = frame[y1:y2, x1:x2]
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            thresh_roi = cv2.adaptiveThreshold(gray_roi, 255, params['adapt_method'], cv2.THRESH_BINARY, params['block_size'], params['c_constant'])
            edges = cv2.Canny(thresh_roi, params['canny_thresh1'], params['canny_thresh2'], apertureSize=3)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=params['hough_threshold'], minLineLength=params['hough_min_len'], maxLineGap=params['hough_max_gap'])

            if self.debug:
                debug_gray[y1:y2, x1:x2], debug_canny[y1:y2, x1:x2] = thresh_roi, edges
                hough_roi_vis = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                if lines is not None:
                    for line in lines: cv2.line(hough_roi_vis, (line[0][0], line[0][1]), (line[0][2], line[0][3]), (0, 255, 0), 2)
                debug_hough[y1:y2, x1:x2] = hough_roi_vis

            current_box_points, found_hough_points = [], False
            if lines is not None:
                diag_dx, diag_dy = (x2 - x1), (y1 - y2)
                diag_angle = np.degrees(np.arctan2(diag_dy, diag_dx))
                diag_angle = diag_angle + 180 if diag_angle < 0 else diag_angle
                diag_length = np.sqrt(diag_dx**2 + diag_dy**2)
                
                valid_lines = []
                for line in lines:
                    lx1, ly1, lx2, ly2 = line[0]
                    line_dx, line_dy = (lx2 - lx1), (ly2 - ly1)
                    line_length = np.sqrt(line_dx**2 + line_dy**2)
                    
                    if diag_length == 0 or line_length == 0: continue
                    
                    line_angle = np.degrees(np.arctan2(-line_dy, line_dx))
                    line_angle = line_angle + 180 if line_angle < 0 else line_angle
                    
                    angle_diff = min(abs(line_angle - diag_angle), 180 - abs(line_angle - diag_angle))
                    length_ratio = line_length / diag_length

                    if angle_diff < params['angle_similarity_thresh'] and length_ratio > params['length_similarity_thresh']:
                        valid_lines.append(line)
                
                if valid_lines:
                    rightmost_line, max_mid_x = None, -1
                    for line in valid_lines:
                        lx1, _, lx2, _ = line[0]
                        mid_x = (lx1 + lx2) / 2
                        if mid_x > max_mid_x:
                            max_mid_x = mid_x
                            rightmost_line = (lx1 + x1, line[0][1] + y1, lx2 + x1, line[0][3] + y1)
                    
                    if rightmost_line:
                        intersections = self._get_line_bbox_intersections(rightmost_line, (x1, y1, x2, y2))
                        if len(intersections) >= 2:
                            current_box_points.extend(sorted(intersections, key=lambda p: p[1], reverse=True)[:2])
                            lines_to_draw.append(current_box_points)
                            found_hough_points = True
                        
            if not found_hough_points:
                current_box_points.append(((x1 + x2) / 2, (y1 + y2) / 2))

            if det == bottom_most_det and len(current_box_points) > 1:
                box_height = y2 - y1
                point_height_diff = abs(current_box_points[0][1] - current_box_points[1][1])
                if box_height < params['bottom_box_height_thresh'] or point_height_diff <= params['bottom_point_height_diff_thresh']:
                    topmost_point = min(current_box_points, key=lambda p: p[1])
                    current_box_points = [topmost_point]
            
            all_rep_points.extend(current_box_points)
        return all_rep_points, lines_to_draw, [debug_gray, debug_canny, debug_hough]

    def _sort_representative_points(self, points, params, frame_idx, frame_width):
        if len(points) < 2:
            return points

        remaining_points = points.copy()
        start_point = max(remaining_points, key=lambda p: p[1])
        sorted_points = [start_point]
        remaining_points.remove(start_point)
        current_point = start_point

        debug_log = [] if self.verbose_sort_debug else None

        while remaining_points:
            distances = [(p, math.dist(current_point, p)) for p in remaining_points]
            if not distances:
                break

            min_distance = min(d for p, d in distances)
            candidates = [p for p, d in distances if d <= min_distance + params['similarity_thresh']]

            if not candidates:
                break

            # ==================== [수정] 후보점들의 선분 기울기 평균 계산 로직 ====================
            trend_slope_abs = 0.0
            if len(candidates) >= 2:
                # 후보 점들을 x좌표 기준으로 정렬
                sorted_candidates_x = sorted(candidates, key=lambda p: p[0])
                segment_slopes = []
                
                # 정렬된 점들 사이의 모든 선분 기울기를 계산
                for i in range(len(sorted_candidates_x) - 1):
                    p1 = sorted_candidates_x[i]
                    p2 = sorted_candidates_x[i+1]
                    
                    dx = p2[0] - p1[0]
                    dy = p2[1] - p1[1]
                    
                    if abs(dx) < 1e-6: # 수직선인 경우
                        segment_slopes.append(999.0)
                    else:
                        segment_slopes.append(dy / dx)
                
                # 계산된 선분 기울기들의 절대값 평균을 사용
                if segment_slopes:
                    trend_slope_abs = sum(abs(s) for s in segment_slopes) / len(segment_slopes)

            slope_threshold = params['slope_threshold']

            if trend_slope_abs > slope_threshold:
                best_next_point = max(candidates, key=lambda p: p[1])
                selection_reason = f"y-value (segment slope {trend_slope_abs:.2f} > {slope_threshold:.2f})"
            else:
                best_next_point = min(candidates, key=lambda p: math.dist(current_point, p))
                selection_reason = f"Euclidean distance (segment slope {trend_slope_abs:.2f} <= {slope_threshold:.2f})"

            if debug_log is not None and candidates:
                log_entry = {
                    'current_point': current_point,
                    'candidates': candidates,
                    'selection': best_next_point,
                    'reason': selection_reason,
                    'trend_slope': trend_slope_abs,
                    'threshold': slope_threshold
                }
                debug_log.append(log_entry)

            sorted_points.append(best_next_point)
            remaining_points.remove(best_next_point)
            remaining_points = [
                p for p in remaining_points
                if math.dist(p, best_next_point)
                > params['point_merge_distance']
            ]
            current_point = best_next_point
            
        if debug_log:
            final_indices = {tuple(map(int, p)): i + 1 for i, p in enumerate(sorted_points)}
            print("=" * 80)
            print(f"============== [Frame {frame_idx}] Sorting Decision Details ==============")
            for i, entry in enumerate(debug_log):
                print(f"\n--- Decision Step {i + 1} ---")
                
                step_current_point = entry['current_point']
                
                current_p_int = tuple(map(int, step_current_point))
                current_idx = final_indices.get(current_p_int, 'N/A')
                print(f"[*] Current Point: {current_idx} {current_p_int}")
                
                trend_slope_val = entry['trend_slope']
                threshold = entry['threshold']
                # ==================== [수정] 로그 출력 문구 변경 ====================
                print(f"    - Candidates' Avg Segment Slope: {trend_slope_val:.2f}, Threshold: {threshold:.2f}")

                print("    - Candidates:")
                sorted_candidates = sorted(entry['candidates'], key=lambda c: math.dist(step_current_point, c))
                for p in sorted_candidates:
                    p_int = tuple(map(int, p))
                    p_idx = final_indices.get(p_int, 'N/A')
                    dist = math.dist(step_current_point, p)
                    dx = p[0] - step_current_point[0]
                    dy = p[1] - step_current_point[1]
                    slope_str = f"{(dy/dx):.2f}" if dx != 0 else "inf"
                    print(f"        -> Point {p_idx} {p_int}, Distance: {dist:.2f}, Slope: {slope_str}")

                selected_p_int = tuple(map(int, entry['selection']))
                selected_idx = final_indices.get(selected_p_int, 'N/A')
                reason = entry['reason']
                print(f"    - Selection Criterion: {reason}")
                print(f"    - Final Selection: Point {selected_idx} {selected_p_int}")

            print("=" * 80)
        
        if len(sorted_points) >= 2:
            indices_to_remove = set()
            for i in range(len(sorted_points) - 1):
                p1 = sorted_points[i]
                p2 = sorted_points[i+1]

                if abs(p1[0] - p2[0]) >= params['x_dist_thresh']:
                    dist_p1_to_edge = min(p1[0], frame_width - p1[0])
                    dist_p2_to_edge = min(p2[0], frame_width - p2[0])

                    if dist_p1_to_edge < dist_p2_to_edge:
                        indices_to_remove.add(i)
                    else:
                        indices_to_remove.add(i + 1)
            
            if indices_to_remove:
                original_count = len(sorted_points)
                sorted_points = [p for i, p in enumerate(sorted_points) if i not in indices_to_remove]
                self.get_logger().debug(
                    f"Filtered {original_count - len(sorted_points)} "
                    'horizontal outlier(s).')
            
        return sorted_points

    def _get_line_bbox_intersections(self, line_coords, bbox_coords):
        lx1, ly1, lx2, ly2 = line_coords
        xmin, ymin, xmax, ymax = bbox_coords
        if lx1 == lx2: return [(lx1, ymin), (lx1, ymax)]
        m = (ly2 - ly1) / (lx2 - lx1)
        c = ly1 - m * lx1
        points = []
        for y in [ymin, ymax]:
            if m != 0:
                x = (y - c) / m
                if xmin <= x <= xmax: points.append((x, y))
        for x in [xmin, xmax]:
            y = m * x + c
            if ymin <= y <= ymax: points.append((x, y))
        unique_points = sorted(list(set(tuple(map(int, p)) for p in points)))
        return unique_points if len(unique_points) >= 2 else []

    @staticmethod
    def _ray_to_frame_boundary(origin, neighbor, frame_height, frame_width):
        """Extend an endpoint away from its neighbor to the first frame edge."""
        origin = np.asarray(origin, dtype=np.float64)
        neighbor = np.asarray(neighbor, dtype=np.float64)
        direction = origin - neighbor
        if np.linalg.norm(direction) < 1e-9:
            return None

        max_x = float(frame_width - 1)
        max_y = float(frame_height - 1)
        candidates = []
        for axis, boundaries in ((0, (0.0, max_x)), (1, (0.0, max_y))):
            if abs(direction[axis]) < 1e-9:
                continue
            for boundary in boundaries:
                distance = (boundary - origin[axis]) / direction[axis]
                if distance <= 1e-9:
                    continue
                point = origin + distance * direction
                if 0.0 <= point[0] <= max_x and 0.0 <= point[1] <= max_y:
                    candidates.append((float(distance), point))
        if not candidates:
            return None
        point = min(candidates, key=lambda candidate: candidate[0])[1]
        return float(point[0]), float(point[1])

    def _extend_path_to_boundaries(
            self, path_points, frame_height, frame_width):
        if len(path_points) < 2:
            return path_points
        max_x = float(frame_width - 1)
        max_y = float(frame_height - 1)
        bounded_path = [
            (
                float(np.clip(point[0], 0.0, max_x)),
                float(np.clip(point[1], 0.0, max_y)),
            )
            for point in path_points
        ]
        result_points = list(bounded_path)

        start_extension = self._ray_to_frame_boundary(
            bounded_path[0], bounded_path[1], frame_height, frame_width)
        if start_extension is not None:
            result_points.insert(0, start_extension)

        end_extension = self._ray_to_frame_boundary(
            bounded_path[-1], bounded_path[-2], frame_height, frame_width)
        if end_extension is not None:
            result_points.append(end_extension)
        return result_points

    def fit_curve_to_points(
            self,
            points,
            mode='spline',
            frame_height=None,
            frame_width=None,
            spline_smoothing=30.0):
        if len(points) < 2: return []
        base_path = []
        if mode == 'spline':
            try:
                points_np = np.array(points, dtype=np.float32)
                x_coords, y_coords = points_np[:, 0], points_np[:, 1]
                num_points = len(points)
                if num_points >= 3:
                    k = 3 if num_points >= 4 else 2
                    tck, u = splprep(
                        [x_coords, y_coords],
                        s=spline_smoothing,
                        k=k,
                    )
                    eval_points = np.linspace(u.min(), u.max(), 100)
                    x_new, y_new = splev(eval_points, tck)
                    base_path = list(zip(x_new, y_new))
                else: base_path = points
            except (ValueError, np.linalg.LinAlgError) as e:
                base_path = points
        elif mode == 'linear': base_path = points
        else:
            self.get_logger().warn(f"Invalid fit mode '{mode}'. Defaulting to linear.")
            base_path = points
        if not base_path: return []
        if frame_height is not None and frame_width is not None:
            return self._extend_path_to_boundaries(base_path, frame_height, frame_width)
        else: return base_path

    def visualize_results(self, frame, detections, vehicle_detections, sorted_points, curve_points, lines_to_draw, params, final_decision_text, frame_idx, all_center_lanes):
        output_frame = frame.copy()
        
        cv2.putText(output_frame, f"Frame: {frame_idx}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

        if final_decision_text:
            cv2.putText(output_frame, final_decision_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
            
        if params['draw_bbox']:
            for det in detections:
                x1, y1, x2, y2 = map(int, [det.xmin, det.ymin, det.xmax, det.ymax])
                cv2.rectangle(output_frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

            for vehicle_det in vehicle_detections:
                vx1, vy1, vx2, vy2 = map(int, [vehicle_det.xmin, vehicle_det.ymin, vehicle_det.xmax, vehicle_det.ymax])
                cv2.rectangle(output_frame, (vx1, vy1), (vx2, vy2), (255, 0, 0), 2)
                filter_zone_y = int(vy1 + (vy2 - vy1) * 0.7)
                cv2.line(output_frame, (vx1, filter_zone_y), (vx2, filter_zone_y), (0, 0, 255), 1)
        
        if params['draw_hough']:
            for line in lines_to_draw:
                if len(line) >= 2:
                    pt1, pt2 = tuple(map(int, line[0])), tuple(map(int, line[1]))
                    cv2.line(output_frame, pt1, pt2, (255, 0, 0), 2)
        for i, point in enumerate(sorted_points):
            x, y = map(int, point)
            if params['draw_points']:
                cv2.circle(output_frame, (x, y), 7, (0, 0, 255), -1)
            if params['draw_numbers']:
                cv2.putText(output_frame, str(i + 1), (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
        if params['draw_curve'] and curve_points:
            curve_pts = np.array(curve_points, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(output_frame, [curve_pts], isClosed=False, color=(255, 0, 255), thickness=2)
        cv2.imshow(self.main_window_name, output_frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.debug:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CenterlaneTracer()
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
