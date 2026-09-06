import rclpy
from rclpy.node import Node
from collections import deque
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from std_msgs.msg import Float32, String
from custom_interfaces.msg import (
    Curve,
    Detections,
    ObstacleState,
    Point2D,
    Ultrasonic,
)
import message_filters
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
import yaml
import os

from cam.dynamic_lane_event_logic import (
    DynamicLaneEventConfig,
    DynamicLaneEventMachine,
    DynamicLaneEventState,
    road_allows_obstacle_event,
    shortcut_blocks_obstacle_event,
    shortcut_protects_override_from_reset,
)
from cam.obstacle_lidar_fusion import (
    LidarSnapshot,
    ProjectedCluster,
    TemporalClusterTracker,
    TemporalClusterTrackingConfig,
    associate_by_horizontal_direction,
    camera_detection_is_near,
    cluster_ordered_scan,
    select_time_aligned_state,
)

class TargetLanePlanner(Node):
    def __init__(self):
        super().__init__('target_lane_planner')
        self.declare_parameter('image_topic', '/perception/scene/image')
        self.declare_parameter('curve_topic', '/center_curve')
        self.declare_parameter('road_curve_topic', '')
        self.declare_parameter('road_curve_max_time_delta_s', 0.25)
        self.declare_parameter(
            'detections_topic', '/scene_yolo/detections')
        self.declare_parameter('image_qos_reliability', 'reliable')
        self.declare_parameter('curve_source_width', 320)
        self.declare_parameter('curve_source_height', 240)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ultrasonic_topic', '/ultrasonic')
        self.declare_parameter('mission_mode_topic', '/mission_mode')
        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('sync_slop', 0.2)
        self.declare_parameter('sensor_timeout_s', 0.3)
        self.declare_parameter('obstacle_lidar_min_range_m', 0.18)
        self.declare_parameter('obstacle_lidar_max_range_m', 5.0)
        self.declare_parameter('obstacle_lidar_cluster_min_points', 2)
        self.declare_parameter('obstacle_lidar_cluster_base_gap_m', 0.08)
        self.declare_parameter(
            'obstacle_lidar_cluster_range_gap_scale', 1.5)
        self.declare_parameter('obstacle_lidar_cluster_max_width_m', 2.0)
        self.declare_parameter(
            'obstacle_lidar_distance_percentile', 50.0)
        self.declare_parameter(
            'obstacle_lidar_association_padding_px', 35.0)
        self.declare_parameter(
            'obstacle_lidar_association_max_above_px', 10.0)
        self.declare_parameter(
            'obstacle_lidar_camera_max_delta_s', 0.15)
        self.declare_parameter('obstacle_lidar_scan_history_size', 5)
        self.declare_parameter('obstacle_lidar_tracking_enabled', True)
        self.declare_parameter('obstacle_lidar_track_max_age_s', 0.60)
        self.declare_parameter('obstacle_lidar_track_max_misses', 3)
        self.declare_parameter(
            'obstacle_lidar_track_max_distance_jump_m', 0.45)
        self.declare_parameter(
            'obstacle_lidar_track_max_range_rate_mps', 3.0)
        self.declare_parameter(
            'obstacle_lidar_track_max_total_distance_jump_m', 1.50)
        self.declare_parameter(
            'obstacle_lidar_track_max_offset_jump_px', 55.0)
        self.declare_parameter(
            'obstacle_lidar_track_max_offset_rate_px_s', 120.0)
        self.declare_parameter(
            'obstacle_lidar_track_bbox_iou_min', 0.10)
        self.declare_parameter(
            'obstacle_lidar_track_bbox_center_gate_px', 150.0)
        self.declare_parameter(
            'obstacle_lidar_near_cluster_distance_m', 0.0)
        self.declare_parameter(
            'obstacle_lidar_near_cluster_min_bbox_height_ratio', 0.16)
        self.declare_parameter(
            'obstacle_lidar_near_cluster_min_bbox_bottom_ratio', 0.56)
        self.declare_parameter(
            'obstacle_camera_fallback_speed_enabled', True)
        self.declare_parameter(
            'obstacle_camera_fallback_min_bottom_ratio', 0.48)
        self.declare_parameter('obstacle_allow_avoid_on_curve', True)
        self.declare_parameter('show_visualization', False)
        self.declare_parameter('dynamic_event_duration_s', 5.0)
        self.declare_parameter('dynamic_speed_confirm_min_matches', 2)
        self.declare_parameter('dynamic_speed_confirm_window_frames', 3)
        self.declare_parameter('dynamic_avoid_confirm_min_matches', 2)
        self.declare_parameter('dynamic_avoid_confirm_window_frames', 3)
        self.declare_parameter('dynamic_rearm_clear_frames', 5)
        self.declare_parameter('dynamic_obstacle_speed', 7.0)
        self.declare_parameter('dynamic_speed_trigger_distance_m', 10.0)
        self.declare_parameter('dynamic_overtake_trigger_distance_m', 4.5)
        self.declare_parameter(
            'dynamic_lane_determination_distance_m', 5.0)
        self.declare_parameter('static_event_duration_s', 4.0)
        self.declare_parameter('static_speed_confirm_min_matches', 2)
        self.declare_parameter('static_speed_confirm_window_frames', 3)
        self.declare_parameter('static_avoid_confirm_min_matches', 2)
        self.declare_parameter('static_avoid_confirm_window_frames', 3)
        self.declare_parameter('static_rearm_clear_frames', 5)
        self.declare_parameter('static_obstacle_speed', 7.0)
        self.declare_parameter('static_speed_trigger_distance_m', 10.0)
        self.declare_parameter('static_overtake_trigger_distance_m', 4.5)
        self.declare_parameter('static_lane_determination_distance_m', 5.0)
        self.declare_parameter(
            'obstacle_speed_limit_topic', '/obstacle_speed_limit')
        self.declare_parameter(
            'shortcut_state_topic', '/shortcut_left_turn/state')

        pkg_share = get_package_share_directory('cam')
        intrinsic_file = os.path.join(pkg_share, 'config', 'fisheye_calib.yaml')
        extrinsic_file = os.path.join(pkg_share, 'config', 'extrinsic.yaml')
        with open(intrinsic_file, 'r') as f: intrinsic_calib = yaml.safe_load(f)
        self.img_size = (intrinsic_calib['image_width'], intrinsic_calib['image_height'])
        self.mtx = np.array(intrinsic_calib['K'])
        with open(extrinsic_file, 'r') as f: extrinsic_calib = yaml.safe_load(f)
        self.R, self.t = np.array(extrinsic_calib['R']), np.array(extrinsic_calib['t']).reshape((3, 1))
        self.bridge = CvBridge()

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
        latest_image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=image_reliability,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.image_subscription = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter('image_topic').value,
            qos_profile=latest_image_qos,
        )
        self.curve_subscription = message_filters.Subscriber(
            self,
            Curve,
            self.get_parameter('curve_topic').value,
            qos_profile=reliable_qos,
        )
        self.road_curve_topic = str(
            self.get_parameter('road_curve_topic').value).strip()
        self.road_curve_history = deque(maxlen=20)
        self.road_curve_max_time_delta_s = max(
            0.0,
            float(self.get_parameter(
                'road_curve_max_time_delta_s').value),
        )
        self.road_curve_subscription = None
        if self.road_curve_topic:
            self.road_curve_subscription = self.create_subscription(
                Curve,
                self.road_curve_topic,
                self.road_curve_callback,
                reliable_qos,
            )
        self.yolo_subscription = message_filters.Subscriber(
            self,
            Detections,
            self.get_parameter('detections_topic').value,
            qos_profile=reliable_qos,
        )
        self.curve_source_width = max(
            1, int(self.get_parameter('curve_source_width').value))
        self.curve_source_height = max(
            1, int(self.get_parameter('curve_source_height').value))
        self.latest_scan_msg = None
        self.latest_ultra_msg = None
        self.last_scan_received_ns = 0
        self.last_ultra_received_ns = 0
        self.sensor_timeout_s = max(
            0.05,
            float(self.get_parameter('sensor_timeout_s').value),
        )
        self.obstacle_lidar_min_range_m = max(
            0.0,
            float(self.get_parameter(
                'obstacle_lidar_min_range_m').value),
        )
        self.obstacle_lidar_max_range_m = max(
            self.obstacle_lidar_min_range_m,
            float(self.get_parameter(
                'obstacle_lidar_max_range_m').value),
        )
        self.obstacle_lidar_cluster_min_points = max(
            1,
            int(self.get_parameter(
                'obstacle_lidar_cluster_min_points').value),
        )
        self.obstacle_lidar_cluster_base_gap_m = max(
            0.0,
            float(self.get_parameter(
                'obstacle_lidar_cluster_base_gap_m').value),
        )
        self.obstacle_lidar_cluster_range_gap_scale = max(
            0.0,
            float(self.get_parameter(
                'obstacle_lidar_cluster_range_gap_scale').value),
        )
        self.obstacle_lidar_cluster_max_width_m = max(
            0.0,
            float(self.get_parameter(
                'obstacle_lidar_cluster_max_width_m').value),
        )
        self.obstacle_lidar_distance_percentile = float(np.clip(
            self.get_parameter(
                'obstacle_lidar_distance_percentile').value,
            0.0,
            100.0,
        ))
        self.obstacle_lidar_association_padding_px = max(
            0.0,
            float(self.get_parameter(
                'obstacle_lidar_association_padding_px').value),
        )
        self.obstacle_lidar_association_max_above_px = max(
            0.0,
            float(self.get_parameter(
                'obstacle_lidar_association_max_above_px').value),
        )
        self.obstacle_lidar_camera_max_delta_s = max(
            0.0,
            float(self.get_parameter(
                'obstacle_lidar_camera_max_delta_s').value),
        )
        self.obstacle_lidar_tracking_enabled = bool(
            self.get_parameter('obstacle_lidar_tracking_enabled').value)
        self.obstacle_cluster_tracker = TemporalClusterTracker(
            TemporalClusterTrackingConfig(
                max_age_s=max(0.0, float(self.get_parameter(
                    'obstacle_lidar_track_max_age_s').value)),
                max_misses=max(1, int(self.get_parameter(
                    'obstacle_lidar_track_max_misses').value)),
                max_distance_jump_m=max(0.0, float(self.get_parameter(
                    'obstacle_lidar_track_max_distance_jump_m').value)),
                max_range_rate_mps=max(0.0, float(self.get_parameter(
                    'obstacle_lidar_track_max_range_rate_mps').value)),
                max_total_distance_jump_m=max(0.0, float(self.get_parameter(
                    'obstacle_lidar_track_max_total_distance_jump_m').value)),
                max_offset_jump_px=max(0.0, float(self.get_parameter(
                    'obstacle_lidar_track_max_offset_jump_px').value)),
                max_offset_rate_px_s=max(0.0, float(self.get_parameter(
                    'obstacle_lidar_track_max_offset_rate_px_s').value)),
                bbox_iou_min=float(np.clip(self.get_parameter(
                    'obstacle_lidar_track_bbox_iou_min').value, 0.0, 1.0)),
                bbox_center_gate_px=max(0.0, float(self.get_parameter(
                    'obstacle_lidar_track_bbox_center_gate_px').value)),
                near_cluster_distance_m=max(0.0, float(self.get_parameter(
                    'obstacle_lidar_near_cluster_distance_m').value)),
                near_cluster_min_bbox_height_ratio=float(np.clip(
                    self.get_parameter(
                        'obstacle_lidar_near_cluster_min_bbox_height_ratio'
                    ).value,
                    0.0,
                    1.0,
                )),
                near_cluster_min_bbox_bottom_ratio=float(np.clip(
                    self.get_parameter(
                        'obstacle_lidar_near_cluster_min_bbox_bottom_ratio'
                    ).value,
                    0.0,
                    1.0,
                )),
            )
        )
        self.obstacle_camera_fallback_speed_enabled = bool(
            self.get_parameter(
                'obstacle_camera_fallback_speed_enabled').value)
        self.obstacle_camera_fallback_min_bottom_ratio = float(np.clip(
            self.get_parameter(
                'obstacle_camera_fallback_min_bottom_ratio').value,
            0.0,
            1.0,
        ))
        self.obstacle_allow_avoid_on_curve = bool(
            self.get_parameter('obstacle_allow_avoid_on_curve').value)
        lidar_history_size = max(
            1,
            int(self.get_parameter(
                'obstacle_lidar_scan_history_size').value),
        )
        self.lidar_snapshots = deque(maxlen=lidar_history_size)
        self.last_fast_path_status = ''
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        latest_sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.ultra_subscription = self.create_subscription(
            Ultrasonic,
            self.get_parameter('ultrasonic_topic').value,
            self.ultrasonic_callback,
            latest_sensor_qos,
        )

        self.obstacle_state_pub = self.create_publisher(ObstacleState, '/obstacle_state', 10)
        self.override_pub = self.create_publisher(String, '/lane_override_cmd', 10)
        self.overtake_state_pub = self.create_publisher(String, '/overtake_state', 10)
        self.obstacle_speed_limit_pub = self.create_publisher(
            Float32,
            self.get_parameter('obstacle_speed_limit_topic').value,
            10,
        )
        self.mission_mode = 'LANE'
        mode_qos = QoSProfile(depth=1)
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mission_mode_sub = self.create_subscription(
            String,
            self.get_parameter('mission_mode_topic').value,
            self.mission_mode_callback,
            mode_qos,
        )
        self.shortcut_state = 'IDLE'
        self.shortcut_state_sub = self.create_subscription(
            String,
            self.get_parameter('shortcut_state_topic').value,
            self.shortcut_state_callback,
            10,
        )
        
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_subscription, self.curve_subscription, self.yolo_subscription],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop').value))
        self.time_synchronizer.registerCallback(self.synchronized_callback)
        self.show_visualization = bool(self.get_parameter('show_visualization').value)

        self.ULTAR_THRESHOLD = 30
        self.ULTAR_BACK_THRESHOLD = 40
        self.CURVE_OVERTAKE_THRESHOLD_M = 1.0
        self.MULTI_VEHICLE_OVERTAKE_GAP_M = 0.8
        self.driving_state = "CENTER_DRIVING"  # "CENTER_DRIVING" or "OVERTAKING"
        self.last_override_cmd = 'reset'

        self.is_passing_obstacle = False    # 장애물 옆을 통과하고 있는지 여부
        self.pass_complete_timer = None     # 장애물을 지나친 후 안전거리 확보를 위한 타이머
        self.PASS_TIMER_DURATION_S = 0.1  # 안전거리 확보 시간 (0.1초)

        self.E_STOP_DISTANCE_M = 0.5  # 급정지를 발동할 거리 (m)
        self.E_STOP_X_MIN = 50     # 급정지를 감지할 전방 카메라 x좌표 (최소)
        self.E_STOP_X_MAX = 600   # 급정지를 감지할 전방 카메라 x좌표 (최대)
        # ========================= 무조건 안 박음 =============================
        self.E_STOP_RECT_X_MIN = 200
        self.E_STOP_RECT_X_MAX = 400
        self.E_STOP_RECT_Y_MIN = 260
        self.E_STOP_RECT_Y_MAX = 400
        # # ========================= E-Stop 범위 작게 ==========================
        # self.E_STOP_RECT_X_MIN = 280
        # self.E_STOP_RECT_X_MAX = 360
        # self.E_STOP_RECT_Y_MIN = 260
        # self.E_STOP_RECT_Y_MAX = 400

        self.E_STOP_RECT_PIXEL_COUNT_THRESHOLD = 3

        self.PROXIMITY_THRESHOLD_M = 0.6
        self.smoothed_obstacle_position = 0.0  # -1 (left) ~ +1 (right) 사이의 값을 가짐
        self.EMA_ALPHA = 0.3  # 스무딩 강도 (0.1: 매우 부드러움, 0.9: 매우 민감)
        self.last_closest_obstacle_lane = "unknown" # 가장 가까운 장애물의 마지막 차선 위치 저장

        # 차선 판단 로직 임계값 변수
        self.AREA_CLIP_THRESHOLD_PERCENT = 13.0
        self.dynamic_speed_trigger_distance_m = max(0.0, float(
            self.get_parameter('dynamic_speed_trigger_distance_m').value))
        self.dynamic_overtake_trigger_distance_m = max(0.0, float(
            self.get_parameter('dynamic_overtake_trigger_distance_m').value))
        self.dynamic_lane_determination_distance_m = max(0.0, float(
            self.get_parameter(
                'dynamic_lane_determination_distance_m').value))
        self.static_speed_trigger_distance_m = max(0.0, float(
            self.get_parameter('static_speed_trigger_distance_m').value))
        self.static_overtake_trigger_distance_m = max(0.0, float(
            self.get_parameter('static_overtake_trigger_distance_m').value))
        self.static_lane_determination_distance_m = max(0.0, float(
            self.get_parameter(
                'static_lane_determination_distance_m').value))
        self.dynamic_event_machine = DynamicLaneEventMachine(
            DynamicLaneEventConfig(
                duration_s=max(0.0, float(self.get_parameter(
                    'dynamic_event_duration_s').value)),
                speed_confirm_min_matches=max(1, int(self.get_parameter(
                    'dynamic_speed_confirm_min_matches').value)),
                speed_confirm_window_frames=max(1, int(self.get_parameter(
                    'dynamic_speed_confirm_window_frames').value)),
                avoid_confirm_min_matches=max(1, int(self.get_parameter(
                    'dynamic_avoid_confirm_min_matches').value)),
                avoid_confirm_window_frames=max(1, int(self.get_parameter(
                    'dynamic_avoid_confirm_window_frames').value)),
                rearm_clear_frames=max(1, int(self.get_parameter(
                    'dynamic_rearm_clear_frames').value)),
                speed=max(0.0, float(self.get_parameter(
                    'dynamic_obstacle_speed').value)),
                static_duration_s=max(0.0, float(self.get_parameter(
                    'static_event_duration_s').value)),
                static_speed_confirm_min_matches=max(1, int(
                    self.get_parameter(
                        'static_speed_confirm_min_matches').value)),
                static_speed_confirm_window_frames=max(1, int(
                    self.get_parameter(
                        'static_speed_confirm_window_frames').value)),
                static_avoid_confirm_min_matches=max(1, int(
                    self.get_parameter(
                        'static_avoid_confirm_min_matches').value)),
                static_avoid_confirm_window_frames=max(1, int(
                    self.get_parameter(
                        'static_avoid_confirm_window_frames').value)),
                static_rearm_clear_frames=max(1, int(self.get_parameter(
                    'static_rearm_clear_frames').value)),
                static_speed=max(0.0, float(self.get_parameter(
                    'static_obstacle_speed').value)),
            )
        )
        self.dynamic_event_timer = self.create_timer(
            0.05, self.dynamic_event_timer_callback)

        self.get_logger().info('🎯 Target Lane Planner has been started!')
        self.get_logger().info(
            'Planner perception coordinates: '
            f'image_qos={reliability_name}/depth1, '
            f'curve_source={self.curve_source_width}x'
            f'{self.curve_source_height}, '
            f'road_curve={self.road_curve_topic or "routed fallback"}, '
            'lidar_fusion=ordered_clusters+temporal_camera_direction'
        )

    @staticmethod
    def stamp_to_ns(stamp):
        """Convert a ROS stamp into integer nanoseconds."""
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def road_curve_callback(self, msg):
        """Cache the physical-road state separately from the routed path."""
        source_ns = self.stamp_to_ns(msg.header.stamp)
        received_ns = self.get_clock().now().nanoseconds
        self.road_curve_history.append((
            source_ns,
            received_ns,
            str(msg.state),
        ))

    def road_curve_state_for(self, image_msg, routed_state):
        """Return the source-time-nearest physical-road classification."""
        target_ns = self.stamp_to_ns(image_msg.header.stamp)
        return select_time_aligned_state(
            self.road_curve_history,
            target_ns,
            self.road_curve_max_time_delta_s,
            routed_state,
        )

    def scan_callback(self, msg):
        """Cluster one ordered scan and retain a short timestamped history."""
        received_ns = self.get_clock().now().nanoseconds
        min_range_m = max(
            self.obstacle_lidar_min_range_m,
            float(msg.range_min),
        )
        max_range_m = self.obstacle_lidar_max_range_m
        if float(msg.range_max) > 0.0:
            max_range_m = min(max_range_m, float(msg.range_max))
        clusters = cluster_ordered_scan(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            min_range_m=min_range_m,
            max_range_m=max_range_m,
            min_points=self.obstacle_lidar_cluster_min_points,
            base_gap_m=self.obstacle_lidar_cluster_base_gap_m,
            range_gap_scale=(
                self.obstacle_lidar_cluster_range_gap_scale),
            max_width_m=self.obstacle_lidar_cluster_max_width_m,
            distance_percentile=self.obstacle_lidar_distance_percentile,
        )
        source_ns = self.stamp_to_ns(msg.header.stamp)
        self.lidar_snapshots.append(LidarSnapshot(
            source_ns=source_ns,
            received_ns=received_ns,
            clusters=tuple(clusters),
        ))
        self.latest_scan_msg = msg
        self.last_scan_received_ns = received_ns

    def select_lidar_snapshot(self, image_msg, now_ns):
        """Select the fresh scan nearest to the camera source timestamp."""
        fresh = [
            snapshot for snapshot in self.lidar_snapshots
            if self.sensor_is_fresh(snapshot.received_ns, now_ns)
        ]
        if not fresh:
            return None
        target_ns = self.stamp_to_ns(image_msg.header.stamp)
        if target_ns <= 0:
            return fresh[-1]
        timestamped = [item for item in fresh if item.source_ns > 0]
        if not timestamped:
            return fresh[-1]
        nearest = min(
            timestamped,
            key=lambda item: abs(item.source_ns - target_ns),
        )
        delta_s = abs(nearest.source_ns - target_ns) * 1e-9
        if delta_s > self.obstacle_lidar_camera_max_delta_s:
            return None
        return nearest

    def project_lidar_clusters(self, clusters, image_width, image_height):
        """Project cluster summaries once for camera-direction matching."""
        projected = []
        scale_x = float(image_width) / float(self.img_size[0])
        scale_y = float(image_height) / float(self.img_size[1])
        for cluster_index, cluster in enumerate(clusters):
            zeros = np.zeros((cluster.points_xy.shape[0], 1))
            points_3d = np.concatenate((cluster.points_xy, zeros), axis=1)
            cam_points = (self.R @ points_3d.T + self.t).T
            # Keep the signed optical depth used by the existing vehicle
            # calibration. The installed extrinsic maps part of the forward
            # LiDAR sector to negative Z before pinhole normalization.
            visible = np.isfinite(cam_points).all(axis=1) & (
                np.abs(cam_points[:, 2]) > 0.05)
            if not np.any(visible):
                continue
            visible_cam_points = cam_points[visible]
            image_points_raw, _ = cv2.projectPoints(
                visible_cam_points,
                np.zeros(3),
                np.zeros(3),
                self.mtx,
                None,
            )
            image_points = image_points_raw.squeeze(axis=1)
            image_points[:, 0] *= scale_x
            image_points[:, 1] *= scale_y
            image_points = image_points[
                np.isfinite(image_points).all(axis=1)]
            if image_points.shape[0] == 0:
                continue
            x_values = image_points[:, 0]
            y_values = image_points[:, 1]
            projected.append(ProjectedCluster(
                cluster_index=cluster_index,
                image_points=image_points,
                center_x=float(np.median(x_values)),
                left_x=float(np.min(x_values)),
                right_x=float(np.max(x_values)),
                center_y=float(np.median(y_values)),
                top_y=float(np.min(y_values)),
                bottom_y=float(np.max(y_values)),
                distance_m=float(cluster.distance_m),
                point_count=int(cluster.points_xy.shape[0]),
                width_m=float(cluster.width_m),
            ))
        return projected

    def ultrasonic_callback(self, msg):
        self.latest_ultra_msg = msg
        self.last_ultra_received_ns = self.get_clock().now().nanoseconds

    def shortcut_state_callback(self, msg):
        fields = dict(
            item.split('=', 1)
            for item in str(msg.data).split()
            if '=' in item
        )
        self.shortcut_state = fields.get('state', 'IDLE').upper()

    def dynamic_event_blocked(self):
        return (
            self.mission_mode in ('CONE', 'STOP')
            or shortcut_blocks_obstacle_event(self.shortcut_state)
        )

    def profile_distance(self, event_kind, field):
        """Return one kind-specific approach, avoid, or lane distance."""
        kind = self.dynamic_event_machine.normalize_obstacle_kind(event_kind)
        prefix = 'static' if kind == 'static' else 'dynamic'
        return float(getattr(self, f'{prefix}_{field}_distance_m'))

    def publish_obstacle_speed_limit(self, decision):
        """Publish -1 when inactive or the latched profile speed cap."""
        self.obstacle_speed_limit_pub.publish(
            Float32(data=float(decision.speed_limit)))

    def set_dynamic_event_state(self, decision):
        previous = self.driving_state
        kind = str(decision.event_kind).upper()
        if decision.state == DynamicLaneEventState.APPROACH:
            self.driving_state = f'{kind}_APPROACH'
        elif decision.state == DynamicLaneEventState.ACTIVE:
            self.driving_state = f'{kind}_AVOIDING'
        elif decision.state == DynamicLaneEventState.WAIT_CLEAR:
            self.driving_state = f'{kind}_WAIT_CLEAR'
        else:
            self.driving_state = 'CENTER_DRIVING'
        if self.driving_state != previous:
            self.get_logger().warn(
                f'Obstacle lane event: {previous} -> {self.driving_state}; '
                f'kind={decision.event_kind}, '
                f'obstacle={decision.obstacle_lane}, '
                f'target={decision.target_lane}, reason={decision.reason}'
            )

    def publish_override_if_changed(self, command):
        if command == self.last_override_cmd:
            return
        if (
            command == 'reset'
            and shortcut_protects_override_from_reset(self.shortcut_state)
        ):
            self.last_override_cmd = 'reset'
            return
        self.override_pub.publish(String(data=str(command)))
        self.last_override_cmd = str(command)
        self.get_logger().warn(
            f"Dynamic lane command sent: '{self.last_override_cmd}'")

    def dynamic_event_timer_callback(self):
        """Advance active timing and keep the profile speed cap fresh."""
        if self.dynamic_event_machine.state == DynamicLaneEventState.IDLE:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.dynamic_event_blocked():
            decision = self.dynamic_event_machine.update(
                now_s,
                obstacle_kind=self.dynamic_event_machine.event_kind,
                object_present=True,
                blocked=True,
            )
        elif self.dynamic_event_machine.state == DynamicLaneEventState.ACTIVE:
            decision = self.dynamic_event_machine.update(
                now_s,
                obstacle_lane=self.dynamic_event_machine.obstacle_lane,
                obstacle_kind=self.dynamic_event_machine.event_kind,
                object_present=True,
                speed_trigger_allowed=False,
                trigger_allowed=False,
                blocked=self.dynamic_event_blocked(),
            )
        else:
            decision = self.dynamic_event_machine.decision(now_s)
        self.set_dynamic_event_state(decision)
        self.publish_override_if_changed(decision.command)
        self.publish_obstacle_speed_limit(decision)
        self.overtake_state_pub.publish(String(data=self.driving_state.upper()))

    def sensor_is_fresh(self, received_ns, now_ns):
        if received_ns <= 0:
            return False
        return (now_ns - received_ns) * 1e-9 <= self.sensor_timeout_s

    def scale_curve_to_image(self, curve_msg, image_width, image_height):
        """Copy a lane-resolution curve into the scene image coordinates."""
        if (
            self.curve_source_width == image_width
            and self.curve_source_height == image_height
        ):
            return curve_msg
        scale_x = float(image_width) / float(self.curve_source_width)
        scale_y = float(image_height) / float(self.curve_source_height)
        output = Curve()
        output.header = curve_msg.header
        output.state = curve_msg.state
        output.points = [
            Point2D(x=float(point.x) * scale_x, y=float(point.y) * scale_y)
            for point in curve_msg.points
        ]
        return output

    def publish_tracking_reset(self, vehicle_count=0, sensor_stale=False):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        decision = self.dynamic_event_machine.update(
            now_s,
            obstacle_lane='unknown',
            obstacle_kind=self.dynamic_event_machine.event_kind,
            object_present=vehicle_count > 0,
            trigger_allowed=False,
            blocked=sensor_stale or self.dynamic_event_blocked(),
        )
        self.set_dynamic_event_state(decision)
        self.is_passing_obstacle = False
        self.pass_complete_timer = None
        self.smoothed_obstacle_position = 0.0
        self.last_closest_obstacle_lane = 'unknown'
        self.publish_override_if_changed(decision.command)
        self.publish_obstacle_speed_limit(decision)

        obstacle_msg = ObstacleState()
        obstacle_msg.distance_m = 0.0 if sensor_stale else 999.0
        obstacle_msg.position = 'sensor_timeout' if sensor_stale else 'none'
        obstacle_msg.vehicle_count = int(vehicle_count)
        self.obstacle_state_pub.publish(obstacle_msg)
        state = 'SENSOR_STALE' if sensor_stale else self.driving_state.upper()
        self.overtake_state_pub.publish(String(data=state))

        next_status = 'sensor_stale' if sensor_stale else 'no_vehicle'
        if sensor_stale and self.last_fast_path_status != next_status:
            self.get_logger().error(
                'Vehicle detected, but LiDAR data is stale. '
                'Cancelling overtake and requesting a stop.'
            )
        self.last_fast_path_status = next_status

    def mission_mode_callback(self, msg):
        new_mode = msg.data.strip().upper()
        if new_mode == self.mission_mode:
            return
        self.mission_mode = new_mode
        if new_mode in ('CONE', 'STOP'):
            self.dynamic_event_machine.reset()
            self.driving_state = 'CENTER_DRIVING'
            self.is_passing_obstacle = False
            self.pass_complete_timer = None
            self.last_override_cmd = 'reset'
            self.override_pub.publish(String(data='reset'))
            self.publish_obstacle_speed_limit(
                self.dynamic_event_machine.decision(
                    self.get_clock().now().nanoseconds * 1e-9))
            self.overtake_state_pub.publish(
                String(data=f'PAUSED_{new_mode}'))
            clear_msg = ObstacleState()
            clear_msg.distance_m = 999.0
            clear_msg.position = 'none'
            clear_msg.vehicle_count = 0
            self.obstacle_state_pub.publish(clear_msg)

    def _find_intersections(self, curve_points, xmin, ymin, xmax, ymax):
        """커브(폴리라인)와 바운딩 박스의 교차점을 찾는 헬퍼 함수"""
        intersection_points = []
        box_segments = [
            ((xmin, ymin), (xmax, ymin)),  # Top
            ((xmin, ymax), (xmax, ymax)),  # Bottom
            ((xmin, ymin), (xmin, ymax)),  # Left
            ((xmax, ymin), (xmax, ymax))   # Right
        ]

        for i in range(len(curve_points) - 1):
            p1 = curve_points[i]
            p2 = curve_points[i+1]
            x1, y1 = p1
            x2, y2 = p2

            for seg in box_segments:
                p3, p4 = seg
                x3, y3 = p3
                x4, y4 = p4

                den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
                if den == 0:
                    continue

                t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
                u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den

                if 0 <= t <= 1 and 0 <= u <= 1:
                    ix = x1 + t * (x2 - x1)
                    iy = y1 + t * (y2 - y1)
                    intersection_points.append(np.array([ix, iy]))
        
        # 중복 제거 및 y좌표 기준 정렬
        if not intersection_points:
            return []
            
        unique_points = np.unique(np.array(intersection_points).round(decimals=2), axis=0)
        sorted_points = sorted(unique_points, key=lambda p: p[1])
        return sorted_points

    def project_lidar_on_image(
        self,
        image,
        detections_msg,
        projected_clusters,
        lidar_source_ns,
        curve_msg,
        road_curve_state,
        left_ultra,
        right_ultra,
        left_back_ultra,
        right_back_ultra,
    ):
        # 1. 모든 장애물 정보 수집 (기존과 동일)
        processed_obstacles = []
        detections = [
            det for det in detections_msg.detections
            if det.class_name in ('dynamic', 'obstacle_vehicle', 'static')
        ]

        h, w, _ = image.shape
        has_curve = curve_msg and len(curve_msg.points) >= 2

        detection_boxes = [
            (det.xmin, det.ymin, det.xmax, det.ymax)
            for det in detections
        ]
        if self.obstacle_lidar_tracking_enabled:
            associations = self.obstacle_cluster_tracker.associate(
                detection_boxes,
                projected_clusters,
                lidar_source_ns,
                padding_px=self.obstacle_lidar_association_padding_px,
                max_above_bbox_px=(
                    self.obstacle_lidar_association_max_above_px),
                image_height=h,
            )
        else:
            associations = associate_by_horizontal_direction(
                detection_boxes,
                projected_clusters,
                padding_px=self.obstacle_lidar_association_padding_px,
                max_above_bbox_px=(
                    self.obstacle_lidar_association_max_above_px),
            )
        cluster_by_detection = {
            association.detection_index: projected_clusters[
                association.cluster_index]
            for association in associations
        }
        camera_fallback_detection = None
        if self.obstacle_camera_fallback_speed_enabled:
            near_detections = [
                det for det in detections
                if camera_detection_is_near(
                    (det.xmin, det.ymin, det.xmax, det.ymax),
                    h,
                    self.obstacle_camera_fallback_min_bottom_ratio,
                )
            ]
            if near_detections:
                camera_fallback_detection = max(
                    near_detections,
                    key=lambda det: (
                        int(det.ymax),
                        (int(det.xmax) - int(det.xmin))
                        * (int(det.ymax) - int(det.ymin)),
                    ),
                )

        if has_curve:
            raw_curve_points = np.array([[p.x, p.y] for p in curve_msg.points])
            sorted_indices = np.argsort(raw_curve_points[:, 1])
            curve_points = raw_curve_points[sorted_indices]
            curve_x_coords = curve_points[:, 0]
            curve_y_coords = curve_points[:, 1]

        for detection_index, det in enumerate(detections):
            xmin, ymin, xmax, ymax = map(int, [det.xmin, det.ymin, det.xmax, det.ymax])
            event_kind = self.dynamic_event_machine.normalize_obstacle_kind(
                det.class_name)
            lane_determination_distance_m = self.profile_distance(
                event_kind, 'lane_determination')
            avg_distance = -1.0
            points_in_path_count = 0
            area_display_str = "N/A"

            # LiDAR objects are formed before camera processing. YOLO confirms
            # each object by calibrated horizontal direction, not by requiring
            # three projected returns to land inside the full 2-D bbox.
            matched_cluster = cluster_by_detection.get(detection_index)
            cluster_image_points = np.empty((0, 2), dtype=np.float64)
            if matched_cluster is not None:
                avg_distance = matched_cluster.distance_m
                cluster_image_points = matched_cluster.image_points
                points_in_path_count = int(np.count_nonzero(
                    (cluster_image_points[:, 0] > self.E_STOP_X_MIN)
                    & (cluster_image_points[:, 0] < self.E_STOP_X_MAX)
                ))
                for point in cluster_image_points:
                    cv2.circle(
                        image,
                        (int(point[0]), int(point[1])),
                        3,
                        (0, 0, 255),
                        -1,
                    )

            # ================================ 장애물 차량 State 판별 (수정된 로직) ================================
            position_str_raw = "unknown"
            if (
                0 < avg_distance <= lane_determination_distance_m
                and has_curve
                and len(curve_points) >= 2
            ):
                intersections = self._find_intersections(curve_points, xmin, ymin, xmax, ymax)

                if len(intersections) >= 1:
                    p_entry = intersections[0]
                    p_exit = intersections[-1]
                    epsilon = 1.0 
                    box_width = xmax - xmin
                    box_height = ymax - ymin
                    is_wide_bbox = box_width > box_height
                    is_side_penetration = False
                    for p in [p_entry, p_exit]:
                        if abs(p[0] - xmin) < epsilon or abs(p[0] - xmax) < epsilon:
                            is_side_penetration = True
                            break
                    is_top_bottom_only = not is_side_penetration
                    penetrates_bottom_edge = (abs(p_entry[1] - ymax) < epsilon) or (abs(p_exit[1] - ymax) < epsilon)
                    is_significant_penetration = True
                    if box_width > 1 and box_height > 1:
                        total_area = box_width * box_height
                        calculated_area = -1
                        entry_on = [abs(p_entry[1] - ymin) < epsilon, abs(p_entry[1] - ymax) < epsilon, abs(p_entry[0] - xmin) < epsilon, abs(p_entry[0] - xmax) < epsilon]
                        exit_on = [abs(p_exit[1] - ymin) < epsilon, abs(p_exit[1] - ymax) < epsilon, abs(p_exit[0] - xmin) < epsilon, abs(p_exit[0] - xmax) < epsilon]
                        if (entry_on[0] and exit_on[2]) or (entry_on[2] and exit_on[0]):
                            p_top = p_entry if entry_on[0] else p_exit
                            p_left = p_exit if entry_on[0] else p_entry
                            calculated_area = 0.5 * abs(p_top[0] - xmin) * abs(p_left[1] - ymin)
                        elif (entry_on[0] and exit_on[3]) or (entry_on[3] and exit_on[0]):
                            p_top = p_entry if entry_on[0] else p_exit
                            p_right = p_exit if entry_on[0] else p_entry
                            calculated_area = 0.5 * abs(p_top[0] - xmax) * abs(p_right[1] - ymin)
                        elif (entry_on[1] and exit_on[2]) or (entry_on[2] and exit_on[1]):
                            p_bot = p_entry if entry_on[1] else p_exit
                            p_left = p_exit if entry_on[1] else p_entry
                            calculated_area = 0.5 * abs(p_bot[0] - xmin) * abs(p_left[1] - ymax)
                        elif (entry_on[1] and exit_on[3]) or (entry_on[3] and exit_on[1]):
                            p_bot = p_entry if entry_on[1] else p_exit
                            p_right = p_exit if entry_on[1] else p_entry
                            calculated_area = 0.5 * abs(p_bot[0] - xmax) * abs(p_right[1] - ymax)
                        elif (entry_on[2] and exit_on[3]) or (entry_on[3] and exit_on[2]): 
                            trapezoid_area = 0.5 * ((ymax - p_entry[1]) + (ymax - p_exit[1])) * box_width
                            calculated_area = min(trapezoid_area, total_area - trapezoid_area)
                        elif (entry_on[0] and exit_on[1]) or (entry_on[1] and exit_on[0]):
                            trapezoid_area = 0.5 * ((xmax - p_entry[0]) + (xmax - p_exit[0])) * box_height
                            calculated_area = min(trapezoid_area, total_area - trapezoid_area)
                        if calculated_area >= 0:
                            area_percent = (calculated_area / total_area) * 100
                            area_display_str = f"{area_percent:.1f}%"
                            if area_percent < self.AREA_CLIP_THRESHOLD_PERCENT: is_significant_penetration = False
                    dx = p_exit[0] - p_entry[0]
                    dy = p_exit[1] - p_entry[1]
                    penetrating_slope = dy / dx if abs(dx) > 1e-6 else float('inf')
                    is_vertical_line = penetrating_slope == float('inf')
                    use_slope_logic = not is_top_bottom_only and is_significant_penetration and is_wide_bbox and not is_vertical_line and not penetrates_bottom_edge
                    
                    # ========================= [수정된 부분 시작] ========================
                    # use_slope_logic이 True여도, 거리가 0.8m 이하이면 강제로 점 기반 판단으로 변경
                    if use_slope_logic and avg_distance > 0.8:
                    # ========================= [수정된 부분 끝] ==========================
                        self.get_logger().info(f"판단: 기울기 기반. Slope:{penetrating_slope:.2f}, Dist:{avg_distance:.2f}m")
                        position_str_raw = "right" if penetrating_slope < 0 else "left"
                        cv2.line(image, tuple(p_entry.astype(int)), tuple(p_exit.astype(int)), (0, 255, 255), 2)
                        if penetrating_slope >= 0: p_diag1, p_diag2 = (xmin, ymin), (xmax, ymax)
                        else: p_diag1, p_diag2 = (xmax, ymin), (xmin, ymax)
                        cv2.line(image, p_diag1, p_diag2, (255, 0, 255), 1)
                    else:
                        if penetrates_bottom_edge: self.get_logger().info(f"판단: 점 비교 (하단면 관통). Dist:{avg_distance:.2f}m")
                        elif not use_slope_logic: self.get_logger().info(f"판단: 점 비교 (기타). Dist:{avg_distance:.2f}m")
                        else: self.get_logger().info(f"판단: 점 비교 (거리 < 0.8m 강제). Dist:{avg_distance:.2f}m")
                        
                        bbox_center_x = (xmin + xmax) / 2
                        bbox_center_y = (ymin + ymax) / 2

                        path_comparison_point = None
                        # [예외 상황] BBox 중심이 경로 Y 범위 밖 (외삽 발생 구간)
                        if len(curve_y_coords) < 2 or bbox_center_y < curve_y_coords[0] or bbox_center_y > curve_y_coords[-1]:
                            self.get_logger().debug(
                                'Bounding-box Y is outside the curve range; '
                                'extending the first path segment.')
                            p0, p1 = curve_points[0], curve_points[1] # 경로의 '첫 번째'와 '두 번째' 점
                            dx, dy = p1[0] - p0[0], p1[1] - p0[1]
                            if abs(dx) < 1e-6: # 수직선 예외 처리
                                path_comparison_point = p1
                            else:
                                m = dy / dx
                                c = p1[1] - m * p1[0]
                                y_at_x0 = c
                                y_at_xw = m * (w - 1) + c
                                intersect_p_left = np.array([0, y_at_x0])
                                intersect_p_right = np.array([w - 1, y_at_xw])
                                dist_left = abs(intersect_p_left[0] - p0[0])
                                dist_right = abs(intersect_p_right[0] - p0[0])
                                if dist_left <= dist_right:
                                    path_comparison_point = intersect_p_left
                                else:
                                    path_comparison_point = intersect_p_right
                        # [일반 상황] BBox 중심이 경로 Y 범위 안 (안전한 보간 구간)
                        else:
                            path_x_at_bbox_y = np.interp(bbox_center_y, curve_y_coords, curve_x_coords)
                            path_comparison_point = np.array([path_x_at_bbox_y, bbox_center_y])
                        
                        position_str_raw = "right" if bbox_center_x > path_comparison_point[0] else "left"
                        cv2.circle(image, (int(bbox_center_x), int(bbox_center_y)), 5, (0, 0, 255), -1)
                        cv2.circle(image, tuple(path_comparison_point.astype(int)), 5, (255, 255, 0), -1)
                        cv2.line(image, (int(bbox_center_x), int(bbox_center_y)), tuple(path_comparison_point.astype(int)), (255, 255, 255), 1)

                else:
                    area_display_str = "No Intersection"
                    self.get_logger().info("판단: 점 비교 (교차점 부족).")
                    bbox_center_x = (xmin + xmax) / 2
                    bbox_center_y = (ymin + ymax) / 2

                    # 교차점 없을 때도 외삽 문제 해결 로직 동일하게 적용
                    path_comparison_point = None
                    if len(curve_y_coords) < 2 or bbox_center_y < curve_y_coords[0] or bbox_center_y > curve_y_coords[-1]:
                        self.get_logger().debug(
                            'Bounding-box Y is outside the curve range; '
                            'extending the first path segment.')
                        p0, p1 = curve_points[0], curve_points[1]
                        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
                        if abs(dx) < 1e-6:
                            path_comparison_point = p1
                        else:
                            m = dy / dx
                            c = p1[1] - m * p1[0]
                            y_at_x0 = c
                            y_at_xw = m * (w - 1) + c
                            intersect_p_left = np.array([0, y_at_x0])
                            intersect_p_right = np.array([w - 1, y_at_xw])
                            dist_left = abs(intersect_p_left[0] - p0[0])
                            dist_right = abs(intersect_p_right[0] - p0[0])
                            if dist_left <= dist_right:
                                path_comparison_point = intersect_p_left
                            else:
                                path_comparison_point = intersect_p_right
                    else:
                        path_x_at_bbox_y = np.interp(bbox_center_y, curve_y_coords, curve_x_coords)
                        path_comparison_point = np.array([path_x_at_bbox_y, bbox_center_y])

                    position_str_raw = "right" if bbox_center_x > path_comparison_point[0] else "left"
                    cv2.circle(image, (int(bbox_center_x), int(bbox_center_y)), 5, (0, 0, 255), -1)
                    cv2.circle(image, tuple(path_comparison_point.astype(int)), 5, (255, 255, 0), -1)
                    cv2.line(image, (int(bbox_center_x), int(bbox_center_y)), tuple(path_comparison_point.astype(int)), (255, 255, 255), 1)


            if avg_distance > 0:
                obstacle_data = {
                    'distance': avg_distance, 'position_raw': position_str_raw,
                    'event_kind': event_kind,
                    'xmin': xmin, 'ymin': ymin, 'xmax': xmax, 'ymax': ymax,
                    'points_in_path_count': points_in_path_count,
                    'cluster_image_points': cluster_image_points,
                    'area_ratio_str': area_display_str
                }
                processed_obstacles.append(obstacle_data)
                cv2.putText(image, f"{avg_distance:.1f}m", (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(image, f"Raw: {position_str_raw}", (xmin, ymin - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        final_position_str = "unknown"
        target_obstacle_for_decision = None

        if processed_obstacles:
            target_obstacle_for_decision = min(processed_obstacles, key=lambda x: x['distance'])
            raw_pos_of_target = target_obstacle_for_decision['position_raw']
            
            # 차선 판단 로직이 수행되지 않아 'unknown'인 경우, 거리를 확인하여 fallback 여부 결정
            if raw_pos_of_target == "unknown":
                # 장애물이 판단 유효 거리 내에 있을 때만 fallback 로직(이전 값 사용)을 적용
                lane_determination_distance_m = self.profile_distance(
                    target_obstacle_for_decision['event_kind'],
                    'lane_determination',
                )
                if (
                    target_obstacle_for_decision['distance']
                    <= lane_determination_distance_m
                ):
                    raw_pos_of_target = self.last_closest_obstacle_lane
                    self.get_logger().debug(
                        'Lane determination failed for a close obstacle; '
                        f'using last lane {raw_pos_of_target}.')
                # 거리가 멀면 'unknown'을 그대로 유지 (fallback 없음)
            else:
                # 새로운 차선 정보가 유효하면(left/right), 최신 정보로 업데이트
                self.last_closest_obstacle_lane = raw_pos_of_target

            current_numeric_pos = 0.0
            if raw_pos_of_target == "left": current_numeric_pos = -1.0
            elif raw_pos_of_target == "right": current_numeric_pos = 1.0
            self.smoothed_obstacle_position = (self.EMA_ALPHA * current_numeric_pos) + ((1 - self.EMA_ALPHA) * self.smoothed_obstacle_position)
            if self.smoothed_obstacle_position < -0.1: final_position_str = "left"
            elif self.smoothed_obstacle_position > 0.1: final_position_str = "right"
            target_obstacle_for_decision['position_final'] = final_position_str
            self.get_logger().info(f"Closest Obstacle Raw: {raw_pos_of_target} -> Smoothed Val: {self.smoothed_obstacle_position:.2f} -> Final Decision: {final_position_str}")
        else:
            self.smoothed_obstacle_position = 0.0

        # ================================ E_STOP ================================
        cv2.rectangle(image, (self.E_STOP_RECT_X_MIN, self.E_STOP_RECT_Y_MIN), (self.E_STOP_RECT_X_MAX, self.E_STOP_RECT_Y_MAX), (0, 0, 255), 1)
        is_pixel_emergency = False
        for obstacle in processed_obstacles:
            cluster_points = obstacle['cluster_image_points']
            points_in_danger_zone = cluster_points[
                (cluster_points[:, 0] > self.E_STOP_RECT_X_MIN)
                & (cluster_points[:, 0] < self.E_STOP_RECT_X_MAX)
                & (cluster_points[:, 1] > self.E_STOP_RECT_Y_MIN)
                & (cluster_points[:, 1] < self.E_STOP_RECT_Y_MAX)
            ]
            if (
                len(points_in_danger_zone)
                > self.E_STOP_RECT_PIXEL_COUNT_THRESHOLD
            ):
                is_pixel_emergency = True
                self.get_logger().error(
                    'RECT E-STOP TRIGGER: matched LiDAR cluster has '
                    f'{len(points_in_danger_zone)} points in the danger zone.')
                cv2.rectangle(
                    image,
                    (self.E_STOP_RECT_X_MIN, self.E_STOP_RECT_Y_MIN),
                    (self.E_STOP_RECT_X_MAX, self.E_STOP_RECT_Y_MAX),
                    (0, 0, 255),
                    3,
                )
                cv2.rectangle(
                    image,
                    (obstacle['xmin'], obstacle['ymin']),
                    (obstacle['xmax'], obstacle['ymax']),
                    (0, 0, 255),
                    3,
                )
                break

        is_obstacle_emergency = False
        emergency_obstacle = None
        for obs in processed_obstacles:
            obs_position = obs['position_raw'] 
            is_center_path = self.last_override_cmd == 'reset'
            is_left_path = self.last_override_cmd == 'go_left'
            is_right_path = self.last_override_cmd == 'go_right'
            collision_path = (obs_position == 'left' and (is_center_path or is_left_path)) or \
                                (obs_position == 'right' and (is_center_path or is_right_path))
            if obs['distance'] < self.E_STOP_DISTANCE_M and obs['points_in_path_count'] > 0 and collision_path:
                is_obstacle_emergency = True
                emergency_obstacle = obs
                self.get_logger().error(f"🚨 E-STOP TRIGGER! Obstacle at {obs['distance']:.2f}m on a collision path.")
                break
        # ================================ E_STOP ================================

        # ============================ obstacle lane event ============================
        obstacle_msg = ObstacleState()
        target_obstacle_for_state_machine = target_obstacle_for_decision
        num_detected_obstacles = len(detections)
        is_curve = str(road_curve_state).strip().lower() == 'curve'
        obstacle_lane = (
            target_obstacle_for_state_machine['position_final']
            if target_obstacle_for_state_machine else 'unknown'
        )
        obstacle_kind = (
            target_obstacle_for_state_machine['event_kind']
            if target_obstacle_for_state_machine else 'unknown'
        )
        if (
            target_obstacle_for_state_machine is None
            and camera_fallback_detection is not None
        ):
            obstacle_kind = self.dynamic_event_machine.normalize_obstacle_kind(
                camera_fallback_detection.class_name)
        emergency = is_obstacle_emergency or is_pixel_emergency
        target_distance_m = (
            target_obstacle_for_state_machine['distance']
            if target_obstacle_for_state_machine is not None
            else float('inf')
        )
        target_bbox = (
            (
                target_obstacle_for_state_machine['xmin'],
                target_obstacle_for_state_machine['ymin'],
                target_obstacle_for_state_machine['xmax'],
                target_obstacle_for_state_machine['ymax'],
            )
            if target_obstacle_for_state_machine is not None
            else None
        )
        if target_bbox is None and camera_fallback_detection is not None:
            target_bbox = (
                camera_fallback_detection.xmin,
                camera_fallback_detection.ymin,
                camera_fallback_detection.xmax,
                camera_fallback_detection.ymax,
            )
        camera_speed_fallback = (
            camera_fallback_detection is not None
        )
        speed_trigger_allowed = (
            (
                camera_speed_fallback
                or (
                    target_obstacle_for_state_machine is not None
                    and target_distance_m < self.profile_distance(
                        obstacle_kind, 'speed_trigger')
                )
            )
            and not emergency
        )
        trigger_allowed = (
            num_detected_obstacles == 1
            and target_obstacle_for_state_machine is not None
            and road_allows_obstacle_event(
                is_curve,
                self.obstacle_allow_avoid_on_curve,
            )
            and target_distance_m < self.profile_distance(
                obstacle_kind, 'overtake_trigger')
            and obstacle_lane in ('left', 'right')
            and not emergency
        )
        decision = self.dynamic_event_machine.update(
            self.get_clock().now().nanoseconds * 1e-9,
            obstacle_lane=obstacle_lane,
            obstacle_kind=obstacle_kind,
            object_present=num_detected_obstacles > 0,
            speed_trigger_allowed=speed_trigger_allowed,
            trigger_allowed=trigger_allowed,
            obstacle_bbox=target_bbox,
            new_target_present=(
                self.obstacle_lidar_tracking_enabled
                and self.obstacle_cluster_tracker.last_rejection_reason
                == 'camera_target_jump'
            ),
            blocked=emergency or self.dynamic_event_blocked(),
        )
        self.set_dynamic_event_state(decision)
        self.publish_obstacle_speed_limit(decision)
        current_override_cmd = decision.command
        # =========================== obstacle lane event ===========================

        # # ============================ 밑에는 확실한 곡선 추월 금지 로직 ======================
        # # --------------- [전환 0] (Curve, 1), (Curve, 2) 의 경우 ---------------
        # # --------------- 무조건 가장 가까운 차의 차선으로 변경 -> 측면 추돌 예방 -------
        # # -- 발생 문제 : centerlane_tracer에서 Curve로 튀면 -> (Curve, 1)이 되어 버려서 추월하다가 장애물 차량의 차선으로 변경하여 측면 추돌해버릴 수도 있음. -----
        # if is_curve and num_detected_vehicles >= 1 :
        #     self.driving_state = "Time_Gap"
        #     if processed_obstacles:
        #         target_obstacle_for_state_machine = min(processed_obstacles, key=lambda obs: obs['distance'])
        #         current_override_cmd = 'reset'
        #         self.get_logger().warn(f"(Curve, 1), (Curve, 2) Case : Target at {target_obstacle_for_state_machine['distance']:.1f}m.")

        # else:
        # # ----------- 나머지 (Straight, 0), (Straight, 1), (Straight, 2), (Curve, 0) 의 경우를 결정 --------
        #     # [상태 1] 기본 (CENTER_DRIVING)
        #     if self.driving_state == "CENTER_DRIVING":

        #         # --------------- [전환 1] (Straight, 2) 의 경우 ------------------
        #         # --------------- 앞에 무조건 직선 구간에 두 대가 있으므로 측면 추돌을 생각할 필요가 X -> center로 달려도 됨.-----
        #         if num_detected_vehicles >= 2:
        #             self.driving_state = "Time_Gap"
        #             self.get_logger().warn("STATE CHANGE: CENTER_DRIVING -> Time_Gap + (Straight, 2).")
        #             if processed_obstacles:
        #                 target_obstacle_for_state_machine = min(processed_obstacles, key=lambda obs: obs['distance'])
        #             current_override_cmd = 'reset'

        #         # --------------- [전환 2] (Straight, 1) 의 경우 ------------------
        #         elif num_detected_vehicles == 1 and processed_obstacles:
        #             obstacle = processed_obstacles[0]
        #             if obstacle['distance'] < self.OVERTAKE_THRESHOLD_M:
        #                 self.driving_state = "OVERTAKING"
        #                 self.is_passing_obstacle = False
        #                 self.pass_complete_timer = None
        #                 target_obstacle_for_state_machine = obstacle
        #                 current_override_cmd = 'go_left' if target_obstacle_for_state_machine['position_final'] == 'right' else 'go_right' # 장애물 차량과 반대 차선으로 차선 변경
        #                 self.get_logger().warn(f"STATE CHANGE: CENTER_DRIVING -> OVERTAKING ({current_override_cmd}).")
        #             else: # 1대 있지만 멀리 있으면 중앙 주행 유지
        #                 current_override_cmd = 'reset'

        #         # --------------- [전환 3] (Straight, 0), (Curve, 0) 의 경우 ------------------       
        #         else: # 감지된 차량이 없거나, 있어도 거리 측정을 못하면 중앙 주행 유지
        #             current_override_cmd = 'reset'

        #     # [상태 2] 추월 중 (OVERTAKING) + (Straight, 1)의 경우
        #     elif self.driving_state == "OVERTAKING":
        #         if processed_obstacles:
        #             target_obstacle_for_state_machine = min(processed_obstacles, key=lambda obs: obs['distance'])

        #         current_override_cmd = self.last_override_cmd # 추월 중에는 차선 변경 명령 유지

        #         # 1단계: 아직 장애물 옆을 지나치기 시작하지 않았을 때
        #         if not self.is_passing_obstacle:
        #             is_side_detected = (self.last_override_cmd == 'go_left' and right_ultra < self.ULTAR_THRESHOLD) or \
        #                                (self.last_override_cmd == 'go_right' and left_ultra < self.ULTAR_THRESHOLD)
        #             if is_side_detected:
        #                 self.is_passing_obstacle = True
        #                 self.get_logger().info("Side of obstacle detected. Now passing alongside.")
        #             else:
        #                 self.get_logger().info(f"Approaching side... L: {left_ultra}, R: {right_ultra}, LB: {left_back_ultra}, RB: {right_back_ultra}")
                
        #         # 2단계: 장애물 옆을 지나치고 있을 때
        #         else:
        #             is_side_cleared = (self.last_override_cmd == 'go_left' and right_ultra > self.ULTAR_THRESHOLD and right_back_ultra > self.ULTAR_BACK_THRESHOLD) or \
        #                               (self.last_override_cmd == 'go_right' and left_ultra > self.ULTAR_THRESHOLD and left_back_ultra > self.ULTAR_BACK_THRESHOLD)

        #             # 3단계: 장애물 끝을 통과하여 타이머를 시작해야 할 때
        #             if is_side_cleared and self.pass_complete_timer is None:
        #                 self.pass_complete_timer = self.get_clock().now()
        #                 self.get_logger().info(f"Side of obstacle cleared. Starting {self.PASS_TIMER_DURATION_S}s safety timer.")
                    
        #             # 타이머가 시작되었다면, 시간이 다 지났는지 확인
        #             if self.pass_complete_timer is not None:
        #                 duration = self.get_clock().now() - self.pass_complete_timer
        #                 # 4단계: 타이머 종료 -> 추월 완료!
        #                 if duration.nanoseconds / 1e9 > self.PASS_TIMER_DURATION_S:
        #                     self.driving_state = "CENTER_DRIVING"
        #                     current_override_cmd = 'reset'
        #                     self.get_logger().warn("STATE CHANGE: OVERTAKING -> CENTER_DRIVING (Timer complete).")
        #                     # 상태 변수 초기화
        #                     self.is_passing_obstacle = False
        #                     self.pass_complete_timer = None
        #                 else:
        #                     self.get_logger().info(f"Safety timer running... ({duration.nanoseconds / 1e9:.2f}s)")
        #             # 아직 장애물 옆을 지나고 있는 경우 (통과 중 센서 값이 20 미만인 상태)
        #             else:
        #                  self.get_logger().info(f"Passing alongside... L: {left_ultra}, R: {right_ultra}, LB: {left_back_ultra}, RB: {right_back_ultra}")

        #     # [상태 3] 다중 차량 추종 (Time_Gap) + (Straight, 2)의 경우
        #     elif self.driving_state == "Time_Gap":
        #         # 조건 1: 차량이 2대 미만이면 즉시 중앙 주행으로 복귀 (상태 탈출)
        #         if num_detected_vehicles < 2:
        #             self.driving_state = "CENTER_DRIVING"
        #             current_override_cmd = 'reset'
        #             self.get_logger().warn("STATE CHANGE: Time_Gap -> CENTER_DRIVING (Vehicles < 2).")
                
        #         # 조건 2: 2대 이상이면 Time_Gap 상태 유지
        #         else:
        #             # Time_Gap 상태에서는 항상 중앙 차선을 유지합니다.
        #             current_override_cmd = 'reset'
                    
        #             # 거리 측정이 가능한 차량이 있다면, 그중 가장 가까운 차를 추종 타겟으로 삼습니다.
        #             if processed_obstacles:
        #                 target_obstacle_for_state_machine = min(processed_obstacles, key=lambda obs: obs['distance'])
        #                 self.get_logger().info(f"Time_Gap: Following closest vehicle at {target_obstacle_for_state_machine['distance']:.1f}m.")
        #             else:
        #                 self.get_logger().info("Time_Gap: 2+ vehicles detected, but no distance info. Driving straight.")

        # # =======================> 확실한 추월 금지 로직 끝 <=========================

        self.publish_override_if_changed(current_override_cmd)
            
        if is_obstacle_emergency or is_pixel_emergency:
            obstacle_msg.distance_m = 0.0
            if is_obstacle_emergency:
                obstacle_msg.position = emergency_obstacle['position_raw']
            else:
                obstacle_msg.position = "pixel_danger"
            if self.driving_state != "E_STOP":
                 self.get_logger().error("STATE CHANGE: ANY -> E_STOP")
                 self.driving_state = "E_STOP"
        else:
            if self.driving_state == "E_STOP":
                self.driving_state = "CENTER_DRIVING"
                self.get_logger().warn("STATE CHANGE: E_STOP -> CENTER_DRIVING (Obstacle Cleared).")
            if target_obstacle_for_decision:
                obstacle_msg.distance_m = float(target_obstacle_for_decision['distance'])
                obstacle_msg.position = target_obstacle_for_decision['position_final']
                tx, ty = target_obstacle_for_decision['xmin'], target_obstacle_for_decision['ymin']
                cv2.putText(image, f"FINAL TARGET: {final_position_str}", (tx, ty - 50), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 255), 2)
            else:
                obstacle_msg.distance_m = 999.0
                obstacle_msg.position = "none"

        obstacle_msg.vehicle_count = num_detected_obstacles
        self.obstacle_state_pub.publish(obstacle_msg)
        self.overtake_state_pub.publish(String(data=self.driving_state.upper()))

        # ================================ 요구사항 추가 부분 시작 ================================
        h, w, _ = image.shape
        closest_lane = 'None'
        farthest_lane = 'None'
        closest_vehicle_ratio = 'N/A'
        if len(processed_obstacles) >= 2:
            sorted_obstacles_by_dist = sorted(processed_obstacles, key=lambda x: x['distance'])
            farthest_lane = sorted_obstacles_by_dist[-1]['position_raw']
        if target_obstacle_for_decision:
            closest_lane = final_position_str # 최종 결정 값(smoothed)을 사용하도록 수정
            closest_vehicle_ratio = target_obstacle_for_decision.get('area_ratio_str', 'N/A')
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_color = (255, 255, 255)
        thickness = 2
        line_type = cv2.LINE_AA
        cv2.putText(image, f"closest vehicle's lane : {closest_lane}", (w - 290, 30), font, font_scale, font_color, thickness, line_type)
        cv2.putText(image, f"farthest vehicle's lane : {farthest_lane}", (w - 290, 60), font, font_scale, font_color, thickness, line_type)
        cv2.putText(image, f"Clip Ratio (Closest): {closest_vehicle_ratio}", (w - 330, 90), font, font_scale, font_color, thickness, line_type)
        # ================================ 요구사항 추가 부분 끝 ==================================

        return image

    def synchronized_callback(self, image_msg, curve_msg, yolo_msg):
        if self.mission_mode in ('CONE', 'STOP'):
            self.obstacle_cluster_tracker.reset()
            return

        obstacle_detections = [
            det for det in yolo_msg.detections
            if det.class_name in ('dynamic', 'obstacle_vehicle', 'static')
        ]
        obstacle_boxes = [
            (det.xmin, det.ymin, det.xmax, det.ymax)
            for det in obstacle_detections
        ]
        image_source_ns = self.stamp_to_ns(image_msg.header.stamp)
        if not obstacle_detections:
            if self.obstacle_lidar_tracking_enabled:
                self.obstacle_cluster_tracker.associate(
                    [], [], image_source_ns)
            self.publish_tracking_reset()
            return

        now_ns = self.get_clock().now().nanoseconds
        lidar_snapshot = self.select_lidar_snapshot(image_msg, now_ns)
        if lidar_snapshot is None:
            if self.obstacle_lidar_tracking_enabled:
                self.obstacle_cluster_tracker.associate(
                    obstacle_boxes,
                    [],
                    image_source_ns,
                    padding_px=self.obstacle_lidar_association_padding_px,
                    max_above_bbox_px=(
                        self.obstacle_lidar_association_max_above_px),
                )
            self.publish_tracking_reset(
                vehicle_count=len(obstacle_detections),
                sensor_stale=True,
            )
            return

        if self.last_fast_path_status == 'sensor_stale':
            self.get_logger().info(
                'LiDAR input recovered; resuming vehicle ranging.'
            )
        self.last_fast_path_status = 'active'
        ultra_msg = self.latest_ultra_msg

        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        curve_msg = self.scale_curve_to_image(
            curve_msg,
            frame.shape[1],
            frame.shape[0],
        )
        road_curve_state = self.road_curve_state_for(
            image_msg,
            curve_msg.state,
        )
        
        left_ultra = 100
        left_back_ultra = 999
        right_ultra = 100
        right_back_ultra = 999
        ultra_is_fresh = self.sensor_is_fresh(
            self.last_ultra_received_ns, now_ns)
        if ultra_msg is not None and ultra_is_fresh:
            if len(ultra_msg.data) > 0: left_ultra = ultra_msg.data[0]
            if len(ultra_msg.data) > 7: left_back_ultra = ultra_msg.data[7]
            if len(ultra_msg.data) > 5: right_back_ultra = ultra_msg.data[5]
            if len(ultra_msg.data) > 4: right_ultra = ultra_msg.data[4]

        projected_clusters = self.project_lidar_clusters(
            lidar_snapshot.clusters,
            frame.shape[1],
            frame.shape[0],
        )
                
        if self.show_visualization and curve_msg.points:
            # 주행 경로를 선으로 그립니다 (기존 코드)
            curve_pts = np.array([[p.x, p.y] for p in curve_msg.points], dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [curve_pts], isClosed=False, color=(255, 0, 255), thickness=2)
            
            # 주행 경로를 구성하는 실제 점들을 원으로 그립니다
            for p in curve_msg.points:
                cv2.circle(frame, (int(p.x), int(p.y)), 3, (0, 255, 255), -1) # 노란색(-1: 채워진 원)

        if self.show_visualization and curve_msg.state:
            cv2.putText(
                frame,
                f'Route/Road: {curve_msg.state}/{road_curve_state}',
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        vehicle_count = len(obstacle_detections)
        if self.show_visualization:
            for det in yolo_msg.detections:
                if det.class_name in (
                        'dynamic', 'obstacle_vehicle', 'static'):
                    x1, y1, x2, y2 = map(
                        int, [det.xmin, det.ymin, det.xmax, det.ymax])
                    cv2.rectangle(
                        frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"Vehicles Detected: {vehicle_count}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"State: {self.driving_state}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Left Ultra: {left_ultra}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Left Back Ultra: {left_back_ultra}", (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Right Ultra: {right_ultra}", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, f"Right Back Ultra: {right_back_ultra}", (20, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        frame = self.project_lidar_on_image(
            frame,
            yolo_msg,
            projected_clusters,
            lidar_snapshot.source_ns,
            curve_msg,
            road_curve_state,
            left_ultra,
            right_ultra,
            left_back_ultra,
            right_back_ultra,
        )
        if self.show_visualization:
            cv2.imshow("target_lane_planner : Integrated Visualization", frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TargetLanePlanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
