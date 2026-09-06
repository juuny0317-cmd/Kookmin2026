from collections import deque
from dataclasses import dataclass
import time

from custom_interfaces.msg import (
    ClusterData,
    Detections,
    StampedMotorCommand,
)
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger


@dataclass
class TimedMotorCommand:
    angle: float = 0.0
    speed: float = 0.0
    received_ns: int = 0
    command_ns: int = 0
    source_ns: int = 0
    state_age_s: float = -1.0
    is_stamped: bool = False


class MissionManagerNode(Node):
    STOP_MODE = 'STOP'
    PAUSED_MODE = 'PAUSED'
    LANE_MODE = 'LANE'
    OVERTAKE_MODE = 'OVERTAKE'
    CONE_MODE = 'CONE'

    def __init__(self):
        super().__init__('mission_manager_node')

        self.declare_parameter('lane_command_topic', '/lane_motor_cmd')
        self.declare_parameter(
            'lane_stamped_command_topic', '/lane_motor_cmd_stamped')
        self.declare_parameter('use_stamped_lane_command', True)
        self.declare_parameter('cone_command_topic', '/cone_motor_cmd')
        self.declare_parameter('motor_topic', '/xycar_motor')
        self.declare_parameter('steering_trim', -6.0)
        self.declare_parameter(
            'detections_topic', '/scene_yolo/detections')
        self.declare_parameter('cone_clusters_topic', '/clusters')
        self.declare_parameter(
            'entry_cone_clusters_topic',
            '/entry_cone_clusters',
        )
        self.declare_parameter(
            'prearm_cone_clusters_topic',
            '/cone_prearm_clusters',
        )
        self.declare_parameter('cone_path_topic', '/path')
        self.declare_parameter('mission_mode_topic', '/mission_mode')
        self.declare_parameter('mission_status_topic', '/mission_status')
        self.declare_parameter('lane_override_topic', '/lane_override_cmd')
        self.declare_parameter(
            'traffic_topics',
            ['/traffic_light_state', '/sign_color'],
        )

        self.declare_parameter('start_service_name', '/start_integrated_drive')
        self.declare_parameter('stop_service_name', '/stop_integrated_drive')
        self.declare_parameter('pause_service_name', '/pause_integrated_drive')
        self.declare_parameter(
            'resume_service_name', '/resume_integrated_drive')
        self.declare_parameter(
            'toggle_pause_service_name', '/toggle_pause_integrated_drive')
        self.declare_parameter('lane_start_service', '/start_lane_detection')
        self.declare_parameter(
            'stanley_start_service',
            '/start_stanley_controller',
        )
        self.declare_parameter('start_active', False)
        self.declare_parameter('require_vesc_ready', True)
        self.declare_parameter('vesc_ready_topic', '/vesc/ready')
        self.declare_parameter('vesc_ready_timeout_s', 1.25)

        self.declare_parameter('decision_rate_hz', 10.0)
        self.declare_parameter('event_driven_lane_output', False)
        self.declare_parameter('command_timeout_s', 0.60)
        self.declare_parameter('lane_source_timeout_s', 0.80)
        self.declare_parameter('future_stamp_tolerance_s', 0.05)
        self.declare_parameter('sensor_timeout_s', 0.80)
        self.declare_parameter('respect_traffic_light', True)
        self.declare_parameter('enable_scene_inputs', True)
        self.declare_parameter('enable_cone_mission', True)

        self.declare_parameter('cone_class_name', 'cone')
        self.declare_parameter('cone_conf_threshold', 0.15)
        self.declare_parameter('cone_min_bbox_area_px', 180.0)
        self.declare_parameter('cone_entry_min_bottom_y_px', 220.0)
        self.declare_parameter('cone_entry_min_detections', 2)
        self.declare_parameter('cone_entry_min_clusters', 2)
        self.declare_parameter('cone_entry_min_path_points', 1)
        self.declare_parameter('cone_entry_use_fusion_gate', True)
        self.declare_parameter('cone_entry_allow_lidar_fallback', True)
        self.declare_parameter('cone_entry_min_fused_clusters', 2)
        self.declare_parameter('cone_entry_require_both_sides', True)
        self.declare_parameter('cone_entry_side_min_abs_y_m', 0.10)
        self.declare_parameter('cone_entry_fusion_timeout_s', 0.60)
        self.declare_parameter('cone_suspect_min_fused_clusters', 1)
        self.declare_parameter('cone_suspect_hold_s', 0.70)
        self.declare_parameter('cone_suspect_speed_cap', 0.0)
        self.declare_parameter('cone_prearm_enabled', True)
        self.declare_parameter('cone_bbox_prearm_enabled', True)
        self.declare_parameter('cone_prearm_use_fusion', False)
        self.declare_parameter('cone_prearm_min_bbox_area_px', 100.0)
        self.declare_parameter('cone_prearm_min_bottom_y_px', 170.0)
        self.declare_parameter('cone_bbox_prearm_min_detections', 1)
        self.declare_parameter('cone_bbox_prearm_clear_frames', 3)
        self.declare_parameter('cone_prearm_min_clusters', 1)
        self.declare_parameter('cone_prearm_window', 3)
        self.declare_parameter('cone_prearm_min_matches', 2)
        self.declare_parameter('cone_prearm_hold_s', 0.60)
        self.declare_parameter('cone_prearm_speed_cap', 4.0)
        self.declare_parameter('cone_perception_mode', 'lidar_dbscan')
        self.declare_parameter('cone_allow_reentry', True)
        self.declare_parameter('cone_enter_confirm_cycles', 3)
        self.declare_parameter('cone_min_mode_duration_s', 3.0)
        self.declare_parameter('cone_exit_hold_s', 1.0)
        self.declare_parameter('cone_speed_accel_rate_per_s', 4.0)
        self.declare_parameter('cone_speed_decel_rate_per_s', 20.0)
        self.declare_parameter('status_publish_rate_hz', 2.0)

        self.command_timeout_s = float(
            self.get_parameter('command_timeout_s').value)
        self.event_driven_lane_output = self.as_bool(
            self.get_parameter('event_driven_lane_output').value)
        self.lane_source_timeout_s = float(
            self.get_parameter('lane_source_timeout_s').value)
        self.future_stamp_tolerance_s = max(
            0.0,
            float(self.get_parameter('future_stamp_tolerance_s').value),
        )
        self.use_stamped_lane_command = self.as_bool(
            self.get_parameter('use_stamped_lane_command').value)
        self.steering_trim = float(
            self.get_parameter('steering_trim').value)
        self.sensor_timeout_s = float(
            self.get_parameter('sensor_timeout_s').value)
        self.respect_traffic_light = self.as_bool(
            self.get_parameter('respect_traffic_light').value
        )
        self.enable_scene_inputs = self.as_bool(
            self.get_parameter('enable_scene_inputs').value)
        self.enable_cone_mission = self.as_bool(
            self.get_parameter('enable_cone_mission').value)
        self.require_vesc_ready = self.as_bool(
            self.get_parameter('require_vesc_ready').value)
        self.vesc_ready_timeout_s = max(
            0.1,
            float(self.get_parameter('vesc_ready_timeout_s').value),
        )
        self.cone_class_name = str(
            self.get_parameter('cone_class_name').value)
        self.cone_conf_threshold = float(
            self.get_parameter('cone_conf_threshold').value)
        self.cone_min_bbox_area_px = float(
            self.get_parameter('cone_min_bbox_area_px').value)
        self.cone_entry_min_bottom_y_px = float(
            self.get_parameter('cone_entry_min_bottom_y_px').value
        )
        self.cone_entry_min_detections = int(
            self.get_parameter('cone_entry_min_detections').value
        )
        self.cone_entry_min_clusters = int(
            self.get_parameter('cone_entry_min_clusters').value
        )
        self.cone_entry_min_path_points = int(
            self.get_parameter('cone_entry_min_path_points').value
        )
        self.cone_entry_use_fusion_gate = self.as_bool(
            self.get_parameter('cone_entry_use_fusion_gate').value
        )
        self.cone_entry_allow_lidar_fallback = self.as_bool(
            self.get_parameter('cone_entry_allow_lidar_fallback').value
        )
        self.cone_entry_min_fused_clusters = int(
            self.get_parameter('cone_entry_min_fused_clusters').value
        )
        self.cone_entry_require_both_sides = self.as_bool(
            self.get_parameter('cone_entry_require_both_sides').value
        )
        self.cone_entry_side_min_abs_y_m = float(
            self.get_parameter('cone_entry_side_min_abs_y_m').value
        )
        self.cone_entry_fusion_timeout_s = float(
            self.get_parameter('cone_entry_fusion_timeout_s').value
        )
        self.cone_suspect_min_fused_clusters = int(
            self.get_parameter('cone_suspect_min_fused_clusters').value
        )
        self.cone_suspect_hold_s = float(
            self.get_parameter('cone_suspect_hold_s').value
        )
        self.cone_suspect_speed_cap = float(
            self.get_parameter('cone_suspect_speed_cap').value
        )
        self.cone_prearm_enabled = self.as_bool(
            self.get_parameter('cone_prearm_enabled').value
        )
        self.cone_bbox_prearm_enabled = self.as_bool(
            self.get_parameter('cone_bbox_prearm_enabled').value
        )
        self.cone_prearm_use_fusion = self.as_bool(
            self.get_parameter('cone_prearm_use_fusion').value
        )
        self.cone_prearm_min_bbox_area_px = max(
            0.0,
            float(self.get_parameter(
                'cone_prearm_min_bbox_area_px').value),
        )
        self.cone_prearm_min_bottom_y_px = max(
            0.0,
            float(self.get_parameter(
                'cone_prearm_min_bottom_y_px').value),
        )
        self.cone_bbox_prearm_min_detections = max(
            1,
            int(self.get_parameter(
                'cone_bbox_prearm_min_detections').value),
        )
        self.cone_bbox_prearm_clear_frames = max(
            1,
            int(self.get_parameter(
                'cone_bbox_prearm_clear_frames').value),
        )
        self.cone_prearm_min_clusters = max(
            1,
            int(self.get_parameter('cone_prearm_min_clusters').value),
        )
        self.cone_prearm_window = max(
            1,
            int(self.get_parameter('cone_prearm_window').value),
        )
        self.cone_prearm_min_matches = min(
            self.cone_prearm_window,
            max(
                1,
                int(self.get_parameter('cone_prearm_min_matches').value),
            ),
        )
        self.cone_prearm_hold_s = max(
            0.0,
            float(self.get_parameter('cone_prearm_hold_s').value),
        )
        self.cone_prearm_speed_cap = float(
            self.get_parameter('cone_prearm_speed_cap').value
        )
        self.cone_perception_mode = str(
            self.get_parameter('cone_perception_mode').value
        ).strip().lower()
        self.cone_allow_reentry = self.as_bool(
            self.get_parameter('cone_allow_reentry').value
        )
        self.cone_enter_confirm_cycles = int(
            self.get_parameter('cone_enter_confirm_cycles').value
        )
        self.cone_min_mode_duration_s = float(
            self.get_parameter('cone_min_mode_duration_s').value
        )
        self.cone_exit_hold_s = float(
            self.get_parameter('cone_exit_hold_s').value)
        self.cone_speed_accel_rate_per_s = max(
            0.0,
            float(self.get_parameter(
                'cone_speed_accel_rate_per_s').value),
        )
        self.cone_speed_decel_rate_per_s = max(
            0.0,
            float(self.get_parameter(
                'cone_speed_decel_rate_per_s').value),
        )
        status_publish_rate_hz = float(
            self.get_parameter('status_publish_rate_hz').value)
        self.status_publish_period_ns = int(
            1e9 / max(status_publish_rate_hz, 0.1))

        self.active = self.as_bool(self.get_parameter('start_active').value)
        self.paused = False
        self.startup_monotonic_s = time.monotonic()
        self.mode = self.LANE_MODE
        self.traffic_state = 'unknown'
        self.lane_override = 'reset'
        self.lane_command = TimedMotorCommand()
        self.cone_command = TimedMotorCommand()
        self.last_selected_speed = None
        self.last_selected_publish_ns = 0
        self.cone_detection_count = 0
        self.cone_bbox_prearm_detection_count = 0
        self.cone_cluster_count = 0
        self.cone_path_count = 0
        self.entry_fused_cluster_count = 0
        self.entry_fused_left_count = 0
        self.entry_fused_right_count = 0
        self.prearm_fused_cluster_count = 0
        self.detection_received_ns = 0
        self.cluster_received_ns = 0
        self.path_received_ns = 0
        self.entry_fusion_received_ns = 0
        self.prearm_fusion_received_ns = 0
        self.cone_confirm_cycles = 0
        self.cone_completed = False
        self.last_cone_evidence_ns = 0
        self.cone_suspect_until_ns = 0
        self.cone_prearm_until_ns = 0
        self.cone_prearm_votes = deque(maxlen=self.cone_prearm_window)
        self.cone_bbox_prearm_votes = deque(
            maxlen=self.cone_prearm_window)
        self.cone_bbox_prearm_miss_count = 0
        self.cone_bbox_prearm_latched = False
        self.last_entry_source = 'none'
        self.transition_started_ns = 0
        self.mode_entered_ns = self.now_ns()
        self.last_status = ''
        self.last_status_publish_ns = 0
        self.last_published_mode = ''
        self.lane_start_requested = False
        self.stanley_start_requested = False
        self.vesc_ready = False
        self.vesc_ready_received_ns = 0
        self.first_lane_command_logged = False
        self.integrated_ready_logged = False
        self.last_readiness_summary = ''

        if self.use_stamped_lane_command:
            self.lane_command_sub = self.create_subscription(
                StampedMotorCommand,
                self.get_parameter('lane_stamped_command_topic').value,
                self.stamped_lane_command_callback,
                10,
            )
        else:
            self.lane_command_sub = self.create_subscription(
                Float32MultiArray,
                self.get_parameter('lane_command_topic').value,
                self.lane_command_callback,
                10,
            )
        if self.enable_cone_mission:
            self.create_subscription(
                Float32MultiArray,
                self.get_parameter('cone_command_topic').value,
                self.cone_command_callback,
                10,
            )
            self.create_subscription(
                ClusterData,
                self.get_parameter('cone_clusters_topic').value,
                self.cluster_callback,
                10,
            )
            self.create_subscription(
                ClusterData,
                self.get_parameter('entry_cone_clusters_topic').value,
                self.entry_fusion_callback,
                10,
            )
            self.create_subscription(
                ClusterData,
                self.get_parameter('prearm_cone_clusters_topic').value,
                self.prearm_fusion_callback,
                10,
            )
            self.create_subscription(
                Path,
                self.get_parameter('cone_path_topic').value,
                self.path_callback,
                10,
            )
        if self.enable_scene_inputs:
            self.create_subscription(
                Detections,
                self.get_parameter('detections_topic').value,
                self.detection_callback,
                10,
            )
        self.create_subscription(
            String,
            self.get_parameter('lane_override_topic').value,
            self.lane_override_callback,
            10,
        )
        if self.require_vesc_ready:
            ready_qos = QoSProfile(depth=1)
            ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.vesc_ready_sub = self.create_subscription(
                Bool,
                self.get_parameter('vesc_ready_topic').value,
                self.vesc_ready_callback,
                ready_qos,
            )
        if self.enable_scene_inputs:
            for topic in self.get_parameter('traffic_topics').value:
                self.create_subscription(
                    String, topic, self.traffic_callback, 10)

        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('motor_topic').value,
            10,
        )
        mode_qos = QoSProfile(depth=1)
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mode_pub = self.create_publisher(
            String,
            self.get_parameter('mission_mode_topic').value,
            mode_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter('mission_status_topic').value,
            10,
        )
        self.override_pub = self.create_publisher(
            String,
            self.get_parameter('lane_override_topic').value,
            10,
        )

        self.lane_start_client = self.create_client(
            Trigger,
            self.get_parameter('lane_start_service').value,
        )
        self.stanley_start_client = self.create_client(
            Trigger,
            self.get_parameter('stanley_start_service').value,
        )
        self.start_service = self.create_service(
            Trigger,
            self.get_parameter('start_service_name').value,
            self.start_callback,
        )
        self.stop_service = self.create_service(
            Trigger,
            self.get_parameter('stop_service_name').value,
            self.stop_callback,
        )
        self.pause_service = self.create_service(
            Trigger,
            self.get_parameter('pause_service_name').value,
            self.pause_callback,
        )
        self.resume_service = self.create_service(
            Trigger,
            self.get_parameter('resume_service_name').value,
            self.resume_callback,
        )
        self.toggle_pause_service = self.create_service(
            Trigger,
            self.get_parameter('toggle_pause_service_name').value,
            self.toggle_pause_callback,
        )

        decision_rate_hz = float(
            self.get_parameter('decision_rate_hz').value)
        timer_period = 1.0 / max(decision_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publish_mode()
        lane_input = 'stamped' if self.use_stamped_lane_command else 'legacy'
        self.get_logger().info(
            'Mission manager ready. Priority: '
            'PAUSED > RED/STOP > CONE > LANE/OVERTAKE. '
            f'lane_input={lane_input}. '
            f'scene_inputs={self.enable_scene_inputs}, '
            f'cone_mission={self.enable_cone_mission}. '
            f'cone_speed_rates='
            f'{self.cone_speed_accel_rate_per_s:.1f}/'
            f'{self.cone_speed_decel_rate_per_s:.1f}. '
            f'vesc_ready_required={self.require_vesc_ready}. '
            'lane_control=prewarmed. '
            f'Call {self.get_parameter("start_service_name").value} to drive.'
        )

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    @staticmethod
    def slew_speed(previous, target, elapsed_s, accel_rate, decel_rate):
        """Rate-limit a non-safety speed change in command units/second."""
        target = float(target)
        if previous is None:
            return target
        previous = float(previous)
        elapsed = max(0.0, min(float(elapsed_s), 0.25))
        if target > previous:
            return min(target, previous + max(0.0, accel_rate) * elapsed)
        if target < previous:
            return max(target, previous - max(0.0, decel_rate) * elapsed)
        return previous

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    @staticmethod
    def stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def timestamp_is_fresh(
            timestamp_ns, now_ns, timeout_s, future_tolerance_s=0.0):
        if timestamp_ns <= 0:
            return False
        age_s = (now_ns - timestamp_ns) * 1e-9
        return -future_tolerance_s <= age_s <= timeout_s

    def lane_command_callback(self, msg: Float32MultiArray):
        self.lane_command = self.read_motor_command(msg)
        self.publish_lane_event_if_safe()

    def stamped_lane_command_callback(self, msg: StampedMotorCommand):
        self.lane_command = TimedMotorCommand(
            angle=float(msg.angle),
            speed=float(msg.speed),
            received_ns=self.now_ns(),
            command_ns=self.stamp_to_ns(msg.command_stamp),
            source_ns=self.stamp_to_ns(msg.header.stamp),
            state_age_s=float(msg.state_age_s),
            is_stamped=True,
        )
        if not self.first_lane_command_logged:
            self.first_lane_command_logged = True
            self.log_startup_milestone('first fresh-candidate lane command')
        self.publish_lane_event_if_safe()

    def vesc_ready_callback(self, msg: Bool):
        was_ready = self.vesc_ready
        self.vesc_ready = bool(msg.data)
        self.vesc_ready_received_ns = self.now_ns()
        if self.vesc_ready != was_ready:
            state = 'READY' if self.vesc_ready else 'NOT_READY'
            self.get_logger().info(f'[STARTUP] VESC {state}')
            if self.vesc_ready:
                self.log_startup_milestone('VESC handshake ready')
        if was_ready and not self.vesc_ready and self.active:
            self.deactivate_for_safety('VESC became not ready')

    def cone_command_callback(self, msg: Float32MultiArray):
        self.cone_command = self.read_motor_command(msg)

    def read_motor_command(self, msg):
        if len(msg.data) < 2:
            return TimedMotorCommand(received_ns=self.now_ns())
        return TimedMotorCommand(
            angle=float(msg.data[0]),
            speed=float(msg.data[1]),
            received_ns=self.now_ns(),
        )

    def command_is_fresh(self, command, now_ns, require_source_stamp=False):
        if not self.timestamp_is_fresh(
                command.received_ns,
                now_ns,
                self.command_timeout_s,
                self.future_stamp_tolerance_s):
            return False

        if not require_source_stamp:
            return True

        if not command.is_stamped:
            return False
        if not self.timestamp_is_fresh(
                command.command_ns,
                now_ns,
                self.command_timeout_s,
                self.future_stamp_tolerance_s):
            return False
        return self.timestamp_is_fresh(
            command.source_ns,
            now_ns,
            self.lane_source_timeout_s,
            self.future_stamp_tolerance_s,
        )

    def log_startup_milestone(self, name):
        elapsed_s = time.monotonic() - self.startup_monotonic_s
        self.get_logger().info(f'[STARTUP +{elapsed_s:.3f}s] {name}')

    def start_readiness_failures(self, now_ns=None):
        now_ns = self.now_ns() if now_ns is None else now_ns
        failures = []
        if self.require_vesc_ready:
            ready_fresh = self.timestamp_is_fresh(
                self.vesc_ready_received_ns,
                now_ns,
                self.vesc_ready_timeout_s,
                self.future_stamp_tolerance_s,
            )
            if not self.vesc_ready:
                failures.append('VESC not ready')
            elif not ready_fresh:
                failures.append('VESC readiness stale')
        if not self.command_is_fresh(
                self.lane_command,
                now_ns,
                require_source_stamp=self.use_stamped_lane_command):
            failures.append('fresh lane command unavailable')
        return failures

    def vesc_is_ready_for_drive(self, now_ns=None):
        if not self.require_vesc_ready:
            return True
        now_ns = self.now_ns() if now_ns is None else now_ns
        return self.vesc_ready and self.timestamp_is_fresh(
            self.vesc_ready_received_ns,
            now_ns,
            self.vesc_ready_timeout_s,
            self.future_stamp_tolerance_s,
        )

    def deactivate_for_safety(self, reason):
        if not self.active:
            return
        self.active = False
        self.paused = False
        self.reset_run_state(clear_lane_command=True)
        self.override_pub.publish(String(data='reset'))
        self.publish_mode()
        self.publish_motor(0.0, 0.0)
        self.get_logger().error(
            f'Integrated drive deactivated: {reason}. '
            'Call /start_integrated_drive again after readiness recovers.')

    def detection_callback(self, msg: Detections):
        entry_count = 0
        bbox_prearm_count = 0
        for detection in msg.detections:
            if detection.class_name != self.cone_class_name:
                continue
            area = max(0.0, float(detection.xmax - detection.xmin)) * max(
                0.0,
                float(detection.ymax - detection.ymin),
            )
            if float(detection.confidence) < self.cone_conf_threshold:
                continue
            bottom_y = float(detection.ymax)
            if (
                area >= self.cone_prearm_min_bbox_area_px
                and bottom_y >= self.cone_prearm_min_bottom_y_px
            ):
                bbox_prearm_count += 1
            if (
                area >= self.cone_min_bbox_area_px
                and bottom_y >= self.cone_entry_min_bottom_y_px
            ):
                entry_count += 1
        self.cone_detection_count = entry_count
        self.cone_bbox_prearm_detection_count = bbox_prearm_count
        self.detection_received_ns = self.now_ns()
        self.update_bbox_prearm(bbox_prearm_count)

    def update_bbox_prearm(self, detection_count):
        if (
            not self.cone_prearm_enabled
            or not self.cone_bbox_prearm_enabled
            or not self.enable_cone_mission
            or self.mode != self.LANE_MODE
            or (self.cone_completed and not self.cone_allow_reentry)
        ):
            return

        matched = detection_count >= self.cone_bbox_prearm_min_detections
        self.cone_bbox_prearm_votes.append(matched)
        if matched:
            self.cone_bbox_prearm_miss_count = 0
        elif self.cone_bbox_prearm_latched:
            self.cone_bbox_prearm_miss_count += 1

        if (
            not self.cone_bbox_prearm_latched
            and sum(self.cone_bbox_prearm_votes)
            >= self.cone_prearm_min_matches
        ):
            self.cone_bbox_prearm_latched = True
            self.cone_bbox_prearm_miss_count = 0
        elif (
            self.cone_bbox_prearm_latched
            and self.cone_bbox_prearm_miss_count
            >= self.cone_bbox_prearm_clear_frames
        ):
            self.reset_bbox_prearm_state()

    def cluster_callback(self, msg: ClusterData):
        self.cone_cluster_count = len(msg.clusters)
        self.cluster_received_ns = self.now_ns()

    def entry_fusion_callback(self, msg: ClusterData):
        side_threshold = self.cone_entry_side_min_abs_y_m
        self.entry_fused_cluster_count = len(msg.clusters)
        self.entry_fused_left_count = sum(
            1 for point in msg.clusters if float(point.y) >= side_threshold
        )
        self.entry_fused_right_count = sum(
            1 for point in msg.clusters if float(point.y) <= -side_threshold
        )
        self.entry_fusion_received_ns = self.now_ns()

    def prearm_fusion_callback(self, msg: ClusterData):
        now_ns = self.now_ns()
        self.prearm_fused_cluster_count = len(msg.clusters)
        self.prearm_fusion_received_ns = now_ns
        matched = (
            self.cone_prearm_enabled
            and self.cone_prearm_use_fusion
            and self.prearm_fused_cluster_count
            >= self.cone_prearm_min_clusters
        )
        self.cone_prearm_votes.append(matched)
        if (
            sum(self.cone_prearm_votes)
            >= self.cone_prearm_min_matches
        ):
            self.cone_prearm_until_ns = now_ns + int(
                self.cone_prearm_hold_s * 1e9)

    def path_callback(self, msg: Path):
        self.cone_path_count = len(msg.poses)
        self.path_received_ns = self.now_ns()

    def lane_override_callback(self, msg: String):
        value = msg.data.strip().lower()
        if value in ('go_left', 'go_right', 'reset'):
            if self.mode == self.CONE_MODE and value != 'reset':
                self.lane_override = 'reset'
                self.override_pub.publish(String(data='reset'))
            else:
                self.lane_override = value
            self.publish_mode()

    def traffic_callback(self, msg: String):
        value = msg.data.strip().lower()
        if value in ('red', 'green', 'yellow', 'unknown'):
            self.traffic_state = value
            self.publish_mode()

    def start_callback(self, request, response):
        if self.active:
            response.success = False
            response.message = 'Integrated drive is already active.'
            return response
        self.request_lane_services()
        if self.active and not self.vesc_is_ready_for_drive():
            self.deactivate_for_safety('VESC readiness lost or stale')
        failures = self.start_readiness_failures()
        if failures:
            response.success = False
            response.message = 'Start rejected: ' + '; '.join(failures) + '.'
            self.get_logger().warn(response.message)
            return response
        self.active = True
        self.paused = False
        self.reset_run_state(clear_lane_command=False)
        self.transition_started_ns = self.now_ns()
        self.mode_entered_ns = self.transition_started_ns
        self.request_lane_services()
        self.publish_mode()
        response.success = True
        response.message = 'Integrated drive started in LANE mode.'
        self.get_logger().warn(response.message)
        return response

    def stop_callback(self, request, response):
        self.active = False
        self.paused = False
        self.reset_run_state(clear_lane_command=True)
        self.override_pub.publish(String(data='reset'))
        self.publish_mode()
        self.publish_motor(0.0, 0.0)
        response.success = True
        response.message = 'Integrated drive stopped.'
        self.get_logger().warn(response.message)
        return response

    def pause_callback(self, request, response):
        del request
        if not self.active:
            response.success = False
            response.message = 'Pause rejected: integrated drive is inactive.'
            return response
        if self.paused:
            response.success = True
            response.message = 'Integrated drive is already paused.'
            return response
        self.paused = True
        self.last_selected_speed = 0.0
        self.last_selected_publish_ns = self.now_ns()
        self.publish_mode()
        self.publish_motor(0.0, 0.0)
        response.success = True
        response.message = 'Integrated drive paused; mission state preserved.'
        self.get_logger().warn(response.message)
        return response

    def resume_callback(self, request, response):
        del request
        if not self.active:
            response.success = False
            response.message = 'Resume rejected: integrated drive is inactive.'
            return response
        if not self.paused:
            response.success = True
            response.message = 'Integrated drive is already running.'
            return response
        failures = self.start_readiness_failures()
        if failures:
            response.success = False
            response.message = 'Resume rejected: ' + '; '.join(failures) + '.'
            self.get_logger().warn(response.message)
            return response
        self.paused = False
        self.last_selected_speed = 0.0
        self.last_selected_publish_ns = self.now_ns()
        self.publish_mode()
        response.success = True
        response.message = (
            f'Integrated drive resumed in {self.effective_mode()} mode.')
        self.get_logger().warn(response.message)
        return response

    def toggle_pause_callback(self, request, response):
        if self.paused:
            return self.resume_callback(request, response)
        return self.pause_callback(request, response)

    def reset_run_state(self, clear_lane_command):
        self.mode = self.LANE_MODE
        self.lane_override = 'reset'
        if clear_lane_command:
            self.traffic_state = 'unknown'
        self.cone_command = TimedMotorCommand()
        self.last_selected_speed = None
        self.last_selected_publish_ns = 0
        if clear_lane_command:
            self.lane_command = TimedMotorCommand()
        self.cone_detection_count = 0
        self.cone_bbox_prearm_detection_count = 0
        self.cone_cluster_count = 0
        self.cone_path_count = 0
        self.entry_fused_cluster_count = 0
        self.entry_fused_left_count = 0
        self.entry_fused_right_count = 0
        self.prearm_fused_cluster_count = 0
        self.detection_received_ns = 0
        self.cluster_received_ns = 0
        self.path_received_ns = 0
        self.entry_fusion_received_ns = 0
        self.prearm_fusion_received_ns = 0
        self.cone_confirm_cycles = 0
        self.cone_completed = False
        self.last_cone_evidence_ns = 0
        self.cone_suspect_until_ns = 0
        self.cone_prearm_until_ns = 0
        self.cone_prearm_votes.clear()
        self.reset_bbox_prearm_state()
        self.last_entry_source = 'none'
        self.transition_started_ns = self.now_ns()
        self.mode_entered_ns = self.transition_started_ns

    def request_lane_services(self):
        lane_ready = self.lane_start_client.service_is_ready()
        if not self.lane_start_requested and lane_ready:
            self.lane_start_client.call_async(Trigger.Request())
            self.lane_start_requested = True
        stanley_ready = self.stanley_start_client.service_is_ready()
        if not self.stanley_start_requested and stanley_ready:
            self.stanley_start_client.call_async(Trigger.Request())
            self.stanley_start_requested = True

    def timer_callback(self):
        # Warm lane perception/control while the mission manager continues to
        # gate /xycar_motor at zero. This keeps staging safe but makes the
        # first fresh lane command available before the start service call.
        self.request_lane_services()
        if (
            self.active
            and not self.paused
            and not self.vesc_is_ready_for_drive()
        ):
            self.deactivate_for_safety('VESC readiness lost or stale')
        failures = self.start_readiness_failures()
        readiness_summary = ', '.join(failures) if failures else 'READY'
        if readiness_summary != self.last_readiness_summary:
            self.get_logger().info(
                f'[INTEGRATED READY] {readiness_summary}')
            self.last_readiness_summary = readiness_summary
        if not failures and not self.integrated_ready_logged:
            self.integrated_ready_logged = True
            self.log_startup_milestone(
                'integrated drive ready for /start_integrated_drive')
        if self.active and not self.paused:
            self.update_mode()
        self.publish_selected_command()
        self.publish_status()

    def has_partial_fusion_evidence(self, now_ns):
        return all([
            self.cone_entry_use_fusion_gate,
            self.is_entry_fusion_fresh(now_ns),
            (
                self.entry_fused_cluster_count
                >= self.cone_suspect_min_fused_clusters
            ),
            self.is_fresh(self.path_received_ns, now_ns),
            self.cone_path_count > 0,
        ])

    def lane_event_publish_is_safe(self, now_ns):
        if not self.active or self.paused or self.mode != self.LANE_MODE:
            return False
        if self.respect_traffic_light and self.traffic_state == 'red':
            return False
        if not self.vesc_is_ready_for_drive(now_ns):
            return False
        if not self.command_is_fresh(
                self.lane_command,
                now_ns,
                require_source_stamp=self.use_stamped_lane_command):
            return False
        if not self.enable_cone_mission:
            return True
        if self.cone_completed and not self.cone_allow_reentry:
            return True
        if self.cone_confirm_cycles > 0:
            return False
        if self.cone_entry_evidence_source(now_ns) != 'none':
            return False
        return not self.has_partial_fusion_evidence(now_ns)

    def publish_lane_event_if_safe(self):
        if not self.event_driven_lane_output:
            return False
        now_ns = self.now_ns()
        if not self.lane_event_publish_is_safe(now_ns):
            return False
        self.publish_selected_command()
        return True

    def update_mode(self):
        now_ns = self.now_ns()

        if self.mode == self.LANE_MODE:
            if not self.enable_cone_mission:
                self.cone_confirm_cycles = 0
                self.cone_suspect_until_ns = 0
                self.reset_cone_prearm_state()
                return
            if self.cone_completed and not self.cone_allow_reentry:
                self.cone_confirm_cycles = 0
                self.cone_suspect_until_ns = 0
                self.reset_cone_prearm_state()
                return
            self.update_cone_suspect(now_ns)
            entry_source = self.cone_entry_evidence_source(now_ns)
            if entry_source != 'none':
                self.cone_confirm_cycles += 1
                self.last_entry_source = entry_source
            else:
                self.cone_confirm_cycles = 0
            if self.cone_confirm_cycles >= self.cone_enter_confirm_cycles:
                self.last_cone_evidence_ns = now_ns
                self.switch_mode(self.CONE_MODE)
        else:
            self.cone_suspect_until_ns = 0
            self.reset_cone_prearm_state()
            if self.has_cone_drive_evidence(now_ns):
                self.last_cone_evidence_ns = now_ns
            mode_age_s = (now_ns - self.mode_entered_ns) * 1e-9
            age_s = (now_ns - self.last_cone_evidence_ns) * 1e-9
            if (
                mode_age_s >= self.cone_min_mode_duration_s
                and age_s > self.cone_exit_hold_s
            ):
                self.switch_mode(self.LANE_MODE)

    def cone_entry_evidence_source(self, now_ns):
        if not self.enable_cone_mission:
            return 'none'
        fusion_ready = self.has_fusion_entry_evidence(now_ns)
        lidar_ready = (
            self.cone_entry_allow_lidar_fallback
            and self.has_lidar_entry_evidence(now_ns)
        )
        if fusion_ready and lidar_ready:
            return 'fusion+lidar'
        if fusion_ready:
            return 'fusion'
        if lidar_ready:
            return 'lidar'
        return 'none'

    def has_lidar_entry_evidence(self, now_ns):
        checks = [
            self.is_fresh(self.cluster_received_ns, now_ns),
            self.is_fresh(self.path_received_ns, now_ns),
            self.cone_cluster_count >= self.cone_entry_min_clusters,
            self.cone_path_count >= self.cone_entry_min_path_points,
        ]
        if self.enable_scene_inputs:
            # DBSCAN can cluster walls, vehicle parts, and other nearby
            # objects into a plausible corridor.  When the scene pipeline is
            # available, do not let that LiDAR-only false positive consume the
            # one-shot cone mission before the real course.  Fusion may still
            # be unavailable because of projection/calibration, so require
            # fresh qualifying YOLO cones rather than a successful projection.
            checks.extend([
                self.is_fresh(self.detection_received_ns, now_ns),
                (
                    self.cone_detection_count
                    >= self.cone_entry_min_detections
                ),
            ])
        return all(checks)

    def has_fusion_entry_evidence(self, now_ns):
        if not self.cone_entry_use_fusion_gate:
            return False
        checks = [
            self.is_entry_fusion_fresh(now_ns),
            self.is_fresh(self.path_received_ns, now_ns),
            (
                self.entry_fused_cluster_count
                >= self.cone_entry_min_fused_clusters
            ),
            self.cone_path_count >= self.cone_entry_min_path_points,
        ]
        if self.cone_entry_require_both_sides:
            checks.extend([
                self.entry_fused_left_count > 0,
                self.entry_fused_right_count > 0,
            ])
        return all(checks)

    def has_cone_drive_evidence(self, now_ns):
        return all([
            self.is_fresh(self.cluster_received_ns, now_ns),
            self.is_fresh(self.path_received_ns, now_ns),
            self.cone_cluster_count > 0,
            self.cone_path_count > 0,
        ])

    def update_cone_suspect(self, now_ns):
        if not self.enable_cone_mission:
            self.cone_suspect_until_ns = 0
            return
        partial_fusion = self.has_partial_fusion_evidence(now_ns)
        if partial_fusion:
            self.cone_suspect_until_ns = now_ns + int(
                max(0.0, self.cone_suspect_hold_s) * 1e9
            )

    def cone_suspect_active(self, now_ns):
        return (
            self.mode == self.LANE_MODE
            and self.cone_suspect_speed_cap > 0.0
            and now_ns <= self.cone_suspect_until_ns
        )

    def cone_prearm_active(self, now_ns):
        bbox_active = (
            self.cone_bbox_prearm_enabled
            and self.cone_bbox_prearm_latched
            and self.is_fresh(self.detection_received_ns, now_ns)
        )
        fusion_active = (
            self.cone_prearm_use_fusion
            and now_ns <= self.cone_prearm_until_ns
        )
        return (
            self.cone_prearm_enabled
            and self.mode == self.LANE_MODE
            and self.cone_prearm_speed_cap > 0.0
            and (bbox_active or fusion_active)
        )

    def cone_prearm_source(self, now_ns):
        bbox_active = (
            self.cone_bbox_prearm_enabled
            and self.cone_bbox_prearm_latched
            and self.is_fresh(self.detection_received_ns, now_ns)
        )
        fusion_active = (
            self.cone_prearm_use_fusion
            and now_ns <= self.cone_prearm_until_ns
        )
        if bbox_active and fusion_active:
            return 'bbox+fusion'
        if bbox_active:
            return 'bbox'
        if fusion_active:
            return 'fusion'
        return 'none'

    def reset_bbox_prearm_state(self):
        self.cone_bbox_prearm_votes.clear()
        self.cone_bbox_prearm_miss_count = 0
        self.cone_bbox_prearm_latched = False

    def reset_cone_prearm_state(self):
        self.cone_prearm_until_ns = 0
        self.cone_prearm_votes.clear()
        self.reset_bbox_prearm_state()

    def is_entry_fusion_fresh(self, now_ns):
        if self.entry_fusion_received_ns <= 0:
            return False
        age_s = (now_ns - self.entry_fusion_received_ns) * 1e-9
        return age_s <= self.cone_entry_fusion_timeout_s

    def is_fresh(self, received_ns, now_ns):
        if received_ns <= 0:
            return False
        return (now_ns - received_ns) * 1e-9 <= self.sensor_timeout_s

    def switch_mode(self, new_mode):
        if new_mode == self.mode:
            return
        previous = self.mode
        self.mode = new_mode
        if previous == self.CONE_MODE and new_mode == self.LANE_MODE:
            self.cone_completed = True
        self.transition_started_ns = self.now_ns()
        self.mode_entered_ns = self.transition_started_ns
        self.cone_confirm_cycles = 0
        if new_mode == self.CONE_MODE:
            self.cone_suspect_until_ns = 0
            self.reset_cone_prearm_state()
            self.lane_override = 'reset'
            self.override_pub.publish(String(data='reset'))
        self.publish_mode()
        self.get_logger().warn(f'MISSION MODE: {previous} -> {new_mode}')

    def publish_mode(self):
        effective_mode = self.effective_mode()
        if effective_mode == self.last_published_mode:
            return
        self.mode_pub.publish(String(data=effective_mode))
        self.last_published_mode = effective_mode

    def effective_mode(self):
        if not self.active:
            return self.STOP_MODE
        if self.paused:
            return self.PAUSED_MODE
        if self.respect_traffic_light and self.traffic_state == 'red':
            return self.STOP_MODE
        if self.mode == self.CONE_MODE:
            return self.CONE_MODE
        if self.lane_override in ('go_left', 'go_right'):
            return self.OVERTAKE_MODE
        return self.LANE_MODE

    def publish_selected_command(self):
        if not self.active or self.paused:
            self.last_selected_speed = 0.0
            self.last_selected_publish_ns = self.now_ns()
            self.publish_motor(0.0, 0.0)
            return
        if self.respect_traffic_light and self.traffic_state == 'red':
            self.last_selected_speed = 0.0
            self.last_selected_publish_ns = self.now_ns()
            self.publish_motor(0.0, 0.0)
            return

        now_ns = self.now_ns()
        if self.mode == self.CONE_MODE:
            command = self.cone_command
            require_source_stamp = False
        else:
            command = self.lane_command
            require_source_stamp = self.use_stamped_lane_command
        if not self.command_is_fresh(
                command,
                now_ns,
                require_source_stamp=require_source_stamp):
            self.last_selected_speed = 0.0
            self.last_selected_publish_ns = now_ns
            self.publish_motor(0.0, 0.0)
            return
        speed = command.speed
        if self.cone_prearm_active(now_ns) and speed > 0.0:
            speed = min(speed, self.cone_prearm_speed_cap)
        if self.cone_suspect_active(now_ns) and speed > 0.0:
            speed = min(speed, self.cone_suspect_speed_cap)
        if self.mode == self.CONE_MODE:
            elapsed_s = (
                (now_ns - self.last_selected_publish_ns) * 1e-9
                if self.last_selected_publish_ns > 0 else 0.0
            )
            speed = self.slew_speed(
                self.last_selected_speed,
                speed,
                elapsed_s,
                self.cone_speed_accel_rate_per_s,
                self.cone_speed_decel_rate_per_s,
            )
        self.last_selected_speed = float(speed)
        self.last_selected_publish_ns = now_ns
        self.publish_motor(command.angle, speed)

    def publish_motor(self, angle, speed):
        msg = Float32MultiArray()
        corrected_angle = max(
            -100.0,
            min(100.0, float(angle) + self.steering_trim),
        )
        msg.data = [corrected_angle, float(speed)]
        self.motor_pub.publish(msg)

    def publish_status(self):
        now_ns = self.now_ns()
        if (
            self.last_status_publish_ns > 0
            and now_ns - self.last_status_publish_ns
            < self.status_publish_period_ns
        ):
            return
        self.last_status_publish_ns = now_ns
        displayed_mode = self.effective_mode()
        entry_source = self.cone_entry_evidence_source(now_ns)
        prearm = self.cone_prearm_active(now_ns)
        prearm_source = self.cone_prearm_source(now_ns)
        suspect = self.cone_suspect_active(now_ns)
        status = (
            f'active={self.active} paused={self.paused} '
            f'mode={displayed_mode} '
            f'navigation={self.mode} '
            f'traffic={self.traffic_state} '
            f'cone_source={self.cone_perception_mode} '
            f'cone_completed={self.cone_completed} '
            f'cones(yolo/lidar/path)={self.cone_detection_count}/'
            f'{self.cone_cluster_count}/{self.cone_path_count} '
            f'entry_fusion={self.entry_fused_cluster_count}'
            f'(L{self.entry_fused_left_count}/'
            f'R{self.entry_fused_right_count}) '
            f'prearm_fusion={self.prearm_fused_cluster_count} '
            f'prearm_fusion_votes={sum(self.cone_prearm_votes)}/'
            f'{self.cone_prearm_window} '
            f'prearm_bbox={self.cone_bbox_prearm_detection_count} '
            f'prearm_bbox_votes={sum(self.cone_bbox_prearm_votes)}/'
            f'{self.cone_prearm_window} '
            f'prearm_bbox_misses={self.cone_bbox_prearm_miss_count}/'
            f'{self.cone_bbox_prearm_clear_frames} '
            f'entry_gate={entry_source} '
            f'prearm={prearm}({prearm_source}) '
            f'suspect={suspect} '
            f'lane_override={self.lane_override}'
        )
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Jazzy can raise a pybind RuntimeError while taking a subscription
        # after SIGINT has already invalidated the ROS context.  It is a
        # shutdown race, not a node failure; preserve real runtime errors.
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.publish_motor(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
