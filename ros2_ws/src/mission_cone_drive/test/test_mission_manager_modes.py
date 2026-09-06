from custom_interfaces.msg import ClusterData, Detection, Detections
from geometry_msgs.msg import Point, PoseStamped
from mission_cone_drive.mission_manager_node import (
    MissionManagerNode,
    TimedMotorCommand,
)
from nav_msgs.msg import Path
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def make_cone_detections(count=2):
    message = Detections()
    for index in range(count):
        detection = Detection()
        detection.class_name = 'cone'
        detection.confidence = 0.9
        detection.xmin = 100 + index * 80
        detection.ymin = 250
        detection.xmax = 140 + index * 80
        detection.ymax = 330
        message.detections.append(detection)
    return message


def make_clusters(count=2):
    message = ClusterData()
    for index in range(count):
        message.clusters.append(
            Point(x=0.5 + index * 0.1, y=(-1) ** index * 0.3))
    return message


def make_one_side_clusters(count=2, side=1.0):
    message = ClusterData()
    for index in range(count):
        message.clusters.append(
            Point(x=0.5 + index * 0.1, y=side * (0.25 + index * 0.05)))
    return message


def make_path(count=3):
    message = Path()
    for index in range(count):
        pose = PoseStamped()
        pose.pose.position.x = 0.3 + index * 0.2
        message.poses.append(pose)
    return message


def test_mode_priority_and_cone_hysteresis():
    rclpy.init()
    node = MissionManagerNode()
    try:
        assert node.effective_mode() == node.STOP_MODE

        node.active = True
        assert node.effective_mode() == node.LANE_MODE

        node.lane_override = 'go_left'
        assert node.effective_mode() == node.OVERTAKE_MODE

        node.traffic_state = 'red'
        assert node.effective_mode() == node.STOP_MODE
        node.traffic_state = 'green'

        node.cone_enter_confirm_cycles = 3
        for _ in range(3):
            node.detection_callback(make_cone_detections())
            node.cluster_callback(make_clusters())
            node.path_callback(make_path())
            node.update_mode()

        assert node.mode == node.CONE_MODE
        assert node.effective_mode() == node.CONE_MODE
        assert node.lane_override == 'reset'

        now_ns = node.now_ns()
        node.cone_min_mode_duration_s = 1.0
        node.cone_exit_hold_s = 0.5
        node.mode_entered_ns = now_ns - int(0.2e9)
        node.last_cone_evidence_ns = now_ns - int(2.0e9)
        node.detection_received_ns = 0
        node.cluster_received_ns = 0
        node.path_received_ns = 0
        node.update_mode()
        assert node.mode == node.CONE_MODE

        node.mode_entered_ns = node.now_ns() - int(2.0e9)
        node.update_mode()
        assert node.mode == node.LANE_MODE

        node.cone_allow_reentry = False
        node.cone_completed = True
        for _ in range(5):
            node.detection_callback(make_cone_detections())
            node.cluster_callback(make_clusters())
            node.path_callback(make_path())
            node.update_mode()
        assert node.mode == node.LANE_MODE
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_start_under_red_holds_then_green_releases_lane_command():
    """Arming on RED must stay stopped and release without a second start."""
    rclpy.init()
    node = MissionManagerNode()
    try:
        now_ns = node.now_ns()
        node.require_vesc_ready = False
        node.traffic_callback(String(data='red'))
        node.lane_command = TimedMotorCommand(
            angle=8.0,
            speed=12.0,
            received_ns=now_ns,
            command_ns=now_ns,
            source_ns=now_ns,
            state_age_s=0.0,
            is_stamped=True,
        )
        published = []
        node.publish_motor = lambda angle, speed: published.append(
            (angle, speed))

        response = node.start_callback(
            Trigger.Request(), Trigger.Response())

        assert response.success
        assert node.active
        assert node.traffic_state == 'red'
        assert node.effective_mode() == node.STOP_MODE
        node.publish_selected_command()
        assert published[-1] == (0.0, 0.0)

        node.traffic_callback(String(data='green'))
        assert node.active
        assert node.effective_mode() == node.LANE_MODE
        node.publish_selected_command()
        assert published[-1] == (8.0, 12.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_entry_fusion_requires_both_sides_and_is_ignored_after_entry():
    rclpy.init()
    node = MissionManagerNode()
    try:
        node.active = True
        node.cone_entry_allow_lidar_fallback = False
        node.cone_entry_use_fusion_gate = True
        node.cone_entry_require_both_sides = True
        node.cone_entry_min_fused_clusters = 2
        node.cone_entry_min_path_points = 1
        node.cone_enter_confirm_cycles = 2
        node.cone_suspect_speed_cap = 4.0

        for _ in range(3):
            node.entry_fusion_callback(make_one_side_clusters())
            node.path_callback(make_path())
            node.update_mode()
        assert node.mode == node.LANE_MODE
        assert node.cone_suspect_active(node.now_ns())

        for _ in range(2):
            node.entry_fusion_callback(make_clusters())
            node.path_callback(make_path())
            node.update_mode()
        assert node.mode == node.CONE_MODE
        assert node.last_entry_source == 'fusion'

        node.cone_min_mode_duration_s = 0.0
        node.cone_exit_hold_s = 0.5
        node.entry_fusion_received_ns = 0
        node.cluster_callback(make_clusters())
        node.path_callback(make_path())
        node.last_cone_evidence_ns = node.now_ns() - int(2.0e9)
        node.update_mode()
        assert node.mode == node.CONE_MODE
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_lidar_fallback_requires_fresh_yolo_cones_with_scene_inputs():
    rclpy.init()
    node = MissionManagerNode()
    try:
        node.enable_scene_inputs = True
        node.cone_entry_allow_lidar_fallback = True
        node.cone_entry_min_clusters = 2
        node.cone_entry_min_path_points = 1
        node.cone_entry_min_detections = 2

        node.cluster_callback(make_clusters())
        node.path_callback(make_path())
        now_ns = node.now_ns()
        assert not node.has_lidar_entry_evidence(now_ns)

        node.detection_callback(make_cone_detections(count=1))
        assert not node.has_lidar_entry_evidence(node.now_ns())

        node.detection_callback(make_cone_detections(count=2))
        assert node.has_lidar_entry_evidence(node.now_ns())

        node.detection_received_ns = (
            node.now_ns() - int((node.sensor_timeout_s + 0.1) * 1e9)
        )
        assert not node.has_lidar_entry_evidence(node.now_ns())
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_bbox_prearm_caps_speed_until_three_missing_frames():
    rclpy.init()
    node = MissionManagerNode()
    try:
        now_ns = node.now_ns()
        node.active = True
        node.traffic_state = 'green'
        node.mode = node.LANE_MODE
        node.lane_override = 'go_left'
        node.cone_entry_allow_lidar_fallback = False
        node.lane_command = TimedMotorCommand(
            angle=12.0,
            speed=10.0,
            received_ns=now_ns,
            command_ns=now_ns,
            source_ns=now_ns,
            is_stamped=True,
        )
        published = []
        node.publish_motor = lambda angle, speed: published.append(
            (angle, speed))

        # Fusion is intentionally ignored for pre-entry slowing. The strict
        # final cone-entry fusion gate remains independent.
        node.prearm_fusion_callback(make_clusters(count=1))
        node.prearm_fusion_callback(make_clusters(count=1))
        assert not node.cone_prearm_active(node.now_ns())

        node.entry_fusion_callback(make_one_side_clusters(count=1))
        node.path_callback(make_path(count=1))
        node.update_mode()
        assert not node.cone_suspect_active(node.now_ns())
        node.publish_selected_command()
        assert published[-1] == (12.0, 10.0)

        node.detection_callback(make_cone_detections(count=1))
        assert not node.cone_prearm_active(node.now_ns())
        node.detection_callback(make_cone_detections(count=1))
        assert node.cone_prearm_active(node.now_ns())
        node.publish_selected_command()
        assert published[-1] == (12.0, 4.0)
        assert node.mode == node.LANE_MODE
        assert node.effective_mode() == node.OVERTAKE_MODE

        for _ in range(2):
            node.detection_callback(Detections())
            assert node.cone_prearm_active(node.now_ns())
        node.detection_callback(Detections())
        assert not node.cone_prearm_active(node.now_ns())
        node.publish_selected_command()
        assert published[-1] == (12.0, 10.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_completed_cone_mission_can_reenter_when_enabled():
    rclpy.init()
    node = MissionManagerNode()
    try:
        node.active = True
        node.cone_allow_reentry = True
        node.cone_enter_confirm_cycles = 2
        node.switch_mode(node.CONE_MODE)
        node.switch_mode(node.LANE_MODE)
        assert node.cone_completed

        for _ in range(2):
            node.detection_callback(make_cone_detections())
            node.cluster_callback(make_clusters())
            node.path_callback(make_path())
            node.update_mode()

        assert node.mode == node.CONE_MODE
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cone_entry_and_exit_publish_fresh_commands_without_stop_frame():
    rclpy.init()
    node = MissionManagerNode()
    try:
        now_ns = node.now_ns()
        node.active = True
        node.traffic_state = 'green'
        node.cone_command = TimedMotorCommand(
            angle=-20.0,
            speed=6.0,
            received_ns=now_ns,
        )
        node.lane_command = TimedMotorCommand(
            angle=10.0,
            speed=8.0,
            received_ns=now_ns,
            command_ns=now_ns,
            source_ns=now_ns,
            is_stamped=True,
        )
        published = []
        node.publish_motor = lambda angle, speed: published.append(
            (angle, speed))

        node.switch_mode(node.CONE_MODE)
        node.publish_selected_command()
        assert published[-1] == (-20.0, 6.0)

        node.switch_mode(node.LANE_MODE)
        node.publish_selected_command()
        assert published[-1] == (10.0, 8.0)
        assert all(speed != 0.0 for _, speed in published)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_cone_mode_uses_dedicated_accel_and_decel_rates():
    rclpy.init()
    node = MissionManagerNode()
    try:
        clock_ns = [node.now_ns()]
        node.now_ns = lambda: clock_ns[0]
        node.active = True
        node.traffic_state = 'green'
        node.cone_speed_accel_rate_per_s = 4.0
        node.cone_speed_decel_rate_per_s = 20.0
        node.lane_command = TimedMotorCommand(
            angle=0.0,
            speed=20.0,
            received_ns=clock_ns[0],
            command_ns=clock_ns[0],
            source_ns=clock_ns[0],
            is_stamped=True,
        )
        node.cone_command = TimedMotorCommand(
            angle=-10.0,
            speed=8.0,
            received_ns=clock_ns[0],
        )
        published = []
        node.publish_motor = lambda angle, speed: published.append(
            (angle, speed))

        node.mode = node.LANE_MODE
        node.publish_selected_command()
        assert published[-1] == (0.0, 20.0)

        clock_ns[0] += int(0.1e9)
        node.mode = node.CONE_MODE
        node.publish_selected_command()
        assert published[-1] == (-10.0, 18.0)

        clock_ns[0] += int(0.1e9)
        node.cone_command.received_ns = clock_ns[0]
        node.cone_command.speed = 20.0
        node.publish_selected_command()
        assert published[-1] == (-10.0, 18.4)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pause_stops_output_and_resume_preserves_mission_state():
    rclpy.init()
    node = MissionManagerNode()
    try:
        now_ns = node.now_ns()
        node.require_vesc_ready = False
        node.active = True
        node.mode = node.CONE_MODE
        node.cone_completed = True
        node.last_entry_source = 'fusion'
        published = []
        node.publish_motor = lambda angle, speed: published.append(
            (angle, speed))

        response = node.pause_callback(
            Trigger.Request(), Trigger.Response())

        assert response.success
        assert node.active
        assert node.paused
        assert node.effective_mode() == node.PAUSED_MODE
        assert published[-1] == (0.0, 0.0)
        assert node.mode == node.CONE_MODE
        assert node.cone_completed
        assert node.last_entry_source == 'fusion'

        node.publish_selected_command()
        assert published[-1] == (0.0, 0.0)

        response = node.resume_callback(
            Trigger.Request(), Trigger.Response())
        assert not response.success
        assert node.paused

        node.lane_command = TimedMotorCommand(
            angle=4.0,
            speed=8.0,
            received_ns=now_ns,
            command_ns=now_ns,
            source_ns=now_ns,
            is_stamped=True,
        )
        response = node.resume_callback(
            Trigger.Request(), Trigger.Response())
        assert response.success
        assert node.active
        assert not node.paused
        assert node.effective_mode() == node.CONE_MODE
        assert node.mode == node.CONE_MODE
        assert node.cone_completed
        assert node.last_entry_source == 'fusion'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_toggle_pause_is_safe_while_inactive_and_stop_clears_pause():
    rclpy.init()
    node = MissionManagerNode()
    try:
        response = node.toggle_pause_callback(
            Trigger.Request(), Trigger.Response())
        assert not response.success
        assert not node.active
        assert not node.paused

        now_ns = node.now_ns()
        node.require_vesc_ready = False
        node.active = True
        node.lane_command = TimedMotorCommand(
            angle=0.0,
            speed=5.0,
            received_ns=now_ns,
            command_ns=now_ns,
            source_ns=now_ns,
            is_stamped=True,
        )
        response = node.toggle_pause_callback(
            Trigger.Request(), Trigger.Response())
        assert response.success
        assert node.paused

        response = node.toggle_pause_callback(
            Trigger.Request(), Trigger.Response())
        assert response.success
        assert not node.paused

        node.pause_callback(Trigger.Request(), Trigger.Response())
        node.stop_callback(Trigger.Request(), Trigger.Response())
        assert not node.active
        assert not node.paused
        assert node.effective_mode() == node.STOP_MODE
    finally:
        node.destroy_node()
        rclpy.shutdown()
