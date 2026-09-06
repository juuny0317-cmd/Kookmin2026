from mission_cone_drive.mission_manager_node import (
    MissionManagerNode,
    TimedMotorCommand,
)
from custom_interfaces.msg import Detection, Detections
import rclpy


def install_fresh_lane_command(node, now_ns):
    node.require_vesc_ready = False
    node.lane_command = TimedMotorCommand(
        angle=12.0,
        speed=8.0,
        received_ns=now_ns,
        command_ns=now_ns,
        source_ns=now_ns,
        state_age_s=0.0,
        is_stamped=True,
    )
    node.transition_started_ns = now_ns - int(1.0e9)


def test_event_lane_gate_preserves_stop_and_cone_priority():
    rclpy.init()
    node = MissionManagerNode()
    try:
        now_ns = node.now_ns()
        install_fresh_lane_command(node, now_ns)
        node.active = True
        node.traffic_state = 'green'
        node.mode = node.LANE_MODE
        node.cone_confirm_cycles = 0
        node.enable_cone_mission = False
        assert node.lane_event_publish_is_safe(now_ns)

        node.traffic_state = 'red'
        assert not node.lane_event_publish_is_safe(now_ns)
        node.traffic_state = 'green'

        node.mode = node.CONE_MODE
        assert not node.lane_event_publish_is_safe(now_ns)
        node.mode = node.LANE_MODE

        node.enable_cone_mission = True
        node.cone_confirm_cycles = 1
        assert not node.lane_event_publish_is_safe(now_ns)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_event_lane_gate_rejects_full_and_partial_cone_evidence():
    rclpy.init()
    node = MissionManagerNode()
    try:
        now_ns = node.now_ns()
        install_fresh_lane_command(node, now_ns)
        node.active = True
        node.traffic_state = 'green'
        node.mode = node.LANE_MODE
        node.cone_confirm_cycles = 0
        node.enable_cone_mission = True

        node.cluster_received_ns = now_ns
        node.path_received_ns = now_ns
        node.cone_cluster_count = max(1, node.cone_entry_min_clusters)
        node.cone_path_count = max(1, node.cone_entry_min_path_points)
        # LiDAR alone is no longer enough when scene perception is enabled.
        assert node.lane_event_publish_is_safe(now_ns)

        detections = Detections()
        for index in range(node.cone_entry_min_detections):
            detection = Detection()
            detection.class_name = 'cone'
            detection.confidence = 0.9
            detection.xmin = 100 + index * 50
            detection.ymin = 240
            detection.xmax = 140 + index * 50
            detection.ymax = 330
            detections.detections.append(detection)
        node.detection_callback(detections)
        assert not node.lane_event_publish_is_safe(now_ns)

        node.cluster_received_ns = 0
        node.cone_cluster_count = 0
        node.entry_fusion_received_ns = now_ns
        node.entry_fused_cluster_count = max(
            1, node.cone_suspect_min_fused_clusters)
        assert node.has_partial_fusion_evidence(now_ns)
        assert not node.lane_event_publish_is_safe(now_ns)

        node.cone_completed = True
        node.cone_allow_reentry = False
        assert node.lane_event_publish_is_safe(now_ns)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_event_lane_callback_uses_existing_arbitration_publish_method():
    rclpy.init()
    node = MissionManagerNode()
    try:
        now_ns = node.now_ns()
        install_fresh_lane_command(node, now_ns)
        node.active = True
        node.enable_cone_mission = False
        node.event_driven_lane_output = True
        calls = []
        node.publish_selected_command = lambda **kwargs: calls.append(kwargs)

        assert node.publish_lane_event_if_safe()
        assert len(calls) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
