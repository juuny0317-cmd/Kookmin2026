from cam.traffic_light_node import TrafficLightNode
from custom_interfaces.msg import Detection, Detections
import rclpy
from std_msgs.msg import String


def make_detection(class_name, confidence):
    detection = Detection()
    detection.class_name = class_name
    detection.confidence = confidence
    detection.xmin = 10
    detection.ymin = 20
    detection.xmax = 30
    detection.ymax = 40
    return detection


def test_red_has_priority_over_higher_confidence_green():
    rclpy.init()
    node = TrafficLightNode()
    try:
        detections = Detections()
        detections.detections = [
            make_detection('green', 0.95),
            make_detection('red', 0.40),
        ]

        raw_state, _ = node.select_direct_state(detections)

        assert raw_state == 'red'
        assert node.update_direct_state(raw_state) == 'red'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_left_is_published_as_green_for_existing_drive_nodes():
    rclpy.init()
    node = TrafficLightNode()
    try:
        assert node.normalize_direct_state('left') == 'green'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_low_confidence_direct_state_is_ignored():
    rclpy.init()
    node = TrafficLightNode()
    try:
        detections = Detections()
        detections.detections = [make_detection('red', 0.10)]

        assert node.select_direct_state(detections) is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_each_direct_class_uses_its_own_threshold():
    rclpy.init()
    node = TrafficLightNode()
    try:
        detections = Detections()
        detections.detections = [
            make_detection('red', 0.21),
            make_detection('green', 0.34),
            make_detection('left', 0.49),
        ]

        raw_state, _ = node.select_direct_state(detections)

        assert raw_state == 'red'

        detections.detections = [make_detection('left', 0.49)]
        assert node.select_direct_state(detections) is None

        detections.detections = [make_detection('left', 0.90)]
        raw_state, _ = node.select_direct_state(detections)
        assert raw_state == 'left'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_red_latch_ignores_unknown_until_stable_green():
    rclpy.init()
    node = TrafficLightNode()
    try:
        assert node.update_direct_state('red') == 'red'
        assert node.update_direct_state('red') == 'red'
        assert node.red_latch_active
        assert node.update_state('unknown') == 'red'
        assert node.update_state('unknown') == 'red'
        node.last_red_sample_ns = (
            node.get_clock().now().nanoseconds - 1_000_000_000
        )
        assert node.update_state('green') == 'red'
        assert node.update_state('green') == 'green'
        assert not node.red_latch_active
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_single_direct_red_uses_timed_hold_but_does_not_latch():
    rclpy.init()
    node = TrafficLightNode()
    try:
        assert node.update_direct_state('red') == 'red'
        assert not node.red_latch_active
        node.last_red_sample_ns = (
            node.get_clock().now().nanoseconds - 1_000_000_000
        )
        assert node.update_state('unknown') == 'red'
        assert node.update_state('unknown') == 'unknown'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_red_stop_feedback_does_not_clear_red_state():
    rclpy.init()
    node = TrafficLightNode()
    try:
        published = []
        node.publish_state = lambda state, raw_state=None: published.append(
            (state, raw_state))

        assert node.update_direct_state('red') == 'red'
        node.mission_mode_callback(String(data='STOP'))

        assert node.current_state == 'red'
        assert node.last_red_sample_ns > 0
        assert published == []

        assert node.update_direct_state('red') == 'red'
        assert node.red_latch_active
        node.mission_mode_callback(String(data='STOP'))

        assert node.current_state == 'red'
        assert node.red_latch_active
        assert published == []
    finally:
        node.destroy_node()
        rclpy.shutdown()
