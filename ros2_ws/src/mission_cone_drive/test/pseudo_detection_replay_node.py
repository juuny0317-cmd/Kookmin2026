#!/usr/bin/env python3

import bisect
import re
from pathlib import Path

import rclpy
from custom_interfaces.msg import Detection, Detections
from rclpy.node import Node
from sensor_msgs.msg import Image


TIMESTAMP_PATTERN = re.compile(r'_(\d{19})_jpg')


class PseudoDetectionReplayNode(Node):
    """Replay YOLO-format labels in step with a rosbag image stream."""

    def __init__(self):
        super().__init__('pseudo_detection_replay_node')
        self.declare_parameter('dataset_root', '')
        self.declare_parameter('run_prefix', 'cone_wasd_01')
        self.declare_parameter('class_id', 4)
        self.declare_parameter('class_name', 'cone')
        self.declare_parameter('max_time_delta_s', 0.30)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('detections_topic', '/yolo_detections')

        self.class_id = int(self.get_parameter('class_id').value)
        self.class_name = str(self.get_parameter('class_name').value)
        self.max_delta_ns = int(
            float(self.get_parameter('max_time_delta_s').value) * 1e9)
        self.label_timestamps, self.labels = self.load_labels(
            Path(str(self.get_parameter('dataset_root').value)),
            str(self.get_parameter('run_prefix').value),
        )
        if not self.label_timestamps:
            raise RuntimeError('No matching pseudo-label files were found.')

        self.label_origin_ns = self.label_timestamps[0]
        self.image_origin_ns = None
        self.publisher = self.create_publisher(
            Detections,
            str(self.get_parameter('detections_topic').value),
            10,
        )
        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Loaded {len(self.label_timestamps)} labeled frames for replay.')

    def load_labels(self, dataset_root, run_prefix):
        by_timestamp = {}
        pattern = f'{run_prefix}*.txt'
        for label_path in dataset_root.rglob(pattern):
            match = TIMESTAMP_PATTERN.search(label_path.name)
            if match is None:
                continue
            timestamp_ns = int(match.group(1))
            boxes = []
            for line in label_path.read_text(encoding='utf-8').splitlines():
                values = line.split()
                if len(values) != 5 or int(values[0]) != self.class_id:
                    continue
                boxes.append(tuple(float(value) for value in values[1:]))
            by_timestamp[timestamp_ns] = boxes
        timestamps = sorted(by_timestamp)
        return timestamps, by_timestamp

    def image_callback(self, image_msg):
        image_stamp_ns = (
            int(image_msg.header.stamp.sec) * 1_000_000_000
            + int(image_msg.header.stamp.nanosec)
        )
        if image_stamp_ns <= 0:
            image_stamp_ns = self.get_clock().now().nanoseconds
        if self.image_origin_ns is None:
            self.image_origin_ns = image_stamp_ns

        target_ns = (
            self.label_origin_ns + image_stamp_ns - self.image_origin_ns)
        label_ns = self.nearest_timestamp(target_ns)
        message = Detections()
        message.header = image_msg.header
        if abs(label_ns - target_ns) <= self.max_delta_ns:
            for center_x, center_y, width, height in self.labels[label_ns]:
                detection = Detection()
                detection.class_name = self.class_name
                detection.confidence = 1.0
                detection.xmin = int((center_x - width / 2.0) * image_msg.width)
                detection.ymin = int((center_y - height / 2.0) * image_msg.height)
                detection.xmax = int((center_x + width / 2.0) * image_msg.width)
                detection.ymax = int((center_y + height / 2.0) * image_msg.height)
                message.detections.append(detection)
        self.publisher.publish(message)

    def nearest_timestamp(self, target_ns):
        index = bisect.bisect_left(self.label_timestamps, target_ns)
        candidates = self.label_timestamps[max(0, index - 1):index + 1]
        return min(candidates, key=lambda value: abs(value - target_ns))


def main(args=None):
    rclpy.init(args=args)
    node = PseudoDetectionReplayNode()
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
