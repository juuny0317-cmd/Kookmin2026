import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRotatorNode(Node):
    def __init__(self):
        super().__init__('scan_rotator_node')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_rotated')
        self.declare_parameter('rotation_deg', 0.0)
        self.declare_parameter('frame_id', 'laser_frame')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.rotation_rad = math.radians(float(self.get_parameter('rotation_deg').value))
        self.frame_id = self.get_parameter('frame_id').value

        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.lidar_callback,
            qos_profile_sensor_data,
        )
        self.rotated_scan_pub = self.create_publisher(LaserScan, self.output_topic, 10)

        self.get_logger().info(
            f'Scan rotator started: {self.input_topic} -> {self.output_topic}, '
            f'rotation={math.degrees(self.rotation_rad):.1f} deg'
        )

    def lidar_callback(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return

        if abs(self.rotation_rad) < 1e-6:
            passthrough_msg = LaserScan()
            passthrough_msg.header = msg.header
            passthrough_msg.header.frame_id = self.frame_id
            passthrough_msg.angle_min = msg.angle_min
            passthrough_msg.angle_max = msg.angle_max
            passthrough_msg.angle_increment = msg.angle_increment
            passthrough_msg.time_increment = msg.time_increment
            passthrough_msg.scan_time = msg.scan_time
            passthrough_msg.range_min = msg.range_min
            passthrough_msg.range_max = msg.range_max
            passthrough_msg.ranges = list(msg.ranges)
            passthrough_msg.intensities = list(msg.intensities)
            self.rotated_scan_pub.publish(passthrough_msg)
            return

        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
        valid = np.isfinite(ranges)

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        cos_t = math.cos(self.rotation_rad)
        sin_t = math.sin(self.rotation_rad)
        x_rot = x * cos_t - y * sin_t
        y_rot = x * sin_t + y * cos_t

        rotated_ranges = np.hypot(x_rot, y_rot)
        rotated_angles = np.arctan2(y_rot, x_rot)

        new_ranges = np.full(ranges.size, np.inf, dtype=np.float32)
        has_intensities = len(msg.intensities) == ranges.size
        new_intensities = np.zeros(ranges.size, dtype=np.float32)

        indices = ((rotated_angles - msg.angle_min) / msg.angle_increment).astype(np.int32)
        for source_idx, target_idx in enumerate(indices):
            if not valid[source_idx] or target_idx < 0 or target_idx >= ranges.size:
                continue
            if rotated_ranges[source_idx] < new_ranges[target_idx]:
                new_ranges[target_idx] = rotated_ranges[source_idx]
                if has_intensities:
                    new_intensities[target_idx] = msg.intensities[source_idx]

        rotated_msg = LaserScan()
        rotated_msg.header = msg.header
        rotated_msg.header.frame_id = self.frame_id
        rotated_msg.angle_min = msg.angle_min
        rotated_msg.angle_max = msg.angle_max
        rotated_msg.angle_increment = msg.angle_increment
        rotated_msg.time_increment = msg.time_increment
        rotated_msg.scan_time = msg.scan_time
        rotated_msg.range_min = msg.range_min
        rotated_msg.range_max = msg.range_max
        rotated_msg.ranges = new_ranges.tolist()
        if has_intensities:
            rotated_msg.intensities = new_intensities.tolist()

        self.rotated_scan_pub.publish(rotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRotatorNode()
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
