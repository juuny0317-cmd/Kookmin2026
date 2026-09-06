"""Republish Gazebo sensors with the exact bag-observed ROS interface."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan


def qos(reliability: ReliabilityPolicy, depth: int) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )


class SensorAdapter(Node):
    def __init__(self) -> None:
        super().__init__('sensor_adapter')
        self.declare_parameter('topic_image_input', '/xycar_sim/image_raw')
        self.declare_parameter('topic_image_output', '/image_raw')
        self.declare_parameter('topic_camera_info_output', '/camera_info')
        self.declare_parameter('topic_scan_input', '/xycar_sim/scan_raw')
        self.declare_parameter('topic_scan_output', '/scan')
        self.declare_parameter('camera_frame_id', 'usb_cam')
        self.declare_parameter('lidar_frame_id', 'laser_frame')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('distortion_model', 'plumb_bob')
        self.declare_parameter('d', [-0.361976, 0.11051, 0.001014, 0.000505, 0.0])
        self.declare_parameter('k', [438.783367, 0.0, 305.593336, 0.0, 437.302876, 243.738352, 0.0, 0.0, 1.0])
        self.declare_parameter('r', [0.999978, 0.002789, -0.006046, -0.002816, 0.999986, -0.004401, 0.006034, 0.004417, 0.999972])
        self.declare_parameter('p', [393.6538, 0.0, 322.797939, 0.0, 0.0, 393.6538, 241.090902, 0.0, 0.0, 0.0, 1.0, 0.0])

        self.camera_qos = qos(ReliabilityPolicy.RELIABLE, 10)
        self.scan_qos = qos(ReliabilityPolicy.BEST_EFFORT, 5)
        # A best-effort input accepts either bridge QoS; outputs reproduce bag QoS.
        self.bridge_input_qos = qos(ReliabilityPolicy.BEST_EFFORT, 5)

        self.image_pub = self.create_publisher(
            Image, self.get_parameter('topic_image_output').value, self.camera_qos)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter('topic_camera_info_output').value, self.camera_qos)
        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter('topic_scan_output').value, self.scan_qos)
        self.create_subscription(
            Image,
            self.get_parameter('topic_image_input').value,
            self._image_callback,
            self.bridge_input_qos,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('topic_scan_input').value,
            self._scan_callback,
            self.bridge_input_qos,
        )

    def _image_callback(self, message: Image) -> None:
        message.header.frame_id = str(self.get_parameter('camera_frame_id').value)
        self.image_pub.publish(message)

        info = CameraInfo()
        info.header = message.header
        info.width = int(self.get_parameter('image_width').value)
        info.height = int(self.get_parameter('image_height').value)
        info.distortion_model = str(self.get_parameter('distortion_model').value)
        info.d = [float(value) for value in self.get_parameter('d').value]
        info.k = [float(value) for value in self.get_parameter('k').value]
        info.r = [float(value) for value in self.get_parameter('r').value]
        info.p = [float(value) for value in self.get_parameter('p').value]
        info.binning_x = 0
        info.binning_y = 0
        info.roi.x_offset = 0
        info.roi.y_offset = 0
        info.roi.height = 0
        info.roi.width = 0
        info.roi.do_rectify = False
        self.camera_info_pub.publish(info)

    def _scan_callback(self, message: LaserScan) -> None:
        message.header.frame_id = str(self.get_parameter('lidar_frame_id').value)
        self.scan_pub.publish(message)


def main() -> None:
    rclpy.init()
    node = SensorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        if 'Unable to convert call argument to Python object' not in str(error):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
