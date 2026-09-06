"""Drive one repeatable calibration case and report measured Gazebo results."""

from __future__ import annotations

import math
import threading
import time

import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float32, Float32MultiArray

from .calibration import XycarCalibration


TURN_CASES = {
    'turn_p20': (20.0, 1.150),
    'turn_p30': (30.0, 0.780),
    'turn_p40': (40.0, 0.5525),
    'turn_m20': (-20.0, 2.415),
    'turn_m30': (-30.0, 1.315),
    'turn_m40': (-40.0, 0.820),
}
SPEED_CASES = {'speed_5': (5.0, 9.84), 'speed_10': (10.0, 4.90), 'speed_15': (15.0, 3.31)}


def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def fit_circle(points: list[tuple[float, float]]) -> tuple[float, float, float]:
    if len(points) < 10:
        raise ValueError('at least ten odometry points are required')
    xy = np.asarray(points, dtype=float)
    matrix = np.column_stack((2.0 * xy[:, 0], 2.0 * xy[:, 1], np.ones(len(xy))))
    target = xy[:, 0] ** 2 + xy[:, 1] ** 2
    center_x, center_y, constant = np.linalg.lstsq(matrix, target, rcond=None)[0]
    radius = math.sqrt(max(0.0, constant + center_x ** 2 + center_y ** 2))
    return float(center_x), float(center_y), float(radius)


class VerificationNode(Node):
    def __init__(self) -> None:
        super().__init__('xycar_verification')
        self.declare_parameter('test_case', 'straight_zero')
        self.declare_parameter('settle_time_sec', 2.0)
        self.declare_parameter('turn_sample_time_sec', 12.0)
        self.declare_parameter('logical_steer_rad_lut', [-0.571108752, 0.0, 0.408565539])
        self.declare_parameter('raw_steer_lut_by_logical', [40.0, 7.0, -40.0])
        self.declare_parameter('raw_steer_lut', [-40.0, 7.0, 40.0])
        self.declare_parameter('curvature_lut', [1.219512195, 0.0, -1.809954751])
        self.declare_parameter('speed_command_lut', [4.0, 25.0])
        self.declare_parameter('speed_mps_lut', [0.399, 2.222])
        self.declare_parameter('wheelbase', 0.355)
        self.declare_parameter('front_track', 0.250)

        self.test_case = str(self.get_parameter('test_case').value)
        valid = {'straight_zero', 'sensor_interface', *TURN_CASES, *SPEED_CASES}
        if self.test_case not in valid:
            raise ValueError(f'unknown test_case {self.test_case!r}; choose one of {sorted(valid)}')

        self.calibration = XycarCalibration(
            self.get_parameter('logical_steer_rad_lut').value,
            self.get_parameter('raw_steer_lut_by_logical').value,
            self.get_parameter('raw_steer_lut').value,
            self.get_parameter('curvature_lut').value,
            self.get_parameter('speed_command_lut').value,
            self.get_parameter('speed_mps_lut').value,
            self.get_parameter('wheelbase').value,
            self.get_parameter('front_track').value,
        )
        qos = reliable_qos()
        self.speed_pub = self.create_publisher(Float32, '/cmd/speed', qos)
        self.steer_pub = self.create_publisher(Float32, '/cmd/steer', qos)
        self.create_subscription(Odometry, '/odom', self._odom_callback, qos)
        self.create_subscription(Float32MultiArray, '/xycar/calibration_state', self._state_callback, qos)
        self.create_subscription(Image, '/image_raw', self._image_callback, qos)
        self.create_subscription(CameraInfo, '/camera_info', self._camera_info_callback, qos)
        self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.wall_start = time.monotonic()
        self.command_start: float | None = None
        self.start_xy: tuple[float, float] | None = None
        self.latest_xy: tuple[float, float] | None = None
        self.points: list[tuple[float, float]] = []
        self.latest_state: list[float] | None = None
        self.image_ok = False
        self.camera_info_ok = False
        self.scan_ok = False
        self.finished = False
        self.create_timer(0.05, self._tick)

    def _odom_callback(self, message: Odometry) -> None:
        self.latest_xy = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self.command_start is not None:
            elapsed = self._sim_time_sec() - self.command_start
            settle = float(self.get_parameter('settle_time_sec').value)
            if self.test_case in TURN_CASES and elapsed >= settle:
                self.points.append(self.latest_xy)

    def _sim_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _state_callback(self, message: Float32MultiArray) -> None:
        self.latest_state = list(message.data)

    def _image_callback(self, message: Image) -> None:
        self.image_ok = (
            message.header.frame_id == 'usb_cam'
            and message.width == 640
            and message.height == 480
            and message.encoding == 'rgb8'
        )

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.camera_info_ok = (
            message.header.frame_id == 'usb_cam'
            and message.width == 640
            and message.height == 480
            and message.distortion_model == 'plumb_bob'
            and len(message.d) == 5
        )

    def _scan_callback(self, message: LaserScan) -> None:
        self.scan_ok = (
            message.header.frame_id == 'laser_frame'
            and len(message.ranges) == 500
            and abs(message.range_min - 0.1) < 1e-3
            and abs(message.range_max - 16.0) < 1e-3
        )

    def _publish(self, speed: float, logical_steer: float) -> None:
        speed_message = Float32()
        speed_message.data = float(speed)
        steer_message = Float32()
        steer_message.data = float(logical_steer)
        self.speed_pub.publish(speed_message)
        self.steer_pub.publish(steer_message)

    def _finish(self, result: str) -> None:
        self._publish(0.0, 0.0)
        self.get_logger().info(f'VERIFICATION RESULT: {result}')
        self.finished = True
        threading.Timer(0.25, self._request_shutdown).start()

    @staticmethod
    def _request_shutdown() -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def _tick(self) -> None:
        if self.finished:
            self._publish(0.0, 0.0)
            return
        wall_elapsed = time.monotonic() - self.wall_start
        if wall_elapsed < 2.0:
            self._publish(0.0, 0.0)
            return

        if self.test_case == 'sensor_interface':
            self._publish(0.0, 0.0)
            if self.image_ok and self.camera_info_ok and self.scan_ok:
                self._finish('PASS image_raw/camera_info/scan types, dimensions and frame_ids match bags')
            elif wall_elapsed > 15.0:
                self._finish(
                    f'FAIL image={self.image_ok}, camera_info={self.camera_info_ok}, scan={self.scan_ok}')
            return

        if self.test_case == 'straight_zero':
            self._publish(0.0, 0.0)
            if self.latest_state is not None and len(self.latest_state) >= 3:
                physical_raw, motor_raw = self.latest_state[1], self.latest_state[2]
                status = 'PASS' if abs(physical_raw - 7.0) < 0.05 and abs(motor_raw + 7.0) < 0.05 else 'FAIL'
                self._finish(
                    f'{status} logical 0 -> physical raw {physical_raw:.3f}; '
                    f'bag-compatible /xycar_motor steer {motor_raw:.3f}')
            return

        if self.latest_xy is None:
            self._publish(0.0, 0.0)
            return
        if self.command_start is None:
            self.command_start = self._sim_time_sec()
            self.start_xy = self.latest_xy

        if self.test_case in SPEED_CASES:
            command, expected_time = SPEED_CASES[self.test_case]
            self._publish(command, 0.0)
            distance = math.dist(self.start_xy, self.latest_xy)
            if distance >= 5.0:
                elapsed = self._sim_time_sec() - self.command_start
                error = elapsed - expected_time
                self._finish(
                    f'command={command:.0f}, 5m simulation_time={elapsed:.3f}s, '
                    f'real={expected_time:.3f}s, error={error:+.3f}s')
            elif self._sim_time_sec() - self.command_start > expected_time * 2.0 + 5.0:
                self._finish(f'FAIL command={command:.0f} did not reach 5m')
            return

        raw, expected_radius = TURN_CASES[self.test_case]
        logical = self.calibration.logical_from_raw(raw)
        self._publish(4.0, logical)
        elapsed = self._sim_time_sec() - self.command_start
        sample_end = float(self.get_parameter('settle_time_sec').value) + float(
            self.get_parameter('turn_sample_time_sec').value)
        if elapsed >= sample_end:
            _, _, radius = fit_circle(self.points)
            error = radius - expected_radius
            side = 'right' if raw > 0.0 else 'left'
            self._finish(
                f'raw={raw:+.0f} ({side}), radius={radius:.4f}m, '
                f'real={expected_radius:.4f}m, error={error:+.4f}m, samples={len(self.points)}')


def main() -> None:
    rclpy.init()
    node = VerificationNode()
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
