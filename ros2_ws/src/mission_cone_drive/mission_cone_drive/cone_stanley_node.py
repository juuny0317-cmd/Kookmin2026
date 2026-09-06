import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from mission_cone_drive.cone_stanley_algorithms import (
    compute_stanley_control,
)


class ConeStanleyNode(Node):
    """Track a vehicle-frame cone corridor path with Stanley control."""

    def __init__(self):
        super().__init__('cone_stanley_node')

        self.declare_parameter('require_green_signal', False)
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('motor_topic', '/cone_motor_cmd')
        self.declare_parameter('activation_topic', '/sign_color')
        self.declare_parameter('min_speed', 2.0)
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('control_x_m', 0.33)
        self.declare_parameter('min_forward_x_m', 0.0)
        self.declare_parameter('heading_window_m', 0.30)
        self.declare_parameter('cross_track_gain', 1.2)
        self.declare_parameter('heading_gain', 1.0)
        self.declare_parameter('softening_speed_mps', 0.50)
        self.declare_parameter('speed_units_per_mps', 20.0)
        self.declare_parameter('max_steering_angle_deg', 20.0)
        self.declare_parameter('servo_scale_deg_per_unit', 0.20)
        self.declare_parameter('max_angle_step', 15.0)
        self.declare_parameter('smoothing_alpha', 0.65)
        self.declare_parameter('min_path_points', 2)
        self.declare_parameter('command_publish_rate_hz', 10.0)
        self.declare_parameter('path_timeout_s', 0.50)
        self.declare_parameter('hold_last_angle_when_stale', True)
        self.declare_parameter('publish_stop_on_inactive', True)
        self.declare_parameter(
            'status_topic',
            '/cone_stanley_status',
        )

        self.require_green_signal = self.as_bool(
            self.get_parameter('require_green_signal').value
        )
        self.is_active = not self.require_green_signal
        self.traffic_state = 'unknown'
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.control_x_m = float(
            self.get_parameter('control_x_m').value
        )
        self.min_forward_x_m = float(
            self.get_parameter('min_forward_x_m').value
        )
        self.heading_window_m = float(
            self.get_parameter('heading_window_m').value
        )
        self.cross_track_gain = float(
            self.get_parameter('cross_track_gain').value
        )
        self.heading_gain = float(
            self.get_parameter('heading_gain').value
        )
        self.softening_speed_mps = float(
            self.get_parameter('softening_speed_mps').value
        )
        self.speed_units_per_mps = max(
            float(self.get_parameter('speed_units_per_mps').value),
            1e-3,
        )
        self.max_steering_angle_deg = float(
            self.get_parameter('max_steering_angle_deg').value
        )
        self.servo_scale = float(
            self.get_parameter('servo_scale_deg_per_unit').value
        )
        self.max_angle_step = max(
            float(self.get_parameter('max_angle_step').value),
            0.0,
        )
        self.smoothing_alpha = float(np.clip(
            self.get_parameter('smoothing_alpha').value,
            0.0,
            1.0,
        ))
        self.min_path_points = max(
            int(self.get_parameter('min_path_points').value),
            2,
        )
        self.command_publish_rate_hz = max(
            float(
                self.get_parameter('command_publish_rate_hz').value
            ),
            1.0,
        )
        self.path_timeout_s = max(
            float(self.get_parameter('path_timeout_s').value),
            0.05,
        )
        self.hold_last_angle_when_stale = self.as_bool(
            self.get_parameter('hold_last_angle_when_stale').value
        )
        self.publish_stop_on_inactive = self.as_bool(
            self.get_parameter('publish_stop_on_inactive').value
        )

        self.path_frame_id = 'rear_axle'
        self.last_path_time = None
        self.last_angle_cmd = 0.0
        self.last_speed_cmd = 0.0
        self.last_status_time = None

        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter('path_topic').value,
            self.path_callback,
            10,
        )
        self.activation_sub = self.create_subscription(
            String,
            self.get_parameter('activation_topic').value,
            self.activation_callback,
            10,
        )
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('motor_topic').value,
            10,
        )
        self.target_pub = self.create_publisher(
            PointStamped,
            '/target_point',
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            10,
        )
        self.command_timer = self.create_timer(
            1.0 / self.command_publish_rate_hz,
            self.command_timer_callback,
        )

        mode = (
            'waiting for green signal'
            if self.require_green_signal
            else 'active immediately'
        )
        self.get_logger().info(
            'Cone Stanley controller started '
            f'({mode}), speed={self.min_speed:.1f}-'
            f'{self.max_speed:.1f}, cte_gain={self.cross_track_gain:.2f}, '
            f'heading_gain={self.heading_gain:.2f}.'
        )

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def activation_callback(self, msg):
        self.traffic_state = msg.data.strip().lower()
        if self.traffic_state == 'green':
            self.is_active = True
        elif self.traffic_state == 'red':
            self.is_active = False
            self.set_motor_command(0.0, 0.0)

    def path_callback(self, msg):
        if not self.is_active or self.traffic_state == 'red':
            self.set_motor_command(0.0, 0.0)
            return
        if len(msg.poses) < self.min_path_points:
            self.stop_for_invalid_path()
            return

        path_points = np.asarray([
            [pose.pose.position.x, pose.pose.position.y]
            for pose in msg.poses
        ], dtype=np.float64)
        if msg.header.frame_id:
            self.path_frame_id = msg.header.frame_id

        speed_command = max(
            abs(self.last_speed_cmd),
            abs(self.max_speed),
        )
        speed_mps = speed_command / self.speed_units_per_mps
        try:
            result = compute_stanley_control(
                path_points,
                speed_mps=speed_mps,
                control_x_m=self.control_x_m,
                min_forward_x_m=self.min_forward_x_m,
                heading_window_m=self.heading_window_m,
                cross_track_gain=self.cross_track_gain,
                heading_gain=self.heading_gain,
                softening_speed_mps=self.softening_speed_mps,
                max_steering_angle_deg=self.max_steering_angle_deg,
                servo_scale_deg_per_unit=self.servo_scale,
            )
        except ValueError as exc:
            self.get_logger().warn(
                f'Invalid cone path for Stanley control: {exc}'
            )
            self.stop_for_invalid_path()
            return

        angle = self.limit_and_smooth_angle(result.servo_command)
        speed = self.compute_speed(angle)
        self.last_path_time = self.get_clock().now()
        self.set_motor_command(angle, speed)
        self.publish_target_point(
            result.reference_x,
            result.reference_y,
        )
        self.publish_status(result, angle, speed)

    def limit_and_smooth_angle(self, raw_angle):
        limited = float(raw_angle)
        if self.max_angle_step > 0.0:
            delta = np.clip(
                limited - self.last_angle_cmd,
                -self.max_angle_step,
                self.max_angle_step,
            )
            limited = self.last_angle_cmd + float(delta)
        smoothed = (
            self.smoothing_alpha * limited
            + (1.0 - self.smoothing_alpha) * self.last_angle_cmd
        )
        return float(np.clip(smoothed, -100.0, 100.0))

    def compute_speed(self, angle_cmd):
        if self.max_speed <= self.min_speed:
            return float(self.min_speed)
        turn_ratio = min(abs(float(angle_cmd)) / 100.0, 1.0)
        return float(
            self.max_speed
            - (self.max_speed - self.min_speed) * turn_ratio
        )

    def stop_for_invalid_path(self):
        safe_angle = (
            self.last_angle_cmd
            if self.hold_last_angle_when_stale
            else 0.0
        )
        self.last_path_time = None
        self.set_motor_command(safe_angle, 0.0)

    def command_timer_callback(self):
        if (
            self.publish_stop_on_inactive
            and (not self.is_active or self.traffic_state == 'red')
        ):
            self.publish_motor_command(0.0, 0.0)
            return
        if self.last_path_time is None:
            self.publish_safe_stop()
            return

        age_s = (
            self.get_clock().now() - self.last_path_time
        ).nanoseconds * 1e-9
        if age_s > self.path_timeout_s:
            self.publish_safe_stop()
            return
        self.publish_motor_command(
            self.last_angle_cmd,
            self.last_speed_cmd,
        )

    def publish_safe_stop(self):
        safe_angle = (
            self.last_angle_cmd
            if self.hold_last_angle_when_stale
            else 0.0
        )
        self.publish_motor_command(safe_angle, 0.0)

    def set_motor_command(self, angle, speed):
        self.last_angle_cmd = float(angle)
        self.last_speed_cmd = float(speed)
        self.publish_motor_command(angle, speed)

    def publish_motor_command(self, angle, speed):
        msg = Float32MultiArray()
        msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(msg)

    def publish_target_point(self, x, y):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.path_frame_id
        msg.point.x = float(x)
        msg.point.y = float(y)
        self.target_pub.publish(msg)

    def publish_status(self, result, angle, speed):
        now = self.get_clock().now()
        if self.last_status_time is not None:
            age_s = (now - self.last_status_time).nanoseconds * 1e-9
            if age_s < 0.50:
                return
        self.last_status_time = now

        status = String()
        status.data = (
            f'cte={result.cross_track_error_m:.3f}m '
            f'heading={math.degrees(result.path_heading_rad):.1f}deg '
            f'cte_term={math.degrees(result.cross_track_term_rad):.1f}deg '
            f'angle={angle:.1f} speed={speed:.1f}'
        )
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ConeStanleyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
