"""Adapt team Float32 commands to the measured Xycar and Gazebo Ackermann input."""

from __future__ import annotations

from collections import deque
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Float32MultiArray

from .calibration import XycarCalibration


def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class CommandAdapter(Node):
    def __init__(self) -> None:
        super().__init__('command_adapter')
        self.declare_parameter('use_sim', True)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('topic_speed_command', '/cmd/speed')
        self.declare_parameter('topic_steer_command', '/cmd/steer')
        self.declare_parameter('topic_motor', '/xycar_motor')
        self.declare_parameter('topic_cmd_vel', '/model/xycar/cmd_vel')
        self.declare_parameter('topic_calibration_state', '/xycar/calibration_state')
        self.declare_parameter('topic_physical_raw_steer', '/xycar/physical_raw_steer')
        self.declare_parameter('topic_motor_raw_steer', '/xycar/motor_topic_raw_steer')
        self.declare_parameter('wheelbase', 0.355)
        self.declare_parameter('front_track', 0.250)
        self.declare_parameter('logical_steer_rad_lut', [-0.571108752, 0.0, 0.408565539])
        self.declare_parameter('raw_steer_lut_by_logical', [40.0, 7.0, -40.0])
        self.declare_parameter('raw_steer_lut', [-40.0, 7.0, 40.0])
        self.declare_parameter('curvature_lut', [1.219512195, 0.0, -1.809954751])
        self.declare_parameter('speed_command_lut', [4.0, 25.0])
        self.declare_parameter('speed_mps_lut', [0.399, 2.222])
        self.declare_parameter('motor_topic_steer_sign', -1.0)
        self.declare_parameter('steering_rate_limit_rad_s', 1.2)
        self.declare_parameter('steering_delay_sec', 0.0)
        self.declare_parameter('motor_delay_sec', 0.0)

        self.use_sim = bool(self.get_parameter('use_sim').value)
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
        self.motor_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('topic_motor').value, qos)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('topic_cmd_vel').value, qos)
        self.state_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('topic_calibration_state').value, qos)
        self.physical_raw_pub = self.create_publisher(
            Float32, self.get_parameter('topic_physical_raw_steer').value, qos)
        self.motor_raw_pub = self.create_publisher(
            Float32, self.get_parameter('topic_motor_raw_steer').value, qos)
        self.create_subscription(
            Float32, self.get_parameter('topic_speed_command').value, self._speed_callback, qos)
        self.create_subscription(
            Float32, self.get_parameter('topic_steer_command').value, self._steer_callback, qos)

        now = self._now()
        self.speed_events: deque[tuple[float, float]] = deque([(now, 0.0)])
        self.steer_events: deque[tuple[float, float]] = deque([(now, 0.0)])
        self.speed_command = 0.0
        self.target_logical_steer = 0.0
        self.applied_logical_steer = 0.0
        self.last_speed_message_time = now
        self.last_steer_message_time = now
        self.last_tick_time = now

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(period, self._tick)

        extreme = self.calibration.ackermann_angles(self.calibration.curvature_from_raw(40.0))
        self.get_logger().warning(
            'raw +40 radius calibration requires Ackermann wheel angles '
            f'left={math.degrees(extreme.left_rad):.2f} deg, '
            f'right={math.degrees(extreme.right_rad):.2f} deg; '
            'the inside angle exceeds the nominal measured ~30 deg and must be re-measured.')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _speed_callback(self, message: Float32) -> None:
        now = self._now()
        self.speed_events.append((now, float(message.data)))
        self.last_speed_message_time = now

    def _steer_callback(self, message: Float32) -> None:
        now = self._now()
        self.steer_events.append((now, float(message.data)))
        self.last_steer_message_time = now

    @staticmethod
    def _apply_delay(
        events: deque[tuple[float, float]], now: float, delay: float, current: float
    ) -> float:
        threshold = now - max(0.0, delay)
        while events and events[0][0] <= threshold:
            _, current = events.popleft()
        return current

    def _tick(self) -> None:
        now = self._now()
        dt = max(0.0, min(0.5, now - self.last_tick_time))
        self.last_tick_time = now

        self.speed_command = self._apply_delay(
            self.speed_events, now, float(self.get_parameter('motor_delay_sec').value), self.speed_command)
        self.target_logical_steer = self._apply_delay(
            self.steer_events,
            now,
            float(self.get_parameter('steering_delay_sec').value),
            self.target_logical_steer,
        )

        timeout = float(self.get_parameter('command_timeout_sec').value)
        if now - self.last_speed_message_time > timeout:
            self.speed_command = 0.0
        if now - self.last_steer_message_time > timeout:
            self.target_logical_steer = 0.0

        rate_limit = max(0.0, float(self.get_parameter('steering_rate_limit_rad_s').value))
        max_step = rate_limit * dt
        steer_error = self.target_logical_steer - self.applied_logical_steer
        if abs(steer_error) <= max_step or rate_limit == 0.0:
            self.applied_logical_steer = self.target_logical_steer
        else:
            self.applied_logical_steer += math.copysign(max_step, steer_error)

        physical_raw = self.calibration.raw_from_logical(self.applied_logical_steer)
        motor_raw = float(self.get_parameter('motor_topic_steer_sign').value) * physical_raw
        curvature = self.calibration.curvature_from_raw(physical_raw)
        speed_mps = self.calibration.speed_from_command(self.speed_command)
        angles = self.calibration.ackermann_angles(curvature)

        motor = Float32MultiArray()
        motor.data = [float(motor_raw), float(self.speed_command)]
        self.motor_pub.publish(motor)

        physical_raw_message = Float32()
        physical_raw_message.data = float(physical_raw)
        self.physical_raw_pub.publish(physical_raw_message)
        motor_raw_message = Float32()
        motor_raw_message.data = float(motor_raw)
        self.motor_raw_pub.publish(motor_raw_message)

        state = Float32MultiArray()
        state.data = [
            float(self.applied_logical_steer),
            float(physical_raw),
            float(motor_raw),
            float(self.speed_command),
            float(speed_mps),
            float(curvature),
            float(angles.left_rad),
            float(angles.right_rad),
        ]
        self.state_pub.publish(state)

        if self.use_sim:
            command = Twist()
            command.linear.x = float(speed_mps)
            command.angular.z = float(speed_mps * curvature)
            self.cmd_vel_pub.publish(command)


def main() -> None:
    rclpy.init()
    node = CommandAdapter()
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
