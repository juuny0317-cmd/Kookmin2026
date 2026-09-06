"""Translate the team's centered motor command into the measured Xycar interface."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Float32MultiArray

from .calibration import PiecewiseLinear, XycarCalibration


def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class TeamMotorAdapter(Node):
    """Adapt ``[centered_angle, speed]`` without changing the team controller.

    The team code uses angle 0 as straight, negative as left and positive as
    right.  The measured vehicle uses physical raw +7 as straight.  The first
    lookup converts the centered team command to physical raw; the existing
    measured asymmetric lookup then converts physical raw to the logical
    steering angle consumed by ``command_adapter``.
    """

    def __init__(self) -> None:
        super().__init__('team_motor_adapter')
        self.declare_parameter('input_topic', '/team/xycar_motor_cmd')
        self.declare_parameter('speed_output_topic', '/cmd/speed')
        self.declare_parameter('steer_output_topic', '/cmd/steer')
        self.declare_parameter('state_topic', '/team/adapter_state')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('min_speed_command', -10.0)
        self.declare_parameter('max_speed_command', 25.0)

        # Team controller angle is centered at zero.  These knots are the
        # measured physical raw knots shifted by the +7 straight centre.
        self.declare_parameter(
            'team_angle_lut', [-47.0, -37.0, -27.0, 0.0, 13.0, 23.0, 33.0])
        self.declare_parameter(
            'physical_raw_lut', [-40.0, -30.0, -20.0, 7.0, 20.0, 30.0, 40.0])
        self.declare_parameter('wheelbase', 0.355)
        self.declare_parameter('front_track', 0.250)
        self.declare_parameter(
            'logical_steer_rad_lut', [-0.571108752, 0.0, 0.408565539])
        self.declare_parameter(
            'raw_steer_lut_by_logical', [40.0, 7.0, -40.0])
        self.declare_parameter('raw_steer_lut', [-40.0, 7.0, 40.0])
        self.declare_parameter('curvature_lut', [1.219512195, 0.0, -1.809954751])
        self.declare_parameter('speed_command_lut', [4.0, 25.0])
        self.declare_parameter('speed_mps_lut', [0.399, 2.222])

        self.team_to_raw = PiecewiseLinear(
            self.get_parameter('team_angle_lut').value,
            self.get_parameter('physical_raw_lut').value,
        )
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
        self.speed_pub = self.create_publisher(
            Float32, self.get_parameter('speed_output_topic').value, qos)
        self.steer_pub = self.create_publisher(
            Float32, self.get_parameter('steer_output_topic').value, qos)
        self.state_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('state_topic').value, qos)
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter('input_topic').value,
            self._command_callback,
            qos,
        )

        self.last_input_time: float | None = None
        self.team_angle = 0.0
        self.speed_command = 0.0
        self.invalid_message_reported = False
        publish_rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / publish_rate, self._tick)
        self.get_logger().info(
            'team motor adapter ready: [angle, speed] on '
            f'{self.get_parameter("input_topic").value}; angle 0 -> physical raw +7')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _command_callback(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            if not self.invalid_message_reported:
                self.get_logger().error(
                    'team motor command must contain [angle, speed]; stopping vehicle')
                self.invalid_message_reported = True
            self.team_angle = 0.0
            self.speed_command = 0.0
            self.last_input_time = self._now()
            return
        angle = float(message.data[0])
        speed = float(message.data[1])
        if not math.isfinite(angle) or not math.isfinite(speed):
            if not self.invalid_message_reported:
                self.get_logger().error(
                    'team motor command contains a non-finite value; stopping vehicle')
                self.invalid_message_reported = True
            self.team_angle = 0.0
            self.speed_command = 0.0
            self.last_input_time = self._now()
            return
        self.invalid_message_reported = False
        self.team_angle = angle
        self.speed_command = min(
            float(self.get_parameter('max_speed_command').value),
            max(float(self.get_parameter('min_speed_command').value), speed),
        )
        self.last_input_time = self._now()

    def _publish(self, team_angle: float, speed_command: float) -> None:
        physical_raw = self.team_to_raw(team_angle)
        logical_steer = self.calibration.logical_from_raw(physical_raw)

        speed_message = Float32()
        speed_message.data = float(speed_command)
        steer_message = Float32()
        steer_message.data = float(logical_steer)
        self.speed_pub.publish(speed_message)
        self.steer_pub.publish(steer_message)

        state = Float32MultiArray()
        state.data = [
            float(team_angle),
            float(speed_command),
            float(physical_raw),
            float(logical_steer),
        ]
        self.state_pub.publish(state)

    def _tick(self) -> None:
        timeout = max(0.0, float(self.get_parameter('command_timeout_sec').value))
        stale = (
            self.last_input_time is None
            or self._now() - self.last_input_time > timeout
        )
        if stale:
            self._publish(0.0, 0.0)
        else:
            self._publish(self.team_angle, self.speed_command)


def main() -> None:
    rclpy.init()
    node = TeamMotorAdapter()
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
