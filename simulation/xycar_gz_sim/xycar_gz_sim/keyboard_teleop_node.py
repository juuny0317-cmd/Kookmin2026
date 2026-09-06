"""Interactive keyboard driver for the Xycar command adapter.

Run this executable in its own terminal.  It publishes the same Float32 command
topics as the team algorithm, so no simulation-only command interface is added.
"""

from __future__ import annotations

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Float32MultiArray


HELP = """
Xycar Y2 keyboard drive
  W / Up       speed command +step
  S / Down     speed command -step (negative is unmeasured reverse mapping)
  A / Left     steer left
  D / Right    steer right
  1 / 2 / 3    speed command 5 / 10 / 15
  Space        speed 0, keep steering
  C            center steering (logical 0 -> physical raw +7)
  X            speed 0 and center steering
  H            print this help
  Q / Ctrl-C   stop, center, and quit
"""


def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_teleop')
        self.declare_parameter('topic_speed_command', '/cmd/speed')
        self.declare_parameter('topic_steer_command', '/cmd/steer')
        self.declare_parameter('topic_calibration_state', '/xycar/calibration_state')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('speed_step', 1.0)
        self.declare_parameter('steer_step_deg', 2.0)
        self.declare_parameter('min_speed_command', -10.0)
        self.declare_parameter('max_speed_command', 25.0)
        self.declare_parameter('min_logical_steer_rad', -0.571108752)
        self.declare_parameter('max_logical_steer_rad', 0.408565539)

        qos = reliable_qos()
        self.speed_pub = self.create_publisher(
            Float32, self.get_parameter('topic_speed_command').value, qos)
        self.steer_pub = self.create_publisher(
            Float32, self.get_parameter('topic_steer_command').value, qos)
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter('topic_calibration_state').value,
            self._state_callback,
            qos,
        )

        self.speed_command = 0.0
        self.logical_steer = 0.0
        self.calibration_state: list[float] | None = None
        self.quit_requested = False

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(upper, max(lower, value))

    def _state_callback(self, message: Float32MultiArray) -> None:
        self.calibration_state = list(message.data)

    def handle_key(self, key: str) -> None:
        speed_step = float(self.get_parameter('speed_step').value)
        steer_step = math.radians(float(self.get_parameter('steer_step_deg').value))
        min_speed = float(self.get_parameter('min_speed_command').value)
        max_speed = float(self.get_parameter('max_speed_command').value)
        min_steer = float(self.get_parameter('min_logical_steer_rad').value)
        max_steer = float(self.get_parameter('max_logical_steer_rad').value)

        if key in ('w', 'W', '\x1b[A'):
            self.speed_command = self._clamp(
                self.speed_command + speed_step, min_speed, max_speed)
        elif key in ('s', 'S', '\x1b[B'):
            self.speed_command = self._clamp(
                self.speed_command - speed_step, min_speed, max_speed)
        elif key in ('a', 'A', '\x1b[D'):
            self.logical_steer = self._clamp(
                self.logical_steer + steer_step, min_steer, max_steer)
        elif key in ('d', 'D', '\x1b[C'):
            self.logical_steer = self._clamp(
                self.logical_steer - steer_step, min_steer, max_steer)
        elif key == '1':
            self.speed_command = self._clamp(5.0, min_speed, max_speed)
        elif key == '2':
            self.speed_command = self._clamp(10.0, min_speed, max_speed)
        elif key == '3':
            self.speed_command = self._clamp(15.0, min_speed, max_speed)
        elif key == ' ':
            self.speed_command = 0.0
        elif key in ('c', 'C'):
            self.logical_steer = 0.0
        elif key in ('x', 'X'):
            self.speed_command = 0.0
            self.logical_steer = 0.0
        elif key in ('h', 'H'):
            sys.stdout.write('\r\x1b[2K' + HELP + '\n')
            sys.stdout.flush()
        elif key in ('q', 'Q', '\x03'):
            self.speed_command = 0.0
            self.logical_steer = 0.0
            self.quit_requested = True

    def publish_command(self) -> None:
        speed = Float32()
        speed.data = float(self.speed_command)
        steer = Float32()
        steer.data = float(self.logical_steer)
        self.speed_pub.publish(speed)
        self.steer_pub.publish(steer)

    def status_line(self) -> str:
        logical_deg = math.degrees(self.logical_steer)
        if self.calibration_state is None or len(self.calibration_state) < 8:
            detail = 'waiting for adapter state'
        else:
            state = self.calibration_state
            detail = (
                f'speed={state[4]:+.3f} m/s | physical raw={state[1]:+.2f} | '
                f'wheel L/R={math.degrees(state[6]):+.1f}/{math.degrees(state[7]):+.1f} deg'
            )
        reverse_note = ' [reverse mapping unmeasured]' if self.speed_command < 0.0 else ''
        return (
            f'speed cmd={self.speed_command:+.1f}{reverse_note} | '
            f'logical steer={logical_deg:+.1f} deg | {detail}'
        )


def read_key(file_descriptor: int, timeout: float) -> str | None:
    readable, _, _ = select.select([file_descriptor], [], [], max(0.0, timeout))
    if not readable:
        return None
    first = os.read(file_descriptor, 1).decode(errors='ignore')
    if first != '\x1b':
        return first
    sequence = first
    for _ in range(2):
        readable, _, _ = select.select([file_descriptor], [], [], 0.02)
        if not readable:
            break
        sequence += os.read(file_descriptor, 1).decode(errors='ignore')
    return sequence


def main() -> None:
    if not sys.stdin.isatty():
        raise RuntimeError('keyboard_teleop must be run in an interactive terminal')

    rclpy.init()
    node = KeyboardTeleop()
    file_descriptor = sys.stdin.fileno()
    original_terminal = termios.tcgetattr(file_descriptor)
    rate_hz = float(node.get_parameter('publish_rate_hz').value)
    period = 1.0 / rate_hz
    next_publish = time.monotonic()

    sys.stdout.write(HELP + '\n')
    sys.stdout.flush()
    try:
        tty.setraw(file_descriptor)
        while rclpy.ok() and not node.quit_requested:
            now = time.monotonic()
            key = read_key(file_descriptor, min(period, max(0.0, next_publish - now)))
            if key is not None:
                node.handle_key(key)
                node.publish_command()

            now = time.monotonic()
            if now >= next_publish:
                node.publish_command()
                rclpy.spin_once(node, timeout_sec=0.0)
                sys.stdout.write('\r\x1b[2K' + node.status_line())
                sys.stdout.flush()
                next_publish = now + period
    finally:
        node.speed_command = 0.0
        node.logical_steer = 0.0
        # Q exits while the ROS context is alive, so send several explicit stop
        # samples. Ctrl-C may already have shut the context down; publishing in
        # that state raises RCLError and used to leave an alarming traceback.
        if rclpy.ok():
            for _ in range(5):
                node.publish_command()
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.02)
        termios.tcsetattr(file_descriptor, termios.TCSADRAIN, original_terminal)
        sys.stdout.write('\r\x1b[2KStopped: speed 0, steering centered.\n')
        sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
