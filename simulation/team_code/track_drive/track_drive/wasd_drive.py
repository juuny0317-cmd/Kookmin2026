#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


HELP = """
WASD manual drive

  w/s : speed up/down
  a/d : steer left/right
  space : stop speed and steering
  q/e : decrease/increase speed step
  z/c : decrease/increase steering step
  x : stop
  Ctrl+C : quit
"""


class WasdDrive(Node):
    def __init__(self):
        super().__init__('wasd_drive')
        self.declare_parameter('motor_topic', '/xycar_motor')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('speed_step', 2.0)
        self.declare_parameter('angle_step', 10.0)
        self.declare_parameter('max_speed', 30.0)
        self.declare_parameter('max_angle', 80.0)

        self.motor_topic = self.get_parameter('motor_topic').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.speed_step = float(self.get_parameter('speed_step').value)
        self.angle_step = float(self.get_parameter('angle_step').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_angle = float(self.get_parameter('max_angle').value)

        self.speed = 0.0
        self.angle = 0.0
        self.msg = Float32MultiArray()
        self.pub = self.create_publisher(Float32MultiArray, self.motor_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_drive)
        self.get_logger().info(
            f'wasd_drive ready: topic={self.motor_topic}, '
            f'max_speed={self.max_speed}, max_angle={self.max_angle}'
        )

    def clamp(self):
        self.speed = max(-self.max_speed, min(self.max_speed, self.speed))
        self.angle = max(-self.max_angle, min(self.max_angle, self.angle))

    def publish_drive(self):
        self.clamp()
        self.msg.data = [float(self.angle), float(self.speed)]
        self.pub.publish(self.msg)

    def stop(self):
        self.speed = 0.0
        self.angle = 0.0
        self.publish_drive()

    def handle_key(self, key):
        if key == 'w':
            self.speed += self.speed_step
        elif key == 's':
            self.speed -= self.speed_step
        elif key == 'a':
            self.angle -= self.angle_step
        elif key == 'd':
            self.angle += self.angle_step
        elif key == ' ':
            self.stop()
        elif key == 'x':
            self.stop()
        elif key == 'q':
            self.speed_step = max(0.5, self.speed_step - 0.5)
        elif key == 'e':
            self.speed_step += 0.5
        elif key == 'z':
            self.angle_step = max(1.0, self.angle_step - 1.0)
        elif key == 'c':
            self.angle_step += 1.0
        self.clamp()
        self.print_status()

    def print_status(self):
        sys.stdout.write(
            f'\rangle={self.angle:6.1f} speed={self.speed:6.1f} '
            f'speed_step={self.speed_step:4.1f} angle_step={self.angle_step:4.1f}   '
        )
        sys.stdout.flush()


def read_key(timeout=0.05):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    return sys.stdin.read(1)


def main(args=None):
    rclpy.init(args=args)
    node = WasdDrive()
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        os.system('clear')
        print(HELP)
        node.print_status()
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key()
            if key:
                node.handle_key(key.lower())
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nStopped.')


if __name__ == '__main__':
    main()
