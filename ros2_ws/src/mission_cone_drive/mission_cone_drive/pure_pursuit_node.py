from geometry_msgs.msg import PointStamped
import math
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


WHEELBASE = 0.33
MIN_LOOKAHEAD = 0.7
MAX_LOOKAHEAD = 2.5
LOOKAHEAD_SCALE = 0.25
MAX_SPEED = 20.0
MIN_SPEED = 10.0


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('motor_topic', '/cone_motor_cmd')
        self.declare_parameter('activation_topic', '/sign_color')
        self.declare_parameter('require_green_signal', True)
        self.declare_parameter('min_speed', MIN_SPEED)
        self.declare_parameter('max_speed', MAX_SPEED)

        self.require_green_signal = self.as_bool(
            self.get_parameter('require_green_signal').value)
        self.min_speed = max(
            0.0, float(self.get_parameter('min_speed').value))
        self.max_speed = max(
            self.min_speed,
            float(self.get_parameter('max_speed').value),
        )
        self.is_active = not self.require_green_signal
        self.path_frame_id = 'rear_axle'

        self.activation_sub = self.create_subscription(
            String,
            self.get_parameter('activation_topic').value,
            self.activation_callback,
            10,
        )
        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter('path_topic').value,
            self.path_callback,
            10,
        )
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('motor_topic').value,
            10,
        )
        self.target_pub = self.create_publisher(
            PointStamped, '/target_point', 10)

        self.get_logger().info(
            'Cone Pure Pursuit started: '
            f'lookahead={MIN_LOOKAHEAD:.1f}-{MAX_LOOKAHEAD:.1f}m, '
            f'scale={LOOKAHEAD_SCALE:.2f}, '
            f'speed={self.min_speed:.0f}-{self.max_speed:.0f}, '
            'direct steering'
        )

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def activation_callback(self, msg):
        state = msg.data.strip().lower()
        if state == 'green':
            self.is_active = True
        elif state == 'red' and self.require_green_signal:
            self.is_active = False

    def path_callback(self, msg):
        if not self.is_active:
            return

        angle_cmd = 0.0
        speed = self.min_speed
        target = None

        if len(msg.poses) > 1:
            interp_x = np.asarray([
                pose.pose.position.x for pose in msg.poses])
            interp_y = np.asarray([
                pose.pose.position.y for pose in msg.poses])
            lookahead = self.dynamic_lookahead_from_path(
                interp_x, interp_y)
            angle_cmd, target = self.pure_pursuit_control(
                interp_x, interp_y, lookahead)
            speed = self.compute_speed(
                angle_cmd, self.min_speed, self.max_speed)
        elif len(msg.poses) == 1:
            target = (
                msg.poses[0].pose.position.x,
                msg.poses[0].pose.position.y,
            )
            angle_cmd = self.single_target_control(target)
            speed = self.compute_speed(
                angle_cmd, self.min_speed, self.max_speed)

        self.publish_motor_command(angle_cmd, speed)
        if target is not None:
            if msg.header.frame_id:
                self.path_frame_id = msg.header.frame_id
            self.publish_target_point(target)

    @staticmethod
    def single_target_control(target):
        alpha = math.atan2(target[1], target[0])
        lookahead = np.hypot(target[0], target[1])
        if lookahead < 1e-6:
            return 0.0

        steering_rad = math.atan2(
            2.0 * WHEELBASE * math.sin(alpha), lookahead)
        angle_deg = -math.degrees(steering_rad)
        return float(np.clip(angle_deg / 0.2, -100.0, 100.0))

    def pure_pursuit_control(self, interp_x, interp_y, lookahead):
        distances = np.hypot(interp_x, interp_y)
        candidates = np.where(distances > lookahead)[0]
        if not candidates.any():
            target_x, target_y = interp_x[-1], interp_y[-1]
        else:
            best_index = candidates[0]
            target_x = interp_x[best_index]
            target_y = interp_y[best_index]

        angle_cmd = self.single_target_control((target_x, target_y))
        return angle_cmd, (target_x, target_y)

    @staticmethod
    def dynamic_lookahead_from_path(interp_x, interp_y):
        dx = np.diff(interp_x)
        dy = np.diff(interp_y)
        total_length = np.sum(np.hypot(dx, dy))
        return np.clip(
            LOOKAHEAD_SCALE * total_length,
            MIN_LOOKAHEAD,
            MAX_LOOKAHEAD,
        )

    @staticmethod
    def compute_speed(
            angle_cmd, min_speed=MIN_SPEED, max_speed=MAX_SPEED):
        minimum = max(0.0, float(min_speed))
        maximum = max(minimum, float(max_speed))
        normalized_angle = float(np.clip(abs(angle_cmd) / 100.0, 0.0, 1.0))
        return float(
            maximum - (maximum - minimum) * normalized_angle)

    def publish_motor_command(self, angle_cmd, speed):
        msg = Float32MultiArray()
        msg.data = [float(angle_cmd), float(speed)]
        self.motor_pub.publish(msg)

    def publish_target_point(self, target):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.path_frame_id
        msg.point.x = float(target[0])
        msg.point.y = float(target[1])
        self.target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
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
