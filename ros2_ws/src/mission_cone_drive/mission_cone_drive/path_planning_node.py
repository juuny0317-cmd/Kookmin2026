import numpy as np
import rclpy
from custom_interfaces.msg import ClusterData, ConeData
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from mission_cone_drive.corridor_geometry import CorridorGeometry


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        self.declare_parameter('require_green_signal', True)
        self.declare_parameter('clusters_topic', '/clusters')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('activation_topic', '/sign_color')
        self.declare_parameter('path_frame_id', 'laser_frame')
        self.declare_parameter('use_precomputed_corridor', False)
        self.declare_parameter(
            'preclassified_cones_topic', '/lidar_corridor_cones')
        self.declare_parameter(
            'precomputed_midpoints_topic', '/lidar_corridor_midpoints')

        self.declare_parameter('x_min_m', 0.15)
        self.declare_parameter('x_max_m', 2.20)
        self.declare_parameter('side_split_y_m', 0.04)
        self.declare_parameter('y_abs_max_m', 1.20)

        self.declare_parameter('gap_min_width_m', 0.35)
        self.declare_parameter('gap_max_width_m', 1.15)
        self.declare_parameter('gap_expected_width_m', 0.65)
        self.declare_parameter('cone_spacing_m', 0.30)
        self.declare_parameter('pair_min_normal_alignment', 0.45)
        self.declare_parameter('pair_max_longitudinal_m', 0.45)
        self.declare_parameter('midpoint_reference_gate_m', 0.45)

        self.declare_parameter('one_side_half_width_m', 0.40)
        self.declare_parameter('one_side_min_points', 2)
        self.declare_parameter('path_points', 30)
        self.declare_parameter('path_lost_keep_frames', 3)
        self.declare_parameter('path_smoothing_alpha', 0.75)
        self.declare_parameter('max_path_slope', 1.60)
        self.declare_parameter('min_midpoints_for_path', 2)
        self.declare_parameter('min_path_span_m', 0.35)
        self.declare_parameter('max_path_lateral_jump_m', 0.45)

        self.require_green_signal = self.as_bool(
            self.get_parameter('require_green_signal').value)
        self.is_active = not self.require_green_signal

        self.path_frame_id = str(
            self.get_parameter('path_frame_id').value)
        self.x_min_m = float(self.get_parameter('x_min_m').value)
        self.x_max_m = float(self.get_parameter('x_max_m').value)
        self.side_split_y_m = float(
            self.get_parameter('side_split_y_m').value)
        self.y_abs_max_m = float(self.get_parameter('y_abs_max_m').value)

        self.gap_min_width_m = float(
            self.get_parameter('gap_min_width_m').value)
        self.gap_max_width_m = float(
            self.get_parameter('gap_max_width_m').value)
        self.gap_expected_width_m = float(
            self.get_parameter('gap_expected_width_m').value)
        self.cone_spacing_m = float(
            self.get_parameter('cone_spacing_m').value)
        self.pair_min_normal_alignment = float(
            self.get_parameter('pair_min_normal_alignment').value)
        self.pair_max_longitudinal_m = float(
            self.get_parameter('pair_max_longitudinal_m').value)
        self.midpoint_reference_gate_m = float(
            self.get_parameter('midpoint_reference_gate_m').value)

        self.one_side_half_width_m = float(
            self.get_parameter('one_side_half_width_m').value)
        self.one_side_min_points = int(
            self.get_parameter('one_side_min_points').value)
        self.path_points = int(self.get_parameter('path_points').value)
        self.path_lost_keep_frames = int(
            self.get_parameter('path_lost_keep_frames').value)
        self.path_smoothing_alpha = float(
            self.get_parameter('path_smoothing_alpha').value)
        self.max_path_slope = float(self.get_parameter('max_path_slope').value)
        self.min_midpoints_for_path = int(
            self.get_parameter('min_midpoints_for_path').value)
        self.min_path_span_m = float(
            self.get_parameter('min_path_span_m').value)
        self.max_path_lateral_jump_m = float(
            self.get_parameter('max_path_lateral_jump_m').value
        )

        self.corridor_geometry = CorridorGeometry(
            min_width_m=self.gap_min_width_m,
            max_width_m=self.gap_max_width_m,
            expected_width_m=self.gap_expected_width_m,
            cone_spacing_m=self.cone_spacing_m,
            side_deadband_m=self.side_split_y_m,
            min_normal_alignment=self.pair_min_normal_alignment,
            max_pair_longitudinal_m=self.pair_max_longitudinal_m,
            midpoint_reference_gate_m=self.midpoint_reference_gate_m,
        )

        self.prev_path = []
        self.prev_midpoints = []
        self.lost_frames = 0
        self.latest_preclassified_left = []
        self.latest_preclassified_right = []

        self.use_precomputed_corridor = self.as_bool(
            self.get_parameter('use_precomputed_corridor').value)
        if self.use_precomputed_corridor:
            self.preclassified_cones_sub = self.create_subscription(
                ConeData,
                self.get_parameter('preclassified_cones_topic').value,
                self.preclassified_cones_callback,
                10,
            )
            self.precomputed_midpoints_sub = self.create_subscription(
                ClusterData,
                self.get_parameter('precomputed_midpoints_topic').value,
                self.precomputed_midpoints_callback,
                10,
            )
        else:
            self.cluster_sub = self.create_subscription(
                ClusterData,
                self.get_parameter('clusters_topic').value,
                self.cluster_callback,
                10,
            )
        self.activation_sub = self.create_subscription(
            String,
            self.get_parameter('activation_topic').value,
            self.activation_callback,
            10,
        )
        self.path_pub = self.create_publisher(
            Path,
            self.get_parameter('path_topic').value,
            10,
        )
        self.cone_pub = self.create_publisher(ConeData, '/cone_clusters', 10)
        self.midpoint_pub = self.create_publisher(
            ClusterData, '/midpoints', 10)

        mode = (
            'waiting for green signal'
            if self.require_green_signal
            else 'active immediately'
        )
        self.get_logger().info(
            f'Cone path planner started ({mode}). '
            f'precomputed_corridor={self.use_precomputed_corridor}, '
            f'corridor={self.gap_min_width_m:.2f}-'
            f'{self.gap_max_width_m:.2f}m, '
            f'expected={self.gap_expected_width_m:.2f}m, '
            f'cone_spacing={self.cone_spacing_m:.2f}m'
        )

    def activation_callback(self, msg: String):
        state = msg.data.strip().lower()
        if state == 'green':
            if not self.is_active:
                self.get_logger().info(
                    'Cone path planner activated by green signal.')
            self.is_active = True
        elif state == 'red':
            if self.is_active:
                self.get_logger().warn(
                    'Cone path planner paused by red signal.')
            self.is_active = False
            self.prev_path = []
            self.prev_midpoints = []
            self.lost_frames = 0
            self.publish_cones([], [])
            self.publish_midpoints([])
            self.publish_path([])

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def cluster_callback(self, msg: ClusterData):
        if not self.is_active:
            return

        centers = self.filter_centers(msg.clusters)
        observation = self.corridor_geometry.extract(centers, self.prev_path)
        left_cones = observation.left_cones
        right_cones = observation.right_cones
        self.publish_cones(left_cones, right_cones)

        midpoints = list(observation.paired_midpoints)
        if len(midpoints) < self.min_midpoints_for_path:
            one_side_midpoints = (
                self.corridor_geometry.estimate_center_from_one_side(
                    left_cones=left_cones,
                    right_cones=right_cones,
                    reference_path=self.prev_path,
                    half_width_m=self.one_side_half_width_m,
                    min_points=self.one_side_min_points,
                )
            )
            if one_side_midpoints:
                midpoints.extend(one_side_midpoints)

        self.process_midpoints(midpoints)

    def preclassified_cones_callback(self, msg: ConeData):
        self.latest_preclassified_left = [
            (float(point.x), float(point.y))
            for point in msg.left_cones
        ]
        self.latest_preclassified_right = [
            (float(point.x), float(point.y))
            for point in msg.right_cones
        ]

    def precomputed_midpoints_callback(self, msg: ClusterData):
        if not self.is_active:
            return

        self.publish_cones(
            self.latest_preclassified_left,
            self.latest_preclassified_right,
        )
        midpoints = self.filter_centers(msg.clusters)
        self.process_midpoints(midpoints)

    def process_midpoints(self, midpoints):
        midpoints = self.clean_midpoints(midpoints)
        path_points = self.interpolate_path(midpoints)
        path_is_reliable = self.path_is_reliable(midpoints, path_points)

        if path_points and path_is_reliable:
            self.lost_frames = 0
            self.prev_path = path_points
            self.prev_midpoints = midpoints
        elif self.prev_path and self.lost_frames < self.path_lost_keep_frames:
            self.lost_frames += 1
            path_points = self.prev_path
            midpoints = self.prev_midpoints
        else:
            self.lost_frames = min(
                self.lost_frames + 1,
                self.path_lost_keep_frames,
            )

        self.publish_midpoints(midpoints)
        self.publish_path(path_points)

    def filter_centers(self, points):
        centers = []
        for point in points:
            x = float(point.x)
            y = float(point.y)
            if x < self.x_min_m or x > self.x_max_m:
                continue
            if abs(y) > self.y_abs_max_m:
                continue
            centers.append((x, y))
        return sorted(centers, key=lambda p: (p[0], p[1]))

    def clean_midpoints(self, midpoints):
        if not midpoints:
            return []

        sorted_points = sorted(midpoints, key=lambda p: p[0])
        deduped = []
        for point in sorted_points:
            if deduped and abs(point[0] - deduped[-1][0]) < 0.08:
                prev = deduped[-1]
                deduped[-1] = (
                    (prev[0] + point[0]) * 0.5,
                    (prev[1] + point[1]) * 0.5,
                )
            else:
                deduped.append(point)

        cleaned = []
        for point in deduped:
            if not cleaned:
                cleaned.append(point)
                continue
            dx = max(point[0] - cleaned[-1][0], 0.05)
            max_dy = 0.10 + self.max_path_slope * dx
            if abs(point[1] - cleaned[-1][1]) <= max_dy:
                cleaned.append(point)

        return cleaned

    def interpolate_path(self, midpoints):
        if not midpoints:
            return []

        if len(midpoints) < self.min_midpoints_for_path:
            return []

        xs = np.asarray([p[0] for p in midpoints], dtype=np.float32)
        ys = np.asarray([p[1] for p in midpoints], dtype=np.float32)
        xs, unique_idx = np.unique(xs, return_index=True)
        ys = ys[unique_idx]
        if xs.size < 2:
            return []

        count = max(2, self.path_points)
        interp_x = np.linspace(float(xs.min()), float(xs.max()), count)
        interp_y = np.interp(interp_x, xs, ys)
        path = [(float(x), float(y)) for x, y in zip(interp_x, interp_y)]
        return self.smooth_path(path)

    def path_is_reliable(self, midpoints, path):
        if not path:
            return False
        if len(midpoints) < self.min_midpoints_for_path:
            return False

        xs = [float(p[0]) for p in midpoints]
        if max(xs) - min(xs) < self.min_path_span_m:
            return False

        if self.prev_path:
            first_x, first_y = path[0]
            predicted_y = self.predict_path_y(first_x)
            if abs(first_y - predicted_y) > self.max_path_lateral_jump_m:
                return False

        return True

    def smooth_path(self, path):
        if not self.prev_path or not path:
            return path

        prev_xs = np.asarray([p[0] for p in self.prev_path], dtype=np.float32)
        prev_ys = np.asarray([p[1] for p in self.prev_path], dtype=np.float32)
        alpha = float(np.clip(self.path_smoothing_alpha, 0.0, 1.0))
        smoothed = []
        for x, y in path:
            prev_y = float(np.interp(x, prev_xs, prev_ys))
            smoothed.append((x, alpha * y + (1.0 - alpha) * prev_y))
        return smoothed

    def predict_path_y(self, x):
        if not self.prev_path:
            return 0.0
        xs = np.asarray([p[0] for p in self.prev_path], dtype=np.float32)
        ys = np.asarray([p[1] for p in self.prev_path], dtype=np.float32)
        return float(np.interp(float(x), xs, ys))

    def publish_cones(self, left_cones, right_cones):
        msg = ConeData()
        msg.left_cones = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in left_cones
        ]
        msg.right_cones = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in right_cones
        ]
        self.cone_pub.publish(msg)

    def publish_midpoints(self, midpoints):
        msg = ClusterData()
        msg.clusters = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in midpoints
        ]
        self.midpoint_pub.publish(msg)

    def publish_path(self, points):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.path_frame_id
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
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
