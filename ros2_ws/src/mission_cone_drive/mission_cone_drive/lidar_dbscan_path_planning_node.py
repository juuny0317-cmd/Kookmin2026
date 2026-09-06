from custom_interfaces.msg import ClusterData, ConeData
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.interpolate import CubicSpline
from std_msgs.msg import String


LEFT_FIRST_CONE_RANGE = 0.8
RIGHT_FIRST_CONE_RANGE = 0.8
LEFT_SECTOR_START = 30.0
LEFT_SECTOR_END = 100.0
RIGHT_SECTOR_START = 260.0
RIGHT_SECTOR_END = 350.0
GROW_DISTANCE = 0.5
MAX_PAIR_DISTANCE = 1.3
VIRTUAL_CONE_X = 0.1
VIRTUAL_HALF_WIDTH = 0.5
LIDAR_TO_REAR_AXLE = 0.42
PATH_POINTS = 100


class LidarDbscanPathPlanningNode(Node):
    def __init__(self):
        super().__init__('lidar_dbscan_path_planning_node')

        self.declare_parameter('clusters_topic', '/clusters')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('activation_topic', '/sign_color')
        self.declare_parameter('require_green_signal', True)
        self.declare_parameter('path_frame_id', 'rear_axle')

        self.require_green_signal = self.as_bool(
            self.get_parameter('require_green_signal').value)
        self.is_active = not self.require_green_signal
        self.path_frame_id = str(
            self.get_parameter('path_frame_id').value)
        self.prev_interp_x = None
        self.prev_interp_y = None

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
        self.cone_pub = self.create_publisher(
            ConeData, '/cone_clusters', 10)
        self.midpoint_pub = self.create_publisher(
            ClusterData, '/midpoints', 10)

        self.get_logger().info(
            'Cone path planner started: '
            f'seed={LEFT_FIRST_CONE_RANGE:.1f}m, '
            f'grow={GROW_DISTANCE:.1f}m, '
            f'max_pair={MAX_PAIR_DISTANCE:.1f}m, '
            f'path_points={PATH_POINTS}'
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
            self.publish_result([], [], [], [])

    def cluster_callback(self, msg):
        if not self.is_active:
            return

        cluster_centers = [
            (float(point.x), float(point.y))
            for point in msg.clusters
        ]
        left_cones, right_cones = self.form_cone_groups(cluster_centers)

        if not left_cones and len(right_cones) > 1:
            left_cones.append((VIRTUAL_CONE_X, VIRTUAL_HALF_WIDTH))
        elif not right_cones and len(left_cones) > 1:
            right_cones.append((VIRTUAL_CONE_X, -VIRTUAL_HALF_WIDTH))

        midpoints = self.calculate_midpoints_from_final_cones(
            left_cones, right_cones)
        path = self.build_path(midpoints)
        self.publish_result(left_cones, right_cones, midpoints, path)

    def form_cone_groups(self, cluster_centers):
        left_seed = None
        right_seed = None
        min_left_range = float('inf')
        min_right_range = float('inf')

        for point in cluster_centers:
            distance = np.hypot(point[0], point[1])
            angle_deg = np.rad2deg(np.arctan2(point[1], point[0]))
            if angle_deg < 0.0:
                angle_deg += 360.0

            if (
                    LEFT_SECTOR_START <= angle_deg <= LEFT_SECTOR_END
                    and distance <= LEFT_FIRST_CONE_RANGE):
                if distance < min_left_range:
                    min_left_range = distance
                    left_seed = point
            elif (
                    RIGHT_SECTOR_START <= angle_deg <= RIGHT_SECTOR_END
                    and distance <= RIGHT_FIRST_CONE_RANGE):
                if distance < min_right_range:
                    min_right_range = distance
                    right_seed = point

        used_clusters = set()
        left_cones = self.grow_clusters(
            [left_seed], cluster_centers, used_clusters
        ) if left_seed else []
        right_cones = self.grow_clusters(
            [right_seed], cluster_centers, used_clusters
        ) if right_seed else []
        return left_cones, right_cones

    @staticmethod
    def grow_clusters(seed_list, all_clusters, used_clusters_set):
        if not seed_list or tuple(seed_list[0]) in used_clusters_set:
            return []

        grown = [seed_list[0]]
        queue = [seed_list[0]]
        used_clusters_set.add(tuple(seed_list[0]))
        head = 0
        while head < len(queue):
            base = queue[head]
            head += 1
            for point in all_clusters:
                point_tuple = tuple(point)
                if point_tuple in used_clusters_set:
                    continue
                if np.hypot(
                        point[0] - base[0],
                        point[1] - base[1]) <= GROW_DISTANCE:
                    grown.append(point)
                    queue.append(point)
                    used_clusters_set.add(point_tuple)
        return grown

    def calculate_midpoints_from_final_cones(
            self, left_cones, right_cones):
        if len(left_cones) >= 2 and len(right_cones) >= 2:
            return self.calculate_midpoints_pair(left_cones, right_cones)
        if len(left_cones) == 1 and len(right_cones) > 1:
            return [
                (
                    (left_cones[0][0] + right[0]) / 2.0,
                    (left_cones[0][1] + right[1]) / 2.0,
                )
                for right in right_cones
            ]
        if len(right_cones) == 1 and len(left_cones) > 1:
            return [
                (
                    (left[0] + right_cones[0][0]) / 2.0,
                    (left[1] + right_cones[0][1]) / 2.0,
                )
                for left in left_cones
            ]
        if len(left_cones) == 1 and len(right_cones) == 1:
            return [(
                (left_cones[0][0] + right_cones[0][0]) / 2.0,
                (left_cones[0][1] + right_cones[0][1]) / 2.0,
            )]
        return []

    @staticmethod
    def calculate_midpoints_pair(left_cones, right_cones):
        midpoints = []
        left_sorted = sorted(left_cones, key=lambda point: point[0])
        right_sorted = sorted(right_cones, key=lambda point: point[0])
        if not left_sorted or not right_sorted:
            return midpoints

        for left in left_sorted:
            candidates = [
                (abs(left[1] - right[1]), right)
                for right in right_sorted
            ]
            if not candidates:
                continue
            _, best_right = min(candidates)
            distance = np.hypot(
                left[0] - best_right[0],
                left[1] - best_right[1],
            )
            if distance > MAX_PAIR_DISTANCE:
                continue
            midpoints.append((
                (left[0] + best_right[0]) / 2.0,
                (left[1] + best_right[1]) / 2.0,
            ))
        return midpoints

    def interpolate_path(self, midpoints):
        if len(midpoints) < 2:
            return self.prev_interp_x, self.prev_interp_y

        ordered = sorted(midpoints, key=lambda point: point[0])
        xs = np.asarray([point[0] for point in ordered])
        ys = np.asarray([point[1] for point in ordered])
        xs, unique_indices = np.unique(xs, return_index=True)
        ys = ys[unique_indices]
        if len(xs) < 2:
            return self.prev_interp_x, self.prev_interp_y

        spline = CubicSpline(xs, ys)
        interp_x = np.linspace(xs.min(), xs.max(), PATH_POINTS)
        interp_y = spline(interp_x)
        self.prev_interp_x = interp_x
        self.prev_interp_y = interp_y
        return interp_x, interp_y

    def build_path(self, midpoints):
        if len(midpoints) >= 2:
            rear_axle_midpoints = [
                (point[0] + LIDAR_TO_REAR_AXLE, point[1])
                for point in midpoints
            ]
            interp_x, interp_y = self.interpolate_path(rear_axle_midpoints)
            if interp_x is not None:
                return list(zip(interp_x, interp_y))
        elif len(midpoints) == 1:
            return [(
                midpoints[0][0] + LIDAR_TO_REAR_AXLE,
                midpoints[0][1],
            )]
        return []

    def publish_result(self, left, right, midpoints, path):
        cone_msg = ConeData()
        cone_msg.left_cones = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in left
        ]
        cone_msg.right_cones = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in right
        ]
        self.cone_pub.publish(cone_msg)

        midpoint_msg = ClusterData()
        midpoint_msg.clusters = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in midpoints
        ]
        self.midpoint_pub.publish(midpoint_msg)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.path_frame_id
        for x, y in path:
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDbscanPathPlanningNode()
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
