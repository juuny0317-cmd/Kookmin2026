import math

import numpy as np
import rclpy
from custom_interfaces.msg import ClusterData, ConeData
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from mission_cone_drive.lidar_corridor_filter import LidarCorridorFilter


class LidarConeFilterNode(Node):
    """Produce cone corridor candidates from LiDAR without camera input."""

    def __init__(self):
        super().__init__('lidar_cone_filter_node')

        self.declare_parameter('scan_topic', '/scan_rotated')
        self.declare_parameter('raw_clusters_topic', '/clusters_raw')
        self.declare_parameter('output_clusters_topic', '/clusters')
        self.declare_parameter(
            'corridor_cones_topic', '/lidar_corridor_cones')
        self.declare_parameter(
            'corridor_midpoints_topic', '/lidar_corridor_midpoints')
        self.declare_parameter('status_topic', '/cone_lidar_status')

        self.declare_parameter('min_range_m', 0.23)
        self.declare_parameter('max_range_m', 1.80)
        self.declare_parameter('x_min_m', 0.08)
        self.declare_parameter('x_max_m', 1.80)
        self.declare_parameter('y_abs_max_m', 0.80)
        self.declare_parameter('cluster_euclidean_gap_m', 0.12)
        self.declare_parameter('cluster_index_gap', 3)
        self.declare_parameter('cluster_min_points', 2)
        self.declare_parameter('cluster_min_chord_m', 0.02)
        self.declare_parameter('cluster_max_radius_m', 0.12)
        self.declare_parameter('cluster_max_chord_m', 0.24)
        self.declare_parameter('dedupe_radius_m', 0.08)

        self.declare_parameter('gap_min_width_m', 0.50)
        self.declare_parameter('gap_max_width_m', 1.00)
        self.declare_parameter('gap_expected_width_m', 0.80)
        self.declare_parameter('cone_spacing_m', 0.30)
        self.declare_parameter('pair_min_normal_alignment', 0.45)
        self.declare_parameter('pair_max_longitudinal_m', 0.45)
        self.declare_parameter('midpoint_reference_gate_m', 0.45)
        self.declare_parameter('association_distance_m', 0.22)
        self.declare_parameter('min_track_hits', 2)
        self.declare_parameter('max_track_misses', 2)
        self.declare_parameter('track_latest_weight', 0.80)
        self.declare_parameter('min_entry_pairs', 2)
        self.declare_parameter('min_pair_span_m', 0.18)
        self.declare_parameter('max_entry_width_spread_m', 0.20)
        self.declare_parameter('one_side_half_width_m', 0.40)
        self.declare_parameter('boundary_tolerance_m', 0.18)
        self.declare_parameter('min_one_side_points', 2)
        self.declare_parameter('two_point_support_confirm_frames', 2)
        self.declare_parameter('established_hold_frames', 10)

        self.min_range_m = self._float_parameter('min_range_m')
        self.max_range_m = self._float_parameter('max_range_m')
        self.x_min_m = self._float_parameter('x_min_m')
        self.x_max_m = self._float_parameter('x_max_m')
        self.y_abs_max_m = self._float_parameter('y_abs_max_m')
        self.cluster_euclidean_gap_m = self._float_parameter(
            'cluster_euclidean_gap_m')
        self.cluster_index_gap = self._int_parameter('cluster_index_gap')
        self.cluster_min_points = self._int_parameter('cluster_min_points')
        self.cluster_min_chord_m = self._float_parameter(
            'cluster_min_chord_m')
        self.cluster_max_radius_m = self._float_parameter(
            'cluster_max_radius_m')
        self.cluster_max_chord_m = self._float_parameter(
            'cluster_max_chord_m')
        self.dedupe_radius_m = self._float_parameter('dedupe_radius_m')

        self.filter = LidarCorridorFilter(
            min_width_m=self._float_parameter('gap_min_width_m'),
            max_width_m=self._float_parameter('gap_max_width_m'),
            expected_width_m=self._float_parameter('gap_expected_width_m'),
            cone_spacing_m=self._float_parameter('cone_spacing_m'),
            min_normal_alignment=self._float_parameter(
                'pair_min_normal_alignment'),
            max_pair_longitudinal_m=self._float_parameter(
                'pair_max_longitudinal_m'),
            midpoint_reference_gate_m=self._float_parameter(
                'midpoint_reference_gate_m'),
            association_distance_m=self._float_parameter(
                'association_distance_m'),
            min_track_hits=self._int_parameter('min_track_hits'),
            max_track_misses=self._int_parameter('max_track_misses'),
            track_latest_weight=self._float_parameter(
                'track_latest_weight'),
            min_entry_pairs=self._int_parameter('min_entry_pairs'),
            min_pair_span_m=self._float_parameter('min_pair_span_m'),
            max_entry_width_spread_m=self._float_parameter(
                'max_entry_width_spread_m'),
            one_side_half_width_m=self._float_parameter(
                'one_side_half_width_m'),
            boundary_tolerance_m=self._float_parameter(
                'boundary_tolerance_m'),
            min_one_side_points=self._int_parameter(
                'min_one_side_points'),
            two_point_support_confirm_frames=self._int_parameter(
                'two_point_support_confirm_frames'),
            established_hold_frames=self._int_parameter(
                'established_hold_frames'),
        )

        self.raw_pub = self.create_publisher(
            ClusterData,
            self.get_parameter('raw_clusters_topic').value,
            10,
        )
        self.output_pub = self.create_publisher(
            ClusterData,
            self.get_parameter('output_clusters_topic').value,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            10,
        )
        self.corridor_cones_pub = self.create_publisher(
            ConeData,
            self.get_parameter('corridor_cones_topic').value,
            10,
        )
        self.corridor_midpoints_pub = self.create_publisher(
            ClusterData,
            self.get_parameter('corridor_midpoints_topic').value,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.last_filter_state = ''

        self.get_logger().info(
            'LiDAR-only cone filter started: temporal tracking + '
            'corridor geometry, no image subscription.')

    def _float_parameter(self, name):
        return float(self.get_parameter(name).value)

    def _int_parameter(self, name):
        return int(self.get_parameter(name).value)

    def scan_callback(self, msg):
        candidates = self.extract_cluster_centers(msg)
        result = self.filter.update(candidates)
        self.publish_clusters(self.raw_pub, candidates)
        self.publish_clusters(self.output_pub, result.accepted_cones)
        self.publish_cones(result.left_cones, result.right_cones)
        self.publish_clusters(
            self.corridor_midpoints_pub, result.midpoints)
        self.status_pub.publish(String(data=(
            f'state={result.state} raw={len(candidates)} '
            f'tracked={len(result.tracked_candidates)} '
            f'pairs={result.pair_count} '
            f'accepted={len(result.accepted_cones)}'
        )))
        if result.state != self.last_filter_state:
            self.get_logger().info(
                f'LiDAR corridor state: {self.last_filter_state or "startup"} '
                f'-> {result.state}; raw={len(candidates)}, '
                f'tracked={len(result.tracked_candidates)}, '
                f'pairs={result.pair_count}, '
                f'accepted={len(result.accepted_cones)}'
            )
            self.last_filter_state = result.state

    def extract_cluster_centers(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return []

        angles = (
            msg.angle_min
            + np.arange(ranges.size, dtype=np.float32)
            * msg.angle_increment
        )
        valid = (
            np.isfinite(ranges)
            & (ranges >= self.min_range_m)
            & (ranges <= self.max_range_m)
        )
        points = []
        for index in np.where(valid)[0]:
            distance = float(ranges[index])
            angle = float(angles[index])
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if x < self.x_min_m or x > self.x_max_m:
                continue
            if abs(y) > self.y_abs_max_m:
                continue
            points.append((int(index), x, y))

        if not points:
            return []
        groups = self.group_scan_neighbors(points)
        centers = self.cluster_centers(groups)
        return self.remove_near_duplicates(centers)

    def group_scan_neighbors(self, points):
        groups = []
        current = [points[0]]
        for point in points[1:]:
            previous = current[-1]
            index_gap = point[0] - previous[0]
            euclidean_gap = math.hypot(
                point[1] - previous[1],
                point[2] - previous[2],
            )
            if (
                index_gap <= self.cluster_index_gap
                and euclidean_gap <= self.cluster_euclidean_gap_m
            ):
                current.append(point)
            else:
                groups.append(current)
                current = [point]
        groups.append(current)
        return groups

    def cluster_centers(self, groups):
        centers = []
        for group in groups:
            if len(group) < self.cluster_min_points:
                continue
            xs = np.asarray([point[1] for point in group], dtype=np.float32)
            ys = np.asarray([point[2] for point in group], dtype=np.float32)
            center_x = float(np.median(xs))
            center_y = float(np.median(ys))
            radius = float(np.max(np.hypot(
                xs - center_x,
                ys - center_y,
            )))
            chord = math.hypot(
                float(xs[-1] - xs[0]),
                float(ys[-1] - ys[0]),
            )
            if radius > self.cluster_max_radius_m:
                continue
            if chord < self.cluster_min_chord_m:
                continue
            if chord > self.cluster_max_chord_m:
                continue
            centers.append((center_x, center_y))
        return centers

    def remove_near_duplicates(self, centers):
        selected = []
        for center in sorted(
            centers,
            key=lambda point: math.hypot(point[0], point[1]),
        ):
            if any(
                math.hypot(
                    center[0] - previous[0],
                    center[1] - previous[1],
                ) < self.dedupe_radius_m
                for previous in selected
            ):
                continue
            selected.append(center)
        return sorted(selected, key=lambda point: point[0])

    @staticmethod
    def publish_clusters(publisher, centers):
        message = ClusterData()
        message.clusters = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in centers
        ]
        publisher.publish(message)

    def publish_cones(self, left_cones, right_cones):
        message = ConeData()
        message.left_cones = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in left_cones
        ]
        message.right_cones = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in right_cones
        ]
        self.corridor_cones_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LidarConeFilterNode()
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
