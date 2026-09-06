import math

from custom_interfaces.msg import ClusterData
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
from std_msgs.msg import String


MAX_RANGE = 1.5
MIN_RANGE_THRESHOLD = 0.18
DBSCAN_EPS = 0.15
DBSCAN_MIN_SAMPLES = 5
MAX_CONE_DIAMETER = 0.3
FILTER_DEGREE = 36


def filter_front_clusters_by_angle(cluster_centers, degree_bin):
    def angle_bin(theta):
        return int(np.rad2deg(theta) // degree_bin)

    used_bins = set()
    front_cone_clusters = []
    for center in sorted(
            cluster_centers, key=lambda point: np.hypot(point[0], point[1])):
        x, y = center
        bin_index = angle_bin(math.atan2(y, x))
        if bin_index not in used_bins:
            front_cone_clusters.append(center)
            used_bins.add(bin_index)
    return front_cone_clusters


class PreprocessingNode(Node):
    def __init__(self):
        super().__init__('preprocessing_node')

        self.declare_parameter('scan_topic', '/scan_rotated')
        self.declare_parameter('clusters_topic', '/clusters')
        self.declare_parameter('activation_topic', '/sign_color')
        self.declare_parameter('require_green_signal', True)

        self.require_green_signal = self.as_bool(
            self.get_parameter('require_green_signal').value)
        self.is_active = not self.require_green_signal

        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.lidar_callback,
            qos_profile_sensor_data,
        )
        self.activation_sub = self.create_subscription(
            String,
            self.get_parameter('activation_topic').value,
            self.activation_callback,
            10,
        )
        self.cluster_pub = self.create_publisher(
            ClusterData,
            self.get_parameter('clusters_topic').value,
            10,
        )

        self.get_logger().info(
            'Cone preprocessing started: '
            f'range={MIN_RANGE_THRESHOLD:.2f}-{MAX_RANGE:.2f}m, '
            f'DBSCAN eps={DBSCAN_EPS:.2f}m/'
            f'min_samples={DBSCAN_MIN_SAMPLES}, '
            f'max_diameter={MAX_CONE_DIAMETER:.2f}m, '
            f'angle_bin={FILTER_DEGREE}deg'
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
            self.publish_clusters([])

    def lidar_callback(self, msg):
        if not self.is_active:
            return

        scan_data = np.asarray(msg.ranges, dtype=np.float32)
        angles = np.linspace(
            msg.angle_min,
            msg.angle_min + msg.angle_increment * (len(scan_data) - 1),
            len(scan_data),
        )
        x_all = scan_data * np.cos(angles)
        y_all = scan_data * np.sin(angles)
        front_mask = (
            (angles >= -np.pi / 2.0) & (angles <= np.pi / 2.0)
        )
        valid = (
            np.isfinite(scan_data)
            & (scan_data <= MAX_RANGE)
            & (scan_data > MIN_RANGE_THRESHOLD)
            & front_mask
        )
        if not np.any(valid):
            self.publish_clusters([])
            return

        points = np.column_stack((x_all[valid], y_all[valid]))
        if points.shape[0] < DBSCAN_MIN_SAMPLES:
            self.publish_clusters([])
            return

        labels = DBSCAN(
            eps=DBSCAN_EPS,
            min_samples=DBSCAN_MIN_SAMPLES,
        ).fit(points).labels_
        cluster_centers = []
        for label in set(labels):
            if label == -1:
                continue
            cluster_points = points[labels == label]
            if len(cluster_points) > 1:
                diameter = np.hypot(
                    np.max(cluster_points[:, 0])
                    - np.min(cluster_points[:, 0]),
                    np.max(cluster_points[:, 1])
                    - np.min(cluster_points[:, 1]),
                )
                if diameter > MAX_CONE_DIAMETER:
                    continue
            distances = np.hypot(
                cluster_points[:, 0], cluster_points[:, 1])
            nearest = cluster_points[int(np.argmin(distances))]
            cluster_centers.append((nearest[0], nearest[1]))

        filtered_centers = filter_front_clusters_by_angle(
            cluster_centers,
            FILTER_DEGREE,
        )
        self.publish_clusters(filtered_centers)

    def publish_clusters(self, centers):
        msg = ClusterData()
        msg.clusters = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in centers
        ]
        self.cluster_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PreprocessingNode()
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
