import rclpy
from custom_interfaces.msg import ClusterData
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from mission_cone_drive.lidar_dbscan_algorithms import (
    LidarDbscanClusterConfig,
    extract_lidar_dbscan_clusters,
)


class LidarDbscanPreprocessingNode(Node):
    """Run DBSCAN and angular-sector LiDAR cone preprocessing."""

    def __init__(self):
        super().__init__('lidar_dbscan_preprocessing_node')

        self.declare_parameter('scan_topic', '/scan_rotated')
        self.declare_parameter('raw_clusters_topic', '/clusters_raw')
        self.declare_parameter('clusters_topic', '/clusters')
        self.declare_parameter('enable_prearm_output', False)
        self.declare_parameter(
            'prearm_raw_clusters_topic', '/prearm_clusters_raw')
        self.declare_parameter('activation_topic', '/sign_color')
        self.declare_parameter('require_green_signal', False)
        self.declare_parameter('min_range_m', 0.18)
        self.declare_parameter('max_range_m', 1.50)
        self.declare_parameter('dbscan_eps_m', 0.04)
        self.declare_parameter('dbscan_min_samples', 3)
        self.declare_parameter('max_cone_diameter_m', 0.30)
        self.declare_parameter('angle_bin_deg', 0.0)
        self.declare_parameter('min_cluster_separation_m', 0.15)
        self.declare_parameter('prearm_min_range_m', 0.18)
        self.declare_parameter('prearm_max_range_m', 3.00)
        self.declare_parameter('prearm_dbscan_eps_m', 0.15)
        self.declare_parameter('prearm_dbscan_min_samples', 3)
        self.declare_parameter('prearm_max_cone_diameter_m', 0.30)
        self.declare_parameter('prearm_angle_bin_deg', 0.0)
        self.declare_parameter('prearm_min_cluster_separation_m', 0.15)

        self.config = LidarDbscanClusterConfig(
            min_range_m=float(self.get_parameter('min_range_m').value),
            max_range_m=float(self.get_parameter('max_range_m').value),
            dbscan_eps_m=float(
                self.get_parameter('dbscan_eps_m').value),
            dbscan_min_samples=int(
                self.get_parameter('dbscan_min_samples').value),
            max_cone_diameter_m=float(
                self.get_parameter('max_cone_diameter_m').value),
            angle_bin_deg=float(
                self.get_parameter('angle_bin_deg').value),
            min_cluster_separation_m=float(
                self.get_parameter('min_cluster_separation_m').value),
        )
        self.require_green_signal = self.as_bool(
            self.get_parameter('require_green_signal').value)
        self.enable_prearm_output = self.as_bool(
            self.get_parameter('enable_prearm_output').value)
        self.prearm_config = LidarDbscanClusterConfig(
            min_range_m=float(
                self.get_parameter('prearm_min_range_m').value),
            max_range_m=float(
                self.get_parameter('prearm_max_range_m').value),
            dbscan_eps_m=float(
                self.get_parameter('prearm_dbscan_eps_m').value),
            dbscan_min_samples=int(
                self.get_parameter('prearm_dbscan_min_samples').value),
            max_cone_diameter_m=float(self.get_parameter(
                'prearm_max_cone_diameter_m').value),
            angle_bin_deg=float(
                self.get_parameter('prearm_angle_bin_deg').value),
            min_cluster_separation_m=float(self.get_parameter(
                'prearm_min_cluster_separation_m').value),
        )
        self.is_active = not self.require_green_signal

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.activation_sub = self.create_subscription(
            String,
            self.get_parameter('activation_topic').value,
            self.activation_callback,
            10,
        )
        self.raw_cluster_pub = self.create_publisher(
            ClusterData,
            self.get_parameter('raw_clusters_topic').value,
            10,
        )
        self.cluster_pub = self.create_publisher(
            ClusterData,
            self.get_parameter('clusters_topic').value,
            10,
        )
        self.prearm_raw_cluster_pub = None
        if self.enable_prearm_output:
            self.prearm_raw_cluster_pub = self.create_publisher(
                ClusterData,
                self.get_parameter('prearm_raw_clusters_topic').value,
                10,
            )

        activation = (
            'green-signal activation'
            if self.require_green_signal
            else 'continuous perception for mission FSM'
        )
        self.get_logger().info(
            'LiDAR DBSCAN cone preprocessing started: '
            f'{activation}, range={self.config.min_range_m:.2f}-'
            f'{self.config.max_range_m:.2f}m, '
            f'DBSCAN eps={self.config.dbscan_eps_m:.2f}m/'
            f'min_samples={self.config.dbscan_min_samples}, '
            f'min_separation={self.config.min_cluster_separation_m:.2f}m, '
            f'angle_bin={self.config.angle_bin_deg:.1f}deg, '
            f'prearm={self.enable_prearm_output}'
        )
        if self.enable_prearm_output:
            self.get_logger().info(
                'PRE-ARM DBSCAN: '
                f'range={self.prearm_config.min_range_m:.2f}-'
                f'{self.prearm_config.max_range_m:.2f}m, '
                f'eps={self.prearm_config.dbscan_eps_m:.2f}m/'
                f'min_samples={self.prearm_config.dbscan_min_samples}'
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
            self.publish_clusters(self.raw_cluster_pub, [])
            self.publish_clusters(self.cluster_pub, [])
            if self.prearm_raw_cluster_pub is not None:
                self.publish_clusters(self.prearm_raw_cluster_pub, [])

    def scan_callback(self, msg):
        if not self.is_active:
            return

        raw_centers, filtered_centers = extract_lidar_dbscan_clusters(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            self.config,
        )
        self.publish_clusters(self.raw_cluster_pub, raw_centers)
        self.publish_clusters(self.cluster_pub, filtered_centers)
        if self.prearm_raw_cluster_pub is not None:
            prearm_raw_centers, _ = extract_lidar_dbscan_clusters(
                msg.ranges,
                msg.angle_min,
                msg.angle_increment,
                self.prearm_config,
            )
            self.publish_clusters(
                self.prearm_raw_cluster_pub, prearm_raw_centers)

    @staticmethod
    def publish_clusters(publisher, centers):
        msg = ClusterData()
        msg.clusters = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in centers
        ]
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDbscanPreprocessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
