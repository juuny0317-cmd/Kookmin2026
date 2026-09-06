import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from custom_interfaces.msg import ClusterData, Detections
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class ConeEntryFusionNode(Node):
    def __init__(self):
        super().__init__('cone_entry_fusion_node')

        self.declare_parameter('raw_clusters_topic', '/clusters_raw')
        self.declare_parameter(
            'prearm_raw_clusters_topic', '/prearm_clusters_raw')
        self.declare_parameter(
            'detections_topic', '/scene_yolo/detections')
        self.declare_parameter(
            'output_clusters_topic',
            '/entry_cone_clusters',
        )
        self.declare_parameter(
            'prearm_output_clusters_topic',
            '/cone_prearm_clusters',
        )
        self.declare_parameter(
            'status_topic',
            '/cone_entry_fusion_status',
        )
        self.declare_parameter('cone_class_name', 'cone')
        self.declare_parameter('cone_conf_threshold', 0.15)
        self.declare_parameter('min_bbox_area_px', 180.0)
        self.declare_parameter('min_bottom_y_px', 220.0)
        self.declare_parameter('entry_max_range_m', 1.50)
        self.declare_parameter('prearm_min_bbox_area_px', 100.0)
        self.declare_parameter('prearm_min_bottom_y_px', 170.0)
        self.declare_parameter('prearm_max_range_m', 3.00)
        self.declare_parameter('max_detection_age_s', 0.75)
        self.declare_parameter('bbox_padding_x_px', 20.0)
        self.declare_parameter('bbox_padding_y_px', 30.0)
        self.declare_parameter('min_camera_depth_m', 0.03)
        self.declare_parameter('cluster_rotation_deg', 0.0)
        self.declare_parameter('projection_model', 'fisheye')
        self.declare_parameter('calibration_package', 'cam')
        self.declare_parameter('intrinsic_filename', 'fisheye_calib.yaml')
        self.declare_parameter('extrinsic_filename', 'extrinsic.yaml')

        self.cone_class_name = str(
            self.get_parameter('cone_class_name').value)
        self.cone_conf_threshold = float(
            self.get_parameter('cone_conf_threshold').value)
        self.min_bbox_area_px = float(
            self.get_parameter('min_bbox_area_px').value)
        self.min_bottom_y_px = float(
            self.get_parameter('min_bottom_y_px').value)
        self.entry_max_range_m = max(
            0.0,
            float(self.get_parameter('entry_max_range_m').value),
        )
        self.prearm_min_bbox_area_px = float(
            self.get_parameter('prearm_min_bbox_area_px').value)
        self.prearm_min_bottom_y_px = float(
            self.get_parameter('prearm_min_bottom_y_px').value)
        self.prearm_max_range_m = max(
            self.entry_max_range_m,
            float(self.get_parameter('prearm_max_range_m').value),
        )
        self.max_detection_age_s = float(
            self.get_parameter('max_detection_age_s').value)
        self.bbox_padding_x_px = float(
            self.get_parameter('bbox_padding_x_px').value)
        self.bbox_padding_y_px = float(
            self.get_parameter('bbox_padding_y_px').value)
        self.min_camera_depth_m = float(
            self.get_parameter('min_camera_depth_m').value)
        self.cluster_rotation_rad = math.radians(float(
            self.get_parameter('cluster_rotation_deg').value
        ))
        self.projection_model = str(
            self.get_parameter('projection_model').value
        ).strip().lower()
        if self.projection_model not in ('fisheye', 'pinhole'):
            raise ValueError(
                'projection_model must be either "fisheye" or "pinhole"'
            )

        calibration = self.load_calibration()
        (
            self.camera_matrix,
            self.distortion,
            self.rotation,
            self.translation,
        ) = calibration
        self.latest_boxes = []
        self.latest_prearm_boxes = []
        self.latest_detection_source_ns = 0
        self.latest_entry_raw_count = 0
        self.latest_entry_match_count = 0
        self.latest_prearm_raw_count = 0
        self.latest_prearm_match_count = 0

        self.detection_sub = self.create_subscription(
            Detections,
            self.get_parameter('detections_topic').value,
            self.detection_callback,
            10,
        )
        self.cluster_sub = self.create_subscription(
            ClusterData,
            self.get_parameter('raw_clusters_topic').value,
            self.cluster_callback,
            10,
        )
        self.prearm_cluster_sub = self.create_subscription(
            ClusterData,
            self.get_parameter('prearm_raw_clusters_topic').value,
            self.prearm_cluster_callback,
            10,
        )
        self.cluster_pub = self.create_publisher(
            ClusterData,
            self.get_parameter('output_clusters_topic').value,
            10,
        )
        self.prearm_cluster_pub = self.create_publisher(
            ClusterData,
            self.get_parameter('prearm_output_clusters_topic').value,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            10,
        )

        self.get_logger().info(
            'Entry-only YOLO-LiDAR cone gate started: '
            f'confidence>={self.cone_conf_threshold:.2f}, '
            f'area>={self.min_bbox_area_px:.0f}px, '
            f'bottom_y>={self.min_bottom_y_px:.0f}px, '
            f'entry_range<={self.entry_max_range_m:.2f}m, '
            f'prearm_area>={self.prearm_min_bbox_area_px:.0f}px, '
            f'prearm_bottom_y>={self.prearm_min_bottom_y_px:.0f}px, '
            f'prearm_range<={self.prearm_max_range_m:.2f}m, '
            f'detection_age<={self.max_detection_age_s:.2f}s, '
            f'projection={self.projection_model}, '
            'cluster_rotation='
            f'{math.degrees(self.cluster_rotation_rad):.1f}deg'
        )

    def load_calibration(self):
        package_name = str(self.get_parameter('calibration_package').value)
        package_dir = get_package_share_directory(package_name)
        config_dir = os.path.join(package_dir, 'config')
        intrinsic_path = os.path.join(
            config_dir,
            str(self.get_parameter('intrinsic_filename').value),
        )
        extrinsic_path = os.path.join(
            config_dir,
            str(self.get_parameter('extrinsic_filename').value),
        )

        with open(intrinsic_path, 'r', encoding='utf-8') as stream:
            intrinsic = yaml.safe_load(stream)
        with open(extrinsic_path, 'r', encoding='utf-8') as stream:
            extrinsic = yaml.safe_load(stream)

        camera_matrix = np.asarray(intrinsic['K'], dtype=np.float64)
        distortion = np.asarray(
            intrinsic.get('D', [0.0, 0.0, 0.0, 0.0]),
            dtype=np.float64,
        ).reshape(-1)
        if self.projection_model == 'fisheye' and distortion.size != 4:
            raise ValueError(
                'Fisheye projection requires exactly four D coefficients'
            )
        rotation = np.asarray(extrinsic['R'], dtype=np.float64)
        translation = np.asarray(extrinsic['t'], dtype=np.float64).reshape(3)
        return camera_matrix, distortion, rotation, translation

    def detection_callback(self, msg: Detections):
        entry_boxes = []
        prearm_boxes = []
        for detection in msg.detections:
            if detection.class_name != self.cone_class_name:
                continue
            if float(detection.confidence) < self.cone_conf_threshold:
                continue
            width = max(0.0, float(detection.xmax - detection.xmin))
            height = max(0.0, float(detection.ymax - detection.ymin))
            box = (
                float(detection.xmin),
                float(detection.ymin),
                float(detection.xmax),
                float(detection.ymax),
                float(detection.confidence),
            )
            area = width * height
            bottom_y = float(detection.ymax)
            if (
                area >= self.prearm_min_bbox_area_px
                and bottom_y >= self.prearm_min_bottom_y_px
            ):
                prearm_boxes.append(box)
            if (
                area >= self.min_bbox_area_px
                and bottom_y >= self.min_bottom_y_px
            ):
                entry_boxes.append(box)
        self.latest_boxes = entry_boxes
        self.latest_prearm_boxes = prearm_boxes
        self.latest_detection_source_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

    def cluster_callback(self, msg: ClusterData):
        raw_points = [
            (float(point.x), float(point.y))
            for point in msg.clusters
        ]
        entry_matched_points = self.match_points(
            raw_points,
            self.latest_boxes,
            self.entry_max_range_m,
        )
        self.publish_clusters(self.cluster_pub, entry_matched_points)
        self.latest_entry_raw_count = len(raw_points)
        self.latest_entry_match_count = len(entry_matched_points)
        self.publish_status()

    def prearm_cluster_callback(self, msg: ClusterData):
        raw_points = [
            (float(point.x), float(point.y))
            for point in msg.clusters
        ]
        prearm_matched_points = self.match_points(
            raw_points,
            self.latest_prearm_boxes,
            self.prearm_max_range_m,
        )
        self.publish_clusters(
            self.prearm_cluster_pub, prearm_matched_points)
        self.latest_prearm_raw_count = len(raw_points)
        self.latest_prearm_match_count = len(prearm_matched_points)
        self.publish_status()

    def publish_status(self):
        status = String()
        status.data = (
            f'entry_boxes={len(self.latest_boxes)} '
            f'prearm_boxes={len(self.latest_prearm_boxes)} '
            f'entry_raw={self.latest_entry_raw_count} '
            f'prearm_raw={self.latest_prearm_raw_count} '
            f'entry_matches={self.latest_entry_match_count} '
            f'prearm_matches={self.latest_prearm_match_count}'
        )
        self.status_pub.publish(status)

    def match_points(self, raw_points, boxes, max_range_m):
        if not raw_points or not boxes or not self.detections_are_fresh():
            return []
        projected = self.project_points(raw_points)
        return [
            point for point, pixel in zip(raw_points, projected)
            if (
                pixel is not None
                and math.hypot(point[0], point[1]) <= max_range_m
                and self.pixel_matches_cone(pixel, boxes)
            )
        ]

    @staticmethod
    def publish_clusters(publisher, points):
        output = ClusterData()
        output.clusters = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in points
        ]
        publisher.publish(output)

    def detections_are_fresh(self):
        if self.latest_detection_source_ns <= 0:
            return False
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - self.latest_detection_source_ns) * 1e-9
        return 0.0 <= age <= self.max_detection_age_s

    def project_points(self, points):
        cos_t = math.cos(self.cluster_rotation_rad)
        sin_t = math.sin(self.cluster_rotation_rad)
        raw_lidar_points = [
            (
                cos_t * x + sin_t * y,
                -sin_t * x + cos_t * y,
            )
            for x, y in points
        ]
        lidar_points = np.asarray(
            [[x, y, 0.0] for x, y in raw_lidar_points],
            dtype=np.float64,
        )
        camera_points = (self.rotation @ lidar_points.T).T + self.translation
        projected = [None] * len(points)
        valid_indices = np.flatnonzero(
            camera_points[:, 2] > self.min_camera_depth_m
        )
        if not len(valid_indices):
            return projected

        valid_camera_points = camera_points[valid_indices]
        if self.projection_model == 'fisheye':
            image_points, _ = cv2.fisheye.projectPoints(
                valid_camera_points.reshape(-1, 1, 3),
                np.zeros((3, 1), dtype=np.float64),
                np.zeros((3, 1), dtype=np.float64),
                self.camera_matrix,
                self.distortion.reshape(4, 1),
            )
            image_points = image_points.reshape(-1, 2)
        else:
            homogeneous = (
                self.camera_matrix @ valid_camera_points.T
            ).T
            image_points = homogeneous[:, :2] / homogeneous[:, 2:3]

        for index, pixel in zip(valid_indices, image_points):
            if np.all(np.isfinite(pixel)):
                projected[int(index)] = (
                    float(pixel[0]),
                    float(pixel[1]),
                )
        return projected

    def pixel_matches_cone(self, pixel, boxes=None):
        pixel_x, pixel_y = pixel
        candidates = self.latest_boxes if boxes is None else boxes
        for xmin, ymin, xmax, ymax, _ in candidates:
            horizontal_match = (
                xmin - self.bbox_padding_x_px
                <= pixel_x
                <= xmax + self.bbox_padding_x_px
            )
            vertical_match = (
                ymin - self.bbox_padding_y_px
                <= pixel_y
                <= ymax + self.bbox_padding_y_px
            )
            if horizontal_match and vertical_match:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ConeEntryFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
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
