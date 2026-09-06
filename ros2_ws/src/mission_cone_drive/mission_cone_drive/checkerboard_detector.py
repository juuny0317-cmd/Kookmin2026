import threading

import cv2
import message_filters
import rclpy
from cv_bridge import CvBridge
from custom_interfaces.msg import Detections
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


class CheckerboardDetector(Node):
    def __init__(self):
        super().__init__('checkerboard_detector')

        self.declare_parameter('activation_topic', '/sign_color')
        self.declare_parameter('detection_topic', '/yolo_detections')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('class_name', 'checkerboard')
        self.declare_parameter('bbox_area_threshold', 4000)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.2)
        self.declare_parameter('enable_visualization', False)
        self.declare_parameter('start_lane_service', '/start_lane_detection')
        self.declare_parameter('start_stanley_service', '/start_stanley_controller')

        self.class_name = self.get_parameter('class_name').value
        self.bbox_area_threshold = int(self.get_parameter('bbox_area_threshold').value)
        self.enable_visualization = bool(self.get_parameter('enable_visualization').value)
        self.is_started = False
        self.is_done = False
        self.latest_detections = []
        self.lock = threading.Lock()
        self.bridge = CvBridge()

        self.mode_pub = self.create_publisher(String, '/driving_mode', 10)
        self.sign_sub = self.create_subscription(
            String,
            self.get_parameter('activation_topic').value,
            self.sign_callback,
            10,
        )

        self.detection_sub = message_filters.Subscriber(
            self,
            Detections,
            self.get_parameter('detection_topic').value,
        )
        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter('image_topic').value,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.detection_sub, self.image_sub],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop').value),
        )
        self.sync.registerCallback(self.synced_callback)

        self.lane_client = self.create_client(
            Trigger,
            self.get_parameter('start_lane_service').value,
        )
        self.stanley_client = self.create_client(
            Trigger,
            self.get_parameter('start_stanley_service').value,
        )

        self.window_name = 'checkerboard_detector'
        if self.enable_visualization:
            cv2.namedWindow(self.window_name)

        self.get_logger().info(
            f'Checkerboard detector started. class={self.class_name}, '
            f'area_threshold={self.bbox_area_threshold}'
        )

    def sign_callback(self, msg: String):
        state = msg.data.strip().lower()
        if state == 'green':
            self.is_started = True
        elif state == 'red':
            self.is_started = False

    def synced_callback(self, detections_msg: Detections, image_msg: Image):
        if not self.is_started or self.is_done:
            return

        detections = []
        should_switch = False

        for detection in detections_msg.detections:
            if detection.class_name != self.class_name:
                continue

            xmin = int(detection.xmin)
            ymin = int(detection.ymin)
            xmax = int(detection.xmax)
            ymax = int(detection.ymax)
            area = max(0, xmax - xmin) * max(0, ymax - ymin)
            detections.append((xmin, ymin, xmax, ymax, float(detection.confidence), area))

            if area >= self.bbox_area_threshold:
                should_switch = True

        with self.lock:
            self.latest_detections = detections

        if self.enable_visualization:
            self.visualize(image_msg)

        if should_switch:
            self.switch_to_lane_driving()

    def switch_to_lane_driving(self):
        if self.is_done:
            return
        self.is_done = True

        mode_msg = String()
        mode_msg.data = 'STANLEY'
        self.mode_pub.publish(mode_msg)

        self.get_logger().warn('Checkerboard detected. Requesting lane driving restart.')
        self.call_service_if_available(self.lane_client, 'lane detection')
        self.call_service_if_available(self.stanley_client, 'stanley controller')

    def call_service_if_available(self, client, label):
        if not client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(f'{label} service is not available yet.')
            return

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda fut: self.get_logger().info(f'{label} service response: {fut.result().message}')
            if fut.result() is not None
            else self.get_logger().warn(f'{label} service call failed.')
        )

    def visualize(self, image_msg: Image):
        frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        with self.lock:
            detections = list(self.latest_detections)

        for xmin, ymin, xmax, ymax, confidence, area in detections:
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 255), 2)
            cv2.putText(
                frame,
                f'{self.class_name} {confidence:.2f} area={area}',
                (xmin, max(20, ymin - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CheckerboardDetector()
    try:
        rclpy.spin(node)
    finally:
        if node.enable_visualization:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
