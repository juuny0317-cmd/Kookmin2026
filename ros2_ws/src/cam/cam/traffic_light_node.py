from collections import deque

from custom_interfaces.msg import Detections
import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String


class TrafficLightNode(Node):

    def __init__(self):
        super().__init__('traffic_light_node')

        self.declare_parameter('image_topic', '/perception/scene/image')
        self.declare_parameter(
            'detection_topic', '/scene_yolo/detections')
        self.declare_parameter('image_qos_reliability', 'reliable')
        self.declare_parameter('traffic_class_name', 'race_traffic_light')
        self.declare_parameter(
            'direct_state_class_names', 'red,green,left')
        self.declare_parameter('direct_state_min_confidence', 0.20)
        self.declare_parameter('direct_red_min_confidence', 0.20)
        self.declare_parameter('direct_green_min_confidence', 0.35)
        self.declare_parameter('direct_left_min_confidence', 0.50)
        self.declare_parameter('left_is_green', True)
        self.declare_parameter('direct_red_immediate', True)
        self.declare_parameter('state_topic', '/traffic_light_state')
        self.declare_parameter('sign_color_topic', '/sign_color')
        self.declare_parameter(
            'raw_state_topic', '/traffic_light_raw_state')
        self.declare_parameter('mission_mode_topic', '/mission_mode')
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.2)
        self.declare_parameter('min_color_ratio', 0.025)
        self.declare_parameter('min_color_pixels', 20)
        self.declare_parameter('dominance_ratio', 1.25)
        self.declare_parameter('stable_frames', 2)
        self.declare_parameter('red_release_hold_s', 0.5)
        self.declare_parameter('red_latch_until_green', True)
        self.declare_parameter('red_latch_confirm_frames', 2)
        self.declare_parameter('debug_view', False)

        self.image_topic = self.get_parameter('image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.traffic_class_name = self.get_parameter('traffic_class_name').value
        self.direct_state_class_names = {
            item.strip().lower()
            for item in str(self.get_parameter(
                'direct_state_class_names').value).split(',')
            if item.strip()
        }
        self.direct_state_min_confidence = float(
            self.get_parameter('direct_state_min_confidence').value)
        self.direct_state_min_confidences = {
            'red': float(self.get_parameter(
                'direct_red_min_confidence').value),
            'green': float(self.get_parameter(
                'direct_green_min_confidence').value),
            'left': float(self.get_parameter(
                'direct_left_min_confidence').value),
        }
        self.left_is_green = self.as_bool(
            self.get_parameter('left_is_green').value)
        self.direct_red_immediate = self.as_bool(
            self.get_parameter('direct_red_immediate').value)
        self.min_color_ratio = float(self.get_parameter('min_color_ratio').value)
        self.min_color_pixels = int(self.get_parameter('min_color_pixels').value)
        self.dominance_ratio = float(self.get_parameter('dominance_ratio').value)
        self.stable_frames = max(1, int(self.get_parameter('stable_frames').value))
        self.red_release_hold_s = max(
            0.0,
            float(self.get_parameter('red_release_hold_s').value),
        )
        self.red_latch_until_green = self.as_bool(
            self.get_parameter('red_latch_until_green').value)
        self.red_latch_confirm_frames = max(
            1,
            int(self.get_parameter('red_latch_confirm_frames').value),
        )
        self.debug_view = self.as_bool(self.get_parameter('debug_view').value)

        self.bridge = CvBridge()
        self.state_history = deque(maxlen=self.stable_frames)
        self.current_state = 'unknown'
        self.last_red_sample_ns = 0
        self.consecutive_red_samples = 0
        self.red_latch_active = False

        self.state_pub = self.create_publisher(String, self.get_parameter('state_topic').value, 10)
        self.sign_pub = self.create_publisher(
            String,
            self.get_parameter('sign_color_topic').value,
            10,
        )
        self.raw_state_pub = self.create_publisher(
            String,
            self.get_parameter('raw_state_topic').value,
            10,
        )
        mission_mode_qos = QoSProfile(depth=1)
        mission_mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mission_mode_sub = self.create_subscription(
            String,
            self.get_parameter('mission_mode_topic').value,
            self.mission_mode_callback,
            mission_mode_qos,
        )

        reliability_name = str(
            self.get_parameter('image_qos_reliability').value
        ).strip().lower()
        if reliability_name not in ('reliable', 'best_effort'):
            raise ValueError(
                'image_qos_reliability must be reliable or best_effort')
        image_reliability = (
            ReliabilityPolicy.RELIABLE
            if reliability_name == 'reliable'
            else ReliabilityPolicy.BEST_EFFORT
        )
        latest_image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=image_reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            self.image_topic,
            qos_profile=latest_image_qos,
        )
        self.detection_sub = message_filters.Subscriber(
            self,
            Detections,
            self.detection_topic,
            qos_profile=detection_qos,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.detection_sub],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop').value),
        )
        self.sync.registerCallback(self.synced_callback)

        if self.debug_view:
            cv2.namedWindow('traffic_light_node')

        self.get_logger().info(
            f'Traffic light node started. class={self.traffic_class_name}, '
            f'direct={sorted(self.direct_state_class_names)}, '
            f'direct_thresholds={self.direct_state_min_confidences}, '
            f'red_latch={self.red_latch_until_green}, '
            f'red_latch_confirm={self.red_latch_confirm_frames}, '
            f'image={self.image_topic}, detections={self.detection_topic}, '
            f'image_qos={reliability_name}/depth1'
        )

    def synced_callback(self, image_msg: Image, detections_msg: Detections):
        direct_detection = self.select_direct_state(detections_msg)
        if direct_detection is not None and not self.debug_view:
            raw_state = direct_detection[0]
            self.publish_state(
                self.update_direct_state(raw_state), raw_state=raw_state)
            return

        bbox = self.select_best_traffic_light(detections_msg)

        if (
            direct_detection is None
            and bbox is None
            and not self.debug_view
        ):
            self.publish_state(
                self.update_state('unknown'), raw_state='unknown')
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            self.publish_state(self.update_state('unknown'))
            return

        raw_state = 'unknown'
        if direct_detection is not None:
            raw_state, bbox = direct_detection
            sample_state = self.normalize_direct_state(raw_state)
            red_score = 1.0 if sample_state == 'red' else 0.0
            green_score = 1.0 if sample_state == 'green' else 0.0
            x1, y1, x2, y2 = self.clamp_bbox(bbox, frame)
            roi = frame[y1:y2, x1:x2]
            color = (0, 0, 255) if sample_state == 'red' else (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        elif bbox is None:
            sample_state = 'unknown'
            red_score = 0.0
            green_score = 0.0
            roi = None
        else:
            x1, y1, x2, y2 = self.clamp_bbox(bbox, frame)
            roi = frame[y1:y2, x1:x2]
            sample_state, red_score, green_score = self.detect_color_state(roi)
            if self.debug_view:
                color = (0, 0, 255) if sample_state == 'red' else (0, 255, 0)
                if sample_state == 'unknown':
                    color = (255, 255, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        state = (
            self.update_direct_state(raw_state)
            if direct_detection is not None
            else self.update_state(sample_state)
        )
        if direct_detection is None:
            raw_state = sample_state
        self.publish_state(state, raw_state=raw_state)

        if self.debug_view:
            cv2.putText(
                frame,
                f'state={state} red={red_score:.3f} green={green_score:.3f}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            cv2.imshow('traffic_light_node', frame)
            if roi is not None and roi.size > 0:
                cv2.imshow('traffic_light_roi', roi)
            cv2.waitKey(1)

    def mission_mode_callback(self, msg: String):
        if msg.data.strip().upper() != 'STOP':
            return
        # A red light makes the mission manager publish STOP.  Treating that
        # safety output as an explicit drive reset clears the very red state
        # that caused it, creating a STOP -> unknown -> drive feedback loop.
        # Keep a current/latched red until the normal green-release logic
        # clears it.  STOP can still reset non-red state during startup or an
        # ordinary manual stop.
        if self.current_state == 'red' or self.red_latch_active:
            return
        self.state_history.clear()
        self.current_state = 'unknown'
        self.last_red_sample_ns = 0
        self.consecutive_red_samples = 0
        self.red_latch_active = False
        self.publish_state('unknown', raw_state='reset')
        self.get_logger().info('Traffic-light latch reset by drive STOP.')

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def select_best_traffic_light(self, detections_msg: Detections):
        candidates = [
            det for det in detections_msg.detections
            if det.class_name == self.traffic_class_name
        ]
        if not candidates:
            return None

        best = max(
            candidates,
            key=lambda det: (
                float(det.confidence)
                * max(1, int(det.xmax - det.xmin))
                * max(1, int(det.ymax - det.ymin))
            ),
        )
        return int(best.xmin), int(best.ymin), int(best.xmax), int(best.ymax)

    def select_direct_state(self, detections_msg: Detections):
        candidates = [
            det for det in detections_msg.detections
            if det.class_name.strip().lower()
            in self.direct_state_class_names
            and float(det.confidence) >= self.direct_state_threshold(
                det.class_name)
        ]
        if not candidates:
            return None

        # A simultaneous red detection wins regardless of another light's
        # confidence. Failing safe is preferable to driving through red.
        red_candidates = [
            det for det in candidates
            if det.class_name.strip().lower() == 'red'
        ]
        selected = max(
            red_candidates or candidates,
            key=lambda det: float(det.confidence),
        )
        return (
            selected.class_name.strip().lower(),
            (
                int(selected.xmin),
                int(selected.ymin),
                int(selected.xmax),
                int(selected.ymax),
            ),
        )

    def direct_state_threshold(self, class_name):
        return self.direct_state_min_confidences.get(
            str(class_name).strip().lower(),
            self.direct_state_min_confidence,
        )

    def normalize_direct_state(self, raw_state):
        raw_state = str(raw_state).strip().lower()
        if raw_state == 'left' and self.left_is_green:
            return 'green'
        if raw_state in ('red', 'green'):
            return raw_state
        return 'unknown'

    def update_direct_state(self, raw_state):
        raw_state = str(raw_state).strip().lower()
        if raw_state == 'red' and self.direct_red_immediate:
            self.record_red_sample(self.get_clock().now().nanoseconds)
            self.current_state = 'red'
            self.state_history.clear()
            return 'red'
        return self.update_state(self.normalize_direct_state(raw_state))

    def record_red_sample(self, now_ns):
        self.last_red_sample_ns = int(now_ns)
        self.consecutive_red_samples += 1
        if (
            self.red_latch_until_green
            and self.consecutive_red_samples
            >= self.red_latch_confirm_frames
        ):
            self.red_latch_active = True

    @staticmethod
    def clamp_bbox(bbox, frame):
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = bbox
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))
        return x1, y1, max(x2, x1 + 1), max(y2, y1 + 1)

    def detect_color_state(self, roi):
        if roi is None or roi.size == 0:
            return 'unknown', 0.0, 0.0

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_mask_low = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([10, 255, 255]))
        red_mask_high = cv2.inRange(hsv, np.array([170, 80, 80]), np.array([179, 255, 255]))
        red_mask = cv2.bitwise_or(red_mask_low, red_mask_high)
        green_mask = cv2.inRange(hsv, np.array([45, 70, 80]), np.array([95, 255, 255]))

        area = float(roi.shape[0] * roi.shape[1])
        red_pixels = float(cv2.countNonZero(red_mask))
        green_pixels = float(cv2.countNonZero(green_mask))
        red_score = red_pixels / area
        green_score = green_pixels / area

        min_ratio = self.min_color_ratio
        min_pixels = float(self.min_color_pixels)

        if (
            red_score >= min_ratio
            and red_pixels >= min_pixels
            and red_score > green_score * self.dominance_ratio
        ):
            return 'red', red_score, green_score
        if (
            green_score >= min_ratio
            and green_pixels >= min_pixels
            and green_score > red_score * self.dominance_ratio
        ):
            return 'green', red_score, green_score
        return 'unknown', red_score, green_score

    def stabilize_state(self, sample_state):
        self.state_history.append(sample_state)
        if len(self.state_history) < self.stable_frames:
            return self.current_state

        if all(state == self.state_history[0] for state in self.state_history):
            self.current_state = self.state_history[0]

        return self.current_state

    def update_state(self, sample_state):
        now_ns = self.get_clock().now().nanoseconds
        was_red = self.current_state == 'red'
        if sample_state == 'red':
            self.record_red_sample(now_ns)
        elif not self.red_latch_active:
            self.consecutive_red_samples = 0

        if (
            self.red_latch_active
            and sample_state != 'green'
        ):
            # Losing the light after passing it is not evidence of permission
            # to move.  Only a stabilized green/left indication releases a
            # confirmed red; explicit stop service remains the manual fallback.
            self.state_history.clear()
            self.current_state = 'red'
            return 'red'

        state = self.stabilize_state(sample_state)
        red_age_s = float('inf')
        if self.last_red_sample_ns > 0:
            red_age_s = (now_ns - self.last_red_sample_ns) * 1e-9

        if (
            (was_red or state == 'red')
            and red_age_s <= self.red_release_hold_s
        ):
            self.current_state = 'red'
            return 'red'
        if state == 'green':
            self.red_latch_active = False
            self.consecutive_red_samples = 0
        return state

    def publish_state(self, state, raw_state=None):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.sign_pub.publish(msg)
        raw_msg = String()
        raw_msg.data = str(raw_state if raw_state is not None else state)
        self.raw_state_pub.publish(raw_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node.debug_view:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
