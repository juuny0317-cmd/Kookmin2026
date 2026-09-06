"""Latest-frame camera router for independent lane and scene pipelines."""

import threading
import time

from cam.perception_utils import stamp_to_ns
from custom_interfaces.msg import PipelineTiming
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


def reliability_from_string(value: str) -> ReliabilityPolicy:
    normalized = str(value).strip().lower()
    if normalized == 'reliable':
        return ReliabilityPolicy.RELIABLE
    if normalized == 'best_effort':
        return ReliabilityPolicy.BEST_EFFORT
    raise ValueError(
        'QoS reliability must be either "reliable" or "best_effort", '
        f'got {value!r}'
    )


def latest_image_qos(reliability: str) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=reliability_from_string(reliability),
        durability=DurabilityPolicy.VOLATILE,
    )


def arrival_time_slot(receive_monotonic: float, rate_hz: float) -> int:
    """Return a stable rate slot without delaying a frame to a timer tick."""
    return int(float(receive_monotonic) * float(rate_hz))


class FrameRouter(Node):
    """Subscribe once to the camera and publish rate-limited latest frames."""

    def __init__(self):
        super().__init__('frame_router')
        self.startup_monotonic_s = time.monotonic()
        self.first_camera_frame_logged = False

        self.declare_parameter('input_image_topic', '/image_raw')
        self.declare_parameter('camera_qos_reliability', 'reliable')
        self.declare_parameter('lane_image_topic', '/perception/lane/image')
        self.declare_parameter('lane_image_width', 320)
        self.declare_parameter('lane_image_height', 240)
        self.declare_parameter('lane_publish_rate_hz', 12.0)
        self.declare_parameter('lane_event_driven', False)
        self.declare_parameter('scene_image_topic', '/perception/scene/image')
        self.declare_parameter('scene_image_width', 640)
        self.declare_parameter('scene_image_height', 480)
        self.declare_parameter('scene_publish_rate_hz', 4.0)
        self.declare_parameter('lane_output_qos_reliability', '')
        self.declare_parameter('scene_output_qos_reliability', '')
        # Compatibility fallback for launches written before the two output
        # branches had independent QoS controls.
        self.declare_parameter('output_qos_reliability', 'reliable')
        self.declare_parameter('enable_lane_output', True)
        self.declare_parameter('enable_scene_output', True)
        self.declare_parameter('publish_performance_stats', False)
        self.declare_parameter('performance_log_period_s', 5.0)
        self.declare_parameter('publish_pipeline_timing', False)
        self.declare_parameter('pipeline_timing_topic', '/pipeline_timing')

        self.input_topic = str(self.get_parameter('input_image_topic').value)
        self.camera_reliability = str(
            self.get_parameter('camera_qos_reliability').value)
        compatibility_reliability = str(
            self.get_parameter('output_qos_reliability').value)
        self.lane_output_reliability = str(
            self.get_parameter('lane_output_qos_reliability').value
            or compatibility_reliability)
        self.scene_output_reliability = str(
            self.get_parameter('scene_output_qos_reliability').value
            or compatibility_reliability)
        self.enable_lane = bool(self.get_parameter('enable_lane_output').value)
        self.enable_scene = bool(self.get_parameter('enable_scene_output').value)
        self.publish_stats = bool(
            self.get_parameter('publish_performance_stats').value)
        self.lane_event_driven = bool(
            self.get_parameter('lane_event_driven').value)
        self.stats_period_s = max(
            1.0,
            float(self.get_parameter('performance_log_period_s').value),
        )

        self.lane_size = (
            int(self.get_parameter('lane_image_width').value),
            int(self.get_parameter('lane_image_height').value),
        )
        self.scene_size = (
            int(self.get_parameter('scene_image_width').value),
            int(self.get_parameter('scene_image_height').value),
        )
        if min(*self.lane_size, *self.scene_size) <= 0:
            raise ValueError('All output image dimensions must be positive')

        self.bridge = CvBridge()
        # Keep camera reception responsive when resize or DDS publish work
        # takes longer than its nominal period.  The executor in main() can
        # run these mutually exclusive groups independently.
        self.input_callback_group = MutuallyExclusiveCallbackGroup()
        self.lane_callback_group = MutuallyExclusiveCallbackGroup()
        self.scene_callback_group = MutuallyExclusiveCallbackGroup()
        self.stats_callback_group = MutuallyExclusiveCallbackGroup()
        self.lock = threading.Lock()
        self.latest_msg = None
        self.latest_receive_stamp_ns = 0
        self.latest_receive_monotonic = 0.0
        self.selected_lane_receive_stamp_ns = 0
        self.selected_lane_receive_monotonic = 0.0
        self.latest_generation = 0
        self.last_lane_generation = 0
        self.last_scene_generation = 0
        self.last_lane_stamp_ns = 0
        self.last_scene_stamp_ns = 0
        self.input_count = 0
        self.overwritten_count = 0
        self.lane_count = 0
        self.scene_count = 0
        self.lane_rate_limited_count = 0
        self.last_input_monotonic = None
        self.last_lane_monotonic = None
        self.last_scene_monotonic = None
        self.max_input_gap_s = 0.0
        self.max_lane_gap_s = 0.0
        self.max_scene_gap_s = 0.0
        self.last_stats_monotonic = time.monotonic()
        self.last_stats_counts = (0, 0, 0)
        self.lane_rate_hz = max(
            0.1,
            float(self.get_parameter('lane_publish_rate_hz').value),
        )
        self.lane_event = threading.Event()
        self.lane_worker_stop_event = threading.Event()
        self.lane_worker_thread = None
        self.last_lane_time_slot = None

        input_qos = latest_image_qos(self.camera_reliability)
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            input_qos,
            callback_group=self.input_callback_group,
        )
        self.pipeline_timing_pub = None
        if bool(self.get_parameter('publish_pipeline_timing').value):
            timing_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.pipeline_timing_pub = self.create_publisher(
                PipelineTiming,
                self.get_parameter('pipeline_timing_topic').value,
                timing_qos,
            )

        self.lane_publisher = None
        self.scene_publisher = None
        self.lane_timer = None
        self.scene_timer = None
        if self.enable_lane:
            lane_topic = str(self.get_parameter('lane_image_topic').value)
            lane_output_qos = latest_image_qos(
                self.lane_output_reliability)
            self.lane_publisher = self.create_publisher(
                Image, lane_topic, lane_output_qos)
            if self.lane_event_driven:
                self.lane_worker_thread = threading.Thread(
                    target=self.lane_event_worker,
                    name='frame-router-lane-worker',
                    daemon=True,
                )
                self.lane_worker_thread.start()
            else:
                self.lane_timer = self.create_timer(
                    1.0 / self.lane_rate_hz,
                    self.publish_lane_frame,
                    callback_group=self.lane_callback_group,
                )
        if self.enable_scene:
            scene_topic = str(self.get_parameter('scene_image_topic').value)
            scene_output_qos = latest_image_qos(
                self.scene_output_reliability)
            self.scene_publisher = self.create_publisher(
                Image, scene_topic, scene_output_qos)
            scene_rate = max(
                0.1,
                float(self.get_parameter('scene_publish_rate_hz').value),
            )
            self.scene_timer = self.create_timer(
                1.0 / scene_rate,
                self.publish_scene_frame,
                callback_group=self.scene_callback_group,
            )

        self.stats_timer = None
        if self.publish_stats:
            self.stats_timer = self.create_timer(
                self.stats_period_s,
                self.log_performance,
                callback_group=self.stats_callback_group,
            )

        self.get_logger().info(
            'Frame router started: '
            f'input={self.input_topic} qos={self.camera_reliability}/depth1, '
            f'lane={self.enable_lane} {self.lane_size} '
            f'event_driven={self.lane_event_driven} '
            f'qos={self.lane_output_reliability}/depth1, '
            f'scene={self.enable_scene} {self.scene_size} '
            f'qos={self.scene_output_reliability}/depth1'
        )

    def image_callback(self, msg: Image):
        now = time.monotonic()
        # Some algorithm tests construct the router with ``__new__`` to
        # exercise the latest-frame slot without creating a ROS node.  Treat
        # that intentionally partial object as if the startup milestone was
        # already logged.
        if not getattr(self, 'first_camera_frame_logged', True):
            self.first_camera_frame_logged = True
            self.get_logger().info(
                f'[STARTUP +{now - self.startup_monotonic_s:.3f}s] '
                'camera first frame reached frame_router')
        receive_stamp_ns = (
            self.get_clock().now().nanoseconds
            if getattr(self, 'pipeline_timing_pub', None) is not None
            else 0
        )
        with self.lock:
            if self.last_input_monotonic is not None:
                self.max_input_gap_s = max(
                    self.max_input_gap_s,
                    now - self.last_input_monotonic,
                )
            self.last_input_monotonic = now
            previous_generation = self.latest_generation
            if previous_generation:
                lane_pending = (
                    self.enable_lane
                    and self.last_lane_generation < previous_generation
                )
                scene_pending = (
                    self.enable_scene
                    and self.last_scene_generation < previous_generation
                )
                if lane_pending or scene_pending:
                    self.overwritten_count += 1
            self.latest_generation += 1
            self.latest_msg = msg
            self.latest_receive_stamp_ns = receive_stamp_ns
            self.latest_receive_monotonic = now
            self.input_count += 1
        if getattr(self, 'lane_event_driven', False) and self.enable_lane:
            self.lane_event.set()

    def take_latest(self, branch: str):
        with self.lock:
            if self.latest_msg is None:
                return None
            generation = self.latest_generation
            source_stamp_ns = stamp_to_ns(self.latest_msg.header.stamp)
            if branch == 'lane':
                if generation == self.last_lane_generation:
                    return None
                self.last_lane_generation = generation
                if (
                    source_stamp_ns > 0
                    and source_stamp_ns == self.last_lane_stamp_ns
                ):
                    return None
                self.last_lane_stamp_ns = source_stamp_ns
                self.selected_lane_receive_stamp_ns = (
                    self.latest_receive_stamp_ns)
                self.selected_lane_receive_monotonic = (
                    self.latest_receive_monotonic)
            else:
                if generation == self.last_scene_generation:
                    return None
                self.last_scene_generation = generation
                if (
                    source_stamp_ns > 0
                    and source_stamp_ns == self.last_scene_stamp_ns
                ):
                    return None
                self.last_scene_stamp_ns = source_stamp_ns
            return self.latest_msg

    def resize_message(self, msg: Image, size: tuple[int, int]) -> Image:
        if int(msg.width) == size[0] and int(msg.height) == size[1]:
            return msg
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        resized = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
        output = self.bridge.cv2_to_imgmsg(resized, encoding='bgr8')
        output.header = msg.header
        return output

    def lane_event_worker(self):
        """Immediately process at most one latest frame per 12 Hz slot."""
        while not self.lane_worker_stop_event.is_set():
            # Clear before checking the generation.  If a camera callback
            # arrives between clear() and wait(), Event remains set; if it
            # arrived before clear(), the generation check still observes it.
            # This closes the lost-wakeup window without adding a FIFO.
            self.lane_event.clear()
            with self.lock:
                work_pending = (
                    self.latest_msg is not None
                    and self.latest_generation != self.last_lane_generation
                )
            if not work_pending:
                self.lane_event.wait(timeout=0.1)
                continue
            if self.lane_worker_stop_event.is_set():
                break
            msg = self.take_latest('lane')
            if msg is None:
                continue
            slot = arrival_time_slot(
                self.selected_lane_receive_monotonic,
                self.lane_rate_hz,
            )
            if slot == self.last_lane_time_slot:
                self.lane_rate_limited_count += 1
                continue
            self.last_lane_time_slot = slot
            self.publish_lane_frame(msg)

    def publish_lane_frame(self, selected_msg=None):
        profile_start_monotonic = time.monotonic()
        profile_start_ns = (
            self.get_clock().now().nanoseconds
            if self.pipeline_timing_pub is not None else 0
        )
        msg = (
            selected_msg
            if selected_msg is not None
            else self.take_latest('lane')
        )
        if msg is None or self.lane_publisher is None:
            return
        try:
            output = self.resize_message(msg, self.lane_size)
        except Exception as exc:
            self.get_logger().error(f'Lane image resize failed: {exc}')
            return
        self.lane_publisher.publish(output)
        profile_end_monotonic = time.monotonic()
        profile_end_ns = (
            self.get_clock().now().nanoseconds
            if self.pipeline_timing_pub is not None else 0
        )
        self.lane_count += 1
        self.update_output_gap('lane')
        if self.pipeline_timing_pub is not None:
            timing = PipelineTiming()
            timing.header = msg.header
            timing.stage = 'frame_router_lane'
            timing.receive_stamp_ns = int(
                self.selected_lane_receive_stamp_ns)
            timing.start_stamp_ns = int(profile_start_ns)
            timing.end_stamp_ns = int(profile_end_ns)
            timing.queue_wait_ms = max(
                0.0,
                (profile_start_monotonic
                 - self.selected_lane_receive_monotonic) * 1000.0,
            )
            timing.compute_ms = (
                profile_end_monotonic - profile_start_monotonic) * 1000.0
            timing.received_count = int(self.input_count)
            timing.processed_count = int(self.lane_count)
            timing.replaced_count = int(self.overwritten_count)
            timing.input_reused = False
            self.pipeline_timing_pub.publish(timing)

    def publish_scene_frame(self):
        msg = self.take_latest('scene')
        if msg is None or self.scene_publisher is None:
            return
        try:
            output = self.resize_message(msg, self.scene_size)
        except Exception as exc:
            self.get_logger().error(f'Scene image resize failed: {exc}')
            return
        self.scene_publisher.publish(output)
        self.scene_count += 1
        self.update_output_gap('scene')

    def update_output_gap(self, branch: str):
        now = time.monotonic()
        if branch == 'lane':
            if self.last_lane_monotonic is not None:
                self.max_lane_gap_s = max(
                    self.max_lane_gap_s,
                    now - self.last_lane_monotonic,
                )
            self.last_lane_monotonic = now
        else:
            if self.last_scene_monotonic is not None:
                self.max_scene_gap_s = max(
                    self.max_scene_gap_s,
                    now - self.last_scene_monotonic,
                )
            self.last_scene_monotonic = now

    def source_age_ms(self) -> float:
        with self.lock:
            msg = self.latest_msg
        if msg is None:
            return float('nan')
        source_ns = stamp_to_ns(msg.header.stamp)
        if source_ns <= 0:
            return float('nan')
        age_ms = (self.get_clock().now().nanoseconds - source_ns) * 1e-6
        return age_ms if -1000.0 <= age_ms <= 86_400_000.0 else float('nan')

    def log_performance(self):
        now = time.monotonic()
        elapsed = max(now - self.last_stats_monotonic, 1e-6)
        counts = (self.input_count, self.lane_count, self.scene_count)
        rates = [
            (count - previous) / elapsed
            for count, previous in zip(counts, self.last_stats_counts)
        ]
        self.get_logger().info(
            'Frame router performance: '
            f'input={rates[0]:.1f}Hz lane={rates[1]:.1f}Hz '
            f'scene={rates[2]:.1f}Hz overwritten={self.overwritten_count} '
            f'rate_limited={self.lane_rate_limited_count} '
            f'max_gap(input/lane/scene)='
            f'{self.max_input_gap_s * 1000.0:.1f}/'
            f'{self.max_lane_gap_s * 1000.0:.1f}/'
            f'{self.max_scene_gap_s * 1000.0:.1f}ms '
            f'source_age={self.source_age_ms():.1f}ms'
        )
        self.last_stats_monotonic = now
        self.last_stats_counts = counts
        self.max_input_gap_s = 0.0
        self.max_lane_gap_s = 0.0
        self.max_scene_gap_s = 0.0

    def stop_lane_worker(self):
        if self.lane_worker_thread is None:
            return
        self.lane_worker_stop_event.set()
        self.lane_event.set()
        self.lane_worker_thread.join(timeout=2.0)
        if self.lane_worker_thread.is_alive():
            self.get_logger().warn(
                'Frame router lane worker did not stop within two seconds')
        self.lane_worker_thread = None

    def destroy_node(self):
        self.stop_lane_worker()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrameRouter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
