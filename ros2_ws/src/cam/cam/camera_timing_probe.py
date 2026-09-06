"""Join passive usb_cam timing events with `/image_raw` callback receipt."""

from collections import OrderedDict
import time

from custom_interfaces.msg import CameraTiming
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def reliable_qos(depth=100):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class CameraTimingProbe(Node):
    """Publish a complete T0..T7 event without touching the image pipeline."""

    def __init__(self):
        super().__init__('camera_timing_probe')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('driver_timing_topic', '/camera_timing')
        self.declare_parameter(
            'complete_timing_topic', '/camera_timing_complete')
        self.declare_parameter('cache_size', 512)
        self.declare_parameter('join_timeout_s', 0.50)
        self.cache_size = max(8, int(self.get_parameter('cache_size').value))
        self.join_timeout_s = max(
            0.05, float(self.get_parameter('join_timeout_s').value))
        self.raw_receipts = OrderedDict()
        self.driver_events = OrderedDict()
        self.seen_image_stamps = OrderedDict()
        self.seen_driver_stamps = OrderedDict()
        self.publisher = self.create_publisher(
            CameraTiming,
            str(self.get_parameter('complete_timing_topic').value),
            reliable_qos(),
        )
        self.image_subscription = self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.image_callback,
            reliable_qos(depth=1),
        )
        self.timing_subscription = self.create_subscription(
            CameraTiming,
            str(self.get_parameter('driver_timing_topic').value),
            self.timing_callback,
            reliable_qos(),
        )
        self.expiry_timer = self.create_timer(0.10, self.flush_expired)

    def trim_seen(self, cache):
        limit = self.cache_size * 8
        while len(cache) > limit:
            cache.popitem(last=False)

    def is_duplicate(self, cache, source_ns):
        duplicate = source_ns in cache
        cache[source_ns] = None
        cache.move_to_end(source_ns)
        self.trim_seen(cache)
        return duplicate

    def make_image_only_event(self, header, receive_ns, status):
        timing = CameraTiming()
        timing.header = header
        timing.valid = False
        timing.subscriber_receive_ns = int(receive_ns)
        timing.image_header_stamp_ns = stamp_ns(header.stamp)
        timing.join_valid = False
        timing.join_status = str(status)
        return timing

    def publish_invalid_driver(self, timing, status):
        timing.image_header_stamp_ns = 0
        timing.subscriber_receive_ns = 0
        timing.join_valid = False
        timing.join_status = str(status)
        self.publisher.publish(timing)

    def trim_pending(self, cache, kind):
        while len(cache) > self.cache_size:
            _, item = cache.popitem(last=False)
            if kind == 'image':
                receive_ns, _, header = item
                self.publisher.publish(self.make_image_only_event(
                    header, receive_ns, 'image_cache_evicted'))
            else:
                timing, _ = item
                self.publish_invalid_driver(
                    timing, 'driver_cache_evicted')

    def publish_complete(self, timing, receive_ns, image_stamp_ns):
        timing.subscriber_receive_ns = int(receive_ns)
        timing.image_header_stamp_ns = int(image_stamp_ns)
        source_ns = stamp_ns(timing.header.stamp)
        timing.join_valid = bool(
            source_ns > 0
            and source_ns == image_stamp_ns
            and int(timing.v4l2_stamp_ns) == source_ns
        )
        timing.join_status = (
            'matched' if timing.join_valid else 'header_stamp_mismatch')
        self.publisher.publish(timing)

    def image_callback(self, message):
        # Driver T0..T6 use CLOCK_REALTIME. ROS may use simulated time.
        receive_ns = time.time_ns()
        source_ns = stamp_ns(message.header.stamp)
        if source_ns <= 0:
            self.publisher.publish(self.make_image_only_event(
                message.header, receive_ns, 'invalid_image_header_stamp'))
            return
        if self.is_duplicate(self.seen_image_stamps, source_ns):
            self.publisher.publish(self.make_image_only_event(
                message.header, receive_ns, 'duplicate_image_header_stamp'))
            return
        driver_item = self.driver_events.pop(source_ns, None)
        if driver_item is not None:
            timing, _ = driver_item
            self.publish_complete(timing, receive_ns, source_ns)
            return
        self.raw_receipts[source_ns] = (
            receive_ns, time.monotonic(), message.header)
        self.trim_pending(self.raw_receipts, 'image')

    def timing_callback(self, message):
        source_ns = stamp_ns(message.header.stamp)
        if source_ns <= 0:
            self.publish_invalid_driver(
                message, 'invalid_driver_header_stamp')
            return
        if self.is_duplicate(self.seen_driver_stamps, source_ns):
            self.publish_invalid_driver(
                message, 'duplicate_driver_header_stamp')
            return
        raw_item = self.raw_receipts.pop(source_ns, None)
        if raw_item is not None:
            receive_ns, _, _ = raw_item
            self.publish_complete(message, receive_ns, source_ns)
            return
        self.driver_events[source_ns] = (message, time.monotonic())
        self.trim_pending(self.driver_events, 'driver')

    def flush_expired(self):
        now = time.monotonic()
        while self.raw_receipts:
            source_ns, item = next(iter(self.raw_receipts.items()))
            receive_ns, inserted, header = item
            if now - inserted < self.join_timeout_s:
                break
            self.raw_receipts.pop(source_ns)
            self.publisher.publish(self.make_image_only_event(
                header, receive_ns, 'image_without_driver_timing'))
        while self.driver_events:
            source_ns, item = next(iter(self.driver_events.items()))
            timing, inserted = item
            if now - inserted < self.join_timeout_s:
                break
            self.driver_events.pop(source_ns)
            self.publish_invalid_driver(
                timing, 'driver_timing_without_image')


def main(args=None):
    rclpy.init(args=args)
    node = CameraTimingProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
