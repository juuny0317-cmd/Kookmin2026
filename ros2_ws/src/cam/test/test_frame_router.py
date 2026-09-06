import threading

from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image

from cam.frame_router import arrival_time_slot, FrameRouter


def make_image(stamp_ns, width=640, height=480):
    message = CvBridge().cv2_to_imgmsg(
        np.zeros((height, width, 3), dtype=np.uint8),
        encoding='bgr8',
    )
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = 'camera'
    return message


def make_uninitialized_router():
    router = FrameRouter.__new__(FrameRouter)
    router.lock = threading.Lock()
    router.latest_msg = None
    router.latest_generation = 0
    router.last_lane_generation = 0
    router.last_scene_generation = 0
    router.last_lane_stamp_ns = 0
    router.last_scene_stamp_ns = 0
    router.input_count = 0
    router.overwritten_count = 0
    router.enable_lane = True
    router.enable_scene = True
    router.last_input_monotonic = None
    router.max_input_gap_s = 0.0
    router.bridge = CvBridge()
    router.lane_event_driven = False
    return router


def test_latest_frame_overwrites_unconsumed_input():
    router = make_uninitialized_router()
    first = make_image(1_000_000_001)
    latest = make_image(1_000_000_002)

    router.image_callback(first)
    router.image_callback(latest)

    assert router.overwritten_count == 1
    assert router.take_latest('lane') is latest
    assert router.take_latest('lane') is None


def test_duplicate_source_stamp_is_not_republished():
    router = make_uninitialized_router()
    router.image_callback(make_image(2_000_000_001))
    assert router.take_latest('lane') is not None

    router.image_callback(make_image(2_000_000_001))
    assert router.take_latest('lane') is None


def test_resize_preserves_source_header():
    router = make_uninitialized_router()
    source = make_image(3_000_000_123)

    output = router.resize_message(source, (320, 240))

    assert output.width == 320
    assert output.height == 240
    assert output.header == source.header


def test_arrival_slots_select_about_12hz_from_30hz_without_wait():
    arrivals = [index / 30.0 for index in range(30)]
    slots = [arrival_time_slot(value, 12.0) for value in arrivals]
    selected = [
        arrival
        for index, arrival in enumerate(arrivals)
        if index == 0 or slots[index] != slots[index - 1]
    ]

    assert len(selected) == 12
    assert all(arrival in arrivals for arrival in selected)


def test_generation_retains_work_even_if_event_flag_was_cleared():
    """The worker generation check is the lost-wakeup backstop."""
    router = make_uninitialized_router()
    router.lane_event = threading.Event()
    router.image_callback(make_image(4_000_000_001))

    router.lane_event.clear()
    with router.lock:
        work_pending = (
            router.latest_msg is not None
            and router.latest_generation != router.last_lane_generation
        )

    assert work_pending
