import threading

from custom_interfaces.msg import Detection, Detections
from sensor_msgs.msg import Image

from cam.centerlane_tracer import CenterlaneTracer


def set_stamp(header, stamp_ns):
    header.stamp.sec = stamp_ns // 1_000_000_000
    header.stamp.nanosec = stamp_ns % 1_000_000_000
    header.frame_id = 'camera'


def make_tracer(scene_stamp_ns):
    tracer = CenterlaneTracer.__new__(CenterlaneTracer)
    tracer.enable_scene_detection_cache = True
    tracer.scene_detection_max_age_s = 0.6
    tracer.scene_detection_source_width = 640
    tracer.scene_detection_source_height = 480
    tracer._scene_lock = threading.Lock()
    tracer._scene_frame_mismatch_warned = False
    scene = Detections()
    set_stamp(scene.header, scene_stamp_ns)
    detection = Detection()
    detection.class_name = 'obstacle_vehicle'
    detection.xmin = 100
    detection.ymin = 120
    detection.xmax = 500
    detection.ymax = 480
    scene.detections = [detection]
    tracer._latest_scene_detections = scene
    return tracer


def make_lane_image(stamp_ns):
    image = Image()
    set_stamp(image.header, stamp_ns)
    return image


def test_scene_cache_scales_only_fresh_nonfuture_stamped_boxes():
    reference_ns = 10_000_000_000

    tracer = make_tracer(reference_ns - 200_000_000)
    detections, age = tracer._fresh_scaled_scene_vehicles(
        make_lane_image(reference_ns), 320, 240)
    assert age == 0.2
    assert len(detections) == 1
    assert (
        detections[0].xmin,
        detections[0].ymin,
        detections[0].xmax,
        detections[0].ymax,
    ) == (50, 60, 250, 240)

    stale = make_tracer(reference_ns - 700_000_000)
    assert stale._fresh_scaled_scene_vehicles(
        make_lane_image(reference_ns), 320, 240)[0] == []

    future = make_tracer(reference_ns + 1)
    assert future._fresh_scaled_scene_vehicles(
        make_lane_image(reference_ns), 320, 240)[0] == []

    zero = make_tracer(0)
    assert zero._fresh_scaled_scene_vehicles(
        make_lane_image(reference_ns), 320, 240)[0] == []
