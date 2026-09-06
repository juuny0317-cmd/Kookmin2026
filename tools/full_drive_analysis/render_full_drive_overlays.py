#!/usr/bin/env python3
"""Render exact-source lane/traffic overlays and Hough diagnostics.

The lane frames and recorded detections are joined by their original camera
header.  The current production CenterlaneTracer helpers are then applied to
that exact pair; this does not publish ROS messages or touch the source bags.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import rclpy
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from cam.centerlane_tracer import CenterlaneTracer


BAG_ROOT = Path("/home/xytron/rosbags")
BAGS = (
    "FULL_DRIVE_20260810_112624",
    "FULL_DRIVE_20260810_112801",
)
OUT = Path(
    "/home/xytron/Downloads/analysis/full_drive_20260810_four_problem"
)
SUSPECT_DIR = OUT / "lane_perception_suspect_frames"


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def header_ns(message):
    if hasattr(message, "header"):
        return stamp_ns(message.header.stamp)
    if hasattr(message, "control"):
        return stamp_ns(message.control.header.stamp)
    if hasattr(message, "debug"):
        return stamp_ns(message.debug.header.stamp)
    return 0


def reader(path):
    result = rosbag2_py.SequentialReader()
    result.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_names = {
        item.name: item.type for item in result.get_all_topics_and_types()
    }
    classes = {
        topic: get_message(type_name)
        for topic, type_name in type_names.items()
    }
    return result, classes


def read_index(path):
    wanted = {
        "/image_raw", "/lane_yolo/detections", "/scene_yolo/detections",
        "/center_curve", "/lane_control_state_stamped",
        "/lane_control_state_v2", "/stanley/debug",
        "/traffic_light_state", "/mission_mode", "/xycar_motor",
        "/perception/scene/image",
    }
    bag, classes = reader(path)
    maps = {topic: {} for topic in wanted}
    timeline = {topic: [] for topic in wanted}
    first_raw_source = 0
    while bag.has_next():
        topic, payload, record = bag.read_next()
        if topic not in wanted:
            continue
        message = deserialize_message(payload, classes[topic])
        source = header_ns(message)
        if topic == "/image_raw" and source > 0 and first_raw_source == 0:
            first_raw_source = source
        if source > 0:
            maps[topic].setdefault(source, (int(record), message))
        timeline[topic].append((int(record), message))
    return maps, timeline, first_raw_source, classes


def image_to_bgr(message):
    if message.encoding not in ("bgr8", "rgb8"):
        return CvBridge().imgmsg_to_cv2(message, "bgr8")
    channels = 3
    frame = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
        int(message.height), int(message.step) // channels, channels
    )[:, : int(message.width)].copy()
    if message.encoding == "rgb8":
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame


def filtered_lane_detections(message, height_threshold):
    return [
        det for det in message.detections
        if str(det.class_name) == "center_line"
        and float(det.confidence) >= 0.0
        and int(det.ymax) > height_threshold
    ]


def hough_counts(frame, detections, params):
    raw_total = 0
    valid_total = 0
    per_bbox = []
    h, w = frame.shape[:2]
    for det in detections:
        x1, y1, x2, y2 = map(
            int, (det.xmin, det.ymin, det.xmax, det.ymax)
        )
        y1, y2 = max(0, y1), min(h, y2)
        x1, x2 = max(0, x1), min(w, x2)
        if (
            x2 - x1 < params["min_bbox_width"]
            or y2 - y1 < params["min_bbox_height"]
            or x1 >= x2 or y1 >= y2
        ):
            per_bbox.append({"raw": 0, "valid": 0})
            continue
        roi = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        threshold = cv2.adaptiveThreshold(
            gray, 255, params["adapt_method"], cv2.THRESH_BINARY,
            params["block_size"], params["c_constant"],
        )
        edges = cv2.Canny(
            threshold, params["canny_thresh1"], params["canny_thresh2"],
            apertureSize=3,
        )
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180,
            threshold=params["hough_threshold"],
            minLineLength=params["hough_min_len"],
            maxLineGap=params["hough_max_gap"],
        )
        raw = 0 if lines is None else len(lines)
        valid = 0
        diag_dx, diag_dy = x2 - x1, y1 - y2
        diag_angle = math.degrees(math.atan2(diag_dy, diag_dx))
        if diag_angle < 0:
            diag_angle += 180
        diag_length = math.hypot(diag_dx, diag_dy)
        if lines is not None and diag_length > 0:
            for line in lines:
                lx1, ly1, lx2, ly2 = line[0]
                line_length = math.hypot(lx2 - lx1, ly2 - ly1)
                if line_length <= 0:
                    continue
                line_angle = math.degrees(math.atan2(-(ly2 - ly1), lx2 - lx1))
                if line_angle < 0:
                    line_angle += 180
                angle_diff = min(
                    abs(line_angle - diag_angle),
                    180 - abs(line_angle - diag_angle),
                )
                if (
                    angle_diff < params["angle_similarity_thresh"]
                    and line_length / diag_length > params["length_similarity_thresh"]
                ):
                    valid += 1
        raw_total += raw
        valid_total += valid
        per_bbox.append({"raw": raw, "valid": valid})
    return raw_total, valid_total, per_bbox


def nearest_source(source, mapping, tolerance_ns=120_000_000):
    if source in mapping:
        return mapping[source]
    if not mapping:
        return None
    keys = np.asarray(sorted(mapping), dtype=np.int64)
    index = int(np.argmin(np.abs(keys - int(source))))
    key = int(keys[index])
    if abs(key - source) > tolerance_ns:
        return None
    return mapping[key]


def put_lines(frame, lines, origin=(8, 18), color=(255, 255, 255)):
    x, y = origin
    for line in lines:
        cv2.putText(
            frame, str(line), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.43,
            color, 1, cv2.LINE_AA,
        )
        y += 17


def annotate_lane(
    node, frame, detection_message, curve_message, state_message,
    debug_message, state_v2_message, source_rel,
):
    params = node._scaled_processing_parameters(frame.shape[1], frame.shape[0])
    detections = filtered_lane_detections(
        detection_message, params["height_threshold"]
    )
    reps, lines, _ = node._extract_representative_points(
        frame, detections, params
    )
    sorted_reps = node._sort_representative_points(
        reps, params, 0, frame.shape[1]
    )
    raw_hough, valid_hough, per_bbox = hough_counts(
        frame, detections, params
    )
    for det in detection_message.detections:
        color = (0, 220, 0) if str(det.class_name) == "center_line" else (120, 120, 120)
        cv2.rectangle(
            frame, (int(det.xmin), int(det.ymin)),
            (int(det.xmax), int(det.ymax)), color, 1,
        )
        cv2.putText(
            frame, f"{float(det.confidence):.2f}",
            (int(det.xmin), max(10, int(det.ymin) - 3)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA,
        )
    for line in lines:
        if len(line) >= 2:
            cv2.line(
                frame, tuple(map(int, line[0])), tuple(map(int, line[1])),
                (255, 80, 0), 2,
            )
    for point in sorted_reps:
        cv2.circle(frame, tuple(map(int, point)), 4, (0, 0, 255), -1)
    if curve_message is not None and curve_message.points:
        curve = np.array(
            [(point.x, point.y) for point in curve_message.points],
            dtype=np.int32,
        ).reshape((-1, 1, 2))
        cv2.polylines(frame, [curve], False, (255, 0, 255), 2)
    target_x = float(state_message.state.target_point.x) if state_message else math.nan
    if np.isfinite(target_x):
        cv2.circle(frame, (int(round(target_x)), 60), 6, (0, 255, 255), 2)
        cv2.line(frame, (int(round(target_x)), 50), (int(round(target_x)), 70),
                 (0, 255, 255), 1)
    signed = (
        float(state_v2_message.signed_curvature_hint)
        if state_v2_message is not None else math.nan
    )
    state_name = str(state_message.curve_state) if state_message else "missing"
    heading_source = str(state_message.heading_source) if state_message else "missing"
    cte = float(debug_message.cte_pixels) if debug_message else math.nan
    heading = (
        math.degrees(float(debug_message.heading_error_rad))
        if debug_message else math.nan
    )
    severity = float(debug_message.curvature_severity) if debug_message else math.nan
    steering = float(debug_message.final_servo_angle) if debug_message else math.nan
    speed = float(debug_message.final_speed) if debug_message else math.nan
    text_panel = np.zeros((112, frame.shape[1], 3), dtype=np.uint8)
    panel_lines = [
        f"t={source_rel:.3f}s  YOLO={len(detection_message.detections)} filtered={len(detections)}",
        f"Hough raw/valid={raw_hough}/{valid_hough} success_bbox={len(lines)} fallback_bbox={max(0, len(detections)-len(lines))}",
        f"path={state_name} heading_src={heading_source} reps={len(sorted_reps)} curve_pts={len(curve_message.points) if curve_message else 0}",
        f"target={target_x:.1f} CTE={cte:.1f}px heading={heading:.1f}deg signedK={signed:.3f} severity={severity:.3f}",
        f"steer={steering:.2f} speed={speed:.2f}",
    ]
    put_lines(text_panel, panel_lines, origin=(8, 17))
    annotated = np.vstack((frame, text_panel))
    diagnostics = {
        "filtered_bbox_count": len(detections),
        "hough_raw_candidate_count": raw_hough,
        "hough_valid_candidate_count": valid_hough,
        "hough_success_bbox_count": len(lines),
        "bbox_center_fallback_count": max(0, len(detections) - len(lines)),
        "representative_point_count": len(sorted_reps),
        "representative_points_json": json.dumps(
            [[round(float(x), 3), round(float(y), 3)] for x, y in sorted_reps]
        ),
        "hough_lines_json": json.dumps([
            [[round(float(x), 3), round(float(y), 3)] for x, y in line]
            for line in lines
        ]),
        "per_bbox_hough_counts_json": json.dumps(per_bbox),
    }
    return annotated, diagnostics


def render_lane(name, node):
    path = BAG_ROOT / name
    maps, timeline, first_source, classes = read_index(path)
    suspects = pd.read_csv(OUT / "lane_perception_suspect_events.csv")
    suspects = suspects[suspects.bag == name]
    suspect_reasons = dict(zip(
        suspects.source_timestamp_ns.astype(np.int64), suspects.reason
    ))
    bag, classes2 = reader(path)
    output_path = OUT / f"lane_perception_{name}.mp4"
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), 12.0, (640, 704)
    )
    rows = []
    saved = 0
    while bag.has_next():
        topic, payload, record = bag.read_next()
        if topic != "/perception/lane/image":
            continue
        image = deserialize_message(payload, classes2[topic])
        source = header_ns(image)
        det_item = nearest_source(source, maps["/lane_yolo/detections"])
        if det_item is None:
            continue
        _, detection = det_item
        curve_item = nearest_source(source, maps["/center_curve"])
        state_item = nearest_source(source, maps["/lane_control_state_stamped"])
        debug_item = nearest_source(source, maps["/stanley/debug"])
        state_v2_item = nearest_source(source, maps["/lane_control_state_v2"])
        frame = image_to_bgr(image)
        annotated, diagnostic = annotate_lane(
            node, frame, detection,
            curve_item[1] if curve_item else None,
            state_item[1] if state_item else None,
            debug_item[1] if debug_item else None,
            state_v2_item[1] if state_v2_item else None,
            (source - first_source) / 1e9,
        )
        annotated = cv2.resize(annotated, (640, 704), interpolation=cv2.INTER_NEAREST)
        writer.write(annotated)
        row = {
            "bag": name,
            "source_timestamp_ns": source,
            **diagnostic,
        }
        rows.append(row)
        reason = suspect_reasons.get(source)
        if reason and saved < 80:
            image_out = annotated.copy()
            cv2.rectangle(image_out, (0, 0), (639, 30), (0, 0, 160), -1)
            cv2.putText(image_out, reason[:95], (8, 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imwrite(str(
                SUSPECT_DIR / f"{name}_{source}_{saved:03d}.jpg"
            ), image_out)
            saved += 1
    writer.release()
    return pd.DataFrame(rows)


def state_before(timeline, record_ns, default="unknown"):
    value = default
    for time_ns, message in timeline:
        if time_ns > record_ns:
            break
        value = message.data
    return value


def motor_before(timeline, record_ns):
    value = [math.nan, math.nan]
    for time_ns, message in timeline:
        if time_ns > record_ns:
            break
        value = list(message.data)
    return value


def render_traffic(name):
    path = BAG_ROOT / name
    maps, timeline, first_source, _ = read_index(path)
    bag, classes = reader(path)
    output_path = OUT / f"traffic_overlay_{name.split('_')[-1]}.mp4"
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), 3.0, (640, 600)
    )
    while bag.has_next():
        topic, payload, record = bag.read_next()
        if topic != "/perception/scene/image":
            continue
        image = deserialize_message(payload, classes[topic])
        source = header_ns(image)
        frame = image_to_bgr(image)
        det_item = nearest_source(
            source, maps["/scene_yolo/detections"], tolerance_ns=50_000_000
        )
        detection_age = math.nan
        detection_summary = "none/missing"
        if det_item is not None:
            det_record, detection = det_item
            detection_age = (det_record - source) / 1e6
            labels = []
            for det in detection.detections:
                color = (0, 0, 255) if str(det.class_name).lower() == "red" else (0, 255, 0)
                cv2.rectangle(frame, (int(det.xmin), int(det.ymin)),
                              (int(det.xmax), int(det.ymax)), color, 2)
                label = f"{det.class_name} {float(det.confidence):.2f}"
                labels.append(label)
                cv2.putText(frame, label, (int(det.xmin), max(15, int(det.ymin)-4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            detection_summary = ", ".join(labels) if labels else "empty"
            logical_record = det_record
        else:
            logical_record = int(record)
        traffic_state = state_before(
            timeline["/traffic_light_state"], logical_record
        )
        mission_mode = state_before(timeline["/mission_mode"], logical_record)
        motor = motor_before(timeline["/xycar_motor"], logical_record)
        panel = np.zeros((120, 640, 3), dtype=np.uint8)
        put_lines(panel, [
            f"source t={(source-first_source)/1e9:.3f}s detector source_age={detection_age:.1f}ms",
            f"detections: {detection_summary}",
            f"traffic={traffic_state} mission={mission_mode}",
            f"final motor angle/speed={motor[:2]}",
            "NOTE: traffic/mission strings have no source header; shown at detector record time",
        ], origin=(8, 18))
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
        writer.write(np.vstack((frame, panel)))
    writer.release()


def merge_hough_diagnostics(frames):
    diagnostics = pd.concat(frames, ignore_index=True)
    path = OUT / "lane_hough_statistics.csv"
    original = pd.read_csv(path)
    drop = [column for column in diagnostics.columns if column in original.columns
            and column not in ("bag", "source_timestamp_ns")]
    original = original.drop(columns=drop)
    merged = original.merge(
        diagnostics, on=["bag", "source_timestamp_ns"], how="left"
    )
    merged["hough_success"] = merged.hough_success_bbox_count.fillna(0).gt(0)
    merged["bbox_center_fallback_used"] = merged.bbox_center_fallback_count.fillna(0).gt(0)
    merged.to_csv(path, index=False)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    SUSPECT_DIR.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = CenterlaneTracer()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
    frames = []
    try:
        for name in BAGS:
            print(f"rendering lane {name}", flush=True)
            frames.append(render_lane(name, node))
            print(f"rendering traffic {name}", flush=True)
            render_traffic(name)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    merge_hough_diagnostics(frames)


if __name__ == "__main__":
    main()
