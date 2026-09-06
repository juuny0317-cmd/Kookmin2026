#!/usr/bin/env python3
"""Source-aligned analysis for the 2026-08-10 full-drive bags.

This script is deliberately read-only with respect to the input bags.  It
extracts the recorded production outputs, joins messages by original camera
stamp where possible, evaluates longitudinal policies open-loop, and writes
tables/plots below the requested Downloads analysis directory.
"""

from __future__ import annotations

import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG_ROOT = Path("/home/xytron/rosbags")
BAG_NAMES = (
    "FULL_DRIVE_20260810_112624",
    "FULL_DRIVE_20260810_112801",
)
OUT = Path(
    "/home/xytron/Downloads/analysis/full_drive_20260810_four_problem"
)


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def header_stamp(message) -> int:
    if hasattr(message, "header"):
        return stamp_ns(message.header.stamp)
    if hasattr(message, "control") and hasattr(message.control, "header"):
        return stamp_ns(message.control.header.stamp)
    if hasattr(message, "debug") and hasattr(message.debug, "header"):
        return stamp_ns(message.debug.header.stamp)
    return 0


def stats(values) -> dict:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    return {
        "count": int(len(array)),
        "median": float(np.median(array)) if len(array) else math.nan,
        "p95": float(np.percentile(array, 95)) if len(array) else math.nan,
        "max": float(np.max(array)) if len(array) else math.nan,
    }


def topic_rate(records) -> float:
    if len(records) < 2:
        return math.nan
    duration = (records[-1][0] - records[0][0]) * 1e-9
    return (len(records) - 1) / duration if duration > 0 else math.nan


def open_reader(path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_names = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    classes = {
        topic: get_message(type_name)
        for topic, type_name in type_names.items()
    }
    return reader, type_names, classes


TOPICS = {
    "/image_raw",
    "/perception/lane/image",
    "/perception/scene/image",
    "/lane_yolo/detections",
    "/scene_yolo/detections",
    "/center_curve",
    "/lane_control_state_stamped",
    "/lane_control_state_v2",
    "/stanley/debug",
    "/stanley/debug_v2",
    "/lane_motor_cmd_stamped",
    "/lane_motor_cmd",
    "/xycar_motor",
    "/traffic_light_state",
    "/sign_color",
    "/mission_mode",
    "/mission_status",
    "/rosout",
}


def read_bag(name: str) -> dict:
    reader, type_names, classes = open_reader(BAG_ROOT / name)
    records = defaultdict(list)
    first_record_ns = None
    while reader.has_next():
        topic, payload, record_ns = reader.read_next()
        if first_record_ns is None:
            first_record_ns = int(record_ns)
        if topic not in TOPICS:
            continue
        message = deserialize_message(payload, classes[topic])
        # Do not retain multi-megabyte pixels in this metrics pass.
        if topic in {
            "/image_raw", "/perception/lane/image", "/perception/scene/image"
        }:
            records[topic].append((
                int(record_ns), header_stamp(message),
                int(message.width), int(message.height),
            ))
        else:
            records[topic].append((int(record_ns), message))
    raw_sources = [x[1] for x in records["/image_raw"] if x[1] > 0]
    return {
        "name": name,
        "path": BAG_ROOT / name,
        "types": type_names,
        "records": records,
        "first_record_ns": int(first_record_ns or 0),
        "first_source_ns": min(raw_sources) if raw_sources else 0,
    }


def source_rel_s(bag: dict, source: int) -> float:
    return (int(source) - bag["first_source_ns"]) * 1e-9


def record_rel_s(bag: dict, record: int) -> float:
    return (int(record) - bag["first_record_ns"]) * 1e-9


def to_source_map(records) -> dict:
    result = {}
    for record_ns, message in records:
        source = header_stamp(message)
        if source > 0 and source not in result:
            result[source] = (record_ns, message)
    return result


def consecutive_run_lengths(flags) -> Counter:
    counts = Counter()
    run = 0
    for flag in list(flags) + [False]:
        if flag:
            run += 1
        elif run:
            counts[run] += 1
            run = 0
    return counts


def nearest_future(frame: pd.DataFrame, index: int, seconds: float):
    target = frame.iloc[index].source_time_s + seconds
    later = frame.iloc[index:]
    position = int(np.searchsorted(later.source_time_s.to_numpy(), target))
    if position >= len(later):
        return None
    return later.iloc[position]


def switch_count(values) -> int:
    values = np.asarray(values, dtype=float)
    return int(np.sum(np.abs(np.diff(values)) > 1e-6))


def quick_reversal_count(values, times, window_s=0.5) -> int:
    values = np.asarray(values, dtype=float)
    times = np.asarray(times, dtype=float)
    changes = np.where(np.abs(np.diff(values)) > 1e-6)[0] + 1
    count = 0
    for pos, index in enumerate(changes[:-1]):
        next_index = changes[pos + 1]
        if times[next_index] - times[index] <= window_s:
            count += 1
    return count


def alignment_policy(frame, dwell, hysteresis=False):
    """22 only after position+heading alignment; drop early on approach."""
    output = []
    aligned_count = 0
    high = False
    for row in frame.itertuples():
        enter_aligned = (
            row.curve_state == "Straight"
            and abs(row.cte_px) <= 45.0
            and abs(row.heading_deg) <= 5.0
            and abs(row.final_steering) <= 5.0
            and row.curvature_severity <= 0.30
            and row.path_confidence >= 0.50
        )
        leave_high = (
            row.curve_state == "Curve"
            or abs(row.cte_px) >= 60.0
            or abs(row.heading_deg) >= 8.0
            or abs(row.final_steering) >= 7.0
            or row.curvature_severity >= 0.45
        )
        aligned_count = aligned_count + 1 if enter_aligned else 0
        if leave_high:
            high = False
        elif aligned_count >= dwell:
            high = True
        elif not hysteresis:
            high = False
        output.append(22.0 if high else 12.0)
    return np.asarray(output)


def continuous_preview(frame):
    severity = np.clip(
        (frame.curvature_severity.to_numpy(float) - 0.15) / 0.50,
        0.0,
        1.0,
    )
    return 22.0 - 10.0 * severity


def make_debug_table(bag: dict) -> pd.DataFrame:
    debug = to_source_map(bag["records"]["/stanley/debug"])
    v2 = to_source_map(bag["records"]["/stanley/debug_v2"])
    states = to_source_map(bag["records"]["/lane_control_state_stamped"])
    state_v2 = to_source_map(bag["records"]["/lane_control_state_v2"])
    rows = []
    for source in sorted(debug):
        record_ns, message = debug[source]
        state_item = states.get(source)
        state = state_item[1] if state_item else None
        v2_item = v2.get(source)
        extended = v2_item[1] if v2_item else None
        state_v2_item = state_v2.get(source)
        extended_state = state_v2_item[1] if state_v2_item else None
        rows.append({
            "bag": bag["name"],
            "source_timestamp_ns": source,
            "source_time_s": source_rel_s(bag, source),
            "debug_record_timestamp_ns": record_ns,
            "record_time_s": record_rel_s(bag, record_ns),
            "source_to_debug_ms": (record_ns - source) / 1e6,
            "curve_state": str(message.curve_state),
            "heading_source": str(message.heading_source),
            "curvature": float(state.curvature) if state else math.nan,
            "signed_curvature": (
                float(extended_state.signed_curvature_hint)
                if extended_state else math.nan
            ),
            "curvature_severity": float(message.curvature_severity),
            "path_confidence": float(message.path_confidence),
            "target_x": float(state.state.target_point.x) if state else (
                320.0 + float(message.cte_pixels)
            ),
            "cte_px": float(message.cte_pixels),
            "heading_rad": float(message.heading_error_rad),
            "heading_deg": math.degrees(float(message.heading_error_rad)),
            "heading_term_rad": float(message.heading_term_rad),
            "cte_term_rad": float(message.cte_term_rad),
            "raw_cte_term_rad": (
                float(extended.raw_cte_term_rad)
                if extended else float(message.cte_term_rad)
            ),
            "term_conflict": (
                float(message.heading_term_rad) * float(message.cte_term_rad) < 0
            ),
            "raw_steering": float(message.raw_servo_angle),
            "final_steering": float(message.final_servo_angle),
            "target_speed": float(message.target_speed),
            "final_speed": float(message.final_speed),
            "source_dt_s": float(message.source_dt_s),
            "state_age_s": float(message.state_age_s),
            "rate_limited": bool(message.steering_rate_limited),
        })
    frame = pd.DataFrame(rows)
    if frame.empty:
        return frame
    frame["delta_cte_px"] = frame.cte_px.diff()
    frame["delta_heading_deg"] = frame.heading_deg.diff()
    frame["delta_steering"] = frame.final_steering.diff()
    frame["delta_target_speed"] = frame.target_speed.diff()
    return frame


def lane_tables(bag: dict, debug: pd.DataFrame):
    detection_rows = []
    det_map = to_source_map(bag["records"]["/lane_yolo/detections"])
    previous_centers = None
    previous_source = None
    for source in sorted(det_map):
        record_ns, message = det_map[source]
        detections = [
            item for item in message.detections
            if str(item.class_name) == "center_line"
        ]
        centers = sorted(
            ((item.xmin + item.xmax) * 0.5, (item.ymin + item.ymax) * 0.5)
            for item in detections
        )
        mean_x = float(np.mean([x for x, _ in centers])) if centers else math.nan
        mean_y = float(np.mean([y for _, y in centers])) if centers else math.nan
        continuity_jump = math.nan
        if centers and previous_centers:
            continuity_jump = max(
                min(abs(x - old_x) for old_x, _ in previous_centers)
                for x, _ in centers
            )
        confidence = [float(item.confidence) for item in detections]
        widths = [int(item.xmax - item.xmin) for item in detections]
        heights = [int(item.ymax - item.ymin) for item in detections]
        detection_rows.append({
            "bag": bag["name"],
            "source_timestamp_ns": source,
            "source_time_s": source_rel_s(bag, source),
            "record_timestamp_ns": record_ns,
            "source_to_detection_ms": (record_ns - source) / 1e6,
            "source_gap_ms": (
                (source - previous_source) / 1e6
                if previous_source is not None else math.nan
            ),
            "center_line_count": len(detections),
            "confidence_min": min(confidence) if confidence else math.nan,
            "confidence_mean": float(np.mean(confidence)) if confidence else math.nan,
            "confidence_max": max(confidence) if confidence else math.nan,
            "bbox_center_x_mean": mean_x,
            "bbox_center_y_mean": mean_y,
            "bbox_width_mean": float(np.mean(widths)) if widths else math.nan,
            "bbox_height_mean": float(np.mean(heights)) if heights else math.nan,
            "bbox_match_jump_px": continuity_jump,
            "miss": len(detections) == 0,
        })
        previous_centers = centers
        previous_source = source
    yolo = pd.DataFrame(detection_rows)

    curve_map = to_source_map(bag["records"]["/center_curve"])
    debug_map = debug.set_index("source_timestamp_ns") if not debug.empty else None
    rows = []
    previous_target = math.nan
    previous_heading = math.nan
    previous_signed = math.nan
    for source in sorted(to_source_map(
        bag["records"]["/lane_control_state_stamped"]
    )):
        _, state = to_source_map(
            bag["records"]["/lane_control_state_stamped"]
        )[source]
        curve = curve_map.get(source, (0, None))[1]
        target = float(state.state.target_point.x)
        heading = math.degrees(float(state.state.target_point.z))
        state_v2 = to_source_map(
            bag["records"]["/lane_control_state_v2"]
        ).get(source, (0, None))[1]
        signed = float(state_v2.signed_curvature_hint) if state_v2 else math.nan
        debug_row = debug_map.loc[source] if debug_map is not None and source in debug_map.index else None
        rows.append({
            "bag": bag["name"],
            "source_timestamp_ns": source,
            "source_time_s": source_rel_s(bag, source),
            "curve_state": str(state.curve_state),
            "curve_point_count": len(curve.points) if curve else 0,
            "heading_source": str(state.heading_source),
            "hough_used": str(state.heading_source) == "hough",
            "curve_hold_used": str(state.heading_source) == "curve_hold",
            "bbox_center_fallback_proxy": str(state.heading_source) == "hough",
            "target_x": target,
            "delta_target_x": target - previous_target,
            "heading_deg": heading,
            "delta_heading_deg": heading - previous_heading,
            "curvature": float(state.curvature),
            "signed_curvature": signed,
            "delta_signed_curvature": signed - previous_signed,
            "curvature_severity": float(state.curvature_severity),
            "path_confidence": float(state.path_confidence),
            "steering": (
                float(debug_row.final_steering)
                if debug_row is not None else math.nan
            ),
            "speed": (
                float(debug_row.final_speed)
                if debug_row is not None else math.nan
            ),
        })
        previous_target, previous_heading, previous_signed = target, heading, signed
    hough = pd.DataFrame(rows)
    return yolo, hough


def acceleration_events(debug: pd.DataFrame) -> pd.DataFrame:
    rows = []
    if debug.empty:
        return pd.DataFrame()
    starts = np.where(debug.delta_target_speed.to_numpy(float) > 1e-6)[0]
    for number, index in enumerate(starts, 1):
        row = debug.iloc[index]
        plus03 = nearest_future(debug, index, 0.3)
        plus05 = nearest_future(debug, index, 0.5)
        later = debug.iloc[index + 1:]
        decel = later[
            (later.source_time_s <= row.source_time_s + 0.5)
            & (later.delta_target_speed < -1e-6)
        ]
        rows.append({
            "bag": row.bag,
            "event": number,
            "source_timestamp_ns": int(row.source_timestamp_ns),
            "time_s": row.source_time_s,
            "speed_before": debug.iloc[index - 1].target_speed if index else math.nan,
            "speed_after": row.target_speed,
            "final_speed": row.final_speed,
            "cte_at_accel_px": row.cte_px,
            "abs_cte_at_accel_px": abs(row.cte_px),
            "heading_at_accel_deg": row.heading_deg,
            "abs_heading_at_accel_deg": abs(row.heading_deg),
            "steering_at_accel": row.final_steering,
            "term_conflict": row.term_conflict,
            "cte_0p3s_px": plus03.cte_px if plus03 is not None else math.nan,
            "cte_0p5s_px": plus05.cte_px if plus05 is not None else math.nan,
            "steering_0p3s": plus03.final_steering if plus03 is not None else math.nan,
            "steering_0p5s": plus05.final_steering if plus05 is not None else math.nan,
            "decelerated_within_0p5s": not decel.empty,
        })
    return pd.DataFrame(rows)


def curve_events(debug: pd.DataFrame) -> pd.DataFrame:
    rows = []
    if debug.empty:
        return pd.DataFrame()
    curve = debug.curve_state.eq("Curve").to_numpy()
    entries = np.where(curve & ~np.r_[False, curve[:-1]])[0]
    for number, index in enumerate(entries, 1):
        event = debug.iloc[index]
        prior = debug[
            debug.source_time_s.between(event.source_time_s - 1.0, event.source_time_s)
        ]
        indicators = prior[
            (prior.curvature_severity >= 0.30)
            | (prior.heading_deg.abs() >= 8.0)
        ]
        speed_down = prior[prior.delta_target_speed < -1e-6]
        first = indicators.iloc[0] if not indicators.empty else None
        rows.append({
            "bag": event.bag,
            "event": number,
            "curve_state_source_ns": int(event.source_timestamp_ns),
            "curve_state_time_s": event.source_time_s,
            "first_continuous_indicator_time_s": first.source_time_s if first is not None else math.nan,
            "indicator_lead_ms": (
                (event.source_time_s - first.source_time_s) * 1000
                if first is not None else math.nan
            ),
            "first_indicator_severity": first.curvature_severity if first is not None else math.nan,
            "first_indicator_heading_deg": first.heading_deg if first is not None else math.nan,
            "recorded_speed_reduction_time_s": (
                speed_down.iloc[0].source_time_s if not speed_down.empty else math.nan
            ),
            "target_speed_at_state": event.target_speed,
            "final_speed_at_state": event.final_speed,
            "steering_at_state": event.final_steering,
        })
    return pd.DataFrame(rows)


def s_events(debug: pd.DataFrame) -> pd.DataFrame:
    rows = []
    if debug.empty:
        return pd.DataFrame()
    signed = debug.signed_curvature.to_numpy(float)
    severity = debug.curvature_severity.to_numpy(float)

    # signed_curvature is numerically noisy around zero.  Counting every raw
    # sign flip produced dozens of false "S reversals".  A direction is valid
    # only after two consecutive, geometrically meaningful samples; values in
    # the dead band retain the last confirmed direction.
    qualified = (
        np.isfinite(signed)
        & (np.abs(signed) >= 0.25)
        & np.isfinite(severity)
        & (severity >= 0.30)
    )
    confirmed_sign = 0
    confirmed_value = math.nan
    pending_sign = 0
    pending_count = 0
    pending_first_index = None
    reversals = []
    before_values = []
    for index in range(len(debug)):
        if not qualified[index]:
            pending_sign = 0
            pending_count = 0
            pending_first_index = None
            continue
        candidate = 1 if signed[index] > 0.0 else -1
        if candidate != pending_sign:
            pending_sign = candidate
            pending_count = 1
            pending_first_index = index
        else:
            pending_count += 1
        if pending_count < 2:
            continue
        if confirmed_sign != 0 and candidate != confirmed_sign:
            reversals.append(pending_first_index)
            before_values.append(confirmed_value)
        confirmed_sign = candidate
        confirmed_value = signed[index]

    for number, index in enumerate(reversals, 1):
        event = debug.iloc[index]
        window = debug[
            debug.source_time_s.between(event.source_time_s - 0.75, event.source_time_s + 0.75)
        ]
        rows.append({
            "bag": event.bag,
            "event": number,
            "source_timestamp_ns": int(event.source_timestamp_ns),
            "reversal_time_s": event.source_time_s,
            "confirmed_direction_before": (
                1 if before_values[number - 1] > 0.0 else -1
            ),
            "signed_curvature_before": before_values[number - 1],
            "signed_curvature_after": signed[index],
            "heading_deg": event.heading_deg,
            "steering": event.final_steering,
            "recorded_target_speed": event.target_speed,
            "recorded_final_speed": event.final_speed,
            "window_max_severity": window.curvature_severity.max(),
        })
    return pd.DataFrame(rows)


def policy_metrics(debug: pd.DataFrame, bag_name: str):
    policies = {
        "recorded": debug.target_speed.to_numpy(float),
        "alignment_dwell1": alignment_policy(debug, 1),
        "alignment_dwell2": alignment_policy(debug, 2),
        "alignment_dwell3": alignment_policy(debug, 3),
        "alignment_dwell1_hysteresis": alignment_policy(debug, 1, True),
        "alignment_dwell2_hysteresis": alignment_policy(debug, 2, True),
        "alignment_dwell3_hysteresis": alignment_policy(debug, 3, True),
    }
    preview = continuous_preview(debug)
    policies["continuous_curvature"] = preview
    policies["steering_heading_aware"] = np.minimum(
        np.where(
            (debug.final_steering.abs() > 5.0) | (debug.heading_deg.abs() > 8.0),
            12.0,
            22.0,
        ),
        22.0,
    )
    policies["hybrid_alignment_preview"] = np.minimum(
        policies["alignment_dwell2_hysteresis"], preview
    )
    stable = (
        debug.curve_state.eq("Straight")
        & debug.cte_px.abs().le(45.0)
        & debug.heading_deg.abs().le(5.0)
        & debug.curvature_severity.le(0.30)
        & debug.path_confidence.ge(0.50)
    ).to_numpy()
    unsafe = (
        debug.heading_deg.abs().gt(8.0)
        | debug.cte_px.abs().gt(60.0)
        | debug.curvature_severity.gt(0.45)
    ).to_numpy()
    rows = []
    for name, values in policies.items():
        values = np.asarray(values, dtype=float)
        high = values >= 21.999
        rows.append({
            "bag": bag_name,
            "policy": name,
            "frame_count": len(values),
            "fast_occupancy_percent": float(np.mean(high) * 100),
            "stable_straight_fast_occupancy_percent": (
                float(np.mean(high[stable]) * 100) if np.any(stable) else math.nan
            ),
            "unsafe_fast_frame_count": int(np.sum(high & unsafe)),
            "target_switch_count": switch_count(values),
            "quick_reswitch_within_0p5s": quick_reversal_count(
                values, debug.source_time_s
            ),
            "target_speed_mean": float(np.mean(values)),
            "target_speed_min": float(np.min(values)),
            "target_speed_max": float(np.max(values)),
        })
    return pd.DataFrame(rows), policies


def suspect_table(yolo, hough, debug):
    merged = pd.merge(
        hough,
        yolo[[
            "bag", "source_timestamp_ns", "center_line_count", "miss",
            "bbox_match_jump_px", "source_gap_ms",
        ]],
        on=["bag", "source_timestamp_ns"],
        how="left",
    )
    flags = []
    for row in merged.itertuples():
        reasons = []
        if bool(row.miss) if not pd.isna(row.miss) else False:
            reasons.append("yolo_miss")
        if int(row.curve_point_count) == 0:
            reasons.append("curve_points_empty")
        if bool(row.hough_used):
            reasons.append("hough_heading_fallback")
        if bool(row.curve_hold_used):
            reasons.append("curve_heading_hold")
        if abs(row.delta_target_x) >= 30:
            reasons.append("target_jump_ge_30px")
        if abs(row.delta_heading_deg) >= 8:
            reasons.append("heading_jump_ge_8deg")
        if not pd.isna(row.bbox_match_jump_px) and row.bbox_match_jump_px >= 40:
            reasons.append("bbox_jump_ge_40px")
        if row.path_confidence < 0.50:
            reasons.append("low_path_confidence")
        if reasons:
            flags.append({
                **row._asdict(),
                "reason": ";".join(reasons),
            })
    return pd.DataFrame(flags)


def traffic_tables(bag: dict):
    records = bag["records"]
    rows = []
    scene_detections = records["/scene_yolo/detections"]
    traffic_states = records["/traffic_light_state"]
    mission_modes = records["/mission_mode"]
    motor = records["/xycar_motor"]
    direct_names = {"red", "green", "left"}
    for number, (record_ns, message) in enumerate(scene_detections, 1):
        source = header_stamp(message)
        direct = [
            item for item in message.detections
            if str(item.class_name).lower() in direct_names
        ]
        if not direct:
            continue
        best = max(direct, key=lambda item: float(item.confidence))
        following_state = next((
            (time_ns, msg.data)
            for time_ns, msg in traffic_states if time_ns >= record_ns
        ), (0, "missing"))
        following_mode = next((
            (time_ns, msg.data)
            for time_ns, msg in mission_modes if time_ns >= record_ns
        ), (0, "missing"))
        following_zero = next((
            (time_ns, list(msg.data))
            for time_ns, msg in motor
            if time_ns >= record_ns and len(msg.data) >= 2
            and abs(float(msg.data[1])) < 1e-6
        ), (0, []))
        rows.append({
            "bag": bag["name"],
            "event": number,
            "source_timestamp_ns": source,
            "source_time_s": source_rel_s(bag, source),
            "detection_record_timestamp_ns": record_ns,
            "detection_record_time_s": record_rel_s(bag, record_ns),
            "source_to_detection_ms": (record_ns - source) / 1e6,
            "class": str(best.class_name),
            "confidence": float(best.confidence),
            "xmin": int(best.xmin), "ymin": int(best.ymin),
            "xmax": int(best.xmax), "ymax": int(best.ymax),
            "next_traffic_state": following_state[1],
            "detection_to_state_ms": (
                (following_state[0] - record_ns) / 1e6
                if following_state[0] else math.nan
            ),
            "next_mission_mode": following_mode[1],
            "detection_to_mission_mode_ms": (
                (following_mode[0] - record_ns) / 1e6
                if following_mode[0] else math.nan
            ),
            "detection_to_next_motor_zero_ms": (
                (following_zero[0] - record_ns) / 1e6
                if following_zero[0] else math.nan
            ),
        })

    stage_rows = []
    for topic in [
        "/image_raw", "/perception/lane/image", "/lane_yolo/detections",
        "/perception/scene/image", "/scene_yolo/detections",
        "/traffic_light_state", "/mission_status", "/xycar_motor",
    ]:
        items = records[topic]
        latency = []
        if items and topic in {
            "/image_raw", "/perception/lane/image", "/perception/scene/image"
        }:
            latency = [(x[0] - x[1]) / 1e6 for x in items if x[1] > 0]
        elif topic in {
            "/lane_yolo/detections", "/scene_yolo/detections"
        }:
            latency = [
                (record_ns - header_stamp(message)) / 1e6
                for record_ns, message in items if header_stamp(message) > 0
            ]
        item_stats = stats(latency)
        stage_rows.append({
            "bag": bag["name"],
            "topic": topic,
            "count": len(items),
            "rate_hz": topic_rate(items),
            "source_age_median_ms": item_stats["median"],
            "source_age_p95_ms": item_stats["p95"],
            "source_age_max_ms": item_stats["max"],
        })
    return pd.DataFrame(rows), pd.DataFrame(stage_rows)


def plot_controls(all_debug, policies):
    second = all_debug[all_debug.bag.str.endswith("112801")].copy()
    if second.empty:
        return
    hybrid = policies["FULL_DRIVE_20260810_112801"]["hybrid_alignment_preview"]
    figures = [
        ("straight_alignment_before_after.png", (8.0, 18.0)),
        ("curve_entry_before_after.png", (18.0, 31.0)),
        ("s_speed_before_after.png", (28.0, 41.0)),
    ]
    for filename, limits in figures:
        mask = second.source_time_s.between(*limits).to_numpy()
        if not np.any(mask):
            mask = np.ones(len(second), dtype=bool)
        view = second[mask]
        candidate = hybrid[mask]
        fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
        time_values = view.source_time_s.to_numpy(float)
        axes[0].plot(time_values, view.cte_px.to_numpy(float), label="CTE px")
        axes[0].axhline(45, color="gray", ls="--", lw=0.8)
        axes[0].axhline(-45, color="gray", ls="--", lw=0.8)
        axes[0].set_ylabel("CTE [px]"); axes[0].legend(loc="upper right")
        axes[1].plot(time_values, view.heading_deg.to_numpy(float), label="heading deg")
        axes[1].plot(time_values, view.curvature_severity.to_numpy(float) * 20,
                     label="severity x20", alpha=0.8)
        axes[1].set_ylabel("heading/severity"); axes[1].legend(loc="upper right")
        axes[2].plot(time_values, view.final_steering.to_numpy(float),
                     label="recorded steering")
        axes[2].set_ylabel("steering"); axes[2].legend(loc="upper right")
        axes[3].plot(time_values, view.target_speed.to_numpy(float),
                     label="recorded target", lw=2)
        axes[3].plot(time_values, candidate,
                     label="open-loop hybrid candidate", lw=1.5)
        axes[3].set_ylabel("speed cmd"); axes[3].set_xlabel("source time [s]")
        axes[3].legend(loc="upper right")
        fig.suptitle(filename.replace("_", " "))
        fig.tight_layout()
        fig.savefig(OUT / filename, dpi=150)
        plt.close(fig)


def plot_traffic(bag: dict, suffix: str):
    records = bag["records"]
    fig, ax = plt.subplots(figsize=(14, 5))
    for record_ns, message in records["/scene_yolo/detections"]:
        source = header_stamp(message)
        for det in message.detections:
            cls = str(det.class_name).lower()
            if cls in {"red", "green", "left", "race_traffic_light"}:
                ax.scatter(
                    source_rel_s(bag, source), float(det.confidence) + 2.0,
                    c={"red": "red", "green": "green", "left": "lime"}.get(cls, "black"),
                    marker="o", s=60,
                )
                ax.text(source_rel_s(bag, source), float(det.confidence) + 2.1,
                        f"{cls} {float(det.confidence):.2f}", fontsize=8)
                ax.plot(
                    [source_rel_s(bag, source), record_rel_s(bag, record_ns)],
                    [float(det.confidence) + 2.0] * 2,
                    color="purple", lw=3, alpha=0.6,
                )
    state_y = {"unknown": 0.0, "green": 1.0, "red": 2.0}
    for record_ns, message in records["/traffic_light_state"]:
        ax.scatter(record_rel_s(bag, record_ns), state_y.get(message.data, -0.5),
                   c={"red": "red", "green": "green"}.get(message.data, "gray"),
                   marker="|", s=90)
    for record_ns, message in records["/mission_mode"]:
        ax.axvline(record_rel_s(bag, record_ns), color="orange", alpha=0.5)
        ax.text(record_rel_s(bag, record_ns), -0.35, message.data,
                rotation=90, fontsize=8)
    ax.set_xlabel("time [s] (source for detections; record time for states)")
    ax.set_ylabel("state / detection confidence+2")
    ax.set_title(
        f"Traffic pipeline timeline {bag['name']} (purple = source→detection age)"
    )
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(OUT / f"traffic_red_timeline_{suffix}.png", dpi=160)
    plt.close(fig)


def write_summary(summary):
    (OUT / "analysis_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=True),
        encoding="utf-8",
    )


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    bags = [read_bag(name) for name in BAG_NAMES]
    all_debug = []
    all_yolo = []
    all_hough = []
    all_suspect = []
    all_acceleration = []
    all_curve = []
    all_s = []
    all_policy = []
    all_traffic = []
    all_stages = []
    policy_series = {}
    summary = {"bags": {}}
    for bag in bags:
        debug = make_debug_table(bag)
        yolo, hough = lane_tables(bag, debug)
        suspect = suspect_table(yolo, hough, debug)
        acceleration = acceleration_events(debug)
        curves = curve_events(debug)
        s = s_events(debug)
        policy, series = policy_metrics(debug, bag["name"])
        traffic, stages = traffic_tables(bag)
        all_debug.append(debug); all_yolo.append(yolo); all_hough.append(hough)
        all_suspect.append(suspect); all_acceleration.append(acceleration)
        all_curve.append(curves); all_s.append(s); all_policy.append(policy)
        all_traffic.append(traffic); all_stages.append(stages)
        policy_series[bag["name"]] = series
        miss_runs = consecutive_run_lengths(yolo.miss if not yolo.empty else [])
        summary["bags"][bag["name"]] = {
            "topic_rates": {
                row.topic: row.rate_hz for row in stages.itertuples()
            },
            "lane_yolo_source_age_ms": stats(yolo.source_to_detection_ms),
            "lane_yolo_miss_frames": int(yolo.miss.sum()),
            "lane_yolo_miss_run_histogram": dict(sorted(miss_runs.items())),
            "hough_heading_frames": int(hough.hough_used.sum()),
            "curve_hold_frames": int(hough.curve_hold_used.sum()),
            "empty_curve_frames": int(hough.curve_point_count.eq(0).sum()),
            "debug_source_age_ms": stats(debug.source_to_debug_ms),
            "straight_high_misaligned_frames": int((
                debug.curve_state.eq("Straight")
                & debug.target_speed.gt(12.0)
                & (debug.cte_px.abs().gt(45.0) | debug.heading_deg.abs().gt(5.0))
            ).sum()),
            "term_conflict_frames": int(debug.term_conflict.sum()),
            "traffic_direct_detection_count": len(traffic),
        }
        plot_traffic(bag, bag["name"].split("_")[-1])

    debug_frame = pd.concat(all_debug, ignore_index=True)
    yolo_frame = pd.concat(all_yolo, ignore_index=True)
    hough_frame = pd.concat(all_hough, ignore_index=True)
    suspect_frame = pd.concat(all_suspect, ignore_index=True)
    acceleration_frame = pd.concat(all_acceleration, ignore_index=True)
    curve_frame = pd.concat(all_curve, ignore_index=True)
    s_frame = pd.concat(all_s, ignore_index=True)
    policy_frame = pd.concat(all_policy, ignore_index=True)
    traffic_frame = pd.concat(all_traffic, ignore_index=True)
    stage_frame = pd.concat(all_stages, ignore_index=True)

    yolo_frame.to_csv(OUT / "lane_yolo_statistics.csv", index=False)
    hough_frame.to_csv(OUT / "lane_hough_statistics.csv", index=False)
    suspect_frame.to_csv(OUT / "lane_perception_suspect_events.csv", index=False)
    acceleration_frame.to_csv(OUT / "straight_acceleration_events.csv", index=False)
    curve_frame.to_csv(OUT / "curve_entry_events.csv", index=False)
    s_frame.to_csv(OUT / "s_speed_events.csv", index=False)
    policy_frame.to_csv(OUT / "longitudinal_policy_comparison.csv", index=False)
    traffic_frame.to_csv(OUT / "traffic_latency_events.csv", index=False)
    stage_frame.to_csv(OUT / "traffic_pipeline_stage_timing.csv", index=False)
    debug_frame.to_csv(OUT / "source_aligned_control_timeseries.csv", index=False)
    plot_controls(debug_frame, policy_series)
    write_summary(summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=True))


if __name__ == "__main__":
    main()
