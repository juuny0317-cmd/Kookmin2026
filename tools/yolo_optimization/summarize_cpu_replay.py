#!/usr/bin/env python3
"""Summarize CPU replay cadence and source-to-command/motor proxies."""

from __future__ import annotations

import argparse
from bisect import bisect_left, bisect_right
from collections import defaultdict
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BASE_TOPICS = (
    '/image_raw', '/perception/lane/image', '/lane_yolo/detections',
    '/center_curve', '/lane_control_state_stamped',
    '/lane_motor_cmd_stamped', '/mission_mode', '/mission_status',
    '/pipeline_timing',
)


def stamp_ns(stamp):
    """Convert a builtin_interfaces/Time-compatible value to nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def describe(values):
    """Return the latency statistics used by the CPU A/B report."""
    data = np.asarray(values, dtype=np.float64)
    data = data[np.isfinite(data)]
    if not data.size:
        return {
            'count': 0, 'median': None, 'p95': None,
            'p99': None, 'max': None,
        }
    return {
        'count': int(data.size),
        'median': float(np.percentile(data, 50)),
        'p95': float(np.percentile(data, 95)),
        'p99': float(np.percentile(data, 99)),
        'max': float(np.max(data)),
    }


def read_bag(path, motor_topic):
    """Read only messages needed for cadence and command timing."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    topics = (*BASE_TOPICS, motor_topic)
    message_types = {
        topic: get_message(types[topic]) for topic in topics if topic in types
    }
    rows = defaultdict(list)
    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        message_type = message_types.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(serialized, message_type)
        source_ns = (
            stamp_ns(message.header.stamp)
            if hasattr(message, 'header') else 0)
        rows[topic].append({
            'bag_ns': int(bag_ns),
            'source_ns': source_ns,
            'message': message,
        })
    return rows


def rate_summary(items):
    """Describe record cadence and, where available, unique-source cadence."""
    duration_s = (
        (items[-1]['bag_ns'] - items[0]['bag_ns']) * 1e-9
        if len(items) > 1 else 0.0)
    sources = [item['source_ns'] for item in items if item['source_ns'] > 0]
    unique_sources = len(set(sources))
    return {
        'message_count': len(items),
        'record_rate_hz': (
            (len(items) - 1) / duration_s if duration_s > 0.0 else None),
        'unique_source_count': unique_sources,
        'unique_source_rate_hz': (
            (unique_sources - 1) / duration_s
            if duration_s > 0.0 and unique_sources > 1 else None),
        'adjacent_duplicate_source_count': sum(
            current == previous
            for previous, current in zip(sources, sources[1:])),
        'adjacent_out_of_order_source_count': sum(
            current < previous
            for previous, current in zip(sources, sources[1:])),
    }


def first_by_source(items):
    """Index the first bag record for each valid source stamp."""
    result = {}
    for item in items:
        source = item['source_ns']
        if source > 0:
            result.setdefault(source, item)
    return result


def mode_at(mode_items, status_items, bag_ns):
    """Resolve the effective mission mode immediately before an output."""
    mode_times = [item['bag_ns'] for item in mode_items]
    index = bisect_right(mode_times, bag_ns) - 1
    if index >= 0:
        return str(mode_items[index]['message'].data).strip().upper()
    status_times = [item['bag_ns'] for item in status_items]
    index = bisect_right(status_times, bag_ns) - 1
    if index < 0:
        return 'UNKNOWN'
    for token in str(status_items[index]['message'].data).split():
        if token.startswith('mode='):
            return token.split('=', 1)[1].strip().upper()
    return 'UNKNOWN'


def command_motor_metrics(rows, motor_topic, stale_threshold_ms):
    """Measure first command/motor record proxies and stale-stop events."""
    commands = rows['/lane_motor_cmd_stamped']
    first_commands = first_by_source(commands)
    motors = rows[motor_topic]
    motor_times = [item['bag_ns'] for item in motors]
    source_to_command = []
    source_to_motor = []
    command_to_motor = []
    for source, command in sorted(first_commands.items()):
        source_to_command.append((command['bag_ns'] - source) * 1e-6)
        index = bisect_left(motor_times, command['bag_ns'])
        if index < len(motors):
            motor_ns = motors[index]['bag_ns']
            source_to_motor.append((motor_ns - source) * 1e-6)
            command_to_motor.append((motor_ns - command['bag_ns']) * 1e-6)

    command_times = [item['bag_ns'] for item in commands]
    mode_items = rows['/mission_mode']
    status_items = rows['/mission_status']
    mode_times = [item['bag_ns'] for item in mode_items]
    stale_events = []
    for motor in motors:
        index = bisect_right(command_times, motor['bag_ns']) - 1
        if index < 0:
            continue
        command = commands[index]
        source = command['source_ns']
        motor_data = list(motor['message'].data)
        if source <= 0 or len(motor_data) < 2:
            continue
        source_age_ms = (motor['bag_ns'] - source) * 1e-6
        lane_speed = float(command['message'].speed)
        motor_speed = float(motor_data[1])
        mode = mode_at(mode_items, status_items, motor['bag_ns'])
        mode_index = bisect_right(mode_times, motor['bag_ns']) - 1
        transition_age_ms = (
            (motor['bag_ns'] - mode_items[mode_index]['bag_ns']) * 1e-6
            if mode_index >= 0 else None)
        outside_transition_stop = (
            transition_age_ms is None or transition_age_ms >= 500.0)
        if (motor_speed == 0.0 and lane_speed > 0.0
                and source_age_ms >= stale_threshold_ms
                and mode == 'LANE' and outside_transition_stop):
            stale_events.append({
                'motor_record_ns': motor['bag_ns'],
                'source_ns': source,
                'source_age_ms': source_age_ms,
            })
    return {
        'clock_note': (
            'rosbag record timestamp minus source header; offline proxy, '
            'not a physical motor-driver timestamp'),
        'source_to_first_lane_command_record_ms': describe(
            source_to_command),
        'source_to_first_following_motor_record_ms': describe(
            source_to_motor),
        'lane_command_to_first_following_motor_record_ms': describe(
            command_to_motor),
        'stale_threshold_ms': stale_threshold_ms,
        'mission_freshness_speed_zero_count': len(stale_events),
        'stale_events': stale_events,
    }


def pipeline_summary(items):
    """Summarize timing stages and latest worker counters."""
    by_stage = defaultdict(list)
    for item in items:
        by_stage[str(item['message'].stage)].append(item)
    selected = {}
    latest = {}
    for stage, stage_items in sorted(by_stage.items()):
        message = stage_items[-1]['message']
        latest[stage] = {
            'received_count': int(message.received_count),
            'processed_count': int(message.processed_count),
            'replaced_count': int(message.replaced_count),
        }
        if stage in (
                'frame_router_lane', 'lane_yolo', 'lane_yolo_inference',
                'centerlane_curve', 'lane_detector_hough',
                'stanley_controller'):
            selected[stage] = {
                'event_count': len(stage_items),
                'compute_ms': describe([
                    float(item['message'].compute_ms)
                    for item in stage_items]),
                'queue_wait_ms': describe([
                    float(item['message'].queue_wait_ms)
                    for item in stage_items]),
            }
    return {'selected_stages': selected, 'latest_counters': latest}


def main():
    """Run the replay summary CLI."""
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('--motor-topic', default='/cpu_ab/motor')
    parser.add_argument('--stale-threshold-ms', type=float, default=300.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    bag = args.bag.expanduser().resolve()
    rows = read_bag(bag, args.motor_topic)
    cadence_topics = (
        '/image_raw', '/perception/lane/image', '/lane_yolo/detections',
        '/center_curve', '/lane_control_state_stamped',
        '/lane_motor_cmd_stamped', args.motor_topic,
    )
    report = {
        'bag': str(bag),
        'motor_topic': args.motor_topic,
        'cadence': {
            topic: rate_summary(rows[topic]) for topic in cadence_topics
        },
        'command_motor_proxies': command_motor_metrics(
            rows, args.motor_topic, args.stale_threshold_ms),
        'pipeline_timing': pipeline_summary(rows['/pipeline_timing']),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
