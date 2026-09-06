#!/usr/bin/env python3
"""Build source-aligned freshness statistics, timelines, and plots."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


READ_TOPICS = (
    '/clock',
    '/image_raw',
    '/perception/lane/image',
    '/lane_yolo/detections',
    '/center_curve',
    '/lane_control_state_stamped',
    '/lane_control_state_v2',
    '/stanley/debug',
    '/lane_motor_cmd_stamped',
    '/mission_mode',
    '/mission_status',
    '/pipeline_timing',
    '/camera_timing_complete',
)

SOURCE_STAGE_TOPICS = {
    '/image_raw': 'image_raw',
    '/perception/lane/image': 'frame_router_selected',
    '/lane_yolo/detections': 'yolo_detection',
    '/center_curve': 'center_curve',
    '/lane_control_state_stamped': 'lane_state',
}

PROBE_STAGE_EVENTS = {
    'image_raw_observed': 'image_raw_observed',
    'frame_router_selected_observed': 'frame_router_selected',
    'yolo_detection_observed': 'yolo_detection',
    'center_curve_observed': 'center_curve',
    'lane_state_observed': 'lane_state',
    'stanley_debug_observed': 'stanley_unique',
}

PIPELINE_STAGE_NAMES = {
    'frame_router_lane': 'frame_router_selected_internal',
    'lane_yolo': 'yolo_worker',
    'lane_yolo_model': 'yolo_model',
    'lane_yolo_inference': 'yolo_inference',
    'centerlane_curve': 'center_curve_internal',
    'lane_detector_hough': 'lane_state_internal',
    'stanley_controller': 'stanley_controller',
}


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def describe(values):
    data = np.asarray(list(values), dtype=np.float64)
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


def open_reader(path: Path, wanted_topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    selected = [topic for topic in wanted_topics if topic in topic_types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    classes = {topic: get_message(topic_types[topic]) for topic in selected}
    return reader, classes, topic_types


def message_source_ns(topic, message):
    if topic == '/lane_control_state_v2':
        return stamp_ns(message.control.header.stamp)
    if hasattr(message, 'header'):
        return stamp_ns(message.header.stamp)
    return 0


def find_motor_topic(topic_types):
    candidates = [
        topic for topic, type_name in topic_types.items()
        if topic.startswith('/offline/')
        and type_name == 'std_msgs/msg/Float32MultiArray'
    ]
    return sorted(candidates)[0] if candidates else None


def read_bag(path: Path, raw_only=False):
    wanted = ('/image_raw',) if raw_only else READ_TOPICS
    reader, classes, topic_types = open_reader(path, wanted)
    motor_topic = None if raw_only else find_motor_topic(topic_types)
    if motor_topic is not None and motor_topic not in classes:
        reader, classes, topic_types = open_reader(
            path, (*wanted, motor_topic))

    rows = defaultdict(list)
    while reader.has_next():
        topic, payload, bag_ns = reader.read_next()
        message_type = classes.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(payload, message_type)
        row = {
            'bag_ns': int(bag_ns),
            'source_ns': message_source_ns(topic, message),
        }
        if topic == '/lane_motor_cmd_stamped':
            row.update({
                'command_ns': stamp_ns(message.command_stamp),
                'state_age_s': float(message.state_age_s),
                'angle': float(message.angle),
                'speed': float(message.speed),
            })
        elif topic == '/stanley/debug':
            row.update({
                'state_age_s': float(message.state_age_s),
                'angle': float(message.final_servo_angle),
                'speed': float(message.final_speed),
            })
        elif topic == '/pipeline_timing':
            row.update({
                'stage': str(message.stage),
                'receive_ros_ns': int(message.receive_stamp_ns),
                'start_ros_ns': int(message.start_stamp_ns),
                'end_ros_ns': int(message.end_stamp_ns),
                'queue_wait_ms': float(message.queue_wait_ms),
                'compute_ms': float(message.compute_ms),
                'received_count': int(message.received_count),
                'processed_count': int(message.processed_count),
                'replaced_count': int(message.replaced_count),
                'input_reused': bool(message.input_reused),
            })
        elif topic == '/camera_timing_complete':
            row.update({
                'valid': bool(message.valid),
                'join_valid': bool(message.join_valid),
                'subscriber_receive_ns': int(message.subscriber_receive_ns),
            })
        elif motor_topic is not None and topic == motor_topic:
            values = list(message.data)
            row.update({
                'angle': float(values[0]) if values else math.nan,
                'speed': float(values[1]) if len(values) > 1 else math.nan,
            })
        rows[topic].append(row)
    return rows, motor_topic, topic_types


def unique_source_rows(rows):
    result = []
    seen = set()
    for row in rows:
        source = int(row.get('source_ns', 0))
        if source <= 0 or source in seen:
            continue
        seen.add(source)
        result.append(row)
    return result


def source_intervals(rows):
    unique = unique_source_rows(rows)
    if len(unique) < 2:
        return pd.DataFrame(columns=(
            'previous_source_ns', 'source_ns', 'event_ns', 'dt_ms'))
    return pd.DataFrame([{
        'previous_source_ns': int(previous['source_ns']),
        'source_ns': int(current['source_ns']),
        'event_ns': int(current.get('event_ns', current.get('bag_ns', 0))),
        'dt_ms': (
            int(current['source_ns']) - int(previous['source_ns'])) * 1e-6,
    } for previous, current in zip(unique, unique[1:])])


def event_intervals(rows, time_key='bag_ns'):
    values = [int(row[time_key]) for row in rows if int(row.get(time_key, 0))]
    return np.diff(np.asarray(values, dtype=np.int64)) * 1e-6


def event_interval_frame(rows, time_key='bag_ns'):
    values = [int(row[time_key]) for row in rows if int(row.get(time_key, 0))]
    return pd.DataFrame([{
        'previous_event_ns': previous,
        'event_ns': current,
        'dt_ms': (current - previous) * 1e-6,
    } for previous, current in zip(values, values[1:])])


def interval_distribution(stage, run_name, frame):
    values = frame.dt_ms.to_numpy(dtype=float) if len(frame) else np.array([])
    metrics = describe(values)
    return {
        'run': run_name,
        'stage': stage,
        **{f'{key}_ms': value for key, value in metrics.items() if key != 'count'},
        'interval_count': metrics['count'],
        'dt_lt_120_count': int(np.sum(values < 120.0)),
        'dt_120_180_count': int(np.sum(
            (values >= 120.0) & (values < 180.0))),
        'dt_180_250_count': int(np.sum(
            (values >= 180.0) & (values < 250.0))),
        'dt_ge_250_count': int(np.sum(values >= 250.0)),
        'dt_gt_50_count': int(np.sum(values > 50.0)),
        'dt_gt_100_count': int(np.sum(values > 100.0)),
        'dt_gt_200_count': int(np.sum(values > 200.0)),
    }


def read_probe(path: Path):
    if not path.is_file():
        return pd.DataFrame()
    frame = pd.read_csv(path)
    numeric = [
        column for column in frame.columns
        if column not in ('event', 'frame_id')
    ]
    for column in numeric:
        frame[column] = pd.to_numeric(frame[column], errors='coerce')
    return frame


def probe_stage_rows(frame):
    stages = defaultdict(list)
    if frame.empty:
        return stages
    for event, stage in PROBE_STAGE_EVENTS.items():
        selected = frame[frame.event.eq(event)]
        stages[stage] = [{
            'source_ns': int(row.source_ns),
            'event_ns': int(row.ros_ns),
            'steady_ns': int(row.steady_ns),
            'bag_ns': int(row.ros_ns),
        } for row in selected.itertuples() if row.source_ns > 0]
    for raw_stage, stage in PIPELINE_STAGE_NAMES.items():
        selected = frame[frame.event.eq(f'pipeline:{raw_stage}')]
        stages[stage] = [{
            'source_ns': int(row.source_ns),
            'event_ns': int(row.timing_end_ros_ns),
            'steady_ns': int(row.steady_ns),
            'bag_ns': int(row.timing_end_ros_ns),
            'queue_wait_ms': float(row.queue_wait_ms),
            'compute_ms': float(row.compute_ms),
            'received_count': int(row.received_count),
            'processed_count': int(row.processed_count),
            'replaced_count': int(row.replaced_count),
        } for row in selected.itertuples() if row.source_ns > 0]
    return stages


def rate_hz(rows, unique_source=False):
    values = unique_source_rows(rows) if unique_source else rows
    if len(values) < 2:
        return None
    duration = (values[-1]['bag_ns'] - values[0]['bag_ns']) * 1e-9
    return (len(values) - 1) / duration if duration > 0 else None


def group_episodes(rows, max_gap_ms=80.0):
    if not rows:
        return []
    episodes = [[rows[0]]]
    for row in rows[1:]:
        gap_ms = (row['bag_ns'] - episodes[-1][-1]['bag_ns']) * 1e-6
        if gap_ms <= max_gap_ms:
            episodes[-1].append(row)
        else:
            episodes.append([row])
    return episodes


def first_after(items, timestamp_ns):
    for item in items:
        if item['bag_ns'] >= timestamp_ns:
            return item
    return None


def latency_metrics(rows, motor_topic):
    states = unique_source_rows(rows['/lane_control_state_stamped'])
    commands = unique_source_rows(rows['/lane_motor_cmd_stamped'])
    motors = rows[motor_topic] if motor_topic else []
    source_to_state = [
        (row['bag_ns'] - row['source_ns']) * 1e-6 for row in states]
    source_to_command = [
        (row['bag_ns'] - row['source_ns']) * 1e-6 for row in commands]
    source_to_motor = []
    for command in commands:
        motor = first_after(motors, command['bag_ns'])
        if motor is not None and motor['bag_ns'] - command['bag_ns'] <= 75_000_000:
            source_to_motor.append(
                (motor['bag_ns'] - command['source_ns']) * 1e-6)
    return {
        'source_to_lane_state_ms': describe(source_to_state),
        'source_to_first_lane_command_ms': describe(source_to_command),
        'source_to_first_final_motor_ms': describe(source_to_motor),
    }


def nearest_resource(run_dir: Path, event_ns: int):
    path = run_dir / 'resource_stats.csv'
    if not path.is_file():
        return {}
    frame = pd.read_csv(path)
    if frame.empty or 'timestamp_ns' not in frame:
        return {}
    times = pd.to_numeric(frame.timestamp_ns, errors='coerce')
    valid = times.notna()
    if not valid.any():
        return {}
    selected = frame.loc[(times[valid] - event_ns).abs().idxmin()]
    fields = (
        'timestamp_ns', 'process_cpu_pct_one_core_100', 'total_cpu_pct',
        'busiest_core_cpu_pct', 'cpu_frequency_avg_mhz', 'load1',
        'process_thread_count', 'process_involuntary_ctxt_switches_s',
        'memory_available_mib', 'memory_used_pct', 'swap_used_mib',
        'disk_read_mib_s', 'disk_write_mib_s', 'disk_busy_pct',
        'sample_collect_ms',
    )
    result = {}
    for field in fields:
        value = selected.get(field, math.nan)
        if pd.isna(value):
            result[field] = None
        elif hasattr(value, 'item'):
            result[field] = value.item()
        else:
            result[field] = value
    return result


def resource_run_summary(run_name, kind, run_dir: Path):
    """Summarize a profiler run without requiring the profiler to be on."""
    path = run_dir / 'resource_stats.csv'
    if not path.is_file():
        return None, []
    frame = pd.read_csv(path)
    if frame.empty:
        return None, []

    metric_columns = (
        'process_cpu_pct_one_core_100', 'process_cpu_pct_total_capacity',
        'process_rss_mib', 'process_thread_count', 'total_cpu_pct',
        'busiest_core_cpu_pct', 'cpu_frequency_avg_mhz',
        'cpu_frequency_min_mhz', 'cpu_frequency_max_mhz', 'load1',
        'memory_available_mib', 'memory_used_pct', 'swap_used_mib',
        'disk_read_mib_s', 'disk_write_mib_s', 'disk_busy_pct',
        'sample_collect_ms', 'previous_callback_wall_ms',
    )
    row = {
        'run': run_name,
        'kind': kind,
        'sample_count': len(frame),
    }
    for column in metric_columns:
        values = (
            pd.to_numeric(frame[column], errors='coerce')
            if column in frame else pd.Series(dtype=float))
        stats = describe(values)
        for statistic in ('median', 'p95', 'max'):
            row[f'{column}_{statistic}'] = stats[statistic]
    interval = pd.to_numeric(
        frame.get('sample_interval_ms', pd.Series(dtype=float)),
        errors='coerce')
    callback = pd.to_numeric(
        frame.get('sample_collect_ms', pd.Series(dtype=float)),
        errors='coerce')
    valid = interval.notna() & callback.notna() & interval.gt(0.0)
    row['profiler_callback_duty_pct_median'] = (
        float(np.median(callback[valid] / interval[valid] * 100.0))
        if valid.any() else None)
    row['profiler_callback_duty_pct_p95'] = (
        float(np.percentile(callback[valid] / interval[valid] * 100.0, 95))
        if valid.any() else None)

    process_rows = []
    process_path = run_dir / 'resource_stats_processes.csv'
    if process_path.is_file():
        processes = pd.read_csv(process_path)
        if not processes.empty and 'role' in processes:
            for role, selected in processes.groupby('role'):
                cpu = pd.to_numeric(
                    selected.get('cpu_pct_one_core_100'), errors='coerce')
                threads = pd.to_numeric(
                    selected.get('thread_count'), errors='coerce')
                cpu_stats = describe(cpu)
                thread_stats = describe(threads)
                process_rows.append({
                    'run': run_name,
                    'kind': kind,
                    'role': role,
                    'sample_count': len(selected),
                    'cpu_pct_median': cpu_stats['median'],
                    'cpu_pct_p95': cpu_stats['p95'],
                    'cpu_pct_max': cpu_stats['max'],
                    'thread_count_median': thread_stats['median'],
                    'thread_count_max': thread_stats['max'],
                })
    return row, process_rows


def max_gap_near(stage_rows, event_ns, window_ns=1_000_000_000):
    unique = unique_source_rows(stage_rows)
    candidates = []
    for previous, current in zip(unique, unique[1:]):
        current_event = int(current.get('event_ns', current.get('bag_ns', 0)))
        if abs(current_event - event_ns) <= window_ns:
            candidates.append({
                'gap_ms': (current['source_ns'] - previous['source_ns']) * 1e-6,
                'previous_source_ns': previous['source_ns'],
                'source_ns': current['source_ns'],
            })
    return max(candidates, key=lambda item: item['gap_ms']) if candidates else {}


def max_event_gap_near(rows, event_ns, window_ns=1_000_000_000):
    frame = event_interval_frame(rows)
    if frame.empty:
        return None
    nearby = frame[(frame.event_ns - event_ns).abs() <= window_ns]
    return float(nearby.dt_ms.max()) if not nearby.empty else None


def max_pipeline_value_near(
        timing_rows, stage, field, event_ns, window_ns=1_000_000_000):
    values = [
        float(row[field]) for row in timing_rows
        if row.get('stage') == stage
        and abs(int(row.get('end_ros_ns', row.get('bag_ns', 0))) - event_ns)
        <= window_ns
        and math.isfinite(float(row.get(field, math.nan)))
    ]
    return max(values) if values else None


def classify_first_stage(gaps, raw_steady_gap_ms=None):
    if gaps.get('image_raw', {}).get('gap_ms', 0.0) > 100.0:
        return 'image_raw source'
    if raw_steady_gap_ms is not None and raw_steady_gap_ms > 100.0:
        return 'rosbag/raw callback delivery scheduling'
    ordered = (
        ('frame_router_selected', 'frame router selection/output'),
        ('yolo_detection', 'YOLO worker/output'),
        ('center_curve', 'center_curve'),
        ('lane_state', 'Lane_Detector/state'),
    )
    for key, label in ordered:
        if gaps.get(key, {}).get('gap_ms', 0.0) >= 180.0:
            return label
    return 'not proven from available stage timestamps'


def build_run_summary(run_name, run_dir, kind, input_raw_rows=None):
    bag_path = run_dir / 'replay_output_bag'
    rows, motor_topic, _ = read_bag(bag_path)
    probe = read_probe(run_dir / 'freshness_probe.csv')
    stages = defaultdict(list)
    for topic, stage in SOURCE_STAGE_TOPICS.items():
        stages[stage] = rows[topic]
    if kind == 'offline' and input_raw_rows:
        # The replay consumes this exact raw stream.  Its source timestamps are
        # therefore the upstream stage even when the lightweight output bag
        # intentionally omits 4 GiB of image payloads.
        stages['image_raw'] = input_raw_rows
    probe_stages = probe_stage_rows(probe)
    for stage, values in probe_stages.items():
        if values:
            stages[stage] = values

    commands = rows['/lane_motor_cmd_stamped']
    debug = rows['/stanley/debug']
    if debug:
        start_ns, end_ns = debug[0]['bag_ns'], debug[-1]['bag_ns']
        operational = [
            row for row in commands if start_ns <= row['bag_ns'] <= end_ns]
    else:
        operational = commands
    stale_rows = [
        row for row in operational
        if row.get('state_age_s', -1.0) >= 0.30
        and abs(row.get('speed', math.nan)) <= 1e-9
    ]
    episodes = group_episodes(stale_rows)
    ages = [row.get('state_age_s', math.nan) for row in operational]
    interval_frames = {
        stage: source_intervals(values) for stage, values in stages.items()
    }
    motor_rows = rows[motor_topic] if motor_topic else []
    # /stanley/debug is not published by the early stale-stop branch.  The
    # stamped lane command is therefore the complete event+heartbeat control
    # output and is the correct stream for detecting a scheduler gap.
    stages['stanley_command'] = commands
    stages['final_motor'] = motor_rows
    interval_frames['stanley_command'] = event_interval_frame(commands)
    interval_frames['final_motor'] = event_interval_frame(motor_rows)

    episode_rows = []
    for index, episode in enumerate(episodes, start=1):
        timeout_ns = int(episode[0]['bag_ns'])
        gaps = {
            stage: max_gap_near(values, timeout_ns)
            for stage, values in stages.items()
            if stage in (
                'image_raw', 'frame_router_selected', 'yolo_detection',
                'center_curve', 'lane_state')
        }
        raw_steady_gap_ms = None
        raw_rows = stages.get('image_raw_observed', [])
        raw_near = [
            row for row in raw_rows
            if abs(int(row.get('event_ns', row.get('bag_ns', 0))) - timeout_ns)
            <= 1_000_000_000
        ]
        steady_values = [
            int(row.get('steady_ns', 0)) for row in raw_near
            if int(row.get('steady_ns', 0)) > 0]
        if len(steady_values) > 1:
            raw_steady_gap_ms = float(
                np.max(np.diff(np.asarray(steady_values))) * 1e-6)
        resource = nearest_resource(run_dir, timeout_ns)
        timing_rows = rows['/pipeline_timing']
        episode_rows.append({
            'run': run_name,
            'kind': kind,
            'episode': index,
            'timeout_ns': timeout_ns,
            'timeout_source_ns': int(episode[0]['source_ns']),
            'operational_elapsed_s': (
                (timeout_ns - start_ns) * 1e-9 if debug else None),
            'startup_transient': bool(
                debug and timeout_ns - start_ns < 1_000_000_000),
            'max_state_age_s': max(row['state_age_s'] for row in episode),
            'stale_command_count': len(episode),
            'first_abnormal_stage': classify_first_stage(
                gaps, raw_steady_gap_ms),
            'raw_observed_steady_gap_ms': raw_steady_gap_ms,
            **{
                f'{stage}_source_gap_ms': gaps.get(stage, {}).get('gap_ms')
                for stage in (
                    'image_raw', 'frame_router_selected', 'yolo_detection',
                    'center_curve', 'lane_state')
            },
            'yolo_queue_wait_max_ms': max_pipeline_value_near(
                timing_rows, 'lane_yolo', 'queue_wait_ms', timeout_ns),
            'yolo_worker_max_ms': max_pipeline_value_near(
                timing_rows, 'lane_yolo', 'compute_ms', timeout_ns),
            'yolo_inference_max_ms': max_pipeline_value_near(
                timing_rows, 'lane_yolo_inference', 'compute_ms', timeout_ns),
            'lane_detector_max_ms': max_pipeline_value_near(
                timing_rows, 'lane_detector_hough', 'compute_ms', timeout_ns),
            'stanley_command_interval_max_ms': max_event_gap_near(
                commands, timeout_ns),
            'mission_motor_interval_max_ms': max_event_gap_near(
                motor_rows, timeout_ns),
            'resource_json': json.dumps(resource, ensure_ascii=False),
        })

    motor_intervals = event_intervals(motor_rows)
    stage_profiles = defaultdict(list)
    for row in rows['/pipeline_timing']:
        stage_profiles[row['stage']].append(row)
    profile_summary = {}
    for stage, values in stage_profiles.items():
        profile_summary[stage] = {
            'compute_ms': describe(row['compute_ms'] for row in values),
            'queue_wait_ms': describe(row['queue_wait_ms'] for row in values),
            'last_counters': {
                key: values[-1][key] for key in (
                    'received_count', 'processed_count', 'replaced_count')
            },
        }

    unique_states = unique_source_rows(rows['/lane_control_state_stamped'])
    state_duration_s = (
        (unique_states[-1]['bag_ns'] - unique_states[0]['bag_ns']) * 1e-9
        if len(unique_states) > 1 else 0.0)
    summary = {
        'run': run_name,
        'kind': kind,
        'run_dir': str(run_dir),
        'motor_topic': motor_topic,
        'profile_enabled': not probe.empty,
        'counts': {topic: len(values) for topic, values in rows.items()},
        'cadence_hz': {
            'yolo': rate_hz(rows['/lane_yolo/detections'], True),
            'center_curve': rate_hz(rows['/center_curve'], True),
            'lane_state': rate_hz(rows['/lane_control_state_stamped'], True),
            'lane_command': rate_hz(rows['/lane_motor_cmd_stamped']),
            'final_motor': rate_hz(motor_rows),
        },
        'duration_s': state_duration_s,
        'state_age_s': describe(ages),
        'state_age_ge_0_30_count': int(sum(age >= 0.30 for age in ages)),
        'freshness_zero_command_count': len(stale_rows),
        'freshness_episode_count': len(episodes),
        'steady_freshness_episode_count': int(sum(
            not item['startup_transient'] for item in episode_rows)),
        'steady_freshness_zero_command_count': int(sum(
            item['stale_command_count'] for item in episode_rows
            if not item['startup_transient'])),
        'max_processed_source_gap_ms': (
            float(interval_frames['lane_state'].dt_ms.max())
            if len(interval_frames['lane_state']) else None),
        'processed_source_gap_gt_200_count': (
            int(interval_frames['lane_state'].dt_ms.gt(200.0).sum())
            if len(interval_frames['lane_state']) else 0),
        'final_motor_interval_ms': describe(motor_intervals),
        'latency': latency_metrics(rows, motor_topic),
        'pipeline_timing': profile_summary,
        'resource_profile': resource_run_summary(
            run_name, kind, run_dir)[0],
        'episodes': episode_rows,
    }
    return summary, stages, interval_frames, operational, episode_rows, probe


def raw_bag_summary(path: Path):
    rows, _, _ = read_bag(path, raw_only=True)
    images = rows['/image_raw']
    intervals = source_intervals(images)
    values = intervals.dt_ms.to_numpy(dtype=float)
    return {
        'bag': str(path),
        'image_count': len(images),
        'source_integrity': {
            'unique': len(unique_source_rows(images)),
            'duplicates': len(images) - len(unique_source_rows(images)),
            'out_of_order': int(sum(
                current['source_ns'] < previous['source_ns']
                for previous, current in zip(images, images[1:]))),
        },
        'source_interval_ms': describe(values),
        'dt_gt_50_count': int(np.sum(values > 50.0)),
        'dt_gt_100_count': int(np.sum(values > 100.0)),
        'dt_gt_200_count': int(np.sum(values > 200.0)),
        'record_interval_ms': describe(event_intervals(images)),
    }, intervals, images


def discover_runs(root: Path, category):
    parent = root / category
    if not parent.is_dir():
        return []
    return [
        (path.name, path) for path in sorted(parent.iterdir())
        if (path / 'replay_output_bag' / 'metadata.yaml').is_file()
    ]


def plot_state_age(root, run_data):
    figure, axis = plt.subplots(figsize=(12, 5))
    for name, data in run_data.items():
        operational = data['operational']
        if not operational:
            continue
        origin = operational[0]['bag_ns']
        axis.plot(
            [(row['bag_ns'] - origin) * 1e-9 for row in operational],
            [row.get('state_age_s', math.nan) for row in operational],
            linewidth=0.8, alpha=0.8, label=name,
        )
    axis.axhline(0.30, color='red', linestyle='--', label='0.30 s timeout')
    axis.set_xlabel('run-relative time (s)')
    axis.set_ylabel('LaneControlState source age (s)')
    axis.grid(alpha=0.25)
    axis.legend(fontsize=7, ncol=2)
    figure.tight_layout()
    figure.savefig(root / 'state_age_timeline.png', dpi=150)
    plt.close(figure)


def plot_processed_gap(root, run_data):
    figure, axis = plt.subplots(figsize=(12, 5))
    for name, data in run_data.items():
        frame = data['intervals'].get('lane_state', pd.DataFrame())
        if frame.empty:
            continue
        origin = int(frame.event_ns.iloc[0])
        axis.plot(
            (frame.event_ns.to_numpy(dtype=np.int64) - origin) * 1e-9,
            frame.dt_ms.to_numpy(dtype=float),
            linewidth=0.8, alpha=0.8, label=name,
        )
    axis.axhline(200.0, color='red', linestyle='--', label='200 ms')
    axis.set_xlabel('run-relative time (s)')
    axis.set_ylabel('processed source interval (ms)')
    axis.grid(alpha=0.25)
    axis.legend(fontsize=7, ncol=2)
    figure.tight_layout()
    figure.savefig(root / 'processed_source_gap.png', dpi=150)
    plt.close(figure)


def plot_stage_comparison(root, stage_stats):
    frame = pd.DataFrame(stage_stats)
    if frame.empty:
        return
    selected = frame[
        frame.stage.isin((
            'image_raw', 'frame_router_selected', 'yolo_detection',
            'center_curve', 'lane_state', 'stanley_command', 'final_motor'))
    ]
    if selected.empty:
        return
    labels = [f'{row.run}\n{row.stage}' for row in selected.itertuples()]
    x = np.arange(len(selected))
    figure, axis = plt.subplots(figsize=(max(12, len(selected) * 0.45), 6))
    axis.bar(x - 0.18, selected.p95_ms, width=0.36, label='p95')
    axis.bar(x + 0.18, selected.max_ms, width=0.36, label='max')
    axis.axhline(200.0, color='red', linestyle='--', linewidth=1)
    axis.set_xticks(x)
    axis.set_xticklabels(labels, rotation=75, ha='right', fontsize=7)
    axis.set_ylabel('source/event interval (ms)')
    axis.legend()
    axis.grid(axis='y', alpha=0.25)
    figure.tight_layout()
    figure.savefig(root / 'stage_interval_comparison.png', dpi=150)
    plt.close(figure)


def plot_episode(root, run_name, episode, stages):
    timeout_ns = int(episode['timeout_ns'])
    stage_order = (
        'image_raw', 'frame_router_selected', 'yolo_detection',
        'center_curve', 'lane_state', 'stanley_command', 'final_motor')
    figure, axis = plt.subplots(figsize=(12, 4.8))
    ylabels = []
    for y, stage in enumerate(stage_order):
        ylabels.append(stage)
        values = stages.get(stage, [])
        points = [
            row for row in values
            if abs(int(row.get('event_ns', row.get('bag_ns', 0))) - timeout_ns)
            <= 1_000_000_000
        ]
        axis.scatter(
            [
                (int(row.get('event_ns', row.get('bag_ns', 0))) - timeout_ns)
                * 1e-9 for row in points
            ],
            [y] * len(points), s=10,
        )
    axis.axvline(0.0, color='red', linestyle='--', label='timeout start')
    axis.set_yticks(range(len(stage_order)))
    axis.set_yticklabels(ylabels)
    axis.set_xlim(-1.0, 1.0)
    axis.set_xlabel('time relative to timeout (s), ROS/sim clock')
    axis.set_title(
        f'{run_name} event {episode["episode"]}: '
        f'{episode["first_abnormal_stage"]}')
    axis.grid(axis='x', alpha=0.25)
    figure.tight_layout()
    safe_name = run_name.replace('/', '_')
    figure.savefig(
        root / f'timeout_event_{safe_name}_{episode["episode"]}.png',
        dpi=150,
    )
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--root', type=Path,
        default=Path('/home/xytron/Downloads/analysis/freshness_root_cause'))
    parser.add_argument(
        '--input-bag', type=Path,
        default=Path(
            '/home/xytron/Downloads/oscillation_20260806_104536_replay'))
    parser.add_argument(
        '--include-run', action='append', default=[],
        help='Additional NAME=RUN_DIR entry, such as the prior clean replay.')
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)

    raw_summary, raw_intervals, input_raw_rows = raw_bag_summary(
        args.input_bag.expanduser().resolve())
    raw_intervals.to_csv(
        root / 'original_image_raw_intervals.csv', index=False,
        float_format='%.6f')
    runs = []
    runs.extend((name, path, 'offline') for name, path in discover_runs(
        root, 'offline_runs'))
    runs.extend((name, path, 'live_usb') for name, path in discover_runs(
        root, 'live_runs'))
    for value in args.include_run:
        name, raw_path = value.split('=', 1)
        runs.append((name, Path(raw_path).expanduser().resolve(), 'offline'))

    run_summaries = []
    run_data = {}
    stage_stats = []
    event_rows = []
    waterfall_rows = []
    resource_rows = []
    resource_process_rows = []
    for name, run_dir, kind in runs:
        summary, stages, intervals, operational, episodes, probe = (
            build_run_summary(
                name, run_dir, kind,
                input_raw_rows=input_raw_rows if kind == 'offline' else None,
            ))
        run_summaries.append(summary)
        resource_row, process_rows = resource_run_summary(
            name, kind, run_dir)
        if resource_row is not None:
            resource_rows.append(resource_row)
            resource_process_rows.extend(process_rows)
        run_data[name] = {
            'stages': stages,
            'intervals': intervals,
            'operational': operational,
            'probe': probe,
        }
        for stage, frame in intervals.items():
            stage_stats.append(interval_distribution(stage, name, frame))
        event_rows.extend(episodes)
        for episode in episodes:
            plot_episode(root, name, episode, stages)
            timeout_ns = int(episode['timeout_ns'])
            for stage, values in stages.items():
                previous_source = None
                previous_steady = None
                for row in unique_source_rows(values):
                    event_ns = int(row.get(
                        'event_ns', row.get('bag_ns', 0)))
                    source_ns = int(row.get('source_ns', 0))
                    steady_ns = int(row.get('steady_ns', 0))
                    source_dt_ms = (
                        (source_ns - previous_source) * 1e-6
                        if previous_source is not None else None)
                    steady_dt_ms = (
                        (steady_ns - previous_steady) * 1e-6
                        if previous_steady is not None and steady_ns > 0
                        else None)
                    if abs(event_ns - timeout_ns) <= 1_000_000_000:
                        waterfall_rows.append({
                            'run': name,
                            'episode': episode['episode'],
                            'stage': stage,
                            'event_relative_ms': (
                                event_ns - timeout_ns) * 1e-6,
                            'event_ros_ns': event_ns,
                            'steady_ns': steady_ns or None,
                            'source_ns': source_ns,
                            'source_age_at_event_ms': (
                                event_ns - source_ns) * 1e-6,
                            'source_dt_ms': source_dt_ms,
                            'steady_dt_ms': steady_dt_ms,
                            'queue_wait_ms': row.get('queue_wait_ms'),
                            'compute_ms': row.get('compute_ms'),
                            'received_count': row.get('received_count'),
                            'processed_count': row.get('processed_count'),
                            'replaced_count': row.get('replaced_count'),
                        })
                    previous_source = source_ns
                    if steady_ns > 0:
                        previous_steady = steady_ns

    raw_row = interval_distribution('image_raw_original', 'original_bag', raw_intervals)
    stage_stats.insert(0, raw_row)
    pd.DataFrame(stage_stats).to_csv(
        root / 'stage_gap_statistics.csv', index=False, float_format='%.6f')
    pd.DataFrame(event_rows).to_csv(
        root / 'freshness_event_timeline.csv', index=False, float_format='%.9f')
    pd.DataFrame(waterfall_rows).to_csv(
        root / 'freshness_event_waterfall.csv', index=False,
        float_format='%.9f')
    pd.DataFrame(resource_rows).to_csv(
        root / 'resource_run_summary.csv', index=False,
        float_format='%.6f')
    pd.DataFrame(resource_process_rows).to_csv(
        root / 'resource_process_summary.csv', index=False,
        float_format='%.6f')

    comparison_rows = []
    for summary in run_summaries:
        comparison_rows.append({
            'run': summary['run'],
            'kind': summary['kind'],
            'profile_enabled': summary['profile_enabled'],
            'duration_s': summary['duration_s'],
            'max_state_age_s': summary['state_age_s']['max'],
            'p95_state_age_s': summary['state_age_s']['p95'],
            'p99_state_age_s': summary['state_age_s']['p99'],
            'state_age_ge_0_30_count': summary['state_age_ge_0_30_count'],
            'freshness_zero_command_count': summary[
                'freshness_zero_command_count'],
            'freshness_episode_count': summary['freshness_episode_count'],
            'steady_freshness_episode_count': summary[
                'steady_freshness_episode_count'],
            'steady_freshness_zero_command_count': summary[
                'steady_freshness_zero_command_count'],
            'max_processed_source_gap_ms': summary[
                'max_processed_source_gap_ms'],
            'processed_source_gap_gt_200_count': summary[
                'processed_source_gap_gt_200_count'],
            'yolo_hz': summary['cadence_hz']['yolo'],
            'lane_state_hz': summary['cadence_hz']['lane_state'],
            'final_motor_hz': summary['cadence_hz']['final_motor'],
            'source_to_lane_median_ms': summary['latency'][
                'source_to_lane_state_ms']['median'],
            'source_to_lane_p95_ms': summary['latency'][
                'source_to_lane_state_ms']['p95'],
            'source_to_motor_median_ms': summary['latency'][
                'source_to_first_final_motor_ms']['median'],
            'source_to_motor_p95_ms': summary['latency'][
                'source_to_first_final_motor_ms']['p95'],
        })
    comparison = pd.DataFrame(comparison_rows)
    comparison[comparison.kind.eq('offline')].to_csv(
        root / 'offline_run_comparison.csv', index=False,
        float_format='%.6f')
    comparison[comparison.kind.eq('live_usb')].to_csv(
        root / 'live_usb_run_comparison.csv', index=False,
        float_format='%.6f')

    plot_state_age(root, run_data)
    plot_processed_gap(root, run_data)
    plot_stage_comparison(root, stage_stats)
    report = {
        'original_image_raw': raw_summary,
        'runs': run_summaries,
    }
    (root / 'freshness_analysis.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
        encoding='utf-8')
    print(json.dumps({
        'output': str(root),
        'original_image_raw': raw_summary,
        'run_count': len(run_summaries),
        'event_count': len(event_rows),
    }, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
