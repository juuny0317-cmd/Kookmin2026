#!/usr/bin/env python3
"""Join YOLO timing events to CPU samples and summarize contention evidence."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from bisect import bisect_left
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


RESOURCE_FIELDS = (
    'process_cpu_pct_one_core_100', 'process_cpu_pct_total_capacity',
    'process_rss_mib', 'process_thread_count',
    'process_voluntary_ctxt_switches_s',
    'process_involuntary_ctxt_switches_s', 'process_minor_faults_s',
    'process_major_faults_s', 'process_read_mib_s', 'process_write_mib_s',
    'total_cpu_pct', 'busiest_core_cpu_pct', 'cpu_frequency_avg_mhz',
    'cpu_frequency_min_mhz', 'cpu_frequency_max_mhz',
    'cpu_pressure_some_avg10', 'load1', 'load5', 'load15',
    'sample_collect_ms', 'previous_callback_wall_ms',
)


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def number(value):
    try:
        result = float(value)
        return result if np.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def integer(value):
    value = number(value)
    return int(value) if value is not None else None


def describe(values):
    data = np.asarray([value for value in values if value is not None])
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


def rank(values):
    order = np.argsort(values, kind='mergesort')
    ranks = np.empty(len(values), dtype=np.float64)
    ranks[order] = np.arange(len(values), dtype=np.float64)
    unique, inverse, counts = np.unique(
        values, return_inverse=True, return_counts=True)
    if len(unique) != len(values):
        for index, count in enumerate(counts):
            if count > 1:
                positions = np.flatnonzero(inverse == index)
                ranks[positions] = np.mean(ranks[positions])
    return ranks


def correlations(rows, left_field, right_field):
    pairs = [
        (row.get(left_field), row.get(right_field)) for row in rows
        if row.get(left_field) is not None
        and row.get(right_field) is not None
    ]
    if len(pairs) < 3:
        return {'count': len(pairs), 'pearson': None, 'spearman': None}
    left = np.asarray([item[0] for item in pairs], dtype=np.float64)
    right = np.asarray([item[1] for item in pairs], dtype=np.float64)
    if np.std(left) == 0.0 or np.std(right) == 0.0:
        return {'count': len(pairs), 'pearson': None, 'spearman': None}
    return {
        'count': len(pairs),
        'pearson': float(np.corrcoef(left, right)[0, 1]),
        'spearman': float(np.corrcoef(rank(left), rank(right))[0, 1]),
    }


def read_timing(path, topic, total_stage, model_stage, inference_stage):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if topic not in types:
        raise RuntimeError(f'{topic} is absent from {path}')
    message_type = get_message(types[topic])
    by_stage = defaultdict(dict)
    while reader.has_next():
        name, serialized, bag_ns = reader.read_next()
        if name != topic:
            continue
        message = deserialize_message(serialized, message_type)
        if message.stage not in (total_stage, model_stage, inference_stage):
            continue
        source = stamp_ns(message.header.stamp)
        by_stage[message.stage].setdefault(source, {
            'source_ns': source,
            'bag_record_ns': int(bag_ns),
            'start_ns': int(message.start_stamp_ns),
            'end_ns': int(message.end_stamp_ns),
            'compute_ms': float(message.compute_ms),
            'queue_wait_ms': float(message.queue_wait_ms),
        })
    common = set(by_stage[total_stage]) & set(by_stage[model_stage])
    rows = []
    for source in sorted(common):
        total = by_stage[total_stage][source]
        model = by_stage[model_stage][source]
        pure = by_stage[inference_stage].get(source)
        rows.append({
            'source_ns': source,
            'inference_start_ns': model['start_ns'],
            'inference_end_ns': model['end_ns'],
            'pure_inference_ms': (
                pure['compute_ms'] if pure is not None else None),
            'model_call_ms': model['compute_ms'],
            'total_worker_ms': total['compute_ms'],
            'queue_wait_ms': total['queue_wait_ms'],
        })
    return rows


def parse_json_map(value):
    try:
        parsed = json.loads(value or '{}')
        return {int(key): float(item) for key, item in parsed.items()}
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}


def read_resources(path):
    rows = []
    with path.open(newline='', encoding='utf-8') as stream:
        for item in csv.DictReader(stream):
            row = {
                'timestamp_ns': int(item['timestamp_ns']),
                'target_pid': integer(item.get('target_pid')),
                'target_affinity': item.get('target_affinity', ''),
                'per_core_cpu_pct': parse_json_map(
                    item.get('per_core_cpu_pct_json')),
                'per_core_frequency_mhz': parse_json_map(
                    item.get('per_core_frequency_mhz_json')),
                'resource_error': item.get('error', ''),
            }
            for field in RESOURCE_FIELDS:
                row[field] = number(item.get(field))
            rows.append(row)
    return sorted(rows, key=lambda item: item['timestamp_ns'])


def nearest(resources, timestamps, target):
    index = bisect_left(timestamps, target)
    candidates = []
    if index < len(resources):
        candidates.append(resources[index])
    if index > 0:
        candidates.append(resources[index - 1])
    return min(candidates, key=lambda item: abs(item['timestamp_ns'] - target))


def read_long_table(path, numeric_fields):
    if path is None or not path.exists():
        return []
    rows = []
    with path.open(newline='', encoding='utf-8') as stream:
        for item in csv.DictReader(stream):
            row = dict(item)
            row['timestamp_ns'] = integer(item.get('timestamp_ns'))
            row['sample_index'] = integer(item.get('sample_index'))
            for field in numeric_fields:
                row[field] = number(item.get(field))
            rows.append(row)
    return rows


def process_summary(rows):
    by_role = defaultdict(list)
    for row in rows:
        by_role[row.get('role', 'other')].append(row)
    fields = (
        'cpu_pct_one_core_100', 'cpu_pct_total_capacity', 'rss_mib',
        'thread_count', 'read_mib_s', 'write_mib_s',
    )
    return {
        role: {
            field: describe([item.get(field) for item in items])
            for field in fields
        }
        for role, items in sorted(by_role.items())
    }


def thread_summary(rows):
    if not rows:
        return {'samples': 0, 'thread_cpu_pct': describe([])}
    by_sample = defaultdict(list)
    for row in rows:
        by_sample[row['sample_index']].append(row)
    active_counts = []
    summed_cpu = []
    for items in by_sample.values():
        values = [
            item['cpu_pct_one_core_100'] for item in items
            if item.get('cpu_pct_one_core_100') is not None
        ]
        active_counts.append(sum(value >= 1.0 for value in values))
        if values:
            summed_cpu.append(sum(values))
    return {
        'samples': len(by_sample),
        'observed_thread_count': describe([
            len(items) for items in by_sample.values()]),
        'active_threads_ge_1pct': describe(active_counts),
        'summed_thread_cpu_pct_one_core_100': describe(summed_cpu),
        'individual_thread_cpu_pct_one_core_100': describe([
            row.get('cpu_pct_one_core_100') for row in rows]),
    }


def per_core_summary(resources, field):
    values = defaultdict(list)
    for row in resources:
        for core, value in row[field].items():
            values[core].append(value)
    return {
        str(core): describe(items) for core, items in sorted(values.items())
    }


def quartile_contrast(joined, fields):
    valid = [row for row in joined if row['pure_inference_ms'] is not None]
    if len(valid) < 8:
        return {'count': len(valid), 'fast': {}, 'slow': {}}
    ordered = sorted(valid, key=lambda row: row['pure_inference_ms'])
    width = max(1, len(ordered) // 4)
    fast = ordered[:width]
    slow = ordered[-width:]
    return {
        'count': len(valid),
        'quartile_size': width,
        'fast': {
            field: describe([row.get(field) for row in fast])
            for field in fields
        },
        'slow': {
            field: describe([row.get(field) for row in slow])
            for field in fields
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('--resources', type=Path, required=True)
    parser.add_argument('--processes', type=Path)
    parser.add_argument('--threads', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--timing-topic', default='/pipeline_timing')
    parser.add_argument('--total-stage', default='lane_yolo')
    parser.add_argument('--model-stage', default='lane_yolo_model')
    parser.add_argument('--inference-stage', default='lane_yolo_inference')
    args = parser.parse_args()

    resource_path = args.resources.expanduser().resolve()
    process_path = args.processes
    if process_path is None:
        process_path = resource_path.with_name(
            f'{resource_path.stem}_processes{resource_path.suffix}')
    thread_path = args.threads
    if thread_path is None:
        thread_path = resource_path.with_name(
            f'{resource_path.stem}_threads{resource_path.suffix}')

    timings = read_timing(
        args.bag.expanduser().resolve(), args.timing_topic,
        args.total_stage, args.model_stage, args.inference_stage)
    resources = read_resources(resource_path)
    if not resources:
        raise RuntimeError('Resource CSV contains no samples')
    if not timings:
        raise RuntimeError('No common YOLO total/model timing samples')

    timestamps = [item['timestamp_ns'] for item in resources]
    joined = []
    for timing in timings:
        resource = nearest(resources, timestamps, timing['inference_start_ns'])
        joined.append({
            **timing,
            **{field: resource.get(field) for field in RESOURCE_FIELDS},
            'target_pid': resource.get('target_pid'),
            'target_affinity': resource.get('target_affinity'),
            'resource_sample_ns': resource['timestamp_ns'],
            'resource_delta_ms': (
                abs(resource['timestamp_ns'] - timing['inference_start_ns'])
                * 1e-6),
            'resource_error': resource.get('resource_error', ''),
        })

    process_rows = read_long_table(process_path, (
        'cpu_pct_one_core_100', 'cpu_pct_total_capacity', 'rss_mib',
        'thread_count', 'read_mib_s', 'write_mib_s'))
    thread_rows = read_long_table(thread_path, (
        'cpu_pct_one_core_100', 'voluntary_ctxt_switches_s',
        'involuntary_ctxt_switches_s'))
    contrast_fields = (
        'process_cpu_pct_one_core_100', 'total_cpu_pct',
        'busiest_core_cpu_pct', 'cpu_frequency_avg_mhz',
        'cpu_frequency_min_mhz', 'cpu_pressure_some_avg10',
        'process_involuntary_ctxt_switches_s', 'queue_wait_ms',
    )
    report = {
        'timing_samples': len(timings),
        'resource_samples': len(resources),
        'joined_samples': len(joined),
        'resource_delta_ms': describe([
            row['resource_delta_ms'] for row in joined]),
        'pure_inference_ms': describe([
            row['pure_inference_ms'] for row in joined]),
        'model_call_ms': describe([
            row['model_call_ms'] for row in joined]),
        'total_worker_ms': describe([
            row['total_worker_ms'] for row in joined]),
        'queue_wait_ms': describe([
            row['queue_wait_ms'] for row in joined]),
        'resource_metrics': {
            field: describe([row.get(field) for row in joined])
            for field in RESOURCE_FIELDS
        },
        'per_core_cpu_pct': per_core_summary(
            resources, 'per_core_cpu_pct'),
        'per_core_frequency_mhz': per_core_summary(
            resources, 'per_core_frequency_mhz'),
        'pure_inference_correlations': {
            field: correlations(joined, 'pure_inference_ms', field)
            for field in contrast_fields
        },
        'fast_vs_slow_inference_quartiles': quartile_contrast(
            joined, contrast_fields),
        'process_table': process_summary(process_rows),
        'target_thread_table': thread_summary(thread_rows),
        'alignment_note': (
            'Resource samples are interval averages joined to the nearest '
            'inference start. Treat correlation as evidence, not causation; '
            'repeat at 5 Hz if nearest-sample p95 is too wide.'),
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / 'yolo_resource_aligned.csv'
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(joined[0]))
        writer.writeheader()
        writer.writerows(joined)
    json_path = args.output_dir / 'yolo_resource_metrics.json'
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f'wrote {csv_path}')
    print(f'wrote {json_path}')


if __name__ == '__main__':
    main()
