#!/usr/bin/env python3
"""Source-align two replay bags and compare detection/lane/control outputs."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = (
    '/lane_yolo/detections',
    '/center_curve',
    '/lane_control_state_stamped',
    '/lane_motor_cmd_stamped',
)
SOURCE_IMAGE_TOPIC = '/perception/lane/image'


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def describe(values):
    data = np.asarray(values, dtype=np.float64)
    data = data[np.isfinite(data)]
    if not data.size:
        return {'count': 0, 'median': None, 'p95': None, 'p99': None,
                'max': None}
    return {
        'count': int(data.size),
        'median': float(np.percentile(data, 50)),
        'p95': float(np.percentile(data, 95)),
        'p99': float(np.percentile(data, 99)),
        'max': float(np.max(data)),
    }


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required_topics = (*TOPICS, SOURCE_IMAGE_TOPIC)
    missing = [topic for topic in required_topics if topic not in types]
    if missing:
        raise RuntimeError(f'{path} is missing topics {missing}')
    message_types = {
        topic: get_message(types[topic]) for topic in required_topics}
    output_messages = defaultdict(list)
    source_images = {}
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        source = stamp_ns(message.header.stamp)
        if source <= 0:
            continue
        if topic == SOURCE_IMAGE_TOPIC:
            digest = hashlib.sha256()
            digest.update(str(message.width).encode())
            digest.update(str(message.height).encode())
            digest.update(str(message.step).encode())
            digest.update(str(message.encoding).encode())
            digest.update(bytes(message.data))
            source_images.setdefault(source, digest.hexdigest())
        else:
            output_messages[topic].append((source, message))

    # Replay-generated ROS stamps are scheduler dependent.  Bind each output
    # stamp to the actual routed frame bytes, then use the image digest and its
    # occurrence number as the source identity shared by independent runs.
    digest_occurrences = defaultdict(int)
    source_keys = {}
    for source, digest in sorted(source_images.items()):
        occurrence = digest_occurrences[digest]
        digest_occurrences[digest] += 1
        source_keys[source] = f'{digest}:{occurrence}'
    result = defaultdict(dict)
    for topic, items in output_messages.items():
        for source, message in items:
            key = source_keys.get(source)
            if key is not None and key not in result[topic]:
                result[topic][key] = message
    return result


def compare(reference, candidate):
    report = {}
    checks = {}
    for topic in TOPICS:
        left = reference[topic]
        right = candidate[topic]
        common = sorted(set(left) & set(right))
        report.setdefault('coverage', {})[topic] = {
            'reference_unique': len(left),
            'candidate_unique': len(right),
            'common': len(common),
            'common_over_reference_pct': (
                len(common) / max(1, len(left)) * 100.0),
        }

    detection_count_mismatch = 0
    detection_class_mismatch = 0
    confidence_diffs = []
    box_diffs = []
    for source in sorted(
            set(reference[TOPICS[0]]) & set(candidate[TOPICS[0]])):
        left = list(reference[TOPICS[0]][source].detections)
        right = list(candidate[TOPICS[0]][source].detections)
        if len(left) != len(right):
            detection_count_mismatch += 1
        for l_item, r_item in zip(left, right):
            if str(l_item.class_name) != str(r_item.class_name):
                detection_class_mismatch += 1
            confidence_diffs.append(abs(
                float(l_item.confidence) - float(r_item.confidence)))
            box_diffs.extend([
                abs(int(l_item.xmin) - int(r_item.xmin)),
                abs(int(l_item.ymin) - int(r_item.ymin)),
                abs(int(l_item.xmax) - int(r_item.xmax)),
                abs(int(l_item.ymax) - int(r_item.ymax)),
            ])
    report['detections'] = {
        'count_mismatch_frames': detection_count_mismatch,
        'ordered_class_mismatches': detection_class_mismatch,
        'confidence_abs_difference': describe(confidence_diffs),
        'bbox_coordinate_abs_difference_px': describe(box_diffs),
    }

    curve_count_mismatch = 0
    curve_state_mismatch = 0
    curve_point_distances = []
    for source in sorted(
            set(reference[TOPICS[1]]) & set(candidate[TOPICS[1]])):
        left = reference[TOPICS[1]][source]
        right = candidate[TOPICS[1]][source]
        if str(left.state) != str(right.state):
            curve_state_mismatch += 1
        if len(left.points) != len(right.points):
            curve_count_mismatch += 1
            continue
        for l_point, r_point in zip(left.points, right.points):
            curve_point_distances.append(float(np.hypot(
                float(l_point.x) - float(r_point.x),
                float(l_point.y) - float(r_point.y))))
    report['center_curve'] = {
        'point_count_mismatch_frames': curve_count_mismatch,
        'state_mismatch_frames': curve_state_mismatch,
        'point_distance_px': describe(curve_point_distances),
    }

    state_diffs = defaultdict(list)
    state_mismatches = defaultdict(int)
    for source in sorted(
            set(reference[TOPICS[2]]) & set(candidate[TOPICS[2]])):
        left = reference[TOPICS[2]][source]
        right = candidate[TOPICS[2]][source]
        for field in ('drive_mode',):
            if str(getattr(left.state, field)) != str(
                    getattr(right.state, field)):
                state_mismatches[field] += 1
        for field in (
                'curve_state', 'heading_source', 'curve_heading_valid'):
            if getattr(left, field) != getattr(right, field):
                state_mismatches[field] += 1
        for axis in ('x', 'y', 'z'):
            state_diffs[f'target_{axis}'].append(abs(
                float(getattr(left.state.target_point, axis))
                - float(getattr(right.state.target_point, axis))))
        for field in (
                'curvature', 'curvature_severity', 'heading_preview_ratio',
                'curve_fit_error_ratio', 'path_confidence'):
            state_diffs[field].append(abs(
                float(getattr(left, field)) - float(getattr(right, field))))
    report['lane_control_state'] = {
        'categorical_mismatches': dict(state_mismatches),
        'numeric_abs_differences': {
            key: describe(values) for key, values in state_diffs.items()
        },
    }

    angle_diffs = []
    speed_diffs = []
    for source in sorted(
            set(reference[TOPICS[3]]) & set(candidate[TOPICS[3]])):
        left = reference[TOPICS[3]][source]
        right = candidate[TOPICS[3]][source]
        angle_diffs.append(abs(float(left.angle) - float(right.angle)))
        speed_diffs.append(abs(float(left.speed) - float(right.speed)))
    report['first_lane_command'] = {
        'angle_abs_difference': describe(angle_diffs),
        'speed_abs_difference': describe(speed_diffs),
    }

    min_coverage = min(
        item['common_over_reference_pct']
        for item in report['coverage'].values())
    confidence_max = report['detections'][
        'confidence_abs_difference']['max']
    bbox_max = report['detections'][
        'bbox_coordinate_abs_difference_px']['max']
    checks = {
        'source_coverage_ge_95pct': min_coverage >= 95.0,
        'detection_count_exact': detection_count_mismatch == 0,
        'detection_ordered_class_exact': detection_class_mismatch == 0,
        'detection_bbox_exact': bbox_max in (None, 0.0),
        'detection_confidence_exact': confidence_max in (None, 0.0),
        'nms_output_exact': (
            detection_count_mismatch == 0
            and detection_class_mismatch == 0
            and bbox_max in (None, 0.0)
            and confidence_max in (None, 0.0)),
        'curve_point_count_exact': curve_count_mismatch == 0,
        'curve_state_exact': curve_state_mismatch == 0,
        'curve_point_p95_le_1px': (
            bool(curve_point_distances)
            and bool(np.percentile(curve_point_distances, 95) <= 1.0)),
        'state_categorical_exact': sum(state_mismatches.values()) == 0,
        'target_x_p95_le_1px': (
            bool(state_diffs['target_x'])
            and bool(np.percentile(state_diffs['target_x'], 95) <= 1.0)),
        'heading_z_p95_le_0_005rad': (
            bool(state_diffs['target_z'])
            and bool(np.percentile(state_diffs['target_z'], 95) <= 0.005)),
        'command_angle_p95_le_0_5deg': (
            bool(angle_diffs)
            and bool(np.percentile(angle_diffs, 95) <= 0.5)),
        'command_speed_exact': (
            bool(speed_diffs) and max(speed_diffs) == 0.0),
    }
    return report, checks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('reference_bag', type=Path)
    parser.add_argument('candidate_bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reference = read_bag(args.reference_bag.expanduser().resolve())
    candidate = read_bag(args.candidate_bag.expanduser().resolve())
    comparison, checks = compare(reference, candidate)
    result = {
        'reference_bag': str(args.reference_bag.expanduser().resolve()),
        'candidate_bag': str(args.candidate_bag.expanduser().resolve()),
        'comparison': comparison,
        'checks': checks,
        'pipeline_regression_pass': all(checks.values()),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f'wrote {args.output}')


if __name__ == '__main__':
    main()
