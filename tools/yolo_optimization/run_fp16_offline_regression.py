#!/usr/bin/env python3
"""Compare the current lane YOLO PyTorch FP32/FP16 on identical bag frames.

This deliberately reproduces the node's Ultralytics call, confidence filter,
integer bbox conversion, and class-wise custom NMS.  It refuses to label a
CPU fallback as FP16 and records backend/model/input dtype evidence.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

from cv_bridge import CvBridge
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import torch
from ultralytics import YOLO


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def describe(values):
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


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def load_frames(bag, topic, max_frames, stride):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in types:
        raise RuntimeError(f'{topic} is absent from {bag}')
    message_type = get_message(types[topic])
    bridge = CvBridge()
    frames = []
    topic_index = 0
    while reader.has_next():
        name, serialized, _ = reader.read_next()
        if name != topic:
            continue
        if topic_index % stride:
            topic_index += 1
            continue
        topic_index += 1
        message = deserialize_message(serialized, message_type)
        frame = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        frames.append((stamp_ns(message.header.stamp), frame.copy()))
        if max_frames > 0 and len(frames) >= max_frames:
            break
    if not frames:
        raise RuntimeError(f'No frames selected from {topic}')
    return frames


def iou(left, right):
    ix1 = max(left['xmin'], right['xmin'])
    iy1 = max(left['ymin'], right['ymin'])
    ix2 = min(left['xmax'], right['xmax'])
    iy2 = min(left['ymax'], right['ymax'])
    intersection = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    left_area = max(0, left['xmax'] - left['xmin']) * max(
        0, left['ymax'] - left['ymin'])
    right_area = max(0, right['xmax'] - right['xmin']) * max(
        0, right['ymax'] - right['ymin'])
    union = left_area + right_area - intersection
    return float(intersection) / float(union) if union > 0 else 0.0


def custom_nms(detections, threshold):
    grouped = {}
    for detection in detections:
        grouped.setdefault(detection['class_name'], []).append(detection)
    final = []
    for group in grouped.values():
        remaining = sorted(
            group, key=lambda item: item['confidence'], reverse=True)
        while remaining:
            best = remaining.pop(0)
            final.append(best)
            remaining = [
                item for item in remaining if iou(best, item) < threshold]
    return final


class Runner:
    def __init__(self, model_path, fp16, device, imgsz, class_name,
                 inference_conf, output_conf, custom_nms_iou):
        self.label = 'fp16' if fp16 else 'fp32'
        self.fp16 = bool(fp16)
        self.device_arg = str(device)
        self.imgsz = int(imgsz)
        self.inference_conf = float(inference_conf)
        self.output_conf = float(output_conf)
        self.custom_nms_iou = float(custom_nms_iou)
        self.model = YOLO(str(model_path))
        names = {
            int(key): str(value) for key, value in self.model.names.items()}
        matching = [key for key, value in names.items() if value == class_name]
        if len(matching) != 1:
            raise RuntimeError(
                f'Expected one {class_name!r} class, got names={names}')
        self.class_id = matching[0]
        self.class_name = class_name
        self.warmup = []
        self.inference_ms = []
        self.total_ms = []

    def kwargs(self):
        return {
            'conf': self.inference_conf,
            'imgsz': self.imgsz,
            'classes': [self.class_id],
            'half': self.fp16,
            'device': self.device_arg,
            'verbose': False,
        }

    def call(self, frame):
        torch.cuda.synchronize()
        started = time.perf_counter()
        results = self.model(frame, **self.kwargs())
        torch.cuda.synchronize()
        total_ms = (time.perf_counter() - started) * 1000.0
        speed = getattr(results[0], 'speed', {}) or {}
        inference_ms = float(speed.get('inference', float('nan')))
        return results[0], total_ms, inference_ms

    def warm(self, frame, count):
        for index in range(count):
            _, total_ms, inference_ms = self.call(frame)
            self.warmup.append({
                'iteration': index + 1,
                'total_ms': total_ms,
                'pure_inference_ms': inference_ms,
            })
        self.validate_runtime(frame)

    def validate_runtime(self, frame):
        predictor = self.model.predictor
        backend = predictor.model
        inner = backend.model
        parameter = next(inner.parameters())
        with torch.inference_mode():
            sample_input = predictor.preprocess([frame])
        torch.cuda.synchronize()
        self.runtime = {
            'device': str(predictor.device),
            'device_name': torch.cuda.get_device_name(predictor.device),
            'backend': f'{type(backend).__module__}.{type(backend).__name__}',
            'backend_pt': bool(getattr(backend, 'pt', False)),
            'backend_engine': bool(getattr(backend, 'engine', False)),
            'backend_fp16': bool(getattr(backend, 'fp16', False)),
            'model_parameter_dtype': str(parameter.dtype),
            'input_tensor_dtype': str(sample_input.dtype),
            'input_tensor_device': str(sample_input.device),
            'autocast_used': False,
        }
        expected = torch.float16 if self.fp16 else torch.float32
        failures = []
        if not str(predictor.device).startswith('cuda'):
            failures.append(f'device={predictor.device}')
        if bool(getattr(backend, 'fp16', False)) != self.fp16:
            failures.append(f'backend.fp16={backend.fp16}')
        if parameter.dtype != expected:
            failures.append(f'model={parameter.dtype}, expected={expected}')
        if sample_input.dtype != expected:
            failures.append(
                f'input={sample_input.dtype}, expected={expected}')
        if failures:
            raise RuntimeError(
                f'{self.label} runtime validation failed: '
                + ', '.join(failures))

    def infer(self, frame):
        result, total_ms, inference_ms = self.call(frame)
        self.total_ms.append(total_ms)
        self.inference_ms.append(inference_ms)
        candidates = []
        boxes = result.boxes
        if boxes is not None:
            for box in boxes:
                class_id = int(box.cls[0])
                if class_id != self.class_id:
                    continue
                confidence = float(box.conf[0])
                if confidence < self.output_conf:
                    continue
                coords = box.xyxy[0].cpu().numpy().astype(int)
                candidates.append({
                    'class_name': self.class_name,
                    'confidence': confidence,
                    'xmin': int(coords[0]),
                    'ymin': int(coords[1]),
                    'xmax': int(coords[2]),
                    'ymax': int(coords[3]),
                })
        return custom_nms(candidates, self.custom_nms_iou)

    def report(self):
        return {
            'runtime': self.runtime,
            'warmup_first_20': self.warmup[:20],
            'steady_pure_inference_ms': describe(self.inference_ms),
            'steady_total_call_ms': describe(self.total_ms),
        }


def match_detections(left, right):
    available = set(range(len(right)))
    matches = []
    for left_index, item in enumerate(left):
        candidates = [
            (iou(item, right[index]), index) for index in available
            if item['class_name'] == right[index]['class_name']]
        if not candidates:
            continue
        score, right_index = max(candidates)
        available.remove(right_index)
        matches.append((left_index, right_index, score))
    return matches, len(left) - len(matches), len(right) - len(matches)


def compare_outputs(rows, criteria):
    count_matches = 0
    class_mismatches = 0
    missing = 0
    extra = 0
    ious = []
    centers = []
    width_diffs = []
    height_diffs = []
    confidence_diffs = []
    frame_rows = []
    for source_ns, fp32, fp16 in rows:
        if len(fp32) == len(fp16):
            count_matches += 1
        if Counter(x['class_name'] for x in fp32) != Counter(
                x['class_name'] for x in fp16):
            class_mismatches += 1
        matches, frame_missing, frame_extra = match_detections(fp32, fp16)
        missing += frame_missing
        extra += frame_extra
        for left_index, right_index, score in matches:
            left = fp32[left_index]
            right = fp16[right_index]
            ious.append(score)
            left_center = np.array([
                (left['xmin'] + left['xmax']) / 2.0,
                (left['ymin'] + left['ymax']) / 2.0,
            ])
            right_center = np.array([
                (right['xmin'] + right['xmax']) / 2.0,
                (right['ymin'] + right['ymax']) / 2.0,
            ])
            centers.append(float(np.linalg.norm(left_center - right_center)))
            width_diffs.append(abs(
                (left['xmax'] - left['xmin'])
                - (right['xmax'] - right['xmin'])))
            height_diffs.append(abs(
                (left['ymax'] - left['ymin'])
                - (right['ymax'] - right['ymin'])))
            confidence_diffs.append(abs(
                left['confidence'] - right['confidence']))
        frame_rows.append({
            'source_ns': source_ns,
            'fp32_count': len(fp32),
            'fp16_count': len(fp16),
            'missing': frame_missing,
            'extra': frame_extra,
        })
    frame_count = len(rows)
    total_reference = sum(len(item[1]) for item in rows)
    count_match_pct = count_matches / frame_count * 100.0
    unmatched_ratio = (
        (missing + extra) / max(1, total_reference) * 100.0)
    metrics = {
        'frame_count': frame_count,
        'exact_detection_count_match_pct': count_match_pct,
        'class_mismatch_frame_count': class_mismatches,
        'reference_detection_count': total_reference,
        'detection_loss_count': missing,
        'extra_detection_count': extra,
        'unmatched_detection_ratio_pct': unmatched_ratio,
        'bbox_iou': describe(ious),
        'bbox_center_distance_px': describe(centers),
        'bbox_width_abs_difference_px': describe(width_diffs),
        'bbox_height_abs_difference_px': describe(height_diffs),
        'confidence_abs_difference': describe(confidence_diffs),
        'per_frame': frame_rows,
    }
    checks = {
        'count_match': count_match_pct >= criteria['min_count_match_pct'],
        'class_match': class_mismatches <= criteria['max_class_mismatch'],
        'unmatched': unmatched_ratio <= criteria['max_unmatched_pct'],
        'iou': bool(ious) and float(np.percentile(ious, 5)) >= criteria[
            'min_iou_p5'],
        'center': bool(centers) and float(np.percentile(centers, 95)) <= (
            criteria['max_center_p95_px']),
        'width': bool(width_diffs) and float(
            np.percentile(width_diffs, 95)) <= criteria['max_size_p95_px'],
        'height': bool(height_diffs) and float(
            np.percentile(height_diffs, 95)) <= criteria['max_size_p95_px'],
        'confidence': bool(confidence_diffs) and float(
            np.percentile(confidence_diffs, 95)) <= criteria[
                'max_confidence_p95'],
    }
    return metrics, checks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--topic', default='/perception/lane/image')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--stride', type=int, default=1)
    parser.add_argument('--warmup-iterations', type=int, default=20)
    parser.add_argument('--device', default='0')
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--class-name', default='center_line')
    parser.add_argument('--inference-conf', type=float, default=0.20)
    parser.add_argument('--output-conf', type=float, default=0.25)
    parser.add_argument('--custom-nms-iou', type=float, default=0.60)
    parser.add_argument('--torch-threads', type=int, default=2)
    parser.add_argument('--opencv-threads', type=int, default=1)
    parser.add_argument('--min-count-match-pct', type=float, default=99.0)
    parser.add_argument('--max-class-mismatch', type=int, default=0)
    parser.add_argument('--max-unmatched-pct', type=float, default=0.5)
    parser.add_argument('--min-iou-p5', type=float, default=0.98)
    parser.add_argument('--max-center-p95-px', type=float, default=1.0)
    parser.add_argument('--max-size-p95-px', type=float, default=2.0)
    parser.add_argument('--max-confidence-p95', type=float, default=0.02)
    args = parser.parse_args()

    if not torch.cuda.is_available():
        raise RuntimeError(
            'CUDA is unavailable; refusing CPU half fallback. '
            f'torch={torch.__version__}, torch CUDA={torch.version.cuda}. '
            'Install a Torch CUDA build compatible with the NVIDIA driver '
            'before running FP16 regression.')
    torch.set_num_threads(max(1, args.torch_threads))
    cv2.setNumThreads(max(0, args.opencv_threads))
    frames = load_frames(
        args.bag.expanduser().resolve(), args.topic,
        args.max_frames, max(1, args.stride))
    runners = {
        'fp32': Runner(
            args.model, False, args.device, args.imgsz, args.class_name,
            args.inference_conf, args.output_conf, args.custom_nms_iou),
        'fp16': Runner(
            args.model, True, args.device, args.imgsz, args.class_name,
            args.inference_conf, args.output_conf, args.custom_nms_iou),
    }
    for runner in runners.values():
        runner.warm(frames[0][1], max(1, args.warmup_iterations))

    outputs = []
    for frame_index, (source_ns, frame) in enumerate(frames):
        order = ('fp32', 'fp16') if frame_index % 2 == 0 else (
            'fp16', 'fp32')
        result = {}
        for label in order:
            result[label] = runners[label].infer(frame)
        outputs.append((source_ns, result['fp32'], result['fp16']))

    criteria = {
        'min_count_match_pct': args.min_count_match_pct,
        'max_class_mismatch': args.max_class_mismatch,
        'max_unmatched_pct': args.max_unmatched_pct,
        'min_iou_p5': args.min_iou_p5,
        'max_center_p95_px': args.max_center_p95_px,
        'max_size_p95_px': args.max_size_p95_px,
        'max_confidence_p95': args.max_confidence_p95,
    }
    comparison, equivalence_checks = compare_outputs(outputs, criteria)
    fp32_report = runners['fp32'].report()
    fp16_report = runners['fp16'].report()
    performance_checks = {
        'inference_median_reduced': (
            fp16_report['steady_pure_inference_ms']['median']
            < fp32_report['steady_pure_inference_ms']['median']),
        'inference_p95_not_worse': (
            fp16_report['steady_pure_inference_ms']['p95']
            <= fp32_report['steady_pure_inference_ms']['p95']),
        'total_median_reduced': (
            fp16_report['steady_total_call_ms']['median']
            < fp32_report['steady_total_call_ms']['median']),
    }
    detection_pass = all(equivalence_checks.values())
    performance_pass = all(performance_checks.values())
    report = {
        'bag': str(args.bag.expanduser().resolve()),
        'topic': args.topic,
        'model': str(args.model.expanduser().resolve()),
        'model_sha256': sha256(args.model.expanduser().resolve()),
        'parameters_held_constant': {
            'imgsz': args.imgsz,
            'class_name': args.class_name,
            'inference_conf': args.inference_conf,
            'output_conf': args.output_conf,
            'custom_nms_iou': args.custom_nms_iou,
        },
        'fp32': fp32_report,
        'fp16': fp16_report,
        'detection_comparison': comparison,
        'criteria': criteria,
        'equivalence_checks': equivalence_checks,
        'performance_checks': performance_checks,
        'detection_pass': detection_pass,
        'offline_performance_pass': performance_pass,
        'downstream_center_curve_control_status': 'NOT_RUN',
        'vehicle_test_eligible': False,
        'vehicle_test_blockers': [
            'Run source-aligned center_curve/lane-state/control replay '
            'comparison before vehicle testing.'
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f'wrote {args.output}')


if __name__ == '__main__':
    main()
