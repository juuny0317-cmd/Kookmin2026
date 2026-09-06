#!/usr/bin/env python3
"""
Run source-identical PyTorch FP32 CPU thread-count measurements.

Each intra/inter-op pair runs in a fresh process because PyTorch does not
permit changing inter-op threads after parallel work has started.  The model,
input size, confidence, class filtering, integer bbox conversion and custom
NMS match the lane YOLO node.  This is the pure-inference screen; it does not
select the final vehicle setting without a full-stack replay.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from collections import Counter


def describe(values):
    import numpy as np

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


def parse_cpu_list(value):
    cpus = set()
    for part in str(value).split(','):
        part = part.strip()
        if not part:
            continue
        if '-' in part:
            start, end = (int(item) for item in part.split('-', 1))
            cpus.update(range(start, end + 1))
        else:
            cpus.add(int(part))
    return cpus


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def load_frames(bag, topic, max_frames, stride):
    from cv_bridge import CvBridge
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
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
        selected = topic_index % stride == 0
        topic_index += 1
        if not selected:
            continue
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
    return float(intersection) / float(union) if union else 0.0


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


def cpu_frequencies_mhz():
    values = []
    for index in sorted(os.sched_getaffinity(0)):
        path = Path('/sys/devices/system/cpu') / f'cpu{index}' / (
            'cpufreq/scaling_cur_freq')
        try:
            values.append(float(path.read_text().strip()) / 1000.0)
        except (FileNotFoundError, PermissionError, ValueError):
            continue
    return describe(values)


def runtime_manifest(model, frame, args, torch, cv2):
    predictor = model.predictor
    backend = predictor.model
    inner = backend.model
    parameter = next(inner.parameters())
    with torch.inference_mode():
        sample_input = predictor.preprocess([frame])
    if str(predictor.device) != 'cpu':
        raise RuntimeError(
            f'CPU benchmark created unexpected device {predictor.device}')
    if parameter.dtype != torch.float32 or sample_input.dtype != torch.float32:
        raise RuntimeError(
            'FP32 benchmark created an unexpected dtype: '
            f'model={parameter.dtype}, input={sample_input.dtype}')
    return {
        'torch': str(torch.__version__),
        'torch_cuda_build': str(torch.version.cuda),
        'cuda_available': bool(torch.cuda.is_available()),
        'device': str(predictor.device),
        'backend': f'{type(backend).__module__}.{type(backend).__name__}',
        'backend_pt': bool(getattr(backend, 'pt', False)),
        'model_parameter_dtype': str(parameter.dtype),
        'input_tensor_dtype': str(sample_input.dtype),
        'requested_intra_op_threads': args.worker_intra,
        'requested_inter_op_threads': args.worker_interop,
        'actual_intra_op_threads': int(torch.get_num_threads()),
        'actual_inter_op_threads': int(torch.get_num_interop_threads()),
        'mkl_available': bool(torch.backends.mkl.is_available()),
        'openmp_available': bool(torch.backends.openmp.is_available()),
        'opencv_threads': int(cv2.getNumThreads()),
        'affinity_logical_cpus': sorted(os.sched_getaffinity(0)),
        'omp_num_threads': os.environ.get('OMP_NUM_THREADS'),
        'mkl_num_threads': os.environ.get('MKL_NUM_THREADS'),
        'openblas_num_threads': os.environ.get('OPENBLAS_NUM_THREADS'),
        'omp_dynamic': os.environ.get('OMP_DYNAMIC'),
    }


def infer_detections(result, class_id, class_name, output_conf, nms_iou):
    candidates = []
    boxes = result.boxes
    if boxes is not None:
        for box in boxes:
            if int(box.cls[0]) != class_id:
                continue
            confidence = float(box.conf[0])
            if confidence < output_conf:
                continue
            coords = box.xyxy[0].cpu().numpy().astype(int)
            candidates.append({
                'class_name': class_name,
                'confidence': confidence,
                'xmin': int(coords[0]),
                'ymin': int(coords[1]),
                'xmax': int(coords[2]),
                'ymax': int(coords[3]),
            })
    return custom_nms(candidates, nms_iou)


def worker(args):
    os.environ['OMP_NUM_THREADS'] = str(args.worker_intra)
    os.environ['MKL_NUM_THREADS'] = str(args.worker_intra)
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    os.environ['OMP_DYNAMIC'] = 'FALSE'
    if args.cpu_affinity:
        os.sched_setaffinity(0, parse_cpu_list(args.cpu_affinity))

    import cv2
    import torch
    from ultralytics import YOLO

    torch.set_num_threads(args.worker_intra)
    torch.set_num_interop_threads(args.worker_interop)
    cv2.setNumThreads(args.opencv_threads)
    frames = load_frames(
        args.bag, args.topic, args.max_frames, max(1, args.stride))
    model = YOLO(str(args.model))
    names = {int(key): str(value) for key, value in model.names.items()}
    class_ids = [
        key for key, value in names.items() if value == args.class_name]
    if len(class_ids) != 1:
        raise RuntimeError(
            f'Expected one {args.class_name!r} class, got {names}')
    class_id = class_ids[0]
    kwargs = {
        'conf': args.inference_conf,
        'imgsz': args.imgsz,
        'classes': [class_id],
        'half': False,
        'device': 'cpu',
        'verbose': False,
    }

    # Ultralytics select_device() sets torch intra-op threads to its own
    # module-level NUM_THREADS during the first predictor construction.  That
    # silently invalidates an A/B configured before the first model call.
    # Create the backend once, then restore and verify the requested limits
    # before every measured warmup and source frame.
    setup_started = time.perf_counter()
    setup_result = model(frames[0][1], **kwargs)[0]
    setup_total_ms = (time.perf_counter() - setup_started) * 1000.0
    setup_speed = getattr(setup_result, 'speed', {}) or {}
    torch.set_num_threads(args.worker_intra)
    actual = (
        int(torch.get_num_threads()), int(torch.get_num_interop_threads()))
    requested = (args.worker_intra, args.worker_interop)
    if actual != requested:
        raise RuntimeError(
            'PyTorch thread mismatch after Ultralytics backend setup: '
            f'requested={requested}, actual={actual}')

    warmups = [{
        'iteration': 0,
        'phase': 'unmeasured_backend_setup_before_thread_restore',
        'pure_inference_ms': float(
            setup_speed.get('inference', float('nan'))),
        'total_call_ms': setup_total_ms,
    }]
    for iteration in range(args.warmup_iterations):
        started = time.perf_counter()
        result = model(frames[0][1], **kwargs)[0]
        total_ms = (time.perf_counter() - started) * 1000.0
        speed = getattr(result, 'speed', {}) or {}
        warmups.append({
            'iteration': iteration + 1,
            'phase': 'verified_thread_warmup',
            'pure_inference_ms': float(speed.get('inference', float('nan'))),
            'total_call_ms': total_ms,
        })
    manifest = runtime_manifest(model, frames[0][1], args, torch, cv2)

    frequency_before = cpu_frequencies_mhz()
    wall_started = time.perf_counter()
    cpu_started = time.process_time()
    rows = []
    for source_ns, frame in frames:
        started = time.perf_counter()
        result = model(frame, **kwargs)[0]
        total_ms = (time.perf_counter() - started) * 1000.0
        speed = getattr(result, 'speed', {}) or {}
        rows.append({
            'source_ns': source_ns,
            'preprocess_ms': float(speed.get('preprocess', float('nan'))),
            'pure_inference_ms': float(
                speed.get('inference', float('nan'))),
            'postprocess_ms': float(
                speed.get('postprocess', float('nan'))),
            'total_call_ms': total_ms,
            'detections': infer_detections(
                result, class_id, args.class_name,
                args.output_conf, args.custom_nms_iou),
        })
    cpu_s = time.process_time() - cpu_started
    wall_s = time.perf_counter() - wall_started
    frequency_after = cpu_frequencies_mhz()
    report = {
        'configuration': {
            'intra_op_threads': args.worker_intra,
            'inter_op_threads': args.worker_interop,
            'repeat': args.worker_repeat,
        },
        'runtime': manifest,
        'warmup': warmups,
        'frame_count': len(rows),
        'pure_inference_ms': describe([
            row['pure_inference_ms'] for row in rows]),
        'total_call_ms': describe([
            row['total_call_ms'] for row in rows]),
        'preprocess_ms': describe([row['preprocess_ms'] for row in rows]),
        'postprocess_ms': describe([row['postprocess_ms'] for row in rows]),
        'measured_wall_s': wall_s,
        'measured_process_cpu_s': cpu_s,
        'process_cpu_pct_one_core_100': cpu_s / max(wall_s, 1e-9) * 100.0,
        'cpu_frequency_before_mhz': frequency_before,
        'cpu_frequency_after_mhz': frequency_after,
        'frames': rows,
    }
    args.worker_output.parent.mkdir(parents=True, exist_ok=True)
    args.worker_output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')


def match_detections(left, right):
    available = set(range(len(right)))
    matches = []
    for left_index, item in enumerate(left):
        candidates = [
            (iou(item, right[index]), index) for index in available
            if item['class_name'] == right[index]['class_name']
        ]
        if not candidates:
            continue
        score, right_index = max(candidates)
        available.remove(right_index)
        matches.append((left_index, right_index, score))
    return matches, len(left) - len(matches), len(right) - len(matches)


def detection_comparison(reference_frames, candidate_frames):
    reference = {row['source_ns']: row for row in reference_frames}
    candidate = {row['source_ns']: row for row in candidate_frames}
    common = sorted(set(reference) & set(candidate))
    count_mismatch = 0
    class_mismatch = 0
    missing = 0
    extra = 0
    bbox_differences = []
    confidence_differences = []
    for source in common:
        left = reference[source]['detections']
        right = candidate[source]['detections']
        if len(left) != len(right):
            count_mismatch += 1
        if Counter(item['class_name'] for item in left) != Counter(
                item['class_name'] for item in right):
            class_mismatch += 1
        matches, frame_missing, frame_extra = match_detections(left, right)
        missing += frame_missing
        extra += frame_extra
        for left_index, right_index, _ in matches:
            l_item = left[left_index]
            r_item = right[right_index]
            bbox_differences.extend([
                abs(l_item[field] - r_item[field])
                for field in ('xmin', 'ymin', 'xmax', 'ymax')
            ])
            confidence_differences.append(abs(
                l_item['confidence'] - r_item['confidence']))
    bbox_stats = describe(bbox_differences)
    confidence_stats = describe(confidence_differences)
    exact = (
        len(common) == len(reference) == len(candidate)
        and count_mismatch == 0
        and class_mismatch == 0
        and missing == 0
        and extra == 0
        and (bbox_stats['max'] in (None, 0.0))
        and (confidence_stats['max'] in (None, 0.0))
    )
    return {
        'reference_frames': len(reference),
        'candidate_frames': len(candidate),
        'common_frames': len(common),
        'detection_count_mismatch_frames': count_mismatch,
        'class_mismatch_frames': class_mismatch,
        'missing_detections': missing,
        'extra_detections': extra,
        'bbox_coordinate_abs_difference_px': bbox_stats,
        'confidence_abs_difference': confidence_stats,
        'bit_exact_detection_output': exact,
    }


def aggregate_workers(items):
    pure = []
    total = []
    preprocess = []
    postprocess = []
    cpu = []
    for item in items:
        pure.extend(row['pure_inference_ms'] for row in item['frames'])
        total.extend(row['total_call_ms'] for row in item['frames'])
        preprocess.extend(row['preprocess_ms'] for row in item['frames'])
        postprocess.extend(row['postprocess_ms'] for row in item['frames'])
        cpu.append(item['process_cpu_pct_one_core_100'])
    return {
        'repeats': len(items),
        'runtime': items[0]['runtime'],
        'pure_inference_ms': describe(pure),
        'total_call_ms': describe(total),
        'preprocess_ms': describe(preprocess),
        'postprocess_ms': describe(postprocess),
        'process_cpu_pct_one_core_100': describe(cpu),
        'per_repeat': [{
            'configuration': item['configuration'],
            'pure_inference_ms': item['pure_inference_ms'],
            'total_call_ms': item['total_call_ms'],
            'process_cpu_pct_one_core_100': item[
                'process_cpu_pct_one_core_100'],
            'cpu_frequency_before_mhz': item[
                'cpu_frequency_before_mhz'],
            'cpu_frequency_after_mhz': item[
                'cpu_frequency_after_mhz'],
        } for item in items],
    }


def parent(args):
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    worker_dir = output.parent / f'{output.stem}_workers'
    worker_dir.mkdir(parents=True, exist_ok=True)
    configurations = [
        (intra, interop)
        for interop in args.inter_op
        for intra in args.intra_op
    ]
    collected = {configuration: [] for configuration in configurations}
    for repeat in range(args.repeats):
        order = configurations if repeat % 2 == 0 else list(
            reversed(configurations))
        for intra, interop in order:
            worker_output = worker_dir / (
                f'intra{intra}_inter{interop}_repeat{repeat + 1}.json')
            command = [
                sys.executable, str(Path(__file__).resolve()),
                str(args.bag), '--model', str(args.model),
                '--topic', args.topic,
                '--imgsz', str(args.imgsz),
                '--class-name', args.class_name,
                '--inference-conf', str(args.inference_conf),
                '--output-conf', str(args.output_conf),
                '--custom-nms-iou', str(args.custom_nms_iou),
                '--warmup-iterations', str(args.warmup_iterations),
                '--max-frames', str(args.max_frames),
                '--stride', str(args.stride),
                '--opencv-threads', str(args.opencv_threads),
                '--worker', '--worker-intra', str(intra),
                '--worker-interop', str(interop),
                '--worker-repeat', str(repeat + 1),
                '--worker-output', str(worker_output),
            ]
            if args.cpu_affinity:
                command.extend(['--cpu-affinity', args.cpu_affinity])
            completed = subprocess.run(command, check=False)
            if completed.returncode != 0:
                raise RuntimeError(
                    f'Worker failed ({completed.returncode}): {command}')
            collected[(intra, interop)].append(json.loads(
                worker_output.read_text(encoding='utf-8')))

    reference_key = (args.intra_op[0], args.inter_op[0])
    reference_frames = collected[reference_key][0]['frames']
    configurations_report = {}
    for configuration, items in collected.items():
        intra, interop = configuration
        key = f'intra_{intra}_inter_{interop}'
        aggregate = aggregate_workers(items)
        aggregate['detection_vs_reference'] = detection_comparison(
            reference_frames, items[0]['frames'])
        aggregate['repeat_detection_consistency'] = [
            detection_comparison(items[0]['frames'], item['frames'])
            for item in items[1:]
        ]
        configurations_report[key] = aggregate

    ranked = sorted(
        configurations_report,
        key=lambda key: (
            configurations_report[key]['pure_inference_ms']['median'],
            configurations_report[key]['pure_inference_ms']['p95'],
        ),
    )
    report = {
        'scope': (
            'Pure PyTorch FP32 CPU screen on source-identical routed C1 '
            'frames. Final selection requires full ROS stack replay.'),
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
            'opencv_threads': args.opencv_threads,
            'cpu_affinity': args.cpu_affinity or 'inherited',
        },
        'reference_configuration': (
            f'intra_{reference_key[0]}_inter_{reference_key[1]}'),
        'configurations': configurations_report,
        'pure_inference_ranking': ranked,
        'pure_screen_best': ranked[0],
        'final_pytorch_cpu_setting': 'NOT_SELECTED_FULL_STACK_REQUIRED',
        'onnx_runtime_review': 'DEFERRED_UNTIL_PYTORCH_FULL_STACK_RESULT',
    }
    output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    print(json.dumps({
        'pure_inference_ranking': ranked,
        'metrics': {
            key: {
                'pure_inference_ms': configurations_report[
                    key]['pure_inference_ms'],
                'total_call_ms': configurations_report[
                    key]['total_call_ms'],
                'detection_exact': configurations_report[
                    key]['detection_vs_reference'][
                        'bit_exact_detection_output'],
            }
            for key in ranked
        },
        'output': str(output),
    }, ensure_ascii=False, indent=2))


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--topic', default='/perception/lane/image')
    parser.add_argument('--output', type=Path)
    parser.add_argument(
        '--intra-op', type=int, nargs='+', default=[8, 6, 4, 2])
    parser.add_argument('--inter-op', type=int, nargs='+', default=[16])
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--stride', type=int, default=1)
    parser.add_argument('--warmup-iterations', type=int, default=20)
    parser.add_argument('--cpu-affinity', default='')
    parser.add_argument('--opencv-threads', type=int, default=1)
    parser.add_argument('--imgsz', type=int, default=320)
    parser.add_argument('--class-name', default='center_line')
    parser.add_argument('--inference-conf', type=float, default=0.20)
    parser.add_argument('--output-conf', type=float, default=0.25)
    parser.add_argument('--custom-nms-iou', type=float, default=0.60)
    parser.add_argument(
        '--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--worker-intra', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--worker-interop', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--worker-repeat', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--worker-output', type=Path, help=argparse.SUPPRESS)
    return parser


def main():
    args = build_parser().parse_args()
    args.bag = args.bag.expanduser().resolve()
    args.model = args.model.expanduser().resolve()
    if not args.bag.exists():
        raise FileNotFoundError(args.bag)
    if not args.model.is_file():
        raise FileNotFoundError(args.model)
    if args.worker:
        if args.worker_output is None:
            raise RuntimeError('--worker-output is required in worker mode')
        worker(args)
    else:
        if args.output is None:
            raise RuntimeError('--output is required')
        if any(value <= 0 for value in args.intra_op + args.inter_op):
            raise RuntimeError('Thread counts must be positive')
        args.repeats = max(1, args.repeats)
        parent(args)


if __name__ == '__main__':
    main()
