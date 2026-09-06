#!/usr/bin/env python3
"""Write the CPU-only YOLO runtime preflight manifest."""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
import platform
from pathlib import Path

import cv2
import torch
import ultralytics


def package_version(name):
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def read_optional(path):
    try:
        return path.read_text(encoding='utf-8').strip()
    except (FileNotFoundError, PermissionError):
        return None


def cpu_topology():
    rows = []
    base = Path('/sys/devices/system/cpu')
    for cpu in sorted(os.sched_getaffinity(0)):
        root = base / f'cpu{cpu}'
        rows.append({
            'logical_cpu': cpu,
            'core_id': read_optional(root / 'topology/core_id'),
            'package_id': read_optional(
                root / 'topology/physical_package_id'),
            'thread_siblings_list': read_optional(
                root / 'topology/thread_siblings_list'),
            'scaling_driver': read_optional(
                root / 'cpufreq/scaling_driver'),
            'scaling_governor': read_optional(
                root / 'cpufreq/scaling_governor'),
            'scaling_cur_freq_khz': read_optional(
                root / 'cpufreq/scaling_cur_freq'),
            'scaling_min_freq_khz': read_optional(
                root / 'cpufreq/scaling_min_freq'),
            'scaling_max_freq_khz': read_optional(
                root / 'cpufreq/scaling_max_freq'),
        })
    return rows


def memory_info():
    values = {}
    for line in Path('/proc/meminfo').read_text(
            encoding='utf-8').splitlines():
        key, value = line.split(':', 1)
        values[key] = value.strip()
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = {
        'scope': 'CPU-only; no NVIDIA device, library, or command is probed.',
        'platform': platform.platform(),
        'python': platform.python_version(),
        'processor': platform.processor(),
        'logical_cpu_count': os.cpu_count(),
        'process_affinity': sorted(os.sched_getaffinity(0)),
        'load_average': list(os.getloadavg()),
        'memory': memory_info(),
        'torch': str(torch.__version__),
        'torch_cuda_build_metadata_only': str(torch.version.cuda),
        'torch_cuda_available': bool(torch.cuda.is_available()),
        'torch_intra_op_threads': int(torch.get_num_threads()),
        'torch_inter_op_threads': int(torch.get_num_interop_threads()),
        'mkl_available': bool(torch.backends.mkl.is_available()),
        'openmp_available': bool(torch.backends.openmp.is_available()),
        'torch_parallel_info': torch.__config__.parallel_info(),
        'ultralytics': str(ultralytics.__version__),
        'opencv': str(cv2.__version__),
        'opencv_threads': int(cv2.getNumThreads()),
        'environment_thread_limits': {
            key: os.environ.get(key) for key in (
                'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
                'OPENBLAS_NUM_THREADS', 'OMP_DYNAMIC')
        },
        'cpu_topology': cpu_topology(),
        'backend_availability_not_benchmarked': {
            'onnxruntime_version': package_version('onnxruntime'),
            'onnx_version': package_version('onnx'),
        },
    }
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text + '\n', encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
