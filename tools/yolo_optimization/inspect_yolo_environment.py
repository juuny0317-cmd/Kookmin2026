#!/usr/bin/env python3
"""Write a reproducible pre-run CUDA/PyTorch/Ultralytics manifest."""

from __future__ import annotations

import argparse
import json
import platform
from pathlib import Path
import subprocess

import cv2
import torch
import ultralytics


NVIDIA_QUERY = (
    'name,uuid,driver_version,temperature.gpu,clocks.gr,clocks.mem,'
    'utilization.gpu,utilization.memory,memory.total,memory.used')


def nvidia_smi():
    try:
        completed = subprocess.run(
            [
                'nvidia-smi', f'--query-gpu={NVIDIA_QUERY}',
                '--format=csv,noheader,nounits',
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
        return {
            'returncode': completed.returncode,
            'stdout': completed.stdout.strip(),
            'stderr': completed.stderr.strip(),
        }
    except Exception as exc:
        return {'returncode': None, 'stdout': '', 'stderr': str(exc)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    cuda_available = bool(torch.cuda.is_available())
    report = {
        'platform': platform.platform(),
        'python': platform.python_version(),
        'torch': str(torch.__version__),
        'torch_cuda': str(torch.version.cuda),
        'torchvision': None,
        'ultralytics': str(ultralytics.__version__),
        'opencv': str(cv2.__version__),
        'cuda_available': cuda_available,
        'cuda_device_count': int(torch.cuda.device_count()),
        'cuda_devices': [
            {
                'index': index,
                'name': torch.cuda.get_device_name(index),
                'capability': list(torch.cuda.get_device_capability(index)),
            }
            for index in range(torch.cuda.device_count())
        ] if cuda_available else [],
        'cudnn_enabled': bool(torch.backends.cudnn.enabled),
        'cudnn_version': torch.backends.cudnn.version(),
        'cudnn_benchmark': bool(torch.backends.cudnn.benchmark),
        'matmul_allow_tf32': bool(torch.backends.cuda.matmul.allow_tf32),
        'cudnn_allow_tf32': bool(torch.backends.cudnn.allow_tf32),
        'nvidia_smi': nvidia_smi(),
    }
    try:
        import torchvision
        report['torchvision'] = str(torchvision.__version__)
    except Exception as exc:
        report['torchvision_error'] = str(exc)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + '\n', encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
