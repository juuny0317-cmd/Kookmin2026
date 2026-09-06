"""Ultralytics YOLO ROS node with non-blocking latest-frame inference."""

from collections import defaultdict, deque
import json
import os
from pathlib import Path
import threading
import time
from types import MethodType

from cam.perception_utils import (
    normalize_model_names,
    parse_name_list,
    resolve_enabled_class_ids,
    stamp_to_ns,
)
from custom_interfaces.msg import Detection, Detections, PipelineTiming
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from ultralytics import YOLO


def reliability_from_string(value: str) -> ReliabilityPolicy:
    normalized = str(value).strip().lower()
    if normalized == 'reliable':
        return ReliabilityPolicy.RELIABLE
    if normalized == 'best_effort':
        return ReliabilityPolicy.BEST_EFFORT
    raise ValueError(
        'QoS reliability must be either "reliable" or "best_effort", '
        f'got {value!r}'
    )


def qos_profile(reliability: str, depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability_from_string(reliability),
        durability=DurabilityPolicy.VOLATILE,
    )


def parse_class_name_aliases(value, model_names):
    """Parse ``source=canonical`` aliases and validate model sources."""
    aliases = {}
    available_names = set(normalize_model_names(model_names).values())
    for item in str(value).split(','):
        item = item.strip()
        if not item:
            continue
        if '=' not in item:
            raise ValueError(
                'class_name_aliases entries must use source=canonical, '
                f'got {item!r}'
            )
        source, canonical = (part.strip() for part in item.split('=', 1))
        if not source or not canonical:
            raise ValueError(f'Invalid class name alias: {item!r}')
        if source not in available_names:
            raise ValueError(
                f'Alias source {source!r} is not present in model classes '
                f'{sorted(available_names)}'
            )
        aliases[source] = canonical
    return aliases


def canonical_class_name(source_name, aliases):
    return aliases.get(source_name, source_name)


class YoloNode(Node):
    """Run one fixed-purpose YOLO model using a latest-frame worker."""

    def __init__(self):
        super().__init__('yolo_node')
        self.startup_monotonic_s = time.monotonic()

        self.declare_parameter('image_topic_name', '/image_raw')
        self.declare_parameter('image_qos_reliability', 'reliable')
        self.declare_parameter('detections_topic', '/yolo_detections')
        self.declare_parameter('source_image_topic', '/yolo_source_image')
        self.declare_parameter(
            'annotated_image_topic', '/yolo_annotated_image')
        self.declare_parameter('output_image_qos_reliability', 'best_effort')
        self.declare_parameter('publish_source_image', False)
        self.declare_parameter('publish_annotated_image', False)
        self.declare_parameter('annotated_max_rate_hz', 5.0)

        self.declare_parameter(
            'model_filename', 'center_line_yolov10n_320_best.pt')
        self.declare_parameter('model_path', '')
        self.declare_parameter('inference_class_names', '')
        self.declare_parameter('class_name_aliases', '')
        self.declare_parameter('enable_checkerboard', True)
        self.declare_parameter('class_1_conf_threshold', 0.5)
        self.declare_parameter('general_conf_threshold', 0.25)
        self.declare_parameter('cone_conf_threshold', 0.20)
        self.declare_parameter('iou_threshold', 0.6)
        self.declare_parameter('max_inference_rate_hz', 0.0)
        self.declare_parameter('inference_image_size', 640)
        self.declare_parameter('torch_num_threads', 0)
        self.declare_parameter('torch_num_interop_threads', 1)
        self.declare_parameter('opencv_num_threads', 1)
        self.declare_parameter('torch_compile_mode', 'none')
        self.declare_parameter('compile_input_width', 0)
        self.declare_parameter('compile_input_height', 0)
        self.declare_parameter('compile_warmup_iterations', 2)
        self.declare_parameter('inference_device', '')
        self.declare_parameter('use_fp16', False)
        self.declare_parameter('runtime_report_path', '')

        # Visualization stays entirely disabled unless explicitly requested.
        self.declare_parameter('show_visualization', False)
        self.declare_parameter('show_labels', True)
        self.declare_parameter('show_conf', True)
        self.declare_parameter('publish_performance_stats', False)
        self.declare_parameter('performance_log_period_s', 5.0)
        self.declare_parameter('publish_pipeline_timing', False)
        self.declare_parameter('pipeline_timing_topic', '/pipeline_timing')

        self.image_topic = str(self.get_parameter('image_topic_name').value)
        self.image_reliability = str(
            self.get_parameter('image_qos_reliability').value)
        self.detections_topic = str(
            self.get_parameter('detections_topic').value)
        self.profile_prefix = (
            'lane_yolo' if 'lane' in self.detections_topic
            else 'scene_yolo'
        )
        self.publish_source_image = bool(
            self.get_parameter('publish_source_image').value)
        self.show_window = bool(
            self.get_parameter('show_visualization').value)
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated_image').value
        ) or self.show_window
        annotated_rate = float(
            self.get_parameter('annotated_max_rate_hz').value)
        self.annotated_period_s = (
            1.0 / annotated_rate if annotated_rate > 0.0 else 0.0)
        self.last_annotated_monotonic = 0.0
        self.latest_visualization = None
        self.visualization_lock = threading.Lock()
        self.visualization_timer = None

        self.general_conf_threshold = float(
            self.get_parameter('general_conf_threshold').value)
        self.obstacle_conf_threshold = float(
            self.get_parameter('class_1_conf_threshold').value)
        self.cone_conf_threshold = float(
            self.get_parameter('cone_conf_threshold').value)
        self.inference_conf_threshold = min(
            self.general_conf_threshold,
            self.obstacle_conf_threshold,
            self.cone_conf_threshold,
        )
        self.iou_threshold = float(
            self.get_parameter('iou_threshold').value)
        self.max_inference_rate_hz = float(
            self.get_parameter('max_inference_rate_hz').value)
        self.inference_image_size = int(
            self.get_parameter('inference_image_size').value)
        self.show_labels = bool(self.get_parameter('show_labels').value)
        self.show_conf = bool(self.get_parameter('show_conf').value)
        self.publish_stats = bool(
            self.get_parameter('publish_performance_stats').value)
        self.performance_log_period_s = max(
            1.0,
            float(self.get_parameter('performance_log_period_s').value),
        )

        self.backend_thread_limits_verified = False
        self.configure_thread_limits()
        model_path = str(self.get_parameter('model_path').value).strip()
        if not model_path:
            model_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                str(self.get_parameter('model_filename').value),
            )
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f'YOLO model file not found: {model_path}')
        model_load_started = time.monotonic()
        self.model = YOLO(model_path)
        self.get_logger().info(
            f'[STARTUP +{time.monotonic() - self.startup_monotonic_s:.3f}s] '
            f'{self.profile_prefix} model object loaded '
            f'({time.monotonic() - model_load_started:.3f}s)')
        self.model_names = normalize_model_names(self.model.names)
        self.class_name_aliases = parse_class_name_aliases(
            self.get_parameter('class_name_aliases').value,
            self.model_names,
        )

        requested_names = parse_name_list(
            self.get_parameter('inference_class_names').value)
        self.inference_class_ids = resolve_enabled_class_ids(
            self.model_names,
            requested_names,
            enable_checkerboard=bool(
                self.get_parameter('enable_checkerboard').value),
        )
        self.inference_class_names = [
            self.model_names[class_id] for class_id in self.inference_class_ids
        ]
        self.torch_compile_mode = str(
            self.get_parameter('torch_compile_mode').value
        ).strip().lower()
        self.inference_device = str(
            self.get_parameter('inference_device').value).strip()
        self.use_fp16 = bool(self.get_parameter('use_fp16').value)
        self.runtime_report_path = str(
            self.get_parameter('runtime_report_path').value).strip()
        self.runtime_manifest = {}
        self.warmup_samples = []
        self.preprocess_profiler_installed = False
        self.last_preprocess_profile = {}
        self.compile_model_if_requested()

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_lock = threading.Lock()
        self.new_image_event = threading.Event()
        self.worker_stop_event = threading.Event()
        self.worker_thread = None

        self.received_frames = 0
        self.processed_frames = 0
        self.replaced_frames = 0
        self.last_received_monotonic = None
        self.last_processed_monotonic = None
        self.max_input_gap_s = 0.0
        self.max_output_gap_s = 0.0
        self.inference_samples_ms = deque(maxlen=512)
        self.queue_age_samples_ms = deque(maxlen=512)
        self.end_to_end_samples_ms = deque(maxlen=512)
        self.output_timestamps_monotonic = deque(maxlen=120)
        self.last_performance_monotonic = time.monotonic()
        self.last_performance_counts = (0, 0)
        self.last_model_timing = None
        self.last_stage_timings = []
        self.first_image_logged = False
        self.first_inference_logged = False

        input_qos = qos_profile(self.image_reliability, depth=1)
        detection_qos = qos_profile('reliable', depth=5)
        output_image_qos = qos_profile(
            str(self.get_parameter('output_image_qos_reliability').value),
            depth=1,
        )
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            input_qos,
        )
        self.detection_publisher = self.create_publisher(
            Detections,
            self.detections_topic,
            detection_qos,
        )
        self.pipeline_timing_pub = None
        if bool(self.get_parameter('publish_pipeline_timing').value):
            timing_qos = qos_profile('reliable', depth=100)
            self.pipeline_timing_pub = self.create_publisher(
                PipelineTiming,
                self.get_parameter('pipeline_timing_topic').value,
                timing_qos,
            )
        self.source_image_publisher = None
        if self.publish_source_image:
            self.source_image_publisher = self.create_publisher(
                Image,
                str(self.get_parameter('source_image_topic').value),
                output_image_qos,
            )
        self.annotated_image_publisher = None
        if self.publish_annotated:
            self.annotated_image_publisher = self.create_publisher(
                Image,
                str(self.get_parameter('annotated_image_topic').value),
                output_image_qos,
            )
        self.window_name = f'{self.get_name()} YOLO'
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            # HighGUI event handling is not safe in the inference worker on
            # every Qt/OpenCV backend.  Keep all window calls on the ROS spin
            # thread and pass it only the latest annotated frame.
            self.visualization_timer = self.create_timer(
                1.0 / 30.0,
                self.display_latest_visualization,
            )

        self.worker_thread = threading.Thread(
            target=self.inference_worker,
            name=f'{self.get_name()}-inference-worker',
            daemon=True,
        )
        self.worker_thread.start()

        self.get_logger().info(
            'YOLO latest-frame worker started: '
            f'model={model_path}, input={self.image_topic}, '
            f'detections={self.detections_topic}, '
            f'classes={self.inference_class_names}, '
            f'aliases={self.class_name_aliases}, '
            f'imgsz={self.inference_image_size}, '
            f'device={self.runtime_manifest.get("device", "unknown")}, '
            f'fp16={self.runtime_manifest.get("backend_fp16", False)}, '
            f'torch_threads='
            f'{self.thread_configuration.get("actual_intra_op_threads")}/'
            f'{self.thread_configuration.get("actual_inter_op_threads")}, '
            f'affinity='
            f'{self.thread_configuration.get("process_affinity_logical_cpus")}'
            ', '
            f'torch_compile={self.torch_compile_mode}, '
            f'max_rate={self.max_inference_rate_hz:.1f}Hz, '
            f'input_qos={self.image_reliability}/depth1, '
            f'publish_source={self.publish_source_image}, '
            f'publish_annotated={self.publish_annotated}'
        )
        self.get_logger().info(
            f'[STARTUP +{time.monotonic() - self.startup_monotonic_s:.3f}s] '
            f'{self.profile_prefix} model warmup complete and worker ready')

    def configure_thread_limits(self):
        opencv_threads = int(self.get_parameter('opencv_num_threads').value)
        if opencv_threads >= 0:
            cv2.setNumThreads(opencv_threads)
        torch_threads = int(self.get_parameter('torch_num_threads').value)
        interop_threads = int(
            self.get_parameter('torch_num_interop_threads').value)
        self.thread_configuration = {
            'requested_intra_op_threads': torch_threads,
            'requested_inter_op_threads': interop_threads,
            'opencv_threads': int(cv2.getNumThreads()),
            'omp_num_threads': os.environ.get('OMP_NUM_THREADS'),
            'mkl_num_threads': os.environ.get('MKL_NUM_THREADS'),
            'openblas_num_threads': os.environ.get('OPENBLAS_NUM_THREADS'),
            'omp_dynamic': os.environ.get('OMP_DYNAMIC'),
            'process_affinity_logical_cpus': sorted(os.sched_getaffinity(0)),
            'apply_error': '',
        }
        try:
            import torch

            if torch_threads > 0:
                torch.set_num_threads(torch_threads)
            if interop_threads > 0:
                torch.set_num_interop_threads(interop_threads)
        except Exception as exc:
            self.thread_configuration['apply_error'] = str(exc)
            self.get_logger().warn(
                f'Unable to apply PyTorch thread limits: {exc}')
        finally:
            try:
                import torch

                self.thread_configuration.update({
                    'actual_intra_op_threads': int(
                        torch.get_num_threads()),
                    'actual_inter_op_threads': int(
                        torch.get_num_interop_threads()),
                    'mkl_available': bool(
                        torch.backends.mkl.is_available()),
                    'openmp_available': bool(
                        torch.backends.openmp.is_available()),
                })
            except Exception as exc:
                self.thread_configuration.setdefault(
                    'apply_error', str(exc))

    def verify_backend_thread_limits(self):
        """Restore limits changed by Ultralytics CPU backend setup once."""
        if self.backend_thread_limits_verified:
            return
        import torch

        requested = int(self.get_parameter('torch_num_threads').value)
        if requested > 0:
            # Ultralytics select_device() resets this to its module-level
            # NUM_THREADS while constructing the predictor.  Reapply only
            # after that one-time setup; inter-op must remain configured
            # before parallel work starts and is only verified here.
            torch.set_num_threads(requested)
        self.thread_configuration.update({
            'actual_intra_op_threads': int(torch.get_num_threads()),
            'actual_inter_op_threads': int(
                torch.get_num_interop_threads()),
            'verified_after_backend_setup': True,
        })
        failures = []
        requested_interop = int(
            self.get_parameter('torch_num_interop_threads').value)
        if requested > 0 and torch.get_num_threads() != requested:
            failures.append(
                f'intra requested={requested} '
                f'actual={torch.get_num_threads()}')
        if (requested_interop > 0
                and torch.get_num_interop_threads() != requested_interop):
            failures.append(
                'inter-op requested='
                f'{requested_interop} '
                f'actual={torch.get_num_interop_threads()}')
        if failures:
            raise RuntimeError(
                'PyTorch CPU thread verification failed after Ultralytics '
                'backend setup: ' + ', '.join(failures))
        self.backend_thread_limits_verified = True

    def prediction_kwargs(self):
        kwargs = {
            'conf': self.inference_conf_threshold,
            'imgsz': self.inference_image_size,
            'classes': self.inference_class_ids,
            'half': self.use_fp16,
            'verbose': False,
        }
        if self.inference_device:
            kwargs['device'] = self.inference_device
        return kwargs

    def predict_frame(self, frame):
        results = self.model(frame, **self.prediction_kwargs())
        self.verify_backend_thread_limits()
        return results

    @staticmethod
    def dtype_name(value):
        return str(value).replace('torch.', '') if value is not None else None

    def inspect_runtime(self, sample_frame):
        """Record the backend/device/dtype actually created by Ultralytics."""
        import torch

        predictor = getattr(self.model, 'predictor', None)
        backend = getattr(predictor, 'model', None)
        inner_model = getattr(backend, 'model', None)
        if predictor is None or backend is None or inner_model is None:
            raise RuntimeError('Ultralytics predictor/backend was not created')

        try:
            parameter_dtype = next(inner_model.parameters()).dtype
        except (AttributeError, StopIteration):
            parameter_dtype = None
        with torch.inference_mode():
            sample_tensor = predictor.preprocess([sample_frame])
        if getattr(sample_tensor, 'is_cuda', False):
            torch.cuda.synchronize(sample_tensor.device)
        input_dtype = getattr(sample_tensor, 'dtype', None)
        device = str(getattr(predictor, 'device', 'unknown'))
        backend_fp16 = bool(getattr(backend, 'fp16', False))
        backend_name = (
            f'{type(backend).__module__}.{type(backend).__name__}')
        inner_name = (
            f'{type(inner_model).__module__}.{type(inner_model).__name__}')

        if self.use_fp16:
            failures = []
            if not device.startswith('cuda'):
                failures.append(f'device={device}')
            if not backend_fp16:
                failures.append('AutoBackend.fp16=false')
            if parameter_dtype != torch.float16:
                failures.append(f'model_dtype={parameter_dtype}')
            if input_dtype != torch.float16:
                failures.append(f'input_dtype={input_dtype}')
            if failures:
                raise RuntimeError(
                    'FP16 was requested but real CUDA FP16 was not created: '
                    + ', '.join(failures))

        self.runtime_manifest = {
            'torch_version': str(torch.__version__),
            'torch_cuda_version': str(torch.version.cuda),
            'ultralytics_version': self.ultralytics_version(),
            'cuda_available': bool(torch.cuda.is_available()),
            'device': device,
            'device_name': (
                torch.cuda.get_device_name(predictor.device)
                if device.startswith('cuda') else None),
            'backend': backend_name,
            'inner_model': inner_name,
            'backend_pt': bool(getattr(backend, 'pt', False)),
            'backend_engine': bool(getattr(backend, 'engine', False)),
            'backend_onnx': bool(getattr(backend, 'onnx', False)),
            'backend_triton': bool(getattr(backend, 'triton', False)),
            'backend_fp16': backend_fp16,
            'model_parameter_dtype': self.dtype_name(parameter_dtype),
            'input_tensor_dtype': self.dtype_name(input_dtype),
            'input_tensor_device': str(getattr(sample_tensor, 'device', '')),
            'inference_mode': 'torch.inference_mode via smart_inference_mode',
            'autocast_used': False,
            'torch_compile_mode': self.torch_compile_mode,
            'cudnn_enabled': bool(torch.backends.cudnn.enabled),
            'cudnn_benchmark': bool(torch.backends.cudnn.benchmark),
            'cudnn_version': torch.backends.cudnn.version(),
            'matmul_allow_tf32': bool(
                torch.backends.cuda.matmul.allow_tf32),
            'cudnn_allow_tf32': bool(torch.backends.cudnn.allow_tf32),
            'call_path': (
                'YOLO.__call__ -> YOLO.predict -> '
                'BasePredictor.stream_inference -> AutoBackend'),
            'host_to_device_transfers_per_inference': (
                1 if device.startswith('cuda') else 0),
            'repeated_model_device_or_dtype_conversion': False,
            'cpu_thread_configuration': dict(self.thread_configuration),
        }

    @staticmethod
    def ultralytics_version():
        try:
            import ultralytics
            return str(ultralytics.__version__)
        except Exception:
            return 'unknown'

    def write_runtime_report(self):
        if not self.runtime_report_path:
            return
        path = Path(self.runtime_report_path).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps(
                    {
                        **self.runtime_manifest,
                        'warmup_samples': self.warmup_samples,
                    },
                    ensure_ascii=False,
                    indent=2,
                ) + '\n',
                encoding='utf-8',
            )
        except OSError as exc:
            self.get_logger().warn(
                f'Unable to write YOLO runtime report {path}: {exc}')

    def run_warmup(self, frame, iteration, phase):
        import torch

        if torch.cuda.is_available():
            torch.cuda.synchronize()
        started = time.perf_counter()
        results = self.predict_frame(frame)
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        total_ms = (time.perf_counter() - started) * 1000.0
        speed = getattr(results[0], 'speed', {}) or {}
        sample = {
            'iteration': int(iteration),
            'phase': str(phase),
            'total_call_ms': float(total_ms),
            'preprocess_ms': float(speed.get('preprocess', 0.0)),
            'pure_inference_ms': float(speed.get('inference', 0.0)),
            'postprocess_ms': float(speed.get('postprocess', 0.0)),
        }
        self.warmup_samples.append(sample)
        self.get_logger().info(
            'YOLO warmup '
            f'{iteration}: phase={phase} total={total_ms:.3f}ms '
            f'pre/infer/post={sample["preprocess_ms"]:.3f}/'
            f'{sample["pure_inference_ms"]:.3f}/'
            f'{sample["postprocess_ms"]:.3f}ms')
        return results

    def compile_model_if_requested(self):
        """
        Warm, and optionally compile, the model before subscribing.

        Preparation is deliberately completed during node startup.  Doing the
        first eager inference or compilation on a camera callback would create
        a multi-second perception gap while the vehicle is already active.
        """
        disabled_values = {'', 'none', 'off', 'false', 'disabled'}
        compile_requested = self.torch_compile_mode not in disabled_values
        if not compile_requested:
            self.torch_compile_mode = 'none'

        width = int(self.get_parameter('compile_input_width').value)
        height = int(self.get_parameter('compile_input_height').value)
        warmup_iterations = max(
            1,
            int(self.get_parameter('compile_warmup_iterations').value),
        )
        if width <= 0 or height <= 0:
            if compile_requested:
                self.get_logger().warn(
                    'torch.compile requested without a positive fixed input '
                    'width/height; continuing with eager inference')
            self.torch_compile_mode = 'none'
            return

        dummy_frame = np.zeros((height, width, 3), dtype=np.uint8)
        compile_start = time.monotonic()
        backend = None
        eager_model = None
        try:
            # Let Ultralytics create/fuse its AutoBackend on the first call.
            self.run_warmup(dummy_frame, 1, 'backend_setup')
            self.inspect_runtime(dummy_frame)
            if compile_requested:
                import torch

                predictor = getattr(self.model, 'predictor', None)
                backend = getattr(predictor, 'model', None)
                eager_model = getattr(backend, 'model', None)
                if eager_model is None:
                    raise RuntimeError(
                        'Ultralytics AutoBackend did not expose a PyTorch '
                        'model')
                backend.model = torch.compile(
                    eager_model,
                    mode=self.torch_compile_mode,
                    fullgraph=False,
                )
                remaining_warmups = warmup_iterations
                first_remaining_iteration = 2
            else:
                # The setup call above is also the first eager warmup.
                remaining_warmups = max(0, warmup_iterations - 1)
                first_remaining_iteration = 2

            for offset in range(remaining_warmups):
                self.run_warmup(
                    dummy_frame,
                    first_remaining_iteration + offset,
                    'compiled' if compile_requested else 'eager',
                )
            if compile_requested:
                # Inductor keeps a multi-process compiler pool alive by
                # default. Fixed-shape inference is fully compiled after the
                # warmups, so the pool only wastes CPU and hundreds of MiB.
                from torch._inductor.async_compile import (
                    shutdown_compile_workers,
                )
                shutdown_compile_workers()
        except Exception as exc:
            if backend is not None and eager_model is not None:
                backend.model = eager_model
            if not self.backend_thread_limits_verified:
                raise RuntimeError(
                    'CPU thread startup validation failed: '
                    f'{exc}') from exc
            if self.use_fp16:
                # Precision candidates must never silently become CPU/FP32;
                # doing so would invalidate both the latency and regression
                # result while appearing to be a successful Y1 launch.
                raise RuntimeError(
                    f'FP16 startup validation failed: {exc}') from exc
            action = 'torch.compile' if compile_requested else 'eager warmup'
            self.get_logger().warn(
                f'{action} failed; using eager inference: {exc}')
            self.torch_compile_mode = 'none'
            return

        action = 'Compiled' if compile_requested else 'Warmed eager'
        pure_samples = [
            item['pure_inference_ms'] for item in self.warmup_samples]
        steady_samples = pure_samples[-min(10, len(pure_samples)):]
        self.runtime_manifest['warmup_iterations'] = len(
            self.warmup_samples)
        self.runtime_manifest['warmup_pure_inference_first_20_ms'] = (
            pure_samples[:20])
        self.runtime_manifest['warmup_steady_last_10_median_ms'] = (
            float(np.median(steady_samples)) if steady_samples else None)
        self.write_runtime_report()
        self.get_logger().info(
            f'{action} YOLO model: '
            f'mode={self.torch_compile_mode}, '
            f'device={self.runtime_manifest.get("device")}, '
            f'backend={self.runtime_manifest.get("backend")}, '
            f'model_dtype='
            f'{self.runtime_manifest.get("model_parameter_dtype")}, '
            f'input_dtype={self.runtime_manifest.get("input_tensor_dtype")}, '
            f'input={width}x{height}, '
            f'warmups={warmup_iterations}, '
            f'startup_time={time.monotonic() - compile_start:.1f}s')

    def install_preprocess_profiler_if_ready(self):
        """
        Wrap Ultralytics 8.3.91 preprocessing only while profiling.

        The wrapped operations are intentionally identical to the installed
        BasePredictor.preprocess implementation.  CUDA events are resolved
        only after ``self.model`` returns, avoiding asynchronous H2D timing
        errors without changing any input pixels or inference parameters.
        """
        if self.pipeline_timing_pub is None:
            return
        if self.preprocess_profiler_installed:
            return
        predictor = getattr(self.model, 'predictor', None)
        if predictor is None:
            return

        import torch

        node = self

        def profiled_preprocess(predictor_self, images):
            profile = {}
            not_tensor = not isinstance(images, torch.Tensor)
            if not_tensor:
                started = time.perf_counter_ns()
                images = np.stack(predictor_self.pre_transform(images))
                profile[f'{node.profile_prefix}_letterbox'] = (
                    time.perf_counter_ns() - started) * 1e-6

                started = time.perf_counter_ns()
                images = images[..., ::-1].transpose((0, 3, 1, 2))
                profile[f'{node.profile_prefix}_color_layout'] = (
                    time.perf_counter_ns() - started) * 1e-6

                started = time.perf_counter_ns()
                images = np.ascontiguousarray(images)
                images = torch.from_numpy(images)
                profile[f'{node.profile_prefix}_tensor_build'] = (
                    time.perf_counter_ns() - started) * 1e-6

            is_cuda = getattr(predictor_self.device, 'type', '') == 'cuda'
            if is_cuda:
                h2d_start = torch.cuda.Event(enable_timing=True)
                h2d_end = torch.cuda.Event(enable_timing=True)
                h2d_start.record()
                images = images.to(predictor_self.device)
                h2d_end.record()
                profile[f'_{node.profile_prefix}_h2d_events'] = (
                    h2d_start, h2d_end)
            else:
                started = time.perf_counter_ns()
                images = images.to(predictor_self.device)
                profile[f'{node.profile_prefix}_h2d'] = (
                    time.perf_counter_ns() - started) * 1e-6

            if is_cuda:
                normalize_start = torch.cuda.Event(enable_timing=True)
                normalize_end = torch.cuda.Event(enable_timing=True)
                normalize_start.record()
            else:
                started = time.perf_counter_ns()
            images = (
                images.half() if predictor_self.model.fp16
                else images.float()
            )
            if not_tensor:
                images /= 255
            if is_cuda:
                normalize_end.record()
                profile[f'_{node.profile_prefix}_device_normalize_events'] = (
                    normalize_start, normalize_end)
            else:
                profile[f'{node.profile_prefix}_device_normalize'] = (
                    time.perf_counter_ns() - started) * 1e-6
            node.last_preprocess_profile = profile
            return images

        predictor.preprocess = MethodType(profiled_preprocess, predictor)
        self.preprocess_profiler_installed = True

    def resolve_preprocess_profile(self):
        profile = dict(self.last_preprocess_profile)
        for suffix in ('h2d', 'device_normalize'):
            stage = f'{self.profile_prefix}_{suffix}'
            events = profile.pop(f'_{stage}_events', None)
            if events is None:
                continue
            start_event, end_event = events
            end_event.synchronize()
            profile[stage] = float(start_event.elapsed_time(end_event))
        return profile

    def append_stage_timing(self, stage, start_ns, end_ns, compute_ms):
        self.last_stage_timings.append({
            'stage': str(stage),
            'start_ns': int(start_ns),
            'end_ns': int(end_ns),
            'compute_ms': float(compute_ms),
        })

    @staticmethod
    def calculate_iou(box1, box2):
        inter_xmin = max(box1.xmin, box2.xmin)
        inter_ymin = max(box1.ymin, box2.ymin)
        inter_xmax = min(box1.xmax, box2.xmax)
        inter_ymax = min(box1.ymax, box2.ymax)
        inter_area = max(0, inter_xmax - inter_xmin) * max(
            0, inter_ymax - inter_ymin)
        area1 = max(0, box1.xmax - box1.xmin) * max(
            0, box1.ymax - box1.ymin)
        area2 = max(0, box2.xmax - box2.xmin) * max(
            0, box2.ymax - box2.ymin)
        union = area1 + area2 - inter_area
        return float(inter_area) / float(union) if union > 0 else 0.0

    def apply_class_based_nms(self, detections):
        grouped = defaultdict(list)
        for detection in detections:
            grouped[detection.class_name].append(detection)
        final_detections = []
        for detections_in_class in grouped.values():
            remaining = sorted(
                detections_in_class,
                key=lambda item: item.confidence,
                reverse=True,
            )
            while remaining:
                best = remaining.pop(0)
                final_detections.append(best)
                remaining = [
                    item for item in remaining
                    if self.calculate_iou(best, item) < self.iou_threshold
                ]
        return final_detections

    def image_callback(self, msg: Image):
        now = time.monotonic()
        if not self.first_image_logged:
            self.first_image_logged = True
            self.get_logger().info(
                f'[STARTUP +{now - self.startup_monotonic_s:.3f}s] '
                f'{self.profile_prefix} first input image')
        receive_stamp_ns = (
            self.get_clock().now().nanoseconds
            if self.pipeline_timing_pub is not None else 0
        )
        with self.latest_image_lock:
            if self.last_received_monotonic is not None:
                self.max_input_gap_s = max(
                    self.max_input_gap_s,
                    now - self.last_received_monotonic,
                )
            self.last_received_monotonic = now
            self.received_frames += 1
            if self.latest_image is not None:
                self.replaced_frames += 1
            self.latest_image = (msg, now, receive_stamp_ns)
            self.new_image_event.set()

    def inference_worker(self):
        period_s = (
            1.0 / self.max_inference_rate_hz
            if self.max_inference_rate_hz > 0.0 else 0.0
        )
        next_allowed_monotonic = 0.0
        while not self.worker_stop_event.is_set():
            if not self.new_image_event.wait(timeout=0.1):
                continue
            if self.worker_stop_event.is_set():
                break

            wait_s = next_allowed_monotonic - time.monotonic()
            if wait_s > 0.0 and self.worker_stop_event.wait(wait_s):
                break
            with self.latest_image_lock:
                item = self.latest_image
                self.latest_image = None
                self.new_image_event.clear()
            if item is None:
                continue

            msg, received_monotonic, receive_stamp_ns = item
            inference_start = time.monotonic()
            inference_start_ns = (
                self.get_clock().now().nanoseconds
                if self.pipeline_timing_pub is not None else 0
            )
            next_allowed_monotonic = inference_start + period_s
            queue_age_ms = (inference_start - received_monotonic) * 1000.0
            self.last_model_timing = None
            self.last_stage_timings = []
            self.last_preprocess_profile = {}
            try:
                success = self.process_image(msg)
            except Exception as exc:
                success = False
                self.get_logger().error(f'YOLO inference failed: {exc}')
            inference_ms = (time.monotonic() - inference_start) * 1000.0
            inference_end_ns = (
                self.get_clock().now().nanoseconds
                if self.pipeline_timing_pub is not None else 0
            )
            self.inference_samples_ms.append(inference_ms)
            self.queue_age_samples_ms.append(queue_age_ms)
            if success:
                now = time.monotonic()
                if self.last_processed_monotonic is not None:
                    self.max_output_gap_s = max(
                        self.max_output_gap_s,
                        now - self.last_processed_monotonic,
                    )
                self.last_processed_monotonic = now
                self.processed_frames += 1
                source_age = self.message_age_ms(msg)
                if np.isfinite(source_age):
                    self.end_to_end_samples_ms.append(source_age)
                if not self.first_inference_logged:
                    self.first_inference_logged = True
                    age_text = (
                        f'{source_age:.1f}ms'
                        if np.isfinite(source_age) else 'unavailable')
                    self.get_logger().info(
                        f'[STARTUP +{now - self.startup_monotonic_s:.3f}s] '
                        f'{self.profile_prefix} first inference published: '
                        f'inference={inference_ms:.1f}ms, '
                        f'queue={queue_age_ms:.1f}ms, source_age={age_text}')
                if self.pipeline_timing_pub is not None:
                    timing = PipelineTiming()
                    timing.header = msg.header
                    timing.stage = self.profile_prefix
                    timing.receive_stamp_ns = int(receive_stamp_ns)
                    timing.start_stamp_ns = int(inference_start_ns)
                    timing.end_stamp_ns = int(inference_end_ns)
                    timing.queue_wait_ms = float(queue_age_ms)
                    timing.compute_ms = float(inference_ms)
                    timing.received_count = int(self.received_frames)
                    timing.processed_count = int(self.processed_frames)
                    timing.replaced_count = int(self.replaced_frames)
                    timing.input_reused = False
                    self.pipeline_timing_pub.publish(timing)
                    if self.last_model_timing is not None:
                        model_start_ns, model_end_ns, model_ms = (
                            self.last_model_timing)
                        model_timing = PipelineTiming()
                        model_timing.header = msg.header
                        model_timing.stage = f'{self.profile_prefix}_model'
                        model_timing.receive_stamp_ns = int(model_start_ns)
                        model_timing.start_stamp_ns = int(model_start_ns)
                        model_timing.end_stamp_ns = int(model_end_ns)
                        model_timing.queue_wait_ms = 0.0
                        model_timing.compute_ms = float(model_ms)
                        model_timing.received_count = int(
                            self.received_frames)
                        model_timing.processed_count = int(
                            self.processed_frames)
                        model_timing.replaced_count = int(
                            self.replaced_frames)
                        model_timing.input_reused = False
                        self.pipeline_timing_pub.publish(model_timing)
                    for stage_profile in self.last_stage_timings:
                        stage_timing = PipelineTiming()
                        stage_timing.header = msg.header
                        stage_timing.stage = stage_profile['stage']
                        stage_timing.receive_stamp_ns = int(
                            stage_profile['start_ns'])
                        stage_timing.start_stamp_ns = int(
                            stage_profile['start_ns'])
                        stage_timing.end_stamp_ns = int(
                            stage_profile['end_ns'])
                        stage_timing.queue_wait_ms = 0.0
                        stage_timing.compute_ms = float(
                            stage_profile['compute_ms'])
                        stage_timing.received_count = int(
                            self.received_frames)
                        stage_timing.processed_count = int(
                            self.processed_frames)
                        stage_timing.replaced_count = int(
                            self.replaced_frames)
                        stage_timing.input_reused = False
                        self.pipeline_timing_pub.publish(stage_timing)
            self.log_performance_if_due()

    def confidence_threshold_for(self, class_name: str) -> float:
        if class_name == 'obstacle_vehicle':
            return self.obstacle_conf_threshold
        if class_name == 'cone':
            return self.cone_conf_threshold
        return self.general_conf_threshold

    def process_image(self, msg: Image) -> bool:
        profiling = self.pipeline_timing_pub is not None
        prefix = self.profile_prefix

        def begin_stage():
            if not profiling:
                return None
            return (
                time.perf_counter_ns(),
                self.get_clock().now().nanoseconds,
            )

        def finish_stage(stage, started):
            if started is None:
                return
            end_perf_ns = time.perf_counter_ns()
            end_ns = self.get_clock().now().nanoseconds
            self.append_stage_timing(
                stage,
                started[1],
                end_ns,
                (end_perf_ns - started[0]) * 1e-6,
            )

        cv_bridge_started = begin_stage()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        finish_stage(f'{prefix}_cv_bridge', cv_bridge_started)
        if profiling:
            self.install_preprocess_profiler_if_ready()
            model_start_monotonic = time.monotonic()
            model_start_ns = self.get_clock().now().nanoseconds
        results = self.predict_frame(frame)
        if profiling:
            model_end_monotonic = time.monotonic()
            model_end_ns = self.get_clock().now().nanoseconds
            self.last_model_timing = (
                model_start_ns,
                model_end_ns,
                (model_end_monotonic - model_start_monotonic) * 1000.0,
            )
            # On the first non-warmed call the predictor is created by
            # self.model(), so detailed host/H2D profiling starts next frame.
            self.install_preprocess_profiler_if_ready()
            speed = getattr(results[0], 'speed', {}) or {}
            cursor_ns = model_start_ns
            for speed_name, stage_name in (
                ('preprocess', f'{prefix}_preprocess'),
                ('inference', f'{prefix}_inference'),
                ('postprocess', f'{prefix}_ultralytics_postprocess'),
            ):
                compute_ms = float(speed.get(speed_name, 0.0))
                stage_end_ns = cursor_ns + int(compute_ms * 1_000_000.0)
                self.append_stage_timing(
                    stage_name, cursor_ns, stage_end_ns, compute_ms)
                cursor_ns = stage_end_ns
            for stage_name, compute_ms in (
                    self.resolve_preprocess_profile().items()):
                self.append_stage_timing(
                    stage_name,
                    model_start_ns,
                    model_end_ns,
                    compute_ms,
                )

        detection_build_started = begin_stage()
        candidates = []
        boxes = results[0].boxes
        if boxes is not None:
            for box in boxes:
                class_id = int(box.cls[0])
                if class_id not in self.inference_class_ids:
                    continue
                source_class_name = self.model_names[class_id]
                class_name = canonical_class_name(
                    source_class_name, self.class_name_aliases)
                confidence = float(box.conf[0])
                if confidence < self.confidence_threshold_for(class_name):
                    continue
                coords = box.xyxy[0].cpu().numpy().astype(int)
                detection = Detection()
                detection.class_name = class_name
                detection.confidence = confidence
                detection.xmin = int(coords[0])
                detection.ymin = int(coords[1])
                detection.xmax = int(coords[2])
                detection.ymax = int(coords[3])
                candidates.append(detection)
        finish_stage(
            f'{prefix}_detection_build', detection_build_started)

        custom_nms_started = begin_stage()
        final_detections = self.apply_class_based_nms(candidates)
        finish_stage(f'{prefix}_custom_nms', custom_nms_started)
        message_started = begin_stage()
        output = Detections()
        output.header = msg.header
        output.detections = final_detections
        finish_stage(f'{prefix}_message_build', message_started)
        if self.worker_stop_event.is_set():
            return False
        detection_publish_started = begin_stage()
        self.detection_publisher.publish(output)
        finish_stage(
            f'{prefix}_detection_publish', detection_publish_started)
        self.output_timestamps_monotonic.append(time.monotonic())
        debug_started = begin_stage()
        if self.source_image_publisher is not None:
            self.source_image_publisher.publish(msg)
        self.publish_annotation_if_due(frame, msg, final_detections)
        finish_stage(f'{prefix}_debug_output', debug_started)
        return True

    def publish_annotation_if_due(self, frame, source_msg, detections):
        if self.annotated_image_publisher is None:
            return
        now = time.monotonic()
        if (
            self.annotated_period_s > 0.0
            and now - self.last_annotated_monotonic < self.annotated_period_s
        ):
            return
        annotated = frame.copy()
        for detection in detections:
            source_class_name = next(
                (
                    source for source, canonical
                    in self.class_name_aliases.items()
                    if canonical == detection.class_name
                ),
                detection.class_name,
            )
            class_id = next(
                (
                    key for key, value in self.model_names.items()
                    if value == source_class_name
                ),
                0,
            )
            color = (
                int((37 * class_id + 80) % 255),
                int((97 * class_id + 40) % 255),
                int((157 * class_id + 120) % 255),
            )
            cv2.rectangle(
                annotated,
                (int(detection.xmin), int(detection.ymin)),
                (int(detection.xmax), int(detection.ymax)),
                color,
                2,
            )
            label_parts = []
            if self.show_labels:
                label_parts.append(detection.class_name)
            if self.show_conf:
                label_parts.append(f'{detection.confidence:.2f}')
            if label_parts:
                cv2.putText(
                    annotated,
                    ' '.join(label_parts),
                    (int(detection.xmin), max(15, int(detection.ymin) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                    cv2.LINE_AA,
                )
        cv2.putText(
            annotated,
            f'Inference: {self.current_inference_hz():.1f} Hz',
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        output.header = source_msg.header
        self.annotated_image_publisher.publish(output)
        if self.show_window:
            with self.visualization_lock:
                self.latest_visualization = annotated
        self.last_annotated_monotonic = now

    def display_latest_visualization(self):
        with self.visualization_lock:
            annotated = self.latest_visualization
            self.latest_visualization = None
        if annotated is not None:
            cv2.imshow(self.window_name, annotated)
        cv2.waitKey(1)

    def current_inference_hz(self) -> float:
        if len(self.output_timestamps_monotonic) < 2:
            return 0.0
        elapsed = (
            self.output_timestamps_monotonic[-1]
            - self.output_timestamps_monotonic[0]
        )
        if elapsed <= 0.0:
            return 0.0
        return (len(self.output_timestamps_monotonic) - 1) / elapsed

    def message_age_ms(self, msg: Image) -> float:
        source_ns = stamp_to_ns(msg.header.stamp)
        if source_ns <= 0:
            return float('nan')
        age_ms = (self.get_clock().now().nanoseconds - source_ns) * 1e-6
        if age_ms < -1000.0 or age_ms > 86_400_000.0:
            return float('nan')
        return age_ms

    @staticmethod
    def percentile(values, percentile):
        if not values:
            return float('nan')
        return float(np.percentile(np.asarray(values), percentile))

    def log_performance_if_due(self):
        if not self.publish_stats or not rclpy.ok():
            return
        now = time.monotonic()
        elapsed = now - self.last_performance_monotonic
        if elapsed < self.performance_log_period_s:
            return
        counts = (self.received_frames, self.processed_frames)
        input_rate = (counts[0] - self.last_performance_counts[0]) / elapsed
        output_rate = (counts[1] - self.last_performance_counts[1]) / elapsed
        self.get_logger().info(
            'YOLO performance: '
            f'input={input_rate:.1f}Hz output={output_rate:.1f}Hz '
            f'received={counts[0]} processed={counts[1]} '
            f'replaced={self.replaced_frames} '
            f'inference_p50/p95='
            f'{self.percentile(self.inference_samples_ms, 50):.1f}/'
            f'{self.percentile(self.inference_samples_ms, 95):.1f}ms '
            f'queue_p95='
            f'{self.percentile(self.queue_age_samples_ms, 95):.1f}ms '
            f'e2e_p50/p95='
            f'{self.percentile(self.end_to_end_samples_ms, 50):.1f}/'
            f'{self.percentile(self.end_to_end_samples_ms, 95):.1f}ms '
            f'max_gap(input/output)='
            f'{self.max_input_gap_s * 1000.0:.1f}/'
            f'{self.max_output_gap_s * 1000.0:.1f}ms'
        )
        self.last_performance_monotonic = now
        self.last_performance_counts = counts
        self.max_input_gap_s = 0.0
        self.max_output_gap_s = 0.0

    def stop_worker(self):
        if self.worker_thread is None:
            return
        self.worker_stop_event.set()
        self.new_image_event.set()
        self.worker_thread.join(timeout=5.0)
        if self.worker_thread.is_alive():
            self.get_logger().warn(
                'YOLO inference worker did not stop within five seconds')
        self.worker_thread = None

    def destroy_node(self):
        if self.visualization_timer is not None:
            self.visualization_timer.cancel()
        self.stop_worker()
        if self.show_window:
            cv2.destroyWindow(self.window_name)
        return super().destroy_node()


# Compatibility with code that imported the old class name.
Yolo_Node = YoloNode


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if rclpy.ok():
            if node is not None:
                node.get_logger().error(f'YOLO node terminated: {exc}')
            else:
                print(f'YOLO node failed to initialize: {exc}')
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
