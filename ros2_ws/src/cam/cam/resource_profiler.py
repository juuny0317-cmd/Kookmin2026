"""
Low-rate CPU profiler for timestamp-aligned YOLO measurements.

The node is disabled by default in the integrated launch.  When enabled it
writes three CSV files without publishing a high-rate ROS topic:

* the requested YOLO process plus system/per-core CPU and CPU frequency;
* a process table that exposes ROS and other CPU contenders;
* per-thread CPU for the target YOLO process.

All percentages use interval deltas from ``/proc``.  A process or thread at
100 percent occupies one logical CPU; total/per-core system utilization is in
the conventional 0..100 percent range.
"""

from __future__ import annotations

import csv
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


ROLE_PATTERNS = (
    ('bag_player', ('ros2 bag play', 'rosbag2_player')),
    ('bag_recorder', ('ros2 bag record', 'rosbag2_recorder')),
    ('usb_cam', ('usb_cam_node_exe',)),
    ('frame_router', ('frame_router',)),
    ('lane_yolo', ('lane_yolo_node',)),
    ('scene_yolo', ('scene_yolo_node',)),
    ('centerlane', ('centerlane_tracer',)),
    ('lane_detector', ('lane_detector',)),
    ('stanley', ('integrated_stanley_controller',)),
    ('mission_manager', ('mission_manager_node',)),
    ('resource_profiler', ('resource_profiler',)),
)


def number_from_status(status, key, default=0):
    """Return the first integer in one ``/proc/*/status`` field."""
    try:
        return int(status.get(key, str(default)).split()[0])
    except (TypeError, ValueError, IndexError):
        return default


def read_status(path):
    result = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        if ':' not in line:
            continue
        key, value = line.split(':', 1)
        result[key] = value.strip()
    return result


def read_io(path):
    result = {}
    try:
        for line in path.read_text(encoding='utf-8').splitlines():
            if ':' not in line:
                continue
            key, value = line.split(':', 1)
            result[key] = int(value.strip())
    except (FileNotFoundError, PermissionError, ValueError):
        pass
    return result


def parse_stat(path):
    """Parse fields needed from ``stat`` while allowing spaces in comm."""
    raw = path.read_text(encoding='utf-8')
    close = raw.rfind(')')
    command_name = raw[raw.find('(') + 1:close]
    fields = raw[close + 2:].split()
    ticks = os.sysconf(os.sysconf_names['SC_CLK_TCK'])
    page_size = os.sysconf('SC_PAGE_SIZE')
    return {
        'name': command_name,
        'cpu_s': (int(fields[11]) + int(fields[12])) / ticks,
        'minor_faults': int(fields[7]),
        'major_faults': int(fields[9]),
        'nice': int(fields[16]),
        'threads': int(fields[17]),
        'start_ticks': int(fields[19]),
        'rss_mib': int(fields[21]) * page_size / (1024.0 * 1024.0),
        'last_cpu': int(fields[36]) if len(fields) > 36 else None,
    }


def process_role(command):
    for role, patterns in ROLE_PATTERNS:
        if any(pattern in command for pattern in patterns):
            return role
    return 'other'


def read_process(entry):
    stat = parse_stat(entry / 'stat')
    raw_argv = (entry / 'cmdline').read_bytes().split(b'\0')
    argv = [value.decode(errors='replace') for value in raw_argv if value]
    command = ' '.join(argv)
    role = process_role(command)
    status = {}
    io_values = {}
    if role != 'other':
        status = read_status(entry / 'status')
        io_values = read_io(entry / 'io')
    return {
        **stat,
        'pid': int(entry.name),
        'argv': argv,
        'command': command,
        'role': role,
        'threads': number_from_status(
            status, 'Threads', stat['threads']),
        'voluntary_ctxt': number_from_status(
            status, 'voluntary_ctxt_switches'),
        'involuntary_ctxt': number_from_status(
            status, 'nonvoluntary_ctxt_switches'),
        'read_bytes': int(io_values.get('read_bytes', 0)),
        'write_bytes': int(io_values.get('write_bytes', 0)),
        'affinity': status.get('Cpus_allowed_list', ''),
    }


def process_snapshot(pids=None):
    """Read all processes for discovery, or only cached PIDs for sampling."""
    result = {}
    if pids is None:
        entries = Path('/proc').iterdir()
    else:
        entries = (Path('/proc') / str(pid) for pid in pids)
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            item = read_process(entry)
            result[item['pid']] = item
        except (FileNotFoundError, PermissionError, ProcessLookupError,
                ValueError, IndexError):
            continue
    return result


def read_thread(entry):
    stat = parse_stat(entry / 'stat')
    status = read_status(entry / 'status')
    return {
        **stat,
        'tid': int(entry.name),
        'voluntary_ctxt': number_from_status(
            status, 'voluntary_ctxt_switches'),
        'involuntary_ctxt': number_from_status(
            status, 'nonvoluntary_ctxt_switches'),
    }


def thread_snapshot(pid):
    result = {}
    if pid is None:
        return result
    task_path = Path('/proc') / str(pid) / 'task'
    try:
        entries = list(task_path.iterdir())
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return result
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            item = read_thread(entry)
            result[item['tid']] = item
        except (FileNotFoundError, PermissionError, ProcessLookupError,
                ValueError, IndexError):
            continue
    return result


def find_process(snapshot, pattern, excluded_pid):
    matches = []
    for pid, item in snapshot.items():
        if pid == excluded_pid or pattern not in item['command']:
            continue
        remap = f'__node:={pattern}'
        exact_ros_name = int(remap in item['argv'])
        direct_executable = int(bool(item['argv']) and (
            Path(item['argv'][0]).name == pattern))
        score = (exact_ros_name, direct_executable, item['rss_mib'], pid)
        matches.append((score, pid))
    return max(matches)[1] if matches else None


def read_cpu_snapshot():
    result = {}
    for line in Path('/proc/stat').read_text(
            encoding='utf-8').splitlines():
        parts = line.split()
        if not parts or not parts[0].startswith('cpu'):
            if result:
                break
            continue
        if parts[0] != 'cpu' and not parts[0][3:].isdigit():
            continue
        values = [int(value) for value in parts[1:]]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        result[parts[0]] = (sum(values), idle)
    return result


def cpu_percent(previous, current):
    if previous is None or current is None:
        return None
    total_delta = current[0] - previous[0]
    idle_delta = current[1] - previous[1]
    if total_delta <= 0:
        return None
    return (total_delta - idle_delta) / total_delta * 100.0


def read_cpu_frequencies():
    frequencies = {}
    for index in range(os.cpu_count() or 1):
        path = Path('/sys/devices/system/cpu') / f'cpu{index}' / (
            'cpufreq/scaling_cur_freq')
        try:
            frequencies[index] = float(path.read_text().strip()) / 1000.0
        except (FileNotFoundError, PermissionError, ValueError):
            continue
    if frequencies:
        return frequencies
    cpu_index = None
    try:
        for line in Path('/proc/cpuinfo').read_text(
                encoding='utf-8').splitlines():
            if line.startswith('processor'):
                cpu_index = int(line.split(':', 1)[1])
            elif line.startswith('cpu MHz') and cpu_index is not None:
                frequencies[cpu_index] = float(line.split(':', 1)[1])
    except (FileNotFoundError, PermissionError, ValueError):
        pass
    return frequencies


def read_cpu_pressure():
    result = {}
    try:
        for line in Path('/proc/pressure/cpu').read_text(
                encoding='utf-8').splitlines():
            parts = line.split()
            category = parts[0]
            for item in parts[1:]:
                key, value = item.split('=', 1)
                result[f'{category}_{key}'] = float(value)
    except (FileNotFoundError, PermissionError, ValueError):
        pass
    return result


def read_memory_snapshot():
    """Return system memory and swap counters from ``/proc/meminfo``."""
    values = {}
    try:
        for line in Path('/proc/meminfo').read_text(
                encoding='utf-8').splitlines():
            if ':' not in line:
                continue
            key, raw = line.split(':', 1)
            parts = raw.split()
            if parts:
                values[key] = int(parts[0]) * 1024
    except (FileNotFoundError, PermissionError, ValueError):
        return {}
    total = values.get('MemTotal', 0)
    available = values.get('MemAvailable', 0)
    swap_total = values.get('SwapTotal', 0)
    swap_free = values.get('SwapFree', 0)
    return {
        'memory_total_mib': total / (1024.0 * 1024.0),
        'memory_available_mib': available / (1024.0 * 1024.0),
        'memory_used_pct': (
            (total - available) / total * 100.0 if total > 0 else None),
        'swap_total_mib': swap_total / (1024.0 * 1024.0),
        'swap_used_mib': max(0, swap_total - swap_free) / (
            1024.0 * 1024.0),
    }


def read_disk_snapshot():
    """Aggregate physical block-device sectors and busy milliseconds."""
    result = {'read_sectors': 0, 'write_sectors': 0, 'busy_ms': 0}
    found = False
    try:
        lines = Path('/proc/diskstats').read_text(
            encoding='utf-8').splitlines()
    except (FileNotFoundError, PermissionError):
        return {}
    for line in lines:
        fields = line.split()
        if len(fields) < 14:
            continue
        name = fields[2]
        sys_path = Path('/sys/class/block') / name
        if not sys_path.exists() or (sys_path / 'partition').exists():
            continue
        if name.startswith(('loop', 'ram', 'fd', 'sr')):
            continue
        try:
            result['read_sectors'] += int(fields[5])
            result['write_sectors'] += int(fields[9])
            result['busy_ms'] += int(fields[12])
            found = True
        except ValueError:
            continue
    return result if found else {}


def same_process(left, right):
    return bool(left and right and (
        left['start_ticks'] == right['start_ticks']))


def delta_rate(current, previous, field, elapsed_s, scale=1.0):
    if not same_process(current, previous) or elapsed_s <= 0.0:
        return None
    return max(0.0, current[field] - previous[field]) / elapsed_s / scale


class ResourceProfiler(Node):
    """Sample the target YOLO and its CPU competitors at a low fixed rate."""

    FIELDS = (
        'timestamp_ns', 'monotonic_ns', 'sample_index', 'sample_interval_ms',
        'target_pid', 'target_command', 'target_affinity', 'target_nice',
        'target_last_cpu', 'process_cpu_pct_one_core_100',
        'process_cpu_pct_total_capacity', 'process_rss_mib',
        'process_thread_count', 'process_voluntary_ctxt_switches_s',
        'process_involuntary_ctxt_switches_s', 'process_minor_faults_s',
        'process_major_faults_s', 'process_read_mib_s',
        'process_write_mib_s', 'total_cpu_pct', 'busiest_core',
        'busiest_core_cpu_pct', 'per_core_cpu_pct_json',
        'cpu_frequency_avg_mhz', 'cpu_frequency_min_mhz',
        'cpu_frequency_max_mhz', 'per_core_frequency_mhz_json',
        'cpu_pressure_some_avg10', 'cpu_pressure_some_total_us',
        'load1', 'load5', 'load15', 'logical_cpu_count',
        'memory_total_mib', 'memory_available_mib', 'memory_used_pct',
        'swap_total_mib', 'swap_used_mib', 'disk_read_mib_s',
        'disk_write_mib_s', 'disk_busy_pct',
        'sample_collect_ms', 'previous_callback_wall_ms', 'error',
    )
    PROCESS_FIELDS = (
        'timestamp_ns', 'monotonic_ns', 'sample_index', 'pid', 'role',
        'cpu_pct_one_core_100', 'cpu_pct_total_capacity', 'rss_mib',
        'thread_count', 'read_mib_s', 'write_mib_s', 'affinity', 'nice',
        'last_cpu', 'command',
    )
    THREAD_FIELDS = (
        'timestamp_ns', 'monotonic_ns', 'sample_index', 'target_pid',
        'tid', 'name', 'cpu_pct_one_core_100',
        'voluntary_ctxt_switches_s', 'involuntary_ctxt_switches_s',
        'last_cpu',
    )

    def __init__(self):
        super().__init__('resource_profiler')
        self.declare_parameter('output_path', '/tmp/xycar_resource_stats.csv')
        self.declare_parameter('sample_rate_hz', 2.0)
        self.declare_parameter('target_process_pattern', 'lane_yolo_node')
        self.declare_parameter('process_table_top_n', 20)
        self.declare_parameter('profile_target_threads', False)
        # Accepted for compatibility with the retired NVIDIA profiler.  No
        # NVIDIA library or device is probed by this CPU-only implementation.
        self.declare_parameter('gpu_index', 0)

        output = Path(str(
            self.get_parameter('output_path').value)).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = output.open('w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.stream, fieldnames=self.FIELDS)
        self.writer.writeheader()

        process_path = output.with_name(
            f'{output.stem}_processes{output.suffix or ".csv"}')
        self.process_stream = process_path.open(
            'w', newline='', encoding='utf-8')
        self.process_writer = csv.DictWriter(
            self.process_stream, fieldnames=self.PROCESS_FIELDS)
        self.process_writer.writeheader()

        thread_path = output.with_name(
            f'{output.stem}_threads{output.suffix or ".csv"}')
        self.thread_stream = thread_path.open(
            'w', newline='', encoding='utf-8')
        self.thread_writer = csv.DictWriter(
            self.thread_stream, fieldnames=self.THREAD_FIELDS)
        self.thread_writer.writeheader()
        self.stream.flush()
        self.process_stream.flush()
        self.thread_stream.flush()

        self.target_pattern = str(
            self.get_parameter('target_process_pattern').value)
        self.process_top_n = max(
            0, int(self.get_parameter('process_table_top_n').value))
        self.profile_threads = bool(
            self.get_parameter('profile_target_threads').value)
        self.target_pid = None
        self.tracked_pids = set()
        self.previous_processes = {}
        self.previous_threads = {}
        self.previous_cpu = None
        self.previous_disk = None
        self.previous_sample_monotonic_ns = None
        self.previous_callback_wall_ms = None
        self.sample_index = 0
        self.logical_cpu_count = max(1, os.cpu_count() or 1)
        self.refresh_service = self.create_service(
            Trigger, '/resource_profiler/refresh_processes',
            self.refresh_processes_callback)
        self.refresh_processes()

        rate_hz = max(0.2, min(
            10.0, float(self.get_parameter('sample_rate_hz').value)))
        self.timer = self.create_timer(1.0 / rate_hz, self.sample)
        self.get_logger().info(
            f'CPU resource profiler enabled: rate={rate_hz:.2f}Hz '
            f'target={self.target_pattern!r} output={output} '
            f'processes={process_path} threads={thread_path}')

    def refresh_processes(self):
        """Discover role processes outside the timed sampling callback."""
        discovered = process_snapshot()
        self.target_pid = find_process(
            discovered, self.target_pattern, os.getpid())
        self.tracked_pids = {
            pid for pid, item in discovered.items()
            if item['role'] != 'other'
        }
        if self.target_pid is not None:
            self.tracked_pids.add(self.target_pid)
        self.previous_processes = {}
        self.previous_threads = {}
        return len(self.tracked_pids)

    def refresh_processes_callback(self, request, response):
        """Refresh cached PIDs before starting a replay measurement."""
        del request
        started = time.perf_counter()
        count = self.refresh_processes()
        response.success = self.target_pid is not None
        response.message = (
            f'tracked={count} target_pid={self.target_pid} '
            f'discovery_ms={(time.perf_counter() - started) * 1000.0:.3f}')
        return response

    def process_rows(self, current, elapsed_s, stamp):
        candidates = []
        for pid, item in current.items():
            old = self.previous_processes.get(pid)
            cpu_pct = delta_rate(item, old, 'cpu_s', elapsed_s) * 100.0 \
                if same_process(item, old) and elapsed_s > 0.0 else None
            candidates.append((
                cpu_pct if cpu_pct is not None else -1.0,
                item['role'] != 'other',
                pid,
                item,
                old,
                cpu_pct,
            ))
        selected_pids = {
            item[2] for item in sorted(candidates, reverse=True)[
                :self.process_top_n]
        }
        selected_pids.update(
            item[2] for item in candidates if item[1])
        rows = []
        for _, _, pid, item, old, cpu_pct in candidates:
            if pid not in selected_pids:
                continue
            rows.append({
                **stamp,
                'pid': pid,
                'role': item['role'],
                'cpu_pct_one_core_100': cpu_pct,
                'cpu_pct_total_capacity': (
                    cpu_pct / self.logical_cpu_count
                    if cpu_pct is not None else None),
                'rss_mib': item['rss_mib'],
                'thread_count': item['threads'],
                'read_mib_s': delta_rate(
                    item, old, 'read_bytes', elapsed_s, 1024.0 * 1024.0),
                'write_mib_s': delta_rate(
                    item, old, 'write_bytes', elapsed_s, 1024.0 * 1024.0),
                'affinity': item['affinity'],
                'nice': item['nice'],
                'last_cpu': item['last_cpu'],
                'command': item['command'],
            })
        return rows

    def thread_rows(self, current, elapsed_s, stamp):
        rows = []
        for tid, item in current.items():
            old = self.previous_threads.get(tid)
            rows.append({
                **stamp,
                'target_pid': self.target_pid,
                'tid': tid,
                'name': item['name'],
                'cpu_pct_one_core_100': (
                    delta_rate(item, old, 'cpu_s', elapsed_s) * 100.0
                    if same_process(item, old) and elapsed_s > 0.0
                    else None),
                'voluntary_ctxt_switches_s': delta_rate(
                    item, old, 'voluntary_ctxt', elapsed_s),
                'involuntary_ctxt_switches_s': delta_rate(
                    item, old, 'involuntary_ctxt', elapsed_s),
                'last_cpu': item['last_cpu'],
            })
        return rows

    def sample(self):
        callback_started_ns = time.perf_counter_ns()
        monotonic_ns = time.monotonic_ns()
        timestamp_ns = self.get_clock().now().nanoseconds
        self.sample_index += 1
        elapsed_s = (
            (monotonic_ns - self.previous_sample_monotonic_ns) * 1e-9
            if self.previous_sample_monotonic_ns is not None else 0.0)
        stamp = {
            'timestamp_ns': timestamp_ns,
            'monotonic_ns': monotonic_ns,
            'sample_index': self.sample_index,
        }
        row = dict.fromkeys(self.FIELDS)
        row.update(stamp)
        row['sample_interval_ms'] = elapsed_s * 1000.0 if elapsed_s else None
        row['previous_callback_wall_ms'] = self.previous_callback_wall_ms
        row['logical_cpu_count'] = self.logical_cpu_count
        errors = []

        try:
            current_processes = process_snapshot(self.tracked_pids)
            if self.target_pid not in current_processes or (
                    self.target_pattern not in current_processes[
                        self.target_pid]['command']):
                self.refresh_processes()
                current_processes = process_snapshot(self.tracked_pids)
            target = current_processes.get(self.target_pid)
            old_target = self.previous_processes.get(self.target_pid)
            row['target_pid'] = self.target_pid
            if target is not None:
                process_cpu = (
                    delta_rate(target, old_target, 'cpu_s', elapsed_s) * 100.0
                    if same_process(target, old_target) and elapsed_s > 0.0
                    else None)
                row.update({
                    'target_command': target['command'],
                    'target_affinity': target['affinity'],
                    'target_nice': target['nice'],
                    'target_last_cpu': target['last_cpu'],
                    'process_cpu_pct_one_core_100': process_cpu,
                    'process_cpu_pct_total_capacity': (
                        process_cpu / self.logical_cpu_count
                        if process_cpu is not None else None),
                    'process_rss_mib': target['rss_mib'],
                    'process_thread_count': target['threads'],
                    'process_voluntary_ctxt_switches_s': delta_rate(
                        target, old_target, 'voluntary_ctxt', elapsed_s),
                    'process_involuntary_ctxt_switches_s': delta_rate(
                        target, old_target, 'involuntary_ctxt', elapsed_s),
                    'process_minor_faults_s': delta_rate(
                        target, old_target, 'minor_faults', elapsed_s),
                    'process_major_faults_s': delta_rate(
                        target, old_target, 'major_faults', elapsed_s),
                    'process_read_mib_s': delta_rate(
                        target, old_target, 'read_bytes', elapsed_s,
                        1024.0 * 1024.0),
                    'process_write_mib_s': delta_rate(
                        target, old_target, 'write_bytes', elapsed_s,
                        1024.0 * 1024.0),
                })
            process_rows = self.process_rows(
                current_processes, elapsed_s, stamp)
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            current_processes = {}
            process_rows = []
            errors.append(f'process:{exc}')

        try:
            current_threads = (
                thread_snapshot(self.target_pid)
                if self.profile_threads else {})
            thread_rows = self.thread_rows(current_threads, elapsed_s, stamp)
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            current_threads = {}
            thread_rows = []
            errors.append(f'thread:{exc}')

        try:
            current_cpu = read_cpu_snapshot()
            per_core = {}
            if self.previous_cpu is not None:
                row['total_cpu_pct'] = cpu_percent(
                    self.previous_cpu.get('cpu'), current_cpu.get('cpu'))
                for name, value in current_cpu.items():
                    if name == 'cpu':
                        continue
                    utilization = cpu_percent(
                        self.previous_cpu.get(name), value)
                    if utilization is not None:
                        per_core[int(name[3:])] = utilization
            row['per_core_cpu_pct_json'] = json.dumps(
                per_core, sort_keys=True, separators=(',', ':'))
            if per_core:
                busiest = max(per_core, key=per_core.get)
                row['busiest_core'] = busiest
                row['busiest_core_cpu_pct'] = per_core[busiest]
            row['load1'], row['load5'], row['load15'] = os.getloadavg()
            pressure = read_cpu_pressure()
            row['cpu_pressure_some_avg10'] = pressure.get('some_avg10')
            row['cpu_pressure_some_total_us'] = pressure.get('some_total')
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            current_cpu = {}
            errors.append(f'cpu:{exc}')

        try:
            row.update(read_memory_snapshot())
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            errors.append(f'memory:{exc}')

        try:
            current_disk = read_disk_snapshot()
            if self.previous_disk and current_disk and elapsed_s > 0.0:
                sector_mib = 512.0 / (1024.0 * 1024.0)
                row['disk_read_mib_s'] = max(
                    0, current_disk['read_sectors']
                    - self.previous_disk['read_sectors']) * sector_mib / elapsed_s
                row['disk_write_mib_s'] = max(
                    0, current_disk['write_sectors']
                    - self.previous_disk['write_sectors']) * sector_mib / elapsed_s
                row['disk_busy_pct'] = min(
                    100.0,
                    max(0, current_disk['busy_ms']
                        - self.previous_disk['busy_ms'])
                    / (elapsed_s * 1000.0) * 100.0,
                )
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            current_disk = {}
            errors.append(f'disk:{exc}')

        try:
            frequencies = read_cpu_frequencies()
            row['per_core_frequency_mhz_json'] = json.dumps(
                frequencies, sort_keys=True, separators=(',', ':'))
            if frequencies:
                values = list(frequencies.values())
                row['cpu_frequency_avg_mhz'] = sum(values) / len(values)
                row['cpu_frequency_min_mhz'] = min(values)
                row['cpu_frequency_max_mhz'] = max(values)
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            errors.append(f'frequency:{exc}')

        self.process_writer.writerows(process_rows)
        self.thread_writer.writerows(thread_rows)
        self.process_stream.flush()
        self.thread_stream.flush()
        row['sample_collect_ms'] = (
            time.perf_counter_ns() - callback_started_ns) * 1e-6
        row['error'] = '; '.join(errors)
        self.writer.writerow(row)
        self.stream.flush()

        self.previous_processes = current_processes
        self.previous_threads = current_threads
        self.previous_cpu = current_cpu
        self.previous_disk = current_disk
        self.previous_sample_monotonic_ns = monotonic_ns
        self.previous_callback_wall_ms = (
            time.perf_counter_ns() - callback_started_ns) * 1e-6

    def destroy_node(self):
        if self.timer is not None:
            self.timer.cancel()
        for attribute in ('stream', 'process_stream', 'thread_stream'):
            stream = getattr(self, attribute, None)
            if stream is not None:
                stream.close()
                setattr(self, attribute, None)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ResourceProfiler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
