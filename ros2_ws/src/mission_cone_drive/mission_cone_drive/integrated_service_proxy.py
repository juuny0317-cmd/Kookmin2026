"""Expose host ROS services for an integrated stack running in Docker."""

import ast
import json
import os
import re
import select
import subprocess
import termios
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


TRIGGER_RESPONSE_PATTERN = re.compile(
    r'success=(True|False),\s*message=((?:\'(?:\\.|[^\'])*\')|'
    r'(?:(?:"(?:\\.|[^"])*")))\)'
)
RESPONSE_PREFIX = 'XYCAR_TRIGGER_RESPONSE='
READY_MARKER = 'XYCAR_SERVICE_CLIENT_READY'


def parse_trigger_response(output):
    """Return the Trigger values printed by ``ros2 service call``."""
    for line in reversed(str(output).splitlines()):
        if not line.startswith(RESPONSE_PREFIX):
            continue
        payload = json.loads(line[len(RESPONSE_PREFIX):])
        return bool(payload['success']), str(payload['message'])
    match = TRIGGER_RESPONSE_PATTERN.search(str(output))
    if match is None:
        raise ValueError('Trigger response was not present in command output')
    success = match.group(1) == 'True'
    message = ast.literal_eval(match.group(2))
    return success, str(message)


class IntegratedServiceProxy(Node):
    """Forward host calls to the uniquely labelled runtime container."""

    def __init__(self):
        super().__init__('integrated_service_proxy')
        self.declare_parameter(
            'container_label', 'com.xycar.role=integrated_drive')
        self.declare_parameter(
            'public_start_service', '/start_integrated_drive')
        self.declare_parameter(
            'public_stop_service', '/stop_integrated_drive')
        self.declare_parameter(
            'public_pause_service', '/pause_integrated_drive')
        self.declare_parameter(
            'public_resume_service', '/resume_integrated_drive')
        self.declare_parameter(
            'public_toggle_pause_service', '/toggle_pause_integrated_drive')
        self.declare_parameter(
            'internal_start_service', '/_start_integrated_drive_internal')
        self.declare_parameter(
            'internal_stop_service', '/_stop_integrated_drive_internal')
        self.declare_parameter(
            'internal_pause_service', '/_pause_integrated_drive_internal')
        self.declare_parameter(
            'internal_resume_service', '/_resume_integrated_drive_internal')
        self.declare_parameter(
            'internal_toggle_pause_service',
            '/_toggle_pause_integrated_drive_internal',
        )
        self.declare_parameter('forward_timeout_s', 8.0)
        self.declare_parameter('enable_keyboard_pause', True)
        self.declare_parameter('keyboard_device', '/dev/tty')
        self.declare_parameter('key_release_quiet_s', 0.20)

        self.container_label = str(
            self.get_parameter('container_label').value).strip()
        self.internal_start_service = self.validate_service_name(
            self.get_parameter('internal_start_service').value)
        self.internal_stop_service = self.validate_service_name(
            self.get_parameter('internal_stop_service').value)
        self.internal_pause_service = self.validate_service_name(
            self.get_parameter('internal_pause_service').value)
        self.internal_resume_service = self.validate_service_name(
            self.get_parameter('internal_resume_service').value)
        self.internal_toggle_pause_service = self.validate_service_name(
            self.get_parameter('internal_toggle_pause_service').value)
        self.forward_timeout_s = max(
            1.0, float(self.get_parameter('forward_timeout_s').value))
        self.enable_keyboard_pause = self.as_bool(
            self.get_parameter('enable_keyboard_pause').value)
        self.keyboard_device = str(
            self.get_parameter('keyboard_device').value).strip()
        self.key_release_quiet_s = max(
            0.05,
            float(self.get_parameter('key_release_quiet_s').value),
        )
        self.forward_lock = threading.Lock()
        self.bridge_lock = threading.Lock()
        self.bridge_stop = threading.Event()
        self.bridge_process = None
        self.bridge_container_id = None
        self.keyboard_stop = threading.Event()
        self.keyboard_toggle_requested = threading.Event()
        self.keyboard_fd = None
        self.original_terminal_attributes = None
        self.keyboard_thread = None

        public_start = self.validate_service_name(
            self.get_parameter('public_start_service').value)
        public_stop = self.validate_service_name(
            self.get_parameter('public_stop_service').value)
        public_pause = self.validate_service_name(
            self.get_parameter('public_pause_service').value)
        public_resume = self.validate_service_name(
            self.get_parameter('public_resume_service').value)
        public_toggle_pause = self.validate_service_name(
            self.get_parameter('public_toggle_pause_service').value)
        self.start_service = self.create_service(
            Trigger, public_start, self.start_callback)
        self.stop_service = self.create_service(
            Trigger, public_stop, self.stop_callback)
        self.pause_service = self.create_service(
            Trigger, public_pause, self.pause_callback)
        self.resume_service = self.create_service(
            Trigger, public_resume, self.resume_callback)
        self.toggle_pause_service = self.create_service(
            Trigger, public_toggle_pause, self.toggle_pause_callback)
        self.get_logger().info(
            'Integrated service proxy ready: '
            f'{public_start} -> {self.internal_start_service}, '
            f'{public_stop} -> {self.internal_stop_service}, '
            f'{public_pause} -> {self.internal_pause_service}, '
            f'{public_resume} -> {self.internal_resume_service}, '
            f'{public_toggle_pause} -> '
            f'{self.internal_toggle_pause_service}')
        self.bridge_thread = threading.Thread(
            target=self.prewarm_bridge,
            name='integrated-service-bridge-prewarm',
            daemon=True,
        )
        self.bridge_thread.start()
        self.keyboard_timer = self.create_timer(
            0.05, self.poll_keyboard_toggle)
        if self.enable_keyboard_pause:
            self.start_keyboard_reader()

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    @staticmethod
    def validate_service_name(value):
        name = str(value).strip()
        if re.fullmatch(r'/[A-Za-z0-9_/]+', name) is None:
            raise ValueError(f'Invalid absolute ROS service name: {name!r}')
        return name

    def find_container(self):
        command = [
            'docker', 'ps', '--filter', f'label={self.container_label}',
            '--format', '{{.ID}}',
        ]
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or 'docker ps failed'
            raise RuntimeError(detail)
        container_ids = [
            line.strip() for line in result.stdout.splitlines()
            if line.strip()
        ]
        if not container_ids:
            raise RuntimeError('integrated-drive container is not running')
        if len(container_ids) != 1:
            raise RuntimeError(
                'multiple integrated-drive containers are running; '
                'stop the stale launch before retrying')
        return container_ids[0]

    def bridge_command(self, container_id):
        return [
            'docker', 'exec', '-i', container_id,
            '/opt/mission_overlay_entrypoint.sh',
            'python3', '-m',
            'mission_cone_drive.container_service_client',
            '--stdio-server',
            self.internal_start_service,
            self.internal_stop_service,
            self.internal_pause_service,
            self.internal_resume_service,
            self.internal_toggle_pause_service,
        ]

    def stop_bridge_locked(self):
        process = self.bridge_process
        self.bridge_process = None
        self.bridge_container_id = None
        if process is None:
            return
        try:
            if process.stdin is not None:
                process.stdin.close()
        except OSError:
            pass
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)

    @staticmethod
    def read_bridge_line(process, deadline):
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(
                    'internal service bridge exited with '
                    f'{process.returncode}')
            remaining = max(0.0, deadline - time.monotonic())
            ready, _, _ = select.select(
                [process.stdout], [], [], min(0.2, remaining))
            if ready:
                return process.stdout.readline().rstrip('\n')
        raise RuntimeError('internal service bridge timed out')

    def ensure_bridge_locked(self):
        process = self.bridge_process
        if process is not None and process.poll() is None:
            return process
        self.stop_bridge_locked()
        container_id = self.find_container()
        process = subprocess.Popen(
            self.bridge_command(container_id),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        deadline = time.monotonic() + self.forward_timeout_s
        details = []
        try:
            while True:
                line = self.read_bridge_line(process, deadline)
                if line == READY_MARKER:
                    self.bridge_process = process
                    self.bridge_container_id = container_id
                    return process
                if line:
                    details.append(line)
                    details = details[-8:]
        except Exception:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
            detail = '\n'.join(details)[-400:]
            if detail:
                self.get_logger().error(
                    f'Internal service bridge startup output: {detail}')
            raise

    def prewarm_bridge(self):
        """Discover ROS services before the operator calls START."""
        while not self.bridge_stop.is_set():
            try:
                with self.bridge_lock:
                    self.ensure_bridge_locked()
                self.get_logger().info(
                    'Internal drive-control service bridge prewarmed.')
                return
            except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
                if self.bridge_stop.wait(0.25):
                    return
                last_error = str(exc)
        if not self.bridge_stop.is_set():
            self.get_logger().warn(
                f'Internal service bridge prewarm stopped: {last_error}')

    def start_keyboard_reader(self):
        try:
            keyboard_fd = os.open(
                self.keyboard_device,
                os.O_RDONLY | os.O_NONBLOCK,
            )
            if not os.isatty(keyboard_fd):
                raise OSError(f'{self.keyboard_device} is not a terminal')
            original = termios.tcgetattr(keyboard_fd)
            updated = termios.tcgetattr(keyboard_fd)
            updated[3] &= ~(termios.ICANON | termios.ECHO)
            updated[6][termios.VMIN] = 0
            updated[6][termios.VTIME] = 0
            termios.tcsetattr(keyboard_fd, termios.TCSANOW, updated)
            termios.tcflush(keyboard_fd, termios.TCIFLUSH)
        except OSError as exc:
            try:
                os.close(keyboard_fd)
            except (OSError, UnboundLocalError):
                pass
            self.get_logger().warn(
                f'Spacebar pause unavailable: {exc}. '
                'Pause/resume ROS services remain available.')
            return

        self.keyboard_fd = keyboard_fd
        self.original_terminal_attributes = original
        self.keyboard_thread = threading.Thread(
            target=self.read_keyboard,
            name='integrated-drive-keyboard',
            daemon=True,
        )
        self.keyboard_thread.start()
        self.get_logger().info(
            'Spacebar pause/resume ready. '
            'Space is ignored while integrated drive is inactive.')

    def read_keyboard(self):
        armed = True
        last_space_s = 0.0
        while not self.keyboard_stop.is_set():
            ready, _, _ = select.select(
                [self.keyboard_fd], [], [], 0.05)
            now_s = time.monotonic()
            if not ready:
                if (
                    not armed
                    and now_s - last_space_s >= self.key_release_quiet_s
                ):
                    armed = True
                continue
            try:
                data = os.read(self.keyboard_fd, 64)
            except BlockingIOError:
                continue
            if b' ' not in data:
                continue
            last_space_s = now_s
            if armed:
                armed = False
                self.keyboard_toggle_requested.set()

    def poll_keyboard_toggle(self):
        if not self.keyboard_toggle_requested.is_set():
            return
        self.keyboard_toggle_requested.clear()
        response = self.forward(
            self.internal_toggle_pause_service,
            Trigger.Response(),
        )
        if response.success:
            self.get_logger().warn(f'[SPACEBAR] {response.message}')
        else:
            self.get_logger().warn(
                f'[SPACEBAR] Ignored: {response.message}')

    def request_bridge_locked(self, internal_service):
        process = self.ensure_bridge_locked()
        try:
            process.stdin.write(internal_service + '\n')
            process.stdin.flush()
        except (AttributeError, BrokenPipeError, OSError) as exc:
            self.stop_bridge_locked()
            raise RuntimeError(f'internal service bridge write failed: {exc}')
        deadline = time.monotonic() + self.forward_timeout_s
        details = []
        while True:
            line = self.read_bridge_line(process, deadline)
            if line.startswith(RESPONSE_PREFIX):
                return parse_trigger_response(line)
            if line:
                details.append(line)
                details = details[-8:]

    def forward(self, internal_service, response):
        if not self.forward_lock.acquire(blocking=False):
            response.success = False
            response.message = (
                'Another integrated drive request is in progress.')
            return response
        try:
            with self.bridge_lock:
                success, message = self.request_bridge_locked(
                    internal_service)
            response.success = success
            response.message = message
            if success:
                self.get_logger().info(
                    f'Forwarded {internal_service}: {message}')
            else:
                self.get_logger().warn(
                    f'Forwarded {internal_service}: {message}')
        except (
            OSError,
            subprocess.TimeoutExpired,
            RuntimeError,
            ValueError,
        ) as exc:
            response.success = False
            response.message = f'Integrated service forwarding failed: {exc}'
            self.get_logger().error(response.message)
        finally:
            self.forward_lock.release()
        return response

    def start_callback(self, request, response):
        del request
        return self.forward(self.internal_start_service, response)

    def stop_callback(self, request, response):
        del request
        return self.forward(self.internal_stop_service, response)

    def pause_callback(self, request, response):
        del request
        return self.forward(self.internal_pause_service, response)

    def resume_callback(self, request, response):
        del request
        return self.forward(self.internal_resume_service, response)

    def toggle_pause_callback(self, request, response):
        del request
        return self.forward(self.internal_toggle_pause_service, response)

    def destroy_node(self):
        self.keyboard_stop.set()
        if (
            self.keyboard_thread is not None
            and self.keyboard_thread.is_alive()
        ):
            self.keyboard_thread.join(timeout=0.5)
        if self.keyboard_fd is not None:
            if self.original_terminal_attributes is not None:
                try:
                    termios.tcsetattr(
                        self.keyboard_fd,
                        termios.TCSANOW,
                        self.original_terminal_attributes,
                    )
                except OSError:
                    pass
            try:
                os.close(self.keyboard_fd)
            except OSError:
                pass
            self.keyboard_fd = None
        self.bridge_stop.set()
        bridge_thread = getattr(self, 'bridge_thread', None)
        if bridge_thread is not None and bridge_thread.is_alive():
            bridge_thread.join(timeout=1.0)
        if self.bridge_lock.acquire(timeout=1.0):
            try:
                self.stop_bridge_locked()
            finally:
                self.bridge_lock.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedServiceProxy()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
