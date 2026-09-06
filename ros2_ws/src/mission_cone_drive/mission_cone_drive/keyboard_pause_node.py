"""Toggle integrated-drive pause from the launch terminal spacebar."""

import os
import select
import termios
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


class KeyboardPauseNode(Node):
    """Read space from the controlling terminal and call the toggle service."""

    def __init__(self):
        super().__init__('integrated_keyboard_pause')
        self.declare_parameter(
            'toggle_service_name', '/toggle_pause_integrated_drive')
        self.declare_parameter('keyboard_device', '/dev/tty')
        self.declare_parameter('key_release_quiet_s', 0.20)

        service_name = str(
            self.get_parameter('toggle_service_name').value).strip()
        self.keyboard_device = str(
            self.get_parameter('keyboard_device').value).strip()
        self.key_release_quiet_s = max(
            0.05,
            float(self.get_parameter('key_release_quiet_s').value),
        )
        self.toggle_client = self.create_client(Trigger, service_name)
        self.toggle_requested = threading.Event()
        self.keyboard_stop = threading.Event()
        self.toggle_future = None
        self.keyboard_fd = None
        self.original_terminal_attributes = None
        self.keyboard_thread = None
        self.poll_timer = self.create_timer(0.05, self.poll_toggle_request)
        self.start_keyboard_reader()

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
                self.toggle_requested.set()

    def poll_toggle_request(self):
        if not self.toggle_requested.is_set():
            return
        self.toggle_requested.clear()
        if self.toggle_future is not None and not self.toggle_future.done():
            self.get_logger().warn(
                'Spacebar ignored: a pause/resume request is in progress.')
            return
        if not self.toggle_client.service_is_ready():
            self.get_logger().warn(
                'Spacebar ignored: integrated drive control is not ready.')
            return
        self.toggle_future = self.toggle_client.call_async(Trigger.Request())
        self.toggle_future.add_done_callback(self.toggle_finished)

    def toggle_finished(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Spacebar pause/resume failed: {exc}')
            return
        if result.success:
            self.get_logger().warn(result.message)
        else:
            self.get_logger().warn(f'Spacebar ignored: {result.message}')

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
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPauseNode()
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
