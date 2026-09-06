"""Trigger client used by the host container-service proxy."""

import json
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


RESPONSE_PREFIX = 'XYCAR_TRIGGER_RESPONSE='
READY_MARKER = 'XYCAR_SERVICE_CLIENT_READY'


def call_trigger(service_name, timeout_s=2.0):
    rclpy.init()
    node = Node('integrated_container_service_client')
    try:
        client = node.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f'internal service unavailable: {service_name}')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'internal service timed out: {service_name}')
        result = future.result()
        return bool(result.success), str(result.message)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def serve_trigger_requests(service_names, timeout_s=2.0):
    """Keep ROS discovery warm and serve requests on stdio."""
    rclpy.init()
    node = Node('integrated_container_service_bridge')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    clients = {
        name: node.create_client(Trigger, name)
        for name in service_names
    }
    try:
        for name, client in clients.items():
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f'internal service unavailable: {name}')
        spin_thread.start()
        print(READY_MARKER, flush=True)
        for line in sys.stdin:
            service_name = line.strip()
            try:
                client = clients.get(service_name)
                if client is None:
                    raise RuntimeError(
                        f'invalid internal service request: {service_name!r}')
                future = client.call_async(Trigger.Request())
                completed = threading.Event()
                future.add_done_callback(lambda _future: completed.set())
                if not completed.wait(timeout_s):
                    raise RuntimeError(
                        f'internal service timed out: {service_name}')
                result = future.result()
                if result is None:
                    raise RuntimeError(
                        f'internal service returned no result: {service_name}')
                payload = {
                    'success': bool(result.success),
                    'message': str(result.message),
                }
            except Exception as exc:
                payload = {'success': False, 'message': str(exc)}
            print(
                RESPONSE_PREFIX + json.dumps(payload, ensure_ascii=False),
                flush=True,
            )
    finally:
        executor.shutdown(timeout_sec=1.0)
        if spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    if len(sys.argv) >= 3 and sys.argv[1] == '--stdio-server':
        serve_trigger_requests(sys.argv[2:])
        return
    if len(sys.argv) != 2:
        raise SystemExit(
            'usage: container_service_client SERVICE_NAME | '
            '--stdio-server SERVICE_NAME [SERVICE_NAME ...]')
    try:
        success, message = call_trigger(sys.argv[1])
        payload = {'success': success, 'message': message}
        exit_code = 0
    except Exception as exc:
        payload = {'success': False, 'message': str(exc)}
        exit_code = 1
    print(RESPONSE_PREFIX + json.dumps(payload, ensure_ascii=False))
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
