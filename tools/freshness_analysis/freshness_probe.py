#!/usr/bin/env python3
"""Low-overhead timestamp observer for freshness root-cause runs.

This node is deliberately outside the production launch.  It records only
message metadata and optional ``PipelineTiming`` messages; image pixels and
detection payloads are never copied into the output file.  Rows stay in memory
and are written once during a clean shutdown so disk I/O cannot perturb the
event being diagnosed.
"""

from __future__ import annotations

import csv
from pathlib import Path
import threading
import time

from custom_interfaces.msg import (
    Curve,
    Detections,
    LaneControlState,
    LaneControlStateV2,
    PipelineTiming,
    StampedMotorCommand,
    StanleyDebug,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


FIELDS = (
    'sequence', 'event', 'source_ns', 'ros_ns', 'steady_ns', 'wall_ns',
    'timing_receive_ros_ns', 'timing_start_ros_ns', 'timing_end_ros_ns',
    'queue_wait_ms', 'compute_ms', 'received_count', 'processed_count',
    'replaced_count', 'input_reused', 'state_age_s', 'angle', 'speed',
    'frame_id',
)


def stamp_ns(stamp) -> int:
    """Convert a ROS Time-like object to nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def reliable_qos(depth: int) -> QoSProfile:
    """Match the current reliable production topics without adding history."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class FreshnessProbe(Node):
    """Observe source and callback clocks without participating in decisions."""

    def __init__(self):
        super().__init__('freshness_probe')
        self.declare_parameter('output_path', '/tmp/freshness_timestamps.csv')
        self.declare_parameter(
            'final_motor_topic', '/offline/freshness_test_xycar_motor')
        self.declare_parameter('observe_images', True)
        self.declare_parameter('observe_pipeline_timing', True)

        self.output_path = Path(str(
            self.get_parameter('output_path').value)).expanduser()
        self.rows = []
        self.rows_lock = threading.Lock()
        self.sequence = 0
        self.flushed = False

        if bool(self.get_parameter('observe_images').value):
            self.create_subscription(
                Image, '/image_raw',
                lambda msg: self.record_header('image_raw_observed', msg),
                reliable_qos(1),
            )
            self.create_subscription(
                Image, '/perception/lane/image',
                lambda msg: self.record_header(
                    'frame_router_selected_observed', msg),
                reliable_qos(1),
            )
        self.create_subscription(
            Detections, '/lane_yolo/detections',
            lambda msg: self.record_header('yolo_detection_observed', msg),
            reliable_qos(5),
        )
        self.create_subscription(
            Curve, '/center_curve',
            lambda msg: self.record_header('center_curve_observed', msg),
            reliable_qos(5),
        )
        self.create_subscription(
            LaneControlState, '/lane_control_state_stamped',
            lambda msg: self.record_header('lane_state_observed', msg),
            reliable_qos(5),
        )
        self.create_subscription(
            LaneControlStateV2, '/lane_control_state_v2',
            self.lane_state_v2_callback,
            reliable_qos(5),
        )
        self.create_subscription(
            StanleyDebug, '/stanley/debug', self.stanley_debug_callback,
            reliable_qos(10),
        )
        self.create_subscription(
            StampedMotorCommand, '/lane_motor_cmd_stamped',
            self.lane_command_callback,
            reliable_qos(10),
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter('final_motor_topic').value),
            self.final_motor_callback,
            reliable_qos(10),
        )
        self.create_subscription(
            Clock, '/clock', self.clock_callback, reliable_qos(1))
        if bool(self.get_parameter('observe_pipeline_timing').value):
            self.create_subscription(
                PipelineTiming, '/pipeline_timing',
                self.pipeline_timing_callback,
                reliable_qos(100),
            )
        self.get_logger().info(
            f'Freshness metadata probe enabled: output={self.output_path}')

    def append(self, event, source_ns=0, frame_id='', **values):
        """Capture all local clocks at callback execution."""
        row = dict.fromkeys(FIELDS, '')
        with self.rows_lock:
            self.sequence += 1
            row.update({
                'sequence': self.sequence,
                'event': str(event),
                'source_ns': int(source_ns),
                'ros_ns': int(self.get_clock().now().nanoseconds),
                'steady_ns': int(time.monotonic_ns()),
                'wall_ns': int(time.time_ns()),
                'frame_id': str(frame_id),
            })
            row.update(values)
            self.rows.append(row)

    def record_header(self, event, message):
        self.append(
            event,
            source_ns=stamp_ns(message.header.stamp),
            frame_id=message.header.frame_id,
        )

    def lane_state_v2_callback(self, message):
        self.record_header('lane_state_v2_observed', message.control)

    def stanley_debug_callback(self, message):
        self.append(
            'stanley_debug_observed',
            source_ns=stamp_ns(message.header.stamp),
            frame_id=message.header.frame_id,
            state_age_s=float(message.state_age_s),
            angle=float(message.final_servo_angle),
            speed=float(message.final_speed),
        )

    def lane_command_callback(self, message):
        self.append(
            'lane_command_observed',
            source_ns=stamp_ns(message.header.stamp),
            frame_id=message.header.frame_id,
            state_age_s=float(message.state_age_s),
            angle=float(message.angle),
            speed=float(message.speed),
        )

    def final_motor_callback(self, message):
        data = list(message.data)
        self.append(
            'final_motor_observed',
            angle=float(data[0]) if data else '',
            speed=float(data[1]) if len(data) > 1 else '',
        )

    def clock_callback(self, message):
        self.append('clock_observed', source_ns=stamp_ns(message.clock))

    def pipeline_timing_callback(self, message):
        self.append(
            f'pipeline:{message.stage}',
            source_ns=stamp_ns(message.header.stamp),
            frame_id=message.header.frame_id,
            timing_receive_ros_ns=int(message.receive_stamp_ns),
            timing_start_ros_ns=int(message.start_stamp_ns),
            timing_end_ros_ns=int(message.end_stamp_ns),
            queue_wait_ms=float(message.queue_wait_ms),
            compute_ms=float(message.compute_ms),
            received_count=int(message.received_count),
            processed_count=int(message.processed_count),
            replaced_count=int(message.replaced_count),
            input_reused=int(message.input_reused),
        )

    def flush(self):
        """Persist once; safe to call from both destroy and finally."""
        with self.rows_lock:
            if self.flushed:
                return
            self.flushed = True
            rows = list(self.rows)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(rows)

    def destroy_node(self):
        self.flush()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FreshnessProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
