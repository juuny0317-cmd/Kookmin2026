#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lane following node for the real Xycar.

ROS interface:
  subscribe: /image_raw      sensor_msgs/Image
  publish:   /xycar_motor   std_msgs/Float32MultiArray [angle, speed]

The algorithm keeps only the useful part of the simulation LaneCore:
bird-view transform, white/yellow lane masks, sliding-window lane tracking,
target path inference, and multi-preview PID steering.
"""

from collections import deque
import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


IMAGE_W = 640
IMAGE_H = 480


def clamp(value, low, high):
    return max(low, min(high, value))


class LaneDriveNode(Node):
    def __init__(self):
        super().__init__('lane_drive')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('motor_topic', 'xycar_motor')
        self.declare_parameter('debug_view', False)
        self.declare_parameter('lane_mode', 'center')

        self.declare_parameter('straight_speed', 8.0)
        self.declare_parameter('curve_speed', 5.5)
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('speed_up_delta', 0.7)
        self.declare_parameter('speed_down_delta', 1.4)
        self.declare_parameter('max_angle', 50.0)
        self.declare_parameter('max_angle_delta', 4.0)
        self.declare_parameter('corner_max_angle_delta', 5.5)
        self.declare_parameter('lost_stop_frames', 5)
        self.declare_parameter('max_reuse_frames', 8)

        self.declare_parameter('white_h_low', 0)
        self.declare_parameter('white_s_low', 0)
        self.declare_parameter('white_v_low', 220)
        self.declare_parameter('white_h_high', 180)
        self.declare_parameter('white_s_high', 30)
        self.declare_parameter('white_v_high', 255)
        self.declare_parameter('yellow_h_low', 22)
        self.declare_parameter('yellow_s_low', 130)
        self.declare_parameter('yellow_v_low', 130)
        self.declare_parameter('yellow_h_high', 35)
        self.declare_parameter('yellow_s_high', 255)
        self.declare_parameter('yellow_v_high', 255)
        self.declare_parameter('mask_min_component_area_px', 35)

        self.declare_parameter('warp_top_left_x', 130.0)
        self.declare_parameter('warp_top_right_x', 500.0)
        self.declare_parameter('warp_top_y', 280.0)
        self.declare_parameter('warp_bottom_left_x', 0.0)
        self.declare_parameter('warp_bottom_right_x', 640.0)
        self.declare_parameter('warp_bottom_y', 385.0)
        self.declare_parameter('warp_dst_left_ratio', 0.10)
        self.declare_parameter('warp_dst_right_ratio', 0.90)

        self.declare_parameter('histogram_y_start_ratio', 0.6667)
        self.declare_parameter('histogram_threshold_ratio', 0.38)
        self.declare_parameter('peak_cluster_gap_px', 50)
        self.declare_parameter('yellow_center_switch_gate_px', 120)
        self.declare_parameter('yellow_center_prev_weight', 0.80)
        self.declare_parameter('yellow_center_image_weight', 0.20)

        self.declare_parameter('sliding_windows', 15)
        self.declare_parameter('sliding_margin_px', 50)
        self.declare_parameter('sliding_minpix', 30)
        self.declare_parameter('min_center_line_points', 4)
        self.declare_parameter('path_resample_points', 40)
        self.declare_parameter('path_fit_min_points', 5)
        self.declare_parameter('path_duplicate_x_gap', 1.0)
        self.declare_parameter('path_smoothing_alpha', 0.78)
        self.declare_parameter('lane_half_width_px', 115.0)

        self.declare_parameter('target_forward_px', 120.0)
        self.declare_parameter('curve_target_forward_px', 105.0)
        self.declare_parameter('preview_forward_px', 190.0)
        self.declare_parameter('far_preview_forward_px', 240.0)
        self.declare_parameter('curve_heading_threshold', 0.12)
        self.declare_parameter('preview_heading_threshold', 0.085)
        self.declare_parameter('far_heading_threshold', 0.065)
        self.declare_parameter('near_y_weight', 0.54)
        self.declare_parameter('preview_y_weight', 0.34)
        self.declare_parameter('far_y_weight', 0.12)

        self.declare_parameter('pid_kp', 0.28)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.08)
        self.declare_parameter('pid_integral_limit', 100.0)
        self.declare_parameter('heading_kp', 14.0)
        self.declare_parameter('preview_heading_kp', 6.0)
        self.declare_parameter('far_heading_kp', 8.0)
        self.declare_parameter('curve_steer_boost', 1.0)
        self.declare_parameter('s_curve_speed', 4.0)
        self.declare_parameter('s_curve_max_angle_delta', 8.0)
        self.declare_parameter('s_curve_smooth_window', 1)
        self.declare_parameter('s_curve_near_y_weight', 0.58)
        self.declare_parameter('s_curve_preview_y_weight', 0.37)
        self.declare_parameter('s_curve_far_y_weight', 0.05)
        self.declare_parameter('s_curve_integral_decay', 0.25)
        self.declare_parameter('s_curve_steer_boost', 1.05)
        self.declare_parameter('s_curve_sign_heading_threshold', 0.05)
        self.declare_parameter('straight_smooth_window', 4)
        self.declare_parameter('curve_smooth_window', 2)

        self.declare_parameter('straight_curvature_threshold', 0.012)
        self.declare_parameter('hard_curvature_threshold', 0.024)
        self.declare_parameter('curvature_percentile', 65.0)

        # Legacy launch args from the earlier pure-pursuit version. They are
        # declared so older launch commands do not fail, but this node now uses
        # the PID parameters above.
        self.declare_parameter('pp_steer_gain', 2.0)
        self.declare_parameter('pp_lookahead_base_px', 70.0)
        self.declare_parameter('pp_lookahead_speed_gain', 3.5)

        image_topic = self.param_str('image_topic')
        motor_topic = self.param_str('motor_topic')

        self.bridge = CvBridge()
        self.motor_msg = Float32MultiArray()
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.motor_pub = self.create_publisher(Float32MultiArray, motor_topic, 10)

        self.prev_lane_base = {'left': None, 'center': None, 'right': None}
        self.prev_lane_type = {'left': None, 'center': None, 'right': None}
        self.lane_half_width = self.param_float('lane_half_width_px')
        self.last_valid_path = None
        self.last_angle = 0.0
        self.last_speed = 0.0
        self.lost_frames = 0
        self.angle_buffer = deque()
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.filtered_curvature = 0.0
        self.last_debug = {}

        self.get_logger().info(
            f'lane_drive started: image={image_topic}, motor={motor_topic}, '
            f'mode={self.param_str("lane_mode")}'
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warning(f'cv_bridge failed: {exc}')
            self.publish_drive(0.0, 0.0)
            return

        angle, speed, debug = self.compute_drive_command(frame)
        self.publish_drive(angle, speed)

        if self.param_bool('debug_view'):
            cv2.imshow('lane_drive', debug)
            cv2.waitKey(1)

    def compute_drive_command(self, frame):
        image = cv2.resize(frame, (IMAGE_W, IMAGE_H), interpolation=cv2.INTER_LINEAR)
        bird_view = self.warp_perspective(image)
        white_mask, yellow_mask = self.preprocess_split(bird_view)
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        debug_bird = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

        lanes = self.classify_lanes_by_color_and_position(
            white_mask,
            yellow_mask,
            bird_view.shape[1],
        )
        lane_vehicle_coords = {'left': [], 'center': [], 'right': []}

        for key in ['left', 'center', 'right']:
            candidate = lanes[key]

            if (
                candidate is None and
                self.prev_lane_base[key] is not None and
                self.lost_frames <= self.param_int('max_reuse_frames')
            ):
                candidate = (
                    self.prev_lane_type[key] or ('yellow' if key == 'center' else 'white'),
                    self.prev_lane_base[key],
                )

            if candidate is None:
                continue

            lane_type, base_x = candidate
            mask = yellow_mask if lane_type == 'yellow' else white_mask
            color = {
                'left': (255, 0, 0),
                'center': (0, 255, 255),
                'right': (0, 0, 255),
            }[key]
            pts = self.sliding_window_lane(mask, debug_bird, base_x, color)
            vehicle_pts = self.convert_to_vehicle_coords(pts, bird_view.shape)
            lane_vehicle_coords[key] = vehicle_pts

            if len(pts) >= self.param_int('min_center_line_points'):
                self.prev_lane_base[key] = int(pts[0][0])
                self.prev_lane_type[key] = lane_type

        center_pts, path_mode = self.infer_target_path(lane_vehicle_coords)
        detected_lane_length = len(center_pts)

        detected_path = None
        if detected_lane_length >= self.param_int('min_center_line_points'):
            detected_path = self.generate_resampled_path(center_pts)
            detected_path = self.smooth_path_points(detected_path)

        path, reused_path = self.get_tracking_path(detected_path)

        if not path:
            stop_now = self.lost_frames >= self.param_int('lost_stop_frames')
            angle = self.last_angle if not stop_now else 0.0
            speed = 0.0 if stop_now else min(self.last_speed, self.param_float('curve_speed'))
            debug = self.draw_debug(
                image,
                debug_bird,
                [],
                {},
                angle,
                speed,
                path_mode,
                detected_lane_length,
                reused_path,
            )
            self.last_angle = angle
            self.last_speed = speed
            self.last_debug = {'mode': 'LOST', 'path_mode': path_mode}
            return angle, speed, debug

        curvature = self.estimate_curvature(path)
        self.filtered_curvature = self.filter_curvature(curvature)
        raw_angle, steering_debug = self.get_steering_pid(path)
        angle = self.smooth_and_limit_angle(raw_angle, steering_debug)
        speed = self.select_speed(angle, self.filtered_curvature, reused_path, steering_debug)

        targets = self.debug_targets(path, steering_debug)
        debug = self.draw_debug(
            image,
            debug_bird,
            path,
            targets,
            angle,
            speed,
            path_mode,
            detected_lane_length,
            reused_path,
            steering_debug,
        )

        self.last_angle = angle
        self.last_speed = speed
        self.last_debug = {
            'mode': 'REUSED' if reused_path else 'TRACKING',
            'path_mode': path_mode,
            'curvature': self.filtered_curvature,
            'angle': angle,
            'speed': speed,
            **steering_debug,
        }
        return angle, speed, debug

    def warp_perspective(self, img):
        src = self.warp_src_points()
        dst = np.float32([
            [IMAGE_W * self.param_float('warp_dst_left_ratio'), 0.0],
            [IMAGE_W * self.param_float('warp_dst_right_ratio'), 0.0],
            [IMAGE_W * self.param_float('warp_dst_right_ratio'), IMAGE_H],
            [IMAGE_W * self.param_float('warp_dst_left_ratio'), IMAGE_H],
        ])
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(img, matrix, (IMAGE_W, IMAGE_H))

    def warp_src_points(self):
        return np.float32([
            [self.param_float('warp_top_left_x'), self.param_float('warp_top_y')],
            [self.param_float('warp_top_right_x'), self.param_float('warp_top_y')],
            [self.param_float('warp_bottom_right_x'), self.param_float('warp_bottom_y')],
            [self.param_float('warp_bottom_left_x'), self.param_float('warp_bottom_y')],
        ])

    def preprocess_split(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(
            hsv,
            np.array([
                self.param_int('white_h_low'),
                self.param_int('white_s_low'),
                self.param_int('white_v_low'),
            ]),
            np.array([
                self.param_int('white_h_high'),
                self.param_int('white_s_high'),
                self.param_int('white_v_high'),
            ]),
        )
        yellow_mask = cv2.inRange(
            hsv,
            np.array([
                self.param_int('yellow_h_low'),
                self.param_int('yellow_s_low'),
                self.param_int('yellow_v_low'),
            ]),
            np.array([
                self.param_int('yellow_h_high'),
                self.param_int('yellow_s_high'),
                self.param_int('yellow_v_high'),
            ]),
        )

        open_kernel = np.ones((3, 3), dtype=np.uint8)
        close_kernel = np.ones((9, 5), dtype=np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
        white_mask = self.filter_small_components(white_mask)
        yellow_mask = self.filter_small_components(yellow_mask)
        return white_mask, yellow_mask

    def filter_small_components(self, mask):
        min_area = self.param_int('mask_min_component_area_px')
        if min_area <= 0:
            return mask

        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        filtered = np.zeros_like(mask)
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] >= min_area:
                filtered[labels == label] = 255
        return filtered

    def find_lane_peaks(self, mask):
        height = mask.shape[0]
        y_start = int(height * self.param_float('histogram_y_start_ratio'))
        histogram = np.sum(mask[y_start:, :], axis=0)

        max_value = float(np.max(histogram))
        if max_value <= 0.0:
            return []

        threshold = self.param_float('histogram_threshold_ratio') * max_value
        peak_indices = np.where(histogram > threshold)[0]
        if len(peak_indices) == 0:
            return []

        clusters = []
        cluster = [int(peak_indices[0])]
        gap = self.param_int('peak_cluster_gap_px')

        for x in peak_indices[1:]:
            x = int(x)
            if x - cluster[-1] <= gap:
                cluster.append(x)
            else:
                clusters.append(cluster)
                cluster = [x]
        clusters.append(cluster)

        return [int(np.mean(c)) for c in clusters]

    def classify_lanes_by_color_and_position(self, white_mask, yellow_mask, width):
        lanes = {'left': None, 'center': None, 'right': None}
        white_peaks = self.find_lane_peaks(white_mask)
        yellow_peaks = self.find_lane_peaks(yellow_mask)

        center_x = self.select_yellow_center_peak(yellow_peaks, width)
        if center_x is not None:
            lanes['center'] = ('yellow', center_x)

        if white_peaks:
            lefts = [x for x in sorted(white_peaks) if x < width // 2]
            rights = [x for x in sorted(white_peaks) if x >= width // 2]
            if lefts:
                lanes['left'] = ('white', int(np.mean(lefts)))
            if rights:
                lanes['right'] = ('white', int(np.mean(rights)))

        return lanes

    def select_yellow_center_peak(self, yellow_peaks, width):
        if not yellow_peaks:
            return None

        image_center = width // 2
        prev_center = self.prev_lane_base.get('center')

        if prev_center is None:
            return int(min(yellow_peaks, key=lambda x: abs(x - image_center)))

        gated = [
            x for x in yellow_peaks
            if abs(x - prev_center) <= self.param_int('yellow_center_switch_gate_px')
        ]
        if not gated:
            return None

        prev_weight = self.param_float('yellow_center_prev_weight')
        image_weight = self.param_float('yellow_center_image_weight')
        return int(min(
            gated,
            key=lambda x: prev_weight * abs(x - prev_center) + image_weight * abs(x - image_center),
        ))

    def sliding_window_lane(self, binary_img, out_img, base_x, color):
        height, width = binary_img.shape
        nwindows = self.param_int('sliding_windows')
        margin = self.param_int('sliding_margin_px')
        minpix = self.param_int('sliding_minpix')
        window_height = height // max(1, nwindows)

        nonzero_y, nonzero_x = binary_img.nonzero()
        x_current = int(base_x)
        last_valid_x = int(base_x)
        pts = []

        for window in range(nwindows):
            y_low = height - (window + 1) * window_height
            y_high = height - window * window_height
            x_low = x_current - margin
            x_high = x_current + margin

            cv2.rectangle(
                out_img,
                (max(0, x_low), max(0, y_low)),
                (min(width - 1, x_high), min(height - 1, y_high)),
                color,
                2,
            )

            good = (
                (nonzero_y >= y_low) &
                (nonzero_y < y_high) &
                (nonzero_x >= x_low) &
                (nonzero_x < x_high)
            ).nonzero()[0]

            if len(good) > minpix:
                x_current = int(np.mean(nonzero_x[good]))
                y_current = int(np.mean(nonzero_y[good]))
                last_valid_x = x_current
                pts.append((x_current, y_current))
                cv2.circle(out_img, (x_current, y_current), 5, color, -1)
            else:
                x_current = last_valid_x

        return pts

    @staticmethod
    def convert_to_vehicle_coords(pts, shape):
        height, width = shape[:2]
        return [(height - py, -(px - width // 2)) for px, py in pts]

    def infer_target_path(self, lane_coords):
        left = lane_coords['left']
        center = lane_coords['center']
        right = lane_coords['right']
        self.update_lane_width(left, right)

        mode = self.param_str('lane_mode').lower()
        left_ok = self.path_has_enough_points(left)
        center_ok = self.path_has_enough_points(center)
        right_ok = self.path_has_enough_points(right)

        if mode in ('left', 'lane1', '1'):
            if left_ok and center_ok:
                return self.average_paths(left, center), 'LEFT_LANE'
            if center_ok:
                return self.shift_path(center, self.lane_half_width * 0.5), 'LEFT_FROM_YELLOW'
            if left_ok:
                return self.shift_path(left, -self.lane_half_width * 0.5), 'LEFT_FROM_WHITE'

        if mode in ('right', 'lane2', '2'):
            if center_ok and right_ok:
                return self.average_paths(center, right), 'RIGHT_LANE'
            if center_ok:
                return self.shift_path(center, -self.lane_half_width * 0.5), 'RIGHT_FROM_YELLOW'
            if right_ok:
                return self.shift_path(right, self.lane_half_width * 0.5), 'RIGHT_FROM_WHITE'

        if center_ok and mode not in ('road', 'road_center'):
            return center, 'YELLOW'

        if left_ok and right_ok:
            return self.average_paths(left, right), 'WHITE_ROAD_CENTER'
        if left_ok:
            return self.shift_path(left, -self.lane_half_width), 'LEFT_ONLY_ROAD_CENTER'
        if right_ok:
            return self.shift_path(right, self.lane_half_width), 'RIGHT_ONLY_ROAD_CENTER'

        if center:
            return center, 'SHORT_YELLOW'
        if left:
            return self.shift_path(left, -self.lane_half_width), 'SHORT_LEFT_ONLY'
        if right:
            return self.shift_path(right, self.lane_half_width), 'SHORT_RIGHT_ONLY'

        return [], 'NONE'

    def path_has_enough_points(self, path):
        return bool(path) and len(path) >= self.param_int('min_center_line_points')

    def update_lane_width(self, left, right):
        if not left or not right:
            return
        averaged = self.average_paths(left, right)
        if not averaged:
            return
        left_sorted = sorted(left, key=lambda p: p[0])
        right_sorted = sorted(right, key=lambda p: p[0])
        xs = np.array([p[0] for p in averaged], dtype=np.float32)
        left_y = np.interp(xs, [p[0] for p in left_sorted], [p[1] for p in left_sorted])
        right_y = np.interp(xs, [p[0] for p in right_sorted], [p[1] for p in right_sorted])
        widths = np.abs(left_y - right_y) * 0.5
        if widths.size > 0:
            self.lane_half_width = float(np.median(widths))

    def average_paths(self, path_a, path_b):
        if not path_a or not path_b:
            return []

        a = sorted(path_a, key=lambda p: p[0])
        b = sorted(path_b, key=lambda p: p[0])
        start_x = max(a[0][0], b[0][0])
        end_x = min(a[-1][0], b[-1][0])

        if end_x <= start_x:
            count = min(len(a), len(b))
            return [
                ((a[i][0] + b[i][0]) * 0.5, (a[i][1] + b[i][1]) * 0.5)
                for i in range(count)
            ]

        count = max(self.param_int('path_fit_min_points'), min(len(a), len(b)))
        xs = np.linspace(start_x, end_x, count)
        ay = np.interp(xs, [p[0] for p in a], [p[1] for p in a])
        by = np.interp(xs, [p[0] for p in b], [p[1] for p in b])
        return [(float(x), float((y1 + y2) * 0.5)) for x, y1, y2 in zip(xs, ay, by)]

    @staticmethod
    def shift_path(path, offset_y):
        return [(float(x), float(y + offset_y)) for x, y in path]

    def generate_resampled_path(self, center_line_pts):
        if not center_line_pts or len(center_line_pts) < self.param_int('path_fit_min_points'):
            return center_line_pts

        pts = sorted(center_line_pts, key=lambda p: p[0])
        filtered = []
        last_x = None
        duplicate_gap = self.param_float('path_duplicate_x_gap')

        for x, y in pts:
            if last_x is None or abs(x - last_x) > duplicate_gap:
                filtered.append((float(x), float(y)))
                last_x = x

        if len(filtered) < self.param_int('path_fit_min_points'):
            return filtered

        xs = np.array([p[0] for p in filtered], dtype=np.float32)
        ys = np.array([p[1] for p in filtered], dtype=np.float32)
        dx = np.diff(xs)
        dy = np.diff(ys)
        ds = np.sqrt(dx * dx + dy * dy)
        progress = np.insert(np.cumsum(ds), 0, 0.0)

        if progress[-1] < 1.0:
            return filtered

        sample_count = self.param_int('path_resample_points')
        new_progress = np.linspace(0.0, progress[-1], sample_count)
        new_x = np.interp(new_progress, progress, xs)
        new_y = np.interp(new_progress, progress, ys)
        return [(float(x), float(y)) for x, y in zip(new_x, new_y)]

    @staticmethod
    def smooth_path_points(path, window=5):
        if not path or len(path) < window:
            return path

        xs = np.array([p[0] for p in path], dtype=np.float32)
        ys = np.array([p[1] for p in path], dtype=np.float32)
        kernel = np.ones(window, dtype=np.float32) / float(window)
        padded_y = np.pad(ys, (window // 2, window // 2), mode='edge')
        smooth_y = np.convolve(padded_y, kernel, mode='valid')
        return [(float(x), float(y)) for x, y in zip(xs, smooth_y)]

    def stabilize_path(self, detected_path):
        if not detected_path or len(detected_path) < self.param_int('path_fit_min_points'):
            return None

        path = sorted(
            [(float(x), float(y)) for x, y in detected_path if x >= 0.0],
            key=lambda p: p[0],
        )
        if len(path) < self.param_int('path_fit_min_points'):
            return None

        if self.last_valid_path is not None and len(self.last_valid_path) >= 2:
            cur_x = np.array([p[0] for p in path], dtype=np.float32)
            cur_y = np.array([p[1] for p in path], dtype=np.float32)
            prev_x = np.array([p[0] for p in self.last_valid_path], dtype=np.float32)
            prev_y = np.array([p[1] for p in self.last_valid_path], dtype=np.float32)
            prev_interp_y = np.interp(cur_x, prev_x, prev_y, left=prev_y[0], right=prev_y[-1])
            alpha = clamp(self.param_float('path_smoothing_alpha'), 0.0, 1.0)
            mixed_y = alpha * cur_y + (1.0 - alpha) * prev_interp_y
            path = [(float(x), float(y)) for x, y in zip(cur_x, mixed_y)]

        self.last_valid_path = path
        return path

    def get_tracking_path(self, detected_path):
        stable_path = self.stabilize_path(detected_path)
        if stable_path is not None:
            self.lost_frames = 0
            return stable_path, False

        self.lost_frames += 1
        if self.last_valid_path is not None and self.lost_frames <= self.param_int('max_reuse_frames'):
            return self.last_valid_path, True

        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        return None, False

    def get_steering_pid(self, path):
        if path is None or len(path) < 2:
            return self.last_angle, {
                'target_y': 0.0,
                'preview_y': 0.0,
                'far_y': 0.0,
                'heading_error': 0.0,
                'preview_heading': 0.0,
                'far_preview_heading': 0.0,
                'curve_ahead': False,
                's_curve_like': False,
                'target_forward_px': self.param_float('target_forward_px'),
                'multi_error': 0.0,
            }

        target_forward_px = self.param_float('target_forward_px')
        near_idx, _, near_y = self.find_target_point(path, target_forward_px)
        heading_error = self.path_heading_at(path, near_idx)

        preview_idx, _, preview_y = self.find_target_point(path, self.param_float('preview_forward_px'))
        far_idx, _, far_y = self.find_target_point(path, self.param_float('far_preview_forward_px'))
        preview_heading = self.path_heading_at(path, preview_idx)
        far_preview_heading = self.path_heading_at(path, far_idx)

        curve_ahead = (
            abs(heading_error) > self.param_float('curve_heading_threshold') or
            abs(preview_heading) > self.param_float('preview_heading_threshold') or
            abs(far_preview_heading) > self.param_float('far_heading_threshold')
        )

        if curve_ahead:
            target_forward_px = self.param_float('curve_target_forward_px')
            near_idx, _, near_y = self.find_target_point(path, target_forward_px)
            heading_error = self.path_heading_at(path, near_idx)

        headings = [heading_error, preview_heading, far_preview_heading]
        sign_threshold = self.param_float('s_curve_sign_heading_threshold')
        s_curve_like = any(
            a * b < 0.0 and abs(a) > sign_threshold and abs(b) > sign_threshold
            for i, a in enumerate(headings)
            for b in headings[i + 1:]
        )

        if s_curve_like:
            near_weight = self.param_float('s_curve_near_y_weight')
            preview_weight = self.param_float('s_curve_preview_y_weight')
            far_weight = self.param_float('s_curve_far_y_weight')
            self.pid_integral *= self.param_float('s_curve_integral_decay')
        else:
            near_weight = self.param_float('near_y_weight')
            preview_weight = self.param_float('preview_y_weight')
            far_weight = self.param_float('far_y_weight')

        weight_sum = max(abs(near_weight) + abs(preview_weight) + abs(far_weight), 1.0e-6)
        multi_error = (
            near_weight * near_y +
            preview_weight * preview_y +
            far_weight * far_y
        ) / weight_sum

        dt = 0.05
        self.pid_integral += multi_error * dt
        limit = self.param_float('pid_integral_limit')
        self.pid_integral = float(np.clip(self.pid_integral, -limit, limit))
        derivative = (multi_error - self.pid_prev_error) / dt
        self.pid_prev_error = multi_error

        control = (
            self.param_float('pid_kp') * multi_error +
            self.param_float('pid_ki') * self.pid_integral +
            self.param_float('pid_kd') * derivative +
            self.param_float('heading_kp') * heading_error +
            self.param_float('preview_heading_kp') * preview_heading +
            self.param_float('far_heading_kp') * far_preview_heading
        )

        if s_curve_like:
            control *= self.param_float('s_curve_steer_boost')
        elif curve_ahead:
            control *= self.param_float('curve_steer_boost')

        angle = -control
        return angle, {
            'target_y': float(near_y),
            'preview_y': float(preview_y),
            'far_y': float(far_y),
            'heading_error': float(heading_error),
            'preview_heading': float(preview_heading),
            'far_preview_heading': float(far_preview_heading),
            'curve_ahead': bool(curve_ahead),
            's_curve_like': bool(s_curve_like),
            'target_forward_px': float(target_forward_px),
            'multi_error': float(multi_error),
        }

    @staticmethod
    def find_target_point(path, forward_px):
        target_idx = len(path) - 1
        target_x, target_y = path[-1]

        for i, (x, y) in enumerate(path):
            if x >= forward_px:
                target_idx = i
                target_x, target_y = x, y
                break

        return target_idx, target_x, target_y

    def path_heading_at(self, path, idx):
        if path is None or len(path) < 2:
            return 0.0

        heading_window = 3
        i0 = max(idx - heading_window, 0)
        i1 = min(idx + heading_window, len(path) - 1)
        if i0 == i1:
            return 0.0

        dx = path[i1][0] - path[i0][0]
        dy = path[i1][1] - path[i0][1]
        return math.atan2(dy, max(dx, 1.0e-3))

    def smooth_and_limit_angle(self, raw_angle, steering_debug):
        s_curve_like = bool(steering_debug.get('s_curve_like', False))
        corner_like = (
            bool(steering_debug.get('curve_ahead', False)) or
            s_curve_like or
            self.filtered_curvature > self.param_float('straight_curvature_threshold')
        )
        if s_curve_like:
            window = self.param_int('s_curve_smooth_window')
        elif corner_like:
            window = self.param_int('curve_smooth_window')
        else:
            window = self.param_int('straight_smooth_window')
        window = max(1, window)

        if self.angle_buffer:
            previous = self.angle_buffer[-1]
            sign_flip = raw_angle * previous < 0.0
            if sign_flip and (s_curve_like or abs(raw_angle - previous) >= 18.0):
                self.angle_buffer.clear()

        self.angle_buffer.append(float(raw_angle))
        while len(self.angle_buffer) > window:
            self.angle_buffer.popleft()

        smoothed = float(np.mean(self.angle_buffer))
        if s_curve_like:
            max_delta = self.param_float('s_curve_max_angle_delta')
        elif corner_like:
            max_delta = self.param_float('corner_max_angle_delta')
        else:
            max_delta = self.param_float('max_angle_delta')
        limited = self.last_angle + clamp(smoothed - self.last_angle, -max_delta, max_delta)
        return float(np.clip(limited, -self.param_float('max_angle'), self.param_float('max_angle')))

    def select_speed(self, angle, curvature, reused_path, steering_debug):
        straight_speed = self.param_float('straight_speed')
        curve_speed = self.param_float('curve_speed')
        min_speed = self.param_float('min_speed')

        angle_ratio = clamp(abs(angle) / max(self.param_float('max_angle'), 1.0), 0.0, 1.0)
        curvature_ratio = clamp(
            curvature / max(self.param_float('hard_curvature_threshold'), 1.0e-6),
            0.0,
            1.0,
        )
        ratio = max(angle_ratio, curvature_ratio)
        target = straight_speed - (straight_speed - curve_speed) * ratio

        if reused_path:
            target = min(target, curve_speed)
        if bool(steering_debug.get('s_curve_like', False)):
            target = min(target, self.param_float('s_curve_speed'))

        target = max(min_speed, float(target))
        if self.last_speed <= 0.0:
            return target

        delta = target - self.last_speed
        if delta > self.param_float('speed_up_delta'):
            target = self.last_speed + self.param_float('speed_up_delta')
        elif delta < -self.param_float('speed_down_delta'):
            target = self.last_speed - self.param_float('speed_down_delta')

        return max(min_speed, float(target))

    def estimate_curvature(self, path):
        if path is None or len(path) < 3:
            return 0.0

        xs = np.array([p[0] for p in path], dtype=np.float32)
        ys = np.array([p[1] for p in path], dtype=np.float32)
        dx = np.diff(xs)
        dy = np.diff(ys)
        d2x = np.diff(dx)
        d2y = np.diff(dy)
        denominator = (dx[:-1] * dx[:-1] + dy[:-1] * dy[:-1]) ** 1.5
        valid = denominator > 1.0e-6

        if not np.any(valid):
            return 0.0

        curvature = np.abs(d2x[valid] * dy[:-1][valid] - d2y[valid] * dx[:-1][valid])
        curvature = curvature / denominator[valid]
        if curvature.size == 0:
            return 0.0
        return float(np.percentile(curvature, self.param_float('curvature_percentile')))

    def filter_curvature(self, curvature):
        if curvature > self.filtered_curvature:
            return 0.25 * self.filtered_curvature + 0.75 * curvature
        return 0.64 * self.filtered_curvature + 0.36 * curvature

    def debug_targets(self, path, steering_debug):
        targets = {}
        for name, distance in [
            ('near', steering_debug.get('target_forward_px', self.param_float('target_forward_px'))),
            ('preview', self.param_float('preview_forward_px')),
            ('far', self.param_float('far_preview_forward_px')),
        ]:
            _, x, y = self.find_target_point(path, float(distance))
            targets[name] = (x, y)
        return targets

    def draw_debug(
        self,
        image,
        debug_bird,
        path,
        targets,
        angle,
        speed,
        path_mode,
        detected_lane_length,
        reused_path,
        steering_debug=None,
    ):
        original = image.copy()
        cv2.polylines(
            original,
            [np.int32(self.warp_src_points())],
            isClosed=True,
            color=(0, 255, 255),
            thickness=2,
        )

        for x, y in path:
            px = int(-y + IMAGE_W // 2)
            py = int(IMAGE_H - x)
            if 0 <= px < IMAGE_W and 0 <= py < IMAGE_H:
                cv2.circle(debug_bird, (px, py), 4, (255, 255, 0), -1)

        target_colors = {
            'near': (255, 0, 255),
            'preview': (0, 255, 0),
            'far': (255, 255, 255),
        }
        car_px = (IMAGE_W // 2, IMAGE_H - 1)
        cv2.line(debug_bird, car_px, (IMAGE_W // 2, 0), (0, 180, 0), 1)
        for name, target in targets.items():
            px = int(-target[1] + IMAGE_W // 2)
            py = int(IMAGE_H - target[0])
            if 0 <= px < IMAGE_W and 0 <= py < IMAGE_H:
                cv2.circle(debug_bird, (px, py), 8, target_colors[name], -1)
                cv2.line(debug_bird, car_px, (px, py), target_colors[name], 2)

        mode_text = f'{path_mode} angle={angle:.1f} speed={speed:.1f}'
        count_text = f'pts={len(path)} raw={detected_lane_length} lost={self.lost_frames} reuse={int(reused_path)}'
        cv2.putText(original, mode_text, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
        cv2.putText(debug_bird, count_text, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)

        if steering_debug:
            debug_text = (
                f'y={steering_debug["target_y"]:.0f}/'
                f'{steering_debug["preview_y"]:.0f}/'
                f'{steering_debug["far_y"]:.0f} '
                f'h={steering_debug["heading_error"]:.2f} '
                f'sc={int(steering_debug.get("s_curve_like", False))}'
            )
            cv2.putText(debug_bird, debug_text, (15, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        return np.hstack((original, debug_bird))

    def publish_drive(self, angle, speed):
        self.motor_msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(self.motor_msg)

    def param_str(self, name):
        return str(self.get_parameter(name).value)

    def param_bool(self, name):
        return bool(self.get_parameter(name).value)

    def param_int(self, name):
        return int(self.get_parameter(name).value)

    def param_float(self, name):
        return float(self.get_parameter(name).value)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ROS 2 launch may already have invalidated the context before this
        # finally block runs.  Publishing in that state raises RCLError and
        # made a normal Ctrl-C look like a controller crash.
        if rclpy.ok():
            node.publish_drive(0.0, 0.0)
        if node.param_bool('debug_view'):
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
