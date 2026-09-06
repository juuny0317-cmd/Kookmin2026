from collections import deque
from dataclasses import dataclass
import math

from mission_cone_drive.corridor_geometry import CorridorGeometry
import numpy as np

try:
    from scipy.interpolate import PchipInterpolator
except ImportError:  # pragma: no cover - exercised only on minimal installs
    PchipInterpolator = None


UNVISITED = -2
NOISE = -1


@dataclass(frozen=True)
class LidarDbscanClusterConfig:
    min_range_m: float = 0.18
    max_range_m: float = 1.50
    dbscan_eps_m: float = 0.04
    dbscan_min_samples: int = 3
    max_cone_diameter_m: float = 0.30
    angle_bin_deg: float = 0.0
    min_cluster_separation_m: float = 0.15


@dataclass(frozen=True)
class LidarDbscanPathConfig:
    left_seed_max_range_m: float = 0.80
    right_seed_max_range_m: float = 0.80
    left_sector_start_deg: float = 30.0
    left_sector_end_deg: float = 100.0
    right_sector_start_deg: float = 260.0
    right_sector_end_deg: float = 350.0
    grow_distance_m: float = 0.50
    min_pair_distance_m: float = 0.45
    max_pair_distance_m: float = 1.30
    expected_pair_distance_m: float = 1.00
    cone_spacing_m: float = 0.30
    min_pair_normal_alignment: float = 0.40
    max_pair_longitudinal_m: float = 0.55
    midpoint_reference_gate_m: float = 0.55
    virtual_x_m: float = 0.10
    virtual_half_width_m: float = 0.50
    rear_axle_offset_m: float = 0.42
    path_points: int = 100
    max_abs_path_y_m: float = 1.50
    min_path_span_m: float = 0.20
    max_path_lateral_jump_m: float = 0.12
    max_path_heading_jump_deg: float = 30.0
    max_path_curvature_per_m: float = 6.0
    path_smoothing_alpha: float = 0.90
    path_lost_keep_frames: int = 4
    path_lost_keep_s: float = 0.45
    motion_compensation_enabled: bool = True
    prediction_min_forward_m: float = 0.90
    min_path_horizon_m: float = 0.90
    prediction_max_extension_heading_deg: float = 35.0
    one_side_measurement_alpha: float = 0.25
    one_side_max_lateral_correction_m: float = 0.20
    one_side_max_heading_correction_deg: float = 12.0
    one_side_max_age_since_paired_s: float = 0.45


def _point_grid(points, cell_size):
    grid = {}
    for index, point in enumerate(points):
        key = (
            int(math.floor(float(point[0]) / cell_size)),
            int(math.floor(float(point[1]) / cell_size)),
        )
        grid.setdefault(key, []).append(index)
    return grid


def dbscan_labels(points, eps_m, min_samples):
    """Return DBSCAN labels without requiring scikit-learn on the Xycar."""
    points = np.asarray(points, dtype=np.float32)
    point_count = int(points.shape[0])
    if point_count == 0:
        return np.empty(0, dtype=np.int32)

    eps_m = max(float(eps_m), 1e-6)
    min_samples = max(int(min_samples), 1)
    eps_sq = eps_m * eps_m
    grid = _point_grid(points, eps_m)
    labels = np.full(point_count, UNVISITED, dtype=np.int32)

    def region_query(index):
        point = points[index]
        cell_x = int(math.floor(float(point[0]) / eps_m))
        cell_y = int(math.floor(float(point[1]) / eps_m))
        neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for candidate in grid.get((cell_x + dx, cell_y + dy), ()):
                    delta = points[candidate] - point
                    if float(np.dot(delta, delta)) <= eps_sq:
                        neighbors.append(candidate)
        return neighbors

    cluster_id = 0
    for point_index in range(point_count):
        if labels[point_index] != UNVISITED:
            continue

        neighbors = region_query(point_index)
        if len(neighbors) < min_samples:
            labels[point_index] = NOISE
            continue

        labels[point_index] = cluster_id
        queue = deque(neighbors)
        queued = set(neighbors)
        while queue:
            neighbor_index = queue.popleft()
            if labels[neighbor_index] == NOISE:
                labels[neighbor_index] = cluster_id
            if labels[neighbor_index] != UNVISITED:
                continue

            labels[neighbor_index] = cluster_id
            expanded = region_query(neighbor_index)
            if len(expanded) < min_samples:
                continue
            for expanded_index in expanded:
                if expanded_index not in queued:
                    queue.append(expanded_index)
                    queued.add(expanded_index)

        cluster_id += 1

    return labels


def filter_front_clusters_by_angle(cluster_centers, degree_bin):
    """Keep the nearest cluster in each LiDAR angular sector."""
    degree_bin = float(degree_bin)
    if degree_bin <= 0.0:
        return [
            (float(point[0]), float(point[1]))
            for point in cluster_centers
        ]
    used_bins = set()
    selected = []
    ordered = sorted(
        cluster_centers,
        key=lambda point: math.hypot(point[0], point[1]),
    )
    for x, y in ordered:
        theta_deg = math.degrees(math.atan2(y, x))
        bin_index = int(math.floor(theta_deg / degree_bin))
        if bin_index in used_bins:
            continue
        selected.append((float(x), float(y)))
        used_bins.add(bin_index)
    return selected


def filter_front_clusters_by_separation(cluster_centers, separation_m):
    """Suppress only spatially overlapping candidates.

    A fixed angular bin removes distinct cones that happen to line up in a
    bend.  Euclidean non-maximum suppression keeps the nearest return only
    when two candidate centres are too close to represent separate cones.
    """
    separation_m = max(float(separation_m), 0.0)
    selected = []
    ordered = sorted(
        cluster_centers,
        key=lambda point: math.hypot(point[0], point[1]),
    )
    for point in ordered:
        x, y = float(point[0]), float(point[1])
        if any(
            math.hypot(x - kept_x, y - kept_y) < separation_m
            for kept_x, kept_y in selected
        ):
            continue
        selected.append((x, y))
    return selected


def extract_lidar_dbscan_clusters(
    ranges,
    angle_min,
    angle_increment,
    config,
):
    """Extract DBSCAN cone candidates and filter them by angular bin."""
    ranges = np.asarray(ranges, dtype=np.float32)
    if ranges.size == 0:
        return [], []

    angles = (
        float(angle_min)
        + np.arange(ranges.size, dtype=np.float32) * float(angle_increment)
    )
    valid = (
        np.isfinite(ranges)
        & (ranges > config.min_range_m)
        & (ranges <= config.max_range_m)
        & (angles >= -math.pi / 2.0)
        & (angles <= math.pi / 2.0)
    )
    if int(np.count_nonzero(valid)) < config.dbscan_min_samples:
        return [], []

    valid_ranges = ranges[valid]
    valid_angles = angles[valid]
    points = np.column_stack((
        valid_ranges * np.cos(valid_angles),
        valid_ranges * np.sin(valid_angles),
    )).astype(np.float32)

    labels = dbscan_labels(
        points,
        config.dbscan_eps_m,
        config.dbscan_min_samples,
    )
    raw_centers = []
    for cluster_id in sorted(set(labels.tolist())):
        if cluster_id == NOISE:
            continue
        cluster_points = points[labels == cluster_id]
        if cluster_points.size == 0:
            continue

        span_x = float(np.ptp(cluster_points[:, 0]))
        span_y = float(np.ptp(cluster_points[:, 1]))
        diameter = math.hypot(span_x, span_y)
        if diameter > config.max_cone_diameter_m:
            continue

        distances = np.hypot(cluster_points[:, 0], cluster_points[:, 1])
        nearest = cluster_points[int(np.argmin(distances))]
        raw_centers.append((float(nearest[0]), float(nearest[1])))

    filtered = filter_front_clusters_by_separation(
        raw_centers,
        config.min_cluster_separation_m,
    )
    filtered = filter_front_clusters_by_angle(
        filtered, config.angle_bin_deg)
    return raw_centers, filtered


class LidarDbscanPathBuilder:
    """Build a temporally continuous corridor path from DBSCAN candidates.

    The original implementation paired every left cone with whichever right
    cone had the closest y value.  That allowed the same cone to be reused and
    produced a shallow or delayed centerline at an S-bend.  Boundary cones are
    now matched one-to-one, in forward order, using corridor width,
    longitudinal alignment, and the previous centerline.
    """

    def __init__(self, config=None):
        self.config = config or LidarDbscanPathConfig()
        self.corridor_geometry = CorridorGeometry(
            min_width_m=self.config.min_pair_distance_m,
            max_width_m=self.config.max_pair_distance_m,
            expected_width_m=self.config.expected_pair_distance_m,
            cone_spacing_m=self.config.cone_spacing_m,
            min_normal_alignment=self.config.min_pair_normal_alignment,
            max_pair_longitudinal_m=(
                self.config.max_pair_longitudinal_m),
            midpoint_reference_gate_m=(
                self.config.midpoint_reference_gate_m),
        )
        self.previous_interp_x = None
        self.previous_interp_y = None
        self.lost_frames = 0
        self.lost_time_s = 0.0
        self.age_since_paired_s = float('inf')
        self.last_path_source = 'none'
        self.last_path_confidence = 0.0
        self.last_observation = None
        self.last_path_rejection = 'none'

    def reset(self):
        self.previous_interp_x = None
        self.previous_interp_y = None
        self.lost_frames = 0
        self.lost_time_s = 0.0
        self.age_since_paired_s = float('inf')
        self.last_path_source = 'none'
        self.last_path_confidence = 0.0

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _path_arc_length(path):
        if len(path) < 2:
            return 0.0
        points = np.asarray(path, dtype=np.float64)
        return float(np.sum(np.linalg.norm(
            np.diff(points, axis=0), axis=1)))

    @classmethod
    def _path_heading(cls, path, window_m=0.28, from_end=False):
        if len(path) < 2:
            return 0.0
        points = list(reversed(path)) if from_end else list(path)
        origin = points[0]
        distance = 0.0
        target = points[-1]
        for first, second in zip(points, points[1:]):
            segment = math.hypot(
                second[0] - first[0], second[1] - first[1])
            distance += segment
            target = second
            if distance >= window_m:
                break
        heading = math.atan2(
            target[1] - origin[1], target[0] - origin[0])
        if from_end:
            heading = cls._normalize_angle(heading + math.pi)
        return heading

    @staticmethod
    def _curvature_percentile(path, percentile=95.0):
        if len(path) < 5:
            return 0.0
        points = np.asarray(path, dtype=np.float64)
        first = points[:-2]
        middle = points[1:-1]
        last = points[2:]
        first_vectors = middle - first
        second_vectors = last - middle
        chord_vectors = last - first
        denominator = (
            np.linalg.norm(first_vectors, axis=1)
            * np.linalg.norm(second_vectors, axis=1)
            * np.linalg.norm(chord_vectors, axis=1)
        )
        cross = (
            first_vectors[:, 0] * chord_vectors[:, 1]
            - first_vectors[:, 1] * chord_vectors[:, 0]
        )
        valid = denominator > 1e-9
        if not np.any(valid):
            return 0.0
        curvature = np.abs(2.0 * cross[valid] / denominator[valid])
        return float(np.percentile(curvature, percentile))

    def _motion_compensate_previous_path(
        self,
        longitudinal_motion_m,
        yaw_motion_rad,
    ):
        if self.previous_interp_x is None:
            return
        distance = float(longitudinal_motion_m)
        yaw = float(yaw_motion_rad)
        if not self.config.motion_compensation_enabled:
            return
        if abs(distance) < 1e-9 and abs(yaw) < 1e-9:
            return

        shifted_x = self.previous_interp_x - distance
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        transformed_x = cosine * shifted_x + sine * self.previous_interp_y
        transformed_y = -sine * shifted_x + cosine * self.previous_interp_y
        order = np.argsort(transformed_x)
        transformed_x = transformed_x[order]
        transformed_y = transformed_y[order]
        transformed_x, unique = np.unique(
            transformed_x, return_index=True)
        transformed_y = transformed_y[unique]
        if transformed_x.size < 2:
            self.reset()
            return
        self.previous_interp_x = transformed_x
        self.previous_interp_y = transformed_y

    def _constrain_one_side_measurement(self, candidate, reference):
        if len(candidate) < 2 or len(reference) < 2:
            return []
        reference = sorted(reference, key=lambda point: point[0])
        reference_x = np.asarray(
            [point[0] for point in reference], dtype=np.float64)
        reference_y = np.asarray(
            [point[1] for point in reference], dtype=np.float64)
        alpha = float(np.clip(
            self.config.one_side_measurement_alpha, 0.0, 1.0))
        lateral_limit = max(float(
            self.config.one_side_max_lateral_correction_m), 0.0)
        heading_limit = math.tan(math.radians(max(
            self.config.one_side_max_heading_correction_deg, 0.0)))
        constrained = []
        previous_x = None
        previous_correction = 0.0
        for x, y in sorted(candidate, key=lambda point: point[0]):
            if x < reference_x[0] or x > reference_x[-1]:
                continue
            predicted_y = float(np.interp(x, reference_x, reference_y))
            correction = alpha * float(np.clip(
                y - predicted_y, -lateral_limit, lateral_limit))
            if previous_x is not None:
                maximum_change = heading_limit * max(x - previous_x, 0.0)
                correction = float(np.clip(
                    correction,
                    previous_correction - maximum_change,
                    previous_correction + maximum_change,
                ))
            constrained.append((float(x), predicted_y + correction))
            previous_x = float(x)
            previous_correction = correction
        return constrained

    def _extend_path_horizon(self, path):
        if len(path) < 2:
            return path
        arc_length = self._path_arc_length(path)
        minimum_arc = max(float(self.config.min_path_horizon_m), 0.0)
        minimum_end_x = (
            float(self.config.prediction_min_forward_m)
            - float(self.config.rear_axle_offset_m)
        )
        heading = self._path_heading(path, window_m=0.20, from_end=True)
        heading_limit = math.radians(max(
            self.config.prediction_max_extension_heading_deg, 0.0))
        heading = float(np.clip(heading, -heading_limit, heading_limit))
        cosine = max(math.cos(heading), 1e-3)
        needed_arc = max(minimum_arc - arc_length, 0.0)
        needed_x = max(minimum_end_x - float(path[-1][0]), 0.0)
        extension = max(needed_arc, needed_x / cosine)
        if extension <= 1e-6:
            return path

        step_count = max(int(math.ceil(extension / 0.02)), 2)
        distances = np.linspace(0.0, extension, step_count + 1)[1:]
        extended = list(path)
        end_x, end_y = path[-1]
        extended.extend([
            (
                float(end_x + distance * math.cos(heading)),
                float(end_y + distance * math.sin(heading)),
            )
            for distance in distances
        ])
        return extended

    def _resample_path(self, path):
        if len(path) < 2:
            return path
        points = np.asarray(path, dtype=np.float64)
        segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        distances = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        if distances[-1] <= 1e-9:
            return []
        samples = np.linspace(
            0.0, float(distances[-1]),
            max(int(self.config.path_points), 2),
        )
        return list(zip(
            np.interp(samples, distances, points[:, 0]).tolist(),
            np.interp(samples, distances, points[:, 1]).tolist(),
        ))

    def _update_path_from_pair_anchors(self, observation, reference):
        """Update a predicted centreline from one or more cross-corridor pairs."""
        if observation.pair_count < 1 or len(reference) < 2:
            return None
        anchors = observation.paired_midpoints
        if not anchors:
            return None
        anchor_x = float(np.mean([point[0] for point in anchors]))
        anchor_y = float(np.mean([point[1] for point in anchors]))
        center, tangent, _, valid = self.corridor_geometry.reference_pose(
            anchor_x, reference)
        if not valid:
            return None

        measured_headings = []
        for left, right in zip(
            observation.paired_left_cones,
            observation.paired_right_cones,
        ):
            normal_x = float(left[0] - right[0])
            normal_y = float(left[1] - right[1])
            length = math.hypot(normal_x, normal_y)
            if length < 1e-6:
                continue
            measured_tangent = (normal_y / length, -normal_x / length)
            if measured_tangent[0] < 0.0:
                measured_tangent = (
                    -measured_tangent[0], -measured_tangent[1])
            measured_headings.append(math.atan2(
                measured_tangent[1], measured_tangent[0]))
        if measured_headings:
            measured_heading = math.atan2(
                sum(math.sin(value) for value in measured_headings),
                sum(math.cos(value) for value in measured_headings),
            )
        else:
            measured_heading = math.atan2(tangent[1], tangent[0])

        reference_heading = math.atan2(tangent[1], tangent[0])
        alpha = float(np.clip(
            self.config.one_side_measurement_alpha, 0.0, 1.0))
        lateral_limit = max(float(
            self.config.one_side_max_lateral_correction_m), 0.0)
        heading_limit = math.radians(max(
            self.config.one_side_max_heading_correction_deg, 0.0))
        lateral_correction = alpha * float(np.clip(
            anchor_y - center[1], -lateral_limit, lateral_limit))
        heading_correction = alpha * float(np.clip(
            self._normalize_angle(measured_heading - reference_heading),
            -heading_limit,
            heading_limit,
        ))

        updated = []
        slope = math.tan(heading_correction)
        for x, y in reference:
            correction = lateral_correction + slope * (x - anchor_x)
            correction = float(np.clip(
                correction, -lateral_limit, lateral_limit))
            updated.append((float(x), float(y + correction)))
        updated = self._resample_path(self._extend_path_horizon(updated))
        if not updated:
            return None
        if self._curvature_percentile(updated) > (
            self.config.max_path_curvature_per_m
        ):
            self.last_path_rejection = 'anchor_curvature'
            return None
        self.previous_interp_x = np.asarray(
            [point[0] for point in updated], dtype=np.float64)
        self.previous_interp_y = np.asarray(
            [point[1] for point in updated], dtype=np.float64)
        self.last_path_rejection = 'none'
        return updated

    @staticmethod
    def _angle_deg(point):
        angle = math.degrees(math.atan2(point[1], point[0]))
        return angle + 360.0 if angle < 0.0 else angle

    def form_cone_groups(self, cluster_centers):
        left_seed = None
        right_seed = None
        min_left_range = float('inf')
        min_right_range = float('inf')

        for point in cluster_centers:
            distance = math.hypot(point[0], point[1])
            angle = self._angle_deg(point)
            if (
                self.config.left_sector_start_deg
                <= angle
                <= self.config.left_sector_end_deg
                and distance <= self.config.left_seed_max_range_m
                and distance < min_left_range
            ):
                min_left_range = distance
                left_seed = point
            elif (
                self.config.right_sector_start_deg
                <= angle
                <= self.config.right_sector_end_deg
                and distance <= self.config.right_seed_max_range_m
                and distance < min_right_range
            ):
                min_right_range = distance
                right_seed = point

        used = set()
        left = (
            self.grow_cluster(left_seed, cluster_centers, used)
            if left_seed is not None
            else []
        )
        right = (
            self.grow_cluster(right_seed, cluster_centers, used)
            if right_seed is not None
            else []
        )
        return left, right

    def grow_cluster(self, seed, all_clusters, used):
        seed_key = tuple(seed)
        if seed_key in used:
            return []

        grown = [seed]
        queue = deque([seed])
        used.add(seed_key)
        while queue:
            base = queue.popleft()
            for point in all_clusters:
                point_key = tuple(point)
                if point_key in used:
                    continue
                if (
                    math.hypot(point[0] - base[0], point[1] - base[1])
                    <= self.config.grow_distance_m
                ):
                    grown.append(point)
                    queue.append(point)
                    used.add(point_key)
        return grown

    def add_virtual_boundary(self, left_cones, right_cones):
        left = list(left_cones)
        right = list(right_cones)
        if not left and len(right) > 1:
            left.append((
                self.config.virtual_x_m,
                self.config.virtual_half_width_m,
            ))
        elif not right and left:
            right.append((
                self.config.virtual_x_m,
                -self.config.virtual_half_width_m,
            ))
        return left, right

    def calculate_midpoints(self, left_cones, right_cones):
        if len(left_cones) >= 2 and len(right_cones) >= 2:
            return self._calculate_paired_midpoints(left_cones, right_cones)
        if len(left_cones) == 1 and len(right_cones) == 1:
            left = left_cones[0]
            right = right_cones[0]
            if self._pair_cost(left, right) is not None:
                return [(
                    (left[0] + right[0]) / 2.0,
                    (left[1] + right[1]) / 2.0,
                )]
        return []

    def _calculate_paired_midpoints(self, left_cones, right_cones):
        return [
            (
                (left[0] + right[0]) * 0.5,
                (left[1] + right[1]) * 0.5,
            )
            for left, right in self._match_boundary_pairs(
                left_cones, right_cones)
        ]

    def _match_boundary_pairs(self, left_cones, right_cones):
        """Return a maximum-score monotonic, one-to-one boundary matching."""
        left_sorted = sorted(left_cones, key=lambda point: point[0])
        right_sorted = sorted(right_cones, key=lambda point: point[0])
        left_count = len(left_sorted)
        right_count = len(right_sorted)
        scores = np.zeros((left_count + 1, right_count + 1))
        choices = np.zeros((left_count + 1, right_count + 1), dtype=np.int8)
        pair_reward = 2.75

        for left_index in range(1, left_count + 1):
            for right_index in range(1, right_count + 1):
                best_score = scores[left_index - 1, right_index]
                best_choice = 1  # skip left
                if scores[left_index, right_index - 1] > best_score:
                    best_score = scores[left_index, right_index - 1]
                    best_choice = 2  # skip right

                pair_cost = self._pair_cost(
                    left_sorted[left_index - 1],
                    right_sorted[right_index - 1],
                )
                if pair_cost is not None:
                    candidate = (
                        scores[left_index - 1, right_index - 1]
                        + pair_reward
                        - pair_cost
                    )
                    if candidate > best_score:
                        best_score = candidate
                        best_choice = 3

                scores[left_index, right_index] = best_score
                choices[left_index, right_index] = best_choice

        matches = []
        left_index = left_count
        right_index = right_count
        while left_index > 0 and right_index > 0:
            choice = choices[left_index, right_index]
            if choice == 3:
                matches.append((
                    left_sorted[left_index - 1],
                    right_sorted[right_index - 1],
                ))
                left_index -= 1
                right_index -= 1
            elif choice == 1:
                left_index -= 1
            else:
                right_index -= 1
        matches.reverse()
        return matches

    def _pair_cost(self, left, right):
        dx = abs(float(left[0]) - float(right[0]))
        distance = math.hypot(
            float(left[0]) - float(right[0]),
            float(left[1]) - float(right[1]),
        )
        if distance < self.config.min_pair_distance_m:
            return None
        if distance > self.config.max_pair_distance_m:
            return None
        if dx > self.config.max_pair_longitudinal_m:
            return None

        expected_width = max(self.config.expected_pair_distance_m, 0.10)
        width_error = abs(distance - expected_width) / expected_width
        longitudinal_error = dx / max(
            self.config.max_pair_longitudinal_m, 0.05)
        midpoint_x = (float(left[0]) + float(right[0])) * 0.5
        midpoint_y = (float(left[1]) + float(right[1])) * 0.5
        reference_error = self._reference_error(midpoint_x, midpoint_y)
        if reference_error is None:
            reference_cost = 0.0
        else:
            if reference_error > self.config.midpoint_reference_gate_m:
                return None
            reference_cost = reference_error / max(
                self.config.midpoint_reference_gate_m, 0.05)
        return (
            width_error * 1.25
            + longitudinal_error * 0.55
            + reference_cost * 0.70
        )

    def _reference_error(self, x, y):
        if self.previous_interp_x is None or self.previous_interp_y is None:
            return None
        if x < self.previous_interp_x[0] or x > self.previous_interp_x[-1]:
            return None
        previous_y = float(np.interp(
            x, self.previous_interp_x, self.previous_interp_y))
        return abs(float(y) - previous_y)

    def _one_side_midpoints(self, boundary, is_left):
        if (
            self.previous_interp_x is None
            or self.previous_interp_y is None
            or len(boundary) < 2
        ):
            return []

        previous_x = self.previous_interp_x
        previous_y = self.previous_interp_y
        half_width = max(float(self.config.virtual_half_width_m), 0.05)
        estimates = []
        for x, y in sorted(boundary, key=lambda point: point[0]):
            if x < previous_x[0] or x > previous_x[-1]:
                continue
            index = int(np.searchsorted(previous_x, x))
            before = max(0, min(index - 1, len(previous_x) - 2))
            after = before + 1
            dx = float(previous_x[after] - previous_x[before])
            dy = float(previous_y[after] - previous_y[before])
            length = math.hypot(dx, dy)
            if length < 1e-6:
                continue
            normal_x = -dy / length
            normal_y = dx / length
            direction = -1.0 if is_left else 1.0
            estimates.append((
                float(x) + direction * half_width * normal_x,
                float(y) + direction * half_width * normal_y,
            ))
        return sorted(estimates, key=lambda point: point[0])

    def interpolate_path(self, midpoints):
        self.last_path_rejection = 'none'
        if len(midpoints) < 2:
            self.last_path_rejection = 'insufficient_midpoints'
            return None

        ordered = sorted(midpoints, key=lambda point: point[0])
        xs = np.asarray([point[0] for point in ordered], dtype=np.float64)
        ys = np.asarray([point[1] for point in ordered], dtype=np.float64)
        xs, unique_indices = np.unique(xs, return_index=True)
        ys = ys[unique_indices]
        if xs.size < 2:
            if self.previous_interp_x is None:
                return None
            return list(zip(
                self.previous_interp_x.tolist(),
                self.previous_interp_y.tolist(),
            ))

        if float(xs[-1] - xs[0]) < self.config.min_path_span_m:
            self.last_path_rejection = 'short_span'
            return None

        count = max(int(self.config.path_points), 2)
        interp_x = np.linspace(float(xs.min()), float(xs.max()), count)
        if PchipInterpolator is not None and xs.size >= 3:
            interp_y = PchipInterpolator(xs, ys)(interp_x)
        else:
            interp_y = np.interp(interp_x, xs, ys)

        if (
            not np.all(np.isfinite(interp_y))
            or float(np.max(np.abs(interp_y)))
            > self.config.max_abs_path_y_m
        ):
            self.last_path_rejection = 'bounds'
            return None

        if self.previous_interp_x is not None:
            first_x = max(float(interp_x[0]), float(self.previous_interp_x[0]))
            last_x = min(float(interp_x[-1]), float(self.previous_interp_x[-1]))
            if first_x < last_x:
                comparison_x = np.linspace(first_x, last_x, 25)
                current_y = np.interp(comparison_x, interp_x, interp_y)
                previous_y = np.interp(
                    comparison_x,
                    self.previous_interp_x,
                    self.previous_interp_y,
                )
                if float(np.max(np.abs(current_y - previous_y))) > (
                    self.config.max_path_lateral_jump_m
                ):
                    self.last_path_rejection = 'lateral_jump'
                    return None

            alpha = float(np.clip(
                self.config.path_smoothing_alpha, 0.0, 1.0))
            overlap = np.logical_and(
                interp_x >= self.previous_interp_x[0],
                interp_x <= self.previous_interp_x[-1],
            )
            if np.any(overlap):
                previous_y = np.interp(
                    interp_x[overlap],
                    self.previous_interp_x,
                    self.previous_interp_y,
                )
                interp_y[overlap] = (
                    alpha * interp_y[overlap]
                    + (1.0 - alpha) * previous_y
                )

        path = [
            (float(x), float(y))
            for x, y in zip(interp_x, interp_y)
        ]
        path = self._resample_path(self._extend_path_horizon(path))
        if not path:
            self.last_path_rejection = 'resample'
            return None

        if self.previous_interp_x is not None:
            previous_path = list(zip(
                self.previous_interp_x.tolist(),
                self.previous_interp_y.tolist(),
            ))
            heading_change = abs(self._normalize_angle(
                self._path_heading(path)
                - self._path_heading(previous_path)
            ))
            if heading_change > math.radians(
                self.config.max_path_heading_jump_deg
            ):
                self.last_path_rejection = 'heading_jump'
                return None

        if self._curvature_percentile(path) > (
            self.config.max_path_curvature_per_m
        ):
            self.last_path_rejection = 'curvature'
            return None

        self.previous_interp_x = np.asarray(
            [point[0] for point in path], dtype=np.float64)
        self.previous_interp_y = np.asarray(
            [point[1] for point in path], dtype=np.float64)
        return path

    def build(
        self,
        cluster_centers,
        longitudinal_motion_m=0.0,
        yaw_motion_rad=0.0,
        elapsed_s=0.10,
    ):
        elapsed_s = max(float(elapsed_s), 0.0)
        self.last_path_rejection = 'none'
        self._motion_compensate_previous_path(
            longitudinal_motion_m, yaw_motion_rad)
        if math.isfinite(self.age_since_paired_s):
            self.age_since_paired_s += elapsed_s
        centers = [
            (float(point[0]), float(point[1]))
            for point in cluster_centers
            if len(point) >= 2
            and math.isfinite(float(point[0]))
            and math.isfinite(float(point[1]))
        ]
        previous_path = []
        if self.previous_interp_x is not None:
            previous_path = list(zip(
                self.previous_interp_x.tolist(),
                self.previous_interp_y.tolist(),
            ))
        observation = self.corridor_geometry.extract(
            centers, previous_path)
        self.last_observation = observation
        left = observation.left_cones
        right = observation.right_cones
        midpoints = list(observation.paired_midpoints)
        direct_measurement = observation.pair_count >= 2
        path = None
        source = 'none'

        if direct_measurement and len(midpoints) >= 2:
            path = self.interpolate_path(midpoints)
            if path:
                source = 'paired'

        if (
            path is None
            and observation.pair_count >= 1
            and self.previous_interp_x is not None
        ):
            path = self._update_path_from_pair_anchors(
                observation, previous_path)
            if path:
                source = 'pair_anchor'

        if (
            path is None
            and observation.pair_count == 0
            and self.previous_interp_x is not None
            and self.age_since_paired_s
            <= self.config.one_side_max_age_since_paired_s
        ):
            recovered = []
            if len(left) >= 2:
                recovered.append(
                    self.corridor_geometry.estimate_center_from_one_side(
                        left_cones=left,
                        right_cones=[],
                        reference_path=previous_path,
                        half_width_m=self.config.virtual_half_width_m,
                        min_points=2,
                    )
                )
            if len(right) >= 2:
                recovered.append(
                    self.corridor_geometry.estimate_center_from_one_side(
                        left_cones=[],
                        right_cones=right,
                        reference_path=previous_path,
                        half_width_m=self.config.virtual_half_width_m,
                        min_points=2,
                    )
                )
            recovered = [
                candidate for candidate in recovered if len(candidate) >= 2
            ]
            if recovered:
                def continuity_error(candidate):
                    return float(np.mean([
                        self._reference_error(x, y) or 0.0
                        for x, y in candidate
                    ]))
                midpoints = self._constrain_one_side_measurement(
                    min(recovered, key=continuity_error),
                    previous_path,
                )
                if len(midpoints) >= 2:
                    path = self.interpolate_path(midpoints)
                    if path:
                        source = 'one_side'

        if path:
            self.lost_frames = 0
            self.lost_time_s = 0.0
            if source == 'paired':
                self.age_since_paired_s = 0.0
                self.last_path_source = source
                self.last_path_confidence = min(
                    1.0, 0.65 + 0.12 * observation.pair_count)
            elif source == 'pair_anchor':
                self.age_since_paired_s = 0.0
                self.last_path_source = source
                self.last_path_confidence = min(
                    0.72, 0.50 + 0.08 * observation.pair_count)
            else:
                age_ratio = self.age_since_paired_s / max(
                    self.config.one_side_max_age_since_paired_s, 1e-3)
                self.last_path_source = 'one_side'
                self.last_path_confidence = max(
                    0.35, 0.65 * (1.0 - 0.45 * age_ratio))
        elif (
            self.previous_interp_x is not None
            and self.lost_frames < self.config.path_lost_keep_frames
            and self.lost_time_s < self.config.path_lost_keep_s
            and self.age_since_paired_s
            <= self.config.one_side_max_age_since_paired_s
        ):
            self.lost_frames += 1
            self.lost_time_s += elapsed_s
            path = list(zip(
                self.previous_interp_x.tolist(),
                self.previous_interp_y.tolist(),
            ))
            self.last_path_source = 'predicted'
            self.last_path_confidence = max(
                0.20,
                0.45 * (
                    1.0 - self.lost_time_s
                    / max(self.config.path_lost_keep_s, 1e-3)
                ),
            )
        else:
            path = []

        if not path and self.previous_interp_x is not None:
            self.reset()
            self.last_observation = observation

        rear_axle_path = [
            (x + self.config.rear_axle_offset_m, y)
            for x, y in path
        ]
        return left, right, midpoints, rear_axle_path
