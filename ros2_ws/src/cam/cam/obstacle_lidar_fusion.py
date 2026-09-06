"""LiDAR-first clustering and camera-direction obstacle association."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LidarCluster:
    """One contiguous object candidate from an angularly ordered scan."""

    scan_indices: np.ndarray
    points_xy: np.ndarray
    ranges_m: np.ndarray
    bearings_rad: np.ndarray
    distance_m: float
    width_m: float


@dataclass(frozen=True)
class ProjectedCluster:
    """A LiDAR cluster summarized in camera-image coordinates."""

    cluster_index: int
    image_points: np.ndarray
    center_x: float
    left_x: float
    right_x: float
    center_y: float
    top_y: float
    bottom_y: float
    distance_m: float
    point_count: int
    width_m: float


@dataclass(frozen=True)
class DirectionAssociation:
    """A one-to-one match between a YOLO box and a LiDAR cluster."""

    detection_index: int
    cluster_index: int
    score: float


@dataclass(frozen=True)
class TemporalClusterTrackingConfig:
    """Limits used to keep one camera target on one LiDAR object."""

    max_age_s: float = 0.60
    max_misses: int = 3
    max_distance_jump_m: float = 0.45
    max_range_rate_mps: float = 3.0
    max_total_distance_jump_m: float = 1.50
    max_offset_jump_px: float = 55.0
    max_offset_rate_px_s: float = 120.0
    bbox_iou_min: float = 0.10
    bbox_center_gate_px: float = 150.0
    # Optional acquisition-only camera-scale heuristic. Keep it disabled by
    # default: a crossing obstacle can be physically close while its box is
    # still small, so temporal continuity is the safer default filter.
    near_cluster_distance_m: float = 0.0
    near_cluster_min_bbox_height_ratio: float = 0.16
    near_cluster_min_bbox_bottom_ratio: float = 0.56


@dataclass
class _TemporalClusterTrack:
    """Mutable state for the single obstacle eligible for lane avoidance."""

    bbox: tuple
    relative_center_x_px: float
    distance_m: float
    source_ns: int
    misses: int = 0


@dataclass(frozen=True)
class _DirectionCandidate:
    """One geometrically valid camera-to-cluster pairing."""

    detection_index: int
    cluster_index: int
    score: float
    relative_center_x_px: float
    distance_m: float


@dataclass(frozen=True)
class LidarSnapshot:
    """Clusters and timing metadata produced from one LaserScan."""

    source_ns: int
    received_ns: int
    clusters: tuple


def select_time_aligned_state(
    samples,
    target_ns,
    max_delta_s,
    fallback,
):
    """Select the nearest timestamped state without using stale geometry."""
    if not samples:
        return str(fallback)
    target = int(target_ns)
    if target <= 0:
        return str(samples[-1][2])
    timestamped = [sample for sample in samples if int(sample[0]) > 0]
    if not timestamped:
        return str(fallback)
    source_ns, _received_ns, state = min(
        timestamped,
        key=lambda item: abs(int(item[0]) - target),
    )
    delta_s = abs(int(source_ns) - target) * 1e-9
    if delta_s > max(0.0, float(max_delta_s)):
        return str(fallback)
    return str(state)


def camera_detection_is_near(box, image_height, min_bottom_ratio):
    """Return whether a YOLO obstacle is low enough to require a speed cap."""
    if float(image_height) <= 0.0:
        return False
    _xmin, _ymin, _xmax, ymax = [float(value) for value in box]
    ratio = float(np.clip(min_bottom_ratio, 0.0, 1.0))
    return ymax / float(image_height) >= ratio


def _bbox_iou(first, second):
    """Return intersection-over-union for two valid XYXY boxes."""
    intersection_width = max(
        0.0, min(first[2], second[2]) - max(first[0], second[0]))
    intersection_height = max(
        0.0, min(first[3], second[3]) - max(first[1], second[1]))
    intersection = intersection_width * intersection_height
    first_area = max(0.0, first[2] - first[0]) * max(
        0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(
        0.0, second[3] - second[1])
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def _same_camera_target(first, second, iou_min, center_gate_px):
    """Match a YOLO target through modest scale and center changes."""
    first_box = tuple(float(value) for value in first)
    second_box = tuple(float(value) for value in second)
    if _bbox_iou(first_box, second_box) >= max(0.0, float(iou_min)):
        return True
    first_center = (
        0.5 * (first_box[0] + first_box[2]),
        0.5 * (first_box[1] + first_box[3]),
    )
    second_center = (
        0.5 * (second_box[0] + second_box[2]),
        0.5 * (second_box[1] + second_box[3]),
    )
    center_distance = float(np.hypot(
        first_center[0] - second_center[0],
        first_center[1] - second_center[1],
    ))
    return center_distance <= max(0.0, float(center_gate_px))


def _camera_target_score(reference, candidate):
    """Rank candidate boxes by overlap first and center distance second."""
    reference_box = tuple(float(value) for value in reference)
    candidate_box = tuple(float(value) for value in candidate)
    reference_center = (
        0.5 * (reference_box[0] + reference_box[2]),
        0.5 * (reference_box[1] + reference_box[3]),
    )
    candidate_center = (
        0.5 * (candidate_box[0] + candidate_box[2]),
        0.5 * (candidate_box[1] + candidate_box[3]),
    )
    center_distance = float(np.hypot(
        reference_center[0] - candidate_center[0],
        reference_center[1] - candidate_center[1],
    ))
    return (-_bbox_iou(reference_box, candidate_box), center_distance)


def _make_cluster(
    indices,
    ranges,
    angles,
    distance_percentile,
):
    """Build an immutable cluster summary from scan indices."""
    cluster_indices = np.asarray(indices, dtype=np.int32)
    cluster_ranges = ranges[cluster_indices]
    cluster_angles = angles[cluster_indices]
    points_xy = np.column_stack((
        cluster_ranges * np.cos(cluster_angles),
        cluster_ranges * np.sin(cluster_angles),
    ))
    if points_xy.shape[0] >= 2:
        width_m = float(np.linalg.norm(points_xy[-1] - points_xy[0]))
    else:
        width_m = 0.0
    return LidarCluster(
        scan_indices=cluster_indices,
        points_xy=points_xy,
        ranges_m=cluster_ranges,
        bearings_rad=cluster_angles,
        distance_m=float(np.percentile(
            cluster_ranges, distance_percentile)),
        width_m=width_m,
    )


def cluster_ordered_scan(
    ranges,
    angle_min,
    angle_increment,
    min_range_m=0.18,
    max_range_m=5.0,
    min_points=2,
    base_gap_m=0.08,
    range_gap_scale=1.5,
    max_width_m=2.0,
    distance_percentile=50.0,
):
    """
    Split a LaserScan into contiguous objects in one O(N) pass.

    The allowed gap grows with angular ray spacing and object range. This is
    cheaper and more deterministic than applying a general DBSCAN to every
    camera bounding box.
    """
    scan_ranges = np.asarray(ranges, dtype=np.float64)
    if scan_ranges.ndim != 1 or scan_ranges.size == 0:
        return []
    increment = float(angle_increment)
    angles = float(angle_min) + np.arange(scan_ranges.size) * increment
    valid = (
        np.isfinite(scan_ranges)
        & (scan_ranges >= float(min_range_m))
        & (scan_ranges <= float(max_range_m))
    )
    min_count = max(1, int(min_points))
    base_gap = max(0.0, float(base_gap_m))
    angular_scale = max(0.0, float(range_gap_scale))
    width_limit = max(0.0, float(max_width_m))
    percentile = float(np.clip(distance_percentile, 0.0, 100.0))

    groups = []
    current = []
    previous_index = None
    for index in range(scan_ranges.size):
        if not valid[index]:
            if current:
                groups.append(current)
                current = []
            previous_index = None
            continue
        if previous_index is None:
            current = [index]
            previous_index = index
            continue

        previous_range = scan_ranges[previous_index]
        current_range = scan_ranges[index]
        angle_delta = angles[index] - angles[previous_index]
        point_gap = np.sqrt(
            previous_range * previous_range
            + current_range * current_range
            - 2.0 * previous_range * current_range * np.cos(angle_delta)
        )
        allowed_gap = (
            base_gap
            + angular_scale
            * min(previous_range, current_range)
            * abs(angle_delta)
        )
        if point_gap <= allowed_gap:
            current.append(index)
        else:
            groups.append(current)
            current = [index]
        previous_index = index
    if current:
        groups.append(current)

    clusters = []
    for indices in groups:
        if len(indices) < min_count:
            continue
        cluster = _make_cluster(
            indices,
            scan_ranges,
            angles,
            percentile,
        )
        if width_limit > 0.0 and cluster.width_m > width_limit:
            continue
        clusters.append(cluster)
    return clusters


def _direction_candidates(
    boxes,
    projected_clusters,
    padding_px=35.0,
    max_above_bbox_px=10.0,
):
    """Build geometry-first association candidates without distance bias."""
    padding = max(0.0, float(padding_px))
    max_above = max(0.0, float(max_above_bbox_px))
    candidates = []
    for detection_index, box in enumerate(boxes):
        xmin, _ymin, xmax, _ymax = [float(value) for value in box]
        if xmax <= xmin:
            continue
        box_center = 0.5 * (xmin + xmax)
        half_width = 0.5 * (xmax - xmin)
        gate_radius = max(1.0, half_width + padding)
        gate_left = xmin - padding
        gate_right = xmax + padding
        for cluster_index, projected in enumerate(projected_clusters):
            if projected.center_y < _ymin - max_above:
                continue
            overlap = max(
                0.0,
                min(gate_right, projected.right_x)
                - max(gate_left, projected.left_x),
            )
            center_error = abs(projected.center_x - box_center)
            if center_error > gate_radius:
                continue
            normalized_error = center_error / gate_radius
            overlap_denominator = max(
                1.0, projected.right_x - projected.left_x)
            overlap_ratio = min(1.0, overlap / overlap_denominator)
            # Distance must not decide object identity. A nearby structure in
            # the same general direction otherwise beats the visually aligned
            # vehicle. Range is used only for temporal continuity below.
            score = normalized_error - 0.35 * overlap_ratio
            candidates.append(_DirectionCandidate(
                detection_index=detection_index,
                cluster_index=cluster_index,
                score=float(score),
                relative_center_x_px=float(
                    projected.center_x - box_center),
                distance_m=float(projected.distance_m),
            ))
    return candidates


def associate_by_horizontal_direction(
    boxes,
    projected_clusters,
    padding_px=35.0,
    max_above_bbox_px=10.0,
):
    """
    Associate YOLO boxes and clusters by camera-horizontal direction.

    Full 2-D bbox inclusion is deliberately not a gate. Sparse LiDAR returns
    may land below the visual object. A small upper-bound guard only rejects
    clusters that project clearly above the detected object.
    """
    candidates = _direction_candidates(
        boxes,
        projected_clusters,
        padding_px=padding_px,
        max_above_bbox_px=max_above_bbox_px,
    )

    associations = []
    used_detections = set()
    used_clusters = set()
    for candidate in sorted(
            candidates, key=lambda item: (item.score, item.cluster_index)):
        detection_index = candidate.detection_index
        cluster_index = candidate.cluster_index
        if detection_index in used_detections:
            continue
        if cluster_index in used_clusters:
            continue
        used_detections.add(detection_index)
        used_clusters.add(cluster_index)
        associations.append(DirectionAssociation(
            detection_index=detection_index,
            cluster_index=cluster_index,
            score=float(candidate.score),
        ))
    return associations


class TemporalClusterTracker:
    """Keep one YOLO obstacle associated with a continuous LiDAR object."""

    def __init__(self, config=None):
        self.config = config or TemporalClusterTrackingConfig()
        self.track = None
        self.last_rejection_reason = ''

    def reset(self):
        """Forget the current camera/LiDAR target."""
        self.track = None
        self.last_rejection_reason = ''

    def _elapsed_s(self, source_ns):
        if self.track is None:
            return 0.0
        current = int(source_ns)
        previous = int(self.track.source_ns)
        if current <= 0 or previous <= 0:
            return 0.0
        return max(0.0, (current - previous) * 1e-9)

    def _track_expired(self, source_ns):
        if self.track is None:
            return True
        elapsed_s = self._elapsed_s(source_ns)
        return (
            elapsed_s > max(0.0, float(self.config.max_age_s))
            or self.track.misses >= max(1, int(self.config.max_misses))
        )

    def _note_miss(self, source_ns):
        if self.track is None:
            return
        self.track.misses += 1
        if self._track_expired(source_ns):
            self.reset()

    def _distance_matches_camera_scale(
            self, box, candidate, allowed_distance_error):
        """Accept a large closing step only when bbox growth predicts it."""
        previous_height = max(
            1.0, float(self.track.bbox[3]) - float(self.track.bbox[1]))
        current_height = max(1.0, float(box[3]) - float(box[1]))
        height_ratio = current_height / previous_height
        scale_change = max(height_ratio, 1.0 / height_ratio)
        if scale_change < 1.35:
            return False
        predicted_distance = (
            float(self.track.distance_m) * previous_height / current_height)
        prediction_error = abs(
            float(candidate.distance_m) - predicted_distance)
        return prediction_error <= max(0.0, float(allowed_distance_error))

    def _candidate_is_continuous(self, box, candidate, source_ns):
        elapsed_s = self._elapsed_s(source_ns)
        kinematic_distance_jump = max(
            0.0, float(self.config.max_distance_jump_m)) + max(
                0.0, float(self.config.max_range_rate_mps)) * elapsed_s
        hard_distance_jump = max(
            0.0, float(self.config.max_total_distance_jump_m))
        allowed_distance_jump = kinematic_distance_jump
        if hard_distance_jump > 0.0:
            allowed_distance_jump = min(
                allowed_distance_jump, hard_distance_jump)
        distance_jump = abs(
            float(candidate.distance_m) - float(self.track.distance_m))
        if distance_jump > allowed_distance_jump:
            scale_consistent = (
                hard_distance_jump > 0.0
                and distance_jump > hard_distance_jump
                and self._distance_matches_camera_scale(
                    box, candidate, kinematic_distance_jump)
            )
            if not scale_consistent:
                return False, 'distance_jump'

        allowed_offset_jump = max(
            0.0, float(self.config.max_offset_jump_px)) + max(
                0.0, float(self.config.max_offset_rate_px_s)) * elapsed_s
        offset_jump = abs(
            float(candidate.relative_center_x_px)
            - float(self.track.relative_center_x_px)
        )
        if offset_jump > allowed_offset_jump:
            return False, 'direction_jump'
        return True, ''

    def _candidate_is_camera_scale_plausible(
            self, box, candidate, image_height):
        """Reject a very near cluster paired with a visually distant box."""
        near_limit = max(
            0.0, float(self.config.near_cluster_distance_m))
        if candidate.distance_m >= near_limit or near_limit <= 0.0:
            return True
        height = float(image_height)
        if height <= 0.0:
            return True
        bbox_height_ratio = max(0.0, box[3] - box[1]) / height
        bbox_bottom_ratio = float(box[3]) / height
        return (
            bbox_height_ratio >= float(
                self.config.near_cluster_min_bbox_height_ratio)
            or bbox_bottom_ratio >= float(
                self.config.near_cluster_min_bbox_bottom_ratio)
        )

    def _continuity_score(self, candidate, source_ns):
        elapsed_s = self._elapsed_s(source_ns)
        distance_scale = max(
            1e-6,
            max(0.0, float(self.config.max_distance_jump_m))
            + max(0.0, float(self.config.max_range_rate_mps)) * elapsed_s,
        )
        offset_scale = max(
            1e-6,
            max(0.0, float(self.config.max_offset_jump_px))
            + max(0.0, float(self.config.max_offset_rate_px_s)) * elapsed_s,
        )
        distance_error = abs(
            candidate.distance_m - self.track.distance_m) / distance_scale
        offset_error = abs(
            candidate.relative_center_x_px
            - self.track.relative_center_x_px) / offset_scale
        return float(candidate.score + 0.80 * distance_error + offset_error)

    def _accept(self, box, candidate, source_ns):
        self.track = _TemporalClusterTrack(
            bbox=tuple(float(value) for value in box),
            relative_center_x_px=float(candidate.relative_center_x_px),
            distance_m=float(candidate.distance_m),
            source_ns=int(source_ns),
            misses=0,
        )
        self.last_rejection_reason = ''
        return [DirectionAssociation(
            detection_index=candidate.detection_index,
            cluster_index=candidate.cluster_index,
            score=float(candidate.score),
        )]

    def associate(
        self,
        boxes,
        projected_clusters,
        source_ns,
        padding_px=35.0,
        max_above_bbox_px=10.0,
        image_height=0.0,
    ):
        """Associate a single avoidance target while rejecting track jumps."""
        if not boxes:
            self._note_miss(source_ns)
            return []

        # Before a track exists, preserve the stateless multi-object behavior.
        # Once one target has been established, temporary duplicate detections
        # must not bypass continuity checks and replace its range measurement.
        if self.track is None and len(boxes) != 1:
            return associate_by_horizontal_direction(
                boxes, projected_clusters,
                padding_px=padding_px,
                max_above_bbox_px=max_above_bbox_px)

        if self.track is not None and self._track_expired(source_ns):
            self.reset()
            # Do not turn an expired track into a very different range in the
            # same published frame.  One explicit miss separates identities.
            self.last_rejection_reason = 'track_expired'
            return []

        detection_index = 0
        if self.track is not None and len(boxes) > 1:
            matching_detection_indices = [
                index
                for index, candidate_box in enumerate(boxes)
                if _same_camera_target(
                    self.track.bbox,
                    candidate_box,
                    self.config.bbox_iou_min,
                    self.config.bbox_center_gate_px,
                )
            ]
            if not matching_detection_indices:
                self.last_rejection_reason = 'camera_target_jump'
                self._note_miss(source_ns)
                return []
            detection_index = min(
                matching_detection_indices,
                key=lambda index: _camera_target_score(
                    self.track.bbox, boxes[index]),
            )

        box = tuple(float(value) for value in boxes[detection_index])
        if self.track is not None and not _same_camera_target(
            self.track.bbox,
            box,
            self.config.bbox_iou_min,
            self.config.bbox_center_gate_px,
        ):
            self.reset()
            # Likewise, a camera target change must be visible as a miss before
            # a new LiDAR range is allowed to replace the previous target.
            self.last_rejection_reason = 'camera_target_jump'
            return []

        candidates = _direction_candidates(
            boxes,
            projected_clusters,
            padding_px=padding_px,
            max_above_bbox_px=max_above_bbox_px,
        )
        candidates = [
            candidate
            for candidate in candidates
            if candidate.detection_index == detection_index
            and self._candidate_is_camera_scale_plausible(
                box, candidate, image_height)
        ]
        if not candidates:
            if self.track is not None:
                self.track.bbox = box
            self.last_rejection_reason = 'no_plausible_direction_candidate'
            self._note_miss(source_ns)
            return []

        if self.track is None:
            selected = min(
                candidates,
                key=lambda item: (item.score, item.cluster_index),
            )
            return self._accept(box, selected, source_ns)

        continuous = []
        rejection_reasons = set()
        for candidate in candidates:
            accepted, reason = self._candidate_is_continuous(
                box, candidate, source_ns)
            if accepted:
                continuous.append((
                    self._continuity_score(candidate, source_ns),
                    candidate,
                ))
            else:
                rejection_reasons.add(reason)
        if continuous:
            _score, selected = min(
                continuous,
                key=lambda item: (item[0], item[1].cluster_index),
            )
            return self._accept(box, selected, source_ns)

        # Camera identity can remain valid while a LiDAR observation is
        # rejected. Keep following the visual target without changing range.
        self.track.bbox = box
        self.last_rejection_reason = '+'.join(sorted(rejection_reasons))
        self._note_miss(source_ns)
        return []
