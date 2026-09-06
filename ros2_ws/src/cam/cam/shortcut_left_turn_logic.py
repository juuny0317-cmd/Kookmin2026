"""Branch-aware state and path helpers for the shortcut left turn."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
from statistics import median
from typing import Iterable, Sequence


class ShortcutTurnState(str, Enum):
    """High-level phases of the event-driven left turn."""

    IDLE = 'IDLE'
    PREPARE_LEFT = 'PREPARE_LEFT'
    FOLLOW_BRANCH = 'FOLLOW_BRANCH'
    SHORTCUT_CRUISE = 'SHORTCUT_CRUISE'
    EXIT_LEFT_COMMIT = 'EXIT_LEFT_COMMIT'
    EXIT_HANDOFF = 'EXIT_HANDOFF'
    COMPLETE = 'COMPLETE'
    FAILED = 'FAILED'


@dataclass(frozen=True)
class DetectionBox:
    """ROS-independent center-line detection box."""

    xmin: float
    ymin: float
    xmax: float
    ymax: float
    confidence: float = 1.0


@dataclass(frozen=True)
class CurveObservation:
    """Compact image-space geometry summary."""

    valid: bool = False
    point_count: int = 0
    heading_deg: float = 0.0
    near_x_ratio: float = 0.5
    y_span_ratio: float = 0.0


@dataclass(frozen=True)
class BranchObservation:
    """Left branch separated from the current-road center-line family."""

    valid: bool = False
    split: bool = False
    points: tuple[tuple[float, float], ...] = ()
    point_count: int = 0
    left_count: int = 0
    right_count: int = 0
    gap_ratio: float = 0.0
    fit_error_ratio: float = 0.0
    near_x_ratio: float = 0.5
    heading_deg: float = 0.0
    split_x_ratio: float = 0.0


@dataclass(frozen=True)
class LineObservation:
    """A coherent center-line family measured in image coordinates."""

    valid: bool = False
    points: tuple[tuple[float, float], ...] = ()
    point_count: int = 0
    angle_from_horizontal_deg: float = 0.0
    heading_deg: float = 0.0
    x_span_ratio: float = 0.0
    y_span_ratio: float = 0.0
    center_y_ratio: float = 0.0
    fit_error_ratio: float = 1.0
    near_x_ratio: float = 0.5


@dataclass(frozen=True)
class ShortcutTurnConfig:
    """Conservative defaults tuned from the four shortcut entry bags."""

    left_signal_window: int = 5
    left_signal_min_matches: int = 3
    min_prepare_duration_s: float = 0.8
    fallback_commit_s: float = 5.0
    strong_gap_ratio: float = 0.26
    weak_gap_ratio: float = 0.18
    strong_fit_error_ratio: float = 0.055
    weak_fit_error_ratio: float = 0.035
    strong_confirm_frames: int = 2
    weak_confirm_frames: int = 5
    prepare_speed_limit: float = 3.0
    turn_speed_limit: float = 3.0
    cruise_speed_limit: float = -1.0
    min_follow_duration_s: float = 3.0
    max_follow_duration_s: float = 12.0
    branch_loss_timeout_s: float = 2.0
    fallback_branch_grace_s: float = 3.5
    exit_max_heading_deg: float = 22.0
    exit_single_fit_error_ratio: float = 0.040
    exit_min_points: int = 3
    exit_near_x_min_ratio: float = 0.08
    exit_near_x_max_ratio: float = 0.90
    exit_confirm_frames: int = 5
    entry_alignment_window: int = 5
    entry_alignment_min_matches: int = 3
    entry_max_heading_deg: float = 60.0
    min_cruise_duration_s: float = 4.0
    session_timeout_s: float = 45.0
    crossline_window: int = 3
    crossline_min_matches: int = 2
    crossline_max_angle_deg: float = 22.0
    crossline_min_x_span_ratio: float = 0.30
    crossline_max_y_span_ratio: float = 0.16
    crossline_center_y_min_ratio: float = 0.32
    crossline_center_y_max_ratio: float = 0.72
    exit_min_turn_duration_s: float = 1.5
    exit_capture_max_heading_deg: float = 55.0
    exit_alignment_window: int = 5
    exit_alignment_min_matches: int = 3
    exit_handoff_duration_s: float = 0.8
    exit_timeout_s: float = 15.0
    rearm_non_left_frames: int = 5


@dataclass(frozen=True)
class ShortcutTurnDecision:
    """Action selected for one source frame."""

    state: ShortcutTurnState
    override_command: str | None
    path_active: bool
    speed_limit: float
    use_fallback_path: bool
    branch_confirm_count: int
    exit_confirm_count: int
    crossline_confirm_count: int
    session_active: bool
    handoff_progress: float
    reason: str


def _finite_points(
    points: Iterable[Sequence[float]],
) -> list[tuple[float, float]]:
    result = []
    for point in points:
        if len(point) < 2:
            continue
        x_value = float(point[0])
        y_value = float(point[1])
        if math.isfinite(x_value) and math.isfinite(y_value):
            result.append((x_value, y_value))
    return result


def observe_curve(
    points: Iterable[Sequence[float]],
    width: int,
    height: int,
    min_points: int = 4,
    min_y_span_ratio: float = 0.12,
) -> CurveObservation:
    """Measure path heading from its near and far image-space quarters."""
    if width <= 0 or height <= 0:
        return CurveObservation()
    finite = _finite_points(points)
    if len(finite) < max(2, int(min_points)):
        return CurveObservation(point_count=len(finite))

    ordered = sorted(finite, key=lambda point: point[1], reverse=True)
    group_size = max(1, len(ordered) // 4)
    near_group = ordered[:group_size]
    far_group = ordered[-group_size:]
    near_x = median(point[0] for point in near_group)
    near_y = median(point[1] for point in near_group)
    far_x = median(point[0] for point in far_group)
    far_y = median(point[1] for point in far_group)
    forward = near_y - far_y
    y_span_ratio = max(0.0, forward / float(height))
    heading_deg = math.degrees(math.atan2(
        far_x - near_x,
        max(1e-6, forward),
    ))
    return CurveObservation(
        valid=(forward > 0.0 and y_span_ratio >= min_y_span_ratio),
        point_count=len(finite),
        heading_deg=heading_deg,
        near_x_ratio=near_x / float(width),
        y_span_ratio=y_span_ratio,
    )


def _line_fit_error(points: Sequence[tuple[float, float]]) -> float:
    if len(points) < 3:
        return 0.0
    mean_y = sum(point[1] for point in points) / len(points)
    mean_x = sum(point[0] for point in points) / len(points)
    variance_y = sum((point[1] - mean_y) ** 2 for point in points)
    if variance_y <= 1e-9:
        return math.sqrt(sum(
            (point[0] - mean_x) ** 2 for point in points
        ) / len(points))
    slope = sum(
        (point[1] - mean_y) * (point[0] - mean_x)
        for point in points
    ) / variance_y
    intercept = mean_x - slope * mean_y
    return math.sqrt(sum(
        (point[0] - (slope * point[1] + intercept)) ** 2
        for point in points
    ) / len(points))


def _path_geometry(
    points: Sequence[tuple[float, float]],
    width: int,
) -> tuple[float, float]:
    near = max(points, key=lambda point: point[1])
    far = min(points, key=lambda point: point[1])
    forward = near[1] - far[1]
    heading = math.degrees(math.atan2(
        far[0] - near[0], max(1e-6, forward)))
    return near[0] / float(width), heading


def _box_centers(
    boxes: Iterable[DetectionBox],
    min_confidence: float,
) -> list[tuple[float, float]]:
    centers = []
    for box in boxes:
        if (
            box.confidence < min_confidence
            or box.xmax <= box.xmin
            or box.ymax <= box.ymin
        ):
            continue
        centers.append((
            0.5 * (box.xmin + box.xmax),
            0.5 * (box.ymin + box.ymax),
        ))
    return centers


def _axis_angle_deg(dx: float, dy: float) -> float:
    angle = math.degrees(math.atan2(dy, dx))
    while angle > 90.0:
        angle -= 180.0
    while angle < -90.0:
        angle += 180.0
    return angle


def _principal_line(
    points: Sequence[tuple[float, float]],
    width: int,
    height: int,
) -> LineObservation:
    if len(points) < 2 or width <= 0 or height <= 0:
        return LineObservation(point_count=len(points))
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    cov_xx = sum((point[0] - mean_x) ** 2 for point in points)
    cov_xy = sum(
        (point[0] - mean_x) * (point[1] - mean_y)
        for point in points
    )
    cov_yy = sum((point[1] - mean_y) ** 2 for point in points)
    theta = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)
    axis_x = math.cos(theta)
    axis_y = math.sin(theta)
    if axis_y > 0.0:
        axis_x = -axis_x
        axis_y = -axis_y
    heading_deg = math.degrees(math.atan2(axis_x, max(1e-9, -axis_y)))
    normal_x = -axis_y
    normal_y = axis_x
    residuals = [
        (point[0] - mean_x) * normal_x
        + (point[1] - mean_y) * normal_y
        for point in points
    ]
    fit_error = math.sqrt(
        sum(value * value for value in residuals) / len(residuals))
    near = max(points, key=lambda point: point[1])
    return LineObservation(
        points=tuple(points),
        point_count=len(points),
        angle_from_horizontal_deg=abs(_axis_angle_deg(axis_x, axis_y)),
        heading_deg=heading_deg,
        x_span_ratio=(
            max(point[0] for point in points)
            - min(point[0] for point in points)
        ) / float(width),
        y_span_ratio=(
            max(point[1] for point in points)
            - min(point[1] for point in points)
        ) / float(height),
        center_y_ratio=mean_y / float(height),
        fit_error_ratio=fit_error / float(width),
        near_x_ratio=near[0] / float(width),
    )


def _best_line_family(
    points: Sequence[tuple[float, float]],
    width: int,
    height: int,
    horizontal: bool,
) -> LineObservation:
    if len(points) < 3:
        return LineObservation(point_count=len(points))
    distance_limit = (0.055 if horizontal else 0.075) * height
    best: list[tuple[float, float]] = []
    best_span = -1.0
    for first_index, first in enumerate(points[:-1]):
        for second in points[first_index + 1:]:
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            norm = math.hypot(dx, dy)
            if norm < 1e-6:
                continue
            angle = abs(_axis_angle_deg(dx, dy))
            if horizontal and angle > 32.0:
                continue
            if not horizontal and abs(90.0 - angle) > 42.0:
                continue
            inliers = [
                point for point in points
                if abs(
                    dy * (point[0] - first[0])
                    - dx * (point[1] - first[1])
                ) / norm <= distance_limit
            ]
            projection = [
                (point[0] * dx + point[1] * dy) / norm
                for point in inliers
            ]
            span = max(projection) - min(projection) if projection else 0.0
            if (len(inliers), span) > (len(best), best_span):
                best = inliers
                best_span = span
    return _principal_line(best, width, height)


def observe_crossline(
    boxes: Iterable[DetectionBox],
    width: int,
    height: int,
    min_confidence: float = 0.25,
    max_angle_deg: float = 22.0,
    min_x_span_ratio: float = 0.30,
    max_y_span_ratio: float = 0.16,
    center_y_min_ratio: float = 0.32,
    center_y_max_ratio: float = 0.72,
) -> LineObservation:
    """Find the transverse center line that marks the shortcut exit."""
    if width <= 0 or height <= 0:
        return LineObservation()
    centers = [
        point for point in _box_centers(boxes, min_confidence)
        if 0.24 * height <= point[1] <= 0.78 * height
    ]
    line = _best_line_family(centers, width, height, horizontal=True)
    valid = (
        line.point_count >= 3
        and line.angle_from_horizontal_deg <= max_angle_deg
        and line.x_span_ratio >= min_x_span_ratio
        and line.y_span_ratio <= max_y_span_ratio
        and center_y_min_ratio
        <= line.center_y_ratio <= center_y_max_ratio
        and line.fit_error_ratio <= 0.045
    )
    return LineObservation(**{
        **line.__dict__,
        'valid': valid,
    })


def observe_outgoing_road(
    boxes: Iterable[DetectionBox],
    width: int,
    height: int,
    min_confidence: float = 0.25,
    max_heading_deg: float = 22.0,
    min_points: int = 3,
    min_y_span_ratio: float = 0.12,
    max_fit_error_ratio: float = 0.040,
    near_x_min_ratio: float = 0.08,
    near_x_max_ratio: float = 0.90,
) -> LineObservation:
    """Find one forward-aligned center-line family after the exit turn."""
    if width <= 0 or height <= 0:
        return LineObservation()
    centers = _box_centers(boxes, min_confidence)
    line = _best_line_family(centers, width, height, horizontal=False)
    valid = (
        line.point_count >= min_points
        and abs(line.heading_deg) <= max_heading_deg
        and line.y_span_ratio >= min_y_span_ratio
        and line.fit_error_ratio <= max_fit_error_ratio
        and near_x_min_ratio <= line.near_x_ratio <= near_x_max_ratio
    )
    return LineObservation(**{
        **line.__dict__,
        'valid': valid,
    })


def observe_left_branch(
    boxes: Iterable[DetectionBox],
    width: int,
    height: int,
    committed: bool = False,
    min_confidence: float = 0.20,
    weak_gap_ratio: float = 0.18,
    weak_fit_error_ratio: float = 0.035,
) -> BranchObservation:
    """
    Separate a true left branch from perspective spacing on one line.

    A large x gap alone is insufficient because boxes on a single perspective
    line naturally spread near the vehicle. A branch must also make one-line
    regression residual large. After commitment, a low-residual single family
    is accepted as the selected road rotating into the camera view.
    """
    if width <= 0 or height <= 0:
        return BranchObservation()
    centers = []
    for box in boxes:
        if (
            box.confidence < min_confidence
            or box.xmax <= box.xmin
            or box.ymax <= box.ymin
        ):
            continue
        centers.append((
            0.5 * (box.xmin + box.xmax),
            0.5 * (box.ymin + box.ymax),
        ))
    if len(centers) < 3:
        return BranchObservation(point_count=len(centers))

    ordered_x = sorted(centers, key=lambda point: point[0])
    gaps = [
        ordered_x[index + 1][0] - ordered_x[index][0]
        for index in range(len(ordered_x) - 1)
    ]
    split_index = max(range(len(gaps)), key=gaps.__getitem__)
    gap_ratio = gaps[split_index] / float(width)
    fit_error_ratio = _line_fit_error(centers) / float(width)
    left = ordered_x[:split_index + 1]
    right = ordered_x[split_index + 1:]
    split = (
        gap_ratio >= weak_gap_ratio
        and fit_error_ratio >= weak_fit_error_ratio
        and len(left) >= 1
        and len(right) >= 2
        and sum(point[0] for point in left) / len(left) < 0.48 * width
    )

    selected = left if split else (centers if committed else [])
    valid = len(selected) >= (1 if split else 3)
    near_x_ratio = 0.5
    heading_deg = 0.0
    if valid:
        near_x_ratio, heading_deg = _path_geometry(selected, width)
    split_x_ratio = 0.0
    if gaps:
        split_x_ratio = 0.5 * (
            ordered_x[split_index][0]
            + ordered_x[split_index + 1][0]
        ) / float(width)
    return BranchObservation(
        valid=valid,
        split=split,
        points=tuple(selected),
        point_count=len(centers),
        left_count=len(left) if split else 0,
        right_count=len(right) if split else 0,
        gap_ratio=gap_ratio,
        fit_error_ratio=fit_error_ratio,
        near_x_ratio=near_x_ratio,
        heading_deg=heading_deg,
        split_x_ratio=split_x_ratio,
    )


def generate_left_turn_path(
    observation: BranchObservation,
    width: int,
    height: int,
    sample_count: int = 60,
    fallback: bool = False,
) -> list[tuple[float, float]]:
    """Connect the vehicle anchor to the selected branch with a cubic curve."""
    if width <= 0 or height <= 0:
        return []
    p0 = (0.50 * width, 0.98 * height)
    p1 = (0.48 * width, 0.73 * height)

    if observation.valid and observation.points and not fallback:
        points = list(observation.points)
        if observation.split:
            junction = max(points, key=lambda point: point[0])
            endpoint = min(points, key=lambda point: point[0])
        else:
            junction = max(points, key=lambda point: point[1])
            endpoint = min(points, key=lambda point: point[1])
        tangent_x = endpoint[0] - junction[0]
        tangent_y = endpoint[1] - junction[1]
        tangent_norm = math.hypot(tangent_x, tangent_y)
        if tangent_norm < 1e-6:
            tangent_x, tangent_y, tangent_norm = -1.0, -0.1, 1.0
        tangent_x /= tangent_norm
        tangent_y /= tangent_norm
        chord = max(30.0, math.dist(p0, endpoint))
        control_length = min(0.32 * chord, 0.30 * width)
        p2 = (
            endpoint[0] - tangent_x * control_length,
            endpoint[1] - tangent_y * control_length,
        )
        p3 = endpoint
    else:
        p2 = (0.32 * width, 0.50 * height)
        p3 = (0.04 * width, 0.48 * height)

    result = []
    for index in range(max(4, int(sample_count))):
        t_value = index / float(max(3, sample_count - 1))
        one_minus = 1.0 - t_value
        weights = (
            one_minus ** 3,
            3.0 * one_minus ** 2 * t_value,
            3.0 * one_minus * t_value ** 2,
            t_value ** 3,
        )
        x_value = sum(
            weight * point[0]
            for weight, point in zip(weights, (p0, p1, p2, p3))
        )
        y_value = sum(
            weight * point[1]
            for weight, point in zip(weights, (p0, p1, p2, p3))
        )
        result.append((
            min(max(x_value, 0.0), width - 1.0),
            min(max(y_value, 0.0), height - 1.0),
        ))
    return result


def generate_exit_connection_path(
    crossline: LineObservation,
    width: int,
    height: int,
    sample_count: int = 60,
) -> list[tuple[float, float]]:
    """Join the vehicle anchor to the observed road with a left tangent."""
    if width <= 0 or height <= 0:
        return []
    p0 = (0.50 * width, 0.98 * height)
    p1 = (0.48 * width, 0.76 * height)
    if crossline.valid and crossline.points:
        ordered = sorted(crossline.points, key=lambda point: point[0])
        left_count = max(1, len(ordered) // 3)
        endpoint = (
            median(point[0] for point in ordered[:left_count]),
            median(point[1] for point in ordered[:left_count]),
        )
        endpoint = (
            min(max(endpoint[0], 0.04 * width), 0.42 * width),
            min(max(endpoint[1], 0.30 * height), 0.68 * height),
        )
    else:
        endpoint = (0.06 * width, 0.50 * height)
    control_length = min(0.28 * math.dist(p0, endpoint), 0.30 * width)
    p2 = (
        min(endpoint[0] + control_length, 0.48 * width),
        endpoint[1],
    )
    p3 = endpoint
    result = []
    for index in range(max(4, int(sample_count))):
        t_value = index / float(max(3, sample_count - 1))
        one_minus = 1.0 - t_value
        weights = (
            one_minus ** 3,
            3.0 * one_minus ** 2 * t_value,
            3.0 * one_minus * t_value ** 2,
            t_value ** 3,
        )
        result.append((
            sum(
                weight * point[0]
                for weight, point in zip(weights, (p0, p1, p2, p3))
            ),
            sum(
                weight * point[1]
                for weight, point in zip(weights, (p0, p1, p2, p3))
            ),
        ))
    return result


def resample_path(
    points: Sequence[Sequence[float]],
    sample_count: int,
) -> list[tuple[float, float]]:
    """Resample a polyline by arc length for pointwise path blending."""
    finite = _finite_points(points)
    count = max(2, int(sample_count))
    if len(finite) < 2:
        return []
    distances = [0.0]
    for first, second in zip(finite, finite[1:]):
        distances.append(distances[-1] + math.dist(first, second))
    if distances[-1] <= 1e-6:
        return [finite[0]] * count
    result = []
    segment = 0
    for index in range(count):
        target = distances[-1] * index / float(count - 1)
        while segment + 1 < len(distances) and distances[segment + 1] < target:
            segment += 1
        if segment + 1 >= len(finite):
            result.append(finite[-1])
            continue
        span = distances[segment + 1] - distances[segment]
        alpha = 0.0 if span <= 1e-9 else (
            target - distances[segment]) / span
        result.append((
            (1.0 - alpha) * finite[segment][0]
            + alpha * finite[segment + 1][0],
            (1.0 - alpha) * finite[segment][1]
            + alpha * finite[segment + 1][1],
        ))
    return result


def blend_paths(
    previous: Sequence[Sequence[float]],
    current: Sequence[Sequence[float]],
    alpha: float = 0.40,
) -> list[tuple[float, float]]:
    """Smooth equal-length route samples without changing their topology."""
    if not previous or len(previous) != len(current):
        return [(float(point[0]), float(point[1])) for point in current]
    alpha = min(max(float(alpha), 0.0), 1.0)
    return [
        (
            (1.0 - alpha) * float(old[0]) + alpha * float(new[0]),
            (1.0 - alpha) * float(old[1]) + alpha * float(new[1]),
        )
        for old, new in zip(previous, current)
    ]


class ShortcutLeftTurnMachine:
    """Latch one shortcut session and isolate its entry and exit turns."""

    def __init__(self, config: ShortcutTurnConfig | None = None):
        """Initialize the state machine with optional thresholds."""
        self.config = config or ShortcutTurnConfig()
        if self.config.left_signal_window < 1:
            raise ValueError('left_signal_window must be at least 1')
        if not 1 <= self.config.left_signal_min_matches <= (
                self.config.left_signal_window):
            raise ValueError(
                'left_signal_min_matches must be between 1 and '
                'left_signal_window')
        self.reset()

    def reset(self) -> None:
        """Return to an unarmed state and clear temporal votes."""
        self.state = ShortcutTurnState.IDLE
        self.left_count = 0
        self.non_left_count = 0
        self.left_signal_votes = deque(
            maxlen=self.config.left_signal_window)
        self.strong_count = 0
        self.weak_count = 0
        self.state_entered_s = 0.0
        self.session_started_s: float | None = None
        self.last_branch_seen_s: float | None = None
        self.last_timestamp_s: float | None = None
        self.fallback_active = False
        self.split_seen = False
        self.session_active = False
        self.handoff_progress = 0.0
        self.entry_alignment_votes = deque(
            maxlen=max(1, self.config.entry_alignment_window))
        self.crossline_votes = deque(
            maxlen=max(1, self.config.crossline_window))
        self.exit_alignment_votes = deque(
            maxlen=max(1, self.config.exit_alignment_window))
        self.handoff_quality_votes = deque(maxlen=3)
        self.last_reason = 'waiting_for_left_signal'

    @staticmethod
    def _matches(votes) -> int:
        return sum(1 for vote in votes if vote)

    def _enter(self, state: ShortcutTurnState, now_s: float) -> None:
        self.state = state
        self.state_entered_s = now_s

    def observe_signal(self, raw_signal: str) -> None:
        """Count one newly received raw traffic-light observation."""
        signal = str(raw_signal).strip().lower()
        if self.state == ShortcutTurnState.IDLE:
            self.left_signal_votes.append(signal == 'left')
            self.left_count = self._matches(self.left_signal_votes)
        elif self.state == ShortcutTurnState.COMPLETE:
            self.non_left_count = (
                self.non_left_count + 1 if signal != 'left' else 0
            )

    def arm_exit(
        self,
        timestamp_s: float,
        bypass_cruise_guard: bool = True,
    ) -> None:
        """Arm an already-entered shortcut for exit-only bag replay."""
        now_s = float(timestamp_s)
        self.session_active = True
        self.session_started_s = now_s
        self._enter(ShortcutTurnState.SHORTCUT_CRUISE, now_s)
        if bypass_cruise_guard:
            self.state_entered_s -= self.config.min_cruise_duration_s
        self.crossline_votes.clear()
        self.exit_alignment_votes.clear()
        self.last_reason = 'manual_exit_session_armed'

    def _decision(self, override_command=None) -> ShortcutTurnDecision:
        path_active = self.state in (
            ShortcutTurnState.FOLLOW_BRANCH,
            ShortcutTurnState.EXIT_LEFT_COMMIT,
            ShortcutTurnState.EXIT_HANDOFF,
            ShortcutTurnState.FAILED,
        )
        if self.state == ShortcutTurnState.PREPARE_LEFT:
            speed_limit = self.config.prepare_speed_limit
        elif self.state in (
            ShortcutTurnState.FOLLOW_BRANCH,
            ShortcutTurnState.EXIT_LEFT_COMMIT,
            ShortcutTurnState.EXIT_HANDOFF,
        ):
            speed_limit = self.config.turn_speed_limit
        elif self.state == ShortcutTurnState.SHORTCUT_CRUISE:
            speed_limit = self.config.cruise_speed_limit
        elif self.state == ShortcutTurnState.FAILED:
            speed_limit = 0.0
        else:
            speed_limit = -1.0
        return ShortcutTurnDecision(
            state=self.state,
            override_command=override_command,
            path_active=path_active,
            speed_limit=speed_limit,
            use_fallback_path=self.fallback_active,
            branch_confirm_count=max(self.strong_count, self.weak_count),
            exit_confirm_count=self._matches(self.exit_alignment_votes),
            crossline_confirm_count=self._matches(self.crossline_votes),
            session_active=self.session_active,
            handoff_progress=self.handoff_progress,
            reason=self.last_reason,
        )

    def update(
        self,
        timestamp_s: float,
        raw_signal: str,
        base_curve: CurveObservation,
        branch: BranchObservation,
        crossline: LineObservation | None = None,
        outgoing: LineObservation | None = None,
        force_trigger: bool = False,
        signal_is_new: bool = True,
    ) -> ShortcutTurnDecision:
        """Advance one source frame and return commands for the ROS wrapper."""
        now_s = float(timestamp_s)
        if (
            self.last_timestamp_s is not None
            and now_s + 1e-6 < self.last_timestamp_s
        ):
            self.reset()
            self.last_reason = 'source_time_reset'
        self.last_timestamp_s = now_s
        signal = str(raw_signal).strip().lower()
        if signal_is_new:
            self.observe_signal(signal)
        crossline = crossline or LineObservation()
        outgoing = outgoing or LineObservation()
        override_command = None

        if self.state == ShortcutTurnState.IDLE:
            live_trigger = (
                len(self.left_signal_votes) >= self.config.left_signal_window
                and self.left_count >= self.config.left_signal_min_matches
            )
            if force_trigger or live_trigger:
                self._enter(ShortcutTurnState.PREPARE_LEFT, now_s)
                self.session_started_s = now_s
                self.session_active = True
                self.strong_count = 0
                self.weak_count = 0
                self.fallback_active = False
                self.split_seen = force_trigger
                self.last_reason = (
                    'replay_or_manual_entry_trigger'
                    if force_trigger else 'left_signal_confirmed'
                )
                override_command = 'go_left'

        elif self.state == ShortcutTurnState.PREPARE_LEFT:
            elapsed_s = max(0.0, now_s - self.state_entered_s)
            strong = (
                branch.valid
                and branch.split
                and branch.left_count >= 2
                and branch.gap_ratio >= self.config.strong_gap_ratio
                and branch.fit_error_ratio
                >= self.config.strong_fit_error_ratio
            )
            weak = (
                branch.valid
                and branch.split
                and branch.gap_ratio >= self.config.weak_gap_ratio
                and branch.fit_error_ratio
                >= self.config.weak_fit_error_ratio
            )
            self.strong_count = self.strong_count + 1 if strong else 0
            self.weak_count = self.weak_count + 1 if weak else 0
            self.split_seen = self.split_seen or branch.split
            confirmed = (
                self.strong_count >= self.config.strong_confirm_frames
                or self.weak_count >= self.config.weak_confirm_frames
            )
            fallback_due = elapsed_s >= self.config.fallback_commit_s
            if elapsed_s >= self.config.min_prepare_duration_s and (
                    confirmed or fallback_due):
                self._enter(ShortcutTurnState.FOLLOW_BRANCH, now_s)
                self.last_branch_seen_s = now_s if branch.valid else None
                self.fallback_active = not branch.valid
                self.entry_alignment_votes.clear()
                self.last_reason = (
                    'left_branch_confirmed'
                    if confirmed else 'event_fallback_committed'
                )
                override_command = 'reset'
            else:
                self.last_reason = (
                    'left_branch_confirming' if weak
                    else 'preparing_left_lane'
                )

        elif self.state == ShortcutTurnState.FOLLOW_BRANCH:
            elapsed_s = max(0.0, now_s - self.state_entered_s)
            if branch.valid:
                self.last_branch_seen_s = now_s
                self.split_seen = self.split_seen or branch.split
                if not branch.split:
                    self.fallback_active = False
            entry_aligned = (
                elapsed_s >= self.config.min_follow_duration_s
                and self.split_seen
                and base_curve.valid
                and abs(base_curve.heading_deg)
                <= self.config.entry_max_heading_deg
            )
            self.entry_alignment_votes.append(entry_aligned)
            if self._matches(self.entry_alignment_votes) >= (
                    self.config.entry_alignment_min_matches):
                self._enter(ShortcutTurnState.SHORTCUT_CRUISE, now_s)
                self.crossline_votes.clear()
                self.fallback_active = False
                self.last_reason = 'shortcut_entry_complete_cruise_latched'
            elif elapsed_s >= self.config.max_follow_duration_s:
                self._enter(ShortcutTurnState.FAILED, now_s)
                self.last_reason = 'entry_turn_timeout_stopped'
            else:
                self.last_reason = (
                    'shortcut_entry_alignment_confirming'
                    if entry_aligned else 'following_left_entry_branch'
                )

        elif self.state == ShortcutTurnState.SHORTCUT_CRUISE:
            elapsed_s = max(0.0, now_s - self.state_entered_s)
            session_elapsed_s = (
                max(0.0, now_s - self.session_started_s)
                if self.session_started_s is not None else 0.0
            )
            eligible = elapsed_s >= self.config.min_cruise_duration_s
            self.crossline_votes.append(eligible and crossline.valid)
            if (
                eligible
                and self._matches(self.crossline_votes)
                >= self.config.crossline_min_matches
            ):
                self._enter(ShortcutTurnState.EXIT_LEFT_COMMIT, now_s)
                self.exit_alignment_votes.clear()
                self.fallback_active = not crossline.valid
                self.last_reason = 'shortcut_exit_crossline_confirmed_2_of_3'
            elif session_elapsed_s >= self.config.session_timeout_s:
                self._enter(ShortcutTurnState.FAILED, now_s)
                self.last_reason = 'shortcut_session_timeout_stopped'
            else:
                self.last_reason = (
                    'shortcut_exit_crossline_confirming'
                    if self._matches(self.crossline_votes) > 0
                    else 'shortcut_cruise_exit_guarded'
                )

        elif self.state == ShortcutTurnState.EXIT_LEFT_COMMIT:
            elapsed_s = max(0.0, now_s - self.state_entered_s)
            aligned = (
                elapsed_s >= self.config.exit_min_turn_duration_s
                and outgoing.valid
                and base_curve.valid
                and abs(base_curve.heading_deg)
                <= self.config.exit_max_heading_deg
            )
            self.exit_alignment_votes.append(aligned)
            if elapsed_s >= self.config.exit_min_turn_duration_s:
                self._enter(ShortcutTurnState.COMPLETE, now_s)
                self.session_active = False
                self.non_left_count = 0
                self.handoff_progress = 1.0
                self.last_reason = 'shortcut_exit_turn_duration_complete'
            else:
                self.last_reason = (
                    'outgoing_road_alignment_confirming'
                    if aligned else 'following_exit_left_connection'
                )

        elif self.state == ShortcutTurnState.EXIT_HANDOFF:
            elapsed_s = max(0.0, now_s - self.state_entered_s)
            quality_ok = (
                outgoing.valid
                and base_curve.valid
                and abs(base_curve.heading_deg)
                <= self.config.exit_max_heading_deg
            )
            self.handoff_quality_votes.append(quality_ok)
            if (
                len(self.handoff_quality_votes) >= 3
                and self._matches(self.handoff_quality_votes) == 0
            ):
                self._enter(ShortcutTurnState.EXIT_LEFT_COMMIT, now_s)
                self.handoff_progress = 0.0
                self.exit_alignment_votes.clear()
                self.last_reason = 'handoff_quality_lost_recommit'
            else:
                duration = max(1e-3, self.config.exit_handoff_duration_s)
                linear = min(max(elapsed_s / duration, 0.0), 1.0)
                self.handoff_progress = linear * linear * (3.0 - 2.0 * linear)
                if linear >= 1.0:
                    self._enter(ShortcutTurnState.COMPLETE, now_s)
                    self.session_active = False
                    self.non_left_count = 0
                    self.handoff_progress = 1.0
                    self.last_reason = 'shortcut_exit_handoff_complete'
                else:
                    self.last_reason = 'blending_exit_path_to_normal_lane'

        elif self.state == ShortcutTurnState.COMPLETE:
            if self.non_left_count >= self.config.rearm_non_left_frames:
                self.reset()
                self.last_timestamp_s = now_s
                self.last_reason = 'rearmed'

        return self._decision(override_command)
