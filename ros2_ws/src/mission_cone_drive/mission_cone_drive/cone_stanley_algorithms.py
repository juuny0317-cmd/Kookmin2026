from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class StanleyControlResult:
    """Geometric terms and steering output for one Stanley update."""

    reference_x: float
    reference_y: float
    path_heading_rad: float
    cross_track_error_m: float
    heading_term_rad: float
    cross_track_term_rad: float
    steering_rad: float
    servo_command: float


def normalize_angle(angle_rad):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def prepare_path_points(path_points):
    """Return finite path points with consecutive duplicates removed."""
    points = np.asarray(path_points, dtype=np.float64).reshape(-1, 2)
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < 2:
        raise ValueError('Stanley control requires at least two path points')

    deltas = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.concatenate(([True], deltas > 1e-4))
    points = points[keep]
    if len(points) < 2:
        raise ValueError('Stanley path has no usable segment')
    return points


def closest_path_reference(
    path_points,
    control_x_m,
    min_forward_x_m=0.0,
    heading_window_m=0.30,
):
    """Find the closest forward path reference and a stable path heading."""
    points = prepare_path_points(path_points)
    control = np.asarray([float(control_x_m), 0.0], dtype=np.float64)
    segments = points[1:] - points[:-1]
    lengths_sq = np.sum(segments * segments, axis=1)

    valid = lengths_sq > 1e-8
    forward = np.maximum(points[:-1, 0], points[1:, 0]) >= float(
        min_forward_x_m
    )
    candidate_indices = np.flatnonzero(valid & forward)
    if not len(candidate_indices):
        candidate_indices = np.flatnonzero(valid)
    if not len(candidate_indices):
        raise ValueError('Stanley path has no valid segment')

    starts = points[candidate_indices]
    candidate_segments = segments[candidate_indices]
    candidate_lengths_sq = lengths_sq[candidate_indices]
    ratios = np.sum(
        (control - starts) * candidate_segments,
        axis=1,
    ) / candidate_lengths_sq
    ratios = np.clip(ratios, 0.0, 1.0)
    projections = starts + ratios[:, None] * candidate_segments
    distances_sq = np.sum((projections - control) ** 2, axis=1)
    local_index = int(np.argmin(distances_sq))
    segment_index = int(candidate_indices[local_index])
    reference = projections[local_index]

    target = _heading_target(
        points,
        segment_index,
        reference,
        max(float(heading_window_m), 1e-3),
    )
    tangent = target - reference
    if np.linalg.norm(tangent) <= 1e-6:
        tangent = segments[segment_index]
    heading = math.atan2(float(tangent[1]), float(tangent[0]))

    path_left_normal = np.asarray(
        [-math.sin(heading), math.cos(heading)],
        dtype=np.float64,
    )
    cross_track_error = float(
        np.dot(reference - control, path_left_normal)
    )
    return (
        float(reference[0]),
        float(reference[1]),
        float(heading),
        cross_track_error,
    )


def _heading_target(points, segment_index, reference, window_m):
    remaining = float(window_m)
    current = np.asarray(reference, dtype=np.float64)

    for point_index in range(segment_index + 1, len(points)):
        endpoint = points[point_index]
        segment = endpoint - current
        length = float(np.linalg.norm(segment))
        if length <= 1e-8:
            current = endpoint
            continue
        if length >= remaining:
            return current + segment * (remaining / length)
        remaining -= length
        current = endpoint
    return points[-1]


def compute_stanley_control(
    path_points,
    speed_mps,
    control_x_m=0.33,
    min_forward_x_m=0.0,
    heading_window_m=0.30,
    cross_track_gain=1.2,
    heading_gain=1.0,
    softening_speed_mps=0.50,
    max_steering_angle_deg=20.0,
    servo_scale_deg_per_unit=0.20,
):
    """Compute a Xycar steering command from a vehicle-frame path."""
    reference_x, reference_y, path_heading, cross_track_error = (
        closest_path_reference(
            path_points,
            control_x_m=control_x_m,
            min_forward_x_m=min_forward_x_m,
            heading_window_m=heading_window_m,
        )
    )

    heading_term = float(heading_gain) * normalize_angle(path_heading)
    denominator = max(
        abs(float(speed_mps)) + float(softening_speed_mps),
        1e-3,
    )
    cross_track_term = math.atan2(
        float(cross_track_gain) * cross_track_error,
        denominator,
    )
    steering = normalize_angle(heading_term + cross_track_term)
    max_steering = math.radians(abs(float(max_steering_angle_deg)))
    steering = float(np.clip(steering, -max_steering, max_steering))

    servo_scale = max(abs(float(servo_scale_deg_per_unit)), 1e-6)
    servo_command = -math.degrees(steering) / servo_scale
    servo_command = float(np.clip(servo_command, -100.0, 100.0))
    return StanleyControlResult(
        reference_x=reference_x,
        reference_y=reference_y,
        path_heading_rad=path_heading,
        cross_track_error_m=cross_track_error,
        heading_term_rad=heading_term,
        cross_track_term_rad=cross_track_term,
        steering_rad=steering,
        servo_command=servo_command,
    )
