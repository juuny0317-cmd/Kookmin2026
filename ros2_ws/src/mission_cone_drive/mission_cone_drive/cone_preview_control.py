"""Preview path tracking and curvature speed planning for cone driving."""

from dataclasses import dataclass
import math

from mission_cone_drive.cone_stanley_algorithms import (
    closest_path_reference,
    normalize_angle,
    prepare_path_points,
)
import numpy as np


@dataclass(frozen=True)
class ConePreviewControlResult:
    reference_x: float
    reference_y: float
    target_x: float
    target_y: float
    path_heading_rad: float
    cross_track_error_m: float
    current_curvature_per_m: float
    preview_curvature_per_m: float
    curvature_risk_per_m: float
    feedforward_term_rad: float
    heading_term_rad: float
    cross_track_term_rad: float
    steering_rad: float
    raw_angle_command: float
    final_angle_command: float


def _polyline_distances(points):
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(segment_lengths)))


def _sample_polyline(points, distances, distance_m):
    target = float(np.clip(distance_m, distances[0], distances[-1]))
    index = int(np.searchsorted(distances, target, side='right') - 1)
    index = max(0, min(index, len(points) - 2))
    length = float(distances[index + 1] - distances[index])
    if length <= 1e-9:
        return points[index].copy()
    ratio = (target - float(distances[index])) / length
    return points[index] + ratio * (points[index + 1] - points[index])


def _closest_arc_distance(points, distances, control_x_m):
    control = np.asarray([float(control_x_m), 0.0], dtype=np.float64)
    best_distance_sq = float('inf')
    best_arc_distance = 0.0
    for index, segment in enumerate(np.diff(points, axis=0)):
        length_sq = float(np.dot(segment, segment))
        if length_sq <= 1e-9:
            continue
        ratio = float(np.clip(
            np.dot(control - points[index], segment) / length_sq,
            0.0,
            1.0,
        ))
        projection = points[index] + ratio * segment
        distance_sq = float(np.dot(projection - control, projection - control))
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_arc_distance = (
                float(distances[index])
                + ratio * math.sqrt(length_sq)
            )
    return best_arc_distance


def _three_point_curvature(first, middle, last):
    first_to_middle = middle - first
    middle_to_last = last - middle
    first_to_last = last - first
    first_length = float(np.linalg.norm(first_to_middle))
    second_length = float(np.linalg.norm(middle_to_last))
    chord_length = float(np.linalg.norm(first_to_last))
    denominator = first_length * second_length * chord_length
    if denominator <= 1e-9:
        return 0.0
    cross = (
        float(first_to_middle[0]) * float(first_to_last[1])
        - float(first_to_middle[1]) * float(first_to_last[0])
    )
    return 2.0 * cross / denominator


def _curvature_at(points, distances, center_s, span_m):
    available_before = float(center_s - distances[0])
    available_after = float(distances[-1] - center_s)
    half_span = min(float(span_m), available_before, available_after)
    if half_span < 0.04:
        return 0.0
    first = _sample_polyline(points, distances, center_s - half_span)
    middle = _sample_polyline(points, distances, center_s)
    last = _sample_polyline(points, distances, center_s + half_span)
    return float(_three_point_curvature(first, middle, last))


def path_curvature_metrics(
    path_points,
    control_x_m=0.0,
    steering_preview_m=0.38,
    speed_preview_m=0.85,
    curvature_span_m=0.14,
):
    """Return current, steering-preview, and robust preview curvature."""
    points = prepare_path_points(path_points)
    distances = _polyline_distances(points)
    reference_s = _closest_arc_distance(points, distances, control_x_m)
    path_end = float(distances[-1])
    span = max(float(curvature_span_m), 0.04)

    current_center = min(reference_s + span, path_end - span)
    current_center = max(current_center, span)
    preview_center = min(reference_s + steering_preview_m, path_end - span)
    preview_center = max(preview_center, span)
    current = _curvature_at(points, distances, current_center, span)
    preview = _curvature_at(points, distances, preview_center, span)

    risk_end = min(reference_s + max(speed_preview_m, span), path_end - span)
    risk_start = min(max(reference_s + span, span), risk_end)
    if risk_end <= risk_start + 1e-6:
        samples = [current, preview]
    else:
        sample_centers = np.linspace(risk_start, risk_end, 9)
        samples = [
            _curvature_at(points, distances, center, span)
            for center in sample_centers
        ]
        samples.extend((current, preview))
    finite_abs = np.asarray([
        abs(value) for value in samples if math.isfinite(value)
    ], dtype=np.float64)
    risk = float(np.percentile(finite_abs, 90.0)) if finite_abs.size else 0.0
    target = _sample_polyline(points, distances, min(
        reference_s + max(float(steering_preview_m), 0.0),
        path_end,
    ))
    return current, preview, risk, (float(target[0]), float(target[1]))


def limit_raw_angle_for_downstream_trim(
    raw_angle,
    downstream_trim=-6.0,
    final_angle_min=-42.0,
    final_angle_max=42.0,
):
    """Limit before trim so both calibrated physical endpoints remain usable."""
    trim = float(downstream_trim)
    raw_min = float(final_angle_min) - trim
    raw_max = float(final_angle_max) - trim
    limited = float(np.clip(float(raw_angle), raw_min, raw_max))
    final = float(np.clip(
        limited + trim,
        float(final_angle_min),
        float(final_angle_max),
    ))
    return limited, final


def smooth_steering_command(
    previous_angle,
    target_angle,
    max_angle_step=12.0,
    smoothing_alpha=0.65,
):
    """Low-pass and rate-limit a raw steering command."""
    previous = float(previous_angle)
    target = float(target_angle)
    alpha = float(np.clip(smoothing_alpha, 0.0, 1.0))
    maximum_step = max(float(max_angle_step), 0.0)
    blended = previous + alpha * (target - previous)
    if maximum_step <= 0.0:
        return blended
    return float(
        previous
        + np.clip(blended - previous, -maximum_step, maximum_step)
    )


def compute_cone_preview_control(
    path_points,
    speed_mps,
    control_x_m=0.0,
    heading_window_m=0.28,
    steering_preview_m=0.38,
    speed_preview_m=0.85,
    curvature_span_m=0.14,
    wheelbase_m=0.33,
    curvature_feedforward_gain=1.0,
    heading_gain=0.80,
    cross_track_gain=0.90,
    softening_speed_mps=0.35,
    steering_rad_per_unit=0.0068,
    downstream_trim=-6.0,
    final_angle_min=-42.0,
    final_angle_max=42.0,
):
    """Compute curvature feed-forward plus heading/CTE feedback steering."""
    points = prepare_path_points(path_points)
    reference_x, reference_y, path_heading, cross_track_error = (
        closest_path_reference(
            points,
            control_x_m=float(control_x_m),
            min_forward_x_m=0.0,
            heading_window_m=float(heading_window_m),
        )
    )
    current_curvature, preview_curvature, curvature_risk, target = (
        path_curvature_metrics(
            points,
            control_x_m=float(control_x_m),
            steering_preview_m=float(steering_preview_m),
            speed_preview_m=float(speed_preview_m),
            curvature_span_m=float(curvature_span_m),
        )
    )

    feedforward_term = (
        float(curvature_feedforward_gain)
        * math.atan(float(wheelbase_m) * preview_curvature)
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
    steering = normalize_angle(
        feedforward_term + heading_term + cross_track_term)

    radians_per_unit = max(abs(float(steering_rad_per_unit)), 1e-6)
    raw_angle = -steering / radians_per_unit
    raw_angle, final_angle = limit_raw_angle_for_downstream_trim(
        raw_angle,
        downstream_trim=downstream_trim,
        final_angle_min=final_angle_min,
        final_angle_max=final_angle_max,
    )
    steering = -raw_angle * radians_per_unit
    return ConePreviewControlResult(
        reference_x=reference_x,
        reference_y=reference_y,
        target_x=target[0],
        target_y=target[1],
        path_heading_rad=path_heading,
        cross_track_error_m=cross_track_error,
        current_curvature_per_m=current_curvature,
        preview_curvature_per_m=preview_curvature,
        curvature_risk_per_m=curvature_risk,
        feedforward_term_rad=feedforward_term,
        heading_term_rad=heading_term,
        cross_track_term_rad=cross_track_term,
        steering_rad=steering,
        raw_angle_command=raw_angle,
        final_angle_command=final_angle,
    )


def curvature_speed_command(
    curvature_risk_per_m,
    final_angle_command,
    min_speed=4.0,
    max_speed=8.0,
    lateral_accel_limit_mps2=0.30,
    speed_mps_per_unit=0.08,
    straight_angle_command=-6.0,
    final_angle_min=-42.0,
    final_angle_max=42.0,
):
    """Return 0-or-[min,max] speed from preview curvature and steering load."""
    minimum = max(float(min_speed), 0.0)
    maximum = max(float(max_speed), minimum)
    if maximum <= minimum:
        return minimum

    curvature = abs(float(curvature_risk_per_m))
    if curvature <= 1e-4:
        curvature_speed = maximum
    else:
        speed_mps = math.sqrt(
            max(float(lateral_accel_limit_mps2), 1e-4) / curvature)
        curvature_speed = speed_mps / max(float(speed_mps_per_unit), 1e-4)

    final_angle = float(final_angle_command)
    straight_angle = float(straight_angle_command)
    if final_angle < straight_angle:
        steering_span = max(straight_angle - float(final_angle_min), 1e-3)
    else:
        steering_span = max(float(final_angle_max) - straight_angle, 1e-3)
    steering_ratio = min(
        abs(final_angle - straight_angle) / steering_span,
        1.0,
    )
    steering_speed = maximum - (maximum - minimum) * steering_ratio
    return float(np.clip(
        min(curvature_speed, steering_speed), minimum, maximum))
