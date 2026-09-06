import math

import pytest

from mission_cone_drive.corridor_geometry import CorridorGeometry


def make_geometry():
    return CorridorGeometry(
        min_width_m=0.50,
        max_width_m=1.00,
        expected_width_m=0.80,
        cone_spacing_m=0.30,
    )


def make_boundaries(centerline, half_width=0.40):
    left = []
    right = []
    for index, point in enumerate(centerline):
        before = centerline[max(0, index - 1)]
        after = centerline[min(len(centerline) - 1, index + 1)]
        dx = after[0] - before[0]
        dy = after[1] - before[1]
        length = math.hypot(dx, dy)
        normal = (-dy / length, dx / length)
        left.append((
            point[0] + normal[0] * half_width,
            point[1] + normal[1] * half_width,
        ))
        right.append((
            point[0] - normal[0] * half_width,
            point[1] - normal[1] * half_width,
        ))
    return left, right


def assert_points_close(actual, expected, tolerance=1e-5):
    assert len(actual) == len(expected)
    for actual_point, expected_point in zip(actual, expected):
        assert actual_point[0] == pytest.approx(
            expected_point[0], abs=tolerance)
        assert actual_point[1] == pytest.approx(
            expected_point[1], abs=tolerance)


def test_straight_corridor_is_paired_at_known_width():
    geometry = make_geometry()
    xs = (0.30, 0.60, 0.90, 1.20)
    left = [(x, 0.40) for x in xs]
    right = [(x, -0.40) for x in xs]

    observation = geometry.extract(left + right, [])

    assert observation.pair_count == 4
    assert_points_close(observation.left_cones, left)
    assert_points_close(observation.right_cones, right)
    assert_points_close(
        observation.paired_midpoints,
        [(x, 0.0) for x in xs],
    )


def test_curve_keeps_right_boundary_when_it_crosses_positive_y():
    geometry = make_geometry()
    centerline = [
        (0.30, 0.05),
        (0.52, 0.20),
        (0.72, 0.42),
        (0.88, 0.68),
    ]
    left, right = make_boundaries(centerline)
    assert any(point[1] > 0.0 for point in right)

    observation = geometry.extract(left + right, centerline)

    assert observation.pair_count == 4
    assert_points_close(observation.left_cones, left)
    assert_points_close(observation.right_cones, right)
    assert_points_close(observation.paired_midpoints, centerline)


def test_one_side_fallback_requires_previous_centerline():
    geometry = make_geometry()
    centerline = [
        (0.30, 0.05),
        (0.52, 0.20),
        (0.72, 0.42),
        (0.88, 0.68),
    ]
    _, right = make_boundaries(centerline)

    without_reference = geometry.extract(right, [])
    unsafe_path = geometry.estimate_center_from_one_side(
        without_reference.left_cones,
        without_reference.right_cones,
        [],
        half_width_m=0.40,
        min_points=2,
    )
    assert unsafe_path == []

    with_reference = geometry.extract(right, centerline)
    recovered_path = geometry.estimate_center_from_one_side(
        with_reference.left_cones,
        with_reference.right_cones,
        centerline,
        half_width_m=0.40,
        min_points=2,
    )
    assert len(recovered_path) >= 2
    for point in recovered_path:
        predicted, _, normal, _ = geometry.reference_pose(
            point[0], centerline)
        lateral_error = abs(
            (point[0] - predicted[0]) * normal[0]
            + (point[1] - predicted[1]) * normal[1]
        )
        assert lateral_error < 0.08


def test_same_boundary_spacing_is_not_mistaken_for_corridor_width():
    geometry = make_geometry()
    centerline = [(0.30, 0.0), (0.60, 0.0), (0.90, 0.0)]
    one_boundary = [(0.30, 0.40), (0.60, 0.40), (0.90, 0.40)]

    observation = geometry.extract(one_boundary, centerline)

    assert observation.pair_count == 0
    assert observation.left_cones == one_boundary
    assert observation.right_cones == []


def test_current_corner_geometry_overrides_stale_reference():
    geometry = make_geometry()
    stale_reference = [
        (0.16, 0.00),
        (0.51, 0.01),
        (0.99, -0.49),
        (1.36, -0.33),
    ]
    points = [
        (0.09, 0.33), (0.30, -0.31),
        (0.40, 0.40), (0.69, -0.33),
        (0.75, 0.61), (0.97, -0.15),
        (1.23, 0.07), (1.58, -0.56),
    ]

    observation = geometry.extract(points, stale_reference)

    assert observation.pair_count == 3
    assert (0.09, 0.33) in observation.paired_left_cones
    assert (0.40, 0.40) in observation.paired_left_cones
    assert (0.75, 0.61) in observation.paired_left_cones
    assert (0.30, -0.31) in observation.paired_right_cones
    assert (0.69, -0.33) in observation.paired_right_cones
    assert (0.97, -0.15) in observation.paired_right_cones
    assert (0.97, -0.15) not in observation.paired_left_cones


def test_single_current_pair_is_not_discarded_by_stale_reference():
    geometry = make_geometry()
    stale_reference = [(0.20, -0.70), (1.20, -0.70)]
    points = [(0.42, 0.46), (0.47, -0.36)]

    observation = geometry.extract(points, stale_reference)

    assert observation.pair_count == 1
    assert_points_close(observation.paired_midpoints, [(0.445, 0.05)])


def test_pair_chain_does_not_zigzag_across_corridor():
    geometry = make_geometry()
    stale_reference = [
        (0.29, 0.04),
        (0.68, 0.22),
        (1.08, 0.35),
    ]
    points = [
        (0.33, 0.09),
        (0.64, -0.23),
        (0.83, -0.59),
        (0.89, 0.55),
        (1.09, 0.47),
        (1.12, 0.40),
        (1.22, 0.65),
        (1.46, 0.73),
        (1.74, -0.04),
    ]

    observation = geometry.extract(points, stale_reference)

    assert observation.pair_count <= 2
