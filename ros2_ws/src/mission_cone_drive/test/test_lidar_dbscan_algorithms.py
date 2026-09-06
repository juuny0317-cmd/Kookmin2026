import math

from mission_cone_drive.lidar_dbscan_algorithms import (
    dbscan_labels,
    extract_lidar_dbscan_clusters,
    filter_front_clusters_by_angle,
    filter_front_clusters_by_separation,
    LidarDbscanClusterConfig,
    LidarDbscanPathBuilder,
    LidarDbscanPathConfig,
)
import numpy as np
import pytest


def test_dbscan_keeps_dense_groups_and_rejects_noise():
    points = np.asarray([
        [0.50, 0.30],
        [0.51, 0.30],
        [0.49, 0.29],
        [0.50, 0.31],
        [0.52, 0.30],
        [1.00, -0.30],
        [1.01, -0.30],
        [0.99, -0.29],
        [1.00, -0.31],
        [1.02, -0.30],
        [1.40, 0.00],
    ], dtype=np.float32)

    labels = dbscan_labels(points, eps_m=0.05, min_samples=5)

    assert set(labels.tolist()) == {-1, 0, 1}
    assert labels[-1] == -1


def test_angular_filter_keeps_nearest_cluster_per_lidar_bin():
    centers = [
        (0.50, 0.20),
        (0.90, 0.36),
        (0.60, -0.30),
    ]

    selected = filter_front_clusters_by_angle(centers, degree_bin=36.0)

    assert (0.50, 0.20) in selected
    assert (0.90, 0.36) not in selected
    assert (0.60, -0.30) in selected


def test_spatial_filter_keeps_aligned_but_distinct_cones():
    centers = [
        (0.50, 0.20),
        (0.55, 0.22),
        (0.95, 0.38),
    ]

    selected = filter_front_clusters_by_separation(
        centers, separation_m=0.15)

    assert (0.50, 0.20) in selected
    assert (0.55, 0.22) not in selected
    assert (0.95, 0.38) in selected


def test_scan_extraction_matches_front_lidar_dbscan_pipeline():
    count = 720
    angle_min = -math.pi
    increment = 2.0 * math.pi / count
    ranges = np.full(count, np.inf, dtype=np.float32)
    center_index = int((math.radians(40.0) - angle_min) / increment)
    ranges[center_index - 3:center_index + 4] = 0.70

    raw, filtered = extract_lidar_dbscan_clusters(
        ranges,
        angle_min,
        increment,
        LidarDbscanClusterConfig(),
    )

    assert len(raw) == 1
    assert len(filtered) == 1
    assert 0.60 < math.hypot(*filtered[0]) < 0.75


def test_lidar_dbscan_path_reproduces_seed_grow_midpoint_spline():
    builder = LidarDbscanPathBuilder(LidarDbscanPathConfig())
    clusters = [
        (0.45, 0.35),
        (0.75, 0.35),
        (1.05, 0.35),
        (0.45, -0.35),
        (0.75, -0.35),
        (1.05, -0.35),
    ]

    left, right, midpoints, path = builder.build(clusters)

    assert len(left) == 3
    assert len(right) == 3
    assert len(midpoints) == 3
    assert len(path) == 100
    assert all(abs(y) < 1e-6 for _, y in path)
    assert abs(path[0][0] - (0.45 + 0.42)) < 1e-6


def test_lidar_dbscan_path_does_not_invent_initial_one_side_corridor():
    builder = LidarDbscanPathBuilder(LidarDbscanPathConfig())
    clusters = [
        (0.45, 0.35),
        (0.75, 0.35),
    ]

    left, right, midpoints, path = builder.build(clusters)

    assert left == []
    assert right == []
    assert midpoints == []
    assert path == []


def test_lidar_dbscan_pairing_is_one_to_one():
    builder = LidarDbscanPathBuilder(LidarDbscanPathConfig())
    left = [(0.45, 0.45), (0.75, 0.45), (1.05, 0.45)]
    right = [(0.48, -0.45), (1.08, -0.45)]

    matches = builder._match_boundary_pairs(left, right)

    assert len(matches) == 2
    assert len({pair[0] for pair in matches}) == len(matches)
    assert len({pair[1] for pair in matches}) == len(matches)


def test_lidar_dbscan_temporal_corridor_preserves_s_bend():
    config = LidarDbscanPathConfig(
        min_pair_distance_m=0.50,
        max_pair_distance_m=0.95,
        expected_pair_distance_m=0.70,
        virtual_half_width_m=0.35,
        max_path_lateral_jump_m=0.55,
    )
    builder = LidarDbscanPathBuilder(config)
    xs = (0.35, 0.62, 0.89, 1.16, 1.43)
    straight = (
        [(x, 0.35) for x in xs]
        + [(x, -0.35) for x in xs]
    )
    builder.build(straight)

    centerline = [
        (0.35, 0.00),
        (0.62, 0.15),
        (0.89, 0.05),
        (1.16, -0.16),
        (1.43, -0.02),
    ]
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
            point[0] + normal[0] * 0.35,
            point[1] + normal[1] * 0.35,
        ))
        right.append((
            point[0] - normal[0] * 0.35,
            point[1] - normal[1] * 0.35,
        ))

    _, _, midpoints, path = builder.build(left + right)

    assert len(midpoints) == len(centerline)
    assert len(path) == 100
    path_y = [point[1] for point in path]
    assert max(path_y) > 0.10
    assert min(path_y) < -0.10


def test_path_rejects_a_downstream_jump_over_the_full_overlap():
    config = LidarDbscanPathConfig(max_path_lateral_jump_m=0.30)
    builder = LidarDbscanPathBuilder(config)
    builder.previous_interp_x = np.linspace(0.0, 1.0, 100)
    builder.previous_interp_y = np.zeros(100)

    path = builder.interpolate_path([
        (0.0, 0.0),
        (0.5, 0.0),
        (1.0, 0.80),
    ])

    assert path is None


def test_path_memory_is_cleared_after_hold_expires():
    config = LidarDbscanPathConfig(path_lost_keep_frames=1)
    builder = LidarDbscanPathBuilder(config)
    corridor = [
        (0.40, 0.45), (0.80, 0.45),
        (0.40, -0.45), (0.80, -0.45),
    ]
    assert builder.build(corridor)[3]

    assert builder.build([])[3]
    assert builder.build([])[3] == []
    assert builder.previous_interp_x is None


def test_direct_corridor_path_is_extended_to_controller_horizon():
    config = LidarDbscanPathConfig(
        min_path_horizon_m=0.90,
        prediction_min_forward_m=0.90,
    )
    builder = LidarDbscanPathBuilder(config)
    corridor = [
        (0.35, 0.45), (0.65, 0.45),
        (0.35, -0.45), (0.65, -0.45),
    ]

    _, _, _, path = builder.build(corridor)

    path_points = np.asarray(path)
    arc = np.sum(np.linalg.norm(np.diff(path_points, axis=0), axis=1))
    assert arc >= 0.89
    assert builder.last_path_source == 'paired'
    assert builder.last_path_confidence > 0.80


def test_one_side_recovery_expires_after_last_paired_corridor():
    config = LidarDbscanPathConfig(
        one_side_max_age_since_paired_s=0.25,
        path_lost_keep_s=0.25,
        path_lost_keep_frames=4,
    )
    builder = LidarDbscanPathBuilder(config)
    corridor = [
        (0.35, 0.45), (0.70, 0.45),
        (0.35, -0.45), (0.70, -0.45),
    ]
    one_side = [(0.32, 0.45), (0.67, 0.45)]
    assert builder.build(corridor, elapsed_s=0.10)[3]

    assert builder.build(one_side, elapsed_s=0.10)[3]
    assert builder.last_path_source in ('one_side', 'predicted')
    assert builder.build(one_side, elapsed_s=0.20)[3] == []
    assert builder.last_path_source == 'none'


def test_motion_compensation_moves_predicted_path_toward_vehicle():
    builder = LidarDbscanPathBuilder(LidarDbscanPathConfig())
    corridor = [
        (0.35, 0.45), (0.70, 0.45),
        (0.35, -0.45), (0.70, -0.45),
    ]
    first_path = builder.build(corridor, elapsed_s=0.10)[3]

    predicted_path = builder.build(
        [], longitudinal_motion_m=0.10, elapsed_s=0.10)[3]

    assert predicted_path
    assert predicted_path[0][0] == pytest.approx(
        first_path[0][0] - 0.10, abs=1e-5)


def test_single_cross_corridor_pair_updates_existing_path_anchor():
    builder = LidarDbscanPathBuilder(LidarDbscanPathConfig())
    corridor = [
        (0.35, 0.45), (0.70, 0.45),
        (0.35, -0.45), (0.70, -0.45),
    ]
    assert builder.build(corridor, elapsed_s=0.10)[3]

    _, _, midpoints, path = builder.build(
        [(0.52, 0.48), (0.54, -0.42)], elapsed_s=0.30)

    assert len(midpoints) == 1
    assert path
    assert builder.last_path_source == 'pair_anchor'
    assert builder.age_since_paired_s == pytest.approx(0.0)
