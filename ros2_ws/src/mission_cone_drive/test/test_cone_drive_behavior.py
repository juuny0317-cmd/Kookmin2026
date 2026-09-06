import math

from custom_interfaces.msg import ClusterData
from mission_cone_drive.lidar_dbscan_path_planning_node import (
    LidarDbscanPathPlanningNode,
)
from mission_cone_drive.preprocessing_node import (
    filter_front_clusters_by_angle,
)
from mission_cone_drive.pure_pursuit_node import PurePursuitNode
import numpy as np


def make_path_planner():
    planner = object.__new__(LidarDbscanPathPlanningNode)
    planner.prev_interp_x = None
    planner.prev_interp_y = None
    return planner


def test_preprocessing_keeps_nearest_cluster_per_36_degree_bin():
    centers = [
        (0.50, 0.20),
        (0.90, 0.36),
        (0.60, -0.30),
    ]

    selected = filter_front_clusters_by_angle(centers, 36)

    assert (0.50, 0.20) in selected
    assert (0.90, 0.36) not in selected
    assert (0.60, -0.30) in selected


def test_path_planner_uses_seed_growth_and_cubic_spline():
    planner = make_path_planner()
    clusters = [
        (0.45, 0.35),
        (0.75, 0.35),
        (1.05, 0.35),
        (0.45, -0.35),
        (0.75, -0.35),
        (1.05, -0.35),
    ]

    left, right = planner.form_cone_groups(clusters)
    midpoints = planner.calculate_midpoints_from_final_cones(left, right)
    path = planner.build_path(midpoints)

    assert len(left) == 3
    assert len(right) == 3
    assert len(midpoints) == 3
    assert len(path) == 100
    assert math.isclose(path[0][0], 0.87, abs_tol=1e-6)


def run_cluster_callback_with_groups(left, right):
    planner = make_path_planner()
    planner.is_active = True
    captured = {}

    def form_groups(_centers):
        return list(left), list(right)

    def capture_result(out_left, out_right, midpoints, path):
        captured.update({
            'left': out_left,
            'right': out_right,
            'midpoints': midpoints,
            'path': path,
        })

    planner.form_cone_groups = form_groups
    planner.publish_result = capture_result
    planner.cluster_callback(ClusterData())
    return captured


def test_virtual_boundary_requires_two_real_cones_on_either_side():
    single_left = run_cluster_callback_with_groups([(0.45, 0.35)], [])
    single_right = run_cluster_callback_with_groups([], [(0.45, -0.35)])
    two_left = run_cluster_callback_with_groups(
        [(0.45, 0.35), (0.75, 0.35)], [])
    two_right = run_cluster_callback_with_groups(
        [], [(0.45, -0.35), (0.75, -0.35)])

    assert single_left['right'] == []
    assert single_left['path'] == []
    assert single_right['left'] == []
    assert single_right['path'] == []
    assert len(two_left['right']) == 1
    assert len(two_left['path']) == 100
    assert len(two_right['left']) == 1
    assert len(two_right['path']) == 100


def test_pairing_can_reuse_the_same_opposite_cone():
    planner = make_path_planner()
    left = [(0.45, 0.35), (0.75, 0.35), (1.05, 0.35)]
    right = [(0.45, -0.35), (0.75, -0.35), (1.05, -0.35)]

    midpoints = planner.calculate_midpoints_pair(left, right)

    assert [point[0] for point in midpoints] == [0.45, 0.60, 0.75]


def test_pure_pursuit_has_direct_bounded_steering():
    assert PurePursuitNode.single_target_control((1.0, 0.0)) == 0.0
    assert PurePursuitNode.single_target_control((0.0, 1.0)) == -100.0
    assert PurePursuitNode.single_target_control((0.0, -1.0)) == 100.0


def test_lookahead_and_speed_match_path_and_steering():
    short_x = np.asarray([0.0, 0.2])
    short_y = np.zeros(2)
    long_x = np.asarray([0.0, 20.0])
    long_y = np.zeros(2)

    assert PurePursuitNode.dynamic_lookahead_from_path(
        short_x, short_y) == 0.7
    assert PurePursuitNode.dynamic_lookahead_from_path(
        long_x, long_y) == 2.5
    assert PurePursuitNode.compute_speed(0.0) == 20.0
    assert PurePursuitNode.compute_speed(100.0) == 10.0
    assert PurePursuitNode.compute_speed(0.0, 4.0, 4.0) == 4.0
    assert PurePursuitNode.compute_speed(100.0, 4.0, 4.0) == 4.0
