from mission_cone_drive.lidar_corridor_filter import LidarCorridorFilter


def repeat_update(filter_core, points, count=2):
    result = None
    for _ in range(count):
        result = filter_core.update(points)
    return result


def test_rejects_persistent_clutter_without_corridor():
    filter_core = LidarCorridorFilter()
    clutter = [
        (0.30, 0.70),
        (0.60, 0.68),
        (0.90, 0.69),
        (1.20, 0.67),
    ]

    result = repeat_update(filter_core, clutter, count=4)

    assert result.accepted_cones == []
    assert result.state == 'searching'


def test_rejects_inconsistent_two_pair_furniture_pattern():
    filter_core = LidarCorridorFilter()
    furniture_pattern = [
        (0.112, 0.359),
        (0.270, 0.524),
        (0.518, -0.264),
        (0.864, -0.275),
    ]

    result = repeat_update(filter_core, furniture_pattern, count=4)

    assert result.state == 'searching'
    assert result.accepted_cones == []


def test_accepts_two_sided_corridor_and_rejects_extra_objects():
    filter_core = LidarCorridorFilter()
    corridor = [
        (0.35, 0.40), (0.35, -0.40),
        (0.65, 0.40), (0.65, -0.40),
        (0.95, 0.40), (0.95, -0.40),
    ]
    clutter = [(0.42, 0.02), (1.10, 0.78)]

    result = repeat_update(filter_core, corridor + clutter)

    assert result.state == 'paired'
    assert result.pair_count == 3
    assert len(result.accepted_cones) == 6
    assert (0.42, 0.02) not in result.accepted_cones
    assert (1.10, 0.78) not in result.accepted_cones


def test_uses_centerline_side_not_vehicle_y_sign_on_curve():
    filter_core = LidarCorridorFilter(
        association_distance_m=0.70,
        midpoint_reference_gate_m=0.80,
        track_latest_weight=1.0,
    )
    straight = [
        (0.35, 0.40), (0.35, -0.40),
        (0.65, 0.40), (0.65, -0.40),
        (0.95, 0.40), (0.95, -0.40),
    ]
    repeat_update(filter_core, straight)

    curved = [
        (0.35, 0.55), (0.35, -0.25),
        (0.65, 0.75), (0.65, -0.05),
        (0.95, 0.95), (0.95, 0.15),
    ]
    result = repeat_update(filter_core, curved)

    assert result.state == 'paired'
    assert any(point[1] > 0.0 for point in result.right_cones)
    assert all(
        left[1] > right[1]
        for left, right in zip(result.left_cones, result.right_cones)
    )


def test_keeps_established_corridor_with_one_visible_side():
    filter_core = LidarCorridorFilter()
    corridor = [
        (0.35, 0.40), (0.35, -0.40),
        (0.65, 0.40), (0.65, -0.40),
        (0.95, 0.40), (0.95, -0.40),
    ]
    repeat_update(filter_core, corridor)

    right_only = [
        (0.32, -0.40),
        (0.62, -0.40),
        (0.92, -0.40),
    ]
    result = repeat_update(filter_core, right_only)

    assert result.state == 'partial'
    assert result.left_cones == []
    assert len(result.right_cones) >= 2
    assert len(result.midpoints) >= 2


def test_holds_established_path_when_only_unsupported_pair_is_visible():
    filter_core = LidarCorridorFilter(
        association_distance_m=0.10,
        min_track_hits=1,
        track_latest_weight=1.0,
    )
    corridor = [
        (0.35, 0.40), (0.35, -0.40),
        (0.65, 0.40), (0.65, -0.40),
        (0.95, 0.40), (0.95, -0.40),
    ]
    filter_core.update(corridor)

    furniture_pair = [
        (0.45, 0.05),
        (0.90, -0.55),
    ]
    result = filter_core.update(furniture_pair)

    assert result.pair_count == 1
    assert result.state == 'holding'
    assert result.accepted_cones == []
    assert result.midpoints == []


def test_confirms_two_boundary_points_before_updating_path():
    filter_core = LidarCorridorFilter(
        min_one_side_points=2,
        two_point_support_confirm_frames=2,
        min_track_hits=1,
    )
    corridor = [
        (0.35, 0.40), (0.35, -0.40),
        (0.65, 0.40), (0.65, -0.40),
        (0.95, 0.40), (0.95, -0.40),
    ]
    filter_core.update(corridor)

    boundary_points = [
        (0.35, -0.40),
        (0.65, -0.40),
    ]
    first_result = filter_core.update(boundary_points)
    second_result = filter_core.update(boundary_points)

    assert first_result.state == 'holding'
    assert first_result.accepted_cones == []
    assert first_result.midpoints == []
    assert second_result.state == 'partial'
    assert len(second_result.accepted_cones) == 2
    assert len(second_result.midpoints) >= 2


def test_drops_corridor_after_hold_window():
    filter_core = LidarCorridorFilter(established_hold_frames=2)
    corridor = [
        (0.35, 0.40), (0.35, -0.40),
        (0.65, 0.40), (0.65, -0.40),
    ]
    repeat_update(filter_core, corridor)

    result = repeat_update(filter_core, [], count=4)

    assert result.state in ('lost', 'searching')
    assert result.accepted_cones == []
