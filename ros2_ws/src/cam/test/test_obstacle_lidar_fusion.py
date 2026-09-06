"""Tests for LiDAR-first obstacle clustering and direction matching."""

import numpy as np

from cam.obstacle_lidar_fusion import (
    ProjectedCluster,
    TemporalClusterTracker,
    TemporalClusterTrackingConfig,
    associate_by_horizontal_direction,
    camera_detection_is_near,
    cluster_ordered_scan,
    select_time_aligned_state,
)


def make_projection(index, center_x, distance_m=2.0, y=450.0):
    """Build a compact projected cluster for association tests."""
    points = np.array([
        [center_x - 4.0, y],
        [center_x + 4.0, y],
    ])
    return ProjectedCluster(
        cluster_index=index,
        image_points=points,
        center_x=float(center_x),
        left_x=float(center_x - 4.0),
        right_x=float(center_x + 4.0),
        center_y=float(y),
        top_y=float(y),
        bottom_y=float(y),
        distance_m=float(distance_m),
        point_count=2,
        width_m=0.1,
    )


def test_ordered_scan_splits_objects_before_camera_matching():
    """Range discontinuities form independent object candidates."""
    ranges = np.full(30, np.inf)
    ranges[4:8] = [2.00, 2.01, 2.02, 2.01]
    ranges[15:18] = [3.00, 3.02, 3.01]

    clusters = cluster_ordered_scan(
        ranges,
        angle_min=-0.3,
        angle_increment=0.02,
        min_points=2,
    )

    assert len(clusters) == 2
    assert clusters[0].scan_indices.tolist() == [4, 5, 6, 7]
    assert clusters[1].scan_indices.tolist() == [15, 16, 17]


def test_cluster_distance_uses_configured_percentile():
    """A far return does not force the whole object distance backward."""
    ranges = [np.inf, 1.0, 1.1, 1.8, np.inf]

    clusters = cluster_ordered_scan(
        ranges,
        angle_min=0.0,
        angle_increment=0.01,
        min_points=2,
        base_gap_m=1.0,
        distance_percentile=50.0,
    )

    assert len(clusters) == 1
    assert np.isclose(clusters[0].distance_m, 1.1)


def test_direction_match_does_not_require_points_inside_bbox_height():
    """Horizontal agreement is enough even when LiDAR lands below the box."""
    boxes = [(260.0, 100.0, 360.0, 300.0)]
    projections = [make_projection(0, center_x=315.0, y=350.0)]

    associations = associate_by_horizontal_direction(
        boxes,
        projections,
        padding_px=20.0,
    )

    assert len(associations) == 1
    assert associations[0].detection_index == 0
    assert associations[0].cluster_index == 0


def test_direction_association_is_one_to_one():
    """Two YOLO objects cannot claim the same LiDAR object."""
    boxes = [
        (80.0, 100.0, 180.0, 300.0),
        (430.0, 100.0, 550.0, 300.0),
    ]
    projections = [
        make_projection(0, center_x=125.0),
        make_projection(1, center_x=500.0),
    ]

    associations = associate_by_horizontal_direction(
        boxes,
        projections,
        padding_px=20.0,
    )

    pairs = {
        (item.detection_index, item.cluster_index)
        for item in associations
    }
    assert pairs == {(0, 0), (1, 1)}


def test_initial_direction_match_does_not_prefer_unaligned_near_cluster():
    """Visual alignment wins over an unrelated but much nearer object."""
    boxes = [(260.0, 100.0, 360.0, 300.0)]
    projections = [
        make_projection(0, center_x=270.0, distance_m=0.7),
        make_projection(1, center_x=315.0, distance_m=4.0),
    ]

    associations = associate_by_horizontal_direction(boxes, projections)

    assert len(associations) == 1
    assert associations[0].cluster_index == 1


def test_temporal_tracker_keeps_continuous_range_when_near_distractor_appears():
    """A new close cluster cannot steal an established camera target."""
    tracker = TemporalClusterTracker()
    boxes = [(260.0, 100.0, 360.0, 300.0)]

    first = tracker.associate(
        boxes,
        [make_projection(0, center_x=315.0, distance_m=4.0)],
        source_ns=1_000_000_000,
    )
    second = tracker.associate(
        boxes,
        [
            make_projection(0, center_x=316.0, distance_m=0.8),
            make_projection(1, center_x=318.0, distance_m=3.8),
        ],
        source_ns=1_100_000_000,
    )

    assert first[0].cluster_index == 0
    assert second[0].cluster_index == 1
    assert np.isclose(tracker.track.distance_m, 3.8)


def test_temporal_tracker_does_not_bypass_tracking_for_duplicate_boxes():
    """An established target stays selected while YOLO briefly returns two."""
    tracker = TemporalClusterTracker()
    first_box = (260.0, 100.0, 360.0, 300.0)
    tracker.associate(
        [first_box],
        [make_projection(0, center_x=315.0, distance_m=4.0)],
        source_ns=1_000_000_000,
    )

    associations = tracker.associate(
        [
            (264.0, 100.0, 364.0, 300.0),
            (430.0, 100.0, 530.0, 300.0),
        ],
        [
            make_projection(0, center_x=319.0, distance_m=3.8),
            make_projection(1, center_x=480.0, distance_m=0.7),
        ],
        source_ns=1_100_000_000,
    )

    assert len(associations) == 1
    assert associations[0].detection_index == 0
    assert associations[0].cluster_index == 0
    assert np.isclose(tracker.track.distance_m, 3.8)


def test_temporal_tracker_rejects_an_implausible_distance_jump():
    """Do not publish a different object as a sudden range measurement."""
    tracker = TemporalClusterTracker()
    boxes = [(260.0, 100.0, 360.0, 300.0)]
    tracker.associate(
        boxes,
        [make_projection(0, center_x=315.0, distance_m=4.0)],
        source_ns=1_000_000_000,
    )

    rejected = tracker.associate(
        boxes,
        [make_projection(0, center_x=316.0, distance_m=0.8)],
        source_ns=1_100_000_000,
    )

    assert rejected == []
    assert tracker.last_rejection_reason == 'distance_jump'
    assert np.isclose(tracker.track.distance_m, 4.0)


def test_temporal_tracker_applies_absolute_jump_cap_after_long_gap():
    """Elapsed-time allowance cannot admit a single multi-meter switch."""
    tracker = TemporalClusterTracker(TemporalClusterTrackingConfig(
        max_age_s=2.0,
        max_distance_jump_m=0.45,
        max_range_rate_mps=3.0,
        max_total_distance_jump_m=1.5,
    ))
    boxes = [(260.0, 100.0, 360.0, 300.0)]
    tracker.associate(
        boxes,
        [make_projection(0, center_x=315.0, distance_m=3.0)],
        source_ns=1_000_000_000,
    )

    rejected = tracker.associate(
        boxes,
        [make_projection(0, center_x=316.0, distance_m=1.4)],
        source_ns=1_500_000_000,
    )

    assert rejected == []
    assert tracker.last_rejection_reason == 'distance_jump'
    assert np.isclose(tracker.track.distance_m, 3.0)


def test_tracker_allows_large_closing_step_when_bbox_growth_agrees():
    """Rapid visual growth distinguishes a real closing target from a swap."""
    tracker = TemporalClusterTracker(TemporalClusterTrackingConfig(
        max_total_distance_jump_m=1.5,
    ))
    tracker.associate(
        [(260.0, 180.0, 360.0, 260.0)],
        [make_projection(0, center_x=315.0, distance_m=3.7)],
        source_ns=1_000_000_000,
    )

    accepted = tracker.associate(
        [(350.0, 150.0, 530.0, 285.0)],
        [make_projection(0, center_x=445.0, distance_m=1.8)],
        source_ns=1_400_000_000,
    )

    assert len(accepted) == 1
    assert np.isclose(tracker.track.distance_m, 1.8)


def test_temporal_tracker_allows_physical_range_rate():
    """A closing target inside the configured radial rate remains tracked."""
    tracker = TemporalClusterTracker(TemporalClusterTrackingConfig(
        max_distance_jump_m=0.2,
        max_range_rate_mps=3.0,
    ))
    boxes = [(260.0, 100.0, 360.0, 300.0)]
    tracker.associate(
        boxes,
        [make_projection(0, center_x=315.0, distance_m=4.0)],
        source_ns=1_000_000_000,
    )

    accepted = tracker.associate(
        boxes,
        [make_projection(0, center_x=318.0, distance_m=3.3)],
        source_ns=1_200_000_000,
    )

    assert len(accepted) == 1
    assert np.isclose(tracker.track.distance_m, 3.3)


def test_temporal_tracker_reacquires_after_repeated_misses():
    """A lost target may be reacquired only after the old track expires."""
    tracker = TemporalClusterTracker(TemporalClusterTrackingConfig(
        max_misses=2,
    ))
    boxes = [(260.0, 100.0, 360.0, 300.0)]
    tracker.associate(
        boxes,
        [make_projection(0, center_x=315.0, distance_m=4.0)],
        source_ns=1_000_000_000,
    )

    assert tracker.associate(boxes, [], 1_100_000_000) == []
    assert tracker.associate(boxes, [], 1_200_000_000) == []
    reacquired = tracker.associate(
        boxes,
        [make_projection(0, center_x=316.0, distance_m=0.8)],
        source_ns=1_300_000_000,
    )

    assert len(reacquired) == 1
    assert np.isclose(tracker.track.distance_m, 0.8)


def test_expired_tracker_inserts_a_miss_before_reacquisition():
    """A new identity cannot replace an expired range in the same frame."""
    tracker = TemporalClusterTracker(TemporalClusterTrackingConfig(
        max_age_s=0.5,
    ))
    boxes = [(260.0, 100.0, 360.0, 300.0)]
    tracker.associate(
        boxes,
        [make_projection(0, center_x=315.0, distance_m=4.0)],
        source_ns=1_000_000_000,
    )

    expired = tracker.associate(
        boxes,
        [make_projection(0, center_x=315.0, distance_m=0.8)],
        source_ns=1_600_000_000,
    )
    reacquired = tracker.associate(
        boxes,
        [make_projection(0, center_x=315.0, distance_m=0.8)],
        source_ns=1_700_000_000,
    )

    assert expired == []
    assert len(reacquired) == 1
    assert np.isclose(tracker.track.distance_m, 0.8)


def test_camera_target_change_inserts_a_miss_before_reacquisition():
    """A disjoint YOLO box cannot replace the active range immediately."""
    tracker = TemporalClusterTracker()
    tracker.associate(
        [(100.0, 100.0, 200.0, 300.0)],
        [make_projection(0, center_x=150.0, distance_m=4.0)],
        source_ns=1_000_000_000,
    )
    changed_box = [(400.0, 100.0, 500.0, 300.0)]

    changed = tracker.associate(
        changed_box,
        [make_projection(0, center_x=450.0, distance_m=0.8)],
        source_ns=1_100_000_000,
    )
    reacquired = tracker.associate(
        changed_box,
        [make_projection(0, center_x=450.0, distance_m=0.8)],
        source_ns=1_200_000_000,
    )

    assert changed == []
    assert len(reacquired) == 1
    assert np.isclose(tracker.track.distance_m, 0.8)


def test_tracker_rejects_near_cluster_for_visually_distant_box():
    """A small high bbox cannot initialize an implausibly near LiDAR track."""
    tracker = TemporalClusterTracker(TemporalClusterTrackingConfig(
        near_cluster_distance_m=1.20,
    ))
    distant_box = [(320.0, 199.0, 373.0, 237.0)]

    association = tracker.associate(
        distant_box,
        [make_projection(0, center_x=346.0, distance_m=0.77)],
        source_ns=1_000_000_000,
        image_height=480.0,
    )

    assert association == []
    assert tracker.track is None


def test_tracker_accepts_near_cluster_for_large_low_box():
    """A genuinely close visual target may still acquire a near cluster."""
    tracker = TemporalClusterTracker(TemporalClusterTrackingConfig(
        near_cluster_distance_m=1.20,
    ))
    close_box = [(400.0, 180.0, 560.0, 340.0)]

    association = tracker.associate(
        close_box,
        [make_projection(0, center_x=480.0, distance_m=0.77)],
        source_ns=1_000_000_000,
        image_height=480.0,
    )

    assert len(association) == 1
    assert np.isclose(tracker.track.distance_m, 0.77)


def test_physical_road_state_is_independent_from_routed_curve():
    """A shortcut Curve label cannot overwrite a fresh Straight sample."""
    history = [(1_000_000_000, 1_010_000_000, 'Straight')]

    state = select_time_aligned_state(
        history,
        target_ns=1_020_000_000,
        max_delta_s=0.1,
        fallback='Curve',
    )

    assert state == 'Straight'


def test_stale_physical_road_state_uses_conservative_route_fallback():
    """An old base state is not applied to a newer camera frame."""
    history = [(1_000_000_000, 1_010_000_000, 'Straight')]

    state = select_time_aligned_state(
        history,
        target_ns=2_000_000_000,
        max_delta_s=0.1,
        fallback='Curve',
    )

    assert state == 'Curve'


def test_camera_only_near_detection_can_request_early_speed_cap():
    """A low YOLO box supplies approach evidence before LiDAR association."""
    box = (250.0, 205.0, 310.0, 245.0)

    assert camera_detection_is_near(box, 480, 0.48)
    assert not camera_detection_is_near(box, 480, 0.55)
