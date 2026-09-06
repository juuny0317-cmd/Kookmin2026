"""Tests for class-aware timed obstacle lane events."""

from cam.dynamic_lane_event_logic import (
    DynamicLaneEventConfig,
    DynamicLaneEventMachine,
    DynamicLaneEventState,
    road_allows_obstacle_event,
    shortcut_blocks_obstacle_event,
    shortcut_protects_override_from_reset,
)


def trigger(machine, event_kind, obstacle_lane, timestamp_s=0.0):
    """Confirm one event using the configured two-frame default."""
    first = machine.update(
        timestamp_s,
        obstacle_lane=obstacle_lane,
        obstacle_kind=event_kind,
        object_present=True,
        trigger_allowed=True,
    )
    active = machine.update(
        timestamp_s + 0.1,
        obstacle_lane=obstacle_lane,
        obstacle_kind=event_kind,
        object_present=True,
        trigger_allowed=True,
    )
    assert first.state == DynamicLaneEventState.IDLE
    assert active.state == DynamicLaneEventState.ACTIVE
    return active


def test_shortcut_blocks_obstacle_events_while_it_owns_the_route():
    """Protect every shortcut lane-command and routed-path phase."""
    blocking_states = (
        'PREPARE_LEFT',
        'FOLLOW_BRANCH',
        'EXIT_LEFT_COMMIT',
        'EXIT_HANDOFF',
        'FAILED',
    )

    assert all(
        shortcut_blocks_obstacle_event(state)
        for state in blocking_states
    )


def test_shortcut_allows_obstacle_events_on_unowned_base_route():
    """Keep obstacle avoidance available during ordinary lane following."""
    unblocked_states = ('IDLE', 'SHORTCUT_CRUISE', 'COMPLETE', 'unknown')

    assert not any(
        shortcut_blocks_obstacle_event(state)
        for state in unblocked_states
    )


def test_shortcut_exit_allows_reset_to_clear_a_cruise_obstacle_command():
    """Clear a prior obstacle override when the shortcut exit takes control."""
    assert shortcut_protects_override_from_reset('PREPARE_LEFT')
    assert shortcut_protects_override_from_reset('FOLLOW_BRANCH')
    assert shortcut_protects_override_from_reset('FAILED')
    assert not shortcut_protects_override_from_reset('EXIT_LEFT_COMMIT')
    assert not shortcut_protects_override_from_reset('EXIT_HANDOFF')


def test_curve_does_not_block_obstacle_avoidance_when_enabled():
    """Permit the same confirmed obstacle event on straight and curved road."""
    assert road_allows_obstacle_event(False, True)
    assert road_allows_obstacle_event(True, True)


def test_curve_avoidance_can_be_disabled_for_field_fallback():
    """Retain an explicit switch for reverting to the conservative policy."""
    assert road_allows_obstacle_event(False, False)
    assert not road_allows_obstacle_event(True, False)


def test_dynamic_keeps_existing_five_second_opposite_lane_event():
    """Keep the existing dynamic timing and left-to-right command."""
    machine = DynamicLaneEventMachine()
    active = trigger(machine, 'obstacle_vehicle', 'left')

    assert active.event_kind == 'dynamic'
    assert active.target_lane == 'right'
    assert active.command == 'go_right'
    assert machine.update(
        5.0, obstacle_kind='static', object_present=True,
    ).state == DynamicLaneEventState.ACTIVE
    complete = machine.update(
        5.2, obstacle_kind='static', object_present=True)
    assert complete.state == DynamicLaneEventState.WAIT_CLEAR
    assert complete.command == 'reset'


def test_static_uses_independent_four_second_event():
    """Use the same lane policy with the independent static timer."""
    machine = DynamicLaneEventMachine()
    active = trigger(machine, 'static', 'right')

    assert active.event_kind == 'static'
    assert active.target_lane == 'left'
    assert active.command == 'go_left'
    assert machine.update(
        4.0, obstacle_kind='dynamic', object_present=True,
    ).state == DynamicLaneEventState.ACTIVE
    complete = machine.update(
        4.2, obstacle_kind='dynamic', object_present=True)
    assert complete.state == DynamicLaneEventState.WAIT_CLEAR
    assert complete.command == 'reset'


def test_active_event_kind_cannot_be_preempted():
    """Do not let a later class replace an active event or its timer."""
    machine = DynamicLaneEventMachine()
    trigger(machine, 'dynamic', 'left')

    decision = machine.update(
        2.0,
        obstacle_lane='right',
        obstacle_kind='static',
        object_present=True,
        trigger_allowed=True,
    )

    assert decision.state == DynamicLaneEventState.ACTIVE
    assert decision.event_kind == 'dynamic'
    assert decision.command == 'go_right'


def test_confirmation_does_not_mix_dynamic_and_static_frames():
    """Require consecutive confirmation for one class and one lane."""
    machine = DynamicLaneEventMachine()
    machine.update(
        0.0, obstacle_lane='left', obstacle_kind='dynamic',
        object_present=True, trigger_allowed=True)
    mixed = machine.update(
        0.1, obstacle_lane='left', obstacle_kind='static',
        object_present=True, trigger_allowed=True)
    confirmed = machine.update(
        0.2, obstacle_lane='left', obstacle_kind='static',
        object_present=True, trigger_allowed=True)

    assert mixed.state == DynamicLaneEventState.IDLE
    assert confirmed.state == DynamicLaneEventState.ACTIVE
    assert confirmed.event_kind == 'static'


def test_static_rearms_after_its_own_clear_count():
    """Allow the same static obstacle to trigger again on a later lap."""
    machine = DynamicLaneEventMachine(DynamicLaneEventConfig(
        static_duration_s=4.0,
        static_speed_confirm_min_matches=1,
        static_avoid_confirm_min_matches=1,
        static_rearm_clear_frames=3,
    ))
    active = machine.update(
        0.0, obstacle_lane='right', obstacle_kind='static',
        object_present=True, trigger_allowed=True)
    assert active.state == DynamicLaneEventState.ACTIVE
    assert machine.update(
        4.1, obstacle_kind='static', object_present=True,
    ).state == DynamicLaneEventState.WAIT_CLEAR

    machine.update(4.2, obstacle_kind='static', object_present=False)
    machine.update(4.3, obstacle_kind='static', object_present=False)
    rearmed = machine.update(
        4.4, obstacle_kind='static', object_present=False)
    assert rearmed.state == DynamicLaneEventState.IDLE

    next_lap = machine.update(
        4.5, obstacle_lane='right', obstacle_kind='static',
        object_present=True, trigger_allowed=True)
    assert next_lap.state == DynamicLaneEventState.ACTIVE
    assert next_lap.event_kind == 'static'


def test_wait_clear_rearms_when_tracker_separates_a_new_target():
    """A second obstacle need not disappear from YOLO before confirmation."""
    machine = DynamicLaneEventMachine(DynamicLaneEventConfig(
        duration_s=1.0,
        speed_confirm_min_matches=2,
        avoid_confirm_min_matches=2,
    ))
    trigger(machine, 'dynamic', 'right')
    waiting = machine.update(
        1.2, obstacle_kind='dynamic', object_present=True)
    assert waiting.state == DynamicLaneEventState.WAIT_CLEAR

    rearmed = machine.update(
        1.3,
        obstacle_kind='dynamic',
        object_present=True,
        new_target_present=True,
    )
    assert rearmed.state == DynamicLaneEventState.IDLE
    assert rearmed.reason == 'rearmed_for_new_target'

    first = machine.update(
        1.4,
        obstacle_lane='left',
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=True,
        obstacle_bbox=(220, 80, 290, 195),
    )
    second = machine.update(
        1.5,
        obstacle_lane='left',
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=True,
        obstacle_bbox=(224, 82, 294, 198),
    )

    assert first.state == DynamicLaneEventState.IDLE
    assert second.state == DynamicLaneEventState.ACTIVE
    assert second.command == 'go_right'


def test_dynamic_slows_in_approach_before_close_lane_change():
    machine = DynamicLaneEventMachine(DynamicLaneEventConfig(
        speed=6.0,
        speed_confirm_min_matches=2,
        speed_confirm_window_frames=3,
        avoid_confirm_min_matches=2,
        avoid_confirm_window_frames=3,
    ))

    first = machine.update(
        0.0,
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=False,
    )
    approach = machine.update(
        0.1,
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=False,
    )

    assert first.state == DynamicLaneEventState.IDLE
    assert first.speed_limit < 0.0
    assert approach.state == DynamicLaneEventState.APPROACH
    assert approach.command == 'reset'
    assert approach.speed_limit == 6.0

    confirming = machine.update(
        0.2,
        obstacle_lane='left',
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=True,
    )
    active = machine.update(
        0.3,
        obstacle_lane='left',
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=True,
    )
    assert confirming.state == DynamicLaneEventState.APPROACH
    assert active.state == DynamicLaneEventState.ACTIVE
    assert active.command == 'go_right'
    assert active.speed_limit == 6.0


def test_static_profile_speed_persists_through_immediate_return():
    machine = DynamicLaneEventMachine(DynamicLaneEventConfig(
        static_speed=5.0,
        static_duration_s=1.0,
        static_speed_confirm_min_matches=1,
        static_avoid_confirm_min_matches=1,
        static_rearm_clear_frames=2,
    ))
    active = machine.update(
        0.0,
        obstacle_lane='right',
        obstacle_kind='static',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=True,
    )
    returning = machine.update(
        1.1,
        obstacle_kind='static',
        object_present=True,
    )

    assert active.speed_limit == 5.0
    assert returning.state == DynamicLaneEventState.WAIT_CLEAR
    assert returning.command == 'reset'
    assert returning.speed_limit == 5.0

    machine.update(1.2, obstacle_kind='static', object_present=False)
    normal = machine.update(
        1.3, obstacle_kind='static', object_present=False)
    assert normal.state == DynamicLaneEventState.IDLE
    assert normal.speed_limit < 0.0


def test_speed_confirmation_allows_one_dropout_in_two_of_three():
    """Confirm slowdown from two eligible frames in a three-frame window."""
    machine = DynamicLaneEventMachine(DynamicLaneEventConfig(speed=6.0))

    first = machine.update(
        0.0,
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=False,
    )
    dropout = machine.update(
        0.1,
        obstacle_kind='dynamic',
        object_present=False,
        speed_trigger_allowed=False,
        trigger_allowed=False,
    )
    confirmed = machine.update(
        0.2,
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=False,
    )

    assert first.state == DynamicLaneEventState.IDLE
    assert dropout.state == DynamicLaneEventState.IDLE
    assert confirmed.state == DynamicLaneEventState.APPROACH
    assert confirmed.speed_limit == 6.0


def test_avoid_confirmation_has_its_own_two_of_three_history():
    """Do not reuse slowdown matches as avoidance matches."""
    machine = DynamicLaneEventMachine()
    machine.update(
        0.0,
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=False,
    )
    approach = machine.update(
        0.1,
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=False,
    )
    assert approach.state == DynamicLaneEventState.APPROACH

    first_avoid = machine.update(
        0.2,
        obstacle_lane='left',
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=True,
        obstacle_bbox=(100, 80, 180, 200),
    )
    machine.update(
        0.3,
        obstacle_kind='dynamic',
        object_present=False,
        speed_trigger_allowed=False,
        trigger_allowed=False,
    )
    active = machine.update(
        0.4,
        obstacle_lane='left',
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=True,
        obstacle_bbox=(104, 82, 186, 204),
    )

    assert first_avoid.state == DynamicLaneEventState.APPROACH
    assert active.state == DynamicLaneEventState.ACTIVE
    assert active.command == 'go_right'


def test_avoid_confirmation_does_not_mix_different_targets():
    """Require matching bounding boxes even when the lane label is equal."""
    machine = DynamicLaneEventMachine()
    machine.update(
        0.0, obstacle_kind='dynamic', object_present=True,
        speed_trigger_allowed=True, trigger_allowed=False)
    machine.update(
        0.1, obstacle_kind='dynamic', object_present=True,
        speed_trigger_allowed=True, trigger_allowed=False)

    machine.update(
        0.2,
        obstacle_lane='left',
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=True,
        obstacle_bbox=(40, 80, 100, 190),
    )
    machine.update(
        0.3,
        obstacle_kind='dynamic',
        object_present=False,
        speed_trigger_allowed=False,
        trigger_allowed=False,
    )
    different_target = machine.update(
        0.4,
        obstacle_lane='left',
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=True,
        obstacle_bbox=(220, 80, 290, 195),
    )
    same_new_target = machine.update(
        0.5,
        obstacle_lane='left',
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=True,
        obstacle_bbox=(224, 82, 294, 198),
    )

    assert different_target.state == DynamicLaneEventState.APPROACH
    assert same_new_target.state == DynamicLaneEventState.ACTIVE


def test_avoid_confirmation_does_not_mix_lanes():
    """Require the same lane label for all counted avoidance matches."""
    machine = DynamicLaneEventMachine()
    machine.update(
        0.0, obstacle_kind='dynamic', object_present=True,
        speed_trigger_allowed=True, trigger_allowed=False)
    machine.update(
        0.1, obstacle_kind='dynamic', object_present=True,
        speed_trigger_allowed=True, trigger_allowed=False)

    bbox = (100, 80, 180, 200)
    machine.update(
        0.2, obstacle_lane='left', obstacle_kind='dynamic',
        object_present=True, speed_trigger_allowed=True,
        trigger_allowed=True, obstacle_bbox=bbox)
    machine.update(
        0.3, obstacle_kind='dynamic', object_present=False,
        speed_trigger_allowed=False, trigger_allowed=False)
    changed_lane = machine.update(
        0.4, obstacle_lane='right', obstacle_kind='dynamic',
        object_present=True, speed_trigger_allowed=True,
        trigger_allowed=True, obstacle_bbox=bbox)
    same_lane = machine.update(
        0.5, obstacle_lane='right', obstacle_kind='dynamic',
        object_present=True, speed_trigger_allowed=True,
        trigger_allowed=True, obstacle_bbox=bbox)

    assert changed_lane.state == DynamicLaneEventState.APPROACH
    assert same_lane.state == DynamicLaneEventState.ACTIVE
    assert same_lane.command == 'go_left'


def test_confirmation_match_and_window_sizes_are_independently_tunable():
    """Allow a configured three-of-four slowdown rule."""
    machine = DynamicLaneEventMachine(DynamicLaneEventConfig(
        speed_confirm_min_matches=3,
        speed_confirm_window_frames=4,
    ))

    for timestamp_s, present in (
        (0.0, True),
        (0.1, False),
        (0.2, True),
    ):
        decision = machine.update(
            timestamp_s,
            obstacle_kind='dynamic',
            object_present=present,
            speed_trigger_allowed=present,
            trigger_allowed=False,
        )
        assert decision.state == DynamicLaneEventState.IDLE

    confirmed = machine.update(
        0.3,
        obstacle_kind='dynamic',
        object_present=True,
        speed_trigger_allowed=True,
        trigger_allowed=False,
    )
    assert confirmed.state == DynamicLaneEventState.APPROACH
