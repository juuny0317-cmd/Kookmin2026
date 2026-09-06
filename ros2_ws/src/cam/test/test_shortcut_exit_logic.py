"""Tests for shortcut entry, exit detection, and path handoff."""

from cam.shortcut_left_turn_logic import (
    BranchObservation,
    CurveObservation,
    DetectionBox,
    LineObservation,
    ShortcutLeftTurnMachine,
    ShortcutTurnConfig,
    ShortcutTurnState,
    generate_exit_connection_path,
    observe_crossline,
    observe_outgoing_road,
)


def boxes_for(points):
    """Create small detection boxes centered at image points."""
    return [
        DetectionBox(x - 4.0, y - 3.0, x + 4.0, y + 3.0, 0.9)
        for x, y in points
    ]


def enter_shortcut_cruise(machine):
    """Drive the unchanged live-entry sequence into shortcut cruise."""
    base = CurveObservation(valid=True, heading_deg=4.0)
    branch = BranchObservation(
        valid=True,
        split=True,
        point_count=4,
        left_count=2,
        right_count=2,
        gap_ratio=0.30,
        fit_error_ratio=0.06,
    )

    for index, signal in enumerate(
            ('left', 'green', 'left', 'green', 'left')):
        machine.update(
            index * 0.1, signal, base, BranchObservation())
    machine.update(1.2, 'unknown', base, branch)
    machine.update(1.21, 'unknown', base, branch)
    machine.update(4.21, 'unknown', base, BranchObservation())
    machine.update(4.22, 'unknown', base, BranchObservation())
    decision = machine.update(4.23, 'unknown', base, BranchObservation())

    assert decision.state == ShortcutTurnState.SHORTCUT_CRUISE
    return base


def test_speed_limit_is_selected_only_by_shortcut_state():
    """Keep each existing phase cap and add a cruise-only cap."""
    config = ShortcutTurnConfig(
        prepare_speed_limit=14.0,
        turn_speed_limit=13.0,
        cruise_speed_limit=8.0,
    )
    machine = ShortcutLeftTurnMachine(config)
    expected = {
        ShortcutTurnState.IDLE: -1.0,
        ShortcutTurnState.PREPARE_LEFT: 14.0,
        ShortcutTurnState.FOLLOW_BRANCH: 13.0,
        ShortcutTurnState.SHORTCUT_CRUISE: 8.0,
        ShortcutTurnState.EXIT_LEFT_COMMIT: 13.0,
        ShortcutTurnState.EXIT_HANDOFF: 13.0,
        ShortcutTurnState.FAILED: 0.0,
        ShortcutTurnState.COMPLETE: -1.0,
    }

    for state, speed_limit in expected.items():
        machine.state = state
        assert machine._decision().speed_limit == speed_limit


def test_default_cruise_speed_limit_preserves_unlimited_behavior():
    """Keep legacy cruise behavior when the new cap is not configured."""
    machine = ShortcutLeftTurnMachine()
    machine.state = ShortcutTurnState.SHORTCUT_CRUISE

    assert machine.config.cruise_speed_limit == -1.0
    assert machine._decision().speed_limit == -1.0


def test_cruise_cap_starts_only_after_entry_completion():
    """Do not apply the cruise cap during preparation or entry turning."""
    machine = ShortcutLeftTurnMachine(ShortcutTurnConfig(
        prepare_speed_limit=14.0,
        turn_speed_limit=14.0,
        cruise_speed_limit=8.0,
    ))

    prepare = machine.update(
        0.0,
        'unknown',
        CurveObservation(valid=True),
        BranchObservation(),
        force_trigger=True,
    )
    assert prepare.state == ShortcutTurnState.PREPARE_LEFT
    assert prepare.speed_limit == 14.0

    machine.state = ShortcutTurnState.FOLLOW_BRANCH
    assert machine._decision().speed_limit == 14.0

    entry_machine = ShortcutLeftTurnMachine(machine.config)
    enter_shortcut_cruise(entry_machine)
    assert entry_machine._decision().speed_limit == 8.0
    assert not entry_machine._decision().path_active


def test_cruise_cap_is_replaced_immediately_when_exit_starts():
    """Switch from the cruise cap to the existing turn cap on exit."""
    machine = ShortcutLeftTurnMachine(ShortcutTurnConfig(
        turn_speed_limit=14.0,
        cruise_speed_limit=8.0,
    ))
    machine.arm_exit(0.0)
    base = CurveObservation(valid=True)
    crossline = LineObservation(valid=True)

    cruise = machine.update(
        0.0, 'unknown', base, BranchObservation(), crossline=crossline)
    exit_started = machine.update(
        0.1, 'unknown', base, BranchObservation(), crossline=crossline)

    assert cruise.state == ShortcutTurnState.SHORTCUT_CRUISE
    assert cruise.speed_limit == 8.0
    assert not cruise.path_active
    assert exit_started.state == ShortcutTurnState.EXIT_LEFT_COMMIT
    assert exit_started.speed_limit == 14.0
    assert exit_started.path_active


def test_crossline_and_outgoing_geometry_are_separate():
    """Keep horizontal exit markers separate from forward road lines."""
    horizontal = boxes_for([
        (45.0, 126.0),
        (120.0, 124.0),
        (205.0, 126.0),
        (280.0, 123.0),
    ])
    vertical = boxes_for([
        (166.0, 220.0),
        (163.0, 174.0),
        (159.0, 126.0),
        (156.0, 76.0),
    ])

    assert observe_crossline(horizontal, 320, 240).valid
    assert not observe_outgoing_road(horizontal, 320, 240).valid
    assert observe_outgoing_road(vertical, 320, 240).valid
    assert not observe_crossline(vertical, 320, 240).valid


def test_idle_ignores_entry_and_exit_geometry_without_left_signal():
    """Ignore shortcut geometry until a left-signal session is active."""
    machine = ShortcutLeftTurnMachine()
    crossline = LineObservation(valid=True)
    base = CurveObservation(valid=True)
    branch = BranchObservation(
        valid=True,
        split=True,
        left_count=2,
        right_count=2,
        gap_ratio=0.30,
        fit_error_ratio=0.06,
    )

    for index in range(12):
        decision = machine.update(
            index * 0.1,
            'unknown',
            base,
            branch,
            crossline=crossline,
        )

    assert decision.state == ShortcutTurnState.IDLE
    assert not decision.session_active
    assert not decision.path_active


def test_live_entry_requires_three_of_five_signal_frames():
    """Trigger on three left votes in a complete five-frame window."""
    machine = ShortcutLeftTurnMachine()
    base = CurveObservation(valid=True)

    decisions = [
        machine.update(
            index * 0.1, signal, base, BranchObservation())
        for index, signal in enumerate(
            ('left', 'green', 'left', 'green', 'left'))
    ]

    assert all(
        decision.state == ShortcutTurnState.IDLE
        for decision in decisions[:4]
    )
    assert decisions[-1].state == ShortcutTurnState.PREPARE_LEFT
    assert decisions[-1].session_active
    assert decisions[-1].override_command == 'go_left'
    assert not machine.split_seen


def test_two_left_votes_in_five_do_not_trigger():
    """Reject a complete window containing only two left votes."""
    machine = ShortcutLeftTurnMachine()
    base = CurveObservation(valid=True)

    decisions = [
        machine.update(
            index * 0.1, signal, base, BranchObservation())
        for index, signal in enumerate(
            ('left', 'green', 'left', 'green', 'green'))
    ]

    assert all(
        decision.state == ShortcutTurnState.IDLE
        for decision in decisions
    )


def test_cached_left_is_not_recounted_by_lane_callbacks():
    """Count one raw signal message once across repeated lane callbacks."""
    machine = ShortcutLeftTurnMachine()
    base = CurveObservation(valid=True)
    machine.observe_signal('left')

    decisions = [
        machine.update(
            index * 0.1,
            'left',
            base,
            BranchObservation(),
            signal_is_new=False,
        )
        for index in range(8)
    ]

    assert machine.left_count == 1
    assert all(
        decision.state == ShortcutTurnState.IDLE
        for decision in decisions
    )


def test_left_vote_window_is_configurable():
    """Allow the launch layer to select a different N-of-M rule."""
    machine = ShortcutLeftTurnMachine(ShortcutTurnConfig(
        left_signal_window=4,
        left_signal_min_matches=2,
    ))
    base = CurveObservation(valid=True)

    decisions = [
        machine.update(
            index * 0.1, signal, base, BranchObservation())
        for index, signal in enumerate(
            ('left', 'green', 'left', 'green'))
    ]

    assert all(
        decision.state == ShortcutTurnState.IDLE
        for decision in decisions[:3]
    )
    assert decisions[-1].state == ShortcutTurnState.PREPARE_LEFT


def test_prepare_left_confirms_branch_after_lane_shift_starts():
    """Enter branch following only after later branch confirmation."""
    machine = ShortcutLeftTurnMachine()
    base = CurveObservation(valid=True)
    branch = BranchObservation(
        valid=True,
        split=True,
        point_count=4,
        left_count=2,
        right_count=2,
        gap_ratio=0.30,
        fit_error_ratio=0.06,
    )

    for index, signal in enumerate(
            ('left', 'green', 'left', 'green', 'left')):
        machine.update(
            index * 0.1, signal, base, BranchObservation())
    first_branch = machine.update(1.2, 'unknown', base, branch)
    confirmed = machine.update(1.3, 'unknown', base, branch)

    assert first_branch.state == ShortcutTurnState.PREPARE_LEFT
    assert confirmed.state == ShortcutTurnState.FOLLOW_BRANCH
    assert confirmed.override_command == 'reset'


def test_replay_entry_trigger_carries_the_missing_signal_context():
    """Permit explicit replay triggering without a recorded signal."""
    machine = ShortcutLeftTurnMachine()
    decision = machine.update(
        0.0,
        'unknown',
        CurveObservation(valid=True),
        BranchObservation(),
        force_trigger=True,
    )

    assert decision.state == ShortcutTurnState.PREPARE_LEFT
    assert machine.split_seen


def test_entry_can_handoff_to_offset_shortcut_lane():
    """Accept the strongly angled lane visible after shortcut entry."""
    machine = ShortcutLeftTurnMachine()
    machine.update(
        0.0,
        'unknown',
        CurveObservation(valid=True),
        BranchObservation(),
        force_trigger=True,
    )
    machine.state = ShortcutTurnState.FOLLOW_BRANCH
    machine.state_entered_s = 0.0
    base = CurveObservation(valid=True, heading_deg=-55.0)

    machine.update(3.0, 'unknown', base, BranchObservation())
    machine.update(3.1, 'unknown', base, BranchObservation())
    decision = machine.update(3.2, 'unknown', base, BranchObservation())

    assert decision.state == ShortcutTurnState.SHORTCUT_CRUISE


def test_exit_uses_two_of_three_then_configured_source_time_completion():
    """Keep the entry guard, then use the default 1.5 s exit duration."""
    machine = ShortcutLeftTurnMachine()
    base = enter_shortcut_cruise(machine)
    crossline = LineObservation(valid=True)

    first = machine.update(
        8.3, 'unknown', base, BranchObservation(),
        crossline=crossline,
    )
    second = machine.update(8.4, 'unknown', base, BranchObservation())
    assert first.state == ShortcutTurnState.SHORTCUT_CRUISE
    assert second.state == ShortcutTurnState.SHORTCUT_CRUISE

    committed = machine.update(
        8.5, 'unknown', base, BranchObservation(), crossline=crossline)
    assert committed.state == ShortcutTurnState.EXIT_LEFT_COMMIT

    before = machine.update(
        9.99, 'unknown', CurveObservation(), BranchObservation())
    assert before.state == ShortcutTurnState.EXIT_LEFT_COMMIT
    assert before.path_active
    assert before.speed_limit == machine.config.turn_speed_limit
    assert before.exit_confirm_count == 0

    complete = machine.update(
        10.0, 'left', CurveObservation(), BranchObservation())
    assert complete.state == ShortcutTurnState.COMPLETE
    assert not complete.session_active
    assert not complete.path_active
    assert complete.speed_limit == -1.0
    assert complete.exit_confirm_count == 0
    assert complete.handoff_progress == 1.0
    assert complete.reason == 'shortcut_exit_turn_duration_complete'

    no_reentry = machine.update(
        10.1,
        'left',
        CurveObservation(),
        BranchObservation(),
        crossline=crossline,
    )
    assert no_reentry.state == ShortcutTurnState.COMPLETE


def test_exit_completion_uses_configured_turn_duration():
    """Allow the exit-commit duration to be tuned independently."""
    config = ShortcutTurnConfig(
        exit_min_turn_duration_s=0.5,
        exit_alignment_window=1,
        exit_alignment_min_matches=1,
        exit_timeout_s=0.1,
    )
    machine = ShortcutLeftTurnMachine(config)
    machine.arm_exit(0.0)
    base = CurveObservation(valid=True, heading_deg=0.0)
    outgoing = LineObservation(valid=True, heading_deg=0.0)
    crossline = LineObservation(valid=True)

    machine.update(
        0.0, 'unknown', base, BranchObservation(), crossline=crossline)
    committed = machine.update(
        0.1, 'unknown', base, BranchObservation(), crossline=crossline)
    assert committed.state == ShortcutTurnState.EXIT_LEFT_COMMIT

    still_committed = machine.update(
        0.59, 'unknown', base, BranchObservation(), outgoing=outgoing)
    assert still_committed.state == ShortcutTurnState.EXIT_LEFT_COMMIT
    assert still_committed.exit_confirm_count == 0
    assert still_committed.path_active

    complete = machine.update(
        0.61, 'unknown', CurveObservation(), BranchObservation())
    assert complete.state == ShortcutTurnState.COMPLETE
    assert not complete.path_active
    assert complete.speed_limit == -1.0


def test_source_timestamp_rewind_still_resets_the_session():
    """Preserve reset behavior when replay source time moves backwards."""
    machine = ShortcutLeftTurnMachine()
    machine.update(
        10.0,
        'unknown',
        CurveObservation(valid=True),
        BranchObservation(),
        force_trigger=True,
    )

    decision = machine.update(
        5.0, 'unknown', CurveObservation(), BranchObservation())

    assert decision.state == ShortcutTurnState.IDLE
    assert not decision.session_active
    assert decision.reason == 'source_time_reset'


def test_default_exit_duration_is_exactly_one_point_five_seconds():
    assert ShortcutTurnConfig().exit_min_turn_duration_s == 1.5


def test_exit_connection_finishes_on_observed_left_side():
    """Connect the vehicle anchor to the observed left road opening."""
    crossline = LineObservation(
        valid=True,
        points=((45.0, 126.0), (120.0, 124.0), (205.0, 126.0)),
    )
    path = generate_exit_connection_path(crossline, 320, 240, 60)

    assert len(path) == 60
    assert path[0][0] == 160.0
    assert path[-1][0] < 0.25 * 320
    assert path[-1][1] < path[0][1]
