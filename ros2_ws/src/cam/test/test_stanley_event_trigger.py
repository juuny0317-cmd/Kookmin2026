import threading
import time

from cam.Integrated_Stanley_Controller import IntegratedStanleyController
from custom_interfaces.msg import XycarState
from std_msgs.msg import Header


def make_controller(enabled):
    controller = IntegratedStanleyController.__new__(
        IntegratedStanleyController)
    controller.event_driven_control = enabled
    controller.trigger_count = 0

    def count_control():
        controller.trigger_count += 1

    controller.control_timer_callback = count_control
    return controller


def test_event_trigger_is_disabled_by_default_path():
    controller = make_controller(False)
    controller.trigger_event_control_if_enabled()
    assert controller.trigger_count == 0


def test_event_trigger_runs_existing_control_callback_once():
    controller = make_controller(True)
    controller.trigger_event_control_if_enabled()
    assert controller.trigger_count == 1


def test_rejected_state_does_not_trigger_event_control():
    controller = make_controller(True)
    controller.trigger_event_control_if_enabled(state_accepted=False)
    assert controller.trigger_count == 0


def test_control_callbacks_are_serialized():
    controller = IntegratedStanleyController.__new__(
        IntegratedStanleyController)
    controller.control_execution_lock = threading.Lock()
    active = 0
    maximum_active = 0
    calls = 0
    counter_lock = threading.Lock()

    def fake_control():
        nonlocal active, maximum_active, calls
        with counter_lock:
            active += 1
            maximum_active = max(maximum_active, active)
        time.sleep(0.01)
        with counter_lock:
            active -= 1
            calls += 1

    controller.run_control_once = fake_control
    workers = [
        threading.Thread(target=controller.control_timer_callback)
        for _ in range(4)
    ]
    for worker in workers:
        worker.start()
    for worker in workers:
        worker.join()

    assert calls == 4
    assert maximum_active == 1


def make_state_receiver():
    controller = IntegratedStanleyController.__new__(
        IntegratedStanleyController)
    controller.state_lock = threading.Lock()
    controller.latest_xycar_state = None
    controller.latest_state_source_header = None
    controller.latest_state_source_ns = 0
    controller.latest_state_received_ns = 0
    controller.latest_state_received_monotonic = 0.0
    controller.latest_control_metadata = {}
    controller.last_controlled_source_ns = 0
    controller.last_claimed_state_source_ns = 0
    controller.state_receive_count = 0
    controller.state_update_count = 0
    controller.duplicate_source_reject_count = 0
    controller.out_of_order_source_reject_count = 0
    controller.state_replaced_count = 0
    controller.pipeline_timing_pub = None
    return controller


def test_duplicate_and_out_of_order_sources_cannot_overwrite_latest_state():
    controller = make_state_receiver()
    header = Header()
    first = XycarState()
    first.drive_mode = 'first'
    duplicate = XycarState()
    duplicate.drive_mode = 'duplicate'
    older = XycarState()
    older.drive_mode = 'older'

    assert controller.update_latest_state(
        first, header, 200, 1, 1.0, {})
    assert not controller.update_latest_state(
        duplicate, header, 200, 2, 2.0, {})
    assert not controller.update_latest_state(
        older, header, 199, 3, 3.0, {})

    assert controller.latest_xycar_state.drive_mode == 'first'
    assert controller.state_receive_count == 3
    assert controller.state_update_count == 1
    assert controller.duplicate_source_reject_count == 1
    assert controller.out_of_order_source_reject_count == 1


def test_newer_source_replaces_uncontrolled_state_latest_only():
    controller = make_state_receiver()
    header = Header()

    assert controller.update_latest_state(
        XycarState(), header, 300, 1, 1.0, {})
    assert controller.update_latest_state(
        XycarState(), header, 301, 2, 2.0, {})

    assert controller.latest_state_source_ns == 301
    assert controller.state_replaced_count == 1


def test_newer_source_does_not_mark_inflight_source_as_replaced():
    controller = make_state_receiver()
    header = Header()

    assert controller.update_latest_state(
        XycarState(), header, 400, 1, 1.0, {})
    controller.last_claimed_state_source_ns = 400
    assert controller.update_latest_state(
        XycarState(), header, 401, 2, 2.0, {})

    assert controller.latest_state_source_ns == 401
    assert controller.state_replaced_count == 0
