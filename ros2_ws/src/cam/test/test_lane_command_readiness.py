"""Safety tests for the integrated-drive readiness classifier."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[3]
    / 'scripts'
    / 'check_lane_command_ready.py'
)
SPEC = spec_from_file_location('check_lane_command_ready', SCRIPT_PATH)
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
classify_readiness_mode = MODULE.classify_readiness_mode
classify_source_frame = MODULE.classify_source_frame


def classify(speed, traffic_state, traffic_is_fresh=True,
             timing_is_fresh=True):
    """Call the classifier with production threshold values."""
    return classify_readiness_mode(
        speed=speed,
        timing_is_fresh=timing_is_fresh,
        traffic_state=traffic_state,
        traffic_is_fresh=traffic_is_fresh,
        minimum_speed=0.5,
        red_hold_max_speed=0.05,
    )


def test_fresh_moving_command_is_ready():
    assert classify(10.0, 'green') == 'moving'


def test_fresh_red_zero_command_is_safe_hold():
    assert classify(0.0, 'red') == 'red_hold'


def test_stale_red_zero_command_is_not_ready():
    assert classify(0.0, 'red', traffic_is_fresh=False) is None


def test_unknown_or_green_zero_command_is_not_ready():
    assert classify(0.0, 'unknown') is None
    assert classify(0.0, 'green') is None


def test_red_with_moving_command_is_rejected():
    assert classify(10.0, 'red') is None


def test_stale_command_is_rejected_in_both_modes():
    assert classify(10.0, 'green', timing_is_fresh=False) is None
    assert classify(0.0, 'red', timing_is_fresh=False) is None


def test_only_newer_source_stamps_are_new_frames():
    assert classify_source_frame(100, 0) == 'new'
    assert classify_source_frame(101, 100) == 'new'
    assert classify_source_frame(100, 100) == 'duplicate'
    assert classify_source_frame(99, 100) == 'out_of_order'
    assert classify_source_frame(0, 100) == 'invalid'
