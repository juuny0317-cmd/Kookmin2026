import math

import pytest

from xycar_motor_native.motor_command_adapter import (
    convert_command,
    convert_duty_command,
    fixed_startup_duty,
    slew_limit,
    startup_boost_required,
    update_erpm_pi,
)


def test_calibrated_conversion():
    steering, speed = convert_command(10.0, 20.0)
    assert steering == pytest.approx(-0.068)
    assert speed == pytest.approx(1.6)


def test_commands_are_clamped():
    steering, speed = convert_command(-500.0, 500.0)
    assert steering == pytest.approx(0.2856)
    assert speed == pytest.approx(2.16)


def test_duty_conversion_for_speed_four():
    servo, duty_cycle = convert_duty_command(0.0, 4.0)
    assert servo == pytest.approx(0.5004)
    assert duty_cycle == pytest.approx(0.024)


def test_duty_conversion_preserves_reverse_and_clamps_servo():
    servo, duty_cycle = convert_duty_command(-500.0, -500.0)
    assert servo == pytest.approx(0.15)
    assert duty_cycle == pytest.approx(-0.162)


def test_duty_conversion_reaches_both_safe_steering_endpoints():
    left_servo, _ = convert_duty_command(-42.0, 0.0)
    right_servo, _ = convert_duty_command(42.0, 0.0)
    assert left_servo == pytest.approx(0.15)
    assert right_servo == pytest.approx(0.85)


def test_erpm_pi_adds_duty_when_load_slows_motor():
    duty, integral = update_erpm_pi(
        1476.48, 500.0, 0.024, 0.0, 0.05,
        0.000010, 0.000010, 0.10, 0.25)
    assert duty > 0.033
    assert integral > 0.0


def test_erpm_pi_never_reverses_to_correct_overspeed():
    duty, integral = update_erpm_pi(
        1476.48, 5000.0, 0.024, 0.0, 0.05,
        0.000010, 0.000010, 0.10, 0.25)
    assert duty == 0.0
    assert integral < 0.0


def test_erpm_pi_preserves_reverse_direction():
    duty, _ = update_erpm_pi(
        -1476.48, -500.0, -0.024, 0.0, 0.05,
        0.000010, 0.000010, 0.10, 0.25)
    assert duty < -0.033


def test_slew_limit_bounds_each_control_step():
    assert slew_limit(0.0, 0.1, 0.30, 0.05) == pytest.approx(0.015)
    assert slew_limit(0.1, 0.0, 0.30, 0.05) == pytest.approx(0.085)


def test_startup_duty_is_fixed_across_mission_speeds():
    assert fixed_startup_duty(1476.48, 0.08) == pytest.approx(0.08)
    assert fixed_startup_duty(2952.96, 0.08) == pytest.approx(0.08)
    assert fixed_startup_duty(-2952.96, 0.08) == pytest.approx(-0.08)
    assert fixed_startup_duty(0.0, 0.08) == 0.0


def test_startup_boost_exits_after_forward_breakaway():
    assert startup_boost_required(1476.48, 0.0, True, 700.0, 0.5)
    assert not startup_boost_required(
        1476.48, 750.0, True, 700.0, 0.5)


def test_startup_boost_handles_reverse_and_timer_expiry():
    assert startup_boost_required(-1476.48, -300.0, True, 700.0, 0.5)
    assert not startup_boost_required(
        -1476.48, -750.0, True, 700.0, 0.5)
    assert not startup_boost_required(
        -1476.48, 0.0, False, 700.0, 0.5)


def test_startup_boost_requires_sustained_exit_confirmation():
    assert startup_boost_required(
        1476.48, 1200.0, True, 1200.0, 0.75, 0.10, 0.15)
    assert not startup_boost_required(
        1476.48, 1200.0, True, 1200.0, 0.75, 0.15, 0.15)


def test_ground_twitch_does_not_count_as_full_breakaway():
    assert startup_boost_required(
        1476.48, 1165.0, True, 1600.0, 0.95, 0.30, 0.25)
    assert not startup_boost_required(
        1476.48, 1420.0, True, 1600.0, 0.95, 0.25, 0.25)


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_non_finite_commands_are_rejected(value):
    with pytest.raises(ValueError):
        convert_command(value, 0.0)
