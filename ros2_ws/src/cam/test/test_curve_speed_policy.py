import math

import pytest

from cam.curve_speed_policy import CurveSpeedPolicy, CurveSpeedPolicyConfig


def make_policy(**overrides):
    values = {
        'straight_speed': 20.0,
        'strong_curve_speed': 10.0,
        'medium_curve_speed': 12.0,
        'mild_curve_speed': 14.0,
        'recovery_speed': 13.0,
        'strong_threshold': 0.60,
        'medium_threshold': 0.35,
        'straight_enter_cte_px': 45.0,
        'straight_exit_cte_px': 60.0,
        'straight_enter_heading_deg': 5.0,
        'straight_exit_heading_deg': 8.0,
        'straight_enter_risk': 0.20,
        'straight_exit_risk': 0.35,
        'straight_confirm_frames': 2,
        'curve_confirm_frames': 2,
        'curvature_filter_window': 1,
        'curve_accel_rate_per_s': 6.0,
        'straight_accel_rate_per_s': 12.0,
        'curve_decel_rate_per_s': 96.0,
        'straight_decel_rate_per_s': 36.0,
    }
    values.update(overrides)
    return CurveSpeedPolicy(CurveSpeedPolicyConfig(**values))


def update(policy, *, curve='Curve', risk=0.0, cte=0.0, heading=0.0,
           signed=float('nan'), preview=float('nan')):
    return policy.update(
        curve_state=curve,
        cte_px=cte,
        heading_error_rad=math.radians(heading),
        signed_curvature=signed,
        preview_signed_curvature=preview,
        curvature_severity=risk,
    )


@pytest.mark.parametrize(
    'risk, speed, mode',
    (
        (0.80, 10.0, CurveSpeedPolicy.CURVE_STRONG),
        (0.60, 10.0, CurveSpeedPolicy.CURVE_STRONG),
        (0.59, 12.0, CurveSpeedPolicy.CURVE_MEDIUM),
        (0.35, 12.0, CurveSpeedPolicy.CURVE_MEDIUM),
        (0.34, 14.0, CurveSpeedPolicy.CURVE_MILD),
    ),
)
def test_curve_risk_selects_one_exact_speed_band(risk, speed, mode):
    decision = update(make_policy(), risk=risk)

    assert decision.target_speed == speed
    assert math.isinf(decision.hard_speed_cap)
    assert decision.mode == mode
    assert decision.decel_rate_per_s == 96.0


def test_straight_requires_two_strictly_aligned_frames():
    policy = make_policy()

    first = update(policy, curve='Straight', risk=0.10, cte=10, heading=2)
    second = update(policy, curve='Straight', risk=0.10, cte=10, heading=2)

    assert first.mode == CurveSpeedPolicy.RECOVERY
    assert first.target_speed == 13.0
    assert first.decel_rate_per_s == 96.0
    assert second.mode == CurveSpeedPolicy.STRAIGHT
    assert second.target_speed == 20.0
    assert second.decel_rate_per_s == 36.0


def test_curve_evidence_leaves_straight_immediately():
    policy = make_policy()
    update(policy, curve='Straight')
    assert update(policy, curve='Straight').mode == policy.STRAIGHT

    decision = update(policy, curve='Straight', risk=0.60)

    assert decision.mode == CurveSpeedPolicy.CURVE_STRONG
    assert decision.target_speed == 10.0


def test_official_curve_holds_mild_band_when_curvature_temporarily_drops():
    policy = make_policy()
    update(policy, curve='Straight')
    update(policy, curve='Straight')

    decision = update(policy, curve='Curve', risk=0.0)

    assert decision.mode == CurveSpeedPolicy.CURVE_MILD
    assert decision.target_speed == 14.0


def test_bad_alignment_uses_recovery_speed_until_straight_is_confirmed():
    policy = make_policy()

    decisions = [
        update(
            policy,
            curve='Straight',
            risk=0.0,
            cte=100.0,
            heading=20.0,
        )
        for _ in range(10)
    ]

    assert all(item.mode == CurveSpeedPolicy.RECOVERY for item in decisions)
    assert all(item.target_speed == 13.0 for item in decisions)


def test_curve_exit_uses_recovery_speed_not_mild_curve_speed():
    policy = make_policy(recovery_speed=9.0)
    update(policy, curve='Curve', risk=0.80)

    decision = update(
        policy,
        curve='Straight',
        risk=0.0,
        cte=80.0,
        heading=12.0,
    )

    assert decision.mode == CurveSpeedPolicy.RECOVERY
    assert decision.target_speed == 9.0


def test_preview_curvature_selects_curve_band_before_official_state():
    policy = make_policy(curvature_scale=0.50)

    decision = update(
        policy,
        curve='Straight',
        preview=0.75,
    )

    assert math.isclose(decision.filtered_curvature_risk, 0.60)
    assert decision.mode == CurveSpeedPolicy.CURVE_STRONG


def test_invalid_curvature_is_finite_and_conservative_until_aligned():
    decision = update(
        make_policy(),
        curve='Straight',
        signed=float('nan'),
        preview=float('inf'),
    )

    assert decision.mode == CurveSpeedPolicy.RECOVERY
    assert math.isfinite(decision.target_speed)


@pytest.mark.parametrize(
    'overrides, message',
    (
        ({'medium_curve_speed': 9.0}, 'strong <= medium'),
        ({'recovery_speed': 21.0}, 'recovery <= straight'),
        ({'medium_threshold': 0.70}, 'medium < strong'),
        ({'straight_enter_risk': 0.50}, 'enter <= exit'),
        ({'curve_decel_rate_per_s': -1.0}, 'rates must be >= 0'),
    ),
)
def test_configuration_rejects_ambiguous_order(overrides, message):
    with pytest.raises(ValueError, match=message):
        make_policy(**overrides)
