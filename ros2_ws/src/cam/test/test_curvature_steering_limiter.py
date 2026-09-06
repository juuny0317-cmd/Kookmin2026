"""Tests for the pure-Python curvature-aware steering limiter."""

import math

from cam.curvature_steering_limiter import (
    CurvatureSteeringLimiter,
    CurvatureSteeringLimiterConfig,
)
from cam.curve_speed_policy import CurveSpeedPolicy, CurveSpeedPolicyConfig


def make_limiter(**overrides):
    """Build an enabled limiter with competition defaults."""
    values = {
        'enabled': True,
        'straight_rate_per_s': 120.0,
        'curve_rate_per_s': 300.0,
        'reversal_rate_per_s': 360.0,
        'risk_low': 0.15,
        'risk_high': 0.60,
        'hold_frames': 4,
        'curvature_filter_window': 1,
    }
    values.update(overrides)
    return CurvatureSteeringLimiter(
        CurvatureSteeringLimiterConfig(**values))


def limit_with_risk(limiter, risk, dt=1.0 / 15.0, raw_angle=100.0,
                    last_angle=0.0):
    """Apply one source frame using a direct current-risk hint."""
    return limiter.limit(
        raw_angle=raw_angle,
        last_angle=last_angle,
        source_dt_s=dt,
        current_risk_hint=risk,
    )


def test_disabled_matches_legacy_fifteen_per_frame_limit():
    """Preserve the legacy per-frame delta exactly while disabled."""
    limiter = make_limiter(enabled=False)

    positive = limit_with_risk(limiter, 1.0, raw_angle=80.0)
    negative = limit_with_risk(
        limiter, 1.0, raw_angle=-80.0, last_angle=10.0)

    assert positive.angle == 15.0
    assert positive.max_delta == 15.0
    assert positive.rate_limited
    assert not positive.reversal_applied
    assert negative.angle == -5.0


def test_zero_risk_allows_about_eight_at_fifteen_hz():
    """Use the straight rate when curvature risk is zero."""
    result = limit_with_risk(make_limiter(), 0.0)

    assert math.isclose(result.rate_per_s, 120.0)
    assert math.isclose(result.max_delta, 8.0)
    assert math.isclose(result.angle, 8.0)


def test_high_risk_allows_about_twenty_at_fifteen_hz():
    """Use the curve rate at the high-risk threshold."""
    result = limit_with_risk(make_limiter(), 0.60)

    assert math.isclose(result.rate_per_s, 300.0)
    assert math.isclose(result.max_delta, 20.0)


def test_midrange_rate_is_continuous_and_monotonic():
    """Interpolate steering rate smoothly across intermediate risks."""
    risks = [0.15, 0.20, 0.30, 0.40, 0.50, 0.60]
    rates = [
        limit_with_risk(make_limiter(), risk).rate_per_s
        for risk in risks
    ]

    assert rates == sorted(rates)
    assert len(set(rates)) == len(rates)
    assert rates[0] == 120.0
    assert rates[-1] == 300.0


def test_risk_drop_holds_curve_authority_for_four_frames():
    """Retain curve authority briefly after measured risk falls."""
    limiter = make_limiter(hold_frames=4)
    high = limit_with_risk(limiter, 0.60)
    held = [limit_with_risk(limiter, 0.0) for _ in range(4)]
    falling = limit_with_risk(limiter, 0.0)

    assert high.rate_per_s == 300.0
    assert all(result.rate_per_s == 300.0 for result in held)
    assert 120.0 < falling.rate_per_s < 300.0


def test_large_high_risk_sign_reversal_uses_reversal_rate():
    """Allow the reversal rate for a large high-risk sign change."""
    result = limit_with_risk(
        make_limiter(),
        0.60,
        raw_angle=-50.0,
        last_angle=50.0,
    )

    assert result.reversal_applied
    assert result.rate_per_s == 360.0
    assert math.isclose(result.max_delta, 24.0)
    assert math.isclose(result.angle, 26.0)


def test_low_risk_sign_reversal_keeps_straight_rate():
    """Reject reversal authority for straight-line sign noise."""
    result = limit_with_risk(
        make_limiter(),
        0.0,
        raw_angle=-50.0,
        last_angle=50.0,
    )

    assert not result.reversal_applied
    assert result.rate_per_s == 120.0
    assert math.isclose(result.max_delta, 8.0)


def test_source_rate_does_not_change_per_second_limit():
    """Keep the same per-second limit at different source rates."""
    results = [
        limit_with_risk(make_limiter(), 0.0, dt=1.0 / frequency)
        for frequency in (10.0, 15.0, 20.0)
    ]

    assert all(result.rate_per_s == 120.0 for result in results)
    assert [result.max_delta for result in results] == [12.0, 8.0, 6.0]


def test_invalid_values_hold_a_finite_safe_command():
    """Hold a finite command when angle or timing inputs are invalid."""
    limiter = make_limiter()

    bad_angle = limiter.limit(
        raw_angle=float('nan'),
        last_angle=20.0,
        source_dt_s=1.0 / 15.0,
        current_curvature=float('inf'),
        preview_curvature=float('nan'),
        current_risk_hint=float('nan'),
    )
    bad_dt = limiter.limit(
        raw_angle=80.0,
        last_angle=20.0,
        source_dt_s=float('inf'),
    )

    assert bad_angle.angle == 20.0
    assert math.isfinite(bad_angle.angle)
    assert not bad_angle.reversal_applied
    assert bad_dt.angle == 20.0
    assert bad_dt.max_delta == 0.0


def test_preview_curvature_keeps_risk_high_at_current_inflection():
    """Use preview curvature when current curvature crosses zero."""
    limiter = make_limiter(curvature_scale=0.50)

    result = limiter.limit(
        raw_angle=-50.0,
        last_angle=50.0,
        source_dt_s=1.0 / 15.0,
        current_curvature=0.0,
        preview_curvature=-0.75,
        current_risk_hint=0.0,
    )

    assert math.isclose(result.effective_risk, 0.60)
    assert result.reversal_applied


def test_reset_clears_curvature_and_hold_memory():
    """Clear all retained authority when the limiter resets."""
    limiter = make_limiter()
    limit_with_risk(limiter, 0.60)

    limiter.reset()
    result = limit_with_risk(limiter, 0.0)

    assert result.effective_risk == 0.0
    assert result.rate_per_s == 120.0
    assert limiter.hold_remaining_frames == 0


def test_limiter_does_not_change_curve_speed_policy_results():
    """Keep limiter state independent from longitudinal planning."""
    config = CurveSpeedPolicyConfig(curvature_filter_window=1)
    before = CurveSpeedPolicy(config).update(
        curve_state='Curve', cte_px=0.0, heading_error_rad=0.0,
        curvature_severity=0.60)

    limit_with_risk(make_limiter(), 0.60)

    after = CurveSpeedPolicy(config).update(
        curve_state='Curve', cte_px=0.0, heading_error_rad=0.0,
        curvature_severity=0.60)
    assert after.target_speed == before.target_speed
    assert after.mode == before.mode
    assert after.filtered_curvature_risk == before.filtered_curvature_risk
