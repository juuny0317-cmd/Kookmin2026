"""Curvature-aware steering slew limiting without ROS dependencies."""

import math
from collections import deque
from dataclasses import dataclass
from statistics import median


@dataclass(frozen=True)
class CurvatureSteeringLimiterConfig:
    """Steering slew limits expressed in servo-command units."""

    enabled: bool = False
    straight_rate_per_s: float = 120.0
    curve_rate_per_s: float = 300.0
    reversal_rate_per_s: float = 360.0
    risk_low: float = 0.15
    risk_high: float = 0.60
    hold_frames: int = 4
    curvature_filter_window: int = 3
    curvature_scale: float = 0.50
    risk_fall_alpha: float = 0.25
    reversal_min_delta: float = 30.0
    reversal_min_risk: float = 0.25
    legacy_max_delta: float = 15.0


@dataclass(frozen=True)
class SteeringLimitResult:
    """One source-frame steering limit decision."""

    angle: float
    effective_risk: float
    rate_per_s: float
    max_delta: float
    reversal_applied: bool
    rate_limited: bool


class CurvatureSteeringLimiter:
    """Limit servo-command slew using current and preview curvature."""

    def __init__(self, config=None):
        """Initialize limiter state from a pure-Python configuration."""
        self.config = config or CurvatureSteeringLimiterConfig()
        window = max(1, int(self.config.curvature_filter_window))
        self.current_curvature_history = deque(maxlen=window)
        self.preview_curvature_history = deque(maxlen=window)
        self.reset()

    def reset(self):
        """Clear curvature and high-authority hold memory."""
        self.current_curvature_history.clear()
        self.preview_curvature_history.clear()
        self.effective_risk = 0.0
        self.hold_remaining_frames = 0
        self.last_result = SteeringLimitResult(
            angle=0.0,
            effective_risk=0.0,
            rate_per_s=0.0,
            max_delta=float(self.config.legacy_max_delta),
            reversal_applied=False,
            rate_limited=False,
        )

    @staticmethod
    def _finite(value):
        try:
            return math.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    @staticmethod
    def _clip(value, lower, upper):
        return max(float(lower), min(float(upper), float(value)))

    @classmethod
    def _median_finite(cls, values):
        finite_values = [
            float(value) for value in values if cls._finite(value)
        ]
        return float(median(finite_values)) if finite_values else 0.0

    def _curvature_risk(self, curvature):
        if not self._finite(curvature):
            return 0.0
        magnitude = abs(float(curvature))
        scale = max(1e-6, float(self.config.curvature_scale))
        return magnitude / (magnitude + scale)

    def _measured_risk(
            self, current_curvature, preview_curvature,
            current_risk_hint):
        current = (
            float(current_curvature)
            if self._finite(current_curvature) else float('nan')
        )
        preview = (
            float(preview_curvature)
            if self._finite(preview_curvature) else float('nan')
        )
        self.current_curvature_history.append(current)
        self.preview_curvature_history.append(preview)

        filtered_current = self._median_finite(
            self.current_curvature_history)
        filtered_preview = self._median_finite(
            self.preview_curvature_history)
        public_current_risk = (
            self._clip(current_risk_hint, 0.0, 1.0)
            if self._finite(current_risk_hint) else 0.0
        )
        filtered_current_risk = max(
            public_current_risk,
            self._curvature_risk(filtered_current),
        )
        filtered_preview_risk = self._curvature_risk(filtered_preview)
        return max(filtered_current_risk, filtered_preview_risk)

    def _update_effective_risk(self, measured_risk):
        measured = self._clip(measured_risk, 0.0, 1.0)
        if measured >= self.effective_risk:
            self.effective_risk = measured
            self.hold_remaining_frames = (
                max(0, int(self.config.hold_frames))
                if measured > 0.0 else 0
            )
        elif self.hold_remaining_frames > 0:
            self.hold_remaining_frames -= 1
        else:
            alpha = self._clip(self.config.risk_fall_alpha, 0.0, 1.0)
            self.effective_risk += alpha * (
                measured - self.effective_risk)
        self.effective_risk = self._clip(
            self.effective_risk, 0.0, 1.0)
        return self.effective_risk

    def _risk_rate(self, risk):
        straight_rate = max(0.0, float(self.config.straight_rate_per_s))
        curve_rate = max(straight_rate, float(
            self.config.curve_rate_per_s))
        low = self._clip(self.config.risk_low, 0.0, 1.0)
        high = max(low + 1e-6, self._clip(
            self.config.risk_high, 0.0, 1.0))
        t = self._clip((float(risk) - low) / (high - low), 0.0, 1.0)
        smooth_t = t * t * (3.0 - 2.0 * t)
        return straight_rate + smooth_t * (curve_rate - straight_rate)

    def limit(
            self, raw_angle, last_angle, source_dt_s,
            current_curvature=float('nan'),
            preview_curvature=float('nan'), current_risk_hint=0.0):
        """Return a bounded servo command for one fresh source frame."""
        raw_is_finite = self._finite(raw_angle)
        last_is_finite = self._finite(last_angle)
        safe_last = float(last_angle) if last_is_finite else 0.0
        if not raw_is_finite or not last_is_finite:
            result = SteeringLimitResult(
                angle=safe_last,
                effective_risk=float(self.effective_risk),
                rate_per_s=0.0,
                max_delta=0.0,
                reversal_applied=False,
                rate_limited=False,
            )
            self.last_result = result
            return result

        raw = float(raw_angle)
        delta = raw - safe_last
        dt = (
            float(source_dt_s)
            if self._finite(source_dt_s) and float(source_dt_s) > 0.0
            else 0.0
        )

        if not bool(self.config.enabled):
            max_delta = max(0.0, float(self.config.legacy_max_delta))
            rate = max_delta / dt if dt > 0.0 else 0.0
            bounded_delta = self._clip(delta, -max_delta, max_delta)
            result = SteeringLimitResult(
                angle=safe_last + bounded_delta,
                effective_risk=0.0,
                rate_per_s=rate,
                max_delta=max_delta,
                reversal_applied=False,
                rate_limited=abs(delta) > max_delta,
            )
            self.last_result = result
            return result

        measured_risk = self._measured_risk(
            current_curvature,
            preview_curvature,
            current_risk_hint,
        )
        effective_risk = self._update_effective_risk(measured_risk)
        reversal = bool(
            raw * safe_last < 0.0
            and abs(delta) >= float(self.config.reversal_min_delta)
            and effective_risk >= float(self.config.reversal_min_risk)
        )
        rate = self._risk_rate(effective_risk)
        if reversal:
            rate = max(
                rate,
                max(0.0, float(self.config.reversal_rate_per_s)),
            )
        max_delta = rate * dt
        bounded_delta = self._clip(delta, -max_delta, max_delta)
        result = SteeringLimitResult(
            angle=safe_last + bounded_delta,
            effective_risk=effective_risk,
            rate_per_s=rate,
            max_delta=max_delta,
            reversal_applied=reversal,
            rate_limited=abs(delta) > max_delta,
        )
        self.last_result = result
        return result
