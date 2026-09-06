"""Lane speed selection using direct curve, recovery, and straight bands."""

from collections import deque
from dataclasses import dataclass
import math
from statistics import median


@dataclass(frozen=True)
class CurveSpeedPolicyConfig:
    """Tunable direct lane speeds and per-mode longitudinal rates."""

    straight_speed: float = 16.0
    strong_curve_speed: float = 10.0
    medium_curve_speed: float = 12.0
    mild_curve_speed: float = 14.0
    recovery_speed: float = 14.0
    strong_threshold: float = 0.60
    medium_threshold: float = 0.35
    straight_enter_cte_px: float = 45.0
    straight_exit_cte_px: float = 60.0
    straight_enter_heading_deg: float = 5.0
    straight_exit_heading_deg: float = 8.0
    straight_enter_risk: float = 0.20
    straight_exit_risk: float = 0.35
    straight_confirm_frames: int = 2
    curve_confirm_frames: int = 2
    curvature_filter_window: int = 3
    curvature_scale: float = 0.50
    curve_accel_rate_per_s: float = 6.0
    straight_accel_rate_per_s: float = 12.0
    curve_decel_rate_per_s: float = 96.0
    straight_decel_rate_per_s: float = 36.0

    def __post_init__(self):
        finite_values = (
            self.straight_speed,
            self.strong_curve_speed,
            self.medium_curve_speed,
            self.mild_curve_speed,
            self.recovery_speed,
            self.strong_threshold,
            self.medium_threshold,
            self.straight_enter_cte_px,
            self.straight_exit_cte_px,
            self.straight_enter_heading_deg,
            self.straight_exit_heading_deg,
            self.straight_enter_risk,
            self.straight_exit_risk,
            self.curvature_scale,
            self.curve_accel_rate_per_s,
            self.straight_accel_rate_per_s,
            self.curve_decel_rate_per_s,
            self.straight_decel_rate_per_s,
        )
        if not all(math.isfinite(float(value)) for value in finite_values):
            raise ValueError('curve speed policy values must be finite')
        if not (
            0.0 <= float(self.strong_curve_speed)
            <= float(self.medium_curve_speed)
            <= float(self.mild_curve_speed)
            <= float(self.straight_speed)
        ):
            raise ValueError(
                'curve speed policy requires 0 <= strong <= medium '
                '<= mild <= straight'
            )
        if not 0.0 <= float(self.recovery_speed) <= float(
                self.straight_speed):
            raise ValueError(
                'recovery speed requires 0 <= recovery <= straight'
            )
        if not (
            0.0 <= float(self.medium_threshold)
            < float(self.strong_threshold)
            <= 1.0
        ):
            raise ValueError(
                'curve speed thresholds require 0 <= medium < strong <= 1'
            )
        if not (
            0.0 <= float(self.straight_enter_risk)
            <= float(self.straight_exit_risk)
            <= 1.0
        ):
            raise ValueError(
                'straight risk gates require 0 <= enter <= exit <= 1'
            )
        if not (
            0.0 <= float(self.straight_enter_cte_px)
            <= float(self.straight_exit_cte_px)
        ):
            raise ValueError('straight CTE gates require 0 <= enter <= exit')
        if not (
            0.0 <= float(self.straight_enter_heading_deg)
            <= float(self.straight_exit_heading_deg)
        ):
            raise ValueError(
                'straight heading gates require 0 <= enter <= exit'
            )
        if (
            int(self.straight_confirm_frames) < 1
            or int(self.curve_confirm_frames) < 1
            or int(self.curvature_filter_window) < 1
        ):
            raise ValueError('curve speed frame counts must be >= 1')
        if float(self.curvature_scale) <= 0.0:
            raise ValueError('curvature_scale must be > 0')
        if (
            float(self.curve_accel_rate_per_s) < 0.0
            or float(self.straight_accel_rate_per_s) < 0.0
            or float(self.curve_decel_rate_per_s) < 0.0
            or float(self.straight_decel_rate_per_s) < 0.0
        ):
            raise ValueError('lane speed acceleration/deceleration rates must be >= 0')


@dataclass(frozen=True)
class CurveSpeedDecision:
    """Selected lane speed and the direct evidence behind that selection."""

    target_speed: float
    mode: str
    hard_speed_cap: float
    accel_rate_per_s: float
    decel_rate_per_s: float
    signed_curvature: float
    preview_signed_curvature: float
    filtered_curvature_risk: float
    straight_alignment: bool
    curve_evidence: bool


class CurveSpeedPolicy:
    """Select a direct speed band without curve lifecycle phases."""

    STRAIGHT = 'straight'
    CURVE_STRONG = 'lane_curve_strong'
    CURVE_MEDIUM = 'lane_curve_medium'
    CURVE_MILD = 'lane_curve_mild'
    RECOVERY = 'lane_straight_recovery'

    def __init__(self, config=None):
        self.config = config or CurveSpeedPolicyConfig()
        window = max(1, int(self.config.curvature_filter_window))
        self.current_curvature_history = deque(maxlen=window)
        self.preview_curvature_history = deque(maxlen=window)
        self.reset()

    def reset(self):
        """Start conservatively until straight alignment is confirmed."""
        self.current_curvature_history.clear()
        self.preview_curvature_history.clear()
        self.mode = self.CURVE_MILD
        self.straight_count = 0
        self.curve_count = 0

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
        scale = float(self.config.curvature_scale)
        return magnitude / (magnitude + scale)

    def _filtered_geometry(
            self, signed_curvature, preview_signed_curvature,
            curvature_severity):
        current = (
            float(signed_curvature)
            if self._finite(signed_curvature) else float('nan')
        )
        preview = (
            float(preview_signed_curvature)
            if self._finite(preview_signed_curvature) else float('nan')
        )
        self.current_curvature_history.append(current)
        self.preview_curvature_history.append(preview)
        filtered_current = self._median_finite(
            self.current_curvature_history)
        filtered_preview = self._median_finite(
            self.preview_curvature_history)
        public_risk = (
            self._clip(curvature_severity, 0.0, 1.0)
            if self._finite(curvature_severity) else 0.0
        )
        risk = max(
            public_risk,
            self._curvature_risk(filtered_current),
            self._curvature_risk(filtered_preview),
        )
        return filtered_current, filtered_preview, self._clip(risk, 0.0, 1.0)

    def _curve_mode(self, risk):
        if risk >= float(self.config.strong_threshold):
            return self.CURVE_STRONG
        if risk >= float(self.config.medium_threshold):
            return self.CURVE_MEDIUM
        return self.CURVE_MILD

    def _speed(self, mode):
        return {
            self.STRAIGHT: self.config.straight_speed,
            self.CURVE_STRONG: self.config.strong_curve_speed,
            self.CURVE_MEDIUM: self.config.medium_curve_speed,
            self.CURVE_MILD: self.config.mild_curve_speed,
            self.RECOVERY: self.config.recovery_speed,
        }[mode]

    def update(
            self, *, curve_state, cte_px, heading_error_rad,
            signed_curvature=float('nan'),
            preview_signed_curvature=float('nan'),
            curvature_severity=0.0):
        """Select straight or a curvature-risk band for one source frame."""
        signed, preview, risk = self._filtered_geometry(
            signed_curvature,
            preview_signed_curvature,
            curvature_severity,
        )
        official_curve = str(curve_state).strip().lower() == 'curve'
        cte = abs(float(cte_px)) if self._finite(cte_px) else float('inf')
        heading_deg = (
            abs(math.degrees(float(heading_error_rad)))
            if self._finite(heading_error_rad) else float('inf')
        )
        strict_alignment = bool(
            not official_curve
            and cte <= float(self.config.straight_enter_cte_px)
            and heading_deg
            <= float(self.config.straight_enter_heading_deg)
            and risk <= float(self.config.straight_enter_risk)
        )
        loose_alignment = bool(
            not official_curve
            and cte <= float(self.config.straight_exit_cte_px)
            and heading_deg
            <= float(self.config.straight_exit_heading_deg)
            and risk <= float(self.config.straight_exit_risk)
        )

        if self.mode == self.STRAIGHT:
            self.curve_count = self.curve_count + 1 if not loose_alignment else 0
            immediate_curve = bool(
                official_curve
                or risk >= float(self.config.straight_exit_risk)
            )
            if (
                immediate_curve
                or self.curve_count >= int(self.config.curve_confirm_frames)
            ):
                self.mode = (
                    self.RECOVERY
                    if not official_curve
                    and risk < float(self.config.medium_threshold)
                    else self._curve_mode(risk)
                )
                self.straight_count = 0
                self.curve_count = 0
        else:
            self.straight_count = (
                self.straight_count + 1 if strict_alignment else 0
            )
            if self.straight_count >= int(
                    self.config.straight_confirm_frames):
                self.mode = self.STRAIGHT
                self.straight_count = 0
                self.curve_count = 0
            else:
                self.mode = (
                    self.RECOVERY
                    if not official_curve
                    and risk < float(self.config.medium_threshold)
                    else self._curve_mode(risk)
                )

        speed = float(self._speed(self.mode))
        return CurveSpeedDecision(
            target_speed=speed,
            mode=self.mode,
            # Direct bands are reached through the configured deceleration
            # slew.  Safety authorities (traffic, shortcut, and obstacle)
            # retain their independent immediate caps in the controller.
            hard_speed_cap=float('inf'),
            accel_rate_per_s=float(
                self.config.straight_accel_rate_per_s
                if self.mode == self.STRAIGHT
                else self.config.curve_accel_rate_per_s
            ),
            decel_rate_per_s=float(
                self.config.straight_decel_rate_per_s
                if self.mode == self.STRAIGHT
                else self.config.curve_decel_rate_per_s
            ),
            signed_curvature=float(signed),
            preview_signed_curvature=float(preview),
            filtered_curvature_risk=float(risk),
            straight_alignment=strict_alignment,
            curve_evidence=bool(
                official_curve
                or risk >= float(self.config.medium_threshold)
                or not loose_alignment
            ),
        )
