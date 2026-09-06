"""Pure calibration math shared by the runtime adapter and verification node."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import Sequence


class PiecewiseLinear:
    """Clamped piecewise-linear lookup table with strict input validation."""

    def __init__(self, x: Sequence[float], y: Sequence[float]) -> None:
        self.x = tuple(float(value) for value in x)
        self.y = tuple(float(value) for value in y)
        if len(self.x) != len(self.y) or len(self.x) < 2:
            raise ValueError('lookup arrays must have the same length and at least two values')
        if any(b <= a for a, b in zip(self.x, self.x[1:])):
            raise ValueError('lookup x values must be strictly increasing')

    def __call__(self, value: float) -> float:
        value = float(value)
        if value <= self.x[0]:
            return self.y[0]
        if value >= self.x[-1]:
            return self.y[-1]
        upper = bisect_right(self.x, value)
        lower = upper - 1
        ratio = (value - self.x[lower]) / (self.x[upper] - self.x[lower])
        return self.y[lower] + ratio * (self.y[upper] - self.y[lower])


@dataclass(frozen=True)
class AckermannAngles:
    left_rad: float
    right_rad: float


class XycarCalibration:
    """Asymmetric steering and measured speed-command calibration."""

    def __init__(
        self,
        logical_steer_rad_lut: Sequence[float],
        raw_steer_lut_by_logical: Sequence[float],
        raw_steer_lut: Sequence[float],
        curvature_lut: Sequence[float],
        speed_command_lut: Sequence[float],
        speed_mps_lut: Sequence[float],
        wheelbase: float,
        front_track: float,
    ) -> None:
        self.logical_to_raw = PiecewiseLinear(logical_steer_rad_lut, raw_steer_lut_by_logical)
        self.raw_to_logical = PiecewiseLinear(raw_steer_lut, curvature_to_center_angles(curvature_lut, wheelbase))
        self.raw_to_curvature = PiecewiseLinear(raw_steer_lut, curvature_lut)
        self.speed_to_mps = PiecewiseLinear(speed_command_lut, speed_mps_lut)
        self.wheelbase = float(wheelbase)
        self.front_track = float(front_track)

    def raw_from_logical(self, logical_steer_rad: float) -> float:
        return self.logical_to_raw(logical_steer_rad)

    def logical_from_raw(self, raw_steer: float) -> float:
        return self.raw_to_logical(raw_steer)

    def curvature_from_raw(self, raw_steer: float) -> float:
        return self.raw_to_curvature(raw_steer)

    def speed_from_command(self, command: float) -> float:
        sign = -1.0 if command < 0.0 else 1.0
        magnitude = abs(float(command))
        if magnitude == 0.0:
            return 0.0
        # No measurement exists below command 4.  A zero anchor is used only
        # so that stop remains exactly zero; this interval remains unmeasured.
        if magnitude < self.speed_to_mps.x[0]:
            return sign * magnitude / self.speed_to_mps.x[0] * self.speed_to_mps.y[0]
        return sign * self.speed_to_mps(magnitude)

    def ackermann_angles(self, curvature: float) -> AckermannAngles:
        if abs(curvature) < 1e-12:
            return AckermannAngles(0.0, 0.0)
        radius = 1.0 / abs(curvature)
        half_track = self.front_track * 0.5
        if radius <= half_track:
            raise ValueError('turn radius must be larger than half the front track')
        inner = math.atan(self.wheelbase / (radius - half_track))
        outer = math.atan(self.wheelbase / (radius + half_track))
        if curvature > 0.0:  # ROS convention: positive yaw / left turn
            return AckermannAngles(inner, outer)
        return AckermannAngles(-outer, -inner)


def curvature_to_center_angles(curvatures: Sequence[float], wheelbase: float) -> list[float]:
    return [math.atan(float(wheelbase) * float(curvature)) for curvature in curvatures]
