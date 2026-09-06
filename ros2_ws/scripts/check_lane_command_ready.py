#!/usr/bin/env python3
"""Classify whether a lane command is fresh enough for drive handoff."""

from __future__ import annotations

import math


def classify_source_frame(source_stamp_ns: int, previous_stamp_ns: int) -> str:
    """Classify a source timestamp relative to the last accepted frame."""
    current = int(source_stamp_ns)
    previous = int(previous_stamp_ns)
    if current <= 0:
        return 'invalid'
    if previous <= 0 or current > previous:
        return 'new'
    if current == previous:
        return 'duplicate'
    return 'out_of_order'


def classify_readiness_mode(
        *, speed: float, timing_is_fresh: bool, traffic_state: str,
        traffic_is_fresh: bool, minimum_speed: float,
        red_hold_max_speed: float) -> str | None:
    """Return the safe readiness mode, or ``None`` for an invalid command."""
    try:
        command_speed = abs(float(speed))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(command_speed) or not timing_is_fresh:
        return None

    state = str(traffic_state).strip().lower()
    if not traffic_is_fresh:
        return None
    if state == 'red':
        if command_speed <= max(0.0, float(red_hold_max_speed)):
            return 'red_hold'
        return None
    if command_speed >= max(0.0, float(minimum_speed)):
        return 'moving'
    return None


if __name__ == '__main__':
    raise SystemExit(
        'This module supplies readiness classifiers for tests and diagnostics.')

