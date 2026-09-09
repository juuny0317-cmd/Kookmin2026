#!/usr/bin/env python3
"""Recalculate vehicle calibration metrics and render a dependency-free SVG."""

from __future__ import annotations

import csv
import json
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
DATA = REPO / "evaluation" / "calibration"
MEDIA = REPO / "media" / "calibration"


def read_csv(name: str) -> list[dict[str, str]]:
    with (DATA / name).open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def pct(numerator: float, denominator: float) -> float:
    return 100.0 * numerator / denominator


speed_rows = read_csv("speed_5m_measurements.csv")
steering_rows = read_csv("steering_circle_measurements.csv")
comparison_rows = read_csv("sim_real_comparison.csv")

speed_points = []
for row in speed_rows:
    command = float(row["command"])
    distance = float(row["distance_m"])
    elapsed = float(row["time_s"])
    speed_points.append(
        {
            "command": int(command),
            "distance_m": distance,
            "time_s": elapsed,
            "speed_mps": distance / elapsed,
        }
    )

by_magnitude: dict[int, dict[str, float]] = {}
for row in steering_rows:
    magnitude = abs(int(float(row["physical_raw"])))
    by_magnitude.setdefault(magnitude, {})[row["direction"]] = float(
        row["radius_m"]
    )

asymmetry = []
for magnitude, directions in sorted(by_magnitude.items()):
    right = directions["right"]
    left = directions["left"]
    asymmetry.append(
        {
            "command_magnitude": magnitude,
            "right_radius_m": right,
            "left_radius_m": left,
            "left_to_right_ratio": left / right,
            "left_radius_larger_pct": pct(left - right, right),
        }
    )

sim_real = []
for row in comparison_rows:
    real_value = float(row["real_value"])
    sim_value = float(row["sim_value"])
    sim_real.append(
        {
            "test_case": row["test_case"],
            "direction": row["direction"],
            "real_value": real_value,
            "sim_value": sim_value,
            "unit": row["unit"],
            "absolute_error": abs(sim_value - real_value),
            "absolute_error_pct": pct(abs(sim_value - real_value), real_value),
        }
    )

summary = {
    "provenance": {
        "A": "value read directly from repository code or configuration",
        "B": "value read from repository evaluation data",
        "C": "user-supplied real-vehicle measurement",
        "D": "derived mathematically from A-C inputs",
        "E": "not currently quantifiable",
    },
    "speed_5m": speed_points,
    "speed_range": {
        "command_4_to_25_time_reduction_pct": pct(
            speed_points[0]["time_s"] - speed_points[-1]["time_s"],
            speed_points[0]["time_s"],
        ),
        "command_4_to_25_speed_ratio": (
            speed_points[-1]["speed_mps"] / speed_points[0]["speed_mps"]
        ),
    },
    "steering_asymmetry": asymmetry,
    "sim_real_comparison": sim_real,
}

DATA.mkdir(parents=True, exist_ok=True)
MEDIA.mkdir(parents=True, exist_ok=True)
(DATA / "calibration_summary.json").write_text(
    json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
)


width, height = 960, 560
left, right, top, bottom = 90, 40, 55, 80
plot_w = width - left - right
plot_h = height - top - bottom
x_min, x_max = 4.0, 25.0
y_min, y_max = 0.0, 2.4


def sx(value: float) -> float:
    return left + (value - x_min) / (x_max - x_min) * plot_w


def sy(value: float) -> float:
    return top + plot_h - (value - y_min) / (y_max - y_min) * plot_h


parts = [
    f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
    '<rect width="100%" height="100%" fill="#0b1020"/>',
    '<style>text{font-family:Arial,sans-serif;fill:#e8edf7}.tick{font-size:16px;fill:#aeb9cc}.title{font-size:27px;font-weight:700}.sub{font-size:15px;fill:#9aa8be}.grid{stroke:#26324a;stroke-width:1}.axis{stroke:#9aa8be;stroke-width:2}.line{fill:none;stroke:#34d6c7;stroke-width:4}.dot{fill:#ffb347;stroke:#0b1020;stroke-width:2}</style>',
    '<text class="title" x="90" y="33">5 m measured command-speed curve</text>',
    '<text class="sub" x="90" y="53">v = 5 m / measured elapsed time · real vehicle measurements</text>',
]

for y in [0.0, 0.5, 1.0, 1.5, 2.0]:
    py = sy(y)
    parts.append(f'<line class="grid" x1="{left}" y1="{py:.1f}" x2="{width-right}" y2="{py:.1f}"/>')
    parts.append(f'<text class="tick" x="{left-14}" y="{py+5:.1f}" text-anchor="end">{y:.1f}</text>')

for x in [4, 5, 6, 7, 8, 9, 10, 12, 15, 20, 25]:
    px = sx(float(x))
    parts.append(f'<line class="grid" x1="{px:.1f}" y1="{top}" x2="{px:.1f}" y2="{top+plot_h}"/>')
    parts.append(f'<text class="tick" x="{px:.1f}" y="{top+plot_h+28}" text-anchor="middle">{x}</text>')

parts.extend(
    [
        f'<line class="axis" x1="{left}" y1="{top+plot_h}" x2="{width-right}" y2="{top+plot_h}"/>',
        f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top+plot_h}"/>',
        f'<text class="tick" x="{left+plot_w/2:.1f}" y="{height-22}" text-anchor="middle">speed command</text>',
        f'<text class="tick" transform="translate(24 {top+plot_h/2:.1f}) rotate(-90)" text-anchor="middle">measured speed (m/s)</text>',
    ]
)

path = " ".join(
    ("M" if index == 0 else "L")
    + f" {sx(float(point['command'])):.1f} {sy(point['speed_mps']):.1f}"
    for index, point in enumerate(speed_points)
)
parts.append(f'<path class="line" d="{path}"/>')
for point in speed_points:
    x = sx(float(point["command"]))
    y = sy(point["speed_mps"])
    parts.append(f'<circle class="dot" cx="{x:.1f}" cy="{y:.1f}" r="6"/>')
parts.append("</svg>")
(MEDIA / "speed-command-curve.svg").write_text("\n".join(parts), encoding="utf-8")

print(json.dumps(summary["speed_range"], indent=2))
print(f"wrote {DATA / 'calibration_summary.json'}")
print(f"wrote {MEDIA / 'speed-command-curve.svg'}")
