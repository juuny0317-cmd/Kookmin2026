#!/usr/bin/env python3
"""Rebuild the public perception and S-curve validation figures.

The script deliberately consumes one recorded camera frame and the two
recorded S-curve replay CSVs checked into ``evaluation/``.  It runs the exact
lane and scene checkpoints selected by the integrated launch configuration,
then records source/model checksums and numeric outputs in a manifest.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import matplotlib
import numpy as np
from ultralytics import YOLO


matplotlib.use("Agg")
from matplotlib import pyplot as plt  # noqa: E402


ORANGE = (0, 132, 255)
GREEN = (74, 222, 128)
CYAN = (235, 188, 73)
MAGENTA = (255, 102, 196)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_args() -> argparse.Namespace:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=repo)
    return parser.parse_args()


def infer(model_path: Path, frame_path: Path, image_size: int) -> list[dict]:
    model = YOLO(model_path)
    result = model(
        str(frame_path),
        imgsz=image_size,
        conf=0.25,
        device="cpu",
        verbose=False,
    )[0]
    detections = []
    for box, class_id, confidence in zip(
        result.boxes.xyxy.tolist(),
        result.boxes.cls.tolist(),
        result.boxes.conf.tolist(),
    ):
        detections.append(
            {
                "class": str(model.names[int(class_id)]),
                "confidence": round(float(confidence), 6),
                "xyxy": [round(float(value), 3) for value in box],
            }
        )
    return detections


def render_yolo(
    frame: np.ndarray,
    lane_detections: list[dict],
    scene_detections: list[dict],
    output_path: Path,
) -> None:
    output = frame.copy()
    cv2.rectangle(output, (0, 0), (639, 47), (9, 11, 15), -1)
    cv2.putText(
        output,
        "Recorded frame | final lane + scene checkpoints",
        (12, 19),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        ORANGE,
        1,
        cv2.LINE_AA,
    )
    groups = []
    for class_name, detections, color in (
        ("center_line", lane_detections, ORANGE),
        ("green", scene_detections, GREEN),
    ):
        matching = [item for item in detections if item["class"] == class_name]
        if matching:
            groups.append(
                (f"{class_name} x{len(matching)} | max "
                 f"{max(item['confidence'] for item in matching):.2f}", color)
            )
    label_x = 12
    for text, color in groups:
        cv2.putText(
            output,
            text,
            (label_x, 39),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            color,
            1,
            cv2.LINE_AA,
        )
        label_x += cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.43, 1
        )[0][0] + 22
    for detection in lane_detections + scene_detections:
        x1, y1, x2, y2 = (int(round(v)) for v in detection["xyxy"])
        color = ORANGE if detection["class"] == "center_line" else GREEN
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), output, [cv2.IMWRITE_JPEG_QUALITY, 94]):
        raise RuntimeError(f"Could not write {output_path}")


def render_opencv(
    frame: np.ndarray,
    lane_detections: list[dict],
    output_path: Path,
) -> dict:
    """Show the production ROI processing stages used after lane YOLO."""
    output = frame.copy()
    all_edges = np.zeros(frame.shape[:2], dtype=np.uint8)
    hough_segments = 0
    processed_rois = 0

    for detection in lane_detections:
        x1, y1, x2, y2 = (int(round(v)) for v in detection["xyxy"])
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
        if x2 <= x1 or y2 <= y1:
            continue
        roi = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        threshold = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            13,
            -20,
        )
        edges = cv2.Canny(threshold, 50, 150, apertureSize=3)
        all_edges[y1:y2, x1:x2] = np.maximum(
            all_edges[y1:y2, x1:x2], edges)
        lines = cv2.HoughLinesP(
            edges,
            1,
            np.pi / 180,
            threshold=20,
            minLineLength=20,
            maxLineGap=5,
        )
        if lines is not None:
            for line in lines[:, 0]:
                lx1, ly1, lx2, ly2 = (int(value) for value in line)
                cv2.line(
                    output,
                    (x1 + lx1, y1 + ly1),
                    (x1 + lx2, y1 + ly2),
                    MAGENTA,
                    2,
                    cv2.LINE_AA,
                )
            hough_segments += len(lines)
        cv2.rectangle(output, (x1, y1), (x2, y2), ORANGE, 1)
        processed_rois += 1

    edge_pixels = all_edges > 0
    edge_layer = np.zeros_like(output)
    edge_layer[edge_pixels] = CYAN
    output = cv2.addWeighted(output, 1.0, edge_layer, 0.72, 0.0)
    cv2.rectangle(output, (0, 0), (639, 48), (9, 11, 15), -1)
    cv2.putText(
        output,
        "OpenCV lane ROI: Adaptive Threshold > Canny > Hough",
        (12, 19),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        ORANGE,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        f"ROIs {processed_rois} | edge pixels {int(edge_pixels.sum())} | line segments {hough_segments}",
        (12, 39),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (220, 225, 232),
        1,
        cv2.LINE_AA,
    )
    if not cv2.imwrite(str(output_path), output, [cv2.IMWRITE_JPEG_QUALITY, 94]):
        raise RuntimeError(f"Could not write {output_path}")
    return {
        "processed_lane_rois": processed_rois,
        "edge_pixel_count": int(edge_pixels.sum()),
        "hough_segment_count": int(hough_segments),
    }


def read_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    columns = (
        "elapsed_s",
        "cte_px",
        "heading_error_deg",
        "target_speed_b",
        "command_speed_b",
    )
    return {
        column: np.asarray([float(row[column]) for row in rows], dtype=np.float64)
        for column in columns
    }


def run_metrics(data: dict[str, np.ndarray]) -> dict:
    absolute_cte = np.abs(data["cte_px"])
    speed_error = np.abs(data["target_speed_b"] - data["command_speed_b"])
    return {
        "samples": int(data["elapsed_s"].size),
        "duration_s": round(float(data["elapsed_s"][-1]), 6),
        "cte_abs_median_px": round(float(np.median(absolute_cte)), 6),
        "cte_abs_p95_px": round(float(np.percentile(absolute_cte, 95)), 6),
        "heading_abs_p95_deg": round(
            float(np.percentile(np.abs(data["heading_error_deg"]), 95)), 6
        ),
        "command_to_target_mae": round(float(np.mean(speed_error)), 6),
        "command_to_target_max_error": round(float(np.max(speed_error)), 6),
    }


def style_axis(axis) -> None:
    axis.set_facecolor("#0c1016")
    axis.grid(True, color="#313946", linewidth=0.6, alpha=0.55)
    axis.tick_params(colors="#bcc5d1")
    axis.xaxis.label.set_color("#bcc5d1")
    axis.yaxis.label.set_color("#bcc5d1")
    axis.title.set_color("#f3f4f6")
    for spine in axis.spines.values():
        spine.set_color("#3a4350")


def render_tracking_plot(runs: list[tuple[str, Path]], output_path: Path) -> dict:
    figure, axes = plt.subplots(2, 2, figsize=(14, 7.5), sharex="row")
    figure.patch.set_facecolor("#090b0f")
    metrics = {}
    for row_index, (label, path) in enumerate(runs):
        data = read_csv(path)
        metrics[label] = run_metrics(data)
        time_s = data["elapsed_s"]

        error_axis = axes[row_index, 0]
        error_axis.plot(time_s, data["cte_px"], color="#ff8500", linewidth=1.25)
        error_axis.axhline(0.0, color="#e5e7eb", linewidth=0.8, linestyle="--")
        error_axis.fill_between(
            time_s, -45.0, 45.0, color="#2e7d5b", alpha=0.12, label="±45 px reference"
        )
        error_axis.set_ylabel("Cross-track error (px)")
        error_axis.set_title(f"{label}: detected lane center vs image center")
        error_axis.legend(loc="upper right", frameon=False, labelcolor="#d1d5db")

        speed_axis = axes[row_index, 1]
        speed_axis.step(
            time_s,
            data["target_speed_b"],
            where="post",
            color="#ff8500",
            linewidth=1.5,
            label="policy target",
        )
        speed_axis.plot(
            time_s,
            data["command_speed_b"],
            color="#49b6ff",
            linewidth=1.2,
            label="rate-limited command",
        )
        speed_axis.set_ylabel("Speed command")
        speed_axis.set_title(f"{label}: target and published command")
        speed_axis.legend(loc="lower right", frameon=False, labelcolor="#d1d5db")

        for axis in axes[row_index]:
            style_axis(axis)
            axis.set_xlabel("Replay time (s)")

    figure.suptitle(
        "S-curve replay evidence — lateral error and speed-command tracking",
        color="#ff8500",
        fontsize=16,
        fontweight="bold",
    )
    figure.text(
        0.5,
        0.012,
        "Recorded perception replay; command is controller output, not measured wheel speed.",
        ha="center",
        color="#9ca3af",
        fontsize=9,
    )
    figure.tight_layout(rect=(0, 0.035, 1, 0.95))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, facecolor=figure.get_facecolor())
    plt.close(figure)
    return metrics


def main() -> None:
    args = parse_args()
    repo = args.repo.resolve()
    frame_path = repo / "evaluation/perception/frame_0490_t231582ms.jpg"
    lane_model = repo / "ros2_ws/src/cam/cam/center_line_yolov10n_320_best.pt"
    scene_model = repo / "ros2_ws/src/cam/cam/all_second_yolo_v2_yolov10n_640_best_e46.pt"
    run_paths = [
        ("Run 02", repo / "evaluation/s_curve/s_curve_run02_ab.csv"),
        ("Run 03", repo / "evaluation/s_curve/s_curve_run03_ab.csv"),
    ]

    frame = cv2.imread(str(frame_path))
    if frame is None:
        raise FileNotFoundError(frame_path)
    lane_detections = infer(lane_model, frame_path, 320)
    scene_detections = infer(scene_model, frame_path, 640)
    render_yolo(
        frame,
        lane_detections,
        scene_detections,
        repo / "media/perception/yolo_lane_scene.jpg",
    )
    opencv_metrics = render_opencv(
        frame,
        lane_detections,
        repo / "media/perception/opencv_lane_roi.jpg",
    )
    tracking_metrics = render_tracking_plot(
        run_paths,
        repo / "media/validation/s_curve_tracking.png",
    )

    inputs = [frame_path, lane_model, scene_model, *(path for _, path in run_paths)]
    manifest = {
        "method": {
            "lane_model_image_size": 320,
            "scene_model_image_size": 640,
            "confidence_threshold": 0.25,
            "device": "cpu",
            "opencv": {
                "adaptive_threshold": "GAUSSIAN_C, block=13, C=-20",
                "canny": [50, 150],
                "hough_lines_p": {
                    "threshold": 20,
                    "min_line_length": 20,
                    "max_line_gap": 5,
                },
            },
        },
        "input_sha256": {str(path.relative_to(repo)): sha256(path) for path in inputs},
        "detections": {
            "lane": lane_detections,
            "scene": scene_detections,
        },
        "opencv_result": opencv_metrics,
        "s_curve_replay_metrics": tracking_metrics,
    }
    manifest_path = repo / "evaluation/validation_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
