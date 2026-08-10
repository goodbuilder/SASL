#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


DEFAULT_IMAGE_WIDTH_PX = 1280.0
DEFAULT_IMAGE_HEIGHT_PX = 720.0
DEFAULT_BASELINE_M = 25.0
DEFAULT_HORIZONTAL_FOV_RAD = 0.154997710698


@dataclass(frozen=True)
class Camera:
    width_px: float
    height_px: float
    fx_px: float
    baseline_m: float

    @property
    def cx_px(self) -> float:
        return self.width_px * 0.5

    @property
    def cy_px(self) -> float:
        return self.height_px * 0.5


@dataclass(frozen=True)
class BBox:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) * 0.5, (self.y1 + self.y2) * 0.5)

    def anchor(self, alpha_u: float, alpha_v: float) -> tuple[float, float]:
        return (self.x1 + alpha_u * self.width, self.y1 + alpha_v * self.height)


@dataclass(frozen=True)
class Sample:
    trial_t_s: float | None
    truth_right_m: float
    truth_down_m: float
    truth_forward_m: float
    left_bbox: BBox
    right_bbox: BBox
    raw_right_m: float
    raw_down_m: float
    raw_forward_m: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Train a bbox-internal origin proxy anchor from trial_01 truth reprojection "
            "and validate stereo triangulation on trial_02-06."
        )
    )
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--train-trial", type=int, default=1)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--image-width-px", type=float, default=DEFAULT_IMAGE_WIDTH_PX)
    parser.add_argument("--image-height-px", type=float, default=DEFAULT_IMAGE_HEIGHT_PX)
    parser.add_argument("--baseline-m", type=float, default=DEFAULT_BASELINE_M)
    parser.add_argument("--horizontal-fov-rad", type=float, default=DEFAULT_HORIZONTAL_FOV_RAD)
    parser.add_argument("--anchor-mode", choices=("global", "disparity_bin", "screen_grid"), default="global")
    parser.add_argument("--depth-bins", type=int, default=4)
    parser.add_argument("--screen-cols", type=int, default=5)
    parser.add_argument("--screen-rows", type=int, default=3)
    parser.add_argument("--min-bin-samples", type=int, default=30)
    parser.add_argument("--nearest-screen-cell", action="store_true")
    parser.add_argument(
        "--force-center-u",
        action="store_true",
        help="Keep left/right horizontal bbox anchor at 0.5 and only learn/apply vertical anchor fractions.",
    )
    parser.add_argument(
        "--right-optical-rotation-deg",
        default="",
        help=(
            "Optional actual-to-nominal optical roll,pitch,yaw in degrees. "
            "When empty, use metadata attitude_error_deg if present."
        ),
    )
    parser.add_argument("--allow-anchor-outside-bbox", action="store_true")
    return parser.parse_args()


def validation_trials(text: str) -> list[int]:
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def focal_px(width_px: float, horizontal_fov_rad: float) -> float:
    return (float(width_px) * 0.5) / math.tan(float(horizontal_fov_rad) * 0.5)


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def first_float(row: dict[str, str], names: Sequence[str]) -> float | None:
    for name in names:
        value = finite_float(row.get(name))
        if value is not None:
            return value
    return None


def load_samples(path: Path, camera: Camera) -> list[Sample]:
    samples: list[Sample] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            truth_right = first_float(row, ("truth_right_m", "gt_right_m"))
            truth_down = first_float(row, ("truth_down_m", "gt_down_m"))
            truth_forward = first_float(row, ("truth_forward_m", "gt_forward_m"))
            left_x1 = first_float(row, ("left_x1_px", "left_x1", "left_bbox_x1_px"))
            left_y1 = first_float(row, ("left_y1_px", "left_y1", "left_bbox_y1_px"))
            left_x2 = first_float(row, ("left_x2_px", "left_x2", "left_bbox_x2_px"))
            left_y2 = first_float(row, ("left_y2_px", "left_y2", "left_bbox_y2_px"))
            right_x1 = first_float(row, ("right_x1_px", "right_x1", "right_bbox_x1_px"))
            right_y1 = first_float(row, ("right_y1_px", "right_y1", "right_bbox_y1_px"))
            right_x2 = first_float(row, ("right_x2_px", "right_x2", "right_bbox_x2_px"))
            right_y2 = first_float(row, ("right_y2_px", "right_y2", "right_bbox_y2_px"))
            if None in (
                truth_right,
                truth_down,
                truth_forward,
                left_x1,
                left_y1,
                left_x2,
                left_y2,
                right_x1,
                right_y1,
                right_x2,
                right_y2,
            ):
                continue
            left_bbox = BBox(float(left_x1), float(left_y1), float(left_x2), float(left_y2))
            right_bbox = BBox(float(right_x1), float(right_y1), float(right_x2), float(right_y2))
            if left_bbox.width <= 1.0 or left_bbox.height <= 1.0 or right_bbox.width <= 1.0 or right_bbox.height <= 1.0:
                continue
            raw_right = first_float(row, ("estimated_right_m", "raw_right_m"))
            raw_down = first_float(row, ("estimated_down_m", "raw_down_m"))
            raw_forward = first_float(row, ("estimated_forward_m", "raw_forward_m"))
            if raw_right is None or raw_down is None or raw_forward is None:
                raw = triangulate_rectified(left_bbox.center, right_bbox.center, camera)
                if raw is None:
                    continue
                raw_right, raw_down, raw_forward = raw
            samples.append(
                Sample(
                    trial_t_s=first_float(row, ("trial_t_s",)),
                    truth_right_m=float(truth_right),
                    truth_down_m=float(truth_down),
                    truth_forward_m=float(truth_forward),
                    left_bbox=left_bbox,
                    right_bbox=right_bbox,
                    raw_right_m=float(raw_right),
                    raw_down_m=float(raw_down),
                    raw_forward_m=float(raw_forward),
                )
            )
    return samples


def matrix_from_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float) -> tuple[tuple[float, float, float], ...]:
    cr = math.cos(roll_rad)
    sr = math.sin(roll_rad)
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def matrix_transpose(matrix: tuple[tuple[float, float, float], ...]) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(matrix[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def matmul(
    a: tuple[tuple[float, float, float], ...],
    b: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )  # type: ignore[return-value]


def apply_matrix(
    matrix: tuple[tuple[float, float, float], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def parse_rotation_deg(text: str) -> tuple[float, float, float] | None:
    if not text.strip():
        return None
    parts = [float(part.strip()) for part in text.split(",") if part.strip()]
    if len(parts) != 3:
        raise ValueError("rotation must contain roll,pitch,yaw")
    return (math.radians(parts[0]), math.radians(parts[1]), math.radians(parts[2]))


def optical_actual_to_nominal_from_sdf_error(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> tuple[tuple[float, float, float], ...]:
    link_to_optical = (
        (0.0, -1.0, 0.0),
        (0.0, 0.0, -1.0),
        (1.0, 0.0, 0.0),
    )
    optical_to_link = matrix_transpose(link_to_optical)
    nominal_world_from_link = matrix_from_rpy(0.0, 0.0, math.pi * 0.5)
    actual_world_from_link = matrix_from_rpy(
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.pi * 0.5 + math.radians(yaw_deg),
    )
    nominal_to_actual = matmul(
        matmul(link_to_optical, matrix_transpose(actual_world_from_link)),
        matmul(nominal_world_from_link, optical_to_link),
    )
    return matrix_transpose(nominal_to_actual)


def rotation_matrix_from_report(args: argparse.Namespace) -> tuple[tuple[float, float, float], ...]:
    fixed = parse_rotation_deg(args.right_optical_rotation_deg)
    if fixed is not None:
        return matrix_from_rpy(*fixed)
    metadata_path = args.report_dir / "metadata.json"
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        attitude = metadata.get("attitude_error_deg") or {}
        return optical_actual_to_nominal_from_sdf_error(
            float(attitude.get("right_roll", 0.0)),
            float(attitude.get("right_pitch", 0.0)),
            float(attitude.get("right_yaw", 0.0)),
        )
    return matrix_from_rpy(0.0, 0.0, 0.0)


def project_truth(sample: Sample, camera: Camera, right_actual_to_nominal: tuple[tuple[float, float, float], ...]) -> dict[str, float]:
    left_u = camera.cx_px + (sample.truth_right_m + camera.baseline_m * 0.5) * camera.fx_px / sample.truth_forward_m
    left_v = camera.cy_px + sample.truth_down_m * camera.fx_px / sample.truth_forward_m
    right_nominal = (
        sample.truth_right_m - camera.baseline_m * 0.5,
        sample.truth_down_m,
        sample.truth_forward_m,
    )
    right_actual = apply_matrix(matrix_transpose(right_actual_to_nominal), right_nominal)
    right_u = camera.cx_px + right_actual[0] * camera.fx_px / right_actual[2]
    right_v = camera.cy_px + right_actual[1] * camera.fx_px / right_actual[2]
    return {"left_u": left_u, "left_v": left_v, "right_u": right_u, "right_v": right_v}


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a: tuple[float, float, float], value: float) -> tuple[float, float, float]:
    return (a[0] * value, a[1] * value, a[2] * value)


def normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(dot(v, v))
    if length <= 0.0:
        raise ValueError("zero-length vector")
    return (v[0] / length, v[1] / length, v[2] / length)


def triangulate_rays(
    left_ray: tuple[float, float, float],
    right_ray_in_left: tuple[float, float, float],
    camera: Camera,
) -> tuple[float, float, float] | None:
    d1 = normalize(left_ray)
    d2 = normalize(right_ray_in_left)
    translation = (camera.baseline_m, 0.0, 0.0)
    a = dot(d1, d1)
    b = dot(d1, d2)
    c = dot(d2, d2)
    w0 = scale(translation, -1.0)
    d = dot(d1, w0)
    e = dot(d2, w0)
    denom = a * c - b * b
    if abs(denom) <= 1.0e-12:
        return None
    s = (b * e - c * d) / denom
    u = (a * e - b * d) / denom
    if s <= 0.0 or u <= 0.0:
        return None
    midpoint_left = scale(add(scale(d1, s), add(translation, scale(d2, u))), 0.5)
    return (midpoint_left[0] - camera.baseline_m * 0.5, midpoint_left[1], midpoint_left[2])


def triangulate_with_rotation(
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    camera: Camera,
    right_actual_to_nominal: tuple[tuple[float, float, float], ...],
) -> tuple[float, float, float] | None:
    left_ray = normalize(((left_pixel[0] - camera.cx_px) / camera.fx_px, (left_pixel[1] - camera.cy_px) / camera.fx_px, 1.0))
    right_ray = normalize(
        ((right_pixel[0] - camera.cx_px) / camera.fx_px, (right_pixel[1] - camera.cy_px) / camera.fx_px, 1.0)
    )
    return triangulate_rays(left_ray, apply_matrix(right_actual_to_nominal, right_ray), camera)


def triangulate_rectified(
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    camera: Camera,
) -> tuple[float, float, float] | None:
    disparity = abs(left_pixel[0] - right_pixel[0])
    if disparity < 0.5:
        return None
    forward = camera.fx_px * camera.baseline_m / disparity
    right = ((left_pixel[0] + right_pixel[0]) * 0.5 - camera.cx_px) * forward / camera.fx_px
    down = ((left_pixel[1] + right_pixel[1]) * 0.5 - camera.cy_px) * forward / camera.fx_px
    return (right, down, forward)


def train_global_anchor(
    samples: Sequence[Sample],
    camera: Camera,
    right_actual_to_nominal: tuple[tuple[float, float, float], ...],
    *,
    allow_outside_bbox: bool,
) -> dict[str, float]:
    values: dict[str, list[float]] = {"left_u": [], "left_v": [], "right_u": [], "right_v": []}
    for sample in samples:
        projected = project_truth(sample, camera, right_actual_to_nominal)
        left_alpha_u = (projected["left_u"] - sample.left_bbox.x1) / sample.left_bbox.width
        left_alpha_v = (projected["left_v"] - sample.left_bbox.y1) / sample.left_bbox.height
        right_alpha_u = (projected["right_u"] - sample.right_bbox.x1) / sample.right_bbox.width
        right_alpha_v = (projected["right_v"] - sample.right_bbox.y1) / sample.right_bbox.height
        if not allow_outside_bbox:
            if not (0.0 <= left_alpha_u <= 1.0 and 0.0 <= left_alpha_v <= 1.0):
                continue
            if not (0.0 <= right_alpha_u <= 1.0 and 0.0 <= right_alpha_v <= 1.0):
                continue
        values["left_u"].append(left_alpha_u)
        values["left_v"].append(left_alpha_v)
        values["right_u"].append(right_alpha_u)
        values["right_v"].append(right_alpha_v)
    if not values["left_u"]:
        raise RuntimeError("no anchor training samples survived filtering")
    return {key: float(statistics.median(items)) for key, items in values.items()}


def center_disparity(sample: Sample) -> float:
    return abs(sample.left_bbox.center[0] - sample.right_bbox.center[0])


def quantile_edges(values: Sequence[float], bins: int) -> list[float]:
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        raise ValueError("cannot build bin edges from empty values")
    if bins <= 1:
        return [ordered[0] - 1.0e-6, ordered[-1] + 1.0e-6]
    edges: list[float] = []
    for index in range(bins + 1):
        position = (len(ordered) - 1) * index / bins
        lo = math.floor(position)
        hi = math.ceil(position)
        if lo == hi:
            edge = ordered[int(position)]
        else:
            edge = ordered[lo] * (hi - position) + ordered[hi] * (position - lo)
        edges.append(float(edge))
    edges[0] -= 1.0e-6
    edges[-1] += 1.0e-6
    return edges


def bin_index(value: float, edges: Sequence[float]) -> int | None:
    if len(edges) < 2 or not math.isfinite(float(value)):
        return None
    number = float(value)
    for index in range(len(edges) - 1):
        if edges[index] <= number < edges[index + 1] or (index == len(edges) - 2 and number <= edges[index + 1]):
            return index
    return None


def screen_cell_index(sample: Sample, camera: Camera, cols: int, rows: int) -> tuple[int, int] | None:
    if cols <= 0 or rows <= 0:
        return None
    u, v = sample.left_bbox.center
    if not (0.0 <= u <= camera.width_px and 0.0 <= v <= camera.height_px):
        return None
    col = min(cols - 1, max(0, int(u / camera.width_px * cols)))
    row = min(rows - 1, max(0, int(v / camera.height_px * rows)))
    return row, col


def nearest_screen_cell_key(
    sample: Sample,
    camera: Camera,
    cols: int,
    rows: int,
    cells: dict[str, dict[str, object]],
) -> str | None:
    current = screen_cell_index(sample, camera, cols, rows)
    if current is None:
        return None
    best_key: str | None = None
    best_distance = float("inf")
    for key in cells.keys():
        try:
            row_text, col_text = key.split("_", 1)
            row = int(row_text)
            col = int(col_text)
        except ValueError:
            continue
        distance = (row - current[0]) ** 2 + (col - current[1]) ** 2
        if distance < best_distance:
            best_distance = distance
            best_key = key
    return best_key


def train_anchor_profile(
    samples: Sequence[Sample],
    camera: Camera,
    right_actual_to_nominal: tuple[tuple[float, float, float], ...],
    *,
    mode: str,
    depth_bins: int,
    min_bin_samples: int,
    allow_outside_bbox: bool,
) -> dict[str, object]:
    global_anchor = train_global_anchor(
        samples,
        camera,
        right_actual_to_nominal,
        allow_outside_bbox=allow_outside_bbox,
    )
    profile: dict[str, object] = {
        "mode": mode,
        "global_anchor": global_anchor,
        "fallback": "global_anchor",
    }
    if mode == "global":
        return profile
    if mode == "disparity_bin":
        edges = quantile_edges([center_disparity(sample) for sample in samples], depth_bins)
        cells: dict[str, dict[str, object]] = {}
        for index in range(len(edges) - 1):
            cell_samples = [
                sample
                for sample in samples
                if (bin_index(center_disparity(sample), edges) == index)
            ]
            if len(cell_samples) < min_bin_samples:
                continue
            cells[str(index)] = {
                "sample_count": len(cell_samples),
                "anchor": train_global_anchor(
                    cell_samples,
                    camera,
                    right_actual_to_nominal,
                    allow_outside_bbox=allow_outside_bbox,
                ),
            }
        profile["disparity_edges_px"] = edges
        profile["cells"] = cells
        return profile
    if mode == "screen_grid":
        raise RuntimeError("screen_grid profile requires screen-specific arguments")
    return profile


def train_screen_grid_anchor_profile(
    samples: Sequence[Sample],
    camera: Camera,
    right_actual_to_nominal: tuple[tuple[float, float, float], ...],
    *,
    cols: int,
    rows: int,
    min_bin_samples: int,
    allow_outside_bbox: bool,
    nearest_fallback: bool,
) -> dict[str, object]:
    global_anchor = train_global_anchor(
        samples,
        camera,
        right_actual_to_nominal,
        allow_outside_bbox=allow_outside_bbox,
    )
    cells: dict[str, dict[str, object]] = {}
    for row in range(rows):
        for col in range(cols):
            cell_samples = [
                sample
                for sample in samples
                if screen_cell_index(sample, camera, cols, rows) == (row, col)
            ]
            if len(cell_samples) < min_bin_samples:
                continue
            cells[f"{row}_{col}"] = {
                "sample_count": len(cell_samples),
                "anchor": train_global_anchor(
                    cell_samples,
                    camera,
                    right_actual_to_nominal,
                    allow_outside_bbox=allow_outside_bbox,
                ),
            }
    return {
        "mode": "screen_grid",
        "global_anchor": global_anchor,
        "screen_cols": cols,
        "screen_rows": rows,
        "cells": cells,
        "fallback": "nearest_screen_cell" if nearest_fallback else "global_anchor",
        "nearest_screen_cell": nearest_fallback,
    }


def force_center_u(profile: dict[str, object]) -> None:
    global_anchor = profile.get("global_anchor")
    if isinstance(global_anchor, dict):
        global_anchor["left_u"] = 0.5
        global_anchor["right_u"] = 0.5
    cells = profile.get("cells")
    if not isinstance(cells, dict):
        return
    for cell in cells.values():
        if not isinstance(cell, dict):
            continue
        anchor = cell.get("anchor")
        if isinstance(anchor, dict):
            anchor["left_u"] = 0.5
            anchor["right_u"] = 0.5


def anchor_for_sample(profile: dict[str, object], sample: Sample, camera: Camera) -> tuple[dict[str, float], str]:
    global_anchor = dict(profile["global_anchor"])  # type: ignore[arg-type]
    mode = profile.get("mode")
    if mode == "global":
        return {key: float(value) for key, value in global_anchor.items()}, "global"
    if mode == "disparity_bin":
        edges = profile.get("disparity_edges_px")
        cells = profile.get("cells")
        if not isinstance(edges, list) or not isinstance(cells, dict):
            return {key: float(value) for key, value in global_anchor.items()}, "global_fallback"
        index = bin_index(center_disparity(sample), [float(value) for value in edges])
        if index is None:
            return {key: float(value) for key, value in global_anchor.items()}, "global_fallback"
        cell = cells.get(str(index))
        if not isinstance(cell, dict) or not isinstance(cell.get("anchor"), dict):
            return {key: float(value) for key, value in global_anchor.items()}, "global_fallback"
        return {key: float(value) for key, value in cell["anchor"].items()}, f"disparity_bin_{index}"
    if mode == "screen_grid":
        cells = profile.get("cells")
        cols = int(profile.get("screen_cols", 0))
        rows = int(profile.get("screen_rows", 0))
        if not isinstance(cells, dict):
            return {key: float(value) for key, value in global_anchor.items()}, "global_fallback"
        cell_index = screen_cell_index(sample, camera, cols, rows)
        cell_key = f"{cell_index[0]}_{cell_index[1]}" if cell_index is not None else None
        cell = cells.get(cell_key) if cell_key is not None else None
        if isinstance(cell, dict) and isinstance(cell.get("anchor"), dict):
            return {key: float(value) for key, value in cell["anchor"].items()}, f"screen_grid_{cell_key}"
        if profile.get("nearest_screen_cell"):
            nearest_key = nearest_screen_cell_key(sample, camera, cols, rows, cells)
            nearest_cell = cells.get(nearest_key) if nearest_key is not None else None
            if isinstance(nearest_cell, dict) and isinstance(nearest_cell.get("anchor"), dict):
                return {key: float(value) for key, value in nearest_cell["anchor"].items()}, f"screen_nearest_{nearest_key}"
        return {key: float(value) for key, value in global_anchor.items()}, "global_fallback"
    return {key: float(value) for key, value in global_anchor.items()}, "global_fallback"


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def evaluate(
    samples: Sequence[Sample],
    camera: Camera,
    right_actual_to_nominal: tuple[tuple[float, float, float], ...],
    anchor_profile: dict[str, object],
) -> tuple[list[dict[str, str]], dict[str, float | int | str]]:
    rows: list[dict[str, str]] = []
    errors = {
        "raw_right": [],
        "raw_down": [],
        "raw_forward": [],
        "anchor_right": [],
        "anchor_down": [],
        "anchor_forward": [],
        "center_rot_right": [],
        "center_rot_down": [],
        "center_rot_forward": [],
    }
    skipped = 0
    fallback_counts: dict[str, int] = {}
    for sample in samples:
        anchor, anchor_cell = anchor_for_sample(anchor_profile, sample, camera)
        fallback_counts[anchor_cell] = fallback_counts.get(anchor_cell, 0) + 1
        anchor_left = sample.left_bbox.anchor(anchor["left_u"], anchor["left_v"])
        anchor_right = sample.right_bbox.anchor(anchor["right_u"], anchor["right_v"])
        anchor_point = triangulate_with_rotation(anchor_left, anchor_right, camera, right_actual_to_nominal)
        center_point = triangulate_with_rotation(sample.left_bbox.center, sample.right_bbox.center, camera, right_actual_to_nominal)
        if anchor_point is None or center_point is None:
            skipped += 1
            continue
        raw = (sample.raw_right_m, sample.raw_down_m, sample.raw_forward_m)
        truth = (sample.truth_right_m, sample.truth_down_m, sample.truth_forward_m)
        raw_error = tuple(raw[index] - truth[index] for index in range(3))
        anchor_error = tuple(anchor_point[index] - truth[index] for index in range(3))
        center_error = tuple(center_point[index] - truth[index] for index in range(3))
        for axis, index in (("right", 0), ("down", 1), ("forward", 2)):
            errors[f"raw_{axis}"].append(raw_error[index])
            errors[f"anchor_{axis}"].append(anchor_error[index])
            errors[f"center_rot_{axis}"].append(center_error[index])
        rows.append(
            {
                "trial_t_s": "" if sample.trial_t_s is None else f"{sample.trial_t_s:.6f}",
                "truth_right_m": f"{truth[0]:.9f}",
                "truth_down_m": f"{truth[1]:.9f}",
                "truth_forward_m": f"{truth[2]:.9f}",
                "raw_right_m": f"{raw[0]:.9f}",
                "raw_down_m": f"{raw[1]:.9f}",
                "raw_forward_m": f"{raw[2]:.9f}",
                "anchor_left_u_px": f"{anchor_left[0]:.9f}",
                "anchor_left_v_px": f"{anchor_left[1]:.9f}",
                "anchor_right_u_px": f"{anchor_right[0]:.9f}",
                "anchor_right_v_px": f"{anchor_right[1]:.9f}",
                "anchor_cell": anchor_cell,
                "anchor_right_m": f"{anchor_point[0]:.9f}",
                "anchor_down_m": f"{anchor_point[1]:.9f}",
                "anchor_forward_m": f"{anchor_point[2]:.9f}",
                "center_rot_right_m": f"{center_point[0]:.9f}",
                "center_rot_down_m": f"{center_point[1]:.9f}",
                "center_rot_forward_m": f"{center_point[2]:.9f}",
                "raw_error_right_m": f"{raw_error[0]:.9f}",
                "raw_error_down_m": f"{raw_error[1]:.9f}",
                "raw_error_forward_m": f"{raw_error[2]:.9f}",
                "anchor_error_right_m": f"{anchor_error[0]:.9f}",
                "anchor_error_down_m": f"{anchor_error[1]:.9f}",
                "anchor_error_forward_m": f"{anchor_error[2]:.9f}",
            }
        )
    summary: dict[str, float | int | str] = {"sample_count": len(samples), "evaluated_count": len(rows), "skipped_count": skipped}
    for key, count in fallback_counts.items():
        summary[f"anchor_{key}_count"] = count
    pass_all = True
    for axis in ("right", "down", "forward"):
        raw_value = rmse(errors[f"raw_{axis}"])
        anchor_value = rmse(errors[f"anchor_{axis}"])
        center_value = rmse(errors[f"center_rot_{axis}"])
        improvement = raw_value - anchor_value
        summary[f"raw_{axis}_rmse_m"] = raw_value
        summary[f"anchor_{axis}_rmse_m"] = anchor_value
        summary[f"center_rot_{axis}_rmse_m"] = center_value
        summary[f"{axis}_rmse_improvement_m"] = improvement
        if not (math.isfinite(improvement) and improvement >= 0.0):
            pass_all = False
    raw_position = [
        math.sqrt(r * r + d * d + f * f)
        for r, d, f in zip(errors["raw_right"], errors["raw_down"], errors["raw_forward"])
    ]
    anchor_position = [
        math.sqrt(r * r + d * d + f * f)
        for r, d, f in zip(errors["anchor_right"], errors["anchor_down"], errors["anchor_forward"])
    ]
    summary["raw_position_rmse_m"] = rmse(raw_position)
    summary["anchor_position_rmse_m"] = rmse(anchor_position)
    summary["position_rmse_improvement_m"] = summary["raw_position_rmse_m"] - summary["anchor_position_rmse_m"]
    summary["pass"] = "yes" if pass_all else "no"
    return rows, summary


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    camera = Camera(
        width_px=float(args.image_width_px),
        height_px=float(args.image_height_px),
        fx_px=focal_px(args.image_width_px, args.horizontal_fov_rad),
        baseline_m=float(args.baseline_m),
    )
    rotation = rotation_matrix_from_report(args)
    normalized_dir = args.report_dir / "normalized"
    train_samples = load_samples(normalized_dir / f"trial_{args.train_trial:02d}.csv", camera)
    if not train_samples:
        raise RuntimeError("no training samples")
    if args.anchor_mode == "screen_grid":
        anchor_profile = train_screen_grid_anchor_profile(
            train_samples,
            camera,
            rotation,
            cols=int(args.screen_cols),
            rows=int(args.screen_rows),
            min_bin_samples=int(args.min_bin_samples),
            allow_outside_bbox=bool(args.allow_anchor_outside_bbox),
            nearest_fallback=bool(args.nearest_screen_cell),
        )
    else:
        anchor_profile = train_anchor_profile(
            train_samples,
            camera,
            rotation,
            mode=args.anchor_mode,
            depth_bins=args.depth_bins,
            min_bin_samples=args.min_bin_samples,
            allow_outside_bbox=bool(args.allow_anchor_outside_bbox),
        )
    if args.force_center_u:
        force_center_u(anchor_profile)

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = args.report_dir / f"bbox_anchor_global_{dt.datetime.now():%Y%m%d_%H%M%S}"
    profile = {
        "method": "bbox_internal_origin_anchor_profile_v1",
        "source_report_dir": str(args.report_dir),
        "train_trial": args.train_trial,
        "train_sample_count": len(train_samples),
        "anchor_mode": args.anchor_mode,
        "force_center_u": bool(args.force_center_u),
        "anchor_profile": anchor_profile,
        "camera": {
            "image_width_px": camera.width_px,
            "image_height_px": camera.height_px,
            "fx_px": camera.fx_px,
            "baseline_m": camera.baseline_m,
        },
        "right_actual_to_nominal_rotation": rotation,
        "label_contract": "anchor fractions are robust medians of truth-origin reprojection within each YOLO bbox",
        "application_contract": "runtime uses bbox x1/y1/width/height only, then stereo ray triangulation with right extrinsic rotation",
    }
    write_json(output_dir / "bbox_anchor_profile.json", profile)

    gate_rows: list[dict[str, str]] = []
    final_pass = True
    for trial in validation_trials(args.validation_trials):
        samples = load_samples(normalized_dir / f"trial_{trial:02d}.csv", camera)
        rows, summary = evaluate(samples, camera, rotation, anchor_profile)
        write_csv(output_dir / f"validation_trial_{trial:02d}_anchor_comparison.csv", rows)
        write_json(output_dir / f"validation_trial_{trial:02d}_summary.json", summary)
        if summary["pass"] != "yes":
            final_pass = False
        gate_rows.append(
            {
                "trial": str(trial),
                "sample_count": str(summary["sample_count"]),
                "evaluated_count": str(summary["evaluated_count"]),
                "right_rmse_improvement_m": f"{float(summary['right_rmse_improvement_m']):.9f}",
                "down_rmse_improvement_m": f"{float(summary['down_rmse_improvement_m']):.9f}",
                "forward_rmse_improvement_m": f"{float(summary['forward_rmse_improvement_m']):.9f}",
                "position_rmse_improvement_m": f"{float(summary['position_rmse_improvement_m']):.9f}",
                "pass": str(summary["pass"]),
            }
        )
    write_csv(output_dir / "validation_gate_summary.csv", gate_rows)
    (output_dir / "result.txt").write_text(
        f"final_pass={'yes' if final_pass else 'no'}\n"
        f"profile={output_dir / 'bbox_anchor_profile.json'}\n"
        f"summary={output_dir / 'validation_gate_summary.csv'}\n",
        encoding="utf-8",
    )
    print(f"Profile: {output_dir / 'bbox_anchor_profile.json'}")
    print(f"Summary: {output_dir / 'validation_gate_summary.csv'}")
    print(f"Final pass: {'yes' if final_pass else 'no'}")
    return 0 if final_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
