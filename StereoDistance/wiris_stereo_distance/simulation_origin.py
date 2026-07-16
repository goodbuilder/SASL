from __future__ import annotations

import csv
import json
import math
import random
import statistics
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from wiris_stereo_distance.coordinate_contract import FRD_COMPARISON_AXIS_CONTRACT, MEASUREMENT_FRAME
from wiris_stereo_distance.reprojection_profile import (
    DEFAULT_REPROJECTION_CORRECTION_MODE,
    DEFAULT_REPROJECTION_DISPARITY_GAIN,
    DEFAULT_REPROJECTION_OUTPUT_POLICY,
    DEFAULT_REPROJECTION_SCREEN_U_GAIN,
    DEFAULT_REPROJECTION_SCREEN_V_GAIN,
    ORIGIN_BBOX_REPROJECTION_METHOD,
    OriginBBoxReprojectionProfile,
    RectifiedStereoCameraModel,
    calibration_envelope_from_intrinsics,
    load_reprojection_profile,
    residual_samples_from_rows,
    save_reprojection_profile,
)
from wiris_stereo_distance.uam_telemetry import UamAlignmentProfile, estimate_alignment_profile_from_rows


SIM_AXIS_CONTRACT = FRD_COMPARISON_AXIS_CONTRACT
DEFAULT_TARGET_WIDTH_M = 15.0
DEFAULT_OVERLAP_MARGIN_M = 3.0
DEFAULT_PROFILE_RANGES_M = (1100.0, 900.0, 700.0, 500.0, 380.0, 320.0)
DEFAULT_LATERAL_FRACTIONS = (-0.65, 0.55, -0.45, 0.4, -0.25, 0.0)
DEFAULT_ALTITUDE_OFFSETS_M = (-2.0, 2.0, -1.0, 1.0, 0.0, 0.0)
ORIGIN_BBOX_PROFILE_METHOD = "origin-bbox-grid-v1"
DEFAULT_IMAGE_WIDTH_PX = 1280.0
DEFAULT_IMAGE_HEIGHT_PX = 720.0
DEFAULT_CAMERA_HORIZONTAL_FOV_RAD = 0.154997710698
DEFAULT_CAMERA_BASELINE_M = 25.0
DEFAULT_ORIGIN_BBOX_U_BINS = (0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0)
DEFAULT_ORIGIN_BBOX_V_BINS = (0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0)
DEFAULT_ORIGIN_BBOX_RANGE_BINS_M = (0.0, 450.0, 650.0, 850.0, 1050.0, 1250.0, 2000.0)
DEFAULT_ORIGIN_BBOX_MIN_CELL_SAMPLES = 50
DEFAULT_ORIGIN_BBOX_RESIDUAL_GAIN = 0.2
DEFAULT_ORIGIN_BBOX_MIN_CORRECTION_NORM_M = 0.25
DEFAULT_ORIGIN_BBOX_MIN_RANGE_CORRECTION_M = 0.25
DEFAULT_ORIGIN_BBOX_AXIS_GAIN_CANDIDATES = (0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.33, 0.5, 0.75, 1.0)
DEFAULT_ORIGIN_BBOX_MIN_VALIDATION_CELL_SAMPLES = 10
DEFAULT_ORIGIN_BBOX_MIN_AXIS_IMPROVEMENT_M = 0.02
DEFAULT_ORIGIN_BBOX_MIN_AXIS_IMPROVEMENT_RATIO = 0.001
DEFAULT_ORIGIN_BBOX_AXIS_MODEL_KINDS = ("axis_range", "axis_poly", "axis_screen_poly")
DEFAULT_ORIGIN_BBOX_AXIS_MODEL_RIDGE_LAMBDAS = (0.0, 0.1, 1.0, 10.0, 100.0, 1000.0)
DEFAULT_RANDOM_ORIGIN_LATERAL_FRACTION_LIMIT = 0.85
DEFAULT_RANDOM_ORIGIN_ALTITUDE_JITTER_M = 2.5


def _coerce_float(value: object) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        parsed = float(text)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def _first_float(row: dict[str, object], fields: Sequence[str]) -> float | None:
    for field in fields:
        value = _coerce_float(row.get(field))
        if value is not None:
            return value
    return None


def _norm3(values: tuple[float, float, float]) -> float:
    return math.sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2])


def _sub3(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _fmt_float(value: float | None) -> str:
    if value is None:
        return ""
    return f"{float(value):.6f}"


def _tuple_complete(values: tuple[float | None, float | None, float | None]) -> bool:
    return all(value is not None for value in values)


def _add3(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def _median3(values: Sequence[tuple[float, float, float]]) -> tuple[float, float, float]:
    return (
        statistics.median(value[0] for value in values),
        statistics.median(value[1] for value in values),
        statistics.median(value[2] for value in values),
    )


def _bin_index(value: float, bins: Sequence[float]) -> int | None:
    if len(bins) < 2:
        return None
    if value < bins[0] or value > bins[-1]:
        return None
    for index in range(len(bins) - 1):
        if bins[index] <= value < bins[index + 1]:
            return index
    if value == bins[-1]:
        return len(bins) - 2
    return None


def _bin_center(bins: Sequence[float], index: int) -> float:
    return (float(bins[index]) + float(bins[index + 1])) * 0.5


def _origin_bbox_screen_features(row: dict[str, object], image_width_px: float, image_height_px: float) -> tuple[float, float]:
    left_cx = _first_float(row, ("left_cx",))
    left_cy = _first_float(row, ("left_cy",))
    right_cx = _first_float(row, ("right_cx",))
    right_cy = _first_float(row, ("right_cy",))
    if None in (left_cx, right_cx):
        u_norm = 0.5
    else:
        u_norm = max(0.0, min(1.0, ((float(left_cx) + float(right_cx)) * 0.5) / image_width_px))
    if None in (left_cy, right_cy):
        v_norm = 0.5
    else:
        v_norm = max(0.0, min(1.0, ((float(left_cy) + float(right_cy)) * 0.5) / image_height_px))
    return u_norm, v_norm


def _origin_bbox_axis_model_features(
    row: dict[str, object],
    *,
    axis_name: str,
    feature_kind: str,
    image_width_px: float,
    image_height_px: float,
) -> list[float] | None:
    measurement = _first_float(
        row,
        (
            f"measurement_{axis_name}_m",
            f"raw_{axis_name}_m",
            f"stereo_center_{axis_name}_m",
        ),
    )
    range_m = _first_float(row, ("yolo_range_m", "estimated_range_m", "raw_range_m", "truth_range_m"))
    if measurement is None or range_m is None:
        return None
    measurement = float(measurement)
    range_km = float(range_m) / 1000.0
    u_norm, v_norm = _origin_bbox_screen_features(row, image_width_px, image_height_px)
    if feature_kind == "axis_range":
        return [1.0, measurement, range_km]
    if feature_kind == "axis_poly":
        return [1.0, measurement, measurement * measurement, range_km, measurement * range_km]
    if feature_kind == "axis_screen_poly":
        return [
            1.0,
            measurement,
            measurement * measurement,
            range_km,
            measurement * range_km,
            u_norm,
            v_norm,
            measurement * u_norm,
        ]
    return None


def _fit_origin_bbox_axis_model(
    rows: Sequence[dict[str, object]],
    *,
    axis_name: str,
    feature_kind: str,
    ridge_lambda: float,
    image_width_px: float,
    image_height_px: float,
) -> dict[str, object] | None:
    features: list[list[float]] = []
    targets: list[float] = []
    for row in rows:
        values = _origin_bbox_axis_model_features(
            row,
            axis_name=axis_name,
            feature_kind=feature_kind,
            image_width_px=image_width_px,
            image_height_px=image_height_px,
        )
        raw_error = _first_float(row, (f"raw_error_{axis_name}_m",))
        if values is None or raw_error is None:
            continue
        features.append(values)
        targets.append(-float(raw_error))
    if len(features) < max(8, len(features[0]) if features else 0):
        return None
    matrix = np.asarray(features, dtype=float)
    target = np.asarray(targets, dtype=float)
    mean = np.mean(matrix[:, 1:], axis=0)
    scale = np.std(matrix[:, 1:], axis=0)
    scale[scale < 1e-9] = 1.0
    normalized = matrix.copy()
    normalized[:, 1:] = (matrix[:, 1:] - mean) / scale
    regularizer = np.eye(normalized.shape[1], dtype=float)
    regularizer[0, 0] = 0.0
    try:
        coefficients = np.linalg.solve(
            normalized.T @ normalized + float(ridge_lambda) * regularizer,
            normalized.T @ target,
        )
    except np.linalg.LinAlgError:
        return None
    return {
        "axis": axis_name,
        "feature_kind": feature_kind,
        "ridge_lambda": float(ridge_lambda),
        "coefficients": [float(value) for value in coefficients],
        "feature_mean": [float(value) for value in mean],
        "feature_scale": [float(value) for value in scale],
        "sample_count": len(features),
    }


def _predict_origin_bbox_axis_model(
    row: dict[str, object],
    model: dict[str, object],
    *,
    image_width_px: float,
    image_height_px: float,
) -> float | None:
    values = _origin_bbox_axis_model_features(
        row,
        axis_name=str(model.get("axis", "")),
        feature_kind=str(model.get("feature_kind", "")),
        image_width_px=image_width_px,
        image_height_px=image_height_px,
    )
    if values is None:
        return None
    coefficients = [float(value) for value in model.get("coefficients", [])]
    mean = [float(value) for value in model.get("feature_mean", [])]
    scale = [float(value) for value in model.get("feature_scale", [])]
    if len(coefficients) != len(values) or len(mean) != len(values) - 1 or len(scale) != len(values) - 1:
        return None
    normalized = list(values)
    for index in range(1, len(normalized)):
        normalized[index] = (normalized[index] - mean[index - 1]) / scale[index - 1]
    return float(sum(value * coefficient for value, coefficient in zip(normalized, coefficients)))


@dataclass(frozen=True)
class OriginBboxPrediction:
    corrected_m: tuple[float, float, float]
    corrected_range_m: float
    residual_m: tuple[float, float, float]
    range_residual_m: float
    fallback_level: str
    cell_key: str
    sample_count: int


@dataclass(frozen=True)
class OriginBboxProfile:
    profile_id: str
    image_width_px: float
    image_height_px: float
    u_bins: tuple[float, ...]
    v_bins: tuple[float, ...]
    range_bins_m: tuple[float, ...]
    cells: dict[str, dict[str, object]]
    global_residual_m: tuple[float, float, float]
    global_range_residual_m: float
    sample_count: int
    min_cell_samples: int
    residual_gain: float
    min_correction_norm_m: float
    min_range_correction_m: float
    training_range_min_m: float
    training_range_max_m: float
    created_unix_s: float
    method: str = ORIGIN_BBOX_PROFILE_METHOD
    measurement_frame: str = MEASUREMENT_FRAME
    truth_frame: str = MEASUREMENT_FRAME

    def to_dict(self) -> dict[str, object]:
        return {
            "profile_id": self.profile_id,
            "method": self.method,
            "measurement_frame": self.measurement_frame,
            "truth_frame": self.truth_frame,
            "axis_contract": SIM_AXIS_CONTRACT,
            "image_width_px": self.image_width_px,
            "image_height_px": self.image_height_px,
            "u_bins": list(self.u_bins),
            "v_bins": list(self.v_bins),
            "range_bins_m": list(self.range_bins_m),
            "cells": self.cells,
            "global_residual_m": list(self.global_residual_m),
            "global_range_residual_m": self.global_range_residual_m,
            "sample_count": self.sample_count,
            "min_cell_samples": self.min_cell_samples,
            "residual_gain": self.residual_gain,
            "min_correction_norm_m": self.min_correction_norm_m,
            "min_range_correction_m": self.min_range_correction_m,
            "training_range_min_m": self.training_range_min_m,
            "training_range_max_m": self.training_range_max_m,
            "created_unix_s": self.created_unix_s,
        }

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> "OriginBboxProfile":
        return cls(
            profile_id=str(payload["profile_id"]),
            image_width_px=float(payload.get("image_width_px", DEFAULT_IMAGE_WIDTH_PX)),
            image_height_px=float(payload.get("image_height_px", DEFAULT_IMAGE_HEIGHT_PX)),
            u_bins=tuple(float(value) for value in payload.get("u_bins", DEFAULT_ORIGIN_BBOX_U_BINS)),
            v_bins=tuple(float(value) for value in payload.get("v_bins", DEFAULT_ORIGIN_BBOX_V_BINS)),
            range_bins_m=tuple(
                float(value) for value in payload.get("range_bins_m", DEFAULT_ORIGIN_BBOX_RANGE_BINS_M)
            ),
            cells={str(key): dict(value) for key, value in dict(payload.get("cells", {})).items()},
            global_residual_m=tuple(float(value) for value in payload.get("global_residual_m", (0.0, 0.0, 0.0))),
            global_range_residual_m=float(payload.get("global_range_residual_m", 0.0)),
            sample_count=int(payload.get("sample_count", 0)),
            min_cell_samples=int(payload.get("min_cell_samples", DEFAULT_ORIGIN_BBOX_MIN_CELL_SAMPLES)),
            residual_gain=float(payload.get("residual_gain", DEFAULT_ORIGIN_BBOX_RESIDUAL_GAIN)),
            min_correction_norm_m=float(
                payload.get("min_correction_norm_m", DEFAULT_ORIGIN_BBOX_MIN_CORRECTION_NORM_M)
            ),
            min_range_correction_m=float(
                payload.get("min_range_correction_m", DEFAULT_ORIGIN_BBOX_MIN_RANGE_CORRECTION_M)
            ),
            training_range_min_m=float(payload.get("training_range_min_m", 0.0)),
            training_range_max_m=float(payload.get("training_range_max_m", 0.0)),
            created_unix_s=float(payload.get("created_unix_s", time.time())),
            method=str(payload.get("method", ORIGIN_BBOX_PROFILE_METHOD)),
            measurement_frame=str(payload.get("measurement_frame", MEASUREMENT_FRAME)),
            truth_frame=str(payload.get("truth_frame", MEASUREMENT_FRAME)),
        )

    def _cell_key(self, range_bin: int, u_bin: int, v_bin: int) -> str:
        return f"r{range_bin}:u{u_bin}:v{v_bin}"

    def _features_from_row(self, row: dict[str, object]) -> tuple[float, float, float] | None:
        left_cx = _first_float(row, ("left_cx",))
        left_cy = _first_float(row, ("left_cy",))
        right_cx = _first_float(row, ("right_cx",))
        right_cy = _first_float(row, ("right_cy",))
        range_m = _first_float(row, ("yolo_range_m", "estimated_range_m", "raw_range_m", "truth_range_m"))
        if None in (left_cx, left_cy, right_cx, right_cy, range_m):
            return None
        u_norm = max(0.0, min(1.0, ((float(left_cx) + float(right_cx)) * 0.5) / self.image_width_px))
        v_norm = max(0.0, min(1.0, ((float(left_cy) + float(right_cy)) * 0.5) / self.image_height_px))
        return u_norm, v_norm, float(range_m)

    def _cell_center(self, cell: dict[str, object]) -> tuple[float, float, float]:
        range_bin = int(cell["range_bin"])
        u_bin = int(cell["u_bin"])
        v_bin = int(cell["v_bin"])
        return (
            _bin_center(self.u_bins, u_bin),
            _bin_center(self.v_bins, v_bin),
            _bin_center(self.range_bins_m, range_bin),
        )

    def _nearest_cell(self, u_norm: float, v_norm: float, range_m: float) -> tuple[str, dict[str, object]] | None:
        if not self.cells:
            return None
        range_scale = max(1.0, self.range_bins_m[-1] - self.range_bins_m[0])
        best: tuple[float, str, dict[str, object]] | None = None
        for key, cell in self.cells.items():
            cell_u, cell_v, cell_range = self._cell_center(cell)
            distance = math.sqrt(
                (u_norm - cell_u) * (u_norm - cell_u)
                + (v_norm - cell_v) * (v_norm - cell_v)
                + ((range_m - cell_range) / range_scale) ** 2
            )
            if best is None or distance < best[0]:
                best = (distance, key, cell)
        return None if best is None else (best[1], best[2])

    def predict_row(self, row: dict[str, object]) -> OriginBboxPrediction | None:
        features = self._features_from_row(row)
        if features is None:
            return None
        u_norm, v_norm, range_m = features
        measured = (
            _first_float(row, ("measurement_right_m", "raw_right_m", "stereo_center_right_m")),
            _first_float(row, ("measurement_down_m", "raw_down_m", "stereo_center_down_m")),
            _first_float(row, ("measurement_forward_m", "raw_forward_m", "stereo_center_forward_m")),
        )
        if not _tuple_complete(measured):
            return None
        range_bin = _bin_index(range_m, self.range_bins_m)
        u_bin = _bin_index(u_norm, self.u_bins)
        v_bin = _bin_index(v_norm, self.v_bins)
        cell_key = ""
        cell = None
        fallback_level = "global"
        if None not in (range_bin, u_bin, v_bin):
            candidate_key = self._cell_key(int(range_bin), int(u_bin), int(v_bin))
            cell = self.cells.get(candidate_key)
            if cell is not None:
                cell_key = candidate_key
                fallback_level = "exact"
        if cell is None and range_bin is not None and u_bin is not None and v_bin is not None:
            same_range = [
                (key, value)
                for key, value in self.cells.items()
                if int(value.get("range_bin", -1)) == int(range_bin)
            ]
            if same_range:
                cell_key, cell = min(
                    same_range,
                    key=lambda item: (
                        int(item[1].get("u_bin", 0)) - int(u_bin)
                    )
                    ** 2
                    + (int(item[1].get("v_bin", 0)) - int(v_bin)) ** 2,
                )
                fallback_level = "same_range_nearest"
        if cell is None:
            nearest = self._nearest_cell(u_norm, v_norm, range_m)
            if nearest is not None and self.training_range_min_m <= range_m <= self.training_range_max_m:
                cell_key, cell = nearest
                fallback_level = "nearest"
        if cell is None:
            residual = self.global_residual_m
            sample_count = self.sample_count
        else:
            residual_values = cell.get("residual_m", self.global_residual_m)
            residual = tuple(float(value) for value in residual_values)
            range_residual = float(cell.get("range_residual_m", self.global_range_residual_m))
            sample_count = int(cell.get("count", 0))
        if cell is None:
            range_residual = self.global_range_residual_m
        axis_profile_allowed = fallback_level == "exact"
        axis_gains = None if cell is None or not axis_profile_allowed else cell.get("axis_gains")
        axis_models = None if cell is None or not axis_profile_allowed else cell.get("axis_models")
        modeled_residual = [None, None, None]
        if isinstance(axis_models, dict):
            for axis_index, axis_name in enumerate(("right", "down", "forward")):
                model = axis_models.get(axis_name)
                if isinstance(model, dict):
                    modeled_residual[axis_index] = _predict_origin_bbox_axis_model(
                        row,
                        model,
                        image_width_px=self.image_width_px,
                        image_height_px=self.image_height_px,
                    )
        if not axis_profile_allowed and cell is not None and ("axis_gains" in cell or "axis_models" in cell):
            residual = (0.0, 0.0, 0.0)
            fallback_level = f"{fallback_level}_axis_noop"
        elif axis_gains is None:
            residual = tuple(
                float(modeled_residual[index])
                if modeled_residual[index] is not None
                else residual[index] * self.residual_gain
                for index in range(3)
            )
            if _norm3(residual) < self.min_correction_norm_m:
                residual = (0.0, 0.0, 0.0)
                fallback_level = f"{fallback_level}_deadband"
        else:
            gains = tuple(float(value) for value in axis_gains)
            residual = tuple(
                float(modeled_residual[index])
                if modeled_residual[index] is not None
                else residual[index] * gains[index]
                for index in range(3)
            )
        range_residual *= self.residual_gain
        if abs(range_residual) < self.min_range_correction_m:
            range_residual = 0.0
        corrected = _add3(
            (float(measured[0]), float(measured[1]), float(measured[2])),
            (float(residual[0]), float(residual[1]), float(residual[2])),
        )
        measured_range_m = _first_float(row, ("yolo_range_m", "estimated_range_m", "raw_range_m", "truth_range_m"))
        corrected_range_m = _norm3(corrected) if measured_range_m is None else float(measured_range_m) + range_residual
        return OriginBboxPrediction(
            corrected_m=corrected,
            corrected_range_m=corrected_range_m,
            residual_m=(float(residual[0]), float(residual[1]), float(residual[2])),
            range_residual_m=float(range_residual),
            fallback_level=fallback_level,
            cell_key=cell_key,
            sample_count=sample_count,
        )


def read_env_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.exists():
        return values
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


def _env_float(env: dict[str, str], key: str, default: float) -> float:
    value = _coerce_float(env.get(key))
    return default if value is None else value


def generate_origin_waypoints(
    *,
    drone_env: dict[str, str],
    camera_env: dict[str, str],
    target_width_m: float = DEFAULT_TARGET_WIDTH_M,
    overlap_margin_m: float = DEFAULT_OVERLAP_MARGIN_M,
    ranges_m: Sequence[float] = DEFAULT_PROFILE_RANGES_M,
) -> list[dict[str, float]]:
    spawn_y_m = _env_float(drone_env, "DRONE_SPAWN_Y_M", 2000.0)
    camera_height_m = _env_float(camera_env, "CAMERA_HEIGHT_M", 10.0)
    baseline_m = _env_float(camera_env, "CAMERA_BASELINE_M", 25.0)
    hfov_rad = _env_float(camera_env, "CAMERA_HORIZONTAL_FOV_RAD", 0.154997710698)
    half_tan = math.tan(hfov_rad / 2.0)
    target_half_width_m = target_width_m / 2.0
    waypoints: list[dict[str, float]] = []
    for index, range_m in enumerate(ranges_m):
        full_overlap_center_m = range_m * half_tan - baseline_m / 2.0 - target_half_width_m - overlap_margin_m
        safe_lateral_m = max(0.0, full_overlap_center_m)
        fraction = DEFAULT_LATERAL_FRACTIONS[index % len(DEFAULT_LATERAL_FRACTIONS)]
        right_m = safe_lateral_m * fraction
        altitude_m = camera_height_m + DEFAULT_ALTITUDE_OFFSETS_M[index % len(DEFAULT_ALTITUDE_OFFSETS_M)]
        waypoints.append(
            {
                "north": range_m - spawn_y_m,
                "east": right_m,
                "altitude": max(1.0, altitude_m),
                "planned_right_m": right_m,
                "planned_down_m": camera_height_m - max(1.0, altitude_m),
                "planned_forward_m": range_m,
                "range_m": range_m,
                "max_full_overlap_center_m": safe_lateral_m,
            }
        )
    return waypoints


def generate_random_origin_waypoints(
    *,
    drone_env: dict[str, str],
    camera_env: dict[str, str],
    seed: int,
    target_width_m: float = DEFAULT_TARGET_WIDTH_M,
    overlap_margin_m: float = DEFAULT_OVERLAP_MARGIN_M,
    range_min_m: float = min(DEFAULT_PROFILE_RANGES_M),
    range_max_m: float = max(DEFAULT_PROFILE_RANGES_M),
    waypoint_count: int = len(DEFAULT_PROFILE_RANGES_M),
    lateral_fraction_limit: float = DEFAULT_RANDOM_ORIGIN_LATERAL_FRACTION_LIMIT,
    altitude_jitter_m: float = DEFAULT_RANDOM_ORIGIN_ALTITUDE_JITTER_M,
) -> list[dict[str, float]]:
    if waypoint_count < 2:
        raise ValueError("random origin path needs at least 2 waypoints")
    if range_min_m <= 0.0 or range_max_m <= range_min_m:
        raise ValueError("random origin path expects 0 < range_min_m < range_max_m")
    if not 0.0 <= lateral_fraction_limit <= 1.0:
        raise ValueError("lateral_fraction_limit must be between 0 and 1")
    rng = random.Random(int(seed))
    spawn_y_m = _env_float(drone_env, "DRONE_SPAWN_Y_M", 2000.0)
    camera_height_m = _env_float(camera_env, "CAMERA_HEIGHT_M", 10.0)
    baseline_m = _env_float(camera_env, "CAMERA_BASELINE_M", 25.0)
    hfov_rad = _env_float(camera_env, "CAMERA_HORIZONTAL_FOV_RAD", 0.154997710698)
    half_tan = math.tan(hfov_rad / 2.0)
    target_half_width_m = target_width_m / 2.0
    ranges = sorted(
        (rng.uniform(float(range_min_m), float(range_max_m)) for _ in range(int(waypoint_count))),
        reverse=True,
    )
    fractions = [rng.uniform(-lateral_fraction_limit, lateral_fraction_limit) for _ in ranges]
    if len(fractions) >= 3 and all(value >= 0.0 for value in fractions):
        fractions[-1] = -abs(fractions[-1] or lateral_fraction_limit * 0.5)
    if len(fractions) >= 3 and all(value <= 0.0 for value in fractions):
        fractions[0] = abs(fractions[0] or lateral_fraction_limit * 0.5)
    waypoints: list[dict[str, float]] = []
    for range_m, fraction in zip(ranges, fractions):
        full_overlap_center_m = range_m * half_tan - baseline_m / 2.0 - target_half_width_m - overlap_margin_m
        safe_lateral_m = max(0.0, full_overlap_center_m)
        right_m = safe_lateral_m * fraction
        altitude_m = max(1.0, camera_height_m + rng.uniform(-altitude_jitter_m, altitude_jitter_m))
        waypoints.append(
            {
                "north": range_m - spawn_y_m,
                "east": right_m,
                "altitude": altitude_m,
                "planned_right_m": right_m,
                "planned_down_m": camera_height_m - altitude_m,
                "planned_forward_m": range_m,
                "range_m": range_m,
                "max_full_overlap_center_m": safe_lateral_m,
            }
        )
    return waypoints


def write_waypoint_csv(path: Path, waypoints: Sequence[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = (
        "north",
        "east",
        "altitude",
        "range_m",
        "planned_right_m",
        "planned_down_m",
        "planned_forward_m",
        "max_full_overlap_center_m",
    )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for waypoint in waypoints:
            writer.writerow({field: f"{float(waypoint[field]):.6f}" for field in fields})


def load_telemetry_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def simulation_comparison_rows(rows: Iterable[dict[str, object]]) -> list[dict[str, object]]:
    comparisons: list[dict[str, object]] = []
    for index, row in enumerate(rows, start=1):
        axis_contract = str(row.get("axis_contract") or row.get("measurement_axis_contract") or "").lower()
        legacy_is_frd = "forward_right_down" in axis_contract
        if legacy_is_frd:
            measurement_legacy_fields = (("estimated_y_m",), ("estimated_z_m",), ("estimated_x_m",))
            truth_legacy_fields = (("drone_y_m",), ("drone_z_m",), ("drone_x_m",))
        else:
            measurement_legacy_fields = (("estimated_x_m",), ("estimated_y_m",), ("estimated_z_m",))
            truth_legacy_fields = (("drone_x_m",), ("drone_y_m",), ("drone_z_m",))
        measurement = (
            _first_float(row, ("raw_right_m", "estimated_right_m", "stereo_center_right_m", "measurement_right_m", *measurement_legacy_fields[0])),
            _first_float(row, ("raw_down_m", "estimated_down_m", "stereo_center_down_m", "measurement_down_m", *measurement_legacy_fields[1])),
            _first_float(row, ("raw_forward_m", "estimated_forward_m", "stereo_center_forward_m", "measurement_forward_m", *measurement_legacy_fields[2])),
        )
        truth = (
            _first_float(row, ("drone_right_m", "truth_right_m", "truth_rdf_right_m", *truth_legacy_fields[0])),
            _first_float(row, ("drone_down_m", "truth_down_m", "truth_rdf_down_m", *truth_legacy_fields[1])),
            _first_float(row, ("drone_forward_m", "truth_forward_m", "truth_rdf_forward_m", *truth_legacy_fields[2])),
        )
        if None in measurement or None in truth:
            continue
        corrected_candidate = (
            _first_float(row, ("corrected_right_m",)),
            _first_float(row, ("corrected_down_m",)),
            _first_float(row, ("corrected_forward_m",)),
        )
        corrected: tuple[float, float, float] | None = None
        if _tuple_complete(corrected_candidate):
            corrected = (
                float(corrected_candidate[0]),
                float(corrected_candidate[1]),
                float(corrected_candidate[2]),
            )
        timestamp_s = _first_float(row, ("time_sec", "t_s", "ros_time_sec")) or float(index)
        measured = (float(measurement[0]), float(measurement[1]), float(measurement[2]))
        actual = (float(truth[0]), float(truth[1]), float(truth[2]))
        measured_frd = (measured[2], measured[0], measured[1])
        actual_frd = (actual[2], actual[0], actual[1])
        raw_error = _sub3(measured, actual)
        raw_frd_error = (raw_error[2], raw_error[0], raw_error[1])
        raw_position_error_m = _norm3(raw_error)
        yolo_range_m = _first_float(row, ("raw_range_m", "estimated_range_m", "measured_m", "target_distance_m"))
        if yolo_range_m is None:
            yolo_range_m = _norm3(measured)
        truth_range_m = _first_float(row, ("ground_truth_m", "truth_range_m"))
        if truth_range_m is None:
            truth_range_m = _norm3(actual)
        raw_range_error_m = float(yolo_range_m) - float(truth_range_m)
        corrected_range_m = _first_float(row, ("corrected_range_m",))
        corrected_frd: tuple[float, float, float] | None = None
        corrected_error: tuple[float, float, float] | None = None
        corrected_frd_error: tuple[float, float, float] | None = None
        corrected_position_error_m: float | None = None
        corrected_range_error_m: float | None = None
        if corrected is not None:
            corrected_frd = (corrected[2], corrected[0], corrected[1])
            corrected_error = _sub3(corrected, actual)
            corrected_frd_error = (corrected_error[2], corrected_error[0], corrected_error[1])
            corrected_position_error_m = _norm3(corrected_error)
            if corrected_range_m is None:
                corrected_range_m = _norm3(corrected)
            corrected_range_error_m = float(corrected_range_m) - float(truth_range_m)
        comparison = {
                "row_index": index,
                "timestamp_s": f"{timestamp_s:.6f}",
                "measurement_frame": MEASUREMENT_FRAME,
                "measurement_axis_contract": SIM_AXIS_CONTRACT,
                "camera_comparison_axis_contract": SIM_AXIS_CONTRACT,
                "truth_frame": MEASUREMENT_FRAME,
                "truth_axis_contract": SIM_AXIS_CONTRACT,
                "stereo_center_right_m": f"{measured[0]:.6f}",
                "stereo_center_down_m": f"{measured[1]:.6f}",
                "stereo_center_forward_m": f"{measured[2]:.6f}",
                "measurement_right_m": f"{measured[0]:.6f}",
                "measurement_down_m": f"{measured[1]:.6f}",
                "measurement_forward_m": f"{measured[2]:.6f}",
                "measurement_frd_forward_m": f"{measured_frd[0]:.6f}",
                "measurement_frd_right_m": f"{measured_frd[1]:.6f}",
                "measurement_frd_down_m": f"{measured_frd[2]:.6f}",
                "raw_right_m": f"{measured[0]:.6f}",
                "raw_down_m": f"{measured[1]:.6f}",
                "raw_forward_m": f"{measured[2]:.6f}",
                "raw_frd_forward_m": f"{measured_frd[0]:.6f}",
                "raw_frd_right_m": f"{measured_frd[1]:.6f}",
                "raw_frd_down_m": f"{measured_frd[2]:.6f}",
                "truth_right_m": f"{actual[0]:.6f}",
                "truth_down_m": f"{actual[1]:.6f}",
                "truth_forward_m": f"{actual[2]:.6f}",
                "truth_frd_forward_m": f"{actual_frd[0]:.6f}",
                "truth_frd_right_m": f"{actual_frd[1]:.6f}",
                "truth_frd_down_m": f"{actual_frd[2]:.6f}",
                "estimated_range_m": f"{float(yolo_range_m):.6f}",
                "yolo_range_m": f"{float(yolo_range_m):.6f}",
                "truth_range_m": f"{float(truth_range_m):.6f}",
                "raw_error_right_m": f"{raw_error[0]:.6f}",
                "raw_error_down_m": f"{raw_error[1]:.6f}",
                "raw_error_forward_m": f"{raw_error[2]:.6f}",
                "raw_error_frd_forward_m": f"{raw_frd_error[0]:.6f}",
                "raw_error_frd_right_m": f"{raw_frd_error[1]:.6f}",
                "raw_error_frd_down_m": f"{raw_frd_error[2]:.6f}",
                "raw_range_error_m": f"{raw_range_error_m:.6f}",
                "raw_position_error_m": f"{raw_position_error_m:.6f}",
                "corrected_right_m": "" if corrected is None else f"{corrected[0]:.6f}",
                "corrected_down_m": "" if corrected is None else f"{corrected[1]:.6f}",
                "corrected_forward_m": "" if corrected is None else f"{corrected[2]:.6f}",
                "corrected_frd_forward_m": "" if corrected_frd is None else f"{corrected_frd[0]:.6f}",
                "corrected_frd_right_m": "" if corrected_frd is None else f"{corrected_frd[1]:.6f}",
                "corrected_frd_down_m": "" if corrected_frd is None else f"{corrected_frd[2]:.6f}",
                "corrected_error_right_m": "" if corrected_error is None else f"{corrected_error[0]:.6f}",
                "corrected_error_down_m": "" if corrected_error is None else f"{corrected_error[1]:.6f}",
                "corrected_error_forward_m": "" if corrected_error is None else f"{corrected_error[2]:.6f}",
                "corrected_error_frd_forward_m": "" if corrected_frd_error is None else f"{corrected_frd_error[0]:.6f}",
                "corrected_error_frd_right_m": "" if corrected_frd_error is None else f"{corrected_frd_error[1]:.6f}",
                "corrected_error_frd_down_m": "" if corrected_frd_error is None else f"{corrected_frd_error[2]:.6f}",
                "corrected_range_m": _fmt_float(corrected_range_m),
                "corrected_range_error_m": _fmt_float(corrected_range_error_m),
                "corrected_position_error_m": _fmt_float(corrected_position_error_m),
                "left_cx": row.get("left_cx", row.get("left_bbox_center_x_px", "")),
                "left_cy": row.get("left_cy", row.get("left_bbox_center_y_px", "")),
                "right_cx": row.get("right_cx", row.get("right_bbox_center_x_px", "")),
                "right_cy": row.get("right_cy", row.get("right_bbox_center_y_px", "")),
                "left_corrected_cx": row.get("left_corrected_cx", ""),
                "left_corrected_cy": row.get("left_corrected_cy", ""),
                "right_corrected_cx": row.get("right_corrected_cx", ""),
                "right_corrected_cy": row.get("right_corrected_cy", ""),
                "left_bbox_du_px": row.get("left_bbox_du_px", ""),
                "left_bbox_dv_px": row.get("left_bbox_dv_px", ""),
                "right_bbox_du_px": row.get("right_bbox_du_px", ""),
                "right_bbox_dv_px": row.get("right_bbox_dv_px", ""),
                "left_confidence": row.get("left_confidence", row.get("left_bbox_confidence", "")),
                "right_confidence": row.get("right_confidence", row.get("right_bbox_confidence", "")),
                "left_width_px": row.get("left_width_px", row.get("left_bbox_width_px", "")),
                "left_height_px": row.get("left_height_px", row.get("left_bbox_height_px", "")),
                "right_width_px": row.get("right_width_px", row.get("right_bbox_width_px", "")),
                "right_height_px": row.get("right_height_px", row.get("right_bbox_height_px", "")),
                "raw_disparity_px": row.get("raw_disparity_px", row.get("disparity_px", "")),
                "bbox_available": str(bool(str(row.get("left_cx") or "").strip() and str(row.get("right_cx") or "").strip())).lower(),
                "bbox_corrected_available": str(corrected is not None).lower(),
            }
        comparisons.append(comparison)
    return comparisons


def _summary(values: Sequence[float], prefix: str) -> dict[str, float]:
    if not values:
        return {}
    abs_values = sorted(abs(value) for value in values)
    p95_index = min(len(abs_values) - 1, int(math.ceil(len(abs_values) * 0.95)) - 1)
    return {
        f"{prefix}_count": float(len(values)),
        f"{prefix}_mean_m": statistics.fmean(values),
        f"{prefix}_median_abs_m": abs_values[len(abs_values) // 2],
        f"{prefix}_p95_abs_m": abs_values[p95_index],
        f"{prefix}_rmse_m": math.sqrt(statistics.fmean(value * value for value in values)),
    }


def build_profile_from_telemetry(
    telemetry_csv: Path,
    *,
    profile_id: str,
    min_samples: int = 30,
) -> tuple[UamAlignmentProfile, list[dict[str, object]]]:
    rows = simulation_comparison_rows(load_telemetry_rows(telemetry_csv))
    if len(rows) < min_samples:
        raise RuntimeError(f"not enough matched simulation rows for profile: got {len(rows)}, need {min_samples}")
    profile = estimate_alignment_profile_from_rows(rows, profile_id=profile_id, min_samples=min_samples)
    profile = replace(profile, measurement_frame=MEASUREMENT_FRAME, truth_frame=MEASUREMENT_FRAME)
    return profile, rows


def build_origin_bbox_profile_from_telemetry(
    telemetry_csv: Path,
    *,
    profile_id: str,
    min_samples: int = 30,
    validation_telemetry_csv: Path | None = None,
    image_width_px: float = DEFAULT_IMAGE_WIDTH_PX,
    image_height_px: float = DEFAULT_IMAGE_HEIGHT_PX,
    u_bins: Sequence[float] = DEFAULT_ORIGIN_BBOX_U_BINS,
    v_bins: Sequence[float] = DEFAULT_ORIGIN_BBOX_V_BINS,
    range_bins_m: Sequence[float] = DEFAULT_ORIGIN_BBOX_RANGE_BINS_M,
    min_cell_samples: int = DEFAULT_ORIGIN_BBOX_MIN_CELL_SAMPLES,
    residual_gain: float = DEFAULT_ORIGIN_BBOX_RESIDUAL_GAIN,
    min_correction_norm_m: float = DEFAULT_ORIGIN_BBOX_MIN_CORRECTION_NORM_M,
    min_range_correction_m: float = DEFAULT_ORIGIN_BBOX_MIN_RANGE_CORRECTION_M,
    axis_gain_candidates: Sequence[float] = DEFAULT_ORIGIN_BBOX_AXIS_GAIN_CANDIDATES,
    axis_model_kinds: Sequence[str] = DEFAULT_ORIGIN_BBOX_AXIS_MODEL_KINDS,
    disabled_axes: Sequence[str] = (),
    min_validation_cell_samples: int = DEFAULT_ORIGIN_BBOX_MIN_VALIDATION_CELL_SAMPLES,
    min_axis_improvement_m: float = DEFAULT_ORIGIN_BBOX_MIN_AXIS_IMPROVEMENT_M,
    min_axis_improvement_ratio: float = DEFAULT_ORIGIN_BBOX_MIN_AXIS_IMPROVEMENT_RATIO,
) -> tuple[OriginBboxProfile, list[dict[str, object]]]:
    rows = simulation_comparison_rows(load_telemetry_rows(telemetry_csv))
    if len(rows) < min_samples:
        raise RuntimeError(f"not enough matched simulation rows for origin-bbox profile: got {len(rows)}, need {min_samples}")

    residuals_by_cell: dict[str, list[tuple[float, float, float]]] = {}
    range_residuals_by_cell: dict[str, list[float]] = {}
    ranges: list[float] = []
    all_residuals: list[tuple[float, float, float]] = []
    all_range_residuals: list[float] = []
    temp_profile = OriginBboxProfile(
        profile_id=profile_id,
        image_width_px=float(image_width_px),
        image_height_px=float(image_height_px),
        u_bins=tuple(float(value) for value in u_bins),
        v_bins=tuple(float(value) for value in v_bins),
        range_bins_m=tuple(float(value) for value in range_bins_m),
        cells={},
        global_residual_m=(0.0, 0.0, 0.0),
        global_range_residual_m=0.0,
        sample_count=0,
        min_cell_samples=int(min_cell_samples),
        residual_gain=float(residual_gain),
        min_correction_norm_m=float(min_correction_norm_m),
        min_range_correction_m=float(min_range_correction_m),
        training_range_min_m=0.0,
        training_range_max_m=0.0,
        created_unix_s=time.time(),
    )

    for row in rows:
        features = temp_profile._features_from_row(row)
        if features is None:
            continue
        u_norm, v_norm, range_m = features
        measured = (
            _first_float(row, ("measurement_right_m", "raw_right_m")),
            _first_float(row, ("measurement_down_m", "raw_down_m")),
            _first_float(row, ("measurement_forward_m", "raw_forward_m")),
        )
        truth = (
            _first_float(row, ("truth_right_m",)),
            _first_float(row, ("truth_down_m",)),
            _first_float(row, ("truth_forward_m",)),
        )
        yolo_range_m = _first_float(row, ("yolo_range_m", "estimated_range_m", "raw_range_m"))
        truth_range_m = _first_float(row, ("truth_range_m",))
        if not _tuple_complete(measured) or not _tuple_complete(truth):
            continue
        residual = (
            float(truth[0]) - float(measured[0]),
            float(truth[1]) - float(measured[1]),
            float(truth[2]) - float(measured[2]),
        )
        range_residual = 0.0 if yolo_range_m is None or truth_range_m is None else float(truth_range_m) - float(yolo_range_m)
        range_bin = _bin_index(range_m, temp_profile.range_bins_m)
        u_bin = _bin_index(u_norm, temp_profile.u_bins)
        v_bin = _bin_index(v_norm, temp_profile.v_bins)
        if None in (range_bin, u_bin, v_bin):
            continue
        key = temp_profile._cell_key(int(range_bin), int(u_bin), int(v_bin))
        residuals_by_cell.setdefault(key, []).append(residual)
        range_residuals_by_cell.setdefault(key, []).append(range_residual)
        ranges.append(range_m)
        all_residuals.append(residual)
        all_range_residuals.append(range_residual)

    if len(all_residuals) < min_samples:
        raise RuntimeError(
            f"not enough origin-bbox residual samples after bbox/range filtering: got {len(all_residuals)}, need {min_samples}"
        )

    cells: dict[str, dict[str, object]] = {}
    for key, residual_values in residuals_by_cell.items():
        if len(residual_values) < int(min_cell_samples):
            continue
        parts = key.replace("r", "").replace("u", "").replace("v", "").split(":")
        range_bin, u_bin, v_bin = (int(part) for part in parts)
        residual_m = _median3(residual_values)
        range_residual_m = statistics.median(range_residuals_by_cell.get(key, (0.0,)))
        residual_norms = [_norm3(value) for value in residual_values]
        abs_range_residuals = sorted(abs(value) for value in range_residuals_by_cell.get(key, (0.0,)))
        cells[key] = {
            "range_bin": range_bin,
            "u_bin": u_bin,
            "v_bin": v_bin,
            "count": len(residual_values),
            "residual_m": [residual_m[0], residual_m[1], residual_m[2]],
            "range_residual_m": range_residual_m,
            "residual_norm_median_m": statistics.median(residual_norms),
            "residual_norm_p95_m": sorted(residual_norms)[min(len(residual_norms) - 1, int(math.ceil(len(residual_norms) * 0.95)) - 1)],
            "range_residual_median_abs_m": abs_range_residuals[len(abs_range_residuals) // 2],
            "range_residual_p95_abs_m": abs_range_residuals[
                min(len(abs_range_residuals) - 1, int(math.ceil(len(abs_range_residuals) * 0.95)) - 1)
            ],
        }

    if not cells:
        raise RuntimeError(
            f"origin-bbox profile has no populated cells; lower min_cell_samples below {int(min_cell_samples)}"
        )
    if validation_telemetry_csv is not None:
        validation_rows = simulation_comparison_rows(load_telemetry_rows(validation_telemetry_csv))
        _apply_origin_bbox_axis_validation_gates(
            cells=cells,
            training_rows=rows,
            validation_rows=validation_rows,
            image_width_px=float(image_width_px),
            image_height_px=float(image_height_px),
            u_bins=tuple(float(value) for value in u_bins),
            v_bins=tuple(float(value) for value in v_bins),
            range_bins_m=tuple(float(value) for value in range_bins_m),
            gain_candidates=axis_gain_candidates,
            axis_model_kinds=axis_model_kinds,
            min_cell_samples=int(min_validation_cell_samples),
            min_improvement_m=float(min_axis_improvement_m),
            min_improvement_ratio=float(min_axis_improvement_ratio),
        )
    _disable_origin_bbox_axes(cells, disabled_axes)

    return (
        OriginBboxProfile(
            profile_id=profile_id,
            image_width_px=float(image_width_px),
            image_height_px=float(image_height_px),
            u_bins=tuple(float(value) for value in u_bins),
            v_bins=tuple(float(value) for value in v_bins),
            range_bins_m=tuple(float(value) for value in range_bins_m),
            cells=cells,
            global_residual_m=_median3(all_residuals),
            global_range_residual_m=statistics.median(all_range_residuals),
            sample_count=len(all_residuals),
            min_cell_samples=int(min_cell_samples),
            residual_gain=float(residual_gain),
            min_correction_norm_m=float(min_correction_norm_m),
            min_range_correction_m=float(min_range_correction_m),
            training_range_min_m=min(ranges),
            training_range_max_m=max(ranges),
            created_unix_s=time.time(),
        ),
        rows,
    )


def _default_focal_px(image_width_px: float) -> float:
    return (float(image_width_px) * 0.5) / math.tan(DEFAULT_CAMERA_HORIZONTAL_FOV_RAD * 0.5)


def _camera_model_from_rows(
    rows: Sequence[dict[str, object]],
    *,
    image_width_px: float = DEFAULT_IMAGE_WIDTH_PX,
    image_height_px: float = DEFAULT_IMAGE_HEIGHT_PX,
    focal_length_px: float | None = None,
    baseline_m: float | None = None,
) -> RectifiedStereoCameraModel:
    first = rows[0] if rows else {}
    width = _first_float(first, ("left_intr_width", "image_width_px"))
    height = _first_float(first, ("left_intr_height", "image_height_px"))
    left_fx = _first_float(first, ("left_intr_fx", "runtime_focal_px"))
    left_fy = _first_float(first, ("left_intr_fy",))
    left_cx = _first_float(first, ("left_intr_cx",))
    left_cy = _first_float(first, ("left_intr_cy",))
    right_cx = _first_float(first, ("right_intr_cx",))
    right_cy = _first_float(first, ("right_intr_cy",))
    baseline = _first_float(first, ("baseline_m", "manual_baseline_setting_m"))
    return RectifiedStereoCameraModel(
        image_width_px=float(width if width is not None else image_width_px),
        image_height_px=float(height if height is not None else image_height_px),
        focal_length_px=float(
            focal_length_px
            if focal_length_px is not None
            else (left_fx if left_fx is not None else _default_focal_px(image_width_px))
        ),
        baseline_m=float(baseline_m if baseline_m is not None else (baseline if baseline is not None else DEFAULT_CAMERA_BASELINE_M)),
        left_cx_px=left_cx,
        left_cy_px=left_cy,
        right_cx_px=right_cx,
        right_cy_px=right_cy,
        fy_px=left_fy,
    )


def build_origin_bbox_reprojection_profile_from_telemetry(
    telemetry_csv: Path,
    *,
    profile_id: str,
    min_samples: int = 30,
    image_width_px: float = DEFAULT_IMAGE_WIDTH_PX,
    image_height_px: float = DEFAULT_IMAGE_HEIGHT_PX,
    focal_length_px: float | None = None,
    baseline_m: float | None = None,
    u_bins: int = 6,
    v_bins: int = 4,
    depth_bins: int = 6,
    min_cell_samples: int = 3,
    bin_axis: str = "disparity",
    lookup_policy: str = "exact",
    correction_mode: str = DEFAULT_REPROJECTION_CORRECTION_MODE,
    disparity_gain: float = DEFAULT_REPROJECTION_DISPARITY_GAIN,
    screen_u_gain: float = DEFAULT_REPROJECTION_SCREEN_U_GAIN,
    screen_v_gain: float = DEFAULT_REPROJECTION_SCREEN_V_GAIN,
    output_policy: str = DEFAULT_REPROJECTION_OUTPUT_POLICY,
) -> tuple[OriginBBoxReprojectionProfile, list[dict[str, object]]]:
    rows = simulation_comparison_rows(load_telemetry_rows(telemetry_csv))
    camera = _camera_model_from_rows(
        rows,
        image_width_px=image_width_px,
        image_height_px=image_height_px,
        focal_length_px=focal_length_px,
        baseline_m=baseline_m,
    )
    samples = residual_samples_from_rows(rows, camera)
    if len(samples) < min_samples:
        raise RuntimeError(
            f"not enough reprojection residual samples after bbox/truth filtering: got {len(samples)}, need {min_samples}"
        )
    profile = OriginBBoxReprojectionProfile.build(
        samples,
        image_width_px=camera.image_width_px,
        image_height_px=camera.image_height_px,
        u_bins=u_bins,
        v_bins=v_bins,
        depth_bins=depth_bins,
        min_samples_per_cell=min_cell_samples,
        profile_id=profile_id,
        bin_axis=bin_axis,
        lookup_policy=lookup_policy,
        correction_mode=correction_mode,
        disparity_gain=disparity_gain,
        screen_u_gain=screen_u_gain,
        screen_v_gain=screen_v_gain,
        output_policy=output_policy,
        calibration_envelope=calibration_envelope_from_intrinsics(
            None,
            profile_source="simulation-origin-telemetry-build",
        ),
    )
    return profile, rows


def evaluate_origin_bbox_reprojection_profile_rows(
    rows: Sequence[dict[str, object]],
    profile: OriginBBoxReprojectionProfile,
    *,
    image_width_px: float = DEFAULT_IMAGE_WIDTH_PX,
    image_height_px: float = DEFAULT_IMAGE_HEIGHT_PX,
    focal_length_px: float | None = None,
    baseline_m: float | None = None,
) -> tuple[list[dict[str, object]], dict[str, float | str]]:
    evaluated: list[dict[str, object]] = []
    camera = _camera_model_from_rows(
        rows,
        image_width_px=image_width_px,
        image_height_px=image_height_px,
        focal_length_px=focal_length_px,
        baseline_m=baseline_m,
    )
    raw_range_errors: list[float] = []
    raw_position_errors: list[float] = []
    corrected_range_errors: list[float] = []
    corrected_position_errors: list[float] = []
    raw_axis_errors: dict[str, list[float]] = {"right": [], "down": [], "forward": []}
    corrected_axis_errors: dict[str, list[float]] = {"right": [], "down": [], "forward": []}
    fallback_counts: dict[str, int] = {}
    for row in rows:
        truth = (
            _first_float(row, ("truth_right_m",)),
            _first_float(row, ("truth_down_m",)),
            _first_float(row, ("truth_forward_m",)),
        )
        left_u = _first_float(row, ("left_cx", "left_bbox_center_x_px"))
        left_v = _first_float(row, ("left_cy", "left_bbox_center_y_px"))
        right_u = _first_float(row, ("right_cx", "right_bbox_center_x_px"))
        right_v = _first_float(row, ("right_cy", "right_bbox_center_y_px"))
        if not _tuple_complete(truth) or None in (left_u, left_v, right_u, right_v):
            continue
        raw_disparity = _first_float(row, ("raw_disparity_px", "disparity_px"))
        if raw_disparity is None:
            raw_disparity = abs(float(left_u) - float(right_u))
        raw_range = _first_float(row, ("raw_range_m", "estimated_range_m", "measurement_distance_m", "distance_m"))
        lookup = profile.apply(
            float(left_u),
            float(left_v),
            float(right_u),
            float(right_v),
            raw_disparity_px=raw_disparity,
            raw_range_m=raw_range,
        )
        try:
            corrected = camera.triangulate_centers(
                left_u=lookup.corrected_left_u,
                left_v=lookup.corrected_left_v,
                right_u=lookup.corrected_right_u,
                right_v=lookup.corrected_right_v,
            )
        except ValueError:
            continue
        truth_point = (float(truth[0]), float(truth[1]), float(truth[2]))
        corrected_point = (corrected["right_m"], corrected["down_m"], corrected["forward_m"])
        if profile.output_policy == "range_only_forward_fusion":
            raw_right = _first_float(row, ("raw_right_m", "stereo_center_right_m", "measurement_right_m"))
            raw_down = _first_float(row, ("raw_down_m", "stereo_center_down_m", "measurement_down_m"))
            if raw_right is not None and raw_down is not None:
                corrected_point = (float(raw_right), float(raw_down), corrected_point[2])
        error = _sub3(corrected_point, truth_point)
        error_frd = (error[2], error[0], error[1])
        truth_range_m = float(row["truth_range_m"])
        corrected_range_m = _norm3(corrected_point)
        range_error_m = corrected_range_m - truth_range_m
        position_error_m = _norm3(error)
        raw_range_error_m = _first_float(row, ("raw_range_error_m",))
        raw_position_error_m = _first_float(row, ("raw_position_error_m",))
        if raw_range_error_m is not None:
            raw_range_errors.append(raw_range_error_m)
        if raw_position_error_m is not None:
            raw_position_errors.append(raw_position_error_m)
        for axis_name, axis_index in (("right", 0), ("down", 1), ("forward", 2)):
            raw_axis_error_m = _first_float(row, (f"raw_error_{axis_name}_m",))
            if raw_axis_error_m is not None:
                raw_axis_errors[axis_name].append(raw_axis_error_m)
            corrected_axis_errors[axis_name].append(error[axis_index])
        corrected_range_errors.append(range_error_m)
        corrected_position_errors.append(position_error_m)
        fallback_counts[lookup.fallback_reason] = fallback_counts.get(lookup.fallback_reason, 0) + 1
        output = dict(row)
        output.update(
            {
                "origin_bbox_reprojection_profile_id": profile.profile_id,
                "origin_bbox_reprojection_corrected_right_m": f"{corrected_point[0]:.6f}",
                "origin_bbox_reprojection_corrected_down_m": f"{corrected_point[1]:.6f}",
                "origin_bbox_reprojection_corrected_forward_m": f"{corrected_point[2]:.6f}",
                "origin_bbox_reprojection_corrected_range_m": f"{corrected_range_m:.6f}",
                "origin_bbox_reprojection_corrected_frd_forward_m": f"{corrected_point[2]:.6f}",
                "origin_bbox_reprojection_corrected_frd_right_m": f"{corrected_point[0]:.6f}",
                "origin_bbox_reprojection_corrected_frd_down_m": f"{corrected_point[1]:.6f}",
                "origin_bbox_reprojection_error_right_m": f"{error[0]:.6f}",
                "origin_bbox_reprojection_error_down_m": f"{error[1]:.6f}",
                "origin_bbox_reprojection_error_forward_m": f"{error[2]:.6f}",
                "origin_bbox_reprojection_error_frd_forward_m": f"{error_frd[0]:.6f}",
                "origin_bbox_reprojection_error_frd_right_m": f"{error_frd[1]:.6f}",
                "origin_bbox_reprojection_error_frd_down_m": f"{error_frd[2]:.6f}",
                "origin_bbox_reprojection_range_error_m": f"{range_error_m:.6f}",
                "origin_bbox_reprojection_position_error_m": f"{position_error_m:.6f}",
                "left_corrected_cx": f"{lookup.corrected_left_u:.6f}",
                "left_corrected_cy": f"{lookup.corrected_left_v:.6f}",
                "right_corrected_cx": f"{lookup.corrected_right_u:.6f}",
                "right_corrected_cy": f"{lookup.corrected_right_v:.6f}",
                "left_bbox_du_px": f"{lookup.du_left:.6f}",
                "left_bbox_dv_px": f"{lookup.dv_left:.6f}",
                "right_bbox_du_px": f"{lookup.du_right:.6f}",
                "right_bbox_dv_px": f"{lookup.dv_right:.6f}",
                "origin_bbox_reprojection_fallback_level": str(lookup.fallback_level),
                "origin_bbox_reprojection_fallback_reason": lookup.fallback_reason,
                "origin_bbox_reprojection_cell_key": lookup.cell_key,
                "origin_bbox_reprojection_cell_sample_count": lookup.sample_count,
            }
        )
        evaluated.append(output)

    summary: dict[str, float | str] = {
        "profile_id": profile.profile_id,
        "profile_method": profile.method,
        "sample_count": float(len(evaluated)),
        "profile_cell_count": float(len(profile.cells)),
        "profile_bin_axis": profile.bin_axis,
        "profile_lookup_policy": profile.lookup_policy,
        "profile_output_policy": profile.output_policy,
        "profile_correction_mode": profile.correction_mode,
        "profile_disparity_gain": profile.disparity_gain,
        "profile_screen_u_gain": profile.screen_u_gain,
        "profile_screen_v_gain": profile.screen_v_gain,
    }
    summary.update(_summary(raw_range_errors, "raw_range_error"))
    summary.update(_summary(raw_position_errors, "raw_position_error"))
    summary.update(_summary(corrected_range_errors, "origin_bbox_reprojection_range_error"))
    summary.update(_summary(corrected_position_errors, "origin_bbox_reprojection_position_error"))
    for axis_name in ("right", "down", "forward"):
        summary.update(_summary(raw_axis_errors[axis_name], f"raw_{axis_name}_error"))
        summary.update(_summary(corrected_axis_errors[axis_name], f"origin_bbox_reprojection_{axis_name}_error"))
        raw_axis_rmse = summary.get(f"raw_{axis_name}_error_rmse_m")
        corrected_axis_rmse = summary.get(f"origin_bbox_reprojection_{axis_name}_error_rmse_m")
        if isinstance(raw_axis_rmse, float) and isinstance(corrected_axis_rmse, float):
            summary[f"origin_bbox_reprojection_{axis_name}_rmse_delta_m"] = corrected_axis_rmse - raw_axis_rmse
            summary[f"origin_bbox_reprojection_{axis_name}_improvement_ratio"] = (
                (raw_axis_rmse - corrected_axis_rmse) / raw_axis_rmse if raw_axis_rmse else 0.0
            )
    raw_range_rmse = summary.get("raw_range_error_rmse_m")
    corrected_range_rmse = summary.get("origin_bbox_reprojection_range_error_rmse_m")
    raw_position_rmse = summary.get("raw_position_error_rmse_m")
    corrected_position_rmse = summary.get("origin_bbox_reprojection_position_error_rmse_m")
    if isinstance(raw_range_rmse, float) and isinstance(corrected_range_rmse, float):
        summary["origin_bbox_reprojection_range_rmse_delta_m"] = corrected_range_rmse - raw_range_rmse
        summary["origin_bbox_reprojection_range_improvement_ratio"] = (
            (raw_range_rmse - corrected_range_rmse) / raw_range_rmse if raw_range_rmse else 0.0
        )
    if isinstance(raw_position_rmse, float) and isinstance(corrected_position_rmse, float):
        summary["origin_bbox_reprojection_position_rmse_delta_m"] = corrected_position_rmse - raw_position_rmse
        summary["origin_bbox_reprojection_position_improvement_ratio"] = (
            (raw_position_rmse - corrected_position_rmse) / raw_position_rmse if raw_position_rmse else 0.0
        )
    for fallback_reason, count in sorted(fallback_counts.items()):
        summary[f"origin_bbox_reprojection_fallback_{fallback_reason}_count"] = float(count)
    return evaluated, summary


def _rmse(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(statistics.fmean(value * value for value in values))


def _disable_origin_bbox_axes(cells: dict[str, dict[str, object]], disabled_axes: Sequence[str]) -> None:
    disabled = {str(axis).strip().lower() for axis in disabled_axes if str(axis).strip()}
    if not disabled:
        return
    axis_indexes = {"right": 0, "down": 1, "forward": 2}
    unknown = sorted(disabled - set(axis_indexes))
    if unknown:
        raise ValueError(f"unknown Origin-BBOX axis name(s): {', '.join(unknown)}")
    for cell in cells.values():
        gains = list(cell.get("axis_gains", [0.0, 0.0, 0.0]))
        while len(gains) < 3:
            gains.append(0.0)
        axis_models = dict(cell.get("axis_models", {}))
        for axis_name in disabled:
            gains[axis_indexes[axis_name]] = 0.0
            axis_models.pop(axis_name, None)
        cell["axis_gains"] = gains[:3]
        cell["axis_models"] = axis_models
        existing_disabled = {
            str(axis).strip().lower()
            for axis in cell.get("axis_disabled", [])
            if str(axis).strip()
        }
        cell["axis_disabled"] = sorted(existing_disabled | disabled)


def _apply_origin_bbox_axis_validation_gates(
    *,
    cells: dict[str, dict[str, object]],
    training_rows: Sequence[dict[str, object]],
    validation_rows: Sequence[dict[str, object]],
    image_width_px: float,
    image_height_px: float,
    u_bins: tuple[float, ...],
    v_bins: tuple[float, ...],
    range_bins_m: tuple[float, ...],
    gain_candidates: Sequence[float],
    axis_model_kinds: Sequence[str],
    min_cell_samples: int,
    min_improvement_m: float,
    min_improvement_ratio: float,
) -> None:
    validation_profile = OriginBboxProfile(
        profile_id="origin_bbox_axis_validation",
        image_width_px=image_width_px,
        image_height_px=image_height_px,
        u_bins=u_bins,
        v_bins=v_bins,
        range_bins_m=range_bins_m,
        cells=cells,
        global_residual_m=(0.0, 0.0, 0.0),
        global_range_residual_m=0.0,
        sample_count=0,
        min_cell_samples=min_cell_samples,
        residual_gain=1.0,
        min_correction_norm_m=0.0,
        min_range_correction_m=0.0,
        training_range_min_m=0.0,
        training_range_max_m=0.0,
        created_unix_s=time.time(),
    )
    training_rows_by_cell: dict[str, list[dict[str, object]]] = {key: [] for key in cells}
    rows_by_cell: dict[str, list[dict[str, object]]] = {key: [] for key in cells}
    for row in training_rows:
        features = validation_profile._features_from_row(row)
        if features is None:
            continue
        u_norm, v_norm, range_m = features
        range_bin = _bin_index(range_m, range_bins_m)
        u_bin = _bin_index(u_norm, u_bins)
        v_bin = _bin_index(v_norm, v_bins)
        if None in (range_bin, u_bin, v_bin):
            continue
        key = validation_profile._cell_key(int(range_bin), int(u_bin), int(v_bin))
        if key in training_rows_by_cell:
            training_rows_by_cell[key].append(row)
    for row in validation_rows:
        features = validation_profile._features_from_row(row)
        if features is None:
            continue
        u_norm, v_norm, range_m = features
        range_bin = _bin_index(range_m, range_bins_m)
        u_bin = _bin_index(u_norm, u_bins)
        v_bin = _bin_index(v_norm, v_bins)
        if None in (range_bin, u_bin, v_bin):
            continue
        key = validation_profile._cell_key(int(range_bin), int(u_bin), int(v_bin))
        if key in rows_by_cell:
            rows_by_cell[key].append(row)

    gains = tuple(float(value) for value in gain_candidates)
    axis_fields = (
        ("right", 0, "raw_error_right_m"),
        ("down", 1, "raw_error_down_m"),
        ("forward", 2, "raw_error_forward_m"),
    )
    for key, cell in cells.items():
        rows = rows_by_cell.get(key, [])
        cell_axis_gains = [0.0, 0.0, 0.0]
        cell_axis_models: dict[str, dict[str, object]] = {}
        validation_stats: dict[str, object] = {
            "sample_count": len(rows),
            "min_cell_samples": min_cell_samples,
        }
        residual = tuple(float(value) for value in cell.get("residual_m", (0.0, 0.0, 0.0)))
        if len(rows) >= min_cell_samples:
            for axis_name, axis_index, error_field in axis_fields:
                raw_errors = [
                    float(value)
                    for row in rows
                    for value in (_first_float(row, (error_field,)),)
                    if value is not None
                ]
                raw_rmse = _rmse(raw_errors)
                best_gain = 0.0
                best_rmse = raw_rmse
                best_model: dict[str, object] | None = None
                for gain in gains:
                    corrected_errors = [error + residual[axis_index] * gain for error in raw_errors]
                    corrected_rmse = _rmse(corrected_errors)
                    if corrected_rmse < best_rmse:
                        best_rmse = corrected_rmse
                        best_gain = gain
                        best_model = None
                for feature_kind in axis_model_kinds:
                    for ridge_lambda in DEFAULT_ORIGIN_BBOX_AXIS_MODEL_RIDGE_LAMBDAS:
                        model = _fit_origin_bbox_axis_model(
                            training_rows_by_cell.get(key, []),
                            axis_name=axis_name,
                            feature_kind=feature_kind,
                            ridge_lambda=ridge_lambda,
                            image_width_px=image_width_px,
                            image_height_px=image_height_px,
                        )
                        if model is None:
                            continue
                        model_corrections = [
                            _predict_origin_bbox_axis_model(
                                row,
                                model,
                                image_width_px=image_width_px,
                                image_height_px=image_height_px,
                            )
                            for row in rows
                        ]
                        if any(value is None for value in model_corrections):
                            continue
                        corrected_errors = [
                            error + float(correction)
                            for error, correction in zip(raw_errors, model_corrections)
                        ]
                        corrected_rmse = _rmse(corrected_errors)
                        if corrected_rmse < best_rmse:
                            best_rmse = corrected_rmse
                            best_gain = 0.0
                            best_model = model
                improvement = raw_rmse - best_rmse
                required = max(min_improvement_m, raw_rmse * min_improvement_ratio)
                if improvement >= required:
                    if best_model is not None:
                        cell_axis_models[axis_name] = best_model
                    else:
                        cell_axis_gains[axis_index] = best_gain
                validation_stats[axis_name] = {
                    "raw_rmse_m": raw_rmse,
                    "selected_rmse_m": best_rmse,
                    "selected_gain": best_gain,
                    "selected_model": None if best_model is None else {
                        "feature_kind": best_model.get("feature_kind"),
                        "ridge_lambda": best_model.get("ridge_lambda"),
                    },
                    "applied_gain": cell_axis_gains[axis_index],
                    "applied_model": axis_name in cell_axis_models,
                    "improvement_m": improvement,
                    "required_improvement_m": required,
                }
        cell["axis_gains"] = cell_axis_gains
        cell["axis_models"] = cell_axis_models
        cell["axis_validation"] = validation_stats


def evaluate_profile_rows(
    rows: Sequence[dict[str, object]],
    profile: UamAlignmentProfile,
) -> tuple[list[dict[str, object]], dict[str, float | str]]:
    evaluated: list[dict[str, object]] = []
    range_errors: list[float] = []
    position_errors: list[float] = []
    raw_range_errors: list[float] = []
    raw_position_errors: list[float] = []
    corrected_range_errors: list[float] = []
    corrected_position_errors: list[float] = []
    for row in rows:
        measured = (
            float(row["measurement_right_m"]),
            float(row["measurement_down_m"]),
            float(row["measurement_forward_m"]),
        )
        truth = (
            float(row["truth_right_m"]),
            float(row["truth_down_m"]),
            float(row["truth_forward_m"]),
        )
        corrected = profile.apply(measured)
        corrected_frd = (corrected[2], corrected[0], corrected[1])
        truth_frd = (truth[2], truth[0], truth[1])
        origin_range_m = _norm3(_sub3(truth, profile.origin_offset_m))
        yolo_range_m = float(row["yolo_range_m"])
        range_error_m = yolo_range_m - origin_range_m
        position_error_m = _norm3(_sub3(corrected, truth))
        frd_error = _sub3(corrected_frd, truth_frd)
        range_errors.append(range_error_m)
        position_errors.append(position_error_m)
        raw_range_error_m = _first_float(row, ("raw_range_error_m",))
        raw_position_error_m = _first_float(row, ("raw_position_error_m",))
        corrected_range_error_m = _first_float(row, ("corrected_range_error_m",))
        corrected_position_error_m = _first_float(row, ("corrected_position_error_m",))
        if raw_range_error_m is not None:
            raw_range_errors.append(raw_range_error_m)
        if raw_position_error_m is not None:
            raw_position_errors.append(raw_position_error_m)
        if corrected_range_error_m is not None:
            corrected_range_errors.append(corrected_range_error_m)
        if corrected_position_error_m is not None:
            corrected_position_errors.append(corrected_position_error_m)
        output = dict(row)
        output.update(
            {
                "profile_id": profile.profile_id,
                "profile_right_m": f"{corrected[0]:.6f}",
                "profile_down_m": f"{corrected[1]:.6f}",
                "profile_forward_m": f"{corrected[2]:.6f}",
                "profile_frd_forward_m": f"{corrected_frd[0]:.6f}",
                "profile_frd_right_m": f"{corrected_frd[1]:.6f}",
                "profile_frd_down_m": f"{corrected_frd[2]:.6f}",
                "error_frd_forward_m": f"{frd_error[0]:.6f}",
                "error_frd_right_m": f"{frd_error[1]:.6f}",
                "error_frd_down_m": f"{frd_error[2]:.6f}",
                "origin_range_m": f"{origin_range_m:.6f}",
                "range_error_m": f"{range_error_m:.6f}",
                "position_error_m": f"{position_error_m:.6f}",
            }
        )
        evaluated.append(output)
    summary: dict[str, float | str] = {
        "profile_id": profile.profile_id,
        "sample_count": float(len(evaluated)),
        "bbox_sample_count": float(sum(1 for row in evaluated if row.get("bbox_available") == "true")),
        "bbox_corrected_sample_count": float(
            sum(1 for row in evaluated if row.get("bbox_corrected_available") == "true")
        ),
    }
    summary.update(_summary(raw_range_errors, "raw_range_error"))
    summary.update(_summary(raw_position_errors, "raw_position_error"))
    summary.update(_summary(corrected_range_errors, "corrected_range_error"))
    summary.update(_summary(corrected_position_errors, "corrected_position_error"))
    summary.update(_summary(range_errors, "range_error"))
    summary.update(_summary(position_errors, "position_error"))
    return evaluated, summary


def evaluate_origin_bbox_profile_rows(
    rows: Sequence[dict[str, object]],
    profile: OriginBboxProfile,
) -> tuple[list[dict[str, object]], dict[str, float | str]]:
    evaluated: list[dict[str, object]] = []
    raw_range_errors: list[float] = []
    raw_position_errors: list[float] = []
    origin_bbox_range_errors: list[float] = []
    origin_bbox_position_errors: list[float] = []
    raw_axis_errors: dict[str, list[float]] = {"right": [], "down": [], "forward": []}
    origin_bbox_axis_errors: dict[str, list[float]] = {"right": [], "down": [], "forward": []}
    fallback_counts: dict[str, int] = {}
    for row in rows:
        truth = (
            _first_float(row, ("truth_right_m",)),
            _first_float(row, ("truth_down_m",)),
            _first_float(row, ("truth_forward_m",)),
        )
        if not _tuple_complete(truth):
            continue
        prediction = profile.predict_row(row)
        if prediction is None:
            continue
        corrected = prediction.corrected_m
        truth_point = (float(truth[0]), float(truth[1]), float(truth[2]))
        corrected_frd = (corrected[2], corrected[0], corrected[1])
        truth_frd = (truth_point[2], truth_point[0], truth_point[1])
        error = _sub3(corrected, truth_point)
        error_frd = (error[2], error[0], error[1])
        corrected_range_m = prediction.corrected_range_m
        truth_range_m = float(row["truth_range_m"])
        range_error_m = corrected_range_m - truth_range_m
        position_error_m = _norm3(error)
        raw_range_error_m = _first_float(row, ("raw_range_error_m",))
        raw_position_error_m = _first_float(row, ("raw_position_error_m",))
        if raw_range_error_m is not None:
            raw_range_errors.append(raw_range_error_m)
        if raw_position_error_m is not None:
            raw_position_errors.append(raw_position_error_m)
        for axis_name, axis_index in (("right", 0), ("down", 1), ("forward", 2)):
            raw_axis_error_m = _first_float(row, (f"raw_error_{axis_name}_m",))
            if raw_axis_error_m is not None:
                raw_axis_errors[axis_name].append(raw_axis_error_m)
            origin_bbox_axis_errors[axis_name].append(error[axis_index])
        origin_bbox_range_errors.append(range_error_m)
        origin_bbox_position_errors.append(position_error_m)
        fallback_counts[prediction.fallback_level] = fallback_counts.get(prediction.fallback_level, 0) + 1
        output = dict(row)
        output.update(
            {
                "origin_bbox_profile_id": profile.profile_id,
                "origin_bbox_corrected_right_m": f"{corrected[0]:.6f}",
                "origin_bbox_corrected_down_m": f"{corrected[1]:.6f}",
                "origin_bbox_corrected_forward_m": f"{corrected[2]:.6f}",
                "origin_bbox_corrected_range_m": f"{corrected_range_m:.6f}",
                "origin_bbox_corrected_frd_forward_m": f"{corrected_frd[0]:.6f}",
                "origin_bbox_corrected_frd_right_m": f"{corrected_frd[1]:.6f}",
                "origin_bbox_corrected_frd_down_m": f"{corrected_frd[2]:.6f}",
                "origin_bbox_residual_right_m": f"{prediction.residual_m[0]:.6f}",
                "origin_bbox_residual_down_m": f"{prediction.residual_m[1]:.6f}",
                "origin_bbox_residual_forward_m": f"{prediction.residual_m[2]:.6f}",
                "origin_bbox_range_residual_m": f"{prediction.range_residual_m:.6f}",
                "origin_bbox_error_right_m": f"{error[0]:.6f}",
                "origin_bbox_error_down_m": f"{error[1]:.6f}",
                "origin_bbox_error_forward_m": f"{error[2]:.6f}",
                "origin_bbox_error_frd_forward_m": f"{error_frd[0]:.6f}",
                "origin_bbox_error_frd_right_m": f"{error_frd[1]:.6f}",
                "origin_bbox_error_frd_down_m": f"{error_frd[2]:.6f}",
                "origin_bbox_range_error_m": f"{range_error_m:.6f}",
                "origin_bbox_position_error_m": f"{position_error_m:.6f}",
                "origin_bbox_fallback_level": prediction.fallback_level,
                "origin_bbox_cell_key": prediction.cell_key,
                "origin_bbox_cell_sample_count": prediction.sample_count,
                "truth_frd_forward_m": f"{truth_frd[0]:.6f}",
                "truth_frd_right_m": f"{truth_frd[1]:.6f}",
                "truth_frd_down_m": f"{truth_frd[2]:.6f}",
            }
        )
        evaluated.append(output)

    summary: dict[str, float | str] = {
        "profile_id": profile.profile_id,
        "profile_method": profile.method,
        "sample_count": float(len(evaluated)),
        "profile_cell_count": float(len(profile.cells)),
        "profile_sample_count": float(profile.sample_count),
        "profile_residual_gain": profile.residual_gain,
        "profile_min_correction_norm_m": profile.min_correction_norm_m,
        "profile_min_range_correction_m": profile.min_range_correction_m,
        "profile_training_range_min_m": profile.training_range_min_m,
        "profile_training_range_max_m": profile.training_range_max_m,
    }
    summary.update(_summary(raw_range_errors, "raw_range_error"))
    summary.update(_summary(raw_position_errors, "raw_position_error"))
    summary.update(_summary(origin_bbox_range_errors, "origin_bbox_range_error"))
    summary.update(_summary(origin_bbox_position_errors, "origin_bbox_position_error"))
    for axis_name in ("right", "down", "forward"):
        summary.update(_summary(raw_axis_errors[axis_name], f"raw_{axis_name}_error"))
        summary.update(_summary(origin_bbox_axis_errors[axis_name], f"origin_bbox_{axis_name}_error"))
        raw_axis_rmse = summary.get(f"raw_{axis_name}_error_rmse_m")
        corrected_axis_rmse = summary.get(f"origin_bbox_{axis_name}_error_rmse_m")
        if isinstance(raw_axis_rmse, float) and isinstance(corrected_axis_rmse, float):
            summary[f"origin_bbox_{axis_name}_rmse_delta_m"] = corrected_axis_rmse - raw_axis_rmse
            summary[f"origin_bbox_{axis_name}_improvement_ratio"] = (
                (raw_axis_rmse - corrected_axis_rmse) / raw_axis_rmse if raw_axis_rmse else 0.0
            )
    raw_range_rmse = summary.get("raw_range_error_rmse_m")
    corrected_range_rmse = summary.get("origin_bbox_range_error_rmse_m")
    raw_position_rmse = summary.get("raw_position_error_rmse_m")
    corrected_position_rmse = summary.get("origin_bbox_position_error_rmse_m")
    if isinstance(raw_range_rmse, float) and isinstance(corrected_range_rmse, float):
        summary["origin_bbox_range_rmse_delta_m"] = corrected_range_rmse - raw_range_rmse
        summary["origin_bbox_range_improvement_ratio"] = (
            (raw_range_rmse - corrected_range_rmse) / raw_range_rmse if raw_range_rmse else 0.0
        )
    if isinstance(raw_position_rmse, float) and isinstance(corrected_position_rmse, float):
        summary["origin_bbox_position_rmse_delta_m"] = corrected_position_rmse - raw_position_rmse
        summary["origin_bbox_position_improvement_ratio"] = (
            (raw_position_rmse - corrected_position_rmse) / raw_position_rmse if raw_position_rmse else 0.0
        )
    for fallback_level, count in sorted(fallback_counts.items()):
        summary[f"origin_bbox_fallback_{fallback_level}_count"] = float(count)
    return evaluated, summary


def write_dynamic_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = list(dict.fromkeys(field for row in rows for field in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_profile(path: Path) -> UamAlignmentProfile:
    return UamAlignmentProfile.from_dict(json.loads(path.read_text(encoding="utf-8")))


def load_origin_bbox_profile(path: Path) -> OriginBboxProfile:
    return OriginBboxProfile.from_dict(json.loads(path.read_text(encoding="utf-8")))


def load_origin_bbox_reprojection_profile(path: Path) -> OriginBBoxReprojectionProfile:
    return load_reprojection_profile(path)


def detect_profile_kind(path: Path) -> str:
    payload = json.loads(path.read_text(encoding="utf-8"))
    method = str(payload.get("method", ""))
    if method == ORIGIN_BBOX_REPROJECTION_METHOD:
        return "origin-bbox-reprojection"
    if method == ORIGIN_BBOX_PROFILE_METHOD:
        return "origin-bbox"
    return "alignment"


def save_profile(path: Path, profile: UamAlignmentProfile) -> None:
    write_json(path, profile.to_dict())


def aggregate_summary_files(paths: Sequence[Path]) -> dict[str, object]:
    rows = [json.loads(path.read_text(encoding="utf-8")) for path in paths]
    summary: dict[str, object] = {
        "created_unix_s": time.time(),
        "run_count": len(rows),
        "runs": rows,
    }
    for key in (
        "raw_range_error_rmse_m",
        "raw_position_error_rmse_m",
        "corrected_range_error_rmse_m",
        "corrected_position_error_rmse_m",
        "range_error_rmse_m",
        "position_error_rmse_m",
        "origin_bbox_range_error_rmse_m",
        "origin_bbox_position_error_rmse_m",
        "raw_right_error_rmse_m",
        "raw_down_error_rmse_m",
        "raw_forward_error_rmse_m",
        "origin_bbox_right_error_rmse_m",
        "origin_bbox_down_error_rmse_m",
        "origin_bbox_forward_error_rmse_m",
        "origin_bbox_reprojection_range_error_rmse_m",
        "origin_bbox_reprojection_position_error_rmse_m",
        "origin_bbox_reprojection_right_error_rmse_m",
        "origin_bbox_reprojection_down_error_rmse_m",
        "origin_bbox_reprojection_forward_error_rmse_m",
    ):
        values = [float(row[key]) for row in rows if row.get(key) is not None]
        prefix = key[:-2]
        summary[f"{prefix}_mean_m"] = statistics.fmean(values) if values else None
        summary[f"{prefix}_max_m"] = max(values) if values else None
    for key in (
        "origin_bbox_range_improvement_ratio",
        "origin_bbox_position_improvement_ratio",
        "origin_bbox_right_improvement_ratio",
        "origin_bbox_down_improvement_ratio",
        "origin_bbox_forward_improvement_ratio",
        "origin_bbox_reprojection_range_improvement_ratio",
        "origin_bbox_reprojection_position_improvement_ratio",
        "origin_bbox_reprojection_right_improvement_ratio",
        "origin_bbox_reprojection_down_improvement_ratio",
        "origin_bbox_reprojection_forward_improvement_ratio",
    ):
        values = [float(row[key]) for row in rows if row.get(key) is not None]
        summary[f"{key}_mean"] = statistics.fmean(values) if values else None
        summary[f"{key}_min"] = min(values) if values else None
    return summary
