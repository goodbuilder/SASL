#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Mapping, Protocol, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_from_catalog import method_contract_fields  # noqa: E402


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
NED_AXIS_CONTRACT = "px4_local_position_ned:north,east,down"
SYNTHETIC_NED_TO_RDF_CONTRACT = (
    "explicit_transform: truth_rdf = R_ned_to_stereo_rdf * (truth_ned - origin_ned); "
    "direct axis alias rdf_right=east/rdf_down=down/rdf_forward=north is forbidden"
)
STEREODISTANCE_PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
DEFAULT_TARGET_RANGE_M = 1852.0
SYNTHETIC_STEREO_ORIGIN_NED_M = np.array([120.0, -35.0, 8.0], dtype=float)
SYNTHETIC_NED_TO_STEREO_RDF_ROTATION = Rotation.from_euler("zyx", [17.0, -4.0, 2.5], degrees=True)


def ned_to_stereo_rdf_explicit(ned_m: Sequence[float]) -> np.ndarray:
    """Convert PX4 local NED to stereo-center RDF with an explicit extrinsic."""
    ned = np.asarray(ned_m, dtype=float)
    return SYNTHETIC_NED_TO_STEREO_RDF_ROTATION.apply(ned - SYNTHETIC_STEREO_ORIGIN_NED_M)


def stereo_rdf_to_ned_explicit(rdf_m: Sequence[float]) -> np.ndarray:
    """Inverse of the explicit synthetic stereo-center RDF extrinsic."""
    rdf = np.asarray(rdf_m, dtype=float)
    return SYNTHETIC_NED_TO_STEREO_RDF_ROTATION.inv().apply(rdf) + SYNTHETIC_STEREO_ORIGIN_NED_M


@dataclass(frozen=True)
class CameraModel:
    width_px: int = 1280
    height_px: int = 720
    horizontal_fov_rad: float = 0.154997710698
    baseline_m: float = 25.0
    baseline_scale_error: float = 0.006
    focal_scale_error: float = -0.004

    @property
    def fx_true_px(self) -> float:
        return (self.width_px * 0.5) / math.tan(self.horizontal_fov_rad * 0.5)

    @property
    def fy_true_px(self) -> float:
        return self.fx_true_px

    @property
    def cx_px(self) -> float:
        return self.width_px * 0.5

    @property
    def cy_px(self) -> float:
        return self.height_px * 0.5

    @property
    def fx_meas_px(self) -> float:
        return self.fx_true_px * (1.0 + self.focal_scale_error)

    @property
    def fy_meas_px(self) -> float:
        return self.fx_meas_px

    @property
    def baseline_meas_m(self) -> float:
        return self.baseline_m * (1.0 + self.baseline_scale_error)


@dataclass(frozen=True)
class ErrorInjection:
    yaw_deg: float = 0.35
    pitch_deg: float = -0.18
    roll_deg: float = 0.08
    disparity_bias_px: float = 0.35
    disparity_noise_px: float = 0.07
    bbox_noise_px: float = 0.18
    disparity_outlier_rate: float = 0.0
    disparity_outlier_px: float = 0.0
    bbox_outlier_rate: float = 0.0
    bbox_outlier_px: float = 0.0
    down_bias_px: float = 0.12

    @property
    def raw_rotation(self) -> Rotation:
        # scipy uses axis order names; this script stores RDF points as row
        # vectors and applies the matrix transpose in apply_rotation_to_rows().
        return Rotation.from_euler(
            "zyx",
            [self.yaw_deg, self.pitch_deg, self.roll_deg],
            degrees=True,
        )


@dataclass(frozen=True)
class CalibrationSample:
    sample_id: int
    t_s: float
    split: str
    truth_ned_m: np.ndarray
    truth_rdf_m: np.ndarray
    raw_rdf_m: np.ndarray
    left_u_px: float
    left_v_px: float
    measured_disparity_px: float
    true_disparity_px: float
    truth_source_frame: str = ""
    truth_source_axis_order: str = ""
    truth_source_fields: str = ""
    truth_rdf_transform_contract: str = ""
    truth_source_origin_m: str = ""
    truth_source_to_stereo_rdf_translation_m: str = ""
    truth_source_to_stereo_rdf_rotation_matrix: str = ""

    @property
    def truth_range_m(self) -> float:
        return float(np.linalg.norm(self.truth_rdf_m))

    @property
    def raw_range_m(self) -> float:
        return float(np.linalg.norm(self.raw_rdf_m))


class CalibrationDataSource(Protocol):
    name: str

    def load_samples(self) -> list[CalibrationSample]:
        ...


class SyntheticCalibrationDataSource:
    name = "synthetic"

    def __init__(
        self,
        *,
        camera: CameraModel,
        injection: ErrorInjection,
        sample_count: int,
        seed: int,
        target_range_m: float,
        target_window_m: float = 30.0,
        target_oversample_count: int = 12,
    ) -> None:
        self.camera = camera
        self.injection = injection
        self.sample_count = sample_count
        self.seed = seed
        self.target_range_m = target_range_m
        self.target_window_m = target_window_m
        self.target_oversample_count = max(0, int(target_oversample_count))

    def load_samples(self) -> list[CalibrationSample]:
        trajectory_rng = np.random.default_rng(self.seed)
        measurement_rng = np.random.default_rng(self.seed + 1000003)
        camera = self.camera
        samples: list[CalibrationSample] = []
        base_forward = np.linspace(240.0, 2050.0, self.sample_count)
        if self.target_oversample_count > 0 and self.sample_count > 0:
            target_count = min(self.sample_count, self.target_oversample_count)
            center_index = int(round((self.target_range_m - 240.0) / max(1.0e-9, 2050.0 - 240.0) * (self.sample_count - 1)))
            half_count = target_count // 2
            start = min(max(0, center_index - half_count), max(0, self.sample_count - target_count))
            target_indices = np.arange(start, start + target_count, dtype=int)
            span = min(max(2.0, self.target_window_m * 0.85), max(2.0, self.target_window_m - 1.0))
            offsets = np.linspace(-span, span, target_count) if target_count > 1 else np.array([0.0])
            jitter = trajectory_rng.normal(0.0, max(0.05, self.target_window_m * 0.08), size=target_count)
            target_forward = self.target_range_m + np.clip(offsets + jitter, -max(0.1, self.target_window_m * 0.95), max(0.1, self.target_window_m * 0.95))
            base_forward[target_indices] = target_forward
        for index, forward_m in enumerate(base_forward):
            phase = index / max(1, self.sample_count - 1)
            right_m = 26.0 * math.sin(phase * math.tau * 2.3) + trajectory_rng.normal(0.0, 2.2)
            down_m = 4.0 + 13.0 * math.sin(phase * math.tau * 1.7 + 0.4) + trajectory_rng.normal(0.0, 1.0)
            forward_m = float(forward_m + trajectory_rng.normal(0.0, 4.5))
            if abs(forward_m - self.target_range_m) < 12.0:
                forward_m = self.target_range_m + trajectory_rng.normal(0.0, 2.0)
            truth_rdf = np.array([right_m, down_m, forward_m], dtype=float)

            # Synthetic truth still records PX4 local NED, but the NED/RDF
            # relationship is an explicit non-aligned extrinsic, not an axis alias.
            truth_ned = stereo_rdf_to_ned_explicit(truth_rdf)

            true_disparity = camera.fx_true_px * camera.baseline_m / truth_rdf[2]
            left_u_true = camera.cx_px + (truth_rdf[0] + camera.baseline_m * 0.5) * camera.fx_true_px / truth_rdf[2]
            left_v_true = camera.cy_px + truth_rdf[1] * camera.fy_true_px / truth_rdf[2]

            measured_disparity = (
                true_disparity
                + self.injection.disparity_bias_px
                + 0.00018 * (truth_rdf[2] - 1100.0)
                + measurement_rng.normal(0.0, self.injection.disparity_noise_px)
            )
            if measurement_rng.random() < max(0.0, self.injection.disparity_outlier_rate):
                measured_disparity += measurement_rng.normal(0.0, max(0.0, self.injection.disparity_outlier_px))
            measured_disparity = max(float(measured_disparity), 1.0e-6)
            left_u_meas = left_u_true + measurement_rng.normal(0.0, self.injection.bbox_noise_px)
            left_v_meas = (
                left_v_true
                + self.injection.down_bias_px
                + measurement_rng.normal(0.0, self.injection.bbox_noise_px)
            )
            if measurement_rng.random() < max(0.0, self.injection.bbox_outlier_rate):
                left_u_meas += measurement_rng.normal(0.0, max(0.0, self.injection.bbox_outlier_px))
                left_v_meas += measurement_rng.normal(0.0, max(0.0, self.injection.bbox_outlier_px))
            raw_rdf = triangulate_left_disparity(
                left_u_px=left_u_meas,
                left_v_px=left_v_meas,
                disparity_px=measured_disparity,
                fx_px=camera.fx_meas_px,
                fy_px=camera.fy_meas_px,
                cx_px=camera.cx_px,
                cy_px=camera.cy_px,
                baseline_m=camera.baseline_meas_m,
            )
            raw_rdf = apply_rotation_to_rows(raw_rdf[None, :], self.injection.raw_rotation)[0]

            samples.append(
                CalibrationSample(
                    sample_id=index,
                    t_s=index / 5.0,
                    split="",
                    truth_ned_m=truth_ned,
                    truth_rdf_m=truth_rdf,
                    raw_rdf_m=raw_rdf,
                    left_u_px=float(left_u_meas),
                    left_v_px=float(left_v_meas),
                    measured_disparity_px=float(measured_disparity),
                    true_disparity_px=float(true_disparity),
                    truth_source_frame="px4_local_position_ned",
                    truth_source_axis_order="north,east,down",
                    truth_source_fields="truth_north_m,truth_east_m,truth_down_ned_m",
                    truth_rdf_transform_contract=SYNTHETIC_NED_TO_RDF_CONTRACT,
                    truth_source_origin_m=",".join(f"{value:.9f}" for value in SYNTHETIC_STEREO_ORIGIN_NED_M),
                    truth_source_to_stereo_rdf_translation_m="0.000000000,0.000000000,0.000000000",
                    truth_source_to_stereo_rdf_rotation_matrix=json.dumps(
                        SYNTHETIC_NED_TO_STEREO_RDF_ROTATION.as_matrix().tolist(),
                        separators=(",", ":"),
                    ),
                )
            )
        return samples


class CsvCalibrationDataSource:
    name = "csv"

    def __init__(self, path: Path, *, camera: CameraModel) -> None:
        self.path = path
        self.camera = camera

    def load_samples(self) -> list[CalibrationSample]:
        samples: list[CalibrationSample] = []
        with self.path.open(newline="", encoding="utf-8") as handle:
            for row_index, row in enumerate(csv.DictReader(handle)):
                truth_rdf = first_vector(
                    row,
                    (
                        ("truth_right_m", "truth_down_m", "truth_forward_m"),
                        ("drone_right_m", "drone_down_m", "drone_forward_m"),
                        ("gt_right_m", "gt_down_m", "gt_forward_m"),
                    ),
                )
                truth_ned = first_vector(
                    row,
                    (
                        ("truth_north_m", "truth_east_m", "truth_down_ned_m"),
                        ("north_m", "east_m", "down_m"),
                    ),
                )
                raw_rdf = first_vector(
                    row,
                    (
                        ("raw_right_m", "raw_down_m", "raw_forward_m"),
                        ("estimated_right_m", "estimated_down_m", "estimated_forward_m"),
                    ),
                )
                left_u = finite_float(row.get("left_u_px") or row.get("left_cx_px") or row.get("left_cx"))
                left_v = finite_float(row.get("left_v_px") or row.get("left_cy_px") or row.get("left_cy"))
                right_u = finite_float(row.get("right_u_px") or row.get("right_cx_px") or row.get("right_cx"))
                d_meas = finite_float(row.get("measured_disparity_px") or row.get("raw_disparity_px"))
                if d_meas is None and left_u is not None and right_u is not None:
                    d_meas = abs(float(left_u) - float(right_u))
                if truth_rdf is None and truth_ned is not None:
                    continue
                if truth_ned is None and truth_rdf is not None:
                    truth_ned = np.array([math.nan, math.nan, math.nan], dtype=float)
                if raw_rdf is None and None not in (left_u, left_v, d_meas):
                    raw_rdf = triangulate_left_disparity(
                        left_u_px=float(left_u),
                        left_v_px=float(left_v),
                        disparity_px=float(d_meas),
                        fx_px=self.camera.fx_meas_px,
                        fy_px=self.camera.fy_meas_px,
                        cx_px=self.camera.cx_px,
                        cy_px=self.camera.cy_px,
                        baseline_m=self.camera.baseline_meas_m,
                    )
                if truth_rdf is None or truth_ned is None or raw_rdf is None:
                    continue
                true_disparity = self.camera.fx_true_px * self.camera.baseline_m / truth_rdf[2]
                if d_meas is None:
                    d_meas = self.camera.fx_meas_px * self.camera.baseline_meas_m / raw_rdf[2]
                if left_u is None:
                    left_u = self.camera.cx_px + (raw_rdf[0] + self.camera.baseline_meas_m * 0.5) * self.camera.fx_meas_px / raw_rdf[2]
                if left_v is None:
                    left_v = self.camera.cy_px + raw_rdf[1] * self.camera.fy_meas_px / raw_rdf[2]
                samples.append(
                    CalibrationSample(
                        sample_id=row_index,
                        t_s=finite_float(row.get("t_s")) or finite_float(row.get("trial_t_s")) or float(row_index),
                        split=row.get("split", ""),
                        truth_ned_m=truth_ned,
                        truth_rdf_m=truth_rdf,
                        raw_rdf_m=raw_rdf,
                        left_u_px=float(left_u),
                        left_v_px=float(left_v),
                        measured_disparity_px=float(d_meas),
                        true_disparity_px=float(true_disparity),
                        truth_source_frame=str(row.get("truth_source_frame") or row.get("source_frame") or ""),
                        truth_source_axis_order=str(row.get("truth_source_axis_order") or row.get("source_axis_order") or ""),
                        truth_source_fields=str(row.get("truth_source_fields") or row.get("source_fields") or ""),
                        truth_rdf_transform_contract=str(
                            row.get("truth_rdf_transform_contract")
                            or row.get("truth_rdf_contract")
                            or "already_stereo_center_RDF_truth_or_external_transform_declared"
                        ),
                        truth_source_origin_m=str(row.get("truth_source_origin_m") or row.get("origin_source_m") or ""),
                        truth_source_to_stereo_rdf_translation_m=str(
                            row.get("truth_source_to_stereo_rdf_translation_m")
                            or row.get("translation_rdf_m")
                            or ""
                        ),
                        truth_source_to_stereo_rdf_rotation_matrix=str(
                            row.get("truth_source_to_stereo_rdf_rotation_matrix")
                            or row.get("rotation_matrix_source_to_stereo_rdf")
                            or ""
                        ),
                    )
                )
        if not samples:
            raise ValueError(f"no usable calibration rows loaded from {self.path}")
        return samples


class Ros2TopicCalibrationDataSource:
    name = "ros2_topic"

    def load_samples(self) -> list[CalibrationSample]:
        raise NotImplementedError(
            "ROS2 topic collection is intentionally separated from fitting. "
            "Record /fmu/out/vehicle_local_position, camera image, and YOLO BBox "
            "topics to CSV with explicit NED/RDF fields, then run this pipeline "
            "with --input-csv. Synthetic mode remains the fallback."
        )


@dataclass(frozen=True)
class FittedMethod:
    name: str
    predict: Callable[[Sequence[CalibrationSample]], np.ndarray]
    parameters: dict[str, object]
    model_complexity: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare RTK/PX4 NED calibrated stereo RDF correction methods. "
            "Synthetic mode runs without Gazebo/PX4 topics."
        )
    )
    parser.add_argument("--mode", choices=("synthetic", "csv", "ros2"), default="synthetic")
    parser.add_argument("--input-csv", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--sample-count", type=int, default=420)
    parser.add_argument("--seed", type=int, default=20260613)
    parser.add_argument("--train-ratio", type=float, default=0.80)
    parser.add_argument("--target-range-m", type=float, default=DEFAULT_TARGET_RANGE_M)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--synthetic-target-oversample-count", type=int, default=12)
    parser.add_argument("--baseline-m", type=float, default=25.0)
    parser.add_argument("--baseline-scale-error", type=float, default=0.006)
    parser.add_argument("--focal-scale-error", type=float, default=-0.004)
    parser.add_argument("--yaw-error-deg", type=float, default=0.35)
    parser.add_argument("--pitch-error-deg", type=float, default=-0.18)
    parser.add_argument("--roll-error-deg", type=float, default=0.08)
    parser.add_argument("--disparity-bias-px", type=float, default=0.35)
    parser.add_argument("--disparity-noise-px", type=float, default=0.07)
    parser.add_argument("--bbox-noise-px", type=float, default=0.18)
    parser.add_argument("--disparity-outlier-rate", type=float, default=0.0)
    parser.add_argument("--disparity-outlier-px", type=float, default=0.0)
    parser.add_argument("--bbox-outlier-rate", type=float, default=0.0)
    parser.add_argument("--bbox-outlier-px", type=float, default=0.0)
    parser.add_argument(
        "--local-sweep-config-dir",
        type=Path,
        help=(
            "Directory containing rtk_stereo_local_sweep_candidate JSON files. "
            "Supported configs are appended to the normal method search. "
            "Offline fitting may use truth; exported runtime profiles must remain raw RDF/disparity only."
        ),
    )
    return parser.parse_args()


def finite_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def first_vector(
    row: dict[str, str],
    alternatives: Sequence[tuple[str, str, str]],
) -> np.ndarray | None:
    for names in alternatives:
        values = [finite_float(row.get(name)) for name in names]
        if all(value is not None for value in values):
            return np.array(values, dtype=float)
    return None


def triangulate_left_disparity(
    *,
    left_u_px: float,
    left_v_px: float,
    disparity_px: float,
    fx_px: float,
    fy_px: float,
    cx_px: float,
    cy_px: float,
    baseline_m: float,
) -> np.ndarray:
    if disparity_px <= 1.0e-9:
        return np.array([math.nan, math.nan, math.nan], dtype=float)
    forward_m = fx_px * baseline_m / disparity_px
    right_m = (left_u_px - cx_px) * forward_m / fx_px - baseline_m * 0.5
    down_m = (left_v_px - cy_px) * forward_m / fy_px
    return np.array([right_m, down_m, forward_m], dtype=float)


def apply_rotation_to_rows(points: np.ndarray, rotation: Rotation) -> np.ndarray:
    return np.asarray(points, dtype=float) @ rotation.as_matrix().T


def split_samples(
    samples: Sequence[CalibrationSample],
    *,
    train_ratio: float,
    seed: int,
    target_range_m: float | None = None,
    target_window_m: float | None = None,
) -> tuple[list[CalibrationSample], list[CalibrationSample]]:
    train_ratio = min(0.95, max(0.05, train_ratio))
    rng = np.random.default_rng(seed)
    indices = np.arange(len(samples))
    train_count = int(round(len(samples) * train_ratio))
    train_count = max(1, min(len(samples) - 1, train_count)) if len(samples) > 1 else len(samples)
    train_ids: set[int]
    if target_range_m is not None and target_window_m is not None:
        truth_ranges = np.array([sample.truth_range_m for sample in samples], dtype=float)
        target_indices = indices[np.abs(truth_ranges - float(target_range_m)) <= float(target_window_m)]
        non_target_indices = np.array([index for index in indices if index not in set(int(i) for i in target_indices)], dtype=int)
        rng.shuffle(target_indices)
        rng.shuffle(non_target_indices)
        if target_indices.size >= 2:
            target_train_count = int(round(target_indices.size * train_ratio))
            target_train_count = max(1, min(int(target_indices.size) - 1, target_train_count))
        else:
            target_train_count = int(target_indices.size)
        target_train = [int(index) for index in target_indices[:target_train_count]]
        remaining_train_count = max(0, train_count - len(target_train))
        train_ids = set(target_train)
        train_ids.update(int(index) for index in non_target_indices[:remaining_train_count])
        if len(train_ids) < train_count:
            train_ids.update(int(index) for index in target_indices[target_train_count:target_train_count + (train_count - len(train_ids))])
    else:
        rng.shuffle(indices)
        train_ids = set(int(index) for index in indices[:train_count])
    train: list[CalibrationSample] = []
    validation: list[CalibrationSample] = []
    for index, sample in enumerate(samples):
        split = "train" if index in train_ids else "validation"
        updated = CalibrationSample(
            sample_id=sample.sample_id,
            t_s=sample.t_s,
            split=split,
            truth_ned_m=sample.truth_ned_m,
            truth_rdf_m=sample.truth_rdf_m,
            raw_rdf_m=sample.raw_rdf_m,
            left_u_px=sample.left_u_px,
            left_v_px=sample.left_v_px,
            measured_disparity_px=sample.measured_disparity_px,
            true_disparity_px=sample.true_disparity_px,
            truth_source_frame=sample.truth_source_frame,
            truth_source_axis_order=sample.truth_source_axis_order,
            truth_source_fields=sample.truth_source_fields,
            truth_rdf_transform_contract=sample.truth_rdf_transform_contract,
            truth_source_origin_m=sample.truth_source_origin_m,
            truth_source_to_stereo_rdf_translation_m=sample.truth_source_to_stereo_rdf_translation_m,
            truth_source_to_stereo_rdf_rotation_matrix=sample.truth_source_to_stereo_rdf_rotation_matrix,
        )
        if split == "train":
            train.append(updated)
        else:
            validation.append(updated)
    return train, validation


def stack(samples: Sequence[CalibrationSample], field: str) -> np.ndarray:
    return np.vstack([getattr(sample, field) for sample in samples])


def measured_disparity(samples: Sequence[CalibrationSample]) -> np.ndarray:
    return np.array([sample.measured_disparity_px for sample in samples], dtype=float)


def true_disparity(samples: Sequence[CalibrationSample]) -> np.ndarray:
    return np.array([sample.true_disparity_px for sample in samples], dtype=float)


def fit_kabsch_rotation(source: np.ndarray, target: np.ndarray) -> Rotation:
    covariance = source.T @ target
    u_matrix, _, vt_matrix = np.linalg.svd(covariance)
    rotation_matrix = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation_matrix) < 0:
        vt_matrix[-1, :] *= -1.0
        rotation_matrix = vt_matrix.T @ u_matrix.T
    return Rotation.from_matrix(rotation_matrix)


def fit_scaled_rotation(source: np.ndarray, target: np.ndarray) -> tuple[float, Rotation]:
    rotation = fit_kabsch_rotation(source, target)
    rotated = apply_rotation_to_rows(source, rotation)
    denom = float(np.sum(rotated * rotated))
    scale = 1.0 if denom <= 0.0 else float(np.sum(target * rotated) / denom)
    return scale, rotation


def fit_umeyama_sim3(source: np.ndarray, target: np.ndarray) -> tuple[float, Rotation, np.ndarray]:
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must be Nx3 arrays")
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = (target_centered.T @ source_centered) / source.shape[0]
    u_matrix, singular_values, vt_matrix = np.linalg.svd(covariance)
    sign = np.ones(3)
    if np.linalg.det(u_matrix) * np.linalg.det(vt_matrix) < 0:
        sign[-1] = -1.0
    rotation_matrix = u_matrix @ np.diag(sign) @ vt_matrix
    source_variance = float(np.mean(np.sum(source_centered * source_centered, axis=1)))
    scale = 1.0 if source_variance <= 0.0 else float(np.sum(singular_values * sign) / source_variance)
    translation = target_mean - scale * (source_mean @ rotation_matrix.T)
    return scale, Rotation.from_matrix(rotation_matrix), translation


def fit_weighted_umeyama_sim3(
    source: np.ndarray,
    target: np.ndarray,
    weights: np.ndarray,
) -> tuple[float, Rotation, np.ndarray]:
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must be Nx3 arrays")
    weights = np.asarray(weights, dtype=float).reshape(-1)
    if weights.shape[0] != source.shape[0]:
        raise ValueError("weights must match source row count")
    weights = np.where(np.isfinite(weights) & (weights > 0.0), weights, 0.0)
    weight_sum = float(np.sum(weights))
    if weight_sum <= 1.0e-12:
        return fit_umeyama_sim3(source, target)
    weights = weights / weight_sum
    source_mean = np.sum(source * weights[:, None], axis=0)
    target_mean = np.sum(target * weights[:, None], axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = target_centered.T @ (source_centered * weights[:, None])
    u_matrix, singular_values, vt_matrix = np.linalg.svd(covariance)
    sign = np.ones(3)
    if np.linalg.det(u_matrix) * np.linalg.det(vt_matrix) < 0:
        sign[-1] = -1.0
    rotation_matrix = u_matrix @ np.diag(sign) @ vt_matrix
    source_variance = float(np.sum(weights * np.sum(source_centered * source_centered, axis=1)))
    scale = 1.0 if source_variance <= 0.0 else float(np.sum(singular_values * sign) / source_variance)
    translation = target_mean - scale * (source_mean @ rotation_matrix.T)
    return scale, Rotation.from_matrix(rotation_matrix), translation


def predict_sim3(points: np.ndarray, scale: float, rotation: Rotation, translation: np.ndarray) -> np.ndarray:
    return scale * apply_rotation_to_rows(points, rotation) + translation


def sim3_transform_to_json(
    *,
    scale: float,
    rotation: Rotation,
    translation: np.ndarray,
    sample_count: int,
) -> dict[str, object]:
    return {
        "scale": float(scale),
        "rotation_matrix_rdf": rotation.as_matrix().tolist(),
        "translation_rdf_m": np.asarray(translation, dtype=float).tolist(),
        "sample_count": int(sample_count),
    }


def predict_piecewise_sim3_raw_forward(points: np.ndarray, model: Mapping[str, object]) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    x_values = np.asarray(model.get("x_values_m", []), dtype=float)
    transforms = list(model.get("transforms", []))
    if x_values.size == 0 or len(transforms) == 0:
        scale = float(model.get("fallback_scale", 1.0))
        rotation = Rotation.from_matrix(np.asarray(model.get("fallback_rotation_matrix_rdf", np.eye(3)), dtype=float))
        translation = np.asarray(model.get("fallback_translation_rdf_m", [0.0, 0.0, 0.0]), dtype=float)
        return predict_sim3(points, scale, rotation, translation)

    def apply_transform(transform: Mapping[str, object]) -> np.ndarray:
        scale = float(transform.get("scale", 1.0))
        rotation_matrix = np.asarray(transform.get("rotation_matrix_rdf", np.eye(3)), dtype=float)
        translation = np.asarray(transform.get("translation_rdf_m", [0.0, 0.0, 0.0]), dtype=float)
        return scale * (points @ rotation_matrix.T) + translation

    transformed_by_node = [apply_transform(transform) for transform in transforms]
    output = np.zeros_like(points, dtype=float)
    raw_forward = points[:, 2]
    for row_index, value in enumerate(raw_forward):
        if x_values.size == 1 or value <= x_values[0]:
            output[row_index] = transformed_by_node[0][row_index]
            continue
        if value >= x_values[-1]:
            output[row_index] = transformed_by_node[-1][row_index]
            continue
        right_index = int(np.searchsorted(x_values, value, side="right"))
        left_index = max(0, right_index - 1)
        right_index = min(len(x_values) - 1, right_index)
        width = max(1.0e-12, float(x_values[right_index] - x_values[left_index]))
        alpha = float((value - x_values[left_index]) / width)
        output[row_index] = (1.0 - alpha) * transformed_by_node[left_index][row_index] + alpha * transformed_by_node[right_index][row_index]
    return output


def predict_blended_piecewise_sim3_raw_forward(
    points: np.ndarray,
    *,
    global_scale: float,
    global_rotation: Rotation,
    global_translation: np.ndarray,
    piecewise_model: Mapping[str, object],
    piecewise_alpha: float,
) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    piecewise_alpha = min(1.0, max(0.0, float(piecewise_alpha)))
    global_points = predict_sim3(points, global_scale, global_rotation, global_translation)
    piecewise_points = predict_piecewise_sim3_raw_forward(points, piecewise_model)
    return (1.0 - piecewise_alpha) * global_points + piecewise_alpha * piecewise_points


def fit_piecewise_sim3_raw_forward(
    source: np.ndarray,
    target: np.ndarray,
    *,
    bin_count: int,
    min_bin_samples: int = 4,
) -> dict[str, object]:
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    fallback_scale, fallback_rotation, fallback_translation = fit_umeyama_sim3(source, target)
    bin_count = max(1, int(bin_count))
    min_bin_samples = max(4, int(min_bin_samples))
    raw_forward = source[:, 2]
    finite = np.isfinite(raw_forward)
    if int(np.sum(finite)) < min_bin_samples:
        return {
            "x_values_m": [],
            "transforms": [],
            "fallback_scale": float(fallback_scale),
            "fallback_rotation_matrix_rdf": fallback_rotation.as_matrix().tolist(),
            "fallback_translation_rdf_m": fallback_translation.tolist(),
            "fit_basis": "piecewise raw-forward Sim(3); insufficient finite samples, fallback global Sim(3)",
        }
    quantiles = np.linspace(0.0, 1.0, bin_count + 1)
    edges = np.unique(np.quantile(raw_forward[finite], quantiles))
    if edges.size < 2:
        return {
            "x_values_m": [float(np.median(raw_forward[finite]))],
            "transforms": [
                sim3_transform_to_json(
                    scale=fallback_scale,
                    rotation=fallback_rotation,
                    translation=fallback_translation,
                    sample_count=int(np.sum(finite)),
                )
            ],
            "fallback_scale": float(fallback_scale),
            "fallback_rotation_matrix_rdf": fallback_rotation.as_matrix().tolist(),
            "fallback_translation_rdf_m": fallback_translation.tolist(),
            "fit_basis": "piecewise raw-forward Sim(3); degenerate range, single global node",
        }
    nodes: list[tuple[float, dict[str, object]]] = []
    for index in range(edges.size - 1):
        lo = float(edges[index])
        hi = float(edges[index + 1])
        if index == edges.size - 2:
            mask = finite & (raw_forward >= lo) & (raw_forward <= hi)
        else:
            mask = finite & (raw_forward >= lo) & (raw_forward < hi)
        sample_count = int(np.sum(mask))
        if sample_count >= min_bin_samples:
            scale, rotation, translation = fit_umeyama_sim3(source[mask], target[mask])
        else:
            scale, rotation, translation = fallback_scale, fallback_rotation, fallback_translation
        center = float(np.median(raw_forward[mask])) if sample_count > 0 else float((lo + hi) * 0.5)
        nodes.append(
            (
                center,
                sim3_transform_to_json(
                    scale=scale,
                    rotation=rotation,
                    translation=translation,
                    sample_count=sample_count,
                ),
            )
        )
    nodes.sort(key=lambda item: item[0])
    return {
        "x_values_m": [center for center, _ in nodes],
        "transforms": [transform for _, transform in nodes],
        "fallback_scale": float(fallback_scale),
        "fallback_rotation_matrix_rdf": fallback_rotation.as_matrix().tolist(),
        "fallback_translation_rdf_m": fallback_translation.tolist(),
        "bin_count": int(bin_count),
        "min_bin_samples": int(min_bin_samples),
        "fit_basis": (
            "range-dependent piecewise Sim(3) fitted in raw RDF forward quantile bins; "
            "runtime linearly blends adjacent transformed RDF points by raw_forward_m"
        ),
    }


def fit_affine_rdf(source: np.ndarray, target: np.ndarray, *, ridge_lambda: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must be Nx3 arrays")
    design = np.column_stack((source, np.ones(source.shape[0], dtype=float)))
    if ridge_lambda > 0.0:
        penalty = np.eye(4, dtype=float) * float(ridge_lambda)
        penalty[-1, -1] = 0.0
        coefficients = np.linalg.solve(design.T @ design + penalty, design.T @ target)
    else:
        coefficients, *_ = np.linalg.lstsq(design, target, rcond=None)
    matrix = coefficients[:3, :].T
    translation = coefficients[3, :]
    return matrix, translation


def predict_affine_rdf(points: np.ndarray, matrix: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return np.asarray(points, dtype=float) @ np.asarray(matrix, dtype=float).T + np.asarray(translation, dtype=float)


def fit_diagonal_affine_rdf(
    source: np.ndarray,
    target: np.ndarray,
    *,
    gain_identity_lambda: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must be Nx3 arrays")
    gains = np.ones(3, dtype=float)
    translation = np.zeros(3, dtype=float)
    for axis in range(3):
        design = np.column_stack((source[:, axis], np.ones(source.shape[0], dtype=float)))
        if gain_identity_lambda > 0.0:
            penalty = np.diag([float(gain_identity_lambda), 0.0])
            rhs = design.T @ target[:, axis] + np.array([float(gain_identity_lambda), 0.0])
            coefficients = np.linalg.solve(design.T @ design + penalty, rhs)
        else:
            coefficients, *_ = np.linalg.lstsq(design, target[:, axis], rcond=None)
        gains[axis] = float(coefficients[0])
        translation[axis] = float(coefficients[1])
    return np.diag(gains), translation


def fit_trimmed_umeyama_sim3(
    source: np.ndarray,
    target: np.ndarray,
    *,
    keep_fraction: float = 0.8,
    iterations: int = 3,
) -> tuple[float, Rotation, np.ndarray, np.ndarray]:
    if source.shape[0] < 4:
        scale, rotation, translation = fit_umeyama_sim3(source, target)
        return scale, rotation, translation, np.ones(source.shape[0], dtype=bool)
    keep_fraction = min(1.0, max(0.1, float(keep_fraction)))
    keep_count = max(4, min(source.shape[0], int(round(source.shape[0] * keep_fraction))))
    inlier_mask = np.ones(source.shape[0], dtype=bool)
    scale, rotation, translation = fit_umeyama_sim3(source, target)
    for _ in range(max(1, iterations)):
        predicted = predict_sim3(source, scale, rotation, translation)
        residual_norm = np.linalg.norm(predicted - target, axis=1)
        keep_indices = np.argsort(residual_norm)[:keep_count]
        inlier_mask = np.zeros(source.shape[0], dtype=bool)
        inlier_mask[keep_indices] = True
        scale, rotation, translation = fit_umeyama_sim3(source[inlier_mask], target[inlier_mask])
    return scale, rotation, translation, inlier_mask


def fit_ransac_umeyama_sim3(
    source: np.ndarray,
    target: np.ndarray,
    *,
    iterations: int = 160,
    subset_size: int = 4,
    threshold_m: float | None = None,
    seed: int = 20260613,
) -> tuple[float, Rotation, np.ndarray, np.ndarray]:
    if source.shape[0] < subset_size:
        scale, rotation, translation = fit_umeyama_sim3(source, target)
        return scale, rotation, translation, np.ones(source.shape[0], dtype=bool)
    base_scale, base_rotation, base_translation = fit_umeyama_sim3(source, target)
    base_residual = np.linalg.norm(predict_sim3(source, base_scale, base_rotation, base_translation) - target, axis=1)
    if threshold_m is None:
        median = float(np.median(base_residual))
        mad = float(np.median(np.abs(base_residual - median)))
        threshold = max(1.0, median + 3.0 * 1.4826 * mad)
    else:
        threshold = max(1.0e-6, float(threshold_m))
    rng = np.random.default_rng(seed)
    best_mask = np.ones(source.shape[0], dtype=bool)
    best_score = (-1, float("inf"))
    best_params = (base_scale, base_rotation, base_translation)
    subset_size = max(4, min(source.shape[0], int(subset_size)))
    for _ in range(max(1, iterations)):
        subset = rng.choice(source.shape[0], size=subset_size, replace=False)
        try:
            scale, rotation, translation = fit_umeyama_sim3(source[subset], target[subset])
        except np.linalg.LinAlgError:
            continue
        residual = np.linalg.norm(predict_sim3(source, scale, rotation, translation) - target, axis=1)
        mask = residual <= threshold
        if int(np.sum(mask)) < subset_size:
            continue
        mean_inlier_error = float(np.mean(residual[mask]))
        score = (int(np.sum(mask)), -mean_inlier_error)
        if score > best_score:
            best_score = score
            best_mask = mask
            best_params = (scale, rotation, translation)
    if int(np.sum(best_mask)) >= subset_size:
        return (*fit_umeyama_sim3(source[best_mask], target[best_mask]), best_mask)
    return (*best_params, best_mask)


def fit_huber_umeyama_sim3(
    source: np.ndarray,
    target: np.ndarray,
    *,
    base_weights: np.ndarray | None = None,
    iterations: int = 5,
    delta_m: float | None = None,
) -> tuple[float, Rotation, np.ndarray, np.ndarray]:
    weights = np.ones(source.shape[0], dtype=float) if base_weights is None else np.asarray(base_weights, dtype=float).copy()
    weights = np.where(np.isfinite(weights) & (weights > 0.0), weights, 0.0)
    if float(np.sum(weights)) <= 1.0e-12:
        weights = np.ones(source.shape[0], dtype=float)
    scale, rotation, translation = fit_weighted_umeyama_sim3(source, target, weights)
    for _ in range(max(1, iterations)):
        residual_norm = np.linalg.norm(predict_sim3(source, scale, rotation, translation) - target, axis=1)
        finite_residual = residual_norm[np.isfinite(residual_norm)]
        if delta_m is None:
            median = float(np.median(finite_residual)) if finite_residual.size else 1.0
            mad = float(np.median(np.abs(finite_residual - median))) if finite_residual.size else 1.0
            delta = max(1.0e-6, 1.345 * 1.4826 * mad)
        else:
            delta = max(1.0e-6, float(delta_m))
        huber_weights = np.ones_like(residual_norm)
        large = residual_norm > delta
        huber_weights[large] = delta / np.maximum(residual_norm[large], 1.0e-12)
        weights = (np.ones(source.shape[0], dtype=float) if base_weights is None else np.asarray(base_weights, dtype=float)) * huber_weights
        scale, rotation, translation = fit_weighted_umeyama_sim3(source, target, weights)
    return scale, rotation, translation, weights


def fit_tukey_umeyama_sim3(
    source: np.ndarray,
    target: np.ndarray,
    *,
    base_weights: np.ndarray | None = None,
    iterations: int = 6,
    tuning_m: float | None = None,
) -> tuple[float, Rotation, np.ndarray, np.ndarray]:
    weights = np.ones(source.shape[0], dtype=float) if base_weights is None else np.asarray(base_weights, dtype=float).copy()
    weights = np.where(np.isfinite(weights) & (weights > 0.0), weights, 0.0)
    if float(np.sum(weights)) <= 1.0e-12:
        weights = np.ones(source.shape[0], dtype=float)
    scale, rotation, translation = fit_weighted_umeyama_sim3(source, target, weights)
    for _ in range(max(1, iterations)):
        residual_norm = np.linalg.norm(predict_sim3(source, scale, rotation, translation) - target, axis=1)
        finite_residual = residual_norm[np.isfinite(residual_norm)]
        if tuning_m is None:
            median = float(np.median(finite_residual)) if finite_residual.size else 1.0
            mad = float(np.median(np.abs(finite_residual - median))) if finite_residual.size else 1.0
            tuning = max(1.0e-6, 4.685 * 1.4826 * mad)
        else:
            tuning = max(1.0e-6, float(tuning_m))
        ratio = residual_norm / tuning
        tukey_weights = np.zeros_like(residual_norm)
        inlier = np.isfinite(ratio) & (ratio < 1.0)
        tukey_weights[inlier] = (1.0 - ratio[inlier] ** 2) ** 2
        combined_weights = (
            np.ones(source.shape[0], dtype=float) if base_weights is None else np.asarray(base_weights, dtype=float)
        ) * tukey_weights
        if float(np.sum(combined_weights)) <= 1.0e-12:
            break
        weights = combined_weights
        scale, rotation, translation = fit_weighted_umeyama_sim3(source, target, weights)
    return scale, rotation, translation, weights


def fit_truncated_umeyama_sim3(
    source: np.ndarray,
    target: np.ndarray,
    *,
    base_weights: np.ndarray | None = None,
    iterations: int = 6,
    threshold_m: float | None = None,
    min_inlier_count: int = 4,
) -> tuple[float, Rotation, np.ndarray, np.ndarray, float]:
    weights = np.ones(source.shape[0], dtype=float) if base_weights is None else np.asarray(base_weights, dtype=float).copy()
    weights = np.where(np.isfinite(weights) & (weights > 0.0), weights, 0.0)
    if float(np.sum(weights)) <= 1.0e-12:
        weights = np.ones(source.shape[0], dtype=float)
    min_inlier_count = max(4, min(source.shape[0], int(min_inlier_count)))
    if base_weights is None and source.shape[0] >= min_inlier_count:
        scale, rotation, translation, initial_mask = fit_trimmed_umeyama_sim3(
            source,
            target,
            keep_fraction=max(0.5, min(0.9, min_inlier_count / max(source.shape[0], 1))),
            iterations=2,
        )
        weights = initial_mask.astype(float)
    else:
        scale, rotation, translation = fit_weighted_umeyama_sim3(source, target, weights)
    cutoff = 1.0
    for _ in range(max(1, iterations)):
        residual_norm = np.linalg.norm(predict_sim3(source, scale, rotation, translation) - target, axis=1)
        finite_residual = residual_norm[np.isfinite(residual_norm)]
        if threshold_m is None:
            median = float(np.median(finite_residual)) if finite_residual.size else 1.0
            mad = float(np.median(np.abs(finite_residual - median))) if finite_residual.size else 1.0
            cutoff = max(1.0e-6, median + 3.0 * 1.4826 * mad)
        else:
            cutoff = max(1.0e-6, float(threshold_m))
        tls_weights = np.zeros_like(residual_norm)
        inlier = np.isfinite(residual_norm) & (residual_norm <= cutoff)
        if int(np.sum(inlier)) < min_inlier_count:
            break
        tls_weights[inlier] = 1.0
        weights = (np.ones(source.shape[0], dtype=float) if base_weights is None else np.asarray(base_weights, dtype=float)) * tls_weights
        scale, rotation, translation = fit_weighted_umeyama_sim3(source, target, weights)
    return scale, rotation, translation, weights, cutoff


def fit_robust_nonlinear_sim3(
    source: np.ndarray,
    target: np.ndarray,
    *,
    loss: str,
    f_scale_m: float | None = None,
) -> tuple[float, Rotation, np.ndarray, dict[str, object]]:
    init_scale, init_rotation, init_translation = fit_umeyama_sim3(source, target)
    init_residual = predict_sim3(source, init_scale, init_rotation, init_translation) - target
    residual_norm = np.linalg.norm(init_residual, axis=1)
    if f_scale_m is None:
        median = float(np.median(residual_norm))
        mad = float(np.median(np.abs(residual_norm - median)))
        f_scale = max(1.0e-3, 1.4826 * mad)
    else:
        f_scale = max(1.0e-6, float(f_scale_m))
    initial = np.concatenate(
        (
            np.array([math.log(max(init_scale, 1.0e-12))], dtype=float),
            init_rotation.as_rotvec(),
            init_translation,
        )
    )

    def residual(params: np.ndarray) -> np.ndarray:
        scale = math.exp(float(params[0]))
        rotation = Rotation.from_rotvec(params[1:4])
        translation = params[4:7]
        return (predict_sim3(source, scale, rotation, translation) - target).reshape(-1)

    result = least_squares(
        residual,
        initial,
        loss=loss,
        f_scale=f_scale,
        max_nfev=200,
    )
    scale = math.exp(float(result.x[0]))
    rotation = Rotation.from_rotvec(result.x[1:4])
    translation = np.array(result.x[4:7], dtype=float)
    return scale, rotation, translation, {
        "loss": loss,
        "f_scale_m": f_scale,
        "success": bool(result.success),
        "cost": float(result.cost),
        "nfev": int(result.nfev),
    }


def target_range_weights(
    samples: Sequence[CalibrationSample],
    *,
    target_range_m: float,
    sigma_m: float = 250.0,
    floor: float = 0.08,
) -> np.ndarray:
    truth_ranges = np.array([sample.truth_range_m for sample in samples], dtype=float)
    sigma_m = max(1.0e-6, float(sigma_m))
    floor = min(1.0, max(0.0, float(floor)))
    weights = floor + (1.0 - floor) * np.exp(-0.5 * ((truth_ranges - target_range_m) / sigma_m) ** 2)
    return weights.astype(float)


def predict_disparity_corrected(
    samples: Sequence[CalibrationSample],
    camera: CameraModel,
    coefficients: np.ndarray,
) -> np.ndarray:
    rows = []
    for sample in samples:
        d_corr = float(np.polyval(coefficients, sample.measured_disparity_px))
        d_corr = max(d_corr, 1.0e-6)
        rows.append(
            triangulate_left_disparity(
                left_u_px=sample.left_u_px,
                left_v_px=sample.left_v_px,
                disparity_px=d_corr,
                fx_px=camera.fx_meas_px,
                fy_px=camera.fy_meas_px,
                cx_px=camera.cx_px,
                cy_px=camera.cy_px,
                baseline_m=camera.baseline_meas_m,
            )
        )
    return np.vstack(rows)


def fit_disparity_forward_product(samples: Sequence[CalibrationSample]) -> float:
    """Fit calibrated fB-like product in RDF forward convention from training truth."""
    products = []
    for sample in samples:
        if sample.measured_disparity_px > 1.0e-9 and sample.truth_rdf_m[2] > 1.0e-9:
            products.append(float(sample.measured_disparity_px * sample.truth_rdf_m[2]))
    if not products:
        raise ValueError("cannot fit disparity-forward product without valid train samples")
    return float(np.median(np.asarray(products, dtype=float)))


def predict_disparity_product_ray_scaled(
    samples: Sequence[CalibrationSample],
    calibrated_product_px_m: float,
) -> np.ndarray:
    """Preserve raw RDF bearing and set forward from calibrated_product / measured disparity."""
    raw_points = stack(samples, "raw_rdf_m")
    disparities = measured_disparity(samples)
    corrected = np.array(raw_points, dtype=float, copy=True)
    safe_disparity = np.where(disparities > 1.0e-9, disparities, np.nan)
    safe_forward = np.where(np.abs(raw_points[:, 2]) > 1.0e-9, raw_points[:, 2], np.nan)
    corrected_forward = float(calibrated_product_px_m) / safe_disparity
    ray_scale = corrected_forward / safe_forward
    valid = np.isfinite(ray_scale)
    corrected[valid] = raw_points[valid] * ray_scale[valid, None]
    return corrected


def predict_disparity_ray_rotation(
    samples: Sequence[CalibrationSample],
    coefficients_high_to_low: Sequence[float],
    rotation: Rotation,
) -> np.ndarray:
    """Scale raw RDF ray with corrected disparity, then rotate in stereo-center RDF."""
    raw_points = stack(samples, "raw_rdf_m")
    disparities = measured_disparity(samples)
    coefficients = np.asarray(coefficients_high_to_low, dtype=float).reshape(-1)
    corrected_disparity = np.polyval(coefficients, disparities)
    safe = (
        np.isfinite(corrected_disparity)
        & (corrected_disparity > 1.0e-9)
        & np.isfinite(disparities)
        & (disparities > 1.0e-9)
    )
    ray_scaled = np.array(raw_points, dtype=float, copy=True)
    ray_scaled[safe] = raw_points[safe] * (disparities[safe] / corrected_disparity[safe])[:, None]
    return apply_rotation_to_rows(ray_scaled, rotation)


def predict_left_disparity_camera_geometry(
    samples: Sequence[CalibrationSample],
    *,
    camera: CameraModel,
    model: Mapping[str, object],
) -> np.ndarray:
    """Triangulate stereo-center RDF from left image center and corrected disparity."""
    coefficients = np.asarray(model["disparity_correction_coefficients_high_to_low"], dtype=float)
    left_u_offset_px = float(model.get("left_u_offset_px", 0.0))
    left_v_offset_px = float(model.get("left_v_offset_px", 0.0))
    rows = []
    for sample in samples:
        d_corr = float(np.polyval(coefficients, sample.measured_disparity_px))
        rows.append(
            triangulate_left_disparity(
                left_u_px=sample.left_u_px + left_u_offset_px,
                left_v_px=sample.left_v_px + left_v_offset_px,
                disparity_px=max(d_corr, 1.0e-6),
                fx_px=float(model.get("fx_px", camera.fx_meas_px)),
                fy_px=float(model.get("fy_px", camera.fy_meas_px)),
                cx_px=float(model.get("cx_px", camera.cx_px)),
                cy_px=float(model.get("cy_px", camera.cy_px)),
                baseline_m=float(model.get("baseline_m", camera.baseline_meas_m)),
            )
        )
    return np.vstack(rows)


def fit_nonlinear_left_disparity_camera_geometry(
    samples: Sequence[CalibrationSample],
    *,
    camera: CameraModel,
    weights: np.ndarray | None = None,
    loss: str = "linear",
    f_scale_m: float = 1.0,
) -> tuple[dict[str, object], dict[str, object]]:
    """Fit truth-supervised but runtime-truth-free left-center/disparity triangulation."""
    truth_points = stack(samples, "truth_rdf_m")
    disparities = measured_disparity(samples)
    left_u = np.asarray([sample.left_u_px for sample in samples], dtype=float)
    left_v = np.asarray([sample.left_v_px for sample in samples], dtype=float)
    valid = (
        np.isfinite(disparities)
        & (disparities > 1.0e-9)
        & np.isfinite(left_u)
        & np.isfinite(left_v)
        & np.all(np.isfinite(truth_points), axis=1)
    )
    if int(np.sum(valid)) < 4:
        model = {
            "fx_px": camera.fx_meas_px,
            "fy_px": camera.fy_meas_px,
            "cx_px": camera.cx_px,
            "cy_px": camera.cy_px,
            "baseline_m": camera.baseline_meas_m,
            "left_u_offset_px": 0.0,
            "left_v_offset_px": 0.0,
            "disparity_correction_coefficients_high_to_low": [1.0, 0.0],
        }
        return model, {
            "success": False,
            "reason": "insufficient_valid_samples",
            "valid_count": int(np.sum(valid)),
        }
    truth_valid = truth_points[valid]
    disparity_valid = disparities[valid]
    left_u_valid = left_u[valid]
    left_v_valid = left_v[valid]
    true_d_valid = np.asarray([sample.true_disparity_px for sample, item in zip(samples, valid) if item], dtype=float)
    if weights is None:
        weight_valid = np.ones_like(disparity_valid, dtype=float)
    else:
        raw_weights = np.asarray(weights, dtype=float).reshape(-1)
        if raw_weights.size != disparities.size:
            raise ValueError("weights must match sample count")
        weight_valid = np.where(np.isfinite(raw_weights[valid]) & (raw_weights[valid] > 0.0), raw_weights[valid], 0.0)
    sqrt_weights = np.sqrt(weight_valid)
    initial_coeff = np.polyfit(disparity_valid, true_d_valid, deg=1)
    if not np.all(np.isfinite(initial_coeff)) or initial_coeff[0] <= 0.0:
        initial_coeff = np.array([1.0, 0.0], dtype=float)

    def unpack(params: np.ndarray) -> tuple[np.ndarray, float, float]:
        slope = math.exp(float(params[0]))
        intercept = float(params[1])
        return np.array([slope, intercept], dtype=float), float(params[2]), float(params[3])

    def residual(params: np.ndarray) -> np.ndarray:
        coefficients, u_offset, v_offset = unpack(params)
        corrected_disparity = np.polyval(coefficients, disparity_valid)
        safe_disparity = np.where(corrected_disparity > 1.0e-9, corrected_disparity, 1.0e-9)
        forward = camera.fx_meas_px * camera.baseline_meas_m / safe_disparity
        right = (left_u_valid + u_offset - camera.cx_px) * forward / camera.fx_meas_px - camera.baseline_meas_m * 0.5
        down = (left_v_valid + v_offset - camera.cy_px) * forward / camera.fy_meas_px
        predicted = np.column_stack([right, down, forward])
        return ((predicted - truth_valid) * sqrt_weights[:, None]).reshape(-1)

    x0 = np.array(
        [
            math.log(max(float(initial_coeff[0]), 1.0e-9)),
            float(initial_coeff[1]),
            0.0,
            0.0,
        ],
        dtype=float,
    )
    result = least_squares(
        residual,
        x0,
        loss=loss,
        f_scale=max(1.0e-9, float(f_scale_m)),
        max_nfev=250,
    )
    coefficients, u_offset, v_offset = unpack(result.x)
    model = {
        "fx_px": camera.fx_meas_px,
        "fy_px": camera.fy_meas_px,
        "cx_px": camera.cx_px,
        "cy_px": camera.cy_px,
        "baseline_m": camera.baseline_meas_m,
        "left_u_offset_px": u_offset,
        "left_v_offset_px": v_offset,
        "disparity_correction_coefficients_high_to_low": coefficients.tolist(),
    }
    return model, {
        "success": bool(result.success),
        "cost": float(result.cost),
        "nfev": int(result.nfev),
        "loss": loss,
        "f_scale_m": float(f_scale_m),
        "valid_count": int(np.sum(valid)),
        "mean_weight": float(np.mean(weight_valid)) if weight_valid.size else 0.0,
        "initial_coefficients_high_to_low": initial_coeff.tolist(),
    }


def fit_reprojection_left_disparity_camera_geometry(
    samples: Sequence[CalibrationSample],
    *,
    camera: CameraModel,
    weights: np.ndarray | None = None,
    loss: str = "linear",
    f_scale_px: float = 1.0,
    disparity_sigma_px: float = 1.0,
    bbox_sigma_px: float = 1.0,
) -> tuple[dict[str, object], dict[str, object]]:
    """Fit left-center/disparity parameters from truth-to-observation reprojection residuals.

    Calibration-time truth defines the expected left camera center and disparity. Runtime
    still uses only measured left bbox center, measured disparity, and fixed fitted parameters.
    """
    truth_points = stack(samples, "truth_rdf_m")
    disparities = measured_disparity(samples)
    left_u = np.asarray([sample.left_u_px for sample in samples], dtype=float)
    left_v = np.asarray([sample.left_v_px for sample in samples], dtype=float)
    true_d = true_disparity(samples)
    valid = (
        np.isfinite(disparities)
        & (disparities > 1.0e-9)
        & np.isfinite(left_u)
        & np.isfinite(left_v)
        & np.isfinite(true_d)
        & np.all(np.isfinite(truth_points), axis=1)
        & (truth_points[:, 2] > 1.0e-9)
    )
    if int(np.sum(valid)) < 4:
        model = {
            "fx_px": camera.fx_meas_px,
            "fy_px": camera.fy_meas_px,
            "cx_px": camera.cx_px,
            "cy_px": camera.cy_px,
            "baseline_m": camera.baseline_meas_m,
            "left_u_offset_px": 0.0,
            "left_v_offset_px": 0.0,
            "disparity_correction_coefficients_high_to_low": [1.0, 0.0],
        }
        return model, {
            "success": False,
            "reason": "insufficient_valid_samples",
            "valid_count": int(np.sum(valid)),
        }
    truth_valid = truth_points[valid]
    disparity_valid = disparities[valid]
    left_u_valid = left_u[valid]
    left_v_valid = left_v[valid]
    true_d_valid = true_d[valid]
    truth_left_u = camera.cx_px + (truth_valid[:, 0] + camera.baseline_meas_m * 0.5) * camera.fx_meas_px / truth_valid[:, 2]
    truth_left_v = camera.cy_px + truth_valid[:, 1] * camera.fy_meas_px / truth_valid[:, 2]
    if weights is None:
        weight_valid = np.ones_like(disparity_valid, dtype=float)
    else:
        raw_weights = np.asarray(weights, dtype=float).reshape(-1)
        if raw_weights.size != disparities.size:
            raise ValueError("weights must match sample count")
        weight_valid = np.where(np.isfinite(raw_weights[valid]) & (raw_weights[valid] > 0.0), raw_weights[valid], 0.0)
    sqrt_weights = np.sqrt(weight_valid)
    initial_coeff = np.polyfit(disparity_valid, true_d_valid, deg=1)
    if not np.all(np.isfinite(initial_coeff)) or initial_coeff[0] <= 0.0:
        initial_coeff = np.array([1.0, 0.0], dtype=float)
    initial_u_offset = float(np.average(truth_left_u - left_u_valid, weights=np.maximum(weight_valid, 1.0e-12)))
    initial_v_offset = float(np.average(truth_left_v - left_v_valid, weights=np.maximum(weight_valid, 1.0e-12)))

    def unpack(params: np.ndarray) -> tuple[np.ndarray, float, float]:
        slope = math.exp(float(params[0]))
        intercept = float(params[1])
        return np.array([slope, intercept], dtype=float), float(params[2]), float(params[3])

    def residual(params: np.ndarray) -> np.ndarray:
        coefficients, u_offset, v_offset = unpack(params)
        corrected_disparity = np.polyval(coefficients, disparity_valid)
        disparity_residual = (corrected_disparity - true_d_valid) / max(1.0e-9, float(disparity_sigma_px))
        u_residual = (left_u_valid + u_offset - truth_left_u) / max(1.0e-9, float(bbox_sigma_px))
        v_residual = (left_v_valid + v_offset - truth_left_v) / max(1.0e-9, float(bbox_sigma_px))
        residuals = np.column_stack([disparity_residual, u_residual, v_residual])
        return (residuals * sqrt_weights[:, None]).reshape(-1)

    result = least_squares(
        residual,
        np.array(
            [
                math.log(max(float(initial_coeff[0]), 1.0e-9)),
                float(initial_coeff[1]),
                initial_u_offset,
                initial_v_offset,
            ],
            dtype=float,
        ),
        loss=loss,
        f_scale=max(1.0e-9, float(f_scale_px)),
        max_nfev=250,
    )
    coefficients, u_offset, v_offset = unpack(result.x)
    model = {
        "fx_px": camera.fx_meas_px,
        "fy_px": camera.fy_meas_px,
        "cx_px": camera.cx_px,
        "cy_px": camera.cy_px,
        "baseline_m": camera.baseline_meas_m,
        "left_u_offset_px": u_offset,
        "left_v_offset_px": v_offset,
        "disparity_correction_coefficients_high_to_low": coefficients.tolist(),
    }
    return model, {
        "success": bool(result.success),
        "cost": float(result.cost),
        "nfev": int(result.nfev),
        "loss": loss,
        "f_scale_px": float(f_scale_px),
        "disparity_sigma_px": float(disparity_sigma_px),
        "bbox_sigma_px": float(bbox_sigma_px),
        "valid_count": int(np.sum(valid)),
        "mean_weight": float(np.mean(weight_valid)) if weight_valid.size else 0.0,
        "initial_coefficients_high_to_low": initial_coeff.tolist(),
        "initial_left_u_offset_px": initial_u_offset,
        "initial_left_v_offset_px": initial_v_offset,
    }


def fit_nonlinear_disparity_ray_rotation(
    samples: Sequence[CalibrationSample],
    *,
    weights: np.ndarray | None = None,
    loss: str = "linear",
    f_scale_m: float = 1.0,
) -> tuple[np.ndarray, Rotation, dict[str, object]]:
    """Fit d_corr=a*d+b and RDF rotation jointly from calibration truth.

    Runtime apply is still truth-free: scale the raw RDF ray by d_meas/d_corr
    and apply the fitted stereo-center RDF rotation.
    """
    raw_points = stack(samples, "raw_rdf_m")
    truth_points = stack(samples, "truth_rdf_m")
    disparities = measured_disparity(samples)
    valid = (
        np.isfinite(disparities)
        & (disparities > 1.0e-9)
        & np.all(np.isfinite(raw_points), axis=1)
        & np.all(np.isfinite(truth_points), axis=1)
    )
    if int(np.sum(valid)) < 4:
        return np.array([1.0, 0.0], dtype=float), Rotation.identity(), {
            "success": False,
            "reason": "insufficient_valid_samples",
            "valid_count": int(np.sum(valid)),
        }
    raw_valid = raw_points[valid]
    truth_valid = truth_points[valid]
    disparity_valid = disparities[valid]
    if weights is None:
        weight_valid = np.ones_like(disparity_valid, dtype=float)
    else:
        raw_weights = np.asarray(weights, dtype=float).reshape(-1)
        if raw_weights.size != disparities.size:
            raise ValueError("weights must match sample count")
        weight_valid = raw_weights[valid]
        weight_valid = np.where(np.isfinite(weight_valid) & (weight_valid > 0.0), weight_valid, 0.0)
    sqrt_weights = np.sqrt(weight_valid)
    initial_coeff = np.polyfit(disparity_valid, true_disparity([samples[index] for index, item in enumerate(valid) if item]), deg=1)
    if not np.all(np.isfinite(initial_coeff)) or initial_coeff[0] <= 0.0:
        initial_coeff = np.array([1.0, 0.0], dtype=float)
    initial_rotation = fit_kabsch_rotation(
        raw_valid * (disparity_valid / np.maximum(np.polyval(initial_coeff, disparity_valid), 1.0e-9))[:, None],
        truth_valid,
    )

    def unpack(params: np.ndarray) -> tuple[np.ndarray, Rotation]:
        slope = math.exp(float(params[0]))
        intercept = float(params[1])
        return np.array([slope, intercept], dtype=float), Rotation.from_rotvec(params[2:5])

    def residual(params: np.ndarray) -> np.ndarray:
        coeff, rot = unpack(params)
        corrected_disparity = np.polyval(coeff, disparity_valid)
        safe_disparity = np.where(corrected_disparity > 1.0e-9, corrected_disparity, 1.0e-9)
        ray_scaled = raw_valid * (disparity_valid / safe_disparity)[:, None]
        predicted = apply_rotation_to_rows(ray_scaled, rot)
        residuals = predicted - truth_valid
        return (residuals * sqrt_weights[:, None]).reshape(-1)

    x0 = np.array(
        [
            math.log(max(float(initial_coeff[0]), 1.0e-9)),
            float(initial_coeff[1]),
            *initial_rotation.as_rotvec().tolist(),
        ],
        dtype=float,
    )
    result = least_squares(
        residual,
        x0,
        loss=loss,
        f_scale=max(1.0e-9, float(f_scale_m)),
        max_nfev=250,
    )
    coeff, rotation = unpack(result.x)
    return coeff, rotation, {
        "success": bool(result.success),
        "cost": float(result.cost),
        "nfev": int(result.nfev),
        "loss": loss,
        "f_scale_m": float(f_scale_m),
        "valid_count": int(np.sum(valid)),
        "mean_weight": float(np.mean(weight_valid)) if weight_valid.size else 0.0,
        "initial_coefficients_high_to_low": initial_coeff.tolist(),
    }


def fit_inverse_depth_ray_coefficients(
    raw_forward_m: np.ndarray,
    truth_forward_m: np.ndarray,
    *,
    weights: np.ndarray | None = None,
) -> np.ndarray:
    raw_forward = np.asarray(raw_forward_m, dtype=float).reshape(-1)
    truth_forward = np.asarray(truth_forward_m, dtype=float).reshape(-1)
    if raw_forward.size != truth_forward.size:
        raise ValueError("raw_forward_m and truth_forward_m must have same length")
    valid = np.isfinite(raw_forward) & np.isfinite(truth_forward) & (raw_forward > 1.0e-9) & (truth_forward > 1.0e-9)
    if int(np.sum(valid)) < 2:
        return np.array([1.0, 0.0], dtype=float)
    x_values = 1.0 / raw_forward[valid]
    y_values = 1.0 / truth_forward[valid]
    if weights is None:
        return np.polyfit(x_values, y_values, deg=1)
    weight_values = np.asarray(weights, dtype=float).reshape(-1)
    if weight_values.size != raw_forward.size:
        raise ValueError("weights must match raw_forward_m length")
    safe_weights = np.sqrt(np.where(np.isfinite(weight_values[valid]) & (weight_values[valid] > 0.0), weight_values[valid], 0.0))
    if float(np.sum(safe_weights)) <= 1.0e-12:
        return np.polyfit(x_values, y_values, deg=1)
    return np.polyfit(x_values, y_values, deg=1, w=safe_weights)


def weighted_quantile(values: np.ndarray, weights: np.ndarray, quantile: float) -> float:
    value_array = np.asarray(values, dtype=float).reshape(-1)
    weight_array = np.asarray(weights, dtype=float).reshape(-1)
    if value_array.size != weight_array.size:
        raise ValueError("values and weights must have the same length")
    valid = np.isfinite(value_array) & np.isfinite(weight_array) & (weight_array > 0.0)
    if not np.any(valid):
        return float(np.median(value_array[np.isfinite(value_array)])) if np.any(np.isfinite(value_array)) else 0.0
    sorted_indices = np.argsort(value_array[valid])
    sorted_values = value_array[valid][sorted_indices]
    sorted_weights = weight_array[valid][sorted_indices]
    cumulative = np.cumsum(sorted_weights)
    threshold = min(1.0, max(0.0, float(quantile))) * float(cumulative[-1])
    return float(sorted_values[min(int(np.searchsorted(cumulative, threshold, side="left")), sorted_values.size - 1)])


def fit_theil_sen_inverse_depth_ray_coefficients(
    raw_forward_m: np.ndarray,
    truth_forward_m: np.ndarray,
    *,
    weights: np.ndarray | None = None,
    max_pairs: int = 120000,
) -> tuple[np.ndarray, dict[str, object]]:
    raw_forward = np.asarray(raw_forward_m, dtype=float).reshape(-1)
    truth_forward = np.asarray(truth_forward_m, dtype=float).reshape(-1)
    if raw_forward.size != truth_forward.size:
        raise ValueError("raw_forward_m and truth_forward_m must have same length")
    valid = np.isfinite(raw_forward) & np.isfinite(truth_forward) & (raw_forward > 1.0e-9) & (truth_forward > 1.0e-9)
    if int(np.sum(valid)) < 2:
        return np.array([1.0, 0.0], dtype=float), {
            "valid_count": int(np.sum(valid)),
            "pair_count": 0,
            "max_pairs": int(max_pairs),
        }
    x_values = 1.0 / raw_forward[valid]
    y_values = 1.0 / truth_forward[valid]
    if weights is None:
        weight_values = np.ones_like(x_values)
    else:
        raw_weights = np.asarray(weights, dtype=float).reshape(-1)
        if raw_weights.size != raw_forward.size:
            raise ValueError("weights must match raw_forward_m length")
        weight_values = np.where(np.isfinite(raw_weights[valid]) & (raw_weights[valid] > 0.0), raw_weights[valid], 0.0)
        if float(np.sum(weight_values)) <= 1.0e-12:
            weight_values = np.ones_like(x_values)
    slopes: list[float] = []
    slope_weights: list[float] = []
    pair_count = 0
    step = 1
    total_pairs = int(x_values.size * (x_values.size - 1) / 2)
    if total_pairs > max(1, int(max_pairs)):
        step = int(math.ceil(total_pairs / max(1, int(max_pairs))))
    for left in range(x_values.size - 1):
        dx = x_values[left + 1 :] - x_values[left]
        dy = y_values[left + 1 :] - y_values[left]
        safe = np.abs(dx) > 1.0e-15
        if not np.any(safe):
            continue
        local_slopes = dy[safe] / dx[safe]
        local_weights = np.sqrt(weight_values[left] * weight_values[left + 1 :][safe])
        if step > 1:
            offset = pair_count % step
            local_slopes = local_slopes[offset::step]
            local_weights = local_weights[offset::step]
        slopes.extend(float(value) for value in local_slopes if math.isfinite(float(value)))
        slope_weights.extend(
            float(weight)
            for value, weight in zip(local_slopes, local_weights)
            if math.isfinite(float(value)) and math.isfinite(float(weight)) and float(weight) > 0.0
        )
        pair_count += int(local_slopes.size)
    if not slopes:
        coeff = fit_inverse_depth_ray_coefficients(raw_forward, truth_forward, weights=weights)
        return coeff, {
            "valid_count": int(np.sum(valid)),
            "pair_count": 0,
            "max_pairs": int(max_pairs),
            "fallback": "weighted_polyfit",
        }
    slope_array = np.asarray(slopes, dtype=float)
    slope_weight_array = np.asarray(slope_weights, dtype=float)
    slope = weighted_quantile(slope_array, slope_weight_array, 0.5)
    intercept_values = y_values - slope * x_values
    intercept = weighted_quantile(intercept_values, weight_values, 0.5)
    coeff = np.array([slope, intercept], dtype=float)
    info = {
        "valid_count": int(np.sum(valid)),
        "pair_count": int(len(slopes)),
        "total_pair_count": total_pairs,
        "max_pairs": int(max_pairs),
        "pair_subsample_step": step,
        "mean_weight": float(np.mean(weight_values)),
        "fit_basis": (
            "Theil-Sen median pairwise slope in inverse-depth space; intercept is weighted median residual. "
            "Runtime scales the raw RDF ray using raw_forward_m only."
        ),
    }
    return coeff, info


def fit_ridge_inverse_depth_ray_coefficients(
    raw_forward_m: np.ndarray,
    truth_forward_m: np.ndarray,
    *,
    weights: np.ndarray | None = None,
    ridge_lambda: float = 1.0,
    prior_coefficients: Sequence[float] = (1.0, 0.0),
) -> tuple[np.ndarray, dict[str, object]]:
    raw_forward = np.asarray(raw_forward_m, dtype=float).reshape(-1)
    truth_forward = np.asarray(truth_forward_m, dtype=float).reshape(-1)
    if raw_forward.size != truth_forward.size:
        raise ValueError("raw_forward_m and truth_forward_m must have same length")
    valid = np.isfinite(raw_forward) & np.isfinite(truth_forward) & (raw_forward > 1.0e-9) & (truth_forward > 1.0e-9)
    if int(np.sum(valid)) < 2:
        return np.array([1.0, 0.0], dtype=float), {
            "ridge_lambda": float(ridge_lambda),
            "valid_count": int(np.sum(valid)),
            "x_scale_1_per_m": 1.0,
            "prior_coefficients_high_to_low": list(prior_coefficients),
        }
    x_values = 1.0 / raw_forward[valid]
    y_values = 1.0 / truth_forward[valid]
    x_scale = max(float(np.median(np.abs(x_values))), 1.0e-12)
    x_normalized = x_values / x_scale
    y_normalized = y_values / x_scale
    design = np.column_stack([x_normalized, np.ones_like(x_normalized)])
    prior = np.asarray(prior_coefficients, dtype=float).reshape(-1)
    if prior.size != 2:
        raise ValueError("prior_coefficients must contain [slope, intercept]")
    prior_normalized = np.array([float(prior[0]), float(prior[1]) / x_scale], dtype=float)
    if weights is None:
        weight_values = np.ones_like(x_values)
    else:
        raw_weights = np.asarray(weights, dtype=float).reshape(-1)
        if raw_weights.size != raw_forward.size:
            raise ValueError("weights must match raw_forward_m length")
        weight_values = np.where(
            np.isfinite(raw_weights[valid]) & (raw_weights[valid] > 0.0),
            raw_weights[valid],
            0.0,
        )
        if float(np.sum(weight_values)) <= 1.0e-12:
            weight_values = np.ones_like(x_values)
    sqrt_weights = np.sqrt(weight_values)
    weighted_design = design * sqrt_weights[:, None]
    weighted_target = y_normalized * sqrt_weights
    ridge = max(float(ridge_lambda), 0.0)
    lhs = weighted_design.T @ weighted_design + ridge * np.eye(2)
    rhs = weighted_design.T @ weighted_target + ridge * prior_normalized
    try:
        beta = np.linalg.solve(lhs, rhs)
    except np.linalg.LinAlgError:
        beta = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    coeff = np.array([float(beta[0]), float(beta[1]) * x_scale], dtype=float)
    info = {
        "ridge_lambda": float(ridge_lambda),
        "valid_count": int(np.sum(valid)),
        "x_scale_1_per_m": x_scale,
        "prior_coefficients_high_to_low": [float(prior[0]), float(prior[1])],
        "mean_weight": float(np.mean(weight_values)),
    }
    return coeff, info


def fit_huber_inverse_depth_ray_coefficients(
    raw_forward_m: np.ndarray,
    truth_forward_m: np.ndarray,
    *,
    weights: np.ndarray | None = None,
    delta: float = 1.345,
    iterations: int = 8,
) -> tuple[np.ndarray, dict[str, object]]:
    raw_forward = np.asarray(raw_forward_m, dtype=float).reshape(-1)
    truth_forward = np.asarray(truth_forward_m, dtype=float).reshape(-1)
    if raw_forward.size != truth_forward.size:
        raise ValueError("raw_forward_m and truth_forward_m must have same length")
    valid = np.isfinite(raw_forward) & np.isfinite(truth_forward) & (raw_forward > 1.0e-9) & (truth_forward > 1.0e-9)
    if int(np.sum(valid)) < 2:
        return np.array([1.0, 0.0], dtype=float), {
            "iterations": 0,
            "delta": float(delta),
            "valid_count": int(np.sum(valid)),
            "mean_irls_weight": 1.0,
            "min_irls_weight": 1.0,
        }
    x_values = 1.0 / raw_forward[valid]
    y_values = 1.0 / truth_forward[valid]
    if weights is None:
        base_weights = np.ones_like(x_values)
    else:
        weight_values = np.asarray(weights, dtype=float).reshape(-1)
        if weight_values.size != raw_forward.size:
            raise ValueError("weights must match raw_forward_m length")
        base_weights = np.where(
            np.isfinite(weight_values[valid]) & (weight_values[valid] > 0.0),
            weight_values[valid],
            0.0,
        )
        if float(np.sum(base_weights)) <= 1.0e-12:
            base_weights = np.ones_like(x_values)
    coeff = np.polyfit(x_values, y_values, deg=1, w=np.sqrt(base_weights))
    irls_weights = np.ones_like(x_values)
    for _ in range(max(1, int(iterations))):
        residual = y_values - np.polyval(coeff, x_values)
        median = float(np.median(residual))
        mad = float(np.median(np.abs(residual - median)))
        scale = max(1.4826 * mad, float(np.std(residual)), 1.0e-12)
        threshold = max(float(delta), 1.0e-9) * scale
        abs_residual = np.abs(residual)
        irls_weights = np.ones_like(abs_residual)
        large = abs_residual > threshold
        irls_weights[large] = threshold / np.maximum(abs_residual[large], 1.0e-12)
        combined_weights = base_weights * irls_weights
        if float(np.sum(combined_weights)) <= 1.0e-12:
            break
        coeff = np.polyfit(x_values, y_values, deg=1, w=np.sqrt(combined_weights))
    info = {
        "iterations": max(1, int(iterations)),
        "delta": float(delta),
        "valid_count": int(np.sum(valid)),
        "mean_irls_weight": float(np.mean(irls_weights)),
        "min_irls_weight": float(np.min(irls_weights)),
        "mean_combined_weight": float(np.mean(base_weights * irls_weights)),
    }
    return coeff, info


def apply_inverse_depth_ray_correction(points: np.ndarray, coefficients: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    coeff = np.asarray(coefficients, dtype=float)
    if coeff.size == 0:
        return points.copy()
    forward = points[:, 2]
    safe_forward = np.where(forward > 1.0e-9, forward, 1.0e-9)
    inverse_depth = 1.0 / safe_forward
    corrected_inverse_depth = np.polyval(coeff, inverse_depth)
    corrected_inverse_depth = np.where(corrected_inverse_depth > 1.0e-12, corrected_inverse_depth, 1.0e-12)
    corrected_forward = 1.0 / corrected_inverse_depth
    scale = corrected_forward / safe_forward
    return points * scale[:, None]


def predict_inverse_depth_ray_corrected(
    samples: Sequence[CalibrationSample],
    coefficients: np.ndarray,
) -> np.ndarray:
    return apply_inverse_depth_ray_correction(stack(samples, "raw_rdf_m"), coefficients)


def predict_inverse_depth_ray_corrected_sim3(
    samples: Sequence[CalibrationSample],
    coefficients: np.ndarray,
    scale: float,
    rotation: Rotation,
    translation: np.ndarray,
) -> np.ndarray:
    corrected_ray = predict_inverse_depth_ray_corrected(samples, coefficients)
    return predict_sim3(corrected_ray, scale, rotation, translation)


def predict_disparity_corrected_sim3(
    samples: Sequence[CalibrationSample],
    camera: CameraModel,
    coefficients: np.ndarray,
    scale: float,
    rotation: Rotation,
    translation: np.ndarray,
) -> np.ndarray:
    corrected_ray = predict_disparity_corrected(samples, camera, coefficients)
    return predict_sim3(corrected_ray, scale, rotation, translation)


def fit_ridge_polyfit(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    degree: int,
    alpha: float,
    penalize_intercept: bool = False,
) -> np.ndarray:
    x_values = np.asarray(x_values, dtype=float).reshape(-1)
    y_values = np.asarray(y_values, dtype=float).reshape(-1)
    if degree < 0:
        raise ValueError("degree must be non-negative")
    if x_values.size != y_values.size:
        raise ValueError("x_values and y_values must have same length")
    if x_values.size <= degree:
        return np.polyfit(x_values, y_values, deg=min(degree, max(0, x_values.size - 1)))
    x_center = float(np.mean(x_values))
    x_scale = float(np.std(x_values))
    if x_scale <= 1.0e-12:
        x_scale = 1.0
    x_normalized = (x_values - x_center) / x_scale
    design = np.vander(x_normalized, degree + 1, increasing=False)
    regularizer = max(0.0, float(alpha)) * np.eye(degree + 1)
    if not penalize_intercept:
        regularizer[-1, -1] = 0.0
    coeff_normalized = np.linalg.solve(design.T @ design + regularizer, design.T @ y_values)
    normalized_poly = np.poly1d(coeff_normalized)
    original_basis = np.poly1d([1.0 / x_scale, -x_center / x_scale])
    coeff_original = normalized_poly(original_basis).c
    return np.asarray(coeff_original, dtype=float)


def fit_weighted_ridge_polyfit(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    degree: int,
    alpha: float,
    weights: np.ndarray,
    penalize_intercept: bool = False,
) -> np.ndarray:
    x_values = np.asarray(x_values, dtype=float).reshape(-1)
    y_values = np.asarray(y_values, dtype=float).reshape(-1)
    weights = np.asarray(weights, dtype=float).reshape(-1)
    if degree < 0:
        raise ValueError("degree must be non-negative")
    if x_values.size != y_values.size or x_values.size != weights.size:
        raise ValueError("x_values, y_values, and weights must have same length")
    if x_values.size <= degree:
        safe_weights = np.sqrt(np.where(np.isfinite(weights) & (weights > 0.0), weights, 0.0))
        if float(np.sum(safe_weights)) <= 1.0e-12:
            return np.polyfit(x_values, y_values, deg=min(degree, max(0, x_values.size - 1)))
        return np.polyfit(
            x_values,
            y_values,
            deg=min(degree, max(0, x_values.size - 1)),
            w=safe_weights,
        )
    positive_weights = np.where(np.isfinite(weights) & (weights > 0.0), weights, 0.0)
    if float(np.sum(positive_weights)) <= 1.0e-12:
        return fit_ridge_polyfit(
            x_values,
            y_values,
            degree=degree,
            alpha=alpha,
            penalize_intercept=penalize_intercept,
        )
    x_center = float(np.average(x_values, weights=positive_weights))
    x_scale = float(np.sqrt(np.average(np.square(x_values - x_center), weights=positive_weights)))
    if not math.isfinite(x_scale) or x_scale <= 1.0e-12:
        x_center = float(np.mean(x_values))
        x_scale = float(np.std(x_values))
    if x_scale <= 1.0e-12:
        x_scale = 1.0
    x_normalized = (x_values - x_center) / x_scale
    design = np.vander(x_normalized, degree + 1, increasing=False)
    weighted_design = design * positive_weights[:, None]
    regularizer = max(0.0, float(alpha)) * np.eye(degree + 1)
    if not penalize_intercept:
        regularizer[-1, -1] = 0.0
    coeff_normalized = np.linalg.solve(weighted_design.T @ design + regularizer, weighted_design.T @ y_values)
    normalized_poly = np.poly1d(coeff_normalized)
    original_basis = np.poly1d([1.0 / x_scale, -x_center / x_scale])
    coeff_original = normalized_poly(original_basis).c
    return np.asarray(coeff_original, dtype=float)


def fit_best_ridge_polyfit(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    degrees: Sequence[int] = (0, 1, 2),
    alphas: Sequence[float] = (0.0, 0.1, 1.0, 10.0, 100.0),
) -> tuple[np.ndarray, dict[str, object]]:
    best: tuple[float, int, float, np.ndarray] | None = None
    for degree in degrees:
        for alpha in alphas:
            coefficients = fit_ridge_polyfit(x_values, y_values, degree=int(degree), alpha=float(alpha))
            prediction = np.polyval(coefficients, x_values)
            residual = prediction - y_values
            score = rmse(residual) + 0.002 * int(degree) + 0.0005 * math.log10(1.0 + float(alpha))
            candidate = (score, int(degree), float(alpha), coefficients)
            if best is None or candidate[0] < best[0]:
                best = candidate
    if best is None:
        raise ValueError("no ridge polynomial candidates were fitted")
    _, degree, alpha, coefficients = best
    return coefficients, {
        "degree": degree,
        "alpha": alpha,
        "candidate_degrees": list(degrees),
        "candidate_alphas": list(alphas),
    }


def raw_rdf_polynomial_feature_matrix(
    raw_rdf: np.ndarray,
    *,
    degree: int,
    forward_center_m: float,
    forward_scale_m: float,
) -> tuple[np.ndarray, list[str]]:
    """Build dimensionless raw-RDF features for runtime-applicable residual fields."""
    raw_rdf = np.asarray(raw_rdf, dtype=float)
    if raw_rdf.ndim != 2 or raw_rdf.shape[1] != 3:
        raise ValueError("raw_rdf must be an Nx3 RDF array")
    if degree not in (1, 2):
        raise ValueError("raw RDF residual polynomial degree must be 1 or 2")
    forward = raw_rdf[:, 2]
    safe_forward = np.where(np.abs(forward) > 1.0e-9, forward, 1.0e-9)
    right_over_forward = raw_rdf[:, 0] / safe_forward
    down_over_forward = raw_rdf[:, 1] / safe_forward
    scale = float(forward_scale_m)
    if abs(scale) <= 1.0e-9:
        scale = 1.0
    forward_norm = (forward - float(forward_center_m)) / scale
    columns = [
        np.ones_like(forward),
        right_over_forward,
        down_over_forward,
        forward_norm,
    ]
    names = [
        "1",
        "raw_right_over_raw_forward",
        "raw_down_over_raw_forward",
        "raw_forward_normalized",
    ]
    if degree >= 2:
        columns.extend(
            [
                right_over_forward * right_over_forward,
                right_over_forward * down_over_forward,
                down_over_forward * down_over_forward,
                right_over_forward * forward_norm,
                down_over_forward * forward_norm,
                forward_norm * forward_norm,
            ]
        )
        names.extend(
            [
                "raw_right_over_raw_forward^2",
                "raw_right_over_raw_forward*raw_down_over_raw_forward",
                "raw_down_over_raw_forward^2",
                "raw_right_over_raw_forward*raw_forward_normalized",
                "raw_down_over_raw_forward*raw_forward_normalized",
                "raw_forward_normalized^2",
            ]
        )
    return np.column_stack(columns), names


def fit_ridge_raw_rdf_residual_field(
    raw_rdf: np.ndarray,
    residual_rdf: np.ndarray,
    *,
    degree: int,
    alpha: float,
    residual_norm_cap_m: float | None = None,
) -> dict[str, object]:
    raw_rdf = np.asarray(raw_rdf, dtype=float)
    residual_rdf = np.asarray(residual_rdf, dtype=float)
    if raw_rdf.shape != residual_rdf.shape or raw_rdf.ndim != 2 or raw_rdf.shape[1] != 3:
        raise ValueError("raw_rdf and residual_rdf must be matching Nx3 RDF arrays")
    forward = raw_rdf[:, 2]
    forward_center_m = float(np.median(forward)) if forward.size else 0.0
    q25, q75 = np.quantile(forward, [0.25, 0.75]) if forward.size else (0.0, 1.0)
    forward_scale_m = float(q75 - q25)
    if abs(forward_scale_m) <= 1.0e-9:
        forward_scale_m = float(np.std(forward)) if forward.size else 1.0
    if abs(forward_scale_m) <= 1.0e-9:
        forward_scale_m = 1.0
    design, feature_names = raw_rdf_polynomial_feature_matrix(
        raw_rdf,
        degree=degree,
        forward_center_m=forward_center_m,
        forward_scale_m=forward_scale_m,
    )
    regularizer = max(0.0, float(alpha)) * np.eye(design.shape[1])
    regularizer[0, 0] = 0.0
    coefficients = np.linalg.solve(design.T @ design + regularizer, design.T @ residual_rdf)
    fitted = design @ coefficients
    if residual_norm_cap_m is not None:
        cap = max(0.0, float(residual_norm_cap_m))
        norms = np.linalg.norm(fitted, axis=1)
        scale = np.minimum(1.0, cap / np.maximum(norms, 1.0e-12))
        fitted = fitted * scale[:, None]
    residual_error = fitted - residual_rdf
    return {
        "degree": int(degree),
        "alpha": float(alpha),
        "feature_names": feature_names,
        "forward_center_m": forward_center_m,
        "forward_scale_m": forward_scale_m,
        "coefficient_matrix_m": coefficients.tolist(),
        "residual_norm_cap_m": None if residual_norm_cap_m is None else float(residual_norm_cap_m),
        "train_residual_fit_rmse_m": rmse(np.linalg.norm(residual_error, axis=1)),
        "fit_basis": (
            "ridge-regularized vector residual field over raw stereo-center RDF features; "
            "runtime uses raw RDF only, no calibration truth"
        ),
    }


def apply_raw_rdf_residual_field(
    points: np.ndarray,
    raw_rdf: np.ndarray,
    model: Mapping[str, object],
) -> np.ndarray:
    corrected = np.array(points, dtype=float, copy=True)
    coefficients = np.asarray(model.get("coefficient_matrix_m", []), dtype=float)
    if coefficients.ndim != 2 or coefficients.shape[1] != 3 or coefficients.size == 0:
        return corrected
    features, _ = raw_rdf_polynomial_feature_matrix(
        raw_rdf,
        degree=int(model.get("degree", 1)),
        forward_center_m=float(model.get("forward_center_m", 0.0)),
        forward_scale_m=float(model.get("forward_scale_m", 1.0)),
    )
    if features.shape[1] != coefficients.shape[0]:
        raise ValueError("raw RDF residual polynomial feature/coefficient size mismatch")
    residual = features @ coefficients
    cap_value = model.get("residual_norm_cap_m")
    if cap_value is not None:
        cap = max(0.0, float(cap_value))
        norms = np.linalg.norm(residual, axis=1)
        scale = np.minimum(1.0, cap / np.maximum(norms, 1.0e-12))
        residual = residual * scale[:, None]
    corrected += residual
    return corrected


def fit_weighted_polyfit(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    degree: int,
    weights: np.ndarray,
) -> np.ndarray:
    x_values = np.asarray(x_values, dtype=float).reshape(-1)
    y_values = np.asarray(y_values, dtype=float).reshape(-1)
    weights = np.asarray(weights, dtype=float).reshape(-1)
    if x_values.size != y_values.size or x_values.size != weights.size:
        raise ValueError("x_values, y_values, and weights must have same length")
    degree = min(int(degree), max(0, x_values.size - 1))
    safe_weights = np.sqrt(np.where(np.isfinite(weights) & (weights > 0.0), weights, 0.0))
    if float(np.sum(safe_weights)) <= 1.0e-12:
        return np.polyfit(x_values, y_values, deg=degree)
    return np.polyfit(x_values, y_values, deg=degree, w=safe_weights)


def fit_binned_residual_lut(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    bin_count: int = 7,
    min_samples_per_bin: int = 4,
) -> dict[str, object]:
    x_values = np.asarray(x_values, dtype=float).reshape(-1)
    y_values = np.asarray(y_values, dtype=float).reshape(-1)
    valid = np.isfinite(x_values) & np.isfinite(y_values)
    x_values = x_values[valid]
    y_values = y_values[valid]
    if x_values.size == 0:
        return {"x_values_px": [], "residual_values_m": [], "bin_sample_counts": []}
    order = np.argsort(x_values)
    x_values = x_values[order]
    y_values = y_values[order]
    bin_count = max(1, min(int(bin_count), int(x_values.size)))
    edges = np.linspace(0, x_values.size, bin_count + 1, dtype=int)
    x_lut: list[float] = []
    y_lut: list[float] = []
    counts: list[int] = []
    for start, stop in zip(edges[:-1], edges[1:]):
        if stop <= start:
            continue
        bin_x = x_values[start:stop]
        bin_y = y_values[start:stop]
        if bin_x.size < max(1, min_samples_per_bin) and x_values.size >= min_samples_per_bin:
            continue
        x_lut.append(float(np.median(bin_x)))
        y_lut.append(float(np.median(bin_y)))
        counts.append(int(bin_x.size))
    if not x_lut:
        x_lut = [float(np.median(x_values))]
        y_lut = [float(np.median(y_values))]
        counts = [int(x_values.size)]
    return {
        "x_values_px": x_lut,
        "residual_values_m": y_lut,
        "bin_sample_counts": counts,
        "fit_basis": "quantile-binned median residual LUT with linear interpolation and edge clamping",
    }


def fit_weighted_binned_residual_lut(
    x_values: np.ndarray,
    y_values: np.ndarray,
    weights: np.ndarray,
    *,
    bin_count: int = 7,
    min_samples_per_bin: int = 4,
) -> dict[str, object]:
    x_values = np.asarray(x_values, dtype=float).reshape(-1)
    y_values = np.asarray(y_values, dtype=float).reshape(-1)
    weights = np.asarray(weights, dtype=float).reshape(-1)
    if x_values.size != y_values.size or x_values.size != weights.size:
        raise ValueError("x_values, y_values, and weights must have same length")
    valid = np.isfinite(x_values) & np.isfinite(y_values) & np.isfinite(weights) & (weights > 0.0)
    x_values = x_values[valid]
    y_values = y_values[valid]
    weights = weights[valid]
    if x_values.size == 0:
        return {"x_values_px": [], "residual_values_m": [], "bin_sample_counts": [], "bin_weight_sums": []}
    order = np.argsort(x_values)
    x_values = x_values[order]
    y_values = y_values[order]
    weights = weights[order]
    bin_count = max(1, min(int(bin_count), int(x_values.size)))
    edges = np.linspace(0, x_values.size, bin_count + 1, dtype=int)
    x_lut: list[float] = []
    y_lut: list[float] = []
    counts: list[int] = []
    weight_sums: list[float] = []
    for start, stop in zip(edges[:-1], edges[1:]):
        if stop <= start:
            continue
        bin_x = x_values[start:stop]
        bin_y = y_values[start:stop]
        bin_weights = weights[start:stop]
        if bin_x.size < max(1, min_samples_per_bin) and x_values.size >= min_samples_per_bin:
            continue
        x_lut.append(weighted_quantile(bin_x, bin_weights, 0.5))
        y_lut.append(weighted_quantile(bin_y, bin_weights, 0.5))
        counts.append(int(bin_x.size))
        weight_sums.append(float(np.sum(bin_weights)))
    if not x_lut:
        x_lut = [weighted_quantile(x_values, weights, 0.5)]
        y_lut = [weighted_quantile(y_values, weights, 0.5)]
        counts = [int(x_values.size)]
        weight_sums = [float(np.sum(weights))]
    return {
        "x_values_px": x_lut,
        "residual_values_m": y_lut,
        "bin_sample_counts": counts,
        "bin_weight_sums": weight_sums,
        "fit_basis": "target-weighted quantile-binned weighted-median residual LUT with linear interpolation and edge clamping",
    }


def weighted_pava(values: np.ndarray, weights: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float).reshape(-1)
    weights = np.asarray(weights, dtype=float).reshape(-1)
    if values.size != weights.size:
        raise ValueError("values and weights must have same length")
    if values.size == 0:
        return np.asarray([], dtype=float)
    safe_weights = np.where(np.isfinite(weights) & (weights > 0.0), weights, 1.0)
    block_values: list[float] = []
    block_weights: list[float] = []
    block_counts: list[int] = []
    for value, weight in zip(values, safe_weights):
        block_values.append(float(value))
        block_weights.append(float(weight))
        block_counts.append(1)
        while len(block_values) >= 2 and block_values[-2] > block_values[-1]:
            merged_weight = block_weights[-2] + block_weights[-1]
            merged_value = (
                block_values[-2] * block_weights[-2] + block_values[-1] * block_weights[-1]
            ) / merged_weight
            merged_count = block_counts[-2] + block_counts[-1]
            block_values[-2:] = [float(merged_value)]
            block_weights[-2:] = [float(merged_weight)]
            block_counts[-2:] = [int(merged_count)]
    return np.asarray(
        [value for value, count in zip(block_values, block_counts) for _ in range(count)],
        dtype=float,
    )


def fit_monotone_binned_residual_lut(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    bin_count: int = 7,
    min_samples_per_bin: int = 4,
) -> dict[str, object]:
    base_lut = fit_binned_residual_lut(
        x_values,
        y_values,
        bin_count=bin_count,
        min_samples_per_bin=min_samples_per_bin,
    )
    residual_values = np.asarray(base_lut.get("residual_values_m", []), dtype=float)
    counts = np.asarray(base_lut.get("bin_sample_counts", []), dtype=float)
    if residual_values.size == 0:
        base_lut["fit_basis"] = "monotone quantile-binned median residual LUT; empty fit"
        return base_lut
    if counts.size != residual_values.size:
        counts = np.ones_like(residual_values)
    increasing = weighted_pava(residual_values, counts)
    decreasing = -weighted_pava(-residual_values, counts)
    inc_loss = float(np.sum(counts * np.square(increasing - residual_values)))
    dec_loss = float(np.sum(counts * np.square(decreasing - residual_values)))
    if inc_loss <= dec_loss:
        monotone = increasing
        direction = "nondecreasing"
        projection_loss = inc_loss
    else:
        monotone = decreasing
        direction = "nonincreasing"
        projection_loss = dec_loss
    base_lut["residual_values_m"] = [float(value) for value in monotone]
    base_lut["raw_residual_values_m"] = [float(value) for value in residual_values]
    base_lut["monotone_direction"] = direction
    base_lut["monotone_projection_loss"] = projection_loss
    base_lut["fit_basis"] = (
        "quantile-binned median residual LUT projected with weighted PAVA isotonic "
        "regression; direction selected by lower weighted training projection loss"
    )
    return base_lut


def fit_smoothed_binned_residual_lut(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    bin_count: int = 9,
    min_samples_per_bin: int = 4,
    prior_strength: float = 8.0,
    neighbor_weight: float = 0.25,
) -> dict[str, object]:
    base_lut = fit_binned_residual_lut(
        x_values,
        y_values,
        bin_count=bin_count,
        min_samples_per_bin=min_samples_per_bin,
    )
    residual_values = np.asarray(base_lut.get("residual_values_m", []), dtype=float)
    counts = np.asarray(base_lut.get("bin_sample_counts", []), dtype=float)
    if residual_values.size == 0:
        base_lut["fit_basis"] = "smoothed quantile-binned median residual LUT; empty fit"
        return base_lut
    if counts.size != residual_values.size:
        counts = np.ones_like(residual_values)
    prior = float(np.median(residual_values))
    prior_strength = max(0.0, float(prior_strength))
    shrunk = (counts * residual_values + prior_strength * prior) / (counts + prior_strength)
    neighbor_weight = min(0.5, max(0.0, float(neighbor_weight)))
    smoothed = np.array(shrunk, dtype=float, copy=True)
    if shrunk.size >= 3 and neighbor_weight > 0.0:
        smoothed[1:-1] = (
            neighbor_weight * shrunk[:-2]
            + (1.0 - 2.0 * neighbor_weight) * shrunk[1:-1]
            + neighbor_weight * shrunk[2:]
        )
    base_lut["residual_values_m"] = [float(value) for value in smoothed]
    base_lut["raw_residual_values_m"] = [float(value) for value in residual_values]
    base_lut["prior_residual_m"] = float(prior)
    base_lut["prior_strength_samples"] = float(prior_strength)
    base_lut["neighbor_weight"] = float(neighbor_weight)
    base_lut["fit_basis"] = (
        "quantile-binned median residual LUT with empirical-Bayes shrinkage "
        "toward the global median and nearest-neighbor smoothing"
    )
    return base_lut


def fit_kernel_smoothed_residual_lut(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    support_count: int = 11,
    bandwidth: float,
    min_effective_weight: float = 1.0e-9,
) -> dict[str, object]:
    x_values = np.asarray(x_values, dtype=float).reshape(-1)
    y_values = np.asarray(y_values, dtype=float).reshape(-1)
    valid = np.isfinite(x_values) & np.isfinite(y_values)
    x_values = x_values[valid]
    y_values = y_values[valid]
    if x_values.size == 0:
        return {
            "x_values_px": [],
            "residual_values_m": [],
            "fit_basis": "Gaussian-kernel Nadaraya-Watson residual LUT; empty fit",
        }
    order = np.argsort(x_values)
    x_values = x_values[order]
    y_values = y_values[order]
    support_count = max(1, min(int(support_count), int(x_values.size)))
    quantiles = np.linspace(0.0, 1.0, support_count)
    support_x = np.unique(np.quantile(x_values, quantiles))
    if support_x.size == 0:
        support_x = np.array([float(np.median(x_values))], dtype=float)
    span = float(np.max(x_values) - np.min(x_values))
    if not math.isfinite(bandwidth) or bandwidth <= 0.0:
        bandwidth = max(span / max(1, support_count - 1), 1.0e-6)
    bandwidth = max(float(bandwidth), 1.0e-9)
    global_median = float(np.median(y_values))
    support_y: list[float] = []
    effective_counts: list[float] = []
    for center in support_x:
        normalized = (x_values - float(center)) / bandwidth
        weights = np.exp(-0.5 * np.square(normalized))
        weight_sum = float(np.sum(weights))
        effective_counts.append(weight_sum)
        if weight_sum <= min_effective_weight:
            support_y.append(global_median)
        else:
            support_y.append(float(np.sum(weights * y_values) / weight_sum))
    return {
        "x_values_px": [float(value) for value in support_x],
        "residual_values_m": support_y,
        "kernel_bandwidth": float(bandwidth),
        "kernel_support_count": int(support_x.size),
        "kernel_effective_weight_sum": [float(value) for value in effective_counts],
        "fit_basis": (
            "Gaussian-kernel Nadaraya-Watson residual LUT sampled at quantile support "
            "points with linear runtime interpolation and edge clamping"
        ),
    }


def fit_local_linear_residual_lut(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    support_count: int = 11,
    bandwidth: float,
    min_effective_weight: float = 1.0e-9,
    ridge_alpha: float = 1.0e-9,
) -> dict[str, object]:
    x_values = np.asarray(x_values, dtype=float).reshape(-1)
    y_values = np.asarray(y_values, dtype=float).reshape(-1)
    valid = np.isfinite(x_values) & np.isfinite(y_values)
    x_values = x_values[valid]
    y_values = y_values[valid]
    if x_values.size == 0:
        return {
            "x_values_px": [],
            "residual_values_m": [],
            "fit_basis": "Gaussian-kernel local-linear residual LUT; empty fit",
        }
    order = np.argsort(x_values)
    x_values = x_values[order]
    y_values = y_values[order]
    support_count = max(1, min(int(support_count), int(x_values.size)))
    support_x = np.unique(np.quantile(x_values, np.linspace(0.0, 1.0, support_count)))
    if support_x.size == 0:
        support_x = np.array([float(np.median(x_values))], dtype=float)
    span = float(np.max(x_values) - np.min(x_values))
    if not math.isfinite(bandwidth) or bandwidth <= 0.0:
        bandwidth = max(span / max(1, support_count - 1), 1.0e-6)
    bandwidth = max(float(bandwidth), 1.0e-9)
    global_median = float(np.median(y_values))
    support_y: list[float] = []
    effective_counts: list[float] = []
    local_slopes: list[float] = []
    ridge_alpha = max(0.0, float(ridge_alpha))
    for center in support_x:
        dx = x_values - float(center)
        normalized = dx / bandwidth
        weights = np.exp(-0.5 * np.square(normalized))
        weight_sum = float(np.sum(weights))
        effective_counts.append(weight_sum)
        if weight_sum <= min_effective_weight:
            support_y.append(global_median)
            local_slopes.append(0.0)
            continue
        design = np.column_stack([np.ones_like(dx), dx])
        weighted_design = design * weights[:, None]
        normal_matrix = design.T @ weighted_design
        normal_matrix[1, 1] += ridge_alpha
        rhs = design.T @ (weights * y_values)
        try:
            beta = np.linalg.solve(normal_matrix, rhs)
        except np.linalg.LinAlgError:
            beta = np.linalg.pinv(normal_matrix) @ rhs
        support_y.append(float(beta[0]))
        local_slopes.append(float(beta[1]))
    return {
        "x_values_px": [float(value) for value in support_x],
        "residual_values_m": support_y,
        "local_linear_slopes_m_per_x": local_slopes,
        "kernel_bandwidth": float(bandwidth),
        "kernel_support_count": int(support_x.size),
        "kernel_effective_weight_sum": [float(value) for value in effective_counts],
        "local_linear_ridge_alpha": float(ridge_alpha),
        "fit_basis": (
            "Gaussian-kernel local-linear forward residual regression sampled at quantile "
            "support points with linear runtime interpolation and edge clamping"
        ),
    }


def fit_smoothing_spline_residual_lut(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    support_count: int = 11,
    smoothing_factor_scale: float = 1.0,
) -> dict[str, object]:
    x_values = np.asarray(x_values, dtype=float).reshape(-1)
    y_values = np.asarray(y_values, dtype=float).reshape(-1)
    valid = np.isfinite(x_values) & np.isfinite(y_values)
    x_values = x_values[valid]
    y_values = y_values[valid]
    if x_values.size == 0:
        return {
            "x_values_px": [],
            "residual_values_m": [],
            "fit_basis": "cubic smoothing-spline residual LUT; empty fit",
        }
    order = np.argsort(x_values)
    x_sorted = x_values[order]
    y_sorted = y_values[order]
    unique_x, inverse, counts = np.unique(x_sorted, return_inverse=True, return_counts=True)
    y_sum = np.zeros_like(unique_x, dtype=float)
    np.add.at(y_sum, inverse, y_sorted)
    unique_y = y_sum / counts
    if unique_x.size == 1:
        return {
            "x_values_px": [float(unique_x[0])],
            "residual_values_m": [float(unique_y[0])],
            "spline_degree": 0,
            "spline_smoothing_factor": 0.0,
            "fit_basis": "cubic smoothing-spline residual LUT collapsed to one support point",
        }
    support_count = max(1, min(int(support_count), int(unique_x.size)))
    support_x = np.unique(np.quantile(unique_x, np.linspace(0.0, 1.0, support_count)))
    degree = min(3, int(unique_x.size) - 1)
    residual_variance = float(np.var(unique_y))
    smoothing_factor = max(0.0, float(smoothing_factor_scale)) * float(unique_x.size) * residual_variance
    try:
        spline = UnivariateSpline(unique_x, unique_y, w=np.sqrt(counts), k=degree, s=smoothing_factor)
        support_y = spline(support_x)
    except Exception:
        support_y = np.interp(support_x, unique_x, unique_y)
    return {
        "x_values_px": [float(value) for value in support_x],
        "residual_values_m": [float(value) for value in support_y],
        "spline_degree": int(degree),
        "spline_support_count": int(support_x.size),
        "spline_smoothing_factor": float(smoothing_factor),
        "spline_smoothing_factor_scale": float(smoothing_factor_scale),
        "unique_sample_count": int(unique_x.size),
        "fit_basis": (
            "regularized cubic smoothing-spline forward residual curve sampled as a "
            "piecewise-linear runtime LUT with edge clamping"
        ),
    }


def fit_capped_binned_residual_lut(
    x_values: np.ndarray,
    y_values: np.ndarray,
    *,
    bin_count: int = 7,
    min_samples_per_bin: int = 4,
    mad_sigma: float = 2.5,
    quantile: float = 0.90,
) -> dict[str, object]:
    base_lut = fit_binned_residual_lut(
        x_values,
        y_values,
        bin_count=bin_count,
        min_samples_per_bin=min_samples_per_bin,
    )
    residual_values = np.asarray(base_lut.get("residual_values_m", []), dtype=float)
    sample_residuals = np.asarray(y_values, dtype=float).reshape(-1)
    sample_residuals = sample_residuals[np.isfinite(sample_residuals)]
    if residual_values.size == 0 or sample_residuals.size == 0:
        base_lut["fit_basis"] = "capped quantile-binned median residual LUT; empty fit"
        return base_lut
    center = float(np.median(sample_residuals))
    mad = float(np.median(np.abs(sample_residuals - center)))
    robust_sigma = 1.4826 * mad
    quantile = min(1.0, max(0.5, float(quantile)))
    quantile_cap = float(np.quantile(np.abs(sample_residuals - center), quantile))
    sigma_cap = float(max(0.0, mad_sigma) * robust_sigma)
    positive_caps = [value for value in (quantile_cap, sigma_cap) if math.isfinite(value) and value > 1.0e-12]
    cap = min(positive_caps) if positive_caps else float(np.max(np.abs(sample_residuals - center)))
    capped = np.clip(residual_values, center - cap, center + cap)
    base_lut["residual_values_m"] = [float(value) for value in capped]
    base_lut["raw_residual_values_m"] = [float(value) for value in residual_values]
    base_lut["cap_center_residual_m"] = center
    base_lut["cap_half_width_m"] = float(cap)
    base_lut["cap_mad_sigma"] = float(mad_sigma)
    base_lut["cap_quantile"] = float(quantile)
    base_lut["fit_basis"] = (
        "quantile-binned median residual LUT with robust winsorized cap "
        "from training residual MAD and absolute-residual quantile"
    )
    return base_lut


def relabel_residual_lut_x_values(
    lut: Mapping[str, object],
    *,
    x_key: str,
    fit_basis_suffix: str,
) -> dict[str, object]:
    relabeled = dict(lut)
    if "x_values_px" in relabeled:
        relabeled[x_key] = relabeled.pop("x_values_px")
    basis = str(relabeled.get("fit_basis", "residual LUT"))
    relabeled["fit_basis"] = f"{basis}; x-axis={fit_basis_suffix}"
    return relabeled


def scale_residual_lut(
    lut: Mapping[str, object],
    *,
    residual_scale: float,
    fit_basis_suffix: str,
) -> dict[str, object]:
    scaled = dict(lut)
    scale_value = float(residual_scale)
    residual_values = np.asarray(scaled.get("residual_values_m", []), dtype=float)
    scaled["residual_values_m"] = [float(scale_value * value) for value in residual_values]
    scaled["residual_scale"] = scale_value
    basis = str(scaled.get("fit_basis", "residual LUT"))
    scaled["fit_basis"] = f"{basis}; {fit_basis_suffix}"
    return scaled


def fit_range_normalized_residual_lut(
    x_forward_m: np.ndarray,
    residual_m: np.ndarray,
    *,
    bin_count: int = 7,
    min_samples_per_bin: int = 4,
    prior_strength: float = 8.0,
) -> dict[str, object]:
    x_forward_m = np.asarray(x_forward_m, dtype=float).reshape(-1)
    residual_m = np.asarray(residual_m, dtype=float).reshape(-1)
    valid = np.isfinite(x_forward_m) & np.isfinite(residual_m) & (np.abs(x_forward_m) > 1.0e-6)
    x_valid = x_forward_m[valid]
    residual_valid = residual_m[valid]
    if x_valid.size == 0:
        return {
            "x_values_m": [],
            "residual_values_m": [],
            "bin_sample_counts": [],
            "fit_basis": "range-normalized residual LUT; empty fit",
        }
    normalized_residual = residual_valid / x_valid
    base_lut = fit_binned_residual_lut(
        x_valid,
        normalized_residual,
        bin_count=bin_count,
        min_samples_per_bin=min_samples_per_bin,
    )
    x_lut = np.asarray(base_lut.get("x_values_px", []), dtype=float)
    ratio_lut = np.asarray(base_lut.get("residual_values_m", []), dtype=float)
    counts = np.asarray(base_lut.get("bin_sample_counts", []), dtype=float)
    if x_lut.size == 0 or ratio_lut.size == 0:
        return {
            "x_values_m": [],
            "residual_values_m": [],
            "bin_sample_counts": [],
            "fit_basis": "range-normalized residual LUT; empty fit",
        }
    if counts.size != ratio_lut.size:
        counts = np.ones_like(ratio_lut)
    prior = float(np.median(normalized_residual))
    prior_strength = max(0.0, float(prior_strength))
    shrunk_ratio = (counts * ratio_lut + prior_strength * prior) / (counts + prior_strength)
    residual_lut = shrunk_ratio * x_lut
    return {
        "x_values_m": [float(value) for value in x_lut],
        "residual_values_m": [float(value) for value in residual_lut],
        "normalized_residual_values": [float(value) for value in shrunk_ratio],
        "raw_normalized_residual_values": [float(value) for value in ratio_lut],
        "normalized_residual_prior": prior,
        "prior_strength_samples": float(prior_strength),
        "bin_sample_counts": [int(value) for value in counts],
        "fit_basis": (
            "quantile-binned median residual/forward ratio LUT with empirical-Bayes shrinkage; "
            "exported as meter residuals keyed by raw RDF forward_m"
        ),
    }


def fit_inverse_depth_residual_lut(
    x_key_forward_m: np.ndarray,
    base_forward_m: np.ndarray,
    truth_forward_m: np.ndarray,
    *,
    bin_count: int = 7,
    min_samples_per_bin: int = 4,
    prior_strength: float = 8.0,
) -> dict[str, object]:
    x_key_forward_m = np.asarray(x_key_forward_m, dtype=float).reshape(-1)
    base_forward_m = np.asarray(base_forward_m, dtype=float).reshape(-1)
    truth_forward_m = np.asarray(truth_forward_m, dtype=float).reshape(-1)
    valid = (
        np.isfinite(x_key_forward_m)
        & np.isfinite(base_forward_m)
        & np.isfinite(truth_forward_m)
        & (np.abs(base_forward_m) > 1.0e-6)
        & (np.abs(truth_forward_m) > 1.0e-6)
    )
    x_valid = x_key_forward_m[valid]
    base_valid = base_forward_m[valid]
    truth_valid = truth_forward_m[valid]
    if x_valid.size == 0:
        return {
            "x_values_m": [],
            "residual_values_m": [],
            "bin_sample_counts": [],
            "fit_basis": "inverse-depth forward residual LUT; empty fit",
        }
    order = np.argsort(x_valid)
    x_valid = x_valid[order]
    base_valid = base_valid[order]
    truth_valid = truth_valid[order]
    bin_count = max(1, min(int(bin_count), int(x_valid.size)))
    edges = np.linspace(0, x_valid.size, bin_count + 1, dtype=int)
    x_lut: list[float] = []
    base_lut: list[float] = []
    inv_delta_lut: list[float] = []
    counts: list[int] = []
    inv_delta_samples = (1.0 / truth_valid) - (1.0 / base_valid)
    for start, stop in zip(edges[:-1], edges[1:]):
        if stop <= start:
            continue
        bin_x = x_valid[start:stop]
        bin_base = base_valid[start:stop]
        bin_inv_delta = inv_delta_samples[start:stop]
        if bin_x.size < max(1, min_samples_per_bin) and x_valid.size >= min_samples_per_bin:
            continue
        x_lut.append(float(np.median(bin_x)))
        base_lut.append(float(np.median(bin_base)))
        inv_delta_lut.append(float(np.median(bin_inv_delta)))
        counts.append(int(bin_x.size))
    if not x_lut:
        x_lut = [float(np.median(x_valid))]
        base_lut = [float(np.median(base_valid))]
        inv_delta_lut = [float(np.median(inv_delta_samples))]
        counts = [int(x_valid.size)]
    inv_delta = np.asarray(inv_delta_lut, dtype=float)
    base_values = np.asarray(base_lut, dtype=float)
    count_values = np.asarray(counts, dtype=float)
    prior_strength = max(0.0, float(prior_strength))
    prior = float(np.median(inv_delta_samples))
    shrunk_inv_delta = (count_values * inv_delta + prior_strength * prior) / (count_values + prior_strength)
    corrected_forward = 1.0 / ((1.0 / base_values) + shrunk_inv_delta)
    residual_lut = corrected_forward - base_values
    return {
        "x_values_m": [float(value) for value in x_lut],
        "residual_values_m": [float(value) for value in residual_lut],
        "inverse_depth_delta_values_1_per_m": [float(value) for value in shrunk_inv_delta],
        "raw_inverse_depth_delta_values_1_per_m": [float(value) for value in inv_delta],
        "base_forward_values_m": [float(value) for value in base_values],
        "inverse_depth_delta_prior_1_per_m": prior,
        "prior_strength_samples": float(prior_strength),
        "bin_sample_counts": [int(value) for value in counts],
        "fit_basis": (
            "quantile-binned median inverse-depth delta, fitted as 1/truth_forward - "
            "1/base_forward and exported as meter residuals keyed by raw RDF forward_m. "
            "This follows stereo depth geometry where disparity scale/bias is closer to "
            "linear in inverse depth than in range."
        ),
    }


def fit_monotone_inverse_depth_residual_lut(
    x_key_forward_m: np.ndarray,
    base_forward_m: np.ndarray,
    truth_forward_m: np.ndarray,
    *,
    bin_count: int = 7,
    min_samples_per_bin: int = 4,
    prior_strength: float = 8.0,
) -> dict[str, object]:
    base_lut = fit_inverse_depth_residual_lut(
        x_key_forward_m,
        base_forward_m,
        truth_forward_m,
        bin_count=bin_count,
        min_samples_per_bin=min_samples_per_bin,
        prior_strength=prior_strength,
    )
    inv_delta = np.asarray(base_lut.get("inverse_depth_delta_values_1_per_m", []), dtype=float)
    counts = np.asarray(base_lut.get("bin_sample_counts", []), dtype=float)
    base_values = np.asarray(base_lut.get("base_forward_values_m", []), dtype=float)
    if inv_delta.size == 0:
        base_lut["fit_basis"] = "monotone inverse-depth forward residual LUT; empty fit"
        return base_lut
    if counts.size != inv_delta.size:
        counts = np.ones_like(inv_delta)
    increasing = weighted_pava(inv_delta, counts)
    decreasing = -weighted_pava(-inv_delta, counts)
    inc_loss = float(np.sum(counts * np.square(increasing - inv_delta)))
    dec_loss = float(np.sum(counts * np.square(decreasing - inv_delta)))
    if inc_loss <= dec_loss:
        monotone_delta = increasing
        direction = "nondecreasing"
        projection_loss = inc_loss
    else:
        monotone_delta = decreasing
        direction = "nonincreasing"
        projection_loss = dec_loss
    corrected_forward = 1.0 / ((1.0 / base_values) + monotone_delta)
    residual_lut = corrected_forward - base_values
    base_lut["residual_values_m"] = [float(value) for value in residual_lut]
    base_lut["inverse_depth_delta_values_1_per_m"] = [float(value) for value in monotone_delta]
    base_lut["pre_monotone_inverse_depth_delta_values_1_per_m"] = [float(value) for value in inv_delta]
    base_lut["monotone_direction"] = direction
    base_lut["monotone_projection_loss"] = projection_loss
    base_lut["fit_basis"] = (
        "quantile-binned median inverse-depth delta projected with weighted PAVA isotonic "
        "regression before conversion back to meter forward residuals. The runtime LUT "
        "contract stays raw RDF forward_m -> forward residual_m."
    )
    return base_lut


def fit_polynomial_raw_forward_residual_lut(
    x_forward_m: np.ndarray,
    residual_m: np.ndarray,
    *,
    degree: int = 2,
    ridge_alpha: float = 1.0,
    bin_count: int = 7,
    min_samples_per_bin: int = 4,
    weights: np.ndarray | None = None,
) -> dict[str, object]:
    x_forward_m = np.asarray(x_forward_m, dtype=float).reshape(-1)
    residual_m = np.asarray(residual_m, dtype=float).reshape(-1)
    if weights is None:
        weights_array = np.ones_like(x_forward_m, dtype=float)
    else:
        weights_array = np.asarray(weights, dtype=float).reshape(-1)
        if weights_array.size != x_forward_m.size:
            raise ValueError("weights must have same length as x_forward_m")
    valid = np.isfinite(x_forward_m) & np.isfinite(residual_m) & np.isfinite(weights_array)
    x_valid = x_forward_m[valid]
    residual_valid = residual_m[valid]
    weights_valid = weights_array[valid]
    if x_valid.size == 0:
        return {
            "x_values_m": [],
            "residual_values_m": [],
            "bin_sample_counts": [],
            "fit_basis": "raw-forward polynomial residual LUT; empty fit",
        }
    degree = min(max(0, int(degree)), max(0, int(x_valid.size) - 1))
    if weights is None:
        coefficients = fit_ridge_polyfit(x_valid, residual_valid, degree=degree, alpha=float(ridge_alpha))
    else:
        coefficients = fit_weighted_ridge_polyfit(
            x_valid,
            residual_valid,
            degree=degree,
            alpha=float(ridge_alpha),
            weights=weights_valid,
        )
    support_lut = fit_binned_residual_lut(
        x_valid,
        residual_valid,
        bin_count=bin_count,
        min_samples_per_bin=min_samples_per_bin,
    )
    x_lut = np.asarray(support_lut.get("x_values_px", []), dtype=float)
    if x_lut.size == 0:
        x_lut = np.asarray([float(np.median(x_valid))], dtype=float)
    residual_lut = np.polyval(coefficients, x_lut)
    return {
        "x_values_m": [float(value) for value in x_lut],
        "residual_values_m": [float(value) for value in residual_lut],
        "polynomial_coefficients_high_to_low": [float(value) for value in coefficients],
        "polynomial_degree": int(degree),
        "ridge_alpha": float(ridge_alpha),
        "weighted_fit": bool(weights is not None),
        "mean_weight": float(np.mean(weights_valid)) if weights is not None else 1.0,
        "bin_sample_counts": [int(value) for value in support_lut.get("bin_sample_counts", [])],
        "fit_basis": (
            "ridge-regularized polynomial forward residual over raw RDF forward_m; "
            "exported as a piecewise-linear meter residual LUT. Degree 2 matches the "
            "first-order stereo depth error from disparity bias, where range error scales with Z^2."
        ),
    }


def fit_polynomial_plus_binned_raw_forward_residual_lut(
    x_forward_m: np.ndarray,
    residual_m: np.ndarray,
    *,
    degree: int = 2,
    ridge_alpha: float = 10.0,
    bin_count: int = 7,
    min_samples_per_bin: int = 4,
    residual_shrink: float = 0.5,
) -> dict[str, object]:
    x_forward_m = np.asarray(x_forward_m, dtype=float).reshape(-1)
    residual_m = np.asarray(residual_m, dtype=float).reshape(-1)
    valid = np.isfinite(x_forward_m) & np.isfinite(residual_m)
    x_valid = x_forward_m[valid]
    residual_valid = residual_m[valid]
    if x_valid.size == 0:
        return {
            "x_values_m": [],
            "residual_values_m": [],
            "bin_sample_counts": [],
            "fit_basis": "raw-forward polynomial plus local residual LUT; empty fit",
        }
    degree = min(max(0, int(degree)), max(0, int(x_valid.size) - 1))
    coefficients = fit_ridge_polyfit(x_valid, residual_valid, degree=degree, alpha=float(ridge_alpha))
    polynomial_prediction = np.polyval(coefficients, x_valid)
    local_residual = residual_valid - polynomial_prediction
    local_lut = fit_binned_residual_lut(
        x_valid,
        local_residual,
        bin_count=bin_count,
        min_samples_per_bin=min_samples_per_bin,
    )
    x_lut = np.asarray(local_lut.get("x_values_px", []), dtype=float)
    local_values = np.asarray(local_lut.get("residual_values_m", []), dtype=float)
    if x_lut.size == 0:
        x_lut = np.asarray([float(np.median(x_valid))], dtype=float)
        local_values = np.asarray([float(np.median(local_residual))], dtype=float)
    residual_shrink = min(1.0, max(0.0, float(residual_shrink)))
    polynomial_lut = np.polyval(coefficients, x_lut)
    combined_lut = polynomial_lut + residual_shrink * local_values
    return {
        "x_values_m": [float(value) for value in x_lut],
        "residual_values_m": [float(value) for value in combined_lut],
        "polynomial_residual_values_m": [float(value) for value in polynomial_lut],
        "local_residual_values_m": [float(value) for value in local_values],
        "local_residual_shrink": float(residual_shrink),
        "polynomial_coefficients_high_to_low": [float(value) for value in coefficients],
        "polynomial_degree": int(degree),
        "ridge_alpha": float(ridge_alpha),
        "bin_sample_counts": [int(value) for value in local_lut.get("bin_sample_counts", [])],
        "fit_basis": (
            "ridge-regularized polynomial forward residual over raw RDF forward_m plus "
            "shrunk quantile-binned median residual-of-residual LUT. The polynomial captures "
            "stereo depth bias geometry and the local LUT captures remaining calibration structure."
        ),
    }


def apply_forward_residual_lut(points: np.ndarray, d_meas: np.ndarray, lut: Mapping[str, object]) -> np.ndarray:
    x_values = np.asarray(lut.get("x_values_px", lut.get("x_values_m", [])), dtype=float)
    residual_values = np.asarray(lut.get("residual_values_m", []), dtype=float)
    corrected = np.array(points, dtype=float, copy=True)
    if x_values.size == 0 or residual_values.size == 0 or x_values.size != residual_values.size:
        return corrected
    if x_values.size == 1:
        residual = np.full_like(np.asarray(d_meas, dtype=float), float(residual_values[0]), dtype=float)
    else:
        order = np.argsort(x_values)
        x_sorted = x_values[order]
        y_sorted = residual_values[order]
        residual = np.interp(np.asarray(d_meas, dtype=float), x_sorted, y_sorted, left=y_sorted[0], right=y_sorted[-1])
    corrected[:, 2] += residual
    return corrected


def interpolated_forward_residual_lut(x_query: np.ndarray, lut: Mapping[str, object]) -> np.ndarray:
    x_values = np.asarray(lut.get("x_values_px", lut.get("x_values_m", [])), dtype=float)
    residual_values = np.asarray(lut.get("residual_values_m", []), dtype=float)
    query = np.asarray(x_query, dtype=float)
    if x_values.size == 0 or residual_values.size == 0 or x_values.size != residual_values.size:
        return np.zeros_like(query, dtype=float)
    if x_values.size == 1:
        return np.full_like(query, float(residual_values[0]), dtype=float)
    order = np.argsort(x_values)
    x_sorted = x_values[order]
    y_sorted = residual_values[order]
    return np.interp(query, x_sorted, y_sorted, left=y_sorted[0], right=y_sorted[-1])


def apply_agreement_gated_forward_residual_luts(
    points: np.ndarray,
    d_meas: np.ndarray,
    raw_forward_m: np.ndarray,
    disparity_lut: Mapping[str, object],
    raw_forward_lut: Mapping[str, object],
    *,
    max_abs_difference_m: float,
    alpha_disagree: float,
) -> np.ndarray:
    disparity_residual = interpolated_forward_residual_lut(d_meas, disparity_lut)
    raw_forward_residual = interpolated_forward_residual_lut(raw_forward_m, raw_forward_lut)
    difference = np.abs(disparity_residual - raw_forward_residual)
    alpha = np.where(
        difference > max(0.0, float(max_abs_difference_m)),
        min(1.0, max(0.0, float(alpha_disagree))),
        1.0,
    )
    corrected = np.array(points, dtype=float, copy=True)
    corrected[:, 2] += alpha * (disparity_residual + raw_forward_residual)
    return corrected


def apply_rdf_residual_lut(points: np.ndarray, d_meas: np.ndarray, lut_by_axis: Mapping[str, Mapping[str, object]]) -> np.ndarray:
    corrected = np.array(points, dtype=float, copy=True)
    for column, axis in enumerate(("right", "down", "forward")):
        lut = lut_by_axis.get(axis)
        if not lut:
            continue
        axis_corrected = apply_forward_residual_lut(
            np.column_stack(
                (
                    np.zeros(corrected.shape[0], dtype=float),
                    np.zeros(corrected.shape[0], dtype=float),
                    corrected[:, column],
                )
            ),
            d_meas,
            lut,
        )
        corrected[:, column] = axis_corrected[:, 2]
    return corrected


def load_local_sweep_configs(config_dir: Path | None) -> list[dict[str, object]]:
    if config_dir is None:
        return []
    if not config_dir.is_dir():
        raise ValueError(f"--local-sweep-config-dir is not a directory: {config_dir}")
    configs: list[dict[str, object]] = []
    for path in sorted(config_dir.glob("*.json")):
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError(f"local sweep config must be a JSON object: {path}")
        payload = dict(payload)
        payload["_config_path"] = str(path)
        configs.append(payload)
    return configs


def local_sweep_config_supported(config: Mapping[str, object]) -> tuple[bool, str]:
    if str(config.get("mode", "")) != "rtk_stereo_local_sweep_candidate":
        return False, "mode_not_rtk_stereo_local_sweep_candidate"
    if bool(config.get("runtime_truth_used", False)):
        return False, "runtime_truth_used_true"
    if str(config.get("registration_model", "")) != "sim3":
        return False, "unsupported_registration_model"
    if str(config.get("forward_key", "")) not in {"raw_forward", "base_forward"}:
        return False, "unsupported_forward_key"
    if str(config.get("robust_loss", "none")) not in {"none", "huber", "cauchy"}:
        return False, "unsupported_robust_loss"
    if str(config.get("residual_basis", "")) not in {
        "inverse_depth",
        "range_normalized",
        "meter",
        "blended_disparity_inverse_depth",
    }:
        return False, "unsupported_residual_basis"
    return True, "native_pipeline"


def local_config_float(config: Mapping[str, object], name: str, default: float) -> float:
    value = finite_float(config.get(name))
    return default if value is None else float(value)


def local_config_int(config: Mapping[str, object], name: str, default: int) -> int:
    value = finite_float(config.get(name))
    return default if value is None else int(round(float(value)))


def local_robust_weights(residual: np.ndarray, robust_loss: str) -> tuple[np.ndarray, dict[str, object]]:
    residual = np.asarray(residual, dtype=float).reshape(-1)
    finite = residual[np.isfinite(residual)]
    if robust_loss == "none" or finite.size == 0:
        return np.ones_like(residual, dtype=float), {"robust_loss": "none", "scale_m": 0.0}
    median = float(np.median(finite))
    mad = float(np.median(np.abs(finite - median)))
    scale = max(1.4826 * mad, float(np.std(finite)), 1.0e-9)
    normalized = np.abs(residual - median) / scale
    if robust_loss == "huber":
        delta = 1.345
        weights = np.ones_like(residual, dtype=float)
        large = normalized > delta
        weights[large] = delta / np.maximum(normalized[large], 1.0e-12)
        return weights, {"robust_loss": "huber", "scale_m": scale, "delta": delta}
    if robust_loss == "cauchy":
        c = 2.3849
        weights = 1.0 / (1.0 + np.square(normalized / c))
        return weights, {"robust_loss": "cauchy", "scale_m": scale, "c": c}
    raise ValueError(f"unsupported robust_loss: {robust_loss}")


def append_local_sweep_methods(
    methods: list[FittedMethod],
    *,
    configs: Sequence[Mapping[str, object]],
    raw_train: np.ndarray,
    truth_train: np.ndarray,
    d_meas: np.ndarray,
    sim_scale: float,
    sim_rotation: Rotation,
    sim_translation: np.ndarray,
    sim_train: np.ndarray,
    forward_residual: np.ndarray,
    target_weights: np.ndarray,
) -> list[dict[str, object]]:
    status_rows: list[dict[str, object]] = []
    existing_names = {method.name for method in methods}
    for config in configs:
        method_name = str(config.get("method", "")).strip()
        supported, reason = local_sweep_config_supported(config)
        row = {
            "method": method_name,
            "config_path": config.get("_config_path", ""),
            "status": "PENDING",
            "reason": reason,
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": STEREODISTANCE_PUBLIC_TOPIC_CONTRACT,
            "runtime_truth_used": False,
        }
        if not method_name:
            row.update(status="SKIP", reason="missing_method")
            status_rows.append(row)
            continue
        if method_name in existing_names:
            row.update(status="SKIP", reason="method_already_exists")
            status_rows.append(row)
            continue
        if not supported:
            row.update(status="SKIP")
            status_rows.append(row)
            continue

        bin_count = local_config_int(config, "bin_count", 11)
        prior_strength = local_config_float(config, "prior_strength_samples", 8.0)
        residual_gain = local_config_float(config, "residual_gain", 1.0)
        target_weight = max(0.0, local_config_float(config, "target_weight", 0.0))
        residual_basis = str(config.get("residual_basis", "inverse_depth"))
        robust_loss = str(config.get("robust_loss", "none"))
        forward_key = str(config.get("forward_key", "raw_forward"))
        monotone = bool(config.get("monotone", False))
        x_key = raw_train[:, 2] if forward_key == "raw_forward" else sim_train[:, 2]
        robust_weights, robust_info = local_robust_weights(forward_residual, robust_loss)
        fit_weights = (1.0 + target_weight * np.asarray(target_weights, dtype=float)) * robust_weights
        weighted_fit = target_weight > 0.0 or robust_loss != "none"

        disp_lut: Mapping[str, object] | None = None
        if residual_basis == "inverse_depth":
            if weighted_fit:
                inv_delta = (1.0 / truth_train[:, 2]) - (1.0 / sim_train[:, 2])
                base_lut = fit_weighted_binned_residual_lut(
                    x_key,
                    inv_delta,
                    fit_weights,
                    bin_count=bin_count,
                )
                x_lut = np.asarray(base_lut.get("x_values_px", []), dtype=float)
                inv_delta_lut = np.asarray(base_lut.get("residual_values_m", []), dtype=float)
                counts = np.asarray(base_lut.get("bin_weight_sums", base_lut.get("bin_sample_counts", [])), dtype=float)
                base_forward_lut = np.asarray(
                    [
                        weighted_quantile(sim_train[:, 2][np.argsort(np.abs(x_key - x_value))[: max(1, min(12, x_key.size))]], np.ones(max(1, min(12, x_key.size))), 0.5)
                        for x_value in x_lut
                    ],
                    dtype=float,
                )
                if counts.size != inv_delta_lut.size:
                    counts = np.ones_like(inv_delta_lut)
                prior = weighted_quantile(inv_delta, fit_weights, 0.5)
                shrunk = (counts * inv_delta_lut + prior_strength * prior) / (counts + prior_strength)
                if monotone and shrunk.size:
                    increasing = weighted_pava(shrunk, counts)
                    decreasing = -weighted_pava(-shrunk, counts)
                    shrunk = increasing if float(np.sum(counts * np.square(increasing - shrunk))) <= float(np.sum(counts * np.square(decreasing - shrunk))) else decreasing
                corrected_forward = 1.0 / ((1.0 / base_forward_lut) + shrunk)
                lut = {
                    "x_values_m": [float(value) for value in x_lut],
                    "residual_values_m": [float(value) for value in corrected_forward - base_forward_lut],
                    "inverse_depth_delta_values_1_per_m": [float(value) for value in shrunk],
                    "base_forward_values_m": [float(value) for value in base_forward_lut],
                    "prior_strength_samples": float(prior_strength),
                    "target_weight": float(target_weight),
                    "robust_fit": robust_info,
                    "bin_weight_sums": [float(value) for value in counts],
                    "fit_basis": "target-weighted local sweep inverse-depth forward residual LUT",
                }
            else:
                lut = (
                    fit_monotone_inverse_depth_residual_lut(
                        x_key,
                        sim_train[:, 2],
                        truth_train[:, 2],
                        bin_count=bin_count,
                        prior_strength=prior_strength,
                    )
                    if monotone
                    else fit_inverse_depth_residual_lut(
                        x_key,
                        sim_train[:, 2],
                        truth_train[:, 2],
                        bin_count=bin_count,
                        prior_strength=prior_strength,
                    )
                )
        elif residual_basis == "range_normalized":
            if weighted_fit:
                normalized = forward_residual / np.where(np.abs(x_key) > 1.0e-6, x_key, np.nan)
                base_lut = fit_weighted_binned_residual_lut(x_key, normalized, fit_weights, bin_count=bin_count)
                x_lut = np.asarray(base_lut.get("x_values_px", []), dtype=float)
                ratio = np.asarray(base_lut.get("residual_values_m", []), dtype=float)
                counts = np.asarray(base_lut.get("bin_weight_sums", base_lut.get("bin_sample_counts", [])), dtype=float)
                if counts.size != ratio.size:
                    counts = np.ones_like(ratio)
                prior = weighted_quantile(normalized, fit_weights, 0.5)
                shrunk = (counts * ratio + prior_strength * prior) / (counts + prior_strength)
                if monotone and shrunk.size:
                    increasing = weighted_pava(shrunk, counts)
                    decreasing = -weighted_pava(-shrunk, counts)
                    shrunk = increasing if float(np.sum(counts * np.square(increasing - shrunk))) <= float(np.sum(counts * np.square(decreasing - shrunk))) else decreasing
                lut = {
                    "x_values_m": [float(value) for value in x_lut],
                    "residual_values_m": [float(value) for value in shrunk * x_lut],
                    "normalized_residual_values": [float(value) for value in shrunk],
                    "target_weight": float(target_weight),
                    "robust_fit": robust_info,
                    "bin_weight_sums": [float(value) for value in counts],
                    "fit_basis": "target-weighted local sweep range-normalized residual LUT",
                }
            else:
                lut = fit_range_normalized_residual_lut(
                    x_key,
                    forward_residual,
                    bin_count=bin_count,
                    prior_strength=prior_strength,
                )
                if monotone:
                    ratio = np.asarray(lut.get("normalized_residual_values", []), dtype=float)
                    counts = np.asarray(lut.get("bin_sample_counts", []), dtype=float)
                    x_lut = np.asarray(lut.get("x_values_m", []), dtype=float)
                    if ratio.size and x_lut.size == ratio.size:
                        if counts.size != ratio.size:
                            counts = np.ones_like(ratio)
                        increasing = weighted_pava(ratio, counts)
                        decreasing = -weighted_pava(-ratio, counts)
                        monotone_ratio = increasing if float(np.sum(counts * np.square(increasing - ratio))) <= float(np.sum(counts * np.square(decreasing - ratio))) else decreasing
                        lut["normalized_residual_values"] = [float(value) for value in monotone_ratio]
                        lut["pre_monotone_normalized_residual_values"] = [float(value) for value in ratio]
                        lut["residual_values_m"] = [float(value) for value in monotone_ratio * x_lut]
                        lut["fit_basis"] = f"{lut.get('fit_basis', 'range-normalized residual LUT')}; monotone ratio projection"
        elif residual_basis == "meter":
            base_lut = (
                fit_monotone_binned_residual_lut(x_key, forward_residual, bin_count=bin_count)
                if monotone and not weighted_fit
                else fit_weighted_binned_residual_lut(x_key, forward_residual, fit_weights, bin_count=bin_count)
                if weighted_fit
                else fit_binned_residual_lut(x_key, forward_residual, bin_count=bin_count)
            )
            lut = relabel_residual_lut_x_values(
                base_lut,
                x_key="x_values_m",
                fit_basis_suffix=f"{forward_key} meter residual",
            )
        else:
            if weighted_fit:
                inv_delta = (1.0 / truth_train[:, 2]) - (1.0 / sim_train[:, 2])
                base_lut = fit_weighted_binned_residual_lut(x_key, inv_delta, fit_weights, bin_count=bin_count)
                x_lut = np.asarray(base_lut.get("x_values_px", []), dtype=float)
                inv_delta_lut = np.asarray(base_lut.get("residual_values_m", []), dtype=float)
                counts = np.asarray(base_lut.get("bin_weight_sums", base_lut.get("bin_sample_counts", [])), dtype=float)
                base_forward_lut = np.asarray(
                    [
                        weighted_quantile(sim_train[:, 2][np.argsort(np.abs(x_key - x_value))[: max(1, min(12, x_key.size))]], np.ones(max(1, min(12, x_key.size))), 0.5)
                        for x_value in x_lut
                    ],
                    dtype=float,
                )
                if counts.size != inv_delta_lut.size:
                    counts = np.ones_like(inv_delta_lut)
                prior = weighted_quantile(inv_delta, fit_weights, 0.5)
                shrunk = (counts * inv_delta_lut + prior_strength * prior) / (counts + prior_strength)
                if monotone and shrunk.size:
                    increasing = weighted_pava(shrunk, counts)
                    decreasing = -weighted_pava(-shrunk, counts)
                    shrunk = increasing if float(np.sum(counts * np.square(increasing - shrunk))) <= float(np.sum(counts * np.square(decreasing - shrunk))) else decreasing
                corrected_forward = 1.0 / ((1.0 / base_forward_lut) + shrunk)
                lut = {
                    "x_values_m": [float(value) for value in x_lut],
                    "residual_values_m": [float(value) for value in 0.5 * (corrected_forward - base_forward_lut)],
                    "inverse_depth_delta_values_1_per_m": [float(value) for value in shrunk],
                    "base_forward_values_m": [float(value) for value in base_forward_lut],
                    "prior_strength_samples": float(prior_strength),
                    "target_weight": float(target_weight),
                    "robust_fit": robust_info,
                    "bin_weight_sums": [float(value) for value in counts],
                    "fit_basis": "weighted local sweep blended inverse-depth share",
                }
            else:
                lut = (
                    fit_monotone_inverse_depth_residual_lut(
                        x_key,
                        sim_train[:, 2],
                        truth_train[:, 2],
                        bin_count=bin_count,
                        prior_strength=prior_strength,
                    )
                    if monotone
                    else fit_inverse_depth_residual_lut(
                        x_key,
                        sim_train[:, 2],
                        truth_train[:, 2],
                        bin_count=bin_count,
                        prior_strength=prior_strength,
                    )
                )
                values = np.asarray(lut.get("residual_values_m", []), dtype=float)
                if values.size:
                    lut["residual_values_m"] = [float(0.5 * value) for value in values]
            values = np.asarray(lut.get("residual_values_m", []), dtype=float)
            disp_lut = scale_residual_lut(
                fit_weighted_binned_residual_lut(d_meas, forward_residual, fit_weights, bin_count=bin_count)
                if weighted_fit
                else fit_binned_residual_lut(d_meas, forward_residual, bin_count=bin_count),
                residual_scale=0.5,
                fit_basis_suffix="local sweep blended disparity/inverse-depth share",
            )

        if abs(residual_gain - 1.0) > 1.0e-12:
            lut = scale_residual_lut(
                lut,
                residual_scale=residual_gain,
                fit_basis_suffix=f"local sweep residual gain {residual_gain:g}",
            )

        parameters: dict[str, object] = {
            "sim3_scale": sim_scale,
            "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
            "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
            "sim3_translation_rdf_m": sim_translation.tolist(),
            "forward_residual_raw_forward_lut": lut,
            "residual_lut_bin_count": bin_count,
            "local_sweep_config": {key: value for key, value in config.items() if not str(key).startswith("_")},
            "local_sweep_forward_key": forward_key,
            "local_sweep_target_weight": target_weight,
            "local_sweep_robust_fit": robust_info,
            "reference_basis": (
                "Local sweep Sim(3)+forward residual candidate fitted offline from PX4/RTK truth. "
                "Runtime apply uses raw stereo-center RDF/disparity and fixed exported parameters only."
            ),
        }
        if disp_lut is not None:
            parameters["forward_residual_disparity_lut"] = disp_lut

        def predict_local(
            samples: Sequence[CalibrationSample],
            *,
            sim_scale: float = sim_scale,
            sim_rotation: Rotation = sim_rotation,
            sim_translation: np.ndarray = sim_translation,
            lut: Mapping[str, object] = lut,
            disp_lut: Mapping[str, object] | None = disp_lut,
            forward_key: str = forward_key,
        ) -> np.ndarray:
            sim_points = predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation)
            query = stack(samples, "raw_rdf_m")[:, 2] if forward_key == "raw_forward" else sim_points[:, 2]
            corrected = apply_forward_residual_lut(sim_points, query, lut)
            if disp_lut is not None:
                corrected = apply_forward_residual_lut(corrected, measured_disparity(samples), disp_lut)
            return corrected

        disp_complexity = len(disp_lut.get("x_values_px", [])) if disp_lut is not None else 0
        methods.append(
            FittedMethod(
                name=method_name,
                predict=predict_local,
                parameters=parameters,
                model_complexity=8 + len(lut.get("x_values_m", [])) + disp_complexity,
            )
        )
        existing_names.add(method_name)
        row.update(status="ADDED", reason="native_pipeline")
        status_rows.append(row)
    return status_rows


def fit_methods(
    train_samples: Sequence[CalibrationSample],
    *,
    camera: CameraModel,
    target_range_m: float = DEFAULT_TARGET_RANGE_M,
    local_sweep_configs: Sequence[Mapping[str, object]] = (),
    local_sweep_status_rows: list[dict[str, object]] | None = None,
) -> list[FittedMethod]:
    raw_train = stack(train_samples, "raw_rdf_m")
    truth_train = stack(train_samples, "truth_rdf_m")
    d_meas = measured_disparity(train_samples)

    methods: list[FittedMethod] = [
        FittedMethod(
            name="raw",
            predict=lambda samples: stack(samples, "raw_rdf_m"),
            parameters={"contract": RDF_AXIS_CONTRACT},
            model_complexity=0,
        )
    ]

    rotation = fit_kabsch_rotation(raw_train, truth_train)
    methods.append(
        FittedMethod(
            name="global_rotation",
            predict=lambda samples, rotation=rotation: apply_rotation_to_rows(stack(samples, "raw_rdf_m"), rotation),
            parameters={
                "scale": 1.0,
                "rotation_euler_zyx_deg": rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": rotation.as_matrix().tolist(),
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "reference_basis": "Kabsch rotation-only estimated-to-truth calibration exported as zero-translation Sim(3)",
            },
            model_complexity=3,
        )
    )

    scale_value, scaled_rotation = fit_scaled_rotation(raw_train, truth_train)
    methods.append(
        FittedMethod(
            name="global_scale_rotation",
            predict=lambda samples, scale_value=scale_value, scaled_rotation=scaled_rotation: (
                scale_value * apply_rotation_to_rows(stack(samples, "raw_rdf_m"), scaled_rotation)
            ),
            parameters={
                "scale": scale_value,
                "rotation_euler_zyx_deg": scaled_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": scaled_rotation.as_matrix().tolist(),
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "reference_basis": "scaled Kabsch rotation estimated-to-truth calibration exported as zero-translation Sim(3)",
            },
            model_complexity=4,
        )
    )

    sim_scale, sim_rotation, sim_translation = fit_umeyama_sim3(raw_train, truth_train)
    methods.append(
        FittedMethod(
            name="sim3",
            predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation: (
                predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation)
            ),
            parameters={
                "scale": sim_scale,
                "rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": sim_translation.tolist(),
            },
            model_complexity=7,
        )
    )

    disparity_forward_product = fit_disparity_forward_product(train_samples)
    product_scaled_train = predict_disparity_product_ray_scaled(train_samples, disparity_forward_product)
    methods.append(
        FittedMethod(
            name="disparity_product_ray_scale",
            predict=lambda samples, disparity_forward_product=disparity_forward_product: (
                predict_disparity_product_ray_scaled(samples, disparity_forward_product)
            ),
            parameters={
                "calibrated_disparity_forward_product_px_m": disparity_forward_product,
                "axis_contract": RDF_AXIS_CONTRACT,
                "reference_basis": (
                    "Stereo depth geometry Z=fB/d; preserve raw stereo-center RDF bearing and "
                    "replace forward with calibrated median(d_measured * truth_forward) / d_measured."
                ),
            },
            model_complexity=2,
        )
    )

    product_sim_scale, product_sim_rotation, product_sim_translation = fit_umeyama_sim3(
        product_scaled_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="disparity_product_ray_scale_sim3",
            predict=lambda samples, disparity_forward_product=disparity_forward_product, product_sim_scale=product_sim_scale, product_sim_rotation=product_sim_rotation, product_sim_translation=product_sim_translation: (
                predict_sim3(
                    predict_disparity_product_ray_scaled(samples, disparity_forward_product),
                    product_sim_scale,
                    product_sim_rotation,
                    product_sim_translation,
                )
            ),
            parameters={
                "calibrated_disparity_forward_product_px_m": disparity_forward_product,
                "raw_rdf_ray_disparity_forward_product_px_m": disparity_forward_product,
                "scale": product_sim_scale,
                "rotation_euler_zyx_deg": product_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": product_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": product_sim_translation.tolist(),
                "axis_contract": RDF_AXIS_CONTRACT,
                "reference_basis": (
                    "Calibrated disparity-forward product ray scaling followed by Umeyama Sim(3) "
                    "estimated-to-truth alignment in stereo-center RDF."
                ),
            },
            model_complexity=9,
        )
    )

    left_geom_model, left_geom_info = fit_nonlinear_left_disparity_camera_geometry(
        train_samples,
        camera=camera,
        loss="huber",
        f_scale_m=1.5,
    )
    left_geom_train = predict_left_disparity_camera_geometry(
        train_samples,
        camera=camera,
        model=left_geom_model,
    )
    left_geom_scale, left_geom_rotation, left_geom_translation = fit_umeyama_sim3(left_geom_train, truth_train)
    methods.append(
        FittedMethod(
            name="huber_left_disparity_camera_geometry_sim3",
            predict=lambda samples, left_geom_model=left_geom_model, left_geom_scale=left_geom_scale, left_geom_rotation=left_geom_rotation, left_geom_translation=left_geom_translation: (
                predict_sim3(
                    predict_left_disparity_camera_geometry(samples, camera=camera, model=left_geom_model),
                    left_geom_scale,
                    left_geom_rotation,
                    left_geom_translation,
                )
            ),
            parameters={
                "scale": left_geom_scale,
                "rotation_euler_zyx_deg": left_geom_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": left_geom_rotation.as_matrix().tolist(),
                "translation_rdf_m": left_geom_translation.tolist(),
                "left_disparity_triangulation_model": left_geom_model,
                "left_disparity_camera_geometry_fit": left_geom_info,
                "axis_contract": RDF_AXIS_CONTRACT,
                "reference_basis": (
                    "Huber nonlinear least-squares fit of left-image center offsets and corrected disparity "
                    "for stereo-center RDF triangulation, followed by Umeyama Sim(3) estimated-to-truth alignment. "
                    "Runtime uses only left bbox center, measured disparity, and calibration-fixed parameters."
                ),
            },
            model_complexity=10,
        )
    )
    left_geom_sim3_train = predict_sim3(
        left_geom_train,
        left_geom_scale,
        left_geom_rotation,
        left_geom_translation,
    )
    left_geom_quadratic_raw_forward_lut = fit_polynomial_raw_forward_residual_lut(
        raw_train[:, 2],
        truth_train[:, 2] - left_geom_sim3_train[:, 2],
        degree=2,
        ridge_alpha=100.0,
    )
    methods.append(
        FittedMethod(
            name="huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut",
            predict=lambda samples, left_geom_model=left_geom_model, left_geom_scale=left_geom_scale, left_geom_rotation=left_geom_rotation, left_geom_translation=left_geom_translation, left_geom_quadratic_raw_forward_lut=left_geom_quadratic_raw_forward_lut: (
                apply_forward_residual_lut(
                    predict_sim3(
                        predict_left_disparity_camera_geometry(samples, camera=camera, model=left_geom_model),
                        left_geom_scale,
                        left_geom_rotation,
                        left_geom_translation,
                    ),
                    stack(samples, "raw_rdf_m")[:, 2],
                    left_geom_quadratic_raw_forward_lut,
                )
            ),
            parameters={
                "scale": left_geom_scale,
                "rotation_euler_zyx_deg": left_geom_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": left_geom_rotation.as_matrix().tolist(),
                "translation_rdf_m": left_geom_translation.tolist(),
                "left_disparity_triangulation_model": left_geom_model,
                "forward_residual_raw_forward_lut": left_geom_quadratic_raw_forward_lut,
                "left_disparity_camera_geometry_fit": left_geom_info,
                "axis_contract": RDF_AXIS_CONTRACT,
                "reference_basis": (
                    "Huber left-image center/disparity triangulation plus Sim(3), with a ridge-regularized "
                    "quadratic raw-forward residual LUT. Runtime uses left bbox center, measured disparity, "
                    "raw_forward_m, and calibration-fixed parameters only."
                ),
            },
            model_complexity=13,
        )
    )
    for left_geom_residual_shrink in (0.25, 0.5, 0.75):
        left_geom_shrink_lut = scale_residual_lut(
            left_geom_quadratic_raw_forward_lut,
            residual_scale=left_geom_residual_shrink,
            fit_basis_suffix=(
                "unweighted left-disparity geometry residual shrinkage for validation-stable deployment; "
                "runtime x-axis remains raw RDF forward_m"
            ),
        )
        left_geom_shrink_label = f"shrink{int(round(left_geom_residual_shrink * 100)):03d}"
        methods.append(
            FittedMethod(
                name=(
                    "huber_left_disparity_camera_geometry_sim3_"
                    f"quadratic_residual_raw_forward_lut_{left_geom_shrink_label}"
                ),
                predict=lambda samples, left_geom_model=left_geom_model, left_geom_scale=left_geom_scale, left_geom_rotation=left_geom_rotation, left_geom_translation=left_geom_translation, left_geom_shrink_lut=left_geom_shrink_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(
                            predict_left_disparity_camera_geometry(samples, camera=camera, model=left_geom_model),
                            left_geom_scale,
                            left_geom_rotation,
                            left_geom_translation,
                        ),
                        stack(samples, "raw_rdf_m")[:, 2],
                        left_geom_shrink_lut,
                    )
                ),
                parameters={
                    "scale": left_geom_scale,
                    "rotation_euler_zyx_deg": left_geom_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": left_geom_rotation.as_matrix().tolist(),
                    "translation_rdf_m": left_geom_translation.tolist(),
                    "left_disparity_triangulation_model": left_geom_model,
                    "forward_residual_raw_forward_lut": left_geom_shrink_lut,
                    "left_disparity_camera_geometry_fit": left_geom_info,
                    "residual_shrink": left_geom_residual_shrink,
                    "axis_contract": RDF_AXIS_CONTRACT,
                    "reference_basis": (
                        "Huber left-image center/disparity triangulation plus Sim(3), with a shrunk "
                        "ridge-regularized quadratic raw-forward residual LUT. Runtime uses left bbox "
                        "center, measured disparity, raw_forward_m, and calibration-fixed parameters only."
                    ),
                },
                model_complexity=13,
            )
        )

    reprojection_left_geom_model, reprojection_left_geom_info = fit_reprojection_left_disparity_camera_geometry(
        train_samples,
        camera=camera,
        loss="huber",
        f_scale_px=1.5,
        disparity_sigma_px=1.0,
        bbox_sigma_px=1.0,
    )
    reprojection_left_geom_train = predict_left_disparity_camera_geometry(
        train_samples,
        camera=camera,
        model=reprojection_left_geom_model,
    )
    reprojection_left_geom_scale, reprojection_left_geom_rotation, reprojection_left_geom_translation = fit_umeyama_sim3(
        reprojection_left_geom_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="huber_left_reprojection_geometry_sim3",
            predict=lambda samples, reprojection_left_geom_model=reprojection_left_geom_model, reprojection_left_geom_scale=reprojection_left_geom_scale, reprojection_left_geom_rotation=reprojection_left_geom_rotation, reprojection_left_geom_translation=reprojection_left_geom_translation: (
                predict_sim3(
                    predict_left_disparity_camera_geometry(samples, camera=camera, model=reprojection_left_geom_model),
                    reprojection_left_geom_scale,
                    reprojection_left_geom_rotation,
                    reprojection_left_geom_translation,
                )
            ),
            parameters={
                "scale": reprojection_left_geom_scale,
                "rotation_euler_zyx_deg": reprojection_left_geom_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": reprojection_left_geom_rotation.as_matrix().tolist(),
                "translation_rdf_m": reprojection_left_geom_translation.tolist(),
                "left_disparity_triangulation_model": reprojection_left_geom_model,
                "left_reprojection_geometry_fit": reprojection_left_geom_info,
                "axis_contract": RDF_AXIS_CONTRACT,
                "reference_basis": (
                    "Huber nonlinear least-squares fit of left-image center offsets and corrected disparity "
                    "by minimizing truth-to-observation reprojection residuals in pixel/disparity space, "
                    "followed by Umeyama Sim(3) estimated-to-truth alignment. Runtime uses only left bbox "
                    "center, measured disparity, and calibration-fixed parameters."
                ),
            },
            model_complexity=10,
        )
    )

    target_left_geom_weights = target_range_weights(train_samples, target_range_m=target_range_m, sigma_m=160.0)
    target_left_geom_model, target_left_geom_info = fit_nonlinear_left_disparity_camera_geometry(
        train_samples,
        camera=camera,
        weights=target_left_geom_weights,
        loss="huber",
        f_scale_m=1.5,
    )
    target_left_geom_train = predict_left_disparity_camera_geometry(
        train_samples,
        camera=camera,
        model=target_left_geom_model,
    )
    target_left_geom_scale, target_left_geom_rotation, target_left_geom_translation = fit_umeyama_sim3(
        target_left_geom_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="target_weighted_huber_left_disparity_camera_geometry_sim3_sigma160",
            predict=lambda samples, target_left_geom_model=target_left_geom_model, target_left_geom_scale=target_left_geom_scale, target_left_geom_rotation=target_left_geom_rotation, target_left_geom_translation=target_left_geom_translation: (
                predict_sim3(
                    predict_left_disparity_camera_geometry(samples, camera=camera, model=target_left_geom_model),
                    target_left_geom_scale,
                    target_left_geom_rotation,
                    target_left_geom_translation,
                )
            ),
            parameters={
                "scale": target_left_geom_scale,
                "rotation_euler_zyx_deg": target_left_geom_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": target_left_geom_rotation.as_matrix().tolist(),
                "translation_rdf_m": target_left_geom_translation.tolist(),
                "left_disparity_triangulation_model": target_left_geom_model,
                "left_disparity_camera_geometry_fit": target_left_geom_info,
                "target_range_weight_sigma_m": 160.0,
                "axis_contract": RDF_AXIS_CONTRACT,
                "reference_basis": (
                    "Target-range weighted Huber nonlinear least-squares fit of left-image center offsets "
                    "and corrected disparity for stereo-center RDF triangulation, followed by Umeyama Sim(3). "
                    "Runtime uses only left bbox center, measured disparity, and calibration-fixed parameters."
                ),
            },
            model_complexity=11,
        )
    )
    target_left_geom_grid_fits = {
        160: (
            target_left_geom_model,
            target_left_geom_info,
            target_left_geom_scale,
            target_left_geom_rotation,
            target_left_geom_translation,
            target_left_geom_train,
        )
    }
    for target_left_geom_sigma_m in (80.0, 250.0, 400.0, 600.0, 800.0):
        target_left_geom_sigma_label = int(round(target_left_geom_sigma_m))
        grid_target_left_geom_weights = target_range_weights(
            train_samples,
            target_range_m=target_range_m,
            sigma_m=target_left_geom_sigma_m,
        )
        grid_target_left_geom_model, grid_target_left_geom_info = fit_nonlinear_left_disparity_camera_geometry(
            train_samples,
            camera=camera,
            weights=grid_target_left_geom_weights,
            loss="huber",
            f_scale_m=1.5,
        )
        grid_target_left_geom_train = predict_left_disparity_camera_geometry(
            train_samples,
            camera=camera,
            model=grid_target_left_geom_model,
        )
        (
            grid_target_left_geom_scale,
            grid_target_left_geom_rotation,
            grid_target_left_geom_translation,
        ) = fit_umeyama_sim3(
            grid_target_left_geom_train,
            truth_train,
        )
        target_left_geom_grid_fits[target_left_geom_sigma_label] = (
            grid_target_left_geom_model,
            grid_target_left_geom_info,
            grid_target_left_geom_scale,
            grid_target_left_geom_rotation,
            grid_target_left_geom_translation,
            grid_target_left_geom_train,
        )
        methods.append(
            FittedMethod(
                name=f"target_weighted_huber_left_disparity_camera_geometry_sim3_sigma{target_left_geom_sigma_label:03d}",
                predict=lambda samples, grid_target_left_geom_model=grid_target_left_geom_model, grid_target_left_geom_scale=grid_target_left_geom_scale, grid_target_left_geom_rotation=grid_target_left_geom_rotation, grid_target_left_geom_translation=grid_target_left_geom_translation: (
                    predict_sim3(
                        predict_left_disparity_camera_geometry(samples, camera=camera, model=grid_target_left_geom_model),
                        grid_target_left_geom_scale,
                        grid_target_left_geom_rotation,
                        grid_target_left_geom_translation,
                    )
                ),
                parameters={
                    "scale": grid_target_left_geom_scale,
                    "rotation_euler_zyx_deg": grid_target_left_geom_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": grid_target_left_geom_rotation.as_matrix().tolist(),
                    "translation_rdf_m": grid_target_left_geom_translation.tolist(),
                    "left_disparity_triangulation_model": grid_target_left_geom_model,
                    "left_disparity_camera_geometry_fit": grid_target_left_geom_info,
                    "target_range_weight_sigma_m": target_left_geom_sigma_m,
                    "axis_contract": RDF_AXIS_CONTRACT,
                    "reference_basis": (
                        "Target-range weighted Huber nonlinear least-squares sigma-grid fit of left-image "
                        "center offsets and corrected disparity for stereo-center RDF triangulation, followed "
                        "by Umeyama Sim(3). Runtime uses only left bbox center, measured disparity, and "
                        "calibration-fixed parameters."
                    ),
                },
            model_complexity=11,
        )
    )
    (
        target_left_geom_sigma400_model,
        target_left_geom_sigma400_info,
        target_left_geom_sigma400_scale,
        target_left_geom_sigma400_rotation,
        target_left_geom_sigma400_translation,
        target_left_geom_sigma400_train,
    ) = target_left_geom_grid_fits[400]
    target_left_geom_sigma400_sim3_train = predict_sim3(
        target_left_geom_sigma400_train,
        target_left_geom_sigma400_scale,
        target_left_geom_sigma400_rotation,
        target_left_geom_sigma400_translation,
    )
    target_left_geom_sigma400_quadratic_raw_forward_lut = fit_polynomial_raw_forward_residual_lut(
        raw_train[:, 2],
        truth_train[:, 2] - target_left_geom_sigma400_sim3_train[:, 2],
        degree=2,
        ridge_alpha=100.0,
        weights=target_range_weights(train_samples, target_range_m=target_range_m, sigma_m=80.0, floor=0.01),
    )
    for target_left_geom_residual_shrink in (0.25, 0.5, 0.75):
        target_left_geom_sigma400_shrink_lut = scale_residual_lut(
            target_left_geom_sigma400_quadratic_raw_forward_lut,
            residual_scale=target_left_geom_residual_shrink,
            fit_basis_suffix=(
                "target-window residual shrinkage for validation-stable left-disparity geometry deployment; "
                "runtime x-axis remains raw RDF forward_m"
            ),
        )
        target_left_geom_shrink_label = f"shrink{int(round(target_left_geom_residual_shrink * 100)):03d}"
        methods.append(
            FittedMethod(
                name=(
                    "target_weighted_huber_left_disparity_camera_geometry_sim3_"
                    f"quadratic_residual_raw_forward_lut_sigma400_{target_left_geom_shrink_label}"
                ),
                predict=lambda samples, target_left_geom_sigma400_model=target_left_geom_sigma400_model, target_left_geom_sigma400_scale=target_left_geom_sigma400_scale, target_left_geom_sigma400_rotation=target_left_geom_sigma400_rotation, target_left_geom_sigma400_translation=target_left_geom_sigma400_translation, target_left_geom_sigma400_shrink_lut=target_left_geom_sigma400_shrink_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(
                            predict_left_disparity_camera_geometry(
                                samples,
                                camera=camera,
                                model=target_left_geom_sigma400_model,
                            ),
                            target_left_geom_sigma400_scale,
                            target_left_geom_sigma400_rotation,
                            target_left_geom_sigma400_translation,
                        ),
                        stack(samples, "raw_rdf_m")[:, 2],
                        target_left_geom_sigma400_shrink_lut,
                    )
                ),
                parameters={
                    "scale": target_left_geom_sigma400_scale,
                    "rotation_euler_zyx_deg": target_left_geom_sigma400_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": target_left_geom_sigma400_rotation.as_matrix().tolist(),
                    "translation_rdf_m": target_left_geom_sigma400_translation.tolist(),
                    "left_disparity_triangulation_model": target_left_geom_sigma400_model,
                    "forward_residual_raw_forward_lut": target_left_geom_sigma400_shrink_lut,
                    "left_disparity_camera_geometry_fit": target_left_geom_sigma400_info,
                    "target_range_weight_sigma_m": 400.0,
                    "target_residual_weight_sigma_m": 80.0,
                    "target_residual_shrink": target_left_geom_residual_shrink,
                    "axis_contract": RDF_AXIS_CONTRACT,
                    "reference_basis": (
                        "Target-range weighted Huber left-image center/disparity triangulation plus Sim(3), "
                        "with a shrunk target-weighted ridge-regularized quadratic raw-forward residual LUT. "
                        "Runtime uses left bbox center, measured disparity, raw_forward_m, and calibration-fixed "
                        "parameters only."
                    ),
                },
                model_complexity=14,
            )
        )
    target_left_geom_sim3_train = predict_sim3(
        target_left_geom_train,
        target_left_geom_scale,
        target_left_geom_rotation,
        target_left_geom_translation,
    )
    target_left_geom_quadratic_raw_forward_lut = fit_polynomial_raw_forward_residual_lut(
        raw_train[:, 2],
        truth_train[:, 2] - target_left_geom_sim3_train[:, 2],
        degree=2,
        ridge_alpha=100.0,
        weights=target_range_weights(train_samples, target_range_m=target_range_m, sigma_m=80.0, floor=0.01),
    )
    methods.append(
        FittedMethod(
            name="target_weighted_huber_left_disparity_camera_geometry_sim3_quadratic_residual_raw_forward_lut_sigma160",
            predict=lambda samples, target_left_geom_model=target_left_geom_model, target_left_geom_scale=target_left_geom_scale, target_left_geom_rotation=target_left_geom_rotation, target_left_geom_translation=target_left_geom_translation, target_left_geom_quadratic_raw_forward_lut=target_left_geom_quadratic_raw_forward_lut: (
                apply_forward_residual_lut(
                    predict_sim3(
                        predict_left_disparity_camera_geometry(samples, camera=camera, model=target_left_geom_model),
                        target_left_geom_scale,
                        target_left_geom_rotation,
                        target_left_geom_translation,
                    ),
                    stack(samples, "raw_rdf_m")[:, 2],
                    target_left_geom_quadratic_raw_forward_lut,
                )
            ),
            parameters={
                "scale": target_left_geom_scale,
                "rotation_euler_zyx_deg": target_left_geom_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": target_left_geom_rotation.as_matrix().tolist(),
                "translation_rdf_m": target_left_geom_translation.tolist(),
                "left_disparity_triangulation_model": target_left_geom_model,
                "forward_residual_raw_forward_lut": target_left_geom_quadratic_raw_forward_lut,
                "left_disparity_camera_geometry_fit": target_left_geom_info,
                "target_range_weight_sigma_m": 160.0,
                "axis_contract": RDF_AXIS_CONTRACT,
                "reference_basis": (
                    "Target-range weighted Huber left-image center/disparity triangulation plus Sim(3), "
                    "with a target-weighted ridge-regularized quadratic raw-forward residual LUT. Runtime "
                    "uses left bbox center, measured disparity, raw_forward_m, and calibration-fixed parameters only."
                ),
            },
            model_complexity=14,
        )
    )

    target_reprojection_left_geom_model, target_reprojection_left_geom_info = fit_reprojection_left_disparity_camera_geometry(
        train_samples,
        camera=camera,
        weights=target_left_geom_weights,
        loss="huber",
        f_scale_px=1.5,
        disparity_sigma_px=1.0,
        bbox_sigma_px=1.0,
    )
    target_reprojection_left_geom_train = predict_left_disparity_camera_geometry(
        train_samples,
        camera=camera,
        model=target_reprojection_left_geom_model,
    )
    (
        target_reprojection_left_geom_scale,
        target_reprojection_left_geom_rotation,
        target_reprojection_left_geom_translation,
    ) = fit_umeyama_sim3(target_reprojection_left_geom_train, truth_train)
    methods.append(
        FittedMethod(
            name="target_weighted_huber_left_reprojection_geometry_sim3_sigma160",
            predict=lambda samples, target_reprojection_left_geom_model=target_reprojection_left_geom_model, target_reprojection_left_geom_scale=target_reprojection_left_geom_scale, target_reprojection_left_geom_rotation=target_reprojection_left_geom_rotation, target_reprojection_left_geom_translation=target_reprojection_left_geom_translation: (
                predict_sim3(
                    predict_left_disparity_camera_geometry(samples, camera=camera, model=target_reprojection_left_geom_model),
                    target_reprojection_left_geom_scale,
                    target_reprojection_left_geom_rotation,
                    target_reprojection_left_geom_translation,
                )
            ),
            parameters={
                "scale": target_reprojection_left_geom_scale,
                "rotation_euler_zyx_deg": target_reprojection_left_geom_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": target_reprojection_left_geom_rotation.as_matrix().tolist(),
                "translation_rdf_m": target_reprojection_left_geom_translation.tolist(),
                "left_disparity_triangulation_model": target_reprojection_left_geom_model,
                "left_reprojection_geometry_fit": target_reprojection_left_geom_info,
                "target_range_weight_sigma_m": 160.0,
                "axis_contract": RDF_AXIS_CONTRACT,
                "reference_basis": (
                    "Target-range weighted Huber fit of left-image center offsets and corrected disparity "
                    "from truth-to-observation reprojection residuals, followed by Umeyama Sim(3). Runtime "
                    "uses only left bbox center, measured disparity, and calibration-fixed parameters."
                ),
            },
            model_complexity=11,
        )
    )

    joint_disp_coeff, joint_disp_rotation, joint_disp_info = fit_nonlinear_disparity_ray_rotation(
        train_samples,
        loss="linear",
    )
    methods.append(
        FittedMethod(
            name="joint_disparity_ray_rotation_nls",
            predict=lambda samples, joint_disp_coeff=joint_disp_coeff, joint_disp_rotation=joint_disp_rotation: (
                predict_disparity_ray_rotation(samples, joint_disp_coeff, joint_disp_rotation)
            ),
            parameters={
                "scale": 1.0,
                "rotation_euler_zyx_deg": joint_disp_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": joint_disp_rotation.as_matrix().tolist(),
                "translation_rdf_m": [0.0, 0.0, 0.0],
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": joint_disp_coeff.tolist(),
                "joint_disparity_ray_rotation_fit": joint_disp_info,
                "reference_basis": (
                    "Nonlinear least-squares joint fit of corrected disparity d_corr=a*d+b and "
                    "RDF rotation. Runtime scales raw RDF by d/d_corr and applies the fixed rotation."
                ),
            },
            model_complexity=5,
        )
    )

    for huber_f_scale_m, huber_name_suffix in (
        (1.5, ""),
        (0.75, "_fscale0p75"),
        (1.0, "_fscale1p0"),
        (2.0, "_fscale2p0"),
        (3.0, "_fscale3p0"),
    ):
        joint_huber_coeff, joint_huber_rotation, joint_huber_info = fit_nonlinear_disparity_ray_rotation(
            train_samples,
            loss="huber",
            f_scale_m=huber_f_scale_m,
        )
        methods.append(
            FittedMethod(
                name=f"huber_joint_disparity_ray_rotation_nls{huber_name_suffix}",
                predict=lambda samples, joint_huber_coeff=joint_huber_coeff, joint_huber_rotation=joint_huber_rotation: (
                    predict_disparity_ray_rotation(samples, joint_huber_coeff, joint_huber_rotation)
                ),
                parameters={
                    "scale": 1.0,
                    "rotation_euler_zyx_deg": joint_huber_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": joint_huber_rotation.as_matrix().tolist(),
                    "translation_rdf_m": [0.0, 0.0, 0.0],
                    "raw_rdf_ray_disparity_correction_coefficients_high_to_low": joint_huber_coeff.tolist(),
                    "joint_disparity_ray_rotation_fit": joint_huber_info,
                    "reference_basis": (
                        "Huber robust nonlinear least-squares joint fit of corrected disparity d_corr=a*d+b "
                        "and RDF rotation. Runtime scales raw RDF by d/d_corr and applies the fixed rotation."
                    ),
                },
                model_complexity=5,
            )
        )

    for sigma_m in (30.0, 45.0, 60.0, 90.0, 120.0, 160.0, 250.0):
        sigma_label = int(round(sigma_m))
        target_joint_weights = target_range_weights(train_samples, target_range_m=target_range_m, sigma_m=sigma_m)
        target_joint_f_scale_grid = [(1.5, "")]
        if sigma_label == 60:
            target_joint_f_scale_grid.extend(
                [
                    (0.75, "_fscale0p75"),
                    (1.0, "_fscale1p0"),
                    (2.0, "_fscale2p0"),
                    (3.0, "_fscale3p0"),
                ]
            )
        for target_joint_f_scale_m, target_joint_name_suffix in target_joint_f_scale_grid:
            target_joint_coeff, target_joint_rotation, target_joint_info = fit_nonlinear_disparity_ray_rotation(
                train_samples,
                weights=target_joint_weights,
                loss="huber",
                f_scale_m=target_joint_f_scale_m,
            )
            methods.append(
                FittedMethod(
                    name=(
                        f"target_weighted_huber_joint_disparity_ray_rotation_nls_"
                        f"sigma{sigma_label:03d}{target_joint_name_suffix}"
                    ),
                    predict=lambda samples, target_joint_coeff=target_joint_coeff, target_joint_rotation=target_joint_rotation: (
                        predict_disparity_ray_rotation(samples, target_joint_coeff, target_joint_rotation)
                    ),
                    parameters={
                        "scale": 1.0,
                        "rotation_euler_zyx_deg": target_joint_rotation.as_euler("zyx", degrees=True).tolist(),
                        "rotation_matrix_rdf": target_joint_rotation.as_matrix().tolist(),
                        "translation_rdf_m": [0.0, 0.0, 0.0],
                        "raw_rdf_ray_disparity_correction_coefficients_high_to_low": target_joint_coeff.tolist(),
                        "joint_disparity_ray_rotation_fit": target_joint_info,
                        "target_range_weight_sigma_m": sigma_m,
                        "reference_basis": (
                            "Target-range weighted Huber nonlinear least-squares joint fit of corrected disparity "
                            "d_corr=a*d+b and RDF rotation. Runtime scales raw RDF by d/d_corr and applies "
                            "the fixed rotation."
                        ),
                    },
                    model_complexity=6,
                )
            )

    diagonal_affine_matrix, diagonal_affine_translation = fit_diagonal_affine_rdf(raw_train, truth_train)
    methods.append(
        FittedMethod(
            name="diagonal_affine_rdf",
            predict=lambda samples, diagonal_affine_matrix=diagonal_affine_matrix, diagonal_affine_translation=diagonal_affine_translation: (
                predict_affine_rdf(
                    stack(samples, "raw_rdf_m"),
                    diagonal_affine_matrix,
                    diagonal_affine_translation,
                )
            ),
            parameters={
                "affine_matrix_rdf": diagonal_affine_matrix.tolist(),
                "translation_rdf_m": diagonal_affine_translation.tolist(),
                "reference_basis": (
                    "axis-wise gain+bias estimated-to-truth RDF calibration; diagonal affine matrix "
                    "keeps right/down/forward axes independent and avoids shear"
                ),
            },
            model_complexity=6,
        )
    )
    for identity_lambda in (1.0e-2, 1.0):
        shrunk_matrix, shrunk_translation = fit_diagonal_affine_rdf(
            raw_train,
            truth_train,
            gain_identity_lambda=identity_lambda,
        )
        lambda_label = f"{identity_lambda:g}".replace(".", "p")
        methods.append(
            FittedMethod(
                name=f"identity_shrunk_diagonal_affine_rdf_lambda{lambda_label}",
                predict=lambda samples, shrunk_matrix=shrunk_matrix, shrunk_translation=shrunk_translation: (
                    predict_affine_rdf(stack(samples, "raw_rdf_m"), shrunk_matrix, shrunk_translation)
                ),
                parameters={
                    "affine_matrix_rdf": shrunk_matrix.tolist(),
                    "translation_rdf_m": shrunk_translation.tolist(),
                    "gain_identity_lambda": float(identity_lambda),
                    "reference_basis": (
                        "axis-wise gain+bias RDF calibration with gain shrinkage toward identity; "
                        "runtime uses only raw RDF and fixed diagonal affine parameters"
                    ),
                },
                model_complexity=6,
            )
        )
    for bin_count in (3, 5):
        piecewise_sim3 = fit_piecewise_sim3_raw_forward(raw_train, truth_train, bin_count=bin_count)
        methods.append(
            FittedMethod(
                name=f"sim3_piecewise_raw_forward_bins{bin_count}",
                predict=lambda samples, piecewise_sim3=piecewise_sim3: (
                    predict_piecewise_sim3_raw_forward(stack(samples, "raw_rdf_m"), piecewise_sim3)
                ),
                parameters={
                    "scale": sim_scale,
                    "rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "translation_rdf_m": sim_translation.tolist(),
                    "piecewise_sim3_raw_forward": piecewise_sim3,
                    "reference_basis": (
                        "range-dependent Sim(3) nodes fitted in raw RDF forward quantile bins; "
                        "runtime blends adjacent node outputs by raw_forward_m"
                    ),
                },
                model_complexity=7 + 7 * len(piecewise_sim3.get("transforms", [])),
            )
        )
        for piecewise_alpha in (0.25, 0.5, 0.75):
            alpha_label = f"alpha{int(round(piecewise_alpha * 100)):03d}"
            methods.append(
                FittedMethod(
                    name=f"sim3_blended_piecewise_raw_forward_bins{bin_count}_{alpha_label}",
                    predict=lambda samples, piecewise_sim3=piecewise_sim3, piecewise_alpha=piecewise_alpha: (
                        predict_blended_piecewise_sim3_raw_forward(
                            stack(samples, "raw_rdf_m"),
                            global_scale=sim_scale,
                            global_rotation=sim_rotation,
                            global_translation=sim_translation,
                            piecewise_model=piecewise_sim3,
                            piecewise_alpha=piecewise_alpha,
                        )
                    ),
                    parameters={
                        "scale": sim_scale,
                        "rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                        "rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                        "translation_rdf_m": sim_translation.tolist(),
                        "piecewise_sim3_raw_forward": piecewise_sim3,
                        "piecewise_sim3_blend_alpha": float(piecewise_alpha),
                        "reference_basis": (
                            "global Sim(3) and range-dependent Sim(3) node outputs are convex-blended "
                            "in stereo-center RDF by raw_forward_m; runtime uses raw RDF only"
                        ),
                    },
                    model_complexity=7 + 3 * len(piecewise_sim3.get("transforms", [])),
                )
            )

    affine_matrix, affine_translation = fit_affine_rdf(raw_train, truth_train)
    methods.append(
        FittedMethod(
            name="affine_rdf",
            predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation: (
                predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation)
            ),
            parameters={
                "affine_matrix_rdf": affine_matrix.tolist(),
                "translation_rdf_m": affine_translation.tolist(),
                "reference_basis": (
                    "unconstrained affine estimated-to-truth RDF calibration; offline comparison only until "
                    "StereoDistance runtime profile schema supports affine correction"
                ),
            },
            model_complexity=12,
        )
    )
    affine_train = predict_affine_rdf(raw_train, affine_matrix, affine_translation)
    affine_forward_lut = fit_binned_residual_lut(d_meas, truth_train[:, 2] - affine_train[:, 2])
    methods.append(
        FittedMethod(
            name="affine_binned_residual_disparity",
            predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_forward_lut=affine_forward_lut: (
                apply_forward_residual_lut(
                    predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                    measured_disparity(samples),
                    affine_forward_lut,
                )
            ),
            parameters={
                "affine_matrix_rdf": affine_matrix.tolist(),
                "translation_rdf_m": affine_translation.tolist(),
                "forward_residual_disparity_lut": affine_forward_lut,
                "reference_basis": (
                    "affine estimated-to-truth RDF calibration plus quantile-binned median forward "
                    "residual LUT over measured disparity"
                ),
            },
            model_complexity=12 + len(affine_forward_lut.get("x_values_px", [])),
        )
    )
    for bin_count in (7, 11):
        affine_raw_forward_lut = relabel_residual_lut_x_values(
            fit_binned_residual_lut(
                raw_train[:, 2],
                truth_train[:, 2] - affine_train[:, 2],
                bin_count=bin_count,
            ),
            x_key="x_values_m",
            fit_basis_suffix="raw RDF forward_m",
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"affine_binned_residual_raw_forward{suffix}",
                predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_raw_forward_lut=affine_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        affine_raw_forward_lut,
                    )
                ),
                parameters={
                    "affine_matrix_rdf": affine_matrix.tolist(),
                    "translation_rdf_m": affine_translation.tolist(),
                    "forward_residual_raw_forward_lut": affine_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "affine estimated-to-truth RDF calibration plus quantile-binned median forward "
                        "residual LUT over raw RDF forward distance"
                    ),
                },
                model_complexity=12 + len(affine_raw_forward_lut.get("x_values_m", [])),
            )
        )
        affine_monotone_raw_forward_lut = relabel_residual_lut_x_values(
            fit_monotone_binned_residual_lut(
                raw_train[:, 2],
                truth_train[:, 2] - affine_train[:, 2],
                bin_count=bin_count,
            ),
            x_key="x_values_m",
            fit_basis_suffix="raw RDF forward_m",
        )
        methods.append(
            FittedMethod(
                name=f"affine_monotone_binned_residual_raw_forward{suffix}",
                predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_monotone_raw_forward_lut=affine_monotone_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        affine_monotone_raw_forward_lut,
                    )
                ),
                parameters={
                    "affine_matrix_rdf": affine_matrix.tolist(),
                    "translation_rdf_m": affine_translation.tolist(),
                    "forward_residual_raw_forward_lut": affine_monotone_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "affine estimated-to-truth RDF calibration plus isotonic-regression "
                        "monotone forward residual LUT over raw RDF forward distance"
                    ),
                },
                model_complexity=12 + len(affine_monotone_raw_forward_lut.get("x_values_m", [])),
            )
        )
    for bin_count in (7, 11):
        affine_normalized_raw_forward_lut = fit_range_normalized_residual_lut(
            raw_train[:, 2],
            truth_train[:, 2] - affine_train[:, 2],
            bin_count=bin_count,
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"affine_range_normalized_binned_residual_raw_forward{suffix}",
                predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_normalized_raw_forward_lut=affine_normalized_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        affine_normalized_raw_forward_lut,
                    )
                ),
                parameters={
                    "affine_matrix_rdf": affine_matrix.tolist(),
                    "translation_rdf_m": affine_translation.tolist(),
                    "forward_residual_raw_forward_lut": affine_normalized_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "affine estimated-to-truth RDF calibration plus range-normalized forward "
                        "residual LUT over raw RDF forward distance; residual/forward is fitted then "
                        "exported as meter residuals"
                    ),
                },
                model_complexity=12 + len(affine_normalized_raw_forward_lut.get("x_values_m", [])),
            )
        )
    for bin_count in (7, 11):
        affine_inverse_depth_raw_forward_lut = fit_inverse_depth_residual_lut(
            raw_train[:, 2],
            affine_train[:, 2],
            truth_train[:, 2],
            bin_count=bin_count,
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"affine_inverse_depth_binned_residual_raw_forward{suffix}",
                predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_inverse_depth_raw_forward_lut=affine_inverse_depth_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        affine_inverse_depth_raw_forward_lut,
                    )
                ),
                parameters={
                    "affine_matrix_rdf": affine_matrix.tolist(),
                    "translation_rdf_m": affine_translation.tolist(),
                    "forward_residual_raw_forward_lut": affine_inverse_depth_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "affine estimated-to-truth RDF calibration plus inverse-depth forward "
                        "residual LUT over raw RDF forward distance; inverse-depth delta is fitted "
                        "then exported as meter residuals"
                    ),
                },
                model_complexity=12 + len(affine_inverse_depth_raw_forward_lut.get("x_values_m", [])),
            )
        )
        affine_monotone_inverse_depth_raw_forward_lut = fit_monotone_inverse_depth_residual_lut(
            raw_train[:, 2],
            affine_train[:, 2],
            truth_train[:, 2],
            bin_count=bin_count,
        )
        methods.append(
            FittedMethod(
                name=f"affine_monotone_inverse_depth_binned_residual_raw_forward{suffix}",
                predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_monotone_inverse_depth_raw_forward_lut=affine_monotone_inverse_depth_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        affine_monotone_inverse_depth_raw_forward_lut,
                    )
                ),
                parameters={
                    "affine_matrix_rdf": affine_matrix.tolist(),
                    "translation_rdf_m": affine_translation.tolist(),
                    "forward_residual_raw_forward_lut": affine_monotone_inverse_depth_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "affine estimated-to-truth RDF calibration plus isotonic-regression constrained "
                        "inverse-depth forward residual LUT over raw RDF forward distance; inverse-depth "
                        "delta is fitted, projected monotone, then exported as meter residuals"
                    ),
                },
                model_complexity=12 + len(affine_monotone_inverse_depth_raw_forward_lut.get("x_values_m", [])),
            )
        )
    affine_quadratic_raw_forward_lut = fit_polynomial_raw_forward_residual_lut(
        raw_train[:, 2],
        truth_train[:, 2] - affine_train[:, 2],
        degree=2,
        ridge_alpha=10.0,
        bin_count=7,
    )
    methods.append(
        FittedMethod(
            name="affine_quadratic_residual_raw_forward_lut",
            predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_quadratic_raw_forward_lut=affine_quadratic_raw_forward_lut: (
                apply_forward_residual_lut(
                    predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                    stack(samples, "raw_rdf_m")[:, 2],
                    affine_quadratic_raw_forward_lut,
                )
            ),
            parameters={
                "affine_matrix_rdf": affine_matrix.tolist(),
                "translation_rdf_m": affine_translation.tolist(),
                "forward_residual_raw_forward_lut": affine_quadratic_raw_forward_lut,
                "reference_basis": (
                    "affine estimated-to-truth RDF calibration plus ridge-regularized quadratic "
                    "forward residual over raw RDF forward distance, exported as a runtime LUT"
                ),
            },
            model_complexity=15,
        )
    )
    for bin_count, residual_shrink in (
        (3, 0.25),
        (3, 0.5),
        (5, 0.25),
        (5, 0.5),
        (7, 0.25),
        (7, 0.5),
        (7, 0.75),
    ):
        affine_quadratic_local_raw_forward_lut = fit_polynomial_plus_binned_raw_forward_residual_lut(
            raw_train[:, 2],
            truth_train[:, 2] - affine_train[:, 2],
            degree=2,
            ridge_alpha=10.0,
            bin_count=bin_count,
            residual_shrink=residual_shrink,
        )
        bin_suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        shrink_suffix = "" if abs(residual_shrink - 0.5) < 1.0e-12 else f"_shrink{int(round(residual_shrink * 100)):03d}"
        methods.append(
            FittedMethod(
                name=f"affine_quadratic_local_residual_raw_forward_lut{bin_suffix}{shrink_suffix}",
                predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_quadratic_local_raw_forward_lut=affine_quadratic_local_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        affine_quadratic_local_raw_forward_lut,
                    )
                ),
                parameters={
                    "affine_matrix_rdf": affine_matrix.tolist(),
                    "translation_rdf_m": affine_translation.tolist(),
                    "forward_residual_raw_forward_lut": affine_quadratic_local_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "local_residual_shrink": residual_shrink,
                    "reference_basis": (
                        "affine estimated-to-truth RDF calibration plus ridge-regularized quadratic "
                        "forward residual and shrunk local residual LUT over raw RDF forward distance"
                    ),
                },
                model_complexity=15 + len(affine_quadratic_local_raw_forward_lut.get("x_values_m", [])),
            )
        )
    for bin_count in (5, 9, 11):
        affine_variant_forward_lut = fit_binned_residual_lut(
            d_meas,
            truth_train[:, 2] - affine_train[:, 2],
            bin_count=bin_count,
        )
        methods.append(
            FittedMethod(
                name=f"affine_binned_residual_disparity_bins{bin_count}",
                predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_variant_forward_lut=affine_variant_forward_lut: (
                    apply_forward_residual_lut(
                        predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                        measured_disparity(samples),
                        affine_variant_forward_lut,
                    )
                ),
                parameters={
                    "affine_matrix_rdf": affine_matrix.tolist(),
                    "translation_rdf_m": affine_translation.tolist(),
                    "forward_residual_disparity_lut": affine_variant_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "affine estimated-to-truth RDF calibration plus quantile-binned median forward "
                        "residual LUT over measured disparity; bin-count grid-search candidate"
                    ),
                },
                model_complexity=12 + len(affine_variant_forward_lut.get("x_values_px", [])),
            )
        )
    for bin_count in (7, 11):
        affine_smoothed_forward_lut = fit_smoothed_binned_residual_lut(
            d_meas,
            truth_train[:, 2] - affine_train[:, 2],
            bin_count=bin_count,
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"affine_smoothed_binned_residual_disparity{suffix}",
                predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_smoothed_forward_lut=affine_smoothed_forward_lut: (
                    apply_forward_residual_lut(
                        predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                        measured_disparity(samples),
                        affine_smoothed_forward_lut,
                    )
                ),
                parameters={
                    "affine_matrix_rdf": affine_matrix.tolist(),
                    "translation_rdf_m": affine_translation.tolist(),
                    "forward_residual_disparity_lut": affine_smoothed_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "affine estimated-to-truth RDF calibration plus smoothed quantile-binned median "
                        "forward residual LUT over measured disparity"
                    ),
                },
                model_complexity=12 + len(affine_smoothed_forward_lut.get("x_values_px", [])),
            )
        )
    for bin_count in (7, 11):
        affine_capped_forward_lut = fit_capped_binned_residual_lut(
            d_meas,
            truth_train[:, 2] - affine_train[:, 2],
            bin_count=bin_count,
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"affine_capped_binned_residual_disparity{suffix}",
                predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_capped_forward_lut=affine_capped_forward_lut: (
                    apply_forward_residual_lut(
                        predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                        measured_disparity(samples),
                        affine_capped_forward_lut,
                    )
                ),
                parameters={
                    "affine_matrix_rdf": affine_matrix.tolist(),
                    "translation_rdf_m": affine_translation.tolist(),
                    "forward_residual_disparity_lut": affine_capped_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "affine estimated-to-truth RDF calibration plus robustly capped quantile-binned "
                        "median forward residual LUT over measured disparity"
                    ),
                },
                model_complexity=12 + len(affine_capped_forward_lut.get("x_values_px", [])),
            )
        )
    affine_rdf_lut = {
        axis: fit_binned_residual_lut(d_meas, truth_train[:, axis_index] - affine_train[:, axis_index])
        for axis_index, axis in enumerate(("right", "down", "forward"))
    }
    methods.append(
        FittedMethod(
            name="affine_binned_rdf_residual_disparity",
            predict=lambda samples, affine_matrix=affine_matrix, affine_translation=affine_translation, affine_rdf_lut=affine_rdf_lut: (
                apply_rdf_residual_lut(
                    predict_affine_rdf(stack(samples, "raw_rdf_m"), affine_matrix, affine_translation),
                    measured_disparity(samples),
                    affine_rdf_lut,
                )
            ),
            parameters={
                "affine_matrix_rdf": affine_matrix.tolist(),
                "translation_rdf_m": affine_translation.tolist(),
                "rdf_residual_disparity_lut": affine_rdf_lut,
                "reference_basis": (
                    "affine estimated-to-truth RDF calibration plus per-RDF-axis quantile-binned median "
                    "residual LUTs over measured disparity"
                ),
            },
            model_complexity=12 + sum(len(lut.get("x_values_px", [])) for lut in affine_rdf_lut.values()),
        )
    )

    ridge_affine_matrix, ridge_affine_translation = fit_affine_rdf(raw_train, truth_train, ridge_lambda=1.0e-3)
    methods.append(
        FittedMethod(
            name="ridge_affine_rdf",
            predict=lambda samples, ridge_affine_matrix=ridge_affine_matrix, ridge_affine_translation=ridge_affine_translation: (
                predict_affine_rdf(stack(samples, "raw_rdf_m"), ridge_affine_matrix, ridge_affine_translation)
            ),
            parameters={
                "affine_matrix_rdf": ridge_affine_matrix.tolist(),
                "translation_rdf_m": ridge_affine_translation.tolist(),
                "ridge_lambda": 1.0e-3,
                "reference_basis": (
                    "ridge-regularized affine estimated-to-truth RDF calibration; offline comparison only until "
                    "StereoDistance runtime profile schema supports affine correction"
                ),
            },
            model_complexity=12,
        )
    )

    trimmed_scale, trimmed_rotation, trimmed_translation, trimmed_mask = fit_trimmed_umeyama_sim3(raw_train, truth_train)
    methods.append(
        FittedMethod(
            name="trimmed_sim3",
            predict=lambda samples, trimmed_scale=trimmed_scale, trimmed_rotation=trimmed_rotation, trimmed_translation=trimmed_translation: (
                predict_sim3(stack(samples, "raw_rdf_m"), trimmed_scale, trimmed_rotation, trimmed_translation)
            ),
            parameters={
                "scale": trimmed_scale,
                "rotation_euler_zyx_deg": trimmed_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": trimmed_rotation.as_matrix().tolist(),
                "translation_rdf_m": trimmed_translation.tolist(),
                "trim_keep_fraction": float(np.mean(trimmed_mask)),
                "reference_basis": "least-trimmed consensus refit around Umeyama similarity transform",
            },
            model_complexity=8,
        )
    )

    ransac_scale, ransac_rotation, ransac_translation, ransac_mask = fit_ransac_umeyama_sim3(raw_train, truth_train)
    methods.append(
        FittedMethod(
            name="ransac_sim3",
            predict=lambda samples, ransac_scale=ransac_scale, ransac_rotation=ransac_rotation, ransac_translation=ransac_translation: (
                predict_sim3(stack(samples, "raw_rdf_m"), ransac_scale, ransac_rotation, ransac_translation)
            ),
            parameters={
                "scale": ransac_scale,
                "rotation_euler_zyx_deg": ransac_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": ransac_rotation.as_matrix().tolist(),
                "translation_rdf_m": ransac_translation.tolist(),
                "consensus_fraction": float(np.mean(ransac_mask)),
                "ransac_iterations": 160,
                "ransac_subset_size": 4,
                "reference_basis": "Random Sample Consensus model fitting with Umeyama similarity transform refit on the consensus set",
            },
            model_complexity=9,
        )
    )

    huber_scale, huber_rotation, huber_translation, huber_weights = fit_huber_umeyama_sim3(raw_train, truth_train)
    methods.append(
        FittedMethod(
            name="huber_sim3",
            predict=lambda samples, huber_scale=huber_scale, huber_rotation=huber_rotation, huber_translation=huber_translation: (
                predict_sim3(stack(samples, "raw_rdf_m"), huber_scale, huber_rotation, huber_translation)
            ),
            parameters={
                "scale": huber_scale,
                "rotation_euler_zyx_deg": huber_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": huber_rotation.as_matrix().tolist(),
                "translation_rdf_m": huber_translation.tolist(),
                "mean_weight": float(np.mean(huber_weights)),
                "min_weight": float(np.min(huber_weights)),
                "reference_basis": "Huber M-estimation via iteratively reweighted Umeyama similarity transform",
            },
            model_complexity=8,
        )
    )

    tukey_scale, tukey_rotation, tukey_translation, tukey_weights = fit_tukey_umeyama_sim3(raw_train, truth_train)
    methods.append(
        FittedMethod(
            name="tukey_sim3",
            predict=lambda samples, tukey_scale=tukey_scale, tukey_rotation=tukey_rotation, tukey_translation=tukey_translation: (
                predict_sim3(stack(samples, "raw_rdf_m"), tukey_scale, tukey_rotation, tukey_translation)
            ),
            parameters={
                "scale": tukey_scale,
                "rotation_euler_zyx_deg": tukey_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": tukey_rotation.as_matrix().tolist(),
                "translation_rdf_m": tukey_translation.tolist(),
                "mean_weight": float(np.mean(tukey_weights)),
                "zero_weight_fraction": float(np.mean(tukey_weights <= 1.0e-12)),
                "reference_basis": "Tukey biweight redescending M-estimation via iteratively reweighted Umeyama similarity transform",
            },
            model_complexity=8,
        )
    )

    tls_scale, tls_rotation, tls_translation, tls_weights, tls_cutoff = fit_truncated_umeyama_sim3(
        raw_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="truncated_ls_sim3",
            predict=lambda samples, tls_scale=tls_scale, tls_rotation=tls_rotation, tls_translation=tls_translation: (
                predict_sim3(stack(samples, "raw_rdf_m"), tls_scale, tls_rotation, tls_translation)
            ),
            parameters={
                "scale": tls_scale,
                "rotation_euler_zyx_deg": tls_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": tls_rotation.as_matrix().tolist(),
                "translation_rdf_m": tls_translation.tolist(),
                "inlier_fraction": float(np.mean(tls_weights > 0.0)),
                "truncated_ls_cutoff_m": float(tls_cutoff),
                "reference_basis": (
                    "truncated least-squares robust point-set registration refit with an Umeyama "
                    "similarity transform; calibration-time truth only fixes the active parameters"
                ),
            },
            model_complexity=8,
        )
    )

    range_weights = target_range_weights(train_samples, target_range_m=target_range_m)
    range_scale, range_rotation, range_translation = fit_weighted_umeyama_sim3(raw_train, truth_train, range_weights)
    methods.append(
        FittedMethod(
            name="range_weighted_sim3",
            predict=lambda samples, range_scale=range_scale, range_rotation=range_rotation, range_translation=range_translation: (
                predict_sim3(stack(samples, "raw_rdf_m"), range_scale, range_rotation, range_translation)
            ),
            parameters={
                "scale": range_scale,
                "rotation_euler_zyx_deg": range_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": range_rotation.as_matrix().tolist(),
                "translation_rdf_m": range_translation.tolist(),
                "target_range_m": target_range_m,
                "range_weight_sigma_m": 250.0,
                "range_weight_floor": 0.08,
                "mean_weight": float(np.mean(range_weights)),
                "reference_basis": "weighted Procrustes/Umeyama objective emphasizing the mission target range",
            },
            model_complexity=8,
        )
    )

    tls_range_scale, tls_range_rotation, tls_range_translation, tls_range_weights, tls_range_cutoff = (
        fit_truncated_umeyama_sim3(
            raw_train,
            truth_train,
            base_weights=range_weights,
        )
    )
    methods.append(
        FittedMethod(
            name="truncated_ls_range_weighted_sim3",
            predict=lambda samples, tls_range_scale=tls_range_scale, tls_range_rotation=tls_range_rotation, tls_range_translation=tls_range_translation: (
                predict_sim3(stack(samples, "raw_rdf_m"), tls_range_scale, tls_range_rotation, tls_range_translation)
            ),
            parameters={
                "scale": tls_range_scale,
                "rotation_euler_zyx_deg": tls_range_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": tls_range_rotation.as_matrix().tolist(),
                "translation_rdf_m": tls_range_translation.tolist(),
                "target_range_m": target_range_m,
                "range_weight_sigma_m": 250.0,
                "range_weight_floor": 0.08,
                "inlier_fraction": float(np.mean(tls_range_weights > 0.0)),
                "truncated_ls_cutoff_m": float(tls_range_cutoff),
                "reference_basis": (
                    "truncated least-squares robust point-set registration plus target-range weighted "
                    "Umeyama similarity transform; runtime uses only raw stereo RDF"
                ),
            },
            model_complexity=9,
        )
    )

    huber_range_scale, huber_range_rotation, huber_range_translation, huber_range_weights = fit_huber_umeyama_sim3(
        raw_train,
        truth_train,
        base_weights=range_weights,
    )
    methods.append(
        FittedMethod(
            name="huber_range_weighted_sim3",
            predict=lambda samples, huber_range_scale=huber_range_scale, huber_range_rotation=huber_range_rotation, huber_range_translation=huber_range_translation: (
                predict_sim3(stack(samples, "raw_rdf_m"), huber_range_scale, huber_range_rotation, huber_range_translation)
            ),
            parameters={
                "scale": huber_range_scale,
                "rotation_euler_zyx_deg": huber_range_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": huber_range_rotation.as_matrix().tolist(),
                "translation_rdf_m": huber_range_translation.tolist(),
                "target_range_m": target_range_m,
                "range_weight_sigma_m": 250.0,
                "range_weight_floor": 0.08,
                "mean_weight": float(np.mean(huber_range_weights)),
                "min_weight": float(np.min(huber_range_weights)),
                "reference_basis": "Huber IRLS plus target-range weighted Umeyama similarity transform",
            },
            model_complexity=9,
        )
    )

    tukey_range_scale, tukey_range_rotation, tukey_range_translation, tukey_range_weights = fit_tukey_umeyama_sim3(
        raw_train,
        truth_train,
        base_weights=range_weights,
    )
    methods.append(
        FittedMethod(
            name="tukey_range_weighted_sim3",
            predict=lambda samples, tukey_range_scale=tukey_range_scale, tukey_range_rotation=tukey_range_rotation, tukey_range_translation=tukey_range_translation: (
                predict_sim3(stack(samples, "raw_rdf_m"), tukey_range_scale, tukey_range_rotation, tukey_range_translation)
            ),
            parameters={
                "scale": tukey_range_scale,
                "rotation_euler_zyx_deg": tukey_range_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": tukey_range_rotation.as_matrix().tolist(),
                "translation_rdf_m": tukey_range_translation.tolist(),
                "target_range_m": target_range_m,
                "range_weight_sigma_m": 250.0,
                "range_weight_floor": 0.08,
                "mean_weight": float(np.mean(tukey_range_weights)),
                "zero_weight_fraction": float(np.mean(tukey_range_weights <= 1.0e-12)),
                "reference_basis": "Tukey biweight IRLS plus target-range weighted Umeyama similarity transform",
            },
            model_complexity=9,
        )
    )

    for loss_name, method_name in (("soft_l1", "soft_l1_sim3"), ("cauchy", "cauchy_sim3")):
        robust_scale, robust_rotation, robust_translation, robust_info = fit_robust_nonlinear_sim3(
            raw_train,
            truth_train,
            loss=loss_name,
        )
        methods.append(
            FittedMethod(
                name=method_name,
                predict=lambda samples, robust_scale=robust_scale, robust_rotation=robust_rotation, robust_translation=robust_translation: (
                    predict_sim3(stack(samples, "raw_rdf_m"), robust_scale, robust_rotation, robust_translation)
                ),
                parameters={
                    "scale": robust_scale,
                    "rotation_euler_zyx_deg": robust_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": robust_rotation.as_matrix().tolist(),
                    "translation_rdf_m": robust_translation.tolist(),
                    "robust_loss": robust_info["loss"],
                    "robust_f_scale_m": robust_info["f_scale_m"],
                    "optimizer_success": robust_info["success"],
                    "optimizer_cost": robust_info["cost"],
                    "optimizer_nfev": robust_info["nfev"],
                    "reference_basis": "robust nonlinear least-squares Sim(3) with scipy least_squares M-estimator loss",
                },
                model_complexity=9,
            )
    )

    d_true = true_disparity(train_samples)
    target_disparity_weights = target_range_weights(
        train_samples,
        target_range_m=target_range_m,
        sigma_m=120.0,
        floor=0.02,
    )
    target_forward_residual_weights = target_range_weights(
        train_samples,
        target_range_m=target_range_m,
        sigma_m=80.0,
        floor=0.01,
    )
    inverse_depth_coeff = fit_inverse_depth_ray_coefficients(raw_train[:, 2], truth_train[:, 2])
    methods.append(
        FittedMethod(
            name="global_inverse_depth_linear",
            predict=lambda samples, inverse_depth_coeff=inverse_depth_coeff: (
                predict_inverse_depth_ray_corrected(samples, inverse_depth_coeff)
            ),
            parameters={
                "inverse_depth_coefficients_high_to_low": inverse_depth_coeff.tolist(),
                "correction_space": "raw_rdf_inverse_depth_ray_scale",
                "reference_basis": (
                    "Calibration-time linear mapping from 1/raw_forward to 1/truth_forward. "
                    "Runtime scales the raw RDF ray using raw_forward_m only."
                ),
            },
            model_complexity=2,
        )
    )
    huber_inverse_depth_coeff, huber_inverse_depth_info = fit_huber_inverse_depth_ray_coefficients(
        raw_train[:, 2],
        truth_train[:, 2],
    )
    methods.append(
        FittedMethod(
            name="huber_global_inverse_depth_linear",
            predict=lambda samples, huber_inverse_depth_coeff=huber_inverse_depth_coeff: (
                predict_inverse_depth_ray_corrected(samples, huber_inverse_depth_coeff)
            ),
            parameters={
                "inverse_depth_coefficients_high_to_low": huber_inverse_depth_coeff.tolist(),
                "correction_space": "raw_rdf_inverse_depth_ray_scale",
                "huber_inverse_depth_fit": huber_inverse_depth_info,
                "reference_basis": (
                    "Huber M-estimation/IRLS linear mapping from 1/raw_forward to 1/truth_forward. "
                    "Runtime scales the raw RDF ray using raw_forward_m only."
                ),
            },
            model_complexity=3,
        )
    )
    theil_sen_inverse_depth_coeff, theil_sen_inverse_depth_info = fit_theil_sen_inverse_depth_ray_coefficients(
        raw_train[:, 2],
        truth_train[:, 2],
    )
    methods.append(
        FittedMethod(
            name="theil_sen_global_inverse_depth_linear",
            predict=lambda samples, theil_sen_inverse_depth_coeff=theil_sen_inverse_depth_coeff: (
                predict_inverse_depth_ray_corrected(samples, theil_sen_inverse_depth_coeff)
            ),
            parameters={
                "inverse_depth_coefficients_high_to_low": theil_sen_inverse_depth_coeff.tolist(),
                "correction_space": "raw_rdf_inverse_depth_ray_scale",
                "theil_sen_inverse_depth_fit": theil_sen_inverse_depth_info,
                "reference_basis": (
                    "Theil-Sen robust median-slope linear mapping from 1/raw_forward to 1/truth_forward. "
                    "Runtime scales the raw RDF ray using raw_forward_m only."
                ),
            },
            model_complexity=4,
        )
    )
    for ridge_lambda in (0.1, 1.0, 10.0):
        ridge_inverse_depth_coeff, ridge_inverse_depth_info = fit_ridge_inverse_depth_ray_coefficients(
            raw_train[:, 2],
            truth_train[:, 2],
            ridge_lambda=ridge_lambda,
        )
        ridge_suffix = f"lambda{str(ridge_lambda).replace('.', 'p')}"
        methods.append(
            FittedMethod(
                name=f"ridge_global_inverse_depth_linear_{ridge_suffix}",
                predict=lambda samples, ridge_inverse_depth_coeff=ridge_inverse_depth_coeff: (
                    predict_inverse_depth_ray_corrected(samples, ridge_inverse_depth_coeff)
                ),
                parameters={
                    "inverse_depth_coefficients_high_to_low": ridge_inverse_depth_coeff.tolist(),
                    "correction_space": "raw_rdf_inverse_depth_ray_scale",
                    "ridge_inverse_depth_fit": ridge_inverse_depth_info,
                    "reference_basis": (
                        "Tikhonov/ridge-regularized linear mapping from 1/raw_forward to 1/truth_forward, "
                        "shrunk toward identity ray scale. Runtime scales the raw RDF ray using raw_forward_m only."
                    ),
                },
                model_complexity=3,
            )
        )
    inverse_depth_train = predict_inverse_depth_ray_corrected(train_samples, inverse_depth_coeff)
    inverse_depth_sim_scale, inverse_depth_sim_rotation, inverse_depth_sim_translation = fit_umeyama_sim3(
        inverse_depth_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="inverse_depth_linear_then_sim3",
            predict=lambda samples, inverse_depth_coeff=inverse_depth_coeff, inverse_depth_sim_scale=inverse_depth_sim_scale, inverse_depth_sim_rotation=inverse_depth_sim_rotation, inverse_depth_sim_translation=inverse_depth_sim_translation: (
                predict_inverse_depth_ray_corrected_sim3(
                    samples,
                    inverse_depth_coeff,
                    inverse_depth_sim_scale,
                    inverse_depth_sim_rotation,
                    inverse_depth_sim_translation,
                )
            ),
            parameters={
                "scale": inverse_depth_sim_scale,
                "rotation_euler_zyx_deg": inverse_depth_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": inverse_depth_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": inverse_depth_sim_translation.tolist(),
                "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": inverse_depth_coeff.tolist(),
                "reference_basis": (
                    "Calibration-time inverse-depth ray correction followed by Sim(3) estimated-to-truth "
                    "registration. Runtime uses raw stereo-center RDF and fixed calibration parameters only."
                ),
            },
            model_complexity=9,
        )
    )
    huber_inverse_depth_train = predict_inverse_depth_ray_corrected(train_samples, huber_inverse_depth_coeff)
    (
        huber_inverse_depth_sim_scale,
        huber_inverse_depth_sim_rotation,
        huber_inverse_depth_sim_translation,
    ) = fit_umeyama_sim3(
        huber_inverse_depth_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="huber_inverse_depth_linear_then_sim3",
            predict=lambda samples, huber_inverse_depth_coeff=huber_inverse_depth_coeff, huber_inverse_depth_sim_scale=huber_inverse_depth_sim_scale, huber_inverse_depth_sim_rotation=huber_inverse_depth_sim_rotation, huber_inverse_depth_sim_translation=huber_inverse_depth_sim_translation: (
                predict_inverse_depth_ray_corrected_sim3(
                    samples,
                    huber_inverse_depth_coeff,
                    huber_inverse_depth_sim_scale,
                    huber_inverse_depth_sim_rotation,
                    huber_inverse_depth_sim_translation,
                )
            ),
            parameters={
                "scale": huber_inverse_depth_sim_scale,
                "rotation_euler_zyx_deg": huber_inverse_depth_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": huber_inverse_depth_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": huber_inverse_depth_sim_translation.tolist(),
                "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": huber_inverse_depth_coeff.tolist(),
                "huber_inverse_depth_fit": huber_inverse_depth_info,
                "reference_basis": (
                    "Huber M-estimation/IRLS inverse-depth ray correction followed by Sim(3) "
                    "estimated-to-truth registration. Runtime uses raw stereo-center RDF and fixed "
                    "calibration parameters only."
                ),
            },
            model_complexity=10,
        )
    )
    theil_sen_inverse_depth_train = predict_inverse_depth_ray_corrected(train_samples, theil_sen_inverse_depth_coeff)
    (
        theil_sen_inverse_depth_sim_scale,
        theil_sen_inverse_depth_sim_rotation,
        theil_sen_inverse_depth_sim_translation,
    ) = fit_umeyama_sim3(
        theil_sen_inverse_depth_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="theil_sen_inverse_depth_linear_then_sim3",
            predict=lambda samples, theil_sen_inverse_depth_coeff=theil_sen_inverse_depth_coeff, theil_sen_inverse_depth_sim_scale=theil_sen_inverse_depth_sim_scale, theil_sen_inverse_depth_sim_rotation=theil_sen_inverse_depth_sim_rotation, theil_sen_inverse_depth_sim_translation=theil_sen_inverse_depth_sim_translation: (
                predict_inverse_depth_ray_corrected_sim3(
                    samples,
                    theil_sen_inverse_depth_coeff,
                    theil_sen_inverse_depth_sim_scale,
                    theil_sen_inverse_depth_sim_rotation,
                    theil_sen_inverse_depth_sim_translation,
                )
            ),
            parameters={
                "scale": theil_sen_inverse_depth_sim_scale,
                "rotation_euler_zyx_deg": theil_sen_inverse_depth_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": theil_sen_inverse_depth_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": theil_sen_inverse_depth_sim_translation.tolist(),
                "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": theil_sen_inverse_depth_coeff.tolist(),
                "theil_sen_inverse_depth_fit": theil_sen_inverse_depth_info,
                "reference_basis": (
                    "Theil-Sen robust inverse-depth ray correction followed by Sim(3) estimated-to-truth "
                    "registration. Runtime uses raw stereo-center RDF and fixed calibration parameters only."
                ),
            },
            model_complexity=11,
        )
    )
    target_inverse_depth_coeff = fit_inverse_depth_ray_coefficients(
        raw_train[:, 2],
        truth_train[:, 2],
        weights=target_disparity_weights,
    )
    methods.append(
        FittedMethod(
            name="target_weighted_global_inverse_depth_linear",
            predict=lambda samples, target_inverse_depth_coeff=target_inverse_depth_coeff: (
                predict_inverse_depth_ray_corrected(samples, target_inverse_depth_coeff)
            ),
            parameters={
                "inverse_depth_coefficients_high_to_low": target_inverse_depth_coeff.tolist(),
                "correction_space": "raw_rdf_inverse_depth_ray_scale",
                "target_range_m": target_range_m,
                "range_weight_sigma_m": 120.0,
                "range_weight_floor": 0.02,
                "mean_weight": float(np.mean(target_disparity_weights)),
                "reference_basis": (
                    "Target-range weighted calibration-time linear mapping from 1/raw_forward to "
                    "1/truth_forward. Runtime scales the raw RDF ray using raw_forward_m only."
                ),
            },
            model_complexity=3,
        )
    )
    target_huber_inverse_depth_coeff, target_huber_inverse_depth_info = fit_huber_inverse_depth_ray_coefficients(
        raw_train[:, 2],
        truth_train[:, 2],
        weights=target_disparity_weights,
    )
    methods.append(
        FittedMethod(
            name="target_weighted_huber_global_inverse_depth_linear",
            predict=lambda samples, target_huber_inverse_depth_coeff=target_huber_inverse_depth_coeff: (
                predict_inverse_depth_ray_corrected(samples, target_huber_inverse_depth_coeff)
            ),
            parameters={
                "inverse_depth_coefficients_high_to_low": target_huber_inverse_depth_coeff.tolist(),
                "correction_space": "raw_rdf_inverse_depth_ray_scale",
                "target_range_m": target_range_m,
                "range_weight_sigma_m": 120.0,
                "range_weight_floor": 0.02,
                "mean_weight": float(np.mean(target_disparity_weights)),
                "huber_inverse_depth_fit": target_huber_inverse_depth_info,
                "reference_basis": (
                    "Target-range weighted Huber M-estimation/IRLS linear mapping from 1/raw_forward "
                    "to 1/truth_forward. Runtime scales the raw RDF ray using raw_forward_m only."
                ),
            },
            model_complexity=4,
        )
    )
    target_theil_sen_inverse_depth_coeff, target_theil_sen_inverse_depth_info = fit_theil_sen_inverse_depth_ray_coefficients(
        raw_train[:, 2],
        truth_train[:, 2],
        weights=target_disparity_weights,
    )
    methods.append(
        FittedMethod(
            name="target_weighted_theil_sen_global_inverse_depth_linear",
            predict=lambda samples, target_theil_sen_inverse_depth_coeff=target_theil_sen_inverse_depth_coeff: (
                predict_inverse_depth_ray_corrected(samples, target_theil_sen_inverse_depth_coeff)
            ),
            parameters={
                "inverse_depth_coefficients_high_to_low": target_theil_sen_inverse_depth_coeff.tolist(),
                "correction_space": "raw_rdf_inverse_depth_ray_scale",
                "target_range_m": target_range_m,
                "range_weight_sigma_m": 120.0,
                "range_weight_floor": 0.02,
                "mean_weight": float(np.mean(target_disparity_weights)),
                "theil_sen_inverse_depth_fit": target_theil_sen_inverse_depth_info,
                "reference_basis": (
                    "Target-range weighted Theil-Sen robust median-slope mapping from 1/raw_forward "
                    "to 1/truth_forward. Runtime scales the raw RDF ray using raw_forward_m only."
                ),
            },
            model_complexity=5,
        )
    )
    target_inverse_depth_train = predict_inverse_depth_ray_corrected(train_samples, target_inverse_depth_coeff)
    (
        target_inverse_depth_sim_scale,
        target_inverse_depth_sim_rotation,
        target_inverse_depth_sim_translation,
    ) = fit_umeyama_sim3(
        target_inverse_depth_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="target_weighted_inverse_depth_linear_then_sim3",
            predict=lambda samples, target_inverse_depth_coeff=target_inverse_depth_coeff, target_inverse_depth_sim_scale=target_inverse_depth_sim_scale, target_inverse_depth_sim_rotation=target_inverse_depth_sim_rotation, target_inverse_depth_sim_translation=target_inverse_depth_sim_translation: (
                predict_inverse_depth_ray_corrected_sim3(
                    samples,
                    target_inverse_depth_coeff,
                    target_inverse_depth_sim_scale,
                    target_inverse_depth_sim_rotation,
                    target_inverse_depth_sim_translation,
                )
            ),
            parameters={
                "scale": target_inverse_depth_sim_scale,
                "rotation_euler_zyx_deg": target_inverse_depth_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": target_inverse_depth_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": target_inverse_depth_sim_translation.tolist(),
                "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": target_inverse_depth_coeff.tolist(),
                "target_range_m": target_range_m,
                "range_weight_sigma_m": 120.0,
                "range_weight_floor": 0.02,
                "mean_weight": float(np.mean(target_disparity_weights)),
                "reference_basis": (
                    "Target-range weighted inverse-depth ray correction followed by Sim(3) "
                    "estimated-to-truth registration. Runtime uses raw stereo-center RDF and fixed "
                    "calibration parameters only."
                ),
            },
            model_complexity=10,
        )
    )
    target_huber_inverse_depth_train = predict_inverse_depth_ray_corrected(train_samples, target_huber_inverse_depth_coeff)
    (
        target_huber_inverse_depth_sim_scale,
        target_huber_inverse_depth_sim_rotation,
        target_huber_inverse_depth_sim_translation,
    ) = fit_umeyama_sim3(
        target_huber_inverse_depth_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="target_weighted_huber_inverse_depth_linear_then_sim3",
            predict=lambda samples, target_huber_inverse_depth_coeff=target_huber_inverse_depth_coeff, target_huber_inverse_depth_sim_scale=target_huber_inverse_depth_sim_scale, target_huber_inverse_depth_sim_rotation=target_huber_inverse_depth_sim_rotation, target_huber_inverse_depth_sim_translation=target_huber_inverse_depth_sim_translation: (
                predict_inverse_depth_ray_corrected_sim3(
                    samples,
                    target_huber_inverse_depth_coeff,
                    target_huber_inverse_depth_sim_scale,
                    target_huber_inverse_depth_sim_rotation,
                    target_huber_inverse_depth_sim_translation,
                )
            ),
            parameters={
                "scale": target_huber_inverse_depth_sim_scale,
                "rotation_euler_zyx_deg": target_huber_inverse_depth_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": target_huber_inverse_depth_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": target_huber_inverse_depth_sim_translation.tolist(),
                "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": target_huber_inverse_depth_coeff.tolist(),
                "target_range_m": target_range_m,
                "range_weight_sigma_m": 120.0,
                "range_weight_floor": 0.02,
                "mean_weight": float(np.mean(target_disparity_weights)),
                "huber_inverse_depth_fit": target_huber_inverse_depth_info,
                "reference_basis": (
                    "Target-range weighted Huber M-estimation/IRLS inverse-depth ray correction followed "
                    "by Sim(3) estimated-to-truth registration. Runtime uses raw stereo-center RDF and "
                    "fixed calibration parameters only."
                ),
            },
            model_complexity=11,
        )
    )
    target_theil_sen_inverse_depth_train = predict_inverse_depth_ray_corrected(
        train_samples,
        target_theil_sen_inverse_depth_coeff,
    )
    (
        target_theil_sen_inverse_depth_sim_scale,
        target_theil_sen_inverse_depth_sim_rotation,
        target_theil_sen_inverse_depth_sim_translation,
    ) = fit_umeyama_sim3(
        target_theil_sen_inverse_depth_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="target_weighted_theil_sen_inverse_depth_linear_then_sim3",
            predict=lambda samples, target_theil_sen_inverse_depth_coeff=target_theil_sen_inverse_depth_coeff, target_theil_sen_inverse_depth_sim_scale=target_theil_sen_inverse_depth_sim_scale, target_theil_sen_inverse_depth_sim_rotation=target_theil_sen_inverse_depth_sim_rotation, target_theil_sen_inverse_depth_sim_translation=target_theil_sen_inverse_depth_sim_translation: (
                predict_inverse_depth_ray_corrected_sim3(
                    samples,
                    target_theil_sen_inverse_depth_coeff,
                    target_theil_sen_inverse_depth_sim_scale,
                    target_theil_sen_inverse_depth_sim_rotation,
                    target_theil_sen_inverse_depth_sim_translation,
                )
            ),
            parameters={
                "scale": target_theil_sen_inverse_depth_sim_scale,
                "rotation_euler_zyx_deg": target_theil_sen_inverse_depth_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": target_theil_sen_inverse_depth_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": target_theil_sen_inverse_depth_sim_translation.tolist(),
                "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": target_theil_sen_inverse_depth_coeff.tolist(),
                "target_range_m": target_range_m,
                "range_weight_sigma_m": 120.0,
                "range_weight_floor": 0.02,
                "mean_weight": float(np.mean(target_disparity_weights)),
                "theil_sen_inverse_depth_fit": target_theil_sen_inverse_depth_info,
                "reference_basis": (
                    "Target-range weighted Theil-Sen inverse-depth ray correction followed by Sim(3) "
                    "estimated-to-truth registration. Runtime uses raw stereo-center RDF and fixed "
                    "calibration parameters only."
                ),
            },
            model_complexity=12,
        )
    )
    for sigma_m in (60.0, 250.0):
        sigma_weights = target_range_weights(
            train_samples,
            target_range_m=target_range_m,
            sigma_m=sigma_m,
            floor=0.02,
        )
        sigma_inverse_depth_coeff = fit_inverse_depth_ray_coefficients(
            raw_train[:, 2],
            truth_train[:, 2],
            weights=sigma_weights,
        )
        sigma_suffix = f"sigma{int(round(sigma_m)):03d}"
        methods.append(
            FittedMethod(
                name=f"target_weighted_global_inverse_depth_linear_{sigma_suffix}",
                predict=lambda samples, sigma_inverse_depth_coeff=sigma_inverse_depth_coeff: (
                    predict_inverse_depth_ray_corrected(samples, sigma_inverse_depth_coeff)
                ),
                parameters={
                    "inverse_depth_coefficients_high_to_low": sigma_inverse_depth_coeff.tolist(),
                    "correction_space": "raw_rdf_inverse_depth_ray_scale",
                    "target_range_m": target_range_m,
                    "range_weight_sigma_m": sigma_m,
                    "range_weight_floor": 0.02,
                    "mean_weight": float(np.mean(sigma_weights)),
                    "reference_basis": (
                        "Target-range weighted inverse-depth ray correction with sigma grid search. "
                        "Runtime scales the raw RDF ray using raw_forward_m only."
                    ),
                },
                model_complexity=3,
            )
        )
        sigma_inverse_depth_train = predict_inverse_depth_ray_corrected(train_samples, sigma_inverse_depth_coeff)
        (
            sigma_inverse_depth_sim_scale,
            sigma_inverse_depth_sim_rotation,
            sigma_inverse_depth_sim_translation,
        ) = fit_umeyama_sim3(
            sigma_inverse_depth_train,
            truth_train,
        )
        methods.append(
            FittedMethod(
                name=f"target_weighted_inverse_depth_linear_then_sim3_{sigma_suffix}",
                predict=lambda samples, sigma_inverse_depth_coeff=sigma_inverse_depth_coeff, sigma_inverse_depth_sim_scale=sigma_inverse_depth_sim_scale, sigma_inverse_depth_sim_rotation=sigma_inverse_depth_sim_rotation, sigma_inverse_depth_sim_translation=sigma_inverse_depth_sim_translation: (
                    predict_inverse_depth_ray_corrected_sim3(
                        samples,
                        sigma_inverse_depth_coeff,
                        sigma_inverse_depth_sim_scale,
                        sigma_inverse_depth_sim_rotation,
                        sigma_inverse_depth_sim_translation,
                    )
                ),
                parameters={
                    "scale": sigma_inverse_depth_sim_scale,
                    "rotation_euler_zyx_deg": sigma_inverse_depth_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": sigma_inverse_depth_sim_rotation.as_matrix().tolist(),
                    "translation_rdf_m": sigma_inverse_depth_sim_translation.tolist(),
                    "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": sigma_inverse_depth_coeff.tolist(),
                    "target_range_m": target_range_m,
                    "range_weight_sigma_m": sigma_m,
                    "range_weight_floor": 0.02,
                    "mean_weight": float(np.mean(sigma_weights)),
                    "reference_basis": (
                        "Target-range weighted inverse-depth ray correction with sigma grid search, "
                        "followed by Sim(3) estimated-to-truth registration. Runtime uses raw RDF and "
                        "fixed calibration parameters only."
                    ),
                },
                model_complexity=10,
            )
        )
    for ridge_lambda in (0.1, 1.0, 10.0):
        target_ridge_inverse_depth_coeff, target_ridge_inverse_depth_info = fit_ridge_inverse_depth_ray_coefficients(
            raw_train[:, 2],
            truth_train[:, 2],
            weights=target_disparity_weights,
            ridge_lambda=ridge_lambda,
        )
        ridge_suffix = f"lambda{str(ridge_lambda).replace('.', 'p')}"
        methods.append(
            FittedMethod(
                name=f"target_weighted_ridge_global_inverse_depth_linear_{ridge_suffix}",
                predict=lambda samples, target_ridge_inverse_depth_coeff=target_ridge_inverse_depth_coeff: (
                    predict_inverse_depth_ray_corrected(samples, target_ridge_inverse_depth_coeff)
                ),
                parameters={
                    "inverse_depth_coefficients_high_to_low": target_ridge_inverse_depth_coeff.tolist(),
                    "correction_space": "raw_rdf_inverse_depth_ray_scale",
                    "target_range_m": target_range_m,
                    "range_weight_sigma_m": 120.0,
                    "range_weight_floor": 0.02,
                    "mean_weight": float(np.mean(target_disparity_weights)),
                    "ridge_inverse_depth_fit": target_ridge_inverse_depth_info,
                    "reference_basis": (
                        "Target-range weighted Tikhonov/ridge inverse-depth ray correction shrunk toward "
                        "identity ray scale. Runtime scales the raw RDF ray using raw_forward_m only."
                    ),
                },
                model_complexity=4,
            )
        )
    for shrink in (0.25, 0.4, 0.5, 0.6, 0.75):
        shrunk_inverse_depth_coeff = (1.0 - shrink) * inverse_depth_coeff + shrink * target_inverse_depth_coeff
        shrink_suffix = f"shrink{int(round(shrink * 100)):03d}"
        methods.append(
            FittedMethod(
                name=f"target_shrunk_global_inverse_depth_linear_{shrink_suffix}",
                predict=lambda samples, shrunk_inverse_depth_coeff=shrunk_inverse_depth_coeff: (
                    predict_inverse_depth_ray_corrected(samples, shrunk_inverse_depth_coeff)
                ),
                parameters={
                    "inverse_depth_coefficients_high_to_low": shrunk_inverse_depth_coeff.tolist(),
                    "correction_space": "raw_rdf_inverse_depth_ray_scale",
                    "target_range_m": target_range_m,
                    "target_weight_shrink": shrink,
                    "reference_basis": (
                        "Convex blend of global and target-range weighted inverse-depth ray coefficients. "
                        "Runtime scales the raw RDF ray using raw_forward_m only."
                    ),
                },
                model_complexity=3,
            )
        )
        shrunk_inverse_depth_train = predict_inverse_depth_ray_corrected(train_samples, shrunk_inverse_depth_coeff)
        (
            shrunk_inverse_depth_sim_scale,
            shrunk_inverse_depth_sim_rotation,
            shrunk_inverse_depth_sim_translation,
        ) = fit_umeyama_sim3(
            shrunk_inverse_depth_train,
            truth_train,
        )
        methods.append(
            FittedMethod(
                name=f"target_shrunk_inverse_depth_linear_then_sim3_{shrink_suffix}",
                predict=lambda samples, shrunk_inverse_depth_coeff=shrunk_inverse_depth_coeff, shrunk_inverse_depth_sim_scale=shrunk_inverse_depth_sim_scale, shrunk_inverse_depth_sim_rotation=shrunk_inverse_depth_sim_rotation, shrunk_inverse_depth_sim_translation=shrunk_inverse_depth_sim_translation: (
                    predict_inverse_depth_ray_corrected_sim3(
                        samples,
                        shrunk_inverse_depth_coeff,
                        shrunk_inverse_depth_sim_scale,
                        shrunk_inverse_depth_sim_rotation,
                        shrunk_inverse_depth_sim_translation,
                    )
                ),
                parameters={
                    "scale": shrunk_inverse_depth_sim_scale,
                    "rotation_euler_zyx_deg": shrunk_inverse_depth_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": shrunk_inverse_depth_sim_rotation.as_matrix().tolist(),
                    "translation_rdf_m": shrunk_inverse_depth_sim_translation.tolist(),
                    "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": shrunk_inverse_depth_coeff.tolist(),
                    "target_range_m": target_range_m,
                    "target_weight_shrink": shrink,
                    "reference_basis": (
                        "Convex blend of global and target-range weighted inverse-depth ray coefficients "
                        "followed by Sim(3) estimated-to-truth registration. Runtime uses raw RDF and "
                        "fixed calibration parameters only."
                    ),
                },
                model_complexity=10,
            )
        )
    for shrink in (0.25, 0.4, 0.5, 0.6, 0.75):
        shrunk_huber_inverse_depth_coeff = (
            (1.0 - shrink) * huber_inverse_depth_coeff
            + shrink * target_huber_inverse_depth_coeff
        )
        shrink_suffix = f"shrink{int(round(shrink * 100)):03d}"
        methods.append(
            FittedMethod(
                name=f"target_shrunk_huber_global_inverse_depth_linear_{shrink_suffix}",
                predict=lambda samples, shrunk_huber_inverse_depth_coeff=shrunk_huber_inverse_depth_coeff: (
                    predict_inverse_depth_ray_corrected(samples, shrunk_huber_inverse_depth_coeff)
                ),
                parameters={
                    "inverse_depth_coefficients_high_to_low": shrunk_huber_inverse_depth_coeff.tolist(),
                    "correction_space": "raw_rdf_inverse_depth_ray_scale",
                    "target_range_m": target_range_m,
                    "target_weight_shrink": shrink,
                    "huber_inverse_depth_fit_global": huber_inverse_depth_info,
                    "huber_inverse_depth_fit_target_weighted": target_huber_inverse_depth_info,
                    "reference_basis": (
                        "Convex blend of global Huber and target-range weighted Huber inverse-depth "
                        "ray coefficients. Runtime scales the raw RDF ray using raw_forward_m only."
                    ),
                },
                model_complexity=4,
            )
        )
        shrunk_huber_inverse_depth_train = predict_inverse_depth_ray_corrected(
            train_samples,
            shrunk_huber_inverse_depth_coeff,
        )
        (
            shrunk_huber_inverse_depth_sim_scale,
            shrunk_huber_inverse_depth_sim_rotation,
            shrunk_huber_inverse_depth_sim_translation,
        ) = fit_umeyama_sim3(
            shrunk_huber_inverse_depth_train,
            truth_train,
        )
        methods.append(
            FittedMethod(
                name=f"target_shrunk_huber_inverse_depth_linear_then_sim3_{shrink_suffix}",
                predict=lambda samples, shrunk_huber_inverse_depth_coeff=shrunk_huber_inverse_depth_coeff, shrunk_huber_inverse_depth_sim_scale=shrunk_huber_inverse_depth_sim_scale, shrunk_huber_inverse_depth_sim_rotation=shrunk_huber_inverse_depth_sim_rotation, shrunk_huber_inverse_depth_sim_translation=shrunk_huber_inverse_depth_sim_translation: (
                    predict_inverse_depth_ray_corrected_sim3(
                        samples,
                        shrunk_huber_inverse_depth_coeff,
                        shrunk_huber_inverse_depth_sim_scale,
                        shrunk_huber_inverse_depth_sim_rotation,
                        shrunk_huber_inverse_depth_sim_translation,
                    )
                ),
                parameters={
                    "scale": shrunk_huber_inverse_depth_sim_scale,
                    "rotation_euler_zyx_deg": shrunk_huber_inverse_depth_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": shrunk_huber_inverse_depth_sim_rotation.as_matrix().tolist(),
                    "translation_rdf_m": shrunk_huber_inverse_depth_sim_translation.tolist(),
                    "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low": shrunk_huber_inverse_depth_coeff.tolist(),
                    "target_range_m": target_range_m,
                    "target_weight_shrink": shrink,
                    "huber_inverse_depth_fit_global": huber_inverse_depth_info,
                    "huber_inverse_depth_fit_target_weighted": target_huber_inverse_depth_info,
                    "reference_basis": (
                        "Convex blend of global Huber and target-range weighted Huber inverse-depth "
                        "ray coefficients followed by Sim(3) estimated-to-truth registration. Runtime "
                        "uses raw RDF and fixed calibration parameters only."
                    ),
                },
                model_complexity=11,
            )
        )
    linear_coeff = np.polyfit(d_meas, d_true, deg=1)
    methods.append(
        FittedMethod(
            name="global_disparity_linear",
            predict=lambda samples, linear_coeff=linear_coeff: predict_disparity_corrected(samples, camera, linear_coeff),
            parameters={"d_corr_coefficients_high_to_low": linear_coeff.tolist()},
            model_complexity=2,
        )
    )
    linear_disparity_train = predict_disparity_corrected(train_samples, camera, linear_coeff)
    linear_disparity_sim_scale, linear_disparity_sim_rotation, linear_disparity_sim_translation = fit_umeyama_sim3(
        linear_disparity_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="disparity_linear_then_sim3",
            predict=lambda samples, linear_coeff=linear_coeff, linear_disparity_sim_scale=linear_disparity_sim_scale, linear_disparity_sim_rotation=linear_disparity_sim_rotation, linear_disparity_sim_translation=linear_disparity_sim_translation: (
                predict_disparity_corrected_sim3(
                    samples,
                    camera,
                    linear_coeff,
                    linear_disparity_sim_scale,
                    linear_disparity_sim_rotation,
                    linear_disparity_sim_translation,
                )
            ),
            parameters={
                "scale": linear_disparity_sim_scale,
                "rotation_euler_zyx_deg": linear_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": linear_disparity_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": linear_disparity_sim_translation.tolist(),
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": linear_coeff.tolist(),
                "reference_basis": (
                    "Calibration-time linear measured-disparity correction applied to the raw RDF ray "
                    "before Sim(3) estimated-to-truth registration. Runtime uses measured disparity, raw RDF, "
                    "and fixed calibration parameters only."
                ),
            },
            model_complexity=9,
        )
    )
    linear_disparity_sim_train = predict_sim3(
        linear_disparity_train,
        linear_disparity_sim_scale,
        linear_disparity_sim_rotation,
        linear_disparity_sim_translation,
    )
    linear_disparity_quadratic_raw_forward_lut = fit_polynomial_raw_forward_residual_lut(
        raw_train[:, 2],
        truth_train[:, 2] - linear_disparity_sim_train[:, 2],
        degree=2,
        ridge_alpha=100.0,
        bin_count=9,
    )
    linear_disparity_inverse_depth_raw_forward_lut = fit_inverse_depth_residual_lut(
        raw_train[:, 2],
        linear_disparity_sim_train[:, 2],
        truth_train[:, 2],
        bin_count=9,
        prior_strength=12.0,
    )
    methods.append(
        FittedMethod(
            name="disparity_linear_then_sim3_quadratic_residual_raw_forward_lut",
            predict=lambda samples, linear_coeff=linear_coeff, linear_disparity_sim_scale=linear_disparity_sim_scale, linear_disparity_sim_rotation=linear_disparity_sim_rotation, linear_disparity_sim_translation=linear_disparity_sim_translation, linear_disparity_quadratic_raw_forward_lut=linear_disparity_quadratic_raw_forward_lut: (
                apply_forward_residual_lut(
                    predict_disparity_corrected_sim3(
                        samples,
                        camera,
                        linear_coeff,
                        linear_disparity_sim_scale,
                        linear_disparity_sim_rotation,
                        linear_disparity_sim_translation,
                    ),
                    stack(samples, "raw_rdf_m")[:, 2],
                    linear_disparity_quadratic_raw_forward_lut,
                )
            ),
            parameters={
                "scale": linear_disparity_sim_scale,
                "rotation_euler_zyx_deg": linear_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": linear_disparity_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": linear_disparity_sim_translation.tolist(),
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": linear_coeff.tolist(),
                "forward_residual_raw_forward_lut": linear_disparity_quadratic_raw_forward_lut,
                "reference_basis": (
                    "Linear measured-disparity ray correction followed by Sim(3), then a quadratic "
                    "raw-forward residual LUT for remaining stereo depth bias. Runtime uses raw RDF, "
                    "measured disparity, and fixed calibration parameters only."
                ),
            },
            model_complexity=12,
        )
    )
    methods.append(
        FittedMethod(
            name="disparity_linear_then_sim3_inverse_depth_residual_raw_forward_lut",
            predict=lambda samples, linear_coeff=linear_coeff, linear_disparity_sim_scale=linear_disparity_sim_scale, linear_disparity_sim_rotation=linear_disparity_sim_rotation, linear_disparity_sim_translation=linear_disparity_sim_translation, linear_disparity_inverse_depth_raw_forward_lut=linear_disparity_inverse_depth_raw_forward_lut: (
                apply_forward_residual_lut(
                    predict_disparity_corrected_sim3(
                        samples,
                        camera,
                        linear_coeff,
                        linear_disparity_sim_scale,
                        linear_disparity_sim_rotation,
                        linear_disparity_sim_translation,
                    ),
                    stack(samples, "raw_rdf_m")[:, 2],
                    linear_disparity_inverse_depth_raw_forward_lut,
                )
            ),
            parameters={
                "scale": linear_disparity_sim_scale,
                "rotation_euler_zyx_deg": linear_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": linear_disparity_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": linear_disparity_sim_translation.tolist(),
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": linear_coeff.tolist(),
                "forward_residual_raw_forward_lut": linear_disparity_inverse_depth_raw_forward_lut,
                "reference_basis": (
                    "Linear measured-disparity ray correction followed by Sim(3), then an inverse-depth "
                    "raw-forward residual LUT. The residual model follows stereo geometry by fitting "
                    "1/truth_forward - 1/base_forward on calibration data; runtime uses raw RDF, measured "
                    "disparity, and fixed calibration parameters only."
                ),
            },
            model_complexity=13,
        )
    )

    quadratic_coeff = np.polyfit(d_meas, d_true, deg=2)
    methods.append(
        FittedMethod(
            name="global_disparity_quadratic",
            predict=lambda samples, quadratic_coeff=quadratic_coeff: predict_disparity_corrected(samples, camera, quadratic_coeff),
            parameters={"d_corr_coefficients_high_to_low": quadratic_coeff.tolist()},
            model_complexity=3,
        )
    )
    quadratic_disparity_train = predict_disparity_corrected(train_samples, camera, quadratic_coeff)
    (
        quadratic_disparity_sim_scale,
        quadratic_disparity_sim_rotation,
        quadratic_disparity_sim_translation,
    ) = fit_umeyama_sim3(
        quadratic_disparity_train,
        truth_train,
    )
    methods.append(
        FittedMethod(
            name="disparity_quadratic_then_sim3",
            predict=lambda samples, quadratic_coeff=quadratic_coeff, quadratic_disparity_sim_scale=quadratic_disparity_sim_scale, quadratic_disparity_sim_rotation=quadratic_disparity_sim_rotation, quadratic_disparity_sim_translation=quadratic_disparity_sim_translation: (
                predict_disparity_corrected_sim3(
                    samples,
                    camera,
                    quadratic_coeff,
                    quadratic_disparity_sim_scale,
                    quadratic_disparity_sim_rotation,
                    quadratic_disparity_sim_translation,
                )
            ),
            parameters={
                "scale": quadratic_disparity_sim_scale,
                "rotation_euler_zyx_deg": quadratic_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": quadratic_disparity_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": quadratic_disparity_sim_translation.tolist(),
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": quadratic_coeff.tolist(),
                "reference_basis": (
                    "Calibration-time quadratic measured-disparity correction applied to the raw RDF ray "
                    "before Sim(3) estimated-to-truth registration. Runtime uses measured disparity, raw RDF, "
                    "and fixed calibration parameters only."
                ),
            },
            model_complexity=10,
        )
    )
    quadratic_disparity_sim_train = predict_sim3(
        quadratic_disparity_train,
        quadratic_disparity_sim_scale,
        quadratic_disparity_sim_rotation,
        quadratic_disparity_sim_translation,
    )
    quadratic_disparity_quadratic_raw_forward_lut = fit_polynomial_raw_forward_residual_lut(
        raw_train[:, 2],
        truth_train[:, 2] - quadratic_disparity_sim_train[:, 2],
        degree=2,
        ridge_alpha=100.0,
        bin_count=9,
    )
    quadratic_disparity_inverse_depth_raw_forward_lut = fit_inverse_depth_residual_lut(
        raw_train[:, 2],
        quadratic_disparity_sim_train[:, 2],
        truth_train[:, 2],
        bin_count=9,
        prior_strength=12.0,
    )
    methods.append(
        FittedMethod(
            name="disparity_quadratic_then_sim3_quadratic_residual_raw_forward_lut",
            predict=lambda samples, quadratic_coeff=quadratic_coeff, quadratic_disparity_sim_scale=quadratic_disparity_sim_scale, quadratic_disparity_sim_rotation=quadratic_disparity_sim_rotation, quadratic_disparity_sim_translation=quadratic_disparity_sim_translation, quadratic_disparity_quadratic_raw_forward_lut=quadratic_disparity_quadratic_raw_forward_lut: (
                apply_forward_residual_lut(
                    predict_disparity_corrected_sim3(
                        samples,
                        camera,
                        quadratic_coeff,
                        quadratic_disparity_sim_scale,
                        quadratic_disparity_sim_rotation,
                        quadratic_disparity_sim_translation,
                    ),
                    stack(samples, "raw_rdf_m")[:, 2],
                    quadratic_disparity_quadratic_raw_forward_lut,
                )
            ),
            parameters={
                "scale": quadratic_disparity_sim_scale,
                "rotation_euler_zyx_deg": quadratic_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": quadratic_disparity_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": quadratic_disparity_sim_translation.tolist(),
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": quadratic_coeff.tolist(),
                "forward_residual_raw_forward_lut": quadratic_disparity_quadratic_raw_forward_lut,
                "reference_basis": (
                    "Quadratic measured-disparity ray correction followed by Sim(3), then a quadratic "
                    "raw-forward residual LUT for remaining stereo depth bias. Runtime uses raw RDF, "
                    "measured disparity, and fixed calibration parameters only."
                ),
            },
            model_complexity=13,
        )
    )
    methods.append(
        FittedMethod(
            name="disparity_quadratic_then_sim3_inverse_depth_residual_raw_forward_lut",
            predict=lambda samples, quadratic_coeff=quadratic_coeff, quadratic_disparity_sim_scale=quadratic_disparity_sim_scale, quadratic_disparity_sim_rotation=quadratic_disparity_sim_rotation, quadratic_disparity_sim_translation=quadratic_disparity_sim_translation, quadratic_disparity_inverse_depth_raw_forward_lut=quadratic_disparity_inverse_depth_raw_forward_lut: (
                apply_forward_residual_lut(
                    predict_disparity_corrected_sim3(
                        samples,
                        camera,
                        quadratic_coeff,
                        quadratic_disparity_sim_scale,
                        quadratic_disparity_sim_rotation,
                        quadratic_disparity_sim_translation,
                    ),
                    stack(samples, "raw_rdf_m")[:, 2],
                    quadratic_disparity_inverse_depth_raw_forward_lut,
                )
            ),
            parameters={
                "scale": quadratic_disparity_sim_scale,
                "rotation_euler_zyx_deg": quadratic_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": quadratic_disparity_sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": quadratic_disparity_sim_translation.tolist(),
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low": quadratic_coeff.tolist(),
                "forward_residual_raw_forward_lut": quadratic_disparity_inverse_depth_raw_forward_lut,
                "reference_basis": (
                    "Quadratic measured-disparity ray correction followed by Sim(3), then an inverse-depth "
                    "raw-forward residual LUT. The residual model follows stereo geometry by fitting "
                    "1/truth_forward - 1/base_forward on calibration data; runtime uses raw RDF, measured "
                    "disparity, and fixed calibration parameters only."
                ),
            },
            model_complexity=14,
        )
    )

    for degree, complexity in ((1, 10), (2, 11)):
        target_disparity_coeff = fit_weighted_polyfit(
            d_meas,
            d_true,
            degree=degree,
            weights=target_disparity_weights,
        )
        target_disparity_train = predict_disparity_corrected(train_samples, camera, target_disparity_coeff)
        (
            target_disparity_sim_scale,
            target_disparity_sim_rotation,
            target_disparity_sim_translation,
        ) = fit_umeyama_sim3(target_disparity_train, truth_train)
        degree_name = "linear" if degree == 1 else "quadratic"
        methods.append(
            FittedMethod(
                name=f"target_weighted_disparity_{degree_name}_then_sim3",
                predict=lambda samples, target_disparity_coeff=target_disparity_coeff, target_disparity_sim_scale=target_disparity_sim_scale, target_disparity_sim_rotation=target_disparity_sim_rotation, target_disparity_sim_translation=target_disparity_sim_translation: (
                    predict_disparity_corrected_sim3(
                        samples,
                        camera,
                        target_disparity_coeff,
                        target_disparity_sim_scale,
                        target_disparity_sim_rotation,
                        target_disparity_sim_translation,
                    )
                ),
                parameters={
                    "scale": target_disparity_sim_scale,
                    "rotation_euler_zyx_deg": target_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": target_disparity_sim_rotation.as_matrix().tolist(),
                    "translation_rdf_m": target_disparity_sim_translation.tolist(),
                    "raw_rdf_ray_disparity_correction_coefficients_high_to_low": target_disparity_coeff.tolist(),
                    "target_range_m": target_range_m,
                    "target_weight_sigma_m": 120.0,
                    "target_weight_floor": 0.02,
                    "mean_weight": float(np.mean(target_disparity_weights)),
                    "reference_basis": (
                        "Target-range-weighted measured-disparity correction applied to the raw RDF ray "
                        "before Sim(3) estimated-to-truth registration. Runtime uses measured disparity, "
                        "raw RDF, and fixed calibration parameters only."
                    ),
                },
                model_complexity=complexity,
            )
        )
        target_disparity_sim_train = predict_sim3(
            target_disparity_train,
            target_disparity_sim_scale,
            target_disparity_sim_rotation,
            target_disparity_sim_translation,
        )
        target_disparity_quadratic_raw_forward_lut = fit_polynomial_raw_forward_residual_lut(
            raw_train[:, 2],
            truth_train[:, 2] - target_disparity_sim_train[:, 2],
            degree=2,
            ridge_alpha=100.0,
            bin_count=9,
        )
        target_disparity_inverse_depth_raw_forward_lut = fit_inverse_depth_residual_lut(
            raw_train[:, 2],
            target_disparity_sim_train[:, 2],
            truth_train[:, 2],
            bin_count=9,
            prior_strength=12.0,
        )
        methods.append(
            FittedMethod(
                name=f"target_weighted_disparity_{degree_name}_then_sim3_quadratic_residual_raw_forward_lut",
                predict=lambda samples, target_disparity_coeff=target_disparity_coeff, target_disparity_sim_scale=target_disparity_sim_scale, target_disparity_sim_rotation=target_disparity_sim_rotation, target_disparity_sim_translation=target_disparity_sim_translation, target_disparity_quadratic_raw_forward_lut=target_disparity_quadratic_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_disparity_corrected_sim3(
                            samples,
                            camera,
                            target_disparity_coeff,
                            target_disparity_sim_scale,
                            target_disparity_sim_rotation,
                            target_disparity_sim_translation,
                        ),
                        stack(samples, "raw_rdf_m")[:, 2],
                        target_disparity_quadratic_raw_forward_lut,
                    )
                ),
                parameters={
                    "scale": target_disparity_sim_scale,
                    "rotation_euler_zyx_deg": target_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": target_disparity_sim_rotation.as_matrix().tolist(),
                    "translation_rdf_m": target_disparity_sim_translation.tolist(),
                    "raw_rdf_ray_disparity_correction_coefficients_high_to_low": target_disparity_coeff.tolist(),
                    "forward_residual_raw_forward_lut": target_disparity_quadratic_raw_forward_lut,
                    "target_range_m": target_range_m,
                    "target_weight_sigma_m": 120.0,
                    "target_weight_floor": 0.02,
                    "mean_weight": float(np.mean(target_disparity_weights)),
                    "reference_basis": (
                        "Target-range-weighted measured-disparity ray correction followed by Sim(3), then "
                        "a quadratic raw-forward residual LUT for remaining stereo depth bias. Runtime uses "
                        "raw RDF, measured disparity, and fixed calibration parameters only."
                    ),
                },
                model_complexity=complexity + 3,
            )
        )
        methods.append(
            FittedMethod(
                name=f"target_weighted_disparity_{degree_name}_then_sim3_inverse_depth_residual_raw_forward_lut",
                predict=lambda samples, target_disparity_coeff=target_disparity_coeff, target_disparity_sim_scale=target_disparity_sim_scale, target_disparity_sim_rotation=target_disparity_sim_rotation, target_disparity_sim_translation=target_disparity_sim_translation, target_disparity_inverse_depth_raw_forward_lut=target_disparity_inverse_depth_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_disparity_corrected_sim3(
                            samples,
                            camera,
                            target_disparity_coeff,
                            target_disparity_sim_scale,
                            target_disparity_sim_rotation,
                            target_disparity_sim_translation,
                        ),
                        stack(samples, "raw_rdf_m")[:, 2],
                        target_disparity_inverse_depth_raw_forward_lut,
                    )
                ),
                parameters={
                    "scale": target_disparity_sim_scale,
                    "rotation_euler_zyx_deg": target_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": target_disparity_sim_rotation.as_matrix().tolist(),
                    "translation_rdf_m": target_disparity_sim_translation.tolist(),
                    "raw_rdf_ray_disparity_correction_coefficients_high_to_low": target_disparity_coeff.tolist(),
                    "forward_residual_raw_forward_lut": target_disparity_inverse_depth_raw_forward_lut,
                    "target_range_m": target_range_m,
                    "target_weight_sigma_m": 120.0,
                    "target_weight_floor": 0.02,
                    "mean_weight": float(np.mean(target_disparity_weights)),
                    "reference_basis": (
                        "Target-range-weighted measured-disparity ray correction followed by Sim(3), then "
                        "an inverse-depth raw-forward residual LUT. The residual model follows stereo "
                        "geometry by fitting 1/truth_forward - 1/base_forward on calibration data; runtime "
                        "uses raw RDF, measured disparity, and fixed calibration parameters only."
                    ),
                },
                model_complexity=complexity + 4,
            )
        )
        for residual_sigma_m in (40.0, 80.0, 160.0):
            residual_weights = (
                target_forward_residual_weights
                if abs(residual_sigma_m - 80.0) < 1.0e-12
                else target_range_weights(
                    train_samples,
                    target_range_m=target_range_m,
                    sigma_m=residual_sigma_m,
                    floor=0.01,
                )
            )
            target_disparity_target_weighted_raw_forward_lut = fit_polynomial_raw_forward_residual_lut(
                raw_train[:, 2],
                truth_train[:, 2] - target_disparity_sim_train[:, 2],
                degree=2,
                ridge_alpha=300.0,
                bin_count=9,
                weights=residual_weights,
            )
            sigma_suffix = "" if abs(residual_sigma_m - 80.0) < 1.0e-12 else f"_sigma{int(round(residual_sigma_m)):03d}"
            methods.append(
                FittedMethod(
                    name=(
                        f"target_weighted_disparity_{degree_name}_then_sim3_"
                        f"target_weighted_quadratic_residual_raw_forward_lut{sigma_suffix}"
                    ),
                    predict=lambda samples, target_disparity_coeff=target_disparity_coeff, target_disparity_sim_scale=target_disparity_sim_scale, target_disparity_sim_rotation=target_disparity_sim_rotation, target_disparity_sim_translation=target_disparity_sim_translation, target_disparity_target_weighted_raw_forward_lut=target_disparity_target_weighted_raw_forward_lut: (
                        apply_forward_residual_lut(
                            predict_disparity_corrected_sim3(
                                samples,
                                camera,
                                target_disparity_coeff,
                                target_disparity_sim_scale,
                                target_disparity_sim_rotation,
                                target_disparity_sim_translation,
                            ),
                            stack(samples, "raw_rdf_m")[:, 2],
                            target_disparity_target_weighted_raw_forward_lut,
                        )
                    ),
                    parameters={
                        "scale": target_disparity_sim_scale,
                        "rotation_euler_zyx_deg": target_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                        "rotation_matrix_rdf": target_disparity_sim_rotation.as_matrix().tolist(),
                        "translation_rdf_m": target_disparity_sim_translation.tolist(),
                        "raw_rdf_ray_disparity_correction_coefficients_high_to_low": target_disparity_coeff.tolist(),
                        "forward_residual_raw_forward_lut": target_disparity_target_weighted_raw_forward_lut,
                        "target_range_m": target_range_m,
                        "target_weight_sigma_m": 120.0,
                        "target_weight_floor": 0.02,
                        "target_residual_weight_sigma_m": residual_sigma_m,
                        "target_residual_weight_floor": 0.01,
                        "mean_weight": float(np.mean(target_disparity_weights)),
                        "mean_residual_weight": float(np.mean(residual_weights)),
                        "reference_basis": (
                            "Target-range-weighted measured-disparity ray correction followed by Sim(3), then "
                            "a target-range-weighted quadratic raw-forward residual LUT with residual sigma grid. "
                            "The residual fit gives higher calibration-time weight to the 1852 m operating range "
                            "while exported runtime still uses only raw RDF, measured disparity, and fixed "
                            "calibration parameters."
                        ),
                    },
                    model_complexity=complexity + 4,
                )
            )
            for residual_shrink in (0.25, 0.5, 0.75):
                if abs(residual_sigma_m - 80.0) < 1.0e-12 and abs(residual_shrink - 0.75) >= 1.0e-12:
                    shrink_sigma_suffix = ""
                else:
                    shrink_sigma_suffix = sigma_suffix
                shrink_lut = scale_residual_lut(
                    target_disparity_target_weighted_raw_forward_lut,
                    residual_scale=residual_shrink,
                    fit_basis_suffix=(
                        "target-window residual shrinkage for validation-stable deployment; "
                        "runtime x-axis remains raw RDF forward_m"
                    ),
                )
                shrink_label = f"shrink{int(round(residual_shrink * 100)):03d}"
                methods.append(
                    FittedMethod(
                        name=(
                            f"target_weighted_disparity_{degree_name}_then_sim3_"
                            f"target_weighted_quadratic_residual_raw_forward_lut"
                            f"{shrink_sigma_suffix}_{shrink_label}"
                        ),
                        predict=lambda samples, target_disparity_coeff=target_disparity_coeff, target_disparity_sim_scale=target_disparity_sim_scale, target_disparity_sim_rotation=target_disparity_sim_rotation, target_disparity_sim_translation=target_disparity_sim_translation, shrink_lut=shrink_lut: (
                            apply_forward_residual_lut(
                                predict_disparity_corrected_sim3(
                                    samples,
                                    camera,
                                    target_disparity_coeff,
                                    target_disparity_sim_scale,
                                    target_disparity_sim_rotation,
                                    target_disparity_sim_translation,
                                ),
                                stack(samples, "raw_rdf_m")[:, 2],
                                shrink_lut,
                            )
                        ),
                        parameters={
                            "scale": target_disparity_sim_scale,
                            "rotation_euler_zyx_deg": target_disparity_sim_rotation.as_euler("zyx", degrees=True).tolist(),
                            "rotation_matrix_rdf": target_disparity_sim_rotation.as_matrix().tolist(),
                            "translation_rdf_m": target_disparity_sim_translation.tolist(),
                            "raw_rdf_ray_disparity_correction_coefficients_high_to_low": target_disparity_coeff.tolist(),
                            "forward_residual_raw_forward_lut": shrink_lut,
                            "target_range_m": target_range_m,
                            "target_weight_sigma_m": 120.0,
                            "target_weight_floor": 0.02,
                            "target_residual_weight_sigma_m": residual_sigma_m,
                            "target_residual_weight_floor": 0.01,
                            "target_residual_shrink": residual_shrink,
                            "mean_weight": float(np.mean(target_disparity_weights)),
                            "mean_residual_weight": float(np.mean(residual_weights)),
                            "reference_basis": (
                                "Target-range-weighted measured-disparity ray correction followed by Sim(3), then "
                                "a shrunk target-range-weighted quadratic raw-forward residual LUT with residual "
                                "sigma grid. Shrinkage tests whether 1852 m worst-case improvement can be kept "
                                "without the validation RMSE penalty of the full residual. Runtime uses raw RDF, "
                                "measured disparity, and fixed calibration parameters only."
                            ),
                        },
                        model_complexity=complexity + 4,
                    )
                )

    sim_train = sim_scale * apply_rotation_to_rows(raw_train, sim_rotation) + sim_translation
    forward_residual = truth_train[:, 2] - sim_train[:, 2]
    if local_sweep_configs:
        local_target_weights = target_range_weights(
            train_samples,
            target_range_m=target_range_m,
            sigma_m=120.0,
            floor=0.02,
        )
        status_rows = append_local_sweep_methods(
            methods,
            configs=local_sweep_configs,
            raw_train=raw_train,
            truth_train=truth_train,
            d_meas=d_meas,
            sim_scale=sim_scale,
            sim_rotation=sim_rotation,
            sim_translation=sim_translation,
            sim_train=sim_train,
            forward_residual=forward_residual,
            target_weights=local_target_weights,
        )
        if local_sweep_status_rows is not None:
            local_sweep_status_rows.extend(status_rows)
    residual_coeff = np.polyfit(d_meas, forward_residual, deg=2)
    methods.append(
        FittedMethod(
            name="sim3_residual_disparity",
            predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, residual_coeff=residual_coeff: (
                apply_forward_residual(
                    sim_scale * apply_rotation_to_rows(stack(samples, "raw_rdf_m"), sim_rotation) + sim_translation,
                    measured_disparity(samples),
                    residual_coeff,
                )
            ),
            parameters={
                "sim3_scale": sim_scale,
                "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                "sim3_translation_rdf_m": sim_translation.tolist(),
                "residual_forward_from_disparity_coefficients_high_to_low": residual_coeff.tolist(),
            },
            model_complexity=10,
        )
    )
    sim_residual_rdf = truth_train - sim_train
    for degree, alpha, residual_cap in (
        (1, 10.0, None),
        (1, 100.0, 4.0),
        (2, 100.0, 4.0),
        (2, 1000.0, 4.0),
    ):
        raw_rdf_residual_model = fit_ridge_raw_rdf_residual_field(
            raw_train,
            sim_residual_rdf,
            degree=degree,
            alpha=alpha,
            residual_norm_cap_m=residual_cap,
        )
        suffix = f"_deg{degree}_alpha{int(alpha)}"
        if residual_cap is not None:
            suffix += f"_cap{int(residual_cap)}m"
        methods.append(
            FittedMethod(
                name=f"sim3_ridge_raw_rdf_residual_field{suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, raw_rdf_residual_model=raw_rdf_residual_model: (
                    apply_raw_rdf_residual_field(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        stack(samples, "raw_rdf_m"),
                        raw_rdf_residual_model,
                    )
                ),
                parameters={
                    "scale": sim_scale,
                    "rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "translation_rdf_m": sim_translation.tolist(),
                    "rdf_residual_raw_rdf_polynomial": raw_rdf_residual_model,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus ridge-regularized raw-RDF "
                        "polynomial vector residual field. The field is fitted with calibration-time "
                        "truth but runtime evaluation uses only raw stereo-center RDF."
                    ),
                },
                model_complexity=8 + 3 * len(raw_rdf_residual_model.get("feature_names", [])),
            )
        )
    for degree, complexity in ((0, 8), (1, 9)):
        low_degree_coeff = np.polyfit(d_meas, forward_residual, deg=degree)
        methods.append(
            FittedMethod(
                name=f"sim3_residual_disparity_deg{degree}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, low_degree_coeff=low_degree_coeff: (
                    apply_forward_residual(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        measured_disparity(samples),
                        low_degree_coeff,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "residual_forward_from_disparity_coefficients_high_to_low": low_degree_coeff.tolist(),
                    "residual_polynomial_degree": degree,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus low-degree polynomial "
                        "forward residual correction as a function of measured disparity"
                    ),
                },
                model_complexity=complexity,
            )
        )
    target_residual_weights = target_range_weights(
        train_samples,
        target_range_m=target_range_m,
        sigma_m=120.0,
        floor=0.02,
    )
    for degree, complexity in ((1, 10), (2, 11)):
        target_forward_coeff = fit_weighted_polyfit(
            d_meas,
            forward_residual,
            degree=degree,
            weights=target_residual_weights,
        )
        suffix = f"_deg{degree}" if degree != 2 else ""
        methods.append(
            FittedMethod(
                name=f"sim3_target_weighted_residual_disparity{suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, target_forward_coeff=target_forward_coeff: (
                    apply_forward_residual(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        measured_disparity(samples),
                        target_forward_coeff,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "residual_forward_from_disparity_coefficients_high_to_low": target_forward_coeff.tolist(),
                    "residual_polynomial_degree": degree,
                    "target_range_m": target_range_m,
                    "target_weight_sigma_m": 120.0,
                    "target_weight_floor": 0.02,
                    "mean_weight": float(np.mean(target_residual_weights)),
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus target-range-weighted polynomial "
                        "forward residual correction as a function of measured disparity"
                    ),
                },
                model_complexity=complexity,
            )
        )
    ridge_forward_coeff, ridge_forward_info = fit_best_ridge_polyfit(d_meas, forward_residual)
    methods.append(
        FittedMethod(
            name="sim3_ridge_residual_disparity",
            predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, ridge_forward_coeff=ridge_forward_coeff: (
                apply_forward_residual(
                    predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                    measured_disparity(samples),
                    ridge_forward_coeff,
                )
            ),
            parameters={
                "sim3_scale": sim_scale,
                "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                "sim3_translation_rdf_m": sim_translation.tolist(),
                "residual_forward_from_disparity_coefficients_high_to_low": ridge_forward_coeff.tolist(),
                "ridge_residual_fit": ridge_forward_info,
                "reference_basis": (
                    "Sim(3) estimated-to-truth registration plus ridge-regularized polynomial "
                    "forward residual correction as a function of measured disparity"
                ),
            },
            model_complexity=8 + int(ridge_forward_info["degree"]),
        )
    )
    binned_forward_lut = fit_binned_residual_lut(d_meas, forward_residual)
    methods.append(
        FittedMethod(
            name="sim3_binned_residual_disparity",
            predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, binned_forward_lut=binned_forward_lut: (
                apply_forward_residual_lut(
                    predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                    measured_disparity(samples),
                    binned_forward_lut,
                )
            ),
            parameters={
                "sim3_scale": sim_scale,
                "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                "sim3_translation_rdf_m": sim_translation.tolist(),
                "forward_residual_disparity_lut": binned_forward_lut,
                "reference_basis": (
                    "Sim(3) estimated-to-truth registration plus quantile-binned median forward "
                    "residual LUT over measured disparity, equivalent to a piecewise-linear calibration curve"
                ),
            },
            model_complexity=8 + len(binned_forward_lut.get("x_values_px", [])),
        )
    )
    for bin_count in (7, 11):
        raw_forward_lut = relabel_residual_lut_x_values(
            fit_binned_residual_lut(
                raw_train[:, 2],
                forward_residual,
                bin_count=bin_count,
            ),
            x_key="x_values_m",
            fit_basis_suffix="raw RDF forward_m",
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"sim3_binned_residual_raw_forward{suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, raw_forward_lut=raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        raw_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_raw_forward_lut": raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus quantile-binned median forward "
                        "residual LUT over raw RDF forward distance"
                    ),
                },
                model_complexity=8 + len(raw_forward_lut.get("x_values_m", [])),
            )
        )
    for kernel_sigma_m in (60.0, 120.0, 250.0):
        kernel_raw_forward_lut = relabel_residual_lut_x_values(
            fit_kernel_smoothed_residual_lut(
                raw_train[:, 2],
                forward_residual,
                support_count=11,
                bandwidth=kernel_sigma_m,
            ),
            x_key="x_values_m",
            fit_basis_suffix="raw RDF forward_m",
        )
        sigma_label = int(round(kernel_sigma_m))
        methods.append(
            FittedMethod(
                name=f"sim3_kernel_smoothed_residual_raw_forward_sigma{sigma_label:03d}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, kernel_raw_forward_lut=kernel_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        kernel_raw_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_raw_forward_lut": kernel_raw_forward_lut,
                    "kernel_bandwidth_m": kernel_sigma_m,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus Gaussian-kernel "
                        "Nadaraya-Watson forward residual regression over raw RDF forward distance"
                    ),
                },
                model_complexity=8 + len(kernel_raw_forward_lut.get("x_values_m", [])),
            )
        )
    for local_linear_sigma_m in (60.0, 120.0, 250.0):
        local_linear_raw_forward_lut = relabel_residual_lut_x_values(
            fit_local_linear_residual_lut(
                raw_train[:, 2],
                forward_residual,
                support_count=11,
                bandwidth=local_linear_sigma_m,
            ),
            x_key="x_values_m",
            fit_basis_suffix="raw RDF forward_m",
        )
        sigma_label = int(round(local_linear_sigma_m))
        methods.append(
            FittedMethod(
                name=f"sim3_local_linear_residual_raw_forward_sigma{sigma_label:03d}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, local_linear_raw_forward_lut=local_linear_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        local_linear_raw_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_raw_forward_lut": local_linear_raw_forward_lut,
                    "local_linear_bandwidth_m": local_linear_sigma_m,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus Gaussian-kernel local-linear "
                        "forward residual regression over raw RDF forward distance. Local-linear "
                        "fitting is evaluated as a boundary-bias-reduced alternative to local-constant "
                        "Nadaraya-Watson smoothing, then exported as a fixed runtime LUT."
                    ),
                },
                model_complexity=8 + len(local_linear_raw_forward_lut.get("x_values_m", [])),
            )
        )
    for smoothing_scale in (0.25, 1.0, 4.0):
        spline_raw_forward_lut = relabel_residual_lut_x_values(
            fit_smoothing_spline_residual_lut(
                raw_train[:, 2],
                forward_residual,
                support_count=11,
                smoothing_factor_scale=smoothing_scale,
            ),
            x_key="x_values_m",
            fit_basis_suffix="raw RDF forward_m",
        )
        smooth_label = int(round(smoothing_scale * 100.0))
        methods.append(
            FittedMethod(
                name=f"sim3_spline_residual_raw_forward_smooth{smooth_label:03d}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, spline_raw_forward_lut=spline_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        spline_raw_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_raw_forward_lut": spline_raw_forward_lut,
                    "spline_smoothing_factor_scale": smoothing_scale,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus regularized cubic smoothing-spline "
                        "forward residual over raw RDF forward distance. The smoothing-spline family is "
                        "used as a low-curvature calibration curve and exported as a fixed runtime LUT."
                    ),
                },
                model_complexity=8 + len(spline_raw_forward_lut.get("x_values_m", [])),
            )
        )
    for bin_count in (7, 11):
        monotone_raw_forward_lut = relabel_residual_lut_x_values(
            fit_monotone_binned_residual_lut(
                raw_train[:, 2],
                forward_residual,
                bin_count=bin_count,
            ),
            x_key="x_values_m",
            fit_basis_suffix="raw RDF forward_m",
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"sim3_monotone_binned_residual_raw_forward{suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, monotone_raw_forward_lut=monotone_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        monotone_raw_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_raw_forward_lut": monotone_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus isotonic-regression "
                        "monotone forward residual LUT over raw RDF forward distance"
                    ),
                },
                model_complexity=8 + len(monotone_raw_forward_lut.get("x_values_m", [])),
            )
        )
    for bin_count in (7, 11):
        normalized_raw_forward_lut = fit_range_normalized_residual_lut(
            raw_train[:, 2],
            forward_residual,
            bin_count=bin_count,
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"sim3_range_normalized_binned_residual_raw_forward{suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, normalized_raw_forward_lut=normalized_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        normalized_raw_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_raw_forward_lut": normalized_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus range-normalized forward "
                        "residual LUT over raw RDF forward distance; residual/forward is fitted then "
                        "exported as meter residuals"
                    ),
                },
                model_complexity=8 + len(normalized_raw_forward_lut.get("x_values_m", [])),
            )
        )
    for bin_count in (7, 11):
        inverse_depth_raw_forward_lut = fit_inverse_depth_residual_lut(
            raw_train[:, 2],
            sim_train[:, 2],
            truth_train[:, 2],
            bin_count=bin_count,
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"sim3_inverse_depth_binned_residual_raw_forward{suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, inverse_depth_raw_forward_lut=inverse_depth_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        inverse_depth_raw_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_raw_forward_lut": inverse_depth_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus inverse-depth forward "
                        "residual LUT over raw RDF forward distance; inverse-depth delta is fitted "
                        "then exported as meter residuals"
                    ),
                },
                model_complexity=8 + len(inverse_depth_raw_forward_lut.get("x_values_m", [])),
            )
        )
        monotone_inverse_depth_raw_forward_lut = fit_monotone_inverse_depth_residual_lut(
            raw_train[:, 2],
            sim_train[:, 2],
            truth_train[:, 2],
            bin_count=bin_count,
        )
        methods.append(
            FittedMethod(
                name=f"sim3_monotone_inverse_depth_binned_residual_raw_forward{suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, monotone_inverse_depth_raw_forward_lut=monotone_inverse_depth_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        monotone_inverse_depth_raw_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_raw_forward_lut": monotone_inverse_depth_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus isotonic-regression constrained "
                        "inverse-depth forward residual LUT over raw RDF forward distance; inverse-depth "
                        "delta is fitted, projected monotone, then exported as meter residuals"
                    ),
                },
                model_complexity=8 + len(monotone_inverse_depth_raw_forward_lut.get("x_values_m", [])),
            )
        )
    quadratic_raw_forward_lut = fit_polynomial_raw_forward_residual_lut(
        raw_train[:, 2],
        forward_residual,
        degree=2,
        ridge_alpha=10.0,
        bin_count=7,
    )
    methods.append(
        FittedMethod(
            name="sim3_quadratic_residual_raw_forward_lut",
            predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, quadratic_raw_forward_lut=quadratic_raw_forward_lut: (
                apply_forward_residual_lut(
                    predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                    stack(samples, "raw_rdf_m")[:, 2],
                    quadratic_raw_forward_lut,
                )
            ),
            parameters={
                "sim3_scale": sim_scale,
                "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                "sim3_translation_rdf_m": sim_translation.tolist(),
                "forward_residual_raw_forward_lut": quadratic_raw_forward_lut,
                "reference_basis": (
                    "Sim(3) estimated-to-truth registration plus ridge-regularized quadratic "
                    "forward residual over raw RDF forward distance, exported as a runtime LUT"
                ),
            },
            model_complexity=11,
        )
    )
    for bin_count, residual_shrink in (
        (3, 0.25),
        (3, 0.5),
        (5, 0.25),
        (5, 0.5),
        (7, 0.25),
        (7, 0.5),
        (7, 0.75),
    ):
        quadratic_local_raw_forward_lut = fit_polynomial_plus_binned_raw_forward_residual_lut(
            raw_train[:, 2],
            forward_residual,
            degree=2,
            ridge_alpha=10.0,
            bin_count=bin_count,
            residual_shrink=residual_shrink,
        )
        bin_suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        shrink_suffix = "" if abs(residual_shrink - 0.5) < 1.0e-12 else f"_shrink{int(round(residual_shrink * 100)):03d}"
        methods.append(
            FittedMethod(
                name=f"sim3_quadratic_local_residual_raw_forward_lut{bin_suffix}{shrink_suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, quadratic_local_raw_forward_lut=quadratic_local_raw_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        stack(samples, "raw_rdf_m")[:, 2],
                        quadratic_local_raw_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_raw_forward_lut": quadratic_local_raw_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "local_residual_shrink": residual_shrink,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus ridge-regularized quadratic "
                        "forward residual and shrunk local residual LUT over raw RDF forward distance"
                    ),
                },
                model_complexity=11 + len(quadratic_local_raw_forward_lut.get("x_values_m", [])),
            )
        )
    for bin_count in (7, 11):
        for blend_alpha in (0.25, 0.5, 0.75):
            blend_disparity_lut = scale_residual_lut(
                fit_binned_residual_lut(
                    d_meas,
                    forward_residual,
                    bin_count=bin_count,
                ),
                residual_scale=blend_alpha,
                fit_basis_suffix="dual-key residual blend: measured disparity share",
            )
            blend_raw_forward_lut = scale_residual_lut(
                relabel_residual_lut_x_values(
                    fit_binned_residual_lut(
                        raw_train[:, 2],
                        forward_residual,
                        bin_count=bin_count,
                    ),
                    x_key="x_values_m",
                    fit_basis_suffix="raw RDF forward_m",
                ),
                residual_scale=1.0 - blend_alpha,
                fit_basis_suffix="dual-key residual blend: raw forward share",
            )
            suffix = "" if bin_count == 7 else f"_bins{bin_count}"
            alpha_suffix = "" if abs(blend_alpha - 0.5) < 1.0e-12 else f"_alpha{int(round(blend_alpha * 100)):03d}"
            methods.append(
                FittedMethod(
                    name=f"sim3_blended_binned_residual_disparity_raw_forward{suffix}{alpha_suffix}",
                    predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, blend_disparity_lut=blend_disparity_lut, blend_raw_forward_lut=blend_raw_forward_lut: (
                        apply_forward_residual_lut(
                            apply_forward_residual_lut(
                                predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                                measured_disparity(samples),
                                blend_disparity_lut,
                            ),
                            stack(samples, "raw_rdf_m")[:, 2],
                            blend_raw_forward_lut,
                        )
                    ),
                    parameters={
                        "sim3_scale": sim_scale,
                        "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                        "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                        "sim3_translation_rdf_m": sim_translation.tolist(),
                        "forward_residual_disparity_lut": blend_disparity_lut,
                        "forward_residual_raw_forward_lut": blend_raw_forward_lut,
                        "residual_lut_bin_count": bin_count,
                        "residual_blend_alpha_disparity": blend_alpha,
                        "reference_basis": (
                            "Sim(3) estimated-to-truth registration plus a grid-searched blend of "
                            "quantile-binned median forward residual LUTs keyed by measured disparity "
                            "and raw RDF forward distance"
                        ),
                    },
                    model_complexity=8
                    + len(blend_disparity_lut.get("x_values_px", []))
                    + len(blend_raw_forward_lut.get("x_values_m", [])),
                )
            )
    for bin_count in (7, 11):
        gate_max_difference_m = 0.75
        gate_alpha_disagree = 0.0
        gated_disparity_lut = scale_residual_lut(
            fit_binned_residual_lut(
                d_meas,
                forward_residual,
                bin_count=bin_count,
            ),
            residual_scale=0.5,
            fit_basis_suffix="agreement-gated dual-key residual blend: measured disparity share",
        )
        gated_raw_forward_lut = scale_residual_lut(
            relabel_residual_lut_x_values(
                fit_binned_residual_lut(
                    raw_train[:, 2],
                    forward_residual,
                    bin_count=bin_count,
                ),
                x_key="x_values_m",
                fit_basis_suffix="raw RDF forward_m",
            ),
            residual_scale=0.5,
            fit_basis_suffix="agreement-gated dual-key residual blend: raw forward share",
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"sim3_agreement_gated_binned_residual_disparity_raw_forward{suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, gated_disparity_lut=gated_disparity_lut, gated_raw_forward_lut=gated_raw_forward_lut, gate_max_difference_m=gate_max_difference_m, gate_alpha_disagree=gate_alpha_disagree: (
                    apply_agreement_gated_forward_residual_luts(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        measured_disparity(samples),
                        stack(samples, "raw_rdf_m")[:, 2],
                        gated_disparity_lut,
                        gated_raw_forward_lut,
                        max_abs_difference_m=gate_max_difference_m,
                        alpha_disagree=gate_alpha_disagree,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_disparity_lut": gated_disparity_lut,
                    "forward_residual_raw_forward_lut": gated_raw_forward_lut,
                    "forward_residual_agreement_gate": {
                        "max_abs_difference_m": gate_max_difference_m,
                        "alpha_disagree": gate_alpha_disagree,
                    },
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus 50/50 dual-key forward residual LUTs; "
                        "runtime suppresses the residual when disparity-key and raw-forward-key residuals disagree"
                    ),
                },
                model_complexity=10
                + len(gated_disparity_lut.get("x_values_px", []))
                + len(gated_raw_forward_lut.get("x_values_m", [])),
            )
        )
    for bin_count in (5, 9, 11):
        variant_forward_lut = fit_binned_residual_lut(d_meas, forward_residual, bin_count=bin_count)
        methods.append(
            FittedMethod(
                name=f"sim3_binned_residual_disparity_bins{bin_count}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, variant_forward_lut=variant_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        measured_disparity(samples),
                        variant_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_disparity_lut": variant_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus quantile-binned median forward "
                        "residual LUT over measured disparity; bin-count grid-search candidate"
                    ),
                },
                model_complexity=8 + len(variant_forward_lut.get("x_values_px", [])),
            )
        )
    for kernel_support_count in (5, 7, 11):
        for kernel_sigma_px in (2.0, 4.0, 8.0):
            kernel_disparity_lut = fit_kernel_smoothed_residual_lut(
                d_meas,
                forward_residual,
                support_count=kernel_support_count,
                bandwidth=kernel_sigma_px,
            )
            sigma_label = int(round(kernel_sigma_px * 10.0))
            support_suffix = "" if kernel_support_count == 11 else f"_support{kernel_support_count}"
            methods.append(
                FittedMethod(
                    name=f"sim3_kernel_smoothed_binned_residual_disparity_sigma{sigma_label:03d}{support_suffix}",
                    predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, kernel_disparity_lut=kernel_disparity_lut: (
                        apply_forward_residual_lut(
                            predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                            measured_disparity(samples),
                            kernel_disparity_lut,
                        )
                    ),
                    parameters={
                        "sim3_scale": sim_scale,
                        "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                        "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                        "sim3_translation_rdf_m": sim_translation.tolist(),
                        "forward_residual_disparity_lut": kernel_disparity_lut,
                        "kernel_bandwidth_px": kernel_sigma_px,
                        "kernel_support_count": kernel_support_count,
                        "reference_basis": (
                            "Sim(3) estimated-to-truth registration plus Gaussian-kernel "
                            "Nadaraya-Watson forward residual regression over measured disparity; "
                            "support-count grid-search candidate"
                        ),
                    },
                    model_complexity=8 + len(kernel_disparity_lut.get("x_values_px", [])),
                )
            )
    for local_linear_support_count in (5, 7, 11):
        for local_linear_sigma_px in (2.0, 4.0, 8.0):
            local_linear_disparity_lut = fit_local_linear_residual_lut(
                d_meas,
                forward_residual,
                support_count=local_linear_support_count,
                bandwidth=local_linear_sigma_px,
            )
            sigma_label = int(round(local_linear_sigma_px * 10.0))
            support_suffix = "" if local_linear_support_count == 11 else f"_support{local_linear_support_count}"
            methods.append(
                FittedMethod(
                    name=f"sim3_local_linear_residual_disparity_sigma{sigma_label:03d}{support_suffix}",
                    predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, local_linear_disparity_lut=local_linear_disparity_lut: (
                        apply_forward_residual_lut(
                            predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                            measured_disparity(samples),
                            local_linear_disparity_lut,
                        )
                    ),
                    parameters={
                        "sim3_scale": sim_scale,
                        "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                        "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                        "sim3_translation_rdf_m": sim_translation.tolist(),
                        "forward_residual_disparity_lut": local_linear_disparity_lut,
                        "local_linear_bandwidth_px": local_linear_sigma_px,
                        "local_linear_support_count": local_linear_support_count,
                        "reference_basis": (
                            "Sim(3) estimated-to-truth registration plus Gaussian-kernel local-linear "
                            "forward residual regression over measured disparity. Local-linear fitting "
                            "is evaluated as a boundary-bias-reduced alternative to local-constant "
                            "Nadaraya-Watson smoothing, then exported as a fixed runtime LUT."
                        ),
                    },
                    model_complexity=8 + len(local_linear_disparity_lut.get("x_values_px", [])),
                )
            )
    for smoothing_scale in (0.25, 1.0, 4.0):
        spline_disparity_lut = fit_smoothing_spline_residual_lut(
            d_meas,
            forward_residual,
            support_count=11,
            smoothing_factor_scale=smoothing_scale,
        )
        smooth_label = int(round(smoothing_scale * 100.0))
        methods.append(
            FittedMethod(
                name=f"sim3_spline_residual_disparity_smooth{smooth_label:03d}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, spline_disparity_lut=spline_disparity_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        measured_disparity(samples),
                        spline_disparity_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_disparity_lut": spline_disparity_lut,
                    "spline_smoothing_factor_scale": smoothing_scale,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus regularized cubic smoothing-spline "
                        "forward residual over measured disparity. The smoothing-spline family is used "
                        "as a low-curvature calibration curve and exported as a fixed runtime LUT."
                    ),
                },
                model_complexity=8 + len(spline_disparity_lut.get("x_values_px", [])),
            )
        )
    for bin_count in (7, 11):
        smoothed_forward_lut = fit_smoothed_binned_residual_lut(
            d_meas,
            forward_residual,
            bin_count=bin_count,
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"sim3_smoothed_binned_residual_disparity{suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, smoothed_forward_lut=smoothed_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        measured_disparity(samples),
                        smoothed_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_disparity_lut": smoothed_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus smoothed quantile-binned median "
                        "forward residual LUT over measured disparity"
                    ),
                },
                model_complexity=8 + len(smoothed_forward_lut.get("x_values_px", [])),
            )
        )
    for bin_count in (7, 11):
        capped_forward_lut = fit_capped_binned_residual_lut(
            d_meas,
            forward_residual,
            bin_count=bin_count,
        )
        suffix = "" if bin_count == 7 else f"_bins{bin_count}"
        methods.append(
            FittedMethod(
                name=f"sim3_capped_binned_residual_disparity{suffix}",
                predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, capped_forward_lut=capped_forward_lut: (
                    apply_forward_residual_lut(
                        predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                        measured_disparity(samples),
                        capped_forward_lut,
                    )
                ),
                parameters={
                    "sim3_scale": sim_scale,
                    "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                    "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                    "sim3_translation_rdf_m": sim_translation.tolist(),
                    "forward_residual_disparity_lut": capped_forward_lut,
                    "residual_lut_bin_count": bin_count,
                    "reference_basis": (
                        "Sim(3) estimated-to-truth registration plus robustly capped quantile-binned "
                        "median forward residual LUT over measured disparity"
                    ),
                },
                model_complexity=8 + len(capped_forward_lut.get("x_values_px", [])),
            )
        )
    binned_rdf_lut = {
        axis: fit_binned_residual_lut(d_meas, truth_train[:, axis_index] - sim_train[:, axis_index])
        for axis_index, axis in enumerate(("right", "down", "forward"))
    }
    methods.append(
        FittedMethod(
            name="sim3_binned_rdf_residual_disparity",
            predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, binned_rdf_lut=binned_rdf_lut: (
                apply_rdf_residual_lut(
                    predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                    measured_disparity(samples),
                    binned_rdf_lut,
                )
            ),
            parameters={
                "sim3_scale": sim_scale,
                "sim3_rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "sim3_rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                "sim3_translation_rdf_m": sim_translation.tolist(),
                "rdf_residual_disparity_lut": binned_rdf_lut,
                "reference_basis": (
                    "Sim(3) estimated-to-truth registration plus per-RDF-axis quantile-binned median "
                    "residual LUTs over measured disparity with piecewise-linear interpolation"
                ),
            },
            model_complexity=8 + sum(len(lut.get("x_values_px", [])) for lut in binned_rdf_lut.values()),
        )
    )
    rdf_residual_coefficients = {
        "right": np.polyfit(d_meas, truth_train[:, 0] - sim_train[:, 0], deg=2),
        "down": np.polyfit(d_meas, truth_train[:, 1] - sim_train[:, 1], deg=2),
        "forward": residual_coeff,
    }
    methods.append(
        FittedMethod(
            name="sim3_rdf_residual_disparity",
            predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, rdf_residual_coefficients=rdf_residual_coefficients: (
                apply_rdf_residual(
                    predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                    measured_disparity(samples),
                    rdf_residual_coefficients,
                )
            ),
            parameters={
                "scale": sim_scale,
                "rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": sim_translation.tolist(),
                "rdf_residual_disparity_coefficients_high_to_low": {
                    axis: coefficients.tolist()
                    for axis, coefficients in rdf_residual_coefficients.items()
                },
                "reference_basis": (
                    "Sim(3) estimated-to-truth registration plus per-RDF-axis polynomial residual "
                    "correction as a function of measured disparity"
                ),
            },
            model_complexity=13,
        )
    )
    target_rdf_coefficients = {
        axis: fit_weighted_polyfit(
            d_meas,
            truth_train[:, axis_index] - sim_train[:, axis_index],
            degree=2,
            weights=target_residual_weights,
        )
        for axis_index, axis in enumerate(("right", "down", "forward"))
    }
    methods.append(
        FittedMethod(
            name="sim3_target_weighted_rdf_residual_disparity",
            predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, target_rdf_coefficients=target_rdf_coefficients: (
                apply_rdf_residual(
                    predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                    measured_disparity(samples),
                    target_rdf_coefficients,
                )
            ),
            parameters={
                "scale": sim_scale,
                "rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": sim_translation.tolist(),
                "rdf_residual_disparity_coefficients_high_to_low": {
                    axis: coefficients.tolist()
                    for axis, coefficients in target_rdf_coefficients.items()
                },
                "target_range_m": target_range_m,
                "target_weight_sigma_m": 120.0,
                "target_weight_floor": 0.02,
                "mean_weight": float(np.mean(target_residual_weights)),
                "reference_basis": (
                    "Sim(3) estimated-to-truth registration plus target-range-weighted per-RDF-axis "
                    "polynomial residual correction as a function of measured disparity"
                ),
            },
            model_complexity=14,
        )
    )
    ridge_rdf_coefficients: dict[str, np.ndarray] = {}
    ridge_rdf_info: dict[str, object] = {}
    for axis_index, axis in enumerate(("right", "down", "forward")):
        coefficients, info = fit_best_ridge_polyfit(d_meas, truth_train[:, axis_index] - sim_train[:, axis_index])
        ridge_rdf_coefficients[axis] = coefficients
        ridge_rdf_info[axis] = info
    methods.append(
        FittedMethod(
            name="sim3_ridge_rdf_residual_disparity",
            predict=lambda samples, sim_scale=sim_scale, sim_rotation=sim_rotation, sim_translation=sim_translation, ridge_rdf_coefficients=ridge_rdf_coefficients: (
                apply_rdf_residual(
                    predict_sim3(stack(samples, "raw_rdf_m"), sim_scale, sim_rotation, sim_translation),
                    measured_disparity(samples),
                    ridge_rdf_coefficients,
                )
            ),
            parameters={
                "scale": sim_scale,
                "rotation_euler_zyx_deg": sim_rotation.as_euler("zyx", degrees=True).tolist(),
                "rotation_matrix_rdf": sim_rotation.as_matrix().tolist(),
                "translation_rdf_m": sim_translation.tolist(),
                "rdf_residual_disparity_coefficients_high_to_low": {
                    axis: coefficients.tolist()
                    for axis, coefficients in ridge_rdf_coefficients.items()
                },
                "ridge_residual_fit_by_axis": ridge_rdf_info,
                "reference_basis": (
                    "Sim(3) estimated-to-truth registration plus ridge-regularized per-RDF-axis "
                    "polynomial residual correction as a function of measured disparity"
                ),
            },
            model_complexity=9 + sum(int(info["degree"]) for info in ridge_rdf_info.values()),
        )
    )
    return methods


def apply_forward_residual(points: np.ndarray, d_meas: np.ndarray, coefficients: np.ndarray) -> np.ndarray:
    corrected = np.array(points, dtype=float, copy=True)
    corrected[:, 2] += np.polyval(coefficients, d_meas)
    return corrected


def apply_rdf_residual(points: np.ndarray, d_meas: np.ndarray, coefficients_by_axis: Mapping[str, np.ndarray]) -> np.ndarray:
    corrected = np.array(points, dtype=float, copy=True)
    for column, axis in enumerate(("right", "down", "forward")):
        coefficients = coefficients_by_axis.get(axis)
        if coefficients is not None:
            corrected[:, column] += np.polyval(coefficients, d_meas)
    return corrected


def rmse(values: np.ndarray) -> float:
    flat = np.asarray(values, dtype=float).reshape(-1)
    return float(math.sqrt(np.mean(flat * flat))) if flat.size else float("nan")


def evaluate_predictions(
    samples: Sequence[CalibrationSample],
    predictions: np.ndarray,
    *,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, float | int]:
    truth = stack(samples, "truth_rdf_m")
    errors = predictions - truth
    truth_range = np.linalg.norm(truth, axis=1)
    pred_range = np.linalg.norm(predictions, axis=1)
    range_errors = pred_range - truth_range
    target_mask = np.abs(truth_range - target_range_m) <= target_window_m
    target_errors = range_errors[target_mask]
    result: dict[str, float | int] = {
        "sample_count": len(samples),
        "range_rmse_m": rmse(range_errors),
        "right_rmse_m": rmse(errors[:, 0]),
        "down_rmse_m": rmse(errors[:, 1]),
        "forward_rmse_m": rmse(errors[:, 2]),
        "position_rmse_m": rmse(np.linalg.norm(errors, axis=1)),
        "mean_range_error_m": float(np.mean(range_errors)) if len(range_errors) else float("nan"),
        "abs_mean_range_error_m": float(abs(np.mean(range_errors))) if len(range_errors) else float("nan"),
        "target_window_count": int(target_errors.size),
        "target_abs_mean_error_m": float(abs(np.mean(target_errors))) if target_errors.size else float("inf"),
        "target_rmse_m": rmse(target_errors) if target_errors.size else float("inf"),
        "target_max_abs_error_m": float(np.max(np.abs(target_errors))) if target_errors.size else float("inf"),
    }
    return result


def rows_for_samples(samples: Sequence[CalibrationSample]) -> list[dict[str, object]]:
    rows = []
    for sample in samples:
        rows.append(
            {
                "sample_id": sample.sample_id,
                "split": sample.split,
                "t_s": f"{sample.t_s:.6f}",
                "ned_axis_contract": NED_AXIS_CONTRACT,
                "rdf_axis_contract": RDF_AXIS_CONTRACT,
                "truth_source_frame": sample.truth_source_frame,
                "truth_source_axis_order": sample.truth_source_axis_order,
                "truth_source_fields": sample.truth_source_fields,
                "truth_rdf_transform_contract": sample.truth_rdf_transform_contract,
                "truth_source_origin_m": sample.truth_source_origin_m,
                "truth_source_to_stereo_rdf_translation_m": sample.truth_source_to_stereo_rdf_translation_m,
                "truth_source_to_stereo_rdf_rotation_matrix": sample.truth_source_to_stereo_rdf_rotation_matrix,
                "truth_north_m": f"{sample.truth_ned_m[0]:.9f}",
                "truth_east_m": f"{sample.truth_ned_m[1]:.9f}",
                "truth_down_ned_m": f"{sample.truth_ned_m[2]:.9f}",
                "truth_right_m": f"{sample.truth_rdf_m[0]:.9f}",
                "truth_down_m": f"{sample.truth_rdf_m[1]:.9f}",
                "truth_forward_m": f"{sample.truth_rdf_m[2]:.9f}",
                "truth_range_m": f"{sample.truth_range_m:.9f}",
                "raw_right_m": f"{sample.raw_rdf_m[0]:.9f}",
                "raw_down_m": f"{sample.raw_rdf_m[1]:.9f}",
                "raw_forward_m": f"{sample.raw_rdf_m[2]:.9f}",
                "raw_range_m": f"{sample.raw_range_m:.9f}",
                "left_u_px": f"{sample.left_u_px:.9f}",
                "left_v_px": f"{sample.left_v_px:.9f}",
                "measured_disparity_px": f"{sample.measured_disparity_px:.9f}",
                "true_disparity_px": f"{sample.true_disparity_px:.9f}",
                "raw_disparity_residual_px": f"{sample.measured_disparity_px - sample.true_disparity_px:.9f}",
            }
        )
    return rows


def prediction_rows(
    samples: Sequence[CalibrationSample],
    predictions_by_method: dict[str, np.ndarray],
) -> list[dict[str, object]]:
    rows = []
    truth = stack(samples, "truth_rdf_m")
    truth_range = np.linalg.norm(truth, axis=1)
    for method_name, predictions in predictions_by_method.items():
        errors = predictions - truth
        pred_range = np.linalg.norm(predictions, axis=1)
        for index, sample in enumerate(samples):
            rows.append(
                {
                    "method": method_name,
                    "sample_id": sample.sample_id,
                    "split": sample.split,
                    "rdf_axis_contract": RDF_AXIS_CONTRACT,
                    "pred_right_m": f"{predictions[index, 0]:.9f}",
                    "pred_down_m": f"{predictions[index, 1]:.9f}",
                    "pred_forward_m": f"{predictions[index, 2]:.9f}",
                    "pred_range_m": f"{pred_range[index]:.9f}",
                    "truth_right_m": f"{truth[index, 0]:.9f}",
                    "truth_down_m": f"{truth[index, 1]:.9f}",
                    "truth_forward_m": f"{truth[index, 2]:.9f}",
                    "truth_range_m": f"{truth_range[index]:.9f}",
                    "error_right_m": f"{errors[index, 0]:.9f}",
                    "error_down_m": f"{errors[index, 1]:.9f}",
                    "error_forward_m": f"{errors[index, 2]:.9f}",
                    "range_error_m": f"{pred_range[index] - truth_range[index]:.9f}",
                    "measured_disparity_px": f"{sample.measured_disparity_px:.9f}",
                    "true_disparity_px": f"{sample.true_disparity_px:.9f}",
                }
            )
    return rows


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def json_safe(value: object) -> object:
    if isinstance(value, Mapping):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        value = float(value)
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    return value


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_safe(payload), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def runtime_profile_for_method(method: FittedMethod, *, experiment_id: str) -> dict[str, object] | None:
    if "d_corr_coefficients_high_to_low" in method.parameters or "inverse_depth_coefficients_high_to_low" in method.parameters:
        correction_space = str(method.parameters.get("correction_space", "raw_rdf_ray_scale"))
        return {
            "mode": "estimated_to_truth_disparity_poly",
            "profile_id": f"{experiment_id}-{method.name}",
            "source_experiment": experiment_id,
            "method": method.name,
            "axis_contract": (
                "internal_policy_vector=RDF(right,down,forward); "
                "legacy_stereodistance_position_topics=FRD_range(forward,right,down,range); "
                "disparity correction scales the raw RDF ray by corrected_forward/raw_forward"
            ),
            "training_contract": (
                "Ray-scale correction parameters are fitted from calibration data with PX4/RTK truth; "
                "runtime apply uses only raw stereo RDF, measured disparity when required, and calibration-fixed parameters."
            ),
            "runtime_inputs": [
                "raw_right_m",
                "raw_down_m",
                "raw_forward_m",
                "raw_disparity_px",
            ],
            "forbidden_runtime_inputs": [
                "calibration_truth",
                "px4_rtk_truth",
                "true_disparity",
                "true_image_feature_location",
            ],
            "correction_space": correction_space,
            **(
                {"d_corr_coefficients_high_to_low": method.parameters["d_corr_coefficients_high_to_low"]}
                if "d_corr_coefficients_high_to_low" in method.parameters
                else {}
            ),
            **(
                {"inverse_depth_coefficients_high_to_low": method.parameters["inverse_depth_coefficients_high_to_low"]}
                if "inverse_depth_coefficients_high_to_low" in method.parameters
                else {}
            ),
        }
    if {"affine_matrix_rdf", "translation_rdf_m"}.issubset(method.parameters):
        profile = {
            "mode": "estimated_to_truth_affine_rdf",
            "profile_id": f"{experiment_id}-{method.name}",
            "source_experiment": experiment_id,
            "method": method.name,
            "axis_contract": (
                "internal_policy_vector=RDF(right,down,forward); "
                "legacy_stereodistance_position_topics=FRD_range(forward,right,down,range)"
            ),
            "training_contract": (
                "Affine RDF parameters are fitted from calibration data with PX4/RTK truth; "
                "runtime apply uses only raw stereo RDF, measured disparity, and calibration-fixed parameters."
            ),
            "runtime_inputs": [
                "raw_right_m",
                "raw_down_m",
                "raw_forward_m",
                "raw_disparity_px",
            ],
            "forbidden_runtime_inputs": [
                "calibration_truth",
                "px4_rtk_truth",
                "true_disparity",
                "true_image_feature_location",
            ],
            "correction_space": "rdf_affine_estimated_to_truth",
            "affine_matrix_rdf": method.parameters["affine_matrix_rdf"],
            "translation_rdf_m": method.parameters["translation_rdf_m"],
        }
        if "forward_residual_disparity_lut" in method.parameters:
            profile["forward_residual_disparity_lut"] = method.parameters["forward_residual_disparity_lut"]
        if "forward_residual_raw_forward_lut" in method.parameters:
            profile["forward_residual_raw_forward_lut"] = method.parameters["forward_residual_raw_forward_lut"]
        if "rdf_residual_disparity_lut" in method.parameters:
            profile["rdf_residual_disparity_lut"] = method.parameters["rdf_residual_disparity_lut"]
        return profile
    if {"scale", "rotation_matrix_rdf", "rotation_euler_zyx_deg", "translation_rdf_m"}.issubset(method.parameters):
        training_contract = (
            "Sim(3) parameters are fitted from calibration data with PX4/RTK truth; "
            "runtime apply uses only raw stereo RDF and calibration-fixed parameters."
        )
        if "rdf_residual_disparity_coefficients_high_to_low" in method.parameters:
            training_contract = (
                "Sim(3) and per-RDF-axis residual disparity parameters are fitted from calibration data with PX4/RTK truth; "
                "runtime apply uses only raw stereo RDF, measured disparity, and calibration-fixed parameters."
            )
        if "rdf_residual_raw_rdf_polynomial" in method.parameters:
            training_contract = (
                "Sim(3) and raw-RDF polynomial residual-field parameters are fitted from calibration data with PX4/RTK truth; "
                "runtime apply uses only raw stereo RDF and calibration-fixed parameters."
            )
        if "raw_rdf_ray_disparity_correction_coefficients_high_to_low" in method.parameters:
            training_contract = (
                "Measured-disparity ray-scale correction and Sim(3) parameters are fitted from calibration data with PX4/RTK truth; "
                "runtime apply uses only raw stereo RDF, measured disparity, and calibration-fixed parameters."
            )
        if "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low" in method.parameters:
            training_contract = (
                "Inverse-depth ray-scale correction and Sim(3) parameters are fitted from calibration data with PX4/RTK truth; "
                "runtime apply uses only raw stereo RDF and calibration-fixed parameters."
            )
        if "raw_rdf_ray_disparity_forward_product_px_m" in method.parameters:
            training_contract = (
                "Calibrated disparity-forward product ray-scale correction and Sim(3) parameters are fitted "
                "from calibration data with PX4/RTK truth; runtime apply uses only raw stereo RDF, measured "
                "disparity, and calibration-fixed parameters."
            )
        if "left_disparity_triangulation_model" in method.parameters:
            training_contract = (
                "Left-image center/disparity triangulation parameters and Sim(3) alignment are fitted from "
                "calibration data with PX4/RTK truth; runtime apply uses only left bbox center, measured "
                "disparity, and calibration-fixed parameters."
            )
        if "piecewise_sim3_raw_forward" in method.parameters:
            training_contract = (
                "Range-dependent Sim(3) nodes are fitted from calibration data with PX4/RTK truth; "
                "runtime apply uses only raw stereo RDF raw_forward_m and calibration-fixed transform nodes."
            )
            if "piecewise_sim3_blend_alpha" in method.parameters:
                training_contract = (
                    "Global Sim(3) and range-dependent Sim(3) nodes are fitted from calibration data with PX4/RTK truth; "
                    "runtime apply convex-blends global and piecewise RDF outputs using only raw stereo RDF raw_forward_m "
                    "and calibration-fixed parameters."
                )
        profile = {
            "mode": "estimated_to_truth_sim3",
            "profile_id": f"{experiment_id}-{method.name}",
            "source_experiment": experiment_id,
            "method": method.name,
            "axis_contract": (
                "internal_policy_vector=RDF(right,down,forward); "
                "legacy_stereodistance_position_topics=FRD_range(forward,right,down,range); "
                "named CSV/report fields must preserve explicit axis names"
            ),
            "training_contract": training_contract,
            "runtime_inputs": [
                "raw_right_m",
                "raw_down_m",
                "raw_forward_m",
                "raw_disparity_px",
            ],
            "forbidden_runtime_inputs": [
                "calibration_truth",
                "px4_rtk_truth",
                "true_disparity",
                "true_image_feature_location",
            ],
            "scale": method.parameters["scale"],
            "rotation_matrix_rdf": method.parameters["rotation_matrix_rdf"],
            "rotation_euler_zyx_deg": method.parameters["rotation_euler_zyx_deg"],
            "translation_rdf_m": method.parameters["translation_rdf_m"],
        }
        if "rdf_residual_disparity_coefficients_high_to_low" in method.parameters:
            profile["rdf_residual_disparity_coefficients_high_to_low"] = method.parameters[
                "rdf_residual_disparity_coefficients_high_to_low"
            ]
        if "forward_residual_raw_forward_lut" in method.parameters:
            profile["forward_residual_raw_forward_lut"] = method.parameters["forward_residual_raw_forward_lut"]
        if "rdf_residual_raw_rdf_polynomial" in method.parameters:
            profile["rdf_residual_raw_rdf_polynomial"] = method.parameters["rdf_residual_raw_rdf_polynomial"]
        if "raw_rdf_ray_disparity_correction_coefficients_high_to_low" in method.parameters:
            profile["raw_rdf_ray_disparity_correction_coefficients_high_to_low"] = method.parameters[
                "raw_rdf_ray_disparity_correction_coefficients_high_to_low"
            ]
        if "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low" in method.parameters:
            profile["raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low"] = method.parameters[
                "raw_rdf_ray_inverse_depth_correction_coefficients_high_to_low"
            ]
        if "raw_rdf_ray_disparity_forward_product_px_m" in method.parameters:
            profile["raw_rdf_ray_disparity_forward_product_px_m"] = method.parameters[
                "raw_rdf_ray_disparity_forward_product_px_m"
            ]
        if "piecewise_sim3_raw_forward" in method.parameters:
            profile["piecewise_sim3_raw_forward"] = method.parameters["piecewise_sim3_raw_forward"]
        if "piecewise_sim3_blend_alpha" in method.parameters:
            profile["piecewise_sim3_blend_alpha"] = method.parameters["piecewise_sim3_blend_alpha"]
        if "left_disparity_triangulation_model" in method.parameters:
            profile["left_disparity_triangulation_model"] = method.parameters["left_disparity_triangulation_model"]
            profile["runtime_inputs"] = [
                "left_u_px",
                "left_v_px",
                "raw_disparity_px",
                "raw_right_m",
                "raw_down_m",
                "raw_forward_m",
            ]
        return profile
    if {
        "sim3_scale",
        "sim3_rotation_matrix_rdf",
        "sim3_rotation_euler_zyx_deg",
        "sim3_translation_rdf_m",
    }.issubset(method.parameters):
        has_forward_poly = "residual_forward_from_disparity_coefficients_high_to_low" in method.parameters
        has_forward_lut = "forward_residual_disparity_lut" in method.parameters
        has_forward_raw_lut = "forward_residual_raw_forward_lut" in method.parameters
        has_rdf_lut = "rdf_residual_disparity_lut" in method.parameters
        if not has_forward_poly and not has_forward_lut and not has_forward_raw_lut and not has_rdf_lut:
            return None
        profile = {
            "mode": "estimated_to_truth_sim3",
            "profile_id": f"{experiment_id}-{method.name}",
            "source_experiment": experiment_id,
            "method": method.name,
            "axis_contract": (
                "internal_policy_vector=RDF(right,down,forward); "
                "legacy_stereodistance_position_topics=FRD_range(forward,right,down,range); "
                "named CSV/report fields must preserve explicit axis names"
            ),
            "training_contract": (
                "Sim(3) and residual parameters are fitted from calibration data with PX4/RTK truth; "
                "runtime apply uses only raw stereo RDF, measured disparity when required, and calibration-fixed parameters."
            ),
            "runtime_inputs": [
                "raw_right_m",
                "raw_down_m",
                "raw_forward_m",
                "raw_disparity_px",
            ],
            "forbidden_runtime_inputs": [
                "calibration_truth",
                "px4_rtk_truth",
                "true_disparity",
                "true_image_feature_location",
            ],
            "scale": method.parameters["sim3_scale"],
            "rotation_matrix_rdf": method.parameters["sim3_rotation_matrix_rdf"],
            "rotation_euler_zyx_deg": method.parameters["sim3_rotation_euler_zyx_deg"],
            "translation_rdf_m": method.parameters["sim3_translation_rdf_m"],
        }
        if has_forward_poly:
            profile["forward_residual_disparity_coefficients_high_to_low"] = method.parameters[
                "residual_forward_from_disparity_coefficients_high_to_low"
            ]
        if has_forward_lut:
            profile["forward_residual_disparity_lut"] = method.parameters["forward_residual_disparity_lut"]
        if has_forward_raw_lut:
            profile["forward_residual_raw_forward_lut"] = method.parameters["forward_residual_raw_forward_lut"]
            profile["forward_residual_raw_forward_query"] = method.parameters.get(
                "local_sweep_forward_key",
                "raw_forward",
            )
        if "forward_residual_agreement_gate" in method.parameters:
            profile["forward_residual_agreement_gate"] = method.parameters["forward_residual_agreement_gate"]
        if has_rdf_lut:
            profile["rdf_residual_disparity_lut"] = method.parameters["rdf_residual_disparity_lut"]
        return profile
    return None


def experiment_id_from_output_dir(output_dir: Path) -> str:
    if output_dir.name.startswith("EXP-"):
        return output_dir.name
    if output_dir.parent.name.startswith("EXP-"):
        return output_dir.parent.name
    return "rtk_stereo_calibration"


def format_float(value: object) -> str:
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if math.isinf(number):
        return "inf"
    if math.isnan(number):
        return "nan"
    return f"{number:.6f}"


def choose_recommendation(rows: Sequence[dict[str, object]]) -> tuple[str, str]:
    raw_row = next(row for row in rows if row["method"] == "raw")
    raw_val = float(raw_row["validation_range_rmse_m"])
    candidates = []
    for row in rows:
        if row["method"] == "raw":
            continue
        validation_rmse = float(row["validation_range_rmse_m"])
        target_abs = float(row["validation_target_abs_mean_error_m"])
        target_window_count = int(row.get("validation_target_window_count", 0))
        generalization_gap = float(row["generalization_gap_range_rmse_m"])
        complexity = int(row["model_complexity"])
        axis_regression_penalty = sum(
            max(0.0, float(row[f"validation_{axis}_rmse_m"]) - float(raw_row[f"validation_{axis}_rmse_m"]))
            for axis in ("right", "down", "forward")
        )
        improves = validation_rmse < raw_val
        target_ok = target_window_count > 0 and math.isfinite(target_abs) and target_abs <= 10.0
        axes_ok = axis_regression_penalty <= 1.0e-9
        if not improves or not target_ok or not axes_ok:
            continue
        score = (
            validation_rmse
            + 0.35 * target_abs
            + 0.20 * max(0.0, generalization_gap)
            + 0.15 * axis_regression_penalty
            + 0.08 * complexity
        )
        candidates.append((score, target_ok, improves, row))
    if not candidates:
        reason = (
            "No correction candidate passed validation gating. A candidate must reduce validation range RMSE "
            "relative to raw and have at least one validation sample in the 1852m target window with absolute "
            "mean range error within +/-10m, with no RDF axis RMSE regression relative to raw. "
            "Keeping raw output and exporting no final runtime profile."
        )
        return "raw", reason
    candidates.sort(key=lambda item: item[0])
    best = candidates[0][3]
    reason = (
        "Selected by validation range RMSE, 1852m target-window absolute mean error, "
        "RDF axis non-regression, train/validation gap, and model complexity. "
        f"Raw validation range RMSE={raw_val:.3f} m; selected "
        f"{best['method']} validation range RMSE={float(best['validation_range_rmse_m']):.3f} m, "
        f"1852m abs mean error={float(best['validation_target_abs_mean_error_m']):.3f} m."
    )
    return str(best["method"]), reason


def policy_recommendation_rows(
    rows: Sequence[dict[str, object]],
    *,
    final_recommendation: str,
) -> list[dict[str, object]]:
    raw_row = next(row for row in rows if row["method"] == "raw")
    raw_validation_rmse = float(raw_row["validation_range_rmse_m"])
    candidates = [
        row
        for row in rows
        if row["method"] != "raw"
        and float(row["validation_range_rmse_m"]) < raw_validation_rmse
        and int(row.get("validation_target_window_count", 0)) > 0
        and math.isfinite(float(row["validation_target_abs_mean_error_m"]))
        and float(row["validation_target_abs_mean_error_m"]) <= 10.0
    ]
    if not candidates:
        return []

    def balanced_score(row: dict[str, object]) -> tuple[float, float, int]:
        axis_regression_penalty = sum(
            max(0.0, float(row[f"validation_{axis}_rmse_m"]) - float(raw_row[f"validation_{axis}_rmse_m"]))
            for axis in ("right", "down", "forward")
        )
        return (
            float(row["validation_range_rmse_m"])
            + 0.35 * float(row["validation_target_abs_mean_error_m"])
            + 0.20 * max(0.0, float(row["generalization_gap_range_rmse_m"]))
            + 0.15 * axis_regression_penalty
            + 0.08 * int(row["model_complexity"]),
            float(row["validation_range_rmse_m"]),
            int(row["model_complexity"]),
        )

    def axis_balanced_score(row: dict[str, object]) -> tuple[float, float, int]:
        axis_regression_penalty = sum(
            max(0.0, float(row[f"validation_{axis}_rmse_m"]) - float(raw_row[f"validation_{axis}_rmse_m"]))
            for axis in ("right", "down", "forward")
        )
        axis_values = [
            float(row["validation_right_rmse_m"]),
            float(row["validation_down_rmse_m"]),
            float(row["validation_forward_rmse_m"]),
        ]
        axis_mean = sum(axis_values) / len(axis_values)
        return (
            axis_mean
            + 0.30 * axis_regression_penalty
            + 0.25 * float(row["validation_target_abs_mean_error_m"])
            + 0.04 * int(row["model_complexity"]),
            float(row["validation_position_rmse_m"]),
            int(row["model_complexity"]),
        )

    stable_margin_candidates = [
        row
        for row in candidates
        if raw_validation_rmse - float(row["validation_range_rmse_m"]) >= 0.5
        and math.isfinite(float(row["validation_target_max_abs_error_m"]))
    ]
    if not stable_margin_candidates:
        stable_margin_candidates = candidates

    policies = [
        (
            "balanced",
            "Minimize validation range RMSE while accounting for 1852m error, train/validation gap, RDF-axis regression, and complexity.",
            min(candidates, key=balanced_score),
        ),
        (
            "axis_balanced",
            "Minimize RDF axis-wise validation RMSE while penalizing axis regressions, 1852m error, and complexity.",
            min(candidates, key=axis_balanced_score),
        ),
        (
            "target_specialist",
            "Prioritize the 1852m target-window absolute mean range error, then validation RMSE and complexity.",
            min(
                candidates,
                key=lambda row: (
                    float(row["validation_target_abs_mean_error_m"]),
                    float(row["validation_range_rmse_m"]),
                    int(row["model_complexity"]),
                ),
            ),
        ),
        (
            "target_max_stable",
            (
                "Prioritize 1852m target-window maximum absolute range error among methods that keep "
                "at least 0.5 m validation range RMSE improvement; then validation RMSE and complexity."
            ),
            min(
                stable_margin_candidates,
                key=lambda row: (
                    float(row["validation_target_max_abs_error_m"]),
                    float(row["validation_range_rmse_m"]),
                    int(row["model_complexity"]),
                ),
            ),
        ),
        (
            "lowest_complexity_pass",
            "Choose the least complex correction that improves raw validation RMSE and stays within the 1852m tolerance.",
            min(
                candidates,
                key=lambda row: (
                    int(row["model_complexity"]),
                    float(row["validation_range_rmse_m"]),
                    float(row["validation_target_abs_mean_error_m"]),
                ),
            ),
        ),
        (
            "lowest_validation_rmse",
            "Choose the lowest validation range RMSE among methods passing the 1852m tolerance.",
            min(
                candidates,
                key=lambda row: (
                    float(row["validation_range_rmse_m"]),
                    float(row["validation_target_abs_mean_error_m"]),
                    int(row["model_complexity"]),
                ),
            ),
        ),
    ]
    policy_rows = []

    def make_policy_row(policy_name: str, purpose: str, row: dict[str, object]) -> dict[str, object]:
        return {
            "policy": policy_name,
            "purpose": purpose,
            "method": row["method"],
            "is_final_recommendation": row["method"] == final_recommendation,
            "validation_range_rmse_m": row["validation_range_rmse_m"],
            "validation_position_rmse_m": row["validation_position_rmse_m"],
            "validation_right_rmse_m": row["validation_right_rmse_m"],
            "validation_down_rmse_m": row["validation_down_rmse_m"],
            "validation_forward_rmse_m": row["validation_forward_rmse_m"],
            "validation_target_abs_mean_error_m": row["validation_target_abs_mean_error_m"],
            "validation_target_rmse_m": row["validation_target_rmse_m"],
            "validation_target_max_abs_error_m": row["validation_target_max_abs_error_m"],
            "generalization_gap_range_rmse_m": row["generalization_gap_range_rmse_m"],
            "model_complexity": row["model_complexity"],
            "raw_validation_range_rmse_m": raw_validation_rmse,
            "target_tolerance_m": 10.0,
        }

    for policy_name, purpose, row in policies:
        policy_rows.append(make_policy_row(policy_name, purpose, row))
    if not any(row["is_final_recommendation"] for row in policy_rows):
        final_row = next((row for row in candidates if row["method"] == final_recommendation), None)
        if final_row is not None:
            policy_rows.append(
                make_policy_row(
                    "final_recommendation",
                    "Method selected by the final validation gate, including RDF axis non-regression.",
                    final_row,
                )
            )
    return policy_rows


def pareto_frontier_rows(rows: Sequence[dict[str, object]], *, recommendation: str) -> list[dict[str, object]]:
    candidate_rows = [row for row in rows if row["method"] != "raw"]
    frontier_methods: set[str] = set()
    for row in candidate_rows:
        dominated = False
        metrics = (
            float(row["validation_range_rmse_m"]),
            float(row["validation_target_abs_mean_error_m"]),
            int(row["model_complexity"]),
        )
        for other in candidate_rows:
            if other is row:
                continue
            other_metrics = (
                float(other["validation_range_rmse_m"]),
                float(other["validation_target_abs_mean_error_m"]),
                int(other["model_complexity"]),
            )
            no_worse = all(other_value <= value for other_value, value in zip(other_metrics, metrics))
            strictly_better = any(other_value < value for other_value, value in zip(other_metrics, metrics))
            if no_worse and strictly_better:
                dominated = True
                break
        if not dominated:
            frontier_methods.add(str(row["method"]))
    ordered_rows = sorted(
        candidate_rows,
        key=lambda row: (
            0 if str(row["method"]) in frontier_methods else 1,
            float(row["validation_range_rmse_m"]),
            float(row["validation_target_abs_mean_error_m"]),
            int(row["model_complexity"]),
        ),
    )
    pareto_rows = []
    for rank, row in enumerate(ordered_rows, start=1):
        pareto_rows.append(
            {
                "rank": rank,
                "method": row["method"],
                "is_pareto_frontier": str(row["method"]) in frontier_methods,
                "is_recommendation": row["method"] == recommendation,
                "validation_range_rmse_m": row["validation_range_rmse_m"],
                "validation_position_rmse_m": row["validation_position_rmse_m"],
                "validation_target_abs_mean_error_m": row["validation_target_abs_mean_error_m"],
                "validation_target_rmse_m": row["validation_target_rmse_m"],
                "generalization_gap_range_rmse_m": row["generalization_gap_range_rmse_m"],
                "model_complexity": row["model_complexity"],
            }
        )
    return pareto_rows


def target_window_sample_count(
    samples: Sequence[CalibrationSample],
    *,
    target_range_m: float,
    target_window_m: float,
) -> int:
    return int(
        sum(
            abs(sample.truth_range_m - float(target_range_m)) <= float(target_window_m)
            for sample in samples
        )
    )


def write_summary(
    path: Path,
    *,
    source_name: str,
    output_dir: Path,
    rows: Sequence[dict[str, object]],
    recommendation: str,
    reason: str,
    config: dict[str, object],
    pareto_rows: Sequence[dict[str, object]],
    policy_rows: Sequence[dict[str, object]],
) -> None:
    headers = [
        "method",
        "registration_model",
        "residual_model",
        "runtime_profile_family",
        "train_range_rmse_m",
        "validation_range_rmse_m",
        "validation_right_rmse_m",
        "validation_down_rmse_m",
        "validation_forward_rmse_m",
        "validation_target_abs_mean_error_m",
        "validation_target_rmse_m",
        "generalization_gap_range_rmse_m",
        "model_complexity",
    ]
    widths = {header: max(len(header), 12) for header in headers}
    formatted_rows: list[dict[str, str]] = []
    for row in rows:
        formatted = {header: format_float(row.get(header, "")) for header in headers}
        formatted_rows.append(formatted)
        for header, value in formatted.items():
            widths[header] = max(widths[header], len(value))
    lines = [
        "RTK stereo calibration method comparison",
        "",
        f"Data source: {source_name}",
        f"Output directory: {output_dir}",
        f"NED contract: {NED_AXIS_CONTRACT}",
        f"RDF contract: {RDF_AXIS_CONTRACT}",
        f"NED to stereo RDF truth transform: {SYNTHETIC_NED_TO_RDF_CONTRACT}",
        "Split target-window coverage: train={train}, validation={validation}, total={total}".format(
            train=config.get("train_target_window_count", "unknown"),
            validation=config.get("validation_target_window_count", "unknown"),
            total=config.get("total_target_window_count", "unknown"),
        ),
        "",
        "Method performance table:",
        " | ".join(header.ljust(widths[header]) for header in headers),
        "-+-".join("-" * widths[header] for header in headers),
    ]
    for row in formatted_rows:
        lines.append(" | ".join(row[header].ljust(widths[header]) for header in headers))
    frontier = [row for row in pareto_rows if row["is_pareto_frontier"]]
    pareto_headers = [
        "method",
        "validation_range_rmse_m",
        "validation_target_abs_mean_error_m",
        "model_complexity",
        "is_recommendation",
    ]
    pareto_widths = {header: max(len(header), 12) for header in pareto_headers}
    formatted_pareto: list[dict[str, str]] = []
    for row in frontier:
        formatted = {header: format_float(row.get(header, "")) for header in pareto_headers}
        formatted_pareto.append(formatted)
        for header, value in formatted.items():
            pareto_widths[header] = max(pareto_widths[header], len(value))
    lines.extend(
        [
            "",
            "Pareto frontier by validation range RMSE, 1852m abs mean error, and model complexity:",
            " | ".join(header.ljust(pareto_widths[header]) for header in pareto_headers),
            "-+-".join("-" * pareto_widths[header] for header in pareto_headers),
        ]
    )
    for row in formatted_pareto:
        lines.append(" | ".join(row[header].ljust(pareto_widths[header]) for header in pareto_headers))
    policy_headers = [
        "policy",
        "method",
        "validation_range_rmse_m",
        "validation_target_abs_mean_error_m",
        "model_complexity",
        "is_final_recommendation",
    ]
    policy_widths = {header: max(len(header), 12) for header in policy_headers}
    formatted_policies: list[dict[str, str]] = []
    for row in policy_rows:
        formatted = {header: format_float(row.get(header, "")) for header in policy_headers}
        formatted_policies.append(formatted)
        for header, value in formatted.items():
            policy_widths[header] = max(policy_widths[header], len(value))
    lines.extend(
        [
            "",
            "Policy recommendations:",
            " | ".join(header.ljust(policy_widths[header]) for header in policy_headers),
            "-+-".join("-" * policy_widths[header] for header in policy_headers),
        ]
    )
    for row in formatted_policies:
        lines.append(" | ".join(row[header].ljust(policy_widths[header]) for header in policy_headers))
    lines.extend(
        [
            "",
            f"Final recommendation: {recommendation}",
            reason,
            "",
            "Pass/fail criteria:",
            "- Validation performance is prioritized over train performance.",
            "- 1852m target-window absolute mean range error should be within +/-10m.",
            "- Correction should reduce validation range RMSE relative to raw.",
            "- Lower model complexity and smaller train/validation gap are preferred when metrics are close.",
            "",
            "Configuration:",
            json.dumps(config, ensure_ascii=False, indent=2),
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def plot_pareto_frontier(path: Path, pareto_rows: Sequence[dict[str, object]]) -> None:
    rows = list(pareto_rows)
    plt.figure(figsize=(11, 7))
    non_frontier = [row for row in rows if not bool(row["is_pareto_frontier"])]
    frontier = [row for row in rows if bool(row["is_pareto_frontier"])]
    for subset, label, marker, alpha in (
        (non_frontier, "dominated", "o", 0.35),
        (frontier, "Pareto frontier", "D", 0.9),
    ):
        if not subset:
            continue
        x_values = [float(row["validation_range_rmse_m"]) for row in subset]
        y_values = [float(row["validation_target_abs_mean_error_m"]) for row in subset]
        sizes = [35.0 + 10.0 * int(row["model_complexity"]) for row in subset]
        plt.scatter(x_values, y_values, s=sizes, marker=marker, alpha=alpha, label=label)
    for row in frontier:
        plt.annotate(
            str(row["method"]),
            (float(row["validation_range_rmse_m"]), float(row["validation_target_abs_mean_error_m"])),
            textcoords="offset points",
            xytext=(6, 4),
            fontsize=8,
        )
    for row in rows:
        if bool(row["is_recommendation"]):
            plt.scatter(
                [float(row["validation_range_rmse_m"])],
                [float(row["validation_target_abs_mean_error_m"])],
                s=[170.0],
                facecolors="none",
                edgecolors="tab:red",
                linewidths=2.0,
                label="final recommendation",
            )
            break
    plt.axhline(10.0, color="tab:red", linestyle="--", linewidth=1)
    plt.xlabel("validation range RMSE (m)")
    plt.ylabel("1852m target-window abs mean error (m)")
    plt.title("Calibration method Pareto tradeoff")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def plot_distance_errors(
    path: Path,
    validation_samples: Sequence[CalibrationSample],
    predictions_by_method: dict[str, np.ndarray],
    recommendation: str,
    target_range_m: float,
) -> None:
    truth = stack(validation_samples, "truth_rdf_m")
    truth_range = np.linalg.norm(truth, axis=1)
    order = np.argsort(truth_range)
    plt.figure(figsize=(11, 6))
    for method in dict.fromkeys(("raw", recommendation)):
        predictions = predictions_by_method[method]
        pred_range = np.linalg.norm(predictions, axis=1)
        plt.plot(truth_range[order], (pred_range - truth_range)[order], label=method)
    plt.axhline(10.0, color="tab:red", linestyle="--", linewidth=1)
    plt.axhline(-10.0, color="tab:red", linestyle="--", linewidth=1)
    plt.axvline(target_range_m, color="tab:gray", linestyle=":", linewidth=1)
    plt.xlabel("truth range (m)")
    plt.ylabel("range error (m)")
    plt.title("Validation range error before/after correction")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def plot_disparity_residuals(
    path: Path,
    samples: Sequence[CalibrationSample],
    methods: Sequence[FittedMethod],
) -> None:
    d_meas = measured_disparity(samples)
    d_true = true_disparity(samples)
    order = np.argsort(d_meas)
    plt.figure(figsize=(11, 6))
    plt.scatter(d_meas, d_meas - d_true, s=12, alpha=0.45, label="raw d_meas - d_true")
    for method in methods:
        coeff = method.parameters.get("d_corr_coefficients_high_to_low")
        if coeff is None:
            continue
        coeff_array = np.array(coeff, dtype=float)
        residual = np.polyval(coeff_array, d_meas) - d_true
        plt.plot(d_meas[order], residual[order], linewidth=2, label=method.name)
    plt.axhline(0.0, color="black", linewidth=1)
    plt.xlabel("measured disparity (px)")
    plt.ylabel("residual disparity (px)")
    plt.title("Residual disparity by distance/disparity")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def plot_axis_rmse(path: Path, comparison_rows: Sequence[dict[str, object]]) -> None:
    methods = [str(row["method"]) for row in comparison_rows]
    axes = ["right", "down", "forward"]
    values = np.array(
        [
            [float(row[f"validation_{axis}_rmse_m"]) for axis in axes]
            for row in comparison_rows
        ]
    )
    x_values = np.arange(len(methods))
    width = 0.25
    plt.figure(figsize=(12, 6))
    for axis_index, axis in enumerate(axes):
        plt.bar(x_values + (axis_index - 1) * width, values[:, axis_index], width=width, label=axis)
    plt.xticks(x_values, methods, rotation=30, ha="right")
    plt.ylabel("validation RMSE (m)")
    plt.title("RDF axis-wise validation RMSE")
    plt.grid(True, axis="y", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def target_window_rows(
    validation_samples: Sequence[CalibrationSample],
    predictions_by_method: dict[str, np.ndarray],
    *,
    target_range_m: float,
    target_window_m: float,
) -> list[dict[str, object]]:
    truth = stack(validation_samples, "truth_rdf_m")
    truth_range = np.linalg.norm(truth, axis=1)
    mask = np.abs(truth_range - target_range_m) <= target_window_m
    rows = []
    for method, predictions in predictions_by_method.items():
        pred_range = np.linalg.norm(predictions, axis=1)
        errors = pred_range[mask] - truth_range[mask]
        rows.append(
            {
                "method": method,
                "target_range_m": target_range_m,
                "window_half_width_m": target_window_m,
                "sample_count": int(errors.size),
                "mean_error_m": float(np.mean(errors)) if errors.size else float("nan"),
                "abs_mean_error_m": float(abs(np.mean(errors))) if errors.size else float("inf"),
                "rmse_m": rmse(errors) if errors.size else float("inf"),
                "max_abs_error_m": float(np.max(np.abs(errors))) if errors.size else float("inf"),
                "pass_abs_mean_within_10m": "yes" if errors.size and abs(float(np.mean(errors))) <= 10.0 else "no",
            }
        )
    return rows


def build_source(args: argparse.Namespace, camera: CameraModel, injection: ErrorInjection) -> CalibrationDataSource:
    if args.mode == "synthetic":
        return SyntheticCalibrationDataSource(
            camera=camera,
            injection=injection,
            sample_count=args.sample_count,
            seed=args.seed,
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
            target_oversample_count=args.synthetic_target_oversample_count,
        )
    if args.mode == "csv":
        if args.input_csv is None:
            raise ValueError("--input-csv is required for --mode csv")
        return CsvCalibrationDataSource(args.input_csv, camera=camera)
    return Ros2TopicCalibrationDataSource()


def default_output_dir() -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parents[1] / "results" / f"rtk_stereo_calibration_{stamp}"


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir or default_output_dir()
    output_dir.mkdir(parents=True, exist_ok=True)

    camera = CameraModel(
        baseline_m=args.baseline_m,
        baseline_scale_error=args.baseline_scale_error,
        focal_scale_error=args.focal_scale_error,
    )
    injection = ErrorInjection(
        yaw_deg=args.yaw_error_deg,
        pitch_deg=args.pitch_error_deg,
        roll_deg=args.roll_error_deg,
        disparity_bias_px=args.disparity_bias_px,
        disparity_noise_px=args.disparity_noise_px,
        bbox_noise_px=args.bbox_noise_px,
        disparity_outlier_rate=args.disparity_outlier_rate,
        disparity_outlier_px=args.disparity_outlier_px,
        bbox_outlier_rate=args.bbox_outlier_rate,
        bbox_outlier_px=args.bbox_outlier_px,
    )
    source = build_source(args, camera, injection)
    loaded_samples = source.load_samples()
    train_samples, validation_samples = split_samples(
        loaded_samples,
        train_ratio=args.train_ratio,
        seed=args.seed + 17,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    all_samples = sorted([*train_samples, *validation_samples], key=lambda sample: sample.sample_id)
    train_target_window_count = target_window_sample_count(
        train_samples,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    validation_target_window_count = target_window_sample_count(
        validation_samples,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    total_target_window_count = target_window_sample_count(
        all_samples,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    write_csv(output_dir / "calibration_samples.csv", rows_for_samples(all_samples))

    local_sweep_configs = load_local_sweep_configs(args.local_sweep_config_dir)
    local_sweep_status_rows: list[dict[str, object]] = []
    methods = fit_methods(
        train_samples,
        camera=camera,
        target_range_m=args.target_range_m,
        local_sweep_configs=local_sweep_configs,
        local_sweep_status_rows=local_sweep_status_rows,
    )
    train_predictions = {method.name: method.predict(train_samples) for method in methods}
    validation_predictions = {method.name: method.predict(validation_samples) for method in methods}

    comparison_rows: list[dict[str, object]] = []
    for method in methods:
        train_metrics = evaluate_predictions(
            train_samples,
            train_predictions[method.name],
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
        )
        validation_metrics = evaluate_predictions(
            validation_samples,
            validation_predictions[method.name],
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
        )
        row: dict[str, object] = {
            "method": method.name,
            **method_contract_fields(method.name),
            "rdf_axis_contract": RDF_AXIS_CONTRACT,
            "model_complexity": method.model_complexity,
            "train_range_rmse_m": train_metrics["range_rmse_m"],
            "validation_range_rmse_m": validation_metrics["range_rmse_m"],
            "generalization_gap_range_rmse_m": (
                float(validation_metrics["range_rmse_m"]) - float(train_metrics["range_rmse_m"])
            ),
            "parameters_json": json.dumps(method.parameters, ensure_ascii=False, sort_keys=True),
        }
        for prefix, metrics in (("train", train_metrics), ("validation", validation_metrics)):
            for key, value in metrics.items():
                row[f"{prefix}_{key}"] = value
        comparison_rows.append(row)

    recommendation, reason = choose_recommendation(comparison_rows)
    pareto_rows = pareto_frontier_rows(comparison_rows, recommendation=recommendation)
    policy_rows = policy_recommendation_rows(comparison_rows, final_recommendation=recommendation)
    write_csv(output_dir / "method_comparison.csv", comparison_rows)
    write_csv(output_dir / "pareto_frontier.csv", pareto_rows)
    write_csv(output_dir / "policy_recommendations.csv", policy_rows)
    write_csv(output_dir / "validation_predictions.csv", prediction_rows(validation_samples, validation_predictions))
    write_csv(
        output_dir / "target_1852m_stats.csv",
        target_window_rows(
            validation_samples,
            validation_predictions,
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
        ),
    )
    write_json(
        output_dir / "method_parameters.json",
        {method.name: method.parameters for method in methods},
    )
    write_json(
        output_dir / "pareto_frontier.json",
        {
            "selection_dimensions": [
                "validation_range_rmse_m",
                "validation_target_abs_mean_error_m",
                "model_complexity",
            ],
            "rows": pareto_rows,
        },
    )
    write_json(
        output_dir / "policy_recommendations.json",
        {
            "policies": policy_rows,
            "runtime_profile_contract": "estimated_to_truth_sim3 profiles use internal RDF(right,down,forward) and runtime raw stereo RDF + measured disparity only.",
        },
    )
    config = {
        "mode": args.mode,
        "source": source.name,
        "sample_count": len(all_samples),
        "train_count": len(train_samples),
        "validation_count": len(validation_samples),
        "train_target_window_count": train_target_window_count,
        "validation_target_window_count": validation_target_window_count,
        "total_target_window_count": total_target_window_count,
        "split_strategy": "target_window_stratified_when_target_range_and_window_are_available",
        "train_ratio": args.train_ratio,
        "target_range_m": args.target_range_m,
        "target_window_m": args.target_window_m,
        "synthetic_target_oversample_count": args.synthetic_target_oversample_count,
        "local_sweep_config_dir": str(args.local_sweep_config_dir) if args.local_sweep_config_dir else "",
        "local_sweep_config_count": len(local_sweep_configs),
        "local_sweep_added_count": sum(1 for row in local_sweep_status_rows if row.get("status") == "ADDED"),
        "local_sweep_skipped_count": sum(1 for row in local_sweep_status_rows if row.get("status") == "SKIP"),
        "camera": camera.__dict__,
        "error_injection": {
            "yaw_deg": injection.yaw_deg,
            "pitch_deg": injection.pitch_deg,
            "roll_deg": injection.roll_deg,
            "disparity_bias_px": injection.disparity_bias_px,
            "disparity_noise_px": injection.disparity_noise_px,
            "bbox_noise_px": injection.bbox_noise_px,
            "disparity_outlier_rate": injection.disparity_outlier_rate,
            "disparity_outlier_px": injection.disparity_outlier_px,
            "bbox_outlier_rate": injection.bbox_outlier_rate,
            "bbox_outlier_px": injection.bbox_outlier_px,
            "down_bias_px": injection.down_bias_px,
        },
        "contracts": {
            "ned": NED_AXIS_CONTRACT,
            "rdf": RDF_AXIS_CONTRACT,
            "truth_rdf_transform_contract": "truth_rdf must be stereo_center_RDF from per-sample T_source_to_stereo_rdf metadata or an already stereo-frame truth producer",
            "direct_source_axis_alias_allowed": False,
            "ned_to_stereo_rdf": SYNTHETIC_NED_TO_RDF_CONTRACT,
            "direct_ned_to_rdf_axis_alias_allowed": False,
            "synthetic_stereo_origin_ned_m": SYNTHETIC_STEREO_ORIGIN_NED_M.tolist(),
            "synthetic_ned_to_stereo_rdf_euler_zyx_deg": [17.0, -4.0, 2.5],
            "stereodistance_public_topic": STEREODISTANCE_PUBLIC_TOPIC_CONTRACT,
        },
        "recommendation": recommendation,
        "recommendation_reason": reason,
    }
    write_json(output_dir / "run_config.json", config)
    if local_sweep_status_rows:
        write_csv(output_dir / "local_sweep_config_status.csv", local_sweep_status_rows)
        write_json(
            output_dir / "local_sweep_config_status.json",
            {
                "mode": "rtk_stereo_local_sweep_config_status",
                "config_dir": str(args.local_sweep_config_dir) if args.local_sweep_config_dir else "",
                "config_count": len(local_sweep_configs),
                "added_count": sum(1 for row in local_sweep_status_rows if row.get("status") == "ADDED"),
                "skipped_count": sum(1 for row in local_sweep_status_rows if row.get("status") == "SKIP"),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": STEREODISTANCE_PUBLIC_TOPIC_CONTRACT,
                "runtime_truth_used": False,
                "rows": local_sweep_status_rows,
            },
        )
    raw_comparison_row = next(row for row in comparison_rows if row["method"] == "raw")
    runtime_profile_manifest: list[dict[str, object]] = []
    for method in methods:
        candidate_profile = runtime_profile_for_method(
            method,
            experiment_id=experiment_id_from_output_dir(output_dir),
        )
        if candidate_profile is None:
            continue
        method_row = next(row for row in comparison_rows if row["method"] == method.name)
        candidate_profile["validation_summary"] = {
            "validation_range_rmse_raw_m": float(raw_comparison_row["validation_range_rmse_m"]),
            "validation_range_rmse_corrected_m": float(method_row["validation_range_rmse_m"]),
            "target_1852m_abs_mean_error_m": float(method_row["validation_target_abs_mean_error_m"]),
        }
        profile_path = output_dir / "runtime_profiles" / f"{method.name}.json"
        write_json(profile_path, candidate_profile)
        runtime_profile_manifest.append(
            {
                "method": method.name,
                "profile_path": str(profile_path.relative_to(output_dir)),
                "validation_range_rmse_m": float(method_row["validation_range_rmse_m"]),
                "target_1852m_abs_mean_error_m": float(method_row["validation_target_abs_mean_error_m"]),
                "model_complexity": int(method.model_complexity),
                "selected": method.name == recommendation,
            }
        )
    if runtime_profile_manifest:
        write_json(output_dir / "runtime_profiles" / "manifest.json", runtime_profile_manifest)
    policy_profile_manifest: list[dict[str, object]] = []
    for policy_row in policy_rows:
        method_name = str(policy_row["method"])
        method = next((candidate for candidate in methods if candidate.name == method_name), None)
        if method is None:
            continue
        candidate_profile = runtime_profile_for_method(
            method,
            experiment_id=experiment_id_from_output_dir(output_dir),
        )
        if candidate_profile is None:
            continue
        method_row = next(row for row in comparison_rows if row["method"] == method_name)
        candidate_profile["policy"] = policy_row["policy"]
        candidate_profile["policy_purpose"] = policy_row["purpose"]
        candidate_profile["validation_summary"] = {
            "validation_range_rmse_raw_m": float(raw_comparison_row["validation_range_rmse_m"]),
            "validation_range_rmse_corrected_m": float(method_row["validation_range_rmse_m"]),
            "target_1852m_abs_mean_error_m": float(method_row["validation_target_abs_mean_error_m"]),
        }
        profile_path = output_dir / "policy_profiles" / f"{policy_row['policy']}.json"
        write_json(profile_path, candidate_profile)
        policy_profile_manifest.append(
            {
                "policy": policy_row["policy"],
                "method": method_name,
                "profile_path": str(profile_path.relative_to(output_dir)),
                "validation_range_rmse_m": float(method_row["validation_range_rmse_m"]),
                "target_1852m_abs_mean_error_m": float(method_row["validation_target_abs_mean_error_m"]),
                "model_complexity": int(method.model_complexity),
            }
        )
    if policy_profile_manifest:
        write_json(output_dir / "policy_profiles" / "manifest.json", policy_profile_manifest)
    recommended_method = next(method for method in methods if method.name == recommendation)
    runtime_profile = runtime_profile_for_method(
        recommended_method,
        experiment_id=experiment_id_from_output_dir(output_dir),
    )
    if runtime_profile is not None:
        recommendation_row = next(row for row in comparison_rows if row["method"] == recommendation)
        runtime_profile["validation_summary"] = {
            "validation_range_rmse_raw_m": float(raw_comparison_row["validation_range_rmse_m"]),
            "validation_range_rmse_corrected_m": float(recommendation_row["validation_range_rmse_m"]),
            "target_1852m_abs_mean_error_m": float(recommendation_row["validation_target_abs_mean_error_m"]),
        }
        write_json(output_dir / "sim3_runtime_profile.json", runtime_profile)
    write_summary(
        output_dir / "summary.txt",
        source_name=source.name,
        output_dir=output_dir,
        rows=comparison_rows,
        recommendation=recommendation,
        reason=reason,
        config=config,
        pareto_rows=pareto_rows,
        policy_rows=policy_rows,
    )

    plot_distance_errors(
        output_dir / "range_error_before_after.png",
        validation_samples,
        validation_predictions,
        recommendation,
        args.target_range_m,
    )
    plot_disparity_residuals(
        output_dir / "residual_disparity_by_distance.png",
        validation_samples,
        methods,
    )
    plot_axis_rmse(output_dir / "rdf_axis_rmse.png", comparison_rows)
    plot_pareto_frontier(output_dir / "pareto_frontier.png", pareto_rows)

    print(f"wrote results to {output_dir}")
    print(f"final recommendation: {recommendation}")
    print(reason)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
