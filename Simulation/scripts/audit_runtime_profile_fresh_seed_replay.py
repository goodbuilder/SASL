#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

import rtk_stereo_calibration_pipeline as pipe  # noqa: E402
from stereo_ground_sim.runtime_guard_policy import (  # noqa: E402
    EstimatedToTruthAffineProfile,
    EstimatedToTruthDisparityProfile,
    EstimatedToTruthSim3Profile,
)


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
TRUTH_RDF_TRANSFORM_CONTRACT = (
    "explicit T_source_to_stereo_rdf required; direct axis alias "
    "rdf_right=east/rdf_down=down/rdf_forward=north is forbidden"
)
NED_TO_RDF_CONTRACT = TRUTH_RDF_TRANSFORM_CONTRACT
DEFAULT_SCENARIOS = (
    "nominal",
    "yaw_pitch_stress",
    "disparity_noise_stress",
    "intrinsic_scale_stress",
    "combined_sensor_stress",
    "bbox_outlier_stress",
)
SCENARIO_PRESETS: dict[str, dict[str, float]] = {
    "nominal": {},
    "yaw_pitch_stress": {
        "yaw_deg": 0.70,
        "pitch_deg": -0.35,
        "roll_deg": 0.12,
    },
    "disparity_noise_stress": {
        "disparity_bias_px": 0.55,
        "disparity_noise_px": 0.12,
        "bbox_noise_px": 0.25,
    },
    "intrinsic_scale_stress": {
        "baseline_scale_error": 0.012,
        "focal_scale_error": -0.008,
    },
    "combined_sensor_stress": {
        "yaw_deg": 0.85,
        "pitch_deg": -0.45,
        "roll_deg": 0.16,
        "baseline_scale_error": 0.012,
        "focal_scale_error": -0.008,
        "disparity_bias_px": 0.60,
        "disparity_noise_px": 0.14,
        "bbox_noise_px": 0.30,
    },
    "bbox_outlier_stress": {
        "bbox_outlier_rate": 0.08,
        "bbox_outlier_px": 2.5,
        "disparity_outlier_rate": 0.06,
        "disparity_outlier_px": 0.45,
    },
}


RuntimeProfile = EstimatedToTruthSim3Profile | EstimatedToTruthDisparityProfile | EstimatedToTruthAffineProfile


def load_profile(path: Path) -> RuntimeProfile:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("mode") == EstimatedToTruthSim3Profile.mode:
        return EstimatedToTruthSim3Profile.from_json_dict(payload)
    if payload.get("mode") == EstimatedToTruthDisparityProfile.mode:
        return EstimatedToTruthDisparityProfile.from_json_dict(payload)
    if payload.get("mode") == EstimatedToTruthAffineProfile.mode:
        return EstimatedToTruthAffineProfile.from_json_dict(payload)
    raise ValueError(f"unsupported runtime profile mode: {payload.get('mode')!r}")


def sample_payload(
    sample: pipe.CalibrationSample,
    *,
    raw_rdf_m: np.ndarray | None = None,
    measured_disparity_px: float | None = None,
) -> dict[str, float]:
    raw = sample.raw_rdf_m if raw_rdf_m is None else raw_rdf_m
    disparity = sample.measured_disparity_px if measured_disparity_px is None else measured_disparity_px
    return {
        "right_m": float(raw[0]),
        "down_m": float(raw[1]),
        "forward_m": float(raw[2]),
        "range_m": float(np.linalg.norm(raw)),
        "disparity_px": float(disparity),
        "left_u_px": float(sample.left_u_px),
        "left_v_px": float(sample.left_v_px),
    }


def parse_scenarios(value: str) -> list[str]:
    names = [item.strip() for item in value.split(",") if item.strip()]
    if not names:
        raise argparse.ArgumentTypeError("scenario list must not be empty")
    unknown = [name for name in names if name not in SCENARIO_PRESETS]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown scenarios: {', '.join(unknown)}")
    return names


def camera_and_injection_for_scenario(scenario: str) -> tuple[pipe.CameraModel, pipe.ErrorInjection]:
    preset = SCENARIO_PRESETS.get(scenario)
    if preset is None:
        raise ValueError(f"unknown scenario: {scenario}")
    camera_fields = {"baseline_scale_error", "focal_scale_error"}
    camera_kwargs = {key: value for key, value in preset.items() if key in camera_fields}
    injection_kwargs = {key: value for key, value in preset.items() if key not in camera_fields}
    return pipe.CameraModel(**camera_kwargs), pipe.ErrorInjection(**injection_kwargs)


def build_seed_samples(
    *,
    seed: int,
    scenario: str,
    sample_count: int,
    train_ratio: float,
    target_range_m: float,
    target_window_m: float,
    target_oversample_count: int,
) -> tuple[list[pipe.CalibrationSample], list[pipe.CalibrationSample]]:
    camera, injection = camera_and_injection_for_scenario(scenario)
    source = pipe.SyntheticCalibrationDataSource(
        camera=camera,
        injection=injection,
        sample_count=sample_count,
        seed=seed,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
        target_oversample_count=target_oversample_count,
    )
    samples = source.load_samples()
    return pipe.split_samples(
        samples,
        train_ratio=train_ratio,
        seed=seed + 17,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )


def evaluate_seed(
    profile: RuntimeProfile,
    *,
    seed: int,
    scenario: str,
    sample_count: int,
    train_ratio: float,
    target_range_m: float,
    target_window_m: float,
    target_oversample_count: int,
    temporal_filter_window: int,
    temporal_filter_threshold_m: float,
    raw_range_prefilter_window: int,
    raw_range_prefilter_threshold_m: float,
    raw_range_prefilter_target_window_m: float,
    disparity_prefilter_window: int,
    disparity_prefilter_threshold_px: float,
    disparity_prefilter_raw_range_window_m: float,
    alpha_beta_range_gate_m: float,
    alpha_beta_range_alpha: float,
    alpha_beta_range_beta: float,
    alpha_beta_range_min_history: int,
    alpha_beta_range_target_window_m: float,
    prefilter_history_scope: str,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    train, validation = build_seed_samples(
        seed=seed,
        scenario=scenario,
        sample_count=sample_count,
        train_ratio=train_ratio,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
        target_oversample_count=target_oversample_count,
    )
    corrected_points_before_filter: list[np.ndarray] = []
    raw_points: list[np.ndarray] = []
    prediction_bases: list[dict[str, object]] = []
    invalid_count = 0
    fallback_count = 0
    sequence_samples = validation
    if prefilter_history_scope == "all_samples":
        sequence_samples = sorted([*train, *validation], key=lambda item: item.sample_id)
    sequence_raw = np.vstack([sample.raw_rdf_m.astype(float) for sample in sequence_samples]) if sequence_samples else np.empty((0, 3))
    sequence_disparity = np.asarray([sample.measured_disparity_px for sample in sequence_samples], dtype=float)
    sequence_times = np.asarray([sample.t_s for sample in sequence_samples], dtype=float)
    range_filtered_raw, raw_range_prefilter_count = apply_trailing_raw_range_prefilter(
        sequence_raw,
        window=raw_range_prefilter_window,
        threshold_m=raw_range_prefilter_threshold_m,
        target_range_m=target_range_m,
        target_window_m=raw_range_prefilter_target_window_m,
    )
    filtered_raw, filtered_disparity, disparity_prefilter_count = apply_trailing_disparity_prefilter(
        range_filtered_raw,
        sequence_disparity,
        window=disparity_prefilter_window,
        threshold_px=disparity_prefilter_threshold_px,
        target_range_m=target_range_m,
        raw_range_window_m=disparity_prefilter_raw_range_window_m,
    )
    tracked_raw, alpha_beta_range_prefilter_count = apply_alpha_beta_range_prefilter(
        filtered_raw,
        sequence_times,
        gate_m=alpha_beta_range_gate_m,
        alpha=alpha_beta_range_alpha,
        beta=alpha_beta_range_beta,
        min_history=alpha_beta_range_min_history,
        target_range_m=target_range_m,
        target_window_m=alpha_beta_range_target_window_m,
    )
    alpha_beta_changed_by_sample_id = {
        int(sample.sample_id): abs(
            float(np.linalg.norm(tracked_raw[index])) - float(np.linalg.norm(filtered_raw[index]))
        )
        > 1.0e-12
        for index, sample in enumerate(sequence_samples)
    }
    raw_range_changed_by_sample_id = {
        int(sample.sample_id): abs(
            float(np.linalg.norm(range_filtered_raw[index])) - float(np.linalg.norm(sequence_raw[index]))
        )
        > 1.0e-12
        for index, sample in enumerate(sequence_samples)
    }
    filtered_by_sample_id = {
        int(sample.sample_id): (tracked_raw[index].astype(float), float(filtered_disparity[index]))
        for index, sample in enumerate(sequence_samples)
    }
    validation_prefilter_count = 0
    validation_disparity_prefilter_count = 0
    validation_alpha_beta_range_prefilter_count = 0
    for sample in validation:
        raw, measured_disparity = filtered_by_sample_id[int(sample.sample_id)]
        raw_changed = abs(float(np.linalg.norm(raw)) - float(np.linalg.norm(sample.raw_rdf_m))) > 1.0e-12
        raw_range_changed = bool(raw_range_changed_by_sample_id[int(sample.sample_id)])
        disparity_changed = abs(measured_disparity - sample.measured_disparity_px) > 1.0e-12
        alpha_beta_changed = bool(alpha_beta_changed_by_sample_id[int(sample.sample_id)])
        validation_prefilter_count += int(raw_range_changed)
        validation_disparity_prefilter_count += int(disparity_changed)
        validation_alpha_beta_range_prefilter_count += int(alpha_beta_changed)
        result = profile.apply(
            sample_payload(sample, raw_rdf_m=raw, measured_disparity_px=measured_disparity)
        )
        if result.corrected is None:
            invalid_count += 1
            continue
        corrected = np.array(
            [
                float(result.corrected["right_m"]),
                float(result.corrected["down_m"]),
                float(result.corrected["forward_m"]),
            ],
            dtype=float,
        )
        fallback_level = int(result.status.get("fallback_level", 0))
        if fallback_level != 0:
            fallback_count += 1
        raw_points.append(raw)
        corrected_points_before_filter.append(corrected)
        truth = sample.truth_rdf_m.astype(float)
        prediction_bases.append(
            {
                "seed": seed,
                "scenario": scenario,
                "sample_id": sample.sample_id,
                "split": sample.split,
                "truth_right_m": truth[0],
                "truth_down_m": truth[1],
                "truth_forward_m": truth[2],
                "raw_right_m": raw[0],
                "raw_down_m": raw[1],
                "raw_forward_m": raw[2],
                "corrected_right_m": corrected[0],
                "corrected_down_m": corrected[1],
                "corrected_forward_m": corrected[2],
                "corrected_range_before_temporal_filter_m": float(np.linalg.norm(corrected)),
                "measured_disparity_px": measured_disparity,
                "measured_disparity_before_prefilter_px": sample.measured_disparity_px,
                "disparity_prefilter_applied": bool(disparity_changed),
                "raw_range_before_prefilter_m": float(np.linalg.norm(sample.raw_rdf_m)),
                "raw_range_prefilter_applied": bool(raw_changed),
                "alpha_beta_range_prefilter_applied": bool(alpha_beta_changed),
                "profile_fallback_level": fallback_level,
                "profile_fallback_reason": result.status.get("fallback_reason", ""),
                "rdf_axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    temporal_filter_count = 0
    if corrected_points_before_filter:
        corrected_arr = np.vstack(corrected_points_before_filter)
        corrected_arr, temporal_filter_count = apply_trailing_temporal_range_filter(
            corrected_arr,
            window=temporal_filter_window,
            threshold_m=temporal_filter_threshold_m,
        )
        raw_arr = np.vstack(raw_points)
        evaluated_samples = [sample for sample in validation if sample.split == "validation"][: corrected_arr.shape[0]]
    else:
        corrected_arr = np.empty((0, 3), dtype=float)
        raw_arr = np.empty((0, 3), dtype=float)
        evaluated_samples = []
    prediction_rows: list[dict[str, object]] = []
    for base, corrected in zip(prediction_bases, corrected_arr):
        updated = dict(base)
        updated["corrected_right_m"] = float(corrected[0])
        updated["corrected_down_m"] = float(corrected[1])
        updated["corrected_forward_m"] = float(corrected[2])
        updated["temporal_filter_enabled"] = bool(temporal_filter_window > 1 and temporal_filter_threshold_m > 0.0)
        prediction_rows.append(updated)
    raw_metrics = pipe.evaluate_predictions(
        evaluated_samples,
        raw_arr,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    corrected_metrics = pipe.evaluate_predictions(
        evaluated_samples,
        corrected_arr,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    row = {
        "seed": seed,
        "scenario": scenario,
        "status": "PASS"
        if corrected_metrics["range_rmse_m"] < raw_metrics["range_rmse_m"]
        and corrected_metrics["position_rmse_m"] < raw_metrics["position_rmse_m"]
        and corrected_metrics["target_window_count"] > 0
        and corrected_metrics["target_max_abs_error_m"] <= 10.0
        and invalid_count == 0
        else "FAIL",
        "validation_sample_count": len(validation),
        "evaluated_sample_count": int(corrected_arr.shape[0]),
        "invalid_count": invalid_count,
        "fallback_count": fallback_count,
        "temporal_filter_count": temporal_filter_count,
        "raw_range_prefilter_count": raw_range_prefilter_count,
        "raw_range_prefilter_validation_count": validation_prefilter_count,
        "disparity_prefilter_count": disparity_prefilter_count,
        "disparity_prefilter_validation_count": validation_disparity_prefilter_count,
        "alpha_beta_range_prefilter_count": alpha_beta_range_prefilter_count,
        "alpha_beta_range_prefilter_validation_count": validation_alpha_beta_range_prefilter_count,
        "range_rmse_raw_m": raw_metrics["range_rmse_m"],
        "range_rmse_corrected_m": corrected_metrics["range_rmse_m"],
        "position_rmse_raw_m": raw_metrics["position_rmse_m"],
        "position_rmse_corrected_m": corrected_metrics["position_rmse_m"],
        "right_rmse_corrected_m": corrected_metrics["right_rmse_m"],
        "down_rmse_corrected_m": corrected_metrics["down_rmse_m"],
        "forward_rmse_corrected_m": corrected_metrics["forward_rmse_m"],
        "target_count": corrected_metrics["target_window_count"],
        "target_max_abs_error_corrected_m": corrected_metrics["target_max_abs_error_m"],
        "target_rmse_corrected_m": corrected_metrics["target_rmse_m"],
        "runtime_truth_used": False,
        "prefilter_history_scope": prefilter_history_scope,
    }
    return row, prediction_rows


def apply_trailing_raw_range_prefilter(
    raw_points: np.ndarray,
    *,
    window: int,
    threshold_m: float,
    target_range_m: float = 0.0,
    target_window_m: float = 0.0,
) -> tuple[np.ndarray, int]:
    if window <= 1 or threshold_m <= 0.0 or raw_points.size == 0:
        return raw_points, 0
    source = np.asarray(raw_points, dtype=float)
    filtered = source.copy()
    ranges = np.linalg.norm(source, axis=1)
    changed = 0
    safe_window = max(3, int(window))
    for index in range(ranges.size):
        current = float(ranges[index])
        if not np.isfinite(current) or current <= 1.0e-9:
            continue
        if target_window_m > 0.0 and target_range_m > 0.0 and abs(current - float(target_range_m)) > float(target_window_m):
            continue
        start = max(0, index - safe_window)
        history = ranges[start:index]
        history = history[np.isfinite(history) & (history > 1.0e-9)]
        if history.size < 4:
            continue
        deltas = np.diff(history)
        deltas = deltas[np.isfinite(deltas)]
        if deltas.size == 0:
            continue
        expected = float(history[-1] + np.median(deltas))
        if not np.isfinite(expected) or expected <= 1.0e-9:
            continue
        if abs(current - expected) <= float(threshold_m):
            continue
        norm = float(np.linalg.norm(filtered[index]))
        if norm <= 1.0e-12:
            continue
        filtered[index] = filtered[index] * (expected / norm)
        changed += 1
    return filtered, changed


def apply_trailing_temporal_range_filter(
    points: np.ndarray,
    *,
    window: int,
    threshold_m: float,
) -> tuple[np.ndarray, int]:
    if window <= 1 or threshold_m <= 0.0 or points.size == 0:
        return points, 0
    filtered = np.asarray(points, dtype=float).copy()
    source_ranges = np.linalg.norm(np.asarray(points, dtype=float), axis=1)
    changed = 0
    safe_window = max(2, int(window))
    for index in range(source_ranges.size):
        start = max(0, index - safe_window)
        history = source_ranges[start:index]
        if history.size < 3:
            continue
        x_history = np.arange(start, index, dtype=float)
        try:
            slope, intercept = np.polyfit(x_history, history, deg=1)
            expected = float(slope * float(index) + intercept)
        except (TypeError, ValueError, np.linalg.LinAlgError):
            expected = float(np.median(history))
        current = float(source_ranges[index])
        if not np.isfinite(expected) or not np.isfinite(current):
            continue
        if expected <= 1.0e-9:
            continue
        if abs(current - expected) <= float(threshold_m):
            continue
        norm = float(np.linalg.norm(filtered[index]))
        if norm <= 1.0e-12:
            continue
        filtered[index] = filtered[index] * (expected / norm)
        changed += 1
    return filtered, changed


def apply_trailing_disparity_prefilter(
    raw_points: np.ndarray,
    disparities: np.ndarray,
    *,
    window: int,
    threshold_px: float,
    target_range_m: float = 0.0,
    raw_range_window_m: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, int]:
    if window <= 1 or threshold_px <= 0.0 or raw_points.size == 0:
        return raw_points, disparities, 0
    filtered_raw = np.asarray(raw_points, dtype=float).copy()
    source_disparities = np.asarray(disparities, dtype=float).reshape(-1)
    filtered_disparities = source_disparities.copy()
    changed = 0
    safe_window = max(2, int(window))
    raw_ranges = np.linalg.norm(filtered_raw, axis=1)
    for index in range(source_disparities.size):
        if raw_range_window_m > 0.0 and target_range_m > 0.0:
            raw_range = float(raw_ranges[index])
            if not np.isfinite(raw_range) or abs(raw_range - float(target_range_m)) > float(raw_range_window_m):
                continue
        start = max(0, index - safe_window)
        history = source_disparities[start:index]
        history = history[np.isfinite(history) & (history > 1.0e-9)]
        if history.size < 3:
            continue
        x_history = np.arange(index - history.size, index, dtype=float)
        try:
            slope, intercept = np.polyfit(x_history, history, deg=1)
            expected = float(slope * float(index) + intercept)
        except (TypeError, ValueError, np.linalg.LinAlgError):
            expected = float(np.median(history))
        current = float(source_disparities[index])
        if not np.isfinite(expected) or expected <= 1.0e-9 or not np.isfinite(current) or current <= 1.0e-9:
            continue
        if abs(current - expected) <= float(threshold_px):
            continue
        # Stereo depth is proportional to 1 / disparity. Preserve the raw RDF bearing
        # but replace the range implied by the outlying measured disparity.
        scale = current / expected
        filtered_raw[index] = filtered_raw[index] * scale
        filtered_disparities[index] = expected
        changed += 1
    return filtered_raw, filtered_disparities, changed


def apply_alpha_beta_range_prefilter(
    raw_points: np.ndarray,
    times_s: np.ndarray,
    *,
    gate_m: float,
    alpha: float,
    beta: float,
    min_history: int,
    target_range_m: float = 0.0,
    target_window_m: float = 0.0,
) -> tuple[np.ndarray, int]:
    if gate_m <= 0.0 or raw_points.size == 0:
        return raw_points, 0
    source = np.asarray(raw_points, dtype=float)
    times = np.asarray(times_s, dtype=float).reshape(-1)
    if times.size != source.shape[0]:
        times = np.arange(source.shape[0], dtype=float)
    filtered = source.copy()
    ranges = np.linalg.norm(source, axis=1)
    safe_alpha = float(np.clip(alpha, 0.0, 1.0))
    safe_beta = float(np.clip(beta, 0.0, 1.0))
    safe_min_history = max(2, int(min_history))
    changed = 0
    state_range: float | None = None
    state_rate = 0.0
    previous_time: float | None = None
    accepted_count = 0
    for index, measurement in enumerate(ranges):
        current_time = float(times[index]) if np.isfinite(times[index]) else float(index)
        if not np.isfinite(measurement) or measurement <= 1.0e-9:
            continue
        if state_range is None:
            state_range = float(measurement)
            previous_time = current_time
            accepted_count = 1
            continue
        dt = current_time - float(previous_time if previous_time is not None else current_time - 1.0)
        if not np.isfinite(dt) or dt <= 1.0e-9:
            dt = 1.0
        predicted = float(state_range + state_rate * dt)
        if not np.isfinite(predicted) or predicted <= 1.0e-9:
            predicted = float(state_range)
        scoped = True
        if target_window_m > 0.0 and target_range_m > 0.0:
            scoped = abs(float(measurement) - float(target_range_m)) <= float(target_window_m)
        residual = float(measurement - predicted)
        reject = scoped and accepted_count >= safe_min_history and abs(residual) > float(gate_m)
        used_measurement = predicted if reject else float(measurement)
        if reject:
            norm = float(np.linalg.norm(filtered[index]))
            if norm > 1.0e-12:
                filtered[index] = filtered[index] * (predicted / norm)
                changed += 1
        innovation = float(used_measurement - predicted)
        state_range = float(predicted + safe_alpha * innovation)
        state_rate = float(state_rate + (safe_beta * innovation / dt))
        previous_time = current_time
        if not reject:
            accepted_count += 1
    return filtered, changed


def aggregate(rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    def values(name: str) -> np.ndarray:
        return np.asarray([float(row[name]) for row in rows], dtype=float)

    pass_count = sum(1 for row in rows if row.get("status") == "PASS")
    return {
        "status": "STRICT_PASS" if pass_count == len(rows) and rows else "FAIL",
        "pass_count": pass_count,
        "trial_count": len(rows),
        "range_rmse_raw_mean_m": float(np.mean(values("range_rmse_raw_m"))) if rows else float("nan"),
        "range_rmse_corrected_mean_m": float(np.mean(values("range_rmse_corrected_m"))) if rows else float("nan"),
        "range_rmse_corrected_max_m": float(np.max(values("range_rmse_corrected_m"))) if rows else float("nan"),
        "position_rmse_raw_mean_m": float(np.mean(values("position_rmse_raw_m"))) if rows else float("nan"),
        "position_rmse_corrected_mean_m": float(np.mean(values("position_rmse_corrected_m"))) if rows else float("nan"),
        "position_rmse_corrected_max_m": float(np.max(values("position_rmse_corrected_m"))) if rows else float("nan"),
        "target_max_abs_error_corrected_max_m": float(np.max(values("target_max_abs_error_corrected_m"))) if rows else float("nan"),
        "target_rmse_corrected_mean_m": float(np.mean(values("target_rmse_corrected_m"))) if rows else float("nan"),
        "invalid_count_total": int(sum(int(row["invalid_count"]) for row in rows)),
        "fallback_count_total": int(sum(int(row["fallback_count"]) for row in rows)),
        "temporal_filter_count_total": int(sum(int(row.get("temporal_filter_count", 0)) for row in rows)),
        "raw_range_prefilter_count_total": int(sum(int(row.get("raw_range_prefilter_count", 0)) for row in rows)),
        "raw_range_prefilter_validation_count_total": int(
            sum(int(row.get("raw_range_prefilter_validation_count", 0)) for row in rows)
        ),
        "disparity_prefilter_count_total": int(sum(int(row.get("disparity_prefilter_count", 0)) for row in rows)),
        "disparity_prefilter_validation_count_total": int(
            sum(int(row.get("disparity_prefilter_validation_count", 0)) for row in rows)
        ),
        "alpha_beta_range_prefilter_count_total": int(
            sum(int(row.get("alpha_beta_range_prefilter_count", 0)) for row in rows)
        ),
        "alpha_beta_range_prefilter_validation_count_total": int(
            sum(int(row.get("alpha_beta_range_prefilter_validation_count", 0)) for row in rows)
        ),
        "runtime_truth_used": False,
    }


def aggregate_by_scenario(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    scenario_names = sorted({str(row["scenario"]) for row in rows})
    scenario_rows = []
    for scenario in scenario_names:
        subset = [row for row in rows if str(row["scenario"]) == scenario]
        scenario_rows.append({"scenario": scenario, **aggregate(subset)})
    return scenario_rows


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(pipe.json_safe(payload), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit an exported runtime profile on fresh synthetic seeds.")
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument("--seed-start", type=int, default=20296359)
    parser.add_argument("--trial-count", type=int, default=200)
    parser.add_argument(
        "--scenarios",
        type=parse_scenarios,
        default=list(DEFAULT_SCENARIOS),
        help=f"Comma-separated scenario list. Known: {', '.join(DEFAULT_SCENARIOS)}",
    )
    parser.add_argument("--sample-count", type=int, default=1680)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-oversample-count", type=int, default=72)
    parser.add_argument(
        "--temporal-filter-window",
        type=int,
        default=1,
        help="Trailing corrected-range median window. 1 disables runtime-safe temporal filtering.",
    )
    parser.add_argument(
        "--temporal-filter-threshold-m",
        type=float,
        default=0.0,
        help="Replace corrected range by trailing median when deviation exceeds this threshold. 0 disables.",
    )
    parser.add_argument(
        "--disparity-prefilter-window",
        type=int,
        default=1,
        help="Trailing measured-disparity linear prediction window. 1 disables runtime-safe disparity prefiltering.",
    )
    parser.add_argument(
        "--raw-range-prefilter-window",
        type=int,
        default=1,
        help="Trailing raw RDF range delta-median window. 1 disables runtime-safe raw range prefiltering.",
    )
    parser.add_argument(
        "--raw-range-prefilter-threshold-m",
        type=float,
        default=0.0,
        help="Replace raw RDF range by trailing delta-median prediction when deviation exceeds this threshold. 0 disables.",
    )
    parser.add_argument(
        "--raw-range-prefilter-target-window-m",
        type=float,
        default=0.0,
        help="If positive, apply raw range prefilter only when raw RDF range is within this window of target range.",
    )
    parser.add_argument(
        "--disparity-prefilter-threshold-px",
        type=float,
        default=0.0,
        help="Replace measured disparity by trailing prediction when deviation exceeds this threshold. 0 disables.",
    )
    parser.add_argument(
        "--disparity-prefilter-raw-range-window-m",
        type=float,
        default=0.0,
        help="If positive, apply disparity prefilter only when raw RDF range is within this window of target range.",
    )
    parser.add_argument(
        "--prefilter-history-scope",
        choices=["validation", "all_samples"],
        default="validation",
        help="Use validation-only history or all chronological raw samples for runtime-safe prefilters.",
    )
    parser.add_argument(
        "--alpha-beta-range-gate-m",
        type=float,
        default=0.0,
        help="Runtime-safe raw range alpha-beta tracker gate. 0 disables.",
    )
    parser.add_argument(
        "--alpha-beta-range-alpha",
        type=float,
        default=0.35,
        help="Alpha gain for the raw range alpha-beta tracker.",
    )
    parser.add_argument(
        "--alpha-beta-range-beta",
        type=float,
        default=0.08,
        help="Beta gain for the raw range alpha-beta tracker.",
    )
    parser.add_argument(
        "--alpha-beta-range-min-history",
        type=int,
        default=5,
        help="Accepted raw measurements required before alpha-beta gating is allowed.",
    )
    parser.add_argument(
        "--alpha-beta-range-target-window-m",
        type=float,
        default=0.0,
        help="If positive, apply alpha-beta range gating only when raw RDF range is within this window of target range.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    profile = load_profile(args.profile_json)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    seed_rows: list[dict[str, object]] = []
    prediction_rows: list[dict[str, object]] = []
    for scenario_index, scenario in enumerate(args.scenarios):
        for offset in range(int(args.trial_count)):
            seed = int(args.seed_start) + scenario_index * int(args.trial_count) + offset
            row, predictions = evaluate_seed(
                profile,
                seed=seed,
                scenario=scenario,
                sample_count=int(args.sample_count),
                train_ratio=float(args.train_ratio),
                target_range_m=float(args.target_range_m),
                target_window_m=float(args.target_window_m),
                target_oversample_count=int(args.target_oversample_count),
                temporal_filter_window=int(args.temporal_filter_window),
                temporal_filter_threshold_m=float(args.temporal_filter_threshold_m),
                raw_range_prefilter_window=int(args.raw_range_prefilter_window),
                raw_range_prefilter_threshold_m=float(args.raw_range_prefilter_threshold_m),
                raw_range_prefilter_target_window_m=float(args.raw_range_prefilter_target_window_m),
                disparity_prefilter_window=int(args.disparity_prefilter_window),
                disparity_prefilter_threshold_px=float(args.disparity_prefilter_threshold_px),
                disparity_prefilter_raw_range_window_m=float(args.disparity_prefilter_raw_range_window_m),
                alpha_beta_range_gate_m=float(args.alpha_beta_range_gate_m),
                alpha_beta_range_alpha=float(args.alpha_beta_range_alpha),
                alpha_beta_range_beta=float(args.alpha_beta_range_beta),
                alpha_beta_range_min_history=int(args.alpha_beta_range_min_history),
                alpha_beta_range_target_window_m=float(args.alpha_beta_range_target_window_m),
                prefilter_history_scope=str(args.prefilter_history_scope),
            )
            seed_rows.append(row)
            if offset < 2:
                prediction_rows.extend(predictions)
    scenario_rows = aggregate_by_scenario(seed_rows)
    summary = {
        "experiment_id": args.experiment_id,
        "profile_json": str(args.profile_json),
        "profile_id": profile.profile_id,
        "seed_start": args.seed_start,
        "seed_end": args.seed_start + len(args.scenarios) * args.trial_count - 1,
        "scenarios": args.scenarios,
        "coordinate_contracts_explicit": True,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": TRUTH_RDF_TRANSFORM_CONTRACT,
        "ned_to_rdf_contract": NED_TO_RDF_CONTRACT,
        "temporal_filter_window": int(args.temporal_filter_window),
        "temporal_filter_threshold_m": float(args.temporal_filter_threshold_m),
        "raw_range_prefilter_window": int(args.raw_range_prefilter_window),
        "raw_range_prefilter_threshold_m": float(args.raw_range_prefilter_threshold_m),
        "raw_range_prefilter_target_window_m": float(args.raw_range_prefilter_target_window_m),
        "disparity_prefilter_window": int(args.disparity_prefilter_window),
        "disparity_prefilter_threshold_px": float(args.disparity_prefilter_threshold_px),
        "disparity_prefilter_raw_range_window_m": float(args.disparity_prefilter_raw_range_window_m),
        "alpha_beta_range_gate_m": float(args.alpha_beta_range_gate_m),
        "alpha_beta_range_alpha": float(args.alpha_beta_range_alpha),
        "alpha_beta_range_beta": float(args.alpha_beta_range_beta),
        "alpha_beta_range_min_history": int(args.alpha_beta_range_min_history),
        "alpha_beta_range_target_window_m": float(args.alpha_beta_range_target_window_m),
        "prefilter_history_scope": str(args.prefilter_history_scope),
        "temporal_filter_contract": (
            "Optional trailing linear-prediction filter uses corrected RDF range history only; "
            "PX4/NED truth is offline evaluation only."
        ),
        "disparity_prefilter_contract": (
            "Optional trailing linear-prediction prefilter uses measured disparity history only; "
            "raw RDF bearing is preserved and range is rescaled by measured/predicted disparity; "
            "optional raw-range window uses raw RDF range only."
        ),
        "raw_range_prefilter_contract": (
            "Optional trailing delta-median prefilter uses raw RDF range history only; "
            "raw RDF bearing is preserved and range is rescaled to the predicted raw range."
        ),
        "alpha_beta_range_prefilter_contract": (
            "Optional alpha-beta raw range tracker uses chronological raw RDF range and t_s only; "
            "raw RDF bearing is preserved and range is replaced by the constant-velocity prediction when gated."
        ),
        "prefilter_history_scope_contract": (
            "all_samples history uses chronological raw measurements only; train/validation truth labels are not used."
        ),
        **aggregate(seed_rows),
    }
    write_csv(args.output_dir / "fresh_seed_replay_summary.csv", seed_rows)
    write_csv(args.output_dir / "fresh_seed_replay_scenario_summary.csv", scenario_rows)
    write_csv(args.output_dir / "fresh_seed_replay_predictions_sample.csv", prediction_rows)
    write_json(args.output_dir / "fresh_seed_replay_metrics.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: fixed exported StereoDistance runtime profile replay on independent synthetic seeds.",
                "",
                "Coordinate contract: profile input/output is stereo-center RDF right/down/forward; public StereoDistance topics remain FRD+range forward/right/down/range. Synthetic PX4/local NED truth is converted to RDF only for offline evaluation.",
                "",
                f"Scenarios: {', '.join(args.scenarios)}",
                "",
                f"Result: {summary['status']}. Pass {summary['pass_count']}/{summary['trial_count']}, range RMSE mean raw -> corrected {summary['range_rmse_raw_mean_m']:.9f} -> {summary['range_rmse_corrected_mean_m']:.9f} m, target max abs corrected max {summary['target_max_abs_error_corrected_max_m']:.9f} m.",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(pipe.json_safe(summary), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
