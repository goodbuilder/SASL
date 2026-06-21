#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
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

import audit_runtime_profile_fresh_seed_replay as replay_audit  # noqa: E402
import rtk_stereo_calibration_pipeline as pipe  # noqa: E402
import run_runtime_correlated_repeated_observation_sweep as correlated  # noqa: E402
import run_runtime_image_prefilter_sweep as image_prefilter  # noqa: E402
import run_runtime_multiburst_observation_fusion_sweep as multiburst  # noqa: E402
import run_runtime_repeated_observation_fusion_sweep as repeated  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "calibration-time residual fit uses train truth only; runtime applies a polynomial residual model "
    "from fused measured_disparity_px, inverse_depth, corrected stereo-center RDF raw_forward_m, "
    "or measured_disparity_px*raw_forward_m; PX4/RTK truth is audit-only"
)
RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
TRUTH_RDF_TRANSFORM_CONTRACT = (
    "explicit T_source_to_stereo_rdf required; direct axis alias "
    "rdf_right=east/rdf_down=down/rdf_forward=north is forbidden"
)
NED_TO_RDF_CONTRACT = TRUTH_RDF_TRANSFORM_CONTRACT


@dataclass(frozen=True)
class MultiburstProfileSeries:
    seed: int
    scenario: str
    samples: list[pipe.CalibrationSample]
    train_ids: set[int]
    validation_ids: set[int]
    raw_validation_points: np.ndarray
    observation_points: np.ndarray
    fused_disparity_px: np.ndarray


@dataclass(frozen=True)
class ResidualModel:
    feature_space: str
    correction_space: str
    degree: int
    ridge_lambda: float
    correction_clip: float
    feature_mean: float
    feature_scale: float
    coefficients_low_to_high: tuple[float, ...]


def parse_int_list(value: str) -> list[int]:
    parsed = [int(item.strip()) for item in value.split(",") if item.strip()]
    if not parsed:
        raise argparse.ArgumentTypeError("integer list must not be empty")
    return parsed


def feature_values(
    disparity_px: np.ndarray,
    *,
    feature_space: str,
    raw_forward_m: np.ndarray | None = None,
) -> np.ndarray:
    disparity = np.asarray(disparity_px, dtype=float)
    if feature_space == "disparity":
        return disparity
    if feature_space == "inverse_depth":
        return disparity
    if feature_space == "log_disparity":
        return np.log(np.maximum(disparity, 1.0e-9))
    if feature_space in {"raw_forward", "log_raw_forward"}:
        if raw_forward_m is None:
            raise ValueError(f"feature_space {feature_space} requires raw_forward_m")
        raw_forward = np.asarray(raw_forward_m, dtype=float)
        if feature_space == "raw_forward":
            return raw_forward
        return np.log(np.maximum(raw_forward, 1.0e-9))
    if feature_space in {"disparity_forward_product", "log_disparity_forward_product"}:
        if raw_forward_m is None:
            raise ValueError(f"feature_space {feature_space} requires raw_forward_m")
        product = disparity * np.asarray(raw_forward_m, dtype=float)
        if feature_space == "disparity_forward_product":
            return product
        return np.log(np.maximum(product, 1.0e-9))
    raise ValueError(
        "feature_space must be disparity, inverse_depth, log_disparity, raw_forward, "
        "log_raw_forward, disparity_forward_product, or log_disparity_forward_product"
    )


def normalized_vander(values: np.ndarray, *, degree: int, mean: float, scale: float) -> np.ndarray:
    z = (np.asarray(values, dtype=float) - float(mean)) / max(float(scale), 1.0e-12)
    return np.vander(z, N=int(degree) + 1, increasing=True)


def fit_ridge_polynomial(
    x: np.ndarray,
    y: np.ndarray,
    *,
    degree: int,
    ridge_lambda: float,
) -> tuple[float, float, tuple[float, ...]]:
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    mask = np.isfinite(x) & np.isfinite(y)
    if int(np.count_nonzero(mask)) <= int(degree):
        return 0.0, 1.0, tuple(0.0 for _ in range(int(degree) + 1))
    finite_x = x[mask]
    finite_y = y[mask]
    mean = float(np.mean(finite_x))
    scale = float(np.std(finite_x))
    if not math.isfinite(scale) or scale <= 1.0e-12:
        scale = 1.0
    design = normalized_vander(finite_x, degree=degree, mean=mean, scale=scale)
    penalty = np.eye(design.shape[1], dtype=float) * max(0.0, float(ridge_lambda))
    penalty[0, 0] = 0.0
    lhs = design.T @ design + penalty
    rhs = design.T @ finite_y
    try:
        coeff = np.linalg.solve(lhs, rhs)
    except np.linalg.LinAlgError:
        coeff = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
    return mean, scale, tuple(float(value) for value in coeff)


def predict_model(
    model: ResidualModel,
    disparity_px: np.ndarray,
    *,
    raw_forward_m: np.ndarray | None = None,
) -> np.ndarray:
    x = feature_values(disparity_px, feature_space=model.feature_space, raw_forward_m=raw_forward_m)
    design = normalized_vander(x, degree=model.degree, mean=model.feature_mean, scale=model.feature_scale)
    pred = design @ np.asarray(model.coefficients_low_to_high, dtype=float)
    return np.clip(pred, -float(model.correction_clip), float(model.correction_clip))


def method_name(
    *,
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
    feature_space: str,
    correction_space: str,
    degree: int,
    ridge_lambda: float,
    correction_clip: float,
) -> str:
    base = multiburst.method_name(
        burst_count=burst_count,
        observations_per_burst=observations_per_burst,
        inner_mode=inner_mode,
        outer_mode=outer_mode,
        trim_fraction=trim_fraction,
        common_fraction=common_fraction,
    )

    def label(value: float) -> str:
        return f"{value:g}".replace(".", "p").replace("-", "m")

    return (
        f"{base}_residual_{correction_space}_{feature_space}"
        f"_deg{int(degree)}_ridge{label(ridge_lambda)}_clip{label(correction_clip)}"
    )


def build_series(
    *,
    profile: replay_audit.RuntimeProfile,
    seed: int,
    scenario: str,
    sample_count: int,
    train_ratio: float,
    target_range_m: float,
    target_window_m: float,
    target_oversample_count: int,
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
) -> MultiburstProfileSeries:
    camera, injection = replay_audit.camera_and_injection_for_scenario(scenario)
    train, validation = replay_audit.build_seed_samples(
        seed=seed,
        scenario=scenario,
        sample_count=sample_count,
        train_ratio=train_ratio,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
        target_oversample_count=target_oversample_count,
    )
    samples = sorted([*train, *validation], key=lambda sample: sample.sample_id)
    train_ids = {sample.sample_id for sample in train}
    validation_ids = {sample.sample_id for sample in validation}
    rng = np.random.default_rng(
        int(seed)
        + 62000023
        + int(burst_count) * 1009
        + int(observations_per_burst) * 917
        + int(round(common_fraction * 1000.0))
    )
    corrected_points: list[np.ndarray] = []
    fused_disparities: list[float] = []
    for sample in samples:
        left_u, left_v, disparity = multiburst.synthesize_multiburst_observation(
            truth_rdf_m=sample.truth_rdf_m,
            camera=camera,
            injection=injection,
            rng=rng,
            burst_count=burst_count,
            observations_per_burst=observations_per_burst,
            common_fraction=common_fraction,
            inner_mode=inner_mode,
            outer_mode=outer_mode,
            trim_fraction=trim_fraction,
        )
        fused_disparities.append(float(disparity))
        raw_rdf = image_prefilter.reconstruct_raw_rdf(
            camera=camera,
            injection=injection,
            left_u_px=left_u,
            left_v_px=left_v,
            disparity_px=disparity,
        )
        result = profile.apply(
            image_prefilter.profile_payload(
                raw_rdf_m=raw_rdf,
                left_u_px=left_u,
                left_v_px=left_v,
                disparity_px=disparity,
            )
        )
        if result.corrected is None:
            corrected_points.append(np.array([math.nan, math.nan, math.nan], dtype=float))
        else:
            corrected_points.append(
                np.array(
                    [
                        float(result.corrected["right_m"]),
                        float(result.corrected["down_m"]),
                        float(result.corrected["forward_m"]),
                    ],
                    dtype=float,
                )
            )
    validation_sorted = [sample for sample in samples if sample.sample_id in validation_ids]
    return MultiburstProfileSeries(
        seed=seed,
        scenario=scenario,
        samples=samples,
        train_ids=train_ids,
        validation_ids=validation_ids,
        raw_validation_points=pipe.stack(validation_sorted, "raw_rdf_m"),
        observation_points=np.vstack(corrected_points),
        fused_disparity_px=np.asarray(fused_disparities, dtype=float),
    )


def fit_residual_model(
    series: MultiburstProfileSeries,
    *,
    feature_space: str,
    correction_space: str,
    degree: int,
    ridge_lambda: float,
    correction_clip: float,
) -> ResidualModel:
    train_mask = np.asarray([sample.sample_id in series.train_ids for sample in series.samples], dtype=bool)
    points = series.observation_points
    truth = np.vstack([sample.truth_rdf_m for sample in series.samples])
    if correction_space == "forward_add":
        residual = truth[:, 2] - points[:, 2]
    elif correction_space == "ray_scale":
        denom = np.sum(points * points, axis=1)
        scale = np.sum(truth * points, axis=1) / np.maximum(denom, 1.0e-12)
        residual = scale - 1.0
    else:
        raise ValueError("correction_space must be forward_add or ray_scale")
    x = feature_values(
        series.fused_disparity_px,
        feature_space=feature_space,
        raw_forward_m=series.observation_points[:, 2],
    )
    mean, scale, coeff = fit_ridge_polynomial(
        x[train_mask],
        residual[train_mask],
        degree=degree,
        ridge_lambda=ridge_lambda,
    )
    return ResidualModel(
        feature_space=feature_space,
        correction_space=correction_space,
        degree=int(degree),
        ridge_lambda=float(ridge_lambda),
        correction_clip=float(correction_clip),
        feature_mean=mean,
        feature_scale=scale,
        coefficients_low_to_high=coeff,
    )


def apply_residual_model(series: MultiburstProfileSeries, model: ResidualModel) -> np.ndarray:
    correction = predict_model(
        model,
        series.fused_disparity_px,
        raw_forward_m=series.observation_points[:, 2],
    )
    corrected = np.asarray(series.observation_points, dtype=float).copy()
    if model.correction_space == "forward_add":
        corrected[:, 2] = corrected[:, 2] + correction
    elif model.correction_space == "ray_scale":
        corrected = corrected * (1.0 + correction[:, None])
    else:
        raise ValueError("unknown correction_space")
    return corrected


def evaluate_model(
    series: MultiburstProfileSeries,
    model: ResidualModel,
    *,
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    residual_points = apply_residual_model(series, model)
    validation_samples: list[pipe.CalibrationSample] = []
    observation_validation: list[np.ndarray] = []
    residual_validation: list[np.ndarray] = []
    for sample, observation_point, residual_point in zip(series.samples, series.observation_points, residual_points):
        if sample.sample_id in series.validation_ids:
            validation_samples.append(sample)
            observation_validation.append(observation_point)
            residual_validation.append(residual_point)
    observation_arr = (
        np.vstack(observation_validation) if observation_validation else np.empty((0, 3), dtype=float)
    )
    residual_arr = np.vstack(residual_validation) if residual_validation else np.empty((0, 3), dtype=float)
    raw_metrics = pipe.evaluate_predictions(
        validation_samples,
        series.raw_validation_points,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    observation_metrics = pipe.evaluate_predictions(
        validation_samples,
        observation_arr,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    residual_metrics = pipe.evaluate_predictions(
        validation_samples,
        residual_arr,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    passed = (
        residual_metrics["range_rmse_m"] < raw_metrics["range_rmse_m"]
        and residual_metrics["position_rmse_m"] < raw_metrics["position_rmse_m"]
        and residual_metrics["target_window_count"] > 0
        and residual_metrics["target_max_abs_error_m"] <= 10.0
    )
    return {
        "seed": series.seed,
        "scenario": series.scenario,
        "method": method_name(
            burst_count=burst_count,
            observations_per_burst=observations_per_burst,
            inner_mode=inner_mode,
            outer_mode=outer_mode,
            trim_fraction=trim_fraction,
            common_fraction=common_fraction,
            feature_space=model.feature_space,
            correction_space=model.correction_space,
            degree=model.degree,
            ridge_lambda=model.ridge_lambda,
            correction_clip=model.correction_clip,
        ),
        "status": "PASS" if passed else "FAIL",
        "burst_count": int(burst_count),
        "observations_per_burst": int(observations_per_burst),
        "total_observation_count": int(burst_count) * int(observations_per_burst),
        "inner_mode": inner_mode,
        "outer_mode": outer_mode,
        "trim_fraction": float(trim_fraction),
        "common_fraction": float(common_fraction),
        "feature_space": model.feature_space,
        "correction_space": model.correction_space,
        "degree": int(model.degree),
        "ridge_lambda": float(model.ridge_lambda),
        "correction_clip": float(model.correction_clip),
        "feature_mean": float(model.feature_mean),
        "feature_scale": float(model.feature_scale),
        "validation_sample_count": len(validation_samples),
        "range_rmse_raw_m": raw_metrics["range_rmse_m"],
        "range_rmse_observation_m": observation_metrics["range_rmse_m"],
        "range_rmse_residual_m": residual_metrics["range_rmse_m"],
        "position_rmse_raw_m": raw_metrics["position_rmse_m"],
        "position_rmse_observation_m": observation_metrics["position_rmse_m"],
        "position_rmse_residual_m": residual_metrics["position_rmse_m"],
        "right_rmse_residual_m": residual_metrics["right_rmse_m"],
        "down_rmse_residual_m": residual_metrics["down_rmse_m"],
        "forward_rmse_residual_m": residual_metrics["forward_rmse_m"],
        "target_count": residual_metrics["target_window_count"],
        "target_max_abs_error_observation_m": observation_metrics["target_max_abs_error_m"],
        "target_max_abs_error_residual_m": residual_metrics["target_max_abs_error_m"],
        "target_rmse_observation_m": observation_metrics["target_rmse_m"],
        "target_rmse_residual_m": residual_metrics["target_rmse_m"],
        "runtime_truth_used": False,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def aggregate(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row["method"]), []).append(row)
    summaries: list[dict[str, object]] = []
    for method, method_rows in grouped.items():
        values = lambda name: np.array([float(row[name]) for row in method_rows], dtype=float)
        pass_count = sum(1 for row in method_rows if row.get("status") == "PASS")
        first = method_rows[0]
        summaries.append(
            {
                "method": method,
                "status": "STRICT_PASS" if pass_count == len(method_rows) else "FAIL",
                "pass_count": pass_count,
                "trial_count": len(method_rows),
                "burst_count": first["burst_count"],
                "observations_per_burst": first["observations_per_burst"],
                "total_observation_count": first["total_observation_count"],
                "inner_mode": first["inner_mode"],
                "outer_mode": first["outer_mode"],
                "trim_fraction": first["trim_fraction"],
                "common_fraction": first["common_fraction"],
                "feature_space": first["feature_space"],
                "correction_space": first["correction_space"],
                "degree": first["degree"],
                "ridge_lambda": first["ridge_lambda"],
                "correction_clip": first["correction_clip"],
                "range_rmse_raw_mean_m": float(np.mean(values("range_rmse_raw_m"))),
                "range_rmse_observation_mean_m": float(np.mean(values("range_rmse_observation_m"))),
                "range_rmse_residual_mean_m": float(np.mean(values("range_rmse_residual_m"))),
                "position_rmse_observation_mean_m": float(np.mean(values("position_rmse_observation_m"))),
                "position_rmse_residual_mean_m": float(np.mean(values("position_rmse_residual_m"))),
                "target_max_abs_error_observation_max_m": float(
                    np.max(values("target_max_abs_error_observation_m"))
                ),
                "target_max_abs_error_residual_max_m": float(np.max(values("target_max_abs_error_residual_m"))),
                "target_rmse_observation_mean_m": float(np.mean(values("target_rmse_observation_m"))),
                "target_rmse_residual_mean_m": float(np.mean(values("target_rmse_residual_m"))),
                "residual_improves_observation_target_max": float(
                    np.max(values("target_max_abs_error_residual_m"))
                )
                < float(np.max(values("target_max_abs_error_observation_m"))),
                "residual_improves_observation_range_mean": float(np.mean(values("range_rmse_residual_m")))
                < float(np.mean(values("range_rmse_observation_m"))),
                "runtime_truth_used": False,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "rdf_axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    return sorted(
        summaries,
        key=lambda row: (
            -int(row["pass_count"]),
            float(row["target_max_abs_error_residual_max_m"]),
            float(row["range_rmse_residual_mean_m"]),
            float(row["position_rmse_residual_mean_m"]),
        ),
    )


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(pipe.json_safe(payload), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit calibration-time disparity/inverse-depth residual models after multi-burst observation fusion."
    )
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scenario", default="disparity_noise_stress", choices=tuple(replay_audit.SCENARIO_PRESETS))
    parser.add_argument("--seed-start", type=int, default=21220000)
    parser.add_argument("--trial-count", type=int, default=20)
    parser.add_argument("--sample-count", type=int, default=1680)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-oversample-count", type=int, default=72)
    parser.add_argument("--burst-specs", type=multiburst.parse_burst_specs, default=multiburst.parse_burst_specs("7x3"))
    parser.add_argument("--inner-modes", type=repeated.parse_modes, default=repeated.parse_modes("trimmed_mean"))
    parser.add_argument("--outer-modes", type=repeated.parse_modes, default=repeated.parse_modes("trimmed_mean"))
    parser.add_argument("--trim-fractions", type=correlated.parse_float_list, default=correlated.parse_float_list("0.1"))
    parser.add_argument("--common-fractions", type=correlated.parse_float_list, default=correlated.parse_float_list("0.5"))
    parser.add_argument("--feature-spaces", type=lambda value: [item.strip() for item in value.split(",") if item.strip()], default=["disparity", "log_disparity"])
    parser.add_argument("--correction-spaces", type=lambda value: [item.strip() for item in value.split(",") if item.strip()], default=["forward_add", "ray_scale"])
    parser.add_argument("--degrees", type=parse_int_list, default=parse_int_list("1,2,3"))
    parser.add_argument("--ridge-lambdas", type=correlated.parse_float_list, default=correlated.parse_float_list("0,0.1,1"))
    parser.add_argument("--correction-clips", type=correlated.parse_float_list, default=correlated.parse_float_list("0.5,1,2,4"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile = replay_audit.load_profile(args.profile_json)
    rows: list[dict[str, object]] = []
    for burst_count, observations_per_burst in args.burst_specs:
        for inner_mode in args.inner_modes:
            for outer_mode in args.outer_modes:
                for trim_fraction in args.trim_fractions:
                    for common_fraction in args.common_fractions:
                        series_list = [
                            build_series(
                                profile=profile,
                                seed=int(args.seed_start) + index,
                                scenario=str(args.scenario),
                                sample_count=int(args.sample_count),
                                train_ratio=float(args.train_ratio),
                                target_range_m=float(args.target_range_m),
                                target_window_m=float(args.target_window_m),
                                target_oversample_count=int(args.target_oversample_count),
                                burst_count=int(burst_count),
                                observations_per_burst=int(observations_per_burst),
                                inner_mode=str(inner_mode),
                                outer_mode=str(outer_mode),
                                trim_fraction=float(trim_fraction),
                                common_fraction=float(common_fraction),
                            )
                            for index in range(int(args.trial_count))
                        ]
                        for feature_space in args.feature_spaces:
                            for correction_space in args.correction_spaces:
                                for degree in args.degrees:
                                    for ridge_lambda in args.ridge_lambdas:
                                        for correction_clip in args.correction_clips:
                                            for series in series_list:
                                                model = fit_residual_model(
                                                    series,
                                                    feature_space=str(feature_space),
                                                    correction_space=str(correction_space),
                                                    degree=int(degree),
                                                    ridge_lambda=float(ridge_lambda),
                                                    correction_clip=float(correction_clip),
                                                )
                                                rows.append(
                                                    evaluate_model(
                                                        series,
                                                        model,
                                                        burst_count=int(burst_count),
                                                        observations_per_burst=int(observations_per_burst),
                                                        inner_mode=str(inner_mode),
                                                        outer_mode=str(outer_mode),
                                                        trim_fraction=float(trim_fraction),
                                                        common_fraction=float(common_fraction),
                                                        target_range_m=float(args.target_range_m),
                                                        target_window_m=float(args.target_window_m),
                                                    )
                                                )
    summaries = aggregate(rows)
    write_csv(args.output_dir / "multiburst_disparity_residual_seed_metrics.csv", rows)
    write_csv(args.output_dir / "multiburst_disparity_residual_method_summary.csv", summaries)
    summary = {
        "experiment_id": args.experiment_id,
        "profile_json": str(args.profile_json),
        "scenario": args.scenario,
        "seed_start": args.seed_start,
        "seed_end": int(args.seed_start) + int(args.trial_count) - 1,
        "trial_count": args.trial_count,
        "best_method": summaries[0] if summaries else {},
        "coordinate_contracts_explicit": True,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": TRUTH_RDF_TRANSFORM_CONTRACT,
        "ned_to_rdf_contract": NED_TO_RDF_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }
    write_json(args.output_dir / "multiburst_disparity_residual_metrics.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: fit train-split residual models from fused measured disparity/inverse-depth to remaining corrected RDF error.",
                "",
                "Coordinate contract: observation/profile outputs and residual correction are stereo-center RDF right/down/forward. PX4 local NED truth is used only for calibration-time fitting and offline validation. Public StereoDistance topics remain FRD+range.",
                "",
                "Runtime contract: fitted residual application consumes fused measured_disparity_px and corrected RDF only; it does not consume PX4/RTK truth, target-window labels, true image feature, or true disparity.",
                "",
                f"Best method: `{summary['best_method'].get('method', '')}`",
                f"Best status: `{summary['best_method'].get('status', '')}`",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(pipe.json_safe(summary), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
