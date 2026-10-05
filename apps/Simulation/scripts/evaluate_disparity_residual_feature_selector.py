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

import audit_runtime_profile_fresh_seed_replay as replay_audit  # noqa: E402
import evaluate_disparity_residual_catalog_transfer as transfer  # noqa: E402
import rtk_stereo_calibration_pipeline as pipe  # noqa: E402
import run_runtime_multiburst_disparity_residual_sweep as residual  # noqa: E402
from stereo_ground_sim.runtime_deployment_selector import (  # noqa: E402
    DisparityResidualRuntimeProfile,
)
from stereo_ground_sim.disparity_residual_feature_summary import feature_summary_from_arrays  # noqa: E402


def parse_scenarios(value: str) -> list[str]:
    return transfer.parse_scenarios(value)


def parse_float_list(value: str) -> list[float]:
    parsed = [float(item.strip()) for item in value.split(",") if item.strip()]
    if not parsed:
        raise argparse.ArgumentTypeError("float list must not be empty")
    return parsed


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate a truth-free nearest-centroid residual-profile selector from runtime feature summaries."
    )
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--base-profile-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--scenarios",
        type=parse_scenarios,
        default=parse_scenarios("bbox_outlier_stress,combined_sensor_stress,disparity_noise_stress,intrinsic_scale_stress"),
    )
    parser.add_argument("--seed-start", type=int, default=21220000)
    parser.add_argument("--trial-count", type=int, default=20)
    parser.add_argument("--selector-train-count", type=int, default=10)
    parser.add_argument("--sample-count", type=int, default=1680)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-oversample-count", type=int, default=72)
    parser.add_argument("--disable-margin-thresholds", type=parse_float_list, default=parse_float_list("0,0.05,0.1,0.2"))
    return parser.parse_args(argv)


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(pipe.json_safe(payload), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def finite_stats(prefix: str, values: np.ndarray) -> dict[str, float]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {
            f"{prefix}_mean": 0.0,
            f"{prefix}_std": 0.0,
            f"{prefix}_p05": 0.0,
            f"{prefix}_p50": 0.0,
            f"{prefix}_p95": 0.0,
        }
    return {
        f"{prefix}_mean": float(np.mean(finite)),
        f"{prefix}_std": float(np.std(finite)),
        f"{prefix}_p05": float(np.percentile(finite, 5.0)),
        f"{prefix}_p50": float(np.percentile(finite, 50.0)),
        f"{prefix}_p95": float(np.percentile(finite, 95.0)),
    }


def runtime_feature_map(series: residual.MultiburstProfileSeries) -> dict[str, float]:
    left_u = np.asarray([sample.left_u_px for sample in series.samples], dtype=float)
    left_v = np.asarray([sample.left_v_px for sample in series.samples], dtype=float)
    measured_disparity = np.asarray([sample.measured_disparity_px for sample in series.samples], dtype=float)
    raw_points = np.vstack([sample.raw_rdf_m for sample in series.samples])
    observation_points = np.asarray(series.observation_points, dtype=float)
    return {
        key: value
        for key, value in feature_summary_from_arrays(
            left_u_px=left_u,
            left_v_px=left_v,
            measured_disparity_px=measured_disparity,
            raw_rdf_m=raw_points,
            fused_disparity_px=np.asarray(series.fused_disparity_px, dtype=float),
            observation_rdf_m=observation_points,
            feature_label=series.scenario,
            observation_rdf_source="multiburst_observation_points",
        ).items()
        if isinstance(value, (int, float)) and not isinstance(value, bool) and "truth" not in str(key).lower()
    }


def feature_matrix(feature_rows: Sequence[Mapping[str, float]], feature_names: Sequence[str]) -> np.ndarray:
    return np.asarray([[float(row.get(name, 0.0)) for name in feature_names] for row in feature_rows], dtype=float)


def fit_centroids(rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    feature_names = sorted(
        {
            key
            for row in rows
            for key, value in row.items()
            if key.startswith(("left_", "measured_", "log_", "fused_", "raw_", "observation_"))
            and isinstance(value, (int, float))
        }
    )
    matrix = feature_matrix(rows, feature_names)
    mean = np.mean(matrix, axis=0)
    scale = np.std(matrix, axis=0)
    scale[~np.isfinite(scale) | (scale <= 1.0e-12)] = 1.0
    centroids: dict[str, list[float]] = {}
    labels = sorted({str(row["scenario"]) for row in rows})
    for label in labels:
        label_rows = [row for row in rows if str(row["scenario"]) == label]
        label_matrix = (feature_matrix(label_rows, feature_names) - mean) / scale
        centroids[label] = [float(value) for value in np.mean(label_matrix, axis=0)]
    return {
        "feature_names": feature_names,
        "mean": [float(value) for value in mean],
        "scale": [float(value) for value in scale],
        "centroids": centroids,
    }


def select_label(feature_row: Mapping[str, float], model: Mapping[str, object]) -> dict[str, object]:
    feature_names = [str(name) for name in model["feature_names"]]
    mean = np.asarray(model["mean"], dtype=float)
    scale = np.asarray(model["scale"], dtype=float)
    vector = (feature_matrix([feature_row], feature_names)[0] - mean) / scale
    distances: list[tuple[str, float]] = []
    centroids = model["centroids"]
    if not isinstance(centroids, Mapping):
        raise ValueError("centroids must be a mapping")
    for label, centroid in centroids.items():
        distances.append((str(label), float(np.linalg.norm(vector - np.asarray(centroid, dtype=float)))))
    distances.sort(key=lambda item: item[1])
    best_label, best_distance = distances[0]
    second_distance = distances[1][1] if len(distances) > 1 else float("inf")
    margin = float(second_distance - best_distance) if math.isfinite(second_distance) else float("inf")
    return {
        "selected_label": best_label,
        "best_distance": best_distance,
        "second_distance": second_distance,
        "distance_margin": margin,
    }


def validation_triplets(series: residual.MultiburstProfileSeries, points: np.ndarray) -> tuple[list[pipe.CalibrationSample], np.ndarray]:
    validation_samples: list[pipe.CalibrationSample] = []
    validation_points: list[np.ndarray] = []
    for sample, point in zip(series.samples, points):
        if sample.sample_id in series.validation_ids:
            validation_samples.append(sample)
            validation_points.append(point)
    arr = np.vstack(validation_points) if validation_points else np.empty((0, 3), dtype=float)
    return validation_samples, arr


def evaluate_selected(
    *,
    scenario: str,
    seed: int,
    selected_label: str,
    disabled_by_margin: bool,
    series: residual.MultiburstProfileSeries,
    runtime_profiles: Mapping[str, DisparityResidualRuntimeProfile],
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    if disabled_by_margin:
        selected_points = np.asarray(series.observation_points, dtype=float)
    else:
        selected_points = transfer.apply_runtime_profile(series, runtime_profiles[selected_label])
    validation_samples, observation_validation = validation_triplets(series, np.asarray(series.observation_points, dtype=float))
    _, selected_validation = validation_triplets(series, selected_points)
    raw_metrics = pipe.evaluate_predictions(
        validation_samples,
        series.raw_validation_points,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    observation_metrics = pipe.evaluate_predictions(
        validation_samples,
        observation_validation,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    selected_metrics = pipe.evaluate_predictions(
        validation_samples,
        selected_validation,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    passed = (
        selected_metrics["range_rmse_m"] < raw_metrics["range_rmse_m"]
        and selected_metrics["position_rmse_m"] < raw_metrics["position_rmse_m"]
        and selected_metrics["target_window_count"] > 0
        and selected_metrics["target_max_abs_error_m"] <= 10.0
    )
    return {
        "scenario": scenario,
        "seed": seed,
        "selected_label": selected_label,
        "label_matched": selected_label == scenario,
        "disabled_by_margin": disabled_by_margin,
        "status": "PASS" if passed else "FAIL",
        "range_rmse_raw_m": raw_metrics["range_rmse_m"],
        "range_rmse_observation_m": observation_metrics["range_rmse_m"],
        "range_rmse_selected_m": selected_metrics["range_rmse_m"],
        "position_rmse_raw_m": raw_metrics["position_rmse_m"],
        "position_rmse_observation_m": observation_metrics["position_rmse_m"],
        "position_rmse_selected_m": selected_metrics["position_rmse_m"],
        "target_max_abs_error_observation_m": observation_metrics["target_max_abs_error_m"],
        "target_max_abs_error_selected_m": selected_metrics["target_max_abs_error_m"],
        "target_rmse_observation_m": observation_metrics["target_rmse_m"],
        "target_rmse_selected_m": selected_metrics["target_rmse_m"],
        "runtime_truth_used": False,
    }


def aggregate(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[float, list[Mapping[str, object]]] = {}
    for row in rows:
        grouped.setdefault(float(row["disable_margin_threshold"]), []).append(row)
    summaries: list[dict[str, object]] = []
    for threshold, items in grouped.items():
        values = lambda name: np.asarray([float(row[name]) for row in items], dtype=float)
        pass_count = sum(1 for row in items if row.get("status") == "PASS")
        label_count = sum(1 for row in items if row.get("label_matched") is True)
        disabled_count = sum(1 for row in items if row.get("disabled_by_margin") is True)
        summaries.append(
            {
                "disable_margin_threshold": threshold,
                "status": "STRICT_PASS" if pass_count == len(items) else "FAIL",
                "pass_count": pass_count,
                "trial_count": len(items),
                "label_match_count": label_count,
                "disabled_count": disabled_count,
                "range_rmse_selected_mean_m": float(np.mean(values("range_rmse_selected_m"))),
                "target_max_abs_error_selected_max_m": float(np.max(values("target_max_abs_error_selected_m"))),
                "target_rmse_selected_mean_m": float(np.mean(values("target_rmse_selected_m"))),
                "runtime_truth_used": False,
            }
        )
    return sorted(
        summaries,
        key=lambda row: (
            row["status"] != "STRICT_PASS",
            -int(row["pass_count"]),
            -int(row["label_match_count"]),
            int(row["disabled_count"]),
            float(row["range_rmse_selected_mean_m"]),
        ),
    )


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile = replay_audit.load_profile(args.base_profile_json)
    profile_paths = transfer.catalog_profiles(args.catalog_json)
    runtime_profiles = {
        label: DisparityResidualRuntimeProfile.from_json_file(path)
        for label, path in profile_paths.items()
    }
    profile_payloads = {label: transfer.load_json(path) for label, path in profile_paths.items()}
    default_observation = next(iter(profile_payloads.values())).get("observation_fusion", {}) if profile_payloads else {}
    if not isinstance(default_observation, Mapping):
        default_observation = {}
    series_by_key: dict[tuple[str, int], residual.MultiburstProfileSeries] = {}
    feature_rows: list[dict[str, object]] = []
    for scenario in args.scenarios:
        for seed_index in range(int(args.trial_count)):
            seed = int(args.seed_start) + seed_index
            series = residual.build_series(
                profile=profile,
                seed=seed,
                scenario=scenario,
                sample_count=int(args.sample_count),
                train_ratio=float(args.train_ratio),
                target_range_m=float(args.target_range_m),
                target_window_m=float(args.target_window_m),
                target_oversample_count=int(args.target_oversample_count),
                burst_count=int(default_observation.get("burst_count", 7)),
                observations_per_burst=int(default_observation.get("observations_per_burst", 3)),
                inner_mode=str(default_observation.get("inner_mode", "trimmed_mean")),
                outer_mode=str(default_observation.get("outer_mode", "trimmed_mean")),
                trim_fraction=float(default_observation.get("trim_fraction", 0.05)),
                common_fraction=float(default_observation.get("common_fraction", 0.5)),
            )
            series_by_key[(scenario, seed)] = series
            feature_row = {
                "scenario": scenario,
                "seed": seed,
                "selector_split": "train" if seed_index < int(args.selector_train_count) else "validation",
                **runtime_feature_map(series),
                "runtime_truth_used": False,
            }
            feature_rows.append(feature_row)
    train_rows = [row for row in feature_rows if row["selector_split"] == "train"]
    validation_rows = [row for row in feature_rows if row["selector_split"] == "validation"]
    model = fit_centroids(train_rows)
    seed_rows: list[dict[str, object]] = []
    for feature_row in validation_rows:
        selection = select_label(feature_row, model)
        scenario = str(feature_row["scenario"])
        seed = int(feature_row["seed"])
        series = series_by_key[(scenario, seed)]
        for threshold in args.disable_margin_thresholds:
            disabled = float(selection["distance_margin"]) < float(threshold)
            row = evaluate_selected(
                scenario=scenario,
                seed=seed,
                selected_label=str(selection["selected_label"]),
                disabled_by_margin=disabled,
                series=series,
                runtime_profiles=runtime_profiles,
                target_range_m=float(args.target_range_m),
                target_window_m=float(args.target_window_m),
            )
            row.update(selection)
            row["disable_margin_threshold"] = float(threshold)
            seed_rows.append(row)
    summaries = aggregate(seed_rows)
    best = summaries[0] if summaries else {}
    feature_csv = args.output_dir / "disparity_residual_feature_selector_features.csv"
    seed_csv = args.output_dir / "disparity_residual_feature_selector_seed_metrics.csv"
    summary_csv = args.output_dir / "disparity_residual_feature_selector_summary.csv"
    model_json = args.output_dir / "disparity_residual_feature_selector_model.json"
    write_csv(feature_csv, feature_rows)
    write_csv(seed_csv, seed_rows)
    write_csv(summary_csv, summaries)
    write_json(model_json, model)
    payload = {
        "mode": "disparity_residual_feature_selector_evaluation",
        "status": "PASS" if best.get("status") == "STRICT_PASS" else "FAIL",
        "catalog_json": str(args.catalog_json),
        "base_profile_json": str(args.base_profile_json),
        "best_selector": best,
        "selector_train_count": int(args.selector_train_count),
        "selector_validation_count": len(validation_rows),
        "output_feature_csv": str(feature_csv),
        "output_seed_csv": str(seed_csv),
        "output_summary_csv": str(summary_csv),
        "output_model_json": str(model_json),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "coordinate_contract": (
            "selector features use image-space left_u/left_v, measured/fused disparity, and raw/observation "
            "stereo-center RDF summaries only; truth is offline evaluation only; selected residual profiles "
            "consume measured disparity and corrected stereo-center RDF right/down/forward"
        ),
    }
    write_json(args.output_dir / "disparity_residual_feature_selector_summary.json", payload)
    (args.output_dir / "disparity_residual_feature_selector_summary.txt").write_text(
        "\n".join(
            [
                "Disparity residual truth-free feature selector evaluation",
                f"status: {payload['status']}",
                f"best_disable_margin_threshold: {best.get('disable_margin_threshold', '')}",
                f"pass_count: {best.get('pass_count', '')}/{best.get('trial_count', '')}",
                f"label_match_count: {best.get('label_match_count', '')}/{best.get('trial_count', '')}",
                f"disabled_count: {best.get('disabled_count', '')}/{best.get('trial_count', '')}",
                f"range_rmse_selected_mean_m: {best.get('range_rmse_selected_mean_m', '')}",
                f"target_max_abs_error_selected_max_m: {best.get('target_max_abs_error_selected_max_m', '')}",
                "runtime_truth_used: False",
                str(payload["coordinate_contract"]),
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(pipe.json_safe(payload), ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
