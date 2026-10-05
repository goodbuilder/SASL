#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
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
import rtk_stereo_calibration_pipeline as pipe  # noqa: E402
import run_runtime_multiburst_disparity_residual_sweep as residual  # noqa: E402
from stereo_ground_sim.runtime_deployment_selector import (  # noqa: E402
    DisparityResidualRuntimeProfile,
    RuntimeMeasurementSample,
)


def parse_scenarios(value: str) -> list[str]:
    scenarios = [item.strip() for item in value.split(",") if item.strip()]
    unknown = [scenario for scenario in scenarios if scenario not in replay_audit.SCENARIO_PRESETS]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown scenarios: {','.join(unknown)}")
    if not scenarios:
        raise argparse.ArgumentTypeError("scenario list must not be empty")
    return scenarios


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate a residual disparity profile catalog as a cross-scenario transfer matrix."
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
    parser.add_argument("--sample-count", type=int, default=1680)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-oversample-count", type=int, default=72)
    parser.add_argument(
        "--label-matched-only",
        action="store_true",
        help="Evaluate only the profile whose catalog label matches each scenario.",
    )
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def resolve_catalog_path(catalog_json: Path, value: object) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else (catalog_json.parent / path).resolve()


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


def catalog_profiles(catalog_json: Path) -> dict[str, Path]:
    catalog = load_json(catalog_json)
    profile_paths = catalog.get("profile_paths", {})
    if not isinstance(profile_paths, Mapping):
        raise ValueError("catalog profile_paths must be an object")
    return {str(label): resolve_catalog_path(catalog_json, path) for label, path in profile_paths.items()}


def apply_runtime_profile(
    series: residual.MultiburstProfileSeries,
    runtime_profile: DisparityResidualRuntimeProfile,
) -> np.ndarray:
    corrected_points: list[np.ndarray] = []
    for point, disparity in zip(series.observation_points, series.fused_disparity_px):
        corrected = runtime_profile.apply(
            RuntimeMeasurementSample(
                raw_right_m=float(point[0]),
                raw_down_m=float(point[1]),
                raw_forward_m=float(point[2]),
                measured_disparity_px=float(disparity),
            )
        )
        if corrected is None:
            corrected_points.append(np.array([np.nan, np.nan, np.nan], dtype=float))
        else:
            corrected_points.append(
                np.array(
                    [corrected.raw_right_m, corrected.raw_down_m, corrected.raw_forward_m],
                    dtype=float,
                )
            )
    return np.vstack(corrected_points)


def evaluate_profile_on_series(
    *,
    eval_scenario: str,
    profile_label: str,
    series: residual.MultiburstProfileSeries,
    runtime_profile: DisparityResidualRuntimeProfile,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    residual_points = apply_runtime_profile(series, runtime_profile)
    validation_samples: list[pipe.CalibrationSample] = []
    observation_validation: list[np.ndarray] = []
    residual_validation: list[np.ndarray] = []
    for sample, observation_point, residual_point in zip(series.samples, series.observation_points, residual_points):
        if sample.sample_id in series.validation_ids:
            validation_samples.append(sample)
            observation_validation.append(observation_point)
            residual_validation.append(residual_point)
    observation_arr = np.vstack(observation_validation) if observation_validation else np.empty((0, 3), dtype=float)
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
    improves_observation_range = residual_metrics["range_rmse_m"] < observation_metrics["range_rmse_m"]
    improves_observation_target = (
        residual_metrics["target_max_abs_error_m"] < observation_metrics["target_max_abs_error_m"]
    )
    return {
        "eval_scenario": eval_scenario,
        "profile_label": profile_label,
        "seed": series.seed,
        "status": "PASS" if passed else "FAIL",
        "range_rmse_raw_m": raw_metrics["range_rmse_m"],
        "range_rmse_observation_m": observation_metrics["range_rmse_m"],
        "range_rmse_residual_m": residual_metrics["range_rmse_m"],
        "position_rmse_raw_m": raw_metrics["position_rmse_m"],
        "position_rmse_observation_m": observation_metrics["position_rmse_m"],
        "position_rmse_residual_m": residual_metrics["position_rmse_m"],
        "target_max_abs_error_observation_m": observation_metrics["target_max_abs_error_m"],
        "target_max_abs_error_residual_m": residual_metrics["target_max_abs_error_m"],
        "target_rmse_observation_m": observation_metrics["target_rmse_m"],
        "target_rmse_residual_m": residual_metrics["target_rmse_m"],
        "improves_observation_range": improves_observation_range,
        "improves_observation_target": improves_observation_target,
        "runtime_truth_used": False,
        "rdf_axis_contract": residual.RDF_AXIS_CONTRACT,
        "public_topic_contract": residual.PUBLIC_TOPIC_CONTRACT,
    }


def aggregate(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str], list[Mapping[str, object]]] = {}
    for row in rows:
        grouped.setdefault((str(row["eval_scenario"]), str(row["profile_label"])), []).append(row)
    summaries: list[dict[str, object]] = []
    for (eval_scenario, profile_label), items in grouped.items():
        values = lambda name: np.asarray([float(row[name]) for row in items], dtype=float)
        pass_count = sum(1 for row in items if row.get("status") == "PASS")
        range_improve_count = sum(1 for row in items if row.get("improves_observation_range") is True)
        target_improve_count = sum(1 for row in items if row.get("improves_observation_target") is True)
        summaries.append(
            {
                "eval_scenario": eval_scenario,
                "profile_label": profile_label,
                "status": "STRICT_PASS" if pass_count == len(items) else "FAIL",
                "pass_count": pass_count,
                "trial_count": len(items),
                "range_improves_observation_count": range_improve_count,
                "target_improves_observation_count": target_improve_count,
                "range_rmse_observation_mean_m": float(np.mean(values("range_rmse_observation_m"))),
                "range_rmse_residual_mean_m": float(np.mean(values("range_rmse_residual_m"))),
                "position_rmse_observation_mean_m": float(np.mean(values("position_rmse_observation_m"))),
                "position_rmse_residual_mean_m": float(np.mean(values("position_rmse_residual_m"))),
                "target_max_abs_error_observation_max_m": float(np.max(values("target_max_abs_error_observation_m"))),
                "target_max_abs_error_residual_max_m": float(np.max(values("target_max_abs_error_residual_m"))),
                "target_rmse_observation_mean_m": float(np.mean(values("target_rmse_observation_m"))),
                "target_rmse_residual_mean_m": float(np.mean(values("target_rmse_residual_m"))),
                "runtime_truth_used": False,
            }
        )
    return sorted(
        summaries,
        key=lambda row: (
            str(row["eval_scenario"]),
            -int(row["pass_count"]),
            float(row["target_max_abs_error_residual_max_m"]),
            float(row["range_rmse_residual_mean_m"]),
        ),
    )


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile = replay_audit.load_profile(args.base_profile_json)
    profile_paths = catalog_profiles(args.catalog_json)
    runtime_profiles = {
        label: DisparityResidualRuntimeProfile.from_json_file(path)
        for label, path in profile_paths.items()
    }
    profile_payloads = {label: load_json(path) for label, path in profile_paths.items()}
    rows: list[dict[str, object]] = []
    for eval_scenario in args.scenarios:
        if args.label_matched_only and eval_scenario not in runtime_profiles:
            raise ValueError(f"missing label-matched profile for scenario: {eval_scenario}")
        for seed_index in range(int(args.trial_count)):
            seed = int(args.seed_start) + seed_index
            profile_items = (
                [(eval_scenario, runtime_profiles[eval_scenario])]
                if args.label_matched_only
                else list(runtime_profiles.items())
            )
            for profile_label, runtime_profile in profile_items:
                observation = profile_payloads[profile_label].get("observation_fusion", {})
                if not isinstance(observation, Mapping):
                    observation = {}
                series = residual.build_series(
                    profile=profile,
                    seed=seed,
                    scenario=eval_scenario,
                    sample_count=int(args.sample_count),
                    train_ratio=float(args.train_ratio),
                    target_range_m=float(args.target_range_m),
                    target_window_m=float(args.target_window_m),
                    target_oversample_count=int(args.target_oversample_count),
                    burst_count=int(observation.get("burst_count", 7)),
                    observations_per_burst=int(observation.get("observations_per_burst", 3)),
                    inner_mode=str(observation.get("inner_mode", "trimmed_mean")),
                    outer_mode=str(observation.get("outer_mode", "trimmed_mean")),
                    trim_fraction=float(observation.get("trim_fraction", 0.05)),
                    common_fraction=float(observation.get("common_fraction", 0.5)),
                )
                rows.append(
                    evaluate_profile_on_series(
                        eval_scenario=eval_scenario,
                        profile_label=profile_label,
                        series=series,
                        runtime_profile=runtime_profile,
                        target_range_m=float(args.target_range_m),
                        target_window_m=float(args.target_window_m),
                    )
                )
    summaries = aggregate(rows)
    best_by_scenario: dict[str, Mapping[str, object]] = {}
    for row in summaries:
        best_by_scenario.setdefault(str(row["eval_scenario"]), row)
    label_matched = [
        row for row in summaries if str(row["eval_scenario"]) == str(row["profile_label"])
    ]
    label_matched_pass = sum(1 for row in label_matched if row.get("status") == "STRICT_PASS")
    output_seed_csv = args.output_dir / "disparity_residual_catalog_transfer_seed_metrics.csv"
    output_matrix_csv = args.output_dir / "disparity_residual_catalog_transfer_matrix.csv"
    write_csv(output_seed_csv, rows)
    write_csv(output_matrix_csv, summaries)
    summary = {
        "mode": "disparity_residual_catalog_transfer_evaluation",
        "status": "PASS" if label_matched_pass == len(args.scenarios) else "FAIL",
        "catalog_json": str(args.catalog_json),
        "base_profile_json": str(args.base_profile_json),
        "scenario_count": len(args.scenarios),
        "profile_count": len(runtime_profiles),
        "label_matched_only": bool(args.label_matched_only),
        "seed_start": int(args.seed_start),
        "seed_end": int(args.seed_start) + int(args.trial_count) - 1,
        "trial_count": int(args.trial_count),
        "label_matched_strict_pass_count": label_matched_pass,
        "label_matched_scenario_count": len(args.scenarios),
        "best_by_scenario": best_by_scenario,
        "output_seed_csv": str(output_seed_csv),
        "output_matrix_csv": str(output_matrix_csv),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "coordinate_contract": (
            "truth is used only for offline transfer metrics; residual profile application consumes "
            "measured disparity and corrected stereo-center RDF right/down/forward; public output remains FRD+range"
        ),
        "failure_count": 0 if label_matched_pass == len(args.scenarios) else len(args.scenarios) - label_matched_pass,
        "failures": [] if label_matched_pass == len(args.scenarios) else ["label_matched_profiles:not_all_strict_pass"],
    }
    write_json(args.output_dir / "disparity_residual_catalog_transfer_summary.json", summary)
    (args.output_dir / "disparity_residual_catalog_transfer_summary.txt").write_text(
        "\n".join(
            [
                "Disparity residual catalog transfer evaluation",
                f"status: {summary['status']}",
                f"catalog_json: {summary['catalog_json']}",
                f"label_matched: {label_matched_pass}/{len(args.scenarios)}",
                f"output_matrix_csv: {output_matrix_csv}",
                "runtime_truth_used: False",
                str(summary["coordinate_contract"]),
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> None:
    summary = run()
    print(json.dumps(pipe.json_safe(summary), ensure_ascii=False, indent=2, allow_nan=False))
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
