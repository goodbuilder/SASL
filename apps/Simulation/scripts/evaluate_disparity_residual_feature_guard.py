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
import run_runtime_correlated_repeated_observation_sweep as correlated  # noqa: E402
import run_runtime_multiburst_disparity_residual_sweep as residual  # noqa: E402
from stereo_ground_sim.runtime_deployment_selector import (  # noqa: E402
    DisparityResidualRuntimeProfile,
)


def parse_scenarios(value: str) -> list[str]:
    return transfer.parse_scenarios(value)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate truth-free feature-domain guards for residual disparity profile catalog transfer."
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
    parser.add_argument("--z-p95-thresholds", type=correlated.parse_float_list, default=correlated.parse_float_list("2,3,4,5,6"))
    parser.add_argument("--correction-p95-thresholds", type=correlated.parse_float_list, default=correlated.parse_float_list("0.01,0.03,0.05,0.1,0.2,0.5"))
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


def feature_stats(series: residual.MultiburstProfileSeries, profile: DisparityResidualRuntimeProfile) -> dict[str, float]:
    values = residual.feature_values(series.fused_disparity_px, feature_space=profile.feature_space)
    z = (values - float(profile.feature_mean)) / max(float(profile.feature_scale), 1.0e-12)
    correction = residual.normalized_vander(
        values,
        degree=int(profile.degree),
        mean=float(profile.feature_mean),
        scale=float(profile.feature_scale),
    ) @ np.asarray(profile.coefficients_low_to_high, dtype=float)
    if float(profile.correction_clip) > 0.0:
        correction = np.clip(correction, -float(profile.correction_clip), float(profile.correction_clip))
    finite_z = np.abs(z[np.isfinite(z)])
    finite_correction = np.abs(correction[np.isfinite(correction)])
    return {
        "feature_z_abs_p95": float(np.percentile(finite_z, 95.0)) if finite_z.size else float("inf"),
        "feature_z_abs_max": float(np.max(finite_z)) if finite_z.size else float("inf"),
        "correction_abs_p95": float(np.percentile(finite_correction, 95.0)) if finite_correction.size else float("inf"),
        "correction_abs_max": float(np.max(finite_correction)) if finite_correction.size else float("inf"),
    }


def apply_guarded_points(
    *,
    series: residual.MultiburstProfileSeries,
    runtime_profile: DisparityResidualRuntimeProfile,
    apply_residual: bool,
) -> np.ndarray:
    if apply_residual:
        return transfer.apply_runtime_profile(series, runtime_profile)
    return np.asarray(series.observation_points, dtype=float)


def evaluate_guarded_series(
    *,
    eval_scenario: str,
    profile_label: str,
    series: residual.MultiburstProfileSeries,
    runtime_profile: DisparityResidualRuntimeProfile,
    z_p95_threshold: float,
    correction_p95_threshold: float,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    stats = feature_stats(series, runtime_profile)
    apply_residual = (
        math.isfinite(stats["feature_z_abs_p95"])
        and math.isfinite(stats["correction_abs_p95"])
        and stats["feature_z_abs_p95"] <= float(z_p95_threshold)
        and stats["correction_abs_p95"] <= float(correction_p95_threshold)
    )
    guarded_points = apply_guarded_points(
        series=series,
        runtime_profile=runtime_profile,
        apply_residual=apply_residual,
    )
    validation_samples: list[pipe.CalibrationSample] = []
    observation_validation: list[np.ndarray] = []
    guarded_validation: list[np.ndarray] = []
    for sample, observation_point, guarded_point in zip(series.samples, series.observation_points, guarded_points):
        if sample.sample_id in series.validation_ids:
            validation_samples.append(sample)
            observation_validation.append(observation_point)
            guarded_validation.append(guarded_point)
    observation_arr = np.vstack(observation_validation) if observation_validation else np.empty((0, 3), dtype=float)
    guarded_arr = np.vstack(guarded_validation) if guarded_validation else np.empty((0, 3), dtype=float)
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
    guarded_metrics = pipe.evaluate_predictions(
        validation_samples,
        guarded_arr,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    passed = (
        guarded_metrics["range_rmse_m"] < raw_metrics["range_rmse_m"]
        and guarded_metrics["position_rmse_m"] < raw_metrics["position_rmse_m"]
        and guarded_metrics["target_window_count"] > 0
        and guarded_metrics["target_max_abs_error_m"] <= 10.0
    )
    return {
        "eval_scenario": eval_scenario,
        "profile_label": profile_label,
        "seed": series.seed,
        "z_p95_threshold": float(z_p95_threshold),
        "correction_p95_threshold": float(correction_p95_threshold),
        "guard_applied_residual": apply_residual,
        "status": "PASS" if passed else "FAIL",
        **stats,
        "range_rmse_raw_m": raw_metrics["range_rmse_m"],
        "range_rmse_observation_m": observation_metrics["range_rmse_m"],
        "range_rmse_guarded_m": guarded_metrics["range_rmse_m"],
        "position_rmse_raw_m": raw_metrics["position_rmse_m"],
        "position_rmse_observation_m": observation_metrics["position_rmse_m"],
        "position_rmse_guarded_m": guarded_metrics["position_rmse_m"],
        "target_max_abs_error_observation_m": observation_metrics["target_max_abs_error_m"],
        "target_max_abs_error_guarded_m": guarded_metrics["target_max_abs_error_m"],
        "target_rmse_observation_m": observation_metrics["target_rmse_m"],
        "target_rmse_guarded_m": guarded_metrics["target_rmse_m"],
        "runtime_truth_used": False,
    }


def aggregate(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, float, float], list[Mapping[str, object]]] = {}
    for row in rows:
        grouped.setdefault(
            (str(row["profile_label"]), float(row["z_p95_threshold"]), float(row["correction_p95_threshold"])),
            [],
        ).append(row)
    summaries: list[dict[str, object]] = []
    for (profile_label, z_threshold, correction_threshold), items in grouped.items():
        values = lambda name: np.asarray([float(row[name]) for row in items], dtype=float)
        pass_count = sum(1 for row in items if row.get("status") == "PASS")
        apply_count = sum(1 for row in items if row.get("guard_applied_residual") is True)
        label_matched_items = [row for row in items if row.get("eval_scenario") == row.get("profile_label")]
        label_matched_pass = sum(1 for row in label_matched_items if row.get("status") == "PASS")
        wrong_items = [row for row in items if row.get("eval_scenario") != row.get("profile_label")]
        wrong_apply = sum(1 for row in wrong_items if row.get("guard_applied_residual") is True)
        viable_guard = (
            bool(label_matched_items)
            and label_matched_pass == len(label_matched_items)
            and wrong_apply == 0
            and apply_count > 0
        )
        summaries.append(
            {
                "profile_label": profile_label,
                "z_p95_threshold": z_threshold,
                "correction_p95_threshold": correction_threshold,
                "status": "STRICT_PASS" if pass_count == len(items) else "FAIL",
                "viable_guard": viable_guard,
                "pass_count": pass_count,
                "trial_count": len(items),
                "apply_count": apply_count,
                "label_matched_pass_count": label_matched_pass,
                "label_matched_trial_count": len(label_matched_items),
                "wrong_profile_apply_count": wrong_apply,
                "wrong_profile_trial_count": len(wrong_items),
                "range_rmse_guarded_mean_m": float(np.mean(values("range_rmse_guarded_m"))),
                "target_max_abs_error_guarded_max_m": float(np.max(values("target_max_abs_error_guarded_m"))),
                "target_rmse_guarded_mean_m": float(np.mean(values("target_rmse_guarded_m"))),
                "runtime_truth_used": False,
            }
        )
    return sorted(
        summaries,
        key=lambda row: (
            not bool(row["viable_guard"]),
            -int(row["label_matched_pass_count"]),
            int(row["wrong_profile_apply_count"]),
            -int(row["apply_count"]),
            -int(row["pass_count"]),
            float(row["range_rmse_guarded_mean_m"]),
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
    series_cache: dict[tuple[str, int, str], residual.MultiburstProfileSeries] = {}
    rows: list[dict[str, object]] = []
    for eval_scenario in args.scenarios:
        for seed_index in range(int(args.trial_count)):
            seed = int(args.seed_start) + seed_index
            for profile_label, runtime_profile in runtime_profiles.items():
                observation = profile_payloads[profile_label].get("observation_fusion", {})
                if not isinstance(observation, Mapping):
                    observation = {}
                cache_key = (
                    eval_scenario,
                    seed,
                    json.dumps(observation, sort_keys=True, ensure_ascii=False),
                )
                if cache_key not in series_cache:
                    series_cache[cache_key] = residual.build_series(
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
                series = series_cache[cache_key]
                for z_threshold in args.z_p95_thresholds:
                    for correction_threshold in args.correction_p95_thresholds:
                        rows.append(
                            evaluate_guarded_series(
                                eval_scenario=eval_scenario,
                                profile_label=profile_label,
                                series=series,
                                runtime_profile=runtime_profile,
                                z_p95_threshold=float(z_threshold),
                                correction_p95_threshold=float(correction_threshold),
                                target_range_m=float(args.target_range_m),
                                target_window_m=float(args.target_window_m),
                            )
                        )
    summaries = aggregate(rows)
    best = summaries[0] if summaries else {}
    viable_summaries = [row for row in summaries if row.get("viable_guard") is True]
    seed_csv = args.output_dir / "disparity_residual_feature_guard_seed_metrics.csv"
    summary_csv = args.output_dir / "disparity_residual_feature_guard_summary.csv"
    write_csv(seed_csv, rows)
    write_csv(summary_csv, summaries)
    payload = {
        "mode": "disparity_residual_feature_guard_evaluation",
        "status": "PASS" if viable_summaries else "FAIL",
        "catalog_json": str(args.catalog_json),
        "base_profile_json": str(args.base_profile_json),
        "best_guard": best,
        "viable_guard_count": len(viable_summaries),
        "output_seed_csv": str(seed_csv),
        "output_summary_csv": str(summary_csv),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "coordinate_contract": (
            "guard decisions use measured-disparity feature z-score and residual correction magnitude only; "
            "truth is offline evaluation only; runtime residual application consumes measured disparity and "
            "corrected stereo-center RDF right/down/forward"
        ),
    }
    write_json(args.output_dir / "disparity_residual_feature_guard_summary.json", payload)
    (args.output_dir / "disparity_residual_feature_guard_summary.txt").write_text(
        "\n".join(
            [
                "Disparity residual feature-domain guard evaluation",
                f"status: {payload['status']}",
                f"best_profile_label: {best.get('profile_label', '')}",
                f"best_z_p95_threshold: {best.get('z_p95_threshold', '')}",
                f"best_correction_p95_threshold: {best.get('correction_p95_threshold', '')}",
                f"label_matched_pass: {best.get('label_matched_pass_count', '')}/{best.get('label_matched_trial_count', '')}",
                f"wrong_profile_apply: {best.get('wrong_profile_apply_count', '')}/{best.get('wrong_profile_trial_count', '')}",
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
