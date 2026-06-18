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
import run_runtime_correlated_repeated_observation_sweep as correlated  # noqa: E402
import run_runtime_multiburst_disparity_residual_sweep as residual  # noqa: E402
import run_runtime_multiburst_observation_fusion_sweep as multiburst  # noqa: E402
import run_runtime_repeated_observation_fusion_sweep as repeated  # noqa: E402


def parse_scenarios(value: str) -> list[str]:
    scenarios = [item.strip() for item in value.split(",") if item.strip()]
    unknown = [scenario for scenario in scenarios if scenario not in replay_audit.SCENARIO_PRESETS]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown scenarios: {','.join(unknown)}")
    if not scenarios:
        raise argparse.ArgumentTypeError("scenario list must not be empty")
    return scenarios


def parse_str_list(value: str) -> list[str]:
    parsed = [item.strip() for item in value.split(",") if item.strip()]
    if not parsed:
        raise argparse.ArgumentTypeError("list must not be empty")
    return parsed


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


def train_arrays(
    series_list: Sequence[residual.MultiburstProfileSeries],
    *,
    feature_space: str,
    correction_space: str,
) -> tuple[np.ndarray, np.ndarray]:
    features: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    for series in series_list:
        train_mask = np.asarray([sample.sample_id in series.train_ids for sample in series.samples], dtype=bool)
        points = np.asarray(series.observation_points, dtype=float)
        truth = np.vstack([sample.truth_rdf_m for sample in series.samples])
        if correction_space == "forward_add":
            target = truth[:, 2] - points[:, 2]
        elif correction_space == "ray_scale":
            denom = np.sum(points * points, axis=1)
            target = np.sum(truth * points, axis=1) / np.maximum(denom, 1.0e-12) - 1.0
        else:
            raise ValueError("correction_space must be forward_add or ray_scale")
        features.append(residual.feature_values(series.fused_disparity_px[train_mask], feature_space=feature_space))
        targets.append(target[train_mask])
    return np.concatenate(features), np.concatenate(targets)


def fit_pooled_model(
    series_list: Sequence[residual.MultiburstProfileSeries],
    *,
    feature_space: str,
    correction_space: str,
    degree: int,
    ridge_lambda: float,
    correction_clip: float,
) -> residual.ResidualModel:
    x, y = train_arrays(series_list, feature_space=feature_space, correction_space=correction_space)
    mean, scale, coeff = residual.fit_ridge_polynomial(
        x,
        y,
        degree=int(degree),
        ridge_lambda=float(ridge_lambda),
    )
    return residual.ResidualModel(
        feature_space=str(feature_space),
        correction_space=str(correction_space),
        degree=int(degree),
        ridge_lambda=float(ridge_lambda),
        correction_clip=float(correction_clip),
        feature_mean=mean,
        feature_scale=scale,
        coefficients_low_to_high=coeff,
    )


def evaluate_series(
    series: residual.MultiburstProfileSeries,
    model: residual.ResidualModel,
    *,
    method: str,
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    row = residual.evaluate_model(
        series,
        model,
        burst_count=burst_count,
        observations_per_burst=observations_per_burst,
        inner_mode=inner_mode,
        outer_mode=outer_mode,
        trim_fraction=trim_fraction,
        common_fraction=common_fraction,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    row["method"] = method
    row["fit_scope"] = "pooled_train_splits"
    return row


def aggregate(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row["method"]), []).append(row)
    summaries: list[dict[str, object]] = []
    for method, method_rows in grouped.items():
        values = lambda name: np.asarray([float(row[name]) for row in method_rows], dtype=float)
        pass_count = sum(1 for row in method_rows if row.get("status") == "PASS")
        scenarios = sorted({str(row["scenario"]) for row in method_rows})
        first = method_rows[0]
        summaries.append(
            {
                "method": method,
                "status": "STRICT_PASS" if pass_count == len(method_rows) else "FAIL",
                "pass_count": pass_count,
                "trial_count": len(method_rows),
                "scenario_count": len(scenarios),
                "scenarios": ",".join(scenarios),
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
                "target_max_abs_error_observation_max_m": float(
                    np.max(values("target_max_abs_error_observation_m"))
                ),
                "target_max_abs_error_residual_max_m": float(np.max(values("target_max_abs_error_residual_m"))),
                "range_rmse_observation_mean_m": float(np.mean(values("range_rmse_observation_m"))),
                "range_rmse_residual_mean_m": float(np.mean(values("range_rmse_residual_m"))),
                "position_rmse_observation_mean_m": float(np.mean(values("position_rmse_observation_m"))),
                "position_rmse_residual_mean_m": float(np.mean(values("position_rmse_residual_m"))),
                "runtime_truth_used": False,
                "runtime_input_contract": residual.RUNTIME_INPUT_CONTRACT,
                "rdf_axis_contract": residual.RDF_AXIS_CONTRACT,
                "public_topic_contract": residual.PUBLIC_TOPIC_CONTRACT,
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


def profile_payload(
    *,
    profile_id: str,
    method: str,
    model: residual.ResidualModel,
    profile_json: Path,
    scenarios: Sequence[str],
    seed_start: int,
    trial_count: int,
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
) -> dict[str, object]:
    return {
        "profile_type": "DisparityResidualRuntimeProfile",
        "profile_id": profile_id,
        "method": method,
        "feature_space": model.feature_space,
        "correction_space": model.correction_space,
        "degree": model.degree,
        "ridge_lambda": model.ridge_lambda,
        "correction_clip": model.correction_clip,
        "feature_mean": model.feature_mean,
        "feature_scale": model.feature_scale,
        "coefficients_low_to_high": list(model.coefficients_low_to_high),
        "observation_fusion": {
            "burst_count": int(burst_count),
            "observations_per_burst": int(observations_per_burst),
            "inner_mode": inner_mode,
            "outer_mode": outer_mode,
            "trim_fraction": float(trim_fraction),
            "common_fraction": float(common_fraction),
        },
        "fit_source": {
            "base_profile_json": str(profile_json),
            "scenarios": list(scenarios),
            "seed_start": int(seed_start),
            "seed_end": int(seed_start) + int(trial_count) - 1,
            "trial_count_per_scenario": int(trial_count),
            "sample_count": int(sample_count),
            "train_ratio": float(train_ratio),
            "target_range_m": float(target_range_m),
            "target_window_m": float(target_window_m),
            "target_oversample_count": int(target_oversample_count),
            "truth_usage": "pooled_calibration_train_splits_only",
        },
        "runtime_inputs": [
            "raw_right_m",
            "raw_down_m",
            "raw_forward_m",
            "measured_disparity_px",
            "left_u_px",
            "left_v_px",
        ],
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "runtime_input_contract": residual.RUNTIME_INPUT_CONTRACT,
        "rdf_axis_contract": residual.RDF_AXIS_CONTRACT,
        "public_topic_contract": residual.PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": residual.TRUTH_RDF_TRANSFORM_CONTRACT,
        "ned_to_rdf_contract": residual.NED_TO_RDF_CONTRACT,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Search and export one pooled runtime disparity residual profile across scenarios and seeds."
    )
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
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
    parser.add_argument("--burst-specs", type=multiburst.parse_burst_specs, default=multiburst.parse_burst_specs("7x3"))
    parser.add_argument("--inner-modes", type=repeated.parse_modes, default=repeated.parse_modes("trimmed_mean"))
    parser.add_argument("--outer-modes", type=repeated.parse_modes, default=repeated.parse_modes("trimmed_mean"))
    parser.add_argument("--trim-fractions", type=correlated.parse_float_list, default=correlated.parse_float_list("0.05"))
    parser.add_argument("--common-fractions", type=correlated.parse_float_list, default=correlated.parse_float_list("0.5"))
    parser.add_argument("--feature-spaces", type=parse_str_list, default=parse_str_list("log_disparity,disparity"))
    parser.add_argument("--correction-spaces", type=parse_str_list, default=parse_str_list("ray_scale,forward_add"))
    parser.add_argument("--degrees", type=residual.parse_int_list, default=residual.parse_int_list("1,2,3"))
    parser.add_argument("--ridge-lambdas", type=correlated.parse_float_list, default=correlated.parse_float_list("0,0.1,1"))
    parser.add_argument("--correction-clips", type=correlated.parse_float_list, default=correlated.parse_float_list("0.5,1,2"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile = replay_audit.load_profile(args.profile_json)
    all_rows: list[dict[str, object]] = []
    model_by_method: dict[str, residual.ResidualModel] = {}
    best_context_by_method: dict[str, dict[str, object]] = {}
    for burst_count, observations_per_burst in args.burst_specs:
        for inner_mode in args.inner_modes:
            for outer_mode in args.outer_modes:
                for trim_fraction in args.trim_fractions:
                    for common_fraction in args.common_fractions:
                        series_list = [
                            residual.build_series(
                                profile=profile,
                                seed=int(args.seed_start) + seed_index,
                                scenario=scenario,
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
                            for scenario in args.scenarios
                            for seed_index in range(int(args.trial_count))
                        ]
                        for feature_space in args.feature_spaces:
                            for correction_space in args.correction_spaces:
                                for degree in args.degrees:
                                    for ridge_lambda in args.ridge_lambdas:
                                        for correction_clip in args.correction_clips:
                                            model = fit_pooled_model(
                                                series_list,
                                                feature_space=str(feature_space),
                                                correction_space=str(correction_space),
                                                degree=int(degree),
                                                ridge_lambda=float(ridge_lambda),
                                                correction_clip=float(correction_clip),
                                            )
                                            method = residual.method_name(
                                                burst_count=int(burst_count),
                                                observations_per_burst=int(observations_per_burst),
                                                inner_mode=str(inner_mode),
                                                outer_mode=str(outer_mode),
                                                trim_fraction=float(trim_fraction),
                                                common_fraction=float(common_fraction),
                                                feature_space=model.feature_space,
                                                correction_space=model.correction_space,
                                                degree=model.degree,
                                                ridge_lambda=model.ridge_lambda,
                                                correction_clip=model.correction_clip,
                                            ) + "_pooled"
                                            model_by_method[method] = model
                                            best_context_by_method[method] = {
                                                "burst_count": int(burst_count),
                                                "observations_per_burst": int(observations_per_burst),
                                                "inner_mode": str(inner_mode),
                                                "outer_mode": str(outer_mode),
                                                "trim_fraction": float(trim_fraction),
                                                "common_fraction": float(common_fraction),
                                            }
                                            for series in series_list:
                                                all_rows.append(
                                                    evaluate_series(
                                                        series,
                                                        model,
                                                        method=method,
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
    summaries = aggregate(all_rows)
    best = summaries[0] if summaries else {}
    best_method = str(best.get("method", ""))
    best_model = model_by_method.get(best_method)
    best_context = best_context_by_method.get(best_method, {})
    profile_json = args.output_dir / "pooled_disparity_residual_profile.json"
    if best_model is not None:
        payload = profile_payload(
            profile_id=f"{args.experiment_id}_pooled_disparity_residual",
            method=best_method,
            model=best_model,
            profile_json=args.profile_json,
            scenarios=args.scenarios,
            seed_start=int(args.seed_start),
            trial_count=int(args.trial_count),
            sample_count=int(args.sample_count),
            train_ratio=float(args.train_ratio),
            target_range_m=float(args.target_range_m),
            target_window_m=float(args.target_window_m),
            target_oversample_count=int(args.target_oversample_count),
            burst_count=int(best_context["burst_count"]),
            observations_per_burst=int(best_context["observations_per_burst"]),
            inner_mode=str(best_context["inner_mode"]),
            outer_mode=str(best_context["outer_mode"]),
            trim_fraction=float(best_context["trim_fraction"]),
            common_fraction=float(best_context["common_fraction"]),
        )
        profile_json.write_text(json.dumps(pipe.json_safe(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_csv(args.output_dir / "pooled_disparity_residual_seed_metrics.csv", all_rows)
    write_csv(args.output_dir / "pooled_disparity_residual_method_summary.csv", summaries)
    summary = {
        "experiment_id": args.experiment_id,
        "profile_json": str(args.profile_json),
        "output_profile_json": str(profile_json) if best_model is not None else "",
        "scenario_count": len(args.scenarios),
        "scenarios": list(args.scenarios),
        "seed_start": int(args.seed_start),
        "seed_end": int(args.seed_start) + int(args.trial_count) - 1,
        "trial_count_per_scenario": int(args.trial_count),
        "best_method": best,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "runtime_input_contract": residual.RUNTIME_INPUT_CONTRACT,
        "rdf_axis_contract": residual.RDF_AXIS_CONTRACT,
        "public_topic_contract": residual.PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": residual.TRUTH_RDF_TRANSFORM_CONTRACT,
        "ned_to_rdf_contract": residual.NED_TO_RDF_CONTRACT,
    }
    (args.output_dir / "pooled_disparity_residual_summary.json").write_text(
        json.dumps(pipe.json_safe(summary), ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    (args.output_dir / "pooled_disparity_residual_summary.txt").write_text(
        "\n".join(
            [
                "Pooled runtime disparity residual profile search",
                f"experiment_id: {args.experiment_id}",
                f"status: {best.get('status', '')}",
                f"best_method: {best.get('method', '')}",
                f"pass_count: {best.get('pass_count', '')}/{best.get('trial_count', '')}",
                f"target_max_abs_error_residual_max_m: {best.get('target_max_abs_error_residual_max_m', '')}",
                f"range_rmse_residual_mean_m: {best.get('range_rmse_residual_mean_m', '')}",
                f"output_profile_json: {summary['output_profile_json']}",
                "runtime_truth_used: False",
                "Coordinate contract: pooled fit uses calibration/evaluation truth only for train/eval; runtime profile consumes measured disparity and stereo-center RDF right/down/forward.",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(pipe.json_safe(summary), ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if best.get("status") == "STRICT_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
