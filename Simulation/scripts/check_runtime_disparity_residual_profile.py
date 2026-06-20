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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check a runtime disparity residual profile JSON on synthetic validation split.")
    parser.add_argument("--residual-profile-json", type=Path, required=True)
    parser.add_argument("--base-profile-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scenario", default="disparity_noise_stress", choices=tuple(replay_audit.SCENARIO_PRESETS))
    parser.add_argument("--seed", type=int, default=21220000)
    parser.add_argument("--sample-count", type=int, default=1680)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-oversample-count", type=int, default=72)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    runtime_profile = DisparityResidualRuntimeProfile.from_json_file(args.residual_profile_json)
    payload = json.loads(args.residual_profile_json.read_text(encoding="utf-8"))
    observation = payload.get("observation_fusion", {})
    if not isinstance(observation, Mapping):
        observation = {}
    base_profile = replay_audit.load_profile(args.base_profile_json)
    series = residual.build_series(
        profile=base_profile,
        seed=int(args.seed),
        scenario=str(args.scenario),
        sample_count=int(args.sample_count),
        train_ratio=float(args.train_ratio),
        target_range_m=float(args.target_range_m),
        target_window_m=float(args.target_window_m),
        target_oversample_count=int(args.target_oversample_count),
        burst_count=int(float(observation.get("burst_count", 7))),
        observations_per_burst=int(float(observation.get("observations_per_burst", 3))),
        inner_mode=str(observation.get("inner_mode", "trimmed_mean")),
        outer_mode=str(observation.get("outer_mode", "trimmed_mean")),
        trim_fraction=float(observation.get("trim_fraction", 0.05)),
        common_fraction=float(observation.get("common_fraction", 0.5)),
    )
    validation_samples: list[pipe.CalibrationSample] = []
    observation_points: list[np.ndarray] = []
    residual_points: list[np.ndarray] = []
    rows: list[dict[str, object]] = []
    for sample, point, disparity in zip(series.samples, series.observation_points, series.fused_disparity_px):
        runtime_sample = RuntimeMeasurementSample(
            raw_right_m=float(point[0]),
            raw_down_m=float(point[1]),
            raw_forward_m=float(point[2]),
            measured_disparity_px=float(disparity),
        )
        corrected = runtime_profile.apply(runtime_sample)
        if sample.sample_id not in series.validation_ids or corrected is None:
            continue
        residual_point = np.array(
            [corrected.raw_right_m, corrected.raw_down_m, corrected.raw_forward_m],
            dtype=float,
        )
        validation_samples.append(sample)
        observation_points.append(np.asarray(point, dtype=float))
        residual_points.append(residual_point)
        rows.append(
            {
                "sample_id": sample.sample_id,
                "split": "validation",
                "measured_disparity_px": float(disparity),
                "observation_right_m": float(point[0]),
                "observation_down_m": float(point[1]),
                "observation_forward_m": float(point[2]),
                "residual_right_m": float(residual_point[0]),
                "residual_down_m": float(residual_point[1]),
                "residual_forward_m": float(residual_point[2]),
                "truth_right_m": float(sample.truth_rdf_m[0]),
                "truth_down_m": float(sample.truth_rdf_m[1]),
                "truth_forward_m": float(sample.truth_rdf_m[2]),
                "runtime_truth_used": False,
                "rdf_axis_contract": residual.RDF_AXIS_CONTRACT,
                "public_topic_contract": residual.PUBLIC_TOPIC_CONTRACT,
            }
        )
    observation_arr = np.vstack(observation_points) if observation_points else np.empty((0, 3), dtype=float)
    residual_arr = np.vstack(residual_points) if residual_points else np.empty((0, 3), dtype=float)
    observation_metrics = pipe.evaluate_predictions(
        validation_samples,
        observation_arr,
        target_range_m=float(args.target_range_m),
        target_window_m=float(args.target_window_m),
    )
    residual_metrics = pipe.evaluate_predictions(
        validation_samples,
        residual_arr,
        target_range_m=float(args.target_range_m),
        target_window_m=float(args.target_window_m),
    )
    raw_metrics = pipe.evaluate_predictions(
        validation_samples,
        series.raw_validation_points,
        target_range_m=float(args.target_range_m),
        target_window_m=float(args.target_window_m),
    )
    status = (
        "PASS"
        if residual_metrics["range_rmse_m"] < raw_metrics["range_rmse_m"]
        and residual_metrics["position_rmse_m"] < raw_metrics["position_rmse_m"]
        and residual_metrics["target_window_count"] > 0
        and residual_metrics["target_max_abs_error_m"] <= 10.0
        else "FAIL"
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_csv = args.output_dir / "runtime_disparity_residual_profile_validation_samples.csv"
    write_csv(output_csv, rows)
    summary = {
        "status": status,
        "residual_profile_json": str(args.residual_profile_json),
        "base_profile_json": str(args.base_profile_json),
        "scenario": str(args.scenario),
        "seed": int(args.seed),
        "validation_sample_count": len(validation_samples),
        "range_rmse_raw_m": raw_metrics["range_rmse_m"],
        "range_rmse_observation_m": observation_metrics["range_rmse_m"],
        "range_rmse_residual_m": residual_metrics["range_rmse_m"],
        "position_rmse_raw_m": raw_metrics["position_rmse_m"],
        "position_rmse_observation_m": observation_metrics["position_rmse_m"],
        "position_rmse_residual_m": residual_metrics["position_rmse_m"],
        "target_max_abs_error_raw_m": raw_metrics["target_max_abs_error_m"],
        "target_max_abs_error_observation_m": observation_metrics["target_max_abs_error_m"],
        "target_max_abs_error_residual_m": residual_metrics["target_max_abs_error_m"],
        "target_rmse_raw_m": raw_metrics["target_rmse_m"],
        "target_rmse_observation_m": observation_metrics["target_rmse_m"],
        "target_rmse_residual_m": residual_metrics["target_rmse_m"],
        "residual_improves_raw_range": residual_metrics["range_rmse_m"] < raw_metrics["range_rmse_m"],
        "residual_improves_raw_position": residual_metrics["position_rmse_m"] < raw_metrics["position_rmse_m"],
        "residual_improves_observation_range": residual_metrics["range_rmse_m"]
        < observation_metrics["range_rmse_m"],
        "residual_improves_observation_position": residual_metrics["position_rmse_m"]
        < observation_metrics["position_rmse_m"],
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "runtime_input_contract": payload.get("runtime_input_contract", ""),
        "rdf_axis_contract": residual.RDF_AXIS_CONTRACT,
        "public_topic_contract": residual.PUBLIC_TOPIC_CONTRACT,
        "output_csv": str(output_csv),
    }
    (args.output_dir / "runtime_disparity_residual_profile_check.json").write_text(
        json.dumps(pipe.json_safe(summary), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (args.output_dir / "runtime_disparity_residual_profile_check.txt").write_text(
        "\n".join(
            [
                "Runtime disparity residual profile check",
                f"status: {summary['status']}",
                f"scenario: {summary['scenario']}",
                f"seed: {summary['seed']}",
                f"range_rmse_raw_m: {summary['range_rmse_raw_m']:.6f}",
                f"range_rmse_observation_m: {summary['range_rmse_observation_m']:.6f}",
                f"range_rmse_residual_m: {summary['range_rmse_residual_m']:.6f}",
                f"target_max_abs_error_raw_m: {summary['target_max_abs_error_raw_m']:.6f}",
                f"target_max_abs_error_observation_m: {summary['target_max_abs_error_observation_m']:.6f}",
                f"target_max_abs_error_residual_m: {summary['target_max_abs_error_residual_m']:.6f}",
                "runtime_truth_used: False",
                "coordinate_contract: runtime apply uses fused measured_disparity_px and corrected stereo-center RDF right/down/forward only; truth is validation-only.",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(pipe.json_safe(summary), ensure_ascii=False, indent=2))
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
