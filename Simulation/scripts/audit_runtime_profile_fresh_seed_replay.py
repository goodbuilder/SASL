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
NED_TO_RDF_CONTRACT = "rdf_right=east, rdf_down=down, rdf_forward=north"
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


def sample_payload(sample: pipe.CalibrationSample) -> dict[str, float]:
    raw = sample.raw_rdf_m
    return {
        "right_m": float(raw[0]),
        "down_m": float(raw[1]),
        "forward_m": float(raw[2]),
        "range_m": float(np.linalg.norm(raw)),
        "disparity_px": float(sample.measured_disparity_px),
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
) -> tuple[dict[str, object], list[dict[str, object]]]:
    _, validation = build_seed_samples(
        seed=seed,
        scenario=scenario,
        sample_count=sample_count,
        train_ratio=train_ratio,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
        target_oversample_count=target_oversample_count,
    )
    corrected_points: list[np.ndarray] = []
    raw_points: list[np.ndarray] = []
    prediction_rows: list[dict[str, object]] = []
    invalid_count = 0
    fallback_count = 0
    for sample in validation:
        raw = sample.raw_rdf_m.astype(float)
        result = profile.apply(sample_payload(sample))
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
        corrected_points.append(corrected)
        truth = sample.truth_rdf_m.astype(float)
        prediction_rows.append(
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
                "measured_disparity_px": sample.measured_disparity_px,
                "profile_fallback_level": fallback_level,
                "profile_fallback_reason": result.status.get("fallback_reason", ""),
                "rdf_axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    if corrected_points:
        corrected_arr = np.vstack(corrected_points)
        raw_arr = np.vstack(raw_points)
        evaluated_samples = [sample for sample in validation if sample.split == "validation"][: corrected_arr.shape[0]]
    else:
        corrected_arr = np.empty((0, 3), dtype=float)
        raw_arr = np.empty((0, 3), dtype=float)
        evaluated_samples = []
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
    }
    return row, prediction_rows


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
        "ned_to_rdf_contract": NED_TO_RDF_CONTRACT,
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
