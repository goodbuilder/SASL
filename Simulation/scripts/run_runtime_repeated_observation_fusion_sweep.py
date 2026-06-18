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
import rtk_stereo_calibration_pipeline as pipe  # noqa: E402
import run_runtime_image_prefilter_sweep as image_prefilter  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "repeated-observation fusion consumes multiple independent left_u_px, left_v_px, "
    "and measured_disparity_px observations for the same timestamp plus camera intrinsics/baseline; "
    "PX4/RTK truth is synthetic generation and audit-only"
)


def synthesize_observation(
    *,
    truth_rdf_m: np.ndarray,
    camera: pipe.CameraModel,
    injection: pipe.ErrorInjection,
    rng: np.random.Generator,
) -> tuple[float, float, float]:
    true_disparity = camera.fx_true_px * camera.baseline_m / float(truth_rdf_m[2])
    left_u_true = camera.cx_px + (float(truth_rdf_m[0]) + camera.baseline_m * 0.5) * camera.fx_true_px / float(truth_rdf_m[2])
    left_v_true = camera.cy_px + float(truth_rdf_m[1]) * camera.fy_true_px / float(truth_rdf_m[2])
    measured_disparity = (
        true_disparity
        + injection.disparity_bias_px
        + 0.00018 * (float(truth_rdf_m[2]) - 1100.0)
        + rng.normal(0.0, injection.disparity_noise_px)
    )
    if rng.random() < max(0.0, injection.disparity_outlier_rate):
        measured_disparity += rng.normal(0.0, max(0.0, injection.disparity_outlier_px))
    measured_disparity = max(float(measured_disparity), 1.0e-6)
    left_u_meas = left_u_true + rng.normal(0.0, injection.bbox_noise_px)
    left_v_meas = left_v_true + injection.down_bias_px + rng.normal(0.0, injection.bbox_noise_px)
    if rng.random() < max(0.0, injection.bbox_outlier_rate):
        left_u_meas += rng.normal(0.0, max(0.0, injection.bbox_outlier_px))
        left_v_meas += rng.normal(0.0, max(0.0, injection.bbox_outlier_px))
    return float(left_u_meas), float(left_v_meas), float(measured_disparity)


def fuse_values(values: Sequence[float], *, mode: str, trim_fraction: float = 0.2) -> float:
    arr = np.sort(np.asarray(values, dtype=float))
    if arr.size == 0:
        return float("nan")
    if mode == "median":
        return float(np.median(arr))
    if mode == "mean":
        return float(np.mean(arr))
    if mode == "trimmed_mean":
        trim = int(math.floor(float(trim_fraction) * arr.size))
        if trim * 2 >= arr.size:
            return float(np.median(arr))
        return float(np.mean(arr[trim : arr.size - trim]))
    raise ValueError(f"unsupported fusion mode: {mode}")


def method_name(*, repeat_count: int, mode: str, trim_fraction: float) -> str:
    trim_label = f"{trim_fraction:g}".replace(".", "p")
    return f"repeated_observation_{mode}_n{int(repeat_count)}_trim{trim_label}"


def build_repeated_corrected_points(
    *,
    samples: Sequence[pipe.CalibrationSample],
    profile: replay_audit.RuntimeProfile,
    camera: pipe.CameraModel,
    injection: pipe.ErrorInjection,
    seed: int,
    repeat_count: int,
    mode: str,
    trim_fraction: float,
) -> np.ndarray:
    rng = np.random.default_rng(int(seed) + 42000019 + int(repeat_count) * 1009)
    corrected_points: list[np.ndarray] = []
    for sample in samples:
        observations = [
            synthesize_observation(
                truth_rdf_m=sample.truth_rdf_m,
                camera=camera,
                injection=injection,
                rng=rng,
            )
            for _ in range(int(repeat_count))
        ]
        left_u = fuse_values([obs[0] for obs in observations], mode=mode, trim_fraction=trim_fraction)
        left_v = fuse_values([obs[1] for obs in observations], mode=mode, trim_fraction=trim_fraction)
        disparity = fuse_values([obs[2] for obs in observations], mode=mode, trim_fraction=trim_fraction)
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
    return np.vstack(corrected_points)


def evaluate_seed(
    *,
    profile: replay_audit.RuntimeProfile,
    seed: int,
    scenario: str,
    sample_count: int,
    train_ratio: float,
    target_range_m: float,
    target_window_m: float,
    target_oversample_count: int,
    repeat_count: int,
    mode: str,
    trim_fraction: float,
) -> dict[str, object]:
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
    validation_ids = {sample.sample_id for sample in validation}
    corrected_all = build_repeated_corrected_points(
        samples=samples,
        profile=profile,
        camera=camera,
        injection=injection,
        seed=seed,
        repeat_count=repeat_count,
        mode=mode,
        trim_fraction=trim_fraction,
    )
    validation_samples: list[pipe.CalibrationSample] = []
    corrected_validation: list[np.ndarray] = []
    raw_validation: list[np.ndarray] = []
    for sample, corrected in zip(samples, corrected_all):
        if sample.sample_id in validation_ids:
            validation_samples.append(sample)
            corrected_validation.append(corrected)
            raw_validation.append(sample.raw_rdf_m)
    corrected_arr = np.vstack(corrected_validation) if corrected_validation else np.empty((0, 3), dtype=float)
    raw_arr = np.vstack(raw_validation) if raw_validation else np.empty((0, 3), dtype=float)
    raw_metrics = pipe.evaluate_predictions(
        validation_samples,
        raw_arr,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    corrected_metrics = pipe.evaluate_predictions(
        validation_samples,
        corrected_arr,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    passed = (
        corrected_metrics["range_rmse_m"] < raw_metrics["range_rmse_m"]
        and corrected_metrics["position_rmse_m"] < raw_metrics["position_rmse_m"]
        and corrected_metrics["target_window_count"] > 0
        and corrected_metrics["target_max_abs_error_m"] <= 10.0
    )
    return {
        "seed": int(seed),
        "scenario": scenario,
        "method": method_name(repeat_count=repeat_count, mode=mode, trim_fraction=trim_fraction),
        "status": "PASS" if passed else "FAIL",
        "repeat_count": int(repeat_count),
        "fusion_mode": mode,
        "trim_fraction": float(trim_fraction),
        "validation_sample_count": len(validation_samples),
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
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "image_contract": image_prefilter.IMAGE_CONTRACT,
        "rdf_axis_contract": image_prefilter.RDF_AXIS_CONTRACT,
        "public_topic_contract": image_prefilter.PUBLIC_TOPIC_CONTRACT,
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
                "repeat_count": first["repeat_count"],
                "fusion_mode": first["fusion_mode"],
                "trim_fraction": first["trim_fraction"],
                "range_rmse_raw_mean_m": float(np.mean(values("range_rmse_raw_m"))),
                "range_rmse_corrected_mean_m": float(np.mean(values("range_rmse_corrected_m"))),
                "range_rmse_corrected_max_m": float(np.max(values("range_rmse_corrected_m"))),
                "position_rmse_raw_mean_m": float(np.mean(values("position_rmse_raw_m"))),
                "position_rmse_corrected_mean_m": float(np.mean(values("position_rmse_corrected_m"))),
                "position_rmse_corrected_max_m": float(np.max(values("position_rmse_corrected_m"))),
                "target_max_abs_error_corrected_max_m": float(np.max(values("target_max_abs_error_corrected_m"))),
                "target_rmse_corrected_mean_m": float(np.mean(values("target_rmse_corrected_m"))),
                "runtime_truth_used": False,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "image_contract": image_prefilter.IMAGE_CONTRACT,
                "rdf_axis_contract": image_prefilter.RDF_AXIS_CONTRACT,
                "public_topic_contract": image_prefilter.PUBLIC_TOPIC_CONTRACT,
            }
        )
    summaries.sort(
        key=lambda row: (
            -int(row["pass_count"]),
            float(row["target_max_abs_error_corrected_max_m"]),
            float(row["range_rmse_corrected_mean_m"]),
        )
    )
    return summaries


def parse_int_list(value: str) -> list[int]:
    parsed = [int(item.strip()) for item in value.split(",") if item.strip()]
    if not parsed:
        raise argparse.ArgumentTypeError("list must not be empty")
    return parsed


def parse_modes(value: str) -> list[str]:
    modes = [item.strip() for item in value.split(",") if item.strip()]
    unknown = [mode for mode in modes if mode not in {"median", "mean", "trimmed_mean"}]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown fusion modes: {', '.join(unknown)}")
    return modes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep repeated independent image observation fusion before StereoDistance runtime profile correction."
    )
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scenario", default="bbox_outlier_stress", choices=tuple(replay_audit.SCENARIO_PRESETS))
    parser.add_argument("--seed-start", type=int, default=20297659)
    parser.add_argument("--trial-count", type=int, default=50)
    parser.add_argument("--sample-count", type=int, default=1680)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-oversample-count", type=int, default=72)
    parser.add_argument("--repeat-counts", type=parse_int_list, default=parse_int_list("1,3,5,7"))
    parser.add_argument("--fusion-modes", type=parse_modes, default=parse_modes("median,trimmed_mean"))
    parser.add_argument("--trim-fraction", type=float, default=0.2)
    return parser.parse_args()


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


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile = replay_audit.load_profile(args.profile_json)
    rows: list[dict[str, object]] = []
    for repeat_count in args.repeat_counts:
        for mode in args.fusion_modes:
            for trial_index in range(int(args.trial_count)):
                rows.append(
                    evaluate_seed(
                        profile=profile,
                        seed=int(args.seed_start) + trial_index,
                        scenario=str(args.scenario),
                        sample_count=int(args.sample_count),
                        train_ratio=float(args.train_ratio),
                        target_range_m=float(args.target_range_m),
                        target_window_m=float(args.target_window_m),
                        target_oversample_count=int(args.target_oversample_count),
                        repeat_count=int(repeat_count),
                        mode=str(mode),
                        trim_fraction=float(args.trim_fraction),
                    )
                )
    summaries = aggregate(rows)
    write_csv(args.output_dir / "repeated_observation_seed_metrics.csv", rows)
    write_csv(args.output_dir / "repeated_observation_method_summary.csv", summaries)
    summary = {
        "experiment_id": args.experiment_id,
        "profile_json": str(args.profile_json),
        "scenario": args.scenario,
        "seed_start": args.seed_start,
        "seed_end": int(args.seed_start) + int(args.trial_count) - 1,
        "trial_count": args.trial_count,
        "best_method": summaries[0] if summaries else {},
        "coordinate_contracts_explicit": True,
        "image_contract": image_prefilter.IMAGE_CONTRACT,
        "rdf_axis_contract": image_prefilter.RDF_AXIS_CONTRACT,
        "public_topic_contract": image_prefilter.PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": image_prefilter.TRUTH_RDF_TRANSFORM_CONTRACT,
        "ned_to_rdf_contract": image_prefilter.NED_TO_RDF_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }
    write_json(args.output_dir / "repeated_observation_metrics.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: test repeated independent image observations before StereoDistance profile correction.",
                "",
                "Coordinate contract: repeated left image u/v and disparity observations are fused in pixel space, re-triangulated into stereo-center RDF right/down/forward, then corrected by the runtime profile. PX4 local NED truth is used only to synthesize/audit samples.",
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
