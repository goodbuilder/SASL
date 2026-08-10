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
import run_runtime_correlated_repeated_observation_sweep as correlated  # noqa: E402
import run_runtime_image_prefilter_sweep as image_prefilter  # noqa: E402
import run_runtime_multiburst_observation_fusion_sweep as multiburst  # noqa: E402
import run_runtime_repeated_observation_fusion_sweep as repeated  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "small-drone observation fusion consumes only runtime image left_u_px, left_v_px, "
    "measured_disparity_px, fixed camera intrinsics, and fixed stereo baseline; "
    "PX4/RTK/NED truth is synthetic validation only"
)

SCENARIO_PRESETS: dict[str, dict[str, float]] = {
    "small_drone_close_2m_4x": {
        "range_min_m": 20.0,
        "range_max_m": 120.0,
        "target_range_m": 50.0,
        "target_window_m": 8.0,
        "target_error_limit_m": 1.5,
        "baseline_m": 2.0,
        "horizontal_fov_rad": 0.308155618,
        "right_amplitude_m": 3.0,
        "down_center_m": 1.8,
        "down_amplitude_m": 1.4,
        "bbox_noise_px": 0.24,
        "bbox_outlier_px": 2.8,
        "bbox_outlier_rate": 0.08,
        "disparity_noise_px": 0.10,
        "disparity_outlier_px": 0.38,
        "disparity_outlier_rate": 0.06,
    },
    "small_drone_mid_5m_8x": {
        "range_min_m": 80.0,
        "range_max_m": 350.0,
        "target_range_m": 180.0,
        "target_window_m": 20.0,
        "target_error_limit_m": 3.0,
        "baseline_m": 5.0,
        "horizontal_fov_rad": 0.154997710698,
        "right_amplitude_m": 8.0,
        "down_center_m": 3.5,
        "down_amplitude_m": 3.0,
        "bbox_noise_px": 0.22,
        "bbox_outlier_px": 2.5,
        "bbox_outlier_rate": 0.08,
        "disparity_noise_px": 0.09,
        "disparity_outlier_px": 0.36,
        "disparity_outlier_rate": 0.06,
    },
}


def scenario_camera_and_injection(name: str) -> tuple[pipe.CameraModel, pipe.ErrorInjection, Mapping[str, float]]:
    preset = SCENARIO_PRESETS.get(name)
    if preset is None:
        raise ValueError(f"unknown small-drone scenario: {name}")
    camera = pipe.CameraModel(
        baseline_m=float(preset["baseline_m"]),
        horizontal_fov_rad=float(preset["horizontal_fov_rad"]),
        baseline_scale_error=0.006,
        focal_scale_error=-0.004,
    )
    injection = pipe.ErrorInjection(
        yaw_deg=0.20,
        pitch_deg=-0.10,
        roll_deg=0.05,
        disparity_bias_px=0.06,
        disparity_noise_px=float(preset["disparity_noise_px"]),
        bbox_noise_px=float(preset["bbox_noise_px"]),
        disparity_outlier_rate=float(preset["disparity_outlier_rate"]),
        disparity_outlier_px=float(preset["disparity_outlier_px"]),
        bbox_outlier_rate=float(preset["bbox_outlier_rate"]),
        bbox_outlier_px=float(preset["bbox_outlier_px"]),
        down_bias_px=0.10,
    )
    return camera, injection, preset


def synthesize_small_drone_samples(
    *,
    scenario: str,
    seed: int,
    sample_count: int,
    target_oversample_count: int,
) -> list[pipe.CalibrationSample]:
    camera, injection, preset = scenario_camera_and_injection(scenario)
    trajectory_rng = np.random.default_rng(seed)
    measurement_rng = np.random.default_rng(seed + 1000003)
    range_min = float(preset["range_min_m"])
    range_max = float(preset["range_max_m"])
    target_range_m = float(preset["target_range_m"])
    target_window_m = float(preset["target_window_m"])
    base_forward = np.linspace(range_min, range_max, int(sample_count), dtype=float)
    if target_oversample_count > 0 and sample_count > 0:
        target_count = min(int(sample_count), int(target_oversample_count))
        center_index = int(round((target_range_m - range_min) / max(1.0e-9, range_max - range_min) * (sample_count - 1)))
        half_count = target_count // 2
        start = min(max(0, center_index - half_count), max(0, sample_count - target_count))
        target_indices = np.arange(start, start + target_count, dtype=int)
        span = min(max(1.0, target_window_m * 0.75), max(1.0, target_window_m - 0.5))
        offsets = np.linspace(-span, span, target_count) if target_count > 1 else np.array([0.0])
        jitter = trajectory_rng.normal(0.0, max(0.03, target_window_m * 0.06), size=target_count)
        base_forward[target_indices] = target_range_m + np.clip(offsets + jitter, -target_window_m * 0.95, target_window_m * 0.95)

    samples: list[pipe.CalibrationSample] = []
    for index, forward_m in enumerate(base_forward):
        phase = index / max(1, int(sample_count) - 1)
        right_m = float(preset["right_amplitude_m"]) * math.sin(phase * math.tau * 2.1) + trajectory_rng.normal(0.0, 0.35)
        down_m = (
            float(preset["down_center_m"])
            + float(preset["down_amplitude_m"]) * math.sin(phase * math.tau * 1.4 + 0.3)
            + trajectory_rng.normal(0.0, 0.18)
        )
        forward_m = float(forward_m + trajectory_rng.normal(0.0, max(0.15, target_window_m * 0.05)))
        truth_rdf = np.array([right_m, down_m, max(1.0, forward_m)], dtype=float)
        truth_ned = np.array([truth_rdf[2], truth_rdf[0], truth_rdf[1]], dtype=float)

        left_u, left_v, measured_disparity = repeated.synthesize_observation(
            truth_rdf_m=truth_rdf,
            camera=camera,
            injection=injection,
            rng=measurement_rng,
        )
        raw_rdf = image_prefilter.reconstruct_raw_rdf(
            camera=camera,
            injection=injection,
            left_u_px=left_u,
            left_v_px=left_v,
            disparity_px=measured_disparity,
        )
        true_disparity = camera.fx_true_px * camera.baseline_m / truth_rdf[2]
        samples.append(
            pipe.CalibrationSample(
                sample_id=index,
                t_s=index / 30.0,
                split="",
                truth_ned_m=truth_ned,
                truth_rdf_m=truth_rdf,
                raw_rdf_m=raw_rdf,
                left_u_px=left_u,
                left_v_px=left_v,
                measured_disparity_px=measured_disparity,
                true_disparity_px=float(true_disparity),
            )
        )
    return samples


def fused_rdf_for_sample(
    *,
    sample: pipe.CalibrationSample,
    camera: pipe.CameraModel,
    injection: pipe.ErrorInjection,
    rng: np.random.Generator,
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
) -> np.ndarray:
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
    return image_prefilter.reconstruct_raw_rdf(
        camera=camera,
        injection=injection,
        left_u_px=left_u,
        left_v_px=left_v,
        disparity_px=disparity,
    )


def method_name(
    *,
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
) -> str:
    return "small_drone_" + multiburst.method_name(
        burst_count=burst_count,
        observations_per_burst=observations_per_burst,
        inner_mode=inner_mode,
        outer_mode=outer_mode,
        trim_fraction=trim_fraction,
        common_fraction=common_fraction,
    )


def evaluate_seed(
    *,
    seed: int,
    scenario: str,
    sample_count: int,
    train_ratio: float,
    target_oversample_count: int,
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
    correction_mode: str,
) -> dict[str, object]:
    camera, _injection, preset = scenario_camera_and_injection(scenario)
    samples = synthesize_small_drone_samples(
        scenario=scenario,
        seed=seed,
        sample_count=sample_count,
        target_oversample_count=target_oversample_count,
    )
    train, validation = pipe.split_samples(
        samples,
        train_ratio=train_ratio,
        seed=seed + 17,
        target_range_m=float(preset["target_range_m"]),
        target_window_m=float(preset["target_window_m"]),
    )
    rng = np.random.default_rng(
        int(seed)
        + 72000029
        + int(burst_count) * 1009
        + int(observations_per_burst) * 917
        + int(round(common_fraction * 1000.0))
    )

    def fused_points_for(sample_rows: Sequence[pipe.CalibrationSample]) -> np.ndarray:
        rows: list[np.ndarray] = []
        for sample in sample_rows:
            rows.append(
                fused_rdf_for_sample(
                    sample=sample,
                    camera=camera,
                    injection=_injection,
                    rng=rng,
                    burst_count=burst_count,
                    observations_per_burst=observations_per_burst,
                    inner_mode=inner_mode,
                    outer_mode=outer_mode,
                    trim_fraction=trim_fraction,
                    common_fraction=common_fraction,
                )
            )
        return np.vstack(rows) if rows else np.empty((0, 3), dtype=float)

    fused_train_arr = fused_points_for(train)
    fused_arr = fused_points_for(validation)
    raw_arr = pipe.stack(validation, "raw_rdf_m")
    train_raw_arr = pipe.stack(train, "raw_rdf_m")
    if correction_mode == "fusion_only":
        corrected_train_arr = fused_train_arr
        corrected_arr = fused_arr
    elif correction_mode == "sim3":
        scale, rotation, translation = pipe.fit_umeyama_sim3(fused_train_arr, pipe.stack(train, "truth_rdf_m"))
        corrected_train_arr = pipe.predict_sim3(fused_train_arr, scale, rotation, translation)
        corrected_arr = pipe.predict_sim3(fused_arr, scale, rotation, translation)
    elif correction_mode == "affine_ridge":
        matrix, translation = pipe.fit_affine_rdf(fused_train_arr, pipe.stack(train, "truth_rdf_m"), ridge_lambda=1.0e-3)
        corrected_train_arr = fused_train_arr @ matrix.T + translation
        corrected_arr = fused_arr @ matrix.T + translation
    else:
        raise ValueError(f"unsupported correction mode: {correction_mode}")

    raw_metrics = pipe.evaluate_predictions(
        validation,
        raw_arr,
        target_range_m=float(preset["target_range_m"]),
        target_window_m=float(preset["target_window_m"]),
    )
    corrected_metrics = pipe.evaluate_predictions(
        validation,
        corrected_arr,
        target_range_m=float(preset["target_range_m"]),
        target_window_m=float(preset["target_window_m"]),
    )
    train_raw_metrics = pipe.evaluate_predictions(
        train,
        train_raw_arr,
        target_range_m=float(preset["target_range_m"]),
        target_window_m=float(preset["target_window_m"]),
    )
    train_corrected_metrics = pipe.evaluate_predictions(
        train,
        corrected_train_arr,
        target_range_m=float(preset["target_range_m"]),
        target_window_m=float(preset["target_window_m"]),
    )
    axis_pass = (
        corrected_metrics["right_rmse_m"] < raw_metrics["right_rmse_m"]
        and corrected_metrics["down_rmse_m"] < raw_metrics["down_rmse_m"]
        and corrected_metrics["forward_rmse_m"] < raw_metrics["forward_rmse_m"]
    )
    passed = (
        corrected_metrics["range_rmse_m"] < raw_metrics["range_rmse_m"]
        and corrected_metrics["position_rmse_m"] < raw_metrics["position_rmse_m"]
        and axis_pass
        and corrected_metrics["target_window_count"] > 0
        and corrected_metrics["target_max_abs_error_m"] <= float(preset["target_error_limit_m"])
    )
    full_method_name = (
        method_name(
            burst_count=burst_count,
            observations_per_burst=observations_per_burst,
            inner_mode=inner_mode,
            outer_mode=outer_mode,
            trim_fraction=trim_fraction,
            common_fraction=common_fraction,
        )
        + f"_{correction_mode}"
    )
    return {
        "seed": int(seed),
        "scenario": scenario,
        "method": full_method_name,
        "status": "PASS" if passed else "FAIL",
        "correction_mode": correction_mode,
        "burst_count": int(burst_count),
        "observations_per_burst": int(observations_per_burst),
        "total_observation_count": int(burst_count) * int(observations_per_burst),
        "inner_mode": inner_mode,
        "outer_mode": outer_mode,
        "trim_fraction": float(trim_fraction),
        "common_fraction": float(common_fraction),
        "validation_sample_count": len(validation),
        "train_range_rmse_raw_m": train_raw_metrics["range_rmse_m"],
        "train_range_rmse_corrected_m": train_corrected_metrics["range_rmse_m"],
        "train_position_rmse_raw_m": train_raw_metrics["position_rmse_m"],
        "train_position_rmse_corrected_m": train_corrected_metrics["position_rmse_m"],
        "range_rmse_raw_m": raw_metrics["range_rmse_m"],
        "range_rmse_corrected_m": corrected_metrics["range_rmse_m"],
        "position_rmse_raw_m": raw_metrics["position_rmse_m"],
        "position_rmse_corrected_m": corrected_metrics["position_rmse_m"],
        "right_rmse_raw_m": raw_metrics["right_rmse_m"],
        "right_rmse_corrected_m": corrected_metrics["right_rmse_m"],
        "down_rmse_raw_m": raw_metrics["down_rmse_m"],
        "down_rmse_corrected_m": corrected_metrics["down_rmse_m"],
        "forward_rmse_raw_m": raw_metrics["forward_rmse_m"],
        "forward_rmse_corrected_m": corrected_metrics["forward_rmse_m"],
        "target_count": corrected_metrics["target_window_count"],
        "target_max_abs_error_corrected_m": corrected_metrics["target_max_abs_error_m"],
        "target_error_limit_m": float(preset["target_error_limit_m"]),
        "target_rmse_corrected_m": corrected_metrics["target_rmse_m"],
        "runtime_truth_used": False,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "image_contract": image_prefilter.IMAGE_CONTRACT,
        "rdf_axis_contract": image_prefilter.RDF_AXIS_CONTRACT,
        "public_topic_contract": image_prefilter.PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": image_prefilter.TRUTH_RDF_TRANSFORM_CONTRACT,
        "ned_to_rdf_contract": image_prefilter.NED_TO_RDF_CONTRACT,
    }


def aggregate(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str], list[Mapping[str, object]]] = {}
    for row in rows:
        grouped.setdefault((str(row["scenario"]), str(row["method"])), []).append(row)
    summaries: list[dict[str, object]] = []
    for (scenario, method), method_rows in grouped.items():
        values = lambda name: np.asarray([float(row[name]) for row in method_rows], dtype=float)
        pass_count = sum(1 for row in method_rows if row.get("status") == "PASS")
        first = method_rows[0]
        summaries.append(
            {
                "scenario": scenario,
                "method": method,
                "status": "STRICT_PASS" if pass_count == len(method_rows) else "FAIL",
                "pass_count": pass_count,
                "trial_count": len(method_rows),
                "burst_count": first["burst_count"],
                "observations_per_burst": first["observations_per_burst"],
                "total_observation_count": first["total_observation_count"],
                "correction_mode": first["correction_mode"],
                "inner_mode": first["inner_mode"],
                "outer_mode": first["outer_mode"],
                "trim_fraction": first["trim_fraction"],
                "common_fraction": first["common_fraction"],
                "range_rmse_raw_mean_m": float(np.mean(values("range_rmse_raw_m"))),
                "range_rmse_corrected_mean_m": float(np.mean(values("range_rmse_corrected_m"))),
                "range_rmse_corrected_max_m": float(np.max(values("range_rmse_corrected_m"))),
                "position_rmse_raw_mean_m": float(np.mean(values("position_rmse_raw_m"))),
                "position_rmse_corrected_mean_m": float(np.mean(values("position_rmse_corrected_m"))),
                "position_rmse_corrected_max_m": float(np.max(values("position_rmse_corrected_m"))),
                "right_rmse_raw_mean_m": float(np.mean(values("right_rmse_raw_m"))),
                "right_rmse_corrected_mean_m": float(np.mean(values("right_rmse_corrected_m"))),
                "down_rmse_raw_mean_m": float(np.mean(values("down_rmse_raw_m"))),
                "down_rmse_corrected_mean_m": float(np.mean(values("down_rmse_corrected_m"))),
                "forward_rmse_raw_mean_m": float(np.mean(values("forward_rmse_raw_m"))),
                "forward_rmse_corrected_mean_m": float(np.mean(values("forward_rmse_corrected_m"))),
                "target_max_abs_error_corrected_max_m": float(np.max(values("target_max_abs_error_corrected_m"))),
                "target_error_limit_m": first["target_error_limit_m"],
                "target_rmse_corrected_mean_m": float(np.mean(values("target_rmse_corrected_m"))),
                "runtime_truth_used": False,
            }
        )
    summaries.sort(
        key=lambda row: (
            str(row["scenario"]),
            -int(row["pass_count"]),
            int(row["total_observation_count"]),
            float(row["target_max_abs_error_corrected_max_m"]),
            float(row["range_rmse_corrected_mean_m"]),
        )
    )
    return summaries


def parse_scenarios(value: str) -> list[str]:
    names = [item.strip() for item in value.split(",") if item.strip()]
    unknown = [name for name in names if name not in SCENARIO_PRESETS]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown scenarios: {', '.join(unknown)}")
    return names


def parse_correction_modes(value: str) -> list[str]:
    modes = [item.strip() for item in value.split(",") if item.strip()]
    allowed = {"fusion_only", "sim3", "affine_ridge"}
    unknown = [mode for mode in modes if mode not in allowed]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown correction modes: {', '.join(unknown)}")
    if not modes:
        raise argparse.ArgumentTypeError("correction mode list must not be empty")
    return modes


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
    parser = argparse.ArgumentParser(description="Sweep runtime-safe observation fusion for small-drone stereo scenarios.")
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--scenarios",
        type=parse_scenarios,
        default=list(SCENARIO_PRESETS),
        help=f"Comma-separated scenario list. Known: {', '.join(SCENARIO_PRESETS)}",
    )
    parser.add_argument("--seed-start", type=int, default=20298613)
    parser.add_argument("--trial-count", type=int, default=50)
    parser.add_argument("--sample-count", type=int, default=720)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument("--target-oversample-count", type=int, default=48)
    parser.add_argument("--burst-specs", type=multiburst.parse_burst_specs, default=multiburst.parse_burst_specs("1x3,1x5,3x3,3x5,5x3,5x5"))
    parser.add_argument("--inner-modes", type=repeated.parse_modes, default=repeated.parse_modes("median,trimmed_mean"))
    parser.add_argument("--outer-modes", type=repeated.parse_modes, default=repeated.parse_modes("median"))
    parser.add_argument("--correction-modes", type=parse_correction_modes, default=parse_correction_modes("fusion_only,sim3,affine_ridge"))
    parser.add_argument("--trim-fraction", type=float, default=0.2)
    parser.add_argument("--common-fractions", type=correlated.parse_float_list, default=correlated.parse_float_list("0,0.5,1"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    for scenario_index, scenario in enumerate(args.scenarios):
        for common_fraction in args.common_fractions:
            for burst_count, observations_per_burst in args.burst_specs:
                for inner_mode in args.inner_modes:
                    for outer_mode in args.outer_modes:
                        for correction_mode in args.correction_modes:
                            for trial_index in range(int(args.trial_count)):
                                rows.append(
                                    evaluate_seed(
                                        seed=int(args.seed_start) + scenario_index * int(args.trial_count) + trial_index,
                                        scenario=str(scenario),
                                        sample_count=int(args.sample_count),
                                        train_ratio=float(args.train_ratio),
                                        target_oversample_count=int(args.target_oversample_count),
                                        burst_count=int(burst_count),
                                        observations_per_burst=int(observations_per_burst),
                                        inner_mode=str(inner_mode),
                                        outer_mode=str(outer_mode),
                                        trim_fraction=float(args.trim_fraction),
                                        common_fraction=float(common_fraction),
                                        correction_mode=str(correction_mode),
                                    )
                                )
    summaries = aggregate(rows)
    best_by_scenario = {}
    for scenario in args.scenarios:
        scenario_rows = [row for row in summaries if row["scenario"] == scenario]
        best_by_scenario[scenario] = scenario_rows[0] if scenario_rows else {}
    summary = {
        "experiment_id": args.experiment_id,
        "scenarios": args.scenarios,
        "seed_start": args.seed_start,
        "trial_count_per_scenario": args.trial_count,
        "sample_count": args.sample_count,
        "target_oversample_count": args.target_oversample_count,
        "candidate_count": len(summaries),
        "best_by_scenario": best_by_scenario,
        "scenario_presets": SCENARIO_PRESETS,
        "coordinate_contracts_explicit": True,
        "image_contract": image_prefilter.IMAGE_CONTRACT,
        "rdf_axis_contract": image_prefilter.RDF_AXIS_CONTRACT,
        "public_topic_contract": image_prefilter.PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": image_prefilter.TRUTH_RDF_TRANSFORM_CONTRACT,
        "ned_to_rdf_contract": image_prefilter.NED_TO_RDF_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }
    write_csv(args.output_dir / "small_drone_observation_seed_metrics.csv", rows)
    write_csv(args.output_dir / "small_drone_observation_method_summary.csv", summaries)
    write_json(args.output_dir / "small_drone_observation_metrics.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: test runtime-safe observation fusion for small-drone range and camera geometry.",
                "",
                "Reference policy: R1 internal precedent from EXP-201/202/210/212 observation-fusion results; R2 stereo triangulation and robust median/trimmed mean statistics; R3 local producer/schema contracts in rtk_stereo_calibration_pipeline and StereoDistance topic contract helpers.",
                "",
                "Coordinate contract: image u/v/disparity observations are fused in pixel space, re-triangulated into stereo-center RDF right/down/forward, and compared with truth_rdf produced by an explicit T_source_to_stereo_rdf. Direct NED-to-RDF axis aliasing is forbidden, including in synthetic runs. Public StereoDistance position topics remain FRD+range. Truth is validation-only and is not a runtime input.",
                "",
                f"Best by scenario: `{json.dumps(pipe.json_safe(best_by_scenario), ensure_ascii=False, sort_keys=True)}`",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(pipe.json_safe(summary), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
