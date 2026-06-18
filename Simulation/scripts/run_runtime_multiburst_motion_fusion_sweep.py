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
import run_runtime_motion_fusion_sweep as motion  # noqa: E402
import run_runtime_multiburst_observation_fusion_sweep as multiburst  # noqa: E402
import run_runtime_repeated_observation_fusion_sweep as repeated  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "runtime-safe combination: multi-burst observation fusion consumes image left_u_px/left_v_px/"
    "measured_disparity_px, then causal alpha-beta motion fusion consumes the corrected "
    "stereo-center RDF stream; PX4/RTK truth is audit-only"
)


def method_name(
    *,
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
    alpha: float,
    beta: float,
    innovation_clip_m: float,
    axes: str,
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
        f"{base}_motion_{axes}_alpha_beta"
        f"_a{label(alpha)}_b{label(beta)}_g{label(innovation_clip_m)}"
    )


def build_multiburst_series(
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
) -> motion.CachedSeedSeries:
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
    validation_sorted = [sample for sample in samples if sample.sample_id in validation_ids]
    corrected_points = multiburst.build_multiburst_corrected_points(
        samples=samples,
        profile=profile,
        camera=camera,
        injection=injection,
        seed=seed,
        burst_count=burst_count,
        observations_per_burst=observations_per_burst,
        inner_mode=inner_mode,
        outer_mode=outer_mode,
        trim_fraction=trim_fraction,
        common_fraction=common_fraction,
    )
    return motion.CachedSeedSeries(
        seed=seed,
        scenario=scenario,
        samples=samples,
        validation_ids=validation_ids,
        raw_validation_points=pipe.stack(validation_sorted, "raw_rdf_m"),
        corrected_points=corrected_points,
    )


def evaluate_combined(
    series: motion.CachedSeedSeries,
    *,
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
    alpha: float,
    beta: float,
    innovation_clip_m: float,
    axes: str,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    fused = motion.alpha_beta_innovation_clip(
        samples=series.samples,
        measurements_rdf_m=series.corrected_points,
        alpha=alpha,
        beta=beta,
        innovation_clip_m=innovation_clip_m,
        axes=axes,
    )
    validation_samples: list[pipe.CalibrationSample] = []
    observation_points: list[np.ndarray] = []
    motion_points: list[np.ndarray] = []
    for sample, observation_point, motion_point in zip(series.samples, series.corrected_points, fused):
        if sample.sample_id in series.validation_ids:
            validation_samples.append(sample)
            observation_points.append(observation_point)
            motion_points.append(motion_point)
    observation_validation = (
        np.vstack(observation_points) if observation_points else np.empty((0, 3), dtype=float)
    )
    motion_validation = np.vstack(motion_points) if motion_points else np.empty((0, 3), dtype=float)
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
    motion_metrics = pipe.evaluate_predictions(
        validation_samples,
        motion_validation,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    passed = (
        motion_metrics["range_rmse_m"] < raw_metrics["range_rmse_m"]
        and motion_metrics["position_rmse_m"] < raw_metrics["position_rmse_m"]
        and motion_metrics["target_window_count"] > 0
        and motion_metrics["target_max_abs_error_m"] <= 10.0
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
            alpha=alpha,
            beta=beta,
            innovation_clip_m=innovation_clip_m,
            axes=axes,
        ),
        "status": "PASS" if passed else "FAIL",
        "burst_count": int(burst_count),
        "observations_per_burst": int(observations_per_burst),
        "total_observation_count": int(burst_count) * int(observations_per_burst),
        "inner_mode": inner_mode,
        "outer_mode": outer_mode,
        "trim_fraction": float(trim_fraction),
        "common_fraction": float(common_fraction),
        "alpha": float(alpha),
        "beta": float(beta),
        "innovation_clip_m": float(innovation_clip_m),
        "fusion_axes": axes,
        "validation_sample_count": len(validation_samples),
        "range_rmse_raw_m": raw_metrics["range_rmse_m"],
        "range_rmse_observation_m": observation_metrics["range_rmse_m"],
        "range_rmse_motion_m": motion_metrics["range_rmse_m"],
        "position_rmse_raw_m": raw_metrics["position_rmse_m"],
        "position_rmse_observation_m": observation_metrics["position_rmse_m"],
        "position_rmse_motion_m": motion_metrics["position_rmse_m"],
        "right_rmse_motion_m": motion_metrics["right_rmse_m"],
        "down_rmse_motion_m": motion_metrics["down_rmse_m"],
        "forward_rmse_motion_m": motion_metrics["forward_rmse_m"],
        "target_count": motion_metrics["target_window_count"],
        "target_max_abs_error_observation_m": observation_metrics["target_max_abs_error_m"],
        "target_max_abs_error_motion_m": motion_metrics["target_max_abs_error_m"],
        "target_rmse_observation_m": observation_metrics["target_rmse_m"],
        "target_rmse_motion_m": motion_metrics["target_rmse_m"],
        "runtime_truth_used": False,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "image_contract": multiburst.image_prefilter.IMAGE_CONTRACT,
        "rdf_axis_contract": motion.RDF_AXIS_CONTRACT,
        "public_topic_contract": motion.PUBLIC_TOPIC_CONTRACT,
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
                "alpha": first["alpha"],
                "beta": first["beta"],
                "innovation_clip_m": first["innovation_clip_m"],
                "fusion_axes": first["fusion_axes"],
                "range_rmse_raw_mean_m": float(np.mean(values("range_rmse_raw_m"))),
                "range_rmse_observation_mean_m": float(np.mean(values("range_rmse_observation_m"))),
                "range_rmse_motion_mean_m": float(np.mean(values("range_rmse_motion_m"))),
                "position_rmse_observation_mean_m": float(np.mean(values("position_rmse_observation_m"))),
                "position_rmse_motion_mean_m": float(np.mean(values("position_rmse_motion_m"))),
                "target_max_abs_error_observation_max_m": float(
                    np.max(values("target_max_abs_error_observation_m"))
                ),
                "target_max_abs_error_motion_max_m": float(np.max(values("target_max_abs_error_motion_m"))),
                "target_rmse_observation_mean_m": float(np.mean(values("target_rmse_observation_m"))),
                "target_rmse_motion_mean_m": float(np.mean(values("target_rmse_motion_m"))),
                "runtime_truth_used": False,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "rdf_axis_contract": motion.RDF_AXIS_CONTRACT,
                "public_topic_contract": motion.PUBLIC_TOPIC_CONTRACT,
            }
        )
    return sorted(
        summaries,
        key=lambda row: (
            -int(row["pass_count"]),
            float(row["target_max_abs_error_motion_max_m"]),
            float(row["range_rmse_motion_mean_m"]),
            float(row["position_rmse_motion_mean_m"]),
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
        description="Sweep multi-burst image observation fusion followed by runtime-safe alpha-beta RDF motion fusion."
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
    parser.add_argument("--alphas", type=motion.parse_float_list, default=motion.parse_float_list("0.8,0.9,1.0"))
    parser.add_argument("--betas", type=motion.parse_float_list, default=motion.parse_float_list("0.001,0.005"))
    parser.add_argument("--innovation-clips-m", type=motion.parse_float_list, default=motion.parse_float_list("8,12,20,40"))
    parser.add_argument("--axes", choices=("rdf", "forward"), default="forward")
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
                            build_multiburst_series(
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
                        for alpha in args.alphas:
                            for beta in args.betas:
                                for clip in args.innovation_clips_m:
                                    for series in series_list:
                                        rows.append(
                                            evaluate_combined(
                                                series,
                                                burst_count=int(burst_count),
                                                observations_per_burst=int(observations_per_burst),
                                                inner_mode=str(inner_mode),
                                                outer_mode=str(outer_mode),
                                                trim_fraction=float(trim_fraction),
                                                common_fraction=float(common_fraction),
                                                alpha=float(alpha),
                                                beta=float(beta),
                                                innovation_clip_m=float(clip),
                                                axes=str(args.axes),
                                                target_range_m=float(args.target_range_m),
                                                target_window_m=float(args.target_window_m),
                                            )
                                        )
    summaries = aggregate(rows)
    write_csv(args.output_dir / "multiburst_motion_fusion_seed_metrics.csv", rows)
    write_csv(args.output_dir / "multiburst_motion_fusion_method_summary.csv", summaries)
    summary = {
        "experiment_id": args.experiment_id,
        "profile_json": str(args.profile_json),
        "scenario": args.scenario,
        "seed_start": args.seed_start,
        "seed_end": int(args.seed_start) + int(args.trial_count) - 1,
        "trial_count": args.trial_count,
        "best_method": summaries[0] if summaries else {},
        "coordinate_contracts_explicit": True,
        "image_contract": multiburst.image_prefilter.IMAGE_CONTRACT,
        "rdf_axis_contract": motion.RDF_AXIS_CONTRACT,
        "public_topic_contract": motion.PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": motion.TRUTH_RDF_TRANSFORM_CONTRACT,
        "ned_to_rdf_contract": motion.NED_TO_RDF_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }
    write_json(args.output_dir / "multiburst_motion_fusion_metrics.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: test multi-burst image/disparity observation fusion followed by causal motion innovation gating.",
                "",
                "Coordinate contract: image u/v/disparity observations are fused and re-triangulated into stereo-center RDF right/down/forward; motion state is also RDF. PX4 local NED truth is converted to RDF for audit only. Public StereoDistance topics remain FRD+range.",
                "",
                "Runtime contract: no PX4/RTK truth, target-window label, true image feature, or true disparity is consumed by either stage.",
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
