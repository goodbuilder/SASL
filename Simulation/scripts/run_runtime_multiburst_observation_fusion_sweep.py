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
import run_runtime_correlated_repeated_observation_sweep as correlated  # noqa: E402
import run_runtime_image_prefilter_sweep as image_prefilter  # noqa: E402
import run_runtime_repeated_observation_fusion_sweep as repeated  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "multi-burst observation fusion consumes left_u_px, left_v_px, and measured_disparity_px "
    "observations grouped into short bursts; common-mode error may exist within a burst, "
    "but burst-to-burst common error is synthetic/audit-only and PX4/RTK truth is not a runtime input"
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
    trim_label = f"{trim_fraction:g}".replace(".", "p")
    common_label = f"{common_fraction:g}".replace(".", "p")
    return (
        f"multiburst_observation_inner_{inner_mode}_outer_{outer_mode}"
        f"_b{int(burst_count)}x{int(observations_per_burst)}"
        f"_trim{trim_label}_common{common_label}"
    )


def fuse_observation_tuples(
    observations: Sequence[tuple[float, float, float]],
    *,
    mode: str,
    trim_fraction: float,
) -> tuple[float, float, float]:
    return (
        repeated.fuse_values([obs[0] for obs in observations], mode=mode, trim_fraction=trim_fraction),
        repeated.fuse_values([obs[1] for obs in observations], mode=mode, trim_fraction=trim_fraction),
        repeated.fuse_values([obs[2] for obs in observations], mode=mode, trim_fraction=trim_fraction),
    )


def synthesize_multiburst_observation(
    *,
    truth_rdf_m: np.ndarray,
    camera: pipe.CameraModel,
    injection: pipe.ErrorInjection,
    rng: np.random.Generator,
    burst_count: int,
    observations_per_burst: int,
    common_fraction: float,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
) -> tuple[float, float, float]:
    burst_level_observations: list[tuple[float, float, float]] = []
    for _ in range(int(burst_count)):
        burst_observations = correlated.synthesize_correlated_observations(
            truth_rdf_m=truth_rdf_m,
            camera=camera,
            injection=injection,
            rng=rng,
            repeat_count=int(observations_per_burst),
            common_fraction=float(common_fraction),
        )
        burst_level_observations.append(
            fuse_observation_tuples(
                burst_observations,
                mode=inner_mode,
                trim_fraction=trim_fraction,
            )
        )
    return fuse_observation_tuples(
        burst_level_observations,
        mode=outer_mode,
        trim_fraction=trim_fraction,
    )


def build_multiburst_corrected_points(
    *,
    samples: Sequence[pipe.CalibrationSample],
    profile: replay_audit.RuntimeProfile,
    camera: pipe.CameraModel,
    injection: pipe.ErrorInjection,
    seed: int,
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
) -> np.ndarray:
    rng = np.random.default_rng(
        int(seed)
        + 62000023
        + int(burst_count) * 1009
        + int(observations_per_burst) * 917
        + int(round(common_fraction * 1000.0))
    )
    corrected_points: list[np.ndarray] = []
    for sample in samples:
        left_u, left_v, disparity = synthesize_multiburst_observation(
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
    burst_count: int,
    observations_per_burst: int,
    inner_mode: str,
    outer_mode: str,
    trim_fraction: float,
    common_fraction: float,
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
    corrected_all = build_multiburst_corrected_points(
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
        "method": method_name(
            burst_count=burst_count,
            observations_per_burst=observations_per_burst,
            inner_mode=inner_mode,
            outer_mode=outer_mode,
            trim_fraction=trim_fraction,
            common_fraction=common_fraction,
        ),
        "status": "PASS" if passed else "FAIL",
        "burst_count": int(burst_count),
        "observations_per_burst": int(observations_per_burst),
        "total_observation_count": int(burst_count) * int(observations_per_burst),
        "inner_mode": inner_mode,
        "outer_mode": outer_mode,
        "trim_fraction": float(trim_fraction),
        "common_fraction": float(common_fraction),
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
                "burst_count": first["burst_count"],
                "observations_per_burst": first["observations_per_burst"],
                "total_observation_count": first["total_observation_count"],
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


def parse_burst_specs(value: str) -> list[tuple[int, int]]:
    specs: list[tuple[int, int]] = []
    for item in value.split(","):
        item = item.strip().lower()
        if not item:
            continue
        if "x" not in item:
            raise argparse.ArgumentTypeError("burst specs must use BxN, for example 3x5")
        burst_text, obs_text = item.split("x", 1)
        specs.append((int(burst_text), int(obs_text)))
    if not specs:
        raise argparse.ArgumentTypeError("burst spec list must not be empty")
    return specs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep hierarchical multi-burst image observation fusion before StereoDistance profile correction."
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
    parser.add_argument("--burst-specs", type=parse_burst_specs, default=parse_burst_specs("1x15,3x5,5x5,5x7"))
    parser.add_argument("--inner-modes", type=repeated.parse_modes, default=repeated.parse_modes("trimmed_mean"))
    parser.add_argument("--outer-modes", type=repeated.parse_modes, default=repeated.parse_modes("median,trimmed_mean"))
    parser.add_argument("--trim-fraction", type=float, default=0.2)
    parser.add_argument("--common-fractions", type=correlated.parse_float_list, default=correlated.parse_float_list("0.25,0.5,0.75,1"))
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
    for common_fraction in args.common_fractions:
        for burst_count, observations_per_burst in args.burst_specs:
            for inner_mode in args.inner_modes:
                for outer_mode in args.outer_modes:
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
                                burst_count=int(burst_count),
                                observations_per_burst=int(observations_per_burst),
                                inner_mode=str(inner_mode),
                                outer_mode=str(outer_mode),
                                trim_fraction=float(args.trim_fraction),
                                common_fraction=float(common_fraction),
                            )
                        )
    summaries = aggregate(rows)
    write_csv(args.output_dir / "multiburst_observation_seed_metrics.csv", rows)
    write_csv(args.output_dir / "multiburst_observation_method_summary.csv", summaries)
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
    write_json(args.output_dir / "multiburst_observation_metrics.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: test hierarchical multi-burst fusion under within-burst common-mode image/disparity error.",
                "",
                "Coordinate contract: bursted left image u/v and disparity observations are fused in pixel space, re-triangulated into stereo-center RDF right/down/forward, then corrected by the runtime profile. PX4 local NED truth is used only to synthesize/audit samples.",
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
