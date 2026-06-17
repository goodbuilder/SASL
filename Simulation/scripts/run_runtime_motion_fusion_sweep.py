#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
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


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
NED_TO_RDF_CONTRACT = "rdf_right=east, rdf_down=down, rdf_forward=north"
RUNTIME_INPUT_CONTRACT = (
    "causal alpha-beta motion fusion consumes timestamped corrected stereo-center RDF only; "
    "PX4/RTK truth is audit-only"
)


@dataclass(frozen=True)
class CachedSeedSeries:
    seed: int
    scenario: str
    samples: list[pipe.CalibrationSample]
    validation_ids: set[int]
    raw_validation_points: np.ndarray
    corrected_points: np.ndarray


def parse_float_list(value: str) -> list[float]:
    parsed = [float(item.strip()) for item in value.split(",") if item.strip()]
    if not parsed:
        raise argparse.ArgumentTypeError("list must not be empty")
    return parsed


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


def alpha_beta_innovation_clip(
    *,
    samples: Sequence[pipe.CalibrationSample],
    measurements_rdf_m: np.ndarray,
    alpha: float,
    beta: float,
    innovation_clip_m: float,
    axes: str = "rdf",
) -> np.ndarray:
    """Causal constant-velocity fusion over stereo-center RDF measurements.

    The filter never consumes truth. It predicts the next RDF point from the
    previous fused point and velocity, clips the measurement innovation norm,
    and applies an alpha-beta update. Axis names are explicit to avoid x/y/z
    ambiguity: array columns are RDF right/down/forward.
    """
    if len(samples) != int(measurements_rdf_m.shape[0]):
        raise ValueError("sample count and measurement row count must match")
    if measurements_rdf_m.ndim != 2 or measurements_rdf_m.shape[1] != 3:
        raise ValueError("measurements must be an Nx3 RDF array")
    if len(samples) == 0:
        return np.empty((0, 3), dtype=float)
    alpha = min(1.0, max(0.0, float(alpha)))
    beta = max(0.0, float(beta))
    clip = max(0.0, float(innovation_clip_m))
    if axes == "forward":
        fused_axis_mask = np.array([False, False, True], dtype=bool)
    elif axes == "rdf":
        fused_axis_mask = np.array([True, True, True], dtype=bool)
    else:
        raise ValueError("axes must be 'rdf' or 'forward'")

    fused: list[np.ndarray] = []
    position = np.asarray(measurements_rdf_m[0], dtype=float).copy()
    velocity = np.zeros(3, dtype=float)
    last_t = float(samples[0].t_s)
    fused.append(position.copy())
    for sample, measurement in zip(samples[1:], measurements_rdf_m[1:]):
        dt = max(1.0e-3, float(sample.t_s) - last_t)
        last_t = float(sample.t_s)
        measurement = np.asarray(measurement, dtype=float)
        prediction = position + velocity * dt
        innovation = measurement - prediction
        innovation = np.where(fused_axis_mask, innovation, 0.0)
        norm = float(np.linalg.norm(innovation))
        if clip > 0.0 and norm > clip:
            innovation = innovation * (clip / norm)
        position = prediction + alpha * innovation
        velocity = velocity + (beta / dt) * innovation
        position = np.where(fused_axis_mask, position, measurement)
        velocity = np.where(fused_axis_mask, velocity, 0.0)
        fused.append(position.copy())
    return np.vstack(fused)


def build_cached_series(
    *,
    profile: replay_audit.RuntimeProfile,
    seed: int,
    scenario: str,
    sample_count: int,
    train_ratio: float,
    target_range_m: float,
    target_window_m: float,
    target_oversample_count: int,
) -> CachedSeedSeries:
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
    corrected_points: list[np.ndarray] = []
    for sample in samples:
        result = profile.apply(sample_payload(sample))
        if result.corrected is None:
            corrected_points.append(np.array([math.nan, math.nan, math.nan], dtype=float))
            continue
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
    validation_ids = {sample.sample_id for sample in validation}
    validation_sorted = [sample for sample in samples if sample.sample_id in validation_ids]
    raw_validation_points = pipe.stack(validation_sorted, "raw_rdf_m")
    return CachedSeedSeries(
        seed=seed,
        scenario=scenario,
        samples=samples,
        validation_ids=validation_ids,
        raw_validation_points=raw_validation_points,
        corrected_points=np.vstack(corrected_points),
    )


def evaluate_fusion(
    series: CachedSeedSeries,
    *,
    alpha: float,
    beta: float,
    innovation_clip_m: float,
    axes: str,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    fused = alpha_beta_innovation_clip(
        samples=series.samples,
        measurements_rdf_m=series.corrected_points,
        alpha=alpha,
        beta=beta,
        innovation_clip_m=innovation_clip_m,
        axes=axes,
    )
    validation_samples: list[pipe.CalibrationSample] = []
    validation_points: list[np.ndarray] = []
    for sample, point in zip(series.samples, fused):
        if sample.sample_id in series.validation_ids:
            validation_samples.append(sample)
            validation_points.append(point)
    fused_validation = np.vstack(validation_points) if validation_points else np.empty((0, 3), dtype=float)
    raw_metrics = pipe.evaluate_predictions(
        validation_samples,
        series.raw_validation_points,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    fused_metrics = pipe.evaluate_predictions(
        validation_samples,
        fused_validation,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    passed = (
        fused_metrics["range_rmse_m"] < raw_metrics["range_rmse_m"]
        and fused_metrics["position_rmse_m"] < raw_metrics["position_rmse_m"]
        and fused_metrics["target_window_count"] > 0
        and fused_metrics["target_max_abs_error_m"] <= 10.0
    )
    return {
        "seed": series.seed,
        "scenario": series.scenario,
        "method": method_name(alpha=alpha, beta=beta, innovation_clip_m=innovation_clip_m, axes=axes),
        "status": "PASS" if passed else "FAIL",
        "alpha": float(alpha),
        "beta": float(beta),
        "innovation_clip_m": float(innovation_clip_m),
        "fusion_axes": axes,
        "validation_sample_count": len(validation_samples),
        "range_rmse_raw_m": raw_metrics["range_rmse_m"],
        "range_rmse_fused_m": fused_metrics["range_rmse_m"],
        "position_rmse_raw_m": raw_metrics["position_rmse_m"],
        "position_rmse_fused_m": fused_metrics["position_rmse_m"],
        "right_rmse_fused_m": fused_metrics["right_rmse_m"],
        "down_rmse_fused_m": fused_metrics["down_rmse_m"],
        "forward_rmse_fused_m": fused_metrics["forward_rmse_m"],
        "target_count": fused_metrics["target_window_count"],
        "target_max_abs_error_fused_m": fused_metrics["target_max_abs_error_m"],
        "target_rmse_fused_m": fused_metrics["target_rmse_m"],
        "runtime_truth_used": False,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def method_name(*, alpha: float, beta: float, innovation_clip_m: float, axes: str) -> str:
    def label(value: float) -> str:
        return f"{value:g}".replace(".", "p").replace("-", "m")

    return (
        f"causal_alpha_beta_{axes}_innovation_clip"
        f"_a{label(alpha)}_b{label(beta)}_g{label(innovation_clip_m)}"
    )


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
                "alpha": first["alpha"],
                "beta": first["beta"],
                "innovation_clip_m": first["innovation_clip_m"],
                "fusion_axes": first["fusion_axes"],
                "range_rmse_raw_mean_m": float(np.mean(values("range_rmse_raw_m"))),
                "range_rmse_fused_mean_m": float(np.mean(values("range_rmse_fused_m"))),
                "range_rmse_fused_max_m": float(np.max(values("range_rmse_fused_m"))),
                "position_rmse_raw_mean_m": float(np.mean(values("position_rmse_raw_m"))),
                "position_rmse_fused_mean_m": float(np.mean(values("position_rmse_fused_m"))),
                "position_rmse_fused_max_m": float(np.max(values("position_rmse_fused_m"))),
                "target_max_abs_error_fused_max_m": float(np.max(values("target_max_abs_error_fused_m"))),
                "target_rmse_fused_mean_m": float(np.mean(values("target_rmse_fused_m"))),
                "runtime_truth_used": False,
                "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
                "rdf_axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    summaries.sort(
        key=lambda row: (
            -int(row["pass_count"]),
            float(row["target_max_abs_error_fused_max_m"]),
            float(row["range_rmse_fused_mean_m"]),
            float(row["position_rmse_fused_mean_m"]),
        )
    )
    return summaries


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
        description=(
            "Sweep runtime-safe causal alpha-beta motion fusion over StereoDistance profile outputs. "
            "Calibration truth is audit-only; runtime features are timestamped corrected RDF samples."
        )
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
    parser.add_argument("--alphas", type=parse_float_list, default=parse_float_list("0.5,0.65,0.8,0.9,1.0"))
    parser.add_argument("--betas", type=parse_float_list, default=parse_float_list("0.001,0.005,0.02,0.08"))
    parser.add_argument("--innovation-clips-m", type=parse_float_list, default=parse_float_list("8,12,20,40,80"))
    parser.add_argument("--axes", choices=("rdf", "forward"), default="rdf")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile = replay_audit.load_profile(args.profile_json)
    series = [
        build_cached_series(
            profile=profile,
            seed=int(args.seed_start) + index,
            scenario=str(args.scenario),
            sample_count=int(args.sample_count),
            train_ratio=float(args.train_ratio),
            target_range_m=float(args.target_range_m),
            target_window_m=float(args.target_window_m),
            target_oversample_count=int(args.target_oversample_count),
        )
        for index in range(int(args.trial_count))
    ]
    rows: list[dict[str, object]] = []
    for alpha in args.alphas:
        for beta in args.betas:
            for clip in args.innovation_clips_m:
                for cached in series:
                    rows.append(
                        evaluate_fusion(
                            cached,
                            alpha=float(alpha),
                            beta=float(beta),
                            innovation_clip_m=float(clip),
                            axes=str(args.axes),
                            target_range_m=float(args.target_range_m),
                            target_window_m=float(args.target_window_m),
                        )
                    )
    summaries = aggregate(rows)
    write_csv(args.output_dir / "motion_fusion_seed_metrics.csv", rows)
    write_csv(args.output_dir / "motion_fusion_method_summary.csv", summaries)
    summary = {
        "experiment_id": args.experiment_id,
        "profile_json": str(args.profile_json),
        "scenario": args.scenario,
        "seed_start": args.seed_start,
        "seed_end": int(args.seed_start) + int(args.trial_count) - 1,
        "trial_count": args.trial_count,
        "best_method": summaries[0] if summaries else {},
        "coordinate_contracts_explicit": True,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "ned_to_rdf_contract": NED_TO_RDF_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }
    write_json(args.output_dir / "motion_fusion_metrics.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: test runtime-safe causal multi-frame motion fusion over corrected StereoDistance RDF outputs.",
                "",
                "Coordinate contract: profile outputs and fusion state are stereo-center RDF right/down/forward. Calibration/evaluation truth is PX4 local NED converted to RDF and is audit-only. Public StereoDistance topics remain FRD+range forward/right/down/range.",
                "",
                "Runtime contract: the alpha-beta fusion uses timestamped corrected RDF samples only. It does not consume PX4/RTK truth, target-window labels, or true disparity.",
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
