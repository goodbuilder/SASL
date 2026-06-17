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
IMAGE_CONTRACT = "left image pixel center u/v and measured stereo disparity in pixels"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
NED_TO_RDF_CONTRACT = "rdf_right=east, rdf_down=down, rdf_forward=north"
RUNTIME_INPUT_CONTRACT = (
    "causal image-space prefilter consumes timestamped left_u_px, left_v_px, measured_disparity_px, "
    "camera intrinsics/baseline, and calibration-fixed profile parameters; PX4/RTK truth is audit-only"
)


@dataclass(frozen=True)
class CachedImageSeries:
    seed: int
    scenario: str
    samples: list[pipe.CalibrationSample]
    validation_ids: set[int]
    camera: pipe.CameraModel
    injection: pipe.ErrorInjection
    raw_validation_points: np.ndarray
    left_u_px: np.ndarray
    left_v_px: np.ndarray
    disparity_px: np.ndarray


def parse_float_list(value: str) -> list[float]:
    parsed = [float(item.strip()) for item in value.split(",") if item.strip()]
    if not parsed:
        raise argparse.ArgumentTypeError("list must not be empty")
    return parsed


def causal_alpha_beta_scalar(
    values: Sequence[float],
    times_s: Sequence[float],
    *,
    alpha: float,
    beta: float,
    innovation_clip: float,
) -> np.ndarray:
    values_arr = np.asarray(values, dtype=float)
    times_arr = np.asarray(times_s, dtype=float)
    if values_arr.ndim != 1 or times_arr.ndim != 1 or values_arr.shape[0] != times_arr.shape[0]:
        raise ValueError("values and times must be same-length 1D arrays")
    if values_arr.size == 0:
        return np.array([], dtype=float)
    alpha = min(1.0, max(0.0, float(alpha)))
    beta = max(0.0, float(beta))
    clip = max(0.0, float(innovation_clip))
    filtered = np.empty_like(values_arr)
    position = float(values_arr[0])
    velocity = 0.0
    last_t = float(times_arr[0])
    filtered[0] = position
    for index in range(1, values_arr.size):
        dt = max(1.0e-3, float(times_arr[index]) - last_t)
        last_t = float(times_arr[index])
        prediction = position + velocity * dt
        innovation = float(values_arr[index]) - prediction
        if clip > 0.0 and abs(innovation) > clip:
            innovation = math.copysign(clip, innovation)
        position = prediction + alpha * innovation
        velocity = velocity + (beta / dt) * innovation
        filtered[index] = position
    return filtered


def reconstruct_raw_rdf(
    *,
    camera: pipe.CameraModel,
    injection: pipe.ErrorInjection,
    left_u_px: float,
    left_v_px: float,
    disparity_px: float,
) -> np.ndarray:
    raw_rdf = pipe.triangulate_left_disparity(
        left_u_px=float(left_u_px),
        left_v_px=float(left_v_px),
        disparity_px=max(float(disparity_px), 1.0e-6),
        fx_px=camera.fx_meas_px,
        fy_px=camera.fy_meas_px,
        cx_px=camera.cx_px,
        cy_px=camera.cy_px,
        baseline_m=camera.baseline_meas_m,
    )
    return pipe.apply_rotation_to_rows(raw_rdf[None, :], injection.raw_rotation)[0]


def profile_payload(
    *,
    raw_rdf_m: np.ndarray,
    left_u_px: float,
    left_v_px: float,
    disparity_px: float,
) -> dict[str, float]:
    return {
        "right_m": float(raw_rdf_m[0]),
        "down_m": float(raw_rdf_m[1]),
        "forward_m": float(raw_rdf_m[2]),
        "range_m": float(np.linalg.norm(raw_rdf_m)),
        "disparity_px": float(disparity_px),
        "left_u_px": float(left_u_px),
        "left_v_px": float(left_v_px),
    }


def build_cached_series(
    *,
    seed: int,
    scenario: str,
    sample_count: int,
    train_ratio: float,
    target_range_m: float,
    target_window_m: float,
    target_oversample_count: int,
) -> CachedImageSeries:
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
    return CachedImageSeries(
        seed=seed,
        scenario=scenario,
        samples=samples,
        validation_ids=validation_ids,
        camera=camera,
        injection=injection,
        raw_validation_points=pipe.stack(validation_sorted, "raw_rdf_m"),
        left_u_px=np.array([sample.left_u_px for sample in samples], dtype=float),
        left_v_px=np.array([sample.left_v_px for sample in samples], dtype=float),
        disparity_px=np.array([sample.measured_disparity_px for sample in samples], dtype=float),
    )


def evaluate_prefilter(
    series: CachedImageSeries,
    *,
    profile: replay_audit.RuntimeProfile,
    alpha: float,
    beta: float,
    uv_clip_px: float,
    disparity_clip_px: float,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    times = np.array([sample.t_s for sample in series.samples], dtype=float)
    filtered_u = causal_alpha_beta_scalar(series.left_u_px, times, alpha=alpha, beta=beta, innovation_clip=uv_clip_px)
    filtered_v = causal_alpha_beta_scalar(series.left_v_px, times, alpha=alpha, beta=beta, innovation_clip=uv_clip_px)
    filtered_d = causal_alpha_beta_scalar(
        series.disparity_px,
        times,
        alpha=alpha,
        beta=beta,
        innovation_clip=disparity_clip_px,
    )
    validation_samples: list[pipe.CalibrationSample] = []
    corrected_points: list[np.ndarray] = []
    for index, sample in enumerate(series.samples):
        raw_rdf = reconstruct_raw_rdf(
            camera=series.camera,
            injection=series.injection,
            left_u_px=float(filtered_u[index]),
            left_v_px=float(filtered_v[index]),
            disparity_px=float(filtered_d[index]),
        )
        result = profile.apply(
            profile_payload(
                raw_rdf_m=raw_rdf,
                left_u_px=float(filtered_u[index]),
                left_v_px=float(filtered_v[index]),
                disparity_px=float(filtered_d[index]),
            )
        )
        if sample.sample_id in series.validation_ids and result.corrected is not None:
            validation_samples.append(sample)
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
    corrected_arr = np.vstack(corrected_points) if corrected_points else np.empty((0, 3), dtype=float)
    raw_metrics = pipe.evaluate_predictions(
        validation_samples,
        series.raw_validation_points[: corrected_arr.shape[0]],
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
        "seed": series.seed,
        "scenario": series.scenario,
        "method": method_name(alpha=alpha, beta=beta, uv_clip_px=uv_clip_px, disparity_clip_px=disparity_clip_px),
        "status": "PASS" if passed else "FAIL",
        "alpha": float(alpha),
        "beta": float(beta),
        "uv_clip_px": float(uv_clip_px),
        "disparity_clip_px": float(disparity_clip_px),
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
        "image_contract": IMAGE_CONTRACT,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def method_name(*, alpha: float, beta: float, uv_clip_px: float, disparity_clip_px: float) -> str:
    def label(value: float) -> str:
        return f"{value:g}".replace(".", "p").replace("-", "m")

    return (
        f"causal_image_prefilter_a{label(alpha)}_b{label(beta)}"
        f"_uv{label(uv_clip_px)}_d{label(disparity_clip_px)}"
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
                "uv_clip_px": first["uv_clip_px"],
                "disparity_clip_px": first["disparity_clip_px"],
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
                "image_contract": IMAGE_CONTRACT,
                "rdf_axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    summaries.sort(
        key=lambda row: (
            -int(row["pass_count"]),
            float(row["target_max_abs_error_corrected_max_m"]),
            float(row["range_rmse_corrected_mean_m"]),
            float(row["position_rmse_corrected_mean_m"]),
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
        description="Sweep causal image-space prefiltering before StereoDistance runtime profile correction."
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
    parser.add_argument("--alphas", type=parse_float_list, default=parse_float_list("0.65,0.75,0.85,1.0"))
    parser.add_argument("--betas", type=parse_float_list, default=parse_float_list("0,0.001,0.005,0.02"))
    parser.add_argument("--uv-clips-px", type=parse_float_list, default=parse_float_list("1.0,2.5,5.0"))
    parser.add_argument("--disparity-clips-px", type=parse_float_list, default=parse_float_list("0.03,0.05,0.08,0.12,0.2,0.4"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile = replay_audit.load_profile(args.profile_json)
    series = [
        build_cached_series(
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
            for uv_clip in args.uv_clips_px:
                for disparity_clip in args.disparity_clips_px:
                    for cached in series:
                        rows.append(
                            evaluate_prefilter(
                                cached,
                                profile=profile,
                                alpha=float(alpha),
                                beta=float(beta),
                                uv_clip_px=float(uv_clip),
                                disparity_clip_px=float(disparity_clip),
                                target_range_m=float(args.target_range_m),
                                target_window_m=float(args.target_window_m),
                            )
                        )
    summaries = aggregate(rows)
    write_csv(args.output_dir / "image_prefilter_seed_metrics.csv", rows)
    write_csv(args.output_dir / "image_prefilter_method_summary.csv", summaries)
    summary = {
        "experiment_id": args.experiment_id,
        "profile_json": str(args.profile_json),
        "scenario": args.scenario,
        "seed_start": args.seed_start,
        "seed_end": int(args.seed_start) + int(args.trial_count) - 1,
        "trial_count": args.trial_count,
        "best_method": summaries[0] if summaries else {},
        "coordinate_contracts_explicit": True,
        "image_contract": IMAGE_CONTRACT,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "ned_to_rdf_contract": NED_TO_RDF_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "runtime_truth_used": False,
    }
    write_json(args.output_dir / "image_prefilter_metrics.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: test runtime-safe causal image-space prefiltering before StereoDistance profile correction.",
                "",
                "Coordinate contract: left image u/v and disparity are filtered in pixel space, re-triangulated into stereo-center RDF right/down/forward with measured camera intrinsics/baseline, then corrected by the runtime profile. PX4 local NED truth is converted to RDF for audit only.",
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
