#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

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
    "causal inverse-depth prefilter consumes timestamped left_u_px, left_v_px, "
    "measured inverse depth derived from disparity/(fx*baseline), camera intrinsics/baseline, "
    "and calibration-fixed profile parameters; PX4/RTK truth is audit-only"
)


def method_name(*, alpha: float, beta: float, uv_clip_px: float, inverse_depth_clip_1pm: float) -> str:
    def label(value: float) -> str:
        return f"{value:g}".replace(".", "p").replace("-", "m")

    return (
        f"causal_inverse_depth_prefilter_a{label(alpha)}_b{label(beta)}"
        f"_uv{label(uv_clip_px)}_id{label(inverse_depth_clip_1pm)}"
    )


def evaluate_prefilter(
    series: image_prefilter.CachedImageSeries,
    *,
    profile: replay_audit.RuntimeProfile,
    alpha: float,
    beta: float,
    uv_clip_px: float,
    inverse_depth_clip_1pm: float,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    times = np.array([sample.t_s for sample in series.samples], dtype=float)
    filtered_u = image_prefilter.causal_alpha_beta_scalar(
        series.left_u_px,
        times,
        alpha=alpha,
        beta=beta,
        innovation_clip=uv_clip_px,
    )
    filtered_v = image_prefilter.causal_alpha_beta_scalar(
        series.left_v_px,
        times,
        alpha=alpha,
        beta=beta,
        innovation_clip=uv_clip_px,
    )
    product = series.camera.fx_meas_px * series.camera.baseline_meas_m
    inverse_depth = series.disparity_px / max(product, 1.0e-12)
    filtered_inverse_depth = image_prefilter.causal_alpha_beta_scalar(
        inverse_depth,
        times,
        alpha=alpha,
        beta=beta,
        innovation_clip=inverse_depth_clip_1pm,
    )
    filtered_inverse_depth = np.maximum(filtered_inverse_depth, 1.0e-9)
    filtered_disparity = filtered_inverse_depth * product

    validation_samples: list[pipe.CalibrationSample] = []
    corrected_points: list[np.ndarray] = []
    for index, sample in enumerate(series.samples):
        raw_rdf = image_prefilter.reconstruct_raw_rdf(
            camera=series.camera,
            injection=series.injection,
            left_u_px=float(filtered_u[index]),
            left_v_px=float(filtered_v[index]),
            disparity_px=float(filtered_disparity[index]),
        )
        result = profile.apply(
            image_prefilter.profile_payload(
                raw_rdf_m=raw_rdf,
                left_u_px=float(filtered_u[index]),
                left_v_px=float(filtered_v[index]),
                disparity_px=float(filtered_disparity[index]),
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
        "method": method_name(
            alpha=alpha,
            beta=beta,
            uv_clip_px=uv_clip_px,
            inverse_depth_clip_1pm=inverse_depth_clip_1pm,
        ),
        "status": "PASS" if passed else "FAIL",
        "alpha": float(alpha),
        "beta": float(beta),
        "uv_clip_px": float(uv_clip_px),
        "inverse_depth_clip_1pm": float(inverse_depth_clip_1pm),
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


def aggregate(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(str(row["method"]), []).append(row)
    summaries = []
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
                "inverse_depth_clip_1pm": first["inverse_depth_clip_1pm"],
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
            float(row["position_rmse_corrected_mean_m"]),
        )
    )
    return summaries


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep causal inverse-depth-domain prefiltering before StereoDistance runtime profile correction."
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
    parser.add_argument("--alphas", type=image_prefilter.parse_float_list, default=image_prefilter.parse_float_list("0.85,0.9,0.95,1.0"))
    parser.add_argument("--betas", type=image_prefilter.parse_float_list, default=image_prefilter.parse_float_list("0,0.001,0.005"))
    parser.add_argument("--uv-clips-px", type=image_prefilter.parse_float_list, default=image_prefilter.parse_float_list("20,999"))
    parser.add_argument(
        "--inverse-depth-clips-1pm",
        type=image_prefilter.parse_float_list,
        default=image_prefilter.parse_float_list("0.0000001,0.00000025,0.0000005,0.000001,0.0000025,0.000005,999"),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile = replay_audit.load_profile(args.profile_json)
    series = [
        image_prefilter.build_cached_series(
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
    rows = []
    for alpha in args.alphas:
        for beta in args.betas:
            for uv_clip in args.uv_clips_px:
                for inverse_depth_clip in args.inverse_depth_clips_1pm:
                    for cached in series:
                        rows.append(
                            evaluate_prefilter(
                                cached,
                                profile=profile,
                                alpha=float(alpha),
                                beta=float(beta),
                                uv_clip_px=float(uv_clip),
                                inverse_depth_clip_1pm=float(inverse_depth_clip),
                                target_range_m=float(args.target_range_m),
                                target_window_m=float(args.target_window_m),
                            )
                        )
    summaries = aggregate(rows)
    image_prefilter.write_csv(args.output_dir / "inverse_depth_prefilter_seed_metrics.csv", rows)
    image_prefilter.write_csv(args.output_dir / "inverse_depth_prefilter_method_summary.csv", summaries)
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
    image_prefilter.write_json(args.output_dir / "inverse_depth_prefilter_metrics.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: test runtime-safe causal inverse-depth-domain prefiltering before StereoDistance profile correction.",
                "",
                "Coordinate contract: measured disparity is converted to inverse depth using fx*baseline, filtered in 1/m, converted back to disparity, re-triangulated into stereo-center RDF right/down/forward, then corrected by the runtime profile. PX4 local NED truth is audit-only.",
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
