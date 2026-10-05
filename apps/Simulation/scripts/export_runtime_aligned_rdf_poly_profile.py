#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import rtk_stereo_calibration_pipeline as pipe  # noqa: E402
import audit_runtime_profile_fresh_seed_replay as replay_audit  # noqa: E402
import run_runtime_aligned_rdf_poly_degree_search as search  # noqa: E402


def zscore_poly_to_raw_disparity(coefficients_high_to_low: Sequence[float], *, mean: float, std: float) -> list[float]:
    scale = float(std) if math.isfinite(float(std)) and abs(float(std)) > 1.0e-12 else 1.0
    z_of_d = np.poly1d([1.0 / scale, -float(mean) / scale])
    raw_poly = np.poly1d(np.asarray(coefficients_high_to_low, dtype=float))(z_of_d)
    return [float(value) for value in raw_poly.coeffs]


def fit_profile(args: argparse.Namespace) -> tuple[dict[str, object], list[pipe.CalibrationSample]]:
    sample_count = int(args.train_sample_count) + int(args.validation_sample_count)
    camera, injection = replay_audit.camera_and_injection_for_scenario(str(args.scenario))
    source = pipe.SyntheticCalibrationDataSource(
        camera=camera,
        injection=injection,
        sample_count=sample_count,
        seed=int(args.seed),
        target_range_m=float(args.target_range_m),
        target_window_m=float(args.target_window_m),
        target_oversample_count=int(args.target_oversample_count),
    )
    samples = source.load_samples()
    train, validation = pipe.split_samples(
        samples,
        train_ratio=float(args.train_sample_count) / float(sample_count),
        seed=int(args.seed) + 17,
        target_range_m=float(args.target_range_m),
        target_window_m=float(args.target_window_m),
    )
    all_samples = train + validation
    raw_train = pipe.stack(train, "raw_rdf_m")
    truth_train = pipe.stack(train, "truth_rdf_m")
    d_train = pipe.measured_disparity(train)
    d_mean = float(np.mean(d_train))
    d_std = float(np.std(d_train))
    x_train = search.zscore(d_train, mean=d_mean, std=d_std)
    scale, rotation, translation, weights = pipe.fit_huber_umeyama_sim3(
        raw_train,
        truth_train,
        delta_m=float(args.huber_delta_m),
    )
    sim_train = pipe.predict_sim3(raw_train, scale, rotation, translation)
    residual_train = truth_train - sim_train
    z_coefficients = {
        "right": search.ridge_polyfit_fixed_degree(
            x_train,
            residual_train[:, 0],
            degree=int(args.right_degree),
            alpha=float(args.ridge_alpha),
        ),
        "down": search.ridge_polyfit_fixed_degree(
            x_train,
            residual_train[:, 1],
            degree=int(args.down_degree),
            alpha=float(args.ridge_alpha),
        ),
        "forward": search.ridge_polyfit_fixed_degree(
            x_train,
            residual_train[:, 2],
            degree=int(args.forward_degree),
            alpha=float(args.ridge_alpha),
        ),
    }
    raw_coefficients = {
        axis: zscore_poly_to_raw_disparity(coefficients, mean=d_mean, std=d_std)
        for axis, coefficients in z_coefficients.items()
    }
    method = (
        f"sim3_rdf_d{int(args.right_degree)}_d{int(args.down_degree)}_d{int(args.forward_degree)}"
        f"_zscore_ridge{search.value_label(float(args.ridge_alpha))}"
        f"_huber{search.value_label(float(args.huber_delta_m))}"
        f"_forward_clamp_{search.value_label(float(args.forward_clamp_m))}_runtime_aligned"
    )
    profile = {
        "mode": "estimated_to_truth_sim3",
        "profile_id": f"{args.experiment_id}-recommended-runtime-profile",
        "source_experiment": str(args.experiment_id),
        "method": method,
        "source_scenario": str(args.scenario),
        "axis_contract": (
            "internal_policy_vector=RDF(right,down,forward); "
            "legacy_stereodistance_position_topics=FRD_range(forward,right,down,range)"
        ),
        "training_contract": (
            "Global Sim(3) and RDF residual disparity polynomials are fitted from calibration-time "
            "PX4/RTK truth; runtime uses only raw stereo RDF, measured disparity, and calibration-fixed "
            "parameters. Residual polynomials were fitted in z-scored measured disparity and exported "
            "as raw-disparity-domain coefficients for the StereoDistance runtime loader."
        ),
        "runtime_inputs": [
            "raw_right_m",
            "raw_down_m",
            "raw_forward_m",
            "raw_disparity_px",
        ],
        "forbidden_runtime_inputs": [
            "calibration_truth",
            "px4_rtk_truth",
            "true_disparity",
            "true_image_feature_location",
        ],
        "scale": float(scale),
        "rotation_matrix_rdf": rotation.as_matrix().tolist(),
        "rotation_euler_zyx_deg": rotation.as_euler("zyx", degrees=True).tolist(),
        "translation_rdf_m": [float(value) for value in translation],
        "rdf_residual_disparity_coefficients_high_to_low": raw_coefficients,
        "forward_residual_clamp_max_abs_m": float(args.forward_clamp_m),
        "coordinate_contract": {
            "input_frame": "stereo_center_opencv_optical",
            "input_axis_order": "right,down,forward",
            "output_frame": "stereo_center_opencv_optical",
            "output_axis_order": "right,down,forward",
            "truth_source_frame_for_calibration_only": "source frame declared by calibration data",
            "truth_rdf_transform_contract": (
                "truth_rdf must be stereo_center_RDF from explicit T_source_to_stereo_rdf; "
                "direct NED-to-RDF axis alias is forbidden"
            ),
            "public_stereodistance_topic_order": "forward,right,down,range",
        },
        "robust_fit_audit": {
            "ridge_lambda": float(args.ridge_alpha),
            "huber_delta_m": float(args.huber_delta_m),
            "rdf_forward_residual_uses_forward_clamp": True,
            "right_degree": int(args.right_degree),
            "down_degree": int(args.down_degree),
            "forward_degree": int(args.forward_degree),
            "disparity_zscore_mean_px": d_mean,
            "disparity_zscore_std_px": d_std,
            "zscore_coefficients_high_to_low": {
                axis: [float(value) for value in coefficients]
                for axis, coefficients in z_coefficients.items()
            },
            "mean_huber_weight": float(np.mean(weights)),
            "min_huber_weight": float(np.min(weights)),
            "train_sample_count": len(train),
            "validation_sample_count": len(validation),
            "source_scenario": str(args.scenario),
        },
    }
    return profile, all_samples


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a runtime-aligned RDF polynomial StereoDistance profile.")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--experiment-id", required=True)
    parser.add_argument("--seed", type=int, default=20295559)
    parser.add_argument(
        "--scenario",
        default="nominal",
        choices=tuple(replay_audit.SCENARIO_PRESETS),
        help="Synthetic scenario used to fit the exported profile.",
    )
    parser.add_argument("--train-sample-count", type=int, default=1260)
    parser.add_argument("--validation-sample-count", type=int, default=420)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-oversample-count", type=int, default=72)
    parser.add_argument("--right-degree", type=int, default=6)
    parser.add_argument("--down-degree", type=int, default=7)
    parser.add_argument("--forward-degree", type=int, default=12)
    parser.add_argument("--ridge-alpha", type=float, default=0.1)
    parser.add_argument("--huber-delta-m", type=float, default=8.0)
    parser.add_argument("--forward-clamp-m", type=float, default=2.0)
    return parser.parse_args()


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(pipe.json_safe(payload), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile, samples = fit_profile(args)
    profile_path = args.output_dir / "recommended_runtime_profile.json"
    samples_path = args.output_dir / "calibration_samples.csv"
    write_json(profile_path, profile)
    pipe.write_csv(samples_path, pipe.rows_for_samples(samples))
    summary = {
        "experiment_id": args.experiment_id,
        "profile_path": str(profile_path),
        "calibration_samples_csv": str(samples_path),
        "profile_id": profile["profile_id"],
        "method": profile["method"],
        "source_scenario": profile["source_scenario"],
        "coordinate_contracts_explicit": True,
        "runtime_truth_used": False,
        "axis_contract": profile["axis_contract"],
        "coordinate_contract": profile["coordinate_contract"],
    }
    write_json(args.output_dir / "export_summary.json", summary)
    (args.output_dir / "README.md").write_text(
        "\n".join(
            [
                f"# {args.experiment_id}",
                "",
                "Purpose: export the current runtime-aligned RDF polynomial correction as a StereoDistance runtime profile.",
                "",
                "Coordinate contract: calibration truth is PX4/local NED north/east/down converted to stereo-center RDF right/down/forward. Runtime inputs are raw RDF and measured disparity only. Public StereoDistance topics remain FRD+range forward/right/down/range.",
                "",
                f"Profile: `{profile_path.name}`",
                f"Source scenario: `{args.scenario}`",
                f"Samples: `{samples_path.name}`",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(pipe.json_safe(summary), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
