#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


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
import run_runtime_multiburst_disparity_residual_sweep as residual  # noqa: E402
import run_runtime_multiburst_observation_fusion_sweep as multiburst  # noqa: E402


RUNTIME_INPUT_CONTRACT = (
    "exported residual profile was fitted with calibration train truth, but runtime apply consumes only "
    "fused measured_disparity_px and corrected stereo-center RDF"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a runtime-loadable disparity residual profile JSON.")
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--scenario", default="disparity_noise_stress", choices=tuple(replay_audit.SCENARIO_PRESETS))
    parser.add_argument("--seed", type=int, default=21220000)
    parser.add_argument("--sample-count", type=int, default=1680)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-oversample-count", type=int, default=72)
    parser.add_argument("--burst-spec", type=multiburst.parse_burst_specs, default=multiburst.parse_burst_specs("7x3"))
    parser.add_argument("--inner-mode", default="trimmed_mean")
    parser.add_argument("--outer-mode", default="trimmed_mean")
    parser.add_argument("--trim-fraction", type=float, default=0.05)
    parser.add_argument("--common-fraction", type=float, default=0.5)
    parser.add_argument("--feature-space", default="log_disparity")
    parser.add_argument("--correction-space", default="ray_scale")
    parser.add_argument("--degree", type=int, default=2)
    parser.add_argument("--ridge-lambda", type=float, default=1.0)
    parser.add_argument("--correction-clip", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    burst_count, observations_per_burst = args.burst_spec[0]
    profile = replay_audit.load_profile(args.profile_json)
    series = residual.build_series(
        profile=profile,
        seed=int(args.seed),
        scenario=str(args.scenario),
        sample_count=int(args.sample_count),
        train_ratio=float(args.train_ratio),
        target_range_m=float(args.target_range_m),
        target_window_m=float(args.target_window_m),
        target_oversample_count=int(args.target_oversample_count),
        burst_count=int(burst_count),
        observations_per_burst=int(observations_per_burst),
        inner_mode=str(args.inner_mode),
        outer_mode=str(args.outer_mode),
        trim_fraction=float(args.trim_fraction),
        common_fraction=float(args.common_fraction),
    )
    model = residual.fit_residual_model(
        series,
        feature_space=str(args.feature_space),
        correction_space=str(args.correction_space),
        degree=int(args.degree),
        ridge_lambda=float(args.ridge_lambda),
        correction_clip=float(args.correction_clip),
    )
    method = residual.method_name(
        burst_count=int(burst_count),
        observations_per_burst=int(observations_per_burst),
        inner_mode=str(args.inner_mode),
        outer_mode=str(args.outer_mode),
        trim_fraction=float(args.trim_fraction),
        common_fraction=float(args.common_fraction),
        feature_space=model.feature_space,
        correction_space=model.correction_space,
        degree=model.degree,
        ridge_lambda=model.ridge_lambda,
        correction_clip=model.correction_clip,
    )
    payload = {
        "profile_type": "DisparityResidualRuntimeProfile",
        "profile_id": f"{args.scenario}_seed_{int(args.seed)}_disparity_residual",
        "method": method,
        "feature_space": model.feature_space,
        "correction_space": model.correction_space,
        "degree": model.degree,
        "ridge_lambda": model.ridge_lambda,
        "correction_clip": model.correction_clip,
        "feature_mean": model.feature_mean,
        "feature_scale": model.feature_scale,
        "coefficients_low_to_high": list(model.coefficients_low_to_high),
        "observation_fusion": {
            "burst_count": int(burst_count),
            "observations_per_burst": int(observations_per_burst),
            "inner_mode": str(args.inner_mode),
            "outer_mode": str(args.outer_mode),
            "trim_fraction": float(args.trim_fraction),
            "common_fraction": float(args.common_fraction),
        },
        "fit_source": {
            "base_profile_json": str(args.profile_json),
            "scenario": str(args.scenario),
            "seed": int(args.seed),
            "sample_count": int(args.sample_count),
            "train_ratio": float(args.train_ratio),
            "target_range_m": float(args.target_range_m),
            "target_window_m": float(args.target_window_m),
            "target_oversample_count": int(args.target_oversample_count),
            "truth_usage": "calibration_train_split_only",
        },
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "rdf_axis_contract": residual.RDF_AXIS_CONTRACT,
        "public_topic_contract": residual.PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": residual.TRUTH_RDF_TRANSFORM_CONTRACT,
        "ned_to_rdf_contract": residual.NED_TO_RDF_CONTRACT,
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(pipe.json_safe(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(pipe.json_safe(payload), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
