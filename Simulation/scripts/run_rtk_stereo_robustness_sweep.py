#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
MULTI_SEED_SCRIPT = ROOT_DIR / "scripts" / "run_rtk_stereo_policy_multi_seed_validation.py"
PIPELINE_SCRIPT = ROOT_DIR / "scripts" / "rtk_stereo_calibration_pipeline.py"
PROFILE_REPLAY_SCRIPT = ROOT_DIR / "scripts" / "stereodistance_profile_replay.py"
PROFILE_PROMOTION_CHECK_SCRIPT = ROOT_DIR / "scripts" / "stereodistance_profile_promotion_check.py"

RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
NED_AXIS_CONTRACT = "px4_local_position_ned:north,east,down"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


@dataclass(frozen=True)
class Scenario:
    name: str
    purpose: str
    args: tuple[str, ...]


SCENARIOS: dict[str, Scenario] = {
    "nominal": Scenario(
        name="nominal",
        purpose="default synthetic calibration error injection",
        args=(),
    ),
    "yaw_pitch_stress": Scenario(
        name="yaw_pitch_stress",
        purpose="larger camera attitude error while keeping disparity noise near nominal",
        args=(
            "--yaw-error-deg",
            "0.70",
            "--pitch-error-deg",
            "-0.35",
            "--roll-error-deg",
            "0.12",
        ),
    ),
    "disparity_noise_stress": Scenario(
        name="disparity_noise_stress",
        purpose="larger disparity bias, disparity noise, and bbox-center noise",
        args=(
            "--disparity-bias-px",
            "0.55",
            "--disparity-noise-px",
            "0.12",
            "--bbox-noise-px",
            "0.25",
        ),
    ),
    "intrinsic_scale_stress": Scenario(
        name="intrinsic_scale_stress",
        purpose="larger baseline and focal scale mismatch",
        args=(
            "--baseline-scale-error",
            "0.012",
            "--focal-scale-error",
            "-0.008",
        ),
    ),
    "combined_sensor_stress": Scenario(
        name="combined_sensor_stress",
        purpose="simultaneous attitude, intrinsic scale, disparity bias/noise, and bbox-center stress",
        args=(
            "--yaw-error-deg",
            "0.85",
            "--pitch-error-deg",
            "-0.45",
            "--roll-error-deg",
            "0.16",
            "--baseline-scale-error",
            "0.012",
            "--focal-scale-error",
            "-0.008",
            "--disparity-bias-px",
            "0.60",
            "--disparity-noise-px",
            "0.14",
            "--bbox-noise-px",
            "0.30",
        ),
    ),
    "bbox_outlier_stress": Scenario(
        name="bbox_outlier_stress",
        purpose="gross bbox-center and disparity outliers to exercise redescending/RANSAC/trimmed estimators",
        args=(
            "--bbox-outlier-rate",
            "0.08",
            "--bbox-outlier-px",
            "2.5",
            "--disparity-outlier-rate",
            "0.06",
            "--disparity-outlier-px",
            "0.45",
        ),
    ),
    "sparse_target_window": Scenario(
        name="sparse_target_window",
        purpose="non-default diagnostic scenario with weak 1852m target-window coverage",
        args=(
            "--synthetic-target-oversample-count",
            "2",
        ),
    ),
}

DEFAULT_SCENARIOS = (
    "nominal",
    "yaw_pitch_stress",
    "disparity_noise_stress",
    "intrinsic_scale_stress",
    "combined_sensor_stress",
    "bbox_outlier_stress",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run multi-seed RTK stereo calibration across multiple synthetic error "
            "injection scenarios and choose methods that remain valid across scenarios."
        )
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scenarios", default=",".join(DEFAULT_SCENARIOS))
    parser.add_argument(
        "--scenario-config-json",
        type=Path,
        help=(
            "Optional JSON file defining additional synthetic error scenarios. "
            "Use either {'scenarios': [{'name': ..., 'purpose': ..., 'args': [...]}, ...]} "
            "or {'scenario_name': {'purpose': ..., 'args': [...]}}."
        ),
    )
    parser.add_argument("--seeds", default="20260613,20260614,20260615")
    parser.add_argument("--sample-count", type=int, default=520)
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--synthetic-target-oversample-count", type=int, default=12)
    parser.add_argument("--disparity-outlier-rate", type=float, default=0.0)
    parser.add_argument("--disparity-outlier-px", type=float, default=0.0)
    parser.add_argument("--bbox-outlier-rate", type=float, default=0.0)
    parser.add_argument("--bbox-outlier-px", type=float, default=0.0)
    parser.add_argument("--min-validation-improvement-m", type=float, default=0.01)
    parser.add_argument(
        "--pooled-candidate-scan-limit",
        type=int,
        default=64,
        help=(
            "Maximum number of pooled runtime profiles to replay across all scenarios. "
            "Set to 0 to replay every pooled runtime profile."
        ),
    )
    parser.add_argument(
        "--skip-near-tie-transfer",
        action="store_true",
        help=(
            "Skip cross-scenario replay for scenario near-tie profiles. "
            "Use this for quick pooled/profile-policy probes when near-tie transfer is not under test."
        ),
    )
    return parser.parse_args()


def load_custom_scenarios(path: Path | None) -> dict[str, Scenario]:
    if path is None:
        return {}
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, Mapping) and isinstance(payload.get("scenarios"), list):
        entries = payload["scenarios"]
    elif isinstance(payload, Mapping):
        entries = [
            {"name": name, **value}
            for name, value in payload.items()
            if isinstance(value, Mapping)
        ]
    else:
        raise ValueError("--scenario-config-json must be a JSON object")
    scenarios: dict[str, Scenario] = {}
    for entry in entries:
        if not isinstance(entry, Mapping):
            raise ValueError("scenario entries must be objects")
        name = str(entry.get("name", "")).strip()
        if not name:
            raise ValueError("custom scenario requires a non-empty name")
        purpose = str(entry.get("purpose", "custom synthetic error scenario"))
        args_value = entry.get("args", [])
        if not isinstance(args_value, list) or not all(isinstance(item, (str, int, float)) for item in args_value):
            raise ValueError(f"custom scenario {name!r} args must be a list of strings/numbers")
        scenarios[name] = Scenario(
            name=name,
            purpose=purpose,
            args=tuple(str(item) for item in args_value),
        )
    return scenarios


def scenario_registry(custom_scenarios: Mapping[str, Scenario] | None = None) -> dict[str, Scenario]:
    registry = dict(SCENARIOS)
    if custom_scenarios:
        registry.update(custom_scenarios)
    return registry


def parse_scenario_list(value: str, *, registry: Mapping[str, Scenario] | None = None) -> list[Scenario]:
    scenarios_by_name = registry or SCENARIOS
    scenarios = []
    for item in value.split(","):
        name = item.strip()
        if not name:
            continue
        if name not in scenarios_by_name:
            known = ", ".join(scenarios_by_name)
            raise ValueError(f"unknown scenario '{name}'. Known scenarios: {known}")
        scenarios.append(scenarios_by_name[name])
    if not scenarios:
        raise ValueError("--scenarios must contain at least one scenario")
    return scenarios


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def finite_float(value: Any, default: float = math.nan) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def method_contract_fields(method: str) -> dict[str, str]:
    name = str(method or "")
    if not name:
        return {
            "registration_model": "none",
            "residual_model": "none",
            "regularization_model": "none",
            "runtime_profile_family": "none",
            "selection_note": "no method selected",
        }
    if name == "raw":
        registration = "none_raw_stereo_rdf"
    elif "affine" in name:
        registration = "affine_rdf_estimated_to_truth"
    elif "piecewise_raw_forward" in name and "sim3" in name:
        registration = "piecewise_raw_forward_sim3_estimated_to_truth"
    elif "inverse_depth_linear_then_sim3" in name:
        registration = "inverse_depth_ray_scale_then_sim3_estimated_to_truth"
    elif "sim3" in name:
        registration = "sim3_estimated_to_truth"
    elif "global_scale_rotation" in name:
        registration = "scaled_rotation_estimated_to_truth"
    elif "global_rotation" in name:
        registration = "rotation_estimated_to_truth"
    elif "global_inverse_depth" in name or "ridge_global_inverse_depth" in name:
        registration = "raw_rdf_inverse_depth_ray_scale"
    elif "disparity" in name:
        registration = "raw_rdf_ray_scale"
    else:
        registration = "other_estimated_to_truth"

    if "piecewise_raw_forward" in name and "sim3" in name:
        residual = "none"
    elif "quadratic_local" in name and "raw_forward" in name:
        residual = "forward_quadratic_local_raw_forward_lut"
    elif "inverse_depth" in name and "raw_forward" in name:
        residual = "forward_inverse_depth_raw_forward_lut"
    elif "quadratic" in name and "raw_forward" in name:
        residual = "forward_quadratic_raw_forward_lut"
    elif "range_normalized" in name and "raw_forward" in name:
        residual = "forward_range_normalized_raw_forward_lut"
    elif "agreement_gated" in name and "raw_forward" in name:
        residual = "forward_disparity_raw_forward_agreement_gated_lut"
    elif "blended" in name and "raw_forward" in name:
        residual = "forward_disparity_raw_forward_blend_lut"
    elif "raw_forward" in name:
        residual = "forward_raw_rdf_forward_lut"
    elif "rdf_residual" in name or "binned_rdf_residual" in name:
        residual = "per_rdf_axis_disparity_residual"
    elif "binned_residual" in name:
        residual = "forward_disparity_lut"
    elif "residual_disparity" in name:
        residual = "forward_disparity_polynomial"
    elif "global_disparity" in name:
        residual = "corrected_disparity_polynomial"
    elif "global_inverse_depth" in name or "ridge_global_inverse_depth" in name:
        residual = "corrected_inverse_depth_linear"
    else:
        residual = "none"

    regularizers: list[str] = []
    if "smoothed_binned" in name:
        regularizers.append("lut_shrinkage_neighbor_smoothing")
    if "kernel_smoothed" in name:
        regularizers.append("kernel_smoothing")
    if "capped_binned" in name:
        regularizers.append("lut_robust_winsorized_cap")
    if "blended" in name and "piecewise_raw_forward" not in name:
        regularizers.append("dual_key_residual_blend")
    if "blended_piecewise_raw_forward" in name:
        regularizers.append("global_piecewise_registration_blend")
    if "blended_piecewise_raw_forward" in name and "_alpha" in name:
        regularizers.append("piecewise_blend_alpha_grid")
    if "agreement_gated" in name:
        regularizers.append("dual_key_residual_agreement_gate")
    if "range_normalized" in name:
        regularizers.append("range_normalized_residual")
    if "inverse_depth" in name:
        regularizers.append("inverse_depth_residual")
    if "quadratic_local" in name:
        regularizers.append("semi_parametric_local_residual")
    if "quadratic" in name and "raw_forward" in name:
        regularizers.append("stereo_depth_bias_quadratic")
    if "ridge" in name:
        regularizers.append("ridge")
    if "huber" in name:
        regularizers.append("huber_irls")
    if "tukey" in name:
        regularizers.append("tukey_biweight_irls")
    if "trimmed" in name:
        regularizers.append("least_trimmed_consensus")
    if "ransac" in name:
        regularizers.append("ransac_consensus")
    if "range_weighted" in name or "target_weighted" in name or "target_shrunk" in name:
        regularizers.append("target_range_weighting")
    if "target_shrunk" in name:
        regularizers.append("target_weight_shrinkage")
    if "piecewise_raw_forward" in name and "sim3" in name:
        regularizers.append("raw_forward_piecewise_registration")
    if "bins" in name or "_support" in name:
        regularizers.append("lut_bin_count_grid")
    regularization = "+".join(regularizers) if regularizers else "none"

    if registration == "affine_rdf_estimated_to_truth":
        profile_family = "estimated_to_truth_affine_rdf"
    elif registration in {
        "sim3_estimated_to_truth",
        "piecewise_raw_forward_sim3_estimated_to_truth",
        "inverse_depth_ray_scale_then_sim3_estimated_to_truth",
        "scaled_rotation_estimated_to_truth",
        "rotation_estimated_to_truth",
    }:
        profile_family = "estimated_to_truth_sim3"
    elif residual in {"corrected_disparity_polynomial", "corrected_inverse_depth_linear"}:
        profile_family = "estimated_to_truth_disparity_poly"
    elif name == "raw":
        profile_family = "none_raw"
    else:
        profile_family = "offline_or_nonpreferred_runtime_candidate"

    if registration == "piecewise_raw_forward_sim3_estimated_to_truth":
        selection_note = "range-dependent Sim(3) registration nodes blended by runtime raw RDF forward distance"
    elif registration == "inverse_depth_ray_scale_then_sim3_estimated_to_truth":
        selection_note = "inverse-depth ray-scale correction followed by Sim(3) pose/origin registration"
    elif residual == "forward_quadratic_local_raw_forward_lut":
        selection_note = "quadratic raw-forward residual calibration with shrunk local residual-of-residual LUT"
    elif residual == "forward_inverse_depth_raw_forward_lut":
        selection_note = "inverse-depth raw-forward residual calibration for stereo disparity scale/bias error"
    elif residual == "forward_quadratic_raw_forward_lut":
        selection_note = "quadratic raw-forward residual calibration for disparity-bias-like range error"
    elif residual == "forward_range_normalized_raw_forward_lut":
        selection_note = "range-proportional forward residual calibration keyed by runtime raw RDF forward distance"
    elif residual == "forward_disparity_raw_forward_agreement_gated_lut":
        selection_note = "dual-key residual calibration that suppresses residual correction when runtime LUT predictions disagree"
    elif residual == "forward_disparity_raw_forward_blend_lut":
        selection_note = "dual-key piecewise residual calibration split between measured disparity and raw RDF forward distance"
    elif residual == "forward_raw_rdf_forward_lut":
        selection_note = "piecewise forward residual calibration keyed by runtime raw RDF forward distance"
    elif residual == "forward_disparity_lut" and "capped_binned" in name:
        selection_note = "limits residual LUT gain to reduce transfer instability from sparse or stressed calibration data"
    elif residual == "forward_disparity_lut" and "smoothed_binned" in name:
        selection_note = "useful when outlier or sparse-bin stability beats raw bin medians"
    elif residual == "forward_disparity_lut":
        selection_note = "piecewise disparity residual calibration; validate bin count against held-out flight data"
    elif registration == "affine_rdf_estimated_to_truth":
        selection_note = "higher-flexibility estimated-to-truth mapping; require validation stability before promotion"
    elif "sim3" in name:
        selection_note = "similarity registration baseline for global pose/origin/scale error"
    elif residual == "corrected_disparity_polynomial":
        selection_note = "range-only ray-scale correction; limited pose/origin correction"
    elif residual == "corrected_inverse_depth_linear":
        selection_note = "range-only inverse-depth ray-scale correction; useful for stereo depth scale/bias diagnostics"
    else:
        selection_note = "diagnostic candidate"

    return {
        "registration_model": registration,
        "residual_model": residual,
        "regularization_model": regularization,
        "runtime_profile_family": profile_family,
        "selection_note": selection_note,
    }


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return bool(value)
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def json_safe(value: object) -> object:
    if isinstance(value, Mapping):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        value = float(value)
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    return value


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_safe(payload), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def run_scenario(args: argparse.Namespace, scenario: Scenario, scenario_dir: Path) -> int:
    command = [
        sys.executable,
        str(MULTI_SEED_SCRIPT),
        "--output-dir",
        str(scenario_dir),
        "--seeds",
        args.seeds,
        "--sample-count",
        str(args.sample_count),
        "--train-ratio",
        str(args.train_ratio),
        "--target-range-m",
        str(args.target_range_m),
        "--target-window-m",
        str(args.target_window_m),
        "--synthetic-target-oversample-count",
        str(args.synthetic_target_oversample_count),
        "--disparity-outlier-rate",
        str(args.disparity_outlier_rate),
        "--disparity-outlier-px",
        str(args.disparity_outlier_px),
        "--bbox-outlier-rate",
        str(args.bbox_outlier_rate),
        "--bbox-outlier-px",
        str(args.bbox_outlier_px),
        "--min-validation-improvement-m",
        str(args.min_validation_improvement_m),
        *scenario.args,
    ]
    completed = subprocess.run(command, cwd=ROOT_DIR)
    return int(completed.returncode)


def collect_scenario_outputs(
    *,
    scenario: Scenario,
    scenario_dir: Path,
    exit_code: int,
    sweep_output_dir: Path,
) -> tuple[dict[str, object], list[dict[str, object]], dict[str, object]]:
    recommendation_path = scenario_dir / "multi_seed_recommendation.json"
    representative_checks_path = scenario_dir / "representative_profile_checks.json"
    cross_seed_path = scenario_dir / "representative_cross_seed_replay_summary.json"
    recommendation = json.loads(recommendation_path.read_text(encoding="utf-8")) if recommendation_path.exists() else {}
    representative_checks = json.loads(representative_checks_path.read_text(encoding="utf-8")) if representative_checks_path.exists() else {}
    cross_seed = json.loads(cross_seed_path.read_text(encoding="utf-8")) if cross_seed_path.exists() else {}

    run_row = {
        "scenario": scenario.name,
        "purpose": scenario.purpose,
        "scenario_dir": str(scenario_dir.relative_to(sweep_output_dir)),
        "exit_code": exit_code,
        "pass_run": exit_code == 0,
        "recommended_method": recommendation.get("method", ""),
        "recommendation_mode": recommendation.get("recommendation_mode", "none"),
        "pass_representative_profile_checks": bool(representative_checks.get("pass_representative_profile_checks", False)),
        "pass_cross_seed_replay": bool(cross_seed.get("pass_cross_seed_replay", False)),
        "axis_contract": RDF_AXIS_CONTRACT,
    }
    gate_rows = []
    for row in read_csv(scenario_dir / "method_stability_gate.csv"):
        gate_rows.append({"scenario": scenario.name, **row})
    recommendation_row = {
        "scenario": scenario.name,
        "method": recommendation.get("method", ""),
        **method_contract_fields(str(recommendation.get("method", ""))),
        "recommendation_mode": recommendation.get("recommendation_mode", "none"),
        "score": recommendation.get("score", ""),
        "validation_range_rmse_mean_m": recommendation.get("validation_range_rmse_mean_m", ""),
        "target_abs_mean_error_mean_m": recommendation.get("target_abs_mean_error_mean_m", ""),
        "target_abs_mean_error_max_m": recommendation.get("target_abs_mean_error_max_m", ""),
        "axis_rmse_regression_sum_m": recommendation.get("axis_rmse_regression_sum_m", ""),
        "model_complexity_mean": recommendation.get("model_complexity_mean", ""),
        "pass_representative_profile_checks": bool(representative_checks.get("pass_representative_profile_checks", False)),
        "pass_cross_seed_replay": bool(cross_seed.get("pass_cross_seed_replay", False)),
    }
    return run_row, gate_rows, recommendation_row


def collect_scenario_near_tie_profiles(
    *,
    output_dir: Path,
    scenarios: Sequence[Scenario],
) -> list[dict[str, object]]:
    rows_out: list[dict[str, object]] = []
    for scenario in scenarios:
        scenario_dir = output_dir / scenario.name
        for row in read_csv(scenario_dir / "selection_near_tie_profiles.csv"):
            profile_path = str(row.get("profile_path", ""))
            absolute_profile_path = scenario_dir / profile_path if profile_path else scenario_dir
            rows_out.append(
                {
                    "scenario": scenario.name,
                    "purpose": scenario.purpose,
                    **row,
                    "scenario_profile_path": (
                        str((scenario_dir / profile_path).relative_to(output_dir))
                        if profile_path
                        else ""
                    ),
                    "scenario_profile_exists": bool(profile_path) and absolute_profile_path.exists(),
                    "axis_contract": RDF_AXIS_CONTRACT,
                    "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
                }
            )
    rows_out.sort(
        key=lambda row: (
            str(row.get("scenario", "")),
            int(float(row.get("selection_rank", 999999) or 999999)),
            int(float(row.get("seed", 0) or 0)),
        )
    )
    return rows_out


def aggregate_robust_methods(
    method_gate_rows: Sequence[Mapping[str, object]],
    *,
    scenario_count: int,
) -> list[dict[str, object]]:
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in method_gate_rows:
        method = str(row.get("method", ""))
        if method == "raw" or not method:
            continue
        grouped.setdefault(method, []).append(row)
    summary_rows = []
    for method, rows in grouped.items():
        strict_count = sum(str(row.get("strict_pass", "")).lower() == "true" for row in rows)
        performance_count = sum(str(row.get("performance_pass", "")).lower() == "true" for row in rows)
        coverage_count = sum(str(row.get("run_coverage_ok", "")).lower() == "true" for row in rows)
        target_count = sum(str(row.get("target_error_within_tolerance", "")).lower() == "true" for row in rows)
        validation_mean = np.array([finite_float(row.get("method_validation_range_rmse_mean_m"), float("inf")) for row in rows], dtype=float)
        target_mean = np.array([finite_float(row.get("method_target_abs_mean_error_mean_m"), float("inf")) for row in rows], dtype=float)
        target_max = np.array([finite_float(row.get("method_target_abs_mean_error_max_m"), float("inf")) for row in rows], dtype=float)
        axis_sum = np.array([finite_float(row.get("axis_rmse_regression_sum_m"), 0.0) for row in rows], dtype=float)
        complexity = np.array([finite_float(row.get("model_complexity_mean"), 0.0) for row in rows], dtype=float)
        finite_validation = validation_mean[np.isfinite(validation_mean)]
        finite_target_mean = target_mean[np.isfinite(target_mean)]
        finite_target_max = target_max[np.isfinite(target_max)]
        finite_axis_sum = axis_sum[np.isfinite(axis_sum)]
        finite_complexity = complexity[np.isfinite(complexity)]
        missing_penalty = max(0, scenario_count - len(rows))
        strict_all = strict_count == scenario_count and scenario_count > 0
        performance_all = performance_count == scenario_count and scenario_count > 0
        robust_score = (
            (float(finite_validation.mean()) if finite_validation.size else 1.0e9)
            + 0.25 * (float(finite_target_mean.mean()) if finite_target_mean.size else 1.0e9)
            + 0.10 * (float(finite_axis_sum.mean()) if finite_axis_sum.size else 1.0e9)
            + 0.005 * (float(finite_complexity.mean()) if finite_complexity.size else 1.0e9)
            + 10.0 * missing_penalty
            + 3.0 * max(0, scenario_count - performance_count)
            + 1.0 * max(0, scenario_count - strict_count)
        )
        summary_rows.append(
            {
                "method": method,
                **method_contract_fields(method),
                "scenario_count": len(rows),
                "expected_scenario_count": scenario_count,
                "strict_pass_count": strict_count,
                "performance_pass_count": performance_count,
                "run_coverage_ok_count": coverage_count,
                "target_error_within_tolerance_count": target_count,
                "strict_pass_all_scenarios": strict_all,
                "performance_pass_all_scenarios": performance_all,
                "validation_range_rmse_mean_m": float(finite_validation.mean()) if finite_validation.size else float("inf"),
                "validation_range_rmse_max_m": float(finite_validation.max()) if finite_validation.size else float("inf"),
                "target_abs_mean_error_mean_m": float(finite_target_mean.mean()) if finite_target_mean.size else float("inf"),
                "target_abs_mean_error_max_m": float(finite_target_max.max()) if finite_target_max.size else float("inf"),
                "axis_rmse_regression_sum_mean_m": float(finite_axis_sum.mean()) if finite_axis_sum.size else float("inf"),
                "model_complexity_mean": float(finite_complexity.mean()) if finite_complexity.size else float("inf"),
                "robust_score": robust_score,
                "axis_contract": RDF_AXIS_CONTRACT,
            }
        )
    summary_rows.sort(
        key=lambda row: (
            not bool(row["strict_pass_all_scenarios"]),
            not bool(row["performance_pass_all_scenarios"]),
            -int(row["strict_pass_count"]),
            -int(row["performance_pass_count"]),
            float(row["robust_score"]),
            str(row["method"]),
        )
    )
    return summary_rows


def choose_robust_recommendation(summary_rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    if not summary_rows:
        return {
            "recommendation_mode": "none",
            "method": "",
            "reason": "no_method_gate_rows_available",
        }
    strict = [row for row in summary_rows if bool(row.get("strict_pass_all_scenarios"))]
    performance = [row for row in summary_rows if bool(row.get("performance_pass_all_scenarios"))]
    if strict:
        mode = "strict_all_scenarios"
        candidates = strict
    elif performance:
        mode = "performance_all_scenarios"
        candidates = performance
    else:
        mode = "best_partial_robustness"
        candidates = list(summary_rows)
    selected = min(
        candidates,
        key=lambda row: (
            -int(row.get("strict_pass_count", 0)),
            -int(row.get("performance_pass_count", 0)),
            float(row.get("robust_score", float("inf"))),
            str(row.get("method", "")),
        ),
    )
    return {
        "recommendation_mode": mode,
        "method": selected["method"],
        **method_contract_fields(str(selected["method"])),
        "reason": "selected_from_cross_scenario_method_stability_gate",
        "robust_score": selected["robust_score"],
        "strict_pass_count": selected["strict_pass_count"],
        "performance_pass_count": selected["performance_pass_count"],
        "expected_scenario_count": selected["expected_scenario_count"],
        "validation_range_rmse_mean_m": selected["validation_range_rmse_mean_m"],
        "validation_range_rmse_max_m": selected["validation_range_rmse_max_m"],
        "target_abs_mean_error_mean_m": selected["target_abs_mean_error_mean_m"],
        "target_abs_mean_error_max_m": selected["target_abs_mean_error_max_m"],
        "axis_rmse_regression_sum_mean_m": selected["axis_rmse_regression_sum_mean_m"],
        "model_complexity_mean": selected["model_complexity_mean"],
        "axis_contract": RDF_AXIS_CONTRACT,
        "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
    }


def aggregate_robust_method_families(
    robust_summary_rows: Sequence[Mapping[str, object]],
) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str, str, str], list[Mapping[str, object]]] = {}
    for row in robust_summary_rows:
        key = (
            str(row.get("registration_model", "")),
            str(row.get("residual_model", "")),
            str(row.get("regularization_model", "")),
            str(row.get("runtime_profile_family", "")),
        )
        grouped.setdefault(key, []).append(row)
    family_rows: list[dict[str, object]] = []
    for (registration, residual, regularization, profile_family), rows in grouped.items():
        best = min(
            rows,
            key=lambda row: (
                not bool(row.get("strict_pass_all_scenarios")),
                not bool(row.get("performance_pass_all_scenarios")),
                finite_float(row.get("robust_score"), float("inf")),
                finite_float(row.get("validation_range_rmse_mean_m"), float("inf")),
                str(row.get("method", "")),
            ),
        )
        strict_all_count = sum(bool(row.get("strict_pass_all_scenarios")) for row in rows)
        performance_all_count = sum(bool(row.get("performance_pass_all_scenarios")) for row in rows)
        validation_values = np.array(
            [finite_float(row.get("validation_range_rmse_mean_m"), float("nan")) for row in rows],
            dtype=float,
        )
        target_values = np.array(
            [finite_float(row.get("target_abs_mean_error_max_m"), float("nan")) for row in rows],
            dtype=float,
        )
        score_values = np.array(
            [finite_float(row.get("robust_score"), float("nan")) for row in rows],
            dtype=float,
        )
        finite_validation = validation_values[np.isfinite(validation_values)]
        finite_target = target_values[np.isfinite(target_values)]
        finite_scores = score_values[np.isfinite(score_values)]
        family_rows.append(
            {
                "registration_model": registration,
                "residual_model": residual,
                "regularization_model": regularization,
                "runtime_profile_family": profile_family,
                "method_count": len(rows),
                "strict_all_method_count": strict_all_count,
                "performance_all_method_count": performance_all_count,
                "best_method": best.get("method", ""),
                "best_method_selection_note": best.get("selection_note", ""),
                "best_robust_score": best.get("robust_score", float("inf")),
                "best_validation_range_rmse_mean_m": best.get("validation_range_rmse_mean_m", float("inf")),
                "best_target_abs_mean_error_max_m": best.get("target_abs_mean_error_max_m", float("inf")),
                "family_validation_range_rmse_mean_m": float(finite_validation.mean()) if finite_validation.size else float("inf"),
                "family_target_abs_mean_error_max_m": float(finite_target.max()) if finite_target.size else float("inf"),
                "family_robust_score_mean": float(finite_scores.mean()) if finite_scores.size else float("inf"),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "offline family diagnostic; runtime profiles still use raw RDF/disparity only",
            }
        )
    family_rows.sort(
        key=lambda row: (
            -int(row["strict_all_method_count"]),
            -int(row["performance_all_method_count"]),
            finite_float(row["best_robust_score"], float("inf")),
            str(row["registration_model"]),
            str(row["residual_model"]),
            str(row["regularization_model"]),
        )
    )
    return family_rows


def robust_policy_recommendation_rows(summary_rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    candidates = [
        row for row in summary_rows
        if bool(row.get("performance_pass_all_scenarios"))
    ]
    if not candidates:
        candidates = [
            row for row in summary_rows
            if str(row.get("method", "")) and str(row.get("method", "")) != "raw"
        ]
    if not candidates:
        return []

    def validation_mean(row: Mapping[str, object]) -> float:
        return finite_float(row.get("validation_range_rmse_mean_m"), float("inf"))

    def validation_max(row: Mapping[str, object]) -> float:
        return finite_float(row.get("validation_range_rmse_max_m"), float("inf"))

    def target_mean(row: Mapping[str, object]) -> float:
        return finite_float(row.get("target_abs_mean_error_mean_m"), float("inf"))

    def target_max(row: Mapping[str, object]) -> float:
        return finite_float(row.get("target_abs_mean_error_max_m"), float("inf"))

    def complexity(row: Mapping[str, object]) -> float:
        return finite_float(row.get("model_complexity_mean"), float("inf"))

    def axis_regression(row: Mapping[str, object]) -> float:
        return finite_float(row.get("axis_rmse_regression_sum_mean_m"), float("inf"))

    def pass_counts(row: Mapping[str, object]) -> tuple[int, int]:
        return (
            int(row.get("strict_pass_count", 0)),
            int(row.get("performance_pass_count", 0)),
        )

    policy_specs = [
        (
            "balanced",
            "validation RMSE first, with 1852m target mean, axis regression, and complexity penalties",
            lambda row: (
                validation_mean(row)
                + 0.25 * target_mean(row)
                + 0.10 * axis_regression(row)
                + 0.005 * complexity(row),
                validation_mean(row),
                target_max(row),
                complexity(row),
            ),
        ),
        (
            "validation_best",
            "lowest cross-scenario validation range RMSE among pass candidates",
            lambda row: (
                validation_mean(row),
                validation_max(row),
                target_max(row),
                complexity(row),
            ),
        ),
        (
            "target_guard",
            "lowest worst-case 1852m target-window error among pass candidates",
            lambda row: (
                target_max(row),
                target_mean(row),
                validation_mean(row),
                complexity(row),
            ),
        ),
        (
            "simple_stable",
            "lowest model complexity among all-scenario pass candidates, then validation and target error",
            lambda row: (
                complexity(row),
                validation_mean(row),
                target_max(row),
                target_mean(row),
            ),
        ),
    ]
    rows = []
    for policy, purpose, key_func in policy_specs:
        selected = min(
            candidates,
            key=lambda row: (
                -pass_counts(row)[0],
                -pass_counts(row)[1],
                *key_func(row),
                str(row.get("method", "")),
            ),
        )
        rows.append(
            {
                "policy": policy,
                "purpose": purpose,
                "method": selected["method"],
                **method_contract_fields(str(selected["method"])),
                "strict_pass_count": selected["strict_pass_count"],
                "performance_pass_count": selected["performance_pass_count"],
                "expected_scenario_count": selected["expected_scenario_count"],
                "strict_pass_all_scenarios": selected["strict_pass_all_scenarios"],
                "performance_pass_all_scenarios": selected["performance_pass_all_scenarios"],
                "validation_range_rmse_mean_m": selected["validation_range_rmse_mean_m"],
                "validation_range_rmse_max_m": selected["validation_range_rmse_max_m"],
                "target_abs_mean_error_mean_m": selected["target_abs_mean_error_mean_m"],
                "target_abs_mean_error_max_m": selected["target_abs_mean_error_max_m"],
                "axis_rmse_regression_sum_mean_m": selected["axis_rmse_regression_sum_mean_m"],
                "model_complexity_mean": selected["model_complexity_mean"],
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
            }
        )
    return rows


def collect_policy_profile_candidates(
    *,
    output_dir: Path,
    scenarios: Sequence[Scenario],
    policy_rows: Sequence[Mapping[str, object]],
) -> list[dict[str, object]]:
    policy_by_method = {
        str(row["policy"]): row
        for row in policy_rows
        if str(row.get("method", ""))
    }
    candidates: list[dict[str, object]] = []
    for policy, policy_row in policy_by_method.items():
        method = str(policy_row["method"])
        target_validation = finite_float(policy_row.get("validation_range_rmse_mean_m"), float("inf"))
        target_1852 = finite_float(policy_row.get("target_abs_mean_error_mean_m"), float("inf"))
        for scenario in scenarios:
            scenario_dir = output_dir / scenario.name
            for row in read_csv(scenario_dir / "per_seed_method_comparison.csv"):
                if str(row.get("method")) != method:
                    continue
                seed = int(float(row.get("seed", 0) or 0))
                profile_path = scenario_dir / f"seed_{seed}" / "runtime_profiles" / f"{method}.json"
                validation_rmse = finite_float(row.get("validation_range_rmse_m"), float("inf"))
                target_abs = finite_float(row.get("validation_target_abs_mean_error_m"), float("inf"))
                profile_score = (
                    abs(validation_rmse - target_validation)
                    + 0.25 * abs(target_abs - target_1852)
                    + (0.0 if profile_path.exists() else 1.0e6)
                )
                candidates.append(
                    {
                        "policy": policy,
                        "method": method,
                        **method_contract_fields(method),
                        "scenario": scenario.name,
                        "seed": seed,
                        "profile_path": str(profile_path.relative_to(output_dir)) if profile_path.exists() else "",
                        "profile_exists": profile_path.exists(),
                        "validation_range_rmse_m": validation_rmse,
                        "validation_target_abs_mean_error_m": target_abs,
                        "validation_position_rmse_m": finite_float(row.get("validation_position_rmse_m"), float("inf")),
                        "validation_right_rmse_m": finite_float(row.get("validation_right_rmse_m"), float("inf")),
                        "validation_down_rmse_m": finite_float(row.get("validation_down_rmse_m"), float("inf")),
                        "validation_forward_rmse_m": finite_float(row.get("validation_forward_rmse_m"), float("inf")),
                        "model_complexity": finite_float(row.get("model_complexity"), float("inf")),
                        "profile_selection_score": profile_score,
                        "axis_contract": RDF_AXIS_CONTRACT,
                        "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
                    }
                )
    return candidates


def choose_policy_representative_profiles(
    *,
    output_dir: Path,
    policy_rows: Sequence[Mapping[str, object]],
    profile_candidates: Sequence[Mapping[str, object]],
) -> list[dict[str, object]]:
    selected_rows: list[dict[str, object]] = []
    for policy_row in policy_rows:
        policy = str(policy_row["policy"])
        candidates = [
            row for row in profile_candidates
            if str(row.get("policy")) == policy
            and bool(row.get("profile_exists"))
            and math.isfinite(finite_float(row.get("validation_range_rmse_m"), float("nan")))
        ]
        if not candidates:
            selected_rows.append(
                {
                    "policy": policy,
                    "method": policy_row.get("method", ""),
                    **method_contract_fields(str(policy_row.get("method", ""))),
                    "selected": False,
                    "reason": "no_existing_runtime_profile_candidates",
                    "axis_contract": RDF_AXIS_CONTRACT,
                }
            )
            continue
        selected = min(
            candidates,
            key=lambda row: (
                finite_float(row.get("profile_selection_score"), float("inf")),
                finite_float(row.get("validation_range_rmse_m"), float("inf")),
                finite_float(row.get("validation_target_abs_mean_error_m"), float("inf")),
                str(row.get("scenario", "")),
                int(row.get("seed", 0)),
            ),
        )
        source_profile = output_dir / str(selected["profile_path"])
        profile_output_dir = output_dir / "robust_policy_profiles"
        profile_output_dir.mkdir(parents=True, exist_ok=True)
        copied_profile = profile_output_dir / f"{policy}_runtime_profile.json"
        shutil.copyfile(source_profile, copied_profile)
        selected_rows.append(
            {
                "policy": policy,
                "method": selected["method"],
                **method_contract_fields(str(selected["method"])),
                "selected": True,
                "reason": "closest_seed_profile_to_policy_mean_metrics",
                "source_scenario": selected["scenario"],
                "source_seed": selected["seed"],
                "source_profile_path": selected["profile_path"],
                "representative_profile_path": str(copied_profile.relative_to(output_dir)),
                "profile_selection_score": selected["profile_selection_score"],
                "validation_range_rmse_m": selected["validation_range_rmse_m"],
                "validation_target_abs_mean_error_m": selected["validation_target_abs_mean_error_m"],
                "validation_position_rmse_m": selected["validation_position_rmse_m"],
                "validation_right_rmse_m": selected["validation_right_rmse_m"],
                "validation_down_rmse_m": selected["validation_down_rmse_m"],
                "validation_forward_rmse_m": selected["validation_forward_rmse_m"],
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": selected["runtime_input_contract"],
            }
        )
    return selected_rows


def run_policy_profile_transfer_replay(
    *,
    output_dir: Path,
    scenarios: Sequence[Scenario],
    representative_profiles: Sequence[Mapping[str, object]],
    target_range_m: float,
    target_window_m: float,
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    replay_rows: list[dict[str, object]] = []
    for profile_row in representative_profiles:
        if not bool(profile_row.get("selected")):
            continue
        policy = str(profile_row["policy"])
        profile_path = output_dir / str(profile_row["representative_profile_path"])
        for scenario in scenarios:
            scenario_dir = output_dir / scenario.name
            for run_row in read_csv(scenario_dir / "seed_runs.csv"):
                if str(run_row.get("pass_run", "")).lower() != "true":
                    continue
                seed = int(float(run_row.get("seed", 0) or 0))
                sample_csv = Path(str(run_row["run_dir"])) / "calibration_samples.csv"
                replay_dir = output_dir / "robust_policy_profile_replay" / policy / scenario.name / f"seed_{seed}"
                command = [
                    sys.executable,
                    str(PROFILE_REPLAY_SCRIPT),
                    "--profile-json",
                    str(profile_path),
                    "--input-csv",
                    str(sample_csv),
                    "--output-dir",
                    str(replay_dir),
                    "--split",
                    "validation",
                    "--target-range-m",
                    str(target_range_m),
                    "--target-window-m",
                    str(target_window_m),
                ]
                result = subprocess.run(command, cwd=ROOT_DIR)
                replay_summary_csv = replay_dir / "profile_replay_summary.csv"
                replay_summary = dict(read_csv(replay_summary_csv)[0]) if replay_summary_csv.exists() and read_csv(replay_summary_csv) else {}
                raw_rmse = finite_float(replay_summary.get("validation_range_rmse_raw_m"), float("inf"))
                corrected_rmse = finite_float(replay_summary.get("validation_range_rmse_corrected_m"), float("inf"))
                target_abs = finite_float(replay_summary.get("target_abs_mean_error_corrected_m"), float("inf"))
                target_count = int(float(replay_summary.get("target_window_count", 0) or 0))
                pass_replay = (
                    result.returncode == 0
                    and math.isfinite(raw_rmse)
                    and math.isfinite(corrected_rmse)
                    and corrected_rmse < raw_rmse
                    and target_count > 0
                    and math.isfinite(target_abs)
                    and target_abs <= 10.0
                )
                replay_rows.append(
                    {
                        "policy": policy,
                        "method": profile_row["method"],
                        "source_scenario": profile_row.get("source_scenario", ""),
                        "source_seed": profile_row.get("source_seed", ""),
                        "target_scenario": scenario.name,
                        "target_seed": seed,
                        "pass_replay": pass_replay,
                        "replay_returncode": int(result.returncode),
                        "profile_path": str(profile_path.relative_to(output_dir)),
                        "sample_csv": str(sample_csv.relative_to(output_dir)) if sample_csv.is_relative_to(output_dir) else str(sample_csv),
                        "replay_output_dir": str(replay_dir.relative_to(output_dir)),
                        "validation_range_rmse_raw_m": raw_rmse,
                        "validation_range_rmse_corrected_m": corrected_rmse,
                        "validation_range_rmse_improvement_m": raw_rmse - corrected_rmse,
                        "target_window_count": target_count,
                        "target_abs_mean_error_corrected_m": target_abs,
                        "axis_contract": RDF_AXIS_CONTRACT,
                        "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
                    }
                )
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in replay_rows:
        grouped.setdefault(str(row["policy"]), []).append(row)
    summary_rows = []
    for policy, rows in grouped.items():
        corrected = [finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")) for row in rows]
        target_abs = [finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")) for row in rows]
        improvements = [finite_float(row.get("validation_range_rmse_improvement_m"), float("-inf")) for row in rows]
        finite_corrected = [value for value in corrected if math.isfinite(value)]
        finite_target = [value for value in target_abs if math.isfinite(value)]
        finite_improvements = [value for value in improvements if math.isfinite(value)]
        summary_rows.append(
            {
                "policy": policy,
                "method": rows[0]["method"],
                "transfer_replay_count": len(rows),
                "transfer_replay_pass_count": sum(1 for row in rows if bool(row.get("pass_replay"))),
                "pass_all_transfer_replays": bool(rows) and all(bool(row.get("pass_replay")) for row in rows),
                "validation_range_rmse_corrected_mean_m": float(np.mean(finite_corrected)) if finite_corrected else float("inf"),
                "validation_range_rmse_corrected_max_m": float(np.max(finite_corrected)) if finite_corrected else float("inf"),
                "validation_range_rmse_improvement_min_m": float(np.min(finite_improvements)) if finite_improvements else float("-inf"),
                "target_abs_mean_error_corrected_mean_m": float(np.mean(finite_target)) if finite_target else float("inf"),
                "target_abs_mean_error_corrected_max_m": float(np.max(finite_target)) if finite_target else float("inf"),
                "axis_contract": RDF_AXIS_CONTRACT,
            }
        )
    summary_rows.sort(
        key=lambda row: (
            not bool(row["pass_all_transfer_replays"]),
            float(row["validation_range_rmse_corrected_mean_m"]),
            float(row["target_abs_mean_error_corrected_max_m"]),
            str(row["policy"]),
        )
    )
    return replay_rows, summary_rows


def build_pooled_calibration_rows(
    *,
    output_dir: Path,
    scenarios: Sequence[Scenario],
) -> list[dict[str, object]]:
    pooled_rows: list[dict[str, object]] = []
    pooled_sample_id = 0
    for scenario in scenarios:
        scenario_dir = output_dir / scenario.name
        for run_row in read_csv(scenario_dir / "seed_runs.csv"):
            if str(run_row.get("pass_run", "")).lower() != "true":
                continue
            seed = str(run_row.get("seed", ""))
            sample_csv = Path(str(run_row["run_dir"])) / "calibration_samples.csv"
            for sample_row in read_csv(sample_csv):
                pooled = dict(sample_row)
                pooled["sample_id"] = pooled_sample_id
                pooled["source_scenario"] = scenario.name
                pooled["source_seed"] = seed
                pooled_rows.append(pooled)
                pooled_sample_id += 1
    return pooled_rows


def pooled_universal_profile_sort_key(row: Mapping[str, object]) -> tuple[object, ...]:
    return (
        not bool(row.get("pass_all_transfer_replays")),
        -int(row.get("transfer_replay_pass_count", 0) or 0),
        finite_float(row.get("target_abs_mean_error_corrected_max_m"), float("inf")),
        finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf")),
        finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
        finite_float(row.get("validation_range_rmse_improvement_min_m"), float("-inf")) <= 0.0,
        int(row.get("model_complexity", 9999) or 9999),
        str(row.get("method", "")),
    )


def run_pooled_calibration_profile(
    *,
    args: argparse.Namespace,
    output_dir: Path,
    scenarios: Sequence[Scenario],
) -> tuple[list[dict[str, object]], list[dict[str, object]], dict[str, object]]:
    pooled_dir = output_dir / "pooled_scenario_profile"
    pooled_csv = pooled_dir / "pooled_calibration_samples.csv"
    pooled_run_dir = pooled_dir / "pipeline"
    pooled_rows = build_pooled_calibration_rows(output_dir=output_dir, scenarios=scenarios)
    write_csv(pooled_csv, pooled_rows)
    seed = str(args.seeds).split(",")[0].strip() or "20260613"
    command = [
        sys.executable,
        str(PIPELINE_SCRIPT),
        "--mode",
        "csv",
        "--input-csv",
        str(pooled_csv),
        "--output-dir",
        str(pooled_run_dir),
        "--seed",
        seed,
        "--train-ratio",
        str(args.train_ratio),
        "--target-range-m",
        str(args.target_range_m),
        "--target-window-m",
        str(args.target_window_m),
    ]
    result = subprocess.run(command, cwd=ROOT_DIR)
    comparison_rows = read_csv(pooled_run_dir / "method_comparison.csv")
    runtime_dir = pooled_run_dir / "runtime_profiles"
    candidate_by_method: dict[str, dict[str, object]] = {}
    candidate_sources: dict[str, set[str]] = {}

    def add_candidate(row: Mapping[str, object], *, source: str) -> None:
        method_name = str(row.get("method", ""))
        if not method_name or method_name == "raw":
            return
        profile_path = runtime_dir / f"{method_name}.json"
        if not profile_path.exists():
            return
        if method_name not in candidate_by_method:
            candidate_by_method[method_name] = {
                    "method": method_name,
                    "profile_path": profile_path,
                    "source": source,
                    "pooled_validation_range_rmse_m": finite_float(row.get("validation_range_rmse_m"), float("inf")),
                    "pooled_target_abs_mean_error_m": finite_float(row.get("validation_target_abs_mean_error_m"), float("inf")),
                    "pooled_generalization_gap_range_rmse_m": finite_float(
                        row.get("generalization_gap_range_rmse_m"),
                        float("inf"),
                    ),
                    "model_complexity": int(float(row.get("model_complexity", 9999) or 9999)),
                }
            candidate_sources[method_name] = {source}
        else:
            candidate_sources.setdefault(method_name, set()).add(source)
            candidate_by_method[method_name]["source"] = "+".join(sorted(candidate_sources[method_name]))

    non_raw_rows = [row for row in comparison_rows if str(row.get("method", "")) != "raw"]
    for row in sorted(non_raw_rows, key=lambda item: finite_float(item.get("validation_range_rmse_m"), float("inf")))[:10]:
        add_candidate(row, source="pooled_validation_top")
    for row in sorted(non_raw_rows, key=lambda item: finite_float(item.get("validation_target_abs_mean_error_m"), float("inf")))[:8]:
        add_candidate(row, source="pooled_target_top")
    for row in sorted(
        non_raw_rows,
        key=lambda item: (
            int(float(item.get("model_complexity", 9999) or 9999)),
            finite_float(item.get("validation_range_rmse_m"), float("inf")),
        ),
    )[:6]:
        add_candidate(row, source="pooled_low_complexity_top")
    for row in sorted(
        non_raw_rows,
        key=lambda item: (
            int(float(item.get("model_complexity", 9999) or 9999)),
            finite_float(item.get("validation_target_abs_mean_error_m"), float("inf")),
            finite_float(item.get("validation_range_rmse_m"), float("inf")),
        ),
    )[:30]:
        add_candidate(row, source="pooled_low_complexity_target_guard")
    for row in sorted(
        non_raw_rows,
        key=lambda item: (
            finite_float(item.get("validation_range_rmse_m"), float("inf"))
            + 0.35 * finite_float(item.get("validation_target_abs_mean_error_m"), float("inf"))
            + 0.15 * max(0.0, finite_float(item.get("generalization_gap_range_rmse_m"), 0.0))
            + 0.03 * int(float(item.get("model_complexity", 9999) or 9999)),
            int(float(item.get("model_complexity", 9999) or 9999)),
            str(item.get("method", "")),
        ),
    )[:40]:
        add_candidate(row, source="pooled_combined_guard_score")
    recommendation_json = pooled_run_dir / "policy_recommendations.json"
    if recommendation_json.exists():
        try:
            recommendation_payload = json.loads(recommendation_json.read_text(encoding="utf-8"))
            recommendations = (
                recommendation_payload.get("policies", [])
                if isinstance(recommendation_payload, Mapping)
                else recommendation_payload
            )
            comparison_by_method = {str(row.get("method", "")): row for row in comparison_rows}
            for policy_row in recommendations:
                if not isinstance(policy_row, Mapping):
                    continue
                method_name = str(policy_row.get("method", ""))
                if method_name in comparison_by_method:
                    add_candidate(comparison_by_method[method_name], source=f"policy:{policy_row.get('policy', '')}")
        except (json.JSONDecodeError, OSError):
            pass
    candidate_limit = max(0, int(getattr(args, "pooled_candidate_scan_limit", 64)))
    if candidate_limit > 0 and len(candidate_by_method) > candidate_limit:
        ranked_candidates = sorted(
            candidate_by_method.values(),
            key=lambda item: (
                finite_float(item.get("pooled_validation_range_rmse_m"), float("inf"))
                + 0.35 * finite_float(item.get("pooled_target_abs_mean_error_m"), float("inf"))
                + 0.15 * max(0.0, finite_float(item.get("pooled_generalization_gap_range_rmse_m"), 0.0))
                + 0.03 * int(item.get("model_complexity", 9999) or 9999),
                int(item.get("model_complexity", 9999) or 9999),
                str(item.get("method", "")),
            ),
        )
        keep_methods = {str(item["method"]) for item in ranked_candidates[:candidate_limit]}
        candidate_by_method = {
            method: candidate
            for method, candidate in candidate_by_method.items()
            if method in keep_methods
        }

    replay_rows: list[dict[str, object]] = []
    candidate_summaries: list[dict[str, object]] = []
    for candidate in candidate_by_method.values():
        method = str(candidate["method"])
        profile_path = Path(candidate["profile_path"])
        candidate_rows: list[dict[str, object]] = []
        for scenario in scenarios:
            scenario_dir = output_dir / scenario.name
            for run_row in read_csv(scenario_dir / "seed_runs.csv"):
                if str(run_row.get("pass_run", "")).lower() != "true":
                    continue
                target_seed = int(float(run_row.get("seed", 0) or 0))
                sample_csv = Path(str(run_row["run_dir"])) / "calibration_samples.csv"
                replay_dir = pooled_dir / "candidate_transfer_replay" / method / scenario.name / f"seed_{target_seed}"
                replay_command = [
                    sys.executable,
                    str(PROFILE_REPLAY_SCRIPT),
                    "--profile-json",
                    str(profile_path),
                    "--input-csv",
                    str(sample_csv),
                    "--output-dir",
                    str(replay_dir),
                    "--split",
                    "validation",
                    "--target-range-m",
                    str(args.target_range_m),
                    "--target-window-m",
                    str(args.target_window_m),
                ]
                replay_result = subprocess.run(replay_command, cwd=ROOT_DIR)
                replay_summary_csv = replay_dir / "profile_replay_summary.csv"
                replay_summary = dict(read_csv(replay_summary_csv)[0]) if replay_summary_csv.exists() and read_csv(replay_summary_csv) else {}
                raw_rmse = finite_float(replay_summary.get("validation_range_rmse_raw_m"), float("inf"))
                corrected_rmse = finite_float(replay_summary.get("validation_range_rmse_corrected_m"), float("inf"))
                target_abs = finite_float(replay_summary.get("target_abs_mean_error_corrected_m"), float("inf"))
                target_count = int(float(replay_summary.get("target_window_count", 0) or 0))
                pass_replay = (
                    replay_result.returncode == 0
                    and math.isfinite(raw_rmse)
                    and math.isfinite(corrected_rmse)
                    and corrected_rmse < raw_rmse
                    and target_count > 0
                    and math.isfinite(target_abs)
                    and target_abs <= 10.0
                )
                replay_rows.append(
                    {
                        "profile_kind": "pooled_scenario_profile",
                        "method": method,
                        "candidate_source": candidate["source"],
                        "target_scenario": scenario.name,
                        "target_seed": target_seed,
                        "pass_replay": pass_replay,
                        "replay_returncode": int(replay_result.returncode),
                        "profile_path": str(profile_path.relative_to(output_dir)),
                        "sample_csv": str(sample_csv.relative_to(output_dir)) if sample_csv.is_relative_to(output_dir) else str(sample_csv),
                        "replay_output_dir": str(replay_dir.relative_to(output_dir)),
                        "validation_range_rmse_raw_m": raw_rmse,
                        "validation_range_rmse_corrected_m": corrected_rmse,
                        "validation_range_rmse_improvement_m": raw_rmse - corrected_rmse,
                        "target_window_count": target_count,
                        "target_abs_mean_error_corrected_m": target_abs,
                        "axis_contract": RDF_AXIS_CONTRACT,
                        "runtime_input_contract": "pooled profile replay uses raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
                    }
                )
                candidate_rows.append(replay_rows[-1])
        corrected = [finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")) for row in candidate_rows]
        target_abs = [finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")) for row in candidate_rows]
        improvements = [finite_float(row.get("validation_range_rmse_improvement_m"), float("-inf")) for row in candidate_rows]
        finite_corrected = [value for value in corrected if math.isfinite(value)]
        finite_target = [value for value in target_abs if math.isfinite(value)]
        finite_improvements = [value for value in improvements if math.isfinite(value)]
        candidate_summaries.append(
            {
                "method": method,
                "candidate_source": candidate["source"],
                "profile_path": str(profile_path.relative_to(output_dir)),
                "pooled_validation_range_rmse_m": candidate["pooled_validation_range_rmse_m"],
                "pooled_target_abs_mean_error_m": candidate["pooled_target_abs_mean_error_m"],
                "model_complexity": candidate["model_complexity"],
                "transfer_replay_count": len(candidate_rows),
                "transfer_replay_pass_count": sum(1 for row in candidate_rows if bool(row.get("pass_replay"))),
                "pass_all_transfer_replays": bool(candidate_rows) and all(bool(row.get("pass_replay")) for row in candidate_rows),
                "validation_range_rmse_corrected_mean_m": float(np.mean(finite_corrected)) if finite_corrected else float("inf"),
                "validation_range_rmse_corrected_max_m": float(np.max(finite_corrected)) if finite_corrected else float("inf"),
                "validation_range_rmse_improvement_min_m": float(np.min(finite_improvements)) if finite_improvements else float("-inf"),
                "target_abs_mean_error_corrected_mean_m": float(np.mean(finite_target)) if finite_target else float("inf"),
                "target_abs_mean_error_corrected_max_m": float(np.max(finite_target)) if finite_target else float("inf"),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "pooled candidate transfer diagnostic; runtime uses raw RDF/disparity only",
            }
        )
    candidate_summaries.sort(
        key=pooled_universal_profile_sort_key
    )
    selected_candidate = candidate_summaries[0] if candidate_summaries else {}
    selected_profile_path = output_dir / str(selected_candidate.get("profile_path", ""))
    selected_profile_exists = bool(selected_candidate) and selected_profile_path.is_file()
    promotion_dir = pooled_dir / "promotion_check"
    promotion_command = [
        sys.executable,
        str(PROFILE_PROMOTION_CHECK_SCRIPT),
        "--profile-json",
        str(selected_profile_path),
        "--sample-csv",
        str(pooled_csv),
        "--output-dir",
        str(promotion_dir),
        "--split",
        "validation",
        "--max-samples",
        "64",
    ]
    promotion_result = subprocess.run(promotion_command, cwd=ROOT_DIR) if selected_profile_exists else None
    promotion_json = promotion_dir / "promotion_check.json"
    promotion_summary = json.loads(promotion_json.read_text(encoding="utf-8")) if promotion_json.exists() else {}
    selected_method = selected_candidate.get("method")
    selected_replay_rows = [row for row in replay_rows if row.get("method") == selected_method]
    corrected = [finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")) for row in selected_replay_rows]
    target_abs = [finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")) for row in selected_replay_rows]
    improvements = [finite_float(row.get("validation_range_rmse_improvement_m"), float("-inf")) for row in selected_replay_rows]
    finite_corrected = [value for value in corrected if math.isfinite(value)]
    finite_target = [value for value in target_abs if math.isfinite(value)]
    finite_improvements = [value for value in improvements if math.isfinite(value)]
    summary = {
        "selected": bool(selected_candidate),
        "method": selected_candidate.get("method", ""),
        "candidate_source": selected_candidate.get("candidate_source", ""),
        "pipeline_returncode": int(result.returncode),
        "profile_path": selected_candidate.get("profile_path", ""),
        "pooled_csv": str(pooled_csv.relative_to(output_dir)),
        "pooled_sample_count": len(pooled_rows),
        "candidate_profile_count": len(candidate_summaries),
        "promotion_returncode": int(promotion_result.returncode) if promotion_result is not None else 1,
        "pass_promotion_check": bool(promotion_summary.get("pass_promotion_check", False)),
        "schema_load_ok": bool(promotion_summary.get("schema_load_ok", False)),
        "node_loader_ok": bool(promotion_summary.get("node_loader_ok", False)),
        "runtime_truth_used": bool(promotion_summary.get("runtime_truth_used", True)),
        "forbidden_runtime_inputs_declared": bool(promotion_summary.get("forbidden_runtime_inputs_declared", False)),
        "transfer_replay_count": len(selected_replay_rows),
        "transfer_replay_pass_count": sum(1 for row in selected_replay_rows if bool(row.get("pass_replay"))),
        "pass_all_transfer_replays": bool(selected_replay_rows) and all(bool(row.get("pass_replay")) for row in selected_replay_rows),
        "validation_range_rmse_corrected_mean_m": float(np.mean(finite_corrected)) if finite_corrected else float("inf"),
        "validation_range_rmse_corrected_max_m": float(np.max(finite_corrected)) if finite_corrected else float("inf"),
        "validation_range_rmse_improvement_min_m": float(np.min(finite_improvements)) if finite_improvements else float("-inf"),
        "target_abs_mean_error_corrected_mean_m": float(np.mean(finite_target)) if finite_target else float("inf"),
        "target_abs_mean_error_corrected_max_m": float(np.max(finite_target)) if finite_target else float("inf"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "runtime_input_contract": "single profile fitted from pooled scenario calibration data; runtime uses raw RDF/disparity only",
        "training_contract": "Calibration-time PX4/RTK truth from multiple scenarios is pooled for fitting/evaluation only.",
        "selection_objective": (
            "pass all transfer replays, then minimize 1852m target worst-case error, "
            "validation worst-case RMSE, validation mean RMSE, and model complexity"
        ),
        "candidate_summaries": candidate_summaries[:20],
    }
    summary_rows = candidate_summaries
    return replay_rows, summary_rows, summary


def transfer_failure_diagnostic_rows(
    replay_rows: Sequence[Mapping[str, object]],
) -> tuple[list[dict[str, object]], dict[str, object]]:
    grouped: dict[tuple[str, str], list[Mapping[str, object]]] = {}
    for row in replay_rows:
        grouped.setdefault((str(row.get("policy", "")), str(row.get("target_scenario", ""))), []).append(row)
    rows_out: list[dict[str, object]] = []
    for (policy, scenario), rows in sorted(grouped.items()):
        improvements = [finite_float(row.get("validation_range_rmse_improvement_m"), float("nan")) for row in rows]
        target_abs = [finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan")) for row in rows]
        raw_rmse = [finite_float(row.get("validation_range_rmse_raw_m"), float("nan")) for row in rows]
        corrected_rmse = [finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")) for row in rows]
        finite_improvements = [value for value in improvements if math.isfinite(value)]
        finite_target = [value for value in target_abs if math.isfinite(value)]
        finite_raw = [value for value in raw_rmse if math.isfinite(value)]
        finite_corrected = [value for value in corrected_rmse if math.isfinite(value)]
        failed_rows = [row for row in rows if not bool(row.get("pass_replay"))]
        reasons: set[str] = set()
        for row in failed_rows:
            returncode = int(row.get("replay_returncode", 1) or 1)
            improvement = finite_float(row.get("validation_range_rmse_improvement_m"), float("nan"))
            target_count = int(row.get("target_window_count", 0) or 0)
            target_value = finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan"))
            if returncode != 0:
                reasons.add("replay_gate_failed")
            if not math.isfinite(improvement) or improvement <= 0.0:
                reasons.add("no_validation_rmse_improvement")
            if target_count <= 0:
                reasons.add("missing_target_window_validation_samples")
            if not math.isfinite(target_value) or target_value > 10.0:
                reasons.add("target_error_over_10m")
        if not reasons and failed_rows:
            reasons.add("unknown_failure")
        pass_count = sum(1 for row in rows if bool(row.get("pass_replay")))
        fail_count = len(rows) - pass_count
        rows_out.append(
            {
                "policy": policy,
                "target_scenario": scenario,
                "source_scenario": rows[0].get("source_scenario", ""),
                "method": rows[0].get("method", ""),
                "replay_count": len(rows),
                "pass_count": pass_count,
                "fail_count": fail_count,
                "pass_all": fail_count == 0 and bool(rows),
                "min_validation_range_rmse_improvement_m": float(np.min(finite_improvements)) if finite_improvements else float("nan"),
                "mean_validation_range_rmse_raw_m": float(np.mean(finite_raw)) if finite_raw else float("nan"),
                "mean_validation_range_rmse_corrected_m": float(np.mean(finite_corrected)) if finite_corrected else float("nan"),
                "max_target_abs_mean_error_corrected_m": float(np.max(finite_target)) if finite_target else float("nan"),
                "failure_reasons": ";".join(sorted(reasons)),
                "recommended_action": (
                    "use_scenario_catalog_profile"
                    if fail_count
                    else "universal_profile_transfer_ok_for_scenario"
                ),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "diagnostic only; runtime uses raw stereo-center RDF(right,down,forward) plus measured disparity and no truth",
            }
        )
    failed_policy_scenarios = [row for row in rows_out if int(row["fail_count"]) > 0]
    summary = {
        "policy_scenario_count": len(rows_out),
        "failed_policy_scenario_count": len(failed_policy_scenarios),
        "pass_all_policy_scenarios": bool(rows_out) and not failed_policy_scenarios,
        "worst_negative_improvement_m": (
            float(np.min([float(row["min_validation_range_rmse_improvement_m"]) for row in rows_out]))
            if rows_out
            else float("nan")
        ),
        "worst_target_abs_mean_error_m": (
            float(np.max([float(row["max_target_abs_mean_error_corrected_m"]) for row in rows_out]))
            if rows_out
            else float("nan")
        ),
        "axis_contract": RDF_AXIS_CONTRACT,
        "runtime_input_contract": "diagnostic summary; no runtime truth input is required or permitted",
        "rows": rows_out,
    }
    return rows_out, summary


def pooled_profile_failure_diagnostics(
    replay_rows: Sequence[Mapping[str, object]],
    candidate_summary_rows: Sequence[Mapping[str, object]],
    selected_summary: Mapping[str, object],
) -> tuple[list[dict[str, object]], dict[str, object]]:
    scenarios = sorted({str(row.get("target_scenario", "")) for row in replay_rows if row.get("target_scenario")})
    selected_method = str(selected_summary.get("method", ""))
    rows_out: list[dict[str, object]] = []
    for scenario in scenarios:
        scenario_rows = [row for row in replay_rows if str(row.get("target_scenario", "")) == scenario]
        passing_rows = [row for row in scenario_rows if parse_bool(row.get("pass_replay"))]
        failing_rows = [row for row in scenario_rows if not parse_bool(row.get("pass_replay"))]
        selected_rows = [row for row in scenario_rows if str(row.get("method", "")) == selected_method]
        selected_row = selected_rows[0] if selected_rows else {}
        reasons: set[str] = set()
        for row in failing_rows:
            returncode = int(row.get("replay_returncode", 1) or 1)
            improvement = finite_float(row.get("validation_range_rmse_improvement_m"), float("nan"))
            target_count = int(float(row.get("target_window_count", 0) or 0))
            target_abs = finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan"))
            if returncode != 0:
                reasons.add("replay_gate_failed")
            if not math.isfinite(improvement) or improvement <= 0.0:
                reasons.add("no_validation_rmse_improvement")
            if target_count <= 0:
                reasons.add("missing_target_window_validation_samples")
            if not math.isfinite(target_abs) or target_abs > 10.0:
                reasons.add("target_error_over_10m")
        best_passing = min(
            passing_rows,
            key=lambda row: (
                finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")),
                finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")),
                str(row.get("method", "")),
            ),
            default={},
        )
        best_any = min(
            scenario_rows,
            key=lambda row: (
                not parse_bool(row.get("pass_replay")),
                finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")),
                finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")),
                str(row.get("method", "")),
            ),
            default={},
        )
        if not passing_rows:
            recommended_action = "use_catalog_or_minimal_set_profile_for_this_scenario"
        elif selected_row and not parse_bool(selected_row.get("pass_replay")):
            recommended_action = "do_not_promote_selected_pooled_profile_for_this_scenario"
        else:
            recommended_action = "pooled_profile_candidate_available"
        rows_out.append(
            {
                "target_scenario": scenario,
                "candidate_count": len(scenario_rows),
                "pass_count": len(passing_rows),
                "fail_count": len(failing_rows),
                "pass_any_pooled_candidate": bool(passing_rows),
                "selected_method": selected_method,
                "selected_pass_replay": parse_bool(selected_row.get("pass_replay")) if selected_row else False,
                "selected_validation_range_rmse_raw_m": finite_float(selected_row.get("validation_range_rmse_raw_m"), float("nan")) if selected_row else float("nan"),
                "selected_validation_range_rmse_corrected_m": finite_float(selected_row.get("validation_range_rmse_corrected_m"), float("nan")) if selected_row else float("nan"),
                "selected_validation_range_rmse_improvement_m": finite_float(selected_row.get("validation_range_rmse_improvement_m"), float("nan")) if selected_row else float("nan"),
                "selected_target_abs_mean_error_corrected_m": finite_float(selected_row.get("target_abs_mean_error_corrected_m"), float("nan")) if selected_row else float("nan"),
                "best_passing_method": best_passing.get("method", ""),
                "best_passing_validation_range_rmse_corrected_m": finite_float(best_passing.get("validation_range_rmse_corrected_m"), float("nan")) if best_passing else float("nan"),
                "best_passing_target_abs_mean_error_corrected_m": finite_float(best_passing.get("target_abs_mean_error_corrected_m"), float("nan")) if best_passing else float("nan"),
                "best_any_method": best_any.get("method", ""),
                "best_any_pass_replay": parse_bool(best_any.get("pass_replay")) if best_any else False,
                "failure_reasons": ",".join(sorted(reasons)) if reasons else "",
                "recommended_action": recommended_action,
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "pooled replay diagnostic; runtime profile application uses raw RDF/disparity only",
            }
        )
    selected_fail_scenarios = [
        row["target_scenario"]
        for row in rows_out
        if str(row.get("selected_method", "")) == selected_method and not parse_bool(row.get("selected_pass_replay"))
    ]
    no_pooled_pass_scenarios = [
        row["target_scenario"]
        for row in rows_out
        if not parse_bool(row.get("pass_any_pooled_candidate"))
    ]
    candidate_pass_all_count = sum(1 for row in candidate_summary_rows if parse_bool(row.get("pass_all_transfer_replays")))
    summary = {
        "scenario_count": len(rows_out),
        "candidate_profile_count": len(candidate_summary_rows),
        "candidate_pass_all_count": candidate_pass_all_count,
        "selected_method": selected_method,
        "selected_pass_all_transfer_replays": parse_bool(selected_summary.get("pass_all_transfer_replays")),
        "selected_transfer_replay_pass_count": int(selected_summary.get("transfer_replay_pass_count", 0) or 0),
        "selected_transfer_replay_count": int(selected_summary.get("transfer_replay_count", 0) or 0),
        "selected_failed_scenarios": selected_fail_scenarios,
        "no_pooled_candidate_pass_scenarios": no_pooled_pass_scenarios,
        "recommended_action": (
            "promote_selected_pooled_profile"
            if parse_bool(selected_summary.get("pass_all_transfer_replays"))
            else "use_catalog_or_minimal_set_for_failed_scenarios"
        ),
        "rows": rows_out,
        "axis_contract": RDF_AXIS_CONTRACT,
        "runtime_input_contract": "pooled replay diagnostic; runtime profile application uses raw RDF/disparity only",
    }
    return rows_out, summary


def collect_scenario_profile_catalog(
    *,
    output_dir: Path,
    scenarios: Sequence[Scenario],
) -> tuple[list[dict[str, object]], dict[str, object]]:
    catalog_dir = output_dir / "scenario_profile_catalog"
    catalog_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    for scenario in scenarios:
        scenario_dir = output_dir / scenario.name
        profile_path = scenario_dir / "representative_runtime_profile.json"
        representative_path = scenario_dir / "multi_seed_representative_profile.json"
        recommendation_path = scenario_dir / "multi_seed_recommendation.json"
        cross_seed_path = scenario_dir / "representative_cross_seed_replay_summary.json"
        representative = json.loads(representative_path.read_text(encoding="utf-8")) if representative_path.exists() else {}
        recommendation = json.loads(recommendation_path.read_text(encoding="utf-8")) if recommendation_path.exists() else {}
        cross_seed = json.loads(cross_seed_path.read_text(encoding="utf-8")) if cross_seed_path.exists() else {}
        copied_profile_path = catalog_dir / f"{scenario.name}_runtime_profile.json"
        profile_exists = profile_path.exists()
        if profile_exists:
            shutil.copyfile(profile_path, copied_profile_path)
        rows.append(
            {
                "scenario": scenario.name,
                "purpose": scenario.purpose,
                "method": recommendation.get("method", representative.get("method", "")),
                **method_contract_fields(str(recommendation.get("method", representative.get("method", "")))),
                "recommendation_mode": recommendation.get("recommendation_mode", ""),
                "source_seed": representative.get("seed", ""),
                "source_profile_path": str(profile_path.relative_to(output_dir)) if profile_exists else "",
                "catalog_profile_path": str(copied_profile_path.relative_to(output_dir)) if profile_exists else "",
                "profile_exists": profile_exists,
                "pass_cross_seed_replay": bool(cross_seed.get("pass_cross_seed_replay", False)),
                "passed_seed_count": int(cross_seed.get("passed_seed_count", 0) or 0),
                "seed_count": int(cross_seed.get("seed_count", 0) or 0),
                "validation_range_rmse_mean_m": recommendation.get("validation_range_rmse_mean_m", ""),
                "target_abs_mean_error_max_m": recommendation.get("target_abs_mean_error_max_m", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "scenario-specific raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
            }
        )
    pass_all = bool(rows) and all(bool(row["profile_exists"]) and bool(row["pass_cross_seed_replay"]) for row in rows)
    summary = {
        "catalog_profile_count": len(rows),
        "profile_exists_count": sum(1 for row in rows if bool(row["profile_exists"])),
        "cross_seed_pass_count": sum(1 for row in rows if bool(row["pass_cross_seed_replay"])),
        "pass_scenario_profile_catalog": pass_all,
        "catalog_dir": str(catalog_dir.relative_to(output_dir)),
        "axis_contract": RDF_AXIS_CONTRACT,
        "runtime_input_contract": "select the profile calibrated for the current environment/scenario; runtime does not use truth",
        "rows": rows,
    }
    write_json(
        catalog_dir / "manifest.json",
        {
            "mode": "scenario_profile_catalog",
            "selection_contract": (
                "Choose the profile calibrated for the active environment/scenario. "
                "Do not choose a profile by runtime truth; use deployment metadata or explicit operator configuration."
            ),
            "axis_contract": RDF_AXIS_CONTRACT,
            "runtime_input_contract": summary["runtime_input_contract"],
            "profiles": rows,
        },
    )
    return rows, summary


def run_scenario_catalog_promotion_checks(
    *,
    output_dir: Path,
    catalog_rows: Sequence[Mapping[str, object]],
) -> tuple[list[dict[str, object]], dict[str, object]]:
    rows_out: list[dict[str, object]] = []
    for row in catalog_rows:
        scenario = str(row.get("scenario", ""))
        if not bool(row.get("profile_exists")):
            rows_out.append(
                {
                    "scenario": scenario,
                    "method": row.get("method", ""),
                    "pass_promotion_check": False,
                    "reason": "catalog_profile_missing",
                    "axis_contract": RDF_AXIS_CONTRACT,
                }
            )
            continue
        profile_path = output_dir / str(row["catalog_profile_path"])
        source_seed = row.get("source_seed", "")
        sample_csv = output_dir / scenario / f"seed_{source_seed}" / "calibration_samples.csv"
        if not sample_csv.exists():
            run_rows = read_csv(output_dir / scenario / "seed_runs.csv")
            first_pass = next((run_row for run_row in run_rows if str(run_row.get("pass_run", "")).lower() == "true"), None)
            sample_csv = Path(str(first_pass["run_dir"])) / "calibration_samples.csv" if first_pass else sample_csv
        promotion_dir = output_dir / "scenario_profile_catalog_promotion" / scenario
        command = [
            sys.executable,
            str(PROFILE_PROMOTION_CHECK_SCRIPT),
            "--profile-json",
            str(profile_path),
            "--output-dir",
            str(promotion_dir),
            "--split",
            "validation",
        ]
        if sample_csv.exists():
            command.extend(["--sample-csv", str(sample_csv)])
        result = subprocess.run(command, cwd=ROOT_DIR)
        promotion_json = promotion_dir / "promotion_check.json"
        promotion_summary = json.loads(promotion_json.read_text(encoding="utf-8")) if promotion_json.exists() else {}
        pass_promotion = result.returncode == 0 and bool(promotion_summary.get("pass_promotion_check"))
        rows_out.append(
            {
                "scenario": scenario,
                "method": row.get("method", ""),
                "profile_path": row.get("catalog_profile_path", ""),
                "sample_csv": str(sample_csv.relative_to(output_dir)) if sample_csv.exists() and sample_csv.is_relative_to(output_dir) else str(sample_csv),
                "promotion_output_dir": str(promotion_dir.relative_to(output_dir)),
                "promotion_returncode": int(result.returncode),
                "pass_promotion_check": pass_promotion,
                "schema_load_ok": bool(promotion_summary.get("schema_load_ok", False)),
                "node_loader_ok": bool(promotion_summary.get("node_loader_ok", False)),
                "runtime_truth_used": bool(promotion_summary.get("runtime_truth_used", True)),
                "forbidden_runtime_inputs_declared": bool(promotion_summary.get("forbidden_runtime_inputs_declared", False)),
                "sample_apply_count": int(promotion_summary.get("sample_apply_count", 0) or 0),
                "sample_invalid_count": int(promotion_summary.get("sample_invalid_count", 0) or 0),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    summary = {
        "catalog_promotion_check_count": len(rows_out),
        "catalog_promotion_pass_count": sum(1 for row in rows_out if bool(row.get("pass_promotion_check"))),
        "pass_catalog_promotion_checks": bool(rows_out) and all(bool(row.get("pass_promotion_check")) for row in rows_out),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "rows": rows_out,
    }
    return rows_out, summary


def run_catalog_profile_transfer_matrix(
    *,
    output_dir: Path,
    scenarios: Sequence[Scenario],
    catalog_rows: Sequence[Mapping[str, object]],
    target_range_m: float,
    target_window_m: float,
) -> tuple[list[dict[str, object]], list[dict[str, object]], list[dict[str, object]]]:
    replay_rows: list[dict[str, object]] = []
    for source_row in catalog_rows:
        if not bool(source_row.get("profile_exists")):
            continue
        source_scenario = str(source_row.get("scenario", ""))
        profile_path = output_dir / str(source_row.get("catalog_profile_path", ""))
        if not profile_path.exists():
            continue
        for target_scenario in scenarios:
            target_dir = output_dir / target_scenario.name
            for run_row in read_csv(target_dir / "seed_runs.csv"):
                if str(run_row.get("pass_run", "")).lower() != "true":
                    continue
                seed = int(float(run_row.get("seed", 0) or 0))
                sample_csv = Path(str(run_row["run_dir"])) / "calibration_samples.csv"
                replay_dir = (
                    output_dir
                    / "scenario_profile_catalog_transfer_replay"
                    / source_scenario
                    / target_scenario.name
                    / f"seed_{seed}"
                )
                command = [
                    sys.executable,
                    str(PROFILE_REPLAY_SCRIPT),
                    "--profile-json",
                    str(profile_path),
                    "--input-csv",
                    str(sample_csv),
                    "--output-dir",
                    str(replay_dir),
                    "--split",
                    "validation",
                    "--target-range-m",
                    str(target_range_m),
                    "--target-window-m",
                    str(target_window_m),
                ]
                result = subprocess.run(command, cwd=ROOT_DIR)
                replay_summary_csv = replay_dir / "profile_replay_summary.csv"
                replay_summary = dict(read_csv(replay_summary_csv)[0]) if replay_summary_csv.exists() and read_csv(replay_summary_csv) else {}
                raw_rmse = finite_float(replay_summary.get("validation_range_rmse_raw_m"), float("inf"))
                corrected_rmse = finite_float(replay_summary.get("validation_range_rmse_corrected_m"), float("inf"))
                target_abs = finite_float(replay_summary.get("target_abs_mean_error_corrected_m"), float("inf"))
                target_count = int(float(replay_summary.get("target_window_count", 0) or 0))
                pass_replay = (
                    result.returncode == 0
                    and math.isfinite(raw_rmse)
                    and math.isfinite(corrected_rmse)
                    and corrected_rmse < raw_rmse
                    and target_count > 0
                    and math.isfinite(target_abs)
                    and target_abs <= 10.0
                )
                replay_rows.append(
                    {
                        "source_scenario": source_scenario,
                        "source_method": source_row.get("method", ""),
                        "target_scenario": target_scenario.name,
                        "target_seed": seed,
                        "pass_replay": pass_replay,
                        "replay_returncode": int(result.returncode),
                        "profile_path": str(profile_path.relative_to(output_dir)),
                        "sample_csv": str(sample_csv.relative_to(output_dir)) if sample_csv.is_relative_to(output_dir) else str(sample_csv),
                        "replay_output_dir": str(replay_dir.relative_to(output_dir)),
                        "validation_range_rmse_raw_m": raw_rmse,
                        "validation_range_rmse_corrected_m": corrected_rmse,
                        "validation_range_rmse_improvement_m": raw_rmse - corrected_rmse,
                        "target_window_count": target_count,
                        "target_abs_mean_error_corrected_m": target_abs,
                        "axis_contract": RDF_AXIS_CONTRACT,
                        "runtime_input_contract": "catalog profile replay diagnostic; runtime uses raw stereo-center RDF(right,down,forward) plus measured disparity and no truth",
                    }
                )
    pair_groups: dict[tuple[str, str], list[Mapping[str, object]]] = {}
    source_groups: dict[str, list[Mapping[str, object]]] = {}
    for row in replay_rows:
        pair_groups.setdefault((str(row["source_scenario"]), str(row["target_scenario"])), []).append(row)
        source_groups.setdefault(str(row["source_scenario"]), []).append(row)
    pair_summary_rows: list[dict[str, object]] = []
    for (source_scenario, target_scenario), rows in sorted(pair_groups.items()):
        improvements = [finite_float(row.get("validation_range_rmse_improvement_m"), float("nan")) for row in rows]
        corrected = [finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")) for row in rows]
        target_abs = [finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan")) for row in rows]
        finite_improvements = [value for value in improvements if math.isfinite(value)]
        finite_corrected = [value for value in corrected if math.isfinite(value)]
        finite_target = [value for value in target_abs if math.isfinite(value)]
        pass_count = sum(1 for row in rows if bool(row.get("pass_replay")))
        pair_summary_rows.append(
            {
                "source_scenario": source_scenario,
                "target_scenario": target_scenario,
                "source_method": rows[0].get("source_method", ""),
                "replay_count": len(rows),
                "pass_count": pass_count,
                "fail_count": len(rows) - pass_count,
                "pass_all": bool(rows) and pass_count == len(rows),
                "min_validation_range_rmse_improvement_m": float(np.min(finite_improvements)) if finite_improvements else float("nan"),
                "mean_validation_range_rmse_corrected_m": float(np.mean(finite_corrected)) if finite_corrected else float("nan"),
                "max_target_abs_mean_error_corrected_m": float(np.max(finite_target)) if finite_target else float("nan"),
                "axis_contract": RDF_AXIS_CONTRACT,
            }
        )
    source_summary_rows: list[dict[str, object]] = []
    for source_scenario, rows in sorted(source_groups.items()):
        target_scenarios = sorted({str(row.get("target_scenario", "")) for row in rows})
        pass_count = sum(1 for row in rows if bool(row.get("pass_replay")))
        total = len(rows)
        pair_rows = [row for row in pair_summary_rows if row["source_scenario"] == source_scenario]
        covered_targets = [str(row["target_scenario"]) for row in pair_rows if bool(row.get("pass_all"))]
        source_summary_rows.append(
            {
                "source_scenario": source_scenario,
                "source_method": rows[0].get("source_method", ""),
                "target_scenario_count": len(target_scenarios),
                "covered_target_scenario_count": len(covered_targets),
                "covered_target_scenarios": ",".join(covered_targets),
                "transfer_replay_count": total,
                "transfer_replay_pass_count": pass_count,
                "pass_all_transfer_replays": bool(rows) and pass_count == total,
                "axis_contract": RDF_AXIS_CONTRACT,
            }
        )
    source_summary_rows.sort(
        key=lambda row: (
            -int(row["covered_target_scenario_count"]),
            -int(row["transfer_replay_pass_count"]),
            str(row["source_scenario"]),
        )
    )
    return replay_rows, pair_summary_rows, source_summary_rows


def run_scenario_near_tie_profile_transfer_replay(
    *,
    output_dir: Path,
    scenarios: Sequence[Scenario],
    near_tie_profiles: Sequence[Mapping[str, object]],
    target_range_m: float,
    target_window_m: float,
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    replay_rows: list[dict[str, object]] = []
    for profile_row in near_tie_profiles:
        if not bool(profile_row.get("scenario_profile_exists")):
            continue
        source_scenario = str(profile_row.get("scenario", ""))
        source_seed = int(float(profile_row.get("seed", 0) or 0))
        method = str(profile_row.get("method", ""))
        selection_rank = int(float(profile_row.get("selection_rank", 999999) or 999999))
        profile_path = output_dir / str(profile_row.get("scenario_profile_path", ""))
        if not profile_path.exists():
            continue
        profile_id = f"{source_scenario}_rank{selection_rank:03d}_seed{source_seed}_{method}"
        for target_scenario in scenarios:
            target_dir = output_dir / target_scenario.name
            for run_row in read_csv(target_dir / "seed_runs.csv"):
                if str(run_row.get("pass_run", "")).lower() != "true":
                    continue
                target_seed = int(float(run_row.get("seed", 0) or 0))
                sample_csv = Path(str(run_row["run_dir"])) / "calibration_samples.csv"
                replay_dir = (
                    output_dir
                    / "scenario_near_tie_profile_transfer_replay"
                    / profile_id
                    / target_scenario.name
                    / f"seed_{target_seed}"
                )
                command = [
                    sys.executable,
                    str(PROFILE_REPLAY_SCRIPT),
                    "--profile-json",
                    str(profile_path),
                    "--input-csv",
                    str(sample_csv),
                    "--output-dir",
                    str(replay_dir),
                    "--split",
                    "validation",
                    "--target-range-m",
                    str(target_range_m),
                    "--target-window-m",
                    str(target_window_m),
                ]
                result = subprocess.run(command, cwd=ROOT_DIR)
                replay_summary_csv = replay_dir / "profile_replay_summary.csv"
                replay_summary = dict(read_csv(replay_summary_csv)[0]) if replay_summary_csv.exists() and read_csv(replay_summary_csv) else {}
                raw_rmse = finite_float(replay_summary.get("validation_range_rmse_raw_m"), float("inf"))
                corrected_rmse = finite_float(replay_summary.get("validation_range_rmse_corrected_m"), float("inf"))
                target_abs = finite_float(replay_summary.get("target_abs_mean_error_corrected_m"), float("inf"))
                target_count = int(float(replay_summary.get("target_window_count", 0) or 0))
                pass_replay = (
                    result.returncode == 0
                    and math.isfinite(raw_rmse)
                    and math.isfinite(corrected_rmse)
                    and corrected_rmse < raw_rmse
                    and target_count > 0
                    and math.isfinite(target_abs)
                    and target_abs <= 10.0
                )
                replay_rows.append(
                    {
                        "profile_id": profile_id,
                        "source_scenario": source_scenario,
                        "source_seed": source_seed,
                        "source_selection_rank": selection_rank,
                        "source_method": method,
                        "source_score_delta_from_winner": finite_float(
                            profile_row.get("selection_score_delta_from_winner"),
                            float("inf"),
                        ),
                        "target_scenario": target_scenario.name,
                        "target_seed": target_seed,
                        "pass_replay": pass_replay,
                        "replay_returncode": int(result.returncode),
                        "profile_path": str(profile_path.relative_to(output_dir)),
                        "sample_csv": str(sample_csv.relative_to(output_dir)) if sample_csv.is_relative_to(output_dir) else str(sample_csv),
                        "replay_output_dir": str(replay_dir.relative_to(output_dir)),
                        "validation_range_rmse_raw_m": raw_rmse,
                        "validation_range_rmse_corrected_m": corrected_rmse,
                        "validation_range_rmse_improvement_m": raw_rmse - corrected_rmse,
                        "target_window_count": target_count,
                        "target_abs_mean_error_corrected_m": target_abs,
                        "axis_contract": RDF_AXIS_CONTRACT,
                        "runtime_input_contract": "near-tie transfer diagnostic; runtime uses raw stereo-center RDF(right,down,forward) plus measured disparity and no truth",
                    }
                )
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in replay_rows:
        grouped.setdefault(str(row["profile_id"]), []).append(row)
    summary_rows: list[dict[str, object]] = []
    for profile_id, rows in sorted(grouped.items()):
        corrected = [finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")) for row in rows]
        target_abs = [finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan")) for row in rows]
        improvements = [finite_float(row.get("validation_range_rmse_improvement_m"), float("nan")) for row in rows]
        finite_corrected = [value for value in corrected if math.isfinite(value)]
        finite_target = [value for value in target_abs if math.isfinite(value)]
        finite_improvements = [value for value in improvements if math.isfinite(value)]
        pass_count = sum(1 for row in rows if bool(row.get("pass_replay")))
        summary_rows.append(
            {
                "profile_id": profile_id,
                "source_scenario": rows[0].get("source_scenario", ""),
                "source_seed": rows[0].get("source_seed", ""),
                "source_selection_rank": rows[0].get("source_selection_rank", ""),
                "source_method": rows[0].get("source_method", ""),
                "source_profile_path": rows[0].get("profile_path", ""),
                **method_contract_fields(str(rows[0].get("source_method", ""))),
                "source_score_delta_from_winner": rows[0].get("source_score_delta_from_winner", ""),
                "transfer_replay_count": len(rows),
                "transfer_replay_pass_count": pass_count,
                "pass_all_transfer_replays": bool(rows) and pass_count == len(rows),
                "validation_range_rmse_corrected_mean_m": float(np.mean(finite_corrected)) if finite_corrected else float("inf"),
                "validation_range_rmse_corrected_max_m": float(np.max(finite_corrected)) if finite_corrected else float("inf"),
                "validation_range_rmse_improvement_min_m": float(np.min(finite_improvements)) if finite_improvements else float("-inf"),
                "target_abs_mean_error_corrected_mean_m": float(np.mean(finite_target)) if finite_target else float("inf"),
                "target_abs_mean_error_corrected_max_m": float(np.max(finite_target)) if finite_target else float("inf"),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "near-tie transfer diagnostic; runtime uses raw stereo-center RDF(right,down,forward) plus measured disparity and no truth",
            }
        )
    summary_rows.sort(
        key=lambda row: (
            not bool(row["pass_all_transfer_replays"]),
            finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_corrected_max_m"), float("inf")),
            finite_float(row.get("source_score_delta_from_winner"), float("inf")),
            str(row.get("profile_id", "")),
        )
    )
    return replay_rows, summary_rows


def choose_best_near_tie_transfer_profile(
    *,
    output_dir: Path,
    transfer_summary_rows: Sequence[Mapping[str, object]],
) -> dict[str, object]:
    candidates = [
        row for row in transfer_summary_rows
        if bool(row.get("pass_all_transfer_replays"))
        and str(row.get("source_profile_path", ""))
    ]
    if not candidates:
        return {
            "selected": False,
            "reason": "no_near_tie_profile_passed_all_transfer_replays",
            "axis_contract": RDF_AXIS_CONTRACT,
            "runtime_input_contract": "no near-tie runtime profile promoted",
        }
    selected = min(
        candidates,
        key=lambda row: (
            finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_corrected_max_m"), float("inf")),
            finite_float(row.get("source_score_delta_from_winner"), float("inf")),
            int(float(row.get("source_selection_rank", 999999) or 999999)),
            str(row.get("profile_id", "")),
        ),
    )
    source_profile = output_dir / str(selected.get("source_profile_path", ""))
    if not source_profile.exists():
        return {
            "selected": False,
            "reason": "selected_near_tie_source_profile_missing",
            "source_profile_path": selected.get("source_profile_path", ""),
            "axis_contract": RDF_AXIS_CONTRACT,
            "runtime_input_contract": "no near-tie runtime profile promoted",
        }
    output_profile = output_dir / "scenario_near_tie_best_runtime_profile.json"
    shutil.copyfile(source_profile, output_profile)
    return {
        "selected": True,
        "reason": "lowest_mean_validation_rmse_among_near_tie_profiles_that_pass_all_transfer_replays",
        "profile_id": selected.get("profile_id", ""),
        "method": selected.get("source_method", ""),
        **method_contract_fields(str(selected.get("source_method", ""))),
        "source_scenario": selected.get("source_scenario", ""),
        "source_seed": selected.get("source_seed", ""),
        "source_selection_rank": selected.get("source_selection_rank", ""),
        "source_score_delta_from_winner": selected.get("source_score_delta_from_winner", ""),
        "source_profile_path": selected.get("source_profile_path", ""),
        "promoted_profile_path": str(output_profile.relative_to(output_dir)),
        "transfer_replay_count": selected.get("transfer_replay_count", 0),
        "transfer_replay_pass_count": selected.get("transfer_replay_pass_count", 0),
        "validation_range_rmse_corrected_mean_m": selected.get("validation_range_rmse_corrected_mean_m", float("inf")),
        "validation_range_rmse_corrected_max_m": selected.get("validation_range_rmse_corrected_max_m", float("inf")),
        "validation_range_rmse_improvement_min_m": selected.get("validation_range_rmse_improvement_min_m", float("-inf")),
        "target_abs_mean_error_corrected_mean_m": selected.get("target_abs_mean_error_corrected_mean_m", float("inf")),
        "target_abs_mean_error_corrected_max_m": selected.get("target_abs_mean_error_corrected_max_m", float("inf")),
        "axis_contract": RDF_AXIS_CONTRACT,
        "runtime_input_contract": "promoted near-tie runtime profile consumes raw stereo-center RDF(right,down,forward) plus measured disparity and no truth",
    }


def aggregate_near_tie_transfer_methods(
    transfer_summary_rows: Sequence[Mapping[str, object]],
) -> list[dict[str, object]]:
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in transfer_summary_rows:
        method = str(row.get("source_method", ""))
        if not method:
            continue
        grouped.setdefault(method, []).append(row)
    rows_out: list[dict[str, object]] = []
    for method, rows in grouped.items():
        pass_all_rows = [row for row in rows if bool(row.get("pass_all_transfer_replays"))]
        corrected_mean_values = [
            finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("nan"))
            for row in rows
        ]
        target_max_values = [
            finite_float(row.get("target_abs_mean_error_corrected_max_m"), float("nan"))
            for row in rows
        ]
        improvement_min_values = [
            finite_float(row.get("validation_range_rmse_improvement_min_m"), float("nan"))
            for row in rows
        ]
        rank_values = [
            int(float(row.get("source_selection_rank", 999999) or 999999))
            for row in rows
        ]
        finite_corrected = [value for value in corrected_mean_values if math.isfinite(value)]
        finite_target = [value for value in target_max_values if math.isfinite(value)]
        finite_improvements = [value for value in improvement_min_values if math.isfinite(value)]
        best_profile = min(
            rows,
            key=lambda row: (
                not bool(row.get("pass_all_transfer_replays")),
                finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
                finite_float(row.get("target_abs_mean_error_corrected_max_m"), float("inf")),
                finite_float(row.get("source_score_delta_from_winner"), float("inf")),
                str(row.get("profile_id", "")),
            ),
        )
        rows_out.append(
            {
                "source_method": method,
                **method_contract_fields(method),
                "near_tie_profile_count": len(rows),
                "pass_all_profile_count": len(pass_all_rows),
                "pass_all_profile_fraction": float(len(pass_all_rows) / len(rows)) if rows else 0.0,
                "best_profile_id": best_profile.get("profile_id", ""),
                "best_source_scenario": best_profile.get("source_scenario", ""),
                "best_source_seed": best_profile.get("source_seed", ""),
                "best_source_selection_rank": best_profile.get("source_selection_rank", ""),
                "best_source_profile_path": best_profile.get("source_profile_path", ""),
                "best_validation_range_rmse_corrected_mean_m": finite_float(
                    best_profile.get("validation_range_rmse_corrected_mean_m"),
                    float("inf"),
                ),
                "best_target_abs_mean_error_corrected_max_m": finite_float(
                    best_profile.get("target_abs_mean_error_corrected_max_m"),
                    float("inf"),
                ),
                "validation_range_rmse_corrected_mean_of_profiles_m": float(np.mean(finite_corrected)) if finite_corrected else float("inf"),
                "validation_range_rmse_corrected_worst_profile_m": float(np.max(finite_corrected)) if finite_corrected else float("inf"),
                "target_abs_mean_error_corrected_worst_profile_m": float(np.max(finite_target)) if finite_target else float("inf"),
                "validation_range_rmse_improvement_worst_profile_m": float(np.min(finite_improvements)) if finite_improvements else float("-inf"),
                "source_selection_rank_best": int(min(rank_values)) if rank_values else 999999,
                "source_selection_rank_worst": int(max(rank_values)) if rank_values else 999999,
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "method-level near-tie transfer diagnostic; runtime profiles use raw RDF/disparity only",
            }
        )
    rows_out.sort(
        key=lambda row: (
            -int(row.get("pass_all_profile_count", 0) or 0),
            finite_float(row.get("best_validation_range_rmse_corrected_mean_m"), float("inf")),
            finite_float(row.get("best_target_abs_mean_error_corrected_max_m"), float("inf")),
            finite_float(row.get("validation_range_rmse_corrected_mean_of_profiles_m"), float("inf")),
            str(row.get("source_method", "")),
        )
    )
    return rows_out


def run_best_near_tie_profile_promotion_check(
    *,
    output_dir: Path,
    best_profile: Mapping[str, object],
) -> dict[str, object]:
    if not bool(best_profile.get("selected")):
        return {
            "pass_promotion_check": False,
            "reason": "no_best_near_tie_profile_selected",
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        }
    promoted_profile_path = str(best_profile.get("promoted_profile_path", ""))
    profile_path = output_dir / promoted_profile_path if promoted_profile_path else output_dir
    if not promoted_profile_path or not profile_path.exists():
        return {
            "pass_promotion_check": False,
            "reason": "promoted_near_tie_profile_missing",
            "profile_path": promoted_profile_path,
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        }
    source_scenario = str(best_profile.get("source_scenario", ""))
    source_seed = str(best_profile.get("source_seed", ""))
    sample_csv = output_dir / source_scenario / f"seed_{source_seed}" / "calibration_samples.csv"
    if not sample_csv.exists() and source_scenario:
        run_rows = read_csv(output_dir / source_scenario / "seed_runs.csv")
        first_pass = next((run_row for run_row in run_rows if str(run_row.get("pass_run", "")).lower() == "true"), None)
        sample_csv = Path(str(first_pass["run_dir"])) / "calibration_samples.csv" if first_pass else sample_csv
    promotion_dir = output_dir / "scenario_near_tie_best_promotion_check"
    command = [
        sys.executable,
        str(PROFILE_PROMOTION_CHECK_SCRIPT),
        "--profile-json",
        str(profile_path),
        "--output-dir",
        str(promotion_dir),
        "--split",
        "validation",
    ]
    if sample_csv.exists():
        command.extend(["--sample-csv", str(sample_csv)])
    result = subprocess.run(command, cwd=ROOT_DIR)
    promotion_json = promotion_dir / "promotion_check.json"
    promotion_summary = json.loads(promotion_json.read_text(encoding="utf-8")) if promotion_json.exists() else {}
    pass_promotion = result.returncode == 0 and bool(promotion_summary.get("pass_promotion_check"))
    return {
        "pass_promotion_check": pass_promotion,
        "method": best_profile.get("method", ""),
        "profile_path": promoted_profile_path,
        "source_scenario": source_scenario,
        "source_seed": source_seed,
        "sample_csv": str(sample_csv.relative_to(output_dir)) if sample_csv.exists() and sample_csv.is_relative_to(output_dir) else str(sample_csv),
        "promotion_output_dir": str(promotion_dir.relative_to(output_dir)),
        "promotion_returncode": int(result.returncode),
        "schema_load_ok": bool(promotion_summary.get("schema_load_ok", False)),
        "node_loader_ok": bool(promotion_summary.get("node_loader_ok", False)),
        "runtime_truth_used": bool(promotion_summary.get("runtime_truth_used", True)),
        "forbidden_runtime_inputs_declared": bool(promotion_summary.get("forbidden_runtime_inputs_declared", False)),
        "sample_apply_count": int(promotion_summary.get("sample_apply_count", 0) or 0),
        "sample_invalid_count": int(promotion_summary.get("sample_invalid_count", 0) or 0),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def choose_catalog_minimal_profile_set(
    pair_summary_rows: Sequence[Mapping[str, object]],
) -> tuple[list[dict[str, object]], dict[str, object]]:
    target_scenarios = sorted({str(row.get("target_scenario", "")) for row in pair_summary_rows if row.get("target_scenario", "")})
    source_groups: dict[str, list[Mapping[str, object]]] = {}
    for row in pair_summary_rows:
        source_groups.setdefault(str(row.get("source_scenario", "")), []).append(row)

    selected_rows: list[dict[str, object]] = []
    uncovered = set(target_scenarios)
    selected_sources: set[str] = set()
    selection_index = 1
    while uncovered:
        candidates: list[tuple[tuple[object, ...], str, set[str], list[Mapping[str, object]]]] = []
        for source_scenario, rows in source_groups.items():
            if source_scenario in selected_sources:
                continue
            covered = {
                str(row.get("target_scenario", ""))
                for row in rows
                if parse_bool(row.get("pass_all")) and str(row.get("target_scenario", "")) in uncovered
            }
            if not covered:
                continue
            all_pass_targets = {
                str(row.get("target_scenario", ""))
                for row in rows
                if parse_bool(row.get("pass_all")) and str(row.get("target_scenario", ""))
            }
            pass_count = sum(int(row.get("pass_count", 0) or 0) for row in rows)
            replay_count = sum(int(row.get("replay_count", 0) or 0) for row in rows)
            corrected_values = [
                finite_float(row.get("mean_validation_range_rmse_corrected_m"), float("nan"))
                for row in rows
                if parse_bool(row.get("pass_all"))
            ]
            target_values = [
                finite_float(row.get("max_target_abs_mean_error_corrected_m"), float("nan"))
                for row in rows
                if parse_bool(row.get("pass_all"))
            ]
            finite_corrected = [value for value in corrected_values if math.isfinite(value)]
            finite_target = [value for value in target_values if math.isfinite(value)]
            rank = (
                -len(covered),
                -len(all_pass_targets),
                -(pass_count / replay_count if replay_count else 0.0),
                float(np.mean(finite_corrected)) if finite_corrected else float("inf"),
                float(np.max(finite_target)) if finite_target else float("inf"),
                source_scenario,
            )
            candidates.append((rank, source_scenario, covered, rows))
        if not candidates:
            break
        _, source_scenario, newly_covered, rows = min(candidates, key=lambda item: item[0])
        selected_sources.add(source_scenario)
        uncovered -= newly_covered
        all_pass_targets = sorted(
            {
                str(row.get("target_scenario", ""))
                for row in rows
                if parse_bool(row.get("pass_all")) and str(row.get("target_scenario", ""))
            }
        )
        selected_rows.append(
            {
                "selection_index": selection_index,
                "source_scenario": source_scenario,
                "source_method": rows[0].get("source_method", "") if rows else "",
                "newly_covered_target_scenario_count": len(newly_covered),
                "newly_covered_target_scenarios": ",".join(sorted(newly_covered)),
                "total_covered_target_scenario_count": len(set(target_scenarios) - uncovered),
                "all_pass_target_scenario_count": len(all_pass_targets),
                "all_pass_target_scenarios": ",".join(all_pass_targets),
                "remaining_uncovered_target_scenario_count": len(uncovered),
                "remaining_uncovered_target_scenarios": ",".join(sorted(uncovered)),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "minimal catalog set diagnostic; selected runtime profile still uses raw RDF/disparity only",
            }
        )
        selection_index += 1

    summary = {
        "target_scenario_count": len(target_scenarios),
        "target_scenarios": target_scenarios,
        "selected_profile_count": len(selected_rows),
        "selected_source_scenarios": [str(row["source_scenario"]) for row in selected_rows],
        "covered_target_scenario_count": len(set(target_scenarios) - uncovered),
        "covered_target_scenarios": sorted(set(target_scenarios) - uncovered),
        "uncovered_target_scenario_count": len(uncovered),
        "uncovered_target_scenarios": sorted(uncovered),
        "pass_minimal_catalog_profile_set": bool(target_scenarios) and not uncovered,
        "selection_rule": (
            "Greedy set cover over scenario_profile_catalog_transfer_matrix.csv using only "
            "source-target pairs where every validation replay passed."
        ),
        "selection_contract": (
            "This chooses deployment profile options from offline validation. Runtime selection among the chosen "
            "profiles must use deployment/calibration metadata or operator configuration, not live truth."
        ),
        "axis_contract": RDF_AXIS_CONTRACT,
        "runtime_input_contract": "runtime profiles selected by this set use raw stereo-center RDF(right,down,forward) plus measured disparity",
        "rows": selected_rows,
    }
    return selected_rows, summary


def deployment_strategy_recommendation(
    *,
    policy_transfer_summary_rows: Sequence[Mapping[str, object]],
    scenario_catalog_summary: Mapping[str, object],
    scenario_catalog_promotion_summary: Mapping[str, object],
    pooled_profile_summary: Mapping[str, object] | None = None,
    best_near_tie_profile: Mapping[str, object] | None = None,
    best_near_tie_promotion_check: Mapping[str, object] | None = None,
) -> dict[str, object]:
    transfer_pass_rows = [
        row for row in policy_transfer_summary_rows
        if bool(row.get("pass_all_transfer_replays", False))
    ]
    transfer_rows = list(policy_transfer_summary_rows)
    best_transfer = min(
        transfer_rows,
        key=lambda row: (
            not bool(row.get("pass_all_transfer_replays", False)),
            -int(row.get("transfer_replay_pass_count", 0) or 0),
            finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_corrected_max_m"), float("inf")),
            str(row.get("policy", "")),
        ),
        default={},
    )
    catalog_ready = (
        bool(scenario_catalog_summary.get("pass_scenario_profile_catalog", False))
        and bool(scenario_catalog_promotion_summary.get("pass_catalog_promotion_checks", False))
    )
    pooled_profile = pooled_profile_summary or {}
    pooled_pass = (
        bool(pooled_profile.get("pass_all_transfer_replays", False))
        and bool(pooled_profile.get("pass_promotion_check", False))
        and not bool(pooled_profile.get("runtime_truth_used", True))
    )
    best_near_tie = best_near_tie_profile or {}
    near_tie_promotion = best_near_tie_promotion_check or {}
    near_tie_pass = bool(best_near_tie.get("selected")) and bool(near_tie_promotion.get("pass_promotion_check", False))
    if near_tie_pass:
        near_tie_sort_key = (
            finite_float(best_near_tie.get("validation_range_rmse_corrected_mean_m"), float("inf")),
            finite_float(best_near_tie.get("target_abs_mean_error_corrected_max_m"), float("inf")),
            str(best_near_tie.get("profile_id", "")),
        )
    else:
        near_tie_sort_key = (float("inf"), float("inf"), "")
    if transfer_pass_rows:
        best_pass = min(
            transfer_pass_rows,
            key=lambda row: (
                finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
                finite_float(row.get("target_abs_mean_error_corrected_max_m"), float("inf")),
                str(row.get("policy", "")),
            ),
        )
        policy_sort_key = (
            finite_float(best_pass.get("validation_range_rmse_corrected_mean_m"), float("inf")),
            finite_float(best_pass.get("target_abs_mean_error_corrected_max_m"), float("inf")),
            str(best_pass.get("policy", "")),
        )
        if near_tie_pass and near_tie_sort_key < policy_sort_key:
            strategy = "single_near_tie_runtime_profile"
            selected_policy = ""
            reason = (
                "A near-tie candidate profile passed replay on every scenario/seed split and beat "
                "the best representative policy profile on transfer validation RMSE."
            )
            selected_profile_contract = "single promoted near-tie runtime profile; runtime uses raw RDF/disparity only"
        else:
            strategy = "single_runtime_profile"
            selected_policy = str(best_pass.get("policy", ""))
            reason = (
                "At least one representative policy profile passed replay on every scenario/seed split; "
                "use the best passing policy profile as the default runtime profile."
            )
            selected_profile_contract = "single calibrated runtime profile; runtime uses raw RDF/disparity only"
    elif near_tie_pass:
        strategy = "single_near_tie_runtime_profile"
        selected_policy = ""
        reason = (
            "No representative policy profile passed every transfer replay, but a near-tie candidate "
            "profile did. Promote the best near-tie transfer profile as the default runtime profile."
        )
        selected_profile_contract = "single promoted near-tie runtime profile; runtime uses raw RDF/disparity only"
    elif pooled_pass:
        strategy = "single_pooled_runtime_profile"
        selected_policy = ""
        reason = (
            "No representative policy or near-tie profile passed every transfer replay, but a profile "
            "fitted from pooled scenario calibration data passed all scenario/seed replays and promotion checks."
        )
        selected_profile_contract = "single pooled calibrated runtime profile; runtime uses raw RDF/disparity only"
    elif catalog_ready:
        strategy = "scenario_profile_catalog"
        selected_policy = ""
        reason = (
            "No single representative profile passed every transfer replay, but every scenario-specific "
            "catalog profile passed cross-seed replay and StereoDistance promotion checks. Select a "
            "profile by deployment metadata or explicit operator scenario, not by runtime truth."
        )
        selected_profile_contract = (
            "scenario-specific calibrated runtime profile selected from catalog metadata; runtime uses raw RDF/disparity only"
        )
    else:
        strategy = "no_validated_runtime_strategy"
        selected_policy = ""
        reason = (
            "Neither universal transfer replay nor scenario-specific catalog promotion fully passed. "
            "Treat the sweep as diagnostic and collect more calibration data or add narrower scenarios."
        )
        selected_profile_contract = "no runtime profile should be promoted automatically"
    return {
        "deployment_strategy": strategy,
        "selected_policy": selected_policy,
        "best_transfer_policy": best_transfer.get("policy", ""),
        "best_transfer_method": best_transfer.get("method", ""),
        "best_transfer_pass_count": int(best_transfer.get("transfer_replay_pass_count", 0) or 0) if best_transfer else 0,
        "best_transfer_total_count": int(best_transfer.get("transfer_replay_count", 0) or 0) if best_transfer else 0,
        "best_near_tie_profile_id": best_near_tie.get("profile_id", ""),
        "best_near_tie_method": best_near_tie.get("method", ""),
        "best_near_tie_profile_path": best_near_tie.get("promoted_profile_path", ""),
        "best_near_tie_transfer_pass_count": int(best_near_tie.get("transfer_replay_pass_count", 0) or 0) if best_near_tie else 0,
        "best_near_tie_transfer_total_count": int(best_near_tie.get("transfer_replay_count", 0) or 0) if best_near_tie else 0,
        "best_near_tie_promotion_pass": bool(near_tie_promotion.get("pass_promotion_check", False)),
        "pooled_profile_method": pooled_profile.get("method", ""),
        "pooled_profile_path": pooled_profile.get("profile_path", ""),
        "pooled_profile_pass": pooled_pass,
        "pooled_profile_transfer_pass_count": int(pooled_profile.get("transfer_replay_pass_count", 0) or 0) if pooled_profile else 0,
        "pooled_profile_transfer_total_count": int(pooled_profile.get("transfer_replay_count", 0) or 0) if pooled_profile else 0,
        "pooled_profile_promotion_pass": bool(pooled_profile.get("pass_promotion_check", False)),
        "single_profile_pass": bool(transfer_pass_rows) or near_tie_pass or pooled_pass,
        "representative_policy_profile_pass": bool(transfer_pass_rows),
        "near_tie_profile_pass": near_tie_pass,
        "scenario_catalog_ready": catalog_ready,
        "scenario_catalog_profiles": int(scenario_catalog_summary.get("catalog_profile_count", 0) or 0),
        "scenario_catalog_promotion_pass_count": int(scenario_catalog_promotion_summary.get("catalog_promotion_pass_count", 0) or 0),
        "reason": reason,
        "selection_contract": (
            "Calibration-time truth is used only to fit and validate profiles. Runtime profile selection must use "
            "deployment metadata, explicit operator configuration, or known calibration scenario labels; it must not use "
            "PX4/RTK truth or validation errors from the live measurement."
        ),
        "runtime_profile_contract": selected_profile_contract,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def write_deployment_strategy_txt(path: Path, recommendation: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance deployment strategy recommendation",
        "",
        f"strategy: {recommendation.get('deployment_strategy', '')}",
        f"single_profile_pass: {recommendation.get('single_profile_pass', False)}",
        f"scenario_catalog_ready: {recommendation.get('scenario_catalog_ready', False)}",
        (
            "best_transfer: {policy}/{method} {passed}/{total}".format(
                policy=recommendation.get("best_transfer_policy", ""),
                method=recommendation.get("best_transfer_method", ""),
                passed=recommendation.get("best_transfer_pass_count", 0),
                total=recommendation.get("best_transfer_total_count", 0),
            )
        ),
        (
            "best_near_tie: {profile}/{method} {passed}/{total}, promotion={promotion}, path={path}".format(
                profile=recommendation.get("best_near_tie_profile_id", ""),
                method=recommendation.get("best_near_tie_method", ""),
                passed=recommendation.get("best_near_tie_transfer_pass_count", 0),
                total=recommendation.get("best_near_tie_transfer_total_count", 0),
                promotion=recommendation.get("best_near_tie_promotion_pass", False),
                path=recommendation.get("best_near_tie_profile_path", ""),
            )
        ),
        (
            "pooled_profile: {method} {passed}/{total}, promotion={promotion}, path={path}".format(
                method=recommendation.get("pooled_profile_method", ""),
                passed=recommendation.get("pooled_profile_transfer_pass_count", 0),
                total=recommendation.get("pooled_profile_transfer_total_count", 0),
                promotion=recommendation.get("pooled_profile_promotion_pass", False),
                path=recommendation.get("pooled_profile_path", ""),
            )
        ),
        f"reason: {recommendation.get('reason', '')}",
        f"selection_contract: {recommendation.get('selection_contract', '')}",
        f"runtime_profile_contract: {recommendation.get('runtime_profile_contract', '')}",
        f"axis_contract: {recommendation.get('axis_contract', '')}",
        f"public_topic_contract: {recommendation.get('public_topic_contract', '')}",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_summary_txt(
    path: Path,
    *,
    run_rows: Sequence[Mapping[str, object]],
    recommendation_rows: Sequence[Mapping[str, object]],
    robust_summary_rows: Sequence[Mapping[str, object]],
    robust_family_summary_rows: Sequence[Mapping[str, object]],
    robust_recommendation: Mapping[str, object],
    policy_rows: Sequence[Mapping[str, object]],
    scenario_near_tie_profiles: Sequence[Mapping[str, object]],
    scenario_near_tie_transfer_summary_rows: Sequence[Mapping[str, object]],
    scenario_near_tie_method_summary_rows: Sequence[Mapping[str, object]],
    policy_profile_rows: Sequence[Mapping[str, object]],
    policy_transfer_summary_rows: Sequence[Mapping[str, object]],
    transfer_diagnostic_rows: Sequence[Mapping[str, object]],
    scenario_catalog_rows: Sequence[Mapping[str, object]],
    scenario_catalog_summary: Mapping[str, object],
    scenario_catalog_promotion_summary: Mapping[str, object],
    catalog_transfer_source_summary_rows: Sequence[Mapping[str, object]],
    catalog_minimal_profile_set_rows: Sequence[Mapping[str, object]],
    catalog_minimal_profile_set_summary: Mapping[str, object],
    deployment_strategy: Mapping[str, object],
) -> None:
    lines = [
        "RTK stereo robustness sweep",
        "",
        f"NED contract: {NED_AXIS_CONTRACT}",
        f"RDF contract: {RDF_AXIS_CONTRACT}",
        f"StereoDistance public topic contract: {PUBLIC_TOPIC_CONTRACT}",
        "",
        "Scenario runs:",
    ]
    for row in run_rows:
        lines.append(
            "- {scenario}: pass_run={pass_run}, recommended={method}, profile_check={profile}, cross_seed={cross}".format(
                scenario=row["scenario"],
                pass_run=row["pass_run"],
                method=row["recommended_method"] or "none",
                profile=row["pass_representative_profile_checks"],
                cross=row["pass_cross_seed_replay"],
            )
        )
    lines.extend(["", "Scenario recommendations:"])
    for row in recommendation_rows:
        lines.append(
            "- {scenario}: mode={mode}, method={method}, family={family}/{residual}/{regularization}, validation_mean={rmse}, target_max={target}".format(
                scenario=row["scenario"],
                mode=row["recommendation_mode"],
                method=row["method"] or "none",
                family=row.get("registration_model", ""),
                residual=row.get("residual_model", ""),
                regularization=row.get("regularization_model", ""),
                rmse=row["validation_range_rmse_mean_m"],
                target=row["target_abs_mean_error_max_m"],
            )
        )
    lines.extend(["", "Robust method ranking:"])
    for row in list(robust_summary_rows)[:12]:
        lines.append(
            "- {method}: strict={strict}/{total}, performance={perf}/{total}, "
            "family={family}/{residual}/{regularization}, "
            "validation_mean={rmse:.6f}m, target_max={target:.6f}m, score={score:.6f}".format(
                method=row["method"],
                strict=row["strict_pass_count"],
                perf=row["performance_pass_count"],
                total=row["expected_scenario_count"],
                family=row.get("registration_model", ""),
                residual=row.get("residual_model", ""),
                regularization=row.get("regularization_model", ""),
                rmse=float(row["validation_range_rmse_mean_m"]),
                target=float(row["target_abs_mean_error_max_m"]),
                score=float(row["robust_score"]),
            )
        )
    lines.extend(["", "Robust method-family ranking:"])
    for row in list(robust_family_summary_rows)[:10]:
        lines.append(
            "- {family}/{residual}/{regularization}: methods={count}, strict_methods={strict}, "
            "best={method}, best_validation={rmse:.6f}m, best_target_max={target:.6f}m, score={score:.6f}".format(
                family=row.get("registration_model", ""),
                residual=row.get("residual_model", ""),
                regularization=row.get("regularization_model", ""),
                count=row.get("method_count", 0),
                strict=row.get("strict_all_method_count", 0),
                method=row.get("best_method", ""),
                rmse=float(row.get("best_validation_range_rmse_mean_m", float("inf"))),
                target=float(row.get("best_target_abs_mean_error_max_m", float("inf"))),
                score=float(row.get("best_robust_score", float("inf"))),
            )
        )
    lines.extend(
        [
            "",
            "Robust recommendation:",
            "- mode={mode}, method={method}, reason={reason}".format(
                mode=robust_recommendation.get("recommendation_mode", "none"),
                method=robust_recommendation.get("method", ""),
                reason=robust_recommendation.get("reason", ""),
            ),
        ]
    )
    lines.extend(["", "Robust policy recommendations:"])
    for row in policy_rows:
        lines.append(
            "- {policy}: method={method}, validation_mean={rmse:.6f}m, "
            "target_max={target:.6f}m, complexity={complexity:.3f}, family={family}/{residual}/{regularization}".format(
                policy=row["policy"],
                method=row["method"],
                rmse=float(row["validation_range_rmse_mean_m"]),
                target=float(row["target_abs_mean_error_max_m"]),
                complexity=float(row["model_complexity_mean"]),
                family=row.get("registration_model", ""),
                residual=row.get("residual_model", ""),
                regularization=row.get("regularization_model", ""),
            )
        )
    lines.extend(["", "Scenario near-tie runtime profiles:"])
    if scenario_near_tie_profiles:
        for row in list(scenario_near_tie_profiles)[:16]:
            lines.append(
                "- {scenario} rank={rank}, seed={seed}, method={method}, "
                "profile={profile}, delta={delta}".format(
                    scenario=row.get("scenario", ""),
                    rank=row.get("selection_rank", ""),
                    seed=row.get("seed", ""),
                    method=row.get("method", ""),
                    profile=row.get("scenario_profile_path", "") or "missing",
                    delta=row.get("selection_score_delta_from_winner", ""),
                )
            )
    else:
        lines.append("- none")
    lines.extend(["", "Scenario near-tie transfer summary:"])
    if scenario_near_tie_transfer_summary_rows:
        for row in list(scenario_near_tie_transfer_summary_rows)[:12]:
            lines.append(
                "- source={scenario}/seed_{seed}, rank={rank}, method={method}, "
                "pass={passed}, pass_count={pass_count}/{total}, corrected_mean={rmse:.6f}m, "
                "target_max={target:.6f}m".format(
                    scenario=row.get("source_scenario", ""),
                    seed=row.get("source_seed", ""),
                    rank=row.get("source_selection_rank", ""),
                    method=row.get("source_method", ""),
                    passed=row.get("pass_all_transfer_replays", False),
                    pass_count=row.get("transfer_replay_pass_count", 0),
                    total=row.get("transfer_replay_count", 0),
                    rmse=finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("nan")),
                    target=finite_float(row.get("target_abs_mean_error_corrected_max_m"), float("nan")),
                )
            )
    else:
        lines.append("- none")
    lines.extend(["", "Scenario near-tie method transfer ranking:"])
    if scenario_near_tie_method_summary_rows:
        for row in list(scenario_near_tie_method_summary_rows)[:10]:
            lines.append(
                "- method={method}, profiles={passed}/{total}, best={best}, "
                "best_rmse={rmse:.6f}m, worst_target={target:.6f}m".format(
                    method=row.get("source_method", ""),
                    passed=row.get("pass_all_profile_count", 0),
                    total=row.get("near_tie_profile_count", 0),
                    best=row.get("best_profile_id", ""),
                    rmse=finite_float(row.get("best_validation_range_rmse_corrected_mean_m"), float("nan")),
                    target=finite_float(row.get("target_abs_mean_error_corrected_worst_profile_m"), float("nan")),
                )
            )
    else:
        lines.append("- none")
    lines.extend(["", "Robust policy representative profiles:"])
    for row in policy_profile_rows:
        lines.append(
            "- {policy}: selected={selected}, method={method}, source={scenario}/seed_{seed}, profile={profile}".format(
                policy=row["policy"],
                selected=row.get("selected", False),
                method=row.get("method", ""),
                scenario=row.get("source_scenario", ""),
                seed=row.get("source_seed", ""),
                profile=row.get("representative_profile_path", ""),
            )
        )
    lines.extend(["", "Robust policy transfer replay:"])
    for row in policy_transfer_summary_rows:
        lines.append(
            "- {policy}: pass={passed}, pass_count={passed_count}/{total}, "
            "corrected_rmse_mean={rmse:.6f}m, target_max={target:.6f}m".format(
                policy=row["policy"],
                passed=row["pass_all_transfer_replays"],
                passed_count=row["transfer_replay_pass_count"],
                total=row["transfer_replay_count"],
                rmse=float(row["validation_range_rmse_corrected_mean_m"]),
                target=float(row["target_abs_mean_error_corrected_max_m"]),
            )
        )
    failing_transfer_rows = [row for row in transfer_diagnostic_rows if int(row.get("fail_count", 0) or 0) > 0]
    if failing_transfer_rows:
        lines.extend(["", "Robust policy transfer diagnostics:"])
        for row in sorted(
            failing_transfer_rows,
            key=lambda item: (
                -int(item.get("fail_count", 0) or 0),
                finite_float(item.get("min_validation_range_rmse_improvement_m"), float("inf")),
                str(item.get("policy", "")),
                str(item.get("target_scenario", "")),
            ),
        )[:12]:
            lines.append(
                "- {policy}->{scenario}: fail_count={fail}/{total}, min_improvement={improvement:.6f}m, "
                "target_max={target:.6f}m, reasons={reasons}, action={action}".format(
                    policy=row.get("policy", ""),
                    scenario=row.get("target_scenario", ""),
                    fail=row.get("fail_count", 0),
                    total=row.get("replay_count", 0),
                    improvement=finite_float(row.get("min_validation_range_rmse_improvement_m"), float("nan")),
                    target=finite_float(row.get("max_target_abs_mean_error_corrected_m"), float("nan")),
                    reasons=row.get("failure_reasons", ""),
                    action=row.get("recommended_action", ""),
                )
            )
    lines.extend(["", "Scenario-specific profile catalog:"])
    lines.append(
        "- pass={passed}, profiles={profiles}/{total}, cross_seed_pass={cross}/{total}".format(
            passed=scenario_catalog_summary.get("pass_scenario_profile_catalog", False),
            profiles=scenario_catalog_summary.get("profile_exists_count", 0),
            cross=scenario_catalog_summary.get("cross_seed_pass_count", 0),
            total=scenario_catalog_summary.get("catalog_profile_count", 0),
        )
    )
    lines.append(
        "- promotion_pass={passed}, promotion_count={passed_count}/{total}".format(
            passed=scenario_catalog_promotion_summary.get("pass_catalog_promotion_checks", False),
            passed_count=scenario_catalog_promotion_summary.get("catalog_promotion_pass_count", 0),
            total=scenario_catalog_promotion_summary.get("catalog_promotion_check_count", 0),
        )
    )
    for row in scenario_catalog_rows:
        lines.append(
            "- {scenario}: method={method}, seed={seed}, pass_cross_seed={passed}, profile={profile}".format(
                scenario=row["scenario"],
                method=row.get("method", ""),
                seed=row.get("source_seed", ""),
                passed=row.get("pass_cross_seed_replay", False),
                profile=row.get("catalog_profile_path", ""),
            )
        )
    if catalog_transfer_source_summary_rows:
        lines.extend(["", "Scenario catalog transfer coverage:"])
        for row in list(catalog_transfer_source_summary_rows)[:8]:
            lines.append(
                "- {source}: covered={covered}/{total}, pass_count={passed}/{replays}, targets={targets}".format(
                    source=row.get("source_scenario", ""),
                    covered=row.get("covered_target_scenario_count", 0),
                    total=row.get("target_scenario_count", 0),
                    passed=row.get("transfer_replay_pass_count", 0),
                    replays=row.get("transfer_replay_count", 0),
                    targets=row.get("covered_target_scenarios", ""),
                )
            )
    if catalog_minimal_profile_set_summary:
        lines.extend(["", "Scenario catalog minimal profile set:"])
        lines.append(
            "- pass={passed}, profiles={profiles}, covered={covered}/{total}, uncovered={uncovered}".format(
                passed=catalog_minimal_profile_set_summary.get("pass_minimal_catalog_profile_set", False),
                profiles=catalog_minimal_profile_set_summary.get("selected_profile_count", 0),
                covered=catalog_minimal_profile_set_summary.get("covered_target_scenario_count", 0),
                total=catalog_minimal_profile_set_summary.get("target_scenario_count", 0),
                uncovered=",".join(catalog_minimal_profile_set_summary.get("uncovered_target_scenarios", []) or []),
            )
        )
        for row in catalog_minimal_profile_set_rows:
            lines.append(
                "- #{idx} {source}: new={new_targets}, all_pass={all_targets}, remaining={remaining}".format(
                    idx=row.get("selection_index", ""),
                    source=row.get("source_scenario", ""),
                    new_targets=row.get("newly_covered_target_scenarios", ""),
                    all_targets=row.get("all_pass_target_scenarios", ""),
                    remaining=row.get("remaining_uncovered_target_scenarios", ""),
                )
            )
    lines.extend(
        [
            "",
            "Deployment strategy:",
            "- strategy={strategy}, single_profile_pass={single}, catalog_ready={catalog}, best_transfer={policy}/{method} {passed}/{total}".format(
                strategy=deployment_strategy.get("deployment_strategy", ""),
                single=deployment_strategy.get("single_profile_pass", False),
                catalog=deployment_strategy.get("scenario_catalog_ready", False),
                policy=deployment_strategy.get("best_transfer_policy", ""),
                method=deployment_strategy.get("best_transfer_method", ""),
                passed=deployment_strategy.get("best_transfer_pass_count", 0),
                total=deployment_strategy.get("best_transfer_total_count", 0),
            ),
            "- best_near_tie={profile}/{method} {passed}/{total}, promotion={promotion}, path={path}".format(
                profile=deployment_strategy.get("best_near_tie_profile_id", ""),
                method=deployment_strategy.get("best_near_tie_method", ""),
                passed=deployment_strategy.get("best_near_tie_transfer_pass_count", 0),
                total=deployment_strategy.get("best_near_tie_transfer_total_count", 0),
                promotion=deployment_strategy.get("best_near_tie_promotion_pass", False),
                path=deployment_strategy.get("best_near_tie_profile_path", ""),
            ),
            "- pooled_profile={method} {passed}/{total}, promotion={promotion}, path={path}".format(
                method=deployment_strategy.get("pooled_profile_method", ""),
                passed=deployment_strategy.get("pooled_profile_transfer_pass_count", 0),
                total=deployment_strategy.get("pooled_profile_transfer_total_count", 0),
                promotion=deployment_strategy.get("pooled_profile_promotion_pass", False),
                path=deployment_strategy.get("pooled_profile_path", ""),
            ),
            "- reason={reason}".format(reason=deployment_strategy.get("reason", "")),
            "- selection_contract={contract}".format(contract=deployment_strategy.get("selection_contract", "")),
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    custom_scenarios = load_custom_scenarios(args.scenario_config_json)
    scenarios = parse_scenario_list(args.scenarios, registry=scenario_registry(custom_scenarios))

    run_rows: list[dict[str, object]] = []
    scenario_recommendations: list[dict[str, object]] = []
    scenario_method_gate_rows: list[dict[str, object]] = []
    for scenario in scenarios:
        scenario_dir = output_dir / scenario.name
        exit_code = run_scenario(args, scenario, scenario_dir)
        run_row, gate_rows, recommendation_row = collect_scenario_outputs(
            scenario=scenario,
            scenario_dir=scenario_dir,
            exit_code=exit_code,
            sweep_output_dir=output_dir,
        )
        run_rows.append(run_row)
        scenario_method_gate_rows.extend(gate_rows)
        scenario_recommendations.append(recommendation_row)

    robust_summary = aggregate_robust_methods(scenario_method_gate_rows, scenario_count=len(scenarios))
    robust_family_summary = aggregate_robust_method_families(robust_summary)
    robust_recommendation = choose_robust_recommendation(robust_summary)
    robust_policy_rows = robust_policy_recommendation_rows(robust_summary)
    scenario_near_tie_profiles = collect_scenario_near_tie_profiles(
        output_dir=output_dir,
        scenarios=scenarios,
    )
    if args.skip_near_tie_transfer:
        scenario_near_tie_transfer_replay_rows = []
        scenario_near_tie_transfer_summary_rows = []
        scenario_near_tie_method_summary_rows = []
    else:
        (
            scenario_near_tie_transfer_replay_rows,
            scenario_near_tie_transfer_summary_rows,
        ) = run_scenario_near_tie_profile_transfer_replay(
            output_dir=output_dir,
            scenarios=scenarios,
            near_tie_profiles=scenario_near_tie_profiles,
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
        )
        scenario_near_tie_method_summary_rows = aggregate_near_tie_transfer_methods(
            scenario_near_tie_transfer_summary_rows
        )
    policy_profile_candidates = collect_policy_profile_candidates(
        output_dir=output_dir,
        scenarios=scenarios,
        policy_rows=robust_policy_rows,
    )
    policy_representative_profiles = choose_policy_representative_profiles(
        output_dir=output_dir,
        policy_rows=robust_policy_rows,
        profile_candidates=policy_profile_candidates,
    )
    policy_transfer_replay_rows, policy_transfer_summary_rows = run_policy_profile_transfer_replay(
        output_dir=output_dir,
        scenarios=scenarios,
        representative_profiles=policy_representative_profiles,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    transfer_diagnostic_rows, transfer_diagnostic_summary = transfer_failure_diagnostic_rows(
        policy_transfer_replay_rows
    )
    scenario_catalog_rows, scenario_catalog_summary = collect_scenario_profile_catalog(
        output_dir=output_dir,
        scenarios=scenarios,
    )
    scenario_catalog_promotion_rows, scenario_catalog_promotion_summary = run_scenario_catalog_promotion_checks(
        output_dir=output_dir,
        catalog_rows=scenario_catalog_rows,
    )
    (
        catalog_transfer_replay_rows,
        catalog_transfer_pair_summary_rows,
        catalog_transfer_source_summary_rows,
    ) = run_catalog_profile_transfer_matrix(
        output_dir=output_dir,
        scenarios=scenarios,
        catalog_rows=scenario_catalog_rows,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    catalog_minimal_profile_set_rows, catalog_minimal_profile_set_summary = choose_catalog_minimal_profile_set(
        catalog_transfer_pair_summary_rows
    )
    if args.skip_near_tie_transfer:
        best_near_tie_profile = {
            "selected": False,
            "reason": "near_tie_transfer_skipped_by_cli",
            "axis_contract": RDF_AXIS_CONTRACT,
            "runtime_input_contract": "near-tie transfer skipped; no near-tie runtime profile promoted",
        }
        best_near_tie_promotion_check = {
            "pass_promotion_check": False,
            "reason": "near_tie_transfer_skipped_by_cli",
        }
    else:
        best_near_tie_profile = choose_best_near_tie_transfer_profile(
            output_dir=output_dir,
            transfer_summary_rows=scenario_near_tie_transfer_summary_rows,
        )
        best_near_tie_promotion_check = run_best_near_tie_profile_promotion_check(
            output_dir=output_dir,
            best_profile=best_near_tie_profile,
        )
    pooled_profile_replay_rows, pooled_profile_summary_rows, pooled_profile_summary = run_pooled_calibration_profile(
        args=args,
        output_dir=output_dir,
        scenarios=scenarios,
    )
    pooled_profile_diagnostic_rows, pooled_profile_diagnostic_summary = pooled_profile_failure_diagnostics(
        pooled_profile_replay_rows,
        pooled_profile_summary_rows,
        pooled_profile_summary,
    )
    deployment_strategy = deployment_strategy_recommendation(
        policy_transfer_summary_rows=policy_transfer_summary_rows,
        scenario_catalog_summary=scenario_catalog_summary,
        scenario_catalog_promotion_summary=scenario_catalog_promotion_summary,
        pooled_profile_summary=pooled_profile_summary,
        best_near_tie_profile=best_near_tie_profile,
        best_near_tie_promotion_check=best_near_tie_promotion_check,
    )
    write_csv(output_dir / "scenario_runs.csv", run_rows)
    write_csv(output_dir / "scenario_recommendations.csv", scenario_recommendations)
    write_csv(output_dir / "scenario_method_gates.csv", scenario_method_gate_rows)
    write_csv(output_dir / "robust_method_summary.csv", robust_summary)
    write_csv(output_dir / "robust_method_family_summary.csv", robust_family_summary)
    write_csv(output_dir / "robust_policy_recommendations.csv", robust_policy_rows)
    write_csv(output_dir / "scenario_near_tie_profiles.csv", scenario_near_tie_profiles)
    write_csv(output_dir / "scenario_near_tie_profile_transfer_replay.csv", scenario_near_tie_transfer_replay_rows)
    write_csv(output_dir / "scenario_near_tie_profile_transfer_summary.csv", scenario_near_tie_transfer_summary_rows)
    write_csv(output_dir / "scenario_near_tie_method_transfer_summary.csv", scenario_near_tie_method_summary_rows)
    write_csv(output_dir / "robust_policy_profile_candidates.csv", policy_profile_candidates)
    write_csv(output_dir / "robust_policy_representative_profiles.csv", policy_representative_profiles)
    write_csv(output_dir / "robust_policy_profile_transfer_replay.csv", policy_transfer_replay_rows)
    write_csv(output_dir / "robust_policy_profile_transfer_summary.csv", policy_transfer_summary_rows)
    write_csv(output_dir / "robust_policy_profile_transfer_diagnostics.csv", transfer_diagnostic_rows)
    write_csv(output_dir / "scenario_profile_catalog.csv", scenario_catalog_rows)
    write_csv(output_dir / "scenario_profile_catalog_promotion_checks.csv", scenario_catalog_promotion_rows)
    write_csv(output_dir / "scenario_profile_catalog_transfer_replay.csv", catalog_transfer_replay_rows)
    write_csv(output_dir / "scenario_profile_catalog_transfer_matrix.csv", catalog_transfer_pair_summary_rows)
    write_csv(output_dir / "scenario_profile_catalog_transfer_coverage.csv", catalog_transfer_source_summary_rows)
    write_csv(output_dir / "scenario_profile_catalog_minimal_set.csv", catalog_minimal_profile_set_rows)
    write_csv(output_dir / "pooled_scenario_profile_transfer_replay.csv", pooled_profile_replay_rows)
    write_csv(output_dir / "pooled_scenario_profile_summary.csv", pooled_profile_summary_rows)
    write_csv(output_dir / "pooled_scenario_profile_transfer_diagnostics.csv", pooled_profile_diagnostic_rows)
    write_json(output_dir / "robust_policy_profile_transfer_diagnostics.json", transfer_diagnostic_summary)
    write_json(output_dir / "pooled_scenario_profile_transfer_diagnostics.json", pooled_profile_diagnostic_summary)
    write_json(
        output_dir / "scenario_profile_catalog_transfer_summary.json",
        {
            "profile_replay_count": len(catalog_transfer_replay_rows),
            "profile_pair_count": len(catalog_transfer_pair_summary_rows),
            "profile_source_count": len(catalog_transfer_source_summary_rows),
            "pass_all_pair_count": sum(1 for row in catalog_transfer_pair_summary_rows if parse_bool(row.get("pass_all"))),
            "best_source_profiles": catalog_transfer_source_summary_rows[:5],
            "axis_contract": RDF_AXIS_CONTRACT,
            "runtime_input_contract": "catalog transfer diagnostic; runtime profile application uses raw RDF/disparity only",
        },
    )
    write_json(output_dir / "scenario_profile_catalog_minimal_set.json", catalog_minimal_profile_set_summary)
    write_json(output_dir / "scenario_near_tie_best_profile.json", best_near_tie_profile)
    write_json(output_dir / "scenario_near_tie_best_promotion_check.json", best_near_tie_promotion_check)
    write_json(output_dir / "pooled_scenario_profile_summary.json", pooled_profile_summary)
    write_json(output_dir / "deployment_strategy_recommendation.json", deployment_strategy)
    write_json(
        output_dir / "scenario_near_tie_profiles.json",
        {
            "profile_count": len(scenario_near_tie_profiles),
            "profiles": scenario_near_tie_profiles,
            "axis_contract": RDF_AXIS_CONTRACT,
            "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
        },
    )
    write_json(
        output_dir / "scenario_near_tie_profile_transfer_summary.json",
        {
            "transfer_replay_count": len(scenario_near_tie_transfer_replay_rows),
            "profile_count": len(scenario_near_tie_transfer_summary_rows),
            "pass_all_profile_count": sum(
                1 for row in scenario_near_tie_transfer_summary_rows
                if bool(row.get("pass_all_transfer_replays"))
            ),
            "best_profiles": scenario_near_tie_transfer_summary_rows[:8],
            "axis_contract": RDF_AXIS_CONTRACT,
            "runtime_input_contract": "near-tie transfer diagnostic; runtime uses raw stereo-center RDF(right,down,forward) plus measured disparity and no truth",
        },
    )
    write_json(
        output_dir / "scenario_near_tie_method_transfer_summary.json",
        {
            "method_count": len(scenario_near_tie_method_summary_rows),
            "methods": scenario_near_tie_method_summary_rows,
            "axis_contract": RDF_AXIS_CONTRACT,
            "runtime_input_contract": "method-level near-tie transfer diagnostic; runtime profiles use raw RDF/disparity only",
        },
    )
    write_deployment_strategy_txt(output_dir / "deployment_strategy_recommendation.txt", deployment_strategy)
    write_json(
        output_dir / "robustness_summary.json",
        {
            "scenario_count": len(scenarios),
            "scenarios": [
                {
                    "name": scenario.name,
                    "purpose": scenario.purpose,
                    "args": list(scenario.args),
                }
                for scenario in scenarios
            ],
            "ned_axis_contract": NED_AXIS_CONTRACT,
            "rdf_axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "scenario_runs": run_rows,
            "scenario_recommendations": scenario_recommendations,
            "robust_method_summary": robust_summary,
            "robust_method_family_summary": robust_family_summary,
            "robust_recommendation": robust_recommendation,
            "robust_policy_recommendations": robust_policy_rows,
            "scenario_near_tie_profiles": scenario_near_tie_profiles,
            "scenario_near_tie_profile_transfer_summary": scenario_near_tie_transfer_summary_rows,
            "scenario_near_tie_method_transfer_summary": scenario_near_tie_method_summary_rows,
            "scenario_near_tie_best_profile": best_near_tie_profile,
            "scenario_near_tie_best_promotion_check": best_near_tie_promotion_check,
            "robust_policy_profile_candidates": policy_profile_candidates,
            "robust_policy_representative_profiles": policy_representative_profiles,
            "robust_policy_profile_transfer_summary": policy_transfer_summary_rows,
            "robust_policy_profile_transfer_diagnostics": transfer_diagnostic_summary,
            "scenario_profile_catalog": scenario_catalog_summary,
            "scenario_profile_catalog_promotion_checks": scenario_catalog_promotion_summary,
            "scenario_profile_catalog_transfer_coverage": catalog_transfer_source_summary_rows,
            "scenario_profile_catalog_minimal_set": catalog_minimal_profile_set_summary,
            "pooled_scenario_profile_summary": pooled_profile_summary,
            "pooled_scenario_profile_transfer_diagnostics": pooled_profile_diagnostic_summary,
            "deployment_strategy_recommendation": deployment_strategy,
        },
    )
    write_json(output_dir / "scenario_profile_catalog.json", scenario_catalog_summary)
    write_json(output_dir / "scenario_profile_catalog_promotion_checks.json", scenario_catalog_promotion_summary)
    write_summary_txt(
        output_dir / "robustness_summary.txt",
        run_rows=run_rows,
        recommendation_rows=scenario_recommendations,
        robust_summary_rows=robust_summary,
        robust_family_summary_rows=robust_family_summary,
        robust_recommendation=robust_recommendation,
        policy_rows=robust_policy_rows,
        scenario_near_tie_profiles=scenario_near_tie_profiles,
        scenario_near_tie_transfer_summary_rows=scenario_near_tie_transfer_summary_rows,
        scenario_near_tie_method_summary_rows=scenario_near_tie_method_summary_rows,
        policy_profile_rows=policy_representative_profiles,
        policy_transfer_summary_rows=policy_transfer_summary_rows,
        transfer_diagnostic_rows=transfer_diagnostic_rows,
        scenario_catalog_rows=scenario_catalog_rows,
        scenario_catalog_summary=scenario_catalog_summary,
        scenario_catalog_promotion_summary=scenario_catalog_promotion_summary,
        catalog_transfer_source_summary_rows=catalog_transfer_source_summary_rows,
        catalog_minimal_profile_set_rows=catalog_minimal_profile_set_rows,
        catalog_minimal_profile_set_summary=catalog_minimal_profile_set_summary,
        deployment_strategy=deployment_strategy,
    )
    print(f"wrote robustness sweep summary to {output_dir}")
    return 0 if all(bool(row["pass_run"]) for row in run_rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
