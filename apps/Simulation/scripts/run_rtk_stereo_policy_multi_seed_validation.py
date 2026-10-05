#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
PIPELINE_SCRIPT = ROOT_DIR / "scripts" / "rtk_stereo_calibration_pipeline.py"
PROFILE_REPLAY_SCRIPT = ROOT_DIR / "scripts" / "stereodistance_profile_replay.py"
PROFILE_PROMOTION_CHECK_SCRIPT = ROOT_DIR / "scripts" / "stereodistance_profile_promotion_check.py"

from select_stereodistance_profile_from_catalog import method_contract_fields  # noqa: E402

RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
NED_AXIS_CONTRACT = "px4_local_position_ned:north,east,down"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
TRUTH_RDF_TRANSFORM_CONTRACT = (
    "truth_rdf must be stereo_center_RDF from explicit T_source_to_stereo_rdf "
    "or an already stereo-frame truth producer; direct NED-to-RDF axis alias is forbidden"
)
FORBIDDEN_DIRECT_ALIAS_TOKENS = (
    "synthetic_bridge: rdf_right=east",
    "rdf_right=east, rdf_down=down, rdf_forward=north",
)
FORBIDDEN_CONTEXT_TOKENS = (
    "forbidden",
    "not allowed",
    "invalid",
    "must not",
)
MULTI_SEED_ACCEPTANCE_ITEMS = (
    "seed_run_coverage",
    "coordinate_contract",
    "method_aggregate_summary",
    "policy_stability_gate",
    "method_stability_gate",
    "multi_seed_recommendation",
    "recommended_runtime_profiles",
    "representative_profile_checks",
    "representative_cross_seed_replay",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the RTK stereo calibration pipeline over multiple seeds and aggregate "
            "policy stability, method metrics, and target-window performance."
        )
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seeds", default="20260613,20260614,20260615")
    parser.add_argument("--mode", choices=("synthetic", "csv"), default="synthetic")
    parser.add_argument("--input-csv", type=Path)
    parser.add_argument("--sample-count", type=int, default=520)
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--synthetic-target-oversample-count", type=int, default=12)
    parser.add_argument("--baseline-scale-error", type=float, default=0.006)
    parser.add_argument("--focal-scale-error", type=float, default=-0.004)
    parser.add_argument("--yaw-error-deg", type=float, default=0.35)
    parser.add_argument("--pitch-error-deg", type=float, default=-0.18)
    parser.add_argument("--roll-error-deg", type=float, default=0.08)
    parser.add_argument("--disparity-bias-px", type=float, default=0.35)
    parser.add_argument("--disparity-noise-px", type=float, default=0.07)
    parser.add_argument("--bbox-noise-px", type=float, default=0.18)
    parser.add_argument("--disparity-outlier-rate", type=float, default=0.0)
    parser.add_argument("--disparity-outlier-px", type=float, default=0.0)
    parser.add_argument("--bbox-outlier-rate", type=float, default=0.0)
    parser.add_argument("--bbox-outlier-px", type=float, default=0.0)
    parser.add_argument("--min-validation-improvement-m", type=float, default=0.01)
    return parser.parse_args()


def resolve_runtime_paths(args: argparse.Namespace) -> argparse.Namespace:
    args.output_dir = args.output_dir.expanduser().resolve()
    if getattr(args, "input_csv", None) is not None:
        args.input_csv = args.input_csv.expanduser().resolve()
    return args


def parse_seed_list(value: str) -> list[int]:
    seeds = []
    for item in value.split(","):
        stripped = item.strip()
        if not stripped:
            continue
        seeds.append(int(stripped))
    if not seeds:
        raise ValueError("--seeds must contain at least one integer")
    return seeds


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


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


def finite_float(value: Any, default: float = math.nan) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def mean(values: Sequence[float]) -> float:
    return float(sum(values) / len(values)) if values else float("nan")


def finite_values(values: Sequence[float]) -> list[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def finite_mean(values: Sequence[float]) -> float:
    clean = finite_values(values)
    return mean(clean)


def collect_contract_text(payload: Mapping[str, object]) -> str:
    parts: list[str] = []
    for key in (
        "truth_rdf_transform_contract",
        "ned_to_rdf_contract",
        "truth_source_to_stereo_rdf_contract",
    ):
        value = payload.get(key)
        if value is not None:
            parts.append(str(value))
    contracts = payload.get("contracts")
    if isinstance(contracts, Mapping):
        for value in contracts.values():
            parts.append(str(value))
    return "\n".join(parts)


def direct_axis_alias_forbidden(payload: Mapping[str, object]) -> bool:
    for key in (
        "direct_ned_to_rdf_axis_alias_allowed",
        "direct_source_axis_alias_allowed",
        "direct_axis_alias_allowed",
    ):
        if bool(payload.get(key, False)):
            return False
    contracts = payload.get("contracts")
    if isinstance(contracts, Mapping):
        for key in (
            "direct_ned_to_rdf_axis_alias_allowed",
            "direct_source_axis_alias_allowed",
            "direct_axis_alias_allowed",
        ):
            if bool(contracts.get(key, False)):
                return False
    text = collect_contract_text(payload).lower()
    if not text:
        return False
    for token in FORBIDDEN_DIRECT_ALIAS_TOKENS:
        if token in text and not any(context in text for context in FORBIDDEN_CONTEXT_TOKENS):
            return False
    return True


def seed_contract_summary(seed_contract_rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    completed = [row for row in seed_contract_rows if bool(row.get("run_config_loaded"))]
    forbidden_count = sum(1 for row in completed if bool(row.get("direct_axis_alias_forbidden")))
    return {
        "seed_contract_count": len(seed_contract_rows),
        "loaded_contract_count": len(completed),
        "direct_axis_alias_forbidden_count": forbidden_count,
        "all_loaded_contracts_forbid_direct_axis_alias": bool(completed) and forbidden_count == len(completed),
        "truth_rdf_transform_contract": TRUTH_RDF_TRANSFORM_CONTRACT,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "ned_axis_contract": NED_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def bootstrap_mean_ci(
    values: Sequence[float],
    *,
    iterations: int = 1000,
    confidence: float = 0.90,
    seed: int = 20260613,
) -> tuple[float, float]:
    finite_values = np.array([float(value) for value in values if math.isfinite(float(value))], dtype=float)
    if finite_values.size == 0:
        return float("nan"), float("nan")
    if finite_values.size == 1:
        value = float(finite_values[0])
        return value, value
    rng = np.random.default_rng(seed)
    indices = rng.integers(0, finite_values.size, size=(max(1, iterations), finite_values.size))
    means = finite_values[indices].mean(axis=1)
    alpha = (1.0 - confidence) * 0.5
    return float(np.quantile(means, alpha)), float(np.quantile(means, 1.0 - alpha))


def run_pipeline(args: argparse.Namespace, seed: int, run_dir: Path) -> int:
    command = [
        sys.executable,
        str(PIPELINE_SCRIPT),
        "--mode",
        args.mode,
        "--output-dir",
        str(run_dir),
        "--seed",
        str(seed),
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
        "--baseline-scale-error",
        str(args.baseline_scale_error),
        "--focal-scale-error",
        str(args.focal_scale_error),
        "--yaw-error-deg",
        str(args.yaw_error_deg),
        "--pitch-error-deg",
        str(args.pitch_error_deg),
        "--roll-error-deg",
        str(args.roll_error_deg),
        "--disparity-bias-px",
        str(args.disparity_bias_px),
        "--disparity-noise-px",
        str(args.disparity_noise_px),
        "--bbox-noise-px",
        str(args.bbox_noise_px),
        "--disparity-outlier-rate",
        str(args.disparity_outlier_rate),
        "--disparity-outlier-px",
        str(args.disparity_outlier_px),
        "--bbox-outlier-rate",
        str(args.bbox_outlier_rate),
        "--bbox-outlier-px",
        str(args.bbox_outlier_px),
    ]
    if args.mode == "csv":
        if args.input_csv is None:
            raise ValueError("--input-csv is required for --mode csv")
        command.extend(["--input-csv", str(args.input_csv)])
    completed = subprocess.run(command, cwd=ROOT_DIR)
    return int(completed.returncode)


def aggregate_method_rows(per_seed_method_rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[str, list[Mapping[str, object]]] = defaultdict(list)
    for row in per_seed_method_rows:
        grouped[str(row["method"])].append(row)
    aggregate_rows = []
    for method, rows in grouped.items():
        validation_rmse = [finite_float(row["validation_range_rmse_m"]) for row in rows]
        validation_position = [finite_float(row.get("validation_position_rmse_m")) for row in rows]
        validation_right = [finite_float(row.get("validation_right_rmse_m")) for row in rows]
        validation_down = [finite_float(row.get("validation_down_rmse_m")) for row in rows]
        validation_forward = [finite_float(row.get("validation_forward_rmse_m")) for row in rows]
        target_abs = [finite_float(row["validation_target_abs_mean_error_m"]) for row in rows]
        complexity = [finite_float(row["model_complexity"]) for row in rows]
        selected_count = sum(1 for row in rows if str(row.get("is_final_recommendation", "")).lower() in ("1", "true", "yes"))
        validation_ci_low, validation_ci_high = bootstrap_mean_ci(validation_rmse)
        target_ci_low, target_ci_high = bootstrap_mean_ci(target_abs)
        finite_target_abs = finite_values(target_abs)
        aggregate_rows.append(
            {
                "method": method,
                **method_contract_fields(method),
                "seed_count": len(rows),
                "selected_count": selected_count,
                "target_abs_mean_error_finite_seed_count": len(finite_target_abs),
                "validation_range_rmse_mean_m": finite_mean(validation_rmse),
                "validation_range_rmse_mean_ci90_low_m": validation_ci_low,
                "validation_range_rmse_mean_ci90_high_m": validation_ci_high,
                "validation_range_rmse_max_m": float(max(finite_values(validation_rmse))) if finite_values(validation_rmse) else float("inf"),
                "validation_position_rmse_mean_m": finite_mean(validation_position),
                "validation_right_rmse_mean_m": finite_mean(validation_right),
                "validation_down_rmse_mean_m": finite_mean(validation_down),
                "validation_forward_rmse_mean_m": finite_mean(validation_forward),
                "target_abs_mean_error_mean_m": mean(finite_target_abs) if finite_target_abs else float("inf"),
                "target_abs_mean_error_mean_ci90_low_m": target_ci_low,
                "target_abs_mean_error_mean_ci90_high_m": target_ci_high,
                "target_abs_mean_error_max_m": float(max(finite_target_abs)) if finite_target_abs else float("inf"),
                "model_complexity_mean": finite_mean(complexity),
            }
        )
    aggregate_rows.sort(
        key=lambda row: (
            float(row["validation_range_rmse_mean_m"]),
            float(row["target_abs_mean_error_mean_m"]),
            float(row["model_complexity_mean"]),
        )
    )
    return aggregate_rows


def aggregate_policy_rows(per_seed_policy_rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[str, list[Mapping[str, object]]] = defaultdict(list)
    for row in per_seed_policy_rows:
        grouped[str(row["policy"])].append(row)
    aggregate_rows = []
    for policy, rows in grouped.items():
        methods = [str(row["method"]) for row in rows]
        method_counts = Counter(methods)
        validation_rmse = [finite_float(row["validation_range_rmse_m"]) for row in rows]
        validation_position = [finite_float(row.get("validation_position_rmse_m")) for row in rows]
        validation_right = [finite_float(row.get("validation_right_rmse_m")) for row in rows]
        validation_down = [finite_float(row.get("validation_down_rmse_m")) for row in rows]
        validation_forward = [finite_float(row.get("validation_forward_rmse_m")) for row in rows]
        target_abs = [finite_float(row["validation_target_abs_mean_error_m"]) for row in rows]
        most_common_method, most_common_count = method_counts.most_common(1)[0]
        validation_ci_low, validation_ci_high = bootstrap_mean_ci(validation_rmse)
        target_ci_low, target_ci_high = bootstrap_mean_ci(target_abs)
        finite_target_abs = finite_values(target_abs)
        aggregate_rows.append(
            {
                "policy": policy,
                "seed_count": len(rows),
                "unique_method_count": len(method_counts),
                "most_common_method": most_common_method,
                "most_common_method_count": most_common_count,
                "method_counts_json": json.dumps(dict(method_counts), sort_keys=True),
                "target_abs_mean_error_finite_seed_count": len(finite_target_abs),
                "validation_range_rmse_mean_m": finite_mean(validation_rmse),
                "validation_range_rmse_mean_ci90_low_m": validation_ci_low,
                "validation_range_rmse_mean_ci90_high_m": validation_ci_high,
                "validation_range_rmse_max_m": float(max(finite_values(validation_rmse))) if finite_values(validation_rmse) else float("inf"),
                "validation_position_rmse_mean_m": finite_mean(validation_position),
                "validation_right_rmse_mean_m": finite_mean(validation_right),
                "validation_down_rmse_mean_m": finite_mean(validation_down),
                "validation_forward_rmse_mean_m": finite_mean(validation_forward),
                "target_abs_mean_error_mean_m": mean(finite_target_abs) if finite_target_abs else float("inf"),
                "target_abs_mean_error_mean_ci90_low_m": target_ci_low,
                "target_abs_mean_error_mean_ci90_high_m": target_ci_high,
                "target_abs_mean_error_max_m": float(max(finite_target_abs)) if finite_target_abs else float("inf"),
            }
        )
    aggregate_rows.sort(key=lambda row: (str(row["policy"])))
    return aggregate_rows


def policy_stability_gate_rows(
    policy_rows: Sequence[Mapping[str, object]],
    method_rows: Sequence[Mapping[str, object]],
    *,
    completed_seed_count: int,
    target_tolerance_m: float = 10.0,
    min_validation_improvement_m: float = 0.01,
) -> list[dict[str, object]]:
    raw_row = next((row for row in method_rows if row["method"] == "raw"), None)
    raw_mean = finite_float(raw_row.get("validation_range_rmse_mean_m") if raw_row else None, float("inf"))
    raw_axis_means = {
        axis: finite_float(raw_row.get(f"validation_{axis}_rmse_mean_m") if raw_row else None, float("inf"))
        for axis in ("right", "down", "forward")
    }
    gate_rows = []
    for row in policy_rows:
        seed_count = int(row["seed_count"])
        most_common_count = int(row["most_common_method_count"])
        validation_mean = finite_float(row["validation_range_rmse_mean_m"], float("inf"))
        validation_ci_high = finite_float(row.get("validation_range_rmse_mean_ci90_high_m"), validation_mean)
        target_finite_count = int(row.get("target_abs_mean_error_finite_seed_count", 0))
        target_max = finite_float(row["target_abs_mean_error_max_m"], float("inf"))
        axis_regressions = {
            axis: max(
                0.0,
                finite_float(row.get(f"validation_{axis}_rmse_mean_m"), float("inf")) - raw_axis_means[axis],
            )
            for axis in ("right", "down", "forward")
        }
        axis_regression_max = max(axis_regressions.values()) if axis_regressions else float("inf")
        axis_regression_sum = sum(axis_regressions.values())
        axis_not_regressed = axis_regression_max <= min_validation_improvement_m
        run_coverage_ok = completed_seed_count == seed_count and seed_count > 0
        method_stable = most_common_count > seed_count / 2.0
        validation_improved = validation_mean <= raw_mean - min_validation_improvement_m
        validation_ci_improved = validation_ci_high <= raw_mean - min_validation_improvement_m
        target_coverage_ok = target_finite_count == completed_seed_count and completed_seed_count > 0
        target_ok = target_coverage_ok and target_max <= target_tolerance_m
        strict_pass = run_coverage_ok and method_stable and validation_improved and validation_ci_improved and target_ok
        performance_pass = run_coverage_ok and validation_improved and target_ok
        reasons = []
        if not run_coverage_ok:
            reasons.append("incomplete_seed_coverage")
        if not method_stable:
            reasons.append("policy_method_not_majority_stable")
        if not validation_improved:
            reasons.append("policy_mean_validation_rmse_not_better_than_raw")
        if not validation_ci_improved:
            reasons.append("policy_ci90_high_not_better_than_raw")
        if not target_coverage_ok:
            reasons.append("policy_target_window_missing_in_some_seeds")
        if target_coverage_ok and not target_ok:
            reasons.append("policy_target_error_exceeds_tolerance")
        gate_rows.append(
            {
                "policy": row["policy"],
                "strict_pass": strict_pass,
                "performance_pass": performance_pass,
                "run_coverage_ok": run_coverage_ok,
                "method_stable": method_stable,
                "validation_improved_vs_raw_mean": validation_improved,
                "validation_ci90_high_improved_vs_raw_mean": validation_ci_improved,
                "target_window_coverage_ok": target_coverage_ok,
                "target_abs_mean_error_finite_seed_count": target_finite_count,
                "target_error_within_tolerance": target_ok,
                "axis_rmse_not_regressed_vs_raw_mean": axis_not_regressed,
                "axis_rmse_regression_max_m": axis_regression_max,
                "axis_rmse_regression_sum_m": axis_regression_sum,
                "policy_validation_position_rmse_mean_m": finite_float(row.get("validation_position_rmse_mean_m"), float("inf")),
                "policy_validation_right_rmse_mean_m": finite_float(row.get("validation_right_rmse_mean_m"), float("inf")),
                "policy_validation_down_rmse_mean_m": finite_float(row.get("validation_down_rmse_mean_m"), float("inf")),
                "policy_validation_forward_rmse_mean_m": finite_float(row.get("validation_forward_rmse_mean_m"), float("inf")),
                "most_common_method": row["most_common_method"],
                "most_common_method_count": most_common_count,
                "seed_count": seed_count,
                "raw_validation_range_rmse_mean_m": raw_mean,
                "policy_validation_range_rmse_mean_m": validation_mean,
                "policy_validation_range_rmse_mean_ci90_high_m": validation_ci_high,
                "policy_target_abs_mean_error_max_m": target_max,
                "target_tolerance_m": target_tolerance_m,
                "min_validation_improvement_m": min_validation_improvement_m,
                "reasons": ";".join(reasons),
            }
        )
    return gate_rows


def method_stability_gate_rows(
    method_rows: Sequence[Mapping[str, object]],
    *,
    completed_seed_count: int,
    target_tolerance_m: float = 10.0,
    min_validation_improvement_m: float = 0.01,
) -> list[dict[str, object]]:
    raw_row = next((row for row in method_rows if row["method"] == "raw"), None)
    raw_mean = finite_float(raw_row.get("validation_range_rmse_mean_m") if raw_row else None, float("inf"))
    raw_axis_means = {
        axis: finite_float(raw_row.get(f"validation_{axis}_rmse_mean_m") if raw_row else None, float("inf"))
        for axis in ("right", "down", "forward")
    }
    gate_rows = []
    for row in method_rows:
        method = str(row["method"])
        seed_count = int(row["seed_count"])
        validation_mean = finite_float(row["validation_range_rmse_mean_m"], float("inf"))
        validation_ci_high = finite_float(row.get("validation_range_rmse_mean_ci90_high_m"), validation_mean)
        validation_max = finite_float(row.get("validation_range_rmse_max_m"), validation_mean)
        target_mean = finite_float(row["target_abs_mean_error_mean_m"], float("inf"))
        target_max = finite_float(row["target_abs_mean_error_max_m"], float("inf"))
        target_finite_count = int(row.get("target_abs_mean_error_finite_seed_count", 0))
        complexity = finite_float(row.get("model_complexity_mean"), float("inf"))
        axis_regressions = {
            axis: max(
                0.0,
                finite_float(row.get(f"validation_{axis}_rmse_mean_m"), float("inf")) - raw_axis_means[axis],
            )
            for axis in ("right", "down", "forward")
        }
        axis_regression_max = max(axis_regressions.values()) if axis_regressions else float("inf")
        axis_regression_sum = sum(axis_regressions.values())
        axis_not_regressed = method != "raw" and axis_regression_max <= min_validation_improvement_m
        run_coverage_ok = completed_seed_count == seed_count and seed_count > 0
        validation_improved = method != "raw" and validation_mean <= raw_mean - min_validation_improvement_m
        validation_ci_improved = method != "raw" and validation_ci_high <= raw_mean - min_validation_improvement_m
        target_coverage_ok = target_finite_count == completed_seed_count and completed_seed_count > 0
        target_ok = target_coverage_ok and target_max <= target_tolerance_m
        strict_pass = run_coverage_ok and validation_improved and validation_ci_improved and target_ok
        performance_pass = run_coverage_ok and validation_improved and target_ok
        reasons = []
        if method == "raw":
            reasons.append("raw_is_baseline_not_correction_candidate")
        if not run_coverage_ok:
            reasons.append("incomplete_seed_coverage")
        if not validation_improved:
            reasons.append("method_mean_validation_rmse_not_better_than_raw")
        if not validation_ci_improved:
            reasons.append("method_ci90_high_not_better_than_raw")
        if not target_coverage_ok:
            reasons.append("method_target_window_missing_in_some_seeds")
        if target_coverage_ok and not target_ok:
            reasons.append("method_target_error_exceeds_tolerance")
        gate_rows.append(
            {
                "method": method,
                "registration_model": str(row.get("registration_model", "")),
                "residual_model": str(row.get("residual_model", "")),
                "regularization_model": str(row.get("regularization_model", "")),
                "runtime_profile_family": str(row.get("runtime_profile_family", "")),
                "selection_note": str(row.get("selection_note", "")),
                "strict_pass": strict_pass,
                "performance_pass": performance_pass,
                "run_coverage_ok": run_coverage_ok,
                "validation_improved_vs_raw_mean": validation_improved,
                "validation_ci90_high_improved_vs_raw_mean": validation_ci_improved,
                "target_window_coverage_ok": target_coverage_ok,
                "target_abs_mean_error_finite_seed_count": target_finite_count,
                "target_error_within_tolerance": target_ok,
                "axis_rmse_not_regressed_vs_raw_mean": axis_not_regressed,
                "axis_rmse_regression_max_m": axis_regression_max,
                "axis_rmse_regression_sum_m": axis_regression_sum,
                "method_validation_position_rmse_mean_m": finite_float(row.get("validation_position_rmse_mean_m"), float("inf")),
                "method_validation_right_rmse_mean_m": finite_float(row.get("validation_right_rmse_mean_m"), float("inf")),
                "method_validation_down_rmse_mean_m": finite_float(row.get("validation_down_rmse_mean_m"), float("inf")),
                "method_validation_forward_rmse_mean_m": finite_float(row.get("validation_forward_rmse_mean_m"), float("inf")),
                "seed_count": seed_count,
                "selected_count": int(row.get("selected_count", 0)),
                "raw_validation_range_rmse_mean_m": raw_mean,
                "method_validation_range_rmse_mean_m": validation_mean,
                "method_validation_range_rmse_mean_ci90_high_m": validation_ci_high,
                "method_validation_range_rmse_max_m": validation_max,
                "method_target_abs_mean_error_mean_m": target_mean,
                "method_target_abs_mean_error_max_m": target_max,
                "model_complexity_mean": complexity,
                "target_tolerance_m": target_tolerance_m,
                "min_validation_improvement_m": min_validation_improvement_m,
                "reasons": ";".join(reasons),
            }
        )
    return gate_rows


SELECTION_SCORE_CONTRACT = (
    "validation_mean + 0.20*target_mean + 0.15*target_max + "
    "0.05*validation_ci_width + 0.03*validation_worst_seed_gap + "
    "0.10*axis_regression_sum + 0.005*model_complexity"
)


def multi_seed_selection_score(row: Mapping[str, object]) -> float:
    validation_mean = finite_float(row["method_validation_range_rmse_mean_m"], float("inf"))
    validation_max = finite_float(row.get("method_validation_range_rmse_max_m"), validation_mean)
    target_mean = finite_float(row["method_target_abs_mean_error_mean_m"], float("inf"))
    target_max = finite_float(row.get("method_target_abs_mean_error_max_m"), target_mean)
    validation_ci_high = finite_float(row["method_validation_range_rmse_mean_ci90_high_m"], validation_mean)
    validation_ci_width = max(0.0, validation_ci_high - validation_mean)
    complexity = finite_float(row["model_complexity_mean"], 0.0)
    axis_regression_sum = finite_float(row.get("axis_rmse_regression_sum_m"), 0.0)
    return (
        validation_mean
        + 0.20 * target_mean
        + 0.15 * target_max
        + 0.05 * validation_ci_width
        + 0.03 * max(0.0, validation_max - validation_mean)
        + 0.10 * axis_regression_sum
        + 0.005 * complexity
    )


def annotate_method_selection_scores(method_gate_rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    rows = [dict(row) for row in method_gate_rows]
    scoreable = [
        row for row in rows
        if str(row.get("method", "")) != "raw"
        and (bool(row.get("strict_pass")) or bool(row.get("performance_pass")))
    ]
    ranked = sorted(
        scoreable,
        key=lambda row: (
            multi_seed_selection_score(row),
            finite_float(row["method_validation_range_rmse_mean_m"], float("inf")),
            finite_float(row.get("method_target_abs_mean_error_max_m"), float("inf")),
            finite_float(row.get("method_validation_range_rmse_max_m"), float("inf")),
            finite_float(row["method_target_abs_mean_error_mean_m"], float("inf")),
            finite_float(row["model_complexity_mean"], float("inf")),
            str(row["method"]),
        ),
    )
    rank_by_method = {str(row["method"]): index for index, row in enumerate(ranked, start=1)}
    score_by_method = {str(row["method"]): multi_seed_selection_score(row) for row in ranked}
    for row in rows:
        method = str(row.get("method", ""))
        row["selection_score_contract"] = SELECTION_SCORE_CONTRACT
        row["selection_score"] = score_by_method.get(method, float("inf"))
        row["selection_rank"] = rank_by_method.get(method, "")
    return rows


def selection_near_tie_rows(
    method_gate_rows: Sequence[Mapping[str, object]],
    *,
    score_delta_threshold: float = 0.02,
    max_rank: int = 5,
) -> list[dict[str, object]]:
    ranked = [
        row for row in method_gate_rows
        if str(row.get("selection_rank", "")) != ""
        and math.isfinite(finite_float(row.get("selection_score"), float("nan")))
    ]
    ranked.sort(key=lambda row: int(row["selection_rank"]))
    if not ranked:
        return []
    winner = ranked[0]
    winner_score = finite_float(winner.get("selection_score"), float("inf"))
    winner_validation = finite_float(winner.get("method_validation_range_rmse_mean_m"), float("inf"))
    winner_target_max = finite_float(winner.get("method_target_abs_mean_error_max_m"), float("inf"))
    threshold = max(0.0, float(score_delta_threshold))
    max_rank = max(1, int(max_rank))
    rows: list[dict[str, object]] = []
    for row in ranked:
        rank = int(row["selection_rank"])
        score = finite_float(row.get("selection_score"), float("inf"))
        score_delta = score - winner_score
        include = rank <= max_rank or score_delta <= threshold
        if not include:
            continue
        validation = finite_float(row.get("method_validation_range_rmse_mean_m"), float("inf"))
        target_max = finite_float(row.get("method_target_abs_mean_error_max_m"), float("inf"))
        rows.append(
            {
                "selection_rank": rank,
                "method": str(row.get("method", "")),
                "selection_score": score,
                "selection_score_delta_from_winner": score_delta,
                "within_near_tie_threshold": score_delta <= threshold,
                "near_tie_score_delta_threshold": threshold,
                "strict_pass": bool(row.get("strict_pass")),
                "performance_pass": bool(row.get("performance_pass")),
                "registration_model": str(row.get("registration_model", "")),
                "residual_model": str(row.get("residual_model", "")),
                "regularization_model": str(row.get("regularization_model", "")),
                "runtime_profile_family": str(row.get("runtime_profile_family", "")),
                "method_validation_range_rmse_mean_m": validation,
                "validation_rmse_delta_from_winner_m": validation - winner_validation,
                "method_target_abs_mean_error_max_m": target_max,
                "target_abs_max_delta_from_winner_m": target_max - winner_target_max,
                "model_complexity_mean": finite_float(row.get("model_complexity_mean"), float("inf")),
                "selection_score_contract": str(row.get("selection_score_contract", SELECTION_SCORE_CONTRACT)),
            }
        )
    return rows


def choose_multi_seed_recommendation(method_gate_rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    strict_candidates = [row for row in method_gate_rows if bool(row.get("strict_pass"))]
    performance_candidates = [
        row for row in method_gate_rows
        if bool(row.get("performance_pass")) and str(row.get("method")) != "raw"
    ]
    mode = "strict_pass" if strict_candidates else "performance_pass" if performance_candidates else "none"
    candidates = strict_candidates if strict_candidates else performance_candidates
    if not candidates:
        return {
            "recommendation_mode": "none",
            "method": "",
            "score": None,
            "reason": "no_method_passed_multi_seed_gate",
        }

    ranked = sorted(
        candidates,
        key=lambda row: (
            multi_seed_selection_score(row),
            finite_float(row["method_validation_range_rmse_mean_m"], float("inf")),
            finite_float(row.get("method_target_abs_mean_error_max_m"), float("inf")),
            finite_float(row.get("method_validation_range_rmse_max_m"), float("inf")),
            finite_float(row["method_target_abs_mean_error_mean_m"], float("inf")),
            finite_float(row["model_complexity_mean"], float("inf")),
            str(row["method"]),
        ),
    )
    selected = ranked[0]
    return {
        "recommendation_mode": mode,
        "method": selected["method"],
        "score": multi_seed_selection_score(selected),
        "reason": "selected_from_multi_seed_method_gate",
        "strict_pass": bool(selected.get("strict_pass")),
        "performance_pass": bool(selected.get("performance_pass")),
        "validation_range_rmse_mean_m": selected["method_validation_range_rmse_mean_m"],
        "validation_range_rmse_mean_ci90_high_m": selected["method_validation_range_rmse_mean_ci90_high_m"],
        "target_abs_mean_error_mean_m": selected["method_target_abs_mean_error_mean_m"],
        "target_abs_mean_error_max_m": selected["method_target_abs_mean_error_max_m"],
        "validation_range_rmse_max_m": selected.get("method_validation_range_rmse_max_m"),
        "axis_rmse_regression_max_m": selected.get("axis_rmse_regression_max_m"),
        "axis_rmse_regression_sum_m": selected.get("axis_rmse_regression_sum_m"),
        "model_complexity_mean": selected["model_complexity_mean"],
        "score_contract": SELECTION_SCORE_CONTRACT,
    }


def build_multi_seed_acceptance_matrix(
    *,
    seed_count: int,
    completed_seed_count: int,
    contract_summary: Mapping[str, object],
    method_rows: Sequence[Mapping[str, object]],
    policy_gate_rows: Sequence[Mapping[str, object]],
    method_gate_rows: Sequence[Mapping[str, object]],
    recommendation: Mapping[str, object],
    recommended_profiles: Sequence[Mapping[str, object]],
    representative_checks: Mapping[str, object],
    cross_seed_replay: Mapping[str, object],
) -> dict[str, dict[str, object]]:
    strict_policy_count = sum(1 for row in policy_gate_rows if bool(row.get("strict_pass")))
    performance_policy_count = sum(1 for row in policy_gate_rows if bool(row.get("performance_pass")))
    strict_method_count = sum(1 for row in method_gate_rows if bool(row.get("strict_pass")))
    performance_method_count = sum(1 for row in method_gate_rows if bool(row.get("performance_pass")))
    recommended_method = str(recommendation.get("method", ""))
    profile_exists_count = sum(1 for row in recommended_profiles if bool(row.get("profile_exists")))
    return {
        "seed_run_coverage": {
            "status": "PASS" if seed_count > 0 and completed_seed_count == seed_count else "FAIL",
            "path": "seed_runs.csv",
            "evidence": {
                "seed_count": seed_count,
                "completed_seed_count": completed_seed_count,
            },
        },
        "coordinate_contract": {
            "status": "PASS" if bool(contract_summary.get("all_loaded_contracts_forbid_direct_axis_alias")) else "FAIL",
            "path": "seed_coordinate_contracts.csv",
            "evidence": {
                "loaded_contract_count": contract_summary.get("loaded_contract_count", 0),
                "direct_axis_alias_forbidden_count": contract_summary.get(
                    "direct_axis_alias_forbidden_count",
                    0,
                ),
                "truth_rdf_transform_contract": contract_summary.get(
                    "truth_rdf_transform_contract",
                    TRUTH_RDF_TRANSFORM_CONTRACT,
                ),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            },
        },
        "method_aggregate_summary": {
            "status": "PASS" if bool(method_rows) and any(str(row.get("method")) == "raw" for row in method_rows) else "FAIL",
            "path": "method_aggregate_summary.csv",
            "evidence": {
                "method_count": len(method_rows),
                "has_raw_baseline": any(str(row.get("method")) == "raw" for row in method_rows),
            },
        },
        "policy_stability_gate": {
            "status": "PASS" if bool(policy_gate_rows) and performance_policy_count > 0 else "FAIL",
            "path": "policy_stability_gate.csv",
            "evidence": {
                "policy_count": len(policy_gate_rows),
                "strict_pass_count": strict_policy_count,
                "performance_pass_count": performance_policy_count,
            },
        },
        "method_stability_gate": {
            "status": "PASS" if bool(method_gate_rows) and performance_method_count > 0 else "FAIL",
            "path": "method_stability_gate.csv",
            "evidence": {
                "method_count": len(method_gate_rows),
                "strict_pass_count": strict_method_count,
                "performance_pass_count": performance_method_count,
            },
        },
        "multi_seed_recommendation": {
            "status": "PASS" if bool(recommended_method) else "FAIL",
            "path": "multi_seed_recommendation.json",
            "evidence": {
                "method": recommended_method,
                "recommendation_mode": recommendation.get("recommendation_mode", ""),
                "score": recommendation.get("score"),
                "reason": recommendation.get("reason", ""),
            },
        },
        "recommended_runtime_profiles": {
            "status": "PASS" if bool(recommended_profiles) and profile_exists_count == len(recommended_profiles) else "FAIL",
            "path": "multi_seed_recommended_profiles.csv",
            "evidence": {
                "profile_count": len(recommended_profiles),
                "profile_exists_count": profile_exists_count,
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
            },
        },
        "representative_profile_checks": {
            "status": "PASS" if bool(representative_checks.get("pass_representative_profile_checks")) else "FAIL",
            "path": "representative_profile_checks.json",
            "evidence": {
                "replay_pass": bool(representative_checks.get("replay_pass")),
                "promotion_pass": bool(representative_checks.get("promotion_pass")),
                "validation_range_rmse_raw_m": representative_checks.get("validation_range_rmse_raw_m"),
                "validation_range_rmse_corrected_m": representative_checks.get("validation_range_rmse_corrected_m"),
                "target_abs_mean_error_corrected_m": representative_checks.get("target_abs_mean_error_corrected_m"),
            },
        },
        "representative_cross_seed_replay": {
            "status": "PASS" if bool(cross_seed_replay.get("pass_cross_seed_replay")) else "FAIL",
            "path": "representative_cross_seed_replay_summary.csv",
            "evidence": {
                "seed_count": cross_seed_replay.get("seed_count", 0),
                "passed_seed_count": cross_seed_replay.get("passed_seed_count", 0),
            },
        },
    }


def method_family_diagnostic_rows(method_gate_rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str, str, str], list[Mapping[str, object]]] = defaultdict(list)
    for row in method_gate_rows:
        if str(row.get("method", "")) == "raw":
            continue
        grouped[
            (
                str(row.get("registration_model", "")),
                str(row.get("residual_model", "")),
                str(row.get("regularization_model", "")),
                str(row.get("runtime_profile_family", "")),
            )
        ].append(row)

    diagnostic_rows: list[dict[str, object]] = []
    for (
        registration_model,
        residual_model,
        regularization_model,
        runtime_profile_family,
    ), rows in grouped.items():
        sorted_rows = sorted(
            rows,
            key=lambda row: (
                finite_float(row.get("method_validation_range_rmse_mean_m"), float("inf")),
                finite_float(row.get("method_target_abs_mean_error_max_m"), float("inf")),
                finite_float(row.get("model_complexity_mean"), float("inf")),
                str(row.get("method", "")),
            ),
        )
        best = sorted_rows[0]
        strict_pass_rows = [row for row in rows if bool(row.get("strict_pass"))]
        performance_pass_rows = [row for row in rows if bool(row.get("performance_pass"))]
        validation_values = [
            finite_float(row.get("method_validation_range_rmse_mean_m"), float("nan"))
            for row in rows
        ]
        target_values = [
            finite_float(row.get("method_target_abs_mean_error_max_m"), float("nan"))
            for row in rows
        ]
        complexity_values = [
            finite_float(row.get("model_complexity_mean"), float("nan"))
            for row in rows
        ]
        diagnostic_rows.append(
            {
                "family_id": "|".join(
                    (
                        registration_model,
                        residual_model,
                        regularization_model,
                        runtime_profile_family,
                    )
                ),
                "registration_model": registration_model,
                "residual_model": residual_model,
                "regularization_model": regularization_model,
                "runtime_profile_family": runtime_profile_family,
                "method_count": len(rows),
                "strict_pass_count": len(strict_pass_rows),
                "performance_pass_count": len(performance_pass_rows),
                "best_method": str(best.get("method", "")),
                "best_strict_pass": bool(best.get("strict_pass")),
                "best_performance_pass": bool(best.get("performance_pass")),
                "best_validation_range_rmse_mean_m": finite_float(
                    best.get("method_validation_range_rmse_mean_m"),
                    float("inf"),
                ),
                "best_target_abs_mean_error_max_m": finite_float(
                    best.get("method_target_abs_mean_error_max_m"),
                    float("inf"),
                ),
                "best_model_complexity_mean": finite_float(best.get("model_complexity_mean"), float("inf")),
                "family_validation_range_rmse_mean_m": finite_mean(validation_values),
                "family_target_abs_mean_error_max_mean_m": finite_mean(target_values),
                "family_model_complexity_mean": finite_mean(complexity_values),
                "selection_note": str(best.get("selection_note", "")),
                "top_methods_json": json.dumps(
                    [
                        {
                            "method": str(row.get("method", "")),
                            "strict_pass": bool(row.get("strict_pass")),
                            "validation_range_rmse_mean_m": finite_float(
                                row.get("method_validation_range_rmse_mean_m"),
                                float("inf"),
                            ),
                            "target_abs_mean_error_max_m": finite_float(
                                row.get("method_target_abs_mean_error_max_m"),
                                float("inf"),
                            ),
                        }
                        for row in sorted_rows[:5]
                    ],
                    sort_keys=True,
                ),
            }
        )
    diagnostic_rows.sort(
        key=lambda row: (
            not bool(row["best_strict_pass"]),
            finite_float(row["best_validation_range_rmse_mean_m"], float("inf")),
            finite_float(row["best_target_abs_mean_error_max_m"], float("inf")),
            finite_float(row["best_model_complexity_mean"], float("inf")),
            str(row["family_id"]),
        )
    )
    for index, row in enumerate(diagnostic_rows, start=1):
        row["family_rank"] = index
    return diagnostic_rows


def recommended_profile_rows(
    *,
    output_dir: Path,
    run_rows: Sequence[Mapping[str, object]],
    per_seed_method_rows: Sequence[Mapping[str, object]],
    recommendation: Mapping[str, object],
) -> list[dict[str, object]]:
    method_name = str(recommendation.get("method") or "")
    if not method_name:
        return []
    method_rows_by_seed = {
        int(row["seed"]): row
        for row in per_seed_method_rows
        if str(row.get("method")) == method_name
    }
    rows: list[dict[str, object]] = []
    for run_row in run_rows:
        seed = int(run_row["seed"])
        if not bool(run_row.get("pass_run")):
            continue
        run_dir = Path(str(run_row["run_dir"]))
        method_row = method_rows_by_seed.get(seed)
        if method_row is None:
            continue
        profile_path = run_dir / "runtime_profiles" / f"{method_name}.json"
        rows.append(
            {
                "seed": seed,
                "method": method_name,
                "profile_path": str(profile_path.relative_to(output_dir)) if profile_path.exists() else "",
                "profile_exists": profile_path.exists(),
                "is_seed_final_recommendation": str(method_row.get("is_final_recommendation", "")).lower() in ("1", "true", "yes"),
                "validation_range_rmse_m": finite_float(method_row.get("validation_range_rmse_m"), float("inf")),
                "validation_position_rmse_m": finite_float(method_row.get("validation_position_rmse_m"), float("inf")),
                "validation_right_rmse_m": finite_float(method_row.get("validation_right_rmse_m"), float("inf")),
                "validation_down_rmse_m": finite_float(method_row.get("validation_down_rmse_m"), float("inf")),
                "validation_forward_rmse_m": finite_float(method_row.get("validation_forward_rmse_m"), float("inf")),
                "validation_target_abs_mean_error_m": finite_float(method_row.get("validation_target_abs_mean_error_m"), float("inf")),
                "validation_target_window_count": int(float(method_row.get("validation_target_window_count", 0) or 0)),
                "model_complexity": int(float(method_row.get("model_complexity", 0) or 0)),
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
            }
        )
    return rows


def near_tie_profile_candidate_rows(
    *,
    output_dir: Path,
    run_rows: Sequence[Mapping[str, object]],
    per_seed_method_rows: Sequence[Mapping[str, object]],
    near_tie_rows: Sequence[Mapping[str, object]],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for near_tie in near_tie_rows:
        method = str(near_tie.get("method", ""))
        if not method:
            continue
        profile_rows = recommended_profile_rows(
            output_dir=output_dir,
            run_rows=run_rows,
            per_seed_method_rows=per_seed_method_rows,
            recommendation={"method": method},
        )
        for profile_row in profile_rows:
            row_out = dict(profile_row)
            row_out.update(
                {
                    "selection_rank": near_tie.get("selection_rank", ""),
                    "selection_score": near_tie.get("selection_score", float("inf")),
                    "selection_score_delta_from_winner": near_tie.get(
                        "selection_score_delta_from_winner",
                        float("inf"),
                    ),
                    "within_near_tie_threshold": bool(near_tie.get("within_near_tie_threshold")),
                    "near_tie_score_delta_threshold": near_tie.get("near_tie_score_delta_threshold", ""),
                    "registration_model": near_tie.get("registration_model", ""),
                    "residual_model": near_tie.get("residual_model", ""),
                    "regularization_model": near_tie.get("regularization_model", ""),
                    "runtime_profile_family": near_tie.get("runtime_profile_family", ""),
                }
            )
            rows.append(row_out)
    rows.sort(
        key=lambda row: (
            int(row.get("selection_rank") or 999999),
            int(row.get("seed") or 0),
        )
    )
    return rows


def choose_representative_profile(
    profile_rows: Sequence[Mapping[str, object]],
    recommendation: Mapping[str, object],
) -> dict[str, object]:
    candidates = [
        row for row in profile_rows
        if bool(row.get("profile_exists"))
        and math.isfinite(finite_float(row.get("validation_range_rmse_m"), float("nan")))
        and math.isfinite(finite_float(row.get("validation_target_abs_mean_error_m"), float("nan")))
    ]
    if not candidates:
        return {
            "selected": False,
            "reason": "no_existing_seed_profile_for_recommended_method",
        }
    target_validation = finite_float(
        recommendation.get("validation_range_rmse_mean_m"),
        finite_mean([finite_float(row.get("validation_range_rmse_m"), float("inf")) for row in candidates]),
    )
    target_abs = finite_float(
        recommendation.get("target_abs_mean_error_mean_m"),
        finite_mean([finite_float(row.get("validation_target_abs_mean_error_m"), float("inf")) for row in candidates]),
    )
    validation_values = finite_values([finite_float(row.get("validation_range_rmse_m"), float("inf")) for row in candidates])
    target_values = finite_values([finite_float(row.get("validation_target_abs_mean_error_m"), float("inf")) for row in candidates])
    validation_scale = max(1.0e-9, max(validation_values) - min(validation_values)) if validation_values else 1.0
    target_scale = max(1.0e-9, max(target_values) - min(target_values)) if target_values else 1.0

    def score(row: Mapping[str, object]) -> float:
        validation_delta = abs(finite_float(row.get("validation_range_rmse_m"), float("inf")) - target_validation) / validation_scale
        target_delta = abs(finite_float(row.get("validation_target_abs_mean_error_m"), float("inf")) - target_abs) / target_scale
        seed_final_bonus = -0.05 if bool(row.get("is_seed_final_recommendation")) else 0.0
        return validation_delta + target_delta + seed_final_bonus

    selected = min(
        candidates,
        key=lambda row: (
            score(row),
            finite_float(row.get("validation_range_rmse_m"), float("inf")),
            finite_float(row.get("validation_target_abs_mean_error_m"), float("inf")),
            int(row.get("seed", 0)),
        ),
    )
    return {
        "selected": True,
        "reason": "closest_existing_seed_profile_to_multi_seed_mean_metrics",
        "selection_score": score(selected),
        "seed": selected["seed"],
        "method": selected["method"],
        "profile_path": selected["profile_path"],
        "validation_range_rmse_m": selected["validation_range_rmse_m"],
        "validation_target_abs_mean_error_m": selected["validation_target_abs_mean_error_m"],
        "validation_position_rmse_m": selected["validation_position_rmse_m"],
        "validation_right_rmse_m": selected["validation_right_rmse_m"],
        "validation_down_rmse_m": selected["validation_down_rmse_m"],
        "validation_forward_rmse_m": selected["validation_forward_rmse_m"],
        "axis_contract": selected["axis_contract"],
        "runtime_input_contract": selected["runtime_input_contract"],
    }


def run_representative_profile_checks(
    *,
    output_dir: Path,
    representative_profile: Mapping[str, object],
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    if not bool(representative_profile.get("selected")):
        return {
            "pass_representative_profile_checks": False,
            "reason": "no_representative_profile_selected",
        }
    profile_path = output_dir / str(representative_profile.get("representative_profile_path", ""))
    seed = representative_profile.get("seed")
    sample_csv = output_dir / f"seed_{seed}" / "calibration_samples.csv"
    if not profile_path.exists() or not sample_csv.exists():
        return {
            "pass_representative_profile_checks": False,
            "reason": "missing_representative_profile_or_seed_samples",
            "profile_path": str(profile_path.relative_to(output_dir)) if profile_path.exists() else "",
            "sample_csv": str(sample_csv.relative_to(output_dir)) if sample_csv.exists() else "",
        }

    replay_dir = output_dir / "representative_profile_replay"
    promotion_dir = output_dir / "representative_promotion_check"
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
        str(target_range_m),
        "--target-window-m",
        str(target_window_m),
    ]
    promotion_command = [
        sys.executable,
        str(PROFILE_PROMOTION_CHECK_SCRIPT),
        "--profile-json",
        str(profile_path),
        "--sample-csv",
        str(sample_csv),
        "--output-dir",
        str(promotion_dir),
        "--split",
        "validation",
    ]
    replay_result = subprocess.run(replay_command, cwd=ROOT_DIR)
    promotion_result = subprocess.run(promotion_command, cwd=ROOT_DIR)

    replay_summary: dict[str, object] = {}
    replay_summary_csv = replay_dir / "profile_replay_summary.csv"
    if replay_summary_csv.exists():
        rows = read_csv(replay_summary_csv)
        replay_summary = dict(rows[0]) if rows else {}
    promotion_summary: dict[str, object] = {}
    promotion_json = promotion_dir / "promotion_check.json"
    if promotion_json.exists():
        promotion_summary = json.loads(promotion_json.read_text(encoding="utf-8"))

    raw_rmse = finite_float(replay_summary.get("validation_range_rmse_raw_m"), float("inf"))
    corrected_rmse = finite_float(replay_summary.get("validation_range_rmse_corrected_m"), float("inf"))
    target_abs = finite_float(replay_summary.get("target_abs_mean_error_corrected_m"), float("inf"))
    replay_pass = (
        replay_result.returncode == 0
        and math.isfinite(raw_rmse)
        and math.isfinite(corrected_rmse)
        and corrected_rmse < raw_rmse
        and math.isfinite(target_abs)
        and target_abs <= 10.0
    )
    promotion_pass = promotion_result.returncode == 0 and bool(promotion_summary.get("pass_promotion_check"))
    summary = {
        "pass_representative_profile_checks": bool(replay_pass and promotion_pass),
        "profile_path": str(profile_path.relative_to(output_dir)),
        "sample_csv": str(sample_csv.relative_to(output_dir)),
        "replay_output_dir": str(replay_dir.relative_to(output_dir)),
        "promotion_output_dir": str(promotion_dir.relative_to(output_dir)),
        "replay_returncode": int(replay_result.returncode),
        "promotion_returncode": int(promotion_result.returncode),
        "replay_pass": bool(replay_pass),
        "promotion_pass": bool(promotion_pass),
        "validation_range_rmse_raw_m": raw_rmse,
        "validation_range_rmse_corrected_m": corrected_rmse,
        "target_abs_mean_error_corrected_m": target_abs,
        "promotion_check": promotion_summary,
    }
    write_json(output_dir / "representative_profile_checks.json", summary)
    return summary


def run_representative_cross_seed_replay(
    *,
    output_dir: Path,
    run_rows: Sequence[Mapping[str, object]],
    representative_profile: Mapping[str, object],
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    if not bool(representative_profile.get("selected")):
        return {
            "pass_cross_seed_replay": False,
            "reason": "no_representative_profile_selected",
            "rows": [],
        }
    profile_path = output_dir / str(representative_profile.get("representative_profile_path", ""))
    if not profile_path.exists():
        return {
            "pass_cross_seed_replay": False,
            "reason": "representative_profile_missing",
            "rows": [],
        }
    replay_root = output_dir / "representative_cross_seed_replay"
    rows_out: list[dict[str, object]] = []
    for run_row in run_rows:
        seed = int(run_row["seed"])
        if not bool(run_row.get("pass_run")):
            continue
        sample_csv = Path(str(run_row["run_dir"])) / "calibration_samples.csv"
        replay_dir = replay_root / f"seed_{seed}"
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
        replay_summary: dict[str, object] = {}
        replay_summary_csv = replay_dir / "profile_replay_summary.csv"
        if replay_summary_csv.exists():
            summaries = read_csv(replay_summary_csv)
            replay_summary = dict(summaries[0]) if summaries else {}
        raw_rmse = finite_float(replay_summary.get("validation_range_rmse_raw_m"), float("inf"))
        corrected_rmse = finite_float(replay_summary.get("validation_range_rmse_corrected_m"), float("inf"))
        target_abs = finite_float(replay_summary.get("target_abs_mean_error_corrected_m"), float("inf"))
        target_count = int(float(replay_summary.get("target_window_count", 0) or 0))
        pass_seed = (
            result.returncode == 0
            and math.isfinite(raw_rmse)
            and math.isfinite(corrected_rmse)
            and corrected_rmse < raw_rmse
            and target_count > 0
            and math.isfinite(target_abs)
            and target_abs <= 10.0
        )
        rows_out.append(
            {
                "seed": seed,
                "pass_seed_replay": pass_seed,
                "replay_returncode": int(result.returncode),
                "replay_output_dir": str(replay_dir.relative_to(output_dir)),
                "sample_csv": str(sample_csv.relative_to(output_dir)),
                "validation_range_rmse_raw_m": raw_rmse,
                "validation_range_rmse_corrected_m": corrected_rmse,
                "validation_range_rmse_improvement_m": raw_rmse - corrected_rmse,
                "target_window_count": target_count,
                "target_abs_mean_error_corrected_m": target_abs,
                "axis_contract": RDF_AXIS_CONTRACT,
                "runtime_input_contract": "raw stereo-center RDF(right,down,forward) plus measured disparity; no runtime truth",
            }
        )
    pass_cross_seed = bool(rows_out) and all(bool(row["pass_seed_replay"]) for row in rows_out)
    summary = {
        "pass_cross_seed_replay": pass_cross_seed,
        "profile_path": str(profile_path.relative_to(output_dir)),
        "seed_count": len(rows_out),
        "passed_seed_count": sum(1 for row in rows_out if bool(row["pass_seed_replay"])),
        "rows": rows_out,
    }
    write_csv(output_dir / "representative_cross_seed_replay_summary.csv", rows_out)
    write_json(output_dir / "representative_cross_seed_replay_summary.json", summary)
    return summary


def write_summary(
    path: Path,
    *,
    seed_count: int,
    contract_summary: Mapping[str, object],
    policy_rows: Sequence[Mapping[str, object]],
    method_rows: Sequence[Mapping[str, object]],
    gate_rows: Sequence[Mapping[str, object]],
    method_gate_rows: Sequence[Mapping[str, object]],
    near_tie_rows: Sequence[Mapping[str, object]],
    family_diagnostics: Sequence[Mapping[str, object]],
    recommendation: Mapping[str, object],
    recommended_profiles: Sequence[Mapping[str, object]],
    near_tie_profiles: Sequence[Mapping[str, object]],
    representative_profile: Mapping[str, object],
    representative_checks: Mapping[str, object],
    cross_seed_replay: Mapping[str, object],
    acceptance_matrix: Mapping[str, Mapping[str, object]],
) -> None:
    lines = [
        "RTK stereo policy multi-seed validation",
        "",
        f"Seed count: {seed_count}",
        f"NED contract: {NED_AXIS_CONTRACT}",
        f"RDF contract: {RDF_AXIS_CONTRACT}",
        f"StereoDistance public topic contract: {PUBLIC_TOPIC_CONTRACT}",
        f"Truth RDF transform contract: {contract_summary.get('truth_rdf_transform_contract', TRUTH_RDF_TRANSFORM_CONTRACT)}",
        "Direct NED-to-RDF axis alias forbidden: {passed} ({count}/{total} loaded seed configs)".format(
            passed=contract_summary.get("all_loaded_contracts_forbid_direct_axis_alias", False),
            count=contract_summary.get("direct_axis_alias_forbidden_count", 0),
            total=contract_summary.get("loaded_contract_count", 0),
        ),
        "",
        "Policy stability:",
    ]
    for row in policy_rows:
        lines.append(
            "- {policy}: most_common={method} ({count}/{total}), "
            "validation_rmse_mean={rmse:.6f}m, axis_mean=({right:.6f},{down:.6f},{forward:.6f})m, "
            "target_abs_mean={target:.6f}m".format(
                policy=row["policy"],
                method=row["most_common_method"],
                count=row["most_common_method_count"],
                total=row["seed_count"],
                rmse=float(row["validation_range_rmse_mean_m"]),
                right=float(row.get("validation_right_rmse_mean_m", float("nan"))),
                down=float(row.get("validation_down_rmse_mean_m", float("nan"))),
                forward=float(row.get("validation_forward_rmse_mean_m", float("nan"))),
                target=float(row["target_abs_mean_error_mean_m"]),
            )
        )
    lines.extend(["", "Policy stability gate:"])
    for row in gate_rows:
        lines.append(
            "- {policy}: strict_pass={strict_pass}, performance_pass={performance_pass}, "
            "stable={stable}, axis_regression_max={axis:.6f}m, reasons={reasons}".format(
                policy=row["policy"],
                strict_pass=row["strict_pass"],
                performance_pass=row["performance_pass"],
                stable=row["method_stable"],
                axis=float(row.get("axis_rmse_regression_max_m", float("nan"))),
                reasons=row["reasons"] or "none",
            )
        )
    lines.extend(["", "Method stability gate:"])
    for row in method_gate_rows:
        if str(row["method"]) == "raw":
            continue
        lines.append(
            "- {method}: strict_pass={strict_pass}, performance_pass={performance_pass}, "
            "validation_rmse_mean={rmse:.6f}m, target_abs_max={target:.6f}m, reasons={reasons}".format(
                method=row["method"],
                strict_pass=row["strict_pass"],
                performance_pass=row["performance_pass"],
                rmse=float(row["method_validation_range_rmse_mean_m"]),
                target=float(row["method_target_abs_mean_error_max_m"]),
                reasons=row["reasons"] or "none",
            )
        )
    lines.extend(["", "Method family diagnostics:"])
    for row in list(family_diagnostics)[:8]:
        lines.append(
            "- rank={rank}, best={method}, family=({registration}, {residual}, {regularization}, {runtime}), "
            "strict={strict_count}/{method_count}, validation_rmse_mean={rmse:.6f}m, target_abs_max={target:.6f}m".format(
                rank=row["family_rank"],
                method=row["best_method"],
                registration=row["registration_model"],
                residual=row["residual_model"],
                regularization=row["regularization_model"],
                runtime=row["runtime_profile_family"],
                strict_count=row["strict_pass_count"],
                method_count=row["method_count"],
                rmse=float(row["best_validation_range_rmse_mean_m"]),
                target=float(row["best_target_abs_mean_error_max_m"]),
            )
        )
    lines.extend(
        [
            "",
            "Multi-seed recommendation:",
            "- mode={mode}, method={method}, validation_rmse_mean={rmse}, "
            "target_abs_max={target}, reason={reason}".format(
                mode=recommendation.get("recommendation_mode", "none"),
                method=recommendation.get("method", ""),
                rmse=recommendation.get("validation_range_rmse_mean_m", "nan"),
                target=recommendation.get("target_abs_mean_error_max_m", "nan"),
                reason=recommendation.get("reason", ""),
            ),
        ]
    )
    lines.extend(["", "Selection near ties:"])
    if near_tie_rows:
        for row in list(near_tie_rows)[:8]:
            lines.append(
                "- rank={rank}, method={method}, score={score:.6f}, delta={delta:.6f}, "
                "validation_rmse_mean={rmse:.6f}m, target_abs_max={target:.6f}m, near_tie={near}".format(
                    rank=row["selection_rank"],
                    method=row["method"],
                    score=float(row["selection_score"]),
                    delta=float(row["selection_score_delta_from_winner"]),
                    rmse=float(row["method_validation_range_rmse_mean_m"]),
                    target=float(row["method_target_abs_mean_error_max_m"]),
                    near=row["within_near_tie_threshold"],
                )
            )
    else:
        lines.append("- none")
    lines.extend(["", "Selection near-tie runtime profiles:"])
    if near_tie_profiles:
        for row in list(near_tie_profiles)[:12]:
            lines.append(
                "- rank={rank}, seed={seed}, method={method}, profile={profile}, "
                "validation_rmse={rmse:.6f}m, target_abs={target:.6f}m".format(
                    rank=row["selection_rank"],
                    seed=row["seed"],
                    method=row["method"],
                    profile=row["profile_path"] or "missing",
                    rmse=float(row["validation_range_rmse_m"]),
                    target=float(row["validation_target_abs_mean_error_m"]),
                )
            )
    else:
        lines.append("- none")
    lines.extend(["", "Recommended runtime profiles:"])
    if recommended_profiles:
        for row in recommended_profiles:
            lines.append(
                "- seed={seed}, profile={profile}, validation_rmse={rmse:.6f}m, "
                "target_abs={target:.6f}m, seed_final={seed_final}".format(
                    seed=row["seed"],
                    profile=row["profile_path"] or "missing",
                    rmse=float(row["validation_range_rmse_m"]),
                    target=float(row["validation_target_abs_mean_error_m"]),
                    seed_final=row["is_seed_final_recommendation"],
                )
            )
    else:
        lines.append("- none")
    lines.extend(
        [
            "",
            "Representative runtime profile:",
            "- selected={selected}, seed={seed}, profile={profile}, reason={reason}".format(
                selected=representative_profile.get("selected", False),
                seed=representative_profile.get("seed", ""),
                profile=representative_profile.get("profile_path", ""),
                reason=representative_profile.get("reason", ""),
            ),
        ]
    )
    lines.extend(
        [
            "",
            "Representative profile checks:",
            "- pass={passed}, replay_pass={replay}, promotion_pass={promotion}, replay_dir={replay_dir}, promotion_dir={promotion_dir}".format(
                passed=representative_checks.get("pass_representative_profile_checks", False),
                replay=representative_checks.get("replay_pass", False),
                promotion=representative_checks.get("promotion_pass", False),
                replay_dir=representative_checks.get("replay_output_dir", ""),
                promotion_dir=representative_checks.get("promotion_output_dir", ""),
            ),
        ]
    )
    lines.extend(
        [
            "",
            "Representative cross-seed replay:",
            "- pass={passed}, passed_seed_count={passed_count}/{seed_count}, summary=representative_cross_seed_replay_summary.csv".format(
                passed=cross_seed_replay.get("pass_cross_seed_replay", False),
                passed_count=cross_seed_replay.get("passed_seed_count", 0),
                seed_count=cross_seed_replay.get("seed_count", 0),
            ),
        ]
    )
    lines.extend(["", "Multi-seed acceptance matrix:"])
    for name, item in acceptance_matrix.items():
        evidence = item.get("evidence", {}) if isinstance(item, Mapping) else {}
        lines.append(
            "- {name}: status={status}, path={path}, evidence={evidence}".format(
                name=name,
                status=item.get("status", "") if isinstance(item, Mapping) else "",
                path=item.get("path", "") if isinstance(item, Mapping) else "",
                evidence=json.dumps(json_safe(evidence), sort_keys=True),
            )
        )
    lines.extend(["", "Top methods by mean validation RMSE:"])
    for row in list(method_rows)[:8]:
        lines.append(
            "- {method}: validation_rmse_mean={rmse:.6f}m, "
            "target_abs_mean={target:.6f}m, selected_count={selected}".format(
                method=row["method"],
                rmse=float(row["validation_range_rmse_mean_m"]),
                target=float(row["target_abs_mean_error_mean_m"]),
                selected=row["selected_count"],
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_policy_stability(path: Path, policy_rows: Sequence[Mapping[str, object]], gate_rows: Sequence[Mapping[str, object]]) -> None:
    if not policy_rows:
        return
    gate_by_policy = {str(row["policy"]): row for row in gate_rows}
    labels = [str(row["policy"]) for row in policy_rows]
    x_values = np.arange(len(labels))
    validation_mean = np.array([finite_float(row["validation_range_rmse_mean_m"]) for row in policy_rows], dtype=float)
    validation_low = np.array([finite_float(row["validation_range_rmse_mean_ci90_low_m"]) for row in policy_rows], dtype=float)
    validation_high = np.array([finite_float(row["validation_range_rmse_mean_ci90_high_m"]) for row in policy_rows], dtype=float)
    target_mean = np.array([finite_float(row["target_abs_mean_error_mean_m"]) for row in policy_rows], dtype=float)
    target_low = np.array([finite_float(row["target_abs_mean_error_mean_ci90_low_m"]) for row in policy_rows], dtype=float)
    target_high = np.array([finite_float(row["target_abs_mean_error_mean_ci90_high_m"]) for row in policy_rows], dtype=float)
    validation_error = np.vstack([validation_mean - validation_low, validation_high - validation_mean]) if len(labels) > 1 else None
    target_error = np.vstack([target_mean - target_low, target_high - target_mean]) if len(labels) > 1 else None
    plt.figure(figsize=(12, 6))
    width = 0.35
    colors = [
        "tab:green" if bool(gate_by_policy.get(label, {}).get("strict_pass")) else "tab:orange"
        for label in labels
    ]
    plt.bar(x_values - width / 2, validation_mean, width=width, yerr=validation_error, capsize=4, label="validation RMSE", color=colors, alpha=0.78)
    plt.bar(x_values + width / 2, target_mean, width=width, yerr=target_error, capsize=4, label="1852m abs mean error", color="tab:blue", alpha=0.55)
    plt.xticks(x_values, labels, rotation=25, ha="right")
    plt.ylabel("meters")
    plt.title("Policy stability across seeds (mean with bootstrap 90% CI)")
    plt.grid(True, axis="y", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(path, dpi=150)
    plt.close()


def plot_method_aggregate(path: Path, method_rows: Sequence[Mapping[str, object]], *, top_n: int = 12) -> None:
    rows = list(method_rows)[:top_n]
    if not rows:
        return
    labels = [str(row["method"]) for row in rows]
    y_values = np.arange(len(labels))
    validation_mean = np.array([finite_float(row["validation_range_rmse_mean_m"]) for row in rows], dtype=float)
    validation_low = np.array([finite_float(row["validation_range_rmse_mean_ci90_low_m"]) for row in rows], dtype=float)
    validation_high = np.array([finite_float(row["validation_range_rmse_mean_ci90_high_m"]) for row in rows], dtype=float)
    xerr = np.vstack([validation_mean - validation_low, validation_high - validation_mean]) if len(rows) > 1 else None
    target_mean = np.array([finite_float(row["target_abs_mean_error_mean_m"]) for row in rows], dtype=float)
    plt.figure(figsize=(12, 7))
    scatter = plt.scatter(validation_mean, y_values, s=70, c=target_mean, cmap="viridis_r", zorder=3)
    plt.errorbar(validation_mean, y_values, xerr=xerr, fmt="none", ecolor="tab:gray", capsize=4, zorder=2)
    plt.yticks(y_values, labels)
    plt.gca().invert_yaxis()
    plt.xlabel("validation range RMSE mean (m), bootstrap 90% CI")
    plt.title("Top calibration methods across seeds")
    plt.grid(True, axis="x", alpha=0.3)
    colorbar = plt.colorbar(scatter)
    colorbar.set_label("1852m abs mean error mean (m)")
    plt.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(path, dpi=150)
    plt.close()


def main() -> int:
    args = resolve_runtime_paths(parse_args())
    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    seeds = parse_seed_list(args.seeds)

    run_rows: list[dict[str, object]] = []
    seed_contract_rows: list[dict[str, object]] = []
    per_seed_method_rows: list[dict[str, object]] = []
    per_seed_policy_rows: list[dict[str, object]] = []
    for seed in seeds:
        run_dir = output_dir / f"seed_{seed}"
        exit_code = run_pipeline(args, seed, run_dir)
        run_rows.append(
            {
                "seed": seed,
                "run_dir": str(run_dir),
                "exit_code": exit_code,
                "pass_run": exit_code == 0,
            }
        )
        if exit_code != 0:
            seed_contract_rows.append(
                {
                    "seed": seed,
                    "run_config_loaded": False,
                    "direct_axis_alias_forbidden": False,
                    "truth_rdf_transform_contract": "",
                    "ned_axis_contract": NED_AXIS_CONTRACT,
                    "rdf_axis_contract": RDF_AXIS_CONTRACT,
                    "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                }
            )
            continue
        run_config = json.loads((run_dir / "run_config.json").read_text(encoding="utf-8"))
        run_contracts = run_config.get("contracts") if isinstance(run_config.get("contracts"), dict) else {}
        seed_contract_rows.append(
            {
                "seed": seed,
                "run_config_loaded": True,
                "direct_axis_alias_forbidden": direct_axis_alias_forbidden(run_config),
                "truth_rdf_transform_contract": run_contracts.get(
                    "truth_rdf_transform_contract",
                    run_config.get("truth_rdf_transform_contract", TRUTH_RDF_TRANSFORM_CONTRACT),
                ),
                "ned_to_stereo_rdf_contract": run_contracts.get("ned_to_stereo_rdf", ""),
                "direct_ned_to_rdf_axis_alias_allowed": bool(
                    run_contracts.get(
                        "direct_ned_to_rdf_axis_alias_allowed",
                        run_config.get("direct_ned_to_rdf_axis_alias_allowed", False),
                    )
                ),
                "synthetic_stereo_origin_ned_m": json.dumps(
                    run_contracts.get("synthetic_stereo_origin_ned_m", []),
                    sort_keys=True,
                ),
                "synthetic_ned_to_stereo_rdf_euler_zyx_deg": json.dumps(
                    run_contracts.get("synthetic_ned_to_stereo_rdf_euler_zyx_deg", []),
                    sort_keys=True,
                ),
                "ned_axis_contract": NED_AXIS_CONTRACT,
                "rdf_axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
        recommendation = str(run_config.get("recommendation", ""))
        for row in read_csv(run_dir / "method_comparison.csv"):
            row_out: dict[str, object] = {"seed": seed, **row}
            row_out["is_final_recommendation"] = row.get("method") == recommendation
            per_seed_method_rows.append(row_out)
        for row in read_csv(run_dir / "policy_recommendations.csv"):
            per_seed_policy_rows.append({"seed": seed, **row})

    write_csv(output_dir / "seed_runs.csv", run_rows)
    write_csv(output_dir / "seed_coordinate_contracts.csv", seed_contract_rows)
    write_csv(output_dir / "per_seed_method_comparison.csv", per_seed_method_rows)
    write_csv(output_dir / "per_seed_policy_recommendations.csv", per_seed_policy_rows)
    method_aggregate = aggregate_method_rows(per_seed_method_rows) if per_seed_method_rows else []
    policy_aggregate = aggregate_policy_rows(per_seed_policy_rows) if per_seed_policy_rows else []
    write_csv(output_dir / "method_aggregate_summary.csv", method_aggregate)
    write_csv(output_dir / "policy_stability_summary.csv", policy_aggregate)
    completed_seed_count = sum(1 for row in run_rows if row["pass_run"])
    contract_summary = seed_contract_summary(seed_contract_rows)
    gate_rows = policy_stability_gate_rows(
        policy_aggregate,
        method_aggregate,
        completed_seed_count=completed_seed_count,
        min_validation_improvement_m=args.min_validation_improvement_m,
    )
    method_gate_rows = method_stability_gate_rows(
        method_aggregate,
        completed_seed_count=completed_seed_count,
        min_validation_improvement_m=args.min_validation_improvement_m,
    )
    method_gate_rows = annotate_method_selection_scores(method_gate_rows)
    near_tie_rows = selection_near_tie_rows(method_gate_rows)
    family_diagnostics = method_family_diagnostic_rows(method_gate_rows)
    multi_seed_recommendation = choose_multi_seed_recommendation(method_gate_rows)
    recommended_profiles = recommended_profile_rows(
        output_dir=output_dir,
        run_rows=run_rows,
        per_seed_method_rows=per_seed_method_rows,
        recommendation=multi_seed_recommendation,
    )
    near_tie_profiles = near_tie_profile_candidate_rows(
        output_dir=output_dir,
        run_rows=run_rows,
        per_seed_method_rows=per_seed_method_rows,
        near_tie_rows=near_tie_rows,
    )
    representative_profile = choose_representative_profile(recommended_profiles, multi_seed_recommendation)
    if bool(representative_profile.get("selected")) and representative_profile.get("profile_path"):
        source_profile = output_dir / str(representative_profile["profile_path"])
        if source_profile.exists():
            representative_output = output_dir / "representative_runtime_profile.json"
            shutil.copyfile(source_profile, representative_output)
            representative_profile["representative_profile_path"] = str(representative_output.relative_to(output_dir))
    representative_checks = run_representative_profile_checks(
        output_dir=output_dir,
        representative_profile=representative_profile,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    cross_seed_replay = run_representative_cross_seed_replay(
        output_dir=output_dir,
        run_rows=run_rows,
        representative_profile=representative_profile,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
    )
    acceptance_matrix = build_multi_seed_acceptance_matrix(
        seed_count=len(seeds),
        completed_seed_count=completed_seed_count,
        contract_summary=contract_summary,
        method_rows=method_aggregate,
        policy_gate_rows=gate_rows,
        method_gate_rows=method_gate_rows,
        recommendation=multi_seed_recommendation,
        recommended_profiles=recommended_profiles,
        representative_checks=representative_checks,
        cross_seed_replay=cross_seed_replay,
    )
    acceptance_pass = all(item.get("status") == "PASS" for item in acceptance_matrix.values())
    write_csv(output_dir / "policy_stability_gate.csv", gate_rows)
    write_csv(output_dir / "method_stability_gate.csv", method_gate_rows)
    write_csv(output_dir / "selection_near_ties.csv", near_tie_rows)
    write_csv(output_dir / "method_family_diagnostics.csv", family_diagnostics)
    write_csv(output_dir / "multi_seed_recommended_profiles.csv", recommended_profiles)
    write_csv(output_dir / "selection_near_tie_profiles.csv", near_tie_profiles)
    write_json(output_dir / "multi_seed_recommendation.json", multi_seed_recommendation)
    write_json(output_dir / "selection_near_ties.json", {"near_ties": near_tie_rows})
    write_json(output_dir / "method_family_diagnostics.json", {"families": family_diagnostics})
    write_json(
        output_dir / "multi_seed_recommended_profiles.json",
        {
            "recommendation": multi_seed_recommendation,
            "profiles": recommended_profiles,
        },
    )
    write_json(
        output_dir / "selection_near_tie_profiles.json",
        {
            "near_ties": near_tie_rows,
            "profiles": near_tie_profiles,
        },
    )
    write_json(output_dir / "multi_seed_representative_profile.json", representative_profile)
    summary = {
        "seed_count": len(seeds),
        "completed_seed_count": completed_seed_count,
        "coordinate_contract_summary": contract_summary,
        "seed_coordinate_contracts": seed_contract_rows,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "ned_axis_contract": NED_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "truth_rdf_transform_contract": TRUTH_RDF_TRANSFORM_CONTRACT,
        "policy_stability_summary": policy_aggregate,
        "policy_stability_gate": gate_rows,
        "method_stability_gate": method_gate_rows,
        "selection_near_ties": near_tie_rows,
        "method_family_diagnostics": family_diagnostics,
        "method_aggregate_summary": method_aggregate,
        "multi_seed_recommendation": multi_seed_recommendation,
        "multi_seed_recommended_profiles": recommended_profiles,
        "selection_near_tie_profiles": near_tie_profiles,
        "multi_seed_representative_profile": representative_profile,
        "representative_profile_checks": representative_checks,
        "representative_cross_seed_replay": cross_seed_replay,
        "multi_seed_acceptance_matrix": acceptance_matrix,
        "multi_seed_acceptance_pass": acceptance_pass,
    }
    write_json(output_dir / "multi_seed_summary.json", summary)
    plot_policy_stability(output_dir / "policy_stability_ci.png", policy_aggregate, gate_rows)
    plot_method_aggregate(output_dir / "method_aggregate_ci.png", method_aggregate)
    write_summary(
        output_dir / "multi_seed_summary.txt",
        seed_count=len(seeds),
        contract_summary=contract_summary,
        policy_rows=policy_aggregate,
        method_rows=method_aggregate,
        gate_rows=gate_rows,
        method_gate_rows=method_gate_rows,
        near_tie_rows=near_tie_rows,
        family_diagnostics=family_diagnostics,
        recommendation=multi_seed_recommendation,
        recommended_profiles=recommended_profiles,
        near_tie_profiles=near_tie_profiles,
        representative_profile=representative_profile,
        representative_checks=representative_checks,
        cross_seed_replay=cross_seed_replay,
        acceptance_matrix=acceptance_matrix,
    )
    print(f"wrote multi-seed summary to {output_dir}")
    return 0 if all(bool(row["pass_run"]) for row in run_rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
