#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import sys
from pathlib import Path
from typing import Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.runtime_guard_policy import (  # noqa: E402
    EstimatedToTruthAffineProfile,
    EstimatedToTruthDisparityProfile,
    EstimatedToTruthSim3Profile,
)


REQUIRED_FILES = {
    "method_comparison": "method_comparison.csv",
    "validation_predictions": "validation_predictions.csv",
    "target_1852m_stats": "target_1852m_stats.csv",
    "summary": "summary.txt",
    "run_config": "run_config.json",
    "method_parameters": "method_parameters.json",
    "top_level_runtime_profile": "sim3_runtime_profile.json",
    "policy_recommendations": "policy_recommendations.csv",
    "policy_recommendations_json": "policy_recommendations.json",
    "range_error_plot": "range_error_before_after.png",
    "residual_disparity_plot": "residual_disparity_by_distance.png",
    "rdf_axis_plot": "rdf_axis_rmse.png",
    "pareto_plot": "pareto_frontier.png",
}

REQUIRED_METHOD_COLUMNS = {
    "method",
    "rdf_axis_contract",
    "registration_model",
    "residual_model",
    "regularization_model",
    "runtime_profile_family",
    "selection_note",
    "model_complexity",
    "train_range_rmse_m",
    "validation_range_rmse_m",
    "validation_right_rmse_m",
    "validation_down_rmse_m",
    "validation_forward_rmse_m",
    "validation_target_window_count",
    "validation_target_abs_mean_error_m",
    "validation_target_max_abs_error_m",
    "parameters_json",
}

REQUIRED_TARGET_COLUMNS = {
    "method",
    "target_range_m",
    "window_half_width_m",
    "sample_count",
    "mean_error_m",
    "abs_mean_error_m",
    "rmse_m",
    "max_abs_error_m",
    "pass_abs_mean_within_10m",
}

REQUIRED_VALIDATION_PREDICTION_COLUMNS = {
    "method",
    "sample_id",
    "split",
    "rdf_axis_contract",
    "pred_right_m",
    "pred_down_m",
    "pred_forward_m",
    "pred_range_m",
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "error_right_m",
    "error_down_m",
    "error_forward_m",
    "range_error_m",
    "measured_disparity_px",
    "true_disparity_px",
}

REQUIRED_POLICY_COLUMNS = {
    "policy",
    "purpose",
    "method",
    "is_final_recommendation",
    "validation_range_rmse_m",
    "validation_position_rmse_m",
    "validation_right_rmse_m",
    "validation_down_rmse_m",
    "validation_forward_rmse_m",
    "validation_target_abs_mean_error_m",
    "validation_target_max_abs_error_m",
    "raw_validation_range_rmse_m",
    "target_tolerance_m",
}

RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
EXPECTED_TRAIN_RATIO = 0.8
MIN_PLOT_WIDTH_PX = 640
MIN_PLOT_HEIGHT_PX = 360
PROFILE_OPTIONAL_FAMILIES = {"none_raw", "offline_or_nonpreferred_runtime_candidate"}
DELIVERABLE_EVIDENCE_ITEMS = (
    "method_comparison_csv",
    "range_error_plot_png",
    "residual_disparity_plot_png",
    "rdf_axis_rmse_plot_png",
    "rdf_axis_rmse_nonregression",
    "target_1852m_stats_csv",
    "summary_txt_final_recommendation",
    "validation_predictions_csv",
    "runtime_profiles_manifest",
    "policy_profiles_manifest",
)
RDF_AXIS_RMSE_COLUMNS = {
    "right": "validation_right_rmse_m",
    "down": "validation_down_rmse_m",
    "forward": "validation_forward_rmse_m",
}


def read_csv_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def load_json(path: Path) -> object:
    return json.loads(path.read_text(encoding="utf-8"))


def finite_float(value: object, default: float = math.inf) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def png_dimensions(path: Path) -> tuple[int, int] | None:
    if not path.is_file() or path.stat().st_size < 24:
        return None
    header = path.read_bytes()[:24]
    if header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
        return None
    width, height = struct.unpack(">II", header[16:24])
    return int(width), int(height)


def png_is_valid_plot(path: Path) -> bool:
    dimensions = png_dimensions(path)
    if dimensions is None:
        return False
    width, height = dimensions
    return width >= MIN_PLOT_WIDTH_PX and height >= MIN_PLOT_HEIGHT_PX


def plot_dimension_string_is_valid(value: object) -> bool:
    if not isinstance(value, str) or "x" not in value:
        return False
    left, right = value.split("x", 1)
    try:
        width = int(left)
        height = int(right)
    except ValueError:
        return False
    return width >= MIN_PLOT_WIDTH_PX and height >= MIN_PLOT_HEIGHT_PX


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def row_by_method(rows: list[Mapping[str, str]], method: str) -> Mapping[str, str] | None:
    return next((row for row in rows if row.get("method") == method), None)


def has_forbidden_truth_name(values: object) -> bool:
    if not isinstance(values, list):
        return True
    joined = " ".join(str(value).lower() for value in values)
    return any(token in joined for token in ("truth", "gt_", "px4", "rtk", "ned"))


def truth_forbidden_declared(values: object) -> bool:
    if not isinstance(values, list):
        return False
    joined = " ".join(str(value).lower() for value in values)
    return all(token in joined for token in ("truth", "px4", "rtk"))


def values_close(left: object, right: object, *, tolerance: float = 1.0e-9) -> bool:
    return abs(finite_float(left) - finite_float(right)) <= tolerance


def load_runtime_profile(payload: Mapping[str, object]) -> object:
    mode = payload.get("mode")
    if mode == EstimatedToTruthSim3Profile.mode:
        return EstimatedToTruthSim3Profile.from_json_dict(payload)
    if mode == EstimatedToTruthDisparityProfile.mode:
        return EstimatedToTruthDisparityProfile.from_json_dict(payload)
    if mode == EstimatedToTruthAffineProfile.mode:
        return EstimatedToTruthAffineProfile.from_json_dict(payload)
    raise ValueError(f"unsupported selected runtime profile mode: {mode!r}")


def runtime_profile_apply_smoke(profile: object) -> tuple[bool, str]:
    raw = {
        "right_m": 4.0,
        "down_m": 1.5,
        "forward_m": 800.0,
        "range_m": math.sqrt(4.0 * 4.0 + 1.5 * 1.5 + 800.0 * 800.0),
        "left_u_px": 640.0,
        "left_v_px": 360.0,
        "disparity_px": 25.0,
        "raw_disparity_px": 25.0,
    }
    try:
        result = profile.apply(raw)  # type: ignore[attr-defined]
    except Exception as exc:  # pragma: no cover - defensive audit path
        return False, f"exception:{exc}"
    corrected = result.corrected
    if not isinstance(corrected, Mapping):
        return False, "no_corrected_mapping"
    for field in ("right_m", "down_m", "forward_m", "range_m", "disparity_px"):
        if finite_float(corrected.get(field)) == math.inf:
            return False, f"nonfinite_{field}"
    status = result.status if isinstance(result.status, Mapping) else {}
    if int(status.get("fallback_level", 0) or 0) >= 3:
        return False, f"fallback_level_{status.get('fallback_level')}"
    return True, str(status.get("profile_mode", ""))


def build_deliverable_evidence_matrix(
    *,
    result_dir: Path,
    recommendation: str,
    method_rows: list[Mapping[str, str]],
    prediction_rows: list[Mapping[str, str]],
    target_rows: list[Mapping[str, str]],
    selected_profile_path: str,
    runtime_profile_count: int,
    runtime_profile_load_pass_count: int,
    policy_profile_count: int,
    policy_profile_load_pass_count: int,
    plot_dimensions: Mapping[str, str],
    runtime_profile_missing_methods: list[str],
    target_tolerance_m: float,
) -> dict[str, dict[str, object]]:
    recommended_row = row_by_method(method_rows, recommendation)
    target_row = row_by_method(target_rows, recommendation)
    raw_row = row_by_method(method_rows, "raw")
    raw_rmse = finite_float(raw_row.get("validation_range_rmse_m")) if raw_row else math.inf
    recommendation_rmse = (
        finite_float(recommended_row.get("validation_range_rmse_m")) if recommended_row else math.inf
    )
    target_abs_mean = finite_float(target_row.get("abs_mean_error_m")) if target_row else math.inf
    runtime_profiles_ok = runtime_profile_count > 0 and runtime_profile_load_pass_count == runtime_profile_count
    policy_profiles_ok = policy_profile_count > 0 and policy_profile_load_pass_count == policy_profile_count
    rdf_axis_evidence: dict[str, dict[str, float | bool]] = {}
    rdf_axes_nonregressing = raw_row is not None and recommended_row is not None
    if raw_row is not None and recommended_row is not None:
        for axis, column in RDF_AXIS_RMSE_COLUMNS.items():
            raw_axis_rmse = finite_float(raw_row.get(column))
            recommended_axis_rmse = finite_float(recommended_row.get(column))
            axis_nonregressing = recommended_axis_rmse <= raw_axis_rmse + 1.0e-9
            rdf_axis_evidence[axis] = {
                "raw_rmse_m": raw_axis_rmse,
                "recommended_rmse_m": recommended_axis_rmse,
                "nonregressing": axis_nonregressing,
            }
            rdf_axes_nonregressing = rdf_axes_nonregressing and axis_nonregressing
    return {
        "method_comparison_csv": {
            "status": "PASS" if method_rows and recommended_row is not None else "FAIL",
            "path": "method_comparison.csv",
            "evidence": {
                "method_count": len(method_rows),
                "recommendation": recommendation,
                "raw_validation_range_rmse_m": raw_rmse,
                "recommended_validation_range_rmse_m": recommendation_rmse,
            },
        },
        "range_error_plot_png": {
            "status": "PASS" if plot_dimension_string_is_valid(plot_dimensions.get("range_error_plot")) else "FAIL",
            "path": "range_error_before_after.png",
            "evidence": {"dimensions": plot_dimensions.get("range_error_plot", "")},
        },
        "residual_disparity_plot_png": {
            "status": "PASS" if plot_dimension_string_is_valid(plot_dimensions.get("residual_disparity_plot")) else "FAIL",
            "path": "residual_disparity_by_distance.png",
            "evidence": {"dimensions": plot_dimensions.get("residual_disparity_plot", "")},
        },
        "rdf_axis_rmse_plot_png": {
            "status": "PASS" if plot_dimension_string_is_valid(plot_dimensions.get("rdf_axis_plot")) else "FAIL",
            "path": "rdf_axis_rmse.png",
            "evidence": {"dimensions": plot_dimensions.get("rdf_axis_plot", "")},
        },
        "rdf_axis_rmse_nonregression": {
            "status": "PASS" if rdf_axes_nonregressing else "FAIL",
            "path": "method_comparison.csv",
            "evidence": {
                "recommendation": recommendation,
                "axis_contract": RDF_AXIS_CONTRACT,
                "axes": rdf_axis_evidence,
            },
        },
        "target_1852m_stats_csv": {
            "status": "PASS" if target_row is not None and target_abs_mean <= target_tolerance_m else "FAIL",
            "path": "target_1852m_stats.csv",
            "evidence": {
                "target_1852m_method_count": len(target_rows),
                "recommended_abs_mean_error_m": target_abs_mean,
                "target_tolerance_m": target_tolerance_m,
                "recommended_sample_count": int(float(target_row.get("sample_count", 0) or 0)) if target_row else 0,
            },
        },
        "summary_txt_final_recommendation": {
            "status": "PASS" if recommendation else "FAIL",
            "path": "summary.txt",
            "evidence": {"recommendation": recommendation},
        },
        "validation_predictions_csv": {
            "status": "PASS" if prediction_rows else "FAIL",
            "path": "validation_predictions.csv",
            "evidence": {"validation_prediction_row_count": len(prediction_rows)},
        },
        "runtime_profiles_manifest": {
            "status": "PASS" if runtime_profiles_ok and selected_profile_path else "FAIL",
            "path": "runtime_profiles/manifest.json",
            "evidence": {
                "runtime_profile_count": runtime_profile_count,
                "runtime_profile_load_pass_count": runtime_profile_load_pass_count,
                "selected_profile_path": selected_profile_path,
                "missing_methods_with_allowed_profile_family": runtime_profile_missing_methods,
            },
        },
        "policy_profiles_manifest": {
            "status": "PASS" if policy_profiles_ok else "FAIL",
            "path": "policy_profiles/manifest.json",
            "evidence": {
                "policy_profile_count": policy_profile_count,
                "policy_profile_load_pass_count": policy_profile_load_pass_count,
            },
        },
    }


def audit_outputs(result_dir: Path, *, target_tolerance_m: float = 10.0) -> dict[str, object]:
    failures: list[str] = []
    result_dir = result_dir.resolve()
    paths = {name: result_dir / relative for name, relative in REQUIRED_FILES.items()}

    for name, path in paths.items():
        require(path.is_file(), failures, f"missing_required_file:{name}:{path}")
    if failures:
        return {
            "status": "FAIL",
            "result_dir": str(result_dir),
            "failure_count": len(failures),
            "failures": failures,
            "runtime_truth_used": False,
            "coordinate_contracts_explicit": False,
        }

    method_columns, method_rows = read_csv_rows(paths["method_comparison"])
    prediction_columns, prediction_rows = read_csv_rows(paths["validation_predictions"])
    target_columns, target_rows = read_csv_rows(paths["target_1852m_stats"])
    policy_columns, policy_rows = read_csv_rows(paths["policy_recommendations"])
    run_config = load_json(paths["run_config"])
    method_parameters = load_json(paths["method_parameters"])
    top_level_runtime_profile = load_json(paths["top_level_runtime_profile"])
    policy_json = load_json(paths["policy_recommendations_json"])
    summary_text = paths["summary"].read_text(encoding="utf-8")

    require(set(method_columns) >= REQUIRED_METHOD_COLUMNS, failures, "method_comparison:missing_required_columns")
    require(
        set(prediction_columns) >= REQUIRED_VALIDATION_PREDICTION_COLUMNS,
        failures,
        "validation_predictions:missing_required_columns",
    )
    require(set(target_columns) >= REQUIRED_TARGET_COLUMNS, failures, "target_1852m_stats:missing_required_columns")
    require(set(policy_columns) >= REQUIRED_POLICY_COLUMNS, failures, "policy_recommendations:missing_required_columns")
    require(bool(method_rows), failures, "method_comparison:no_rows")
    require(bool(prediction_rows), failures, "validation_predictions:no_rows")
    require(bool(target_rows), failures, "target_1852m_stats:no_rows")
    require(bool(policy_rows), failures, "policy_recommendations:no_rows")

    raw_row = row_by_method(method_rows, "raw")
    require(raw_row is not None, failures, "method_comparison:missing_raw_method")

    assert isinstance(run_config, dict)
    recommendation = str(run_config.get("recommendation", ""))
    recommended_row = row_by_method(method_rows, recommendation)
    require(bool(recommendation), failures, "run_config:missing_recommendation")
    require(recommended_row is not None, failures, "run_config:recommendation_not_in_method_comparison")
    require(recommendation in summary_text, failures, "summary:recommendation_not_mentioned")
    require("Method performance table" in summary_text, failures, "summary:missing_method_performance_table")
    require("Final recommendation" in summary_text, failures, "summary:missing_final_recommendation")

    contracts = run_config.get("contracts", {})
    if isinstance(contracts, Mapping):
        require(contracts.get("rdf") == RDF_AXIS_CONTRACT, failures, "run_config:rdf_contract_mismatch")
        require(
            contracts.get("stereodistance_public_topic") == PUBLIC_TOPIC_CONTRACT,
            failures,
            "run_config:public_topic_contract_mismatch",
        )
    else:
        failures.append("run_config:contracts_not_object")

    target_range_m = finite_float(run_config.get("target_range_m"))
    require(abs(target_range_m - 1852.0) <= 1.0e-6, failures, "run_config:target_range_not_1852m")
    train_ratio = finite_float(run_config.get("train_ratio"))
    require(abs(train_ratio - EXPECTED_TRAIN_RATIO) <= 1.0e-9, failures, "run_config:train_ratio_not_80_20")
    train_count = int(run_config.get("train_count", 0) or 0)
    validation_count = int(run_config.get("validation_count", 0) or 0)
    sample_count = int(run_config.get("sample_count", 0) or 0)
    require(train_count > 0, failures, "run_config:no_train_samples")
    require(validation_count > 0, failures, "run_config:no_validation_samples")
    require(train_count + validation_count == sample_count, failures, "run_config:split_count_mismatch")
    if sample_count > 0:
        require(
            abs((validation_count / sample_count) - (1.0 - EXPECTED_TRAIN_RATIO)) <= (1.0 / sample_count),
            failures,
            "run_config:validation_split_not_20_percent",
        )
    require(
        str(run_config.get("split_strategy", "")).startswith("target_window_stratified"),
        failures,
        "run_config:split_strategy_not_target_stratified",
    )
    require(
        int(run_config.get("validation_target_window_count", 0)) > 0,
        failures,
        "run_config:no_validation_target_window_samples",
    )

    method_parameter_names = set(method_parameters.keys()) if isinstance(method_parameters, Mapping) else set()
    for row in method_rows:
        method = str(row.get("method", ""))
        require(row.get("rdf_axis_contract") == RDF_AXIS_CONTRACT, failures, f"method:{method}:rdf_contract_mismatch")
        require(method in method_parameter_names, failures, f"method:{method}:missing_parameters")
        require(finite_float(row.get("validation_range_rmse_m")) < math.inf, failures, f"method:{method}:nonfinite_validation_range_rmse")
        require(finite_float(row.get("validation_target_abs_mean_error_m")) < math.inf, failures, f"method:{method}:nonfinite_target_abs_mean")

    prediction_methods = {row.get("method", "") for row in prediction_rows}
    method_names = {row.get("method", "") for row in method_rows}
    require(prediction_methods == method_names, failures, "validation_predictions:method_set_mismatch")
    for row in prediction_rows[: min(len(prediction_rows), 200)]:
        method = str(row.get("method", ""))
        require(row.get("split") == "validation", failures, f"validation_predictions:{method}:split_not_validation")
        require(row.get("rdf_axis_contract") == RDF_AXIS_CONTRACT, failures, f"validation_predictions:{method}:rdf_contract_mismatch")
        for field in (
            "pred_right_m",
            "pred_down_m",
            "pred_forward_m",
            "pred_range_m",
            "truth_right_m",
            "truth_down_m",
            "truth_forward_m",
            "truth_range_m",
            "error_right_m",
            "error_down_m",
            "error_forward_m",
            "range_error_m",
            "measured_disparity_px",
            "true_disparity_px",
        ):
            require(finite_float(row.get(field)) < math.inf, failures, f"validation_predictions:{method}:nonfinite_{field}")

    if raw_row is not None and recommended_row is not None:
        raw_validation_rmse = finite_float(raw_row.get("validation_range_rmse_m"))
        recommended_validation_rmse = finite_float(recommended_row.get("validation_range_rmse_m"))
        recommended_target_abs = finite_float(recommended_row.get("validation_target_abs_mean_error_m"))
        recommended_target_count = int(float(recommended_row.get("validation_target_window_count", "0") or 0))
        require(
            recommended_validation_rmse < raw_validation_rmse,
            failures,
            "recommendation:validation_range_rmse_not_improved_over_raw",
        )
        require(
            recommended_target_count > 0,
            failures,
            "recommendation:no_target_window_samples",
        )
        require(
            recommended_target_abs <= target_tolerance_m,
            failures,
            "recommendation:target_abs_mean_error_exceeds_tolerance",
        )
        for axis, column in RDF_AXIS_RMSE_COLUMNS.items():
            raw_axis_rmse = finite_float(raw_row.get(column))
            recommended_axis_rmse = finite_float(recommended_row.get(column))
            require(
                recommended_axis_rmse <= raw_axis_rmse + 1.0e-9,
                failures,
                f"recommendation:{axis}_validation_rmse_regressed_over_raw",
            )

    recommended_target_row = row_by_method(target_rows, recommendation)
    require(recommended_target_row is not None, failures, "target_1852m_stats:missing_recommended_method")
    if recommended_target_row is not None:
        require(
            int(float(recommended_target_row.get("sample_count", "0") or 0)) > 0,
            failures,
            "target_1852m_stats:recommended_method_no_samples",
        )
        require(
            finite_float(recommended_target_row.get("abs_mean_error_m")) <= target_tolerance_m,
            failures,
            "target_1852m_stats:recommended_abs_mean_exceeds_tolerance",
        )
        require(
            str(recommended_target_row.get("pass_abs_mean_within_10m", "")).lower() == "yes",
            failures,
            "target_1852m_stats:recommended_not_marked_pass",
        )

    final_policy_rows = [
        row for row in policy_rows if str(row.get("is_final_recommendation", "")).lower() == "true"
    ]
    require(len(final_policy_rows) >= 1, failures, "policy_recommendations:no_final_policy")
    for final_policy in final_policy_rows:
        require(final_policy.get("method") == recommendation, failures, "policy_recommendations:final_method_not_recommendation")
        require(
            finite_float(final_policy.get("validation_range_rmse_m")) < finite_float(final_policy.get("raw_validation_range_rmse_m")),
            failures,
            "policy_recommendations:final_method_not_improved_over_raw",
        )
        require(
            finite_float(final_policy.get("validation_target_abs_mean_error_m")) <= target_tolerance_m,
            failures,
            "policy_recommendations:final_target_abs_mean_exceeds_tolerance",
        )
    if isinstance(policy_json, Mapping):
        require(isinstance(policy_json.get("policies"), list), failures, "policy_recommendations_json:policies_not_list")
        require(
            "RDF(right,down,forward)" in str(policy_json.get("runtime_profile_contract", "")),
            failures,
            "policy_recommendations_json:missing_rdf_runtime_contract",
        )
    else:
        failures.append("policy_recommendations_json:not_object")

    plot_dimensions: dict[str, str] = {}
    for name in ("range_error_plot", "residual_disparity_plot", "rdf_axis_plot", "pareto_plot"):
        dimensions = png_dimensions(paths[name])
        require(dimensions is not None, failures, f"{name}:invalid_png")
        if dimensions is not None:
            width, height = dimensions
            plot_dimensions[name] = f"{width}x{height}"
            require(
                png_is_valid_plot(paths[name]),
                failures,
                f"{name}:png_dimensions_too_small:{width}x{height}",
            )

    runtime_profiles_manifest = result_dir / "runtime_profiles" / "manifest.json"
    require(runtime_profiles_manifest.is_file(), failures, "runtime_profiles:missing_manifest")
    selected_profile_path = ""
    selected_profile_mode = ""
    selected_profile_load_status = ""
    selected_profile_apply_status = ""
    top_level_profile_equivalence_status = ""
    runtime_profile_count = 0
    runtime_profile_load_pass_count = 0
    runtime_profile_missing_methods: list[str] = []
    policy_profile_count = 0
    policy_profile_load_pass_count = 0
    if runtime_profiles_manifest.is_file():
        profiles = load_json(runtime_profiles_manifest)
        if isinstance(profiles, list):
            runtime_profile_count = len(profiles)
            require(runtime_profile_count > 0, failures, "runtime_profiles:no_profiles")
            selected_profiles = [profile for profile in profiles if isinstance(profile, Mapping) and profile.get("selected") is True]
            require(len(selected_profiles) == 1, failures, "runtime_profiles:selected_profile_count_not_one")
            for index, manifest_profile in enumerate(profiles):
                if not isinstance(manifest_profile, Mapping):
                    failures.append(f"runtime_profiles:{index}:manifest_row_not_object")
                    continue
                method = str(manifest_profile.get("method", ""))
                method_row = row_by_method(method_rows, method)
                require(method_row is not None, failures, f"runtime_profiles:{method}:method_not_in_method_comparison")
                require(
                    values_close(
                        manifest_profile.get("validation_range_rmse_m"),
                        method_row.get("validation_range_rmse_m") if method_row else None,
                    ),
                    failures,
                    f"runtime_profiles:{method}:validation_rmse_mismatch",
                )
                require(
                    values_close(
                        manifest_profile.get("target_1852m_abs_mean_error_m"),
                        method_row.get("validation_target_abs_mean_error_m") if method_row else None,
                    ),
                    failures,
                    f"runtime_profiles:{method}:target_error_mismatch",
                )
                profile_path = result_dir / str(manifest_profile.get("profile_path", ""))
                require(profile_path.is_file(), failures, f"runtime_profiles:{method}:profile_file_missing")
                if not profile_path.is_file():
                    continue
                profile_payload = load_json(profile_path)
                if not isinstance(profile_payload, Mapping):
                    failures.append(f"runtime_profiles:{method}:profile_not_object")
                    continue
                require(profile_payload.get("method") == method, failures, f"runtime_profiles:{method}:profile_method_mismatch")
                require("RDF(right,down,forward)" in str(profile_payload.get("axis_contract", "")), failures, f"runtime_profiles:{method}:axis_contract_not_rdf")
                require(not has_forbidden_truth_name(profile_payload.get("runtime_inputs", [])), failures, f"runtime_profiles:{method}:runtime_inputs_include_truth")
                require(truth_forbidden_declared(profile_payload.get("forbidden_runtime_inputs", [])), failures, f"runtime_profiles:{method}:forbidden_truth_inputs_not_declared")
                try:
                    loaded_runtime_profile = load_runtime_profile(profile_payload)
                    apply_ok, apply_reason = runtime_profile_apply_smoke(loaded_runtime_profile)
                    require(apply_ok, failures, f"runtime_profiles:{method}:apply_smoke_failed:{apply_reason}")
                    if apply_ok:
                        runtime_profile_load_pass_count += 1
                except Exception:
                    failures.append(f"runtime_profiles:{method}:runtime_loader_rejected_profile")
            require(
                runtime_profile_load_pass_count == runtime_profile_count,
                failures,
                "runtime_profiles:load_apply_pass_count_mismatch",
            )
            runtime_profile_methods = {
                str(manifest_profile.get("method", ""))
                for manifest_profile in profiles
                if isinstance(manifest_profile, Mapping)
            }
            for row in method_rows:
                method = str(row.get("method", ""))
                if method in runtime_profile_methods:
                    continue
                runtime_profile_missing_methods.append(method)
                family = str(row.get("runtime_profile_family", ""))
                require(
                    family in PROFILE_OPTIONAL_FAMILIES,
                    failures,
                    f"runtime_profiles:{method}:missing_profile_for_runtime_family:{family}",
                )
            if selected_profiles:
                selected_manifest = selected_profiles[0]
                require(selected_manifest.get("method") == recommendation, failures, "runtime_profiles:selected_method_not_recommendation")
                require(values_close(selected_manifest.get("validation_range_rmse_m"), recommended_row.get("validation_range_rmse_m") if recommended_row else None), failures, "runtime_profiles:selected_validation_rmse_mismatch")
                require(values_close(selected_manifest.get("target_1852m_abs_mean_error_m"), recommended_row.get("validation_target_abs_mean_error_m") if recommended_row else None), failures, "runtime_profiles:selected_target_error_mismatch")
                selected_profile_path = str(selected_manifest.get("profile_path", ""))
                profile_path = result_dir / selected_profile_path
                require(profile_path.is_file(), failures, "runtime_profiles:selected_profile_file_missing")
                if profile_path.is_file():
                    selected_profile = load_json(profile_path)
                    if isinstance(selected_profile, Mapping):
                        selected_profile_mode = str(selected_profile.get("mode", ""))
                        require(selected_profile.get("method") == recommendation, failures, "selected_profile:method_not_recommendation")
                        require(bool(selected_profile_mode), failures, "selected_profile:missing_mode")
                        require("RDF(right,down,forward)" in str(selected_profile.get("axis_contract", "")), failures, "selected_profile:axis_contract_not_rdf")
                        require(not has_forbidden_truth_name(selected_profile.get("runtime_inputs", [])), failures, "selected_profile:runtime_inputs_include_truth")
                        require(truth_forbidden_declared(selected_profile.get("forbidden_runtime_inputs", [])), failures, "selected_profile:forbidden_truth_inputs_not_declared")
                        validation_summary = selected_profile.get("validation_summary", {})
                        if isinstance(validation_summary, Mapping):
                            require(values_close(validation_summary.get("validation_range_rmse_raw_m"), raw_row.get("validation_range_rmse_m") if raw_row else None), failures, "selected_profile:raw_validation_rmse_mismatch")
                            require(values_close(validation_summary.get("validation_range_rmse_corrected_m"), recommended_row.get("validation_range_rmse_m") if recommended_row else None), failures, "selected_profile:corrected_validation_rmse_mismatch")
                            require(values_close(validation_summary.get("target_1852m_abs_mean_error_m"), recommended_row.get("validation_target_abs_mean_error_m") if recommended_row else None), failures, "selected_profile:target_error_mismatch")
                        else:
                            failures.append("selected_profile:validation_summary_not_object")
                        try:
                            loaded_profile = load_runtime_profile(selected_profile)
                            selected_profile_load_status = "PASS"
                            apply_ok, apply_reason = runtime_profile_apply_smoke(loaded_profile)
                            selected_profile_apply_status = "PASS" if apply_ok else f"FAIL:{apply_reason}"
                            require(apply_ok, failures, f"selected_profile:apply_smoke_failed:{apply_reason}")
                        except Exception as exc:
                            selected_profile_load_status = f"FAIL:{exc}"
                            failures.append("selected_profile:runtime_loader_rejected_profile")
                        if isinstance(top_level_runtime_profile, Mapping):
                            if dict(top_level_runtime_profile) == dict(selected_profile):
                                top_level_profile_equivalence_status = "PASS"
                            else:
                                top_level_profile_equivalence_status = "FAIL"
                                failures.append("top_level_runtime_profile:not_equal_selected_profile")
                            require(
                                top_level_runtime_profile.get("method") == recommendation,
                                failures,
                                "top_level_runtime_profile:method_not_recommendation",
                            )
                            require(
                                top_level_runtime_profile.get("mode") == selected_profile_mode,
                                failures,
                                "top_level_runtime_profile:mode_not_selected_profile_mode",
                            )
                        else:
                            failures.append("top_level_runtime_profile:not_object")
                    else:
                        failures.append("selected_profile:not_object")
        else:
            failures.append("runtime_profiles:manifest_not_list")

    policy_profiles_manifest = result_dir / "policy_profiles" / "manifest.json"
    require(policy_profiles_manifest.is_file(), failures, "policy_profiles:missing_manifest")
    if policy_profiles_manifest.is_file():
        policy_profiles = load_json(policy_profiles_manifest)
        if isinstance(policy_profiles, list):
            policy_profile_count = len(policy_profiles)
            require(policy_profile_count == len(policy_rows), failures, "policy_profiles:manifest_count_mismatch")
            policy_methods = {(row.get("policy"), row.get("method")) for row in policy_rows}
            for profile_row in policy_profiles:
                if not isinstance(profile_row, Mapping):
                    failures.append("policy_profiles:row_not_object")
                    continue
                policy = str(profile_row.get("policy", ""))
                method = str(profile_row.get("method", ""))
                require((policy, method) in policy_methods, failures, f"policy_profiles:{policy}:not_in_policy_recommendations")
                method_row = row_by_method(method_rows, method)
                require(method_row is not None, failures, f"policy_profiles:{policy}:method_not_in_method_comparison")
                require(values_close(profile_row.get("validation_range_rmse_m"), method_row.get("validation_range_rmse_m") if method_row else None), failures, f"policy_profiles:{policy}:validation_rmse_mismatch")
                require(values_close(profile_row.get("target_1852m_abs_mean_error_m"), method_row.get("validation_target_abs_mean_error_m") if method_row else None), failures, f"policy_profiles:{policy}:target_error_mismatch")
                profile_path = result_dir / str(profile_row.get("profile_path", ""))
                require(profile_path.is_file(), failures, f"policy_profiles:{policy}:profile_file_missing")
                if not profile_path.is_file():
                    continue
                profile_payload = load_json(profile_path)
                if not isinstance(profile_payload, Mapping):
                    failures.append(f"policy_profiles:{policy}:profile_not_object")
                    continue
                require(profile_payload.get("policy") == policy, failures, f"policy_profiles:{policy}:profile_policy_mismatch")
                require(profile_payload.get("method") == method, failures, f"policy_profiles:{policy}:profile_method_mismatch")
                require("RDF(right,down,forward)" in str(profile_payload.get("axis_contract", "")), failures, f"policy_profiles:{policy}:axis_contract_not_rdf")
                require(not has_forbidden_truth_name(profile_payload.get("runtime_inputs", [])), failures, f"policy_profiles:{policy}:runtime_inputs_include_truth")
                try:
                    loaded_policy_profile = load_runtime_profile(profile_payload)
                    apply_ok, apply_reason = runtime_profile_apply_smoke(loaded_policy_profile)
                    require(apply_ok, failures, f"policy_profiles:{policy}:apply_smoke_failed:{apply_reason}")
                    if apply_ok:
                        policy_profile_load_pass_count += 1
                except Exception:
                    failures.append(f"policy_profiles:{policy}:runtime_loader_rejected_profile")
        else:
            failures.append("policy_profiles:manifest_not_list")

    deliverable_evidence_matrix = build_deliverable_evidence_matrix(
        result_dir=result_dir,
        recommendation=recommendation,
        method_rows=method_rows,
        prediction_rows=prediction_rows,
        target_rows=target_rows,
        selected_profile_path=selected_profile_path,
        runtime_profile_count=runtime_profile_count,
        runtime_profile_load_pass_count=runtime_profile_load_pass_count,
        policy_profile_count=policy_profile_count,
        policy_profile_load_pass_count=policy_profile_load_pass_count,
        plot_dimensions=plot_dimensions,
        runtime_profile_missing_methods=runtime_profile_missing_methods,
        target_tolerance_m=target_tolerance_m,
    )
    for name, item in deliverable_evidence_matrix.items():
        require(item.get("status") == "PASS", failures, f"deliverable_evidence:{name}:not_pass")

    return {
        "status": "PASS" if not failures else "FAIL",
        "result_dir": str(result_dir),
        "recommendation": recommendation,
        "method_count": len(method_rows),
        "target_1852m_method_count": len(target_rows),
        "validation_prediction_row_count": len(prediction_rows),
        "policy_count": len(policy_rows),
        "train_count": train_count if "train_count" in locals() else 0,
        "validation_count": validation_count if "validation_count" in locals() else 0,
        "selected_profile_path": selected_profile_path,
        "selected_profile_mode": selected_profile_mode,
        "selected_profile_load_status": selected_profile_load_status,
        "selected_profile_apply_status": selected_profile_apply_status,
        "top_level_profile_equivalence_status": top_level_profile_equivalence_status,
        "runtime_profile_count": runtime_profile_count,
        "runtime_profile_load_pass_count": runtime_profile_load_pass_count,
        "runtime_profile_missing_methods": runtime_profile_missing_methods,
        "policy_profile_count": policy_profile_count,
        "policy_profile_load_pass_count": policy_profile_load_pass_count,
        "deliverable_evidence_matrix": deliverable_evidence_matrix,
        "plot_dimensions": plot_dimensions if "plot_dimensions" in locals() else {},
        "target_tolerance_m": target_tolerance_m,
        "failure_count": len(failures),
        "failures": failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, result: Mapping[str, object]) -> None:
    lines = [
        "RTK stereo calibration output audit",
        f"result_dir: {result['result_dir']}",
        f"status: {result['status']}",
        f"recommendation: {result.get('recommendation', '')}",
        f"method_count: {result.get('method_count', 0)}",
        f"target_1852m_method_count: {result.get('target_1852m_method_count', 0)}",
        f"validation_prediction_row_count: {result.get('validation_prediction_row_count', 0)}",
        f"policy_count: {result.get('policy_count', 0)}",
        f"train_count: {result.get('train_count', 0)}",
        f"validation_count: {result.get('validation_count', 0)}",
        f"selected_profile_path: {result.get('selected_profile_path', '')}",
        f"selected_profile_mode: {result.get('selected_profile_mode', '')}",
        f"selected_profile_load_status: {result.get('selected_profile_load_status', '')}",
        f"selected_profile_apply_status: {result.get('selected_profile_apply_status', '')}",
        f"top_level_profile_equivalence_status: {result.get('top_level_profile_equivalence_status', '')}",
        f"runtime_profile_count: {result.get('runtime_profile_count', 0)}",
        f"runtime_profile_load_pass_count: {result.get('runtime_profile_load_pass_count', 0)}",
        f"runtime_profile_missing_methods: {json.dumps(result.get('runtime_profile_missing_methods', []), sort_keys=True)}",
        f"policy_profile_count: {result.get('policy_profile_count', 0)}",
        f"policy_profile_load_pass_count: {result.get('policy_profile_load_pass_count', 0)}",
        f"plot_dimensions: {json.dumps(result.get('plot_dimensions', {}), sort_keys=True)}",
        f"target_tolerance_m: {result.get('target_tolerance_m', '')}",
        f"failure_count: {result['failure_count']}",
        "runtime_truth_used: False",
        "Coordinate contract: PX4/RTK NED truth is calibration/evaluation only; runtime profiles consume stereo-center RDF(right,down,forward); public StereoDistance alias is FRD+range.",
    ]
    evidence_matrix = result.get("deliverable_evidence_matrix", {})
    if isinstance(evidence_matrix, Mapping):
        lines.append("Deliverable evidence matrix:")
        for name, item in evidence_matrix.items():
            if not isinstance(item, Mapping):
                continue
            evidence = item.get("evidence", {})
            lines.append(
                f"- {name}: status={item.get('status', '')}, path={item.get('path', '')}, "
                f"evidence={json.dumps(evidence, sort_keys=True)}"
            )
    for failure in result.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit final RTK stereo calibration pipeline outputs.")
    parser.add_argument("--result-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--target-tolerance-m", type=float, default=10.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = audit_outputs(args.result_dir, target_tolerance_m=args.target_tolerance_m)
    output_dir = args.output_dir if args.output_dir is not None else args.result_dir / "output_audit"
    write_json(output_dir / "rtk_stereo_calibration_output_audit.json", result)
    write_text(output_dir / "rtk_stereo_calibration_output_audit.txt", result)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    if result["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
