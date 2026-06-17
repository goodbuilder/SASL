#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


AXES = ("right", "down", "forward")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate the Estimated-to-Truth Runtime PASS gate for an experiment "
            "metrics.json file."
        )
    )
    parser.add_argument("metrics_json", type=Path)
    parser.add_argument(
        "--write",
        action="store_true",
        help="Write gate_result fields back into metrics.json.",
    )
    return parser.parse_args()


def finite_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def bool_value(payload: dict[str, Any], key: str) -> bool:
    return bool(payload.get(key, False))


def improved(raw: float | None, corrected: float | None, *, strict: bool) -> bool:
    if raw is None or corrected is None:
        return False
    return corrected < raw if strict else corrected <= raw


def evaluate(payload: dict[str, Any]) -> dict[str, Any]:
    checks: dict[str, Any] = {}

    range_raw = finite_float(payload.get("validation_range_rmse_raw_m"))
    range_corr = finite_float(payload.get("validation_range_rmse_corrected_m"))
    position_raw = finite_float(payload.get("validation_position_rmse_raw_m"))
    position_corr = finite_float(payload.get("validation_position_rmse_corrected_m"))
    checks["validation_range_improved"] = improved(range_raw, range_corr, strict=True)
    checks["validation_position_improved"] = improved(position_raw, position_corr, strict=True)

    axis_nonregression = True
    axis_strict = True
    axis_reported = True
    axis_checks = {}
    for axis in AXES:
        raw = finite_float(payload.get(f"validation_{axis}_rmse_raw_m"))
        corrected = finite_float(payload.get(f"validation_{axis}_rmse_corrected_m"))
        if raw is None or corrected is None:
            axis_reported = False
        axis_checks[axis] = {
            "raw_rmse_m": raw,
            "corrected_rmse_m": corrected,
            "nonregression": improved(raw, corrected, strict=False),
            "strict_improvement": improved(raw, corrected, strict=True),
        }
        axis_nonregression = axis_nonregression and axis_checks[axis]["nonregression"]
        axis_strict = axis_strict and axis_checks[axis]["strict_improvement"]
    checks["axis_reported"] = axis_reported
    checks["axis_nonregression"] = axis_nonregression
    checks["axis_strict_improvement"] = axis_strict
    checks["axis_checks"] = axis_checks

    target_error = finite_float(payload.get("target_1852m_abs_mean_error_m"))
    target_tolerance = finite_float(payload.get("target_1852m_tolerance_m")) or 10.0
    target_count = payload.get("target_1852m_sample_count")
    min_target_count = payload.get("target_1852m_min_sample_count", 3)
    try:
        target_count_int = int(target_count)
    except (TypeError, ValueError):
        target_count_int = 0
    try:
        min_target_count_int = int(min_target_count)
    except (TypeError, ValueError):
        min_target_count_int = 3
    checks["target_1852m_error_within_tolerance"] = (
        target_error is not None and target_error <= target_tolerance
    )
    checks["target_1852m_sample_count_ok"] = target_count_int >= min_target_count_int
    checks["target_1852m_sample_count"] = target_count_int
    checks["target_1852m_min_sample_count"] = min_target_count_int

    max_gap = finite_float(payload.get("max_train_validation_gap_range_rmse_m"))
    gap = finite_float(payload.get("train_validation_gap_range_rmse_m"))
    checks["train_validation_gap_reported"] = gap is not None
    checks["train_validation_gap_stable"] = True if max_gap is None or gap is None else gap <= max_gap

    checks["runtime_truth_not_used"] = not bool_value(payload, "runtime_truth_used")
    checks["truth_point_defined"] = bool_value(payload, "truth_point_defined")
    checks["runtime_feature_defined"] = bool_value(payload, "runtime_feature_defined")
    checks["automated_search"] = bool_value(payload, "automated_search")
    checks["search_space_defined"] = bool_value(payload, "search_space_defined")
    checks["selection_metric_defined"] = bool_value(payload, "selection_metric_defined")
    checks["stereodistance_applicable"] = bool_value(payload, "stereodistance_applicable")
    checks["runtime_profile_exportable"] = bool_value(payload, "runtime_profile_exportable")
    checks["offline_only"] = bool_value(payload, "offline_only")
    checks["coordinate_contracts_explicit"] = bool_value(payload, "coordinate_contracts_explicit")
    checks["alias_contracts_explicit"] = bool_value(payload, "alias_contracts_explicit")

    hard_fail_reasons = []
    if not checks["runtime_truth_not_used"]:
        hard_fail_reasons.append("runtime_truth_used")
    if not checks["truth_point_defined"]:
        hard_fail_reasons.append("truth_point_undefined")
    if not checks["runtime_feature_defined"]:
        hard_fail_reasons.append("runtime_feature_undefined")
    if checks["automated_search"] and not checks["search_space_defined"]:
        hard_fail_reasons.append("search_space_undefined")
    if checks["automated_search"] and not checks["selection_metric_defined"]:
        hard_fail_reasons.append("selection_metric_undefined")
    if not checks["stereodistance_applicable"]:
        hard_fail_reasons.append("stereodistance_applicability_missing")
    if not checks["runtime_profile_exportable"]:
        hard_fail_reasons.append("runtime_profile_not_exportable")
    if checks["offline_only"]:
        hard_fail_reasons.append("offline_only")
    if not checks["coordinate_contracts_explicit"]:
        hard_fail_reasons.append("coordinate_contracts_missing")
    if not checks["alias_contracts_explicit"]:
        hard_fail_reasons.append("alias_contracts_missing")
    if not checks["validation_range_improved"]:
        hard_fail_reasons.append("validation_range_not_improved")
    if not checks["target_1852m_error_within_tolerance"]:
        hard_fail_reasons.append("target_1852m_error_fail")

    strict_pass = (
        not hard_fail_reasons
        and checks["validation_position_improved"]
        and checks["axis_reported"]
        and checks["axis_strict_improvement"]
        and checks["target_1852m_sample_count_ok"]
        and checks["train_validation_gap_stable"]
    )
    pass_result = (
        not hard_fail_reasons
        and checks["axis_reported"]
        and checks["axis_nonregression"]
        and checks["target_1852m_sample_count_ok"]
    )
    conditional = (
        not hard_fail_reasons
        and (
            not checks["validation_position_improved"]
            or not checks["axis_nonregression"]
            or not checks["target_1852m_sample_count_ok"]
            or not checks["train_validation_gap_stable"]
        )
    )

    if strict_pass:
        verdict = "STRICT_PASS"
    elif pass_result:
        verdict = "PASS"
    elif conditional:
        verdict = "CONDITIONAL_PASS"
    else:
        verdict = "FAIL"

    if not checks["target_1852m_sample_count_ok"] and "target_1852m_under_sampled" not in hard_fail_reasons:
        hard_fail_reasons.append("target_1852m_under_sampled")
    if checks["axis_reported"] and not checks["axis_nonregression"]:
        hard_fail_reasons.append("axis_regression")
    if not checks["validation_position_improved"]:
        hard_fail_reasons.append("validation_position_not_improved")
    if not checks["train_validation_gap_stable"]:
        hard_fail_reasons.append("train_validation_gap_unstable")

    return {
        "gate": "estimated_to_truth_runtime_pass",
        "verdict": verdict,
        "checks": checks,
        "reasons": hard_fail_reasons,
    }


def main() -> int:
    args = parse_args()
    payload = json.loads(args.metrics_json.read_text(encoding="utf-8"))
    result = evaluate(payload)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.write:
        payload["gate_result"] = result
        payload["status"] = result["verdict"]
        args.metrics_json.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    return 0 if result["verdict"] in {"STRICT_PASS", "PASS"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
