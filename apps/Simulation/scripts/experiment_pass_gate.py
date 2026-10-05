#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


AXES = ("right", "down", "forward")
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


def first_finite(payload: dict[str, Any], *keys: str) -> float | None:
    for key in keys:
        value = finite_float(payload.get(key))
        if value is not None:
            return value
    return None


def first_present(payload: dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in payload:
            return payload[key]
    return None


def collect_contract_text(payload: dict[str, Any]) -> str:
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
    if isinstance(contracts, dict):
        for value in contracts.values():
            parts.append(str(value))
    return "\n".join(parts)


def direct_axis_alias_forbidden(payload: dict[str, Any]) -> bool:
    for key in (
        "direct_ned_to_rdf_axis_alias_allowed",
        "direct_source_axis_alias_allowed",
        "direct_axis_alias_allowed",
    ):
        if bool(payload.get(key, False)):
            return False
    contracts = payload.get("contracts")
    if isinstance(contracts, dict):
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

    target_range_m = first_finite(payload, "target_range_m", "target_1852m_range_m")
    target_error = first_finite(payload, "target_range_abs_mean_error_m", "target_1852m_abs_mean_error_m")
    target_tolerance = first_finite(payload, "target_range_tolerance_m", "target_1852m_tolerance_m") or 10.0
    target_count = first_present(payload, "target_range_sample_count", "target_1852m_sample_count")
    min_target_count = first_present(payload, "target_range_min_sample_count", "target_1852m_min_sample_count")
    if min_target_count is None:
        min_target_count = 3
    try:
        target_count_int = int(target_count)
    except (TypeError, ValueError):
        target_count_int = 0
    try:
        min_target_count_int = int(min_target_count)
    except (TypeError, ValueError):
        min_target_count_int = 3
    checks["target_range_m"] = target_range_m
    checks["target_range_error_within_tolerance"] = (
        target_error is not None and target_error <= target_tolerance
    )
    checks["target_range_sample_count_ok"] = target_count_int >= min_target_count_int
    checks["target_range_sample_count"] = target_count_int
    checks["target_range_min_sample_count"] = min_target_count_int
    checks["target_1852m_error_within_tolerance"] = checks["target_range_error_within_tolerance"]
    checks["target_1852m_sample_count_ok"] = checks["target_range_sample_count_ok"]
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
    checks["direct_axis_alias_forbidden"] = direct_axis_alias_forbidden(payload)

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
    if not checks["direct_axis_alias_forbidden"]:
        hard_fail_reasons.append("direct_axis_alias_not_forbidden")
    if not checks["validation_range_improved"]:
        hard_fail_reasons.append("validation_range_not_improved")
    if not checks["target_range_error_within_tolerance"]:
        hard_fail_reasons.append("target_range_error_fail")

    strict_pass = (
        not hard_fail_reasons
        and checks["validation_position_improved"]
        and checks["axis_reported"]
        and checks["axis_strict_improvement"]
        and checks["target_range_sample_count_ok"]
        and checks["train_validation_gap_stable"]
    )
    pass_result = (
        not hard_fail_reasons
        and checks["axis_reported"]
        and checks["axis_nonregression"]
        and checks["target_range_sample_count_ok"]
    )
    conditional = (
        not hard_fail_reasons
        and (
            not checks["validation_position_improved"]
            or not checks["axis_nonregression"]
            or not checks["target_range_sample_count_ok"]
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

    if not checks["target_range_sample_count_ok"] and "target_range_under_sampled" not in hard_fail_reasons:
        hard_fail_reasons.append("target_range_under_sampled")
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
