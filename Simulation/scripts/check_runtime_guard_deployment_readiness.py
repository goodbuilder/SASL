#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
RUNTIME_INPUT_CONTRACT = (
    "runtime guard selection uses truth-free raw stereo-center RDF(right,down,forward), raw range, "
    "left image center, and measured disparity; PX4/RTK/NED truth is audit-only"
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check runtime_guard automatic deployment readiness evidence.")
    parser.add_argument("--known-guard-summary-json", type=Path, required=True)
    parser.add_argument("--loso-policy-summary-json", type=Path, required=True)
    parser.add_argument("--batch-smoke-summary-json", type=Path, required=True)
    parser.add_argument("--operator-readiness-summary-json", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def encode_csv_value(value: object) -> object:
    if isinstance(value, list):
        return ";".join(str(item) for item in value)
    return value


def write_csv(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(payload.keys()))
        writer.writeheader()
        writer.writerow({key: encode_csv_value(value) for key, value in payload.items()})


def finite_float(value: object, default: float = math.inf) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def finite_int(value: object, default: int = 0) -> int:
    parsed = finite_float(value, float(default))
    return int(parsed) if math.isfinite(parsed) else default


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def contract_ok(payload: Mapping[str, object]) -> bool:
    return (
        "RDF(right,down,forward)" in str(payload.get("axis_contract", ""))
        and "FRD_range" in str(payload.get("public_topic_contract", ""))
    )


def check_readiness(
    *,
    known_guard_summary_json: Path,
    loso_policy_summary_json: Path,
    batch_smoke_summary_json: Path,
    operator_readiness_summary_json: Path,
    target_error_limit_m: float = 10.0,
) -> dict[str, object]:
    failures: list[str] = []
    for label, path in (
        ("known_guard_summary", known_guard_summary_json),
        ("loso_policy_summary", loso_policy_summary_json),
        ("batch_smoke_summary", batch_smoke_summary_json),
        ("operator_readiness_summary", operator_readiness_summary_json),
    ):
        require(path.is_file(), failures, f"{label}:missing")

    known = load_json(known_guard_summary_json) if known_guard_summary_json.is_file() else {}
    loso = load_json(loso_policy_summary_json) if loso_policy_summary_json.is_file() else {}
    batch = load_json(batch_smoke_summary_json) if batch_smoke_summary_json.is_file() else {}
    operator = load_json(operator_readiness_summary_json) if operator_readiness_summary_json.is_file() else {}

    known_count = finite_int(known.get("audit_count"))
    known_pass = finite_int(known.get("pass_count"))
    loso_count = finite_int(loso.get("recommended_audit_count"))
    loso_pass = finite_int(loso.get("recommended_pass_count"))
    batch_count = finite_int(batch.get("run_count"))
    batch_pass = finite_int(batch.get("pass_count"))
    operator_count = finite_int(operator.get("run_count"))
    operator_pass = finite_int(operator.get("pass_count"))
    known_target_max = finite_float(known.get("target_max_abs_error_max_m"))
    loso_target_max = finite_float(loso.get("recommended_target_max_abs_error_max_m"))

    require(known.get("mode") == "runtime_measurement_guard_audit", failures, "known_guard:mode")
    require(bool(known.get("pass_all", False)), failures, "known_guard:pass_all")
    require(bool(known.get("validation_only_pass_all", False)), failures, "known_guard:validation_only_pass_all")
    require(known_count > 0 and known_pass == known_count, failures, "known_guard:pass_count")
    require(known_target_max <= target_error_limit_m, failures, "known_guard:target_limit")
    require(not bool(known.get("runtime_truth_used", False)), failures, "known_guard:runtime_truth_used")
    require(contract_ok(known), failures, "known_guard:coordinate_contract")
    require("raw_right_m" in str(known.get("selection_input_fields", "")), failures, "known_guard:selection_inputs")

    require(loso.get("mode") == "runtime_measurement_guard_policy_audit", failures, "loso_policy:mode")
    require(loso_count > 0 and loso_pass == loso_count, failures, "loso_policy:pass_count")
    require(loso_target_max <= target_error_limit_m, failures, "loso_policy:target_limit")
    require(contract_ok(loso), failures, "loso_policy:coordinate_contract")

    require(batch.get("mode") == "runtime_guard_operating_mode_batch_smoke", failures, "batch_smoke:mode")
    require(batch.get("status") == "PASS", failures, "batch_smoke:status")
    require(batch_count > 0 and batch_pass == batch_count, failures, "batch_smoke:pass_count")
    require(not bool(batch.get("runtime_truth_used", True)), failures, "batch_smoke:runtime_truth_used")
    require(contract_ok(batch), failures, "batch_smoke:coordinate_contract")

    require(operator.get("mode") == "runtime_guard_batch_operator_view_readiness", failures, "operator_readiness:mode")
    require(operator.get("status") == "PASS", failures, "operator_readiness:status")
    require(operator_count > 0 and operator_pass == operator_count, failures, "operator_readiness:pass_count")
    require(operator_count == batch_count, failures, "operator_readiness:run_count_matches_batch")
    require(not bool(operator.get("runtime_truth_used", True)), failures, "operator_readiness:runtime_truth_used")
    require(contract_ok(operator), failures, "operator_readiness:coordinate_contract")

    recommendation = "runtime_guard_candidate"
    if not failures and loso_target_max <= target_error_limit_m and known_target_max <= target_error_limit_m:
        recommendation = "runtime_guard_candidate_ready_for_real_calibration_csv_replay"

    return {
        "mode": "runtime_guard_deployment_readiness",
        "status": "PASS" if not failures else "FAIL",
        "recommendation": recommendation,
        "known_guard_summary_json": str(known_guard_summary_json),
        "loso_policy_summary_json": str(loso_policy_summary_json),
        "batch_smoke_summary_json": str(batch_smoke_summary_json),
        "operator_readiness_summary_json": str(operator_readiness_summary_json),
        "known_guard_pass_count": known_pass,
        "known_guard_audit_count": known_count,
        "known_guard_rmse_mean_m": known.get("validation_range_rmse_corrected_mean_m", ""),
        "known_guard_rmse_max_m": known.get("validation_range_rmse_corrected_max_m", ""),
        "known_guard_target_max_abs_m": known.get("target_max_abs_error_max_m", ""),
        "loso_policy": loso.get("recommended_selection_policy", ""),
        "loso_policy_pass_count": loso_pass,
        "loso_policy_audit_count": loso_count,
        "loso_policy_rmse_mean_m": loso.get("recommended_validation_range_rmse_corrected_mean_m", ""),
        "loso_policy_rmse_max_m": loso.get("recommended_validation_range_rmse_corrected_max_m", ""),
        "loso_policy_target_max_abs_m": loso.get("recommended_target_max_abs_error_max_m", ""),
        "batch_smoke_pass_count": batch_pass,
        "batch_smoke_run_count": batch_count,
        "operator_readiness_pass_count": operator_pass,
        "operator_readiness_run_count": operator_count,
        "target_error_limit_m": target_error_limit_m,
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": RUNTIME_INPUT_CONTRACT,
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Runtime guard deployment readiness",
        f"status: {payload.get('status', '')}",
        f"recommendation: {payload.get('recommendation', '')}",
        f"known_guard_pass: {payload.get('known_guard_pass_count', 0)}/{payload.get('known_guard_audit_count', 0)}",
        f"known_guard_rmse_mean/max_m: {payload.get('known_guard_rmse_mean_m', '')}/{payload.get('known_guard_rmse_max_m', '')}",
        f"known_guard_target_max_abs_m: {payload.get('known_guard_target_max_abs_m', '')}",
        f"loso_policy: {payload.get('loso_policy', '')}",
        f"loso_policy_pass: {payload.get('loso_policy_pass_count', 0)}/{payload.get('loso_policy_audit_count', 0)}",
        f"loso_policy_rmse_mean/max_m: {payload.get('loso_policy_rmse_mean_m', '')}/{payload.get('loso_policy_rmse_max_m', '')}",
        f"loso_policy_target_max_abs_m: {payload.get('loso_policy_target_max_abs_m', '')}",
        f"batch_smoke_pass: {payload.get('batch_smoke_pass_count', 0)}/{payload.get('batch_smoke_run_count', 0)}",
        f"operator_readiness_pass: {payload.get('operator_readiness_pass_count', 0)}/{payload.get('operator_readiness_run_count', 0)}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"axis_contract: {payload.get('axis_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
        f"runtime_input_contract: {payload.get('runtime_input_contract', '')}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.extend(["", "Failures:"])
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    payload = check_readiness(
        known_guard_summary_json=args.known_guard_summary_json,
        loso_policy_summary_json=args.loso_policy_summary_json,
        batch_smoke_summary_json=args.batch_smoke_summary_json,
        operator_readiness_summary_json=args.operator_readiness_summary_json,
        target_error_limit_m=args.target_error_limit_m,
    )
    if args.output_json is not None:
        write_json(args.output_json, payload)
    if args.output_txt is not None:
        write_text(args.output_txt, payload)
    if args.output_csv is not None:
        write_csv(args.output_csv, payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
