#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check readiness evidence for a policy-selected deployable StereoDistance active-profile candidate. "
            "This gate is for policy-selection candidates and does not require matching the matrix score winner."
        )
    )
    parser.add_argument("--candidate-export-json", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    parser.add_argument("--output-csv", type=Path)
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_path(anchor: Path, value: object) -> Path:
    path = Path(str(value))
    if path.is_absolute() or path.exists():
        return path
    return anchor.parent / path


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def has_runtime_truth(values: object) -> bool:
    if not isinstance(values, list):
        return True
    joined = " ".join(str(value).lower() for value in values)
    return any(token in joined for token in ("truth", "gt_", "px4", "rtk", "ned"))


def bool_value(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "pass"}


def int_value(value: object) -> int:
    try:
        return int(float(str(value)))
    except (TypeError, ValueError):
        return 0


def read_public_payload_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def public_payload_rows_valid(rows: Sequence[Mapping[str, str]]) -> bool:
    for row in rows:
        if row.get("data_axis_order") != "forward_m,right_m,down_m,range_m":
            return False
        if row.get("runtime_truth_used") != "False":
            return False
        if row.get("node_public_payload_equivalent") != "True":
            return False
        try:
            values = json.loads(row.get("data", "[]"))
            if not (isinstance(values, list) and len(values) == 4 and all(math.isfinite(float(v)) for v in values)):
                return False
        except (TypeError, ValueError, json.JSONDecodeError):
            return False
    return True


def check_candidate_readiness(candidate_export_json: Path) -> dict[str, object]:
    failures: list[str] = []
    candidate = load_json(candidate_export_json)
    require(candidate.get("mode") == "deployable_profile_candidate_export", failures, "candidate:mode")
    require(candidate.get("status") == "PASS", failures, "candidate:status")
    require(
        candidate.get("recommendation") == "use_as_next_acceptance_dashboard_active_profile",
        failures,
        "candidate:recommendation",
    )
    require(not bool(candidate.get("runtime_truth_used", True)), failures, "candidate:runtime_truth_used")
    require(bool_value(candidate.get("profile_sha256_matches_policy")), failures, "candidate:profile_sha256_matches_policy")
    require(bool_value(candidate.get("promotion_check_pass")), failures, "candidate:promotion_check_pass")
    require(int_value(candidate.get("promotion_sample_apply_count")) > 0, failures, "candidate:promotion_sample_apply_count")
    require(int_value(candidate.get("promotion_sample_invalid_count")) == 0, failures, "candidate:promotion_sample_invalid_count")
    require(candidate.get("public_topic_payload_status") == "PASS", failures, "candidate:public_topic_payload_status")
    require(int_value(candidate.get("public_topic_payload_sample_count")) > 0, failures, "candidate:public_topic_payload_sample_count")
    require(int_value(candidate.get("public_topic_payload_invalid_count")) == 0, failures, "candidate:public_topic_payload_invalid_count")
    require(
        candidate.get("public_topic_payload_node_equivalence_status") == "PASS",
        failures,
        "candidate:public_topic_payload_node_equivalence",
    )
    require(int_value(candidate.get("candidate_pass_count")) == int_value(candidate.get("candidate_expected_run_count")), failures, "candidate:pass_count")
    require(int_value(candidate.get("candidate_axis_pass_count")) == int_value(candidate.get("candidate_expected_run_count")), failures, "candidate:axis_pass_count")
    require(int_value(candidate.get("candidate_target_pass_count")) == int_value(candidate.get("candidate_expected_run_count")), failures, "candidate:target_pass_count")
    require("RDF(right,down,forward)" in str(candidate.get("axis_contract", "")), failures, "candidate:axis_contract")
    require("FRD_range(forward,right,down,range)" in str(candidate.get("public_topic_contract", "")), failures, "candidate:public_topic_contract")
    require("stereo-center RDF(right,down,forward)" in str(candidate.get("runtime_input_contract", "")), failures, "candidate:runtime_input_contract")

    active_profile = resolve_path(candidate_export_json, candidate.get("output_active_profile", ""))
    promotion_json = resolve_path(candidate_export_json, candidate.get("promotion_check_json", ""))
    public_payload_json = resolve_path(candidate_export_json, candidate.get("public_topic_payload_json", ""))
    public_payload_csv = resolve_path(candidate_export_json, candidate.get("public_topic_payload_csv", ""))
    policy_json = resolve_path(candidate_export_json, candidate.get("policy_selection_json", ""))
    require(active_profile.is_file(), failures, "active_profile:missing")
    require(promotion_json.is_file(), failures, "promotion_check:missing")
    require(public_payload_json.is_file(), failures, "public_payload_json:missing")
    require(public_payload_csv.is_file(), failures, "public_payload_csv:missing")
    require(policy_json.is_file(), failures, "policy_selection:missing")

    active_sha = sha256_file(active_profile) if active_profile.is_file() else ""
    require(active_sha == str(candidate.get("active_profile_sha256", "")), failures, "active_profile:sha256")

    profile_payload: dict[str, object] = {}
    if active_profile.is_file():
        profile_payload = load_json(active_profile)
        require(str(profile_payload.get("method", "")) == str(candidate.get("method", "")), failures, "active_profile:method")
        require("RDF(right,down,forward)" in str(profile_payload.get("axis_contract", "")), failures, "active_profile:axis_contract")
        require(not has_runtime_truth(profile_payload.get("runtime_inputs", [])), failures, "active_profile:runtime_inputs_truth")
        require(
            all(token in " ".join(str(v).lower() for v in profile_payload.get("forbidden_runtime_inputs", [])) for token in ("truth", "px4", "rtk")),
            failures,
            "active_profile:forbidden_runtime_inputs",
        )

    promotion: dict[str, object] = {}
    if promotion_json.is_file():
        promotion = load_json(promotion_json)
        require(bool(promotion.get("pass_promotion_check", False)), failures, "promotion:pass")
        require(bool(promotion.get("schema_load_ok", False)), failures, "promotion:schema_load")
        require(bool(promotion.get("node_loader_ok", False)), failures, "promotion:node_loader")
        require(bool(promotion.get("axis_contract_declared", False)), failures, "promotion:axis_contract")
        require(not bool(promotion.get("runtime_truth_used", True)), failures, "promotion:runtime_truth_used")
        require(int_value(promotion.get("sample_apply_count")) == int_value(candidate.get("promotion_sample_apply_count")), failures, "promotion:sample_apply_count")
        require(int_value(promotion.get("sample_invalid_count")) == 0, failures, "promotion:sample_invalid_count")

    public_payload: dict[str, object] = {}
    public_rows = read_public_payload_rows(public_payload_csv)
    if public_payload_json.is_file():
        public_payload = load_json(public_payload_json)
        require(public_payload.get("status") == "PASS", failures, "public_payload:status")
        require(not bool(public_payload.get("runtime_truth_used", True)), failures, "public_payload:runtime_truth_used")
        require(public_payload.get("node_public_payload_equivalence_status") == "PASS", failures, "public_payload:node_equivalence")
        require("RDF(right,down,forward)" in str(public_payload.get("input_coordinate_contract", "")), failures, "public_payload:rdf_contract")
        require("[forward_m,right_m,down_m,range_m]" in str(public_payload.get("public_topic_contract", "")), failures, "public_payload:frd_contract")
        require(len(public_rows) == int_value(public_payload.get("sample_count")), failures, "public_payload_csv:row_count")
        require(public_payload_rows_valid(public_rows), failures, "public_payload_csv:rows_valid")

    policy_selection: dict[str, object] = {}
    if policy_json.is_file():
        policy_selection = load_json(policy_json)
        require(policy_selection.get("status") == "PASS", failures, "policy_selection:status")
        require(policy_selection.get("recommended_method") == candidate.get("method"), failures, "policy_selection:method")
        require(policy_selection.get("recommended_profile_sha256") == candidate.get("active_profile_sha256"), failures, "policy_selection:sha256")
        require(not bool(policy_selection.get("runtime_truth_used", True)), failures, "policy_selection:runtime_truth_used")

    passed = not failures
    return {
        "mode": "deployable_profile_candidate_readiness",
        "status": "PASS" if passed else "FAIL",
        "candidate_export_json": str(candidate_export_json),
        "method": candidate.get("method", ""),
        "recommended_policy": candidate.get("recommended_policy", ""),
        "active_profile": str(active_profile),
        "active_profile_sha256": active_sha,
        "validation_range_rmse_corrected_mean_m": candidate.get("validation_range_rmse_corrected_mean_m", ""),
        "validation_range_rmse_corrected_max_m": candidate.get("validation_range_rmse_corrected_max_m", ""),
        "target_max_abs_error_corrected_max_m": candidate.get("target_max_abs_error_corrected_max_m", ""),
        "candidate_pass_count": candidate.get("candidate_pass_count", ""),
        "candidate_expected_run_count": candidate.get("candidate_expected_run_count", ""),
        "candidate_axis_pass_count": candidate.get("candidate_axis_pass_count", ""),
        "candidate_target_pass_count": candidate.get("candidate_target_pass_count", ""),
        "promotion_check_pass": promotion.get("pass_promotion_check", candidate.get("promotion_check_pass", "")),
        "promotion_sample_apply_count": promotion.get("sample_apply_count", candidate.get("promotion_sample_apply_count", "")),
        "promotion_sample_invalid_count": promotion.get("sample_invalid_count", candidate.get("promotion_sample_invalid_count", "")),
        "public_topic_payload_status": public_payload.get("status", candidate.get("public_topic_payload_status", "")),
        "public_topic_payload_sample_count": public_payload.get("sample_count", candidate.get("public_topic_payload_sample_count", "")),
        "public_topic_payload_node_equivalence_status": public_payload.get(
            "node_public_payload_equivalence_status",
            candidate.get("public_topic_payload_node_equivalence_status", ""),
        ),
        "runtime_truth_used": False,
        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        "runtime_input_contract": (
            "candidate readiness verifies offline policy evidence; runtime profile consumes "
            "stereo-center RDF(right,down,forward) plus measured disparity and publishes FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Deployable StereoDistance active-profile candidate readiness",
        f"status: {payload.get('status', '')}",
        f"candidate_export_json: {payload.get('candidate_export_json', '')}",
        f"recommended_policy: {payload.get('recommended_policy', '')}",
        f"method: {payload.get('method', '')}",
        f"active_profile: {payload.get('active_profile', '')}",
        f"active_profile_sha256: {payload.get('active_profile_sha256', '')}",
        f"validation_range_rmse_corrected_mean_m: {payload.get('validation_range_rmse_corrected_mean_m', '')}",
        f"validation_range_rmse_corrected_max_m: {payload.get('validation_range_rmse_corrected_max_m', '')}",
        f"target_max_abs_error_corrected_max_m: {payload.get('target_max_abs_error_corrected_max_m', '')}",
        f"candidate_pass_count: {payload.get('candidate_pass_count', '')}/{payload.get('candidate_expected_run_count', '')}",
        f"candidate_axis_pass_count: {payload.get('candidate_axis_pass_count', '')}",
        f"candidate_target_pass_count: {payload.get('candidate_target_pass_count', '')}",
        f"promotion_check_pass: {payload.get('promotion_check_pass', '')}",
        f"promotion_sample_apply_count: {payload.get('promotion_sample_apply_count', '')}",
        f"promotion_sample_invalid_count: {payload.get('promotion_sample_invalid_count', '')}",
        f"public_topic_payload_status: {payload.get('public_topic_payload_status', '')}",
        f"public_topic_payload_sample_count: {payload.get('public_topic_payload_sample_count', '')}",
        f"public_topic_payload_node_equivalence_status: {payload.get('public_topic_payload_node_equivalence_status', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', '')}",
        f"axis_contract: {payload.get('axis_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
        f"runtime_input_contract: {payload.get('runtime_input_contract', '')}",
        f"failure_count: {payload.get('failure_count', '')}",
    ]
    failures = payload.get("failures", [])
    if isinstance(failures, list):
        for failure in failures:
            lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    result = check_candidate_readiness(args.candidate_export_json)
    if args.output_json is not None:
        write_json(args.output_json, result)
    if args.output_txt is not None:
        write_text(args.output_txt, result)
    if args.output_csv is not None:
        write_csv(args.output_csv, [result])
    return result


def main() -> int:
    result = run()
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
