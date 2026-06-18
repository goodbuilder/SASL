#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


REQUIRED_FORBIDDEN_TOKENS = ("truth", "px4", "rtk")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate a StereoDistance active profile deployment manifest and its referenced artifacts."
    )
    parser.add_argument("--manifest-json", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def path_from_manifest(manifest_path: Path, value: object) -> Path:
    path = Path(str(value))
    if path.is_absolute():
        return path
    direct = path
    if direct.exists():
        return direct
    return manifest_path.parent / path


def has_forbidden_runtime_token(values: object) -> bool:
    if not isinstance(values, list):
        return True
    joined = " ".join(str(value).lower() for value in values)
    return any(token in joined for token in ("truth", "gt_", "px4", "rtk", "ned"))


def forbidden_truth_declared(values: object) -> bool:
    if not isinstance(values, list):
        return False
    joined = " ".join(str(value).lower() for value in values)
    return all(token in joined for token in REQUIRED_FORBIDDEN_TOKENS)


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def final_acceptance_rdf_axis_nonregression_status(final_summary: Mapping[str, object]) -> str:
    matrix_rows = final_summary.get("matrix_rows", [])
    if not isinstance(matrix_rows, list):
        return "MISSING"
    for row in matrix_rows:
        if not isinstance(row, Mapping):
            continue
        if row.get("stage") == "single_run_output_audit" and row.get("item") == "rdf_axis_rmse_nonregression":
            return "PASS" if row.get("status") == "PASS" else "FAIL"
    return "MISSING"


def validate_manifest(manifest_json: Path) -> dict[str, object]:
    manifest = load_json(manifest_json)
    failures: list[str] = []
    require(manifest.get("mode") == "stereodistance_active_profile_deployment_manifest", failures, "manifest:mode")
    require(manifest.get("status") == "PASS", failures, "manifest:status")
    require(not bool(manifest.get("runtime_truth_used", True)), failures, "manifest:runtime_truth_used")

    profile_path = path_from_manifest(manifest_json, manifest.get("active_profile", ""))
    promotion_path = path_from_manifest(manifest_json, manifest.get("promotion_check_json", ""))
    final_summary_path = path_from_manifest(manifest_json, manifest.get("final_acceptance_summary", ""))
    public_payload_json_path = path_from_manifest(manifest_json, manifest.get("public_topic_payload_json", ""))
    public_payload_csv_path = path_from_manifest(manifest_json, manifest.get("public_topic_payload_csv", ""))
    require(profile_path.is_file(), failures, "active_profile:missing")
    require(promotion_path.is_file(), failures, "promotion_check_json:missing")
    require(final_summary_path.is_file(), failures, "final_acceptance_summary:missing")
    public_payload_expected = bool(str(manifest.get("public_topic_payload_status", "")).strip())
    if public_payload_expected:
        require(public_payload_json_path.is_file(), failures, "public_topic_payload_json:missing")
        require(public_payload_csv_path.is_file(), failures, "public_topic_payload_csv:missing")

    profile_payload: dict[str, object] = {}
    promotion: dict[str, object] = {}
    final_summary: dict[str, object] = {}
    final_rdf_axis_nonregression_status = "MISSING"
    profile_sha256 = ""
    if profile_path.is_file():
        profile_sha256 = sha256_file(profile_path)
        require(profile_sha256 == str(manifest.get("active_profile_sha256", "")), failures, "active_profile:sha256_mismatch")
        profile_payload = load_json(profile_path)
        require(profile_payload.get("profile_id", "") == manifest.get("profile_id", ""), failures, "active_profile:profile_id")
        require(profile_payload.get("method", "") == manifest.get("method", ""), failures, "active_profile:method")
        require(profile_payload.get("mode", "") == manifest.get("profile_mode", ""), failures, "active_profile:mode")
        require("RDF(right,down,forward)" in str(profile_payload.get("axis_contract", "")), failures, "active_profile:axis_contract")
        require(not has_forbidden_runtime_token(profile_payload.get("runtime_inputs", [])), failures, "active_profile:runtime_inputs_truth")
        require(
            forbidden_truth_declared(profile_payload.get("forbidden_runtime_inputs", [])),
            failures,
            "active_profile:forbidden_runtime_inputs",
        )
    if promotion_path.is_file():
        promotion = load_json(promotion_path)
        require(bool(promotion.get("pass_promotion_check", False)), failures, "promotion:pass")
        require(bool(promotion.get("node_loader_ok", False)), failures, "promotion:node_loader")
        require(bool(promotion.get("axis_contract_declared", False)), failures, "promotion:axis_contract")
        require(not bool(promotion.get("runtime_truth_used", True)), failures, "promotion:runtime_truth_used")
        require(int(promotion.get("sample_apply_count", 0) or 0) > 0, failures, "promotion:sample_apply_count")
        require(int(promotion.get("sample_invalid_count", 0) or 0) == 0, failures, "promotion:sample_invalid_count")
    if final_summary_path.is_file():
        final_summary = load_json(final_summary_path)
        final_rdf_axis_nonregression_status = final_acceptance_rdf_axis_nonregression_status(final_summary)
        require(bool(final_summary.get("overall_pass", False)), failures, "final_acceptance:overall_pass")
        require(int(final_summary.get("matrix_item_count", 0) or 0) >= 28, failures, "final_acceptance:matrix_item_count")
        require(
            final_rdf_axis_nonregression_status == "PASS",
            failures,
            "final_acceptance:rdf_axis_rmse_nonregression",
        )
        require(
            manifest.get("final_rdf_axis_nonregression_status") == final_rdf_axis_nonregression_status,
            failures,
            "manifest:final_rdf_axis_nonregression_status",
        )
        stage_status = final_summary.get("stage_status", {})
        require(
            isinstance(stage_status, Mapping) and bool(stage_status.get("standalone_profile_promotion", False)),
            failures,
            "final_acceptance:standalone_profile_promotion",
        )
    public_payload: dict[str, object] = {}
    public_payload_sample_count = 0
    if public_payload_json_path.is_file():
        public_payload = load_json(public_payload_json_path)
        public_payload_sample_count = int(public_payload.get("sample_count", 0) or 0)
        require(public_payload.get("status") == "PASS", failures, "public_topic_payload:status")
        require(public_payload_sample_count > 0, failures, "public_topic_payload:sample_count")
        require(int(public_payload.get("invalid_count", 0) or 0) == 0, failures, "public_topic_payload:invalid_count")
        require(
            public_payload.get("node_public_payload_equivalence_status") == "PASS",
            failures,
            "public_topic_payload:node_equivalence",
        )
        require(
            manifest.get("public_topic_payload_node_equivalence_status") == "PASS",
            failures,
            "public_topic_payload_manifest:node_equivalence",
        )
        require(not bool(public_payload.get("runtime_truth_used", True)), failures, "public_topic_payload:runtime_truth_used")
        require("RDF(right,down,forward)" in str(public_payload.get("input_coordinate_contract", "")), failures, "public_topic_payload:rdf")
        require(
            "[forward_m,right_m,down_m,range_m]" in str(public_payload.get("public_topic_contract", "")),
            failures,
            "public_topic_payload:frd_range",
        )
    if public_payload_csv_path.is_file():
        with public_payload_csv_path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        require(len(rows) == public_payload_sample_count, failures, "public_topic_payload_csv:row_count")
        for row in rows:
            require(row.get("data_axis_order") == "forward_m,right_m,down_m,range_m", failures, "public_topic_payload_csv:axis_order")
            require(row.get("runtime_truth_used") == "False", failures, "public_topic_payload_csv:runtime_truth_used")
            require(row.get("node_public_payload_equivalent") == "True", failures, "public_topic_payload_csv:node_equivalent")
            try:
                values = json.loads(row.get("data", "[]"))
                finite_values = isinstance(values, list) and len(values) == 4 and all(math.isfinite(float(value)) for value in values)
            except (json.JSONDecodeError, TypeError, ValueError):
                finite_values = False
            require(finite_values, failures, "public_topic_payload_csv:data")

    coordinate_contract = str(manifest.get("coordinate_contract", ""))
    require("RDF(right,down,forward)" in coordinate_contract, failures, "manifest:coordinate_contract_rdf")
    require("FRD+range" in coordinate_contract, failures, "manifest:coordinate_contract_public")
    require("truth is calibration/evaluation only" in coordinate_contract, failures, "manifest:coordinate_contract_truth_scope")

    passed = not failures
    return {
        "status": "PASS" if passed else "FAIL",
        "manifest_json": str(manifest_json),
        "active_profile": str(profile_path),
        "active_profile_sha256_actual": profile_sha256,
        "active_profile_sha256_manifest": manifest.get("active_profile_sha256", ""),
        "promotion_check_json": str(promotion_path),
        "final_acceptance_summary": str(final_summary_path),
        "promotion_sample_apply_count": promotion.get("sample_apply_count", 0),
        "promotion_sample_invalid_count": promotion.get("sample_invalid_count", 0),
        "public_topic_payload_sample_count": public_payload.get("sample_count", 0),
        "public_topic_payload_node_equivalence_status": public_payload.get(
            "node_public_payload_equivalence_status",
            "",
        ),
        "final_matrix_item_count": final_summary.get("matrix_item_count", 0),
        "final_rdf_axis_nonregression_status": final_rdf_axis_nonregression_status,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance active profile deployment manifest validation",
        f"status: {payload.get('status', '')}",
        f"manifest_json: {payload.get('manifest_json', '')}",
        f"active_profile: {payload.get('active_profile', '')}",
        f"active_profile_sha256_actual: {payload.get('active_profile_sha256_actual', '')}",
        f"active_profile_sha256_manifest: {payload.get('active_profile_sha256_manifest', '')}",
        f"promotion_sample_apply_count: {payload.get('promotion_sample_apply_count', 0)}",
        f"promotion_sample_invalid_count: {payload.get('promotion_sample_invalid_count', 0)}",
        f"public_topic_payload_sample_count: {payload.get('public_topic_payload_sample_count', 0)}",
        f"public_topic_payload_node_equivalence_status: {payload.get('public_topic_payload_node_equivalence_status', '')}",
        f"final_matrix_item_count: {payload.get('final_matrix_item_count', 0)}",
        f"final_rdf_axis_nonregression_status: {payload.get('final_rdf_axis_nonregression_status', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    failures = payload.get("failures", [])
    if isinstance(failures, list):
        for failure in failures:
            lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    result = validate_manifest(args.manifest_json)
    if args.output_json is not None:
        write_json(args.output_json, result)
    if args.output_txt is not None:
        write_text(args.output_txt, result)
    return result


def main() -> None:
    result = run()
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    if result["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
