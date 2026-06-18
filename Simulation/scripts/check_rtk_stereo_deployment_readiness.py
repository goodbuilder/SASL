#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_stereodistance_active_profile_deployment_package as package_checker  # noqa: E402
import validate_stereodistance_active_profile_deployment_manifest as manifest_validator  # noqa: E402
import export_stereodistance_deployment_package_inventory as package_inventory_exporter  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check final RTK stereo StereoDistance deployment readiness evidence.")
    parser.add_argument("--wrapper-summary-json", type=Path, required=True)
    parser.add_argument(
        "--matrix-active-profile-gate-json",
        type=Path,
        help=(
            "Optional active-profile common-CSV frontier matrix gate JSON. When provided, deployment readiness "
            "requires the active profile to match the matrix-selected method, pass all replay runs, and remain "
            "runtime truth-free."
        ),
    )
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


def coordinate_contract_ok(text: object) -> bool:
    value = str(text)
    return (
        "RDF(right,down,forward)" in value
        and "FRD+range" in value
        and ("truth is calibration/evaluation only" in value or "truth remains calibration/evaluation only" in value)
    )


def public_frd_contract_ok(text: object) -> bool:
    value = str(text)
    return (
        "FRD+range" in value
        or "FRD_range(forward,right,down,range)" in value
        or "[forward_m,right_m,down_m,range_m]" in value
    )


def rdf_axis_nonregression_status(summary: Mapping[str, object]) -> str:
    direct = str(summary.get("final_rdf_axis_nonregression_status", "") or "")
    if direct:
        return direct
    matrix_rows = summary.get("matrix_rows", [])
    if not isinstance(matrix_rows, list):
        return "MISSING"
    for row in matrix_rows:
        if not isinstance(row, Mapping):
            continue
        if row.get("stage") == "single_run_output_audit" and row.get("item") == "rdf_axis_rmse_nonregression":
            return "PASS" if row.get("status") == "PASS" else "FAIL"
    return "MISSING"


def check_matrix_active_profile_gate(
    *,
    gate_path: Path,
    active_profile_sha: str,
    active_profile_method: str,
    failures: list[str],
) -> dict[str, object]:
    gate: dict[str, object] = {}
    require(gate_path.is_file(), failures, "matrix_active_profile_gate:missing")
    if not gate_path.is_file():
        return gate
    gate = load_json(gate_path)
    require(gate.get("status") == "PASS", failures, "matrix_active_profile_gate:status")
    require(gate.get("recommendation") == "keep_active_profile", failures, "matrix_active_profile_gate:recommendation")
    require(
        bool(gate.get("active_matches_matrix_selected_method", False)),
        failures,
        "matrix_active_profile_gate:active_matches_matrix_selected_method",
    )
    require(bool(gate.get("active_method_pass_all_runs", False)), failures, "matrix_active_profile_gate:active_method_pass_all_runs")
    if bool(gate.get("active_profile_candidate_rows_found", False)):
        require(
            bool(gate.get("active_profile_candidate_pass_all_runs", False)),
            failures,
            "matrix_active_profile_gate:active_profile_candidate_pass_all_runs",
        )
    require(bool(gate.get("active_profile_runtime_truth_free", False)), failures, "matrix_active_profile_gate:runtime_truth_free")
    require(not bool(gate.get("runtime_truth_used", True)), failures, "matrix_active_profile_gate:runtime_truth_used")
    require(gate.get("active_profile_sha256", "") == active_profile_sha, failures, "matrix_active_profile_gate:active_profile_sha256")
    require(
        gate.get("active_profile_method", "") == active_profile_method,
        failures,
        "matrix_active_profile_gate:active_profile_method",
    )
    require(
        gate.get("matrix_selected_method", "") == active_profile_method,
        failures,
        "matrix_active_profile_gate:matrix_selected_method",
    )
    require(
        "RDF(right,down,forward)" in str(gate.get("axis_contract", "")),
        failures,
        "matrix_active_profile_gate:rdf_contract",
    )
    require(
        public_frd_contract_ok(gate.get("public_topic_contract", "")),
        failures,
        "matrix_active_profile_gate:frd_contract",
    )
    require(
        "stereo-center RDF(right,down,forward)" in str(gate.get("runtime_input_contract", "")),
        failures,
        "matrix_active_profile_gate:runtime_input_contract",
    )
    return gate


def check_readiness(wrapper_summary_json: Path, matrix_active_profile_gate_json: Path | None = None) -> dict[str, object]:
    failures: list[str] = []
    summary = load_json(wrapper_summary_json)
    require(summary.get("status") == "PASS", failures, "wrapper_summary:status")
    require(bool(summary.get("final_overall_pass", False)), failures, "wrapper_summary:final_overall_pass")
    require(int(summary.get("final_matrix_item_count", 0) or 0) >= 28, failures, "wrapper_summary:final_matrix_item_count")
    require(bool(summary.get("active_profile_hash_stable", False)), failures, "wrapper_summary:active_profile_hash_stable")
    require(bool(summary.get("promotion_pass", False)), failures, "wrapper_summary:promotion_pass")
    require(int(summary.get("promotion_sample_apply_count", 0) or 0) > 0, failures, "wrapper_summary:promotion_sample_apply_count")
    require(int(summary.get("promotion_sample_invalid_count", 0) or 0) == 0, failures, "wrapper_summary:promotion_sample_invalid_count")
    require(summary.get("deployment_manifest_validation_status") == "PASS", failures, "wrapper_summary:manifest_validation")
    require(summary.get("deployment_package_check_status") == "PASS", failures, "wrapper_summary:package_check")
    require(summary.get("deployment_package_inventory_status") == "PASS", failures, "wrapper_summary:package_inventory")
    require(summary.get("operator_view_launch_check_status") == "PASS", failures, "wrapper_summary:operator_view_launch")
    require(summary.get("install_rehearsal_status") == "PASS", failures, "wrapper_summary:install_rehearsal")
    require(summary.get("public_topic_payload_status") == "PASS", failures, "wrapper_summary:public_topic_payload")
    require(
        summary.get("public_topic_payload_node_equivalence_status") == "PASS",
        failures,
        "wrapper_summary:public_topic_payload_node_equivalence",
    )
    require(not bool(summary.get("runtime_truth_used", True)), failures, "wrapper_summary:runtime_truth_used")
    require(bool(summary.get("coordinate_contracts_explicit", False)), failures, "wrapper_summary:coordinate_contracts")
    final_rdf_axis_nonregression_status = rdf_axis_nonregression_status(summary)
    require(
        final_rdf_axis_nonregression_status == "PASS",
        failures,
        "wrapper_summary:final_rdf_axis_nonregression_status",
    )

    active_profile_path = resolve_path(wrapper_summary_json, summary.get("output_active_profile", ""))
    manifest_path = resolve_path(wrapper_summary_json, summary.get("deployment_manifest_json", ""))
    package_manifest_path = resolve_path(wrapper_summary_json, summary.get("deployment_package_manifest_json", ""))
    package_dir = resolve_path(wrapper_summary_json, summary.get("deployment_package_dir", ""))
    package_check_path = resolve_path(wrapper_summary_json, summary.get("deployment_package_check_json", ""))
    package_inventory_path = resolve_path(wrapper_summary_json, summary.get("deployment_package_inventory_json", ""))
    operator_launch_path = resolve_path(wrapper_summary_json, summary.get("operator_view_launch_check_json", ""))
    install_manifest_path = resolve_path(wrapper_summary_json, summary.get("install_rehearsal_json", ""))
    install_profile_path = resolve_path(wrapper_summary_json, summary.get("install_rehearsal_profile", ""))
    public_payload_json_path = resolve_path(wrapper_summary_json, summary.get("public_topic_payload_json", ""))
    public_payload_csv_path = resolve_path(wrapper_summary_json, summary.get("public_topic_payload_csv", ""))
    operator_selector_status = str(summary.get("operator_view_selector_launch_check_status", "") or "")
    operator_selector_path_value = str(summary.get("operator_view_selector_launch_check_json", "") or "")
    operator_selector_path = resolve_path(wrapper_summary_json, operator_selector_path_value) if operator_selector_path_value else Path("")
    frontier_status = str(summary.get("runtime_deployment_frontier_status", "") or "")
    frontier_json_value = str(summary.get("runtime_deployment_frontier_json", "") or "")
    frontier_labels_csv_value = str(summary.get("runtime_deployment_frontier_labels_csv", "") or "")
    frontier_profiles_csv_value = str(summary.get("runtime_deployment_frontier_profiles_csv", "") or "")
    frontier_json_path = resolve_path(wrapper_summary_json, frontier_json_value) if frontier_json_value else Path("")
    frontier_labels_csv_path = resolve_path(wrapper_summary_json, frontier_labels_csv_value) if frontier_labels_csv_value else Path("")
    frontier_profiles_csv_path = resolve_path(wrapper_summary_json, frontier_profiles_csv_value) if frontier_profiles_csv_value else Path("")
    portability_status = str(summary.get("deployment_package_portability_status", "") or "")
    portability_json_value = str(summary.get("deployment_package_portability_json", "") or "")
    portability_json_path = resolve_path(wrapper_summary_json, portability_json_value) if portability_json_value else Path("")
    require(active_profile_path.is_file(), failures, "active_profile:missing")
    require(manifest_path.is_file(), failures, "deployment_manifest:missing")
    require(package_manifest_path.is_file(), failures, "deployment_package_manifest:missing")
    require(package_dir.is_dir(), failures, "deployment_package:missing")
    require(package_check_path.is_file(), failures, "deployment_package_check:missing")
    require(package_inventory_path.is_file(), failures, "deployment_package_inventory:missing")
    require(operator_launch_path.is_file(), failures, "operator_view_launch_check:missing")
    require(install_manifest_path.is_file(), failures, "install_manifest:missing")
    require(install_profile_path.is_file(), failures, "install_profile:missing")
    require(public_payload_json_path.is_file(), failures, "public_topic_payload_json:missing")
    require(public_payload_csv_path.is_file(), failures, "public_topic_payload_csv:missing")
    if operator_selector_status or operator_selector_path_value:
        require(operator_selector_status == "PASS", failures, "wrapper_summary:operator_view_selector_launch")
        require(operator_selector_path.is_file(), failures, "operator_view_selector_launch_check:missing")
    if frontier_status or frontier_json_value:
        require(frontier_status == "PASS", failures, "wrapper_summary:runtime_deployment_frontier")
        require(frontier_json_path.is_file(), failures, "runtime_deployment_frontier:missing")
        require(frontier_labels_csv_path.is_file(), failures, "runtime_deployment_frontier_labels:missing")
        require(frontier_profiles_csv_path.is_file(), failures, "runtime_deployment_frontier_profiles:missing")
    if portability_status or portability_json_value:
        require(portability_status == "PASS", failures, "wrapper_summary:deployment_package_portability")
        require(portability_json_path.is_file(), failures, "deployment_package_portability:missing")

    active_profile_sha = sha256_file(active_profile_path) if active_profile_path.is_file() else ""
    summary_sha = str(summary.get("active_profile_sha256_after_final_dashboard", ""))
    require(active_profile_sha == summary_sha, failures, "active_profile:summary_sha256")

    manifest_validation: dict[str, object] = {}
    if manifest_path.is_file():
        manifest_validation = manifest_validator.validate_manifest(manifest_path)
        require(manifest_validation.get("status") == "PASS", failures, "deployment_manifest:validation")

    package_check: dict[str, object] = {}
    if package_dir.is_dir():
        package_check = package_checker.check_package(package_dir)
        require(package_check.get("status") == "PASS", failures, "deployment_package:check")
        require(package_check.get("active_profile_sha256", "") == active_profile_sha, failures, "deployment_package:active_profile_sha256")

    package_manifest: dict[str, object] = {}
    if package_manifest_path.is_file():
        package_manifest = load_json(package_manifest_path)
        require(package_manifest.get("status") == "PASS", failures, "deployment_package_manifest:status")
        require(
            package_manifest.get("final_rdf_axis_nonregression_status") == final_rdf_axis_nonregression_status,
            failures,
            "deployment_package_manifest:final_rdf_axis_nonregression_status",
        )

    package_inventory: dict[str, object] = {}
    if package_inventory_path.is_file():
        package_inventory = package_inventory_exporter.load_inventory(package_inventory_path)
        package_inventory_validation = package_inventory_exporter.validate_inventory(package_dir, package_inventory)
        require(package_inventory_validation.get("status") == "PASS", failures, "deployment_package_inventory:validation")
        require(int(package_inventory.get("file_count", 0) or 0) > 0, failures, "deployment_package_inventory:file_count")

    operator_launch: dict[str, object] = {}
    if operator_launch_path.is_file():
        operator_launch = load_json(operator_launch_path)
        require(operator_launch.get("status") == "PASS", failures, "operator_view_launch:status")
        require(operator_launch.get("package_check_status") == "PASS", failures, "operator_view_launch:package_check")
        require(
            operator_launch.get("resolved_runtime_profile", "") == str((package_dir / "active_runtime_profile.json").resolve()),
            failures,
            "operator_view_launch:profile",
        )
        require(
            operator_launch.get("resolved_runtime_profile_sha256", "") == active_profile_sha,
            failures,
            "operator_view_launch:profile_sha256",
        )
        require(bool(operator_launch.get("bbox_adjustment_arg_present", False)), failures, "operator_view_launch:bbox_adjustment")
        require(not bool(operator_launch.get("runtime_truth_used", True)), failures, "operator_view_launch:runtime_truth_used")

    operator_selector_launch: dict[str, object] = {}
    if operator_selector_path_value and operator_selector_path.is_file():
        operator_selector_launch = load_json(operator_selector_path)
        require(operator_selector_launch.get("status") == "PASS", failures, "operator_view_selector_launch:status")
        require(operator_selector_launch.get("source_mode") == "selector", failures, "operator_view_selector_launch:source_mode")
        require(operator_selector_launch.get("label_selection_status") == "PASS", failures, "operator_view_selector_launch:label_selection")
        require(operator_selector_launch.get("package_check_status") == "PASS", failures, "operator_view_selector_launch:package_check")
        require(bool(operator_selector_launch.get("selected_label", "")), failures, "operator_view_selector_launch:selected_label")
        selected_profile = Path(str(operator_selector_launch.get("resolved_runtime_profile", "")))
        require(selected_profile.is_file(), failures, "operator_view_selector_launch:profile")
        require(
            operator_selector_launch.get("resolved_runtime_profile_sha256", "") == active_profile_sha,
            failures,
            "operator_view_selector_launch:profile_sha256",
        )
        require(bool(operator_selector_launch.get("bbox_adjustment_arg_present", False)), failures, "operator_view_selector_launch:bbox_adjustment")
        require(
            not bool(operator_selector_launch.get("runtime_truth_used", True)),
            failures,
            "operator_view_selector_launch:runtime_truth_used",
        )
        require(
            "RDF(right,down,forward)" in str(operator_selector_launch.get("coordinate_contract", "")),
            failures,
            "operator_view_selector_launch:rdf_contract",
        )

    runtime_frontier: dict[str, object] = {}
    runtime_frontier_summary: dict[str, object] = {}
    if frontier_json_value and frontier_json_path.is_file():
        runtime_frontier = load_json(frontier_json_path)
        raw_summary = runtime_frontier.get("summary", {})
        runtime_frontier_summary = dict(raw_summary) if isinstance(raw_summary, Mapping) else {}
        require(runtime_frontier_summary.get("status") == "PASS", failures, "runtime_deployment_frontier:status")
        require(
            not bool(runtime_frontier_summary.get("runtime_truth_used", True)),
            failures,
            "runtime_deployment_frontier:runtime_truth_used",
        )
        require(
            bool(runtime_frontier_summary.get("truth_free_runtime_selection", False)),
            failures,
            "runtime_deployment_frontier:truth_free_selection",
        )
        require(
            "RDF(right,down,forward)" in str(runtime_frontier_summary.get("axis_contract", "")),
            failures,
            "runtime_deployment_frontier:rdf_contract",
        )
        labels = runtime_frontier.get("deployment_labels", [])
        profiles = runtime_frontier.get("profiles", [])
        require(isinstance(labels, list) and len(labels) > 0, failures, "runtime_deployment_frontier:labels")
        require(isinstance(profiles, list) and len(profiles) > 0, failures, "runtime_deployment_frontier:profiles")
    if frontier_labels_csv_path.is_file():
        with frontier_labels_csv_path.open(newline="", encoding="utf-8") as handle:
            frontier_label_rows = list(csv.DictReader(handle))
        require(len(frontier_label_rows) > 0, failures, "runtime_deployment_frontier_labels:row_count")
        for row in frontier_label_rows:
            require(row.get("status") == "PASS", failures, "runtime_deployment_frontier_labels:status")
            require(row.get("runtime_truth_used") == "False", failures, "runtime_deployment_frontier_labels:runtime_truth_used")
    if frontier_profiles_csv_path.is_file():
        with frontier_profiles_csv_path.open(newline="", encoding="utf-8") as handle:
            frontier_profile_rows = list(csv.DictReader(handle))
        require(len(frontier_profile_rows) > 0, failures, "runtime_deployment_frontier_profiles:row_count")
        for row in frontier_profile_rows:
            require(row.get("runtime_truth_used") == "False", failures, "runtime_deployment_frontier_profiles:runtime_truth_used")
            require(row.get("runtime_inputs_truth_free") == "True", failures, "runtime_deployment_frontier_profiles:truth_free")

    package_portability: dict[str, object] = {}
    if portability_json_value and portability_json_path.is_file():
        package_portability = load_json(portability_json_path)
        require(package_portability.get("status") == "PASS", failures, "deployment_package_portability:status")
        require(
            summary.get("deployment_package_portability_status") == package_portability.get("status"),
            failures,
            "wrapper_summary:deployment_package_portability_status_match",
        )
        require(
            package_portability.get("selected_label") == "packaged_active",
            failures,
            "deployment_package_portability:selected_label",
        )
        require(
            summary.get("deployment_package_portability_selected_label") == package_portability.get("selected_label"),
            failures,
            "wrapper_summary:deployment_package_portability_selected_label_match",
        )
        require(
            package_portability.get("copied_package_check_status") == "PASS",
            failures,
            "deployment_package_portability:copied_package_check",
        )
        require(
            bool(package_portability.get("copied_package_dir_ephemeral", False)),
            failures,
            "deployment_package_portability:copied_package_dir_ephemeral",
        )
        require(
            "temporary directory" in str(package_portability.get("copied_package_dir_note", "")),
            failures,
            "deployment_package_portability:copied_package_dir_note",
        )
        require(
            summary.get("deployment_package_portability_copied_package_check_status")
            == package_portability.get("copied_package_check_status"),
            failures,
            "wrapper_summary:deployment_package_portability_copied_package_check_match",
        )
        require(
            not bool(package_portability.get("runtime_truth_used", True)),
            failures,
            "deployment_package_portability:runtime_truth_used",
        )
        require(
            "RDF(right,down,forward)" in str(package_portability.get("coordinate_contract", "")),
            failures,
            "deployment_package_portability:rdf_contract",
        )
        require("FRD+range" in str(package_portability.get("coordinate_contract", "")), failures, "deployment_package_portability:frd_contract")

    stored_package_check: dict[str, object] = {}
    if package_check_path.is_file():
        stored_package_check = load_json(package_check_path)
        require(stored_package_check.get("status") == "PASS", failures, "deployment_package_check_file:status")
        require(
            stored_package_check.get("active_profile_sha256", "") == package_check.get("active_profile_sha256", ""),
            failures,
            "deployment_package_check_file:sha256",
        )
        require(
            stored_package_check.get("final_rdf_axis_nonregression_status") == final_rdf_axis_nonregression_status,
            failures,
            "deployment_package_check_file:final_rdf_axis_nonregression_status",
        )
        require(
            package_check.get("final_rdf_axis_nonregression_status") == final_rdf_axis_nonregression_status,
            failures,
            "deployment_package:final_rdf_axis_nonregression_status",
        )
        if package_portability:
            require(
                stored_package_check.get("package_portability_status") == package_portability.get("status"),
                failures,
                "deployment_package_check_file:package_portability_status",
            )
            require(
                stored_package_check.get("package_portability_selected_label") == package_portability.get("selected_label"),
                failures,
                "deployment_package_check_file:package_portability_selected_label",
            )
            require(
                stored_package_check.get("package_portability_copied_package_check_status")
                == package_portability.get("copied_package_check_status"),
                failures,
                "deployment_package_check_file:package_portability_copied_package_check",
            )
            require(
                package_check.get("package_portability_status") == package_portability.get("status"),
                failures,
                "deployment_package:package_portability_status",
            )
            require(
                package_check.get("package_portability_selected_label") == package_portability.get("selected_label"),
                failures,
                "deployment_package:package_portability_selected_label",
            )
            require(
                package_check.get("package_portability_copied_package_check_status")
                == package_portability.get("copied_package_check_status"),
                failures,
                "deployment_package:package_portability_copied_package_check",
            )

    install_manifest: dict[str, object] = {}
    if install_manifest_path.is_file():
        install_manifest = load_json(install_manifest_path)
        require(install_manifest.get("status") == "PASS", failures, "install_manifest:status")
        require(not bool(install_manifest.get("runtime_truth_used", True)), failures, "install_manifest:runtime_truth_used")
        require(coordinate_contract_ok(install_manifest.get("coordinate_contract", "")), failures, "install_manifest:coordinate_contract")
        require(install_manifest.get("installed_profile_sha256", "") == active_profile_sha, failures, "install_manifest:sha256")
        require(install_manifest.get("expected_profile_sha256", "") == active_profile_sha, failures, "install_manifest:expected_sha256")
    install_profile_sha = sha256_file(install_profile_path) if install_profile_path.is_file() else ""
    require(install_profile_sha == active_profile_sha, failures, "install_profile:sha256")

    active_profile: dict[str, object] = {}
    if active_profile_path.is_file():
        active_profile = load_json(active_profile_path)
        require("RDF(right,down,forward)" in str(active_profile.get("axis_contract", "")), failures, "active_profile:axis_contract")
        require(not has_runtime_truth(active_profile.get("runtime_inputs", [])), failures, "active_profile:runtime_inputs_truth")

    active_profile_method = str(active_profile.get("method", install_manifest.get("method", "")))
    matrix_active_profile_gate: dict[str, object] = {}
    if matrix_active_profile_gate_json is not None:
        matrix_active_profile_gate = check_matrix_active_profile_gate(
            gate_path=matrix_active_profile_gate_json,
            active_profile_sha=active_profile_sha,
            active_profile_method=active_profile_method,
            failures=failures,
        )

    public_payload: dict[str, object] = {}
    public_payload_rows = 0
    if public_payload_json_path.is_file():
        public_payload = load_json(public_payload_json_path)
        public_payload_rows = int(public_payload.get("sample_count", 0) or 0)
        require(public_payload.get("status") == "PASS", failures, "public_topic_payload:status")
        require(public_payload_rows > 0, failures, "public_topic_payload:sample_count")
        require(int(public_payload.get("invalid_count", 0) or 0) == 0, failures, "public_topic_payload:invalid_count")
        require(
            public_payload.get("node_public_payload_equivalence_status") == "PASS",
            failures,
            "public_topic_payload:node_equivalence",
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
        require(len(rows) == public_payload_rows, failures, "public_topic_payload_csv:row_count")
        for row in rows:
            require(row.get("data_axis_order") == "forward_m,right_m,down_m,range_m", failures, "public_topic_payload_csv:axis_order")
            require(row.get("runtime_truth_used") == "False", failures, "public_topic_payload_csv:runtime_truth_used")
            require(row.get("node_public_payload_equivalent") == "True", failures, "public_topic_payload_csv:node_equivalent")
            try:
                values = json.loads(row.get("data", "[]"))
            except json.JSONDecodeError:
                values = []
            require(isinstance(values, list) and len(values) == 4, failures, "public_topic_payload_csv:data_len")
            try:
                finite_values = all(math.isfinite(float(value)) for value in values)
            except (TypeError, ValueError):
                finite_values = False
            require(finite_values, failures, "public_topic_payload_csv:data_finite")

    passed = not failures
    return {
        "mode": "rtk_stereo_stereodistance_deployment_readiness",
        "status": "PASS" if passed else "FAIL",
        "wrapper_summary_json": str(wrapper_summary_json),
        "matrix_active_profile_gate_json": str(matrix_active_profile_gate_json or ""),
        "matrix_active_profile_gate_status": matrix_active_profile_gate.get("status", ""),
        "matrix_active_profile_gate_recommendation": matrix_active_profile_gate.get("recommendation", ""),
        "matrix_active_profile_selected_method": matrix_active_profile_gate.get("matrix_selected_method", ""),
        "matrix_active_profile_active_method": matrix_active_profile_gate.get("active_profile_method", ""),
        "matrix_active_profile_pass_count": matrix_active_profile_gate.get("active_method_pass_count", ""),
        "matrix_active_profile_expected_run_count": matrix_active_profile_gate.get("active_method_expected_run_count", ""),
        "matrix_active_profile_candidate_pass_count": matrix_active_profile_gate.get("active_profile_candidate_pass_count", ""),
        "matrix_active_profile_candidate_expected_run_count": matrix_active_profile_gate.get(
            "active_profile_candidate_expected_run_count",
            "",
        ),
        "matrix_active_profile_candidate_pass_all_runs": matrix_active_profile_gate.get(
            "active_profile_candidate_pass_all_runs",
            "",
        ),
        "matrix_active_profile_rmse_mean_m": matrix_active_profile_gate.get(
            "active_method_validation_range_rmse_corrected_mean_m",
            "",
        ),
        "matrix_active_profile_rmse_max_m": matrix_active_profile_gate.get(
            "active_method_validation_range_rmse_corrected_max_m",
            "",
        ),
        "matrix_active_profile_target_max_abs_m": matrix_active_profile_gate.get(
            "active_method_target_max_abs_error_corrected_max_m",
            "",
        ),
        "active_profile": str(active_profile_path),
        "active_profile_sha256": active_profile_sha,
        "method": active_profile_method,
        "profile_mode": active_profile.get("mode", install_manifest.get("profile_mode", "")),
        "deployment_manifest_validation_status": manifest_validation.get("status", ""),
        "deployment_package_check_status": package_check.get("status", ""),
        "deployment_package_inventory_status": package_inventory.get("status", ""),
        "deployment_package_inventory_file_count": package_inventory.get("file_count", 0),
        "operator_view_launch_check_status": operator_launch.get("status", ""),
        "operator_view_selector_launch_check_status": operator_selector_launch.get("status", ""),
        "operator_view_selector_selected_label": operator_selector_launch.get("selected_label", ""),
        "operator_view_selector_resolved_profile": operator_selector_launch.get("resolved_runtime_profile", ""),
        "runtime_deployment_frontier_status": runtime_frontier_summary.get("status", ""),
        "runtime_deployment_frontier_profile_count": runtime_frontier_summary.get("profile_count", 0),
        "runtime_deployment_frontier_label_count": runtime_frontier_summary.get("deployment_label_count", 0),
        "deployment_package_portability_status": package_portability.get("status", ""),
        "deployment_package_portability_selected_label": package_portability.get("selected_label", ""),
        "deployment_package_portability_copied_package_check_status": package_portability.get("copied_package_check_status", ""),
        "install_rehearsal_status": install_manifest.get("status", ""),
        "public_topic_payload_status": public_payload.get("status", ""),
        "public_topic_payload_sample_count": public_payload.get("sample_count", 0),
        "public_topic_payload_node_equivalence_status": public_payload.get(
            "node_public_payload_equivalence_status",
            "",
        ),
        "promotion_sample_apply_count": summary.get("promotion_sample_apply_count", 0),
        "promotion_sample_invalid_count": summary.get("promotion_sample_invalid_count", 0),
        "final_matrix_item_count": summary.get("final_matrix_item_count", 0),
        "final_rdf_axis_nonregression_status": final_rdf_axis_nonregression_status,
        "runtime_truth_used": False,
        "coordinate_contract": (
            "calibration/evaluation truth may be PX4 local NED; runtime active profile consumes "
            "stereo-center RDF(right,down,forward) plus measured disparity; public StereoDistance output uses FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "RTK stereo StereoDistance deployment readiness",
        f"status: {payload.get('status', '')}",
        f"wrapper_summary_json: {payload.get('wrapper_summary_json', '')}",
        f"matrix_active_profile_gate_json: {payload.get('matrix_active_profile_gate_json', '')}",
        f"matrix_active_profile_gate_status: {payload.get('matrix_active_profile_gate_status', '')}",
        f"matrix_active_profile_gate_recommendation: {payload.get('matrix_active_profile_gate_recommendation', '')}",
        f"matrix_active_profile_selected_method: {payload.get('matrix_active_profile_selected_method', '')}",
        f"matrix_active_profile_active_method: {payload.get('matrix_active_profile_active_method', '')}",
        f"matrix_active_profile_pass_count: {payload.get('matrix_active_profile_pass_count', '')}/{payload.get('matrix_active_profile_expected_run_count', '')}",
        f"matrix_active_profile_candidate_pass_count: {payload.get('matrix_active_profile_candidate_pass_count', '')}/{payload.get('matrix_active_profile_candidate_expected_run_count', '')}",
        f"matrix_active_profile_candidate_pass_all_runs: {payload.get('matrix_active_profile_candidate_pass_all_runs', '')}",
        f"matrix_active_profile_rmse_mean_m: {payload.get('matrix_active_profile_rmse_mean_m', '')}",
        f"matrix_active_profile_rmse_max_m: {payload.get('matrix_active_profile_rmse_max_m', '')}",
        f"matrix_active_profile_target_max_abs_m: {payload.get('matrix_active_profile_target_max_abs_m', '')}",
        f"active_profile: {payload.get('active_profile', '')}",
        f"active_profile_sha256: {payload.get('active_profile_sha256', '')}",
        f"method: {payload.get('method', '')}",
        f"profile_mode: {payload.get('profile_mode', '')}",
        f"deployment_manifest_validation_status: {payload.get('deployment_manifest_validation_status', '')}",
        f"deployment_package_check_status: {payload.get('deployment_package_check_status', '')}",
        f"deployment_package_inventory_status: {payload.get('deployment_package_inventory_status', '')}",
        f"deployment_package_inventory_file_count: {payload.get('deployment_package_inventory_file_count', 0)}",
        f"operator_view_launch_check_status: {payload.get('operator_view_launch_check_status', '')}",
        f"operator_view_selector_launch_check_status: {payload.get('operator_view_selector_launch_check_status', '')}",
        f"operator_view_selector_selected_label: {payload.get('operator_view_selector_selected_label', '')}",
        f"operator_view_selector_resolved_profile: {payload.get('operator_view_selector_resolved_profile', '')}",
        f"runtime_deployment_frontier_status: {payload.get('runtime_deployment_frontier_status', '')}",
        f"runtime_deployment_frontier_profile_count: {payload.get('runtime_deployment_frontier_profile_count', 0)}",
        f"runtime_deployment_frontier_label_count: {payload.get('runtime_deployment_frontier_label_count', 0)}",
        f"deployment_package_portability_status: {payload.get('deployment_package_portability_status', '')}",
        f"deployment_package_portability_selected_label: {payload.get('deployment_package_portability_selected_label', '')}",
        f"deployment_package_portability_copied_package_check_status: {payload.get('deployment_package_portability_copied_package_check_status', '')}",
        f"install_rehearsal_status: {payload.get('install_rehearsal_status', '')}",
        f"public_topic_payload_status: {payload.get('public_topic_payload_status', '')}",
        f"public_topic_payload_sample_count: {payload.get('public_topic_payload_sample_count', 0)}",
        f"public_topic_payload_node_equivalence_status: {payload.get('public_topic_payload_node_equivalence_status', '')}",
        f"promotion_sample_apply_count: {payload.get('promotion_sample_apply_count', 0)}",
        f"promotion_sample_invalid_count: {payload.get('promotion_sample_invalid_count', 0)}",
        f"final_matrix_item_count: {payload.get('final_matrix_item_count', 0)}",
        f"final_rdf_axis_nonregression_status: {payload.get('final_rdf_axis_nonregression_status', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"coordinate_contract: {payload.get('coordinate_contract', '')}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_csv(path: Path, payload: Mapping[str, object]) -> None:
    fields = [
        "status",
        "method",
        "profile_mode",
        "active_profile",
        "active_profile_sha256",
        "matrix_active_profile_gate_json",
        "matrix_active_profile_gate_status",
        "matrix_active_profile_gate_recommendation",
        "matrix_active_profile_selected_method",
        "matrix_active_profile_active_method",
        "matrix_active_profile_pass_count",
        "matrix_active_profile_expected_run_count",
        "matrix_active_profile_candidate_pass_count",
        "matrix_active_profile_candidate_expected_run_count",
        "matrix_active_profile_candidate_pass_all_runs",
        "matrix_active_profile_rmse_mean_m",
        "matrix_active_profile_rmse_max_m",
        "matrix_active_profile_target_max_abs_m",
        "deployment_manifest_validation_status",
        "deployment_package_check_status",
        "deployment_package_inventory_status",
        "deployment_package_inventory_file_count",
        "operator_view_launch_check_status",
        "operator_view_selector_launch_check_status",
        "operator_view_selector_selected_label",
        "operator_view_selector_resolved_profile",
        "runtime_deployment_frontier_status",
        "runtime_deployment_frontier_profile_count",
        "runtime_deployment_frontier_label_count",
        "deployment_package_portability_status",
        "deployment_package_portability_selected_label",
        "deployment_package_portability_copied_package_check_status",
        "install_rehearsal_status",
        "public_topic_payload_status",
        "public_topic_payload_sample_count",
        "public_topic_payload_node_equivalence_status",
        "promotion_sample_apply_count",
        "promotion_sample_invalid_count",
        "final_matrix_item_count",
        "final_rdf_axis_nonregression_status",
        "runtime_truth_used",
        "failure_count",
        "failures",
        "coordinate_contract",
    ]
    row: dict[str, object] = {}
    for field in fields:
        value = payload.get(field, "")
        if isinstance(value, list):
            value = ";".join(str(item) for item in value)
        row[field] = value
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerow(row)


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    payload = check_readiness(args.wrapper_summary_json, args.matrix_active_profile_gate_json)
    if args.output_json:
        write_json(args.output_json, payload)
    if args.output_txt:
        write_text(args.output_txt, payload)
    if args.output_csv:
        write_csv(args.output_csv, payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
