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

import stereodistance_profile_promotion_check as promotion_check  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check a StereoDistance active profile deployment package.")
    parser.add_argument("--package-dir", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
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


def has_forbidden_runtime_token(values: object) -> bool:
    if not isinstance(values, list):
        return True
    joined = " ".join(str(value).lower() for value in values)
    return any(token in joined for token in ("truth", "gt_", "px4", "rtk", "ned"))


def forbidden_truth_declared(values: object) -> bool:
    if not isinstance(values, list):
        return False
    joined = " ".join(str(value).lower() for value in values)
    return all(token in joined for token in ("truth", "px4", "rtk"))


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def final_acceptance_has_rdf_axis_nonregression(final_summary: Mapping[str, object]) -> bool:
    matrix_rows = final_summary.get("matrix_rows", [])
    if not isinstance(matrix_rows, list):
        return False
    for row in matrix_rows:
        if not isinstance(row, Mapping):
            continue
        if (
            row.get("stage") == "single_run_output_audit"
            and row.get("item") == "rdf_axis_rmse_nonregression"
            and row.get("status") == "PASS"
        ):
            return True
    return False


def runtime_apply_smoke(profile: object) -> tuple[bool, str]:
    raw = {
        "right_m": 4.0,
        "down_m": 1.5,
        "forward_m": 800.0,
        "range_m": math.sqrt(4.0 * 4.0 + 1.5 * 1.5 + 800.0 * 800.0),
        "disparity_px": 25.0,
        "raw_disparity_px": 25.0,
        "left_u_px": 640.0,
        "left_v_px": 360.0,
    }
    try:
        result = profile.apply(raw)  # type: ignore[attr-defined]
    except Exception as exc:
        return False, f"exception:{exc}"
    corrected = result.corrected
    if not isinstance(corrected, Mapping):
        return False, "corrected_not_mapping"
    for field in ("right_m", "down_m", "forward_m", "range_m", "disparity_px"):
        try:
            value = float(corrected[field])
        except (KeyError, TypeError, ValueError):
            return False, f"missing_or_invalid:{field}"
        if not math.isfinite(value):
            return False, f"nonfinite:{field}"
    return True, str(getattr(profile, "mode", "runtime_profile"))


def role_paths(package_dir: Path, checked_files: Sequence[Mapping[str, object]]) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    for row in checked_files:
        role = str(row.get("role", ""))
        rel_path = str(row.get("path", ""))
        if role and rel_path:
            paths[role] = package_dir / rel_path
    return paths


def check_package(package_dir: Path, *, validate_portability_evidence: bool = True) -> dict[str, object]:
    manifest_path = package_dir / "deployment_package_manifest.json"
    failures: list[str] = []
    require(manifest_path.is_file(), failures, "package_manifest:missing")
    if not manifest_path.is_file():
        return {
            "status": "FAIL",
            "package_dir": str(package_dir),
            "failure_count": len(failures),
            "failures": failures,
            "runtime_truth_used": False,
        }

    manifest = load_json(manifest_path)
    require(manifest.get("mode") == "stereodistance_active_profile_deployment_package", failures, "package_manifest:mode")
    require(manifest.get("status") == "PASS", failures, "package_manifest:status")
    require(not bool(manifest.get("runtime_truth_used", True)), failures, "package_manifest:runtime_truth_used")
    coordinate_contract = str(manifest.get("coordinate_contract", ""))
    require("RDF(right,down,forward)" in coordinate_contract, failures, "coordinate_contract:rdf")
    require("FRD+range" in coordinate_contract, failures, "coordinate_contract:public")
    require("truth is calibration/evaluation only" in coordinate_contract, failures, "coordinate_contract:truth_scope")
    require(not has_forbidden_runtime_token(manifest.get("runtime_inputs", [])), failures, "runtime_inputs:truth")
    require(forbidden_truth_declared(manifest.get("forbidden_runtime_inputs", [])), failures, "forbidden_runtime_inputs:truth")
    public_payload_expected = bool(str(manifest.get("public_topic_payload_status", "")).strip())
    if public_payload_expected:
        require(manifest.get("public_topic_payload_status") == "PASS", failures, "public_topic_payload_manifest:status")
        require(int(manifest.get("public_topic_payload_sample_count", 0) or 0) > 0, failures, "public_topic_payload_manifest:sample_count")
        require(
            manifest.get("public_topic_payload_node_equivalence_status") == "PASS",
            failures,
            "public_topic_payload_manifest:node_equivalence",
        )

    file_rows = manifest.get("files", [])
    require(isinstance(file_rows, list), failures, "files:not_list")
    checked_files: list[dict[str, object]] = []
    if isinstance(file_rows, list):
        require(int(manifest.get("file_count", 0) or 0) == len(file_rows), failures, "file_count:mismatch")
        for row in file_rows:
            if not isinstance(row, Mapping):
                failures.append("files:row_not_mapping")
                continue
            rel_path = Path(str(row.get("path", "")))
            if rel_path.is_absolute() or ".." in rel_path.parts:
                failures.append(f"files:{rel_path}:unsafe_path")
                continue
            file_path = package_dir / rel_path
            role = str(row.get("role", ""))
            exists = file_path.is_file()
            require(exists, failures, f"files:{role}:missing")
            actual_sha = ""
            actual_size = 0
            if exists:
                actual_sha = sha256_file(file_path)
                actual_size = file_path.stat().st_size
                require(actual_sha == str(row.get("sha256", "")), failures, f"files:{role}:sha256")
                require(actual_size == int(row.get("size_bytes", -1) or -1), failures, f"files:{role}:size")
            checked_files.append(
                {
                    "role": role,
                    "path": str(rel_path),
                    "exists": exists,
                    "sha256": actual_sha,
                    "size_bytes": actual_size,
                }
            )

    active_profile_path = package_dir / str(manifest.get("active_profile", ""))
    require(active_profile_path.is_file(), failures, "active_profile:missing")
    active_profile_sha = sha256_file(active_profile_path) if active_profile_path.is_file() else ""
    runtime_profile_load_status = "not_run"
    runtime_profile_apply_status = "not_run"
    node_loader_status = "not_run"
    require(active_profile_sha == str(manifest.get("active_profile_sha256", "")), failures, "active_profile:sha256")
    if active_profile_path.is_file():
        profile = load_json(active_profile_path)
        require(profile.get("profile_id", "") == manifest.get("profile_id", ""), failures, "active_profile:profile_id")
        require(profile.get("method", "") == manifest.get("method", ""), failures, "active_profile:method")
        require(profile.get("mode", "") == manifest.get("profile_mode", ""), failures, "active_profile:mode")
        require("RDF(right,down,forward)" in str(profile.get("axis_contract", "")), failures, "active_profile:axis_contract")
        require(not has_forbidden_runtime_token(profile.get("runtime_inputs", [])), failures, "active_profile:runtime_inputs_truth")
        try:
            runtime_profile = promotion_check.load_runtime_profile(profile)
            runtime_profile_load_status = "PASS"
        except Exception as exc:
            runtime_profile = None
            runtime_profile_load_status = f"FAIL:{exc}"
        require(runtime_profile_load_status == "PASS", failures, "active_profile:runtime_profile_load")
        if runtime_profile is not None:
            apply_ok, runtime_profile_apply_status = runtime_apply_smoke(runtime_profile)
            require(apply_ok, failures, f"active_profile:runtime_profile_apply:{runtime_profile_apply_status}")
        node_loader_ok, node_loader_log = promotion_check.node_loader_check(active_profile_path)
        node_loader_status = "PASS" if node_loader_ok else f"FAIL:{node_loader_log}"
        require(node_loader_ok, failures, "active_profile:node_loader")

    paths_by_role = role_paths(package_dir, checked_files)
    promotion_sample_apply_count = 0
    promotion_sample_invalid_count = 0
    final_matrix_item_count = 0
    for required_role in (
        "promotion_check",
        "final_acceptance_summary",
        "deployment_manifest",
        "deployment_manifest_validation",
        "deployment_readme",
    ):
        require(required_role in paths_by_role, failures, f"files:{required_role}:not_recorded")
    if public_payload_expected:
        for required_role in ("public_topic_payload_summary", "public_topic_payload_csv"):
            require(required_role in paths_by_role, failures, f"files:{required_role}:not_recorded")
    runtime_frontier_expected = bool(str(manifest.get("runtime_deployment_frontier_status", "")).strip())
    if runtime_frontier_expected:
        require(manifest.get("runtime_deployment_frontier_status") == "PASS", failures, "runtime_deployment_frontier_manifest:status")
        for required_role in (
            "runtime_profile_label_selector",
            "package_runtime_profile_label_selector",
            "runtime_deployment_frontier_json",
            "runtime_deployment_frontier_labels_csv",
            "runtime_deployment_frontier_profiles_csv",
        ):
            require(required_role in paths_by_role, failures, f"files:{required_role}:not_recorded")
    promotion_path = paths_by_role.get("promotion_check")
    if promotion_path is not None and promotion_path.is_file():
        promotion = load_json(promotion_path)
        promotion_sample_apply_count = int(promotion.get("sample_apply_count", 0) or 0)
        promotion_sample_invalid_count = int(promotion.get("sample_invalid_count", 0) or 0)
        require(bool(promotion.get("pass_promotion_check", False)), failures, "promotion:pass")
        require(not bool(promotion.get("runtime_truth_used", True)), failures, "promotion:runtime_truth_used")
        require(promotion_sample_apply_count > 0, failures, "promotion:sample_apply_count")
        require(promotion_sample_invalid_count == 0, failures, "promotion:sample_invalid_count")
    final_rdf_axis_nonregression_status = "MISSING"
    final_path = paths_by_role.get("final_acceptance_summary")
    if final_path is not None and final_path.is_file():
        final_summary = load_json(final_path)
        final_matrix_item_count = int(final_summary.get("matrix_item_count", 0) or 0)
        final_rdf_axis_nonregression_status = (
            "PASS" if final_acceptance_has_rdf_axis_nonregression(final_summary) else "FAIL"
        )
        require(
            manifest.get("final_rdf_axis_nonregression_status") == final_rdf_axis_nonregression_status,
            failures,
            "package_manifest:final_rdf_axis_nonregression_status",
        )
        require(bool(final_summary.get("overall_pass", False)), failures, "final_acceptance:overall_pass")
        require(final_matrix_item_count >= 28, failures, "final_acceptance:matrix_item_count")
        require(not bool(final_summary.get("runtime_truth_used", True)), failures, "final_acceptance:runtime_truth_used")
        require(final_rdf_axis_nonregression_status == "PASS", failures, "final_acceptance:rdf_axis_rmse_nonregression")
        stage_status = final_summary.get("stage_status", {})
        require(
            isinstance(stage_status, Mapping) and bool(stage_status.get("standalone_profile_promotion", False)),
            failures,
            "final_acceptance:standalone_profile_promotion",
        )
    public_payload_sample_count = 0
    public_payload_summary_path = paths_by_role.get("public_topic_payload_summary")
    if public_payload_summary_path is not None and public_payload_summary_path.is_file():
        public_payload = load_json(public_payload_summary_path)
        public_payload_sample_count = int(public_payload.get("sample_count", 0) or 0)
        require(public_payload.get("status") == "PASS", failures, "public_topic_payload:status")
        require(public_payload_sample_count > 0, failures, "public_topic_payload:sample_count")
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
    public_payload_csv_path = paths_by_role.get("public_topic_payload_csv")
    if public_payload_csv_path is not None and public_payload_csv_path.is_file():
        with public_payload_csv_path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        require(len(rows) == public_payload_sample_count, failures, "public_topic_payload_csv:row_count")
        for row in rows:
            require(row.get("topic") == "/stereo/corrected_position", failures, "public_topic_payload_csv:topic")
            require(row.get("data_axis_order") == "forward_m,right_m,down_m,range_m", failures, "public_topic_payload_csv:axis_order")
            require(row.get("runtime_truth_used") == "False", failures, "public_topic_payload_csv:runtime_truth_used")
            require(row.get("node_public_payload_equivalent") == "True", failures, "public_topic_payload_csv:node_equivalent")
            try:
                values = json.loads(row.get("data", "[]"))
                finite_values = isinstance(values, list) and len(values) == 4 and all(math.isfinite(float(value)) for value in values)
            except (json.JSONDecodeError, TypeError, ValueError):
                finite_values = False
            require(finite_values, failures, "public_topic_payload_csv:data")
    runtime_frontier_profile_count = 0
    runtime_frontier_label_count = 0
    package_runtime_selector_supported_label_count = 0
    package_portability_status = ""
    package_portability_selected_label = ""
    package_portability_copied_package_check_status = ""
    selector_path = paths_by_role.get("runtime_profile_label_selector")
    if selector_path is not None and selector_path.is_file():
        selector = load_json(selector_path)
        require(selector.get("mode") == "stereodistance_runtime_profile_label_selector", failures, "runtime_selector:mode")
        require("RDF(right,down,forward)" in str(selector.get("axis_contract", "")), failures, "runtime_selector:rdf_contract")
        require("FRD" in str(selector.get("public_topic_contract", "")), failures, "runtime_selector:public_contract")
        require(not has_forbidden_runtime_token(selector.get("selection_input_fields", [])), failures, "runtime_selector:selection_inputs_truth")
    package_selector_path = paths_by_role.get("package_runtime_profile_label_selector")
    if package_selector_path is not None and package_selector_path.is_file():
        package_selector = load_json(package_selector_path)
        require(
            package_selector.get("mode") == "stereodistance_runtime_profile_label_selector",
            failures,
            "package_runtime_selector:mode",
        )
        require(
            "RDF(right,down,forward)" in str(package_selector.get("axis_contract", "")),
            failures,
            "package_runtime_selector:rdf_contract",
        )
        require("FRD" in str(package_selector.get("public_topic_contract", "")), failures, "package_runtime_selector:public_contract")
        require(
            not has_forbidden_runtime_token(package_selector.get("selection_input_fields", [])),
            failures,
            "package_runtime_selector:selection_inputs_truth",
        )
        profile_paths = package_selector.get("profile_paths", {})
        package_paths = package_selector.get("package_paths", {})
        label_profile_map = package_selector.get("label_profile_map", {})
        supported_labels = package_selector.get("supported_deployment_labels", [])
        require(package_selector.get("baseline_selector", "") == "packaged_active", failures, "package_runtime_selector:baseline")
        require(isinstance(label_profile_map, Mapping), failures, "package_runtime_selector:label_profile_map")
        require(isinstance(supported_labels, list), failures, "package_runtime_selector:supported_labels")
        if isinstance(label_profile_map, Mapping):
            for label, selected_profile_label in label_profile_map.items():
                require(str(selected_profile_label) == "packaged_active", failures, f"package_runtime_selector:label:{label}")
        if isinstance(supported_labels, list):
            package_runtime_selector_supported_label_count = len(supported_labels)
            if isinstance(label_profile_map, Mapping):
                require(
                    set(str(label) for label in supported_labels) == set(str(label) for label in label_profile_map.keys()),
                    failures,
                    "package_runtime_selector:supported_label_map",
                )
        require(isinstance(profile_paths, Mapping), failures, "package_runtime_selector:profile_paths")
        require(isinstance(package_paths, Mapping), failures, "package_runtime_selector:package_paths")
        if isinstance(profile_paths, Mapping):
            packaged_profile = Path(str(profile_paths.get("packaged_active", "")))
            packaged_profile_path = (package_selector_path.parent / packaged_profile).resolve()
            require(packaged_profile_path == active_profile_path.resolve(), failures, "package_runtime_selector:packaged_profile")
            require(packaged_profile_path.is_file(), failures, "package_runtime_selector:packaged_profile_exists")
        if isinstance(package_paths, Mapping):
            packaged_dir = Path(str(package_paths.get("packaged_active", "")))
            packaged_package_dir = (package_selector_path.parent / packaged_dir).resolve()
            require(packaged_package_dir == package_dir.resolve(), failures, "package_runtime_selector:packaged_package")
            require(packaged_package_dir.is_dir(), failures, "package_runtime_selector:packaged_package_exists")
    frontier_json_path = paths_by_role.get("runtime_deployment_frontier_json")
    if frontier_json_path is not None and frontier_json_path.is_file():
        frontier = load_json(frontier_json_path)
        frontier_summary = frontier.get("summary", {})
        frontier_summary = dict(frontier_summary) if isinstance(frontier_summary, Mapping) else {}
        runtime_frontier_profile_count = int(frontier_summary.get("profile_count", 0) or 0)
        runtime_frontier_label_count = int(frontier_summary.get("deployment_label_count", 0) or 0)
        require(frontier_summary.get("status") == "PASS", failures, "runtime_deployment_frontier:status")
        require(not bool(frontier_summary.get("runtime_truth_used", True)), failures, "runtime_deployment_frontier:runtime_truth_used")
        require(bool(frontier_summary.get("truth_free_runtime_selection", False)), failures, "runtime_deployment_frontier:truth_free_selection")
        require("RDF(right,down,forward)" in str(frontier_summary.get("axis_contract", "")), failures, "runtime_deployment_frontier:rdf_contract")
        require(runtime_frontier_profile_count > 0, failures, "runtime_deployment_frontier:profile_count")
        require(runtime_frontier_label_count > 0, failures, "runtime_deployment_frontier:label_count")
    frontier_labels_csv = paths_by_role.get("runtime_deployment_frontier_labels_csv")
    if frontier_labels_csv is not None and frontier_labels_csv.is_file():
        with frontier_labels_csv.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        require(len(rows) == runtime_frontier_label_count, failures, "runtime_deployment_frontier_labels:row_count")
        for row in rows:
            require(row.get("status") == "PASS", failures, "runtime_deployment_frontier_labels:status")
            require(row.get("runtime_truth_used") == "False", failures, "runtime_deployment_frontier_labels:runtime_truth_used")
    frontier_profiles_csv = paths_by_role.get("runtime_deployment_frontier_profiles_csv")
    if frontier_profiles_csv is not None and frontier_profiles_csv.is_file():
        with frontier_profiles_csv.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        require(len(rows) == runtime_frontier_profile_count, failures, "runtime_deployment_frontier_profiles:row_count")
        for row in rows:
            require(row.get("runtime_truth_used") == "False", failures, "runtime_deployment_frontier_profiles:runtime_truth_used")
            require(row.get("runtime_inputs_truth_free") == "True", failures, "runtime_deployment_frontier_profiles:truth_free")
    portability_path = package_dir / "evidence" / "package_portability_check.json"
    if validate_portability_evidence and portability_path.is_file():
        require("package_portability_check" in paths_by_role, failures, "files:package_portability_check:not_recorded")
        portability = load_json(portability_path)
        package_portability_status = str(portability.get("status", ""))
        package_portability_selected_label = str(portability.get("selected_label", ""))
        package_portability_copied_package_check_status = str(portability.get("copied_package_check_status", ""))
        require(package_portability_status == "PASS", failures, "package_portability:status")
        require(package_portability_selected_label == "packaged_active", failures, "package_portability:selected_label")
        require(package_portability_copied_package_check_status == "PASS", failures, "package_portability:copied_package_check")
        require(bool(portability.get("copied_package_dir_ephemeral", False)), failures, "package_portability:copied_package_dir_ephemeral")
        require("temporary directory" in str(portability.get("copied_package_dir_note", "")), failures, "package_portability:copied_package_dir_note")
        require(manifest.get("package_portability_status") == package_portability_status, failures, "package_portability_manifest:status")
        require(
            manifest.get("package_portability_selected_label") == package_portability_selected_label,
            failures,
            "package_portability_manifest:selected_label",
        )
        require(
            manifest.get("package_portability_copied_package_check_status") == package_portability_copied_package_check_status,
            failures,
            "package_portability_manifest:copied_package_check",
        )
        require(not bool(portability.get("runtime_truth_used", True)), failures, "package_portability:runtime_truth_used")
        require("RDF(right,down,forward)" in str(portability.get("coordinate_contract", "")), failures, "package_portability:rdf_contract")
        require("FRD+range" in str(portability.get("coordinate_contract", "")), failures, "package_portability:frd_contract")
    deployment_manifest_path = paths_by_role.get("deployment_manifest")
    if deployment_manifest_path is not None and deployment_manifest_path.is_file():
        deployment_manifest = load_json(deployment_manifest_path)
        require(deployment_manifest.get("status") == "PASS", failures, "deployment_manifest:status")
        require(not bool(deployment_manifest.get("runtime_truth_used", True)), failures, "deployment_manifest:runtime_truth_used")
    validation_path = paths_by_role.get("deployment_manifest_validation")
    if validation_path is not None and validation_path.is_file():
        validation = load_json(validation_path)
        require(validation.get("status") == "PASS", failures, "deployment_manifest_validation:status")
        require(int(validation.get("failure_count", 0) or 0) == 0, failures, "deployment_manifest_validation:failure_count")
        require(not bool(validation.get("runtime_truth_used", True)), failures, "deployment_manifest_validation:runtime_truth_used")
    readme_path = paths_by_role.get("deployment_readme")
    if readme_path is not None and readme_path.is_file():
        readme_text = readme_path.read_text(encoding="utf-8")
        require("active_runtime_profile.json" in readme_text, failures, "deployment_readme:active_profile")
        require("RDF" in readme_text and "FRD+range" in readme_text, failures, "deployment_readme:coordinate_contract")
        require("Do not provide PX4/RTK/NED truth" in readme_text, failures, "deployment_readme:truth_scope")
        require("run_operator_view_with_calibration_profile.sh --package-dir" in readme_text, failures, "deployment_readme:operator_wrapper")
        require("run_operator_view.sh --bbox-adjustment" in readme_text, failures, "deployment_readme:bbox_adjustment")

    passed = not failures
    return {
        "status": "PASS" if passed else "FAIL",
        "package_dir": str(package_dir),
        "manifest": str(manifest_path),
        "active_profile": str(active_profile_path),
        "active_profile_sha256": active_profile_sha,
        "runtime_profile_load_status": runtime_profile_load_status,
        "runtime_profile_apply_status": runtime_profile_apply_status,
        "node_loader_status": node_loader_status,
        "promotion_sample_apply_count": promotion_sample_apply_count,
        "promotion_sample_invalid_count": promotion_sample_invalid_count,
        "public_topic_payload_sample_count": public_payload_sample_count,
        "public_topic_payload_node_equivalence_status": public_payload.get(
            "node_public_payload_equivalence_status",
            "",
        )
        if public_payload_summary_path is not None and public_payload_summary_path.is_file()
        else "",
        "runtime_deployment_frontier_profile_count": runtime_frontier_profile_count,
        "runtime_deployment_frontier_label_count": runtime_frontier_label_count,
        "package_runtime_selector_supported_label_count": package_runtime_selector_supported_label_count,
        "package_portability_status": package_portability_status,
        "package_portability_selected_label": package_portability_selected_label,
        "package_portability_copied_package_check_status": package_portability_copied_package_check_status,
        "final_matrix_item_count": final_matrix_item_count,
        "final_rdf_axis_nonregression_status": final_rdf_axis_nonregression_status,
        "file_count": len(checked_files),
        "checked_files": checked_files,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance active profile deployment package check",
        f"status: {payload.get('status', '')}",
        f"package_dir: {payload.get('package_dir', '')}",
        f"active_profile: {payload.get('active_profile', '')}",
        f"active_profile_sha256: {payload.get('active_profile_sha256', '')}",
        f"runtime_profile_load_status: {payload.get('runtime_profile_load_status', '')}",
        f"runtime_profile_apply_status: {payload.get('runtime_profile_apply_status', '')}",
        f"node_loader_status: {payload.get('node_loader_status', '')}",
        f"promotion_sample_apply_count: {payload.get('promotion_sample_apply_count', 0)}",
        f"promotion_sample_invalid_count: {payload.get('promotion_sample_invalid_count', 0)}",
        f"public_topic_payload_sample_count: {payload.get('public_topic_payload_sample_count', 0)}",
        f"public_topic_payload_node_equivalence_status: {payload.get('public_topic_payload_node_equivalence_status', '')}",
        f"runtime_deployment_frontier_profile_count: {payload.get('runtime_deployment_frontier_profile_count', 0)}",
        f"runtime_deployment_frontier_label_count: {payload.get('runtime_deployment_frontier_label_count', 0)}",
        f"package_runtime_selector_supported_label_count: {payload.get('package_runtime_selector_supported_label_count', 0)}",
        f"package_portability_status: {payload.get('package_portability_status', '')}",
        f"package_portability_selected_label: {payload.get('package_portability_selected_label', '')}",
        f"package_portability_copied_package_check_status: {payload.get('package_portability_copied_package_check_status', '')}",
        f"final_matrix_item_count: {payload.get('final_matrix_item_count', 0)}",
        f"final_rdf_axis_nonregression_status: {payload.get('final_rdf_axis_nonregression_status', '')}",
        f"file_count: {payload.get('file_count', 0)}",
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
    result = check_package(args.package_dir)
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
