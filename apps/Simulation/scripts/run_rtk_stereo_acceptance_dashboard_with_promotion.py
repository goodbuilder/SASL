#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import stereodistance_profile_promotion_check as promotion_check  # noqa: E402
import summarize_rtk_stereo_acceptance_matrices as acceptance_summary  # noqa: E402
import validate_stereodistance_active_profile_deployment_manifest as manifest_validator  # noqa: E402
import export_stereodistance_active_profile_deployment_package as deployment_package_exporter  # noqa: E402
import check_stereodistance_active_profile_deployment_package as deployment_package_checker  # noqa: E402
import install_stereodistance_active_profile_package as active_profile_installer  # noqa: E402
import check_rtk_stereo_deployment_readiness as deployment_readiness_checker  # noqa: E402
import export_stereodistance_public_topic_payload_from_profile as public_payload_exporter  # noqa: E402
import export_stereodistance_deployment_package_inventory as package_inventory_exporter  # noqa: E402
import check_stereodistance_operator_view_launch as operator_view_launch_checker  # noqa: E402
import export_stereodistance_runtime_deployment_frontier as runtime_frontier_exporter  # noqa: E402
import check_stereodistance_deployment_package_portability as package_portability_checker  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build the RTK stereo acceptance dashboard end-to-end: export the active profile, "
            "run the StereoDistance promotion check, then fold that check back into the final matrix."
        )
    )
    parser.add_argument("--single-audit-json", type=Path, required=True)
    parser.add_argument("--multi-seed-summary-json", type=Path, required=True)
    parser.add_argument("--robustness-summary-json", type=Path, required=True)
    parser.add_argument("--deployment-strategy-json", type=Path, required=True)
    parser.add_argument("--output-active-profile", type=Path, required=True)
    parser.add_argument("--calibration-samples-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--promotion-output-dir", type=Path)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--max-samples", type=int, default=50)
    parser.add_argument("--scenario")
    parser.add_argument("--catalog-json", type=Path)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument(
        "--runtime-profile-selector-json",
        type=Path,
        help="Optional truth-free runtime profile selector manifest to smoke through the operator wrapper.",
    )
    parser.add_argument(
        "--matrix-active-profile-gate-json",
        type=Path,
        help=(
            "Optional active-profile common-CSV frontier matrix gate JSON to include in deployment readiness. "
            "The gate is offline evaluation evidence only and must not add runtime truth inputs."
        ),
    )
    parser.add_argument("--runtime-profile-selector-deployment-label", default="")
    parser.add_argument("--runtime-profile-selector-scenario-label", default="")
    parser.add_argument("--runtime-profile-selector-scenario-name", default="")
    parser.add_argument(
        "--install-rehearsal-profile",
        type=Path,
        help="Optional path where the validated active_runtime_profile.json is copied after package check PASS.",
    )
    parser.add_argument(
        "--install-rehearsal-dir",
        type=Path,
        help="Optional directory for install_rehearsal install_manifest.json/txt outputs.",
    )
    parser.add_argument("--force-install-rehearsal", action="store_true")
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_profile_payload(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def rdf_axis_nonregression_status(summary: Mapping[str, object]) -> str:
    matrix_rows = summary.get("matrix_rows", [])
    if not isinstance(matrix_rows, list):
        return "MISSING"
    for row in matrix_rows:
        if not isinstance(row, Mapping):
            continue
        if row.get("stage") == "single_run_output_audit" and row.get("item") == "rdf_axis_rmse_nonregression":
            return "PASS" if row.get("status") == "PASS" else "FAIL"
    return "MISSING"


def deployment_manifest(
    *,
    active_profile_path: Path,
    active_profile_sha256: str,
    promotion_check_json: Path,
    promotion: dict[str, object],
    final_summary: dict[str, object],
    public_payload_summary: dict[str, object] | None,
    public_payload_json: Path | None,
    public_payload_txt: Path | None,
    public_payload_csv: Path | None,
    wrapper_status: str,
) -> dict[str, object]:
    profile_payload = load_profile_payload(active_profile_path)
    return {
        "mode": "stereodistance_active_profile_deployment_manifest",
        "status": wrapper_status,
        "active_profile": str(active_profile_path),
        "active_profile_sha256": active_profile_sha256,
        "profile_id": profile_payload.get("profile_id", ""),
        "method": profile_payload.get("method", ""),
        "profile_mode": profile_payload.get("mode", ""),
        "axis_contract": profile_payload.get("axis_contract", ""),
        "runtime_inputs": profile_payload.get("runtime_inputs", []),
        "forbidden_runtime_inputs": profile_payload.get("forbidden_runtime_inputs", []),
        "promotion_check_json": str(promotion_check_json),
        "promotion_pass": bool(promotion.get("pass_promotion_check", False)),
        "promotion_sample_apply_count": promotion.get("sample_apply_count", 0),
        "promotion_sample_invalid_count": promotion.get("sample_invalid_count", 0),
        "final_acceptance_summary": str(
            Path(str(final_summary.get("output_dir", ""))) / "rtk_stereo_acceptance_matrix_summary.json"
            if final_summary.get("output_dir")
            else ""
        ),
        "final_overall_pass": bool(final_summary.get("overall_pass", False)),
        "final_matrix_item_count": final_summary.get("matrix_item_count", 0),
        "final_rdf_axis_nonregression_status": rdf_axis_nonregression_status(final_summary),
        "public_topic_payload_status": "" if public_payload_summary is None else public_payload_summary.get("status", ""),
        "public_topic_payload_sample_count": 0 if public_payload_summary is None else public_payload_summary.get("sample_count", 0),
        "public_topic_payload_invalid_count": 0 if public_payload_summary is None else public_payload_summary.get("invalid_count", 0),
        "public_topic_payload_node_equivalence_status": ""
        if public_payload_summary is None
        else public_payload_summary.get("node_public_payload_equivalence_status", ""),
        "public_topic_payload_json": "" if public_payload_json is None else str(public_payload_json),
        "public_topic_payload_txt": "" if public_payload_txt is None else str(public_payload_txt),
        "public_topic_payload_csv": "" if public_payload_csv is None else str(public_payload_csv),
        "runtime_truth_used": False,
        "coordinate_contract": (
            "runtime consumes stereo-center RDF(right,down,forward) plus measured disparity; "
            "PX4/RTK/NED truth is calibration/evaluation only; public StereoDistance topics use FRD+range"
        ),
    }


def write_manifest_text(path: Path, manifest: dict[str, object]) -> None:
    lines = [
        "StereoDistance active profile deployment manifest",
        f"status: {manifest.get('status', '')}",
        f"active_profile: {manifest.get('active_profile', '')}",
        f"active_profile_sha256: {manifest.get('active_profile_sha256', '')}",
        f"profile_id: {manifest.get('profile_id', '')}",
        f"method: {manifest.get('method', '')}",
        f"profile_mode: {manifest.get('profile_mode', '')}",
        f"promotion_pass: {manifest.get('promotion_pass', False)}",
        f"promotion_sample_apply_count: {manifest.get('promotion_sample_apply_count', 0)}",
        f"promotion_sample_invalid_count: {manifest.get('promotion_sample_invalid_count', 0)}",
        f"final_overall_pass: {manifest.get('final_overall_pass', False)}",
        f"runtime_truth_used: {manifest.get('runtime_truth_used', False)}",
        f"coordinate_contract: {manifest.get('coordinate_contract', '')}",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_text(path: Path, payload: dict[str, object]) -> None:
    lines = [
        "RTK stereo acceptance dashboard with promotion",
        f"status: {payload.get('status', '')}",
        f"output_dir: {payload.get('output_dir', '')}",
        f"output_active_profile: {payload.get('output_active_profile', '')}",
        f"promotion_output_dir: {payload.get('promotion_output_dir', '')}",
        f"initial_overall_pass: {payload.get('initial_overall_pass', False)}",
        f"promotion_pass: {payload.get('promotion_pass', False)}",
        f"promotion_sample_apply_count: {payload.get('promotion_sample_apply_count', 0)}",
        f"promotion_sample_invalid_count: {payload.get('promotion_sample_invalid_count', 0)}",
        f"active_profile_hash_stable: {payload.get('active_profile_hash_stable', False)}",
        f"active_profile_sha256_after_promotion: {payload.get('active_profile_sha256_after_promotion', '')}",
        f"active_profile_sha256_after_final_dashboard: {payload.get('active_profile_sha256_after_final_dashboard', '')}",
        f"final_overall_pass: {payload.get('final_overall_pass', False)}",
        f"final_matrix_item_count: {payload.get('final_matrix_item_count', 0)}",
        f"final_rdf_axis_nonregression_status: {payload.get('final_rdf_axis_nonregression_status', '')}",
        f"deployment_manifest_json: {payload.get('deployment_manifest_json', '')}",
        f"deployment_manifest_txt: {payload.get('deployment_manifest_txt', '')}",
        f"deployment_manifest_validation_status: {payload.get('deployment_manifest_validation_status', '')}",
        f"deployment_manifest_validation_json: {payload.get('deployment_manifest_validation_json', '')}",
        f"deployment_package_manifest_json: {payload.get('deployment_package_manifest_json', '')}",
        f"deployment_package_check_status: {payload.get('deployment_package_check_status', '')}",
        f"deployment_package_check_json: {payload.get('deployment_package_check_json', '')}",
        f"deployment_package_inventory_status: {payload.get('deployment_package_inventory_status', '')}",
        f"deployment_package_inventory_json: {payload.get('deployment_package_inventory_json', '')}",
        f"operator_view_launch_check_status: {payload.get('operator_view_launch_check_status', '')}",
        f"operator_view_launch_check_json: {payload.get('operator_view_launch_check_json', '')}",
        f"operator_view_launch_resolved_profile: {payload.get('operator_view_launch_resolved_profile', '')}",
        f"operator_view_selector_launch_check_status: {payload.get('operator_view_selector_launch_check_status', '')}",
        f"operator_view_selector_launch_check_json: {payload.get('operator_view_selector_launch_check_json', '')}",
        f"operator_view_selector_selected_label: {payload.get('operator_view_selector_selected_label', '')}",
        f"operator_view_selector_resolved_profile: {payload.get('operator_view_selector_resolved_profile', '')}",
        f"runtime_deployment_frontier_status: {payload.get('runtime_deployment_frontier_status', '')}",
        f"runtime_deployment_frontier_json: {payload.get('runtime_deployment_frontier_json', '')}",
        f"runtime_deployment_frontier_labels_csv: {payload.get('runtime_deployment_frontier_labels_csv', '')}",
        f"deployment_package_portability_status: {payload.get('deployment_package_portability_status', '')}",
        f"deployment_package_portability_json: {payload.get('deployment_package_portability_json', '')}",
        f"install_rehearsal_status: {payload.get('install_rehearsal_status', '')}",
        f"install_rehearsal_profile: {payload.get('install_rehearsal_profile', '')}",
        f"install_rehearsal_json: {payload.get('install_rehearsal_json', '')}",
        f"deployment_readiness_status: {payload.get('deployment_readiness_status', '')}",
        f"deployment_readiness_json: {payload.get('deployment_readiness_json', '')}",
        f"deployment_readiness_csv: {payload.get('deployment_readiness_csv', '')}",
        f"matrix_active_profile_gate_json: {payload.get('matrix_active_profile_gate_json', '')}",
        f"matrix_active_profile_gate_status: {payload.get('matrix_active_profile_gate_status', '')}",
        f"matrix_active_profile_gate_recommendation: {payload.get('matrix_active_profile_gate_recommendation', '')}",
        f"public_topic_payload_status: {payload.get('public_topic_payload_status', '')}",
        f"public_topic_payload_node_equivalence_status: {payload.get('public_topic_payload_node_equivalence_status', '')}",
        f"public_topic_payload_csv: {payload.get('public_topic_payload_csv', '')}",
        "runtime_truth_used: False",
        "Coordinate contract: calibration/evaluation truth may be PX4 local NED; active runtime profile consumes stereo-center RDF(right,down,forward) plus measured disparity; public StereoDistance alias is FRD+range.",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    promotion_output_dir = args.promotion_output_dir or (args.output_dir / "active_profile_promotion_check")

    initial_summary = acceptance_summary.run(
        single_audit_json=args.single_audit_json,
        multi_seed_summary_json=args.multi_seed_summary_json,
        robustness_summary_json=args.robustness_summary_json,
        output_dir=args.output_dir,
        deployment_strategy_json=args.deployment_strategy_json,
        output_active_profile=args.output_active_profile,
        scenario=args.scenario,
        catalog_json=args.catalog_json,
        minimal_set_json=args.minimal_set_json,
    )
    promotion = promotion_check.run_check(
        args.output_active_profile,
        promotion_output_dir,
        sample_csv=args.calibration_samples_csv,
        split=args.split,
        max_samples=args.max_samples,
    )
    promotion_check_json = promotion_output_dir / "promotion_check.json"
    active_profile_sha256_after_promotion = sha256_file(args.output_active_profile)
    final_summary = acceptance_summary.run(
        single_audit_json=args.single_audit_json,
        multi_seed_summary_json=args.multi_seed_summary_json,
        robustness_summary_json=args.robustness_summary_json,
        output_dir=args.output_dir,
        deployment_strategy_json=args.deployment_strategy_json,
        output_active_profile=args.output_active_profile,
        scenario=args.scenario,
        catalog_json=args.catalog_json,
        minimal_set_json=args.minimal_set_json,
        promotion_check_json=promotion_check_json,
    )
    active_profile_sha256_after_final_dashboard = sha256_file(args.output_active_profile)
    active_profile_hash_stable = active_profile_sha256_after_promotion == active_profile_sha256_after_final_dashboard
    status = "PASS" if bool(final_summary.get("overall_pass", False)) and active_profile_hash_stable else "FAIL"
    payload = {
        "status": status,
        "output_dir": str(args.output_dir),
        "output_active_profile": str(args.output_active_profile),
        "promotion_output_dir": str(promotion_output_dir),
        "promotion_check_json": str(promotion_check_json),
        "initial_overall_pass": bool(initial_summary.get("overall_pass", False)),
        "promotion_pass": bool(promotion.get("pass_promotion_check", False)),
        "promotion_sample_apply_count": promotion.get("sample_apply_count", 0),
        "promotion_sample_invalid_count": promotion.get("sample_invalid_count", 0),
        "active_profile_sha256_after_promotion": active_profile_sha256_after_promotion,
        "active_profile_sha256_after_final_dashboard": active_profile_sha256_after_final_dashboard,
        "active_profile_hash_stable": active_profile_hash_stable,
        "final_overall_pass": bool(final_summary.get("overall_pass", False)),
        "final_matrix_item_count": final_summary.get("matrix_item_count", 0),
        "final_stage_status": final_summary.get("stage_status", {}),
        "final_rdf_axis_nonregression_status": rdf_axis_nonregression_status(final_summary),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }
    public_payload_summary: dict[str, object] | None = None
    public_payload_dir = args.output_dir / "public_topic_payload"
    if payload["status"] == "PASS":
        public_payload_summary = public_payload_exporter.export_payloads(
            profile_json=args.output_active_profile,
            sample_csv=args.calibration_samples_csv,
            output_dir=public_payload_dir,
            split=args.split,
            max_samples=args.max_samples,
        )
        if public_payload_summary.get("status") != "PASS":
            payload["status"] = "FAIL"
    public_payload_json = public_payload_dir / "public_corrected_position_payload_summary.json" if public_payload_summary else None
    public_payload_txt = public_payload_dir / "public_corrected_position_payload_summary.txt" if public_payload_summary else None
    public_payload_csv = public_payload_dir / "public_corrected_position_payload.csv" if public_payload_summary else None
    manifest = deployment_manifest(
        active_profile_path=args.output_active_profile,
        active_profile_sha256=active_profile_sha256_after_final_dashboard,
        promotion_check_json=promotion_check_json,
        promotion=promotion,
        final_summary={**final_summary, "output_dir": str(args.output_dir)},
        public_payload_summary=public_payload_summary,
        public_payload_json=public_payload_json,
        public_payload_txt=public_payload_txt,
        public_payload_csv=public_payload_csv,
        wrapper_status=str(payload["status"]),
    )
    manifest_json = args.output_dir / "active_profile_deployment_manifest.json"
    manifest_txt = args.output_dir / "active_profile_deployment_manifest.txt"
    write_json(manifest_json, manifest)
    write_manifest_text(manifest_txt, manifest)
    manifest_validation_json = args.output_dir / "active_profile_deployment_manifest_validation.json"
    manifest_validation_txt = args.output_dir / "active_profile_deployment_manifest_validation.txt"
    manifest_validation = manifest_validator.validate_manifest(manifest_json)
    manifest_validator.write_json(manifest_validation_json, manifest_validation)
    manifest_validator.write_text(manifest_validation_txt, manifest_validation)
    if manifest_validation.get("status") != "PASS":
        payload["status"] = "FAIL"
    package_manifest: dict[str, object] | None = None
    package_dir = args.output_dir / "deployment_package"
    if payload["status"] == "PASS":
        package_manifest = deployment_package_exporter.export_package(
            manifest_json=manifest_json,
            manifest_validation_json=manifest_validation_json,
            output_dir=package_dir,
        )
    package_check: dict[str, object] | None = None
    if payload["status"] == "PASS" and args.runtime_profile_selector_json is None:
        package_check = deployment_package_checker.check_package(package_dir)
        deployment_package_checker.write_json(package_dir / "deployment_package_check.json", package_check)
        deployment_package_checker.write_text(package_dir / "deployment_package_check.txt", package_check)
        if package_check.get("status") != "PASS":
            payload["status"] = "FAIL"
    package_inventory: dict[str, object] | None = None
    package_inventory_dir = package_dir / "inventory"
    if payload["status"] == "PASS" and args.runtime_profile_selector_json is None:
        package_inventory = package_inventory_exporter.export_inventory(package_dir, package_inventory_dir)
        if package_inventory.get("status") != "PASS":
            payload["status"] = "FAIL"
    operator_launch_check: dict[str, object] | None = None
    operator_launch_json = args.output_dir / "operator_view_launch_check.json"
    operator_launch_txt = args.output_dir / "operator_view_launch_check.txt"
    if payload["status"] == "PASS":
        operator_launch_check = operator_view_launch_checker.check_operator_view_launch(package_dir=package_dir)
        operator_view_launch_checker.write_json(operator_launch_json, operator_launch_check)
        operator_view_launch_checker.write_text(operator_launch_txt, operator_launch_check)
        if operator_launch_check.get("status") != "PASS":
            payload["status"] = "FAIL"
    operator_selector_launch_check: dict[str, object] | None = None
    operator_selector_launch_json = args.output_dir / "operator_view_launch_check_selector.json"
    operator_selector_launch_txt = args.output_dir / "operator_view_launch_check_selector.txt"
    if payload["status"] == "PASS" and args.runtime_profile_selector_json is not None:
        operator_selector_launch_check = operator_view_launch_checker.check_operator_view_launch(
            selector_json=args.runtime_profile_selector_json,
            deployment_label=args.runtime_profile_selector_deployment_label,
            scenario_label=args.runtime_profile_selector_scenario_label,
            scenario_name=args.runtime_profile_selector_scenario_name,
        )
        operator_view_launch_checker.write_json(operator_selector_launch_json, operator_selector_launch_check)
        operator_view_launch_checker.write_text(operator_selector_launch_txt, operator_selector_launch_check)
        if operator_selector_launch_check.get("status") != "PASS":
            payload["status"] = "FAIL"
    runtime_frontier: dict[str, object] | None = None
    runtime_frontier_dir = args.output_dir / "runtime_deployment_frontier"
    if payload["status"] == "PASS" and args.runtime_profile_selector_json is not None:
        runtime_frontier = runtime_frontier_exporter.export_frontier(
            selector_json=args.runtime_profile_selector_json,
            output_dir=runtime_frontier_dir,
        )
        if runtime_frontier.get("status") != "PASS":
            payload["status"] = "FAIL"
    if payload["status"] == "PASS" and args.runtime_profile_selector_json is not None:
        deployment_package_exporter.append_runtime_deployment_evidence(
            package_dir=package_dir,
            selector_json=args.runtime_profile_selector_json,
            frontier_dir=runtime_frontier_dir,
        )
    package_portability: dict[str, object] | None = None
    package_portability_json = package_dir / "evidence" / "package_portability_check.json"
    package_portability_txt = package_dir / "evidence" / "package_portability_check.txt"
    if payload["status"] == "PASS" and args.runtime_profile_selector_json is not None:
        package_portability = package_portability_checker.check_portability(
            package_dir=package_dir,
            deployment_label=args.runtime_profile_selector_deployment_label,
            scenario_label=args.runtime_profile_selector_scenario_label,
            scenario_name=args.runtime_profile_selector_scenario_name,
        )
        package_portability_checker.write_json(package_portability_json, package_portability)
        package_portability_checker.write_text(package_portability_txt, package_portability)
        if package_portability.get("status") != "PASS":
            payload["status"] = "FAIL"
    if payload["status"] == "PASS" and args.runtime_profile_selector_json is not None:
        deployment_package_exporter.append_package_portability_evidence(
            package_dir=package_dir,
            portability_json=package_portability_json,
            portability_txt=package_portability_txt,
        )
        package_check = deployment_package_checker.check_package(package_dir)
        deployment_package_checker.write_json(package_dir / "deployment_package_check.json", package_check)
        deployment_package_checker.write_text(package_dir / "deployment_package_check.txt", package_check)
        if package_check.get("status") != "PASS":
            payload["status"] = "FAIL"
    if payload["status"] == "PASS" and args.runtime_profile_selector_json is not None:
        package_selector_json = package_dir / "evidence" / "package_runtime_profile_label_selector.json"
        operator_selector_launch_check = operator_view_launch_checker.check_operator_view_launch(
            selector_json=package_selector_json,
            deployment_label=args.runtime_profile_selector_deployment_label,
            scenario_label=args.runtime_profile_selector_scenario_label,
            scenario_name=args.runtime_profile_selector_scenario_name,
        )
        operator_view_launch_checker.write_json(operator_selector_launch_json, operator_selector_launch_check)
        operator_view_launch_checker.write_text(operator_selector_launch_txt, operator_selector_launch_check)
        if operator_selector_launch_check.get("status") != "PASS":
            payload["status"] = "FAIL"
    if payload["status"] == "PASS" and args.runtime_profile_selector_json is not None:
        package_inventory = package_inventory_exporter.export_inventory(package_dir, package_inventory_dir)
        if package_inventory.get("status") != "PASS":
            payload["status"] = "FAIL"
    install_rehearsal: dict[str, object] | None = None
    install_rehearsal_dir = args.install_rehearsal_dir or (args.output_dir / "install_rehearsal")
    if payload["status"] == "PASS" and args.install_rehearsal_profile is not None:
        install_rehearsal = active_profile_installer.install_package(
            package_dir=package_dir,
            output_profile=args.install_rehearsal_profile,
            force=args.force_install_rehearsal,
        )
        active_profile_installer.write_json(install_rehearsal_dir / "install_manifest.json", install_rehearsal)
        active_profile_installer.write_text(install_rehearsal_dir / "install_manifest.txt", install_rehearsal)
        if install_rehearsal.get("status") != "PASS":
            payload["status"] = "FAIL"
    payload["deployment_manifest_json"] = str(manifest_json)
    payload["deployment_manifest_txt"] = str(manifest_txt)
    payload["deployment_manifest_validation_status"] = manifest_validation.get("status", "")
    payload["deployment_manifest_validation_json"] = str(manifest_validation_json)
    payload["deployment_manifest_validation_txt"] = str(manifest_validation_txt)
    final_package_manifest = package_manifest
    final_package_manifest_json = package_dir / "deployment_package_manifest.json"
    if final_package_manifest_json.is_file():
        final_package_manifest = deployment_package_exporter.load_json(final_package_manifest_json)
    payload["deployment_package_dir"] = str(package_dir)
    payload["deployment_package_manifest_json"] = str(final_package_manifest_json)
    payload["deployment_package_file_count"] = (
        final_package_manifest.get("file_count", 0) if final_package_manifest else 0
    )
    payload["deployment_package_check_status"] = package_check.get("status", "") if package_check else ""
    payload["deployment_package_check_json"] = str(package_dir / "deployment_package_check.json")
    payload["deployment_package_check_txt"] = str(package_dir / "deployment_package_check.txt")
    payload["deployment_package_inventory_status"] = package_inventory.get("status", "") if package_inventory else ""
    payload["deployment_package_inventory_json"] = (
        str(package_inventory_dir / "stereodistance_deployment_package_inventory.json") if package_inventory else ""
    )
    payload["deployment_package_inventory_csv"] = (
        str(package_inventory_dir / "stereodistance_deployment_package_inventory.csv") if package_inventory else ""
    )
    payload["deployment_package_inventory_txt"] = (
        str(package_inventory_dir / "stereodistance_deployment_package_inventory.txt") if package_inventory else ""
    )
    payload["deployment_package_inventory_file_count"] = package_inventory.get("file_count", 0) if package_inventory else 0
    payload["operator_view_launch_check_status"] = operator_launch_check.get("status", "") if operator_launch_check else ""
    payload["operator_view_launch_check_json"] = str(operator_launch_json) if operator_launch_check else ""
    payload["operator_view_launch_check_txt"] = str(operator_launch_txt) if operator_launch_check else ""
    payload["operator_view_launch_resolved_profile"] = (
        operator_launch_check.get("resolved_runtime_profile", "") if operator_launch_check else ""
    )
    payload["operator_view_selector_launch_check_status"] = (
        operator_selector_launch_check.get("status", "") if operator_selector_launch_check else ""
    )
    payload["operator_view_selector_launch_check_json"] = (
        str(operator_selector_launch_json) if operator_selector_launch_check else ""
    )
    payload["operator_view_selector_launch_check_txt"] = (
        str(operator_selector_launch_txt) if operator_selector_launch_check else ""
    )
    payload["operator_view_selector_selected_label"] = (
        operator_selector_launch_check.get("selected_label", "") if operator_selector_launch_check else ""
    )
    payload["operator_view_selector_resolved_profile"] = (
        operator_selector_launch_check.get("resolved_runtime_profile", "") if operator_selector_launch_check else ""
    )
    payload["runtime_deployment_frontier_status"] = runtime_frontier.get("status", "") if runtime_frontier else ""
    payload["runtime_deployment_frontier_json"] = (
        str(runtime_frontier_dir / "runtime_deployment_frontier.json") if runtime_frontier else ""
    )
    payload["runtime_deployment_frontier_txt"] = (
        str(runtime_frontier_dir / "runtime_deployment_frontier.txt") if runtime_frontier else ""
    )
    payload["runtime_deployment_frontier_labels_csv"] = (
        str(runtime_frontier_dir / "runtime_deployment_frontier_labels.csv") if runtime_frontier else ""
    )
    payload["runtime_deployment_frontier_profiles_csv"] = (
        str(runtime_frontier_dir / "runtime_deployment_frontier_profiles.csv") if runtime_frontier else ""
    )
    payload["deployment_package_portability_status"] = package_portability.get("status", "") if package_portability else ""
    payload["deployment_package_portability_json"] = str(package_portability_json) if package_portability else ""
    payload["deployment_package_portability_txt"] = str(package_portability_txt) if package_portability else ""
    payload["deployment_package_portability_selected_label"] = (
        package_portability.get("selected_label", "") if package_portability else ""
    )
    payload["deployment_package_portability_copied_package_check_status"] = (
        package_portability.get("copied_package_check_status", "") if package_portability else ""
    )
    payload["install_rehearsal_status"] = install_rehearsal.get("status", "") if install_rehearsal else ""
    payload["install_rehearsal_profile"] = str(args.install_rehearsal_profile) if args.install_rehearsal_profile else ""
    payload["install_rehearsal_json"] = str(install_rehearsal_dir / "install_manifest.json") if install_rehearsal else ""
    payload["install_rehearsal_txt"] = str(install_rehearsal_dir / "install_manifest.txt") if install_rehearsal else ""
    payload["install_rehearsal_sha256"] = install_rehearsal.get("installed_profile_sha256", "") if install_rehearsal else ""
    payload["public_topic_payload_status"] = public_payload_summary.get("status", "") if public_payload_summary else ""
    payload["public_topic_payload_node_equivalence_status"] = (
        public_payload_summary.get("node_public_payload_equivalence_status", "") if public_payload_summary else ""
    )
    payload["public_topic_payload_json"] = (
        str(public_payload_dir / "public_corrected_position_payload_summary.json") if public_payload_summary else ""
    )
    payload["public_topic_payload_txt"] = (
        str(public_payload_dir / "public_corrected_position_payload_summary.txt") if public_payload_summary else ""
    )
    payload["public_topic_payload_csv"] = (
        str(public_payload_dir / "public_corrected_position_payload.csv") if public_payload_summary else ""
    )
    payload["matrix_active_profile_gate_json"] = str(args.matrix_active_profile_gate_json or "")
    wrapper_summary_json = args.output_dir / "rtk_stereo_acceptance_dashboard_with_promotion_summary.json"
    wrapper_summary_txt = args.output_dir / "rtk_stereo_acceptance_dashboard_with_promotion_summary.txt"
    write_json(wrapper_summary_json, payload)
    write_text(wrapper_summary_txt, payload)
    readiness: dict[str, object] | None = None
    if install_rehearsal is not None:
        readiness_json = args.output_dir / "deployment_readiness.json"
        readiness_txt = args.output_dir / "deployment_readiness.txt"
        readiness_csv = args.output_dir / "deployment_readiness.csv"
        readiness = deployment_readiness_checker.check_readiness(
            wrapper_summary_json,
            args.matrix_active_profile_gate_json,
        )
        deployment_readiness_checker.write_json(readiness_json, readiness)
        deployment_readiness_checker.write_text(readiness_txt, readiness)
        deployment_readiness_checker.write_csv(readiness_csv, readiness)
        payload["deployment_readiness_status"] = readiness.get("status", "")
        payload["deployment_readiness_json"] = str(readiness_json)
        payload["deployment_readiness_txt"] = str(readiness_txt)
        payload["deployment_readiness_csv"] = str(readiness_csv)
        payload["matrix_active_profile_gate_status"] = readiness.get("matrix_active_profile_gate_status", "")
        payload["matrix_active_profile_gate_recommendation"] = readiness.get(
            "matrix_active_profile_gate_recommendation",
            "",
        )
        if readiness.get("status") != "PASS":
            payload["status"] = "FAIL"
        write_json(wrapper_summary_json, payload)
        write_text(wrapper_summary_txt, payload)
    else:
        payload["deployment_readiness_status"] = ""
        payload["deployment_readiness_json"] = ""
        payload["deployment_readiness_txt"] = ""
        payload["deployment_readiness_csv"] = ""
        payload["matrix_active_profile_gate_status"] = ""
        payload["matrix_active_profile_gate_recommendation"] = ""
        write_json(wrapper_summary_json, payload)
        write_text(wrapper_summary_txt, payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
