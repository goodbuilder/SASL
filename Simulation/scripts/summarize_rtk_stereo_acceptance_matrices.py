#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from audit_rtk_stereo_calibration_outputs import (  # noqa: E402
    DELIVERABLE_EVIDENCE_ITEMS,
    has_forbidden_truth_name,
    load_runtime_profile,
    runtime_profile_apply_smoke,
    truth_forbidden_declared,
)
from select_stereodistance_profile_from_catalog import select_profile_from_deployment_strategy  # noqa: E402

RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
NED_AXIS_CONTRACT = "px4_local_position_ned:north,east,down"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize single-run, multi-seed, and robustness RTK stereo acceptance matrices."
    )
    parser.add_argument("--single-audit-json", type=Path, required=True)
    parser.add_argument("--multi-seed-summary-json", type=Path, required=True)
    parser.add_argument("--robustness-summary-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--deployment-strategy-json",
        type=Path,
        help="Optional robustness deployment_strategy_recommendation.json used to export an active StereoDistance profile.",
    )
    parser.add_argument(
        "--output-active-profile",
        type=Path,
        help="Optional output path for the selected active StereoDistance runtime profile.",
    )
    parser.add_argument(
        "--scenario",
        help="Deployment scenario label required when the deployment strategy is scenario_profile_catalog.",
    )
    parser.add_argument("--catalog-json", type=Path)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument(
        "--promotion-check-json",
        type=Path,
        help="Optional stereodistance_profile_promotion_check.py output for the exported active profile.",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


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


def matrix_rows(stage: str, matrix: Mapping[str, object]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for item_name, item in matrix.items():
        if not isinstance(item, Mapping):
            rows.append(
                {
                    "stage": stage,
                    "item": str(item_name),
                    "status": "FAIL",
                    "path": "",
                    "evidence_json": "{}",
                }
            )
            continue
        evidence = item.get("evidence", {})
        rows.append(
            {
                "stage": stage,
                "item": str(item_name),
                "status": str(item.get("status", "")),
                "path": str(item.get("path", "")),
                "evidence_json": json.dumps(evidence if isinstance(evidence, Mapping) else {}, sort_keys=True),
            }
        )
    return rows


def audit_active_profile_file(active_profile_selection: Mapping[str, object] | None) -> dict[str, object]:
    if not active_profile_selection:
        return {
            "pass_active_profile_file_check": False,
            "reason": "active_profile_not_selected",
        }
    output_profile = active_profile_selection.get("output_profile", "")
    profile_path = Path(str(output_profile))
    result: dict[str, object] = {
        "profile_path": str(profile_path),
        "profile_exists": profile_path.is_file(),
        "pass_active_profile_file_check": False,
    }
    if not profile_path.is_file():
        result["reason"] = "active_profile_file_missing"
        return result
    try:
        payload = load_json(profile_path)
    except (OSError, json.JSONDecodeError) as exc:
        result["reason"] = f"active_profile_read_error:{exc}"
        return result
    if not isinstance(payload, Mapping):
        result["reason"] = "active_profile_payload_not_mapping"
        return result

    axis_contract = str(payload.get("axis_contract", ""))
    runtime_inputs = payload.get("runtime_inputs", [])
    forbidden_runtime_inputs = payload.get("forbidden_runtime_inputs", [])
    axis_contract_ok = "RDF(right,down,forward)" in axis_contract
    runtime_inputs_truth_free = not has_forbidden_truth_name(runtime_inputs)
    forbidden_truth_inputs_declared = truth_forbidden_declared(forbidden_runtime_inputs)
    try:
        profile = load_runtime_profile(payload)
        runtime_loader_status = "PASS"
    except Exception as exc:
        profile = None
        runtime_loader_status = f"FAIL:{exc}"
    runtime_apply_ok = False
    runtime_apply_status = "not_run"
    if profile is not None:
        runtime_apply_ok, runtime_apply_status = runtime_profile_apply_smoke(profile)

    passed = (
        axis_contract_ok
        and runtime_inputs_truth_free
        and forbidden_truth_inputs_declared
        and runtime_loader_status == "PASS"
        and runtime_apply_ok
    )
    result.update(
        {
            "pass_active_profile_file_check": passed,
            "method": payload.get("method", ""),
            "mode": payload.get("mode", ""),
            "axis_contract": axis_contract,
            "axis_contract_ok": axis_contract_ok,
            "runtime_inputs": runtime_inputs,
            "runtime_inputs_truth_free": runtime_inputs_truth_free,
            "forbidden_runtime_inputs": forbidden_runtime_inputs,
            "forbidden_truth_inputs_declared": forbidden_truth_inputs_declared,
            "runtime_loader_status": runtime_loader_status,
            "runtime_apply_status": runtime_apply_status,
        }
    )
    if not passed:
        result["reason"] = "active_profile_file_contract_or_runtime_check_failed"
    return result


def promotion_check_passed(promotion_check: Mapping[str, object]) -> bool:
    return (
        bool(promotion_check.get("pass_promotion_check", False))
        and bool(promotion_check.get("schema_load_ok", False))
        and bool(promotion_check.get("node_loader_ok", False))
        and bool(promotion_check.get("axis_contract_declared", False))
        and bool(promotion_check.get("forbidden_runtime_inputs_declared", False))
        and not bool(promotion_check.get("runtime_truth_used", True))
        and int(promotion_check.get("sample_apply_count", 0) or 0) > 0
        and int(promotion_check.get("sample_invalid_count", 0) or 0) == 0
    )


def promotion_check_matrix_row(promotion_check: Mapping[str, object]) -> dict[str, object]:
    return {
        "stage": "standalone_profile_promotion",
        "item": "stereodistance_profile_promotion_check",
        "status": "PASS" if promotion_check_passed(promotion_check) else "FAIL",
        "path": str(promotion_check.get("profile_path", "")),
        "evidence_json": json.dumps(
            {
                "profile_mode": promotion_check.get("profile_mode", ""),
                "schema_load_ok": promotion_check.get("schema_load_ok", False),
                "node_loader_ok": promotion_check.get("node_loader_ok", False),
                "axis_contract_declared": promotion_check.get("axis_contract_declared", False),
                "runtime_truth_used": promotion_check.get("runtime_truth_used", True),
                "forbidden_runtime_inputs_declared": promotion_check.get("forbidden_runtime_inputs_declared", False),
                "sample_apply_count": promotion_check.get("sample_apply_count", 0),
                "sample_invalid_count": promotion_check.get("sample_invalid_count", 0),
                "rdf_axis_contract": promotion_check.get("rdf_axis_contract", ""),
                "public_topic_contract": promotion_check.get("public_topic_contract", ""),
            },
            sort_keys=True,
        ),
    }


def summarize_acceptance(
    *,
    single_audit: Mapping[str, object],
    multi_seed_summary: Mapping[str, object],
    robustness_summary: Mapping[str, object],
    active_profile_selection: Mapping[str, object] | None = None,
    active_profile_file_check: Mapping[str, object] | None = None,
    promotion_check: Mapping[str, object] | None = None,
) -> dict[str, object]:
    single_matrix = single_audit.get("deliverable_evidence_matrix", {})
    multi_seed_matrix = multi_seed_summary.get("multi_seed_acceptance_matrix", {})
    robustness_matrix = robustness_summary.get("robustness_acceptance_matrix", {})
    if not isinstance(single_matrix, Mapping):
        single_matrix = {}
    if not isinstance(multi_seed_matrix, Mapping):
        multi_seed_matrix = {}
    if not isinstance(robustness_matrix, Mapping):
        robustness_matrix = {}

    rows = []
    rows.extend(matrix_rows("single_run_output_audit", single_matrix))
    missing_single_items = [
        item_name for item_name in DELIVERABLE_EVIDENCE_ITEMS if item_name not in single_matrix
    ]
    for item_name in missing_single_items:
        rows.append(
            {
                "stage": "single_run_output_audit",
                "item": item_name,
                "status": "FAIL",
                "path": "",
                "evidence_json": json.dumps(
                    {"reason": "missing_required_single_run_evidence"},
                    sort_keys=True,
                ),
            }
        )
    rows.extend(matrix_rows("multi_seed_acceptance", multi_seed_matrix))
    rows.extend(matrix_rows("robustness_acceptance", robustness_matrix))
    failed_rows = [row for row in rows if str(row.get("status", "")) != "PASS"]

    stage_status = {
        "single_run_output_audit": str(single_audit.get("status", "")) == "PASS"
        and bool(single_matrix)
        and not missing_single_items
        and all(str(row.get("status", "")) == "PASS" for row in matrix_rows("single_run_output_audit", single_matrix)),
        "multi_seed_acceptance": bool(multi_seed_summary.get("multi_seed_acceptance_pass", False))
        and bool(multi_seed_matrix),
        "robustness_acceptance": bool(robustness_summary.get("robustness_acceptance_pass", False))
        and bool(robustness_matrix),
    }
    recommendations = {
        "single_run_recommendation": single_audit.get("recommendation", ""),
        "multi_seed_recommendation": (
            multi_seed_summary.get("multi_seed_recommendation", {}).get("method", "")
            if isinstance(multi_seed_summary.get("multi_seed_recommendation"), Mapping)
            else ""
        ),
        "robust_recommendation": (
            robustness_summary.get("robust_recommendation", {}).get("method", "")
            if isinstance(robustness_summary.get("robust_recommendation"), Mapping)
            else ""
        ),
        "deployment_strategy": (
            robustness_summary.get("deployment_strategy_recommendation", {}).get("deployment_strategy", "")
            if isinstance(robustness_summary.get("deployment_strategy_recommendation"), Mapping)
            else ""
        ),
    }
    active_selection = dict(active_profile_selection or {})
    active_profile_selected = bool(active_selection.get("selected", False))
    active_file_check = dict(active_profile_file_check or {})
    active_profile_file_pass = bool(active_file_check.get("pass_active_profile_file_check", False))
    if active_selection:
        stage_status["active_profile_export"] = active_profile_selected and (
            active_profile_file_pass if active_file_check else True
        )
        rows.append(
            {
                "stage": "active_profile_export",
                "item": "active_runtime_profile",
                "status": "PASS" if stage_status["active_profile_export"] else "FAIL",
                "path": str(active_selection.get("output_profile", "")),
                "evidence_json": json.dumps(
                    {
                        "selection_mode": active_selection.get("selection_mode", ""),
                        "deployment_strategy": active_selection.get("deployment_strategy", ""),
                        "method": active_selection.get("method", ""),
                        "axis_contract": active_selection.get("axis_contract", ""),
                        "runtime_input_contract": active_selection.get("runtime_input_contract", ""),
                        "file_check_pass": active_profile_file_pass if active_file_check else "",
                        "runtime_loader_status": active_file_check.get("runtime_loader_status", ""),
                        "runtime_apply_status": active_file_check.get("runtime_apply_status", ""),
                    },
                    sort_keys=True,
                ),
            }
        )
        failed_rows = [row for row in rows if str(row.get("status", "")) != "PASS"]
    standalone_promotion_check = dict(promotion_check or {})
    if standalone_promotion_check:
        stage_status["standalone_profile_promotion"] = promotion_check_passed(standalone_promotion_check)
        rows.append(promotion_check_matrix_row(standalone_promotion_check))
        failed_rows = [row for row in rows if str(row.get("status", "")) != "PASS"]
    overall_pass = all(stage_status.values()) and not failed_rows
    return {
        "overall_pass": overall_pass,
        "stage_status": stage_status,
        "active_profile_selected": active_profile_selected,
        "active_profile_selection": active_selection,
        "active_profile_file_check": active_file_check,
        "standalone_promotion_check": standalone_promotion_check,
        "failure_count": len(failed_rows),
        "failed_items": failed_rows,
        "matrix_item_count": len(rows),
        "matrix_rows": rows,
        "recommendations": recommendations,
        "runtime_truth_used": False,
        "ned_axis_contract": NED_AXIS_CONTRACT,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    recommendations = summary.get("recommendations", {})
    if not isinstance(recommendations, Mapping):
        recommendations = {}
    stage_status = summary.get("stage_status", {})
    if not isinstance(stage_status, Mapping):
        stage_status = {}
    active_profile_file_check = summary.get("active_profile_file_check", {})
    if not isinstance(active_profile_file_check, Mapping):
        active_profile_file_check = {}
    standalone_promotion_check = summary.get("standalone_promotion_check", {})
    if not isinstance(standalone_promotion_check, Mapping):
        standalone_promotion_check = {}
    lines = [
        "RTK stereo acceptance matrix summary",
        f"overall_pass: {summary.get('overall_pass', False)}",
        f"failure_count: {summary.get('failure_count', 0)}",
        f"single_run_recommendation: {recommendations.get('single_run_recommendation', '')}",
        f"multi_seed_recommendation: {recommendations.get('multi_seed_recommendation', '')}",
        f"robust_recommendation: {recommendations.get('robust_recommendation', '')}",
        f"deployment_strategy: {recommendations.get('deployment_strategy', '')}",
        f"active_profile_selected: {summary.get('active_profile_selected', False)}",
        f"active_profile_file_check_pass: {active_profile_file_check.get('pass_active_profile_file_check', False)}",
        f"active_profile_runtime_apply_status: {active_profile_file_check.get('runtime_apply_status', '')}",
        f"standalone_promotion_check_pass: {standalone_promotion_check.get('pass_promotion_check', False)}",
        f"standalone_promotion_sample_apply_count: {standalone_promotion_check.get('sample_apply_count', 0)}",
        f"NED contract: {summary.get('ned_axis_contract', '')}",
        f"RDF contract: {summary.get('rdf_axis_contract', '')}",
        f"StereoDistance public topic contract: {summary.get('public_topic_contract', '')}",
        "runtime_truth_used: False",
        "",
        "Stage status:",
    ]
    for stage, passed in stage_status.items():
        lines.append(f"- {stage}: {'PASS' if passed else 'FAIL'}")
    lines.extend(["", "Failed items:"])
    failed_items = summary.get("failed_items", [])
    if isinstance(failed_items, list) and failed_items:
        for row in failed_items:
            if isinstance(row, Mapping):
                lines.append(f"- {row.get('stage', '')}/{row.get('item', '')}: {row.get('status', '')}")
    else:
        lines.append("- none")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(
    *,
    single_audit_json: Path,
    multi_seed_summary_json: Path,
    robustness_summary_json: Path,
    output_dir: Path,
    deployment_strategy_json: Path | None = None,
    output_active_profile: Path | None = None,
    scenario: str | None = None,
    catalog_json: Path | None = None,
    minimal_set_json: Path | None = None,
    promotion_check_json: Path | None = None,
) -> dict[str, object]:
    active_profile_selection: dict[str, object] | None = None
    active_profile_file_check: dict[str, object] | None = None
    if deployment_strategy_json is not None or output_active_profile is not None:
        if deployment_strategy_json is None or output_active_profile is None:
            raise ValueError("--deployment-strategy-json and --output-active-profile must be provided together")
        active_profile_selection = select_profile_from_deployment_strategy(
            deployment_strategy_json=deployment_strategy_json,
            output_profile=output_active_profile,
            scenario=scenario,
            catalog_json=catalog_json,
            minimal_set_json=minimal_set_json,
        )
        active_profile_file_check = audit_active_profile_file(active_profile_selection)
    promotion_check = load_json(promotion_check_json) if promotion_check_json is not None else None
    summary = summarize_acceptance(
        single_audit=load_json(single_audit_json),
        multi_seed_summary=load_json(multi_seed_summary_json),
        robustness_summary=load_json(robustness_summary_json),
        active_profile_selection=active_profile_selection,
        active_profile_file_check=active_profile_file_check,
        promotion_check=promotion_check,
    )
    write_json(output_dir / "rtk_stereo_acceptance_matrix_summary.json", summary)
    matrix_rows_value = summary.get("matrix_rows", [])
    write_csv(
        output_dir / "rtk_stereo_acceptance_matrix_summary.csv",
        matrix_rows_value if isinstance(matrix_rows_value, list) else [],
    )
    write_text(output_dir / "rtk_stereo_acceptance_matrix_summary.txt", summary)
    return summary


def main() -> None:
    args = parse_args()
    summary = run(
        single_audit_json=args.single_audit_json,
        multi_seed_summary_json=args.multi_seed_summary_json,
        robustness_summary_json=args.robustness_summary_json,
        output_dir=args.output_dir,
        deployment_strategy_json=args.deployment_strategy_json,
        output_active_profile=args.output_active_profile,
        scenario=args.scenario,
        catalog_json=args.catalog_json,
        minimal_set_json=args.minimal_set_json,
        promotion_check_json=args.promotion_check_json,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    if not bool(summary.get("overall_pass", False)):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
