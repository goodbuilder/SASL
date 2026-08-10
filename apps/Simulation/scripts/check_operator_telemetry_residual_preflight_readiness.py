#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Mapping, Sequence


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate operator telemetry residual preflight artifacts before live residual enablement."
    )
    parser.add_argument("--preflight-summary-json", type=Path, required=True)
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


def as_path(value: object, base_dir: Path) -> Path:
    path = Path(str(value))
    if path.is_absolute():
        return path
    cwd_path = (Path.cwd() / path).resolve()
    if cwd_path.exists():
        return cwd_path
    return (base_dir / path).resolve()


def check_readiness(preflight_summary_json: Path) -> dict[str, object]:
    preflight_summary_json = preflight_summary_json.resolve()
    base_dir = preflight_summary_json.parent
    payload = load_json(preflight_summary_json)
    failures: list[str] = []
    if payload.get("status") != "PASS":
        failures.append("preflight:status")
    if payload.get("apply_residual_allowed") is not True:
        failures.append("preflight:apply_residual_allowed")
    if payload.get("operator_launch_status") != "PASS":
        failures.append("preflight:operator_launch_status")
    if payload.get("runtime_truth_used") is not False:
        failures.append("preflight:runtime_truth_used")

    decision_path = as_path(payload.get("selector_decision_json", ""), base_dir)
    launch_path = as_path(payload.get("operator_launch_check_json", ""), base_dir)
    recommended_profile = str(payload.get("recommended_residual_profile", ""))
    resolved_profile = str(payload.get("resolved_disparity_residual_profile", ""))
    if not decision_path.is_file():
        failures.append("decision:missing")
        decision_payload: dict[str, object] = {}
    else:
        decision_payload = load_json(decision_path)
        if decision_payload.get("status") != "PASS":
            failures.append("decision:status")
        if decision_payload.get("apply_residual_allowed") is not True:
            failures.append("decision:apply_residual_allowed")
        if decision_payload.get("runtime_truth_used") is not False:
            failures.append("decision:runtime_truth_used")
        if str(decision_payload.get("recommended_residual_profile", "")) != recommended_profile:
            failures.append("decision:recommended_profile_mismatch")

    if not launch_path.is_file():
        failures.append("operator_launch:missing")
        launch_payload: dict[str, object] = {}
    else:
        launch_payload = load_json(launch_path)
        if launch_payload.get("status") != "PASS":
            failures.append("operator_launch:status")
        if launch_payload.get("runtime_truth_used") is not False:
            failures.append("operator_launch:runtime_truth_used")
        if launch_payload.get("disparity_residual_arg_present") is not True:
            failures.append("operator_launch:residual_arg_missing")
        if str(launch_payload.get("resolved_disparity_residual_profile", "")) != recommended_profile:
            failures.append("operator_launch:recommended_profile_mismatch")
        if str(launch_payload.get("resolved_disparity_residual_profile", "")) != resolved_profile:
            failures.append("operator_launch:resolved_profile_mismatch")

    coordinate_contract = str(payload.get("coordinate_contract", ""))
    if "public FRD" not in coordinate_contract:
        failures.append("coordinate_contract:public_frd")
    if "stereo-center RDF" not in coordinate_contract:
        failures.append("coordinate_contract:stereo_center_rdf")
    if "FRD+range" not in coordinate_contract:
        failures.append("coordinate_contract:public_output")

    return {
        "mode": "operator_telemetry_residual_preflight_readiness",
        "status": "PASS" if not failures else "FAIL",
        "preflight_summary_json": str(preflight_summary_json),
        "selector_decision_json": str(decision_path),
        "operator_launch_check_json": str(launch_path),
        "recommended_residual_label": payload.get("recommended_residual_label", ""),
        "recommended_residual_profile": recommended_profile,
        "resolved_disparity_residual_profile": resolved_profile,
        "apply_residual_allowed": bool(payload.get("apply_residual_allowed", False)),
        "operator_launch_status": payload.get("operator_launch_status", ""),
        "decision_status": decision_payload.get("status", "") if decision_payload else "",
        "decision_apply_residual_allowed": decision_payload.get("apply_residual_allowed", False)
        if decision_payload
        else False,
        "operator_launch_residual_arg_present": launch_payload.get("disparity_residual_arg_present", False)
        if launch_payload
        else False,
        "runtime_truth_used": bool(payload.get("runtime_truth_used", True)),
        "coordinate_contract": (
            "preflight readiness validates a truth-free public-FRD telemetry to stereo-center-RDF selector "
            "decision path, then operator launch consumption of the recommended residual profile; public output "
            "remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator telemetry residual preflight readiness",
        f"status: {payload.get('status', '')}",
        f"preflight_summary_json: {payload.get('preflight_summary_json', '')}",
        f"selector_decision_json: {payload.get('selector_decision_json', '')}",
        f"operator_launch_check_json: {payload.get('operator_launch_check_json', '')}",
        f"recommended_residual_label: {payload.get('recommended_residual_label', '')}",
        f"recommended_residual_profile: {payload.get('recommended_residual_profile', '')}",
        f"resolved_disparity_residual_profile: {payload.get('resolved_disparity_residual_profile', '')}",
        f"apply_residual_allowed: {payload.get('apply_residual_allowed', False)}",
        f"operator_launch_status: {payload.get('operator_launch_status', '')}",
        f"decision_status: {payload.get('decision_status', '')}",
        f"decision_apply_residual_allowed: {payload.get('decision_apply_residual_allowed', False)}",
        f"operator_launch_residual_arg_present: {payload.get('operator_launch_residual_arg_present', False)}",
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


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    payload = check_readiness(args.preflight_summary_json)
    if args.output_json is not None:
        write_json(args.output_json, payload)
    if args.output_txt is not None:
        write_text(args.output_txt, payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
