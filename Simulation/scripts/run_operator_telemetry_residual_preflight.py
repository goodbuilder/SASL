#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_stereodistance_operator_view_launch as launch_check  # noqa: E402
import audit_operator_telemetry_residual_preflight_schema as telemetry_audit  # noqa: E402
import run_operator_telemetry_selector_replay as telemetry_replay  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run truth-free operator telemetry residual selector replay, emit a gated decision artifact, "
            "then verify the operator launch dry-run consumes that decision."
        )
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--package-dir", type=Path)
    source.add_argument("--profile-json", type=Path)
    source.add_argument("--runtime-selector-json", type=Path)
    parser.add_argument("--deployment-label", default="")
    parser.add_argument("--scenario-label", default="")
    parser.add_argument("--scenario-name", default="")
    parser.add_argument("--observation-fusion-manifest", type=Path)
    parser.add_argument("--observation-fusion-tier", default="")
    telemetry = parser.add_mutually_exclusive_group(required=True)
    telemetry.add_argument("--input-telemetry-csv", type=Path)
    telemetry.add_argument("--latest-telemetry-root", type=Path)
    parser.add_argument("--latest-telemetry-pattern", default="**/telemetry.csv")
    parser.add_argument("--selector-model-json", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--window-size", type=int, default=21)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--min-samples", type=int, default=1)
    parser.add_argument("--expected-label", default="")
    parser.add_argument("--feature-label", default="operator_telemetry_preflight_window")
    parser.add_argument("--allow-missing-corrected", action="store_true")
    parser.add_argument("--allow-missing-observation-rdf", action="store_true")
    parser.add_argument("--require-stable-label", action="store_true")
    parser.add_argument("--min-distance-margin", type=float, default=0.0)
    return parser.parse_args(argv)


def resolve_latest_telemetry(root: Path, pattern: str) -> Path:
    if not root.is_dir():
        raise FileNotFoundError(f"latest telemetry root not found: {root}")
    candidates = [path for path in root.glob(pattern) if path.is_file()]
    if not candidates:
        raise FileNotFoundError(f"no telemetry CSV matched {pattern!r} under {root}")
    return max(candidates, key=lambda path: (path.stat().st_mtime, str(path)))


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator telemetry residual preflight",
        f"status: {payload.get('status', '')}",
        f"input_telemetry_csv: {payload.get('input_telemetry_csv', '')}",
        f"schema_audit_json: {payload.get('schema_audit_json', '')}",
        f"schema_audit_status: {payload.get('schema_audit_status', '')}",
        f"schema_audit_valid_row_count: {payload.get('schema_audit_valid_row_count', 0)}",
        f"selector_replay_dir: {payload.get('selector_replay_dir', '')}",
        f"operator_launch_check_json: {payload.get('operator_launch_check_json', '')}",
        f"apply_residual_allowed: {payload.get('apply_residual_allowed', False)}",
        f"recommended_residual_label: {payload.get('recommended_residual_label', '')}",
        f"recommended_residual_profile: {payload.get('recommended_residual_profile', '')}",
        f"operator_launch_status: {payload.get('operator_launch_status', '')}",
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
    args.output_dir.mkdir(parents=True, exist_ok=True)
    input_telemetry_csv = (
        args.input_telemetry_csv
        if args.input_telemetry_csv is not None
        else resolve_latest_telemetry(args.latest_telemetry_root, args.latest_telemetry_pattern)
    )
    selector_replay_dir = args.output_dir / "telemetry_selector_replay"
    operator_launch_json = args.output_dir / "operator_launch_check_from_decision.json"
    operator_launch_txt = args.output_dir / "operator_launch_check_from_decision.txt"
    schema_audit_json = args.output_dir / "operator_telemetry_schema_audit.json"
    schema_audit_txt = args.output_dir / "operator_telemetry_schema_audit.txt"

    schema_audit_payload = telemetry_audit.audit_schema(
        input_csv=input_telemetry_csv,
        allow_missing_corrected=args.allow_missing_corrected,
        min_valid_rows=args.min_samples,
    )
    telemetry_audit.write_json(schema_audit_json, schema_audit_payload)
    telemetry_audit.write_text(schema_audit_txt, schema_audit_payload)

    replay_payload: dict[str, object] = {
        "mode": "operator_telemetry_truth_drop_selector_replay",
        "status": "SKIPPED",
        "input_telemetry_csv": str(input_telemetry_csv),
        "selector_model_json": str(args.selector_model_json),
        "catalog_json": str(args.catalog_json),
        "apply_residual_allowed": False,
        "recommended_residual_label": "",
        "recommended_residual_profile": "",
        "runtime_truth_used": False,
        "coordinate_contract": (
            "operator telemetry input is public FRD(forward,right,down,range); selector replay is skipped "
            "because schema audit failed before any residual profile could be selected"
        ),
        "failure_count": 1,
        "failures": ["schema_audit_failed"],
    }
    if schema_audit_payload.get("status") == "PASS":
        replay_args = [
            "--input-telemetry-csv",
            str(input_telemetry_csv),
            "--selector-model-json",
            str(args.selector_model_json),
            "--catalog-json",
            str(args.catalog_json),
            "--output-dir",
            str(selector_replay_dir),
            "--window-size",
            str(args.window_size),
            "--stride",
            str(args.stride),
            "--min-samples",
            str(args.min_samples),
            "--feature-label",
            args.feature_label,
        ]
        if args.expected_label:
            replay_args.extend(["--expected-label", args.expected_label])
        if args.allow_missing_corrected:
            replay_args.append("--allow-missing-corrected")
        if args.allow_missing_observation_rdf:
            replay_args.append("--allow-missing-observation-rdf")
        if args.require_stable_label:
            replay_args.append("--require-stable-label")
        if args.min_distance_margin > 0.0:
            replay_args.extend(["--min-distance-margin", str(args.min_distance_margin)])
        replay_payload = telemetry_replay.run(replay_args)
    else:
        selector_replay_dir.mkdir(parents=True, exist_ok=True)
        skipped_decision = {
            "mode": "operator_telemetry_selector_decision",
            "status": "SKIPPED",
            "apply_residual_allowed": False,
            "recommended_residual_label": "",
            "recommended_residual_profile": "",
            "selector_label_stable": False,
            "selected_label_counts": {},
            "distance_margin_min": 0.0,
            "require_stable_label": bool(args.require_stable_label),
            "min_distance_margin": max(0.0, float(args.min_distance_margin)),
            "runtime_truth_used": False,
            "decision_contract": (
                "apply_residual_allowed is false because operator telemetry schema audit failed before "
                "truth-free selector replay"
            ),
            "coordinate_contract": replay_payload["coordinate_contract"],
            "failure_count": 1,
            "failures": ["schema_audit_failed"],
        }
        telemetry_replay.write_json(selector_replay_dir / "operator_telemetry_selector_replay_summary.json", replay_payload)
        telemetry_replay.write_text(selector_replay_dir / "operator_telemetry_selector_replay_summary.txt", replay_payload)
        telemetry_replay.write_json(selector_replay_dir / "operator_telemetry_selector_decision.json", skipped_decision)
        telemetry_replay.write_decision_text(selector_replay_dir / "operator_telemetry_selector_decision.txt", skipped_decision)
    decision_json = selector_replay_dir / "operator_telemetry_selector_decision.json"

    launch_payload: dict[str, object] = {
        "status": "SKIPPED",
        "runtime_truth_used": False,
        "failures": ["selector_replay_failed"],
    }
    if replay_payload.get("status") == "PASS" and replay_payload.get("apply_residual_allowed") is True:
        launch_kwargs: dict[str, object] = {
            "package_dir": args.package_dir,
            "profile_json": args.profile_json,
            "selector_json": args.runtime_selector_json,
            "deployment_label": args.deployment_label,
            "scenario_label": args.scenario_label,
            "scenario_name": args.scenario_name,
            "observation_fusion_manifest": args.observation_fusion_manifest,
            "observation_fusion_tier": args.observation_fusion_tier,
            "disparity_residual_decision_json": decision_json,
        }
        launch_payload = launch_check.check_operator_view_launch(**launch_kwargs)
        launch_check.write_json(operator_launch_json, launch_payload)
        launch_check.write_text(operator_launch_txt, launch_payload)

    failures: list[str] = []
    if schema_audit_payload.get("status") != "PASS":
        failures.extend(f"schema_audit:{failure}" for failure in schema_audit_payload.get("failures", []))
    if replay_payload.get("status") != "PASS":
        failures.extend(f"selector_replay:{failure}" for failure in replay_payload.get("failures", []))
    if replay_payload.get("apply_residual_allowed") is not True:
        failures.append("selector_decision:apply_not_allowed")
    if launch_payload.get("status") != "PASS":
        failures.extend(f"operator_launch:{failure}" for failure in launch_payload.get("failures", []))
    runtime_truth_used = bool(replay_payload.get("runtime_truth_used", False)) or bool(
        launch_payload.get("runtime_truth_used", False)
    )
    if runtime_truth_used:
        failures.append("runtime_truth_used")

    payload = {
        "mode": "operator_telemetry_residual_preflight",
        "status": "PASS" if not failures else "FAIL",
        "input_telemetry_csv": str(input_telemetry_csv),
        "latest_telemetry_root": "" if args.latest_telemetry_root is None else str(args.latest_telemetry_root),
        "latest_telemetry_pattern": args.latest_telemetry_pattern,
        "schema_audit_json": str(schema_audit_json),
        "schema_audit_status": schema_audit_payload.get("status", ""),
        "schema_audit_valid_row_count": schema_audit_payload.get("valid_row_count", 0),
        "selector_replay_dir": str(selector_replay_dir),
        "selector_decision_json": str(decision_json),
        "operator_launch_check_json": str(operator_launch_json) if operator_launch_json.is_file() else "",
        "apply_residual_allowed": bool(replay_payload.get("apply_residual_allowed", False)),
        "recommended_residual_label": replay_payload.get("recommended_residual_label", ""),
        "recommended_residual_profile": replay_payload.get("recommended_residual_profile", ""),
        "operator_launch_status": launch_payload.get("status", ""),
        "resolved_disparity_residual_profile": launch_payload.get("resolved_disparity_residual_profile", ""),
        "runtime_truth_used": runtime_truth_used,
        "coordinate_contract": (
            "operator telemetry source is public FRD; selector replay emits stereo-center RDF observation "
            "features and a truth-free residual decision; operator launch consumes the decision only after "
            "gates pass; selected residual consumes corrected stereo-center RDF plus measured disparity; "
            "public output remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "selector_replay_summary": {
            key: value
            for key, value in replay_payload.items()
            if key not in {"feature_rows", "replay_rows", "decision", "export_summary", "replay_summary"}
        },
        "schema_audit_summary": schema_audit_payload,
        "operator_launch_summary": launch_payload,
    }
    write_json(args.output_dir / "operator_telemetry_residual_preflight_summary.json", payload)
    write_text(args.output_dir / "operator_telemetry_residual_preflight_summary.txt", payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
