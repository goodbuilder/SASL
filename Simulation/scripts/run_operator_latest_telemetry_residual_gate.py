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

import check_operator_telemetry_residual_preflight_readiness as readiness_check  # noqa: E402
import run_operator_telemetry_residual_preflight as residual_preflight  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Select the latest operator telemetry.csv, run residual preflight, and run readiness only when "
            "the truth-free preflight permits residual application."
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
    parser.add_argument("--latest-telemetry-root", type=Path, required=True)
    parser.add_argument("--latest-telemetry-pattern", default="**/telemetry.csv")
    parser.add_argument("--selector-model-json", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--window-size", type=int, default=21)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--expected-label", default="")
    parser.add_argument("--feature-label", default="operator_latest_telemetry_gate_window")
    parser.add_argument("--allow-missing-corrected", action="store_true")
    parser.add_argument("--allow-missing-observation-rdf", action="store_true")
    parser.add_argument("--require-stable-label", action="store_true")
    parser.add_argument("--min-distance-margin", type=float, default=0.0)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator latest telemetry residual gate",
        f"status: {payload.get('status', '')}",
        f"input_telemetry_csv: {payload.get('input_telemetry_csv', '')}",
        f"latest_telemetry_root: {payload.get('latest_telemetry_root', '')}",
        f"preflight_summary_json: {payload.get('preflight_summary_json', '')}",
        f"readiness_json: {payload.get('readiness_json', '')}",
        f"preflight_status: {payload.get('preflight_status', '')}",
        f"readiness_status: {payload.get('readiness_status', '')}",
        f"apply_residual_allowed: {payload.get('apply_residual_allowed', False)}",
        f"recommended_residual_label: {payload.get('recommended_residual_label', '')}",
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
    next_action = payload.get("next_action", "")
    if next_action:
        lines.append("")
        lines.append(f"next_action: {next_action}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    preflight_dir = args.output_dir / "preflight"

    preflight_args = [
        "--latest-telemetry-root",
        str(args.latest_telemetry_root),
        "--latest-telemetry-pattern",
        args.latest_telemetry_pattern,
        "--selector-model-json",
        str(args.selector_model_json),
        "--catalog-json",
        str(args.catalog_json),
        "--output-dir",
        str(preflight_dir),
        "--window-size",
        str(args.window_size),
        "--stride",
        str(args.stride),
        "--min-samples",
        str(args.min_samples),
        "--feature-label",
        args.feature_label,
    ]
    if args.package_dir is not None:
        preflight_args.extend(["--package-dir", str(args.package_dir)])
    if args.profile_json is not None:
        preflight_args.extend(["--profile-json", str(args.profile_json)])
    if args.runtime_selector_json is not None:
        preflight_args.extend(["--runtime-selector-json", str(args.runtime_selector_json)])
    if args.deployment_label:
        preflight_args.extend(["--deployment-label", args.deployment_label])
    if args.scenario_label:
        preflight_args.extend(["--scenario-label", args.scenario_label])
    if args.scenario_name:
        preflight_args.extend(["--scenario-name", args.scenario_name])
    if args.observation_fusion_manifest is not None:
        preflight_args.extend(["--observation-fusion-manifest", str(args.observation_fusion_manifest)])
    if args.observation_fusion_tier:
        preflight_args.extend(["--observation-fusion-tier", args.observation_fusion_tier])
    if args.expected_label:
        preflight_args.extend(["--expected-label", args.expected_label])
    if args.allow_missing_corrected:
        preflight_args.append("--allow-missing-corrected")
    if args.allow_missing_observation_rdf:
        preflight_args.append("--allow-missing-observation-rdf")
    if args.require_stable_label:
        preflight_args.append("--require-stable-label")
    if args.min_distance_margin > 0.0:
        preflight_args.extend(["--min-distance-margin", str(args.min_distance_margin)])

    preflight_payload = residual_preflight.run(preflight_args)
    preflight_summary_json = preflight_dir / "operator_telemetry_residual_preflight_summary.json"
    readiness_json = args.output_dir / "operator_latest_telemetry_residual_readiness.json"
    readiness_txt = args.output_dir / "operator_latest_telemetry_residual_readiness.txt"

    readiness_payload: dict[str, object] = {
        "status": "SKIPPED",
        "runtime_truth_used": False,
        "failures": ["preflight_not_passed"],
    }
    if preflight_payload.get("status") == "PASS" and preflight_payload.get("apply_residual_allowed") is True:
        readiness_payload = readiness_check.check_readiness(preflight_summary_json)
        readiness_check.write_json(readiness_json, readiness_payload)
        readiness_check.write_text(readiness_txt, readiness_payload)

    failures: list[str] = []
    if preflight_payload.get("status") != "PASS":
        failures.extend(f"preflight:{failure}" for failure in preflight_payload.get("failures", []))
    if preflight_payload.get("apply_residual_allowed") is not True:
        failures.append("preflight:apply_not_allowed")
    if readiness_payload.get("status") != "PASS":
        failures.extend(f"readiness:{failure}" for failure in readiness_payload.get("failures", []))
    runtime_truth_used = bool(preflight_payload.get("runtime_truth_used", False)) or bool(
        readiness_payload.get("runtime_truth_used", False)
    )
    if runtime_truth_used:
        failures.append("runtime_truth_used")

    next_action = ""
    if failures:
        next_action = (
            "Record a fresh run_operator_view.sh --record-lite telemetry with the current public FRD+range "
            "schema, then rerun this gate."
        )

    payload = {
        "mode": "operator_latest_telemetry_residual_gate",
        "status": "PASS" if not failures else "FAIL",
        "input_telemetry_csv": preflight_payload.get("input_telemetry_csv", ""),
        "latest_telemetry_root": str(args.latest_telemetry_root),
        "latest_telemetry_pattern": args.latest_telemetry_pattern,
        "preflight_summary_json": str(preflight_summary_json),
        "readiness_json": str(readiness_json) if readiness_json.is_file() else "",
        "preflight_status": preflight_payload.get("status", ""),
        "readiness_status": readiness_payload.get("status", ""),
        "apply_residual_allowed": bool(preflight_payload.get("apply_residual_allowed", False)),
        "recommended_residual_label": preflight_payload.get("recommended_residual_label", ""),
        "recommended_residual_profile": preflight_payload.get("recommended_residual_profile", ""),
        "operator_launch_status": preflight_payload.get("operator_launch_status", ""),
        "runtime_truth_used": runtime_truth_used,
        "coordinate_contract": (
            "latest operator telemetry gate consumes a public FRD+range telemetry CSV, lets preflight convert "
            "it to explicit stereo-center RDF selector observations, runs readiness only after truth-free "
            "residual gates pass, and preserves public FRD+range output"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "next_action": next_action,
        "preflight_summary": preflight_payload,
        "readiness_summary": readiness_payload,
    }
    write_json(args.output_dir / "operator_latest_telemetry_residual_gate_summary.json", payload)
    write_text(args.output_dir / "operator_latest_telemetry_residual_gate_summary.txt", payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
