#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_operator_recording_preflight_readiness as recording_readiness  # noqa: E402
import check_operator_recording_install_overlay_readiness as install_overlay_readiness  # noqa: E402
import run_installed_operator_recorder_csv_smoke as installed_csv_smoke  # noqa: E402
import run_operator_current_schema_preflight_smoke as current_schema_smoke  # noqa: E402
import run_operator_latest_telemetry_residual_gate as latest_gate  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the complete operator residual enablement gate suite: recording-path readiness, "
            "current-schema software smoke, and latest real-telemetry live gate."
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
    parser.add_argument("--latest-telemetry-pattern", default="**/telemetry*.csv")
    parser.add_argument("--selector-model-json", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--window-size", type=int, default=21)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--expected-label", default="")
    parser.add_argument("--current-schema-expected-label", default="disparity_noise_stress")
    parser.add_argument("--require-stable-label", action="store_true")
    parser.add_argument("--min-distance-margin", type=float, default=0.0)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator residual enablement suite",
        f"status: {payload.get('status', '')}",
        f"recording_readiness_status: {payload.get('recording_readiness_status', '')}",
        f"install_overlay_readiness_status: {payload.get('install_overlay_readiness_status', '')}",
        f"installed_csv_smoke_status: {payload.get('installed_csv_smoke_status', '')}",
        f"current_schema_smoke_status: {payload.get('current_schema_smoke_status', '')}",
        f"latest_gate_status: {payload.get('latest_gate_status', '')}",
        f"latest_input_telemetry_csv: {payload.get('latest_input_telemetry_csv', '')}",
        f"apply_residual_allowed: {payload.get('apply_residual_allowed', False)}",
        f"recommended_residual_label: {payload.get('recommended_residual_label', '')}",
        f"operator_launch_status: {payload.get('operator_launch_status', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"recording_readiness_json: {payload.get('recording_readiness_json', '')}",
        f"install_overlay_readiness_json: {payload.get('install_overlay_readiness_json', '')}",
        f"installed_csv_smoke_json: {payload.get('installed_csv_smoke_json', '')}",
        f"current_schema_smoke_json: {payload.get('current_schema_smoke_json', '')}",
        f"latest_gate_json: {payload.get('latest_gate_json', '')}",
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


def append_source_args(args: argparse.Namespace, output: list[str]) -> None:
    if args.package_dir is not None:
        output.extend(["--package-dir", str(args.package_dir)])
    if args.profile_json is not None:
        output.extend(["--profile-json", str(args.profile_json)])
    if args.runtime_selector_json is not None:
        output.extend(["--runtime-selector-json", str(args.runtime_selector_json)])
    if args.deployment_label:
        output.extend(["--deployment-label", args.deployment_label])
    if args.scenario_label:
        output.extend(["--scenario-label", args.scenario_label])
    if args.scenario_name:
        output.extend(["--scenario-name", args.scenario_name])
    if args.observation_fusion_manifest is not None:
        output.extend(["--observation-fusion-manifest", str(args.observation_fusion_manifest)])
    if args.observation_fusion_tier:
        output.extend(["--observation-fusion-tier", args.observation_fusion_tier])


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    recording_dir = args.output_dir / "recording_preflight_readiness"
    install_overlay_dir = args.output_dir / "recording_install_overlay_readiness"
    installed_csv_smoke_dir = args.output_dir / "installed_operator_recorder_csv_smoke"
    smoke_dir = args.output_dir / "current_schema_preflight_smoke"
    latest_dir = args.output_dir / "latest_telemetry_residual_gate"

    recording_payload = recording_readiness.check_readiness(ROOT_DIR)
    recording_readiness.write_json(recording_dir / "operator_recording_preflight_readiness.json", recording_payload)
    recording_readiness.write_text(recording_dir / "operator_recording_preflight_readiness.txt", recording_payload)

    install_payload = install_overlay_readiness.check_readiness(ROOT_DIR)
    install_overlay_readiness.write_json(
        install_overlay_dir / "operator_recording_install_overlay_readiness.json",
        install_payload,
    )
    install_overlay_readiness.write_text(
        install_overlay_dir / "operator_recording_install_overlay_readiness.txt",
        install_payload,
    )

    installed_csv_payload = installed_csv_smoke.run(["--output-dir", str(installed_csv_smoke_dir)])

    smoke_args = [
        "--selector-model-json",
        str(args.selector_model_json),
        "--catalog-json",
        str(args.catalog_json),
        "--output-dir",
        str(smoke_dir),
        "--window-size",
        str(max(5, args.min_samples)),
        "--stride",
        "1",
        "--min-samples",
        str(args.min_samples),
    ]
    append_source_args(args, smoke_args)
    if args.current_schema_expected_label:
        smoke_args.extend(["--expected-label", args.current_schema_expected_label])
    if args.require_stable_label:
        smoke_args.append("--require-stable-label")
    if args.min_distance_margin > 0.0:
        smoke_args.extend(["--min-distance-margin", str(args.min_distance_margin)])
    smoke_payload = current_schema_smoke.run(smoke_args)

    gate_args = [
        "--latest-telemetry-root",
        str(args.latest_telemetry_root),
        "--latest-telemetry-pattern",
        args.latest_telemetry_pattern,
        "--selector-model-json",
        str(args.selector_model_json),
        "--catalog-json",
        str(args.catalog_json),
        "--output-dir",
        str(latest_dir),
        "--window-size",
        str(args.window_size),
        "--stride",
        str(args.stride),
        "--min-samples",
        str(args.min_samples),
    ]
    append_source_args(args, gate_args)
    if args.expected_label:
        gate_args.extend(["--expected-label", args.expected_label])
    if args.require_stable_label:
        gate_args.append("--require-stable-label")
    if args.min_distance_margin > 0.0:
        gate_args.extend(["--min-distance-margin", str(args.min_distance_margin)])
    latest_payload = latest_gate.run(gate_args)

    failures: list[str] = []
    if recording_payload.get("status") != "PASS":
        failures.extend(f"recording_readiness:{failure}" for failure in recording_payload.get("failures", []))
    if install_payload.get("status") != "PASS":
        failures.extend(f"install_overlay_readiness:{failure}" for failure in install_payload.get("failures", []))
    if installed_csv_payload.get("status") != "PASS":
        failures.extend(f"installed_csv_smoke:{failure}" for failure in installed_csv_payload.get("failures", []))
    if smoke_payload.get("status") != "PASS":
        failures.extend(f"current_schema_smoke:{failure}" for failure in smoke_payload.get("failures", []))
    if latest_payload.get("status") != "PASS":
        failures.extend(f"latest_gate:{failure}" for failure in latest_payload.get("failures", []))
    runtime_truth_used = any(
        bool(payload.get("runtime_truth_used", False))
        for payload in (recording_payload, install_payload, installed_csv_payload, smoke_payload, latest_payload)
        if isinstance(payload, Mapping)
    )
    if runtime_truth_used:
        failures.append("runtime_truth_used")

    next_action = ""
    if latest_payload.get("status") != "PASS":
        next_action = (
            "The software path is ready, but live enablement still needs a fresh record-lite telemetry "
            "with the current public FRD+range schema; record it and rerun this suite."
        )

    payload = {
        "mode": "operator_residual_enablement_suite",
        "status": "PASS" if not failures else "FAIL",
        "recording_readiness_status": recording_payload.get("status", ""),
        "install_overlay_readiness_status": install_payload.get("status", ""),
        "installed_csv_smoke_status": installed_csv_payload.get("status", ""),
        "current_schema_smoke_status": smoke_payload.get("status", ""),
        "latest_gate_status": latest_payload.get("status", ""),
        "latest_input_telemetry_csv": latest_payload.get("input_telemetry_csv", ""),
        "apply_residual_allowed": bool(latest_payload.get("apply_residual_allowed", False)),
        "recommended_residual_label": latest_payload.get("recommended_residual_label", ""),
        "recommended_residual_profile": latest_payload.get("recommended_residual_profile", ""),
        "operator_launch_status": latest_payload.get("operator_launch_status", ""),
        "runtime_truth_used": runtime_truth_used,
        "recording_readiness_json": str(recording_dir / "operator_recording_preflight_readiness.json"),
        "install_overlay_readiness_json": str(
            install_overlay_dir / "operator_recording_install_overlay_readiness.json"
        ),
        "installed_csv_smoke_json": str(
            installed_csv_smoke_dir / "installed_operator_recorder_csv_smoke_summary.json"
        ),
        "current_schema_smoke_json": str(smoke_dir / "operator_current_schema_preflight_smoke_summary.json"),
        "latest_gate_json": str(latest_dir / "operator_latest_telemetry_residual_gate_summary.json"),
        "coordinate_contract": (
            "suite checks recording output as public FRD+range, validates the current-schema public-FRD "
            "software path into stereo-center RDF selector observations, then gates latest live telemetry "
            "without using PX4/RTK/NED truth at runtime"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "next_action": next_action,
        "recording_readiness_summary": recording_payload,
        "install_overlay_readiness_summary": install_payload,
        "installed_csv_smoke_summary": installed_csv_payload,
        "current_schema_smoke_summary": smoke_payload,
        "latest_gate_summary": latest_payload,
    }
    write_json(args.output_dir / "operator_residual_enablement_suite_summary.json", payload)
    write_text(args.output_dir / "operator_residual_enablement_suite_summary.txt", payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
