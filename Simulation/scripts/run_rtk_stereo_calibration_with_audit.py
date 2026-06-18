#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import audit_rtk_stereo_calibration_outputs as output_audit  # noqa: E402
import export_rtk_stereo_calibration_output_inventory as output_inventory  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description=(
            "Run RTK stereo calibration pipeline and immediately audit the final artifacts. "
            "Unknown arguments are passed through to rtk_stereo_calibration_pipeline.py."
        )
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--audit-output-dir", type=Path, default=None)
    parser.add_argument("--target-tolerance-m", type=float, default=10.0)
    parser.add_argument("--python", type=Path, default=Path(sys.executable), help="Python executable used for the pipeline subprocess.")
    parser.add_argument("--dry-run", action="store_true", help="Print the pipeline command without running it.")
    return parser.parse_known_args(argv)


def pipeline_command(args: argparse.Namespace, passthrough_args: Sequence[str]) -> list[str]:
    return [
        str(args.python),
        str(SCRIPT_DIR / "rtk_stereo_calibration_pipeline.py"),
        "--output-dir",
        str(args.output_dir),
        *passthrough_args,
    ]


def run_pipeline(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.setdefault("MPLCONFIGDIR", "/tmp")
    return subprocess.run(
        list(command),
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,
    )


def write_summary(path: Path, payload: dict[str, object]) -> None:
    lines = [
        "RTK stereo calibration with output audit",
        f"status: {payload['status']}",
        f"output_dir: {payload['output_dir']}",
        f"pipeline_returncode: {payload['pipeline_returncode']}",
        f"audit_status: {payload.get('audit_status', '')}",
        f"recommendation: {payload.get('recommendation', '')}",
        f"method_count: {payload.get('method_count', '')}",
        f"target_1852m_method_count: {payload.get('target_1852m_method_count', '')}",
        f"validation_prediction_row_count: {payload.get('validation_prediction_row_count', '')}",
        f"policy_count: {payload.get('policy_count', '')}",
        f"train_count: {payload.get('train_count', '')}",
        f"validation_count: {payload.get('validation_count', '')}",
        f"selected_profile_path: {payload.get('selected_profile_path', '')}",
        f"selected_profile_mode: {payload.get('selected_profile_mode', '')}",
        f"selected_profile_load_status: {payload.get('selected_profile_load_status', '')}",
        f"selected_profile_apply_status: {payload.get('selected_profile_apply_status', '')}",
        f"top_level_profile_equivalence_status: {payload.get('top_level_profile_equivalence_status', '')}",
        f"runtime_profile_count: {payload.get('runtime_profile_count', '')}",
        f"runtime_profile_load_pass_count: {payload.get('runtime_profile_load_pass_count', '')}",
        f"runtime_profile_missing_methods: {json.dumps(payload.get('runtime_profile_missing_methods', []), sort_keys=True)}",
        f"policy_profile_count: {payload.get('policy_profile_count', '')}",
        f"policy_profile_load_pass_count: {payload.get('policy_profile_load_pass_count', '')}",
        f"plot_dimensions: {json.dumps(payload.get('plot_dimensions', {}), sort_keys=True)}",
        f"deliverable_evidence_items: {json.dumps(payload.get('deliverable_evidence_items', []), sort_keys=True)}",
        f"inventory_status: {payload.get('inventory_status', '')}",
        f"inventory_file_count: {payload.get('inventory_file_count', '')}",
        f"inventory_total_size_bytes: {payload.get('inventory_total_size_bytes', '')}",
        "runtime_truth_used: False",
        "Coordinate contract: calibration truth may be PX4/RTK NED, runtime profiles consume stereo-center RDF(right,down,forward), public StereoDistance alias is FRD+range.",
    ]
    if payload.get("pipeline_stdout"):
        lines.append("pipeline_stdout:")
        lines.extend(str(payload["pipeline_stdout"]).splitlines())
    if payload.get("pipeline_stderr"):
        lines.append("pipeline_stderr:")
        lines.extend(str(payload["pipeline_stderr"]).splitlines())
    for failure in payload.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args, passthrough_args = parse_args(argv)
    command = pipeline_command(args, passthrough_args)
    if args.dry_run:
        return {
            "status": "DRY_RUN",
            "output_dir": str(args.output_dir),
            "pipeline_command": command,
            "pipeline_returncode": None,
            "audit_status": "",
            "runtime_truth_used": False,
            "coordinate_contracts_explicit": True,
        }

    completed = run_pipeline(command)
    if completed.returncode != 0:
        payload = {
            "status": "FAIL",
            "output_dir": str(args.output_dir),
            "pipeline_command": command,
            "pipeline_returncode": completed.returncode,
            "pipeline_stdout": completed.stdout,
            "pipeline_stderr": completed.stderr,
            "audit_status": "SKIPPED",
            "failure_count": 1,
            "failures": ["pipeline:nonzero_returncode"],
            "runtime_truth_used": False,
            "coordinate_contracts_explicit": True,
        }
        summary_dir = args.audit_output_dir if args.audit_output_dir is not None else args.output_dir / "output_audit"
        summary_dir.mkdir(parents=True, exist_ok=True)
        (summary_dir / "rtk_stereo_calibration_with_audit_summary.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        write_summary(summary_dir / "rtk_stereo_calibration_with_audit_summary.txt", payload)
        return payload

    audit = output_audit.audit_outputs(args.output_dir, target_tolerance_m=args.target_tolerance_m)
    audit_dir = args.audit_output_dir if args.audit_output_dir is not None else args.output_dir / "output_audit"
    output_audit.write_json(audit_dir / "rtk_stereo_calibration_output_audit.json", audit)
    output_audit.write_text(audit_dir / "rtk_stereo_calibration_output_audit.txt", audit)
    inventory = output_inventory.export_inventory(args.output_dir, args.output_dir / "inventory")
    payload = {
        "status": "PASS" if audit["status"] == "PASS" else "FAIL",
        "output_dir": str(args.output_dir),
        "pipeline_command": command,
        "pipeline_returncode": completed.returncode,
        "pipeline_stdout": completed.stdout,
        "pipeline_stderr": completed.stderr,
        "audit_status": audit["status"],
        "recommendation": audit.get("recommendation", ""),
        "method_count": audit.get("method_count", 0),
        "target_1852m_method_count": audit.get("target_1852m_method_count", 0),
        "validation_prediction_row_count": audit.get("validation_prediction_row_count", 0),
        "policy_count": audit.get("policy_count", 0),
        "train_count": audit.get("train_count", 0),
        "validation_count": audit.get("validation_count", 0),
        "selected_profile_path": audit.get("selected_profile_path", ""),
        "selected_profile_mode": audit.get("selected_profile_mode", ""),
        "selected_profile_load_status": audit.get("selected_profile_load_status", ""),
        "selected_profile_apply_status": audit.get("selected_profile_apply_status", ""),
        "top_level_profile_equivalence_status": audit.get("top_level_profile_equivalence_status", ""),
        "runtime_profile_count": audit.get("runtime_profile_count", 0),
        "runtime_profile_load_pass_count": audit.get("runtime_profile_load_pass_count", 0),
        "runtime_profile_missing_methods": audit.get("runtime_profile_missing_methods", []),
        "policy_profile_count": audit.get("policy_profile_count", 0),
        "policy_profile_load_pass_count": audit.get("policy_profile_load_pass_count", 0),
        "plot_dimensions": audit.get("plot_dimensions", {}),
        "deliverable_evidence_items": sorted(
            (audit.get("deliverable_evidence_matrix", {}) or {}).keys()
            if isinstance(audit.get("deliverable_evidence_matrix", {}), dict)
            else []
        ),
        "inventory_status": inventory.get("status", ""),
        "inventory_file_count": inventory.get("file_count", 0),
        "inventory_total_size_bytes": inventory.get("total_size_bytes", 0),
        "failure_count": audit.get("failure_count", 0),
        "failures": audit.get("failures", []),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }
    (audit_dir / "rtk_stereo_calibration_with_audit_summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_summary(audit_dir / "rtk_stereo_calibration_with_audit_summary.txt", payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] not in ("PASS", "DRY_RUN"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
