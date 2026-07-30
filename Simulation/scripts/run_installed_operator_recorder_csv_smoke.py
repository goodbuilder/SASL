#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import audit_operator_telemetry_residual_preflight_schema as schema_audit  # noqa: E402


PUBLIC_AXIS_CONTRACT = "stereodistance_public_frd:forward,right,down,range"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Use the installed ROS overlay operator_recorder module to write one telemetry.csv row, "
            "then audit that row for truth-free residual preflight compatibility."
        )
    )
    parser.add_argument("--root-dir", type=Path, default=ROOT_DIR)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--min-valid-rows", type=int, default=1)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Installed operator recorder CSV smoke",
        f"status: {payload.get('status', '')}",
        f"telemetry_csv: {payload.get('telemetry_csv', '')}",
        f"installed_module_file: {payload.get('installed_module_file', '')}",
        f"installed_axis_contract: {payload.get('installed_axis_contract', '')}",
        f"schema_audit_status: {payload.get('schema_audit_status', '')}",
        f"schema_audit_valid_row_count: {payload.get('schema_audit_valid_row_count', 0)}",
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


def run_installed_recorder_probe(*, root_dir: Path, output_dir: Path) -> subprocess.CompletedProcess[str]:
    probe_code = r'''
import csv
import json
import time
from pathlib import Path

import stereo_ground_sim.operator_recorder as recorder


class FakeClockNow:
    @property
    def nanoseconds(self):
        return 123456789


class FakeClock:
    def now(self):
        return FakeClockNow()


class FakeMsg:
    def __init__(self, data):
        self.data = data


class FakePointStamped:
    def __init__(self, forward, right, down):
        self.point = type("Point", (), {"x": forward, "y": right, "z": down})()
        self.header = type(
            "Header",
            (),
            {"stamp": type("Stamp", (), {"sec": 123, "nanosec": 456000000})()},
        )()


output_dir = Path(__import__("os").environ["OUTPUT_DIR"])
output_dir.mkdir(parents=True, exist_ok=True)
telemetry = output_dir / "telemetry.csv"
instance = object.__new__(recorder.OperatorRecorder)
instance.latest = {}
instance.last_stamp = {}
instance.start_monotonic = time.monotonic()
instance.get_clock = lambda: FakeClock()
with telemetry.open("w", newline="", encoding="utf-8") as handle:
    instance.csv_file = handle
    instance.csv_writer = csv.DictWriter(handle, fieldnames=recorder.TELEMETRY_FIELDS)
    instance.csv_writer.writeheader()
    instance._estimated_position_cb(FakeMsg([1000.0, 7.0, 4.0, 1000.0325]))
    instance._corrected_position_cb(FakeMsg([999.0, 6.5, 3.5, 999.0274]))
    instance._ground_truth_position_cb(FakeMsg([1000.0, 7.0, 4.0, 1000.0325]))
    instance._gps_baseline_position_cb("raw_gps_baseline", FakePointStamped(1002.0, 6.0, 5.0))
    instance._gps_baseline_position_cb("global_position_baseline", FakePointStamped(999.0, 8.0, 4.5))
    instance._float_cb(FakeMsg(200.0), "disparity_px", "/stereo/disparity")
    instance._detections_cb(FakeMsg([1, 690, 380, 710, 400, 0.9, 700, 390, 1, 680, 380, 700, 400, 0.8, 690, 390]))
    instance._profile_status_cb(FakeMsg(json.dumps({"profile_id": "installed_smoke", "profile_mode": "sim3", "fallback_level": 0})))
    instance._write_csv_row()
payload = {
    "telemetry_csv": str(telemetry),
    "module_file": str(Path(recorder.__file__).resolve()),
    "axis_contract": recorder.AXIS_CONTRACT,
    "estimated_position_fields": list(recorder.ESTIMATED_POSITION_FIELDS),
    "corrected_position_fields": list(recorder.CORRECTED_POSITION_FIELDS),
    "detection_fields": list(recorder.DETECTION_FIELDS),
}
print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
'''
    script = "set -euo pipefail\nset +u\nsource /opt/ros/humble/setup.bash\nsource \"$ROOT_DIR/install/setup.bash\"\nset -u\npython3 -c \"$PROBE_CODE\"\n"
    env = dict(os.environ)
    env["ROOT_DIR"] = str(root_dir)
    env["OUTPUT_DIR"] = str(output_dir)
    env["PROBE_CODE"] = probe_code
    return subprocess.run(
        ["bash", "-lc", script],
        cwd=root_dir,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def read_first_row(path: Path) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    return rows[0] if rows else {}


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    root_dir = args.root_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    probe = run_installed_recorder_probe(root_dir=root_dir, output_dir=output_dir)
    failures: list[str] = []
    probe_payload: dict[str, object] = {}
    if probe.returncode != 0:
        failures.append(f"installed_probe:returncode:{probe.returncode}")
    else:
        try:
            probe_payload = json.loads(probe.stdout.splitlines()[-1])
        except (IndexError, json.JSONDecodeError):
            failures.append("installed_probe:invalid_json")

    telemetry_csv = Path(str(probe_payload.get("telemetry_csv", output_dir / "telemetry.csv")))
    audit_payload: dict[str, object] = {
        "status": "SKIPPED",
        "valid_row_count": 0,
        "runtime_truth_used": False,
        "failures": ["installed_probe_failed"],
    }
    first_row: dict[str, str] = {}
    if not failures and telemetry_csv.is_file():
        audit_payload = schema_audit.audit_schema(input_csv=telemetry_csv, min_valid_rows=args.min_valid_rows)
        schema_audit.write_json(output_dir / "operator_telemetry_schema_audit.json", audit_payload)
        schema_audit.write_text(output_dir / "operator_telemetry_schema_audit.txt", audit_payload)
        first_row = read_first_row(telemetry_csv)
    elif not telemetry_csv.is_file():
        failures.append("telemetry_csv:missing")

    installed_axis_contract = str(probe_payload.get("axis_contract", ""))
    if installed_axis_contract != PUBLIC_AXIS_CONTRACT:
        failures.append("installed_axis_contract:mismatch")
    if audit_payload.get("status") != "PASS":
        failures.extend(f"schema_audit:{failure}" for failure in audit_payload.get("failures", []))
    runtime_truth_used = bool(audit_payload.get("runtime_truth_used", False))
    if runtime_truth_used:
        failures.append("runtime_truth_used")

    payload = {
        "mode": "installed_operator_recorder_csv_smoke",
        "status": "PASS" if not failures else "FAIL",
        "telemetry_csv": str(telemetry_csv),
        "installed_module_file": probe_payload.get("module_file", ""),
        "installed_axis_contract": installed_axis_contract,
        "schema_audit_json": str(output_dir / "operator_telemetry_schema_audit.json"),
        "schema_audit_status": audit_payload.get("status", ""),
        "schema_audit_valid_row_count": audit_payload.get("valid_row_count", 0),
        "first_row_has_axis_contract": "axis_contract" in first_row,
        "first_row_raw_forward_m": first_row.get("raw_forward_m", ""),
        "first_row_corrected_forward_m": first_row.get("corrected_forward_m", ""),
        "first_row_raw_disparity_px": first_row.get("raw_disparity_px", ""),
        "first_row_raw_gps_baseline_forward_m": first_row.get("raw_gps_baseline_forward_m", ""),
        "first_row_global_position_baseline_forward_m": first_row.get(
            "global_position_baseline_forward_m", ""
        ),
        "first_row_raw_minus_global_baseline_delta_m": first_row.get(
            "raw_minus_global_baseline_delta_m", ""
        ),
        "probe_returncode": probe.returncode,
        "probe_stderr": probe.stderr[-4000:],
        "runtime_truth_used": runtime_truth_used,
        "coordinate_contract": (
            "installed operator_recorder writes public FRD+range telemetry; residual preflight schema audit "
            "then verifies it can be converted to stereo-center RDF selector observations without runtime truth"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "installed_probe": probe_payload,
        "schema_audit_summary": audit_payload,
    }
    write_json(output_dir / "installed_operator_recorder_csv_smoke_summary.json", payload)
    write_text(output_dir / "installed_operator_recorder_csv_smoke_summary.txt", payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
