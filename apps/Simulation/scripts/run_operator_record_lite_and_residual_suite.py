#!/usr/bin/env python3
from __future__ import annotations

import argparse
from datetime import datetime
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import run_operator_residual_enablement_suite as enablement_suite  # noqa: E402
import run_operator_current_schema_preflight_smoke as current_schema_smoke  # noqa: E402
import run_external_bbox_topic_preflight as external_bbox_preflight  # noqa: E402

TELEMETRY_FRESH_TOLERANCE_SEC = 2.0


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Optionally record a fresh operator --record-lite telemetry session, then run the residual "
            "enablement suite on the latest telemetry."
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
    parser.add_argument("--latest-telemetry-root", type=Path, default=ROOT_DIR / "bags")
    parser.add_argument("--latest-telemetry-pattern", default="**/telemetry.csv")
    parser.add_argument("--record-dir", type=Path)
    parser.add_argument("--record-seconds", type=float, default=0.0)
    parser.add_argument("--skip-record", action="store_true")
    parser.add_argument(
        "--synthetic-record",
        action="store_true",
        help=(
            "Write a fresh current-schema public FRD+range telemetry.csv to record-dir instead of launching "
            "the operator. This exercises the non-print residual enablement path when ROS/Gazebo/PX4 are absent."
        ),
    )
    parser.add_argument("--operator-arg", action="append", default=[])
    parser.add_argument("--external-bbox-publisher", action="store_true")
    parser.add_argument("--external-bbox-topic", default="/stereo/external_detections")
    parser.add_argument("--external-bbox-hz", type=float, default=20.0)
    parser.add_argument("--external-bbox-forward-m", type=float, default=1000.0)
    parser.add_argument("--external-bbox-end-forward-m", type=float, default=1000.0)
    parser.add_argument("--external-bbox-ramp-duration-s", type=float, default=0.0)
    parser.add_argument("--external-bbox-right-m", type=float, default=0.0)
    parser.add_argument("--external-bbox-down-m", type=float, default=0.0)
    parser.add_argument("--external-bbox-start-delay-s", type=float, default=4.0)
    parser.add_argument(
        "--external-bbox-preflight",
        action="store_true",
        help=(
            "Run the external BBox topic contract preflight before record-lite/residual suite. "
            "Use --external-bbox-preflight-input-csv for offline samples or let it sample the topic."
        ),
    )
    parser.add_argument("--external-bbox-preflight-input-csv", type=Path)
    parser.add_argument("--external-bbox-preflight-topic", default="")
    parser.add_argument("--external-bbox-preflight-sample-seconds", type=float, default=5.0)
    parser.add_argument("--external-bbox-preflight-max-samples", type=int, default=200)
    parser.add_argument("--external-bbox-preflight-width-px", type=int, default=1280)
    parser.add_argument("--external-bbox-preflight-height-px", type=int, default=720)
    parser.add_argument("--external-bbox-preflight-min-disparity-px", type=float, default=0.5)
    parser.add_argument("--external-bbox-preflight-min-valid-pairs", type=int, default=1)
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
    parser.add_argument("--print-only", action="store_true")
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator record-lite and residual suite",
        f"status: {payload.get('status', '')}",
        f"record_status: {payload.get('record_status', '')}",
        f"suite_status: {payload.get('suite_status', '')}",
        f"record_dir: {payload.get('record_dir', '')}",
        f"record_telemetry_csv: {payload.get('record_telemetry_csv', '')}",
        f"record_telemetry_exists: {payload.get('record_telemetry_exists', False)}",
        f"record_telemetry_mtime: {payload.get('record_telemetry_mtime', '')}",
        f"record_telemetry_fresh: {payload.get('record_telemetry_fresh', False)}",
        f"suite_used_record_telemetry: {payload.get('suite_used_record_telemetry', False)}",
        f"latest_telemetry_root: {payload.get('latest_telemetry_root', '')}",
        f"record_command: {' '.join(str(v) for v in payload.get('record_command', []))}",
        f"sidecar_command: {' '.join(str(v) for v in payload.get('sidecar_command', []))}",
        f"sidecar_returncode: {payload.get('sidecar_returncode', '')}",
        f"external_bbox_preflight_status: {payload.get('external_bbox_preflight_status', '')}",
        f"external_bbox_preflight_summary_json: {payload.get('external_bbox_preflight_summary_json', '')}",
        f"suite_summary_json: {payload.get('suite_summary_json', '')}",
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


def command_path(path: Path) -> str:
    return str(path.expanduser().resolve()) if path.exists() else str(path)


def append_source_args(args: argparse.Namespace, output: list[str]) -> None:
    if args.package_dir is not None:
        output.extend(["--package-dir", command_path(args.package_dir)])
    if args.profile_json is not None:
        output.extend(["--profile-json", command_path(args.profile_json)])
    if args.runtime_selector_json is not None:
        output.extend(["--selector-json", command_path(args.runtime_selector_json)])
    if args.deployment_label:
        output.extend(["--deployment-label", args.deployment_label])
    if args.scenario_label:
        output.extend(["--scenario-label", args.scenario_label])
    if args.scenario_name:
        output.extend(["--scenario-name", args.scenario_name])
    if args.observation_fusion_manifest is not None:
        output.extend(["--observation-fusion-manifest", command_path(args.observation_fusion_manifest)])
    if args.observation_fusion_tier:
        output.extend(["--observation-fusion-tier", args.observation_fusion_tier])


def append_suite_source_args(args: argparse.Namespace, output: list[str]) -> None:
    if args.package_dir is not None:
        output.extend(["--package-dir", command_path(args.package_dir)])
    if args.profile_json is not None:
        output.extend(["--profile-json", command_path(args.profile_json)])
    if args.runtime_selector_json is not None:
        output.extend(["--runtime-selector-json", command_path(args.runtime_selector_json)])
    if args.deployment_label:
        output.extend(["--deployment-label", args.deployment_label])
    if args.scenario_label:
        output.extend(["--scenario-label", args.scenario_label])
    if args.scenario_name:
        output.extend(["--scenario-name", args.scenario_name])
    if args.observation_fusion_manifest is not None:
        output.extend(["--observation-fusion-manifest", command_path(args.observation_fusion_manifest)])
    if args.observation_fusion_tier:
        output.extend(["--observation-fusion-tier", args.observation_fusion_tier])


def default_record_dir(latest_root: Path) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return latest_root / f"operator_{stamp}_lite"


def build_record_command(args: argparse.Namespace, record_dir: Path) -> list[str]:
    command = [str(ROOT_DIR / "run_operator_view_with_calibration_profile.sh")]
    append_source_args(args, command)
    command.extend(["--", "--record-lite", str(record_dir)])
    command.extend(str(value) for value in args.operator_arg)
    return command


def build_external_bbox_publisher_command(args: argparse.Namespace) -> list[str]:
    return [
        str(ROOT_DIR / ".venv" / "bin" / "python"),
        "-m",
        "stereo_ground_sim.external_bbox_publisher",
        "--topic",
        str(args.external_bbox_topic),
        "--hz",
        str(args.external_bbox_hz),
        "--forward-m",
        str(args.external_bbox_forward_m),
        "--end-forward-m",
        str(args.external_bbox_end_forward_m),
        "--ramp-duration-s",
        str(args.external_bbox_ramp_duration_s),
        "--right-m",
        str(args.external_bbox_right_m),
        "--down-m",
        str(args.external_bbox_down_m),
    ]


def build_suite_args(args: argparse.Namespace, suite_dir: Path) -> list[str]:
    suite_args = [
        "--latest-telemetry-root",
        str(args.latest_telemetry_root),
        "--latest-telemetry-pattern",
        args.latest_telemetry_pattern,
        "--selector-model-json",
        str(args.selector_model_json),
        "--catalog-json",
        str(args.catalog_json),
        "--output-dir",
        str(suite_dir),
        "--window-size",
        str(args.window_size),
        "--stride",
        str(args.stride),
        "--min-samples",
        str(args.min_samples),
    ]
    append_suite_source_args(args, suite_args)
    if args.expected_label:
        suite_args.extend(["--expected-label", args.expected_label])
    if args.current_schema_expected_label:
        suite_args.extend(["--current-schema-expected-label", args.current_schema_expected_label])
    if args.require_stable_label:
        suite_args.append("--require-stable-label")
    if args.min_distance_margin > 0.0:
        suite_args.extend(["--min-distance-margin", str(args.min_distance_margin)])
    return suite_args


def run_external_bbox_preflight(args: argparse.Namespace) -> dict[str, object]:
    topic = args.external_bbox_preflight_topic or args.external_bbox_topic
    return external_bbox_preflight.run(
        output_dir=args.output_dir / "external_bbox_topic_preflight",
        input_csv=args.external_bbox_preflight_input_csv,
        topic=topic,
        sample_seconds=float(args.external_bbox_preflight_sample_seconds),
        max_samples=int(args.external_bbox_preflight_max_samples),
        width_px=int(args.external_bbox_preflight_width_px),
        height_px=int(args.external_bbox_preflight_height_px),
        min_disparity_px=float(args.external_bbox_preflight_min_disparity_px),
        min_valid_pairs=int(args.external_bbox_preflight_min_valid_pairs),
    )


def pythonpath_with_package(env: Mapping[str, str]) -> str:
    parts = [str(ROOT_DIR / "stereo_ground_sim")]
    existing = env.get("PYTHONPATH", "")
    if existing:
        parts.append(existing)
    return os.pathsep.join(parts)


def run_record_command(
    command: Sequence[str],
    *,
    record_seconds: float,
    sidecar_command: Sequence[str] | None = None,
    sidecar_start_delay_s: float = 0.0,
) -> tuple[str, int | None, str, str, str, str, int | None]:
    if record_seconds <= 0.0:
        return "SKIPPED", None, "", "record_seconds_not_positive", "", "", None
    process = subprocess.Popen(command, cwd=ROOT_DIR, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    sidecar = None
    sidecar_stdout = ""
    sidecar_stderr = ""
    if sidecar_command is not None:
        if sidecar_start_delay_s > 0.0:
            time.sleep(float(sidecar_start_delay_s))
        env = dict(os.environ)
        env["PYTHONPATH"] = pythonpath_with_package(env)
        sidecar = subprocess.Popen(
            sidecar_command,
            cwd=ROOT_DIR,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )
    try:
        stdout, stderr = process.communicate(timeout=record_seconds)
        status = "EXITED"
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            stdout, stderr = process.communicate(timeout=10.0)
            status = "TIMEOUT_TERMINATED"
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, stderr = process.communicate()
            status = "TIMEOUT_KILLED"
    if sidecar is not None:
        sidecar.terminate()
        try:
            sidecar_stdout, sidecar_stderr = sidecar.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            sidecar.kill()
            sidecar_stdout, sidecar_stderr = sidecar.communicate()
    return status, process.returncode, stdout, stderr, sidecar_stdout, sidecar_stderr, (
        None if sidecar is None else sidecar.returncode
    )


def write_synthetic_record(record_dir: Path) -> Path:
    telemetry_csv = record_dir / "telemetry.csv"
    current_schema_smoke.write_smoke_telemetry(telemetry_csv)
    return telemetry_csv


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    args.latest_telemetry_root = args.latest_telemetry_root.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.latest_telemetry_root.mkdir(parents=True, exist_ok=True)
    record_dir = (args.record_dir.expanduser().resolve() if args.record_dir is not None else default_record_dir(args.latest_telemetry_root))
    suite_dir = args.output_dir / "enablement_suite"
    record_command = build_record_command(args, record_dir)
    sidecar_command = build_external_bbox_publisher_command(args) if args.external_bbox_publisher else None
    suite_args = build_suite_args(args, suite_dir)
    external_bbox_preflight_summary_json = args.output_dir / "external_bbox_topic_preflight" / "external_bbox_topic_preflight_summary.json"

    if args.print_only:
        record_telemetry_csv = record_dir / "telemetry.csv"
        record_telemetry_mtime = record_telemetry_csv.stat().st_mtime if record_telemetry_csv.is_file() else None
        payload = {
            "mode": "operator_record_lite_and_residual_suite",
            "status": "PASS",
            "record_status": "PRINT_ONLY",
            "suite_status": "PRINT_ONLY",
            "record_dir": str(record_dir),
            "record_telemetry_csv": str(record_telemetry_csv),
            "record_telemetry_exists": record_telemetry_csv.is_file(),
            "record_telemetry_mtime": record_telemetry_mtime,
            "record_telemetry_fresh": False,
            "suite_used_record_telemetry": False,
            "latest_telemetry_root": str(args.latest_telemetry_root),
            "record_command": record_command,
            "sidecar_command": list(sidecar_command or []),
            "external_bbox_preflight_enabled": bool(args.external_bbox_preflight),
            "external_bbox_preflight_status": "PRINT_ONLY" if args.external_bbox_preflight else "SKIPPED",
            "external_bbox_preflight_summary_json": str(external_bbox_preflight_summary_json),
            "suite_args": suite_args,
            "suite_summary_json": str(suite_dir / "operator_residual_enablement_suite_summary.json"),
            "runtime_truth_used": False,
            "coordinate_contract": (
                "record command writes public FRD+range telemetry; enablement suite later converts that "
                "telemetry to stereo-center RDF selector observations and never uses PX4/RTK/NED truth at runtime"
            ),
            "failure_count": 0,
            "failures": [],
        }
        write_json(args.output_dir / "operator_record_lite_and_residual_suite_summary.json", payload)
        write_text(args.output_dir / "operator_record_lite_and_residual_suite_summary.txt", payload)
        return payload

    record_status = "SKIPPED"
    record_returncode: int | None = None
    record_stdout = ""
    record_stderr = ""
    sidecar_stdout = ""
    sidecar_stderr = ""
    sidecar_returncode: int | None = None
    external_bbox_preflight_payload: dict[str, object] | None = None
    external_bbox_preflight_status = "SKIPPED"
    preflight_failures: list[str] = []
    record_started_at = time.time()
    if args.external_bbox_preflight:
        external_bbox_preflight_payload = run_external_bbox_preflight(args)
        external_bbox_preflight_status = str(external_bbox_preflight_payload.get("status", ""))
        if external_bbox_preflight_status != "PASS":
            preflight_failures.extend(
                f"external_bbox_preflight:{failure}"
                for failure in external_bbox_preflight_payload.get("failures", [])
            )
            if not preflight_failures:
                preflight_failures.append(f"external_bbox_preflight:status:{external_bbox_preflight_status}")
    if preflight_failures:
        record_telemetry_csv = record_dir / "telemetry.csv"
        payload = {
            "mode": "operator_record_lite_and_residual_suite",
            "status": "FAIL",
            "record_status": "SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT",
            "record_returncode": None,
            "synthetic_record": bool(args.synthetic_record),
            "record_dir": str(record_dir),
            "record_telemetry_csv": str(record_telemetry_csv),
            "record_telemetry_exists": record_telemetry_csv.is_file(),
            "record_telemetry_mtime": record_telemetry_csv.stat().st_mtime if record_telemetry_csv.is_file() else None,
            "record_started_at": record_started_at,
            "record_telemetry_fresh_tolerance_sec": TELEMETRY_FRESH_TOLERANCE_SEC,
            "record_telemetry_fresh": False,
            "suite_used_record_telemetry": False,
            "latest_telemetry_root": str(args.latest_telemetry_root),
            "record_command": record_command,
            "sidecar_command": list(sidecar_command or []),
            "sidecar_returncode": None,
            "record_stdout": "",
            "record_stderr": "",
            "sidecar_stdout": "",
            "sidecar_stderr": "",
            "external_bbox_preflight_enabled": True,
            "external_bbox_preflight_status": external_bbox_preflight_status,
            "external_bbox_preflight_summary_json": str(external_bbox_preflight_summary_json),
            "external_bbox_preflight_summary": external_bbox_preflight_payload,
            "suite_status": "SKIPPED_BY_EXTERNAL_BBOX_PREFLIGHT",
            "suite_summary_json": str(suite_dir / "operator_residual_enablement_suite_summary.json"),
            "runtime_truth_used": bool(
                external_bbox_preflight_payload.get("runtime_truth_used", False)
                if external_bbox_preflight_payload is not None
                else False
            ),
            "coordinate_contract": (
                "external BBox preflight validates image-pixel BBox topic data before record; record command "
                "would write public FRD+range telemetry; enablement suite would convert telemetry to "
                "stereo-center RDF selector observations without runtime truth"
            ),
            "failure_count": len(preflight_failures),
            "failures": preflight_failures,
        }
        write_json(args.output_dir / "operator_record_lite_and_residual_suite_summary.json", payload)
        write_text(args.output_dir / "operator_record_lite_and_residual_suite_summary.txt", payload)
        return payload
    if not args.skip_record:
        if args.synthetic_record:
            synthetic_csv = write_synthetic_record(record_dir)
            record_status = "SYNTHETIC"
            record_returncode = 0
            record_stdout = f"wrote synthetic current-schema telemetry: {synthetic_csv}"
            record_stderr = ""
        else:
            (
                record_status,
                record_returncode,
                record_stdout,
                record_stderr,
                sidecar_stdout,
                sidecar_stderr,
                sidecar_returncode,
            ) = run_record_command(
                record_command,
                record_seconds=float(args.record_seconds),
                sidecar_command=sidecar_command,
                sidecar_start_delay_s=float(args.external_bbox_start_delay_s),
            )

    suite_payload = enablement_suite.run(suite_args)
    record_telemetry_csv = record_dir / "telemetry.csv"
    record_telemetry_exists = record_telemetry_csv.is_file()
    record_telemetry_mtime = record_telemetry_csv.stat().st_mtime if record_telemetry_exists else None
    record_telemetry_fresh = bool(
        args.skip_record
        or (
            record_telemetry_mtime is not None
            and record_telemetry_mtime >= record_started_at - TELEMETRY_FRESH_TOLERANCE_SEC
        )
    )
    latest_input_telemetry = Path(str(suite_payload.get("latest_input_telemetry_csv", "")))
    try:
        suite_used_record_telemetry = (
            record_telemetry_exists
            and latest_input_telemetry.expanduser().resolve() == record_telemetry_csv.expanduser().resolve()
        )
    except OSError:
        suite_used_record_telemetry = False
    failures: list[str] = []
    if not args.skip_record:
        if record_status == "SKIPPED":
            failures.append("record:not_executed")
        if not record_telemetry_exists:
            failures.append("record:telemetry_missing")
        if record_telemetry_exists and not record_telemetry_fresh:
            failures.append("record:telemetry_stale")
        if record_telemetry_exists and not suite_used_record_telemetry:
            failures.append("record:telemetry_not_selected_by_suite")
    if record_status in {"EXITED"} and record_returncode not in (0, None):
        failures.append(f"record:returncode:{record_returncode}")
    if sidecar_command is not None and sidecar_returncode not in (0, None):
        failures.append(f"sidecar:returncode:{sidecar_returncode}")
    if args.external_bbox_preflight and external_bbox_preflight_status != "PASS":
        failures.append(f"external_bbox_preflight:status:{external_bbox_preflight_status}")
    if suite_payload.get("status") != "PASS":
        failures.extend(f"suite:{failure}" for failure in suite_payload.get("failures", []))
    runtime_truth_used = bool(suite_payload.get("runtime_truth_used", False))
    if runtime_truth_used:
        failures.append("runtime_truth_used")
    payload = {
        "mode": "operator_record_lite_and_residual_suite",
        "status": "PASS" if not failures else "FAIL",
        "record_status": record_status,
        "record_returncode": record_returncode,
        "synthetic_record": bool(args.synthetic_record),
        "record_dir": str(record_dir),
        "record_telemetry_csv": str(record_telemetry_csv),
        "record_telemetry_exists": record_telemetry_exists,
        "record_telemetry_mtime": record_telemetry_mtime,
        "record_started_at": record_started_at,
        "record_telemetry_fresh_tolerance_sec": TELEMETRY_FRESH_TOLERANCE_SEC,
        "record_telemetry_fresh": record_telemetry_fresh,
        "suite_used_record_telemetry": suite_used_record_telemetry,
        "latest_telemetry_root": str(args.latest_telemetry_root),
        "record_command": record_command,
        "sidecar_command": list(sidecar_command or []),
        "sidecar_returncode": sidecar_returncode,
        "record_stdout": record_stdout[-4000:],
        "record_stderr": record_stderr[-4000:],
        "sidecar_stdout": sidecar_stdout[-2000:],
        "sidecar_stderr": sidecar_stderr[-2000:],
        "external_bbox_preflight_enabled": bool(args.external_bbox_preflight),
        "external_bbox_preflight_status": external_bbox_preflight_status,
        "external_bbox_preflight_summary_json": str(external_bbox_preflight_summary_json),
        "external_bbox_preflight_summary": external_bbox_preflight_payload,
        "suite_status": suite_payload.get("status", ""),
        "suite_summary_json": str(suite_dir / "operator_residual_enablement_suite_summary.json"),
        "runtime_truth_used": runtime_truth_used,
        "coordinate_contract": (
            "record command or synthetic record writes public FRD+range telemetry; enablement suite converts "
            "telemetry to stereo-center RDF selector observations and gates residual application without "
            "runtime truth"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "suite_summary": suite_payload,
    }
    write_json(args.output_dir / "operator_record_lite_and_residual_suite_summary.json", payload)
    write_text(args.output_dir / "operator_record_lite_and_residual_suite_summary.txt", payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
