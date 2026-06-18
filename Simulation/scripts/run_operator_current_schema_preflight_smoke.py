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

import check_operator_telemetry_residual_preflight_readiness as readiness_check  # noqa: E402
import run_operator_telemetry_residual_preflight as residual_preflight  # noqa: E402


AXIS_CONTRACT = "stereodistance_public_frd:forward,right,down,range"

SMOKE_ROWS: tuple[dict[str, object], ...] = (
    {
        "time_sec": 0.0,
        "raw_forward_m": 332.0,
        "raw_right_m": 0.5,
        "raw_down_m": 1.2,
        "corrected_forward_m": 332.4,
        "corrected_right_m": 0.6,
        "corrected_down_m": 1.3,
        "left_cx": 640.0,
        "left_cy": 360.0,
        "raw_disparity_px": 540.0,
    },
    {
        "time_sec": 0.1,
        "raw_forward_m": 360.0,
        "raw_right_m": 0.8,
        "raw_down_m": 1.4,
        "corrected_forward_m": 360.5,
        "corrected_right_m": 0.9,
        "corrected_down_m": 1.5,
        "left_cx": 650.0,
        "left_cy": 362.0,
        "raw_disparity_px": 500.0,
    },
    {
        "time_sec": 0.2,
        "raw_forward_m": 395.0,
        "raw_right_m": 1.1,
        "raw_down_m": 1.6,
        "corrected_forward_m": 395.7,
        "corrected_right_m": 1.2,
        "corrected_down_m": 1.7,
        "left_cx": 660.0,
        "left_cy": 365.0,
        "raw_disparity_px": 455.0,
    },
    {
        "time_sec": 0.3,
        "raw_forward_m": 440.0,
        "raw_right_m": 1.5,
        "raw_down_m": 1.8,
        "corrected_forward_m": 440.6,
        "corrected_right_m": 1.6,
        "corrected_down_m": 1.9,
        "left_cx": 670.0,
        "left_cy": 367.0,
        "raw_disparity_px": 410.0,
    },
    {
        "time_sec": 0.4,
        "raw_forward_m": 490.0,
        "raw_right_m": 1.8,
        "raw_down_m": 2.0,
        "corrected_forward_m": 490.8,
        "corrected_right_m": 1.9,
        "corrected_down_m": 2.1,
        "left_cx": 680.0,
        "left_cy": 370.0,
        "raw_disparity_px": 370.0,
    },
)

TELEMETRY_COLUMNS = (
    "time_sec",
    "ros_time_sec",
    "axis_contract",
    "disparity_px",
    "raw_disparity_px",
    "raw_forward_m",
    "raw_right_m",
    "raw_down_m",
    "raw_range_m",
    "estimated_forward_m",
    "estimated_right_m",
    "estimated_down_m",
    "estimated_range_m",
    "corrected_forward_m",
    "corrected_right_m",
    "corrected_down_m",
    "corrected_range_m",
    "left_cx",
    "left_cy",
    "left_valid",
    "right_valid",
    "profile_id",
    "profile_mode",
    "profile_fallback_level",
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a current-schema operator telemetry smoke CSV, then run residual preflight and readiness. "
            "The generated source telemetry is public FRD+range; selector artifacts are stereo-center RDF."
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
    parser.add_argument("--selector-model-json", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--window-size", type=int, default=5)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--expected-label", default="disparity_noise_stress")
    parser.add_argument("--require-stable-label", action="store_true")
    parser.add_argument("--min-distance-margin", type=float, default=0.5)
    return parser.parse_args(argv)


def finite_range(row: Mapping[str, object], prefix: str) -> float:
    forward = float(row[f"{prefix}_forward_m"])
    right = float(row[f"{prefix}_right_m"])
    down = float(row[f"{prefix}_down_m"])
    return (forward * forward + right * right + down * down) ** 0.5


def write_smoke_telemetry(path: Path) -> None:
    rows: list[dict[str, object]] = []
    for index, row in enumerate(SMOKE_ROWS):
        raw_range = finite_range(row, "raw")
        corrected_range = finite_range(row, "corrected")
        output = {
            "time_sec": row["time_sec"],
            "ros_time_sec": row["time_sec"],
            "axis_contract": AXIS_CONTRACT,
            "disparity_px": row["raw_disparity_px"],
            "raw_disparity_px": row["raw_disparity_px"],
            "raw_forward_m": row["raw_forward_m"],
            "raw_right_m": row["raw_right_m"],
            "raw_down_m": row["raw_down_m"],
            "raw_range_m": raw_range,
            "estimated_forward_m": row["raw_forward_m"],
            "estimated_right_m": row["raw_right_m"],
            "estimated_down_m": row["raw_down_m"],
            "estimated_range_m": raw_range,
            "corrected_forward_m": row["corrected_forward_m"],
            "corrected_right_m": row["corrected_right_m"],
            "corrected_down_m": row["corrected_down_m"],
            "corrected_range_m": corrected_range,
            "left_cx": row["left_cx"],
            "left_cy": row["left_cy"],
            "left_valid": 1,
            "right_valid": 1,
            "profile_id": "current_schema_smoke",
            "profile_mode": "public_frd_smoke",
            "profile_fallback_level": 0,
        }
        if index == 0:
            output["profile_fallback_level"] = 0
        rows.append(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(TELEMETRY_COLUMNS))
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator current-schema residual preflight smoke",
        f"status: {payload.get('status', '')}",
        f"telemetry_csv: {payload.get('telemetry_csv', '')}",
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
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    telemetry_csv = args.output_dir / "current_schema_operator_telemetry.csv"
    write_smoke_telemetry(telemetry_csv)

    preflight_args = [
        "--input-telemetry-csv",
        str(telemetry_csv),
        "--selector-model-json",
        str(args.selector_model_json),
        "--catalog-json",
        str(args.catalog_json),
        "--output-dir",
        str(args.output_dir / "preflight"),
        "--window-size",
        str(args.window_size),
        "--stride",
        str(args.stride),
        "--min-samples",
        str(args.min_samples),
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
    if args.require_stable_label:
        preflight_args.append("--require-stable-label")
    if args.min_distance_margin > 0.0:
        preflight_args.extend(["--min-distance-margin", str(args.min_distance_margin)])

    preflight_payload = residual_preflight.run(preflight_args)
    preflight_summary_json = args.output_dir / "preflight" / "operator_telemetry_residual_preflight_summary.json"
    readiness_json = args.output_dir / "operator_current_schema_preflight_readiness.json"
    readiness_txt = args.output_dir / "operator_current_schema_preflight_readiness.txt"
    readiness_payload = readiness_check.check_readiness(preflight_summary_json)
    readiness_check.write_json(readiness_json, readiness_payload)
    readiness_check.write_text(readiness_txt, readiness_payload)

    failures: list[str] = []
    if preflight_payload.get("status") != "PASS":
        failures.extend(f"preflight:{failure}" for failure in preflight_payload.get("failures", []))
    if readiness_payload.get("status") != "PASS":
        failures.extend(f"readiness:{failure}" for failure in readiness_payload.get("failures", []))
    runtime_truth_used = bool(preflight_payload.get("runtime_truth_used", False)) or bool(
        readiness_payload.get("runtime_truth_used", False)
    )
    if runtime_truth_used:
        failures.append("runtime_truth_used")

    payload = {
        "mode": "operator_current_schema_residual_preflight_smoke",
        "status": "PASS" if not failures else "FAIL",
        "telemetry_csv": str(telemetry_csv),
        "preflight_summary_json": str(preflight_summary_json),
        "readiness_json": str(readiness_json),
        "preflight_status": preflight_payload.get("status", ""),
        "readiness_status": readiness_payload.get("status", ""),
        "apply_residual_allowed": bool(preflight_payload.get("apply_residual_allowed", False)),
        "recommended_residual_label": preflight_payload.get("recommended_residual_label", ""),
        "recommended_residual_profile": preflight_payload.get("recommended_residual_profile", ""),
        "operator_launch_status": preflight_payload.get("operator_launch_status", ""),
        "runtime_truth_used": runtime_truth_used,
        "axis_contract": AXIS_CONTRACT,
        "coordinate_contract": (
            "generated telemetry source uses public FRD+range fields; residual preflight schema audit and "
            "selector replay convert public FRD to explicit stereo-center RDF observations; selected residual "
            "consumes corrected stereo-center RDF plus measured disparity; public output remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "preflight_summary": preflight_payload,
        "readiness_summary": readiness_payload,
    }
    write_json(args.output_dir / "operator_current_schema_preflight_smoke_summary.json", payload)
    write_text(args.output_dir / "operator_current_schema_preflight_smoke_summary.txt", payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
