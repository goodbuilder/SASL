#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
ESTIMATED_TO_TRUTH_SCRIPT = ROOT_DIR / "scripts" / "run_estimated_to_truth_calibration_exp.py"
REPLAY_SCRIPT = ROOT_DIR / "scripts" / "stereodistance_profile_replay.py"
TRUTH_CONVERTER_SCRIPT = ROOT_DIR / "scripts" / "convert_truth_source_to_stereo_rdf.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the Calibration.md two-flight profile loop from recorded CSVs: "
            "fit on calibration flight, replay on disjoint validation flight, "
            "and report nominal vs corrected error."
        )
    )
    parser.add_argument("--calibration-csv", type=Path, required=True)
    parser.add_argument("--validation-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--truth-transform-json", type=Path)
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--min-target-count", type=int, default=3)
    parser.add_argument("--strict-preflight", action="store_true")
    return parser.parse_args()


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def load_summary_csv(path: Path) -> dict[str, str]:
    rows = read_csv_rows(path)
    if not rows:
        raise ValueError(f"summary CSV is empty: {path}")
    return rows[0]


def write_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def finite_float(value: Any, default: float = math.nan) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def finite_int(value: Any, default: int = 0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def run_command(cmd: Sequence[str], *, cwd: Path) -> None:
    print(" ".join(str(part) for part in cmd), flush=True)
    subprocess.run(list(cmd), cwd=str(cwd), check=True)


def maybe_convert_validation_csv(args: argparse.Namespace) -> tuple[Path, dict[str, Any]]:
    if args.truth_transform_json is None:
        return args.validation_csv, {}
    conversion_dir = args.output_dir / "validation_truth_rdf_conversion"
    converted_csv = conversion_dir / "validation_with_truth_rdf.csv"
    summary_json = conversion_dir / "truth_rdf_conversion_summary.json"
    summary_txt = conversion_dir / "truth_rdf_conversion_summary.txt"
    cmd = [
        sys.executable,
        str(TRUTH_CONVERTER_SCRIPT),
        "--input-csv",
        str(args.validation_csv),
        "--output-csv",
        str(converted_csv),
        "--transform-json",
        str(args.truth_transform_json),
        "--summary-json",
        str(summary_json),
        "--summary-txt",
        str(summary_txt),
        "--strict",
    ]
    run_command(cmd, cwd=ROOT_DIR.parent)
    return converted_csv, json.loads(summary_json.read_text(encoding="utf-8"))


def build_profile(args: argparse.Namespace) -> Path:
    profile_dir = args.output_dir / "calibration_profile"
    cmd = [
        sys.executable,
        str(ESTIMATED_TO_TRUTH_SCRIPT),
        "--mode",
        "csv",
        "--input-csv",
        str(args.calibration_csv),
        "--output-dir",
        str(profile_dir),
        "--target-range-m",
        str(args.target_range_m),
        "--target-window-m",
        str(args.target_window_m),
        "--min-target-count",
        str(args.min_target_count),
    ]
    if args.truth_transform_json is not None:
        cmd.extend(["--truth-transform-json", str(args.truth_transform_json)])
    if args.strict_preflight:
        cmd.append("--strict-preflight")
    run_command(cmd, cwd=ROOT_DIR.parent)
    profile_json = profile_dir / "sim3_runtime_profile.json"
    if not profile_json.exists():
        raise FileNotFoundError(f"runtime profile was not exported: {profile_json}")
    return profile_json


def replay_validation(args: argparse.Namespace, profile_json: Path, validation_csv: Path) -> dict[str, str]:
    replay_dir = args.output_dir / "validation_replay"
    cmd = [
        sys.executable,
        str(REPLAY_SCRIPT),
        "--input-csv",
        str(validation_csv),
        "--profile-json",
        str(profile_json),
        "--output-dir",
        str(replay_dir),
        "--target-range-m",
        str(args.target_range_m),
        "--target-window-m",
        str(args.target_window_m),
    ]
    run_command(cmd, cwd=ROOT_DIR.parent)
    return load_summary_csv(replay_dir / "profile_replay_summary.csv")


def pass_fail(summary: Mapping[str, Any], *, min_target_count: int) -> tuple[bool, list[str]]:
    failures: list[str] = []
    if not finite_float(summary.get("validation_range_rmse_corrected_m")) < finite_float(
        summary.get("validation_range_rmse_raw_m")
    ):
        failures.append("range_rmse_not_improved")
    if not finite_float(summary.get("validation_position_rmse_corrected_m")) < finite_float(
        summary.get("validation_position_rmse_raw_m")
    ):
        failures.append("position_rmse_not_improved")
    if not finite_float(summary.get("validation_position_median_corrected_m")) < finite_float(
        summary.get("validation_position_median_raw_m")
    ):
        failures.append("position_median_not_improved")
    if not finite_float(summary.get("validation_position_p90_corrected_m")) < finite_float(
        summary.get("validation_position_p90_raw_m")
    ):
        failures.append("position_p90_not_improved")
    for axis in ("right", "down", "forward"):
        if not finite_float(summary.get(f"validation_{axis}_rmse_corrected_m")) <= finite_float(
            summary.get(f"validation_{axis}_rmse_raw_m")
        ):
            failures.append(f"{axis}_axis_regressed")
    if finite_int(summary.get("target_window_count")) < min_target_count:
        failures.append("target_window_count_below_minimum")
    if bool(summary.get("runtime_truth_used")):
        failures.append("runtime_truth_used")
    return not failures, failures


def build_loop_summary(
    *,
    args: argparse.Namespace,
    profile_json: Path,
    validation_csv: Path,
    validation_conversion_summary: Mapping[str, Any],
    replay_summary: Mapping[str, Any],
) -> dict[str, Any]:
    passed, failures = pass_fail(replay_summary, min_target_count=args.min_target_count)
    return {
        "contract": "Calibration.md two-flight calibration profile generation and validation loop",
        "status": "PASS" if passed else "FAIL",
        "failure_reasons": failures,
        "calibration_csv": str(args.calibration_csv),
        "validation_csv": str(args.validation_csv),
        "validation_pipeline_csv": str(validation_csv),
        "runtime_profile_json": str(profile_json),
        "truth_source": "gazebo_truth_pose/stereo RDF fields in CSV, or explicit T_source_to_stereo_rdf when provided",
        "runtime_inputs": [
            "nominal raw RDF/FRD fields",
            "measured_disparity_px",
            "left/right bbox centers when profile requires them",
        ],
        "forbidden_runtime_inputs": [
            "gazebo_truth_pose",
            "px4_gps_reference",
            "truth_right_m",
            "truth_down_m",
            "truth_forward_m",
        ],
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "alias_contracts_explicit": True,
        "validation_conversion_summary": dict(validation_conversion_summary),
        "validation_replay_summary": dict(replay_summary),
    }


def main() -> int:
    args = parse_args()
    args.output_dir = args.output_dir.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    profile_json = build_profile(args)
    validation_csv, validation_conversion_summary = maybe_convert_validation_csv(args)
    replay_summary = replay_validation(args, profile_json, validation_csv)
    loop_summary = build_loop_summary(
        args=args,
        profile_json=profile_json,
        validation_csv=validation_csv,
        validation_conversion_summary=validation_conversion_summary,
        replay_summary=replay_summary,
    )
    write_json(args.output_dir / "two_flight_validation_summary.json", loop_summary)
    print(f"two-flight validation status: {loop_summary['status']}")
    print(f"summary: {args.output_dir / 'two_flight_validation_summary.json'}")
    return 0 if loop_summary["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
