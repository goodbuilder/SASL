#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import convert_stereo_bbox_observations_to_fusion_input as bbox_converter  # noqa: E402


TRUTH_COLUMN_PREFIXES = ("truth_", "px4_", "rtk_")
TRUTH_COLUMN_NAMES = {
    "sim_truth_forward_m",
    "sim_truth_right_m",
    "sim_truth_down_m",
    "sim_truth_range_m",
}


def truth_columns(columns: list[str]) -> list[str]:
    return sorted(
        column
        for column in columns
        if column.startswith(TRUTH_COLUMN_PREFIXES) or column in TRUTH_COLUMN_NAMES
    )


def non_truth_columns(columns: list[str]) -> list[str]:
    dropped = set(truth_columns(columns))
    return [column for column in columns if column not in dropped]


def extract_rows(
    rows: list[Mapping[str, object]],
    *,
    columns: list[str],
    group_field: str = "group_id",
    burst_field: str = "burst_id",
) -> tuple[list[dict[str, object]], list[str], list[str]]:
    dropped_truth = truth_columns(columns)
    runtime_columns = non_truth_columns(columns)
    output_rows, failures = bbox_converter.convert_rows(
        rows,
        columns=runtime_columns,
        group_field=group_field,
        burst_field=burst_field,
    )
    for row in output_rows:
        row["input_source"] = "calibration_csv_runtime_observation_extract"
        row["calibration_truth_columns_dropped"] = ",".join(dropped_truth)
        row["runtime_truth_used"] = False
    return output_rows, failures, dropped_truth


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Calibration CSV to observation-fusion runtime input extraction",
        f"input_csv: {summary['input_csv']}",
        f"output_csv: {summary['output_csv']}",
        f"status: {summary['status']}",
        f"input_row_count: {summary['input_row_count']}",
        f"output_row_count: {summary['output_row_count']}",
        f"dropped_truth_column_count: {summary['dropped_truth_column_count']}",
        f"failure_count: {summary['failure_count']}",
        "runtime_truth_used: False",
        "Coordinate contract: calibration truth columns are dropped; runtime output is image u/v + positive disparity px for stereo-center RDF reconstruction.",
    ]
    for column in summary.get("dropped_truth_columns", []):
        lines.append(f"dropped_truth_column: {column}")
    for failure in summary.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract runtime observation-fusion input from calibration recorder CSV.")
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--group-field", default="group_id")
    parser.add_argument("--burst-field", default="burst_id")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    columns, input_rows = bbox_converter.read_rows(args.input_csv)
    output_rows, failures, dropped_truth = extract_rows(
        input_rows,
        columns=columns,
        group_field=args.group_field,
        burst_field=args.burst_field,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_csv = args.output_dir / "observation_fusion_calibration_runtime_input.csv"
    bbox_converter.write_csv(output_csv, output_rows)
    summary = {
        "input_csv": str(args.input_csv),
        "output_csv": str(output_csv),
        "status": "PASS" if output_rows and not failures else "FAIL",
        "input_row_count": len(input_rows),
        "output_row_count": len(output_rows),
        "dropped_truth_column_count": len(dropped_truth),
        "dropped_truth_columns": dropped_truth,
        "failure_count": len(failures),
        "failures": failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }
    (args.output_dir / "observation_fusion_calibration_extract_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_text(args.output_dir / "observation_fusion_calibration_extract_summary.txt", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
