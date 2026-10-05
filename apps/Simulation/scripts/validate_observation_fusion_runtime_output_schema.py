#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
FORBIDDEN_TRUTH_FIELDS = (
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "truth_north_m",
    "truth_east_m",
    "truth_down_ned_m",
)
REQUIRED_FIELDS = (
    "raw_right_m",
    "raw_down_m",
    "raw_forward_m",
    "raw_range_m",
    "raw_public_forward_m",
    "raw_public_right_m",
    "raw_public_down_m",
    "raw_public_range_m",
    "corrected_right_m",
    "corrected_down_m",
    "corrected_forward_m",
    "corrected_range_m",
    "corrected_public_forward_m",
    "corrected_public_right_m",
    "corrected_public_down_m",
    "corrected_public_range_m",
    "rdf_axis_contract",
    "public_topic_contract",
)


def finite_float(value: object) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return float("nan")
    return parsed if math.isfinite(parsed) else float("nan")


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def close_enough(actual: float, expected: float, tolerance: float) -> bool:
    return math.isfinite(actual) and math.isfinite(expected) and abs(actual - expected) <= tolerance


def range3(right_m: float, down_m: float, forward_m: float) -> float:
    return math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)


def validate_row(row: Mapping[str, object], *, row_index: int, tolerance_m: float) -> list[str]:
    failures: list[str] = []
    if str(row.get("rdf_axis_contract", "")) != RDF_AXIS_CONTRACT:
        failures.append(f"row{row_index}:rdf_axis_contract_mismatch")
    if str(row.get("public_topic_contract", "")) != PUBLIC_TOPIC_CONTRACT:
        failures.append(f"row{row_index}:public_topic_contract_mismatch")
    raw_right = finite_float(row.get("raw_right_m"))
    raw_down = finite_float(row.get("raw_down_m"))
    raw_forward = finite_float(row.get("raw_forward_m"))
    raw_range = finite_float(row.get("raw_range_m"))
    corrected_right = finite_float(row.get("corrected_right_m"))
    corrected_down = finite_float(row.get("corrected_down_m"))
    corrected_forward = finite_float(row.get("corrected_forward_m"))
    corrected_range = finite_float(row.get("corrected_range_m"))
    checks = (
        ("raw_public_forward_m", raw_forward),
        ("raw_public_right_m", raw_right),
        ("raw_public_down_m", raw_down),
        ("raw_public_range_m", range3(raw_right, raw_down, raw_forward)),
        ("corrected_public_forward_m", corrected_forward),
        ("corrected_public_right_m", corrected_right),
        ("corrected_public_down_m", corrected_down),
        ("corrected_public_range_m", range3(corrected_right, corrected_down, corrected_forward)),
        ("raw_range_m", range3(raw_right, raw_down, raw_forward)),
        ("corrected_range_m", range3(corrected_right, corrected_down, corrected_forward)),
    )
    for field, expected in checks:
        actual = finite_float(row.get(field))
        if not close_enough(actual, expected, tolerance_m):
            failures.append(f"row{row_index}:{field}_axis_or_range_mismatch")
    if str(row.get("runtime_truth_used", "")).strip().lower() not in {"false", "0", ""}:
        failures.append(f"row{row_index}:runtime_truth_used_not_false")
    return failures


def validate_csv(path: Path, *, tolerance_m: float) -> dict[str, object]:
    fieldnames, rows = read_rows(path)
    missing = [field for field in REQUIRED_FIELDS if field not in fieldnames]
    forbidden_present = [field for field in FORBIDDEN_TRUTH_FIELDS if field in fieldnames]
    failures = [f"missing_field:{field}" for field in missing]
    failures.extend(f"forbidden_truth_field:{field}" for field in forbidden_present)
    if not rows:
        failures.append("no_rows")
    for index, row in enumerate(rows):
        failures.extend(validate_row(row, row_index=index, tolerance_m=tolerance_m))
    return {
        "input_csv": str(path),
        "status": "PASS" if not failures else "FAIL",
        "row_count": len(rows),
        "failure_count": len(failures),
        "failures": failures,
        "runtime_truth_used": False,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate observation-fusion runtime output CSV axis/schema contract.")
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--tolerance-m", type=float, default=1.0e-6)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = validate_csv(args.input_csv, tolerance_m=args.tolerance_m)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "observation_fusion_runtime_output_schema_validation.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    lines = [
        "Observation fusion runtime output schema validation",
        f"input_csv: {args.input_csv}",
        f"status: {summary['status']}",
        f"row_count: {summary['row_count']}",
        f"failure_count: {summary['failure_count']}",
        "Coordinate contract: internal RDF right/down/forward; public aliases FRD+range forward/right/down/range.",
    ]
    for failure in summary["failures"]:
        lines.append(f"- {failure}")
    (args.output_dir / "observation_fusion_runtime_output_schema_validation.txt").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
