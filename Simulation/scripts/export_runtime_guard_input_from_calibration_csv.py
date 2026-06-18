#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
OUTPUT_FIELDS = (
    "sample_id",
    "split",
    "raw_right_m",
    "raw_down_m",
    "raw_forward_m",
    "raw_range_m",
    "left_u_px",
    "left_v_px",
    "measured_disparity_px",
)
REQUIRED_NUMERIC_FIELDS = (
    "raw_right_m",
    "raw_down_m",
    "raw_forward_m",
    "raw_range_m",
    "measured_disparity_px",
)
FORBIDDEN_RUNTIME_TOKENS = (
    "truth",
    "gt_",
    "px4",
    "rtk",
    "ned",
    "validation_range_rmse",
    "target_max_abs_error",
    "corrected_range_error",
    "raw_range_error",
)


def finite_float(value: object) -> float | None:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def has_forbidden_runtime_column(columns: Sequence[str]) -> bool:
    joined = " ".join(column.lower() for column in columns)
    return any(token in joined for token in FORBIDDEN_RUNTIME_TOKENS)


def export_rows(
    rows: Sequence[Mapping[str, object]],
    *,
    split: str = "",
    max_rows: int = 0,
) -> tuple[list[dict[str, object]], list[str]]:
    failures: list[str] = []
    output_rows: list[dict[str, object]] = []
    for row_index, row in enumerate(rows):
        if split and str(row.get("split", "")) != split:
            continue
        output = {field: row.get(field, "") for field in OUTPUT_FIELDS}
        missing = [field for field in REQUIRED_NUMERIC_FIELDS if finite_float(output.get(field)) is None]
        if missing:
            failures.append(f"row{row_index}:missing_or_nonfinite:{','.join(missing)}")
            continue
        disparity = finite_float(output["measured_disparity_px"])
        if disparity is None or disparity <= 0.0:
            failures.append(f"row{row_index}:measured_disparity_px_not_positive")
            continue
        if not str(output.get("sample_id", "")).strip():
            output["sample_id"] = str(row_index)
        output_rows.append(output)
        if max_rows > 0 and len(output_rows) >= max_rows:
            break
    if not output_rows:
        failures.append("no_runtime_rows_exported")
    return output_rows, failures


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(OUTPUT_FIELDS))
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Runtime guard input export from calibration CSV",
        f"status: {summary.get('status', '')}",
        f"input_csv: {summary.get('input_csv', '')}",
        f"output_csv: {summary.get('output_csv', '')}",
        f"input_row_count: {summary.get('input_row_count', '')}",
        f"output_row_count: {summary.get('output_row_count', '')}",
        f"split: {summary.get('split', '')}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', False)}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        f"failure_count: {summary.get('failure_count', 0)}",
    ]
    for failure in summary.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_runtime_guard_input(
    *,
    input_csv: Path,
    output_csv: Path,
    output_json: Path | None = None,
    output_txt: Path | None = None,
    split: str = "",
    max_rows: int = 0,
) -> dict[str, object]:
    columns, rows = read_rows(input_csv)
    missing_columns = [field for field in OUTPUT_FIELDS if field not in columns]
    exportable_rows, failures = export_rows(rows, split=split, max_rows=max_rows)
    for field in missing_columns:
        if field in REQUIRED_NUMERIC_FIELDS:
            failures.insert(0, f"missing_required_column:{field}")
    write_csv(output_csv, exportable_rows)
    output_columns = list(OUTPUT_FIELDS)
    if has_forbidden_runtime_column(output_columns):
        failures.append("output_schema_contains_forbidden_runtime_column")
    summary = {
        "mode": "runtime_guard_input_export_from_calibration_csv",
        "status": "PASS" if exportable_rows and not failures else "FAIL",
        "input_csv": str(input_csv),
        "output_csv": str(output_csv),
        "input_row_count": len(rows),
        "output_row_count": len(exportable_rows),
        "split": split,
        "max_rows": max_rows,
        "output_fields": ",".join(output_columns),
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": (
            "runtime guard input contains only sample_id/split, stereo-center RDF(right,down,forward), "
            "raw range, left image center, and measured disparity; PX4/RTK/NED truth is excluded"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }
    if output_json is not None:
        write_json(output_json, summary)
    if output_txt is not None:
        write_text(output_txt, summary)
    return summary


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a truth-free runtime-guard input CSV from a calibration CSV using the CSV parser. "
            "This avoids comma-containing coordinate-contract fields corrupting column order."
        )
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    parser.add_argument("--split", default="")
    parser.add_argument("--max-rows", type=int, default=0)
    return parser.parse_args(argv)


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return export_runtime_guard_input(
        input_csv=args.input_csv,
        output_csv=args.output_csv,
        output_json=args.output_json,
        output_txt=args.output_txt,
        split=args.split,
        max_rows=args.max_rows,
    )


def main() -> None:
    summary = run()
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
