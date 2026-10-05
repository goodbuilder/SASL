#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping


FORBIDDEN_TRUTH_COLUMNS = (
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "truth_north_m",
    "truth_east_m",
    "truth_down_m",
    "px4_north_m",
    "px4_east_m",
    "px4_down_m",
    "rtk_north_m",
    "rtk_east_m",
    "rtk_down_m",
)


def finite_float(value: object) -> float | None:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def first_finite(row: Mapping[str, object], names: tuple[str, ...]) -> float | None:
    for name in names:
        value = finite_float(row.get(name))
        if value is not None:
            return value
    return None


def bbox_center(row: Mapping[str, object], prefix: str) -> tuple[float | None, float | None]:
    direct_x = first_finite(row, (f"{prefix}_bbox_cx_px", f"{prefix}_cx_px", f"{prefix}_u_px"))
    direct_y = first_finite(row, (f"{prefix}_bbox_cy_px", f"{prefix}_cy_px", f"{prefix}_v_px"))
    if direct_x is not None and direct_y is not None:
        return direct_x, direct_y
    xmin = first_finite(row, (f"{prefix}_bbox_xmin_px", f"{prefix}_xmin_px"))
    xmax = first_finite(row, (f"{prefix}_bbox_xmax_px", f"{prefix}_xmax_px"))
    ymin = first_finite(row, (f"{prefix}_bbox_ymin_px", f"{prefix}_ymin_px"))
    ymax = first_finite(row, (f"{prefix}_bbox_ymax_px", f"{prefix}_ymax_px"))
    if None in (xmin, xmax, ymin, ymax):
        return None, None
    assert xmin is not None and xmax is not None and ymin is not None and ymax is not None
    return (xmin + xmax) * 0.5, (ymin + ymax) * 0.5


def convert_rows(
    rows: list[Mapping[str, object]],
    *,
    columns: list[str],
    group_field: str = "group_id",
    burst_field: str = "burst_id",
) -> tuple[list[dict[str, object]], list[str]]:
    failures: list[str] = []
    forbidden_present = sorted(set(columns).intersection(FORBIDDEN_TRUTH_COLUMNS))
    for column in forbidden_present:
        failures.append(f"forbidden_truth_column:{column}")
    output_rows: list[dict[str, object]] = []
    for row_index, row in enumerate(rows):
        left_u, left_v = bbox_center(row, "left")
        right_u, right_v = bbox_center(row, "right")
        if None in (left_u, left_v, right_u):
            failures.append(f"row{row_index}:missing_left_or_right_bbox_center")
            continue
        assert left_u is not None and left_v is not None and right_u is not None
        measured_disparity = first_finite(row, ("measured_disparity_px", "raw_disparity_px"))
        if measured_disparity is None:
            measured_disparity = left_u - right_u
        if measured_disparity <= 0.0:
            failures.append(f"row{row_index}:measured_disparity_px_not_positive")
            continue
        group_id = str(row.get(group_field, "default")).strip() or "default"
        output: dict[str, object] = {
            "group_id": group_id,
            "left_u_px": left_u,
            "left_v_px": left_v,
            "measured_disparity_px": measured_disparity,
            "runtime_truth_used": False,
            "input_source": "stereo_bbox_center",
            "image_contract": "left/right bbox centers in image pixels; disparity is left_u_px - right_u_px unless explicitly measured",
        }
        if burst_field in row and str(row.get(burst_field, "")).strip():
            output["burst_id"] = str(row[burst_field]).strip()
        if right_v is not None:
            output["right_v_px"] = right_v
            output["bbox_vertical_center_delta_px"] = left_v - right_v
        output_rows.append(output)
    return output_rows, failures


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def write_csv(path: Path, rows: list[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Stereo bbox observation to fusion input conversion",
        f"input_csv: {summary['input_csv']}",
        f"output_csv: {summary['output_csv']}",
        f"status: {summary['status']}",
        f"input_row_count: {summary['input_row_count']}",
        f"output_row_count: {summary['output_row_count']}",
        f"failure_count: {summary['failure_count']}",
        "runtime_truth_used: False",
        "Coordinate contract: left/right bbox image centers -> left image u/v + positive measured disparity px; ready for stereo-center RDF reconstruction.",
    ]
    for failure in summary.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert stereo bbox CSV rows to observation-fusion runtime input CSV.")
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--group-field", default="group_id")
    parser.add_argument("--burst-field", default="burst_id")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    columns, input_rows = read_rows(args.input_csv)
    output_rows, failures = convert_rows(
        input_rows,
        columns=columns,
        group_field=args.group_field,
        burst_field=args.burst_field,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_csv = args.output_dir / "observation_fusion_bbox_converted_input.csv"
    write_csv(output_csv, output_rows)
    summary = {
        "input_csv": str(args.input_csv),
        "output_csv": str(output_csv),
        "status": "PASS" if output_rows and not failures else "FAIL",
        "input_row_count": len(input_rows),
        "output_row_count": len(output_rows),
        "failure_count": len(failures),
        "failures": failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }
    (args.output_dir / "observation_fusion_bbox_conversion_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_text(args.output_dir / "observation_fusion_bbox_conversion_summary.txt", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
