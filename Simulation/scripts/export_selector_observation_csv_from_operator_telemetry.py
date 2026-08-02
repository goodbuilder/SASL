#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


INPUT_AXIS_CONTRACT = "operator_telemetry_public_frd:forward,right,down,range"
OUTPUT_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
MAX_REPORTED_FAILURES = 200

TRUTH_TOKENS = ("truth", "drone_", "actual", "target_distance", "distance_error", "px4", "rtk", "ned")
OUTPUT_COLUMNS = (
    "left_u_px",
    "left_v_px",
    "measured_disparity_px",
    "fused_disparity_px",
    "nominal_pose_right_m",
    "nominal_pose_down_m",
    "nominal_pose_forward_m",
    "observation_right_m",
    "observation_down_m",
    "observation_forward_m",
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a truth-free residual-selector observation CSV from operator_recorder telemetry.csv. "
            "The source telemetry uses public FRD fields; the output uses explicit stereo-center RDF columns."
        )
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--summary-txt", type=Path)
    parser.add_argument("--allow-missing-corrected", action="store_true")
    return parser.parse_args(argv)


def finite_float(value: object) -> float | None:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), [dict(row) for row in reader]


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(OUTPUT_COLUMNS))
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def truth_like_columns(columns: Sequence[str]) -> list[str]:
    return sorted(column for column in columns if any(token in column.lower() for token in TRUTH_TOKENS))


def pick_float(row: Mapping[str, object], names: Sequence[str]) -> float | None:
    for name in names:
        if name not in row:
            continue
        value = finite_float(row.get(name))
        if value is not None:
            return value
    return None


def convert_row(
    row: Mapping[str, object],
    *,
    allow_missing_corrected: bool,
) -> tuple[dict[str, object] | None, list[str]]:
    failures: list[str] = []
    left_u = pick_float(row, ("left_cx", "left_cx_px", "left_u_px"))
    left_v = pick_float(row, ("left_cy", "left_cy_px", "left_v_px"))
    disparity = pick_float(row, ("raw_disparity_px", "disparity_px", "bbox_adjustment_raw_disparity_px"))
    nominal_forward = pick_float(row, ("nominal_pose_forward_m",))
    nominal_right = pick_float(row, ("nominal_pose_right_m",))
    nominal_down = pick_float(row, ("nominal_pose_down_m",))
    corrected_forward = pick_float(row, ("corrected_forward_m",))
    corrected_right = pick_float(row, ("corrected_right_m",))
    corrected_down = pick_float(row, ("corrected_down_m",))

    required = {
        "left_u_px": left_u,
        "left_v_px": left_v,
        "measured_disparity_px": disparity,
        "nominal_pose_forward_m": nominal_forward,
        "nominal_pose_right_m": nominal_right,
        "nominal_pose_down_m": nominal_down,
    }
    for name, value in required.items():
        if value is None:
            failures.append(f"missing_or_nonfinite:{name}")
    if disparity is not None and disparity <= 0.0:
        failures.append("measured_disparity_px_not_positive")
    if corrected_forward is None or corrected_right is None or corrected_down is None:
        if allow_missing_corrected:
            corrected_forward = nominal_forward
            corrected_right = nominal_right
            corrected_down = nominal_down
        else:
            for name, value in (
                ("observation_forward_m", corrected_forward),
                ("observation_right_m", corrected_right),
                ("observation_down_m", corrected_down),
            ):
                if value is None:
                    failures.append(f"missing_or_nonfinite:{name}")
    if failures:
        return None, failures
    return (
        {
            "left_u_px": left_u,
            "left_v_px": left_v,
            "measured_disparity_px": disparity,
            "fused_disparity_px": disparity,
            # Source telemetry field names are public FRD; output names are explicit RDF components.
            "nominal_pose_right_m": nominal_right,
            "nominal_pose_down_m": nominal_down,
            "nominal_pose_forward_m": nominal_forward,
            "observation_right_m": corrected_right,
            "observation_down_m": corrected_down,
            "observation_forward_m": corrected_forward,
        },
        [],
    )


def export_observation_csv(
    *,
    input_csv: Path,
    output_csv: Path,
    allow_missing_corrected: bool = False,
) -> dict[str, object]:
    columns, rows = read_rows(input_csv)
    output_rows: list[dict[str, object]] = []
    failures: list[str] = []
    skipped_row_failures: list[str] = []
    skipped_invalid_row_count = 0
    dropped_truth_columns = truth_like_columns(columns)
    for index, row in enumerate(rows):
        converted, row_failures = convert_row(row, allow_missing_corrected=allow_missing_corrected)
        if converted is None:
            skipped_invalid_row_count += 1
            for failure in row_failures:
                if len(skipped_row_failures) < MAX_REPORTED_FAILURES:
                    skipped_row_failures.append(f"row{index}:{failure}")
            continue
        output_rows.append(converted)
    if not rows:
        failures.append("input_csv:no_rows")
    if not output_rows:
        failures.append("output_csv:no_valid_rows")
        remaining_slots = max(0, MAX_REPORTED_FAILURES - len(failures))
        failures.extend(skipped_row_failures[:remaining_slots])
    write_csv(output_csv, output_rows)
    summary = {
        "mode": "operator_telemetry_to_selector_observation_csv",
        "status": "PASS" if not failures else "FAIL",
        "input_csv": str(input_csv),
        "output_csv": str(output_csv),
        "input_row_count": len(rows),
        "output_row_count": len(output_rows),
        "skipped_invalid_row_count": skipped_invalid_row_count,
        "dropped_truth_column_count": len(dropped_truth_columns),
        "dropped_truth_columns": dropped_truth_columns,
        "allow_missing_corrected": bool(allow_missing_corrected),
        "runtime_truth_used": False,
        "input_axis_contract": INPUT_AXIS_CONTRACT,
        "axis_contract": OUTPUT_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "coordinate_contract": (
            "operator telemetry source fields are public FRD(forward,right,down,range); exporter writes explicit "
            "selector RDF columns raw_right/raw_down/raw_forward and observation_right/observation_down/"
            "observation_forward; truth/PX4/RTK/NED fields are dropped and never emitted"
        ),
        "failure_count": len(failures),
        "skipped_row_failure_count": len(skipped_row_failures),
        "skipped_row_failures": skipped_row_failures,
        "reported_failure_count": len(failures),
        "failure_report_truncated": len(failures) >= MAX_REPORTED_FAILURES,
        "max_reported_failures": MAX_REPORTED_FAILURES,
        "failures": failures,
    }
    return summary


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator telemetry to selector observation CSV export",
        f"status: {payload.get('status', '')}",
        f"input_csv: {payload.get('input_csv', '')}",
        f"output_csv: {payload.get('output_csv', '')}",
        f"input_row_count: {payload.get('input_row_count', 0)}",
        f"output_row_count: {payload.get('output_row_count', 0)}",
        f"skipped_invalid_row_count: {payload.get('skipped_invalid_row_count', 0)}",
        f"dropped_truth_column_count: {payload.get('dropped_truth_column_count', 0)}",
        f"allow_missing_corrected: {payload.get('allow_missing_corrected', False)}",
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
    payload = export_observation_csv(
        input_csv=args.input_csv,
        output_csv=args.output_csv,
        allow_missing_corrected=args.allow_missing_corrected,
    )
    if args.summary_json is not None:
        write_json(args.summary_json, payload)
    if args.summary_txt is not None:
        write_text(args.summary_txt, payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
