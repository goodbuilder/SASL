#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping


REQUIRED_FIELDS = (
    "group_id",
    "topic",
    "message_type",
    "axis_order",
    "public_topic_contract",
    "forward_m",
    "right_m",
    "down_m",
    "range_m",
    "float32_multi_array_data_json",
    "runtime_truth_used",
)
PUBLIC_TOPIC = "/stereo/corrected_position"
MESSAGE_TYPE = "std_msgs/msg/Float32MultiArray"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


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


def close_enough(actual: float | None, expected: float, tolerance_m: float) -> bool:
    return actual is not None and abs(actual - expected) <= tolerance_m


def validate_row(row: Mapping[str, object], *, row_index: int, tolerance_m: float) -> list[str]:
    failures: list[str] = []
    if str(row.get("topic", "")) != PUBLIC_TOPIC:
        failures.append(f"row{row_index}:topic_mismatch")
    if str(row.get("message_type", "")) != MESSAGE_TYPE:
        failures.append(f"row{row_index}:message_type_mismatch")
    if str(row.get("axis_order", "")) != "forward,right,down,range":
        failures.append(f"row{row_index}:axis_order_mismatch")
    if str(row.get("public_topic_contract", "")) != PUBLIC_TOPIC_CONTRACT:
        failures.append(f"row{row_index}:public_topic_contract_mismatch")
    if str(row.get("runtime_truth_used", "")).strip().lower() not in {"false", "0", ""}:
        failures.append(f"row{row_index}:runtime_truth_used_not_false")

    forward = finite_float(row.get("forward_m"))
    right = finite_float(row.get("right_m"))
    down = finite_float(row.get("down_m"))
    range_m = finite_float(row.get("range_m"))
    for field, value in (("forward_m", forward), ("right_m", right), ("down_m", down), ("range_m", range_m)):
        if value is None:
            failures.append(f"row{row_index}:{field}_not_finite")
    if all(value is not None for value in (forward, right, down, range_m)):
        assert forward is not None and right is not None and down is not None and range_m is not None
        expected_range = math.sqrt(forward * forward + right * right + down * down)
        if abs(range_m - expected_range) > tolerance_m:
            failures.append(f"row{row_index}:range_m_mismatch")
        try:
            payload = json.loads(str(row.get("float32_multi_array_data_json", "")))
        except json.JSONDecodeError:
            failures.append(f"row{row_index}:payload_json_invalid")
            payload = None
        if not isinstance(payload, list) or len(payload) != 4:
            failures.append(f"row{row_index}:payload_not_len4_array")
        elif not all(
            close_enough(finite_float(payload[index]), expected, tolerance_m)
            for index, expected in enumerate((forward, right, down, range_m))
        ):
            failures.append(f"row{row_index}:payload_order_or_value_mismatch")
    return failures


def validate_csv(path: Path, *, tolerance_m: float) -> dict[str, object]:
    fieldnames, rows = read_rows(path)
    failures = [f"missing_field:{field}" for field in REQUIRED_FIELDS if field not in fieldnames]
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
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "axis_order": "forward,right,down,range",
    }


def validate_against_source(
    *,
    payload_csv: Path,
    source_csv: Path,
    tolerance_m: float,
) -> dict[str, object]:
    _, payload_rows = read_rows(payload_csv)
    _, source_rows = read_rows(source_csv)
    source_by_group = {str(row.get("group_id", "")): row for row in source_rows}
    failures: list[str] = []
    if not payload_rows:
        failures.append("no_payload_rows")
    if not source_rows:
        failures.append("no_source_rows")
    for index, payload_row in enumerate(payload_rows):
        group_id = str(payload_row.get("group_id", ""))
        source_row = source_by_group.get(group_id)
        if source_row is None:
            failures.append(f"row{index}:source_group_missing:{group_id}")
            continue
        expected = (
            finite_float(source_row.get("corrected_public_forward_m")),
            finite_float(source_row.get("corrected_public_right_m")),
            finite_float(source_row.get("corrected_public_down_m")),
            finite_float(source_row.get("corrected_public_range_m")),
        )
        actual = (
            finite_float(payload_row.get("forward_m")),
            finite_float(payload_row.get("right_m")),
            finite_float(payload_row.get("down_m")),
            finite_float(payload_row.get("range_m")),
        )
        for field, actual_value, expected_value in zip(
            ("forward_m", "right_m", "down_m", "range_m"),
            actual,
            expected,
        ):
            if actual_value is None or expected_value is None or abs(actual_value - expected_value) > tolerance_m:
                failures.append(f"row{index}:{field}_source_mismatch")
        try:
            payload = json.loads(str(payload_row.get("float32_multi_array_data_json", "")))
        except json.JSONDecodeError:
            failures.append(f"row{index}:payload_json_invalid")
            payload = []
        if not isinstance(payload, list) or len(payload) != 4:
            failures.append(f"row{index}:payload_not_len4_array")
        else:
            for payload_index, expected_value in enumerate(expected):
                payload_value = finite_float(payload[payload_index])
                if payload_value is None or expected_value is None or abs(payload_value - expected_value) > tolerance_m:
                    failures.append(f"row{index}:payload_index{payload_index}_source_mismatch")
    return {
        "payload_csv": str(payload_csv),
        "source_csv": str(source_csv),
        "status": "PASS" if not failures else "FAIL",
        "payload_row_count": len(payload_rows),
        "source_row_count": len(source_rows),
        "failure_count": len(failures),
        "failures": failures,
        "runtime_truth_used": False,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "axis_order": "forward,right,down,range",
    }


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion public topic payload validation",
        f"input_csv: {summary['input_csv']}",
        f"status: {summary['status']}",
        f"row_count: {summary['row_count']}",
        f"failure_count: {summary['failure_count']}",
        "runtime_truth_used: False",
        "Coordinate contract: public payload must be Float32MultiArray [forward,right,down,range].",
    ]
    for failure in summary.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_source_equivalence_text(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion public topic payload source equivalence",
        f"payload_csv: {summary['payload_csv']}",
        f"source_csv: {summary['source_csv']}",
        f"status: {summary['status']}",
        f"payload_row_count: {summary['payload_row_count']}",
        f"source_row_count: {summary['source_row_count']}",
        f"failure_count: {summary['failure_count']}",
        "runtime_truth_used: False",
        "Coordinate contract: payload FRD+range values must match corrected_public_* source aliases.",
    ]
    for failure in summary.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate public StereoDistance corrected-position payload CSV.")
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--source-csv", type=Path, default=None)
    parser.add_argument("--tolerance-m", type=float, default=1.0e-6)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = validate_csv(args.input_csv, tolerance_m=args.tolerance_m)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "observation_fusion_public_topic_payload_validation.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_text(args.output_dir / "observation_fusion_public_topic_payload_validation.txt", summary)
    if args.source_csv is not None:
        source_summary = validate_against_source(
            payload_csv=args.input_csv,
            source_csv=args.source_csv,
            tolerance_m=args.tolerance_m,
        )
        (args.output_dir / "observation_fusion_public_topic_payload_source_equivalence.json").write_text(
            json.dumps(source_summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        write_source_equivalence_text(
            args.output_dir / "observation_fusion_public_topic_payload_source_equivalence.txt",
            source_summary,
        )
        if source_summary["status"] != "PASS":
            summary["status"] = "FAIL"
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
