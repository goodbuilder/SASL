#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Mapping, Sequence


FIELD_NAMES = (
    "left_valid",
    "left_x1",
    "left_y1",
    "left_x2",
    "left_y2",
    "left_confidence",
    "left_cx",
    "left_cy",
    "right_valid",
    "right_x1",
    "right_y1",
    "right_x2",
    "right_y2",
    "right_confidence",
    "right_cx",
    "right_cy",
)

FORBIDDEN_TRUTH_COLUMNS = (
    "actual_distance_m",
    "target_distance_m",
    "ground_truth_m",
    "truth_range_m",
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "px4_north_m",
    "px4_east_m",
    "px4_down_m",
    "rtk_north_m",
    "rtk_east_m",
    "rtk_down_m",
    "drone_forward_m",
    "drone_right_m",
    "drone_down_m",
)


def finite_float(value: object) -> float | None:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def parse_values(values: Sequence[object]) -> list[float | None]:
    return [finite_float(value) for value in values]


def parse_data_cell(value: object) -> list[str]:
    text = str(value or "").strip()
    if not text:
        return []
    return [part for part in re.split(r"[\s,;]+", text.strip("[]()")) if part]


def rows_from_csv(path: Path) -> tuple[list[str], list[list[object]], list[str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        columns = list(reader.fieldnames or [])
        rows: list[list[object]] = []
        for row in reader:
            if all(name in row for name in FIELD_NAMES):
                rows.append([row[name] for name in FIELD_NAMES])
            elif "data" in row:
                rows.append(parse_data_cell(row.get("data")))
            else:
                rows.append([])
        return columns, rows, sorted(set(columns).intersection(FORBIDDEN_TRUTH_COLUMNS))


def validate_detection(
    *,
    side: str,
    values: Sequence[float | None],
    width_px: int,
    height_px: int,
    row_index: int,
) -> tuple[dict[str, float] | None, list[str]]:
    failures: list[str] = []
    prefix = f"row{row_index}:{side}"
    if len(values) != 8:
        return None, [f"{prefix}:field_count:{len(values)}!=8"]
    names = ("valid", "x1", "y1", "x2", "y2", "confidence", "cx", "cy")
    by_name = dict(zip(names, values))
    for name, value in by_name.items():
        if value is None:
            failures.append(f"{prefix}:{name}_not_finite")
    if failures:
        return None, failures
    valid = float(by_name["valid"])
    if valid <= 0.5:
        failures.append(f"{prefix}:valid_flag_not_set")
    x1 = float(by_name["x1"])
    y1 = float(by_name["y1"])
    x2 = float(by_name["x2"])
    y2 = float(by_name["y2"])
    confidence = float(by_name["confidence"])
    cx = float(by_name["cx"])
    cy = float(by_name["cy"])
    if x2 <= x1:
        failures.append(f"{prefix}:x2_not_greater_than_x1")
    if y2 <= y1:
        failures.append(f"{prefix}:y2_not_greater_than_y1")
    if not (0.0 <= confidence <= 1.0):
        failures.append(f"{prefix}:confidence_out_of_unit_range")
    if not (x1 <= cx <= x2):
        failures.append(f"{prefix}:cx_outside_bbox")
    if not (y1 <= cy <= y2):
        failures.append(f"{prefix}:cy_outside_bbox")
    if x1 < 0.0 or y1 < 0.0 or x2 > float(width_px) or y2 > float(height_px):
        failures.append(f"{prefix}:bbox_outside_image_bounds")
    if cx < 0.0 or cy < 0.0 or cx > float(width_px) or cy > float(height_px):
        failures.append(f"{prefix}:center_outside_image_bounds")
    if failures:
        return None, failures
    return {
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "confidence": confidence,
        "cx": cx,
        "cy": cy,
    }, []


def validate_value_rows(
    *,
    rows: Sequence[Sequence[object]],
    width_px: int,
    height_px: int,
    min_disparity_px: float,
    forbidden_truth_columns: Sequence[str] = (),
    source: str = "",
) -> dict[str, object]:
    failures: list[str] = []
    row_failure_samples: list[dict[str, object]] = []
    valid_pair_count = 0
    disparity_values: list[float] = []
    if not rows:
        failures.append("no_rows")
    for column in forbidden_truth_columns:
        failures.append(f"forbidden_truth_column:{column}")
    for row_index, raw_values in enumerate(rows):
        parsed = parse_values(raw_values)
        row_failures: list[str] = []
        if len(parsed) != len(FIELD_NAMES):
            row_failures.append(f"row{row_index}:field_count:{len(parsed)}!=16")
        else:
            left, left_failures = validate_detection(
                side="left",
                values=parsed[:8],
                width_px=width_px,
                height_px=height_px,
                row_index=row_index,
            )
            right, right_failures = validate_detection(
                side="right",
                values=parsed[8:],
                width_px=width_px,
                height_px=height_px,
                row_index=row_index,
            )
            row_failures.extend(left_failures)
            row_failures.extend(right_failures)
            if left is not None and right is not None:
                disparity = float(left["cx"]) - float(right["cx"])
                if disparity < float(min_disparity_px):
                    row_failures.append(f"row{row_index}:left_minus_right_disparity_below_min:{disparity:g}")
                else:
                    disparity_values.append(disparity)
                    valid_pair_count += 1
        failures.extend(row_failures)
        if row_failures and len(row_failure_samples) < 10:
            row_failure_samples.append(
                {
                    "row_index": row_index,
                    "failures": row_failures,
                    "values": [None if value is None else float(value) for value in parsed[: len(FIELD_NAMES)]],
                }
            )
    return {
        "mode": "external_bbox_topic_contract_validation",
        "status": "PASS" if not failures else "FAIL",
        "source": source,
        "row_count": len(rows),
        "valid_pair_count": valid_pair_count,
        "width_px": int(width_px),
        "height_px": int(height_px),
        "min_disparity_px": float(min_disparity_px),
        "disparity_min_px": min(disparity_values) if disparity_values else None,
        "disparity_max_px": max(disparity_values) if disparity_values else None,
        "message_type": "std_msgs/msg/Float32MultiArray",
        "message_contract": list(FIELD_NAMES),
        "runtime_truth_used": False,
        "coordinate_contract": (
            "input is left/right image pixel BBox centers and boxes; left_cx - right_cx is positive "
            "measured disparity; StereoDistance reconstructs stereo-center RDF(right,down,forward) "
            "internally and publishes public FRD+range(forward,right,down,range); PX4/RTK/NED truth "
            "is forbidden for this runtime topic"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "row_failure_sample_count": len(row_failure_samples),
        "row_failure_samples": row_failure_samples,
    }


def validate_csv(
    *,
    input_csv: Path,
    width_px: int = 1280,
    height_px: int = 720,
    min_disparity_px: float = 0.5,
) -> dict[str, object]:
    columns, rows, forbidden = rows_from_csv(input_csv)
    payload = validate_value_rows(
        rows=rows,
        width_px=width_px,
        height_px=height_px,
        min_disparity_px=min_disparity_px,
        forbidden_truth_columns=forbidden,
        source=str(input_csv),
    )
    payload["input_csv"] = str(input_csv)
    payload["input_columns"] = columns
    return payload


def validate_values(
    *,
    values: Sequence[object],
    width_px: int = 1280,
    height_px: int = 720,
    min_disparity_px: float = 0.5,
) -> dict[str, object]:
    return validate_value_rows(
        rows=[values],
        width_px=width_px,
        height_px=height_px,
        min_disparity_px=min_disparity_px,
        source="cli_values",
    )


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "External BBox topic contract validation",
        f"status: {payload.get('status', '')}",
        f"source: {payload.get('source', '')}",
        f"row_count: {payload.get('row_count', 0)}",
        f"valid_pair_count: {payload.get('valid_pair_count', 0)}",
        f"message_type: {payload.get('message_type', '')}",
        f"message_contract: {','.join(str(v) for v in payload.get('message_contract', []))}",
        f"disparity_min_px: {payload.get('disparity_min_px', '')}",
        f"disparity_max_px: {payload.get('disparity_max_px', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"coordinate_contract: {payload.get('coordinate_contract', '')}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    for failure in payload.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate the external stereo BBox Float32MultiArray contract consumed by StereoDistance. "
            "Use this before replacing the sidecar with a real YOLO/BBox publisher."
        )
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--values", nargs="+")
    source.add_argument("--input-csv", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--width-px", type=int, default=1280)
    parser.add_argument("--height-px", type=int, default=720)
    parser.add_argument("--min-disparity-px", type=float, default=0.5)
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args()
    if args.input_csv is not None:
        payload = validate_csv(
            input_csv=args.input_csv,
            width_px=args.width_px,
            height_px=args.height_px,
            min_disparity_px=args.min_disparity_px,
        )
    else:
        payload = validate_values(
            values=args.values or [],
            width_px=args.width_px,
            height_px=args.height_px,
            min_disparity_px=args.min_disparity_px,
        )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "external_bbox_topic_contract_validation.json"
    txt_path = args.output_dir / "external_bbox_topic_contract_validation.txt"
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    write_text(txt_path, payload)
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
