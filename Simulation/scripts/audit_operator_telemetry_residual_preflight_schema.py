#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import export_selector_observation_csv_from_operator_telemetry as telemetry_export  # noqa: E402


FIELD_GROUPS: tuple[tuple[str, tuple[str, ...], str], ...] = (
    ("left_u_px", ("left_cx", "left_cx_px", "left_u_px"), "image coordinate u, pixels"),
    ("left_v_px", ("left_cy", "left_cy_px", "left_v_px"), "image coordinate v, pixels"),
    (
        "measured_disparity_px",
        ("raw_disparity_px", "disparity_px", "bbox_adjustment_raw_disparity_px"),
        "measured stereo disparity, pixels",
    ),
    (
        "nominal_pose_forward_m",
        ("nominal_pose_forward_m", "raw_forward_m"),
        "public FRD nominal forward, meters",
    ),
    (
        "nominal_pose_right_m",
        ("nominal_pose_right_m", "raw_right_m"),
        "public FRD nominal right, meters",
    ),
    (
        "nominal_pose_down_m",
        ("nominal_pose_down_m", "raw_down_m"),
        "public FRD nominal down, meters",
    ),
    ("observation_forward_m", ("corrected_forward_m",), "public FRD corrected forward, meters"),
    ("observation_right_m", ("corrected_right_m",), "public FRD corrected right, meters"),
    ("observation_down_m", ("corrected_down_m",), "public FRD corrected down, meters"),
)

RAW_REQUIRED_GROUPS = {
    "left_u_px",
    "left_v_px",
    "measured_disparity_px",
    "nominal_pose_forward_m",
    "nominal_pose_right_m",
    "nominal_pose_down_m",
}
CORRECTED_REQUIRED_GROUPS = {"observation_forward_m", "observation_right_m", "observation_down_m"}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit whether an operator telemetry.csv can enter the truth-free residual preflight. "
            "Input telemetry is public FRD; downstream selector observations are explicit stereo-center RDF."
        )
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path)
    parser.add_argument("--summary-txt", type=Path)
    parser.add_argument("--allow-missing-corrected", action="store_true")
    parser.add_argument("--min-valid-rows", type=int, default=1)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def group_report(
    *,
    group_name: str,
    aliases: Sequence[str],
    description: str,
    columns: Sequence[str],
    rows: Sequence[Mapping[str, object]],
) -> dict[str, object]:
    present_aliases = [alias for alias in aliases if alias in columns]
    finite_count = 0
    positive_count = 0
    for row in rows:
        value = telemetry_export.pick_float(row, aliases)
        if value is None:
            continue
        finite_count += 1
        if value > 0.0:
            positive_count += 1
    return {
        "group": group_name,
        "description": description,
        "aliases": list(aliases),
        "present_aliases": present_aliases,
        "column_present": bool(present_aliases),
        "finite_row_count": finite_count,
        "positive_row_count": positive_count,
    }


def audit_schema(
    *,
    input_csv: Path,
    allow_missing_corrected: bool = False,
    min_valid_rows: int = 1,
) -> dict[str, object]:
    columns, rows = telemetry_export.read_rows(input_csv)
    min_valid_rows = max(1, int(min_valid_rows))
    groups = [
        group_report(group_name=name, aliases=aliases, description=description, columns=columns, rows=rows)
        for name, aliases, description in FIELD_GROUPS
    ]
    required_groups = set(RAW_REQUIRED_GROUPS)
    if not allow_missing_corrected:
        required_groups.update(CORRECTED_REQUIRED_GROUPS)

    missing_required_groups = [
        {
            "group": group["group"],
            "aliases": group["aliases"],
            "reason": "no_supported_column_present",
        }
        for group in groups
        if str(group["group"]) in required_groups and not bool(group["column_present"])
    ]
    nonfinite_required_groups = [
        {
            "group": group["group"],
            "aliases": group["aliases"],
            "finite_row_count": group["finite_row_count"],
            "reason": "no_finite_values",
        }
        for group in groups
        if str(group["group"]) in required_groups
        and bool(group["column_present"])
        and int(group["finite_row_count"]) == 0
    ]

    valid_row_count = 0
    invalid_row_count = 0
    failure_samples: list[str] = []
    for index, row in enumerate(rows):
        converted, failures = telemetry_export.convert_row(row, allow_missing_corrected=allow_missing_corrected)
        if converted is not None:
            valid_row_count += 1
            continue
        invalid_row_count += 1
        for failure in failures:
            if len(failure_samples) < telemetry_export.MAX_REPORTED_FAILURES:
                failure_samples.append(f"row{index}:{failure}")

    failures: list[str] = []
    if not rows:
        failures.append("input_csv:no_rows")
    for group in missing_required_groups:
        failures.append(f"missing_required_group:{group['group']}")
    for group in nonfinite_required_groups:
        failures.append(f"nonfinite_required_group:{group['group']}")
    if valid_row_count < min_valid_rows:
        failures.append(f"valid_row_count_below_min:{valid_row_count}<{min_valid_rows}")

    payload = {
        "mode": "operator_telemetry_residual_preflight_schema_audit",
        "status": "PASS" if not failures else "FAIL",
        "input_csv": str(input_csv),
        "input_row_count": len(rows),
        "input_column_count": len(columns),
        "valid_row_count": valid_row_count,
        "invalid_row_count": invalid_row_count,
        "min_valid_rows": min_valid_rows,
        "allow_missing_corrected": bool(allow_missing_corrected),
        "required_groups": sorted(required_groups),
        "missing_required_groups": missing_required_groups,
        "nonfinite_required_groups": nonfinite_required_groups,
        "field_groups": groups,
        "dropped_truth_column_count": len(telemetry_export.truth_like_columns(columns)),
        "dropped_truth_columns": telemetry_export.truth_like_columns(columns),
        "runtime_truth_used": False,
        "input_axis_contract": telemetry_export.INPUT_AXIS_CONTRACT,
        "axis_contract": telemetry_export.OUTPUT_AXIS_CONTRACT,
        "public_topic_contract": telemetry_export.PUBLIC_TOPIC_CONTRACT,
        "coordinate_contract": (
            "operator telemetry input must expose public FRD forward/right/down fields plus image center "
            "and measured disparity; preflight converts those fields to explicit stereo-center RDF "
            "right/down/forward observations; truth/PX4/RTK/NED fields are report-only and are not "
            "runtime selector inputs"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "row_failure_sample_count": len(failure_samples),
        "row_failure_samples": failure_samples,
    }
    return payload


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator telemetry residual preflight schema audit",
        f"status: {payload.get('status', '')}",
        f"input_csv: {payload.get('input_csv', '')}",
        f"input_row_count: {payload.get('input_row_count', 0)}",
        f"valid_row_count: {payload.get('valid_row_count', 0)}",
        f"invalid_row_count: {payload.get('invalid_row_count', 0)}",
        f"min_valid_rows: {payload.get('min_valid_rows', 1)}",
        f"allow_missing_corrected: {payload.get('allow_missing_corrected', False)}",
        f"dropped_truth_column_count: {payload.get('dropped_truth_column_count', 0)}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"coordinate_contract: {payload.get('coordinate_contract', '')}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    missing = payload.get("missing_required_groups", [])
    if isinstance(missing, list) and missing:
        lines.append("")
        lines.append("Missing required groups:")
        for group in missing:
            if isinstance(group, Mapping):
                lines.append(f"- {group.get('group', '')}: aliases={group.get('aliases', [])}")
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    samples = payload.get("row_failure_samples", [])
    if isinstance(samples, list) and samples:
        lines.append("")
        lines.append("Row failure samples:")
        lines.extend(f"- {sample}" for sample in samples)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    payload = audit_schema(
        input_csv=args.input_csv,
        allow_missing_corrected=args.allow_missing_corrected,
        min_valid_rows=args.min_valid_rows,
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
