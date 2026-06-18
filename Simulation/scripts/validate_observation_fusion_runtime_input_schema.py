#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping


REQUIRED_COLUMNS = ("group_id", "left_u_px", "left_v_px", "measured_disparity_px")
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


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def finite_float(value: object) -> float | None:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def validate_rows(
    *,
    columns: list[str],
    rows: list[Mapping[str, object]],
    require_burst_id: bool,
) -> dict[str, object]:
    failures: list[str] = []
    missing_columns = [column for column in REQUIRED_COLUMNS if column not in columns]
    for column in missing_columns:
        failures.append(f"missing_required_column:{column}")
    if require_burst_id and "burst_id" not in columns:
        failures.append("missing_required_column:burst_id")
    forbidden_present = sorted(set(columns).intersection(FORBIDDEN_TRUTH_COLUMNS))
    for column in forbidden_present:
        failures.append(f"forbidden_truth_column:{column}")
    if not rows:
        failures.append("no_rows")

    group_ids: set[str] = set()
    burst_ids_by_group: dict[str, set[str]] = {}
    for row_index, row in enumerate(rows):
        group_id = str(row.get("group_id", "")).strip()
        if not group_id:
            failures.append(f"row{row_index}:empty_group_id")
        else:
            group_ids.add(group_id)
        if require_burst_id:
            burst_id = str(row.get("burst_id", "")).strip()
            if not burst_id:
                failures.append(f"row{row_index}:empty_burst_id")
            elif group_id:
                burst_ids_by_group.setdefault(group_id, set()).add(burst_id)
        for field in ("left_u_px", "left_v_px", "measured_disparity_px"):
            value = finite_float(row.get(field))
            if value is None:
                failures.append(f"row{row_index}:{field}_not_finite")
            elif field == "measured_disparity_px" and value <= 0.0:
                failures.append(f"row{row_index}:measured_disparity_px_not_positive")

    burst_count_by_group = {group: len(bursts) for group, bursts in sorted(burst_ids_by_group.items())}
    return {
        "status": "PASS" if not failures else "FAIL",
        "row_count": len(rows),
        "group_count": len(group_ids),
        "require_burst_id": require_burst_id,
        "burst_count_by_group": burst_count_by_group,
        "failure_count": len(failures),
        "failures": failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "input_contract": "group_id[/burst_id], left image u/v pixel centers, positive measured stereo disparity px",
        "output_contract": "validated image observations for stereo-center RDF reconstruction",
    }


def validate_csv(path: Path, *, require_burst_id: bool = False) -> dict[str, object]:
    columns, rows = read_rows(path)
    payload = validate_rows(columns=columns, rows=rows, require_burst_id=require_burst_id)
    payload["input_csv"] = str(path)
    return payload


def write_text(path: Path, validation: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion runtime input schema validation",
        f"input_csv: {validation['input_csv']}",
        f"status: {validation['status']}",
        f"row_count: {validation['row_count']}",
        f"group_count: {validation['group_count']}",
        f"require_burst_id: {validation['require_burst_id']}",
        f"failure_count: {validation['failure_count']}",
        "runtime_truth_used: False",
        "Coordinate contract: image u/v/disparity input; no PX4/RTK truth fields; output is ready for stereo-center RDF reconstruction.",
    ]
    for group, count in dict(validation.get("burst_count_by_group", {})).items():
        lines.append(f"burst_count[{group}]: {count}")
    for failure in validation.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate observation-fusion runtime input CSV axis/schema contract.")
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--require-burst-id", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    validation = validate_csv(args.input_csv, require_burst_id=args.require_burst_id)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "observation_fusion_runtime_input_schema_validation.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_text(args.output_dir / "observation_fusion_runtime_input_schema_validation.txt", validation)
    print(json.dumps(validation, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
