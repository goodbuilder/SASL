#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.disparity_residual_feature_summary import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    feature_summary_from_arrays,
)


REQUIRED_COLUMNS = (
    "left_u_px",
    "left_v_px",
    "measured_disparity_px",
    "raw_right_m",
    "raw_down_m",
    "raw_forward_m",
)
FORBIDDEN_TRUTH_TOKENS = ("truth", "true_", "gt_", "px4", "rtk", "ned", "validation", "replay")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export one truth-free residual selector feature row from runtime observation/RDF CSV."
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--feature-label", default="runtime_observation_window")
    parser.add_argument("--allow-missing-observation-rdf", action="store_true")
    return parser.parse_args(argv)


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def forbidden_columns(columns: Sequence[str]) -> list[str]:
    return sorted(
        column
        for column in columns
        if any(token in column.lower() for token in FORBIDDEN_TRUTH_TOKENS)
    )


def finite_float(value: object) -> float | None:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def column_values(rows: Sequence[Mapping[str, object]], column: str, failures: list[str]) -> np.ndarray:
    values: list[float] = []
    for index, row in enumerate(rows):
        value = finite_float(row.get(column))
        if value is None:
            failures.append(f"row{index}:{column}_not_finite")
            value = float("nan")
        values.append(value)
    return np.asarray(values, dtype=float)


def build_feature_row(
    *,
    rows: Sequence[Mapping[str, object]],
    columns: Sequence[str],
    feature_label: str,
    allow_missing_observation_rdf: bool,
) -> tuple[dict[str, object], list[str]]:
    failures: list[str] = []
    missing = [column for column in REQUIRED_COLUMNS if column not in columns]
    failures.extend(f"missing_required_column:{column}" for column in missing)
    forbidden = forbidden_columns(columns)
    failures.extend(f"forbidden_truth_column:{column}" for column in forbidden)
    if not rows:
        failures.append("no_rows")
    if failures:
        return {
            "feature_label": feature_label,
            "runtime_truth_used": False,
        }, failures

    left_u = column_values(rows, "left_u_px", failures)
    left_v = column_values(rows, "left_v_px", failures)
    measured_disparity = column_values(rows, "measured_disparity_px", failures)
    if np.any(measured_disparity[np.isfinite(measured_disparity)] <= 0.0):
        failures.append("measured_disparity_px_not_positive")
    fused_disparity = (
        column_values(rows, "fused_disparity_px", failures)
        if "fused_disparity_px" in columns
        else measured_disparity
    )
    raw_points = np.column_stack(
        [
            column_values(rows, "raw_right_m", failures),
            column_values(rows, "raw_down_m", failures),
            column_values(rows, "raw_forward_m", failures),
        ]
    )
    observation_columns = ("observation_right_m", "observation_down_m", "observation_forward_m")
    if all(column in columns for column in observation_columns):
        observation_points = np.column_stack([column_values(rows, column, failures) for column in observation_columns])
        observation_source = "observation_rdf_columns"
    elif allow_missing_observation_rdf:
        observation_points = raw_points.copy()
        observation_source = "raw_rdf_fallback"
    else:
        failures.extend(f"missing_required_column:{column}" for column in observation_columns)
        observation_points = raw_points.copy()
        observation_source = "missing"

    features = feature_summary_from_arrays(
        left_u_px=left_u,
        left_v_px=left_v,
        measured_disparity_px=measured_disparity,
        raw_rdf_m=raw_points,
        fused_disparity_px=fused_disparity,
        observation_rdf_m=observation_points,
        feature_label=feature_label,
        observation_rdf_source=observation_source,
    )
    return features, failures


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Disparity residual feature row export",
        f"status: {summary.get('status', '')}",
        f"input_csv: {summary.get('input_csv', '')}",
        f"feature_json: {summary.get('feature_json', '')}",
        f"feature_csv: {summary.get('feature_csv', '')}",
        f"row_count: {summary.get('row_count', 0)}",
        f"observation_rdf_source: {summary.get('observation_rdf_source', '')}",
        "runtime_truth_used: False",
        "Coordinate contract: input raw/observation positions are stereo-center RDF(right,down,forward); selector feature output remains truth-free; public output stays FRD+range.",
        f"failure_count: {summary.get('failure_count', 0)}",
    ]
    for failure in summary.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_feature_row(
    *,
    input_csv: Path,
    output_dir: Path,
    feature_label: str = "runtime_observation_window",
    allow_missing_observation_rdf: bool = False,
) -> dict[str, object]:
    columns, rows = read_rows(input_csv)
    feature_row, failures = build_feature_row(
        rows=rows,
        columns=columns,
        feature_label=feature_label,
        allow_missing_observation_rdf=allow_missing_observation_rdf,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    feature_json = output_dir / "disparity_residual_selector_feature_row.json"
    feature_csv = output_dir / "disparity_residual_selector_feature_row.csv"
    write_json(feature_json, feature_row)
    write_csv(feature_csv, [feature_row])
    summary = {
        "mode": "disparity_residual_selector_feature_row_export",
        "status": "PASS" if not failures else "FAIL",
        "input_csv": str(input_csv),
        "feature_json": str(feature_json),
        "feature_csv": str(feature_csv),
        "feature_label": feature_label,
        "row_count": len(rows),
        "observation_rdf_source": feature_row.get("observation_rdf_source", ""),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "failure_count": len(failures),
        "failures": failures,
    }
    write_json(output_dir / "disparity_residual_selector_feature_row_summary.json", summary)
    write_text(output_dir / "disparity_residual_selector_feature_row_summary.txt", summary)
    return summary


def main() -> None:
    args = parse_args()
    payload = export_feature_row(
        input_csv=args.input_csv,
        output_dir=args.output_dir,
        feature_label=args.feature_label,
        allow_missing_observation_rdf=args.allow_missing_observation_rdf,
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
