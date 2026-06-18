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

from stereo_ground_sim.disparity_residual_feature_selector import (  # noqa: E402
    DisparityResidualFeatureSelector,
    FORBIDDEN_RUNTIME_TOKENS,
)


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
ALLOWED_METADATA_COLUMNS = {
    "runtime_truth_used",
    "selector_split",
    "scenario",
    "seed",
}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay saved truth-free disparity residual feature-window rows through the same "
            "selector class used by StereoDistanceNode."
        )
    )
    parser.add_argument("--selector-model-json", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--feature-csv", type=Path, required=True)
    parser.add_argument("--selector-split", default="validation")
    parser.add_argument("--expected-label-column", default="scenario")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


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


def load_csv_rows(path: Path) -> list[dict[str, object]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def has_forbidden_columns(row: Mapping[str, object]) -> bool:
    names = " ".join(
        str(name).lower()
        for name in row.keys()
        if str(name).lower() not in ALLOWED_METADATA_COLUMNS
    )
    return any(token in names for token in FORBIDDEN_RUNTIME_TOKENS)


def numeric_values(rows: Sequence[Mapping[str, object]], name: str) -> np.ndarray:
    values: list[float] = []
    for row in rows:
        try:
            parsed = float(row.get(name, "nan"))
        except (TypeError, ValueError):
            parsed = float("nan")
        values.append(parsed if math.isfinite(parsed) else float("nan"))
    return np.asarray(values, dtype=float)


def finite_mean(rows: Sequence[Mapping[str, object]], name: str) -> float:
    values = numeric_values(rows, name)
    values = values[np.isfinite(values)]
    return float(np.mean(values)) if values.size else 0.0


def replay_selector(
    *,
    selector_model_json: Path,
    catalog_json: Path,
    feature_csv: Path,
    selector_split: str = "validation",
    expected_label_column: str = "scenario",
) -> dict[str, object]:
    selector = DisparityResidualFeatureSelector(
        selector_model_json=selector_model_json,
        catalog_json=catalog_json,
    )
    rows = load_csv_rows(feature_csv)
    failures: list[str] = []
    if not rows:
        failures.append("feature_csv:empty")
    if any(has_forbidden_columns(row) for row in rows):
        failures.append("feature_csv:forbidden_column")
    replay_rows: list[dict[str, object]] = []
    for index, row in enumerate(rows):
        split = str(row.get("selector_split", ""))
        if selector_split and split != selector_split:
            continue
        expected_label = str(row.get(expected_label_column, ""))
        selection = selector.select(row)
        selected_label = str(selection.get("selected_label", ""))
        selected_profile = str(selection.get("selected_profile", ""))
        selected_profile_exists = bool(selection.get("selected_profile_exists", False))
        label_matched = bool(expected_label and selected_label == expected_label)
        runtime_truth_used = str(row.get("runtime_truth_used", "False")).strip().lower() in {
            "1",
            "true",
            "yes",
            "on",
        }
        replay_rows.append(
            {
                "row_index": index,
                "selector_split": split,
                "expected_label": expected_label,
                "selected_label": selected_label,
                "label_matched": label_matched,
                "selected_profile": selected_profile,
                "selected_profile_exists": selected_profile_exists,
                "best_distance": selection.get("best_distance", 0.0),
                "second_distance": selection.get("second_distance", 0.0),
                "distance_margin": selection.get("distance_margin", 0.0),
                "runtime_truth_used": runtime_truth_used,
            }
        )
    if not replay_rows:
        failures.append("feature_csv:no_selected_rows")
    label_match_count = sum(1 for row in replay_rows if row["label_matched"])
    profile_exists_count = sum(1 for row in replay_rows if row["selected_profile_exists"])
    runtime_truth_count = sum(1 for row in replay_rows if row["runtime_truth_used"])
    if replay_rows and label_match_count != len(replay_rows):
        failures.append("replay:label_mismatch")
    if replay_rows and profile_exists_count != len(replay_rows):
        failures.append("replay:profile_missing")
    if runtime_truth_count:
        failures.append("replay:runtime_truth_used")
    return {
        "mode": "disparity_residual_feature_selector_replay",
        "status": "PASS" if not failures else "FAIL",
        "selector_model_json": str(selector_model_json),
        "catalog_json": str(catalog_json),
        "feature_csv": str(feature_csv),
        "selector_split": selector_split,
        "expected_label_column": expected_label_column,
        "replay_row_count": len(replay_rows),
        "label_match_count": label_match_count,
        "profile_exists_count": profile_exists_count,
        "runtime_truth_count": runtime_truth_count,
        "distance_margin_mean": finite_mean(replay_rows, "distance_margin"),
        "distance_margin_min": float(np.nanmin(numeric_values(replay_rows, "distance_margin")))
        if replay_rows
        else 0.0,
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "replay consumes saved runtime feature-window rows only: image-space left_u/left_v, "
            "measured/fused disparity, and raw/observation stereo-center RDF summary statistics"
        ),
        "coordinate_contract": (
            "selector input is stereo-center RDF(right,down,forward) summary metadata; selected residual "
            "profiles consume corrected stereo-center RDF and measured disparity; public output remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "rows": replay_rows,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Disparity residual feature selector replay",
        f"status: {payload.get('status', '')}",
        f"selector_split: {payload.get('selector_split', '')}",
        f"replay_row_count: {payload.get('replay_row_count', 0)}",
        f"label_match_count: {payload.get('label_match_count', 0)}",
        f"profile_exists_count: {payload.get('profile_exists_count', 0)}",
        f"runtime_truth_count: {payload.get('runtime_truth_count', 0)}",
        f"distance_margin_mean: {payload.get('distance_margin_mean', '')}",
        f"distance_margin_min: {payload.get('distance_margin_min', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"selection_contract: {payload.get('selection_contract', '')}",
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
    payload = replay_selector(
        selector_model_json=args.selector_model_json,
        catalog_json=args.catalog_json,
        feature_csv=args.feature_csv,
        selector_split=args.selector_split,
        expected_label_column=args.expected_label_column,
    )
    rows = payload.pop("rows")
    if args.output_json is not None:
        write_json(args.output_json, payload)
    if args.output_csv is not None:
        write_csv(args.output_csv, rows if isinstance(rows, list) else [])
    if args.output_txt is not None:
        write_text(args.output_txt, payload)
    payload["rows"] = rows
    return payload


def main() -> None:
    payload = run()
    printable = dict(payload)
    printable.pop("rows", None)
    print(json.dumps(printable, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
