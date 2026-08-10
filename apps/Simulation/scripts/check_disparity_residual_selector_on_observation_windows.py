#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter
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
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.disparity_residual_feature_selector import (  # noqa: E402
    DisparityResidualFeatureSelector,
)
from stereo_ground_sim.disparity_residual_feature_summary import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    feature_summary_from_runtime_rows,
)

import export_disparity_residual_feature_row_from_observations as feature_export  # noqa: E402


REQUIRED_COLUMNS = feature_export.REQUIRED_COLUMNS
FORBIDDEN_TRUTH_TOKENS = feature_export.FORBIDDEN_TRUTH_TOKENS


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build rolling truth-free observation windows from a runtime observation CSV and replay "
            "the residual feature selector on each window."
        )
    )
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--selector-model-json", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--window-size", type=int, default=21)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--min-samples", type=int, default=1)
    parser.add_argument("--feature-label", default="runtime_observation_window")
    parser.add_argument("--expected-label", default="")
    parser.add_argument("--allow-missing-observation-rdf", action="store_true")
    parser.add_argument("--require-stable-label", action="store_true")
    parser.add_argument("--min-distance-margin", type=float, default=0.0)
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


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), [dict(row) for row in reader]


def forbidden_columns(columns: Sequence[str]) -> list[str]:
    return sorted(
        column
        for column in columns
        if any(token in column.lower() for token in FORBIDDEN_TRUTH_TOKENS)
    )


def runtime_truth_value(row: Mapping[str, object]) -> bool:
    return str(row.get("runtime_truth_used", "False")).strip().lower() in {"1", "true", "yes", "on"}


def window_slices(rows: Sequence[Mapping[str, object]], window_size: int, stride: int, min_samples: int):
    size = max(1, int(window_size))
    step = max(1, int(stride))
    minimum = max(1, int(min_samples))
    for start in range(0, len(rows), step):
        end = min(len(rows), start + size)
        if end - start < minimum:
            continue
        yield start, end, rows[start:end]
        if end == len(rows):
            break


def selector_windows(
    *,
    input_csv: Path,
    selector_model_json: Path,
    catalog_json: Path,
    window_size: int = 21,
    stride: int = 1,
    min_samples: int = 1,
    feature_label: str = "runtime_observation_window",
    expected_label: str = "",
    allow_missing_observation_rdf: bool = False,
    require_stable_label: bool = False,
    min_distance_margin: float = 0.0,
) -> dict[str, object]:
    columns, rows = read_rows(input_csv)
    failures: list[str] = []
    missing = [column for column in REQUIRED_COLUMNS if column not in columns]
    failures.extend(f"missing_required_column:{column}" for column in missing)
    forbidden = forbidden_columns(columns)
    failures.extend(f"forbidden_truth_column:{column}" for column in forbidden)
    if not rows:
        failures.append("no_rows")
    if any(runtime_truth_value(row) for row in rows):
        failures.append("runtime_truth_used")
    selector = DisparityResidualFeatureSelector(
        selector_model_json=selector_model_json,
        catalog_json=catalog_json,
    )
    feature_rows: list[dict[str, object]] = []
    replay_rows: list[dict[str, object]] = []
    if not failures:
        for window_index, (start, end, window_rows) in enumerate(
            window_slices(rows, window_size, stride, min_samples)
        ):
            feature_row = feature_summary_from_runtime_rows(
                window_rows,
                feature_label=f"{feature_label}_{window_index}",
                observation_rdf_source=(
                    "observation_rdf_columns"
                    if all(
                        column in columns
                        for column in ("observation_right_m", "observation_down_m", "observation_forward_m")
                    )
                    else "raw_rdf_fallback"
                ),
            )
            if not allow_missing_observation_rdf and feature_row.get("observation_rdf_source") == "raw_rdf_fallback":
                failures.append("missing_observation_rdf_columns")
                break
            selection = selector.select(feature_row)
            selected_label = str(selection.get("selected_label", ""))
            selected_profile_exists = bool(selection.get("selected_profile_exists", False))
            label_matched = bool(expected_label and selected_label == expected_label)
            feature_export_row = {
                "window_index": window_index,
                "window_start": start,
                "window_end": end,
                "window_row_count": len(window_rows),
                "expected_label": expected_label,
                **feature_row,
            }
            feature_rows.append(feature_export_row)
            replay_rows.append(
                {
                    "window_index": window_index,
                    "window_start": start,
                    "window_end": end,
                    "window_row_count": len(window_rows),
                    "expected_label": expected_label,
                    "selected_label": selected_label,
                    "label_matched": label_matched,
                    "selected_profile": str(selection.get("selected_profile", "")),
                    "selected_profile_exists": selected_profile_exists,
                    "best_distance": selection.get("best_distance", 0.0),
                    "second_distance": selection.get("second_distance", 0.0),
                    "distance_margin": selection.get("distance_margin", 0.0),
                    "runtime_truth_used": False,
                }
            )
    if not replay_rows and not failures:
        failures.append("no_windows")
    margins = np.asarray([float(row["distance_margin"]) for row in replay_rows], dtype=float)
    label_counts = dict(sorted(Counter(str(row["selected_label"]) for row in replay_rows).items()))
    dominant_label = ""
    dominant_count = 0
    if label_counts:
        dominant_label, dominant_count = max(label_counts.items(), key=lambda item: (item[1], item[0]))
    selector_label_stable = bool(replay_rows) and len(label_counts) == 1
    distance_margin_min = float(np.min(margins)) if margins.size else 0.0
    margin_threshold = max(0.0, float(min_distance_margin))
    if replay_rows and expected_label and any(not row["label_matched"] for row in replay_rows):
        failures.append("selector:label_mismatch")
    if replay_rows and any(not row["selected_profile_exists"] for row in replay_rows):
        failures.append("selector:profile_missing")
    if replay_rows and require_stable_label and not selector_label_stable:
        failures.append("selector:unstable_label")
    if replay_rows and margin_threshold > 0.0 and distance_margin_min < margin_threshold:
        failures.append(f"selector:distance_margin_below_threshold:{distance_margin_min:.6f}<{margin_threshold:.6f}")
    return {
        "mode": "disparity_residual_selector_observation_window_replay",
        "status": "PASS" if not failures else "FAIL",
        "input_csv": str(input_csv),
        "selector_model_json": str(selector_model_json),
        "catalog_json": str(catalog_json),
        "window_size": max(1, int(window_size)),
        "stride": max(1, int(stride)),
        "min_samples": max(1, int(min_samples)),
        "input_row_count": len(rows),
        "window_count": len(replay_rows),
        "expected_label": expected_label,
        "require_stable_label": bool(require_stable_label),
        "min_distance_margin": margin_threshold,
        "label_match_count": sum(1 for row in replay_rows if row["label_matched"]),
        "profile_exists_count": sum(1 for row in replay_rows if row["selected_profile_exists"]),
        "selected_labels": sorted({str(row["selected_label"]) for row in replay_rows}),
        "selected_label_counts": label_counts,
        "dominant_selected_label": dominant_label,
        "dominant_selected_label_count": dominant_count,
        "selector_label_stable": selector_label_stable,
        "distance_margin_mean": float(np.mean(margins)) if margins.size else 0.0,
        "distance_margin_min": distance_margin_min,
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "runtime observation CSV windows consume image left_u/left_v, measured/fused disparity, "
            "and raw/observation stereo-center RDF summary features only"
        ),
        "coordinate_contract": (
            "input raw/observation positions are stereo-center RDF(right,down,forward); selected residual "
            "profiles consume corrected stereo-center RDF plus measured disparity; public output remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "feature_rows": feature_rows,
        "replay_rows": replay_rows,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Disparity residual selector observation-window replay",
        f"status: {payload.get('status', '')}",
        f"input_csv: {payload.get('input_csv', '')}",
        f"input_row_count: {payload.get('input_row_count', 0)}",
        f"window_size: {payload.get('window_size', 0)}",
        f"stride: {payload.get('stride', 0)}",
        f"min_samples: {payload.get('min_samples', 0)}",
        f"window_count: {payload.get('window_count', 0)}",
        f"expected_label: {payload.get('expected_label', '')}",
        f"require_stable_label: {payload.get('require_stable_label', False)}",
        f"min_distance_margin: {payload.get('min_distance_margin', 0.0)}",
        f"selected_labels: {','.join(str(v) for v in payload.get('selected_labels', []))}",
        f"selected_label_counts: {json.dumps(payload.get('selected_label_counts', {}), ensure_ascii=False, sort_keys=True)}",
        f"dominant_selected_label: {payload.get('dominant_selected_label', '')}",
        f"dominant_selected_label_count: {payload.get('dominant_selected_label_count', 0)}",
        f"selector_label_stable: {payload.get('selector_label_stable', False)}",
        f"label_match_count: {payload.get('label_match_count', 0)}",
        f"profile_exists_count: {payload.get('profile_exists_count', 0)}",
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
    payload = selector_windows(
        input_csv=args.input_csv,
        selector_model_json=args.selector_model_json,
        catalog_json=args.catalog_json,
        window_size=args.window_size,
        stride=args.stride,
        min_samples=args.min_samples,
        feature_label=args.feature_label,
        expected_label=args.expected_label,
        allow_missing_observation_rdf=args.allow_missing_observation_rdf,
        require_stable_label=args.require_stable_label,
        min_distance_margin=args.min_distance_margin,
    )
    feature_rows = payload.pop("feature_rows")
    replay_rows = payload.pop("replay_rows")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_json(args.output_dir / "observation_window_selector_replay_summary.json", payload)
    write_csv(args.output_dir / "observation_window_selector_features.csv", feature_rows)
    write_csv(args.output_dir / "observation_window_selector_replay.csv", replay_rows)
    write_text(args.output_dir / "observation_window_selector_replay_summary.txt", payload)
    payload["feature_rows"] = feature_rows
    payload["replay_rows"] = replay_rows
    return payload


def main() -> None:
    payload = run()
    printable = dict(payload)
    printable.pop("feature_rows", None)
    printable.pop("replay_rows", None)
    print(json.dumps(printable, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
