#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


FORBIDDEN_RUNTIME_TOKENS = ("truth", "true_", "gt_", "px4", "rtk", "ned", "validation", "replay", "target_window")
RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Resolve a disparity residual profile from a truth-free runtime feature summary. "
            "The model is trained offline, but selection consumes only image/disparity/RDF summary features."
        )
    )
    parser.add_argument("--selector-model-json", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--feature-json", type=Path)
    group.add_argument("--feature-csv", type=Path)
    parser.add_argument("--csv-row-index", type=int, default=0)
    parser.add_argument("--disable-margin-threshold", type=float, default=0.0)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def resolve_path(base_json: Path, value: object) -> Path:
    path = Path(str(value))
    return path if path.is_absolute() else (base_json.parent / path).resolve()


def has_forbidden_name(values: object) -> bool:
    if not isinstance(values, list):
        return True
    names = " ".join(str(value).lower() for value in values)
    return any(token in names for token in FORBIDDEN_RUNTIME_TOKENS)


def feature_row_from_csv(path: Path, row_index: int) -> dict[str, object]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if row_index < 0 or row_index >= len(rows):
        raise IndexError(f"csv row index out of range: {row_index}")
    return dict(rows[row_index])


def numeric_feature_value(row: Mapping[str, object], name: str) -> float:
    value = row.get(name, 0.0)
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return 0.0
    return parsed if math.isfinite(parsed) else 0.0


def select_label(feature_row: Mapping[str, object], model: Mapping[str, object]) -> dict[str, object]:
    feature_names = [str(name) for name in model.get("feature_names", [])]
    if not feature_names:
        raise ValueError("selector model feature_names is empty")
    mean = np.asarray(model.get("mean", []), dtype=float)
    scale = np.asarray(model.get("scale", []), dtype=float)
    if mean.shape != (len(feature_names),) or scale.shape != (len(feature_names),):
        raise ValueError("selector model mean/scale length mismatch")
    scale = np.where(np.isfinite(scale) & (np.abs(scale) > 1.0e-12), scale, 1.0)
    vector = np.asarray([numeric_feature_value(feature_row, name) for name in feature_names], dtype=float)
    normalized = (vector - mean) / scale
    centroids = model.get("centroids", {})
    if not isinstance(centroids, Mapping) or not centroids:
        raise ValueError("selector model centroids must be a non-empty object")
    distances: list[tuple[str, float]] = []
    for label, centroid_values in centroids.items():
        centroid = np.asarray(centroid_values, dtype=float)
        if centroid.shape != normalized.shape:
            raise ValueError(f"centroid length mismatch for {label}")
        distances.append((str(label), float(np.linalg.norm(normalized - centroid))))
    distances.sort(key=lambda item: item[1])
    selected_label, best_distance = distances[0]
    second_distance = distances[1][1] if len(distances) > 1 else best_distance + 1.0e9
    return {
        "selected_label": selected_label,
        "best_distance": best_distance,
        "second_distance": second_distance,
        "distance_margin": float(second_distance - best_distance) if math.isfinite(second_distance) else float("inf"),
    }


def profile_paths_from_catalog(catalog_json: Path) -> dict[str, Path]:
    catalog = load_json(catalog_json)
    paths = catalog.get("profile_paths", {})
    if not isinstance(paths, Mapping):
        raise ValueError("catalog profile_paths must be an object")
    return {str(label): resolve_path(catalog_json, path) for label, path in paths.items()}


def select_residual_profile_by_features(
    *,
    selector_model_json: Path,
    catalog_json: Path,
    feature_row: Mapping[str, object],
    disable_margin_threshold: float = 0.0,
) -> dict[str, object]:
    selector_model_json = selector_model_json.resolve()
    catalog_json = catalog_json.resolve()
    model = load_json(selector_model_json)
    failures: list[str] = []
    feature_names = model.get("feature_names", [])
    if has_forbidden_name(feature_names):
        failures.append("selector_model:feature_names_truth")
    selection = select_label(feature_row, model)
    disabled_by_margin = float(selection["distance_margin"]) < float(disable_margin_threshold)
    paths = profile_paths_from_catalog(catalog_json)
    selected_profile = paths.get(str(selection["selected_label"]), Path(""))
    if not disabled_by_margin and not selected_profile.is_file():
        failures.append("selection:profile_file_missing")
    profile_payload: dict[str, object] = {}
    if selected_profile.is_file():
        profile_payload = load_json(selected_profile)
        if profile_payload.get("runtime_truth_used") is not False:
            failures.append("profile:runtime_truth_used")
        runtime_inputs = profile_payload.get("runtime_inputs", [])
        if has_forbidden_name(runtime_inputs):
            failures.append("profile:runtime_inputs_truth")
    return {
        "mode": "stereodistance_disparity_residual_profile_feature_selection",
        "status": "PASS" if not failures else "FAIL",
        "selector_model_json": str(selector_model_json),
        "catalog_json": str(catalog_json),
        "selected_label": selection["selected_label"],
        "selected_profile": "" if disabled_by_margin else str(selected_profile),
        "disabled_by_margin": disabled_by_margin,
        "disable_margin_threshold": float(disable_margin_threshold),
        "best_distance": selection["best_distance"],
        "second_distance": selection["second_distance"],
        "distance_margin": selection["distance_margin"],
        "profile_id": profile_payload.get("profile_id", ""),
        "method": profile_payload.get("method", ""),
        "feature_space": profile_payload.get("feature_space", ""),
        "correction_space": profile_payload.get("correction_space", ""),
        "selection_input_fields": feature_names,
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "selection consumes truth-free runtime feature summaries only: image-space left_u/left_v, "
            "measured/fused disparity, and stereo-center RDF summary statistics"
        ),
        "coordinate_contract": (
            "selected residual profile consumes measured disparity and corrected stereo-center RDF(right,down,forward); "
            "public output remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance disparity residual profile feature selection",
        f"status: {payload.get('status', '')}",
        f"selected_label: {payload.get('selected_label', '')}",
        f"selected_profile: {payload.get('selected_profile', '')}",
        f"disabled_by_margin: {payload.get('disabled_by_margin', False)}",
        f"distance_margin: {payload.get('distance_margin', '')}",
        f"profile_id: {payload.get('profile_id', '')}",
        f"method: {payload.get('method', '')}",
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
    feature_row = load_json(args.feature_json) if args.feature_json is not None else feature_row_from_csv(args.feature_csv, args.csv_row_index)
    payload = select_residual_profile_by_features(
        selector_model_json=args.selector_model_json,
        catalog_json=args.catalog_json,
        feature_row=feature_row,
        disable_margin_threshold=float(args.disable_margin_threshold),
    )
    if args.output_json is not None:
        write_json(args.output_json, payload)
    if args.output_txt is not None:
        write_text(args.output_txt, payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
