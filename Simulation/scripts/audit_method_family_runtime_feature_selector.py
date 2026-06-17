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


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_by_runtime_measurement_guard import (  # noqa: E402
    FEATURE_NAMES,
    FEATURE_SUBSETS,
    finite_float,
    runtime_measurement_features,
)
from stereodistance_profile_replay import load_profile, replay_rows  # noqa: E402


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit truth-free runtime feature selection among previously selected method-family profiles. "
            "Selection uses raw stereo-center RDF/disparity features only; truth is used only for replay metrics."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--family-selection-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--feature-subset", choices=tuple(FEATURE_SUBSETS.keys()) + ("all",), default="all")
    parser.add_argument(
        "--selection-policy",
        choices=("nearest", "feature_vote", "robust_bbox_fallback"),
        default="nearest",
    )
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--include-target-run", action="store_true")
    parser.add_argument("--exclude-target-scenario", action="store_true")
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def selected_profile_path(row: Mapping[str, str]) -> Path:
    exported = str(row.get("exported_profile_path", "")).strip()
    if exported:
        return Path(exported)
    return Path(str(row.get("selected_profile_path", "")).strip())


def calibration_csv_for_row(sweep_dir: Path, row: Mapping[str, str]) -> Path:
    return sweep_dir / str(row.get("scenario", "")) / str(row.get("seed", "")) / "calibration_samples.csv"


def feature_names_for_subset(feature_subset: str) -> tuple[str, ...]:
    if feature_subset == "all":
        return tuple(FEATURE_NAMES)
    return tuple(FEATURE_SUBSETS[feature_subset])


def feature_names_for_policy(feature_subset: str, selection_policy: str) -> tuple[str, ...]:
    if selection_policy == "feature_vote":
        return tuple(FEATURE_NAMES)
    return feature_names_for_subset(feature_subset)


def feature_vector(features: Mapping[str, float], names: Sequence[str]) -> np.ndarray:
    return np.asarray([finite_float(features.get(name), 0.0) for name in names], dtype=float)


def standardize_vectors(
    target_vector: np.ndarray,
    source_vectors: Sequence[np.ndarray],
) -> tuple[np.ndarray, list[np.ndarray]]:
    matrix = np.vstack(source_vectors)
    center = np.mean(matrix, axis=0)
    scale = np.std(matrix, axis=0)
    scale = np.where(np.isfinite(scale) & (scale > 1.0e-9), scale, 1.0)
    return (target_vector - center) / scale, [(vector - center) / scale for vector in source_vectors]


def scaled_distance(
    target_features: Mapping[str, float],
    source_items: Sequence[Mapping[str, object]],
    source_feature_names: Sequence[str],
) -> list[dict[str, object]]:
    target_vector = feature_vector(target_features, source_feature_names)
    source_vectors = [
        feature_vector(source["features"], source_feature_names)  # type: ignore[arg-type]
        for source in source_items
    ]
    target_scaled, source_scaled = standardize_vectors(target_vector, source_vectors)
    rows: list[dict[str, object]] = []
    for source, vector in zip(source_items, source_scaled):
        rows.append(
            {
                "source_scenario": source.get("scenario", ""),
                "source_seed": source.get("seed", ""),
                "source_method": source.get("method", ""),
                "source_profile_path": source.get("profile_path", ""),
                "distance": float(np.linalg.norm(target_scaled - vector)),
            }
        )
    return sorted(
        rows,
        key=lambda row: (
            finite_float(row.get("distance"), float("inf")),
            str(row["source_scenario"]),
            str(row["source_seed"]),
        ),
    )


def select_nearest_source(
    target: Mapping[str, object],
    sources: Sequence[Mapping[str, object]],
    feature_names: Sequence[str],
) -> tuple[Mapping[str, object], list[dict[str, object]], list[dict[str, object]]]:
    distance_rows = scaled_distance(target["features"], sources, feature_names)  # type: ignore[arg-type]
    selected = distance_rows[0]
    source = next(
        item
        for item in sources
        if item.get("scenario") == selected["source_scenario"] and item.get("seed") == selected["source_seed"]
    )
    return source, distance_rows, []


def select_feature_vote_source(
    target: Mapping[str, object],
    sources: Sequence[Mapping[str, object]],
) -> tuple[Mapping[str, object], list[dict[str, object]], list[dict[str, object]]]:
    aggregate: dict[tuple[str, str], dict[str, float]] = {}
    vote_rows: list[dict[str, object]] = []
    for subset_name, subset_features in FEATURE_SUBSETS.items():
        ranked = scaled_distance(target["features"], sources, subset_features)  # type: ignore[arg-type]
        for rank, row in enumerate(ranked, start=1):
            key = (str(row["source_scenario"]), str(row["source_seed"]))
            stats = aggregate.setdefault(key, {"vote_count": 0.0, "rank_sum": 0.0, "distance_sum": 0.0})
            if rank == 1:
                stats["vote_count"] += 1.0
            stats["rank_sum"] += float(rank)
            stats["distance_sum"] += finite_float(row.get("distance"), float("inf"))
        if ranked:
            vote_rows.append(
                {
                    "feature_subset": subset_name,
                    "winner_scenario": ranked[0]["source_scenario"],
                    "winner_seed": ranked[0]["source_seed"],
                    "winner_method": ranked[0]["source_method"],
                    "winner_distance": ranked[0]["distance"],
                    "feature_names": ",".join(subset_features),
                    "ranked_sources_json": json.dumps(ranked[:8], ensure_ascii=False, allow_nan=False),
                }
            )
    if not aggregate:
        raise ValueError("feature_vote selection has no candidate sources")
    selected_key = sorted(
        aggregate.items(),
        key=lambda item: (-item[1]["vote_count"], item[1]["rank_sum"], item[1]["distance_sum"], item[0]),
    )[0][0]
    source = next(item for item in sources if item.get("scenario") == selected_key[0] and item.get("seed") == selected_key[1])
    ranked_sources = [
        {
            "source_scenario": key[0],
            "source_seed": key[1],
            "vote_count": stats["vote_count"],
            "rank_sum": stats["rank_sum"],
            "distance_sum": stats["distance_sum"],
        }
        for key, stats in aggregate.items()
    ]
    ranked_sources.sort(key=lambda row: (-row["vote_count"], row["rank_sum"], row["distance_sum"], row["source_scenario"], row["source_seed"]))
    return source, ranked_sources, vote_rows


def select_robust_bbox_fallback_source(
    target: Mapping[str, object],
    sources: Sequence[Mapping[str, object]],
    feature_names: Sequence[str],
) -> tuple[Mapping[str, object], list[dict[str, object]], list[dict[str, object]]]:
    preliminary_source, preliminary_rows, _ = select_nearest_source(target, sources, feature_names)
    fallback_rows: list[dict[str, object]] = [
        {
            "policy_step": "nearest_preliminary",
            "source_scenario": preliminary_source.get("scenario", ""),
            "source_seed": preliminary_source.get("seed", ""),
            "source_method": preliminary_source.get("method", ""),
        }
    ]
    if preliminary_source.get("scenario") in {"nominal", "yaw_pitch_stress", "bbox_outlier_stress"}:
        bbox_sources = [source for source in sources if source.get("scenario") == "bbox_outlier_stress"]
        if bbox_sources:
            selected_source, bbox_rows, _ = select_nearest_source(target, bbox_sources, feature_names)
            fallback_rows.append(
                {
                    "policy_step": "low_range_family_bbox_fallback",
                    "source_scenario": selected_source.get("scenario", ""),
                    "source_seed": selected_source.get("seed", ""),
                    "source_method": selected_source.get("method", ""),
                }
            )
            return selected_source, bbox_rows, fallback_rows
    return preliminary_source, preliminary_rows, fallback_rows


def load_selected_library(sweep_dir: Path, selected_csv: Path, feature_names: Sequence[str]) -> list[dict[str, object]]:
    library: list[dict[str, object]] = []
    for row in read_csv(selected_csv):
        if str(row.get("selected", "")).strip() != "True":
            continue
        profile_path = selected_profile_path(row)
        csv_path = calibration_csv_for_row(sweep_dir, row)
        if not profile_path.is_file() or not csv_path.is_file():
            continue
        sample_rows = read_csv(csv_path)
        features = runtime_measurement_features(sample_rows)
        library.append(
            {
                "scenario": str(row.get("scenario", "")),
                "seed": str(row.get("seed", "")),
                "method": str(row.get("selected_method", "")),
                "profile_path": str(profile_path),
                "csv_path": str(csv_path),
                "features": features,
                "feature_vector": feature_vector(features, feature_names),
            }
        )
    return library


def candidate_sources_for_target(
    library: Sequence[Mapping[str, object]],
    target: Mapping[str, object],
    *,
    include_target_run: bool,
    exclude_target_scenario: bool,
) -> list[Mapping[str, object]]:
    out: list[Mapping[str, object]] = []
    for item in library:
        same_run = item.get("scenario") == target.get("scenario") and item.get("seed") == target.get("seed")
        same_scenario = item.get("scenario") == target.get("scenario")
        if same_run and not include_target_run:
            continue
        if same_scenario and exclude_target_scenario:
            continue
        out.append(item)
    return out


def replay_selected_source(
    *,
    source: Mapping[str, object],
    target_rows: Sequence[Mapping[str, str]],
    split: str,
    target_range_m: float,
    target_window_m: float,
) -> dict[str, object]:
    profile = load_profile(Path(str(source["profile_path"])))
    _, summary = replay_rows(
        target_rows,
        profile,
        split_filter=split,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    return dict(summary)


def audit_target(
    target: Mapping[str, object],
    library: Sequence[Mapping[str, object]],
    *,
    feature_names: Sequence[str],
    split: str,
    target_range_m: float,
    target_window_m: float,
    target_error_limit_m: float,
    include_target_run: bool,
    exclude_target_scenario: bool,
    selection_policy: str,
) -> dict[str, object]:
    sources = candidate_sources_for_target(
        library,
        target,
        include_target_run=include_target_run,
        exclude_target_scenario=exclude_target_scenario,
    )
    if not sources:
        raise ValueError(f"no source profiles available for {target.get('scenario')}/{target.get('seed')}")
    if selection_policy == "feature_vote":
        selected_source, distance_rows, vote_rows = select_feature_vote_source(target, sources)
    elif selection_policy == "robust_bbox_fallback":
        selected_source, distance_rows, vote_rows = select_robust_bbox_fallback_source(target, sources, feature_names)
    else:
        selected_source, distance_rows, vote_rows = select_nearest_source(target, sources, feature_names)
    selected_distance = distance_rows[0]
    selected_distance_value = finite_float(
        selected_distance.get("distance", selected_distance.get("distance_sum", float("nan"))),
        float("nan"),
    )
    target_rows = read_csv(Path(str(target["csv_path"])))
    replay = replay_selected_source(
        source=selected_source,
        target_rows=target_rows,
        split=split,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    raw_rmse = finite_float(replay.get("validation_range_rmse_raw_m"), float("nan"))
    corrected_rmse = finite_float(replay.get("validation_range_rmse_corrected_m"), float("nan"))
    improvement = raw_rmse - corrected_rmse if math.isfinite(raw_rmse) and math.isfinite(corrected_rmse) else float("nan")
    target_count = int(finite_float(replay.get("target_window_count"), 0.0))
    target_abs = finite_float(replay.get("target_abs_mean_error_corrected_m"), float("nan"))
    target_max = finite_float(replay.get("target_max_abs_error_corrected_m"), float("nan"))
    fail_reasons: list[str] = []
    if not math.isfinite(improvement) or improvement <= 0.0:
        fail_reasons.append("no_positive_range_rmse_improvement")
    if target_count <= 0:
        fail_reasons.append("no_target_window_samples")
    if not math.isfinite(target_abs):
        fail_reasons.append("target_error_not_finite")
    elif target_abs > target_error_limit_m:
        fail_reasons.append("target_error_over_limit")
    if not math.isfinite(target_max):
        fail_reasons.append("target_max_error_not_finite")
    elif target_max > target_error_limit_m:
        fail_reasons.append("target_max_error_over_limit")
    passed = not fail_reasons
    target_features = target["features"]
    row = {
        "target_scenario": target.get("scenario", ""),
        "target_seed": target.get("seed", ""),
        "target_csv_path": target.get("csv_path", ""),
        "selected_source_scenario": selected_source.get("scenario", ""),
        "selected_source_seed": selected_source.get("seed", ""),
        "selected_source_method": selected_source.get("method", ""),
        "selected_source_profile_path": selected_source.get("profile_path", ""),
        "selected_distance": selected_distance_value,
        "candidate_source_count": len(sources),
        "include_target_run": include_target_run,
        "exclude_target_scenario": exclude_target_scenario,
        "selection_policy": selection_policy,
        "pass_runtime_feature_selector_replay": passed,
        "pass_reason": "pass" if passed else ",".join(fail_reasons),
        "validation_range_rmse_raw_m": replay.get("validation_range_rmse_raw_m", ""),
        "validation_range_rmse_corrected_m": replay.get("validation_range_rmse_corrected_m", ""),
        "validation_range_rmse_improvement_m": improvement,
        "target_window_count": target_count,
        "target_abs_mean_error_corrected_m": replay.get("target_abs_mean_error_corrected_m", ""),
        "target_max_abs_error_corrected_m": replay.get("target_max_abs_error_corrected_m", ""),
        "distance_rows_json": json.dumps(distance_rows[:8], ensure_ascii=False, allow_nan=False),
        "feature_vote_rows_json": json.dumps(vote_rows, ensure_ascii=False, allow_nan=False),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "runtime feature selector uses raw RDF/disparity feature statistics only; replay truth is audit-only"
        ),
        "runtime_input_contract": "selected source profile consumes raw stereo-center RDF/disparity and fixed calibration parameters only",
    }
    for name in FEATURE_NAMES:
        row[f"feature_{name}"] = finite_float(target_features.get(name), float("nan"))
    return row


def audit_runtime_feature_selector(
    sweep_dir: Path,
    family_selection_csv: Path,
    output_dir: Path,
    *,
    feature_subset: str = "all",
    split: str = "validation",
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
    include_target_run: bool = False,
    exclude_target_scenario: bool = False,
    selection_policy: str = "nearest",
) -> dict[str, object]:
    feature_names = feature_names_for_policy(feature_subset, selection_policy)
    library = load_selected_library(sweep_dir, family_selection_csv, feature_names)
    if not library:
        raise ValueError("family selection CSV produced no usable selected profiles")
    rows = [
        audit_target(
            target,
            library,
            feature_names=feature_names,
            split=split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
            target_error_limit_m=target_error_limit_m,
            include_target_run=include_target_run,
            exclude_target_scenario=exclude_target_scenario,
            selection_policy=selection_policy,
        )
        for target in library
    ]
    pass_count = sum(1 for row in rows if bool(row.get("pass_runtime_feature_selector_replay")))
    rmse_values = [
        finite_float(row.get("validation_range_rmse_corrected_m"))
        for row in rows
        if math.isfinite(finite_float(row.get("validation_range_rmse_corrected_m")))
    ]
    target_values = [
        finite_float(row.get("target_max_abs_error_corrected_m"))
        for row in rows
        if math.isfinite(finite_float(row.get("target_max_abs_error_corrected_m")))
    ]
    summary = {
        "mode": "method_family_runtime_feature_selector_audit",
        "sweep_dir": str(sweep_dir),
        "family_selection_csv": str(family_selection_csv),
        "feature_subset": feature_subset,
        "selection_policy": selection_policy,
        "feature_names": ",".join(feature_names),
        "audit_count": len(rows),
        "pass_count": pass_count,
        "pass_all": bool(rows) and pass_count == len(rows),
        "validation_range_rmse_corrected_mean_m": float(np.mean(rmse_values)) if rmse_values else None,
        "validation_range_rmse_corrected_max_m": max(rmse_values, default=None),
        "target_max_abs_error_max_m": max(target_values, default=None),
        "target_error_limit_m": target_error_limit_m,
        "include_target_run": include_target_run,
        "exclude_target_scenario": exclude_target_scenario,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Runtime feature selector uses raw RDF/disparity features only. "
            "Selection policy may rank nearest feature profiles, vote across feature groups, "
            "or apply a conservative raw-feature fallback. Truth appears only in replay metrics "
            "for this offline audit."
        ),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "runtime_feature_selector_audit.csv", rows)
    write_json(output_dir / "runtime_feature_selector_summary.json", summary)
    write_summary_txt(output_dir / "runtime_feature_selector_summary.txt", summary, rows)
    return summary


def write_csv_rows(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    write_csv(path, rows)


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Method family runtime feature selector audit",
        "",
        f"feature_subset: {summary.get('feature_subset', '')}",
        f"selection_policy: {summary.get('selection_policy', '')}",
        f"audit_count: {summary.get('audit_count', '')}",
        f"pass_count: {summary.get('pass_count', '')}",
        f"pass_all: {summary.get('pass_all', '')}",
        f"validation_range_rmse_corrected_mean_m: {summary.get('validation_range_rmse_corrected_mean_m', '')}",
        f"validation_range_rmse_corrected_max_m: {summary.get('validation_range_rmse_corrected_max_m', '')}",
        f"target_max_abs_error_max_m: {summary.get('target_max_abs_error_max_m', '')}",
        f"include_target_run: {summary.get('include_target_run', '')}",
        f"exclude_target_scenario: {summary.get('exclude_target_scenario', '')}",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        "",
        "Rows:",
    ]
    for row in rows:
        lines.append(
            "- {target}/{seed}: selected={source}/{source_seed} {method}, pass={passed}, rmse={rmse}, target_max={target_max}, reason={reason}".format(
                target=row.get("target_scenario", ""),
                seed=row.get("target_seed", ""),
                source=row.get("selected_source_scenario", ""),
                source_seed=row.get("selected_source_seed", ""),
                method=row.get("selected_source_method", ""),
                passed=row.get("pass_runtime_feature_selector_replay", ""),
                rmse=row.get("validation_range_rmse_corrected_m", ""),
                target_max=row.get("target_max_abs_error_corrected_m", ""),
                reason=row.get("pass_reason", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    summary = audit_runtime_feature_selector(
        args.sweep_dir,
        args.family_selection_csv,
        args.output_dir,
        feature_subset=args.feature_subset,
        selection_policy=args.selection_policy,
        split=args.split,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
        include_target_run=args.include_target_run,
        exclude_target_scenario=args.exclude_target_scenario,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
