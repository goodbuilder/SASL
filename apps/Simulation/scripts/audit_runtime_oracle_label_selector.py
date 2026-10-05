#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_by_calibration_replay import (  # noqa: E402
    finite_float,
    read_csv,
    replay_candidate,
    write_csv,
    write_json,
)
from select_stereodistance_profile_by_runtime_measurement_guard import (  # noqa: E402
    FEATURE_NAMES,
    RAW_FEATURE_FIELDS,
    default_transfer_matrix_csv,
    feature_vector,
    load_transfer_matrix,
    runtime_measurement_features,
    sweep_calibration_csvs,
    transfer_confusion_cluster,
    transfer_source_risk_score,
)
from select_stereodistance_profile_from_catalog import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    load_catalog,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit a truth-free runtime selector trained from calibration-time replay labels. "
            "Calibration truth is used only to create offline training labels; each target run is "
            "selected from raw stereo-center RDF/disparity/image-center features."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--calibration-replay-runs-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--transfer-matrix-csv", type=Path)
    parser.add_argument("--transfer-low-risk-rmse-limit-m", type=float, default=1.3)
    parser.add_argument("--transfer-low-risk-target-limit-m", type=float, default=2.5)
    parser.add_argument("--transfer-risk-target-weight", type=float, default=0.10)
    parser.add_argument("--top-label-count", type=int, default=3)
    parser.add_argument(
        "--selection-policy",
        choices=("nearest_label_centroid", "label_centroid_transfer_risk"),
        default="label_centroid_transfer_risk",
    )
    parser.add_argument("--allow-unvalidated", action="store_true")
    return parser.parse_args()


def label_rows_by_csv(path: Path) -> dict[Path, dict[str, str]]:
    rows = read_csv(path)
    mapping: dict[Path, dict[str, str]] = {}
    for row in rows:
        sample_csv = str(row.get("sample_csv", "")).strip()
        label = str(row.get("calibration_source_scenario", "")).strip()
        selected = str(row.get("calibration_selected", "")).strip() == "True"
        passed = str(row.get("calibration_pass", "")).strip() == "True"
        if sample_csv and label and selected and passed:
            mapping[Path(sample_csv).resolve()] = dict(row)
    return mapping


def normalized_matrix(vectors: Sequence[np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    matrix = np.vstack(vectors)
    center = np.nanmean(matrix, axis=0)
    scale = np.nanstd(matrix, axis=0)
    scale = np.where(np.isfinite(scale) & (scale > 1e-9), scale, 1.0)
    return (np.nan_to_num(matrix, nan=0.0) - center) / scale, center, scale


def fit_label_centroids(training: Sequence[Mapping[str, object]]) -> dict[str, object]:
    vectors = [np.asarray(item["feature_vector"], dtype=float) for item in training]
    normalized, center, scale = normalized_matrix(vectors)
    centroids: dict[str, np.ndarray] = {}
    counts: dict[str, int] = {}
    for item, vector in zip(training, normalized):
        label = str(item["label_source_scenario"])
        centroids.setdefault(label, np.zeros(vector.shape, dtype=float))
        centroids[label] += vector
        counts[label] = counts.get(label, 0) + 1
    for label, total in list(centroids.items()):
        centroids[label] = total / float(counts[label])
    return {"center": center, "scale": scale, "centroids": centroids, "counts": counts}


def rank_label_centroids(features: Mapping[str, float], model: Mapping[str, object]) -> list[dict[str, object]]:
    vector = (np.nan_to_num(feature_vector(features), nan=0.0) - model["center"]) / model["scale"]
    rows: list[dict[str, object]] = []
    for label, centroid in sorted(model["centroids"].items()):
        rows.append(
            {
                "label_source_scenario": label,
                "label_centroid_distance": float(np.linalg.norm(vector - centroid)),
                "training_label_count": int(model["counts"][label]),
            }
        )
    return sorted(rows, key=lambda row: (row["label_centroid_distance"], row["label_source_scenario"]))


def build_labeled_library(
    *,
    sweep_dir: Path,
    label_by_csv: Mapping[Path, Mapping[str, str]],
    exclude_csv: Path,
) -> list[dict[str, object]]:
    library: list[dict[str, object]] = []
    exclude_resolved = exclude_csv.resolve()
    for scenario, seed, csv_path in sweep_calibration_csvs(sweep_dir):
        resolved = csv_path.resolve()
        if resolved == exclude_resolved:
            continue
        label_row = label_by_csv.get(resolved)
        if label_row is None:
            continue
        features = runtime_measurement_features(read_csv(csv_path))
        label = str(label_row.get("calibration_source_scenario", "")).strip()
        library.append(
            {
                "target_scenario": scenario,
                "target_seed": seed,
                "csv_path": str(csv_path),
                "label_source_scenario": label,
                "features": features,
                "feature_vector": feature_vector(features),
            }
        )
    if not library:
        raise ValueError("no labeled calibration replay rows available for runtime oracle-label selector")
    return library


def select_label(
    *,
    rows: Sequence[Mapping[str, str]],
    training: Sequence[Mapping[str, object]],
    policy: str,
    transfer_matrix: Mapping[tuple[str, str], Mapping[str, str]],
    transfer_low_risk_rmse_limit_m: float,
    transfer_low_risk_target_limit_m: float,
    transfer_risk_target_weight: float,
    top_label_count: int,
) -> dict[str, object]:
    features = runtime_measurement_features(rows)
    model = fit_label_centroids(training)
    ranked = rank_label_centroids(features, model)
    if not ranked:
        raise ValueError("oracle-label selector has no label centroids")
    preliminary = str(ranked[0]["label_source_scenario"])
    selected = preliminary
    transfer_cluster: list[str] = []
    transfer_score_rows: list[dict[str, object]] = []
    fallback_reason = ""
    if policy == "label_centroid_transfer_risk":
        top_labels = [str(row["label_source_scenario"]) for row in ranked[: max(1, top_label_count)]]
        for label in top_labels:
            for scenario in transfer_confusion_cluster(
                preliminary=label,
                candidate_scenarios=top_labels,
                matrix=transfer_matrix,
                rmse_limit_m=transfer_low_risk_rmse_limit_m,
                target_limit_m=transfer_low_risk_target_limit_m,
            ):
                if scenario not in transfer_cluster:
                    transfer_cluster.append(scenario)
        if not transfer_cluster:
            transfer_cluster = [preliminary]
        scored: list[tuple[float, str, dict[str, object]]] = []
        for source in transfer_cluster:
            score, score_row = transfer_source_risk_score(
                source=source,
                targets=transfer_cluster,
                matrix=transfer_matrix,
                target_weight=transfer_risk_target_weight,
            )
            scored.append((score, source, score_row))
        scored = sorted(scored, key=lambda item: (item[0], item[1]))
        selected = scored[0][1]
        transfer_score_rows = [item[2] for item in scored]
        if selected != preliminary:
            fallback_reason = "label_centroid_transfer_cluster_prefers_lower_risk_source"
    elif policy != "nearest_label_centroid":
        raise ValueError("unsupported oracle-label selection policy")
    return {
        "preliminary_source_scenario": preliminary,
        "selected_source_scenario": selected,
        "selection_policy": policy,
        "fallback_reason": fallback_reason,
        "label_distance_rows": ranked,
        "transfer_confusion_cluster": transfer_cluster,
        "transfer_score_rows": transfer_score_rows,
        "features": features,
    }


def audit_selector(args: argparse.Namespace) -> dict[str, object]:
    catalog = load_catalog(args.catalog_json)
    label_by_csv = label_rows_by_csv(args.calibration_replay_runs_csv)
    transfer_matrix_csv = args.transfer_matrix_csv or default_transfer_matrix_csv(args.sweep_dir)
    transfer_matrix = load_transfer_matrix(transfer_matrix_csv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    audit_rows: list[dict[str, object]] = []
    for target_scenario, target_seed, input_csv in sweep_calibration_csvs(args.sweep_dir):
        rows = read_csv(input_csv)
        training = build_labeled_library(sweep_dir=args.sweep_dir, label_by_csv=label_by_csv, exclude_csv=input_csv)
        selection = select_label(
            rows=rows,
            training=training,
            policy=args.selection_policy,
            transfer_matrix=transfer_matrix,
            transfer_low_risk_rmse_limit_m=args.transfer_low_risk_rmse_limit_m,
            transfer_low_risk_target_limit_m=args.transfer_low_risk_target_limit_m,
            transfer_risk_target_weight=args.transfer_risk_target_weight,
            top_label_count=args.top_label_count,
        )
        selected = str(selection["selected_source_scenario"])
        replay = replay_candidate(
            catalog_json=args.catalog_json,
            catalog=catalog,
            scenario=selected,
            rows=rows,
            split=args.split,
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
            target_error_limit_m=args.target_error_limit_m,
            allow_unvalidated=args.allow_unvalidated,
            stability_splits=("train", "validation"),
        )
        calibration_label = str(label_by_csv.get(input_csv.resolve(), {}).get("calibration_source_scenario", ""))
        row = {
            "target_scenario": target_scenario,
            "target_seed": target_seed,
            "input_csv": str(input_csv),
            "calibration_replay_label_source_scenario": calibration_label,
            "matches_calibration_replay_label": selected == calibration_label,
            "preliminary_source_scenario": selection.get("preliminary_source_scenario", ""),
            "selected_source_scenario": selected,
            "selection_policy": args.selection_policy,
            "fallback_reason": selection.get("fallback_reason", ""),
            "label_distance_rows_json": json.dumps(
                selection.get("label_distance_rows", []), ensure_ascii=False, allow_nan=False
            ),
            "transfer_confusion_cluster": ",".join(selection.get("transfer_confusion_cluster", [])),
            "transfer_score_rows_json": json.dumps(
                selection.get("transfer_score_rows", []), ensure_ascii=False, allow_nan=False
            ),
            "pass_runtime_oracle_label_selector_replay": bool(replay.get("pass_replay")),
            "pass_reason": replay.get("pass_reason", ""),
            "selected_method": replay.get("method", ""),
            "validation_range_rmse_raw_m": replay.get("validation_range_rmse_raw_m"),
            "validation_range_rmse_corrected_m": replay.get("validation_range_rmse_corrected_m"),
            "validation_range_rmse_improvement_m": replay.get("validation_range_rmse_improvement_m"),
            "target_window_count": replay.get("target_window_count"),
            "target_abs_mean_error_corrected_m": replay.get("target_abs_mean_error_corrected_m"),
            "target_max_abs_error_corrected_m": replay.get("target_max_abs_error_corrected_m"),
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "selection_input_fields": ",".join(RAW_FEATURE_FIELDS),
            "selection_feature_names": ",".join(FEATURE_NAMES),
            "selection_contract": (
                "calibration replay truth creates offline labels only; runtime selection uses raw "
                "stereo-center RDF/disparity/image-center features"
            ),
        }
        for name in FEATURE_NAMES:
            row[f"feature_{name}"] = selection["features"].get(name)
        audit_rows.append(row)
    write_csv(args.output_dir / "runtime_oracle_label_selector_audit.csv", audit_rows)
    summary = summarize_rows(args, audit_rows, transfer_matrix_csv)
    write_json(args.output_dir / "runtime_oracle_label_selector_summary.json", summary)
    write_summary_txt(args.output_dir / "runtime_oracle_label_selector_summary.txt", summary, audit_rows)
    plot_selection_counts(args.output_dir / "runtime_oracle_label_selector_selection_counts.png", audit_rows)
    return summary


def summarize_rows(args: argparse.Namespace, rows: Sequence[Mapping[str, object]], transfer_matrix_csv: Path) -> dict[str, object]:
    pass_rows = [row for row in rows if bool(row.get("pass_runtime_oracle_label_selector_replay"))]
    matches = [row for row in rows if bool(row.get("matches_calibration_replay_label"))]
    corrected = [
        finite_float(row.get("validation_range_rmse_corrected_m"), float("nan"))
        for row in rows
        if math.isfinite(finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")))
    ]
    target_abs = [
        finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan"))
        for row in rows
        if math.isfinite(finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan")))
    ]
    target_max = [
        finite_float(row.get("target_max_abs_error_corrected_m"), float("nan"))
        for row in rows
        if math.isfinite(finite_float(row.get("target_max_abs_error_corrected_m"), float("nan")))
    ]
    return {
        "mode": "runtime_oracle_label_selector_audit",
        "selection_policy": args.selection_policy,
        "audit_count": len(rows),
        "pass_count": len(pass_rows),
        "pass_all": len(rows) > 0 and len(pass_rows) == len(rows),
        "calibration_replay_label_match_count": len(matches),
        "validation_range_rmse_corrected_mean_m": float(np.mean(corrected)) if corrected else float("nan"),
        "validation_range_rmse_corrected_max_m": float(np.max(corrected)) if corrected else float("nan"),
        "target_abs_mean_error_mean_m": float(np.mean(target_abs)) if target_abs else float("nan"),
        "target_abs_mean_error_max_m": float(np.max(target_abs)) if target_abs else float("nan"),
        "target_max_abs_error_max_m": float(np.max(target_max)) if target_max else float("nan"),
        "transfer_matrix_csv": str(transfer_matrix_csv),
        "transfer_low_risk_rmse_limit_m": args.transfer_low_risk_rmse_limit_m,
        "transfer_low_risk_target_limit_m": args.transfer_low_risk_target_limit_m,
        "transfer_risk_target_weight": args.transfer_risk_target_weight,
        "top_label_count": args.top_label_count,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_input_fields": ",".join(RAW_FEATURE_FIELDS),
        "selection_contract": (
            "offline calibration replay truth labels train the selector; runtime target selection uses raw "
            "stereo-center RDF/disparity/image-center features only"
        ),
    }


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Runtime oracle-label selector audit",
        "",
        "Purpose:",
        (
            "Train a runtime selector from calibration-time replay labels, then leave-one-out audit "
            "selection using raw StereoDistance measurements only."
        ),
        "",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_input_fields: {summary.get('selection_input_fields', '')}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Summary:",
    ]
    for key in (
        "selection_policy",
        "audit_count",
        "pass_count",
        "pass_all",
        "calibration_replay_label_match_count",
        "validation_range_rmse_corrected_mean_m",
        "validation_range_rmse_corrected_max_m",
        "target_abs_mean_error_mean_m",
        "target_abs_mean_error_max_m",
        "target_max_abs_error_max_m",
    ):
        value = summary.get(key, "")
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Rows:"])
    for row in rows:
        lines.append(
            "{target_scenario}/{target_seed}: label={label} preliminary={preliminary} selected={selected} "
            "pass={passed} rmse_corrected={rmse} target_max_abs={target_max} reason={reason}".format(
                target_scenario=row.get("target_scenario", ""),
                target_seed=row.get("target_seed", ""),
                label=row.get("calibration_replay_label_source_scenario", ""),
                preliminary=row.get("preliminary_source_scenario", ""),
                selected=row.get("selected_source_scenario", ""),
                passed=row.get("pass_runtime_oracle_label_selector_replay", ""),
                rmse=row.get("validation_range_rmse_corrected_m", ""),
                target_max=row.get("target_max_abs_error_corrected_m", ""),
                reason=row.get("pass_reason", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_selection_counts(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    pairs: dict[tuple[str, str], int] = {}
    target_names: list[str] = []
    selected_names: list[str] = []
    for row in rows:
        target = str(row.get("target_scenario", "") or "input")
        selected = str(row.get("selected_source_scenario", "") or "none")
        pairs[(target, selected)] = pairs.get((target, selected), 0) + 1
        if target not in target_names:
            target_names.append(target)
        if selected not in selected_names:
            selected_names.append(selected)
    if not pairs:
        return
    matrix = np.zeros((len(target_names), len(selected_names)), dtype=float)
    for (target, selected), count in pairs.items():
        matrix[target_names.index(target), selected_names.index(selected)] = count
    fig, ax = plt.subplots(figsize=(max(7.0, 1.3 * len(selected_names)), max(4.5, 0.65 * len(target_names))))
    im = ax.imshow(matrix, cmap="Greens", aspect="auto")
    ax.set_xticks(np.arange(len(selected_names)), selected_names, rotation=35, ha="right")
    ax.set_yticks(np.arange(len(target_names)), target_names)
    ax.set_xlabel("selected source profile")
    ax.set_ylabel("target audit scenario")
    ax.set_title("Runtime oracle-label selector counts")
    for y in range(matrix.shape[0]):
        for x in range(matrix.shape[1]):
            value = int(matrix[y, x])
            if value:
                ax.text(x, y, str(value), ha="center", va="center", color="black")
    fig.colorbar(im, ax=ax, label="count")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> int:
    summary = audit_selector(parse_args())
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
