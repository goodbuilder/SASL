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

from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from select_stereodistance_profile_by_runtime_measurement_guard import FEATURE_NAMES  # noqa: E402
from select_stereodistance_profile_from_catalog import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit whether LOSO full-gap StereoDistance families are separable from raw runtime "
            "measurement features. Uses raw RDF/disparity/image-center aggregate features only; "
            "coverage-gap labels come from offline calibration replay analysis."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--coverage-gap-json", type=Path)
    parser.add_argument("--runtime-audit-csv", type=Path)
    parser.add_argument("--target-scenarios", default="")
    return parser.parse_args()


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def parse_csv_list(value: str) -> list[str]:
    return [part.strip() for part in str(value).split(",") if part.strip()]


def load_target_scenarios(coverage_gap_json: Path, explicit: str) -> list[str]:
    explicit_targets = parse_csv_list(explicit)
    if explicit_targets:
        return explicit_targets
    payload = read_json(coverage_gap_json)
    targets = payload.get("full_gap_scenarios", [])
    if not isinstance(targets, list):
        return []
    return [str(item) for item in targets if str(item).strip()]


def feature_columns(rows: Sequence[Mapping[str, str]]) -> list[str]:
    available = set(rows[0].keys()) if rows else set()
    return [f"feature_{name}" for name in FEATURE_NAMES if f"feature_{name}" in available]


def finite_feature_values(rows: Sequence[Mapping[str, str]], feature: str) -> np.ndarray:
    values = [finite_float(row.get(feature), float("nan")) for row in rows]
    return np.asarray([value for value in values if math.isfinite(value)], dtype=float)


def best_threshold_for_feature(
    positives: np.ndarray,
    negatives: np.ndarray,
) -> dict[str, object]:
    if positives.size == 0 or negatives.size == 0:
        return {
            "threshold": "",
            "direction": "",
            "accuracy": 0.0,
            "true_positive_count": 0,
            "false_positive_count": 0,
            "false_negative_count": int(positives.size),
            "true_negative_count": int(negatives.size),
        }
    values = np.unique(np.concatenate([positives, negatives]))
    thresholds = []
    if values.size == 1:
        thresholds = [float(values[0])]
    else:
        thresholds.append(float(values[0]) - 1.0e-9)
        thresholds.extend(float((left + right) * 0.5) for left, right in zip(values[:-1], values[1:]))
        thresholds.append(float(values[-1]) + 1.0e-9)
    best: dict[str, object] | None = None
    total = positives.size + negatives.size
    for threshold in thresholds:
        for direction in ("ge", "le"):
            if direction == "ge":
                tp = int(np.sum(positives >= threshold))
                fp = int(np.sum(negatives >= threshold))
            else:
                tp = int(np.sum(positives <= threshold))
                fp = int(np.sum(negatives <= threshold))
            fn = int(positives.size - tp)
            tn = int(negatives.size - fp)
            accuracy = float((tp + tn) / total)
            score = (accuracy, tp, tn, -fp, -fn)
            if best is None or score > best["_score"]:
                best = {
                    "threshold": float(threshold),
                    "direction": direction,
                    "accuracy": accuracy,
                    "true_positive_count": tp,
                    "false_positive_count": fp,
                    "false_negative_count": fn,
                    "true_negative_count": tn,
                    "_score": score,
                }
    assert best is not None
    best.pop("_score", None)
    return best


def separability_row(
    *,
    target_scenario: str,
    feature: str,
    positive_rows: Sequence[Mapping[str, str]],
    negative_rows: Sequence[Mapping[str, str]],
) -> dict[str, object]:
    positives = finite_feature_values(positive_rows, feature)
    negatives = finite_feature_values(negative_rows, feature)
    pos_mean = float(np.mean(positives)) if positives.size else float("nan")
    neg_mean = float(np.mean(negatives)) if negatives.size else float("nan")
    pooled_std = float(np.sqrt((np.var(positives) + np.var(negatives)) * 0.5)) if positives.size and negatives.size else float("nan")
    effect = abs(pos_mean - neg_mean) / max(pooled_std, 1.0e-12) if math.isfinite(pooled_std) else float("nan")
    threshold = best_threshold_for_feature(positives, negatives)
    return {
        "target_scenario": target_scenario,
        "feature": feature,
        "positive_count": int(positives.size),
        "negative_count": int(negatives.size),
        "positive_mean": pos_mean if math.isfinite(pos_mean) else "",
        "negative_mean": neg_mean if math.isfinite(neg_mean) else "",
        "absolute_mean_delta": abs(pos_mean - neg_mean) if math.isfinite(pos_mean) and math.isfinite(neg_mean) else "",
        "pooled_std": pooled_std if math.isfinite(pooled_std) else "",
        "standardized_effect_size": effect if math.isfinite(effect) else "",
        **threshold,
        "axis_contract": RDF_AXIS_CONTRACT,
        "runtime_input_contract": "raw runtime feature aggregates only; labels are offline coverage-gap diagnostics",
    }


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "StereoDistance gap feature separability audit",
        "",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        "runtime_input_contract: raw runtime feature aggregates only; labels are offline coverage-gap diagnostics",
        "",
        "Summary:",
    ]
    for key in (
        "target_scenarios",
        "best_feature",
        "best_target_scenario",
        "best_accuracy",
        "best_standardized_effect_size",
        "detection_recommendation",
    ):
        value = summary.get(key, "")
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Top feature rows:"])
    for row in rows[:10]:
        lines.append(
            "{target} {feature}: acc={acc}, effect={effect}, threshold={direction} {threshold}".format(
                target=row.get("target_scenario", ""),
                feature=row.get("feature", ""),
                acc=row.get("accuracy", ""),
                effect=row.get("standardized_effect_size", ""),
                direction=row.get("direction", ""),
                threshold=row.get("threshold", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_top_features(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    top_rows = list(rows[: min(12, len(rows))])
    labels = [f"{row.get('target_scenario', '')}\n{row.get('feature', '').removeprefix('feature_')}" for row in top_rows]
    accuracy = np.asarray([finite_float(row.get("accuracy"), 0.0) for row in top_rows], dtype=float)
    effect = np.asarray([finite_float(row.get("standardized_effect_size"), 0.0) for row in top_rows], dtype=float)
    x = np.arange(len(top_rows))
    fig, axes = plt.subplots(2, 1, figsize=(max(10, len(top_rows) * 0.9), 7), sharex=True, constrained_layout=True)
    axes[0].bar(x, accuracy, color="tab:green")
    axes[0].set_ylim(0.0, 1.05)
    axes[0].set_ylabel("threshold accuracy")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[1].bar(x, effect, color="tab:blue")
    axes[1].set_ylabel("standardized effect")
    axes[1].set_xticks(x, labels, rotation=45, ha="right")
    axes[1].grid(True, axis="y", alpha=0.3)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def audit_gap_feature_separability(
    *,
    sweep_dir: Path,
    output_dir: Path,
    coverage_gap_json: Path | None = None,
    runtime_audit_csv: Path | None = None,
    target_scenarios_text: str = "",
) -> dict[str, object]:
    gap_json = coverage_gap_json or (sweep_dir / "loso_coverage_gap_summary" / "loso_coverage_gap_summary.json")
    runtime_csv = runtime_audit_csv or (
        sweep_dir
        / "runtime_measurement_guard_policy_audit_loso"
        / "runs"
        / "all_catalog_transfer_risk_fallback"
        / "runtime_measurement_guard_audit.csv"
    )
    targets = load_target_scenarios(gap_json, target_scenarios_text)
    rows = read_csv(runtime_csv)
    features = feature_columns(rows)
    output_rows: list[dict[str, object]] = []
    for target in targets:
        positives = [row for row in rows if row.get("target_scenario") == target]
        negatives = [row for row in rows if row.get("target_scenario") != target]
        for feature in features:
            output_rows.append(
                separability_row(
                    target_scenario=target,
                    feature=feature,
                    positive_rows=positives,
                    negative_rows=negatives,
                )
            )
    output_rows = sorted(
        output_rows,
        key=lambda row: (
            -finite_float(row.get("accuracy"), 0.0),
            -finite_float(row.get("standardized_effect_size"), 0.0),
            str(row.get("target_scenario", "")),
            str(row.get("feature", "")),
        ),
    )
    best = output_rows[0] if output_rows else {}
    best_accuracy = finite_float(best.get("accuracy"), float("nan"))
    best_effect = finite_float(best.get("standardized_effect_size"), float("nan"))
    detection_recommendation = (
        "raw_feature_gap_guard_candidate"
        if math.isfinite(best_accuracy) and best_accuracy >= 0.95 and math.isfinite(best_effect) and best_effect >= 2.0
        else "raw_features_insufficient_for_gap_guard"
    )
    summary = {
        "mode": "stereodistance_gap_feature_separability",
        "sweep_dir": str(sweep_dir),
        "coverage_gap_json": str(gap_json),
        "runtime_audit_csv": str(runtime_csv),
        "target_scenarios": targets,
        "feature_count": len(features),
        "row_count": len(output_rows),
        "best_target_scenario": best.get("target_scenario", ""),
        "best_feature": best.get("feature", ""),
        "best_accuracy": best_accuracy if math.isfinite(best_accuracy) else "",
        "best_standardized_effect_size": best_effect if math.isfinite(best_effect) else "",
        "best_threshold": best.get("threshold", ""),
        "best_direction": best.get("direction", ""),
        "detection_recommendation": detection_recommendation,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "raw runtime feature aggregates only; labels are offline coverage-gap diagnostics",
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "gap_feature_separability.csv", output_rows)
    write_json(output_dir / "gap_feature_separability_summary.json", summary)
    write_summary_txt(output_dir / "gap_feature_separability_summary.txt", summary, output_rows)
    plot_top_features(output_dir / "gap_feature_separability_top_features.png", output_rows)
    return summary


def main() -> int:
    args = parse_args()
    summary = audit_gap_feature_separability(
        sweep_dir=args.sweep_dir,
        output_dir=args.output_dir,
        coverage_gap_json=args.coverage_gap_json,
        runtime_audit_csv=args.runtime_audit_csv,
        target_scenarios_text=args.target_scenarios,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
