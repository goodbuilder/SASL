#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit a truth-free runtime selector portfolio over existing selector replay CSVs. "
            "Offline replay metrics train a simple raw-feature threshold rule; runtime selection uses "
            "only raw RDF/disparity feature columns."
        )
    )
    parser.add_argument(
        "--candidate",
        action="append",
        required=True,
        help="Candidate selector in label=/path/to/audit.csv form.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument(
        "--objective",
        choices=("target_max_then_rmse", "rmse_then_target_max", "target_cap_then_rmse"),
        default="target_max_then_rmse",
    )
    parser.add_argument(
        "--feature-names",
        default="",
        help=(
            "Optional comma-separated feature column list. Use this to keep the runtime rule low-dimensional "
            "when the candidate CSVs expose many feature_* columns."
        ),
    )
    parser.add_argument(
        "--max-depth",
        type=int,
        choices=(1, 2),
        default=1,
        help="Maximum depth of the raw-feature selector rule. Depth 1 is a single threshold stump.",
    )
    parser.add_argument("--include-target-run", action="store_true")
    parser.add_argument(
        "--precision-override-label",
        default="",
        help=(
            "Optional candidate label to use only when a raw-feature threshold has zero false positives "
            "on the current audit rows. This is a diagnostic conservative override, not an oracle runtime input."
        ),
    )
    parser.add_argument(
        "--precision-override-baseline-label",
        default="",
        help="Candidate label used when the conservative override condition is false. Defaults to the first candidate.",
    )
    parser.add_argument(
        "--precision-override-feature",
        default="",
        help="Optional feature name for the conservative override threshold. Defaults to the best zero-FP feature.",
    )
    parser.add_argument(
        "--precision-override-direction",
        choices=("", "le", "ge"),
        default="",
        help="Optional direction for --precision-override-feature.",
    )
    parser.add_argument(
        "--precision-override-threshold",
        type=float,
        default=None,
        help="Optional threshold for --precision-override-feature.",
    )
    return parser.parse_args()


def finite_float(value: object, default: float = float("nan")) -> float:
    if value in (None, ""):
        return default
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


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


def parse_candidate(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise ValueError(f"candidate must be label=path, got {value!r}")
    label, path = value.split("=", 1)
    label = label.strip()
    if not label:
        raise ValueError("candidate label is empty")
    return label, Path(path.strip())


def row_key(row: Mapping[str, str]) -> tuple[str, str]:
    return (str(row.get("target_scenario", "")), str(row.get("target_seed", "")))


def row_pass(row: Mapping[str, str], target_error_limit_m: float) -> bool:
    pass_fields = (
        "pass_runtime_measurement_guard_replay",
        "pass_runtime_feature_selector_replay",
        "pass_selector_portfolio_replay",
    )
    explicit = next((str(row.get(name, "")).strip() for name in pass_fields if str(row.get(name, "")).strip()), "")
    if explicit in {"True", "true", "1", "PASS", "pass"}:
        return True
    if explicit in {"False", "false", "0", "FAIL", "fail"}:
        return False
    target_max = finite_float(row.get("target_max_abs_error_corrected_m"), float("inf"))
    improvement = finite_float(row.get("validation_range_rmse_improvement_m"), float("nan"))
    return math.isfinite(target_max) and target_max <= target_error_limit_m and math.isfinite(improvement) and improvement > 0.0


def feature_names(rows_by_label: Mapping[str, Mapping[tuple[str, str], Mapping[str, str]]]) -> list[str]:
    common: set[str] | None = None
    for rows in rows_by_label.values():
        for row in rows.values():
            names = {
                name
                for name, value in row.items()
                if name.startswith("feature_") and math.isfinite(finite_float(value))
            }
            common = names if common is None else common & names
    return sorted(common or [])


def load_candidates(candidate_specs: Sequence[str]) -> tuple[list[str], dict[str, dict[tuple[str, str], dict[str, str]]]]:
    labels: list[str] = []
    rows_by_label: dict[str, dict[tuple[str, str], dict[str, str]]] = {}
    for spec in candidate_specs:
        label, path = parse_candidate(spec)
        labels.append(label)
        rows_by_label[label] = {row_key(row): row for row in read_csv(path)}
    if len(set(labels)) != len(labels):
        raise ValueError("candidate labels must be unique")
    common_keys: set[tuple[str, str]] | None = None
    for rows in rows_by_label.values():
        keys = set(rows.keys())
        common_keys = keys if common_keys is None else common_keys & keys
    if not common_keys:
        raise ValueError("candidate CSVs have no common target rows")
    for label in labels:
        rows_by_label[label] = {key: rows_by_label[label][key] for key in sorted(common_keys)}
    return labels, rows_by_label


def candidate_metrics(
    rows_by_label: Mapping[str, Mapping[tuple[str, str], Mapping[str, str]]],
    key: tuple[str, str],
    label: str,
    target_error_limit_m: float,
) -> dict[str, object]:
    row = rows_by_label[label][key]
    return {
        "label": label,
        "pass": row_pass(row, target_error_limit_m),
        "validation_range_rmse_corrected_m": finite_float(row.get("validation_range_rmse_corrected_m")),
        "validation_range_rmse_improvement_m": finite_float(row.get("validation_range_rmse_improvement_m")),
        "target_abs_mean_error_corrected_m": finite_float(row.get("target_abs_mean_error_corrected_m")),
        "target_max_abs_error_corrected_m": finite_float(row.get("target_max_abs_error_corrected_m")),
        "selected_source_scenario": row.get("selected_source_scenario", ""),
        "selected_source_seed": row.get("selected_source_seed", ""),
        "selected_method": row.get("selected_method", row.get("selected_source_method", "")),
    }


def oracle_label_for_key(
    rows_by_label: Mapping[str, Mapping[tuple[str, str], Mapping[str, str]]],
    key: tuple[str, str],
    labels: Sequence[str],
    *,
    target_error_limit_m: float,
    objective: str,
) -> str:
    metrics = {
        label: candidate_metrics(rows_by_label, key, label, target_error_limit_m)
        for label in labels
    }

    def score(label: str) -> tuple[object, ...]:
        row = metrics[label]
        passed = bool(row["pass"])
        rmse = finite_float(row["validation_range_rmse_corrected_m"], float("inf"))
        target_max = finite_float(row["target_max_abs_error_corrected_m"], float("inf"))
        if objective == "rmse_then_target_max":
            return (0 if passed else 1, rmse, target_max, label)
        if objective == "target_cap_then_rmse":
            return (0 if passed else 1, max(0.0, target_max - target_error_limit_m), rmse, target_max, label)
        return (0 if passed else 1, target_max, rmse, label)

    return min(labels, key=score)


def rule_label(
    rule: Mapping[str, object],
    feature_row: Mapping[str, str],
) -> str:
    if rule["kind"] == "tree":
        value = finite_float(feature_row.get(str(rule["feature_name"])), float("nan"))
        if not math.isfinite(value):
            return rule_label(rule["missing_rule"], feature_row)  # type: ignore[index]
        branch = "left_rule" if value <= float(rule["threshold"]) else "right_rule"
        return rule_label(rule[branch], feature_row)  # type: ignore[index]
    if rule["kind"] == "constant":
        return str(rule["constant_label"])
    value = finite_float(feature_row.get(str(rule["feature_name"])), float("nan"))
    if not math.isfinite(value):
        return str(rule["missing_label"])
    return str(rule["left_label"] if value <= float(rule["threshold"]) else rule["right_label"])


def score_rule(
    rule: Mapping[str, object],
    keys: Sequence[tuple[str, str]],
    labels: Sequence[str],
    rows_by_label: Mapping[str, Mapping[tuple[str, str], Mapping[str, str]]],
    *,
    target_error_limit_m: float,
    objective: str,
) -> tuple[object, ...]:
    selected: list[dict[str, object]] = []
    for key in keys:
        feature_row = rows_by_label[labels[0]][key]
        label = rule_label(rule, feature_row)
        selected.append(candidate_metrics(rows_by_label, key, label, target_error_limit_m))
    pass_count = sum(1 for row in selected if bool(row["pass"]))
    target_values = [finite_float(row["target_max_abs_error_corrected_m"], float("inf")) for row in selected]
    rmse_values = [finite_float(row["validation_range_rmse_corrected_m"], float("inf")) for row in selected]
    if objective == "rmse_then_target_max":
        primary = float(np.mean(rmse_values)) if rmse_values else float("inf")
        secondary = max(target_values, default=float("inf"))
    elif objective == "target_cap_then_rmse":
        target_max = max(target_values, default=float("inf"))
        primary = max(0.0, target_max - float(target_error_limit_m))
        secondary = float(np.mean(rmse_values)) if rmse_values else float("inf")
    else:
        primary = max(target_values, default=float("inf"))
        secondary = float(np.mean(rmse_values)) if rmse_values else float("inf")
    return (-pass_count, primary, secondary, str(rule))


def threshold_diagnostics_for_label(
    rows: Sequence[Mapping[str, object]],
    *,
    feature_name: str,
    oracle_label: str,
) -> dict[str, object]:
    positives = [
        finite_float(row.get(feature_name), float("nan"))
        for row in rows
        if str(row.get("oracle_candidate", "")) == oracle_label
    ]
    negatives = [
        finite_float(row.get(feature_name), float("nan"))
        for row in rows
        if str(row.get("oracle_candidate", "")) != oracle_label
    ]
    positives = [value for value in positives if math.isfinite(value)]
    negatives = [value for value in negatives if math.isfinite(value)]
    if not positives or not negatives:
        return {
            "feature_name": feature_name,
            "oracle_label": oracle_label,
            "positive_count": len(positives),
            "negative_count": len(negatives),
            "threshold": "",
            "direction": "",
            "accuracy": 0.0,
            "true_positive_count": 0,
            "false_positive_count": 0,
            "false_negative_count": len(positives),
            "true_negative_count": len(negatives),
            "positive_mean": float(np.mean(positives)) if positives else "",
            "negative_mean": float(np.mean(negatives)) if negatives else "",
            "standardized_effect_size": "",
        }
    values = sorted(set(positives + negatives))
    thresholds: list[float]
    if len(values) == 1:
        thresholds = [float(values[0])]
    else:
        thresholds = [float(values[0]) - 1.0e-9]
        thresholds.extend(float((left + right) * 0.5) for left, right in zip(values[:-1], values[1:]))
        thresholds.append(float(values[-1]) + 1.0e-9)
    best: dict[str, object] | None = None
    total = len(positives) + len(negatives)
    for threshold in thresholds:
        for direction in ("ge", "le"):
            if direction == "ge":
                tp = sum(1 for value in positives if value >= threshold)
                fp = sum(1 for value in negatives if value >= threshold)
            else:
                tp = sum(1 for value in positives if value <= threshold)
                fp = sum(1 for value in negatives if value <= threshold)
            fn = len(positives) - tp
            tn = len(negatives) - fp
            accuracy = float((tp + tn) / total)
            score = (accuracy, tp, tn, -fp, -fn)
            if best is None or score > best["_score"]:
                best = {
                    "feature_name": feature_name,
                    "oracle_label": oracle_label,
                    "positive_count": len(positives),
                    "negative_count": len(negatives),
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
    positive_mean = float(np.mean(positives))
    negative_mean = float(np.mean(negatives))
    pooled_std = float(np.sqrt((np.var(positives) + np.var(negatives)) * 0.5))
    best.pop("_score", None)
    best.update(
        {
            "positive_mean": positive_mean,
            "negative_mean": negative_mean,
            "standardized_effect_size": abs(positive_mean - negative_mean) / max(pooled_std, 1.0e-12),
            "axis_contract": RDF_AXIS_CONTRACT,
            "runtime_input_contract": (
                "feature threshold uses raw stereo-center RDF/disparity aggregates; oracle label is offline replay only"
            ),
        }
    )
    return best


def oracle_separability_rows(
    rows: Sequence[Mapping[str, object]],
    labels: Sequence[str],
    features: Sequence[str],
) -> list[dict[str, object]]:
    diagnostics = [
        threshold_diagnostics_for_label(rows, feature_name=feature, oracle_label=label)
        for label in labels
        for feature in features
    ]
    return sorted(
        diagnostics,
        key=lambda row: (
            -finite_float(row.get("accuracy"), 0.0),
            -finite_float(row.get("standardized_effect_size"), 0.0),
            str(row.get("oracle_label", "")),
            str(row.get("feature_name", "")),
        ),
    )


def zero_false_positive_override_rule(
    separability_rows: Sequence[Mapping[str, object]],
    *,
    override_label: str,
    feature_name: str = "",
    direction: str = "",
    threshold: float | None = None,
) -> dict[str, object]:
    candidates = [
        row
        for row in separability_rows
        if str(row.get("oracle_label", "")) == override_label
        and int(finite_float(row.get("false_positive_count"), float("inf"))) == 0
        and int(finite_float(row.get("true_positive_count"), 0.0)) > 0
    ]
    if feature_name:
        normalized_feature = feature_name if feature_name.startswith("feature_") else f"feature_{feature_name}"
        candidates = [row for row in candidates if str(row.get("feature_name", "")) == normalized_feature]
    if direction:
        candidates = [row for row in candidates if str(row.get("direction", "")) == direction]
    if threshold is not None:
        candidates = [dict(row, threshold=float(threshold)) for row in candidates]
    if not candidates:
        raise ValueError(f"no zero-false-positive override rule found for label {override_label!r}")
    return dict(
        min(
            candidates,
            key=lambda row: (
                -int(finite_float(row.get("true_positive_count"), 0.0)),
                -finite_float(row.get("accuracy"), 0.0),
                -finite_float(row.get("standardized_effect_size"), 0.0),
                str(row.get("feature_name", "")),
            ),
        )
    )


def precision_override_selects(rule: Mapping[str, object], row: Mapping[str, object]) -> bool:
    value = finite_float(row.get(str(rule.get("feature_name", ""))), float("nan"))
    if not math.isfinite(value):
        return False
    threshold = finite_float(rule.get("threshold"), float("nan"))
    if not math.isfinite(threshold):
        return False
    if str(rule.get("direction", "")) == "ge":
        return value >= threshold
    return value <= threshold


def audit_precision_override(
    audit_rows: Sequence[Mapping[str, object]],
    separability_rows: Sequence[Mapping[str, object]],
    *,
    labels: Sequence[str],
    override_label: str,
    baseline_label: str,
    target_error_limit_m: float,
    feature_name: str = "",
    direction: str = "",
    threshold: float | None = None,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    if override_label not in labels:
        raise ValueError(f"precision override label {override_label!r} is not one of {labels}")
    if baseline_label not in labels:
        raise ValueError(f"precision override baseline label {baseline_label!r} is not one of {labels}")
    rule = zero_false_positive_override_rule(
        separability_rows,
        override_label=override_label,
        feature_name=feature_name,
        direction=direction,
        threshold=threshold,
    )
    output_rows: list[dict[str, object]] = []
    for row in audit_rows:
        candidate_payload = json.loads(str(row.get("candidate_metrics_json", "{}")))
        selected_label = override_label if precision_override_selects(rule, row) else baseline_label
        selected = candidate_payload[selected_label]
        oracle_label = str(row.get("oracle_candidate", ""))
        oracle = candidate_payload.get(oracle_label, {})
        selected_rmse = finite_float(selected.get("validation_range_rmse_corrected_m"), float("nan"))
        oracle_rmse = finite_float(oracle.get("validation_range_rmse_corrected_m"), float("nan"))
        selected_target = finite_float(selected.get("target_max_abs_error_corrected_m"), float("nan"))
        oracle_target = finite_float(oracle.get("target_max_abs_error_corrected_m"), float("nan"))
        output_rows.append(
            {
                "target_scenario": row.get("target_scenario", ""),
                "target_seed": row.get("target_seed", ""),
                "selected_candidate": selected_label,
                "oracle_candidate": oracle_label,
                "selected_matches_oracle": selected_label == oracle_label,
                "override_applied": selected_label == override_label,
                "pass_precision_override_replay": bool(selected.get("pass")),
                "validation_range_rmse_corrected_m": selected.get("validation_range_rmse_corrected_m", ""),
                "validation_range_rmse_improvement_m": selected.get("validation_range_rmse_improvement_m", ""),
                "target_abs_mean_error_corrected_m": selected.get("target_abs_mean_error_corrected_m", ""),
                "target_max_abs_error_corrected_m": selected.get("target_max_abs_error_corrected_m", ""),
                "oracle_validation_range_rmse_corrected_m": oracle.get("validation_range_rmse_corrected_m", ""),
                "oracle_target_max_abs_error_corrected_m": oracle.get("target_max_abs_error_corrected_m", ""),
                "rmse_regret_m": (
                    selected_rmse - oracle_rmse if math.isfinite(selected_rmse) and math.isfinite(oracle_rmse) else ""
                ),
                "target_max_regret_m": (
                    selected_target - oracle_target if math.isfinite(selected_target) and math.isfinite(oracle_target) else ""
                ),
                "rule_feature_name": rule.get("feature_name", ""),
                "rule_direction": rule.get("direction", ""),
                "rule_threshold": rule.get("threshold", ""),
                "override_label": override_label,
                "baseline_label": baseline_label,
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "selection_contract": (
                    "Conservative override rule is learned offline from replay diagnostics; runtime apply uses only "
                    "the named raw RDF/disparity feature threshold."
                ),
            }
        )
    pass_count = sum(1 for row in output_rows if bool(row["pass_precision_override_replay"]))
    oracle_match_count = sum(1 for row in output_rows if bool(row["selected_matches_oracle"]))
    override_count = sum(1 for row in output_rows if bool(row["override_applied"]))
    rmse_values = [
        finite_float(row["validation_range_rmse_corrected_m"])
        for row in output_rows
        if math.isfinite(finite_float(row["validation_range_rmse_corrected_m"]))
    ]
    target_values = [
        finite_float(row["target_max_abs_error_corrected_m"])
        for row in output_rows
        if math.isfinite(finite_float(row["target_max_abs_error_corrected_m"]))
    ]
    rmse_regret_values = [
        finite_float(row["rmse_regret_m"])
        for row in output_rows
        if math.isfinite(finite_float(row["rmse_regret_m"]))
    ]
    target_regret_values = [
        finite_float(row["target_max_regret_m"])
        for row in output_rows
        if math.isfinite(finite_float(row["target_max_regret_m"]))
    ]
    summary = {
        "mode": "runtime_selector_portfolio_precision_override",
        "candidate_labels": ",".join(labels),
        "override_label": override_label,
        "baseline_label": baseline_label,
        "rule_feature_name": rule.get("feature_name", ""),
        "rule_direction": rule.get("direction", ""),
        "rule_threshold": rule.get("threshold", ""),
        "rule_true_positive_count": rule.get("true_positive_count", ""),
        "rule_false_positive_count": rule.get("false_positive_count", ""),
        "audit_count": len(output_rows),
        "override_count": override_count,
        "pass_count": pass_count,
        "pass_all": bool(output_rows) and pass_count == len(output_rows),
        "selected_oracle_match_count": oracle_match_count,
        "selected_oracle_match_fraction": float(oracle_match_count / len(output_rows)) if output_rows else None,
        "validation_range_rmse_corrected_mean_m": float(np.mean(rmse_values)) if rmse_values else None,
        "validation_range_rmse_corrected_max_m": max(rmse_values, default=None),
        "target_max_abs_error_max_m": max(target_values, default=None),
        "rmse_regret_mean_m": float(np.mean(rmse_regret_values)) if rmse_regret_values else None,
        "rmse_regret_max_m": max(rmse_regret_values, default=None),
        "target_max_regret_max_m": max(target_regret_values, default=None),
        "target_error_limit_m": target_error_limit_m,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Runtime-safe conservative override: use baseline candidate unless a raw feature crosses "
            "the zero-false-positive offline diagnostic threshold."
        ),
    }
    return summary, output_rows


def candidate_rules(
    keys: Sequence[tuple[str, str]],
    labels: Sequence[str],
    rows_by_label: Mapping[str, Mapping[tuple[str, str], Mapping[str, str]]],
    features: Sequence[str],
) -> list[dict[str, object]]:
    rules: list[dict[str, object]] = [
        {
            "kind": "constant",
            "constant_label": label,
            "missing_label": label,
        }
        for label in labels
    ]
    if not keys:
        return rules
    feature_rows = rows_by_label[labels[0]]
    for feature in features:
        values = sorted(
            {
                finite_float(feature_rows[key].get(feature), float("nan"))
                for key in keys
                if math.isfinite(finite_float(feature_rows[key].get(feature), float("nan")))
            }
        )
        thresholds = [(left + right) * 0.5 for left, right in zip(values, values[1:]) if left != right]
        for threshold in thresholds:
            for left_label in labels:
                for right_label in labels:
                    rules.append(
                        {
                            "kind": "threshold",
                            "feature_name": feature,
                            "threshold": float(threshold),
                            "left_label": left_label,
                            "right_label": right_label,
                            "missing_label": left_label,
                        }
                    )
    return rules


def select_best_rule(
    train_keys: Sequence[tuple[str, str]],
    labels: Sequence[str],
    rows_by_label: Mapping[str, Mapping[tuple[str, str], Mapping[str, str]]],
    features: Sequence[str],
    *,
    target_error_limit_m: float,
    objective: str,
    max_depth: int = 1,
) -> dict[str, object]:
    if max_depth <= 1:
        rules = candidate_rules(train_keys, labels, rows_by_label, features)
    else:
        rules = candidate_tree_rules(
            train_keys,
            labels,
            rows_by_label,
            features,
            target_error_limit_m=target_error_limit_m,
            objective=objective,
        )
    return dict(
        min(
            rules,
            key=lambda rule: score_rule(
                rule,
                train_keys,
                labels,
                rows_by_label,
                target_error_limit_m=target_error_limit_m,
                objective=objective,
            ),
        )
    )


def best_rule_for_subset(
    subset_keys: Sequence[tuple[str, str]],
    labels: Sequence[str],
    rows_by_label: Mapping[str, Mapping[tuple[str, str], Mapping[str, str]]],
    features: Sequence[str],
    *,
    target_error_limit_m: float,
    objective: str,
) -> dict[str, object]:
    if not subset_keys:
        return {
            "kind": "constant",
            "constant_label": labels[0],
            "missing_label": labels[0],
        }
    return select_best_rule(
        subset_keys,
        labels,
        rows_by_label,
        features,
        target_error_limit_m=target_error_limit_m,
        objective=objective,
        max_depth=1,
    )


def candidate_tree_rules(
    keys: Sequence[tuple[str, str]],
    labels: Sequence[str],
    rows_by_label: Mapping[str, Mapping[tuple[str, str], Mapping[str, str]]],
    features: Sequence[str],
    *,
    target_error_limit_m: float,
    objective: str,
) -> list[dict[str, object]]:
    rules = candidate_rules(keys, labels, rows_by_label, features)
    if not keys:
        return rules
    feature_rows = rows_by_label[labels[0]]
    for feature in features:
        values = sorted(
            {
                finite_float(feature_rows[key].get(feature), float("nan"))
                for key in keys
                if math.isfinite(finite_float(feature_rows[key].get(feature), float("nan")))
            }
        )
        thresholds = [(left + right) * 0.5 for left, right in zip(values, values[1:]) if left != right]
        for threshold in thresholds:
            left_keys = [
                key
                for key in keys
                if finite_float(feature_rows[key].get(feature), float("inf")) <= threshold
            ]
            right_keys = [key for key in keys if key not in set(left_keys)]
            if not left_keys or not right_keys:
                continue
            left_rule = best_rule_for_subset(
                left_keys,
                labels,
                rows_by_label,
                features,
                target_error_limit_m=target_error_limit_m,
                objective=objective,
            )
            right_rule = best_rule_for_subset(
                right_keys,
                labels,
                rows_by_label,
                features,
                target_error_limit_m=target_error_limit_m,
                objective=objective,
            )
            rules.append(
                {
                    "kind": "tree",
                    "feature_name": feature,
                    "threshold": float(threshold),
                    "left_rule": left_rule,
                    "right_rule": right_rule,
                    "missing_rule": left_rule,
                }
            )
    return rules


def audit_selector_portfolio(
    candidate_specs: Sequence[str],
    output_dir: Path,
    *,
    target_error_limit_m: float = 10.0,
    objective: str = "target_max_then_rmse",
    selected_feature_names: Sequence[str] = (),
    include_target_run: bool = False,
    max_depth: int = 1,
    precision_override_label: str = "",
    precision_override_baseline_label: str = "",
    precision_override_feature: str = "",
    precision_override_direction: str = "",
    precision_override_threshold: float | None = None,
) -> dict[str, object]:
    labels, rows_by_label = load_candidates(candidate_specs)
    keys = sorted(next(iter(rows_by_label.values())).keys())
    features = feature_names(rows_by_label)
    if selected_feature_names:
        requested = [name if name.startswith("feature_") else f"feature_{name}" for name in selected_feature_names]
        missing = [name for name in requested if name not in features]
        if missing:
            raise ValueError(f"requested feature names are missing from candidate CSVs: {missing}")
        features = requested
    if not features:
        raise ValueError("candidate CSVs expose no common finite feature_* columns")
    audit_rows: list[dict[str, object]] = []
    for key in keys:
        train_keys = keys if include_target_run else [candidate for candidate in keys if candidate != key]
        rule = select_best_rule(
            train_keys,
            labels,
            rows_by_label,
            features,
            target_error_limit_m=target_error_limit_m,
            objective=objective,
            max_depth=max_depth,
        )
        feature_row = rows_by_label[labels[0]][key]
        selected_label = rule_label(rule, feature_row)
        selected = candidate_metrics(rows_by_label, key, selected_label, target_error_limit_m)
        candidate_payload = {
            label: candidate_metrics(rows_by_label, key, label, target_error_limit_m)
            for label in labels
        }
        oracle_label = oracle_label_for_key(
            rows_by_label,
            key,
            labels,
            target_error_limit_m=target_error_limit_m,
            objective=objective,
        )
        oracle = candidate_payload[oracle_label]
        selected_rmse = finite_float(selected["validation_range_rmse_corrected_m"], float("nan"))
        oracle_rmse = finite_float(oracle["validation_range_rmse_corrected_m"], float("nan"))
        selected_target_max = finite_float(selected["target_max_abs_error_corrected_m"], float("nan"))
        oracle_target_max = finite_float(oracle["target_max_abs_error_corrected_m"], float("nan"))
        audit_row = {
                "target_scenario": key[0],
                "target_seed": key[1],
                "selected_candidate": selected_label,
                "oracle_candidate": oracle_label,
                "selected_matches_oracle": selected_label == oracle_label,
                "pass_selector_portfolio_replay": selected["pass"],
                "validation_range_rmse_corrected_m": selected["validation_range_rmse_corrected_m"],
                "validation_range_rmse_improvement_m": selected["validation_range_rmse_improvement_m"],
                "target_abs_mean_error_corrected_m": selected["target_abs_mean_error_corrected_m"],
                "target_max_abs_error_corrected_m": selected["target_max_abs_error_corrected_m"],
                "oracle_validation_range_rmse_corrected_m": oracle["validation_range_rmse_corrected_m"],
                "oracle_target_max_abs_error_corrected_m": oracle["target_max_abs_error_corrected_m"],
                "oracle_selected_method": oracle["selected_method"],
                "rmse_regret_m": selected_rmse - oracle_rmse if math.isfinite(selected_rmse) and math.isfinite(oracle_rmse) else "",
                "target_max_regret_m": (
                    selected_target_max - oracle_target_max
                    if math.isfinite(selected_target_max) and math.isfinite(oracle_target_max)
                    else ""
                ),
                "selected_source_scenario": selected["selected_source_scenario"],
                "selected_source_seed": selected["selected_source_seed"],
                "selected_method": selected["selected_method"],
                "rule_kind": rule["kind"],
                "rule_feature_name": rule.get("feature_name", ""),
                "rule_threshold": rule.get("threshold", ""),
                "rule_left_label": rule.get("left_label", ""),
                "rule_right_label": rule.get("right_label", ""),
                "rule_constant_label": rule.get("constant_label", ""),
                "rule_json": json.dumps(rule, ensure_ascii=False, allow_nan=False),
                "include_target_run": include_target_run,
                "max_depth": max_depth,
                "objective": objective,
                "candidate_metrics_json": json.dumps(candidate_payload, ensure_ascii=False, allow_nan=False),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "selection_contract": (
                    "Portfolio rule is trained offline from replay metrics. Runtime selection consumes only "
                    "raw stereo-center RDF/disparity feature columns from the candidate audit rows."
                ),
            }
        audit_row.update({feature: feature_row.get(feature, "") for feature in features})
        audit_rows.append(audit_row)
    pass_count = sum(1 for row in audit_rows if bool(row["pass_selector_portfolio_replay"]))
    oracle_match_count = sum(1 for row in audit_rows if bool(row["selected_matches_oracle"]))
    target_values = [
        finite_float(row["target_max_abs_error_corrected_m"])
        for row in audit_rows
        if math.isfinite(finite_float(row["target_max_abs_error_corrected_m"]))
    ]
    rmse_values = [
        finite_float(row["validation_range_rmse_corrected_m"])
        for row in audit_rows
        if math.isfinite(finite_float(row["validation_range_rmse_corrected_m"]))
    ]
    oracle_rmse_values = [
        finite_float(row["oracle_validation_range_rmse_corrected_m"])
        for row in audit_rows
        if math.isfinite(finite_float(row["oracle_validation_range_rmse_corrected_m"]))
    ]
    rmse_regret_values = [
        finite_float(row["rmse_regret_m"])
        for row in audit_rows
        if math.isfinite(finite_float(row["rmse_regret_m"]))
    ]
    target_regret_values = [
        finite_float(row["target_max_regret_m"])
        for row in audit_rows
        if math.isfinite(finite_float(row["target_max_regret_m"]))
    ]
    summary = {
        "mode": "runtime_selector_portfolio_audit",
        "candidate_labels": ",".join(labels),
        "feature_count": len(features),
        "feature_names": ",".join(features),
        "audit_count": len(audit_rows),
        "pass_count": pass_count,
        "pass_all": bool(audit_rows) and pass_count == len(audit_rows),
        "selected_oracle_match_count": oracle_match_count,
        "selected_oracle_match_fraction": float(oracle_match_count / len(audit_rows)) if audit_rows else None,
        "validation_range_rmse_corrected_mean_m": float(np.mean(rmse_values)) if rmse_values else None,
        "validation_range_rmse_corrected_max_m": max(rmse_values, default=None),
        "oracle_validation_range_rmse_corrected_mean_m": float(np.mean(oracle_rmse_values)) if oracle_rmse_values else None,
        "rmse_regret_mean_m": float(np.mean(rmse_regret_values)) if rmse_regret_values else None,
        "rmse_regret_max_m": max(rmse_regret_values, default=None),
        "target_max_regret_max_m": max(target_regret_values, default=None),
        "target_max_abs_error_max_m": max(target_values, default=None),
        "target_error_limit_m": target_error_limit_m,
        "objective": objective,
        "selected_feature_names": ",".join(features),
        "include_target_run": include_target_run,
        "max_depth": max_depth,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Rule training uses offline replay metrics; runtime rule evaluation uses raw RDF/disparity features only."
        ),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    separability_rows = oracle_separability_rows(audit_rows, labels, features)
    best_separability = separability_rows[0] if separability_rows else {}
    summary.update(
        {
            "oracle_separability_best_label": best_separability.get("oracle_label", ""),
            "oracle_separability_best_feature": best_separability.get("feature_name", ""),
            "oracle_separability_best_accuracy": best_separability.get("accuracy", ""),
            "oracle_separability_best_effect_size": best_separability.get("standardized_effect_size", ""),
            "oracle_separability_contract": (
                "Diagnostics only: oracle labels use validation truth replay; runtime may use only the feature threshold."
            ),
        }
    )
    write_csv(output_dir / "runtime_selector_portfolio_audit.csv", audit_rows)
    write_csv(output_dir / "runtime_selector_portfolio_oracle_separability.csv", separability_rows)
    write_json(output_dir / "runtime_selector_portfolio_summary.json", summary)
    write_summary_txt(output_dir / "runtime_selector_portfolio_summary.txt", summary, audit_rows)
    write_oracle_separability_txt(
        output_dir / "runtime_selector_portfolio_oracle_separability.txt",
        summary,
        separability_rows,
    )
    if precision_override_label:
        baseline_label = precision_override_baseline_label or labels[0]
        override_summary, override_rows = audit_precision_override(
            audit_rows,
            separability_rows,
            labels=labels,
            override_label=precision_override_label,
            baseline_label=baseline_label,
            target_error_limit_m=target_error_limit_m,
            feature_name=precision_override_feature,
            direction=precision_override_direction,
            threshold=precision_override_threshold,
        )
        write_csv(output_dir / "runtime_selector_portfolio_precision_override_audit.csv", override_rows)
        write_json(output_dir / "runtime_selector_portfolio_precision_override_summary.json", override_summary)
        write_precision_override_txt(
            output_dir / "runtime_selector_portfolio_precision_override_summary.txt",
            override_summary,
            override_rows,
        )
        summary.update(
            {
                "precision_override_label": override_summary["override_label"],
                "precision_override_baseline_label": override_summary["baseline_label"],
                "precision_override_count": override_summary["override_count"],
                "precision_override_pass_count": override_summary["pass_count"],
                "precision_override_rmse_mean_m": override_summary["validation_range_rmse_corrected_mean_m"],
                "precision_override_target_max_abs_error_max_m": override_summary["target_max_abs_error_max_m"],
                "precision_override_rule_feature_name": override_summary["rule_feature_name"],
                "precision_override_rule_direction": override_summary["rule_direction"],
                "precision_override_rule_threshold": override_summary["rule_threshold"],
            }
        )
        write_json(output_dir / "runtime_selector_portfolio_summary.json", summary)
    return summary


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Runtime selector portfolio audit",
        "",
        f"candidate_labels: {summary.get('candidate_labels', '')}",
        f"objective: {summary.get('objective', '')}",
        f"max_depth: {summary.get('max_depth', '')}",
        f"audit_count: {summary.get('audit_count', '')}",
        f"pass_count: {summary.get('pass_count', '')}",
        f"pass_all: {summary.get('pass_all', '')}",
        f"selected_oracle_match_count: {summary.get('selected_oracle_match_count', '')}",
        f"selected_oracle_match_fraction: {summary.get('selected_oracle_match_fraction', '')}",
        f"validation_range_rmse_corrected_mean_m: {summary.get('validation_range_rmse_corrected_mean_m', '')}",
        f"validation_range_rmse_corrected_max_m: {summary.get('validation_range_rmse_corrected_max_m', '')}",
        f"oracle_validation_range_rmse_corrected_mean_m: {summary.get('oracle_validation_range_rmse_corrected_mean_m', '')}",
        f"rmse_regret_mean_m: {summary.get('rmse_regret_mean_m', '')}",
        f"rmse_regret_max_m: {summary.get('rmse_regret_max_m', '')}",
        f"target_max_regret_max_m: {summary.get('target_max_regret_max_m', '')}",
        f"target_max_abs_error_max_m: {summary.get('target_max_abs_error_max_m', '')}",
        f"oracle_separability_best_label: {summary.get('oracle_separability_best_label', '')}",
        f"oracle_separability_best_feature: {summary.get('oracle_separability_best_feature', '')}",
        f"oracle_separability_best_accuracy: {summary.get('oracle_separability_best_accuracy', '')}",
        f"oracle_separability_best_effect_size: {summary.get('oracle_separability_best_effect_size', '')}",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Rows:",
    ]
    for row in rows:
        lines.append(
            "- {target}/{seed}: selected={selected}, oracle={oracle}, pass={passed}, rmse={rmse}, "
            "target_max={target_max}, rmse_regret={regret}, rule={rule}".format(
                target=row.get("target_scenario", ""),
                seed=row.get("target_seed", ""),
                selected=row.get("selected_candidate", ""),
                oracle=row.get("oracle_candidate", ""),
                passed=row.get("pass_selector_portfolio_replay", ""),
                rmse=row.get("validation_range_rmse_corrected_m", ""),
                target_max=row.get("target_max_abs_error_corrected_m", ""),
                regret=row.get("rmse_regret_m", ""),
                rule=row.get("rule_feature_name", row.get("rule_constant_label", "")),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_oracle_separability_txt(
    path: Path,
    summary: Mapping[str, object],
    rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "Runtime selector portfolio oracle separability",
        "",
        "Purpose: diagnose whether offline oracle candidate labels can be approximated from runtime raw features.",
        "Runtime contract: feature thresholds may be used at runtime; oracle labels are calibration/replay truth only.",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        "",
        "Top threshold diagnostics:",
    ]
    for row in rows[:12]:
        lines.append(
            "- label={label} feature={feature} direction={direction} threshold={threshold} "
            "accuracy={accuracy} tp={tp} fp={fp} fn={fn} tn={tn} effect={effect}".format(
                label=row.get("oracle_label", ""),
                feature=row.get("feature_name", ""),
                direction=row.get("direction", ""),
                threshold=row.get("threshold", ""),
                accuracy=row.get("accuracy", ""),
                tp=row.get("true_positive_count", ""),
                fp=row.get("false_positive_count", ""),
                fn=row.get("false_negative_count", ""),
                tn=row.get("true_negative_count", ""),
                effect=row.get("standardized_effect_size", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_precision_override_txt(
    path: Path,
    summary: Mapping[str, object],
    rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "Runtime selector portfolio precision override",
        "",
        "Purpose: evaluate a conservative runtime-safe override that favors precision over recall.",
        f"override_label: {summary.get('override_label', '')}",
        f"baseline_label: {summary.get('baseline_label', '')}",
        f"rule: {summary.get('rule_feature_name', '')} {summary.get('rule_direction', '')} {summary.get('rule_threshold', '')}",
        f"rule_true_positive_count: {summary.get('rule_true_positive_count', '')}",
        f"rule_false_positive_count: {summary.get('rule_false_positive_count', '')}",
        f"audit_count: {summary.get('audit_count', '')}",
        f"override_count: {summary.get('override_count', '')}",
        f"pass_count: {summary.get('pass_count', '')}",
        f"pass_all: {summary.get('pass_all', '')}",
        f"selected_oracle_match_count: {summary.get('selected_oracle_match_count', '')}",
        f"selected_oracle_match_fraction: {summary.get('selected_oracle_match_fraction', '')}",
        f"validation_range_rmse_corrected_mean_m: {summary.get('validation_range_rmse_corrected_mean_m', '')}",
        f"validation_range_rmse_corrected_max_m: {summary.get('validation_range_rmse_corrected_max_m', '')}",
        f"target_max_abs_error_max_m: {summary.get('target_max_abs_error_max_m', '')}",
        f"rmse_regret_mean_m: {summary.get('rmse_regret_mean_m', '')}",
        f"rmse_regret_max_m: {summary.get('rmse_regret_max_m', '')}",
        f"target_max_regret_max_m: {summary.get('target_max_regret_max_m', '')}",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Rows:",
    ]
    for row in rows:
        lines.append(
            "- {target}/{seed}: selected={selected}, oracle={oracle}, override={override}, "
            "pass={passed}, rmse={rmse}, target_max={target_max}".format(
                target=row.get("target_scenario", ""),
                seed=row.get("target_seed", ""),
                selected=row.get("selected_candidate", ""),
                oracle=row.get("oracle_candidate", ""),
                override=row.get("override_applied", ""),
                passed=row.get("pass_precision_override_replay", ""),
                rmse=row.get("validation_range_rmse_corrected_m", ""),
                target_max=row.get("target_max_abs_error_corrected_m", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    selected_feature_names = [name.strip() for name in args.feature_names.split(",") if name.strip()]
    summary = audit_selector_portfolio(
        args.candidate,
        args.output_dir,
        target_error_limit_m=args.target_error_limit_m,
        objective=args.objective,
        selected_feature_names=selected_feature_names,
        include_target_run=args.include_target_run,
        max_depth=args.max_depth,
        precision_override_label=args.precision_override_label,
        precision_override_baseline_label=args.precision_override_baseline_label,
        precision_override_feature=args.precision_override_feature,
        precision_override_direction=args.precision_override_direction,
        precision_override_threshold=args.precision_override_threshold,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
