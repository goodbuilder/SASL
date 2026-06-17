#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_from_catalog import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    catalog_root,
    find_profile_row,
    load_catalog,
    load_minimal_set,
    method_contract_fields,
)
from stereodistance_profile_replay import load_profile, replay_rows  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Select a StereoDistance runtime profile by replaying validated catalog candidates "
            "on a calibration-flight CSV. The CSV truth is calibration-time evidence only; the "
            "selected runtime profile still consumes raw RDF/disparity and no truth."
        )
    )
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--split", default="validation")
    parser.add_argument(
        "--stability-splits",
        default="train,validation",
        help=(
            "Comma-separated split labels that must also replay cleanly when present in the CSV. "
            "Use an empty string to disable split-stability replay."
        ),
    )
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument(
        "--candidate-source-mode",
        choices=("minimal_set", "all_catalog"),
        default="minimal_set",
        help=(
            "minimal_set uses --minimal-set-json when provided; all_catalog replays every "
            "validated catalog profile as an offline calibration-time search."
        ),
    )
    parser.add_argument("--allow-unvalidated", action="store_true")
    parser.add_argument("--selection-summary-json", type=Path)
    return parser.parse_args()


def finite_float(value: object, default: float = math.nan) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def json_safe_float(value: object) -> float | None:
    parsed = finite_float(value, float("nan"))
    return parsed if math.isfinite(parsed) else None


def method_complexity_estimate(method: str) -> int:
    name = str(method or "")
    if not name or name == "raw":
        return 0
    if name == "global_rotation":
        return 3
    if name == "global_scale_rotation":
        return 4
    if name.startswith("global_disparity_linear"):
        return 2
    if name.startswith("global_disparity_quadratic"):
        return 3
    if name == "sim3":
        return 7
    if "quadratic" in name and "raw_forward" in name:
        return 15 if name.startswith("affine") else 11
    if name in {"trimmed_sim3", "huber_sim3", "tukey_sim3", "range_weighted_sim3"}:
        return 8
    if name in {"ransac_sim3", "huber_range_weighted_sim3", "tukey_range_weighted_sim3", "soft_l1_sim3", "cauchy_sim3"}:
        return 9
    if name.startswith("affine"):
        base = 12
    elif "sim3" in name:
        base = 8
    else:
        base = 6
    bin_bonus = 0
    if "bins11" in name:
        bin_bonus = 11
    elif "bins9" in name:
        bin_bonus = 9
    elif "bins5" in name:
        bin_bonus = 5
    elif "binned" in name:
        bin_bonus = 7
    if "rdf_residual" in name:
        bin_bonus *= 3
    if "blended" in name or "agreement_gated" in name:
        bin_bonus *= 2
    if "agreement_gated" in name:
        base += 2
    return int(base + bin_bonus)


def replay_selection_score(row: Mapping[str, object]) -> float:
    corrected_rmse = finite_float(row.get("validation_range_rmse_corrected_m"), float("inf"))
    target_abs = finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf"))
    target_max_abs = finite_float(row.get("target_max_abs_error_corrected_m"), target_abs)
    catalog_target_max_abs = finite_float(row.get("catalog_target_abs_mean_error_max_m"), target_max_abs)
    complexity = finite_float(row.get("model_complexity_estimate"), float("inf"))
    stability_penalty = finite_float(row.get("stability_penalty_m"), 0.0)
    worst_case_penalty = 0.10 * target_max_abs + 0.05 * catalog_target_max_abs
    return corrected_rmse + 0.30 * target_abs + worst_case_penalty + 0.08 * complexity + stability_penalty


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def parse_split_list(value: str | Sequence[str] | None) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        raw_items = value.split(",")
    else:
        raw_items = list(value)
    splits: list[str] = []
    for item in raw_items:
        split = str(item).strip()
        if split and split not in splits:
            splits.append(split)
    return splits


def available_named_splits(rows: Sequence[Mapping[str, str]]) -> set[str]:
    return {str(row.get("split", "")).strip() for row in rows if str(row.get("split", "")).strip()}


def candidate_source_scenarios(catalog: Mapping[str, object], minimal_set_json: Path | None) -> list[str]:
    if minimal_set_json is not None:
        minimal_set = load_minimal_set(minimal_set_json)
        rows = minimal_set.get("rows", [])
        sources: list[str] = []
        for row in rows if isinstance(rows, list) else []:
            if isinstance(row, Mapping):
                source = str(row.get("source_scenario", "")).strip()
                if source and source not in sources:
                    sources.append(source)
        if not sources:
            raise ValueError("minimal catalog profile set has no source_scenario entries")
        return sources
    profiles = catalog.get("profiles", [])
    return [
        str(profile.get("scenario", ""))
        for profile in profiles
        if isinstance(profile, Mapping) and str(profile.get("scenario", "")).strip()
    ]


def replay_candidate(
    *,
    catalog_json: Path,
    catalog: Mapping[str, object],
    scenario: str,
    rows: Sequence[Mapping[str, str]],
    split: str,
    target_range_m: float,
    target_window_m: float,
    target_error_limit_m: float,
    allow_unvalidated: bool,
    stability_splits: Sequence[str] = (),
) -> dict[str, object]:
    profile_row = find_profile_row(catalog, scenario)
    if not bool(profile_row.get("profile_exists")):
        raise ValueError(f"catalog profile for scenario {scenario!r} is marked missing")
    if not allow_unvalidated and not bool(profile_row.get("pass_cross_seed_replay")):
        raise ValueError(f"catalog profile for scenario {scenario!r} did not pass cross-seed replay")
    relative_profile = str(profile_row.get("catalog_profile_path", ""))
    if not relative_profile:
        raise ValueError(f"catalog profile for scenario {scenario!r} has no catalog_profile_path")
    profile_path = catalog_root(catalog_json) / relative_profile
    profile = load_profile(profile_path)
    _, summary = replay_rows(
        rows,
        profile,
        split_filter=split,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    raw_rmse = finite_float(summary.get("validation_range_rmse_raw_m"), float("inf"))
    corrected_rmse = finite_float(summary.get("validation_range_rmse_corrected_m"), float("inf"))
    target_abs = finite_float(summary.get("target_abs_mean_error_corrected_m"), float("inf"))
    target_max_abs = finite_float(summary.get("target_max_abs_error_corrected_m"), float("inf"))
    target_count = int(float(summary.get("target_window_count", 0) or 0))
    evaluated_count = int(float(summary.get("evaluated_row_count", 0) or 0))
    improvement = raw_rmse - corrected_rmse
    present_splits = available_named_splits(rows)
    stability_rows: list[dict[str, object]] = []
    stability_fail_reasons: list[str] = []
    stability_improvements: list[float] = []
    stability_target_errors: list[float] = []
    stability_evaluated_counts: list[int] = []
    for stability_split in stability_splits:
        if present_splits and stability_split not in present_splits:
            continue
        _, split_summary = replay_rows(
            rows,
            profile,
            split_filter=stability_split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
        )
        split_raw_rmse = finite_float(split_summary.get("validation_range_rmse_raw_m"), float("inf"))
        split_corrected_rmse = finite_float(split_summary.get("validation_range_rmse_corrected_m"), float("inf"))
        split_improvement = split_raw_rmse - split_corrected_rmse
        split_target_abs = finite_float(split_summary.get("target_abs_mean_error_corrected_m"), float("nan"))
        split_target_max_abs = finite_float(split_summary.get("target_max_abs_error_corrected_m"), split_target_abs)
        split_target_count = int(float(split_summary.get("target_window_count", 0) or 0))
        split_evaluated_count = int(float(split_summary.get("evaluated_row_count", 0) or 0))
        stability_evaluated_counts.append(split_evaluated_count)
        if math.isfinite(split_improvement):
            stability_improvements.append(split_improvement)
        if math.isfinite(split_target_abs):
            stability_target_errors.append(split_target_abs)
        if split_evaluated_count <= 0:
            stability_fail_reasons.append(f"{stability_split}:no_evaluated_rows")
        if not math.isfinite(split_raw_rmse) or not math.isfinite(split_corrected_rmse):
            stability_fail_reasons.append(f"{stability_split}:rmse_not_finite")
        if not math.isfinite(split_improvement) or split_improvement <= 0.0:
            stability_fail_reasons.append(f"{stability_split}:no_positive_range_rmse_improvement")
        if split_target_count > 0 and (not math.isfinite(split_target_abs) or split_target_abs > target_error_limit_m):
            stability_fail_reasons.append(f"{stability_split}:target_error_over_limit")
        if split_target_count > 0 and (
            not math.isfinite(split_target_max_abs) or split_target_max_abs > target_error_limit_m
        ):
            stability_fail_reasons.append(f"{stability_split}:target_max_error_over_limit")
        stability_rows.append(
            {
                "split": stability_split,
                "evaluated_row_count": split_evaluated_count,
                "validation_range_rmse_raw_m": json_safe_float(split_raw_rmse),
                "validation_range_rmse_corrected_m": json_safe_float(split_corrected_rmse),
                "validation_range_rmse_improvement_m": json_safe_float(split_improvement),
                "target_window_count": split_target_count,
                "target_abs_mean_error_corrected_m": json_safe_float(split_target_abs),
                "target_max_abs_error_corrected_m": json_safe_float(split_target_max_abs),
            }
        )
    stability_pass = not stability_fail_reasons
    stability_min_improvement = min(stability_improvements) if stability_improvements else float("nan")
    stability_max_target_abs = max(stability_target_errors) if stability_target_errors else float("nan")
    stability_min_evaluated = min(stability_evaluated_counts) if stability_evaluated_counts else evaluated_count
    stability_penalty = 0.0
    if math.isfinite(stability_min_improvement) and math.isfinite(improvement):
        stability_penalty += max(0.0, improvement - stability_min_improvement) * 0.15
    if math.isfinite(stability_max_target_abs) and math.isfinite(target_abs):
        stability_penalty += max(0.0, stability_max_target_abs - target_abs) * 0.10
    method = str(profile_row.get("method", ""))
    complexity = method_complexity_estimate(method)
    fail_reasons: list[str] = []
    if evaluated_count <= 0:
        fail_reasons.append("no_evaluated_rows")
    if not math.isfinite(raw_rmse):
        fail_reasons.append("raw_rmse_not_finite")
    if not math.isfinite(corrected_rmse):
        fail_reasons.append("corrected_rmse_not_finite")
    if not math.isfinite(improvement) or improvement <= 0.0:
        fail_reasons.append("no_positive_range_rmse_improvement")
    if not stability_pass:
        fail_reasons.extend(stability_fail_reasons)
    if target_count <= 0:
        fail_reasons.append("target_window_missing")
    if not math.isfinite(target_abs):
        fail_reasons.append("target_error_not_finite")
    elif target_abs > target_error_limit_m:
        fail_reasons.append("target_error_over_limit")
    if target_count > 0 and (not math.isfinite(target_max_abs) or target_max_abs > target_error_limit_m):
        fail_reasons.append("target_max_error_over_limit")
    pass_replay = (
        evaluated_count > 0
        and math.isfinite(raw_rmse)
        and math.isfinite(corrected_rmse)
        and improvement > 0.0
        and stability_pass
        and target_count > 0
        and math.isfinite(target_abs)
        and target_abs <= target_error_limit_m
        and math.isfinite(target_max_abs)
        and target_max_abs <= target_error_limit_m
    )
    result = {
        "scenario": scenario,
        "method": method,
        **method_contract_fields(method),
        "profile_path": str(profile_path),
        "pass_replay": pass_replay,
        "pass_reason": "pass" if pass_replay else ";".join(fail_reasons),
        "evaluated_row_count": evaluated_count,
        "validation_range_rmse_raw_m": raw_rmse,
        "validation_range_rmse_corrected_m": corrected_rmse,
        "validation_range_rmse_improvement_m": improvement,
        "range_improvement_margin_m": improvement,
        "target_window_count": target_count,
        "target_abs_mean_error_corrected_m": target_abs,
        "target_max_abs_error_corrected_m": target_max_abs,
        "target_error_limit_m": target_error_limit_m,
        "target_error_margin_m": target_error_limit_m - target_abs if math.isfinite(target_abs) else float("-inf"),
        "target_max_error_margin_m": (
            target_error_limit_m - target_max_abs if math.isfinite(target_max_abs) else float("-inf")
        ),
        "stability_splits_evaluated": ",".join(str(row["split"]) for row in stability_rows),
        "stability_pass": stability_pass,
        "stability_fail_reason": "pass" if stability_pass else ";".join(stability_fail_reasons),
        "stability_min_range_improvement_m": json_safe_float(stability_min_improvement),
        "stability_max_target_abs_mean_error_m": json_safe_float(stability_max_target_abs),
        "stability_min_evaluated_row_count": stability_min_evaluated,
        "stability_penalty_m": json_safe_float(stability_penalty),
        "stability_split_summaries_json": json.dumps(stability_rows, ensure_ascii=False, allow_nan=False),
        "model_complexity_estimate": complexity,
        "catalog_validation_range_rmse_mean_m": finite_float(profile_row.get("validation_range_rmse_mean_m"), float("nan")),
        "catalog_target_abs_mean_error_max_m": finite_float(profile_row.get("target_abs_mean_error_max_m"), float("nan")),
        "axis_contract": RDF_AXIS_CONTRACT,
        "runtime_input_contract": "candidate replay uses calibration-time truth for selection; selected runtime profile uses raw RDF/disparity only",
    }
    result["replay_selection_score"] = replay_selection_score(result)
    return result


def select_by_calibration_replay(
    *,
    catalog_json: Path,
    input_csv: Path,
    output_profile: Path,
    output_dir: Path,
    minimal_set_json: Path | None = None,
    split: str = "validation",
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
    allow_unvalidated: bool = False,
    stability_splits: Sequence[str] = ("train", "validation"),
    candidate_source_mode: str = "minimal_set",
) -> dict[str, object]:
    catalog = load_catalog(catalog_json)
    rows = read_csv(input_csv)
    output_dir.mkdir(parents=True, exist_ok=True)
    candidate_rows: list[dict[str, object]] = []
    if candidate_source_mode == "all_catalog":
        source_scenarios = candidate_source_scenarios(catalog, None)
    elif candidate_source_mode == "minimal_set":
        source_scenarios = candidate_source_scenarios(catalog, minimal_set_json)
    else:
        raise ValueError("candidate_source_mode must be 'minimal_set' or 'all_catalog'")
    for scenario in source_scenarios:
        candidate_rows.append(
            replay_candidate(
                catalog_json=catalog_json,
                catalog=catalog,
                scenario=scenario,
                rows=rows,
                split=split,
                target_range_m=target_range_m,
                target_window_m=target_window_m,
                target_error_limit_m=target_error_limit_m,
                allow_unvalidated=allow_unvalidated,
                stability_splits=stability_splits,
            )
        )
    candidate_rows = sorted(
        candidate_rows,
        key=lambda row: (
            not bool(row.get("pass_replay")),
            finite_float(row.get("replay_selection_score"), float("inf")),
            finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")),
            finite_float(row.get("model_complexity_estimate"), float("inf")),
            str(row.get("scenario", "")),
        ),
    )
    for index, row in enumerate(candidate_rows, start=1):
        row["replay_selection_rank"] = index
    write_csv(output_dir / "calibration_replay_profile_candidates.csv", candidate_rows)
    passing = [row for row in candidate_rows if bool(row.get("pass_replay"))]
    if not passing:
        summary = {
            "selected": False,
            "reason": "no catalog candidate passed calibration replay gates",
            "candidate_count": len(candidate_rows),
            "pass_count": 0,
            "catalog_json": str(catalog_json),
            "minimal_set_json": str(minimal_set_json) if minimal_set_json else "",
            "candidate_source_mode": candidate_source_mode,
            "candidate_source_scenarios": ",".join(source_scenarios),
            "input_csv": str(input_csv),
            "split": split,
            "stability_splits_requested": ",".join(stability_splits),
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "selection_contract": "calibration-time truth may select a profile; runtime must not use truth",
            "runtime_input_contract": "no runtime profile promoted",
        }
        write_json(output_dir / "calibration_replay_profile_selection.json", summary)
        raise ValueError(summary["reason"])
    best = min(
        passing,
        key=lambda row: (
            finite_float(row.get("replay_selection_score"), float("inf")),
            finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")),
            finite_float(row.get("model_complexity_estimate"), float("inf")),
            str(row.get("scenario", "")),
        ),
    )
    source_profile = Path(str(best["profile_path"]))
    output_profile.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source_profile, output_profile)
    summary = {
        "selected": True,
        "selection_mode": "calibration_replay_catalog_candidate",
        "scenario": best.get("scenario", ""),
        "method": best.get("method", ""),
        **method_contract_fields(str(best.get("method", ""))),
        "source_profile": str(source_profile),
        "output_profile": str(output_profile),
        "candidate_count": len(candidate_rows),
        "pass_count": len(passing),
        "catalog_json": str(catalog_json),
        "minimal_set_json": str(minimal_set_json) if minimal_set_json else "",
        "candidate_source_mode": candidate_source_mode,
        "candidate_source_scenarios": ",".join(source_scenarios),
        "input_csv": str(input_csv),
        "split": split,
        "stability_splits_requested": ",".join(stability_splits),
        "stability_splits_evaluated": best.get("stability_splits_evaluated", ""),
        "stability_min_range_improvement_m": best.get("stability_min_range_improvement_m"),
        "stability_max_target_abs_mean_error_m": best.get("stability_max_target_abs_mean_error_m"),
        "stability_penalty_m": best.get("stability_penalty_m"),
        "validation_range_rmse_raw_m": best.get("validation_range_rmse_raw_m"),
        "validation_range_rmse_corrected_m": best.get("validation_range_rmse_corrected_m"),
        "target_abs_mean_error_corrected_m": best.get("target_abs_mean_error_corrected_m"),
        "target_max_abs_error_corrected_m": best.get("target_max_abs_error_corrected_m"),
        "replay_selection_score": best.get("replay_selection_score"),
        "model_complexity_estimate": best.get("model_complexity_estimate"),
        "replay_selection_rank": best.get("replay_selection_rank"),
        "target_window_count": best.get("target_window_count"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Calibration-time truth is used only to choose among prevalidated catalog profiles. "
            "The selected runtime profile must not use truth."
        ),
        "runtime_input_contract": "selected profile is consumed as a normal single StereoDistance runtime profile; no runtime truth",
    }
    write_json(output_dir / "calibration_replay_profile_selection.json", summary)
    return summary


def main() -> int:
    args = parse_args()
    summary = select_by_calibration_replay(
        catalog_json=args.catalog_json,
        input_csv=args.input_csv,
        output_profile=args.output_profile,
        output_dir=args.output_dir,
        minimal_set_json=args.minimal_set_json,
        split=args.split,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
        allow_unvalidated=args.allow_unvalidated,
        stability_splits=parse_split_list(args.stability_splits),
        candidate_source_mode=args.candidate_source_mode,
    )
    summary_path = args.selection_summary_json or (args.output_profile.with_suffix(args.output_profile.suffix + ".selection.json"))
    write_json(summary_path, summary)
    print(f"selected profile -> {args.output_profile}")
    print(
        "selected {scenario}/{method}: range RMSE raw -> corrected {raw:.3f} -> {corrected:.3f} m, "
        "target_abs={target:.3f} m, target_max_abs={target_max:.3f} m".format(
            scenario=summary["scenario"],
            method=summary["method"],
            raw=float(summary["validation_range_rmse_raw_m"]),
            corrected=float(summary["validation_range_rmse_corrected_m"]),
            target=float(summary["target_abs_mean_error_corrected_m"]),
            target_max=float(summary["target_max_abs_error_corrected_m"]),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
