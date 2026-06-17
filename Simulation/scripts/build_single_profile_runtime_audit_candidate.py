#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from stereodistance_profile_replay import load_profile, replay_rows  # noqa: E402


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
SELECTION_CONTRACT = (
    "single-profile candidate uses copied truth-free runtime features for selection; "
    "calibration-time truth is used only to audit replay metrics"
)
DEFAULT_SELECTION_INPUT_FIELDS = (
    "raw_right_m,raw_down_m,raw_forward_m,raw_range_m,measured_disparity_px,left_u_px,left_v_px"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build a runtime-selector-compatible audit CSV for one fixed StereoDistance profile. "
            "The output keeps raw feature columns from an existing runtime guard audit and replays "
            "the fixed profile offline against each input CSV for validation metrics."
        )
    )
    parser.add_argument("--guard-audit-csv", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--label", default="single_profile")
    parser.add_argument("--split", default="validation")
    parser.add_argument("--stability-splits", default="train,validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args()


def finite_float(value: object, default: float = float("nan")) -> float:
    if value in (None, ""):
        return default
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def json_safe_float(value: object) -> float | None:
    parsed = finite_float(value)
    return parsed if math.isfinite(parsed) else None


def parse_split_list(value: str | Sequence[str] | None) -> list[str]:
    if value is None:
        return []
    raw_items = value.split(",") if isinstance(value, str) else list(value)
    splits: list[str] = []
    for item in raw_items:
        split = str(item).strip()
        if split and split not in splits:
            splits.append(split)
    return splits


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


def available_named_splits(rows: Sequence[Mapping[str, str]]) -> set[str]:
    return {str(row.get("split", "")).strip() for row in rows if str(row.get("split", "")).strip()}


def replay_profile_on_rows(
    *,
    rows: Sequence[Mapping[str, str]],
    profile: object,
    split: str,
    stability_splits: Sequence[str],
    target_range_m: float,
    target_window_m: float,
    target_error_limit_m: float,
) -> dict[str, object]:
    _, summary = replay_rows(
        rows,
        profile,  # type: ignore[arg-type]
        split_filter=split,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    raw_rmse = finite_float(summary.get("validation_range_rmse_raw_m"), float("inf"))
    corrected_rmse = finite_float(summary.get("validation_range_rmse_corrected_m"), float("inf"))
    improvement = raw_rmse - corrected_rmse
    target_count = int(float(summary.get("target_window_count", 0) or 0))
    target_abs = finite_float(summary.get("target_abs_mean_error_corrected_m"), float("inf"))
    target_max_abs = finite_float(summary.get("target_max_abs_error_corrected_m"), float("inf"))
    evaluated_count = int(float(summary.get("evaluated_row_count", 0) or 0))

    present_splits = available_named_splits(rows)
    stability_rows: list[dict[str, object]] = []
    fail_reasons: list[str] = []
    for stability_split in stability_splits:
        if present_splits and stability_split not in present_splits:
            continue
        _, split_summary = replay_rows(
            rows,
            profile,  # type: ignore[arg-type]
            split_filter=stability_split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
        )
        split_raw_rmse = finite_float(split_summary.get("validation_range_rmse_raw_m"), float("inf"))
        split_corrected_rmse = finite_float(split_summary.get("validation_range_rmse_corrected_m"), float("inf"))
        split_improvement = split_raw_rmse - split_corrected_rmse
        split_target_count = int(float(split_summary.get("target_window_count", 0) or 0))
        split_target_max_abs = finite_float(split_summary.get("target_max_abs_error_corrected_m"), float("inf"))
        split_evaluated_count = int(float(split_summary.get("evaluated_row_count", 0) or 0))
        if split_evaluated_count <= 0:
            fail_reasons.append(f"{stability_split}:no_evaluated_rows")
        if not math.isfinite(split_improvement) or split_improvement <= 0.0:
            fail_reasons.append(f"{stability_split}:no_positive_range_rmse_improvement")
        if split_target_count > 0 and (
            not math.isfinite(split_target_max_abs) or split_target_max_abs > target_error_limit_m
        ):
            fail_reasons.append(f"{stability_split}:target_max_error_over_limit")
        stability_rows.append(
            {
                "split": stability_split,
                "evaluated_row_count": split_evaluated_count,
                "validation_range_rmse_raw_m": json_safe_float(split_raw_rmse),
                "validation_range_rmse_corrected_m": json_safe_float(split_corrected_rmse),
                "validation_range_rmse_improvement_m": json_safe_float(split_improvement),
                "target_window_count": split_target_count,
                "target_max_abs_error_corrected_m": json_safe_float(split_target_max_abs),
            }
        )

    if evaluated_count <= 0:
        fail_reasons.append("no_evaluated_rows")
    if not math.isfinite(improvement) or improvement <= 0.0:
        fail_reasons.append("no_positive_range_rmse_improvement")
    if target_count <= 0:
        fail_reasons.append("target_window_missing")
    if not math.isfinite(target_abs) or target_abs > target_error_limit_m:
        fail_reasons.append("target_error_over_limit")
    if not math.isfinite(target_max_abs) or target_max_abs > target_error_limit_m:
        fail_reasons.append("target_max_error_over_limit")

    return {
        "pass_replay": not fail_reasons,
        "pass_reason": "pass" if not fail_reasons else ";".join(fail_reasons),
        "evaluated_row_count": evaluated_count,
        "validation_range_rmse_raw_m": raw_rmse,
        "validation_range_rmse_corrected_m": corrected_rmse,
        "validation_range_rmse_improvement_m": improvement,
        "target_window_count": target_count,
        "target_abs_mean_error_corrected_m": target_abs,
        "target_max_abs_error_corrected_m": target_max_abs,
        "stability_split_summaries_json": json.dumps(stability_rows, ensure_ascii=False, allow_nan=False),
    }


def build_single_profile_candidate(
    *,
    guard_audit_csv: Path,
    profile_json: Path,
    output_csv: Path,
    label: str,
    split: str = "validation",
    stability_splits: Sequence[str] = ("train", "validation"),
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
) -> list[dict[str, object]]:
    guard_rows = read_csv(guard_audit_csv)
    profile = load_profile(profile_json)
    output_rows: list[dict[str, object]] = []
    for guard_row in guard_rows:
        input_csv = Path(str(guard_row.get("input_csv", "")))
        if not input_csv.is_file():
            raise FileNotFoundError(f"input_csv from guard audit is missing: {input_csv}")
        calibration_rows = read_csv(input_csv)
        replay = replay_profile_on_rows(
            rows=calibration_rows,
            profile=profile,
            split=split,
            stability_splits=stability_splits,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
            target_error_limit_m=target_error_limit_m,
        )
        row: dict[str, object] = {
            "target_scenario": guard_row.get("target_scenario", ""),
            "target_seed": guard_row.get("target_seed", ""),
            "input_csv": str(input_csv),
            "selected_source_scenario": label,
            "selected_source_seed": "",
            "selection_policy": "fixed_single_profile",
            "fallback_reason": "fixed_profile_candidate",
            "pass_runtime_measurement_guard_replay": bool(replay["pass_replay"]),
            "pass_reason": replay["pass_reason"],
            "validation_only_pass_runtime_measurement_guard_replay": bool(replay["pass_replay"]),
            "validation_only_pass_reason": replay["pass_reason"],
            "selected_method": getattr(profile, "profile_id", profile_json.stem),
            "profile_json": str(profile_json),
            "validation_range_rmse_raw_m": replay["validation_range_rmse_raw_m"],
            "validation_range_rmse_corrected_m": replay["validation_range_rmse_corrected_m"],
            "validation_range_rmse_improvement_m": replay["validation_range_rmse_improvement_m"],
            "target_window_count": replay["target_window_count"],
            "target_abs_mean_error_corrected_m": replay["target_abs_mean_error_corrected_m"],
            "target_max_abs_error_corrected_m": replay["target_max_abs_error_corrected_m"],
            "stability_split_summaries_json": replay["stability_split_summaries_json"],
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "selection_input_fields": guard_row.get("selection_input_fields", DEFAULT_SELECTION_INPUT_FIELDS),
            "forbidden_selection_fields": guard_row.get("forbidden_selection_fields", ""),
            "selection_contract": SELECTION_CONTRACT,
        }
        for key, value in guard_row.items():
            if key.startswith("feature_"):
                row[key] = value
        output_rows.append(row)
    write_csv(output_csv, output_rows)
    return output_rows


def main() -> int:
    args = parse_args()
    rows = build_single_profile_candidate(
        guard_audit_csv=args.guard_audit_csv,
        profile_json=args.profile_json,
        output_csv=args.output_csv,
        label=args.label,
        split=args.split,
        stability_splits=parse_split_list(args.stability_splits),
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
    )
    pass_count = sum(1 for row in rows if bool(row.get("pass_runtime_measurement_guard_replay")))
    target_values = [
        finite_float(row.get("target_max_abs_error_corrected_m"))
        for row in rows
        if math.isfinite(finite_float(row.get("target_max_abs_error_corrected_m")))
    ]
    print(f"wrote {len(rows)} rows to {args.output_csv}")
    print(f"pass_count={pass_count}/{len(rows)} target_max_abs_error_max_m={max(target_values) if target_values else float('nan')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
