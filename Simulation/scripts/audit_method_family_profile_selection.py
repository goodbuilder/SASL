#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit and optionally export the best runtime profile inside one method family for each "
            "scenario/seed run in a sweep. This is an offline calibration-time selector over "
            "method_comparison.csv; exported profiles still consume raw RDF/disparity only."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--family-name", default="affine_runtime")
    parser.add_argument("--method-substring", default="affine")
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--export-selected-profiles", action="store_true")
    return parser.parse_args()


def finite_float(value: object, default: float = float("nan")) -> float:
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


def scenario_seed_from_path(path: Path, sweep_dir: Path) -> tuple[str, str]:
    relative = path.relative_to(sweep_dir)
    parts = relative.parts
    if len(parts) >= 3:
        return parts[0], parts[1]
    return "", ""


def profile_path_for_method(run_dir: Path, method: str) -> Path:
    return run_dir / "runtime_profiles" / f"{method}.json"


def selection_score(row: Mapping[str, object]) -> float:
    target_max = finite_float(row.get("validation_target_max_abs_error_m"), float("inf"))
    validation_rmse = finite_float(row.get("validation_range_rmse_m"), float("inf"))
    target_abs = finite_float(row.get("validation_target_abs_mean_error_m"), target_max)
    complexity = finite_float(row.get("model_complexity"), float("inf"))
    return target_max + 0.25 * validation_rmse + 0.10 * target_abs + 0.02 * complexity


def candidate_row(
    row: Mapping[str, str],
    *,
    raw_row: Mapping[str, str],
    profile_path: Path,
    family_name: str,
    method_substring: str,
    target_error_limit_m: float,
) -> dict[str, object]:
    validation_rmse = finite_float(row.get("validation_range_rmse_m"))
    raw_rmse = finite_float(raw_row.get("validation_range_rmse_m"))
    improvement = raw_rmse - validation_rmse if math.isfinite(raw_rmse) and math.isfinite(validation_rmse) else float("nan")
    target_count = int(finite_float(row.get("validation_target_window_count"), 0.0))
    target_max = finite_float(row.get("validation_target_max_abs_error_m"))
    profile_exists = profile_path.is_file()
    pass_reasons: list[str] = []
    if not profile_exists:
        pass_reasons.append("missing_runtime_profile")
    if not math.isfinite(improvement) or improvement <= 0.0:
        pass_reasons.append("no_positive_range_rmse_improvement")
    if target_count <= 0:
        pass_reasons.append("no_target_window_samples")
    if not math.isfinite(target_max) or target_max > target_error_limit_m:
        pass_reasons.append("target_max_error_over_limit")
    runtime_family = str(row.get("runtime_profile_family", ""))
    if runtime_family in {"", "none_raw", "offline_or_nonpreferred_runtime_candidate"}:
        pass_reasons.append("not_runtime_preferred")
    passed = not pass_reasons
    result = {
        "family_name": family_name,
        "method_substring": method_substring,
        "method": row.get("method", ""),
        "runtime_profile_family": runtime_family,
        "registration_model": row.get("registration_model", ""),
        "residual_model": row.get("residual_model", ""),
        "regularization_model": row.get("regularization_model", ""),
        "model_complexity": row.get("model_complexity", ""),
        "profile_path": str(profile_path),
        "profile_exists": profile_exists,
        "validation_range_rmse_raw_m": raw_row.get("validation_range_rmse_m", ""),
        "validation_range_rmse_m": row.get("validation_range_rmse_m", ""),
        "validation_range_rmse_improvement_vs_raw_m": improvement,
        "validation_target_window_count": target_count,
        "validation_target_abs_mean_error_m": row.get("validation_target_abs_mean_error_m", ""),
        "validation_target_max_abs_error_m": row.get("validation_target_max_abs_error_m", ""),
        "pass_family_selection_gate": passed,
        "pass_reason": "pass" if passed else ",".join(pass_reasons),
        "selection_score": selection_score(row),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "offline calibration-time method family selection uses validation metrics; exported runtime profile "
            "must consume raw RDF/disparity and fixed calibration parameters only"
        ),
    }
    return result


def select_run_family_profile(
    comparison_csv: Path,
    *,
    sweep_dir: Path,
    family_name: str,
    method_substring: str,
    target_error_limit_m: float,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    rows = read_csv(comparison_csv)
    scenario, seed = scenario_seed_from_path(comparison_csv, sweep_dir)
    run_dir = comparison_csv.parent
    raw_row = next((row for row in rows if row.get("method") == "raw"), {})
    candidates = [
        candidate_row(
            row,
            raw_row=raw_row,
            profile_path=profile_path_for_method(run_dir, str(row.get("method", ""))),
            family_name=family_name,
            method_substring=method_substring,
            target_error_limit_m=target_error_limit_m,
        )
        for row in rows
        if method_substring in str(row.get("method", "")) and row.get("method") != "raw"
    ]
    candidates.sort(
        key=lambda row: (
            not bool(row.get("pass_family_selection_gate")),
            finite_float(row.get("selection_score"), float("inf")),
            finite_float(row.get("validation_target_max_abs_error_m"), float("inf")),
            finite_float(row.get("validation_range_rmse_m"), float("inf")),
            finite_float(row.get("model_complexity"), float("inf")),
            str(row.get("method", "")),
        )
    )
    for index, row in enumerate(candidates, start=1):
        row["family_selection_rank"] = index
        row["scenario"] = scenario
        row["seed"] = seed
    passing = [row for row in candidates if bool(row.get("pass_family_selection_gate"))]
    best = passing[0] if passing else (candidates[0] if candidates else {})
    selected = bool(best) and bool(best.get("pass_family_selection_gate"))
    summary = {
        "scenario": scenario,
        "seed": seed,
        "family_name": family_name,
        "method_substring": method_substring,
        "selected": selected,
        "candidate_count": len(candidates),
        "pass_count": len(passing),
        "selected_method": best.get("method", ""),
        "selected_profile_path": best.get("profile_path", ""),
        "selected_runtime_profile_family": best.get("runtime_profile_family", ""),
        "selected_validation_range_rmse_m": best.get("validation_range_rmse_m", ""),
        "selected_validation_range_rmse_improvement_vs_raw_m": best.get(
            "validation_range_rmse_improvement_vs_raw_m", ""
        ),
        "selected_validation_target_max_abs_error_m": best.get("validation_target_max_abs_error_m", ""),
        "selected_selection_score": best.get("selection_score", ""),
        "selected_pass_reason": best.get("pass_reason", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "offline family selector may use calibration-time validation metrics; selected profile runtime "
            "application must not use truth"
        ),
        "runtime_input_contract": (
            "selected runtime profile consumes raw stereo-center RDF(right,down,forward), measured disparity "
            "when the profile requires it, and fixed calibration parameters"
        ),
    }
    return candidates, summary


def audit_method_family_profile_selection(
    sweep_dir: Path,
    output_dir: Path,
    *,
    family_name: str,
    method_substring: str,
    target_error_limit_m: float,
    export_selected_profiles: bool = False,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    all_candidate_rows: list[dict[str, object]] = []
    selected_rows: list[dict[str, object]] = []
    for comparison_csv in sorted(sweep_dir.glob("*/*/method_comparison.csv")):
        candidates, selected = select_run_family_profile(
            comparison_csv,
            sweep_dir=sweep_dir,
            family_name=family_name,
            method_substring=method_substring,
            target_error_limit_m=target_error_limit_m,
        )
        all_candidate_rows.extend(candidates)
        if selected.get("scenario") and selected.get("seed"):
            if export_selected_profiles and selected.get("selected"):
                source_profile = Path(str(selected.get("selected_profile_path", "")))
                export_path = output_dir / "selected_profiles" / str(selected["scenario"]) / str(selected["seed"]) / source_profile.name
                export_path.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source_profile, export_path)
                selected["exported_profile_path"] = str(export_path)
            selected_rows.append(selected)
    pass_count = sum(1 for row in selected_rows if bool(row.get("selected")))
    target_values = [
        finite_float(row.get("selected_validation_target_max_abs_error_m"))
        for row in selected_rows
        if bool(row.get("selected")) and math.isfinite(finite_float(row.get("selected_validation_target_max_abs_error_m")))
    ]
    rmse_values = [
        finite_float(row.get("selected_validation_range_rmse_m"))
        for row in selected_rows
        if bool(row.get("selected")) and math.isfinite(finite_float(row.get("selected_validation_range_rmse_m")))
    ]
    summary = {
        "mode": "method_family_profile_selection_audit",
        "sweep_dir": str(sweep_dir),
        "family_name": family_name,
        "method_substring": method_substring,
        "run_count": len(selected_rows),
        "selected_pass_count": pass_count,
        "selected_pass_all": bool(selected_rows) and pass_count == len(selected_rows),
        "selected_validation_target_max_abs_error_max_m": max(target_values, default=None),
        "selected_validation_range_rmse_mean_m": float(sum(rmse_values) / len(rmse_values)) if rmse_values else None,
        "selected_validation_range_rmse_max_m": max(rmse_values, default=None),
        "target_error_limit_m": target_error_limit_m,
        "export_selected_profiles": export_selected_profiles,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "This audit uses calibration-time validation metrics to choose among already exported runtime profiles "
            "inside a method family. Runtime profile application remains truth-free."
        ),
    }
    write_csv(output_dir / "method_family_profile_candidates.csv", all_candidate_rows)
    write_csv(output_dir / "method_family_profile_selected.csv", selected_rows)
    write_json(output_dir / "method_family_profile_selection_summary.json", summary)
    write_summary_txt(output_dir / "method_family_profile_selection_summary.txt", summary, selected_rows)
    return summary


def write_summary_txt(path: Path, summary: Mapping[str, object], selected_rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Method family profile selection audit",
        "",
        f"family_name: {summary.get('family_name', '')}",
        f"method_substring: {summary.get('method_substring', '')}",
        f"run_count: {summary.get('run_count', '')}",
        f"selected_pass_count: {summary.get('selected_pass_count', '')}",
        f"selected_pass_all: {summary.get('selected_pass_all', '')}",
        f"selected_validation_target_max_abs_error_max_m: {summary.get('selected_validation_target_max_abs_error_max_m', '')}",
        f"selected_validation_range_rmse_mean_m: {summary.get('selected_validation_range_rmse_mean_m', '')}",
        f"selected_validation_range_rmse_max_m: {summary.get('selected_validation_range_rmse_max_m', '')}",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        "",
        "Selected rows:",
    ]
    for row in selected_rows:
        lines.append(
            "- {scenario}/{seed}: selected={selected}, method={method}, rmse={rmse}, target_max={target_max}, reason={reason}".format(
                scenario=row.get("scenario", ""),
                seed=row.get("seed", ""),
                selected=row.get("selected", ""),
                method=row.get("selected_method", ""),
                rmse=row.get("selected_validation_range_rmse_m", ""),
                target_max=row.get("selected_validation_target_max_abs_error_m", ""),
                reason=row.get("selected_pass_reason", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    summary = audit_method_family_profile_selection(
        args.sweep_dir,
        args.output_dir,
        family_name=args.family_name,
        method_substring=args.method_substring,
        target_error_limit_m=args.target_error_limit_m,
        export_selected_profiles=args.export_selected_profiles,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
