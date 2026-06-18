#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from export_runtime_guard_input_from_calibration_csv import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    export_runtime_guard_input,
)
from export_stereodistance_profile_by_operating_mode import export_by_operating_mode  # noqa: E402
from select_stereodistance_profile_by_runtime_measurement_guard import sweep_calibration_csvs  # noqa: E402
from stereodistance_profile_promotion_check import run_check as run_promotion_check  # noqa: E402


def parse_csv_list(value: str) -> list[str]:
    return [item.strip() for item in str(value).split(",") if item.strip()]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Batch-smoke the truth-free runtime_guard operating-mode path over every scenario/seed "
            "calibration CSV in a sweep. Each row is converted with the CSV parser, exported to an "
            "active profile, and checked through the StereoDistance promotion gate."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--scenarios", default="", help="Optional comma-separated scenario filter.")
    parser.add_argument("--seeds", default="", help="Optional comma-separated seed directory filter.")
    parser.add_argument("--split", default="validation")
    parser.add_argument("--max-samples", type=int, default=50)
    parser.add_argument("--runtime-candidate-source-mode", choices=("minimal_set", "all_catalog"), default="all_catalog")
    parser.add_argument(
        "--runtime-selection-policy",
        choices=(
            "nearest_centroid",
            "robust_family_fallback",
            "transfer_risk_fallback",
            "intrinsic_scale_gap_transfer_risk",
            "feature_vote_transfer_risk",
            "knn_transfer_risk",
            "centroid_knn_union_transfer_risk",
            "centroid_knn_veto_transfer_risk",
        ),
        default="transfer_risk_fallback",
    )
    parser.add_argument("--transfer-matrix-csv", type=Path)
    parser.add_argument("--transfer-low-risk-rmse-limit-m", type=float, default=1.3)
    parser.add_argument("--transfer-low-risk-target-limit-m", type=float, default=2.5)
    parser.add_argument("--transfer-risk-target-weight", type=float, default=0.10)
    parser.add_argument("--knn-neighbor-count", type=int, default=5)
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


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Runtime guard operating-mode batch smoke",
        f"status: {summary.get('status', '')}",
        f"sweep_dir: {summary.get('sweep_dir', '')}",
        f"run_count: {summary.get('run_count', 0)}",
        f"pass_count: {summary.get('pass_count', 0)}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', False)}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        f"failure_count: {summary.get('failure_count', 0)}",
        "",
        "Rows:",
    ]
    for row in rows:
        lines.append(
            "- {scenario}/{seed}: status={status} source={source} profile={profile_id} apply={apply_count} invalid={invalid_count}".format(
                scenario=row.get("scenario", ""),
                seed=row.get("seed", ""),
                status=row.get("status", ""),
                source=row.get("selected_source_scenario", ""),
                profile_id=row.get("profile_id", ""),
                apply_count=row.get("sample_apply_count", ""),
                invalid_count=row.get("sample_invalid_count", ""),
            )
        )
        failures = str(row.get("failures", "")).strip()
        if failures:
            lines.append(f"  failures={failures}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def selected_calibration_csvs(
    *,
    sweep_dir: Path,
    scenarios: Sequence[str],
    seeds: Sequence[str],
) -> list[tuple[str, str, Path]]:
    scenario_filter = set(scenarios)
    seed_filter = set(seeds)
    selected: list[tuple[str, str, Path]] = []
    for scenario, seed, csv_path in sweep_calibration_csvs(sweep_dir):
        if scenario_filter and scenario not in scenario_filter:
            continue
        if seed_filter and seed not in seed_filter:
            continue
        selected.append((scenario, seed, csv_path))
    return selected


def run_one(
    *,
    scenario: str,
    seed: str,
    input_csv: Path,
    args: argparse.Namespace,
) -> dict[str, object]:
    run_dir = args.output_dir / scenario / seed
    runtime_csv = run_dir / "runtime_guard_input.csv"
    active_profile = run_dir / "active_runtime_profile.json"
    input_summary = export_runtime_guard_input(
        input_csv=input_csv,
        output_csv=runtime_csv,
        output_json=run_dir / "runtime_guard_input_export_summary.json",
        output_txt=run_dir / "runtime_guard_input_export_summary.txt",
    )
    export_summary = export_by_operating_mode(
        catalog_json=args.catalog_json,
        input_csv=runtime_csv,
        output_profile=active_profile,
        output_dir=run_dir / "operating_mode_export",
        sweep_dir=args.sweep_dir,
        minimal_set_json=args.minimal_set_json,
        mode="runtime_guard",
        runtime_candidate_source_mode=args.runtime_candidate_source_mode,
        runtime_selection_policy=args.runtime_selection_policy,
        transfer_matrix_csv=args.transfer_matrix_csv,
        transfer_low_risk_rmse_limit_m=args.transfer_low_risk_rmse_limit_m,
        transfer_low_risk_target_limit_m=args.transfer_low_risk_target_limit_m,
        transfer_risk_target_weight=args.transfer_risk_target_weight,
        knn_neighbor_count=args.knn_neighbor_count,
    )
    promotion = run_promotion_check(
        active_profile,
        run_dir / "promotion_check",
        sample_csv=runtime_csv,
        split=args.split,
        max_samples=args.max_samples,
    )
    failures: list[str] = []
    if input_summary.get("status") != "PASS":
        failures.append("runtime_input_export_failed")
    if not bool(export_summary.get("selected", False)):
        failures.append("operating_mode_export_not_selected")
    if export_summary.get("selected_operating_mode") != "runtime_guard":
        failures.append("operating_mode_not_runtime_guard")
    if bool(export_summary.get("has_truth_fields", True)):
        failures.append("runtime_csv_has_truth_fields")
    if not bool(promotion.get("pass_promotion_check", False)):
        failures.append("promotion_check_failed")
    if bool(promotion.get("runtime_truth_used", True)):
        failures.append("promotion_runtime_truth_used")
    status = "PASS" if not failures else "FAIL"
    selection = export_summary.get("selection_summary", {})
    if not isinstance(selection, Mapping):
        selection = {}
    return {
        "scenario": scenario,
        "seed": seed,
        "status": status,
        "input_csv": str(input_csv),
        "runtime_csv": str(runtime_csv),
        "active_profile": str(active_profile),
        "selected_source_scenario": export_summary.get("selected_source_scenario", ""),
        "preliminary_source_scenario": selection.get("preliminary_source_scenario", ""),
        "selection_policy": selection.get("selection_policy", ""),
        "calibration_required_detected": export_summary.get("calibration_required_detected", False),
        "profile_id": promotion.get("profile_id", ""),
        "profile_mode": promotion.get("profile_mode", ""),
        "sample_apply_count": promotion.get("sample_apply_count", 0),
        "sample_invalid_count": promotion.get("sample_invalid_count", 0),
        "input_export_status": input_summary.get("status", ""),
        "operating_mode": export_summary.get("selected_operating_mode", ""),
        "promotion_pass": promotion.get("pass_promotion_check", False),
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "failures": ",".join(failures),
    }


def run_batch(args: argparse.Namespace) -> dict[str, object]:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    scenario_filter = parse_csv_list(args.scenarios)
    seed_filter = parse_csv_list(args.seeds)
    csvs = selected_calibration_csvs(sweep_dir=args.sweep_dir, scenarios=scenario_filter, seeds=seed_filter)
    rows = [run_one(scenario=scenario, seed=seed, input_csv=csv_path, args=args) for scenario, seed, csv_path in csvs]
    pass_count = sum(1 for row in rows if row.get("status") == "PASS")
    failures = []
    if not rows:
        failures.append("no_calibration_csvs_selected")
    failed_rows = [f"{row.get('scenario')}/{row.get('seed')}:{row.get('failures')}" for row in rows if row.get("status") != "PASS"]
    failures.extend(failed_rows)
    summary = {
        "mode": "runtime_guard_operating_mode_batch_smoke",
        "status": "PASS" if rows and pass_count == len(rows) else "FAIL",
        "sweep_dir": str(args.sweep_dir),
        "catalog_json": str(args.catalog_json),
        "output_dir": str(args.output_dir),
        "run_count": len(rows),
        "pass_count": pass_count,
        "split": args.split,
        "runtime_candidate_source_mode": args.runtime_candidate_source_mode,
        "runtime_selection_policy": args.runtime_selection_policy,
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": (
            "Each calibration CSV is converted to a truth-free runtime guard CSV containing stereo-center "
            "RDF(right,down,forward), raw range, left image center, and measured disparity before export."
        ),
        "failure_count": len(failures),
        "failures": failures,
    }
    write_csv(args.output_dir / "runtime_guard_operating_mode_batch_smoke.csv", rows)
    write_json(args.output_dir / "runtime_guard_operating_mode_batch_smoke_summary.json", summary)
    write_summary_txt(args.output_dir / "runtime_guard_operating_mode_batch_smoke_summary.txt", summary, rows)
    return summary


def main() -> None:
    summary = run_batch(parse_args())
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
