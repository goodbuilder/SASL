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
from select_stereodistance_profile_from_catalog import PUBLIC_TOPIC_CONTRACT, RDF_AXIS_CONTRACT  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Summarize leave-one-scenario-out coverage gaps for StereoDistance calibration/profile "
            "selection. Full-catalog and LOSO calibration replay use truth only for offline selection; "
            "runtime profiles still consume raw RDF/disparity."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--full-selection-dir", type=Path)
    parser.add_argument("--loso-selection-dir", type=Path)
    parser.add_argument("--runtime-loso-dir", type=Path)
    return parser.parse_args()


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def scenario_summary_by_name(summary: Mapping[str, object]) -> dict[str, Mapping[str, object]]:
    rows = summary.get("scenario_summary", [])
    if not isinstance(rows, list):
        return {}
    output: dict[str, Mapping[str, object]] = {}
    for row in rows:
        if isinstance(row, Mapping):
            scenario = str(row.get("target_scenario", "")).strip()
            if scenario:
                output[scenario] = row
    return output


def counts_by_value(rows: Sequence[Mapping[str, object]], key: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in rows:
        value = str(row.get(key, "")).strip()
        if value:
            counts[value] = counts.get(value, 0) + 1
    return dict(sorted(counts.items(), key=lambda item: (-item[1], item[0])))


def first_finite(values: Sequence[object], default: object = "") -> object:
    for value in values:
        parsed = finite_float(value, float("nan"))
        if math.isfinite(parsed):
            return parsed
    return default


def scenario_runtime_loso_counts(runtime_loso_csv: Path | None) -> dict[str, dict[str, object]]:
    if runtime_loso_csv is None or not runtime_loso_csv.is_file():
        return {}
    rows = read_csv(runtime_loso_csv)
    by_scenario: dict[str, list[Mapping[str, str]]] = {}
    for row in rows:
        by_scenario.setdefault(str(row.get("target_scenario", "")), []).append(row)
    output: dict[str, dict[str, object]] = {}
    for scenario, scenario_rows in by_scenario.items():
        output[scenario] = {
            "runtime_loso_run_count": len(scenario_rows),
            "runtime_loso_pass_count": sum(
                1 for row in scenario_rows if parse_bool(row.get("pass_runtime_measurement_guard_replay"))
            ),
            "runtime_loso_selected_source_counts_json": json.dumps(
                counts_by_value(scenario_rows, "selected_source_scenario"), ensure_ascii=False, sort_keys=True
            ),
        }
    return output


def runtime_loso_csv_for_dir(runtime_dir: Path) -> Path:
    direct = runtime_dir / "runtime_measurement_guard_audit.csv"
    if direct.is_file():
        return direct
    legacy = runtime_dir / "runs" / "all_catalog_transfer_risk_fallback" / "runtime_measurement_guard_audit.csv"
    return legacy


def build_coverage_rows(
    *,
    full_summary: Mapping[str, object],
    loso_summary: Mapping[str, object],
    loso_run_rows: Sequence[Mapping[str, object]],
    runtime_loso: Mapping[str, Mapping[str, object]],
) -> list[dict[str, object]]:
    full_by_scenario = scenario_summary_by_name(full_summary)
    loso_by_scenario = scenario_summary_by_name(loso_summary)
    loso_runs_by_scenario: dict[str, list[Mapping[str, object]]] = {}
    for row in loso_run_rows:
        loso_runs_by_scenario.setdefault(str(row.get("target_scenario", "")), []).append(row)

    scenario_names = sorted(set(full_by_scenario) | set(loso_by_scenario) | set(loso_runs_by_scenario))
    rows: list[dict[str, object]] = []
    for scenario in scenario_names:
        full = full_by_scenario.get(scenario, {})
        loso = loso_by_scenario.get(scenario, {})
        run_rows = loso_runs_by_scenario.get(scenario, [])
        run_count = int(first_finite((full.get("run_count"), loso.get("run_count"), len(run_rows)), 0) or 0)
        full_pass = int(finite_float(full.get("calibration_pass_count"), 0.0))
        loso_pass = int(finite_float(loso.get("calibration_pass_count"), 0.0))
        full_rmse = finite_float(full.get("calibration_validation_range_rmse_corrected_mean_m"), float("nan"))
        loso_rmse = finite_float(loso.get("calibration_validation_range_rmse_corrected_mean_m"), float("nan"))
        failed_rows = [row for row in run_rows if not parse_bool(row.get("calibration_pass"))]
        top_sources = counts_by_value(failed_rows, "top_candidate_source_scenario")
        top_reasons = counts_by_value(failed_rows, "top_candidate_pass_reason")
        indispensable = run_count > 0 and full_pass == run_count and loso_pass < run_count
        if indispensable and loso_pass == 0:
            gap_type = "target_family_required"
        elif indispensable:
            gap_type = "target_family_partially_required"
        elif full_pass < run_count:
            gap_type = "full_catalog_not_validated"
        else:
            gap_type = "covered_by_other_profiles"
        runtime = runtime_loso.get(scenario, {})
        rows.append(
            {
                "target_scenario": scenario,
                "run_count": run_count,
                "full_calibration_pass_count": full_pass,
                "loso_calibration_pass_count": loso_pass,
                "calibration_pass_loss": full_pass - loso_pass,
                "coverage_gap_type": gap_type,
                "target_family_indispensable": indispensable,
                "full_calibration_rmse_mean_m": full_rmse if math.isfinite(full_rmse) else "",
                "loso_calibration_rmse_mean_m": loso_rmse if math.isfinite(loso_rmse) else "",
                "loso_minus_full_calibration_rmse_mean_m": (
                    loso_rmse - full_rmse if math.isfinite(full_rmse) and math.isfinite(loso_rmse) else ""
                ),
                "loso_failed_top_candidate_source_counts_json": json.dumps(
                    top_sources, ensure_ascii=False, sort_keys=True
                ),
                "loso_failed_top_candidate_reason_counts_json": json.dumps(
                    top_reasons, ensure_ascii=False, sort_keys=True
                ),
                "runtime_loso_pass_count": runtime.get("runtime_loso_pass_count", ""),
                "runtime_loso_run_count": runtime.get("runtime_loso_run_count", ""),
                "runtime_loso_selected_source_counts_json": runtime.get("runtime_loso_selected_source_counts_json", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "selection_contract": (
                    "Calibration replay may use calibration-time truth for offline selection; "
                    "runtime profile application remains raw RDF/disparity only."
                ),
            }
        )
    return rows


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "StereoDistance LOSO coverage gap summary",
        "",
        f"axis_contract: {RDF_AXIS_CONTRACT}",
        f"public_topic_contract: {PUBLIC_TOPIC_CONTRACT}",
        "selection_contract: calibration truth is offline selection evidence only; runtime profiles use raw RDF/disparity",
        "",
        "Summary:",
    ]
    for key in (
        "scenario_count",
        "indispensable_scenario_count",
        "partial_gap_scenario_count",
        "full_gap_scenario_count",
        "runtime_loso_available",
    ):
        lines.append(f"{key}: {summary.get(key, '')}")
    lines.extend(["", "Scenario gaps:"])
    for row in rows:
        lines.append(
            "{scenario}: gap={gap}, full={full}/{total}, loso={loso}/{total}, "
            "runtime_loso={runtime}/{runtime_total}, failed_top_sources={sources}, failed_reasons={reasons}".format(
                scenario=row.get("target_scenario", ""),
                gap=row.get("coverage_gap_type", ""),
                full=row.get("full_calibration_pass_count", ""),
                loso=row.get("loso_calibration_pass_count", ""),
                total=row.get("run_count", ""),
                runtime=row.get("runtime_loso_pass_count", ""),
                runtime_total=row.get("runtime_loso_run_count", ""),
                sources=row.get("loso_failed_top_candidate_source_counts_json", ""),
                reasons=row.get("loso_failed_top_candidate_reason_counts_json", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_gaps(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    labels = [str(row["target_scenario"]) for row in rows]
    run_count = np.asarray([finite_float(row.get("run_count"), 0.0) for row in rows], dtype=float)
    full_pass = np.asarray([finite_float(row.get("full_calibration_pass_count"), 0.0) for row in rows], dtype=float)
    loso_pass = np.asarray([finite_float(row.get("loso_calibration_pass_count"), 0.0) for row in rows], dtype=float)
    x = np.arange(len(rows))
    fig, ax = plt.subplots(figsize=(max(9, len(rows) * 1.2), 5), constrained_layout=True)
    width = 0.35
    ax.bar(x - width / 2, full_pass, width=width, label="full catalog")
    ax.bar(x + width / 2, loso_pass, width=width, label="LOSO catalog")
    ax.plot(x, run_count, color="black", marker="o", linewidth=1, label="run count")
    ax.set_ylabel("calibration replay pass count")
    ax.set_xticks(x, labels, rotation=35, ha="right")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def summarize_coverage_gaps(
    *,
    sweep_dir: Path,
    output_dir: Path,
    full_selection_dir: Path | None = None,
    loso_selection_dir: Path | None = None,
    runtime_loso_dir: Path | None = None,
) -> dict[str, object]:
    full_dir = full_selection_dir or (sweep_dir / "profile_selection_strategy_audit_all_catalog")
    loso_dir = loso_selection_dir or (sweep_dir / "profile_selection_strategy_audit_all_catalog_loso")
    runtime_dir = runtime_loso_dir or (sweep_dir / "runtime_measurement_guard_policy_audit_loso")
    full_summary = read_json(full_dir / "profile_selection_strategy_summary.json")
    loso_summary = read_json(loso_dir / "profile_selection_strategy_summary.json")
    loso_rows = read_csv(loso_dir / "profile_selection_strategy_runs.csv")
    runtime_csv = runtime_loso_csv_for_dir(runtime_dir)
    runtime_by_scenario = scenario_runtime_loso_counts(runtime_csv)
    rows = build_coverage_rows(
        full_summary=full_summary,
        loso_summary=loso_summary,
        loso_run_rows=loso_rows,
        runtime_loso=runtime_by_scenario,
    )
    indispensable = [row for row in rows if parse_bool(row.get("target_family_indispensable"))]
    full_gaps = [row for row in indispensable if row.get("coverage_gap_type") == "target_family_required"]
    partial_gaps = [
        row for row in indispensable if row.get("coverage_gap_type") == "target_family_partially_required"
    ]
    summary = {
        "mode": "stereodistance_loso_coverage_gap_summary",
        "sweep_dir": str(sweep_dir),
        "full_selection_dir": str(full_dir),
        "loso_selection_dir": str(loso_dir),
        "runtime_loso_dir": str(runtime_dir),
        "scenario_count": len(rows),
        "indispensable_scenario_count": len(indispensable),
        "full_gap_scenario_count": len(full_gaps),
        "partial_gap_scenario_count": len(partial_gaps),
        "indispensable_scenarios": [row["target_scenario"] for row in indispensable],
        "full_gap_scenarios": [row["target_scenario"] for row in full_gaps],
        "partial_gap_scenarios": [row["target_scenario"] for row in partial_gaps],
        "runtime_loso_available": bool(runtime_by_scenario),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Calibration replay may use calibration-time truth for offline selection; "
            "runtime profile application remains raw RDF/disparity only."
        ),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "loso_coverage_gap_scenarios.csv", rows)
    write_json(output_dir / "loso_coverage_gap_summary.json", summary)
    write_summary_txt(output_dir / "loso_coverage_gap_summary.txt", summary, rows)
    plot_gaps(output_dir / "loso_coverage_gap_pass_counts.png", rows)
    return summary


def main() -> int:
    args = parse_args()
    summary = summarize_coverage_gaps(
        sweep_dir=args.sweep_dir,
        output_dir=args.output_dir,
        full_selection_dir=args.full_selection_dir,
        loso_selection_dir=args.loso_selection_dir,
        runtime_loso_dir=args.runtime_loso_dir,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
