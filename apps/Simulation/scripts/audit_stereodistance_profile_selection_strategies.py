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

from select_stereodistance_profile_by_calibration_replay import (  # noqa: E402
    candidate_source_scenarios,
    finite_float,
    parse_split_list,
    read_csv,
    replay_candidate,
    replay_selection_score,
    write_csv,
    write_json,
)
from select_stereodistance_profile_from_catalog import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    find_minimal_set_source_scenario,
    load_catalog,
    load_minimal_set,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit deployment-label profile selection against calibration-replay profile selection "
            "for every scenario/seed in a robustness sweep output."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--split", default="validation")
    parser.add_argument(
        "--stability-splits",
        default="train,validation",
        help="Comma-separated split labels that must also replay cleanly when present in the CSV.",
    )
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument(
        "--candidate-source-mode",
        choices=("minimal_set", "all_catalog"),
        default="minimal_set",
        help=(
            "minimal_set audits deployment-ready set-cover candidates; all_catalog lets calibration replay "
            "search every validated catalog profile as an offline diagnostic."
        ),
    )
    parser.add_argument(
        "--exclude-target-scenario-from-candidates",
        action="store_true",
        help=(
            "Run a leave-one-scenario-out calibration replay audit by removing each target scenario "
            "from candidate source profiles before selecting with calibration-time truth."
        ),
    )
    parser.add_argument("--allow-unvalidated", action="store_true")
    return parser.parse_args()


def parse_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"true", "1", "yes", "y"}


def scenario_names_from_runs(sweep_dir: Path) -> list[str]:
    rows = read_csv(sweep_dir / "scenario_runs.csv")
    names = [str(row.get("scenario", "")) for row in rows if str(row.get("scenario", ""))]
    if names:
        return names
    return sorted(path.name for path in sweep_dir.iterdir() if (path / "seed_runs.csv").exists())


def passed_seed_runs(scenario_dir: Path) -> list[dict[str, str]]:
    return [row for row in read_csv(scenario_dir / "seed_runs.csv") if parse_bool(row.get("pass_run"))]


def choose_best_candidate(candidate_rows: Sequence[Mapping[str, object]]) -> Mapping[str, object] | None:
    passing = [row for row in candidate_rows if parse_bool(row.get("pass_replay"))]
    if not passing:
        return None
    return min(
        passing,
        key=lambda row: (
            finite_float(row.get("replay_selection_score"), replay_selection_score(row)),
            finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")),
            finite_float(row.get("model_complexity_estimate"), float("inf")),
            str(row.get("scenario", "")),
        ),
    )


def compact_candidate_diagnostic(row: Mapping[str, object]) -> dict[str, object]:
    return {
        "scenario": row.get("scenario", ""),
        "method": row.get("method", ""),
        "pass_replay": parse_bool(row.get("pass_replay")),
        "pass_reason": row.get("pass_reason", ""),
        "validation_range_rmse_corrected_m": (
            value
            if math.isfinite(
                value := finite_float(row.get("validation_range_rmse_corrected_m"), float("nan"))
            )
            else None
        ),
        "target_abs_mean_error_corrected_m": (
            value
            if math.isfinite(
                value := finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan"))
            )
            else None
        ),
        "target_max_abs_error_corrected_m": (
            value
            if math.isfinite(
                value := finite_float(row.get("target_max_abs_error_corrected_m"), float("nan"))
            )
            else None
        ),
        "replay_selection_score": (
            value
            if math.isfinite(value := finite_float(row.get("replay_selection_score"), replay_selection_score(row)))
            else None
        ),
    }


def ranked_candidate_diagnostics(candidate_rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    ranked = sorted(
        candidate_rows,
        key=lambda row: (
            not parse_bool(row.get("pass_replay")),
            finite_float(row.get("replay_selection_score"), replay_selection_score(row)),
            finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")),
            str(row.get("scenario", "")),
        ),
    )
    return [compact_candidate_diagnostic(row) for row in ranked]


def audit_selection_strategies(
    *,
    sweep_dir: Path,
    output_dir: Path,
    catalog_json: Path | None = None,
    minimal_set_json: Path | None = None,
    split: str = "validation",
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
    allow_unvalidated: bool = False,
    stability_splits: Sequence[str] = ("train", "validation"),
    candidate_source_mode: str = "minimal_set",
    exclude_target_scenario_from_candidates: bool = False,
) -> tuple[list[dict[str, object]], list[dict[str, object]], dict[str, object]]:
    catalog_path = catalog_json or (sweep_dir / "scenario_profile_catalog" / "manifest.json")
    minimal_path = minimal_set_json or (sweep_dir / "scenario_profile_catalog_minimal_set.json")
    catalog = load_catalog(catalog_path)
    minimal_set = load_minimal_set(minimal_path)
    if candidate_source_mode == "all_catalog":
        candidate_sources = candidate_source_scenarios(catalog, None)
    elif candidate_source_mode == "minimal_set":
        candidate_sources = candidate_source_scenarios(catalog, minimal_path)
    else:
        raise ValueError("candidate_source_mode must be 'minimal_set' or 'all_catalog'")

    run_rows: list[dict[str, object]] = []
    for scenario in scenario_names_from_runs(sweep_dir):
        label_source = find_minimal_set_source_scenario(minimal_set, scenario)
        eligible_candidate_sources = list(candidate_sources)
        if exclude_target_scenario_from_candidates:
            eligible_candidate_sources = [
                source for source in eligible_candidate_sources if source != scenario
            ]
        if not eligible_candidate_sources:
            raise ValueError("target-scenario exclusion removed every calibration replay candidate")
        for seed_row in passed_seed_runs(sweep_dir / scenario):
            seed = int(float(seed_row.get("seed", 0) or 0))
            sample_csv = Path(str(seed_row["run_dir"])) / "calibration_samples.csv"
            rows = read_csv(sample_csv)
            candidate_rows = [
                replay_candidate(
                    catalog_json=catalog_path,
                    catalog=catalog,
                    scenario=source,
                    rows=rows,
                    split=split,
                    target_range_m=target_range_m,
                    target_window_m=target_window_m,
                    target_error_limit_m=target_error_limit_m,
                    allow_unvalidated=allow_unvalidated,
                    stability_splits=stability_splits,
                )
                for source in eligible_candidate_sources
            ]
            label_row = next(
                (row for row in candidate_rows if str(row.get("scenario", "")) == label_source),
                None,
            )
            calibration_row = choose_best_candidate(candidate_rows)
            ranked_diagnostics = ranked_candidate_diagnostics(candidate_rows)
            top_candidate = ranked_diagnostics[0] if ranked_diagnostics else {}
            selected = calibration_row is not None
            calibration_source = str(calibration_row.get("scenario", "")) if calibration_row else ""
            label_rmse_value = finite_float(
                label_row.get("validation_range_rmse_corrected_m") if label_row else None,
                float("inf"),
            )
            label_rmse = label_rmse_value if math.isfinite(label_rmse_value) else ""
            calibration_rmse = finite_float(
                calibration_row.get("validation_range_rmse_corrected_m") if calibration_row else None,
                float("inf"),
            )
            run_rows.append(
                {
                    "target_scenario": scenario,
                    "seed": seed,
                    "sample_csv": str(sample_csv),
                    "label_source_scenario": label_source,
                    "label_source_available": label_row is not None,
                    "label_pass": parse_bool(label_row.get("pass_replay")) if label_row else False,
                    "label_validation_range_rmse_corrected_m": label_rmse,
                    "label_target_abs_mean_error_corrected_m": (
                        label_row.get("target_abs_mean_error_corrected_m", "") if label_row else ""
                    ),
                    "calibration_selected": selected,
                    "calibration_source_scenario": calibration_source,
                    "calibration_pass": bool(calibration_row),
                    "calibration_validation_range_rmse_corrected_m": (
                        calibration_rmse if math.isfinite(calibration_rmse) else ""
                    ),
                    "calibration_target_abs_mean_error_corrected_m": (
                        calibration_row.get("target_abs_mean_error_corrected_m", "") if calibration_row else ""
                    ),
                    "calibration_stability_pass": (
                        calibration_row.get("stability_pass", "") if calibration_row else ""
                    ),
                    "calibration_stability_min_range_improvement_m": (
                        calibration_row.get("stability_min_range_improvement_m", "") if calibration_row else ""
                    ),
                    "top_candidate_source_scenario": top_candidate.get("scenario", ""),
                    "top_candidate_pass": top_candidate.get("pass_replay", ""),
                    "top_candidate_pass_reason": top_candidate.get("pass_reason", ""),
                    "top_candidate_validation_range_rmse_corrected_m": top_candidate.get(
                        "validation_range_rmse_corrected_m", ""
                    ),
                    "top_candidate_target_abs_mean_error_corrected_m": top_candidate.get(
                        "target_abs_mean_error_corrected_m", ""
                    ),
                    "candidate_ranked_diagnostics_json": json.dumps(
                        ranked_diagnostics[:5], ensure_ascii=False, allow_nan=False
                    ),
                    "calibration_beats_label_rmse": (
                        selected and math.isfinite(label_rmse_value) and calibration_rmse < label_rmse_value
                    ),
                    "calibration_matches_label_source": selected and calibration_source == label_source,
                    "candidate_source_scenarios": ",".join(candidate_sources),
                    "eligible_candidate_source_scenarios": ",".join(eligible_candidate_sources),
                    "candidate_source_mode": candidate_source_mode,
                    "exclude_target_scenario_from_candidates": bool(exclude_target_scenario_from_candidates),
                    "axis_contract": RDF_AXIS_CONTRACT,
                    "runtime_input_contract": "audit uses calibration-time truth only; runtime profile inputs remain raw RDF/disparity",
                }
            )

    scenario_groups: dict[str, list[Mapping[str, object]]] = {}
    for row in run_rows:
        scenario_groups.setdefault(str(row["target_scenario"]), []).append(row)
    scenario_rows: list[dict[str, object]] = []
    for scenario, rows in sorted(scenario_groups.items()):
        label_rmse_values = [
            finite_float(row.get("label_validation_range_rmse_corrected_m"), float("nan"))
            for row in rows
        ]
        calibration_rmse_values = [
            finite_float(row.get("calibration_validation_range_rmse_corrected_m"), float("nan"))
            for row in rows
            if parse_bool(row.get("calibration_selected"))
        ]
        finite_label = [value for value in label_rmse_values if math.isfinite(value)]
        finite_calibration = [value for value in calibration_rmse_values if math.isfinite(value)]
        scenario_rows.append(
            {
                "target_scenario": scenario,
                "run_count": len(rows),
                "label_pass_count": sum(1 for row in rows if parse_bool(row.get("label_pass"))),
                "calibration_pass_count": sum(1 for row in rows if parse_bool(row.get("calibration_pass"))),
                "calibration_matches_label_count": sum(
                    1 for row in rows if parse_bool(row.get("calibration_matches_label_source"))
                ),
                "calibration_beats_label_count": sum(
                    1 for row in rows if parse_bool(row.get("calibration_beats_label_rmse"))
                ),
                "label_validation_range_rmse_corrected_mean_m": (
                    sum(finite_label) / len(finite_label) if finite_label else None
                ),
                "calibration_validation_range_rmse_corrected_mean_m": (
                    sum(finite_calibration) / len(finite_calibration) if finite_calibration else None
                ),
                "axis_contract": RDF_AXIS_CONTRACT,
            }
        )

    summary = {
        "selection_strategy_audit": "deployment_label_minimal_set_vs_calibration_replay",
        "sweep_dir": str(sweep_dir),
        "catalog_json": str(catalog_path),
        "minimal_set_json": str(minimal_path),
        "candidate_source_scenarios": candidate_sources,
        "candidate_source_mode": candidate_source_mode,
        "exclude_target_scenario_from_candidates": bool(exclude_target_scenario_from_candidates),
        "stability_splits_requested": list(stability_splits),
        "run_count": len(run_rows),
        "scenario_count": len(scenario_rows),
        "label_pass_count": sum(1 for row in run_rows if parse_bool(row.get("label_pass"))),
        "calibration_pass_count": sum(1 for row in run_rows if parse_bool(row.get("calibration_pass"))),
        "calibration_matches_label_count": sum(
            1 for row in run_rows if parse_bool(row.get("calibration_matches_label_source"))
        ),
        "calibration_beats_label_count": sum(
            1 for row in run_rows if parse_bool(row.get("calibration_beats_label_rmse"))
        ),
        "selection_contract": (
            "Deployment-label selection uses known calibration/deployment scenario labels. "
            "Calibration-replay selection may use calibration-time truth to choose a prevalidated profile. "
            "Neither strategy permits runtime truth input."
        ),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "selected runtime profiles consume stereo-center RDF(right,down,forward) plus measured disparity",
        "scenario_summary": scenario_rows,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "profile_selection_strategy_runs.csv", run_rows)
    write_csv(output_dir / "profile_selection_strategy_scenarios.csv", scenario_rows)
    write_json(output_dir / "profile_selection_strategy_summary.json", summary)
    return run_rows, scenario_rows, summary


def write_summary_txt(path: Path, summary: Mapping[str, object], scenario_rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "StereoDistance profile selection strategy audit",
        "",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        f"candidate_source_scenarios: {','.join(summary.get('candidate_source_scenarios', []) or [])}",
        f"candidate_source_mode: {summary.get('candidate_source_mode', '')}",
        f"exclude_target_scenario_from_candidates: {summary.get('exclude_target_scenario_from_candidates', '')}",
        f"stability_splits_requested: {','.join(summary.get('stability_splits_requested', []) or [])}",
        "",
        "Overall:",
        (
            "- runs={runs}, label_pass={label}, calibration_pass={cal}, "
            "matches={matches}, calibration_beats_label={beats}"
        ).format(
            runs=summary.get("run_count", 0),
            label=summary.get("label_pass_count", 0),
            cal=summary.get("calibration_pass_count", 0),
            matches=summary.get("calibration_matches_label_count", 0),
            beats=summary.get("calibration_beats_label_count", 0),
        ),
        "",
        "By scenario:",
    ]
    for row in scenario_rows:
        lines.append(
            (
                "- {scenario}: runs={runs}, label_pass={label}, calibration_pass={cal}, "
                "matches={matches}, beats={beats}, label_rmse_mean={label_rmse:.6f}m, "
                "calibration_rmse_mean={cal_rmse:.6f}m"
            ).format(
                scenario=row.get("target_scenario", ""),
                runs=row.get("run_count", 0),
                label=row.get("label_pass_count", 0),
                cal=row.get("calibration_pass_count", 0),
                matches=row.get("calibration_matches_label_count", 0),
                beats=row.get("calibration_beats_label_count", 0),
                label_rmse=finite_float(row.get("label_validation_range_rmse_corrected_mean_m"), float("inf")),
                cal_rmse=finite_float(row.get("calibration_validation_range_rmse_corrected_mean_m"), float("inf")),
            )
        )
    lines.extend(["", f"selection_contract: {summary.get('selection_contract', '')}"])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    _, scenario_rows, summary = audit_selection_strategies(
        sweep_dir=args.sweep_dir,
        output_dir=args.output_dir,
        catalog_json=args.catalog_json,
        minimal_set_json=args.minimal_set_json,
        split=args.split,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
        allow_unvalidated=args.allow_unvalidated,
        stability_splits=parse_split_list(args.stability_splits),
        candidate_source_mode=args.candidate_source_mode,
        exclude_target_scenario_from_candidates=args.exclude_target_scenario_from_candidates,
    )
    write_summary_txt(args.output_dir / "profile_selection_strategy_summary.txt", summary, scenario_rows)
    print(f"wrote profile selection strategy audit to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
