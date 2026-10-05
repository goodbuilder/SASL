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

from audit_stereodistance_profile_selection_strategies import (  # noqa: E402
    parse_bool,
    passed_seed_runs,
    scenario_names_from_runs,
)
from explore_stereodistance_profile_blends import (  # noqa: E402
    explore_blends,
    parse_float_list,
)
from select_stereodistance_profile_by_calibration_replay import (  # noqa: E402
    finite_float,
    parse_split_list,
    read_csv,
    write_csv,
    write_json,
)
from select_stereodistance_profile_from_catalog import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit catalog-profile blend policies across every scenario/seed in a robustness sweep. "
            "The audit uses calibration-time truth only; exported profiles remain normal runtime profiles."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--stability-splits", default="train,validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--alphas", default="0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1")
    parser.add_argument(
        "--candidate-source-mode",
        choices=("minimal_set", "all_catalog"),
        default="minimal_set",
        help=(
            "minimal_set blends deployment-ready set-cover candidates; all_catalog explores every "
            "validated catalog profile pair as an offline diagnostic."
        ),
    )
    return parser.parse_args()


def first_row(path: Path) -> dict[str, str]:
    rows = read_csv(path)
    if not rows:
        raise ValueError(f"CSV has no rows: {path}")
    return rows[0]


def policy_rows(path: Path) -> list[dict[str, str]]:
    return read_csv(path)


def audit_blend_strategies(
    *,
    sweep_dir: Path,
    output_dir: Path,
    catalog_json: Path | None = None,
    minimal_set_json: Path | None = None,
    split: str = "validation",
    stability_splits: Sequence[str] = ("train", "validation"),
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
    alphas: Sequence[float] = tuple(i / 10.0 for i in range(11)),
    candidate_source_mode: str = "minimal_set",
) -> tuple[list[dict[str, object]], list[dict[str, object]], dict[str, object]]:
    catalog_path = catalog_json or (sweep_dir / "scenario_profile_catalog" / "manifest.json")
    minimal_path = minimal_set_json or (sweep_dir / "scenario_profile_catalog_minimal_set.json")
    run_rows: list[dict[str, object]] = []
    output_dir.mkdir(parents=True, exist_ok=True)
    per_run_root = output_dir / "per_run_blend_exploration"
    for scenario in scenario_names_from_runs(sweep_dir):
        for seed_row in passed_seed_runs(sweep_dir / scenario):
            seed = int(float(seed_row.get("seed", 0) or 0))
            sample_csv = Path(str(seed_row["run_dir"])) / "calibration_samples.csv"
            run_output = per_run_root / scenario / f"seed_{seed}"
            run_profile = run_output / "active_runtime_profile_blend.json"
            try:
                selection = explore_blends(
                    catalog_json=catalog_path,
                    input_csv=sample_csv,
                    output_dir=run_output,
                    output_profile=run_profile,
                    minimal_set_json=minimal_path,
                    split=split,
                    stability_splits=stability_splits,
                    target_range_m=target_range_m,
                    target_window_m=target_window_m,
                    target_error_limit_m=target_error_limit_m,
                    alphas=alphas,
                    candidate_source_mode=candidate_source_mode,
                )
                policy_csv = run_output / "profile_blend_policy_recommendations.csv"
                for row in policy_rows(policy_csv):
                    run_rows.append(
                        {
                            "target_scenario": scenario,
                            "seed": seed,
                            "sample_csv": str(sample_csv),
                            "policy": row.get("policy", ""),
                            "selected": True,
                            "source_a": row.get("source_a", ""),
                            "source_b": row.get("source_b", ""),
                            "alpha_second_profile": row.get("alpha_second_profile", ""),
                            "pass_replay": parse_bool(row.get("pass_replay")),
                            "validation_range_rmse_corrected_m": row.get("validation_range_rmse_corrected_m", ""),
                            "target_abs_mean_error_corrected_m": row.get("target_abs_mean_error_corrected_m", ""),
                            "target_max_abs_error_corrected_m": row.get("target_max_abs_error_corrected_m", ""),
                            "stability_min_range_improvement_m": row.get("stability_min_range_improvement_m", ""),
                            "replay_selection_score": row.get("replay_selection_score", ""),
                            "is_pareto_frontier": row.get("is_pareto_frontier", ""),
                            "run_output_dir": str(run_output),
                            "selected_output_profile": selection.get("output_profile", ""),
                            "candidate_source_mode": candidate_source_mode,
                            "axis_contract": RDF_AXIS_CONTRACT,
                            "runtime_input_contract": "blend audit uses calibration truth offline; runtime profiles consume raw RDF/disparity only",
                        }
                    )
            except Exception as exc:
                run_rows.append(
                    {
                        "target_scenario": scenario,
                        "seed": seed,
                        "sample_csv": str(sample_csv),
                        "policy": "",
                        "selected": False,
                        "pass_replay": False,
                        "failure_reason": str(exc),
                        "run_output_dir": str(run_output),
                        "axis_contract": RDF_AXIS_CONTRACT,
                        "runtime_input_contract": "blend audit failed before runtime profile promotion",
                    }
                )

    policy_groups: dict[str, list[Mapping[str, object]]] = {}
    for row in run_rows:
        policy = str(row.get("policy", ""))
        if policy:
            policy_groups.setdefault(policy, []).append(row)
    policy_summary: list[dict[str, object]] = []
    for policy, rows in sorted(policy_groups.items()):
        rmse_values = [
            finite_float(row.get("validation_range_rmse_corrected_m"), float("nan"))
            for row in rows
            if parse_bool(row.get("pass_replay"))
        ]
        target_values = [
            finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan"))
            for row in rows
            if parse_bool(row.get("pass_replay"))
        ]
        target_max_values = [
            finite_float(row.get("target_max_abs_error_corrected_m"), float("nan"))
            for row in rows
            if parse_bool(row.get("pass_replay"))
        ]
        stability_values = [
            finite_float(row.get("stability_min_range_improvement_m"), float("nan"))
            for row in rows
            if parse_bool(row.get("pass_replay"))
        ]
        finite_rmse = [value for value in rmse_values if math.isfinite(value)]
        finite_target = [value for value in target_values if math.isfinite(value)]
        finite_target_max = [value for value in target_max_values if math.isfinite(value)]
        finite_stability = [value for value in stability_values if math.isfinite(value)]
        policy_summary.append(
            {
                "policy": policy,
                "run_count": len(rows),
                "pass_count": sum(1 for row in rows if parse_bool(row.get("pass_replay"))),
                "validation_range_rmse_corrected_mean_m": (
                    sum(finite_rmse) / len(finite_rmse) if finite_rmse else float("inf")
                ),
                "validation_range_rmse_corrected_max_m": max(finite_rmse) if finite_rmse else float("inf"),
                "target_abs_mean_error_mean_m": (
                    sum(finite_target) / len(finite_target) if finite_target else float("inf")
                ),
                "target_abs_mean_error_max_m": max(finite_target) if finite_target else float("inf"),
                "target_max_abs_error_mean_m": (
                    sum(finite_target_max) / len(finite_target_max) if finite_target_max else float("inf")
                ),
                "target_max_abs_error_max_m": max(finite_target_max) if finite_target_max else float("inf"),
                "stability_min_range_improvement_min_m": min(finite_stability) if finite_stability else float("-inf"),
                "alpha_values": ",".join(
                    sorted({str(row.get("alpha_second_profile", "")) for row in rows if row.get("alpha_second_profile", "")})
                ),
                "axis_contract": RDF_AXIS_CONTRACT,
            }
        )

    summary = {
        "blend_strategy_audit": "catalog_profile_blend_policy_replay",
        "sweep_dir": str(sweep_dir),
        "catalog_json": str(catalog_path),
        "minimal_set_json": str(minimal_path),
        "run_count": len(run_rows),
        "policy_count": len(policy_summary),
        "stability_splits_requested": list(stability_splits),
        "target_error_limit_m": target_error_limit_m,
        "candidate_source_mode": candidate_source_mode,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Blend policies are selected with calibration-time truth only. Exported runtime profiles "
            "must consume raw RDF/disparity and fixed calibration parameters only."
        ),
        "policy_summary": policy_summary,
    }
    write_csv(output_dir / "blend_strategy_policy_runs.csv", run_rows)
    write_csv(output_dir / "blend_strategy_policy_summary.csv", policy_summary)
    write_json(output_dir / "blend_strategy_summary.json", summary)
    write_summary_txt(output_dir / "blend_strategy_summary.txt", summary, policy_summary)
    return run_rows, policy_summary, summary


def write_summary_txt(path: Path, summary: Mapping[str, object], policy_summary: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "StereoDistance blend strategy audit",
        "",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        f"candidate_source_mode: {summary.get('candidate_source_mode', '')}",
        f"stability_splits_requested: {','.join(summary.get('stability_splits_requested', []) or [])}",
        "",
        "Policy summary:",
    ]
    for row in policy_summary:
        lines.append(
            (
                "- {policy}: runs={runs}, pass={passed}, rmse_mean={rmse_mean:.6f}m, "
                "rmse_max={rmse_max:.6f}m, target_mean={target_mean:.6f}m, "
                "target_abs_max={target_max:.6f}m, target_max_abs_max={target_max_abs:.6f}m, "
                "min_stability_improvement={stability:.6f}m, "
                "alpha_values={alphas}"
            ).format(
                policy=row.get("policy", ""),
                runs=row.get("run_count", 0),
                passed=row.get("pass_count", 0),
                rmse_mean=finite_float(row.get("validation_range_rmse_corrected_mean_m"), float("inf")),
                rmse_max=finite_float(row.get("validation_range_rmse_corrected_max_m"), float("inf")),
                target_mean=finite_float(row.get("target_abs_mean_error_mean_m"), float("inf")),
                target_max=finite_float(row.get("target_abs_mean_error_max_m"), float("inf")),
                target_max_abs=finite_float(row.get("target_max_abs_error_max_m"), float("inf")),
                stability=finite_float(row.get("stability_min_range_improvement_min_m"), float("-inf")),
                alphas=row.get("alpha_values", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    _, _, summary = audit_blend_strategies(
        sweep_dir=args.sweep_dir,
        output_dir=args.output_dir,
        catalog_json=args.catalog_json,
        minimal_set_json=args.minimal_set_json,
        split=args.split,
        stability_splits=parse_split_list(args.stability_splits),
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
        alphas=parse_float_list(args.alphas),
        candidate_source_mode=args.candidate_source_mode,
    )
    print(f"wrote blend strategy audit to {args.output_dir}")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
