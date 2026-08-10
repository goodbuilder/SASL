#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Mapping, Sequence


def read_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


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


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_float(value: object, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value: object, default: int = 0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def row_from_result_dir(result_dir: Path) -> dict[str, object]:
    metrics = read_json(result_dir / "multiburst_observation_metrics.json")
    best = metrics.get("best_method", {})
    if not isinstance(best, dict):
        best = {}
    return {
        "scenario_dir": result_dir.name,
        "experiment_id": metrics.get("experiment_id", ""),
        "scenario": metrics.get("scenario", ""),
        "profile_json": metrics.get("profile_json", ""),
        "method": best.get("method", ""),
        "status": best.get("status", ""),
        "pass_count": best.get("pass_count", 0),
        "trial_count": best.get("trial_count", 0),
        "burst_count": best.get("burst_count", ""),
        "observations_per_burst": best.get("observations_per_burst", ""),
        "total_observation_count": best.get("total_observation_count", ""),
        "inner_mode": best.get("inner_mode", ""),
        "outer_mode": best.get("outer_mode", ""),
        "trim_fraction": best.get("trim_fraction", ""),
        "common_fraction": best.get("common_fraction", ""),
        "range_rmse_raw_mean_m": best.get("range_rmse_raw_mean_m", ""),
        "range_rmse_corrected_mean_m": best.get("range_rmse_corrected_mean_m", ""),
        "position_rmse_raw_mean_m": best.get("position_rmse_raw_mean_m", ""),
        "position_rmse_corrected_mean_m": best.get("position_rmse_corrected_mean_m", ""),
        "target_max_abs_error_corrected_max_m": best.get("target_max_abs_error_corrected_max_m", ""),
        "target_rmse_corrected_mean_m": best.get("target_rmse_corrected_mean_m", ""),
        "runtime_truth_used": best.get("runtime_truth_used", False),
        "runtime_input_contract": best.get("runtime_input_contract", metrics.get("runtime_input_contract", "")),
        "rdf_axis_contract": best.get("rdf_axis_contract", metrics.get("rdf_axis_contract", "")),
        "public_topic_contract": best.get("public_topic_contract", metrics.get("public_topic_contract", "")),
    }


def discover_result_dirs(root: Path) -> list[Path]:
    if not root.is_dir():
        return []
    return [
        child
        for child in sorted(root.iterdir())
        if child.is_dir() and (child / "multiburst_observation_metrics.json").is_file()
    ]


def method_rows_from_result_dir(result_dir: Path) -> list[dict[str, object]]:
    summary_csv = result_dir / "multiburst_observation_method_summary.csv"
    if not summary_csv.is_file():
        return []
    rows: list[dict[str, object]] = []
    for row in read_csv_rows(summary_csv):
        out: dict[str, object] = dict(row)
        out["scenario_dir"] = result_dir.name
        out["source_csv"] = str(summary_csv)
        out["pass_count"] = as_int(row.get("pass_count"))
        out["trial_count"] = as_int(row.get("trial_count"))
        out["burst_count"] = as_int(row.get("burst_count"))
        out["observations_per_burst"] = as_int(row.get("observations_per_burst"))
        out["total_observation_count"] = as_int(row.get("total_observation_count"))
        out["trim_fraction"] = as_float(row.get("trim_fraction"))
        out["common_fraction"] = as_float(row.get("common_fraction"))
        out["range_rmse_raw_mean_m"] = as_float(row.get("range_rmse_raw_mean_m"))
        out["range_rmse_corrected_mean_m"] = as_float(row.get("range_rmse_corrected_mean_m"))
        out["range_rmse_corrected_max_m"] = as_float(row.get("range_rmse_corrected_max_m"))
        out["position_rmse_raw_mean_m"] = as_float(row.get("position_rmse_raw_mean_m"))
        out["position_rmse_corrected_mean_m"] = as_float(row.get("position_rmse_corrected_mean_m"))
        out["position_rmse_corrected_max_m"] = as_float(row.get("position_rmse_corrected_max_m"))
        out["target_max_abs_error_corrected_max_m"] = as_float(row.get("target_max_abs_error_corrected_max_m"))
        out["target_rmse_corrected_mean_m"] = as_float(row.get("target_rmse_corrected_mean_m"))
        out["runtime_truth_used"] = str(row.get("runtime_truth_used", "")).lower() == "true"
        rows.append(out)
    return rows


def aggregate_method_rows(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row.get("method", "")), []).append(row)

    aggregates: list[dict[str, object]] = []
    for method, group in grouped.items():
        scenario_count = len({str(row.get("scenario", row.get("scenario_dir", ""))) for row in group})
        strict_rows = [row for row in group if row.get("status") == "STRICT_PASS"]
        target_max_values = [as_float(row.get("target_max_abs_error_corrected_max_m"), float("inf")) for row in group]
        range_values = [as_float(row.get("range_rmse_corrected_mean_m"), float("inf")) for row in group]
        position_values = [as_float(row.get("position_rmse_corrected_mean_m"), float("inf")) for row in group]
        first = group[0]
        aggregates.append(
            {
                "method": method,
                "scenario_count": scenario_count,
                "strict_pass_scenario_count": len(strict_rows),
                "all_scenarios_strict_pass": len(strict_rows) == len(group),
                "total_observation_count": first.get("total_observation_count", ""),
                "burst_count": first.get("burst_count", ""),
                "observations_per_burst": first.get("observations_per_burst", ""),
                "inner_mode": first.get("inner_mode", ""),
                "outer_mode": first.get("outer_mode", ""),
                "trim_fraction": first.get("trim_fraction", ""),
                "common_fraction": first.get("common_fraction", ""),
                "target_max_abs_error_worst_m": max(target_max_values) if target_max_values else "",
                "range_rmse_corrected_mean_across_scenarios_m": (
                    sum(range_values) / len(range_values) if range_values else ""
                ),
                "position_rmse_corrected_mean_across_scenarios_m": (
                    sum(position_values) / len(position_values) if position_values else ""
                ),
                "runtime_truth_used": any(bool(row.get("runtime_truth_used", False)) for row in group),
                "runtime_input_contract": first.get("runtime_input_contract", ""),
                "rdf_axis_contract": first.get("rdf_axis_contract", ""),
                "public_topic_contract": first.get("public_topic_contract", ""),
            }
        )
    return sorted(
        aggregates,
        key=lambda row: (
            0 if row.get("all_scenarios_strict_pass") is True else 1,
            as_float(row.get("target_max_abs_error_worst_m"), float("inf")),
            as_float(row.get("range_rmse_corrected_mean_across_scenarios_m"), float("inf")),
            as_int(row.get("total_observation_count"), 999999),
        ),
    )


def latency_frontier_rows(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    candidates = [
        row
        for row in rows
        if row.get("status") == "STRICT_PASS"
        and not bool(row.get("runtime_truth_used", False))
        and as_float(row.get("target_max_abs_error_corrected_max_m"), float("inf")) <= 10.0
        and as_float(row.get("range_rmse_corrected_mean_m"), float("inf"))
        < as_float(row.get("range_rmse_raw_mean_m"), -float("inf"))
        and as_float(row.get("position_rmse_corrected_mean_m"), float("inf"))
        < as_float(row.get("position_rmse_raw_mean_m"), -float("inf"))
    ]
    best_by_scenario_obs: dict[tuple[str, int], Mapping[str, object]] = {}
    for row in candidates:
        key = (str(row.get("scenario", row.get("scenario_dir", ""))), as_int(row.get("total_observation_count")))
        previous = best_by_scenario_obs.get(key)
        if previous is None or (
            as_float(row.get("target_max_abs_error_corrected_max_m"), float("inf")),
            as_float(row.get("range_rmse_corrected_mean_m"), float("inf")),
        ) < (
            as_float(previous.get("target_max_abs_error_corrected_max_m"), float("inf")),
            as_float(previous.get("range_rmse_corrected_mean_m"), float("inf")),
        ):
            best_by_scenario_obs[key] = row
    return sorted(
        (dict(row) for row in best_by_scenario_obs.values()),
        key=lambda row: (
            str(row.get("scenario", row.get("scenario_dir", ""))),
            as_int(row.get("total_observation_count"), 999999),
            as_float(row.get("target_max_abs_error_corrected_max_m"), float("inf")),
        ),
    )


def rank_key(row: Mapping[str, object]) -> tuple[int, float, float, float]:
    trial_count = int(float(row.get("trial_count", 0) or 0))
    pass_count = int(float(row.get("pass_count", 0) or 0))
    return (
        0 if row.get("status") == "STRICT_PASS" else 1,
        -(pass_count / trial_count if trial_count else 0.0),
        float(row.get("target_max_abs_error_corrected_max_m", "inf")),
        float(row.get("range_rmse_corrected_mean_m", "inf")),
    )


def write_summary_txt(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    strict_pass_count = sum(1 for row in rows if row.get("status") == "STRICT_PASS")
    runtime_truth_used = any(str(row.get("runtime_truth_used", "")).lower() == "true" for row in rows)
    lines = [
        "Multi-burst observation fusion followup",
        "",
        f"strict_pass_count: {strict_pass_count}/{len(rows)}",
        f"runtime_truth_used: {runtime_truth_used}",
        (
            "recommended_runtime_structure: multi-burst pixel/disparity fusion before the StereoDistance runtime "
            "profile; prefer b5x5 trimmed_mean/median variants when common-mode image error is plausible."
        ),
        "",
        "Rows:",
    ]
    for row in sorted(rows, key=rank_key):
        lines.append(
            (
                "- {scenario_dir}: scenario={scenario}, status={status}, pass={pass_count}/{trial_count}, "
                "obs={total_obs}, common={common_fraction}, range_mean={range_mean:.6f}m, "
                "pos_mean={pos_mean:.6f}m, target_max={target_max:.6f}m"
            ).format(
                scenario_dir=row.get("scenario_dir", ""),
                scenario=row.get("scenario", ""),
                status=row.get("status", ""),
                pass_count=int(float(row.get("pass_count", 0) or 0)),
                trial_count=int(float(row.get("trial_count", 0) or 0)),
                total_obs=row.get("total_observation_count", ""),
                common_fraction=row.get("common_fraction", ""),
                range_mean=float(row.get("range_rmse_corrected_mean_m", "nan")),
                pos_mean=float(row.get("position_rmse_corrected_mean_m", "nan")),
                target_max=float(row.get("target_max_abs_error_corrected_max_m", "nan")),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_frontier_txt(
    path: Path,
    *,
    frontier_rows: Sequence[Mapping[str, object]],
    aggregate_rows: Sequence[Mapping[str, object]],
) -> None:
    runtime_truth_used = any(bool(row.get("runtime_truth_used", False)) for row in frontier_rows)
    best_aggregate = aggregate_rows[0] if aggregate_rows else {}
    lines = [
        "Multi-burst observation fusion latency frontier",
        "",
        f"frontier_row_count: {len(frontier_rows)}",
        f"aggregate_method_count: {len(aggregate_rows)}",
        f"runtime_truth_used: {runtime_truth_used}",
        (
            "selection_rule: prefer all-scenario STRICT_PASS methods by worst 1852m-window target error, "
            "then mean validation range RMSE, then lower observation count."
        ),
        (
            "runtime_contract: input image left_u_px/left_v_px/measured_disparity_px grouped by burst; "
            "output stereo-center RDF(right,down,forward); public topic remains FRD+range."
        ),
        "",
    ]
    if best_aggregate:
        lines.extend(
            [
                "Best aggregate method:",
                (
                    "- {method}: scenarios={strict}/{count}, obs={obs}, target_worst={target:.6f}m, "
                    "range_mean={range_mean:.6f}m"
                ).format(
                    method=best_aggregate.get("method", ""),
                    strict=best_aggregate.get("strict_pass_scenario_count", ""),
                    count=best_aggregate.get("scenario_count", ""),
                    obs=best_aggregate.get("total_observation_count", ""),
                    target=as_float(best_aggregate.get("target_max_abs_error_worst_m"), float("nan")),
                    range_mean=as_float(
                        best_aggregate.get("range_rmse_corrected_mean_across_scenarios_m"),
                        float("nan"),
                    ),
                ),
                "",
            ]
        )
    lines.append("Latency frontier rows:")
    for row in frontier_rows:
        lines.append(
            (
                "- {scenario}: obs={obs}, method={method}, status={status}, "
                "target_max={target:.6f}m, range_mean={range_mean:.6f}m"
            ).format(
                scenario=row.get("scenario", row.get("scenario_dir", "")),
                obs=row.get("total_observation_count", ""),
                method=row.get("method", ""),
                status=row.get("status", ""),
                target=as_float(row.get("target_max_abs_error_corrected_max_m"), float("nan")),
                range_mean=as_float(row.get("range_rmse_corrected_mean_m"), float("nan")),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize multi-burst observation fusion result directories.")
    parser.add_argument("--result-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows = [row_from_result_dir(path) for path in discover_result_dirs(args.result_root)]
    method_rows = [row for path in discover_result_dirs(args.result_root) for row in method_rows_from_result_dir(path)]
    frontier = latency_frontier_rows(method_rows)
    aggregates = aggregate_method_rows(method_rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "multiburst_observation_all_scenarios_summary.csv", rows)
    write_summary_txt(args.output_dir / "multiburst_observation_all_scenarios_summary.txt", rows)
    write_csv(args.output_dir / "multiburst_observation_latency_frontier.csv", frontier)
    write_csv(args.output_dir / "multiburst_observation_method_aggregate.csv", aggregates)
    write_frontier_txt(
        args.output_dir / "multiburst_observation_latency_frontier.txt",
        frontier_rows=frontier,
        aggregate_rows=aggregates,
    )
    print(f"wrote {len(rows)} rows to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
