#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Mapping, Sequence


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
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


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


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


def discover_result_dirs(root: Path) -> list[Path]:
    if not root.is_dir():
        return []
    return [
        child
        for child in sorted(root.iterdir())
        if child.is_dir() and (child / "multiburst_disparity_residual_metrics.json").is_file()
    ]


def best_row_from_result_dir(result_dir: Path) -> dict[str, object]:
    metrics = json.loads((result_dir / "multiburst_disparity_residual_metrics.json").read_text(encoding="utf-8"))
    best = metrics.get("best_method", {})
    if not isinstance(best, dict):
        best = {}
    row = dict(best)
    row["scenario_dir"] = result_dir.name
    row["experiment_id"] = metrics.get("experiment_id", "")
    row["scenario"] = metrics.get("scenario", result_dir.name)
    row["source_csv"] = str(result_dir / "multiburst_disparity_residual_method_summary.csv")
    return row


def method_rows_from_result_dir(result_dir: Path) -> list[dict[str, object]]:
    summary_csv = result_dir / "multiburst_disparity_residual_method_summary.csv"
    if not summary_csv.is_file():
        return []
    metrics = json.loads((result_dir / "multiburst_disparity_residual_metrics.json").read_text(encoding="utf-8"))
    rows: list[dict[str, object]] = []
    for row in read_csv_rows(summary_csv):
        out: dict[str, object] = dict(row)
        out["scenario_dir"] = result_dir.name
        out["scenario"] = metrics.get("scenario", result_dir.name)
        out["source_csv"] = str(summary_csv)
        rows.append(out)
    return rows


def aggregate_method_rows(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    grouped: dict[str, list[Mapping[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row.get("method", "")), []).append(row)

    output: list[dict[str, object]] = []
    for method, group in grouped.items():
        strict_rows = [row for row in group if row.get("status") == "STRICT_PASS"]
        first = group[0]
        output.append(
            {
                "method": method,
                "scenario_count": len({str(row.get("scenario", row.get("scenario_dir", ""))) for row in group}),
                "strict_pass_scenario_count": len(strict_rows),
                "all_scenarios_strict_pass": len(strict_rows) == len(group),
                "total_observation_count": first.get("total_observation_count", ""),
                "burst_count": first.get("burst_count", ""),
                "observations_per_burst": first.get("observations_per_burst", ""),
                "inner_mode": first.get("inner_mode", ""),
                "outer_mode": first.get("outer_mode", ""),
                "trim_fraction": first.get("trim_fraction", ""),
                "common_fraction": first.get("common_fraction", ""),
                "feature_space": first.get("feature_space", ""),
                "correction_space": first.get("correction_space", ""),
                "degree": first.get("degree", ""),
                "ridge_lambda": first.get("ridge_lambda", ""),
                "correction_clip": first.get("correction_clip", ""),
                "target_max_abs_error_observation_worst_m": max(
                    as_float(row.get("target_max_abs_error_observation_max_m"), float("inf")) for row in group
                ),
                "target_max_abs_error_residual_worst_m": max(
                    as_float(row.get("target_max_abs_error_residual_max_m"), float("inf")) for row in group
                ),
                "range_rmse_observation_mean_across_scenarios_m": sum(
                    as_float(row.get("range_rmse_observation_mean_m"), float("inf")) for row in group
                )
                / len(group),
                "range_rmse_residual_mean_across_scenarios_m": sum(
                    as_float(row.get("range_rmse_residual_mean_m"), float("inf")) for row in group
                )
                / len(group),
                "position_rmse_observation_mean_across_scenarios_m": sum(
                    as_float(row.get("position_rmse_observation_mean_m"), float("inf")) for row in group
                )
                / len(group),
                "position_rmse_residual_mean_across_scenarios_m": sum(
                    as_float(row.get("position_rmse_residual_mean_m"), float("inf")) for row in group
                )
                / len(group),
                "runtime_truth_used": any(str(row.get("runtime_truth_used", "")).lower() == "true" for row in group),
                "runtime_input_contract": first.get("runtime_input_contract", ""),
                "rdf_axis_contract": first.get("rdf_axis_contract", ""),
                "public_topic_contract": first.get("public_topic_contract", ""),
            }
        )
    max_scenario_count = max((as_int(row.get("scenario_count")) for row in output), default=0)
    return sorted(
        output,
        key=lambda row: (
            0 if as_int(row.get("scenario_count")) == max_scenario_count else 1,
            0 if row.get("all_scenarios_strict_pass") is True else 1,
            -as_int(row.get("scenario_count")),
            as_float(row.get("target_max_abs_error_residual_worst_m"), float("inf")),
            as_float(row.get("range_rmse_residual_mean_across_scenarios_m"), float("inf")),
            as_int(row.get("total_observation_count"), 999999),
        ),
    )


def write_summary_txt(
    path: Path,
    *,
    best_rows: Sequence[Mapping[str, object]],
    aggregate_rows: Sequence[Mapping[str, object]],
) -> None:
    best = aggregate_rows[0] if aggregate_rows else {}
    lines = [
        "Multi-burst disparity residual summary",
        "",
        f"scenario_count: {len(best_rows)}",
        f"aggregate_method_count: {len(aggregate_rows)}",
        f"runtime_truth_used: {any(bool(row.get('runtime_truth_used')) for row in aggregate_rows)}",
        "",
        "Best aggregate method:",
        (
            "- {method}: scenarios={strict}/{count}, obs={obs}, target_worst={target:.6f}m, "
            "range_mean={range_mean:.6f}m"
        ).format(
            method=best.get("method", ""),
            strict=best.get("strict_pass_scenario_count", 0),
            count=best.get("scenario_count", 0),
            obs=best.get("total_observation_count", ""),
            target=as_float(best.get("target_max_abs_error_residual_worst_m")),
            range_mean=as_float(best.get("range_rmse_residual_mean_across_scenarios_m")),
        ),
        "",
        "Scenario best rows:",
    ]
    for row in best_rows:
        lines.append(
            (
                "- {scenario}: status={status}, target_obs={target_obs:.6f}m, "
                "target_res={target_res:.6f}m, range_obs={range_obs:.6f}m, "
                "range_res={range_res:.6f}m"
            ).format(
                scenario=row.get("scenario", ""),
                status=row.get("status", ""),
                target_obs=as_float(row.get("target_max_abs_error_observation_max_m")),
                target_res=as_float(row.get("target_max_abs_error_residual_max_m")),
                range_obs=as_float(row.get("range_rmse_observation_mean_m")),
                range_res=as_float(row.get("range_rmse_residual_mean_m")),
            )
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize multi-burst disparity residual result directories.")
    parser.add_argument("--result-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result_dirs = discover_result_dirs(args.result_root)
    best_rows = [best_row_from_result_dir(result_dir) for result_dir in result_dirs]
    method_rows: list[dict[str, object]] = []
    for result_dir in result_dirs:
        method_rows.extend(method_rows_from_result_dir(result_dir))
    aggregate_rows = aggregate_method_rows(method_rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "multiburst_disparity_residual_scenario_best.csv", best_rows)
    write_csv(args.output_dir / "multiburst_disparity_residual_method_aggregate.csv", aggregate_rows)
    payload = {
        "status": "PASS" if best_rows and aggregate_rows else "FAIL",
        "scenario_count": len(best_rows),
        "aggregate_method_count": len(aggregate_rows),
        "best_method": aggregate_rows[0] if aggregate_rows else {},
        "runtime_truth_used": any(bool(row.get("runtime_truth_used")) for row in aggregate_rows),
    }
    write_json(args.output_dir / "multiburst_disparity_residual_summary.json", payload)
    write_summary_txt(
        args.output_dir / "multiburst_disparity_residual_summary.txt",
        best_rows=best_rows,
        aggregate_rows=aggregate_rows,
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
