#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Mapping


SUCCESS_STATUSES = {
    "PASS",
    "STRICT_PASS",
    "STABLE_BASELINE",
}
FAIL_STATUSES = {
    "FAIL",
    "CONTRACT_INVALID_FOR_CURRENT_GATE",
    "REJECTED",
}
PARTIAL_STATUSES = {
    "PARTIAL_PASS",
    "CONDITIONAL_PASS",
    "NEAR_PASS",
    "FAMILY_LOCAL_PASS",
}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def as_int(value: object, default: int = 0) -> int:
    try:
        if value in ("", None):
            return default
        return int(float(str(value)))
    except (TypeError, ValueError):
        return default


def as_float(value: object, default: float = 0.0) -> float:
    try:
        if value in ("", None):
            return default
        return float(str(value))
    except (TypeError, ValueError):
        return default


def status_bucket(status: object) -> str:
    normalized = str(status or "").strip().upper()
    if normalized in SUCCESS_STATUSES:
        return "success"
    if (
        normalized in FAIL_STATUSES
        or "FAIL" in normalized
        or "REJECT" in normalized
        or "INVALID" in normalized
        or normalized == "BLOCKED"
    ):
        return "failure"
    if normalized in PARTIAL_STATUSES or "PARTIAL" in normalized or "CONDITIONAL" in normalized or "NEAR" in normalized:
        return "partial"
    return "other"


def add_counts(stats: dict[str, object], *, status: str, pass_count: int, total_count: int) -> None:
    bucket = status_bucket(status)
    stats["row_count"] = as_int(stats.get("row_count")) + 1
    stats[f"{bucket}_row_count"] = as_int(stats.get(f"{bucket}_row_count")) + 1
    if total_count > 0:
        fail_count = max(0, total_count - pass_count)
        stats["trial_count_sum"] = as_int(stats.get("trial_count_sum")) + total_count
        stats["pass_count_sum"] = as_int(stats.get("pass_count_sum")) + pass_count
        stats["fail_count_sum"] = as_int(stats.get("fail_count_sum")) + fail_count


def finalize(rows: Iterable[dict[str, object]]) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    for row in rows:
        total = as_int(row.get("trial_count_sum"))
        passes = as_int(row.get("pass_count_sum"))
        row_count = as_int(row.get("row_count"))
        success_rows = as_int(row.get("success_row_count"))
        row["failure_row_count"] = as_int(row.get("failure_row_count"))
        row["success_row_count"] = as_int(row.get("success_row_count"))
        row["partial_row_count"] = as_int(row.get("partial_row_count"))
        row["other_row_count"] = as_int(row.get("other_row_count"))
        row["fail_count_sum"] = as_int(row.get("fail_count_sum"))
        row["pass_rate_by_trials"] = round(passes / total, 6) if total else ""
        row["pass_rate_by_trials_percent"] = round((passes / total) * 100.0, 2) if total else ""
        row["success_row_rate_percent"] = round((success_rows / row_count) * 100.0, 2) if row_count else ""
        output.append(row)
    return sorted(
        output,
        key=lambda item: (
            -as_int(item.get("success_row_count")),
            as_int(item.get("failure_row_count")),
            -as_int(item.get("pass_count_sum")),
            str(item.get("method", "")),
        ),
    )


def summarize_registry(registry_csv: Path) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    by_method: dict[str, dict[str, object]] = {}
    by_family: dict[str, dict[str, object]] = {}
    for row in read_csv(registry_csv):
        method = row.get("method_slug") or row.get("method") or "unknown"
        family = row.get("method_family") or "unknown"
        status = row.get("status", "")
        pass_count = as_int(row.get("pass_trial_count"))
        total_count = as_int(row.get("total_trial_count"))
        if total_count <= 0:
            pass_count = 1 if status_bucket(status) == "success" else 0
            total_count = 1 if status_bucket(status) in {"success", "failure", "partial"} else 0
        method_stats = by_method.setdefault(
            method,
            {
                "scope": "registry_exp_rows",
                "method": method,
                "method_family": family,
                "experiment_ids": [],
                "row_count": 0,
                "success_row_count": 0,
                "failure_row_count": 0,
                "partial_row_count": 0,
                "other_row_count": 0,
                "pass_count_sum": 0,
                "fail_count_sum": 0,
                "trial_count_sum": 0,
            },
        )
        method_stats["experiment_ids"].append(row.get("experiment_id", ""))
        add_counts(method_stats, status=status, pass_count=pass_count, total_count=total_count)
        family_stats = by_family.setdefault(
            family,
            {
                "scope": "registry_family_rows",
                "method_family": family,
                "row_count": 0,
                "success_row_count": 0,
                "failure_row_count": 0,
                "partial_row_count": 0,
                "other_row_count": 0,
                "pass_count_sum": 0,
                "fail_count_sum": 0,
                "trial_count_sum": 0,
            },
        )
        add_counts(family_stats, status=status, pass_count=pass_count, total_count=total_count)
    method_rows = finalize(by_method.values())
    for row in method_rows:
        row["experiment_ids"] = ";".join(item for item in row["experiment_ids"] if item)
    return method_rows, finalize(by_family.values())


def discover_method_summary_csvs(root: Path) -> list[Path]:
    return sorted(root.glob("EXP-*/*method_summary.csv")) + sorted(root.glob("EXP-*/*/*method_summary.csv"))


def summarize_candidate_csvs(root: Path) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    by_method: dict[str, dict[str, object]] = {}
    by_family: dict[str, dict[str, object]] = {}
    for path in discover_method_summary_csvs(root):
        family = path.name.replace("_method_summary.csv", "")
        experiment_id = next((part for part in path.parts if part.startswith("EXP-")), "")
        for row in read_csv(path):
            method = row.get("method") or row.get("method_slug") or "unknown"
            status = row.get("status", "")
            pass_count = as_int(row.get("pass_count"))
            total_count = as_int(row.get("trial_count"))
            stats = by_method.setdefault(
                method,
                {
                    "scope": "candidate_method_summary_rows",
                    "method": method,
                    "method_family": family,
                    "experiment_ids": [],
                    "scenario_count": 0,
                    "row_count": 0,
                    "success_row_count": 0,
                    "failure_row_count": 0,
                    "partial_row_count": 0,
                    "other_row_count": 0,
                    "pass_count_sum": 0,
                    "fail_count_sum": 0,
                    "trial_count_sum": 0,
                    "best_range_rmse_m": "",
                    "best_target_max_abs_error_m": "",
                },
            )
            stats["experiment_ids"].append(experiment_id)
            if row.get("scenario"):
                stats["scenario_count"] = as_int(stats.get("scenario_count")) + 1
            add_counts(stats, status=status, pass_count=pass_count, total_count=total_count)
            range_value = as_float(
                row.get("range_rmse_residual_mean_m")
                or row.get("range_rmse_corrected_mean_m")
                or row.get("range_rmse_corrected_max_m"),
                default=float("inf"),
            )
            target_value = as_float(
                row.get("target_max_abs_error_residual_max_m")
                or row.get("target_max_abs_error_corrected_max_m"),
                default=float("inf"),
            )
            if range_value != float("inf"):
                current = as_float(stats.get("best_range_rmse_m"), default=float("inf"))
                stats["best_range_rmse_m"] = min(current, range_value)
            if target_value != float("inf"):
                current = as_float(stats.get("best_target_max_abs_error_m"), default=float("inf"))
                stats["best_target_max_abs_error_m"] = min(current, target_value)
            family_stats = by_family.setdefault(
                family,
                {
                    "scope": "candidate_family_rows",
                    "method_family": family,
                    "row_count": 0,
                    "success_row_count": 0,
                    "failure_row_count": 0,
                    "partial_row_count": 0,
                    "other_row_count": 0,
                    "pass_count_sum": 0,
                    "fail_count_sum": 0,
                    "trial_count_sum": 0,
                },
            )
            add_counts(family_stats, status=status, pass_count=pass_count, total_count=total_count)
    method_rows = finalize(by_method.values())
    for row in method_rows:
        row["experiment_ids"] = ";".join(sorted(set(item for item in row["experiment_ids"] if item)))
        for key in ("best_range_rmse_m", "best_target_max_abs_error_m"):
            if row.get(key) == float("inf"):
                row[key] = ""
    return method_rows, finalize(by_family.values())


def write_markdown(
    path: Path,
    *,
    registry_method_rows: list[Mapping[str, object]],
    registry_family_rows: list[Mapping[str, object]],
    candidate_method_rows: list[Mapping[str, object]],
    candidate_family_rows: list[Mapping[str, object]],
) -> None:
    def table(rows: list[Mapping[str, object]], fields: list[str], limit: int = 25) -> list[str]:
        lines = ["| " + " | ".join(fields) + " |", "| " + " | ".join("---" for _ in fields) + " |"]
        for row in rows[:limit]:
            lines.append("| " + " | ".join(str(row.get(field, "")) for field in fields) + " |")
        return lines

    lines = [
        "# Experiment Method Statistics",
        "",
        "Generated from `experiment_registry.csv` plus machine-readable `*method_summary.csv` files under `Simulation/reports/experiments`.",
        "",
        "Status buckets: `PASS`, `STRICT_PASS`, and `STABLE_BASELINE` count as success; statuses containing `FAIL`, `REJECT`, or `INVALID`, plus `BLOCKED`, count as failure; partial, conditional, near-pass, and family-local statuses are counted separately.",
        "",
        "Coordinate contract: these statistics do not transform coordinates. They preserve the source experiment contracts: image `u/v/disparity_px`, internal stereo-center RDF `right/down/forward`, and public StereoDistance FRD+range `forward/right/down/range` where applicable.",
        "",
        "## Registry Method Rows",
        "",
        *table(
            registry_method_rows,
            [
                "method",
                "method_family",
                "success_row_count",
                "failure_row_count",
                "partial_row_count",
                "pass_count_sum",
                "fail_count_sum",
                "pass_rate_by_trials_percent",
                "success_row_rate_percent",
            ],
        ),
        "",
        "## Registry Family Rows",
        "",
        *table(
            registry_family_rows,
            [
                "method_family",
                "success_row_count",
                "failure_row_count",
                "partial_row_count",
                "pass_count_sum",
                "fail_count_sum",
                "pass_rate_by_trials_percent",
                "success_row_rate_percent",
            ],
        ),
        "",
        "## Candidate Method Rows",
        "",
        *table(
            candidate_method_rows,
            [
                "method",
                "method_family",
                "success_row_count",
                "failure_row_count",
                "pass_count_sum",
                "fail_count_sum",
                "pass_rate_by_trials_percent",
                "success_row_rate_percent",
                "best_range_rmse_m",
                "best_target_max_abs_error_m",
            ],
        ),
        "",
        "## Candidate Family Rows",
        "",
        *table(
            candidate_family_rows,
            [
                "method_family",
                "success_row_count",
                "failure_row_count",
                "pass_count_sum",
                "fail_count_sum",
                "pass_rate_by_trials_percent",
                "success_row_rate_percent",
            ],
        ),
        "",
        "Full CSV outputs are adjacent to this Markdown report.",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize EXP success/failure counts by method.")
    parser.add_argument("--experiments-dir", type=Path, default=Path("Simulation/reports/experiments"))
    parser.add_argument("--output-dir", type=Path, default=Path("Simulation/reports/experiments/method_statistics"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    registry = args.experiments_dir / "experiment_registry.csv"
    registry_method_rows, registry_family_rows = summarize_registry(registry)
    candidate_method_rows, candidate_family_rows = summarize_candidate_csvs(args.experiments_dir)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "registry_method_statistics.csv", registry_method_rows)
    write_csv(args.output_dir / "registry_family_statistics.csv", registry_family_rows)
    write_csv(args.output_dir / "candidate_method_statistics.csv", candidate_method_rows)
    write_csv(args.output_dir / "candidate_family_statistics.csv", candidate_family_rows)
    write_markdown(
        args.output_dir / "method_statistics.md",
        registry_method_rows=registry_method_rows,
        registry_family_rows=registry_family_rows,
        candidate_method_rows=candidate_method_rows,
        candidate_family_rows=candidate_family_rows,
    )
    print(f"registry_methods={len(registry_method_rows)}")
    print(f"registry_families={len(registry_family_rows)}")
    print(f"candidate_methods={len(candidate_method_rows)}")
    print(f"candidate_families={len(candidate_family_rows)}")
    print(f"output_dir={args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
