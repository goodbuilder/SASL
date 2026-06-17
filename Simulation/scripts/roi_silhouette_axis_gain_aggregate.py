#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Sequence


AXES = ("right", "down", "forward", "position")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Aggregate ROI silhouette axis-gain candidate summaries across "
            "multiple validation reports. Missing candidate summaries are "
            "reported as skipped inputs, not validation evidence."
        )
    )
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def candidate_summary_path(path: Path) -> Path | None:
    if path.is_file() and path.name == "axis_gain_candidate_summary.csv":
        return path
    candidate = path / "axis_gain_candidate_summary.csv"
    if candidate.exists():
        return candidate
    return None


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def candidate_key(row: dict[str, str]) -> tuple[str, str, str, str]:
    return (
        row.get("mode", ""),
        row.get("right_gain", ""),
        row.get("down_gain", ""),
        row.get("forward_gain", ""),
    )


def safe_float(row: dict[str, str], name: str) -> float:
    try:
        return float(row.get(name, "nan"))
    except ValueError:
        return float("nan")


def main() -> int:
    args = parse_args()
    loaded: list[tuple[Path, list[dict[str, str]]]] = []
    skipped: list[dict[str, object]] = []
    for item in args.inputs:
        path = candidate_summary_path(item)
        if path is None:
            skipped.append({"input": str(item), "reason": "missing_axis_gain_candidate_summary"})
            continue
        rows = read_rows(path)
        if not rows:
            skipped.append({"input": str(item), "reason": "empty_axis_gain_candidate_summary"})
            continue
        loaded.append((path, rows))

    by_candidate: dict[tuple[str, str, str, str], list[tuple[Path, dict[str, str]]]] = {}
    for path, rows in loaded:
        for row in rows:
            by_candidate.setdefault(candidate_key(row), []).append((path, row))

    required_report_count = len(loaded)
    aggregate_rows: list[dict[str, object]] = []
    for (mode, right_gain, down_gain, forward_gain), entries in by_candidate.items():
        present_count = len(entries)
        all_present = present_count == required_report_count
        pass_all_count = sum(row.get("pass_all_nonregression") == "yes" for _path, row in entries)
        improvements = {
            axis: [safe_float(row, f"min_{axis}_rmse_improvement_m") for _path, row in entries]
            for axis in AXES
        }
        mean_improvements = {
            axis: [safe_float(row, f"mean_{axis}_rmse_improvement_m") for _path, row in entries]
            for axis in AXES
        }
        aggregate_rows.append(
            {
                "mode": mode,
                "right_gain": right_gain,
                "down_gain": down_gain,
                "forward_gain": forward_gain,
                "report_count": present_count,
                "required_report_count": required_report_count,
                "present_in_all_reports": "yes" if all_present else "no",
                "pass_all_report_count": pass_all_count,
                "pass_all_reports": "yes" if all_present and pass_all_count == required_report_count else "no",
                "min_right_rmse_improvement_m": f"{min(improvements['right']):.9f}",
                "min_down_rmse_improvement_m": f"{min(improvements['down']):.9f}",
                "min_forward_rmse_improvement_m": f"{min(improvements['forward']):.9f}",
                "min_position_rmse_improvement_m": f"{min(improvements['position']):.9f}",
                "mean_right_rmse_improvement_m": f"{sum(mean_improvements['right']) / present_count:.9f}",
                "mean_down_rmse_improvement_m": f"{sum(mean_improvements['down']) / present_count:.9f}",
                "mean_forward_rmse_improvement_m": f"{sum(mean_improvements['forward']) / present_count:.9f}",
                "mean_position_rmse_improvement_m": f"{sum(mean_improvements['position']) / present_count:.9f}",
                "source_summaries": ";".join(str(path) for path, _row in entries),
            }
        )

    aggregate_rows.sort(
        key=lambda row: (
            row["pass_all_reports"] == "yes",
            int(row["pass_all_report_count"]),
            float(row["min_down_rmse_improvement_m"]),
            float(row["min_forward_rmse_improvement_m"]),
            float(row["mean_position_rmse_improvement_m"]),
        ),
        reverse=True,
    )
    write_csv(args.output_dir / "axis_gain_multi_report_summary.csv", aggregate_rows)
    write_csv(args.output_dir / "skipped_inputs.csv", skipped)
    print(f"Loaded reports: {required_report_count}")
    print(f"Skipped inputs: {len(skipped)}")
    print(f"Summary: {args.output_dir / 'axis_gain_multi_report_summary.csv'}")
    if aggregate_rows:
        best = aggregate_rows[0]
        print(
            "Best: "
            f"mode={best['mode']} gains=({best['right_gain']},{best['down_gain']},{best['forward_gain']}) "
            f"pass_all_reports={best['pass_all_reports']} report_count={best['report_count']}"
        )
        return 0 if best["pass_all_reports"] == "yes" else 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
