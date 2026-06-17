#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path
from typing import Iterable, Sequence


AXES = ("right", "down", "forward", "position")
RDF_AXES = ("right", "down", "forward")
SEED_RE = re.compile(r"seed\d+")
STRICT_EPSILON_M = 1.0e-9


SERIES_DEFINITIONS = (
    ("default", ""),
    ("size", "size_"),
    ("blended", "blended_"),
    ("filtered_vs_raw", "filtered_vs_raw_"),
    ("filtered_vs_anchor", "filtered_vs_anchor_"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Aggregate heterogeneous validation_gate_summary.csv reports into "
            "common RDF/Position RMSE-improvement rankings. Positive improvement "
            "means raw RMSE is larger than corrected RMSE."
        )
    )
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        help="Report directories, validation_gate_summary.csv files, or roots to scan recursively.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--min-reports", type=int, default=1)
    return parser.parse_args()


def validation_gate_paths(inputs: Sequence[Path]) -> list[Path]:
    roots = list(inputs) if inputs else [Path("reports")]
    paths: list[Path] = []
    for item in roots:
        if item.is_file() and item.name == "validation_gate_summary.csv":
            paths.append(item)
        elif item.is_dir() and (item / "validation_gate_summary.csv").exists():
            paths.append(item / "validation_gate_summary.csv")
        elif item.is_dir():
            paths.extend(sorted(item.glob("**/validation_gate_summary.csv")))
    return sorted(dict.fromkeys(paths))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def safe_float(value: object) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def report_name(path: Path) -> str:
    return path.parent.name


def method_key(path: Path, series: str) -> str:
    normalized = SEED_RE.sub("seed*", report_name(path))
    if series == "default":
        return normalized
    return f"{normalized}::{series}"


def series_available(row: dict[str, str], prefix: str) -> bool:
    return all(f"{prefix}{axis}_rmse_improvement_m" in row for axis in RDF_AXES)


def iter_trial_rows(path: Path) -> Iterable[dict[str, object]]:
    rows = read_csv(path)
    for row in rows:
        for series, prefix in SERIES_DEFINITIONS:
            if not series_available(row, prefix):
                continue
            values = {
                axis: safe_float(row.get(f"{prefix}{axis}_rmse_improvement_m"))
                for axis in AXES
            }
            if any(values[axis] is None for axis in RDF_AXES):
                continue
            pass_rdf = all(float(values[axis]) >= 0.0 for axis in RDF_AXES)
            pass_strict_rdf = (
                all(float(values[axis]) > STRICT_EPSILON_M for axis in RDF_AXES)
                and values["position"] is not None
                and float(values["position"]) > STRICT_EPSILON_M
            )
            output = {
                "method_key": method_key(path, series),
                "series": series,
                "report_dir": report_name(path),
                "source_summary": str(path),
                "trial": row.get("trial", ""),
                "sample_count": row.get("sample_count") or row.get("samples") or "",
                "evaluated_count": row.get("evaluated_count") or "",
                "right_rmse_improvement_m": f"{float(values['right']):.9f}",
                "down_rmse_improvement_m": f"{float(values['down']):.9f}",
                "forward_rmse_improvement_m": f"{float(values['forward']):.9f}",
                "position_rmse_improvement_m": (
                    "" if values["position"] is None else f"{float(values['position']):.9f}"
                ),
                "pass_rdf_nonregression": "yes" if pass_rdf else "no",
                "pass_strict_rdf_all_axis": "yes" if pass_strict_rdf else "no",
            }
            yield output


def mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def summarize(method_key_value: str, rows: Sequence[dict[str, object]], min_reports: int) -> dict[str, object]:
    reports = sorted({str(row["report_dir"]) for row in rows})
    axis_values: dict[str, list[float]] = {
        axis: [
            float(row[f"{axis}_rmse_improvement_m"])
            for row in rows
            if str(row.get(f"{axis}_rmse_improvement_m", "")) != ""
        ]
        for axis in AXES
    }
    pass_count = sum(row.get("pass_rdf_nonregression") == "yes" for row in rows)
    strict_pass_count = sum(row.get("pass_strict_rdf_all_axis") == "yes" for row in rows)
    report_count = len(reports)
    trial_count = len(rows)
    has_enough_reports = report_count >= min_reports
    pass_all = pass_count == trial_count and has_enough_reports
    strict_pass_all = strict_pass_count == trial_count and has_enough_reports
    result: dict[str, object] = {
        "method_key": method_key_value,
        "series": str(rows[0]["series"]) if rows else "",
        "report_count": report_count,
        "trial_count": trial_count,
        "pass_trial_count": pass_count,
        "pass_all_trials": "yes" if pass_count == trial_count else "no",
        "meets_min_reports": "yes" if has_enough_reports else "no",
        "pass_all_with_min_reports": "yes" if pass_all else "no",
        "strict_pass_trial_count": strict_pass_count,
        "strict_pass_all_trials": "yes" if strict_pass_count == trial_count else "no",
        "strict_pass_all_with_min_reports": "yes" if strict_pass_all else "no",
        "strict_epsilon_m": f"{STRICT_EPSILON_M:.12f}",
        "source_reports": ";".join(reports),
    }
    for axis in AXES:
        values = axis_values[axis]
        if values:
            result[f"min_{axis}_rmse_improvement_m"] = f"{min(values):.9f}"
            result[f"mean_{axis}_rmse_improvement_m"] = f"{mean(values):.9f}"
            result[f"max_{axis}_rmse_improvement_m"] = f"{max(values):.9f}"
        else:
            result[f"min_{axis}_rmse_improvement_m"] = ""
            result[f"mean_{axis}_rmse_improvement_m"] = ""
            result[f"max_{axis}_rmse_improvement_m"] = ""
    return result


def sort_summary_key(row: dict[str, object]) -> tuple[object, ...]:
    return (
        row.get("strict_pass_all_with_min_reports") == "yes",
        row.get("pass_all_with_min_reports") == "yes",
        int(row.get("strict_pass_trial_count", 0)),
        row.get("pass_all_trials") == "yes",
        int(row.get("report_count", 0)),
        float(row.get("min_right_rmse_improvement_m") or "-inf"),
        float(row.get("min_down_rmse_improvement_m") or "-inf"),
        float(row.get("min_forward_rmse_improvement_m") or "-inf"),
        float(row.get("mean_position_rmse_improvement_m") or "-inf"),
    )


def main() -> int:
    args = parse_args()
    paths = validation_gate_paths(args.inputs)
    trial_rows: list[dict[str, object]] = []
    skipped: list[dict[str, object]] = []
    for path in paths:
        rows = list(iter_trial_rows(path))
        if rows:
            trial_rows.extend(rows)
        else:
            skipped.append({"input": str(path), "reason": "no_supported_improvement_series"})

    by_method: dict[str, list[dict[str, object]]] = {}
    for row in trial_rows:
        by_method.setdefault(str(row["method_key"]), []).append(row)

    summary_rows = [
        summarize(method, rows, int(args.min_reports))
        for method, rows in by_method.items()
    ]
    summary_rows.sort(key=sort_summary_key, reverse=True)

    write_csv(args.output_dir / "validation_gate_trial_rows.csv", trial_rows)
    write_csv(args.output_dir / "validation_gate_method_summary.csv", summary_rows)
    write_csv(args.output_dir / "skipped_inputs.csv", skipped)

    print(f"Loaded validation summaries: {len(paths) - len(skipped)}")
    print(f"Skipped validation summaries: {len(skipped)}")
    print(f"Trial rows: {len(trial_rows)}")
    print(f"Method rows: {len(summary_rows)}")
    if summary_rows:
        best = summary_rows[0]
        print(
            "Best: "
            f"{best['method_key']} reports={best['report_count']} "
            f"pass_all={best['pass_all_trials']} "
            f"strict_pass={best['strict_pass_trial_count']}/{best['trial_count']} "
            f"min_rdf=({best['min_right_rmse_improvement_m']},"
            f"{best['min_down_rmse_improvement_m']},"
            f"{best['min_forward_rmse_improvement_m']})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
