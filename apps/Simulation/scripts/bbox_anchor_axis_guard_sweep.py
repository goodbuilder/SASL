#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Sequence


RDF_AXES = ("right", "down", "forward")
AXES = ("right", "down", "forward", "position")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sweep runtime-safe axis/range guards over bbox-anchor validation "
            "comparisons. Right/Down/Forward can use independent gains, and "
            "Right/Down can fall back to raw below raw Forward range thresholds."
        )
    )
    parser.add_argument("report_dirs", nargs="+", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--validation-trials", default="2,3,4,5,6")
    parser.add_argument("--right-gains", default="0,0.01,0.025,0.05,0.075,0.1,0.15,0.2,0.25,0.5,1.0")
    parser.add_argument("--down-gains", default="0,0.01,0.025,0.05,0.075,0.1,0.15,0.2,0.25,0.5,1.0")
    parser.add_argument("--forward-gains", default="1.0")
    parser.add_argument("--right-min-ranges-m", default="none,500,800,1100,1400")
    parser.add_argument("--down-min-ranges-m", default="none,500,800,1100,1400")
    parser.add_argument("--forward-min-ranges-m", default="none,500,800,1100,1400")
    parser.add_argument("--range-edges-m", default="0,500,800,1100,1400,2000")
    parser.add_argument("--diagnostic-right-gain", type=float, default=1.0)
    parser.add_argument("--diagnostic-down-gain", type=float, default=1.0)
    parser.add_argument("--diagnostic-forward-gain", type=float, default=1.0)
    parser.add_argument("--diagnostic-right-min-range-m", type=float, default=1100.0)
    parser.add_argument("--diagnostic-down-min-range-m", type=float, default=800.0)
    parser.add_argument("--diagnostic-forward-min-range-m", type=float, default=500.0)
    return parser.parse_args()


def parse_int_list(text: str) -> list[int]:
    return [int(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_float_list(text: str) -> list[float]:
    return [float(part.strip()) for part in str(text).split(",") if part.strip()]


def parse_optional_float_list(text: str) -> list[float | None]:
    values: list[float | None] = []
    for part in str(text).split(","):
        item = part.strip().lower()
        if not item:
            continue
        values.append(None if item in {"none", "raw", "off"} else float(item))
    return values


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else float("nan")


def position_error(row: dict[str, float], prefix: str) -> float:
    return math.sqrt(sum(float(row[f"{prefix}_{axis}"]) ** 2 for axis in RDF_AXES))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def range_bin(raw_forward_m: float, edges: Sequence[float]) -> int:
    for index in range(max(0, len(edges) - 1)):
        if float(edges[index]) <= raw_forward_m < float(edges[index + 1]):
            return index
    return max(0, len(edges) - 2)


def load_samples(report_dirs: Sequence[Path], trials: Sequence[int]) -> list[dict[str, object]]:
    samples: list[dict[str, object]] = []
    for report_dir in report_dirs:
        for trial in trials:
            path = report_dir / f"validation_trial_{trial:02d}_anchor_comparison.csv"
            if not path.exists():
                continue
            with path.open(newline="", encoding="utf-8") as handle:
                for index, row in enumerate(csv.DictReader(handle)):
                    parsed = {
                        "report_dir": report_dir.name,
                        "trial": trial,
                        "row_index": index,
                        "raw_forward_m": float(row["raw_forward_m"]),
                    }
                    for axis in RDF_AXES:
                        parsed[f"raw_error_{axis}"] = float(row[f"raw_error_{axis}_m"])
                        parsed[f"anchor_error_{axis}"] = float(row[f"anchor_error_{axis}_m"])
                    samples.append(parsed)
    return samples


def guarded_axis_error(
    *,
    raw_error: float,
    anchor_error: float,
    raw_forward_m: float,
    gain: float,
    min_range_m: float | None,
) -> tuple[float, str]:
    if min_range_m is not None and raw_forward_m < min_range_m:
        return raw_error, "range_guarded_noop"
    if gain == 0.0:
        return raw_error, "gain_noop"
    return raw_error + gain * (anchor_error - raw_error), "none"


def corrected_errors(
    sample: dict[str, object],
    *,
    right_gain: float,
    down_gain: float,
    forward_gain: float,
    right_min_range_m: float | None,
    down_min_range_m: float | None,
    forward_min_range_m: float | None,
) -> dict[str, object]:
    raw_forward = float(sample["raw_forward_m"])
    right, right_fallback = guarded_axis_error(
        raw_error=float(sample["raw_error_right"]),
        anchor_error=float(sample["anchor_error_right"]),
        raw_forward_m=raw_forward,
        gain=right_gain,
        min_range_m=right_min_range_m,
    )
    down, down_fallback = guarded_axis_error(
        raw_error=float(sample["raw_error_down"]),
        anchor_error=float(sample["anchor_error_down"]),
        raw_forward_m=raw_forward,
        gain=down_gain,
        min_range_m=down_min_range_m,
    )
    forward, forward_fallback = guarded_axis_error(
        raw_error=float(sample["raw_error_forward"]),
        anchor_error=float(sample["anchor_error_forward"]),
        raw_forward_m=raw_forward,
        gain=forward_gain,
        min_range_m=forward_min_range_m,
    )
    return {
        "corrected_error_right": right,
        "corrected_error_down": down,
        "corrected_error_forward": forward,
        "right_fallback": right_fallback,
        "down_fallback": down_fallback,
        "forward_fallback": forward_fallback,
    }


def summarize_rows(rows: Sequence[dict[str, object]]) -> dict[str, float | str]:
    result: dict[str, float | str] = {}
    for axis in RDF_AXES:
        raw = [float(row[f"raw_error_{axis}"]) for row in rows]
        corrected = [float(row[f"corrected_error_{axis}"]) for row in rows]
        result[f"{axis}_rmse_improvement_m"] = rmse(raw) - rmse(corrected)
    raw_position = [
        math.sqrt(sum(float(row[f"raw_error_{axis}"]) ** 2 for axis in RDF_AXES))
        for row in rows
    ]
    corrected_position = [
        math.sqrt(sum(float(row[f"corrected_error_{axis}"]) ** 2 for axis in RDF_AXES))
        for row in rows
    ]
    result["position_rmse_improvement_m"] = rmse(raw_position) - rmse(corrected_position)
    result["pass_rdf_nonregression"] = (
        "yes" if all(float(result[f"{axis}_rmse_improvement_m"]) >= 0.0 for axis in RDF_AXES) else "no"
    )
    return result


def apply_candidate(
    samples: Sequence[dict[str, object]],
    *,
    right_gain: float,
    down_gain: float,
    forward_gain: float,
    right_min_range_m: float | None,
    down_min_range_m: float | None,
    forward_min_range_m: float | None,
    range_edges: Sequence[float],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for sample in samples:
        corrected = corrected_errors(
            sample,
            right_gain=right_gain,
            down_gain=down_gain,
            forward_gain=forward_gain,
            right_min_range_m=right_min_range_m,
            down_min_range_m=down_min_range_m,
            forward_min_range_m=forward_min_range_m,
        )
        row = dict(sample)
        row.update(corrected)
        row["raw_error_position"] = math.sqrt(sum(float(row[f"raw_error_{axis}"]) ** 2 for axis in RDF_AXES))
        row["corrected_error_position"] = math.sqrt(
            sum(float(row[f"corrected_error_{axis}"]) ** 2 for axis in RDF_AXES)
        )
        row["range_bin"] = range_bin(float(row["raw_forward_m"]), range_edges)
        rows.append(row)
    return rows


def label_optional(value: float | None) -> str:
    return "" if value is None else f"{value:g}"


def candidate_id(
    right_gain: float,
    down_gain: float,
    forward_gain: float,
    right_min_range_m: float | None,
    down_min_range_m: float | None,
    forward_min_range_m: float | None,
) -> str:
    return (
        f"rg={right_gain:g};dg={down_gain:g};fg={forward_gain:g};"
        f"rmin={label_optional(right_min_range_m) or 'none'};"
        f"dmin={label_optional(down_min_range_m) or 'none'};"
        f"fmin={label_optional(forward_min_range_m) or 'none'}"
    )


def per_trial_summary(
    rows: Sequence[dict[str, object]],
    *,
    right_gain: float,
    down_gain: float,
    forward_gain: float,
    right_min_range_m: float | None,
    down_min_range_m: float | None,
    forward_min_range_m: float | None,
) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    keys = sorted({(str(row["report_dir"]), int(row["trial"])) for row in rows})
    for report_dir, trial in keys:
        part = [row for row in rows if str(row["report_dir"]) == report_dir and int(row["trial"]) == trial]
        summary = summarize_rows(part)
        output.append(
            {
                "candidate_id": candidate_id(
                    right_gain,
                    down_gain,
                    forward_gain,
                    right_min_range_m,
                    down_min_range_m,
                    forward_min_range_m,
                ),
                "report_dir": report_dir,
                "trial": trial,
                "sample_count": len(part),
                "right_gain": right_gain,
                "down_gain": down_gain,
                "forward_gain": forward_gain,
                "right_min_range_m": label_optional(right_min_range_m),
                "down_min_range_m": label_optional(down_min_range_m),
                "forward_min_range_m": label_optional(forward_min_range_m),
                **{
                    f"{axis}_rmse_improvement_m": f"{float(summary[f'{axis}_rmse_improvement_m']):.9f}"
                    for axis in AXES
                },
                "pass_rdf_nonregression": summary["pass_rdf_nonregression"],
                "right_range_guarded_noop_count": sum(
                    row["right_fallback"] == "range_guarded_noop" for row in part
                ),
                "down_range_guarded_noop_count": sum(row["down_fallback"] == "range_guarded_noop" for row in part),
                "forward_range_guarded_noop_count": sum(
                    row["forward_fallback"] == "range_guarded_noop" for row in part
                ),
            }
        )
    return output


def candidate_summary(per_trial: Sequence[dict[str, object]]) -> dict[str, object]:
    report_count = len({str(row["report_dir"]) for row in per_trial})
    result: dict[str, object] = {
        "candidate_id": per_trial[0]["candidate_id"],
        "right_gain": per_trial[0]["right_gain"],
        "down_gain": per_trial[0]["down_gain"],
        "forward_gain": per_trial[0]["forward_gain"],
        "right_min_range_m": per_trial[0]["right_min_range_m"],
        "down_min_range_m": per_trial[0]["down_min_range_m"],
        "forward_min_range_m": per_trial[0]["forward_min_range_m"],
        "report_count": report_count,
        "trial_count": len(per_trial),
        "pass_trial_count": sum(row["pass_rdf_nonregression"] == "yes" for row in per_trial),
        "pass_all_nonregression": "yes"
        if all(row["pass_rdf_nonregression"] == "yes" for row in per_trial)
        else "no",
        "right_range_guarded_noop_count": sum(int(row["right_range_guarded_noop_count"]) for row in per_trial),
        "down_range_guarded_noop_count": sum(int(row["down_range_guarded_noop_count"]) for row in per_trial),
        "forward_range_guarded_noop_count": sum(
            int(row["forward_range_guarded_noop_count"]) for row in per_trial
        ),
    }
    for axis in AXES:
        values = [float(row[f"{axis}_rmse_improvement_m"]) for row in per_trial]
        result[f"min_{axis}_rmse_improvement_m"] = f"{min(values):.9f}"
        result[f"mean_{axis}_rmse_improvement_m"] = f"{sum(values) / len(values):.9f}"
        result[f"max_{axis}_rmse_improvement_m"] = f"{max(values):.9f}"
    return result


def group_summary(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    for group_name, key_func in (
        ("all", lambda row: ("all",)),
        ("report", lambda row: (str(row["report_dir"]),)),
        ("trial", lambda row: (str(row["trial"]),)),
        ("range_bin", lambda row: (str(row["range_bin"]),)),
    ):
        keys = sorted({key_func(row) for row in rows})
        for key in keys:
            part = [row for row in rows if key_func(row) == key]
            summary = summarize_rows(part)
            output.append(
                {
                    "group": group_name,
                    "key": ":".join(key),
                    "sample_count": len(part),
                    **{
                        f"{axis}_rmse_improvement_m": f"{float(summary[f'{axis}_rmse_improvement_m']):.9f}"
                        for axis in AXES
                    },
                    "pass_rdf_nonregression": summary["pass_rdf_nonregression"],
                }
            )
    return output


def main() -> int:
    args = parse_args()
    trials = parse_int_list(args.validation_trials)
    range_edges = parse_float_list(args.range_edges_m)
    samples = load_samples(args.report_dirs, trials)
    candidate_rows: list[dict[str, object]] = []
    trial_rows: list[dict[str, object]] = []
    diagnostic_rows: list[dict[str, object]] = []
    diagnostic_group_rows: list[dict[str, object]] = []

    diagnostic_key = candidate_id(
        args.diagnostic_right_gain,
        args.diagnostic_down_gain,
        args.diagnostic_forward_gain,
        args.diagnostic_right_min_range_m,
        args.diagnostic_down_min_range_m,
        args.diagnostic_forward_min_range_m,
    )

    for right_gain in parse_float_list(args.right_gains):
        for down_gain in parse_float_list(args.down_gains):
            for forward_gain in parse_float_list(args.forward_gains):
                for right_min_range_m in parse_optional_float_list(args.right_min_ranges_m):
                    for down_min_range_m in parse_optional_float_list(args.down_min_ranges_m):
                        for forward_min_range_m in parse_optional_float_list(args.forward_min_ranges_m):
                            corrected = apply_candidate(
                                samples,
                                right_gain=right_gain,
                                down_gain=down_gain,
                                forward_gain=forward_gain,
                                right_min_range_m=right_min_range_m,
                                down_min_range_m=down_min_range_m,
                                forward_min_range_m=forward_min_range_m,
                                range_edges=range_edges,
                            )
                            per_trial = per_trial_summary(
                                corrected,
                                right_gain=right_gain,
                                down_gain=down_gain,
                                forward_gain=forward_gain,
                                right_min_range_m=right_min_range_m,
                                down_min_range_m=down_min_range_m,
                                forward_min_range_m=forward_min_range_m,
                            )
                            trial_rows.extend(per_trial)
                            summary = candidate_summary(per_trial)
                            candidate_rows.append(summary)
                            if summary["candidate_id"] == diagnostic_key:
                                diagnostic_rows = corrected
                                diagnostic_group_rows = group_summary(corrected)

    candidate_rows.sort(
        key=lambda row: (
            row["pass_all_nonregression"] == "yes",
            float(row["min_right_rmse_improvement_m"]),
            float(row["min_down_rmse_improvement_m"]),
            float(row["min_forward_rmse_improvement_m"]),
            float(row["mean_position_rmse_improvement_m"]),
        ),
        reverse=True,
    )

    output_dir = args.output_dir
    write_csv(output_dir / "axis_guard_candidate_summary.csv", candidate_rows)
    write_csv(output_dir / "axis_guard_trial_summary.csv", trial_rows)
    if diagnostic_rows:
        output_diagnostics: list[dict[str, object]] = []
        for row in diagnostic_rows:
            output_diagnostics.append(
                {
                    "report_dir": row["report_dir"],
                    "trial": row["trial"],
                    "row_index": row["row_index"],
                    "raw_forward_m": f"{float(row['raw_forward_m']):.9f}",
                    "range_bin": row["range_bin"],
                    "fallback": "none",
                    **{
                        f"raw_error_{axis}_m": f"{float(row[f'raw_error_{axis}']):.9f}"
                        for axis in RDF_AXES
                    },
                    **{
                        f"corrected_error_{axis}_m": f"{float(row[f'corrected_error_{axis}']):.9f}"
                        for axis in RDF_AXES
                    },
                    "raw_error_position_m": f"{float(row['raw_error_position']):.9f}",
                    "corrected_error_position_m": f"{float(row['corrected_error_position']):.9f}",
                    "right_fallback": row["right_fallback"],
                    "down_fallback": row["down_fallback"],
                    "forward_fallback": row["forward_fallback"],
                }
            )
        write_csv(output_dir / "candidate_sample_diagnostics.csv", output_diagnostics)
        write_csv(output_dir / "candidate_group_summary.csv", diagnostic_group_rows)
    write_json(
        output_dir / "metadata.json",
        {
            "input_reports": [str(path) for path in args.report_dirs],
            "sample_count": len(samples),
            "contract": "range guards use only raw Forward; guarded axes are raw/no-op fallbacks",
            "diagnostic_candidate": diagnostic_key,
        },
    )

    if candidate_rows:
        best = candidate_rows[0]
        print(
            "Best: "
            f"{best['candidate_id']} pass_all={best['pass_all_nonregression']} "
            f"min_rdf=({best['min_right_rmse_improvement_m']},"
            f"{best['min_down_rmse_improvement_m']},"
            f"{best['min_forward_rmse_improvement_m']})"
        )
        print(f"Candidate summary: {output_dir / 'axis_guard_candidate_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
