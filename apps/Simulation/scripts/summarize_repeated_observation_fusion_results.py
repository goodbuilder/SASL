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


def row_from_result_dir(result_dir: Path) -> dict[str, object]:
    metrics = read_json(result_dir / "repeated_observation_metrics.json")
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
        "repeat_count": best.get("repeat_count", ""),
        "fusion_mode": best.get("fusion_mode", ""),
        "trim_fraction": best.get("trim_fraction", ""),
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
        if child.is_dir() and (child / "repeated_observation_metrics.json").is_file()
    ]


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
        "Repeated observation fusion all-scenarios followup",
        "",
        f"strict_pass_count: {strict_pass_count}/{len(rows)}",
        f"runtime_truth_used: {runtime_truth_used}",
        (
            "recommended_runtime_structure: repeated_observation_trimmed_mean_n7 with a runtime profile selector; "
            "use identity passthrough when a calibrated profile regresses validation range RMSE."
        ),
        "",
        "Rows:",
    ]
    for row in sorted(rows, key=rank_key):
        lines.append(
            (
                "- {scenario_dir}: scenario={scenario}, status={status}, pass={pass_count}/{trial_count}, "
                "range_mean={range_mean:.6f}m, pos_mean={pos_mean:.6f}m, target_max={target_max:.6f}m"
            ).format(
                scenario_dir=row.get("scenario_dir", ""),
                scenario=row.get("scenario", ""),
                status=row.get("status", ""),
                pass_count=int(float(row.get("pass_count", 0) or 0)),
                trial_count=int(float(row.get("trial_count", 0) or 0)),
                range_mean=float(row.get("range_rmse_corrected_mean_m", "nan")),
                pos_mean=float(row.get("position_rmse_corrected_mean_m", "nan")),
                target_max=float(row.get("target_max_abs_error_corrected_max_m", "nan")),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize repeated observation fusion result directories.")
    parser.add_argument("--result-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows = [row_from_result_dir(path) for path in discover_result_dirs(args.result_root)]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "repeated_observation_all_scenarios_summary.csv", rows)
    write_summary_txt(args.output_dir / "repeated_observation_all_scenarios_summary.txt", rows)
    print(f"wrote {len(rows)} rows to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
