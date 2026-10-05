#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from audit_stereodistance_active_profile_transfer import pass_replay_summary  # noqa: E402
from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from select_stereodistance_profile_by_runtime_measurement_guard import sweep_calibration_csvs  # noqa: E402
from select_stereodistance_profile_from_catalog import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    catalog_root,
    load_catalog,
)
from stereodistance_profile_replay import load_profile, replay_rows  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build a transfer matrix for a policy-aware StereoDistance scenario profile catalog. "
            "Runtime profiles consume raw stereo-center RDF/disparity only; truth is used only for "
            "offline transfer replay metrics."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def finite_values(rows: Sequence[Mapping[str, object]], key: str) -> list[float]:
    values = [finite_float(row.get(key), float("nan")) for row in rows]
    return [value for value in values if math.isfinite(value)]


def profile_path_for_row(catalog_json: Path, row: Mapping[str, object]) -> Path:
    relative = str(row.get("catalog_profile_path", ""))
    if not relative:
        raise ValueError(f"catalog row for {row.get('scenario', '')!r} has no catalog_profile_path")
    return catalog_root(catalog_json) / relative


def build_transfer_matrix(
    *,
    sweep_dir: Path,
    catalog_json: Path,
    output_dir: Path,
    split: str = "validation",
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
) -> dict[str, object]:
    catalog = load_catalog(catalog_json)
    output_dir.mkdir(parents=True, exist_ok=True)
    replay_rows_out: list[dict[str, object]] = []
    pair_groups: dict[tuple[str, str], list[dict[str, object]]] = {}
    profiles = [row for row in catalog.get("profiles", []) if isinstance(row, Mapping)]
    target_csvs = list(sweep_calibration_csvs(sweep_dir))
    for profile_row in profiles:
        source_scenario = str(profile_row.get("scenario", ""))
        profile_json = profile_path_for_row(catalog_json, profile_row)
        profile = load_profile(profile_json)
        for target_scenario, target_seed, sample_csv in target_csvs:
            predictions, summary = replay_rows(
                read_csv(sample_csv),
                profile,
                split_filter=split,
                target_range_m=target_range_m,
                target_window_m=target_window_m,
            )
            passed, reason = pass_replay_summary(summary, target_error_limit_m=target_error_limit_m)
            replay_dir = output_dir / "per_pair_replay" / source_scenario / target_scenario / target_seed
            write_csv(replay_dir / "profile_replay_predictions.csv", predictions)
            write_csv(replay_dir / "profile_replay_summary.csv", [summary])
            row = {
                "source_scenario": source_scenario,
                "source_method": profile_row.get("method", summary.get("profile_id", "")),
                "source_policy": profile_row.get("source_policy", ""),
                "target_scenario": target_scenario,
                "target_seed": target_seed,
                "profile_json": str(profile_json),
                "sample_csv": str(sample_csv),
                "pass_replay": passed,
                "pass_reason": reason,
                "validation_range_rmse_raw_m": summary.get("validation_range_rmse_raw_m", ""),
                "validation_range_rmse_corrected_m": summary.get("validation_range_rmse_corrected_m", ""),
                "validation_range_rmse_improvement_m": finite_float(
                    summary.get("validation_range_rmse_raw_m"), float("nan")
                )
                - finite_float(summary.get("validation_range_rmse_corrected_m"), float("nan")),
                "target_window_count": summary.get("target_window_count", ""),
                "target_abs_mean_error_corrected_m": summary.get("target_abs_mean_error_corrected_m", ""),
                "target_max_abs_error_corrected_m": summary.get("target_max_abs_error_corrected_m", ""),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": "policy catalog transfer replay uses raw stereo-center RDF/disparity only; truth is audit-only",
            }
            replay_rows_out.append(row)
            pair_groups.setdefault((source_scenario, target_scenario), []).append(row)
    pair_rows: list[dict[str, object]] = []
    for (source_scenario, target_scenario), rows in sorted(pair_groups.items()):
        rmse = finite_values(rows, "validation_range_rmse_corrected_m")
        improvement = finite_values(rows, "validation_range_rmse_improvement_m")
        target_abs = finite_values(rows, "target_abs_mean_error_corrected_m")
        target_max = finite_values(rows, "target_max_abs_error_corrected_m")
        pass_count = sum(1 for row in rows if bool(row.get("pass_replay")))
        pair_rows.append(
            {
                "source_scenario": source_scenario,
                "target_scenario": target_scenario,
                "source_method": rows[0].get("source_method", ""),
                "source_policy": rows[0].get("source_policy", ""),
                "replay_count": len(rows),
                "pass_count": pass_count,
                "fail_count": len(rows) - pass_count,
                "pass_all": bool(rows) and pass_count == len(rows),
                "min_validation_range_rmse_improvement_m": float(np.min(improvement)) if improvement else float("nan"),
                "mean_validation_range_rmse_corrected_m": float(np.mean(rmse)) if rmse else float("nan"),
                "max_validation_range_rmse_corrected_m": float(np.max(rmse)) if rmse else float("nan"),
                "max_target_abs_mean_error_corrected_m": float(np.max(target_abs)) if target_abs else float("nan"),
                "max_target_abs_error_corrected_m": float(np.max(target_max)) if target_max else float("nan"),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
    pass_all_count = sum(1 for row in pair_rows if bool(row.get("pass_all")))
    summary = {
        "mode": "policy_catalog_transfer_matrix",
        "catalog_json": str(catalog_json),
        "sweep_dir": str(sweep_dir),
        "profile_replay_count": len(replay_rows_out),
        "profile_pair_count": len(pair_rows),
        "pass_all_pair_count": pass_all_count,
        "pass_all_pair_ratio": pass_all_count / len(pair_rows) if pair_rows else float("nan"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "policy catalog transfer matrix replays raw stereo-center RDF/disparity only; truth is audit-only",
    }
    write_csv(output_dir / "policy_catalog_transfer_replay.csv", replay_rows_out)
    write_csv(output_dir / "policy_catalog_transfer_matrix.csv", pair_rows)
    write_json(output_dir / "policy_catalog_transfer_summary.json", summary)
    write_summary_txt(output_dir / "policy_catalog_transfer_summary.txt", summary, pair_rows)
    return summary


def write_summary_txt(path: Path, summary: Mapping[str, object], rows: Sequence[Mapping[str, object]]) -> None:
    lines = [
        "Policy catalog transfer matrix",
        "",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "",
        "Summary:",
    ]
    for key, value in summary.items():
        if isinstance(value, float):
            value = f"{value:.9f}" if math.isfinite(value) else "nan"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Pairs:"])
    for row in rows:
        lines.append(
            "{source}->{target}: pass={passed}/{count} rmse_mean={rmse} "
            "target_abs_max={target_abs} target_max={target_max}".format(
                source=row.get("source_scenario", ""),
                target=row.get("target_scenario", ""),
                passed=row.get("pass_count", ""),
                count=row.get("replay_count", ""),
                rmse=row.get("mean_validation_range_rmse_corrected_m", ""),
                target_abs=row.get("max_target_abs_mean_error_corrected_m", ""),
                target_max=row.get("max_target_abs_error_corrected_m", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    summary = build_transfer_matrix(
        sweep_dir=args.sweep_dir,
        catalog_json=args.catalog_json,
        output_dir=args.output_dir,
        split=args.split,
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
