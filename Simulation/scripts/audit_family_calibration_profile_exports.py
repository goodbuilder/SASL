#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from audit_stereodistance_active_profile_transfer import pass_replay_summary  # noqa: E402
from select_stereodistance_profile_by_calibration_replay import finite_float, write_csv, write_json  # noqa: E402
from select_stereodistance_profile_by_runtime_measurement_guard import sweep_calibration_csvs  # noqa: E402
from stereodistance_profile_replay import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    load_profile,
    replay_rows,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "For each selected scenario/seed, fit a calibration-time StereoDistance runtime profile "
            "from that seed's CSV and replay it on the same scenario family. Truth is used only for "
            "calibration-time fitting and offline replay metrics; exported profiles consume raw "
            "stereo-center RDF/disparity only."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--target-scenarios", default="")
    parser.add_argument("--source-seeds", default="")
    parser.add_argument(
        "--export-policies",
        default="final_recommendation",
        help=(
            "Comma-separated policy profiles to export from each calibration fit. "
            "Use final_recommendation for the selected pipeline recommendation, or policy names "
            "from policy_recommendations.csv such as balanced,target_specialist,lowest_validation_rmse."
        ),
    )
    parser.add_argument("--split", default="validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--seed", type=int, default=20260613)
    parser.add_argument("--pipeline-script", type=Path, default=SCRIPT_DIR / "rtk_stereo_calibration_pipeline.py")
    return parser.parse_args()


def parse_csv_filter(value: str | Sequence[str] | None) -> list[str]:
    if value is None:
        return []
    items = value.split(",") if isinstance(value, str) else list(value)
    parsed: list[str] = []
    for item in items:
        text = str(item).strip()
        if text and text not in parsed:
            parsed.append(text)
    return parsed


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def selected_runtime_profile_path(fit_dir: Path) -> Path:
    profile = fit_dir / "sim3_runtime_profile.json"
    if profile.is_file():
        return profile
    manifest_path = fit_dir / "runtime_profiles" / "manifest.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        for row in manifest if isinstance(manifest, list) else []:
            if isinstance(row, Mapping) and bool(row.get("selected")):
                relative = str(row.get("profile_path", ""))
                if relative:
                    candidate = fit_dir / relative
                    if candidate.is_file():
                        return candidate
    raise FileNotFoundError(f"no selected runtime profile found under {fit_dir}")


def runtime_profile_path_for_method(fit_dir: Path, method_name: str) -> Path:
    manifest_path = fit_dir / "runtime_profiles" / "manifest.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        for row in manifest if isinstance(manifest, list) else []:
            if not isinstance(row, Mapping):
                continue
            if row.get("method") != method_name:
                continue
            relative = str(row.get("profile_path", ""))
            if relative:
                candidate = fit_dir / relative
                if candidate.is_file():
                    return candidate
    legacy_candidate = fit_dir / f"{method_name}.json"
    if legacy_candidate.is_file():
        return legacy_candidate
    raise FileNotFoundError(f"no runtime profile for method {method_name!r} found under {fit_dir}")


def profile_exports_for_policies(fit_dir: Path, policies: Sequence[str]) -> list[dict[str, object]]:
    run_config_path = fit_dir / "run_config.json"
    run_config = json.loads(run_config_path.read_text(encoding="utf-8")) if run_config_path.is_file() else {}
    policy_rows = read_csv(fit_dir / "policy_recommendations.csv") if (fit_dir / "policy_recommendations.csv").is_file() else []
    exports: list[dict[str, object]] = []
    for policy in policies:
        if policy == "final_recommendation":
            method = str(run_config.get("recommendation", ""))
            source_profile = selected_runtime_profile_path(fit_dir)
            purpose = str(run_config.get("recommendation_reason", ""))
        else:
            policy_row = next((row for row in policy_rows if row.get("policy") == policy), None)
            if policy_row is None:
                raise ValueError(f"policy {policy!r} was not found in {fit_dir / 'policy_recommendations.csv'}")
            method = str(policy_row.get("method", ""))
            source_profile = runtime_profile_path_for_method(fit_dir, method)
            purpose = str(policy_row.get("purpose", ""))
        exports.append(
            {
                "source_policy": policy,
                "method": method,
                "source_profile": source_profile,
                "purpose": purpose,
            }
        )
    return exports


def run_pipeline(
    *,
    pipeline_script: Path,
    input_csv: Path,
    fit_dir: Path,
    target_range_m: float,
    target_window_m: float,
    train_ratio: float,
    seed: int,
) -> subprocess.CompletedProcess[str]:
    command = [
        sys.executable,
        str(pipeline_script),
        "--mode",
        "csv",
        "--input-csv",
        str(input_csv),
        "--output-dir",
        str(fit_dir),
        "--target-range-m",
        str(target_range_m),
        "--target-window-m",
        str(target_window_m),
        "--train-ratio",
        str(train_ratio),
        "--seed",
        str(seed),
    ]
    return subprocess.run(command, check=True, text=True, capture_output=True)


def scenario_seed_csvs(sweep_dir: Path, scenarios: Sequence[str], source_seeds: Sequence[str]) -> list[tuple[str, str, Path]]:
    scenario_filter = set(scenarios)
    seed_filter = set(source_seeds)
    rows: list[tuple[str, str, Path]] = []
    for scenario, seed, path in sweep_calibration_csvs(sweep_dir):
        if scenario_filter and scenario not in scenario_filter:
            continue
        if seed_filter and seed not in seed_filter:
            continue
        rows.append((scenario, seed, path))
    return rows


def family_target_csvs(sweep_dir: Path, scenario: str) -> list[tuple[str, Path]]:
    return [(seed, path) for row_scenario, seed, path in sweep_calibration_csvs(sweep_dir) if row_scenario == scenario]


def replay_profile_on_family(
    *,
    profile_json: Path,
    sweep_dir: Path,
    scenario: str,
    source_seed: str,
    source_policy: str,
    split: str,
    target_range_m: float,
    target_window_m: float,
    target_error_limit_m: float,
    output_dir: Path,
) -> list[dict[str, object]]:
    profile = load_profile(profile_json)
    rows_out: list[dict[str, object]] = []
    for target_seed, sample_csv in family_target_csvs(sweep_dir, scenario):
        predictions, summary = replay_rows(
            read_csv(sample_csv),
            profile,
            split_filter=split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
        )
        passed, reason = pass_replay_summary(summary, target_error_limit_m=target_error_limit_m)
        prediction_dir = output_dir / "per_run_predictions" / scenario / source_seed / source_policy / target_seed
        write_csv(prediction_dir / "profile_replay_predictions.csv", predictions)
        write_csv(prediction_dir / "profile_replay_summary.csv", [summary])
        rows_out.append(
            {
                "scenario": scenario,
                "source_seed": source_seed,
                "source_policy": source_policy,
                "target_seed": target_seed,
                "is_heldout_seed": source_seed != target_seed,
                "sample_csv": str(sample_csv),
                "profile_json": str(profile_json),
                "profile_id": summary.get("profile_id", ""),
                "profile_mode": summary.get("profile_mode", ""),
                "pass_family_profile_transfer": passed,
                "pass_reason": reason,
                "evaluated_row_count": summary.get("evaluated_row_count", ""),
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
                "runtime_input_contract": "family profile replay uses raw stereo-center RDF/disparity only; truth is audit-only",
            }
        )
    return rows_out


def summarize(rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    heldout_rows = [row for row in rows if bool(row.get("is_heldout_seed"))]
    rmse = [finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")) for row in rows]
    rmse = [value for value in rmse if math.isfinite(value)]
    target_max = [finite_float(row.get("target_max_abs_error_corrected_m"), float("nan")) for row in rows]
    target_max = [value for value in target_max if math.isfinite(value)]
    pass_count = sum(1 for row in rows if bool(row.get("pass_family_profile_transfer")))
    heldout_pass_count = sum(1 for row in heldout_rows if bool(row.get("pass_family_profile_transfer")))
    return {
        "mode": "family_calibration_profile_export_audit",
        "family_replay_count": len(rows),
        "family_replay_pass_count": pass_count,
        "family_replay_pass_all": bool(rows) and pass_count == len(rows),
        "heldout_replay_count": len(heldout_rows),
        "heldout_replay_pass_count": heldout_pass_count,
        "heldout_replay_pass_all": bool(heldout_rows) and heldout_pass_count == len(heldout_rows),
        "validation_range_rmse_corrected_mean_m": float(np.mean(rmse)) if rmse else float("nan"),
        "validation_range_rmse_corrected_max_m": float(np.max(rmse)) if rmse else float("nan"),
        "target_max_abs_error_max_m": float(np.max(target_max)) if target_max else float("nan"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "family profile replay uses raw stereo-center RDF/disparity only; truth is audit-only",
    }


def summarize_by_scenario(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    scenarios = sorted({str(row.get("scenario", "")) for row in rows if str(row.get("scenario", ""))})
    summary_rows: list[dict[str, object]] = []
    for scenario in scenarios:
        scenario_rows = [row for row in rows if row.get("scenario") == scenario]
        heldout_rows = [row for row in scenario_rows if bool(row.get("is_heldout_seed"))]
        rmse = [
            finite_float(row.get("validation_range_rmse_corrected_m"), float("nan"))
            for row in scenario_rows
        ]
        rmse = [value for value in rmse if math.isfinite(value)]
        improvement = [
            finite_float(row.get("validation_range_rmse_improvement_m"), float("nan"))
            for row in scenario_rows
        ]
        improvement = [value for value in improvement if math.isfinite(value)]
        target_abs = [
            finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan"))
            for row in scenario_rows
        ]
        target_abs = [value for value in target_abs if math.isfinite(value)]
        target_max = [
            finite_float(row.get("target_max_abs_error_corrected_m"), float("nan"))
            for row in scenario_rows
        ]
        target_max = [value for value in target_max if math.isfinite(value)]
        pass_count = sum(1 for row in scenario_rows if bool(row.get("pass_family_profile_transfer")))
        heldout_pass_count = sum(1 for row in heldout_rows if bool(row.get("pass_family_profile_transfer")))
        summary_rows.append(
            {
                "scenario": scenario,
                "family_replay_count": len(scenario_rows),
                "family_replay_pass_count": pass_count,
                "family_replay_pass_all": bool(scenario_rows) and pass_count == len(scenario_rows),
                "heldout_replay_count": len(heldout_rows),
                "heldout_replay_pass_count": heldout_pass_count,
                "heldout_replay_pass_all": bool(heldout_rows) and heldout_pass_count == len(heldout_rows),
                "validation_range_rmse_corrected_mean_m": float(np.mean(rmse)) if rmse else float("nan"),
                "validation_range_rmse_corrected_max_m": float(np.max(rmse)) if rmse else float("nan"),
                "validation_range_rmse_improvement_min_m": float(np.min(improvement)) if improvement else float("nan"),
                "target_abs_mean_error_corrected_max_m": float(np.max(target_abs)) if target_abs else float("nan"),
                "target_max_abs_error_max_m": float(np.max(target_max)) if target_max else float("nan"),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": (
                    "family profile replay uses raw stereo-center RDF/disparity only; truth is audit-only"
                ),
            }
        )
    return summary_rows


def summarize_by_policy(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    policies = sorted({str(row.get("source_policy", "")) for row in rows if str(row.get("source_policy", ""))})
    summary_rows: list[dict[str, object]] = []
    for policy in policies:
        policy_rows = [row for row in rows if row.get("source_policy") == policy]
        heldout_rows = [row for row in policy_rows if bool(row.get("is_heldout_seed"))]
        rmse = [
            finite_float(row.get("validation_range_rmse_corrected_m"), float("nan"))
            for row in policy_rows
        ]
        rmse = [value for value in rmse if math.isfinite(value)]
        improvement = [
            finite_float(row.get("validation_range_rmse_improvement_m"), float("nan"))
            for row in policy_rows
        ]
        improvement = [value for value in improvement if math.isfinite(value)]
        target_abs = [
            finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan"))
            for row in policy_rows
        ]
        target_abs = [value for value in target_abs if math.isfinite(value)]
        target_max = [
            finite_float(row.get("target_max_abs_error_corrected_m"), float("nan"))
            for row in policy_rows
        ]
        target_max = [value for value in target_max if math.isfinite(value)]
        pass_count = sum(1 for row in policy_rows if bool(row.get("pass_family_profile_transfer")))
        heldout_pass_count = sum(1 for row in heldout_rows if bool(row.get("pass_family_profile_transfer")))
        summary_rows.append(
            {
                "source_policy": policy,
                "family_replay_count": len(policy_rows),
                "family_replay_pass_count": pass_count,
                "family_replay_pass_all": bool(policy_rows) and pass_count == len(policy_rows),
                "heldout_replay_count": len(heldout_rows),
                "heldout_replay_pass_count": heldout_pass_count,
                "heldout_replay_pass_all": bool(heldout_rows) and heldout_pass_count == len(heldout_rows),
                "validation_range_rmse_corrected_mean_m": float(np.mean(rmse)) if rmse else float("nan"),
                "validation_range_rmse_corrected_max_m": float(np.max(rmse)) if rmse else float("nan"),
                "validation_range_rmse_improvement_min_m": float(np.min(improvement)) if improvement else float("nan"),
                "target_abs_mean_error_corrected_max_m": float(np.max(target_abs)) if target_abs else float("nan"),
                "target_max_abs_error_max_m": float(np.max(target_max)) if target_max else float("nan"),
                "axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "runtime_input_contract": (
                    "family profile replay uses raw stereo-center RDF/disparity only; truth is audit-only"
                ),
            }
        )
    return summary_rows


def summarize_group(rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    heldout_rows = [row for row in rows if bool(row.get("is_heldout_seed"))]
    rmse = [finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")) for row in rows]
    rmse = [value for value in rmse if math.isfinite(value)]
    improvement = [finite_float(row.get("validation_range_rmse_improvement_m"), float("nan")) for row in rows]
    improvement = [value for value in improvement if math.isfinite(value)]
    target_abs = [finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan")) for row in rows]
    target_abs = [value for value in target_abs if math.isfinite(value)]
    target_max = [finite_float(row.get("target_max_abs_error_corrected_m"), float("nan")) for row in rows]
    target_max = [value for value in target_max if math.isfinite(value)]
    pass_count = sum(1 for row in rows if bool(row.get("pass_family_profile_transfer")))
    heldout_pass_count = sum(1 for row in heldout_rows if bool(row.get("pass_family_profile_transfer")))
    return {
        "family_replay_count": len(rows),
        "family_replay_pass_count": pass_count,
        "family_replay_pass_all": bool(rows) and pass_count == len(rows),
        "heldout_replay_count": len(heldout_rows),
        "heldout_replay_pass_count": heldout_pass_count,
        "heldout_replay_pass_all": bool(heldout_rows) and heldout_pass_count == len(heldout_rows),
        "validation_range_rmse_corrected_mean_m": float(np.mean(rmse)) if rmse else float("nan"),
        "validation_range_rmse_corrected_max_m": float(np.max(rmse)) if rmse else float("nan"),
        "validation_range_rmse_improvement_min_m": float(np.min(improvement)) if improvement else float("nan"),
        "target_abs_mean_error_corrected_max_m": float(np.max(target_abs)) if target_abs else float("nan"),
        "target_max_abs_error_max_m": float(np.max(target_max)) if target_max else float("nan"),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "family profile replay uses raw stereo-center RDF/disparity only; truth is audit-only",
    }


def summarize_by_scenario_policy(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    keys = sorted(
        {
            (str(row.get("scenario", "")), str(row.get("source_policy", "")))
            for row in rows
            if str(row.get("scenario", "")) and str(row.get("source_policy", ""))
        }
    )
    summary_rows: list[dict[str, object]] = []
    for scenario, policy in keys:
        group_rows = [
            row
            for row in rows
            if row.get("scenario") == scenario and row.get("source_policy") == policy
        ]
        summary = summarize_group(group_rows)
        summary.update({"scenario": scenario, "source_policy": policy})
        summary_rows.append(summary)
    return summary_rows


def recommend_policy_by_scenario(
    scenario_policy_rows: Sequence[Mapping[str, object]]
) -> list[dict[str, object]]:
    scenarios = sorted({str(row.get("scenario", "")) for row in scenario_policy_rows if row.get("scenario")})
    recommendation_rows: list[dict[str, object]] = []
    for scenario in scenarios:
        rows = [row for row in scenario_policy_rows if row.get("scenario") == scenario]
        recommended_policy, reason = choose_recommended_policy(rows)
        selected = next((row for row in rows if row.get("source_policy") == recommended_policy), {})
        recommendation = dict(selected)
        recommendation["scenario"] = scenario
        recommendation["recommended_source_policy"] = recommended_policy
        recommendation["recommended_source_policy_reason"] = reason
        recommendation_rows.append(recommendation)
    return recommendation_rows


def choose_recommended_policy(policy_rows: Sequence[Mapping[str, object]]) -> tuple[str, str]:
    if not policy_rows:
        return "", "No policy profiles were evaluated."

    def value(row: Mapping[str, object], key: str, default: float) -> float:
        return finite_float(row.get(key), default)

    stable_passes = [
        row
        for row in policy_rows
        if bool(row.get("heldout_replay_pass_all"))
        and value(row, "validation_range_rmse_improvement_min_m", float("-inf")) > 0.5
    ]
    if stable_passes:
        best = min(
            stable_passes,
            key=lambda row: (
                value(row, "target_max_abs_error_max_m", float("inf")),
                value(row, "validation_range_rmse_corrected_max_m", float("inf")),
                str(row.get("source_policy", "")),
            ),
        )
        return (
            str(best.get("source_policy", "")),
            (
                "Selected among policies that pass all heldout replays and keep at least 0.5 m "
                "minimum validation RMSE improvement; tie-breakers are 1852m target max error "
                "and validation RMSE max."
            ),
        )

    best = min(
        policy_rows,
        key=lambda row: (
            -int(finite_float(row.get("heldout_replay_pass_count"), 0.0)),
            value(row, "target_max_abs_error_max_m", float("inf")),
            value(row, "validation_range_rmse_corrected_max_m", float("inf")),
            str(row.get("source_policy", "")),
        ),
    )
    return (
        str(best.get("source_policy", "")),
        (
            "No policy passed the stable heldout criteria; selected the highest heldout pass count, "
            "then lowest 1852m target max error and validation RMSE max."
        ),
    )


def write_summary_txt(
    path: Path,
    summary: Mapping[str, object],
    rows: Sequence[Mapping[str, object]],
    scenario_rows: Sequence[Mapping[str, object]],
    policy_rows: Sequence[Mapping[str, object]],
    scenario_policy_rows: Sequence[Mapping[str, object]],
    scenario_recommendation_rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "Family calibration profile export audit",
        "",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "",
        "Summary:",
    ]
    for key, value in summary.items():
        if isinstance(value, float):
            value = f"{value:.9f}"
        lines.append(f"{key}: {value}")
    lines.extend(["", "Scenario summary:"])
    for row in scenario_rows:
        lines.append(
            "{scenario}: family_pass={family_pass}/{family_count} heldout_pass={heldout_pass}/{heldout_count} "
            "rmse_max={rmse_max} improvement_min={improvement_min} target_max={target_max}".format(
                scenario=row.get("scenario", ""),
                family_pass=row.get("family_replay_pass_count", ""),
                family_count=row.get("family_replay_count", ""),
                heldout_pass=row.get("heldout_replay_pass_count", ""),
                heldout_count=row.get("heldout_replay_count", ""),
                rmse_max=row.get("validation_range_rmse_corrected_max_m", ""),
                improvement_min=row.get("validation_range_rmse_improvement_min_m", ""),
                target_max=row.get("target_max_abs_error_max_m", ""),
            )
        )
    lines.extend(["", "Policy summary:"])
    for row in policy_rows:
        lines.append(
            "{policy}: family_pass={family_pass}/{family_count} heldout_pass={heldout_pass}/{heldout_count} "
            "rmse_max={rmse_max} improvement_min={improvement_min} target_max={target_max}".format(
                policy=row.get("source_policy", ""),
                family_pass=row.get("family_replay_pass_count", ""),
                family_count=row.get("family_replay_count", ""),
                heldout_pass=row.get("heldout_replay_pass_count", ""),
                heldout_count=row.get("heldout_replay_count", ""),
                rmse_max=row.get("validation_range_rmse_corrected_max_m", ""),
                improvement_min=row.get("validation_range_rmse_improvement_min_m", ""),
                target_max=row.get("target_max_abs_error_max_m", ""),
            )
        )
    lines.extend(["", "Scenario policy recommendations:"])
    for row in scenario_recommendation_rows:
        lines.append(
            "{scenario}: policy={policy} heldout_pass={heldout_pass}/{heldout_count} rmse_max={rmse_max} "
            "improvement_min={improvement_min} target_max={target_max}".format(
                scenario=row.get("scenario", ""),
                policy=row.get("recommended_source_policy", ""),
                heldout_pass=row.get("heldout_replay_pass_count", ""),
                heldout_count=row.get("heldout_replay_count", ""),
                rmse_max=row.get("validation_range_rmse_corrected_max_m", ""),
                improvement_min=row.get("validation_range_rmse_improvement_min_m", ""),
                target_max=row.get("target_max_abs_error_max_m", ""),
            )
        )
    lines.extend(["", "Rows:"])
    for row in rows:
        lines.append(
            "{scenario}/{source}->{target}: heldout={heldout} pass={passed} rmse={rmse} "
            "target_max={target_max} reason={reason}".format(
                scenario=row.get("scenario", ""),
                source=row.get("source_seed", ""),
                target=row.get("target_seed", ""),
                heldout=row.get("is_heldout_seed", ""),
                passed=row.get("pass_family_profile_transfer", ""),
                rmse=row.get("validation_range_rmse_corrected_m", ""),
                target_max=row.get("target_max_abs_error_corrected_m", ""),
                reason=row.get("pass_reason", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_summary(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    labels = [f"{row['scenario']}\n{row['source_seed']}->{row['target_seed']}" for row in rows]
    rmse = np.asarray([finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")) for row in rows])
    target_max = np.asarray([finite_float(row.get("target_max_abs_error_corrected_m"), float("nan")) for row in rows])
    passed = np.asarray([1.0 if bool(row.get("pass_family_profile_transfer")) else 0.0 for row in rows])
    x = np.arange(len(rows))
    fig, axes = plt.subplots(3, 1, figsize=(max(10, 0.75 * len(rows)), 8), sharex=True, constrained_layout=True)
    axes[0].bar(x, passed, color="tab:green")
    axes[0].set_ylabel("pass")
    axes[0].set_ylim(0, 1.2)
    axes[1].bar(x, rmse, color="tab:blue")
    axes[1].set_ylabel("RMSE (m)")
    axes[2].bar(x, target_max, color="tab:orange")
    axes[2].axhline(10.0, color="tab:red", linestyle="--", linewidth=1)
    axes[2].set_ylabel("target max abs (m)")
    axes[2].set_xticks(x, labels, rotation=65, ha="right")
    for axis in axes:
        axis.grid(True, axis="y", alpha=0.3)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def audit_family_exports(args: argparse.Namespace) -> dict[str, object]:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    scenarios = parse_csv_filter(args.target_scenarios)
    source_seeds = parse_csv_filter(args.source_seeds)
    export_policies = parse_csv_filter(args.export_policies) or ["final_recommendation"]
    source_rows = scenario_seed_csvs(args.sweep_dir, scenarios, source_seeds)
    rows_out: list[dict[str, object]] = []
    export_rows: list[dict[str, object]] = []
    for scenario, source_seed, input_csv in source_rows:
        run_dir = args.output_dir / "exports" / scenario / source_seed
        fit_dir = run_dir / "calibration_fit"
        completed = run_pipeline(
            pipeline_script=args.pipeline_script,
            input_csv=input_csv,
            fit_dir=fit_dir,
            target_range_m=args.target_range_m,
            target_window_m=args.target_window_m,
            train_ratio=args.train_ratio,
            seed=args.seed,
        )
        run_config_path = fit_dir / "run_config.json"
        run_config = json.loads(run_config_path.read_text(encoding="utf-8")) if run_config_path.is_file() else {}
        for export in profile_exports_for_policies(fit_dir, export_policies):
            source_policy = str(export["source_policy"])
            source_profile = Path(export["source_profile"])
            active_profile = (
                run_dir / "active_runtime_profile.json"
                if source_policy == "final_recommendation"
                else run_dir / source_policy / "active_runtime_profile.json"
            )
            active_profile.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source_profile, active_profile)
            export_rows.append(
                {
                    "scenario": scenario,
                    "source_seed": source_seed,
                    "source_policy": source_policy,
                    "input_csv": str(input_csv),
                    "active_profile": str(active_profile),
                    "source_profile": str(source_profile),
                    "recommended_method": run_config.get("recommendation", ""),
                    "exported_method": export.get("method", ""),
                    "recommendation_reason": run_config.get("recommendation_reason", ""),
                    "policy_purpose": export.get("purpose", ""),
                    "pipeline_stdout": completed.stdout,
                    "pipeline_stderr": completed.stderr,
                    "axis_contract": RDF_AXIS_CONTRACT,
                    "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                    "runtime_input_contract": "exported profile consumes raw stereo-center RDF/disparity only",
                }
            )
            rows_out.extend(
                replay_profile_on_family(
                    profile_json=active_profile,
                    sweep_dir=args.sweep_dir,
                    scenario=scenario,
                    source_seed=source_seed,
                    source_policy=source_policy,
                    split=args.split,
                    target_range_m=args.target_range_m,
                    target_window_m=args.target_window_m,
                    target_error_limit_m=args.target_error_limit_m,
                    output_dir=args.output_dir,
                )
            )
    write_csv(args.output_dir / "family_calibration_profile_exports.csv", export_rows)
    write_csv(args.output_dir / "family_calibration_profile_transfer.csv", rows_out)
    scenario_summary_rows = summarize_by_scenario(rows_out)
    policy_summary_rows = summarize_by_policy(rows_out)
    scenario_policy_summary_rows = summarize_by_scenario_policy(rows_out)
    scenario_policy_recommendation_rows = recommend_policy_by_scenario(scenario_policy_summary_rows)
    recommended_policy, recommended_policy_reason = choose_recommended_policy(policy_summary_rows)
    write_csv(args.output_dir / "family_calibration_profile_scenario_summary.csv", scenario_summary_rows)
    write_csv(args.output_dir / "family_calibration_profile_policy_summary.csv", policy_summary_rows)
    write_csv(
        args.output_dir / "family_calibration_profile_scenario_policy_summary.csv",
        scenario_policy_summary_rows,
    )
    write_csv(
        args.output_dir / "family_calibration_profile_scenario_policy_recommendations.csv",
        scenario_policy_recommendation_rows,
    )
    summary = summarize(rows_out)
    summary.update(
        {
            "sweep_dir": str(args.sweep_dir),
            "target_scenarios": ",".join(scenarios),
            "source_seeds": ",".join(source_seeds),
            "export_policies": ",".join(export_policies),
            "export_count": len(export_rows),
            "scenario_summary_count": len(scenario_summary_rows),
            "policy_summary_count": len(policy_summary_rows),
            "scenario_policy_summary_count": len(scenario_policy_summary_rows),
            "scenario_policy_recommendation_count": len(scenario_policy_recommendation_rows),
            "recommended_source_policy": recommended_policy,
            "recommended_source_policy_reason": recommended_policy_reason,
        }
    )
    write_json(args.output_dir / "family_calibration_profile_export_summary.json", summary)
    write_json(args.output_dir / "family_calibration_profile_scenario_summary.json", scenario_summary_rows)
    write_json(args.output_dir / "family_calibration_profile_policy_summary.json", policy_summary_rows)
    write_json(
        args.output_dir / "family_calibration_profile_scenario_policy_summary.json",
        scenario_policy_summary_rows,
    )
    write_json(
        args.output_dir / "family_calibration_profile_scenario_policy_recommendations.json",
        scenario_policy_recommendation_rows,
    )
    write_summary_txt(
        args.output_dir / "family_calibration_profile_export_summary.txt",
        summary,
        rows_out,
        scenario_summary_rows,
        policy_summary_rows,
        scenario_policy_summary_rows,
        scenario_policy_recommendation_rows,
    )
    plot_summary(args.output_dir / "family_calibration_profile_export_summary.png", rows_out)
    return summary


def main() -> int:
    summary = audit_family_exports(parse_args())
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
