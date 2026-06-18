#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import shutil
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
SELECTION_CONTRACT = (
    "Runtime profile selection may use explicit deployment/scenario labels only. It must not inspect "
    "PX4/RTK truth, validation errors, replay winners, or live target error."
)
FORBIDDEN_RUNTIME_FIELDS = [
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "truth_north_m",
    "truth_east_m",
    "truth_down_ned_m",
    "validation_range_rmse_corrected_m",
    "target_max_abs_error_corrected_m",
    "replay_selected_method",
]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export a portable StereoDistance runtime profile label selector package from a common-CSV "
            "selector portfolio audit. Calibration truth is used only by the upstream audit; this package "
            "selects profiles from deployment/scenario labels."
        )
    )
    parser.add_argument("--selector-summary-json", type=Path, required=True)
    parser.add_argument("--selected-runs-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--selector-mode", default="scenario_loso")
    parser.add_argument(
        "--objective",
        choices=("score", "target_then_rmse", "rmse_then_target"),
        default="rmse_then_target",
    )
    return parser.parse_args(argv)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def finite_float(value: object, default: float = math.inf) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def safe_label(value: object) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value).strip())
    return text.strip("._-") or "profile"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def row_sort_key(row: Mapping[str, object], objective: str) -> tuple[float, ...]:
    target = finite_float(row.get("target_max_abs_error_corrected_m"))
    rmse = finite_float(row.get("validation_range_rmse_corrected_m"))
    score = finite_float(row.get("common_csv_score"))
    if objective == "target_then_rmse":
        return (target, rmse, score)
    if objective == "rmse_then_target":
        return (rmse, target, score)
    return (score, target, rmse)


def choose_scenario_rows(rows: Sequence[Mapping[str, str]], objective: str) -> dict[str, Mapping[str, str]]:
    by_scenario: dict[str, list[Mapping[str, str]]] = {}
    for row in rows:
        scenario = str(row.get("scenario", "")).strip()
        if scenario:
            by_scenario.setdefault(scenario, []).append(row)
    selected: dict[str, Mapping[str, str]] = {}
    for scenario, scenario_rows in by_scenario.items():
        by_method: dict[str, list[Mapping[str, str]]] = {}
        for row in scenario_rows:
            by_method.setdefault(str(row.get("method", "")), []).append(row)
        method_scores: list[tuple[tuple[float, ...], str, Mapping[str, str]]] = []
        for method, method_rows in by_method.items():
            mean_row = {
                "target_max_abs_error_corrected_m": mean(
                    finite_float(row.get("target_max_abs_error_corrected_m")) for row in method_rows
                ),
                "validation_range_rmse_corrected_m": mean(
                    finite_float(row.get("validation_range_rmse_corrected_m")) for row in method_rows
                ),
                "common_csv_score": mean(finite_float(row.get("common_csv_score")) for row in method_rows),
            }
            representative = sorted(method_rows, key=lambda row: row_sort_key(row, objective))[0]
            method_scores.append((row_sort_key(mean_row, objective), method, representative))
        selected[scenario] = sorted(method_scores, key=lambda item: (item[0], item[1]))[0][2]
    return selected


def mean(values: Sequence[float]) -> float:
    finite_values = [value for value in values if math.isfinite(value)]
    return sum(finite_values) / len(finite_values) if finite_values else math.inf


def relative_to_output(path: Path, output_dir: Path) -> str:
    return str(path.resolve().relative_to(output_dir.resolve()))


def copy_profile(profile_json: Path, label: str, output_dir: Path) -> tuple[Path, Path, dict[str, object]]:
    profile_payload = load_json(profile_json)
    package_dir = output_dir / "packages" / safe_label(label)
    package_dir.mkdir(parents=True, exist_ok=True)
    output_profile = package_dir / "active_runtime_profile.json"
    shutil.copy2(profile_json, output_profile)
    return output_profile, package_dir, profile_payload


def export_selector_package(
    *,
    selector_summary_json: Path,
    selected_runs_csv: Path,
    output_dir: Path,
    selector_mode: str = "scenario_loso",
    objective: str = "rmse_then_target",
) -> dict[str, object]:
    summary = load_json(selector_summary_json)
    rows = read_csv(selected_runs_csv)
    mode_rows = [row for row in rows if row.get("selector_mode") == selector_mode]
    global_rows = [row for row in rows if row.get("selector_mode") == "global_single"]
    if not mode_rows:
        raise ValueError(f"no selected rows for selector mode: {selector_mode}")
    output_dir.mkdir(parents=True, exist_ok=True)

    scenario_rows = choose_scenario_rows(mode_rows, objective)
    baseline_row = sorted(global_rows or mode_rows, key=lambda row: row_sort_key(row, objective))[0]
    profile_records: dict[str, dict[str, object]] = {}
    label_profile_map: dict[str, str] = {}

    def ensure_profile(label: str, row: Mapping[str, str]) -> str:
        profile_path = Path(str(row.get("profile_json", "")))
        if not profile_path.is_file():
            raise ValueError(f"profile_json missing for {label}: {profile_path}")
        profile_label = safe_label(label)
        if profile_label not in profile_records:
            copied_profile, package_dir, profile_payload = copy_profile(profile_path, profile_label, output_dir)
            profile_records[profile_label] = {
                "profile_label": profile_label,
                "source_profile_json": str(profile_path),
                "profile_json": relative_to_output(copied_profile, output_dir),
                "package_dir": relative_to_output(package_dir, output_dir),
                "profile_sha256": sha256_file(copied_profile),
                "method": profile_payload.get("method", row.get("method", "")),
                "profile_id": profile_payload.get("profile_id", ""),
                "profile_mode": profile_payload.get("mode", ""),
                "axis_contract": profile_payload.get("axis_contract", ""),
                "runtime_truth_used": False,
            }
        return profile_label

    baseline_label = ensure_profile("global_single_baseline", baseline_row)
    for scenario, row in scenario_rows.items():
        profile_label = ensure_profile(f"scenario_{scenario}", row)
        label_profile_map[scenario] = profile_label
    for row in mode_rows:
        scenario = safe_label(row.get("scenario", ""))
        seed = safe_label(row.get("seed", ""))
        if scenario and seed:
            profile_label = ensure_profile(f"run_{scenario}_{seed}", row)
            label_profile_map[f"{scenario}:{seed}"] = profile_label
            label_profile_map[f"{scenario}_{seed}"] = profile_label

    evidence_dir = output_dir / "evidence"
    evidence_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(selector_summary_json, evidence_dir / "selector_portfolio_summary.json")
    shutil.copy2(selected_runs_csv, evidence_dir / "selector_portfolio_selected_runs.csv")

    profile_paths = {label: record["profile_json"] for label, record in sorted(profile_records.items())}
    package_paths = {label: record["package_dir"] for label, record in sorted(profile_records.items())}
    manifest = {
        "mode": "stereodistance_runtime_profile_label_selector",
        "baseline_selector": baseline_label,
        "label_profile_map": label_profile_map,
        "profile_paths": profile_paths,
        "package_paths": package_paths,
        "validated_evidence": {
            "selector_portfolio_summary": "evidence/selector_portfolio_summary.json",
            "selector_portfolio_selected_runs": "evidence/selector_portfolio_selected_runs.csv",
        },
        "selection_input_fields": ["deployment_label", "scenario_label", "scenario_name"],
        "forbidden_runtime_fields": FORBIDDEN_RUNTIME_FIELDS,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": SELECTION_CONTRACT,
        "selector_source_mode": selector_mode,
        "selector_objective": objective,
        "runtime_truth_used": False,
    }
    selector_json = output_dir / "runtime_profile_label_selector.json"
    write_json(selector_json, manifest)

    profile_rows = list(profile_records.values())
    label_rows = [
        {
            "deployment_label": label,
            "profile_label": profile_label,
            "selector_source_mode": selector_mode,
            "runtime_truth_used": False,
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        }
        for label, profile_label in sorted(label_profile_map.items())
    ]
    export_summary = {
        "mode": "common_csv_selector_package_export",
        "status": "PASS",
        "selector_json": str(selector_json),
        "selector_summary_json": str(selector_summary_json),
        "selected_runs_csv": str(selected_runs_csv),
        "selector_mode": selector_mode,
        "objective": objective,
        "portfolio_recommended_selector_mode": summary.get("recommended_deployable_selector_mode", ""),
        "baseline_profile_label": baseline_label,
        "scenario_label_count": len(scenario_rows),
        "deployment_label_count": len(label_profile_map),
        "profile_count": len(profile_records),
        "profile_rows_csv": str(output_dir / "selector_package_profiles.csv"),
        "label_rows_csv": str(output_dir / "selector_package_labels.csv"),
        "runtime_truth_used": False,
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": SELECTION_CONTRACT,
    }
    write_csv(output_dir / "selector_package_profiles.csv", profile_rows)
    write_csv(output_dir / "selector_package_labels.csv", label_rows)
    write_json(output_dir / "selector_package_export_summary.json", export_summary)
    write_text(output_dir / "selector_package_export_summary.txt", export_summary, profile_rows, label_rows)
    return export_summary


def write_text(
    path: Path,
    summary: Mapping[str, object],
    profile_rows: Sequence[Mapping[str, object]],
    label_rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "Common-CSV selector package export",
        f"status: {summary.get('status', '')}",
        f"selector_json: {summary.get('selector_json', '')}",
        f"selector_mode: {summary.get('selector_mode', '')}",
        f"objective: {summary.get('objective', '')}",
        f"baseline_profile_label: {summary.get('baseline_profile_label', '')}",
        f"scenario_label_count: {summary.get('scenario_label_count', '')}",
        f"deployment_label_count: {summary.get('deployment_label_count', '')}",
        f"profile_count: {summary.get('profile_count', '')}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', False)}",
        f"axis_contract: {summary.get('axis_contract', '')}",
        f"public_topic_contract: {summary.get('public_topic_contract', '')}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Profiles:",
    ]
    for row in profile_rows:
        lines.append(f"- {row.get('profile_label', '')}: method={row.get('method', '')}")
    lines.extend(["", "Labels:"])
    for row in label_rows:
        lines.append(f"- {row.get('deployment_label', '')} -> {row.get('profile_label', '')}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    summary = export_selector_package(
        selector_summary_json=args.selector_summary_json,
        selected_runs_csv=args.selected_runs_csv,
        output_dir=args.output_dir,
        selector_mode=args.selector_mode,
        objective=args.objective,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
