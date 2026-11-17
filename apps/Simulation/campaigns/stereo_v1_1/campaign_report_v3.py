#!/usr/bin/env python3
"""Build the v1.2 V3 report workbook from the completed campaign and analysis.

The exporter is report-only. It reads a frozen, audited analysis and immutable
run artifacts; V0 tracking is recomputed only after the freeze/audit gate. All
positions and errors in this workbook use world ENU [east,north,up] metres.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import sys
import tempfile
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM_ROOT))
from campaigns.stereo_v1_1.analysis_variant_comparison import NUMERIC_FIELDS as VARIANT_NUMERIC_FIELDS
from campaigns.stereo_v1_1.gt_alignment import GroundTruthSeries


SHEET_NAMES = (
    "00_요약", "01_실행현황", "02_경로추종", "03_관측품질", "04_교정비교",
    "05_그래프", "06_리뷰판정", "07_사후QC", "08_V0_MC분포", "09_GT제외프레임",
    "10_계산추적", "11_방법계약", "12_경로선정추적", "13_QC시간간격", "14_P95짝비교",
    "15_선정민감도", "16_실행별V0", "17_추종P95출처", "98_출처", "99_정의",
)
V0_START_NORTH_M = 600.0
V0_END_NORTH_M = 400.0
V0_GLIDE_ANGLE_DEG = 20.0
TARGET_NORTHINGS_M = [600, 550, 500, 450, 400]
MC_NUMERIC_FIELDS = {
    "trial_index", "detection_rate", "valid_measurement_rate", "lateral_east_rmse_m",
    "forward_north_rmse_m", "vertical_up_rmse_m", "position_3d_rmse_m", "position_3d_p95_m",
}
MAIN_METHODS = ("A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint")
MAIN_VARIANTS = ("RPY_IMU", "YAW_ONLY")
MAIN_SCOPES = ("all_data", "matched_200")
VALID_ATTEMPT_STATUSES = ("VALID", "VALID_WITH_WARNINGS")


def _json(path: Path) -> dict:
    def reject(value):
        raise ValueError(f"non-finite JSON constant {value} in {path}")
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject)


def _jsonl(path: Path):
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                yield json.loads(line)


def _csv_rows(path: Path):
    with path.open(encoding="utf-8-sig", newline="") as handle:
        yield from csv.DictReader(handle)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _verify_analysis_checksums(analysis_root: Path) -> Path:
    inventory_path = analysis_root / "analysis_checksums.sha256"
    if not inventory_path.is_file():
        raise ValueError("audited analysis is missing analysis_checksums.sha256")
    checksums = {}
    for line in inventory_path.read_text(encoding="utf-8").splitlines():
        try:
            digest, relative = line.split("  ", 1)
        except ValueError as exc:
            raise ValueError("malformed analysis checksum inventory") from exc
        if (not re.fullmatch(r"[0-9a-f]{64}", digest) or not relative
                or relative in checksums):
            raise ValueError("malformed or duplicate analysis checksum entry")
        path = _owned(analysis_root, relative)
        if not path.is_file() or _sha256(path) != digest:
            raise ValueError(f"analysis checksum mismatch: {relative}")
        checksums[relative] = digest
    actual = {
        path.relative_to(analysis_root).as_posix()
        for path in analysis_root.rglob("*")
        if path.is_file() and path.name != "analysis_checksums.sha256"
    }
    if actual != set(checksums):
        raise ValueError("analysis checksum inventory does not match the analysis directory")
    return inventory_path


def _owned(root: Path, relative: str | Path) -> Path:
    rel = Path(relative)
    path = (root / rel).resolve()
    if rel.is_absolute() or not path.is_relative_to(root.resolve()):
        raise ValueError(f"campaign artifact path escapes output root: {relative}")
    return path


def _number(value):
    if value in (None, ""):
        return None
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"non-finite metric: {value}")
    return result


def _first_valid_attempt(slot: dict) -> dict:
    attempts = slot.get("attempts")
    if not isinstance(attempts, list) or not attempts:
        raise ValueError(f"run {slot.get('run_id')} has no attempt records")
    for attempt in attempts:
        if not isinstance(attempt, dict):
            raise ValueError(f"run {slot.get('run_id')} contains a malformed attempt record")
        if attempt.get("run_id") not in (None, slot.get("run_id")):
            raise ValueError(f"run {slot.get('run_id')} contains an attempt from another run")
        if attempt.get("status") in VALID_ATTEMPT_STATUSES:
            if (slot.get("status") in VALID_ATTEMPT_STATUSES
                    and slot["status"] != attempt.get("status")):
                raise ValueError(f"run {slot.get('run_id')} status differs from its first valid attempt")
            if not isinstance(attempt.get("attempt_id"), str) or not attempt["attempt_id"]:
                raise ValueError(f"run {slot.get('run_id')} has a valid attempt without an attempt ID")
            if not isinstance(attempt.get("recording_path"), str) or not attempt["recording_path"]:
                raise ValueError(f"run {slot.get('run_id')} has a valid attempt without a recording path")
            return attempt
    raise ValueError(f"run {slot.get('run_id')} has no valid attempt")


def _rig_group_from_calibration_id(calibration_run_id: object, declared_group: object = None) -> str:
    if not isinstance(calibration_run_id, str):
        raise ValueError("comparison row is missing calibration_run_id")
    match = re.fullmatch(r"(G0|G1)-(P[0-4])-R(\d+)", calibration_run_id)
    if not match:
        raise ValueError(f"invalid calibration run identity in comparison row: {calibration_run_id}")
    rig_group = match.group(1)
    if declared_group not in (None, "", rig_group):
        raise ValueError(f"comparison rig_group conflicts with run ID: {calibration_run_id}")
    return rig_group


def _main_comparison_summary(metric_rows: list[dict], plan: dict, protocol_spec: dict,
                             trial_count: int, legacy_summary_rows: list[dict]) -> tuple[list[dict], bool]:
    planned_runs = plan.get("planned_runs", [])
    calibration_runs = [row for row in planned_runs if row.get("use_for_calibration")]
    evaluation_runs = [row for row in planned_runs if row.get("use_for_final_evaluation")]
    v0_by_group_repetition = {}
    for row in evaluation_runs:
        if row.get("path_id") != "V0":
            raise ValueError("final evaluation run plan contains a non-V0 path")
        key = (row.get("rig_group"), int(row.get("repetition", 0)))
        if key in v0_by_group_repetition:
            raise ValueError(f"duplicate V0 run for rig group/repetition: {key}")
        v0_by_group_repetition[key] = row.get("run_id")

    yaw_cases = {
        "G0": tuple(row["id"] for row in protocol_spec["reprocessing"]["yaw_bias_cases_deg"]),
        "G1": ("G1_PRIMARY_ZERO",),
    }
    calibration_ids_by_group = {group: set() for group in yaw_cases}
    for row in calibration_runs:
        run_id = row.get("run_id")
        group = _rig_group_from_calibration_id(run_id, row.get("rig_group"))
        calibration_ids_by_group[group].add(run_id)
    if not calibration_ids_by_group["G0"] or not calibration_ids_by_group["G1"]:
        raise ValueError("comparison requires calibration runs from both G0 and G1")

    expected_keys = {
        (group, scope, method, variant)
        for group in yaw_cases for scope in MAIN_SCOPES
        for method in MAIN_METHODS for variant in MAIN_VARIANTS
    }
    grouped = {
        key: {
            "cases": set(), "calibration_ids": set(), "evaluation_ids": set(), "trials": set(),
            "sums": {field: 0.0 for field in (
                "position_3d_rmse_m", "position_3d_p95_m", "detection_rate", "valid_measurement_rate",
            )},
            "counts": {field: 0 for field in (
                "position_3d_rmse_m", "position_3d_p95_m", "detection_rate", "valid_measurement_rate",
            )},
        }
        for key in expected_keys
    }
    for row in metric_rows:
        method, variant = row.get("method"), row.get("attitude_variant")
        if method not in MAIN_METHODS or variant not in MAIN_VARIANTS:
            continue
        calibration_id = row.get("calibration_run_id")
        rig_group = _rig_group_from_calibration_id(calibration_id, row.get("rig_group"))
        key = (rig_group, row.get("data_scope"), method, variant)
        if key not in grouped:
            raise ValueError(f"unexpected group/scope in main comparison: {key}")
        match = re.fullmatch(r"G[01]-P[0-4]-R(\d+)", calibration_id)
        repetition = int(match.group(1))
        evaluation_id = row.get("evaluation_run_id")
        expected_evaluation_id = v0_by_group_repetition.get((rig_group, repetition))
        if not expected_evaluation_id or evaluation_id != expected_evaluation_id:
            raise ValueError(f"comparison does not use the matching same-group V0 run: {calibration_id} → {evaluation_id}")
        yaw_case = row.get("yaw_case")
        if yaw_case not in yaw_cases[rig_group]:
            raise ValueError(f"unexpected yaw case for {rig_group}: {yaw_case}")
        try:
            trial_index = int(row.get("trial_index"))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid Monte Carlo trial index for {calibration_id}") from exc
        if not 0 <= trial_index < trial_count:
            raise ValueError(f"Monte Carlo trial index outside frozen range: {trial_index}")
        case_id = (calibration_id, evaluation_id, yaw_case, trial_index)
        group = grouped[key]
        if case_id in group["cases"]:
            raise ValueError(f"duplicate main-comparison case: {key} {case_id}")
        group["cases"].add(case_id)
        group["calibration_ids"].add(calibration_id)
        group["evaluation_ids"].add(evaluation_id)
        group["trials"].add(trial_index)
        for field, target in (
            ("position_3d_rmse_m", "position_3d_rmse_m"),
            ("position_3d_p95_m", "position_3d_p95_m"),
            ("detection_rate", "detection_rate"),
            ("valid_measurement_rate", "valid_measurement_rate"),
        ):
            value = _number(row.get(field))
            if value is not None:
                group["sums"][target] += value
                group["counts"][target] += 1

    rows = []
    for group in ("G0", "G1"):
        for scope in MAIN_SCOPES:
            for method in MAIN_METHODS:
                for variant in MAIN_VARIANTS:
                    key = (group, scope, method, variant)
                    result = grouped[key]
                    expected_cases = {
                        (calibration_id,
                         v0_by_group_repetition[(group, int(calibration_id.rsplit("R", 1)[1]))],
                         yaw_case, trial)
                        for calibration_id in calibration_ids_by_group[group]
                        for yaw_case in yaw_cases[group]
                        for trial in range(trial_count)
                    }
                    if result["cases"] != expected_cases:
                        missing = len(expected_cases - result["cases"])
                        extra = len(result["cases"] - expected_cases)
                        raise ValueError(f"incomplete main-comparison matrix for {key}: {missing} missing, {extra} unexpected cases")
                    item = {
                        "rig_group": group,
                        "data_scope": scope,
                        "method": method,
                        "attitude_variant": variant,
                        "evaluation_case_count": len(result["cases"]),
                        "source_calibration_run_count": len(result["calibration_ids"]),
                        "source_evaluation_run_count": len(result["evaluation_ids"]),
                        "offline_mc_trial_count": len(result["trials"]),
                        "metric_available_case_count": result["counts"]["position_3d_rmse_m"],
                    }
                    for field, target in (
                        ("position_3d_rmse_m", "position_3d_rmse_mean_m"),
                        ("position_3d_p95_m", "position_3d_p95_mean_m"),
                        ("detection_rate", "detection_rate_mean"),
                        ("valid_measurement_rate", "valid_measurement_rate_mean"),
                    ):
                        count = result["counts"][field]
                        item[target] = result["sums"][field] / count if count else None
                    rows.append(item)

    source_has_groups = ["rig_group" in row for row in legacy_summary_rows]
    if source_has_groups and any(source_has_groups) and not all(source_has_groups):
        raise ValueError("analysis summary mixes grouped and pooled comparison rows")
    if source_has_groups and all(source_has_groups):
        summary_by_key = {
            (row["rig_group"], row["data_scope"], row["method"], row["attitude_variant"]): row
            for row in legacy_summary_rows
        }
        if set(summary_by_key) != expected_keys:
            raise ValueError("grouped analysis summary does not contain the complete G0/G1 comparison matrix")
        for row in rows:
            key = (row["rig_group"], row["data_scope"], row["method"], row["attitude_variant"])
            source = summary_by_key[key]
            for field in (
                "evaluation_case_count", "metric_available_case_count", "position_3d_rmse_mean_m",
                "position_3d_p95_mean_m", "detection_rate_mean", "valid_measurement_rate_mean",
            ):
                expected, actual = _number(row.get(field)), _number(source.get(field))
                if expected is None or actual is None:
                    if expected != actual:
                        raise ValueError(f"analysis summary differs from raw comparison metrics for {key}: {field}")
                elif not math.isclose(expected, actual, rel_tol=1e-10, abs_tol=1e-12):
                    raise ValueError(f"analysis summary differs from raw comparison metrics for {key}: {field}")
    elif legacy_summary_rows:
        legacy_keys = {
            (row.get("data_scope"), row.get("method"), row.get("attitude_variant"))
            for row in legacy_summary_rows
        }
        expected_legacy_keys = {
            (scope, method, variant)
            for scope in MAIN_SCOPES for method in MAIN_METHODS for variant in MAIN_VARIANTS
        }
        if len(legacy_keys) != len(legacy_summary_rows) or legacy_keys != expected_legacy_keys:
            raise ValueError("legacy pooled analysis summary does not contain the full main comparison matrix")
        expected_total = sum(
            len(calibration_ids_by_group[group]) * len(yaw_cases[group]) * trial_count
            for group in yaw_cases
        )
        expected_source_calibrations = sum(len(ids) for ids in calibration_ids_by_group.values())
        if any(row.get("evaluation_case_count") != expected_total
               or row.get("source_calibration_run_count") != expected_source_calibrations
               or row.get("source_evaluation_run_count") != len(v0_by_group_repetition)
               or row.get("offline_mc_trial_count") != trial_count
               for row in legacy_summary_rows):
            raise ValueError("legacy pooled analysis summary counts differ from the frozen comparison matrix")
    return rows, not (source_has_groups and all(source_has_groups))


def _calibration_convergence_by_group(metric_rows: list[dict]) -> list[dict]:
    grouped = {}
    fit_statuses = ("CONVERGED", "OPTIMIZER_FAILED", "RANK_DEFICIENT")
    for row in metric_rows:
        rig_group = _rig_group_from_calibration_id(
            row.get("calibration_run_id"), row.get("rig_group"),
        )
        method, variant = row.get("method"), row.get("attitude_variant")
        if method not in MAIN_METHODS or variant not in MAIN_VARIANTS:
            continue
        key = (rig_group, method, variant)
        statuses = grouped.setdefault(key, {})
        status = row.get("convergence_status") or "UNKNOWN"
        statuses[status] = statuses.get(status, 0) + 1

    result = []
    for (rig_group, method, variant), statuses in sorted(grouped.items()):
        total = sum(statuses.values())
        attempted = sum(statuses.get(status, 0) for status in fit_statuses)
        nonconverged = statuses.get("OPTIMIZER_FAILED", 0) + statuses.get("RANK_DEFICIENT", 0)
        result.append({
            "rig_group": rig_group,
            "method": method,
            "attitude_variant": variant,
            "calibration_case_count": total,
            "attempted_fit_count": attempted,
            "nonconverged_fit_count": nonconverged,
            "not_run_or_other_status_count": total - attempted,
            "nonconvergence_fraction_among_attempted_fits": nonconverged / attempted if attempted else None,
            "failure_or_not_run_fraction_all_cases": (total - statuses.get("CONVERGED", 0)) / total if total else None,
        })
    return result


def _mc_cell(row: dict, key: str):
    if key not in MC_NUMERIC_FIELDS:
        return row.get(key)
    number = _number(row.get(key))
    if key == "trial_index" and number is not None:
        if not number.is_integer():
            raise ValueError(f"Monte Carlo trial index must be an integer: {number}")
        return int(number)
    return number


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    point = fraction * (len(ordered) - 1)
    lower, upper = math.floor(point), math.ceil(point)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (point - lower)


def _paired_variant_run_rows(rows: list[dict], metric: str) -> list[list[object]]:
    sums: dict[tuple[str, str, int, str], tuple[float, int]] = {}
    for row in rows:
        if (row.get("family") != "primary" or row.get("condition_kind") != "yaw_case"
                or row.get("condition_id") != "Y0" or row.get("data_scope") != "all_data"
                or row.get("comparison_status") != "COMPARABLE_CASE_METRIC"):
            continue
        value = _number(row.get(metric))
        if value is None:
            continue
        key = (row["rig_group"], row["path_id"], int(row["physical_repetition"]), row["method"])
        total, count = sums.get(key, (0.0, 0))
        sums[key] = total + value, count + 1
    return [
        [f"{group}-{path}-R{repetition:02d} / {method} · n={count}", total / count]
        for (group, path, repetition, method), (total, count) in sorted(sums.items())
    ]


def _postfreeze_tracking(attempt_dir: Path, freeze: dict) -> dict:
    """Recompute GT-to-reference tracking after the audited freeze in world ENU."""
    if (freeze.get("status") != "FROZEN_BEFORE_V0_POSITION_EVALUATION"
            or freeze.get("v0_position_error_read_before_freeze") is not False):
        raise ValueError("post-freeze tracking requires a valid pre-evaluation freeze")
    alignment_path = attempt_dir / "gt_alignment_summary.json"
    maximum_gap_s = float(_json(alignment_path).get("maximum_bracketing_gap_s", 0.04))
    series = GroundTruthSeries(
        _jsonl(attempt_dir / "ground_truth.jsonl"),
        max_bracketing_gap_ns=round(maximum_gap_s * 1_000_000_000),
    )
    errors: list[tuple[float, float, float]] = []
    total = 0
    for row in _jsonl(attempt_dir / "flight_state.jsonl"):
        if row.get("phase") != "science":
            continue
        total += 1
        reference = row.get("reference_position_world_enu_m")
        if not isinstance(reference, list) or len(reference) != 3:
            continue
        aligned = series.align(row["simulation_epoch"], row["simulation_timestamp_ns"])
        if aligned.status not in ("EXACT", "INTERPOLATED") or aligned.position_world_enu_m is None:
            continue
        errors.append(tuple(aligned.position_world_enu_m[i] - float(reference[i]) for i in range(3)))
    return _tracking_summary(
        errors, total,
        "raw GT interpolated at science control timestamps after analysis freeze",
    )


def _recorded_tracking(attempt_dir: Path) -> dict:
    """Summarize saved flight-state GT−reference values with a 40 ms time gate."""
    errors: list[tuple[float, float, float]] = []
    total = 0
    for row in _jsonl(attempt_dir / "flight_state.jsonl"):
        if row.get("phase") != "science":
            continue
        total += 1
        reference = row.get("reference_position_world_enu_m")
        ground_truth = row.get("gt_reference_position_world_enu_m")
        simulation_stamp = row.get("simulation_timestamp_ns")
        ground_truth_stamp = row.get("gt_timestamp_ns")
        if (not isinstance(reference, list) or len(reference) != 3
                or not isinstance(ground_truth, list) or len(ground_truth) != 3
                or simulation_stamp is None or ground_truth_stamp is None
                or abs(int(ground_truth_stamp) - int(simulation_stamp)) > 40_000_000):
            continue
        errors.append(tuple(float(ground_truth[i]) - float(reference[i]) for i in range(3)))
    return _tracking_summary(
        errors, total,
        "saved flight-state world ENU GT−reference; |GT time offset| ≤40 ms",
    )


def _tracking_summary(errors: list[tuple[float, float, float]], total: int, method: str) -> dict:
    norms = [math.sqrt(sum(component * component for component in row)) for row in errors]
    return {
        "science_control_rows": total,
        "valid_gt_aligned_rows": len(errors),
        "position_3d_rmse_m": math.sqrt(sum(value * value for value in norms) / len(norms)) if norms else None,
        "position_3d_p95_m": _percentile(norms, 0.95),
        "input_frame": "gazebo_world_enu:east,north,up_m",
        "method": method,
    }


def _observation_counts(attempt_dir: Path) -> dict:
    counts = {}
    for side in ("left", "right"):
        side_counts = {name: 0 for name in ("DETECTED", "NOT_DETECTED", "SKIPPED_RATE_LIMIT", "INFERENCE_ERROR")}
        path = attempt_dir / f"observations_{side}.jsonl"
        if path.is_file():
            for row in _jsonl(path):
                status = row.get("detection_status", "UNKNOWN")
                side_counts[status] = side_counts.get(status, 0) + 1
        counts[side] = side_counts
    return counts


def _load_campaign(output_root: Path, audit_report: Path):
    status_path = output_root / "run_status.json"
    plan_path = output_root / "workbook_campaign_plan.json"
    latest_path = output_root / "analysis_latest.json"
    status, plan, latest, audit = map(_json, (status_path, plan_path, latest_path, audit_report))
    if status.get("planned_run_count") != 39 or status.get("valid_run_count") != 39:
        raise ValueError("V3 report requires all 39 source-QC-valid campaign runs")
    slots = status.get("run_slots")
    if (not isinstance(slots, list) or len(slots) != 39
            or any(not isinstance(slot, dict)
                   or slot.get("status") not in VALID_ATTEMPT_STATUSES for slot in slots)):
        raise ValueError("V3 report requires all 39 campaign slots to have source-QC-valid results")
    for slot in slots:
        _first_valid_attempt(slot)
    if plan.get("campaign", {}).get("protocol_id") != "stereo_calibration_campaign_v1_2":
        raise ValueError("V3 report requires the v1.2 workbook campaign")
    if latest.get("analysis_id") != audit.get("analysis_id") or audit.get("status") != "PASS":
        raise ValueError("V3 report requires a PASS audit for the latest analysis")
    summary_path = _owned(output_root, latest["analysis_summary"])
    analysis_root = summary_path.parent
    if analysis_root.name != latest["analysis_id"] or summary_path.name != "analysis_summary.json":
        raise ValueError("latest-analysis pointer does not identify its version directory")
    _verify_analysis_checksums(analysis_root)
    freeze_path = analysis_root / "analysis_freeze.json"
    if _sha256(freeze_path) != latest.get("analysis_freeze_sha256"):
        raise ValueError("latest-analysis freeze hash mismatch")
    summary, freeze = _json(summary_path), _json(freeze_path)
    if (summary.get("analysis_id") != latest["analysis_id"]
            or summary.get("analysis_freeze_sha256") != latest["analysis_freeze_sha256"]
            or summary.get("mc_trial_count") != 100
            or freeze.get("mc_trial_count") != 100
            or summary.get("v0_position_errors_evaluated_after_freeze") is not True
            or summary.get("v0_recalibration_performed") is not False):
        raise ValueError("analysis summary/freeze contract mismatch; V3 requires MC100")
    if (freeze.get("status") != "FROZEN_BEFORE_V0_POSITION_EVALUATION"
            or freeze.get("v0_position_error_read_before_freeze") is not False):
        raise ValueError("invalid analysis freeze contract")
    selection_path = analysis_root / "path_selection.json"
    if freeze.get("path_selection_sha256") != _sha256(selection_path):
        raise ValueError("frozen path selection hash mismatch")
    selection = _json(selection_path)
    selected_ids = {
        freeze.get("selected_path_id"), summary.get("selected_path_id"), selection.get("selected_path_id"),
    }
    if len(selected_ids) != 1 or None in selected_ids:
        raise ValueError("frozen, summary and path-selection IDs do not agree")
    if next(iter(selected_ids)) not in {f"P{index}" for index in range(5)}:
        raise ValueError("selected path ID is outside the v1.2 P0–P4 set")
    geometry = selection.get("target_geometry", {})
    if (geometry.get("frame") != "world_enu:east,north,up_m"
            or geometry.get("distances_m") != TARGET_NORTHINGS_M):
        raise ValueError("frozen V0 target geometry is not world ENU north 600→400 m")
    v0_rows = [row for row in plan["planned_runs"] if row.get("use_for_final_evaluation")]
    if not v0_rows or any(
        float(row["parameters"]["Path"]["north_start_m"]) != V0_START_NORTH_M
        or float(row["parameters"]["Path"]["north_end_m"]) != V0_END_NORTH_M
        or float(row["parameters"]["Path"]["glide_angle_deg"]) != V0_GLIDE_ANGLE_DEG
        for row in v0_rows
    ):
        raise ValueError("frozen workbook V0 path must start at N=600 m and end at N=400 m")
    protocol_spec_path = _owned(output_root, "frozen_protocol/v1_2_specification.json")
    protocol_spec = _json(protocol_spec_path)
    analysis_manifest = _json(analysis_root / "analysis_run_manifest.json")
    protocol_bindings = analysis_manifest.get("source_campaign_manifest_sha256", {})
    required_bindings = {
        "run_status.json", "workbook_campaign_plan.json",
        "frozen_protocol/v1_2_specification.json",
    }
    if not isinstance(protocol_bindings, dict) or not required_bindings.issubset(protocol_bindings):
        raise ValueError("analysis manifest does not bind the frozen campaign status, plan and protocol")
    for relative, expected_hash in protocol_bindings.items():
        source_path = _owned(output_root, relative)
        if not source_path.is_file() or _sha256(source_path) != expected_hash:
            raise ValueError(f"campaign source manifest hash mismatch: {relative}")
    if analysis_manifest.get("analysis_id") != latest["analysis_id"]:
        raise ValueError("analysis run manifest ID differs from latest-analysis pointer")
    expected_attempt_checksums = analysis_manifest.get("source_attempt_checksums_sha256", {})
    selected_attempt_keys = set()
    for slot in slots:
        attempt = _first_valid_attempt(slot)
        attempt_key = f"{slot['run_id']}/{attempt['attempt_id']}"
        selected_attempt_keys.add(attempt_key)
        checksum_path = _owned(output_root, attempt["recording_path"]) / "checksums.sha256"
        if (attempt_key not in expected_attempt_checksums or not checksum_path.is_file()
                or _sha256(checksum_path) != expected_attempt_checksums[attempt_key]):
            raise ValueError(f"selected source attempt differs from audited analysis: {attempt_key}")
    if set(expected_attempt_checksums) != selected_attempt_keys:
        raise ValueError("analysis attempt checksum inventory differs from selected campaign attempts")
    if (protocol_spec.get("document_version") != "1.2"
            or protocol_spec.get("schema_version") != "stereo_calibration_campaign_v1_2"
            or protocol_bindings.get("frozen_protocol/v1_2_specification.json") != _sha256(protocol_spec_path)):
        raise ValueError("frozen v1.2 specification does not match the audited analysis source")
    return (status, plan, latest, audit, summary_path, analysis_root, summary,
            freeze, selection, protocol_spec, protocol_spec_path)


def _write_table(sheet, workbook, headers, rows, *, widths=None, freeze=1, start_row=0, autofilter=True):
    header = workbook.add_format({"bold": True, "font_color": "white", "bg_color": "#253B56", "text_wrap": True, "valign": "vcenter"})
    body = workbook.add_format({"valign": "top"})
    number = workbook.add_format({"num_format": "0.000000", "valign": "top"})
    sheet.freeze_panes(freeze, 0)
    sheet.write_row(start_row, 0, headers, header)
    last_row = start_row
    for row_index, row in enumerate(rows, start_row + 1):
        last_row = row_index
        for col_index, value in enumerate(row):
            if isinstance(value, float) and not math.isfinite(value):
                value = None
            fmt = number if isinstance(value, float) else body
            if value is None:
                sheet.write_blank(row_index, col_index, None, body)
            elif isinstance(value, (dict, list, tuple)):
                sheet.write(row_index, col_index, json.dumps(value, ensure_ascii=False, sort_keys=True), body)
            else:
                sheet.write(row_index, col_index, value, fmt)
    if autofilter and last_row > start_row and headers:
        sheet.autofilter(start_row, 0, last_row, len(headers) - 1)
    for index, width in enumerate(widths or [22] * len(headers)):
        sheet.set_column(index, index, width)
    sheet.set_row(start_row, 34)


def _table_sheet(workbook, name, headers, rows, widths=None, *, start_row=0, freeze=1, autofilter=True):
    sheet = workbook.get_worksheet_by_name(name)
    if sheet is None:
        sheet = workbook.add_worksheet(name)
    _write_table(sheet, workbook, headers, rows, widths=widths, start_row=start_row,
                 freeze=freeze, autofilter=autofilter)
    return sheet


def build_report(output_root: Path, audit_report: Path, output_path: Path | None = None) -> dict:
    output_root, audit_report = output_root.expanduser().resolve(), audit_report.expanduser().resolve()
    (status, plan, latest, audit, summary_path, analysis_root,
     summary, freeze, selection, protocol_spec, protocol_spec_path) = _load_campaign(output_root, audit_report)
    output_path = output_path or output_root / "reports" / "stereo_v1_2_summary_report_ready_v3.xlsx"
    output_path = output_path.expanduser().resolve()
    reports_root = output_root / "reports"
    if not output_path.is_relative_to(reports_root.resolve()):
        raise ValueError("V3 XLSX output must remain under the campaign reports directory")
    reports_root.mkdir(parents=True, exist_ok=True)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    slots = status["run_slots"]
    run_context = []
    for slot in slots:
        attempt = _first_valid_attempt(slot)
        attempt_dir = _owned(output_root, attempt["recording_path"])
        qc_path, alignment_path = attempt_dir / "qc_report.json", attempt_dir / "gt_alignment_summary.json"
        qc, alignment = _json(qc_path), _json(alignment_path)
        if (qc.get("run_id") != slot.get("run_id")
                or qc.get("attempt_id") != attempt.get("attempt_id")
                or qc.get("status") != attempt.get("status")):
            raise ValueError(f"source QC identity/status differs from selected attempt: {slot.get('run_id')}")
        report_slot = {
            **slot,
            "attempt_id": attempt["attempt_id"],
            "status": attempt["status"],
            "recording_path": attempt["recording_path"],
        }
        image_summary_path = attempt_dir / "image_index_summary.json"
        image_summary = _json(image_summary_path) if image_summary_path.is_file() else {}
        controller_path = attempt_dir / "controller_result.json"
        controller = _json(controller_path) if controller_path.is_file() else {}
        evidence = qc.get("evidence", {})
        capture = evidence.get("v12_science_capture", {})
        cadence = evidence.get("science_setpoint_cadence_recomputed", {})
        run_context.append({
            "slot": report_slot, "attempt": attempt, "attempt_dir": attempt_dir,
            "qc": qc, "qc_path": qc_path, "alignment": alignment,
            "alignment_path": alignment_path, "image_summary": image_summary,
            "image_summary_path": image_summary_path,
            "controller": controller, "capture": capture, "cadence": cadence,
            "recorded_tracking": _recorded_tracking(attempt_dir),
            "controller_path": controller_path,
            "observation_paths": [attempt_dir / f"observations_{side}.jsonl" for side in ("left", "right")],
            "postfreeze_paths": [attempt_dir / "ground_truth.jsonl", attempt_dir / "flight_state.jsonl"],
            "observations": _observation_counts(attempt_dir),
        })

    metric_rows = list(_csv_rows(analysis_root / "evaluation_metrics.csv"))
    calibration_metric_rows = list(_csv_rows(analysis_root / "calibration_metrics.csv"))
    variant_rows = list(_csv_rows(analysis_root / "attitude_variant_comparisons.csv"))
    path_predictions = selection.get("per_calibration_prediction", [])
    geometry = selection.get("target_geometry", {})
    path_order = ["P0", "P1", "P2", "P3", "P4"]
    candidate_summary = selection.get("candidate_summary", [])
    if not isinstance(candidate_summary, list) or not candidate_summary:
        raise ValueError("V3 path selection requires covariance scores for all candidate paths")
    candidate_by_path = {row.get("path_id"): row for row in candidate_summary}
    selected_path_id = selection.get("selected_path_id")
    if set(candidate_by_path) != set(path_order) or len(candidate_by_path) != len(candidate_summary):
        raise ValueError("V3 path selection must include exactly one covariance summary for paths P0–P4")
    try:
        score_order = sorted(
            path_order,
            key=lambda path_id: (
                _number(candidate_by_path[path_id].get("predicted_position_uncertainty_mean_sigma_m")),
                _number(candidate_by_path[path_id].get("reference_duration_mean_s")),
                path_id,
            ),
        )
    except (TypeError, ValueError) as exc:
        raise ValueError("V3 path selection contains an invalid score or reference duration") from exc
    if score_order[0] != selected_path_id:
        raise ValueError("frozen selected path does not match the covariance score and tie-break order")
    runner_up_path_id = score_order[1]
    reference_duration_by_path = {
        path_id: _number(candidate_by_path[path_id].get("reference_duration_mean_s"))
        for path_id in path_order
    }
    trial_count = int(freeze["mc_trial_count"])
    comparison_rows, legacy_pooled_summary_reaggregated = _main_comparison_summary(
        metric_rows, plan, protocol_spec, trial_count,
        summary.get("main_comparison_summary", []),
    )

    report_dir = Path(tempfile.mkdtemp(prefix=".report-v3-", dir=reports_root))
    temporary_xlsx = report_dir / output_path.name
    workbook = None
    try:
        import xlsxwriter
        workbook = xlsxwriter.Workbook(str(temporary_xlsx), {
            "constant_memory": True,
            "strings_to_formulas": False,
            "strings_to_urls": False,
        })
        workbook.set_properties({
            "title": "Stereo v1.2 campaign results · V3",
            "subject": "Audited simulation and offline analysis report",
            "author": "SASL Simulation",
        })
        title_fmt = workbook.add_format({"bold": True, "font_size": 16, "font_color": "#253B56"})
        report_header_fmt = workbook.add_format({"bold": True, "font_color": "white", "bg_color": "#253B56", "text_wrap": True, "valign": "vcenter"})
        report_note_fmt = workbook.add_format({"bg_color": "#DDF1EC", "text_wrap": True, "valign": "vcenter"})
        for name in SHEET_NAMES:
            workbook.add_worksheet(name)

        # The 20-sheet order and V3-specific tabs follow the reviewed V3 contract.
        summary_sheet = workbook.get_worksheet_by_name(SHEET_NAMES[0])
        summary_sheet.write("A1", "Stereo v1.2 · V3 campaign report", title_fmt)
        summary_pairs = [
            ("Campaign", plan["campaign"].get("campaign_id")),
            ("Protocol", plan["campaign"].get("protocol_id")),
            ("Analysis ID", latest["analysis_id"]),
            ("Selected path", selected_path_id),
            ("Source-valid runs", status.get("valid_run_count")),
            ("VALID_WITH_WARNINGS runs", sum(slot.get("status") == "VALID_WITH_WARNINGS" for slot in slots)),
            ("Planned runs", status.get("planned_run_count")),
            ("Monte Carlo trials", trial_count),
            ("Analysis audit", f"{audit['status']} · coverage/schema/provenance; no numeric accuracy threshold"),
            ("V0 science reference", "world ENU [E,N,U]=[0,600,218.382]→[0,400,145.588] m"),
            ("V0 start contract", "N=600 m is the planned science-window start; actual arrival is reported per run"),
            ("Main comparison groups", "G0 is the primary comparison; G1 nominal-zero results are reported separately and never pooled"),
            ("Freeze", freeze.get("status")),
            ("Accuracy gate", "No numeric accuracy acceptance threshold is defined by the protocol"),
            ("Evidence boundary", protocol_spec["split_policy"]["limitation"]),
        ]
        summary_sheet.write_row("A3", ["Item", "Value"], workbook.add_format({"bold": True, "bg_color": "#DCE6F1"}))
        for index, pair in enumerate(summary_pairs, 3):
            summary_sheet.write_row(index, 0, pair)
        summary_sheet.set_column("A:A", 28)
        summary_sheet.set_column("B:B", 105)
        summary_sheet.freeze_panes(3, 0)

        run_rows = []
        tracking_rows = []
        observation_rows = []
        qc_rows = []
        cadence_rows = []
        gt_excluded_rows = []
        p95_source_rows = []
        postfreeze_by_run = {}
        for ctx in run_context:
            slot, attempt, attempt_dir = ctx["slot"], ctx["attempt"], ctx["attempt_dir"]
            qc, alignment = ctx["qc"], ctx["alignment"]
            run_id = slot["run_id"]
            attempt_id = attempt["attempt_id"]
            tracking = ctx["recorded_tracking"]
            capture = ctx["capture"]
            cameras = capture.get("cameras", {})
            cadence = ctx["cadence"]
            group, path_id = slot.get("rig_group"), slot.get("path_id")
            run_rows.append([
                slot.get("sequence"), run_id, attempt_id, group, path_id, slot.get("repetition"),
                slot.get("role"), slot.get("data_split"), slot.get("status"),
                slot.get("attempt_count"), slot.get("recording_path"),
                ctx["controller"].get("controller_status"),
            ])
            tracking_rows.append([
                run_id, attempt_id, group, path_id, tracking.get("science_control_rows"),
                tracking.get("valid_gt_aligned_rows"), tracking.get("position_3d_rmse_m"),
                tracking.get("position_3d_p95_m"), alignment.get("tracking_metrics_revealed"),
                "world ENU [east,north,up] m",
                "Flight-state tracking is recomputed after audit; V0 QC metric stays hidden before freeze",
            ])
            left, right = ctx["observations"]["left"], ctx["observations"]["right"]
            observation_rows.append([
                run_id, ctx["image_summary"].get("left", {}).get("frame_count"),
                ctx["image_summary"].get("right", {}).get("frame_count"),
                ctx["image_summary"].get("pair_count"),
                left.get("DETECTED", 0), right.get("DETECTED", 0),
                left.get("NOT_DETECTED", 0), right.get("NOT_DETECTED", 0),
                left.get("SKIPPED_RATE_LIMIT", 0), right.get("SKIPPED_RATE_LIMIT", 0),
                left.get("INFERENCE_ERROR", 0), right.get("INFERENCE_ERROR", 0),
            ])
            qc_rows.append([
                run_id, attempt_id, qc.get("status"), len(qc.get("infrastructure_errors", [])),
                len(qc.get("protocol_errors", [])), len(qc.get("warnings", [])),
                json.dumps(qc.get("infrastructure_errors", []), ensure_ascii=False),
                json.dumps(qc.get("protocol_errors", []), ensure_ascii=False),
                json.dumps(qc.get("warnings", []), ensure_ascii=False),
                _sha256(ctx["qc_path"]), _sha256(ctx["alignment_path"]),
            ])
            observed = [cameras.get(side, {}) for side in ("left", "right")]
            cadence_rows.append([
                run_id, capture.get("status"), capture.get("start_simulation_ns"), capture.get("end_simulation_ns"),
                observed[0].get("unique_frame_count"), observed[0].get("observed_window_hz"), observed[0].get("max_gap_ns"),
                observed[1].get("unique_frame_count"), observed[1].get("observed_window_hz"), observed[1].get("max_gap_ns"),
                cadence.get("required_hz"), cadence.get("expected_full_reference_slots"),
                cadence.get("covered_full_reference_slots"), cadence.get("missing_full_slot_count"),
                cadence.get("max_simulation_gap_s"), cadence.get("science_window_duration_s"),
                cadence.get("sufficient_unique_simulation_cadence"),
            ])
            image_all = alignment.get("image_count_by_alignment_status", {})
            image_science = alignment.get("science_image_count_by_alignment_status", {})
            for state in sorted(set(image_all) | set(image_science)):
                if state not in ("EXACT", "INTERPOLATED"):
                    gt_excluded_rows.append([run_id, attempt_id, state, image_all.get(state, 0), image_science.get(state, 0)])

            # This truth-derived tracking comparison runs only after freeze + audit.
            recomputed = _postfreeze_tracking(attempt_dir, freeze)
            postfreeze_by_run[run_id] = recomputed
            recorded_p95 = _number(tracking.get("position_3d_p95_m"))
            recomputed_p95 = _number(recomputed.get("position_3d_p95_m"))
            p95_delta = (recorded_p95 - recomputed_p95
                         if recorded_p95 is not None and recomputed_p95 is not None else None)
            p95_source_rows.append([
                run_id, attempt_id, group, path_id, recorded_p95,
                tracking.get("valid_gt_aligned_rows"), recomputed_p95,
                recomputed.get("valid_gt_aligned_rows"),
                p95_delta,
                "미제공" if recorded_p95 is None else ("예" if recorded_p95 > 1 else "아니오"),
                "미제공" if recomputed_p95 is None else ("예" if recomputed_p95 > 1 else "아니오"),
                f"{slot['recording_path']}/flight_state.jsonl (기록값); {slot['recording_path']}/ground_truth.jsonl (재정렬값)",
                "개별 SHA-256은 98_출처 참조",
                abs(p95_delta) if p95_delta is not None else None,
            ])

        _table_sheet(workbook, SHEET_NAMES[1], ["sequence", "run_id", "attempt", "group", "path", "repetition", "role", "split", "status", "attempt_count", "recording_path", "controller_status"], run_rows,
                     [10, 20, 12, 10, 10, 12, 20, 14, 20, 14, 55, 24])
        _table_sheet(workbook, SHEET_NAMES[2], ["run_id", "attempt", "group", "path", "science rows", "valid GT rows", "recorded RMSE ENU 3D (m)", "recorded P95 ENU 3D (m)", "revealed before freeze", "frame", "note"], tracking_rows,
                     [20, 12, 10, 10, 16, 16, 22, 22, 20, 36, 55])
        _table_sheet(workbook, SHEET_NAMES[3], ["run_id", "left indexed frames", "right indexed frames", "pairs", "left detected", "right detected", "left not detected", "right not detected", "left skipped", "right skipped", "left inference errors", "right inference errors"], observation_rows,
                     [20, 20, 20, 14, 16, 16, 20, 20, 16, 16, 20, 20])

        comparison_headers = ["rig_group", "scope", "method", "attitude_variant", "evaluation_case_count", "metric_available_case_count", "position_3d_rmse_mean_m", "position_3d_p95_mean_m", "detection_rate_mean", "valid_measurement_rate_mean"]
        comparison_numeric_fields = {
            "evaluation_case_count", "metric_available_case_count", "position_3d_rmse_mean_m",
            "position_3d_p95_mean_m", "detection_rate_mean", "valid_measurement_rate_mean",
        }
        comparison_scopes = [row.get("data_scope") for row in comparison_rows]
        if any(scope not in ("all_data", "matched_200") for scope in comparison_scopes):
            raise ValueError("V3 main comparison contains a missing or unknown data scope")
        comparison_data = [[
            _number(row.get("data_scope" if key == "scope" else key))
            if key in comparison_numeric_fields else row.get("data_scope" if key == "scope" else key)
            for key in comparison_headers
        ] for row in comparison_rows]
        comparison_sheet = workbook.get_worksheet_by_name(SHEET_NAMES[4])
        comparison_sheet.merge_range(
            "A1:J1",
            "G0: 주 비교 · G1: nominal-zero 별도 확인. 실험군은 합산하지 않았습니다.",
            report_note_fmt,
        )
        _table_sheet(workbook, SHEET_NAMES[4], comparison_headers, comparison_data,
                     [12, 18, 28, 20, 22, 25, 24, 24, 24, 28], start_row=2, freeze=3)

        _table_sheet(workbook, SHEET_NAMES[6], ["path", "calibration rows", "median predicted sigma (m)", "selected by freeze"], [
            [path_id,
             sum(1 for row in path_predictions if row.get("path_id") == path_id),
             (lambda values: (sorted(values)[len(values) // 2] if len(values) % 2 else
                              (sorted(values)[len(values) // 2 - 1] + sorted(values)[len(values) // 2]) / 2)
              if values else None)([
                 float(row["predicted_position_uncertainty_mean_sigma_m"])
                 for row in path_predictions if row.get("path_id") == path_id
                 and row.get("predicted_position_uncertainty_mean_sigma_m") is not None
             ]),
             path_id == selection.get("selected_path_id")]
            for path_id in path_order
        ], [16, 22, 30, 20])
        _table_sheet(workbook, SHEET_NAMES[7], ["run_id", "attempt", "QC status", "infrastructure error count", "protocol error count", "warning count", "infrastructure errors", "protocol errors", "warnings", "QC SHA-256", "GT alignment SHA-256"], qc_rows,
                     [20, 12, 22, 22, 20, 16, 60, 60, 60, 65, 65])
        mc_headers = ["rig_group", "calibration_run_id", "evaluation_run_id", "trial_index", "yaw_case", "data_scope", "method", "attitude_variant", "evaluation_status", "detection_rate", "valid_measurement_rate", "lateral_east_rmse_m", "forward_north_rmse_m", "vertical_up_rmse_m", "position_3d_rmse_m", "position_3d_p95_m"]
        mc_rows = (
            {**row, "rig_group": row.get("rig_group") or _rig_group_from_calibration_id(row.get("calibration_run_id"))}
            for row in metric_rows if "-V0-" in row.get("evaluation_run_id", "")
        )
        mc_sheet = workbook.get_worksheet_by_name(SHEET_NAMES[8])
        _write_table(mc_sheet, workbook, mc_headers,
                     ([_mc_cell(row, key) for key in mc_headers] for row in mc_rows),
                     widths=[12, 22, 22, 12, 20, 18, 28, 20, 28, 18, 22, 22, 24, 22, 22, 22])

        _table_sheet(workbook, SHEET_NAMES[9], ["run_id", "attempt", "GT alignment status", "all image count", "science image count"], gt_excluded_rows,
                     [20, 12, 28, 20, 22])
        trace_rows = [
            ["Source-QC status", "39/39 valid source runs", "run_status.json + workbook_campaign_plan.json", "Campaign audit and frozen run plan", "No failed or warning run is omitted"],
            ["Path selection", selection.get("selected_path_id"), "path_selection.json", "G0 trial-0 covariance only; V0 position metrics not opened before freeze", "Not an accuracy pass or global-optimality claim"],
            ["G0/G1 comparison", "Separate group aggregates", "evaluation_metrics.csv + frozen v1.2 specification", "G0 Y0–Y6 primary; G1 G1_PRIMARY_ZERO separate", "Different rig groups are never pooled"],
            ["V0 RMSE/P95", "Mean over available Monte Carlo evaluation cases", "evaluation_metrics.csv", "Estimate minus truth in world ENU [east,north,up] m", "MC cases are not independent flights"],
            ["Tracking P95", "Type-7 percentile over science control rows", "gt_alignment_summary.json; post-freeze recomputation", "GT minus reference in world ENU [east,north,up] m", "Post-freeze tracking is distinct from estimator accuracy"],
            ["V0 reference", "[E,N,U]=[0,600,218.382]→[0,400,145.588] m", "frozen workbook Path parameters", "20° glide; science start is planned target after entry/stabilization", "Per-run actual arrival is not inferred from plan"],
            ["Audit", "PASS", str(audit_report.relative_to(output_root)), audit.get("scope"), "Coverage/schema/provenance audit only; no numeric accuracy threshold"],
        ]
        _table_sheet(workbook, SHEET_NAMES[10], ["measure", "reported value", "source", "calculation/frame contract", "interpretation boundary"], trace_rows,
                     [24, 46, 72, 80, 75])
        method_rows = [
            ["A_rtk_independent", "Independent left/right camera calibration", "camera RTK prior propagated into image covariance", "world ENU position error; optical RDF rays"],
            ["B_rtk_joint", "Joint left/right calibration", "shared world ENU point with RTK and camera IMU priors", "world ENU position error; optical RDF rays"],
            ["C_no_drone_rtk_joint", "Joint calibration without drone RTK", "image and camera IMU inputs; rank failures retained", "world ENU position error; optical RDF rays"],
            ["RPY_IMU", "Estimate roll, pitch, yaw", "measured camera IMU roll/pitch priors", "rad/rad² internally; do not relabel PX4 FRD as optical RDF"],
            ["YAW_ONLY", "Estimate yaw; hold measured roll/pitch", "same measured IMU roll/pitch observations", "diagnostic paired comparison"],
            ["V0", "Frozen calibration evaluated against V0", "V0 position truth is post-freeze evaluation only", "not used to re-fit the calibration or choose path"],
        ]
        _table_sheet(workbook, SHEET_NAMES[11], ["method/variant", "purpose", "input contract", "frame/interpretation"], method_rows,
                     [32, 48, 72, 76])

        selection_rows = [[
            row.get("path_id"), row.get("repetition"), row.get("method"), row.get("attitude_variant"),
            row.get("predicted_position_uncertainty_mean_sigma_m"),
            row.get("predicted_position_uncertainty_p95_sigma_m"),
            row.get("target_geometry", {}).get("frame") if isinstance(row.get("target_geometry"), dict) else geometry.get("frame"),
        ] for row in path_predictions]
        _table_sheet(workbook, SHEET_NAMES[12], ["path", "repetition", "method", "attitude", "mean target sigma (m)", "P95 target sigma (m)", "target frame"], selection_rows,
                     [14, 14, 30, 20, 25, 24, 42])
        cadence_sheet = _table_sheet(workbook, SHEET_NAMES[13], ["run_id", "capture status", "science start ns", "science end ns", "left frames", "left observed Hz", "left max gap ns", "right frames", "right observed Hz", "right max gap ns", "required setpoint Hz", "expected slots", "covered slots", "missing slots", "max simulation gap s", "science duration s", "50Hz cadence pass"], cadence_rows,
                                     [20, 18, 22, 22, 16, 18, 22, 16, 18, 22, 22, 18, 18, 18, 24, 22, 22])
        variant_headers = list(variant_rows[0]) if variant_rows else ["comparison_status"]
        variant_data = [[_number(row.get(key)) if key in VARIANT_NUMERIC_FIELDS else row.get(key)
                         for key in variant_headers] for row in variant_rows]
        _table_sheet(workbook, SHEET_NAMES[14], variant_headers, variant_data,
                     [max(16, min(32, len(header) + 4)) for header in variant_headers])

        # Leave-one-repetition-out sensitivity applies the original median rule to 12 remaining method×run values.
        loo_rows = []
        loo_winners = []
        for omit in range(1, 6):
            scores = {}
            for path_id in path_order:
                values = [float(row["predicted_position_uncertainty_mean_sigma_m"])
                          for row in path_predictions
                          if row.get("path_id") == path_id and int(row.get("repetition", -1)) != omit
                          and row.get("predicted_position_uncertainty_mean_sigma_m") is not None]
                if values:
                    values.sort()
                    middle = len(values) // 2
                    scores[path_id] = values[middle] if len(values) % 2 else (values[middle - 1] + values[middle]) / 2
            winner = min(
                scores,
                key=lambda path_id: (
                    scores[path_id],
                    reference_duration_by_path[path_id]
                    if reference_duration_by_path[path_id] is not None else math.inf,
                    path_id,
                ),
            ) if scores else None
            loo_winners.append(winner)
            ordered_scores = sorted(scores.values())
            loo_rows.append([f"R0{omit}", *[scores.get(path_id) for path_id in path_order], winner,
                             scores[selected_path_id] - scores[runner_up_path_id]
                             if selected_path_id in scores and runner_up_path_id in scores else None,
                             ordered_scores[1] - ordered_scores[0] if len(ordered_scores) > 1 else None])
        loo_sheet = workbook.get_worksheet_by_name(SHEET_NAMES[15])
        loo_sheet.merge_range(
            "A1:I1",
            f"동결 선택 {selected_path_id} · 공분산 점수 2위 {runner_up_path_id} · 반복 제외 민감도",
            title_fmt,
        )
        loo_sheet.merge_range(
            "A2:I2",
            f"동결된 G0/Y0/all_data/trial 0 예측 75개에서 같은 반복번호의 A/B/C 3개 값을 경로마다 제외하고 중앙값 점수(m)를 다시 계산합니다. 점수 동률은 참조시간, 경로 ID 순으로 정리합니다. 사후 진단이며 동결 선택 {selected_path_id}를 변경하지 않습니다.",
            report_note_fmt,
        )
        _write_table(loo_sheet, workbook,
                     ["제외 반복", *path_order, "남은 자료의 1위", f"{selected_path_id}−{runner_up_path_id} (m)", "1위−2위 간격 (m)"],
                     loo_rows, widths=[19, 20, 20, 20, 20, 20, 22, 23, 24],
                     freeze=4, start_row=3, autofilter=False)
        loo_sheet.write_row(
            11, 0,
            [f"{selected_path_id} 1위 횟수", f"{runner_up_path_id} 1위 횟수", "동결 선택과 다른 1위 횟수"],
            report_header_fmt,
        )
        selected_wins = sum(winner == selected_path_id for winner in loo_winners)
        runner_up_wins = sum(winner == runner_up_path_id for winner in loo_winners)
        changed_winners = sum(winner != selected_path_id for winner in loo_winners)
        loo_sheet.write_row(12, 0, [selected_wins, runner_up_wins, changed_winners])
        loo_sheet.merge_range(
            "A15:I15",
            "5개 분석은 서로 독립 비행 실험 5회가 아닙니다. 이번 5개 제외 조건에는 동률이 없었습니다. 동률이 생기면 원래 규칙의 짧은 참조시간→경로 ID를 적용해야 합니다. 다른 MC trial의 순위는 계산하지 않았습니다.",
            report_note_fmt,
        )

        # Match the V3 G0 scope: five paths × five repetitions × A/B × two scopes,
        # each with seven primary yaw cases and 100 trials.
        groups = {}
        selected_metric_rows = 0
        for row in metric_rows:
            calibration = row.get("calibration_run_id", "")
            evaluation = row.get("evaluation_run_id", "")
            if (row.get("attitude_variant") != "RPY_IMU"
                    or row.get("method") not in ("A_rtk_independent", "B_rtk_joint")
                    or not re.fullmatch(r"G0-P[0-4]-R0[1-5]", calibration)
                    or not re.fullmatch(r"G0-V0-R0[1-5]", evaluation)
                    or row.get("yaw_case") not in {f"Y{index}" for index in range(7)}
                    or row.get("data_scope") not in ("all_data", "matched_200")):
                continue
            if calibration[-3:] != evaluation[-3:] or row.get("evaluation_status") != "EVALUATED":
                raise ValueError(f"V3 V0 row is not a valid matched evaluation: {calibration} → {evaluation}")
            parts = calibration.split("-")
            key = (parts[1], int(parts[2][1:]), row["method"], row["data_scope"], calibration, evaluation)
            group = groups.setdefault(key, {"rmse": [], "p95": [], "case_ids": set()})
            case_id = (row["yaw_case"], int(row["trial_index"]))
            if case_id in group["case_ids"]:
                raise ValueError(f"V3 contains a duplicate V0 Monte Carlo case: {key} {case_id}")
            group["case_ids"].add(case_id)
            selected_metric_rows += 1
            rmse, p95 = _number(row.get("position_3d_rmse_m")), _number(row.get("position_3d_p95_m"))
            if rmse is None or p95 is None:
                raise ValueError(f"V3 V0 case lacks RMSE/P95 metrics: {key} {case_id}")
            group["rmse"].append(rmse)
            group["p95"].append(p95)
        if (len(groups) != 100 or selected_metric_rows != 70_000
                or any(len(group["case_ids"]) != 700 for group in groups.values())):
            raise ValueError(
                f"V3 requires 100 G0 run groups × 700 cases; found {len(groups)} groups and {selected_metric_rows} rows"
            )
        run_groups = {}
        for key, group in sorted(groups.items()):
            path_id, repetition, method, scope, calibration, evaluation = key
            mean_rmse = sum(group["rmse"]) / len(group["rmse"])
            mean_p95 = sum(group["p95"]) / len(group["p95"])
            run_groups[path_id, repetition, method, scope] = (mean_rmse, mean_p95, len(group["rmse"]))
        run_sheet = workbook.get_worksheet_by_name(SHEET_NAMES[16])
        expected_run_groups = {
            (path_id, repetition, method, scope)
            for path_id in path_order for repetition in range(1, 6)
            for method in ("A_rtk_independent", "B_rtk_joint")
            for scope in ("all_data", "matched_200")
        }
        if set(run_groups) != expected_run_groups:
            raise ValueError("V3 run-level table must contain every G0 path/repetition/method/scope group")
        duration_by_run = {
            ctx["slot"]["run_id"]: _number(ctx["cadence"].get("science_window_duration_s"))
            for ctx in run_context
        }
        run_sheet.merge_range(
            "A1:N1",
            f"V0 위치오차 · 동결 선택 {selected_path_id}와 공분산 점수 2위 {runner_up_path_id} 비교",
            title_fmt,
        )
        run_sheet.merge_range(
            "A2:N2",
            "G0 existing-yaw Y0–Y6 × MC 100을 각 교정 run·대응 V0 run쌍 안에서 평균한 기술값입니다. 700 case는 독립 비행 700회가 아닙니다. 비행 반복 독립성은 검증되지 않았습니다.",
            report_note_fmt,
        )
        start_up = V0_START_NORTH_M * math.tan(math.radians(V0_GLIDE_ANGLE_DEG))
        end_up = V0_END_NORTH_M * math.tan(math.radians(V0_GLIDE_ANGLE_DEG))
        run_sheet.merge_range(
            "A3:N3",
            f"계획된 V0 science 참조: world ENU [E,N,U]=[0,600,{start_up:.3f}]m에서 [0,400,{end_up:.3f}]m까지. N=600m가 science 구간 시작점이며 이륙·진입·5초 안정화는 구간 바깥입니다. 이 표기는 각 실행이 시작점에 실제 도달했다는 뜻이 아닙니다.",
            report_note_fmt,
        )
        top_rows = []
        for repetition in range(1, 6):
            top_rows.append([
                f"R0{repetition}",
                run_groups[selected_path_id, repetition, "A_rtk_independent", "all_data"][0],
                run_groups[runner_up_path_id, repetition, "A_rtk_independent", "all_data"][0],
                run_groups[selected_path_id, repetition, "B_rtk_joint", "all_data"][0],
                run_groups[runner_up_path_id, repetition, "B_rtk_joint", "all_data"][0],
            ])
        _write_table(run_sheet, workbook,
                     ["대응 반복", f"A {selected_path_id} RMSE (m)", f"A {runner_up_path_id} RMSE (m)",
                      f"B {selected_path_id} RMSE (m)", f"B {runner_up_path_id} RMSE (m)"],
                     top_rows, widths=[17, 22, 23, 24, 26], freeze=47, start_row=3, autofilter=False)
        pair_rows = []
        for repetition in range(1, 6):
            for method in ("A_rtk_independent", "B_rtk_joint"):
                for scope in ("all_data", "matched_200"):
                    selected_rmse = run_groups[selected_path_id, repetition, method, scope][0]
                    runner_up_rmse = run_groups[runner_up_path_id, repetition, method, scope][0]
                    selected_time = duration_by_run.get(f"G0-{selected_path_id}-R{repetition:02d}")
                    runner_up_time = duration_by_run.get(f"G0-{runner_up_path_id}-R{repetition:02d}")
                    pair_rows.append([
                        f"R0{repetition}", method, scope, selected_rmse, runner_up_rmse,
                        runner_up_rmse - selected_rmse,
                        selected_time, runner_up_time,
                        runner_up_time - selected_time
                        if selected_time is not None and runner_up_time is not None else None,
                    ])
        _write_table(run_sheet, workbook,
                     ["반복", "방법", "범위", f"{selected_path_id} RMSE (m)", f"{runner_up_path_id} RMSE (m)",
                      f"{runner_up_path_id}−{selected_path_id} (m)",
                      f"{selected_path_id} science duration (s)", f"{runner_up_path_id} science duration (s)",
                      f"{runner_up_path_id}−{selected_path_id} time (s)"],
                     pair_rows, widths=[17, 23, 20, 22, 22, 20, 23, 23, 22],
                     freeze=47, start_row=20, autofilter=False)
        run_sheet.merge_range(
            "A43:I43",
            f"{runner_up_path_id}−{selected_path_id} > 0이면 이 저장된 실행쌍·조건에서 점수 2위 {runner_up_path_id}의 값이 동결 선택 {selected_path_id}보다 큽니다. 선택은 covariance 기준이며 이 표는 동결 후 V0 기술 비교입니다. 반복번호는 대응 V0 run의 식별자일 뿐 독립성 보증이 아닙니다. all_data와 matched_200 순위를 섞지 마세요.",
            report_note_fmt,
        )
        detail_table_rows = [
            [path_id, f"R0{repetition}", method, scope, calibration, evaluation, count, rmse, p95,
             duration_by_run.get(calibration),
             f"{analysis_root.relative_to(output_root)}/evaluation_metrics.csv"]
            for (path_id, repetition, method, scope, calibration, evaluation), count, rmse, p95
            in ((key, len(groups[key]["case_ids"]), run_groups[key[:4]][0], run_groups[key[:4]][1])
                for key in sorted(groups))
        ]
        _write_table(run_sheet, workbook,
                     ["경로", "반복", "방법", "범위", "교정 run", "V0 run", "case 수",
                      "3D RMSE 평균 (m)", "3D P95 평균 (m)", "science duration (s)", "원본 CSV"],
                     detail_table_rows, widths=[12, 14, 30, 18, 22, 22, 14, 22, 22, 22, 72],
                     freeze=47, start_row=46, autofilter=True)
        run_sheet.set_row(2, 54)
        run_chart = workbook.add_chart({"type": "line"})
        for col, name in (
            (1, f"A {selected_path_id} RMSE"), (2, f"A {runner_up_path_id} RMSE"),
            (3, f"B {selected_path_id} RMSE"), (4, f"B {runner_up_path_id} RMSE"),
        ):
            run_chart.add_series({
                "name": name,
                "categories": [SHEET_NAMES[16], 4, 0, 8, 0],
                "values": [SHEET_NAMES[16], 4, col, 8, col],
            })
        run_chart.set_title({"name": "G0 V0 3D RMSE · 5 run쌍 평균 (all_data; MC/yaw는 실행 안에서 집계)"})
        run_chart.set_legend({"position": "top"})
        run_chart.set_y_axis({"num_format": "0.000", "name": "3D RMSE (m)"})
        run_chart.set_size({"width": 760, "height": 360})
        run_sheet.insert_chart("G4", run_chart)

        p95_sheet = workbook.get_worksheet_by_name(SHEET_NAMES[17])
        p95_sheet.merge_range("A1:N1", "추종 위치 P95 · 기록값과 재정렬값 대조", title_fmt)
        p95_sheet.merge_range(
            "A2:N2",
            "두 값 모두 world ENU [east,north,up]의 GT−참조 3D 거리(m)에 type-7 P95를 사용합니다. 기록값은 flight_state 저장 GT와 40ms 지연 제한을 사용하고, 재정렬값은 science control 시각에 원시 GT를 다시 보간합니다. V0 재정렬 tracking은 원본 QC에서 숨겨졌습니다.",
            report_note_fmt,
        )
        both_p95 = [row for row in p95_source_rows if row[4] is not None and row[6] is not None]
        deltas = [row[8] for row in both_p95]
        p95_sheet.write_row(3, 0, ["유효 run", "재정렬 P95 존재", "양쪽 모두 >1m", "재정렬 P95 미제공"], report_header_fmt)
        p95_sheet.write_row(4, 0, [
            len(p95_source_rows), len([row for row in p95_source_rows if row[6] is not None]),
            len([row for row in both_p95 if row[4] > 1 and row[6] > 1]),
            len([row for row in p95_source_rows if row[6] is None]),
        ])
        p95_sheet.write_row(3, 5, ["최대 |차이| (m)", "평균 차이 (m)"], report_header_fmt)
        p95_sheet.write_row(4, 5, [max((abs(value) for value in deltas), default=None),
                                   sum(deltas) / len(deltas) if deltas else None])
        _write_table(p95_sheet, workbook,
                     ["run_id", "attempt", "그룹", "경로", "기록 P95 (m)", "기록 표본 n",
                      "재정렬 P95 (m)", "재정렬 표본 n", "기록−재정렬 (m)", "기록 >1m",
                      "재정렬 >1m", "원본 파일 · 캠페인 상대", "원본 SHA-256 · 98_출처", "|차이| (m)"],
                     p95_source_rows, widths=[19, 12, 11, 11, 21, 17, 22, 22, 24, 16, 17, 70, 65, 19],
                     freeze=8, start_row=7, autofilter=True)
        p95_sheet.merge_range(
            "A49:K49",
            "기록 P95는 저장된 flight_state GT와 science 행의 40ms 시각차 제한을 사용합니다. 재정렬 P95는 같은 science 시각에 원시 GT를 보간해 별도 계산하며 표본·GT 위치가 다를 수 있습니다. 원본 QC 파일은 수정하지 않았습니다.",
            report_note_fmt,
        )

        source_rows = [["analysis_id", latest["analysis_id"]], ["analysis_freeze_sha256", latest["analysis_freeze_sha256"]],
                       ["frozen_v1_2_specification", str(protocol_spec_path.relative_to(output_root))],
                       ["frozen_v1_2_specification_sha256", _sha256(protocol_spec_path)],
                       ["analysis_summary", str(summary_path.relative_to(output_root))],
                       ["analysis_summary_sha256", _sha256(summary_path)],
                       ["analysis_run_manifest_sha256", _sha256(analysis_root / "analysis_run_manifest.json")],
                       ["path_selection", str((analysis_root / "path_selection.json").relative_to(output_root))],
                       ["path_selection_sha256", _sha256(analysis_root / "path_selection.json")],
                       ["evaluation_metrics", str((analysis_root / "evaluation_metrics.csv").relative_to(output_root))],
                       ["evaluation_metrics_sha256", _sha256(analysis_root / "evaluation_metrics.csv")],
                       ["attitude_variant_comparisons", str((analysis_root / "attitude_variant_comparisons.csv").relative_to(output_root))],
                       ["attitude_variant_comparisons_sha256", _sha256(analysis_root / "attitude_variant_comparisons.csv")],
                       ["audit_report", str(audit_report.relative_to(output_root))],
                       ["audit_report_sha256", _sha256(audit_report)],
                       ["audit_scope", audit.get("scope")],
                       ["report_builder_sha256", _sha256(Path(__file__).resolve())],
                       ["workbook_campaign_plan_sha256", _sha256(output_root / "workbook_campaign_plan.json")]]
        for ctx in run_context:
            slot = ctx["slot"]
            sources = (
                ("qc_report", ctx["qc_path"]),
                ("gt_alignment_summary", ctx["alignment_path"]),
                ("image_index_summary", ctx["image_summary_path"]),
                ("controller_result", ctx["controller_path"]),
                ("observations_left", ctx["observation_paths"][0]),
                ("observations_right", ctx["observation_paths"][1]),
                ("ground_truth", ctx["postfreeze_paths"][0]),
                ("flight_state", ctx["postfreeze_paths"][1]),
            )
            source_rows.extend(
                [f"{slot['run_id']} {label}_sha256", _sha256(path)]
                for label, path in sources if path.is_file()
            )
        _table_sheet(workbook, SHEET_NAMES[18], ["artifact", "value"], source_rows, [52, 110])
        definitions = [
            ["Position axis contract", "world ENU [east,north,up] metres. PX4 local NED, vehicle body FRD and camera optical RDF are separate frames."],
            ["V0 science reference", "Planned world ENU [E,N,U]=[0,600,218.382]→[0,400,145.588] m at 20°; N=600 m is the planned start, not a claim of actual arrival."],
            ["Position RMSE/P95", "Estimator position minus GT in world ENU; per-case metrics from frozen evaluation_metrics.csv."],
            ["Tracking P95", "Recorded: saved flight-state GT minus reference with |GT time offset| <=40 ms. Recomputed: raw GT interpolated at science control timestamps after freeze. Both are world-ENU 3D distance and type-7 P95."],
            ["Rig group separation", "G0 is the primary Y0–Y6 comparison. G1 uses nominal-zero initial heading (G1_PRIMARY_ZERO) and is a separate check; their metrics are not pooled."],
            ["Sensor-model boundary", "Sensor error values are design-level sensitivity settings, not measured hardware performance."],
            ["External validity", protocol_spec["split_policy"]["limitation"]],
            ["Audit PASS", "Coverage/schema/provenance only. It does not establish an accuracy threshold; the protocol defines no numeric accuracy pass."],
            ["MC cases", "Monte Carlo cases are not independent physical flights."],
            ["Selection", f"Covariance-based selection chose {selected_path_id}; the post-freeze V0 accuracy results are descriptive and did not choose or refit that path."],
            ["Missing metric", "Blank metrics remain unavailable; they are not imputed as zero."],
        ]
        _table_sheet(workbook, SHEET_NAMES[19], ["term", "definition"], definitions, [32, 120])

        # Charts are generated from current-run data and placed on the familiar V3 dashboard.
        graph = workbook.get_worksheet_by_name(SHEET_NAMES[5])
        graph.write("A1", "V3 분석 그래프 · 현재 캠페인 자료", title_fmt)
        graph.write("A3", "차트는 world ENU 지표와 원본 science-window QC를 표시합니다.")
        graph.write("A4", "RPY−YAW 음수 delta는 RPY_IMU 지표가 더 낮음을 뜻합니다. 비교 차트는 Y0/all_data의 comparable MC 평균이며 독립 비행 반복이 아닙니다.")
        graph.set_column("A:Z", 14)
        chart_sources: list[tuple[str, list[str], list[list[object]], list[str], str]] = []
        for metric, label in (("position_3d_rmse_mean_m", "3D RMSE mean (m)"),
                              ("position_3d_p95_mean_m", "3D P95 mean (m)"),
                              ("detection_rate_mean", "Detection rate"),
                              ("valid_measurement_rate_mean", "Valid measurement rate")):
            data = [[f"{row.get('rig_group')} / {row.get('method')} / {row.get('attitude_variant')} / {row.get('data_scope')}", row.get(metric)] for row in comparison_rows]
            chart_sources.append((label, ["comparison", label], data, [label], "column"))
        convergence = _calibration_convergence_by_group(calibration_metric_rows)
        conv_rows = [[f"{row['rig_group']} / {row['method']} / {row['attitude_variant']}",
                      row["attempted_fit_count"], row["nonconverged_fit_count"]]
                     for row in convergence]
        chart_sources.append(("Calibration convergence counts by rig group", ["group", "attempted", "nonconverged"], conv_rows, ["attempted", "nonconverged"], "column"))
        for label, data, cols in (
            ("Recorded tracking P95 by run", [[r[0], r[7]] for r in tracking_rows], ["P95 (m)"]),
            ("Post-freeze tracking P95 by run", [[r[0], postfreeze_by_run[r[0]].get("position_3d_p95_m")] for r in tracking_rows], ["P95 (m)"]),
            ("Left/right science observed camera Hz", [[r[0], r[5], r[8]] for r in cadence_rows], ["left Hz", "right Hz"]),
            ("Left/right science frame count", [[r[0], r[4], r[7]] for r in cadence_rows], ["left frames", "right frames"]),
            ("Missing 50 Hz reference slots", [[r[0], r[13]] for r in cadence_rows], ["missing slots"]),
            ("Maximum science simulation gap", [[r[0], r[14]] for r in cadence_rows], ["gap (s)"]),
            ("Science duration by run", [[r[0], r[15]] for r in cadence_rows], ["duration (s)"]),
            ("Detected observations by run", [[r[0], r[4] + r[5]] for r in observation_rows], ["detected frames"]),
            ("Nominal image pairs by run", [[r[0], r[3]] for r in observation_rows], ["pairs"]),
            ("GT-excluded science images by run", [[slot["run_id"], sum(int(row[4]) for row in gt_excluded_rows if row[0] == slot["run_id"])] for slot in slots], ["excluded frames"]),
        ):
            chart_sources.append((label, ["run_id", *cols], data, cols, "line"))
        v0_by_path = {}
        for (path_id, rep, method, scope), (rmse, p95, n) in run_groups.items():
            if scope != "all_data":
                continue
            item = v0_by_path.setdefault(path_id, {})
            if rmse is not None:
                item.setdefault(f"{method} RMSE", []).append(rmse)
            if p95 is not None:
                item.setdefault(f"{method} P95", []).append(p95)
        for metric_suffix, title in (("RMSE", "V0 all-data RMSE by path"), ("P95", "V0 all-data P95 by path")):
            cols = [f"{method} {metric_suffix}" for method in ("A_rtk_independent", "B_rtk_joint")]
            data = [[path_id, *[
                (sum(v0_by_path.get(path_id, {}).get(col, [])) / len(v0_by_path[path_id][col]))
                if v0_by_path.get(path_id, {}).get(col) else None for col in cols
            ]] for path_id in path_order]
            chart_sources.append((title, ["path", *cols], data, cols, "column"))
        path_score_rows = []
        for path_id in path_order:
            values = [float(row["predicted_position_uncertainty_mean_sigma_m"]) for row in path_predictions
                      if row.get("path_id") == path_id and int(row.get("repetition", -1)) > 0
                      and row.get("method") in ("A_rtk_independent", "B_rtk_joint", "C_no_drone_rtk_joint")
                      and row.get("predicted_position_uncertainty_mean_sigma_m") is not None]
            path_score_rows.append([path_id, sum(values) / len(values) if values else None])
        chart_sources.append(("Covariance target sigma by path", ["path", "mean sigma (m)"], path_score_rows, ["mean sigma (m)"], "column"))
        loo_chart_data = [[row[0], *row[1:6]] for row in loo_rows]
        chart_sources.append(("Leave-one-repetition-out selection score", ["omitted repetition", *path_order], loo_chart_data, path_order, "line"))
        for metric_suffix, title, field in (
            ("RMSE", "RPY−YAW RMSE delta · Y0 all-data comparable MC means", "rmse_delta_rpy_minus_yaw_m"),
            ("P95", "RPY−YAW P95 delta · Y0 all-data comparable MC means", "p95_delta_rpy_minus_yaw_m"),
        ):
            data = _paired_variant_run_rows(variant_rows, field)
            chart_sources.append((title, ["run pair/method", f"mean delta {metric_suffix} (m)"], data,
                                  [f"mean delta {metric_suffix} (m)"], "line"))
        convergence_rows = [
            [f"{row['rig_group']} / {row['method']} / {row['attitude_variant']}",
             row.get("nonconvergence_fraction_among_attempted_fits"),
             row.get("failure_or_not_run_fraction_all_cases")]
            for row in convergence
        ]
        chart_sources.append(("Calibration convergence and total-case failure rates",
                              ["group", "nonconvergence fraction", "failure-or-not-run fraction"],
                              convergence_rows,
                              ["nonconvergence fraction", "failure-or-not-run fraction"], "column"))
        if len(chart_sources) != 22:
            raise ValueError(f"V3 dashboard must have exactly 22 chart sources, found {len(chart_sources)}")
        data_col = 50
        source_row = 400
        for index, (title, headers, data, values, chart_type) in enumerate(chart_sources):
            graph.write_row(source_row, data_col, headers)
            for offset, row in enumerate(data, 1):
                graph.write_row(source_row + offset, data_col, row)
            if not data:
                graph.write_row(source_row + 1, data_col, ["no data", *([None] * (len(headers) - 1))])
            chart = workbook.add_chart({"type": chart_type})
            for value_index, label in enumerate(values, 1):
                chart.add_series({
                    "name": [SHEET_NAMES[5], source_row, data_col + value_index],
                    "categories": [SHEET_NAMES[5], source_row + 1, data_col, source_row + max(1, len(data)), data_col],
                    "values": [SHEET_NAMES[5], source_row + 1, data_col + value_index, source_row + max(1, len(data)), data_col + value_index],
                })
            chart.set_title({"name": title})
            chart.set_legend({"position": "bottom"})
            chart.set_size({"width": 600, "height": 290})
            chart.set_y_axis({"major_gridlines": {"visible": True}})
            graph.insert_chart((index // 2) * 22 + 5, (index % 2) * 10, chart)
            source_row += max(3, len(data) + 3)
        graph.set_column(data_col, data_col + 24, None, None, {"hidden": True})

        workbook.close()
        workbook = None
        with temporary_xlsx.open("rb") as handle:
            os.fsync(handle.fileno())
        os.replace(temporary_xlsx, output_path)
    finally:
        if workbook is not None:
            try:
                workbook.close()
            except Exception:
                pass
        import shutil
        shutil.rmtree(report_dir, ignore_errors=True)

    input_files = [
        output_root / "run_status.json", output_root / "workbook_campaign_plan.json",
        protocol_spec_path,
        output_root / "analysis_latest.json", summary_path, analysis_root / "analysis_freeze.json",
        analysis_root / "analysis_run_manifest.json", analysis_root / "path_selection.json",
        analysis_root / "analysis_checksums.sha256", analysis_root / "calibration_metrics.csv",
        analysis_root / "evaluation_metrics.csv", analysis_root / "attitude_variant_comparisons.csv",
        audit_report,
    ]
    for ctx in run_context:
        input_files.extend((ctx["qc_path"], ctx["alignment_path"],
                            ctx["attempt_dir"] / "checksums.sha256"))
        input_files.extend(path for path in (
            ctx["image_summary_path"], ctx["controller_path"], *ctx["observation_paths"],
            *ctx["postfreeze_paths"],
        ) if path.is_file())
    manifest_path = output_path.with_suffix(".manifest.json")
    manifest = {
        "schema_version": "stereo_v1_2_report_v3_manifest_v3",
        "analysis_id": latest["analysis_id"],
        "analysis_freeze_sha256": latest["analysis_freeze_sha256"],
        "report_builder_sha256": _sha256(Path(__file__).resolve()),
        "output_path_from_campaign_root": str(output_path.relative_to(output_root)),
        "output_sha256": _sha256(output_path),
        "worksheet_names": list(SHEET_NAMES),
        "dashboard_chart_count": len(chart_sources),
        "chart_count": len(chart_sources) + 1,
        "run_count": len(run_context),
        "coordinate_contract": "world ENU [east,north,up] metres; PX4 local NED/body FRD/optical RDF remain distinct",
        "comparison_aggregation": "evaluation and calibration metrics reaggregated by rig_group, scope/method/attitude as applicable; G0/G1 kept separate",
        "comparison_case_counts_by_group": {
            group: max(row["evaluation_case_count"] for row in comparison_rows if row["rig_group"] == group)
            for group in ("G0", "G1")
        },
        "legacy_pooled_analysis_summary_reaggregated": legacy_pooled_summary_reaggregated,
        "main_comparison_summary_by_rig_group": comparison_rows,
        "inputs_sha256": {str(path.relative_to(output_root)): _sha256(path) for path in input_files},
    }
    temp_manifest = manifest_path.with_name(f".{manifest_path.name}.{os.getpid()}.tmp")
    temp_manifest.write_text(json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temp_manifest, manifest_path)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--audit-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    try:
        manifest = build_report(args.output_root, args.audit_report, args.output)
    except (OSError, ValueError, KeyError, TypeError, ImportError) as exc:
        print(json.dumps({"status": "REPORT_FAILED", "error": str(exc)}, ensure_ascii=False), flush=True)
        return 2
    print(json.dumps({"status": "REPORT_COMPLETE", **manifest}, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
