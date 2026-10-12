#!/usr/bin/env python3
"""Initialize all 39 campaign slots without claiming any execution result."""

from __future__ import annotations

import argparse
import json
import os
import hashlib
import sys
import tempfile
import time
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.plan_campaign import build_plan


FROZEN_FILES = (
    "03_experiment_spec.json", "04_run_manifest.json",
    "05_generate_reference.py", "provenance.json",
)
REVALIDATABLE_QC_REGRESSION_ERRORS = frozenset({
    "workbook_run_manifest_vehicle_model_id_mismatch",
    "workbook_run_manifest_vehicle_reference_link_mismatch",
})


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_frozen_protocol(output_root: Path, *, include_protocol: bool = True) -> None:
    source_root = Path(__file__).resolve().parent / "protocol"
    frozen_root = output_root / "frozen_protocol"
    frozen_root.mkdir(exist_ok=False)
    if include_protocol:
        for name in FROZEN_FILES:
            source = source_root / name
            target = frozen_root / name
            with target.open("xb") as handle:
                handle.write(source.read_bytes())
                handle.flush()
                os.fsync(handle.fileno())
            if _sha256(source) != _sha256(target):
                raise ValueError(f"frozen protocol copy hash mismatch: {name}")

    schema_mapping = {
        "schema_version": "stereo_v1_1_topic_schema_mapping_v1",
        "topics": [
            {"topic": "/clock", "message": "rosgraph_msgs/msg/Clock",
             "output": "ground_truth.jsonl.simulation_timestamp_ns",
             "contract": "Gazebo simulation clock; reset increments simulation_epoch"},
            {"topic": "/simulation/ground_truth/vehicle_reference_link",
             "message": "geometry_msgs/msg/PoseStamped", "output": "ground_truth.jsonl",
             "contract": "Gazebo world ENU [east,north,up] metres; orientation world_from_vehicle_reference_link xyzw"},
            {"topic": "/fmu/out/vehicle_local_position_v1",
             "message": "px4_msgs/msg/VehicleLocalPosition", "output": "flight_state.jsonl",
             "contract": "PX4 local NED [north,east,down] metres and m/s; independent observed origin"},
            {"topic": "/fmu/out/vehicle_attitude", "message": "px4_msgs/msg/VehicleAttitude",
             "output": "px4_attitude.jsonl", "contract": "PX4 body attitude; not camera optical pose"},
            {"topic": "/left_camera/image", "message": "sensor_msgs/msg/Image",
             "output": "raw_images_or_lossless_bag/left", "contract": "lossless source image; Gazebo exposure stamp"},
            {"topic": "/right_camera/image", "message": "sensor_msgs/msg/Image",
             "output": "raw_images_or_lossless_bag/right", "contract": "lossless source image; Gazebo exposure stamp"},
            {"topic": "/simulation/camera_pose_diagnostics/left", "message": "std_msgs/msg/String",
             "output": "camera_truth.json.camera_payloads.left",
             "contract": "Gazebo observed camera optical center world ENU and optical RDF orientation xyzw"},
            {"topic": "/simulation/camera_pose_diagnostics/right", "message": "std_msgs/msg/String",
             "output": "camera_truth.json.camera_payloads.right",
             "contract": "Gazebo observed camera optical center world ENU and optical RDF orientation xyzw"},
        ],
        "offline_observation_fields": {
            "observations_left.jsonl": "original source frame id, exposure/receive/inference times, detection status, bbox xyxy and center uv in original 1280x720 pixels",
            "observations_right.jsonl": "original source frame id, exposure/receive/inference times, detection status, bbox xyxy and center uv in original 1280x720 pixels",
            "paired_observations.jsonl": "unique left/right source-frame IDs, pair midpoint and skew, both detection statuses",
            "sensor_observations.json": "explicitly synthetic camera optical-center RTK and camera-mounted IMU roll/pitch inputs; never physical-sensor evidence",
        },
        "camera_projection_contract": "optical RDF [right,down,forward] to world ENU through R_WC; projection uses R_WC.T @ (p_W-C_W)",
        "position_conversion_contract": "world ENU [east,north,up] to PX4 local NED [north,east,down] uses a runtime-observed origin",
    }
    _write_new(output_root / "schema_mapping.json", schema_mapping)

    checksummed = sorted(
        path for path in (frozen_root.rglob("*") if frozen_root.exists() else ())
        if path.is_file()
    )
    checksummed.append(output_root / "schema_mapping.json")
    with (output_root / "checksums.sha256").open("x", encoding="utf-8") as handle:
        for path in checksummed:
            handle.write(f"{_sha256(path)}  {path.relative_to(output_root)}\n")
        handle.flush()
        os.fsync(handle.fileno())


def _write_new(path: Path, payload: dict) -> None:
    with path.open("x", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())


def _atomic_json(path: Path, payload: dict) -> None:
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _append_change(output_root: Path, event: str, **fields) -> None:
    row = {"event": event, "utc_ns": time.time_ns(), **fields}
    with (output_root / "changes_log.jsonl").open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
        handle.flush()
        os.fsync(handle.fileno())


def _refresh_counts(output_root: Path, status: dict) -> None:
    counts: dict[str, int] = {}
    for row in status["run_slots"]:
        counts[row["status"]] = counts.get(row["status"], 0) + 1
    valid = counts.get("VALID", 0) + counts.get("VALID_WITH_WARNINGS", 0)
    status["valid_run_count"] = valid
    summary = _load(output_root / "qc_summary.json")
    summary["status_counts"] = counts
    summary["valid_run_count"] = valid
    summary["technical_failure_count"] = sum(
        attempt["status"] == "INFRASTRUCTURE_FAILED"
        for row in status["run_slots"] for attempt in row["attempts"]
    )
    _atomic_json(output_root / "qc_summary.json", summary)


def _slot(status: dict, run_id: str) -> dict:
    matches = [row for row in status["run_slots"] if row["run_id"] == run_id]
    if len(matches) != 1:
        raise ValueError(f"run_id is not one of the initialized campaign slots: {run_id}")
    return matches[0]


def begin_attempt(output_root: Path, run_id: str) -> tuple[dict, Path]:
    """Allocate a new immutable attempt for a manifest slot under a held runner lock."""
    output_root = output_root.resolve()
    status = _load(output_root / "run_status.json")
    actual = _load(output_root / "manifest_actual.json")
    row = _slot(status, run_id)
    if row["status"] in ("VALID", "VALID_WITH_WARNINGS"):
        raise ValueError("first valid attempt already exists; performance rerun is forbidden")
    if row["status"] == "RUNNING":
        raise ValueError("previous attempt remains RUNNING; reconcile before resume")
    if row["status"] == "PROTOCOL_DEVIATION":
        raise ValueError("protocol deviation requires a documented revision before retry")
    if len(row["attempts"]) >= 2 and all(
        attempt["status"] == "INFRASTRUCTURE_FAILED"
        and attempt.get("failure_reason") == row["attempts"][-1].get("failure_reason")
        for attempt in row["attempts"][-2:]
    ):
        raise ValueError("same technical failure twice; repair the cause before retry")
    number = len(row["attempts"]) + 1
    attempt_id = f"a{number:02d}"
    attempt_path = output_root / "원본_실행별" / run_id / attempt_id
    if attempt_path.exists():
        raise FileExistsError("unregistered attempt directory exists; audit it before resume")
    attempt_path.mkdir(parents=True, exist_ok=False)
    relative = str(attempt_path.relative_to(output_root))
    record = {
        "run_id": run_id,
        "attempt_id": attempt_id,
        "status": "RUNNING",
        "seed_requested": row["seed_requested"],
        "seed_effective": None,
        "recording_path": relative,
        "failure_reason": None,
        "started_utc_ns": time.time_ns(),
    }
    row["attempts"].append(record.copy())
    row["attempt_count"] = len(row["attempts"])
    row["status"] = "RUNNING"
    row["recording_path"] = relative
    actual["attempts"].append(record.copy())
    actual["actual_attempt_count"] = len(actual["attempts"])
    _atomic_json(output_root / "manifest_actual.json", actual)
    _refresh_counts(output_root, status)
    _atomic_json(output_root / "run_status.json", status)
    _append_change(output_root, "attempt_started", run_id=run_id, attempt_id=attempt_id,
                   recording_path=relative, seed_requested=row["seed_requested"])
    return record, attempt_path


def finish_attempt(output_root: Path, run_id: str, attempt_id: str,
                   qc_report: dict, seed_effective: dict | None = None) -> dict:
    """Commit a QC result without deleting any attempt or changing its seed."""
    output_root = output_root.resolve()
    status = _load(output_root / "run_status.json")
    actual = _load(output_root / "manifest_actual.json")
    row = _slot(status, run_id)
    if row["status"] != "RUNNING" or not row["attempts"] or row["attempts"][-1]["attempt_id"] != attempt_id:
        raise ValueError("attempt is not the active RUNNING attempt")
    if qc_report.get("run_id") != run_id or qc_report.get("attempt_id") != attempt_id:
        raise ValueError("QC identity differs from active attempt")
    result = str(qc_report.get("status"))
    if result not in ("VALID", "VALID_WITH_WARNINGS", "INFRASTRUCTURE_FAILED", "PROTOCOL_DEVIATION"):
        raise ValueError("QC returned an unsupported attempt state")
    failure = ",".join(qc_report.get("infrastructure_errors", ()) or qc_report.get("protocol_errors", ())) or None
    if failure is None and qc_report.get("evidence", {}).get("controller_status") != "science_completed":
        failure = qc_report.get("evidence", {}).get("controller_failure_reason") or None
    for record in (row["attempts"][-1], actual["attempts"][-1]):
        if record["run_id"] != run_id or record["attempt_id"] != attempt_id:
            raise ValueError("manifest_actual and run_status disagree on active attempt")
        record["status"] = result
        record["failure_reason"] = failure
        record["seed_effective"] = seed_effective
        record["completed_utc_ns"] = time.time_ns()
    row["status"] = result
    row["failure_reason"] = failure
    row["seed_effective"] = seed_effective
    _atomic_json(output_root / "manifest_actual.json", actual)
    _refresh_counts(output_root, status)
    _atomic_json(output_root / "run_status.json", status)
    _append_change(output_root, "attempt_qc_finalized", run_id=run_id,
                   attempt_id=attempt_id, status=result, failure_reason=failure)
    return row


def _write_attempt_checksums(attempt_dir: Path) -> None:
    checksum_path = attempt_dir / "checksums.sha256"
    descriptor, temporary_name = tempfile.mkstemp(prefix=".checksums.", dir=attempt_dir)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            for path in sorted(item for item in attempt_dir.rglob("*")
                               if item.is_file() and item not in (checksum_path, temporary)):
                handle.write(f"{_sha256(path)}  {path.relative_to(attempt_dir)}\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, checksum_path)
    finally:
        temporary.unlink(missing_ok=True)


def revalidate_known_qc_regression(output_root: Path, run_id: str, attempt_id: str,
                                   qc_report: dict) -> dict:
    """Accept a corrected QC result for a preserved workbook attempt.

    This path is deliberately limited to the two known false-positive errors
    caused by comparing workbook Vehicle values with absent plan-level aliases.
    It never starts another flight or changes the captured source data.
    """
    output_root = output_root.resolve()
    status = _load(output_root / "run_status.json")
    actual = _load(output_root / "manifest_actual.json")
    if status.get("schema_version") != "stereo_v1_1_workbook_run_status_v1":
        raise ValueError("QC regression revalidation is restricted to workbook campaigns")
    row = _slot(status, run_id)
    if row.get("status") != "PROTOCOL_DEVIATION" or not row.get("attempts"):
        raise ValueError("slot is not a completed protocol-deviation attempt")
    attempt = row["attempts"][-1]
    if attempt.get("attempt_id") != attempt_id or attempt.get("status") != "PROTOCOL_DEVIATION":
        raise ValueError("attempt is not the slot's latest protocol-deviation attempt")
    actual_matches = [item for item in actual.get("attempts", [])
                      if item.get("run_id") == run_id and item.get("attempt_id") == attempt_id]
    if len(actual_matches) != 1 or actual_matches[0].get("status") != "PROTOCOL_DEVIATION":
        raise ValueError("manifest_actual does not contain one matching failed attempt")

    attempt_dir = output_root / attempt["recording_path"]
    report_path = attempt_dir / "qc_report.json"
    result_path = attempt_dir / "runner_result.json"
    if not report_path.is_file() or not result_path.is_file():
        raise ValueError("attempt is missing its original QC or runner result")
    previous = _load(report_path)
    previous_result = _load(result_path)
    previous_errors = previous.get("protocol_errors", [])
    if (previous.get("run_id") != run_id or previous.get("attempt_id") != attempt_id
            or previous.get("status") != "PROTOCOL_DEVIATION"
            or set(previous_errors) != REVALIDATABLE_QC_REGRESSION_ERRORS
            or len(previous_errors) != len(REVALIDATABLE_QC_REGRESSION_ERRORS)
            or previous.get("infrastructure_errors")
            or previous_result.get("qc_status") != "PROTOCOL_DEVIATION"):
        raise ValueError("saved QC contains a deviation outside the known validator regression")
    if (qc_report.get("run_id") != run_id or qc_report.get("attempt_id") != attempt_id
            or qc_report.get("status") not in ("VALID", "VALID_WITH_WARNINGS")
            or not qc_report.get("counted_in_campaign")
            or qc_report.get("infrastructure_errors") or qc_report.get("protocol_errors")):
        raise ValueError("fresh QC does not qualify the same attempt for campaign inclusion")
    if (attempt_dir / "qc_report_pre_revalidation.json").exists():
        raise FileExistsError("pre-revalidation QC snapshot already exists")
    if (attempt_dir / "runner_result_pre_revalidation.json").exists():
        raise FileExistsError("pre-revalidation runner snapshot already exists")

    old_qc_sha256 = _sha256(report_path)
    old_result_sha256 = _sha256(result_path)
    for source_path, snapshot_name in (
        (report_path, "qc_report_pre_revalidation.json"),
        (result_path, "runner_result_pre_revalidation.json"),
    ):
        with source_path.open("rb") as source_handle, (attempt_dir / snapshot_name).open("xb") as target_handle:
            target_handle.write(source_handle.read())
            target_handle.flush()
            os.fsync(target_handle.fileno())

    _atomic_json(report_path, qc_report)
    updated_result = {
        **previous_result,
        "qc_status": qc_report["status"],
        "qc_revalidated": True,
        "previous_qc_status": previous["status"],
        "previous_protocol_errors": previous_errors,
    }
    _atomic_json(result_path, updated_result)
    qc_validator = Path(__file__).resolve().with_name("qc_attempt.py")
    revalidation = {
        "schema_version": "stereo_v1_1_qc_revalidation_v1",
        "run_id": run_id,
        "attempt_id": attempt_id,
        "reason": "corrected workbook Vehicle lookup in QC; source attempt retained",
        "previous_qc_status": previous["status"],
        "previous_protocol_errors": previous_errors,
        "previous_qc_report_sha256": old_qc_sha256,
        "previous_runner_result_sha256": old_result_sha256,
        "revalidated_qc_status": qc_report["status"],
        "qc_validator_sha256": _sha256(qc_validator),
        "revalidated_utc_ns": time.time_ns(),
    }
    _write_new(attempt_dir / "qc_revalidation.json", revalidation)
    _write_attempt_checksums(attempt_dir)

    result_status = qc_report["status"]
    failure = None
    attempt["status"] = result_status
    attempt["failure_reason"] = failure
    actual_matches[0]["status"] = result_status
    actual_matches[0]["failure_reason"] = failure
    row["status"] = result_status
    row["failure_reason"] = failure
    _atomic_json(output_root / "manifest_actual.json", actual)
    _refresh_counts(output_root, status)
    _atomic_json(output_root / "run_status.json", status)
    _append_change(
        output_root, "attempt_qc_revalidated_after_validator_fix",
        run_id=run_id, attempt_id=attempt_id,
        previous_qc_status=previous["status"], revalidated_qc_status=result_status,
        previous_protocol_errors=previous_errors,
        previous_qc_report_sha256=old_qc_sha256,
        qc_validator_sha256=revalidation["qc_validator_sha256"],
    )
    return row


def _is_workbook_plan(plan: dict) -> bool:
    return plan.get("schema_version") in {
        "stereo_v1_1_workbook_campaign_plan_v1",
        "stereo_v1_1_workbook_campaign_plan_v2",
        "stereo_v1_1_workbook_campaign_plan_v3",
    }


def initialize(output_root: Path, plan: dict | None = None) -> dict:
    output_root = output_root.resolve()
    if not output_root.is_dir():
        raise ValueError("campaign output root must already exist")
    targets = ("run_status.json", "manifest_actual.json", "qc_summary.json", "changes_log.jsonl",
               "checksums.sha256", "schema_mapping.json", "frozen_protocol")
    if any((output_root / target).exists() for target in targets):
        raise FileExistsError("campaign status files already exist; preserving previous state")
    plan = plan or build_plan()
    planned_runs = plan["planned_runs"]
    planned_count = int(plan.get("planned_run_count", len(planned_runs)))
    if planned_count != len(planned_runs) or planned_count <= 0:
        raise ValueError("campaign plan has an inconsistent or empty run list")
    if len({row["run_id"] for row in planned_runs}) != planned_count:
        raise ValueError("campaign plan run IDs are not unique")
    workbook_plan = _is_workbook_plan(plan)
    _write_frozen_protocol(output_root, include_protocol=not workbook_plan)
    slots = []
    for row in planned_runs:
        slots.append({
            **row,
            "seed_effective": None,
            "attempts": [],
            "failure_reason": None,
        })
    status = {
        "schema_version": ("stereo_v1_1_workbook_run_status_v1"
                            if workbook_plan
                            else "stereo_v1_1_run_status_v1"),
        "protocol_sha256": plan.get("protocol_sha256"),
        "workbook_sha256": plan.get("workbook_sha256"),
        "campaign_plan_sha256": hashlib.sha256(json.dumps(
            plan, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")).hexdigest(),
        "planned_run_count": planned_count,
        "valid_run_count": 0,
        "run_slots": slots,
    }
    manifest_actual = {
        "schema_version": "stereo_v1_1_manifest_actual_v1",
        "planned_run_count": planned_count,
        "actual_attempt_count": 0,
        "attempts": [],
    }
    qc_summary = {
        "schema_version": "stereo_v1_1_qc_summary_v1",
        "planned_run_count": planned_count,
        "status_counts": {"PLANNED": planned_count},
        "valid_run_count": 0,
        "technical_failure_count": 0,
        "development_attempts_counted": 0,
    }
    _write_new(output_root / "run_status.json", status)
    _write_new(output_root / "manifest_actual.json", manifest_actual)
    _write_new(output_root / "qc_summary.json", qc_summary)
    event = {
        "event": "campaign_slots_initialized",
        "utc_ns": time.time_ns(),
        "planned_run_count": planned_count,
        "valid_run_count": 0,
        "source": (
            "manifest_xlsx_workbook" if plan.get("workbook_schema_version") == "sweep_manifest_v1" else
            "numeric_xlsx_workbook" if workbook_plan else
            "frozen_v1_1_manifest_plan"
        ),
        "workbook_sha256": plan.get("workbook_sha256"),
    }
    with (output_root / "changes_log.jsonl").open("x", encoding="utf-8") as handle:
        handle.write(json.dumps(event, ensure_ascii=False, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    return qc_summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(initialize(args.output_root), ensure_ascii=False))


if __name__ == "__main__":
    main()
