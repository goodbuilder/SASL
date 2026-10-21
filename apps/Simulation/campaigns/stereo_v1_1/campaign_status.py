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
             "output": "clock.jsonl.simulation_timestamp_ns",
             "raw_bag_output_v1_2": "raw_images_or_lossless_bag/state",
             "contract": "Gazebo simulation clock; reset increments simulation_epoch"},
            {"topic": "/simulation/ground_truth/vehicle_reference_link",
             "message": "geometry_msgs/msg/PoseStamped", "output": "ground_truth.jsonl",
             "raw_bag_output_v1_2": "raw_images_or_lossless_bag/state",
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
            {"topic": "/left_camera/camera_info", "message": "sensor_msgs/msg/CameraInfo",
             "output": "raw_images_or_lossless_bag/left", "contract": "optical RDF intrinsics; row-major K in pixels"},
            {"topic": "/right_camera/camera_info", "message": "sensor_msgs/msg/CameraInfo",
             "output": "raw_images_or_lossless_bag/right", "contract": "optical RDF intrinsics; row-major K in pixels"},
            {"topic": "/simulation/camera_pose_diagnostics/left", "message": "std_msgs/msg/String",
             "output": "camera_truth.json.camera_payloads.left",
             "raw_bag_output_v1_2": "raw_images_or_lossless_bag/state",
             "contract": "center world ENU; observed_optical_orientation_world_from_entity_xyzw maps camera-link FLU to ENU; convert local axes explicitly to optical RDF"},
            {"topic": "/simulation/camera_pose_diagnostics/right", "message": "std_msgs/msg/String",
             "output": "camera_truth.json.camera_payloads.right",
             "raw_bag_output_v1_2": "raw_images_or_lossless_bag/state",
             "contract": "center world ENU; observed_optical_orientation_world_from_entity_xyzw maps camera-link FLU to ENU; convert local axes explicitly to optical RDF"},
        ],
        "offline_observation_fields": {
            "raw_state_index.jsonl": "v1.2 deserialized raw-bag clock/GT/camera messages; simulation ns and epoch kept separate from rosbag receive UTC ns",
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


def authorize_reviewed_retry(output_root: Path, run_id: str, *, user_instruction: str,
                             evidence_paths: list[str]) -> dict:
    """Record an explicit user-directed one-attempt exception after failure review.

    This does not certify repair, reclassify failures, or modify acceptance QC.
    Caller holds the runner lock. Evidence paths are campaign-relative and hashed.
    """
    output_root = output_root.resolve()
    status = _load(output_root / "run_status.json")
    row = _slot(status, run_id)
    if not user_instruction.strip() or not evidence_paths:
        raise ValueError("explicit user instruction and review evidence required")
    if row["status"] != "INFRASTRUCTURE_FAILED" or len(row["attempts"]) < 2 or not all(
            item["status"] == "INFRASTRUCTURE_FAILED" for item in row["attempts"][-2:]):
        raise ValueError("reviewed retry only applies to two consecutive technical failures")
    next_id = f"a{len(row['attempts']) + 1:02d}"
    authorizations = row.setdefault("reviewed_retry_authorizations", [])
    if any(item["attempt_id"] == next_id for item in authorizations):
        raise ValueError("retry authorization already recorded for this attempt")
    evidence = {}
    for name in evidence_paths:
        path = (output_root / name).resolve()
        if Path(name).is_absolute() or not path.is_relative_to(output_root) or not path.is_file():
            raise ValueError("review evidence must be an existing campaign-relative file")
        evidence[name] = _sha256(path)
    entry = {"attempt_id": next_id, "reviewed_attempt_ids": [x["attempt_id"] for x in row["attempts"]],
             "user_instruction": user_instruction, "evidence_sha256": evidence,
             "cause_repaired": False, "qc_unchanged": True, "used": False,
             "authorized_utc_ns": time.time_ns()}
    authorizations.append(entry)
    _atomic_json(output_root / "run_status.json", status)
    _append_change(output_root, "user_authorized_reviewed_retry", run_id=run_id, **entry)
    return entry


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
    retry_authorization = None
    if len(row["attempts"]) >= 2 and all(
        attempt["status"] == "INFRASTRUCTURE_FAILED"
        for attempt in row["attempts"][-2:]
    ):
        expected_id = f"a{len(row['attempts']) + 1:02d}"
        for item in row.get("reviewed_retry_authorizations", []):
            if (item["attempt_id"] == expected_id and not item["used"]
                    and item["reviewed_attempt_ids"] == [x["attempt_id"] for x in row["attempts"]]
                    and item.get("user_instruction") and item.get("evidence_sha256")
                    and all((output_root / name).is_file() and _sha256(output_root / name) == digest
                            for name, digest in item["evidence_sha256"].items())):
                retry_authorization = item
                break
        if retry_authorization is None:
            raise ValueError("two consecutive technical failures; document and repair the causes before retry")
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
    if retry_authorization is not None:
        retry_authorization["used"] = True
        retry_authorization["used_utc_ns"] = time.time_ns()
        record["reviewed_retry_authorization"] = retry_authorization.copy()
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
    return _revalidate_qc_regression(output_root, run_id, attempt_id, qc_report)


def revalidate_cadence_qc_regression(output_root: Path, run_id: str, attempt_id: str,
                                    qc_report: dict) -> dict:
    """Recover only the v1.2 cadence-only false positive, never missing slots."""
    return _revalidate_qc_regression(output_root, run_id, attempt_id, qc_report,
                                     cadence_slot_fix=True)


def _revalidate_qc_regression(output_root: Path, run_id: str, attempt_id: str,
                              qc_report: dict, *, cadence_slot_fix: bool = False) -> dict:
    """Accept a corrected QC result for a preserved workbook attempt.

    This path is limited to the named Vehicle-alias or cadence-slot validator
    regressions. Each recipe requires its exact original error set and fresh QC.
    It never starts another flight or changes the captured source data.
    """
    output_root = output_root.resolve()
    status = _load(output_root / "run_status.json")
    actual = _load(output_root / "manifest_actual.json")
    if status.get("schema_version") != "stereo_v1_1_workbook_run_status_v1":
        raise ValueError("QC regression revalidation is restricted to workbook campaigns")
    row = _slot(status, run_id)
    previous_status = "INFRASTRUCTURE_FAILED" if cadence_slot_fix else "PROTOCOL_DEVIATION"
    if row.get("status") != previous_status or not row.get("attempts"):
        raise ValueError("slot is not a completed protocol-deviation attempt")
    attempt = row["attempts"][-1]
    if attempt.get("attempt_id") != attempt_id or attempt.get("status") != previous_status:
        raise ValueError("attempt is not the slot's latest protocol-deviation attempt")
    actual_matches = [item for item in actual.get("attempts", [])
                      if item.get("run_id") == run_id and item.get("attempt_id") == attempt_id]
    if len(actual_matches) != 1 or actual_matches[0].get("status") != previous_status:
        raise ValueError("manifest_actual does not contain one matching failed attempt")

    attempt_dir = output_root / attempt["recording_path"]
    report_path = attempt_dir / "qc_report.json"
    result_path = attempt_dir / "runner_result.json"
    if not report_path.is_file() or not result_path.is_file():
        raise ValueError("attempt is missing its original QC or runner result")
    previous = _load(report_path)
    previous_result = _load(result_path)
    previous_errors = previous.get("protocol_errors", [])
    known_error = (set(previous_errors) == REVALIDATABLE_QC_REGRESSION_ERRORS
                   and len(previous_errors) == len(REVALIDATABLE_QC_REGRESSION_ERRORS)
                   and not previous.get("infrastructure_errors"))
    if cadence_slot_fix:
        known_error = (previous.get("infrastructure_errors") ==
                       ["science_setpoint_cadence_below_v12_requirement"]
                       and not previous_errors and not previous_result.get("runner_failure_reason"))
        manifest_path = attempt_dir / "run_manifest.json"
        if not manifest_path.is_file() or _load(manifest_path).get("protocol_version") != "1.2":
            raise ValueError("cadence recovery requires a v1.2 source attempt")
        cadence = qc_report.get("evidence", {}).get("science_setpoint_cadence_recomputed", {})
        if (cadence.get("criterion") != "reference_slot_coverage_v2"
                or cadence.get("missing_full_slot_count") != 0
                or cadence.get("sufficient_unique_simulation_cadence") is not True):
            raise ValueError("fresh cadence QC must substantiate all reference slots")
        if not (attempt_dir / "checksums.sha256").is_file():
            raise ValueError("original attempt checksums are required for cadence recovery")
        for line in (attempt_dir / "checksums.sha256").read_text(encoding="utf-8").splitlines():
            digest, relative = line.split("  ", 1)
            source_file = (attempt_dir / relative).resolve()
            if (not source_file.is_relative_to(attempt_dir.resolve())
                    or not source_file.is_file() or _sha256(source_file) != digest):
                raise ValueError("original source checksums must pass before cadence recovery")
    if (previous.get("run_id") != run_id or previous.get("attempt_id") != attempt_id
            or previous.get("status") != previous_status or not known_error
            or previous_result.get("qc_status") != previous_status):
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
    if cadence_slot_fix and (attempt_dir / "checksums_pre_revalidation.sha256").exists():
        raise FileExistsError("pre-revalidation checksum snapshot already exists")

    old_qc_sha256 = _sha256(report_path)
    old_result_sha256 = _sha256(result_path)
    snapshots = [
        (report_path, "qc_report_pre_revalidation.json"),
        (result_path, "runner_result_pre_revalidation.json"),
    ]
    if cadence_slot_fix:
        snapshots.append((attempt_dir / "checksums.sha256", "checksums_pre_revalidation.sha256"))
    for source_path, snapshot_name in snapshots:
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
        "previous_infrastructure_errors": previous.get("infrastructure_errors", []),
    }
    _atomic_json(result_path, updated_result)
    qc_validator = Path(__file__).resolve().with_name("qc_attempt.py")
    revalidation = {
        "schema_version": "stereo_v1_1_qc_revalidation_v1",
        "run_id": run_id,
        "attempt_id": attempt_id,
        "reason": ("corrected unsupported 30 ms hard gate to anchored 50 Hz reference-slot coverage; raw source retained"
                   if cadence_slot_fix else "corrected workbook Vehicle lookup in QC; source attempt retained"),
        "previous_qc_status": previous["status"],
        "previous_protocol_errors": previous_errors,
        "previous_infrastructure_errors": previous.get("infrastructure_errors", []),
        "previous_qc_report_sha256": old_qc_sha256,
        "previous_runner_result_sha256": old_result_sha256,
        "revalidated_qc_status": qc_report["status"],
        "qc_validator_sha256": _sha256(qc_validator),
        "revalidated_utc_ns": time.time_ns(),
    }
    if cadence_slot_fix:
        revalidation["criterion"] = "reference_slot_coverage_v2"
        revalidation["cadence_evaluator_sha256"] = _sha256(Path(__file__).with_name("setpoint_cadence.py"))
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


def invalidate_pre_science_attempt(output_root: Path, run_id: str, attempt_id: str,
                                    qc_report: dict) -> dict:
    """Correct a v1.2 false-valid nonstart; retain raw and original QC evidence.

    Caller holds the runner lock. This never promotes data or grants a rerun for
    poor in-science performance. All original hashes must pass before mutation.
    """
    output_root = output_root.resolve()
    status = _load(output_root / "run_status.json")
    actual = _load(output_root / "manifest_actual.json")
    row = _slot(status, run_id)
    if row["status"] not in ("VALID", "VALID_WITH_WARNINGS") or not row["attempts"]:
        raise ValueError("only a previously counted latest attempt can be invalidated")
    attempt = row["attempts"][-1]
    if attempt["attempt_id"] != attempt_id:
        raise ValueError("attempt must be latest")
    directory = output_root / attempt["recording_path"]
    controller = _load(directory / "controller_result.json")
    events = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
    if (_load(directory / "run_manifest.json").get("protocol_version") != "1.2"
            or controller.get("controller_status") == "science_completed"
            or any(item.get("event") == "science_start" for item in events)):
        raise ValueError("in-science performance outcomes must not be invalidated")
    if (qc_report.get("run_id") != run_id or qc_report.get("attempt_id") != attempt_id
            or qc_report.get("status") != "INFRASTRUCTURE_FAILED"
            or "pre_science_execution_failure" not in qc_report.get("infrastructure_errors", [])
            or qc_report.get("counted_in_campaign") or qc_report.get("counted_in_39")):
        raise ValueError("fresh QC must establish pre-science execution failure")
    matches = [item for item in actual["attempts"]
               if item["run_id"] == run_id and item["attempt_id"] == attempt_id]
    previous = _load(directory / "qc_report.json")
    previous_result = _load(directory / "runner_result.json")
    if (len(matches) != 1 or matches[0]["status"] != row["status"]
            or previous.get("status") != row["status"]
            or previous_result.get("qc_status") != row["status"]):
        raise ValueError("saved attempt/status/QC identity disagreement")
    checksum_rows = (directory / "checksums.sha256").read_text().splitlines()
    listed = set()
    for line in checksum_rows:
        digest, relative = line.split("  ", 1)
        path = (directory / relative).resolve()
        if relative in listed or not path.is_relative_to(directory.resolve()) or not path.is_file() or _sha256(path) != digest:
            raise ValueError("original attempt hash mismatch")
        listed.add(relative)
    expected = {str(path.relative_to(directory)) for path in directory.rglob("*")
                if path.is_file() and path.name != "checksums.sha256"}
    if not listed or not expected.issubset(listed):
        raise ValueError("original checksum coverage incomplete")
    snapshot_names = ("qc_report.json", "runner_result.json", "checksums.sha256")
    for name in snapshot_names:
        if (directory / (name + ".before_entry_reclassification")).exists():
            raise FileExistsError("original entry classification snapshot already exists")
    for name in snapshot_names:
        with (directory / (name + ".before_entry_reclassification")).open("xb") as f:
            f.write((directory / name).read_bytes())
            f.flush()
            os.fsync(f.fileno())
    reassessment = {"method": "v12_pre_science_classification_correction",
                    "previous_status": row["status"], "new_status": "INFRASTRUCTURE_FAILED",
                    "controller_failure_reason": controller.get("failure_reason"),
                    "raw_preserved": True, "qc_validator_sha256": _sha256(Path(__file__).with_name("qc_attempt.py")),
                    "utc_ns": time.time_ns()}
    _write_new(directory / "entry_classification_reassessment.json", reassessment)
    _atomic_json(directory / "qc_report.json", qc_report)
    _atomic_json(directory / "runner_result.json", {
        **previous_result, "qc_status": "INFRASTRUCTURE_FAILED",
        "previous_qc_status": previous["status"],
        "entry_classification_reassessment": "entry_classification_reassessment.json"})
    _write_attempt_checksums(directory)
    for target in (attempt, matches[0], row):
        target["status"] = "INFRASTRUCTURE_FAILED"
        target["failure_reason"] = "pre_science_execution_failure: " + str(controller.get("failure_reason"))
        target["entry_classification_reassessment"] = str((directory / "entry_classification_reassessment.json").relative_to(output_root))
    _atomic_json(output_root / "manifest_actual.json", actual)
    _refresh_counts(output_root, status)
    _atomic_json(output_root / "run_status.json", status)
    _append_change(output_root, "attempt_entry_failure_reclassified", run_id=run_id,
                   attempt_id=attempt_id, **reassessment)
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
