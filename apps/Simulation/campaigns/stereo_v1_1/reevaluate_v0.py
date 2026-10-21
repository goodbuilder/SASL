#!/usr/bin/env python3
"""Reuse frozen calibration Monte Carlo results for a new V0 recording.

This module is deliberately separate from ``analyze_campaign.py`` because that
file's content hash is part of the frozen analysis identity. ``prepare`` copies
only the matching calibration result bundles and the V0 source package into a
new sibling folder. After the existing pinned YOLO postprocessor has generated
observations in that folder, ``evaluate`` runs only the post-freeze V0 evaluator.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sys
import time

from campaigns.stereo_v1_1 import analyze_campaign as analysis
from campaigns.stereo_v1_1 import generate_sensor_observations as camera_sensor_generator


CAMPAIGN_ROOT = Path(__file__).resolve().parent
SIMULATION_ROOT = CAMPAIGN_ROOT.parents[2]
REPO_ROOT = SIMULATION_ROOT.parents[1]
SCHEMA_VERSION = "stereo_v1_1_v0_reevaluation_v1"
PILOT_SCHEMA = "spawn_n600_vertical_v0_pilot_v1"
EXPECTED_MC_TRIALS = 100
SCOPES = (analysis.ALL_DATA, analysis.MATCHED_DATA)
PILOT_SUPPORT_FILES = (
    "pilot_manifest.json", "pilot_result.json", "workbook_parameters.json",
    "effective_scenario.env", "flight_reference.json", "ground_truth.jsonl",
    "events.jsonl", "camera_truth.json", "coordinate_frame_check.json",
    "controller_result.json", "raw_capture_summary.json",
    "state_capture_summary.json", "controller_timing_diagnostics_summary.json",
)
PILOT_RAW_PATH = "raw_images_or_lossless_bag"


def _json_text(payload: object) -> str:
    return json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True,
                      allow_nan=False) + "\n"


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _read_jsonl(path: Path) -> list[dict]:
    return analysis._read_jsonl(path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_immutable(path: Path, payload: object) -> str:
    return analysis._write_immutable(path, payload)


def _copy_verified(source: Path, target: Path) -> str:
    """Byte-copy an immutable input, verify it, and refuse conflicting reuse."""
    source = Path(source).resolve(strict=True)
    target = Path(target).resolve(strict=False)
    if not source.is_file():
        raise ValueError(f"copy source is not a regular file: {source}")
    source_hash = _sha256(source)
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        if not target.is_file() or _sha256(target) != source_hash:
            raise FileExistsError(f"reevaluation artifact exists with different bytes: {target}")
        return source_hash
    temporary = target.with_name(f".{target.name}.copy-{os.getpid()}")
    try:
        with source.open("rb") as source_handle, temporary.open("xb") as target_handle:
            shutil.copyfileobj(source_handle, target_handle, length=1024 * 1024)
            target_handle.flush()
            os.fsync(target_handle.fileno())
        if _sha256(temporary) != source_hash or _sha256(source) != source_hash:
            raise ValueError(f"copy hash mismatch or source changed: {source}")
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
    return source_hash


def _write_jsonl_immutable(path: Path, rows) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = "".join(
        json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n"
        for row in rows
    )
    digest = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
    if path.exists():
        if path.read_text(encoding="utf-8") != encoded:
            raise FileExistsError(f"reevaluation index exists with different content: {path}")
    else:
        with path.open("x", encoding="utf-8") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
    return digest


def _append_event(path: Path, event: str, **fields) -> None:
    row = {"timestamp_utc": datetime.now(timezone.utc).isoformat(),
           "event": event, **fields}
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
        handle.flush()
        os.fsync(handle.fileno())


def _relative(path: Path, root: Path) -> str:
    resolved = Path(path).resolve(strict=True)
    base = Path(root).resolve(strict=True)
    if not resolved.is_relative_to(base):
        raise ValueError(f"path is outside its declared root: {path}")
    return resolved.relative_to(base).as_posix()


def _safe_join(root: Path, relative: str) -> Path:
    path = Path(relative)
    if path.is_absolute():
        raise ValueError(f"absolute path is forbidden in reevaluation manifests: {relative}")
    resolved = (Path(root) / path).resolve(strict=False)
    if not resolved.is_relative_to(Path(root).resolve()):
        raise ValueError(f"relative path escapes reevaluation root: {relative}")
    return resolved


def _slot_attempt_checksum(root: Path, slot: dict) -> str:
    attempt = slot["attempts"][-1]
    checksum = Path(root) / attempt["recording_path"] / "checksums.sha256"
    if not checksum.is_file():
        raise FileNotFoundError(f"source attempt checksum file is missing: {slot['run_id']}")
    return _sha256(checksum)


def _select_calibration_batches(index: dict, slots: list[dict], group: str,
                                repetition: int, mc_trials: int) -> tuple[list[dict], list[dict]]:
    """Select and validate the complete calibration grid paired to one V0 slot."""
    calibration_slots = [slot for slot in slots
                         if slot.get("path_id") != "V0"
                         and slot.get("rig_group") == group
                         and int(slot.get("repetition", -1)) == repetition]
    if not calibration_slots:
        raise ValueError(f"no calibration runs for {group} repetition {repetition}")
    run_ids = {slot["run_id"] for slot in calibration_slots}
    rows = [dict(row) for row in index.get("batches", []) if row.get("run_id") in run_ids]
    if not rows:
        raise ValueError("frozen calibration index has no matching batches")
    seen = set()
    batches_by_run = defaultdict(set)
    yaw_by_run = defaultdict(set)
    for row in rows:
        key = (row.get("run_id"), row.get("trial_index"), row.get("yaw_case"), row.get("data_scope"))
        if key in seen:
            raise ValueError(f"duplicate frozen calibration batch: {key}")
        seen.add(key)
        if row.get("data_scope") not in SCOPES:
            raise ValueError(f"unexpected calibration sample scope: {row.get('data_scope')}")
        trial = row.get("trial_index")
        if isinstance(trial, bool) or not isinstance(trial, int) or not 0 <= trial < mc_trials:
            raise ValueError(f"invalid MC trial index in frozen calibration index: {trial}")
        batches_by_run[row["run_id"]].add((trial, row["yaw_case"], row["data_scope"]))
        yaw_by_run[row["run_id"]].add(row["yaw_case"])
        relative = row.get("case_directory")
        if not isinstance(relative, str) or Path(relative).is_absolute():
            raise ValueError(f"invalid frozen calibration case path: {relative}")
    expected_runs = {slot["run_id"] for slot in calibration_slots}
    if {row["run_id"] for row in rows} != expected_runs:
        raise ValueError("frozen calibration index does not cover every matching P route")
    common_yaw_cases = None
    for run_id in sorted(expected_runs):
        expected_yaw = yaw_by_run[run_id]
        if common_yaw_cases is None:
            common_yaw_cases = expected_yaw
        elif expected_yaw != common_yaw_cases:
            raise ValueError(f"yaw case grid differs between calibration paths for {run_id}")
        expected_keys = {(trial, yaw, scope)
                         for trial in range(mc_trials)
                         for yaw in expected_yaw
                         for scope in SCOPES}
        if batches_by_run[run_id] != expected_keys:
            missing = len(expected_keys - batches_by_run[run_id])
            extra = len(batches_by_run[run_id] - expected_keys)
            raise ValueError(f"incomplete frozen MC batch grid for {run_id}: missing={missing}, extra={extra}")
    rows.sort(key=lambda row: (row["run_id"], row["trial_index"], row["yaw_case"], row["data_scope"]))
    return calibration_slots, rows


def _validate_parent(campaign_root: Path, parent_analysis_root: Path,
                     source_run_id: str) -> dict:
    campaign_root = Path(campaign_root).resolve(strict=True)
    parent_analysis_root = Path(parent_analysis_root).resolve(strict=True)
    if not parent_analysis_root.is_dir():
        raise ValueError("parent analysis root must be a directory")
    expected_parent = campaign_root / "analysis_versions"
    if not parent_analysis_root.is_relative_to(expected_parent.resolve(strict=True)):
        raise ValueError("parent analysis must belong to this campaign's analysis_versions directory")

    status, slots = analysis._status_slots(campaign_root)
    protocol, source_protocol = analysis._load_analysis_protocol(campaign_root, status)
    freeze_path = parent_analysis_root / "analysis_freeze.json"
    run_manifest_path = parent_analysis_root / "analysis_run_manifest.json"
    summary_path = parent_analysis_root / "analysis_summary.json"
    index_path = parent_analysis_root / "calibration_run_index.json"
    calibration_freeze_path = parent_analysis_root / "calibration_freeze_manifest.json"
    path_selection_path = parent_analysis_root / "path_selection.json"
    for path in (freeze_path, run_manifest_path, summary_path, index_path,
                 calibration_freeze_path, path_selection_path):
        if not path.is_file():
            raise FileNotFoundError(f"frozen parent analysis artifact missing: {path.name}")
    freeze = _read_json(freeze_path)
    run_manifest = _read_json(run_manifest_path)
    summary = _read_json(summary_path)
    index = _read_json(index_path)
    calibration_freeze_manifest = _read_json(calibration_freeze_path)
    path_selection = _read_json(path_selection_path)
    freeze_hash = _sha256(freeze_path)

    if freeze.get("status") != "FROZEN_BEFORE_V0_POSITION_EVALUATION":
        raise ValueError("parent analysis was not frozen before its original V0 evaluation")
    if freeze.get("v0_position_error_read_before_freeze") is not False:
        raise ValueError("parent analysis does not satisfy the V0 holdout freeze contract")
    if freeze.get("mc_trial_count") != EXPECTED_MC_TRIALS:
        raise ValueError(f"parent analysis must contain exactly {EXPECTED_MC_TRIALS} MC trials")
    if freeze.get("algorithm_sha256") != analysis._analysis_hashes():
        raise ValueError("current analysis code differs from the frozen parent algorithm hashes")
    if freeze.get("dependencies") != analysis._analysis_dependencies():
        raise ValueError("current analysis dependencies differ from the frozen parent dependency versions")
    if run_manifest.get("algorithm_sha256") != freeze.get("algorithm_sha256"):
        raise ValueError("parent run manifest and freeze algorithm identities differ")
    if run_manifest.get("dependencies") != freeze.get("dependencies"):
        raise ValueError("parent run manifest and freeze dependency identities differ")
    if run_manifest.get("mc_trial_count") != freeze.get("mc_trial_count"):
        raise ValueError("parent run manifest and freeze MC trial counts differ")
    if source_protocol != freeze.get("source_protocol"):
        raise ValueError("current campaign protocol differs from the frozen parent protocol")
    if summary.get("analysis_freeze_sha256") != freeze_hash:
        raise ValueError("parent analysis summary points to a different freeze")
    if _sha256(calibration_freeze_path) != freeze.get("primary_calibration_checksums_manifest_sha256"):
        raise ValueError("parent calibration freeze manifest hash differs from the freeze")
    selected_hash = hashlib.sha256(analysis._json_text(path_selection).encode("utf-8")).hexdigest()
    if selected_hash != freeze.get("path_selection_sha256"):
        raise ValueError("parent path selection differs from its frozen hash")
    if (path_selection.get("selected_path_id") != freeze.get("selected_path_id")
            or summary.get("selected_path_id") != freeze.get("selected_path_id")):
        raise ValueError("parent summary/path selection differs from frozen selected path")

    for relative, expected in run_manifest.get("source_campaign_manifest_sha256", {}).items():
        source = campaign_root / relative
        if not source.is_file() or _sha256(source) != expected:
            raise ValueError(f"frozen source campaign manifest changed: {relative}")
    base_v0_slot = analysis._slot_by_id(slots, source_run_id)
    if base_v0_slot.get("path_id") != "V0":
        raise ValueError(f"pilot source run is not a V0 slot: {source_run_id}")
    group = str(base_v0_slot["rig_group"])
    repetition = int(base_v0_slot["repetition"])
    source_attempt_id = analysis._attempt_id(base_v0_slot)
    expected_v0_checksum = run_manifest.get("source_attempt_checksums_sha256", {}).get(
        f"{source_run_id}/{source_attempt_id}")
    if expected_v0_checksum is None or _slot_attempt_checksum(campaign_root, base_v0_slot) != expected_v0_checksum:
        raise ValueError("source V0 slot differs from the attempt paired with the frozen parent analysis")

    calibration_slots, batches = _select_calibration_batches(
        index, slots, group, repetition, int(freeze["mc_trial_count"]),
    )
    source_attempts = run_manifest.get("source_attempt_checksums_sha256", {})
    for slot in calibration_slots:
        attempt_id = analysis._attempt_id(slot)
        key = f"{slot['run_id']}/{attempt_id}"
        expected = source_attempts.get(key)
        if expected is None or _slot_attempt_checksum(campaign_root, slot) != expected:
            raise ValueError(f"calibration source changed since the parent freeze: {key}")
    return {
        "campaign_root": campaign_root,
        "parent_analysis_root": parent_analysis_root,
        "status": status,
        "slots": slots,
        "protocol": protocol,
        "source_protocol": source_protocol,
        "freeze": freeze,
        "freeze_hash": freeze_hash,
        "run_manifest": run_manifest,
        "summary": summary,
        "index": index,
        "calibration_freeze_manifest": calibration_freeze_manifest,
        "path_selection": path_selection,
        "base_v0_slot": base_v0_slot,
        "group": group,
        "repetition": repetition,
        "calibration_slots": calibration_slots,
        "batches": batches,
        "mc_trial_count": int(freeze["mc_trial_count"]),
    }


def _condition_and_pilot(campaign_root: Path, pilot_attempt_dir: Path,
                         parent: dict) -> dict:
    campaign_root = Path(campaign_root).resolve(strict=True)
    pilot_attempt_dir = Path(pilot_attempt_dir).resolve(strict=True)
    if not pilot_attempt_dir.is_relative_to(campaign_root):
        raise ValueError("pilot attempt must be stored under the campaign recordings root")
    pilot = _read_json(pilot_attempt_dir / "pilot_manifest.json")
    if pilot.get("schema_version") != PILOT_SCHEMA:
        raise ValueError(f"unsupported V0 pilot manifest: {pilot.get('schema_version')}")
    if pilot.get("path_id") != "V0" or pilot.get("counted_in_campaign") is not False:
        raise ValueError("V0 input must be explicitly marked as a development-only V0 pilot")
    if pilot.get("source_run_id") != parent["base_v0_slot"]["run_id"]:
        raise ValueError("pilot source_run_id does not match the frozen V0 group/repetition")
    if pilot.get("run_id") == pilot.get("source_run_id"):
        raise ValueError("development V0 must have its own run ID")

    names = set(PILOT_SUPPORT_FILES)
    missing = [name for name in names if not (pilot_attempt_dir / name).is_file()]
    if missing:
        raise FileNotFoundError(f"pilot evidence files missing: {', '.join(sorted(missing))}")
    raw_capture = _read_json(pilot_attempt_dir / "raw_capture_summary.json")
    controller = _read_json(pilot_attempt_dir / "controller_result.json")
    pilot_result = _read_json(pilot_attempt_dir / "pilot_result.json")
    frame_check = _read_json(pilot_attempt_dir / "coordinate_frame_check.json")
    if raw_capture.get("status") != "CAPTURE_COMPLETE" or raw_capture.get("protocol_version") != "1.2":
        raise ValueError("V0 pilot does not have a finalized v1.2 lossless capture")
    if controller.get("controller_status") != "science_completed" or pilot_result.get("status") != "science_completed":
        raise ValueError("V0 pilot did not complete its science reference")
    if frame_check.get("status") != "PASS":
        raise ValueError("V0 pilot failed its explicit ENU/NED frame check")
    if controller.get("physical_landing_verified") is True:
        landing_status = "verified"
    else:
        landing_status = "not_verified_by_pilot"

    source_files: dict[str, str] = {}
    copy_files = set(names)
    for stream in raw_capture.get("streams", {}).values():
        for artifact in stream.get("files", []):
            relative = artifact.get("path_from_attempt_root")
            if not isinstance(relative, str):
                raise ValueError("raw capture summary has a malformed file entry")
            source = _safe_join(pilot_attempt_dir, relative)
            if not source.is_file() or source.stat().st_size != int(artifact["size_bytes"]):
                raise ValueError(f"raw capture artifact size differs: {relative}")
            digest = _sha256(source)
            if digest != artifact.get("sha256"):
                raise ValueError(f"raw capture artifact hash differs: {relative}")
            source_files[relative] = digest
            copy_files.add(relative)
    for name in names:
        source_files[name] = _sha256(pilot_attempt_dir / name)

    base_slot = parent["base_v0_slot"]
    base_attempt_dir = analysis._attempt_dir(campaign_root, base_slot)
    base_parameters = _read_json(base_attempt_dir / "workbook_parameters.json")
    new_parameters = _read_json(pilot_attempt_dir / "workbook_parameters.json")
    base_vehicle = base_parameters["Vehicle"]
    new_vehicle = new_parameters["Vehicle"]
    for section in ("CameraModel", "Capture", "Entry", "FrameGate", "Motion"):
        if base_parameters.get(section) != new_parameters.get(section):
            raise ValueError(f"V0 pilot changed frozen non-spawn parameters in {section}")
    for field in ("model_id", "reference_link", "sequence", "spawn_east_m",
                  "spawn_yaw_deg", "verify_spawn_yaw"):
        if base_vehicle.get(field) != new_vehicle.get(field):
            raise ValueError(f"V0 pilot changed non-position vehicle parameter {field}")
    if not math.isclose(float(new_vehicle["spawn_north_m"]), 600.0, abs_tol=1e-9):
        raise ValueError("approved development V0 condition requires spawn_north_m=600")
    pose = [float(new_vehicle[f"spawn_{axis}_m"]) for axis in ("east", "north", "up")]
    if [float(value) for value in pilot.get("model_pose_world_enu_m", [])] != pose:
        raise ValueError("pilot model pose and workbook Vehicle spawn ENU fields disagree")
    new_reference_hash = _sha256(pilot_attempt_dir / "flight_reference.json")
    base_reference_hash = _sha256(base_attempt_dir / "flight_reference.json")
    if new_reference_hash != base_reference_hash:
        raise ValueError("V0 pilot science flight reference differs from the frozen source V0")
    start_events = [row for row in analysis._read_jsonl(pilot_attempt_dir / "events.jsonl")
                    if row.get("event") == "science_start"]
    end_events = [row for row in analysis._read_jsonl(pilot_attempt_dir / "events.jsonl")
                  if row.get("event") == "science_end"]
    if (len(start_events) != 1 or len(end_events) != 1
            or start_events[0].get("simulation_epoch") != end_events[0].get("simulation_epoch")
            or int(end_events[0].get("simulation_timestamp_ns", -1))
            < int(start_events[0].get("simulation_timestamp_ns", 0))):
        raise ValueError("V0 pilot science start/end events are absent or inconsistent")
    gt_first = next(iter(analysis._read_jsonl(pilot_attempt_dir / "ground_truth.jsonl")), None)
    if (gt_first is None or gt_first.get("position_frame") != "gazebo_world_enu"
            or gt_first.get("position_axis_order") != "east,north,up"
            or gt_first.get("quaternion_order") != "xyzw"):
        raise ValueError("pilot ground truth does not satisfy the world ENU/XYZW contract")

    identity = {
        "source_run_id": pilot["source_run_id"],
        "v0_run_id": pilot["run_id"],
        "attempt_id": pilot["attempt_id"],
        "group": parent["group"],
        "repetition": parent["repetition"],
        "role": pilot.get("role"),
        "counted_in_campaign": False,
        "source_attempt_relative_path": _relative(pilot_attempt_dir, campaign_root),
        "source_files_sha256": dict(sorted(source_files.items())),
        "copy_files": sorted(copy_files),
        "model_pose_world_enu_m": pose,
        "baseline_spawn_world_enu_m": [float(base_vehicle[f"spawn_{axis}_m"])
                                        for axis in ("east", "north", "up")],
        "changed_spawn_fields": {
            "north_m": {"baseline": float(base_vehicle["spawn_north_m"]),
                        "pilot": float(new_vehicle["spawn_north_m"])},
            "up_m": {"baseline": float(base_vehicle["spawn_up_m"]),
                     "pilot": float(new_vehicle["spawn_up_m"])},
        },
        "entry_method": pilot.get("method"),
        "science_reference_sha256": new_reference_hash,
        "science_reference_matches_source_v0": True,
        "science_completed": True,
        "physical_landing_status": landing_status,
        "source_counted_in_campaign": False,
    }
    identity["source_identity_sha256"] = hashlib.sha256(
        analysis._json_text(identity).encode("utf-8")
    ).hexdigest()
    return {"pilot": pilot, "source_identity": identity,
            "source_path": pilot_attempt_dir, "copy_files": sorted(copy_files),
            "base_parameters": base_parameters, "new_parameters": new_parameters}


def _validate_camera_info(attempt_dir: Path, parameters: dict) -> None:
    summary = _read_json(attempt_dir / "image_index_summary.json")
    camera = parameters["CameraModel"]
    expected = [camera["fx_px"], 0.0, camera["cx_px"],
                0.0, camera["fy_px"], camera["cy_px"], 0.0, 0.0, 1.0]
    for side in ("left", "right"):
        variants = summary.get("camera_info", {}).get(side, {}).get("variants", [])
        if not variants:
            raise ValueError(f"indexed {side} camera has no CameraInfo records")
        for item in variants:
            if item.get("width") != camera["image_width_px"] or item.get("height") != camera["image_height_px"]:
                raise ValueError(f"indexed {side} CameraInfo dimensions differ from the pilot workbook")
            if len(item.get("k", [])) != 9 or any(
                    not math.isclose(float(actual), float(wanted), rel_tol=1e-7, abs_tol=1e-6)
                    for actual, wanted in zip(item["k"], expected)):
                raise ValueError(f"indexed {side} CameraInfo K differs from the pilot workbook")
            if any(abs(float(value)) > 1e-12 for value in item.get("d", [])):
                raise ValueError(f"indexed {side} CameraInfo distortion differs from the pilot workbook")


def _copy_context(parent: dict, output_root: Path) -> None:
    source_root = parent["parent_analysis_root"]
    _copy_verified(source_root / "analysis_freeze.json", output_root / "analysis_freeze.json")
    _copy_verified(source_root / "path_selection.json", output_root / "path_selection.json")
    context = output_root / "parent_context"
    for name in ("analysis_run_manifest.json", "analysis_summary.json",
                 "analysis_methodology.md", "calibration_freeze_manifest.json"):
        source = source_root / name
        if source.is_file():
            _copy_verified(source, context / name)


def _copy_calibration_case(parent: dict, batch: dict, output_root: Path,
                           sensor_hash_by_trial: dict[int, str]) -> dict:
    source_root = parent["parent_analysis_root"]
    source_case = analysis._case_path(source_root, batch)
    checksums_path = source_case / "calibration_checksums.json"
    source_checksums_hash = _sha256(checksums_path)
    relative_checksums = (Path(batch["case_directory"]) / "calibration_checksums.json").as_posix()
    frozen_checksums_hash = parent["calibration_freeze_manifest"].get(relative_checksums)
    if frozen_checksums_hash != source_checksums_hash:
        raise ValueError(f"source calibration checksum manifest differs from the parent freeze: {relative_checksums}")
    checksums = _read_json(checksums_path)
    source_result = source_case / "calibration_results.json"
    source_sensor = source_case / "sensor_observations.json"
    result_hash = _sha256(source_result)
    sensor_hash = _sha256(source_sensor)
    if result_hash != batch.get("calibration_results_sha256"):
        raise ValueError(f"frozen calibration result hash differs: {relative_checksums}")
    if checksums.get("calibration_results.json") != result_hash:
        raise ValueError(f"calibration result is not covered by its parent case checksum: {relative_checksums}")
    if checksums.get("sensor_observations.json") != sensor_hash:
        raise ValueError(f"camera sensor observation is not covered by its parent case checksum: {relative_checksums}")
    trial = int(batch["trial_index"])
    previous_sensor_hash = sensor_hash_by_trial.setdefault(trial, sensor_hash)
    if previous_sensor_hash != sensor_hash:
        raise ValueError(f"shared camera sensor draw differs across calibration paths for MC{trial:03d}")

    relative_case = batch["case_directory"]
    target_case = _safe_join(output_root, relative_case)
    result_copy_hash = _copy_verified(source_result, target_case / "calibration_results.json")
    sensor_copy_hash = _copy_verified(source_sensor, target_case / "sensor_observations.json")
    _write_immutable(target_case / "calibration_checksums.json", {
        "calibration_results.json": result_copy_hash,
        "sensor_observations.json": sensor_copy_hash,
    })
    return {
        "case_directory": relative_case,
        "run_id": batch["run_id"], "trial_index": trial,
        "yaw_case": batch["yaw_case"], "data_scope": batch["data_scope"],
        "parent_calibration_checksums_sha256": source_checksums_hash,
        "parent_calibration_results_sha256": result_hash,
        "copied_calibration_results_sha256": result_copy_hash,
        "parent_sensor_observations_sha256": sensor_hash,
        "copied_sensor_observations_sha256": sensor_copy_hash,
    }


def _copy_evaluation_trial_configs(parent: dict, output_root: Path,
                                   batches: list[dict]) -> tuple[int, int]:
    """Copy each frozen trial config required by the post-freeze evaluator.

    The V0 evaluator reads trial_config.json for the sensor-only baseline and
    GPS_YAW90 availability. These small inputs are part of each frozen parent
    calibration case, but are kept separate from the calibration result bundle
    so this workflow never recalibrates or changes the copied fit results.
    """
    source_root = parent["parent_analysis_root"]
    freeze_manifest = parent["calibration_freeze_manifest"]
    rows = []
    copied_bytes = 0
    for batch in batches:
        relative_case = batch["case_directory"]
        source_case = analysis._case_path(source_root, batch)
        source_config = source_case / "trial_config.json"
        source_checksums_path = source_case / "calibration_checksums.json"
        source_checksums_hash = _sha256(source_checksums_path)
        relative_checksums = (Path(relative_case) / "calibration_checksums.json").as_posix()
        if freeze_manifest.get(relative_checksums) != source_checksums_hash:
            raise ValueError(f"source calibration checksum manifest differs from the parent freeze: {relative_checksums}")
        source_checksums = _read_json(source_checksums_path)
        config_hash = _sha256(source_config)
        if source_checksums.get("trial_config.json") != config_hash:
            raise ValueError(f"trial config is not covered by its parent case checksum: {relative_case}")

        target_case = _safe_join(output_root, relative_case)
        copied_hash = _copy_verified(source_config, target_case / "trial_config.json")
        copied_bytes += (target_case / "trial_config.json").stat().st_size
        rows.append({
            "case_directory": relative_case,
            "run_id": batch["run_id"], "trial_index": int(batch["trial_index"]),
            "yaw_case": batch["yaw_case"], "data_scope": batch["data_scope"],
            "parent_calibration_checksums_sha256": source_checksums_hash,
            "parent_trial_config_sha256": config_hash,
            "copied_trial_config_sha256": copied_hash,
        })

    index_hash = _write_jsonl_immutable(output_root / "evaluation_trial_config_index.jsonl", rows)
    _write_immutable(output_root / "evaluation_input_reuse_summary.json", {
        "parent_analysis_freeze_sha256": _sha256(output_root / "analysis_freeze.json"),
        "trial_config_count": len(rows),
        "copied_trial_config_bytes": copied_bytes,
        "evaluation_trial_config_index_sha256": index_hash,
        "calibration_recomputed": False,
    })
    return len(rows), copied_bytes


def _copy_pilot_attempt(pilot_info: dict, output_root: Path) -> dict:
    source_root = pilot_info["source_path"]
    target_root = output_root / "v0_source" / "attempt"
    copied = {}
    for relative in pilot_info["copy_files"]:
        source = _safe_join(source_root, relative)
        target = _safe_join(target_root, relative)
        copied[relative] = _copy_verified(source, target)
    identity = {
        "schema_version": "stereo_v1_1_v0_source_copy_v1",
        "source_attempt_relative_path": pilot_info["source_identity"]["source_attempt_relative_path"],
        "source_identity_sha256": pilot_info["source_identity"]["source_identity_sha256"],
        "source_files_sha256": pilot_info["source_identity"]["source_files_sha256"],
        "copied_files_sha256": dict(sorted(copied.items())),
        "raw_bags_copied": True,
        "raw_source_modified": False,
    }
    _write_immutable(output_root / "v0_source_manifest.json", identity)
    return {"attempt_dir": target_root, "manifest": identity}


def _manifest_identity(payload: dict) -> dict:
    return {key: value for key, value in payload.items() if key != "created_at_utc"}


def prepare(campaign_root: Path, parent_analysis_root: Path,
            pilot_attempt_dir: Path, output_root: Path | None = None) -> dict:
    campaign_root = Path(campaign_root).expanduser().resolve(strict=True)
    parent_analysis_root = Path(parent_analysis_root).expanduser().resolve(strict=True)
    pilot_attempt_dir = Path(pilot_attempt_dir).expanduser().resolve(strict=True)
    pilot_manifest = _read_json(pilot_attempt_dir / "pilot_manifest.json")
    source_run_id = str(pilot_manifest.get("source_run_id", ""))
    parent = _validate_parent(campaign_root, parent_analysis_root, source_run_id)
    pilot_info = _condition_and_pilot(campaign_root, pilot_attempt_dir, parent)
    parent_id = parent_analysis_root.name
    v0_run_id = pilot_info["pilot"]["run_id"]
    attempt_id = pilot_info["pilot"]["attempt_id"]
    safe_run_id = re.sub(r"[^A-Za-z0-9_.-]", "_", v0_run_id)
    reeval_code_hash = _sha256(Path(__file__))
    detector_manifest = _read_json(CAMPAIGN_ROOT / "model_manifest.json")
    identity = {
        "schema_version": SCHEMA_VERSION,
        "parent_analysis_id": parent_id,
        "parent_analysis_root_relative": _relative(parent_analysis_root, campaign_root),
        "parent_analysis_freeze_sha256": parent["freeze_hash"],
        "source_campaign_protocol": parent["source_protocol"],
        "v0_source": pilot_info["source_identity"],
        "v0_run_id": v0_run_id,
        "v0_attempt_id": attempt_id,
        "base_v0_run_id": source_run_id,
        "rig_group": parent["group"],
        "repetition": parent["repetition"],
        "mc_trial_count": parent["mc_trial_count"],
        "calibration_batch_count": len(parent["batches"]),
        "calibration_recomputed": False,
        "sensitivity_calibration_recomputed": False,
        "selected_path_id": parent["freeze"]["selected_path_id"],
        "detector_model_sha256": detector_manifest["sha256"],
        "detector_ultralytics_version": detector_manifest["ultralytics_version"],
        "reevaluator_sha256": reeval_code_hash,
        "purpose": "development-only V0 position reevaluation with frozen calibration trials",
    }
    identity_hash = hashlib.sha256(analysis._json_text(identity).encode("utf-8")).hexdigest()
    identity["reevaluation_id"] = f"v0_reeval_{parent_id}_{safe_run_id}_{attempt_id}_{identity_hash[:12]}"
    output_base = (Path(output_root).expanduser().resolve() if output_root is not None
                   else parent_analysis_root.parent / "v0_reevaluations")
    output_base.mkdir(parents=True, exist_ok=True)
    if not output_base.is_relative_to(campaign_root):
        raise ValueError("reevaluation output root must stay inside the campaign recordings root")
    output_dir = (output_base / identity["reevaluation_id"]).resolve()
    if output_dir == parent_analysis_root or output_dir.is_relative_to(parent_analysis_root):
        raise ValueError("V0 reevaluation output must be outside the immutable parent analysis directory")
    identity["campaign_root_relative_to_output"] = os.path.relpath(campaign_root, output_dir)

    calibration_copy_bytes = sum(
        (analysis._case_path(parent_analysis_root, batch) / name).stat().st_size
        for batch in parent["batches"]
        for name in ("calibration_results.json", "sensor_observations.json")
    )
    pilot_copy_bytes = sum(
        _safe_join(pilot_attempt_dir, relative).stat().st_size
        for relative in pilot_info["copy_files"]
    )
    required_copy_bytes = calibration_copy_bytes + pilot_copy_bytes
    if shutil.disk_usage(output_base).free < int(required_copy_bytes * 1.05):
        raise OSError(f"insufficient free space for verified V0 reuse copy ({required_copy_bytes} bytes required)")

    with analysis.analysis_output_lock(output_dir):
        manifest_path = output_dir / "reevaluation_manifest.json"
        if manifest_path.is_file():
            existing = _read_json(manifest_path)
            if _manifest_identity(existing) != identity:
                raise FileExistsError("reevaluation folder already belongs to a different source identity")
            manifest = existing
        else:
            manifest = {**identity, "created_at_utc": datetime.now(timezone.utc).isoformat()}
            _write_immutable(manifest_path, manifest)
        _append_event(output_dir / "reevaluation_events.jsonl", "PREPARING",
                      reevaluation_id=identity["reevaluation_id"])

        _copy_context(parent, output_dir)
        sensor_hash_by_trial: dict[int, str] = {}
        reuse_rows = []
        batch_rows = []
        for batch in parent["batches"]:
            reuse_rows.append(_copy_calibration_case(parent, batch, output_dir, sensor_hash_by_trial))
            batch_rows.append({key: batch[key] for key in (
                "run_id", "trial_index", "yaw_case", "data_scope",
                "calibration_results_sha256", "case_directory",
            )})
        trial_config_count, trial_config_bytes = _copy_evaluation_trial_configs(
            parent, output_dir, parent["batches"],
        )
        _write_jsonl_immutable(output_dir / "calibration_reuse_index.jsonl", reuse_rows)
        _write_immutable(output_dir / "calibration_batches.json", {
            "schema_version": "stereo_v1_1_v0_reused_calibration_batches_v1",
            "batch_count": len(batch_rows), "batches": batch_rows,
        })
        _write_immutable(output_dir / "calibration_reuse_summary.json", {
            "parent_analysis_id": parent_id,
            "parent_analysis_freeze_sha256": parent["freeze_hash"],
            "matching_group": parent["group"], "matching_repetition": parent["repetition"],
            "mc_trial_count": parent["mc_trial_count"],
            "calibration_batch_count": len(batch_rows),
            "unique_mc_trials": len(sensor_hash_by_trial),
            "calibration_solver_invocations": 0,
            "sensitivity_solver_invocations": 0,
            "copied_calibration_result_and_sensor_bytes": calibration_copy_bytes,
            "copied_v0_source_bytes": pilot_copy_bytes,
            "selected_path_id": identity["selected_path_id"],
        })
        copied_pilot = _copy_pilot_attempt(pilot_info, output_dir)
        attempt_dir = copied_pilot["attempt_dir"]
        if not (attempt_dir / "image_index.jsonl").is_file():
            from campaigns.stereo_v1_1.bag_index import index_attempt
            capture = _read_json(attempt_dir / "raw_capture_summary.json")
            tolerance = float(pilot_info["new_parameters"]["Capture"]["pair_tolerance_s"])
            index_summary = index_attempt(
                attempt_dir, v0_run_id, attempt_id,
                pair_tolerance_ns=round(tolerance * 1_000_000_000),
            )
            _write_immutable(output_dir / "v0_index_summary.json", index_summary)
        else:
            index_summary = _read_json(output_dir / "v0_index_summary.json")
        _validate_camera_info(attempt_dir, pilot_info["new_parameters"])
        _append_event(output_dir / "reevaluation_events.jsonl", "PREPARED",
                      calibration_batch_count=len(batch_rows),
                      v0_pair_count=index_summary["pair_count"],
                      evaluation_trial_config_count=trial_config_count,
                      evaluation_trial_config_bytes=trial_config_bytes,
                      yolo_postprocessing_required=True)
    return {
        "status": "PREPARED_FOR_YOLO_AND_V0_EVALUATION",
        "reevaluation_id": identity["reevaluation_id"],
        "output_relative_to_campaign": _relative(output_dir, campaign_root),
        "calibration_batch_count": len(parent["batches"]),
        "mc_trial_count": parent["mc_trial_count"],
        "evaluation_trial_config_count": trial_config_count,
        "calibration_recomputed": False,
        "yolo_postprocessing_required": True,
    }


def _verify_yolo_outputs(attempt_dir: Path, pilot_parameters: dict,
                         detector_manifest: dict) -> dict:
    output = attempt_dir / "yolo-8.4.127"
    summary_path = output / "observation_summary.json"
    if not summary_path.is_file():
        raise FileNotFoundError("pinned YOLO outputs are missing; run the documented postprocessor first")
    summary = _read_json(summary_path)
    pilot_manifest = _read_json(attempt_dir / "pilot_manifest.json")
    capture = pilot_parameters["Capture"]
    expected = {
        "status": "COMPLETE",
        "run_id": pilot_manifest["run_id"],
        "attempt_id": pilot_manifest["attempt_id"],
        "model_sha256": detector_manifest["sha256"],
        "ultralytics_version": detector_manifest["ultralytics_version"],
        "confidence": float(capture["yolo_confidence"]),
        "nms_iou": float(capture["yolo_nms_iou"]),
        "inference_imgsz_px": int(capture["yolo_imgsz_px"]),
        "requested_target_hz": float(capture["yolo_target_hz"]),
    }
    differences = {key: (summary.get(key), value) for key, value in expected.items()
                   if summary.get(key) != value}
    if differences:
        raise ValueError(f"YOLO processing identity differs from the V0 pilot contract: {differences}")
    for filename in ("observations_left.jsonl", "observations_right.jsonl", "paired_observations.jsonl"):
        if not (output / filename).is_file():
            raise FileNotFoundError(f"YOLO output is incomplete: {filename}")
    if summary.get("image_index_sha256") != _sha256(attempt_dir / "image_index.jsonl"):
        raise ValueError("YOLO output was processed from a different image index")
    if summary.get("pair_index_sha256") != _sha256(attempt_dir / "pair_index.jsonl"):
        raise ValueError("YOLO output was processed from a different stereo pair index")
    return summary


def _prepared_slots(parent: dict, v0_manifest: dict, evaluation_attempt_relative: str) -> tuple[list[dict], dict]:
    calibration_slots = parent["calibration_slots"]
    v0_slot = {
        "run_id": v0_manifest["v0_run_id"], "path_id": "V0",
        "rig_group": v0_manifest["rig_group"], "repetition": v0_manifest["repetition"],
        "status": "DEVELOPMENT", "attempts": [{
            "attempt_id": v0_manifest["v0_attempt_id"],
            "recording_path": evaluation_attempt_relative,
        }],
    }
    return [*calibration_slots, v0_slot], v0_slot


def _prepare_evaluation_attempt(output_root: Path, source_attempt: Path) -> Path:
    target = output_root / "v0_source" / "evaluation_attempt"
    required = (
        "image_index.jsonl", "ground_truth.jsonl", "camera_truth.json",
        "events.jsonl", "controller_result.json",
    )
    yolo = source_attempt / "yolo-8.4.127"
    for name in (*required, "pair_index.jsonl"):
        if not (source_attempt / name).is_file():
            raise FileNotFoundError(f"V0 source is missing required indexed evidence: {name}")
    for name in ("observations_left.jsonl", "observations_right.jsonl", "paired_observations.jsonl"):
        if not (yolo / name).is_file():
            raise FileNotFoundError(f"V0 YOLO output is missing: {name}")
    target.mkdir(parents=True, exist_ok=True)
    sources = {name: source_attempt / name for name in required}
    sources.update({name: yolo / name for name in (
        "observations_left.jsonl", "observations_right.jsonl", "paired_observations.jsonl")})
    for name, source in sources.items():
        analysis._copy_verified(source, target / name, owned_root=output_root)
    return target


def _verify_pilot_camera_geometry(parent: dict, v0_attempt: Path) -> tuple[dict, list[dict]]:
    camera_truth, sources = camera_sensor_generator._source_camera_truth(
        parent["campaign_root"], parent["calibration_slots"],
    )
    pilot_camera = _read_json(v0_attempt / "camera_truth.json").get("camera_payloads", {})
    if set(pilot_camera) != {"left", "right"}:
        raise ValueError("new V0 camera truth must contain left and right optical cameras")
    for side in ("left", "right"):
        candidate = pilot_camera[side]
        expected = camera_truth[side]
        position = candidate.get("observed_optical_position_world_enu_m")
        orientation = candidate.get("observed_optical_orientation_world_from_entity_xyzw")
        if position is None or orientation is None:
            raise ValueError(f"new V0 camera truth is incomplete for {side}")
        if math.dist(position, expected["observed_optical_position_world_enu_m"]) > 1e-6:
            raise ValueError(f"new V0 changed the {side} camera optical center")
        if not camera_sensor_generator._q_equivalent(
                orientation, expected["observed_optical_orientation_world_from_entity_xyzw"]):
            raise ValueError(f"new V0 changed the {side} camera optical orientation")
        if (float(candidate.get("desired_to_observed_optical_delta_norm_m", math.inf)) > 1e-6
                or float(candidate.get("desired_to_observed_optical_orientation_delta_deg", math.inf)) > 1e-5):
            raise ValueError(f"new V0 camera truth fails the optical pose gate for {side}")
    return camera_truth, sources


def _generate_v0_sensors(parent: dict, output_root: Path, v0_slot: dict,
                         sensor_hash_by_trial: dict[int, str]) -> dict:
    truth, truth_sources = _verify_pilot_camera_geometry(
        parent, output_root / "v0_source" / "evaluation_attempt",
    )
    sensor_spec = json.loads(camera_sensor_generator.SPEC.read_text(encoding="utf-8-sig"))
    camera_root = output_root / "generated_inputs" / "camera"
    generated_root = {"camera": camera_root}
    for trial in range(parent["mc_trial_count"]):
        observation = camera_sensor_generator._observation(
            parent["group"], parent["repetition"], trial, truth, sensor_spec,
        )
        expected_hash = sensor_hash_by_trial.get(trial)
        observation_text = json.dumps(observation, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        observation_hash = hashlib.sha256(observation_text.encode("utf-8")).hexdigest()
        if expected_hash is None or observation_hash != expected_hash:
            raise ValueError(f"new V0 camera observation differs from frozen MC{trial:03d} calibration draw")
        target = analysis._camera_sensor_path(output_root, v0_slot, trial, camera_root)
        _write_immutable(target, observation)
        provenance = {
            "schema_version": "stereo_v1_1_v0_reevaluation_sensor_provenance_v1",
            "purpose": "reuse the exact calibration camera RTK/IMU draw for the matching new V0",
            "shared_sensor_key": observation["shared_sensor_key"],
            "monte_carlo_trial_index": trial,
            "offline_truth_sources": truth_sources,
            "new_v0_camera_truth_sha256": _sha256(
                output_root / "v0_source" / "evaluation_attempt" / "camera_truth.json"
            ),
            "observation_sha256": observation_hash,
            "camera_position_frame": "gazebo_world_enu:east,north,up_m",
            "camera_orientation_contract": "optical R/P parameters; camera optical rays RDF",
            "estimator_input": False,
        }
        _write_immutable(target.parent / "sensor_generation_provenance.json", provenance)
    return generated_root


class _MetricGroup:
    def __init__(self):
        self.count = 0
        self.calibration_ids = set()
        self.evaluation_ids = set()
        self.trials = set()
        self.means = {name: analysis._Mean() for name in ("rmse", "p95", "detect", "valid")}


def _summarize_results(payloads) -> dict:
    grouped = {}
    convergence: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    result_count = main_count = 0
    for payload in payloads:
        for row in payload.get("results", []):
            result_count += 1
            key = f"{row.get('method')}__{row.get('attitude_variant')}"
            convergence[key][str(row.get("calibration_status"))] += 1
            if row.get("attitude_variant") not in analysis.VARIANTS or row.get("method") not in analysis.METHODS:
                continue
            main_count += 1
            group_key = (row.get("data_scope"), row.get("method"), row.get("attitude_variant"))
            group = grouped.setdefault(group_key, _MetricGroup())
            group.count += 1
            if row.get("calibration_run_id") is not None:
                group.calibration_ids.add(row["calibration_run_id"])
            if row.get("evaluation_run_id") is not None:
                group.evaluation_ids.add(row["evaluation_run_id"])
            if row.get("mc_seed") is not None:
                group.trials.add(row["mc_seed"])
            metrics = row.get("metrics") or {}
            for source, destination in (("position_3d_rmse_m", "rmse"),
                                        ("position_3d_p95_m", "p95"),
                                        ("detection_rate", "detect"),
                                        ("valid_measurement_rate", "valid")):
                group.means[destination].add(metrics.get(source))
    summary_rows = []
    for (scope, method, variant), group in sorted(grouped.items()):
        summary_rows.append({
            "data_scope": scope, "method": method, "attitude_variant": variant,
            "evaluation_case_count": group.count,
            "source_calibration_run_count": len(group.calibration_ids),
            "source_evaluation_run_count": len(group.evaluation_ids),
            "offline_mc_trial_count": len(group.trials),
            "metric_available_case_count": group.means["rmse"].count,
            "position_3d_rmse_mean_m": group.means["rmse"].value(),
            "position_3d_p95_mean_m": group.means["p95"].value(),
            "detection_rate_mean": group.means["detect"].value(),
            "valid_measurement_rate_mean": group.means["valid"].value(),
        })
    return {
        "schema_version": "stereo_v1_1_v0_reevaluation_summary_v1",
        "evaluation_result_count": result_count,
        "main_comparison_result_count": main_count,
        "main_comparison_summary": summary_rows,
        "calibration_convergence_counts": {key: dict(value) for key, value in convergence.items()},
        "aggregation_scope": "means over available per-case metrics; MC cases and physical V0 attempts are distinct",
        "position_metrics_status": "evaluated after the parent analysis freeze",
    }


def _evaluation_payloads(output_root: Path, batches: list[dict]):
    freeze_hash = _sha256(output_root / "analysis_freeze.json")
    for batch in batches:
        case = analysis._case_path(output_root, batch)
        path = case / "evaluation_results.json"
        if not path.is_file() or _sha256(path) != batch.get("evaluation_results_sha256"):
            raise ValueError(f"V0 evaluation result hash differs: {batch['case_directory']}")
        payload = _read_json(path)
        if payload.get("freeze_sha256") != freeze_hash:
            raise ValueError("V0 evaluation result is not tied to the copied parent freeze")
        yield payload


def _evaluate_batches(output_root: Path, slots: list[dict], batches: list[dict],
                      spec: dict, generated_roots: dict[str, Path]) -> tuple[int, float]:
    """Run only the frozen post-freeze evaluator for already copied batches."""
    eval_cache: dict = {}
    result_count = 0
    started = time.monotonic()
    _append_event(output_root / "reevaluation_events.jsonl", "EVALUATION_STARTED",
                  v0_run_id=next(slot["run_id"] for slot in slots if slot.get("path_id") == "V0"),
                  batch_count=len(batches))
    for index, batch in enumerate(batches, start=1):
        payload = analysis._evaluate_calibration_batch(
            output_root, slots, batch, eval_cache, spec, output_root, generated_roots,
        )
        batch["evaluation_results_sha256"] = _sha256(
            analysis._case_path(output_root, batch) / "evaluation_results.json"
        )
        result_count += len(payload.get("results", []))
        if index % 500 == 0 or index == len(batches):
            _append_event(output_root / "reevaluation_events.jsonl", "EVALUATION_PROGRESS",
                          completed_batches=index, total_batches=len(batches),
                          result_rows=result_count)
    eval_cache.clear()
    return result_count, time.monotonic() - started


def _write_final_artifacts(output_root: Path, manifest: dict,
                           batches: list[dict], metric_payloads,
                           evaluation_runner_sha256: str) -> dict:
    summary = _summarize_results(metric_payloads())
    summary.update({
        "reevaluation_id": manifest["reevaluation_id"],
        "development_only": True,
        "counted_in_campaign": False,
        "v0_run_id": manifest["v0_run_id"],
        "v0_attempt_id": manifest["v0_attempt_id"],
        "base_v0_run_id": manifest["base_v0_run_id"],
        "rig_group": manifest["rig_group"],
        "repetition": manifest["repetition"],
        "mc_trial_count": manifest["mc_trial_count"],
        "calibration_batch_count": len(batches),
        "calibration_recomputed": False,
        "sensitivity_calibration_recomputed": False,
        "parent_analysis_id": manifest["parent_analysis_id"],
        "parent_analysis_freeze_sha256": manifest["parent_analysis_freeze_sha256"],
        "prepare_runner_sha256": manifest["reevaluator_sha256"],
        "evaluation_runner_sha256": evaluation_runner_sha256,
        "selected_path_id": manifest["selected_path_id"],
        "yolo_device_requested": manifest.get("yolo_device_requested"),
        "v0_position_errors_evaluated_after_parent_freeze": True,
        "changed_spawn_fields": manifest["v0_source"]["changed_spawn_fields"],
        "entry_method": manifest["v0_source"]["entry_method"],
        "science_reference_matches_source_v0": manifest["v0_source"]["science_reference_matches_source_v0"],
        "limitations": [
            "This is a development engineering pilot, not one of the frozen 39 campaign slots.",
            "The pilot changes spawn north and spawn height and uses a vertical-liftoff gate.",
            "Physical landing was not verified by the pilot record.",
            "CPU YOLO inference was used because CUDA was unavailable in the execution environment.",
        ],
    })
    _write_immutable(output_root / "reevaluation_summary.json", summary)
    _write_immutable(output_root / "calibration_run_index.json", {
        "schema_version": "stereo_v1_1_v0_reevaluation_run_index_v1",
        "batch_count": len(batches),
        "storage_contract": "matching frozen calibration bundles are copied into this output; V0 evaluations are stored per case",
        "batches": batches,
    })
    fields = ["calibration_run_id", "evaluation_run_id", "trial_index", "yaw_case",
              "data_scope", "method", "attitude_variant", "evaluation_status",
              "detection_rate", "valid_measurement_rate", "lateral_east_rmse_m",
              "forward_north_rmse_m", "vertical_up_rmse_m", "position_3d_rmse_m",
              "position_3d_p95_m"]
    _write_csv_digest = analysis._write_csv_immutable(
        output_root / "v0_evaluation_metrics.csv", fields,
        analysis._iter_metric_rows(metric_payloads(), "evaluation"),
    )
    _write_immutable(output_root / "reevaluation_checksums.json", {
        "reevaluation_manifest.json": _sha256(output_root / "reevaluation_manifest.json"),
        "analysis_freeze.json": _sha256(output_root / "analysis_freeze.json"),
        "path_selection.json": _sha256(output_root / "path_selection.json"),
        "calibration_reuse_index.jsonl": _sha256(output_root / "calibration_reuse_index.jsonl"),
        "calibration_batches.json": _sha256(output_root / "calibration_batches.json"),
        "evaluation_trial_config_index.jsonl": _sha256(output_root / "evaluation_trial_config_index.jsonl"),
        "evaluation_input_reuse_summary.json": _sha256(output_root / "evaluation_input_reuse_summary.json"),
        "v0_source_manifest.json": _sha256(output_root / "v0_source_manifest.json"),
        "reevaluation_summary.json": _sha256(output_root / "reevaluation_summary.json"),
        "v0_evaluation_metrics.csv": _write_csv_digest,
        "calibration_run_index.json": _sha256(output_root / "calibration_run_index.json"),
    })
    return summary


def evaluate(reevaluation_root: Path) -> dict:
    output_root = Path(reevaluation_root).expanduser().resolve(strict=True)
    manifest_path = output_root / "reevaluation_manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError("reevaluation manifest is missing; run the prepare step first")
    manifest = _read_json(manifest_path)
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported V0 reevaluation manifest")
    evaluation_runner_sha256 = _sha256(Path(__file__))
    existing_status_path = output_root / "reevaluation_status.json"
    existing_status = _read_json(existing_status_path) if existing_status_path.is_file() else None
    if (existing_status is not None
            and existing_status.get("evaluation_runner_sha256") != evaluation_runner_sha256):
        raise ValueError("completed V0 results were produced by a different reevaluation code version")
    if manifest.get("reevaluator_sha256") != evaluation_runner_sha256:
        existing_cases = list((output_root / "derived").rglob("evaluation_results.json"))
        if existing_cases:
            raise ValueError("evaluation code changed after partial results were written; prepare a fresh reevaluation folder")
    campaign_root = (output_root / manifest["campaign_root_relative_to_output"]).resolve(strict=True)
    parent_analysis_root = (campaign_root / manifest["parent_analysis_root_relative"]).resolve(strict=True)
    parent = _validate_parent(campaign_root, parent_analysis_root, manifest["base_v0_run_id"])
    if parent["freeze_hash"] != manifest["parent_analysis_freeze_sha256"]:
        raise ValueError("parent freeze changed after this V0 reevaluation folder was prepared")
    if parent["mc_trial_count"] != manifest["mc_trial_count"]:
        raise ValueError("parent MC trial count changed after preparation")
    attempt_dir = output_root / "v0_source" / "attempt"
    yolo_summary = _verify_yolo_outputs(attempt_dir, _read_json(attempt_dir / "workbook_parameters.json"),
                                        _read_json(CAMPAIGN_ROOT / "model_manifest.json"))
    source_manifest = _read_json(output_root / "v0_source_manifest.json")
    if source_manifest.get("source_identity_sha256") != manifest["v0_source"]["source_identity_sha256"]:
        raise ValueError("copied V0 source manifest differs from the reevaluation identity")
    for relative, expected_hash in source_manifest.get("copied_files_sha256", {}).items():
        copied_source = _safe_join(attempt_dir, relative)
        if not copied_source.is_file() or _sha256(copied_source) != expected_hash:
            raise ValueError(f"copied V0 source input changed after preparation: {relative}")

    with analysis.analysis_output_lock(output_root):
        attempt_view = _prepare_evaluation_attempt(output_root, attempt_dir)
        new_slot = {
            "run_id": manifest["v0_run_id"], "path_id": "V0",
            "rig_group": manifest["rig_group"], "repetition": manifest["repetition"],
            "status": "DEVELOPMENT", "attempts": [{
                "attempt_id": manifest["v0_attempt_id"],
                "recording_path": _relative(attempt_view, output_root),
            }],
        }
        eval_slots = [*parent["calibration_slots"], new_slot]
        sensor_hashes = {}
        for row in _read_jsonl(output_root / "calibration_reuse_index.jsonl"):
            trial = int(row["trial_index"])
            previous = sensor_hashes.setdefault(trial, row["copied_sensor_observations_sha256"])
            if previous != row["copied_sensor_observations_sha256"]:
                raise ValueError(f"copied camera sensor draw differs across calibration batches at MC{trial:03d}")
        generated_roots = _generate_v0_sensors(parent, output_root, new_slot, sensor_hashes)
        batches = _read_json(output_root / "calibration_batches.json")["batches"]
        trial_config_count, trial_config_bytes = _copy_evaluation_trial_configs(
            parent, output_root, batches,
        )
        _append_event(output_root / "reevaluation_events.jsonl", "EVALUATION_INPUTS_READY",
                      trial_config_count=trial_config_count,
                      trial_config_bytes=trial_config_bytes)
        result_count, elapsed = _evaluate_batches(
            output_root, eval_slots, batches, parent["protocol"], generated_roots,
        )
        def payloads():
            yield from _evaluation_payloads(output_root, batches)
        summary = _write_final_artifacts(
            output_root, manifest, batches, payloads, evaluation_runner_sha256,
        )
        _append_event(output_root / "reevaluation_events.jsonl", "EVALUATION_COMPLETE",
                      elapsed_seconds=round(elapsed, 3), result_rows=result_count)
        status = {
            "status": "DEVELOPMENT_V0_REEVALUATION_COMPLETE",
            "reevaluation_id": manifest["reevaluation_id"],
            "v0_run_id": manifest["v0_run_id"], "v0_attempt_id": manifest["v0_attempt_id"],
            "development_only": True, "counted_in_campaign": False,
            "mc_trial_count": manifest["mc_trial_count"],
            "calibration_batch_count": len(batches), "evaluation_result_count": result_count,
            "calibration_recomputed": False, "sensitivity_calibration_recomputed": False,
            "elapsed_seconds": round(elapsed, 3),
            "parent_analysis_freeze_sha256": manifest["parent_analysis_freeze_sha256"],
            "prepare_runner_sha256": manifest["reevaluator_sha256"],
            "evaluation_runner_sha256": evaluation_runner_sha256,
            "summary_sha256": _sha256(output_root / "reevaluation_summary.json"),
            "summary_rows": len(summary["main_comparison_summary"]),
        "yolo_device_requested": yolo_summary.get("device_requested"),
        }
        status_path = output_root / "reevaluation_status.json"
        if status_path.is_file():
            previous_status = _read_json(status_path)
            stable_fields = set(status) - {"elapsed_seconds"}
            if any(previous_status.get(key) != status[key] for key in stable_fields):
                raise FileExistsError("completed reevaluation status differs from current verified outputs")
            status = previous_status
        else:
            _write_immutable(status_path, status)
    return status


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    prepare_parser = subparsers.add_parser("prepare", help="copy matching frozen calibration outputs and the V0 source")
    prepare_parser.add_argument("--campaign-root", type=Path, required=True)
    prepare_parser.add_argument("--parent-analysis-root", type=Path, required=True)
    prepare_parser.add_argument("--pilot-attempt-dir", type=Path, required=True)
    prepare_parser.add_argument("--output-root", type=Path, default=None,
                                help="defaults to <campaign>/analysis_versions/v0_reevaluations")
    evaluate_parser = subparsers.add_parser("evaluate", help="evaluate prepared YOLO observations with the frozen calibration MC")
    evaluate_parser.add_argument("--reevaluation-root", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = (prepare(args.campaign_root, args.parent_analysis_root,
                          args.pilot_attempt_dir, output_root=args.output_root)
                  if args.command == "prepare" else evaluate(args.reevaluation_root))
    except (OSError, ValueError, RuntimeError, KeyError, FileExistsError, ImportError) as exc:
        print(json.dumps({"status": "V0_REEVALUATION_FAILED", "error": str(exc)},
                         ensure_ascii=False), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
