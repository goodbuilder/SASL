#!/usr/bin/env python3
"""Validate calibration-transfer provenance produced by an attitude sweep."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import sys
from pathlib import Path
from typing import Any


EXPECTED_RECORD_SCHEMA = "s2_atomic_pair_v11"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

ALWAYS_REQUIRED_COLUMNS = {
    "schema_version",
    "session_id",
    "run_id",
    "attempt_id",
    "case_id",
    "effective_config_sha256",
    "record_sequence",
    "record_type",
    "sample_id",
    "epoch",
    "pair_id",
    "simulation_timestamp_ns",
    "left_observation_timestamp_ns",
    "right_observation_timestamp_ns",
    "left_right_dt_s",
    "left_observation_source",
    "right_observation_source",
    "left_observation_valid",
    "right_observation_valid",
    "operational_position_frame",
    "left_operational_position_valid",
    "right_operational_position_valid",
    "left_operational_pose_valid",
    "right_operational_pose_valid",
    "operational_ray_generation_valid",
    "triangulation_valid",
    "triangulation_status",
    "sample_valid",
    "failure_stage",
    "failure_code",
    "failure_reason",
    "gt_timestamp_ns",
    "gt_valid",
    "gt_source",
    "gt_model_name",
    "gt_link_name",
    "gt_world_enu_x_m",
    "gt_world_enu_y_m",
    "gt_world_enu_z_m",
    "gt_forward_m",
    "gt_right_m",
    "gt_down_m",
    "left_injected_roll_deg",
    "left_injected_pitch_deg",
    "left_injected_yaw_deg",
    "right_injected_roll_deg",
    "right_injected_pitch_deg",
    "right_injected_yaw_deg",
    "sample_eligible",
    "terminal_status",
    "terminal_reason",
    "measurement_status",
    "evaluation_status",
    "observation_mode",
    "output_frame",
    "output_axes",
    "position_unit",
    "pixel_unit",
    "angle_unit",
    "timestamp_unit",
}

OBSERVATION_COLUMNS = {
    "left_u_px",
    "left_v_px",
    "right_u_px",
    "right_v_px",
    "left_confidence",
    "right_confidence",
}

BBOX_COLUMNS = {
    "left_bbox_xmin_px",
    "left_bbox_ymin_px",
    "left_bbox_xmax_px",
    "left_bbox_ymax_px",
    "right_bbox_xmin_px",
    "right_bbox_ymin_px",
    "right_bbox_xmax_px",
    "right_bbox_ymax_px",
}

OPERATIONAL_SOURCE_COLUMNS = {
    "left_operational_position_baseline_center_frd_x_m",
    "left_operational_position_baseline_center_frd_y_m",
    "left_operational_position_baseline_center_frd_z_m",
    "right_operational_position_baseline_center_frd_x_m",
    "right_operational_position_baseline_center_frd_y_m",
    "right_operational_position_baseline_center_frd_z_m",
    "operational_position_source",
    "operational_source_snapshot_timestamp_ns",
    "left_operational_position_timestamp_ns",
    "right_operational_position_timestamp_ns",
    "left_position_alignment_dt_s",
    "right_position_alignment_dt_s",
    "left_operational_attitude_qw",
    "left_operational_attitude_qx",
    "left_operational_attitude_qy",
    "left_operational_attitude_qz",
    "right_operational_attitude_qw",
    "right_operational_attitude_qx",
    "right_operational_attitude_qy",
    "right_operational_attitude_qz",
    "operational_attitude_source",
    "operational_attitude_parent_frame",
    "operational_attitude_child_frame",
    "operational_attitude_quaternion_order",
    "left_operational_pose_timestamp_ns",
    "right_operational_pose_timestamp_ns",
    "left_operational_attitude_timestamp_ns",
    "right_operational_attitude_timestamp_ns",
    "left_pose_alignment_dt_s",
    "right_pose_alignment_dt_s",
    "sync_valid",
}

OPERATIONAL_RESULT_COLUMNS = {
    "operational_x_m",
    "operational_y_m",
    "operational_z_m",
    "operational_range_m",
    "operational_azimuth_deg",
    "operational_elevation_deg",
    "operational_ray_valid",
    "operational_ray_left_origin_x_m",
    "operational_ray_left_origin_y_m",
    "operational_ray_left_origin_z_m",
    "operational_ray_left_direction_x",
    "operational_ray_left_direction_y",
    "operational_ray_left_direction_z",
    "operational_ray_right_origin_x_m",
    "operational_ray_right_origin_y_m",
    "operational_ray_right_origin_z_m",
    "operational_ray_right_direction_x",
    "operational_ray_right_direction_y",
    "operational_ray_right_direction_z",
    "operational_ray_gap_m",
    "operational_ray_intersection_angle_deg",
    "operational_ray_left_depth_m",
    "operational_ray_right_depth_m",
    "operational_ray_condition_number",
    "operational_ray_geometry_rank",
    "operational_ray_left_closest_x_m",
    "operational_ray_left_closest_y_m",
    "operational_ray_left_closest_z_m",
    "operational_ray_right_closest_x_m",
    "operational_ray_right_closest_y_m",
    "operational_ray_right_closest_z_m",
    "near_parallel",
    "negative_left_depth",
    "negative_right_depth",
    "nonfinite_input",
    "nonfinite_output",
    "zero_or_small_baseline",
    "condition_exceeded",
}


class Validation:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []

    def require(self, condition: bool, message: str) -> None:
        if not condition:
            self.errors.append(message)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_sha256(payload: dict[str, Any]) -> str:
    canonical = json.dumps(
        payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _load_json(path: Path, validation: Validation) -> dict[str, Any]:
    if not path.is_file():
        validation.errors.append(f"missing file: {path}")
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        validation.errors.append(f"invalid JSON {path}: {exc}")
        return {}
    if not isinstance(value, dict):
        validation.errors.append(f"JSON root must be an object: {path}")
        return {}
    return value


def _bool(value: str) -> bool | None:
    normalized = str(value).strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    return None


def _finite(value: str) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _nonempty(row: dict[str, str], fields: set[str]) -> list[str]:
    return sorted(name for name in fields if row.get(name, "") == "")


def _unit_vector(row: dict[str, str], prefix: str) -> bool:
    try:
        norm = math.sqrt(
            sum(float(row[f"{prefix}_{axis}"]) ** 2 for axis in ("x", "y", "z"))
        )
    except (KeyError, TypeError, ValueError):
        return False
    return math.isfinite(norm) and abs(norm - 1.0) <= 1.0e-6


def _unit_quaternion(row: dict[str, str], side: str) -> bool:
    try:
        norm = math.sqrt(
            sum(
                float(row[f"{side}_operational_attitude_q{axis}"]) ** 2
                for axis in ("w", "x", "y", "z")
            )
        )
    except (KeyError, TypeError, ValueError):
        return False
    return math.isfinite(norm) and abs(norm - 1.0) <= 1.0e-6


def _validate_effective_config(
    case_dir: Path, validation: Validation
) -> tuple[dict[str, Any], str]:
    config_path = case_dir / "effective_config.json"
    config = _load_json(config_path, validation)
    stored_hash = str(config.get("effective_config_sha256", ""))
    validation.require(bool(SHA256_RE.fullmatch(stored_hash)), "invalid effective_config_sha256")
    if config:
        unhashed = dict(config)
        unhashed.pop("effective_config_sha256", None)
        validation.require(
            _canonical_sha256(unhashed) == stored_hash,
            "effective_config_sha256 does not match canonical effective config",
        )
    validation.require(config.get("hash_algorithm") == "sha256", "hash algorithm is not sha256")

    coordinate = config.get("coordinate_contract", {})
    expected_coordinate = {
        "gazebo_world": "ENU",
        "px4_local": "NED",
        "px4_body": "FRD",
        "camera_optical": "RDF",
        "output_frame": "baseline_center_frd",
        "output_axes": "forward_right_down",
        "length_unit": "m",
        "angle_unit": "deg",
        "px4_quaternion_order": "wxyz",
        "gazebo_quaternion_order": "xyzw",
    }
    for name, value in expected_coordinate.items():
        validation.require(coordinate.get(name) == value, f"coordinate contract mismatch: {name}")

    camera = config.get("camera", {})
    intrinsic_fields = (
        "image_width_px",
        "image_height_px",
        "horizontal_fov_rad",
        "fx_px",
        "fy_px",
        "cx_px",
        "cy_px",
        "skew_px",
        "distortion_model",
        "distortion_coefficients",
    )
    intrinsic = {name: camera.get(name) for name in intrinsic_fields}
    validation.require(
        _canonical_sha256(intrinsic) == camera.get("intrinsic_profile_sha256"),
        "intrinsic profile hash mismatch",
    )

    rig = config.get("rig", {})
    rig_fields = (
        "left_nominal_origin_baseline_center_frd_m",
        "right_nominal_origin_baseline_center_frd_m",
        "baseline_vector_baseline_center_frd_m",
        "baseline_length_m",
    )
    rig_contract = {name: rig.get(name) for name in rig_fields}
    validation.require(
        _canonical_sha256(rig_contract) == rig.get("profile_sha256"),
        "rig profile hash mismatch",
    )
    try:
        left = rig_contract["left_nominal_origin_baseline_center_frd_m"]
        right = rig_contract["right_nominal_origin_baseline_center_frd_m"]
        vector = [float(right[index]) - float(left[index]) for index in range(3)]
        length = math.sqrt(sum(value * value for value in vector))
        validation.require(
            all(abs(vector[index] - float(rig_contract["baseline_vector_baseline_center_frd_m"][index])) <= 1.0e-9 for index in range(3)),
            "rig baseline vector is inconsistent with optical centers",
        )
        validation.require(length > 0.0, "rig baseline is zero")
        validation.require(
            abs(length - float(rig_contract["baseline_length_m"])) <= 1.0e-9,
            "rig baseline length is inconsistent with optical centers",
        )
        validation.require(
            all(abs(float(left[index]) + float(right[index])) <= 1.0e-9 for index in range(3)),
            "baseline_center_frd origin is not the optical-center midpoint",
        )
    except (KeyError, TypeError, ValueError, IndexError):
        validation.errors.append("rig contract is incomplete or non-numeric")

    ground_truth = config.get("ground_truth", {})
    validation.require(ground_truth.get("source") == "gazebo", "effective config GT source is not Gazebo")
    validation.require(ground_truth.get("link") == "vehicle_reference_link", "effective config GT link mismatch")
    validation.require(ground_truth.get("runtime_feature_allowed") is False, "Gazebo GT is not marked label-only")
    return config, stored_hash


def validate_case(case_dir: Path) -> dict[str, Any]:
    validation = Validation()
    config, config_hash = _validate_effective_config(case_dir, validation)
    recording_dir = case_dir / "recording"
    csv_path = recording_dir / "pair_records.csv"
    jsonl_path = recording_dir / "pair_records.jsonl"
    metadata = _load_json(recording_dir / "run_metadata.json", validation)
    summary = _load_json(recording_dir / "run_summary.json", validation)
    validation.require((recording_dir / "RUN_FINALIZED_VALID").is_file(), "missing RUN_FINALIZED_VALID")
    validation.require(not (recording_dir / "RUN_FINALIZED_INVALID").exists(), "RUN_FINALIZED_INVALID exists")

    case_id = str(config.get("case_id", case_dir.name))
    session_id = str(config.get("session_id", ""))
    run_id = str(config.get("run_id", ""))
    for name, expected in (
        ("schema_version", EXPECTED_RECORD_SCHEMA),
        ("session_id", session_id),
        ("run_id", run_id),
        ("attempt_id", case_id),
        ("case_id", case_id),
        ("effective_config_sha256", config_hash),
    ):
        validation.require(metadata.get(name) == expected, f"run metadata mismatch: {name}")
    validation.require(metadata.get("mission_profile") == "takeoff_forward_land", "mission profile does not identify Auto trajectory")
    validation.require(metadata.get("trajectory_id") == "takeoff_forward_land", "trajectory_id mismatch")
    validation.require(metadata.get("ground_truth_source") == "gazebo_vehicle_reference_link_world_pose", "run metadata GT source mismatch")
    validation.require(metadata.get("ground_truth_link_name") == "vehicle_reference_link", "run metadata GT link mismatch")
    validation.require(metadata.get("ground_truth_fallback_allowed") is False, "GT fallback must be disabled")
    validation.require(
        metadata.get("operational_pose_alignment_method")
        == "latest_valid_px4_sensor_snapshot_at_image_pair_acceptance",
        "operational pose alignment method missing",
    )
    attitude_alignment_limit_s = metadata.get("px4_attitude_max_age_s")
    position_alignment_limit_s = metadata.get("gps_baseline_max_age_s")
    validation.require(
        isinstance(attitude_alignment_limit_s, (int, float))
        and math.isfinite(float(attitude_alignment_limit_s))
        and float(attitude_alignment_limit_s) > 0.0,
        "metadata PX4 attitude alignment limit is invalid",
    )
    validation.require(
        isinstance(position_alignment_limit_s, (int, float))
        and math.isfinite(float(position_alignment_limit_s))
        and float(position_alignment_limit_s) > 0.0,
        "metadata GPS position alignment limit is invalid",
    )
    attitude_alignment_limit_s = float(attitude_alignment_limit_s or 0.0)
    position_alignment_limit_s = float(position_alignment_limit_s or 0.0)
    validation.require(metadata.get("intrinsic_profile_sha256") == config.get("camera", {}).get("intrinsic_profile_sha256"), "metadata/config intrinsic hash mismatch")
    validation.require(metadata.get("rig_profile_sha256") == config.get("rig", {}).get("profile_sha256"), "metadata/config rig hash mismatch")

    validation.require(summary.get("schema_version") == EXPECTED_RECORD_SCHEMA, "summary schema mismatch")
    validation.require(summary.get("run_valid") is True, "run summary is not valid")
    validation.require(summary.get("finalization_status") == "VALID", "run was not finalized VALID")
    validation.require(summary.get("shutdown_drain_status") == "SUCCESS", "shutdown drain did not succeed")
    validation.require(summary.get("jsonl_equals_csv_rows") is True, "JSONL/CSV row counts differ")
    validation.require(summary.get("unique_pair_keys_equal_records") is True, "pair keys are not unique")
    validation.require(summary.get("measurement_queue_overflow_count") == 0, "measurement queue overflowed")
    validation.require(summary.get("research_recording_error_count") == 0, "research recorder reported errors")

    rows: list[dict[str, str]] = []
    header: list[str] = []
    if not csv_path.is_file():
        validation.errors.append(f"missing file: {csv_path}")
    else:
        try:
            with csv_path.open(encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                header = list(reader.fieldnames or ())
                rows = list(reader)
        except (OSError, csv.Error) as exc:
            validation.errors.append(f"invalid CSV {csv_path}: {exc}")
    missing_columns = sorted(
        (ALWAYS_REQUIRED_COLUMNS | OBSERVATION_COLUMNS | BBOX_COLUMNS | OPERATIONAL_SOURCE_COLUMNS | OPERATIONAL_RESULT_COLUMNS)
        - set(header)
    )
    validation.require(not missing_columns, f"missing CSV columns: {missing_columns}")
    validation.require(bool(rows), "pair_records.csv has no samples")
    if csv_path.is_file():
        validation.require(summary.get("pair_records_csv_sha256") == _sha256(csv_path), "CSV hash mismatch")
    if jsonl_path.is_file():
        validation.require(summary.get("pair_records_jsonl_sha256") == _sha256(jsonl_path), "JSONL hash mismatch")
    else:
        validation.errors.append(f"missing file: {jsonl_path}")
    validation.require(summary.get("csv_data_row_count") == len(rows), "summary CSV row count mismatch")
    validation.require(summary.get("jsonl_line_count") == len(rows), "summary JSONL row count mismatch")

    seen_samples: set[str] = set()
    seen_pairs: set[tuple[str, str]] = set()
    eligible_count = 0
    valid_count = 0
    operational_count = 0
    observation_count = 0
    expected_injection = config.get("attitude_injection", {})
    left_injected = expected_injection.get("left_applied_deg", {})
    right_injected = expected_injection.get("right_applied_deg", {})
    for index, row in enumerate(rows, start=1):
        prefix = f"row {index}"
        validation.require(row.get("record_sequence") == str(index), f"{prefix}: record_sequence mismatch")
        validation.require(row.get("schema_version") == EXPECTED_RECORD_SCHEMA, f"{prefix}: schema mismatch")
        validation.require(row.get("session_id") == session_id, f"{prefix}: session_id mismatch")
        validation.require(row.get("run_id") == run_id, f"{prefix}: run_id mismatch")
        validation.require(row.get("attempt_id") == case_id, f"{prefix}: attempt_id mismatch")
        validation.require(row.get("case_id") == case_id, f"{prefix}: case_id mismatch")
        validation.require(row.get("effective_config_sha256") == config_hash, f"{prefix}: config hash mismatch")
        sample_id = row.get("sample_id", "")
        pair_key = (row.get("epoch", ""), row.get("pair_id", ""))
        validation.require(sample_id == f"{pair_key[0]}:{pair_key[1]}", f"{prefix}: sample_id is not epoch:pair_id")
        validation.require(sample_id not in seen_samples, f"{prefix}: duplicate sample_id")
        validation.require(pair_key not in seen_pairs, f"{prefix}: duplicate epoch/pair_id")
        seen_samples.add(sample_id)
        seen_pairs.add(pair_key)

        validation.require(row.get("output_frame") == "baseline_center_frd", f"{prefix}: output frame mismatch")
        validation.require(row.get("output_axes") == "forward_right_down", f"{prefix}: output axes mismatch")
        validation.require(row.get("position_unit") == "m", f"{prefix}: position unit mismatch")
        validation.require(row.get("angle_unit") == "deg", f"{prefix}: angle unit mismatch")
        validation.require(row.get("timestamp_unit") == "ns", f"{prefix}: timestamp unit mismatch")
        validation.require(row.get("gt_valid") == "true", f"{prefix}: Gazebo GT is unavailable")
        validation.require(row.get("gt_source") == "gazebo_vehicle_reference_link_world_pose", f"{prefix}: GT source mismatch")
        validation.require(row.get("gt_link_name") == "vehicle_reference_link", f"{prefix}: GT link mismatch")
        validation.require(not _nonempty(row, {"gt_timestamp_ns", "gt_world_enu_x_m", "gt_world_enu_y_m", "gt_world_enu_z_m", "gt_forward_m", "gt_right_m", "gt_down_m"}), f"{prefix}: GT position/timestamp incomplete")
        validation.require(row.get("terminal_status", "") != "", f"{prefix}: terminal status missing")
        validation.require(row.get("measurement_status", "") != "", f"{prefix}: measurement status missing")
        validation.require(row.get("evaluation_status", "") != "", f"{prefix}: evaluation status missing")

        for side in ("left", "right"):
            for axis in ("roll", "pitch", "yaw"):
                key = f"{side}_injected_{axis}_deg"
                expected = (left_injected if side == "left" else right_injected).get(axis)
                validation.require(
                    _finite(row.get(key, "")) and abs(float(row[key]) - float(expected)) <= 1.0e-12,
                    f"{prefix}: applied injection mismatch in {key}",
                )

        left_observed = _bool(row.get("left_observation_valid", "")) is True
        right_observed = _bool(row.get("right_observation_valid", "")) is True
        if left_observed or right_observed:
            observation_count += 1
        if left_observed:
            validation.require(not _nonempty(row, {"left_u_px", "left_v_px", "left_confidence"}), f"{prefix}: left observation incomplete")
        if right_observed:
            validation.require(not _nonempty(row, {"right_u_px", "right_v_px", "right_confidence"}), f"{prefix}: right observation incomplete")
        if row.get("observation_mode") == "bbox_yolo" and left_observed:
            validation.require(
                not _nonempty(
                    row,
                    {
                        "left_bbox_xmin_px",
                        "left_bbox_ymin_px",
                        "left_bbox_xmax_px",
                        "left_bbox_ymax_px",
                    },
                ),
                f"{prefix}: left BBox corners incomplete",
            )
        if row.get("observation_mode") == "bbox_yolo" and right_observed:
            validation.require(
                not _nonempty(
                    row,
                    {
                        "right_bbox_xmin_px",
                        "right_bbox_ymin_px",
                        "right_bbox_xmax_px",
                        "right_bbox_ymax_px",
                    },
                ),
                f"{prefix}: right BBox corners incomplete",
            )

        operational_valid = _bool(row.get("operational_ray_valid", "")) is True
        if operational_valid:
            operational_count += 1
            missing = _nonempty(row, OPERATIONAL_SOURCE_COLUMNS | OPERATIONAL_RESULT_COLUMNS)
            validation.require(not missing, f"{prefix}: Operational source/result incomplete: {missing}")
            validation.require(row.get("operational_position_frame") == "baseline_center_frd", f"{prefix}: Operational position frame mismatch")
            validation.require(row.get("operational_attitude_parent_frame") == "earth_ned", f"{prefix}: Operational attitude parent mismatch")
            validation.require(row.get("operational_attitude_child_frame") == "body_frd", f"{prefix}: Operational attitude child mismatch")
            validation.require(row.get("operational_attitude_quaternion_order") == "wxyz", f"{prefix}: Operational quaternion order mismatch")
            validation.require(_unit_quaternion(row, "left"), f"{prefix}: left quaternion is not unit length")
            validation.require(_unit_quaternion(row, "right"), f"{prefix}: right quaternion is not unit length")
            validation.require(_unit_vector(row, "operational_ray_left_direction"), f"{prefix}: left Operational ray is not unit length")
            validation.require(_unit_vector(row, "operational_ray_right_direction"), f"{prefix}: right Operational ray is not unit length")
            validation.require(row.get("sync_valid") == "true", f"{prefix}: Operational sample is not time-aligned")
            for name, limit_s in (
                ("left_pose_alignment_dt_s", attitude_alignment_limit_s),
                ("right_pose_alignment_dt_s", attitude_alignment_limit_s),
                ("left_position_alignment_dt_s", position_alignment_limit_s),
                ("right_position_alignment_dt_s", position_alignment_limit_s),
            ):
                validation.require(
                    _finite(row.get(name, ""))
                    and abs(float(row[name])) <= limit_s,
                    f"{prefix}: {name} exceeds its recorded alignment limit",
                )
            validation.require(
                row.get("left_operational_attitude_timestamp_ns")
                == row.get("left_operational_pose_timestamp_ns"),
                f"{prefix}: left attitude/pose timestamp alias mismatch",
            )
            validation.require(
                row.get("right_operational_attitude_timestamp_ns")
                == row.get("right_operational_pose_timestamp_ns"),
                f"{prefix}: right attitude/pose timestamp alias mismatch",
            )
            validation.require(row.get("triangulation_valid") == "true", f"{prefix}: valid Operational ray lacks valid triangulation")

        sample_valid = _bool(row.get("sample_valid", "")) is True
        if sample_valid:
            valid_count += 1
            validation.require(operational_valid and left_observed and right_observed, f"{prefix}: sample_valid lacks complete runtime inputs")
        else:
            validation.require(row.get("failure_stage", "") != "", f"{prefix}: invalid sample lacks failure_stage")
            validation.require(row.get("failure_code", "") != "", f"{prefix}: invalid sample lacks failure_code")
            validation.require(row.get("failure_reason", "") != "", f"{prefix}: invalid sample lacks failure_reason")

        if _bool(row.get("sample_eligible", "")) is True:
            eligible_count += 1

    validation.require(eligible_count > 0, "run contains no calibration-eligible samples")
    validation.require(valid_count > 0, "run contains no valid Operational/GT samples")
    validation.require(operational_count > 0, "run contains no Operational rays")
    validation.require(observation_count > 0, "run contains no observations")
    validation.require(summary.get("eligible_sample_count") == eligible_count, "eligible sample count mismatch")

    case_result_path = case_dir / "case_result.json"
    if case_result_path.is_file():
        case_result = _load_json(case_result_path, validation)
        validation.require(case_result.get("status") == "PASS", "case_result status is not PASS")
        validation.require(case_result.get("case_id") == case_id, "case_result case_id mismatch")
        validation.require(
            case_result.get("effective_config_sha256") == config_hash,
            "case_result config hash mismatch",
        )
        validation.require(case_result.get("auto_exit_status") == 0, "Auto exit status is not zero")
        validation.require(not case_result.get("failure_stage"), "successful case has a failure_stage")
        validation.require(not case_result.get("failure_reason"), "successful case has a failure_reason")

    return {
        "valid": not validation.errors,
        "case_id": case_id,
        "record_schema": EXPECTED_RECORD_SCHEMA,
        "effective_config_sha256": config_hash,
        "sample_count": len(rows),
        "eligible_sample_count": eligible_count,
        "valid_sample_count": valid_count,
        "operational_ray_count": operational_count,
        "observation_count": observation_count,
        "errors": validation.errors,
        "warnings": validation.warnings,
    }


def _condition_count(sweep_dir: Path) -> int | None:
    contract = sweep_dir / "sweep_contract.txt"
    if not contract.is_file():
        return None
    for line in contract.read_text(encoding="utf-8").splitlines():
        if line.startswith("condition_count="):
            try:
                return int(line.split("=", 1)[1])
            except ValueError:
                return None
    return None


def validate_sweep(sweep_dir: Path) -> dict[str, Any]:
    errors: list[str] = []
    manifest_path = sweep_dir / "sweep_manifest.tsv"
    manifest_rows: list[dict[str, str]] = []
    if not manifest_path.is_file():
        errors.append(f"missing file: {manifest_path}")
    else:
        with manifest_path.open(encoding="utf-8", newline="") as handle:
            manifest_rows = list(csv.DictReader(handle, delimiter="\t"))
    condition_count = _condition_count(sweep_dir)
    expected_case_count = None if condition_count is None else (condition_count * 2 + 1 if condition_count else 1)
    if expected_case_count is None:
        errors.append("condition_count is missing from sweep_contract.txt")
    elif len(manifest_rows) != expected_case_count:
        errors.append(
            f"manifest case count mismatch: expected {expected_case_count}, got {len(manifest_rows)}"
        )
    case_ids = [row.get("case_id", "") for row in manifest_rows]
    if len(set(case_ids)) != len(case_ids):
        errors.append("manifest contains duplicate case IDs")
    results = []
    for row in manifest_rows:
        case_id = row.get("case_id", "")
        if row.get("status") != "PASS":
            errors.append(f"manifest case is not PASS: {case_id}")
        result = validate_case(sweep_dir / case_id)
        results.append(result)
        if not result["valid"]:
            errors.append(f"case validation failed: {case_id}")
        if row.get("effective_config_sha256") != result.get("effective_config_sha256"):
            errors.append(f"manifest config hash mismatch: {case_id}")
    return {
        "valid": not errors,
        "sweep_directory": str(sweep_dir.resolve()),
        "expected_case_count": expected_case_count,
        "case_count": len(manifest_rows),
        "passed_case_count": sum(row.get("status") == "PASS" for row in manifest_rows),
        "total_sample_count": sum(int(result.get("sample_count", 0)) for result in results),
        "total_eligible_sample_count": sum(int(result.get("eligible_sample_count", 0)) for result in results),
        "total_valid_sample_count": sum(int(result.get("valid_sample_count", 0)) for result in results),
        "errors": errors,
        "cases": results,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--case-dir", type=Path)
    group.add_argument("--sweep-dir", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    result = (
        validate_case(args.case_dir.resolve())
        if args.case_dir is not None
        else validate_sweep(args.sweep_dir.resolve())
    )
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if result.get("valid") else 1


if __name__ == "__main__":
    sys.exit(main())
