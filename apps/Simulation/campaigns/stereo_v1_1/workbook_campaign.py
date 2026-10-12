#!/usr/bin/env python3
"""Compile and execute a numeric multi-flight XLSX campaign.

The workbook is the per-flight source of truth.  This module deliberately uses
only Python's standard library to read OOXML, so the simulation's root Python
does not need an Excel package.  ``plan`` is non-flying; ``run`` and ``resume``
delegate each immutable flight to the existing serial capture/QC pipeline.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import posixpath
import re
import sys
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

SIM_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.campaign_status import (
    REVALIDATABLE_QC_REGRESSION_ERRORS,
    initialize,
    revalidate_known_qc_regression,
)


CAMPAIGN_ROOT = Path(__file__).resolve().parent
DEFAULT_WORKBOOK = SIM_ROOT / "config/campaign/v1_1/sweep_manifest_v1_1.xlsx"
LEGACY_DEFAULT_WORKBOOK = SIM_ROOT / "config/campaign/v1_1/multi_flight_parameters_v1_1_numeric_v3.xlsx"
NS = {
    "m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main",
    "r": "http://schemas.openxmlformats.org/officeDocument/2006/relationships",
}
LEGACY_HEADERS_SHA256 = "d2250d1e08d3cda83dd34b703d968a1020bfad6371e6b315c25a783fcf757471"
DEFAULTS_SHEET = "Defaults"
DEFAULTS_HEADERS = ("Parameter group", "Parameter description", "Default value")
SHEET_FIELDS = {
    "Runs": ("sequence", "repetition", "use_for_calibration", "use_for_final_evaluation", "reset_before_run"),
    "Path": ("sequence", "north_start_m", "north_end_m", "bow_amplitude_m", "s_amplitude_m", "glide_angle_deg"),
    "Motion": ("sequence", "heading_deg", "reference_update_hz", "setpoint_hz", "cruise_speed_mps", "ramp_up_s", "ramp_down_s", "intermediate_hold_s", "acceleration_limit_mps2", "tracking_warning_p95_m", "fixed_roll_pitch", "teleport_to_path", "science_includes_ramps"),
    "FrameGate": ("sequence", "origin_min_samples", "origin_max_speed_mps", "origin_max_receive_skew_s", "origin_max_residual_m", "origin_timeout_s", "origin_same_epoch", "origin_stable_reset", "verify_enu_ned_axes"),
    "Entry": ("sequence", "entry_lead_m", "entry_timeout_s", "science_start_window_m", "science_start_position_error_m", "science_start_speed_mps", "science_start_heading_error_deg", "science_start_stable_s", "science_start_max_gap_s", "science_start_readiness_timeout_s", "postscience_hold_s", "landing_timeout_s", "science_after_gates"),
    "CameraGeometry": ("sequence", "baseline_m", "left_east_m", "left_north_m", "left_up_m", "right_east_m", "right_north_m", "right_up_m", "fixed_world_centers"),
    "RenderedCameraPose": ("sequence", "left_roll_deg", "left_pitch_deg", "left_heading_deg", "right_roll_deg", "right_pitch_deg", "right_heading_deg", "apply_physical_pose"),
    "CameraModel": ("sequence", "horizontal_fov_rad", "image_width_px", "image_height_px", "fx_px", "fy_px", "cx_px", "cy_px", "near_clip_m", "far_clip_m", "camera_rate_hz", "lens_distortion"),
    "CameraPX4Mount": ("sequence", "left_mount_roll_deg", "left_mount_pitch_deg", "left_mount_yaw_deg", "right_mount_roll_deg", "right_mount_pitch_deg", "right_mount_yaw_deg", "px4_attitude_max_age_s", "px4_attitude_pair_skew_s"),
    "MagnetometerLeft": ("sequence", "mag_noise_x", "mag_noise_y", "mag_noise_z", "mag_bias_mean_x", "mag_bias_mean_y", "mag_bias_mean_z", "mag_bias_stddev_x", "mag_bias_stddev_y", "mag_bias_stddev_z"),
    "GyroscopeLeft": ("sequence", "gyro_noise_x", "gyro_noise_y", "gyro_noise_z", "gyro_bias_mean_x", "gyro_bias_mean_y", "gyro_bias_mean_z", "gyro_bias_stddev_x", "gyro_bias_stddev_y", "gyro_bias_stddev_z"),
    "GPS": ("sequence", "left_baseline_bearing_offset_deg", "right_baseline_bearing_offset_deg", "left_satellites", "right_satellites", "gps_triangulation_enabled", "use_left_gps", "use_right_gps", "gps_max_age_s"),
    "Vehicle": ("sequence", "spawn_east_m", "spawn_north_m", "spawn_up_m", "spawn_yaw_deg", "verify_spawn_yaw"),
    "Capture": ("sequence", "min_gt_rate_hz", "preferred_gt_rate_hz", "pair_tolerance_s", "max_gt_bracketing_gap_s", "yolo_target_hz", "yolo_imgsz_px", "yolo_confidence", "yolo_nms_iou", "retain_unmatched_frames", "require_lossless_raw"),
    "CaptureTiming": ("sequence", "use_sim_clock", "record_sim_epoch", "record_receive_time", "record_yolo_end_time", "record_source_frame_id", "measure_camera_rate", "linear_gt_position", "slerp_gt_orientation", "allow_gt_extrapolation"),
    "ImageObservation": ("sequence", "distinct_source_frame_pairs", "allow_source_frame_reuse", "retain_undetected_frames", "missing_bbox_is_null", "bbox_original_image_pixels", "inverse_letterbox_bbox", "measure_actual_yolo_rate", "overlay_only_sufficient"),
    "MagnetometerRight": ("sequence", "mag_noise_x", "mag_noise_y", "mag_noise_z", "mag_bias_mean_x", "mag_bias_mean_y", "mag_bias_mean_z", "mag_bias_stddev_x", "mag_bias_stddev_y", "mag_bias_stddev_z"),
    "GyroscopeRight": ("sequence", "gyro_noise_x", "gyro_noise_y", "gyro_noise_z", "gyro_bias_mean_x", "gyro_bias_mean_y", "gyro_bias_mean_z", "gyro_bias_stddev_x", "gyro_bias_stddev_y", "gyro_bias_stddev_z"),
}
LEGACY_SHEET_ORDER = tuple(SHEET_FIELDS)
PARAMETER_SHEETS = tuple(name for name in SHEET_FIELDS if name != "Runs")
EXTRA_PARAMETER_FIELDS_BY_GROUP = {"Vehicle": ("model_id", "reference_link")}
V3_SHEET_ORDER = ("Runs", *PARAMETER_SHEETS, DEFAULTS_SHEET)
FLAG_FIELDS = {
    "Runs": ("use_for_calibration", "use_for_final_evaluation", "reset_before_run"),
    "Motion": ("fixed_roll_pitch", "teleport_to_path", "science_includes_ramps"),
    "FrameGate": ("origin_same_epoch", "origin_stable_reset", "verify_enu_ned_axes"),
    "Entry": ("science_after_gates",),
    "CameraGeometry": ("fixed_world_centers",),
    "RenderedCameraPose": ("apply_physical_pose",),
    "CameraModel": ("lens_distortion",),
    "GPS": ("gps_triangulation_enabled", "use_left_gps", "use_right_gps"),
    "Vehicle": ("verify_spawn_yaw",),
    "Capture": ("retain_unmatched_frames", "require_lossless_raw"),
    "CaptureTiming": ("use_sim_clock", "record_sim_epoch", "record_receive_time", "record_yolo_end_time", "record_source_frame_id", "measure_camera_rate", "linear_gt_position", "slerp_gt_orientation", "allow_gt_extrapolation"),
    "ImageObservation": ("distinct_source_frame_pairs", "allow_source_frame_reuse", "retain_undetected_frames", "missing_bbox_is_null", "bbox_original_image_pixels", "inverse_letterbox_bbox", "measure_actual_yolo_rate", "overlay_only_sufficient"),
}
INTEGER_FIELDS = {
    ("Runs", "sequence"), ("Runs", "repetition"), ("FrameGate", "origin_min_samples"),
    ("CameraModel", "image_width_px"), ("CameraModel", "image_height_px"),
    ("Capture", "yolo_imgsz_px"), ("GPS", "left_satellites"), ("GPS", "right_satellites"),
}

MANIFEST_V1_SHEET_FIELDS = {
    "Guide": ("section", "guidance"),
    "Campaign": (
        "schema_version", "campaign_id", "revision", "protocol_id",
        "instruction_ref", "source_manifest_sha256", "seed_policy",
        "seed_retry_policy", "expected_run_count", "failure_policy",
    ),
    "ParameterCatalog": (
        "parameter_id", "category", "display_name", "data_type", "unit",
        "per_run_allowed", "runtime_binding",
    ),
    "Parameters": (
        "parameter_id", "scope", "campaign_value", "instruction_ref", "reason",
    ),
    "RunManifest": (
        "sequence", "run_id", "attempt_id", "condition_label", "rig_group", "path_id",
        "repetition", "role", "data_split", "use_for_calibration",
        "use_for_final_evaluation", "reset_before_run", "seed_requested",
    ),
    "RunParameters": ("run_id", "parameter_id", "value", "instruction_ref"),
    "Requirements": (
        "requirement_id", "source_ref", "stage", "scope", "evaluator_id",
        "subject_id", "operator", "target_value", "target_upper", "unit",
        "severity", "instruction_text", "evidence_ref",
    ),
    "ArtifactContract": (
        "artifact_id", "path_pattern", "format", "schema_id", "required",
        "checksum_required", "validator_id", "source_ref",
    ),
    "ObservationContract": (
        "field_id", "data_type", "unit", "nullable", "time_frame_contract",
        "required", "validator_id", "source_ref",
    ),
    "OfflinePlan": (
        "plan_id", "stage", "method_id", "attitude_variant", "scenario_id",
        "input_split", "trial_count", "adds_simulation_runs", "release_gate",
        "source_ref",
    ),
    "ChangeNotes": (
        "change_id", "revision", "date", "summary", "reason",
        "affected_runs", "source_ref",
    ),
}
MANIFEST_V1_SHEET_ORDER = tuple(MANIFEST_V1_SHEET_FIELDS)
MANIFEST_V1_SCHEMA = "sweep_manifest_v1"
ENVIRONMENT_CONTRACT_FIELDS = (
    "contract_id", "scope", "applies_to", "setting_id", "declared_value",
    "evidence_path_from_sim_root", "expected_sha256", "validator_id",
    "instruction_ref", "note",
)
MANIFEST_V2_SHEET_FIELDS = {
    **MANIFEST_V1_SHEET_FIELDS,
    "EnvironmentContract": ENVIRONMENT_CONTRACT_FIELDS,
}
MANIFEST_V2_SHEET_ORDER = tuple(MANIFEST_V2_SHEET_FIELDS)
MANIFEST_V2_SCHEMA = "sweep_manifest_v2"
ENVIRONMENT_CONTRACT_REQUIRED_SETTINGS = frozenset({
    "source_world_asset", "installed_world_asset", "world_name",
    "lighting_background", "weather", "wind_setting",
})
ENVIRONMENT_WORLD_SOURCE_PATH = "stereo_ground_sim/worlds/stereo_ground_drone.sdf"
ENVIRONMENT_WORLD_INSTALLED_PATH = "install/stereo_ground_sim/share/stereo_ground_sim/worlds/stereo_ground_drone.sdf"
ENVIRONMENT_CONTRACT_VALIDATORS = frozenset({
    "asset_sha256", "world_name", "lighting_snapshot",
    "weather_snapshot", "no_wind_configured",
})
ALLOWED_PARAMETER_SCOPES = frozenset({"campaign_fixed", "run_variable"})
ALLOWED_SEED_POLICIES = frozenset({"manifest_required", "none"})
ALLOWED_REQUIREMENT_STAGES = frozenset({
    "plan", "preflight", "run_gate", "post_run", "campaign", "analysis", "handoff",
})
ALLOWED_REQUIREMENT_SEVERITIES = frozenset({"block", "warn", "info", "manual"})


def _parameter_data_type(group: str, field: str) -> str:
    if field in EXTRA_PARAMETER_FIELDS_BY_GROUP.get(group, ()):
        return "text"
    if field in FLAG_FIELDS.get(group, ()):
        return "boolean"
    if (group, field) in INTEGER_FIELDS:
        return "integer"
    return "number"


def _parameter_unit(group: str, field: str) -> str:
    if field in EXTRA_PARAMETER_FIELDS_BY_GROUP.get(group, ()):
        return "identifier"
    if field in FLAG_FIELDS.get(group, ()):
        return "boolean"
    if group.startswith("Magnetometer"):
        return "gauss"
    if group.startswith("Gyroscope"):
        return "rad/s"
    for suffix, unit in (
        ("_mps2", "m/s^2"), ("_mps", "m/s"), ("_hz", "Hz"),
        ("_rad", "rad"), ("_deg", "deg"), ("_px", "px"),
        ("_m", "m"), ("_s", "s"),
    ):
        if field.endswith(suffix):
            return unit
    if (group, field) in INTEGER_FIELDS:
        return "count"
    return "unitless"


def parameter_catalog() -> list[dict[str, object]]:
    """Return the code-owned parameter catalog exposed to workbook authors."""
    catalog = []
    for group in PARAMETER_SHEETS:
        for field in SHEET_FIELDS[group][1:]:
            catalog.append({
                "parameter_id": f"{group}.{field}",
                "category": group,
                "display_name": field.replace("_", " "),
                "data_type": _parameter_data_type(group, field),
                "unit": _parameter_unit(group, field),
                "per_run_allowed": True,
                "runtime_binding": f"workbook_parameters.{group}.{field}",
            })
    for group, fields in EXTRA_PARAMETER_FIELDS_BY_GROUP.items():
        for field in fields:
            catalog.append({
                "parameter_id": f"{group}.{field}",
                "category": group,
                "display_name": field.replace("_", " "),
                "data_type": _parameter_data_type(group, field),
                "unit": _parameter_unit(group, field),
                "per_run_allowed": False,
                "runtime_binding": f"workbook_parameters.{group}.{field}",
            })
    return catalog


def _as_text(value, sheet: str, row: int, field: str, *, required: bool = True):
    if value in (None, ""):
        if required:
            raise ValueError(f"{sheet}:{row} {field} is required")
        return None
    if not isinstance(value, str):
        raise ValueError(f"{sheet}:{row} {field} must be text")
    value = value.strip()
    if not value and required:
        raise ValueError(f"{sheet}:{row} {field} is required")
    return value or None


def _as_integer(value, sheet: str, row: int, field: str, *, required: bool = True):
    if value in (None, ""):
        if required:
            raise ValueError(f"{sheet}:{row} {field} is required")
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or int(value) != value:
        raise ValueError(f"{sheet}:{row} {field} must be an integer")
    return int(value)


def _as_flag(value, sheet: str, row: int, field: str) -> bool:
    number = _as_integer(value, sheet, row, field)
    if number not in (0, 1):
        raise ValueError(f"{sheet}:{row} {field} must be 0 or 1")
    return bool(number)


def _catalog_value(value, data_type: str, sheet: str, row: int, field: str):
    if value in (None, ""):
        raise ValueError(f"{sheet}:{row} {field} is required")
    if data_type == "boolean":
        return int(_as_flag(value, sheet, row, field))
    if data_type == "integer":
        return _as_integer(value, sheet, row, field)
    if data_type == "number":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{sheet}:{row} {field} must be numeric")
        if not math.isfinite(float(value)):
            raise ValueError(f"{sheet}:{row} {field} must be finite")
        return float(value)
    if data_type == "text":
        return _as_text(value, sheet, row, field)
    raise ValueError(f"{sheet}:{row} unsupported catalog data type {data_type!r}")


def _manifest_table_rows(archive: zipfile.ZipFile, sheet_targets: list[tuple[str, str]],
                         shared_strings: list[str],
                         schema_fields: dict[str, tuple[str, ...]]) -> dict[str, list[dict[str, object]]]:
    tables = {}
    for sheet_name, member in sheet_targets:
        root = ET.fromstring(archive.read(member))
        data = root.find("m:sheetData", NS)
        if data is None:
            raise ValueError(f"worksheet has no sheetData: {sheet_name}")
        expected = schema_fields[sheet_name]
        header_cells, rows = _sheet_rows(data, sheet_name, len(expected), shared_strings)
        headers = tuple(_as_text(header_cells[i], sheet_name, 1, "header")
                        for i in range(1, len(expected) + 1))
        if headers != expected:
            raise ValueError(f"{sheet_name} headers differ from the supported manifest schema")
        parsed = []
        for excel_row, cells in rows:
            if any(column > len(expected) for column in cells):
                raise ValueError(f"{sheet_name}:{excel_row} contains an extra column")
            parsed.append({
                field: cells.get(column)
                for column, field in enumerate(expected, start=1)
            })
        tables[sheet_name] = parsed
    return tables


def _verify_current_protocol_manifest(campaign: dict, runs: list[dict],
                                     parameter_rows: dict, run_parameters: dict) -> None:
    """Pin the supplied v1.1 student instruction's explicit 39-run ordering."""
    if campaign["protocol_id"] != "stereo_calibration_campaign_v1_1":
        return
    manifest_path = SIM_ROOT / "작업지시서" / "04_run_manifest.json"
    if not manifest_path.is_file():
        raise ValueError("the v1.1 instruction run manifest is missing")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    expected = manifest.get("runs", [])
    if len(expected) != len(runs):
        raise ValueError("workbook run count differs from the v1.1 instruction manifest")
    fields = ("sequence", "run_id", "attempt_id", "rig_group", "path_id", "repetition",
              "seed_requested", "role")
    for actual, source in zip(runs, expected):
        for field in fields:
            if actual[field] != source[field]:
                raise ValueError(
                    f"RunManifest:{actual['sequence']} {field} differs from the v1.1 instruction"
                )
        if actual["data_split"] != source["planned_data_split"]:
            raise ValueError(
                f"RunManifest:{actual['sequence']} data_split differs from the v1.1 instruction"
            )
        if actual["use_for_calibration"] != (source["role"] == "calibration"):
            raise ValueError(f"RunManifest:{actual['sequence']} calibration role flag is inconsistent")
        if actual["use_for_final_evaluation"] != (source["role"] == "final_evaluation"):
            raise ValueError(f"RunManifest:{actual['sequence']} final-evaluation role flag is inconsistent")
        for side in ("left", "right"):
            parameter_id = f"RenderedCameraPose.{side}_heading_deg"
            definition = parameter_rows[parameter_id]
            actual_heading = (definition["campaign_value"] if definition["scope"] == "campaign_fixed"
                              else run_parameters[actual["run_id"]][parameter_id])
            if not _close(actual_heading, source["true_camera_heading_deg"][side]):
                raise ValueError(f"RunManifest:{actual['sequence']} {side} rendered heading differs from instruction")
    source_hash = sha256_file(manifest_path)
    if campaign["source_manifest_sha256"] != source_hash:
        raise ValueError("Campaign source_manifest_sha256 differs from the instruction manifest")


def _read_manifest_workbook(source: Path, archive: zipfile.ZipFile,
                            sheet_targets: list[tuple[str, str]],
                            shared_strings: list[str], *,
                            schema_fields: dict[str, tuple[str, ...]],
                            schema_version: str) -> dict:
    tables = _manifest_table_rows(archive, sheet_targets, shared_strings, schema_fields)
    for sheet_name in ("Guide", "ParameterCatalog", "Parameters", "RunManifest",
                       "Requirements", "ArtifactContract", "ObservationContract",
                       "OfflinePlan", "ChangeNotes"):
        rows = tables[sheet_name]
        ids = [row[next(iter(schema_fields[sheet_name]))] for row in rows]
        if len(ids) != len(set(ids)):
            raise ValueError(f"{sheet_name} has duplicate first-column identifiers")
    if "EnvironmentContract" in tables:
        rows = tables["EnvironmentContract"]
        ids = [row["contract_id"] for row in rows]
        if len(ids) != len(set(ids)):
            raise ValueError("EnvironmentContract has duplicate contract_id values")

    campaigns = tables["Campaign"]
    if len(campaigns) != 1:
        raise ValueError("Campaign must contain exactly one metadata row")
    raw_campaign = campaigns[0]
    campaign = {field: raw_campaign[field] for field in schema_fields["Campaign"]}
    for field in ("schema_version", "campaign_id", "revision", "protocol_id",
                  "instruction_ref", "source_manifest_sha256", "seed_policy",
                  "seed_retry_policy", "failure_policy"):
        campaign[field] = _as_text(campaign[field], "Campaign", 2, field)
    if campaign["schema_version"] != schema_version:
        raise ValueError(f"unsupported campaign schema {campaign['schema_version']!r}")
    if campaign["seed_policy"] not in ALLOWED_SEED_POLICIES:
        raise ValueError(f"unsupported seed_policy {campaign['seed_policy']!r}")
    campaign["expected_run_count"] = _as_integer(
        campaign["expected_run_count"], "Campaign", 2, "expected_run_count"
    )
    if campaign["expected_run_count"] <= 0 or campaign["expected_run_count"] > 999:
        raise ValueError("Campaign expected_run_count must be from 1 to 999")
    if not isinstance(campaign["source_manifest_sha256"], str) or len(campaign["source_manifest_sha256"]) != 64:
        raise ValueError("Campaign source_manifest_sha256 must be a 64-character SHA256")

    observed_catalog = []
    for row_number, raw in enumerate(tables["ParameterCatalog"], start=2):
        entry = {field: _as_text(raw[field], "ParameterCatalog", row_number, field)
                 for field in ("parameter_id", "category", "display_name", "data_type", "unit", "runtime_binding")}
        entry["per_run_allowed"] = _as_flag(raw["per_run_allowed"], "ParameterCatalog", row_number, "per_run_allowed")
        if entry["data_type"] not in ("boolean", "integer", "number", "text"):
            raise ValueError(f"ParameterCatalog:{row_number} has an unsupported data_type")
        observed_catalog.append(entry)
    code_catalog = parameter_catalog()
    if observed_catalog != code_catalog:
        expected_by_id = {row["parameter_id"]: row for row in code_catalog}
        actual_by_id = {row["parameter_id"]: row for row in observed_catalog}
        missing = sorted(expected_by_id.keys() - actual_by_id.keys())
        extra = sorted(actual_by_id.keys() - expected_by_id.keys())
        changed = sorted(key for key in expected_by_id.keys() & actual_by_id.keys()
                         if expected_by_id[key] != actual_by_id[key])
        raise ValueError(f"ParameterCatalog differs from supported runtime catalog (missing={missing}, extra={extra}, changed={changed})")
    catalog_by_id = {row["parameter_id"]: row for row in observed_catalog}

    runs = []
    for row_number, raw in enumerate(tables["RunManifest"], start=2):
        run = {
            "sequence": _as_integer(raw["sequence"], "RunManifest", row_number, "sequence"),
            "run_id": _as_text(raw["run_id"], "RunManifest", row_number, "run_id"),
            "attempt_id": _as_text(raw["attempt_id"], "RunManifest", row_number, "attempt_id"),
            "condition_label": _as_text(raw["condition_label"], "RunManifest", row_number, "condition_label"),
            "rig_group": _as_text(raw["rig_group"], "RunManifest", row_number, "rig_group"),
            "path_id": _as_text(raw["path_id"], "RunManifest", row_number, "path_id"),
            "repetition": _as_integer(raw["repetition"], "RunManifest", row_number, "repetition"),
            "role": _as_text(raw["role"], "RunManifest", row_number, "role"),
            "data_split": _as_text(raw["data_split"], "RunManifest", row_number, "data_split"),
            "use_for_calibration": _as_flag(raw["use_for_calibration"], "RunManifest", row_number, "use_for_calibration"),
            "use_for_final_evaluation": _as_flag(raw["use_for_final_evaluation"], "RunManifest", row_number, "use_for_final_evaluation"),
            "reset_before_run": _as_flag(raw["reset_before_run"], "RunManifest", row_number, "reset_before_run"),
            "seed_requested": _as_integer(raw["seed_requested"], "RunManifest", row_number, "seed_requested",
                                           required=campaign["seed_policy"] == "manifest_required"),
        }
        runs.append(run)
    if len(runs) != campaign["expected_run_count"]:
        raise ValueError("RunManifest row count differs from Campaign expected_run_count")
    if [row["sequence"] for row in runs] != list(range(1, len(runs) + 1)):
        raise ValueError("RunManifest sequence must be dense, one-based, and in worksheet order")
    if len({row["run_id"] for row in runs}) != len(runs):
        raise ValueError("RunManifest run_id values must be unique")
    if campaign["seed_policy"] == "manifest_required":
        seeds = [row["seed_requested"] for row in runs]
        if any(seed is None or seed < 0 for seed in seeds) or len(set(seeds)) != len(seeds):
            raise ValueError("manifest_required seeds must be nonnegative and unique")
    elif any(row["seed_requested"] is not None for row in runs):
        raise ValueError("RunManifest seeds must be blank when Campaign seed_policy is none")

    parameter_rows = {}
    for row_number, raw in enumerate(tables["Parameters"], start=2):
        parameter_id = _as_text(raw["parameter_id"], "Parameters", row_number, "parameter_id")
        if parameter_id not in catalog_by_id:
            raise ValueError(f"Parameters:{row_number} references unsupported parameter {parameter_id!r}")
        if parameter_id in parameter_rows:
            raise ValueError(f"Parameters has duplicate parameter {parameter_id}")
        scope = _as_text(raw["scope"], "Parameters", row_number, "scope")
        if scope not in ALLOWED_PARAMETER_SCOPES:
            raise ValueError(f"Parameters:{row_number} has unsupported scope {scope!r}")
        value = raw["campaign_value"]
        if scope == "campaign_fixed":
            value = _catalog_value(value, catalog_by_id[parameter_id]["data_type"],
                                   "Parameters", row_number, "campaign_value")
        elif value not in (None, ""):
            raise ValueError(f"Parameters:{row_number} run_variable value must be blank; use RunParameters")
        parameter_rows[parameter_id] = {
            "parameter_id": parameter_id, "scope": scope, "campaign_value": value,
            "instruction_ref": _as_text(raw["instruction_ref"], "Parameters", row_number, "instruction_ref"),
            "reason": _as_text(raw["reason"], "Parameters", row_number, "reason"),
        }
    if set(parameter_rows) != set(catalog_by_id):
        raise ValueError("Parameters must classify every supported catalog item exactly once")

    run_parameters = {row["run_id"]: {} for row in runs}
    run_lookup = {row["run_id"]: row for row in runs}
    for row_number, raw in enumerate(tables["RunParameters"], start=2):
        run_id = _as_text(raw["run_id"], "RunParameters", row_number, "run_id")
        parameter_id = _as_text(raw["parameter_id"], "RunParameters", row_number, "parameter_id")
        if run_id not in run_lookup:
            raise ValueError(f"RunParameters:{row_number} has unknown run_id {run_id!r}")
        if parameter_id not in parameter_rows:
            raise ValueError(f"RunParameters:{row_number} has unsupported parameter {parameter_id!r}")
        if parameter_rows[parameter_id]["scope"] != "run_variable":
            raise ValueError(f"RunParameters:{row_number} overrides campaign_fixed {parameter_id}")
        if parameter_id in run_parameters[run_id]:
            raise ValueError(f"RunParameters has duplicate {run_id}/{parameter_id}")
        value = _catalog_value(raw["value"], catalog_by_id[parameter_id]["data_type"],
                               "RunParameters", row_number, "value")
        run_parameters[run_id][parameter_id] = value
    variable_ids = {key for key, row in parameter_rows.items() if row["scope"] == "run_variable"}
    if not variable_ids:
        raise ValueError("Parameters must mark at least one parameter run_variable")
    if any(not catalog_by_id[key]["per_run_allowed"] for key in variable_ids):
        raise ValueError("a run_variable parameter is not marked per_run_allowed in ParameterCatalog")
    for run_id, actual in run_parameters.items():
        if set(actual) != variable_ids:
            missing = sorted(variable_ids - actual.keys())
            extra = sorted(actual.keys() - variable_ids)
            raise ValueError(f"RunParameters for {run_id} is incomplete (missing={missing}, extra={extra})")

    _verify_current_protocol_manifest(campaign, runs, parameter_rows, run_parameters)

    resolved_sheets = {name: {} for name in PARAMETER_SHEETS}
    parameter_sources = {}
    for run in runs:
        sequence, run_id = run["sequence"], run["run_id"]
        resolved_sheets.setdefault("Runs", {})[sequence] = {
            "sequence": sequence, "repetition": run["repetition"],
            "use_for_calibration": int(run["use_for_calibration"]),
            "use_for_final_evaluation": int(run["use_for_final_evaluation"]),
            "reset_before_run": int(run["reset_before_run"]),
        }
        sources = {}
        for parameter_id, definition in parameter_rows.items():
            value = (definition["campaign_value"] if definition["scope"] == "campaign_fixed"
                     else run_parameters[run_id][parameter_id])
            group, field = parameter_id.split(".", 1)
            resolved_sheets[group].setdefault(sequence, {"sequence": sequence})[field] = value
            sources.setdefault(group, {})[field] = definition["scope"]
        parameter_sources[sequence] = sources
    for group in PARAMETER_SHEETS:
        for sequence in range(1, len(runs) + 1):
            fields = (*SHEET_FIELDS[group], *EXTRA_PARAMETER_FIELDS_BY_GROUP.get(group, ()))
            resolved_sheets[group][sequence] = {
                field: resolved_sheets[group][sequence][field]
                for field in fields
            }

    def generic_records(sheet_name: str, id_field: str, required_fields: tuple[str, ...]):
        output = []
        seen = set()
        for row_number, raw in enumerate(tables[sheet_name], start=2):
            record = {}
            for field in schema_fields[sheet_name]:
                value = raw[field]
                if field in required_fields:
                    value = _as_text(value, sheet_name, row_number, field)
                elif value == "":
                    value = None
                record[field] = value
            identifier = record[id_field]
            if identifier in seen:
                raise ValueError(f"{sheet_name} has duplicate {id_field} {identifier!r}")
            seen.add(identifier)
            output.append(record)
        return output

    requirements = generic_records("Requirements", "requirement_id",
                                   ("requirement_id", "source_ref", "stage", "scope", "evaluator_id",
                                    "subject_id", "operator", "severity", "instruction_text"))
    for row_number, row in enumerate(requirements, start=2):
        if row["stage"] not in ALLOWED_REQUIREMENT_STAGES:
            raise ValueError(f"Requirements:{row_number} has an unsupported stage")
        if row["severity"] not in ALLOWED_REQUIREMENT_SEVERITIES:
            raise ValueError(f"Requirements:{row_number} has an unsupported severity")
    artifact_contract = generic_records("ArtifactContract", "artifact_id",
                                        ("artifact_id", "path_pattern", "format", "schema_id",
                                         "validator_id", "source_ref"))
    for row_number, row in enumerate(artifact_contract, start=2):
        row["required"] = _as_flag(row["required"], "ArtifactContract", row_number, "required")
        row["checksum_required"] = _as_flag(
            row["checksum_required"], "ArtifactContract", row_number, "checksum_required"
        )
        relative = posixpath.normpath(str(row["path_pattern"]).replace("\\", "/"))
        if (relative.startswith("/") or relative == ".." or relative.startswith("../")
                or relative in ("", ".")):
            raise ValueError(f"ArtifactContract:{row_number} path_pattern must remain relative to its artifact root")
    observation_contract = generic_records("ObservationContract", "field_id",
                                           ("field_id", "data_type", "time_frame_contract",
                                            "validator_id", "source_ref"))
    for row_number, row in enumerate(observation_contract, start=2):
        row["nullable"] = _as_flag(row["nullable"], "ObservationContract", row_number, "nullable")
        row["required"] = _as_flag(row["required"], "ObservationContract", row_number, "required")
    offline_plan = generic_records("OfflinePlan", "plan_id",
                                   ("plan_id", "stage", "method_id", "attitude_variant",
                                    "scenario_id", "input_split", "release_gate", "source_ref"))
    change_notes = generic_records("ChangeNotes", "change_id",
                                   ("change_id", "revision", "date", "summary", "reason", "source_ref"))
    environment_contract = []
    if schema_version == MANIFEST_V2_SCHEMA:
        environment_contract = generic_records(
            "EnvironmentContract", "contract_id", ENVIRONMENT_CONTRACT_FIELDS
        )
        by_setting = {}
        expected_validators = {
            "source_world_asset": "asset_sha256",
            "installed_world_asset": "asset_sha256",
            "world_name": "world_name",
            "lighting_background": "lighting_snapshot",
            "weather": "weather_snapshot",
            "wind_setting": "no_wind_configured",
        }
        for row_number, row in enumerate(environment_contract, start=2):
            if row["scope"] != "campaign_fixed" or row["applies_to"] != "all_runs":
                raise ValueError(
                    f"EnvironmentContract:{row_number} currently supports campaign_fixed/all_runs only"
                )
            setting_id = row["setting_id"]
            if setting_id not in expected_validators:
                raise ValueError(f"EnvironmentContract:{row_number} has unsupported setting_id {setting_id!r}")
            if row["validator_id"] not in ENVIRONMENT_CONTRACT_VALIDATORS:
                raise ValueError(f"EnvironmentContract:{row_number} has unsupported validator_id")
            if setting_id in by_setting:
                raise ValueError(f"EnvironmentContract has duplicate setting_id {setting_id!r}")
            if row["validator_id"] != expected_validators[setting_id]:
                raise ValueError(
                    f"EnvironmentContract:{row_number} validator_id does not match {setting_id}"
                )
            relative = posixpath.normpath(
                row["evidence_path_from_sim_root"].replace("\\", "/")
            )
            if (relative.startswith("/") or relative in ("", ".", "..")
                    or relative.startswith("../")):
                raise ValueError(
                    f"EnvironmentContract:{row_number} evidence path must stay relative to Simulation"
                )
            digest = row["expected_sha256"].lower()
            if not re.fullmatch(r"[0-9a-f]{64}", digest):
                raise ValueError(f"EnvironmentContract:{row_number} expected_sha256 must be a SHA256")
            row["expected_sha256"] = digest
            row["evidence_path_from_sim_root"] = relative
            by_setting[setting_id] = row
        missing_settings = sorted(set(expected_validators) - by_setting.keys())
        if missing_settings:
            raise ValueError(f"EnvironmentContract is missing required settings: {missing_settings}")
        source_world = by_setting["source_world_asset"]
        installed_world = by_setting["installed_world_asset"]
        if source_world["evidence_path_from_sim_root"] != ENVIRONMENT_WORLD_SOURCE_PATH:
            raise ValueError("EnvironmentContract source path differs from the active Gazebo world binding")
        if installed_world["evidence_path_from_sim_root"] != ENVIRONMENT_WORLD_INSTALLED_PATH:
            raise ValueError("EnvironmentContract installed path differs from the active Gazebo package binding")
        if source_world["expected_sha256"] != installed_world["expected_sha256"]:
            raise ValueError("EnvironmentContract source and installed world hashes must match")
        for setting_id in ("world_name", "lighting_background", "weather", "wind_setting"):
            row = by_setting[setting_id]
            if (row["evidence_path_from_sim_root"] != source_world["evidence_path_from_sim_root"]
                    or row["expected_sha256"] != source_world["expected_sha256"]):
                raise ValueError(
                    f"EnvironmentContract:{setting_id} must reference the pinned source world asset"
                )
        if (installed_world["evidence_path_from_sim_root"]
                == source_world["evidence_path_from_sim_root"]):
            raise ValueError("EnvironmentContract installed world asset must reference the installed copy")
        if by_setting["wind_setting"]["declared_value"] != "none_configured_in_world_sdf":
            raise ValueError("the current workbook runner only supports an unconfigured world wind setting")
        if not by_setting["weather"]["declared_value"].startswith("no_new_weather_sweep; atmosphere="):
            raise ValueError("the current workbook runner only supports the existing atmosphere without a weather sweep")
    return {
        "path": source,
        "sha256": sha256_file(source),
        "schema_version": schema_version,
        "sheets": resolved_sheets,
        "sequences": [row["sequence"] for row in runs],
        "defaults": None,
        "parameter_sources": parameter_sources,
        "campaign": campaign,
        "catalog": observed_catalog,
        "parameters": parameter_rows,
        "run_parameters": run_parameters,
        "run_manifest": runs,
        "requirements": requirements,
        "artifact_contract": artifact_contract,
        "observation_contract": observation_contract,
        "offline_plan": offline_plan,
        "change_notes": change_notes,
        "environment_contract": environment_contract,
        "headers_sha256": hashlib.sha256(json.dumps(
            schema_fields, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")).hexdigest(),
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _column_number(reference: str) -> int:
    letters = "".join(character for character in reference if character.isalpha()).upper()
    number = 0
    for character in letters:
        number = number * 26 + ord(character) - ord("A") + 1
    return number


def _cell_value(cell: ET.Element, shared_strings: list[str], sheet: str) -> int | float | str | None:
    if cell.find("m:f", NS) is not None:
        raise ValueError(f"{sheet}:{cell.attrib.get('r')} contains a formula; use numeric values")
    kind = cell.attrib.get("t")
    if kind == "inlineStr":
        inline = cell.find("m:is", NS)
        return "" if inline is None else "".join(item.text or "" for item in inline.findall(".//m:t", NS))
    value = cell.find("m:v", NS)
    if value is None or value.text is None:
        return None
    raw = value.text
    if kind == "s":
        try:
            return shared_strings[int(raw)]
        except (IndexError, ValueError):
            raise ValueError(f"{sheet}:{cell.attrib.get('r')} has an invalid shared-string index") from None
    if kind in (None, "n"):
        try:
            numeric = float(raw)
        except ValueError:
            raise ValueError(f"{sheet}:{cell.attrib.get('r')} is not numeric") from None
        if not math.isfinite(numeric):
            raise ValueError(f"{sheet}:{cell.attrib.get('r')} must be finite")
        if numeric.is_integer():
            return int(numeric)
        return numeric
    return raw


def _sheet_targets(archive: zipfile.ZipFile) -> list[tuple[str, str]]:
    workbook = ET.fromstring(archive.read("xl/workbook.xml"))
    relations = ET.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
    targets = {item.attrib["Id"]: item.attrib["Target"] for item in relations}
    result = []
    for sheet in workbook.findall("m:sheets/m:sheet", NS):
        name = sheet.attrib["name"]
        relation = sheet.attrib[f"{{{NS['r']}}}id"]
        target = targets[relation]
        member = posixpath.normpath(target.lstrip("/") if target.startswith("/")
                                    else posixpath.join("xl", target))
        if not member.startswith("xl/") or member not in archive.namelist():
            raise ValueError(f"worksheet path is invalid for {name}")
        if sheet.attrib.get("state", "visible") != "visible":
            raise ValueError(f"worksheet {name} must be visible")
        result.append((name, member))
    return result


def _sheet_rows(data: ET.Element, sheet_name: str, field_count: int, shared_strings: list[str]):
    nonempty_rows = []
    for row in data.findall("m:row", NS):
        cells = {}
        for cell in row.findall("m:c", NS):
            column = _column_number(cell.attrib["r"])
            value = _cell_value(cell, shared_strings, sheet_name)
            if value not in (None, ""):
                cells[column] = value
        if cells:
            nonempty_rows.append((int(row.attrib["r"]), cells))
    if not nonempty_rows or nonempty_rows[0][0] != 1:
        raise ValueError(f"{sheet_name} must have a header in row 1")
    header_cells = nonempty_rows[0][1]
    if set(header_cells) != set(range(1, field_count + 1)):
        raise ValueError(f"{sheet_name} header columns do not match the supported sheet shape")
    return header_cells, nonempty_rows[1:]


def _typed_numeric(sheet: str, field: str, value, cell_ref: str):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ValueError(f"{sheet}:{cell_ref} {field} must be numeric")
    if not math.isfinite(float(value)):
        raise ValueError(f"{sheet}:{cell_ref} {field} must be finite")
    is_integer = field == "sequence" or (sheet, field) in INTEGER_FIELDS
    if is_integer and int(value) != value:
        raise ValueError(f"{sheet}:{cell_ref} {field} must be an integer")
    if field in FLAG_FIELDS.get(sheet, ()) and value not in (0, 1):
        raise ValueError(f"{sheet}:{cell_ref} {field} must be 0 or 1")
    return int(value) if is_integer else float(value)


def _parse_full_numeric_rows(sheet_name: str, field_names: tuple[str, ...], rows) -> dict:
    parsed_rows = {}
    for excel_row, cells in rows:
        if set(cells) != set(range(1, len(field_names) + 1)):
            raise ValueError(f"{sheet_name}:{excel_row} contains a blank or extra parameter cell")
        parsed = {
            field: _typed_numeric(sheet_name, field, cells[column], f"{excel_row}")
            for column, field in enumerate(field_names, start=1)
        }
        sequence = parsed["sequence"]
        if sequence in parsed_rows:
            raise ValueError(f"{sheet_name} has duplicate sequence {sequence}")
        parsed_rows[sequence] = parsed
    return parsed_rows


def read_numeric_workbook(path: Path | str) -> dict:
    """Read legacy v2 or shared-default/sparse-override v3 numeric workbooks."""
    source = Path(path).expanduser().resolve()
    if source.suffix.lower() != ".xlsx" or not source.is_file():
        raise ValueError(f"workbook must be an existing .xlsx file: {source}")
    try:
        archive = zipfile.ZipFile(source)
    except (OSError, zipfile.BadZipFile) as exc:
        raise ValueError(f"cannot open XLSX workbook: {exc}") from exc
    with archive:
        names = archive.namelist()
        if "xl/workbook.xml" not in names or "xl/_rels/workbook.xml.rels" not in names:
            raise ValueError("file is not an OOXML spreadsheet workbook")
        shared_strings = []
        if "xl/sharedStrings.xml" in names:
            strings = ET.fromstring(archive.read("xl/sharedStrings.xml"))
            shared_strings = ["".join(item.text or "" for item in value.findall(".//m:t", NS))
                              for value in strings.findall("m:si", NS)]
        sheet_targets = _sheet_targets(archive)
        observed_names = tuple(name for name, _ in sheet_targets)
        if observed_names == LEGACY_SHEET_ORDER:
            schema_version = "stereo_v1_1_numeric_v2"
            legacy_schema = True
        elif observed_names == V3_SHEET_ORDER:
            schema_version = "stereo_v1_1_numeric_v3"
            legacy_schema = False
        elif observed_names == MANIFEST_V1_SHEET_ORDER:
            return _read_manifest_workbook(
                source, archive, sheet_targets, shared_strings,
                schema_fields=MANIFEST_V1_SHEET_FIELDS,
                schema_version=MANIFEST_V1_SCHEMA,
            )
        elif observed_names == MANIFEST_V2_SHEET_ORDER:
            return _read_manifest_workbook(
                source, archive, sheet_targets, shared_strings,
                schema_fields=MANIFEST_V2_SHEET_FIELDS,
                schema_version=MANIFEST_V2_SCHEMA,
            )
        else:
            raise ValueError(
                "workbook tabs/order must match numeric v2 or v3 schema: "
                f"{observed_names}"
            )

        observed_headers: dict[str, list[str]] = {}
        raw_rows: dict[str, list[tuple[int, dict[int, object]]]] = {}
        for sheet_name, member in sheet_targets:
            root = ET.fromstring(archive.read(member))
            data = root.find("m:sheetData", NS)
            if data is None:
                raise ValueError(f"worksheet has no sheetData: {sheet_name}")
            field_names = DEFAULTS_HEADERS if sheet_name == DEFAULTS_SHEET else SHEET_FIELDS[sheet_name]
            header_cells, rows = _sheet_rows(data, sheet_name, len(field_names), shared_strings)
            observed_headers[sheet_name] = [
                str(header_cells[index]) for index in range(1, len(field_names) + 1)
            ]
            raw_rows[sheet_name] = rows

        if legacy_schema:
            header_rows = observed_headers
        else:
            if tuple(observed_headers[DEFAULTS_SHEET]) != DEFAULTS_HEADERS:
                raise ValueError(f"{DEFAULTS_SHEET} headers differ from the supported v3 schema")
            header_rows = {name: observed_headers[name] for name in LEGACY_SHEET_ORDER}
        serialized_headers = json.dumps(
            header_rows, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        if hashlib.sha256(serialized_headers).hexdigest() != LEGACY_HEADERS_SHA256:
            raise ValueError("workbook parameter column labels/order differ from the supported schema")

    # Run sequence is the only dense row set in v3. It defines campaign size;
    # topic tabs may contain only rows that override the shared default.
    sheets: dict[str, dict[int, dict[str, int | float]]] = {
        "Runs": _parse_full_numeric_rows("Runs", SHEET_FIELDS["Runs"], raw_rows["Runs"])
    }
    expected_sequences = sorted(sheets["Runs"])
    if not expected_sequences or expected_sequences[0] < 1 or expected_sequences[-1] > 1000:
        raise ValueError("Run sequence numbers must be unique positive integers from 1 to 1000")
    if len(expected_sequences) > 999:
        raise ValueError("a workbook campaign is limited to 999 runs")
    expected_set = set(expected_sequences)
    parameter_sources: dict[int, dict[str, dict[str, str]]] = {
        sequence: {} for sequence in expected_sequences
    }

    if legacy_schema:
        for name in PARAMETER_SHEETS:
            rows = _parse_full_numeric_rows(name, SHEET_FIELDS[name], raw_rows[name])
            if set(rows) != expected_set:
                missing = sorted(expected_set - set(rows))
                extra = sorted(set(rows) - expected_set)
                raise ValueError(f"{name} run sequences differ from Runs (missing={missing}, extra={extra})")
            sheets[name] = rows
            for sequence in expected_sequences:
                parameter_sources[sequence][name] = {
                    field: "legacy_run_row" for field in SHEET_FIELDS[name][1:]
                }
        defaults = None
    else:
        defaults = {name: {} for name in PARAMETER_SHEETS}
        header_to_field = {}
        for name in PARAMETER_SHEETS:
            descriptions = observed_headers[name][1:]
            fields = SHEET_FIELDS[name][1:]
            if len(set(descriptions)) != len(descriptions):
                raise ValueError(f"{name} has duplicate parameter descriptions")
            header_to_field[name] = dict(zip(descriptions, fields))

        default_keys = set()
        for excel_row, cells in raw_rows[DEFAULTS_SHEET]:
            if set(cells) != {1, 2, 3}:
                raise ValueError(
                    f"{DEFAULTS_SHEET}:{excel_row} requires a group, parameter description, and numeric value"
                )
            group, description, raw_value = cells[1], cells[2], cells[3]
            if not isinstance(group, str) or group not in PARAMETER_SHEETS:
                raise ValueError(f"{DEFAULTS_SHEET}:{excel_row} has an unknown parameter group")
            if not isinstance(description, str) or description not in header_to_field[group]:
                raise ValueError(f"{DEFAULTS_SHEET}:{excel_row} has an unknown parameter description")
            field = header_to_field[group][description]
            key = (group, field)
            if key in default_keys:
                raise ValueError(f"{DEFAULTS_SHEET} has duplicate default for {group}.{field}")
            defaults[group][field] = _typed_numeric(
                group, field, raw_value, f"{excel_row}"
            )
            default_keys.add(key)

        expected_default_keys = {
            (name, field) for name in PARAMETER_SHEETS for field in SHEET_FIELDS[name][1:]
        }
        if default_keys != expected_default_keys:
            missing = sorted(expected_default_keys - default_keys)
            extra = sorted(default_keys - expected_default_keys)
            raise ValueError(f"Defaults parameter set is incomplete (missing={missing}, extra={extra})")

        for name in PARAMETER_SHEETS:
            overrides: dict[int, dict[str, int | float]] = {}
            fields = SHEET_FIELDS[name]
            for excel_row, cells in raw_rows[name]:
                if 1 not in cells or any(column < 1 or column > len(fields) for column in cells):
                    raise ValueError(f"{name}:{excel_row} has a missing sequence or extra parameter cell")
                sequence = _typed_numeric(name, "sequence", cells[1], f"{excel_row}")
                if sequence < 1:
                    raise ValueError(f"{name}:{excel_row} override sequence must be a positive run number")
                if sequence not in expected_set:
                    raise ValueError(f"{name}:{excel_row} override sequence {sequence} is not listed in Runs")
                if sequence in overrides:
                    raise ValueError(f"{name} has duplicate override sequence {sequence}")
                row_overrides = {}
                for column, field in enumerate(fields[1:], start=2):
                    if column not in cells:
                        continue
                    row_overrides[field] = _typed_numeric(
                        name, field, cells[column], f"{excel_row}"
                    )
                if not row_overrides:
                    raise ValueError(f"{name}:{excel_row} has no override values")
                overrides[sequence] = row_overrides

            sheets[name] = {}
            for sequence in expected_sequences:
                effective = {"sequence": sequence, **defaults[name]}
                effective.update(overrides.get(sequence, {}))
                sheets[name][sequence] = effective
                parameter_sources[sequence][name] = {
                    field: ("run_override" if field in overrides.get(sequence, {}) else "shared_default")
                    for field in fields[1:]
                }

    return {
        "path": source,
        "sha256": sha256_file(source),
        "headers_sha256": LEGACY_HEADERS_SHA256,
        "schema_version": schema_version,
        "sheets": sheets,
        "sequences": expected_sequences,
        "defaults": defaults,
        "parameter_sources": parameter_sources,
    }


def _run_parameters(workbook: dict, sequence: int) -> dict:
    return {name: dict(rows[sequence]) for name, rows in workbook["sheets"].items()}


def _evaluate_plan_requirements(workbook: dict) -> list[dict]:
    evaluations = []
    campaign = workbook.get("campaign") or {}
    runs = workbook.get("run_manifest", [])
    for requirement in workbook.get("requirements", []):
        result = {
            "requirement_id": requirement["requirement_id"],
            "stage": requirement["stage"],
            "severity": requirement["severity"],
            "evaluator_id": requirement["evaluator_id"],
            "status": "PENDING",
            "observed_value": None,
            "expected_value": requirement.get("target_value"),
        }
        if requirement["stage"] == "plan":
            if requirement["evaluator_id"] == "manifest.run_count":
                result["observed_value"] = len(runs)
            elif requirement["evaluator_id"] == "manifest.seed_policy":
                result["observed_value"] = campaign.get("seed_policy")
            elif requirement["evaluator_id"] in ("vehicle.model_id", "vehicle.reference_link"):
                parameter_id = requirement["evaluator_id"].replace("vehicle.", "Vehicle.")
                definition = workbook.get("parameters", {}).get(parameter_id)
                if definition is None:
                    raise ValueError(f"plan requirement references unsupported parameter {parameter_id}")
                result["observed_value"] = definition["campaign_value"]
            else:
                result["status"] = "MANUAL_REVIEW"
                evaluations.append(result)
                if requirement["severity"] == "block":
                    raise ValueError(
                        f"plan-blocking requirement {requirement['requirement_id']} has no supported evaluator"
                    )
                continue
            if requirement["operator"] != "equals":
                result["status"] = "UNSUPPORTED_OPERATOR"
            else:
                result["status"] = (
                    "PASS" if result["observed_value"] == result["expected_value"] else "FAIL"
                )
        evaluations.append(result)
        if result["status"] != "PASS" and requirement["stage"] == "plan" and requirement["severity"] == "block":
            raise ValueError(
                f"plan-blocking requirement {requirement['requirement_id']} failed: "
                f"observed={result['observed_value']!r}, expected={result['expected_value']!r}"
            )
    return evaluations


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _close(actual: float, expected: float, *, tolerance: float = 1e-6) -> bool:
    return math.isclose(float(actual), float(expected), rel_tol=0.0, abs_tol=tolerance)


def _path_point(s: float, path: dict, angle_rad: float) -> list[float]:
    north = path["north_start_m"] + (path["north_end_m"] - path["north_start_m"]) * s
    east = (path["bow_amplitude_m"] * math.sin(math.pi * s) ** 2
            + path["s_amplitude_m"] * math.sin(2 * math.pi * s) ** 3)
    return [east, north, north * math.tan(angle_rad)]


def _path_derivatives(s: float, path: dict, angle_rad: float) -> tuple[list[float], list[float]]:
    delta_north = path["north_end_m"] - path["north_start_m"]
    bow = path["bow_amplitude_m"]
    single_s = path["s_amplitude_m"]
    east_first = bow * math.pi * math.sin(2 * math.pi * s)
    east_second = 2 * bow * math.pi**2 * math.cos(2 * math.pi * s)
    k = 2 * math.pi
    sn, cs = math.sin(k * s), math.cos(k * s)
    east_first += 3 * single_s * k * sn * sn * cs
    east_second += 3 * single_s * k * k * (2 * sn * cs * cs - sn**3)
    return ([east_first, delta_north, delta_north * math.tan(angle_rad)],
            [east_second, 0.0, 0.0])


def reference_from_parameters(parameters: dict) -> dict:
    """Build a sampled 3-D arc-length reference from one workbook row."""
    path, motion = parameters["Path"], parameters["Motion"]
    angle = math.radians(path["glide_angle_deg"])
    speed = motion["cruise_speed_mps"]
    ramp_up = motion["ramp_up_s"]
    ramp_down = motion["ramp_down_s"]
    hz = motion["reference_update_hz"]
    if path["north_start_m"] <= 0 or path["north_end_m"] <= 0:
        raise ValueError("path north distances must be positive and stay away from the landing pad")
    if not (math.isfinite(angle) and math.isfinite(speed) and math.isfinite(hz)):
        raise ValueError("path and motion values must be finite")
    n = 10000
    points = [_path_point(index / n, path, angle) for index in range(n + 1)]
    arc = [0.0]
    for before, after in zip(points, points[1:]):
        arc.append(arc[-1] + math.dist(before, after))
    length = arc[-1]
    accel_distance = speed * ramp_up / 2
    decel_distance = speed * ramp_down / 2
    if length <= accel_distance + decel_distance:
        raise ValueError("path is too short for the requested acceleration/deceleration ramps")
    cruise_duration = (length - accel_distance - decel_distance) / speed
    duration = ramp_up + cruise_duration + ramp_down
    tick = 1 / hz
    times = [index * tick for index in range(int(math.floor(duration * hz)) + 1)]
    if times[-1] < duration:
        times.append(duration)
    rows = []
    for t in times:
        if t < ramp_up:
            dist = speed * 0.5 * (t - ramp_up / math.pi * math.sin(math.pi * t / ramp_up))
            actual_speed = speed * 0.5 * (1 - math.cos(math.pi * t / ramp_up))
            speed_dot = speed * 0.5 * math.pi / ramp_up * math.sin(math.pi * t / ramp_up)
        elif t <= ramp_up + cruise_duration:
            dist = accel_distance + speed * (t - ramp_up)
            actual_speed, speed_dot = speed, 0.0
        else:
            q = t - (ramp_up + cruise_duration)
            dist = length - decel_distance + speed * 0.5 * (
                q + ramp_down / math.pi * math.sin(math.pi * q / ramp_down)
            )
            actual_speed = speed * 0.5 * (1 + math.cos(math.pi * q / ramp_down))
            speed_dot = -speed * 0.5 * math.pi / ramp_down * math.sin(math.pi * q / ramp_down)
        dist = min(length, max(0.0, dist))
        index = max(1, min(n, __import__("bisect").bisect_left(arc, dist)))
        fraction = (dist - arc[index - 1]) / (arc[index] - arc[index - 1])
        s = ((index - 1) + fraction) / n
        tangent, second = _path_derivatives(s, path, angle)
        norm = math.sqrt(sum(value * value for value in tangent))
        tangent_second = sum(a * b for a, b in zip(tangent, second))
        velocity = [value / norm * actual_speed for value in tangent]
        acceleration = [
            tangent[axis] / norm * speed_dot
            + (second[axis] / norm**2 - tangent[axis] * tangent_second / norm**4) * actual_speed**2
            for axis in range(3)
        ]
        rows.append({
            "t_science_s": round(t, 8), "s": s,
            "position_world_enu_m": _path_point(s, path, angle),
            "velocity_world_enu_mps": velocity,
            "acceleration_world_enu_mps2": acceleration,
            "heading_deg_clockwise_from_north": motion["heading_deg"],
        })
    peak_acceleration = max(math.sqrt(sum(value * value for value in row["acceleration_world_enu_mps2"]))
                            for row in rows)
    if peak_acceleration > motion["acceleration_limit_mps2"]:
        raise ValueError(
            f"run {parameters['Runs']['sequence']} reference acceleration {peak_acceleration:.3f} m/s^2 "
            f"exceeds limit {motion['acceleration_limit_mps2']:.3f} m/s^2"
        )
    return {"length_3d_m": length, "science_duration_s": duration,
            "reference_update_hz": hz, "peak_acceleration_mps2": peak_acceleration,
            "samples": rows}


def _matmul3(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [[sum(left[row][k] * right[k][column] for k in range(3))
             for column in range(3)] for row in range(3)]


def _camera_rotation(roll_deg: float, pitch_deg: float, heading_deg: float) -> list[list[float]]:
    roll, pitch, heading = map(math.radians, (roll_deg, pitch_deg, heading_deg))
    nominal = [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]
    rx = [[1.0, 0.0, 0.0], [0.0, math.cos(pitch), -math.sin(pitch)],
          [0.0, math.sin(pitch), math.cos(pitch)]]
    rz_optical = [[math.cos(roll), -math.sin(roll), 0.0],
                  [math.sin(roll), math.cos(roll), 0.0], [0.0, 0.0, 1.0]]
    rz_world = [[math.cos(-heading), -math.sin(-heading), 0.0],
                [math.sin(-heading), math.cos(-heading), 0.0], [0.0, 0.0, 1.0]]
    return _matmul3(rz_world, _matmul3(nominal, _matmul3(rx, rz_optical)))


def validate_camera_visibility(parameters: dict, reference: dict) -> float:
    """Check the reference point stays inside both configured camera images."""
    geometry, pose, camera = (parameters[name] for name in
                              ("CameraGeometry", "RenderedCameraPose", "CameraModel"))
    centers = ((geometry["left_east_m"], geometry["left_north_m"], geometry["left_up_m"]),
               (geometry["right_east_m"], geometry["right_north_m"], geometry["right_up_m"]))
    poses = ((pose["left_roll_deg"], pose["left_pitch_deg"], pose["left_heading_deg"]),
             (pose["right_roll_deg"], pose["right_pitch_deg"], pose["right_heading_deg"]))
    min_margin = math.inf
    for center, orientation in zip(centers, poses):
        rotation = _camera_rotation(*orientation)
        for row in reference["samples"]:
            delta = [row["position_world_enu_m"][axis] - center[axis] for axis in range(3)]
            optical = [sum(rotation[world_axis][optical_axis] * delta[world_axis]
                           for world_axis in range(3)) for optical_axis in range(3)]
            depth = optical[2]
            if not camera["near_clip_m"] < depth < camera["far_clip_m"]:
                raise ValueError(f"run {parameters['Runs']['sequence']} target reference is outside camera clip range")
            u = camera["fx_px"] * optical[0] / depth + camera["cx_px"]
            v = camera["fy_px"] * optical[1] / depth + camera["cy_px"]
            margin = min(u, v, camera["image_width_px"] - u, camera["image_height_px"] - v)
            min_margin = min(min_margin, margin)
            if margin < 32:
                raise ValueError(
                    f"run {parameters['Runs']['sequence']} target reference leaves the 32 px camera margin"
                )
    return min_margin


def scenario_environment(parameters: dict, sequence: int) -> dict[str, str]:
    """Map a workbook row into the literal env variables consumed by Gazebo/PX4."""
    geometry = parameters["CameraGeometry"]
    pose = parameters["RenderedCameraPose"]
    camera = parameters["CameraModel"]
    mount = parameters["CameraPX4Mount"]
    gps = parameters["GPS"]
    vehicle = parameters["Vehicle"]
    motion = parameters["Motion"]
    values = {
        "SIM_SCENARIO_NAME": f"stereo_workbook_r{sequence:04d}",
        "CAMERA_BASELINE_M": geometry["baseline_m"],
        "CAMERA_HEIGHT_M": geometry["left_up_m"],
        "CAMERA_HORIZONTAL_FOV_RAD": camera["horizontal_fov_rad"],
        "CAMERA_IMAGE_WIDTH_PX": camera["image_width_px"],
        "CAMERA_IMAGE_HEIGHT_PX": camera["image_height_px"],
        "CAMERA_UPDATE_RATE_HZ": camera["camera_rate_hz"],
        "CAMERA_CLIP_NEAR_M": camera["near_clip_m"],
        "CAMERA_CLIP_FAR_M": camera["far_clip_m"],
        "CAM_L_OPTICAL_ROLL_DEG": pose["left_roll_deg"],
        "CAM_L_OPTICAL_PITCH_DEG": pose["left_pitch_deg"],
        "CAM_L_OPTICAL_YAW_DEG": pose["left_heading_deg"],
        "CAM_R_OPTICAL_ROLL_DEG": pose["right_roll_deg"],
        "CAM_R_OPTICAL_PITCH_DEG": pose["right_pitch_deg"],
        "CAM_R_OPTICAL_YAW_DEG": pose["right_heading_deg"],
        "CAM_L_PX4_ATTITUDE_ROLL_DEG": mount["left_mount_roll_deg"],
        "CAM_L_PX4_ATTITUDE_PITCH_DEG": mount["left_mount_pitch_deg"],
        "CAM_L_PX4_ATTITUDE_YAW_DEG": mount["left_mount_yaw_deg"],
        "CAM_R_PX4_ATTITUDE_ROLL_DEG": mount["right_mount_roll_deg"],
        "CAM_R_PX4_ATTITUDE_PITCH_DEG": mount["right_mount_pitch_deg"],
        "CAM_R_PX4_ATTITUDE_YAW_DEG": mount["right_mount_yaw_deg"],
        "CAM_L_GNSS_BASELINE_TO_FORWARD_YAW_DEG": gps["left_baseline_bearing_offset_deg"],
        "CAM_R_GNSS_BASELINE_TO_FORWARD_YAW_DEG": gps["right_baseline_bearing_offset_deg"],
        "CAM_L_SIM_GPS_USED": gps["left_satellites"],
        "CAM_R_SIM_GPS_USED": gps["right_satellites"],
        "DRONE_SPAWN_X_M": vehicle["spawn_east_m"],
        "DRONE_SPAWN_Y_M": vehicle["spawn_north_m"],
        "DRONE_SPAWN_Z_M": vehicle["spawn_up_m"],
        "DRONE_SPAWN_YAW_DEG": vehicle["spawn_yaw_deg"],
        "DRONE_SETPOINT_HZ": motion["setpoint_hz"],
        "ENABLE_GPS_BASELINE_TRIANGULATION": "true" if gps["gps_triangulation_enabled"] else "false",
        "GPS_BASELINE_MAX_AGE_S": gps["gps_max_age_s"],
        "PX4_ATTITUDE_MAX_AGE_S": mount["px4_attitude_max_age_s"],
        "PX4_ATTITUDE_PAIR_SKEW_S": mount["px4_attitude_pair_skew_s"],
    }
    for side_name, side_env in (("Left", "L"), ("Right", "R")):
        mag = parameters[f"Magnetometer{side_name}"]
        gyro = parameters[f"Gyroscope{side_name}"]
        for axis, letter in zip(("x", "y", "z"), ("X", "Y", "Z")):
            values[f"CAM_{side_env}_MAG_NOISE_{letter}_GAUSS"] = mag[f"mag_noise_{axis}"]
            values[f"CAM_{side_env}_MAG_BIAS_MEAN_{letter}_GAUSS"] = mag[f"mag_bias_mean_{axis}"]
            values[f"CAM_{side_env}_MAG_BIAS_STDDEV_{letter}_GAUSS"] = mag[f"mag_bias_stddev_{axis}"]
            values[f"CAM_{side_env}_GYRO_NOISE_{letter}_RAD_S"] = gyro[f"gyro_noise_{axis}"]
            values[f"CAM_{side_env}_GYRO_BIAS_MEAN_{letter}_RAD_S"] = gyro[f"gyro_bias_mean_{axis}"]
            values[f"CAM_{side_env}_GYRO_BIAS_STDDEV_{letter}_RAD_S"] = gyro[f"gyro_bias_stddev_{axis}"]
    return {name: str(value) for name, value in values.items()}


def write_scenario_environment(path: Path, parameters: dict, sequence: int) -> dict:
    values = scenario_environment(parameters, sequence)
    with path.open("x", encoding="utf-8") as handle:
        handle.write("# Generated from the immutable XLSX row; numeric values only.\n")
        for name, value in sorted(values.items()):
            handle.write(f"{name}={value}\n")
        handle.flush()
        os.fsync(handle.fileno())
    return values


def validate_supported_run(parameters: dict) -> None:
    """Reject workbook requests the current capture/controller cannot honor."""
    runs, motion = parameters["Runs"], parameters["Motion"]
    frame, entry = parameters["FrameGate"], parameters["Entry"]
    geometry, pose = parameters["CameraGeometry"], parameters["RenderedCameraPose"]
    camera, mount, gps = parameters["CameraModel"], parameters["CameraPX4Mount"], parameters["GPS"]
    vehicle = parameters["Vehicle"]
    capture, timing, observation = parameters["Capture"], parameters["CaptureTiming"], parameters["ImageObservation"]

    _require(vehicle.get("model_id", "x500_2m") == "x500_2m",
             "the current workbook runner only supports Vehicle.model_id=x500_2m")
    _require(vehicle.get("reference_link", "vehicle_reference_link") == "vehicle_reference_link",
             "the current capture and GT pipeline only supports Vehicle.reference_link=vehicle_reference_link")
    _require(vehicle.get("reference_link", "vehicle_reference_link") == "vehicle_reference_link",
             "the current capture and GT pipeline only supports Vehicle.reference_link=vehicle_reference_link")

    _require(runs["use_for_calibration"] + runs["use_for_final_evaluation"] == 1,
             "each run must be assigned to calibration or final evaluation, but not both")
    _require(runs["repetition"] > 0, "repetition number must be a positive integer")
    _require(runs["reset_before_run"] == 1, "the serial runner starts a clean SITL stack for every run; reset flag must be 1")
    _require(motion["fixed_roll_pitch"] == 0 and motion["teleport_to_path"] == 0,
             "the runner supports natural roll/pitch and dynamic entry; fixed attitude and teleport are unsupported")
    _require(motion["science_includes_ramps"] == 1, "science interval must include acceleration and deceleration ramps")
    _require(motion["intermediate_hold_s"] == 0,
             "intermediate hold is not executable until the workbook specifies where along s it occurs")
    _require(frame["origin_same_epoch"] == 1 and frame["origin_stable_reset"] == 1 and frame["verify_enu_ned_axes"] == 1,
             "origin gate requires same-epoch observations, stable PX4 reset counters, and ENU/NED verification")
    _require(entry["science_after_gates"] == 1, "science capture must start only after all start gates pass")
    _require(geometry["fixed_world_centers"] == 1,
             "camera centers must remain fixed in world coordinates for this capture pipeline")
    _require(geometry["baseline_m"] > 0, "stereo camera baseline must be positive")
    _require(_close(geometry["left_east_m"], -geometry["baseline_m"] / 2)
             and _close(geometry["right_east_m"], geometry["baseline_m"] / 2)
             and _close(geometry["left_north_m"], 0) and _close(geometry["right_north_m"], 0)
             and _close(geometry["left_up_m"], geometry["right_up_m"]),
             "current Gazebo rig supports a symmetric east-west baseline, common northing and common camera height")
    _require(pose["apply_physical_pose"] == 1, "rendered camera pose must be physically applied in Gazebo")
    _require(camera["lens_distortion"] == 0, "the active Gazebo camera model does not implement lens distortion")
    _require(camera["image_width_px"] > 0 and camera["image_height_px"] > 0
             and camera["horizontal_fov_rad"] > 0 and camera["horizontal_fov_rad"] < math.pi,
             "camera dimensions and field of view are out of range")
    expected_fx = camera["image_width_px"] / (2 * math.tan(camera["horizontal_fov_rad"] / 2))
    expected_fy = expected_fx
    _require(_close(camera["fx_px"], expected_fx, tolerance=0.1)
             and _close(camera["fy_px"], expected_fy, tolerance=0.1)
             and _close(camera["cx_px"], camera["image_width_px"] / 2)
             and _close(camera["cy_px"], camera["image_height_px"] / 2),
             "CameraModel intrinsics must match the horizontal-FOV pinhole model used by Gazebo and YOLO")
    _require(camera["near_clip_m"] > 0 and camera["far_clip_m"] > camera["near_clip_m"]
             and camera["camera_rate_hz"] > 0,
             "camera clip distances and update rate must be positive and ordered")
    _require(all(mount[key] > 0 for key in ("px4_attitude_max_age_s", "px4_attitude_pair_skew_s")),
             "PX4 attitude ages must be positive")
    _require(gps["gps_triangulation_enabled"] == 1 and gps["use_left_gps"] == 1 and gps["use_right_gps"] == 1,
             "the active recorder uses both camera GPS sources for baseline triangulation")
    _require(4 <= gps["left_satellites"] <= 64 and 4 <= gps["right_satellites"] <= 64,
             "PX4 simulated GPS requires 4 to 64 visible satellites")
    _require(entry["entry_lead_m"] > 0 and entry["entry_timeout_s"] > 0
             and entry["science_start_window_m"] > 0 and entry["science_start_position_error_m"] > 0
             and entry["science_start_speed_mps"] > 0 and entry["science_start_heading_error_deg"] > 0
             and entry["science_start_stable_s"] > 0 and entry["science_start_max_gap_s"] > 0
             and entry["science_start_readiness_timeout_s"] > 0 and entry["landing_timeout_s"] > 0,
             "entry/settle/landing gates must have valid positive limits")
    _require(frame["origin_min_samples"] >= 2 and frame["origin_max_speed_mps"] >= 0
             and frame["origin_max_receive_skew_s"] >= 0 and frame["origin_max_residual_m"] > 0
             and frame["origin_timeout_s"] > 0, "frame-origin gate settings are invalid")
    _require(motion["reference_update_hz"] >= 10 and 10 <= motion["setpoint_hz"] <= 100
             and motion["cruise_speed_mps"] > 0 and motion["ramp_up_s"] > 0 and motion["ramp_down_s"] > 0
             and motion["acceleration_limit_mps2"] > 0 and motion["tracking_warning_p95_m"] > 0,
             "motion settings are outside the supported range")
    _require(capture["min_gt_rate_hz"] > 0 and capture["preferred_gt_rate_hz"] >= capture["min_gt_rate_hz"]
             and capture["pair_tolerance_s"] >= 0 and capture["max_gt_bracketing_gap_s"] > 0,
             "capture rate and timing requirements are invalid")
    _require(capture["yolo_target_hz"] > 0 and capture["yolo_imgsz_px"] > 0
             and capture["yolo_imgsz_px"] % 32 == 0
             and 0 <= capture["yolo_confidence"] <= 1 and 0 < capture["yolo_nms_iou"] <= 1,
             "YOLO rate, confidence, IoU, and 32-pixel-compatible image size are invalid")
    _require(capture["retain_unmatched_frames"] == 1 and capture["require_lossless_raw"] == 1,
             "lossless raw images and unmatched frames are mandatory")
    expected_timing = {"use_sim_clock": 1, "record_sim_epoch": 1, "record_receive_time": 1,
                       "record_yolo_end_time": 1, "record_source_frame_id": 1,
                       "measure_camera_rate": 1, "linear_gt_position": 1,
                       "slerp_gt_orientation": 1, "allow_gt_extrapolation": 0}
    _require(timing == {"sequence": timing["sequence"], **expected_timing},
             "CaptureTiming flags must match the implemented simulation-clock, per-frame, no-extrapolation data contract")
    expected_observation = {"distinct_source_frame_pairs": 1, "allow_source_frame_reuse": 0,
                            "retain_undetected_frames": 1, "missing_bbox_is_null": 1,
                            "bbox_original_image_pixels": 1, "inverse_letterbox_bbox": 1,
                            "measure_actual_yolo_rate": 1, "overlay_only_sufficient": 0}
    _require(observation == {"sequence": observation["sequence"], **expected_observation},
             "ImageObservation flags must match the implemented source-frame and raw-image contract")
    for name in ("MagnetometerLeft", "MagnetometerRight"):
        for axis in "xyz":
            _require(parameters[name][f"mag_noise_{axis}"] >= 0
                     and parameters[name][f"mag_bias_stddev_{axis}"] >= 0,
                     f"{name} noise and bias standard deviations must be nonnegative")
    for name in ("GyroscopeLeft", "GyroscopeRight"):
        for axis in "xyz":
            _require(parameters[name][f"gyro_noise_{axis}"] >= 0
                     and parameters[name][f"gyro_bias_stddev_{axis}"] >= 0,
                     f"{name} noise and bias standard deviations must be nonnegative")


def build_workbook_plan(path: Path | str) -> dict:
    workbook = read_numeric_workbook(path)
    requirement_evaluations = _evaluate_plan_requirements(workbook)
    planned = []
    manifest_by_sequence = {
        row["sequence"]: row for row in workbook.get("run_manifest", [])
    }
    for sequence in workbook["sequences"]:
        parameters = _run_parameters(workbook, sequence)
        validate_supported_run(parameters)
        reference = reference_from_parameters(parameters)
        edge_margin = validate_camera_visibility(parameters, reference)
        runs = parameters["Runs"]
        path_values = parameters["Path"]
        manifest = manifest_by_sequence.get(sequence)
        if path_values["bow_amplitude_m"] != 0 and path_values["s_amplitude_m"] != 0:
            shape = "combined"
        elif path_values["bow_amplitude_m"] != 0:
            shape = "bow"
        elif path_values["s_amplitude_m"] != 0:
            shape = "single_s"
        else:
            shape = "straight"
        planned.append({
            "sequence": sequence,
            "run_id": manifest["run_id"] if manifest else f"XLSX-R{sequence:04d}",
            "attempt_id": manifest["attempt_id"] if manifest else None,
            "condition_label": manifest["condition_label"] if manifest else None,
            "rig_group": manifest["rig_group"] if manifest else "WORKBOOK",
            "path_id": manifest["path_id"] if manifest else "V0" if runs["use_for_final_evaluation"] else shape.upper(),
            "shape": shape,
            "repetition": manifest["repetition"] if manifest else runs["repetition"],
            "seed_requested": manifest["seed_requested"] if manifest else None,
            "seed_policy": (workbook.get("campaign", {}).get("seed_policy", "none")),
            "seed_retry_policy": (workbook.get("campaign", {}).get("seed_retry_policy")),
            "role": manifest["role"] if manifest else "final_evaluation" if runs["use_for_final_evaluation"] else "calibration",
            "data_split": manifest["data_split"] if manifest else None,
            "use_for_calibration": manifest["use_for_calibration"] if manifest else bool(runs["use_for_calibration"]),
            "use_for_final_evaluation": manifest["use_for_final_evaluation"] if manifest else bool(runs["use_for_final_evaluation"]),
            "reset_before_run": manifest["reset_before_run"] if manifest else True,
            "workbook_sha256": workbook["sha256"],
            "workbook_schema_version": workbook["schema_version"],
            "parameter_sources": workbook["parameter_sources"][sequence],
            "parameters": parameters,
            "artifact_contract": workbook.get("artifact_contract", []),
            "observation_contract": workbook.get("observation_contract", []),
            "requirements": workbook.get("requirements", []),
            "offline_plan": workbook.get("offline_plan", []),
            "environment_contract": workbook.get("environment_contract", []),
            "reference_preview": {
                "length_3d_m": reference["length_3d_m"],
                "science_duration_s": reference["science_duration_s"],
                "peak_acceleration_mps2": reference["peak_acceleration_mps2"],
                "minimum_camera_edge_margin_px": edge_margin,
            },
            "status": "PLANNED",
            "attempt_count": 0,
            "recording_path": None,
        })
    return {
        "schema_version": "stereo_v1_1_workbook_campaign_plan_v3",
        "execution_enabled": False,
        "flight_results_present": False,
        "workbook_schema_version": workbook["schema_version"],
        "workbook_file_name": workbook["path"].name,
        "workbook_sha256": workbook["sha256"],
        "workbook_headers_sha256": workbook["headers_sha256"],
        "planned_run_count": len(planned),
        "campaign": workbook.get("campaign"),
        "requirements": workbook.get("requirements", []),
        "requirement_evaluations": requirement_evaluations,
        "artifact_contract": workbook.get("artifact_contract", []),
        "observation_contract": workbook.get("observation_contract", []),
        "offline_plan": workbook.get("offline_plan", []),
        "environment_contract": workbook.get("environment_contract", []),
        "change_notes": workbook.get("change_notes", []),
        "seed_policy": (workbook.get("campaign", {}).get("seed_policy")
                        if workbook.get("campaign") else
                        "not_configured_by_workbook; simulator defaults are recorded per attempt"),
        "planned_runs": planned,
    }


def _write_json_new(path: Path, payload: dict) -> None:
    with path.open("x", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")


def initialize_workbook_campaign(output_root: Path, workbook_path: Path) -> dict:
    plan = build_workbook_plan(workbook_path)
    workbook_path = workbook_path.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    summary = initialize(output_root, plan=plan)
    frozen = output_root / "frozen_protocol"
    workbook_copy = frozen / "input_parameters.xlsx"
    with workbook_copy.open("xb") as handle:
        handle.write(workbook_path.read_bytes())
        handle.flush()
        os.fsync(handle.fileno())
    if sha256_file(workbook_copy) != plan["workbook_sha256"]:
        raise ValueError("frozen workbook copy does not match source hash")
    _write_json_new(output_root / "workbook_campaign_plan.json", plan)
    # The shared status engine owns attempt lifecycle; it accepts this compiled
    # plan so run count, IDs, roles and parameter rows stay aligned.
    checksums = output_root / "checksums.sha256"
    with checksums.open("a", encoding="utf-8") as handle:
        for path in (workbook_copy, output_root / "workbook_campaign_plan.json"):
            handle.write(f"{sha256_file(path)}  {path.relative_to(output_root)}\n")
    return summary


def _plan_digest(plan: dict) -> str:
    encoded = json.dumps(plan, ensure_ascii=False, sort_keys=True,
                         separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _load_frozen_plan(output_root: Path) -> dict:
    plan_path = output_root / "workbook_campaign_plan.json"
    status_path = output_root / "run_status.json"
    workbook_copy = output_root / "frozen_protocol" / "input_parameters.xlsx"
    if not plan_path.is_file() or not status_path.is_file() or not workbook_copy.is_file():
        raise ValueError("workbook campaign is not initialized or its frozen input is missing")
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    status = json.loads(status_path.read_text(encoding="utf-8"))
    if _plan_digest(plan) != status.get("campaign_plan_sha256"):
        raise ValueError("frozen workbook plan hash differs from run_status.json")
    if sha256_file(workbook_copy) != plan.get("workbook_sha256"):
        raise ValueError("frozen XLSX hash differs from the compiled workbook plan")
    if plan.get("workbook_sha256") != status.get("workbook_sha256"):
        raise ValueError("run_status.json references a different workbook hash")
    return plan


def execute_campaign(action: str, workbook_path: Path, output_root: Path) -> dict:
    """Run or resume the entire workbook in order, stopping at first failure."""
    if action not in ("run", "resume"):
        raise ValueError("workbook campaign action must be run or resume")
    output_root = output_root.expanduser().resolve()
    if action == "run":
        current_plan = build_workbook_plan(workbook_path)
        status_path = output_root / "run_status.json"
        if not status_path.is_file():
            initialize_workbook_campaign(output_root, workbook_path)
            plan = _load_frozen_plan(output_root)
        else:
            plan = _load_frozen_plan(output_root)
            if current_plan["workbook_sha256"] != plan["workbook_sha256"]:
                raise ValueError("workbook changed after campaign initialization; use resume or choose a new output ID")
    else:
        plan = _load_frozen_plan(output_root)

    from campaigns.stereo_v1_1.campaign_run import (
        _audit_valid, _existing_sitl_processes, _lock, _reconcile_interrupted, _run_one,
    )

    environment_report = output_root / "environment_report.json"
    if not environment_report.is_file():
        raise ValueError("environment_report.json is missing; run non-flying/live preflight first")
    if json.loads(environment_report.read_text(encoding="utf-8")).get("status") != "PREFLIGHT_PASS":
        raise ValueError("campaign preflight is not passing; workbook flights are blocked")
    rows = plan["planned_runs"]
    with _lock(output_root):
        if _existing_sitl_processes():
            raise RuntimeError("Gazebo/PX4 process remains active; stop it before workbook campaign execution")
        status = json.loads((output_root / "run_status.json").read_text(encoding="utf-8"))
        slots = {row["run_id"]: row for row in status["run_slots"]}
        if set(slots) != {row["run_id"] for row in rows}:
            raise ValueError("run_status.json slots do not match the frozen workbook plan")
        for source in rows:
            slot = slots[source["run_id"]]
            if slot["status"] in ("VALID", "VALID_WITH_WARNINGS"):
                continue
            if slot["status"] == "PROTOCOL_DEVIATION":
                attempt = slot.get("attempts", [])[-1] if slot.get("attempts") else {}
                attempt_dir = output_root / attempt.get("recording_path", "")
                report_path = attempt_dir / "qc_report.json"
                previous = json.loads(report_path.read_text(encoding="utf-8")) if report_path.is_file() else {}
                errors = previous.get("protocol_errors", [])
                if (set(errors) == REVALIDATABLE_QC_REGRESSION_ERRORS
                        and len(errors) == len(REVALIDATABLE_QC_REGRESSION_ERRORS)
                        and not previous.get("infrastructure_errors")):
                    from campaigns.stereo_v1_1.qc_attempt import validate as validate_attempt
                    fresh = validate_attempt(
                        attempt_dir, source["run_id"], attempt.get("attempt_id", ""),
                        planned_source=source,
                    )
                    if fresh["status"] in ("VALID", "VALID_WITH_WARNINGS"):
                        revalidate_known_qc_regression(
                            output_root, source["run_id"], attempt["attempt_id"], fresh,
                        )
                        status = json.loads((output_root / "run_status.json").read_text(encoding="utf-8"))
                        slots = {row["run_id"]: row for row in status["run_slots"]}
                        slot = slots[source["run_id"]]
                if slot["status"] == "PROTOCOL_DEVIATION":
                    raise RuntimeError(
                        f"{source['run_id']} remains a protocol deviation; "
                        "only the known workbook Vehicle QC regression can be revalidated automatically"
                    )
                if slot["status"] in ("VALID", "VALID_WITH_WARNINGS"):
                    continue
            if slot["status"] == "RUNNING":
                _reconcile_interrupted(output_root, slot)
                refreshed = json.loads((output_root / "run_status.json").read_text(encoding="utf-8"))
                slot = next(row for row in refreshed["run_slots"] if row["run_id"] == source["run_id"])
                if slot["status"] in ("VALID", "VALID_WITH_WARNINGS"):
                    continue
            qc = _run_one(output_root, source)
            if qc["status"] not in ("VALID", "VALID_WITH_WARNINGS"):
                raise RuntimeError(f"stopping workbook campaign after {source['run_id']} {qc['status']}")
            status = json.loads((output_root / "run_status.json").read_text(encoding="utf-8"))
            slots = {row["run_id"]: row for row in status["run_slots"]}
        audit = _audit_valid(output_root)
    return {"status": "WORKBOOK_CAMPAIGN_COMPLETE" if audit["valid_readback"] == audit["planned"]
            else "WORKBOOK_CAMPAIGN_PARTIAL", "workbook_sha256": plan["workbook_sha256"],
            **audit}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("plan", "initialize", "run", "resume"))
    parser.add_argument("--workbook", type=Path, default=DEFAULT_WORKBOOK)
    parser.add_argument("--output-root", type=Path)
    args = parser.parse_args()
    if args.action in ("run", "resume"):
        if args.output_root is None:
            parser.error(f"{args.action} requires --output-root")
        result = execute_campaign(args.action, args.workbook, args.output_root)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["status"] == "WORKBOOK_CAMPAIGN_COMPLETE" else 1
    plan = build_workbook_plan(args.workbook)
    if args.action == "initialize":
        if args.output_root is None:
            parser.error("initialize requires --output-root")
        initialize_workbook_campaign(args.output_root, args.workbook)
    print(json.dumps({
        "status": "WORKBOOK_PLAN_VALIDATED",
        "execution_enabled": False,
        "flight_results_present": False,
        "workbook_file_name": plan["workbook_file_name"],
        "workbook_sha256": plan["workbook_sha256"],
        "campaign": plan.get("campaign"),
        "environment_contract": plan.get("environment_contract", []),
        "offline_plan": plan.get("offline_plan", []),
        "planned_run_count": plan["planned_run_count"],
        "seed_policy": plan["seed_policy"],
        "requirement_evaluations": plan.get("requirement_evaluations", []),
        "run_variable_parameter_ids": sorted({
            f"{group}.{field}"
            for run in plan["planned_runs"]
            for group, sources in run["parameter_sources"].items()
            for field, scope in sources.items() if scope == "run_variable"
        }),
        "run_variable_assignment_count": sum(
            sum(scope == "run_variable" for fields in run["parameter_sources"].values()
                for scope in fields.values())
            for run in plan["planned_runs"]
        ),
        "planned_runs": [
            f"{run['sequence']:02d}\t{run['run_id']}\t{run['rig_group']}/{run['path_id']}"
            f"\tR{run['repetition']:02d}\tseed={run['seed_requested']}\t{run['role']}"
            f"\t{run.get('data_split') or '-'}"
            for run in plan["planned_runs"]
        ],
        "run_order": [row["run_id"] for row in plan["planned_runs"]],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
