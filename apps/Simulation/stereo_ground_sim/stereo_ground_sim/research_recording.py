"""Append-only atomic pair-level research recording for Simulation.

JSONL is the authoritative pair artifact.  CSV is derived only during an
explicit finalization pass.  This module intentionally has no ROS or geometry
dependencies so file I/O never needs to run under the measurement lock.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import platform
import queue
import socket
import sys
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping


RESEARCH_SCHEMA_VERSION = "s2_atomic_pair_v11"


# Flat scalar fields required to reproduce the reality-transferable
# Operational measurement.  Oracle/ideal fields remain separate diagnostics.
CALIBRATION_SAMPLE_COLUMNS = (
    "sample_id", "simulation_timestamp_ns",
    "left_observation_timestamp_ns", "right_observation_timestamp_ns",
    "left_right_dt_s", "left_pose_alignment_dt_s", "right_pose_alignment_dt_s",
    "left_position_alignment_dt_s", "right_position_alignment_dt_s",
    "observation_gt_dt_s", "sync_valid", "sync_failure_reason",
    "interpolation_method", "source_timestamp_before_ns",
    "source_timestamp_after_ns", "interpolation_alpha",
    "left_observation_source", "right_observation_source",
    "left_observation_valid", "right_observation_valid",
    "left_bbox_xmin_px", "left_bbox_ymin_px", "left_bbox_xmax_px", "left_bbox_ymax_px",
    "right_bbox_xmin_px", "right_bbox_ymin_px", "right_bbox_xmax_px", "right_bbox_ymax_px",
    "left_operational_position_baseline_center_frd_x_m",
    "left_operational_position_baseline_center_frd_y_m",
    "left_operational_position_baseline_center_frd_z_m",
    "right_operational_position_baseline_center_frd_x_m",
    "right_operational_position_baseline_center_frd_y_m",
    "right_operational_position_baseline_center_frd_z_m",
    "operational_position_source", "operational_position_frame",
    "operational_source_snapshot_timestamp_ns",
    "left_operational_position_timestamp_ns",
    "right_operational_position_timestamp_ns",
    "left_operational_position_valid", "right_operational_position_valid",
    "left_operational_attitude_qw", "left_operational_attitude_qx",
    "left_operational_attitude_qy", "left_operational_attitude_qz",
    "right_operational_attitude_qw", "right_operational_attitude_qx",
    "right_operational_attitude_qy", "right_operational_attitude_qz",
    "operational_attitude_source", "operational_attitude_parent_frame",
    "operational_attitude_child_frame", "operational_attitude_quaternion_order",
    "left_operational_pose_timestamp_ns", "right_operational_pose_timestamp_ns",
    "left_operational_attitude_timestamp_ns",
    "right_operational_attitude_timestamp_ns",
    "left_operational_pose_valid", "right_operational_pose_valid",
    "operational_x_m", "operational_y_m", "operational_z_m",
    "operational_range_m", "operational_azimuth_deg", "operational_elevation_deg",
    "operational_ray_generation_valid", "triangulation_valid",
    "triangulation_status", "triangulation_failure_reason",
    "near_parallel", "negative_left_depth", "negative_right_depth",
    "nonfinite_input", "nonfinite_output", "zero_or_small_baseline",
    "condition_exceeded", "sample_valid", "failure_stage", "failure_code",
    "failure_reason", "gt_timestamp_ns", "gt_valid", "gt_model_name", "gt_link_name",
    "gt_world_enu_x_m", "gt_world_enu_y_m", "gt_world_enu_z_m",
    "left_injected_roll_deg", "left_injected_pitch_deg", "left_injected_yaw_deg",
    "right_injected_roll_deg", "right_injected_pitch_deg", "right_injected_yaw_deg",
)


def _diagnostic_record_extension_columns() -> tuple[str, ...]:
    columns: list[str] = []
    for prefix in ("attitude", "operational"):
        columns.extend(
            (
                f"{prefix}_ray_valid",
                f"{prefix}_ray_left_origin_x_m",
                f"{prefix}_ray_left_origin_y_m",
                f"{prefix}_ray_left_origin_z_m",
                f"{prefix}_ray_left_direction_x",
                f"{prefix}_ray_left_direction_y",
                f"{prefix}_ray_left_direction_z",
                f"{prefix}_ray_right_origin_x_m",
                f"{prefix}_ray_right_origin_y_m",
                f"{prefix}_ray_right_origin_z_m",
                f"{prefix}_ray_right_direction_x",
                f"{prefix}_ray_right_direction_y",
                f"{prefix}_ray_right_direction_z",
                f"{prefix}_ray_left_depth_m",
                f"{prefix}_ray_right_depth_m",
                f"{prefix}_ray_geometry_rank",
                f"{prefix}_ray_condition_number",
                f"{prefix}_ray_midpoint_x_m",
                f"{prefix}_ray_midpoint_y_m",
                f"{prefix}_ray_midpoint_z_m",
                f"{prefix}_ray_left_closest_x_m",
                f"{prefix}_ray_left_closest_y_m",
                f"{prefix}_ray_left_closest_z_m",
                f"{prefix}_ray_right_closest_x_m",
                f"{prefix}_ray_right_closest_y_m",
                f"{prefix}_ray_right_closest_z_m",
                f"{prefix}_ray_delta_valid",
                f"{prefix}_ray_delta_failure_reason",
                f"{prefix}_ray_gt_valid",
                f"{prefix}_ray_gt_failure_reason",
                f"{prefix}_ray_gt_forward_error_m",
                f"{prefix}_ray_gt_right_error_m",
                f"{prefix}_ray_gt_down_error_m",
                f"{prefix}_ray_gt_range_error_m",
                f"{prefix}_ray_gt_position_error_m",
                f"{prefix}_ray_left_attitude_truth_residual_deg",
                f"{prefix}_ray_right_attitude_truth_residual_deg",
                f"{prefix}_ray_pixel_source",
                f"{prefix}_ray_attitude_source",
                f"{prefix}_ray_camera_center_source",
            )
        )
        if prefix in ("attitude", "operational"):
            columns.extend(
                (
                    "attitude_ray_source",
                    "attitude_ray_baseline_source",
                    "attitude_ray_gps_baseline_source",
                    "attitude_ray_gps_baseline_world_enu_x_m",
                    "attitude_ray_gps_baseline_world_enu_y_m",
                    "attitude_ray_gps_baseline_world_enu_z_m",
                    "attitude_ray_gps_baseline_length_m",
                    "attitude_ray_gps_baseline_delta_world_enu_x_m",
                    "attitude_ray_gps_baseline_delta_world_enu_y_m",
                    "attitude_ray_gps_baseline_delta_world_enu_z_m",
                    "attitude_ray_gps_baseline_delta_norm_m",
                )
            )
    columns.extend(
        (
            "left_px4_q_w",
            "left_px4_q_x",
            "left_px4_q_y",
            "left_px4_q_z",
            "right_px4_q_w",
            "right_px4_q_x",
            "right_px4_q_y",
            "right_px4_q_z",
            "px4_quaternion_order",
            "px4_attitude_semantics",
            "left_attitude_timestamp_us",
            "right_attitude_timestamp_us",
            "left_attitude_receive_monotonic_ns",
            "right_attitude_receive_monotonic_ns",
            "left_attitude_reset_counter",
            "right_attitude_reset_counter",
            "left_attitude_age_ms",
            "right_attitude_age_ms",
            "attitude_pair_skew_ms",
            "left_gps_timestamp_us",
            "right_gps_timestamp_us",
            "left_gps_age_ms",
            "right_gps_age_ms",
            "gps_pair_skew_ms",
            "simulation_epoch",
            "baseline_anchor_policy",
        )
    )
    # The attitude/operational blocks intentionally share a few provenance
    # names.  Keep the fixed CSV schema unique so DictWriter/DictReader do not
    # collapse duplicate header fields during finalization.
    return tuple(dict.fromkeys(columns))


DIAGNOSTIC_RECORD_EXTENSION_COLUMNS = _diagnostic_record_extension_columns()

PAIR_RECORD_COLUMNS = (
    "schema_version", "session_id", "run_id", "attempt_id", "case_id",
    "effective_config_sha256", "record_sequence", "record_type",
    *CALIBRATION_SAMPLE_COLUMNS,
    "pair_id", "epoch", "left_stamp_ns", "right_stamp_ns",
    "pair_midpoint_stamp_ns", "pair_delta_ns", "pairing_mode",
    "terminal_status", "terminal_reason", "mission_phase", "sample_eligible",
    "measurement_status", "evaluation_status", "gt_alignment_mode",
    "gt_before_stamp_ns", "gt_after_stamp_ns", "gt_interpolation_alpha",
    "gt_nearest_delta_ns", "observation_mode", "observation_source",
    "measurement_gt_independent", "reference_source",
    "projection_camera_pose_source", "left_u_px", "left_v_px", "right_u_px",
    "right_v_px", "left_confidence", "right_confidence",
    "camera_oracle_source", "camera_oracle_expected_sdf_sha256",
    "left_camera_oracle_gazebo_stamp_ns", "left_camera_oracle_base_entity_name",
    "left_camera_oracle_base_entity_id", "left_camera_oracle_optical_entity_name",
    "left_camera_oracle_optical_entity_id", "left_camera_oracle_optical_x_m",
    "left_camera_oracle_optical_y_m", "left_camera_oracle_optical_z_m",
    "left_camera_oracle_optical_qx", "left_camera_oracle_optical_qy",
    "left_camera_oracle_optical_qz", "left_camera_oracle_optical_qw",
    "right_camera_oracle_gazebo_stamp_ns", "right_camera_oracle_base_entity_name",
    "right_camera_oracle_base_entity_id", "right_camera_oracle_optical_entity_name",
    "right_camera_oracle_optical_entity_id", "right_camera_oracle_optical_x_m",
    "right_camera_oracle_optical_y_m", "right_camera_oracle_optical_z_m",
    "right_camera_oracle_optical_qx", "right_camera_oracle_optical_qy",
    "right_camera_oracle_optical_qz", "right_camera_oracle_optical_qw",
    "cam_l_optical_roll_deg", "cam_l_optical_pitch_deg", "cam_l_optical_yaw_deg",
    "cam_r_optical_roll_deg", "cam_r_optical_pitch_deg", "cam_r_optical_yaw_deg",
    # These six columns keep their v3/v4 names for schema-migration
    # compatibility (validate_pair_record's legacy path matches on an exact
    # subset of PAIR_RECORD_COLUMNS). They hold the configured independent
    # PX4 body attitude; the parameter names are cam_*_px4_attitude_*.
    "cam_l_px4_roll_deg", "cam_l_px4_pitch_deg", "cam_l_px4_yaw_deg",
    "cam_r_px4_roll_deg", "cam_r_px4_pitch_deg", "cam_r_px4_yaw_deg",
    "common_optical_roll_error_deg", "relative_optical_roll_error_deg",
    "common_optical_pitch_error_deg", "relative_optical_pitch_error_deg",
    "common_optical_yaw_error_deg", "relative_optical_yaw_error_deg",
    "effective_left_optical_roll_error_deg",
    "effective_left_optical_pitch_error_deg",
    "effective_left_optical_yaw_error_deg",
    "effective_right_optical_roll_error_deg",
    "effective_right_optical_pitch_error_deg",
    "effective_right_optical_yaw_error_deg", "nominal_pose_computation_status",
    "nominal_pose_forward_m", "nominal_pose_right_m", "nominal_pose_down_m", "nominal_pose_range_m",
    "nominal_pose_ray_gap_m", "nominal_pose_intersection_angle_deg", "nominal_pose_left_depth_m",
    "nominal_pose_right_depth_m", "nominal_pose_geometry_rank", "nominal_pose_condition_number",
    "corrected_computation_status", "corrected_forward_m", "corrected_right_m",
    "corrected_down_m", "corrected_range_m", "corrected_ray_gap_m",
    "corrected_intersection_angle_deg", "corrected_left_depth_m",
    "corrected_right_depth_m", "corrected_geometry_rank",
    "corrected_condition_number", "attitude_ray_computation_status",
    "uam_raw_gps_reference_computation_status",
    "uam_raw_gps_reference_forward_m", "uam_raw_gps_reference_right_m",
    "uam_raw_gps_reference_down_m", "uam_raw_gps_reference_range_m",
    "uam_raw_gps_reference_world_enu_x_m",
    "uam_raw_gps_reference_world_enu_y_m",
    "uam_raw_gps_reference_world_enu_z_m",
    "uam_raw_gps_reference_left_timestamp_us",
    "uam_raw_gps_reference_timestamp_us",
    "uam_raw_gps_reference_left_age_ms",
    "uam_raw_gps_reference_uam_age_ms",
    "uam_raw_gps_reference_pair_skew_ms",
    "uam_raw_gps_reference_left_eph_m",
    "uam_raw_gps_reference_uam_eph_m",
    "attitude_ray_failure_reason", "attitude_ray_forward_m",
    "attitude_ray_right_m", "attitude_ray_down_m", "attitude_ray_range_m",
    "attitude_ray_gap_m", "attitude_ray_intersection_angle_deg",
    "attitude_ray_delta_forward_m", "attitude_ray_delta_right_m",
    "attitude_ray_delta_down_m", "attitude_ray_delta_range_m",
    "attitude_ray_position_difference_m", "operational_ray_computation_status",
    "operational_ray_failure_reason", "operational_ray_source", "operational_ray_baseline_source",
    "operational_ray_gps_baseline_source", "operational_ray_forward_m", "operational_ray_right_m",
    "operational_ray_down_m", "operational_ray_range_m", "operational_ray_gap_m",
    "operational_ray_intersection_angle_deg", "operational_ray_delta_forward_m",
    "operational_ray_delta_right_m", "operational_ray_delta_down_m",
    "operational_ray_delta_range_m", "operational_ray_position_difference_m",
    "operational_ray_gps_baseline_world_enu_x_m",
    "operational_ray_gps_baseline_world_enu_y_m",
    "operational_ray_gps_baseline_world_enu_z_m",
    "operational_ray_gps_baseline_length_m",
    "operational_ray_gps_baseline_delta_world_enu_x_m",
    "operational_ray_gps_baseline_delta_world_enu_y_m",
    "operational_ray_gps_baseline_delta_world_enu_z_m",
    "operational_ray_gps_baseline_delta_norm_m", "gt_source", "gt_source_entity",
    "gt_reference_name", "gt_reference_definition", "gt_fallback_used",
    "gt_forward_m", "gt_right_m", "gt_down_m", "gt_range_m",
    "nominal_pose_position_error_m", "nominal_pose_forward_error_m", "nominal_pose_right_error_m",
    "nominal_pose_down_error_m", "nominal_pose_range_error_m", "corrected_position_error_m",
    "corrected_forward_error_m", "corrected_right_error_m",
    "corrected_down_error_m", "corrected_range_error_m",
    "ideal_position_error_m", "ideal_forward_error_m", "ideal_right_error_m",
    "ideal_down_error_m", "ideal_range_error_m",
    "measurement_queue_depth", "measurement_queue_depth_max",
    "measurement_queue_capacity", "measurement_queue_overflow_count",
    "measurement_overrun_invalid", "measurement_rejected_after_overrun_count",
    "measurement_accepted_count", "measurement_processed_count",
    "accepted_pair_discard_count", "measurement_processing_error_count",
    "gt_evaluation_success_count", "gt_evaluation_error_count",
    "research_record_queue_depth", "research_record_queue_depth_max",
    "research_record_queue_capacity", "research_recording_overrun_invalid",
    "research_recording_error_count", "output_frame", "output_axes",
    "position_unit", "pixel_unit", "angle_unit", "timestamp_unit",
    "correction_source", *DIAGNOSTIC_RECORD_EXTENSION_COLUMNS,
    "px4_attitude_model_version", "px4_attitude_reference_source",
    "px4_attitude_frame", "px4_attitude_composition",
    "px4_attitude_quaternion_order", "legacy_px4_absolute_attitude_used",
)

IDENTITY_FIELDS = (
    "schema_version", "session_id", "run_id", "attempt_id", "case_id",
    "effective_config_sha256", "record_sequence", "record_type", "sample_id",
    "pair_id", "epoch", "left_stamp_ns", "right_stamp_ns",
    "pair_midpoint_stamp_ns", "pair_delta_ns", "pairing_mode",
    "terminal_status", "terminal_reason", "measurement_status",
    "evaluation_status", "output_frame",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    encoded = json.dumps(
        dict(payload), ensure_ascii=False, sort_keys=True, allow_nan=False, indent=2
    ) + "\n"
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        handle.write(encoded)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def discover_repository_root(start: Path) -> Path:
    resolved = start.resolve()
    for candidate in (resolved, *resolved.parents):
        if (candidate / ".git").exists():
            return candidate
    raise ValueError("recording enabled but repository root cannot be verified")


def validate_recording_configuration(
    *,
    output_dir: str,
    run_id: str,
    case_id: str,
    repository: str,
    branch: str,
    commit_sha: str,
    git_status: str,
    gazebo_version: str,
    px4_version: str,
    queue_capacity: int,
    repository_root: Path,
) -> Path:
    values = {
        "run_id": run_id,
        "case_id": case_id,
        "repository": repository,
        "branch": branch,
        "gazebo_version": gazebo_version,
        "px4_version": px4_version,
    }
    missing = [name for name, value in values.items() if not str(value).strip()]
    if missing:
        raise ValueError("recording provenance is missing: " + ", ".join(missing))
    sha = str(commit_sha)
    if len(sha) != 40 or any(char not in "0123456789abcdef" for char in sha):
        raise ValueError("research_commit_sha must be 40 lowercase hexadecimal characters")
    if git_status not in {"clean", "dirty"}:
        raise ValueError("research_git_status must be clean or dirty")
    if int(queue_capacity) <= 0:
        raise ValueError("research_record_queue_capacity must be positive")
    candidate = Path(str(output_dir))
    if not candidate.is_absolute():
        raise ValueError("research_output_dir must be absolute")
    resolved_output = candidate.resolve(strict=False)
    resolved_repository = Path(repository_root).resolve(strict=True)
    recordings_root = resolved_repository / "Simulation" / "recordings"
    if resolved_repository.name == "Simulation":
        recordings_root = resolved_repository / "recordings"
    in_allowed_recordings = (
        resolved_output == recordings_root
        or recordings_root in resolved_output.parents
    )
    if (resolved_output == resolved_repository or resolved_repository in resolved_output.parents) \
            and not in_allowed_recordings:
        raise ValueError(
            "research_output_dir must be outside the repository or under apps/Simulation/recordings"
        )
    if candidate.exists() and not candidate.is_dir():
        raise ValueError("research_output_dir exists and is not a directory")
    if candidate.exists() and any(candidate.iterdir()):
        raise ValueError("research_output_dir must be new or empty")
    return resolved_output


def empty_pair_record() -> dict[str, Any]:
    return {name: None for name in PAIR_RECORD_COLUMNS}


def _validate_scalar(value: Any, field: str) -> None:
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float) and math.isfinite(value):
        return
    raise ValueError(f"{field} must be a finite flat JSON scalar or null")


def validate_pair_record(record: Mapping[str, Any]) -> dict[str, Any]:
    if set(record.keys()) == set(PAIR_RECORD_COLUMNS):
        normalized = {name: record[name] for name in PAIR_RECORD_COLUMNS}
    else:
        missing = [name for name in PAIR_RECORD_COLUMNS if name not in record]
        extra = [name for name in record if name not in PAIR_RECORD_COLUMNS]
        raise ValueError(f"pair record columns differ: missing={missing}, extra={extra}")
    for field, value in normalized.items():
        _validate_scalar(value, field)
    for field in IDENTITY_FIELDS:
        if normalized[field] is None or normalized[field] == "":
            raise ValueError(f"mandatory pair identity field is missing: {field}")
    if normalized["schema_version"] != RESEARCH_SCHEMA_VERSION:
        raise ValueError("pair record schema version mismatch")
    if normalized["record_type"] != "pair_terminal":
        raise ValueError("record_type must be pair_terminal")
    if len(str(normalized["effective_config_sha256"])) != 64 or any(
        character not in "0123456789abcdef"
        for character in str(normalized["effective_config_sha256"])
    ):
        raise ValueError("effective_config_sha256 must contain 64 lowercase hex characters")
    for field in (
        "pair_id", "epoch", "left_stamp_ns", "right_stamp_ns",
        "pair_midpoint_stamp_ns", "pair_delta_ns", "record_sequence",
    ):
        if isinstance(normalized[field], bool) or not isinstance(normalized[field], int):
            raise ValueError(f"{field} must be an integer")
    json.dumps(normalized, ensure_ascii=False, sort_keys=True, allow_nan=False)
    return normalized


class ResearchRecorder:
    """Bounded nonblocking producer with one append-only writer thread."""

    def __init__(
        self,
        *,
        output_dir: Path,
        run_metadata: Mapping[str, Any],
        queue_capacity: int,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.queue_capacity = int(queue_capacity)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.incomplete_path = self.output_dir / "RUN_INCOMPLETE"
        self.metadata_path = self.output_dir / "run_metadata.json"
        self.jsonl_path = self.output_dir / "pair_records.jsonl"
        self.csv_path = self.output_dir / "pair_records.csv"
        self.summary_path = self.output_dir / "run_summary.json"
        self.valid_marker_path = self.output_dir / "RUN_FINALIZED_VALID"
        self.invalid_marker_path = self.output_dir / "RUN_FINALIZED_INVALID"
        self.incomplete_path.touch(exist_ok=False)
        metadata = dict(run_metadata)
        metadata["schema_version"] = RESEARCH_SCHEMA_VERSION
        metadata.setdefault("created_utc", utc_now())
        metadata.setdefault("hostname", socket.gethostname())
        metadata.setdefault("platform", platform.platform())
        metadata.setdefault("python_version", sys.version.split()[0])
        atomic_write_json(self.metadata_path, metadata)
        self._queue: queue.Queue[dict[str, Any] | object] = queue.Queue(
            maxsize=self.queue_capacity
        )
        self._sentinel = object()
        self._sequence_lock = threading.Lock()
        self._next_sequence = 1
        self._closed = False
        self._final_summary: dict[str, Any] | None = None
        self.record_count = 0
        self.queue_depth_max = 0
        self.overrun_invalid = False
        self.error_count = 0
        self.first_error_type = ""
        self.first_error_message = ""
        self._jsonl_handle = self.jsonl_path.open("x", encoding="utf-8", newline="")
        self._thread = threading.Thread(
            target=self._writer_loop,
            name="simulation-research-recorder",
            daemon=False,
        )
        self._thread.start()

    @property
    def thread(self) -> threading.Thread:
        return self._thread

    @property
    def queue_depth(self) -> int:
        return self._queue.qsize()

    def _mark_error(self, exc: BaseException) -> None:
        self.error_count += 1
        if not self.first_error_type:
            self.first_error_type = type(exc).__name__
            self.first_error_message = str(exc)

    def submit(self, record: Mapping[str, Any]) -> bool:
        if self._closed:
            self._mark_error(RuntimeError("research recorder is closed"))
            return False
        try:
            with self._sequence_lock:
                prepared = dict(record)
                prepared["record_sequence"] = self._next_sequence
                if set(prepared) != set(PAIR_RECORD_COLUMNS):
                    raise ValueError("pair record does not use the fixed research schema")
                for field in IDENTITY_FIELDS:
                    if prepared[field] is None or prepared[field] == "":
                        raise ValueError(
                            f"mandatory pair identity field is missing: {field}"
                        )
                self._queue.put_nowait(prepared)
                self._next_sequence += 1
                self.queue_depth_max = max(self.queue_depth_max, self._queue.qsize())
            return True
        except queue.Full:
            self.overrun_invalid = True
            self._mark_error(RuntimeError("research recorder queue overflow"))
            return False
        except Exception as exc:
            self._mark_error(exc)
            return False

    def _writer_loop(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is self._sentinel:
                    return
                item = validate_pair_record(item)
                encoded = json.dumps(
                    item, ensure_ascii=False, sort_keys=True, allow_nan=False
                )
                self._jsonl_handle.write(encoded + "\n")
                self._jsonl_handle.flush()
                self.record_count += 1
            except Exception as exc:
                self._mark_error(exc)
            finally:
                self._queue.task_done()

    def _read_and_validate_jsonl(self) -> list[dict[str, Any]]:
        records: list[dict[str, Any]] = []
        keys: set[tuple[int, int]] = set()
        with self.jsonl_path.open("r", encoding="utf-8") as handle:
            for expected_sequence, line in enumerate(handle, start=1):
                if not line.endswith("\n"):
                    raise ValueError("JSONL final line is not newline terminated")
                record = validate_pair_record(json.loads(line))
                if record["record_sequence"] != expected_sequence:
                    raise ValueError("record_sequence is not contiguous")
                key = (record["epoch"], record["pair_id"])
                if key in keys:
                    raise ValueError(f"duplicate pair key: {key}")
                keys.add(key)
                records.append(record)
        return records

    def _export_csv(self, records: list[dict[str, Any]]) -> None:
        temporary = self.csv_path.with_name(self.csv_path.name + ".tmp")
        with temporary.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=PAIR_RECORD_COLUMNS)
            writer.writeheader()
            for record in records:
                row = {
                    key: "" if value is None else (
                        "true" if value is True else "false" if value is False else value
                    )
                    for key, value in record.items()
                }
                writer.writerow(row)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, self.csv_path)

    def _validate_csv(self) -> int:
        with self.csv_path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != PAIR_RECORD_COLUMNS:
                raise ValueError("CSV columns differ from the fixed research schema")
            return sum(1 for _row in reader)

    def close(self, final_state: Mapping[str, Any]) -> dict[str, Any]:
        if self._closed:
            return dict(self._final_summary or {})
        self._closed = True
        self._queue.join()
        self._queue.put(self._sentinel)
        self._thread.join(timeout=10.0)
        if self._thread.is_alive():
            self._mark_error(RuntimeError("research writer thread did not terminate"))
        self._jsonl_handle.flush()
        try:
            os.fsync(self._jsonl_handle.fileno())
        except OSError as exc:
            self._mark_error(exc)
        self._jsonl_handle.close()

        records: list[dict[str, Any]] = []
        csv_row_count = 0
        validation_error = None
        try:
            records = self._read_and_validate_jsonl()
            self._export_csv(records)
            csv_row_count = self._validate_csv()
            if csv_row_count != len(records):
                raise ValueError("JSONL and CSV row counts differ")
        except Exception as exc:
            validation_error = exc
            self._mark_error(exc)

        state = dict(final_state)
        accepted = int(state.get("accepted", 0))
        processed = int(state.get("processed", 0))
        discarded = int(state.get("discarded", 0))
        terminal_count = len(records)
        eligible_count = sum(bool(item["sample_eligible"]) for item in records)
        unique_count = len({(item["epoch"], item["pair_id"]) for item in records})
        accounting = {
            "accepted_equals_processed_plus_discarded": accepted == processed + discarded,
            "terminal_records_equal_processed_plus_discarded": terminal_count == processed + discarded,
            "terminal_records_equal_accepted": terminal_count == accepted,
            "jsonl_equals_csv_rows": (
                validation_error is None and csv_row_count == terminal_count
            ),
            "unique_pair_keys_equal_records": unique_count == terminal_count,
        }
        invalid_reasons = list(state.pop("invalid_reasons", ()))
        conditions = (
            (self.overrun_invalid, "research_record_queue_overflow"),
            (self.error_count > 0, "research_recording_error"),
            (bool(state.get("measurement_overrun_invalid")), "measurement_queue_overrun"),
            (int(state.get("measurement_processing_error_count", 0)) > 0, "measurement_processing_error"),
            (int(state.get("gt_evaluation_error_count", 0)) > 0, "gt_evaluation_error"),
            (state.get("shutdown_drain_status") != "SUCCESS", "shutdown_drain_incomplete"),
            (int(state.get("simulation_reset_count", 0)) > 0, "simulation_reset"),
            (int(state.get("remaining_measurement_queue", 0)) > 0, "measurement_queue_not_empty"),
            (int(state.get("remaining_gt_queue", 0)) > 0, "gt_queue_not_empty"),
            (state.get("inflight_pair") is not None, "inflight_pair_remaining"),
        )
        invalid_reasons.extend(reason for active, reason in conditions if active)
        invalid_reasons.extend(name for name, passed in accounting.items() if not passed)
        invalid_reasons = list(dict.fromkeys(invalid_reasons))
        hashes: dict[str, str | None] = {
            "pair_records_jsonl_sha256": None,
            "pair_records_csv_sha256": None,
            "run_metadata_sha256": None,
        }
        try:
            hashes["pair_records_jsonl_sha256"] = sha256_file(self.jsonl_path)
            if self.csv_path.exists():
                hashes["pair_records_csv_sha256"] = sha256_file(self.csv_path)
            hashes["run_metadata_sha256"] = sha256_file(self.metadata_path)
        except Exception as exc:
            self._mark_error(exc)
            invalid_reasons.append("artifact_hash_failure")
        if self.error_count and "research_recording_error" not in invalid_reasons:
            invalid_reasons.append("research_recording_error")

        run_valid = not invalid_reasons
        summary = {
            "schema_version": RESEARCH_SCHEMA_VERSION,
            "run_id": state.pop("run_id"),
            "case_id": state.pop("case_id"),
            "finalized_utc": utc_now(),
            "finalization_status": "VALID" if run_valid else "INVALID",
            "run_valid": run_valid,
            "invalid_reasons": invalid_reasons,
            **state,
            "terminal_record_count": terminal_count,
            "eligible_sample_count": eligible_count,
            "research_record_queue_depth_max": self.queue_depth_max,
            "research_record_queue_capacity": self.queue_capacity,
            "research_recording_overrun_invalid": self.overrun_invalid,
            "research_recording_error_count": self.error_count,
            "research_recording_first_error_type": self.first_error_type,
            "research_recording_first_error_message": self.first_error_message,
            "jsonl_line_count": terminal_count,
            "csv_data_row_count": csv_row_count,
            "unique_pair_key_count": unique_count,
            **hashes,
            **accounting,
        }
        atomic_write_json(self.summary_path, summary)
        marker = self.valid_marker_path if run_valid else self.invalid_marker_path
        marker.touch(exist_ok=False)
        self.incomplete_path.unlink()
        self._final_summary = dict(summary)
        return dict(summary)


__all__ = [
    "PAIR_RECORD_COLUMNS",
    "RESEARCH_SCHEMA_VERSION",
    "ResearchRecorder",
    "atomic_write_json",
    "discover_repository_root",
    "empty_pair_record",
    "sha256_file",
    "validate_pair_record",
    "validate_recording_configuration",
]
