"""Stable recording schemas, canonical helpers, and compatibility identities."""
from __future__ import annotations
import math
import uuid
from collections.abc import Mapping
from dataclasses import dataclass
from Workstation.workstation_core.measurement_data_store import ROW_WRITE_ADMITTED

RECORDING_PREVIEW_LIMIT = 256

RECORDING_KEY_CACHE_LIMIT = 1024

NATIVE_COMPLETION_RETRY_INTERVAL_MS = 100

NATIVE_COMPLETION_SLOW_RETRY_INTERVAL_MS = 1000

NATIVE_COMPLETION_FAST_RETRY_LIMIT = 32

NATIVE_COMPLETION_TOTAL_RETRY_LIMIT = 64

NATIVE_COMPLETION_MAX_RETRY_AGE_S = 30.0

NATIVE_COMPLETION_RETRY_BATCH_LIMIT = 16

NATIVE_COMPLETION_PENDING_SOFT_LIMIT = 128

NATIVE_COMPLETION_TERMINAL_DROP_CACHE_LIMIT = 128

NATIVE_TERMINAL_FINGERPRINT_LIMIT = 4096

_RECORDING_START_MISSING = object()

@dataclass(frozen=True)
class PairRecordingCommitResult:
    inserted: bool
    enqueue_admitted: bool
    row_index: int
    admission_state: str = ROW_WRITE_ADMITTED
    admission_reason: str = ""
    row_lease: tuple[int, int] | None = None
    direct_persistence: bool = False

@dataclass(frozen=True)
class DistanceHistoryCommitResult:
    history_added: bool
    row_commit: PairRecordingCommitResult | None = None

@dataclass(frozen=True)
class _NativeRowOwner:
    runtime_generation: str
    rows: object
    row_index: int
    row_lease: tuple[int, int] | None = None

@dataclass(frozen=True)
class _NativeTerminalFingerprint:
    canonical_sha256: str
    canonical_json: str
    context: dict[str, object]

@dataclass
class _PendingNativeCompletion:
    request_id: str
    canonical_sha256: str
    payload: dict[str, object]
    owner: _NativeRowOwner | None
    event_recorded: bool = False
    first_seen_monotonic_s: float = 0.0
    attempts: int = 0
    next_retry_monotonic_s: float = 0.0
    slow_retry_logged: bool = False

@dataclass
class _PendingNativeConflict:
    request_id: str
    context: dict[str, object]
    authoritative_sha256: str
    conflicting_sha256: str
    conflicting_canonical_json: str
    first_seen_monotonic_s: float = 0.0
    attempts: int = 0
    next_retry_monotonic_s: float = 0.0
    slow_retry_logged: bool = False

_RECORDING_START_STATE_ATTRIBUTES = (
    "_recording_mode",
    "_manual_recording_active",
    "recording_session_id",
    "_pnp_session_prepared",
    "_session_save_finalized",
    "_measurement_attempt_generation_id",
    "recording_session_started_unix_ns",
    "recording_session_started_monotonic_ns",
    "recording_session_anchor_uncertainty_ns",
    "recording_session_started_wall_s",
    "recording_session_started_monotonic_s",
    "distance_recording_started_monotonic_s",
    "distance_recording_rows",
    "distance_recording_keys",
    "external_parameter_recording_rows",
    "camera_recording_rows",
    "px4_recording_rows",
    "gnss_base_recording_rows",
    "extrinsics_comparison_recording_rows",
    "method_evaluation_bundle_rows",
    "_recording_pair_row_index",
    "_recording_pending_native_request_rows",
    "_recovered_session_metadata",
    "_recovered_session_output_dir",
    "_recording_clock_sync_start_observation",
    "_recording_clock_sync_end_observation",
    "recording_reference_attitudes",
    "recording_reference_gps",
    "recording_origin_gps",
    "recording_nominal_intrinsics_snapshot",
    "recording_nominal_optical_pose_profile_hash",
    "recording_reference_baseline_ned_m",
    "_gnss_baseline_center_recording_sequence",
    "distance_recording_enabled",
    "video_recording_enabled",
    "_last_distance_history_key",
    "_measurement_attempt_counter",
    "_last_measurement_attempt_key",
    "_last_measurement_attempt",
    "_last_frozen_measurement_pair",
    "_method_evaluation_submitted",
    "_method_evaluation_committed",
    "_method_evaluation_accepting",
    "_method_evaluation_stop_retry_scheduled",
    "_native_frame_stop_retry_scheduled",
    "_recording_stop_retry_generation",
    "_recording_stop_retry_request",
    "_measurement_method_result_counters",
    "video_recording_paths",
    "_video_recording_allocation_sizes",
    "_video_recording_start_snapshot",
    "_measurement_video_segments",
    "_last_recording_presence_state",
)

def _nonfinite_external_reason(value: float) -> str:
    if math.isnan(value):
        return "nan"
    return "positive_infinity" if value > 0.0 else "negative_infinity"

def _normalize_external_timeline_value(
    value: object,
    *,
    path: str,
    nonfinite_reasons: dict[str, str],
) -> object:
    """Replace only non-finite numerics while retaining their exact source path."""

    if isinstance(value, float) and not math.isfinite(value):
        nonfinite_reasons[path] = _nonfinite_external_reason(value)
        return None
    if isinstance(value, Mapping):
        normalized: dict[str, object] = {}
        for key, item in value.items():
            name = str(key)
            child_path = f"{path}.{name}" if path else name
            normalized[name] = _normalize_external_timeline_value(
                item,
                path=child_path,
                nonfinite_reasons=nonfinite_reasons,
            )
        return normalized
    if isinstance(value, list):
        return [
            _normalize_external_timeline_value(
                item,
                path=f"{path}[{index}]",
                nonfinite_reasons=nonfinite_reasons,
            )
            for index, item in enumerate(value)
        ]
    if isinstance(value, tuple):
        return tuple(
            _normalize_external_timeline_value(
                item,
                path=f"{path}[{index}]",
                nonfinite_reasons=nonfinite_reasons,
            )
            for index, item in enumerate(value)
        )
    return value

def _normalize_external_timeline_values(
    values: Mapping[str, object],
) -> tuple[dict[str, object], dict[str, str]]:
    """Return a JSON-safe numeric payload and merged producer evidence."""

    detected_reasons: dict[str, str] = {}
    normalized = {
        str(key): _normalize_external_timeline_value(
            value,
            path=str(key),
            nonfinite_reasons=detected_reasons,
        )
        for key, value in values.items()
    }
    raw_fields = normalized.pop("nonfinite_source_fields", ())
    raw_reasons = normalized.pop("nonfinite_source_reasons", {})
    merged_reasons: dict[str, str] = {}
    if isinstance(raw_reasons, Mapping):
        for field, reason in raw_reasons.items():
            field_name = str(field).strip()
            if field_name:
                merged_reasons[field_name] = str(reason or "non_finite")
    if isinstance(raw_fields, (list, tuple)):
        for field in raw_fields:
            field_name = str(field).strip()
            if field_name:
                merged_reasons.setdefault(field_name, "non_finite")
    merged_reasons.update(detected_reasons)
    return normalized, merged_reasons

def new_measurement_attempt_generation_id(session_id: str = "") -> str:
    return f"{session_id or 'runtime'}-{uuid.uuid4().hex}"

RECORDING_CONTROLLER_METHODS = [
    "_refresh_recording_status_label",
    "allocate_measurement_video_segment",
    "record_measurement_video_segment_presence",
    "measurement_video_segments_snapshot",
    "video_recording_start_snapshot",
    "_set_video_recording_start_snapshot",
    "_recording_table_value",
    "_recording_preview_rows",
    "_recording_sheet_names",
    "_recording_sheet_kind",
    "_active_recording_sheet_name",
    "_append_recording_table_row",
    "_refresh_active_recording_sheet",
    "_fill_recording_sheet",
    "_refresh_recording_table",
    "_refresh_recording_sample_detail",
    "toggle_session_recording",
    "start_automatic_session_recording",
    "stop_automatic_session_recording",
    "_start_session_recording",
    "_schedule_legacy_recording_stop_retry",
    "_cancel_legacy_recording_stop_retry",
    "_retry_legacy_recording_stop",
    "_recover_retryable_recording_session",
    "_stop_session_recording",
    "capture_pnp_hr_photos",
    "capture_pnp_oz_photos",
    "import_hr_files",
    "_camera_photo_target_infos",
    "_wiris_ftp_worker_kwargs",
    "_append_pnp_photo_capture_record",
    "_append_oz_photo_capture_record",
    "_handle_pnp_photo_capture_completed",
    "_handle_pnp_photo_capture_failed",
    "_clear_pnp_photo_capture_worker",
    "_handle_oz_photo_capture_completed",
    "_handle_oz_photo_capture_failed",
    "_clear_oz_photo_capture_worker",
    "toggle_distance_recording",
    "toggle_video_recording",
    "_pair_recording_base_row",
    "_persist_native_detection_pair",
    "_persist_native_measurement_pair",
    "_handle_async_native_frame_result",
    "native_frame_persistence_pending_count",
    "_shutdown_native_frame_stores",
    "_upsert_pair_recording_row",
    "_record_unresolved_pair",
    "_record_blocked_measurement_pair",
    "_append_distance_history",
    "_commit_primary_distance_outcome",
    "_append_extrinsics_comparison_snapshot",
    "_distance_history_class_name",
    "_fmt_record_float",
    "_fmt_record_bool",
    "_recording_elapsed_s",
    "_session_utc_ns_for_monotonic_ns",
    "_recording_side",
    "_raw_timeline_row",
    "_record_gui_camera_frame",
    "_record_external_timeline_row",
    "_record_gnss_baseline_center",
    "_record_bbox_parameters",
    "_detection_utc_unix_s",
    "_unix_seconds_to_ns",
    "_record_measurement_sync_timestamp_parameters",
    "_record_frame_timestamp_parameters",
    "_record_live_telemetry_snapshot_parameters",
    "_record_reference_delta_parameters",
    "_record_current_intrinsic_parameters",
    "_record_intrinsic_parameters",
    "_record_current_checkerboard_extrinsics_parameters",
    "_record_checkerboard_extrinsics_parameters",
    "_record_external_parameters",
    "_record_extrinsic_state",
    "_record_gps_state",
    "_record_gnss_relative_state",
    "save_distance_history_csv",
    "_handle_session_saved",
    "_handle_session_save_failed",
    "_handle_session_save_thread_finished",
    "_wait_for_session_save",
    "_record_px4_mag_status_rows",
    "_record_px4_mag_status_snapshot",
    "_recording_camera_timing_diagnostics",
    "_recording_session_metadata",
    "workstation_ulog_snapshot",
    "import_workstation_ulog",
    "_remove_distance_history_point",
]
