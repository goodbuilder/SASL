"""Finite legacy recording entrypoints and named state aliases.

Remove an entry only after all panel/embedding callers use its named owner.
No owner uses this compatibility registry to call another owner.
"""
from __future__ import annotations
from functools import wraps
from .measurement_rows import MeasurementRows
from .native_completion import NativeCompletion
from .native_retry import NativeRetry
from .native_submission import NativeSubmission
from .photo_capture import PhotoCapture
from .photo_records import PhotoRecords
from .presentation import Presentation
from .recovery import Recovery
from .session_start import SessionStart

METHOD_OWNERS = {'_video_segment_registry': 'video_segments',
 '_begin_measurement_video_generation': 'video_segments',
 'measurement_video_segments_snapshot': 'video_segments',
 'allocate_measurement_video_segment': 'video_segments',
 'record_measurement_video_segment_presence': 'video_segments',
 '_handle_recording_spool_error': 'row_collections',
 '_new_recording_spool': 'row_collections',
 '_new_live_rows': 'row_collections',
 '_refresh_recording_status_label': 'presentation',
 'video_recording_start_snapshot': 'video_segments',
 '_set_video_recording_start_snapshot': 'video_segments',
 '_recording_table_value': 'presentation',
 '_recording_preview_rows': 'presentation',
 '_recording_sheet_names': 'presentation',
 '_recording_sheet_kind': 'presentation',
 '_active_recording_sheet_name': 'presentation',
 '_append_recording_table_row': 'presentation',
 '_refresh_active_recording_sheet': 'presentation',
 '_fill_recording_sheet': 'presentation',
 '_refresh_recording_table': 'presentation',
 '_refresh_recording_sample_detail': 'presentation',
 '_widget_text': 'presentation',
 '_recording_start_snapshot': 'session_start',
 '_recording_row_resources': 'session_start',
 '_release_failed_recording_start_rows': 'session_start',
 '_rollback_failed_recording_start': 'session_start',
 '_schedule_legacy_recording_stop_retry': 'session_stop',
 '_cancel_legacy_recording_stop_retry': 'session_stop',
 '_retry_legacy_recording_stop': 'session_stop',
 'toggle_session_recording': 'session_start',
 'start_automatic_session_recording': 'session_start',
 'stop_automatic_session_recording': 'session_stop',
 '_start_session_recording': 'session_start',
 '_start_session_recording_transaction_body': 'session_start',
 '_load_jsonl_records': 'recovery',
 '_next_capture_id': 'photo_capture',
 '_new_capture_id': 'photo_capture',
 '_measurement_artifact_stamp': 'photo_capture',
 '_recover_retryable_recording_session': 'recovery',
 '_stop_session_recording': 'session_stop',
 '_finalize_workstation_ulogs': 'ulog',
 '_register_session_owned_artifact': 'ulog',
 '_camera_photo_capture_active': 'photo_capture',
 '_hr_capture_purpose': 'photo_capture',
 '_prepare_camera_evidence_session': 'photo_capture',
 '_measurement_package_capture_allowed': 'photo_capture',
 '_prepare_pnp_evidence_session': 'photo_capture',
 '_prepare_oz_evidence_session': 'photo_capture',
 '_ensure_ulog_controller': 'ulog',
 'workstation_ulog_snapshot': 'ulog',
 'import_workstation_ulog': 'ulog',
 '_new_camera_request_context': 'photo_capture',
 '_new_hr_request_context': 'photo_capture',
 '_new_oz_request_context': 'photo_capture',
 '_camera_hr_request_delta': 'photo_capture',
 '_hr_capture_event_record': 'photo_records',
 '_oz_capture_event_record': 'photo_records',
 'capture_pnp_hr_photos': 'photo_capture',
 'capture_pnp_oz_photos': 'photo_capture',
 '_wiris_ftp_worker_kwargs': 'photo_capture',
 '_wiris_ftp_status_text': 'photo_capture',
 '_camera_photo_target_infos': 'photo_capture',
 '_append_pnp_photo_capture_record': 'photo_records',
 '_append_oz_photo_capture_record': 'photo_records',
 '_verify_ftp_capture_cameras': 'photo_capture',
 '_handle_pnp_photo_capture_completed': 'photo_capture',
 '_handle_pnp_photo_capture_failed': 'photo_capture',
 '_handle_oz_photo_capture_completed': 'photo_capture',
 '_handle_oz_photo_capture_failed': 'photo_capture',
 '_clear_pnp_photo_capture_worker': 'photo_capture',
 '_clear_oz_photo_capture_worker': 'photo_capture',
 '_hr_capture_path_component': 'photo_records',
 '_known_hr_purpose': 'photo_records',
 'import_hr_files': 'photo_records',
 'toggle_distance_recording': 'session_save',
 'toggle_video_recording': 'session_save',
 'save_distance_history_csv': 'session_save',
 '_handle_session_saved': 'session_save',
 '_handle_session_save_failed': 'session_save',
 '_handle_session_save_thread_finished': 'session_save',
 '_wait_for_session_save': 'session_save',
 '_record_px4_mag_status_rows': 'metadata',
 '_record_px4_mag_status_snapshot': 'metadata',
 '_recording_camera_timing_diagnostics': 'metadata',
 '_recording_optical_pose_session_data': 'metadata',
 '_recording_session_metadata': 'metadata',
 '_remove_distance_history_point': 'presentation',
 '_pair_recording_base_row': 'measurement_rows',
 '_snapshot_current_intrinsics': 'measurement_rows',
 '_record_dual_geometry_parameters': 'measurement_rows',
 '_persist_native_detection_pair': 'native_submission',
 '_native_frame_store_for_session': 'native_submission',
 '_native_evidence_writer_for_session': 'native_submission',
 '_persist_native_artifact_record': 'native_submission',
 '_native_events_path': 'native_submission',
 '_native_completion_fingerprint': 'native_completion',
 '_remember_native_terminal_fingerprint': 'native_completion',
 '_native_result_store': 'native_submission',
 '_acknowledge_native_result': 'native_submission',
 '_native_completion_retry_delay_s': 'native_submission',
 '_log_native_completion_slow_retry': 'native_retry',
 '_native_retry_is_terminal': 'native_retry',
 '_remember_native_completion_terminal_drop': 'native_retry',
 '_terminal_drop_native_completion': 'native_retry',
 '_terminal_drop_native_conflict': 'native_retry',
 '_defer_native_completion_retry': 'native_retry',
 '_finish_native_completion': 'native_completion',
 '_finish_native_completion_from_admitted_row': 'native_completion',
 '_attempt_native_conflict_diagnostic': 'native_retry',
 '_record_native_completion_conflict': 'native_retry',
 '_schedule_native_completion_retry': 'native_retry',
 '_retry_pending_native_completions': 'native_retry',
 '_append_native_frame_completion_event': 'native_completion',
 '_handle_async_native_frame_result': 'native_completion',
 'native_frame_persistence_pending_count': 'native_submission',
 '_shutdown_native_frame_stores': 'native_submission',
 '_persist_native_measurement_pair': 'native_submission',
 '_measurement_result_evidence': 'measurement_rows',
 '_track_pending_native_row': 'native_completion',
 '_append_measurement_row_direct': 'primary_persistence',
 '_persist_compact_primary_measurement': 'primary_persistence',
 '_release_primary_retry_ownership': 'primary_persistence',
 '_remember_pending_primary_row': 'primary_persistence',
 '_remember_compact_primary_persisted': 'primary_persistence',
 '_remember_native_primary_persisted': 'primary_persistence',
 '_consume_native_primary_persisted': 'primary_persistence',
 '_pending_primary_retry_row': 'primary_persistence',
 '_upsert_pair_recording_row': 'primary_persistence',
 '_record_unresolved_pair': 'measurement_rows',
 '_record_blocked_measurement_pair': 'measurement_rows',
 '_append_distance_history': 'history',
 '_compact_primary_retry_row': 'measurement_rows',
 '_commit_primary_distance_outcome': 'history',
 '_commit_distance_history': 'history',
 '_append_extrinsics_comparison_snapshot': 'history',
 '_distance_history_class_name': 'history',
 '_fmt_record_float': 'formatting',
 '_fmt_record_bool': 'formatting',
 '_recording_elapsed_s': 'formatting',
 '_session_utc_ns_for_monotonic_ns': 'formatting',
 '_recording_side': 'formatting',
 '_raw_timeline_row': 'timeline',
 '_record_gui_camera_frame': 'timeline',
 '_record_external_timeline_row': 'timeline',
 '_record_gnss_baseline_center': 'timeline',
 '_record_bbox_parameters': 'timing_fields',
 '_detection_utc_unix_s': 'timing_fields',
 '_unix_seconds_to_ns': 'timing_fields',
 '_record_measurement_sync_timestamp_parameters': 'timing_fields',
 '_record_frame_timestamp_parameters': 'timing_fields',
 '_record_live_telemetry_snapshot_parameters': 'timing_fields',
 '_record_reference_delta_parameters': 'geometry_fields',
 '_record_current_intrinsic_parameters': 'calibration_fields',
 '_record_intrinsic_parameters': 'calibration_fields',
 '_record_current_checkerboard_extrinsics_parameters': 'calibration_fields',
 '_record_checkerboard_extrinsics_parameters': 'calibration_fields',
 '_record_external_parameters': 'geometry_fields',
 '_record_extrinsic_state': 'geometry_fields',
 '_record_gps_state': 'geometry_fields',
 '_record_gnss_relative_state': 'geometry_fields'}
STATIC_METHODS = ('_acknowledge_native_result',
 '_camera_hr_request_delta',
 '_hr_capture_path_component',
 '_hr_capture_purpose',
 '_load_jsonl_records',
 '_measurement_result_evidence',
 '_native_completion_fingerprint',
 '_native_completion_retry_delay_s',
 '_native_retry_is_terminal',
 '_next_capture_id',
 '_recording_row_resources',
 '_verify_ftp_capture_cameras',
 '_widget_text')
QOBJECT_RECEIVERS = ('_clear_oz_photo_capture_worker',
 '_clear_pnp_photo_capture_worker',
 '_handle_async_native_frame_result',
 '_handle_oz_photo_capture_completed',
 '_handle_oz_photo_capture_failed',
 '_handle_pnp_photo_capture_completed',
 '_handle_pnp_photo_capture_failed',
 '_handle_session_save_failed',
 '_handle_session_save_thread_finished',
 '_handle_session_saved')
STATE_ALIASES = {'_pending_hr_requests': 'capture_state',
 '_pending_oz_requests': 'capture_state',
 '_deferred_native_frame_results': 'native_state',
 '_native_row_owners': 'native_state',
 '_native_terminal_fingerprints': 'native_state',
 '_native_terminal_fingerprint_order': 'native_state',
 '_pending_native_completions': 'native_state',
 '_pending_native_conflicts': 'native_state',
 '_native_conflicts_recorded': 'native_state',
 '_native_completion_terminal_drops': 'native_state',
 '_native_completion_terminal_drop_count': 'native_state',
 '_native_completion_terminal_drop_evictions': 'native_state',
 '_native_completion_retry_scheduled': 'native_state',
 '_pair_recording_row_leases': 'ledger',
 '_pending_primary_record_rows': 'ledger',
 '_primary_persistence_lock': 'ledger',
 '_compact_primary_persisted_keys': 'ledger',
 '_compact_primary_persisted_order': 'ledger',
 '_native_primary_persisted_keys': 'ledger',
 '_native_primary_persisted_order': 'ledger',
 '_primary_retry_required_keys': 'ledger'}


class _RecordingEntryPoint:
    def __init__(self, name: str, owner_name: str) -> None:
        self.name = name
        self.owner_name = owner_name

    def __get__(self, controller, owner_type=None):
        if controller is None:
            def unbound(instance, *args, **kwargs):
                return instance._call_recording(self.owner_name, self.name, *args, **kwargs)
            unbound.__name__ = self.name
            return unbound
        target = getattr(getattr(controller.owners, self.owner_name), self.name)
        @wraps(target)
        def delegated(*args, **kwargs):
            return controller._call_recording(self.owner_name, self.name, *args, **kwargs)
        return delegated

    def __set__(self, controller, callback) -> None:
        setattr(getattr(controller.owners, self.owner_name), self.name, callback)


def bind_recording_compatibility(controller_type: type) -> None:
    for name, owner in METHOD_OWNERS.items():
        if name not in STATIC_METHODS:
            setattr(controller_type, name, _RecordingEntryPoint(name, owner))
    for name, group in STATE_ALIASES.items():
        def read(controller, name=name, group=group):
            return getattr(getattr(controller.recording_state, group), name)
        def write(controller, value, name=name, group=group):
            setattr(getattr(controller.recording_state, group), name, value)
        setattr(controller_type, name, property(read, write))
    controller_type._acknowledge_native_result = staticmethod(NativeSubmission._acknowledge_native_result)
    controller_type._camera_hr_request_delta = staticmethod(PhotoCapture._camera_hr_request_delta)
    controller_type._hr_capture_path_component = staticmethod(PhotoRecords._hr_capture_path_component)
    controller_type._hr_capture_purpose = staticmethod(PhotoCapture._hr_capture_purpose)
    controller_type._load_jsonl_records = staticmethod(Recovery._load_jsonl_records)
    controller_type._measurement_result_evidence = staticmethod(MeasurementRows._measurement_result_evidence)
    controller_type._native_completion_fingerprint = staticmethod(NativeCompletion._native_completion_fingerprint)
    controller_type._native_completion_retry_delay_s = staticmethod(NativeSubmission._native_completion_retry_delay_s)
    controller_type._native_retry_is_terminal = staticmethod(NativeRetry._native_retry_is_terminal)
    controller_type._next_capture_id = staticmethod(PhotoCapture._next_capture_id)
    controller_type._recording_row_resources = staticmethod(SessionStart._recording_row_resources)
    controller_type._verify_ftp_capture_cameras = staticmethod(PhotoCapture._verify_ftp_capture_cameras)
    controller_type._widget_text = staticmethod(Presentation._widget_text)
