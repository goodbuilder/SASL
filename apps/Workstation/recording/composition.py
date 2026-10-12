"""Compose recording owners and bind their declared collaborators once."""
from __future__ import annotations
from dataclasses import dataclass
from .binding import RecordingPanelBinding
from .state import RecordingState
from . import ports
from .video_segments import VideoSegments
from .row_collections import RowCollections
from .presentation import Presentation
from .session_start import SessionStart
from .session_stop import SessionStop
from .recovery import Recovery
from .ulog import Ulog
from .photo_capture import PhotoCapture
from .photo_records import PhotoRecords
from .session_save import SessionSave
from .metadata import Metadata
from .measurement_rows import MeasurementRows
from .native_submission import NativeSubmission
from .native_completion import NativeCompletion
from .native_retry import NativeRetry
from .primary_persistence import PrimaryPersistence
from .history import History
from .formatting import Formatting
from .timeline import Timeline
from .timing_fields import TimingFields
from .geometry_fields import GeometryFields
from .calibration_fields import CalibrationFields


@dataclass
class RecordingOwners:
    video_segments: VideoSegments
    row_collections: RowCollections
    presentation: Presentation
    session_start: SessionStart
    session_stop: SessionStop
    recovery: Recovery
    ulog: Ulog
    photo_capture: PhotoCapture
    photo_records: PhotoRecords
    session_save: SessionSave
    metadata: Metadata
    measurement_rows: MeasurementRows
    native_submission: NativeSubmission
    native_completion: NativeCompletion
    native_retry: NativeRetry
    primary_persistence: PrimaryPersistence
    history: History
    formatting: Formatting
    timeline: Timeline
    timing_fields: TimingFields
    geometry_fields: GeometryFields
    calibration_fields: CalibrationFields


def compose_recording(binding: RecordingPanelBinding, state: RecordingState) -> RecordingOwners:
    video_segments_boundary = ports.VideoSegmentsPort(
        _log=binding.field('_log'),
        _measurement_run_generation=binding.field('_measurement_run_generation'),
        _measurement_run_last_generation=binding.field('_measurement_run_last_generation'),
        allocate_measurement_artifact=binding.field('allocate_measurement_artifact'),
        allocate_workstation_data_artifact=binding.field('allocate_workstation_data_artifact'),
        append_workstation_data=binding.field('append_workstation_data'),
        measurement_source_transition=binding.field('measurement_source_transition'),
    )
    video_segments = VideoSegments(
        session=state.session,
        video_state=state.video_state,
        boundary=video_segments_boundary,
    )
    row_collections_boundary = ports.RowCollectionsPort(
        recording_store=binding.field('recording_store'),
        _log=binding.field('_log'),
    )
    row_collections = RowCollections(
        boundary=row_collections_boundary,
    )
    presentation_boundary = ports.PresentationPort(
        _distance_history_key=binding.field('_distance_history_key'),
        _log=binding.field('_log'),
        _measurement_session_hierarchy_enabled=binding.field('_measurement_session_hierarchy_enabled'),
        _refresh_operation_header=binding.field('_refresh_operation_header'),
        _session_recording_active=binding.field('_session_recording_active'),
        append_workstation_run_event=binding.field('append_workstation_run_event'),
        data_storage_snapshot=binding.field('data_storage_snapshot'),
        distance_history=binding.field('distance_history', writable=True),
        recording_sample_detail=binding.field('recording_sample_detail'),
        recording_sheet_tables=binding.field('recording_sheet_tables'),
        recording_sheet_tabs=binding.field('recording_sheet_tabs'),
        recording_status_label=binding.field('recording_status_label'),
    )
    presentation = Presentation(
        rows=state.rows,
        save_state=state.save_state,
        session=state.session,
        video_state=state.video_state,
        boundary=presentation_boundary,
    )
    session_start_boundary = ports.SessionStartPort(
        _active_optical_pose_profile_hash=binding.field('_active_optical_pose_profile_hash'),
        _cancel_pending_camera_start=binding.field('_cancel_pending_camera_start'),
        _format_video_recording_paths=binding.field('_format_video_recording_paths'),
        _last_distance_history_key=binding.field('_last_distance_history_key', writable=True),
        _last_frozen_measurement_pair=binding.field('_last_frozen_measurement_pair', writable=True),
        _last_measurement_attempt=binding.field('_last_measurement_attempt', writable=True),
        _last_measurement_attempt_key=binding.field('_last_measurement_attempt_key', writable=True),
        _log=binding.field('_log'),
        _measurement_attempt_counter=binding.field('_measurement_attempt_counter', writable=True),
        _measurement_attempt_generation_id=binding.field('_measurement_attempt_generation_id', writable=True),
        _measurement_method_result_counters=binding.field('_measurement_method_result_counters', writable=True),
        _measurement_run_generation=binding.field('_measurement_run_generation'),
        _method_evaluation_accepting=binding.field('_method_evaluation_accepting', writable=True),
        _method_evaluation_committed=binding.field('_method_evaluation_committed', writable=True),
        _method_evaluation_stop_retry_scheduled=binding.field('_method_evaluation_stop_retry_scheduled', writable=True),
        _method_evaluation_submitted=binding.field('_method_evaluation_submitted', writable=True),
        _prepare_video_recording_paths=binding.field('_prepare_video_recording_paths'),
        _refresh_operation_header=binding.field('_refresh_operation_header'),
        _session_recording_active=binding.field('_session_recording_active'),
        _shutdown_in_progress=binding.field('_shutdown_in_progress'),
        _start_camera_streams=binding.field('_start_camera_streams'),
        _sync_rtcm_evidence_context=binding.field('_sync_rtcm_evidence_context'),
        append_workstation_data=binding.field('append_workstation_data'),
        camera_connected=binding.field('camera_connected'),
        latest_extrinsics=binding.field('latest_extrinsics'),
        latest_gps=binding.field('latest_gps'),
        pixhawk_connection_thread=binding.field('pixhawk_connection_thread'),
        pixhawk_disconnection_thread=binding.field('pixhawk_disconnection_thread'),
        pixhawk_usb_bootstrap_thread=binding.field('pixhawk_usb_bootstrap_thread'),
        record_button=binding.field('record_button'),
        recording_status_label=binding.field('recording_status_label'),
        stop_yolo_streams=binding.field('stop_yolo_streams'),
        yolo_enabled=binding.field('yolo_enabled'),
    )
    transaction_fields = ports.RecordingStartFields(
        _recording_mode=binding.field('_recording_mode', writable=True, deletable=True),
        _manual_recording_active=binding.field('_manual_recording_active', writable=True, deletable=True),
        recording_session_id=binding.field('recording_session_id', writable=True, deletable=True),
        _pnp_session_prepared=binding.field('_pnp_session_prepared', writable=True, deletable=True),
        _session_save_finalized=binding.field('_session_save_finalized', writable=True, deletable=True),
        _measurement_attempt_generation_id=binding.field('_measurement_attempt_generation_id', writable=True, deletable=True),
        recording_session_started_unix_ns=binding.field('recording_session_started_unix_ns', writable=True, deletable=True),
        recording_session_started_monotonic_ns=binding.field('recording_session_started_monotonic_ns', writable=True, deletable=True),
        recording_session_anchor_uncertainty_ns=binding.field('recording_session_anchor_uncertainty_ns', writable=True, deletable=True),
        recording_session_started_wall_s=binding.field('recording_session_started_wall_s', writable=True, deletable=True),
        recording_session_started_monotonic_s=binding.field('recording_session_started_monotonic_s', writable=True, deletable=True),
        distance_recording_started_monotonic_s=binding.field('distance_recording_started_monotonic_s', writable=True, deletable=True),
        distance_recording_rows=binding.field('distance_recording_rows', writable=True, deletable=True),
        distance_recording_keys=binding.field('distance_recording_keys', writable=True, deletable=True),
        external_parameter_recording_rows=binding.field('external_parameter_recording_rows', writable=True, deletable=True),
        camera_recording_rows=binding.field('camera_recording_rows', writable=True, deletable=True),
        px4_recording_rows=binding.field('px4_recording_rows', writable=True, deletable=True),
        gnss_base_recording_rows=binding.field('gnss_base_recording_rows', writable=True, deletable=True),
        extrinsics_comparison_recording_rows=binding.field('extrinsics_comparison_recording_rows', writable=True, deletable=True),
        method_evaluation_bundle_rows=binding.field('method_evaluation_bundle_rows', writable=True, deletable=True),
        _recording_pair_row_index=binding.field('_recording_pair_row_index', writable=True, deletable=True),
        _recording_pending_native_request_rows=binding.field('_recording_pending_native_request_rows', writable=True, deletable=True),
        _recovered_session_metadata=binding.field('_recovered_session_metadata', writable=True, deletable=True),
        _recovered_session_output_dir=binding.field('_recovered_session_output_dir', writable=True, deletable=True),
        _recording_clock_sync_start_observation=binding.field('_recording_clock_sync_start_observation', writable=True, deletable=True),
        _recording_clock_sync_end_observation=binding.field('_recording_clock_sync_end_observation', writable=True, deletable=True),
        recording_reference_attitudes=binding.field('recording_reference_attitudes', writable=True, deletable=True),
        recording_reference_gps=binding.field('recording_reference_gps', writable=True, deletable=True),
        recording_origin_gps=binding.field('recording_origin_gps', writable=True, deletable=True),
        recording_nominal_intrinsics_snapshot=binding.field('recording_nominal_intrinsics_snapshot', writable=True, deletable=True),
        recording_nominal_optical_pose_profile_hash=binding.field('recording_nominal_optical_pose_profile_hash', writable=True, deletable=True),
        recording_reference_baseline_ned_m=binding.field('recording_reference_baseline_ned_m', writable=True, deletable=True),
        _gnss_baseline_center_recording_sequence=binding.field('_gnss_baseline_center_recording_sequence', writable=True, deletable=True),
        distance_recording_enabled=binding.field('distance_recording_enabled', writable=True, deletable=True),
        video_recording_enabled=binding.field('video_recording_enabled', writable=True, deletable=True),
        _last_distance_history_key=binding.field('_last_distance_history_key', writable=True, deletable=True),
        _measurement_attempt_counter=binding.field('_measurement_attempt_counter', writable=True, deletable=True),
        _last_measurement_attempt_key=binding.field('_last_measurement_attempt_key', writable=True, deletable=True),
        _last_measurement_attempt=binding.field('_last_measurement_attempt', writable=True, deletable=True),
        _last_frozen_measurement_pair=binding.field('_last_frozen_measurement_pair', writable=True, deletable=True),
        _method_evaluation_submitted=binding.field('_method_evaluation_submitted', writable=True, deletable=True),
        _method_evaluation_committed=binding.field('_method_evaluation_committed', writable=True, deletable=True),
        _method_evaluation_accepting=binding.field('_method_evaluation_accepting', writable=True, deletable=True),
        _method_evaluation_stop_retry_scheduled=binding.field('_method_evaluation_stop_retry_scheduled', writable=True, deletable=True),
        _native_frame_stop_retry_scheduled=binding.field('_native_frame_stop_retry_scheduled', writable=True, deletable=True),
        _recording_stop_retry_generation=binding.field('_recording_stop_retry_generation', writable=True, deletable=True),
        _recording_stop_retry_request=binding.field('_recording_stop_retry_request', writable=True, deletable=True),
        _measurement_method_result_counters=binding.field('_measurement_method_result_counters', writable=True, deletable=True),
        video_recording_paths=binding.field('video_recording_paths', writable=True, deletable=True),
        _video_recording_allocation_sizes=binding.field('_video_recording_allocation_sizes', writable=True, deletable=True),
        _video_recording_start_snapshot=binding.field('_video_recording_start_snapshot', writable=True, deletable=True),
        _measurement_video_segments=binding.field('_measurement_video_segments', writable=True, deletable=True),
        _last_recording_presence_state=binding.field('_last_recording_presence_state', writable=True, deletable=True),
    )
    session_start = SessionStart(
        capture_state=state.capture_state,
        ledger=state.ledger,
        native_state=state.native_state,
        references=state.references,
        rows=state.rows,
        save_state=state.save_state,
        session=state.session,
        transaction_fields=transaction_fields,
        video_state=state.video_state,
        boundary=session_start_boundary,
    )
    session_stop_boundary = ports.SessionStopPort(
        _camera_time_sync_snapshot=binding.field('_camera_time_sync_snapshot'),
        _format_video_recording_paths=binding.field('_format_video_recording_paths'),
        _log=binding.field('_log'),
        _measurement_session_hierarchy_enabled=binding.field('_measurement_session_hierarchy_enabled'),
        method_evaluation_pending_count=binding.field('method_evaluation_pending_count'),
        _method_evaluation_accepting=binding.field('_method_evaluation_accepting', writable=True),
        _method_evaluation_stop_retry_scheduled=binding.field('_method_evaluation_stop_retry_scheduled', writable=True),
        _refresh_operation_header=binding.field('_refresh_operation_header'),
        _resume_queued_measurement_start=binding.field('_resume_queued_measurement_start'),
        _session_recording_active=binding.field('_session_recording_active'),
        _start_camera_streams=binding.field('_start_camera_streams'),
        _sync_rtcm_evidence_context=binding.field('_sync_rtcm_evidence_context'),
        target_drone_stream_snapshot=binding.field('target_drone_stream_snapshot'),
        stop_target_drone_stream=binding.field('stop_target_drone_stream'),
        append_workstation_data=binding.field('append_workstation_data'),
        camera_connected=binding.field('camera_connected'),
        flush_workstation_data=binding.field('flush_workstation_data'),
        flush_workstation_run_evidence=binding.field('flush_workstation_run_evidence'),
        left_pane=binding.field('left_pane'),
        qobject_parent=binding.field('qobject_parent'),
        record_button=binding.field('record_button'),
        recording_status_label=binding.field('recording_status_label'),
        right_pane=binding.field('right_pane'),
        stream_backend_combo=binding.field('stream_backend_combo'),
        yolo_enabled=binding.field('yolo_enabled'),
    )
    session_stop = SessionStop(
        rows=state.rows,
        session=state.session,
        video_state=state.video_state,
        boundary=session_stop_boundary,
    )
    recovery_boundary = ports.RecoveryPort(
        _log=binding.field('_log'),
        _measurement_id=binding.field('_measurement_id', writable=True),
        _measurement_package=binding.field('_measurement_package', writable=True),
        _measurement_package_state=binding.field('_measurement_package_state', writable=True),
        _measurement_session=binding.field('_measurement_session', writable=True),
        _measurement_session_hierarchy_enabled=binding.field('_measurement_session_hierarchy_enabled'),
        _measurement_session_id=binding.field('_measurement_session_id', writable=True),
        _measurement_session_state=binding.field('_measurement_session_state', writable=True),
        app_root=binding.field('app_root'),
        record_button=binding.field('record_button'),
        recording_status_label=binding.field('recording_status_label'),
        save_history_button=binding.field('save_history_button'),
    )
    recovery = Recovery(
        capture_state=state.capture_state,
        rows=state.rows,
        save_state=state.save_state,
        session=state.session,
        boundary=recovery_boundary,
    )
    ulog_boundary = ports.UlogPort(
        _measurement_session_hierarchy_enabled=binding.field('_measurement_session_hierarchy_enabled'),
        _recording_session_dir=binding.field('_recording_session_dir'),
        append_workstation_data=binding.field('append_workstation_data'),
        append_workstation_run_event=binding.field('append_workstation_run_event'),
        import_workstation_session_artifact=binding.field('import_workstation_session_artifact'),
        measurement_storage_context=binding.field('measurement_storage_context'),
        register_workstation_session_artifact=binding.field('register_workstation_session_artifact'),
        ulog_automatic_sources=binding.field('ulog_automatic_sources'),
    )
    ulog = Ulog(
        session=state.session,
        ulog_state=state.ulog_state,
        boundary=ulog_boundary,
    )
    photo_capture_boundary = ports.PhotoCapturePort(
        _clear_oz_photo_capture_worker=binding.field('_clear_oz_photo_capture_worker'),
        _clear_pnp_photo_capture_worker=binding.field('_clear_pnp_photo_capture_worker'),
        _continue_measurement_package_finalization=binding.field('_continue_measurement_package_finalization'),
        _control_port_for_camera=binding.field('_control_port_for_camera'),
        _handle_oz_photo_capture_completed=binding.field('_handle_oz_photo_capture_completed'),
        _handle_oz_photo_capture_failed=binding.field('_handle_oz_photo_capture_failed'),
        _handle_pnp_photo_capture_completed=binding.field('_handle_pnp_photo_capture_completed'),
        _handle_pnp_photo_capture_failed=binding.field('_handle_pnp_photo_capture_failed'),
        _log=binding.field('_log'),
        _measurement_id=binding.field('_measurement_id'),
        _measurement_package_explicit_lifecycle=binding.field('_measurement_package_explicit_lifecycle'),
        _measurement_session_hierarchy_enabled=binding.field('_measurement_session_hierarchy_enabled'),
        _recording_session_dir=binding.field('_recording_session_dir'),
        append_workstation_data=binding.field('append_workstation_data'),
        begin_measurement=binding.field('begin_measurement'),
        begin_next_measurement=binding.field('begin_next_measurement'),
        ensure_workstation_run_evidence=binding.field('ensure_workstation_run_evidence'),
        measurement_package_snapshot=binding.field('measurement_package_snapshot'),
        measurement_storage_context=binding.field('measurement_storage_context'),
        qobject_parent=binding.field('qobject_parent'),
        recording_status_label=binding.field('recording_status_label'),
        settings=binding.field('settings'),
        zoom_hosts=binding.field('zoom_hosts'),
        zoom_timeout_spin=binding.field('zoom_timeout_spin'),
    )
    photo_capture = PhotoCapture(
        capture_state=state.capture_state,
        save_state=state.save_state,
        session=state.session,
        boundary=photo_capture_boundary,
    )
    photo_records_boundary = ports.PhotoRecordsPort(
        _log=binding.field('_log'),
        _measurement_id=binding.field('_measurement_id'),
        _measurement_session_hierarchy_enabled=binding.field('_measurement_session_hierarchy_enabled'),
        _recording_session_dir=binding.field('_recording_session_dir'),
        actual_visible_zoom=binding.field('actual_visible_zoom'),
        append_workstation_data=binding.field('append_workstation_data'),
        append_workstation_run_event=binding.field('append_workstation_run_event'),
        zoom_level_spin=binding.field('zoom_level_spin'),
    )
    photo_records = PhotoRecords(
        capture_state=state.capture_state,
        session=state.session,
        boundary=photo_records_boundary,
    )
    session_save_boundary = ports.SessionSavePort(
        _continue_measurement_package_finalization=binding.field('_continue_measurement_package_finalization'),
        _handle_session_save_failed=binding.field('_handle_session_save_failed'),
        _handle_session_save_thread_finished=binding.field('_handle_session_save_thread_finished'),
        _handle_session_saved=binding.field('_handle_session_saved'),
        _log=binding.field('_log'),
        _measurement_session_hierarchy_enabled=binding.field('_measurement_session_hierarchy_enabled'),
        _recording_session_dir=binding.field('_recording_session_dir'),
        _refresh_operation_header=binding.field('_refresh_operation_header'),
        flush_workstation_data=binding.field('flush_workstation_data'),
        qobject_parent=binding.field('qobject_parent'),
        record_button=binding.field('record_button'),
        recording_status_label=binding.field('recording_status_label'),
        save_history_button=binding.field('save_history_button'),
    )
    session_save = SessionSave(
        rows=state.rows,
        save_state=state.save_state,
        session=state.session,
        boundary=session_save_boundary,
    )
    metadata_boundary = ports.MetadataPort(
        _active_baseline_m=binding.field('_active_baseline_m'),
        _active_optical_pose_effective_from_pair_sequence=binding.field('_active_optical_pose_effective_from_pair_sequence'),
        _active_optical_pose_selection_id=binding.field('_active_optical_pose_selection_id'),
        _camera_time_sync_drift_metadata=binding.field('_camera_time_sync_drift_metadata'),
        _camera_time_sync_snapshot=binding.field('_camera_time_sync_snapshot'),
        _distance_mode_label=binding.field('_distance_mode_label'),
        _last_wiris_pc_utc_sync=binding.field('_last_wiris_pc_utc_sync'),
        _measurement_id=binding.field('_measurement_id'),
        _measurement_path_label=binding.field('_measurement_path_label'),
        _measurement_session=binding.field('_measurement_session'),
        _px4_mag_manager_has_data_source=binding.field('_px4_mag_manager_has_data_source'),
        _px4_mag_status_cache=binding.field('_px4_mag_status_cache'),
        _recording_session_dir=binding.field('_recording_session_dir'),
        _sync_px4_mag_manager_store=binding.field('_sync_px4_mag_manager_store'),
        _triangulation_extrinsics_source=binding.field('_triangulation_extrinsics_source'),
        _yolo_time_basis=binding.field('_yolo_time_basis'),
        camera_sync_reason=binding.field('camera_sync_reason'),
        camera_sync_status=binding.field('camera_sync_status'),
        decoder_combo=binding.field('decoder_combo'),
        gnss_fixed_base_profile=binding.field('gnss_fixed_base_profile'),
        gnss_operation_manager=binding.field('gnss_operation_manager'),
        left_url_edit=binding.field('left_url_edit'),
        px4_mag_managers=binding.field('px4_mag_managers'),
        right_url_edit=binding.field('right_url_edit'),
        rtsp_latency_spin=binding.field('rtsp_latency_spin'),
        rtsp_transport_combo=binding.field('rtsp_transport_combo'),
        stream_backend_combo=binding.field('stream_backend_combo'),
        workstation_measurement_methods_snapshot=binding.field('workstation_measurement_methods_snapshot'),
        workstation_optical_pose_snapshot=binding.field('workstation_optical_pose_snapshot'),
    )
    metadata = Metadata(
        capture_state=state.capture_state,
        native_state=state.native_state,
        references=state.references,
        rows=state.rows,
        session=state.session,
        video_state=state.video_state,
        boundary=metadata_boundary,
    )
    measurement_rows_boundary = ports.MeasurementRowsPort(
        _active_calibration_selection_mode=binding.field('_active_calibration_selection_mode'),
        _active_extrinsics_profile_path=binding.field('_active_extrinsics_profile_path'),
        _active_extrinsics_source_zoom_level=binding.field('_active_extrinsics_source_zoom_level'),
        _active_intrinsics_profile_path=binding.field('_active_intrinsics_profile_path'),
        _active_intrinsics_source_zoom_level=binding.field('_active_intrinsics_source_zoom_level'),
        _active_optical_pose_effective_from_pair_sequence=binding.field('_active_optical_pose_effective_from_pair_sequence'),
        _active_optical_pose_selection_id=binding.field('_active_optical_pose_selection_id'),
        _log=binding.field('_log'),
        _measurement_gnss_record_metadata=binding.field('_measurement_gnss_record_metadata'),
        _measurement_id=binding.field('_measurement_id'),
        _refresh_operation_header=binding.field('_refresh_operation_header'),
        active_intrinsics=binding.field('active_intrinsics'),
        zoom_level_spin=binding.field('zoom_level_spin'),
    )
    measurement_rows = MeasurementRows(
        ledger=state.ledger,
        references=state.references,
        rows=state.rows,
        session=state.session,
        video_state=state.video_state,
        boundary=measurement_rows_boundary,
    )
    native_submission_boundary = ports.NativeSubmissionPort(
        native_frames_path=binding.field('native_frames_path'),
        _handle_async_native_frame_result=binding.field('_handle_async_native_frame_result'),
        _log=binding.field('_log'),
        _measurement_session_hierarchy_enabled=binding.field('_measurement_session_hierarchy_enabled'),
        _recording_session_dir=binding.field('_recording_session_dir'),
        append_workstation_data=binding.field('append_workstation_data'),
        left_pane=binding.field('left_pane'),
        measurement_storage_context=binding.field('measurement_storage_context'),
        qobject_parent=binding.field('qobject_parent'),
        right_pane=binding.field('right_pane'),
    )
    native_submission = NativeSubmission(
        native_state=state.native_state,
        session=state.session,
        boundary=native_submission_boundary,
    )
    native_completion_boundary = ports.NativeCompletionPort(
        _log=binding.field('_log'),
        _measurement_session_hierarchy_enabled=binding.field('_measurement_session_hierarchy_enabled'),
        append_workstation_data=binding.field('append_workstation_data'),
    )
    native_completion = NativeCompletion(
        ledger=state.ledger,
        native_state=state.native_state,
        rows=state.rows,
        session=state.session,
        boundary=native_completion_boundary,
    )
    native_retry_boundary = ports.NativeRetryPort(
        _log=binding.field('_log'),
    )
    native_retry = NativeRetry(
        native_state=state.native_state,
        boundary=native_retry_boundary,
    )
    primary_persistence_boundary = ports.PrimaryPersistencePort(
        _log=binding.field('_log'),
        _refresh_operation_header=binding.field('_refresh_operation_header'),
        append_workstation_data=binding.field('append_workstation_data'),
    )
    primary_persistence = PrimaryPersistence(
        ledger=state.ledger,
        native_state=state.native_state,
        rows=state.rows,
        session=state.session,
        boundary=primary_persistence_boundary,
    )
    history_boundary = ports.HistoryPort(
        _last_distance_history_key=binding.field('_last_distance_history_key', writable=True),
        _measurement_id=binding.field('_measurement_id'),
        distance_history=binding.field('distance_history'),
        distance_plot=binding.field('distance_plot'),
        geometry_comparison_rows=binding.field('geometry_comparison_rows'),
    )
    history = History(
        ledger=state.ledger,
        rows=state.rows,
        session=state.session,
        video_state=state.video_state,
        boundary=history_boundary,
    )
    formatting = Formatting(
        session=state.session,
    )
    timeline_boundary = ports.TimelinePort(
        _distance_mode_label=binding.field('_distance_mode_label'),
        _measurement_id=binding.field('_measurement_id'),
        _pixhawk_gnss_source=binding.field('_pixhawk_gnss_source'),
        _selected_rover_gps_state=binding.field('_selected_rover_gps_state'),
        _triangulation_extrinsics_source=binding.field('_triangulation_extrinsics_source'),
        append_workstation_data=binding.field('append_workstation_data'),
        latest_gps=binding.field('latest_gps'),
        mavlink_link_provenance=binding.field('mavlink_link_provenance'),
    )
    timeline = Timeline(
        rows=state.rows,
        session=state.session,
        boundary=timeline_boundary,
    )
    timing_fields_boundary = ports.TimingFieldsPort(
        _camera_time_sync_metrics=binding.field('_camera_time_sync_metrics'),
        _yolo_pair_max_delta_s=binding.field('_yolo_pair_max_delta_s'),
        camera_sync_reason=binding.field('camera_sync_reason'),
        camera_sync_status=binding.field('camera_sync_status'),
    )
    timing_fields = TimingFields(
        boundary=timing_fields_boundary,
    )
    geometry_fields_boundary = ports.GeometryFieldsPort(
        _baseline_forward_geometry_from_camera_translation=binding.field('_baseline_forward_geometry_from_camera_translation'),
        latest_gnss_relative=binding.field('latest_gnss_relative'),
    )
    geometry_fields = GeometryFields(
        references=state.references,
        boundary=geometry_fields_boundary,
    )
    calibration_fields_boundary = ports.CalibrationFieldsPort(
        _active_calibration_selection_mode=binding.field('_active_calibration_selection_mode'),
        _active_extrinsics_profile_hash=binding.field('_active_extrinsics_profile_hash'),
        _active_extrinsics_profile_path=binding.field('_active_extrinsics_profile_path'),
        _active_extrinsics_source_zoom_level=binding.field('_active_extrinsics_source_zoom_level'),
        _active_intrinsics_profile_hash=binding.field('_active_intrinsics_profile_hash'),
        _active_intrinsics_profile_path=binding.field('_active_intrinsics_profile_path'),
        _active_intrinsics_source_zoom_level=binding.field('_active_intrinsics_source_zoom_level'),
        _checkerboard_translation_right_in_left_m=binding.field('_checkerboard_translation_right_in_left_m'),
        _intrinsics_storage_base_path=binding.field('_intrinsics_storage_base_path'),
        _intrinsics_storage_path=binding.field('_intrinsics_storage_path'),
        _workstation_setting=binding.field('_workstation_setting'),
        active_intrinsics=binding.field('active_intrinsics'),
        actual_visible_zoom=binding.field('actual_visible_zoom'),
        focal_spin=binding.field('focal_spin'),
        manual_focal_checkbox=binding.field('manual_focal_checkbox'),
        settings=binding.field('settings'),
        zoom_level_spin=binding.field('zoom_level_spin'),
    )
    calibration_fields = CalibrationFields(
        boundary=calibration_fields_boundary,
    )
    video_segments.bind_collaborators(
        presentation=presentation,
    )
    presentation.bind_collaborators(
        metadata=metadata,
        video_segments=video_segments,
    )
    session_start.bind_collaborators(
        measurement_rows=measurement_rows,
        metadata=metadata,
        photo_capture=photo_capture,
        presentation=presentation,
        row_collections=row_collections,
        session_stop=session_stop,
        video_segments=video_segments,
    )
    session_stop.bind_collaborators(
        native_submission=native_submission,
        photo_capture=photo_capture,
        presentation=presentation,
        session_save=session_save,
        ulog=ulog,
        video_segments=video_segments,
    )
    recovery.bind_collaborators(
        presentation=presentation,
    )
    ulog.bind_collaborators(
        photo_capture=photo_capture,
    )
    photo_capture.bind_collaborators(
        photo_records=photo_records,
        ulog=ulog,
    )
    photo_records.bind_collaborators(
        photo_capture=photo_capture,
    )
    session_save.bind_collaborators(
        metadata=metadata,
        session_start=session_start,
    )
    metadata.bind_collaborators(
        calibration_fields=calibration_fields,
        formatting=formatting,
        timeline=timeline,
    )
    measurement_rows.bind_collaborators(
        calibration_fields=calibration_fields,
        formatting=formatting,
        geometry_fields=geometry_fields,
        native_completion=native_completion,
        native_submission=native_submission,
        presentation=presentation,
        primary_persistence=primary_persistence,
        timing_fields=timing_fields,
    )
    native_submission.bind_collaborators(
        native_completion=native_completion,
        native_retry=native_retry,
    )
    native_completion.bind_collaborators(
        native_retry=native_retry,
        native_submission=native_submission,
        presentation=presentation,
        primary_persistence=primary_persistence,
    )
    native_retry.bind_collaborators(
        native_completion=native_completion,
        native_submission=native_submission,
    )
    primary_persistence.bind_collaborators(
        native_completion=native_completion,
        presentation=presentation,
    )
    history.bind_collaborators(
        calibration_fields=calibration_fields,
        formatting=formatting,
        geometry_fields=geometry_fields,
        measurement_rows=measurement_rows,
        primary_persistence=primary_persistence,
        timing_fields=timing_fields,
    )
    timeline.bind_collaborators(
        formatting=formatting,
        presentation=presentation,
        row_collections=row_collections,
        timing_fields=timing_fields,
    )
    timing_fields.bind_collaborators(
        formatting=formatting,
    )
    geometry_fields.bind_collaborators(
        calibration_fields=calibration_fields,
        formatting=formatting,
    )
    calibration_fields.bind_collaborators(
        formatting=formatting,
    )
    return RecordingOwners(
        video_segments=video_segments,
        row_collections=row_collections,
        presentation=presentation,
        session_start=session_start,
        session_stop=session_stop,
        recovery=recovery,
        ulog=ulog,
        photo_capture=photo_capture,
        photo_records=photo_records,
        session_save=session_save,
        metadata=metadata,
        measurement_rows=measurement_rows,
        native_submission=native_submission,
        native_completion=native_completion,
        native_retry=native_retry,
        primary_persistence=primary_persistence,
        history=history,
        formatting=formatting,
        timeline=timeline,
        timing_fields=timing_fields,
        geometry_fields=geometry_fields,
        calibration_fields=calibration_fields,
    )
