"""Finite recording boundary contracts.

Owners receive these named capabilities and their own state. Only binding.py
knows the application panel. No port forwards arbitrary attribute names.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable

@dataclass(frozen=True)
class FieldBinding:
    read: Callable[[], object]
    write: Callable[[object], None] | None = None
    delete: Callable[[], None] | None = None


def _field_property(name: str, *, writable: bool = False, deletable: bool = False) -> property:
    slot = "_binding_" + name
    return property(
        lambda owner: object.__getattribute__(owner, slot).read(),
        (lambda owner, value: object.__getattribute__(owner, slot).write(value)) if writable else None,
        (lambda owner: object.__getattribute__(owner, slot).delete()) if deletable else None,
    )

class VideoSegmentsPort:
    __slots__ = ('_binding__log', '_binding__measurement_run_generation', '_binding__measurement_run_last_generation', '_binding_allocate_measurement_artifact', '_binding_allocate_workstation_data_artifact', '_binding_append_workstation_data', '_binding_measurement_source_transition')

    def __init__(self, *,
        _log: FieldBinding,
        _measurement_run_generation: FieldBinding,
        _measurement_run_last_generation: FieldBinding,
        allocate_measurement_artifact: FieldBinding,
        allocate_workstation_data_artifact: FieldBinding,
        append_workstation_data: FieldBinding,
        measurement_source_transition: FieldBinding,
    ) -> None:
        self._binding__log = _log
        self._binding__measurement_run_generation = _measurement_run_generation
        self._binding__measurement_run_last_generation = _measurement_run_last_generation
        self._binding_allocate_measurement_artifact = allocate_measurement_artifact
        self._binding_allocate_workstation_data_artifact = allocate_workstation_data_artifact
        self._binding_append_workstation_data = append_workstation_data
        self._binding_measurement_source_transition = measurement_source_transition

    _log = _field_property('_log')
    _measurement_run_generation = _field_property('_measurement_run_generation')
    _measurement_run_last_generation = _field_property('_measurement_run_last_generation')
    allocate_measurement_artifact = _field_property('allocate_measurement_artifact')
    allocate_workstation_data_artifact = _field_property('allocate_workstation_data_artifact')
    append_workstation_data = _field_property('append_workstation_data')
    measurement_source_transition = _field_property('measurement_source_transition')


class RowCollectionsPort:
    __slots__ = ('_binding_recording_store', '_binding__log')

    def __init__(self, *,
        recording_store: FieldBinding,
        _log: FieldBinding,
    ) -> None:
        self._binding_recording_store = recording_store
        self._binding__log = _log

    recording_store = _field_property('recording_store')
    _log = _field_property('_log')


class PresentationPort:
    __slots__ = ('_binding__distance_history_key', '_binding__log', '_binding__measurement_session_hierarchy_enabled', '_binding__refresh_operation_header', '_binding__session_recording_active', '_binding_append_workstation_run_event', '_binding_data_storage_snapshot', '_binding_distance_history', '_binding_recording_sample_detail', '_binding_recording_sheet_tables', '_binding_recording_sheet_tabs', '_binding_recording_status_label')

    def __init__(self, *,
        _distance_history_key: FieldBinding,
        _log: FieldBinding,
        _measurement_session_hierarchy_enabled: FieldBinding,
        _refresh_operation_header: FieldBinding,
        _session_recording_active: FieldBinding,
        append_workstation_run_event: FieldBinding,
        data_storage_snapshot: FieldBinding,
        distance_history: FieldBinding,
        recording_sample_detail: FieldBinding,
        recording_sheet_tables: FieldBinding,
        recording_sheet_tabs: FieldBinding,
        recording_status_label: FieldBinding,
    ) -> None:
        self._binding__distance_history_key = _distance_history_key
        self._binding__log = _log
        self._binding__measurement_session_hierarchy_enabled = _measurement_session_hierarchy_enabled
        self._binding__refresh_operation_header = _refresh_operation_header
        self._binding__session_recording_active = _session_recording_active
        self._binding_append_workstation_run_event = append_workstation_run_event
        self._binding_data_storage_snapshot = data_storage_snapshot
        self._binding_distance_history = distance_history
        self._binding_recording_sample_detail = recording_sample_detail
        self._binding_recording_sheet_tables = recording_sheet_tables
        self._binding_recording_sheet_tabs = recording_sheet_tabs
        self._binding_recording_status_label = recording_status_label

    _distance_history_key = _field_property('_distance_history_key')
    _log = _field_property('_log')
    _measurement_session_hierarchy_enabled = _field_property('_measurement_session_hierarchy_enabled')
    _refresh_operation_header = _field_property('_refresh_operation_header')
    _session_recording_active = _field_property('_session_recording_active')
    append_workstation_run_event = _field_property('append_workstation_run_event')
    data_storage_snapshot = _field_property('data_storage_snapshot')
    distance_history = _field_property('distance_history', writable=True)
    recording_sample_detail = _field_property('recording_sample_detail')
    recording_sheet_tables = _field_property('recording_sheet_tables')
    recording_sheet_tabs = _field_property('recording_sheet_tabs')
    recording_status_label = _field_property('recording_status_label')


class SessionStartPort:
    __slots__ = ('_binding__active_optical_pose_profile_hash', '_binding__cancel_pending_camera_start', '_binding__format_video_recording_paths', '_binding__last_distance_history_key', '_binding__last_frozen_measurement_pair', '_binding__last_measurement_attempt', '_binding__last_measurement_attempt_key', '_binding__log', '_binding__measurement_attempt_counter', '_binding__measurement_attempt_generation_id', '_binding__measurement_method_result_counters', '_binding__measurement_run_generation', '_binding__method_evaluation_accepting', '_binding__method_evaluation_committed', '_binding__method_evaluation_stop_retry_scheduled', '_binding__method_evaluation_submitted', '_binding__prepare_video_recording_paths', '_binding__refresh_operation_header', '_binding__session_recording_active', '_binding__shutdown_in_progress', '_binding__start_camera_streams', '_binding__sync_rtcm_evidence_context', '_binding_append_workstation_data', '_binding_camera_connected', '_binding_latest_extrinsics', '_binding_latest_gps', '_binding_pixhawk_connection_thread', '_binding_pixhawk_disconnection_thread', '_binding_pixhawk_usb_bootstrap_thread', '_binding_record_button', '_binding_recording_status_label', '_binding_stop_yolo_streams', '_binding_yolo_enabled')

    def __init__(self, *,
        _active_optical_pose_profile_hash: FieldBinding,
        _cancel_pending_camera_start: FieldBinding,
        _format_video_recording_paths: FieldBinding,
        _last_distance_history_key: FieldBinding,
        _last_frozen_measurement_pair: FieldBinding,
        _last_measurement_attempt: FieldBinding,
        _last_measurement_attempt_key: FieldBinding,
        _log: FieldBinding,
        _measurement_attempt_counter: FieldBinding,
        _measurement_attempt_generation_id: FieldBinding,
        _measurement_method_result_counters: FieldBinding,
        _measurement_run_generation: FieldBinding,
        _method_evaluation_accepting: FieldBinding,
        _method_evaluation_committed: FieldBinding,
        _method_evaluation_stop_retry_scheduled: FieldBinding,
        _method_evaluation_submitted: FieldBinding,
        _prepare_video_recording_paths: FieldBinding,
        _refresh_operation_header: FieldBinding,
        _session_recording_active: FieldBinding,
        _shutdown_in_progress: FieldBinding,
        _start_camera_streams: FieldBinding,
        _sync_rtcm_evidence_context: FieldBinding,
        append_workstation_data: FieldBinding,
        camera_connected: FieldBinding,
        latest_extrinsics: FieldBinding,
        latest_gps: FieldBinding,
        pixhawk_connection_thread: FieldBinding,
        pixhawk_disconnection_thread: FieldBinding,
        pixhawk_usb_bootstrap_thread: FieldBinding,
        record_button: FieldBinding,
        recording_status_label: FieldBinding,
        stop_yolo_streams: FieldBinding,
        yolo_enabled: FieldBinding,
    ) -> None:
        self._binding__active_optical_pose_profile_hash = _active_optical_pose_profile_hash
        self._binding__cancel_pending_camera_start = _cancel_pending_camera_start
        self._binding__format_video_recording_paths = _format_video_recording_paths
        self._binding__last_distance_history_key = _last_distance_history_key
        self._binding__last_frozen_measurement_pair = _last_frozen_measurement_pair
        self._binding__last_measurement_attempt = _last_measurement_attempt
        self._binding__last_measurement_attempt_key = _last_measurement_attempt_key
        self._binding__log = _log
        self._binding__measurement_attempt_counter = _measurement_attempt_counter
        self._binding__measurement_attempt_generation_id = _measurement_attempt_generation_id
        self._binding__measurement_method_result_counters = _measurement_method_result_counters
        self._binding__measurement_run_generation = _measurement_run_generation
        self._binding__method_evaluation_accepting = _method_evaluation_accepting
        self._binding__method_evaluation_committed = _method_evaluation_committed
        self._binding__method_evaluation_stop_retry_scheduled = _method_evaluation_stop_retry_scheduled
        self._binding__method_evaluation_submitted = _method_evaluation_submitted
        self._binding__prepare_video_recording_paths = _prepare_video_recording_paths
        self._binding__refresh_operation_header = _refresh_operation_header
        self._binding__session_recording_active = _session_recording_active
        self._binding__shutdown_in_progress = _shutdown_in_progress
        self._binding__start_camera_streams = _start_camera_streams
        self._binding__sync_rtcm_evidence_context = _sync_rtcm_evidence_context
        self._binding_append_workstation_data = append_workstation_data
        self._binding_camera_connected = camera_connected
        self._binding_latest_extrinsics = latest_extrinsics
        self._binding_latest_gps = latest_gps
        self._binding_pixhawk_connection_thread = pixhawk_connection_thread
        self._binding_pixhawk_disconnection_thread = pixhawk_disconnection_thread
        self._binding_pixhawk_usb_bootstrap_thread = pixhawk_usb_bootstrap_thread
        self._binding_record_button = record_button
        self._binding_recording_status_label = recording_status_label
        self._binding_stop_yolo_streams = stop_yolo_streams
        self._binding_yolo_enabled = yolo_enabled

    _active_optical_pose_profile_hash = _field_property('_active_optical_pose_profile_hash')
    _cancel_pending_camera_start = _field_property('_cancel_pending_camera_start')
    _format_video_recording_paths = _field_property('_format_video_recording_paths')
    _last_distance_history_key = _field_property('_last_distance_history_key', writable=True)
    _last_frozen_measurement_pair = _field_property('_last_frozen_measurement_pair', writable=True)
    _last_measurement_attempt = _field_property('_last_measurement_attempt', writable=True)
    _last_measurement_attempt_key = _field_property('_last_measurement_attempt_key', writable=True)
    _log = _field_property('_log')
    _measurement_attempt_counter = _field_property('_measurement_attempt_counter', writable=True)
    _measurement_attempt_generation_id = _field_property('_measurement_attempt_generation_id', writable=True)
    _measurement_method_result_counters = _field_property('_measurement_method_result_counters', writable=True)
    _measurement_run_generation = _field_property('_measurement_run_generation')
    _method_evaluation_accepting = _field_property('_method_evaluation_accepting', writable=True)
    _method_evaluation_committed = _field_property('_method_evaluation_committed', writable=True)
    _method_evaluation_stop_retry_scheduled = _field_property('_method_evaluation_stop_retry_scheduled', writable=True)
    _method_evaluation_submitted = _field_property('_method_evaluation_submitted', writable=True)
    _prepare_video_recording_paths = _field_property('_prepare_video_recording_paths')
    _refresh_operation_header = _field_property('_refresh_operation_header')
    _session_recording_active = _field_property('_session_recording_active')
    _shutdown_in_progress = _field_property('_shutdown_in_progress')
    _start_camera_streams = _field_property('_start_camera_streams')
    _sync_rtcm_evidence_context = _field_property('_sync_rtcm_evidence_context')
    append_workstation_data = _field_property('append_workstation_data')
    camera_connected = _field_property('camera_connected')
    latest_extrinsics = _field_property('latest_extrinsics')
    latest_gps = _field_property('latest_gps')
    pixhawk_connection_thread = _field_property('pixhawk_connection_thread')
    pixhawk_disconnection_thread = _field_property('pixhawk_disconnection_thread')
    pixhawk_usb_bootstrap_thread = _field_property('pixhawk_usb_bootstrap_thread')
    record_button = _field_property('record_button')
    recording_status_label = _field_property('recording_status_label')
    stop_yolo_streams = _field_property('stop_yolo_streams')
    yolo_enabled = _field_property('yolo_enabled')


class SessionStopPort:
    __slots__ = ('_binding__camera_time_sync_snapshot', '_binding__format_video_recording_paths', '_binding__log', '_binding__measurement_session_hierarchy_enabled', '_binding_method_evaluation_pending_count', '_binding__method_evaluation_accepting', '_binding__method_evaluation_stop_retry_scheduled', '_binding__refresh_operation_header', '_binding__resume_queued_measurement_start', '_binding__session_recording_active', '_binding__start_camera_streams', '_binding__sync_rtcm_evidence_context', '_binding_target_drone_stream_snapshot', '_binding_stop_target_drone_stream', '_binding_append_workstation_data', '_binding_camera_connected', '_binding_flush_workstation_data', '_binding_flush_workstation_run_evidence', '_binding_left_pane', '_binding_qobject_parent', '_binding_record_button', '_binding_recording_status_label', '_binding_right_pane', '_binding_stream_backend_combo', '_binding_yolo_enabled')

    def __init__(self, *,
        _camera_time_sync_snapshot: FieldBinding,
        _format_video_recording_paths: FieldBinding,
        _log: FieldBinding,
        _measurement_session_hierarchy_enabled: FieldBinding,
        method_evaluation_pending_count: FieldBinding,
        _method_evaluation_accepting: FieldBinding,
        _method_evaluation_stop_retry_scheduled: FieldBinding,
        _refresh_operation_header: FieldBinding,
        _resume_queued_measurement_start: FieldBinding,
        _session_recording_active: FieldBinding,
        _start_camera_streams: FieldBinding,
        _sync_rtcm_evidence_context: FieldBinding,
        target_drone_stream_snapshot: FieldBinding,
        stop_target_drone_stream: FieldBinding,
        append_workstation_data: FieldBinding,
        camera_connected: FieldBinding,
        flush_workstation_data: FieldBinding,
        flush_workstation_run_evidence: FieldBinding,
        left_pane: FieldBinding,
        qobject_parent: FieldBinding,
        record_button: FieldBinding,
        recording_status_label: FieldBinding,
        right_pane: FieldBinding,
        stream_backend_combo: FieldBinding,
        yolo_enabled: FieldBinding,
    ) -> None:
        self._binding__camera_time_sync_snapshot = _camera_time_sync_snapshot
        self._binding__format_video_recording_paths = _format_video_recording_paths
        self._binding__log = _log
        self._binding__measurement_session_hierarchy_enabled = _measurement_session_hierarchy_enabled
        self._binding_method_evaluation_pending_count = method_evaluation_pending_count
        self._binding__method_evaluation_accepting = _method_evaluation_accepting
        self._binding__method_evaluation_stop_retry_scheduled = _method_evaluation_stop_retry_scheduled
        self._binding__refresh_operation_header = _refresh_operation_header
        self._binding__resume_queued_measurement_start = _resume_queued_measurement_start
        self._binding__session_recording_active = _session_recording_active
        self._binding__start_camera_streams = _start_camera_streams
        self._binding__sync_rtcm_evidence_context = _sync_rtcm_evidence_context
        self._binding_target_drone_stream_snapshot = target_drone_stream_snapshot
        self._binding_stop_target_drone_stream = stop_target_drone_stream
        self._binding_append_workstation_data = append_workstation_data
        self._binding_camera_connected = camera_connected
        self._binding_flush_workstation_data = flush_workstation_data
        self._binding_flush_workstation_run_evidence = flush_workstation_run_evidence
        self._binding_left_pane = left_pane
        self._binding_qobject_parent = qobject_parent
        self._binding_record_button = record_button
        self._binding_recording_status_label = recording_status_label
        self._binding_right_pane = right_pane
        self._binding_stream_backend_combo = stream_backend_combo
        self._binding_yolo_enabled = yolo_enabled

    _camera_time_sync_snapshot = _field_property('_camera_time_sync_snapshot')
    _format_video_recording_paths = _field_property('_format_video_recording_paths')
    _log = _field_property('_log')
    _measurement_session_hierarchy_enabled = _field_property('_measurement_session_hierarchy_enabled')
    method_evaluation_pending_count = _field_property('method_evaluation_pending_count')
    _method_evaluation_accepting = _field_property('_method_evaluation_accepting', writable=True)
    _method_evaluation_stop_retry_scheduled = _field_property('_method_evaluation_stop_retry_scheduled', writable=True)
    _refresh_operation_header = _field_property('_refresh_operation_header')
    _resume_queued_measurement_start = _field_property('_resume_queued_measurement_start')
    _session_recording_active = _field_property('_session_recording_active')
    _start_camera_streams = _field_property('_start_camera_streams')
    _sync_rtcm_evidence_context = _field_property('_sync_rtcm_evidence_context')
    target_drone_stream_snapshot = _field_property('target_drone_stream_snapshot')
    stop_target_drone_stream = _field_property('stop_target_drone_stream')
    append_workstation_data = _field_property('append_workstation_data')
    camera_connected = _field_property('camera_connected')
    flush_workstation_data = _field_property('flush_workstation_data')
    flush_workstation_run_evidence = _field_property('flush_workstation_run_evidence')
    left_pane = _field_property('left_pane')
    qobject_parent = _field_property('qobject_parent')
    record_button = _field_property('record_button')
    recording_status_label = _field_property('recording_status_label')
    right_pane = _field_property('right_pane')
    stream_backend_combo = _field_property('stream_backend_combo')
    yolo_enabled = _field_property('yolo_enabled')


class RecoveryPort:
    __slots__ = ('_binding__log', '_binding__measurement_id', '_binding__measurement_package', '_binding__measurement_package_state', '_binding__measurement_session', '_binding__measurement_session_hierarchy_enabled', '_binding__measurement_session_id', '_binding__measurement_session_state', '_binding_app_root', '_binding_record_button', '_binding_recording_status_label', '_binding_save_history_button')

    def __init__(self, *,
        _log: FieldBinding,
        _measurement_id: FieldBinding,
        _measurement_package: FieldBinding,
        _measurement_package_state: FieldBinding,
        _measurement_session: FieldBinding,
        _measurement_session_hierarchy_enabled: FieldBinding,
        _measurement_session_id: FieldBinding,
        _measurement_session_state: FieldBinding,
        app_root: FieldBinding,
        record_button: FieldBinding,
        recording_status_label: FieldBinding,
        save_history_button: FieldBinding,
    ) -> None:
        self._binding__log = _log
        self._binding__measurement_id = _measurement_id
        self._binding__measurement_package = _measurement_package
        self._binding__measurement_package_state = _measurement_package_state
        self._binding__measurement_session = _measurement_session
        self._binding__measurement_session_hierarchy_enabled = _measurement_session_hierarchy_enabled
        self._binding__measurement_session_id = _measurement_session_id
        self._binding__measurement_session_state = _measurement_session_state
        self._binding_app_root = app_root
        self._binding_record_button = record_button
        self._binding_recording_status_label = recording_status_label
        self._binding_save_history_button = save_history_button

    _log = _field_property('_log')
    _measurement_id = _field_property('_measurement_id', writable=True)
    _measurement_package = _field_property('_measurement_package', writable=True)
    _measurement_package_state = _field_property('_measurement_package_state', writable=True)
    _measurement_session = _field_property('_measurement_session', writable=True)
    _measurement_session_hierarchy_enabled = _field_property('_measurement_session_hierarchy_enabled')
    _measurement_session_id = _field_property('_measurement_session_id', writable=True)
    _measurement_session_state = _field_property('_measurement_session_state', writable=True)
    app_root = _field_property('app_root')
    record_button = _field_property('record_button')
    recording_status_label = _field_property('recording_status_label')
    save_history_button = _field_property('save_history_button')


class UlogPort:
    __slots__ = ('_binding__measurement_session_hierarchy_enabled', '_binding__recording_session_dir', '_binding_append_workstation_data', '_binding_append_workstation_run_event', '_binding_import_workstation_session_artifact', '_binding_measurement_storage_context', '_binding_register_workstation_session_artifact', '_binding_ulog_automatic_sources')

    def __init__(self, *,
        _measurement_session_hierarchy_enabled: FieldBinding,
        _recording_session_dir: FieldBinding,
        append_workstation_data: FieldBinding,
        append_workstation_run_event: FieldBinding,
        import_workstation_session_artifact: FieldBinding,
        measurement_storage_context: FieldBinding,
        register_workstation_session_artifact: FieldBinding,
        ulog_automatic_sources: FieldBinding,
    ) -> None:
        self._binding__measurement_session_hierarchy_enabled = _measurement_session_hierarchy_enabled
        self._binding__recording_session_dir = _recording_session_dir
        self._binding_append_workstation_data = append_workstation_data
        self._binding_append_workstation_run_event = append_workstation_run_event
        self._binding_import_workstation_session_artifact = import_workstation_session_artifact
        self._binding_measurement_storage_context = measurement_storage_context
        self._binding_register_workstation_session_artifact = register_workstation_session_artifact
        self._binding_ulog_automatic_sources = ulog_automatic_sources

    _measurement_session_hierarchy_enabled = _field_property('_measurement_session_hierarchy_enabled')
    _recording_session_dir = _field_property('_recording_session_dir')
    append_workstation_data = _field_property('append_workstation_data')
    append_workstation_run_event = _field_property('append_workstation_run_event')
    import_workstation_session_artifact = _field_property('import_workstation_session_artifact')
    measurement_storage_context = _field_property('measurement_storage_context')
    register_workstation_session_artifact = _field_property('register_workstation_session_artifact')
    ulog_automatic_sources = _field_property('ulog_automatic_sources')


class PhotoCapturePort:
    __slots__ = ('_binding__clear_oz_photo_capture_worker', '_binding__clear_pnp_photo_capture_worker', '_binding__continue_measurement_package_finalization', '_binding__control_port_for_camera', '_binding__handle_oz_photo_capture_completed', '_binding__handle_oz_photo_capture_failed', '_binding__handle_pnp_photo_capture_completed', '_binding__handle_pnp_photo_capture_failed', '_binding__log', '_binding__measurement_id', '_binding__measurement_package_explicit_lifecycle', '_binding__measurement_session_hierarchy_enabled', '_binding__recording_session_dir', '_binding_append_workstation_data', '_binding_begin_measurement', '_binding_begin_next_measurement', '_binding_ensure_workstation_run_evidence', '_binding_measurement_package_snapshot', '_binding_measurement_storage_context', '_binding_qobject_parent', '_binding_recording_status_label', '_binding_settings', '_binding_zoom_hosts', '_binding_zoom_timeout_spin')

    def __init__(self, *,
        _clear_oz_photo_capture_worker: FieldBinding,
        _clear_pnp_photo_capture_worker: FieldBinding,
        _continue_measurement_package_finalization: FieldBinding,
        _control_port_for_camera: FieldBinding,
        _handle_oz_photo_capture_completed: FieldBinding,
        _handle_oz_photo_capture_failed: FieldBinding,
        _handle_pnp_photo_capture_completed: FieldBinding,
        _handle_pnp_photo_capture_failed: FieldBinding,
        _log: FieldBinding,
        _measurement_id: FieldBinding,
        _measurement_package_explicit_lifecycle: FieldBinding,
        _measurement_session_hierarchy_enabled: FieldBinding,
        _recording_session_dir: FieldBinding,
        append_workstation_data: FieldBinding,
        begin_measurement: FieldBinding,
        begin_next_measurement: FieldBinding,
        ensure_workstation_run_evidence: FieldBinding,
        measurement_package_snapshot: FieldBinding,
        measurement_storage_context: FieldBinding,
        qobject_parent: FieldBinding,
        recording_status_label: FieldBinding,
        settings: FieldBinding,
        zoom_hosts: FieldBinding,
        zoom_timeout_spin: FieldBinding,
    ) -> None:
        self._binding__clear_oz_photo_capture_worker = _clear_oz_photo_capture_worker
        self._binding__clear_pnp_photo_capture_worker = _clear_pnp_photo_capture_worker
        self._binding__continue_measurement_package_finalization = _continue_measurement_package_finalization
        self._binding__control_port_for_camera = _control_port_for_camera
        self._binding__handle_oz_photo_capture_completed = _handle_oz_photo_capture_completed
        self._binding__handle_oz_photo_capture_failed = _handle_oz_photo_capture_failed
        self._binding__handle_pnp_photo_capture_completed = _handle_pnp_photo_capture_completed
        self._binding__handle_pnp_photo_capture_failed = _handle_pnp_photo_capture_failed
        self._binding__log = _log
        self._binding__measurement_id = _measurement_id
        self._binding__measurement_package_explicit_lifecycle = _measurement_package_explicit_lifecycle
        self._binding__measurement_session_hierarchy_enabled = _measurement_session_hierarchy_enabled
        self._binding__recording_session_dir = _recording_session_dir
        self._binding_append_workstation_data = append_workstation_data
        self._binding_begin_measurement = begin_measurement
        self._binding_begin_next_measurement = begin_next_measurement
        self._binding_ensure_workstation_run_evidence = ensure_workstation_run_evidence
        self._binding_measurement_package_snapshot = measurement_package_snapshot
        self._binding_measurement_storage_context = measurement_storage_context
        self._binding_qobject_parent = qobject_parent
        self._binding_recording_status_label = recording_status_label
        self._binding_settings = settings
        self._binding_zoom_hosts = zoom_hosts
        self._binding_zoom_timeout_spin = zoom_timeout_spin

    _clear_oz_photo_capture_worker = _field_property('_clear_oz_photo_capture_worker')
    _clear_pnp_photo_capture_worker = _field_property('_clear_pnp_photo_capture_worker')
    _continue_measurement_package_finalization = _field_property('_continue_measurement_package_finalization')
    _control_port_for_camera = _field_property('_control_port_for_camera')
    _handle_oz_photo_capture_completed = _field_property('_handle_oz_photo_capture_completed')
    _handle_oz_photo_capture_failed = _field_property('_handle_oz_photo_capture_failed')
    _handle_pnp_photo_capture_completed = _field_property('_handle_pnp_photo_capture_completed')
    _handle_pnp_photo_capture_failed = _field_property('_handle_pnp_photo_capture_failed')
    _log = _field_property('_log')
    _measurement_id = _field_property('_measurement_id')
    _measurement_package_explicit_lifecycle = _field_property('_measurement_package_explicit_lifecycle')
    _measurement_session_hierarchy_enabled = _field_property('_measurement_session_hierarchy_enabled')
    _recording_session_dir = _field_property('_recording_session_dir')
    append_workstation_data = _field_property('append_workstation_data')
    begin_measurement = _field_property('begin_measurement')
    begin_next_measurement = _field_property('begin_next_measurement')
    ensure_workstation_run_evidence = _field_property('ensure_workstation_run_evidence')
    measurement_package_snapshot = _field_property('measurement_package_snapshot')
    measurement_storage_context = _field_property('measurement_storage_context')
    qobject_parent = _field_property('qobject_parent')
    recording_status_label = _field_property('recording_status_label')
    settings = _field_property('settings')
    zoom_hosts = _field_property('zoom_hosts')
    zoom_timeout_spin = _field_property('zoom_timeout_spin')


class PhotoRecordsPort:
    __slots__ = ('_binding__log', '_binding__measurement_id', '_binding__measurement_session_hierarchy_enabled', '_binding__recording_session_dir', '_binding_actual_visible_zoom', '_binding_append_workstation_data', '_binding_append_workstation_run_event', '_binding_zoom_level_spin')

    def __init__(self, *,
        _log: FieldBinding,
        _measurement_id: FieldBinding,
        _measurement_session_hierarchy_enabled: FieldBinding,
        _recording_session_dir: FieldBinding,
        actual_visible_zoom: FieldBinding,
        append_workstation_data: FieldBinding,
        append_workstation_run_event: FieldBinding,
        zoom_level_spin: FieldBinding,
    ) -> None:
        self._binding__log = _log
        self._binding__measurement_id = _measurement_id
        self._binding__measurement_session_hierarchy_enabled = _measurement_session_hierarchy_enabled
        self._binding__recording_session_dir = _recording_session_dir
        self._binding_actual_visible_zoom = actual_visible_zoom
        self._binding_append_workstation_data = append_workstation_data
        self._binding_append_workstation_run_event = append_workstation_run_event
        self._binding_zoom_level_spin = zoom_level_spin

    _log = _field_property('_log')
    _measurement_id = _field_property('_measurement_id')
    _measurement_session_hierarchy_enabled = _field_property('_measurement_session_hierarchy_enabled')
    _recording_session_dir = _field_property('_recording_session_dir')
    actual_visible_zoom = _field_property('actual_visible_zoom')
    append_workstation_data = _field_property('append_workstation_data')
    append_workstation_run_event = _field_property('append_workstation_run_event')
    zoom_level_spin = _field_property('zoom_level_spin')


class SessionSavePort:
    __slots__ = ('_binding__continue_measurement_package_finalization', '_binding__handle_session_save_failed', '_binding__handle_session_save_thread_finished', '_binding__handle_session_saved', '_binding__log', '_binding__measurement_session_hierarchy_enabled', '_binding__recording_session_dir', '_binding__refresh_operation_header', '_binding_flush_workstation_data', '_binding_qobject_parent', '_binding_record_button', '_binding_recording_status_label', '_binding_save_history_button')

    def __init__(self, *,
        _continue_measurement_package_finalization: FieldBinding,
        _handle_session_save_failed: FieldBinding,
        _handle_session_save_thread_finished: FieldBinding,
        _handle_session_saved: FieldBinding,
        _log: FieldBinding,
        _measurement_session_hierarchy_enabled: FieldBinding,
        _recording_session_dir: FieldBinding,
        _refresh_operation_header: FieldBinding,
        flush_workstation_data: FieldBinding,
        qobject_parent: FieldBinding,
        record_button: FieldBinding,
        recording_status_label: FieldBinding,
        save_history_button: FieldBinding,
    ) -> None:
        self._binding__continue_measurement_package_finalization = _continue_measurement_package_finalization
        self._binding__handle_session_save_failed = _handle_session_save_failed
        self._binding__handle_session_save_thread_finished = _handle_session_save_thread_finished
        self._binding__handle_session_saved = _handle_session_saved
        self._binding__log = _log
        self._binding__measurement_session_hierarchy_enabled = _measurement_session_hierarchy_enabled
        self._binding__recording_session_dir = _recording_session_dir
        self._binding__refresh_operation_header = _refresh_operation_header
        self._binding_flush_workstation_data = flush_workstation_data
        self._binding_qobject_parent = qobject_parent
        self._binding_record_button = record_button
        self._binding_recording_status_label = recording_status_label
        self._binding_save_history_button = save_history_button

    _continue_measurement_package_finalization = _field_property('_continue_measurement_package_finalization')
    _handle_session_save_failed = _field_property('_handle_session_save_failed')
    _handle_session_save_thread_finished = _field_property('_handle_session_save_thread_finished')
    _handle_session_saved = _field_property('_handle_session_saved')
    _log = _field_property('_log')
    _measurement_session_hierarchy_enabled = _field_property('_measurement_session_hierarchy_enabled')
    _recording_session_dir = _field_property('_recording_session_dir')
    _refresh_operation_header = _field_property('_refresh_operation_header')
    flush_workstation_data = _field_property('flush_workstation_data')
    qobject_parent = _field_property('qobject_parent')
    record_button = _field_property('record_button')
    recording_status_label = _field_property('recording_status_label')
    save_history_button = _field_property('save_history_button')


class MetadataPort:
    __slots__ = ('_binding__active_baseline_m', '_binding__active_optical_pose_effective_from_pair_sequence', '_binding__active_optical_pose_selection_id', '_binding__camera_time_sync_drift_metadata', '_binding__camera_time_sync_snapshot', '_binding__distance_mode_label', '_binding__last_wiris_pc_utc_sync', '_binding__measurement_id', '_binding__measurement_path_label', '_binding__measurement_session', '_binding__px4_mag_manager_has_data_source', '_binding__px4_mag_status_cache', '_binding__recording_session_dir', '_binding__sync_px4_mag_manager_store', '_binding__triangulation_extrinsics_source', '_binding__yolo_time_basis', '_binding_camera_sync_reason', '_binding_camera_sync_status', '_binding_decoder_combo', '_binding_gnss_fixed_base_profile', '_binding_gnss_operation_manager', '_binding_left_url_edit', '_binding_px4_mag_managers', '_binding_right_url_edit', '_binding_rtsp_latency_spin', '_binding_rtsp_transport_combo', '_binding_stream_backend_combo', '_binding_workstation_measurement_methods_snapshot', '_binding_workstation_optical_pose_snapshot')

    def __init__(self, *,
        _active_baseline_m: FieldBinding,
        _active_optical_pose_effective_from_pair_sequence: FieldBinding,
        _active_optical_pose_selection_id: FieldBinding,
        _camera_time_sync_drift_metadata: FieldBinding,
        _camera_time_sync_snapshot: FieldBinding,
        _distance_mode_label: FieldBinding,
        _last_wiris_pc_utc_sync: FieldBinding,
        _measurement_id: FieldBinding,
        _measurement_path_label: FieldBinding,
        _measurement_session: FieldBinding,
        _px4_mag_manager_has_data_source: FieldBinding,
        _px4_mag_status_cache: FieldBinding,
        _recording_session_dir: FieldBinding,
        _sync_px4_mag_manager_store: FieldBinding,
        _triangulation_extrinsics_source: FieldBinding,
        _yolo_time_basis: FieldBinding,
        camera_sync_reason: FieldBinding,
        camera_sync_status: FieldBinding,
        decoder_combo: FieldBinding,
        gnss_fixed_base_profile: FieldBinding,
        gnss_operation_manager: FieldBinding,
        left_url_edit: FieldBinding,
        px4_mag_managers: FieldBinding,
        right_url_edit: FieldBinding,
        rtsp_latency_spin: FieldBinding,
        rtsp_transport_combo: FieldBinding,
        stream_backend_combo: FieldBinding,
        workstation_measurement_methods_snapshot: FieldBinding,
        workstation_optical_pose_snapshot: FieldBinding,
    ) -> None:
        self._binding__active_baseline_m = _active_baseline_m
        self._binding__active_optical_pose_effective_from_pair_sequence = _active_optical_pose_effective_from_pair_sequence
        self._binding__active_optical_pose_selection_id = _active_optical_pose_selection_id
        self._binding__camera_time_sync_drift_metadata = _camera_time_sync_drift_metadata
        self._binding__camera_time_sync_snapshot = _camera_time_sync_snapshot
        self._binding__distance_mode_label = _distance_mode_label
        self._binding__last_wiris_pc_utc_sync = _last_wiris_pc_utc_sync
        self._binding__measurement_id = _measurement_id
        self._binding__measurement_path_label = _measurement_path_label
        self._binding__measurement_session = _measurement_session
        self._binding__px4_mag_manager_has_data_source = _px4_mag_manager_has_data_source
        self._binding__px4_mag_status_cache = _px4_mag_status_cache
        self._binding__recording_session_dir = _recording_session_dir
        self._binding__sync_px4_mag_manager_store = _sync_px4_mag_manager_store
        self._binding__triangulation_extrinsics_source = _triangulation_extrinsics_source
        self._binding__yolo_time_basis = _yolo_time_basis
        self._binding_camera_sync_reason = camera_sync_reason
        self._binding_camera_sync_status = camera_sync_status
        self._binding_decoder_combo = decoder_combo
        self._binding_gnss_fixed_base_profile = gnss_fixed_base_profile
        self._binding_gnss_operation_manager = gnss_operation_manager
        self._binding_left_url_edit = left_url_edit
        self._binding_px4_mag_managers = px4_mag_managers
        self._binding_right_url_edit = right_url_edit
        self._binding_rtsp_latency_spin = rtsp_latency_spin
        self._binding_rtsp_transport_combo = rtsp_transport_combo
        self._binding_stream_backend_combo = stream_backend_combo
        self._binding_workstation_measurement_methods_snapshot = workstation_measurement_methods_snapshot
        self._binding_workstation_optical_pose_snapshot = workstation_optical_pose_snapshot

    _active_baseline_m = _field_property('_active_baseline_m')
    _active_optical_pose_effective_from_pair_sequence = _field_property('_active_optical_pose_effective_from_pair_sequence')
    _active_optical_pose_selection_id = _field_property('_active_optical_pose_selection_id')
    _camera_time_sync_drift_metadata = _field_property('_camera_time_sync_drift_metadata')
    _camera_time_sync_snapshot = _field_property('_camera_time_sync_snapshot')
    _distance_mode_label = _field_property('_distance_mode_label')
    _last_wiris_pc_utc_sync = _field_property('_last_wiris_pc_utc_sync')
    _measurement_id = _field_property('_measurement_id')
    _measurement_path_label = _field_property('_measurement_path_label')
    _measurement_session = _field_property('_measurement_session')
    _px4_mag_manager_has_data_source = _field_property('_px4_mag_manager_has_data_source')
    _px4_mag_status_cache = _field_property('_px4_mag_status_cache')
    _recording_session_dir = _field_property('_recording_session_dir')
    _sync_px4_mag_manager_store = _field_property('_sync_px4_mag_manager_store')
    _triangulation_extrinsics_source = _field_property('_triangulation_extrinsics_source')
    _yolo_time_basis = _field_property('_yolo_time_basis')
    camera_sync_reason = _field_property('camera_sync_reason')
    camera_sync_status = _field_property('camera_sync_status')
    decoder_combo = _field_property('decoder_combo')
    gnss_fixed_base_profile = _field_property('gnss_fixed_base_profile')
    gnss_operation_manager = _field_property('gnss_operation_manager')
    left_url_edit = _field_property('left_url_edit')
    px4_mag_managers = _field_property('px4_mag_managers')
    right_url_edit = _field_property('right_url_edit')
    rtsp_latency_spin = _field_property('rtsp_latency_spin')
    rtsp_transport_combo = _field_property('rtsp_transport_combo')
    stream_backend_combo = _field_property('stream_backend_combo')
    workstation_measurement_methods_snapshot = _field_property('workstation_measurement_methods_snapshot')
    workstation_optical_pose_snapshot = _field_property('workstation_optical_pose_snapshot')


class MeasurementRowsPort:
    __slots__ = ('_binding__active_calibration_selection_mode', '_binding__active_extrinsics_profile_path', '_binding__active_extrinsics_source_zoom_level', '_binding__active_intrinsics_profile_path', '_binding__active_intrinsics_source_zoom_level', '_binding__active_optical_pose_effective_from_pair_sequence', '_binding__active_optical_pose_selection_id', '_binding__log', '_binding__measurement_gnss_record_metadata', '_binding__measurement_id', '_binding__refresh_operation_header', '_binding_active_intrinsics', '_binding_zoom_level_spin')

    def __init__(self, *,
        _active_calibration_selection_mode: FieldBinding,
        _active_extrinsics_profile_path: FieldBinding,
        _active_extrinsics_source_zoom_level: FieldBinding,
        _active_intrinsics_profile_path: FieldBinding,
        _active_intrinsics_source_zoom_level: FieldBinding,
        _active_optical_pose_effective_from_pair_sequence: FieldBinding,
        _active_optical_pose_selection_id: FieldBinding,
        _log: FieldBinding,
        _measurement_gnss_record_metadata: FieldBinding,
        _measurement_id: FieldBinding,
        _refresh_operation_header: FieldBinding,
        active_intrinsics: FieldBinding,
        zoom_level_spin: FieldBinding,
    ) -> None:
        self._binding__active_calibration_selection_mode = _active_calibration_selection_mode
        self._binding__active_extrinsics_profile_path = _active_extrinsics_profile_path
        self._binding__active_extrinsics_source_zoom_level = _active_extrinsics_source_zoom_level
        self._binding__active_intrinsics_profile_path = _active_intrinsics_profile_path
        self._binding__active_intrinsics_source_zoom_level = _active_intrinsics_source_zoom_level
        self._binding__active_optical_pose_effective_from_pair_sequence = _active_optical_pose_effective_from_pair_sequence
        self._binding__active_optical_pose_selection_id = _active_optical_pose_selection_id
        self._binding__log = _log
        self._binding__measurement_gnss_record_metadata = _measurement_gnss_record_metadata
        self._binding__measurement_id = _measurement_id
        self._binding__refresh_operation_header = _refresh_operation_header
        self._binding_active_intrinsics = active_intrinsics
        self._binding_zoom_level_spin = zoom_level_spin

    _active_calibration_selection_mode = _field_property('_active_calibration_selection_mode')
    _active_extrinsics_profile_path = _field_property('_active_extrinsics_profile_path')
    _active_extrinsics_source_zoom_level = _field_property('_active_extrinsics_source_zoom_level')
    _active_intrinsics_profile_path = _field_property('_active_intrinsics_profile_path')
    _active_intrinsics_source_zoom_level = _field_property('_active_intrinsics_source_zoom_level')
    _active_optical_pose_effective_from_pair_sequence = _field_property('_active_optical_pose_effective_from_pair_sequence')
    _active_optical_pose_selection_id = _field_property('_active_optical_pose_selection_id')
    _log = _field_property('_log')
    _measurement_gnss_record_metadata = _field_property('_measurement_gnss_record_metadata')
    _measurement_id = _field_property('_measurement_id')
    _refresh_operation_header = _field_property('_refresh_operation_header')
    active_intrinsics = _field_property('active_intrinsics')
    zoom_level_spin = _field_property('zoom_level_spin')


class NativeSubmissionPort:
    __slots__ = ('_binding_native_frames_path', '_binding__handle_async_native_frame_result', '_binding__log', '_binding__measurement_session_hierarchy_enabled', '_binding__recording_session_dir', '_binding_append_workstation_data', '_binding_left_pane', '_binding_measurement_storage_context', '_binding_qobject_parent', '_binding_right_pane')

    def __init__(self, *,
        native_frames_path: FieldBinding,
        _handle_async_native_frame_result: FieldBinding,
        _log: FieldBinding,
        _measurement_session_hierarchy_enabled: FieldBinding,
        _recording_session_dir: FieldBinding,
        append_workstation_data: FieldBinding,
        left_pane: FieldBinding,
        measurement_storage_context: FieldBinding,
        qobject_parent: FieldBinding,
        right_pane: FieldBinding,
    ) -> None:
        self._binding_native_frames_path = native_frames_path
        self._binding__handle_async_native_frame_result = _handle_async_native_frame_result
        self._binding__log = _log
        self._binding__measurement_session_hierarchy_enabled = _measurement_session_hierarchy_enabled
        self._binding__recording_session_dir = _recording_session_dir
        self._binding_append_workstation_data = append_workstation_data
        self._binding_left_pane = left_pane
        self._binding_measurement_storage_context = measurement_storage_context
        self._binding_qobject_parent = qobject_parent
        self._binding_right_pane = right_pane

    native_frames_path = _field_property('native_frames_path')
    _handle_async_native_frame_result = _field_property('_handle_async_native_frame_result')
    _log = _field_property('_log')
    _measurement_session_hierarchy_enabled = _field_property('_measurement_session_hierarchy_enabled')
    _recording_session_dir = _field_property('_recording_session_dir')
    append_workstation_data = _field_property('append_workstation_data')
    left_pane = _field_property('left_pane')
    measurement_storage_context = _field_property('measurement_storage_context')
    qobject_parent = _field_property('qobject_parent')
    right_pane = _field_property('right_pane')


class NativeCompletionPort:
    __slots__ = ('_binding__log', '_binding__measurement_session_hierarchy_enabled', '_binding_append_workstation_data')

    def __init__(self, *,
        _log: FieldBinding,
        _measurement_session_hierarchy_enabled: FieldBinding,
        append_workstation_data: FieldBinding,
    ) -> None:
        self._binding__log = _log
        self._binding__measurement_session_hierarchy_enabled = _measurement_session_hierarchy_enabled
        self._binding_append_workstation_data = append_workstation_data

    _log = _field_property('_log')
    _measurement_session_hierarchy_enabled = _field_property('_measurement_session_hierarchy_enabled')
    append_workstation_data = _field_property('append_workstation_data')


class NativeRetryPort:
    __slots__ = ('_binding__log')

    def __init__(self, *,
        _log: FieldBinding,
    ) -> None:
        self._binding__log = _log

    _log = _field_property('_log')


class PrimaryPersistencePort:
    __slots__ = ('_binding__log', '_binding__refresh_operation_header', '_binding_append_workstation_data')

    def __init__(self, *,
        _log: FieldBinding,
        _refresh_operation_header: FieldBinding,
        append_workstation_data: FieldBinding,
    ) -> None:
        self._binding__log = _log
        self._binding__refresh_operation_header = _refresh_operation_header
        self._binding_append_workstation_data = append_workstation_data

    _log = _field_property('_log')
    _refresh_operation_header = _field_property('_refresh_operation_header')
    append_workstation_data = _field_property('append_workstation_data')


class HistoryPort:
    __slots__ = ('_binding__last_distance_history_key', '_binding__measurement_id', '_binding_distance_history', '_binding_distance_plot', '_binding_geometry_comparison_rows')

    def __init__(self, *,
        _last_distance_history_key: FieldBinding,
        _measurement_id: FieldBinding,
        distance_history: FieldBinding,
        distance_plot: FieldBinding,
        geometry_comparison_rows: FieldBinding,
    ) -> None:
        self._binding__last_distance_history_key = _last_distance_history_key
        self._binding__measurement_id = _measurement_id
        self._binding_distance_history = distance_history
        self._binding_distance_plot = distance_plot
        self._binding_geometry_comparison_rows = geometry_comparison_rows

    _last_distance_history_key = _field_property('_last_distance_history_key', writable=True)
    _measurement_id = _field_property('_measurement_id')
    distance_history = _field_property('distance_history')
    distance_plot = _field_property('distance_plot')
    geometry_comparison_rows = _field_property('geometry_comparison_rows')


class TimelinePort:
    __slots__ = ('_binding__distance_mode_label', '_binding__measurement_id', '_binding__pixhawk_gnss_source', '_binding__selected_rover_gps_state', '_binding__triangulation_extrinsics_source', '_binding_append_workstation_data', '_binding_latest_gps', '_binding_mavlink_link_provenance')

    def __init__(self, *,
        _distance_mode_label: FieldBinding,
        _measurement_id: FieldBinding,
        _pixhawk_gnss_source: FieldBinding,
        _selected_rover_gps_state: FieldBinding,
        _triangulation_extrinsics_source: FieldBinding,
        append_workstation_data: FieldBinding,
        latest_gps: FieldBinding,
        mavlink_link_provenance: FieldBinding,
    ) -> None:
        self._binding__distance_mode_label = _distance_mode_label
        self._binding__measurement_id = _measurement_id
        self._binding__pixhawk_gnss_source = _pixhawk_gnss_source
        self._binding__selected_rover_gps_state = _selected_rover_gps_state
        self._binding__triangulation_extrinsics_source = _triangulation_extrinsics_source
        self._binding_append_workstation_data = append_workstation_data
        self._binding_latest_gps = latest_gps
        self._binding_mavlink_link_provenance = mavlink_link_provenance

    _distance_mode_label = _field_property('_distance_mode_label')
    _measurement_id = _field_property('_measurement_id')
    _pixhawk_gnss_source = _field_property('_pixhawk_gnss_source')
    _selected_rover_gps_state = _field_property('_selected_rover_gps_state')
    _triangulation_extrinsics_source = _field_property('_triangulation_extrinsics_source')
    append_workstation_data = _field_property('append_workstation_data')
    latest_gps = _field_property('latest_gps')
    mavlink_link_provenance = _field_property('mavlink_link_provenance')


class TimingFieldsPort:
    __slots__ = ('_binding__camera_time_sync_metrics', '_binding__yolo_pair_max_delta_s', '_binding_camera_sync_reason', '_binding_camera_sync_status')

    def __init__(self, *,
        _camera_time_sync_metrics: FieldBinding,
        _yolo_pair_max_delta_s: FieldBinding,
        camera_sync_reason: FieldBinding,
        camera_sync_status: FieldBinding,
    ) -> None:
        self._binding__camera_time_sync_metrics = _camera_time_sync_metrics
        self._binding__yolo_pair_max_delta_s = _yolo_pair_max_delta_s
        self._binding_camera_sync_reason = camera_sync_reason
        self._binding_camera_sync_status = camera_sync_status

    _camera_time_sync_metrics = _field_property('_camera_time_sync_metrics')
    _yolo_pair_max_delta_s = _field_property('_yolo_pair_max_delta_s')
    camera_sync_reason = _field_property('camera_sync_reason')
    camera_sync_status = _field_property('camera_sync_status')


class GeometryFieldsPort:
    __slots__ = ('_binding__baseline_forward_geometry_from_camera_translation', '_binding_latest_gnss_relative')

    def __init__(self, *,
        _baseline_forward_geometry_from_camera_translation: FieldBinding,
        latest_gnss_relative: FieldBinding,
    ) -> None:
        self._binding__baseline_forward_geometry_from_camera_translation = _baseline_forward_geometry_from_camera_translation
        self._binding_latest_gnss_relative = latest_gnss_relative

    _baseline_forward_geometry_from_camera_translation = _field_property('_baseline_forward_geometry_from_camera_translation')
    latest_gnss_relative = _field_property('latest_gnss_relative')


class CalibrationFieldsPort:
    __slots__ = ('_binding__active_calibration_selection_mode', '_binding__active_extrinsics_profile_hash', '_binding__active_extrinsics_profile_path', '_binding__active_extrinsics_source_zoom_level', '_binding__active_intrinsics_profile_hash', '_binding__active_intrinsics_profile_path', '_binding__active_intrinsics_source_zoom_level', '_binding__checkerboard_translation_right_in_left_m', '_binding__intrinsics_storage_base_path', '_binding__intrinsics_storage_path', '_binding__workstation_setting', '_binding_active_intrinsics', '_binding_actual_visible_zoom', '_binding_focal_spin', '_binding_manual_focal_checkbox', '_binding_settings', '_binding_zoom_level_spin')

    def __init__(self, *,
        _active_calibration_selection_mode: FieldBinding,
        _active_extrinsics_profile_hash: FieldBinding,
        _active_extrinsics_profile_path: FieldBinding,
        _active_extrinsics_source_zoom_level: FieldBinding,
        _active_intrinsics_profile_hash: FieldBinding,
        _active_intrinsics_profile_path: FieldBinding,
        _active_intrinsics_source_zoom_level: FieldBinding,
        _checkerboard_translation_right_in_left_m: FieldBinding,
        _intrinsics_storage_base_path: FieldBinding,
        _intrinsics_storage_path: FieldBinding,
        _workstation_setting: FieldBinding,
        active_intrinsics: FieldBinding,
        actual_visible_zoom: FieldBinding,
        focal_spin: FieldBinding,
        manual_focal_checkbox: FieldBinding,
        settings: FieldBinding,
        zoom_level_spin: FieldBinding,
    ) -> None:
        self._binding__active_calibration_selection_mode = _active_calibration_selection_mode
        self._binding__active_extrinsics_profile_hash = _active_extrinsics_profile_hash
        self._binding__active_extrinsics_profile_path = _active_extrinsics_profile_path
        self._binding__active_extrinsics_source_zoom_level = _active_extrinsics_source_zoom_level
        self._binding__active_intrinsics_profile_hash = _active_intrinsics_profile_hash
        self._binding__active_intrinsics_profile_path = _active_intrinsics_profile_path
        self._binding__active_intrinsics_source_zoom_level = _active_intrinsics_source_zoom_level
        self._binding__checkerboard_translation_right_in_left_m = _checkerboard_translation_right_in_left_m
        self._binding__intrinsics_storage_base_path = _intrinsics_storage_base_path
        self._binding__intrinsics_storage_path = _intrinsics_storage_path
        self._binding__workstation_setting = _workstation_setting
        self._binding_active_intrinsics = active_intrinsics
        self._binding_actual_visible_zoom = actual_visible_zoom
        self._binding_focal_spin = focal_spin
        self._binding_manual_focal_checkbox = manual_focal_checkbox
        self._binding_settings = settings
        self._binding_zoom_level_spin = zoom_level_spin

    _active_calibration_selection_mode = _field_property('_active_calibration_selection_mode')
    _active_extrinsics_profile_hash = _field_property('_active_extrinsics_profile_hash')
    _active_extrinsics_profile_path = _field_property('_active_extrinsics_profile_path')
    _active_extrinsics_source_zoom_level = _field_property('_active_extrinsics_source_zoom_level')
    _active_intrinsics_profile_hash = _field_property('_active_intrinsics_profile_hash')
    _active_intrinsics_profile_path = _field_property('_active_intrinsics_profile_path')
    _active_intrinsics_source_zoom_level = _field_property('_active_intrinsics_source_zoom_level')
    _checkerboard_translation_right_in_left_m = _field_property('_checkerboard_translation_right_in_left_m')
    _intrinsics_storage_base_path = _field_property('_intrinsics_storage_base_path')
    _intrinsics_storage_path = _field_property('_intrinsics_storage_path')
    _workstation_setting = _field_property('_workstation_setting')
    active_intrinsics = _field_property('active_intrinsics')
    actual_visible_zoom = _field_property('actual_visible_zoom')
    focal_spin = _field_property('focal_spin')
    manual_focal_checkbox = _field_property('manual_focal_checkbox')
    settings = _field_property('settings')
    zoom_level_spin = _field_property('zoom_level_spin')


class RecordingStartFields:
    __slots__ = ('_binding__recording_mode', '_binding__manual_recording_active', '_binding_recording_session_id', '_binding__pnp_session_prepared', '_binding__session_save_finalized', '_binding__measurement_attempt_generation_id', '_binding_recording_session_started_unix_ns', '_binding_recording_session_started_monotonic_ns', '_binding_recording_session_anchor_uncertainty_ns', '_binding_recording_session_started_wall_s', '_binding_recording_session_started_monotonic_s', '_binding_distance_recording_started_monotonic_s', '_binding_distance_recording_rows', '_binding_distance_recording_keys', '_binding_external_parameter_recording_rows', '_binding_camera_recording_rows', '_binding_px4_recording_rows', '_binding_gnss_base_recording_rows', '_binding_extrinsics_comparison_recording_rows', '_binding_method_evaluation_bundle_rows', '_binding__recording_pair_row_index', '_binding__recording_pending_native_request_rows', '_binding__recovered_session_metadata', '_binding__recovered_session_output_dir', '_binding__recording_clock_sync_start_observation', '_binding__recording_clock_sync_end_observation', '_binding_recording_reference_attitudes', '_binding_recording_reference_gps', '_binding_recording_origin_gps', '_binding_recording_nominal_intrinsics_snapshot', '_binding_recording_nominal_optical_pose_profile_hash', '_binding_recording_reference_baseline_ned_m', '_binding__gnss_baseline_center_recording_sequence', '_binding_distance_recording_enabled', '_binding_video_recording_enabled', '_binding__last_distance_history_key', '_binding__measurement_attempt_counter', '_binding__last_measurement_attempt_key', '_binding__last_measurement_attempt', '_binding__last_frozen_measurement_pair', '_binding__method_evaluation_submitted', '_binding__method_evaluation_committed', '_binding__method_evaluation_accepting', '_binding__method_evaluation_stop_retry_scheduled', '_binding__native_frame_stop_retry_scheduled', '_binding__recording_stop_retry_generation', '_binding__recording_stop_retry_request', '_binding__measurement_method_result_counters', '_binding_video_recording_paths', '_binding__video_recording_allocation_sizes', '_binding__video_recording_start_snapshot', '_binding__measurement_video_segments', '_binding__last_recording_presence_state')

    def __init__(self, *,
        _recording_mode: FieldBinding,
        _manual_recording_active: FieldBinding,
        recording_session_id: FieldBinding,
        _pnp_session_prepared: FieldBinding,
        _session_save_finalized: FieldBinding,
        _measurement_attempt_generation_id: FieldBinding,
        recording_session_started_unix_ns: FieldBinding,
        recording_session_started_monotonic_ns: FieldBinding,
        recording_session_anchor_uncertainty_ns: FieldBinding,
        recording_session_started_wall_s: FieldBinding,
        recording_session_started_monotonic_s: FieldBinding,
        distance_recording_started_monotonic_s: FieldBinding,
        distance_recording_rows: FieldBinding,
        distance_recording_keys: FieldBinding,
        external_parameter_recording_rows: FieldBinding,
        camera_recording_rows: FieldBinding,
        px4_recording_rows: FieldBinding,
        gnss_base_recording_rows: FieldBinding,
        extrinsics_comparison_recording_rows: FieldBinding,
        method_evaluation_bundle_rows: FieldBinding,
        _recording_pair_row_index: FieldBinding,
        _recording_pending_native_request_rows: FieldBinding,
        _recovered_session_metadata: FieldBinding,
        _recovered_session_output_dir: FieldBinding,
        _recording_clock_sync_start_observation: FieldBinding,
        _recording_clock_sync_end_observation: FieldBinding,
        recording_reference_attitudes: FieldBinding,
        recording_reference_gps: FieldBinding,
        recording_origin_gps: FieldBinding,
        recording_nominal_intrinsics_snapshot: FieldBinding,
        recording_nominal_optical_pose_profile_hash: FieldBinding,
        recording_reference_baseline_ned_m: FieldBinding,
        _gnss_baseline_center_recording_sequence: FieldBinding,
        distance_recording_enabled: FieldBinding,
        video_recording_enabled: FieldBinding,
        _last_distance_history_key: FieldBinding,
        _measurement_attempt_counter: FieldBinding,
        _last_measurement_attempt_key: FieldBinding,
        _last_measurement_attempt: FieldBinding,
        _last_frozen_measurement_pair: FieldBinding,
        _method_evaluation_submitted: FieldBinding,
        _method_evaluation_committed: FieldBinding,
        _method_evaluation_accepting: FieldBinding,
        _method_evaluation_stop_retry_scheduled: FieldBinding,
        _native_frame_stop_retry_scheduled: FieldBinding,
        _recording_stop_retry_generation: FieldBinding,
        _recording_stop_retry_request: FieldBinding,
        _measurement_method_result_counters: FieldBinding,
        video_recording_paths: FieldBinding,
        _video_recording_allocation_sizes: FieldBinding,
        _video_recording_start_snapshot: FieldBinding,
        _measurement_video_segments: FieldBinding,
        _last_recording_presence_state: FieldBinding,
    ) -> None:
        self._binding__recording_mode = _recording_mode
        self._binding__manual_recording_active = _manual_recording_active
        self._binding_recording_session_id = recording_session_id
        self._binding__pnp_session_prepared = _pnp_session_prepared
        self._binding__session_save_finalized = _session_save_finalized
        self._binding__measurement_attempt_generation_id = _measurement_attempt_generation_id
        self._binding_recording_session_started_unix_ns = recording_session_started_unix_ns
        self._binding_recording_session_started_monotonic_ns = recording_session_started_monotonic_ns
        self._binding_recording_session_anchor_uncertainty_ns = recording_session_anchor_uncertainty_ns
        self._binding_recording_session_started_wall_s = recording_session_started_wall_s
        self._binding_recording_session_started_monotonic_s = recording_session_started_monotonic_s
        self._binding_distance_recording_started_monotonic_s = distance_recording_started_monotonic_s
        self._binding_distance_recording_rows = distance_recording_rows
        self._binding_distance_recording_keys = distance_recording_keys
        self._binding_external_parameter_recording_rows = external_parameter_recording_rows
        self._binding_camera_recording_rows = camera_recording_rows
        self._binding_px4_recording_rows = px4_recording_rows
        self._binding_gnss_base_recording_rows = gnss_base_recording_rows
        self._binding_extrinsics_comparison_recording_rows = extrinsics_comparison_recording_rows
        self._binding_method_evaluation_bundle_rows = method_evaluation_bundle_rows
        self._binding__recording_pair_row_index = _recording_pair_row_index
        self._binding__recording_pending_native_request_rows = _recording_pending_native_request_rows
        self._binding__recovered_session_metadata = _recovered_session_metadata
        self._binding__recovered_session_output_dir = _recovered_session_output_dir
        self._binding__recording_clock_sync_start_observation = _recording_clock_sync_start_observation
        self._binding__recording_clock_sync_end_observation = _recording_clock_sync_end_observation
        self._binding_recording_reference_attitudes = recording_reference_attitudes
        self._binding_recording_reference_gps = recording_reference_gps
        self._binding_recording_origin_gps = recording_origin_gps
        self._binding_recording_nominal_intrinsics_snapshot = recording_nominal_intrinsics_snapshot
        self._binding_recording_nominal_optical_pose_profile_hash = recording_nominal_optical_pose_profile_hash
        self._binding_recording_reference_baseline_ned_m = recording_reference_baseline_ned_m
        self._binding__gnss_baseline_center_recording_sequence = _gnss_baseline_center_recording_sequence
        self._binding_distance_recording_enabled = distance_recording_enabled
        self._binding_video_recording_enabled = video_recording_enabled
        self._binding__last_distance_history_key = _last_distance_history_key
        self._binding__measurement_attempt_counter = _measurement_attempt_counter
        self._binding__last_measurement_attempt_key = _last_measurement_attempt_key
        self._binding__last_measurement_attempt = _last_measurement_attempt
        self._binding__last_frozen_measurement_pair = _last_frozen_measurement_pair
        self._binding__method_evaluation_submitted = _method_evaluation_submitted
        self._binding__method_evaluation_committed = _method_evaluation_committed
        self._binding__method_evaluation_accepting = _method_evaluation_accepting
        self._binding__method_evaluation_stop_retry_scheduled = _method_evaluation_stop_retry_scheduled
        self._binding__native_frame_stop_retry_scheduled = _native_frame_stop_retry_scheduled
        self._binding__recording_stop_retry_generation = _recording_stop_retry_generation
        self._binding__recording_stop_retry_request = _recording_stop_retry_request
        self._binding__measurement_method_result_counters = _measurement_method_result_counters
        self._binding_video_recording_paths = video_recording_paths
        self._binding__video_recording_allocation_sizes = _video_recording_allocation_sizes
        self._binding__video_recording_start_snapshot = _video_recording_start_snapshot
        self._binding__measurement_video_segments = _measurement_video_segments
        self._binding__last_recording_presence_state = _last_recording_presence_state

    _recording_mode = _field_property('_recording_mode', writable=True, deletable=True)
    _manual_recording_active = _field_property('_manual_recording_active', writable=True, deletable=True)
    recording_session_id = _field_property('recording_session_id', writable=True, deletable=True)
    _pnp_session_prepared = _field_property('_pnp_session_prepared', writable=True, deletable=True)
    _session_save_finalized = _field_property('_session_save_finalized', writable=True, deletable=True)
    _measurement_attempt_generation_id = _field_property('_measurement_attempt_generation_id', writable=True, deletable=True)
    recording_session_started_unix_ns = _field_property('recording_session_started_unix_ns', writable=True, deletable=True)
    recording_session_started_monotonic_ns = _field_property('recording_session_started_monotonic_ns', writable=True, deletable=True)
    recording_session_anchor_uncertainty_ns = _field_property('recording_session_anchor_uncertainty_ns', writable=True, deletable=True)
    recording_session_started_wall_s = _field_property('recording_session_started_wall_s', writable=True, deletable=True)
    recording_session_started_monotonic_s = _field_property('recording_session_started_monotonic_s', writable=True, deletable=True)
    distance_recording_started_monotonic_s = _field_property('distance_recording_started_monotonic_s', writable=True, deletable=True)
    distance_recording_rows = _field_property('distance_recording_rows', writable=True, deletable=True)
    distance_recording_keys = _field_property('distance_recording_keys', writable=True, deletable=True)
    external_parameter_recording_rows = _field_property('external_parameter_recording_rows', writable=True, deletable=True)
    camera_recording_rows = _field_property('camera_recording_rows', writable=True, deletable=True)
    px4_recording_rows = _field_property('px4_recording_rows', writable=True, deletable=True)
    gnss_base_recording_rows = _field_property('gnss_base_recording_rows', writable=True, deletable=True)
    extrinsics_comparison_recording_rows = _field_property('extrinsics_comparison_recording_rows', writable=True, deletable=True)
    method_evaluation_bundle_rows = _field_property('method_evaluation_bundle_rows', writable=True, deletable=True)
    _recording_pair_row_index = _field_property('_recording_pair_row_index', writable=True, deletable=True)
    _recording_pending_native_request_rows = _field_property('_recording_pending_native_request_rows', writable=True, deletable=True)
    _recovered_session_metadata = _field_property('_recovered_session_metadata', writable=True, deletable=True)
    _recovered_session_output_dir = _field_property('_recovered_session_output_dir', writable=True, deletable=True)
    _recording_clock_sync_start_observation = _field_property('_recording_clock_sync_start_observation', writable=True, deletable=True)
    _recording_clock_sync_end_observation = _field_property('_recording_clock_sync_end_observation', writable=True, deletable=True)
    recording_reference_attitudes = _field_property('recording_reference_attitudes', writable=True, deletable=True)
    recording_reference_gps = _field_property('recording_reference_gps', writable=True, deletable=True)
    recording_origin_gps = _field_property('recording_origin_gps', writable=True, deletable=True)
    recording_nominal_intrinsics_snapshot = _field_property('recording_nominal_intrinsics_snapshot', writable=True, deletable=True)
    recording_nominal_optical_pose_profile_hash = _field_property('recording_nominal_optical_pose_profile_hash', writable=True, deletable=True)
    recording_reference_baseline_ned_m = _field_property('recording_reference_baseline_ned_m', writable=True, deletable=True)
    _gnss_baseline_center_recording_sequence = _field_property('_gnss_baseline_center_recording_sequence', writable=True, deletable=True)
    distance_recording_enabled = _field_property('distance_recording_enabled', writable=True, deletable=True)
    video_recording_enabled = _field_property('video_recording_enabled', writable=True, deletable=True)
    _last_distance_history_key = _field_property('_last_distance_history_key', writable=True, deletable=True)
    _measurement_attempt_counter = _field_property('_measurement_attempt_counter', writable=True, deletable=True)
    _last_measurement_attempt_key = _field_property('_last_measurement_attempt_key', writable=True, deletable=True)
    _last_measurement_attempt = _field_property('_last_measurement_attempt', writable=True, deletable=True)
    _last_frozen_measurement_pair = _field_property('_last_frozen_measurement_pair', writable=True, deletable=True)
    _method_evaluation_submitted = _field_property('_method_evaluation_submitted', writable=True, deletable=True)
    _method_evaluation_committed = _field_property('_method_evaluation_committed', writable=True, deletable=True)
    _method_evaluation_accepting = _field_property('_method_evaluation_accepting', writable=True, deletable=True)
    _method_evaluation_stop_retry_scheduled = _field_property('_method_evaluation_stop_retry_scheduled', writable=True, deletable=True)
    _native_frame_stop_retry_scheduled = _field_property('_native_frame_stop_retry_scheduled', writable=True, deletable=True)
    _recording_stop_retry_generation = _field_property('_recording_stop_retry_generation', writable=True, deletable=True)
    _recording_stop_retry_request = _field_property('_recording_stop_retry_request', writable=True, deletable=True)
    _measurement_method_result_counters = _field_property('_measurement_method_result_counters', writable=True, deletable=True)
    video_recording_paths = _field_property('video_recording_paths', writable=True, deletable=True)
    _video_recording_allocation_sizes = _field_property('_video_recording_allocation_sizes', writable=True, deletable=True)
    _video_recording_start_snapshot = _field_property('_video_recording_start_snapshot', writable=True, deletable=True)
    _measurement_video_segments = _field_property('_measurement_video_segments', writable=True, deletable=True)
    _last_recording_presence_state = _field_property('_last_recording_presence_state', writable=True, deletable=True)


