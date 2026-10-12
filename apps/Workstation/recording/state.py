"""Authoritative recording state and the shared primary/native admission ledger.

Fields without an initial value remain absent until the legacy boundary supplies
one. This preserves optional-feature and rollback semantics for small adapters.
"""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING
import threading
from .contracts import (
    _NativeRowOwner, _NativeTerminalFingerprint, _PendingNativeCompletion,
    _PendingNativeConflict, NATIVE_COMPLETION_TERMINAL_DROP_CACHE_LIMIT,
)

if TYPE_CHECKING:
    from PyQt5.QtCore import QThread, QTimer
    from Workstation.workstation_core.measurement_attempt import AttitudeSnapshot, GpsSnapshot, StereoIntrinsicsSnapshot
    from Workstation.workstation_core.measurement_data_store import LiveDataRows
    from Workstation.workstation_core.session_storage.spool import SpoolRows
    from Workstation.workstation_core.native_frame_store import AsyncNativeFrameStore
    from Workstation.workstation_core.native_evidence_writer import AsyncNativeEvidenceWriter
    from Workstation.workers.session_workers import SessionSaveWorker
    from Workstation.wiris_workers.capture import WirisPnPPhotoCaptureWorker, WirisOZPhotoCaptureWorker
    from Workstation.controllers.ulog_controller import ULogController


@dataclass(init=False)
class RecordingSessionState:
    _automatic_session_recording: bool
    _manual_recording_active: bool
    _recording_mode: str
    _recording_clock_sync_end_observation: dict[str, object] | None
    _recording_clock_sync_start_observation: dict[str, object] | None
    _recording_stop_retry_generation: str
    _recording_stop_retry_request: dict[str, object] | None
    _recording_stop_retry_timer: QTimer | None
    _native_frame_stop_retry_scheduled: bool
    _recovered_session_metadata: dict[str, object] | None
    _recovered_session_output_dir: Path | None
    recording_session_id: str
    recording_session_started_unix_ns: int | None
    recording_session_started_monotonic_ns: int | None
    recording_session_anchor_uncertainty_ns: int | None
    recording_session_started_wall_s: float | None
    recording_session_started_monotonic_s: float | None
    distance_recording_started_monotonic_s: float | None
    distance_recording_enabled: bool
    video_recording_enabled: bool

    def __init__(self) -> None:
        pass


@dataclass(init=False)
class RecordingRowsState:
    distance_recording_rows: LiveDataRows | SpoolRows | list[dict[str, object]]
    distance_recording_keys: list[tuple[object, ...]]
    external_parameter_recording_rows: deque[dict[str, object]]
    camera_recording_rows: dict[str, LiveDataRows | SpoolRows | list[dict[str, object]]]
    px4_recording_rows: dict[str, LiveDataRows | SpoolRows | list[dict[str, object]]]
    gnss_base_recording_rows: LiveDataRows | SpoolRows | list[dict[str, object]]
    extrinsics_comparison_recording_rows: LiveDataRows | SpoolRows | list[dict[str, object]]
    method_evaluation_bundle_rows: LiveDataRows | SpoolRows | list[dict[str, object]]
    method_result_rows: LiveDataRows | SpoolRows | list[dict[str, object]]
    _gnss_baseline_center_recording_sequence: int

    def __init__(self) -> None:
        pass


@dataclass(init=False)
class RecordingReferenceState:
    recording_reference_attitudes: dict[str, AttitudeSnapshot]
    recording_reference_gps: dict[str, GpsSnapshot]
    recording_origin_gps: GpsSnapshot | None
    recording_nominal_intrinsics_snapshot: StereoIntrinsicsSnapshot | None
    recording_nominal_optical_pose_profile_hash: str
    recording_reference_baseline_ned_m: tuple[float, float, float] | None

    def __init__(self) -> None:
        pass


@dataclass(init=False)
class VideoSegmentState:
    _measurement_video_segments: dict[str, list[dict[str, object]]]
    _measurement_video_segments_by_generation: dict[str, dict[str, list[dict[str, object]]]]
    _recording_presence_state_by_generation: dict[str, str]
    _last_recording_presence_state: str
    _video_recording_allocation_sizes: dict[str, int]
    _video_recording_start_snapshot: dict[str, object]
    video_recording_paths: dict[str, Path | None]

    def __init__(self) -> None:
        pass


@dataclass(init=False)
class PhotoCaptureState:
    _pnp_session_prepared: bool
    pnp_photo_capture_records: list[dict[str, object]]
    oz_photo_capture_records: list[dict[str, object]]
    pnp_photo_capture_thread: QThread | None
    pnp_photo_capture_worker: WirisPnPPhotoCaptureWorker | None
    oz_photo_capture_thread: QThread | None
    oz_photo_capture_worker: WirisOZPhotoCaptureWorker | None
    _pending_hr_requests: dict[str, dict[str, object]]
    _pending_oz_requests: dict[str, dict[str, object]]

    def __init__(self) -> None:
        self._pending_hr_requests = {}
        self._pending_oz_requests = {}


@dataclass(init=False)
class SessionSaveState:
    _session_save_finalized: bool
    _session_save_last_error: str
    _session_save_retry_required: bool
    _session_save_shutdown_timeout_logged: bool
    _session_save_shutdown_wait_started_s: float | None
    _session_save_thread: QThread | None
    _session_save_worker: SessionSaveWorker | None

    def __init__(self) -> None:
        pass


@dataclass(init=False)
class NativeEvidenceState:
    _native_frame_async_stores: dict[str, AsyncNativeFrameStore]
    _native_evidence_writers: dict[str, AsyncNativeEvidenceWriter]
    _recording_pending_native_request_rows: dict[str, tuple[str, int]]
    _deferred_native_frame_results: dict[str, dict[str, object]]
    _native_row_owners: dict[str, _NativeRowOwner]
    _native_terminal_fingerprints: dict[str, _NativeTerminalFingerprint]
    _native_terminal_fingerprint_order: deque[str]
    _pending_native_completions: dict[str, _PendingNativeCompletion]
    _pending_native_conflicts: dict[str, _PendingNativeConflict]
    _native_conflicts_recorded: set[str]
    _native_completion_terminal_drops: deque[dict[str, object]]
    _native_completion_terminal_drop_count: int
    _native_completion_terminal_drop_evictions: int
    _native_completion_retry_scheduled: bool
    _native_completion_last_terminal_drop: dict[str, object]

    def __init__(self) -> None:
        self._deferred_native_frame_results = {}
        self._native_row_owners = {}
        self._native_terminal_fingerprints = {}
        self._native_terminal_fingerprint_order = deque()
        self._pending_native_completions = {}
        self._pending_native_conflicts = {}
        self._native_conflicts_recorded = set()
        self._native_completion_terminal_drops = deque(maxlen=NATIVE_COMPLETION_TERMINAL_DROP_CACHE_LIMIT)
        self._native_completion_terminal_drop_count = 0
        self._native_completion_terminal_drop_evictions = 0
        self._native_completion_retry_scheduled = False


@dataclass(init=False)
class PrimaryPersistenceLedger:
    _recording_pair_row_index: dict[tuple[object, ...], int]
    _pair_recording_row_leases: dict[tuple[object, ...], tuple[int, int]]
    _pending_primary_record_rows: dict[tuple[object, ...], dict[str, object]]
    _primary_persistence_lock: threading.RLock
    _compact_primary_persisted_keys: set[tuple[object, ...]]
    _compact_primary_persisted_order: deque[tuple[object, ...]]
    _native_primary_persisted_keys: set[tuple[object, ...]]
    _native_primary_persisted_order: deque[tuple[object, ...]]
    _primary_retry_required_keys: set[tuple[object, ...]]

    def __init__(self) -> None:
        self._pair_recording_row_leases = {}
        self._pending_primary_record_rows = {}
        self._primary_persistence_lock = threading.RLock()
        self._compact_primary_persisted_keys = set()
        self._compact_primary_persisted_order = deque()
        self._native_primary_persisted_keys = set()
        self._native_primary_persisted_order = deque()
        self._primary_retry_required_keys = set()


@dataclass(init=False)
class ULogState:
    _workstation_ulog_controller: ULogController | None

    def __init__(self) -> None:
        pass


RECORDING_STATE_FIELDS = {'_automatic_session_recording': 'session',
 '_manual_recording_active': 'session',
 '_recording_mode': 'session',
 '_recording_clock_sync_end_observation': 'session',
 '_recording_clock_sync_start_observation': 'session',
 '_recording_stop_retry_generation': 'session',
 '_recording_stop_retry_request': 'session',
 '_recording_stop_retry_timer': 'session',
 '_native_frame_stop_retry_scheduled': 'session',
 '_recovered_session_metadata': 'session',
 '_recovered_session_output_dir': 'session',
 'recording_session_id': 'session',
 'recording_session_started_unix_ns': 'session',
 'recording_session_started_monotonic_ns': 'session',
 'recording_session_anchor_uncertainty_ns': 'session',
 'recording_session_started_wall_s': 'session',
 'recording_session_started_monotonic_s': 'session',
 'distance_recording_started_monotonic_s': 'session',
 'distance_recording_enabled': 'session',
 'video_recording_enabled': 'session',
 'distance_recording_rows': 'rows',
 'distance_recording_keys': 'rows',
 'external_parameter_recording_rows': 'rows',
 'camera_recording_rows': 'rows',
 'px4_recording_rows': 'rows',
 'gnss_base_recording_rows': 'rows',
 'extrinsics_comparison_recording_rows': 'rows',
 'method_evaluation_bundle_rows': 'rows',
 'method_result_rows': 'rows',
 '_gnss_baseline_center_recording_sequence': 'rows',
 'recording_reference_attitudes': 'references',
 'recording_reference_gps': 'references',
 'recording_origin_gps': 'references',
 'recording_nominal_intrinsics_snapshot': 'references',
 'recording_nominal_optical_pose_profile_hash': 'references',
 'recording_reference_baseline_ned_m': 'references',
 '_measurement_video_segments': 'video_state',
 '_measurement_video_segments_by_generation': 'video_state',
 '_recording_presence_state_by_generation': 'video_state',
 '_last_recording_presence_state': 'video_state',
 '_video_recording_allocation_sizes': 'video_state',
 '_video_recording_start_snapshot': 'video_state',
 'video_recording_paths': 'video_state',
 '_pnp_session_prepared': 'capture_state',
 'pnp_photo_capture_records': 'capture_state',
 'oz_photo_capture_records': 'capture_state',
 'pnp_photo_capture_thread': 'capture_state',
 'pnp_photo_capture_worker': 'capture_state',
 'oz_photo_capture_thread': 'capture_state',
 'oz_photo_capture_worker': 'capture_state',
 '_session_save_finalized': 'save_state',
 '_session_save_last_error': 'save_state',
 '_session_save_retry_required': 'save_state',
 '_session_save_shutdown_timeout_logged': 'save_state',
 '_session_save_shutdown_wait_started_s': 'save_state',
 '_session_save_thread': 'save_state',
 '_session_save_worker': 'save_state',
 '_native_frame_async_stores': 'native_state',
 '_native_evidence_writers': 'native_state',
 '_recording_pending_native_request_rows': 'native_state',
 '_recording_pair_row_index': 'ledger',
 '_workstation_ulog_controller': 'ulog_state'}

RECORDING_STATE_FIELDS.update({
    "_native_completion_terminal_drop_count": "native_state",
    "_native_completion_last_terminal_drop": "native_state",
})

@dataclass
class RecordingState:
    session: RecordingSessionState = field(default_factory=RecordingSessionState)
    rows: RecordingRowsState = field(default_factory=RecordingRowsState)
    references: RecordingReferenceState = field(default_factory=RecordingReferenceState)
    video_state: VideoSegmentState = field(default_factory=VideoSegmentState)
    capture_state: PhotoCaptureState = field(default_factory=PhotoCaptureState)
    save_state: SessionSaveState = field(default_factory=SessionSaveState)
    native_state: NativeEvidenceState = field(default_factory=NativeEvidenceState)
    ledger: PrimaryPersistenceLedger = field(default_factory=PrimaryPersistenceLedger)
    ulog_state: ULogState = field(default_factory=ULogState)

    def state_for_field(self, name: str) -> object:
        """Resolve only fields declared by the recording compatibility contract."""
        return getattr(self, RECORDING_STATE_FIELDS[name])

    def resource_inventory(self) -> dict[str, object]:
        """Expose the exact owners participating in recording shutdown/retry.

        Native writers and admission queues are included explicitly because
        their names do not follow the GUI's thread/worker suffix convention.
        Values are the live handles, never copies of the owned resources.
        """
        resources: dict[str, object] = {}
        for owner, names in (
            (self.session, ("_recording_stop_retry_timer",)),
            (self.capture_state, (
                "pnp_photo_capture_thread", "pnp_photo_capture_worker",
                "oz_photo_capture_thread", "oz_photo_capture_worker",
                "_pending_hr_requests", "_pending_oz_requests",
            )),
            (self.save_state, ("_session_save_thread", "_session_save_worker")),
            (self.native_state, (
                "_native_frame_async_stores", "_native_evidence_writers",
                "_pending_native_completions", "_pending_native_conflicts",
                "_native_row_owners", "_deferred_native_frame_results",
            )),
            (self.ledger, ("_pending_primary_record_rows",)),
            (self.ulog_state, ("_workstation_ulog_controller",)),
            (self.rows, (
                "distance_recording_rows", "camera_recording_rows",
                "px4_recording_rows", "gnss_base_recording_rows",
                "extrinsics_comparison_recording_rows",
                "method_evaluation_bundle_rows", "method_result_rows",
            )),
        ):
            for name in names:
                if hasattr(owner, name):
                    resources[name] = getattr(owner, name)
        return resources
