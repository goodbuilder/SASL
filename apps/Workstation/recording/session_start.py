"""Open recording generations transactionally and restore exact prior owners on failure.

RecordingStartFields is a finite transaction inventory spanning recording and
measurement state; resource rollback discards only allocations made by this start."""
from __future__ import annotations
import time
import uuid
from collections import deque
from collections.abc import Mapping, Sequence
from pathlib import Path
from Workstation.workstation_core.measurement_attempt import AttitudeSnapshot, GpsSnapshot
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.rtk_geometry import ned_delta_m
from .contracts import (RECORDING_PREVIEW_LIMIT, _RECORDING_START_MISSING, _RECORDING_START_STATE_ATTRIBUTES, new_measurement_attempt_generation_id)
from .state import (PhotoCaptureState, PrimaryPersistenceLedger, NativeEvidenceState, RecordingReferenceState, RecordingRowsState, SessionSaveState, RecordingSessionState, VideoSegmentState)
from .ports import SessionStartPort
from .ports import RecordingStartFields
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .measurement_rows import MeasurementRows
    from .metadata import Metadata
    from .photo_capture import PhotoCapture
    from .presentation import Presentation
    from .row_collections import RowCollections
    from .session_stop import SessionStop
    from .video_segments import VideoSegments


class SessionStart:
    def __init__(self, *,
        capture_state: PhotoCaptureState,
        ledger: PrimaryPersistenceLedger,
        native_state: NativeEvidenceState,
        references: RecordingReferenceState,
        rows: RecordingRowsState,
        save_state: SessionSaveState,
        session: RecordingSessionState,
        video_state: VideoSegmentState,
        transaction_fields: RecordingStartFields,
        boundary: SessionStartPort,
    ) -> None:
        self.capture_state = capture_state
        self.ledger = ledger
        self.native_state = native_state
        self.references = references
        self.rows = rows
        self.save_state = save_state
        self.session = session
        self.video_state = video_state
        self.transaction_fields = transaction_fields
        self.boundary = boundary

    def bind_collaborators(self, *,
        measurement_rows: "MeasurementRows",
        metadata: "Metadata",
        photo_capture: "PhotoCapture",
        presentation: "Presentation",
        row_collections: "RowCollections",
        session_stop: "SessionStop",
        video_segments: "VideoSegments",
    ) -> None:
        self.measurement_rows = measurement_rows
        self.metadata = metadata
        self.photo_capture = photo_capture
        self.presentation = presentation
        self.row_collections = row_collections
        self.session_stop = session_stop
        self.video_segments = video_segments

    def _recording_start_snapshot(self) -> dict[str, object]:
        retry_timer = getattr(self.session, "_recording_stop_retry_timer", None)
        try:
            retry_timer_active = bool(retry_timer is not None and retry_timer.isActive())
            retry_timer_remaining_ms = (
                max(1, int(retry_timer.remainingTime()))
                if retry_timer_active
                else 0
            )
        except (AttributeError, RuntimeError, TypeError, ValueError):
            retry_timer_active = False
            retry_timer_remaining_ms = 0
        return {
            "attributes": {
                name: getattr(self.transaction_fields, name, _RECORDING_START_MISSING)
                for name in _RECORDING_START_STATE_ATTRIBUTES
            },
            "deferred_native_frame_results": self.native_state._deferred_native_frame_results,
            "record_button_text": self.presentation._widget_text(
                getattr(self.boundary, "record_button", None)
            ),
            "status_label_text": self.presentation._widget_text(
                getattr(self.boundary, "recording_status_label", None)
            ),
            "camera_connected": bool(
                getattr(self.boundary, "camera_connected", False)
            ),
            "yolo_enabled": bool(getattr(self.boundary, "yolo_enabled", False)),
            "recording_stop_retry_timer_active": retry_timer_active,
            "recording_stop_retry_timer_remaining_ms": retry_timer_remaining_ms,
        }

    @staticmethod
    def _recording_row_resources(value: object) -> tuple[object, ...]:
        if isinstance(value, Mapping):
            return tuple(value.values())
        return (value,)

    def _release_failed_recording_start_rows(
        self,
        previous_attributes: Mapping[str, object],
        created_resources: Sequence[object] = (),
    ) -> None:
        row_attributes = (
            "distance_recording_rows",
            "camera_recording_rows",
            "px4_recording_rows",
            "gnss_base_recording_rows",
            "extrinsics_comparison_recording_rows",
            "method_evaluation_bundle_rows",
        )
        previous_ids = {
            id(resource)
            for name in row_attributes
            for resource in self._recording_row_resources(
                previous_attributes.get(name, _RECORDING_START_MISSING)
            )
            if resource is not _RECORDING_START_MISSING
        }
        candidates: list[object] = list(created_resources)
        for name in row_attributes:
            current = getattr(self.transaction_fields, name, _RECORDING_START_MISSING)
            candidates.extend(self._recording_row_resources(current))
        released: set[int] = set()
        for resource in candidates:
            identity = id(resource)
            if (
                resource is _RECORDING_START_MISSING
                or identity in previous_ids
                or identity in released
            ):
                continue
            released.add(identity)
            closer = getattr(resource, "discard_spool", None)
            if not callable(closer):
                closer = getattr(resource, "close", None)
            if not callable(closer):
                continue
            try:
                closer()
            except (OSError, RuntimeError, TypeError, ValueError):
                pass

    def _rollback_failed_recording_start(
        self,
        snapshot: Mapping[str, object],
        *,
        camera_start_attempted: bool,
        created_row_resources: Sequence[object] = (),
    ) -> None:
        attributes = snapshot.get("attributes")
        previous_attributes = (
            dict(attributes) if isinstance(attributes, Mapping) else {}
        )
        if camera_start_attempted:
            cancel_start = getattr(self.boundary, "_cancel_pending_camera_start", None)
            if callable(cancel_start):
                try:
                    cancel_start()
                except (AttributeError, RuntimeError, TypeError, ValueError):
                    pass
            stopper = getattr(self.boundary, "stop_yolo_streams", None)
            if callable(stopper):
                try:
                    stopper(wait=False)
                except TypeError:
                    try:
                        stopper()
                    except Exception:
                        pass
                except Exception:
                    pass

        self._release_failed_recording_start_rows(
            previous_attributes,
            created_row_resources,
        )
        for name, value in previous_attributes.items():
            if value is _RECORDING_START_MISSING:
                try:
                    delattr(self.transaction_fields, name)
                except AttributeError:
                    pass
            else:
                setattr(self.transaction_fields, name, value)
        deferred = snapshot.get(
            "deferred_native_frame_results", _RECORDING_START_MISSING
        )
        if deferred is not _RECORDING_START_MISSING:
            self.native_state._deferred_native_frame_results = deferred  # type: ignore[assignment]
        retry_timer = getattr(self.session, "_recording_stop_retry_timer", None)
        if (
            retry_timer is not None
            and bool(snapshot.get("recording_stop_retry_timer_active", False))
            and getattr(self.session, "_recording_stop_retry_request", None) is not None
        ):
            try:
                retry_timer.start(
                    max(
                        1,
                        int(
                            snapshot.get(
                                "recording_stop_retry_timer_remaining_ms", 100
                            )
                            or 100
                        ),
                    )
                )
            except (RuntimeError, TypeError, ValueError):
                pass

        for widget_name, snapshot_name in (
            ("record_button", "record_button_text"),
            ("recording_status_label", "status_label_text"),
        ):
            text = snapshot.get(snapshot_name)
            setter = getattr(getattr(self.boundary, widget_name, None), "setText", None)
            if text is not None and callable(setter):
                try:
                    setter(str(text))
                except (AttributeError, RuntimeError, TypeError):
                    pass

        sync_rtcm_context = getattr(self.boundary, "_sync_rtcm_evidence_context", None)
        if callable(sync_rtcm_context):
            try:
                sync_rtcm_context()
            except (AttributeError, RuntimeError, TypeError, ValueError):
                pass
        for method_name in (
            "_refresh_recording_table",
            "_refresh_recording_status_label",
            "_refresh_operation_header",
        ):
            refresher = {"_refresh_recording_table": self.presentation._refresh_recording_table, "_refresh_recording_status_label": self.presentation._refresh_recording_status_label, "_refresh_operation_header": getattr(self.boundary, "_refresh_operation_header", None)}.get(method_name)
            if callable(refresher):
                try:
                    refresher()
                except (AttributeError, RuntimeError, TypeError, ValueError):
                    pass

        if camera_start_attempted and bool(snapshot.get("camera_connected", False)):
            restarter = getattr(self.boundary, "_start_camera_streams", None)
            if callable(restarter):
                detection = bool(snapshot.get("yolo_enabled", False))
                try:
                    restarter(
                        enable_detection=detection,
                        switch_to_depth=detection,
                    )
                except Exception:
                    pass

    def toggle_session_recording(self) -> bool | None:
        if self.boundary._session_recording_active():
            self.session_stop._stop_session_recording()
            if not self.boundary._session_recording_active():
                self.session._automatic_session_recording = False
            return None
        return self._start_session_recording()

    def start_automatic_session_recording(self) -> bool:
        """Start best-effort recording for a transient Measurement run."""

        if self.boundary._session_recording_active():
            return True
        previous_automatic = bool(
            getattr(self.session, "_automatic_session_recording", False)
        )
        previous_manual = bool(
            getattr(self.session, "_manual_recording_active", False)
        )
        previous_mode = str(
            getattr(self.session, "_recording_mode", "manual") or "manual"
        )
        self.session._automatic_session_recording = True
        self.session._manual_recording_active = False
        self.session._recording_mode = "automatic_detection"
        try:
            result = self._start_session_recording(start_camera_streams=False)
        except Exception:
            self.session._automatic_session_recording = previous_automatic
            self.session._manual_recording_active = previous_manual
            self.session._recording_mode = previous_mode
            raise
        if result is False:
            self.session._automatic_session_recording = previous_automatic
            self.session._manual_recording_active = previous_manual
            self.session._recording_mode = previous_mode
            return False
        try:
            refresh = getattr(self.presentation, "_refresh_recording_status_label", None)
            if callable(refresh):
                refresh()
            else:
                self.boundary.recording_status_label.setText(
                    "측정 데이터 기록: STARTING · video=STARTING"
                )
        except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
            logger = getattr(self.boundary, "_log", None)
            if callable(logger):
                try:
                    logger(f"[저장] 자동 기록 상태 표시 갱신 실패: {exc}")
                except Exception:
                    pass
        return True

    def _start_session_recording(
        self,
        *,
        start_camera_streams: bool = True,
    ) -> bool | None:
        automatic = bool(
            getattr(self.session, "_automatic_session_recording", False)
        )
        if not automatic and bool(
            getattr(self.boundary, "_shutdown_in_progress", False)
        ):
            status_text = "Manual recording start rejected: application shutdown is in progress"
            self.boundary.recording_status_label.setText(status_text)
            logger = getattr(self.boundary, "_log", None)
            if callable(logger):
                logger(f"[저장] {status_text}")
            return False

        snapshot = self._recording_start_snapshot()
        transaction: dict[str, object] = {
            "camera_start_attempted": False,
            "created_row_resources": [],
        }
        try:
            return self._start_session_recording_transaction_body(
                start_camera_streams=start_camera_streams,
                transaction=transaction,
            )
        except Exception:
            self._rollback_failed_recording_start(
                snapshot,
                camera_start_attempted=bool(
                    transaction.get("camera_start_attempted", False)
                ),
                created_row_resources=tuple(
                    transaction.get("created_row_resources", ()) or ()
                ),
            )
            raise

    def _start_session_recording_transaction_body(
        self,
        *,
        start_camera_streams: bool,
        transaction: dict[str, object],
    ) -> bool | None:
        if self.photo_capture._camera_photo_capture_active():
            self.boundary.recording_status_label.setText("데이터 기록: HR/OZ still 캡처 완료 대기 중")
            return False
        if start_camera_streams and (
            self.boundary.pixhawk_connection_thread is not None
            or self.boundary.pixhawk_disconnection_thread is not None
            or self.boundary.pixhawk_usb_bootstrap_thread is not None
        ):
            status_text = "데이터 기록: Pixhawk 연결/부트스트랩 완료 대기 중"
            self.boundary.recording_status_label.setText(status_text)
            self.boundary._log(f"[저장] {status_text}")
            return False
        if not bool(getattr(self.session, "_automatic_session_recording", False)):
            self.session._recording_mode = "manual"
        self.session_stop._cancel_legacy_recording_stop_retry()
        self.session._manual_recording_active = not bool(
            getattr(self.session, "_automatic_session_recording", False)
        )
        generation = str(
            getattr(self.boundary, "_measurement_run_generation", "") or ""
        )
        self.session.recording_session_id = generation or f"manual-{uuid.uuid4().hex}"
        self.video_segments._begin_measurement_video_generation(self.session.recording_session_id)
        self.capture_state._pnp_session_prepared = False
        self.save_state._session_save_finalized = False
        self.boundary._measurement_attempt_generation_id = new_measurement_attempt_generation_id(
            self.session.recording_session_id
        )
        monotonic_before_ns = time.monotonic_ns()
        self.session.recording_session_started_unix_ns = time.time_ns()
        monotonic_after_ns = time.monotonic_ns()
        self.session.recording_session_started_monotonic_ns = (
            monotonic_before_ns + monotonic_after_ns
        ) // 2
        self.session.recording_session_anchor_uncertainty_ns = (
            monotonic_after_ns - monotonic_before_ns
        ) // 2
        self.session.recording_session_started_wall_s = self.session.recording_session_started_unix_ns * 1e-9
        self.session.recording_session_started_monotonic_s = self.session.recording_session_started_monotonic_ns * 1e-9
        self.session.distance_recording_started_monotonic_s = self.session.recording_session_started_monotonic_s
        created_row_resources = transaction.get("created_row_resources")
        if not isinstance(created_row_resources, list):
            created_row_resources = []
            transaction["created_row_resources"] = created_row_resources

        def allocate_rows(stream: str, **kwargs: object) -> object:
            rows = self.row_collections._new_live_rows(stream, **kwargs)
            created_row_resources.append(rows)
            return rows

        self.rows.distance_recording_rows = allocate_rows(
            "measurements",
            preview_limit=RECORDING_PREVIEW_LIMIT,
            priority=True,
        )
        self.rows.distance_recording_keys = []
        previous_external_rows = getattr(
            self.rows, "external_parameter_recording_rows", None
        )
        self.rows.external_parameter_recording_rows = deque(
            maxlen=getattr(previous_external_rows, "maxlen", 256) or 256
        )
        self.rows.camera_recording_rows = {
            side: allocate_rows(f"camera.{side}") for side in ("left", "right")
        }
        self.rows.px4_recording_rows = {
            side: allocate_rows(f"px4.{side}") for side in ("left", "right")
        }
        self.rows.gnss_base_recording_rows = allocate_rows("gnss.base")
        self.rows.extrinsics_comparison_recording_rows = allocate_rows(
            "geometry.comparisons"
        )
        self.rows.method_evaluation_bundle_rows = allocate_rows(
            "method_evaluations", priority=True
        )
        self.rows.method_result_rows = allocate_rows("method_results", priority=True)
        self.rows._gnss_baseline_center_recording_sequence = 0
        self.ledger._recording_pair_row_index = {}
        self.native_state._recording_pending_native_request_rows = {}
        self.native_state._deferred_native_frame_results = {}
        self.session._recovered_session_metadata = None
        self.session._recovered_session_output_dir = None
        self.session._recording_clock_sync_start_observation = None
        self.session._recording_clock_sync_end_observation = None
        self.references.recording_reference_attitudes = {
            camera: AttitudeSnapshot.from_state(state)
            for camera, state in self.boundary.latest_extrinsics.items()
            if camera in {"left", "right"}
        }
        self.references.recording_reference_gps = {
            camera: GpsSnapshot.from_state(state)
            for camera, state in self.boundary.latest_gps.items()
            if camera in {"left", "right"}
        }
        self.references.recording_origin_gps = self.references.recording_reference_gps.get("left")
        self.references.recording_nominal_intrinsics_snapshot = self.measurement_rows._snapshot_current_intrinsics()
        self.references.recording_nominal_optical_pose_profile_hash = str(
            getattr(self.boundary, "_active_optical_pose_profile_hash", "") or ""
        )
        left_reference_gps = self.references.recording_reference_gps.get("left")
        right_reference_gps = self.references.recording_reference_gps.get("right")
        self.references.recording_reference_baseline_ned_m = (
            None
            if left_reference_gps is None or right_reference_gps is None
            else ned_delta_m(left_reference_gps, right_reference_gps)
        )
        self.session.distance_recording_enabled = True
        self.session.video_recording_enabled = True
        sync_rtcm_context = getattr(self.boundary, "_sync_rtcm_evidence_context", None)
        if callable(sync_rtcm_context):
            sync_rtcm_context()
        self.metadata._record_px4_mag_status_snapshot()
        self.boundary._last_distance_history_key = None
        self.boundary._measurement_attempt_counter = 0
        self.boundary._last_measurement_attempt_key = None
        self.boundary._last_measurement_attempt = None
        self.boundary._last_frozen_measurement_pair = None
        self.boundary._method_evaluation_submitted = set()
        self.boundary._method_evaluation_committed = set()
        self.boundary._method_evaluation_accepting = True
        self.boundary._method_evaluation_stop_retry_scheduled = False
        self.boundary._measurement_method_result_counters = {
            method_id: {"VALID": 0, "BLOCKED": 0, "FAILED": 0}
            for method_id in MEASUREMENT_METHOD_IDS
        }
        self.boundary._prepare_video_recording_paths()
        video_start_sides: dict[str, dict[str, object]] = {}
        allocation_sizes: dict[str, int] = {}
        for side in ("left", "right"):
            raw_path = self.video_state.video_recording_paths.get(side)
            if raw_path is None:
                video_start_sides[side] = {
                    "state": "FAILED",
                    "path": "",
                    "size_bytes": 0,
                    "detail": "video path was not allocated; data collection continues",
                }
                continue
            path = Path(raw_path).expanduser()
            try:
                baseline_size = path.stat().st_size if path.is_file() else 0
            except OSError:
                baseline_size = 0
            allocation_sizes[side] = baseline_size
            video_start_sides[side] = {
                "state": "WAITING_FOR_CAMERA",
                "path": str(path),
                "size_bytes": baseline_size,
                "detail": "waiting for camera worker and positive file growth",
            }
        self.video_state._video_recording_allocation_sizes = allocation_sizes
        self.video_segments._set_video_recording_start_snapshot(
            video_start_sides,
            detail="video paths allocated; camera worker postcondition pending",
        )
        self.presentation._refresh_recording_table()
        self.boundary.record_button.setText("수동 기록 중지")
        self.boundary._log(
            "[저장] 데이터 기록을 시작했습니다: "
            f"mode={self.session._recording_mode}, {self.boundary._format_video_recording_paths()}"
        )
        if start_camera_streams and self.boundary.camera_connected:
            transaction["camera_start_attempted"] = True
            self.boundary._start_camera_streams(
                enable_detection=self.boundary.yolo_enabled,
                switch_to_depth=self.boundary.yolo_enabled,
            )
        elif start_camera_streams:
            self.boundary._log("[저장] 카메라 스트림을 시작하면 좌/우 영상 녹화가 함께 시작됩니다.")
        self.presentation._refresh_recording_status_label()
        self.boundary._refresh_operation_header()
        append = getattr(self.boundary, "append_workstation_data", None)
        if callable(append):
            try:
                append(
                    "events",
                    {
                        "event_type": "recording_started",
                        "recording_mode": str(
                            getattr(self.session, "_recording_mode", "manual")
                            or "manual"
                        ),
                        "runtime_generation": self.session.recording_session_id,
                    },
                    priority=True,
                )
            except (OSError, RuntimeError, TypeError, ValueError) as exc:
                # Measurement data is availability-first.  Failing to enqueue
                # this status event must not turn an otherwise complete start
                # into a half-rolled-back camera/session boundary.
                try:
                    self.boundary._log(
                        f"[저장] recording_started event 기록 실패: {exc}"
                    )
                except Exception:
                    pass
        # The view controller owns any pre-generation primary outcome and can
        # persist it through its compact late/direct path.  New rows must not
        # inherit a stale full-row retry candidate from the retired recording.
        self.ledger._pending_primary_record_rows.clear()
        return None
