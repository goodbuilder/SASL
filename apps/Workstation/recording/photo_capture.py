"""Own HR/OZ request contexts, worker lifetimes, and completion dispatch.

QObject receiver capabilities preserve GUI-thread delivery. Host request clocks
remain distinct from camera exposure timestamps and FTP verification evidence."""
from __future__ import annotations
import re
import time
import uuid
from collections.abc import Mapping, Sequence
from pathlib import Path
from PyQt5 import QtCore
from Workstation.workstation_core.hr_evidence import CapturePurpose, verify_capture_ftp_metadata
from Workstation.workstation_core.measurement_data_store import creation_timestamp_text
from Workstation.controllers.run_evidence_controller import persistence_failure_error
from Workstation.workstation_core.session_storage.paths import session_id_now
from Workstation.wiris_workers.capture import WirisOZPhotoCaptureWorker, WirisPnPPhotoCaptureWorker
from .state import (PhotoCaptureState, SessionSaveState, RecordingSessionState)
from .ports import PhotoCapturePort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .photo_records import PhotoRecords
    from .ulog import Ulog


class PhotoCapture:
    def __init__(self, *,
        capture_state: PhotoCaptureState,
        save_state: SessionSaveState,
        session: RecordingSessionState,
        boundary: PhotoCapturePort,
    ) -> None:
        self.capture_state = capture_state
        self.save_state = save_state
        self.session = session
        self.boundary = boundary

    def bind_collaborators(self, *,
        photo_records: "PhotoRecords",
        ulog: "Ulog",
    ) -> None:
        self.photo_records = photo_records
        self.ulog = ulog

    @staticmethod
    def _next_capture_id(records: Sequence[Mapping[str, object]], prefix: str) -> str:
        normalized = re.sub(r"[^a-z0-9]+", "", str(prefix).lower()) or "capture"
        pattern = re.compile(rf"^{re.escape(normalized)}-(\d+)$")
        high_water = max(
            (
                int(match.group(1))
                for record in records
                for match in (pattern.fullmatch(str(record.get("capture_id") or "").lower()),)
                if match is not None
            ),
            default=0,
        )
        return f"{normalized}-{high_water + 1:03d}"

    def _new_capture_id(
        self,
        records: Sequence[Mapping[str, object]],
        prefix: str,
    ) -> str:
        if not bool(
            getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)
        ):
            normalized = re.sub(r"[^a-z0-9]+", "", str(prefix).lower()) or "capture"
            reader = getattr(self.boundary, "measurement_storage_context", None)
            context = reader() if callable(reader) else {}
            if context.get("measurement_file_stamp"):
                return f"{normalized}-{context['measurement_file_stamp']}-{uuid.uuid4().hex[:8]}"
            return f"{normalized}-{creation_timestamp_text()}"
        return self._next_capture_id(records, prefix)

    def _measurement_artifact_stamp(self, utc_ns: int | None = None) -> str:
        reader = getattr(self.boundary, "measurement_storage_context", None)
        context = reader() if callable(reader) else {}
        return str(context.get("measurement_file_stamp") or creation_timestamp_text(utc_ns))

    def _camera_photo_capture_active(self) -> bool:
        return bool(
            getattr(self.capture_state, "pnp_photo_capture_thread", None) is not None
            or getattr(self.capture_state, "oz_photo_capture_thread", None) is not None
        )

    @staticmethod
    def _hr_capture_purpose(value: CapturePurpose | str) -> CapturePurpose:
        try:
            return CapturePurpose(str(getattr(value, "value", value)).strip().lower())
        except ValueError as exc:
            allowed = ", ".join(item.value for item in CapturePurpose)
            raise ValueError(f"HR capture purpose must be one of: {allowed}") from exc

    def _prepare_camera_evidence_session(self, capture_channel: str) -> None:
        """Prepare camera evidence without pretending Detection started.

        The public capture method may lazily create a child only for ``Pre``.
        Post/checkpoint capture never allocates a new child after sealing, so
        evidence cannot silently cross a Measurement boundary.
        """

        if not bool(getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)):
            if not str(getattr(self.session, "recording_session_id", "") or ""):
                self.session.recording_session_id = "measurement-data"
            writer = getattr(self.boundary, "append_workstation_data", None)
            if callable(writer):
                writer(
                    "capture.events",
                    {
                        "event_type": f"{capture_channel}_capture_requested",
                        "capture_channel": str(capture_channel),
                    },
                    priority=True,
                )
            return
        if not self._measurement_package_capture_allowed():
            raise RuntimeError(
                "Create Session Folder is required before pre/post evidence capture"
            )
        if not self.session.recording_session_id:
            self.session.recording_session_id = session_id_now()
            self.capture_state.pnp_photo_capture_records = []
            self.capture_state.oz_photo_capture_records = []
            self.save_state._session_save_finalized = False
            self.capture_state._pnp_session_prepared = True
        self.ulog._ensure_ulog_controller()
        evidence_start = getattr(self.boundary, "ensure_workstation_run_evidence", None)
        if callable(evidence_start):
            evidence_result = evidence_start(
                f"{capture_channel}_capture_requested",
                {
                    "session_id": self.session.recording_session_id,
                    "measurement_id": str(getattr(self.boundary, "_measurement_id", "") or ""),
                    "capture_channel": capture_channel,
                },
                source="RecordingController",
                coordinate_contract="native raster u-right/v-down; OpenCV optical RDF",
            )
            evidence_error = persistence_failure_error(evidence_result)
            if evidence_error is not None:
                raise OSError(
                    f"{capture_channel} run evidence setup failed: {evidence_error}"
                )

    def _measurement_package_capture_allowed(self, *, prepare_if_needed: bool = False) -> bool:
        """Keep legacy callers compatible while enforcing explicit UI sessions."""

        if not bool(getattr(self.boundary, "_measurement_package_explicit_lifecycle", False)):
            return True
        snapshotter = getattr(self.boundary, "measurement_package_snapshot", None)
        if not callable(snapshotter):
            return False
        try:
            snapshot = snapshotter()
        except Exception:
            return False
        if not isinstance(snapshot, Mapping):
            return False
        state = str(snapshot.get("state") or "").upper()
        if state in {"COLLECTING", "POST_CAPTURE"}:
            return True
        if not prepare_if_needed or state not in {
            "IDLE",
            "SEALED_COMPLETE",
            "SEALED_INCOMPLETE",
        }:
            return False
        starter = getattr(self.boundary, "begin_measurement", None)
        if not callable(starter):
            starter = getattr(self.boundary, "begin_next_measurement", None)
        if not callable(starter):
            return False
        try:
            if starter() is False:
                return False
            prepared = snapshotter()
        except Exception:
            return False
        return isinstance(prepared, Mapping) and str(
            prepared.get("state") or ""
        ).upper() == "COLLECTING"

    def _prepare_pnp_evidence_session(self) -> None:
        self._prepare_camera_evidence_session("hr")

    def _prepare_oz_evidence_session(self) -> None:
        self._prepare_camera_evidence_session("oz")

    def _new_camera_request_context(
        self,
        capture_id: str,
        purpose: CapturePurpose,
        capture_channel: str,
    ) -> dict[str, object]:
        """Anchor an HR request in host UTC and the session monotonic clock.

        ``request_session_delta_ns`` is a signed host-monotonic delta from the
        distance-session anchor.  It is ``None`` for HR captures made before a
        distance session starts.  Neither value is a camera exposure time.
        """

        monotonic_before_ns = time.monotonic_ns()
        request_utc_ns = time.time_ns()
        monotonic_after_ns = time.monotonic_ns()
        request_monotonic_ns = (monotonic_before_ns + monotonic_after_ns) // 2
        session_started_ns = getattr(
            self.session, "recording_session_started_monotonic_ns", None
        )
        request_session_delta_ns = (
            None
            if session_started_ns is None
            else request_monotonic_ns - int(session_started_ns)
        )
        return {
            "capture_id": str(capture_id),
            "purpose": purpose.value,
            "capture_channel": str(capture_channel),
            "request_utc_ns": request_utc_ns,
            "request_monotonic_ns": request_monotonic_ns,
            "request_clock_anchor_uncertainty_ns": (
                monotonic_after_ns - monotonic_before_ns
            )
            // 2,
            "request_utc_method": "host_time_ns_at_monotonic_midpoint",
            "request_session_delta_ns": request_session_delta_ns,
            "request_session_delta_s": (
                None
                if request_session_delta_ns is None
                else request_session_delta_ns * 1e-9
            ),
            "request_time_semantics": (
                "Workstation command request; not camera exposure or camera-file UTC"
            ),
        }

    def _new_hr_request_context(
        self,
        capture_id: str,
        purpose: CapturePurpose,
    ) -> dict[str, object]:
        return self._new_camera_request_context(capture_id, purpose, "hr")

    def _new_oz_request_context(
        self,
        capture_id: str,
        purpose: CapturePurpose,
    ) -> dict[str, object]:
        return self._new_camera_request_context(capture_id, purpose, "oz")

    @staticmethod
    def _camera_hr_request_delta(record: dict[str, object]) -> int | None:
        cameras = record.get("cameras")
        if not isinstance(cameras, dict):
            return None
        left = cameras.get("left")
        right = cameras.get("right")
        if not isinstance(left, dict) or not isinstance(right, dict):
            return None
        try:
            left_ns = int(left["capture_requested_unix_ns"])
            right_ns = int(right["capture_requested_unix_ns"])
        except (KeyError, TypeError, ValueError):
            return None
        return right_ns - left_ns

    def capture_pnp_hr_photos(
        self,
        purpose: CapturePurpose | str = CapturePurpose.UNSPECIFIED,
    ) -> bool:
        """Capture one left/right HR pair and append its evidence to the session JSONL.

        ``purpose`` is an experiment label (pre/post/checkpoint/unspecified),
        not a change to WIRIS capture behavior.  The legacy zero-argument API
        remains equivalent to ``purpose="unspecified"``.
        """

        normalized_purpose = self._hr_capture_purpose(purpose)
        if not self._measurement_package_capture_allowed(
            prepare_if_needed=normalized_purpose is CapturePurpose.PRE
        ):
            self.boundary.recording_status_label.setText(
                "Capture HR Still: 먼저 Session 폴더를 생성하세요"
            )
            return False
        if self._camera_photo_capture_active():
            self.boundary.recording_status_label.setText("HR/OZ still 캡처가 이미 진행 중입니다")
            return False
        self._prepare_pnp_evidence_session()
        capture_id = self._new_capture_id(self.capture_state.pnp_photo_capture_records, "hr")
        request = self._new_hr_request_context(capture_id, normalized_purpose)
        self.capture_state._pending_hr_requests[capture_id] = request
        target_infos = self._camera_photo_target_infos()
        if len(target_infos) != 2:
            self.photo_records._append_pnp_photo_capture_record(
                self.photo_records._hr_capture_event_record(
                    {
                        "state": "not_started",
                        "error": "both WIRIS control hosts are required",
                        "stored_on_camera": False,
                        "ftp_downloaded": False,
                    },
                    capture_id=capture_id,
                    request=self.capture_state._pending_hr_requests.pop(capture_id, None),
                )
            )
            return False
        self.capture_state.pnp_photo_capture_thread = QtCore.QThread(self.boundary.qobject_parent)
        self.capture_state.pnp_photo_capture_worker = WirisPnPPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            capture_purpose=normalized_purpose.value,
            completion_timeout_s=10.0,
            **self._wiris_ftp_worker_kwargs(),
        )
        self.capture_state.pnp_photo_capture_worker.moveToThread(self.capture_state.pnp_photo_capture_thread)
        self.capture_state.pnp_photo_capture_thread.started.connect(self.capture_state.pnp_photo_capture_worker.run)
        self.capture_state.pnp_photo_capture_worker.completed.connect(
            self.boundary._handle_pnp_photo_capture_completed
        )
        self.capture_state.pnp_photo_capture_worker.failed.connect(
            lambda error, capture_id=capture_id: self.boundary._handle_pnp_photo_capture_failed(
                capture_id, error
            )
        )
        self.capture_state.pnp_photo_capture_worker.finished.connect(self.capture_state.pnp_photo_capture_thread.quit)
        self.capture_state.pnp_photo_capture_worker.finished.connect(self.capture_state.pnp_photo_capture_worker.deleteLater)
        self.capture_state.pnp_photo_capture_thread.finished.connect(self.capture_state.pnp_photo_capture_thread.deleteLater)
        self.capture_state.pnp_photo_capture_thread.finished.connect(self.boundary._clear_pnp_photo_capture_worker)
        self.capture_state.pnp_photo_capture_thread.start()
        self.boundary.recording_status_label.setText(
            f"Capture HR Still {capture_id}: 좌/우 카메라 내부 저장 중"
        )
        return True

    def capture_pnp_oz_photos(
        self,
        purpose: CapturePurpose | str = CapturePurpose.UNSPECIFIED,
    ) -> bool:
        """Capture one visible operating-zoom still pair.

        OZ stills use the camera's normal visible-image storage command
        (``SIVI TRUE``) and are kept as a separate evidence stream from both
        the RTSP/MP4 session recording and the visible-HR capture stream.
        """

        normalized_purpose = self._hr_capture_purpose(purpose)
        if not self._measurement_package_capture_allowed(
            prepare_if_needed=normalized_purpose is CapturePurpose.PRE
        ):
            self.boundary.recording_status_label.setText(
                "Capture OZ Still: 먼저 Session 폴더를 생성하세요"
            )
            return False
        if self._camera_photo_capture_active():
            self.boundary.recording_status_label.setText("HR/OZ still 캡처가 이미 진행 중입니다")
            return False
        self._prepare_oz_evidence_session()
        capture_id = self._new_capture_id(
            getattr(self.capture_state, "oz_photo_capture_records", ()),
            "oz",
        )
        request = self._new_oz_request_context(capture_id, normalized_purpose)
        self.capture_state._pending_oz_requests[capture_id] = request
        target_infos = self._camera_photo_target_infos()
        if len(target_infos) != 2:
            self.photo_records._append_oz_photo_capture_record(
                self.photo_records._oz_capture_event_record(
                    {
                        "state": "not_started",
                        "error": "both WIRIS control hosts are required",
                        "stored_on_camera": False,
                        "ftp_downloaded": False,
                    },
                    capture_id=capture_id,
                    request=self.capture_state._pending_oz_requests.pop(capture_id, None),
                )
            )
            return False
        self.capture_state.oz_photo_capture_thread = QtCore.QThread(self.boundary.qobject_parent)
        self.capture_state.oz_photo_capture_worker = WirisOZPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            capture_purpose=normalized_purpose.value,
            completion_timeout_s=10.0,
            **self._wiris_ftp_worker_kwargs(),
        )
        self.capture_state.oz_photo_capture_worker.moveToThread(self.capture_state.oz_photo_capture_thread)
        self.capture_state.oz_photo_capture_thread.started.connect(self.capture_state.oz_photo_capture_worker.run)
        self.capture_state.oz_photo_capture_worker.completed.connect(
            self.boundary._handle_oz_photo_capture_completed
        )
        self.capture_state.oz_photo_capture_worker.failed.connect(
            lambda error, capture_id=capture_id: self.boundary._handle_oz_photo_capture_failed(
                capture_id, error
            )
        )
        self.capture_state.oz_photo_capture_worker.finished.connect(self.capture_state.oz_photo_capture_thread.quit)
        self.capture_state.oz_photo_capture_worker.finished.connect(self.capture_state.oz_photo_capture_worker.deleteLater)
        self.capture_state.oz_photo_capture_thread.finished.connect(self.capture_state.oz_photo_capture_thread.deleteLater)
        self.capture_state.oz_photo_capture_thread.finished.connect(self.boundary._clear_oz_photo_capture_worker)
        self.capture_state.oz_photo_capture_thread.start()
        self.boundary.recording_status_label.setText(
            f"Capture OZ Still {capture_id}: 좌/우 카메라 내부 저장 중"
        )
        return True

    def _wiris_ftp_worker_kwargs(self, *, output_dir: Path | None = None) -> dict[str, object]:
        """FTP config shared by every WIRIS still-capture worker (HR/OZ/etc).

        The FTP server lives on the same WIRIS Enterprise host as the
        CPTR/ICPT command channel (only the port differs), so no separate
        host is needed here -- the worker reuses the host from target_infos.
        Each camera has its own local FTP account, so the port/user/password/
        remote dir/timeout are looked up per ``left``/``right`` label.

        ``output_dir`` lets a caller outside the Recording session (e.g.
        Calibration, whose save root is independent of any recording
        session) stage FTP downloads under its own directory instead of the
        default ``<recording_session_dir>/frames``.
        """

        settings = getattr(self.boundary, "settings", None)
        if output_dir is None and self.session.recording_session_id:
            root = self.boundary._recording_session_dir(self.session.recording_session_id)
            output_dir = (
                root / "images" / "camera_stills"
                if not bool(
                    getattr(
                        self.boundary,
                        "_measurement_session_hierarchy_enabled",
                        True,
                    )
                )
                else root / "frames"
            )
        return {
            "ftp_enabled": bool(getattr(settings, "wiris_ftp_enabled", False)),
            "ftp_settings_by_label": {
                "left": {
                    "port": int(getattr(settings, "left_ftp_port", 21)),
                    "user": getattr(settings, "left_ftp_user", "anonymous"),
                    "password": getattr(settings, "left_ftp_password", ""),
                    "remote_dir": getattr(settings, "left_ftp_remote_dir", "/"),
                    "timeout_s": float(getattr(settings, "left_ftp_timeout_s", 10.0)),
                },
                "right": {
                    "port": int(getattr(settings, "right_ftp_port", 21)),
                    "user": getattr(settings, "right_ftp_user", "anonymous"),
                    "password": getattr(settings, "right_ftp_password", ""),
                    "remote_dir": getattr(settings, "right_ftp_remote_dir", "/"),
                    "timeout_s": float(getattr(settings, "right_ftp_timeout_s", 10.0)),
                },
            },
            "ftp_output_dir": output_dir,
        }

    def _wiris_ftp_status_text(self, ftp_downloaded: bool) -> str:
        settings = getattr(self.boundary, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            return "FTP=disabled"
        return "FTP=ok" if ftp_downloaded else "FTP=failed"

    def _camera_photo_target_infos(self) -> tuple[tuple[str, str, int, float], ...]:
        return tuple(
            (
                label,
                host,
                self.boundary._control_port_for_camera(label),
                self.boundary.zoom_timeout_spin.value(),
            )
            for label, host in self.boundary.zoom_hosts("both")
            if str(host or "").strip()
        )

    @staticmethod
    def _verify_ftp_capture_cameras(cameras: object) -> None:
        """Re-verify each FTP-downloaded capture's bytes against its reported
        metadata, downgrading that camera's state to "failed" on mismatch.

        wiris_runtime_workers.py already verifies the bytes once immediately
        after the FTP transfer completes; this catches corruption or an
        unexpected file swap in the window between that download and this
        completion-event handling running, before the capture is recorded as
        trusted evidence.
        """

        if not isinstance(cameras, dict):
            return
        for item in cameras.values():
            if not isinstance(item, dict):
                continue
            local_path = item.get("ftp_local_path")
            if not item.get("ftp_downloaded") or not local_path:
                continue
            try:
                item["capture_metadata_verified"] = verify_capture_ftp_metadata(
                    Path(local_path), item
                )
            except (OSError, ValueError) as exc:
                item["state"] = "failed"
                item["error"] = f"capture metadata verification failed: {exc}"

    def _handle_pnp_photo_capture_completed(self, result: object) -> None:
        record = dict(result or {}) if isinstance(result, dict) else {"state": "failed"}
        cameras = record.get("cameras")
        self._verify_ftp_capture_cameras(cameras)
        camera_states = (
            [str(item.get("state", "failed")) for item in cameras.values()]
            if isinstance(cameras, dict)
            else ["failed"]
        )
        record["state"] = "completed" if camera_states and all(
            state == "completed" for state in camera_states
        ) else "partial_or_failed"
        record["stored_on_camera"] = record["state"] == "completed"
        ftp_states = (
            [bool(item.get("ftp_downloaded")) for item in cameras.values()]
            if isinstance(cameras, dict)
            else []
        )
        record["ftp_downloaded"] = bool(ftp_states) and all(ftp_states)
        ftp_text = self._wiris_ftp_status_text(record["ftp_downloaded"])
        capture_id = str(record.get("capture_id") or "")
        record = self.photo_records._hr_capture_event_record(
            record,
            capture_id=capture_id,
            request=self.capture_state._pending_hr_requests.pop(capture_id, None),
        )
        self.photo_records._append_pnp_photo_capture_record(record)
        self.boundary._log(
            f"[Capture HR Still] {capture_id} 카메라 내부 저장: state={record['state']}, {ftp_text}"
        )
        self.boundary.recording_status_label.setText(
            f"Capture HR Still {capture_id}: {record['state']} · camera internal · {ftp_text}"
        )

    def _handle_pnp_photo_capture_failed(self, capture_id: str, error: str) -> None:
        self.photo_records._append_pnp_photo_capture_record(
            self.photo_records._hr_capture_event_record(
                {
                    "state": "failed",
                    "error": str(error),
                    "stored_on_camera": False,
                    "ftp_downloaded": False,
                },
                capture_id=str(capture_id),
                request=self.capture_state._pending_hr_requests.pop(str(capture_id), None),
            )
        )
        self.boundary._log(f"[Capture HR Still] {capture_id} 촬영 실패: {error}")
        self.boundary.recording_status_label.setText(f"Capture HR Still {capture_id}: failed ({error})")

    def _handle_oz_photo_capture_completed(self, result: object) -> None:
        record = dict(result or {}) if isinstance(result, dict) else {"state": "failed"}
        cameras = record.get("cameras")
        self._verify_ftp_capture_cameras(cameras)
        camera_states = (
            [str(item.get("state", "failed")) for item in cameras.values()]
            if isinstance(cameras, dict)
            else ["failed"]
        )
        record["state"] = "completed" if camera_states and all(
            state == "completed" for state in camera_states
        ) else "partial_or_failed"
        record["stored_on_camera"] = record["state"] == "completed"
        ftp_states = (
            [bool(item.get("ftp_downloaded")) for item in cameras.values()]
            if isinstance(cameras, dict)
            else []
        )
        record["ftp_downloaded"] = bool(ftp_states) and all(ftp_states)
        ftp_text = self._wiris_ftp_status_text(record["ftp_downloaded"])
        capture_id = str(record.get("capture_id") or "")
        record = self.photo_records._oz_capture_event_record(
            record,
            capture_id=capture_id,
            request=self.capture_state._pending_oz_requests.pop(capture_id, None),
        )
        self.photo_records._append_oz_photo_capture_record(record)
        self.boundary._log(
            f"[Capture OZ Still] {capture_id} 카메라 내부 저장: state={record['state']}, {ftp_text}"
        )
        self.boundary.recording_status_label.setText(
            f"Capture OZ Still {capture_id}: {record['state']} · camera internal · {ftp_text}"
        )

    def _handle_oz_photo_capture_failed(self, capture_id: str, error: str) -> None:
        self.photo_records._append_oz_photo_capture_record(
            self.photo_records._oz_capture_event_record(
                {
                    "state": "failed",
                    "error": str(error),
                    "stored_on_camera": False,
                    "ftp_downloaded": False,
                },
                capture_id=str(capture_id),
                request=self.capture_state._pending_oz_requests.pop(str(capture_id), None),
            )
        )
        self.boundary._log(f"[Capture OZ Still] {capture_id} 촬영 실패: {error}")
        self.boundary.recording_status_label.setText(f"Capture OZ Still {capture_id}: failed ({error})")

    def _clear_pnp_photo_capture_worker(self) -> None:
        self.capture_state.pnp_photo_capture_thread = None
        self.capture_state.pnp_photo_capture_worker = None
        continuation = getattr(
            self.boundary, "_continue_measurement_package_finalization", None
        )
        if callable(continuation):
            continuation()

    def _clear_oz_photo_capture_worker(self) -> None:
        self.capture_state.oz_photo_capture_thread = None
        self.capture_state.oz_photo_capture_worker = None
        continuation = getattr(
            self.boundary, "_continue_measurement_package_finalization", None
        )
        if callable(continuation):
            continuation()
