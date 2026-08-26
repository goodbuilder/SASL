"""Workstation camera stream and detection lifecycle controller."""

from __future__ import annotations

import time
from collections.abc import Mapping
from pathlib import Path

from PyQt5 import QtCore, QtGui
from sasl_common.operator_runtime import (
    DECODER_AUTO,
    DECODER_SOFTWARE,
    NVIDIA_H264_DECODER,
    SOFTWARE_H264_DECODER,
    current_decoder_selection,
)

from Workstation.camera_time_sync_controller import (
    CAMERA_SYNC_STATUS_NOT_VERIFIED,
    CAMERA_SYNC_STATUS_RECONNECTING,
    CAMERA_SYNC_STATUS_WARMING,
)
from Workstation.workstation_core.yolo_stream import recording_integrity_contract
from Workstation.workstation_core.mp4_integrity import H264_MP4_INTEGRITY_SCHEMA

CAMERA_START_READINESS_TIMEOUT_S = 35.0
CAMERA_START_READINESS_POLL_MS = 100
VIDEO_START_READINESS_TIMEOUT_S = 35.0
VIDEO_START_READINESS_POLL_MS = 100

_VIDEO_START_FAILURE_MARKERS = (
    "MP4 저장 경로 준비 실패",
    "영상 녹화 파일 열기 실패",
    "영상 녹화 시작 실패",
    "영상 녹화 writer 오류",
)

_CAMERA_START_FAILURE_MARKERS = (
    "GStreamer 사용 불가",
    "GStreamer 오류",
    "스트림 오류",
    "스트림 프로세스 오류",
    "스트림 프로세스 크래시 복구 한도 초과",
    "스트림 열기 실패",
    "PyAV 사용 불가",
    "PyAV 스트림 열기 실패",
    "PyAV 비디오 스트림 없음",
    "OpenCV 사용 불가",
    "DECODER_FALLBACK_REQUIRED:",
)

_CAMERA_INTEGRITY_TERMINAL_MARKERS = (
    "INTEGRITY_RECOVERY_TIMEOUT:",
    "H264 IDR random-access recovery unavailable",
)

CAMERA_STREAM_LIFECYCLE_METHODS = [
    "toggle_camera_connection",
    "toggle_yolo_detection",
    "connect_cameras",
    "start_yolo",
    "stop_yolo_detection",
    "_resume_queued_measurement_start",
    "_start_camera_streams",
    "_camera_stream_frame_ready",
    "_camera_stream_start_status",
    "_poll_camera_start_readiness",
    "_cancel_camera_start_readiness",
    "_poll_video_start_readiness",
    "_cancel_video_start_readiness",
    "_schedule_camera_start_retry",
    "_cancel_pending_camera_start",
    "stop_yolo_streams",
    "_schedule_camera_stop_watchdog",
    "_cancel_camera_stop_watchdog",
    "_stop_camera_panes",
    "_clear_calibration_previews",
    "_clear_detection_state",
    "stop_all"
]


class CameraStreamLifecycleController:
    def __init__(self, panel):
        self.panel = panel

    def toggle_camera_connection(self, *_qt_signal_args: object) -> None:
        if (
            self.panel.camera_connected
            or self.panel._pending_camera_start is not None
            or getattr(self.panel, "_camera_start_readiness", None) is not None
        ):
            self.panel.stop_yolo_streams()
        else:
            self.panel.connect_cameras()

    def toggle_yolo_detection(self) -> bool | None:
        run_state = self._measurement_run_state()
        if run_state == "QUEUED":
            self.panel._log("[YOLO] 측정 시작 요청이 이미 대기 중입니다.")
            return None
        if self.panel.yolo_enabled or run_state in {"RUNNING", "STOPPING"}:
            self.panel.stop_yolo_detection()
        else:
            return self.panel.start_yolo()
        return None

    def connect_cameras(self) -> None:
        if self.panel.auto_wiris_connect_pending:
            self.panel._cancel_auto_wiris_connect_retry()
        self.panel._start_camera_streams(enable_detection=False, switch_to_depth=False)

    def _apply_decoder_start_policy(self) -> bool:
        backend_combo = getattr(self.panel, "stream_backend_combo", None)
        if backend_combo is not None and str(backend_combo.currentData() or "gstreamer") != "gstreamer":
            return True
        decoder_combo = getattr(self.panel, "decoder_combo", None)
        if decoder_combo is None:
            return True
        requested = str(decoder_combo.currentData() or DECODER_AUTO)
        auto_capability = current_decoder_selection(DECODER_AUTO)
        nvidia_available = (
            auto_capability.allowed
            and auto_capability.effective == NVIDIA_H264_DECODER
        )
        fallback_reasons = getattr(self.panel, "decoder_fallback_reasons", None)
        if not isinstance(fallback_reasons, dict):
            fallback_reasons = {"left": "", "right": ""}
            self.panel.decoder_fallback_reasons = fallback_reasons
        overrides = getattr(self.panel, "_runtime_decoder_overrides", None)
        if not isinstance(overrides, dict):
            overrides = {"left": None, "right": None}
            self.panel._runtime_decoder_overrides = overrides
        legacy_override = getattr(self.panel, "_runtime_decoder_override", None)
        if requested == DECODER_AUTO and legacy_override == DECODER_SOFTWARE:
            for side in ("left", "right"):
                overrides[side] = DECODER_SOFTWARE
        selection = current_decoder_selection(requested)
        if not selection.allowed:
            self.panel.decoder_selection_status = {
                "requested": selection.requested,
                "effective": selection.effective,
                "candidates": tuple(selection.candidates),
                "allowed": selection.allowed,
                "degraded": selection.degraded,
                "reason": selection.reason,
                "nvidia_available": nvidia_available,
            }
            message = (
                "NVIDIA 디코더를 시작할 수 없습니다: "
                f"{selection.reason}. 디코더를 Auto 또는 Software로 변경하고 "
                "tools/check_operator_runtime.sh --execution을 실행하세요."
            )
            self.panel._log(f"[카메라] {message}")
            label = getattr(self.panel, "camera_connection_label", None)
            if label is not None:
                label.setText(message)
            return False

        if requested != DECODER_AUTO:
            overrides.update(left=None, right=None)
            fallback_reasons.update(left="", right="")
        elif selection.effective == SOFTWARE_H264_DECODER:
            for side in ("left", "right"):
                overrides[side] = DECODER_SOFTWARE
                fallback_reasons[side] = str(selection.reason)
            if legacy_override != DECODER_SOFTWARE:
                self.panel._log(
                    f"[카메라] Auto decoder 경고: {selection.reason}; avdec_h264를 사용합니다."
                )
        elif not any(overrides.get(side) for side in ("left", "right")):
            fallback_reasons.update(left="", right="")

        overridden_sides = tuple(
            side
            for side in ("left", "right")
            if overrides.get(side) == DECODER_SOFTWARE
        )
        effective = selection.effective
        candidates = tuple(selection.candidates)
        reason = str(selection.reason or "")
        degraded = bool(selection.degraded)
        if requested == DECODER_AUTO and overridden_sides:
            effective = (
                SOFTWARE_H264_DECODER
                if len(overridden_sides) == 2
                else "per-side"
            )
            candidates = tuple(
                dict.fromkeys(
                    SOFTWARE_H264_DECODER
                    if side in overridden_sides
                    else str(selection.effective)
                    for side in ("left", "right")
                )
            )
            override_reasons = [
                f"{side}={fallback_reasons.get(side)}"
                for side in overridden_sides
                if fallback_reasons.get(side)
            ]
            reason = "; ".join(override_reasons) or "per-side runtime NVIDIA fallback override active"
            degraded = True
        self.panel.decoder_selection_status = {
            "requested": selection.requested,
            "effective": effective,
            "candidates": candidates,
            "allowed": selection.allowed,
            "degraded": degraded,
            "reason": reason,
            "nvidia_available": nvidia_available,
        }
        return True

    def _measurement_run_state(self) -> str:
        reader = getattr(self.panel, "measurement_run_snapshot", None)
        legacy = False
        if not callable(reader):
            reader = getattr(self.panel, "measurement_package_snapshot", None)
            legacy = True
        if not callable(reader):
            return "IDLE"
        try:
            snapshot = reader()
        except Exception:
            return "IDLE"
        if not isinstance(snapshot, Mapping):
            return "IDLE"
        state = str(snapshot.get("state") or "IDLE").upper()
        if legacy:
            return {
                "COLLECTING": "STARTING",
                "POST_CAPTURE": "STOPPING",
                "VERIFYING": "STOPPING",
                "SEALED_COMPLETE": "IDLE",
                "SEALED_INCOMPLETE": "IDLE",
                "RECOVERABLE": "ERROR",
            }.get(state, state)
        return state

    def _measurement_package_state(self) -> str:
        """Compatibility projection for tests/older callers."""

        return {
            "IDLE": "IDLE",
            "QUEUED": "COLLECTING",
            "STARTING": "COLLECTING",
            "RUNNING": "COLLECTING",
            "STOPPING": "POST_CAPTURE",
            "ERROR": "RECOVERABLE",
        }.get(self._measurement_run_state(), "IDLE")

    def _uses_explicit_package_lifecycle(self) -> bool:
        return bool(getattr(self.panel, "_measurement_package_explicit_lifecycle", False))

    def _prepare_package_for_detection(self, state: str) -> bool:
        """Prepare one transient run without consulting persisted evidence."""

        run_state = self._measurement_run_state()
        if run_state in {"STARTING", "RUNNING"}:
            return True
        if run_state not in {"IDLE", "QUEUED", "ERROR"}:
            return False
        starter = getattr(self.panel, "begin_measurement", None)
        if not callable(starter):
            starter = getattr(self.panel, "begin_next_measurement", None)
        if not callable(starter):
            if (
                not callable(getattr(self.panel, "measurement_run_snapshot", None))
                and not bool(
                    getattr(self.panel, "_measurement_package_explicit_lifecycle", False)
                )
            ):
                return True
            self.panel._log("[YOLO] 측정 시작 owner를 사용할 수 없습니다.")
            return False
        try:
            started = starter()
        except Exception as exc:
            self.panel._log(f"[YOLO] 새 Measurement 생성 실패: {exc}")
            return False
        if started is False:
            reader = getattr(self.panel, "measurement_run_snapshot", None)
            try:
                snapshot = reader() if callable(reader) else {}
            except Exception:
                snapshot = {}
            detail = (
                str(snapshot.get("error") or snapshot.get("detail") or "")
                if isinstance(snapshot, Mapping)
                else ""
            )
            suffix = f" · {detail}" if detail else ""
            self.panel._log(f"[YOLO] Measurement 시작 준비가 거부되었습니다{suffix}")
            return False
        return self._measurement_run_state() == "STARTING"

    def _reserve_package_detection(self) -> bool:
        if not self._prepare_package_for_detection(self._measurement_package_state()):
            return False
        if not callable(getattr(self.panel, "measurement_run_snapshot", None)):
            starter = getattr(self.panel, "begin_measurement_detection", None)
            if callable(starter):
                return bool(starter())
        return True

    def _complete_package_detection(self) -> bool:
        state = self._measurement_run_state()
        if state not in {"STARTING", "RUNNING", "ERROR", "STOPPING"}:
            return True
        completer = getattr(self.panel, "request_measurement_stop", None)
        if not callable(completer):
            completer = getattr(self.panel, "complete_measurement_detection", None)
        return True if not callable(completer) else bool(completer())

    def _camera_busy_reason(self) -> str:
        busy_threads = (
            ("pnp_photo_capture_thread", "HR capture is using the cameras"),
            ("oz_photo_capture_thread", "OZ capture is using the cameras"),
            ("optical_pose_wiris_ftp_thread", "Alignment HR capture is using the cameras"),
            ("optical_pose_wiris_ftp_oz_thread", "Alignment OZ capture is using the cameras"),
            ("calibration_wiris_ftp_thread", "Calibration HR capture is using the cameras"),
            ("calibration_wiris_ftp_oz_thread", "Calibration OZ download is using the cameras"),
            ("calibration_wiris_oz_capture_thread", "Calibration OZ capture is using the cameras"),
            ("calibration_wiris_oz_sync_thread", "Calibration camera synchronization is running"),
            ("calibration_wiris_oz_delete_thread", "Calibration camera cleanup is running"),
            ("calibration_wiris_oz_format_thread", "Calibration image-format change is running"),
            ("calibration_wiris_oz_gilc_thread", "Calibration GILC operation is running"),
            ("wiris_time_sync_thread", "camera time synchronization is running"),
            ("wiris_image_format_thread", "camera image-format change is running"),
            ("wiris_command_thread", "a camera command is running"),
            ("wiris_zoom_thread", "camera zoom is changing"),
            ("wiris_stip_thread", "camera STIP configuration is running"),
            ("wiris_lrf_thread", "camera LRF query is running"),
            ("camera_auto_detect_thread", "camera discovery is running"),
            ("rtsp_check_thread", "camera RTSP check is running"),
            ("rtsp_enable_thread", "camera RTSP activation is running"),
        )
        for attribute, reason in busy_threads:
            if getattr(self.panel, attribute, None) is not None:
                return reason
        stus_operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        if isinstance(stus_operation, Mapping) and bool(stus_operation.get("active")):
            return (
                "camera time synchronization closed-loop is running "
                f"(attempt {stus_operation.get('attempts_started', 1)}/"
                f"{stus_operation.get('max_attempts', 2)})"
            )
        if getattr(self.panel, "_rtsp_reactivation_side", None) is not None:
            return "camera RTSP reactivation is running"
        if bool(getattr(self.panel, "auto_wiris_connect_pending", False)):
            return "camera auto-connect is running"
        return ""

    def _queue_measurement_start(self, reason: str) -> None:
        queue_start = getattr(self.panel, "queue_measurement_start", None)
        if callable(queue_start):
            queue_start(reason)
        self.panel._pending_measurement_run_start = True

        def retry() -> None:
            if bool(getattr(self.panel, "_shutdown_in_progress", False)):
                self.panel._pending_measurement_run_start = False
                return
            if not bool(getattr(self.panel, "_pending_measurement_run_start", False)):
                return
            if self._camera_busy_reason() or bool(
                getattr(self.panel, "_manual_recording_active", False)
            ):
                QtCore.QTimer.singleShot(100, retry)
                return
            self.panel._pending_measurement_run_start = False
            self.panel.start_yolo()

        QtCore.QTimer.singleShot(100, retry)

    def _resume_queued_measurement_start(self) -> bool:
        if not bool(getattr(self.panel, "_pending_measurement_run_start", False)):
            return False
        if self._camera_busy_reason() or bool(
            getattr(self.panel, "_manual_recording_active", False)
        ):
            return False
        self.panel._pending_measurement_run_start = False
        QtCore.QTimer.singleShot(0, self.panel.start_yolo)
        return True

    def _detection_inputs_valid(self) -> bool:
        if not hasattr(self.panel, "left_url_edit") or not hasattr(
            self.panel, "right_url_edit"
        ):
            return True
        left_url = self.panel.left_url_edit.text().strip()
        right_url = self.panel.right_url_edit.text().strip()
        if not left_url or not right_url:
            self.panel._log("[카메라] Left/Right RTSP URL을 입력해야 합니다.")
            return False
        if self.panel._rtsp_urls_are_same_stream():
            message = "카메라: 좌/우 RTSP URL이 같은 스트림입니다. 좌우 카메라 IP를 분리하세요."
            self.panel.camera_connection_label.setText(message)
            self.panel._set_distance_text("거리: 좌/우 RTSP URL이 동일해 측정 차단", quality="ERROR")
            self.panel._log(f"[카메라] {message}")
            return False
        return True

    def start_yolo(self) -> bool | None:
        if bool(getattr(self.panel, "_shutdown_in_progress", False)):
            self.panel._log("[YOLO] application shutdown is in progress; Measurement start rejected")
            return False
        if not self._detection_inputs_valid():
            return False
        if bool(getattr(self.panel, "_manual_recording_active", False)):
            self._queue_measurement_start("stopping Manual record")
            stopper = getattr(self.panel, "_stop_session_recording", None)
            if callable(stopper):
                stopper(restart_streams=False)
            return None
        busy_reason = self._camera_busy_reason()
        if busy_reason:
            self._queue_measurement_start(busy_reason)
            return None
        if not self._reserve_package_detection():
            self.panel._log("[YOLO] 측정 시작 준비가 거부되었습니다: live run state를 확인하세요.")
            return False
        if not self.panel._session_recording_active():
            try:
                started = self.panel.start_automatic_session_recording()
            except Exception as exc:
                self.panel._log(f"[저장] 자동 데이터 기록 준비 실패: {exc}")
                cancel = getattr(self.panel, "cancel_measurement_detection_start", None)
                if callable(cancel):
                    cancel(str(exc))
                return False
            if started is False:
                cancel = getattr(self.panel, "cancel_measurement_detection_start", None)
                if callable(cancel):
                    cancel("measurement recording could not start")
                return False
        stream_result = self.panel._start_camera_streams(
            enable_detection=True, switch_to_depth=True
        )
        if stream_result is False:
            self._rollback_camera_start(
                "camera/decoder start request was rejected",
                enable_detection=True,
            )
            return False
        return None

    def stop_yolo_detection(self) -> None:
        if (
            self.panel._pending_camera_start is not None
            or getattr(self.panel, "_camera_start_readiness", None) is not None
        ):
            cancel = getattr(self.panel, "cancel_measurement_detection_start", None)
            cancelled = bool(cancel("Detection camera start was cancelled")) if callable(cancel) else False
            if not cancelled:
                self._complete_package_detection()
            self.panel.stop_yolo_streams()
            return
        if not self.panel.camera_connected:
            self.panel._log("[YOLO] 연결된 카메라 스트림이 없어 중지할 YOLO가 없습니다.")
            return
        if not self.panel.yolo_enabled:
            self.panel._log("[YOLO] 이미 중지 상태입니다. 카메라 프리뷰는 유지 중입니다.")
            return
        self.panel._log("[YOLO] 검출을 중지하고 카메라 프리뷰만 유지합니다.")
        self._complete_package_detection()
        if bool(getattr(self.panel, "_automatic_session_recording", False)):
            self.panel.stop_yolo_streams()
            if self.panel._session_recording_active():
                return
            self.panel.connect_cameras()
            return
        self.panel._start_camera_streams(enable_detection=False, switch_to_depth=False)

    def _set_video_start_rows(
        self,
        rows: Mapping[str, Mapping[str, object]],
        *,
        detail: str,
    ) -> dict[str, object]:
        setter = getattr(self.panel, "_set_video_recording_start_snapshot", None)
        if callable(setter):
            value = setter(rows, detail=detail)
            return dict(value) if isinstance(value, Mapping) else {}
        snapshot = {
            "schema": "workstation-video-start-v1",
            "state": "STARTING",
            "detail": str(detail),
            "sides": {side: dict(row) for side, row in rows.items()},
        }
        self.panel._video_recording_start_snapshot = snapshot
        return snapshot

    def _cancel_video_start_readiness(self, *, mark_idle: bool = False) -> None:
        self.panel._video_start_readiness_generation = int(
            getattr(self.panel, "_video_start_readiness_generation", 0)
        ) + 1
        self.panel._video_start_readiness = None
        if mark_idle:
            self._set_video_start_rows({}, detail="video recording stopped")

    def _begin_video_start_readiness(self, started_sides: tuple[str, ...]) -> None:
        self._cancel_video_start_readiness(mark_idle=False)
        if not bool(getattr(self.panel, "video_recording_enabled", False)):
            return
        paths = dict(getattr(self.panel, "video_recording_paths", {}) or {})
        baselines = dict(
            getattr(self.panel, "_video_recording_allocation_sizes", {}) or {}
        )
        rows: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            raw_path = paths.get(side)
            if raw_path is None:
                rows[side] = {
                    "state": "FAILED",
                    "path": "",
                    "size_bytes": 0,
                    "detail": "video path was not allocated",
                }
                continue
            path = Path(raw_path).expanduser()
            baseline = max(0, int(baselines.get(side, 0) or 0))
            if side not in started_sides:
                rows[side] = {
                    "state": "FAILED",
                    "path": str(path),
                    "size_bytes": baseline,
                    "detail": "camera worker did not accept start",
                }
                continue
            rows[side] = {
                "state": "STARTING",
                "path": str(path),
                "size_bytes": baseline,
                "detail": "waiting for positive MP4 byte growth",
            }
        generation = int(
            getattr(self.panel, "_video_start_readiness_generation", 0)
        ) + 1
        self.panel._video_start_readiness_generation = generation
        self.panel._video_start_readiness = {
            "generation": generation,
            "deadline_monotonic_s": time.monotonic()
            + VIDEO_START_READINESS_TIMEOUT_S,
            "baseline_sizes": {
                side: max(0, int(baselines.get(side, 0) or 0))
                for side in ("left", "right")
            },
            "rows": rows,
        }
        self._set_video_start_rows(
            rows,
            detail="camera workers accepted; waiting for MP4 file growth",
        )
        if any(row["state"] == "STARTING" for row in rows.values()):
            QtCore.QTimer.singleShot(
                VIDEO_START_READINESS_POLL_MS,
                lambda generation=generation: self._poll_video_start_readiness(
                    generation
                ),
            )

    def _poll_video_start_readiness(self, generation: int) -> None:
        readiness = getattr(self.panel, "_video_start_readiness", None)
        if not isinstance(readiness, dict) or int(
            readiness.get("generation", -1)
        ) != int(generation):
            return
        rows = {
            side: dict(row)
            for side, row in dict(readiness.get("rows") or {}).items()
            if isinstance(row, Mapping)
        }
        baselines = dict(readiness.get("baseline_sizes") or {})
        for side, row in rows.items():
            if row.get("state") != "STARTING":
                continue
            path = Path(str(row.get("path") or "")).expanduser()
            try:
                size_bytes = path.stat().st_size if path.is_file() else 0
            except OSError as exc:
                row["detail"] = f"video path stat failed: {exc}"
                size_bytes = 0
            row["size_bytes"] = max(0, int(size_bytes))
            if size_bytes > max(0, int(baselines.get(side, 0) or 0)):
                row["state"] = "RUNNING"
                row["detail"] = "positive MP4 byte growth observed"
        timed_out = time.monotonic() >= float(
            readiness.get("deadline_monotonic_s") or 0.0
        )
        if timed_out:
            for row in rows.values():
                if row.get("state") == "STARTING":
                    row["state"] = "FAILED"
                    row["detail"] = (
                        "no positive MP4 byte growth within "
                        f"{VIDEO_START_READINESS_TIMEOUT_S:.1f}s"
                    )
        readiness["rows"] = rows
        pending = any(row.get("state") == "STARTING" for row in rows.values())
        if pending:
            self._set_video_start_rows(
                rows,
                detail="waiting for MP4 file growth",
            )
            QtCore.QTimer.singleShot(
                VIDEO_START_READINESS_POLL_MS,
                lambda generation=generation: self._poll_video_start_readiness(
                    generation
                ),
            )
            return
        snapshot = self._set_video_start_rows(
            rows,
            detail=(
                "video recording readiness timeout"
                if timed_out
                else "video recording file growth verified"
            ),
        )
        self.panel._video_start_readiness = None
        state = str(snapshot.get("state") or "FAILED")
        if state in {"FAILED", "PARTIAL"}:
            self.panel._log(
                f"[저장] 영상 녹화 시작 상태: {state}; "
                "Measurement는 현재 frame으로 계속합니다."
            )
        self.panel._refresh_operation_header()

    def _mark_video_start_side_failed(self, side: str, reason: str) -> None:
        readiness = getattr(self.panel, "_video_start_readiness", None)
        if not isinstance(readiness, dict):
            return
        rows = readiness.get("rows")
        if not isinstance(rows, dict) or side not in rows:
            return
        row = dict(rows[side])
        if row.get("state") == "RUNNING":
            return
        row["state"] = "FAILED"
        row["detail"] = str(reason)
        rows[side] = row
        self._poll_video_start_readiness(int(readiness["generation"]))

    def _cancel_camera_start_readiness(self) -> None:
        self.panel._camera_start_readiness_generation = int(
            getattr(self.panel, "_camera_start_readiness_generation", 0)
        ) + 1
        self.panel._camera_start_readiness = None

    def _begin_camera_start_readiness(
        self,
        started_sides: tuple[str, ...],
        *,
        enable_detection: bool,
        switch_to_depth: bool,
    ) -> None:
        self._cancel_camera_start_readiness()
        generation = int(
            getattr(self.panel, "_camera_start_readiness_generation", 0)
        ) + 1
        self.panel._camera_start_readiness_generation = generation
        expected_stream_generations = {
            side: int(getattr(getattr(self.panel, f"{side}_pane"), "stream_generation", 0))
            for side in started_sides
        }
        unavailable = tuple(side for side in ("left", "right") if side not in started_sides)
        self.panel._camera_start_readiness = {
            "generation": generation,
            "started_sides": tuple(started_sides),
            "expected_stream_generations": expected_stream_generations,
            "frame_ready_sides": set(),
            "failed_sides": {
                side: "camera worker did not accept start" for side in unavailable
            },
            "enable_detection": bool(enable_detection),
            "switch_to_depth": bool(switch_to_depth),
            "deadline_monotonic_s": time.monotonic()
            + CAMERA_START_READINESS_TIMEOUT_S,
        }
        waiting = ", ".join(started_sides)
        suffix = f"; unavailable={','.join(unavailable)}" if unavailable else ""
        self.panel.camera_connection_label.setText(
            f"camera STARTING: first decoded frame ({waiting}){suffix}"
        )
        self._begin_video_start_readiness(tuple(started_sides))
        QtCore.QTimer.singleShot(
            CAMERA_START_READINESS_POLL_MS,
            lambda generation=generation: self._poll_camera_start_readiness(
                generation
            ),
        )

    def _camera_stream_frame_ready(
        self,
        camera: str,
        _image: object,
        metadata: object,
    ) -> None:
        frame_metadata = dict(metadata) if isinstance(metadata, Mapping) else {}
        try:
            observed_generation = int(frame_metadata.get("stream_generation"))
        except (TypeError, ValueError):
            return
        pane = getattr(self.panel, f"{camera}_pane", None)
        current_generation = int(getattr(pane, "stream_generation", -1))
        if observed_generation != current_generation:
            return
        unavailable_sides = set(
            getattr(self.panel, "_camera_integrity_unavailable_sides", ()) or ()
        )
        integrity_recovered = False
        if camera in unavailable_sides:
            failed_generations = getattr(
                self.panel,
                "_camera_integrity_failed_generations",
                {},
            )
            try:
                failed_generation = int(
                    dict(failed_generations or {})[camera]
                )
            except (KeyError, TypeError, ValueError):
                # An unavailable latch without provenance never clears on a
                # frame. Recovery requires a tracked newer generation.
                return
            if observed_generation <= failed_generation:
                return
            unavailable_sides.discard(camera)
            self.panel._camera_integrity_unavailable_sides = unavailable_sides
            if isinstance(failed_generations, dict):
                failed_generations.pop(camera, None)
            integrity_recovered = True

        # Decoder selection/status is launch intent only. The first decoded
        # current-generation frame is the boundary ACK that makes a decoder
        # active, including a same-generation preview-only CPU fallback.
        active_decoder = str(frame_metadata.get("active_decoder") or "").strip()
        setter = getattr(self.panel, "_set_active_stream_decoder", None)
        if active_decoder and callable(setter):
            setter(camera, active_decoder)

        readiness = getattr(self.panel, "_camera_start_readiness", None)
        if not isinstance(readiness, dict) or camera not in readiness.get(
            "started_sides", ()
        ):
            if integrity_recovered:
                self._mark_live_camera_side_recovered(camera)
            return
        if camera in dict(readiness.get("failed_sides") or {}):
            return
        try:
            expected_generation = int(
                dict(readiness.get("expected_stream_generations") or {})[camera]
            )
        except (KeyError, TypeError, ValueError):
            return
        if observed_generation != expected_generation:
            return
        ready = readiness.get("frame_ready_sides")
        if not isinstance(ready, set):
            ready = set(ready or ())
            readiness["frame_ready_sides"] = ready
        ready.add(camera)
        self._evaluate_camera_start_readiness(readiness)

    def _camera_stream_start_status(self, camera: str, status: str) -> None:
        text = str(status)
        if any(marker in text for marker in _VIDEO_START_FAILURE_MARKERS):
            self._mark_video_start_side_failed(camera, text)
        readiness = getattr(self.panel, "_camera_start_readiness", None)
        if any(marker in text for marker in _CAMERA_INTEGRITY_TERMINAL_MARKERS):
            if isinstance(readiness, dict) and camera in readiness.get(
                "started_sides", ()
            ):
                # This also handles a side that supplied its first IDR while
                # its peer was still pending; it must no longer count ready.
                unavailable = set(
                    getattr(
                        self.panel,
                        "_camera_integrity_unavailable_sides",
                        (),
                    )
                    or ()
                )
                unavailable.add(camera)
                self.panel._camera_integrity_unavailable_sides = unavailable
                pane = getattr(self.panel, f"{camera}_pane", None)
                failed_generations = getattr(
                    self.panel,
                    "_camera_integrity_failed_generations",
                    None,
                )
                if not isinstance(failed_generations, dict):
                    failed_generations = {}
                    self.panel._camera_integrity_failed_generations = failed_generations
                failed_generations[camera] = int(
                    getattr(pane, "stream_generation", -1)
                )
                self._mark_camera_start_side_failed(camera, text)
                return
            self._mark_live_camera_side_unavailable(camera, text)
            return
        if not isinstance(readiness, dict) or camera not in readiness.get(
            "started_sides", ()
        ):
            return
        ready = set(readiness.get("frame_ready_sides") or ())
        if camera in ready:
            return
        if any(marker in text for marker in _CAMERA_START_FAILURE_MARKERS):
            self._mark_camera_start_side_failed(camera, text)

    def _mark_live_camera_side_unavailable(self, camera: str, reason: str) -> None:
        """Retire one integrity-failed side without disturbing its peer."""

        if camera not in {"left", "right"}:
            return
        unavailable = set(
            getattr(self.panel, "_camera_integrity_unavailable_sides", ()) or ()
        )
        if camera in unavailable:
            return
        unavailable.add(camera)
        self.panel._camera_integrity_unavailable_sides = unavailable

        pane = getattr(self.panel, f"{camera}_pane", None)
        failed_generations = getattr(
            self.panel,
            "_camera_integrity_failed_generations",
            None,
        )
        if not isinstance(failed_generations, dict):
            failed_generations = {}
            self.panel._camera_integrity_failed_generations = failed_generations
        failed_generations[camera] = int(getattr(pane, "stream_generation", -1))
        stopper = getattr(pane, "stop", None)
        if callable(stopper):
            stopper(wait=False)

        active_decoders = getattr(self.panel, "active_stream_decoders", None)
        if isinstance(active_decoders, dict):
            active_decoders[camera] = None
        frame_times = getattr(self.panel, "camera_frame_times", None)
        if isinstance(frame_times, dict):
            values = frame_times.get(camera)
            clear = getattr(values, "clear", None)
            if callable(clear):
                clear()
        frame_metadata = getattr(self.panel, "camera_frame_metadata", None)
        if isinstance(frame_metadata, dict):
            frame_metadata[camera] = {}
        metadata_history = getattr(self.panel, "camera_frame_metadata_history", None)
        if isinstance(metadata_history, dict):
            values = metadata_history.get(camera)
            clear = getattr(values, "clear", None)
            if callable(clear):
                clear()
        detection_buffers = getattr(self.panel, "detection_buffers", None)
        if isinstance(detection_buffers, dict):
            values = detection_buffers.get(camera)
            clear = getattr(values, "clear", None)
            if callable(clear):
                clear()
        setattr(self.panel, f"latest_{camera}", None)
        for cache_name in (
            "latest_calibration_preview_images",
            "calibration_preview_overlay_images",
            "calibration_preview_detection_results",
        ):
            cache = getattr(self.panel, cache_name, None)
            if isinstance(cache, dict):
                cache.pop(camera, None)
        preview_request_ids = getattr(
            self.panel,
            "calibration_preview_request_ids",
            None,
        )
        if isinstance(preview_request_ids, dict):
            # Invalidate any queued checkerboard result for this generation.
            preview_request_ids[camera] = 0
        calibration_preview = getattr(
            self.panel,
            f"calibration_{camera}_preview",
            None,
        )
        set_pixmap = getattr(calibration_preview, "setPixmap", None)
        if callable(set_pixmap):
            set_pixmap(QtGui.QPixmap())
        set_preview_text = getattr(calibration_preview, "setText", None)
        if callable(set_preview_text):
            set_preview_text(f"{camera} unavailable (H264 IDR integrity)")

        active_sides = tuple(
            side
            for side in ("left", "right")
            if side not in unavailable
            and (
                getattr(getattr(self.panel, f"{side}_pane", None), "worker", None)
                is not None
                or getattr(getattr(self.panel, f"{side}_pane", None), "thread", None)
                is not None
            )
        )
        self.panel.camera_connected = bool(active_sides)
        self.panel.yolo_enabled = bool(
            active_sides
            and any(
                bool(
                    getattr(
                        getattr(self.panel, f"{side}_pane", None),
                        "detection_stream_active",
                        False,
                    )
                )
                for side in active_sides
            )
        )

        current_measurement_sides = tuple(
            getattr(self.panel, "_measurement_run_active_sides", ()) or ()
        )
        if current_measurement_sides:
            remaining_measurement_sides = tuple(
                side for side in current_measurement_sides if side != camera
            )
            self.panel._measurement_run_active_sides = remaining_measurement_sides
            owner = getattr(self.panel, "_measurement_run_controller_owner", None)
            if owner is not None and getattr(owner, "state", "") == "RUNNING":
                set_state = getattr(owner, "_set_state", None)
                if callable(set_state):
                    if remaining_measurement_sides:
                        set_state(
                            "RUNNING",
                            "Measurement partial: "
                            f"{remaining_measurement_sides[0]} only; "
                            f"{camera} integrity unavailable",
                        )
                    else:
                        set_state(
                            "ERROR",
                            "Measurement camera unavailable",
                            str(reason),
                        )

        clear_pair = getattr(self.panel, "_clear_measurement_pair_display", None)
        if callable(clear_pair):
            clear_pair()
        refresh_distance = getattr(self.panel, "_refresh_distance", None)
        if callable(refresh_distance):
            refresh_distance()
        label = getattr(self.panel, "camera_connection_label", None)
        set_text = getattr(label, "setText", None)
        if callable(set_text):
            if active_sides:
                set_text(
                    f"camera partial: {', '.join(active_sides)} LIVE; "
                    f"{camera} unavailable (H264 IDR integrity)"
                )
            else:
                set_text(
                    "camera unavailable: no verified H264 IDR random-access side"
                )
        log = getattr(self.panel, "_log", None)
        if callable(log):
            log(
                f"[카메라] {camera} integrity unavailable; "
                f"opposite side preserved={bool(active_sides)}; {reason}"
            )
        for refresh_name in (
            "_update_runtime_camera_stats_labels",
            "_refresh_session_toggle_buttons",
            "_refresh_operation_header",
        ):
            refresh = getattr(self.panel, refresh_name, None)
            if callable(refresh):
                refresh()

    def _mark_live_camera_side_recovered(self, camera: str) -> None:
        """Publish recovery only after a newer generation's verified frame."""

        unavailable = set(
            getattr(self.panel, "_camera_integrity_unavailable_sides", ()) or ()
        )
        active_sides = tuple(
            side
            for side in ("left", "right")
            if side not in unavailable
            and (
                getattr(getattr(self.panel, f"{side}_pane", None), "worker", None)
                is not None
                or getattr(getattr(self.panel, f"{side}_pane", None), "thread", None)
                is not None
            )
        )
        self.panel.camera_connected = bool(active_sides)
        self.panel.yolo_enabled = bool(
            active_sides
            and any(
                bool(
                    getattr(
                        getattr(self.panel, f"{side}_pane", None),
                        "detection_stream_active",
                        False,
                    )
                )
                for side in active_sides
            )
        )
        label = getattr(self.panel, "camera_connection_label", None)
        set_text = getattr(label, "setText", None)
        if callable(set_text):
            if len(active_sides) == 2:
                set_text("camera LIVE: left/right current frame verified")
            else:
                peer = "right" if active_sides == ("left",) else "left"
                set_text(
                    f"camera partial: {', '.join(active_sides)} LIVE; "
                    f"{peer} unavailable"
                )
        log = getattr(self.panel, "_log", None)
        if callable(log):
            log(
                f"[카메라] {camera} newer-generation H264 IDR/exact-PTS frame verified"
            )
        for refresh_name in (
            "_update_runtime_camera_stats_labels",
            "_refresh_session_toggle_buttons",
            "_refresh_operation_header",
        ):
            refresh = getattr(self.panel, refresh_name, None)
            if callable(refresh):
                refresh()

    def _mark_camera_start_side_failed(self, camera: str, reason: str) -> None:
        readiness = getattr(self.panel, "_camera_start_readiness", None)
        if not isinstance(readiness, dict) or camera not in readiness.get(
            "started_sides", ()
        ):
            return
        failed = readiness.get("failed_sides")
        if not isinstance(failed, dict):
            failed = {}
            readiness["failed_sides"] = failed
        if camera in failed:
            return
        failed[camera] = str(reason)
        self._mark_video_start_side_failed(camera, reason)
        pane = getattr(self.panel, f"{camera}_pane", None)
        stopper = getattr(pane, "stop", None)
        if callable(stopper):
            stopper(wait=False)
        self._evaluate_camera_start_readiness(readiness)

    def _evaluate_camera_start_readiness(self, readiness: dict[str, object]) -> None:
        if readiness is not getattr(self.panel, "_camera_start_readiness", None):
            return
        started = tuple(str(side) for side in readiness.get("started_sides", ()))
        failed = dict(readiness.get("failed_sides") or {})
        ready = tuple(
            side
            for side in ("left", "right")
            if side in set(readiness.get("frame_ready_sides") or ())
            and side in started
            and side not in failed
        )
        pending = tuple(
            side for side in started if side not in ready and side not in failed
        )
        if pending:
            return
        if ready:
            self._confirm_camera_start_readiness(readiness, ready)
            return
        details = "; ".join(f"{side}: {reason}" for side, reason in failed.items())
        self._rollback_camera_start(
            "no current-generation first decoded frame"
            + (f" ({details})" if details else ""),
            enable_detection=bool(readiness.get("enable_detection", False)),
        )

    def _confirm_camera_start_readiness(
        self,
        readiness: dict[str, object],
        ready_sides: tuple[str, ...],
    ) -> None:
        failed = dict(readiness.get("failed_sides") or {})
        enable_detection = bool(readiness.get("enable_detection", False))
        self._cancel_camera_start_readiness()
        self.panel.camera_connected = True
        self.panel.yolo_enabled = enable_detection
        self.panel.start_button.setEnabled(True)
        self.panel.camera_connect_button.setEnabled(True)
        if len(ready_sides) == 2:
            self.panel.camera_connection_label.setText(
                "camera LIVE: left/right current frame verified"
            )
        else:
            failed_text = ", ".join(failed) or (
                "right" if ready_sides == ("left",) else "left"
            )
            self.panel.camera_connection_label.setText(
                f"camera partial: {', '.join(ready_sides)} LIVE; "
                f"{failed_text} unavailable"
            )
            self.panel._log(
                f"[카메라] current-generation frame 확인: "
                f"{', '.join(ready_sides)}; failed={failed_text}"
            )
        self.panel._refresh_session_toggle_buttons()
        self.panel._refresh_operation_header()
        if enable_detection:
            confirmer = getattr(self.panel, "confirm_measurement_running", None)
            if callable(confirmer):
                confirmer(ready_sides)

    def _poll_camera_start_readiness(self, generation: int) -> None:
        readiness = getattr(self.panel, "_camera_start_readiness", None)
        if not isinstance(readiness, dict) or int(
            readiness.get("generation", -1)
        ) != int(generation):
            return
        self._evaluate_camera_start_readiness(readiness)
        if readiness is not getattr(self.panel, "_camera_start_readiness", None):
            return
        if time.monotonic() >= float(
            readiness.get("deadline_monotonic_s") or 0.0
        ):
            ready = set(readiness.get("frame_ready_sides") or ())
            failed = readiness.get("failed_sides")
            if not isinstance(failed, dict):
                failed = {}
                readiness["failed_sides"] = failed
            for side in tuple(readiness.get("started_sides", ())):
                if side not in ready and side not in failed:
                    failed[side] = (
                        "first decoded frame timeout after "
                        f"{CAMERA_START_READINESS_TIMEOUT_S:.1f}s"
                    )
                    self._mark_video_start_side_failed(side, failed[side])
                    pane = getattr(self.panel, f"{side}_pane", None)
                    stopper = getattr(pane, "stop", None)
                    if callable(stopper):
                        stopper(wait=False)
            self._evaluate_camera_start_readiness(readiness)
            return
        QtCore.QTimer.singleShot(
            CAMERA_START_READINESS_POLL_MS,
            lambda generation=generation: self._poll_camera_start_readiness(
                generation
            ),
        )

    def _rollback_camera_start(
        self,
        reason: str,
        *,
        enable_detection: bool,
    ) -> None:
        failure = str(reason or "camera start failed")
        self._cancel_camera_start_readiness()
        video = getattr(self.panel, "_video_start_readiness", None)
        if isinstance(video, dict):
            rows = video.get("rows")
            if isinstance(rows, dict):
                for side, raw in tuple(rows.items()):
                    row = dict(raw) if isinstance(raw, Mapping) else {}
                    if row.get("state") != "RUNNING":
                        row["state"] = "FAILED"
                        row["detail"] = failure
                        rows[side] = row
                self._set_video_start_rows(rows, detail=failure)
        else:
            reader = getattr(self.panel, "video_recording_start_snapshot", None)
            snapshot = reader() if callable(reader) else {}
            snapshot_rows = (
                dict(snapshot.get("sides") or {})
                if isinstance(snapshot, Mapping)
                else {}
            )
            snapshot_state = (
                str(snapshot.get("state") or "")
                if isinstance(snapshot, Mapping)
                else ""
            )
            if snapshot_state == "STARTING":
                failed_rows: dict[str, dict[str, object]] = {}
                for side, raw in snapshot_rows.items():
                    row = dict(raw) if isinstance(raw, Mapping) else {}
                    if row.get("state") != "RUNNING":
                        row["state"] = "FAILED"
                        row["detail"] = failure
                    failed_rows[str(side)] = row
                self._set_video_start_rows(failed_rows, detail=failure)
        self._cancel_video_start_readiness(mark_idle=False)
        stopper = getattr(self.panel, "stop_yolo_streams", None)
        if callable(stopper):
            try:
                stopper(wait=False)
            except TypeError:
                stopper()
        if bool(getattr(self.panel, "_automatic_session_recording", False)):
            stop_recording = getattr(
                self.panel, "stop_automatic_session_recording", None
            )
            if callable(stop_recording):
                stop_recording(restart_streams=False)
        if enable_detection:
            fail = getattr(self.panel, "fail_measurement_start", None)
            if callable(fail):
                fail(failure)
            else:
                cancel = getattr(
                    self.panel, "cancel_measurement_detection_start", None
                )
                if callable(cancel):
                    cancel(failure)
        self.panel.camera_connected = False
        self.panel.yolo_enabled = False
        for attribute in ("start_button", "camera_connect_button"):
            setter = getattr(getattr(self.panel, attribute, None), "setEnabled", None)
            if callable(setter):
                setter(True)
        set_text = getattr(
            getattr(self.panel, "camera_connection_label", None), "setText", None
        )
        if callable(set_text):
            set_text(f"camera ERROR: {failure}")
        self.panel._log(f"[카메라] 시작 실패: {failure}")
        refresh = getattr(self.panel, "_refresh_session_toggle_buttons", None)
        if callable(refresh):
            refresh()
        refresh = getattr(self.panel, "_refresh_operation_header", None)
        if callable(refresh):
            refresh()

    def _start_camera_streams(self, *, enable_detection: bool, switch_to_depth: bool) -> bool:
        # A leftover stop watchdog from a prior disconnect must not call
        # pane.stop() again once this start attempt owns the pane's teardown
        # (via _stop_camera_panes below and, if needed, its own start retry).
        self._cancel_camera_stop_watchdog()
        self._cancel_camera_start_readiness()
        self._cancel_video_start_readiness(mark_idle=False)
        if not self._apply_decoder_start_policy():
            return False
        kwargs = self.panel.yolo_kwargs()
        left_url = self.panel.left_url_edit.text().strip()
        right_url = self.panel.right_url_edit.text().strip()
        if not left_url or not right_url:
            self.panel._log("[카메라] Left/Right RTSP URL을 입력해야 합니다.")
            return False
        if self.panel._rtsp_urls_are_same_stream():
            message = "카메라: 좌/우 RTSP URL이 같은 스트림입니다. 좌우 카메라 IP를 분리하세요."
            self.panel.camera_connection_label.setText(message)
            self.panel._set_distance_text("거리: 좌/우 RTSP URL이 동일해 측정 차단", quality="ERROR")
            self.panel._log(f"[카메라] {message}")
            return False
        self.panel._log("[연결] 카메라 프리뷰/YOLO는 설정된 인터페이스와 RTSP URL을 사용합니다.")
        was_camera_connected = bool(self.panel.camera_connected)
        self.panel.start_button.setEnabled(False)
        self.panel.camera_connect_button.setEnabled(False)
        # QThread/process creation is request acceptance only.  These public
        # states become true after a current-generation decoded frame ACK.
        self.panel.camera_connected = False
        self.panel.yolo_enabled = False
        if switch_to_depth:
            self.panel.tabs.setCurrentIndex(self.panel.depth_tab_index)
        if enable_detection:
            message = "[YOLO] 좌/우 WIRIS 스트림을 YOLO Depth 측정 모드로 시작합니다."
        else:
            message = "[카메라] 좌/우 WIRIS RTSP 프리뷰 스트림을 시작합니다."
        if self.panel.camera_sync_status not in {
            CAMERA_SYNC_STATUS_RECONNECTING,
            CAMERA_SYNC_STATUS_WARMING,
        }:
            self.panel._camera_sync_transport_override = None
            self.panel._set_camera_sync_state(
                CAMERA_SYNC_STATUS_NOT_VERIFIED,
                "새 스트림은 STUS 후 RTCP/NTP 검증이 필요합니다.",
            )
        if was_camera_connected:
            message += " 기존 스트림은 재시작합니다."
        self.panel._log(message)
        panes_stopped = self.panel._stop_camera_panes(clear_previews=False)
        self.panel.active_stream_decoders["left"] = None
        self.panel.active_stream_decoders["right"] = None
        self.panel._clear_detection_state()
        # Restarting the stream re-establishes the RTCP Sender Report anchor, so
        # samples taken against the previous anchor are a different series and
        # would also hold the y axis at the pre-resync scale.
        if hasattr(self.panel, "clock_offset_plot"):
            self.panel.clock_offset_plot.clear()
            self.panel._last_clock_offset_sample_s = 0.0
        if enable_detection:
            self.panel._clear_calibration_previews("YOLO depth mode")
        if not panes_stopped:
            self.panel.camera_connected = False
            self.panel.yolo_enabled = False
            self.panel._schedule_camera_start_retry(enable_detection, switch_to_depth)
            return True

        def start_both() -> bool:
            decoder_overrides = getattr(
                self.panel,
                "_runtime_decoder_overrides",
                {},
            )
            legacy_override = getattr(self.panel, "_runtime_decoder_override", None)
            left_kwargs = dict(kwargs)
            right_kwargs = dict(kwargs)
            for side, side_kwargs in (
                ("left", left_kwargs),
                ("right", right_kwargs),
            ):
                override = (
                    decoder_overrides.get(side)
                    if isinstance(decoder_overrides, Mapping)
                    else None
                ) or legacy_override
                if override:
                    side_kwargs["decoder"] = str(override)
            left_started = self.panel.left_pane.start_stream(
                left_url,
                enable_detection=enable_detection,
                **left_kwargs,
                **self.panel._video_recording_kwargs("left"),
            )
            right_started = self.panel.right_pane.start_stream(
                right_url,
                enable_detection=enable_detection,
                **right_kwargs,
                **self.panel._video_recording_kwargs("right"),
            )

            started_sides = tuple(
                side for side, started in (("left", left_started), ("right", right_started)) if started
            )
            if not started_sides:
                self.panel.camera_connected = False
                self.panel.yolo_enabled = False
                self.panel._schedule_camera_start_retry(enable_detection, switch_to_depth)
                self.panel._refresh_operation_header()
                return True

            self.panel._pending_camera_start = None
            if len(started_sides) != 2:
                failed_side = "right" if left_started else "left"
                self.panel._log(
                    f"[카메라] {failed_side} Preview worker 시작 거부; "
                    f"{', '.join(started_sides)} first-frame ACK를 대기합니다."
                )
            self._begin_camera_start_readiness(
                started_sides,
                enable_detection=enable_detection,
                switch_to_depth=switch_to_depth,
            )
            self.panel._refresh_session_toggle_buttons()
            self.panel._refresh_operation_header()
            return True

        return start_both()

    def _schedule_camera_start_retry(self, enable_detection: bool, switch_to_depth: bool) -> None:
        self.panel._camera_start_request_generation += 1
        generation = self.panel._camera_start_request_generation
        self.panel._pending_camera_start = (bool(enable_detection), bool(switch_to_depth))
        deadline = getattr(self.panel, "_camera_start_retry_deadline_s", None)
        if deadline is None:
            deadline = time.monotonic() + CAMERA_START_READINESS_TIMEOUT_S
            self.panel._camera_start_retry_deadline_s = deadline
        self.panel.camera_connection_label.setText("카메라: 이전 worker 종료 후 자동 재시작 대기 중")
        self.panel.start_button.setEnabled(False)
        self.panel.camera_connect_button.setEnabled(False)

        def retry() -> None:
            if generation != self.panel._camera_start_request_generation:
                return
            pending = self.panel._pending_camera_start
            if pending is None:
                return
            if time.monotonic() >= float(
                getattr(self.panel, "_camera_start_retry_deadline_s", 0.0)
                or 0.0
            ):
                self.panel._pending_camera_start = None
                self.panel._camera_start_retry_deadline_s = None
                self._rollback_camera_start(
                    "camera worker replacement timeout before first decoded frame",
                    enable_detection=bool(pending[0]),
                )
                return
            left_stopped = self.panel.left_pane.stop(wait=False)
            right_stopped = self.panel.right_pane.stop(wait=False)
            if not (left_stopped and right_stopped):
                QtCore.QTimer.singleShot(100, retry)
                return
            self.panel._pending_camera_start = None
            self.panel._camera_start_retry_deadline_s = None
            self.panel._start_camera_streams(
                enable_detection=pending[0],
                switch_to_depth=pending[1],
            )

        QtCore.QTimer.singleShot(100, retry)

    def _cancel_pending_camera_start(self) -> None:
        self.panel._camera_start_request_generation += 1
        self.panel._pending_camera_start = None
        self.panel._camera_start_retry_deadline_s = None
        self._cancel_camera_start_readiness()
        self._cancel_video_start_readiness(mark_idle=False)

    def _schedule_camera_stop_watchdog(self) -> None:
        """Keep re-issuing stop() until a wedged camera worker actually exits.

        camera_pane.stop(wait=False) only escalates to terminate/kill the
        child process once enough time has elapsed *and* stop() is called
        again; a plain disconnect otherwise calls it exactly once and, if the
        worker did not exit immediately (e.g. GStreamer wedged on a network
        interface that was not up yet), leaves it in "종료 대기 중" forever
        with the leaked process never escalated. This mirrors
        _schedule_camera_start_retry's retry loop, but only drives teardown;
        it never restarts a stream.
        """
        self.panel._camera_stop_watchdog_generation += 1
        generation = self.panel._camera_stop_watchdog_generation

        def retry() -> None:
            if generation != self.panel._camera_stop_watchdog_generation:
                return
            left_stopped = self.panel.left_pane.stop(wait=False)
            right_stopped = self.panel.right_pane.stop(wait=False)
            if not (left_stopped and right_stopped):
                QtCore.QTimer.singleShot(250, retry)
                return
            if not self._append_pending_video_finalization_artifacts():
                QtCore.QTimer.singleShot(250, retry)
                return
            finisher = getattr(self.panel, "finish_measurement_run", None)
            if callable(finisher) and self._measurement_run_state() == "STOPPING":
                finisher()
            self._resume_queued_measurement_start()

        QtCore.QTimer.singleShot(250, retry)

    def _append_pending_video_finalization_artifacts(self) -> bool:
        pending = getattr(self.panel, "_pending_video_finalization_artifacts", None)
        if not isinstance(pending, dict):
            return True
        paths = pending.get("paths")
        if not isinstance(paths, dict):
            self.panel._pending_video_finalization_artifacts = None
            return True
        backend = str(pending.get("backend") or "gstreamer")
        append = getattr(self.panel, "append_workstation_data", None)
        recorded_sides = {
            str(side) for side in tuple(pending.get("recorded_sides", ()) or ())
        }
        for side, raw_path in paths.items():
            side = str(side)
            if side in recorded_sides:
                continue
            path = Path(raw_path).expanduser()
            pane = getattr(self.panel, f"{side}_pane", None)
            finalizations = getattr(pane, "recording_finalizations", {})
            payload = (
                dict(finalizations.get(str(path), {}))
                if isinstance(finalizations, dict)
                else {}
            )
            side_backend = str(payload.get("stream_backend") or backend)
            container_success = bool(
                payload.get(
                    "container_finalization_success",
                    payload.get("success", False),
                )
            )
            h264_copy = side_backend.strip().lower() in {"gst", "gstreamer"}
            integrity_pass = bool(
                payload.get("integrity_schema") == H264_MP4_INTEGRITY_SCHEMA
                and payload.get("integrity_validation_status") == "PASS"
                and payload.get("recording_usable") is True
                and payload.get("recording_integrity_verified") is True
            )
            success = bool(
                payload.get("success", False)
                and (integrity_pass if h264_copy else True)
            )
            detail = str(
                payload.get("detail")
                or "worker exited without recording-finalization evidence"
            )
            if h264_copy and not integrity_pass:
                payload.update(
                    integrity_validation_status="FAIL",
                    recording_integrity_verified=False,
                    recording_usable=False,
                )
                failures = list(
                    payload.get("integrity_failure_reasons", ()) or ()
                )
                if payload.get("integrity_schema") != H264_MP4_INTEGRITY_SCHEMA:
                    failures.append("integrity_validation_schema_invalid")
                if not failures:
                    failures.append("finalized_mp4_integrity_evidence_missing")
                payload["integrity_failure_reasons"] = list(dict.fromkeys(failures))
                if container_success and "MP4 integrity FAIL" not in detail:
                    detail = (
                        f"{detail}; MP4 integrity FAIL: "
                        + ",".join(str(reason) for reason in failures)
                    )
            try:
                present = path.is_file()
                size_bytes = path.stat().st_size if present else 0
            except OSError:
                present = False
                size_bytes = 0
            artifact = {
                "event_type": "video_finalized",
                "side": side,
                "path": str(path),
                "media_type": "video/mp4",
                "stream_backend": side_backend,
                **recording_integrity_contract(side_backend),
                **payload,
                "success": success,
                "container_finalization_success": container_success,
                "finalization_success": success,
                "finalization_detail": detail,
                "present": present,
                "size_bytes": size_bytes,
            }
            accepted = True
            if callable(append):
                accepted = append(
                    "artifacts",
                    artifact,
                    priority=True,
                ) is not False
            if not accepted:
                logger = getattr(self.panel, "_log", None)
                if callable(logger):
                    logger(
                        f"[저장] {side} MP4 finalization evidence queue 실패; "
                        "DataStorage를 닫지 않고 재시도합니다."
                    )
                pending["recorded_sides"] = tuple(sorted(recorded_sides))
                return False
            finalization_evidence = getattr(
                self.panel,
                "_measurement_package_video_finalization_evidence",
                None,
            )
            if not isinstance(finalization_evidence, dict):
                finalization_evidence = {}
                self.panel._measurement_package_video_finalization_evidence = (
                    finalization_evidence
                )
            finalization_evidence[side] = dict(artifact)
            recorded_sides.add(side)
            if isinstance(finalizations, dict):
                finalizations.pop(str(path), None)
            if not success:
                logger = getattr(self.panel, "_log", None)
                if callable(logger):
                    logger(
                        f"[저장] {side} MP4 finalization 검증 실패: {path} ({detail})"
                    )
        pending["recorded_sides"] = tuple(sorted(recorded_sides))
        if any(str(side) not in recorded_sides for side in paths):
            return False
        flush = getattr(self.panel, "flush_workstation_data", None)
        if callable(flush) and flush() is False:
            return False
        self.panel._pending_video_finalization_artifacts = None
        return True

    def _cancel_camera_stop_watchdog(self) -> None:
        self.panel._camera_stop_watchdog_generation += 1

    def stop_yolo_streams(self, *, wait: bool = False) -> bool:
        if bool(getattr(self.panel, "yolo_enabled", False)):
            self._complete_package_detection()
        if self.panel.auto_wiris_connect_pending:
            self.panel._cancel_auto_wiris_connect_retry()
        self.panel._cancel_pending_camera_start()
        self.panel._camera_sync_timer.stop()
        self.panel._camera_sync_deadline_monotonic_s = None
        self.panel._camera_sync_stus_sr_baseline = {}
        self.panel._camera_sync_last_sr_counts = {}
        self.panel._camera_sync_transport_override = None
        stus_operation = getattr(self.panel, "_camera_sync_stus_operation", None)
        if isinstance(stus_operation, Mapping) and bool(stus_operation.get("active")):
            finish_stus = getattr(self.panel, "_finish_wiris_time_sync_operation", None)
            if callable(finish_stus):
                finish_stus(
                    "cancelled",
                    "camera streams were stopped before STUS closed-loop verification completed",
                )
        finalize_session = self.panel._session_recording_active()
        recording_paths = dict(
            getattr(self.panel, "video_recording_paths", {}) or {}
        )
        if finalize_session and recording_paths:
            backend_combo = getattr(self.panel, "stream_backend_combo", None)
            backend = (
                backend_combo.currentData()
                if backend_combo is not None
                else "gstreamer"
            )
            self.panel._pending_video_finalization_artifacts = {
                "paths": recording_paths,
                "backend": backend,
            }
        final_clock_sync_snapshot = (
            self.panel._camera_time_sync_snapshot("last_observed_before_stream_stop")
            if finalize_session
            else None
        )
        if final_clock_sync_snapshot is not None:
            self.panel._recording_clock_sync_end_observation = final_clock_sync_snapshot
        panes_stopped = self.panel._stop_camera_panes(clear_previews=True, wait=wait)
        if wait and not panes_stopped:
            self.panel._log("[종료] 카메라 worker가 아직 종료 중입니다. 다음 종료 시도에서 다시 확인합니다.")
            return False
        if not panes_stopped:
            self._schedule_camera_stop_watchdog()
        if finalize_session:
            sessionless = not bool(
                getattr(self.panel, "_measurement_session_hierarchy_enabled", True)
            )
            if bool(getattr(self.panel, "_automatic_session_recording", False)):
                self.panel.stop_automatic_session_recording(
                    final_clock_sync_snapshot=final_clock_sync_snapshot
                )
            elif sessionless:
                # Manual recording shares the camera resource. Releasing the
                # panes must also retire its transient owner; otherwise a
                # stale manual flag can leave the next Measurement queued
                # forever even though every camera worker has stopped.
                self.panel._stop_session_recording(
                    restart_streams=False,
                    final_clock_sync_snapshot=final_clock_sync_snapshot,
                )
            else:
                self.panel.distance_recording_enabled = False
                self.panel.video_recording_enabled = False
                self.panel.distance_recording_started_monotonic_s = None
                self.panel.record_button.setText("세션 기록 시작")
                self.panel._log(f"[저장] 카메라 Preview 해제로 세션 기록을 중지했습니다: {self.panel._format_video_recording_paths()}")
                self.panel.save_distance_history_csv(final_clock_sync_snapshot=final_clock_sync_snapshot)
        finalization_recorded = True
        if panes_stopped:
            finalization_recorded = self._append_pending_video_finalization_artifacts()
            finisher = getattr(self.panel, "finish_measurement_run", None)
            if (
                finalization_recorded
                and callable(finisher)
                and self._measurement_run_state() == "STOPPING"
            ):
                finisher()
        self.panel._clear_detection_state()
        self.panel.camera_connected = False
        self.panel.yolo_enabled = False
        self.panel._set_camera_sync_state(
            CAMERA_SYNC_STATUS_NOT_VERIFIED,
            "스트림이 중지되어 동기화 검증이 초기화되었습니다.",
        )
        last_sync = getattr(self.panel, "_last_wiris_pc_utc_sync", None)
        if (
            isinstance(last_sync, dict)
            and last_sync.get("command") == "STUS"
            and last_sync.get("result") in {"success", "pending-verification"}
        ):
            last_sync["result"] = "revoked"
            last_sync["verification_reason"] = (
                "camera stream generation stopped; fresh STUS/SR verification required"
            )
            last_sync["revoked_at_unix_s"] = time.time()
        self.panel.camera_connection_label.setText("카메라: 연결 해제")
        self.panel.start_button.setEnabled(True)
        self.panel.camera_connect_button.setEnabled(True)
        self.panel._refresh_session_toggle_buttons()
        self.panel._refresh_operation_header()
        return bool(panes_stopped and finalization_recorded)

    def _stop_camera_panes(self, *, clear_previews: bool, wait: bool = False) -> bool:
        left_stopped = self.panel.left_pane.stop(wait=wait)
        right_stopped = self.panel.right_pane.stop(wait=wait)
        self.panel.camera_frame_times["left"].clear()
        self.panel.camera_frame_times["right"].clear()
        self.panel.camera_frame_metadata["left"] = {}
        self.panel.camera_frame_metadata["right"] = {}
        self.panel.camera_frame_metadata_history["left"].clear()
        self.panel.camera_frame_metadata_history["right"].clear()
        self.panel.camera_stream_stats["left"] = {}
        self.panel.camera_stream_stats["right"] = {}
        self.panel.active_stream_decoders["left"] = None
        self.panel.active_stream_decoders["right"] = None
        self.panel._update_runtime_camera_stats_labels()
        if clear_previews:
            clear_signal = getattr(self.panel, "camera_preview_cleared", None)
            emit = getattr(clear_signal, "emit", None)
            if callable(emit):
                emit()
            self.panel._clear_calibration_previews("left preview", right_text="right preview")
        return bool(left_stopped and right_stopped)

    def _clear_calibration_previews(self, left_text: str, *, right_text: str | None = None) -> None:
        self.panel.latest_calibration_preview_images.clear()
        self.panel.calibration_preview_overlay_images.clear()
        self.panel.calibration_preview_request_ids["left"] = 0
        self.panel.calibration_preview_request_ids["right"] = 0
        if right_text is None:
            right_text = left_text
        for label, text in (
            (self.panel.calibration_left_preview, left_text),
            (self.panel.calibration_right_preview, right_text),
        ):
            label.setPixmap(QtGui.QPixmap())
            label.setText(text)

    def _clear_detection_state(self) -> None:
        measurement_owner = getattr(
            self.panel,
            "_measurement_view_controller_owner",
            None,
        )
        reset_pairing = getattr(
            measurement_owner,
            "reset_detection_pair_coordinator",
            None,
        )
        if callable(reset_pairing):
            reset_pairing()
        self.panel._camera_backpressure_degraded = False
        self.panel.latest_left = None
        self.panel.latest_right = None
        self.panel.detection_buffers["left"].clear()
        self.panel.detection_buffers["right"].clear()
        self.panel.left_pane.latest_detection = None
        self.panel.right_pane.latest_detection = None
        self.panel._clear_measurement_pair_display()
        self.panel._refresh_distance()

    def stop_all(self, *, wait: bool = False) -> bool:
        camera_stopped = self.panel.stop_yolo_streams(wait=wait)
        pixhawk_stopped = self.panel._stop_pixhawk_operation_workers(wait=wait)
        if pixhawk_stopped and self.panel.mavlink_pixhawk is not None:
            self.panel.disconnect_mavlink_pixhawk()
            # disconnect_mavlink_pixhawk() may complete synchronously when no
            # transport is open.  Otherwise closeEvent must retry until the
            # new disconnect worker and any retired RTCM worker are finished.
            pixhawk_stopped = bool(
                self.panel.mavlink_pixhawk is None
                and self.panel.pixhawk_disconnection_thread is None
                and self.panel.pixhawk_rtcm_thread is None
            )
        disconnection_thread = self.panel.pixhawk_disconnection_thread
        if disconnection_thread is not None and disconnection_thread.isRunning():
            pixhawk_stopped = False
        rtcm_thread = self.panel.pixhawk_rtcm_thread
        if rtcm_thread is not None and rtcm_thread.isRunning():
            pixhawk_stopped = False
        preview_stopped = self.panel._stop_calibration_preview_worker(wait=wait)
        all_stopped = bool(camera_stopped and pixhawk_stopped and preview_stopped)
        for attr in (
            "camera_auto_detect_thread",
            "wiris_control_check_thread",
            "wiris_command_thread",
            "wiris_zoom_thread",
            "wiris_time_sync_thread",
            "wiris_stip_thread",
            "camera_network_discovery_thread",
            "camera_network_apply_thread",
            "interface_status_thread",
            "pixhawk_usb_discovery_thread",
            "pixhawk_host_ip_discovery_thread",
            "rtsp_check_thread",
            "rtsp_enable_thread",
        ):
            thread = getattr(self.panel, attr, None)
            if thread is None:
                continue
            thread.quit()
            if thread.isRunning():
                if not wait or not thread.wait(1000):
                    all_stopped = False
                    continue
            if getattr(self.panel, attr, None) is thread:
                setattr(self.panel, attr, None)
        return all_stopped
