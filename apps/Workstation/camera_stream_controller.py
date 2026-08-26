"""Workstation camera stream and detection lifecycle controller."""

from __future__ import annotations

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

CAMERA_STREAM_LIFECYCLE_METHODS = [
    "toggle_camera_connection",
    "toggle_yolo_detection",
    "connect_cameras",
    "start_yolo",
    "stop_yolo_detection",
    "_resume_queued_measurement_start",
    "_start_camera_streams",
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
        if self.panel.camera_connected or self.panel._pending_camera_start is not None:
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
        if (
            requested == DECODER_AUTO
            and getattr(self.panel, "_runtime_decoder_override", None) == DECODER_SOFTWARE
        ):
            reason = next((str(value) for value in fallback_reasons.values() if str(value)), "")
            self.panel.decoder_selection_status = {
                "requested": DECODER_AUTO,
                "effective": SOFTWARE_H264_DECODER,
                "candidates": (SOFTWARE_H264_DECODER,),
                "allowed": True,
                "degraded": True,
                "reason": reason or "runtime NVIDIA fallback override active",
                "nvidia_available": nvidia_available,
            }
            return True
        selection = current_decoder_selection(requested)
        self.panel.decoder_selection_status = {
            "requested": selection.requested,
            "effective": selection.effective,
            "candidates": tuple(selection.candidates),
            "allowed": selection.allowed,
            "degraded": selection.degraded,
            "reason": selection.reason,
            "nvidia_available": nvidia_available,
        }
        if not selection.allowed:
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
        if selection.effective == NVIDIA_H264_DECODER:
            fallback_reasons.update(left="", right="")
        elif requested == DECODER_SOFTWARE:
            fallback_reasons.update(left="", right="")
        if requested == DECODER_AUTO and selection.effective == SOFTWARE_H264_DECODER:
            if getattr(self.panel, "_runtime_decoder_override", None) != DECODER_SOFTWARE:
                self.panel._log(
                    f"[카메라] Auto decoder 경고: {selection.reason}; avdec_h264를 사용합니다."
                )
            self.panel._runtime_decoder_override = DECODER_SOFTWARE
            fallback_reasons.update(left=selection.reason, right=selection.reason)
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
        return False if stream_result is False else None

    def stop_yolo_detection(self) -> None:
        if self.panel._pending_camera_start is not None:
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

    def _start_camera_streams(self, *, enable_detection: bool, switch_to_depth: bool) -> bool:
        # A leftover stop watchdog from a prior disconnect must not call
        # pane.stop() again once this start attempt owns the pane's teardown
        # (via _stop_camera_panes below and, if needed, its own start retry).
        self._cancel_camera_stop_watchdog()
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
        self.panel.start_button.setEnabled(False)
        self.panel.camera_connect_button.setEnabled(False)
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
        if self.panel.camera_connected:
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
            left_started = self.panel.left_pane.start_stream(
                left_url,
                enable_detection=enable_detection,
                **kwargs,
                **self.panel._video_recording_kwargs("left"),
            )
            right_started = self.panel.right_pane.start_stream(
                right_url,
                enable_detection=enable_detection,
                **kwargs,
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
            self.panel.camera_connected = True
            self.panel.yolo_enabled = enable_detection
            self.panel.start_button.setEnabled(True)
            self.panel.camera_connect_button.setEnabled(True)
            if len(started_sides) != 2:
                failed_side = "right" if left_started else "left"
                self.panel._log(
                    f"[카메라] {failed_side} Preview 시작 실패. "
                    f"{', '.join(started_sides)} 스트림은 유지하며 실패한 카메라만 재활성화하세요."
                )
                self.panel.camera_connection_label.setText(
                    f"camera partial: {', '.join(started_sides)} connected; {failed_side} unavailable"
                )
            self.panel._refresh_session_toggle_buttons()
            self.panel._refresh_operation_header()
            if enable_detection:
                confirmer = getattr(self.panel, "confirm_measurement_running", None)
                if callable(confirmer):
                    confirmer(started_sides)
            return True

        return start_both()

    def _schedule_camera_start_retry(self, enable_detection: bool, switch_to_depth: bool) -> None:
        self.panel._camera_start_request_generation += 1
        generation = self.panel._camera_start_request_generation
        self.panel._pending_camera_start = (bool(enable_detection), bool(switch_to_depth))
        self.panel.camera_connection_label.setText("카메라: 이전 worker 종료 후 자동 재시작 대기 중")
        self.panel.start_button.setEnabled(False)
        self.panel.camera_connect_button.setEnabled(False)

        def retry() -> None:
            if generation != self.panel._camera_start_request_generation:
                return
            pending = self.panel._pending_camera_start
            if pending is None:
                return
            left_stopped = self.panel.left_pane.stop(wait=False)
            right_stopped = self.panel.right_pane.stop(wait=False)
            if not (left_stopped and right_stopped):
                QtCore.QTimer.singleShot(100, retry)
                return
            self.panel._pending_camera_start = None
            self.panel._start_camera_streams(
                enable_detection=pending[0],
                switch_to_depth=pending[1],
            )

        QtCore.QTimer.singleShot(100, retry)

    def _cancel_pending_camera_start(self) -> None:
        self.panel._camera_start_request_generation += 1
        self.panel._pending_camera_start = None

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
            self._append_pending_video_finalization_artifacts()
            finisher = getattr(self.panel, "finish_measurement_run", None)
            if callable(finisher) and self._measurement_run_state() == "STOPPING":
                finisher()
            self._resume_queued_measurement_start()

        QtCore.QTimer.singleShot(250, retry)

    def _append_pending_video_finalization_artifacts(self) -> None:
        pending = getattr(self.panel, "_pending_video_finalization_artifacts", None)
        if not isinstance(pending, dict):
            return
        paths = pending.get("paths")
        if not isinstance(paths, dict):
            self.panel._pending_video_finalization_artifacts = None
            return
        backend = str(pending.get("backend") or "gstreamer")
        append = getattr(self.panel, "append_workstation_data", None)
        for side, raw_path in paths.items():
            path = Path(raw_path).expanduser()
            pane = getattr(self.panel, f"{side}_pane", None)
            finalizations = getattr(pane, "recording_finalizations", {})
            payload = (
                dict(finalizations.pop(str(path), {}))
                if isinstance(finalizations, dict)
                else {}
            )
            success = bool(payload.get("success", False))
            detail = str(
                payload.get("detail")
                or "worker exited without recording-finalization evidence"
            )
            if callable(append):
                append(
                    "artifacts",
                    {
                        "event_type": "video_finalized",
                        "side": str(side),
                        "path": str(path),
                        "media_type": "video/mp4",
                        "stream_backend": backend,
                        **recording_integrity_contract(backend),
                        **payload,
                        "finalization_success": success,
                        "finalization_detail": detail,
                        "present": path.is_file(),
                        "size_bytes": path.stat().st_size if path.is_file() else 0,
                    },
                    priority=True,
                )
            if not success:
                logger = getattr(self.panel, "_log", None)
                if callable(logger):
                    logger(
                        f"[저장] {side} MP4 finalization 검증 실패: {path} ({detail})"
                    )
        self.panel._pending_video_finalization_artifacts = None
        flush = getattr(self.panel, "flush_workstation_data", None)
        if callable(flush):
            flush()

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
        self.panel._camera_sync_transport_override = None
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
        if panes_stopped:
            self._append_pending_video_finalization_artifacts()
            finisher = getattr(self.panel, "finish_measurement_run", None)
            if callable(finisher) and self._measurement_run_state() == "STOPPING":
                finisher()
        self.panel._clear_detection_state()
        self.panel.camera_connected = False
        self.panel.yolo_enabled = False
        self.panel._set_camera_sync_state(
            CAMERA_SYNC_STATUS_NOT_VERIFIED,
            "스트림이 중지되어 동기화 검증이 초기화되었습니다.",
        )
        self.panel.camera_connection_label.setText("카메라: 연결 해제")
        self.panel.start_button.setEnabled(True)
        self.panel.camera_connect_button.setEnabled(True)
        self.panel._refresh_session_toggle_buttons()
        self.panel._refresh_operation_header()
        return panes_stopped

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
