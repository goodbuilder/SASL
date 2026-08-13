"""Workstation camera stream and detection lifecycle controller."""

from __future__ import annotations

from PyQt5 import QtCore, QtGui

from Workstation.camera_time_sync_controller import (
    CAMERA_SYNC_STATUS_NOT_VERIFIED,
    CAMERA_SYNC_STATUS_RECONNECTING,
    CAMERA_SYNC_STATUS_WARMING,
)

CAMERA_STREAM_LIFECYCLE_METHODS = [
    "toggle_camera_connection",
    "toggle_yolo_detection",
    "connect_cameras",
    "start_yolo",
    "stop_yolo_detection",
    "_start_camera_streams",
    "_schedule_camera_start_retry",
    "_cancel_pending_camera_start",
    "stop_yolo_streams",
    "_stop_camera_panes",
    "_clear_calibration_previews",
    "_clear_detection_state",
    "stop_all"
]


class CameraStreamLifecycleController:
    def __init__(self, panel):
        self.panel = panel

    def toggle_camera_connection(self) -> None:
        if self.panel.camera_connected or self.panel._pending_camera_start is not None:
            self.panel.stop_yolo_streams()
        else:
            self.panel.connect_cameras()

    def toggle_yolo_detection(self) -> bool | None:
        if self.panel._session_recording_active() and (
            self.panel.camera_connected or self.panel._pending_camera_start is not None
        ):
            self.panel._log("[YOLO] 세션 기록 중에는 실행 중인 카메라의 Detection 모드를 바꿀 수 없습니다.")
            return False
        if self.panel.yolo_enabled or self.panel._pending_camera_start is not None:
            self.panel.stop_yolo_detection()
        else:
            self.panel.start_yolo()
        return None

    def connect_cameras(self) -> None:
        if self.panel.auto_wiris_connect_pending:
            self.panel._cancel_auto_wiris_connect_retry()
        self.panel._start_camera_streams(enable_detection=False, switch_to_depth=False)

    def start_yolo(self) -> None:
        self.panel._start_camera_streams(enable_detection=True, switch_to_depth=True)

    def stop_yolo_detection(self) -> None:
        if self.panel._pending_camera_start is not None:
            self.panel.stop_yolo_streams()
            return
        if not self.panel.camera_connected:
            self.panel._log("[YOLO] 연결된 카메라 스트림이 없어 중지할 YOLO가 없습니다.")
            return
        if not self.panel.yolo_enabled:
            self.panel._log("[YOLO] 이미 중지 상태입니다. 카메라 프리뷰는 유지 중입니다.")
            return
        self.panel._log("[YOLO] 검출을 중지하고 카메라 프리뷰만 유지합니다.")
        self.panel._start_camera_streams(enable_detection=False, switch_to_depth=False)

    def _start_camera_streams(self, *, enable_detection: bool, switch_to_depth: bool) -> None:
        kwargs = self.panel.yolo_kwargs()
        left_url = self.panel.left_url_edit.text().strip()
        right_url = self.panel.right_url_edit.text().strip()
        if not left_url or not right_url:
            self.panel._log("[카메라] Left/Right RTSP URL을 입력해야 합니다.")
            return
        if self.panel._rtsp_urls_are_same_stream():
            message = "카메라: 좌/우 RTSP URL이 같은 스트림입니다. 좌우 카메라 IP를 분리하세요."
            self.panel.camera_connection_label.setText(message)
            self.panel._set_distance_text("거리: 좌/우 RTSP URL이 동일해 측정 차단", quality="ERROR")
            self.panel._log(f"[카메라] {message}")
            return
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
            return

        def start_both() -> None:
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
                return

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

        start_both()

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

    def stop_yolo_streams(self, *, wait: bool = False) -> bool:
        if self.panel.auto_wiris_connect_pending:
            self.panel._cancel_auto_wiris_connect_retry()
        self.panel._cancel_pending_camera_start()
        self.panel._camera_sync_timer.stop()
        self.panel._camera_sync_deadline_monotonic_s = None
        self.panel._camera_sync_transport_override = None
        finalize_session = self.panel._session_recording_active()
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
        if finalize_session:
            self.panel.distance_recording_enabled = False
            self.panel.video_recording_enabled = False
            self.panel.distance_recording_started_monotonic_s = None
            self.panel.record_button.setText("세션 기록 시작")
            self.panel._log(f"[저장] 카메라 Preview 해제로 세션 기록을 중지했습니다: {self.panel._format_video_recording_paths()}")
            self.panel.save_distance_history_csv(final_clock_sync_snapshot=final_clock_sync_snapshot)
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
