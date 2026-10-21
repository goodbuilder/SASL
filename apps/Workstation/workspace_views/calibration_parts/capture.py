"""CalibrationCaptureSurface owns its widgets and transient presentation state.

Domain formulas, native raster/timing admission and calibration solves remain with
the existing backend and core contracts. No workspace host is accepted.
"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.ui.designer import load_form
from Workstation.ui.components import StatusIndicator
from Workstation.application.status_model import RuntimeStatus, StatusSeverity
from ..shared import _command_button

from .widgets import _ElidedStatusLabel


class CalibrationCaptureSurface(QtCore.QObject):
    def __init__(self, commands, preview, initial_setup, refresh_labels, parent=None, *,
                 read_capture, zoom_command=lambda: None, capture_signal=None):
        super().__init__(parent)
        if not callable(read_capture) or not callable(zoom_command):
            raise TypeError("Calibration capture requires status and zoom-command readers")
        self._read_capture = read_capture
        self._zoom_command = zoom_command
        self._capture_signal = capture_signal
        self._capture_signal_connected = False
        self._disposed = False
        self.commands = commands
        self.preview = preview
        self._sync_simple_calibration_labels = refresh_labels
        self._initial_calibration_setup = dict(initial_setup)
        self.setup_widgets = {}
        self.camera_buttons = {}
        self.invoke_command = None
        self._calibration_capture_log_lines = {"left": [], "right": []}
        self.zoom_group = load_form("calibration_zoom.ui", QtWidgets.QGroupBox())
        for name in ("zoom_level_spin", "zoom_set_button", "zoom_in_button", "zoom_out_button", "zoom_query_button", "zoom_readback_label"):
            setattr(self, name, getattr(self.zoom_group, name))
        # The numeric value remains a WIRIS zoom index, not an optical ratio.
        self.zoom_readback_label.setObjectName("statusStrip")
        self.zoom_set_button.clicked.connect(lambda: self._send_calibration_zoom("set", self.zoom_level_spin.value()))
        self.zoom_query_button.clicked.connect(lambda: self._send_calibration_zoom("query", None))
        self.zoom_in_button.clicked.connect(lambda: self._send_calibration_zoom("in", None))
        self.zoom_out_button.clicked.connect(lambda: self._send_calibration_zoom("out", None))


    def _build_simple_calibration_tabs(self, quality_text="", profile_text="") -> QtWidgets.QTabWidget:
        """Bind the Designer capture-to-apply surface to existing command owners."""
        tabs = load_form("calibration_simple.ui", QtWidgets.QTabWidget())
        self.simple_calibration_buttons = {}
        for button in tabs.findChildren(QtWidgets.QPushButton):
            command = button.property("commandId")
            if not command:
                continue
            button.setEnabled(self.invoke_command is not None)
            if self.invoke_command is not None:
                button.clicked.connect(lambda _checked=False, command=command: self.invoke_command(command))
            self.simple_calibration_buttons.setdefault(command, []).append(button)
            self.commands.primary.setdefault(command, button)
            self.commands.instances.setdefault(command, []).append(button)
        simple_zoom_spin = tabs.simple_zoom_spin
        simple_zoom_spin.setObjectName("simpleCalibrationZoomIndex")
        simple_zoom_spin.setRange(self.zoom_level_spin.minimum(), self.zoom_level_spin.maximum())
        simple_zoom_spin.setValue(self.zoom_level_spin.value())
        simple_zoom_spin.valueChanged.connect(self.zoom_level_spin.setValue)
        self.zoom_level_spin.valueChanged.connect(simple_zoom_spin.setValue)
        tabs.simple_set_zoom.clicked.connect(lambda: self._send_calibration_zoom("set", simple_zoom_spin.value()))
        tabs.simple_query_zoom.clicked.connect(lambda: self._send_calibration_zoom("query", None))
        self.simple_calibration_zoom_status = tabs.simple_calibration_zoom_status
        self.simple_calibration_zoom_status.setObjectName("simpleCalibrationZoomStatus")
        self.simple_calibration_zoom_status.setText(self.zoom_readback_label.text())
        self.simple_calibration_board_indicators = {}
        for side, label in (("left", "보드 L"), ("right", "보드 R")):
            indicator = StatusIndicator(label, compact=True)
            indicator.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            self.simple_calibration_board_indicators[side] = indicator
            tabs.board_status_row.addWidget(indicator, 1)
        for name, object_name, text in (
            ("simple_calibration_run_status", "simpleCalibrationRunStatus", "이번 촬영: 검사 0 · Stereo 저장 0 · K,D 전용 0 · 실패 0"),
            ("simple_calibration_set_status", "simpleCalibrationSetStatus", "현재 캡처 세트: L 0/20 · R 0/20 · Stereo 저장 후보 0 · K,D 전용 L0/R0"),
            ("simple_calibration_capture_status", "simpleCalibrationCaptureStatus", f"최근 상태: {self.capture_summary_label.text()}"),
        ):
            label = _ElidedStatusLabel(text)
            label.setObjectName(object_name)
            setattr(self, name, label)
            getattr(tabs, name + "Layout").addWidget(label)
        tabs.previewLayout.addWidget(self.preview.build_simple_previews(tabs.capture_page))
        self.simple_calibration_quality_status = tabs.simple_calibration_quality_status
        self.simple_calibration_quality_status.setObjectName("simpleCalibrationQualityStatus")
        self.simple_calibration_quality_status.setText(quality_text)
        self.simple_calibration_profile_status = tabs.simple_calibration_profile_status
        self.simple_calibration_profile_status.setObjectName("simpleCalibrationProfileStatus")
        self.simple_calibration_profile_status.setText(profile_text)
        for label in (tabs.capture_title, tabs.compute_title, tabs.profile_title, tabs.compute_note):
            label.setObjectName("compactContract")
        return tabs


    def calibration_capture_snapshot(self) -> dict[str, object]:
        return self._read_capture()

    def stop(self):
        self._disposed = True
        if self._capture_signal_connected:
            try:
                self._capture_signal.disconnect(self._append_calibration_capture_event)
            except (TypeError, RuntimeError):
                pass
            self._capture_signal_connected = False

    @staticmethod
    def _calibration_capture_count(value: object) -> str:
        if isinstance(value, bool):
            return "--"
        try:
            count = int(value)
        except (TypeError, ValueError, OverflowError):
            return "--"
        return str(count) if count >= 0 else "--"


    def apply_calibration_capture_snapshot(self, value: object) -> dict[str, object]:
        snapshot = dict(value) if isinstance(value, Mapping) else {}
        detections = snapshot.get("detection")
        detections = detections if isinstance(detections, Mapping) else {}
        state_labels = {
            "DETECTING": "검출 중",
            "FOUND": "인식",
            "NOT_FOUND": "미인식",
            "NO_FRAME": "프레임 없음",
            "STALE": "STALE",
            "UNAVAILABLE": "사용 불가",
        }
        state_severity = {
            "DETECTING": StatusSeverity.INFO,
            "FOUND": StatusSeverity.OK,
            "NOT_FOUND": StatusSeverity.INFO,
            "NO_FRAME": StatusSeverity.UNKNOWN,
            "STALE": StatusSeverity.WARN,
            "UNAVAILABLE": StatusSeverity.WARN,
        }
        for side in ("left", "right"):
            row = detections.get(side)
            row = row if isinstance(row, Mapping) else {}
            state = str(row.get("state") or "NO_FRAME").upper()
            corner_count = self._calibration_capture_count(row.get("corner_count"))
            display_state = state_labels.get(state, state)
            if state == "FOUND" and corner_count != "--":
                display_state += f" {corner_count}"
            try:
                age_s = float(row.get("age_s"))
            except (TypeError, ValueError, OverflowError):
                age_s = None
            else:
                if not math.isfinite(age_s) or age_s < 0.0:
                    age_s = None
            detail = str(row.get("detail") or "checkerboard evidence unavailable")
            indicator = getattr(self, "simple_calibration_board_indicators", {}).get(side)
            if isinstance(indicator, StatusIndicator):
                indicator.set_status(
                    RuntimeStatus(
                        key=f"calibration.board.{side}",
                        label=f"보드 {'L' if side == 'left' else 'R'}",
                        state=display_state,
                        severity=state_severity.get(state, StatusSeverity.UNKNOWN),
                        detail=detail,
                        source="CheckerboardPreviewWorker",
                        age_s=age_s,
                        clock="app-receive monotonic",
                        observed_monotonic_s=time.monotonic(),
                    )
                )
            age_text = "" if age_s is None else f" · {age_s:.1f} s"
            self.preview.set_detection_text(side, f"보드: {display_state}{age_text}")

        run = snapshot.get("run")
        run = run if isinstance(run, Mapping) else {}
        run_text = (
            "이번 촬영: 검사 "
            f"{self._calibration_capture_count(run.get('inspection_request_count'))} · "
            "Stereo 저장 "
            f"{self._calibration_capture_count(run.get('stereo_saved_count'))} · "
            "K,D 전용 "
            f"{self._calibration_capture_count(run.get('intrinsics_only_saved_count'))} · "
            f"실패 {self._calibration_capture_count(run.get('failed_count'))}"
        )
        self.simple_calibration_run_status.set_full_text(run_text)
        self.simple_calibration_run_status.setToolTip(
            run_text
            + "\n검사는 checkerboard 저장 검증 worker가 실제 시작된 경우만 셉니다."
        )

        capture_set = snapshot.get("capture_set")
        capture_set = capture_set if isinstance(capture_set, Mapping) else {}
        minimum = self._calibration_capture_count(
            snapshot.get("minimum_images_per_side")
        )
        set_id = str(snapshot.get("capture_set_id") or "--")
        set_text = (
            f"현재 캡처 세트 {set_id}: "
            f"L {self._calibration_capture_count(capture_set.get('left_image_count'))}/{minimum} · "
            f"R {self._calibration_capture_count(capture_set.get('right_image_count'))}/{minimum} · "
            "Stereo 저장 후보 "
            f"{self._calibration_capture_count(capture_set.get('stereo_candidate_count'))} · "
            "K,D 전용 L"
            f"{self._calibration_capture_count(capture_set.get('left_intrinsics_only_count'))}/R"
            f"{self._calibration_capture_count(capture_set.get('right_intrinsics_only_count'))}"
        )
        warning_count = self._calibration_capture_count(
            capture_set.get("evidence_warning_pair_count")
        )
        if warning_count not in {"--", "0"}:
            set_text += f" · 증거 경고 {warning_count}"
        self.simple_calibration_set_status.set_full_text(set_text)
        set_tooltip = [set_text]
        capture_root = str(snapshot.get("capture_root") or capture_set.get("root") or "")
        if capture_root:
            set_tooltip.append(f"Root: {capture_root}")
        error = str(capture_set.get("error") or "")
        if error:
            set_tooltip.append(f"Inventory: {error}")
        set_tooltip.append(
            "Stereo 저장 후보는 수동 제외 및 solver pose 필터 전의 persisted pair 수입니다."
        )
        self.simple_calibration_set_status.setToolTip("\n".join(set_tooltip))

        last = snapshot.get("last")
        last = last if isinstance(last, Mapping) else {}
        message = str(last.get("message") or "").strip()
        if not message:
            message = self.capture_summary_label.text()
        target = self.simple_calibration_capture_status
        target.set_full_text(f"최근 상태: {message}")
        target.setToolTip(message)
        return snapshot


    def _sync_simple_calibration_status(
        self,
        capture_status: object = None,
    ) -> None:
        """Mirror authoritative advanced status into the simple surface.

        Without a snapshot this reads the capture status from the backend, which
        stats the capture root; callers that only changed a label should use
        _sync_simple_calibration_labels() instead.
        """

        self._sync_simple_calibration_labels()
        if hasattr(self, "simple_calibration_run_status"):
            self.apply_calibration_capture_snapshot(
                capture_status
                if isinstance(capture_status, Mapping)
                else self.calibration_capture_snapshot()
            )


    def _send_calibration_zoom(
        self,
        action: str,
        level: int | None,
    ) -> None:
        sender = self._zoom_command()
        if not callable(sender):
            self.zoom_readback_label.setText("Zoom control unavailable")
            return
        try:
            sender("both", action, level)
        except Exception as exc:
            self.zoom_readback_label.setText(
                f"Zoom command failed: {exc}"
            )
            return
        if action == "set":
            status = f"Requested both={level}×; GZVV readback pending"
        elif action == "in":
            status = "Requested Zoom-in for both; GZVV readback pending"
        elif action == "out":
            status = "Requested Zoom-out for both; GZVV readback pending"
        else:
            status = "GZVV readback pending"
        self.zoom_readback_label.setText(status)
        self._sync_simple_calibration_labels()


    def _register_capture_button(
        self,
        command_id: str,
        label: str,
    ) -> QtWidgets.QPushButton:
        button = _command_button(label, command_id, self.invoke_command)
        button.setMinimumWidth(0)
        button.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Fixed,
        )
        self.commands.primary[command_id] = button
        self.commands.instances.setdefault(command_id, []).append(button)
        return button


    def _build_shared_capture_surface(self) -> QtWidgets.QGroupBox:
        group = load_form("calibration_capture.ui", QtWidgets.QGroupBox())
        group.setObjectName("calibrationSharedCaptureSurface")
        for button in group.findChildren(QtWidgets.QPushButton):
            command = button.property("commandId")
            if not command:
                continue
            button.setEnabled(self.invoke_command is not None)
            if self.invoke_command is not None:
                button.clicked.connect(lambda _checked=False, command=command: self.invoke_command(command))
            if command == "camera.toggle":
                self.camera_buttons[command] = button
            else:
                self.commands.primary[command] = button
                self.commands.instances.setdefault(command, []).append(button)
            if command == "calibration.capture_images":
                self.capture_primary_button = button
                font = button.font()
                font.setBold(True)
                button.setFont(font)
        for key in ("pair_max_delta_ms", "auto_interval_s"):
            control = getattr(group, f"setup_widgets_{key}")
            self.setup_widgets[key] = control
            value = self._initial_calibration_setup.get(key)
            if value is not None:
                control.setValue(float(value) if key == "pair_max_delta_ms" else float(value or 1.0))
        self.setup_widgets["pair_max_delta_ms"].setObjectName("calibrationPairMaxDeltaMs")
        self.capture_summary_label = group.capture_summary_label
        self.capture_summary_label.setObjectName("statusStrip")
        self._capture_summary_states = {"left": "WAITING", "right": "WAITING"}
        self._capture_pair_text = "--"
        for name in ("auto_capture_settings_group", "auto_capture_settings_content", "capture_response_outer_group", "capture_response_content"):
            setattr(self, name, getattr(group, name))
        for outer, inner in ((self.auto_capture_settings_group, self.auto_capture_settings_content), (self.capture_response_outer_group, self.capture_response_content)):
            inner.setVisible(False)
            outer.toggled.connect(inner.setVisible)
        self._bind_capture_response(group)
        return group

    def _bind_capture_response(self, group):
        self.calibration_capture_response_edits = {}
        self.calibration_capture_response_status = {}
        for side in ("left", "right"):
            status = getattr(group, f"calibration_capture_response_status_{side}")
            status.setObjectName("statusStrip")
            self.calibration_capture_response_status[side] = status
            log = getattr(group, f"calibration_capture_response_edits_{side}")
            log.setMaximumBlockCount(160)
            self.calibration_capture_response_edits[side] = log
        for button in group.findChildren(QtWidgets.QPushButton):
            if button.property("captureClear"):
                button.clicked.connect(self._clear_calibration_capture_response_logs)

    def _build_calibration_capture_response_group(self) -> QtWidgets.QGroupBox:
        """Show the raw capture lifecycle separately for the two cameras."""
        group = load_form("calibration_capture_response.ui", QtWidgets.QGroupBox())
        self._bind_capture_response(group)
        return group


    def _connect_calibration_capture_event_signal(self) -> None:
        if self._disposed or self._capture_signal_connected:
            return
        signal = self._capture_signal
        connect = getattr(signal, "connect", None)
        if callable(connect):
            connect(self._append_calibration_capture_event)
            self._capture_signal_connected = True

    def _clear_calibration_capture_response_logs(self) -> None:
        for side in ("left", "right"):
            self._calibration_capture_log_lines[side] = []
            status = self.calibration_capture_response_status.get(side)
            if status is not None:
                status.setText("대기")
            editor = self.calibration_capture_response_edits.get(side)
            if editor is not None:
                editor.clear()
        self._capture_summary_states = {"left": "WAITING", "right": "WAITING"}
        self._capture_pair_text = "--"
        self._refresh_capture_summary()


    def _refresh_capture_summary(self) -> None:
        self.capture_summary_label.setText(
            f"L: {self._capture_summary_states['left']} · "
            f"R: {self._capture_summary_states['right']} · "
            f"Pair: {self._capture_pair_text}"
        )
        self._sync_simple_calibration_status()


    @QtCore.pyqtSlot(object)
    def _append_calibration_capture_event(self, payload: object) -> None:
        if self._disposed or not isinstance(payload, Mapping):
            return
        operation = str(payload.get("operation") or "Capture Live Pair")
        phase = str(payload.get("phase") or "event")
        if phase == "ftp_pair_ready":
            self._capture_summary_states = {"left": "READY", "right": "READY"}
            pair_delta = payload.get("pair_delta_ms", payload.get("timestamp_delta_ms"))
            if pair_delta is None:
                self._capture_pair_text = "READY"
            else:
                try:
                    self._capture_pair_text = f"Δt={float(pair_delta):.1f} ms"
                except (TypeError, ValueError):
                    self._capture_pair_text = str(pair_delta)
            self._refresh_capture_summary()
            self.preview.show_ftp_pair(payload)
            return
        if "FTP" in operation and phase == "failed":
            detail = str(payload.get("response") or payload.get("state") or "unknown error")
            self.preview.set_ftp_status(
                f"{operation} failed: {detail} · last successful FTP pair retained."
            )
        side = str(payload.get("side") or "").strip().lower()
        if side not in {"left", "right"}:
            return
        state = str(payload.get("state") or "")
        self._capture_summary_states[side] = (state or phase).upper()
        pair_delta = payload.get("pair_delta_ms", payload.get("timestamp_delta_ms"))
        if pair_delta is not None:
            try:
                self._capture_pair_text = f"Δt={float(pair_delta):.1f} ms"
            except (TypeError, ValueError):
                self._capture_pair_text = str(pair_delta)
        self._refresh_capture_summary()
        command = str(payload.get("command") or "").strip()
        response = str(payload.get("response") or "").strip()
        capture_id = str(payload.get("capture_id") or "").strip()
        timestamp = QtCore.QDateTime.currentDateTime().toString("HH:mm:ss.zzz")
        prefix = f"[{timestamp}] {operation}"
        if capture_id:
            prefix += f" {capture_id}"
        lines: list[str]
        if phase == "command":
            lines = [f"{prefix} · {side}", f"> {command or '<command>'}"]
        elif phase == "response":
            lines = [f"{prefix} · {side}", f"< {command or '<command>'}: {response or '<empty>'}"]
        else:
            detail = response or str(payload.get("path") or "") or state or phase
            lines = [f"{prefix} · {phase} · {state or 'unknown'}: {detail}"]
            remote_path = str(payload.get("remote_path") or "").strip()
            if remote_path:
                lines.append(f"remote: {remote_path}")
        buffer = self._calibration_capture_log_lines.setdefault(side, [])
        buffer.extend(lines)
        del buffer[:-160]
        editor = self.calibration_capture_response_edits.get(side)
        if editor is not None:
            editor.setPlainText("\n".join(buffer))
            cursor = editor.textCursor()
            cursor.movePosition(QtGui.QTextCursor.End)
            editor.setTextCursor(cursor)
        status = self.calibration_capture_response_status.get(side)
        if status is not None:
            status.setText(f"{operation}: {state or phase}")


