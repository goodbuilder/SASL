"""CalibrationCaptureSurface owns its widgets and transient presentation state.

Domain formulas, native raster/timing admission and calibration solves remain with
the existing backend and core contracts. No workspace host is accepted.
"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from PyQt5 import QtCore, QtGui, QtWidgets
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
        zoom_group = QtWidgets.QGroupBox("Actual zoom readback")
        zoom_layout = QtWidgets.QHBoxLayout(zoom_group)
        self.zoom_level_spin = QtWidgets.QSpinBox()
        # This control carries the WIRIS zoom *index*, not the optical ratio:
        # index 0 is the nominal 1.0x position and index 1 is nominal 1.2x.
        self.zoom_level_spin.setRange(0, 100)
        self.zoom_level_spin.setValue(1)
        self.zoom_level_spin.setToolTip(
            "WIRIS zoom index (index 0 = nominal 1.0x; index 1 = nominal 1.2x)."
        )
        self.zoom_set_button = QtWidgets.QPushButton("Set both")
        self.zoom_in_button = QtWidgets.QPushButton("Zoom-in")
        self.zoom_out_button = QtWidgets.QPushButton("Zoom-out")
        self.zoom_query_button = QtWidgets.QPushButton("GZVV")
        self.zoom_readback_label = QtWidgets.QLabel(
            "left=unknown, right=unknown"
        )
        self.zoom_readback_label.setObjectName("statusStrip")
        self.zoom_readback_label.setWordWrap(True)
        self.zoom_set_button.clicked.connect(
            lambda: self._send_calibration_zoom(
                "set", self.zoom_level_spin.value()
            )
        )
        self.zoom_query_button.clicked.connect(
            lambda: self._send_calibration_zoom("query", None)
        )
        self.zoom_in_button.clicked.connect(
            lambda: self._send_calibration_zoom("in", None)
        )
        self.zoom_out_button.clicked.connect(
            lambda: self._send_calibration_zoom("out", None)
        )
        self.zoom_in_button.setToolTip(
            "Send WIRIS SZIN to both cameras, then refresh the GZVV readback."
        )
        self.zoom_out_button.setToolTip(
            "Send WIRIS SZOT to both cameras, then refresh the GZVV readback."
        )
        zoom_layout.addWidget(QtWidgets.QLabel("Index"))
        zoom_layout.addWidget(self.zoom_level_spin)
        zoom_layout.addWidget(self.zoom_set_button)
        zoom_layout.addWidget(self.zoom_in_button)
        zoom_layout.addWidget(self.zoom_out_button)
        zoom_layout.addWidget(self.zoom_query_button)
        zoom_layout.addWidget(self.zoom_readback_label, 1)
        self.zoom_group = zoom_group


    def _build_simple_calibration_tabs(self, quality_text="", profile_text="") -> QtWidgets.QTabWidget:
        """Build the StereoWIRIS-style primary capture-to-apply workflow.

        This is intentionally a thin command surface.  The authoritative
        calculation, quality, zoom, native-raster, persistence, and Apply
        gates remain owned by the existing Workstation controllers exposed in
        the advanced surface below it.
        """

        tabs = QtWidgets.QTabWidget()
        tabs.setObjectName("simpleCalibrationTabs")
        tabs.setDocumentMode(True)
        self.simple_calibration_buttons: dict[str, list[QtWidgets.QPushButton]] = {}

        def command_button(command_id: str, label: str) -> QtWidgets.QPushButton:
            button = _command_button(label, command_id, self.invoke_command)
            button.setMinimumWidth(150)
            button.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Fixed,
            )
            self.simple_calibration_buttons.setdefault(command_id, []).append(button)
            self.commands.primary.setdefault(command_id, button)
            self.commands.instances.setdefault(command_id, []).append(button)
            return button

        capture_page = QtWidgets.QWidget()
        capture_layout = QtWidgets.QVBoxLayout(capture_page)
        capture_layout.setContentsMargins(8, 8, 8, 8)
        capture_layout.setSpacing(6)
        capture_title = QtWidgets.QLabel(
            "1. 운용 줌을 확인하고 보드를 상·하·좌·우, 거리와 기울기를 바꿔 촬영하세요."
        )
        capture_title.setObjectName("compactContract")
        capture_title.setWordWrap(True)
        capture_layout.addWidget(capture_title)
        zoom_row = QtWidgets.QHBoxLayout()
        zoom_row.addWidget(QtWidgets.QLabel("줌 index"))
        simple_zoom_spin = QtWidgets.QSpinBox()
        simple_zoom_spin.setObjectName("simpleCalibrationZoomIndex")
        simple_zoom_spin.setRange(self.zoom_level_spin.minimum(), self.zoom_level_spin.maximum())
        simple_zoom_spin.setValue(self.zoom_level_spin.value())
        simple_zoom_spin.valueChanged.connect(self.zoom_level_spin.setValue)
        self.zoom_level_spin.valueChanged.connect(simple_zoom_spin.setValue)
        zoom_row.addWidget(simple_zoom_spin)
        simple_set_zoom = QtWidgets.QPushButton("좌우 줌 적용")
        simple_set_zoom.clicked.connect(
            lambda: self._send_calibration_zoom("set", simple_zoom_spin.value())
        )
        simple_query_zoom = QtWidgets.QPushButton("줌 확인")
        simple_query_zoom.clicked.connect(
            lambda: self._send_calibration_zoom("query", None)
        )
        zoom_row.addWidget(simple_set_zoom)
        zoom_row.addWidget(simple_query_zoom)
        zoom_row.addStretch(1)
        capture_layout.addLayout(zoom_row)
        simple_zoom_status = QtWidgets.QLabel(self.zoom_readback_label.text())
        simple_zoom_status.setObjectName("simpleCalibrationZoomStatus")
        simple_zoom_status.setWordWrap(True)
        self.simple_calibration_zoom_status = simple_zoom_status
        capture_layout.addWidget(simple_zoom_status)

        simple_preview_splitter = self.preview.build_simple_previews(capture_page)

        board_status_row = QtWidgets.QHBoxLayout()
        board_status_row.setSpacing(6)
        self.simple_calibration_board_indicators: dict[str, StatusIndicator] = {}
        for side, label in (("left", "보드 L"), ("right", "보드 R")):
            indicator = StatusIndicator(label, compact=True)
            indicator.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Fixed,
            )
            self.simple_calibration_board_indicators[side] = indicator
            board_status_row.addWidget(indicator, 1)
        capture_layout.addLayout(board_status_row)

        capture_actions = QtWidgets.QGridLayout()
        for index, (command_id, label) in enumerate(
            (
                ("camera.toggle", "미리보기"),
                ("calibration.capture_images", "Live pair 캡처"),
                ("calibration.capture_images_wiris_ftp_oz", "OZ 촬영 + 가져오기"),
                ("calibration.auto_capture", "자동 캡처"),
            )
        ):
            capture_actions.addWidget(command_button(command_id, label), index // 2, index % 2)
        capture_layout.addLayout(capture_actions)

        simple_run_status = _ElidedStatusLabel(
            "이번 촬영: 검사 0 · Stereo 저장 0 · K,D 전용 0 · 실패 0"
        )
        simple_run_status.setObjectName("simpleCalibrationRunStatus")
        self.simple_calibration_run_status = simple_run_status
        capture_layout.addWidget(simple_run_status)

        simple_set_status = _ElidedStatusLabel(
            "현재 캡처 세트: L 0/20 · R 0/20 · Stereo 저장 후보 0 · K,D 전용 L0/R0"
        )
        simple_set_status.setObjectName("simpleCalibrationSetStatus")
        self.simple_calibration_set_status = simple_set_status
        capture_layout.addWidget(simple_set_status)

        simple_capture_status = _ElidedStatusLabel(
            f"최근 상태: {self.capture_summary_label.text()}"
        )
        simple_capture_status.setObjectName("simpleCalibrationCaptureStatus")
        self.simple_calibration_capture_status = simple_capture_status
        capture_layout.addWidget(simple_capture_status)
        capture_layout.addWidget(simple_preview_splitter)
        capture_layout.addStretch(1)
        tabs.addTab(capture_page, "촬영")

        compute_page = QtWidgets.QWidget()
        compute_layout = QtWidgets.QVBoxLayout(compute_page)
        compute_layout.setContentsMargins(8, 8, 8, 8)
        compute_layout.setSpacing(8)
        compute_title = QtWidgets.QLabel(
            "2. 전체 사진으로 K,D를 계산하고 품질 통과 후 필요하면 stereo R,t를 계산하세요."
        )
        compute_title.setObjectName("compactContract")
        compute_title.setWordWrap(True)
        compute_layout.addWidget(compute_title)
        compute_actions = QtWidgets.QGridLayout()
        for index, (command_id, label) in enumerate(
            (
                ("calibration.run_intrinsic", "K,D 계산"),
                ("calibration.save_intrinsics", "K,D 저장"),
                ("calibration.run_extrinsics", "R,t 계산"),
                ("calibration.save_extrinsics", "R,t 저장"),
                ("calibration.apply_bundle", "검증된 bundle 적용"),
            )
        ):
            compute_actions.addWidget(command_button(command_id, label), index // 2, index % 2)
        compute_layout.addLayout(compute_actions)
        simple_quality_status = QtWidgets.QLabel(quality_text)
        simple_quality_status.setObjectName("simpleCalibrationQualityStatus")
        simple_quality_status.setWordWrap(True)
        self.simple_calibration_quality_status = simple_quality_status
        compute_layout.addWidget(simple_quality_status)
        compute_note = QtWidgets.QLabel(
            "FAIL은 저장·적용을 차단하고, WARNING은 고급 캘리브레이션에서 사유 검토와 명시적 승인이 필요합니다."
        )
        compute_note.setObjectName("compactContract")
        compute_note.setWordWrap(True)
        compute_layout.addWidget(compute_note)
        compute_layout.addStretch(1)
        tabs.addTab(compute_page, "계산·적용")

        profile_page = QtWidgets.QWidget()
        profile_layout = QtWidgets.QVBoxLayout(profile_page)
        profile_layout.setContentsMargins(8, 8, 8, 8)
        profile_layout.setSpacing(8)
        profile_title = QtWidgets.QLabel(
            "3. 현재 줌·해상도와 일치하는 K,D와 호환되는 R,t만 불러와 적용합니다."
        )
        profile_title.setObjectName("compactContract")
        profile_title.setWordWrap(True)
        profile_layout.addWidget(profile_title)
        profile_actions = QtWidgets.QGridLayout()
        for index, (command_id, label) in enumerate(
            (
                ("calibration.load_intrinsics", "K,D 불러오기"),
                ("calibration.load_extrinsics", "R,t 불러오기"),
                ("calibration.apply_bundle", "검증된 bundle 적용"),
            )
        ):
            profile_actions.addWidget(command_button(command_id, label), index // 2, index % 2)
        profile_layout.addLayout(profile_actions)
        simple_profile_status = QtWidgets.QLabel(
            profile_text
        )
        simple_profile_status.setObjectName("simpleCalibrationProfileStatus")
        simple_profile_status.setWordWrap(True)
        self.simple_calibration_profile_status = simple_profile_status
        profile_layout.addWidget(simple_profile_status)
        profile_layout.addStretch(1)
        tabs.addTab(profile_page, "프로파일")
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
        group = QtWidgets.QGroupBox("Capture while viewing")
        group.setObjectName("calibrationSharedCaptureSurface")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        actions = QtWidgets.QGridLayout()
        actions.setHorizontalSpacing(6)
        actions.setVerticalSpacing(5)
        capture_actions = (
            ("camera.toggle", "Preview"),
            ("calibration.capture_images", "Capture Live Pair"),
            ("calibration.auto_capture", "Start auto capture"),
            ("calibration.capture_images_wiris_ftp", "Capture HR Still (FTP)"),
            ("calibration.capture_images_wiris_ftp_oz", "Capture OZ Still (FTP)"),
            ("calibration.capture_images_wiris_oz_capture_only", "OZ 촬영"),
            ("calibration.sync_images_wiris_oz_ftp", "FTP 자동 전송"),
        )
        for index, (command_id, label) in enumerate(capture_actions):
            if command_id == "camera.toggle":
                button = _command_button(label, command_id, self.invoke_command)
                self.camera_buttons[command_id] = button
            else:
                button = self._register_capture_button(command_id, label)
                if command_id == "calibration.auto_capture":
                    button.setToolTip(
                        "Capture checkerboards only from exact NAL1 non-delta native "
                        "FrameKeys. Expired keys wait for the next candidate; pairs over "
                        "Max Live pair Δt remain per-camera K,D inputs and are excluded "
                        "from stereo R,t."
                    )
            actions.addWidget(button, index // 4, index % 4)
            if command_id == "calibration.capture_images":
                self.capture_primary_button = button
                font = button.font()
                font.setBold(True)
                button.setFont(font)
        layout.addLayout(actions)

        pair_timing = QtWidgets.QHBoxLayout()
        pair_timing_label = QtWidgets.QLabel("Max Live pair Δt")
        pair_timing_label.setToolTip(
            "Maximum absolute left/right timing difference accepted for a calibration "
            "stereo pair. Live capture uses same-host decoded-frame app-receive monotonic "
            "timestamps; FTP capture uses the synchronized CPTR host-dispatch proxy. "
            "Neither host proxy proves exposure simultaneity. Captures over the limit "
            "remain available per camera for K,D only."
        )
        pair_max_delta = QtWidgets.QDoubleSpinBox()
        pair_max_delta.setObjectName("calibrationPairMaxDeltaMs")
        pair_max_delta.setRange(0.0, 1000.0)
        pair_max_delta.setSingleStep(5.0)
        pair_max_delta.setDecimals(1)
        initial_pair_max_delta = self._initial_calibration_setup.get(
            "pair_max_delta_ms"
        )
        pair_max_delta.setValue(
            100.0
            if initial_pair_max_delta is None
            else float(initial_pair_max_delta)
        )
        pair_max_delta.setSuffix(" ms")
        pair_max_delta.setToolTip(pair_timing_label.toolTip())
        self.setup_widgets["pair_max_delta_ms"] = pair_max_delta
        pair_timing.addWidget(pair_timing_label)
        pair_timing.addWidget(pair_max_delta)
        pair_timing.addWidget(
            QtWidgets.QLabel(
                "Live: app-receive proxy · FTP: CPTR dispatch proxy · exposure unverified"
            )
        )
        pair_timing.addStretch(1)
        layout.addLayout(pair_timing)

        self.capture_summary_label = QtWidgets.QLabel(
            "L: WAITING · R: WAITING · Pair: --"
        )
        self.capture_summary_label.setObjectName("statusStrip")
        self.capture_summary_label.setWordWrap(True)
        layout.addWidget(self.capture_summary_label)
        self._capture_summary_states = {"left": "WAITING", "right": "WAITING"}
        self._capture_pair_text = "--"

        auto_group = QtWidgets.QGroupBox("Auto capture settings")
        self.auto_capture_settings_group = auto_group
        auto_group.setCheckable(True)
        auto_group.setChecked(False)
        auto_content = QtWidgets.QWidget()
        self.auto_capture_settings_content = auto_content
        auto_layout = QtWidgets.QGridLayout(auto_content)
        auto_layout.setContentsMargins(6, 4, 6, 4)
        auto_interval = QtWidgets.QDoubleSpinBox()
        auto_interval.setRange(0.1, 10.0)
        auto_interval.setSingleStep(0.1)
        auto_interval.setDecimals(1)
        auto_interval.setSuffix(" s")
        auto_interval.setValue(
            float(self._initial_calibration_setup.get("auto_interval_s") or 1.0)
        )
        self.setup_widgets["auto_interval_s"] = auto_interval
        auto_layout.addWidget(QtWidgets.QLabel("Interval"), 0, 0)
        auto_layout.addWidget(auto_interval, 0, 1)
        auto_group_layout = QtWidgets.QVBoxLayout(auto_group)
        auto_group_layout.addWidget(auto_content)
        auto_content.setVisible(False)
        auto_group.toggled.connect(auto_content.setVisible)
        layout.addWidget(auto_group)

        response_group = QtWidgets.QGroupBox("Capture command / response")
        self.capture_response_outer_group = response_group
        response_group.setCheckable(True)
        response_group.setChecked(False)
        response_content = self._build_calibration_capture_response_group()
        self.capture_response_content = response_content
        response_layout = QtWidgets.QVBoxLayout(response_group)
        response_layout.addWidget(response_content)
        response_content.setVisible(False)
        response_group.toggled.connect(response_content.setVisible)
        layout.addWidget(response_group)
        return group


    def _build_calibration_capture_response_group(self) -> QtWidgets.QGroupBox:
        """Show the raw capture lifecycle separately for the two cameras."""

        group = QtWidgets.QGroupBox("Capture command / response")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(5)
        split = QtWidgets.QHBoxLayout()
        split.setSpacing(6)
        self.calibration_capture_response_edits: dict[str, QtWidgets.QPlainTextEdit] = {}
        self.calibration_capture_response_status: dict[str, QtWidgets.QLabel] = {}
        for side, title in (("left", "Left camera"), ("right", "Right camera")):
            side_group = QtWidgets.QGroupBox(title)
            side_layout = QtWidgets.QVBoxLayout(side_group)
            side_layout.setContentsMargins(5, 5, 5, 5)
            status = QtWidgets.QLabel("대기")
            status.setObjectName("statusStrip")
            status.setWordWrap(True)
            log = QtWidgets.QPlainTextEdit()
            log.setReadOnly(True)
            log.setMaximumBlockCount(160)
            log.setMinimumHeight(92)
            log.setPlaceholderText("이 카메라의 capture 명령과 응답이 표시됩니다.")
            self.calibration_capture_response_status[side] = status
            self.calibration_capture_response_edits[side] = log
            side_layout.addWidget(status)
            side_layout.addWidget(log)
            split.addWidget(side_group, 1)
        layout.addLayout(split)
        actions = QtWidgets.QHBoxLayout()
        actions.addStretch(1)
        clear_button = QtWidgets.QPushButton("Clear capture responses")
        clear_button.clicked.connect(self._clear_calibration_capture_response_logs)
        actions.addWidget(clear_button)
        layout.addLayout(actions)
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


