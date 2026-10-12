"""Live measurement controls, results and package presentation."""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from typing import Callable
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.workstation_core.coordinate_contract import MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME
from Workstation.ui.components import EngineeringValue, WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus, StatusModel, StatusSeverity
from .camera_preview import CameraPreview, _preview_pair_height
from .shared import (
    _command_button,
    _compact_text,
    _display_number,
    _evidence,
    _live_inspection,
    _sync_table_rows,
    _table_model,
)


class MeasurementWorkspace(QtWidgets.QWidget):
    inspection_requested = QtCore.pyqtSignal(object)
    CAMERA_ASPECT_RATIO = 16.0 / 9.0
    RUN_STATUS_TEXT_LIMIT = 48

    @classmethod
    def _set_run_status_text(
        cls,
        label: QtWidgets.QLabel,
        text: str,
        *,
        detail: str = "",
    ) -> None:
        """Keep the Run summary compact while retaining its full evidence."""

        label.setText(_compact_text(text, limit=cls.RUN_STATUS_TEXT_LIMIT))
        label.setToolTip(detail or text)

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        invoke_command: Callable[[str], bool] | None = None,
        open_session_details: Callable[[], None] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.invoke_command = invoke_command
        self._sessionless = not bool(
            getattr(
                getattr(self.backend, "panel", self.backend),
                "_measurement_session_hierarchy_enabled",
                True,
            )
        )
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Measurement"))

        run_group = QtWidgets.QGroupBox("Run")
        run_layout = QtWidgets.QHBoxLayout(run_group)
        run_layout.setContentsMargins(6, 6, 6, 6)
        run_layout.setSpacing(6)
        self.control_buttons: dict[str, QtWidgets.QPushButton] = {}
        preview_button = _command_button("Preview", "camera.toggle", invoke_command)
        self.control_buttons["camera.toggle"] = preview_button
        run_layout.addWidget(preview_button)
        detection_button = _command_button("측정 시작", "detection.toggle", invoke_command)
        detection_button.setToolTip(
            "Request Measurement start/stop at any lifecycle state. Unsafe requests are "
            "reported by the runtime without disabling this control."
        )
        self.control_buttons["detection.toggle"] = detection_button
        run_layout.addWidget(detection_button)
        self.finalize_session_button = QtWidgets.QPushButton("Retry Save")
        self.finalize_session_button.setObjectName("measurementPackageFinalize")
        self.finalize_session_button.setProperty("commandId", "session.finalize")
        self.finalize_session_button.setToolTip(
            "Retry automatic persistence/sealing after a recoverable save failure."
        )
        self.finalize_session_button.clicked.connect(self._finalize_measurement_package)
        run_layout.addWidget(self.finalize_session_button)
        if self._sessionless:
            self.finalize_session_button.hide()
        package_status_layout = QtWidgets.QVBoxLayout()
        package_status_layout.setContentsMargins(0, 0, 0, 0)
        package_status_layout.setSpacing(2)
        self.session_status_label = QtWidgets.QLabel(
            "Measurement: IDLE" if self._sessionless else "Session: IDLE"
        )
        self.session_status_label.setObjectName("measurementPackageLifecycle")
        self.session_status_label.setToolTip(
            "Transient run state; stored data never owns Measurement admission."
            if self._sessionless
            else "Create the parent Session in Sessions > Current before measuring."
        )
        package_status_layout.addWidget(self.session_status_label)
        self.measurement_status_label = QtWidgets.QLabel(
            "Recording: IDLE" if self._sessionless else "Measurement: NONE"
        )
        self.measurement_status_label.setObjectName("measurementPackageCurrent")
        self.measurement_status_label.setToolTip("No active Measurement.")
        package_status_layout.addWidget(self.measurement_status_label)
        self.session_step_label = QtWidgets.QLabel(
            "Next: 측정 시작" if self._sessionless else "Next: Sessions > Current"
        )
        self.session_step_label.setObjectName("measurementPackageStep")
        self.session_step_label.setToolTip(
            "Workflow: 측정 시작 → available inputs → 측정 종료 → immediately ready"
            if self._sessionless
            else "Workflow: Sessions > Current → 측정 시작 → 측정 종료 → automatic save"
        )
        package_status_layout.addWidget(self.session_step_label)
        self.evidence_summary_label = QtWidgets.QLabel(
            "Storage: OK" if self._sessionless else "Evidence: PENDING"
        )
        self.evidence_summary_label.setObjectName("measurementPackageEvidence")
        package_status_layout.addWidget(self.evidence_summary_label)
        for status_label in (
            self.session_status_label,
            self.measurement_status_label,
            self.session_step_label,
            self.evidence_summary_label,
        ):
            status_label.setMinimumWidth(0)
            status_label.setWordWrap(False)
            status_label.setSizePolicy(
                QtWidgets.QSizePolicy.Ignored,
                QtWidgets.QSizePolicy.Preferred,
            )
        run_layout.addLayout(package_status_layout, 1)
        run_layout.addStretch(1)
        layout.addWidget(run_group)

        upstream_group = QtWidgets.QGroupBox("Inputs")
        upstream_layout = QtWidgets.QVBoxLayout(upstream_group)
        upstream_layout.setContentsMargins(6, 6, 6, 6)
        upstream_layout.setSpacing(4)
        self.preview_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.preview_splitter.setChildrenCollapsible(False)
        self.preview_row = self.preview_splitter
        preview_row = self.preview_splitter
        self.previews = {
            side: CameraPreview(side, crop_to_fill=False, lock_chrome_height=True)
            for side in ("left", "right")
        }
        for side in ("left", "right"):
            preview = self.previews[side]
            preview.setMinimumHeight(280)
            preview.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            preview_row.addWidget(preview)
        preview_row.setMinimumHeight(280)
        preview_row.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        preview_row.setStretchFactor(0, 1)
        preview_row.setStretchFactor(1, 1)
        upstream_layout.addWidget(preview_row)
        self.distance_timing_contract = QtWidgets.QLabel(
            "Timing: Settings preferred · fallback APP RECEIVE · |Δ|≤100 ms"
        )
        self.distance_timing_contract.setObjectName("compactContract")
        self.distance_timing_contract.setWordWrap(False)
        self.distance_timing_contract.setMinimumWidth(0)
        self.distance_timing_contract.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored,
            QtWidgets.QSizePolicy.Preferred,
        )
        self.distance_timing_contract.setToolTip(
            "App receive is the default. RTCP/NTP preferred selects pairs only with VERIFIED, "
            "current same-epoch evidence; any timing fault falls back to exact app receive."
        )
        upstream_layout.addWidget(self.distance_timing_contract)
        self.upstream_values: dict[str, EngineeringValue] = {}
        upstream_fields = (
            ("camera.left", "CAM L"),
            ("camera.right", "CAM R"),
            ("detection", "Detection"),
            ("synchronization", "NTP Clock"),
            ("gnss", "GNSS / RTK"),
            ("recording", "Recording"),
            ("fps", "Decoded FPS"),
        )
        facts_layout = QtWidgets.QGridLayout()
        facts_layout.setContentsMargins(0, 0, 0, 0)
        facts_layout.setHorizontalSpacing(8)
        facts_layout.setVerticalSpacing(4)
        for index, (key, label) in enumerate(upstream_fields):
            value = EngineeringValue(label)
            self.upstream_values[key] = value
            value.activated.connect(lambda key=key: self._inspect_upstream(key))
            facts_layout.addWidget(value, index // 4, index % 4)
        upstream_layout.addLayout(facts_layout)
        details = QtWidgets.QWidget()
        details_layout = QtWidgets.QVBoxLayout(details)
        details_layout.setContentsMargins(0, 0, 0, 0)
        details_layout.setSpacing(6)
        self.upstream_group = upstream_group
        self.details = details
        # Inputs must be part of the same page flow as the result/settings
        # content.  Keeping it as the first child of the details page lets the
        # outer Measurement workspace scroll move the whole page together.
        details_layout.addWidget(upstream_group)

        method_group = QtWidgets.QGroupBox("Measurement method")
        method_layout = QtWidgets.QGridLayout(method_group)
        method_layout.setContentsMargins(6, 6, 6, 6)
        method_layout.setHorizontalSpacing(8)
        self.measurement_method_combo = QtWidgets.QComboBox()
        self.measurement_method_combo.setObjectName("measurementMethodCombo")
        self.measurement_method_combo.setToolTip(
            "현재 exact K,D와 필요한 live/candidate 입력이 모두 준비된 방식만 표시합니다. "
            "선택 방식이 준비 상태를 잃으면 이 Measurement가 생성될 때 고정된 fallback "
            "순서로 자동 전환합니다."
        )
        self.measurement_method_combo.currentIndexChanged.connect(
            self._measurement_method_selected
        )
        self.measurement_method_status = QtWidgets.QLabel("준비된 측정 방식 확인 중")
        self.measurement_method_status.setObjectName("statusStrip")
        self.measurement_method_status.setWordWrap(True)
        method_layout.addWidget(QtWidgets.QLabel("Ready method"), 0, 0)
        method_layout.addWidget(self.measurement_method_combo, 0, 1)
        method_layout.addWidget(self.measurement_method_status, 1, 0, 1, 2)
        self.measurement_fill_method_combo = QtWidgets.QComboBox()
        self.measurement_fill_method_combo.setObjectName("measurementFillMethodCombo")
        self.measurement_fill_method_combo.setToolTip(
            "방식이 직접 제공하는 축은 OWN으로 잠깁니다. 미관측 회전축은 PX4, "
            "미관측 이동축은 GNSS 토글로 이 Measurement에서만 채웁니다."
        )
        self.measurement_fill_method_combo.currentIndexChanged.connect(
            self._measurement_fill_method_changed
        )
        method_layout.addWidget(QtWidgets.QLabel("DOF fill for"), 2, 0)
        method_layout.addWidget(self.measurement_fill_method_combo, 2, 1)
        fill_widget = QtWidgets.QWidget()
        fill_layout = QtWidgets.QGridLayout(fill_widget)
        fill_layout.setContentsMargins(0, 0, 0, 0)
        fill_layout.setHorizontalSpacing(6)
        fill_layout.setVerticalSpacing(3)
        dof_columns = (
            ("roll_deg", "Roll"),
            ("pitch_deg", "Pitch"),
            ("yaw_deg", "Yaw"),
            ("north_m", "North"),
            ("east_m", "East"),
            ("down_m", "Down"),
        )
        for column, (_dof, label) in enumerate(dof_columns, start=1):
            fill_layout.addWidget(QtWidgets.QLabel(label), 0, column)
        self.measurement_fill_buttons: dict[str, QtWidgets.QPushButton] = {}
        for row, side in enumerate(("left", "right"), start=1):
            fill_layout.addWidget(QtWidgets.QLabel(side.title()), row, 0)
            for column, (dof, _label) in enumerate(dof_columns, start=1):
                component = f"{side}.{dof}"
                button = QtWidgets.QPushButton("--")
                button.setCheckable(True)
                button.setObjectName(
                    "measurementFill" + side.title() + "".join(part.title() for part in dof.split("_"))
                )
                button.setProperty("component", component)
                button.toggled.connect(
                    lambda checked, component=component: self._measurement_fill_toggled(
                        component,
                        checked,
                    )
                )
                self.measurement_fill_buttons[component] = button
                fill_layout.addWidget(button, row, column)
        self.measurement_fill_status = QtWidgets.QLabel(
            "Pre 증거로 Measurement 초안이 준비된 경우 측정 시작 전에 fill source를 조정할 수 있습니다."
        )
        self.measurement_fill_status.setObjectName("statusStrip")
        self.measurement_fill_status.setWordWrap(True)
        method_layout.addWidget(fill_widget, 3, 0, 1, 2)
        method_layout.addWidget(self.measurement_fill_status, 4, 0, 1, 2)
        method_layout.setColumnStretch(1, 1)
        details_layout.addWidget(method_group)

        from Workstation.measurement.measurement_comparison import MeasurementComparisonPane
        result_reader = getattr(self.backend, "measurement_results_snapshot", None)
        self.method_results_pane = MeasurementComparisonPane(
            result_reader if callable(result_reader) else lambda *_args: {
                "rows": [], "reason": "기록 결과 없음 · results snapshot unavailable"}, self)
        details_layout.addWidget(self.method_results_pane)
        quality_group = QtWidgets.QGroupBox("Quality")
        quality_group.hide()  # Legacy adapters retained; visible results use pair records.
        quality_layout = QtWidgets.QGridLayout(quality_group)
        quality_layout.setContentsMargins(6, 6, 6, 6)
        quality_layout.setHorizontalSpacing(8)
        quality_layout.setVerticalSpacing(4)
        self.quality_labels: dict[str, QtWidgets.QLabel] = {}
        for key, label in (
            ("status", "Status"),
            ("gap", "Ray gap"),
            ("distance", "Midpoint range"),
            ("mode", "Method"),
            ("attitude", "Attitude timing"),
            ("raw_point", "Raw point"),
        ):
            value = QtWidgets.QLabel("--")
            value.setWordWrap(True)
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            if key in {"status", "attitude", "raw_point"}:
                value.setObjectName("statusStrip")
            self.quality_labels[key] = value
            if key == "status":
                quality_layout.addWidget(QtWidgets.QLabel(label), 0, 0)
                quality_layout.addWidget(value, 0, 1, 1, 3)
            elif key == "attitude":
                quality_layout.addWidget(QtWidgets.QLabel(label), 2, 0)
                quality_layout.addWidget(value, 2, 1, 1, 3)
            elif key == "raw_point":
                quality_layout.addWidget(QtWidgets.QLabel(label), 3, 0)
                quality_layout.addWidget(value, 3, 1, 1, 3)
            else:
                column = {"gap": 0, "distance": 1, "mode": 2}[key]
                quality_layout.addWidget(QtWidgets.QLabel(label), 1, column * 2)
                quality_layout.addWidget(value, 1, column * 2 + 1)
        quality_layout.setColumnStretch(1, 1)
        quality_layout.setColumnStretch(3, 1)
        details_layout.addWidget(quality_group)

        values_group = QtWidgets.QGroupBox("Result")
        values_group.hide()
        values_layout = QtWidgets.QGridLayout(values_group)
        self.values = {
            "range": EngineeringValue("Range"),
            "forward": EngineeringValue("Forward"),
            "right": EngineeringValue("Right"),
            "down": EngineeringValue("Down"),
        }
        for index, key in enumerate(("range", "forward", "right", "down")):
            self.values[key].activated.connect(lambda key=key: self._inspect_result(key))
            values_layout.addWidget(self.values[key], index // 2, index % 2)
        details_layout.addWidget(values_group)

        self.target_coordinates_group = QtWidgets.QGroupBox("Target 위치 · WGS84")
        self.target_coordinates_group.hide()
        target_layout = QtWidgets.QGridLayout(self.target_coordinates_group)
        self.target_coordinate_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate((
            ("target_latitude_deg", "Target 위도"),
            ("target_longitude_deg", "Target 경도"),
            ("target_altitude_ellipsoid_m", "Target 타원체고"),
            ("target_altitude_msl_m", "Target MSL"),
        )):
            value = EngineeringValue(label)
            value.setObjectName(key)
            value.activated.connect(lambda key=key: self._inspect_result(key))
            self.target_coordinate_values[key] = value
            target_layout.addWidget(value, index // 2, index % 2)
        self.target_coordinate_status = QtWidgets.QLabel("측정 결과 대기 중")
        self.target_coordinate_status.setObjectName("statusStrip")
        self.target_coordinate_status.setWordWrap(True)
        target_layout.addWidget(self.target_coordinate_status, 2, 0, 1, 2)
        details_layout.addWidget(self.target_coordinates_group)

        comparison_group = QtWidgets.QGroupBox("Extrinsics 분석 미리보기 (F/R/D/Rg)")
        comparison_layout = QtWidgets.QVBoxLayout(comparison_group)
        comparison_layout.setContentsMargins(6, 6, 6, 6)
        comparison_note = QtWidgets.QLabel(
            "동일 좌/우 검출 픽셀에 대해 각 외부 파라미터 소스로 계산한 결과 미리보기 · "
            "이 표는 분석 미리보기입니다. 위 Method 비교 표는 실제 pair별 계산 기록을 표시합니다."
        )
        comparison_note.setObjectName("statusStrip")
        comparison_note.setWordWrap(True)
        comparison_layout.addWidget(comparison_note)
        self.geometry_comparison_model = _table_model(
            self, ("Method", "Forward m", "Right m", "Down m", "Range m", "Status")
        )
        self.geometry_comparison_table = QtWidgets.QTableView()
        self.geometry_comparison_table.setModel(self.geometry_comparison_model)
        self.geometry_comparison_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.geometry_comparison_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.geometry_comparison_table.verticalHeader().setVisible(False)
        self.geometry_comparison_table.horizontalHeader().setStretchLastSection(True)
        self.geometry_comparison_table.setMaximumHeight(220)
        comparison_layout.addWidget(self.geometry_comparison_table)
        details_layout.addWidget(comparison_group)

        settings_group = QtWidgets.QGroupBox("Settings")
        settings_layout = QtWidgets.QVBoxLayout(settings_group)
        settings_layout.setContentsMargins(6, 6, 6, 6)
        settings_layout.setSpacing(5)
        settings = _evidence(self.backend).measurement_settings()
        self.settings_widgets: dict[str, QtWidgets.QWidget] = {}

        def combo(
            key: str,
            items: tuple[tuple[str, str], ...],
        ) -> QtWidgets.QComboBox:
            widget = QtWidgets.QComboBox()
            for label, value in items:
                widget.addItem(label, value)
            current = str(settings.get(key) or "")
            index = widget.findData(current)
            widget.setCurrentIndex(max(0, index))
            self.settings_widgets[key] = widget
            return widget

        def double_spin(
            key: str,
            minimum: float,
            maximum: float,
            decimals: int,
            suffix: str,
        ) -> QtWidgets.QDoubleSpinBox:
            widget = QtWidgets.QDoubleSpinBox()
            widget.setRange(minimum, maximum)
            widget.setDecimals(decimals)
            widget.setValue(float(settings.get(key) if settings.get(key) is not None else minimum))
            widget.setSuffix(suffix)
            self.settings_widgets[key] = widget
            return widget

        def toggle(key: str, label: str) -> QtWidgets.QPushButton:
            widget = QtWidgets.QPushButton(label)
            widget.setCheckable(True)
            widget.setChecked(bool(settings.get(key, False)))
            widget.setProperty("role", "toggle")
            self.settings_widgets[key] = widget
            return widget

        def text_edit(key: str, placeholder: str = "") -> QtWidgets.QLineEdit:
            widget = QtWidgets.QLineEdit(str(settings.get(key) or ""))
            if placeholder:
                widget.setPlaceholderText(placeholder)
            self.settings_widgets[key] = widget
            return widget

        geometry_form = QtWidgets.QGridLayout()
        geometry_form.setHorizontalSpacing(8)
        geometry_form.setVerticalSpacing(5)
        geometry_form.setColumnStretch(1, 1)
        geometry_form.setColumnStretch(3, 1)
        geometry_fields = (
            (
                "Confidence",
                double_spin("yolo_confidence", 0.01, 1.0, 3, ""),
            ),
            (
                "IoU",
                double_spin("yolo_iou", 0.01, 1.0, 3, ""),
            ),
            (
                "P1 GNSS source",
                combo(
                    "pixhawk_left_gnss_source",
                    (("GPS1", "GPS1"), ("GPS2", "GPS2")),
                ),
            ),
            (
                "P2 GNSS source",
                combo(
                    "pixhawk_right_gnss_source",
                    (("GPS1", "GPS1"), ("GPS2", "GPS2")),
                ),
            ),
        )
        for index, (label, widget) in enumerate(geometry_fields):
            row = index // 2
            column = (index % 2) * 2
            geometry_form.addWidget(QtWidgets.QLabel(label), row, column)
            geometry_form.addWidget(widget, row, column + 1)
        settings_layout.addLayout(geometry_form)
        gate_group = QtWidgets.QGroupBox("Pair gates")
        gate_form = QtWidgets.QGridLayout(gate_group)
        gate_form.setHorizontalSpacing(8)
        gate_form.setVerticalSpacing(5)
        gate_form.setColumnStretch(1, 1)
        gate_form.setColumnStretch(4, 1)
        gate_form.addWidget(toggle("class_gate_enabled", "Class gate"), 0, 0)
        gate_form.addWidget(text_edit("class_gate", "class id or name"), 0, 1, 1, 2)
        matching_toggle = toggle("matching_gate_enabled", "Time matching")
        matching_toggle.setChecked(True)
        matching_toggle.setEnabled(False)
        gate_form.addWidget(matching_toggle, 1, 0)
        time_basis_combo = combo(
            "time_basis",
            (
                ("App receive (default)", "app_receive_monotonic"),
                ("RTCP/NTP preferred · fallback App receive", "camera_rtcp"),
            ),
        )
        time_basis_combo.setEnabled(True)
        time_basis_combo.setToolTip(
            "RTCP/NTP is used only while Clock and current frame timing are usable; "
            "otherwise live distance uses exact app-receive monotonic time."
        )
        gate_form.addWidget(time_basis_combo, 1, 1)
        time_delta_toggle = toggle("time_delta_gate_enabled", "Max Δt")
        time_delta_toggle.setChecked(True)
        time_delta_toggle.setEnabled(False)
        gate_form.addWidget(time_delta_toggle, 1, 2)
        app_delta_spin = double_spin("max_pair_delta_s", 0.0, 0.100, 3, " s")
        if app_delta_spin.value() > 0.100:
            app_delta_spin.setValue(0.100)
        app_delta_spin.setToolTip(
            "Authoritative app-receive pair limit; values may be stricter than 100 ms."
        )
        gate_form.addWidget(app_delta_spin, 1, 3)
        gate_form.addWidget(toggle("vertical_delta_gate_enabled", "Vertical Δ"), 1, 4)
        gate_form.addWidget(double_spin("max_vertical_delta_px", 0.0, 5000.0, 1, " px"), 1, 5)
        settings_layout.addWidget(gate_group)

        self.measurement_settings_status = QtWidgets.QLabel(
            "Apply updates idle preview now; an active Measurement keeps timing until the next run. Save persists."
        )
        self.measurement_settings_status.setObjectName("statusStrip")
        self.measurement_settings_status.setWordWrap(True)
        settings_layout.addWidget(self.measurement_settings_status)
        settings_actions = QtWidgets.QHBoxLayout()
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("measurement.apply_settings", "Apply"),
            ("measurement.save_settings", "Save"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            settings_actions.addWidget(button)
        settings_actions.addStretch(1)
        settings_layout.addLayout(settings_actions)
        details_layout.addWidget(settings_group)

        self.history_model = QtGui.QStandardItemModel(0, 5, self)
        self.history_model.setHorizontalHeaderLabels(["timestamp_s", "Forward m", "Right m", "Down m", "Quality"])
        self.history = QtWidgets.QTableView()
        self.history.setModel(self.history_model)
        self.history.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.history.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.history.verticalHeader().setVisible(False)
        self.history.setColumnWidth(0, 150)
        for column in (1, 2, 3):
            self.history.setColumnWidth(column, 100)
        self.history.horizontalHeader().setStretchLastSection(True)
        self.history.clicked.connect(self._inspect_history_row)
        details_layout.addWidget(self.history, 1)

        self.session_details_group = QtWidgets.QGroupBox(
            "Data" if self._sessionless else "Session Details"
        )
        session_details_layout = QtWidgets.QHBoxLayout(self.session_details_group)
        session_details_layout.setContentsMargins(6, 6, 6, 6)
        session_details_label = QtWidgets.QLabel(
            "Open type-owned paths, capture tools, ULogs, and live writer health."
            if self._sessionless
            else "Review measurement packages, captures, ULogs, media, and saved-file evidence."
        )
        session_details_label.setObjectName("compactContract")
        session_details_label.setWordWrap(True)
        session_details_layout.addWidget(session_details_label, 1)
        self.session_details_button = QtWidgets.QPushButton(
            "Open Data" if self._sessionless else "Open Session Details"
        )
        self.session_details_button.setObjectName("measurementSessionDetails")
        self.session_details_button.setToolTip(
            "Open optional evidence, capture, ULog, and saved-file review tools."
        )
        self.session_details_button.setEnabled(open_session_details is not None)
        if open_session_details is not None:
            self.session_details_button.clicked.connect(open_session_details)
        session_details_layout.addWidget(self.session_details_button)
        details_layout.addWidget(self.session_details_group)
        layout.addWidget(details, 1)
        self._connect_panes()
        self._refresh_package_controls({}, getattr(self.backend, "panel", self.backend))

    @classmethod
    def preview_pair_orientation(cls, width: int, height: int) -> QtCore.Qt.Orientation:
        """Choose the pair layout that gives two 16:9 frames more usable area."""

        if width <= 0 or height <= 0:
            return QtCore.Qt.Horizontal
        return (
            QtCore.Qt.Horizontal
            if float(width) / float(height) >= cls.CAMERA_ASPECT_RATIO
            else QtCore.Qt.Vertical
        )

    def _update_preview_pair_orientation(self) -> None:
        width = self.preview_splitter.width()
        if width <= 0:
            return
        available_height = self.preview_splitter.height()
        parent = self.parentWidget()
        while parent is not None:
            if isinstance(parent, QtWidgets.QScrollArea):
                viewport_height = parent.viewport().height()
                if viewport_height > 0:
                    available_height = viewport_height
                    break
            parent = parent.parentWidget()
        horizontal_width = max(1, width - self.preview_splitter.handleWidth())
        horizontal_preview_width = max(1, horizontal_width // 2)
        horizontal_required_height = max(
            int(preview.heightForWidth(horizontal_preview_width))
            for preview in self.previews.values()
        )
        orientation = (
            QtCore.Qt.Horizontal
            if available_height >= horizontal_required_height
            else QtCore.Qt.Vertical
        )
        if self.preview_splitter.orientation() == orientation:
            return
        self.preview_splitter.setOrientation(orientation)
        self.preview_splitter.setSizes([1, 1])

    def _fit_preview_layout(self) -> None:
        self._update_preview_pair_orientation()
        required_height = _preview_pair_height(self.preview_splitter, self.previews)
        if required_height <= 0:
            return
        target_height = max(280, required_height)
        if (
            self.preview_splitter.minimumHeight() == target_height
            and self.preview_splitter.maximumHeight() == target_height
        ):
            return
        # Measurement is a fixed-aspect inspection surface.  Leaving the
        # maximum height unconstrained lets the outer QScrollArea stretch the
        # splitter, after which KeepAspectRatioByExpanding crops the live image
        # and makes it look zoomed.  Recompute the natural height on resize,
        # then clamp both bounds to that value.
        self.preview_splitter.setMinimumHeight(target_height)
        self.preview_splitter.setMaximumHeight(target_height)

    def refresh_preview_layout(self) -> None:
        """Clamp the Measurement preview after navigation changes its viewport."""

        self._fit_preview_layout()

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802 - Qt override.
        super().resizeEvent(event)
        self._update_preview_pair_orientation()
        QtCore.QTimer.singleShot(0, self._fit_preview_layout)

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: N802 - Qt override.
        super().showEvent(event)
        # The first show can establish the final central-widget geometry
        # without delivering a resize event to this page.  Fit once after the
        # layout has settled so the first camera frame cannot trigger a large
        # orientation/height correction.
        QtCore.QTimer.singleShot(0, self._fit_preview_layout)

    def _inspect_upstream(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._upstream_inspection(key),
                lambda key=key: self._upstream_inspection(key),
            )
        )

    def _upstream_inspection(self, key: str) -> dict[str, object]:
        status = self.status_model.record(key)
        return {
            "selection": self.upstream_values[key].label.text(),
            "value": status.state if status is not None else "NO DATA",
            "freshness": "--" if status is None or status.age_s is None else f"age {status.age_s:.2f}s · {status.clock}",
            "contract": "Producer contract is preserved; inspect the owning device screen for field coordinates",
            "verdict": status.detail if status is not None else "No evidence",
            "source": status.source if status is not None else "StatusModel",
        }

    def _inspect_result(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._result_inspection(key),
                lambda key=key: self._result_inspection(key),
            )
        )

    def _result_inspection(self, key: str) -> dict[str, object]:
        status = self.status_model.record("measurement")
        if key in self.target_coordinate_values:
            value = self.target_coordinate_values[key]
            return {
                "selection": value.label.text(),
                "value": f"{value.value.text()} {value.unit.text()}".strip(),
                "freshness": self.target_coordinate_status.text(),
                "contract": "WGS84 latitude/longitude degrees; ellipsoid/MSL altitude metres as labelled",
                "verdict": "계산된 표적 위치" if value.value.text() != "--" else "좌표를 계산할 입력 없음",
                "source": "same accepted DistanceHistoryPoint as Range/F/R/D",
            }
        value = self.values[key]
        return {
            "selection": f"Measurement {value.label.text()}",
            "value": f"{value.value.text()} {value.unit.text()}".strip(),
            "freshness": "--" if status is None or status.age_s is None else f"age {status.age_s:.2f}s · {status.clock}",
            "contract": f"{MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}] · metres",
            "verdict": self.quality_labels["status"].text(),
            "source": "accepted DistanceHistoryPoint",
        }

    def _inspect_history_row(self, index: QtCore.QModelIndex) -> None:
        row = index.row()
        values = [self.history_model.item(row, column).text() for column in range(self.history_model.columnCount())]
        self.inspection_requested.emit(
            {
                "selection": f"Measurement history row {row + 1}",
                "value": f"F/R/D={values[1]}/{values[2]}/{values[3]} m · quality={values[4]}",
                "freshness": f"timestamp_s={values[0]}",
                "contract": f"{MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}] · metres",
                "verdict": values[4],
                "source": "DistanceHistoryPoint",
            }
        )

    def settings_payload(self) -> dict[str, object]:
        payload: dict[str, object] = {}
        for key, widget in self.settings_widgets.items():
            if isinstance(widget, QtWidgets.QComboBox):
                payload[key] = widget.currentData()
            elif isinstance(widget, QtWidgets.QAbstractButton):
                payload[key] = widget.isChecked()
            elif isinstance(widget, QtWidgets.QLineEdit):
                payload[key] = widget.text().strip()
            elif isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox)):
                payload[key] = widget.value()
        return payload

    def _measurement_fill_method_changed(self, _index: int) -> None:
        if bool(self.measurement_fill_method_combo.property("refreshing")):
            return
        self._render_measurement_fill_controls()

    def _render_measurement_fill_controls(self) -> None:
        row = self.measurement_fill_method_combo.currentData(QtCore.Qt.UserRole + 1)
        capability = dict(row) if isinstance(row, Mapping) else {}
        components = capability.get("components")
        component_rows = dict(components) if isinstance(components, Mapping) else {}
        locked_count = 0
        fill_count = 0
        unavailable_count = 0
        for component, button in self.measurement_fill_buttons.items():
            values = component_rows.get(component)
            state = dict(values) if isinstance(values, Mapping) else {}
            button.setProperty("refreshing", True)
            try:
                provided = bool(state.get("provided", False))
                editable = bool(state.get("editable", False))
                locked = bool(state.get("locked", True))
                source = str(state.get("source") or "none")
                compatible = str(state.get("compatible_fill_source") or "")
                if provided:
                    button.setText("OWN")
                    button.setChecked(True)
                    button.setEnabled(False)
                    locked_count += 1
                elif editable:
                    button.setText("PX4" if compatible == "live_pixhawk" else "GNSS")
                    button.setChecked(bool(state.get("fill_enabled", False)))
                    button.setEnabled(not locked)
                    fill_count += int(button.isChecked())
                else:
                    button.setText("N/A")
                    button.setChecked(False)
                    button.setEnabled(False)
                    unavailable_count += 1
                readiness = "ready" if bool(state.get("source_ready", False)) else "not ready"
                button.setToolTip(
                    f"{component} · source={source} · {state.get('reason', '')} · {readiness}"
                )
            finally:
                button.setProperty("refreshing", False)
        method_id = str(capability.get("method_id") or "")
        if not method_id:
            self.measurement_fill_status.setText("구성할 Measurement method가 없습니다.")
            return
        policy_state = "LOCKED" if any(
            bool(dict(value).get("locked", True))
            for value in component_rows.values()
            if isinstance(value, Mapping) and bool(dict(value).get("editable", False))
        ) else "EDITABLE"
        self.measurement_fill_status.setText(
            f"{policy_state} · method={method_id} · own={locked_count} · "
            f"fill enabled={fill_count} · unavailable={unavailable_count}"
        )

    def _measurement_fill_toggled(self, component: str, checked: bool) -> None:
        button = self.measurement_fill_buttons[component]
        if bool(button.property("refreshing")):
            return
        method_id = self.measurement_fill_method_combo.currentData()
        configurator = getattr(self.backend, "configure_measurement_method_fill", None)
        if not method_id or not callable(configurator):
            return
        try:
            snapshot = configurator(
                {
                    "method_id": str(method_id),
                    "component": component,
                    "enabled": bool(checked),
                }
            )
        except Exception as exc:
            self.measurement_fill_status.setText(f"DOF fill 변경 실패: {exc}")
            button.setProperty("refreshing", True)
            try:
                button.setChecked(not checked)
            finally:
                button.setProperty("refreshing", False)
            return
        if isinstance(snapshot, Mapping):
            self._refresh_measurement_methods(snapshot)

    def _measurement_method_selected(self, _index: int) -> None:
        method_id = self.measurement_method_combo.currentData()
        if not method_id or bool(self.measurement_method_combo.property("refreshing")):
            return
        selector = getattr(self.backend, "select_measurement_method", None)
        if not callable(selector):
            self.measurement_method_status.setText("Measurement method selector unavailable")
            return
        try:
            snapshot = selector({"method_id": str(method_id)})
        except Exception as exc:
            self.measurement_method_status.setText(f"방식 선택 실패: {exc}")
            return
        if isinstance(snapshot, Mapping):
            self._refresh_measurement_methods(snapshot)

    def _measurement_methods_snapshot(self) -> dict[str, object]:
        reader = getattr(self.backend, "measurement_methods_snapshot", None)
        if not callable(reader):
            return {}
        try:
            value = reader()
        except Exception:
            return {}
        return dict(value) if isinstance(value, Mapping) else {}

    def _refresh_measurement_methods(self, snapshot: Mapping[str, object] | None = None) -> None:
        values = dict(snapshot or self._measurement_methods_snapshot())
        ready = values.get("ready_methods")
        rows = [dict(row) for row in ready if isinstance(row, Mapping)] if isinstance(ready, list) else []
        raw_capabilities = values.get("method_capabilities")
        capabilities = (
            [dict(row) for row in raw_capabilities if isinstance(row, Mapping)]
            if isinstance(raw_capabilities, list)
            else []
        )
        selected_method = str(values.get("selected_method_id") or "")
        selected_instance = str(values.get("selected_instance_id") or "")
        selection_reason = str(values.get("selection_reason") or "")
        reverse_yaw_widget = self.settings_widgets.get(
            "measurement_baseline_yaw_reverse"
        )
        if reverse_yaw_widget is not None:
            reverse_yaw_widget.setEnabled(not bool(values.get("fill_policy_locked", False)))
        previous_fill_method = str(self.measurement_fill_method_combo.currentData() or "")
        self.measurement_fill_method_combo.setProperty("refreshing", True)
        try:
            self.measurement_fill_method_combo.clear()
            fill_index = 0
            preferred_fill_method = previous_fill_method or selected_method
            for index, capability in enumerate(capabilities):
                method_id = str(capability.get("method_id") or "")
                label = str(capability.get("label") or method_id)
                state = "READY" if bool(capability.get("ready", False)) else "BLOCKED"
                self.measurement_fill_method_combo.addItem(f"{label} · {state}", method_id)
                self.measurement_fill_method_combo.setItemData(
                    index,
                    capability,
                    QtCore.Qt.UserRole + 1,
                )
                if method_id == preferred_fill_method:
                    fill_index = index
            self.measurement_fill_method_combo.setEnabled(bool(capabilities))
            if capabilities:
                self.measurement_fill_method_combo.setCurrentIndex(fill_index)
        finally:
            self.measurement_fill_method_combo.setProperty("refreshing", False)
        self._render_measurement_fill_controls()
        self.measurement_method_combo.setProperty("refreshing", True)
        try:
            self.measurement_method_combo.clear()
            if not rows:
                self.measurement_method_combo.addItem("준비된 측정 방식 없음", None)
                self.measurement_method_combo.setEnabled(False)
                self.measurement_method_status.setText(
                    "측정 차단 · Calibration/Alignment에서 한 방식의 K,D와 R,t 입력을 준비하세요."
                )
                return
            selected_index = 0
            for index, row in enumerate(rows):
                method_id = str(row.get("method_id") or "")
                label = str(row.get("label") or method_id)
                rank = row.get("fallback_rank")
                rank_text = "--" if rank is None else str(rank)
                self.measurement_method_combo.addItem(f"#{rank_text} · {label}", method_id)
                self.measurement_method_combo.setItemData(index, row, QtCore.Qt.UserRole + 1)
                self.measurement_method_combo.setItemData(
                    index, "\n".join(str(w) for w in row.get("warnings", ()) or ()),
                    QtCore.Qt.ToolTipRole,
                )
                if method_id == selected_method:
                    selected_index = index
            self.measurement_method_combo.setEnabled(True)
            self.measurement_method_combo.setCurrentIndex(selected_index)
            current = rows[selected_index]
            components = current.get("component_sources")
            component_text = ", ".join(
                f"{key}={value}" for key, value in dict(components or {}).items()
            )
            input_values = current.get("input_values")
            input_values = input_values if isinstance(input_values, Mapping) else {}
            branch = input_values.get("baseline_yaw_selection")
            branch = branch if isinstance(branch, Mapping) else {}
            branch_text = ""
            if branch:
                selected_yaw = branch.get("selected_yaw_deg")
                offset = branch.get("selected_offset_deg")
                mode = str(branch.get("mode") or "auto")
                selected_text = "--" if selected_yaw is None else f"{float(selected_yaw):.2f}°"
                offset_text = "--" if offset is None else f"{float(offset):+.0f}°"
                branch_text = f" · yaw={selected_text} ({offset_text}, {mode})"
                if branch.get("low_margin"):
                    branch_text += " · low margin"
                if branch.get("consensus") is False:
                    branch_text += " · P1/P2 disagree"
            self.measurement_method_status.setText(
                f"{selection_reason or 'selected'} · instance={selected_instance or current.get('instance_id', '--')}"
                + (f" · {component_text}" if component_text else "")
                + branch_text
            )
        finally:
            self.measurement_method_combo.setProperty("refreshing", False)

    def _connect_panes(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        clear_signal = getattr(panel, "camera_preview_cleared", None)
        connect = getattr(clear_signal, "connect", None)
        if callable(connect):
            connect(self._clear_camera_previews)
        for side in ("left", "right"):
            pane = getattr(panel, f"{side}_pane", None)
            signal = getattr(pane, "frame_changed", None)
            if signal is not None:
                signal.connect(
                    lambda _label, image, metadata, side=side: self._on_live_camera_frame(
                        side, image, metadata
                    )
                )
            measurement_signal = getattr(pane, "measurement_frame_changed", None)
            if measurement_signal is not None:
                measurement_signal.connect(
                    lambda _label, image, metadata, side=side: self.previews[side].set_frame(
                        image, metadata
                    )
                )
            detection_signal = getattr(pane, "detection_changed", None)
            if detection_signal is not None:
                detection_signal.connect(
                    lambda _label, detection, side=side: self.previews[side].set_detection(detection)
                )
            stats_signal = getattr(pane, "stats_changed", None)
            if stats_signal is not None:
                stats_signal.connect(lambda _label, stats, side=side: self.previews[side].set_metadata(stats))

    def _on_live_camera_frame(self, side: str, image: QtGui.QImage, metadata: object) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        pane = getattr(panel, f"{side}_pane", None)
        if bool(getattr(pane, "measurement_pair_display_active", False)):
            return
        self.previews[side].set_frame(image, metadata)

    def _clear_camera_previews(self) -> None:
        for preview in self.previews.values():
            preview.clear()

    def _refresh_preview_metadata(self, panel: object) -> None:
        metadata = getattr(panel, "camera_frame_metadata", {}) or {}
        if not isinstance(metadata, dict):
            return
        for side, preview in self.previews.items():
            current = metadata.get(side)
            if isinstance(current, dict):
                preview.set_metadata(current)

    def _refresh_quality(
        self,
        panel: object,
        measurement_status: RuntimeStatus | None = None,
    ) -> None:
        quality_sources = {
            "status": ("triangulation_quality_status_label", "3D ray triangulation 대기 중"),
            "gap": ("triangulation_quality_gap_label", "gap: --"),
            "distance": ("triangulation_quality_distance_label", "distance: --"),
            "mode": ("triangulation_quality_mode_label", "mode: --"),
            "attitude": ("triangulation_quality_attitude_label", "Attitude timing: --"),
            "raw_point": ("triangulation_quality_raw_point_label", "Raw baseline_center_frd point: --"),
        }
        for key, (attribute, default) in quality_sources.items():
            widget = getattr(panel, attribute, None)
            text = getattr(widget, "text", None)
            self.quality_labels[key].setText(str(text()) if callable(text) else default)
        if measurement_status is not None and measurement_status.state != "VALID":
            state = str(measurement_status.state or "NO DATA")
            detail = str(measurement_status.detail or "measurement evidence is not current")
            self.quality_labels["status"].setText(f"{state} · {detail}")

    def _package_method(self, *names: str) -> Callable[..., object] | None:
        panel = getattr(self.backend, "panel", self.backend)
        for owner in (self.backend, panel):
            for name in names:
                method = getattr(owner, name, None)
                if callable(method):
                    return method
        return None

    def _measurement_package_snapshot(self) -> dict[str, object] | None:
        reader = self._package_method("measurement_package_snapshot")
        if reader is None:
            return None
        try:
            value = reader()
        except Exception as exc:
            self._set_run_status_text(
                self.evidence_summary_label,
                "Evidence: ERROR",
                detail=f"Evidence: status unavailable · {exc}",
            )
            return {}
        return dict(value) if isinstance(value, Mapping) else {}

    def _measurement_session_snapshot(self) -> dict[str, object] | None:
        reader = self._package_method("measurement_session_snapshot")
        if reader is None:
            return None
        try:
            value = reader()
        except Exception as exc:
            self._set_run_status_text(
                self.evidence_summary_label,
                "Evidence: ERROR",
                detail=f"Session evidence: status unavailable · {exc}",
            )
            return {}
        return dict(value) if isinstance(value, Mapping) else {}

    def _refresh_package_controls(
        self,
        _runtime: Mapping[str, RuntimeStatus],
        _panel: object | None = None,
    ) -> None:
        if self._sessionless:
            panel = getattr(self.backend, "panel", self.backend)
            run_reader = getattr(self.backend, "measurement_run_snapshot", None)
            if not callable(run_reader):
                run_reader = getattr(panel, "measurement_run_snapshot", None)
            storage_reader = getattr(self.backend, "data_storage_snapshot", None)
            if not callable(storage_reader):
                storage_reader = getattr(panel, "data_storage_snapshot", None)
            try:
                run = run_reader() if callable(run_reader) else {}
            except Exception as exc:
                run = {"state": "ERROR", "error": str(exc)}
            try:
                storage = storage_reader() if callable(storage_reader) else {}
            except Exception as exc:
                storage = {"state": "UNAVAILABLE", "error": str(exc)}
            state = str(run.get("state") or "IDLE").upper()
            detail = str(run.get("detail") or run.get("error") or "")
            sources = run.get("sources")
            sources = sources if isinstance(sources, Mapping) else {}
            source_counts = {
                source_state: sum(
                    1
                    for value in sources.values()
                    if isinstance(value, Mapping)
                    and str(value.get("state") or "").upper() == source_state
                )
                for source_state in ("LIVE", "PENDING", "WAITING_RESOURCE", "UNAVAILABLE")
            }
            source_summary = (
                f"live={source_counts['LIVE']} · wait="
                f"{source_counts['PENDING'] + source_counts['WAITING_RESOURCE']} · "
                f"unavailable={source_counts['UNAVAILABLE']}"
            )
            self._set_run_status_text(
                self.session_status_label,
                f"Measurement: {state} · {source_summary}",
                detail=(
                    f"Measurement: {state} · {source_summary}"
                    + (f" · {detail}" if detail else "")
                ),
            )
            recording = "MANUAL" if bool(
                getattr(panel, "_manual_recording_active", False)
            ) else "MEASUREMENT" if bool(
                getattr(panel, "_automatic_session_recording", False)
            ) else "IDLE"
            presence = str(
                getattr(panel, "_last_recording_presence_state", "IDLE") or "IDLE"
            )
            self._set_run_status_text(
                self.measurement_status_label,
                f"Recording: {recording if recording != 'IDLE' else presence}",
                detail=(
                    f"Recording: {recording if recording != 'IDLE' else presence} · "
                    "SAVED means file presence only; playback integrity is not verified"
                ),
            )
            next_text = {
                "IDLE": "Next: 측정 시작",
                "QUEUED": "Next: camera owner release",
                "STARTING": "Starting…",
                "RUNNING": "Next: 측정 종료",
                "STOPPING": "Stopping camera/video…",
                "ERROR": "Next: retry Measurement",
            }.get(state, state)
            self._set_run_status_text(
                self.session_step_label,
                next_text,
                detail=str(run.get("queued_reason") or detail or next_text),
            )
            storage_state = str(storage.get("state") or "UNAVAILABLE").upper()
            dropped = int(storage.get("dropped_records") or 0)
            pending = int(storage.get("priority_pending") or 0) + int(
                storage.get("bulk_pending") or 0
            )
            retry_pending = int(storage.get("essential_retry_pending") or 0)
            self._set_run_status_text(
                self.evidence_summary_label,
                f"Storage: {storage_state} · retry={retry_pending} · drop={dropped}",
                detail=(
                    f"Storage: {storage_state} · pending={pending} · "
                    f"essential retry={retry_pending} · dropped={dropped} · "
                    f"root={storage.get('root', '--')}"
                ),
            )
            self.finalize_session_button.setEnabled(False)
            return
        package = self._measurement_package_snapshot()
        session = self._measurement_session_snapshot()
        if package is None:
            self.finalize_session_button.setEnabled(False)
            return
        self._apply_measurement_package_snapshot(package, session=session)

    @staticmethod
    def _package_artifact_counts(snapshot: Mapping[str, object]) -> dict[str, int]:
        counts = {"present": 0, "missing": 0, "failed": 0, "not_applicable": 0}
        artifacts = snapshot.get("artifacts")
        rows = artifacts if isinstance(artifacts, (tuple, list)) else ()
        for raw in rows:
            if not isinstance(raw, Mapping):
                continue
            state = str(raw.get("state", raw.get("status", "")) or "").lower()
            if state in counts:
                counts[state] += 1
        return counts

    def _apply_measurement_package_snapshot(
        self,
        package: Mapping[str, object],
        *,
        session: Mapping[str, object] | None = None,
    ) -> None:
        state = str(package.get("state", "IDLE") or "IDLE").upper()
        session_id = str(package.get("session_id", "") or "")
        measurement_id = str(package.get("measurement_id", "") or "")
        revision = str(package.get("revision", "") or "")
        detail = str(package.get("detail", package.get("error", "")) or "")
        identity = " · ".join(value for value in (session_id, revision) if value)
        session_state = str(
            (session or {}).get("state", package.get("session_state", "")) or ""
        ).upper()
        if session is not None:
            rows = (session or {}).get("measurements")
            measurement_count = len(rows) if isinstance(rows, (tuple, list)) else 0
            full_session_text = (
                f"Session: {session_state or 'IDLE'}"
                + (f" · {session_id}" if session_id else "")
                + f" · measurements={measurement_count}"
            )
            self._set_run_status_text(
                self.session_status_label,
                f"Session: {session_state or 'IDLE'} · {measurement_count} measurements",
                detail=full_session_text,
            )
            visible_measurement_state = (
                "NONE" if not measurement_id and state == "IDLE" else state
            )
            full_measurement_text = f"Measurement: {visible_measurement_state}"
            measurement_text = full_measurement_text
            if measurement_id:
                full_measurement_text += f" · {measurement_id} · {revision}"
                measurement_text += f" · {measurement_id}/{revision}"
            if detail:
                full_measurement_text += f" · {detail}"
            self._set_run_status_text(
                self.measurement_status_label,
                measurement_text,
                detail=full_measurement_text,
            )
        else:
            status_text = f"Session: {state}"
            if identity:
                status_text += f" · {identity}"
            if detail:
                status_text += f" · {detail}"
            self._set_run_status_text(
                self.session_status_label,
                f"Session: {state}",
                detail=status_text,
            )
            measurement_text = f"Measurement: {state}" + (
                f" · {revision}" if revision else ""
            )
            self._set_run_status_text(
                self.measurement_status_label,
                measurement_text,
            )

        detection_started = bool(package.get("detection_started", False))
        detection_completed = bool(package.get("detection_completed", False))
        step_text, step_detail = {
            "IDLE": (
                "Next: Sessions > Current",
                "Workflow: create or open the parent Session in Sessions > Current",
            ),
            "POST_CAPTURE": (
                "Next: Save",
                "Workflow: Measurement complete · automatic save pending",
            ),
            "VERIFYING": (
                "Saving…",
                "Workflow: draining producers and verifying inventory",
            ),
            "SEALED_COMPLETE": (
                "Saved",
                "Workflow: sealed with complete evidence",
            ),
            "SEALED_INCOMPLETE": (
                "Saved · evidence missing",
                "Workflow: sealed with documented missing evidence",
            ),
            "RECOVERABLE": (
                "Next: Retry Save",
                "Workflow: recoverable spool · retry Finalize",
            ),
            "INTEGRITY_FAILED": (
                "Integrity failed",
                "Workflow: sealed data failed integrity verification",
            ),
        }.get(state, (f"State: {state}", f"Workflow: {state}"))
        if session is not None and state == "IDLE" and session_state == "OPEN":
            step_text = "Next: 측정 시작"
            step_detail = "Workflow: session open · start creates the first Measurement"
        elif session is not None and state in {"SEALED_COMPLETE", "SEALED_INCOMPLETE"} and session_state == "OPEN":
            step_text = "Next: 다음 측정 시작"
            step_detail = "Workflow: previous Measurement sealed · start creates the next child"
        elif session is not None and session_state == "SEALED":
            step_text = "Session closed"
            step_detail = "Workflow: session sealed · start a new session when ready"
        if state == "COLLECTING":
            step_text, step_detail = (
                ("Detection running", "Workflow: Detection running")
                if detection_started and not detection_completed
                else (
                    "Next: 측정 시작",
                    "Workflow: Pre evidence draft · start Detection once",
                )
            )
        self._set_run_status_text(
            self.session_step_label,
            step_text,
            detail=step_detail,
        )

        completeness = package.get("evidence_completeness")
        completeness = completeness if isinstance(completeness, Mapping) else {}
        evidence_state = str(
            completeness.get(
                "status",
                package.get("completeness", package.get("completeness_state", "")),
            )
            or ""
        ).upper()
        if not evidence_state:
            if state == "SEALED_COMPLETE":
                evidence_state = "COMPLETE"
            elif state in {"SEALED_INCOMPLETE", "INTEGRITY_FAILED"}:
                evidence_state = "INCOMPLETE"
            else:
                evidence_state = "PENDING"
        counts = self._package_artifact_counts(package)
        missing_required = completeness.get("missing_required")
        if isinstance(missing_required, (tuple, list)):
            counts["missing"] = max(counts["missing"], len(missing_required))
        count_text = ", ".join(
            f"{key}={value}" for key, value in counts.items() if value
        )
        compact_counts = " / ".join(
            f"{value} {'ok' if key == 'present' else key.replace('_', ' ')}"
            for key, value in counts.items()
            if value
        )
        evidence_text = f"Evidence: {evidence_state}" + (
            f" · {compact_counts}" if compact_counts else ""
        )
        self._set_run_status_text(
            self.evidence_summary_label,
            evidence_text,
            detail=f"Evidence: {evidence_state}"
            + (f" · {count_text}" if count_text else ""),
        )

        finalize_method = self._package_method(
            "finalize_measurement_package", "seal_session", "finalize_session"
        )
        self.finalize_session_button.setEnabled(
            finalize_method is not None and state in {"POST_CAPTURE", "RECOVERABLE"}
        )

    def _run_package_action(
        self,
        names: tuple[str, ...],
        *,
        unavailable_text: str,
        pending_text: str,
        action: Callable[[], object] | None = None,
    ) -> None:
        method = action or self._package_method(*names)
        if method is None:
            self._set_run_status_text(
                self.session_status_label,
                "Session: unavailable",
                detail=unavailable_text,
            )
            return
        try:
            result = method()
        except Exception as exc:
            self._set_run_status_text(
                self.session_status_label,
                "Session: ERROR",
                detail=f"Session: ERROR · {exc}",
            )
            return
        package = dict(result) if isinstance(result, Mapping) else self._measurement_package_snapshot()
        if package:
            self._apply_measurement_package_snapshot(
                package,
                session=self._measurement_session_snapshot(),
            )
        else:
            self._set_run_status_text(
                self.session_status_label,
                "Session: pending",
                detail=pending_text,
            )

    def _finalize_measurement_package(self) -> None:
        self._run_package_action(
            ("finalize_measurement_package", "seal_session", "finalize_session"),
            unavailable_text="Session: Finalize control unavailable",
            pending_text="Session: Finalize requested · waiting for producer drain",
        )

    def refresh_run_controls(self, snapshot: dict[str, RuntimeStatus], panel: object) -> None:
        camera = snapshot.get("camera.left")
        detection = snapshot.get("detection")
        recording = snapshot.get("recording")
        camera_state = camera.state if camera else "STOPPED"
        detection_state = detection.state if detection else "DISABLED"
        recording_state = recording.state if recording else "IDLE"
        run_reader = getattr(self.backend, "measurement_run_snapshot", None)
        if not callable(run_reader):
            run_reader = getattr(panel, "measurement_run_snapshot", None)
        try:
            run_snapshot = run_reader() if callable(run_reader) else {}
        except Exception:
            run_snapshot = {}
        run_state = str(
            run_snapshot.get("state") if isinstance(run_snapshot, Mapping) else ""
        ).upper()
        recording_locked = recording_state in {"ACTIVE", "FINALIZING"}
        self.control_buttons["camera.toggle"].setText(
            "Stop preview" if camera_state in {"LIVE", "STARTING"} else "Preview"
        )
        self.control_buttons["detection.toggle"].setText(
            "측정 종료"
            if run_state in {"RUNNING", "STOPPING"}
            or detection_state in {"RUNNING", "STARTING"}
            else "측정 시작"
        )
        self.control_buttons["camera.toggle"].setEnabled(not recording_locked)
        # Keep the operator command available in every lifecycle state.  The
        # camera/package owners remain the source of truth for rejecting an
        # unsafe duplicate interval, active writer, unfinished save, or bad
        # RTSP input and can therefore return an actionable failure reason.
        self.control_buttons["detection.toggle"].setEnabled(self.invoke_command is not None)
        if self._sessionless:
            self._refresh_package_controls(snapshot, panel)
            return
        session_id = str(getattr(panel, "recording_session_id", "") or "")
        automatic = bool(getattr(panel, "_automatic_session_recording", False))
        if recording_state == "ACTIVE" and automatic:
            self._set_run_status_text(
                self.session_status_label,
                "Session: recording",
                detail=f"Session: recording automatically · {session_id or 'preparing'}",
            )
        elif recording_state == "FINALIZING":
            self._set_run_status_text(
                self.session_status_label,
                "Session: saving",
                detail=f"Session: saving · {session_id or 'current run'}",
            )
        elif recording_state == "RECOVERABLE":
            self._set_run_status_text(
                self.session_status_label,
                "Session: retry save",
                detail=f"Session: save needs a retry · {session_id or 'recovered run'}",
            )
        elif recording_state == "SAVED":
            self._set_run_status_text(
                self.session_status_label,
                "Session: saved",
                detail=f"Session: saved · {session_id or 'latest run'}",
            )
        elif recording_state == "ACTIVE":
            self._set_run_status_text(
                self.session_status_label,
                "Session: manual recording",
                detail=f"Session: manual evidence recording · {session_id or 'current run'}",
            )
        else:
            self._set_run_status_text(
                self.session_status_label,
                "Session: automatic",
                detail="Session: automatic · 측정 시작 creates a child; 측정 종료 saves it",
            )
        self._refresh_package_controls(snapshot, panel)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        self._refresh_preview_metadata(panel)
        self._refresh_distance_timing_contract(panel)
        measurement_status = snapshot.get("measurement")
        self._refresh_quality(panel, measurement_status)
        self._refresh_measurement_methods()
        self.refresh_run_controls(snapshot, panel)
        for key, value in self.upstream_values.items():
            status = snapshot.get(key)
            if status is None:
                value.set_value("NO DATA", source="status adapter")
                continue
            value.set_value(
                status.state,
                source=status.source,
                age_s=status.age_s,
                valid=(
                    True
                    if status.severity == StatusSeverity.OK
                    else False
                    if status.severity == StatusSeverity.BAD
                    else None
                ),
            )
        point = None
        history = getattr(panel, "distance_history", None)
        try:
            if history:
                point = list(history)[-1]
        except (TypeError, IndexError):
            point = None
        if point is not None:
            current_valid = bool(measurement_status and measurement_status.state == "VALID")
            rejected = bool(
                measurement_status
                and measurement_status.state in {"ERROR", "REJECTED"}
            )
            result_valid = True if current_valid else False if rejected else None
            result_age_s = measurement_status.age_s if measurement_status else None
            self.values["range"].set_value(getattr(point, "distance_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint", age_s=result_age_s, valid=result_valid)
            self.values["forward"].set_value(getattr(point, "forward_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint", age_s=result_age_s, valid=result_valid)
            self.values["right"].set_value(getattr(point, "right_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint", age_s=result_age_s, valid=result_valid)
            self.values["down"].set_value(getattr(point, "down_m", None), unit="m", frame=MEASUREMENT_FRAME, source="DistanceHistoryPoint", age_s=result_age_s, valid=result_valid)

        else:
            for value in self.values.values():
                value.set_value(None, unit="m", frame=MEASUREMENT_FRAME, source="no accepted measurement")
        self._refresh_target_coordinates(point)
        rows = []
        try:
            rows = list(history or [])[-100:]
        except TypeError:
            rows = []
        history_rows = tuple(
            (
                getattr(row, "timestamp_s", "--"),
                getattr(row, "forward_m", "--"),
                getattr(row, "right_m", "--"),
                getattr(row, "down_m", "--"),
                getattr(row, "quality", getattr(row, "method", "--")),
            )
            for row in rows
        )
        _sync_table_rows(self.history_model, history_rows)

        comparison_rows = tuple(
            (
                row.get("method", "--"),
                _display_number(row.get("forward_m"), digits=4),
                _display_number(row.get("right_m"), digits=4),
                _display_number(row.get("down_m"), digits=4),
                _display_number(row.get("range_m"), digits=4),
                row.get("status", "--"),
            )
            for row in (getattr(panel, "geometry_comparison_rows", ()) or ())
        )
        _sync_table_rows(self.geometry_comparison_model, comparison_rows)

    def _refresh_target_coordinates(self, point: object | None) -> None:
        """Render the latest accepted result beside Range without live GNSS reads."""
        timestamp = getattr(point, "timestamp_s", None)
        age_s = (
            max(0.0, time.monotonic() - float(timestamp))
            if isinstance(timestamp, (int, float)) and math.isfinite(timestamp)
            else None
        )
        for key, value in self.target_coordinate_values.items():
            raw = getattr(point, key, None)
            number = (
                float(raw) if isinstance(raw, (int, float))
                and not isinstance(raw, bool) and math.isfinite(raw) else None
            )
            is_angle = key.endswith("_deg")
            value.set_value(
                None if number is None else f"{number:.9f}" if is_angle else f"{number:.3f}",
                unit="deg" if is_angle else "m",
                frame="WGS84" if is_angle else "WGS84 ellipsoid height" if "ellipsoid" in key else "MSL height",
                source="same measurement as Range/F/R/D",
            )
        if point is None:
            text = "측정 결과 대기 중"
        else:
            text = "마지막 측정 결과"
            if age_s is not None:
                text += f" · {age_s:.1f}s 전"
            if self.target_coordinate_values["target_latitude_deg"].value.text() == "--":
                text += " · 월드 좌표 계산 입력 없음"
            elif self.target_coordinate_values["target_altitude_msl_m"].value.text() == "--":
                text += " · MSL 기준 없음"
        self.target_coordinate_status.setText(text)

    def _refresh_distance_timing_contract(self, panel: object) -> None:
        requested_reader = getattr(panel, "_yolo_time_basis", None)
        requested = (
            str(requested_reader())
            if callable(requested_reader)
            else str(
                getattr(
                    getattr(panel, "settings", None),
                    "yolo_gate_time_basis",
                    "app_receive_monotonic",
                )
            )
        )
        result = getattr(panel, "_coordinated_pairing_result", None)
        timing = getattr(result, "timing", None)
        effective = str(getattr(timing, "effective_basis", requested) or requested)
        fallback_reason = str(getattr(timing, "fallback_reason", "") or "")
        requested_text = "RTCP/NTP" if requested == "camera_rtcp" else "APP RECEIVE"
        effective_text = "RTCP/NTP" if effective == "camera_rtcp" else "APP RECEIVE"
        suffix = f" · fallback={fallback_reason}" if fallback_reason else ""
        basis_text = (
            requested_text
            if requested_text == effective_text
            else f"{requested_text} → {effective_text}"
        )
        self.distance_timing_contract.setText(
            f"Timing: {basis_text}{suffix} · |Δ|≤100 ms"
        )
        self.distance_timing_contract.setToolTip(
            "The active Measurement request is frozen at start. RTCP/NTP is selected only with "
            "VERIFIED/current same-epoch evidence; otherwise exact app receive is authoritative."
        )
