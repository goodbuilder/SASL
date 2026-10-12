"""Calibration workspace composition and the supported legacy view edge."""
from __future__ import annotations

from collections.abc import Mapping
from typing import Callable
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.workstation_core.coordinate_contract import CAMERA_FRAME, MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME
from Workstation.workstation_core.settings import DEFAULT_CALIBRATION_ROOT
from Workstation.ui.components import EngineeringValue, WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus
from .shared import _command_button, _display_number, _evidence, _sync_table_rows, _table_model
from .calibration_parts.widgets import _ContentSizedOperationTabs, _ElidedStatusLabel, CalibrationErrorPlot
from .calibration_parts.commands import CalibrationCommandWidgets
from .calibration_parts.compatibility import ViewAlias
from .calibration_parts.capture import CalibrationCaptureSurface
from .calibration_parts.preview import CalibrationPreview
from .calibration_parts.inputs import CalibrationInputPicker
from .calibration_parts.quality import CalibrationQualityPanel
from .calibration_parts.validation import CalibrationValidationPanel
from .calibration_parts.contracts import PreviewChannel, ValidationActions


class _CalibrationViewSources:
    """Outer legacy adapter. Parts receive named readers, actions and signals."""

    def __init__(self, backend):
        self.backend = backend

    def setup_snapshot(self):
        return dict(_evidence(self.backend).calibration_setup())

    def apply_setup_command(self):
        return getattr(self.backend, "calibration_apply_setup", None)

    def zoom_command(self):
        return getattr(self.backend, "advanced_send_zoom", None)

    def preview_command(self):
        return getattr(self.backend, "advanced_refresh_calibration_preview", None)

    def preview_channels(self):
        panel = getattr(self.backend, "panel", self.backend)
        return tuple(
            PreviewChannel(side, getattr(pane, "frame_changed", None),
                           getattr(pane, "detection_changed", None), getattr(pane, "stats_changed", None))
            for side in ("left", "right")
            for pane in (getattr(panel, f"{side}_pane", None),)
        )

    def preview_signals(self):
        panel = getattr(self.backend, "panel", self.backend)
        return (getattr(panel, "camera_preview_cleared", None),
                getattr(panel, "calibration_preview_changed", None))

    def capture_signal(self):
        return getattr(getattr(self.backend, "panel", None), "calibration_capture_event", None)

    def diagnostics_signal(self):
        return getattr(getattr(self.backend, "panel", None), "calibration_diagnostics_changed", None)

    def validation_actions(self):
        return ValidationActions(
            configure=getattr(self.backend, "configure_workstation_checkerboard_validation", None),
            load_reference=getattr(self.backend, "load_workstation_checkerboard_validation_json", None),
            run=getattr(self.backend, "run_workstation_checkerboard_validation", None),
            snapshot=getattr(self.backend, "workstation_checkerboard_validation_snapshot", None),
            save=getattr(self.backend, "save_workstation_checkerboard_validation_result", None),
            apply=getattr(self.backend, "apply_workstation_checkerboard_validation_result", None),
        )

    def capture_snapshot(self) -> dict[str, object]:
        """Read the Calibration owner's capture status without deriving truth."""

        panel = getattr(self.backend, "panel", self.backend)
        for owner in (self.backend, panel):
            reader = getattr(owner, "workstation_calibration_capture_snapshot", None)
            if not callable(reader):
                continue
            try:
                snapshot = reader()
            except Exception as exc:
                return {
                    "schema": "workstation-calibration-capture-status-v1",
                    "capture_set_id": "",
                    "detection": {},
                    "run": {},
                    "capture_set": {
                        "state": "UNAVAILABLE",
                        "error": f"capture snapshot unavailable: {exc}",
                    },
                    "last": {"message": f"Capture status unavailable: {exc}"},
                }
            if isinstance(snapshot, Mapping):
                return dict(snapshot)
        adapter = _evidence(self.backend)
        calibration_reader = getattr(adapter, "calibration", None)
        calibration = calibration_reader() if callable(calibration_reader) else {}
        snapshot = calibration.get("capture_status") if isinstance(calibration, Mapping) else None
        return dict(snapshot) if isinstance(snapshot, Mapping) else {}


    def quality_snapshot(self) -> dict[str, object]:
        """Read one owner-produced quality snapshot without deriving metrics."""

        panel = getattr(self.backend, "panel", self.backend)
        for owner in (self.backend, panel):
            for name in (
                "calibration_quality_snapshot",
                "workstation_calibration_quality_snapshot",
                "checkerboard_calibration_quality_snapshot",
            ):
                reader = getattr(owner, name, None)
                if not callable(reader):
                    continue
                try:
                    snapshot = reader()
                except Exception as exc:
                    return {
                        "state": "UNVERIFIED",
                        "reasons": (f"quality snapshot unavailable: {exc}",),
                        "source": f"{type(owner).__name__}.{name}",
                    }
                if isinstance(snapshot, Mapping):
                    return CalibrationQualityPanel._calibration_quality_mapping(snapshot)
        for owner in (panel, self.backend):
            for name in ("calibration_quality", "last_calibration_quality"):
                snapshot = getattr(owner, name, None)
                if isinstance(snapshot, Mapping):
                    return CalibrationQualityPanel._calibration_quality_mapping(snapshot)
        result = getattr(panel, "last_calibration_result", None)
        snapshot = getattr(result, "quality_snapshot", None) if result is not None else None
        if isinstance(snapshot, Mapping):
            return CalibrationQualityPanel._calibration_quality_mapping(snapshot)
        return {}


    def diagnostic_rows(self) -> tuple[dict[str, object], ...]:
        panel = getattr(self.backend, "panel", self.backend)
        rows = getattr(panel, "calibration_image_diagnostics", ())
        if not rows:
            result = getattr(panel, "last_calibration_result", None)
            rows = getattr(result, "image_diagnostics", ()) if result is not None else ()
        return tuple(dict(row) for row in rows if isinstance(row, Mapping))



class CalibrationWorkspace(QtWidgets.QWidget):
    """Compose independent calibration views over the existing backend contract."""

    _CALIBRATION_QUALITY_GATED_COMMANDS = CalibrationQualityPanel._CALIBRATION_QUALITY_GATED_COMMANDS

    def __init__(
        self,
        backend: object,
        open_advanced: Callable[[], None],
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.invoke_command = invoke_command
        self.setup_widgets: dict[str, QtWidgets.QWidget] = {}
        self._initial_calibration_setup = dict(_evidence(self.backend).calibration_setup())
        self._commands = CalibrationCommandWidgets()
        sources = _CalibrationViewSources(backend)
        clear_signal, overlay_signal = sources.preview_signals()
        self._preview = CalibrationPreview(
            parent=self, preview_command=sources.preview_command,
            channels=sources.preview_channels(), clear_signal=clear_signal, overlay_signal=overlay_signal,
        )
        self._capture = CalibrationCaptureSurface(
            self._commands, self._preview, self._initial_calibration_setup,
            self._sync_simple_calibration_labels, parent=self,
            read_capture=sources.capture_snapshot, zoom_command=sources.zoom_command,
            capture_signal=sources.capture_signal(),
        )
        self._capture.invoke_command = invoke_command
        self._inputs = self._new_input_picker(parent=self)
        self.destroyed.connect(self._inputs.cancel_pending_thumbnails)
        self._quality = CalibrationQualityPanel(
            self._commands,
            read_selection=self._inputs.selection_snapshot,
            selection_saved=self._inputs.refresh_selection,
            refresh_labels=self._sync_simple_calibration_labels,
            read_quality=sources.quality_snapshot, read_diagnostics=sources.diagnostic_rows,
            read_setup=sources.setup_snapshot, diagnostics_signal=sources.diagnostics_signal(),
            default_root=DEFAULT_CALIBRATION_ROOT, parent=self,
        )
        self._validation = CalibrationValidationPanel(
            self._commands, read_actions=sources.validation_actions,
            present_quality=self._quality.apply_calibration_quality_snapshot,
            warning_confirmed=lambda: self._quality._calibration_quality_warning_confirmed,
            parent=self,
        )

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Calibration"))

        advanced_content = QtWidgets.QWidget()
        advanced_layout = QtWidgets.QVBoxLayout(advanced_content)
        advanced_layout.setContentsMargins(6, 6, 6, 6)
        advanced_layout.setSpacing(6)

        advanced_layout.addWidget(self._capture.zoom_group)

        camera_group = QtWidgets.QGroupBox("Shared calibration inputs")
        camera_layout = QtWidgets.QVBoxLayout(camera_group)
        camera_layout.setContentsMargins(6, 6, 6, 6)
        camera_layout.setSpacing(5)
        camera_layout.setAlignment(QtCore.Qt.AlignTop)

        calibration_session_group = self._inputs._build_calibration_session_group()
        calibration_file_group = self._inputs._build_calibration_file_selection_group()
        self.operation_tabs = self._build_operation_tabs()
        self.capture_surface = self._capture._build_shared_capture_surface()
        self.setup_widgets.update(self._capture.setup_widgets)
        camera_layout.addWidget(self.operation_tabs)
        camera_layout.addWidget(self._preview.camera_mode_tabs)
        camera_layout.addWidget(self.capture_surface)
        camera_layout.addWidget(calibration_session_group)
        camera_layout.addWidget(calibration_file_group)
        self._calibration_workflow_syncing = False
        self._calibration_workflow_index = 0
        self.operation_tabs.currentChanged.connect(
            self._on_calibration_kind_changed
        )
        self.intrinsics_operation_tabs.currentChanged.connect(
            self._on_calibration_workflow_changed
        )
        self.extrinsics_operation_tabs.currentChanged.connect(
            self._on_calibration_workflow_changed
        )
        self._sync_calibration_capture_surface()
        self._quality.refresh_calibration_quality()
        advanced_layout.addWidget(camera_group)

        self._capture._connect_calibration_capture_event_signal()
        self._quality._connect_calibration_diagnostics_signal()
        self._preview._connect_camera_panes()
        self._preview._connect_calibration_preview_signal()

        profile_group = QtWidgets.QGroupBox("Active calibration bundle")
        profile_layout = QtWidgets.QGridLayout(profile_group)
        self.profile_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("state", "State"),
                ("profile_path", "K,D profile"),
                ("zoom", "Zoom"),
                ("extrinsics_profile_path", "R,t profile"),
                ("extrinsics_source_zoom", "R,t source zoom"),
                ("combination", "Combination"),
                ("zoom_readback", "Actual zoom"),
                ("validation_mode", "Calculation mode"),
                ("status_label", "Result"),
                ("valid_pair_count", "Valid pairs"),
                ("progress", "Progress"),
            )
        ):
            value = EngineeringValue(label)
            self.profile_values[key] = value
            profile_layout.addWidget(value, index // 2, index % 2)
        advanced_layout.addWidget(profile_group)

        intrinsic_group = QtWidgets.QGroupBox("Intrinsics result")
        intrinsic_layout = QtWidgets.QVBoxLayout(intrinsic_group)
        self.intrinsic_model = _table_model(
            self,
            (
                "Camera",
                "Grid px",
                "fx px",
                "fy px",
                "cx px",
                "cy px",
                "Reproj px",
                "Images",
                "Frame",
            ),
        )
        self.intrinsic_table = QtWidgets.QTableView()
        self.intrinsic_table.setModel(self.intrinsic_model)
        self.intrinsic_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.intrinsic_table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectRows
        )
        self.intrinsic_table.verticalHeader().setVisible(False)
        self.intrinsic_table.setMaximumHeight(108)
        self.intrinsic_table.horizontalHeader().setStretchLastSection(True)
        intrinsic_layout.addWidget(self.intrinsic_table)
        self.intrinsics_result_group = intrinsic_group
        self.intrinsics_page_layout.addWidget(intrinsic_group)

        self.extrinsic_value = EngineeringValue("Stereo R/T")
        metrics_group = QtWidgets.QGroupBox("Extrinsics result")
        metrics_layout = QtWidgets.QVBoxLayout(metrics_group)
        metrics_layout.addWidget(self.extrinsic_value)
        self.calibration_contract = QtWidgets.QLabel()
        self.calibration_contract.setObjectName("compactContract")
        self.calibration_contract.setWordWrap(True)
        self.calibration_contract.setTextInteractionFlags(
            QtCore.Qt.TextSelectableByMouse
        )
        metrics_layout.addWidget(self.calibration_contract)
        self.extrinsics_result_group = metrics_group
        self.extrinsics_page_layout.addWidget(metrics_group)

        setup_group = QtWidgets.QGroupBox("Shared checkerboard setup")
        setup_layout = QtWidgets.QVBoxLayout(setup_group)
        setup_layout.setContentsMargins(6, 6, 6, 6)
        setup_layout.setSpacing(5)
        setup = _evidence(self.backend).calibration_setup()
        self._initial_calibration_setup = dict(setup)

        def setup_spin(
            key: str,
            minimum: int,
            maximum: int,
            suffix: str = "",
        ) -> QtWidgets.QSpinBox:
            widget = QtWidgets.QSpinBox()
            widget.setRange(minimum, maximum)
            value = setup.get(key)
            default_value = 20 if key == "calculation_pair_count" else minimum
            widget.setValue(int(value if value is not None else default_value))
            widget.setSuffix(suffix)
            if key == "calculation_pair_count":
                widget.setSpecialValueText("all pairs")
            self.setup_widgets[key] = widget
            return widget

        def setup_double(
            key: str,
            minimum: float,
            maximum: float,
            suffix: str = "",
        ) -> QtWidgets.QDoubleSpinBox:
            widget = QtWidgets.QDoubleSpinBox()
            widget.setRange(minimum, maximum)
            widget.setDecimals(3)
            value = setup.get(key)
            widget.setValue(float(value if value is not None else minimum))
            widget.setSuffix(suffix)
            self.setup_widgets[key] = widget
            return widget

        def setup_text(
            key: str,
            placeholder: str = "",
        ) -> QtWidgets.QLineEdit:
            widget = QtWidgets.QLineEdit(str(setup.get(key) or ""))
            if placeholder:
                widget.setPlaceholderText(placeholder)
            self.setup_widgets[key] = widget
            return widget

        def setup_combo(key: str) -> QtWidgets.QComboBox:
            widget = QtWidgets.QComboBox()
            widget.addItem(
                "Pixhawk attitude + GPS/RTK baseline", "pixhawk"
            )
            widget.addItem("Checkerboard stereo R/T", "checkerboard")
            current = str(setup.get(key) or "pixhawk")
            widget.setCurrentIndex(max(0, widget.findData(current)))
            self.setup_widgets[key] = widget
            return widget

        setup_form = QtWidgets.QGridLayout()
        setup_form.setHorizontalSpacing(8)
        setup_form.setVerticalSpacing(5)
        setup_form.setColumnStretch(1, 1)
        setup_form.setColumnStretch(3, 1)
        for index, (label, widget) in enumerate(
            (
                ("Inner columns", setup_spin("inner_cols", 2, 30)),
                ("Inner rows", setup_spin("inner_rows", 2, 30)),
                (
                    "Square size",
                    setup_double("square_size_mm", 1.0, 500.0, " mm"),
                ),
                ("Minimum images", setup_spin("min_images", 3, 100)),
                (
                    "Calculation pairs",
                    setup_spin(
                        "calculation_pair_count", 0, 10000, " pair"
                    ),
                ),
            )
        ):
            row = index // 2
            column = (index % 2) * 2
            setup_form.addWidget(QtWidgets.QLabel(label), row, column)
            setup_form.addWidget(widget, row, column + 1)
        setup_layout.addLayout(setup_form)

        path_form = QtWidgets.QGridLayout()
        path_form.setHorizontalSpacing(8)
        path_form.setVerticalSpacing(5)
        path_form.setColumnStretch(1, 1)
        path_form.setColumnStretch(3, 1)
        path_form.addWidget(QtWidgets.QLabel("Calibration root"), 0, 0)
        path_form.addWidget(
            setup_text("calibration_root", DEFAULT_CALIBRATION_ROOT),
            0,
            1,
            1,
            3,
        )
        path_form.addWidget(QtWidgets.QLabel("Calculation mode"), 1, 0)
        path_form.addWidget(setup_combo("validation_mode"), 1, 1)
        setup_layout.addLayout(path_form)
        setup_contract = QtWidgets.QLabel(
            "Square: mm · pixels/intrinsics: OpenCV optical RDF · "
            "X_right = R_left_to_right X_left + T_left_to_right_m · "
            "save zoom comes from the captured camera readback"
        )
        setup_contract.setObjectName("compactContract")
        setup_contract.setWordWrap(True)
        setup_layout.addWidget(setup_contract)
        setup_actions = QtWidgets.QHBoxLayout()
        for command_id, label in (
            ("calibration.apply_setup", "Apply setup"),
            ("calibration.save_setup", "Save setup"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self._commands.primary[command_id] = button
            setup_actions.addWidget(button)
        setup_actions.addStretch(1)
        setup_layout.addLayout(setup_actions)
        advanced_layout.addWidget(setup_group)

        validation_group = QtWidgets.QGroupBox(
            "Advanced intrinsics validation (optional)"
        )
        self.checkerboard_validation_outer_group = validation_group
        validation_group.setCheckable(True)
        validation_group.setChecked(False)
        validation_layout = QtWidgets.QVBoxLayout(validation_group)
        validation_content = self._validation._build_checkerboard_validation_group()
        validation_layout.addWidget(validation_content)
        validation_content.setVisible(False)
        validation_group.toggled.connect(validation_content.setVisible)
        validation_group.toggled.connect(
            lambda _checked: self._quality._apply_calibration_quality_gate()
        )
        self.intrinsics_page_layout.addWidget(validation_group)
        self.intrinsics_page_layout.addStretch(1)
        self.extrinsics_page_layout.addStretch(1)
        self._sync_calibration_capture_surface()

        handoff_button = QtWidgets.QPushButton("Advanced Zoom")
        handoff_button.setToolTip(
            "Open Advanced camera controls for physical zoom commands."
        )
        handoff_button.clicked.connect(open_advanced)
        advanced_layout.addStretch(1)
        advanced_layout.addWidget(handoff_button, alignment=QtCore.Qt.AlignRight)

        self.simple_calibration_tabs = self._capture._build_simple_calibration_tabs(
            self._quality.calibration_quality_gate_label.text(),
            self.profile_values["state"].value.text(),
        )
        layout.addWidget(self.simple_calibration_tabs)
        self._capture.apply_calibration_capture_snapshot(
            self._capture.calibration_capture_snapshot()
        )

        advanced_group = QtWidgets.QGroupBox()
        advanced_group.setObjectName("advancedCalibrationGroup")
        advanced_group.setToolTip(
            "파일별 선택, subset 품질, K/D·R/t 상세, 세션 경로를 검토합니다."
        )
        advanced_group_layout = QtWidgets.QVBoxLayout(advanced_group)
        advanced_group_layout.setContentsMargins(6, 6, 6, 6)
        advanced_toggle = QtWidgets.QToolButton()
        advanced_toggle.setObjectName("advancedCalibrationToggle")
        advanced_toggle.setText("고급 캘리브레이션")
        advanced_toggle.setCheckable(True)
        advanced_toggle.setChecked(False)
        advanced_toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        advanced_toggle.setArrowType(QtCore.Qt.RightArrow)
        advanced_toggle.toggled.connect(
            lambda checked: advanced_toggle.setArrowType(
                QtCore.Qt.DownArrow if checked else QtCore.Qt.RightArrow
            )
        )
        advanced_group_layout.addWidget(advanced_toggle)
        advanced_group_layout.addWidget(advanced_content)
        advanced_content.setVisible(False)
        advanced_toggle.toggled.connect(advanced_content.setVisible)
        self.advanced_calibration_group = advanced_group
        self.advanced_calibration_toggle = advanced_toggle
        self.advanced_calibration_content = advanced_content
        layout.addWidget(advanced_group)
        layout.addStretch(1)


    def _sync_simple_calibration_labels(self) -> None:
        capture = vars(self).get("_capture")
        if capture is None:
            return
        for target_name, source in (
            ("simple_calibration_zoom_status", capture.zoom_readback_label),
            ("simple_calibration_quality_status", getattr(vars(self).get("_quality"), "calibration_quality_gate_label", None)),
        ):
            target = getattr(capture, target_name, None)
            if isinstance(target, QtWidgets.QLabel) and isinstance(source, QtWidgets.QLabel):
                target.setText(source.text())
        profile = getattr(capture, "simple_calibration_profile_status", None)
        state = getattr(getattr(self, "profile_values", {}).get("state"), "value", None)
        if isinstance(profile, QtWidgets.QLabel) and isinstance(state, QtWidgets.QLabel):
            profile.setText(f"Active bundle: {state.text()}")


    def _build_operation_tabs(self) -> QtWidgets.QTabWidget:
        kind_tabs = _ContentSizedOperationTabs()
        kind_tabs.setObjectName("calibrationKindTabs")
        kind_tabs.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Maximum,
        )
        # Command instances are owned by the registry shared with the components.

        def command_button(
            command_id: str,
            label: str,
        ) -> QtWidgets.QPushButton:
            button = _command_button(
                label, command_id, self.invoke_command
            )
            button.setMinimumWidth(0)
            button.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Fixed,
            )
            self._commands.primary.setdefault(command_id, button)
            self._commands.instances.setdefault(
                command_id, []
            ).append(button)
            return button

        def workflow_tabs(
            *,
            kind: str,
            capture_actions: tuple[tuple[str, str], ...],
            compute_actions: tuple[tuple[str, str], ...],
            profile_actions: tuple[tuple[str, str], ...],
            capture_hint: str,
            include_auto_capture: bool,
            include_intrinsic_quality: bool,
        ) -> QtWidgets.QTabWidget:
            tabs = _ContentSizedOperationTabs()
            tabs.setObjectName(f"calibration{kind}WorkflowTabs")
            tabs.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Maximum,
            )
            sections = (
                ("Capture", capture_actions),
                ("Compute", compute_actions),
                ("Profiles", profile_actions),
            )
            for tab_label, actions in sections:
                page = QtWidgets.QWidget()
                page_layout = QtWidgets.QVBoxLayout(page)
                page_layout.setContentsMargins(6, 6, 6, 6)
                page_layout.setSpacing(6)
                actions_layout = QtWidgets.QGridLayout()
                columns = 4 if tab_label == "Capture" else 3
                for index, (command_id, label) in enumerate(actions):
                    actions_layout.addWidget(
                        command_button(command_id, label),
                        index // columns,
                        index % columns,
                    )
                page_layout.addLayout(actions_layout)
                if tab_label == "Capture":
                    hint = QtWidgets.QLabel(capture_hint)
                    hint.setObjectName("compactContract")
                    hint.setWordWrap(True)
                    hint.setFixedHeight(hint.fontMetrics().lineSpacing() * 2 + 6)
                    page_layout.addWidget(hint)
                if tab_label == "Capture" and include_auto_capture:
                    auto_group = QtWidgets.QGroupBox(
                        "Automatic capture"
                    )
                    auto_layout = QtWidgets.QGridLayout(auto_group)
                    auto_layout.setHorizontalSpacing(8)
                    auto_layout.setVerticalSpacing(5)
                    auto_layout.setColumnStretch(1, 1)
                    auto_interval = QtWidgets.QDoubleSpinBox()
                    auto_interval.setRange(0.1, 10.0)
                    auto_interval.setSingleStep(0.1)
                    auto_interval.setDecimals(1)
                    auto_interval.setSuffix(" s")
                    auto_interval.setValue(
                        float(
                            self._initial_calibration_setup.get(
                                "auto_interval_s"
                            )
                            or 1.0
                        )
                    )
                    self.setup_widgets["auto_interval_s"] = (
                        auto_interval
                    )
                    auto_button = command_button(
                        "calibration.auto_capture",
                        "Start auto capture",
                    )
                    auto_button.setToolTip(
                        "Capture only new left/right checkerboard "
                        "detections whose decoded-frame app-receive timestamps differ "
                        "by no more than the configured Max Live pair Δt."
                    )
                    auto_layout.addWidget(
                        QtWidgets.QLabel("Interval"), 0, 0
                    )
                    auto_layout.addWidget(auto_interval, 0, 1)
                    auto_layout.addWidget(
                        auto_button, 1, 0, 1, 2
                    )
                    page_layout.addWidget(auto_group)
                    page_layout.addWidget(
                        self._capture._build_calibration_capture_response_group()
                    )
                if (
                    tab_label == "Compute"
                    and include_intrinsic_quality
                ):
                    page_layout.addWidget(
                        self._quality._build_calibration_quality_group()
                    )
                    page_layout.addWidget(
                        self._quality._build_calibration_outlier_group()
                    )
                page_layout.addStretch(1)
                tabs.addTab(page, tab_label)
            return tabs

        intrinsics_page = QtWidgets.QWidget()
        self.intrinsics_page_layout = QtWidgets.QVBoxLayout(
            intrinsics_page
        )
        self.intrinsics_page_layout.setContentsMargins(6, 6, 6, 6)
        self.intrinsics_page_layout.setAlignment(QtCore.Qt.AlignTop)
        intrinsics_contract = QtWidgets.QLabel(
            "Per-camera image pixels → K,D in OpenCV optical RDF. "
            "Profiles are camera-, native-raster-, focus-, and "
            "zoom-specific."
        )
        intrinsics_contract.setObjectName("compactContract")
        intrinsics_contract.setWordWrap(True)
        self.intrinsics_contract_label = intrinsics_contract
        self.intrinsics_page_layout.addWidget(intrinsics_contract)
        self.intrinsics_operation_tabs = workflow_tabs(
            kind="Intrinsics",
            capture_actions=(),
            compute_actions=(
                (
                    "calibration.run_intrinsic",
                    "Run calibration (K,D)",
                ),
                (
                    "calibration.save_intrinsics",
                    "Save Intrinsics",
                ),
            ),
            profile_actions=(
                (
                    "calibration.load_intrinsics",
                    "Load Intrinsics",
                ),
            ),
            capture_hint=(
                "Each camera contributes its own checkerboard image "
                "set. Left/right files remain separate inputs to the "
                "two K,D solves."
            ),
            include_auto_capture=False,
            include_intrinsic_quality=True,
        )
        self.intrinsics_page_layout.addWidget(
            self.intrinsics_operation_tabs
        )
        kind_tabs.addTab(intrinsics_page, "Intrinsics")

        extrinsics_page = QtWidgets.QWidget()
        self.extrinsics_page_layout = QtWidgets.QVBoxLayout(
            extrinsics_page
        )
        self.extrinsics_page_layout.setContentsMargins(6, 6, 6, 6)
        self.extrinsics_page_layout.setAlignment(QtCore.Qt.AlignTop)
        self.extrinsics_bundle_contract_label = QtWidgets.QLabel(
            "Input bundle: Left K,D + Right K,D + synchronized "
            "stereo pairs. Output: "
            "X_right = R_left_to_right X_left + "
            "T_left_to_right_m in OpenCV optical RDF."
        )
        self.extrinsics_bundle_contract_label.setObjectName(
            "compactContract"
        )
        self.extrinsics_bundle_contract_label.setWordWrap(True)
        self.extrinsics_bundle_contract_label.setTextInteractionFlags(
            QtCore.Qt.TextSelectableByMouse
        )
        self.extrinsics_page_layout.addWidget(
            self.extrinsics_bundle_contract_label
        )
        self.extrinsics_operation_tabs = workflow_tabs(
            kind="Extrinsics",
            capture_actions=(),
            compute_actions=(
                ("calibration.run_extrinsics", "Calculate R,t"),
                ("calibration.save_extrinsics", "Save R,t"),
            ),
            profile_actions=(
                ("calibration.load_extrinsics", "Load R,t"),
                (
                    "calibration.apply_bundle",
                    "Apply verified K,D + R,t bundle",
                ),
            ),
            capture_hint=(
                "Extrinsics consumes synchronized left/right rows "
                "from the shared photo selection. Applied Left/Right "
                "K,D must match the native raster. Camera, focus, and zoom "
                "provenance is operator-reviewed and does not gate readiness."
            ),
            include_auto_capture=False,
            include_intrinsic_quality=False,
        )
        self.extrinsics_page_layout.addWidget(
            self.extrinsics_operation_tabs
        )
        kind_tabs.addTab(extrinsics_page, "Extrinsics")
        self._inputs._update_calibration_compute_selection_state()
        return kind_tabs


    def _on_calibration_workflow_changed(self, index: int) -> None:
        if self._calibration_workflow_syncing:
            return
        self._calibration_workflow_index = max(0, int(index))
        self._calibration_workflow_syncing = True
        try:
            for tabs in (
                self.intrinsics_operation_tabs,
                self.extrinsics_operation_tabs,
            ):
                if tabs.currentIndex() != self._calibration_workflow_index:
                    tabs.setCurrentIndex(self._calibration_workflow_index)
        finally:
            self._calibration_workflow_syncing = False
        self._sync_calibration_capture_surface()


    def _on_calibration_kind_changed(self, _index: int) -> None:
        self._calibration_workflow_syncing = True
        try:
            workflow_tabs = (
                self.intrinsics_operation_tabs
                if self.operation_tabs.currentIndex() == 0
                else self.extrinsics_operation_tabs
            )
            if workflow_tabs.currentIndex() != self._calibration_workflow_index:
                workflow_tabs.setCurrentIndex(self._calibration_workflow_index)
        finally:
            self._calibration_workflow_syncing = False
        self._sync_calibration_capture_surface()


    def _sync_calibration_capture_surface(self, _index: int = -1) -> None:
        kind_index = self.operation_tabs.currentIndex()
        workflow_tabs = (
            self.intrinsics_operation_tabs
            if kind_index == 0
            else self.extrinsics_operation_tabs
        )
        capture_visible = workflow_tabs.currentIndex() == 0
        self._preview.camera_mode_tabs.setVisible(capture_visible)
        self.capture_surface.setVisible(capture_visible)
        if hasattr(self, "intrinsics_result_group"):
            self.intrinsics_result_group.setVisible(
                kind_index == 0 and workflow_tabs.currentIndex() != 0
            )
        if hasattr(self, "extrinsics_result_group"):
            self.extrinsics_result_group.setVisible(
                kind_index == 1 and workflow_tabs.currentIndex() != 0
            )
        if hasattr(self, "checkerboard_validation_outer_group"):
            self.checkerboard_validation_outer_group.setVisible(
                kind_index == 0 and workflow_tabs.currentIndex() == 2
            )
        if hasattr(self._capture, "capture_primary_button"):
            self._capture.capture_primary_button.setText(
                "Capture Live Pair"
                if kind_index == 0
                else "Capture Stereo Pair"
            )
        if capture_visible:
            self.operation_tabs.setFixedHeight(260)
        else:
            self.operation_tabs.setMinimumHeight(0)
            self.operation_tabs.setMaximumHeight(QtWidgets.QWIDGETSIZE_MAX)
            self.operation_tabs.updateGeometry()


    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: N802 - Qt override.
        super().showEvent(event)
        self._preview._start_checkerboard_preview()


    def hideEvent(self, event: QtGui.QHideEvent) -> None:  # noqa: N802 - Qt override.
        self._preview._checkerboard_preview_timer.stop()
        super().hideEvent(event)


    def setup_payload(self) -> dict[str, object]:
        payload: dict[str, object] = {}
        for key, widget in self.setup_widgets.items():
            if isinstance(widget, QtWidgets.QComboBox):
                payload[key] = widget.currentData()
            elif isinstance(widget, QtWidgets.QLineEdit):
                payload[key] = widget.text().strip()
            elif isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox)):
                payload[key] = widget.value()
        # The visible Photo set is the calculation input root. Keep the hidden
        # owner-side validation widget in the same transaction so Run cannot
        # scan a stale directory that differs from the table the operator saw.
        if "calibration_root" in payload:
            payload["validation_root"] = payload["calibration_root"]
        return payload


    def calibration_input_request(self) -> dict[str, object]:
        """Return the visible calculation authority, not hidden owner widgets."""

        root = str(
            getattr(self._inputs, "_calibration_session_explicit_root", "") or ""
        ).strip()
        if not root:
            raise RuntimeError("Select a Photo set before running calibration")
        setup = self.setup_payload()
        return {
            "root": root,
            "mode": str(setup.get("validation_mode") or "pixhawk"),
            "pair_limit": int(setup.get("calculation_pair_count") or 0),
        }


    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        camera = snapshot.get("camera.left")
        preview_text = (
            "Stop preview" if camera and camera.state in {"LIVE", "STARTING"} else "Preview"
        )
        self._capture.camera_buttons["camera.toggle"].setText(preview_text)
        for button in self._capture.simple_calibration_buttons.get("camera.toggle", ()):
            button.setText(preview_text)
        panel = getattr(self.backend, "panel", self.backend)
        calibration_controller = getattr(panel, "_calibration_workflow_controller_owner", None)
        auto_capture_active = bool(getattr(calibration_controller, "_auto_capture_active", False))
        auto_capture_button = self._commands.primary.get("calibration.auto_capture")
        if auto_capture_button is not None:
            auto_capture_button.setText("Stop auto capture" if auto_capture_active else "Start auto capture")
        for button in self._capture.simple_calibration_buttons.get("calibration.auto_capture", ()):
            button.setText("Stop auto capture" if auto_capture_active else "Start auto capture")
        recording = snapshot.get("recording")
        recording_locked = bool(recording and recording.state in {"ACTIVE", "FINALIZING"})
        self._capture.camera_buttons["camera.toggle"].setEnabled(not recording_locked)
        for button in self._capture.simple_calibration_buttons.get("camera.toggle", ()):
            button.setEnabled(not recording_locked)
        self._inputs._refresh_calibration_session_selector()
        self._inputs._sync_calibration_file_selection()
        self._quality._sync_calibration_outlier_view()
        self._quality.refresh_calibration_quality()
        # Keep the old in-memory pair picker path available for compatibility
        # with legacy embedded callers and tests; the visible Workstation UI
        # now uses explicit left/right file lists above.
        if hasattr(self._inputs, "calibration_pair_picker_list"):
            self._inputs._sync_calibration_pair_picker()
        data = _evidence(self.backend).calibration()
        self._capture.zoom_readback_label.setText(
            f"capture zoom={data.get('zoom', '--')} | "
            f"actual={data.get('zoom_readback', 'left=unknown, right=unknown')} | "
            "capture metadata is required"
        )
        for key, value in self.profile_values.items():
            value.set_value(data.get(key), source=str(data.get("source", "")))
        self._capture._sync_simple_calibration_status(data.get("capture_status"))
        intrinsic_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            camera = data.get(side, {})
            if not isinstance(camera, dict):
                camera = {}
            grid = "--"
            if camera.get("width_px") is not None and camera.get("height_px") is not None:
                grid = f"{camera['width_px']}×{camera['height_px']}"
            intrinsic_rows.append(
                (
                    side.upper(),
                    grid,
                    _display_number(camera.get("fx_px")),
                    _display_number(camera.get("fy_px")),
                    _display_number(camera.get("cx_px")),
                    _display_number(camera.get("cy_px")),
                    _display_number(camera.get("reprojection_error_px")),
                    camera.get("image_count", "--"),
                    camera.get("frame_contract", CAMERA_FRAME),
                ),
            )
        _sync_table_rows(self.intrinsic_model, tuple(intrinsic_rows))
        extrinsics = data.get("extrinsics", {})
        if not isinstance(extrinsics, dict):
            extrinsics = {}
        translation = extrinsics.get("translation_left_to_right_m")
        translation_text = "--" if not translation else "[" + ", ".join(_display_number(value, digits=6) for value in translation) + "] m"
        rotation = extrinsics.get("rotation_left_to_right")
        rotation_text = (
            "--"
            if not rotation
            else "[" + "; ".join(
                ", ".join(_display_number(value, digits=6) for value in row)
                for row in rotation
            ) + "]"
        )
        right_origin = extrinsics.get("right_camera_origin_in_left_rdf_m")
        right_origin_text = "--" if not right_origin else "[" + ", ".join(_display_number(value, digits=6) for value in right_origin) + "] m"
        self.extrinsic_value.set_value(
            f"{extrinsics.get('state', 'NO DATA')} | R_left_to_right={rotation_text} | "
            f"T_left_to_right={translation_text} | "
            f"C_right_in_left=-RᵀT={right_origin_text} | "
            f"baseline={_display_number(extrinsics.get('baseline_m'))} m",
            source=str(data.get("source", "")),
            frame=str(extrinsics.get("frame_contract", CAMERA_FRAME)),
        )
        self.calibration_contract.setText(
            f"Pixels/intrinsics: {CAMERA_FRAME} · "
            "X_right = R_left_to_right X_left + T_left_to_right_m · "
            f"output: {MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]"
        )
        left_data = data.get("left")
        left_data = left_data if isinstance(left_data, Mapping) else {}
        right_data = data.get("right")
        right_data = right_data if isinstance(right_data, Mapping) else {}
        common_intrinsics = str(data.get("profile_path") or "--")
        left_intrinsics = str(left_data.get("profile_path") or common_intrinsics)
        right_intrinsics = str(right_data.get("profile_path") or common_intrinsics)
        extrinsics_profile = str(data.get("extrinsics_profile_path") or "--")
        self.extrinsics_bundle_contract_label.setText(
            f"Input bundle · Left K,D={left_intrinsics} · "
            f"Right K,D={right_intrinsics} · R,t={extrinsics_profile} · "
            f"zoom={data.get('zoom', '--')} · "
            "X_right = R_left_to_right X_left + T_left_to_right_m "
            "[OpenCV optical RDF]"
        )


    def _new_input_picker(self, parent=None):
        sources = _CalibrationViewSources(vars(self).get("backend"))
        return CalibrationInputPicker(
            read_setup=sources.setup_snapshot, apply_setup_command=sources.apply_setup_command,
            setup_payload=self.setup_payload,
            root_widget=lambda: vars(self).get("setup_widgets", {}).get("calibration_root"),
            commands=vars(self).get("_commands"),
            selection_changed=self._on_calibration_inputs_changed,
            default_root=DEFAULT_CALIBRATION_ROOT, parent=parent,
        )

    def _on_calibration_inputs_changed(self):
        quality = vars(self).get("_quality")
        if quality is not None:
            quality.invalidate_diagnostics()

    def _calibration_component(self, name):
        # Only the historical input helpers support __new__-only embedded callers.
        # The production path always composes every owner above before use.
        key = "_" + name
        component = vars(self).get(key)
        if component is None and name == "inputs":
            component = self._new_input_picker()
            vars(self)[key] = component
        if component is None:
            raise AttributeError(f"Calibration component is not composed: {name}")
        return component

    def view_timers(self):
        return (
            self._preview._checkerboard_preview_timer,
            self._preview._ftp_preview_timer,
            self._validation._checkerboard_validation_poll_timer,
        )

    def dispose_view_resources(self):
        self._preview.stop()
        self._capture.stop()
        self._quality.stop()
        self._validation.stop()
        self._inputs.cancel_pending_thumbnails()

    # Compatibility edge for existing commands, embedded views and tests.
    # New implementation code addresses the responsible owner directly.
    operation_buttons = property(lambda self: self._commands.primary)
    operation_button_instances = property(lambda self: self._commands.instances)
    _calibration_capture_log_lines = ViewAlias("capture", "_calibration_capture_log_lines")
    _calibration_diagnostics_signature = ViewAlias("quality", "_calibration_diagnostics_signature")
    _calibration_file_picker_signature = ViewAlias("inputs", "_calibration_file_picker_signature")
    _calibration_file_selection_path_errors = ViewAlias("inputs", "_calibration_file_selection_path_errors")
    _calibration_pair_picker_items = ViewAlias("inputs", "_calibration_pair_picker_items")
    _calibration_pair_picker_root = ViewAlias("inputs", "_calibration_pair_picker_root")
    _calibration_pair_picker_session_started_unix_ns = ViewAlias("inputs", "_calibration_pair_picker_session_started_unix_ns")
    _calibration_pair_picker_signature = ViewAlias("inputs", "_calibration_pair_picker_signature")
    _calibration_pair_picker_thumbnail_generation = ViewAlias("inputs", "_calibration_pair_picker_thumbnail_generation")
    _calibration_pair_picker_thumbnail_index = ViewAlias("inputs", "_calibration_pair_picker_thumbnail_index")
    _calibration_pair_picker_thumbnail_paths = ViewAlias("inputs", "_calibration_pair_picker_thumbnail_paths")
    _calibration_pair_picker_visible_keys = ViewAlias("inputs", "_calibration_pair_picker_visible_keys")
    _calibration_quality_gate_owned = ViewAlias("quality", "_calibration_quality_gate_owned")
    _calibration_quality_original_tooltips = ViewAlias("quality", "_calibration_quality_original_tooltips")
    _calibration_quality_signature = ViewAlias("quality", "_calibration_quality_signature")
    _calibration_quality_snapshot_cache = ViewAlias("quality", "_calibration_quality_snapshot_cache")
    _calibration_quality_state = ViewAlias("quality", "_calibration_quality_state")
    _calibration_quality_warning_confirmed = ViewAlias("quality", "_calibration_quality_warning_confirmed")
    _calibration_session_catalog_signature = ViewAlias("inputs", "_calibration_session_catalog_signature")
    _calibration_session_explicit_root = ViewAlias("inputs", "_calibration_session_explicit_root")
    _camera_mode_programmatic_change = ViewAlias("preview", "_camera_mode_programmatic_change")
    _camera_preview_metadata = ViewAlias("preview", "_camera_preview_metadata")
    _capture_pair_text = ViewAlias("capture", "_capture_pair_text")
    _capture_summary_states = ViewAlias("capture", "_capture_summary_states")
    _checkerboard_overlay_active = ViewAlias("preview", "_checkerboard_overlay_active")
    _checkerboard_preview_supported = ViewAlias("preview", "_checkerboard_preview_supported")
    _checkerboard_preview_timer = ViewAlias("preview", "_checkerboard_preview_timer")
    _checkerboard_validation_poll_timer = ViewAlias("validation", "_checkerboard_validation_poll_timer")
    _ftp_auto_preview_active = ViewAlias("preview", "_ftp_auto_preview_active")
    _ftp_manual_hold = ViewAlias("preview", "_ftp_manual_hold")
    _ftp_preview_status_text = ViewAlias("preview", "_ftp_preview_status_text")
    _ftp_preview_timer = ViewAlias("preview", "_ftp_preview_timer")
    _last_calibration_session_apply_performed = ViewAlias("inputs", "_last_calibration_session_apply_performed")
    _simple_calibration_detection_text = ViewAlias("preview", "_simple_calibration_detection_text")
    auto_capture_settings_content = ViewAlias("capture", "auto_capture_settings_content")
    auto_capture_settings_group = ViewAlias("capture", "auto_capture_settings_group")
    calibration_capture_response_edits = ViewAlias("capture", "calibration_capture_response_edits")
    calibration_capture_response_status = ViewAlias("capture", "calibration_capture_response_status")
    calibration_center_coverage_table = ViewAlias("quality", "calibration_center_coverage_table")
    calibration_clear_files_button = ViewAlias("inputs", "calibration_clear_files_button")
    calibration_file_selection_summary = ViewAlias("inputs", "calibration_file_selection_summary")
    calibration_left_files_button = ViewAlias("inputs", "calibration_left_files_button")
    calibration_manual_selection_checkbox = ViewAlias("inputs", "calibration_manual_selection_checkbox")
    calibration_outlier_plot = ViewAlias("quality", "calibration_outlier_plot")
    calibration_outlier_summary = ViewAlias("quality", "calibration_outlier_summary")
    calibration_outlier_table = ViewAlias("quality", "calibration_outlier_table")
    calibration_pair_picker_list = ViewAlias("inputs", "calibration_pair_picker_list")
    calibration_pair_picker_summary = ViewAlias("inputs", "calibration_pair_picker_summary")
    calibration_quality_gate_label = ViewAlias("quality", "calibration_quality_gate_label")
    calibration_quality_group = ViewAlias("quality", "calibration_quality_group")
    calibration_quality_reasons_label = ViewAlias("quality", "calibration_quality_reasons_label")
    calibration_quality_state_label = ViewAlias("quality", "calibration_quality_state_label")
    calibration_quality_values = ViewAlias("quality", "calibration_quality_values")
    calibration_quality_warning_checkbox = ViewAlias("quality", "calibration_quality_warning_checkbox")
    calibration_right_files_button = ViewAlias("inputs", "calibration_right_files_button")
    calibration_selected_files_table = ViewAlias("inputs", "calibration_selected_files_table")
    calibration_session_combo = ViewAlias("inputs", "calibration_session_combo")
    calibration_session_path_label = ViewAlias("inputs", "calibration_session_path_label")
    calibration_session_refresh_button = ViewAlias("inputs", "calibration_session_refresh_button")
    calibration_session_result_label = ViewAlias("inputs", "calibration_session_result_label")
    calibration_show_all_pairs_checkbox = ViewAlias("inputs", "calibration_show_all_pairs_checkbox")
    camera_buttons = ViewAlias("capture", "camera_buttons")
    camera_mode_tabs = ViewAlias("preview", "camera_mode_tabs")
    capture_primary_button = ViewAlias("capture", "capture_primary_button")
    capture_response_content = ViewAlias("capture", "capture_response_content")
    capture_response_outer_group = ViewAlias("capture", "capture_response_outer_group")
    capture_summary_label = ViewAlias("capture", "capture_summary_label")
    checkerboard_validation_apply_button = ViewAlias("validation", "checkerboard_validation_apply_button")
    checkerboard_validation_json_edit = ViewAlias("validation", "checkerboard_validation_json_edit")
    checkerboard_validation_mode_combo = ViewAlias("validation", "checkerboard_validation_mode_combo")
    checkerboard_validation_random_button = ViewAlias("validation", "checkerboard_validation_random_button")
    checkerboard_validation_random_pair_count_spin = ViewAlias("validation", "checkerboard_validation_random_pair_count_spin")
    checkerboard_validation_random_runs_spin = ViewAlias("validation", "checkerboard_validation_random_runs_spin")
    checkerboard_validation_results_text = ViewAlias("validation", "checkerboard_validation_results_text")
    checkerboard_validation_root_edit = ViewAlias("validation", "checkerboard_validation_root_edit")
    checkerboard_validation_save_button = ViewAlias("validation", "checkerboard_validation_save_button")
    checkerboard_validation_scan_label = ViewAlias("validation", "checkerboard_validation_scan_label")
    checkerboard_validation_seed_spin = ViewAlias("validation", "checkerboard_validation_seed_spin")
    checkerboard_validation_status_label = ViewAlias("validation", "checkerboard_validation_status_label")
    ftp_preview_splitter = ViewAlias("preview", "ftp_preview_splitter")
    ftp_previews = ViewAlias("preview", "ftp_previews")
    preview_splitter = ViewAlias("preview", "preview_splitter")
    previews = ViewAlias("preview", "previews")
    simple_calibration_board_indicators = ViewAlias("capture", "simple_calibration_board_indicators")
    simple_calibration_buttons = ViewAlias("capture", "simple_calibration_buttons")
    simple_calibration_capture_status = ViewAlias("capture", "simple_calibration_capture_status")
    simple_calibration_preview_splitter = ViewAlias("preview", "simple_calibration_preview_splitter")
    simple_calibration_previews = ViewAlias("preview", "simple_calibration_previews")
    simple_calibration_profile_status = ViewAlias("capture", "simple_calibration_profile_status")
    simple_calibration_quality_status = ViewAlias("capture", "simple_calibration_quality_status")
    simple_calibration_run_status = ViewAlias("capture", "simple_calibration_run_status")
    simple_calibration_set_status = ViewAlias("capture", "simple_calibration_set_status")
    simple_calibration_zoom_status = ViewAlias("capture", "simple_calibration_zoom_status")
    zoom_group = ViewAlias("capture", "zoom_group")
    zoom_in_button = ViewAlias("capture", "zoom_in_button")
    zoom_level_spin = ViewAlias("capture", "zoom_level_spin")
    zoom_out_button = ViewAlias("capture", "zoom_out_button")
    zoom_query_button = ViewAlias("capture", "zoom_query_button")
    zoom_readback_label = ViewAlias("capture", "zoom_readback_label")
    zoom_set_button = ViewAlias("capture", "zoom_set_button")
    _append_calibration_capture_event = ViewAlias("capture", "_append_calibration_capture_event")
    _apply_calibration_quality_gate = ViewAlias("quality", "_apply_calibration_quality_gate")
    _apply_checkerboard_validation_result = ViewAlias("validation", "_apply_checkerboard_validation_result")
    _apply_checkerboard_validation_snapshot = ViewAlias("validation", "_apply_checkerboard_validation_snapshot")
    _build_calibration_capture_response_group = ViewAlias("capture", "_build_calibration_capture_response_group")
    _build_calibration_file_selection_group = ViewAlias("inputs", "_build_calibration_file_selection_group")
    _build_calibration_outlier_group = ViewAlias("quality", "_build_calibration_outlier_group")
    _build_calibration_pair_picker_group = ViewAlias("inputs", "_build_calibration_pair_picker_group")
    _build_calibration_quality_group = ViewAlias("quality", "_build_calibration_quality_group")
    _build_calibration_session_group = ViewAlias("inputs", "_build_calibration_session_group")
    _build_checkerboard_validation_group = ViewAlias("validation", "_build_checkerboard_validation_group")
    _build_shared_capture_surface = ViewAlias("capture", "_build_shared_capture_surface")
    _build_simple_calibration_tabs = ViewAlias("capture", "_build_simple_calibration_tabs")
    _calibration_capture_count = staticmethod(CalibrationCaptureSurface._calibration_capture_count)
    _calibration_capture_metadata_path = staticmethod(CalibrationInputPicker._calibration_capture_metadata_path)
    _calibration_coverage_grid = staticmethod(CalibrationQualityPanel._calibration_coverage_grid)
    _calibration_diagnostic_rows = ViewAlias("quality", "_calibration_diagnostic_rows")
    _calibration_exclusion_sets = ViewAlias("quality", "_calibration_exclusion_sets")
    _calibration_file_capture_time = staticmethod(CalibrationInputPicker._calibration_file_capture_time)
    _calibration_file_info = staticmethod(CalibrationInputPicker._calibration_file_info)
    _calibration_file_key = staticmethod(CalibrationInputPicker._calibration_file_key)
    _calibration_file_root = staticmethod(CalibrationInputPicker._calibration_file_root)
    _calibration_file_selection_for_root = ViewAlias("inputs", "_calibration_file_selection_for_root")
    _calibration_file_signature = staticmethod(CalibrationInputPicker._calibration_file_signature)
    _calibration_file_tooltip = staticmethod(CalibrationInputPicker._calibration_file_tooltip)
    _calibration_grid_cell = staticmethod(CalibrationQualityPanel._calibration_grid_cell)
    _calibration_grid_cell_text = staticmethod(CalibrationQualityPanel._calibration_grid_cell_text)
    _calibration_pair_selection_path = ViewAlias("inputs", "_calibration_pair_selection_path")
    _calibration_quality_display = staticmethod(CalibrationQualityPanel._calibration_quality_display)
    _calibration_quality_field = staticmethod(CalibrationQualityPanel._calibration_quality_field)
    _calibration_quality_mapping = staticmethod(CalibrationQualityPanel._calibration_quality_mapping)
    _calibration_quality_reasons = staticmethod(CalibrationQualityPanel._calibration_quality_reasons)
    _calibration_quality_state_text = staticmethod(CalibrationQualityPanel._calibration_quality_state_text)
    _checkerboard_validation_payload = ViewAlias("validation", "_checkerboard_validation_payload")
    _choose_calibration_files = ViewAlias("inputs", "_choose_calibration_files")
    _choose_checkerboard_validation_json = ViewAlias("validation", "_choose_checkerboard_validation_json")
    _choose_checkerboard_validation_root = ViewAlias("validation", "_choose_checkerboard_validation_root")
    _clear_calibration_capture_response_logs = ViewAlias("capture", "_clear_calibration_capture_response_logs")
    _clear_calibration_file_selection = ViewAlias("inputs", "_clear_calibration_file_selection")
    _clear_camera_previews = ViewAlias("preview", "_clear_camera_previews")
    _connect_calibration_capture_event_signal = ViewAlias("capture", "_connect_calibration_capture_event_signal")
    _connect_calibration_diagnostics_signal = ViewAlias("quality", "_connect_calibration_diagnostics_signal")
    _connect_calibration_preview_signal = ViewAlias("preview", "_connect_calibration_preview_signal")
    _connect_camera_panes = ViewAlias("preview", "_connect_camera_panes")
    _format_calibration_file_capture_time = staticmethod(CalibrationInputPicker._format_calibration_file_capture_time)
    _handle_calibration_outlier_action = ViewAlias("quality", "_handle_calibration_outlier_action")
    _handle_calibration_pair_picker_filter_changed = ViewAlias("inputs", "_handle_calibration_pair_picker_filter_changed")
    _load_calibration_pair_selection = ViewAlias("inputs", "_load_calibration_pair_selection")
    _load_checkerboard_validation_json = ViewAlias("validation", "_load_checkerboard_validation_json")
    _load_next_calibration_pair_thumbnail = ViewAlias("inputs", "_load_next_calibration_pair_thumbnail")
    _normalize_legacy_calibration_selection = staticmethod(CalibrationInputPicker._normalize_legacy_calibration_selection)
    _on_calibration_diagnostics_changed = ViewAlias("quality", "_on_calibration_diagnostics_changed")
    _on_calibration_pair_selection_changed = ViewAlias("inputs", "_on_calibration_pair_selection_changed")
    _on_calibration_preview_changed = ViewAlias("preview", "_on_calibration_preview_changed")
    _on_calibration_session_selected = ViewAlias("inputs", "_on_calibration_session_selected")
    _on_camera_frame = ViewAlias("preview", "_on_camera_frame")
    _on_camera_mode_changed = ViewAlias("preview", "_on_camera_mode_changed")
    _on_camera_mode_tab_clicked = ViewAlias("preview", "_on_camera_mode_tab_clicked")
    _poll_checkerboard_validation = ViewAlias("validation", "_poll_checkerboard_validation")
    _refresh_calibration_session_selector = ViewAlias("inputs", "_refresh_calibration_session_selector")
    _refresh_capture_summary = ViewAlias("capture", "_refresh_capture_summary")
    _register_capture_button = ViewAlias("capture", "_register_capture_button")
    _request_checkerboard_preview = ViewAlias("preview", "_request_checkerboard_preview")
    _resolve_calibration_selection_paths = ViewAlias("inputs", "_resolve_calibration_selection_paths")
    _return_to_live_after_ftp_preview = ViewAlias("preview", "_return_to_live_after_ftp_preview")
    _run_checkerboard_validation = ViewAlias("validation", "_run_checkerboard_validation")
    _save_calibration_file_selection = ViewAlias("inputs", "_save_calibration_file_selection")
    _save_checkerboard_validation_result = ViewAlias("validation", "_save_checkerboard_validation_result")
    _scan_checkerboard_validation = ViewAlias("validation", "_scan_checkerboard_validation")
    _send_calibration_zoom = ViewAlias("capture", "_send_calibration_zoom")
    _set_ftp_preview_status = ViewAlias("preview", "_set_ftp_preview_status")
    _set_simple_calibration_preview_meta = ViewAlias("preview", "_set_simple_calibration_preview_meta")
    _show_ftp_pair_preview = ViewAlias("preview", "_show_ftp_pair_preview")
    _show_preview_frame = ViewAlias("preview", "_show_preview_frame")
    _start_checkerboard_preview = ViewAlias("preview", "_start_checkerboard_preview")
    _sync_calibration_file_selection = ViewAlias("inputs", "_sync_calibration_file_selection")
    _sync_calibration_outlier_view = ViewAlias("quality", "_sync_calibration_outlier_view")
    _sync_calibration_pair_picker = ViewAlias("inputs", "_sync_calibration_pair_picker")
    _sync_simple_calibration_status = ViewAlias("capture", "_sync_simple_calibration_status")
    _update_calibration_compute_selection_state = ViewAlias("inputs", "_update_calibration_compute_selection_state")
    apply_calibration_capture_snapshot = ViewAlias("capture", "apply_calibration_capture_snapshot")
    apply_calibration_quality_snapshot = ViewAlias("quality", "apply_calibration_quality_snapshot")
    apply_selected_calibration_session = ViewAlias("inputs", "apply_selected_calibration_session")
    calibration_capture_snapshot = ViewAlias("capture", "calibration_capture_snapshot")
    calibration_quality_action_allowed = ViewAlias("quality", "calibration_quality_action_allowed")
    calibration_quality_snapshot = ViewAlias("quality", "calibration_quality_snapshot")
    confirm_calibration_quality_warning = ViewAlias("quality", "confirm_calibration_quality_warning")
    refresh_calibration_quality = ViewAlias("quality", "refresh_calibration_quality")
