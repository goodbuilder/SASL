"""Camera discovery, live input status and preview workspace."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Callable
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.workstation_core.network import candidate_network_role_labels
from Workstation.workstation_core.runtime_camera_status import (
    decoder_presentation,
    format_camera_input_metrics,
)
from Workstation.ui.components import EngineeringValue, WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus, StatusModel
from .camera_preview import CameraPreview, _balance_preview_pair, _preview_pair_height
from .network_helpers import _apply_network_role_discovery_status, _network_role_column_text
from .shared import (
    _append_table_row,
    _command_button,
    _display_class,
    _display_number,
    _evidence,
    _live_inspection,
    _sync_table_rows,
    _table_model,
)


class CameraWorkspace(QtWidgets.QWidget):
    inspection_requested = QtCore.pyqtSignal(object)
    CAMERA_ASPECT_RATIO = 16.0 / 9.0
    # A bottom Diagnostics dock can reduce the central workspace to a
    # landscape rectangle.  Once that dock is hidden, the same workspace can
    # become tall enough that side-by-side 16:9 panes waste most of the
    # available height.  In that case the camera pair moves to a vertical
    # stack and the details pane moves alongside it.
    TALL_LAYOUT_RATIO = 1.45

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.setMinimumWidth(0)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Cameras"))

        # Keep the live pair usable even when the camera workspace also shows
        # discovery, detection, and mapping controls.  Those controls have
        # sizeable layout hints and previously squeezed this splitter down to
        # a narrow strip on typical workstation displays.
        self.workspace_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.workspace_splitter.setChildrenCollapsible(False)
        self.workspace_splitter.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        self.preview_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.preview_splitter.setChildrenCollapsible(False)
        self.preview_splitter.setMinimumHeight(320)
        self.preview_splitter.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Expanding,
        )
        self.previews = {side: CameraPreview(side) for side in ("left", "right")}
        self.preview_splitter.addWidget(self.previews["left"])
        self.preview_splitter.addWidget(self.previews["right"])
        self.preview_splitter.setStretchFactor(0, 1)
        self.preview_splitter.setStretchFactor(1, 1)
        self.workspace_splitter.addWidget(self.preview_splitter)

        details = QtWidgets.QWidget()
        details_layout = QtWidgets.QVBoxLayout(details)
        details_layout.setContentsMargins(0, 0, 0, 0)
        details_layout.setSpacing(6)
        facts = QtWidgets.QGroupBox("Status")
        facts_layout = QtWidgets.QGridLayout(facts)
        self.camera_values = {}
        for col, (key, label) in enumerate((("camera.left", "CAM L"), ("camera.right", "CAM R"), ("fps", "Decoded FPS"), ("detection", "Detection"))):
            value = EngineeringValue(label)
            self.camera_values[key] = value
            facts_layout.addWidget(value, 0, col)
        self.decoder_values: dict[str, EngineeringValue] = {}
        for col, side in enumerate(("left", "right")):
            value = EngineeringValue(f"Decoder {side.upper()}")
            self.decoder_values[side] = value
            facts_layout.addWidget(value, 1, col)
        details_layout.addWidget(facts)

        stream_metrics_group = QtWidgets.QGroupBox("RTP / decode")
        stream_metrics_layout = QtWidgets.QVBoxLayout(stream_metrics_group)
        stream_metrics_layout.setContentsMargins(6, 6, 6, 6)
        stream_metrics_layout.setSpacing(2)
        self.stream_metric_labels: dict[str, QtWidgets.QLabel] = {}
        for side in ("left", "right"):
            label = QtWidgets.QLabel(format_camera_input_metrics(side, {}))
            label.setObjectName("statusStrip")
            label.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
            label.setWordWrap(False)
            label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
            label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            label.setFixedHeight(label.fontMetrics().lineSpacing() * 2 + 2)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            self.stream_metric_labels[side] = label
            stream_metrics_layout.addWidget(label)
        stream_metrics_group.setToolTip(
            "GStreamer worker의 최근 5초 rolling rate입니다. "
            "RTP missing은 sequence gap으로 추정한 누락 패킷 수입니다."
        )
        self.stream_metrics_group = stream_metrics_group
        details_layout.addWidget(stream_metrics_group)

        detection_group = QtWidgets.QGroupBox("Detection")
        detection_layout = QtWidgets.QVBoxLayout(detection_group)
        detection_layout.setContentsMargins(6, 6, 6, 6)
        config_layout = QtWidgets.QGridLayout()
        config_layout.setContentsMargins(0, 0, 0, 0)
        self.detection_config_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("weights_path", "Weights"),
                ("device", "Device"),
                ("input_size_px", "Input"),
                ("confidence", "Conf"),
                ("iou", "IoU"),
                ("gate_enabled", "Gates"),
            )
        ):
            value = EngineeringValue(label)
            self.detection_config_values[key] = value
            config_layout.addWidget(value, index // 3, index % 3)
        detection_layout.addLayout(config_layout)
        self.target_model = _table_model(
            self,
            ("Camera", "State", "Class", "Conf", "bbox_px [x1,y1,x2,y2]", "Detection t", "Age"),
        )
        self.target_table = QtWidgets.QTableView()
        self.target_table.setModel(self.target_model)
        self.target_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.target_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.target_table.verticalHeader().setVisible(False)
        self.target_table.setMaximumHeight(118)
        self.target_table.horizontalHeader().setStretchLastSection(True)
        detection_layout.addWidget(self.target_table)
        details_layout.addWidget(detection_group)

        operations_group = QtWidgets.QGroupBox("Connect")
        operations_layout = QtWidgets.QGridLayout(operations_group)
        operations_layout.setContentsMargins(6, 6, 6, 6)
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for index, (command_id, label) in enumerate(
            (
                ("camera.discover_host_ip", "Find network"),
                ("camera.toggle", "Preview"),
                ("camera.auto_connect", "Connect cameras"),
                ("camera.check_control", "Check control"),
                ("camera.enable_rtsp", "Enable stream"),
                ("camera.check_rtsp", "Check video"),
            )
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            operations_layout.addWidget(button, index // 3, index % 3)
        details_layout.addWidget(operations_group)

        candidates_group = QtWidgets.QGroupBox("Discovered endpoints")
        candidates_layout = QtWidgets.QVBoxLayout(candidates_group)
        self.discovery_status = EngineeringValue("Discovery")
        candidates_layout.addWidget(self.discovery_status)
        self.candidate_model = _table_model(
            self,
            ("Kind", "Address", "Port", "Status", "Interface", "Host IP", "Assigned"),
        )
        self.candidate_table = QtWidgets.QTableView()
        self.candidate_table.setModel(self.candidate_model)
        self.candidate_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.candidate_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.candidate_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.candidate_table.verticalHeader().setVisible(False)
        self.candidate_table.setMaximumHeight(120)
        self.candidate_table.horizontalHeader().setStretchLastSection(True)
        self.candidate_table.selectionModel().selectionChanged.connect(self._refresh_candidate_actions)
        self.candidate_table.clicked.connect(self._inspect_candidate_row)
        candidates_layout.addWidget(self.candidate_table)
        candidate_actions = QtWidgets.QHBoxLayout()
        self.candidate_buttons: dict[str, QtWidgets.QPushButton] = {}
        for target, label in (("nic", "Assign Camera NIC"), ("left", "Assign CAM L"), ("right", "Assign CAM R")):
            button = QtWidgets.QPushButton(label)
            button.setEnabled(False)
            button.clicked.connect(lambda _checked=False, target=target: self._apply_candidate(target))
            self.candidate_buttons[target] = button
            candidate_actions.addWidget(button)
        candidate_actions.addStretch(1)
        candidates_layout.addLayout(candidate_actions)
        self.candidate_feedback = QtWidgets.QLabel("")
        self.candidate_feedback.setObjectName("statusStrip")
        self.candidate_feedback.setWordWrap(True)
        candidates_layout.addWidget(self.candidate_feedback)
        details_layout.addWidget(candidates_group)
        self._candidate_signature: tuple[tuple[object, ...], ...] = ()
        self._recording_locked = False
        self._network_role_locked = False

        mapping_group = QtWidgets.QGroupBox("Mapping")
        mapping_layout = QtWidgets.QHBoxLayout(mapping_group)
        mapping_layout.setContentsMargins(6, 6, 6, 6)
        for command_id, label, tooltip in (
            ("camera.swap", "Swap L/R", "Swap the current Left/Right camera mapping."),
            ("camera.save_settings", "Save", "Save the current camera settings."),
            (
                "camera.reactivate_left",
                "Reactivate CAM L",
                "Safely send SETH TRUE for the left camera, verify frames, and reconnect its preview.",
            ),
            (
                "camera.reactivate_right",
                "Reactivate CAM R",
                "Safely send SETH TRUE for the right camera, verify frames, and reconnect its preview.",
            ),
            ("advanced.open", "Advanced", "Open Workstation advanced diagnostics and tools."),
        ):
            button = _command_button(label, command_id, invoke_command)
            button.setToolTip(tooltip)
            self.operation_buttons[command_id] = button
            mapping_layout.addWidget(button)
        mapping_layout.addStretch(1)
        details_layout.addWidget(mapping_group)

        details_layout.addWidget(self._build_image_format_group(invoke_command))
        details_layout.addStretch(1)

        self.details_scroll = QtWidgets.QScrollArea()
        self.details_scroll.setWidgetResizable(True)
        self.details_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.details_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.details_scroll.setWidget(details)
        self.details_scroll.setMinimumHeight(120)
        self.workspace_splitter.addWidget(self.details_scroll)
        self.workspace_splitter.setStretchFactor(0, 3)
        self.workspace_splitter.setStretchFactor(1, 1)
        self.workspace_splitter.setSizes([600, 240])
        layout.addWidget(self.workspace_splitter, 1)
        self._connect_panes()

    # -- Image Format group ---------------------------------------------
    #
    # CPTR emits one file per enabled SIxx save-format toggle
    # (SIRJ=radiometric, SISR=super-resolution, SIHR=HR, SIVI=OZ visible).
    # Left/right were found with different toggles on, which breaks any
    # "one file per capture" assumption downstream (e.g. the calibration OZ
    # FTP sync). This group makes that state visible and lets the operator
    # force both cameras to a known state; the calibration OZ capture path
    # additionally forces it automatically before every trigger.

    IMAGE_FORMAT_TOGGLES = (
        ("SIRJ", "GIRJ", "Radiometric"),
        ("SISR", "GISR", "Super-res"),
        ("SIHR", "GIHR", "HR"),
        ("SIVI", "GIVI", "Visible (OZ)"),
    )

    def _build_image_format_group(
        self, invoke_command: Callable[[str], bool] | None
    ) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Image Format")
        layout = QtWidgets.QVBoxLayout(group)

        status_layout = QtWidgets.QGridLayout()
        status_layout.setHorizontalSpacing(10)
        self.image_format_values: dict[str, EngineeringValue] = {}
        get_commands = tuple(get_cmd for _set_cmd, get_cmd, _label in self.IMAGE_FORMAT_TOGGLES) + ("GILC",)
        for row, side in enumerate(("left", "right")):
            for col, get_cmd in enumerate(get_commands):
                value = EngineeringValue(f"{side.title()} {get_cmd}")
                self.image_format_values[f"{side}_{get_cmd}"] = value
                status_layout.addWidget(value, row, col)
        layout.addLayout(status_layout)

        toggle_layout = QtWidgets.QHBoxLayout()
        self.image_format_checkboxes: dict[str, QtWidgets.QCheckBox] = {}
        for set_cmd, _get_cmd, label in self.IMAGE_FORMAT_TOGGLES:
            checkbox = QtWidgets.QCheckBox(label)
            checkbox.setToolTip(
                f"체크: 양쪽 카메라에 {set_cmd} TRUE 전송. 해제: 양쪽에 {set_cmd} FALSE 전송."
            )
            checkbox.toggled.connect(
                lambda checked, set_cmd=set_cmd: self._on_image_format_checkbox_toggled(set_cmd, checked)
            )
            self.image_format_checkboxes[set_cmd] = checkbox
            toggle_layout.addWidget(checkbox)
        toggle_layout.addStretch(1)
        layout.addLayout(toggle_layout)

        actions_layout = QtWidgets.QHBoxLayout()
        refresh_button = _command_button(
            "새로고침", "camera.refresh_image_format_status", invoke_command
        )
        refresh_button.setToolTip("좌우 GIVI/GIRJ/GISR/GIHR/GILC 상태를 다시 조회합니다.")
        self.operation_buttons["camera.refresh_image_format_status"] = refresh_button
        actions_layout.addWidget(refresh_button)
        preset_button = _command_button(
            "OZ 전용 초기화", "camera.apply_oz_only_image_format", invoke_command
        )
        preset_button.setToolTip(
            "SIRJ FALSE, SISR FALSE, SIHR FALSE, SIVI TRUE를 양쪽 카메라에 전송합니다."
        )
        self.operation_buttons["camera.apply_oz_only_image_format"] = preset_button
        actions_layout.addWidget(preset_button)
        actions_layout.addStretch(1)
        layout.addLayout(actions_layout)
        return group

    def _on_image_format_checkbox_toggled(self, set_command: str, checked: bool) -> None:
        setter = getattr(self.backend, "camera_set_image_format", None)
        if not callable(setter):
            return
        setter(set_command, checked)

    def _sync_image_format_status(self) -> None:
        values = getattr(self, "image_format_values", None)
        if values is None:
            return
        status = _evidence(self.backend).image_format_status()
        for side in ("left", "right"):
            responses = status.get(side) if isinstance(status.get(side), dict) else {}
            for _set_cmd, get_cmd, _label in self.IMAGE_FORMAT_TOGGLES:
                widget = values.get(f"{side}_{get_cmd}")
                if widget is not None:
                    widget.set_value(responses.get(get_cmd))
            gilc_widget = values.get(f"{side}_GILC")
            if gilc_widget is not None:
                gilc_widget.set_value(responses.get("GILC"))
        left = status.get("left") if isinstance(status.get("left"), dict) else {}
        right = status.get("right") if isinstance(status.get("right"), dict) else {}
        for set_cmd, get_cmd, _label in self.IMAGE_FORMAT_TOGGLES:
            checkbox = self.image_format_checkboxes.get(set_cmd)
            if checkbox is None:
                continue
            both_true = str(left.get(get_cmd, "")).strip().upper() == "TRUE" and (
                str(right.get(get_cmd, "")).strip().upper() == "TRUE"
            )
            checkbox.blockSignals(True)
            checkbox.setChecked(both_true)
            checkbox.blockSignals(False)

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
        if self.workspace_splitter.width() <= 0 or self.workspace_splitter.height() <= 0:
            return
        available_height = self._available_camera_height()
        tall_layout = (
            float(self.workspace_splitter.width()) / float(max(1, available_height))
            < self.TALL_LAYOUT_RATIO
        )
        orientation = QtCore.Qt.Vertical if tall_layout else QtCore.Qt.Horizontal
        if self.preview_splitter.orientation() == orientation:
            return
        self.preview_splitter.setOrientation(orientation)
        self.preview_splitter.setSizes([1, 1])

    def _available_camera_height(self) -> int:
        """Use the visible scroll viewport when content is taller than the window."""

        parent = self.parentWidget()
        while parent is not None:
            if isinstance(parent, QtWidgets.QScrollArea):
                viewport_height = parent.viewport().height()
                if viewport_height > 0:
                    return min(self.workspace_splitter.height(), viewport_height)
            parent = parent.parentWidget()
        return self.workspace_splitter.height()

    def _fit_preview_layout(self) -> None:
        self._update_preview_pair_orientation()
        if self.workspace_splitter.width() <= 0 or self.workspace_splitter.height() <= 0:
            return
        tall_layout = self.preview_splitter.orientation() == QtCore.Qt.Vertical
        if tall_layout:
            # In the tall mode the preview pair is the left pane of a
            # horizontal workspace splitter.  Give the details pane a stable
            # inspection width, then size the pair from its real 16:9
            # height-for-width contract.  The outer workspace scroll area may
            # extend below the viewport when two full-width previews cannot
            # fit at once; it must not crop native image coordinates.
            self.workspace_splitter.setOrientation(QtCore.Qt.Horizontal)
            total_width = max(1, self.workspace_splitter.width() - self.workspace_splitter.handleWidth())
            details_width = min(380, max(280, int(round(total_width * 0.35))))
            preview_width = max(1, total_width - details_width)
            self.workspace_splitter.setSizes([preview_width, details_width])
            self.preview_splitter.setMinimumHeight(0)
            self.preview_splitter.setMaximumHeight(16777215)
            required_height = _preview_pair_height(self.preview_splitter, self.previews)
            self.preview_splitter.setMinimumHeight(max(320, required_height))
            _balance_preview_pair(self.preview_splitter)
            return

        # Landscape mode keeps the familiar side-by-side pair above the
        # scrollable detail surface.  The pair gets its natural aspect-ratio
        # height; extra vertical room is intentionally left to the details
        # surface instead of stretching/cropping the live image.
        self.workspace_splitter.setOrientation(QtCore.Qt.Vertical)
        self.preview_splitter.setMinimumHeight(0)
        self.preview_splitter.setMaximumHeight(16777215)
        required_height = _preview_pair_height(self.preview_splitter, self.previews)
        if required_height <= 0:
            return
        target_height = max(320, required_height)
        self.preview_splitter.setMinimumHeight(target_height)
        total_height = self.workspace_splitter.height()
        if total_height <= 0:
            return
        self.workspace_splitter.setStretchFactor(0, 0)
        self.workspace_splitter.setStretchFactor(1, 1)
        self.workspace_splitter.setSizes(
            [target_height, max(1, total_height - target_height)]
        )

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._update_preview_pair_orientation()
        QtCore.QTimer.singleShot(0, self._fit_preview_layout)

    def showEvent(self, event: QtGui.QShowEvent) -> None:  # noqa: N802
        super().showEvent(event)
        # Navigation swaps the page without necessarily resizing the widget;
        # size the preview once it becomes visible so the first paint cannot
        # inherit an unconstrained splitter height from the scroll area.
        QtCore.QTimer.singleShot(0, self._fit_preview_layout)

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
                signal.connect(lambda _label, image, metadata, side=side: self.previews[side].set_frame(image, metadata))
            stats_signal = getattr(pane, "stats_changed", None)
            if stats_signal is not None:
                stats_signal.connect(lambda _label, stats, side=side: self.previews[side].set_metadata(stats))

    def _clear_camera_previews(self) -> None:
        for preview in self.previews.values():
            preview.clear()

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        self._sync_image_format_status()
        for side in ("left", "right"):
            status = snapshot.get(f"camera.{side}")
            self.camera_values[f"camera.{side}"].set_value(
                status.state if status else "UNKNOWN",
                source=status.source if status else "status adapter",
                age_s=status.age_s if status else None,
                valid=(
                    True
                    if status is not None and status.state == "LIVE"
                    else False
                    if status is not None and status.state == "ERROR"
                    else None
                ),
            )
        fps = snapshot.get("fps")
        self.camera_values["fps"].set_value(fps.detail if fps else "--", source=fps.source if fps else "")
        detection = snapshot.get("detection")
        self.camera_values["detection"].set_value(
            detection.state if detection else "UNKNOWN",
            source=detection.source if detection else "",
        )
        recording = snapshot.get("recording")
        recording_locked = recording is not None and recording.state in {"ACTIVE", "FINALIZING"}
        self._recording_locked = recording_locked
        panel = getattr(self.backend, "panel", self.backend)
        stream_stats_by_side = getattr(panel, "camera_stream_stats", {}) or {}
        if not isinstance(stream_stats_by_side, Mapping):
            stream_stats_by_side = {}
        for side, label in self.stream_metric_labels.items():
            side_stats = stream_stats_by_side.get(side, {})
            side_stats = dict(side_stats) if isinstance(side_stats, Mapping) else {}
            metrics_text = format_camera_input_metrics(side, side_stats)
            if label.text() != metrics_text:
                label.setText(metrics_text)
            window_s = side_stats.get("rtp_metrics_window_s", 5.0)
            try:
                window_text = f"{max(1.0, float(window_s)):.0f}s"
            except (TypeError, ValueError):
                window_text = "5s"
            label.setToolTip(
                f"최근 {window_text} rolling rate\n"
                "RTP missing = sequence gap으로 추정한 누락 패킷\n"
                f"누적 RTP {side_stats.get('rtp_packets_received_total', '--')} pkt, "
                f"누적 missing {side_stats.get('rtp_missing_packets_total', '--')} pkt"
            )
        camera_recovery_allowed = not recording_locked or bool(
            getattr(panel, "_automatic_session_recording", False)
        )
        active_decoders = getattr(panel, "active_stream_decoders", {})
        if not isinstance(active_decoders, Mapping):
            active_decoders = {}
        decoder_combo = getattr(panel, "decoder_combo", None)
        requested_decoder = (
            decoder_combo.currentData()
            if decoder_combo is not None and callable(getattr(decoder_combo, "currentData", None))
            else getattr(getattr(panel, "settings", None), "rtsp_decoder", "auto")
        )
        fallback_reasons = getattr(panel, "decoder_fallback_reasons", {}) or {}
        runtime_overrides = getattr(panel, "_runtime_decoder_overrides", {}) or {}
        for side, value in self.decoder_values.items():
            decoder = decoder_presentation(
                requested_mode=requested_decoder,
                active_decoder=active_decoders.get(side),
                runtime_override=(
                    runtime_overrides.get(side)
                    if isinstance(runtime_overrides, Mapping) and side in runtime_overrides
                    else getattr(panel, "_runtime_decoder_override", None)
                ),
                fallback_reason=(fallback_reasons.get(side) if isinstance(fallback_reasons, Mapping) else ""),
            )
            value.set_value(
                decoder.value_text,
                source=decoder.detail,
            )
        rtsp_operation_active = any(
            getattr(panel, attribute, None) is not None
            for attribute in ("_rtsp_reactivation_side", "rtsp_enable_thread", "rtsp_check_thread")
        )
        preview_button = self.operation_buttons.get("camera.toggle")
        if preview_button is not None:
            camera_running = any(
                status is not None and status.state in {"LIVE", "STARTING"}
                for status in (snapshot.get("camera.left"), snapshot.get("camera.right"))
            )
            preview_button.setText("Stop preview" if camera_running else "Preview")
            preview_button.setEnabled(not recording_locked)
        for command_id in (
            "camera.swap",
            "camera.save_settings",
        ):
            button = self.operation_buttons.get(command_id)
            if button is not None:
                button.setEnabled(not recording_locked)
        for command_id in ("camera.reactivate_left", "camera.reactivate_right"):
            button = self.operation_buttons.get(command_id)
            if button is not None:
                button.setEnabled(camera_recovery_allowed and not rtsp_operation_active)
        connection = evidence.camera_connection_candidates()
        self._network_role_locked = bool(connection.get("network_role_active", False))
        role_state, role_detail = _apply_network_role_discovery_status(
            self.discovery_status, connection
        )
        if role_state != "IDLE":
            self.candidate_feedback.setText(f"Network role: {role_state} · {role_detail}")
        candidates = connection.get("candidates")
        candidates = tuple(row for row in candidates if isinstance(row, dict)) if isinstance(candidates, (tuple, list)) else ()
        network_roles = connection.get("network_roles")
        network_roles = network_roles if isinstance(network_roles, Mapping) else {}

        def assigned_roles(row: Mapping[str, object]) -> str:
            if not str(row.get("interface") or ""):
                return "--"
            return _network_role_column_text(candidate_network_role_labels(row, network_roles))
        candidate_signature = tuple(
            (
                *tuple(
                    row.get(key, "")
                    for key in ("kind", "address", "port", "status", "interface", "host_ip")
                ),
                assigned_roles(row),
            )
            for row in candidates
        )
        if candidate_signature != self._candidate_signature:
            selected = self._selected_candidate_identity()
            self.candidate_model.removeRows(0, self.candidate_model.rowCount())
            for row in candidates:
                _append_table_row(
                    self.candidate_model,
                    (
                        *tuple(
                            row.get(key, "")
                            for key in ("kind", "address", "port", "status", "interface", "host_ip")
                        ),
                        assigned_roles(row),
                    ),
                )
            self._candidate_signature = candidate_signature
            if selected:
                for row_index, row in enumerate(candidates):
                    if (str(row.get("kind", "")), str(row.get("address", ""))) == selected:
                        self.candidate_table.selectRow(row_index)
                        break
        self.operation_buttons["camera.discover_host_ip"].setEnabled(
            not recording_locked
            and not bool(connection.get("discovery_active", False))
            and not bool(connection.get("network_role_active", False))
        )
        self.operation_buttons["camera.auto_connect"].setEnabled(
            not recording_locked and not bool(connection.get("discovery_active", False))
        )
        self._refresh_candidate_actions()
        config = evidence.detection_config()
        for key, value in self.detection_config_values.items():
            raw = config.get(key)
            if key == "input_size_px" and raw not in (None, ""):
                raw = f"{raw} px"
            elif key in {"confidence", "iou"} and raw not in (None, ""):
                raw = f"{float(raw):.2f}"
            elif key == "gate_enabled":
                raw = ", ".join(str(item) for item in (raw or ())) or "OFF"
            value.set_value(raw, source=str(config.get("source", "")))
        target_rows: list[tuple[object, ...]] = []
        for side in ("left", "right"):
            target = evidence.detection(side)
            bbox = "--"
            if target.get("x1_px") is not None:
                bbox = "[" + ", ".join(
                    _display_number(target.get(name), digits=1)
                    for name in ("x1_px", "y1_px", "x2_px", "y2_px")
                ) + "]"
            class_text = _display_class(target)
            confidence = _display_number(target.get("confidence"), digits=3)
            timestamp = _display_number(target.get("timestamp_s"), digits=3)
            age = _display_number(target.get("age_s"), digits=2, suffix="s")
            target_rows.append(
                (
                    side.upper(),
                    target.get("runtime_state", "NO DATA"),
                    class_text,
                    confidence,
                    bbox,
                    timestamp,
                    age,
                ),
            )
        _sync_table_rows(self.target_model, tuple(target_rows))

    def _selected_candidate_row(self) -> int:
        selected = self.candidate_table.selectionModel().selectedRows()
        return selected[0].row() if selected else -1

    def _selected_candidate_identity(self) -> tuple[str, str] | None:
        row = self._selected_candidate_row()
        kind = self.candidate_model.item(row, 0) if row >= 0 else None
        address = self.candidate_model.item(row, 1) if row >= 0 else None
        if kind is None or address is None:
            return None
        return kind.text(), address.text()

    def _refresh_candidate_actions(self, *_args: object) -> None:
        row = self._selected_candidate_row()
        item = self.candidate_model.item(row, 0) if row >= 0 else None
        kind = item.text() if item is not None else ""
        self.candidate_buttons["nic"].setEnabled(
            not self._recording_locked and not self._network_role_locked and kind == "NIC"
        )
        side_enabled = kind in {"RTSP", "WIRIS"}
        self.candidate_buttons["left"].setEnabled(
            not self._recording_locked and not self._network_role_locked and side_enabled
        )
        self.candidate_buttons["right"].setEnabled(
            not self._recording_locked and not self._network_role_locked and side_enabled
        )

    def _apply_candidate(self, target: str) -> None:
        row = self._selected_candidate_row()
        method = getattr(self.backend, "camera_apply_candidate", None)
        if row < 0 or not callable(method):
            self.candidate_feedback.setText("ERROR: select a compatible endpoint")
            return
        try:
            applied = method(row, target)
        except Exception as exc:
            self.candidate_feedback.setText(f"ERROR: {exc}")
            return
        if applied is False:
            self.candidate_feedback.setText("ERROR: endpoint was not applied")
            return
        self.candidate_feedback.setText(
            f"Requested row {row + 1} for {target.upper()} · follow network role status"
        )
        self.refresh(self.status_model.snapshot())

    def _inspect_candidate_row(self, index: QtCore.QModelIndex) -> None:
        row = index.row()
        self.inspection_requested.emit(
            _live_inspection(
                self._candidate_inspection(row),
                lambda row=row: self._candidate_inspection(row),
            )
        )

    def _candidate_inspection(self, row: int) -> dict[str, object]:
        candidates = _evidence(self.backend).camera_connection_candidates().get("candidates")
        candidates = candidates if isinstance(candidates, (tuple, list)) else ()
        candidate = dict(candidates[row]) if 0 <= row < len(candidates) and isinstance(candidates[row], dict) else {}
        return {
            "selection": f"Camera endpoint · {candidate.get('kind', '--')}",
            "value": (
                f"address={candidate.get('address', '--')} · port={candidate.get('port', '--')} · "
                f"host interface={candidate.get('interface', '--')} {candidate.get('host_ip', '')}"
            ),
            "freshness": "Latest completed camera network discovery",
            "contract": "Network/RTSP/control endpoint metadata; no image pixels or camera-frame coordinates",
            "verdict": candidate.get("status") or "UNVERIFIED",
            "source": "CameraNetworkDiscoveryWorker",
        }
