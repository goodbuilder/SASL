"""Data and recorded evidence review, including method worker recovery."""

from __future__ import annotations

import json
import os
import time
from collections.abc import Mapping
from pathlib import Path
from typing import Callable
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.ui.method_worker_pane import MethodWorkerRecoveryPane
from Workstation.workstation_core.theme import ERROR_TEXT, PANEL_BORDER, SUCCESS_TEXT
from Workstation.ui.components import EngineeringValue, StatusIndicator, WorkspaceHeader
from Workstation.application.protocol import evaluate_recording_protocol
from Workstation.application.status_model import RuntimeStatus, StatusModel
from .session import SessionWorkspace
from .target_reference import TargetReferenceView
from .shared import _append_table_row, _compact_text, _evidence, _sync_table_rows, _table_model


SESSION_CATALOG_REFRESH_INTERVAL_S = 5.0


class RecordingWorkspace(QtWidgets.QWidget):
    """Sessionless data status/capture tools with legacy UI fallback."""

    hr_command_requested = QtCore.pyqtSignal(str, object)
    ulog_command_requested = QtCore.pyqtSignal(str, object)
    inspection_requested = QtCore.pyqtSignal(object)

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        open_advanced: Callable[[], None],
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
        *,
        open_measurement: Callable[[], None] | None = None,
        begin_session_action: Callable[[], object] | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.invoke_command = invoke_command
        panel = getattr(self.backend, "panel", self.backend)
        self._sessionless = not bool(
            getattr(panel, "_measurement_session_hierarchy_enabled", True)
        )
        self.session_records: dict[str, dict[str, object]] = {}
        self._session_catalog_signature: tuple[object, ...] = ()
        self._last_session_scan_monotonic_s = float("-inf")
        self._last_session_scan_root = ""
        self._evidence_query_runtime = None
        self._catalog_query_request = None
        self._artifact_query_request = None
        self._query_disposed = False
        self._catalog_query_failed = False
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        self.header = WorkspaceHeader(
            "Data" if self._sessionless else "Sessions",
            (
                "Type-owned append files, storage health, and capture tools"
                if self._sessionless
                else "Current session, evidence completeness, and saved review"
            ),
        )
        layout.addWidget(self.header)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setObjectName("sessionsTabs")
        self._tab_ids = ("current", "evidence", "review")
        tab_layouts: dict[str, QtWidgets.QVBoxLayout] = {}
        for tab_id, title in zip(
            self._tab_ids,
            ("Current", "Evidence", "Review"),
        ):
            page = QtWidgets.QWidget()
            page.setObjectName(f"sessions{title}Tab")
            page_layout = QtWidgets.QVBoxLayout(page)
            page_layout.setContentsMargins(6, 6, 6, 6)
            page_layout.setSpacing(6)
            tab_layouts[tab_id] = page_layout
            self.tabs.addTab(page, title)
        layout.addWidget(self.tabs, 1)
        self._tab_layouts = tab_layouts

        self.current_session = SessionWorkspace(
            backend,
            status_model,
            open_measurement,
            lambda: self.select_tab("review"),
            begin_session_action=begin_session_action,
            embedded=True,
        )
        self.current_session.inspection_requested.connect(self.inspection_requested.emit)
        tab_layouts["current"].addWidget(self.current_session)
        if self._sessionless:
            self.current_session.hide()

        measurements_group = QtWidgets.QGroupBox("Measurements")
        measurements_layout = QtWidgets.QVBoxLayout(measurements_group)
        measurements_layout.setContentsMargins(6, 6, 6, 6)
        self.measurements_model = _table_model(
            self,
            ("#", "Measurement", "State", "Evidence", "Revision"),
        )
        self.measurements_table = QtWidgets.QTableView()
        self.measurements_table.setObjectName("sessionsMeasurementTable")
        self.measurements_table.setModel(self.measurements_model)
        self.measurements_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.measurements_table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectRows
        )
        self.measurements_table.verticalHeader().setVisible(False)
        self.measurements_table.horizontalHeader().setStretchLastSection(True)
        self.measurements_table.setMaximumHeight(180)
        measurements_layout.addWidget(self.measurements_table)
        self.measurements_group = measurements_group
        tab_layouts["current"].addWidget(measurements_group)
        if self._sessionless:
            measurements_group.hide()

        package_group = QtWidgets.QGroupBox("Evidence package")
        package_group.setObjectName("measurementPackageDetails")
        package_layout = QtWidgets.QVBoxLayout(package_group)
        package_layout.setContentsMargins(6, 6, 6, 6)
        package_layout.setSpacing(5)
        package_summary_layout = QtWidgets.QGridLayout()
        package_summary_layout.setContentsMargins(0, 0, 0, 0)
        self.package_state_label = QtWidgets.QLabel("State: IDLE")
        self.package_state_label.setObjectName("measurementPackageDetailsState")
        self.package_completeness_label = QtWidgets.QLabel("Evidence: PENDING")
        self.package_completeness_label.setObjectName("measurementPackageDetailsCompleteness")
        self.package_verification_label = QtWidgets.QLabel("Verification: NOT RUN")
        self.package_verification_label.setObjectName("measurementPackageDetailsVerification")
        self.package_revision_label = QtWidgets.QLabel("Revision: --")
        self.package_revision_label.setObjectName("measurementPackageDetailsRevision")
        for index, label in enumerate(
            (
                self.package_state_label,
                self.package_completeness_label,
                self.package_verification_label,
                self.package_revision_label,
            )
        ):
            label.setWordWrap(True)
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            package_summary_layout.addWidget(label, index // 2, index % 2)
        package_layout.addLayout(package_summary_layout)

        self.package_source_model = _table_model(
            self,
            ("Source", "Expected", "State / active range", "Detail"),
        )
        self.package_source_table = QtWidgets.QTableView()
        self.package_source_table.setObjectName("measurementPackageSourceExpectations")
        self.package_source_table.setModel(self.package_source_model)
        self.package_source_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.package_source_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.package_source_table.verticalHeader().setVisible(False)
        self.package_source_table.horizontalHeader().setStretchLastSection(True)
        self.package_source_table.setMaximumHeight(125)
        package_layout.addWidget(self.package_source_table)

        self.package_artifact_model = _table_model(
            self,
            (
                "Artifact",
                "Role",
                "Required",
                "State",
                "Bytes",
                "SHA-256",
                "Revision",
                "Missing / failed reason",
            ),
        )
        self.package_artifact_table = QtWidgets.QTableView()
        self.package_artifact_table.setObjectName("measurementPackageRequiredArtifacts")
        self.package_artifact_table.setModel(self.package_artifact_model)
        self.package_artifact_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.package_artifact_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.package_artifact_table.verticalHeader().setVisible(False)
        self.package_artifact_table.horizontalHeader().setStretchLastSection(True)
        self.package_artifact_table.setMaximumHeight(170)
        package_layout.addWidget(self.package_artifact_table)

        package_actions = QtWidgets.QHBoxLayout()
        self.verify_package_button = QtWidgets.QPushButton("Verify package")
        self.verify_package_button.setObjectName("measurementPackageVerify")
        self.verify_package_button.setProperty("commandId", "session.verify_package")
        self.verify_package_button.clicked.connect(self._verify_measurement_package)
        package_actions.addWidget(self.verify_package_button)
        self.create_revision_button = QtWidgets.QPushButton("Create Revision")
        self.create_revision_button.setObjectName("measurementPackageCreateRevision")
        self.create_revision_button.setProperty("commandId", "session.create_revision")
        self.create_revision_button.setToolTip(
            "Create a new append-only revision; the sealed revision remains unchanged."
        )
        self.create_revision_button.clicked.connect(self._begin_measurement_revision)
        package_actions.addWidget(self.create_revision_button)
        self.package_action_status_label = QtWidgets.QLabel("Package actions: ready")
        self.package_action_status_label.setObjectName("measurementPackageActionStatus")
        self.package_action_status_label.setWordWrap(True)
        package_actions.addWidget(self.package_action_status_label, 1)
        package_layout.addLayout(package_actions)
        self.package_group = package_group
        tab_layouts["evidence"].addWidget(package_group)
        if self._sessionless:
            package_group.hide()

        self.target_reference = TargetReferenceView(backend, self)
        self.target_reference_group = self.target_reference
        # Named compatibility aliases share the actual view's controls.
        self.run_platform_combo = self.target_reference.platform
        self.target_endpoint_edit = self.target_reference.endpoint
        self.target_system_id_spin = self.target_reference.system
        self.target_component_id_spin = self.target_reference.component
        self.target_connect_button = self.target_reference.connect_button
        self.target_stop_button = self.target_reference.stop_button
        self.run_reference_label = self.target_reference.reference_label
        self.target_stream_state_label = self.target_reference.state_label
        tab_layouts["current"].addWidget(self.target_reference)

        status_group = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QGridLayout(status_group)
        self.indicators: dict[str, StatusIndicator] = {}
        for index, (key, label) in enumerate(
            (
                ("recording", "Recording"),
                ("measurement", "Measurement"),
                ("camera.left", "CAM L"),
                ("camera.right", "CAM R"),
                ("fps", "FPS"),
            )
        ):
            indicator = StatusIndicator(label)
            self.indicators[key] = indicator
            status_layout.addWidget(indicator, index // 2, index % 2)
        self.status_group = status_group
        tab_layouts["current"].addWidget(status_group)

        protocol_group = QtWidgets.QGroupBox("Preflight")
        protocol_layout = QtWidgets.QVBoxLayout(protocol_group)
        protocol_layout.setContentsMargins(6, 6, 6, 6)
        self.protocol_state_label = QtWidgets.QLabel("BLOCKED · evidence waiting")
        self.protocol_state_label.setObjectName("protocolState")
        self.protocol_state_label.setWordWrap(True)
        protocol_layout.addWidget(self.protocol_state_label)
        self.protocol_check_labels: dict[str, QtWidgets.QLabel] = {}
        protocol_checks_layout = QtWidgets.QGridLayout()
        protocol_checks_layout.setContentsMargins(0, 0, 0, 0)
        # Pixhawk mode currently evaluates nine checks; keep one spare row so
        # an additional source-backed gate does not disappear silently.
        for index in range(10):
            label = QtWidgets.QLabel("--")
            label.setObjectName("protocolCheck")
            label.setWordWrap(True)
            self.protocol_check_labels[str(index)] = label
            protocol_checks_layout.addWidget(label, index // 2, index % 2)
        protocol_layout.addLayout(protocol_checks_layout)
        self.preflight_group = protocol_group

        session_group = QtWidgets.QGroupBox("Capture evidence")
        session_layout = QtWidgets.QGridLayout(session_group)
        self.session_values: dict[str, EngineeringValue] = {}
        capture_fields = (
            (
                ("state", "Storage"),
                ("session_id", "Recording mode"),
                ("output_dir", "Data root"),
                ("elapsed_s", "Elapsed"),
                ("status_label", "Writer status"),
            )
            if self._sessionless
            else (
                ("state", "State"),
                ("session_id", "Session ID"),
                ("output_dir", "Output"),
                ("elapsed_s", "Elapsed"),
                ("status_label", "Finalization"),
            )
        )
        for index, (key, label) in enumerate(capture_fields):
            value = EngineeringValue(label)
            self.session_values[key] = value
            session_layout.addWidget(value, index // 2, index % 2)
        hr_capture_controls = QtWidgets.QHBoxLayout()
        hr_capture_controls.setContentsMargins(0, 0, 0, 0)
        hr_capture_controls.addWidget(QtWidgets.QLabel("HR purpose"))
        self.pnp_capture_purpose_combo = QtWidgets.QComboBox()
        self.pnp_capture_purpose_combo.setObjectName("hrCapturePurpose")
        self.pnp_capture_purpose_combo.addItem("Pre", "pre")
        self.pnp_capture_purpose_combo.addItem("Post", "post")
        self.pnp_capture_purpose_combo.addItem("Checkpoint", "checkpoint")
        self.pnp_capture_purpose_combo.addItem("Unspecified", "unspecified")
        hr_capture_controls.addWidget(self.pnp_capture_purpose_combo)
        self.pnp_capture_button = QtWidgets.QPushButton("Capture HR Still")
        self.pnp_capture_button.setProperty("commandId", "recording.hr.capture")
        self.pnp_capture_button.setToolTip(
            "Sends SIHR TRUE, CPTR, and ICPT to both cameras. Photos remain in camera "
            "internal storage. The chosen Pre/Post/Checkpoint purpose is recorded with "
            "the capture event; image import is a later, explicit evidence action."
        )
        self.pnp_capture_button.clicked.connect(self._capture_pnp_hr_pair)
        hr_capture_controls.addWidget(self.pnp_capture_button)
        self.oz_capture_button = QtWidgets.QPushButton("Capture OZ Still")
        self.oz_capture_button.setProperty("commandId", "recording.oz.capture")
        self.oz_capture_button.setToolTip(
            "Sends SIVI TRUE, CPTR, and ICPT to both cameras for a separate visible/OZ "
            "still. It is stored on the camera independently from the recorded RTSP video."
        )
        self.oz_capture_button.clicked.connect(self._capture_pnp_oz_pair)
        hr_capture_controls.addWidget(self.oz_capture_button)
        hr_capture_controls.addStretch(1)
        session_layout.addLayout(hr_capture_controls, 3, 0, 1, 2)
        self.pnp_capture_status_label = QtWidgets.QLabel(
            "Capture HR Still: camera-internal storage · select event, then import exact left/right files"
        )
        self.pnp_capture_status_label.setObjectName("compactContract")
        self.pnp_capture_status_label.setWordWrap(True)
        session_layout.addWidget(self.pnp_capture_status_label, 4, 0, 1, 2)
        self.oz_capture_status_label = QtWidgets.QLabel(
            "Capture OZ Still: camera-internal FHD still · separate from recorded RTSP video"
        )
        self.oz_capture_status_label.setObjectName("compactContract")
        self.oz_capture_status_label.setWordWrap(True)
        session_layout.addWidget(self.oz_capture_status_label, 5, 0, 1, 2)
        self.hr_capture_model = _table_model(
            self,
            ("Capture event", "Purpose", "State", "Left HR", "Right HR"),
        )
        self.hr_capture_table = QtWidgets.QTableView()
        self.hr_capture_table.setObjectName("hrCaptureEventTable")
        self.hr_capture_table.setModel(self.hr_capture_model)
        self.hr_capture_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.hr_capture_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.hr_capture_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.hr_capture_table.verticalHeader().setVisible(False)
        self.hr_capture_table.horizontalHeader().setStretchLastSection(True)
        self.hr_capture_table.setMaximumHeight(112)
        self.hr_capture_table.selectionModel().selectionChanged.connect(
            self._refresh_hr_import_actions
        )
        session_layout.addWidget(self.hr_capture_table, 6, 0, 1, 2)
        hr_import_actions = QtWidgets.QHBoxLayout()
        self.hr_import_left_button = QtWidgets.QPushButton("Import selected Left HR")
        self.hr_import_left_button.setProperty("commandId", "recording.hr.import")
        self.hr_import_left_button.clicked.connect(
            lambda _checked=False: self._import_selected_hr("left")
        )
        self.hr_import_right_button = QtWidgets.QPushButton("Import selected Right HR")
        self.hr_import_right_button.setProperty("commandId", "recording.hr.import")
        self.hr_import_right_button.clicked.connect(
            lambda _checked=False: self._import_selected_hr("right")
        )
        hr_import_actions.addWidget(self.hr_import_left_button)
        hr_import_actions.addWidget(self.hr_import_right_button)
        hr_import_actions.addStretch(1)
        session_layout.addLayout(hr_import_actions, 7, 0, 1, 2)
        self._refresh_hr_import_actions()
        self.capture_evidence_group = session_group
        tab_layouts["evidence"].addWidget(session_group)

        ulog_group = QtWidgets.QGroupBox("PX4 ULog evidence")
        ulog_layout = QtWidgets.QGridLayout(ulog_group)
        self.ulog_source_combo = QtWidgets.QComboBox()
        self.ulog_source_combo.addItem("P1 / Left", "p1")
        self.ulog_source_combo.addItem("P2 / Right", "p2")
        self.ulog_source_combo.addItem("Target Drone", "target")
        self.ulog_ownership_combo = QtWidgets.QComboBox()
        self.ulog_ownership_combo.addItem("Ownership unknown", "UNKNOWN")
        self.ulog_ownership_combo.addItem("Workstation-owned claim", "WORKSTATION")
        self.ulog_ownership_combo.addItem("External logger", "EXTERNAL")
        self.ulog_import_button = QtWidgets.QPushButton("Import ULog")
        self.ulog_import_button.setProperty("commandId", "recording.ulog.import")
        self.ulog_import_button.clicked.connect(self._import_ulog)
        self.ulog_status_label = QtWidgets.QLabel(
            "No ULog imported · automatic capture requires verified logger ACK/status ownership"
        )
        self.ulog_status_label.setObjectName("compactContract")
        self.ulog_status_label.setWordWrap(True)
        ulog_layout.addWidget(QtWidgets.QLabel("Source"), 0, 0)
        ulog_layout.addWidget(self.ulog_source_combo, 0, 1)
        ulog_layout.addWidget(QtWidgets.QLabel("Provenance"), 0, 2)
        ulog_layout.addWidget(self.ulog_ownership_combo, 0, 3)
        ulog_layout.addWidget(self.ulog_import_button, 0, 4)
        ulog_layout.addWidget(self.ulog_status_label, 1, 0, 1, 5)
        self.ulog_model = _table_model(
            self,
            ("Source", "Acquisition", "Ownership", "Bytes", "SHA-256", "Path"),
        )
        self.ulog_table = QtWidgets.QTableView()
        self.ulog_table.setObjectName("ulogEvidenceTable")
        self.ulog_table.setModel(self.ulog_model)
        self.ulog_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.ulog_table.verticalHeader().setVisible(False)
        self.ulog_table.setMaximumHeight(120)
        self.ulog_table.horizontalHeader().setStretchLastSection(True)
        ulog_layout.addWidget(self.ulog_table, 2, 0, 1, 5)
        tab_layouts["evidence"].addWidget(ulog_group)

        data_group = QtWidgets.QGroupBox("Files")
        data_layout = QtWidgets.QVBoxLayout(data_group)
        self.count_values: dict[str, EngineeringValue] = {}
        count_layout = QtWidgets.QGridLayout()
        for index, (key, label) in enumerate(
            (("pairs", "Pairs"), ("camera_left", "CAM L rows"), ("camera_right", "CAM R rows"), ("px4_left", "P1 rows"), ("px4_right", "P2 rows"), ("gnss_base", "GNSS rows"))
        ):
            value = EngineeringValue(label)
            self.count_values[key] = value
            count_layout.addWidget(value, index // 3, index % 3)
        data_layout.addLayout(count_layout)
        self.artifact_model = _table_model(
            self,
            (
                (
                    "Type",
                    "JSONL path",
                    "JSONL state",
                    "CSV path",
                    "CSV state",
                    "JSONL drop",
                    "CSV drop",
                    "Last write",
                    "Error",
                )
                if self._sessionless
                else ("File", "Path", "Kind")
            ),
        )
        self.artifact_table = QtWidgets.QTableView()
        self.artifact_table.setModel(self.artifact_model)
        self.artifact_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.artifact_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.artifact_table.verticalHeader().setVisible(False)
        self.artifact_table.setMaximumHeight(150)
        self.artifact_table.horizontalHeader().setStretchLastSection(True)
        data_layout.addWidget(self.artifact_table)
        if self._sessionless:
            data_actions = QtWidgets.QGridLayout()
            data_actions.setContentsMargins(0, 0, 0, 0)
            data_actions.setHorizontalSpacing(6)
            data_actions.setVerticalSpacing(4)
            self.open_data_folder_button = QtWidgets.QPushButton("Open Data Folder")
            self.open_data_folder_button.setObjectName("dataWorkspaceOpenRoot")
            self.open_data_folder_button.clicked.connect(self._open_data_folder)
            self.open_legacy_recordings_button = QtWidgets.QPushButton(
                "Open Legacy Recordings Folder"
            )
            self.open_legacy_recordings_button.setObjectName("dataWorkspaceOpenLegacy")
            self.open_legacy_recordings_button.clicked.connect(
                self._open_legacy_recordings_folder
            )
            self.retry_storage_button = QtWidgets.QPushButton("Retry Storage")
            self.retry_storage_button.setObjectName("dataWorkspaceRetryStorage")
            self.retry_storage_button.clicked.connect(self._retry_data_storage)
            self.open_measurement_workbook_button = QtWidgets.QPushButton(
                "Open Latest Measurement XLSX"
            )
            self.open_measurement_workbook_button.setObjectName(
                "dataWorkspaceOpenMeasurementWorkbook"
            )
            self.open_measurement_workbook_button.setAccessibleName(
                "Open latest Measurement workbook"
            )
            self.open_measurement_workbook_button.setToolTip(
                "Open the newest <start time>_measurement.xlsx in the data folder "
                "with the system spreadsheet application. Each Measurement "
                "(Start to Stop) has its own workbook."
            )
            self.open_measurement_workbook_button.clicked.connect(
                self._open_measurement_workbook
            )
            data_actions.addWidget(self.open_data_folder_button, 0, 0)
            data_actions.addWidget(self.open_legacy_recordings_button, 0, 1)
            data_actions.addWidget(self.retry_storage_button, 0, 2)
            data_actions.addWidget(self.open_measurement_workbook_button, 1, 0)
            self.rebuild_measurement_workbook_button = QtWidgets.QPushButton("Rebuild Latest XLSX")
            self.rebuild_measurement_workbook_button.setObjectName("dataWorkspaceRebuildMeasurementWorkbook")
            self.rebuild_measurement_workbook_button.setToolTip("Rebuild the latest Measurement workbook from its saved JSONL/CSV inputs.")
            self.rebuild_measurement_workbook_button.clicked.connect(self._rebuild_measurement_workbook)
            data_actions.addWidget(self.rebuild_measurement_workbook_button, 1, 1)
            data_actions.setColumnStretch(3, 1)
            data_layout.addLayout(data_actions)
            self.measurement_workbook_status_label = QtWidgets.QLabel(
                "Measurement XLSX · IDLE"
            )
            self.measurement_workbook_status_label.setObjectName("statusStrip")
            self.measurement_workbook_status_label.setTextInteractionFlags(
                QtCore.Qt.TextSelectableByMouse
            )
            self.measurement_workbook_status_label.setWordWrap(True)
            status_height = (
                self.measurement_workbook_status_label.fontMetrics().lineSpacing() * 2 + 8
            )
            self.measurement_workbook_status_label.setMinimumHeight(status_height)
            self.measurement_workbook_status_label.setMaximumHeight(status_height)
            data_layout.addWidget(self.measurement_workbook_status_label)
            self.method_worker_pane = MethodWorkerRecoveryPane(
                getattr(self.backend, "measurement_worker_snapshot", None),
                getattr(self.backend, "restart_measurement_method_worker", None),
            )
            data_layout.addWidget(self.method_worker_pane)
        tab_layouts["current"].addWidget(data_group)

        review_group = QtWidgets.QGroupBox("Review")
        review_layout = QtWidgets.QVBoxLayout(review_group)
        review_layout.setContentsMargins(6, 6, 6, 6)
        review_layout.setSpacing(5)
        selector_layout = QtWidgets.QGridLayout()
        selector_layout.setHorizontalSpacing(8)
        selector_layout.setColumnStretch(1, 1)
        self.session_selector = QtWidgets.QComboBox()
        self.session_selector.setObjectName("sessionReviewSelector")
        self.session_selector.currentIndexChanged.connect(lambda _index: self._refresh_review_artifacts())
        self.refresh_sessions_button = QtWidgets.QPushButton("Refresh")
        self.refresh_sessions_button.clicked.connect(
            lambda _checked=False: self.refresh_sessions(force=True)
        )
        self.open_session_button = QtWidgets.QPushButton("Open folder")
        self.open_session_button.clicked.connect(self._open_selected_session)
        selector_layout.addWidget(QtWidgets.QLabel("Session"), 0, 0)
        selector_layout.addWidget(self.session_selector, 0, 1)
        selector_layout.addWidget(self.refresh_sessions_button, 0, 2)
        selector_layout.addWidget(self.open_session_button, 0, 3)
        self.artifact_selector = QtWidgets.QComboBox()
        self.artifact_selector.setObjectName("sessionArtifactSelector")
        self.artifact_selector.currentIndexChanged.connect(lambda _index: self._preview_selected_artifact())
        self.preview_artifact_button = QtWidgets.QPushButton("Preview")
        self.preview_artifact_button.clicked.connect(self._preview_selected_artifact)
        self.open_artifact_button = QtWidgets.QPushButton("Open file")
        self.open_artifact_button.clicked.connect(self._open_selected_artifact)
        selector_layout.addWidget(QtWidgets.QLabel("Artifact"), 1, 0)
        selector_layout.addWidget(self.artifact_selector, 1, 1)
        selector_layout.addWidget(self.preview_artifact_button, 1, 2)
        selector_layout.addWidget(self.open_artifact_button, 1, 3)
        review_layout.addLayout(selector_layout)
        self.review_status_label = QtWidgets.QLabel(
            "Read-only review"
        )
        self.review_status_label.setObjectName("statusStrip")
        self.review_status_label.setWordWrap(True)
        self.review_status_label.setToolTip(
            "Saved fields and timestamps are unchanged; no NED/ENU conversion is applied."
        )
        review_layout.addWidget(self.review_status_label)
        self.review_model = QtGui.QStandardItemModel(0, 0, self)
        self.review_table = QtWidgets.QTableView()
        self.review_table.setModel(self.review_model)
        self.review_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.review_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.review_table.verticalHeader().setVisible(False)
        self.review_table.setMaximumHeight(230)
        self.review_table.horizontalHeader().setStretchLastSection(True)
        review_layout.addWidget(self.review_table)
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        tab_layouts["review"].addWidget(review_group)

        self.contract_label = QtWidgets.QLabel(
            "Time: per-record UTC/source clock" if self._sessionless else "Time: recording clock"
        )
        self.contract_label.setToolTip(
            "Every record retains its own UTC, monotonic, and source-clock evidence."
            if self._sessionless
            else "Session IDs use wall time; samples retain the recording time axis."
        )
        self.contract_label.setObjectName("compactContract")
        tab_layouts["evidence"].addWidget(self.contract_label)
        button = QtWidgets.QPushButton("Advanced files")
        button.setToolTip(
            "Open playback, export, and media review tools."
        )
        button.clicked.connect(open_advanced)
        tab_layouts["review"].addWidget(button, alignment=QtCore.Qt.AlignRight)
        for tab_layout in tab_layouts.values():
            tab_layout.addStretch(1)
        self.tabs.currentChanged.connect(self._inspect_current_tab)
        if self._sessionless:
            review_index = self.tabs.indexOf(review_group.parentWidget())
            if review_index >= 0:
                self.tabs.removeTab(review_index)
            self._tab_ids = ("current", "evidence")
            self.tabs.setTabText(0, "Status")
            self.tabs.setTabText(1, "Capture")
            self._refresh_data_storage()
        else:
            self._initialize_session_queries(panel)
            self.refresh_sessions()

    def _data_storage_snapshot(self) -> dict[str, object]:
        reader = getattr(self.backend, "data_storage_snapshot", None)
        if not callable(reader):
            reader = getattr(
                getattr(self.backend, "panel", self.backend),
                "data_storage_snapshot",
                None,
            )
        try:
            value = reader() if callable(reader) else {}
        except Exception as exc:
            value = {"state": "UNAVAILABLE", "streams": {}, "error": str(exc)}
        return dict(value) if isinstance(value, Mapping) else {}

    def _refresh_data_storage(self) -> None:
        if not self._sessionless:
            return
        storage = self._data_storage_snapshot()
        streams = storage.get("streams")
        streams = streams if isinstance(streams, Mapping) else {}
        rows = []
        for name, raw in sorted(streams.items()):
            value = raw if isinstance(raw, Mapping) else {}
            csv_value = value.get("csv")
            csv_value = csv_value if isinstance(csv_value, Mapping) else {}
            last = value.get("last_write_utc_ns")
            last_text = "--" if last in (None, "") else str(last)
            errors = []
            jsonl_error = str(value.get("last_error") or "").strip()
            csv_error = str(csv_value.get("last_error") or "").strip()
            if jsonl_error:
                errors.append(f"JSONL: {jsonl_error}")
            if csv_error:
                errors.append(f"CSV: {csv_error}")
            rows.append(
                (
                    name,
                    value.get("path", "--"),
                    value.get("state", "IDLE"),
                    csv_value.get("path", "--"),
                    csv_value.get("state", "IDLE"),
                    value.get("dropped_records", 0),
                    csv_value.get("dropped_records", 0),
                    last_text,
                    _compact_text(" · ".join(errors), limit=96) or "--",
                )
            )
        _sync_table_rows(self.artifact_model, tuple(rows))
        root = str(storage.get("root") or "--")
        state = str(storage.get("state") or "UNAVAILABLE")
        dropped = int(storage.get("dropped_records") or 0)
        csv_dropped = int(storage.get("csv_dropped_records") or 0)
        self.session_values["state"].set_value(
            state,
            source="MeasurementDataStore",
            valid=True if state == "OK" else False if state == "UNAVAILABLE" else None,
        )
        self.session_values["session_id"].set_value(
            str(getattr(getattr(self.backend, "panel", self.backend), "_recording_mode", "idle")),
            source="transient recording owner",
        )
        self.session_values["output_dir"].set_value(root, source="measurement_data_root")
        self.session_values["status_label"].set_value(
            f"priority={int(storage.get('priority_pending') or 0)} · "
            f"bulk={int(storage.get('bulk_pending') or 0)} · "
            f"dropped={dropped} · csv_dropped={csv_dropped}",
            source="bounded writer queues",
        )
        self._refresh_measurement_workbook()

    def _measurement_workbook_snapshot(self) -> dict[str, object]:
        method = getattr(self.backend, "measurement_workbook_snapshot", None)
        if not callable(method):
            method = getattr(
                getattr(self.backend, "panel", self.backend),
                "measurement_workbook_snapshot",
                None,
            )
        try:
            value = method() if callable(method) else {}
        except Exception as exc:
            value = {"state": "FAILED", "error": str(exc)}
        return dict(value) if isinstance(value, Mapping) else {}

    def _refresh_measurement_workbook(self) -> None:
        if not self._sessionless or not hasattr(self, "measurement_workbook_status_label"):
            return
        if hasattr(self, "method_worker_pane"):
            self.method_worker_pane.refresh()
        snapshot = self._measurement_workbook_snapshot()
        state = (
            "REBUILDING" if snapshot.get("rebuilding") else "REBUILD FAILED"
            if snapshot.get("rebuild_error") else str(snapshot.get("state") or "IDLE").replace("_", " ")
        )
        latest = str(snapshot.get("latest_path") or "")
        latest_name = Path(latest).name if latest else "--"
        pending = int(snapshot.get("pending_method_jobs") or 0)
        error = str(snapshot.get("rebuild_error") or snapshot.get("error") or "")
        detail = (
            _compact_text(error, limit=88)
            if error
            else f"file={_compact_text(latest_name, limit=40)} · pending methods={pending}"
        )
        self.measurement_workbook_status_label.setText(
            f"Measurement XLSX · {state} · {detail}"
        )
        records = snapshot.get("records")
        record_lines = []
        if isinstance(records, (list, tuple)):
            for row in records:
                if not isinstance(row, Mapping):
                    continue
                record_lines.append(
                    f"{row.get('measurement_id', '--')}: {row.get('state', '--')} · "
                    f"pending={row.get('pending_method_jobs', 0)} · "
                    f"{Path(str(row.get('workbook_path') or '')).name or '--'}"
                )
        self.measurement_workbook_status_label.setToolTip(
            "\n".join(
                value
                for value in (
                    f"State: {state}",
                    f"Latest workbook: {latest or '--'}",
                    f"Writes: {snapshot.get('write_count', 0)} · failures: {snapshot.get('failure_count', 0)}",
                    f"Error: {error}" if error else "",
                    *record_lines,
                )
                if value
            )
        )
        self.open_measurement_workbook_button.setEnabled(
            bool(latest and Path(latest).is_file())
        )
        self.rebuild_measurement_workbook_button.setEnabled(
            not snapshot.get("recording_active") and not snapshot.get("rebuilding") and not pending and state != "WRITING"
        )

    def _rebuild_measurement_workbook(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        action = getattr(self.backend, "rebuild_latest_measurement_workbook", None)
        if not callable(action):
            action = getattr(panel, "rebuild_latest_measurement_workbook", None)
        if callable(action):
            action()
        self._refresh_measurement_workbook()

    def _open_measurement_workbook(self) -> None:
        snapshot = self._measurement_workbook_snapshot()
        path = Path(str(snapshot.get("latest_path") or ""))
        if not path.is_file():
            self._refresh_measurement_workbook()
            return
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))
        self._refresh_measurement_workbook()

    def _open_data_folder(self) -> None:
        root = Path(str(self._data_storage_snapshot().get("root") or ""))
        try:
            root.mkdir(parents=True, exist_ok=True)
        except OSError:
            return
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(root)))

    def _open_legacy_recordings_folder(self) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        root = Path(getattr(panel, "app_root", Path.cwd())) / "recordings"
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(root)))

    def _retry_data_storage(self) -> None:
        method = getattr(self.backend, "retry_data_storage", None)
        if not callable(method):
            method = getattr(
                getattr(self.backend, "panel", self.backend),
                "retry_data_storage",
                None,
            )
        if callable(method):
            try:
                method()
            except Exception:
                pass
        self._refresh_data_storage()

    def select_tab(self, tab_id: str) -> bool:
        try:
            index = self._tab_ids.index(str(tab_id))
        except ValueError:
            return False
        self.tabs.setCurrentIndex(index)
        return True

    def current_tab_id(self) -> str:
        index = self.tabs.currentIndex()
        return self._tab_ids[index] if 0 <= index < len(self._tab_ids) else "current"

    def _inspect_current_tab(self, index: int) -> None:
        tab_id = self._tab_ids[index] if 0 <= index < len(self._tab_ids) else "current"
        labels = {
            "current": "Data status" if self._sessionless else "Current session",
            "evidence": "Capture tools" if self._sessionless else "Active session evidence",
            "review": "Saved session review",
        }
        self.inspection_requested.emit(
            {
                "selection": labels[tab_id],
                "value": tab_id.upper(),
                "freshness": "Live owner snapshot" if tab_id != "review" else "Saved catalog",
                "contract": (
                    "Type-owned paths and producer fields are shown without coordinate conversion"
                    if self._sessionless
                    else "Session/package fields are projected without coordinate conversion"
                ),
                "verdict": "Review the selected tab",
                "source": "DataWorkspace" if self._sessionless else "SessionsWorkspace",
            }
        )

    def _refresh_current_measurements(self) -> None:
        reader = self._package_method("measurement_session_snapshot")
        try:
            snapshot = reader() if callable(reader) else {}
        except Exception:
            snapshot = {}
        snapshot = snapshot if isinstance(snapshot, Mapping) else {}
        rows = snapshot.get("measurements")
        rows = rows if isinstance(rows, (tuple, list)) else ()
        self.measurements_model.removeRows(0, self.measurements_model.rowCount())
        for row in rows:
            if not isinstance(row, Mapping):
                continue
            complete = row.get("evidence_complete")
            evidence = (
                "COMPLETE"
                if complete is True
                else "INCOMPLETE"
                if complete is False
                else "PENDING"
            )
            _append_table_row(
                self.measurements_model,
                (
                    row.get("sequence", "--"),
                    row.get("measurement_id", "--"),
                    row.get("state", "--"),
                    evidence,
                    row.get("revision", "--"),
                ),
            )
        for column in range(4):
            self.measurements_table.resizeColumnToContents(column)

    def refresh_run_preparation(self, snapshot: dict[str, RuntimeStatus]) -> None:
        self._refresh_protocol(snapshot)

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        if not self._sessionless:
            self.current_session.refresh(snapshot)
            self._refresh_current_measurements()
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        recording_status = snapshot.get("recording")
        recording_locked = bool(
            recording_status
            and recording_status.state in {"ACTIVE", "FINALIZING"}
        )
        manual_record_button = self.operation_buttons.get("recording.toggle")
        if manual_record_button is not None:
            panel = getattr(self.backend, "panel", self.backend)
            automatic = bool(getattr(panel, "_automatic_session_recording", False))
            manual_active = bool(
                recording_status
                and recording_status.state == "ACTIVE"
                and not automatic
            )
            manual_record_button.setText(
                "Stop manual recording" if manual_active else "Manual record"
            )
            manual_record_button.setEnabled(
                manual_active
                or not recording_status
                or recording_status.state in {"IDLE", "SAVED", "DEGRADED"}
            )
        self.refresh_run_preparation(snapshot)
        data = _evidence(self.backend).recording()
        panel = getattr(self.backend, "panel", self.backend)
        pnp_capture_active = getattr(panel, "pnp_photo_capture_thread", None) is not None
        oz_capture_active = getattr(panel, "oz_photo_capture_thread", None) is not None
        capture_active = bool(pnp_capture_active or oz_capture_active)
        self.pnp_capture_button.setEnabled(not capture_active)
        self.oz_capture_button.setEnabled(not capture_active)
        records = list(getattr(panel, "pnp_photo_capture_records", ()) or ())
        self._sync_hr_capture_records(records)
        if pnp_capture_active:
            self.pnp_capture_status_label.setText("Capture HR Still: in progress")
        elif records:
            latest = records[-1]
            ftp_text = "FTP=ok" if latest.get("ftp_downloaded") else "FTP=disabled/failed"
            self.pnp_capture_status_label.setText(
                f"Capture HR Still: {latest.get('capture_id', 'unknown')} · "
                f"{latest.get('state', 'unknown')} · camera-internal · {ftp_text}"
            )
        else:
            self.pnp_capture_status_label.setText(
                "Capture HR Still: camera-internal storage · select event, then import exact left/right files"
            )
        oz_records = list(getattr(panel, "oz_photo_capture_records", ()) or ())
        if oz_capture_active:
            self.oz_capture_status_label.setText("Capture OZ Still: in progress")
        elif oz_records:
            latest_oz = oz_records[-1]
            oz_ftp_text = "FTP=ok" if latest_oz.get("ftp_downloaded") else "FTP=disabled/failed"
            self.oz_capture_status_label.setText(
                f"Capture OZ Still: {latest_oz.get('capture_id', 'unknown')} · "
                f"{latest_oz.get('state', 'unknown')} · camera-internal · RTSP separate · {oz_ftp_text}"
            )
        else:
            self.oz_capture_status_label.setText(
                "Capture OZ Still: camera-internal FHD still · separate from RTSP video"
            )
        for key, value in self.session_values.items():
            raw = data.get(key)
            if key == "elapsed_s" and raw not in (None, ""):
                raw = f"{float(raw):.1f} s"
            value.set_value(raw, source=str(data.get("source", "")))
        counts = data.get("sample_counts", {})
        for key, value in self.count_values.items():
            value.set_value(counts.get(key, 0) if isinstance(counts, dict) else "--", source="session spool rows")
        if not self._sessionless:
            artifact_rows: list[tuple[object, ...]] = []
            output_dir = str(data.get("output_dir") or "")
            paths = data.get("video_paths", {})
            if isinstance(paths, dict):
                for side, path in sorted(paths.items()):
                    artifact_rows.append((f"{side} video", path or "not prepared", "Video"))
            for artifact in data.get("artifacts", ()) or ():
                artifact_rows.append(
                    (artifact, f"{output_dir}/{artifact}" if output_dir else "--", "Session file")
                )
            if not artifact_rows:
                artifact_rows.append(("--", "No finalized files", "Recording"))
            _sync_table_rows(self.artifact_model, tuple(artifact_rows))
        ulog_reader = getattr(self.backend, "ulog_snapshot", None)
        if callable(ulog_reader):
            try:
                self.apply_ulog_snapshot(ulog_reader())
            except Exception as exc:
                self.ulog_status_label.setText(f"ULog status unavailable · {exc}")
        target_drone_reader = getattr(self.backend, "target_drone_snapshot", None)
        if callable(target_drone_reader):
            try:
                self._apply_target_drone_snapshot(target_drone_reader())
            except Exception as exc:
                self.target_stream_state_label.setText(f"Target Drone: status unavailable · {exc}")
        if self._sessionless:
            self._refresh_data_storage()
        else:
            self.refresh_sessions(force=False)
            self._refresh_package_details()

    def _package_method(self, *names: str) -> Callable[..., object] | None:
        panel = getattr(self.backend, "panel", self.backend)
        for owner in (self.backend, panel):
            for name in names:
                method = getattr(owner, name, None)
                if callable(method):
                    return method
        return None

    @staticmethod
    def _record_package_snapshot(record: Mapping[str, object] | None) -> dict[str, object]:
        if not isinstance(record, Mapping):
            return {}
        values = dict(record)
        for key in ("measurement_package", "package_manifest", "package"):
            nested = record.get(key)
            if isinstance(nested, Mapping):
                values.update(nested)
        if not values.get("state"):
            values["state"] = values.get("lifecycle_state", values.get("status", "IDLE"))
        if not values.get("evidence_completeness") and values.get("completeness_state"):
            values["evidence_completeness"] = {
                "status": values.get("completeness_state"),
                "complete": values.get("evidence_complete"),
                "missing_required": values.get("missing_reasons", ()),
            }
        return values

    def _selected_package_snapshot(self) -> dict[str, object]:
        record = self._selected_record() if hasattr(self, "session_selector") else None
        selected = self._record_package_snapshot(record)
        reader = self._package_method("measurement_package_snapshot")
        if reader is None:
            return selected
        try:
            live = reader()
        except Exception as exc:
            self.package_action_status_label.setText(f"Package status unavailable: {exc}")
            return selected
        if not isinstance(live, Mapping):
            return selected
        live_values = dict(live)
        selected_id = str(selected.get("session_id", "") or "")
        selected_measurement_id = str(selected.get("measurement_id", "") or "")
        live_id = str(live_values.get("session_id", "") or "")
        live_measurement_id = str(live_values.get("measurement_id", "") or "")
        live_state = str(live_values.get("state", "IDLE") or "IDLE").upper()
        if selected_id and live_id and (
            selected_id != live_id
            or selected_measurement_id != live_measurement_id
        ):
            selected["_actions_available"] = False
            return selected
        if live_state in {"COLLECTING", "POST_CAPTURE", "VERIFYING", "RECOVERABLE"}:
            live_values["_actions_available"] = True
            return live_values
        if selected_id and not live_id:
            selected["_actions_available"] = False
            return selected
        selected.update(live_values)
        if isinstance(live_values.get("artifacts"), (tuple, list, Mapping)):
            selected["artifact_details"] = live_values["artifacts"]
        selected["_actions_available"] = True
        return selected

    @staticmethod
    def _package_mapping_rows(value: object, *, identity_key: str) -> list[dict[str, object]]:
        if isinstance(value, Mapping):
            rows: list[dict[str, object]] = []
            for identity, raw in value.items():
                if isinstance(raw, Mapping):
                    row = dict(raw)
                else:
                    row = {"state": raw}
                row.setdefault(identity_key, str(identity))
                rows.append(row)
            return rows
        if isinstance(value, (tuple, list)):
            return [dict(raw) for raw in value if isinstance(raw, Mapping)]
        return []

    @staticmethod
    def _source_expectation_row(source: Mapping[str, object]) -> tuple[object, ...]:
        source_id = source.get(
            "source_id",
            source.get("producer_id", source.get("name", source.get("id", "--"))),
        )
        required_roles = source.get("required_roles")
        if isinstance(required_roles, (tuple, list)):
            expected = ", ".join(str(role) for role in required_roles) or "none"
        else:
            required = source.get("required", source.get("expected", True))
            expected = (
                "required"
                if required is True
                else "optional"
                if required is False
                else str(required or "--")
            )
        state = str(source.get("state", source.get("status", "")) or "")
        if not state and "active" in source:
            state = "ACTIVE" if bool(source.get("active")) else "INACTIVE"
        metadata = source.get("metadata")
        metadata = metadata if isinstance(metadata, Mapping) else {}
        start = source.get(
            "active_from",
            source.get(
                "activated_utc_ns",
                source.get(
                    "active_from_pair_sequence",
                    metadata.get("active_from", metadata.get("activated_utc_ns")),
                ),
            ),
        )
        end = source.get(
            "active_until",
            source.get(
                "deactivated_utc_ns",
                source.get(
                    "active_to_pair_sequence",
                    metadata.get("active_until", metadata.get("deactivated_utc_ns")),
                ),
            ),
        )
        active_range = " → ".join(
            ("--" if value in (None, "") else str(value)) for value in (start, end)
        )
        state_range = state or "--"
        if start not in (None, "") or end not in (None, ""):
            state_range += f" · {active_range}"
        detail = source.get(
            "detail",
            source.get(
                "reason",
                source.get(
                    "expectation",
                    metadata.get("detail", metadata.get("reason", source.get("role", ""))),
                ),
            ),
        )
        return (source_id, expected, state_range, detail or "--")

    @staticmethod
    def _artifact_inventory_row(
        artifact: Mapping[str, object],
        *,
        package_revision: object,
    ) -> tuple[object, ...]:
        path = artifact.get(
            "relative_path",
            artifact.get("path", artifact.get("name", artifact.get("artifact", "--"))),
        )
        role = artifact.get("role", artifact.get("artifact_role", artifact.get("kind", "--")))
        required = artifact.get("required")
        required_text = "yes" if required is True else "no" if required is False else "--"
        state = artifact.get("state", artifact.get("status", "--"))
        size_bytes = artifact.get("size_bytes", artifact.get("bytes", "--"))
        sha256 = artifact.get("sha256", artifact.get("hash", "--"))
        revision = artifact.get("revision", package_revision or "--")
        metadata = artifact.get("metadata")
        metadata = metadata if isinstance(metadata, Mapping) else {}
        reason = artifact.get(
            "failure_reason",
            artifact.get(
                "missing_reason",
                artifact.get(
                    "reason",
                    artifact.get(
                        "error",
                        artifact.get(
                            "detail",
                            metadata.get(
                                "failure_reason",
                                metadata.get("missing_reason", metadata.get("reason", "")),
                            ),
                        ),
                    ),
                ),
            ),
        )
        return (
            path,
            role,
            required_text,
            state,
            size_bytes if size_bytes not in (None, "") else "--",
            sha256 or "--",
            revision,
            reason or "--",
        )

    @staticmethod
    def _reason_count(value: object) -> int:
        if isinstance(value, Mapping):
            return len(value)
        if isinstance(value, (tuple, list, set)):
            return len(value)
        return 1 if value not in (None, "") else 0

    def _apply_package_details(self, package: Mapping[str, object]) -> None:
        state = str(package.get("state", package.get("lifecycle_state", "IDLE")) or "IDLE").upper()
        session_id = str(package.get("session_id", "") or "")
        self.package_state_label.setText(
            f"State: {state}" + (f" · {session_id}" if session_id else "")
        )
        revision = package.get("revision", package.get("revision_number", "--"))
        self.package_revision_label.setText(f"Revision: {revision or '--'}")

        completeness = package.get("evidence_completeness")
        completeness = completeness if isinstance(completeness, Mapping) else {}
        completeness_state = str(
            completeness.get(
                "status",
                package.get("completeness_state", package.get("completeness", "")),
            )
            or ""
        ).upper()
        if not completeness_state:
            completeness_state = (
                "COMPLETE"
                if state == "SEALED_COMPLETE"
                else "INCOMPLETE"
                if state in {"SEALED_INCOMPLETE", "INTEGRITY_FAILED"}
                else "PENDING"
            )
        missing_count = max(
            self._reason_count(completeness.get("missing_required")),
            self._reason_count(package.get("missing_reasons")),
        )
        failed_count = self._reason_count(package.get("failed_reasons"))
        reason_summary = ", ".join(
            text
            for count, text in (
                (missing_count, f"missing={missing_count}"),
                (failed_count, f"failed={failed_count}"),
            )
            if count
        )
        self.package_completeness_label.setText(
            f"Evidence: {completeness_state}"
            + (f" · {reason_summary}" if reason_summary else "")
        )

        verification = package.get("hash_verification", package.get("verification"))
        verification = verification if isinstance(verification, Mapping) else {}
        verification_state = str(
            package.get(
                "integrity_status",
                package.get(
                    "verification_state",
                    verification.get("status", verification.get("state", "NOT RUN")),
                ),
            )
            or "NOT RUN"
        ).upper()
        verification_parts = []
        manifest_status = verification.get("manifest_status")
        if manifest_status not in (None, ""):
            verification_parts.append(f"manifest={manifest_status}")
        declared = verification.get(
            "artifact_declared_count", verification.get("artifact_hashes_declared")
        )
        not_run = verification.get(
            "artifact_not_run_count", verification.get("artifacts_not_run")
        )
        if declared not in (None, ""):
            verification_parts.append(f"artifacts={declared}")
        if not_run not in (None, "", 0):
            verification_parts.append(f"not-run={not_run}")
        self.package_verification_label.setText(
            f"Verification: {verification_state}"
            + (f" · {', '.join(verification_parts)}" if verification_parts else "")
        )

        sources = self._package_mapping_rows(
            package.get("source_expectations"), identity_key="source_id"
        )
        source_rows = tuple(self._source_expectation_row(source) for source in sources)
        _sync_table_rows(self.package_source_model, source_rows)

        artifact_values = package.get("artifact_details", package.get("artifacts"))
        artifacts = self._package_mapping_rows(artifact_values, identity_key="relative_path")
        artifact_rows = tuple(
            self._artifact_inventory_row(artifact, package_revision=revision)
            for artifact in artifacts
        )
        _sync_table_rows(self.package_artifact_model, artifact_rows)

        sealed = state in {"SEALED_COMPLETE", "SEALED_INCOMPLETE"}
        actions_available = bool(package.get("_actions_available", True))
        self.verify_package_button.setEnabled(
            actions_available
            and state in {"SEALED_COMPLETE", "SEALED_INCOMPLETE", "INTEGRITY_FAILED"}
            and self._package_method("verify_measurement_package", "verify_package") is not None
        )
        self.create_revision_button.setEnabled(
            actions_available
            and sealed
            and self._package_method("begin_measurement_revision", "create_revision") is not None
        )

    def _refresh_package_details(self) -> None:
        package = self._selected_package_snapshot()
        if not package:
            self._apply_package_details({"state": "IDLE"})
            return
        self._apply_package_details(package)

    def _verify_measurement_package(self) -> None:
        method = self._package_method("verify_measurement_package", "verify_package")
        if method is None:
            self.package_action_status_label.setText("Verify package: control unavailable")
            return
        try:
            result = method()
        except Exception as exc:
            self.package_action_status_label.setText(f"Verify package: ERROR · {exc}")
            return
        self._refresh_package_details()
        result_mapping = result if isinstance(result, Mapping) else None
        if result_mapping is None:
            mapper = getattr(result, "to_mapping", None)
            try:
                mapped = mapper() if callable(mapper) else None
            except Exception:
                mapped = None
            result_mapping = mapped if isinstance(mapped, Mapping) else None
        if result_mapping is not None:
            state = str(
                result_mapping.get("status", result_mapping.get("state", "completed"))
                or "completed"
            )
            errors = result_mapping.get("errors", result_mapping.get("issues", ()))
            error_count = self._reason_count(errors)
            self.package_verification_label.setText(
                f"Verification: {state.upper()}"
                + (f" · issues={error_count}" if error_count else "")
            )
        self.package_action_status_label.setText("Verify package: completed")

    def _begin_measurement_revision(self) -> None:
        method = self._package_method("begin_measurement_revision", "create_revision")
        if method is None:
            self.package_action_status_label.setText("Create Revision: control unavailable")
            return
        try:
            result = method()
        except Exception as exc:
            self.package_action_status_label.setText(f"Create Revision: ERROR · {exc}")
            return
        self._refresh_package_details()
        self.package_action_status_label.setText(
            "Create Revision: started"
            if result is not False
            else "Create Revision: request rejected"
        )

    def target_drone_payload(self) -> dict[str, object]:
        return self.target_reference.payload()

    def _connect_target_drone(self) -> None:
        self.target_reference.connect_target()

    def _stop_target_drone(self) -> None:
        self.target_reference.stop_target()

    def _apply_target_drone_snapshot(self, value: object) -> None:
        if isinstance(value, Mapping):
            self.target_reference.render_connection(value)
        self.target_reference.refresh()

    def _refresh_run_reference_label(self) -> None:
        self.target_reference.refresh_mode()

    def _capture_pnp_hr_pair(self) -> None:
        payload = {
            "purpose": str(
                self.pnp_capture_purpose_combo.currentData()
                or "unspecified"
            ),
            "storage": "camera_internal_then_exact_file_import",
        }
        self._request_hr_command("recording.hr.capture", payload)

    def _capture_pnp_oz_pair(self) -> None:
        payload = {
            "purpose": str(
                self.pnp_capture_purpose_combo.currentData()
                or "unspecified"
            ),
            "storage": "camera_internal_separate_from_rtsp",
            "capture_channel": "oz",
        }
        self._request_hr_command("recording.oz.capture", payload)

    @staticmethod
    def _hr_side_state(record: Mapping[str, object], side: str) -> str:
        imported = record.get(side)
        if isinstance(imported, Mapping):
            digest = str(imported.get("sha256", "") or "")
            return f"IMPORTED {digest[:8]}…" if digest else str(imported.get("state", "IMPORTED"))
        cameras = record.get("cameras")
        camera = cameras.get(side) if isinstance(cameras, Mapping) else None
        if isinstance(camera, Mapping):
            return str(camera.get("import_state", camera.get("state", "--")) or "--")
        return "--"

    def _sync_hr_capture_records(self, records: list[object]) -> None:
        selected_capture_id = self._selected_hr_capture_id()
        rows: list[tuple[object, ...]] = []
        for raw in records:
            if not isinstance(raw, Mapping):
                continue
            rows.append(
                (
                    raw.get("capture_id", raw.get("event_id", "--")),
                    raw.get("purpose", "unspecified"),
                    raw.get("state", "--"),
                    self._hr_side_state(raw, "left"),
                    self._hr_side_state(raw, "right"),
                )
            )
        _sync_table_rows(self.hr_capture_model, tuple(rows))
        selected_row = -1
        if selected_capture_id:
            for row in range(self.hr_capture_model.rowCount()):
                item = self.hr_capture_model.item(row, 0)
                if item is not None and item.text() == selected_capture_id:
                    selected_row = row
                    break
        if selected_row < 0 and rows:
            selected_row = len(rows) - 1
        if selected_row >= 0:
            self.hr_capture_table.selectRow(selected_row)
        else:
            self.hr_capture_table.clearSelection()
        self._refresh_hr_import_actions()

    def _selected_hr_capture_id(self) -> str:
        rows = self.hr_capture_table.selectionModel().selectedRows()
        row = rows[0].row() if rows else self.hr_capture_table.currentIndex().row()
        item = self.hr_capture_model.item(row, 0) if row >= 0 else None
        return "" if item is None else item.text().strip()

    def _selected_hr_capture_purpose(self) -> str:
        rows = self.hr_capture_table.selectionModel().selectedRows()
        row = rows[0].row() if rows else self.hr_capture_table.currentIndex().row()
        item = self.hr_capture_model.item(row, 1) if row >= 0 else None
        return "unspecified" if item is None else (item.text().strip().lower() or "unspecified")

    def _refresh_hr_import_actions(self, *_args: object) -> None:
        enabled = bool(self._selected_hr_capture_id())
        self.hr_import_left_button.setEnabled(enabled)
        self.hr_import_right_button.setEnabled(enabled)

    def _import_selected_hr(self, side: str) -> None:
        capture_id = self._selected_hr_capture_id()
        if not capture_id:
            self.pnp_capture_status_label.setText("HR Import: select a capture event")
            return
        self._request_hr_command(
            "recording.hr.import",
            {
                "capture_id": capture_id,
                "purpose": self._selected_hr_capture_purpose(),
                "side": str(side),
                "file_selection": "owner_dialog_required",
                "byte_contract": "exact source bytes; owner computes SHA-256 and decoded resolution",
            },
        )

    def _import_ulog(self) -> None:
        # Qt delivers same-thread slots synchronously.  Publish the pending
        # state first so a completed owner readback is the last writer.
        self.ulog_status_label.setText("ULog import requested: awaiting owner file selection")
        self.ulog_command_requested.emit(
            "recording.ulog.import",
            {
                "source": str(self.ulog_source_combo.currentData() or "p1"),
                "ownership": str(self.ulog_ownership_combo.currentData() or "UNKNOWN"),
                "file_selection": "owner_dialog_required",
            },
        )

    def apply_ulog_snapshot(self, snapshot: object) -> None:
        if not isinstance(snapshot, Mapping):
            return
        artifacts = snapshot.get("artifacts")
        rows: list[tuple[object, ...]] = []
        for artifact in artifacts if isinstance(artifacts, (tuple, list)) else ():
            if not isinstance(artifact, Mapping):
                continue
            rows.append(
                (
                    artifact.get("source_id", "--"),
                    artifact.get("acquisition", "--"),
                    artifact.get("ownership", "--"),
                    artifact.get("size_bytes", "--"),
                    str(artifact.get("sha256", ""))[:16],
                    artifact.get("relative_path", "--"),
                )
            )
        _sync_table_rows(self.ulog_model, tuple(rows))
        if rows:
            self.ulog_status_label.setText(
                f"ULog artifacts: {len(rows)} · exact bytes/hash verified · runtime telemetry remains reference"
            )
        elif str(snapshot.get("state", "")) == "NOT_PREPARED":
            self.ulog_status_label.setText(
                "No ULog imported · automatic capture requires verified logger ACK/status ownership"
            )

    def _request_hr_command(self, command_id: str, payload: dict[str, object]) -> None:
        """Emit typed camera-still work, with legacy capture fallback when unconnected.

        The presentation layer never opens or hashes an HR file.  Main-window
        or controller owners receive only the selected event/side/purpose and
        perform the file dialog, exact-byte import, validation, and storage.
        """

        if self.receivers(self.hr_command_requested) > 0:
            action = "Capture requested" if command_id.endswith("capture") else "Import requested"
            status_label = (
                self.pnp_capture_status_label
                if command_id.startswith("recording.hr")
                else self.oz_capture_status_label
            )
            channel = "HR" if command_id.startswith("recording.hr") else "OZ"
            status_label.setText(f"{channel} {action}: awaiting owner")
            # Direct Qt connections run before emit() returns.  Set pending
            # first so the owner's synchronous success/failure text wins.
            self.hr_command_requested.emit(command_id, dict(payload))
            return
        if command_id == "recording.hr.capture":
            method = getattr(self.backend, "capture_pnp_hr_photos", None)
            status_label = self.pnp_capture_status_label
            channel = "HR"
        elif command_id == "recording.oz.capture":
            method = getattr(self.backend, "capture_pnp_oz_photos", None)
            status_label = self.oz_capture_status_label
            channel = "OZ"
        else:
            method = getattr(self.backend, "import_pnp_hr_photo", None)
            status_label = self.pnp_capture_status_label
            channel = "HR"
        if not callable(method):
            status_label.setText(f"{channel} {command_id.rsplit('.', 1)[-1].title()}: control unavailable")
            return
        try:
            try:
                result = method(dict(payload))
            except TypeError:
                # Keep the established no-argument capture facade functional
                # until its owner adopts the typed purpose payload.
                if command_id not in {"recording.hr.capture", "recording.oz.capture"}:
                    raise
                result = method()
            if result is not False:
                state = "in progress" if command_id.endswith("capture") else "requested"
                status_label.setText(f"{channel} {command_id.rsplit('.', 1)[-1].title()}: {state}")
        except Exception as exc:
            status_label.setText(f"{channel} {command_id.rsplit('.', 1)[-1].title()}: {exc}")

    def _refresh_protocol(self, snapshot: dict[str, RuntimeStatus]) -> None:
        evidence = _evidence(self.backend)
        panel = getattr(self.backend, "panel", self.backend)
        settings = getattr(panel, "settings", None)
        snapshot_method = getattr(self.backend, "workstation_settings_snapshot", None)
        try:
            settings_snapshot = snapshot_method() if callable(snapshot_method) else {}
        except Exception:
            settings_snapshot = {}
        settings_snapshot = settings_snapshot if isinstance(settings_snapshot, Mapping) else {}
        gnss_config = settings_snapshot.get("gnss_operation")
        gnss_config = gnss_config if isinstance(gnss_config, Mapping) else {}
        readiness_config = gnss_config.get("readiness")
        readiness_config = readiness_config if isinstance(readiness_config, Mapping) else {}
        try:
            minimum_fix_type = int(readiness_config.get("minimum_fix_type", 3))
        except (TypeError, ValueError):
            minimum_fix_type = 3
        physical_alignment = evidence.physical_alignment()
        checkpoints = physical_alignment.get("time_sync_checkpoints")
        if not isinstance(checkpoints, (tuple, list)):
            checkpoints = getattr(panel, "workstation_time_sync_checkpoints", ())
        final_sync_verified = any(
            isinstance(item, Mapping)
            and str(item.get("phase", "")) == "final"
            and "VERIFIED" in str(item.get("status", "")).upper()
            for item in checkpoints or ()
        )
        measurement = evidence.measurement_settings()
        method_snapshot_reader = getattr(self.backend, "measurement_methods_snapshot", None)
        try:
            method_snapshot = method_snapshot_reader() if callable(method_snapshot_reader) else {}
        except Exception:
            method_snapshot = {}
        selected_method: dict[str, object] | None = None
        if isinstance(method_snapshot, Mapping):
            selected_id = str(method_snapshot.get("selected_method_id") or "")
            rows = method_snapshot.get("ready_methods")
            if isinstance(rows, list):
                selected_method = next(
                    (
                        dict(row)
                        for row in rows
                        if isinstance(row, Mapping)
                        and str(row.get("method_id") or "") == selected_id
                    ),
                    None,
                )
            if selected_method is None and str(method_snapshot.get("schema") or ""):
                selected_method = {
                    "method_id": "",
                    "instance_id": "",
                    "label": "Ready measurement method",
                    "ready": False,
                    "reasons": ["no ready measurement method"],
                    "requires_live_pixhawk": False,
                    "requires_live_gnss": False,
                }
        readiness = evaluate_recording_protocol(
            snapshot,
            distance_mode=str(measurement.get("distance_mode", "pixhawk") or "pixhawk"),
            calibration=evidence.calibration(),
            baseline=evidence.baseline(),
            require_rtk_fixed=bool(
                settings_snapshot.get(
                    "require_rtk_fixed",
                    getattr(settings, "require_rtk_fixed", False),
                )
            ),
            minimum_fix_type=minimum_fix_type,
            physical_alignment=physical_alignment,
            final_sync_verified=final_sync_verified,
            measurement_method=selected_method,
        )
        self.protocol_state_label.setText(f"{readiness.state} · {len(readiness.blocking)} checks")
        state_style = (
            f"color:{SUCCESS_TEXT}; border:1px solid {PANEL_BORDER}; padding:5px;"
            if readiness.state == "READY"
            else f"color:{ERROR_TEXT}; border:1px solid {ERROR_TEXT}; padding:5px;"
        )
        self.protocol_state_label.setStyleSheet(state_style)
        for index, label in enumerate(self.protocol_check_labels.values()):
            if index >= len(readiness.checks):
                label.clear()
                label.setVisible(False)
                continue
            check = readiness.checks[index]
            label.setVisible(True)
            detail = str(check.detail)
            label.setText(f"{'✓' if check.passed else '×'} {check.label}: {_compact_text(detail, limit=112)}")
            label.setToolTip(detail)
            label.setStyleSheet(
                f"color:{SUCCESS_TEXT};" if check.passed else f"color:{ERROR_TEXT};"
            )

    def _initialize_session_queries(self, panel) -> None:
        application = getattr(panel, "application", None)
        if not isinstance(panel, QtCore.QObject) or application is None:
            # Legacy lightweight owners have no application thread lifecycle.
            # Keep their historical synchronous adapter API for compatibility.
            return
        register = getattr(application, "register_evidence_query_runtime", None)
        if not callable(register):
            raise TypeError("PanelApplication must register session evidence query resources")
        from Workstation.evidence_views.query_runtime import SessionEvidenceQueryRuntime

        runtime = SessionEvidenceQueryRuntime(panel)
        runtime.completed.connect(self._handle_evidence_query_result, QtCore.Qt.QueuedConnection)
        self.destroyed.connect(runtime.request_shutdown)
        register(runtime)
        self._evidence_query_runtime = runtime

    def _session_query_root(self) -> str:
        # The legacy adapter owns the existing app_root/fallback contract.
        # Capture that scalar on the GUI; the worker receives no adapter/host.
        return os.path.abspath(os.fspath(_evidence(self.backend)._app_root()))

    def dispose_view_resources(self) -> bool:
        self._query_disposed = True
        self._catalog_query_request = None
        self._artifact_query_request = None
        runtime = self._evidence_query_runtime
        return True if runtime is None else runtime.request_shutdown()

    @QtCore.pyqtSlot(object)
    def _handle_evidence_query_result(self, result) -> None:
        runtime = self._evidence_query_runtime
        if self._query_disposed or runtime is None:
            return
        request = result.request
        if not runtime.is_current(request) or request.data_root != self._session_query_root():
            return
        if request.kind == "catalog":
            if request != self._catalog_query_request:
                return
            if result.error:
                self._catalog_query_failed = True
                self.review_status_label.setText(f"Session list unavailable: {result.error}")
                return
            refresh_preview = self._catalog_query_failed
            self._catalog_query_failed = False
            self._apply_session_catalog(result.data(), refresh_preview=refresh_preview)
            return
        if request != self._artifact_query_request:
            return
        record = self._selected_record()
        artifact = self._selected_artifact_name()
        if record is None or (
            request.session_id != str(record.get("session_id") or "")
            or request.measurement_id != str(record.get("measurement_id") or "")
            or request.artifact_name != artifact
        ):
            return
        data = {"error": result.error} if result.error else result.data()
        self._apply_artifact_preview(data, record, artifact)

    def refresh_sessions(self, *, force: bool = True) -> None:
        if self._query_disposed:
            return
        if self._sessionless:
            self._refresh_data_storage()
            return
        runtime = self._evidence_query_runtime
        root = self._session_query_root() if runtime is not None else ""
        root_changed = root != self._last_session_scan_root
        now_s = time.monotonic()
        if (
            not force and not root_changed
            and now_s - self._last_session_scan_monotonic_s
            < SESSION_CATALOG_REFRESH_INTERVAL_S
        ):
            return
        if not force and not root_changed and runtime is not None and runtime.has_work("catalog"):
            # A slow hash scan must eventually deliver. Periodic refreshes do
            # not supersede identical in-flight scans; explicit Refresh does.
            return
        self._last_session_scan_monotonic_s = now_s
        if runtime is None:
            self._apply_session_catalog(_evidence(self.backend).session_catalog())
            return
        if root_changed:
            runtime.invalidate()
            self._catalog_query_request = None
            self._artifact_query_request = None
            self._session_catalog_signature = ()
            self.session_records = {}
            for selector in (self.session_selector, self.artifact_selector):
                selector.blockSignals(True)
                try:
                    selector.clear()
                finally:
                    selector.blockSignals(False)
            self.review_model.clear()
        self._last_session_scan_root = root
        if not self.session_records:
            self.review_status_label.setText("Loading saved sessions…")
        self._catalog_query_request = runtime.submit_catalog(root)

    def _apply_session_catalog(self, records, *, refresh_preview=False) -> None:
        signature = tuple(
            (
                record.get("session_id"),
                record.get("measurement_id"),
                record.get("status"),
                record.get("lifecycle_state"),
                record.get("completeness_state"),
                record.get("revision"),
                record.get("integrity_status"),
                record.get("error"),
                record.get("retryable"),
                tuple(record.get("artifacts", ()) or ()),
                json.dumps(record.get("source_expectations", ()), sort_keys=True, default=str),
                json.dumps(record.get("artifact_details", ()), sort_keys=True, default=str),
                json.dumps(record.get("hash_verification", {}), sort_keys=True, default=str),
            )
            for record in records
        )
        if signature == self._session_catalog_signature and self.session_selector.count() > 0:
            if refresh_preview:
                self._preview_selected_artifact()
            return
        current = str(self.session_selector.currentData() or "")
        panel = getattr(self.backend, "panel", self.backend)
        if not current:
            panel_session_id = str(getattr(panel, "recording_session_id", "") or "")
            panel_measurement_id = str(getattr(panel, "_measurement_id", "") or "")
            current = (
                f"{panel_session_id}::{panel_measurement_id}"
                if panel_session_id and panel_measurement_id
                else panel_session_id
            )
        self.session_records = {
            str(record.get("catalog_key") or record.get("session_id")): dict(record)
            for record in records
            if record.get("session_id")
        }
        self._session_catalog_signature = signature
        self.session_selector.blockSignals(True)
        try:
            self.session_selector.clear()
            for record in records:
                session_id = str(record.get("session_id") or "")
                measurement_id = str(record.get("measurement_id") or "")
                catalog_key = str(record.get("catalog_key") or session_id)
                status = str(record.get("status") or "unknown")
                identity = f"{session_id} / {measurement_id}" if measurement_id else session_id
                self.session_selector.addItem(f"{identity} [{status}]", catalog_key)
            if current:
                index = self.session_selector.findData(current)
                if index >= 0:
                    self.session_selector.setCurrentIndex(index)
        finally:
            self.session_selector.blockSignals(False)
        self._refresh_review_artifacts()

    def _selected_record(self) -> dict[str, object] | None:
        catalog_key = str(self.session_selector.currentData() or "")
        return self.session_records.get(catalog_key)

    def _refresh_review_artifacts(self) -> None:
        record = self._selected_record()
        current = str(self.artifact_selector.currentData() or "")
        self.artifact_selector.blockSignals(True)
        try:
            self.artifact_selector.clear()
            if record is not None:
                for artifact in record.get("artifacts", ()) or ():
                    self.artifact_selector.addItem(str(artifact), str(artifact))
            if current:
                index = self.artifact_selector.findData(current)
                if index >= 0:
                    self.artifact_selector.setCurrentIndex(index)
            if self.artifact_selector.currentIndex() < 0 and self.artifact_selector.count() > 0:
                self.artifact_selector.setCurrentIndex(0)
        finally:
            self.artifact_selector.blockSignals(False)
        self._preview_selected_artifact()
        self._refresh_package_details()

    def _selected_artifact_name(self) -> str:
        return str(self.artifact_selector.currentData() or "")

    def _selected_artifact_path(self) -> Path | None:
        record = self._selected_record()
        if record is None:
            return None
        output_dir = Path(str(record.get("path") or ""))
        artifact = self._selected_artifact_name()
        if not artifact:
            return output_dir if output_dir.exists() else None
        relative = Path(artifact)
        if relative.is_absolute() or ".." in relative.parts:
            return None
        root = output_dir.resolve()
        candidates = (output_dir / relative, output_dir / "published" / relative)
        for candidate in candidates:
            try:
                candidate.resolve().relative_to(root)
            except (OSError, ValueError):
                continue
            if candidate.exists():
                return candidate
        return candidates[0]

    def _open_selected_session(self) -> None:
        record = self._selected_record()
        if record is None:
            self.review_status_label.setText("No session selected")
            return
        self._open_path(Path(str(record.get("path") or "")))

    def _open_selected_artifact(self) -> None:
        path = self._selected_artifact_path()
        if path is None:
            self.review_status_label.setText("No file selected")
            return
        self._open_path(path)

    def _open_path(self, path: Path) -> None:
        if not path.exists():
            self.review_status_label.setText(f"Path unavailable: {path}")
            return
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))
        self.review_status_label.setText(f"Opened: {path}")

    def _preview_selected_artifact(self) -> None:
        if self._query_disposed:
            return
        record = self._selected_record()
        artifact = self._selected_artifact_name()
        runtime = self._evidence_query_runtime
        if record is None or not artifact:
            if runtime is not None:
                runtime.invalidate("artifact")
                self._artifact_query_request = None
            self.review_model.clear()
            self.review_status_label.setText("Select a session and file")
            return
        measurement_id = str(record.get("measurement_id") or "")
        if runtime is not None:
            self.review_model.clear()
            self.review_status_label.setText(f"Loading {artifact}…")
            self._artifact_query_request = runtime.submit_artifact(
                self._session_query_root(), str(record.get("session_id") or ""),
                artifact, measurement_id=measurement_id or None,
            )
            return
        if measurement_id:
            data = _evidence(self.backend).session_artifact_rows(
                str(record.get("session_id")), artifact, measurement_id=measurement_id,
            )
        else:
            data = _evidence(self.backend).session_artifact_rows(
                str(record.get("session_id")), artifact,
            )
        self._apply_artifact_preview(data, record, artifact)

    def _apply_artifact_preview(self, data, record, artifact) -> None:
        error = str(data.get("error") or "")
        if error:
            self.review_model.clear()
            self.review_status_label.setText(f"{artifact}: {error}")
            return
        columns = tuple(str(value) for value in data.get("columns", ()) or ())
        rows = data.get("rows", ()) or ()
        self.review_model.clear()
        self.review_model.setColumnCount(len(columns))
        self.review_model.setHorizontalHeaderLabels(list(columns))
        for row in rows:
            if not isinstance(row, dict):
                continue
            _append_table_row(self.review_model, tuple(row.get(column, "") for column in columns))
        record_error = str(record.get("error") or "")
        failure_note = f" | manifest error: {record_error}" if record_error else ""
        review_detail = (
            f"{artifact}: {len(rows)} rows · {data.get('time_axis', 'stored time axis')} · "
            "fields unchanged · no NED/ENU conversion"
            f"{failure_note}"
        )
        self.review_status_label.setText(f"{artifact}: {len(rows)} rows")
        self.review_status_label.setToolTip(
            f"{review_detail}\n{data.get('coordinate_contract', 'stored fields preserved')}"
        )


# Public Data vocabulary retains the two historical class aliases.
DataWorkspace = RecordingWorkspace


SessionsWorkspace = RecordingWorkspace
