"""Measurement session lifecycle controls and feedback."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Callable
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus, StatusModel


class SessionWorkspace(QtWidgets.QWidget):
    """Create and present the timestamped folder that owns one work session."""

    inspection_requested = QtCore.pyqtSignal(object)

    def __init__(
        self,
        backend: object,
        status_model: StatusModel,
        open_measurement: Callable[[], None] | None = None,
        open_review: Callable[[], None] | None = None,
        parent: QtWidgets.QWidget | None = None,
        *,
        begin_session_action: Callable[[], object] | None = None,
        embedded: bool = False,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        self.begin_session_action = begin_session_action
        self._can_open_measurement = open_measurement is not None
        self._can_open_review = open_review is not None
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(*(0, 0, 0, 0) if embedded else (8, 8, 8, 8))
        layout.setSpacing(6)
        if not embedded:
            layout.addWidget(
                WorkspaceHeader(
                    "Session",
                    "Create one timestamped folder, then keep this run's outputs together",
                )
            )

        self.lifecycle_group = QtWidgets.QGroupBox("Session folder")
        lifecycle_layout = QtWidgets.QVBoxLayout(self.lifecycle_group)
        lifecycle_layout.setContentsMargins(10, 10, 10, 10)
        lifecycle_layout.setSpacing(8)

        action_row = QtWidgets.QHBoxLayout()
        action_row.setSpacing(8)
        self.start_session_button = QtWidgets.QPushButton("Create Session Folder")
        self.start_session_button.setObjectName("sessionWorkspaceOpen")
        self.start_session_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.start_session_button.setSizePolicy(
            QtWidgets.QSizePolicy.Maximum,
            QtWidgets.QSizePolicy.Fixed,
        )
        self.start_session_button.setToolTip(
            "Create recordings/YYYYMMDD_HHMMSS_nanoseconds and route subsequent outputs into it."
        )
        self.start_session_button.clicked.connect(self._begin_measurement_session)
        self.close_session_button = QtWidgets.QPushButton("Close Session")
        self.close_session_button.setObjectName("sessionWorkspaceClose")
        self.close_session_button.setSizePolicy(
            QtWidgets.QSizePolicy.Maximum,
            QtWidgets.QSizePolicy.Fixed,
        )
        self.close_session_button.setToolTip(
            "Stop active Detection, save and seal the current Measurement, then close the Session."
        )
        self.close_session_button.clicked.connect(self._close_measurement_session)
        action_row.addWidget(self.start_session_button)
        action_row.addWidget(self.close_session_button)
        action_row.addStretch(1)
        lifecycle_layout.addLayout(action_row)

        folder_form = QtWidgets.QFormLayout()
        folder_form.setContentsMargins(0, 0, 0, 0)
        folder_form.setHorizontalSpacing(12)
        folder_form.setVerticalSpacing(6)
        self.session_status_label = QtWidgets.QLabel("No session folder")
        self.session_status_label.setObjectName("sessionWorkspaceLifecycle")
        self.session_status_label.setWordWrap(True)
        self.folder_name_label = QtWidgets.QLabel("—")
        self.folder_name_label.setObjectName("sessionWorkspaceFolderName")
        self.folder_name_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.folder_name_label.setWordWrap(True)
        self.folder_created_label = QtWidgets.QLabel("—")
        self.folder_created_label.setObjectName("sessionWorkspaceCreated")
        self.folder_created_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.folder_path_label = QtWidgets.QLabel("—")
        self.folder_path_label.setObjectName("sessionWorkspaceFolderPath")
        self.folder_path_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.folder_path_label.setWordWrap(True)
        folder_form.addRow("State", self.session_status_label)
        folder_form.addRow("Folder name", self.folder_name_label)
        folder_form.addRow("Created", self.folder_created_label)
        folder_form.addRow("Storage path", self.folder_path_label)
        lifecycle_layout.addLayout(folder_form)

        self.storage_rule_label = QtWidgets.QLabel(
            "Outputs created after this folder opens are stored beneath it. "
            "Calibration, Alignment, Measurement, and Recording remain responsible for their own files."
        )
        self.storage_rule_label.setObjectName("compactContract")
        self.storage_rule_label.setWordWrap(True)
        lifecycle_layout.addWidget(self.storage_rule_label)
        self.action_feedback_label = QtWidgets.QLabel("")
        self.action_feedback_label.setObjectName("sessionWorkspaceFeedback")
        self.action_feedback_label.setProperty("severity", "info")
        self.action_feedback_label.setWordWrap(True)
        self.action_feedback_label.hide()
        lifecycle_layout.addWidget(self.action_feedback_label)
        layout.addWidget(self.lifecycle_group)

        self.navigation_group = QtWidgets.QGroupBox("Continue")
        navigation_row = QtWidgets.QHBoxLayout(self.navigation_group)
        navigation_row.setContentsMargins(10, 10, 10, 10)
        navigation_row.setSpacing(8)
        self.open_measurement_button = QtWidgets.QPushButton("Continue to Measurement")
        self.open_measurement_button.setEnabled(self._can_open_measurement)
        if open_measurement is not None:
            self.open_measurement_button.clicked.connect(open_measurement)
        self.open_review_button = QtWidgets.QPushButton("Review Stored Files")
        self.open_review_button.setEnabled(self._can_open_review)
        if open_review is not None:
            self.open_review_button.clicked.connect(open_review)
        navigation_row.addWidget(self.open_measurement_button)
        navigation_row.addWidget(self.open_review_button)
        navigation_row.addStretch(1)
        layout.addWidget(self.navigation_group)
        if not embedded:
            layout.addStretch(1)

        self._action_feedback = ""
        self.refresh({})

    def _set_action_feedback(self, text: str, *, severity: str = "error") -> None:
        self._action_feedback = str(text)
        self.action_feedback_label.setProperty("severity", str(severity))
        style = self.action_feedback_label.style()
        style.unpolish(self.action_feedback_label)
        style.polish(self.action_feedback_label)
        self._render_action_feedback()

    def _owner_method(self, *names: str) -> Callable[..., object] | None:
        panel = getattr(self.backend, "panel", self.backend)
        for owner in (self.backend, panel):
            for name in names:
                method = getattr(owner, name, None)
                if callable(method):
                    return method
        return None

    def _snapshot(self, *names: str) -> dict[str, object]:
        reader = self._owner_method(*names)
        if reader is None:
            return {}
        try:
            value = reader()
        except Exception as exc:
            return {"state": "ERROR", "error": str(exc)}
        return dict(value) if isinstance(value, Mapping) else {}

    def _run_action(
        self,
        names: tuple[str, ...],
        *,
        action: Callable[[], object] | None = None,
    ) -> None:
        method = action or self._owner_method(*names)
        if method is None:
            self._set_action_feedback("Session folder control is unavailable.")
            return
        try:
            result = method()
        except Exception as exc:
            self._set_action_feedback(f"Session folder action failed: {exc}")
            return
        if result is False:
            session = self._snapshot("measurement_session_snapshot")
            package = self._snapshot("measurement_package_snapshot")
            session_state = str(session.get("state") or "").upper()
            package_state = str(package.get("state") or "").upper()
            close_pending = (
                any(
                    name in {"close_measurement_session", "close_session"}
                    for name in names
                )
                and (
                    session_state == "CLOSING"
                    or package_state == "VERIFYING"
                    or bool(package.get("finalize_requested", False))
                )
            )
            if close_pending:
                self._set_action_feedback("Closing Session…", severity="info")
                self.refresh(self.status_model.snapshot())
                return
            reason = str(
                session.get("error")
                or package.get("error")
                or package.get("detail")
                or "The backend rejected the request."
            )
            self._set_action_feedback(f"Session folder was not changed: {reason}")
            self.refresh(self.status_model.snapshot())
            return
        self._set_action_feedback("", severity="info")
        self.refresh(self.status_model.snapshot())

    def _render_action_feedback(self) -> None:
        self.action_feedback_label.setText(self._action_feedback)
        self.action_feedback_label.setVisible(bool(self._action_feedback))

    def _begin_measurement_session(self) -> None:
        self._run_action(
            ("begin_measurement_session", "begin_session"),
            action=self.begin_session_action,
        )

    def _close_measurement_session(self) -> None:
        self._run_action(("close_measurement_session", "close_session"))

    def refresh(self, _runtime: dict[str, RuntimeStatus]) -> None:
        session = self._snapshot("measurement_session_snapshot")
        session_state = str(session.get("state") or "IDLE").upper()
        session_id = str(session.get("session_id") or "").strip()
        session_dir = str(session.get("session_dir") or "").strip()
        metadata = session.get("metadata")
        metadata = metadata if isinstance(metadata, Mapping) else {}
        opened_date = str(metadata.get("opened_local_date") or "").strip()
        opened_time = str(metadata.get("opened_local_time") or "").strip()
        created_text = " ".join(value for value in (opened_date, opened_time) if value)
        if not created_text and len(session_id) >= 15:
            date_part = session_id[:8]
            time_part = session_id[9:15]
            if date_part.isdigit() and time_part.isdigit():
                created_text = (
                    f"{date_part[:4]}-{date_part[4:6]}-{date_part[6:8]} "
                    f"{time_part[:2]}:{time_part[2:4]}:{time_part[4:6]}"
                )

        state_text = {
            "IDLE": "No folder created",
            "OPEN": "Ready for saving",
            "CLOSING": "Closing folder",
            "SEALED": "Closed",
            "RECOVERABLE": "Recovery required",
            "ERROR": "Folder status unavailable",
        }.get(session_state, session_state.replace("_", " ").title())
        self.session_status_label.setText(state_text)
        self.folder_name_label.setText(session_id or "—")
        self.folder_created_label.setText(created_text or "—")
        self.folder_path_label.setText(session_dir or "—")

        snapshot_error = str(session.get("error") or "").strip()
        current_measurement = session.get("current_measurement")
        current_measurement = (
            current_measurement if isinstance(current_measurement, Mapping) else {}
        )
        current_state = str(current_measurement.get("state") or "").upper()
        if session_state == "SEALED":
            self._set_action_feedback("", severity="info")
        elif (
            current_state == "RECOVERABLE"
            and self.action_feedback_label.property("severity") == "info"
            and self._action_feedback
        ):
            reason = str(
                current_measurement.get("error")
                or current_measurement.get("detail")
                or "Measurement recovery is required."
            )
            self._set_action_feedback(
                f"Session folder was not changed: {reason}",
                severity="error",
            )
        elif snapshot_error and not self._action_feedback:
            self._set_action_feedback(
                f"Session folder status error: {snapshot_error}",
                severity="error",
            )
        self._render_action_feedback()

        begin = self.begin_session_action or self._owner_method(
            "begin_measurement_session", "begin_session"
        )
        close = self._owner_method("close_measurement_session", "close_session")
        self.start_session_button.setEnabled(
            callable(begin) and session_state in {"IDLE", "SEALED"}
        )
        self.start_session_button.setText(
            "Create Another Session Folder" if session_state == "SEALED" else "Create Session Folder"
        )
        self.close_session_button.setEnabled(
            close is not None and session_state in {"OPEN", "CLOSING", "RECOVERABLE"}
        )
        self.open_measurement_button.setEnabled(
            self._can_open_measurement and session_state == "OPEN" and bool(session_id)
        )
        self.open_review_button.setEnabled(
            self._can_open_review and bool(session_id)
        )
        self.navigation_group.setVisible(
            bool(session_id) and (self._can_open_measurement or self._can_open_review)
        )
