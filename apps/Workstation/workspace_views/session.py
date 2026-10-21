"""Measurement session lifecycle controls and feedback."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Callable
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.designer import load_form
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
        load_form("recording_session.ui", self)
        layout = self.pageLayout
        layout.setContentsMargins(*(0, 0, 0, 0) if embedded else (8, 8, 8, 8))
        if embedded:
            self.headerHost.hide()
        else:
            self.headerLayout.addWidget(WorkspaceHeader(
                "Session", "Create one timestamped folder, then keep this run's outputs together"
            ))
        self.start_session_button = self.sessionWorkspaceOpen
        self.close_session_button = self.sessionWorkspaceClose
        self.start_session_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.start_session_button.clicked.connect(self._begin_measurement_session)
        self.close_session_button.clicked.connect(self._close_measurement_session)
        self.session_status_label = self.sessionWorkspaceLifecycle
        self.folder_name_label = self.sessionWorkspaceFolderName
        self.folder_created_label = self.sessionWorkspaceCreated
        self.folder_path_label = self.sessionWorkspaceFolderPath
        for label in (self.folder_name_label, self.folder_created_label, self.folder_path_label):
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.storage_rule_label.setObjectName("compactContract")
        self.action_feedback_label = self.sessionWorkspaceFeedback
        self.action_feedback_label.setProperty("severity", "info")
        self.action_feedback_label.hide()
        self.open_measurement_button.setEnabled(self._can_open_measurement)
        self.open_review_button.setEnabled(self._can_open_review)
        if open_measurement is not None:
            self.open_measurement_button.clicked.connect(open_measurement)
        if open_review is not None:
            self.open_review_button.clicked.connect(open_review)
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
