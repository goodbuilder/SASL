"""Camera time synchronization controls and status."""

from __future__ import annotations

from typing import Callable
from PyQt5 import QtCore, QtWidgets
from Workstation.workstation_core.theme import ERROR_TEXT
from Workstation.ui.components import WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus
from .shared import _command_button, _compact_text, _evidence


class TimeWorkspace(QtWidgets.QWidget):
    """Field-facing WIRIS clock and RTCP/NTP verification surface.

    The page only exposes the existing clock workflow owner.  It does not
    create a second clock service or reinterpret timestamps; the labels mirror
    the legacy panel's source-backed status widgets.
    """

    def __init__(
        self,
        backend: object,
        invoke_command: Callable[[str], bool] | None = None,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.backend = backend
        self.invoke_command = invoke_command
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(
            WorkspaceHeader("Clock", "RTCP/NTP")
        )

        workflow_group = QtWidgets.QGroupBox("Clock")
        workflow_layout = QtWidgets.QHBoxLayout(workflow_group)
        workflow_layout.setContentsMargins(6, 6, 6, 6)
        workflow_layout.setSpacing(6)
        self.operation_buttons: dict[str, QtWidgets.QPushButton] = {}
        for command_id, label in (
            ("time.set_pc", "Set camera clock"),
            ("time.check_sync", "Check sync"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            workflow_layout.addWidget(button)
        workflow_layout.addStretch(1)
        layout.addWidget(workflow_group)

        sync_group = QtWidgets.QGroupBox("Sync status")
        sync_layout = QtWidgets.QVBoxLayout(sync_group)
        sync_status_row = QtWidgets.QHBoxLayout()
        self.sync_status = QtWidgets.QLabel("Waiting for RTCP/NTP")
        self.sync_status.setObjectName("statusStrip")
        self.sync_status.setWordWrap(True)
        self.sync_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.sync_reason = QtWidgets.QLabel("")
        self.sync_reason.setObjectName("statusReason")
        self.sync_reason.setWordWrap(True)
        self.sync_reason.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.sync_reason.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.sync_reason.setStyleSheet(f"color: {ERROR_TEXT};")
        self.sync_reason.hide()
        sync_status_row.addWidget(self.sync_status, 1)
        sync_status_row.addWidget(self.sync_reason, 1)
        sync_layout.addLayout(sync_status_row)
        self.schedule_status = QtWidgets.QLabel(
            "STUS schedule: MANUAL · manual seed required"
        )
        self.schedule_status.setObjectName("statusStrip")
        self.schedule_status.setWordWrap(True)
        self.schedule_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        sync_layout.addWidget(self.schedule_status)
        self.sync_summary = QtWidgets.QPlainTextEdit()
        self.sync_summary.setReadOnly(True)
        self.sync_summary.setMaximumBlockCount(80)
        # Tall enough to show recent RTCP/NTP sync lines without an inner
        # scrollbar; the workspace itself scrolls at the page level.
        self.sync_summary.setFixedHeight(200)
        sync_layout.addWidget(self.sync_summary)
        layout.addWidget(sync_group)

        settings = _evidence(self.backend).measurement_settings()
        self.settings_widgets: dict[str, QtWidgets.QWidget] = {}

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

        def int_spin(
            key: str,
            minimum: int,
            maximum: int,
            suffix: str = "",
        ) -> QtWidgets.QSpinBox:
            widget = QtWidgets.QSpinBox()
            widget.setRange(minimum, maximum)
            value = settings.get(key)
            widget.setValue(int(value if value is not None else minimum))
            widget.setSuffix(suffix)
            self.settings_widgets[key] = widget
            return widget

        limits_group = QtWidgets.QGroupBox("Sync limits")
        limits_form = QtWidgets.QGridLayout(limits_group)
        limits_form.setHorizontalSpacing(8)
        limits_form.setVerticalSpacing(5)
        limits_form.setColumnStretch(1, 1)
        limits_form.setColumnStretch(3, 1)
        limits_fields = (
            ("History", double_spin("camera_sync_history_window_s", 0.1, 5.0, 2, " s")),
            ("Receive skew", double_spin("camera_sync_receive_pair_max_delta_ms", 0.0, 2000.0, 1, " ms")),
            ("L/R RTCP Δt gate", double_spin("camera_sync_rtcp_pair_max_delta_ms", 0.0, 1000.0, 1, " ms")),
            ("SR stale", double_spin("camera_sync_max_sr_age_s", 0.1, 60.0, 2, " s")),
            ("SR-PC diagnostic", double_spin("camera_sync_max_pc_offset_ms", 0.0, 5000.0, 1, " ms")),
            ("Clock rate error", double_spin("camera_sync_clock_rate_error_pct", 0.0, 10.0, 3, " %")),
            ("Required SR", int_spin("camera_sync_required_sr_count", 1, 20, " count")),
            ("Warmup", double_spin("camera_sync_warmup_timeout_s", 1.0, 120.0, 1, " s")),
        )
        for index, (label, widget) in enumerate(limits_fields):
            row = index // 2
            column = (index % 2) * 2
            limits_form.addWidget(QtWidgets.QLabel(label), row, column)
            limits_form.addWidget(widget, row, column + 1)
        layout.addWidget(limits_group)

        self.sync_settings_status = QtWidgets.QLabel(
            "Sync limits apply now; Save keeps them for the next run."
        )
        self.sync_settings_status.setObjectName("statusStrip")
        self.sync_settings_status.setWordWrap(True)
        layout.addWidget(self.sync_settings_status)
        for command_id, label in (
            ("time.apply_settings", "Apply limits"),
            ("time.save_settings", "Save limits"),
        ):
            button = _command_button(label, command_id, invoke_command)
            self.operation_buttons[command_id] = button
            workflow_layout.addWidget(button)

        contract = QtWidgets.QLabel("RTCP/NTP timestamps")
        contract.setObjectName("compactContract")
        layout.addWidget(contract)
        layout.addStretch(1)

    @staticmethod
    def _widget_text(widget: object, default: str = "--") -> str:
        if widget is None:
            return default
        text = getattr(widget, "text", None)
        if callable(text):
            return str(text())
        plain_text = getattr(widget, "toPlainText", None)
        if callable(plain_text):
            return str(plain_text())
        return str(widget)

    def refresh(self, _snapshot: dict[str, RuntimeStatus]) -> None:
        panel = getattr(self.backend, "panel", self.backend)
        # The Workstation owns a separate visible Time workspace while the
        # source panel (and its legacy time tab) stays hidden.  The STUS
        # controller continues a low-rate health poll after initial VERIFIED;
        # this bounded 4 Hz workspace refresh mirrors that source-backed state
        # without creating a second timing owner.
        update_clock_sync = getattr(panel, "_update_clock_sync_display", None)
        if callable(update_clock_sync):
            update_clock_sync()
        sync_status = self._widget_text(
            getattr(panel, "clock_sync_status_label", None), "Waiting for RTCP/NTP"
        )
        self.sync_status.setText(_compact_text(sync_status))
        self.sync_status.setToolTip(sync_status)
        sync_state = str(getattr(panel, "camera_sync_status", "") or "").lower()
        if sync_state == "failed":
            reason = str(getattr(panel, "camera_sync_reason", "") or "reason unavailable")
            self.sync_reason.setText(f"Failed: {reason}")
            self.sync_reason.setToolTip(reason)
            self.sync_reason.show()
        else:
            self.sync_reason.clear()
            self.sync_reason.hide()
        summary = self._widget_text(getattr(panel, "clock_sync_summary_text", None), "")
        if summary != self.sync_summary.toPlainText():
            self.sync_summary.setPlainText(summary)
        schedule_reader = getattr(panel, "_stus_schedule_snapshot", None)
        schedule = schedule_reader() if callable(schedule_reader) else {}
        mode = str(schedule.get("mode") or "manual").upper()
        state = str(schedule.get("state") or "UNAVAILABLE")
        remaining = schedule.get("remaining_s")
        due_text = "--"
        try:
            remaining_s = max(0.0, float(remaining))
        except (TypeError, ValueError):
            pass
        else:
            due_text = f"{int(remaining_s) // 60:02d}:{int(remaining_s) % 60:02d}"
        frame_state = str(
            getattr(panel, "frame_timing_status", "not-verified")
        ).upper()
        unavailable = set(getattr(panel, "_camera_integrity_unavailable_sides", ()) or ())
        media_state = "UNAVAILABLE" if unavailable else ("LIVE" if frame_state == "CURRENT" else "STOPPED")
        self.schedule_status.setText(
            f"STUS schedule: {mode} · {state} · next {due_text} | "
            f"Clock {str(getattr(panel, 'camera_sync_status', 'unknown')).upper()} · "
            f"Frame timing {frame_state} · Media {media_state}"
        )

    def settings_payload(self) -> dict[str, object]:
        payload: dict[str, object] = {}
        for key, widget in self.settings_widgets.items():
            if isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox)):
                payload[key] = widget.value()
        return payload
