"""Camera time synchronization controls and status."""

from __future__ import annotations

from typing import Callable
from PyQt5 import QtWidgets
from Workstation.workstation_core.theme import ERROR_TEXT
from Workstation.ui.designer import load_form
from Workstation.application.status_model import RuntimeStatus
from .shared import _compact_text, _evidence


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
        load_form("time_sync.ui", self)
        for widget in self.findChildren(QtWidgets.QWidget):
            legacy_name = widget.property("legacyObjectName")
            if legacy_name:
                widget.setObjectName(str(legacy_name))
        self.operation_buttons = {
            'time.set_pc': self.operation_buttons_time_set_pc,
            'time.check_sync': self.operation_buttons_time_check_sync,
            'time.apply_settings': self.operation_buttons_time_apply_settings,
            'time.save_settings': self.operation_buttons_time_save_settings,
        }
        self.settings_widgets = {
            'camera_sync_history_window_s': self.settings_widgets_camera_sync_history_window_s,
            'camera_sync_receive_pair_max_delta_ms': self.settings_widgets_camera_sync_receive_pair_max_delta_ms,
            'camera_sync_rtcp_pair_max_delta_ms': self.settings_widgets_camera_sync_rtcp_pair_max_delta_ms,
            'camera_sync_max_sr_age_s': self.settings_widgets_camera_sync_max_sr_age_s,
            'camera_sync_max_pc_offset_ms': self.settings_widgets_camera_sync_max_pc_offset_ms,
            'camera_sync_clock_rate_error_pct': self.settings_widgets_camera_sync_clock_rate_error_pct,
            'camera_sync_required_sr_count': self.settings_widgets_camera_sync_required_sr_count,
            'camera_sync_warmup_timeout_s': self.settings_widgets_camera_sync_warmup_timeout_s,
        }
        for command_id, button in self.operation_buttons.items():
            button.setEnabled(invoke_command is not None)
            if invoke_command is not None:
                button.clicked.connect(
                    lambda _checked=False, command=command_id: invoke_command(command)
                )
        self.sync_reason.setStyleSheet(f"color: {ERROR_TEXT};")
        self.sync_reason.hide()
        settings = _evidence(self.backend).measurement_settings()
        for key, widget in self.settings_widgets.items():
            value = settings.get(key)
            if value is not None:
                widget.setValue(int(value) if isinstance(widget, QtWidgets.QSpinBox) else float(value))

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
