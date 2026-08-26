from __future__ import annotations

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workstation_core.theme import (
    ACCENT,
    ERROR,
    PANEL_ALT_BG,
    PANEL_BORDER,
    SUCCESS,
    TEXT_MAIN,
    TEXT_MUTED,
    WARNING,
)

from .status_model import RuntimeStatus, StatusSeverity


def _level_color(level: StatusSeverity) -> str:
    if level == StatusSeverity.OK:
        return SUCCESS
    if level == StatusSeverity.WARN:
        return WARNING
    if level == StatusSeverity.BAD:
        return ERROR
    if level == StatusSeverity.INFO:
        return ACCENT
    return TEXT_MUTED


def _format_age(age_s: float) -> str:
    age_s = max(0.0, float(age_s))
    return f"{age_s * 1000.0:.0f} ms" if age_s < 1.0 else f"{age_s:.2f} s"


def _compact_status_detail(detail: object, *, limit: int = 96) -> str:
    """Keep workspace status cards readable while the tooltip keeps full evidence."""

    text = " ".join(str(detail or "").split())
    if len(text) <= limit:
        return text
    return text[: max(1, limit - 1)].rstrip() + "…"


class WorkspaceHeader(QtWidgets.QFrame):
    def __init__(self, title: str, subtitle: str = "", parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("workstationHeader")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(1)
        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setObjectName("workspaceTitle")
        self.subtitle_label = QtWidgets.QLabel(subtitle)
        self.subtitle_label.setObjectName("workspaceSubtitle")
        self.subtitle_label.setWordWrap(True)
        layout.addWidget(self.title_label)
        if subtitle:
            layout.addWidget(self.subtitle_label)


class StatusIndicator(QtWidgets.QFrame):
    def __init__(
        self,
        label: str,
        parent: QtWidgets.QWidget | None = None,
        *,
        compact: bool = False,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("statusIndicator")
        self.compact = bool(compact)
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setHorizontalSpacing(7)
        layout.setVerticalSpacing(0)
        self.label = QtWidgets.QLabel(label)
        self.label.setObjectName("statusIndicatorLabel")
        self.state = QtWidgets.QLabel("UNKNOWN")
        self.state.setObjectName("statusIndicatorState")
        self.age = QtWidgets.QLabel("")
        self.age.setObjectName("statusIndicatorAge")
        self.detail = QtWidgets.QLabel("--")
        self.detail.setObjectName("statusIndicatorDetail")
        self.detail.setWordWrap(True)
        layout.addWidget(self.label, 0, 0)
        layout.addWidget(self.state, 0, 1)
        layout.addWidget(self.age, 0, 2)
        layout.addWidget(self.detail, 1, 0, 1, 3)
        self.setStyleSheet(
            f"QFrame#statusIndicator {{ background: {PANEL_ALT_BG}; border: 1px solid {PANEL_BORDER}; }}"
        )
        if self.compact:
            self.detail.hide()
            self.setFixedHeight(22)
            self.setMinimumWidth(92)
            self.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
            layout.setContentsMargins(4, 0, 4, 0)
            layout.setHorizontalSpacing(4)
        self.set_status(None)

    def set_status(self, status: RuntimeStatus | None) -> None:
        if status is None:
            self.state.setText("UNKNOWN")
            self.detail.setText("no evidence")
            self.state.setStyleSheet(f"color: {TEXT_MUTED};")
            self.age.clear()
            self.setToolTip("No status evidence")
            return
        color = _level_color(status.severity)
        compact_state_names = {
            "STOPPED": "STOP",
            "DISCONNECTED": "DISC",
            "DISABLED": "OFF",
            "WAITING": "WAIT",
            "STARTING": "START",
            "CONNECTED": "CONN",
            "FINALIZING": "SAVE",
            "RECOVERABLE": "RETRY",
            "AVAILABLE": "READY",
            "CONNECTING": "CONN..",
            "CONNECTED_NO_DATA": "NO RTCM",
            "STREAMING": "STREAM",
            "RTCM_ACTIVE": "RTCM OK",
            "RTCM_STALE": "STALE",
            "RECONNECTING": "RETRY",
            "QUALIFYING": "QUAL",
            "ATTITUDE STALE": "ATT STALE",
            "ETH MAVLINK RX STALLED": "ETH STALL",
            "RECOVERING": "RECOVER",
        }
        age = "" if status.age_s is None else f" · age {status.age_s:.2f}s"
        self.state.setText(compact_state_names.get(status.state, status.state) if self.compact else status.state)
        self.age.setText("" if status.age_s is None else _format_age(status.age_s))
        detail = status.detail or "--"
        self.detail.setText(_compact_status_detail(detail))
        self.state.setStyleSheet(f"color: {color}; font-weight: 600;")
        self.age.setStyleSheet(f"color: {TEXT_MUTED};")
        if self.compact:
            self.setStyleSheet(
                f"QFrame#statusIndicator {{ background: {PANEL_ALT_BG}; border: 2px solid {color}; }}"
            )
        tooltip_lines = [f"{status.label}: {status.state}", detail]
        tooltip_lines.append(f"Clock: {status.clock}{age}")
        self.setToolTip("\n".join(tooltip_lines))


class EngineeringValue(QtWidgets.QWidget):
    activated = QtCore.pyqtSignal()

    def __init__(
        self,
        label: str,
        parent: QtWidgets.QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(6)
        self.label = QtWidgets.QLabel(label)
        self.label.setObjectName("engineeringLabel")
        self.value = QtWidgets.QLabel("--")
        self.value.setObjectName("engineeringValue")
        self.value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.value.setWordWrap(True)
        self.unit = QtWidgets.QLabel("")
        self.unit.setObjectName("engineeringUnit")
        self.meta = QtWidgets.QLabel("")
        self.meta.setObjectName("engineeringMeta")
        self.meta.setTextFormat(QtCore.Qt.RichText)
        self.meta.setWordWrap(True)
        self.meta.setVisible(False)
        self.label.setMinimumWidth(72)
        self.label.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        self.value.setMinimumWidth(0)
        self.value.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        self.unit.setMinimumWidth(0)
        self.unit.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        self.meta.setMinimumWidth(0)
        self.meta.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        for child in (self.label, self.value, self.unit, self.meta):
            child.installEventFilter(self)
        layout.addWidget(self.label, 0, 0)
        layout.addWidget(self.value, 0, 1)
        layout.addWidget(self.unit, 0, 2)
        layout.addWidget(self.meta, 1, 1, 1, 2)
        layout.setColumnStretch(1, 1)

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.LeftButton:
            self.activated.emit()
        super().mousePressEvent(event)

    def eventFilter(self, watched: QtCore.QObject, event: QtCore.QEvent) -> bool:
        if event.type() == QtCore.QEvent.MouseButtonPress and isinstance(event, QtGui.QMouseEvent):
            if event.button() == QtCore.Qt.LeftButton:
                self.activated.emit()
        return super().eventFilter(watched, event)

    def set_value(
        self,
        value: object,
        *,
        unit: str = "",
        source: str = "",
        age_s: float | None = None,
        frame: str = "",
        valid: bool | None = None,
    ) -> None:
        if value in (None, ""):
            text = "--"
        else:
            text = str(value)
        self.value.setText(text)
        self.unit.setText(unit)
        flags: list[str] = []
        if valid is not None:
            status_text = "VALID" if valid else "INVALID"
            status_color = SUCCESS if valid else ERROR
            flags.append(f'<span style="color:{status_color};">{status_text}</span>')
        if age_s is not None:
            flags.append(f'<span style="color:{TEXT_MUTED};">{_format_age(age_s)}</span>')
        self.meta.setText(" · ".join(flags))
        self.meta.setVisible(bool(flags))

        tooltip_lines = []
        if text != "--":
            tooltip_lines.append(f"Value: {text}{f' {unit}' if unit else ''}")
        if frame:
            tooltip_lines.append(f"Frame: {frame}")
        if source:
            tooltip_lines.append(f"From: {source}")
        if age_s is not None:
            tooltip_lines.append(f"Age: {max(0.0, float(age_s)):.2f}s")
        if valid is not None:
            tooltip_lines.append(f"Quality: {'valid' if valid else 'invalid'}")
        tooltip = "\n".join(tooltip_lines)
        self.setToolTip(tooltip)
        self.value.setToolTip(tooltip)


class PropertySection(QtWidgets.QGroupBox):
    def __init__(self, title: str, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(title, parent)
        self.form = QtWidgets.QFormLayout(self)
        self.form.setContentsMargins(8, 8, 8, 8)
        self.form.setHorizontalSpacing(8)
        self.form.setVerticalSpacing(5)

    def add_row(self, label: str, widget: QtWidgets.QWidget) -> None:
        label_widget = QtWidgets.QLabel(label)
        label_widget.setObjectName("engineeringLabel")
        label_widget.setWordWrap(True)
        self.form.addRow(label_widget, widget)


class EmptyStateView(QtWidgets.QFrame):
    def __init__(self, title: str, detail: str = "", parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(4)
        self.title = QtWidgets.QLabel(title)
        self.title.setObjectName("emptyStateTitle")
        self.detail = QtWidgets.QLabel(detail)
        self.detail.setObjectName("emptyStateDetail")
        self.detail.setWordWrap(True)
        layout.addStretch(1)
        layout.addWidget(self.title, alignment=QtCore.Qt.AlignCenter)
        layout.addWidget(self.detail, alignment=QtCore.Qt.AlignCenter)
        layout.addStretch(1)


class DiagnosticLogView(QtWidgets.QPlainTextEdit):
    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(2000)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.setPlaceholderText("No source-backed diagnostic log yet.")

    def set_source_text(self, text: str) -> None:
        if text != self.toPlainText():
            self.setPlainText(text)
            self.moveCursor(QtGui.QTextCursor.End)
