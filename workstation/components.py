from __future__ import annotations

from PyQt5 import QtCore, QtGui, QtWidgets

from wiris_stereo_distance.theme import (
    ACCENT,
    ERROR,
    ERROR_BG,
    INFO_BG,
    PANEL_ALT_BG,
    PANEL_BORDER,
    SUCCESS,
    SUCCESS_BG,
    TEXT_MAIN,
    TEXT_MUTED,
    WARNING,
    WARNING_BG,
)

from .status_model import RuntimeStatus, StatusSeverity


def _level_color(level: StatusSeverity) -> tuple[str, str]:
    if level == StatusSeverity.OK:
        return SUCCESS, SUCCESS_BG
    if level == StatusSeverity.WARN:
        return WARNING, WARNING_BG
    if level == StatusSeverity.BAD:
        return ERROR, ERROR_BG
    if level == StatusSeverity.INFO:
        return ACCENT, INFO_BG
    return TEXT_MUTED, PANEL_ALT_BG


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
        show_source: bool = False,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("statusIndicator")
        self.compact = bool(compact)
        self.show_source = bool(show_source)
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setHorizontalSpacing(7)
        layout.setVerticalSpacing(0)
        self.dot = QtWidgets.QLabel("●")
        self.dot.setFixedWidth(14)
        self.label = QtWidgets.QLabel(label)
        self.label.setObjectName("statusIndicatorLabel")
        self.state = QtWidgets.QLabel("UNKNOWN")
        self.state.setObjectName("statusIndicatorState")
        self.detail = QtWidgets.QLabel("--")
        self.detail.setObjectName("statusIndicatorDetail")
        self.detail.setWordWrap(True)
        layout.addWidget(self.dot, 0, 0, 2, 1)
        layout.addWidget(self.label, 0, 1)
        layout.addWidget(self.state, 0, 2)
        layout.addWidget(self.detail, 1, 1, 1, 2)
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
            self.dot.setStyleSheet(f"color: {TEXT_MUTED};")
            self.setToolTip("No status evidence")
            return
        color, background = _level_color(status.severity)
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
        }
        self.state.setText(compact_state_names.get(status.state, status.state) if self.compact else status.state)
        age = ""
        if status.age_s is not None:
            age = f" · age {status.age_s:.2f}s"
        detail = status.detail or "--"
        self.detail.setText(detail)
        self.dot.setStyleSheet(f"color: {color};")
        self.setStyleSheet(
            f"QFrame#statusIndicator {{ background: {background}; border: 1px solid {PANEL_BORDER}; }}"
        )
        tooltip_lines = [f"{status.label}: {status.state}", detail]
        if self.show_source:
            tooltip_lines.append(f"source={status.source}")
        tooltip_lines.append(f"clock={status.clock}{age}")
        self.setToolTip("\n".join(tooltip_lines))


class EngineeringValue(QtWidgets.QWidget):
    def __init__(
        self,
        label: str,
        parent: QtWidgets.QWidget | None = None,
        *,
        show_source: bool = False,
        source_prefix: str = "source=",
    ) -> None:
        super().__init__(parent)
        self.show_source = bool(show_source)
        self.source_prefix = str(source_prefix)
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
        self.meta.setWordWrap(True)
        self.label.setMinimumWidth(72)
        self.label.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        self.value.setMinimumWidth(0)
        self.value.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        self.unit.setMinimumWidth(0)
        self.unit.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        self.meta.setMinimumWidth(0)
        self.meta.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        layout.addWidget(self.label, 0, 0)
        layout.addWidget(self.value, 0, 1)
        layout.addWidget(self.unit, 0, 2)
        layout.addWidget(self.meta, 1, 1, 1, 2)
        layout.setColumnStretch(1, 1)

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
        flags = []
        if frame:
            flags.append(f"frame={frame}")
        if source and self.show_source:
            flags.append(f"{self.source_prefix}{source}")
        if age_s is not None:
            flags.append(f"age={max(0.0, float(age_s)):.2f}s")
        if valid is not None:
            flags.append("VALID" if valid else "INVALID")
        self.meta.setText(" · ".join(flags))


class PropertySection(QtWidgets.QGroupBox):
    def __init__(self, title: str, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(title, parent)
        self.form = QtWidgets.QFormLayout(self)
        self.form.setContentsMargins(8, 8, 8, 8)
        self.form.setHorizontalSpacing(8)
        self.form.setVerticalSpacing(5)

    def add_row(self, label: str, widget: QtWidgets.QWidget) -> None:
        self.form.addRow(label, widget)


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
