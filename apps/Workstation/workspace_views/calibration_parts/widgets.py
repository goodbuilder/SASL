"""Small calibration-specific display widgets."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Sequence
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.workstation_core.theme import SUCCESS_TEXT

class _ElidedStatusLabel(QtWidgets.QLabel):
    """One stable-height status line with full evidence in its tooltip."""

    def __init__(self, text: str = "", parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._full_text = ""
        self.setWordWrap(False)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.setFixedHeight(max(22, self.fontMetrics().lineSpacing() + 8))
        self.set_full_text(text)

    def set_full_text(self, text: object) -> None:
        self._full_text = " ".join(str(text or "").split())
        self.setToolTip(self._full_text)
        self._refresh_elision()

    def _refresh_elision(self) -> None:
        width = max(1, self.contentsRect().width())
        super().setText(
            self.fontMetrics().elidedText(
                self._full_text,
                QtCore.Qt.ElideRight,
                width,
            )
        )

    def resizeEvent(self, event: QtGui.QResizeEvent) -> None:  # noqa: N802
        super().resizeEvent(event)
        self._refresh_elision()


class _ContentSizedOperationTabs(QtWidgets.QTabWidget):
    """Size the operation strip from its active page, not the largest page."""

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.currentChanged.connect(self._refresh_geometry)

    def _refresh_geometry(self, _index: int = -1) -> None:
        self.updateGeometry()
        parent = self.parentWidget()
        while parent is not None:
            parent.updateGeometry()
            parent_layout = parent.layout()
            if parent_layout is not None:
                parent_layout.invalidate()
            parent = parent.parentWidget()

    def _active_page_height(self, *, minimum: bool) -> int:
        current = self.currentWidget()
        if current is None:
            return 0
        page_height = (
            current.minimumSizeHint().height()
            if minimum
            else current.sizeHint().height()
        )
        all_pages = [
            (
                self.widget(index).minimumSizeHint().height()
                if minimum
                else self.widget(index).sizeHint().height()
            )
            for index in range(self.count())
        ]
        base = (
            super().minimumSizeHint()
            if minimum
            else super().sizeHint()
        )
        largest_page = max(all_pages, default=0)
        tab_overhead = max(0, base.height() - largest_page)
        return page_height + tab_overhead

    def sizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt API.
        hint = super().sizeHint()
        active_height = self._active_page_height(minimum=False)
        if active_height > 0:
            hint.setHeight(active_height)
        return hint

    def minimumSizeHint(self) -> QtCore.QSize:  # noqa: N802 - Qt API.
        hint = super().minimumSizeHint()
        active_height = self._active_page_height(minimum=True)
        if active_height > 0:
            hint.setHeight(active_height)
        return hint


class CalibrationErrorPlot(QtWidgets.QWidget):
    """Compact source-backed reprojection plot with exclusion actions."""

    exclude_requested = QtCore.pyqtSignal(object)
    restore_requested = QtCore.pyqtSignal(object)

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._diagnostics: tuple[dict[str, object], ...] = ()
        self._excluded_paths: set[str] = set()
        self._excluded_pair_keys: set[str] = set()
        self._point_positions: list[tuple[QtCore.QPointF, dict[str, object]]] = []
        self._threshold_px: float | None = None
        self.setMinimumHeight(150)

    @staticmethod
    def _finite_error(row: Mapping[str, object]) -> float | None:
        try:
            value = float(row.get("reprojection_error_px"))
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value >= 0.0 else None

    def set_diagnostics(
        self,
        rows: Sequence[Mapping[str, object]],
        *,
        excluded_paths: set[str] | None = None,
        excluded_pair_keys: set[str] | None = None,
    ) -> None:
        self._diagnostics = tuple(dict(row) for row in rows)
        self._excluded_paths = set(excluded_paths or ())
        self._excluded_pair_keys = set(excluded_pair_keys or ())
        errors = sorted(
            value
            for row in self._diagnostics
            if (value := self._finite_error(row)) is not None
        )
        if errors:
            median = errors[len(errors) // 2]
            self._threshold_px = max(1.0, median * 2.0)
        else:
            self._threshold_px = None
        self.update()

    def _layout_points(self) -> None:
        self._point_positions = []
        rows = [
            row for row in self._diagnostics if self._finite_error(row) is not None
        ]
        if not rows:
            return
        left, top, right, bottom = 38.0, 18.0, 18.0, 26.0
        width = max(1.0, self.width() - left - right)
        height = max(1.0, self.height() - top - bottom)
        maximum = max(self._finite_error(row) or 0.0 for row in rows)
        maximum = max(maximum, self._threshold_px or 0.0, 1.0)
        for index, row in enumerate(rows):
            x = left + width * (index + 0.5) / len(rows)
            y = top + height * (1.0 - (self._finite_error(row) or 0.0) / maximum)
            self._point_positions.append((QtCore.QPointF(x, y), row))

    def paintEvent(self, _event: QtGui.QPaintEvent) -> None:  # noqa: N802
        self._layout_points()
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QtGui.QColor("#ffffff"))
        painter.setPen(QtGui.QPen(QtGui.QColor("#c6c6c6"), 1))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))
        for point, row in self._point_positions:
            excluded = (
                str(row.get("path") or "") in self._excluded_paths
                or str(row.get("capture_key") or "") in self._excluded_pair_keys
            )
            error = self._finite_error(row)
            candidate = bool(
                self._threshold_px is not None
                and error is not None
                and error >= self._threshold_px
            )
            color = "#888888" if excluded else "#c0392b" if candidate else SUCCESS_TEXT
            painter.setBrush(QtGui.QColor(color))
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 1))
            painter.drawEllipse(point, 5.0, 5.0)

    def contextMenuEvent(self, event: QtGui.QContextMenuEvent) -> None:  # noqa: N802
        self._layout_points()
        if not self._point_positions:
            return
        target = min(
            self._point_positions,
            key=lambda item: QtCore.QLineF(item[0], event.pos()).length(),
        )
        point, row = target
        if QtCore.QLineF(point, event.pos()).length() > 20.0:
            return
        path = str(row.get("path") or "")
        capture_key = str(row.get("capture_key") or "")
        side = str(row.get("side") or "")
        image_excluded = path in self._excluded_paths
        pair_excluded = capture_key in self._excluded_pair_keys
        menu = QtWidgets.QMenu(self)
        image_action = menu.addAction(
            "Restore Intrinsic image" if image_excluded else "Exclude Intrinsic image"
        )
        pair_action = menu.addAction(
            "Restore Stereo pair" if pair_excluded else "Exclude Stereo pair"
        )
        chosen = menu.exec_(event.globalPos())
        if chosen is None:
            return
        if chosen is image_action:
            signal = self.restore_requested if image_excluded else self.exclude_requested
            signal.emit(
                {
                    "scope": "image",
                    "side": side,
                    "path": path,
                    "capture_key": capture_key,
                }
            )
        elif chosen is pair_action:
            signal = self.restore_requested if pair_excluded else self.exclude_requested
            signal.emit(
                {
                    "scope": "pair",
                    "side": side,
                    "path": path,
                    "capture_key": capture_key,
                }
            )


