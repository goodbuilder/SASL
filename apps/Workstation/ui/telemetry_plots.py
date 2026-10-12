"""Workstation-only telemetry plot widgets.

Input coordinate contracts are presentation-only and unchanged:
- GPS baseline values are local NED [North, East, Down] metres;
- camera translation values are P1 optical RDF [Right, Down, Forward] metres;
- attitude quaternions retain their producer contract.
This module performs no NED/FRD/camera-frame conversion.
"""

import math
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass
from math import isfinite

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workstation_core.attitude_geometry import relative_attitude
from Workstation.workstation_core.pixhawk_attitude import CameraExtrinsicState
from Workstation.workstation_core.rtk_geometry import RtkBaselineEstimate
from Workstation.workstation_core.theme import (
    CHART_AXIS,
    CHART_BG,
    CHART_GRID,
    CHART_MUTED,
    CHART_TEXT,
    ERROR,
    INVERSE_BG,
    INVERSE_TEXT,
    PANEL_BORDER,
    PANEL_BORDER_STRONG,
    SERIES_DISTANCE,
    SERIES_LEFT,
    SERIES_RIGHT,
    SERIES_Z,
    SUCCESS,
    WARNING,
)


@dataclass(frozen=True)
class AttitudeHistoryPoint:
    timestamp_s: float
    left_deg: float | None = None
    right_deg: float | None = None


class AttitudeHistoryPlot(QtWidgets.QWidget):
    def __init__(self, axis_name: str) -> None:
        super().__init__()
        self.axis_name = axis_name
        self.points: deque[AttitudeHistoryPoint] = deque(maxlen=600)
        self.setMinimumHeight(150)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet(f"background: {CHART_BG}; border: 1px solid {PANEL_BORDER};")

    def add_point(self, point: AttitudeHistoryPoint) -> None:
        self.points.append(point)
        self.update()

    @staticmethod
    def auto_y_bounds(values: list[float]) -> tuple[float, float]:
        min_value = min(values)
        max_value = max(values)
        if min_value == max_value:
            padding = max(0.0001, abs(min_value) * 0.05)
            return min_value - padding, max_value + padding
        return min_value, max_value

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(CHART_BG))
        rect = self.rect().adjusted(50, 18, -18, -34)
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)

        if len(self.points) < 2:
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, f"{self.axis_name}: waiting for Pixhawk samples")
            painter.end()
            return

        values = [
            value
            for point in self.points
            for value in (point.left_deg, point.right_deg)
            if value is not None and isfinite(value)
        ]
        if not values:
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, f"{self.axis_name}: waiting for Pixhawk samples")
            painter.end()
            return
        min_value, max_value = self.auto_y_bounds(values)
        start_s = self.points[0].timestamp_s
        end_s = max(self.points[-1].timestamp_s, start_s + 1.0)

        def map_xy(timestamp_s: float, value: float) -> QtCore.QPointF:
            x_ratio = (timestamp_s - start_s) / (end_s - start_s)
            y_ratio = (value - min_value) / (max_value - min_value)
            return QtCore.QPointF(rect.left() + x_ratio * rect.width(), rect.bottom() - y_ratio * rect.height())

        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1))
        mid_y = rect.center().y()
        painter.drawLine(rect.left(), mid_y, rect.right(), mid_y)
        for i in range(1, 4):
            y = rect.top() + rect.height() * i / 4
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))

        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left(), rect.top() - 4, self.axis_name)

        series = (
            ("Pixhawk 1", SERIES_LEFT, lambda point: point.left_deg),
            ("Pixhawk 2", SERIES_RIGHT, lambda point: point.right_deg),
        )
        painter.save()
        painter.setClipRect(rect)
        for name, color, getter in series:
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
            path = QtGui.QPainterPath()
            path_started = False
            for point in self.points:
                value = getter(point)
                if value is None or not isfinite(value):
                    path_started = False
                    continue
                mapped = map_xy(point.timestamp_s, value)
                if not path_started:
                    path.moveTo(mapped)
                    path_started = True
                else:
                    path.lineTo(mapped)
            painter.drawPath(path)
        painter.restore()

        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(8, rect.top() + 6, f"{max_value:.4f} deg")
        painter.drawText(8, rect.bottom(), f"{min_value:.4f} deg")
        legend_x = rect.left()
        for name, color, _getter in series:
            painter.setPen(QtGui.QColor(color))
            painter.drawText(legend_x, self.height() - 10, name)
            legend_x += 58
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.right() - 130, self.height() - 10, f"{end_s - start_s:.1f}s window")
        painter.end()


class Px4ShellDialog(QtWidgets.QDialog):
    def __init__(self, title: str, send_callback, mag_watch_callback=None) -> None:
        super().__init__()
        self.send_callback = send_callback
        self.setWindowTitle(title)
        self.resize(760, 420)
        layout = QtWidgets.QVBoxLayout(self)
        self.output = QtWidgets.QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setStyleSheet(
            f"background: {INVERSE_BG}; color: {INVERSE_TEXT}; font-family: monospace; "
            f"border: 1px solid {PANEL_BORDER_STRONG};"
        )
        command_row = QtWidgets.QHBoxLayout()
        self.command_edit = QtWidgets.QLineEdit()
        self.command_edit.setPlaceholderText("PX4 shell command")
        self.command_edit.returnPressed.connect(self.send_command)
        self.send_button = QtWidgets.QPushButton("Send")
        self.send_button.clicked.connect(self.send_command)
        if mag_watch_callback is not None:
            self.mag_watch_button = QtWidgets.QPushButton("Read MAG instances")
            self.mag_watch_button.setToolTip(
                "PX4 sensor_mag instance 0~4를 읽고 알려진 CAN device_id만 CAN 표에 반영합니다."
            )
            self.mag_watch_button.clicked.connect(mag_watch_callback)
            command_row.addWidget(self.mag_watch_button)
        command_row.addWidget(self.command_edit, 1)
        command_row.addWidget(self.send_button)
        layout.addWidget(self.output, 1)
        layout.addLayout(command_row)

    def append_output(self, text: str) -> None:
        self.output.moveCursor(QtGui.QTextCursor.End)
        self.output.insertPlainText(text)
        self.output.moveCursor(QtGui.QTextCursor.End)

    def send_command(self) -> None:
        command = self.command_edit.text().strip()
        if not command:
            return
        self.append_output(f"\n> {command}\n")
        self.command_edit.clear()
        self.send_callback(command)


@dataclass(frozen=True)
class TelemetryHistoryPoint:
    timestamp_s: float
    series: str
    value: float


class TelemetryHistoryPlot(QtWidgets.QWidget):
    def __init__(self, title: str, y_unit: str) -> None:
        super().__init__()
        self.title = title
        self.y_unit = y_unit
        self.points: deque[TelemetryHistoryPoint] = deque(maxlen=600)
        # Per-series subsequence of ``self.points``, kept incrementally in
        # sync (append/evict) so paintEvent -- which runs on every incoming
        # sample at 10+ Hz -- never has to rescan the whole deque to derive
        # series names or per-series point lists.
        self._series_cache: dict[str, deque[TelemetryHistoryPoint]] = {}
        self.series_labels: dict[str, str] = {
            "Δnorm": "Delta Norm",
            "angle": "Angle",
        }
        self.series_order: dict[str, int] = {
            "Δnorm": 50,
            "angle": 51,
        }
        self.setMouseTracking(True)
        self.setMinimumHeight(200)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet(f"background: {CHART_BG}; border: 1px solid {PANEL_BORDER};")

    def add_point(self, point: TelemetryHistoryPoint) -> None:
        evicted = None
        if self.points.maxlen is not None and len(self.points) >= self.points.maxlen:
            evicted = self.points[0]
        self.points.append(point)
        if evicted is not None:
            bucket = self._series_cache.get(evicted.series)
            if bucket and bucket[0] is evicted:
                bucket.popleft()
        self._series_cache.setdefault(
            point.series, deque(maxlen=self.points.maxlen)
        ).append(point)
        self.update()

    def set_series_labels(self, labels: dict[str, str]) -> None:
        self.series_labels.update(labels)
        self.update()

    def remove_series(self, series: str) -> None:
        self.points = deque((point for point in self.points if point.series != series), maxlen=self.points.maxlen)
        self._series_cache.pop(series, None)
        self.update()

    def _plot_rect(self) -> QtCore.QRect:
        return self.rect().adjusted(54, 24, -18, -34)

    def _series_names(self) -> list[str]:
        names = [name for name, bucket in self._series_cache.items() if bucket]
        return sorted(names, key=lambda name: (self.series_order.get(name, 1000), name))

    def _series_color_map(self) -> dict[str, str]:
        colors = [SERIES_LEFT, SERIES_RIGHT, SERIES_Z, SERIES_DISTANCE]
        return {
            series_name: colors[index % len(colors)]
            for index, series_name in enumerate(self._series_names())
        }

    def _series_label(self, series_name: str) -> str:
        return self.series_labels.get(series_name, series_name)

    def _value_bounds(self) -> tuple[float, float]:
        values = [point.value for point in self.points]
        min_value = min(values)
        max_value = max(values)
        if max_value - min_value < 1e-6:
            min_value -= 1.0
            max_value += 1.0
        return min_value, max_value

    def _time_bounds(self) -> tuple[float, float]:
        start_s = self.points[0].timestamp_s
        end_s = max(self.points[-1].timestamp_s, start_s + 1.0)
        return start_s, end_s

    def _map_point(
        self,
        point: TelemetryHistoryPoint,
        rect: QtCore.QRect,
        start_s: float,
        end_s: float,
        min_value: float,
        max_value: float,
    ) -> QtCore.QPointF:
        x_ratio = (point.timestamp_s - start_s) / (end_s - start_s)
        y_ratio = (point.value - min_value) / (max_value - min_value)
        return QtCore.QPointF(rect.left() + x_ratio * rect.width(), rect.bottom() - y_ratio * rect.height())

    def _format_hover_text(self, point: TelemetryHistoryPoint) -> str:
        return f"{self._series_label(point.series)}\nt={point.timestamp_s:.3f}s\nvalue={point.value:.3f} {self.y_unit}"

    def _nearest_point_at(self, position: QtCore.QPoint) -> TelemetryHistoryPoint | None:
        if len(self.points) < 2:
            return None
        rect = self._plot_rect()
        if not rect.adjusted(-8, -8, 8, 8).contains(position):
            return None
        min_value, max_value = self._value_bounds()
        start_s, end_s = self._time_bounds()
        best_point: TelemetryHistoryPoint | None = None
        best_distance_sq = 14.0 * 14.0
        for point in self.points:
            mapped = self._map_point(point, rect, start_s, end_s, min_value, max_value)
            dx = mapped.x() - position.x()
            dy = mapped.y() - position.y()
            distance_sq = dx * dx + dy * dy
            if distance_sq <= best_distance_sq:
                best_distance_sq = distance_sq
                best_point = point
        return best_point

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - Qt override.
        point = self._nearest_point_at(event.pos())
        if point is None:
            QtWidgets.QToolTip.hideText()
            return
        QtWidgets.QToolTip.showText(event.globalPos(), self._format_hover_text(point), self)

    def leaveEvent(self, event) -> None:  # noqa: N802 - Qt override.
        QtWidgets.QToolTip.hideText()
        super().leaveEvent(event)

    def _draw_legend(self, painter: QtGui.QPainter, rect: QtCore.QRect, series_colors: dict[str, str]) -> None:
        if not series_colors:
            return
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        metrics = QtGui.QFontMetrics(font)
        entries = [(self._series_label(series), color) for series, color in series_colors.items()]
        row_h = max(12, metrics.height())
        width = min(
            max(metrics.horizontalAdvance(label) for label, _color in entries) + 24,
            max(74, rect.width() // 2),
        )
        height = row_h * len(entries) + 8
        legend = QtCore.QRect(rect.right() - width - 6, rect.top() + 6, width, height)
        painter.fillRect(legend, QtGui.QColor(244, 244, 244, 220))
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(legend)
        y = legend.top() + 5 + metrics.ascent()
        for label, color in entries:
            painter.setPen(QtGui.QColor(color))
            painter.drawText(legend.left() + 7, y, label)
            y += row_h

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(CHART_BG))
        rect = self._plot_rect()
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(10, 18, self.title)

        if len(self.points) < 2:
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "Waiting for samples")
            painter.end()
            return

        min_value, max_value = self._value_bounds()
        start_s, end_s = self._time_bounds()

        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1))
        for i in range(1, 4):
            y = rect.top() + rect.height() * i / 4
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))

        series_colors = self._series_color_map()
        for series_name in self._series_names():
            series_points = list(self._series_cache.get(series_name, ()))
            if len(series_points) < 2:
                continue
            painter.setPen(QtGui.QPen(QtGui.QColor(series_colors[series_name]), 2))
            path = QtGui.QPainterPath(
                self._map_point(series_points[0], rect, start_s, end_s, min_value, max_value)
            )
            for point in series_points[1:]:
                path.lineTo(self._map_point(point, rect, start_s, end_s, min_value, max_value))
            painter.drawPath(path)

        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(8, rect.top() + 6, f"{max_value:.2f} {self.y_unit}")
        painter.drawText(8, rect.bottom(), f"{min_value:.2f} {self.y_unit}")
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(rect.left(), rect.bottom() + 18, f"t={start_s:.3f}s")
        painter.drawText(rect.right() - 86, rect.bottom() + 18, f"t={end_s:.3f}s")
        self._draw_legend(painter, rect, series_colors)
        painter.end()


class BaselineYawAlignmentPlot(QtWidgets.QWidget):
    """North-referenced top view for horizontal Baseline-P1/P2 angles."""

    HEIGHT_PX = 250

    def __init__(self) -> None:
        super().__init__()
        self.snapshot: dict[str, object] = {}
        self.setFixedHeight(self.HEIGHT_PX)
        self.setMinimumWidth(360)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.setStyleSheet(f"background: {CHART_BG}; border: 1px solid {PANEL_BORDER};")
        self.setToolTip(
            "Horizontal local NED: North=0 deg, clockwise positive.\n"
            "Baseline-Pn = wrap180(azimuth(P1->P2) - Pn yaw), Pn=P1/P2.\n"
            "Positive is right/clockwise of that Pixhawk Forward direction."
        )

    def set_snapshot(self, snapshot: Mapping[str, object] | None) -> None:
        values = dict(snapshot or {})
        if values != self.snapshot:
            self.snapshot = values
            self.update()

    @staticmethod
    def heading_endpoint(
        center: QtCore.QPointF,
        radius: float,
        heading_deg: float,
    ) -> QtCore.QPointF:
        angle_rad = math.radians(float(heading_deg))
        return QtCore.QPointF(
            center.x() + math.sin(angle_rad) * radius,
            center.y() - math.cos(angle_rad) * radius,
        )

    @staticmethod
    def _finite_angle(value: object) -> float | None:
        try:
            angle = float(value)
        except (TypeError, ValueError):
            return None
        return angle if math.isfinite(angle) else None

    @staticmethod
    def _series_pen(color: str, state: str, width: int) -> QtGui.QPen:
        series_color = QtGui.QColor(color)
        style = QtCore.Qt.SolidLine
        if state == "STALE":
            series_color.setAlpha(130)
            style = QtCore.Qt.DashLine
        return QtGui.QPen(series_color, width, style)

    def _draw_heading_arrow(
        self,
        painter: QtGui.QPainter,
        center: QtCore.QPointF,
        radius: float,
        heading_deg: float,
        *,
        color: str,
        state: str,
        label: str,
        width: int = 2,
        label_tangent_offset: float = 0.0,
    ) -> None:
        end = self.heading_endpoint(center, radius, heading_deg)
        pen = self._series_pen(color, state, width)
        painter.setPen(pen)
        painter.drawLine(center, end)
        line_angle = math.atan2(end.y() - center.y(), end.x() - center.x())
        head_length = 8.0
        for offset in (math.radians(150.0), -math.radians(150.0)):
            head = QtCore.QPointF(
                end.x() + math.cos(line_angle + offset) * head_length,
                end.y() + math.sin(line_angle + offset) * head_length,
            )
            painter.drawLine(end, head)
        painter.setPen(QtGui.QColor(pen.color()))
        unit_x = math.sin(math.radians(float(heading_deg)))
        unit_y = -math.cos(math.radians(float(heading_deg)))
        tangent_x, tangent_y = -unit_y, unit_x
        label_point = QtCore.QPointF(
            end.x() - unit_x * 12.0 + tangent_x * label_tangent_offset,
            end.y() - unit_y * 12.0 + tangent_y * label_tangent_offset,
        )
        painter.drawText(label_point, label)

    @staticmethod
    def _draw_legend_item(
        painter: QtGui.QPainter,
        x: int,
        y: int,
        *,
        color: str,
        label: str,
    ) -> int:
        painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
        painter.drawLine(x, y, x + 20, y)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(x + 25, y + 4, label)
        return x + 25 + painter.fontMetrics().horizontalAdvance(label) + 18

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(CHART_BG))
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(12, 20, "Horizontal top view · North=0° clockwise")
        state = str(self.snapshot.get("state") or "WAITING")
        state_width = painter.fontMetrics().horizontalAdvance(state)
        painter.drawText(self.width() - state_width - 12, 20, state)

        plot = self.rect().adjusted(38, 30, -38, -46)
        radius = max(20.0, min(plot.width(), plot.height()) * 0.43)
        center = QtCore.QPointF(plot.center())
        compass_rect = QtCore.QRectF(
            center.x() - radius,
            center.y() - radius,
            radius * 2.0,
            radius * 2.0,
        )
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawEllipse(compass_rect)
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1, QtCore.Qt.DashLine))
        painter.drawLine(
            QtCore.QPointF(center.x(), center.y() - radius),
            QtCore.QPointF(center.x(), center.y() + radius),
        )
        painter.drawLine(
            QtCore.QPointF(center.x() - radius, center.y()),
            QtCore.QPointF(center.x() + radius, center.y()),
        )
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(int(center.x() - 4), int(center.y() - radius - 5), "N")
        painter.drawText(int(center.x() + radius + 7), int(center.y() + 4), "E")
        painter.drawText(int(center.x() - 4), int(center.y() + radius + 15), "S")
        painter.drawText(int(center.x() - radius - 17), int(center.y() + 4), "W")

        baseline_azimuth = self._finite_angle(self.snapshot.get("baseline_azimuth_deg"))
        if baseline_azimuth is None:
            reason = str(self.snapshot.get("reason") or "P1→P2 horizontal baseline waiting")
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(
                plot,
                QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap,
                reason,
            )
        else:
            self._draw_heading_arrow(
                painter,
                center,
                radius,
                baseline_azimuth,
                color=SERIES_DISTANCE,
                state=str(self.snapshot.get("baseline_state") or "CURRENT"),
                label="B",
                width=3,
                label_tangent_offset=-9.0,
            )
            p1_yaw = self._finite_angle(self.snapshot.get("p1_yaw_deg"))
            if p1_yaw is not None:
                self._draw_heading_arrow(
                    painter,
                    center,
                    radius * 0.82,
                    p1_yaw,
                    color=SERIES_LEFT,
                    state=str(self.snapshot.get("p1_state") or "CURRENT"),
                    label="P1",
                    label_tangent_offset=7.0,
                )
            p2_yaw = self._finite_angle(self.snapshot.get("p2_yaw_deg"))
            if p2_yaw is not None:
                self._draw_heading_arrow(
                    painter,
                    center,
                    radius * 0.66,
                    p2_yaw,
                    color=SERIES_RIGHT,
                    state=str(self.snapshot.get("p2_state") or "CURRENT"),
                    label="P2",
                    label_tangent_offset=10.0,
                )

        legend_y = self.height() - 18
        legend_x = 12
        legend_x = self._draw_legend_item(
            painter,
            legend_x,
            legend_y,
            color=SERIES_DISTANCE,
            label="B P1→P2",
        )
        legend_x = self._draw_legend_item(
            painter,
            legend_x,
            legend_y,
            color=SERIES_LEFT,
            label="P1 Forward",
        )
        legend_x = self._draw_legend_item(
            painter,
            legend_x,
            legend_y,
            color=SERIES_RIGHT,
            label="P2 Forward",
        )
        painter.end()


class RtkAlignmentPlot(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.baseline: RtkBaselineEstimate | None = None
        self.camera_translation_m: tuple[float, float, float] | None = None
        self.left_attitude: CameraExtrinsicState | None = None
        self.right_attitude: CameraExtrinsicState | None = None
        self.setMinimumHeight(260)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet(f"background: {CHART_BG}; border: 1px solid {PANEL_BORDER};")

    def set_alignment(
        self,
        baseline: RtkBaselineEstimate | None,
        camera_translation_m: tuple[float, float, float] | None = None,
        left_attitude: CameraExtrinsicState | None = None,
        right_attitude: CameraExtrinsicState | None = None,
    ) -> None:
        self.baseline = baseline
        self.camera_translation_m = camera_translation_m
        self.left_attitude = left_attitude
        self.right_attitude = right_attitude
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(CHART_BG))
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(12, 22, "GPS baseline alignment")

        content = self.rect().adjusted(14, 34, -14, -18)
        if self.baseline is None:
            painter.drawText(content, QtCore.Qt.AlignCenter, "Waiting for GPS baseline")
            painter.end()
            return

        if self.camera_translation_m is None:
            self._draw_gps_only_panel(painter, content)
        else:
            gap = 12
            panel_width = int((content.width() - gap) * 0.62)
            alignment_rect = QtCore.QRect(content.left(), content.top(), panel_width, content.height())
            component_rect = QtCore.QRect(
                alignment_rect.right() + gap,
                content.top(),
                content.right() - alignment_rect.right() - gap,
                content.height(),
            )
            self._draw_forward_alignment_panel(painter, alignment_rect, self.camera_translation_m)
            self._draw_component_panel(painter, component_rect, self.camera_translation_m)
        painter.end()

    def _draw_forward_alignment_panel(
        self,
        painter: QtGui.QPainter,
        rect: QtCore.QRect,
        camera_translation_m: tuple[float, float, float],
    ) -> None:
        right_m, down_m, forward_m = camera_translation_m
        length_m = math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)
        if length_m <= 1e-9:
            self._draw_vector_panel(
                painter,
                rect,
                title="Top view: Forward vs Baseline",
                x_label="Right",
                y_label="Forward",
                x_m=0.0,
                y_m=0.0,
                footer="Waiting for baseline",
                color=SERIES_RIGHT,
            )
            return
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, forward_m / length_m))))
        effective_m = math.sqrt(right_m * right_m + down_m * down_m)
        level = self._angle_level(angle_deg)
        status_color = {"good": SUCCESS, "warn": WARNING, "bad": ERROR}[level]

        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left() + 8, rect.top() + 18, "Top view: P1 optical RDF Forward vs P1→P2 baseline")

        plot = rect.adjusted(42, 44, -42, -70)
        center = QtCore.QPointF(plot.center())
        scale_value = max(abs(right_m), abs(forward_m), 0.05)
        scale = 0.42 * min(plot.width(), plot.height()) / scale_value
        baseline_end = QtCore.QPointF(center.x() + right_m * scale, center.y() - forward_m * scale)
        forward_end = QtCore.QPointF(center.x(), center.y() - min(plot.height() * 0.36, 150.0))
        right_end = QtCore.QPointF(center.x() + min(plot.width() * 0.36, 170.0), center.y())

        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1, QtCore.Qt.DashLine))
        painter.drawLine(plot.left(), int(center.y()), plot.right(), int(center.y()))
        painter.drawLine(int(center.x()), plot.top(), int(center.x()), plot.bottom())

        target_rect = QtCore.QRectF(center.x() + 14, center.y() - 44, 40, 40)
        painter.setPen(QtGui.QPen(QtGui.QColor(SUCCESS), 2))
        painter.drawArc(target_rect, 180 * 16, 90 * 16)
        painter.drawText(int(center.x() + 58), int(center.y() - 30), "Target 90°")

        self._draw_arrow(painter, center, forward_end, QtGui.QColor(SERIES_Z))
        painter.setPen(QtGui.QColor(SERIES_Z))
        painter.drawText(forward_end + QtCore.QPointF(8, 4), "Forward +Z")

        self._draw_arrow(painter, center, right_end, QtGui.QColor(CHART_MUTED))
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(right_end + QtCore.QPointF(-48, 18), "Right")

        self._draw_arrow(painter, center, baseline_end, QtGui.QColor(SERIES_RIGHT))
        painter.setBrush(QtGui.QColor(SERIES_RIGHT))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(baseline_end, 5, 5)
        painter.setPen(QtGui.QColor(SERIES_RIGHT))
        painter.drawText(baseline_end + QtCore.QPointF(8, -8), "P1→P2 baseline")

        badge = QtCore.QRect(rect.left() + 8, rect.bottom() - 48, 118, 28)
        painter.fillRect(badge, QtGui.QColor(status_color))
        painter.setPen(QtGui.QColor(INVERSE_TEXT))
        painter.drawText(badge, QtCore.Qt.AlignCenter, level.upper())
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(
            badge.right() + 12,
            badge.top() + 19,
            f"angle={angle_deg:.2f}°  effective={effective_m:.3f}/{length_m:.3f} m  forward={forward_m:+.3f} m",
        )

    def _angle_level(self, angle_deg: float) -> str:
        if 80.0 <= angle_deg <= 100.0:
            return "good"
        if 70.0 <= angle_deg <= 110.0:
            return "warn"
        return "bad"

    def _draw_component_panel(
        self,
        painter: QtGui.QPainter,
        rect: QtCore.QRect,
        camera_translation_m: tuple[float, float, float],
    ) -> None:
        right_m, down_m, forward_m = camera_translation_m
        length_m = math.sqrt(right_m * right_m + down_m * down_m + forward_m * forward_m)
        effective_m = math.sqrt(right_m * right_m + down_m * down_m)
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left() + 8, rect.top() + 18, "Baseline components in P1 optical RDF")

        rows = (
            ("Right", right_m, SERIES_RIGHT),
            ("Down", down_m, SERIES_LEFT),
            ("Forward", forward_m, ERROR if abs(forward_m) > max(0.05, length_m * 0.15) else SUCCESS),
            ("Effective", effective_m, SERIES_DISTANCE),
        )
        max_value = max(*(abs(value) for _name, value, _color in rows), 0.05)
        bar_left = rect.left() + 88
        bar_right = rect.right() - 18
        zero_x = int((bar_left + bar_right) / 2)
        top = rect.top() + 52
        row_h = 38
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1))
        painter.drawLine(zero_x, top - 12, zero_x, top + row_h * len(rows) - 4)
        for index, (name, value, color) in enumerate(rows):
            y = top + index * row_h
            painter.setPen(QtGui.QColor(CHART_TEXT))
            painter.drawText(rect.left() + 10, y + 17, name)
            half_width = (bar_right - bar_left) * 0.46
            bar_len = int((abs(value) / max_value) * half_width)
            if value >= 0:
                bar_rect = QtCore.QRect(zero_x, y, bar_len, 20)
            else:
                bar_rect = QtCore.QRect(zero_x - bar_len, y, bar_len, 20)
            painter.fillRect(bar_rect, QtGui.QColor(color))
            painter.setPen(QtGui.QColor(CHART_TEXT))
            painter.drawText(bar_right - 86, y + 17, f"{value:+.3f} m")
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(
            rect.adjusted(10, 0, -10, -10),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom,
            "Preferred: Forward≈0 and Effective≈full baseline.",
        )

    def _format_relative_attitude_line(self) -> str:
        if self.left_attitude is None or self.right_attitude is None:
            return "Relative ΔR/P/Y: waiting"
        relative = relative_attitude(self.left_attitude.q, self.right_attitude.q)
        return f"Relative ΔR/P/Y: {relative.roll_deg:+.2f}° / {relative.pitch_deg:+.2f}° / {relative.yaw_deg:+.2f}°"

    def _attitude_arrow_components(self, state: CameraExtrinsicState) -> tuple[float, float, float]:
        yaw = max(-1.0, min(1.0, state.yaw_deg / 45.0))
        pitch = max(-1.0, min(1.0, state.pitch_deg / 45.0))
        return yaw, pitch, state.roll_deg

    def _draw_camera_attitude_arrows(self, painter: QtGui.QPainter, rect: QtCore.QRect) -> None:
        states = (
            ("L", self.left_attitude, SERIES_LEFT),
            ("R", self.right_attitude, SERIES_Z),
        )
        if all(state is None for _label, state, _color in states):
            return
        overlay_width = min(238, max(150, rect.width() - 24))
        overlay = QtCore.QRect(rect.right() - overlay_width - 10, rect.top() + 28, overlay_width, 92)
        painter.fillRect(overlay, QtGui.QColor(244, 244, 244, 232))
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(overlay)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(overlay.left() + 8, overlay.top() + 16, "Attitude vectors")
        for index, (label, state, color) in enumerate(states):
            origin = QtCore.QPointF(overlay.left() + 58 + index * 92, overlay.top() + 58)
            self._draw_single_attitude_glyph(painter, origin, label, state, QtGui.QColor(color))
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(overlay.left() + 8, overlay.bottom() - 8, "Optical axis: yaw/pitch; arc: roll")

    def _draw_single_attitude_glyph(
        self,
        painter: QtGui.QPainter,
        origin: QtCore.QPointF,
        label: str,
        state: CameraExtrinsicState | None,
        color: QtGui.QColor,
    ) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawLine(QtCore.QPointF(origin.x() - 24, origin.y()), QtCore.QPointF(origin.x() + 24, origin.y()))
        painter.drawLine(QtCore.QPointF(origin.x(), origin.y() - 30), QtCore.QPointF(origin.x(), origin.y() + 8))
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(int(origin.x() - 5), int(origin.y() + 24), label)
        if state is None:
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(int(origin.x() - 16), int(origin.y() - 10), "WAIT")
            return
        yaw, pitch, roll = self._attitude_arrow_components(state)
        end = QtCore.QPointF(origin.x() + yaw * 24.0, origin.y() - 28.0 - pitch * 18.0)
        self._draw_arrow(painter, origin, end, color)
        self._draw_roll_arc(painter, origin, roll, color)

    def _draw_arrow(self, painter: QtGui.QPainter, start: QtCore.QPointF, end: QtCore.QPointF, color: QtGui.QColor) -> None:
        painter.setPen(QtGui.QPen(color, 2))
        painter.drawLine(start, end)
        angle = math.atan2(end.y() - start.y(), end.x() - start.x())
        head_len = 7.0
        for offset in (math.radians(150), -math.radians(150)):
            point = QtCore.QPointF(
                end.x() + math.cos(angle + offset) * head_len,
                end.y() + math.sin(angle + offset) * head_len,
            )
            painter.drawLine(end, point)

    def _draw_roll_arc(self, painter: QtGui.QPainter, origin: QtCore.QPointF, roll_deg: float, color: QtGui.QColor) -> None:
        radius = 14
        rect = QtCore.QRectF(origin.x() - radius, origin.y() - radius, radius * 2, radius * 2)
        span_deg = max(-120.0, min(120.0, roll_deg))
        if abs(span_deg) < 0.5:
            span_deg = 18.0
        painter.setPen(QtGui.QPen(color, 1))
        painter.drawArc(rect, 35 * 16, int(span_deg * 16))

    def _draw_gps_only_panel(self, painter: QtGui.QPainter, rect: QtCore.QRect) -> None:
        if self.baseline is None:
            return
        up_m = -self.baseline.down_m
        self._draw_vector_panel(
            painter,
            rect,
            title="GPS-only E/U",
            x_label="E",
            y_label="U",
            x_m=self.baseline.east_m,
            y_m=up_m,
            footer=f"E={self.baseline.east_m:.3f} m, U={up_m:.3f} m, N={self.baseline.north_m:.3f} m",
            color=SERIES_RIGHT,
        )

    def _draw_vector_panel(
        self,
        painter: QtGui.QPainter,
        rect: QtCore.QRect,
        *,
        title: str,
        x_label: str,
        y_label: str,
        x_m: float,
        y_m: float,
        footer: str,
        color: str,
    ) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left() + 8, rect.top() + 18, title)

        plot = rect.adjusted(32, 34, -24, -42)
        center = QtCore.QPointF(plot.center())
        scale_value = max(abs(x_m), abs(y_m), 0.05)
        scale = 0.45 * min(plot.width(), plot.height()) / scale_value
        end = QtCore.QPointF(center.x() + x_m * scale, center.y() - y_m * scale)

        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1))
        painter.drawLine(plot.left(), int(center.y()), plot.right(), int(center.y()))
        painter.drawLine(int(center.x()), plot.top(), int(center.x()), plot.bottom())
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(plot.right() - 44, int(center.y()) - 6, x_label)
        painter.drawText(int(center.x()) + 6, plot.top() + 14, y_label)

        painter.setPen(QtGui.QPen(QtGui.QColor(color), 3))
        painter.drawLine(center, end)
        painter.setBrush(QtGui.QColor(color))
        painter.drawEllipse(end, 4, 4)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left() + 8, rect.bottom() - 18, footer)
