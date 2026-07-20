from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from math import isfinite

from PyQt5 import QtCore, QtGui, QtWidgets

from wiris_stereo_distance.theme import (
    ACCENT,
    CHART_AXIS,
    CHART_BG,
    CHART_GRID,
    CHART_MUTED,
    CHART_TEXT,
    ERROR,
    PANEL_BORDER,
    SERIES_DISTANCE,
    SERIES_HIGHLIGHT,
    SERIES_LEFT,
    SERIES_RIGHT,
    SERIES_Z,
    SUCCESS,
    TEXT_MUTED,
    WARNING,
)


@dataclass(frozen=True)
class DistanceHistoryPoint:
    timestamp_s: float
    distance_m: float
    quality: str
    method: str
    right_m: float | None = None
    down_m: float | None = None
    forward_m: float | None = None
    live_truth_range_m: float | None = None
    live_range_error_m: float | None = None
    live_truth_timestamp_s: float | None = None
    alignment_profile_id: str | None = None
    class_name: str | None = None
    sample_key: tuple[float, float, str] | None = None
    recording_elapsed_s: float | None = None


class TriangulationQualityPlot(QtWidgets.QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(260)
        self.state: dict[str, object] = {}

    def set_quality(self, state: dict[str, object] | None) -> None:
        self.state = dict(state or {})
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(CHART_BG))
        rect = self.rect().adjusted(12, 12, -12, -12)
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_TEXT), 1))
        painter.drawText(rect.x(), rect.y() + 16, "측정 ray / gap")
        content = rect.adjusted(0, 30, 0, 0)
        if not self.state:
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(content, QtCore.Qt.AlignCenter, "3D ray triangulation 대기 중")
            return

        left_origin = QtCore.QPointF(content.left() + 34, content.bottom() - 42)
        right_origin = QtCore.QPointF(content.left() + 34, content.top() + 42)
        mid_x = content.left() + content.width() * 0.68
        left_end = QtCore.QPointF(mid_x, content.center().y() + 12)
        right_end = QtCore.QPointF(mid_x, content.center().y() - 12)
        midpoint = QtCore.QPointF(mid_x + 28, content.center().y())

        painter.setPen(QtGui.QPen(QtGui.QColor(ACCENT), 3))
        painter.drawLine(left_origin, left_end)
        painter.setPen(QtGui.QPen(QtGui.QColor(SERIES_Z), 3))
        painter.drawLine(right_origin, right_end)
        painter.setPen(QtGui.QPen(QtGui.QColor(SERIES_HIGHLIGHT), 2, QtCore.Qt.DashLine))
        painter.drawLine(left_end, right_end)
        painter.setBrush(QtGui.QColor(SERIES_HIGHLIGHT))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(midpoint, 5, 5)

        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(left_origin + QtCore.QPointF(-12, 22), "Left ray")
        painter.drawText(right_origin + QtCore.QPointF(-12, -10), "Right ray")
        painter.drawText(midpoint + QtCore.QPointF(10, 4), "midpoint")
        summary = (
            f"gap={self._fmt(self.state.get('closest_gap_m'))} m   "
            f"distance={self._fmt(self.state.get('distance_m'))} m   "
            f"{self.state.get('method', '')}"
        )
        painter.drawText(content.adjusted(0, 0, 0, -4), QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom, summary)

    def _fmt(self, value: object) -> str:
        try:
            return f"{float(value):.3f}"
        except (TypeError, ValueError):
            return "--"


class DistanceHistoryPlot(QtWidgets.QWidget):
    point_deleted = QtCore.pyqtSignal(object)
    SERIES = (
        ("right_m", "Right", "m", SERIES_LEFT),
        ("down_m", "Down", "m", SERIES_RIGHT),
        ("forward_m", "Forward", "m", SERIES_Z),
        ("distance_m", "Range", "m", SERIES_DISTANCE),
    )

    def __init__(self) -> None:
        super().__init__()
        self.points: deque[DistanceHistoryPoint] = deque(maxlen=600)
        self._selected_point_keys: set[tuple[float, float, str]] = set()
        self.empty_message = "그래프 미표시: 좌/우 YOLO 검출 쌍 대기 중"
        self.setMinimumHeight(260)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet(f"background: {CHART_BG}; border: 1px solid {PANEL_BORDER};")
        self.setMouseTracking(True)
        self.setToolTip("왼쪽 클릭: 점의 정보를 추가/해제합니다. 빈 곳 왼쪽 클릭: 선택 해제. 오른쪽 클릭: 해당 점 삭제.")

    def set_empty_message(self, message: str | None) -> None:
        cleaned = str(message or "").strip()
        self.empty_message = cleaned or "그래프 미표시: 좌/우 YOLO 검출 쌍 대기 중"
        if not self.points:
            self.update()

    def add_point(self, point: DistanceHistoryPoint) -> None:
        self.points.append(point)
        self.update()

    def clear(self) -> None:
        self.points.clear()
        self._selected_point_keys.clear()
        self.update()

    def _point_key(self, point: DistanceHistoryPoint) -> tuple[float, float, str]:
        return point.sample_key or (point.timestamp_s, point.distance_m, point.method)

    def _selected_points(self) -> list[DistanceHistoryPoint]:
        if not self._selected_point_keys:
            return []
        selected = [point for point in self.points if self._point_key(point) in self._selected_point_keys]
        self._selected_point_keys = {self._point_key(point) for point in selected}
        return selected

    def _selected_point_text(self) -> str | None:
        selected = self._selected_points()
        if not selected:
            return None
        lines = []
        for index, point in enumerate(selected, start=1):
            parts = [f"{index}. YOLO range: {point.distance_m:.2f} m"]
            if point.right_m is not None and point.down_m is not None and point.forward_m is not None:
                parts.append(
                    "Right/Down/Forward: "
                    f"{point.right_m:+.2f}, {point.down_m:+.2f}, {point.forward_m:+.2f} m"
                )
            if point.class_name:
                parts.append(f"클래스: {point.class_name}")
            if point.live_truth_range_m is not None:
                parts.append(f"Telemetry range: {point.live_truth_range_m:.2f} m")
            if point.live_range_error_m is not None:
                parts.append(f"YOLO-Telemetry error: {point.live_range_error_m:+.2f} m")
            if point.alignment_profile_id:
                parts.append(f"Origin profile: {point.alignment_profile_id}")
            if point.recording_elapsed_s is not None:
                parts.append(f"기록+{point.recording_elapsed_s:.2f}s")
            lines.append(" | ".join(parts))
        return "\n".join(lines)

    def _plot_rects(self) -> dict[str, QtCore.QRect]:
        content = self.rect().adjusted(48, 22, -18, -40)
        if content.width() <= 0 or content.height() <= 0:
            return {}
        gap = 10
        pane_w = max(1, (content.width() - gap) // 2)
        pane_h = max(1, (content.height() - gap) // 2)
        left = content.left()
        right = left + pane_w + gap
        top = content.top()
        bottom = top + pane_h + gap
        return {
            "right_m": QtCore.QRect(left, top, pane_w, pane_h),
            "down_m": QtCore.QRect(right, top, content.right() - right + 1, pane_h),
            "forward_m": QtCore.QRect(left, bottom, pane_w, content.bottom() - bottom + 1),
            "distance_m": QtCore.QRect(right, bottom, content.right() - right + 1, content.bottom() - bottom + 1),
        }

    def _point_value(self, point: DistanceHistoryPoint, field: str) -> float | None:
        value = getattr(point, field, None)
        if value is None:
            return None
        value = float(value)
        return value if isfinite(value) else None

    def _panel_series(self, field: str, title: str, unit: str, color: str) -> tuple[tuple[str, str, str, str], ...]:
        if field == "distance_m":
            return (
                ("distance_m", "YOLO Range", unit, SERIES_DISTANCE),
                ("live_truth_range_m", "Telemetry Range", unit, SERIES_HIGHLIGHT),
            )
        return ((field, title, unit, color),)

    def _plot_metrics(self, field: str = "distance_m") -> tuple[QtCore.QRect, float, float, float, float] | None:
        if not self.points:
            return None
        rect = self._plot_rects().get(field)
        if rect is None:
            return None
        if rect.width() <= 0 or rect.height() <= 0:
            return None
        series_fields = [series_field for series_field, _title, _unit, _color in self._panel_series(field, "", "", "")]
        values = [
            value
            for point in self.points
            for series_field in series_fields
            if (value := self._point_value(point, series_field)) is not None
        ]
        if not values:
            return None
        min_value = min(values)
        max_value = max(values)
        if max_value - min_value < 0.5:
            center = (max_value + min_value) / 2.0
            min_value = center - 0.25
            max_value = center + 0.25
            if field == "distance_m":
                min_value = max(0.0, min_value)
        start_s = self.points[0].timestamp_s
        end_s = max(self.points[-1].timestamp_s, start_s + 1.0)
        if end_s <= start_s:
            end_s = start_s + 1.0
        return rect, start_s, end_s, min_value, max_value

    def _map_point(
        self,
        point: DistanceHistoryPoint,
        rect: QtCore.QRect,
        start_s: float,
        end_s: float,
        min_value: float,
        max_value: float,
        field: str = "distance_m",
    ) -> QtCore.QPointF:
        value = self._point_value(point, field)
        if value is None:
            value = min_value
        x_ratio = (point.timestamp_s - start_s) / (end_s - start_s)
        y_ratio = (value - min_value) / (max_value - min_value)
        return QtCore.QPointF(rect.left() + x_ratio * rect.width(), rect.bottom() - y_ratio * rect.height())

    def _point_position(self, point: DistanceHistoryPoint) -> QtCore.QPointF | None:
        metrics = self._plot_metrics("distance_m")
        if metrics is None:
            return None
        rect, start_s, end_s, min_value, max_value = metrics
        return self._map_point(point, rect, start_s, end_s, min_value, max_value, "distance_m")

    def _nearest_point_index(self, pos: QtCore.QPointF, max_distance_px: float = 10.0) -> int | None:
        metrics = self._plot_metrics("distance_m")
        if metrics is None:
            return None
        rect, start_s, end_s, min_value, max_value = metrics
        best_index: int | None = None
        best_distance = float("inf")
        for index, point in enumerate(self.points):
            mapped = self._map_point(point, rect, start_s, end_s, min_value, max_value, "distance_m")
            distance = math.hypot(mapped.x() - pos.x(), mapped.y() - pos.y())
            if distance < best_distance:
                best_distance = distance
                best_index = index
        if best_index is None or best_distance > max_distance_px:
            return None
        return best_index

    def _remove_point_index(self, index: int) -> DistanceHistoryPoint:
        point = self.points[index]
        point_key = self._point_key(point)
        self.points = deque(
            (candidate for i, candidate in enumerate(self.points) if i != index),
            maxlen=self.points.maxlen,
        )
        self._selected_point_keys.discard(point_key)
        self.update()
        return point

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt override.
        if event.button() == QtCore.Qt.LeftButton:
            index = self._nearest_point_index(event.pos())
            if index is None:
                self._selected_point_keys.clear()
            else:
                point_key = self._point_key(self.points[index])
                if point_key in self._selected_point_keys:
                    self._selected_point_keys.remove(point_key)
                else:
                    self._selected_point_keys.add(point_key)
            self.update()
            event.accept()
            return
        if event.button() == QtCore.Qt.RightButton:
            index = self._nearest_point_index(event.pos())
            if index is not None:
                point = self._remove_point_index(index)
                self.point_deleted.emit(point)
                event.accept()
                return
        super().mousePressEvent(event)

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(CHART_BG))
        if not self.points:
            painter.setPen(QtGui.QColor(CHART_MUTED))
            text_rect = self.rect().adjusted(24, 24, -24, -24)
            painter.drawText(text_rect, QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap, self.empty_message)
            painter.end()
            return
        global_start_s = self.points[0].timestamp_s
        global_end_s = max(self.points[-1].timestamp_s, global_start_s + 1.0)
        for field, title, unit, color in self.SERIES:
            self._draw_series_panel(painter, field, title, unit, color)

        selected_points = self._selected_points()
        if selected_points:
            painter.setPen(QtGui.QPen(QtGui.QColor(SERIES_HIGHLIGHT), 2))
            painter.setBrush(QtCore.Qt.NoBrush)
            distance_metrics = self._plot_metrics("distance_m")
            for selected_point in selected_points:
                if distance_metrics is None:
                    continue
                rect, start_s, end_s, min_value, max_value = distance_metrics
                selected_mapped = self._map_point(
                    selected_point,
                    rect,
                    start_s,
                    end_s,
                    min_value,
                    max_value,
                    "distance_m",
                )
                painter.drawEllipse(selected_mapped, 7.0, 7.0)
            selected_text = self._selected_point_text() or ""
            rects = self._plot_rects()
            distance_rect = rects.get("distance_m", self.rect())
            if selected_text:
                text_metrics = painter.fontMetrics()
                selected_lines = selected_text.splitlines()
                text_width = max(text_metrics.horizontalAdvance(line) for line in selected_lines)
                text_height = text_metrics.lineSpacing() * len(selected_lines)
                box_width = min(max(220, text_width + 18), max(220, distance_rect.width() - 16))
                box_height = min(text_height + 10, max(32, distance_rect.height() - 16))
                label_rect = QtCore.QRect(distance_rect.right() - box_width, distance_rect.top() + 8, box_width, box_height)
                painter.fillRect(label_rect, QtGui.QColor(244, 244, 244, 235))
                painter.setPen(QtGui.QPen(QtGui.QColor(SERIES_HIGHLIGHT), 1))
                painter.drawRect(label_rect)
                painter.setPen(QtGui.QColor(CHART_TEXT))
                painter.drawText(label_rect.adjusted(8, 5, -8, -5), QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop, selected_text)

        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(10, self.height() - 12, f"{global_end_s - global_start_s:.1f}s window")
        painter.drawText(
            self.width() - 190,
            self.height() - 12,
            f"latest YOLO {self.points[-1].distance_m:.2f} m",
        )
        painter.end()

    def _draw_series_panel(self, painter: QtGui.QPainter, field: str, title: str, unit: str, color: str) -> None:
        rect = self._plot_rects().get(field)
        if rect is None:
            return
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)
        painter.setPen(QtGui.QColor(CHART_TEXT))
        painter.drawText(rect.left() + 8, rect.top() + 16, title)
        metrics = self._plot_metrics(field)
        if metrics is None:
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(rect.adjusted(8, 20, -8, -8), QtCore.Qt.AlignCenter, "3D 샘플 대기 중")
            return
        plot_rect, start_s, end_s, min_value, max_value = metrics
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1))
        for i in range(1, 4):
            y = plot_rect.top() + plot_rect.height() * i / 4
            painter.drawLine(plot_rect.left(), int(y), plot_rect.right(), int(y))
        panel_series = self._panel_series(field, title, unit, color)
        for series_field, _series_title, _series_unit, series_color in panel_series:
            series_points = [point for point in self.points if self._point_value(point, series_field) is not None]
            if len(series_points) >= 2:
                painter.save()
                painter.setClipRect(plot_rect)
                painter.setPen(QtGui.QPen(QtGui.QColor(series_color), 2))
                path = QtGui.QPainterPath(
                    self._map_point(series_points[0], plot_rect, start_s, end_s, min_value, max_value, series_field)
                )
                for point in series_points[1:]:
                    path.lineTo(self._map_point(point, plot_rect, start_s, end_s, min_value, max_value, series_field))
                painter.drawPath(path)
                painter.restore()
        quality_colors = {
            "GOOD": QtGui.QColor(SUCCESS),
            "DEGRADED": QtGui.QColor(WARNING),
            "BAD": QtGui.QColor(ERROR),
        }
        for series_field, _series_title, _series_unit, series_color in panel_series:
            series_points = [point for point in self.points if self._point_value(point, series_field) is not None]
            for point in series_points:
                mapped = self._map_point(point, plot_rect, start_s, end_s, min_value, max_value, series_field)
                painter.setPen(QtCore.Qt.NoPen)
                brush = quality_colors.get(point.quality, QtGui.QColor(TEXT_MUTED)) if series_field == field else QtGui.QColor(series_color)
                painter.setBrush(brush)
                painter.drawEllipse(mapped, 3.2, 3.2)
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(rect.left() + 6, plot_rect.top() + 12, f"{max_value:.2f} {unit}")
        painter.drawText(rect.left() + 6, plot_rect.bottom(), f"{min_value:.2f} {unit}")
        legend_x = rect.left() + 8
        legend_y = rect.bottom() - 6
        for series_field, series_title, series_unit, series_color in panel_series:
            series_points = [point for point in self.points if self._point_value(point, series_field) is not None]
            latest_value = self._point_value(series_points[-1], series_field) if series_points else None
            painter.setPen(QtGui.QColor(series_color))
            text = series_title if latest_value is None else f"{series_title} {latest_value:.2f} {series_unit}"
            painter.drawText(legend_x, legend_y, text)
            legend_x += max(110, painter.fontMetrics().horizontalAdvance(text) + 14)
        if field == "distance_m" and self.points[-1].live_range_error_m is not None:
            painter.setPen(QtGui.QColor(WARNING))
            painter.drawText(rect.right() - 130, rect.top() + 16, f"Err {self.points[-1].live_range_error_m:+.2f} m")
