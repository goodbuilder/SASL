from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from math import isfinite

from PyQt5 import QtCore, QtGui, QtWidgets

from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME, RayTriangulationResult
from Workstation.workstation_core.theme import (
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
    """One public point in ``baseline_center_frd``.

    Coordinate fields are ordered and named ``Forward/Right/Down`` in metres;
    ``distance_m`` is the Euclidean range from the stereo-baseline midpoint.
    """

    timestamp_s: float
    distance_m: float
    quality: str
    method: str
    forward_m: float | None = None
    right_m: float | None = None
    down_m: float | None = None
    class_name: str | None = None
    sample_key: tuple[object, ...] | None = None
    recording_elapsed_s: float | None = None
    # WGS84 coordinates from this same measurement, separate from FRD metres.
    target_latitude_deg: float | None = None
    target_longitude_deg: float | None = None
    target_altitude_ellipsoid_m: float | None = None
    target_altitude_msl_m: float | None = None


def triangulation_3d_scene_points(
    triangulated: RayTriangulationResult | None,
    *,
    baseline_m: float | None,
) -> dict[str, tuple[float, float, float]] | None:
    """Camera and ray points for the 3D quality view, in baseline_center_frd metres.

    Returns ``None`` when the result isn't expressed in ``baseline_center_frd``
    (the only frame whose Right axis this function uses to place CAM_L/CAM_R) or
    when any required point is missing, so callers can fall back to a
    placeholder rather than plot a point in the wrong frame.
    """

    if triangulated is None or baseline_m is None or not isfinite(float(baseline_m)):
        return None
    if triangulated.output_frame != BASELINE_CENTER_FRD_FRAME:
        return None
    midpoint = triangulated.midpoint_m
    point_on_left = triangulated.point_on_left_ray_m
    point_on_right = triangulated.point_on_right_ray_m
    detected_point = triangulated.point_m
    if midpoint is None or point_on_left is None or point_on_right is None or detected_point is None:
        return None
    half_baseline_m = float(baseline_m) / 2.0
    return {
        "cam_l": (0.0, -half_baseline_m, 0.0),
        "cam_r": (0.0, half_baseline_m, 0.0),
        "point_on_left_ray": tuple(point_on_left),
        "point_on_right_ray": tuple(point_on_right),
        "midpoint": tuple(midpoint),
        "detected_point": tuple(detected_point),
    }


def _frd_to_gl(point: tuple[float, float, float]) -> tuple[float, float, float]:
    """Map (Forward, Right, Down) metres to a GL (X=Right, Y=Forward, Z=Up) scene."""

    forward_m, right_m, down_m = point
    return (right_m, forward_m, -down_m)


class Triangulation3DView(QtWidgets.QWidget):
    """Rotatable 3D view of CAM_L/CAM_R and the triangulated ray geometry.

    Falls back to a static placeholder label if pyqtgraph's OpenGL view can't
    be constructed (missing PyOpenGL, no usable GL context) so a broken driver
    can't crash the rest of the GUI.
    """

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(260)
        self.state: dict[str, object] = {}
        self._is_tab_visible = False
        self._dirty = False
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._summary_label = QtWidgets.QLabel("측정 ray / gap")
        self._summary_label.setStyleSheet(f"color: {CHART_TEXT}; background: {CHART_BG}; padding: 4px 6px;")
        layout.addWidget(self._summary_label)
        self._placeholder_label = QtWidgets.QLabel("3D ray triangulation 대기 중")
        self._placeholder_label.setAlignment(QtCore.Qt.AlignCenter)
        self._placeholder_label.setStyleSheet(f"color: {CHART_MUTED}; background: {CHART_BG};")
        layout.addWidget(self._placeholder_label, 1)
        self._gl_view = self._build_gl_view()
        if self._gl_view is not None:
            layout.addWidget(self._gl_view, 1)
            self._gl_view.hide()

    def _build_gl_view(self):
        try:
            import numpy as np
            import pyqtgraph.opengl as gl
        except Exception:
            return None
        try:
            view = gl.GLViewWidget()
            view.setBackgroundColor(QtGui.QColor(CHART_BG))
            view.setCameraPosition(distance=6.0, elevation=20.0, azimuth=-60.0)
            grid = gl.GLGridItem()
            grid.setSize(10, 10)
            grid.setSpacing(1, 1)
            view.addItem(grid)
            self._cam_scatter = gl.GLScatterPlotItem(size=14.0)
            self._point_scatter = gl.GLScatterPlotItem(size=16.0)
            self._left_ray = gl.GLLinePlotItem(width=2.0, antialias=True)
            self._right_ray = gl.GLLinePlotItem(width=2.0, antialias=True)
            self._gap_line = gl.GLLinePlotItem(width=1.0, antialias=True)
            for item in (self._left_ray, self._right_ray, self._gap_line, self._cam_scatter, self._point_scatter):
                view.addItem(item)
            self._np = np
        except Exception:
            return None
        return view

    def set_tab_visible(self, visible: bool) -> None:
        self._is_tab_visible = visible
        if visible and self._dirty:
            self._repaint()

    def set_quality(self, state: dict[str, object] | None) -> None:
        self.state = dict(state or {})
        summary = (
            f"gap={self._fmt(self.state.get('closest_gap_m'))} m   "
            f"distance={self._fmt(self.state.get('distance_m'))} m   "
            f"{self.state.get('method', '')}"
        )
        self._summary_label.setText(summary if self.state else "측정 ray / gap")
        has_scene = bool(self.state.get("scene")) and self._gl_view is not None
        self._placeholder_label.setVisible(not has_scene)
        if self._gl_view is not None:
            self._gl_view.setVisible(has_scene)
        self._dirty = True
        if self._is_tab_visible:
            self._repaint()

    def _repaint(self) -> None:
        self._dirty = False
        scene = self.state.get("scene")
        if not scene or self._gl_view is None:
            return
        np = self._np
        cam_l = np.array(_frd_to_gl(scene["cam_l"]))
        cam_r = np.array(_frd_to_gl(scene["cam_r"]))
        left_pt = np.array(_frd_to_gl(scene["point_on_left_ray"]))
        right_pt = np.array(_frd_to_gl(scene["point_on_right_ray"]))
        detected_pt = np.array(_frd_to_gl(scene["detected_point"]))
        self._cam_scatter.setData(
            pos=np.array([cam_l, cam_r]),
            color=np.array([[0.20, 0.20, 0.20, 1.0], [0.62, 0.62, 0.62, 1.0]]),
        )
        self._point_scatter.setData(pos=np.array([detected_pt]), color=np.array([[0.80, 0.12, 0.16, 1.0]]))
        self._left_ray.setData(pos=np.array([cam_l, left_pt]), color=(0.24, 0.24, 0.24, 1.0))
        self._right_ray.setData(pos=np.array([cam_r, right_pt]), color=(0.62, 0.62, 0.62, 1.0))
        self._gap_line.setData(pos=np.array([left_pt, right_pt]), color=(1.0, 1.0, 1.0, 0.5))

    def _fmt(self, value: object) -> str:
        try:
            return f"{float(value):.3f}"
        except (TypeError, ValueError):
            return "--"


class DistanceHistoryPlot(QtWidgets.QWidget):
    point_deleted = QtCore.pyqtSignal(object)
    SERIES = (
        ("forward_m", "Forward", "m", SERIES_LEFT),
        ("right_m", "Right", "m", SERIES_RIGHT),
        ("down_m", "Down", "m", SERIES_Z),
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
            if point.forward_m is not None and point.right_m is not None and point.down_m is not None:
                parts.append(
                    "baseline_center_frd Forward/Right/Down: "
                    f"{point.forward_m:+.2f}, {point.right_m:+.2f}, {point.down_m:+.2f} m"
                )
            if point.class_name:
                parts.append(f"클래스: {point.class_name}")
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
            "forward_m": QtCore.QRect(left, top, pane_w, pane_h),
            "right_m": QtCore.QRect(right, top, content.right() - right + 1, pane_h),
            "down_m": QtCore.QRect(left, bottom, pane_w, content.bottom() - bottom + 1),
            "distance_m": QtCore.QRect(right, bottom, content.right() - right + 1, content.bottom() - bottom + 1),
        }

    def _point_value(self, point: DistanceHistoryPoint, field: str) -> float | None:
        value = getattr(point, field, None)
        if value is None:
            return None
        value = float(value)
        return value if isfinite(value) else None

    def _panel_series(self, field: str, title: str, unit: str, color: str) -> tuple[tuple[str, str, str, str], ...]:
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


@dataclass(frozen=True)
class ClockOffsetPoint:
    """One sampled RTCP timing state for both cameras.

    ``transit_delay_ms`` is ``rtcp_frame_receive_delay_s``, which carries the
    camera's absolute clock error and so grows as that clock drifts.
    ``transport_delay_ms`` is ``rtcp_frame_transport_delay_s``, the same figure
    with the Sender Report's own PC offset added back so the clock term
    cancels — that one should stay flat.
    """

    timestamp_s: float
    left_transit_delay_ms: float | None = None
    right_transit_delay_ms: float | None = None
    left_transport_delay_ms: float | None = None
    right_transport_delay_ms: float | None = None
    left_sr_age_s: float | None = None
    right_sr_age_s: float | None = None


class ClockOffsetHistoryPlot(QtWidgets.QWidget):
    """Two stacked panels: RTCP transit delay, and Sender Report anchor age.

    They are separated because a growing transit delay caused by a frozen SR
    anchor looks identical to real clock drift until the anchor age is read
    next to it.
    """

    PANELS = (
        (
            "transit delay (카메라 클럭 오차 포함)",
            "ms",
            (("left_transit_delay_ms", "L", SERIES_LEFT), ("right_transit_delay_ms", "R", SERIES_RIGHT)),
        ),
        (
            "transport delay (클럭 보정됨)",
            "ms",
            (
                ("left_transport_delay_ms", "L", SERIES_LEFT),
                ("right_transport_delay_ms", "R", SERIES_RIGHT),
            ),
        ),
        (
            "Sender Report anchor age",
            "s",
            (("left_sr_age_s", "L", SERIES_LEFT), ("right_sr_age_s", "R", SERIES_RIGHT)),
        ),
    )

    def __init__(self) -> None:
        super().__init__()
        self.points: deque[ClockOffsetPoint] = deque(maxlen=900)
        self.setMinimumHeight(220)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet(f"background: {CHART_BG}; border: 1px solid {PANEL_BORDER};")
        self.setMinimumHeight(300)
        self.setToolTip(
            "시간축: 오른쪽 0 s가 최신 샘플이고 왼쪽의 음수 값은 몇 초 전인지 나타냅니다.\n"
            "L/R: 왼쪽/오른쪽 카메라의 같은 시계열 진단입니다. 값이 없으면 해당 시점에 유효한 RTCP/NTP가 없었다는 뜻입니다.\n"
            "transit delay (ms): rtcp_frame_receive_delay_s입니다. RTCP/NTP로 추정한 frame 시각부터 PC 수신까지의 차이로, "
            "카메라 clock offset과 encoder/network/decoder transit이 함께 들어갑니다. 순수한 네트워크 지연이 아니며, "
            "카메라 clock이 틀리거나 drift하면 커질 수 있습니다.\n"
            "transport delay (ms): rtcp_frame_transport_delay_s입니다. Sender Report의 PC 기준 offset을 상쇄해 "
            "실제 encoder/network/decoder 경로에 가까운 지연만 남긴 값입니다. clock drift와 무관하게 평평해야 하며, "
            "갑자기 증가하면 수신 경로 또는 encoder 지연을 의심합니다.\n"
            "Sender Report anchor age (s): 마지막 RTCP Sender Report를 받은 뒤 지난 시간입니다. 계속 증가하면 anchor가 "
            "갱신되지 않아 transit/transport 값이 오래된 anchor를 기준으로 외삽되고 있다는 뜻입니다.\n"
            "주의: 이 그래프의 transit delay는 YOLO 설정의 `RTP 지연(ms)`(GStreamer jitter buffer 대기시간)와 다릅니다. "
            "또한 좌우 RTCP/NTP Δt 자체를 그리는 그래프가 아니라 카메라별 PC 수신/transport 진단입니다. STUS 후 stream을 재시작하면 "
            "새 RTCP anchor로 그래프가 초기화됩니다."
        )

    def add_point(self, point: ClockOffsetPoint) -> None:
        self.points.append(point)
        self.update()

    def clear(self) -> None:
        self.points.clear()
        self.update()

    def _panel_rects(self) -> list[QtCore.QRectF]:
        content = QtCore.QRectF(self.rect().adjusted(8, 8, -8, -8))
        panel_height = content.height() / len(self.PANELS)
        return [
            QtCore.QRectF(
                content.left(),
                content.top() + index * panel_height,
                content.width(),
                panel_height - 6,
            )
            for index in range(len(self.PANELS))
        ]

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt override.
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor(CHART_BG))
        if not self.points:
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "RTCP/NTP 샘플 대기 중")
            return
        for rect, (title, unit, series) in zip(self._panel_rects(), self.PANELS):
            self._draw_panel(painter, rect, title, unit, series)

    def time_axis_ticks(self, tick_count: int = 4) -> list[float]:
        """Age-in-seconds tick values, newest (0) last.

        Negative because the right edge is the most recent sample, so a tick
        reads as "this many seconds ago".
        """

        if not self.points:
            return []
        span_s = max(0.0, float(self.points[-1].timestamp_s) - float(self.points[0].timestamp_s))
        steps = max(1, int(tick_count))
        return [-span_s * (steps - index) / steps for index in range(steps)] + [0.0]

    def _draw_panel(self, painter, rect, title: str, unit: str, series) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(CHART_AXIS), 1))
        painter.drawRect(rect)
        plot_rect = rect.adjusted(56, 18, -10, -22)
        values = [
            value
            for field, _label, _color in series
            for value in (getattr(point, field) for point in self.points)
            if value is not None and isfinite(value)
        ]
        painter.setPen(QtGui.QColor(CHART_TEXT))
        if not values:
            painter.drawText(rect.adjusted(6, 2, 0, 0), QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft, f"{title}: --")
            return
        minimum = min(0.0, min(values))
        maximum = max(values)
        if maximum - minimum < 1e-9:
            maximum = minimum + 1.0
        latest = {field: getattr(self.points[-1], field) for field, _label, _color in series}
        legend = "  ".join(
            f"{label} {'--' if latest[field] is None else f'{latest[field]:.1f}'}"
            for field, label, _color in series
        )
        painter.drawText(
            rect.adjusted(6, 2, 0, 0),
            QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft,
            f"{title} ({unit})   {legend}",
        )
        painter.setPen(QtGui.QColor(CHART_MUTED))
        painter.drawText(
            QtCore.QRectF(rect.left() + 4, plot_rect.top() - 6, 50, 14),
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
            f"{maximum:.1f}",
        )
        painter.drawText(
            QtCore.QRectF(rect.left() + 4, plot_rect.bottom() - 8, 50, 14),
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
            f"{minimum:.1f}",
        )

        newest_s = float(self.points[-1].timestamp_s)
        oldest_s = float(self.points[0].timestamp_s)
        span_s = max(1e-6, newest_s - oldest_s)

        def x_for_age(age_s: float) -> float:
            # age_s is <= 0; 0 is the newest sample at the right edge.
            return plot_rect.right() + plot_rect.width() * age_s / span_s

        for age_s in self.time_axis_ticks():
            x = x_for_age(age_s)
            painter.setPen(QtGui.QPen(QtGui.QColor(CHART_GRID), 1))
            painter.drawLine(QtCore.QPointF(x, plot_rect.top()), QtCore.QPointF(x, plot_rect.bottom()))
            painter.setPen(QtGui.QColor(CHART_MUTED))
            painter.drawText(
                QtCore.QRectF(x - 30, plot_rect.bottom() + 2, 60, 14),
                QtCore.Qt.AlignCenter,
                f"{age_s:.0f}s",
            )

        for field, _label, color in series:
            # Time-based x, so a sampling gap shows as a gap instead of being
            # compressed away by index-based spacing.
            path_points = [
                QtCore.QPointF(
                    x_for_age(float(point.timestamp_s) - newest_s),
                    plot_rect.bottom() - plot_rect.height() * (value - minimum) / (maximum - minimum),
                )
                for point in self.points
                for value in (getattr(point, field),)
                if value is not None and isfinite(value)
            ]
            if len(path_points) < 2:
                continue
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
            painter.drawPolyline(QtGui.QPolygonF(path_points))
