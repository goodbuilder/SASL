"""Thread-safe, read-only rqt UI for PoseSimulation diagnostics."""

from __future__ import annotations

import math
import os
import threading
from collections import deque
from pathlib import Path
from typing import Any, Mapping, Sequence

from python_qt_binding.QtCore import QObject, Qt, Signal, Slot
from python_qt_binding.QtGui import QColor, QFont, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

try:  # rqt may load the shell in environments without ROS during discovery.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
except Exception:  # pragma: no cover - depends on the runtime environment
    rclpy = None
    DiagnosticArray = None  # type: ignore[assignment,misc]
    SingleThreadedExecutor = None  # type: ignore[assignment,misc]
    String = None  # type: ignore[assignment,misc]

from pose_simulation.ui_snapshot import PoseSimulationSnapshot, build_ui_snapshot


WORLD_NAMESPACE = "/simulation/pose_simulation"


def _load_ui_config(path: str | None) -> Mapping[str, Any]:
    if not path:
        return {}
    try:
        import yaml

        parsed = yaml.safe_load(Path(path).read_text())
        return parsed if isinstance(parsed, Mapping) else {}
    except (OSError, TypeError, ValueError):
        return {}


def _topic(namespace: str, name: str) -> str:
    return f"/{namespace.strip('/')}/{name.strip('/')}"


def _finite(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _diagnostic_level(value: Any) -> int:
    if isinstance(value, (bytes, bytearray)):
        return int(value[0]) if value else 0
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _at(payload: Mapping[str, Any] | None, *keys: str) -> Any:
    value: Any = payload
    for key in keys:
        if not isinstance(value, Mapping):
            return None
        value = value.get(key)
    return value


def _format_value(value: Any, *, precision: int = 3, unit: str = "") -> str:
    parsed = _finite(value)
    if parsed is None:
        return "UNAVAILABLE"
    return f"{parsed:.{precision}f}{unit}"


def _threshold_text(thresholds: Mapping[str, Any], metric: str) -> str:
    setting = thresholds.get(metric)
    if not isinstance(setting, Mapping):
        return "VALUE AVAILABLE — THRESHOLD UNSPECIFIED"
    parts = []
    for name in ("warning", "failure"):
        value = _finite(setting.get(name))
        if value is not None:
            parts.append(f"{name} ≥ {value:g}")
    return "; ".join(parts) if parts else "VALUE AVAILABLE — THRESHOLD UNSPECIFIED"


def _state_text(state: str) -> str:
    value = str(state or "UNAVAILABLE").upper()
    if value == "VALID":
        return "✓ VALID"
    if value in {"WARNING", "DEGRADED"}:
        return "△ WARNING"
    if value in {"STALE", "○ STALE"}:
        return "○ STALE"
    if value in {"UNALIGNED", "UNALIGNED SAMPLE", "PAIR ID MISMATCH", "TIMESTAMP MISMATCH", "FRAME MISMATCH"}:
        return f"△ {value}"
    if value == "UNAVAILABLE":
        return "○ UNAVAILABLE"
    return f"✕ {value}"


def _state_color(state: str) -> str:
    value = str(state or "UNAVAILABLE").upper()
    if value == "VALID":
        return "#62d58a"
    if value in {"WARNING", "DEGRADED", "UNALIGNED", "UNALIGNED SAMPLE", "PAIR ID MISMATCH", "TIMESTAMP MISMATCH"}:
        return "#f2c14e"
    if value in {"STALE", "UNAVAILABLE"}:
        return "#a8adb7"
    return "#ff6b6b"


class PoseSimulationRosBridge(QObject):
    """ROS callbacks build snapshots and emit them; they never touch widgets."""

    snapshot_ready = Signal(object)
    diagnostics_ready = Signal(object)
    bridge_error = Signal(str)

    def __init__(self, namespace: str = WORLD_NAMESPACE, parent: QObject | None = None):
        super().__init__(parent)
        self._namespace = namespace
        self._payloads: dict[str, Mapping[str, Any]] = {}
        self._node = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._owns_context = False
        self._start()

    def _start(self) -> None:
        if rclpy is None or String is None or DiagnosticArray is None:
            self.bridge_error.emit("ROS 2 rclpy/diagnostic messages unavailable")
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_context = True
            self._node = rclpy.create_node("pose_simulation_rqt")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            for kind in ("pose_error", "stereo_geometry", "status"):
                self._node.create_subscription(String, _topic(self._namespace, kind), lambda message, name=kind: self._json_callback(name, message), 10)
            self._node.create_subscription(
                DiagnosticArray,
                _topic(self._namespace, "diagnostics"),
                self._diagnostics_callback,
                10,
            )
            self._thread = threading.Thread(target=self._spin, name="pose-simulation-rqt-ros", daemon=True)
            self._thread.start()
        except Exception as exc:  # pragma: no cover - depends on ROS discovery
            self.bridge_error.emit(f"ROS bridge unavailable: {exc}")

    def _spin(self) -> None:
        try:
            while not self._stop.is_set() and self._executor is not None:
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as exc:  # pragma: no cover - shutdown races are runtime-specific
            if not self._stop.is_set():
                self.bridge_error.emit(f"ROS bridge stopped: {exc}")

    def _json_callback(self, name: str, message: Any) -> None:
        try:
            import json

            payload = json.loads(str(message.data))
            if not isinstance(payload, Mapping):
                raise ValueError("JSON payload is not an object")
            self._payloads[name] = dict(payload)
            snapshot = build_ui_snapshot(
                self._payloads.get("pose_error"),
                self._payloads.get("stereo_geometry"),
                self._payloads.get("status"),
            )
            self.snapshot_ready.emit(snapshot)
        except Exception as exc:
            self.bridge_error.emit(f"{name} payload rejected: {exc}")

    def _diagnostics_callback(self, message: Any) -> None:
        records = tuple(
            {
                "name": str(item.name),
                "level": _diagnostic_level(item.level),
                "message": str(item.message),
            }
            for item in getattr(message, "status", ())
        )
        self.diagnostics_ready.emit(records)

    def shutdown(self) -> None:
        self._stop.set()
        if self._executor is not None:
            try:
                self._executor.shutdown()
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._owns_context and rclpy is not None and rclpy.ok():
            rclpy.shutdown()


class TrendPlot(QWidget):
    """Small dependency-free line plot for bounded diagnostic trends."""

    def __init__(self, title: str, unit: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._title = title
        self._unit = unit
        self._series: Mapping[str, Sequence[tuple[float, float | None]]] = {}
        self._window_s: float | None = 30.0
        self.setMinimumHeight(150)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_data(self, series: Mapping[str, Sequence[tuple[float, float | None]]], window_s: float | None) -> None:
        self._series = series
        self._window_s = window_s
        self.update()

    def paintEvent(self, event) -> None:  # pragma: no cover - visual rendering
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#20242b"))
        painter.setPen(QColor("#e8edf2"))
        painter.setFont(QFont("Sans", 9, QFont.Bold))
        painter.drawText(10, 18, f"{self._title} [{self._unit}]")
        plot = self.rect().adjusted(48, 28, -12, -28)
        painter.setPen(QColor("#535b66"))
        painter.drawRect(plot)
        points = [
            (time_value, value)
            for values in self._series.values()
            for time_value, value in values
            if value is not None and math.isfinite(value)
        ]
        if not points:
            painter.setPen(QColor("#a8adb7"))
            painter.drawText(plot, Qt.AlignCenter, "No valid samples")
            return
        max_time = max(point[0] for point in points)
        min_time = max_time - self._window_s if self._window_s is not None else min(point[0] for point in points)
        visible = [(t, v) for t, v in points if t >= min_time]
        values = [v for _, v in visible]
        min_value = min(values)
        max_value = max(values)
        if abs(max_value - min_value) < 1.0e-12:
            padding = max(1.0, abs(max_value) * 0.1)
            min_value -= padding
            max_value += padding
        else:
            padding = (max_value - min_value) * 0.08
            min_value -= padding
            max_value += padding
        painter.setPen(QColor("#7c8591"))
        painter.drawText(2, plot.top() + 4, f"{max_value:.3g}")
        painter.drawText(2, plot.bottom(), f"{min_value:.3g}")
        palette = ("#63d2ff", "#ff7a8a", "#ffd166", "#9bde7e", "#c792ea")
        for index, (name, values_for_series) in enumerate(self._series.items()):
            path = []
            for timestamp, value in values_for_series:
                if value is None or not math.isfinite(value) or timestamp < min_time:
                    if path:
                        self._draw_polyline(painter, path, palette[index % len(palette)])
                        path = []
                    continue
                x = plot.left() + (0.5 if max_time == min_time else (timestamp - min_time) / (max_time - min_time)) * plot.width()
                y = plot.bottom() - (value - min_value) / (max_value - min_value) * plot.height()
                path.append((int(x), int(y)))
            if path:
                self._draw_polyline(painter, path, palette[index % len(palette)])
        painter.setPen(QColor("#e8edf2"))
        legend_x = plot.left()
        for index, name in enumerate(self._series):
            color = palette[index % len(palette)]
            painter.setPen(QColor(color))
            painter.drawText(legend_x, self.height() - 8, name)
            legend_x += max(70, len(name) * 7 + 12)

    @staticmethod
    def _draw_polyline(painter: QPainter, points: Sequence[tuple[int, int]], color: str) -> None:
        if len(points) < 2:
            return
        painter.setPen(QPen(QColor(color), 2))
        for first, second in zip(points, points[1:]):
            painter.drawLine(first[0], first[1], second[0], second[1])


class PoseSimulationWidget(QWidget):
    """Five-tab, read-only state and metric view."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("PoseSimulationWidget")
        self.setWindowTitle("PoseSimulation")
        self._ui_settings = _load_ui_config(os.environ.get("POSE_SIMULATION_UI_CONFIG"))
        configured_thresholds = self._ui_settings.get("ui_thresholds", {})
        self._thresholds = configured_thresholds if isinstance(configured_thresholds, Mapping) else {}
        self._snapshot: PoseSimulationSnapshot | None = None
        self._diagnostics: tuple[Mapping[str, Any], ...] = ()
        try:
            trend_buffer_size = max(10, int(self._ui_settings.get("trend_buffer_size", 6000)))
        except (TypeError, ValueError):
            trend_buffer_size = 6000
        self._history: deque[tuple[float, dict[str, float | None]]] = deque(maxlen=trend_buffer_size)
        self._last_history_id: str | None = None
        self._trends_paused = False
        self._trend_window_s: float | None = 30.0
        self._status_labels: dict[str, QLabel] = {}
        self._metric_labels: dict[str, QLabel] = {}
        self._build_ui()
        namespace = os.environ.get("POSE_SIMULATION_NAMESPACE") or str(self._ui_settings.get("namespace", WORLD_NAMESPACE))
        self._bridge = PoseSimulationRosBridge(namespace=namespace, parent=self)
        self._bridge.snapshot_ready.connect(self.update_snapshot)
        self._bridge.diagnostics_ready.connect(self.update_diagnostics)
        self._bridge.bridge_error.connect(self._show_bridge_error)
        self._refresh_status_bar()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.addWidget(self._build_status_bar())
        self._tabs = QTabWidget()
        self._tabs.addTab(self._build_overview_tab(), "Overview")
        self._tabs.addTab(self._build_pose_error_tab(), "Pose Error")
        self._tabs.addTab(self._build_stereo_geometry_tab(), "Stereo Geometry")
        self._tabs.addTab(self._build_reference_time_tab(), "Reference & Time")
        self._tabs.addTab(self._build_trends_tab(), "Trends")
        root.addWidget(self._tabs)

    def _build_status_bar(self) -> QWidget:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        frame.setStyleSheet("QFrame { background: #252a33; border: 1px solid #4d5662; } QLabel { padding: 2px 5px; }")
        layout = QGridLayout(frame)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.addWidget(self._make_status_label("DIAGNOSTIC ONLY — NO POSE COMMAND", "application_banner"), 0, 0, 1, 4)
        for column, key in enumerate(("application", "ground_truth", "rtk", "left_optical", "right_optical", "time_alignment", "frame_contract")):
            label = self._make_status_label(f"{key}: UNAVAILABLE", key)
            self._status_labels[key] = label
            layout.addWidget(label, 1, column % 4)
        self._status_labels["write_guard"] = self._make_status_label("POSE WRITE: DISABLED", "write_guard")
        self._status_labels["evaluation"] = self._make_status_label("evaluation: UNAVAILABLE", "evaluation")
        self._status_labels["timestamp"] = self._make_status_label("timestamp: UNAVAILABLE", "timestamp")
        layout.addWidget(self._status_labels["write_guard"], 2, 0)
        layout.addWidget(self._status_labels["evaluation"], 2, 1, 1, 2)
        layout.addWidget(self._status_labels["timestamp"], 2, 3)
        return frame

    @staticmethod
    def _make_status_label(text: str, name: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName(name)
        label.setStyleSheet("color: #a8adb7; font-weight: 600;")
        return label

    def _metric_group(self, title: str, specs: Sequence[tuple[str, str]]) -> QGroupBox:
        group = QGroupBox(title)
        layout = QGridLayout(group)
        layout.setColumnStretch(1, 1)
        for row, (key, label_text) in enumerate(specs):
            label = QLabel(label_text)
            value = QLabel("UNAVAILABLE")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self._metric_labels[key] = value
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
        return group

    def _build_overview_tab(self) -> QWidget:
        widget = QWidget()
        layout = QGridLayout(widget)
        layout.addWidget(self._metric_group("Left actual vs GT ideal", [("ov_l_roll", "Roll error"), ("ov_l_pitch", "Pitch error"), ("ov_l_yaw", "Yaw error"), ("ov_l_rot", "Total rotation error"), ("ov_l_los", "Optical-axis → GT target"), ("ov_l_valid", "Validity")]), 0, 0)
        layout.addWidget(self._metric_group("Right actual vs GT ideal", [("ov_r_roll", "Roll error"), ("ov_r_pitch", "Pitch error"), ("ov_r_yaw", "Yaw error"), ("ov_r_rot", "Total rotation error"), ("ov_r_los", "Optical-axis → GT target"), ("ov_r_valid", "Validity")]), 0, 1)
        layout.addWidget(self._metric_group("Stereo relative", [("ov_rel_roll", "Relative roll"), ("ov_rel_pitch", "Relative pitch"), ("ov_rel_yaw", "Relative yaw"), ("ov_rel_rot", "Relative rotation"), ("ov_rel_axis", "Left-right optical-axis angle"), ("ov_base_len", "Baseline length error")]), 1, 0)
        layout.addWidget(self._metric_group("RTK reference error", [("ov_rtk_h", "Horizontal position"), ("ov_rtk_v", "Vertical position"), ("ov_rtk_3d", "3D position"), ("ov_rtk_los", "RTK LOS → GT LOS"), ("ov_rtk_rot", "RTK ideal → GT ideal"), ("ov_rtk_valid", "Operational validity")]), 1, 1)
        note = QLabel("Thresholds: VALUE AVAILABLE — THRESHOLD UNSPECIFIED when no requirement is configured. Relative yaw is not a complete depth-error verdict.")
        note.setWordWrap(True)
        layout.addWidget(note, 2, 0, 1, 2)
        return widget

    def _build_pose_error_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self._pose_error_table = QTableWidget(5, 7)
        self._pose_error_table.setHorizontalHeaderLabels(["Comparison", "Roll°", "Pitch°", "Yaw°", "Rotation°", "Optical-axis°", "Validity"])
        self._pose_error_table.verticalHeader().setVisible(False)
        self._pose_error_table.setEditTriggers(QTableWidget.NoEditTriggers)
        for row, name in enumerate(("Left actual vs GT ideal", "Right actual vs GT ideal", "Left actual vs RTK ideal", "Right actual vs RTK ideal", "RTK ideal vs GT ideal")):
            self._pose_error_table.setItem(row, 0, QTableWidgetItem(name))
        layout.addWidget(self._pose_error_table)
        equation = QLabel("R_error = R_idealᵀ · R_actual | Euler: fixed extrinsic ZYX (Rz Ry Rx), degrees, quaternion xyzw, frame gazebo_world_enu")
        equation.setWordWrap(True)
        layout.addWidget(equation)
        self._pose_error_note = QLabel("UNAVAILABLE")
        layout.addWidget(self._pose_error_note)
        return widget

    def _build_stereo_geometry_tab(self) -> QWidget:
        widget = QWidget()
        layout = QGridLayout(widget)
        layout.addWidget(self._metric_group("Relative orientation", [("sg_roll", "Relative roll"), ("sg_pitch", "Relative pitch"), ("sg_yaw", "Relative yaw"), ("sg_rot", "Relative rotation"), ("sg_axis", "Left-right optical-axis angle")]), 0, 0)
        layout.addWidget(self._metric_group("Baseline", [("sg_nominal", "Nominal length"), ("sg_actual", "Actual length"), ("sg_len_err", "Length error"), ("sg_dir_err", "Direction error"), ("sg_midpoint", "Actual midpoint E/N/U"), ("sg_midpoint_err", "Midpoint error")]), 0, 1)
        layout.addWidget(self._metric_group("Optical alignment", [("sg_l_gt", "Left optical axis → GT"), ("sg_r_gt", "Right optical axis → GT"), ("sg_l_rtk", "Left optical axis → RTK"), ("sg_r_rtk", "Right optical axis → RTK")]), 1, 0)
        layout.addWidget(self._metric_group("Provenance", [("sg_rig_source", "Rig center source"), ("sg_alignment", "Target-point alignment"), ("sg_lever", "Lever arm applied"), ("sg_threshold", "Threshold policy")]), 1, 1)
        return widget

    def _build_reference_time_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(self._metric_group("Gazebo GT", [("rt_gt_source", "Ground truth source"), ("rt_gt_entity", "Target entity/link"), ("rt_gt_fallback", "Fallback used"), ("rt_gt_frame", "Frame"), ("rt_gt_age", "Pose age"), ("rt_gt_stamp", "Source timestamp"), ("rt_gt_valid", "Validity")]))
        layout.addWidget(self._metric_group("RTK Operational Reference", [("rt_rtk_topic", "Source topic"), ("rt_rtk_status", "Fix status"), ("rt_rtk_lat", "Latitude"), ("rt_rtk_lon", "Longitude"), ("rt_rtk_alt", "Altitude"), ("rt_rtk_datum", "Altitude datum"), ("rt_rtk_valid", "Operational validity")]))
        self._time_table = QTableWidget(4, 5)
        self._time_table.setHorizontalHeaderLabels(["Source", "Source timestamp", "Evaluation timestamp", "Residual ms", "Validity"])
        self._time_table.verticalHeader().setVisible(False)
        self._time_table.setEditTriggers(QTableWidget.NoEditTriggers)
        for row, name in enumerate(("Gazebo GT", "Left optical pose", "Right optical pose", "RTK")):
            self._time_table.setItem(row, 0, QTableWidgetItem(name))
        layout.addWidget(QLabel("Time alignment uses source-specific clocks; PX4/Gazebo clock domains are not subtracted directly."))
        layout.addWidget(self._time_table)
        self._reference_warning = QLabel("ZERO LEVER-ARM ASSUMPTION / fallback warnings appear here.")
        self._reference_warning.setWordWrap(True)
        layout.addWidget(self._reference_warning)
        self._diagnostic_label = QLabel("Diagnostics: UNAVAILABLE")
        self._diagnostic_label.setWordWrap(True)
        layout.addWidget(self._diagnostic_label)
        return widget

    def _build_trends_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        controls = QHBoxLayout()
        controls.addWidget(QLabel("Display window"))
        self._trend_combo = QComboBox()
        self._trend_combo.addItem("10 s", 10.0)
        self._trend_combo.addItem("30 s", 30.0)
        self._trend_combo.addItem("60 s", 60.0)
        self._trend_combo.addItem("Full available buffer", None)
        self._trend_combo.setCurrentIndex(1)
        self._trend_combo.currentIndexChanged.connect(self._trend_window_changed)
        controls.addWidget(self._trend_combo)
        self._trend_pause_button = QPushButton("Pause display")
        self._trend_pause_button.setCheckable(True)
        self._trend_pause_button.toggled.connect(self._trend_pause_changed)
        controls.addWidget(self._trend_pause_button)
        controls.addStretch(1)
        layout.addLayout(controls)
        self._plots = [
            TrendPlot("Relative attitude", "deg"),
            TrendPlot("GT optical-axis errors", "deg"),
            TrendPlot("RTK reference LOS error", "deg"),
            TrendPlot("Source age / time residual", "ms"),
            TrendPlot("Baseline length stability", "m"),
            TrendPlot("Baseline direction stability", "deg"),
        ]
        for plot in self._plots:
            layout.addWidget(plot)
        return widget

    @Slot(object)
    def update_snapshot(self, snapshot: PoseSimulationSnapshot) -> None:
        self._snapshot = snapshot
        self._refresh_status_bar()
        self._refresh_overview()
        self._refresh_pose_error()
        self._refresh_stereo_geometry()
        self._refresh_reference_time()
        if snapshot.pair_state == "ALIGNED" and not self._trends_paused and snapshot.evaluation_id != self._last_history_id:
            self._last_history_id = snapshot.evaluation_id
            timestamp_s = (snapshot.evaluation_timestamp_ns or 0) / 1.0e9
            self._history.append((timestamp_s, self._trend_values(snapshot)))
            self._refresh_trends()

    @Slot(object)
    def update_diagnostics(self, records: Sequence[Mapping[str, Any]]) -> None:
        self._diagnostics = tuple(records)
        if records:
            invalid = [record for record in records if int(record.get("level", 0)) >= 2]
            self._diagnostic_label.setText(f"Diagnostics: {len(records)} statuses; errors={len(invalid)}")
        else:
            self._diagnostic_label.setText("Diagnostics: UNAVAILABLE")

    @Slot(str)
    def _show_bridge_error(self, message: str) -> None:
        self._diagnostic_label.setText(f"Diagnostics: {message}")

    def _refresh_status_bar(self) -> None:
        if self._snapshot is None:
            for key in ("application", "ground_truth", "rtk", "left_optical", "right_optical", "time_alignment", "frame_contract"):
                self._set_state_label(self._status_labels[key], key.upper() + ": UNAVAILABLE", "UNAVAILABLE")
            return
        state = self._snapshot.state
        for key, value in (("application", state.application), ("ground_truth", state.ground_truth), ("rtk", state.rtk), ("left_optical", state.left_optical), ("right_optical", state.right_optical), ("time_alignment", state.time_alignment), ("frame_contract", state.frame_contract)):
            self._set_state_label(self._status_labels[key], f"{key.upper()}: {_state_text(value)}", value)
        self._status_labels["write_guard"].setText("POSE WRITE: DISABLED")
        self._status_labels["evaluation"].setText(f"evaluation: {self._snapshot.evaluation_id or 'UNAVAILABLE'}")
        self._status_labels["timestamp"].setText(f"timestamp: {self._snapshot.evaluation_timestamp_ns or 'UNAVAILABLE'}")

    @staticmethod
    def _set_state_label(label: QLabel, text: str, state: str) -> None:
        label.setText(text)
        label.setStyleSheet(f"color: {_state_color(state)}; font-weight: 600;")

    def _set_metric(self, key: str, value: Any, *, precision: int = 3, unit: str = "") -> None:
        label = self._metric_labels.get(key)
        if label is not None:
            label.setText(_format_value(value, precision=precision, unit=unit))

    def _set_text_metric(self, key: str, value: Any) -> None:
        label = self._metric_labels.get(key)
        if label is not None:
            label.setText("UNAVAILABLE" if value is None or value == "" else str(value))

    def _refresh_overview(self) -> None:
        snapshot = self._snapshot
        if snapshot is None:
            return
        error = snapshot.pose_error
        stereo = snapshot.stereo_geometry
        left = error.get("left_actual_vs_gt_ideal", {})
        right = error.get("right_actual_vs_gt_ideal", {})
        relative = stereo
        baseline = _at(stereo, "baseline")
        rtk_gt = error.get("rtk_ideal_vs_gt_ideal", {})
        for prefix, mapping in (("ov_l", left), ("ov_r", right)):
            for suffix, field in (("roll", "roll_error_deg"), ("pitch", "pitch_error_deg"), ("yaw", "yaw_error_deg"), ("rot", "rotation_error_deg")):
                self._set_metric(f"{prefix}_{suffix}", _at(mapping, field), unit="°")
            self._set_metric(f"{prefix}_los", _at(mapping, "optical_axis_to_gt_target_error_deg"), unit="°")
            self._set_text_metric(f"{prefix}_valid", _state_text("VALID" if bool(_at(mapping, "valid")) else "INVALID"))
        for suffix, field in (("roll", "relative_roll_error_deg"), ("pitch", "relative_pitch_error_deg"), ("yaw", "relative_yaw_error_deg"), ("rot", "relative_rotation_error_deg"), ("axis", "left_right_optical_axis_angle_deg")):
            self._set_metric(f"ov_rel_{suffix}", _at(relative, field), unit="°")
        self._set_metric("ov_base_len", _at(baseline, "length_error_m"), unit=" m")
        for key in ("ov_rtk_h", "ov_rtk_v", "ov_rtk_3d"):
            self._set_text_metric(key, "UNAVAILABLE — CORE FIELD NOT PROVIDED")
        self._set_metric("ov_rtk_los", _at(rtk_gt, "los_direction_error_deg"), unit="°")
        self._set_metric("ov_rtk_rot", _at(rtk_gt, "rotation_error_deg"), unit="°")
        self._set_text_metric("ov_rtk_valid", _state_text(snapshot.state.rtk))

    def _refresh_pose_error(self) -> None:
        snapshot = self._snapshot
        if snapshot is None:
            return
        rows = (
            snapshot.pose_error.get("left_actual_vs_gt_ideal", {}),
            snapshot.pose_error.get("right_actual_vs_gt_ideal", {}),
            snapshot.pose_error.get("left_actual_vs_rtk_ideal", {}),
            snapshot.pose_error.get("right_actual_vs_rtk_ideal", {}),
            snapshot.pose_error.get("rtk_ideal_vs_gt_ideal", {}),
        )
        for row, mapping in enumerate(rows):
            fields = ("roll_error_deg", "pitch_error_deg", "yaw_error_deg", "rotation_error_deg", "optical_axis_to_gt_target_error_deg")
            for column, field in enumerate(fields, start=1):
                value = _at(mapping, field)
                if row == 2 and field == "optical_axis_to_gt_target_error_deg":
                    value = _at(mapping, "optical_axis_to_rtk_target_error_deg")
                if row == 3 and field == "optical_axis_to_gt_target_error_deg":
                    value = _at(mapping, "optical_axis_to_rtk_target_error_deg")
                if row == 4 and field == "optical_axis_to_gt_target_error_deg":
                    value = _at(mapping, "los_direction_error_deg")
                self._pose_error_table.setItem(row, column, QTableWidgetItem(_format_value(value, unit="°")))
            valid = bool(_at(mapping, "valid"))
            self._pose_error_table.setItem(row, 6, QTableWidgetItem(_state_text("VALID" if valid else "INVALID")))
        self._pose_error_note.setText(f"Reference: Gazebo GT Ideal primary. Sample: {snapshot.evaluation_id or 'UNAVAILABLE'} | {snapshot.reason or 'ALIGNED'}")

    def _refresh_stereo_geometry(self) -> None:
        snapshot = self._snapshot
        if snapshot is None:
            return
        stereo = snapshot.stereo_geometry
        baseline = _at(stereo, "baseline")
        for key, field in (("sg_roll", "relative_roll_error_deg"), ("sg_pitch", "relative_pitch_error_deg"), ("sg_yaw", "relative_yaw_error_deg"), ("sg_rot", "relative_rotation_error_deg"), ("sg_axis", "left_right_optical_axis_angle_deg")):
            self._set_metric(key, _at(stereo, field), unit="°")
        for key, field, unit in (("sg_nominal", "nominal_length_m", " m"), ("sg_actual", "actual_length_m", " m"), ("sg_len_err", "length_error_m", " m"), ("sg_dir_err", "direction_error_deg", "°")):
            self._set_metric(key, _at(baseline, field), unit=unit)
        midpoint = _at(baseline, "midpoint_world_enu_m")
        self._set_text_metric("sg_midpoint", "E/N/U " + "/".join(_format_value(value, precision=3) for value in midpoint) if isinstance(midpoint, Sequence) and len(midpoint) == 3 else "UNAVAILABLE")
        midpoint_status = _at(baseline, "midpoint_error_m")
        self._set_text_metric("sg_midpoint_err", _format_value(midpoint_status, unit=" m") if midpoint_status is not None else "NOT AVAILABLE")
        left_gt = snapshot.pose_error.get("left_actual_vs_gt_ideal", {})
        right_gt = snapshot.pose_error.get("right_actual_vs_gt_ideal", {})
        left_rtk = snapshot.pose_error.get("left_actual_vs_rtk_ideal", {})
        right_rtk = snapshot.pose_error.get("right_actual_vs_rtk_ideal", {})
        self._set_metric("sg_l_gt", _at(left_gt, "optical_axis_to_gt_target_error_deg"), unit="°")
        self._set_metric("sg_r_gt", _at(right_gt, "optical_axis_to_gt_target_error_deg"), unit="°")
        self._set_metric("sg_l_rtk", _at(left_rtk, "optical_axis_to_rtk_target_error_deg"), unit="°")
        self._set_metric("sg_r_rtk", _at(right_rtk, "optical_axis_to_rtk_target_error_deg"), unit="°")
        self._set_text_metric("sg_rig_source", snapshot.status.get("rig_center_source"))
        self._set_text_metric("sg_alignment", snapshot.status.get("target_point_alignment_mode"))
        self._set_text_metric("sg_lever", snapshot.status.get("lever_arm_applied"))
        self._set_text_metric("sg_threshold", _threshold_text(self._thresholds, "relative_yaw_error_deg"))

    def _refresh_reference_time(self) -> None:
        snapshot = self._snapshot
        if snapshot is None:
            return
        status = snapshot.status
        frames = status.get("coordinate_frames", {})
        self._set_text_metric("rt_gt_source", status.get("ground_truth_source"))
        self._set_text_metric("rt_gt_entity", status.get("target_entity_name"))
        self._set_text_metric("rt_gt_fallback", status.get("fallback_used"))
        self._set_text_metric("rt_gt_frame", frames.get("world") if isinstance(frames, Mapping) else "gazebo_world_enu")
        self._set_text_metric("rt_gt_age", _format_value(_at(status, "time_residuals", "gt_receive_age_ms"), unit=" ms"))
        self._set_text_metric("rt_gt_stamp", status.get("target_gt_timestamp_ns"))
        self._set_text_metric("rt_gt_valid", _state_text(snapshot.state.ground_truth))
        self._set_text_metric("rt_rtk_topic", status.get("rtk_source_topic"))
        self._set_text_metric("rt_rtk_status", status.get("rtk_status"))
        self._set_text_metric("rt_rtk_lat", "UNAVAILABLE — CORE DOES NOT REPUBLISH RAW RTK FIELDS")
        self._set_text_metric("rt_rtk_lon", "UNAVAILABLE — CORE DOES NOT REPUBLISH RAW RTK FIELDS")
        self._set_text_metric("rt_rtk_alt", "UNAVAILABLE — CORE DOES NOT REPUBLISH RAW RTK FIELDS")
        self._set_text_metric("rt_rtk_datum", status.get("altitude_datum"))
        self._set_text_metric("rt_rtk_valid", _state_text(snapshot.state.rtk))
        residuals = status.get("time_residuals", {})
        sources = (
            ("Gazebo GT", status.get("target_gt_timestamp_ns"), "gt_time_residual_ms"),
            ("Left optical pose", status.get("left_camera_pose_timestamp_ns"), "left_pose_time_residual_ms"),
            ("Right optical pose", status.get("right_camera_pose_timestamp_ns"), "right_pose_time_residual_ms"),
            ("RTK", status.get("rtk_source_timestamp_us"), "rtk_time_residual_ms"),
        )
        for row, (_, source_stamp, residual_key) in enumerate(sources):
            self._time_table.setItem(row, 1, QTableWidgetItem("UNAVAILABLE" if source_stamp is None else str(source_stamp)))
            self._time_table.setItem(row, 2, QTableWidgetItem("UNAVAILABLE" if snapshot.evaluation_timestamp_ns is None else str(snapshot.evaluation_timestamp_ns)))
            self._time_table.setItem(row, 3, QTableWidgetItem(_format_value(_at(residuals, residual_key), unit=" ms")))
            self._time_table.setItem(row, 4, QTableWidgetItem(_state_text(snapshot.state.time_alignment if row < 3 else snapshot.state.rtk)))
        warnings = []
        if status.get("fallback_used"):
            warnings.append("WARNING — FALLBACK GROUND-TRUTH SOURCE")
        if status.get("target_point_alignment_mode") == "zero_lever_arm_assumption":
            warnings.append("ZERO LEVER-ARM ASSUMPTION")
        self._reference_warning.setText(" | ".join(warnings) if warnings else "No provenance warning")

    def _trend_values(self, snapshot: PoseSimulationSnapshot) -> dict[str, float | None]:
        left = snapshot.pose_error.get("left_actual_vs_gt_ideal", {})
        right = snapshot.pose_error.get("right_actual_vs_gt_ideal", {})
        residuals = snapshot.status.get("time_residuals", {})
        baseline = _at(snapshot.stereo_geometry, "baseline")
        return {
            "relative_roll": _finite(_at(snapshot.stereo_geometry, "relative_roll_error_deg")),
            "relative_pitch": _finite(_at(snapshot.stereo_geometry, "relative_pitch_error_deg")),
            "relative_yaw": _finite(_at(snapshot.stereo_geometry, "relative_yaw_error_deg")),
            "left_gt_los": _finite(_at(left, "optical_axis_to_gt_target_error_deg")),
            "right_gt_los": _finite(_at(right, "optical_axis_to_gt_target_error_deg")),
            "rtk_los": _finite(_at(snapshot.pose_error, "rtk_ideal_vs_gt_ideal", "los_direction_error_deg")),
            "gt_residual": _finite(_at(residuals, "gt_time_residual_ms")),
            "left_age": _finite(_at(residuals, "left_pose_time_residual_ms")),
            "right_age": _finite(_at(residuals, "right_pose_time_residual_ms")),
            "rtk_residual": _finite(_at(residuals, "rtk_time_residual_ms")),
            "baseline_length": _finite(_at(baseline, "length_error_m")),
            "baseline_direction": _finite(_at(baseline, "direction_error_deg")),
        }

    def _refresh_trends(self) -> None:
        if not hasattr(self, "_plots"):
            return
        points = list(self._history)

        def series(keys: Sequence[tuple[str, str]]) -> dict[str, list[tuple[float, float | None]]]:
            return {label: [(timestamp, values.get(key)) for timestamp, values in points] for key, label in keys}

        plot_data = (
            series((("relative_roll", "roll"), ("relative_pitch", "pitch"), ("relative_yaw", "yaw"))),
            series((("left_gt_los", "left"), ("right_gt_los", "right"))),
            series((("rtk_los", "RTK LOS→GT"),)),
            series((("gt_residual", "GT"), ("left_age", "left"), ("right_age", "right"), ("rtk_residual", "RTK"))),
            series((("baseline_length", "length error"),)),
            series((("baseline_direction", "direction error"),)),
        )
        for plot, data in zip(self._plots, plot_data):
            plot.set_data(data, self._trend_window_s)

    @Slot(int)
    def _trend_window_changed(self, index: int) -> None:
        self._trend_window_s = self._trend_combo.itemData(index)
        self._refresh_trends()

    @Slot(bool)
    def _trend_pause_changed(self, paused: bool) -> None:
        self._trends_paused = bool(paused)
        self._trend_pause_button.setText("Resume display" if paused else "Pause display")

    def shutdown(self) -> None:
        if hasattr(self, "_bridge"):
            self._bridge.shutdown()


__all__ = ["PoseSimulationRosBridge", "PoseSimulationWidget"]
