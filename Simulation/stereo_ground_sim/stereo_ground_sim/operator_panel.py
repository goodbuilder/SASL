"""Read-only ROS/rqt research panel for the Simulation stereo pipeline.

Legacy numeric topics intentionally remain un-stamped.  The panel therefore
shows their contract frame and receive age separately from image Header time;
it never presents receive time as a sample timestamp.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque

from python_qt_binding.QtCore import QObject, QTimer, Qt, Signal
from python_qt_binding.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from python_qt_binding.QtWidgets import (
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QSplitter,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from qt_gui.plugin import Plugin

from stereo_ground_sim.simulation_ui_contract import (
    SOURCE_KEYS,
    STALE_TIMEOUT_S,
    UNSYNCHRONIZED,
    finite_float as contract_finite_float,
    fallback_transition,
    format_receive_age,
    format_status_vector,
    parse_px4_status,
    px4_contract_state,
    px4_overall_status,
    source_state,
    status_field,
    stale_display,
    synchronized_comparison_errors,
    transition_events,
)

try:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32, Float32MultiArray, Float64, Float64MultiArray, String
except Exception:  # pragma: no cover - rqt can still load the widget shell.
    rclpy = None
    qos_profile_sensor_data = None
    Image = Float32 = Float32MultiArray = Float64 = Float64MultiArray = String = None


CONTRACT_FRAME = "baseline_center_frd"
CONTRACT_UNIT = "m"
IMAGE_TOPICS = (("Left Camera", "/left_camera/detection_image"), ("Right Camera", "/right_camera/detection_image"))
POSITION_TOPICS = (
    ("ground_truth", "/ground_truth_position"),
    ("observed_raw", "/stereo/estimated_position"),
    ("analytic_raw", "/stereo/attitude_model/raw/estimated_position"),
    ("corrected", "/stereo/corrected_position"),
)
TEXT_TOPICS = (
    ("attitude_status", "/stereo/attitude_model/status"),
    ("observation_status", "/stereo/observation_status"),
    ("profile_status", "/stereo/profile_status"),
)
PX4_STATUS_TOPIC = "/simulation/px4/status"


def finite_float(value):
    if isinstance(value, (bool, str)):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def format_value(value, unit=""):
    value = finite_float(value)
    return "unavailable" if value is None else f"{value:.3f}{(' ' + unit) if unit else ''}"


def improvement(raw_error, corrected_error, *, synchronized=False):
    if not synchronized:
        return UNSYNCHRONIZED
    raw_error = finite_float(raw_error)
    corrected_error = finite_float(corrected_error)
    if raw_error is None or corrected_error is None or abs(raw_error) < 1.0e-12:
        return UNSYNCHRONIZED
    return f"{100.0 * (1.0 - abs(corrected_error) / abs(raw_error)):.1f}%"


def parse_status(value):
    try:
        parsed = json.loads(value) if isinstance(value, str) else value
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}
    return parsed if isinstance(parsed, dict) else {}


class PanelBridge(QObject):
    image = Signal(str, object)
    snapshot = Signal(object)
    plot = Signal(str, object)
    event = Signal(str)
    status = Signal(str)


class CameraView(QLabel):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self._title = title
        self._pixmap = None
        self._last_text = f"{title}\nWAITING — image unavailable"
        self.setMinimumSize(500, 280)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background: #202428; color: #d7dee5; font-weight: 600;")
        self.setText(self._last_text)

    def set_image(self, payload):
        width, height, encoding, step, data, stamp, received = payload
        qimage = self._to_qimage(width, height, encoding, step, data)
        age = time.monotonic() - received
        stamp_text = "unavailable" if stamp is None else f"{stamp:.6f} s"
        if qimage is None:
            self._last_text = f"{self._title}\nINVALID — unsupported image encoding: {encoding}"
            self.setText(self._last_text)
            return
        self._pixmap = QPixmap.fromImage(qimage)
        self._last_text = f"{self._title}\nVALID"
        self.setToolTip(f"Header stamp: {stamp_text}\nreceive age: {age:.3f} s")
        self._refresh_pixmap()

    def set_source_state(self, state, age=None):
        if state == "valid" and self._pixmap is not None:
            self._refresh_pixmap()
            return
        self.setPixmap(QPixmap())
        if state == "stale":
            age_text = "unavailable" if age is None else f"{age:.1f} s"
            self.setText(f"{self._title}\nSTALE — receive age: {age_text}")
        elif state == "invalid":
            self.setText(f"{self._title}\nINVALID — image unavailable")
        else:
            self.setText(f"{self._title}\nWAITING — image unavailable")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_pixmap()

    def _refresh_pixmap(self):
        if self._pixmap is not None:
            self.setPixmap(self._pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def _to_qimage(self, width, height, encoding, step, data):
        if width <= 0 or height <= 0:
            return None
        if encoding in ("rgb8", "bgr8"):
            channels, fmt = 3, QImage.Format_RGB888
        elif encoding in ("rgba8", "bgra8"):
            channels, fmt = 4, QImage.Format_RGBA8888
        elif encoding in ("mono8", "8UC1"):
            channels, fmt = 1, QImage.Format_Grayscale8
        else:
            return None
        min_step = width * channels
        if step < min_step:
            return None
        raw = bytes(data)
        packed = b"".join(raw[row * step : row * step + min_step] for row in range(height))
        if encoding in ("bgr8", "bgra8"):
            stride = channels
            packed = b"".join(
                bytes((*reversed(packed[start : start + 3]),)) if stride == 3 else bytes((packed[start + 2], packed[start + 1], packed[start], packed[start + 3]))
                for start in range(0, len(packed), stride)
            )
        return QImage(packed, width, height, min_step, fmt).copy()


class FrdPlotStack(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(700, 520)
        self.samples = {axis: {name: deque(maxlen=900) for name, _topic in POSITION_TOPICS} for axis, _index in (("Forward", 0), ("Right", 1), ("Down", 2))}

    def append(self, series_name, values):
        now = time.monotonic()
        for axis, index in (("Forward", 0), ("Right", 1), ("Down", 2)):
            value = finite_float(values[index]) if len(values) > index else None
            self.samples[axis][series_name].append((now, value))
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(250, 250, 250))
        margin_left, margin_right, margin_top, margin_bottom, gap = 72, 18, 34, 28, 22
        width = max(1, self.width() - margin_left - margin_right)
        pane_height = max(1, (self.height() - margin_top - margin_bottom - gap * 2) / 3)
        now, window_s = time.monotonic(), 30.0
        colors = {"ground_truth": QColor(32, 86, 255), "observed_raw": QColor(220, 32, 32), "analytic_raw": QColor(180, 100, 20), "corrected": QColor(0, 170, 170)}
        for pane, (axis, _index) in enumerate((("Forward", 0), ("Right", 1), ("Down", 2))):
            top, height, left = int(margin_top + pane * (pane_height + gap)), int(pane_height), margin_left
            right, bottom = left + width, int(top + height)
            finite_values = [v for name, _topic in POSITION_TOPICS for stamp, v in self.samples[axis][name] if now - stamp <= window_s and v is not None]
            ymin, ymax = ((min(finite_values), max(finite_values)) if finite_values else (-1.0, 1.0))
            if abs(ymax - ymin) < 1.0e-6:
                ymin, ymax = ymin - 1.0, ymax + 1.0
            pad = 0.08 * (ymax - ymin)
            ymin, ymax = ymin - pad, ymax + pad
            painter.setPen(QPen(QColor(210, 210, 210), 1)); painter.drawRect(left, top, width, height)
            painter.setPen(QPen(QColor(32, 32, 32), 1)); painter.setFont(QFont("Sans", 11, QFont.Bold))
            painter.drawText(left, top - 10, f"{axis} — {CONTRACT_FRAME} [m]")
            painter.setFont(QFont("Sans", 9)); painter.drawText(8, top + 14, f"{ymax:.1f}"); painter.drawText(8, bottom, f"{ymin:.1f}")
            for name, _topic in POSITION_TOPICS:
                points = [(stamp, value) for stamp, value in self.samples[axis][name] if now - stamp <= window_s and value is not None]
                painter.setPen(QPen(colors[name], 2))
                last = None
                for stamp, value in points:
                    x = int(right - (now - stamp) / window_s * width)
                    y = int(bottom - (value - ymin) / (ymax - ymin) * height)
                    if last is not None:
                        painter.drawLine(last[0], last[1], x, y)
                    last = (x, y)
            painter.setPen(QPen(QColor(32, 32, 32), 1)); painter.drawText(right - 430, top - 10, "receive-time view — GT / Observed Raw / Analytic Raw / Corrected")


def _label(text="unavailable"):
    result = QLabel(text)
    result.setWordWrap(True)
    result.setTextInteractionFlags(Qt.TextSelectableByMouse)
    return result


class OperatorPanel(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("OperatorPanel")
        self._ros_node = None
        self._spin_thread = None
        self._stop_event = threading.Event()
        self._bridge = PanelBridge()
        self._latest = {key: None for key in SOURCE_KEYS}
        self._latest.update({"ground_truth_range": None, "observed_disparity_px": None, "analytic_disparity_px": None, "analytic_vertical_disparity_px": None, "ray_quality": None, "pixels": None, "px4_status": None})
        self._received = {}
        self._source_states = {key: "waiting" for key in SOURCE_KEYS}
        self._last_fallback = None
        self._last_profile_status = None
        self._last_px4_status = None
        self._image_times = {"left": deque(maxlen=40), "right": deque(maxlen=40)}
        self._image_meta = {"left": {}, "right": {}}

        self._widget = QWidget()
        self._widget.setWindowTitle("Stereo Simulation Research Operator Panel")
        self._widget.setMinimumSize(1700, 950)
        root = QVBoxLayout(self._widget)
        root.setContentsMargins(8, 8, 8, 8)

        top_bar = QFrame()
        top_bar.setFrameShape(QFrame.StyledPanel)
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(4, 4, 4, 4)
        self._top_status_labels = {}
        for field in ("AUTHORITY", "GAZEBO MOTION", "PX4 CONNECTION", "ARM", "MODE", "MISSION PHASE", "GT SOURCE", "OVERALL STATUS"):
            block = QVBoxLayout()
            key_label = QLabel(field)
            key_label.setStyleSheet("font-size: 10px; font-weight: 700; color: #5d6872;")
            value_label = _label("unavailable")
            value_label.setStyleSheet("font-weight: 700; padding: 2px;")
            block.addWidget(key_label)
            block.addWidget(value_label)
            top_layout.addLayout(block, 1)
            self._top_status_labels[field] = value_label
        self._status = self._top_status_labels["OVERALL STATUS"]
        root.addWidget(top_bar, 1)

        body_splitter = QSplitter(Qt.Horizontal)
        left_widget = QWidget()
        left = QVBoxLayout(left_widget)
        left.setContentsMargins(2, 2, 2, 2)
        images = QHBoxLayout()
        self._left_view, self._right_view = CameraView("LEFT CAMERA"), CameraView("RIGHT CAMERA")
        self._left_meta = _label("/left_camera/detection_image\nresolution: unavailable | Header stamp: unavailable | receive age: unavailable")
        self._right_meta = _label("/right_camera/detection_image\nresolution: unavailable | Header stamp: unavailable | receive age: unavailable")
        for view, meta in ((self._left_view, self._left_meta), (self._right_view, self._right_meta)):
            meta.setStyleSheet("font-size: 10px; color: #4f5b66;")
            meta.setMinimumHeight(52)
        left_column, right_column = QVBoxLayout(), QVBoxLayout()
        left_column.addWidget(self._left_view, 1); left_column.addWidget(self._left_meta)
        right_column.addWidget(self._right_view, 1); right_column.addWidget(self._right_meta)
        images.addLayout(left_column, 1); images.addLayout(right_column, 1)
        left.addLayout(images, 4)
        self._source_status = _label("Ground Truth source: unavailable\nMatched entity: unavailable\nGround Truth fallback: unavailable\nContract frame: baseline_center_frd [m]\nMessage-level frame: unavailable\nMessage sample timestamp: unavailable\nReceive age: unavailable")
        self._source_status.setStyleSheet("font-size: 8px;")
        self._source_status.setMinimumHeight(0)
        body_splitter.addWidget(left_widget)

        self._primary = QGroupBox("PRIMARY RESULT — read-only")
        primary_layout = QVBoxLayout(self._primary)
        primary_layout.setContentsMargins(4, 2, 4, 2)
        primary_layout.setSpacing(1)

        def compact_label(text="unavailable", *, value=False):
            result = _label(text)
            result.setStyleSheet("font-size: 8px;" + (" font-weight: 600;" if value else ""))
            result.setMinimumHeight(0)
            result.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            return result

        def compact_group(title):
            box = QGroupBox(title)
            box.setStyleSheet("QGroupBox { font-size: 10px; font-weight: 600; margin-top: 5px; } QGroupBox::title { subcontrol-origin: margin; left: 4px; padding: 0 2px; }")
            grid = QGridLayout(box)
            grid.setContentsMargins(4, 10, 4, 1)
            grid.setHorizontalSpacing(7)
            grid.setVerticalSpacing(0)
            return box, grid

        def add_pair(grid, row, key, title, other_key=None, other_title=None, label_map=None):
            label_map = label_map or (self._authority_labels if key in self._authority_labels else self._px4_labels)
            grid.addWidget(compact_label(title), row, 0)
            grid.addWidget(label_map[key], row, 1)
            if other_key is not None:
                grid.addWidget(compact_label(other_title), row, 2)
                grid.addWidget(label_map[other_key], row, 3)

        authority_box, authority_grid = compact_group("FLIGHT AUTHORITY")
        self._authority_labels = {key: compact_label(value=True) for key in ("control", "gazebo", "flight_authority", "fly_target", "mission_profile", "ground_truth", "frame")}
        add_pair(authority_grid, 0, "control", "Control authority", "gazebo", "Gazebo direct motion")
        add_pair(authority_grid, 1, "flight_authority", "flight_authority", "fly_target", "fly_target_in_gazebo")
        add_pair(authority_grid, 2, "mission_profile", "mission_profile", "ground_truth", "Ground Truth provider")
        authority_grid.addWidget(compact_label("Output frame"), 3, 0)
        authority_grid.addWidget(self._authority_labels["frame"], 3, 1, 1, 3)
        primary_layout.addWidget(authority_box)

        px4_box, px4_grid = compact_group("PX4 FLIGHT STATE")
        self._px4_labels = {key: compact_label(value=True) for key in ("heartbeat", "armed", "mode", "failsafe", "phase", "local_ned", "velocity_ned", "sample_timestamp")}
        add_pair(px4_grid, 0, "heartbeat", "Heartbeat", "armed", "Armed")
        add_pair(px4_grid, 1, "mode", "Navigation mode", "failsafe", "Failsafe")
        add_pair(px4_grid, 2, "phase", "Mission phase", "sample_timestamp", "PX4 sample timestamp")
        px4_grid.addWidget(compact_label("PX4 LOCAL NED — AUXILIARY, NOT GROUND TRUTH [m]"), 3, 0)
        px4_grid.addWidget(self._px4_labels["local_ned"], 3, 1, 1, 3)
        px4_grid.addWidget(compact_label("PX4 velocity NED — AUXILIARY [m/s]"), 4, 0)
        px4_grid.addWidget(self._px4_labels["velocity_ned"], 4, 1, 1, 3)
        primary_layout.addWidget(px4_box)

        self._range_labels = {key: compact_label(value=True) for key in ("truth", "observed_raw", "analytic_raw", "corrected", "raw_error", "corrected_error", "improvement")}
        range_box, range_grid = compact_group("RANGE & ERROR")
        range_grid.addWidget(compact_label("Ground Truth / Observed Raw range [m]"), 0, 0)
        range_grid.addWidget(self._range_labels["truth"], 0, 1)
        range_grid.addWidget(self._range_labels["observed_raw"], 0, 3)
        range_grid.addWidget(compact_label("Analytic Raw / Corrected range [m]"), 1, 0)
        range_grid.addWidget(self._range_labels["analytic_raw"], 1, 1)
        range_grid.addWidget(self._range_labels["corrected"], 1, 3)
        range_grid.addWidget(compact_label("Raw error / Corrected error / Improvement"), 2, 0)
        range_grid.addWidget(self._range_labels["raw_error"], 2, 1)
        range_grid.addWidget(self._range_labels["corrected_error"], 2, 2)
        range_grid.addWidget(self._range_labels["improvement"], 2, 3)
        primary_layout.addWidget(range_box)
        self._position_labels = {}
        self._position_state_labels, self._position_age_labels = {}, {}
        position_box = QGroupBox("POSITION — baseline_center_frd [m]")
        position_grid = QGridLayout(position_box)
        position_grid.setContentsMargins(4, 10, 4, 1)
        position_grid.setHorizontalSpacing(4)
        position_grid.setVerticalSpacing(0)
        for col, title in enumerate(("", "Forward", "Right", "Down", "Range", "State", "Receive age")):
            position_grid.addWidget(compact_label(title), 0, col)
        for row, name in enumerate(("Ground Truth", "Observed/BBox Raw", "Analytic Attitude Raw", "Corrected"), 1):
            position_grid.addWidget(compact_label(name), row, 0)
            self._position_labels[name] = []
            for col in range(4):
                value = compact_label(value=True)
                self._position_labels[name].append(value)
                position_grid.addWidget(value, row, col + 1)
            self._position_state_labels[name] = compact_label(value=True)
            self._position_age_labels[name] = compact_label(value=True)
            position_grid.addWidget(self._position_state_labels[name], row, 5)
            position_grid.addWidget(self._position_age_labels[name], row, 6)
        primary_layout.addWidget(position_box)
        self._geometry_labels = {}
        geometry, geometry_grid = compact_group("GEOMETRY QUALITY")
        for key in ("observed_disparity", "analytic_disparity", "analytic_vertical_disparity", "ray_gap", "intersection_angle", "status", "profile"):
            self._geometry_labels[key] = compact_label(value=True)
        add_pair(geometry_grid, 0, "observed_disparity", "Observed/BBox disparity [px]", "analytic_disparity", "Analytic disparity [px]", self._geometry_labels)
        add_pair(geometry_grid, 1, "analytic_vertical_disparity", "Analytic vertical disparity [px]", "ray_gap", "Analytic ray gap [m]", self._geometry_labels)
        add_pair(geometry_grid, 2, "intersection_angle", "Intersection angle [deg]", "status", "Triangulation status", self._geometry_labels)
        geometry_grid.addWidget(compact_label("Profile status"), 3, 0)
        geometry_grid.addWidget(self._geometry_labels["profile"], 3, 1, 1, 3)
        primary_layout.addWidget(geometry)
        source_box, source_grid = compact_group("SOURCE & TIMING")
        source_grid.addWidget(compact_label("Ground Truth source / Matched entity"), 0, 0)
        source_grid.addWidget(self._source_status, 0, 1, 1, 3)
        source_grid.addWidget(compact_label("Frame / sample timestamp"), 1, 0)
        source_grid.addWidget(compact_label("Message-level frame: unavailable | Message sample timestamp: unavailable"), 1, 1, 1, 3)
        primary_layout.addWidget(source_box)
        body_splitter.addWidget(self._primary)
        body_splitter.setStretchFactor(0, 64)
        body_splitter.setStretchFactor(1, 36)
        root.addWidget(body_splitter, 6)

        self._tabs = QTabWidget()
        self._tabs.addTab(self._range_tab(), "Flight Timeline")
        self._plot_stack = FrdPlotStack()
        self._tabs.addTab(self._plot_stack, "FRD")
        self._tabs.addTab(self._geometry_tab(), "Geometry")
        self._tabs.addTab(self._attitude_tab(), "Attitude")
        self._event_log = QPlainTextEdit(); self._event_log.setReadOnly(True); self._tabs.addTab(self._event_log, "Event Log")
        root.addWidget(self._tabs, 3)

        context.add_widget(self._widget)
        self._bridge.image.connect(self._handle_image)
        self._bridge.snapshot.connect(self._render_snapshot)
        self._bridge.plot.connect(self._plot_stack.append)
        self._bridge.event.connect(self._append_event)
        self._bridge.status.connect(self._status.setText)
        self._init_ros()

    def _range_tab(self):
        widget = QWidget(); layout = QFormLayout(widget)
        layout.addRow("Timeline", _label("receive-time view — no common sample timestamp"))
        layout.addRow("Ground Truth Forward range [m]", _label("shown in Primary Result"))
        layout.addRow("Ground Truth altitude [m]", _label("baseline_center_frd Down component; unavailable until received"))
        layout.addRow("PX4 auxiliary altitude [m]", _label("PX4 Local NED Down; not Ground Truth"))
        layout.addRow("PX4 speed [m/s]", _label("read-only telemetry; unavailable until received"))
        layout.addRow("Raw/Corrected error", _label(UNSYNCHRONIZED))
        layout.addRow("Improvement", _label(UNSYNCHRONIZED))
        return widget

    def _geometry_tab(self):
        widget = QWidget(); layout = QFormLayout(widget)
        layout.addRow("Contract frame", _label("baseline_center_frd")); layout.addRow("Message-level frame", _label("unavailable — legacy array topics")); layout.addRow("Sample timestamp", _label("unavailable — no common stamped snapshot")); layout.addRow("Receive age", _label("shown separately; not sample time")); return widget

    def _attitude_tab(self):
        widget = QWidget(); layout = QFormLayout(widget); self._attitude_labels = {}
        for key in ("left_roll_deg", "left_pitch_deg", "left_yaw_deg", "right_roll_deg", "right_pitch_deg", "right_yaw_deg", "common_roll_deg", "common_pitch_deg", "common_yaw_deg", "relative_roll_deg", "relative_pitch_deg", "relative_yaw_deg"):
            self._attitude_labels[key] = _label(); layout.addRow(key.replace("_", " "), self._attitude_labels[key])
        layout.addRow("Attitude status", _label("read-only values from /stereo/attitude_model/status")); return widget

    def _handle_image(self, name, payload):
        view = self._left_view if name == "left" else self._right_view
        view.set_image(payload)
        width, height, _encoding, _step, _data, stamp, received = payload
        self._image_times[name].append(received)
        self._image_meta[name].update(width=width, height=height, stamp=stamp, received=received)
        self._update_image_meta(name)

    def _update_image_meta(self, name, snapshot=None):
        snapshot = snapshot or self._latest
        data = self._image_meta[name]
        observation = snapshot.get("observation_status") or {}
        attitude = snapshot.get("attitude_status") or {}
        observed_prefix = "bbox_left" if name == "left" else "bbox_right"
        ideal_prefix = "actual_left" if name == "left" else "actual_right"
        observed = (observation.get(f"{observed_prefix}_u_px"), observation.get(f"{observed_prefix}_v_px"))
        ideal = (attitude.get(f"{ideal_prefix}_u_px"), attitude.get(f"{ideal_prefix}_v_px"))
        observed_text = "unavailable" if any(finite_float(value) is None for value in observed) else f"({float(observed[0]):.2f}, {float(observed[1]):.2f})"
        ideal_text = "unavailable" if any(finite_float(value) is None for value in ideal) else f"({float(ideal[0]):.2f}, {float(ideal[1]):.2f})"
        delta_text = "unavailable" if observed_text == "unavailable" or ideal_text == "unavailable" else f"({float(observed[0]) - float(ideal[0]):.2f}, {float(observed[1]) - float(ideal[1]):.2f})"
        history = self._image_times[name]
        fps = 0.0
        if len(history) >= 2 and history[-1] > history[0]:
            fps = (len(history) - 1) / (history[-1] - history[0])
        key = f"{name}_image"
        age = None if key not in self._received else time.monotonic() - self._received[key]
        stamp_text = "unavailable" if data.get("stamp") is None else f"{data['stamp']:.6f} s"
        source = observation.get("bbox_observation_source") or observation.get("bbox_configured_source") or "unavailable"
        meta = (f"/{name}_camera/detection_image\n"
                f"resolution: {data.get('width', 'unavailable')}×{data.get('height', 'unavailable')} | Header stamp: {stamp_text} | receive age: {('unavailable' if age is None else f'{age:.1f} s')} | measured FPS: {fps:.1f}\n"
                f"observed centroid [px]: {observed_text} | ideal pixel [px]: {ideal_text} | Δu, Δv [px]: {delta_text} | observation source: {source}")
        (self._left_meta if name == "left" else self._right_meta).setText(meta)

    def _append_event(self, message):
        self._event_log.appendPlainText(f"{time.monotonic():.3f} receive-time — {message}")
        while self._event_log.blockCount() > 500:
            cursor = self._event_log.textCursor(); cursor.movePosition(cursor.Start); cursor.select(cursor.LineUnderCursor); cursor.removeSelectedText(); cursor.deleteChar()

    def _record(self, key, value):
        self._latest[key] = value
        now = time.monotonic()
        self._received[key] = now
        previous = dict(self._source_states)
        self._source_states[key] = source_state(now, value, now, STALE_TIMEOUT_S)
        for event in transition_events(previous, self._source_states):
            self._bridge.event.emit(event)
        self._bridge.snapshot.emit(dict(self._latest))
        if key in {"ground_truth", "observed_raw", "analytic_raw", "corrected"}:
            self._bridge.plot.emit(key, value or [])

    def _position_cb(self, key, msg):
        values = [finite_float(value) for value in list(msg.data)]
        self._record(key, values if len(values) >= 4 else None)

    def _scalar_cb(self, key, msg):
        self._record(key, finite_float(msg.data))

    def _array_cb(self, key, msg):
        self._record(key, [finite_float(value) for value in list(msg.data)])

    def _json_cb(self, key, msg):
        self._record(key, parse_px4_status(msg.data) if key == "px4_status" else parse_status(msg.data))

    def _image_payload(self, msg):
        stamp = None
        if getattr(msg, "header", None) is not None:
            sec = finite_float(msg.header.stamp.sec)
            nanosec = finite_float(msg.header.stamp.nanosec)
            stamp = None if sec is None or nanosec is None else sec + nanosec * 1.0e-9
        return (int(msg.width), int(msg.height), str(msg.encoding), int(msg.step), bytes(msg.data), stamp, time.monotonic())

    def _init_ros(self):
        if rclpy is None or Image is None:
            self._status.setText("ERROR — ROS imports unavailable; operator panel cannot subscribe")
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._ros_node = rclpy.create_node("operator_panel")
            for name, topic in IMAGE_TOPICS:
                self._ros_node.create_subscription(Image, topic, lambda msg, name=name.lower().split()[0]: self._image_cb(name, msg), qos_profile_sensor_data)
            for key, topic in POSITION_TOPICS:
                message_type = Float64MultiArray if key == "analytic_raw" else Float32MultiArray
                self._ros_node.create_subscription(message_type, topic, lambda msg, key=key: self._position_cb(key, msg), 10)
            self._ros_node.create_subscription(Float32, "/ground_truth", lambda msg: self._scalar_cb("ground_truth_range", msg), 10)
            self._ros_node.create_subscription(Float32, "/stereo/disparity", lambda msg: self._scalar_cb("observed_disparity_px", msg), 10)
            self._ros_node.create_subscription(Float64, "/stereo/attitude_model/actual/disparity", lambda msg: self._scalar_cb("analytic_disparity_px", msg), 10)
            self._ros_node.create_subscription(Float64MultiArray, "/stereo/attitude_model/actual/pixels", lambda msg: self._array_cb("pixels", msg), 10)
            self._ros_node.create_subscription(Float64MultiArray, "/stereo/attitude_model/raw/ray_quality", lambda msg: self._array_cb("ray_quality", msg), 10)
            for key, topic in TEXT_TOPICS:
                self._ros_node.create_subscription(String, topic, lambda msg, key=key: self._json_cb(key, msg), 10)
            self._ros_node.create_subscription(String, PX4_STATUS_TOPIC, lambda msg: self._json_cb("px4_status", msg), 10)
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True); self._spin_thread.start()
            self._stale_timer = QTimer(self._widget)
            self._stale_timer.timeout.connect(self._refresh_source_states)
            self._stale_timer.start(200)
        except Exception as exc:
            self._ros_node = None; self._status.setText(f"ERROR — Operator panel subscribe failed: {exc}")

    def _image_cb(self, name, msg):
        key = f"{name}_image"
        self._record(key, {"received": True})
        self._bridge.image.emit(name, self._image_payload(msg))

    def _render_px4_status(self, status):
        status = status if isinstance(status, dict) else {}
        state = self._source_states.get("px4_status", "waiting")
        age = None if "px4_status" not in self._received else time.monotonic() - self._received["px4_status"]
        contract = px4_contract_state(status)
        if contract == "safe":
            authority_text, gazebo_text = "PX4 SITL ONLY", "BLOCKED"
        elif contract == "violation":
            authority_text, gazebo_text = "ERROR — CONTRACT VIOLATION", "ERROR — ENABLED"
        else:
            authority_text, gazebo_text = "unavailable", "unavailable"
        connected = status.get("px4_connected") is True and state == "valid"
        connection_text = "CONNECTED" if connected else ("STALE" if state == "stale" else "unavailable")
        armed_text = "ARMED" if status.get("armed") is True and connected else ("DISARMED" if connected else "unavailable")
        mode = status_field(status, "navigation_mode")
        phase = status_field(status, "mission_phase")
        observation = self._latest.get("observation_status") or {}
        attitude = self._latest.get("attitude_status") or {}
        source = attitude.get("oracle_source_role") or observation.get("oracle_source_role") or "unavailable"
        source_text = str(source)
        overall = px4_overall_status(status, self._source_states)
        if state == "stale":
            overall = f"DEGRADED — PX4 status stale; receive age: {format_receive_age(self._received.get('px4_status'), time.monotonic())}"
        for key, value in (("AUTHORITY", authority_text), ("GAZEBO MOTION", gazebo_text), ("PX4 CONNECTION", connection_text), ("ARM", armed_text), ("MODE", str(mode)), ("MISSION PHASE", str(phase)), ("GT SOURCE", source_text), ("OVERALL STATUS", overall)):
            self._top_status_labels[key].setText(value)
        for key, value in (("control", authority_text), ("gazebo", gazebo_text), ("flight_authority", status_field(status, "flight_authority")), ("fly_target", "false" if contract == "safe" else "unavailable"), ("mission_profile", "manual" if contract == "safe" else "unavailable"), ("ground_truth", "Gazebo target_reference_link"), ("frame", CONTRACT_FRAME)):
            self._authority_labels[key].setText(str(value))
        self._px4_labels["heartbeat"].setText(connection_text)
        self._px4_labels["armed"].setText(armed_text)
        self._px4_labels["mode"].setText(str(mode))
        self._px4_labels["failsafe"].setText("TRUE" if status.get("failsafe") is True else ("FALSE" if connected else "unavailable"))
        self._px4_labels["phase"].setText(str(phase))
        self._px4_labels["local_ned"].setText(format_status_vector(status.get("local_ned_m"), ("north", "east", "down")))
        self._px4_labels["velocity_ned"].setText(format_status_vector(status.get("velocity_ned_mps"), ("north", "east", "down")))
        sample_timestamp = status_field(status, "sample_timestamp_us")
        self._px4_labels["sample_timestamp"].setText("unavailable" if sample_timestamp == "unavailable" else f"{sample_timestamp} µs")
        previous = self._last_px4_status
        if isinstance(previous, dict) and isinstance(status, dict):
            transitions = (("px4_connected", "PX4 connected"), ("armed", "PX4 armed"), ("navigation_mode", "PX4 navigation mode"), ("mission_phase", "PX4 mission phase"), ("failsafe", "PX4 failsafe"))
            for key, label in transitions:
                if previous.get(key) != status.get(key):
                    self._bridge.event.emit(f"{label}: {previous.get(key)} → {status.get(key)}")
        self._last_px4_status = dict(status) if status else None

    def _render_camera_metadata(self, snapshot):
        for name in ("left", "right"):
            self._update_image_meta(name, snapshot)

    def _render_snapshot(self, snapshot):
        self._render_px4_status(snapshot.get("px4_status"))
        names = {"ground_truth": "Ground Truth", "observed_raw": "Observed/BBox Raw", "analytic_raw": "Analytic Attitude Raw", "corrected": "Corrected"}
        ranges = {}
        for key, name in names.items():
            values = snapshot.get(key); ranges[key] = values[3] if values and len(values) > 3 else None
            for index, label in enumerate(self._position_labels[name][:3]):
                label.setText(format_value(values[index] if values and len(values) > index else None, "m"))
            range_value = values[3] if values and len(values) > 3 else None
            self._position_labels[name][3].setText(format_value(range_value, "m"))
        truth_range = snapshot.get("ground_truth_range") or ranges["ground_truth"]
        self._range_labels["truth"].setText(format_value(truth_range, "m")); self._range_labels["observed_raw"].setText(format_value(ranges["observed_raw"], "m")); self._range_labels["analytic_raw"].setText(format_value(ranges["analytic_raw"], "m")); self._range_labels["corrected"].setText(format_value(ranges["corrected"], "m"))
        comparisons = synchronized_comparison_errors(truth_range, ranges["observed_raw"], ranges["analytic_raw"], ranges["corrected"], synchronized=False)
        self._range_labels["raw_error"].setText(str(comparisons["observed_error"])); self._range_labels["corrected_error"].setText(str(comparisons["corrected_error"])); self._range_labels["improvement"].setText(str(comparisons["improvement"]))
        ray = snapshot.get("ray_quality") or []; pixels = snapshot.get("pixels") or []
        self._geometry_labels["observed_disparity"].setText(format_value(snapshot.get("observed_disparity_px"), "px")); self._geometry_labels["analytic_disparity"].setText(format_value(snapshot.get("analytic_disparity_px"), "px")); self._geometry_labels["analytic_vertical_disparity"].setText(format_value(pixels[5] if len(pixels) > 5 else None, "px")); self._geometry_labels["ray_gap"].setText(format_value(ray[0] if len(ray) > 0 else None, "m")); self._geometry_labels["intersection_angle"].setText(format_value(math.degrees(ray[1]) if len(ray) > 1 and ray[1] is not None else None, "deg"))
        observation = snapshot.get("observation_status") or {}; attitude = snapshot.get("attitude_status") or {}; profile = snapshot.get("profile_status") or {}
        self._geometry_labels["status"].setText(str(observation.get("status") or attitude.get("raw_triangulation_status") or "unavailable")); self._geometry_labels["profile"].setText(str(profile.get("profile_id") or profile.get("status") or "unavailable"))
        source = attitude.get("oracle_source_role") or observation.get("oracle_source_role") or "unavailable"; matched = attitude.get("oracle_matched_entity_name") or observation.get("oracle_matched_entity_name") or "unavailable"; fallback = attitude.get("oracle_fallback_used", observation.get("oracle_fallback_used", "unavailable")); receive = self._received.get("attitude_status") or self._received.get("observation_status"); age = "unavailable" if receive is None else f"{time.monotonic() - receive:.3f} s"
        self._source_status.setText(f"Ground Truth source: {source}\nMatched entity: {matched}\nGround Truth fallback: {fallback}\nContract frame: {CONTRACT_FRAME} [{CONTRACT_UNIT}]\nMessage-level frame: unavailable\nMessage sample timestamp: unavailable\nReceive age: {age}")
        for event in fallback_transition(self._last_fallback, {"fallback": fallback, "source": source}):
            self._bridge.event.emit(event)
        self._last_fallback = {"fallback": fallback, "source": source}
        profile_status = profile.get("status") or profile.get("profile_mode") or profile.get("profile_id")
        if self._last_profile_status is not None and profile_status != self._last_profile_status:
            self._bridge.event.emit(f"Profile status changed: {self._last_profile_status} → {profile_status}")
        self._last_profile_status = profile_status
        for key, label in self._attitude_labels.items(): label.setText(format_value(attitude.get(key), "deg"))
        self._render_camera_metadata(snapshot)

    def _refresh_source_states(self):
        now = time.monotonic(); previous = dict(self._source_states)
        for key in SOURCE_KEYS:
            self._source_states[key] = source_state(self._received.get(key), self._latest.get(key), now, STALE_TIMEOUT_S)
        for event in transition_events(previous, self._source_states):
            self._bridge.event.emit(event)
        stale = [key for key, state in self._source_states.items() if state == "stale"]
        state_text = ", ".join(f"{key}={state}" for key, state in self._source_states.items())
        current = self._source_status.text().split("\n")[:5]
        self._source_status.setText("\n".join(current + [f"Source states: {state_text}"]))
        self._render_snapshot(dict(self._latest))
        for name, view in (("left", self._left_view), ("right", self._right_view)):
            key = f"{name}_image"
            age = None if key not in self._received else now - self._received[key]
            view.set_source_state(self._source_states[key], age)
        position_rows = (("ground_truth", "Ground Truth"), ("observed_raw", "Observed/BBox Raw"), ("analytic_raw", "Analytic Attitude Raw"), ("corrected", "Corrected"))
        for source, row_name in position_rows:
            values = self._latest.get(source) or []
            state = self._source_states[source]
            age = now - self._received[source] if source in self._received else None
            for index, label in enumerate(self._position_labels[row_name][:3]):
                label.setText(stale_display(values[index] if len(values) > index else None, state, unit="m", age=age))
            range_value = values[3] if len(values) > 3 else None
            self._position_labels[row_name][3].setText(stale_display(range_value, state, unit="m", age=age))
            self._position_state_labels[row_name].setText(state.upper())
            self._position_age_labels[row_name].setText(format_receive_age(self._received.get(source), now))
        for key, label_key, unit in (("ground_truth_range", "truth", "m"), ("observed_disparity_px", "observed_disparity", "px"), ("analytic_disparity_px", "analytic_disparity", "px")):
            age = now - self._received[key] if key in self._received else None
            label = self._range_labels[label_key] if label_key in self._range_labels else self._geometry_labels[label_key]
            label.setText(stale_display(self._latest.get(key), self._source_states[key], unit=unit, age=age))
        for key, label_key, unit in (("pixels", "analytic_vertical_disparity", "px"), ("ray_quality", "ray_gap", "m")):
            values = self._latest.get(key) or []
            value = values[5] if key == "pixels" and len(values) > 5 else (values[0] if key == "ray_quality" and values else None)
            age = now - self._received[key] if key in self._received else None
            self._geometry_labels[label_key].setText(stale_display(value, self._source_states[key], unit=unit, age=age))
        ray_age = now - self._received["ray_quality"] if "ray_quality" in self._received else None
        ray_state = self._source_states["ray_quality"]
        ray_values = self._latest.get("ray_quality") or []
        self._geometry_labels["intersection_angle"].setText(stale_display(math.degrees(ray_values[1]) if len(ray_values) > 1 and ray_values[1] is not None else None, ray_state, unit="deg", age=ray_age))
        if stale:
            self._status.setText("STALE — " + ", ".join(stale) + "; receive age view")

    def _spin_loop(self):
        while self._ros_node is not None and not self._stop_event.is_set():
            try: rclpy.spin_once(self._ros_node, timeout_sec=0.1)
            except Exception as exc: self._bridge.event.emit(f"spin failed: {exc}"); return

    def shutdown_plugin(self):
        self._stop_event.set()
        if self._spin_thread is not None: self._spin_thread.join(timeout=1.0); self._spin_thread = None
        if self._ros_node is not None:
            try: self._ros_node.destroy_node()
            except Exception: pass
            self._ros_node = None


__all__ = ["CONTRACT_FRAME", "CONTRACT_UNIT", "finite_float", "format_value", "improvement", "parse_status", "CameraView", "FrdPlotStack", "OperatorPanel"]
