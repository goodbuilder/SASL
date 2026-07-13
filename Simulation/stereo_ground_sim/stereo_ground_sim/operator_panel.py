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
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
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
    source_state,
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
        self.setMinimumSize(500, 280)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background: #202428; color: #d7dee5; font-weight: 600;")
        self.setText(f"{title}\nWAITING — image unavailable")

    def set_image(self, payload):
        width, height, encoding, step, data, stamp, received = payload
        qimage = self._to_qimage(width, height, encoding, step, data)
        age = time.monotonic() - received
        stamp_text = "unavailable" if stamp is None else f"{stamp:.6f} s"
        if qimage is None:
            self.setText(f"{self._title}\nINVALID — unsupported image encoding: {encoding}")
            return
        self._pixmap = QPixmap.fromImage(qimage)
        self.setToolTip(f"Header stamp: {stamp_text}\nreceive age: {age:.3f} s")
        self._refresh_pixmap()

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
            painter.setPen(QPen(QColor(32, 32, 32), 1)); painter.drawText(right - 370, top - 10, "GT / Observed Raw / Analytic Raw / Corrected")


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
        self._latest.update({"ground_truth_range": None, "observed_disparity_px": None, "analytic_disparity_px": None, "analytic_vertical_disparity_px": None, "ray_quality": None, "pixels": None})
        self._received = {}
        self._source_states = {key: "waiting" for key in SOURCE_KEYS}
        self._last_fallback = None
        self._last_profile_status = None

        self._widget = QWidget()
        self._widget.setWindowTitle("Stereo Simulation Research Operator Panel")
        self._widget.setMinimumSize(1500, 900)
        root = QVBoxLayout(self._widget)
        self._status = _label("WAITING — ROS data unavailable")
        self._status.setStyleSheet("font-weight: 700; padding: 5px;")
        root.addWidget(self._status)

        body = QHBoxLayout()
        left = QVBoxLayout()
        images = QHBoxLayout()
        self._left_view, self._right_view = CameraView("LEFT CAMERA"), CameraView("RIGHT CAMERA")
        images.addWidget(self._left_view, 1); images.addWidget(self._right_view, 1)
        left.addLayout(images, 4)
        self._source_status = _label("Ground Truth source: unavailable\nMessage-level frame: unavailable\nMessage sample timestamp: unavailable\nReceive age: unavailable")
        left.addWidget(self._source_status)
        body.addLayout(left, 7)

        self._primary = QGroupBox("PRIMARY RESULT — read-only")
        primary_layout = QVBoxLayout(self._primary)
        self._range_labels = {}
        range_form = QFormLayout()
        for key, title in (("truth", "Ground Truth range [m]"), ("observed_raw", "Observed/BBox Raw range [m]"), ("analytic_raw", "Analytic Attitude Raw range [m]"), ("corrected", "Corrected range [m]"), ("raw_error", "Raw range error [m]"), ("corrected_error", "Corrected range error [m]"), ("improvement", "Improvement")):
            self._range_labels[key] = _label()
            range_form.addRow(title, self._range_labels[key])
        primary_layout.addLayout(range_form)
        self._position_labels = {}
        position_box = QGroupBox("3D POSITION — Contract frame: baseline_center_frd [m]")
        position_grid = QGridLayout(position_box)
        for col, title in enumerate(("", "Forward", "Right", "Down", "norm")):
            position_grid.addWidget(_label(title), 0, col)
        for row, name in enumerate(("Ground Truth", "Observed/BBox Raw", "Analytic Attitude Raw", "Corrected"), 1):
            position_grid.addWidget(_label(name), row, 0)
            self._position_labels[name] = []
            for col in range(4):
                value = _label()
                self._position_labels[name].append(value)
                position_grid.addWidget(value, row, col + 1)
        primary_layout.addWidget(position_box)
        self._geometry_labels = {}
        geometry = QGroupBox("GEOMETRY QUALITY")
        geometry_form = QFormLayout(geometry)
        for key, title in (("observed_disparity", "Observed/BBox disparity [px]"), ("analytic_disparity", "Analytic disparity [px]"), ("analytic_vertical_disparity", "Analytic vertical disparity [px]"), ("ray_gap", "Analytic ray gap [m]"), ("intersection_angle", "Analytic intersection angle [deg]"), ("status", "Observation status"), ("profile", "Profile status")):
            self._geometry_labels[key] = _label()
            geometry_form.addRow(title, self._geometry_labels[key])
        primary_layout.addWidget(geometry)
        body.addWidget(self._primary, 5)
        root.addLayout(body, 4)

        self._tabs = QTabWidget()
        self._tabs.addTab(self._range_tab(), "Range & Error")
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
        layout.addRow("Contract frame", _label("baseline_center_frd"))
        layout.addRow("Ground Truth range", _label("shown in Primary Result"))
        layout.addRow("Raw/Corrected range error", _label("shown in Primary Result"))
        layout.addRow("Improvement", _label("N/A when samples are not synchronized"))
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
        (self._left_view if name == "left" else self._right_view).set_image(payload)

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
        self._record(key, parse_status(msg.data))

    def _image_payload(self, msg):
        stamp = None
        if getattr(msg, "header", None) is not None:
            stamp = finite_float(msg.header.stamp.sec) + finite_float(msg.header.stamp.nanosec) * 1.0e-9
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

    def _render_snapshot(self, snapshot):
        names = {"ground_truth": "Ground Truth", "observed_raw": "Observed/BBox Raw", "analytic_raw": "Analytic Attitude Raw", "corrected": "Corrected"}
        ranges = {}
        for key, name in names.items():
            values = snapshot.get(key); ranges[key] = values[3] if values and len(values) > 3 else None
            for index, label in enumerate(self._position_labels[name]):
                label.setText(format_value(values[index] if values and len(values) > index else None, "m"))
            norm = math.sqrt(sum(float(v) ** 2 for v in values[:3])) if values and all(v is not None for v in values[:3]) else None
            self._position_labels[name][3].setText(format_value(norm, "m"))
        truth_range = snapshot.get("ground_truth_range") or ranges["ground_truth"]
        self._range_labels["truth"].setText(format_value(truth_range, "m")); self._range_labels["observed_raw"].setText(format_value(ranges["observed_raw"], "m")); self._range_labels["analytic_raw"].setText(format_value(ranges["analytic_raw"], "m")); self._range_labels["corrected"].setText(format_value(ranges["corrected"], "m"))
        comparisons = synchronized_comparison_errors(truth_range, ranges["observed_raw"], ranges["analytic_raw"], ranges["corrected"], synchronized=False)
        self._range_labels["raw_error"].setText(str(comparisons["observed_error"])); self._range_labels["corrected_error"].setText(str(comparisons["corrected_error"])); self._range_labels["improvement"].setText(str(comparisons["improvement"]))
        ray = snapshot.get("ray_quality") or []; pixels = snapshot.get("pixels") or []
        self._geometry_labels["observed_disparity"].setText(format_value(snapshot.get("observed_disparity_px"), "px")); self._geometry_labels["analytic_disparity"].setText(format_value(snapshot.get("analytic_disparity_px"), "px")); self._geometry_labels["analytic_vertical_disparity"].setText(format_value(pixels[5] if len(pixels) > 5 else None, "px")); self._geometry_labels["ray_gap"].setText(format_value(ray[0] if len(ray) > 0 else None, "m")); self._geometry_labels["intersection_angle"].setText(format_value(math.degrees(ray[1]) if len(ray) > 1 and ray[1] is not None else None, "deg"))
        observation = snapshot.get("observation_status") or {}; attitude = snapshot.get("attitude_status") or {}; profile = snapshot.get("profile_status") or {}
        self._geometry_labels["status"].setText(str(observation.get("status") or attitude.get("raw_triangulation_status") or "unavailable")); self._geometry_labels["profile"].setText(str(profile.get("profile_id") or profile.get("status") or "unavailable"))
        source = attitude.get("oracle_source_role") or observation.get("oracle_source_role") or "unavailable"; fallback = attitude.get("oracle_fallback_used", observation.get("oracle_fallback_used", "unavailable")); receive = self._received.get("attitude_status") or self._received.get("observation_status"); age = "unavailable" if receive is None else f"{time.monotonic() - receive:.3f} s"
        self._source_status.setText(f"Ground Truth source: {source}\nGround Truth fallback: {fallback}\nContract frame: {CONTRACT_FRAME}\nMessage-level frame: unavailable\nMessage sample timestamp: unavailable\nReceive age: {age}")
        for event in fallback_transition(self._last_fallback, {"fallback": fallback, "source": source}):
            self._bridge.event.emit(event)
        self._last_fallback = {"fallback": fallback, "source": source}
        profile_status = profile.get("status") or profile.get("profile_mode") or profile.get("profile_id")
        if self._last_profile_status is not None and profile_status != self._last_profile_status:
            self._bridge.event.emit(f"Profile status changed: {self._last_profile_status} → {profile_status}")
        self._last_profile_status = profile_status
        self._status.setText(f"{str(attitude.get('attitude_model_status') or observation.get('status') or 'WAITING').upper()} — read-only; sample timestamp unavailable")
        for key, label in self._attitude_labels.items(): label.setText(format_value(attitude.get(key), "deg"))

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
        position_rows = (("ground_truth", "Ground Truth"), ("observed_raw", "Observed/BBox Raw"), ("analytic_raw", "Analytic Attitude Raw"), ("corrected", "Corrected"))
        for source, row_name in position_rows:
            values = self._latest.get(source) or []
            state = self._source_states[source]
            age = now - self._received[source] if source in self._received else None
            for index, label in enumerate(self._position_labels[row_name][:3]):
                label.setText(stale_display(values[index] if len(values) > index else None, state, unit="m", age=age))
            norm = math.sqrt(sum(float(v) ** 2 for v in values[:3])) if len(values) >= 3 and all(v is not None for v in values[:3]) else None
            self._position_labels[row_name][3].setText(stale_display(norm, state, unit="m", age=age))
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
