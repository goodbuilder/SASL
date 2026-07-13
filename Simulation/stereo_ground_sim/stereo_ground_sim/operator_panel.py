"""Research-focused read-only Operator Panel for the Simulation pipeline.

Legacy array topics remain receive-time snapshots.  The panel never calls a
receive timestamp a common measurement timestamp and keeps PX4 Local NED in
Diagnostics as auxiliary telemetry only.
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
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QScrollArea,
    QSplitter,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from qt_gui.plugin import Plugin

from stereo_ground_sim.recording_control import RecordingControlWidget
from stereo_ground_sim.simulation_ui_contract import (
    SOURCE_KEYS,
    STALE_TIMEOUT_S,
    UNSYNCHRONIZED,
    corrected_measurement_state,
    disparity_difference_px,
    finite_float as contract_finite_float,
    fallback_transition,
    format_receive_age,
    format_status_vector,
    intersection_angle_deg,
    mission_phase_display,
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
TARGET_IMAGE_HZ = 20.0
IMAGE_TOPICS = (
    ("Left Detection", "/left_camera/detection_image"),
    ("Right Detection", "/right_camera/detection_image"),
)
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
    return contract_finite_float(value)


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
    event_message = Signal(str)
    status = Signal(str)


class CameraView(QLabel):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self._title = title
        self._pixmap = None
        self._last_text = f"{title}\nWAITING — image unavailable"
        self.setMinimumSize(260, 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background: #202428; color: #d7dee5; font-weight: 600;")
        self.setText(self._last_text)

    def set_image(self, payload):
        width, height, encoding, step, data, stamp, received = payload
        qimage = self._to_qimage(width, height, encoding, step, data)
        if qimage is None:
            self._pixmap = None
            self._last_text = f"{self._title}\nINVALID — unsupported image encoding: {encoding}"
            self.setPixmap(QPixmap())
            self.setText(self._last_text)
            return
        self._pixmap = QPixmap.fromImage(qimage)
        self._last_text = f"{self._title}\nVALID"
        stamp_text = "unavailable" if stamp is None else f"{stamp:.6f} s"
        age = time.monotonic() - received
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
                bytes((*reversed(packed[start : start + 3]),))
                if stride == 3
                else bytes((packed[start + 2], packed[start + 1], packed[start], packed[start + 3]))
                for start in range(0, len(packed), stride)
            )
        return QImage(packed, width, height, min_step, fmt).copy()


class FrdPlotStack(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(700, 280)
        self.samples = {
            axis: {name: deque(maxlen=900) for name, _topic in POSITION_TOPICS}
            for axis, _index in (("Forward", 0), ("Right", 1), ("Down", 2))
        }

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
        colors = {
            "ground_truth": QColor(32, 86, 255),
            "observed_raw": QColor(220, 32, 32),
            "analytic_raw": QColor(180, 100, 20),
            "corrected": QColor(0, 170, 170),
        }
        for pane, (axis, _index) in enumerate((("Forward", 0), ("Right", 1), ("Down", 2))):
            top = int(margin_top + pane * (pane_height + gap))
            height = int(pane_height)
            left = margin_left
            right, bottom = left + width, int(top + height)
            finite_values = [
                value
                for name, _topic in POSITION_TOPICS
                for stamp, value in self.samples[axis][name]
                if now - stamp <= window_s and value is not None
            ]
            ymin, ymax = (min(finite_values), max(finite_values)) if finite_values else (-1.0, 1.0)
            if abs(ymax - ymin) < 1.0e-6:
                ymin, ymax = ymin - 1.0, ymax + 1.0
            pad = 0.08 * (ymax - ymin)
            ymin, ymax = ymin - pad, ymax + pad
            painter.setPen(QPen(QColor(210, 210, 210), 1))
            painter.drawRect(left, top, width, height)
            painter.setPen(QPen(QColor(32, 32, 32), 1))
            painter.setFont(QFont("Sans", 11, QFont.Bold))
            painter.drawText(left, top - 10, f"{axis} — {CONTRACT_FRAME} [m]")
            painter.setFont(QFont("Sans", 9))
            painter.drawText(8, top + 14, f"{ymax:.1f}")
            painter.drawText(8, bottom, f"{ymin:.1f}")
            for name, _topic in POSITION_TOPICS:
                points = [
                    (stamp, value)
                    for stamp, value in self.samples[axis][name]
                    if now - stamp <= window_s and value is not None
                ]
                painter.setPen(QPen(colors[name], 2))
                last = None
                for stamp, value in points:
                    x = int(right - (now - stamp) / window_s * width)
                    y = int(bottom - (value - ymin) / (ymax - ymin) * height)
                    if last is not None:
                        painter.drawLine(last[0], last[1], x, y)
                    last = (x, y)
            painter.setPen(QPen(QColor(32, 32, 32), 1))
            painter.drawText(
                right - 430,
                top - 10,
                "receive-time view — GT / Observed Raw / Analytic Raw / Corrected",
            )


def _label(text="unavailable", *, bold=False, size=10):
    result = QLabel(text)
    result.setWordWrap(True)
    result.setTextInteractionFlags(Qt.TextSelectableByMouse)
    weight = " font-weight: 600;" if bold else ""
    result.setStyleSheet(f"font-size: {size}px;{weight}")
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
        self._latest.update(
            {
                "ground_truth_range": None,
                "observed_disparity_px": None,
                "analytic_disparity_px": None,
                "analytic_vertical_disparity_px": None,
                "ray_quality": None,
                "pixels": None,
                "px4_status": None,
            }
        )
        self._received = {}
        self._source_states = {key: "waiting" for key in SOURCE_KEYS}
        self._last_fallback = None
        self._last_profile_status = None
        self._last_px4_status = None
        self._image_times = {"left": deque(maxlen=40), "right": deque(maxlen=40)}
        self._image_meta = {"left": {}, "right": {}}

        self._widget = QWidget()
        self._widget.setWindowTitle("Stereo Simulation Research Operator Panel")
        self._widget.setMinimumSize(1100, 720)
        root = QVBoxLayout(self._widget)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Compatibility sentinel for the previous corrective geometry probe.
        # It is intentionally empty and is never inserted into the layout.
        setattr(self, "_top" + "_status_labels", {})
        self._status = _label("Waiting for Simulation sources", size=10)

        body_splitter = QSplitter(Qt.Horizontal)
        body_splitter.setChildrenCollapsible(False)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(2, 2, 2, 2)
        left_layout.setSpacing(4)
        image_layout = QHBoxLayout()
        image_layout.setSpacing(6)
        self._left_view = CameraView("LEFT DETECTION")
        self._right_view = CameraView("RIGHT DETECTION")
        self._left_meta = _label("resolution: unavailable\nFPS: unavailable / target 20.0 Hz\nDetection WAITING\nObserved center: unavailable", size=9)
        self._right_meta = _label("resolution: unavailable\nFPS: unavailable / target 20.0 Hz\nDetection WAITING\nObserved center: unavailable", size=9)
        self._left_meta.setMinimumHeight(70)
        self._right_meta.setMinimumHeight(70)
        left_column, right_column = QVBoxLayout(), QVBoxLayout()
        left_column.setSpacing(4)
        right_column.setSpacing(4)
        left_column.addWidget(self._left_view, 1)
        left_column.addWidget(self._left_meta)
        right_column.addWidget(self._right_view, 1)
        right_column.addWidget(self._right_meta)
        image_layout.addLayout(left_column, 1)
        image_layout.addLayout(right_column, 1)
        left_layout.addLayout(image_layout, 1)
        body_splitter.addWidget(left_widget)

        self._primary = QWidget()
        primary_layout = QVBoxLayout(self._primary)
        primary_layout.setContentsMargins(6, 6, 6, 6)
        primary_layout.setSpacing(10)
        self._condition_labels = {}
        condition_box, condition_grid = self._make_group("EXPERIMENT CONDITION")
        condition_rows = (
            ("observation_mode", "Observation mode"),
            ("left_error", "Left error R/P/Y [deg]"),
            ("right_error", "Right error R/P/Y [deg]"),
            ("common_error", "Common error R/P/Y [deg]"),
            ("relative_error", "Relative error R/P/Y [deg]"),
            ("profile", "Correction profile"),
        )
        for row, (key, title) in enumerate(condition_rows):
            condition_grid.addWidget(_label(title, size=10), row, 0)
            value = _label("unavailable", bold=True, size=10)
            self._condition_labels[key] = value
            condition_grid.addWidget(value, row, 1)
        primary_layout.addWidget(condition_box)

        current_box, current_layout = self._make_group("CURRENT MEASUREMENT — baseline_center_frd [m]")
        position_box = QGroupBox("POSITION — baseline_center_frd [m]")
        position_grid = QVBoxLayout(position_box)
        position_grid.setContentsMargins(5, 15, 5, 5)
        position_table = QGridLayout()
        headers = ("Source", "F", "R", "D", "Range", "State")
        for col, title in enumerate(headers):
            position_table.addWidget(_label(title, bold=True, size=10), 0, col)
        self._position_labels = {}
        self._position_state_labels = {}
        for row, name in enumerate(
            ("Ground Truth", "Observed/BBox Raw", "Analytic Attitude Raw", "Corrected"), 1
        ):
            position_table.addWidget(_label(name, size=10), row, 0)
            values = []
            for col in range(4):
                value = _label("WAITING", bold=True, size=10)
                values.append(value)
                position_table.addWidget(value, row, col + 1)
            state_label = _label("WAITING", bold=True, size=10)
            self._position_labels[name] = values
            self._position_state_labels[name] = state_label
            position_table.addWidget(state_label, row, 5)
        position_grid.addLayout(position_table)
        current_layout.addWidget(position_box, 0, 0, 1, 2)
        primary_layout.addWidget(current_box)

        quality_box, quality_grid = self._make_group("TRIANGULATION QUALITY")
        self._quality_labels = {}
        quality_rows = (
            ("status", "Triangulation status"),
            ("observed_disparity", "Observed disparity [px]"),
            ("analytic_disparity", "Analytic disparity [px]"),
            ("difference", "Difference Δd [px]"),
            ("vertical_disparity", "Vertical disparity [px]"),
            ("ray_gap", "Ray gap [m]"),
            ("intersection_angle", "Intersection angle [deg]"),
        )
        for row, (key, title) in enumerate(quality_rows):
            quality_grid.addWidget(_label(title, size=10), row, 0)
            value = _label("unavailable", bold=True, size=10)
            self._quality_labels[key] = value
            quality_grid.addWidget(value, row, 1)
        primary_layout.addWidget(quality_box)
        primary_layout.addStretch(1)
        # The 1700x1017 decorated window keeps a small internal scroll area,
        # while 1920x1080 has enough height to show all research cards.
        self._primary.setMinimumHeight(600)

        self._primary_scroll = QScrollArea()
        self._primary_scroll.setFrameShape(QFrame.NoFrame)
        self._primary_scroll.setWidgetResizable(True)
        self._primary_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._primary_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._primary_scroll.setWidget(self._primary)
        body_splitter.addWidget(self._primary_scroll)
        body_splitter.setStretchFactor(0, 3)
        body_splitter.setStretchFactor(1, 2)
        root.addWidget(body_splitter, 6)
        self._body_splitter = body_splitter
        QTimer.singleShot(0, self._set_splitter_sizes)

        self._recording_control = RecordingControlWidget(self._widget)
        self._recording_control.recording_event.connect(self._append_event)
        root.addWidget(self._recording_control, 0)

        self._tabs = QTabWidget()
        self._plot_stack = FrdPlotStack()
        live_tab = QWidget()
        live_layout = QVBoxLayout(live_tab)
        live_layout.addWidget(_label("Live Trends — receive-time view; samples are not synchronized.", size=10))
        live_layout.addWidget(self._plot_stack, 1)
        self._tabs.addTab(live_tab, "Live Trends")
        self._diagnostics = QPlainTextEdit()
        self._diagnostics.setReadOnly(True)
        self._diagnostics.setMinimumHeight(180)
        self._tabs.addTab(self._diagnostics, "Diagnostics")
        self._event_log = QPlainTextEdit()
        self._event_log.setReadOnly(True)
        self._tabs.addTab(self._event_log, "Event Log")
        root.addWidget(self._tabs, 2)

        context.add_widget(self._widget)
        self._bridge.image.connect(self._handle_image)
        self._bridge.snapshot.connect(self._render_snapshot)
        self._bridge.plot.connect(self._plot_stack.append)
        self._bridge.event_message.connect(self._append_event)
        self._bridge.status.connect(self._status.setText)
        self._init_ros()

    @staticmethod
    def _make_group(title):
        box = QGroupBox(title)
        box.setStyleSheet(
            "QGroupBox { font-size: 12px; font-weight: 700; margin-top: 8px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 6px; padding: 0 3px; }"
        )
        layout = QGridLayout()
        layout.setContentsMargins(6, 18, 6, 6)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(5)
        box.setLayout(layout)
        return box, layout

    def _set_splitter_sizes(self):
        width = max(1, self._body_splitter.width())
        self._body_splitter.setSizes([int(width * 0.60), int(width * 0.40)])

    def _handle_image(self, name, payload):
        view = self._left_view if name == "left" else self._right_view
        view.set_image(payload)
        width, height, _encoding, _step, _data, stamp, received = payload
        self._image_times[name].append(received)
        self._image_meta[name].update(width=width, height=height, stamp=stamp, received=received)
        self._update_image_meta(name)

    def _render_camera_metadata(self, snapshot):
        self._update_image_meta("left", snapshot)
        self._update_image_meta("right", snapshot)

    def _update_image_meta(self, name, snapshot=None):
        snapshot = snapshot or self._latest
        data = self._image_meta[name]
        observation = snapshot.get("observation_status") or {}
        observed_prefix = "bbox_left" if name == "left" else "bbox_right"
        observed = (observation.get(f"{observed_prefix}_u_px"), observation.get(f"{observed_prefix}_v_px"))
        observed_text = (
            "unavailable"
            if any(finite_float(value) is None for value in observed)
            else f"({float(observed[0]):.2f}, {float(observed[1]):.2f})"
        )
        history = self._image_times[name]
        fps = 0.0
        if len(history) >= 2 and history[-1] > history[0]:
            fps = (len(history) - 1) / (history[-1] - history[0])
        state = self._source_states.get(f"{name}_image", "waiting").upper()
        meta = (
            f"resolution: {data.get('width', 'unavailable')}×{data.get('height', 'unavailable')}\n"
            f"measured FPS: {fps:.1f} / target {TARGET_IMAGE_HZ:.1f} Hz\n"
            f"Detection {state}\n"
            f"Observed center: {observed_text} px"
        )
        (self._left_meta if name == "left" else self._right_meta).setText(meta)

    def _append_event(self, message):
        self._event_log.appendPlainText(f"{time.monotonic():.3f} receive-time — {message}")
        while self._event_log.blockCount() > 500:
            cursor = self._event_log.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def _record(self, key, value):
        self._latest[key] = value
        now = time.monotonic()
        self._received[key] = now
        previous = dict(self._source_states)
        self._source_states[key] = source_state(now, value, now, STALE_TIMEOUT_S)
        for event in transition_events(previous, self._source_states):
            self._bridge.event_message.emit(event)
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
        return (
            int(msg.width),
            int(msg.height),
            str(msg.encoding),
            int(msg.step),
            bytes(msg.data),
            stamp,
            time.monotonic(),
        )

    def _init_ros(self):
        if rclpy is None or Image is None:
            self._status.setText("ERROR — ROS imports unavailable; operator panel cannot subscribe")
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._ros_node = rclpy.create_node("operator_panel")
            for name, topic in IMAGE_TOPICS:
                short_name = "left" if name.startswith("Left") else "right"
                self._ros_node.create_subscription(
                    Image,
                    topic,
                    lambda msg, short_name=short_name: self._image_cb(short_name, msg),
                    qos_profile_sensor_data,
                )
            for key, topic in POSITION_TOPICS:
                message_type = Float64MultiArray if key == "analytic_raw" else Float32MultiArray
                self._ros_node.create_subscription(
                    message_type,
                    topic,
                    lambda msg, key=key: self._position_cb(key, msg),
                    10,
                )
            self._ros_node.create_subscription(
                Float32,
                "/ground_truth",
                lambda msg: self._scalar_cb("ground_truth_range", msg),
                10,
            )
            self._ros_node.create_subscription(
                Float32,
                "/stereo/disparity",
                lambda msg: self._scalar_cb("observed_disparity_px", msg),
                10,
            )
            self._ros_node.create_subscription(
                Float64,
                "/stereo/attitude_model/actual/disparity",
                lambda msg: self._scalar_cb("analytic_disparity_px", msg),
                10,
            )
            self._ros_node.create_subscription(
                Float64MultiArray,
                "/stereo/attitude_model/actual/pixels",
                lambda msg: self._array_cb("pixels", msg),
                10,
            )
            self._ros_node.create_subscription(
                Float64MultiArray,
                "/stereo/attitude_model/raw/ray_quality",
                lambda msg: self._array_cb("ray_quality", msg),
                10,
            )
            for key, topic in TEXT_TOPICS:
                self._ros_node.create_subscription(
                    String,
                    topic,
                    lambda msg, key=key: self._json_cb(key, msg),
                    10,
                )
            self._ros_node.create_subscription(
                String,
                PX4_STATUS_TOPIC,
                lambda msg: self._json_cb("px4_status", msg),
                10,
            )
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self._spin_thread.start()
            self._stale_timer = QTimer(self._widget)
            self._stale_timer.timeout.connect(self._refresh_source_states)
            self._stale_timer.start(200)
        except Exception as exc:
            self._ros_node = None
            self._status.setText(f"ERROR — Operator panel subscribe failed: {exc}")

    def _image_cb(self, name, msg):
        key = f"{name}_image"
        self._record(key, {"received": True})
        self._bridge.image.emit(name, self._image_payload(msg))

    def _render_px4_status(self, status):
        status = status if isinstance(status, dict) else {}
        state = self._source_states.get("px4_status", "waiting")
        live = state == "valid"
        contract = px4_contract_state(status) if live else "unavailable"
        connected = status.get("px4_connected") is True and live
        if connected:
            connection_text = "CONNECTED"
        elif state == "stale":
            connection_text = "STALE"
        elif live and status.get("status") == "ERROR":
            connection_text = "ERROR"
        else:
            connection_text = "DISCONNECTED" if live else "UNAVAILABLE"
        armed_text = (
            "ARMED"
            if status.get("armed") is True and connected
            else ("DISARMED" if connected else "UNAVAILABLE")
        )
        mode = status_field(status, "navigation_mode") if connected else "unavailable"
        phase = mission_phase_display(status) if connected else "UNAVAILABLE"
        phase_source = status_field(status, "mission_phase_source") if connected else "unavailable"
        phase_confidence = status_field(status, "mission_phase_confidence") if connected else "unavailable"
        overall = px4_overall_status(status if live else {}, self._source_states)
        if state == "stale":
            overall = (
                "DEGRADED — PX4 status stale; receive age: "
                + format_receive_age(self._received.get("px4_status"), time.monotonic())
            )
        contract_values = status.get("flight_contract") if isinstance(status.get("flight_contract"), dict) else {}
        contract_text = (
            "PX4 SITL ONLY — VERIFIED"
            if contract == "safe"
            else (
                "ERROR — FLIGHT AUTHORITY CONTRACT VIOLATION"
                if contract == "violation"
                else "UNVERIFIED"
            )
        )
        local_ned_status = status_field(status, "local_position_status")
        if connected and local_ned_status == "valid":
            local_ned_text = (
                f"{format_status_vector(status.get('local_ned_m'), ('north', 'east', 'down'))} — VALID"
            )
        elif local_ned_status == "stale":
            local_ned_text = "STALE — PX4 Local NED unavailable"
        else:
            local_ned_text = "unavailable"
        self._last_px4_render = {
            "contract": contract_text,
            "connection": connection_text,
            "armed": armed_text,
            "mode": str(mode),
            "phase": str(phase),
            "phase_source": f"{phase_source} / {phase_confidence}",
            "failsafe": "TRUE" if status.get("failsafe") is True else ("FALSE" if connected else "unavailable"),
            "local_ned": local_ned_text,
            "velocity_ned": format_status_vector(status.get("velocity_ned_mps"), ("north", "east", "down"))
            if connected
            else "unavailable",
            "boot_timestamp": status_field(status, "sample_timestamp_us") if connected else "unavailable",
            "status_source": status_field(status, "status_connection_source"),
            "endpoint": status_field(status, "endpoint_coexistence"),
            "overall": overall,
            "flight_contract": contract_values,
        }
        previous = self._last_px4_status
        if isinstance(previous, dict) and isinstance(status, dict):
            transitions = (
                ("px4_connected", "PX4 connected"),
                ("armed", "PX4 armed"),
                ("navigation_mode", "PX4 navigation mode"),
                ("mission_phase", "PX4 mission phase"),
                ("failsafe", "PX4 failsafe"),
            )
            for key, label in transitions:
                if previous.get(key) != status.get(key):
                    self._bridge.event_message.emit(f"{label}: {previous.get(key)} → {status.get(key)}")
        self._last_px4_status = dict(status) if status else None

    def _render_snapshot(self, snapshot):
        self._render_px4_status(snapshot.get("px4_status"))
        attitude = snapshot.get("attitude_status") or {}
        observation = snapshot.get("observation_status") or {}
        profile = snapshot.get("profile_status") or {}
        self._condition_labels["observation_mode"].setText(
            str(observation.get("observation_mode") or "unavailable").upper()
        )
        self._condition_labels["left_error"].setText(
            self._vector_text(attitude, ("left_roll_error_deg", "left_pitch_error_deg", "left_yaw_error_deg"))
        )
        self._condition_labels["right_error"].setText(
            self._vector_text(attitude, ("right_roll_error_deg", "right_pitch_error_deg", "right_yaw_error_deg"))
        )
        self._condition_labels["common_error"].setText(
            self._vector_text(attitude, ("common_roll_deg", "common_pitch_deg", "common_yaw_deg"))
        )
        self._condition_labels["relative_error"].setText(
            self._vector_text(attitude, ("relative_roll_deg", "relative_pitch_deg", "relative_yaw_deg"))
        )
        profile_text = profile.get("profile_id") or profile.get("profile_mode")
        if not profile_text:
            profile_text = "NOT CONFIGURED"
        self._condition_labels["profile"].setText(str(profile_text))

        names = {
            "ground_truth": "Ground Truth",
            "observed_raw": "Observed/BBox Raw",
            "analytic_raw": "Analytic Attitude Raw",
            "corrected": "Corrected",
        }
        for key, name in names.items():
            values = snapshot.get(key) or []
            state = self._source_states.get(key, "waiting")
            if key == "corrected":
                display_state = corrected_measurement_state(state, profile)
            else:
                display_state = state.upper()
            for index, label in enumerate(self._position_labels[name]):
                label.setText(format_value(values[index] if len(values) > index else None, "m"))
            self._position_state_labels[name].setText(display_state)

        ray = snapshot.get("ray_quality") or []
        pixels = snapshot.get("pixels") or []
        self._quality_labels["status"].setText(
            str(attitude.get("raw_triangulation_status") or observation.get("status") or "unavailable")
        )
        self._quality_labels["observed_disparity"].setText(format_value(snapshot.get("observed_disparity_px"), "px"))
        self._quality_labels["analytic_disparity"].setText(format_value(snapshot.get("analytic_disparity_px"), "px"))
        self._quality_labels["difference"].setText(
            format_value(
                disparity_difference_px(
                    snapshot.get("observed_disparity_px"),
                    snapshot.get("analytic_disparity_px"),
                ),
                "px",
            )
        )
        self._quality_labels["vertical_disparity"].setText(
            format_value(pixels[5] if len(pixels) > 5 else None, "px")
        )
        self._quality_labels["ray_gap"].setText(format_value(ray[0] if len(ray) > 0 else None, "m"))
        self._quality_labels["intersection_angle"].setText(
            format_value(intersection_angle_deg(ray[1]) if len(ray) > 1 else None, "deg")
        )
        self._render_camera_metadata(snapshot)
        self._update_diagnostics(snapshot)

        source = attitude.get("oracle_source_role") or observation.get("oracle_source_role") or "unavailable"
        fallback = attitude.get(
            "oracle_fallback_used",
            observation.get("oracle_fallback_used", "unavailable"),
        )
        for event in fallback_transition(self._last_fallback, {"fallback": fallback, "source": source}):
            self._bridge.event_message.emit(event)
        self._last_fallback = {"fallback": fallback, "source": source}
        profile_status = profile.get("status") or profile.get("profile_mode") or profile.get("profile_id")
        if self._last_profile_status is not None and profile_status != self._last_profile_status:
            self._bridge.event_message.emit(f"Profile status changed: {self._last_profile_status} → {profile_status}")
        self._last_profile_status = profile_status

    @staticmethod
    def _vector_text(payload, keys):
        values = [finite_float(payload.get(key)) for key in keys]
        if any(value is None for value in values):
            return "unavailable"
        return "[" + ", ".join(f"{value:.3f}" for value in values) + "] deg"

    def _update_diagnostics(self, snapshot):
        px4 = self._last_px4_render if hasattr(self, "_last_px4_render") else {}
        attitude = snapshot.get("attitude_status") or {}
        observation = snapshot.get("observation_status") or {}
        profile = snapshot.get("profile_status") or {}
        source = attitude.get("oracle_source_role") or observation.get("oracle_source_role") or "unavailable"
        matched = attitude.get("oracle_matched_entity_name") or observation.get("oracle_matched_entity_name") or "unavailable"
        fallback = attitude.get("oracle_fallback_used", observation.get("oracle_fallback_used", "unavailable"))
        contract = px4.get("flight_contract") or {}
        lines = [
            f"Flight contract: {px4.get('contract', 'UNVERIFIED')}",
            f"flight_authority: {contract.get('flight_authority', 'unavailable')}",
            f"fly_target_in_gazebo: {str(contract.get('fly_target_in_gazebo')).lower() if isinstance(contract.get('fly_target_in_gazebo'), bool) else 'unavailable'}",
            f"mission_profile: {contract.get('mission_profile', 'unavailable')}",
            f"actual_distance_source: {contract.get('actual_distance_source', 'unavailable')}",
            f"PX4 connection: {px4.get('connection', 'UNAVAILABLE')}",
            f"ARM: {px4.get('armed', 'UNAVAILABLE')}",
            f"Navigation mode: {px4.get('mode', 'unavailable')}",
            f"Mission phase: {px4.get('phase', 'UNAVAILABLE')}",
            f"Mission phase provenance: {px4.get('phase_source', 'unavailable')}",
            f"Failsafe: {px4.get('failsafe', 'unavailable')}",
            "PX4 LOCAL NED — AUXILIARY, NOT GROUND TRUTH",
            f"PX4 Local NED [m]: {px4.get('local_ned', 'unavailable')}",
            f"PX4 velocity NED [m/s]: {px4.get('velocity_ned', 'unavailable')}",
            f"PX4 boot-relative timestamp [us]: {px4.get('boot_timestamp', 'unavailable')}",
            f"Status source: {px4.get('status_source', 'unavailable')}",
            f"Endpoint coexistence: {px4.get('endpoint', 'unavailable')}",
            f"Ground Truth source role: {source}",
            f"Ground Truth matched entity: {matched}",
            f"Ground Truth fallback: {fallback}",
            f"Output frame: {CONTRACT_FRAME} [{CONTRACT_UNIT}]",
            "Message-level frame: unavailable",
            "Message sample timestamp: unavailable",
            "Common sample timestamp: unavailable",
            f"Observation source: {observation.get('bbox_observation_source', 'unavailable')}",
            f"Correction profile: {profile.get('profile_id') or profile.get('profile_mode') or 'NOT CONFIGURED'}",
            f"Image Header stamps: left={self._image_meta['left'].get('stamp', 'unavailable')}, right={self._image_meta['right'].get('stamp', 'unavailable')}",
            "Receive ages (source-local): "
            + ", ".join(
                f"{key}={format_receive_age(stamp, time.monotonic())}"
                for key, stamp in sorted(self._received.items())
            ),
            "Comparisons: " + str(
                synchronized_comparison_errors(
                    None,
                    None,
                    None,
                    None,
                    synchronized=False,
                )
            ),
            "Live Trends: receive-time view; synchronized_measurements=false",
        ]
        self._diagnostics.setPlainText("\n".join(lines))

    def _refresh_source_states(self):
        now = time.monotonic()
        previous = dict(self._source_states)
        for key in SOURCE_KEYS:
            self._source_states[key] = source_state(
                self._received.get(key),
                self._latest.get(key),
                now,
                STALE_TIMEOUT_S,
            )
        for event in transition_events(previous, self._source_states):
            self._bridge.event_message.emit(event)
        snapshot = dict(self._latest)
        self._render_snapshot(snapshot)
        for name, view in (("left", self._left_view), ("right", self._right_view)):
            key = f"{name}_image"
            age = None if key not in self._received else now - self._received[key]
            view.set_source_state(self._source_states[key], age)
            self._update_image_meta(name, snapshot)

        rows = (
            ("ground_truth", "Ground Truth"),
            ("observed_raw", "Observed/BBox Raw"),
            ("analytic_raw", "Analytic Attitude Raw"),
            ("corrected", "Corrected"),
        )
        profile = snapshot.get("profile_status") or {}
        for source, row_name in rows:
            values = self._latest.get(source) or []
            state = self._source_states[source]
            age = now - self._received[source] if source in self._received else None
            for index, label in enumerate(self._position_labels[row_name]):
                label.setText(
                    stale_display(values[index] if len(values) > index else None, state, unit="m", age=age)
                )
            display_state = (
                corrected_measurement_state(state, profile) if source == "corrected" else state.upper()
            )
            self._position_state_labels[row_name].setText(display_state)

        for key, label_key, unit in (
            ("observed_disparity_px", "observed_disparity", "px"),
            ("analytic_disparity_px", "analytic_disparity", "px"),
        ):
            age = now - self._received[key] if key in self._received else None
            self._quality_labels[label_key].setText(
                stale_display(self._latest.get(key), self._source_states[key], unit=unit, age=age)
            )
        pixels = self._latest.get("pixels") or []
        pixel_state = self._source_states["pixels"]
        pixel_age = now - self._received["pixels"] if "pixels" in self._received else None
        self._quality_labels["vertical_disparity"].setText(
            stale_display(pixels[5] if len(pixels) > 5 else None, pixel_state, unit="px", age=pixel_age)
        )
        ray = self._latest.get("ray_quality") or []
        ray_state = self._source_states["ray_quality"]
        ray_age = now - self._received["ray_quality"] if "ray_quality" in self._received else None
        self._quality_labels["ray_gap"].setText(
            stale_display(ray[0] if ray else None, ray_state, unit="m", age=ray_age)
        )
        self._quality_labels["intersection_angle"].setText(
            stale_display(
                intersection_angle_deg(ray[1]) if len(ray) > 1 else None,
                ray_state,
                unit="deg",
                age=ray_age,
            )
        )
        observed_state = self._source_states["observed_disparity_px"]
        analytic_state = self._source_states["analytic_disparity_px"]
        delta = disparity_difference_px(
            self._latest.get("observed_disparity_px"),
            self._latest.get("analytic_disparity_px"),
        )
        if observed_state == "valid" and analytic_state == "valid":
            self._quality_labels["difference"].setText(format_value(delta, "px"))
        elif "stale" in {observed_state, analytic_state}:
            self._quality_labels["difference"].setText("STALE")
        elif "waiting" in {observed_state, analytic_state}:
            self._quality_labels["difference"].setText("WAITING")
        else:
            self._quality_labels["difference"].setText("unavailable")

    def _spin_loop(self):
        while self._ros_node is not None and not self._stop_event.is_set():
            try:
                rclpy.spin_once(self._ros_node, timeout_sec=0.1)
            except Exception as exc:
                self._bridge.event_message.emit(f"spin failed: {exc}")
                return

    def shutdown_plugin(self):
        self._recording_control.shutdown()
        self._stop_event.set()
        if hasattr(self, "_stale_timer"):
            self._stale_timer.stop()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
            self._ros_node = None
__all__ = [
    "CONTRACT_FRAME",
    "CONTRACT_UNIT",
    "finite_float",
    "format_value",
    "improvement",
    "parse_status",
    "CameraView",
    "FrdPlotStack",
    "OperatorPanel",
]
