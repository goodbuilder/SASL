"""Operational, read-only Qt UI for the independent PoseSimulation app."""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

from python_qt_binding.QtCore import QObject, QTimer, Qt, Signal, Slot
from python_qt_binding.QtGui import QColor, QFont, QImage, QPainter, QPalette, QPen, QPixmap
from python_qt_binding.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

try:  # rqt discovery should still be able to import the plugin without ROS.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from pose_simulation_interfaces.msg import PoseError, PoseSimulationStatus, StereoGeometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
except Exception:  # pragma: no cover - depends on the ROS installation
    rclpy = None
    DiagnosticArray = None  # type: ignore[assignment,misc]
    PoseError = None  # type: ignore[assignment,misc]
    PoseSimulationStatus = None  # type: ignore[assignment,misc]
    StereoGeometry = None  # type: ignore[assignment,misc]
    SingleThreadedExecutor = None  # type: ignore[assignment,misc]
    qos_profile_sensor_data = None  # type: ignore[assignment,misc]

try:  # Images are optional; the Camera tab has an explicit empty state.
    from sensor_msgs.msg import CompressedImage, Image
except Exception:  # pragma: no cover - depends on the ROS installation
    CompressedImage = None  # type: ignore[assignment,misc]
    Image = None  # type: ignore[assignment,misc]

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - optional runtime dependency
    CvBridge = None  # type: ignore[assignment,misc]

from .trend_model import TrendSample, append_sample, series_for_key, split_valid_segments
from .ui_model import (
    OPTICAL_FRAME,
    RIG_FRAME,
    WORLD_FRAME,
    UiSnapshot,
    build_ui_snapshot,
    format_metric,
    source_state_label,
    state_label,
)


DEFAULT_NAMESPACE = "/pose_simulation"
DEFAULT_LEFT_IMAGE_TOPIC = "/left_camera/detection_image"
DEFAULT_RIGHT_IMAGE_TOPIC = "/right_camera/detection_image"


def _load_ui_config(path: str | None) -> Mapping[str, Any]:
    if not path:
        return {}
    try:
        import yaml

        value = yaml.safe_load(Path(path).read_text())
        return value if isinstance(value, Mapping) else {}
    except (OSError, TypeError, ValueError):
        return {}


def _topic(namespace: str, name: str) -> str:
    return f"/{namespace.strip('/')}/{name.strip('/')}"


def _stamp_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    try:
        if sec is not None and nanosec is not None:
            value = int(sec) * 1_000_000_000 + int(nanosec)
            return value if value >= 0 else None
    except (TypeError, ValueError):
        return None
    return None


@dataclass(frozen=True)
class ImageFrame:
    """Immutable RGB frame prepared outside the Qt widget thread."""

    side: str
    data: bytes
    width: int
    height: int
    encoding: str
    stamp_ns: int | None
    received_monotonic_ns: int


class PoseSimulationRosBridge(QObject):
    """ROS worker bridge: callbacks emit immutable data and never touch widgets."""

    snapshot_ready = Signal(object)
    diagnostics_ready = Signal(object)
    image_available = Signal()
    bridge_error = Signal(str)

    def __init__(self, namespace: str = DEFAULT_NAMESPACE, settings: Mapping[str, Any] | None = None, parent: QObject | None = None):
        super().__init__(parent)
        self._namespace = namespace
        self._settings = settings or {}
        self._payloads: dict[str, Any] = {}
        self._node = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._owns_context = False
        self._lock = threading.Lock()
        self._latest_frames: dict[str, tuple[int, ImageFrame]] = {}
        self._frame_sequence = 0
        self._delivered_sequence: dict[str, int] = {}
        self._cv_bridge = CvBridge() if CvBridge is not None else None
        self._start()

    def _start(self) -> None:
        if rclpy is None or PoseError is None or StereoGeometry is None or PoseSimulationStatus is None:
            self.bridge_error.emit("PoseSimulation typed ROS messages are unavailable")
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_context = True
            self._node = rclpy.create_node("pose_simulation_rqt")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self._node.create_subscription(PoseError, _topic(self._namespace, "pose_error"), self._pose_error_callback, 10)
            self._node.create_subscription(StereoGeometry, _topic(self._namespace, "stereo_geometry"), self._stereo_callback, 10)
            self._node.create_subscription(PoseSimulationStatus, _topic(self._namespace, "status"), self._status_callback, 10)
            if DiagnosticArray is not None:
                self._node.create_subscription(DiagnosticArray, _topic(self._namespace, "diagnostics"), self._diagnostics_callback, 10)
            self._create_image_subscriptions()
            self._thread = threading.Thread(target=self._spin, name="pose-simulation-rqt-ros", daemon=True)
            self._thread.start()
        except Exception as exc:  # pragma: no cover - runtime-specific
            self.bridge_error.emit(f"ROS bridge unavailable: {exc}")

    def _create_image_subscriptions(self) -> None:
        configured = self._settings.get("camera", {})
        camera: dict[str, Any] = {
            "enabled": True,
            "left_topic": DEFAULT_LEFT_IMAGE_TOPIC,
            "right_topic": DEFAULT_RIGHT_IMAGE_TOPIC,
            "message_type": "sensor_msgs/msg/Image",
            "display_max_hz": 20.0,
        }
        if isinstance(configured, Mapping):
            camera.update(configured)
        if not bool(camera.get("enabled", True)):
            return
        if qos_profile_sensor_data is None or (Image is None and CompressedImage is None):
            self.bridge_error.emit("Camera stream support is unavailable")
            return
        message_type = str(camera.get("message_type", "sensor_msgs/msg/Image") or "sensor_msgs/msg/Image")
        ros_type = Image if message_type == "sensor_msgs/msg/Image" else CompressedImage if message_type == "sensor_msgs/msg/CompressedImage" else None
        if ros_type is None:
            self.bridge_error.emit(f"Unsupported configured camera message type: {message_type}")
            return
        for side in ("left", "right"):
            topic = str(camera.get(f"{side}_topic", "") or "").strip()
            if not topic:
                continue
            if not topic.startswith("/"):
                self.bridge_error.emit(f"Camera topic must be absolute: {topic}")
                continue
            callback = lambda message, camera_side=side: self._image_callback(camera_side, message, message_type)
            self._node.create_subscription(ros_type, topic, callback, qos_profile_sensor_data)

    def _spin(self) -> None:
        try:
            while not self._stop.is_set() and self._executor is not None:
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as exc:  # pragma: no cover - shutdown races are runtime-specific
            if not self._stop.is_set():
                self.bridge_error.emit(f"ROS bridge stopped: {exc}")

    def _publish_snapshot(self) -> None:
        snapshot = build_ui_snapshot(
            self._payloads.get("pose_error"),
            self._payloads.get("stereo_geometry"),
            self._payloads.get("status"),
        )
        self.snapshot_ready.emit(snapshot)

    def _pose_error_callback(self, message: Any) -> None:
        self._payloads["pose_error"] = message
        self._publish_snapshot()

    def _stereo_callback(self, message: Any) -> None:
        self._payloads["stereo_geometry"] = message
        self._publish_snapshot()

    def _status_callback(self, message: Any) -> None:
        self._payloads["status"] = message
        self._publish_snapshot()

    def _diagnostics_callback(self, message: Any) -> None:
        records = []
        for item in getattr(message, "status", ()):
            records.append(
                {
                    "name": str(getattr(item, "name", "")),
                    "level": int(getattr(item, "level", 0) or 0),
                    "message": str(getattr(item, "message", "")),
                    "values": tuple((str(value.key), str(value.value)) for value in getattr(item, "values", ())),
                }
            )
        self.diagnostics_ready.emit(tuple(records))

    def _image_callback(self, side: str, message: Any, message_type: str) -> None:
        try:
            if self._cv_bridge is None:
                raise RuntimeError("cv_bridge is not installed")
            if message_type == "sensor_msgs/msg/CompressedImage":
                image = self._cv_bridge.compressed_imgmsg_to_cv2(message, desired_encoding="rgb8")
                encoding = "compressed/rgb8"
            else:
                image = self._cv_bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
                encoding = str(getattr(message, "encoding", "rgb8") or "rgb8")
            height, width = image.shape[:2]
            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(f"expected RGB image, got shape {image.shape!r}")
            data = bytes(image.tobytes(order="C"))
            frame = ImageFrame(side, data, int(width), int(height), encoding, _stamp_ns(message), time.monotonic_ns())
            with self._lock:
                self._frame_sequence += 1
                self._latest_frames[side] = (self._frame_sequence, frame)
            # The Qt side polls the coalesced buffer.  This signal is a wake-up
            # hint only; no frame or widget is carried through the callback.
            self.image_available.emit()
        except Exception as exc:  # pragma: no cover - depends on incoming encoding
            self.bridge_error.emit(f"{side} image rejected: {exc}")

    def take_latest_frames(self) -> Mapping[str, ImageFrame]:
        with self._lock:
            result: dict[str, ImageFrame] = {}
            for side, (sequence, frame) in self._latest_frames.items():
                if sequence > self._delivered_sequence.get(side, 0):
                    result[side] = frame
                    self._delivered_sequence[side] = sequence
            return result

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
    """Palette-aware plot with explicit invalid gaps and readable legend."""

    SERIES_COLORS = ("#2673c5", "#c4475b", "#b17800", "#3b8f56", "#7a55a8")

    def __init__(self, title: str, unit: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._title = title
        self._unit = unit
        self._series: Mapping[str, Sequence[tuple[float, float | None]]] = {}
        self._window_s: float | None = 30.0
        self._empty_reason = "Waiting for aligned pose samples"
        self.setMinimumHeight(205)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_data(
        self,
        series: Mapping[str, Sequence[tuple[float, float | None]]],
        window_s: float | None,
        *,
        title: str | None = None,
        unit: str | None = None,
        empty_reason: str | None = None,
    ) -> None:
        if title is not None:
            self._title = title
        if unit is not None:
            self._unit = unit
        if empty_reason is not None:
            self._empty_reason = empty_reason
        self._series = series
        self._window_s = window_s
        self.update()

    def paintEvent(self, event) -> None:  # pragma: no cover - visual rendering
        painter = QPainter(self)
        palette = QApplication.palette()
        painter.fillRect(self.rect(), palette.color(QPalette.Base))
        text_color = palette.color(QPalette.Text)
        muted = palette.color(QPalette.Mid)
        painter.setPen(text_color)
        painter.setFont(QFont("Sans", 9, QFont.Bold))
        painter.drawText(10, 18, f"{self._title} [{self._unit}]")
        plot = self.rect().adjusted(52, 30, -12, -28)
        painter.setPen(QPen(muted, 1))
        painter.drawRect(plot)
        finite_points = [
            (timestamp, value)
            for values in self._series.values()
            for timestamp, value in values
            if value is not None and math.isfinite(value)
        ]
        if not finite_points:
            painter.setPen(muted)
            painter.drawText(plot, Qt.AlignCenter, self._empty_reason)
            return
        max_time = max(timestamp for timestamp, _ in finite_points)
        min_time = max_time - self._window_s if self._window_s is not None else min(timestamp for timestamp, _ in finite_points)
        visible_values = [value for timestamp, value in finite_points if timestamp >= min_time]
        if not visible_values:
            painter.setPen(muted)
            painter.drawText(plot, Qt.AlignCenter, self._empty_reason)
            return
        min_value, max_value = min(visible_values), max(visible_values)
        if abs(max_value - min_value) < 1.0e-12:
            padding = max(1.0, abs(max_value) * 0.1)
        else:
            padding = (max_value - min_value) * 0.08
        min_value -= padding
        max_value += padding
        painter.setPen(muted)
        painter.drawText(2, plot.top() + 4, f"{max_value:.3g}")
        painter.drawText(2, plot.bottom(), f"{min_value:.3g}")
        for index, (name, values) in enumerate(self._series.items()):
            color = self.SERIES_COLORS[index % len(self.SERIES_COLORS)]
            for segment in split_valid_segments([(t, v if t >= min_time else None) for t, v in values]):
                points = []
                for timestamp, value in segment:
                    x = plot.left() + (0.5 if max_time == min_time else (timestamp - min_time) / (max_time - min_time)) * plot.width()
                    y = plot.bottom() - (value - min_value) / (max_value - min_value) * plot.height()
                    points.append((int(x), int(y)))
                painter.setPen(QPen(QColor(color), 2))
                if len(points) == 1:
                    painter.drawEllipse(points[0][0] - 2, points[0][1] - 2, 4, 4)
                else:
                    for first, second in zip(points, points[1:]):
                        painter.drawLine(first[0], first[1], second[0], second[1])
        painter.setFont(QFont("Sans", 8))
        legend_x = plot.left()
        for index, (name, values) in enumerate(self._series.items()):
            color = self.SERIES_COLORS[index % len(self.SERIES_COLORS)]
            current = next((value for _, value in reversed(values) if value is not None and math.isfinite(value)), None)
            if current is None:
                current_text = "—"
            elif self._unit == "deg":
                current_text = f"{current:+.3f}°"
            else:
                current_text = f"{current:.3f} {self._unit}"
            painter.setPen(QColor(color))
            painter.drawText(legend_x, self.height() - 8, f"{name}  {current_text}")
            legend_x += max(92, len(name) * 7 + len(current_text) * 7 + 18)


class CameraPanel(QFrame):
    """16:9 latest-frame display; all updates happen on the Qt main thread."""

    def __init__(self, title: str, topic: str, parent: QWidget | None = None):
        super().__init__(parent)
        self._topic = topic
        self._image: QImage | None = None
        self._last_frame: ImageFrame | None = None
        self._arrival_times: list[float] = []
        self.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(self)
        title_label = QLabel(title)
        title_label.setObjectName("sectionTitle")
        layout.addWidget(title_label)
        self._image_label = QLabel()
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setMinimumSize(320, 180)
        self._image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._image_label.setText(self._empty_text())
        layout.addWidget(self._image_label, 1)
        self._status = QLabel("UNAVAILABLE")
        self._status.setObjectName("mutedText")
        layout.addWidget(self._status)

    def _empty_text(self) -> str:
        return "Camera stream unavailable\nConfigured source: " + (self._topic or "<not configured>")

    def set_frame(self, frame: ImageFrame) -> None:
        self._image = QImage(frame.data, frame.width, frame.height, frame.width * 3, QImage.Format_RGB888).copy()
        self._last_frame = frame
        now = time.monotonic()
        self._arrival_times = [value for value in self._arrival_times if now - value <= 5.0]
        self._arrival_times.append(now)
        self._image_label.setText("")
        self._rescale()
        hz = (len(self._arrival_times) - 1) / (self._arrival_times[-1] - self._arrival_times[0]) if len(self._arrival_times) > 1 and self._arrival_times[-1] > self._arrival_times[0] else 0.0
        sim_time = "—" if frame.stamp_ns is None else f"{frame.stamp_ns / 1.0e9:.3f} s"
        self._status.setText(f"VALID · {frame.width}×{frame.height} · {hz:.1f} Hz · image sim time {sim_time}")

    def set_unavailable(self, message: str | None = None) -> None:
        if self._image is None:
            self._image_label.setText(message or self._empty_text())
            self._status.setText("UNAVAILABLE")

    def _rescale(self) -> None:
        if self._image is None:
            return
        pixmap = QPixmap.fromImage(self._image)
        self._image_label.setPixmap(pixmap.scaled(self._image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event) -> None:  # pragma: no cover - Qt geometry callback
        super().resizeEvent(event)
        self._rescale()


class PoseSimulationWidget(QWidget):
    """Seven-tab operator-facing view with no pose-writing controls."""

    def __init__(self, parent: QWidget | None = None, *, start_bridge: bool = True):
        super().__init__(parent)
        self.setObjectName("PoseSimulationWidget")
        self.setWindowTitle("PoseSimulation")
        self._ui_settings = _load_ui_config(os.environ.get("POSE_SIMULATION_UI_CONFIG"))
        self._namespace = os.environ.get("POSE_SIMULATION_NAMESPACE") or str(self._ui_settings.get("namespace", DEFAULT_NAMESPACE))
        # Latest source sample, last valid sample, and displayed sample are
        # intentionally distinct so Pause display never turns an old value
        # into the current result.
        self._latest_snapshot: UiSnapshot | None = None
        self._last_valid_snapshot: UiSnapshot | None = None
        self._snapshot: UiSnapshot | None = None
        self._diagnostics: tuple[Mapping[str, Any], ...] = ()
        self._bridge_error = ""
        self._display_paused = False
        self._history: list[TrendSample] = []
        self._last_trend_token: object | None = None
        self._trend_window_s: float | None = float(self._ui_settings.get("default_window_s", 30.0) or 30.0)
        self._trend_buffer_size = max(10, int(self._ui_settings.get("trend_buffer_size", 6000) or 6000))
        self._status_labels: dict[str, QLabel] = {}
        self._metric_labels: dict[str, QLabel] = {}
        self._image_panels: dict[str, CameraPanel] = {}
        self._build_ui()
        self._bridge = None
        self._image_timer = QTimer(self)
        self._image_timer.setInterval(max(50, int(1000.0 / max(1.0, self._image_hz()))))
        self._image_timer.timeout.connect(self._poll_images)
        self._image_timer.start()
        if start_bridge:
            self._bridge = PoseSimulationRosBridge(self._namespace, self._ui_settings, parent=self)
            self._bridge.snapshot_ready.connect(self.update_snapshot)
            self._bridge.diagnostics_ready.connect(self.update_diagnostics)
            self._bridge.bridge_error.connect(self._show_bridge_error)
        self._render_initial_state()

    def _camera_settings(self) -> Mapping[str, Any]:
        value = self._ui_settings.get("camera", {})
        settings: dict[str, Any] = {
            "enabled": True,
            "left_topic": DEFAULT_LEFT_IMAGE_TOPIC,
            "right_topic": DEFAULT_RIGHT_IMAGE_TOPIC,
            "message_type": "sensor_msgs/msg/Image",
            "display_max_hz": 20.0,
        }
        if isinstance(value, Mapping):
            settings.update(value)
        return settings

    def _image_hz(self) -> float:
        try:
            return max(1.0, float(self._camera_settings().get("display_max_hz", 20.0)))
        except (TypeError, ValueError):
            return 20.0

    def _apply_style(self) -> None:
        palette = QApplication.palette()
        text = palette.color(QPalette.Text).name()
        muted = palette.color(QPalette.Mid).name()
        accent = palette.color(QPalette.Highlight).name()
        self.setStyleSheet(
            "QGroupBox { font-weight: 600; margin-top: 10px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }"
            f"QLabel#sectionTitle {{ color: {text}; font-weight: 700; }}"
            f"QLabel#mutedText {{ color: {muted}; }}"
            f"QLabel#heroValue {{ color: {accent}; font-size: 16px; font-weight: 700; }}"
            "QTableWidget { alternate-background-color: palette(alternate-base); }"
            "QPlainTextEdit { font-family: monospace; }"
        )

    def _build_ui(self) -> None:
        self._apply_style()
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)
        root.addWidget(self._build_header())
        self._tabs = QTabWidget()
        self._tabs.addTab(self._build_overview_tab(), "Overview")
        self._tabs.addTab(self._build_camera_tab(), "Camera")
        self._tabs.addTab(self._build_pose_error_tab(), "Pose Error")
        self._tabs.addTab(self._build_stereo_geometry_tab(), "Stereo Geometry")
        # Escape the ampersand so Qt does not turn it into a mnemonic marker.
        self._tabs.addTab(self._build_reference_time_tab(), "Reference && Time")
        self._tabs.addTab(self._build_trends_tab(), "Trends")
        self._tabs.addTab(self._build_diagnostics_tab(), "Diagnostics")
        root.addWidget(self._tabs, 1)

    def _build_header(self) -> QWidget:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(10, 7, 10, 7)
        first = QHBoxLayout()
        app = QLabel("PoseSimulation")
        app.setObjectName("sectionTitle")
        first.addWidget(app)
        self._status_labels["overall"] = QLabel("Waiting for sources")
        self._status_labels["overall"].setObjectName("heroValue")
        first.addWidget(self._status_labels["overall"])
        first.addStretch(1)
        self._status_labels["readonly"] = QLabel("Read-only")
        self._status_labels["readonly"].setToolTip("Diagnostic only — no pose command publisher")
        first.addWidget(self._status_labels["readonly"])
        layout.addLayout(first)
        second = QHBoxLayout()
        for key, text in (("evaluation", "Evaluation —"), ("sim_time", "Sim time —"), ("gt", "GT —"), ("left", "Left —"), ("right", "Right —"), ("frames", "Frames —")):
            self._status_labels[key] = QLabel(text)
            second.addWidget(self._status_labels[key])
        second.addStretch(1)
        layout.addLayout(second)
        return frame

    def _metric_group(self, title: str, specs: Sequence[tuple[str, str]], *, hero: str | None = None) -> QGroupBox:
        group = QGroupBox(title)
        layout = QGridLayout(group)
        layout.setColumnStretch(1, 1)
        for row, (key, label_text) in enumerate(specs):
            label = QLabel(label_text)
            value = QLabel("—")
            value.setObjectName("heroValue" if key == hero else "metricValue")
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self._metric_labels[key] = value
            layout.addWidget(label, row, 0)
            layout.addWidget(value, row, 1)
        return group

    def _build_overview_tab(self) -> QWidget:
        widget = QWidget()
        layout = QGridLayout(widget)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.addWidget(self._metric_group("Left camera", (("ov_l_yaw", "Yaw error"), ("ov_l_pitch", "Pitch error"), ("ov_l_roll", "Roll error"), ("ov_l_rot", "Total rotation"), ("ov_l_los", "Target LOS"), ("ov_l_valid", "Validity")),), 0, 0)
        layout.addWidget(self._metric_group("Right camera", (("ov_r_yaw", "Yaw error"), ("ov_r_pitch", "Pitch error"), ("ov_r_roll", "Roll error"), ("ov_r_rot", "Total rotation"), ("ov_r_los", "Target LOS"), ("ov_r_valid", "Validity")),), 0, 1)
        layout.addWidget(self._metric_group("Stereo alignment", (("ov_rel_yaw", "Relative yaw"), ("ov_rel_rot", "Relative rotation"), ("ov_axis", "Optical-axis separation"), ("ov_base_len", "Baseline length"), ("ov_base_dir", "Baseline direction error"), ("ov_stereo_valid", "Validity")), hero="ov_rel_yaw"), 1, 0, 1, 2)
        reason = QGroupBox("Current reason")
        reason_layout = QVBoxLayout(reason)
        self._overview_reason = QLabel("Waiting for aligned pose samples")
        self._overview_reason.setWordWrap(True)
        reason_layout.addWidget(self._overview_reason)
        layout.addWidget(reason, 2, 0, 1, 2)
        layout.setRowStretch(3, 1)
        return widget

    def _build_camera_tab(self) -> QWidget:
        widget = QWidget()
        layout = QGridLayout(widget)
        camera = self._camera_settings()
        left_topic = str(camera.get("left_topic", DEFAULT_LEFT_IMAGE_TOPIC) or "")
        right_topic = str(camera.get("right_topic", DEFAULT_RIGHT_IMAGE_TOPIC) or "")
        self._image_panels["left"] = CameraPanel("Left camera", left_topic)
        self._image_panels["right"] = CameraPanel("Right camera", right_topic)
        layout.addWidget(self._image_panels["left"], 0, 0)
        layout.addWidget(self._image_panels["right"], 0, 1)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.setRowStretch(0, 1)
        return widget

    def _configure_table(self, table: QTableWidget) -> None:
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionMode(QTableWidget.NoSelection)
        table.verticalHeader().setVisible(False)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setAlternatingRowColors(True)

    def _build_pose_error_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        self._pose_error_table = QTableWidget(6, 4)
        self._pose_error_table.setHorizontalHeaderLabels(["Metric", "Left", "Right", "Validity"])
        self._configure_table(self._pose_error_table)
        for row, name in enumerate(("Roll error", "Pitch error", "Yaw error", "Total rotation", "Target LOS", "Gimbal lock")):
            self._pose_error_table.setItem(row, 0, QTableWidgetItem(name))
        layout.addWidget(self._pose_error_table)
        contract = QLabel("Error rotation: R_error = R_idealᵀ · R_actual\nFrame: ideal camera optical RDF; angles are degrees.")
        contract.setObjectName("mutedText")
        contract.setWordWrap(True)
        layout.addWidget(contract)
        return widget

    def _build_stereo_geometry_tab(self) -> QWidget:
        widget = QWidget()
        layout = QGridLayout(widget)
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.addWidget(self._metric_group("Relative orientation", (("sg_roll", "Relative roll"), ("sg_pitch", "Relative pitch"), ("sg_yaw", "Relative yaw"), ("sg_rot", "Relative rotation"), ("sg_axis", "Optical-axis separation")), hero="sg_yaw"), 0, 0)
        layout.addWidget(self._metric_group("Baseline", (("sg_actual", "Actual length"), ("sg_nominal", "Nominal length"), ("sg_len_err", "Length error"), ("sg_dir_err", "Direction error"), ("sg_midpoint", "Actual midpoint E/N/U")),), 0, 1)
        layout.addWidget(self._metric_group("Optical alignment", (("sg_l_los", "Left target LOS"), ("sg_r_los", "Right target LOS")),), 1, 0)
        layout.addWidget(self._metric_group("Interpretation", (("sg_rig_source", "Rig center source"), ("sg_alignment", "Target point alignment"), ("sg_lever", "Lever arm applied")),), 1, 1)
        return widget

    def _build_reference_time_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(self._metric_group("Ground Truth", (("ref_gt_topic", "Source topic"), ("ref_gt_name", "Reference name"), ("ref_gt_frame", "World frame"), ("ref_gt_state", "State")),))
        layout.addWidget(self._metric_group("Camera sources", (("ref_left_topic", "Left pose topic"), ("ref_right_topic", "Right pose topic"), ("ref_camera_mode", "Pose source mode"), ("ref_camera_state", "State")),))
        layout.addWidget(self._metric_group("RTK operational reference", (("ref_rtk_state", "State"), ("ref_rtk_note", "Availability")),))
        self._time_table = QTableWidget(4, 4)
        self._time_table.setHorizontalHeaderLabels(["Source", "State", "Residual", "Method"])
        self._configure_table(self._time_table)
        for row, name in enumerate(("GT", "Left camera", "Right camera", "RTK")):
            self._time_table.setItem(row, 0, QTableWidgetItem(name))
        layout.addWidget(QLabel("Clock domain: Gazebo simulation time. Raw nanosecond stamps are in Diagnostics."))
        layout.addWidget(self._time_table)
        layout.addStretch(1)
        return widget

    def _build_trends_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        controls = QHBoxLayout()
        controls.addWidget(QLabel("Window"))
        self._trend_combo = QComboBox()
        for label, value in (("10 s", 10.0), ("30 s", 30.0), ("60 s", 60.0), ("Full available buffer", None)):
            self._trend_combo.addItem(label, value)
        self._trend_combo.setCurrentIndex(1)
        self._trend_combo.currentIndexChanged.connect(self._trend_window_changed)
        controls.addWidget(self._trend_combo)
        controls.addWidget(QLabel("Baseline"))
        self._baseline_combo = QComboBox()
        self._baseline_combo.addItem("Length error", "length")
        self._baseline_combo.addItem("Direction error", "direction")
        self._baseline_combo.currentIndexChanged.connect(self._baseline_metric_changed)
        controls.addWidget(self._baseline_combo)
        self._pause_button = QPushButton("Pause display")
        self._pause_button.setCheckable(True)
        self._pause_button.toggled.connect(self._trend_pause_changed)
        controls.addWidget(self._pause_button)
        controls.addStretch(1)
        layout.addLayout(controls)
        grid = QGridLayout()
        self._plots = {
            "relative": TrendPlot("Relative attitude", "deg"),
            "optical": TrendPlot("Optical alignment", "deg"),
            "time": TrendPlot("Time alignment", "ms"),
            "baseline": TrendPlot("Baseline stability", "m"),
        }
        grid.addWidget(self._plots["relative"], 0, 0)
        grid.addWidget(self._plots["optical"], 0, 1)
        grid.addWidget(self._plots["time"], 1, 0)
        grid.addWidget(self._plots["baseline"], 1, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        layout.addLayout(grid, 1)
        return widget

    def _build_diagnostics_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        controls = QHBoxLayout()
        controls.addStretch(1)
        self._copy_button = QPushButton("Copy diagnostics")
        self._copy_button.clicked.connect(self._copy_diagnostics)
        controls.addWidget(self._copy_button)
        layout.addLayout(controls)
        self._diagnostic_text = QPlainTextEdit()
        self._diagnostic_text.setReadOnly(True)
        self._diagnostic_text.setLineWrapMode(QPlainTextEdit.NoWrap)
        layout.addWidget(self._diagnostic_text, 1)
        return widget

    def _render_initial_state(self) -> None:
        initial = build_ui_snapshot(None, None, None)
        self._latest_snapshot = initial
        self._snapshot = initial
        self._refresh_all()
        for panel in self._image_panels.values():
            panel.set_unavailable()

    @Slot(object)
    def update_snapshot(self, snapshot: UiSnapshot) -> None:
        self._latest_snapshot = snapshot
        if snapshot.valid:
            self._last_valid_snapshot = snapshot
        self._record_trend(snapshot)
        if self._display_paused:
            return
        self._snapshot = snapshot
        self._refresh_all()

    def _record_trend(self, snapshot: UiSnapshot) -> None:
        token = snapshot.token
        if token == self._last_trend_token:
            return
        self._last_trend_token = token
        values = {
            "relative_roll": snapshot.stereo.relative_roll_deg,
            "relative_pitch": snapshot.stereo.relative_pitch_deg,
            "relative_yaw": snapshot.stereo.relative_yaw_deg,
            "left_los": snapshot.pose_error.los_deg,
            "right_los": snapshot.right_error.los_deg,
            "gt_residual": None,
            "left_residual": None,
            "right_residual": None,
            "rtk_residual": None,
            "baseline_length": snapshot.stereo.baseline_length_error_m,
            "baseline_direction": snapshot.stereo.baseline_direction_error_deg,
        }
        append_sample(
            self._history,
            timestamp_s=snapshot.sim_time_s,
            values=values,
            valid=snapshot.valid,
            token=token,
            maxlen=self._trend_buffer_size,
        )

    @Slot(object)
    def update_diagnostics(self, records: Sequence[Mapping[str, Any]]) -> None:
        self._diagnostics = tuple(records)
        self._refresh_diagnostics()

    @Slot(str)
    def _show_bridge_error(self, message: str) -> None:
        self._bridge_error = str(message)
        self._refresh_diagnostics()

    def _refresh_all(self) -> None:
        self._refresh_header()
        self._refresh_overview()
        self._refresh_pose_error()
        self._refresh_stereo_geometry()
        self._refresh_reference_time()
        self._refresh_trends()
        self._refresh_diagnostics()

    def _set_badge(self, key: str, prefix: str, state: str) -> None:
        label = self._status_labels[key]
        label.setText(f"{prefix} {state_label(state)}")
        value = str(state or "UNAVAILABLE").upper()
        palette = QApplication.palette()
        if value == "VALID":
            color = "#2e8b57"
        elif value in {"UNALIGNED", "WARNING", "WAITING"}:
            color = "#a56a00"
        elif value in {"INVALID"}:
            color = "#b53b3b"
        else:
            color = palette.color(QPalette.Mid).name()
        label.setStyleSheet(f"color: {color}; font-weight: 600;")

    def _refresh_header(self) -> None:
        snapshot = self._snapshot
        if snapshot is None:
            return
        self._status_labels["overall"].setText(snapshot.overall_label)
        self._status_labels["evaluation"].setText(f"Evaluation {snapshot.evaluation_id}" if snapshot.evaluation_id is not None else "Evaluation —")
        self._status_labels["sim_time"].setText(f"Sim time {snapshot.sim_time_s:.3f} s" if snapshot.sim_time_s is not None else "Sim time —")
        self._set_badge("gt", "GT", snapshot.gt_state)
        self._set_badge("left", "Left", snapshot.left_state)
        self._set_badge("right", "Right", snapshot.right_state)
        self._set_badge("frames", "Frames", snapshot.frame_state)

    def _set_metric(self, key: str, value: Any, *, unit: str = "", precision: int = 3) -> None:
        label = self._metric_labels.get(key)
        if label is not None:
            label.setText(format_metric(value, unit=unit, precision=precision))

    def _set_text(self, key: str, value: Any, *, unavailable: str = "—") -> None:
        label = self._metric_labels.get(key)
        if label is not None:
            text = str(value).strip() if value is not None else ""
            label.setText(text if text else unavailable)

    def _refresh_overview(self) -> None:
        snapshot = self._snapshot
        if snapshot is None:
            return
        left, right, stereo = snapshot.pose_error, snapshot.right_error, snapshot.stereo
        for key, value in (("ov_l_yaw", left.yaw_deg), ("ov_l_pitch", left.pitch_deg), ("ov_l_roll", left.roll_deg), ("ov_l_rot", left.rotation_deg), ("ov_l_los", left.los_deg)):
            self._set_metric(key, value, unit="°")
        for key, value in (("ov_r_yaw", right.yaw_deg), ("ov_r_pitch", right.pitch_deg), ("ov_r_roll", right.roll_deg), ("ov_r_rot", right.rotation_deg), ("ov_r_los", right.los_deg)):
            self._set_metric(key, value, unit="°")
        for key, value, unit in (("ov_rel_yaw", stereo.relative_yaw_deg, "°"), ("ov_rel_rot", stereo.relative_rotation_deg, "°"), ("ov_axis", stereo.optical_axis_separation_deg, "°"), ("ov_base_len", stereo.actual_baseline_length_m, " m"), ("ov_base_dir", stereo.baseline_direction_error_deg, "°")):
            self._set_metric(key, value, unit=unit)
        self._set_text("ov_l_valid", source_state_label(snapshot.left_state))
        self._set_text("ov_r_valid", source_state_label(snapshot.right_state))
        self._set_text("ov_stereo_valid", source_state_label("VALID" if stereo.valid else snapshot.overall_state))
        self._overview_reason.setText(snapshot.reason or "Ready")

    def _set_table_cell(self, table: QTableWidget, row: int, column: int, text: str) -> None:
        item = table.item(row, column)
        if item is None:
            item = QTableWidgetItem()
            table.setItem(row, column, item)
        item.setText(text)
        item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter if column > 0 else Qt.AlignLeft | Qt.AlignVCenter)

    def _refresh_pose_error(self) -> None:
        snapshot = self._snapshot
        if snapshot is None:
            return
        left, right = snapshot.pose_error, snapshot.right_error
        pairs = ((left.roll_deg, right.roll_deg), (left.pitch_deg, right.pitch_deg), (left.yaw_deg, right.yaw_deg), (left.rotation_deg, right.rotation_deg), (left.los_deg, right.los_deg))
        for row, (left_value, right_value) in enumerate(pairs):
            self._set_table_cell(self._pose_error_table, row, 1, format_metric(left_value, unit="°"))
            self._set_table_cell(self._pose_error_table, row, 2, format_metric(right_value, unit="°"))
        self._set_table_cell(self._pose_error_table, 5, 1, "Yes" if left.gimbal_lock else "No" if left.valid else "—")
        self._set_table_cell(self._pose_error_table, 5, 2, "Yes" if right.gimbal_lock else "No" if right.valid else "—")
        validity = f"{source_state_label(snapshot.left_state)} / {source_state_label(snapshot.right_state)}"
        for row in range(6):
            self._set_table_cell(self._pose_error_table, row, 3, validity)

    def _refresh_stereo_geometry(self) -> None:
        snapshot = self._snapshot
        if snapshot is None:
            return
        stereo = snapshot.stereo
        for key, value in (("sg_roll", stereo.relative_roll_deg), ("sg_pitch", stereo.relative_pitch_deg), ("sg_yaw", stereo.relative_yaw_deg), ("sg_rot", stereo.relative_rotation_deg), ("sg_axis", stereo.optical_axis_separation_deg), ("sg_actual", stereo.actual_baseline_length_m), ("sg_nominal", stereo.nominal_baseline_length_m), ("sg_len_err", stereo.baseline_length_error_m), ("sg_dir_err", stereo.baseline_direction_error_deg)):
            self._set_metric(key, value, unit=" m" if key in {"sg_actual", "sg_nominal", "sg_len_err"} else "°")
        midpoint = stereo.actual_midpoint_world_enu_m
        if all(value is not None for value in midpoint):
            self._set_text("sg_midpoint", "E/N/U " + "/".join(f"{value:.3f}" for value in midpoint))
        else:
            self._set_text("sg_midpoint", "—")
        self._set_metric("sg_l_los", snapshot.pose_error.los_deg, unit="°")
        self._set_metric("sg_r_los", snapshot.right_error.los_deg, unit="°")
        self._set_text("sg_rig_source", "actual optical midpoint")
        self._set_text("sg_alignment", "Not provided by current typed result")
        self._set_text("sg_lever", "Not provided by current typed result")

    def _refresh_reference_time(self) -> None:
        snapshot = self._snapshot
        if snapshot is None:
            return
        status = snapshot.status
        self._set_text("ref_gt_topic", snapshot.source_topics.get("gt"))
        self._set_text("ref_gt_name", status.get("gt_reference_name"))
        self._set_text("ref_gt_frame", WORLD_FRAME)
        self._set_text("ref_gt_state", source_state_label(snapshot.gt_state))
        self._set_text("ref_left_topic", snapshot.source_topics.get("left"))
        self._set_text("ref_right_topic", snapshot.source_topics.get("right"))
        self._set_text("ref_camera_mode", snapshot.camera_source_mode or "—")
        self._set_text("ref_camera_state", f"{source_state_label(snapshot.left_state)} / {source_state_label(snapshot.right_state)}")
        self._set_text("ref_rtk_state", "Unavailable")
        self._set_text("ref_rtk_note", "RTK comparison is not implemented in M3")
        values = (
            ("Exact" if snapshot.valid else "—", "exact" if snapshot.valid else "—"),
            ("—", "Static run snapshot" if snapshot.camera_source_mode == "static_snapshot" else "—"),
            ("—", "Static run snapshot" if snapshot.camera_source_mode == "static_snapshot" else "—"),
            ("—", "Unavailable"),
        )
        states = (snapshot.gt_state, snapshot.left_state, snapshot.right_state, "UNAVAILABLE")
        for row, ((residual, method), state) in enumerate(zip(values, states)):
            self._set_table_cell(self._time_table, row, 1, source_state_label(state))
            self._set_table_cell(self._time_table, row, 2, residual)
            self._set_table_cell(self._time_table, row, 3, method)

    def _series(self, keys: Sequence[tuple[str, str]]) -> Mapping[str, Sequence[tuple[float, float | None]]]:
        return {label: series_for_key(self._history, key) for key, label in keys}

    def _refresh_trends(self) -> None:
        if not hasattr(self, "_plots"):
            return
        self._plots["relative"].set_data(
            self._series((("relative_roll", "Roll"), ("relative_pitch", "Pitch"), ("relative_yaw", "Yaw"))),
            self._trend_window_s,
            empty_reason="Waiting for aligned pose samples",
        )
        self._plots["optical"].set_data(
            self._series((("left_los", "Left"), ("right_los", "Right"))),
            self._trend_window_s,
            empty_reason="Waiting for valid optical-axis samples",
        )
        self._plots["time"].set_data(
            self._series((("gt_residual", "GT"), ("left_residual", "Left"), ("right_residual", "Right"), ("rtk_residual", "RTK"))),
            self._trend_window_s,
            empty_reason="Source residuals are not provided by the current result message",
        )
        baseline_key = self._baseline_combo.currentData() if hasattr(self, "_baseline_combo") else "length"
        if baseline_key == "direction":
            self._plots["baseline"].set_data(self._series((("baseline_direction", "Direction"),)), self._trend_window_s, title="Baseline stability", unit="deg", empty_reason="Waiting for valid baseline samples")
        else:
            self._plots["baseline"].set_data(self._series((("baseline_length", "Length"),)), self._trend_window_s, title="Baseline stability", unit="m", empty_reason="Waiting for valid baseline samples")

    def _diagnostic_lines(self) -> list[str]:
        snapshot = self._snapshot
        lines = [
            "PoseSimulation — read-only diagnostics",
            "Diagnostic only — no pose command publisher",
            "",
            f"Overall state: {snapshot.overall_label if snapshot else 'Waiting for sources'}",
            f"Raw state: {snapshot.raw_state if snapshot else 'UNAVAILABLE'}",
            f"Reason: {snapshot.reason if snapshot else 'Waiting for PoseSimulation sources'}",
            f"Evaluation ID: {snapshot.evaluation_id if snapshot else '—'}",
            f"Evaluation stamp ns: {snapshot.evaluation_stamp_ns if snapshot else '—'}",
            f"Clock domain: {snapshot.clock_domain if snapshot else 'Unresolved clock domain'}",
            f"Latest source sample: {self._latest_snapshot.overall_label if self._latest_snapshot else '—'}",
            f"Last valid sample: {self._last_valid_snapshot.evaluation_id if self._last_valid_snapshot else '—'}",
            f"Displayed sample: {self._snapshot.evaluation_id if self._snapshot else '—'}",
            f"GT frame: {WORLD_FRAME}",
            f"Rig frame: {RIG_FRAME}",
            f"Camera optical frame: {OPTICAL_FRAME}",
            "Quaternion order: ROS xyzw",
            "Results message types: PoseError, StereoGeometry, PoseSimulationStatus",
            "Results QoS: reliable / volatile / depth 10",
            "Image QoS: sensor-data profile (latest-frame-wins)",
            f"ROS bridge: {'active' if self._bridge is not None else 'unavailable or disabled'}",
            "",
            "Configured image sources:",
        ]
        camera = self._camera_settings()
        lines.extend(
            [
                f"  left: {camera.get('left_topic', '') or '<not configured>'}",
                f"  right: {camera.get('right_topic', '') or '<not configured>'}",
                f"  message type: {camera.get('message_type', 'sensor_msgs/msg/Image')}",
                f"  display max Hz: {camera.get('display_max_hz', 20.0)}",
            ]
        )
        if snapshot:
            lines.extend(
                [
                    "",
                    "Source topics:",
                    f"  GT: {snapshot.source_topics.get('gt') or '—'}",
                    f"  left pose: {snapshot.source_topics.get('left') or '—'}",
                    f"  right pose: {snapshot.source_topics.get('right') or '—'}",
                    f"Camera source mode: {snapshot.camera_source_mode or '—'}",
                    f"GT source stamp ns: {snapshot.source_stamp_ns.get('gt')}",
                    f"Left source stamp ns: {snapshot.source_stamp_ns.get('left')}",
                    f"Right source stamp ns: {snapshot.source_stamp_ns.get('right')}",
                ]
            )
            lines.extend(f"Status {key}: {value}" for key, value in snapshot.status.items())
            lines.extend(f"PoseError {key}: {value}" for key, value in snapshot.pose_error_raw.items())
            lines.extend(f"StereoGeometry {key}: {value}" for key, value in snapshot.stereo_raw.items())
        if self._bridge_error:
            lines.extend(("", f"Bridge: {self._bridge_error}"))
        if self._diagnostics:
            lines.append("")
            lines.append("DiagnosticArray:")
            for record in self._diagnostics:
                lines.append(f"  [{record.get('level', 0)}] {record.get('name', '')}: {record.get('message', '')}")
                for key, value in record.get("values", ()):
                    lines.append(f"      {key}={value}")
        return lines

    def _refresh_diagnostics(self) -> None:
        if hasattr(self, "_diagnostic_text"):
            self._diagnostic_text.setPlainText("\n".join(self._diagnostic_lines()))

    def _copy_diagnostics(self) -> None:
        QApplication.clipboard().setText("\n".join(self._diagnostic_lines()))

    def _poll_images(self) -> None:
        if self._bridge is None:
            return
        for side, frame in self._bridge.take_latest_frames().items():
            panel = self._image_panels.get(side)
            if panel is not None:
                panel.set_frame(frame)

    @Slot(int)
    def _trend_window_changed(self, index: int) -> None:
        self._trend_window_s = self._trend_combo.itemData(index)
        if not self._display_paused:
            self._refresh_trends()

    @Slot(int)
    def _baseline_metric_changed(self, index: int) -> None:
        if not self._display_paused:
            self._refresh_trends()

    @Slot(bool)
    def _trend_pause_changed(self, paused: bool) -> None:
        self._display_paused = bool(paused)
        self._pause_button.setText("Resume display" if paused else "Pause display")
        if not paused:
            self._snapshot = self._latest_snapshot
            self._refresh_all()

    def shutdown(self) -> None:
        if hasattr(self, "_image_timer"):
            self._image_timer.stop()
        if self._bridge is not None:
            self._bridge.shutdown()
            self._bridge = None


__all__ = ["ImageFrame", "PoseSimulationRosBridge", "PoseSimulationWidget", "TrendPlot"]
