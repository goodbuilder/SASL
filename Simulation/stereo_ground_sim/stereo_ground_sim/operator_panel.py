import math
import threading
import time
from collections import deque

from python_qt_binding.QtCore import QObject, Qt, Signal
from python_qt_binding.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from python_qt_binding.QtWidgets import QGridLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget
from qt_gui.plugin import Plugin

try:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray
except Exception:  # pragma: no cover - rqt can still load the widget shell.
    rclpy = None
    qos_profile_sensor_data = None
    Image = None
    Float32MultiArray = None


AXES = (
    ("Forward", 0),
    ("Right", 1),
    ("Down", 2),
)
SERIES = (
    ("truth", "/ground_truth_position", QColor(32, 86, 255)),
    ("raw", "/stereo/estimated_position", QColor(220, 32, 32)),
    ("corrected", "/stereo/corrected_position", QColor(0, 170, 170)),
)
IMAGE_TOPICS = (
    ("Left Camera", "/left_camera/detection_image"),
    ("Right Camera", "/right_camera/detection_image"),
)


class PanelBridge(QObject):
    image = Signal(str, object)
    sample = Signal(str, list)
    status = Signal(str)


class CameraView(QLabel):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self._title = title
        self._pixmap = None
        self.setMinimumSize(560, 300)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background: #202428; color: #d7dee5; font-weight: 600;")
        self.setText(f"{title}\nwaiting for image")

    def set_image(self, payload):
        width, height, encoding, step, data = payload
        qimage = self._to_qimage(width, height, encoding, step, data)
        if qimage is None:
            self.setText(f"{self._title}\nunsupported image encoding: {encoding}")
            return
        self._pixmap = QPixmap.fromImage(qimage)
        self._refresh_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_pixmap()

    def _refresh_pixmap(self):
        if self._pixmap is None:
            return
        scaled = self._pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.setPixmap(scaled)

    def _to_qimage(self, width, height, encoding, step, data):
        if width <= 0 or height <= 0:
            return None
        if encoding in ("rgb8", "bgr8"):
            channels = 3
            fmt = QImage.Format_RGB888
        elif encoding in ("rgba8", "bgra8"):
            channels = 4
            fmt = QImage.Format_RGBA8888
        elif encoding in ("mono8", "8UC1"):
            channels = 1
            fmt = QImage.Format_Grayscale8
        else:
            return None
        min_step = width * channels
        if step < min_step:
            return None
        rows = []
        raw = bytes(data)
        for row in range(height):
            start = row * step
            rows.append(raw[start : start + min_step])
        packed = b"".join(rows)
        if encoding == "bgr8":
            packed = bytes(
                channel
                for pixel_start in range(0, len(packed), 3)
                for channel in (packed[pixel_start + 2], packed[pixel_start + 1], packed[pixel_start])
            )
        elif encoding == "bgra8":
            packed = bytes(
                channel
                for pixel_start in range(0, len(packed), 4)
                for channel in (
                    packed[pixel_start + 2],
                    packed[pixel_start + 1],
                    packed[pixel_start],
                    packed[pixel_start + 3],
                )
            )
        return QImage(packed, width, height, min_step, fmt).copy()


class FrdPlotStack(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(760, 900)
        self.samples = {
            axis_name: {series_name: deque(maxlen=900) for series_name, _, _ in SERIES}
            for axis_name, _ in AXES
        }

    def append(self, series_name, values):
        now = time.monotonic()
        for axis_name, axis_index in AXES:
            value = math.nan
            if len(values) > axis_index:
                try:
                    value = float(values[axis_index])
                except (TypeError, ValueError):
                    value = math.nan
            self.samples[axis_name][series_name].append((now, value))
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(250, 250, 250))
        painter.setRenderHint(QPainter.Antialiasing, True)
        margin_left = 68
        margin_right = 18
        margin_top = 34
        margin_bottom = 28
        gap = 26
        width = max(1, self.width() - margin_left - margin_right)
        pane_height = max(1, (self.height() - margin_top - margin_bottom - gap * 2) / 3)

        now = time.monotonic()
        window_s = 30.0
        for pane_index, (axis_name, _axis_index) in enumerate(AXES):
            top = int(margin_top + pane_index * (pane_height + gap))
            height = int(pane_height)
            left = margin_left
            right = margin_left + width
            bottom = top + height
            finite_values = []
            for series_name, _topic, _color in SERIES:
                finite_values.extend(
                    value
                    for stamp, value in self.samples[axis_name][series_name]
                    if now - stamp <= window_s and math.isfinite(value)
                )
            if finite_values:
                ymin = min(finite_values)
                ymax = max(finite_values)
            else:
                ymin, ymax = -1.0, 1.0
            if abs(ymax - ymin) < 1e-6:
                ymin -= 1.0
                ymax += 1.0
            pad = 0.08 * (ymax - ymin)
            ymin -= pad
            ymax += pad

            painter.setPen(QPen(QColor(210, 210, 210), 1))
            painter.drawRect(left, top, width, height)
            for tick in range(1, 5):
                y = int(top + height * tick / 5)
                painter.drawLine(left, y, right, y)
            for tick in range(1, 6):
                x = int(left + width * tick / 6)
                painter.drawLine(x, top, x, bottom)

            painter.setPen(QPen(QColor(32, 32, 32), 1))
            painter.setFont(QFont("Sans", 11, QFont.Bold))
            painter.drawText(left, top - 10, f"{axis_name} axis, stereo-center FRD meters")
            painter.setFont(QFont("Sans", 9))
            painter.drawText(8, top + 14, f"{ymax:.1f}")
            painter.drawText(8, bottom, f"{ymin:.1f}")

            legend_x = left + 310
            painter.setFont(QFont("Sans", 10))
            for series_name, _topic, color in SERIES:
                painter.setPen(QPen(color, 3))
                painter.drawLine(legend_x, top - 14, legend_x + 24, top - 14)
                painter.setPen(QPen(QColor(32, 32, 32), 1))
                painter.drawText(legend_x + 30, top - 9, series_name)
                legend_x += 116

            for series_name, _topic, color in SERIES:
                points = [
                    (stamp, value)
                    for stamp, value in self.samples[axis_name][series_name]
                    if now - stamp <= window_s and math.isfinite(value)
                ]
                if len(points) == 1:
                    stamp, value = points[0]
                    x = int(right - (now - stamp) / window_s * width)
                    y = int(bottom - (value - ymin) / (ymax - ymin) * height)
                    painter.setPen(QPen(color, 4))
                    painter.drawPoint(x, y)
                elif len(points) > 1:
                    painter.setPen(QPen(color, 2))
                    last_xy = None
                    for stamp, value in points:
                        x = int(right - (now - stamp) / window_s * width)
                        y = int(bottom - (value - ymin) / (ymax - ymin) * height)
                        if last_xy is not None:
                            painter.drawLine(last_xy[0], last_xy[1], x, y)
                        last_xy = (x, y)
            if not finite_values:
                painter.setPen(QPen(QColor(110, 110, 110), 1))
                painter.setFont(QFont("Sans", 12))
                painter.drawText(left + 12, top + 34, "No finite FRD samples in the last 30s")


class OperatorPanel(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("OperatorPanel")
        self._ros_node = None
        self._spin_thread = None
        self._stop_event = threading.Event()
        self._bridge = PanelBridge()

        self._widget = QWidget()
        self._widget.setWindowTitle("Stereo Operator Panel")
        root = QVBoxLayout(self._widget)
        self._status = QLabel(
            "Image left: left camera over right camera. Graphs right: Forward, Right, Down. "
            "FRD contract data[0]=Forward, data[1]=Right, data[2]=Down."
        )
        self._status.setStyleSheet("font-weight: 600;")
        root.addWidget(self._status)

        grid = QGridLayout()
        grid.setColumnStretch(0, 5)
        grid.setColumnStretch(1, 7)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        self._left_view = CameraView("Left Camera")
        self._right_view = CameraView("Right Camera")
        self._plot_stack = FrdPlotStack()
        grid.addWidget(self._left_view, 0, 0)
        grid.addWidget(self._right_view, 1, 0)
        grid.addWidget(self._plot_stack, 0, 1, 2, 1)
        root.addLayout(grid, 1)

        context.add_widget(self._widget)
        self._bridge.image.connect(self._handle_image)
        self._bridge.sample.connect(self._plot_stack.append)
        self._bridge.status.connect(self._status.setText)
        self._init_ros()

    def _handle_image(self, name, payload):
        if name == "left":
            self._left_view.set_image(payload)
        elif name == "right":
            self._right_view.set_image(payload)

    def _init_ros(self):
        if rclpy is None or Image is None or Float32MultiArray is None:
            self._status.setText("ROS imports unavailable; operator panel cannot subscribe.")
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._ros_node = rclpy.create_node("operator_panel")
            self._ros_node.create_subscription(
                Image,
                IMAGE_TOPICS[0][1],
                lambda msg: self._bridge.image.emit("left", self._image_payload(msg)),
                qos_profile_sensor_data,
            )
            self._ros_node.create_subscription(
                Image,
                IMAGE_TOPICS[1][1],
                lambda msg: self._bridge.image.emit("right", self._image_payload(msg)),
                qos_profile_sensor_data,
            )
            for series_name, topic, _color in SERIES:
                self._ros_node.create_subscription(
                    Float32MultiArray,
                    topic,
                    lambda msg, name=series_name: self._bridge.sample.emit(name, list(msg.data)),
                    10,
                )
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self._spin_thread.start()
        except Exception as exc:
            self._ros_node = None
            self._status.setText(f"Operator panel subscribe failed: {exc}")

    def _image_payload(self, msg):
        return (int(msg.width), int(msg.height), str(msg.encoding), int(msg.step), bytes(msg.data))

    def _spin_loop(self):
        if self._ros_node is None or rclpy is None:
            return
        while not self._stop_event.is_set():
            try:
                rclpy.spin_once(self._ros_node, timeout_sec=0.1)
            except Exception as exc:
                self._bridge.status.emit(f"Operator panel spin failed: {exc}")
                return

    def shutdown_plugin(self):
        self._stop_event.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
            self._ros_node = None
