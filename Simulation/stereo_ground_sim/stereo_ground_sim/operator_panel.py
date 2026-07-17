import json
import math
import os
import threading
import time
from collections import deque

from python_qt_binding.QtCore import QObject, Qt, QTimer, Signal
from python_qt_binding.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from python_qt_binding.QtWidgets import QApplication, QGridLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget
from qt_gui.plugin import Plugin

try:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32, Float32MultiArray, String
except Exception:  # pragma: no cover - rqt can still load the widget shell.
    rclpy = None
    qos_profile_sensor_data = None
    Image = None
    Float32 = None
    Float32MultiArray = None
    String = None


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
try:
    PLOT_WINDOW_S = max(1.0, float(os.environ.get("RQT_PLOT_WINDOW_S", "30.0")))
except ValueError:
    PLOT_WINDOW_S = 30.0


class PanelBridge(QObject):
    image = Signal(str, object)
    sample = Signal(str, list)
    scalar = Signal(str, object)
    detections = Signal(list)
    profile = Signal(str)
    status = Signal(str)


class CameraView(QLabel):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self._title = title
        self._pixmap = None
        self.setMinimumSize(480, 270)
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


class ExperimentHud(QWidget):
    def __init__(self, model_filename, parent=None):
        super().__init__(parent)
        self._latest = {
            "truth": [math.nan, math.nan, math.nan, math.nan],
            "raw": [math.nan, math.nan, math.nan, math.nan],
            "corrected": [math.nan, math.nan, math.nan, math.nan],
        }
        self._latest_stamp = {}
        self._error_history = {
            "raw_forward": deque(),
            "corrected_forward": deque(),
            "raw_down": deque(),
            "corrected_down": deque(),
        }
        self._detection_history = deque()
        self._latest_detection = {
            "valid": False,
            "disparity_px": math.nan,
            "confidence_min": math.nan,
        }
        self._latest_scalars = {
            "yolo_hz": math.nan,
            "yolo_ms": math.nan,
        }
        self._profile_text = "--"
        self._profile_not_configured = False

        mission = os.environ.get("RQT_MISSION_LABEL", "Mission: --").strip() or "Mission: --"
        contract = os.environ.get(
            "RQT_COORD_CONTRACT",
            "Frame: public FRD [Forward, Right, Down, Range]",
        ).strip()

        layout = QGridLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setHorizontalSpacing(18)
        layout.setVerticalSpacing(4)

        self._model_label = QLabel(f"PyTorch model: {model_filename}")
        self._mission_label = QLabel(mission)
        self._profile_label = QLabel("Profile: --")
        self._yolo_label = QLabel("YOLO: --")
        self._truth_label = QLabel("Truth F/R/D/R: --")
        self._raw_label = QLabel("Raw F/R/D/R: --")
        self._corrected_label = QLabel("Corrected F/R/D/R: --")
        self._forward_error_label = QLabel("Forward Δ raw/corr: --")
        self._down_error_label = QLabel("Down Δ raw/corr: --")
        self._detection_label = QLabel("Detections: --")
        self._contract_label = QLabel(contract)

        strong_labels = (
            self._model_label,
            self._mission_label,
            self._forward_error_label,
            self._down_error_label,
            self._detection_label,
        )
        for label in strong_labels:
            label.setStyleSheet("font-weight: 700; color: #101418;")
        for label in (
            self._profile_label,
            self._yolo_label,
            self._truth_label,
            self._raw_label,
            self._corrected_label,
            self._contract_label,
        ):
            label.setStyleSheet("font-weight: 600; color: #263238;")
        self._contract_label.setStyleSheet("font-weight: 600; color: #455a64;")

        layout.addWidget(self._model_label, 0, 0)
        layout.addWidget(self._mission_label, 0, 1)
        layout.addWidget(self._profile_label, 0, 2)
        layout.addWidget(self._yolo_label, 0, 3)
        layout.addWidget(self._truth_label, 1, 0)
        layout.addWidget(self._raw_label, 1, 1)
        layout.addWidget(self._corrected_label, 1, 2)
        layout.addWidget(self._detection_label, 1, 3)
        layout.addWidget(self._forward_error_label, 2, 0, 1, 2)
        layout.addWidget(self._down_error_label, 2, 2, 1, 2)
        layout.addWidget(self._contract_label, 3, 0, 1, 4)
        self.setStyleSheet(
            "ExperimentHud { background: #eef3f6; border: 1px solid #b0bec5; }"
            "QLabel { padding: 1px 2px; }"
        )
        self._refresh()

    def update_sample(self, series_name, values):
        now = time.monotonic()
        frd = []
        for index in range(4):
            value = math.nan
            if len(values) > index:
                try:
                    value = float(values[index])
                except (TypeError, ValueError):
                    value = math.nan
            frd.append(value)
        self._latest[series_name] = frd
        self._latest_stamp[series_name] = now
        if series_name in ("raw", "corrected"):
            self._append_error_sample(series_name, axis_index=0, axis_name="forward", now=now)
            self._append_error_sample(series_name, axis_index=2, axis_name="down", now=now)
        self._prune(now)
        self._refresh()

    def update_scalar(self, key, value):
        try:
            self._latest_scalars[key] = float(value)
            self._latest_stamp[key] = time.monotonic()
        except (TypeError, ValueError):
            self._latest_scalars[key] = math.nan
        self._refresh()

    def update_profile(self, payload):
        text = str(payload)
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            parsed = {}
        if isinstance(parsed, dict) and parsed:
            profile_id = str(parsed.get("profile_id") or "")
            mode = str(parsed.get("profile_mode") or parsed.get("mode") or "")
            fallback_reason = str(parsed.get("fallback_reason") or "")
            residual = parsed.get("disparity_residual_status") or ""
            selected = parsed.get("disparity_residual_feature_selected_label") or ""
            self._profile_not_configured = (
                not profile_id
                and not mode
                and fallback_reason == "no_profile"
            )
            if self._profile_not_configured:
                text = "NOT CONFIGURED — no profile"
            else:
                pieces = [f"{profile_id or 'unknown'}/{mode or 'unknown'}"]
                if fallback_reason:
                    pieces.append(f"reason={fallback_reason}")
                if selected:
                    pieces.append(f"selector={selected}")
                if residual:
                    pieces.append(str(residual))
                text = " ".join(pieces)
        else:
            self._profile_not_configured = False
        self._profile_text = text[:120]
        self._latest_stamp["profile"] = time.monotonic()
        self._refresh()

    def update_detections(self, values):
        now = time.monotonic()
        valid = False
        disparity = math.nan
        confidence_min = math.nan
        try:
            data = [float(value) for value in values]
        except (TypeError, ValueError):
            data = []
        if len(data) >= 16:
            left_valid = data[0] >= 0.5
            right_valid = data[8] >= 0.5
            left_conf = data[5]
            right_conf = data[13]
            left_cx = data[6]
            right_cx = data[14]
            disparity = left_cx - right_cx
            confidence_min = min(left_conf, right_conf)
            valid = (
                left_valid
                and right_valid
                and math.isfinite(disparity)
                and disparity > 0.5
                and math.isfinite(confidence_min)
            )
        self._latest_detection = {
            "valid": valid,
            "disparity_px": disparity,
            "confidence_min": confidence_min,
        }
        self._detection_history.append((now, bool(valid)))
        self._latest_stamp["detections"] = now
        self._prune(now)
        self._refresh()

    def _append_error_sample(self, series_name, axis_index, axis_name, now):
        truth_stamp = self._latest_stamp.get("truth")
        if truth_stamp is None or now - truth_stamp > 3.0:
            return
        truth = self._latest["truth"][axis_index]
        estimate = self._latest[series_name][axis_index]
        if not (math.isfinite(truth) and math.isfinite(estimate)):
            return
        key = f"{series_name}_{axis_name}"
        self._error_history[key].append((now, estimate - truth))

    def _prune(self, now):
        cutoff = now - PLOT_WINDOW_S
        for history in self._error_history.values():
            while history and history[0][0] < cutoff:
                history.popleft()
        while self._detection_history and self._detection_history[0][0] < cutoff:
            self._detection_history.popleft()

    def _rmse(self, key):
        values = [value for _stamp, value in self._error_history[key] if math.isfinite(value)]
        if not values:
            return math.nan
        return math.sqrt(sum(value * value for value in values) / len(values))

    def _latest_error(self, key):
        history = self._error_history[key]
        for _stamp, value in reversed(history):
            if math.isfinite(value):
                return value
        return math.nan

    def _valid_ratio(self):
        if not self._detection_history:
            return math.nan
        return sum(1 for _stamp, valid in self._detection_history if valid) / len(self._detection_history)

    def _format_float(self, value, precision=1, signed=False):
        if value is None or not math.isfinite(float(value)):
            return "--"
        sign = "+" if signed else ""
        return f"{float(value):{sign}.{precision}f}"

    def _format_frd(self, values):
        return "/".join(self._format_float(value, 1) for value in values[:4])

    def _refresh(self):
        self._profile_label.setText(f"Profile: {self._profile_text or '--'}")
        yolo_hz = self._format_float(self._latest_scalars.get("yolo_hz", math.nan), 1)
        yolo_ms = self._format_float(self._latest_scalars.get("yolo_ms", math.nan), 1)
        self._yolo_label.setText(f"YOLO: {yolo_hz} Hz / {yolo_ms} ms")
        self._truth_label.setText(f"Truth F/R/D/R: {self._format_frd(self._latest['truth'])}")
        self._raw_label.setText(f"Raw F/R/D/R: {self._format_frd(self._latest['raw'])}")
        corrected = self._latest["corrected"]
        if self._profile_not_configured and not any(math.isfinite(value) for value in corrected):
            self._corrected_label.setText("Corrected F/R/D/R: NOT CONFIGURED")
        else:
            self._corrected_label.setText(f"Corrected F/R/D/R: {self._format_frd(corrected)}")
        raw_f = self._latest_error("raw_forward")
        corr_f = self._latest_error("corrected_forward")
        raw_f_rmse = self._rmse("raw_forward")
        corr_f_rmse = self._rmse("corrected_forward")
        self._forward_error_label.setText(
            "Forward Δ(raw-truth/corr-truth): "
            f"{self._format_float(raw_f, 2, signed=True)} / "
            f"{self._format_float(corr_f, 2, signed=True)} m | "
            f"RMSE{PLOT_WINDOW_S:.0f}s "
            f"{self._format_float(raw_f_rmse, 2)} / {self._format_float(corr_f_rmse, 2)}"
        )
        raw_d = self._latest_error("raw_down")
        corr_d = self._latest_error("corrected_down")
        raw_d_rmse = self._rmse("raw_down")
        corr_d_rmse = self._rmse("corrected_down")
        self._down_error_label.setText(
            "Down Δ(raw-truth/corr-truth): "
            f"{self._format_float(raw_d, 2, signed=True)} / "
            f"{self._format_float(corr_d, 2, signed=True)} m | "
            f"RMSE{PLOT_WINDOW_S:.0f}s "
            f"{self._format_float(raw_d_rmse, 2)} / {self._format_float(corr_d_rmse, 2)}"
        )
        ratio = self._valid_ratio()
        ratio_text = "--" if not math.isfinite(ratio) else f"{ratio * 100.0:.0f}%"
        disparity = self._format_float(self._latest_detection["disparity_px"], 1)
        confidence = self._format_float(self._latest_detection["confidence_min"], 2)
        validity = "valid" if self._latest_detection["valid"] else "invalid"
        self._detection_label.setText(
            f"Detections: {validity}, valid{PLOT_WINDOW_S:.0f}s={ratio_text}, "
            f"disp={disparity}px, conf={confidence}"
        )


class FrdPlotStack(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(660, 720)
        self.samples = {
            axis_name: {series_name: deque() for series_name, _, _ in SERIES}
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
            history = self.samples[axis_name][series_name]
            history.append((now, value))
            cutoff = now - PLOT_WINDOW_S
            while history and history[0][0] < cutoff:
                history.popleft()
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
        window_s = PLOT_WINDOW_S
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
        model_path = os.environ.get("RQT_YOLO_MODEL_PATH", "").strip()
        model_filename = os.path.basename(model_path) if model_path else "not configured"
        self._hud = ExperimentHud(model_filename)
        root.addWidget(self._hud)
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
        QTimer.singleShot(250, self._maximize_rqt_window)
        QTimer.singleShot(1500, self._maximize_rqt_window)
        self._bridge.image.connect(self._handle_image)
        self._bridge.sample.connect(self._handle_sample)
        self._bridge.scalar.connect(self._hud.update_scalar)
        self._bridge.detections.connect(self._hud.update_detections)
        self._bridge.profile.connect(self._hud.update_profile)
        self._bridge.status.connect(self._status.setText)
        self._init_ros()

    def _handle_image(self, name, payload):
        if name == "left":
            self._left_view.set_image(payload)
        elif name == "right":
            self._right_view.set_image(payload)

    def _handle_sample(self, series_name, values):
        self._plot_stack.append(series_name, values)
        self._hud.update_sample(series_name, values)

    def _maximize_rqt_window(self):
        app = QApplication.instance()
        if app is None:
            return
        for widget in app.topLevelWidgets():
            title = str(widget.windowTitle() or "").lower()
            class_name = str(widget.metaObject().className()).lower()
            if "rqt" in title or "rqt" in class_name:
                widget.showMaximized()
                return

    def _init_ros(self):
        if rclpy is None or Image is None or Float32 is None or Float32MultiArray is None or String is None:
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
            self._ros_node.create_subscription(
                Float32,
                "/stereo/yolo_update_hz",
                lambda msg: self._bridge.scalar.emit("yolo_hz", msg.data),
                10,
            )
            self._ros_node.create_subscription(
                Float32,
                "/stereo/yolo_inference_ms",
                lambda msg: self._bridge.scalar.emit("yolo_ms", msg.data),
                10,
            )
            self._ros_node.create_subscription(
                Float32MultiArray,
                "/stereo/detections",
                lambda msg: self._bridge.detections.emit(list(msg.data)),
                10,
            )
            self._ros_node.create_subscription(
                String,
                "/stereo/profile_status",
                lambda msg: self._bridge.profile.emit(msg.data),
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
