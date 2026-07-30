import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

from python_qt_binding.QtCore import QObject, Qt, QTimer, QUrl, Signal
from python_qt_binding.QtGui import QColor, QDesktopServices, QFont, QImage, QPainter, QPen, QPixmap
from python_qt_binding.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from qt_gui.plugin import Plugin

from .recorder_panel import (
    RecordingControls,
    export_analysis_csv,
    read_recent_csv_lines,
    read_recent_csv_rows,
)

try:
    import rclpy
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PointStamped
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32, Float32MultiArray, String
except Exception:  # pragma: no cover - rqt can still load the widget shell.
    rclpy = None
    ParameterType = None
    GetParameters = None
    ListParameters = None
    qos_profile_sensor_data = None
    PointStamped = None
    Image = None
    Float32 = None
    Float32MultiArray = None
    String = None

try:
    from px4_msgs.msg import SensorGps, VehicleGlobalPosition, VehicleOdometry
except Exception:  # pragma: no cover - PX4 messages are optional for the rqt shell.
    SensorGps = None
    VehicleGlobalPosition = None
    VehicleOdometry = None


AXES = (
    ("Forward", 0),
    ("Right", 1),
    ("Down", 2),
    ("Range", 3),
)
SERIES = (
    ("truth", "/ground_truth_position", QColor(55, 71, 79)),
    ("raw", "/stereo/estimated_position", QColor(220, 32, 32)),
    ("optical", "/stereo/optical_position", QColor(0, 170, 170)),
    ("ideal", "/stereo/ideal_position", QColor(32, 86, 255)),
    ("raw_gps_baseline", "/stereo/raw_gps_baseline_position", QColor(235, 135, 0)),
    ("uam_raw_gps_reference", "/stereo/uam_raw_gps_reference_position", QColor(134, 89, 45)),
)
SERIES_LABELS = {
    "truth": "truth",
    "raw": "Raw",
    "optical": "Truth",
    "ideal": "Ideal",
    "raw_gps_baseline": "GPS",
    "uam_raw_gps_reference": "UAM raw GPS reference",
}
_IMAGE_TOPIC_MODE = os.environ.get("RQT_IMAGE_TOPIC_MODE", "detection").strip().lower()
IMAGE_TOPICS = (
    (
        "Left Camera",
        "/left_camera/image" if _IMAGE_TOPIC_MODE == "raw" else "/left_camera/detection_image",
    ),
    (
        "Right Camera",
        "/right_camera/image" if _IMAGE_TOPIC_MODE == "raw" else "/right_camera/detection_image",
    ),
)
try:
    PLOT_WINDOW_S = max(1.0, float(os.environ.get("RQT_PLOT_WINDOW_S", "30.0")))
except ValueError:
    PLOT_WINDOW_S = 30.0
CSV_PREVIEW_TABLE_ROWS = 40
CSV_PREVIEW_RAW_ROWS = 20


def _env_float(name, default, minimum):
    try:
        return max(float(minimum), float(os.environ.get(name, str(default))))
    except (TypeError, ValueError):
        return float(default)


RQT_IMAGE_RENDER_HZ = _env_float("RQT_IMAGE_RENDER_HZ", 5.0, 0.5)
RQT_FACTS_RENDER_HZ = _env_float("RQT_FACTS_RENDER_HZ", 5.0, 0.5)
RQT_PREVIEW_RENDER_HZ = _env_float("RQT_PREVIEW_RENDER_HZ", 2.0, 0.5)


def _finite_tuple(values, size):
    result = tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f"expected {size} finite values")
    return result


def _rotation_from_wxyz(quaternion_wxyz):
    w, x, y, z = _finite_tuple(quaternion_wxyz, 4)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        raise ValueError("quaternion norm must be positive")
    w, x, y, z = (value / norm for value in (w, x, y, z))
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _ned_to_enu_rotation(rotation):
    return (
        (rotation[1][0], rotation[1][1], rotation[1][2]),
        (rotation[0][0], rotation[0][1], rotation[0][2]),
        (-rotation[2][0], -rotation[2][1], -rotation[2][2]),
    )


def _rpy_deg_from_rotation(rotation):
    pitch = math.asin(max(-1.0, min(1.0, -float(rotation[2][0]))))
    roll = math.atan2(float(rotation[2][1]), float(rotation[2][2]))
    yaw = math.atan2(float(rotation[1][0]), float(rotation[0][0]))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class PanelBridge(QObject):
    image = Signal(str, object)
    sample = Signal(str, list)
    scalar = Signal(str, object)
    detections = Signal(list)
    ray_metrics = Signal(str)
    profile = Signal(str)
    px4_raw_gps = Signal(str, object)
    px4_global_position = Signal(str, object)
    px4_odometry = Signal(str, object)
    status = Signal(str)
    runtime_parameters = Signal(str)


class CameraView(QLabel):
    _ASPECT_HEIGHT_PER_WIDTH = 9.0 / 16.0

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self._title = title
        self._pixmap = None
        self.setMinimumSize(480, 480)
        # The source pixmap must not dictate a column width: both stereo panes
        # receive the same horizontal share, then request their 16:9 height.
        self.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.MinimumExpanding)
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

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return max(480, int(width * self._ASPECT_HEIGHT_PER_WIDTH))

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


class TelemetryFacts(QWidget):
    """Direct telemetry values only; the operator makes any quality judgment."""

    def __init__(self, model_filename, parent=None):
        super().__init__(parent)
        self._latest = {
            "truth": [math.nan] * 4,
            "raw": [math.nan] * 4,
            "optical": [math.nan] * 4,
            "ideal": [math.nan] * 4,
            "raw_gps_baseline": [math.nan] * 4,
        }
        self._latest_scalars = {"yolo_hz": math.nan, "yolo_ms": math.nan}
        self._profile_text = "--"
        self._detection_text = "BBox L/R: --"
        self._refresh_pending = True

        layout = QGridLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setHorizontalSpacing(18)
        layout.setVerticalSpacing(4)
        self._model_label = QLabel(f"Model: {model_filename}")
        self._profile_label = QLabel("Profile: --")
        self._yolo_label = QLabel("YOLO: --")
        self._truth_label = QLabel("Truth F/R/D/R: --")
        self._raw_label = QLabel("Raw F/R/D/R: --")
        self._corrected_label = QLabel("Truth F/R/D/R: --")
        self._ideal_label = QLabel("Ideal F/R/D/R: --")
        self._raw_gps_baseline_label = QLabel("GPS F/R/D/R: --")
        self._detection_label = QLabel("BBox L/R: --")
        labels = (
            self._model_label,
            self._profile_label,
            self._yolo_label,
            self._truth_label,
            self._raw_label,
            self._corrected_label,
            self._ideal_label,
            self._raw_gps_baseline_label,
            self._detection_label,
        )
        for label in labels:
            label.setStyleSheet("font-weight: 600; color: #263238;")
        layout.addWidget(self._model_label, 0, 0)
        layout.addWidget(self._profile_label, 0, 1)
        layout.addWidget(self._yolo_label, 0, 2)
        layout.addWidget(self._truth_label, 1, 0)
        layout.addWidget(self._raw_label, 1, 1)
        layout.addWidget(self._corrected_label, 1, 2)
        layout.addWidget(self._ideal_label, 2, 0)
        layout.addWidget(self._raw_gps_baseline_label, 2, 1)
        layout.addWidget(self._detection_label, 3, 0, 1, 3)
        self.setStyleSheet(
            "TelemetryFacts { background: #eef3f6; border: 1px solid #b0bec5; }"
            "QLabel { padding: 1px 2px; }"
        )
        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(int(1000.0 / RQT_FACTS_RENDER_HZ))
        self._refresh_timer.timeout.connect(self._refresh_if_pending)
        self._refresh_timer.start()

    @staticmethod
    def _format_float(value, precision=1):
        if value is None or not math.isfinite(float(value)):
            return "--"
        return f"{float(value):.{precision}f}"

    def _format_frd(self, values):
        return "/".join(self._format_float(value, 1) for value in values[:4])

    def update_sample(self, series_name, values):
        parsed = []
        for index in range(4):
            try:
                parsed.append(float(values[index]))
            except (IndexError, TypeError, ValueError):
                parsed.append(math.nan)
        self._latest[series_name] = parsed
        self._request_refresh()

    def update_scalar(self, key, value):
        try:
            self._latest_scalars[key] = float(value)
        except (TypeError, ValueError):
            self._latest_scalars[key] = math.nan
        self._request_refresh()

    def update_profile(self, payload):
        try:
            parsed = json.loads(str(payload))
        except json.JSONDecodeError:
            parsed = None
        if isinstance(parsed, dict):
            profile_id = str(parsed.get("profile_id") or "--")
            mode = str(parsed.get("profile_mode") or parsed.get("mode") or "--")
            fallback = str(parsed.get("fallback_reason") or "")
            self._profile_text = f"{profile_id}/{mode}" + (f" {fallback}" if fallback else "")
        else:
            self._profile_text = str(payload)[:120] or "--"
        self._request_refresh()

    def update_detections(self, values):
        try:
            data = [float(value) for value in values]
        except (TypeError, ValueError):
            data = []
        if len(data) < 16:
            self._detection_text = "BBox L/R: --"
            self._request_refresh()
            return
        left = ",".join(self._format_float(value, 0) for value in data[1:5])
        right = ",".join(self._format_float(value, 0) for value in data[9:13])
        disparity = self._format_float(data[6] - data[14], 1)
        confidence = "/".join(self._format_float(value, 2) for value in (data[5], data[13]))
        self._detection_text = (
            f"BBox L[x1,y1,x2,y2]={left}  R[x1,y1,x2,y2]={right}  "
            f"disparity={disparity}px  conf L/R={confidence}"
        )
        self._request_refresh()

    def _request_refresh(self):
        self._refresh_pending = True

    def _refresh_if_pending(self):
        if not self._refresh_pending:
            return
        self._refresh_pending = False
        self._refresh()

    def _refresh(self):
        self._profile_label.setText(f"Profile: {self._profile_text}")
        self._yolo_label.setText(
            "YOLO: "
            f"{self._format_float(self._latest_scalars['yolo_hz'])} Hz / "
            f"{self._format_float(self._latest_scalars['yolo_ms'])} ms"
        )
        self._truth_label.setText(f"Truth F/R/D/R: {self._format_frd(self._latest['truth'])}")
        self._raw_label.setText(f"Raw F/R/D/R: {self._format_frd(self._latest['raw'])}")
        self._corrected_label.setText(
            f"Truth F/R/D/R: {self._format_frd(self._latest['optical'])}"
        )
        self._ideal_label.setText(
            f"Ideal F/R/D/R: {self._format_frd(self._latest['ideal'])}"
        )
        self._raw_gps_baseline_label.setText(
            "GPS F/R/D/R: "
            f"{self._format_frd(self._latest['raw_gps_baseline'])}"
        )
        self._detection_label.setText(self._detection_text)


class ExperimentHud(QWidget):
    def __init__(self, model_filename, parent=None):
        super().__init__(parent)
        self._latest = {
            "truth": [math.nan, math.nan, math.nan, math.nan],
            "raw": [math.nan, math.nan, math.nan, math.nan],
            "corrected": [math.nan, math.nan, math.nan, math.nan],
            "raw_gps_baseline": [math.nan, math.nan, math.nan, math.nan],
            "global_position_baseline": [math.nan, math.nan, math.nan, math.nan],
        }
        self._latest_stamp = {}
        self._error_history = {
            "raw_forward": deque(),
            "corrected_forward": deque(),
            "raw_down": deque(),
            "corrected_down": deque(),
            "raw_gps_baseline_forward": deque(),
            "raw_gps_baseline_down": deque(),
            "global_position_baseline_forward": deque(),
            "global_position_baseline_down": deque(),
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
        self._refresh_pending = True

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
        self._corrected_label = QLabel("Truth F/R/D/R: --")
        self._raw_gps_baseline_label = QLabel("GPS F/R/D/R: --")
        self._global_baseline_label = QLabel("Global F/R/D/R: --")
        self._forward_error_label = QLabel("Forward Δ raw/corr: --")
        self._down_error_label = QLabel("Down Δ raw/corr: --")
        self._gps_baseline_error_label = QLabel("Baseline ΔF GPS/Global: --")
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
            self._raw_gps_baseline_label,
            self._global_baseline_label,
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
        layout.addWidget(self._raw_gps_baseline_label, 3, 0, 1, 2)
        layout.addWidget(self._global_baseline_label, 3, 2, 1, 2)
        layout.addWidget(self._gps_baseline_error_label, 4, 0, 1, 2)
        layout.addWidget(self._contract_label, 5, 0, 1, 4)
        self.setStyleSheet(
            "ExperimentHud { background: #eef3f6; border: 1px solid #b0bec5; }"
            "QLabel { padding: 1px 2px; }"
        )
        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(int(1000.0 / RQT_FACTS_RENDER_HZ))
        self._refresh_timer.timeout.connect(self._refresh_if_pending)
        self._refresh_timer.start()
        self._refresh()


class LiveSavedCsvPreview(QWidget):
    """Read-only preview of live telemetry or complete persisted CSV rows."""

    _TABLE_VIEWS = (
        (
            "Summary",
            (
                ("time_sec", "Time (s)"),
                ("ground_truth_m", "Truth range (m)"),
                ("raw_range_m", "Raw range (m)"),
                ("raw_gps_baseline_range_m", "GPS range (m)"),
            ),
        ),
        (
            "GPS",
            (
                ("time_sec", "Time (s)"),
                ("raw_gps_baseline_forward_m", "GPS F (m)"),
                ("raw_gps_baseline_right_m", "GPS R (m)"),
                ("raw_gps_baseline_down_m", "GPS D (m)"),
                ("raw_gps_baseline_position_error_m", "GPS position error (m)"),
            ),
        ),
        (
            "BBox",
            (
                ("time_sec", "Time (s)"),
                ("left_bbox", "Left bbox (x1,y1,x2,y2)"),
                ("left_confidence", "Left confidence"),
                ("right_bbox", "Right bbox (x1,y1,x2,y2)"),
                ("right_confidence", "Right confidence"),
                ("raw_disparity_px", "Raw disparity (px)"),
                ("yolo_measurement_raw_range_m", "YOLO range (m)"),
            ),
        ),
        (
            "Ray summary",
            (
                ("time_sec", "Time (s)"),
                ("nominal_ray_midpoint_forward_m", "F Raw"),
                ("nominal_ray_midpoint_right_m", "Rt Raw"),
                ("nominal_ray_midpoint_down_m", "D Raw"),
                ("nominal_ray_range_m", "Rg Raw"),
                ("nominal_ray_gap_m", "Gap Raw"),
                ("optical_ray_midpoint_forward_m", "F Truth"),
                ("optical_ray_midpoint_right_m", "Rt Truth"),
                ("optical_ray_midpoint_down_m", "D Truth"),
                ("optical_ray_range_m", "Rg Truth"),
                ("optical_ray_gap_m", "Gap Truth"),
                ("ideal_ray_midpoint_forward_m", "F Ideal"),
                ("ideal_ray_midpoint_right_m", "Rt Ideal"),
                ("ideal_ray_midpoint_down_m", "D Ideal"),
                ("ideal_ray_range_m", "Rg Ideal"),
                ("ideal_ray_gap_m", "Gap Ideal"),
                ("drone_forward_m", "F truth"),
                ("drone_right_m", "Rt truth"),
                ("drone_down_m", "D truth"),
                ("ground_truth_m", "Rg truth"),
            ),
        ),
        (
            "Left PX4",
            (
                ("time_sec", "Time (s)"),
                ("left_raw_gps_lat_deg", "Raw GPS lat"),
                ("left_raw_gps_lon_deg", "Raw GPS lon"),
                ("left_raw_gps_alt_msl_m", "Raw GPS alt"),
                ("left_px4_position_n_m", "PX4 N"),
                ("left_px4_position_e_m", "PX4 E"),
                ("left_px4_position_d_m", "PX4 D"),
                ("left_px4_attitude_world_enu_roll_deg", "R ENU"),
                ("left_px4_attitude_world_enu_pitch_deg", "P ENU"),
                ("left_px4_attitude_world_enu_yaw_deg", "Y ENU"),
                ("left_px4_velocity_n_m_s", "Vn"),
                ("left_px4_velocity_e_m_s", "Ve"),
                ("left_px4_velocity_d_m_s", "Vd"),
                ("left_px4_estimated_optical_world_enu_x_m", "Cam X"),
                ("left_px4_estimated_optical_world_enu_y_m", "Cam Y"),
                ("left_px4_estimated_optical_world_enu_z_m", "Cam Z"),
                ("left_px4_estimated_optical_world_enu_roll_deg", "Cam R"),
                ("left_px4_estimated_optical_world_enu_pitch_deg", "Cam P"),
                ("left_px4_estimated_optical_world_enu_yaw_deg", "Cam Yaw"),
            ),
        ),
        (
            "Right PX4",
            (
                ("time_sec", "Time (s)"),
                ("right_raw_gps_lat_deg", "Raw GPS lat"),
                ("right_raw_gps_lon_deg", "Raw GPS lon"),
                ("right_raw_gps_alt_msl_m", "Raw GPS alt"),
                ("right_px4_position_n_m", "PX4 N"),
                ("right_px4_position_e_m", "PX4 E"),
                ("right_px4_position_d_m", "PX4 D"),
                ("right_px4_attitude_world_enu_roll_deg", "R ENU"),
                ("right_px4_attitude_world_enu_pitch_deg", "P ENU"),
                ("right_px4_attitude_world_enu_yaw_deg", "Y ENU"),
                ("right_px4_velocity_n_m_s", "Vn"),
                ("right_px4_velocity_e_m_s", "Ve"),
                ("right_px4_velocity_d_m_s", "Vd"),
                ("right_px4_estimated_optical_world_enu_x_m", "Cam X"),
                ("right_px4_estimated_optical_world_enu_y_m", "Cam Y"),
                ("right_px4_estimated_optical_world_enu_z_m", "Cam Z"),
                ("right_px4_estimated_optical_world_enu_roll_deg", "Cam R"),
                ("right_px4_estimated_optical_world_enu_pitch_deg", "Cam P"),
                ("right_px4_estimated_optical_world_enu_yaw_deg", "Cam Yaw"),
            ),
        ),
        (
            "UAM PX4",
            (
                ("time_sec", "Time (s)"),
                ("uam_raw_gps_lat_deg", "Raw GPS lat"),
                ("uam_raw_gps_lon_deg", "Raw GPS lon"),
                ("uam_raw_gps_alt_msl_m", "Raw GPS alt"),
                ("uam_px4_position_n_m", "PX4 N"),
                ("uam_px4_position_e_m", "PX4 E"),
                ("uam_px4_position_d_m", "PX4 D"),
                ("uam_px4_attitude_world_enu_roll_deg", "R ENU"),
                ("uam_px4_attitude_world_enu_pitch_deg", "P ENU"),
                ("uam_px4_attitude_world_enu_yaw_deg", "Y ENU"),
                ("uam_px4_velocity_n_m_s", "Vn"),
                ("uam_px4_velocity_e_m_s", "Ve"),
                ("uam_px4_velocity_d_m_s", "Vd"),
            ),
        ),
    )

    def __init__(self, recording_controls, parent=None):
        super().__init__(parent)
        self._recording_controls = recording_controls
        self._stopped = False
        self._live_started_monotonic = time.monotonic()
        self._live_latest = {}
        self._live_preview_dirty = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        layout.addWidget(QLabel("CSV preview"))
        self._state_label = QLabel("Waiting for telemetry.")
        self._state_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self._state_label)
        self._coverage_label = QLabel("Coverage: --")
        self._coverage_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self._coverage_label)
        self._views = QTabWidget()
        self._tables = {}
        for view_name, columns in self._TABLE_VIEWS:
            table = self._create_table(columns)
            self._tables[view_name] = (table, columns)
            self._views.addTab(table, view_name)
        raw_tab = QWidget()
        raw_layout = QVBoxLayout(raw_tab)
        raw_layout.setContentsMargins(8, 8, 8, 8)
        raw_layout.addWidget(QLabel("Raw CSV preview"))
        self._raw_preview = QPlainTextEdit()
        self._raw_preview.setReadOnly(True)
        self._raw_preview.setLineWrapMode(QPlainTextEdit.NoWrap)
        self._raw_preview.setStyleSheet("font-family: monospace;")
        self._raw_preview.setPlainText("Waiting for telemetry.")
        raw_layout.addWidget(self._raw_preview, 1)
        self._views.addTab(raw_tab, "Raw CSV")
        layout.addWidget(self._views, 1)
        actions = QHBoxLayout()
        self._open_session_button = QPushButton("Open session")
        self._open_csv_button = QPushButton("Open CSV")
        self._open_xlsx_button = QPushButton("Open XLSX")
        self._export_button = QPushButton("Export analysis CSV")
        actions.addWidget(self._open_session_button)
        actions.addWidget(self._open_csv_button)
        actions.addWidget(self._open_xlsx_button)
        actions.addWidget(self._export_button)
        actions.addStretch(1)
        layout.addLayout(actions)
        self._refresh_button = QPushButton("Refresh CSV preview")
        self._refresh_button.clicked.connect(self.refresh)
        layout.addWidget(self._refresh_button)
        self._open_session_button.clicked.connect(self._open_session)
        self._open_csv_button.clicked.connect(self._open_csv)
        self._open_xlsx_button.clicked.connect(self._open_xlsx)
        self._export_button.clicked.connect(self._export_analysis_csv)
        self._last_signature = None
        self._live_preview_timer = QTimer(self)
        self._live_preview_timer.setInterval(int(1000.0 / RQT_PREVIEW_RENDER_HZ))
        self._live_preview_timer.timeout.connect(self._flush_live_preview)
        self._live_preview_timer.start()
        QTimer.singleShot(500, self._refresh_periodically)

    @staticmethod
    def _file_signature(path):
        try:
            stat = Path(path).stat()
        except OSError:
            return None
        return stat.st_mtime_ns, stat.st_size

    @staticmethod
    def _format_bbox(row, side):
        values = [row.get(f"{side}_{axis}", "") for axis in ("x1", "y1", "x2", "y2")]
        return ", ".join(values) if any(values) else ""

    @staticmethod
    def _format_csv_value(value):
        if value is None:
            return ""
        if isinstance(value, bool):
            return "1" if value else "0"
        try:
            number = float(value)
        except (TypeError, ValueError):
            return str(value)
        if not math.isfinite(number):
            return ""
        return f"{number:.9g}"

    @staticmethod
    def _create_table(columns):
        table = QTableWidget(0, len(columns))
        table.setHorizontalHeaderLabels([label for _field, label in columns])
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setAlternatingRowColors(True)
        table.horizontalHeader().setStretchLastSection(True)
        return table

    def _set_action_state(self, csv_path):
        session_path = self._recording_controls.recording_session_path()
        ready = (
            csv_path is not None
            and Path(csv_path).is_file()
            and session_path is not None
            and not self._recording_controls.is_recording()
        )
        self._open_session_button.setEnabled(ready)
        self._open_csv_button.setEnabled(ready)
        xlsx_path = self._recording_controls.recording_xlsx_path()
        self._open_xlsx_button.setEnabled(
            ready and xlsx_path is not None and xlsx_path.is_file()
        )
        self._export_button.setEnabled(ready)

    def _update_tables(self, rows):
        for table, columns in self._tables.values():
            table.setRowCount(len(rows))
            for row_index, row in enumerate(rows):
                values = {
                    "left_bbox": self._format_bbox(row, "left"),
                    "right_bbox": self._format_bbox(row, "right"),
                    **row,
                }
                for column_index, (field, _label) in enumerate(columns):
                    table.setItem(row_index, column_index, QTableWidgetItem(values.get(field, "")))
            table.resizeColumnsToContents()

    def _live_rows(self):
        if not self._live_latest:
            return []
        row = {
            "time_sec": self._format_csv_value(time.monotonic() - self._live_started_monotonic),
            **self._live_latest,
        }
        return [row]

    def _format_live_raw_preview(self, rows):
        if not rows:
            return "Waiting for telemetry."
        row = rows[-1]
        fields = ["time_sec"] + sorted(field for field in row if field != "time_sec")
        return ",".join(fields) + "\n" + ",".join(row.get(field, "") for field in fields)

    def _show_live_preview(self):
        rows = self._live_rows()
        self._state_label.setText("Latest telemetry")
        self._coverage_label.setText("Coverage: live topics")
        self._update_tables(rows)
        self._raw_preview.setPlainText(self._format_live_raw_preview(rows))
        self._set_action_state(None)

    def _refresh_live_preview(self):
        if self._recording_controls.recording_csv_path() is None:
            self._live_preview_dirty = True

    def _flush_live_preview(self):
        if not self._live_preview_dirty:
            return
        self._live_preview_dirty = False
        if self._recording_controls.recording_csv_path() is None:
            self._show_live_preview()

    def update_live_sample(self, series_name, values):
        try:
            parsed = [float(value) for value in list(values)[:4]]
        except (TypeError, ValueError):
            parsed = []
        while len(parsed) < 4:
            parsed.append(math.nan)
        if not math.isfinite(parsed[3]) and all(math.isfinite(value) for value in parsed[:3]):
            parsed[3] = math.sqrt(sum(value * value for value in parsed[:3]))
        field_prefixes = {
            "truth": ("drone_forward_m", "drone_right_m", "drone_down_m", "ground_truth_m"),
            "raw": ("raw_forward_m", "raw_right_m", "raw_down_m", "raw_range_m"),
            "optical": (
                "optical_forward_m",
                "optical_right_m",
                "optical_down_m",
                "optical_range_m",
            ),
            "ideal": ("ideal_forward_m", "ideal_right_m", "ideal_down_m", "ideal_range_m"),
            "raw_gps_baseline": (
                "raw_gps_baseline_forward_m",
                "raw_gps_baseline_right_m",
                "raw_gps_baseline_down_m",
                "raw_gps_baseline_range_m",
            ),
            "uam_raw_gps_reference": (
                "uam_raw_gps_reference_forward_m",
                "uam_raw_gps_reference_right_m",
                "uam_raw_gps_reference_down_m",
                "uam_raw_gps_reference_range_m",
            ),
        }.get(series_name)
        if field_prefixes is None:
            return
        for field, value in zip(field_prefixes, parsed):
            self._live_latest[field] = self._format_csv_value(value)
        truth = [
            self._live_latest.get("drone_forward_m"),
            self._live_latest.get("drone_right_m"),
            self._live_latest.get("drone_down_m"),
        ]
        baseline = [
            self._live_latest.get("raw_gps_baseline_forward_m"),
            self._live_latest.get("raw_gps_baseline_right_m"),
            self._live_latest.get("raw_gps_baseline_down_m"),
        ]
        try:
            truth_values = [float(value) for value in truth]
            baseline_values = [float(value) for value in baseline]
        except (TypeError, ValueError):
            truth_values = []
            baseline_values = []
        if len(truth_values) == 3 and len(baseline_values) == 3:
            error = math.sqrt(
                sum((estimate - actual) ** 2 for estimate, actual in zip(baseline_values, truth_values))
            )
            self._live_latest["raw_gps_baseline_position_error_m"] = self._format_csv_value(error)
        self._refresh_live_preview()

    def update_live_scalar(self, key, value):
        field = {
            "yolo_hz": "yolo_update_hz",
            "yolo_ms": "yolo_inference_ms",
        }.get(key)
        if field is not None:
            self._live_latest[field] = self._format_csv_value(value)
            self._refresh_live_preview()

    def update_live_detections(self, values):
        try:
            data = [float(value) for value in values]
        except (TypeError, ValueError):
            return
        if len(data) < 16:
            return
        fields = (
            "left_valid",
            "left_x1",
            "left_y1",
            "left_x2",
            "left_y2",
            "left_confidence",
            "left_cx",
            "left_cy",
            "right_valid",
            "right_x1",
            "right_y1",
            "right_x2",
            "right_y2",
            "right_confidence",
            "right_cx",
            "right_cy",
        )
        for field, value in zip(fields, data):
            self._live_latest[field] = self._format_csv_value(value)
        self._live_latest["left_width_px"] = self._format_csv_value(data[3] - data[1])
        self._live_latest["left_height_px"] = self._format_csv_value(data[4] - data[2])
        self._live_latest["right_width_px"] = self._format_csv_value(data[11] - data[9])
        self._live_latest["right_height_px"] = self._format_csv_value(data[12] - data[10])
        self._live_latest["raw_disparity_px"] = self._format_csv_value(data[6] - data[14])
        self._refresh_live_preview()

    def update_live_profile(self, payload):
        try:
            parsed = json.loads(str(payload))
        except json.JSONDecodeError:
            return
        if not isinstance(parsed, dict):
            return
        field_map = {
            "profile_id": "profile_id",
            "profile_mode": "profile_mode",
            "mode": "profile_mode",
            "fallback_level": "profile_fallback_level",
            "fallback_reason": "profile_fallback_reason",
        }
        for source, field in field_map.items():
            if source in parsed:
                self._live_latest[field] = self._format_csv_value(parsed.get(source))
        self._refresh_live_preview()

    def update_live_ray_metrics(self, payload):
        try:
            parsed = json.loads(str(payload))
        except json.JSONDecodeError:
            return
        if not isinstance(parsed, dict):
            return
        for field in (
            "ideal_left_u_px",
            "ideal_left_v_px",
            "ideal_right_u_px",
            "ideal_right_v_px",
            "ideal_disparity_px",
            "ideal_pixel_source",
        ):
            if field in parsed:
                self._live_latest[field] = self._format_csv_value(parsed.get(field))
        for ray_type in ("nominal", "optical", "ideal"):
            ray = parsed.get(ray_type)
            if not isinstance(ray, dict):
                continue
            prefix = f"{ray_type}_ray"
            self._live_latest[f"{prefix}_computation_status"] = self._format_csv_value(
                ray.get("computation_status", "")
            )
            for key in (
                "forward_m",
                "right_m",
                "down_m",
                "range_m",
                "gap_m",
                "intersection_angle_deg",
                "left_depth_m",
                "right_depth_m",
            ):
                if key in ray:
                    self._live_latest[f"{prefix}_{key}"] = self._format_csv_value(ray.get(key))
            for source_key, field_middle in (
                ("left_point_frd_m", "left_point"),
                ("right_point_frd_m", "right_point"),
                ("midpoint_frd_m", "midpoint"),
            ):
                values = ray.get(source_key)
                if not isinstance(values, (list, tuple)) or len(values) < 3:
                    continue
                for axis, value in zip(("forward", "right", "down"), values[:3]):
                    self._live_latest[f"{prefix}_{field_middle}_{axis}_m"] = (
                        self._format_csv_value(value)
                    )
        self._refresh_live_preview()

    def update_live_px4_raw_gps(self, label, msg):
        prefix = f"{label}_raw_gps"
        timestamp_us = int(getattr(msg, "timestamp_sample", 0) or getattr(msg, "timestamp", 0) or 0)
        self._live_latest[f"{prefix}_timestamp_us"] = self._format_csv_value(timestamp_us)
        for attr, suffix in (
            ("latitude_deg", "lat_deg"),
            ("longitude_deg", "lon_deg"),
            ("altitude_msl_m", "alt_msl_m"),
            ("eph", "eph_m"),
            ("epv", "epv_m"),
        ):
            self._live_latest[f"{prefix}_{suffix}"] = self._format_csv_value(getattr(msg, attr, None))
        if bool(getattr(msg, "vel_ned_valid", False)):
            for attr, suffix in (
                ("vel_n_m_s", "vel_n_m_s"),
                ("vel_e_m_s", "vel_e_m_s"),
                ("vel_d_m_s", "vel_d_m_s"),
            ):
                self._live_latest[f"{prefix}_{suffix}"] = self._format_csv_value(getattr(msg, attr, None))
        else:
            for axis in ("n", "e", "d"):
                self._live_latest[f"{prefix}_vel_{axis}_m_s"] = ""
        self._refresh_live_preview()

    def update_live_px4_global_position(self, label, msg):
        prefix = f"{label}_global_position"
        timestamp_us = int(getattr(msg, "timestamp_sample", 0) or getattr(msg, "timestamp", 0) or 0)
        self._live_latest[f"{prefix}_timestamp_us"] = self._format_csv_value(timestamp_us)
        for attr, suffix in (
            ("lat", "lat_deg"),
            ("lon", "lon_deg"),
            ("alt", "alt_msl_m"),
            ("eph", "eph_m"),
            ("epv", "epv_m"),
        ):
            self._live_latest[f"{prefix}_{suffix}"] = self._format_csv_value(getattr(msg, attr, None))
        self._refresh_live_preview()

    def update_live_px4_odometry(self, label, msg):
        if int(getattr(msg, "pose_frame", -1)) != int(getattr(msg, "POSE_FRAME_NED", -2)):
            return
        prefix = f"{label}_px4"
        timestamp_us = int(getattr(msg, "timestamp_sample", 0) or getattr(msg, "timestamp", 0) or 0)
        self._live_latest[f"{prefix}_odometry_timestamp_us"] = self._format_csv_value(timestamp_us)
        for axis, value in zip(("n", "e", "d"), list(getattr(msg, "position", []))[:3]):
            self._live_latest[f"{prefix}_position_{axis}_m"] = self._format_csv_value(value)
        try:
            rotation_world_enu_from_body = _ned_to_enu_rotation(
                _rotation_from_wxyz(getattr(msg, "q", ()))
            )
            attitude = _rpy_deg_from_rotation(rotation_world_enu_from_body)
        except (TypeError, ValueError):
            attitude = ()
        for axis, value in zip(("roll", "pitch", "yaw"), attitude):
            self._live_latest[f"{prefix}_attitude_world_enu_{axis}_deg"] = self._format_csv_value(
                value
            )
        if int(getattr(msg, "velocity_frame", -1)) == int(getattr(msg, "VELOCITY_FRAME_NED", -2)):
            for axis, value in zip(("n", "e", "d"), list(getattr(msg, "velocity", []))[:3]):
                self._live_latest[f"{prefix}_velocity_{axis}_m_s"] = self._format_csv_value(value)
        self._live_latest[f"{prefix}_odometry_reset_counter"] = self._format_csv_value(
            getattr(msg, "reset_counter", None)
        )
        self._refresh_live_preview()

    def _set_progress_text(self, csv_path, progress):
        recording = self._recording_controls.is_recording()
        state = "Writing" if recording else "Last saved"
        if progress:
            state = "Writing" if progress.get("state") == "writing" else "Finalized"
            mode = progress.get("mode", "--")
            rows = progress.get("row_count", "--")
            elapsed = progress.get("elapsed_sec", "--")
            xlsx_status = progress.get("xlsx_status", "unavailable")
            self._state_label.setText(
                f"{state} · {mode} · {rows} rows · {elapsed} s · XLSX {xlsx_status} · {csv_path}"
            )
            valid = progress.get("valid_row_counts", {})
            pair_count = min(valid.get("left_x1", 0), valid.get("right_x1", 0))
            self._coverage_label.setText(
                "Coverage (facts only): "
                f"Ground truth {valid.get('ground_truth_m', 0)}/{rows} · "
                f"Stereo {valid.get('raw_range_m', 0)}/{rows} · "
                f"GPS {valid.get('raw_gps_baseline_forward_m', 0)}/{rows} · "
                f"bbox pair {pair_count}/{rows}"
            )
            return
        self._state_label.setText(f"{state}: {csv_path}")
        self._coverage_label.setText("Coverage: progress.json is unavailable for this older session.")

    def refresh(self, force=False):
        csv_path = self._recording_controls.recording_csv_path()
        if csv_path is None:
            self._show_live_preview()
            return
        progress_path = Path(csv_path).with_name("progress.json")
        signature = (self._file_signature(csv_path), self._file_signature(progress_path))
        if not force and signature == self._last_signature:
            self._set_action_state(csv_path)
            return
        self._last_signature = signature
        try:
            preview = read_recent_csv_lines(csv_path, data_row_count=CSV_PREVIEW_RAW_ROWS)
            rows = read_recent_csv_rows(csv_path, data_row_count=CSV_PREVIEW_TABLE_ROWS)
        except FileNotFoundError:
            preview = "Waiting for recorder to create telemetry CSV…"
            rows = []
        except OSError as exc:
            preview = f"CSV preview unavailable: {exc}"
            rows = []
        try:
            progress = json.loads(progress_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            progress = None
        self._set_progress_text(csv_path, progress)
        self._update_tables(rows)
        self._raw_preview.setPlainText(preview)
        self._set_action_state(csv_path)

    def _refresh_periodically(self):
        if self._stopped:
            return
        self.refresh()
        QTimer.singleShot(
            500 if self._recording_controls.is_recording() else 2000,
            self._refresh_periodically,
        )

    def _open_path(self, path):
        if not QDesktopServices.openUrl(QUrl.fromLocalFile(str(Path(path).resolve()))):
            self._state_label.setText(f"Could not open: {path}")

    def _open_session(self):
        session_path = self._recording_controls.recording_session_path()
        if session_path is not None:
            self._open_path(session_path)

    def _open_csv(self):
        csv_path = self._recording_controls.recording_csv_path()
        if csv_path is not None and Path(csv_path).is_file():
            self._open_path(csv_path)

    def _open_xlsx(self):
        session_path = self._recording_controls.recording_session_path()
        if session_path is None:
            return
        xlsx_path = self._recording_controls.recording_xlsx_path()
        if xlsx_path is not None and xlsx_path.is_file():
            self._open_path(xlsx_path)

    def _export_analysis_csv(self):
        csv_path = self._recording_controls.recording_csv_path()
        if csv_path is None or self._recording_controls.is_recording():
            return
        timestamp = Path(csv_path).stem.removeprefix("telemetry_")
        output_path = Path(csv_path).with_name(f"analysis_{timestamp}.csv")
        try:
            export_analysis_csv(csv_path, output_path)
        except OSError as exc:
            self._state_label.setText(f"Analysis export failed: {exc}")
            return
        self._state_label.setText(f"Analysis export saved: {output_path}")

    def shutdown(self):
        self._stopped = True

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
        if series_name in (
            "raw", "corrected", "raw_gps_baseline", "global_position_baseline"
        ):
            self._append_error_sample(series_name, axis_index=0, axis_name="forward", now=now)
            self._append_error_sample(series_name, axis_index=2, axis_name="down", now=now)
        self._prune(now)
        self._request_refresh()

    def update_scalar(self, key, value):
        try:
            self._latest_scalars[key] = float(value)
            self._latest_stamp[key] = time.monotonic()
        except (TypeError, ValueError):
            self._latest_scalars[key] = math.nan
        self._request_refresh()

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
        self._request_refresh()

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
        self._request_refresh()

    def _request_refresh(self):
        self._refresh_pending = True

    def _refresh_if_pending(self):
        if not self._refresh_pending:
            return
        self._refresh_pending = False
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
            self._corrected_label.setText("Truth F/R/D/R: NOT CONFIGURED")
        else:
            self._corrected_label.setText(f"Truth F/R/D/R: {self._format_frd(corrected)}")
        self._raw_gps_baseline_label.setText(
            "GPS F/R/D/R: "
            f"{self._format_frd(self._latest['raw_gps_baseline'])}"
        )
        self._global_baseline_label.setText(
            "Global F/R/D/R: "
            f"{self._format_frd(self._latest['global_position_baseline'])}"
        )
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
        raw_gps_f = self._latest_error("raw_gps_baseline_forward")
        global_f = self._latest_error("global_position_baseline_forward")
        self._gps_baseline_error_label.setText(
            "PX4 baseline ΔF raw/global: "
            f"{self._format_float(raw_gps_f, 2, signed=True)} / "
            f"{self._format_float(global_f, 2, signed=True)} m"
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
        self.setMinimumSize(660, 880)
        self._repaint_pending = False
        self.samples = {
            axis_name: {series_name: deque() for series_name, _, _ in SERIES}
            for axis_name, _ in AXES
        }
        self._repaint_timer = QTimer(self)
        self._repaint_timer.setInterval(int(1000.0 / RQT_FACTS_RENDER_HZ))
        self._repaint_timer.timeout.connect(self._flush_repaint)
        self._repaint_timer.start()

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
        self._repaint_pending = True

    def _flush_repaint(self):
        if not self._repaint_pending:
            return
        self._repaint_pending = False
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(250, 250, 250))
        painter.setRenderHint(QPainter.Antialiasing, True)
        margin_left = 68
        margin_right = 190
        margin_top = 34
        margin_bottom = 28
        gap = 22
        width = max(1, self.width() - margin_left - margin_right)
        pane_height = max(
            1,
            (self.height() - margin_top - margin_bottom - gap * (len(AXES) - 1))
            / len(AXES),
        )

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
            painter.drawText(left, top - 10, f"{axis_name} (FRD m)")
            painter.setFont(QFont("Sans", 9))
            painter.drawText(8, top + 14, f"{ymax:.1f}")
            painter.drawText(8, bottom, f"{ymin:.1f}")

            legend_x = right + 12
            legend_y = top + 16
            painter.setFont(QFont("Sans", 10))
            for series_name, _topic, color in SERIES:
                painter.setPen(QPen(color, 3))
                painter.drawLine(legend_x, legend_y, legend_x + 20, legend_y)
                painter.setPen(QPen(QColor(32, 32, 32), 1))
                painter.drawText(legend_x + 35, legend_y + 5, SERIES_LABELS[series_name])
                legend_y += 24

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
        self._parameter_get_client = None
        self._parameter_list_client = None
        self._ros_graph_process = None
        self._spin_thread = None
        self._stop_event = threading.Event()
        self._image_emit_lock = threading.Lock()
        self._last_image_emit_monotonic = {}
        self._bridge = PanelBridge()

        self._widget = QWidget()
        self._widget.setWindowTitle("StereoCAM Operator Panel")
        root = QVBoxLayout(self._widget)
        root.setContentsMargins(6, 6, 6, 6)
        model_path = os.environ.get("RQT_YOLO_MODEL_PATH", "").strip()
        model_filename = os.path.basename(model_path) if model_path else "not configured"
        self._status = QLabel(
            "FRD: Forward, Right, Down. Select a tab for setup, live facts, analysis, or raw diagnostics."
        )
        self._status.setStyleSheet("font-weight: 600;")
        self._debug_label = QLabel()
        self._debug_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._debug_label.setStyleSheet("font-family: monospace;")
        self._facts = TelemetryFacts(model_filename)
        self._left_view = CameraView("Left Camera")
        self._right_view = CameraView("Right Camera")
        self._plot_stack = FrdPlotStack()
        self._recording_controls = RecordingControls()

        tabs = QTabWidget()
        tabs.addTab(self._build_setup_tab(model_filename), "Setup")
        tabs.addTab(self._build_flight_tab(), "Flight")
        tabs.addTab(self._build_analysis_tab(), "Analysis")
        tabs.addTab(self._build_record_tab(), "Record")
        tabs.addTab(self._build_debug_tab(), "Debug")
        tabs.setCurrentIndex(0)
        self._tabs = tabs
        root.addWidget(tabs, 1)

        context.add_widget(self._widget)
        self._bridge.image.connect(self._handle_image)
        self._bridge.sample.connect(self._handle_sample)
        self._bridge.scalar.connect(self._handle_scalar)
        self._bridge.detections.connect(self._handle_detections)
        self._bridge.ray_metrics.connect(self._handle_ray_metrics)
        self._bridge.profile.connect(self._handle_profile)
        self._bridge.px4_raw_gps.connect(self._handle_px4_raw_gps)
        self._bridge.px4_global_position.connect(self._handle_px4_global_position)
        self._bridge.px4_odometry.connect(self._handle_px4_odometry)
        self._bridge.status.connect(self._set_status)
        self._bridge.runtime_parameters.connect(self._set_runtime_parameters)
        self._init_ros()
        # A perspective can restore its previously active tab after the plugin
        # constructor returns. Defer the pre-flight default until that restore
        # has completed so a new operator window always opens on Setup.
        QTimer.singleShot(250, lambda: self._tabs.setCurrentIndex(0))

    def restore_settings(self, plugin_settings, instance_settings):
        super().restore_settings(plugin_settings, instance_settings)
        self._tabs.setCurrentIndex(0)

    def _build_setup_tab(self, model_filename):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)
        layout.addWidget(QLabel("Pre-flight setup and recording"))
        summary = QLabel(
            f"Model: {model_filename}\n"
            f"{os.environ.get('RQT_MISSION_LABEL', 'Mission: --').strip() or 'Mission: --'}\n"
            "Coordinate contract: public FRD [Forward, Right, Down, Range]."
        )
        summary.setStyleSheet("font-weight: 600; color: #263238;")
        layout.addWidget(summary)
        layout.addWidget(
            QLabel("Raw GPS and GlobalPosition are recorded together for offline comparison."),
        )
        layout.addWidget(QLabel("StereoCAM run parameters (/stereo_distance_node, read-only)"))
        self._runtime_parameters = QPlainTextEdit()
        self._runtime_parameters.setReadOnly(True)
        self._runtime_parameters.setMinimumHeight(260)
        self._runtime_parameters.setPlainText("Waiting for /stereo_distance_node parameters…")
        self._runtime_parameters.setStyleSheet("font-family: monospace;")
        layout.addWidget(self._runtime_parameters, 1)
        self._refresh_parameters_button = QPushButton("Refresh run parameters")
        self._refresh_parameters_button.clicked.connect(self._request_runtime_parameters)
        layout.addWidget(self._refresh_parameters_button)
        layout.addWidget(self._recording_controls)
        return tab

    def _build_flight_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)
        cameras = QGridLayout()
        cameras.setHorizontalSpacing(8)
        cameras.setColumnStretch(0, 1)
        cameras.setColumnStretch(1, 1)
        cameras.addWidget(self._left_view, 0, 0)
        cameras.addWidget(self._right_view, 0, 1)
        layout.addLayout(cameras)
        layout.addWidget(self._facts)
        layout.addWidget(self._status)
        layout.addStretch(1)
        return tab

    def _build_analysis_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(
            QLabel(
                "FRDR comparison: truth, Raw, Truth, Ideal (simulation-only), GPS, "
                "and Global. "
                "Range is the 3D vector magnitude when a source "
                "publishes F/R/D only."
            )
        )
        layout.addWidget(self._plot_stack, 1)
        return tab

    def _build_record_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        self._record_preview = LiveSavedCsvPreview(self._recording_controls)
        layout.addWidget(self._record_preview)
        return tab

    def _build_debug_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addWidget(QLabel("Raw runtime status (informational; no automated judgment)"))
        self._open_ros_graph_button = QPushButton("Open ROS graph")
        self._open_ros_graph_button.clicked.connect(self._open_ros_graph)
        layout.addWidget(self._open_ros_graph_button)
        layout.addWidget(
            QLabel(
                "Opens rqt_graph in a separate diagnostic window so Flight and "
                "Analysis keep the full operator workspace."
            )
        )
        layout.addWidget(self._debug_label)
        layout.addStretch(1)
        self._set_status("Waiting for ROS topics.")
        return tab

    def _open_ros_graph(self):
        if self._ros_graph_process is not None and self._ros_graph_process.poll() is None:
            self._bridge.status.emit("ROS graph is already open in a separate window.")
            return
        try:
            self._ros_graph_process = subprocess.Popen(
                ["ros2", "run", "rqt_graph", "rqt_graph"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except OSError as exc:
            self._ros_graph_process = None
            self._bridge.status.emit(f"ROS graph could not start: {exc}")
            return
        self._bridge.status.emit("ROS graph opened in a separate diagnostic window.")

    def _set_status(self, text):
        self._status.setText(str(text))
        self._debug_label.setText(
            f"Status: {text}\n"
            f"Images: {IMAGE_TOPICS[0][1]}, {IMAGE_TOPICS[1][1]}\n"
            "Series: /ground_truth_position, /stereo/estimated_position (Raw), "
            "/stereo/optical_position (Truth), /stereo/ideal_position (Ideal), "
            "/stereo/raw_gps_baseline_position (GPS)\n"
            "Detection format: left/right bbox x1,y1,x2,y2, confidence, center."
        )

    def _set_runtime_parameters(self, text):
        self._runtime_parameters.setPlainText(str(text))

    def _handle_image(self, name, payload):
        if name == "left":
            self._left_view.set_image(payload)
        elif name == "right":
            self._right_view.set_image(payload)

    def _handle_sample(self, series_name, values):
        frdr = list(values[:4])
        while len(frdr) < 4:
            frdr.append(math.nan)
        if not math.isfinite(frdr[3]) and all(math.isfinite(value) for value in frdr[:3]):
            frdr[3] = math.sqrt(sum(value * value for value in frdr[:3]))
        self._plot_stack.append(series_name, frdr)
        self._facts.update_sample(series_name, frdr)
        self._record_preview.update_live_sample(series_name, frdr)

    def _handle_scalar(self, key, value):
        self._facts.update_scalar(key, value)
        self._record_preview.update_live_scalar(key, value)

    def _handle_detections(self, values):
        self._facts.update_detections(values)
        self._record_preview.update_live_detections(values)

    def _handle_profile(self, payload):
        self._facts.update_profile(payload)
        self._record_preview.update_live_profile(payload)

    def _handle_ray_metrics(self, payload):
        self._record_preview.update_live_ray_metrics(payload)

    def _handle_px4_raw_gps(self, label, msg):
        self._record_preview.update_live_px4_raw_gps(label, msg)

    def _handle_px4_global_position(self, label, msg):
        self._record_preview.update_live_px4_global_position(label, msg)

    def _handle_px4_odometry(self, label, msg):
        self._record_preview.update_live_px4_odometry(label, msg)

    def _init_ros(self):
        if (
            rclpy is None or ParameterType is None or GetParameters is None
            or ListParameters is None
            or Image is None or PointStamped is None or Float32 is None
            or Float32MultiArray is None or String is None
        ):
            self._status.setText("ROS imports unavailable; operator panel cannot subscribe.")
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._ros_node = rclpy.create_node("operator_panel")
            self._parameter_get_client = self._ros_node.create_client(
                GetParameters, "/stereo_distance_node/get_parameters"
            )
            self._parameter_list_client = self._ros_node.create_client(
                ListParameters, "/stereo_distance_node/list_parameters"
            )
            self._ros_node.create_subscription(
                Image,
                IMAGE_TOPICS[0][1],
                lambda msg: self._emit_image_sample("left", msg),
                qos_profile_sensor_data,
            )
            self._ros_node.create_subscription(
                Image,
                IMAGE_TOPICS[1][1],
                lambda msg: self._emit_image_sample("right", msg),
                qos_profile_sensor_data,
            )
            for series_name, topic, _color in SERIES:
                if series_name in ("raw_gps_baseline", "uam_raw_gps_reference"):
                    continue
                self._ros_node.create_subscription(
                    Float32MultiArray,
                    topic,
                    lambda msg, name=series_name: self._bridge.sample.emit(name, list(msg.data)),
                    10,
                )
            self._ros_node.create_subscription(
                PointStamped,
                "/stereo/raw_gps_baseline_position",
                lambda msg: self._bridge.sample.emit(
                    "raw_gps_baseline", [msg.point.x, msg.point.y, msg.point.z, math.nan]
                ),
                10,
            )
            self._ros_node.create_subscription(
                PointStamped,
                "/stereo/uam_raw_gps_reference_position",
                lambda msg: self._bridge.sample.emit(
                    "uam_raw_gps_reference", [msg.point.x, msg.point.y, msg.point.z, math.nan]
                ),
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
            self._ros_node.create_subscription(
                String,
                "/stereo/ray_metrics",
                lambda msg: self._bridge.ray_metrics.emit(msg.data),
                10,
            )
            if SensorGps is not None and VehicleGlobalPosition is not None and VehicleOdometry is not None:
                px4_sources = (
                    ("left", "/camera_left/fmu/out"),
                    ("right", "/camera_right/fmu/out"),
                    ("uam", "/fmu/out"),
                )
                for label, topic_prefix in px4_sources:
                    self._ros_node.create_subscription(
                        SensorGps,
                        f"{topic_prefix}/vehicle_gps_position",
                        lambda msg, label=label: self._bridge.px4_raw_gps.emit(label, msg),
                        qos_profile_sensor_data,
                    )
                    self._ros_node.create_subscription(
                        VehicleGlobalPosition,
                        f"{topic_prefix}/vehicle_global_position",
                        lambda msg, label=label: self._bridge.px4_global_position.emit(label, msg),
                        qos_profile_sensor_data,
                    )
                    self._ros_node.create_subscription(
                        VehicleOdometry,
                        f"{topic_prefix}/vehicle_odometry",
                        lambda msg, label=label: self._bridge.px4_odometry.emit(label, msg),
                        qos_profile_sensor_data,
                    )
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self._spin_thread.start()
            self._request_runtime_parameters()
        except Exception as exc:
            self._ros_node = None
            self._parameter_get_client = None
            self._parameter_list_client = None
            self._status.setText(f"Operator panel subscribe failed: {exc}")

    def _request_runtime_parameters(self):
        if self._parameter_list_client is None or ListParameters is None:
            self._bridge.runtime_parameters.emit(
                "Runtime parameter service is unavailable."
            )
            return
        request = ListParameters.Request()
        request.depth = 0
        future = self._parameter_list_client.call_async(request)
        future.add_done_callback(self._finish_runtime_parameter_names)

    def _finish_runtime_parameter_names(self, future):
        try:
            names = sorted(
                name for name in future.result().result.names if name != "use_sim_time"
            )
        except Exception as exc:
            self._bridge.runtime_parameters.emit(f"Run parameters unavailable: {exc}")
            return
        if not names or self._parameter_get_client is None:
            self._bridge.runtime_parameters.emit("No runtime parameters returned.")
            return
        request = GetParameters.Request()
        request.names = names
        values_future = self._parameter_get_client.call_async(request)
        values_future.add_done_callback(
            lambda result, parameter_names=names: self._finish_runtime_parameters(
                parameter_names, result
            )
        )

    def _finish_runtime_parameters(self, names, future):
        try:
            values = future.result().values
            lines = ["Source: /stereo_distance_node (live ROS parameters)", ""]
            for name, value in zip(names, values):
                lines.append(f"{name}: {self._format_parameter_value(value)}")
        except Exception as exc:
            self._bridge.runtime_parameters.emit(f"Run parameters unavailable: {exc}")
            return
        self._bridge.runtime_parameters.emit("\n".join(lines))

    @staticmethod
    def _parameter_value(value):
        value_type = int(getattr(value, "type", 0))
        if value_type == ParameterType.PARAMETER_BOOL:
            return bool(value.bool_value)
        if value_type == ParameterType.PARAMETER_INTEGER:
            return int(value.integer_value)
        if value_type == ParameterType.PARAMETER_DOUBLE:
            return float(value.double_value)
        if value_type == ParameterType.PARAMETER_STRING:
            return str(value.string_value)
        if value_type == ParameterType.PARAMETER_BYTE_ARRAY:
            return list(value.byte_array_value)
        if value_type == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(value.bool_array_value)
        if value_type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(value.integer_array_value)
        if value_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(value.double_array_value)
        if value_type == ParameterType.PARAMETER_STRING_ARRAY:
            return list(value.string_array_value)
        return None

    @staticmethod
    def _format_parameter_value(value):
        parsed = OperatorPanel._parameter_value(value)
        if isinstance(parsed, bool):
            return "true" if parsed else "false"
        if isinstance(parsed, float):
            return f"{parsed:g}"
        return "<not set>" if parsed is None else str(parsed)

    def _image_payload(self, msg):
        return (int(msg.width), int(msg.height), str(msg.encoding), int(msg.step), bytes(msg.data))

    def _emit_image_sample(self, name, msg):
        now = time.monotonic()
        min_period_s = 1.0 / RQT_IMAGE_RENDER_HZ
        with self._image_emit_lock:
            last = self._last_image_emit_monotonic.get(name, 0.0)
            if now - last < min_period_s:
                return
            self._last_image_emit_monotonic[name] = now
        self._bridge.image.emit(name, self._image_payload(msg))

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
        self._recording_controls.shutdown()
        self._record_preview.shutdown()
        if self._ros_graph_process is not None and self._ros_graph_process.poll() is None:
            try:
                os.killpg(self._ros_graph_process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        self._ros_graph_process = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
            self._ros_node = None
