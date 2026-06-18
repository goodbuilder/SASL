import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDockWidget,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from qt_gui.plugin import Plugin

try:
    import rclpy
    from std_msgs.msg import Float32, Float32MultiArray, String
except Exception:  # pragma: no cover - rqt can still show recorder controls without ROS imports.
    rclpy = None
    Float32 = None
    Float32MultiArray = None
    String = None


TRUTH_SOURCE_OPTIONS = (
    ("Local NED", "local_ned"),
    ("Global Position", "global_position"),
    ("GPS Position", "gps_position"),
)


def build_operator_recorder_command(*, python_executable, mode, output_dir):
    return [
        str(python_executable),
        "-m",
        "stereo_ground_sim.operator_recorder",
        "--mode",
        str(mode),
        "--output-dir",
        str(output_dir),
    ]


def build_calibration_recorder_command(
    *,
    python_executable,
    output_csv,
    truth_source,
    sim_truth_topic="",
    sim_truth_role="oracle",
):
    cmd = [
        str(python_executable),
        "-m",
        "stereo_ground_sim.calibration_csv_recorder",
        "--output-csv",
        str(output_csv),
        "--truth-source",
        str(truth_source),
        "--sim-truth-role",
        str(sim_truth_role),
    ]
    if sim_truth_topic:
        cmd.extend(["--sim-truth-topic", str(sim_truth_topic)])
    return cmd


class RecorderPanel(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("RecorderPanel")
        self._process = None
        self._record_mode = None
        self._record_path = None
        self._calibration_process = None
        self._calibration_path = None
        self._ros_node = None
        self._ros_subscriptions = []
        self._latest = {
            "yolo_ms": math.nan,
            "yolo_hz": math.nan,
            "truth_frd": None,
            "raw_frd": None,
            "corrected_frd": None,
            "profile_status": "",
        }
        self._latest_stamp = {}

        self._widget = QWidget()
        self._widget.setObjectName("RecorderPanelWidget")
        self._widget.setWindowTitle("Recorder")
        self._widget.setMinimumHeight(104)
        self._widget.setMaximumHeight(116)
        self._widget.setAutoFillBackground(True)
        self._widget.setStyleSheet(
            "#RecorderPanelWidget {"
            "background-color: #efefef;"
            "border-top: 1px solid #bdbdbd;"
            "border-bottom: 1px solid #bdbdbd;"
            "}"
        )

        self._status_label = QLabel("Idle")
        self._status_label.setMinimumWidth(120)
        self._status_label.setStyleSheet("font-weight: 600;")
        self._axis_contract_label = QLabel("FRD m: data[0]=Forward, data[1]=Right, data[2]=Down, data[3]=Range")
        self._axis_contract_label.setStyleSheet("font-weight: 600; color: #263238;")
        self._yolo_ms_label = QLabel("YOLO ms: --")
        self._yolo_hz_label = QLabel("YOLO Hz: --")
        self._truth_label = QLabel("Truth F/R/D/R: --")
        self._raw_label = QLabel("Raw F/R/D/R: --")
        self._corrected_label = QLabel("Corrected F/R/D/R: --")
        self._profile_label = QLabel("Profile: --")
        for label in (
            self._yolo_ms_label,
            self._yolo_hz_label,
            self._truth_label,
            self._raw_label,
            self._corrected_label,
            self._profile_label,
        ):
            label.setMinimumWidth(180)
            label.setStyleSheet("font-family: monospace;")

        default_dir = Path(os.environ.get("OPERATOR_BAG_DIR", Path.cwd() / "bags"))
        self._dir_edit = QLineEdit(str(default_dir))
        self._dir_edit.setMinimumWidth(260)

        self._browse_button = QPushButton("...")
        self._browse_button.setFixedWidth(34)
        self._lite_button = QPushButton("Start Lite")
        self._full_button = QPushButton("Start Full")
        self._debug_button = QPushButton("Start Debug")
        self._stop_button = QPushButton("Stop")
        self._stop_button.setEnabled(False)
        self._truth_source_combo = QComboBox()
        for label, value in TRUTH_SOURCE_OPTIONS:
            self._truth_source_combo.addItem(label, value)
        self._sim_oracle_check = QCheckBox("Gazebo oracle")
        self._sim_oracle_check.setChecked(True)
        self._calibration_start_button = QPushButton("Start Cal CSV")
        self._calibration_stop_button = QPushButton("Stop Cal")
        self._calibration_stop_button.setEnabled(False)

        root_layout = QVBoxLayout(self._widget)
        root_layout.setContentsMargins(8, 4, 8, 4)
        root_layout.setSpacing(3)

        control_layout = QHBoxLayout()
        control_layout.setSpacing(6)
        control_layout.addWidget(QLabel("Record"))
        control_layout.addWidget(self._lite_button)
        control_layout.addWidget(self._full_button)
        control_layout.addWidget(self._debug_button)
        control_layout.addWidget(self._stop_button)
        control_layout.addWidget(QLabel("Save root"))
        control_layout.addWidget(self._dir_edit, 1)
        control_layout.addWidget(self._browse_button)
        control_layout.addWidget(QLabel("Truth"))
        control_layout.addWidget(self._truth_source_combo)
        control_layout.addWidget(self._sim_oracle_check)
        control_layout.addWidget(self._calibration_start_button)
        control_layout.addWidget(self._calibration_stop_button)
        control_layout.addWidget(self._status_label)

        telemetry_layout = QGridLayout()
        telemetry_layout.setContentsMargins(0, 0, 0, 0)
        telemetry_layout.setHorizontalSpacing(18)
        telemetry_layout.setVerticalSpacing(2)
        telemetry_layout.addWidget(self._axis_contract_label, 0, 0, 1, 3)
        telemetry_layout.addWidget(self._yolo_ms_label, 1, 0)
        telemetry_layout.addWidget(self._yolo_hz_label, 1, 1)
        telemetry_layout.addWidget(self._profile_label, 1, 2)
        telemetry_layout.addWidget(self._truth_label, 2, 0)
        telemetry_layout.addWidget(self._raw_label, 2, 1)
        telemetry_layout.addWidget(self._corrected_label, 2, 2)
        root_layout.addLayout(control_layout)
        root_layout.addLayout(telemetry_layout)

        self._browse_button.clicked.connect(self._choose_directory)
        self._lite_button.clicked.connect(lambda: self._start_recording("lite"))
        self._full_button.clicked.connect(lambda: self._start_recording("full"))
        self._debug_button.clicked.connect(lambda: self._start_recording("debug"))
        self._stop_button.clicked.connect(self._stop_recording)
        self._calibration_start_button.clicked.connect(self._start_calibration_csv)
        self._calibration_stop_button.clicked.connect(self._stop_calibration_csv)

        self._timer = QTimer(self._widget)
        self._timer.timeout.connect(self._poll_process)
        self._timer.timeout.connect(self._spin_status_node)
        self._timer.timeout.connect(self._update_status_labels)
        self._timer.start(500)

        self._layout_timer = QTimer(self._widget)
        self._layout_timer.timeout.connect(self._update_overlay_geometry)
        self._layout_timer.start(1000)
        QTimer.singleShot(0, self._attach_overlay)
        QTimer.singleShot(1500, self._attach_overlay)
        QTimer.singleShot(3500, self._attach_overlay)
        self._init_status_subscriptions()

    def _attach_overlay(self):
        app = QApplication.instance()
        if app is None:
            return
        for widget in app.allWidgets():
            if isinstance(widget, QMainWindow):
                self._widget.setParent(widget)
                self._widget.setWindowTitle("Recorder")
                self._widget.show()
                self._update_overlay_geometry()
                return

    def _update_overlay_geometry(self):
        parent = self._widget.parentWidget()
        if parent is None:
            return

        image_bottom = None
        for widget in QApplication.instance().allWidgets():
            if not isinstance(widget, QDockWidget):
                continue
            if widget.windowTitle() not in ("Image View", "Image View (2)"):
                continue
            bottom = widget.geometry().bottom()
            image_bottom = bottom if image_bottom is None else max(image_bottom, bottom)

        y = 0 if image_bottom is None else image_bottom + 4
        self._widget.setGeometry(0, y, parent.width(), 108)
        self._widget.raise_()

    def _init_status_subscriptions(self):
        if rclpy is None or Float32 is None or Float32MultiArray is None or String is None:
            self._profile_label.setText("Profile: ROS imports unavailable")
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._ros_node = rclpy.create_node("operator_recorder_panel_status")
            self._ros_subscriptions = [
                self._ros_node.create_subscription(
                    Float32,
                    "/stereo/yolo_inference_ms",
                    lambda msg: self._set_scalar("yolo_ms", msg.data),
                    10,
                ),
                self._ros_node.create_subscription(
                    Float32,
                    "/stereo/yolo_update_hz",
                    lambda msg: self._set_scalar("yolo_hz", msg.data),
                    10,
                ),
                self._ros_node.create_subscription(
                    Float32MultiArray,
                    "/ground_truth_position",
                    lambda msg: self._set_frd("truth_frd", msg.data),
                    10,
                ),
                self._ros_node.create_subscription(
                    Float32MultiArray,
                    "/stereo/estimated_position",
                    lambda msg: self._set_frd("raw_frd", msg.data),
                    10,
                ),
                self._ros_node.create_subscription(
                    Float32MultiArray,
                    "/stereo/corrected_position",
                    lambda msg: self._set_frd("corrected_frd", msg.data),
                    10,
                ),
                self._ros_node.create_subscription(
                    String,
                    "/stereo/profile_status",
                    lambda msg: self._set_profile_status(msg.data),
                    10,
                ),
            ]
        except Exception as exc:
            self._ros_node = None
            self._profile_label.setText(f"Profile: status subscribe failed: {exc}")

    def _spin_status_node(self):
        if self._ros_node is None or rclpy is None:
            return
        try:
            rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        except Exception as exc:
            self._profile_label.setText(f"Profile: spin failed: {exc}")

    def _set_scalar(self, key, value):
        self._latest[key] = float(value)
        self._latest_stamp[key] = time.monotonic()

    def _set_frd(self, key, data):
        values = list(data)
        while len(values) < 4:
            values.append(math.nan)
        self._latest[key] = tuple(float(value) for value in values[:4])
        self._latest_stamp[key] = time.monotonic()

    def _set_profile_status(self, value):
        text = str(value)
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            parsed = {}
        if isinstance(parsed, dict):
            profile_id = parsed.get("profile_id") or "none"
            mode = parsed.get("profile_mode") or parsed.get("mode") or "none"
            fallback = parsed.get("fallback_reason") or ""
            text = f"{profile_id}/{mode} {fallback}".strip()
        self._latest["profile_status"] = text[:80]
        self._latest_stamp["profile_status"] = time.monotonic()

    def _format_float(self, value, precision=1):
        if value is None or not math.isfinite(float(value)):
            return "--"
        return f"{float(value):.{precision}f}"

    def _format_frd(self, values):
        if values is None:
            return "--"
        return "/".join(self._format_float(value, 1) for value in values)

    def _is_stale(self, key, max_age_s=3.0):
        stamp = self._latest_stamp.get(key)
        return stamp is None or time.monotonic() - stamp > max_age_s

    def _update_status_labels(self):
        yolo_ms = self._format_float(self._latest["yolo_ms"], 1)
        yolo_hz = self._format_float(self._latest["yolo_hz"], 1)
        self._yolo_ms_label.setText(f"YOLO ms: {yolo_ms}{' stale' if self._is_stale('yolo_ms') else ''}")
        self._yolo_hz_label.setText(f"YOLO Hz: {yolo_hz}{' stale' if self._is_stale('yolo_hz') else ''}")
        self._truth_label.setText(f"Truth F/R/D/R: {self._format_frd(self._latest['truth_frd'])}")
        self._raw_label.setText(f"Raw F/R/D/R: {self._format_frd(self._latest['raw_frd'])}")
        self._corrected_label.setText(f"Corrected F/R/D/R: {self._format_frd(self._latest['corrected_frd'])}")
        profile = self._latest["profile_status"] or "--"
        self._profile_label.setText(f"Profile: {profile}")

    def _choose_directory(self):
        directory = QFileDialog.getExistingDirectory(
            self._widget,
            "Select recording save root",
            self._dir_edit.text().strip() or str(Path.cwd() / "bags"),
        )
        if directory:
            self._dir_edit.setText(directory)

    def _start_recording(self, mode):
        if self._process is not None:
            self._set_status(f"Recording {self._record_mode}", error=True)
            return

        save_root = Path(self._dir_edit.text().strip() or Path.cwd() / "bags").expanduser()
        save_root.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        output_path = save_root / f"operator_{timestamp}_{mode}"
        recorder_python = os.environ.get("OPERATOR_RECORDER_PYTHON", "python3")

        cmd = build_operator_recorder_command(
            python_executable=recorder_python,
            mode=mode,
            output_dir=output_path,
        )
        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except OSError as exc:
            self._set_status(f"Start failed: {exc}", error=True)
            self._process = None
            return

        self._record_mode = mode
        self._record_path = output_path
        self._lite_button.setEnabled(False)
        self._full_button.setEnabled(False)
        self._debug_button.setEnabled(False)
        self._stop_button.setEnabled(True)
        self._dir_edit.setEnabled(False)
        self._browse_button.setEnabled(False)
        self._set_status(f"REC {mode}: {output_path.name}")

    def _selected_truth_source(self):
        return str(self._truth_source_combo.currentData() or "local_ned")

    def _start_calibration_csv(self):
        if self._calibration_process is not None:
            self._set_status("Calibration CSV already recording", error=True)
            return

        save_root = Path(self._dir_edit.text().strip() or Path.cwd() / "bags").expanduser()
        save_root.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        truth_source = self._selected_truth_source()
        output_path = save_root / f"calibration_{timestamp}_{truth_source}.csv"
        recorder_python = os.environ.get("OPERATOR_RECORDER_PYTHON", "python3")
        sim_truth_topic = "/ground_truth_position" if self._sim_oracle_check.isChecked() else ""
        cmd = build_calibration_recorder_command(
            python_executable=recorder_python,
            output_csv=output_path,
            truth_source=truth_source,
            sim_truth_topic=sim_truth_topic,
            sim_truth_role="oracle",
        )
        try:
            self._calibration_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except OSError as exc:
            self._set_status(f"Cal start failed: {exc}", error=True)
            self._calibration_process = None
            return

        self._calibration_path = output_path
        self._truth_source_combo.setEnabled(False)
        self._sim_oracle_check.setEnabled(False)
        self._calibration_start_button.setEnabled(False)
        self._calibration_stop_button.setEnabled(True)
        self._set_status(f"CAL {truth_source}: {output_path.name}")

    def _stop_calibration_csv(self):
        if self._calibration_process is None:
            return

        process = self._calibration_process
        self._set_status("Stopping calibration...")
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            process.wait(timeout=5.0)

        stopped_path = self._calibration_path
        self._reset_calibration_controls()
        if stopped_path is not None:
            self._set_status(f"Cal saved: {stopped_path.name}")
        else:
            self._set_status("Calibration stopped")

    def _stop_recording(self):
        if self._process is None:
            return

        process = self._process
        self._set_status("Stopping...")
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            process.wait(timeout=5.0)

        stopped_path = self._record_path
        self._reset_controls()
        if stopped_path is not None:
            self._set_status(f"Saved: {stopped_path.name}")
        else:
            self._set_status("Stopped")

    def _poll_process(self):
        self._poll_calibration_process()
        if self._process is None:
            return

        return_code = self._process.poll()
        if return_code is None:
            return

        path = self._record_path
        self._reset_controls()
        if return_code == 0:
            self._set_status(f"Saved: {path.name if path else 'bag'}")
        else:
            self._set_status(f"Recorder exited: {return_code}", error=True)

    def _poll_calibration_process(self):
        if self._calibration_process is None:
            return
        return_code = self._calibration_process.poll()
        if return_code is None:
            return
        path = self._calibration_path
        self._reset_calibration_controls()
        if return_code == 0:
            self._set_status(f"Cal saved: {path.name if path else 'calibration.csv'}")
        else:
            self._set_status(f"Cal recorder exited: {return_code}", error=True)

    def _reset_controls(self):
        self._process = None
        self._record_mode = None
        self._record_path = None
        self._lite_button.setEnabled(True)
        self._full_button.setEnabled(True)
        self._debug_button.setEnabled(True)
        self._stop_button.setEnabled(False)
        self._dir_edit.setEnabled(True)
        self._browse_button.setEnabled(True)

    def _reset_calibration_controls(self):
        self._calibration_process = None
        self._calibration_path = None
        self._truth_source_combo.setEnabled(True)
        self._sim_oracle_check.setEnabled(True)
        self._calibration_start_button.setEnabled(True)
        self._calibration_stop_button.setEnabled(False)

    def _set_status(self, text, error=False):
        self._status_label.setText(text)
        if error:
            self._status_label.setStyleSheet("font-weight: 600; color: #b00020;")
        else:
            self._status_label.setStyleSheet("font-weight: 600; color: #1b5e20;")

    def shutdown_plugin(self):
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
            self._ros_node = None
        if self._calibration_process is not None:
            self._stop_calibration_csv()
        if self._process is not None:
            self._stop_recording()
