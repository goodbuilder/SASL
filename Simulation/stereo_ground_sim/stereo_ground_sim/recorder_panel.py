import csv
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
    QDockWidget,
    QFileDialog,
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


PRIMARY_RECORD_BUTTON_TEXT = "Save CSV"
RECORDING_MODE_BUTTONS = (
    ("lite", PRIMARY_RECORD_BUTTON_TEXT),
    ("full", "Save MP4"),
    ("debug", "Save Debug"),
)
ANALYSIS_CSV_FIELDS = (
    "time_sec",
    "raw_range_m",
    "left_x1",
    "right_x1",
    "raw_gps_baseline_forward_m",
    "global_position_baseline_forward_m",
    "raw_minus_global_baseline_delta_m",
)


def _read_csv_tail(path, data_row_count):
    """Return the header and complete persisted data lines from a small file tail."""
    target = Path(path)
    if data_row_count < 1:
        raise ValueError("data_row_count must be positive")
    with target.open("rb") as handle:
        header = handle.readline().decode("utf-8", errors="replace").rstrip("\r\n")
        handle.seek(0, os.SEEK_END)
        position = handle.tell()
        chunks = []
        newline_count = 0
        while position > 0 and newline_count <= data_row_count:
            chunk_size = min(8192, position)
            position -= chunk_size
            handle.seek(position)
            chunk = handle.read(chunk_size)
            chunks.append(chunk)
            newline_count += chunk.count(b"\n")
    tail = b"".join(reversed(chunks))
    if tail and not tail.endswith(b"\n"):
        tail = tail.rsplit(b"\n", 1)[0] if b"\n" in tail else b""
    decoded_rows = [line.decode("utf-8", errors="replace") for line in tail.splitlines()]
    data_rows = [line for line in decoded_rows if line and line != header]
    return header, data_rows[-data_row_count:]


def read_recent_csv_lines(path, data_row_count=3):
    """Return the CSV header plus the latest persisted rows without rereading the file."""
    header, data_rows = _read_csv_tail(path, data_row_count)
    return "\n".join([header, *data_rows[-data_row_count:]]) if header else ""


def read_recent_csv_rows(path, data_row_count=3):
    """Return only complete persisted rows, keyed by the original CSV header."""
    header, data_rows = _read_csv_tail(path, data_row_count)
    if not header or not data_rows:
        return []
    return list(csv.DictReader([header, *data_rows]))


def export_analysis_csv(source_path, output_path):
    """Create a compact analysis copy without modifying the immutable raw CSV."""
    source = Path(source_path)
    output = Path(output_path)
    with source.open(newline="", encoding="utf-8") as source_file:
        reader = csv.DictReader(source_file)
        with output.open("w", newline="", encoding="utf-8") as output_file:
            writer = csv.DictWriter(output_file, fieldnames=ANALYSIS_CSV_FIELDS)
            writer.writeheader()
            for row in reader:
                writer.writerow({field: row.get(field, "") for field in ANALYSIS_CSV_FIELDS})


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


def simulation_root():
    """Resolve the source workspace that owns the mission shell runners."""
    candidates = [Path.cwd(), *Path(__file__).resolve().parents]
    for candidate in candidates:
        if (
            (candidate / "run_auto_mission.sh").is_file()
            and (candidate / "run_px4_waypoint_mission.sh").is_file()
        ):
            return candidate
    return Path.cwd()


def build_mission_command(kind, *, root_dir=None):
    root = Path(root_dir) if root_dir is not None else simulation_root()
    scripts = {
        "auto": "run_auto_mission.sh",
        "waypoint": "run_px4_waypoint_mission.sh",
    }
    script = scripts.get(kind)
    if script is None:
        raise ValueError(f"unknown mission kind: {kind}")
    return [str(root / script)]


class RecordingControls(QWidget):
    """Reusable recorder controls for the tabbed operator UI."""

    def __init__(self, parent=None, lock_widgets=()):
        super().__init__(parent)
        self._process = None
        self._record_mode = None
        self._record_path = None
        self._last_record_path = None
        self._lock_widgets = tuple(lock_widgets)
        self._mission_processes = {}

        default_dir = Path(os.environ.get("OPERATOR_BAG_DIR", Path.cwd() / "bags"))
        self._dir_edit = QLineEdit(str(default_dir))
        self._dir_edit.setMinimumWidth(320)
        self._browse_button = QPushButton("Browse…")
        self._record_buttons = {
            mode: QPushButton(button_text) for mode, button_text in RECORDING_MODE_BUTTONS
        }
        self._mission_buttons = {
            "auto": QPushButton("Auto Mission"),
            "waypoint": QPushButton("Waypoint"),
        }
        self._stop_button = QPushButton("Stop")
        self._stop_button.setEnabled(False)
        self._status_label = QLabel("Idle")
        self._status_label.setMinimumWidth(180)
        self._status_label.setStyleSheet("font-weight: 600;")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)
        layout.addWidget(QLabel("Flight recording"))
        controls = QHBoxLayout()
        controls.setSpacing(6)
        for button in self._record_buttons.values():
            controls.addWidget(button)
        for button in self._mission_buttons.values():
            controls.addWidget(button)
        controls.addWidget(self._stop_button)
        controls.addWidget(QLabel("Save to"))
        controls.addWidget(self._dir_edit, 1)
        controls.addWidget(self._browse_button)
        controls.addWidget(self._status_label)
        layout.addLayout(controls)
        layout.addWidget(
            QLabel(
                "Save CSV records telemetry and detections. Save MP4 adds left/right "
                "overlay videos. Save Debug also adds a rosbag. Each detection stores "
                "left/right bbox x1,y1,x2,y2 plus derived centers; vectors use FRD metres."
            )
        )
        self.setToolTip(
            "CSV: telemetry. MP4: CSV plus overlay video. Debug: MP4 plus rosbag. "
            "Stop first to change the output directory."
        )

        self._browse_button.clicked.connect(self._choose_directory)
        for mode, button in self._record_buttons.items():
            button.clicked.connect(
                lambda _checked=False, record_mode=mode: self._start_recording(record_mode)
            )
        for kind, button in self._mission_buttons.items():
            button.clicked.connect(
                lambda _checked=False, mission_kind=kind: self._start_mission(mission_kind)
            )
        self._stop_button.clicked.connect(self._stop_recording)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll_process)
        self._timer.start(500)

    def _choose_directory(self):
        directory = QFileDialog.getExistingDirectory(
            self,
            "Select recording save root",
            self._dir_edit.text().strip() or str(Path.cwd() / "bags"),
        )
        if directory:
            self._dir_edit.setText(directory)

    def _start_recording(self, mode):
        if self._process is not None:
            return
        save_root = Path(self._dir_edit.text().strip() or Path.cwd() / "bags").expanduser()
        try:
            save_root.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self._set_status(f"Save directory failed: {exc}", error=True)
            return
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        output_path = save_root / f"operator_{timestamp}_{mode}"
        cmd = build_operator_recorder_command(
            python_executable=os.environ.get("OPERATOR_RECORDER_PYTHON", "python3"),
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
            return
        self._record_mode = mode
        self._record_path = output_path
        self._last_record_path = output_path
        self._set_record_buttons_enabled(False)
        self._stop_button.setEnabled(True)
        self._dir_edit.setEnabled(False)
        self._browse_button.setEnabled(False)
        self._set_locked_widgets_enabled(False)
        self._set_status(f"Recording: {output_path.name}")

    def _start_mission(self, kind):
        process = self._mission_processes.get(kind)
        if process is not None and process.poll() is None:
            self._set_status(f"{kind.title()} mission already running", error=True)
            return
        root = simulation_root()
        cmd = build_mission_command(kind, root_dir=root)
        try:
            process = subprocess.Popen(
                cmd,
                cwd=str(root),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except OSError as exc:
            self._set_status(f"Mission start failed: {exc}", error=True)
            return
        self._mission_processes[kind] = process
        self._mission_buttons[kind].setEnabled(False)
        label = "Auto Mission" if kind == "auto" else "Waypoint"
        self._set_status(f"Started: {label}")

    def _stop_recording(self):
        if self._process is None:
            return
        process = self._process
        self._set_status("Stopping…")
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
        path = self._record_path
        self._reset_controls()
        self._set_status(f"Saved: {path.name}" if path else "Stopped")

    def _poll_process(self):
        self._poll_missions()
        if self._process is None:
            return
        return_code = self._process.poll()
        if return_code is None:
            return
        path = self._record_path
        self._reset_controls()
        self._set_status(
            f"Saved: {path.name if path else 'recording'}"
            if return_code == 0
            else f"Recorder exited: {return_code}",
            error=return_code != 0,
        )

    def _poll_missions(self):
        for kind, process in list(self._mission_processes.items()):
            return_code = process.poll()
            if return_code is None:
                continue
            self._mission_processes.pop(kind, None)
            self._mission_buttons[kind].setEnabled(True)
            label = "Auto Mission" if kind == "auto" else "Waypoint"
            self._set_status(
                f"{label} complete" if return_code == 0 else f"{label} exited: {return_code}",
                error=return_code != 0,
            )

    def _reset_controls(self):
        self._process = None
        self._record_mode = None
        self._record_path = None
        self._set_record_buttons_enabled(True)
        self._stop_button.setEnabled(False)
        self._dir_edit.setEnabled(True)
        self._browse_button.setEnabled(True)
        self._set_locked_widgets_enabled(True)

    def _set_locked_widgets_enabled(self, enabled):
        for widget in self._lock_widgets:
            widget.setEnabled(enabled)

    def _set_record_buttons_enabled(self, enabled):
        for button in self._record_buttons.values():
            button.setEnabled(enabled)

    def add_lock_widgets(self, *widgets):
        self._lock_widgets += tuple(widgets)

    def recording_csv_path(self):
        return self._recording_artifact_path("csv", "telemetry*.csv")

    def recording_xlsx_path(self):
        return self._recording_artifact_path("xlsx", "session*.xlsx")

    def _recording_artifact_path(self, progress_key, pattern):
        if self._last_record_path is None:
            return None
        progress_path = self._last_record_path / "progress.json"
        try:
            progress = json.loads(progress_path.read_text(encoding="utf-8"))
            artifact_name = progress.get(progress_key, "")
        except (OSError, json.JSONDecodeError):
            artifact_name = ""
        if isinstance(artifact_name, str) and artifact_name:
            candidate = self._last_record_path / Path(artifact_name).name
            if candidate.is_file():
                return candidate
        candidates = sorted(self._last_record_path.glob(pattern), reverse=True)
        if candidates:
            return candidates[0]
        return self._last_record_path / ("telemetry.csv" if progress_key == "csv" else "session.xlsx")

    def recording_session_path(self):
        return self._last_record_path

    def is_recording(self):
        return self._process is not None and self._process.poll() is None

    def _set_status(self, text, error=False):
        self._status_label.setText(text)
        color = "#b00020" if error else "#1b5e20"
        self._status_label.setStyleSheet(f"font-weight: 600; color: {color};")

    def shutdown(self):
        self._timer.stop()
        if self._process is not None:
            self._stop_recording()
        for process in self._mission_processes.values():
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        self._mission_processes.clear()


class RecorderPanel(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("RecorderPanel")
        self._process = None
        self._record_mode = None
        self._record_path = None
        self._mission_processes = {}
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
        # Keep this plugin as a compact recording toolbar. The primary display
        # is standard rqt Image View plus Plot, so this panel must not consume
        # their vertical workspace.
        self._widget.setFixedHeight(42)
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
        self._record_buttons = {
            mode: QPushButton(button_text) for mode, button_text in RECORDING_MODE_BUTTONS
        }
        self._mission_buttons = {
            "auto": QPushButton("Auto Mission"),
            "waypoint": QPushButton("Waypoint"),
        }
        self._stop_button = QPushButton("Stop")
        self._stop_button.setEnabled(False)

        root_layout = QVBoxLayout(self._widget)
        root_layout.setContentsMargins(8, 4, 8, 4)
        root_layout.setSpacing(3)

        control_layout = QHBoxLayout()
        control_layout.setSpacing(6)
        control_layout.addWidget(QLabel("Flight recording"))
        for button in self._record_buttons.values():
            control_layout.addWidget(button)
        for button in self._mission_buttons.values():
            control_layout.addWidget(button)
        control_layout.addWidget(self._stop_button)
        control_layout.addWidget(QLabel("Save to"))
        control_layout.addWidget(self._dir_edit, 1)
        control_layout.addWidget(self._browse_button)
        control_layout.addWidget(self._status_label)

        root_layout.addLayout(control_layout)
        self._widget.setToolTip(
            "CSV/bag recording controls. Saved detections include both image "
            "bounding boxes (left/right x1,y1,x2,y2) and derived centers. "
            "Plot data uses public FRD metres: Forward, Right, Down."
        )

        self._browse_button.clicked.connect(self._choose_directory)
        for mode, button in self._record_buttons.items():
            button.clicked.connect(
                lambda _checked=False, record_mode=mode: self._start_recording(record_mode)
            )
        for kind, button in self._mission_buttons.items():
            button.clicked.connect(
                lambda _checked=False, mission_kind=kind: self._start_mission(mission_kind)
            )
        self._stop_button.clicked.connect(self._stop_recording)

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
        self._set_record_buttons_enabled(False)
        self._stop_button.setEnabled(True)
        self._dir_edit.setEnabled(False)
        self._browse_button.setEnabled(False)
        self._set_status(f"REC {mode}: {output_path.name}")

    def _start_mission(self, kind):
        process = self._mission_processes.get(kind)
        if process is not None and process.poll() is None:
            self._set_status(f"{kind.title()} mission already running", error=True)
            return
        root = simulation_root()
        cmd = build_mission_command(kind, root_dir=root)
        try:
            process = subprocess.Popen(
                cmd,
                cwd=str(root),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
        except OSError as exc:
            self._set_status(f"Mission start failed: {exc}", error=True)
            return
        self._mission_processes[kind] = process
        self._mission_buttons[kind].setEnabled(False)
        label = "Auto Mission" if kind == "auto" else "Waypoint"
        self._set_status(f"Started: {label}")

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
        self._poll_missions()
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

    def _poll_missions(self):
        for kind, process in list(self._mission_processes.items()):
            return_code = process.poll()
            if return_code is None:
                continue
            self._mission_processes.pop(kind, None)
            self._mission_buttons[kind].setEnabled(True)
            label = "Auto Mission" if kind == "auto" else "Waypoint"
            self._set_status(
                f"{label} complete" if return_code == 0 else f"{label} exited: {return_code}",
                error=return_code != 0,
            )

    def _reset_controls(self):
        self._process = None
        self._record_mode = None
        self._record_path = None
        self._set_record_buttons_enabled(True)
        self._stop_button.setEnabled(False)
        self._dir_edit.setEnabled(True)
        self._browse_button.setEnabled(True)

    def _set_status(self, text, error=False):
        self._status_label.setText(text)
        if error:
            self._status_label.setStyleSheet("font-weight: 600; color: #b00020;")
        else:
            self._status_label.setStyleSheet("font-weight: 600; color: #1b5e20;")

    def _set_record_buttons_enabled(self, enabled):
        for button in self._record_buttons.values():
            button.setEnabled(enabled)

    def shutdown_plugin(self):
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
            self._ros_node = None
        if self._process is not None:
            self._stop_recording()
        for process in self._mission_processes.values():
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        self._mission_processes.clear()
