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
    QWidget,
)
from qt_gui.plugin import Plugin


class RecorderPanel(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("RecorderPanel")
        self._process = None
        self._record_mode = None
        self._record_path = None

        self._widget = QWidget()
        self._widget.setObjectName("RecorderPanelWidget")
        self._widget.setWindowTitle("Recorder")
        self._widget.setMinimumHeight(42)
        self._widget.setMaximumHeight(58)
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

        layout = QHBoxLayout(self._widget)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(6)
        layout.addWidget(QLabel("Record"))
        layout.addWidget(self._lite_button)
        layout.addWidget(self._full_button)
        layout.addWidget(self._debug_button)
        layout.addWidget(self._stop_button)
        layout.addWidget(QLabel("Save root"))
        layout.addWidget(self._dir_edit, 1)
        layout.addWidget(self._browse_button)
        layout.addWidget(self._status_label)

        self._browse_button.clicked.connect(self._choose_directory)
        self._lite_button.clicked.connect(lambda: self._start_recording("lite"))
        self._full_button.clicked.connect(lambda: self._start_recording("full"))
        self._debug_button.clicked.connect(lambda: self._start_recording("debug"))
        self._stop_button.clicked.connect(self._stop_recording)

        self._timer = QTimer(self._widget)
        self._timer.timeout.connect(self._poll_process)
        self._timer.start(500)

        self._layout_timer = QTimer(self._widget)
        self._layout_timer.timeout.connect(self._update_overlay_geometry)
        self._layout_timer.start(1000)
        QTimer.singleShot(0, self._attach_overlay)
        QTimer.singleShot(1500, self._attach_overlay)
        QTimer.singleShot(3500, self._attach_overlay)

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
        self._widget.setGeometry(0, y, parent.width(), 54)
        self._widget.raise_()

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

        cmd = [
            recorder_python,
            "-m",
            "stereo_ground_sim.operator_recorder",
            "--mode",
            mode,
            "--output-dir",
            str(output_path),
        ]
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

    def _set_status(self, text, error=False):
        self._status_label.setText(text)
        if error:
            self._status_label.setStyleSheet("font-weight: 600; color: #b00020;")
        else:
            self._status_label.setStyleSheet("font-weight: 600; color: #1b5e20;")

    def shutdown_plugin(self):
        if self._process is not None:
            self._stop_recording()
