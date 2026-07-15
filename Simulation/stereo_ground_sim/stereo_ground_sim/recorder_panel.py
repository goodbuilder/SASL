"""Standalone rqt wrapper for the shared Simulation recording controls."""

from __future__ import annotations

import os
import signal
import subprocess
from pathlib import Path

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from qt_gui.plugin import Plugin

from stereo_ground_sim.recording_control import (
    RecordingControlWidget,
    RecordingReadinessSubscriber,
    build_operator_recorder_command,
)


def build_calibration_recorder_command(
    *,
    python_executable,
    output_csv,
    truth_source,
    sim_truth_topic="",
    sim_truth_role="oracle",
):
    """Keep the legacy calibration entrypoint available to other rqt users."""
    command = [
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
        command.extend(["--sim-truth-topic", str(sim_truth_topic)])
    return command


class RecorderPanel(Plugin):
    """rqt compatibility wrapper; process ownership is shared with OperatorPanel."""

    def __init__(self, context, *, readiness_subscriber=None):
        super().__init__(context)
        self.setObjectName("RecorderPanel")
        self._widget = QWidget()
        self._widget.setObjectName("RecorderPanelWidget")
        self._widget.setWindowTitle("Simulation Recording")
        layout = QVBoxLayout(self._widget)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(QLabel("Simulation research recording"))
        self._recording_control = RecordingControlWidget(self._widget)
        self._recording_control.recording_event.connect(self._on_recording_event)
        self._readiness_subscriber = readiness_subscriber or RecordingReadinessSubscriber(self._widget)
        self._recording_control.set_readiness_model(self._readiness_subscriber.model)
        self._readiness_subscriber.readiness_changed.connect(self._recording_control.set_readiness)
        self._readiness_subscriber.error.connect(self._on_recording_event)
        self._recording_control.mode_combo.currentIndexChanged.connect(self._update_readiness_mode)
        layout.addWidget(self._recording_control)
        self._status_label = QLabel("IDLE")
        self._status_label.setWordWrap(True)
        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("RecorderPanel"))
        status_row.addWidget(self._status_label, 1)
        layout.addLayout(status_row)
        context.add_widget(self._widget)

        # Calibration recording remains a separate legacy command.  It is not
        # mixed with operator_recorder or allowed to block the Qt callback.
        self._calibration_process = None
        self._calibration_timer = QTimer(self._widget)
        self._calibration_timer.timeout.connect(self._poll_calibration)
        self._readiness_subscriber.start()

    def _update_readiness_mode(self, *_args):
        mode = str(self._recording_control.mode_combo.currentData() or "full")
        self._readiness_subscriber.set_mode(mode)

    def set_readiness(self, readiness):
        """Test/integration hook for applying a fixture or external snapshot."""
        self._recording_control.set_readiness(readiness)

    def set_parameter_snapshot_status(self, status):
        self._readiness_subscriber.set_parameter_snapshot_status(status)

    def _on_recording_event(self, message):
        self._status_label.setText(str(message))

    def _poll_calibration(self):
        if self._calibration_process is None:
            self._calibration_timer.stop()
            return
        return_code = self._calibration_process.poll()
        if return_code is None:
            return
        self._status_label.setText(
            "Calibration complete" if return_code == 0 else f"Calibration error: {return_code}"
        )
        self._calibration_process = None
        self._calibration_timer.stop()

    def start_calibration_csv(self, output_csv, truth_source, sim_truth_topic=""):
        if self._calibration_process is not None:
            self._status_label.setText("Calibration CSV already recording")
            return False
        command = build_calibration_recorder_command(
            python_executable=os.environ.get("OPERATOR_RECORDER_PYTHON", "python3"),
            output_csv=Path(output_csv),
            truth_source=truth_source,
            sim_truth_topic=sim_truth_topic,
        )
        try:
            self._calibration_process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except OSError as exc:
            self._status_label.setText(f"Calibration start failed: {exc}")
            self._calibration_process = None
            return False
        self._status_label.setText(f"Calibration recording: {output_csv}")
        self._calibration_timer.start(250)
        return True

    def stop_calibration_csv(self):
        if self._calibration_process is None:
            return False
        try:
            os.killpg(self._calibration_process.pid, signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass
        self._status_label.setText("Stopping calibration…")
        self._calibration_timer.start(100)
        return True

    def shutdown_plugin(self):
        self._recording_control.shutdown()
        self._recording_control.wait_for_shutdown(timeout_s=1.0)
        self._readiness_subscriber.shutdown()
        if self._calibration_process is not None:
            try:
                os.killpg(self._calibration_process.pid, signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
            self._calibration_process = None
        self._calibration_timer.stop()


__all__ = [
    "RecorderPanel",
    "build_operator_recorder_command",
    "build_calibration_recorder_command",
]
