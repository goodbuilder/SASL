"""Shared user-controlled recording lifecycle for the Simulation UI.

The Qt widget in this module is deliberately small.  All process ownership,
mode mapping, output naming, and asynchronous shutdown live in the controller
so the embedded OperatorPanel and the standalone RecorderPanel cannot drift.
"""

from __future__ import annotations

import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from python_qt_binding.QtCore import QObject, QTimer, Qt, Signal
from python_qt_binding.QtWidgets import (
    QFileDialog,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


MODE_OPTIONS = (
    ("Data Only", "lite"),
    ("Research", "full"),
    ("Debug/Reprocess", "debug"),
)
MODE_TO_LABEL = {value: label for label, value in MODE_OPTIONS}
RECORDING_STATES = ("IDLE", "STARTING", "RECORDING", "STOPPING", "COMPLETE", "ERROR")
_SAFE_RUN_NAME = re.compile(r"[^A-Za-z0-9._-]+")
_ACTIVE_ROOT_LOCKS: set[str] = set()


def mode_value(label: str) -> str:
    """Map a user-facing mode label to the recorder mode."""
    for option_label, value in MODE_OPTIONS:
        if str(label) == option_label:
            return value
    return "full"


def mode_label(value: str) -> str:
    return MODE_TO_LABEL.get(str(value), "Research")


def sanitize_run_name(value: str) -> str:
    """Return a path-component-safe run name without accepting traversal."""
    sanitized = _SAFE_RUN_NAME.sub("_", str(value or "").strip())
    sanitized = sanitized.strip("._") or "run"
    if sanitized in {".", ".."}:
        return "run"
    return sanitized


def default_record_root() -> Path:
    """Resolve the reproducible default without creating it."""
    explicit = os.environ.get("OPERATOR_RECORD_ROOT", "").strip()
    if explicit:
        return Path(explicit).expanduser()
    legacy = os.environ.get("OPERATOR_BAG_DIR", "").strip()
    if legacy:
        return Path(legacy).expanduser()
    # .../Simulation/stereo_ground_sim/stereo_ground_sim/recording_control.py
    return Path(__file__).resolve().parents[2] / "results" / "operator_runs"


def unique_output_dir(
    save_root: str | os.PathLike[str],
    run_name: str,
    mode: str,
    *,
    now: datetime | None = None,
) -> Path:
    """Build a unique output path without touching the filesystem."""
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    root = Path(save_root).expanduser()
    base = root / f"{timestamp}_{sanitize_run_name(run_name)}_{mode}"
    candidate = base
    suffix = 2
    while candidate.exists():
        candidate = root / f"{base.name}_{suffix}"
        suffix += 1
    return candidate


def build_operator_recorder_command(
    *,
    python_executable: str | os.PathLike[str] | None = None,
    mode: str,
    output_dir: str | os.PathLike[str],
) -> list[str]:
    return [
        str(python_executable or os.environ.get("OPERATOR_RECORDER_PYTHON", sys.executable)),
        "-m",
        "stereo_ground_sim.operator_recorder",
        "--mode",
        str(mode),
        "--output-dir",
        str(output_dir),
    ]


def _send_group_signal(process, sig) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, sig)
    except (ProcessLookupError, PermissionError):
        try:
            process.send_signal(sig)
        except (ProcessLookupError, PermissionError):
            pass


class RecordingSessionController(QObject):
    """Own one operator_recorder process without blocking the Qt thread."""

    state_changed = Signal(str)
    elapsed_changed = Signal(str)
    output_changed = Signal(str)
    event_message = Signal(str)
    error = Signal(str)

    def __init__(
        self,
        parent=None,
        *,
        command_builder=build_operator_recorder_command,
        process_factory=subprocess.Popen,
        poll_interval_ms=200,
        stop_timeout_s=10.0,
        terminate_timeout_s=5.0,
    ):
        super().__init__(parent)
        self._command_builder = command_builder
        self._process_factory = process_factory
        self._poll_interval_ms = int(poll_interval_ms)
        self._stop_timeout_s = float(stop_timeout_s)
        self._terminate_timeout_s = float(terminate_timeout_s)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.poll)
        self._process = None
        self._state = "IDLE"
        self._mode = None
        self._output_path = None
        self._started_monotonic = None
        self._stop_stage = None
        self._stop_deadline = None
        self._lock_key = None
        self._lock_file = None

    @property
    def state(self) -> str:
        return self._state

    @property
    def process(self):
        return self._process

    @property
    def output_path(self) -> Path | None:
        return self._output_path

    @property
    def active(self) -> bool:
        return self._process is not None and self._state in {"STARTING", "RECORDING", "STOPPING"}

    def _set_state(self, state: str) -> None:
        self._state = str(state)
        self.state_changed.emit(self._state)

    def _release_lock(self) -> None:
        if self._lock_key is not None:
            _ACTIVE_ROOT_LOCKS.discard(self._lock_key)
            self._lock_key = None
        if self._lock_file is not None:
            try:
                import fcntl

                fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
            except (ImportError, OSError):
                pass
            try:
                self._lock_file.close()
            except OSError:
                pass
            self._lock_file = None

    def _acquire_lock(self, root: Path) -> bool:
        key = str(root.resolve())
        if key in _ACTIVE_ROOT_LOCKS:
            return False
        root.mkdir(parents=True, exist_ok=True)
        lock_path = root / ".operator_recording.lock"
        try:
            lock_file = lock_path.open("a+", encoding="utf-8")
            try:
                import fcntl

                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except ImportError:
                pass
            except OSError:
                lock_file.close()
                return False
        except OSError:
            return False
        _ACTIVE_ROOT_LOCKS.add(key)
        self._lock_key = key
        self._lock_file = lock_file
        return True

    def start(self, *, mode: str, run_name: str, save_root: str | os.PathLike[str]) -> bool:
        if self.active:
            self.error.emit("Recording already active")
            return False
        if os.environ.get("OPERATOR_RECORDING_MANAGED_EXTERNALLY", "").strip() in {"1", "true", "yes"}:
            self._set_state("ERROR")
            self.error.emit("EXTERNAL RECORDING ACTIVE")
            self.event_message.emit("recording start rejected: external CLI recording is active")
            return False

        mode = str(mode)
        if mode not in {"lite", "full", "debug"}:
            self._set_state("ERROR")
            self.error.emit(f"Unsupported recording mode: {mode}")
            return False

        root = Path(save_root).expanduser()
        if not self._acquire_lock(root):
            self._set_state("ERROR")
            self.error.emit(f"Recording root is already locked: {root}")
            return False

        output_path = unique_output_dir(root, run_name, mode)
        try:
            output_path.mkdir(parents=True, exist_ok=False)
        except OSError as exc:
            self._release_lock()
            self._set_state("ERROR")
            self.error.emit(f"Recording output directory failed: {exc}")
            return False
        command = self._command_builder(
            python_executable=os.environ.get("OPERATOR_RECORDER_PYTHON", sys.executable),
            mode=mode,
            output_dir=output_path,
        )
        try:
            process = self._process_factory(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except (OSError, TypeError) as exc:
            self._release_lock()
            self._set_state("ERROR")
            self.error.emit(f"Recording start failed: {exc}")
            return False

        self._process = process
        self._mode = mode
        self._output_path = output_path
        self._started_monotonic = time.monotonic()
        self._stop_stage = None
        self._stop_deadline = None
        self.output_changed.emit(str(output_path))
        self._set_state("STARTING")
        self.event_message.emit(f"recording started: {mode_label(mode)} → {output_path}")
        self._timer.start(self._poll_interval_ms)
        return True

    def stop(self) -> bool:
        if not self.active or self._process is None:
            return False
        if self._state == "STOPPING":
            return True
        self._set_state("STOPPING")
        self._stop_stage = "SIGINT"
        self._stop_deadline = time.monotonic() + self._stop_timeout_s
        _send_group_signal(self._process, signal.SIGINT)
        self.event_message.emit("recording stop requested: SIGINT")
        return True

    def poll(self) -> None:
        process = self._process
        if process is None:
            return
        now = time.monotonic()
        if self._started_monotonic is not None:
            self.elapsed_changed.emit(f"{max(0.0, now - self._started_monotonic):.1f} s")
        return_code = process.poll()
        if return_code is not None:
            self._finish(return_code)
            return
        if self._state == "STARTING":
            self._set_state("RECORDING")
        if self._state != "STOPPING" or self._stop_deadline is None or now < self._stop_deadline:
            return
        if self._stop_stage == "SIGINT":
            self._stop_stage = "SIGTERM"
            self._stop_deadline = now + self._terminate_timeout_s
            _send_group_signal(process, signal.SIGTERM)
            self.event_message.emit("recording stop escalation: SIGTERM")
        elif self._stop_stage == "SIGTERM":
            self._stop_stage = "SIGKILL"
            self._stop_deadline = now + self._terminate_timeout_s
            _send_group_signal(process, signal.SIGKILL)
            self.event_message.emit("recording stop escalation: SIGKILL")

    def _finish(self, return_code) -> None:
        output = self._output_path
        was_stopping = self._state == "STOPPING"
        self._timer.stop()
        self._process = None
        self._release_lock()
        if return_code == 0 and was_stopping:
            self._set_state("COMPLETE")
            self.event_message.emit(f"recording complete: {output or 'output unavailable'}")
        elif return_code == 0 and self._state == "RECORDING":
            self._set_state("COMPLETE")
            self.event_message.emit(f"recording exited cleanly: {output or 'output unavailable'}")
        else:
            self._set_state("ERROR")
            self.error.emit(f"Recorder exited with code {return_code}")
            self.event_message.emit(f"recording error: return code {return_code}")
        self._mode = None
        self._started_monotonic = None
        self._stop_stage = None
        self._stop_deadline = None

    def shutdown(self) -> None:
        if self.active:
            self.stop()
            # Do not wait here: shutdown_plugin must leave the Qt event loop
            # responsive. The polling timer continues while Qt tears down.
        else:
            self._timer.stop()
            self._release_lock()


class RecordingControlWidget(QWidget):
    """Reusable recording bar for OperatorPanel and RecorderPanel."""

    recording_event = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.controller = RecordingSessionController(self)
        self.mode_combo = QComboBox()
        for label, value in MODE_OPTIONS:
            self.mode_combo.addItem(label, value)
        self.mode_combo.setCurrentIndex(1)  # Research/full is the default.
        self.run_name_edit = QLineEdit("run")
        self.save_root_edit = QLineEdit(str(default_record_root()))
        self.browse_button = QPushButton("Browse")
        self.start_button = QPushButton("Start Recording")
        self.stop_button = QPushButton("Stop")
        self.state_label = QLabel("IDLE")
        self.elapsed_label = QLabel("0.0 s")
        self.output_label = QLabel("—")
        self.output_label.setWordWrap(True)
        self.output_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(3)
        row = QHBoxLayout()
        row.addWidget(QLabel("Mode"))
        row.addWidget(self.mode_combo)
        row.addWidget(QLabel("Run name"))
        row.addWidget(self.run_name_edit)
        row.addWidget(QLabel("Save root"))
        row.addWidget(self.save_root_edit, 2)
        row.addWidget(self.browse_button)
        row.addWidget(self.start_button)
        row.addWidget(self.stop_button)
        row.addWidget(QLabel("State"))
        row.addWidget(self.state_label)
        row.addWidget(QLabel("Elapsed"))
        row.addWidget(self.elapsed_label)
        layout.addLayout(row)
        output_row = QHBoxLayout()
        output_row.addWidget(QLabel("Output path"))
        output_row.addWidget(self.output_label, 1)
        layout.addLayout(output_row)

        self.browse_button.clicked.connect(self._browse)
        self.start_button.clicked.connect(self._start)
        self.stop_button.clicked.connect(self.controller.stop)
        self.controller.state_changed.connect(self._state_changed)
        self.controller.elapsed_changed.connect(self.elapsed_label.setText)
        self.controller.output_changed.connect(self.output_label.setText)
        self.controller.event_message.connect(self.recording_event.emit)
        self.controller.error.connect(self._error)
        if os.environ.get("OPERATOR_RECORDING_MANAGED_EXTERNALLY", "").strip() in {"1", "true", "yes"}:
            self.set_external_active()

    def _browse(self):
        directory = QFileDialog.getExistingDirectory(
            self,
            "Select recording save root",
            self.save_root_edit.text().strip() or str(default_record_root()),
        )
        if directory:
            self.save_root_edit.setText(directory)

    def _start(self):
        mode = str(self.mode_combo.currentData() or "full")
        self.controller.start(
            mode=mode,
            run_name=sanitize_run_name(self.run_name_edit.text()),
            save_root=self.save_root_edit.text().strip() or default_record_root(),
        )

    def _state_changed(self, state: str):
        self.state_label.setText(state)
        active = state in {"STARTING", "RECORDING", "STOPPING"}
        for widget in (self.mode_combo, self.run_name_edit, self.save_root_edit, self.browse_button):
            widget.setEnabled(not active)
        self.start_button.setEnabled(state in {"IDLE", "COMPLETE", "ERROR"})
        self.stop_button.setEnabled(state in {"STARTING", "RECORDING", "STOPPING"})

    def _error(self, message: str):
        self.state_label.setText("ERROR")
        self.recording_event.emit(str(message))

    def set_external_active(self):
        self.state_label.setText("EXTERNAL RECORDING ACTIVE")
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.mode_combo.setEnabled(False)
        self.run_name_edit.setEnabled(False)
        self.save_root_edit.setEnabled(False)
        self.browse_button.setEnabled(False)

    def shutdown(self):
        self.controller.shutdown()


__all__ = [
    "MODE_OPTIONS",
    "RECORDING_STATES",
    "mode_value",
    "mode_label",
    "sanitize_run_name",
    "default_record_root",
    "unique_output_dir",
    "build_operator_recorder_command",
    "RecordingSessionController",
    "RecordingControlWidget",
]
