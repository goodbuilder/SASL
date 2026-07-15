"""Shared user-controlled recording lifecycle for the Simulation UI.

The Qt widget in this module is deliberately small.  All process ownership,
mode mapping, output naming, and asynchronous shutdown live in the controller
so the embedded OperatorPanel and the standalone RecorderPanel cannot drift.
"""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import threading
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
from stereo_ground_sim.simulation_ui_contract import RecordingReadiness, RecordingReadinessModel

try:  # ROS is optional when the reusable Qt bar is tested standalone.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32MultiArray, Float64MultiArray, String
except Exception:  # pragma: no cover - exercised only outside a ROS runtime.
    rclpy = None
    qos_profile_sensor_data = None
    Image = Float32MultiArray = Float64MultiArray = String = None


MODE_OPTIONS = (
    ("Data Only", "lite"),
    ("Research", "full"),
    ("Debug/Reprocess", "debug"),
)
MODE_TO_LABEL = {value: label for label, value in MODE_OPTIONS}
RECORDING_STATES = (
    "IDLE",
    "STARTING",
    "RECORDING",
    "STOPPING",
    "COMPLETE",
    "COMPLETE — NON-FORMAL",
    "ERROR",
)
_SAFE_RUN_NAME = re.compile(r"[^A-Za-z0-9._-]+")
_ACTIVE_ROOT_LOCKS: set[str] = set()
_ROOT_LOCK_MUTEX = threading.RLock()


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


def _wait_and_reap(process, timeout_s: float) -> bool:
    """Bounded wait that proves both exit and ``wait()`` completion."""
    if process is None:
        return True
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while True:
        if process.poll() is not None:
            try:
                process.wait(timeout=0)
            except AttributeError:
                # A poll-only object cannot prove that the child was reaped.
                return False
            except (OSError, subprocess.TimeoutExpired, TimeoutError):
                return False
            return process.poll() is not None
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        try:
            process.wait(timeout=min(0.05, remaining))
        except AttributeError:
            return False
        except (subprocess.TimeoutExpired, TimeoutError):
            time.sleep(min(0.02, remaining))
        except OSError:
            return False


class RecordingReadinessSubscriber(QObject):
    """ROS-to-model adapter used by the standalone and embedded recording bars.

    ROS callbacks only update the thread-independent model and emit a Qt
    signal. They never touch widgets, so teardown cannot make a callback
    update a destroyed QObject-owned view.
    """

    readiness_changed = Signal(object)
    error = Signal(str)

    def __init__(self, parent=None, *, model=None, node=None):
        super().__init__(parent)
        self.model = model or RecordingReadinessModel()
        self._node = node
        self._subscriptions = []
        self._spin_thread = None
        self._stop_event = threading.Event()
        self._started = False

    def _publish(self):
        self.readiness_changed.emit(self.model.readiness())

    def _ros_receive_time_ns(self):
        if self._node is None:
            return None
        try:
            return int(self._node.get_clock().now().nanoseconds)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None

    def update(self, key, value, *, receive_monotonic_ns=None):
        self.model.update(
            key,
            value,
            receive_ros_time_ns=self._ros_receive_time_ns(),
            receive_monotonic_ns=receive_monotonic_ns,
        )
        self._publish()

    def set_mode(self, mode, observation_mode=None):
        self.model.set_mode(mode, observation_mode)
        self._publish()

    def set_parameter_snapshot_status(self, status):
        self.model.set_parameter_snapshot_status(status)
        self._publish()

    @staticmethod
    def _json_payload(message):
        try:
            value = json.loads(message)
        except (TypeError, ValueError, json.JSONDecodeError):
            return message
        return value

    def _position_callback(self, key, message):
        self.update(key, list(getattr(message, "data", ())))

    def _status_callback(self, key, message):
        self.update(key, self._json_payload(getattr(message, "data", "")))

    def _image_callback(self, key, _message):
        self.update(key, {"received": True})

    def start(self):
        if self._started:
            return True
        self._started = True
        self._stop_event.clear()
        if self._node is None and rclpy is None:
            self._publish()
            return False
        try:
            if self._node is None:
                if not rclpy.ok():
                    rclpy.init(args=None)
                self._node = rclpy.create_node(f"recording_readiness_{id(self)}")
            position_type = Float64MultiArray
            for key, topic in (
                ("ground_truth", "/ground_truth_position"),
                ("observed_raw", "/stereo/estimated_position"),
                ("analytic_raw", "/stereo/attitude_model/raw/estimated_position"),
                ("corrected", "/stereo/corrected_position"),
            ):
                message_type = position_type if key == "analytic_raw" else Float32MultiArray
                self._subscriptions.append(
                    self._node.create_subscription(
                        message_type,
                        topic,
                        lambda message, name=key: self._position_callback(name, message),
                        qos_profile_sensor_data,
                    )
                )
            for key, topic in (
                ("attitude_status", "/stereo/attitude_model/status"),
                ("observation_status", "/stereo/observation_status"),
                ("profile_status", "/stereo/profile_status"),
                ("px4_status", "/simulation/px4/status"),
            ):
                self._subscriptions.append(
                    self._node.create_subscription(
                        String,
                        topic,
                        lambda message, name=key: self._status_callback(name, message),
                        qos_profile_sensor_data,
                    )
                )
            for key, topic in (
                ("left_image", "/left_camera/detection_image"),
                ("right_image", "/right_camera/detection_image"),
            ):
                self._subscriptions.append(
                    self._node.create_subscription(
                        Image,
                        topic,
                        lambda message, name=key: self._image_callback(name, message),
                        qos_profile_sensor_data,
                    )
                )
            if self._node is not None and rclpy is not None and self._spin_thread is None:
                self._spin_thread = threading.Thread(
                    target=self._spin, name="recording-readiness", daemon=True
                )
                self._spin_thread.start()
            self._publish()
            return True
        except Exception as exc:  # keep panel usable when a ROS graph is partial.
            self.error.emit(f"recording readiness subscriber failed: {exc}")
            self._publish()
            return False

    def _spin(self):
        while not self._stop_event.is_set() and self._node is not None and rclpy is not None:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.error.emit(f"recording readiness spin failed: {exc}")
                return

    def shutdown(self):
        self._stop_event.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        self._node = None
        self._subscriptions = []

    stop = shutdown


class RecordingSessionController(QObject):
    """Own one operator_recorder process without blocking the Qt thread."""

    state_changed = Signal(str)
    elapsed_changed = Signal(str)
    output_changed = Signal(str)
    event_message = Signal(str)
    error = Signal(str)
    shutdown_result = Signal(object)

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
        self._lock_error = None
        self._state_lock = threading.RLock()
        self._shutdown_thread = None
        self._shutdown_complete = threading.Event()
        self._shutdown_complete.set()
        self._readiness = None
        self._shutdown_fatal_error = None
        self._last_finish_error = None
        self._last_finish_event = None
        self.shutdown_result.connect(self._on_shutdown_result)

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

    @property
    def shutdown_complete(self) -> bool:
        return self._shutdown_complete.is_set()

    @property
    def shutdown_fatal_error(self):
        return self._shutdown_fatal_error

    def set_readiness(self, readiness) -> None:
        """Store the immutable readiness snapshot used at process start."""
        self._readiness = readiness

    def _set_state(self, state: str) -> None:
        self._state = str(state)
        self.state_changed.emit(self._state)

    def _release_lock(self) -> None:
        with _ROOT_LOCK_MUTEX:
            lock_key = self._lock_key
            lock_file = self._lock_file
            self._lock_key = None
            self._lock_file = None
            if lock_key is not None:
                _ACTIVE_ROOT_LOCKS.discard(lock_key)
        if lock_file is not None:
            try:
                import fcntl

                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
            except (ImportError, OSError):
                pass
            try:
                lock_file.close()
            except OSError:
                pass

    def _acquire_lock(self, root: Path) -> bool:
        try:
            key = str(root.resolve())
        except OSError as exc:
            self._lock_error = f"Recording root unavailable: {exc}"
            return False
        with _ROOT_LOCK_MUTEX:
            if key in _ACTIVE_ROOT_LOCKS:
                return False
            try:
                root.mkdir(parents=True, exist_ok=True)
            except OSError as exc:
                self._lock_error = f"Recording root unavailable: {exc}"
                return False
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
                    self._lock_error = f"Recording root lock unavailable: {root}"
                    return False
            except OSError:
                self._lock_error = f"Recording root lock unavailable: {root}"
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
        self._lock_error = None
        if not self._acquire_lock(root):
            self._set_state("ERROR")
            self.error.emit(self._lock_error or f"Recording root is already locked: {root}")
            return False

        output_path = unique_output_dir(root, run_name, mode)
        try:
            output_path.mkdir(parents=True, exist_ok=False)
        except OSError as exc:
            self._release_lock()
            self._set_state("ERROR")
            self.error.emit(f"Recording output directory failed: {exc}")
            return False
        if self._readiness is not None:
            try:
                (output_path / "readiness_at_start.json").write_text(
                    json.dumps(self._readiness.as_dict(), indent=2, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
            except OSError as exc:
                self.event_message.emit(f"readiness snapshot write failed: {exc}")
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
            if _wait_and_reap(process, 0.0):
                self._finish(return_code, process=process, reaped=True)
            else:
                self._fatal_cleanup_error("Recorder exited but process.wait() did not complete")
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

        elif self._stop_stage == "SIGKILL":
            # A killed child must still be reaped before the output root lock
            # is released. Keep this final check bounded for the Qt timer.
            if _wait_and_reap(process, 0.2):
                self._finish(process.poll() if process.poll() is not None else -signal.SIGKILL, process=process, reaped=True)
            else:
                self._fatal_cleanup_error("Recorder remained alive after SIGKILL; root lock retained")

    def report_eligibility_violation(self, reasons):
        """Persist a mid-run readiness violation for recorder finalization."""
        output = self._output_path
        if output is None:
            return
        if isinstance(reasons, str):
            reasons = [reasons]
        event = {
            "time_monotonic_s": time.monotonic(),
            "type": "eligibility_violation",
            "reasons": [str(reason) for reason in (reasons or [])],
        }
        try:
            with (output / "eligibility_events.jsonl").open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(event, sort_keys=True) + "\n")
        except OSError as exc:
            self.event_message.emit(f"eligibility event write failed: {exc}")
        self.event_message.emit("formal eligibility violation: " + "; ".join(event["reasons"]))

    def _manifest_result(self, output):
        if output is None:
            return None
        path = Path(output) / "manifest.json"
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError):
            return None
        return payload if isinstance(payload, dict) else None

    def _fatal_cleanup_error(self, message, *, notify=True):
        """Retain the process and root lock when cleanup cannot be proven."""
        self._shutdown_fatal_error = self._shutdown_fatal_error or str(message)
        self._state = "ERROR"
        if notify:
            self.state_changed.emit("ERROR")
            self.error.emit(self._shutdown_fatal_error)
            self.event_message.emit(f"recording cleanup error: {self._shutdown_fatal_error}")

    def _finish(self, return_code, *, process=None, reaped=False, notify=True) -> bool:
        if process is None:
            process = self._process
        if process is None:
            return False
        # ``reaped`` is only an optimization hint from the bounded supervisor;
        # verify the invariant again at the lock-release boundary so callers
        # cannot bypass poll()+wait() proof accidentally.
        if not _wait_and_reap(process, 0.0):
            self._fatal_cleanup_error("Recorder process exited but could not be reaped", notify=notify)
            return False
        with self._state_lock:
            if self._process is not process:
                return False
            output = self._output_path
            previous_state = self._state
            was_stopping = previous_state == "STOPPING"
            self._process = None
        if notify and threading.current_thread() is threading.main_thread():
            self._timer.stop()
        # The child has already been waited for above. Lock release is strictly
        # after that proof, including when this function runs in the supervisor.
        self._release_lock()
        manifest = self._manifest_result(output)
        artifact_ok = manifest is not None and manifest.get("artifact_integrity") is True
        recording_state = manifest.get("recording_state") if manifest else None
        formal = manifest is not None and manifest.get("formal_experiment_eligible") is True
        finish_error = None
        finish_event = None
        if return_code == 0 and artifact_ok and recording_state == "complete" and was_stopping:
            final_state = "COMPLETE" if formal else "COMPLETE — NON-FORMAL"
            finish_event = f"recording {'complete' if formal else 'complete — non-formal'}: {output or 'output unavailable'}"
        elif return_code == 0 and artifact_ok and recording_state == "complete" and previous_state == "RECORDING":
            final_state = "COMPLETE" if formal else "COMPLETE — NON-FORMAL"
            finish_event = f"recording {'exited cleanly' if formal else 'exited non-formal'}: {output or 'output unavailable'}"
        else:
            final_state = "ERROR"
            if return_code != 0:
                finish_error = f"Recorder exited with code {return_code}"
            elif manifest is None:
                finish_error = "Recorder exited without a readable manifest"
            else:
                finish_error = "Recorder artifact integrity failed"
            finish_event = f"recording error: {finish_error}"

        self._state = final_state
        self._last_finish_error = finish_error
        self._last_finish_event = finish_event
        self._mode = None
        self._started_monotonic = None
        self._stop_stage = None
        self._stop_deadline = None
        if notify:
            self.state_changed.emit(final_state)
            if finish_error:
                self.error.emit(finish_error)
            if finish_event:
                self.event_message.emit(finish_event)
        return True

    def _on_shutdown_result(self, result):
        """Present supervisor results on the QObject/Qt thread only."""
        if not isinstance(result, dict):
            return
        if not result.get("reaped"):
            message = self._shutdown_fatal_error or "Recorder cleanup did not prove process reaping"
            self._fatal_cleanup_error(message, notify=True)
            return
        for message in result.get("messages", ()):
            self.event_message.emit(message)
        self.state_changed.emit(self._state)
        if self._last_finish_error:
            self.error.emit(self._last_finish_error)
        if self._last_finish_event:
            self.event_message.emit(self._last_finish_event)

    def shutdown(self) -> None:
        process = self._process
        if process is None:
            self._timer.stop()
            self._release_lock()
            self._shutdown_complete.set()
            return
        if self._shutdown_thread is not None and self._shutdown_thread.is_alive():
            return
        self._shutdown_complete.clear()
        self._shutdown_fatal_error = None
        self._timer.stop()
        self._set_state("STOPPING")
        self._stop_stage = "SIGINT"
        self.event_message.emit("recording shutdown: SIGINT")

        def cleanup():
            messages = []
            try:
                reaped = False
                for stage, sig, timeout in (
                    ("SIGINT", signal.SIGINT, self._stop_timeout_s),
                    ("SIGTERM", signal.SIGTERM, self._terminate_timeout_s),
                    ("SIGKILL", signal.SIGKILL, self._terminate_timeout_s),
                ):
                    if process.poll() is not None:
                        reaped = _wait_and_reap(process, 0.0)
                        break
                    if stage != "SIGINT":
                        messages.append(f"recording shutdown escalation: {stage}")
                    _send_group_signal(process, sig)
                    if _wait_and_reap(process, timeout):
                        reaped = True
                        break
                if not reaped and process.poll() is not None:
                    reaped = _wait_and_reap(process, 0.0)
                if not reaped:
                    self._fatal_cleanup_error(
                        "Recorder remained alive after SIGKILL; process.wait() incomplete and root lock retained",
                        notify=False,
                    )
                    result = {"reaped": False, "messages": messages}
                else:
                    return_code = process.poll()
                    if return_code is None:
                        self._fatal_cleanup_error(
                            "Recorder cleanup reported reaped without an exit code; root lock retained",
                            notify=False,
                        )
                        result = {"reaped": False, "messages": messages}
                    elif self._finish(return_code, process=process, reaped=True, notify=False):
                        self._shutdown_complete.set()
                        result = {"reaped": True, "messages": messages}
                    else:
                        result = {"reaped": False, "messages": messages}
            except Exception as exc:  # preserve lock/process ownership on supervisor failure.
                self._fatal_cleanup_error(f"Fatal recorder cleanup error: {exc}", notify=False)
                result = {"reaped": False, "messages": messages}
            self.shutdown_result.emit(result)

        self._shutdown_thread = threading.Thread(target=cleanup, name="recording-shutdown", daemon=False)
        self._shutdown_thread.start()

    def wait_for_shutdown(self, timeout_s=1.0) -> bool:
        """Bound plugin teardown without letting a worker touch Qt widgets."""
        thread = self._shutdown_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, float(timeout_s)))
        return bool(self.shutdown_complete and (thread is None or not thread.is_alive()))


class RecordingControlWidget(QWidget):
    """Reusable recording bar for OperatorPanel and RecorderPanel."""

    recording_event = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.controller = RecordingSessionController(self)
        self._readiness = None
        self._readiness_model = None
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
        self.readiness_label = QLabel("Readiness: BLOCKED — unavailable")
        self.readiness_label.setWordWrap(True)
        self.readiness_label.setToolTip("Recording readiness has not been provided")
        self.elapsed_label = QLabel("0.0 s")
        self.output_label = QLabel("—")
        self.output_label.setWordWrap(True)
        self.output_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.start_button.setEnabled(False)
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
        row.addWidget(self.readiness_label, 2)
        row.addWidget(QLabel("Elapsed"))
        row.addWidget(self.elapsed_label)
        layout.addLayout(row)
        output_row = QHBoxLayout()
        output_row.addWidget(QLabel("Output path"))
        output_row.addWidget(self.output_label, 1)
        layout.addLayout(output_row)

        self.browse_button.clicked.connect(self._browse)
        self.start_button.clicked.connect(self._start)
        self.mode_combo.currentIndexChanged.connect(self._mode_changed)
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
        if self._readiness is not None and not self._mode_start_allowed(mode):
            self.recording_event.emit("recording start blocked: " + "; ".join(self._readiness.reasons))
            return False
        return self.controller.start(
            mode=mode,
            run_name=sanitize_run_name(self.run_name_edit.text()),
            save_root=self.save_root_edit.text().strip() or default_record_root(),
        )

    def _state_changed(self, state: str):
        self.state_label.setText(state)
        active = state in {"STARTING", "RECORDING", "STOPPING"}
        for widget in (self.mode_combo, self.run_name_edit, self.save_root_edit, self.browse_button):
            widget.setEnabled(not active)
        self._refresh_start_enabled()
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

    def _mode_start_allowed(self, mode=None):
        mode = str(mode or self.mode_combo.currentData() or "full")
        if mode == "debug":
            return True
        return self._readiness is not None and bool(self._readiness.start_allowed)

    def _mode_changed(self, *_args):
        """Never reuse a readiness result computed for a different mode."""
        mode = str(self.mode_combo.currentData() or "full")
        if self._readiness_model is not None:
            self._readiness_model.set_mode(mode)
        if mode == "debug":
            self._readiness = RecordingReadiness(True, False, ("DEBUG — NON-FORMAL",))
            self.controller.set_readiness(self._readiness)
            self.readiness_label.setText("Readiness: DEBUG — NON-FORMAL")
            self.readiness_label.setToolTip("Debug/Reprocess is always non-formal")
        else:
            self._readiness = None
            self.controller.set_readiness(None)
            self.readiness_label.setText("Readiness: BLOCKED — unavailable")
            self.readiness_label.setToolTip("Recording readiness has not been provided for this mode")
        self._refresh_start_enabled()

    def _refresh_start_enabled(self, *_args):
        state_allowed = self.controller.state in {"IDLE", "COMPLETE", "COMPLETE — NON-FORMAL", "ERROR"}
        self.start_button.setEnabled(state_allowed and self._mode_start_allowed())

    def set_readiness(self, readiness):
        """Apply the pure readiness result and expose reasons in the UI."""
        previous = self._readiness
        if (
            readiness is not None
            and readiness.formal_eligible
            and self._readiness_model is not None
            and str(self._readiness_model.parameter_snapshot_status).strip().lower() == "pending"
        ):
            reasons = tuple(dict.fromkeys((*readiness.reasons, "PARAMETER SNAPSHOT PENDING")))
            readiness = RecordingReadiness(readiness.start_allowed, False, reasons)
        self._readiness = readiness
        self.controller.set_readiness(readiness)
        if readiness is None:
            self.readiness_label.setText("Readiness: BLOCKED — unavailable")
            self.readiness_label.setToolTip("Recording readiness has not been provided")
        elif readiness.start_allowed:
            if "PARAMETER SNAPSHOT PENDING" in readiness.reasons:
                text = "READY — PARAMETER SNAPSHOT PENDING"
            elif not readiness.formal_eligible and "DEBUG — NON-FORMAL" in readiness.reasons:
                text = "Readiness: DEBUG — NON-FORMAL"
            elif not readiness.formal_eligible:
                text = "Readiness: READY — NON-FORMAL"
            else:
                text = "Readiness: FORMAL READY"
            self.readiness_label.setText(text)
            self.readiness_label.setToolTip("\n".join(readiness.reasons))
        else:
            self.readiness_label.setText("Readiness: BLOCKED — " + "; ".join(readiness.reasons))
            self.readiness_label.setToolTip("\n".join(readiness.reasons))
            if (
                previous is not None
                and previous.formal_eligible
                and not readiness.formal_eligible
                and self.controller.active
            ):
                self.controller.report_eligibility_violation(readiness.reasons)
        self._refresh_start_enabled()

    def set_readiness_model(self, model):
        """Bind mode changes to the shared pure readiness model."""
        self._readiness_model = model
        if model is not None:
            model.set_mode(str(self.mode_combo.currentData() or "full"))

    def shutdown(self):
        self.controller.shutdown()

    def wait_for_shutdown(self, timeout_s=1.0):
        return self.controller.wait_for_shutdown(timeout_s)


__all__ = [
    "MODE_OPTIONS",
    "RECORDING_STATES",
    "mode_value",
    "mode_label",
    "sanitize_run_name",
    "default_record_root",
    "unique_output_dir",
    "build_operator_recorder_command",
    "RecordingReadinessSubscriber",
    "RecordingSessionController",
    "RecordingControlWidget",
]
