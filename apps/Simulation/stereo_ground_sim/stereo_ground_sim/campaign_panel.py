"""Standalone RQT controls for the human-operated v1.1 campaign."""

from __future__ import annotations

import json
import os
import signal
from pathlib import Path

from python_qt_binding.QtCore import QProcess, QProcessEnvironment, QTimer, Qt
from python_qt_binding.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from qt_gui.plugin import Plugin

from campaigns.stereo_v1_1.control_config import DEFAULT_CONFIG, SIM_ROOT, load_control_config


SWEEP_RUNNER = SIM_ROOT / "run_auto_attitude_sweep.sh"
PREPARE_RUNTIME = SIM_ROOT / "scripts" / "prepare_stereo_campaign_runtime.sh"
MAX_LOG_BLOCKS = 2000


class CampaignPanel(Plugin):
    """Read Simulation.env on demand, then run the selected campaign action."""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("StereoCampaignPanel")
        self._config_path = Path(os.environ.get("SIM_CONFIG", str(DEFAULT_CONFIG))).expanduser().resolve()
        self._config = None
        self._output_root = None
        self._running_label = "configured action"
        self._process = QProcess(self)
        self._process.setWorkingDirectory(str(SIM_ROOT))
        self._process.setProcessEnvironment(QProcessEnvironment.systemEnvironment())
        self._process.setProcessChannelMode(QProcess.MergedChannels)
        self._process.readyReadStandardOutput.connect(self._read_process_output)
        self._process.finished.connect(self._process_finished)

        self._widget = QWidget()
        self._widget.setWindowTitle("Stereo v1.1 Campaign")
        root = QVBoxLayout(self._widget)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        root.addWidget(QLabel("Human-operated experiment control. The panel reads the literal campaign block from Simulation.env when refreshed."))
        details = QGridLayout()
        self._action_label = QLabel("—")
        self._run_label = QLabel("—")
        self._output_label = QLabel("—")
        self._input_label = QLabel("—")
        self._output_base_label = QLabel("—")
        self._mc_label = QLabel("—")
        self._px4_label = QLabel("—")
        self._yolo_label = QLabel("—")
        self._yolo_device_label = QLabel("—")
        self._yolo_backend_label = QLabel("—")
        self._config_hash_label = QLabel("—")
        self._workbook_label = QLabel("—")
        for label in (self._action_label, self._run_label, self._output_label, self._input_label,
                      self._output_base_label,
                      self._mc_label, self._px4_label, self._yolo_label,
                      self._yolo_device_label, self._yolo_backend_label, self._config_hash_label,
                      self._workbook_label):
            label.setTextInteractionFlags(label.textInteractionFlags() | Qt.TextSelectableByMouse)
            label.setStyleSheet("font-family: monospace;")
            label.setWordWrap(True)
        details.addWidget(QLabel("Action"), 0, 0)
        details.addWidget(self._action_label, 0, 1)
        self._run_field_label = QLabel("Run ID")
        details.addWidget(self._run_field_label, 1, 0)
        details.addWidget(self._run_label, 1, 1)
        details.addWidget(QLabel("Output"), 2, 0)
        details.addWidget(self._output_label, 2, 1)
        details.addWidget(QLabel("Input root"), 3, 0)
        details.addWidget(self._input_label, 3, 1)
        details.addWidget(QLabel("Output base"), 4, 0)
        details.addWidget(self._output_base_label, 4, 1)
        details.addWidget(QLabel("Monte Carlo trials"), 5, 0)
        details.addWidget(self._mc_label, 5, 1)
        details.addWidget(QLabel("PX4 source"), 6, 0)
        details.addWidget(self._px4_label, 6, 1)
        details.addWidget(QLabel("YOLO Python"), 7, 0)
        details.addWidget(self._yolo_label, 7, 1)
        details.addWidget(QLabel("YOLO device"), 8, 0)
        details.addWidget(self._yolo_device_label, 8, 1)
        details.addWidget(QLabel("YOLO backend build"), 9, 0)
        details.addWidget(self._yolo_backend_label, 9, 1)
        details.addWidget(QLabel("Simulation.env SHA-256"), 10, 0)
        details.addWidget(self._config_hash_label, 10, 1)
        details.addWidget(QLabel("Run parameters XLSX"), 11, 0)
        details.addWidget(self._workbook_label, 11, 1)
        root.addLayout(details)

        self._config_path_label = QLabel(str(self._config_path))
        self._config_path_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._config_path_label.setStyleSheet("font-family: monospace; color: #455a64;")
        self._config_path_label.setWordWrap(True)
        root.addWidget(self._config_path_label)

        buttons = QHBoxLayout()
        self._refresh_button = QPushButton("Refresh Simulation.env")
        self._prepare_button = QPushButton("Prepare pinned YOLO runtime")
        self._start_button = QPushButton("Run configured action")
        self._stop_button = QPushButton("Stop safely")
        self._stop_button.setEnabled(False)
        buttons.addWidget(self._refresh_button)
        buttons.addWidget(self._prepare_button)
        buttons.addWidget(self._start_button)
        buttons.addWidget(self._stop_button)
        root.addLayout(buttons)

        self._status = QLabel("Refresh the config, then run the configured action.")
        self._status.setStyleSheet("font-weight: 600; color: #37474f;")
        self._status.setWordWrap(True)
        root.addWidget(self._status)

        self._progress = QLabel("Campaign status: not initialized")
        self._progress.setStyleSheet("font-family: monospace; color: #455a64;")
        self._progress.setWordWrap(True)
        root.addWidget(self._progress)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(MAX_LOG_BLOCKS)
        root.addWidget(self._log, 1)
        root.addWidget(QLabel(
            "The campaign starts its own headless SITL stack. Close any existing Gazebo/PX4 operator session first. "
            "Workbook actions read numeric rows in order and stop at the first failed run. "
            "The legacy sweep remains a separate Simulation.env action."
        ))

        self._refresh_button.clicked.connect(self.refresh_config)
        self._prepare_button.clicked.connect(self.prepare_runtime)
        self._start_button.clicked.connect(self.start_configured_action)
        self._stop_button.clicked.connect(self.stop_safely)
        self._timer = QTimer(self._widget)
        self._timer.timeout.connect(self.refresh_status)
        self._timer.start(1000)

        context.add_widget(self._widget)
        self.refresh_config()

    def refresh_config(self):
        try:
            config = load_control_config(self._config_path)
            output_root = config.output_root
        except (OSError, ValueError) as exc:
            self._config = None
            self._output_root = None
            self._action_label.setText("INVALID")
            self._status.setText(f"Simulation.env cannot be used: {exc}")
            self._status.setStyleSheet("font-weight: 600; color: #b00020;")
            self._start_button.setEnabled(False)
            return
        self._config = config
        self._output_root = output_root
        self._action_label.setText(config.action)
        workbook_action = config.action in {"xlsx_plan", "xlsx_all", "xlsx_resume"}
        self._run_field_label.setText("Workbook" if workbook_action else "Run ID")
        self._run_label.setText(
            f"{config.workbook_path.name} ({config.workbook_sha256[:12]})"
            if workbook_action and config.workbook_sha256 else config.run_id
        )
        self._workbook_label.setText(
            str(config.workbook_path.relative_to(SIM_ROOT))
            + (f"  sha256={config.workbook_sha256}" if config.workbook_sha256 else "")
        )
        self._output_label.setText(
            str(output_root.relative_to(SIM_ROOT)) if output_root.is_relative_to(SIM_ROOT)
            else f"SASL_WORKSTATION_DATA_ROOT/{config.output_id}"
        )
        if workbook_action:
            self._input_label.setText("the workbook is the per-run input")
        elif config.action == "reprocess":
            self._input_label.setText(str(config.input_root))
        elif config.input_root_setting == "output":
            self._input_label.setText("same as output root")
        elif config.input_root_setting == "repo":
            self._input_label.setText("SASL repository root")
        elif config.input_root_setting == "data_root":
            self._input_label.setText("SASL_WORKSTATION_DATA_ROOT")
        else:
            self._input_label.setText(config.input_root_setting)
        self._output_base_label.setText(config.output_base)
        self._mc_label.setText(str(config.mc_trials))
        self._px4_label.setText(str(config.px4_dir_path.relative_to(SIM_ROOT.parents[1])))
        self._yolo_label.setText(str(config.yolo_python_path.relative_to(SIM_ROOT)))
        self._yolo_device_label.setText(config.yolo_device)
        self._yolo_backend_label.setText(config.yolo_torch_backend)
        self._config_hash_label.setText(config.config_sha256)
        active = self._process.state() != QProcess.NotRunning
        self._start_button.setEnabled(not active)
        self._refresh_button.setEnabled(not active)
        self._prepare_button.setEnabled(not active)
        if not active:
            self._status.setText(
                "Workbook plan loaded and validated." if workbook_action else
                "Config loaded. The scientific protocol remains pinned to the v1.1 manifest."
            )
            self._status.setStyleSheet("font-weight: 600; color: #1b5e20;")
        self.refresh_status()

    def start_configured_action(self):
        if self._process.state() != QProcess.NotRunning:
            return
        self.refresh_config()
        if self._config is None:
            return
        action = self._config.action
        if action in {"development", "run", "all", "resume", "reprocess", "all_results", "legacy_sweep",
                      "xlsx_all", "xlsx_resume"}:
            if action == "all_results":
                scope = "39 sequential SITL runs plus offline analysis"
            elif action == "xlsx_all":
                scope = "all XLSX-defined SITL runs in sequence"
            else:
                scope = action
            source_note = (f"\nInput: {self._config.input_root}" if action == "reprocess" else
                           f"\nWorkbook: {self._config.workbook_path.name}"
                           if action in {"xlsx_all", "xlsx_resume"} else "")
            answer = QMessageBox.question(
                self._widget,
                "Start configured simulation action",
                f"Start '{scope}' using output {self._config.output_id}?{source_note}\n\n"
                "The runner uses SITL only, preserves every attempt, and stops on the first failed slot.",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return
        self._log.clear()
        self._running_label = action
        self._status.setText(f"Starting {action}…")
        self._status.setStyleSheet("font-weight: 600; color: #0d47a1;")
        self._process.setProgram("bash")
        self._process.setArguments([
            str(SWEEP_RUNNER), "--stereo-v1-1", "--config", str(self._config_path),
        ])
        self._process.start()
        if not self._process.waitForStarted(5000):
            self._status.setText(f"Campaign controller did not start: {self._process.errorString()}")
            self._status.setStyleSheet("font-weight: 600; color: #b00020;")
            return
        self._start_button.setEnabled(False)
        self._refresh_button.setEnabled(False)
        self._prepare_button.setEnabled(False)
        self._stop_button.setEnabled(True)

    def prepare_runtime(self):
        if self._process.state() != QProcess.NotRunning:
            return
        answer = QMessageBox.question(
            self._widget,
            "Prepare the pinned YOLO runtime",
            "This reads the current YOLO version settings from Simulation.env and may download the pinned PyTorch, Ultralytics, SciPy, and zstd packages into the ignored apps/Simulation/.venv directory. Continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self._log.clear()
        self._running_label = "pinned YOLO runtime preparation"
        self._status.setText("Preparing the isolated runtime from Simulation.env…")
        self._status.setStyleSheet("font-weight: 600; color: #0d47a1;")
        self._process.setProgram("bash")
        self._process.setArguments([str(PREPARE_RUNTIME)])
        environment = QProcessEnvironment.systemEnvironment()
        environment.insert("SIM_CONFIG", str(self._config_path))
        self._process.setProcessEnvironment(environment)
        self._process.start()
        if not self._process.waitForStarted(5000):
            self._status.setText(f"Runtime preparation did not start: {self._process.errorString()}")
            self._status.setStyleSheet("font-weight: 600; color: #b00020;")
            return
        self._start_button.setEnabled(False)
        self._refresh_button.setEnabled(False)
        self._prepare_button.setEnabled(False)
        self._stop_button.setEnabled(True)

    def stop_safely(self):
        if self._process.state() == QProcess.NotRunning:
            return
        pid = int(self._process.processId())
        try:
            os.kill(pid, signal.SIGINT)
        except OSError as exc:
            self._append_log(f"Stop signal failed: {exc}")
            return
        self._status.setText("Stop requested. The current attempt is being finalized and preserved.")
        self._status.setStyleSheet("font-weight: 600; color: #e65100;")
        self._stop_button.setEnabled(False)

    def _read_process_output(self):
        data = bytes(self._process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if data:
            for line in data.splitlines():
                self._append_log(line)
        self.refresh_status()

    def _append_log(self, line: str):
        self._log.appendPlainText(str(line))
        scrollbar = self._log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _process_finished(self, exit_code: int, _exit_status):
        self._read_process_output()
        self._stop_button.setEnabled(False)
        self._start_button.setEnabled(self._config is not None)
        self._refresh_button.setEnabled(True)
        self._prepare_button.setEnabled(True)
        if exit_code == 0:
            self._status.setText(f"{self._running_label} completed. Review the run status and output artifacts below.")
            self._status.setStyleSheet("font-weight: 600; color: #1b5e20;")
        else:
            self._status.setText(f"Configured action exited with status {exit_code}; inspect the preserved attempt and log.")
            self._status.setStyleSheet("font-weight: 600; color: #b00020;")
        self.refresh_status()

    def refresh_status(self):
        if self._config is None:
            return
        if self._output_root is None:
            return
        if self._config.action == "xlsx_plan":
            self._progress.setText("Workbook plan: validated; no flight status was initialized.")
            return
        status_root = self._output_root
        status_label = ("Workbook campaign status"
                        if self._config.action in {"xlsx_all", "xlsx_resume"}
                        else "Campaign status")
        if self._config.action == "reprocess":
            try:
                status_root = self._config.input_root
            except ValueError:
                return
            status_label = "Source campaign status"
        status_path = status_root / "run_status.json"
        if not status_path.is_file():
            self._progress.setText(f"{status_label}: not initialized")
            return
        try:
            status = json.loads(status_path.read_text(encoding="utf-8"))
            if (self._config.action in {"xlsx_all", "xlsx_resume"}
                    and status.get("workbook_sha256") != self._config.workbook_sha256):
                self._progress.setText(
                    f"{status_label}: output ID is tied to a different or non-workbook campaign."
                )
                return
            slots = status.get("run_slots", [])
            counts = {}
            for row in slots:
                name = str(row.get("status", "UNKNOWN"))
                counts[name] = counts.get(name, 0) + 1
            running = next((row for row in slots if row.get("status") == "RUNNING"), None)
            active = ""
            if running:
                active = f"; active {running['run_id']}/{running.get('attempts', [{}])[-1].get('attempt_id', '?')}"
            valid = counts.get("VALID", 0) + counts.get("VALID_WITH_WARNINGS", 0)
            failures = counts.get("INFRASTRUCTURE_FAILED", 0) + counts.get("PROTOCOL_DEVIATION", 0)
            planned = int(status.get("planned_run_count", len(slots)))
            self._progress.setText(
                f"{status_label}: {planned} slots: valid={valid}, failed={failures}, "
                f"running={counts.get('RUNNING', 0)}, "
                f"planned={counts.get('PLANNED', 0)}{active}"
            )
            self._status.setToolTip(", ".join(f"{key}={value}" for key, value in sorted(counts.items())) + active)
        except (OSError, ValueError, KeyError, TypeError):
            # The status writer uses atomic replacement. A transient read during
            # replacement is harmless; a later timer tick will read it again.
            return

    def shutdown(self):
        self._timer.stop()
        if self._process.state() != QProcess.NotRunning:
            self.stop_safely()

