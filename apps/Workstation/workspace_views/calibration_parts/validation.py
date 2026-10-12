"""CalibrationValidationPanel owns its widgets and transient presentation state.

Domain formulas, native raster/timing admission and calibration solves remain with
the existing backend and core contracts. No workspace host is accepted.
"""

from __future__ import annotations

from collections.abc import Mapping
from PyQt5 import QtCore, QtWidgets


class CalibrationValidationPanel(QtCore.QObject):
    def __init__(self, commands, *, read_actions, present_quality, warning_confirmed, parent=None):
        super().__init__(parent)
        if not callable(read_actions):
            raise TypeError("Calibration validation requires an action reader")
        self._read_actions = read_actions
        self.commands = commands
        self._present_quality = present_quality
        self._warning_confirmed = warning_confirmed
        self._dialog_parent = parent

    def stop(self):
        self._checkerboard_validation_poll_timer.stop()

    def _build_checkerboard_validation_group(self) -> QtWidgets.QGroupBox:
        """Recompute K,D (and R,t) from random subsets of the scanned pairs
        and report how much the result varies, as a check on calibration
        stability -- distinct from the main Capture/Compute/Load & Apply
        flow, which only ever produces or loads one "official" result.

        This talks to the owner only through the payload-in/result-out
        ``*_workstation_checkerboard_validation_*`` facade methods; it does
        not reach into the legacy panel's own hidden validation widgets.
        """

        group = QtWidgets.QGroupBox("Checkerboard random validation")
        grid = QtWidgets.QGridLayout(group)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(5)
        grid.setColumnStretch(1, 1)
        row = 0

        self.checkerboard_validation_root_edit = QtWidgets.QLineEdit()
        self.checkerboard_validation_root_edit.setToolTip(
            "Directory containing left/right checkerboard capture images to validate against."
        )
        browse_root_button = QtWidgets.QPushButton("Browse…")
        browse_root_button.clicked.connect(self._choose_checkerboard_validation_root)
        grid.addWidget(QtWidgets.QLabel("Image root"), row, 0)
        grid.addWidget(self.checkerboard_validation_root_edit, row, 1)
        grid.addWidget(browse_root_button, row, 2)
        row += 1

        self.checkerboard_validation_mode_combo = QtWidgets.QComboBox()
        self.checkerboard_validation_mode_combo.addItem("Pixhawk (K,D only)", "pixhawk")
        self.checkerboard_validation_mode_combo.addItem("Checkerboard (K,D + R,t)", "checkerboard")
        grid.addWidget(QtWidgets.QLabel("Mode"), row, 0)
        grid.addWidget(self.checkerboard_validation_mode_combo, row, 1, 1, 2)
        row += 1

        scan_button = QtWidgets.QPushButton("Scan")
        scan_button.clicked.connect(self._scan_checkerboard_validation)
        self.checkerboard_validation_scan_label = QtWidgets.QLabel("--")
        self.checkerboard_validation_scan_label.setWordWrap(True)
        grid.addWidget(scan_button, row, 0)
        grid.addWidget(self.checkerboard_validation_scan_label, row, 1, 1, 2)
        row += 1

        self.checkerboard_validation_json_edit = QtWidgets.QLineEdit()
        self.checkerboard_validation_json_edit.setToolTip(
            "Existing intrinsics/extrinsics JSON to use as the validation reference, instead of "
            "capturing and computing a fresh result in Compute first."
        )
        browse_json_button = QtWidgets.QPushButton("Browse…")
        browse_json_button.clicked.connect(self._choose_checkerboard_validation_json)
        grid.addWidget(QtWidgets.QLabel("Reference JSON"), row, 0)
        grid.addWidget(self.checkerboard_validation_json_edit, row, 1)
        grid.addWidget(browse_json_button, row, 2)
        row += 1
        load_json_button = QtWidgets.QPushButton("Load JSON as reference")
        load_json_button.clicked.connect(self._load_checkerboard_validation_json)
        grid.addWidget(load_json_button, row, 1, 1, 2)
        row += 1

        self.checkerboard_validation_random_pair_count_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_random_pair_count_spin.setRange(0, 500)
        self.checkerboard_validation_random_pair_count_spin.setToolTip(
            "Stereo pairs per random subset run; 0 auto-sizes the same way a normal run does."
        )
        self.checkerboard_validation_random_runs_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_random_runs_spin.setRange(1, 50)
        self.checkerboard_validation_random_runs_spin.setValue(5)
        self.checkerboard_validation_seed_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_seed_spin.setRange(0, 999999)
        self.checkerboard_validation_seed_spin.setToolTip("0 uses a non-deterministic seed.")
        grid.addWidget(QtWidgets.QLabel("Subset pairs"), row, 0)
        grid.addWidget(self.checkerboard_validation_random_pair_count_spin, row, 1, 1, 2)
        row += 1
        grid.addWidget(QtWidgets.QLabel("Runs"), row, 0)
        grid.addWidget(self.checkerboard_validation_random_runs_spin, row, 1, 1, 2)
        row += 1
        grid.addWidget(QtWidgets.QLabel("Seed"), row, 0)
        grid.addWidget(self.checkerboard_validation_seed_spin, row, 1, 1, 2)
        row += 1

        self.checkerboard_validation_random_button = QtWidgets.QPushButton("Run random validation")
        self.checkerboard_validation_random_button.setToolTip(
            "Requires a reference result first: load one above, or run a normal compute in "
            "Compute. Recomputes K,D (and R,t in Checkerboard mode) from several random "
            "subsets of the scanned stereo pairs and reports how much the result varies."
        )
        self.checkerboard_validation_random_button.clicked.connect(self._run_checkerboard_validation)
        grid.addWidget(self.checkerboard_validation_random_button, row, 0, 1, 3)
        row += 1

        self.checkerboard_validation_status_label = QtWidgets.QLabel("계산 결과 없음")
        self.checkerboard_validation_status_label.setWordWrap(True)
        grid.addWidget(self.checkerboard_validation_status_label, row, 0, 1, 3)
        row += 1

        self.checkerboard_validation_results_text = QtWidgets.QPlainTextEdit()
        self.checkerboard_validation_results_text.setReadOnly(True)
        self.checkerboard_validation_results_text.setMaximumHeight(160)
        grid.addWidget(self.checkerboard_validation_results_text, row, 0, 1, 3)
        row += 1

        self.checkerboard_validation_save_button = QtWidgets.QPushButton("Save result")
        self.checkerboard_validation_save_button.setProperty(
            "commandId", "calibration.validation.save"
        )
        self.checkerboard_validation_save_button.clicked.connect(
            self._save_checkerboard_validation_result
        )
        self.checkerboard_validation_apply_button = QtWidgets.QPushButton("Apply result")
        self.checkerboard_validation_apply_button.setProperty(
            "commandId", "calibration.validation.apply"
        )
        self.checkerboard_validation_apply_button.clicked.connect(
            self._apply_checkerboard_validation_result
        )
        self.commands.primary["calibration.validation.save"] = (
            self.checkerboard_validation_save_button
        )
        self.commands.primary["calibration.validation.apply"] = (
            self.checkerboard_validation_apply_button
        )
        grid.addWidget(self.checkerboard_validation_save_button, row, 0)
        grid.addWidget(self.checkerboard_validation_apply_button, row, 1)

        self._checkerboard_validation_poll_timer = QtCore.QTimer(self)
        self._checkerboard_validation_poll_timer.setInterval(700)
        self._checkerboard_validation_poll_timer.timeout.connect(self._poll_checkerboard_validation)
        return group


    def _choose_checkerboard_validation_root(self) -> None:
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self._dialog_parent,
            "Select checkerboard image root",
            self.checkerboard_validation_root_edit.text(),
        )
        if directory:
            self.checkerboard_validation_root_edit.setText(directory)


    def _choose_checkerboard_validation_json(self) -> None:
        path, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self._dialog_parent,
            "Select reference intrinsics JSON",
            "",
            "Intrinsics JSON (*.json);;All files (*)",
        )
        if path:
            self.checkerboard_validation_json_edit.setText(path)


    def _checkerboard_validation_payload(self) -> dict[str, object]:
        return {
            "root": self.checkerboard_validation_root_edit.text().strip(),
            "mode": str(self.checkerboard_validation_mode_combo.currentData() or "pixhawk"),
            "json_path": self.checkerboard_validation_json_edit.text().strip(),
            "random_pair_count": self.checkerboard_validation_random_pair_count_spin.value(),
            "runs": self.checkerboard_validation_random_runs_spin.value(),
            "seed": self.checkerboard_validation_seed_spin.value(),
        }


    def _apply_checkerboard_validation_snapshot(self, snapshot: object) -> None:
        if not isinstance(snapshot, Mapping):
            return
        self.checkerboard_validation_scan_label.setText(str(snapshot.get("scan_status") or "--"))
        self.checkerboard_validation_status_label.setText(str(snapshot.get("status") or "계산 결과 없음"))
        lines: list[str] = []
        summary = snapshot.get("summary")
        if isinstance(summary, Mapping) and summary:
            lines.append(f"summary: {summary}")
        run_results = snapshot.get("run_results")
        if isinstance(run_results, (list, tuple)):
            lines.extend(str(row) for row in run_results if isinstance(row, Mapping))
        self.checkerboard_validation_results_text.setPlainText("\n".join(lines))
        quality = snapshot.get("calibration_quality")
        if isinstance(quality, Mapping):
            self._present_quality(quality)
        running = bool(snapshot.get("running"))
        self.checkerboard_validation_random_button.setEnabled(not running)
        if running:
            if not self._checkerboard_validation_poll_timer.isActive():
                self._checkerboard_validation_poll_timer.start()
        else:
            self._checkerboard_validation_poll_timer.stop()


    def _scan_checkerboard_validation(self) -> None:
        method = self._read_actions().configure
        if not callable(method):
            return
        payload = {
            "root": self.checkerboard_validation_root_edit.text().strip(),
            "mode": str(self.checkerboard_validation_mode_combo.currentData() or "pixhawk"),
        }
        self._apply_checkerboard_validation_snapshot(method(payload))


    def _load_checkerboard_validation_json(self) -> None:
        method = self._read_actions().load_reference
        if not callable(method):
            return
        self._apply_checkerboard_validation_snapshot(method(self._checkerboard_validation_payload()))


    def _run_checkerboard_validation(self) -> None:
        method = self._read_actions().run
        if not callable(method):
            return
        self._apply_checkerboard_validation_snapshot(method(self._checkerboard_validation_payload()))


    def _poll_checkerboard_validation(self) -> None:
        method = self._read_actions().snapshot
        if not callable(method):
            self._checkerboard_validation_poll_timer.stop()
            return
        self._apply_checkerboard_validation_snapshot(method())


    def _save_checkerboard_validation_result(self) -> None:
        method = self._read_actions().save
        if not callable(method):
            return
        self._apply_checkerboard_validation_snapshot(
            method(
                quality_warning_confirmed=bool(
                    self._warning_confirmed()
                )
            )
        )


    def _apply_checkerboard_validation_result(self) -> None:
        method = self._read_actions().apply
        if not callable(method):
            return
        self._apply_checkerboard_validation_snapshot(
            method(
                quality_warning_confirmed=bool(
                    self._warning_confirmed()
                )
            )
        )


