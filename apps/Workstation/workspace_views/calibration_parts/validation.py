"""CalibrationValidationPanel owns its widgets and transient presentation state.

Domain formulas, native raster/timing admission and calibration solves remain with
the existing backend and core contracts. No workspace host is accepted.
"""

from __future__ import annotations

from collections.abc import Mapping
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.designer import load_form


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
        group = load_form('calibration_validation.ui', QtWidgets.QGroupBox())
        self.checkerboard_validation_root_edit = group.checkerboard_validation_root_edit
        browse_root_button = group.browse_root_button
        browse_root_button.clicked.connect(self._choose_checkerboard_validation_root)
        self.checkerboard_validation_mode_combo = group.checkerboard_validation_mode_combo
        self.checkerboard_validation_mode_combo.setItemData(0, 'pixhawk')
        self.checkerboard_validation_mode_combo.setItemData(1, 'checkerboard')
        scan_button = group.scan_button
        scan_button.clicked.connect(self._scan_checkerboard_validation)
        self.checkerboard_validation_scan_label = group.checkerboard_validation_scan_label
        self.checkerboard_validation_json_edit = group.checkerboard_validation_json_edit
        browse_json_button = group.browse_json_button
        browse_json_button.clicked.connect(self._choose_checkerboard_validation_json)
        load_json_button = group.load_json_button
        load_json_button.clicked.connect(self._load_checkerboard_validation_json)
        self.checkerboard_validation_random_pair_count_spin = group.checkerboard_validation_random_pair_count_spin
        self.checkerboard_validation_random_runs_spin = group.checkerboard_validation_random_runs_spin
        self.checkerboard_validation_seed_spin = group.checkerboard_validation_seed_spin
        self.checkerboard_validation_random_button = group.checkerboard_validation_random_button
        self.checkerboard_validation_random_button.clicked.connect(self._run_checkerboard_validation)
        self.checkerboard_validation_status_label = group.checkerboard_validation_status_label
        self.checkerboard_validation_results_text = group.checkerboard_validation_results_text
        self.checkerboard_validation_save_button = group.checkerboard_validation_save_button
        self.checkerboard_validation_save_button.setProperty('commandId', 'calibration.validation.save')
        self.checkerboard_validation_save_button.clicked.connect(self._save_checkerboard_validation_result)
        self.checkerboard_validation_apply_button = group.checkerboard_validation_apply_button
        self.checkerboard_validation_apply_button.setProperty('commandId', 'calibration.validation.apply')
        self.checkerboard_validation_apply_button.clicked.connect(self._apply_checkerboard_validation_result)
        self.commands.primary['calibration.validation.save'] = self.checkerboard_validation_save_button
        self.commands.primary['calibration.validation.apply'] = self.checkerboard_validation_apply_button
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


