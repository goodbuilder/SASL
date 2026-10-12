"""CalibrationQualityPanel owns its widgets and transient presentation state.

Domain formulas, native raster/timing admission and calibration solves remain with
the existing backend and core contracts. No workspace host is accepted.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.workstation_core.theme import ERROR_TEXT, SUCCESS_TEXT
from Workstation.workstation_core.settings import DEFAULT_CALIBRATION_ROOT
from Workstation.workstation_core.calibration_selection import resolve_calibration_selection_paths
from Workstation.ui.components import EngineeringValue
from ..shared import _display_number

from .selection_files import write_outlier_selection
from .widgets import CalibrationErrorPlot


class CalibrationQualityPanel(QtCore.QObject):
    _CALIBRATION_QUALITY_GATED_COMMANDS = frozenset({
        "calibration.apply_bundle", "calibration.validation.apply",
    })

    def __init__(self, commands, *, read_selection, selection_saved, refresh_labels,
                 read_quality, read_diagnostics, read_setup, diagnostics_signal=None,
                 default_root=DEFAULT_CALIBRATION_ROOT, parent=None):
        super().__init__(parent)
        if not all(callable(reader) for reader in (read_quality, read_diagnostics, read_setup)):
            raise TypeError("Calibration quality requires quality, diagnostics and setup readers")
        self._read_quality = read_quality
        self._read_diagnostics = read_diagnostics
        self._read_setup = read_setup
        self._diagnostics_signal = diagnostics_signal
        self._diagnostics_signal_connected = False
        self._disposed = False
        self.default_root = default_root
        self.commands = commands
        self._load_calibration_pair_selection = read_selection
        self._selection_saved = selection_saved
        self._sync_simple_calibration_labels = refresh_labels
        self._calibration_diagnostics_signature = None
        self._calibration_quality_signature = None
        self._calibration_quality_state = "UNVERIFIED"
        self._calibration_quality_snapshot_cache = {}
        self._calibration_quality_warning_confirmed = False

    def invalidate_diagnostics(self):
        self._calibration_diagnostics_signature = None
        self._sync_calibration_outlier_view()

    @staticmethod
    def _selection_paths(raw_paths, root, *, stored_root=None):
        paths, _errors = resolve_calibration_selection_paths(raw_paths, root, stored_root=stored_root)
        return paths

    def _connect_calibration_diagnostics_signal(self) -> None:
        if self._disposed or self._diagnostics_signal_connected:
            return
        connect = getattr(self._diagnostics_signal, "connect", None)
        if callable(connect):
            connect(self._on_calibration_diagnostics_changed)
            self._diagnostics_signal_connected = True

    def stop(self):
        self._disposed = True
        if self._diagnostics_signal_connected:
            try:
                self._diagnostics_signal.disconnect(self._on_calibration_diagnostics_changed)
            except (TypeError, RuntimeError):
                pass
            self._diagnostics_signal_connected = False

    @QtCore.pyqtSlot(object)
    def _on_calibration_diagnostics_changed(self, _payload: object = None) -> None:
        if self._disposed:
            return
        self._calibration_diagnostics_signature = None
        self._sync_calibration_outlier_view()

    def _build_calibration_outlier_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Outlier analysis")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(5)
        self.calibration_outlier_summary = QtWidgets.QLabel(
            "Run calibration (K,D) 후 사진별 reprojection error가 표시됩니다."
        )
        self.calibration_outlier_summary.setObjectName("statusStrip")
        self.calibration_outlier_summary.setWordWrap(True)
        layout.addWidget(self.calibration_outlier_summary)
        self.calibration_outlier_plot = CalibrationErrorPlot()
        self.calibration_outlier_plot.exclude_requested.connect(
            lambda payload: self._handle_calibration_outlier_action(payload, restore=False)
        )
        self.calibration_outlier_plot.restore_requested.connect(
            lambda payload: self._handle_calibration_outlier_action(payload, restore=True)
        )
        layout.addWidget(self.calibration_outlier_plot)
        self.calibration_outlier_table = QtWidgets.QTableWidget(0, 6)
        self.calibration_outlier_table.setObjectName("calibrationOutlierTable")
        self.calibration_outlier_table.setHorizontalHeaderLabels(
            ("Side", "File", "Error px", "Coverage", "Board", "State")
        )
        self.calibration_outlier_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.calibration_outlier_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.calibration_outlier_table.verticalHeader().setVisible(False)
        self.calibration_outlier_table.setMinimumHeight(72)
        self.calibration_outlier_table.setMaximumHeight(190)
        header = self.calibration_outlier_table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(4, QtWidgets.QHeaderView.ResizeToContents)
        header.setStretchLastSection(True)
        layout.addWidget(self.calibration_outlier_table)
        return group


    def _build_calibration_quality_group(self) -> QtWidgets.QGroupBox:
        """Build a presentation-only view of the owner's quality snapshot."""

        group = QtWidgets.QGroupBox("Calibration quality gate")
        group.setObjectName("calibrationQualityGroup")
        self.calibration_quality_group = group
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(5)

        values_layout = QtWidgets.QGridLayout()
        values_layout.setHorizontalSpacing(8)
        values_layout.setVerticalSpacing(4)
        self.calibration_quality_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("state", "Quality state"),
                ("effective_pose_count", "Effective poses"),
                ("coverage", "Center coverage"),
                ("principal_offsets", "Principal offsets"),
                ("subset_stability", "Subset cx/cy/focal stability"),
                ("uncertainty", "Solver uncertainty / max view error"),
                ("reasons", "Reasons"),
                ("source", "Source"),
            )
        ):
            value = EngineeringValue(label)
            value.setObjectName(f"calibrationQuality_{key}")
            self.calibration_quality_values[key] = value
            values_layout.addWidget(value, index // 2, index % 2)
        self.calibration_quality_state_label = self.calibration_quality_values["state"].value
        self.calibration_quality_reasons_label = self.calibration_quality_values["reasons"].value
        layout.addLayout(values_layout)

        coverage_label = QtWidgets.QLabel(
            "Board-center occupancy (source-provided 3×3 grid; no UI geometry recomputation)"
        )
        coverage_label.setObjectName("compactContract")
        coverage_label.setWordWrap(True)
        layout.addWidget(coverage_label)
        self.calibration_center_coverage_table = QtWidgets.QTableWidget(3, 3)
        self.calibration_center_coverage_table.setObjectName("calibrationCenterCoverage3x3")
        self.calibration_center_coverage_table.setHorizontalHeaderLabels(
            ("Left", "Center", "Right")
        )
        self.calibration_center_coverage_table.setVerticalHeaderLabels(
            ("Top", "Middle", "Bottom")
        )
        self.calibration_center_coverage_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.calibration_center_coverage_table.setSelectionMode(
            QtWidgets.QAbstractItemView.NoSelection
        )
        self.calibration_center_coverage_table.setMaximumHeight(126)
        self.calibration_center_coverage_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch
        )
        self.calibration_center_coverage_table.verticalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch
        )
        layout.addWidget(self.calibration_center_coverage_table)

        self.calibration_quality_warning_checkbox = QtWidgets.QCheckBox(
            "I reviewed this WARNING and explicitly allow Save / Apply for this result"
        )
        self.calibration_quality_warning_checkbox.setObjectName(
            "calibrationQualityWarningConfirmation"
        )
        self.calibration_quality_warning_checkbox.setToolTip(
            "Confirmation applies only to the currently displayed source-backed quality snapshot."
        )
        self.calibration_quality_warning_checkbox.toggled.connect(
            self.confirm_calibration_quality_warning
        )
        self.calibration_quality_warning_checkbox.setVisible(False)
        layout.addWidget(self.calibration_quality_warning_checkbox)

        self.calibration_quality_gate_label = QtWidgets.QLabel(
            "UNVERIFIED · legacy result has no quality snapshot; existing Save / Apply behavior is preserved."
        )
        self.calibration_quality_gate_label.setObjectName("statusStrip")
        self.calibration_quality_gate_label.setWordWrap(True)
        layout.addWidget(self.calibration_quality_gate_label)

        for row in range(3):
            for column in range(3):
                item = QtWidgets.QTableWidgetItem("--")
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.calibration_center_coverage_table.setItem(row, column, item)
        return group


    @staticmethod
    def _calibration_quality_state_text(value: object) -> str:
        state = str(getattr(value, "value", value) or "").strip().upper()
        aliases = {
            "GOOD": "PASS",
            "VALID": "PASS",
            "PASSED": "PASS",
            "OK": "PASS",
            "WARN": "WARNING",
            "INVALID": "FAIL",
            "FAILED": "FAIL",
            "ERROR": "FAIL",
            "UNKNOWN": "UNVERIFIED",
            "NOT_VERIFIED": "UNVERIFIED",
            "LEGACY_UNVERIFIED": "UNVERIFIED",
        }
        normalized = aliases.get(state, state)
        return normalized if normalized in {"PASS", "WARNING", "FAIL"} else "UNVERIFIED"


    @staticmethod
    def _calibration_quality_reasons(value: object) -> tuple[str, ...]:
        reasons: list[str] = []
        rows = value if isinstance(value, (list, tuple, set)) else (value,)
        for row in rows:
            if row in (None, ""):
                continue
            if isinstance(row, Mapping):
                code = str(row.get("code") or row.get("id") or "").strip()
                detail = str(
                    row.get("message")
                    or row.get("reason")
                    or row.get("detail")
                    or row.get("status")
                    or ""
                ).strip()
                text = f"{code}: {detail}" if code and detail else code or detail
            else:
                text = str(row).strip()
            if text and text not in reasons:
                reasons.append(text)
        return tuple(reasons)


    @staticmethod
    def _calibration_quality_display(value: object) -> str:
        if value in (None, "", (), [], {}):
            return "--"
        if isinstance(value, bool):
            return "yes" if value else "no"
        if isinstance(value, float):
            return _display_number(value, digits=4)
        if isinstance(value, Mapping):
            parts = []
            for key, child in value.items():
                if key in {"grid", "grid_3x3", "cells", "coverage_grid_3x3"}:
                    continue
                parts.append(
                    f"{key}={CalibrationQualityPanel._calibration_quality_display(child)}"
                )
            return " · ".join(parts) or "--"
        if isinstance(value, (list, tuple)):
            return ", ".join(
                CalibrationQualityPanel._calibration_quality_display(child) for child in value
            )
        return str(value)


    @staticmethod
    def _calibration_quality_mapping(value: object) -> dict[str, object]:
        if not isinstance(value, Mapping):
            return {}
        snapshot = dict(value)
        for key in ("calibration_quality", "quality_snapshot", "quality"):
            nested = snapshot.get(key)
            if isinstance(nested, Mapping) and any(
                field in nested
                for field in (
                    "state",
                    "status",
                    "verdict",
                    "quality_state",
                    "effective_pose_count",
                    "coverage_grid_3x3",
                )
            ):
                merged = dict(snapshot)
                merged.update(dict(nested))
                return merged
        return snapshot


    def calibration_quality_snapshot(self) -> dict[str, object]:
        return self._read_quality()

    @staticmethod
    def _calibration_quality_field(
        snapshot: Mapping[str, object],
        *names: str,
    ) -> object:
        for container in (
            snapshot,
            snapshot.get("metrics") if isinstance(snapshot.get("metrics"), Mapping) else {},
        ):
            if not isinstance(container, Mapping):
                continue
            for name in names:
                if name in container and container[name] not in (None, ""):
                    return container[name]
        return None


    @staticmethod
    def _calibration_coverage_grid(snapshot: Mapping[str, object]) -> object:
        coverage = snapshot.get("coverage")
        for value in (
            snapshot.get("coverage_grid_3x3"),
            snapshot.get("center_coverage_grid"),
            coverage.get("grid_3x3") if isinstance(coverage, Mapping) else None,
            coverage.get("grid") if isinstance(coverage, Mapping) else None,
            coverage.get("cells") if isinstance(coverage, Mapping) else None,
        ):
            if value not in (None, ""):
                return value
        return None


    @staticmethod
    def _calibration_grid_cell(grid: object, row: int, column: int) -> object:
        if isinstance(grid, (list, tuple)):
            if len(grid) == 9:
                return grid[row * 3 + column]
            if len(grid) > row and isinstance(grid[row], (list, tuple)) and len(grid[row]) > column:
                return grid[row][column]
        if isinstance(grid, Mapping):
            names = (
                ("top_left", "top_center", "top_right"),
                ("middle_left", "center", "middle_right"),
                ("bottom_left", "bottom_center", "bottom_right"),
            )
            for key in (
                names[row][column],
                f"{row},{column}",
                f"r{row}c{column}",
                str(row * 3 + column),
            ):
                if key in grid:
                    return grid[key]
        return None


    @staticmethod
    def _calibration_grid_cell_text(value: object) -> tuple[str, bool | None]:
        if value is None:
            return "--", None
        if isinstance(value, Mapping):
            covered = value.get("covered")
            display = next(
                (
                    value[key]
                    for key in ("label", "count", "pose_count", "fraction", "value")
                    if key in value
                ),
                "covered" if covered is True else "empty" if covered is False else "--",
            )
            return CalibrationQualityPanel._calibration_quality_display(display), (
                bool(covered) if covered is not None else None
            )
        if isinstance(value, bool):
            return ("covered", True) if value else ("empty", False)
        return CalibrationQualityPanel._calibration_quality_display(value), None


    def apply_calibration_quality_snapshot(self, value: object) -> dict[str, object]:
        """Present and gate one source snapshot; never calculate quality here."""

        snapshot = self._calibration_quality_mapping(value)
        raw_state = self._calibration_quality_field(
            snapshot, "state", "quality_state", "verdict", "status"
        )
        state = self._calibration_quality_state_text(raw_state)
        reasons = self._calibration_quality_reasons(
            self._calibration_quality_field(
                snapshot,
                "reasons",
                "quality_reasons",
                "failure_reasons",
                "warning_reasons",
            )
        )
        if not snapshot:
            reasons = ("No source-backed calibration quality snapshot (legacy result)",)
        elif state == "UNVERIFIED" and raw_state not in (None, "") and not reasons:
            reasons = (f"Unsupported source quality state: {raw_state}",)
        source = str(
            snapshot.get("source")
            or snapshot.get("producer")
            or snapshot.get("source_id")
            or "legacy/no quality source"
        )
        effective_pose_count = self._calibration_quality_field(
            snapshot,
            "effective_pose_count",
            "effective_poses",
            "usable_pose_count",
            "inlier_pose_count",
        )
        coverage = self._calibration_quality_field(
            snapshot,
            "coverage",
            "center_coverage",
            "board_center_coverage",
            "coverage_fraction",
        )
        principal_offsets = self._calibration_quality_field(
            snapshot,
            "principal_point_offsets_px",
            "principal_offsets_px",
            "principal_offsets",
        )
        if principal_offsets is None:
            principal_parts = {
                key: self._calibration_quality_field(snapshot, *aliases)
                for key, aliases in (
                    ("left", ("left_principal_offset_px", "left_principal_point_offset_px")),
                    ("right", ("right_principal_offset_px", "right_principal_point_offset_px")),
                    ("cx", ("principal_offset_cx_px", "principal_cx_offset_px")),
                    ("cy", ("principal_offset_cy_px", "principal_cy_offset_px")),
                )
            }
            principal_offsets = {
                key: child for key, child in principal_parts.items() if child is not None
            } or None
        subset_stability = self._calibration_quality_field(
            snapshot,
            "subset_stability_px",
            "subset_stability",
            "random_subset_stability",
        )
        if subset_stability is None:
            subset_parts = {
                key: self._calibration_quality_field(snapshot, *aliases)
                for key, aliases in (
                    ("cx", ("subset_cx_stability_px", "subset_cx_std_px")),
                    ("cy", ("subset_cy_stability_px", "subset_cy_std_px")),
                    (
                        "focal",
                        ("subset_focal_stability_px", "subset_focal_std_px", "subset_fx_std_px"),
                    ),
                )
            }
            subset_stability = {
                key: child for key, child in subset_parts.items() if child is not None
            } or None
        uncertainty = self._calibration_quality_field(
            snapshot,
            "uncertainty",
            "solver_uncertainty",
            "intrinsic_uncertainty",
        )
        signature_values = {
            "state": state,
            "reasons": reasons,
            "effective_pose_count": effective_pose_count,
            "coverage": coverage,
            "principal_offsets": principal_offsets,
            "subset_stability": subset_stability,
            "uncertainty": uncertainty,
            "grid": self._calibration_coverage_grid(snapshot),
            "source": source,
            "quality_id": snapshot.get(
                "quality_id", snapshot.get("result_id", snapshot.get("profile_sha256"))
            ),
        }
        signature = (
            json.dumps(signature_values, sort_keys=True, ensure_ascii=False, default=str),
        )
        if signature != self._calibration_quality_signature:
            self._calibration_quality_signature = signature
            self._calibration_quality_warning_confirmed = False
            checkbox = getattr(self, "calibration_quality_warning_checkbox", None)
            if checkbox is not None:
                checkbox.blockSignals(True)
                checkbox.setChecked(False)
                checkbox.blockSignals(False)

        self._calibration_quality_state = state
        normalized = dict(snapshot)
        normalized.update(state=state, reasons=reasons, source=source)
        self._calibration_quality_snapshot_cache = normalized
        valid = True if state == "PASS" else False if state == "FAIL" else None
        self.calibration_quality_values["state"].set_value(
            state, source=source, valid=valid
        )
        self.calibration_quality_values["effective_pose_count"].set_value(
            effective_pose_count, source=source
        )
        self.calibration_quality_values["coverage"].set_value(
            self._calibration_quality_display(coverage), source=source
        )
        self.calibration_quality_values["principal_offsets"].set_value(
            self._calibration_quality_display(principal_offsets), unit="px", source=source
        )
        self.calibration_quality_values["subset_stability"].set_value(
            self._calibration_quality_display(subset_stability), unit="px", source=source
        )
        self.calibration_quality_values["uncertainty"].set_value(
            self._calibration_quality_display(uncertainty), source=source
        )
        self.calibration_quality_values["reasons"].set_value(
            " · ".join(reasons) if reasons else "none", source=source
        )
        self.calibration_quality_values["source"].set_value(source, source=source)

        grid = self._calibration_coverage_grid(snapshot)
        for row in range(3):
            for column in range(3):
                text, covered = self._calibration_grid_cell_text(
                    self._calibration_grid_cell(grid, row, column)
                )
                item = self.calibration_center_coverage_table.item(row, column)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    self.calibration_center_coverage_table.setItem(row, column, item)
                item.setText(text)
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                item.setToolTip(f"Source: {source}\nValue: {text}")
                item.setForeground(
                    QtGui.QBrush(
                        QtGui.QColor(SUCCESS_TEXT if covered is True else ERROR_TEXT)
                    )
                    if covered is not None
                    else QtGui.QBrush()
                )

        self.calibration_quality_warning_checkbox.setVisible(state == "WARNING")
        self._apply_calibration_quality_gate()
        return dict(normalized)


    def refresh_calibration_quality(self) -> dict[str, object]:
        return self.apply_calibration_quality_snapshot(self.calibration_quality_snapshot())


    def confirm_calibration_quality_warning(self, confirmed: bool = True) -> bool:
        if self._calibration_quality_state != "WARNING":
            return False
        self._calibration_quality_warning_confirmed = bool(confirmed)
        checkbox = getattr(self, "calibration_quality_warning_checkbox", None)
        if checkbox is not None and checkbox.isChecked() != bool(confirmed):
            checkbox.blockSignals(True)
            checkbox.setChecked(bool(confirmed))
            checkbox.blockSignals(False)
        self._apply_calibration_quality_gate()
        return self._calibration_quality_warning_confirmed


    def calibration_quality_action_allowed(self, command_id: str) -> bool:
        if command_id not in self._CALIBRATION_QUALITY_GATED_COMMANDS:
            return True
        if self._calibration_quality_state == "FAIL":
            return False
        if self._calibration_quality_state == "WARNING":
            return bool(self._calibration_quality_warning_confirmed)
        return True


    def _apply_calibration_quality_gate(self) -> None:
        owned = getattr(self, "_calibration_quality_gate_owned", set())
        original_tooltips = getattr(self, "_calibration_quality_original_tooltips", {})
        reasons = tuple(self._calibration_quality_snapshot_cache.get("reasons") or ())
        reason_text = " · ".join(str(reason) for reason in reasons) or "no reason supplied"
        state = self._calibration_quality_state
        for command_id in self._CALIBRATION_QUALITY_GATED_COMMANDS:
            buttons = tuple(self.commands.instances.get(command_id, ()))
            if not buttons:
                primary = self.commands.primary.get(command_id)
                buttons = () if primary is None else (primary,)
            if not buttons:
                continue
            allowed = self.calibration_quality_action_allowed(command_id)
            for index, button in enumerate(buttons):
                owner_key = f"{command_id}:{index}"
                original_tooltips.setdefault(owner_key, button.toolTip())
                if not allowed:
                    if button.isEnabled():
                        owned.add(owner_key)
                    button.setEnabled(False)
                    if state == "FAIL":
                        button.setToolTip(f"Blocked by calibration quality FAIL: {reason_text}")
                    else:
                        button.setToolTip(
                            "Calibration quality WARNING requires the explicit confirmation checkbox. "
                            + reason_text
                        )
                elif owner_key in owned:
                    button.setEnabled(True)
                    owned.discard(owner_key)
                    button.setToolTip(original_tooltips.get(owner_key, ""))
        self._calibration_quality_gate_owned = owned
        self._calibration_quality_original_tooltips = original_tooltips

        if state == "PASS":
            text = "PASS · result can be saved as an approved profile and applied."
        elif state == "WARNING":
            text = (
                "WARNING acknowledged · Apply explicitly enabled; Save remains evidence-preserving."
                if self._calibration_quality_warning_confirmed
                else "WARNING · Save preserves an attempt; review reasons and confirm before Apply."
            )
        elif state == "FAIL":
            text = f"FAIL · Save preserves an attempt; Apply is blocked. {reason_text}"
        else:
            text = (
                "UNVERIFIED · legacy/no-quality result; it is not presented as PASS and "
                "existing Save / Apply behavior is preserved."
            )
        self.calibration_quality_gate_label.setText(text)
        self._sync_simple_calibration_labels()


    def _calibration_diagnostic_rows(self) -> tuple[dict[str, object], ...]:
        return self._read_diagnostics()

    def _calibration_exclusion_sets(self) -> tuple[set[str], set[str]]:
        selection = self._load_calibration_pair_selection()
        root_text = str(self._read_setup().get("calibration_root_resolved") or "")
        root = Path(root_text) if root_text else Path(self.default_root)
        left = {
            str(path)
            for path in self._selection_paths(
                selection.get("excluded_left_paths"),
                root,
                stored_root=selection.get("root"),
            )
        }
        right = {
            str(path)
            for path in self._selection_paths(
                selection.get("excluded_right_paths"),
                root,
                stored_root=selection.get("root"),
            )
        }
        return left, right


    def _sync_calibration_outlier_view(self) -> None:
        plot = getattr(self, "calibration_outlier_plot", None)
        table = getattr(self, "calibration_outlier_table", None)
        if plot is None or table is None:
            return
        rows = self._calibration_diagnostic_rows()
        excluded_left, excluded_right = self._calibration_exclusion_sets()
        selection = self._load_calibration_pair_selection()
        excluded_pairs = {str(key) for key in selection.get("excluded_pair_keys") or ()}
        signature = (
            tuple(tuple(sorted(row.items())) for row in rows),
            tuple(sorted(excluded_left)),
            tuple(sorted(excluded_right)),
            tuple(sorted(excluded_pairs)),
        )
        if signature == self._calibration_diagnostics_signature:
            return
        self._calibration_diagnostics_signature = signature
        excluded_paths = excluded_left | excluded_right
        plot.set_diagnostics(
            rows,
            excluded_paths=excluded_paths,
            excluded_pair_keys=excluded_pairs,
        )
        table.setRowCount(len(rows))
        threshold = plot._threshold_px
        for index, row in enumerate(rows):
            error = plot._finite_error(row)
            path = Path(str(row.get("path") or ""))
            excluded = str(path) in excluded_paths or str(row.get("capture_key") or "") in excluded_pairs
            state = "EXCLUDED" if excluded else "candidate" if threshold is not None and error is not None and error >= threshold else "included"
            coverage = row.get("board_coverage_fraction")
            coverage_text = "--" if coverage is None else f"{float(coverage) * 100.0:.1f}%"
            values = (
                str(row.get("side") or "").upper(),
                path.name or str(row.get("path") or "--"),
                "--" if error is None else f"{error:.3f}",
                coverage_text,
                str(row.get("capture_key") or "--"),
                state,
            )
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setToolTip(str(row.get("path") or value))
                if state == "EXCLUDED":
                    item.setForeground(QtGui.QBrush(QtGui.QColor("#888888")))
                elif state == "candidate":
                    item.setForeground(QtGui.QBrush(QtGui.QColor("#c0392b")))
                table.setItem(index, column, item)
        if not rows:
            self.calibration_outlier_summary.setText(
                "계산 결과가 없습니다. Run calibration (K,D)를 먼저 실행하세요."
            )
        else:
            valid = sum(1 for row in rows if plot._finite_error(row) is not None)
            excluded = len(excluded_paths)
            candidate_count = sum(
                1
                for row in rows
                if plot._finite_error(row) is not None
                and plot._finite_error(row) >= (threshold or float("inf"))
            )
            self.calibration_outlier_summary.setText(
                f"진단 {len(rows)}장 · 유효 잔차 {valid}장 · 자동 후보 {candidate_count}장 · "
                f"제외 {excluded}장 | 점 우클릭으로 제외/복원 후 다시 계산하세요."
            )


    def _handle_calibration_outlier_action(self, payload: object, *, restore: bool) -> None:
        root_text = str(self._read_setup().get("calibration_root_resolved") or "")
        root = Path(root_text) if root_text else Path(self.default_root)
        changed, message = write_outlier_selection(
            root, self._load_calibration_pair_selection(), self._calibration_diagnostic_rows(),
            payload, restore=restore,
        )
        if changed:
            self._calibration_diagnostics_signature = None
            self._selection_saved()
            self._sync_calibration_outlier_view()
        if message:
            self.calibration_outlier_summary.setText(message)


