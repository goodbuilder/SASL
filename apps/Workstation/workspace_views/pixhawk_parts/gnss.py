"""GPS1/GPS2 selection, WGS84 receiver values and NED/body-FRD baseline display."""

from __future__ import annotations

from collections.abc import Mapping
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import EngineeringValue
from ..shared import _display_number, _live_inspection, _sync_table_rows, _table_model

import math
from Workstation.ui.telemetry_plots import BaselineYawAlignmentPlot
from Workstation.workstation_core.settings import normalize_pixhawk_gnss_source
from .formatting import _display_compass_deg, _display_signed_deg


class GnssView(QtCore.QObject):
    inspection_requested = QtCore.pyqtSignal(object)

    def __init__(self, *, read_gps, read_selected_source, set_source, read_baseline, read_yaw, parent=None):
        super().__init__(parent)
        if not all(callable(reader) for reader in (read_gps, read_selected_source, read_baseline, read_yaw)):
            raise TypeError("GNSS view requires receiver, source and baseline readers")
        self._read_gps = read_gps
        self._read_selected_source = read_selected_source
        self._set_source = set_source
        self._read_baseline = read_baseline
        self._read_yaw = read_yaw
        receiver_group = QtWidgets.QGroupBox("P1/P2 GNSS receivers")
        receiver_layout = QtWidgets.QVBoxLayout(receiver_group)
        self.receiver_model = _table_model(
            self,
            (
                "Receiver",
                "Fix number",
                "RTK state",
                "Satellites",
                "Altitude MSL m",
                "Altitude ellipsoid m",
                "HDOP/VDOP",
                "H/V acc m",
                "Age",
            ),
        )
        self.receiver_table = QtWidgets.QTableView()
        self.receiver_table.setModel(self.receiver_model)
        self.receiver_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.receiver_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.receiver_table.verticalHeader().setVisible(False)
        self.receiver_table.setMaximumHeight(190)
        self.receiver_table.horizontalHeader().setStretchLastSection(True)
        self.receiver_table.clicked.connect(self._inspect_receiver_row)
        receiver_layout.addWidget(self.receiver_table)
        self.receiver_group = receiver_group
        self._receiver_row_keys: tuple[tuple[str, str], ...] = ()

        baseline_group = QtWidgets.QGroupBox("P1→P2 baseline")
        baseline_layout = QtWidgets.QGridLayout(baseline_group)
        self.baseline_values: dict[str, EngineeringValue] = {}
        baseline_fields = (
            ("north_m", "NED North"),
            ("east_m", "NED East"),
            ("down_m", "NED Down"),
            ("length_m", "Length"),
            ("p1_body_frd_forward_m", "FRD Forward"),
            ("p1_body_frd_right_m", "FRD Right"),
            ("p1_body_frd_down_m", "FRD Down"),
            ("state", "Quality"),
        )
        for index, (key, label) in enumerate(baseline_fields):
            value = EngineeringValue(label)
            self.baseline_values[key] = value
            value.activated.connect(lambda key=key: self._inspect_baseline_value(key))
            baseline_layout.addWidget(value, index // 4, index % 4)
        self.baseline_group = baseline_group

        baseline_yaw_group = QtWidgets.QGroupBox("P1→P2 baseline / Pixhawk yaw")
        baseline_yaw_group_layout = QtWidgets.QHBoxLayout(baseline_yaw_group)
        baseline_yaw_group_layout.setContentsMargins(8, 8, 8, 8)
        baseline_yaw_group_layout.setSpacing(8)
        summary = QtWidgets.QWidget()
        summary.setMinimumWidth(330)
        summary.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        summary_layout = QtWidgets.QVBoxLayout(summary)
        summary_layout.setContentsMargins(0, 0, 0, 0)
        summary_layout.setSpacing(4)
        self.baseline_yaw_status_label = QtWidgets.QLabel("WAITING · Baseline WAITING · P1 WAITING · P2 WAITING")
        self.baseline_yaw_status_label.setObjectName("statusStrip")
        self.baseline_yaw_status_label.setFixedHeight(24)
        summary_layout.addWidget(self.baseline_yaw_status_label)
        values_grid = QtWidgets.QGridLayout()
        values_grid.setContentsMargins(0, 0, 0, 0)
        values_grid.setHorizontalSpacing(8)
        values_grid.setVerticalSpacing(3)
        self.baseline_yaw_values: dict[str, EngineeringValue] = {}
        baseline_yaw_fields = (
            ("baseline_azimuth", "Baseline azimuth", 0, 0),
            ("p2_minus_p1", "P2−P1 yaw", 0, 1),
            ("p1_yaw", "P1 yaw", 1, 0),
            ("baseline_p1", "Baseline−P1", 1, 1),
            ("p2_yaw", "P2 yaw", 2, 0),
            ("baseline_p2", "Baseline−P2", 2, 1),
        )
        for key, label_text, row, column in baseline_yaw_fields:
            value = EngineeringValue(label_text)
            reserve_width = value.value.fontMetrics().horizontalAdvance("-180.00")
            value.value.setMinimumWidth(reserve_width)
            value.activated.connect(lambda key=key: self._inspect_baseline_yaw_value(key))
            self.baseline_yaw_values[key] = value
            values_grid.addWidget(value, row, column)
            values_grid.setColumnStretch(column, 1)
        summary_layout.addLayout(values_grid)
        self.baseline_yaw_evidence_label = QtWidgets.QLabel("P1/P2 selected GNSS and P1/P2 ATTITUDE evidence waiting")
        self.baseline_yaw_evidence_label.setObjectName("statusStrip")
        self.baseline_yaw_evidence_label.setWordWrap(True)
        self.baseline_yaw_evidence_label.setFixedHeight(42)
        summary_layout.addWidget(self.baseline_yaw_evidence_label)
        self.baseline_yaw_contract_label = QtWidgets.QLabel(
            "Baseline−Pn = wrap180(azimuth(P1→P2) − Pn yaw), Pn=P1/P2 · + right/CW · horizontal NED"
        )
        self.baseline_yaw_contract_label.setObjectName("compactContract")
        self.baseline_yaw_contract_label.setWordWrap(True)
        self.baseline_yaw_contract_label.setFixedHeight(36)
        summary_layout.addWidget(self.baseline_yaw_contract_label)
        self.baseline_yaw_plot = BaselineYawAlignmentPlot()
        baseline_yaw_group_layout.addWidget(summary, 1)
        baseline_yaw_group_layout.addWidget(self.baseline_yaw_plot, 1)
        self.baseline_yaw_group = baseline_yaw_group
        self.baseline_yaw_group = baseline_yaw_group

        gnss_source_group = QtWidgets.QGroupBox("GNSS used for Pixhawk calculation")
        gnss_source_layout = QtWidgets.QGridLayout(gnss_source_group)
        gnss_source_hint = QtWidgets.QLabel(
            "P1/P2의 GPS1·GPS2는 동시에 표시됩니다. 아래 선택값이 P1→P2 baseline과 측정 계산에 사용됩니다."
        )
        gnss_source_hint.setObjectName("statusStrip")
        gnss_source_hint.setWordWrap(True)
        gnss_source_layout.addWidget(gnss_source_hint, 0, 0, 1, 4)
        self.gnss_source_combos: dict[str, QtWidgets.QComboBox] = {}
        for column, (side, label) in enumerate((("left", "P1 antenna"), ("right", "P2 antenna"))):
            gnss_source_layout.addWidget(QtWidgets.QLabel(label), 1, column * 2)
            combo = QtWidgets.QComboBox()
            combo.addItem("GPS1", "GPS1")
            combo.addItem("GPS2", "GPS2")
            combo.currentIndexChanged.connect(lambda _index, side=side: self._set_gnss_source_from_workspace(side))
            self.gnss_source_combos[side] = combo
            gnss_source_layout.addWidget(combo, 1, column * 2 + 1)
        self.gnss_source_summary = QtWidgets.QLabel("계산 source: P1 GPS1 → P2 GPS1")
        self.gnss_source_summary.setObjectName("statusStrip")
        self.gnss_source_summary.setWordWrap(True)
        gnss_source_layout.addWidget(self.gnss_source_summary, 2, 0, 1, 4)
        self.gnss_source_group = gnss_source_group
        if isinstance(parent, QtWidgets.QWidget):
            for group in (self.receiver_group, self.baseline_group, self.baseline_yaw_group, self.gnss_source_group):
                group.setParent(parent)

    def refresh_receivers(self):
        self._refresh_gnss_source_controls()
        receiver_rows = []
        receiver_keys = []
        for side, label in (("left", "P1"), ("right", "P2")):
            selected = self._read_selected_source(side)
            for source in ("GPS1", "GPS2"):
                gps = self._read_gps(side, source=source)
                marker = " · selected" if source == selected else ""
                receiver_keys.append((side, source))
                receiver_rows.append(
                    (
                        f"{label} {source}{marker}",
                        gps.get("fix_type", "--"),
                        gps.get("rtk_label", gps.get("state", "NO DATA")),
                        gps.get("satellites", "--"),
                        _display_number(gps.get("altitude_m")),
                        _display_number(gps.get("altitude_ellipsoid_m")),
                        f"{_display_number(gps.get('hdop'))}/{_display_number(gps.get('vdop'))}",
                        f"{_display_number(gps.get('h_acc_m'))}/{_display_number(gps.get('v_acc_m'))}",
                        _display_number(gps.get("age_s"), digits=2, suffix="s"),
                    )
                )
        self._receiver_row_keys = tuple(receiver_keys)
        _sync_table_rows(self.receiver_model, tuple(receiver_rows))

    def refresh_baseline(self):
        baseline = self._read_baseline()
        projection_state = str(baseline.get("p1_body_frd_state", "WAITING") or "WAITING")
        for key, value in self.baseline_values.items():
            raw = baseline.get(key)
            is_frd = key.startswith("p1_body_frd_")
            is_metric = key.endswith("_m")
            if is_frd:
                valid = None if projection_state == "WAITING" else projection_state == "READY"
                frame = "P1 body FRD [Forward,Right,Down]"
            else:
                valid = None if baseline.get("state") in {"NO DATA", "UNKNOWN"} else baseline.get("state") == "GOOD"
                frame = "local NED [North,East,Down]" if is_metric else ""
            value.set_value(
                _display_number(raw) if is_metric else raw,
                unit="m" if is_metric else "",
                frame=frame,
                valid=valid,
            )
        self._refresh_baseline_yaw_alignment(self._read_yaw())

    def _refresh_baseline_yaw_alignment(self, data: Mapping[str, object]) -> None:
        state = str(data.get("state") or "WAITING")
        baseline_state = str(data.get("baseline_state") or "WAITING")
        p1_state = str(data.get("p1_state") or "WAITING")
        p2_state = str(data.get("p2_state") or "WAITING")
        status_text = f"{state} · Baseline {baseline_state} · P1 {p1_state} · P2 {p2_state}"
        self.baseline_yaw_status_label.setText(status_text)

        field_specs = {
            "baseline_azimuth": (
                "baseline_azimuth_deg",
                _display_compass_deg,
                "selected P1/P2 GNSS baseline",
                "local NED horizontal; North=0 deg, clockwise",
            ),
            "p1_yaw": (
                "p1_yaw_deg",
                _display_compass_deg,
                "P1 MAVLink ATTITUDE",
                "P1 body FRD Forward in world NED; compass-normalized",
            ),
            "baseline_p1": (
                "baseline_p1_deg",
                _display_signed_deg,
                "selected GNSS baseline + P1 MAVLink ATTITUDE",
                "wrap180(P1→P2 azimuth - P1 yaw); + right/CW",
            ),
            "p2_yaw": (
                "p2_yaw_deg",
                _display_compass_deg,
                "P2 MAVLink ATTITUDE",
                "P2 body FRD Forward in world NED; compass-normalized",
            ),
            "baseline_p2": (
                "baseline_p2_deg",
                _display_signed_deg,
                "selected GNSS baseline + P2 MAVLink ATTITUDE",
                "wrap180(P1→P2 azimuth - P2 yaw); + right/CW",
            ),
            "p2_minus_p1": (
                "p2_minus_p1_deg",
                _display_signed_deg,
                "P1/P2 MAVLink ATTITUDE",
                "wrap180(P2 yaw - P1 yaw); + clockwise",
            ),
        }
        for key, (data_key, formatter, source, frame) in field_specs.items():
            self.baseline_yaw_values[key].set_value(
                formatter(data.get(data_key)),
                unit="deg",
                source=source,
                frame=frame,
            )

        def age_text(value: object) -> str:
            if value in (None, ""):
                return "--"
            try:
                age_s = float(value)
            except (TypeError, ValueError):
                return "--"
            return "--" if not math.isfinite(age_s) else f"{max(0.0, age_s):.2f}s"

        pair_delta = data.get("gps_pair_delta_ms")
        pair_delta_text = _display_number(pair_delta, digits=1, suffix="ms")
        evidence_text = (
            f"P1 {data.get('p1_gps_source', 'GPS1')} "
            f"fix={data.get('p1_fix_type', '--')}/{data.get('p1_rtk_label', 'NO DATA')} "
            f"GPS={age_text(data.get('p1_gps_age_s'))} ATT={age_text(data.get('p1_attitude_age_s'))} · "
            f"P2 {data.get('p2_gps_source', 'GPS1')} "
            f"fix={data.get('p2_fix_type', '--')}/{data.get('p2_rtk_label', 'NO DATA')} "
            f"GPS={age_text(data.get('p2_gps_age_s'))} ATT={age_text(data.get('p2_attitude_age_s'))} · "
            f"pair Δt={pair_delta_text} · baseline={data.get('baseline_quality', 'NO DATA')}"
        )
        self.baseline_yaw_evidence_label.setText(evidence_text)
        reason = str(data.get("reason") or "all required baseline/yaw evidence current")
        tooltip = (
            f"{status_text}\n{evidence_text}\nReason: {reason}\n"
            f"Clock: {data.get('clock', '')}\nContract: {data.get('angle_contract', '')}"
        )
        self.baseline_yaw_status_label.setToolTip(tooltip)
        self.baseline_yaw_evidence_label.setToolTip(tooltip)
        self.baseline_yaw_contract_label.setToolTip(
            f"{data.get('frame_contract', '')}\n{data.get('angle_contract', '')}\n"
            "Latest-state physical-alignment aid; not simultaneous measurement evidence."
        )
        self.baseline_yaw_plot.set_snapshot(data)

    def _inspect_receiver_row(self, index: QtCore.QModelIndex) -> None:
        if index.row() < 0 or index.row() >= len(self._receiver_row_keys):
            return
        side, source = self._receiver_row_keys[index.row()]
        self.inspection_requested.emit(
            _live_inspection(
                self._receiver_inspection(side, source),
                lambda side=side, source=source: self._receiver_inspection(side, source),
            )
        )

    def _receiver_inspection(self, side: str, source: str = "GPS1") -> dict[str, object]:
        label = "P1" if side == "left" else "P2"
        data = self._read_gps(side, source=source)
        return {
            "selection": f"{label} {source} GNSS receiver",
            "value": (
                f"fix number={data.get('fix_type', '--')} · {data.get('rtk_label', 'NO DATA')} · "
                f"sats={data.get('satellites', '--')} · alt MSL={data.get('altitude_m', '--')} m · "
                f"HDOP/VDOP={data.get('hdop', '--')}/{data.get('vdop', '--')} · "
                f"H/V acc={data.get('h_acc_m', '--')}/{data.get('v_acc_m', '--')} m"
            ),
            "freshness": f"age={_display_number(data.get('age_s'), digits=2, suffix='s')}",
            "contract": str(data.get("position_contract", "WGS84 latitude/longitude degrees; altitude metres")),
            "verdict": str(data.get("state", "NO DATA")),
            "source": str(data.get("source", "MAVLink GPS telemetry")),
        }

    def _refresh_gnss_source_controls(self) -> None:
        selected: dict[str, str] = {}
        for side, combo in self.gnss_source_combos.items():
            source = self._read_selected_source(side)
            selected[side] = source
            index = combo.findData(source)
            if index >= 0 and combo.currentIndex() != index:
                blocked = combo.blockSignals(True)
                combo.setCurrentIndex(index)
                combo.blockSignals(blocked)
        self.gnss_source_summary.setText(
            f"계산 source: P1 {selected.get('left', 'GPS1')} → P2 {selected.get('right', 'GPS1')}"
        )

    def _set_gnss_source_from_workspace(self, side: str) -> None:
        combo = self.gnss_source_combos.get(side)
        if combo is None:
            return
        source = normalize_pixhawk_gnss_source(combo.currentData())
        setter = self._set_source
        if callable(setter):
            setter(side, source)
        self._refresh_gnss_source_controls()

    def _inspect_baseline_value(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._baseline_inspection(key),
                lambda key=key: self._baseline_inspection(key),
            )
        )

    def _inspect_baseline_yaw_value(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._baseline_yaw_inspection(key),
                lambda key=key: self._baseline_yaw_inspection(key),
            )
        )

    def _baseline_yaw_inspection(self, key: str) -> dict[str, object]:
        data = self._read_yaw()
        fields = {
            "baseline_azimuth": ("baseline_azimuth_deg", _display_compass_deg, "baseline_state"),
            "p1_yaw": ("p1_yaw_deg", _display_compass_deg, "p1_state"),
            "baseline_p1": ("baseline_p1_deg", _display_signed_deg, "p1_state"),
            "p2_yaw": ("p2_yaw_deg", _display_compass_deg, "p2_state"),
            "baseline_p2": ("baseline_p2_deg", _display_signed_deg, "p2_state"),
            "p2_minus_p1": ("p2_minus_p1_deg", _display_signed_deg, "state"),
        }
        data_key, formatter, state_key = fields[key]
        return {
            "selection": self.baseline_yaw_values[key].label.text(),
            "value": f"{formatter(data.get(data_key))} deg",
            "freshness": (
                f"baseline={data.get('baseline_state', 'WAITING')} "
                f"P1={data.get('p1_state', 'WAITING')} "
                f"P2={data.get('p2_state', 'WAITING')} · "
                f"GNSS pair Δt={_display_number(data.get('gps_pair_delta_ms'), digits=1, suffix='ms')}"
            ),
            "contract": str(data.get("angle_contract", "horizontal local NED angle")),
            "verdict": str(data.get(state_key, "WAITING")),
            "source": str(data.get("source", "selected GNSS + MAVLink ATTITUDE")),
        }

    def _baseline_inspection(self, key: str) -> dict[str, object]:
        data = self._read_baseline()
        is_frd = key.startswith("p1_body_frd_")
        contract = (
            str(data.get("p1_body_frd_axis_contract", "P1→P2 in P1 body FRD [Forward,Right,Down]; metres"))
            if is_frd
            else str(data.get("axis_contract", "P1→P2 local NED [North,East,Down]; metres"))
        )
        freshness = (
            f"P1 attitude age={data.get('p1_body_frd_attitude_age_s', '--')}s"
            if is_frd
            else f"P1/P2 age={data.get('left_age_s', '--')}/{data.get('right_age_s', '--')}s"
        )
        verdict = data.get("p1_body_frd_state", "WAITING") if is_frd else data.get("state", "UNKNOWN")
        return {
            "selection": self.baseline_values[key].label.text(),
            "value": f"{_display_number(data.get(key))}{' m' if key.endswith('_m') else ''}",
            "freshness": freshness,
            "contract": contract,
            "verdict": verdict,
            "source": data.get("p1_body_frd_source" if is_frd else "source", "baseline evidence"),
        }
