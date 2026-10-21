"""GPS1/GPS2 selection, WGS84 receiver values and NED/body-FRD baseline display."""

from __future__ import annotations

from collections.abc import Mapping
from PyQt5 import QtCore, QtWidgets
from .designer import load_pixhawk_form
from ..shared import _display_number, _live_inspection, _sync_table_rows, _table_model

import math
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
        self._designer_form = load_pixhawk_form("pixhawk_gnss.ui")
        form = self._designer_form
        self.receiver_table = form.receiver_table
        self.receiver_group = form.receiver_group
        self.baseline_values = {'north_m': form.baseline_values_north_m, 'east_m': form.baseline_values_east_m, 'down_m': form.baseline_values_down_m, 'length_m': form.baseline_values_length_m, 'p1_body_frd_forward_m': form.baseline_values_p1_body_frd_forward_m, 'p1_body_frd_right_m': form.baseline_values_p1_body_frd_right_m, 'p1_body_frd_down_m': form.baseline_values_p1_body_frd_down_m, 'state': form.baseline_values_state}
        self.baseline_group = form.baseline_group
        self.baseline_yaw_status_label = form.baseline_yaw_status_label
        self.baseline_yaw_values = {'baseline_azimuth': form.baseline_yaw_values_baseline_azimuth, 'p2_minus_p1': form.baseline_yaw_values_p2_minus_p1, 'p1_yaw': form.baseline_yaw_values_p1_yaw, 'baseline_p1': form.baseline_yaw_values_baseline_p1, 'p2_yaw': form.baseline_yaw_values_p2_yaw, 'baseline_p2': form.baseline_yaw_values_baseline_p2}
        self.baseline_yaw_evidence_label = form.baseline_yaw_evidence_label
        self.baseline_yaw_contract_label = form.baseline_yaw_contract_label
        self.baseline_yaw_plot = form.baseline_yaw_plot
        self.baseline_yaw_group = form.baseline_yaw_group
        self.gnss_source_combos = {'left': form.gnss_source_combos_left, 'right': form.gnss_source_combos_right}
        self.gnss_source_summary = form.gnss_source_summary
        self.gnss_source_group = form.gnss_source_group
        self.receiver_model = _table_model(self, ('Receiver', 'Fix number', 'RTK state', 'Satellites', 'Altitude MSL m', 'Altitude ellipsoid m', 'HDOP/VDOP', 'H/V acc m', 'Age'))
        self.receiver_table.setModel(self.receiver_model)
        header = form.receiver_table.horizontalHeader()
        for column, (width, mode) in enumerate(((104, 0), (104, 0), (104, 0), (104, 0), (104, 0), (104, 0), (104, 0), (104, 0), (104, 0))):
            header.setSectionResizeMode(column, QtWidgets.QHeaderView.ResizeMode(mode))
            header.resizeSection(column, width)
        self.receiver_table.clicked.connect(self._inspect_receiver_row)
        self._receiver_row_keys: tuple[tuple[str, str], ...] = ()
        for key, value in self.baseline_values.items():
            value.activated.connect(lambda key=key: self._inspect_baseline_value(key))
        for key, value in self.baseline_yaw_values.items():
            value.value.setMinimumWidth(value.value.fontMetrics().horizontalAdvance("-180.00"))
            value.activated.connect(lambda key=key: self._inspect_baseline_yaw_value(key))
        for side, combo in self.gnss_source_combos.items():
            combo.currentIndexChanged.connect(lambda _index, side=side: self._set_gnss_source_from_workspace(side))
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
