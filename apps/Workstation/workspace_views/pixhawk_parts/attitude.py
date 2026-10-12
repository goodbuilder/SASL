"""P1/P2 attitude table; body FRD to world NED orientation is producer-owned."""

from __future__ import annotations

from PyQt5 import QtCore, QtWidgets
from ..shared import _display_number, _live_inspection, _sync_table_rows, _table_model


class AttitudeView(QtCore.QObject):
    inspection_requested = QtCore.pyqtSignal(object)

    def __init__(self, *, read_attitude, parent=None):
        super().__init__(parent)
        if not callable(read_attitude):
            raise TypeError("Attitude view requires its evidence reader")
        self._read_attitude = read_attitude
        attitude_group = QtWidgets.QGroupBox("Attitude (R/P/Y)")
        attitude_layout = QtWidgets.QVBoxLayout(attitude_group)
        self.attitude_model = _table_model(
            self,
            ("Pixhawk", "State", "Roll°", "Pitch°", "Yaw°", "Age", "PX4 time", "Endpoint"),
        )
        self.attitude_table = QtWidgets.QTableView()
        self.attitude_table.setModel(self.attitude_model)
        self.attitude_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.attitude_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.attitude_table.verticalHeader().setVisible(False)
        self.attitude_table.setMaximumHeight(112)
        self.attitude_table.horizontalHeader().setStretchLastSection(True)
        self.attitude_table.clicked.connect(self._inspect_attitude_row)
        attitude_layout.addWidget(self.attitude_table)
        self.group = attitude_group
        if isinstance(parent, QtWidgets.QWidget):
            self.group.setParent(parent)

    def refresh(self):
        rows = []
        for side, label in (("left", "P1"), ("right", "P2")):
            attitude = self._read_attitude(side)
            rows.append(
                (
                    label,
                    attitude.get("state", "WAITING"),
                    _display_number(attitude.get("roll_deg")),
                    _display_number(attitude.get("pitch_deg")),
                    _display_number(attitude.get("yaw_deg")),
                    _display_number(attitude.get("age_s"), digits=2, suffix="s"),
                    _display_number(attitude.get("px4_timestamp_s"), digits=6),
                    attitude.get("mavlink_endpoint", "--") or "--",
                )
            )
        _sync_table_rows(self.attitude_model, tuple(rows))

    def _inspect_attitude_row(self, index: QtCore.QModelIndex) -> None:
        side = "left" if index.row() == 0 else "right"
        self.inspection_requested.emit(
            _live_inspection(
                self._attitude_inspection(side),
                lambda side=side: self._attitude_inspection(side),
            )
        )

    def _attitude_inspection(self, side: str) -> dict[str, object]:
        label = "P1" if side == "left" else "P2"
        data = self._read_attitude(side)
        return {
            "selection": f"{label} attitude",
            "value": (
                f"R/P/Y={_display_number(data.get('roll_deg'))}/"
                f"{_display_number(data.get('pitch_deg'))}/{_display_number(data.get('yaw_deg'))}°"
            ),
            "freshness": f"age={_display_number(data.get('age_s'), digits=2, suffix='s')} · PX4 t={data.get('px4_timestamp_s', '--')}",
            "hide_freshness": True,
            "contract": str(data.get("orientation_contract", "PX4 body FRD → world NED; R/P/Y degrees")),
            "verdict": str(data.get("state", "WAITING")),
            "source": str(data.get("source", "MAVLink attitude telemetry")),
        }
