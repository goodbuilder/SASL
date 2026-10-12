"""Geometry projection rendering only; no NED/FRD rotation or GNSS arithmetic."""

from __future__ import annotations

from collections.abc import Mapping
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import EngineeringValue
from ..shared import _compact_text, _display_number, _live_inspection, _sync_table_rows, _table_model

from .formatting import _display_signed_number


class GeometryView(QtCore.QObject):
    inspection_requested = QtCore.pyqtSignal(object)

    def __init__(self, *, read_geometry, parent=None):
        super().__init__(parent)
        if not callable(read_geometry):
            raise TypeError("Geometry view requires its projection reader")
        self._read_geometry = read_geometry
        drone_geometry_group = QtWidgets.QGroupBox("Drone Pixhawk geometry · F/R/D/Rg")
        drone_geometry_layout = QtWidgets.QVBoxLayout(drone_geometry_group)
        drone_geometry_layout.setContentsMargins(8, 8, 8, 8)
        drone_geometry_layout.setSpacing(5)
        drone_geometry_hint = QtWidgets.QLabel(
            "Dr = Drone Pixhawk · Dr from reference means reference→Dr · F/R/D/Rg = Forward/Right/Down/Range (m)"
        )
        drone_geometry_hint.setObjectName("statusStrip")
        drone_geometry_hint.setWordWrap(True)
        drone_geometry_layout.addWidget(drone_geometry_hint)
        self.drone_geometry_model = _table_model(
            self,
            ("State", "Vector", "F (m)", "R (m)", "D (m)", "Rg (m)", "Frame", "Age"),
        )
        self.drone_geometry_table = QtWidgets.QTableView()
        self.drone_geometry_table.setModel(self.drone_geometry_model)
        self.drone_geometry_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.drone_geometry_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.drone_geometry_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.drone_geometry_table.setAlternatingRowColors(True)
        self.drone_geometry_table.verticalHeader().setVisible(False)
        self.drone_geometry_table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        self.drone_geometry_table.setMinimumHeight(118)
        self.drone_geometry_table.setMaximumHeight(142)
        geometry_header = self.drone_geometry_table.horizontalHeader()
        geometry_header.setStretchLastSection(False)
        geometry_header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        geometry_header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        for column in (2, 3, 4, 5):
            geometry_header.setSectionResizeMode(column, QtWidgets.QHeaderView.Fixed)
            geometry_header.resizeSection(column, 86)
        geometry_header.setSectionResizeMode(6, QtWidgets.QHeaderView.Stretch)
        geometry_header.setSectionResizeMode(7, QtWidgets.QHeaderView.ResizeToContents)
        self.drone_geometry_table.clicked.connect(self._inspect_drone_geometry_vector)
        drone_geometry_layout.addWidget(self.drone_geometry_table)

        drone_angle_title = QtWidgets.QLabel("Horizontal triangle angles (F/R only)")
        drone_angle_title.setObjectName("compactContract")
        drone_geometry_layout.addWidget(drone_angle_title)
        angle_layout = QtWidgets.QGridLayout()
        angle_layout.setContentsMargins(0, 0, 0, 0)
        angle_layout.setHorizontalSpacing(12)
        angle_layout.setVerticalSpacing(3)
        self.drone_geometry_angle_values: dict[str, EngineeringValue] = {}
        for column, (key, label) in enumerate((("dr_p1_p2", "∠Dr-P1-P2"), ("dr_p2_p1", "∠Dr-P2-P1"))):
            value = EngineeringValue(label)
            value.value.setMinimumWidth(value.value.fontMetrics().horizontalAdvance("180.00"))
            value.activated.connect(lambda key=key: self._inspect_drone_geometry_angle(key))
            self.drone_geometry_angle_values[key] = value
            angle_layout.addWidget(value, 0, column)
            angle_layout.setColumnStretch(column, 1)
        drone_geometry_layout.addLayout(angle_layout)
        self.drone_geometry_status_label = QtWidgets.QLabel("WAITING · geometry evidence 대기 중")
        self.drone_geometry_status_label.setObjectName("statusStrip")
        self.drone_geometry_status_label.setFixedHeight(32)
        self.drone_geometry_status_label.setWordWrap(False)
        drone_geometry_layout.addWidget(self.drone_geometry_status_label)
        self.drone_geometry_contract_label = QtWidgets.QLabel(
            "Angles: ∠Dr-P1-P2 / ∠Dr-P2-P1 · horizontal North/East only · unsigned internal angle"
        )
        self.drone_geometry_contract_label.setObjectName("compactContract")
        self.drone_geometry_contract_label.setWordWrap(True)
        self.drone_geometry_contract_label.setFixedHeight(32)
        drone_geometry_layout.addWidget(self.drone_geometry_contract_label)
        self.drone_geometry_group = drone_geometry_group
        self.group = drone_geometry_group
        if isinstance(parent, QtWidgets.QWidget):
            self.group.setParent(parent)

    def refresh(self):
        self._refresh_drone_pixhawk_geometry(self._read_geometry())

    def _refresh_drone_pixhawk_geometry(self, data: Mapping[str, object]) -> None:
        state = str(data.get("state") or "WAITING")
        reason = str(data.get("reason") or "geometry evidence waiting")
        vector_rows = tuple(row for row in (data.get("vectors") or ()) if isinstance(row, Mapping))
        table_rows = tuple(
            (
                str(row.get("state") or "WAITING"),
                str(row.get("label") or "--"),
                _display_signed_number(row.get("forward_m")),
                _display_signed_number(row.get("right_m")),
                _display_signed_number(row.get("down_m")),
                _display_number(row.get("range_m")),
                str(row.get("frame") or "--"),
                _display_number(row.get("age_s"), digits=2, suffix="s"),
            )
            for row in vector_rows
        )
        _sync_table_rows(self.drone_geometry_model, table_rows)
        for row_index, row in enumerate(vector_rows):
            item = self.drone_geometry_model.item(row_index, 1)
            if item is None:
                continue
            item.setToolTip(
                "\n".join(
                    (
                        str(row.get("label") or "--"),
                        f"State: {row.get('state', 'WAITING')}",
                        (
                            "F/R/D/Rg="
                            f"{_display_signed_number(row.get('forward_m'))}/"
                            f"{_display_signed_number(row.get('right_m'))}/"
                            f"{_display_signed_number(row.get('down_m'))}/"
                            f"{_display_number(row.get('range_m'))} m"
                        ),
                        f"Frame: {row.get('frame', '--')}",
                        f"From: {row.get('source', '--')}",
                        f"Reason: {row.get('reason', '--')}",
                    )
                )
            )
            for column in range(self.drone_geometry_model.columnCount()):
                cell = self.drone_geometry_model.item(row_index, column)
                if cell is not None:
                    cell.setToolTip(item.toolTip())

        status_text = f"{state} · {reason}"
        status_tooltip = (
            f"{status_text}\n"
            f"P1 source={data.get('p1_gps_source', 'GPS1')} "
            f"age={_display_number(data.get('p1_gps_age_s'), digits=2, suffix='s')}\n"
            f"P2 source={data.get('p2_gps_source', 'GPS1')} "
            f"age={_display_number(data.get('p2_gps_age_s'), digits=2, suffix='s')}\n"
            f"Dr source=Drone GNSS age={_display_number(data.get('drone_gps_age_s'), digits=2, suffix='s')}\n"
            f"Source: {data.get('source', '')}\n"
            f"Clock: {data.get('clock', '')}"
        )
        self.drone_geometry_status_label.setText(_compact_text(status_text, limit=160))
        self.drone_geometry_status_label.setToolTip(status_tooltip)
        self.drone_geometry_contract_label.setToolTip(
            f"{data.get('frame_contract', '')}\n{data.get('angle_contract', '')}\n"
            "Latest-state display; GNSS/ATTITUDE samples are not simultaneous measurement evidence."
        )

        angle_rows = {str(row.get("key")): row for row in (data.get("angles") or ()) if isinstance(row, Mapping)}
        for key, value in self.drone_geometry_angle_values.items():
            row = angle_rows.get(key, {})
            value.set_value(
                _display_number(row.get("angle_deg"), digits=2),
                unit="°",
                source=str(row.get("source") or data.get("source") or ""),
                age_s=row.get("age_s"),
                frame=str(row.get("frame") or "horizontal local NED (North/East only)"),
            )
            angle_tooltip = "\n".join(
                (
                    f"State: {row.get('state', 'WAITING')}",
                    f"Value: {_display_number(row.get('angle_deg'), digits=2)}°",
                    f"Vertex: {row.get('vertex', '--')}",
                    f"Frame: {row.get('frame', 'horizontal local NED (North/East only)')}",
                    f"From: {row.get('source', data.get('source', '--'))}",
                    f"Reason: {row.get('reason', reason)}",
                    f"Age: {_display_number(row.get('age_s'), digits=2, suffix='s')}",
                )
            )
            value.setToolTip(angle_tooltip)
            value.value.setToolTip(angle_tooltip)

    def _inspect_drone_geometry_vector(self, index: QtCore.QModelIndex) -> None:
        if index.row() < 0:
            return
        data = self._read_geometry()
        rows = tuple(row for row in (data.get("vectors") or ()) if isinstance(row, Mapping))
        if index.row() >= len(rows):
            return
        key = str(rows[index.row()].get("key") or "")
        self.inspection_requested.emit(
            _live_inspection(
                self._drone_geometry_vector_inspection(key),
                lambda key=key: self._drone_geometry_vector_inspection(key),
            )
        )

    def _drone_geometry_vector_inspection(self, key: str) -> dict[str, object]:
        data = self._read_geometry()
        rows = tuple(row for row in (data.get("vectors") or ()) if isinstance(row, Mapping))
        row = next((row for row in rows if str(row.get("key")) == key), {})
        return {
            "selection": str(row.get("label") or key),
            "value": (
                "F/R/D/Rg="
                f"{_display_signed_number(row.get('forward_m'))}/"
                f"{_display_signed_number(row.get('right_m'))}/"
                f"{_display_signed_number(row.get('down_m'))}/"
                f"{_display_number(row.get('range_m'))} m"
            ),
            "freshness": (
                f"state={row.get('state', 'WAITING')} · age={_display_number(row.get('age_s'), digits=2, suffix='s')}"
            ),
            "contract": (
                f"frame={row.get('frame', '--')} · vector direction is named reference → Dr; F/R/D/Rg are metres"
            ),
            "verdict": str(row.get("state", "WAITING")),
            "source": str(row.get("source", data.get("source", "geometry evidence"))),
        }

    def _inspect_drone_geometry_angle(self, key: str) -> None:
        self.inspection_requested.emit(
            _live_inspection(
                self._drone_geometry_angle_inspection(key),
                lambda key=key: self._drone_geometry_angle_inspection(key),
            )
        )

    def _drone_geometry_angle_inspection(self, key: str) -> dict[str, object]:
        data = self._read_geometry()
        rows = {str(row.get("key")): row for row in (data.get("angles") or ()) if isinstance(row, Mapping)}
        row = rows.get(key, {})
        return {
            "selection": str(row.get("label") or key),
            "value": f"{_display_number(row.get('angle_deg'), digits=2)}°",
            "freshness": (
                f"state={row.get('state', 'WAITING')} · age={_display_number(row.get('age_s'), digits=2, suffix='s')}"
            ),
            "contract": str(row.get("frame") or data.get("angle_contract") or "horizontal F/R angle"),
            "verdict": str(row.get("state", "WAITING")),
            "source": str(row.get("source", data.get("source", "geometry evidence"))),
        }
