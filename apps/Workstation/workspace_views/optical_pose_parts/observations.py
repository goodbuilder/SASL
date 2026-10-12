"""Captured RTK history, target bindings, selection and detail widgets.

WGS84 degrees, MSL/ellipsoid metres and capture clocks retain producer labels;
this component never reads live telemetry or solves camera geometry."""

from __future__ import annotations

import time
from collections.abc import Mapping
from PyQt5 import QtCore, QtWidgets
from Workstation.ui.components import EngineeringValue


def _format_drone_rtk_sync_delta_ms(value: object) -> str:
    if value is None:
        return "--"
    try:
        return f"{float(value) * 1000.0:+.0f} ms"
    except (TypeError, ValueError):
        return "--"


def _format_drone_rtk_degrees(value: object) -> str:
    if value is None:
        return "--"
    try:
        # Display-only rounding; the stored observation float is untouched.
        return f"{float(value):.9f}°"
    except (TypeError, ValueError):
        return "--"


def _format_drone_rtk_metres(value: object) -> str:
    if value is None:
        return "--"
    try:
        return f"{float(value):.3f} m"
    except (TypeError, ValueError):
        return "--"


def _format_drone_rtk_seconds(value: object) -> str:
    if value is None:
        return "--"
    try:
        return f"{float(value):.3f} s"
    except (TypeError, ValueError):
        return "--"


def _drone_rtk_gnss_label(row: object) -> str:
    """ "NO DATA" unless this GNSS row actually carries a position fix."""

    if not isinstance(row, Mapping) or not row.get("has_position"):
        return "NO DATA"
    return str(row.get("rtk_label") or "UNKNOWN")


class RtkObservations(QtCore.QObject):
    information_changed = QtCore.pyqtSignal()
    DRONE_RTK_OBSERVATION_HEADERS = (
        "#",
        "State",
        "Drone RTK",
        "P1 RTK",
        "P2 RTK",
        "Sync L",
        "Sync R",
        "Target",
    )

    def __init__(self, parent=None):
        super().__init__(parent)
        observations_group = QtWidgets.QGroupBox("Drone RTK Capture Observations")
        self.observations_group = observations_group
        observations_group.setObjectName("opticalPoseDroneRtkObservationsGroup")
        observations_layout = QtWidgets.QVBoxLayout(observations_group)
        observations_layout.setContentsMargins(6, 6, 6, 6)
        observations_layout.setSpacing(5)

        current_observation_header = QtWidgets.QLabel("Current Observation")
        current_observation_header.setObjectName("compactContract")
        observations_layout.addWidget(current_observation_header)
        self.current_observation_summary_label = QtWidgets.QLabel("No observation yet")
        self.current_observation_summary_label.setObjectName("statusStrip")
        observations_layout.addWidget(self.current_observation_summary_label)
        current_observation_grid = QtWidgets.QGridLayout()
        current_observation_grid.setSpacing(5)
        self.current_observation_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("drone_rtk", "Drone RTK"),
                ("gnss_age", "GNSS age"),
                ("sync", "Sync"),
                ("target", "Target"),
            )
        ):
            value = EngineeringValue(label)
            value.set_value("--")
            self.current_observation_values[key] = value
            current_observation_grid.addWidget(value, index // 2, index % 2)
        observations_layout.addLayout(current_observation_grid)

        history_header = QtWidgets.QLabel("Observation History")
        history_header.setObjectName("compactContract")
        observations_layout.addWidget(history_header)
        self.observation_history_table = QtWidgets.QTableWidget(0, len(self.DRONE_RTK_OBSERVATION_HEADERS))
        self.observation_history_table.setObjectName("opticalPoseDroneRtkObservationTable")
        self.observation_history_table.setHorizontalHeaderLabels(self.DRONE_RTK_OBSERVATION_HEADERS)
        self.observation_history_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.observation_history_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.observation_history_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.observation_history_table.verticalHeader().setVisible(False)
        self.observation_history_table.setMaximumHeight(140)
        self.observation_history_table.horizontalHeader().setStretchLastSection(True)
        self.observation_history_table.itemSelectionChanged.connect(self._refresh_drone_rtk_observation_detail)
        observations_layout.addWidget(self.observation_history_table)

        details_header = QtWidgets.QLabel("Selected Observation Details")
        details_header.setObjectName("compactContract")
        observations_layout.addWidget(details_header)
        details_grid = QtWidgets.QGridLayout()
        details_grid.setSpacing(5)
        self.observation_detail_values: dict[str, EngineeringValue] = {}
        for index, (key, label) in enumerate(
            (
                ("observation_id", "Observation ID"),
                ("observation_state", "State"),
                ("observation_burst_id", "Burst ID"),
                ("drone_latitude", "Drone latitude"),
                ("drone_longitude", "Drone longitude"),
                ("drone_altitude", "Drone altitude"),
                ("drone_ellipsoid_height", "Drone ellipsoid height"),
                ("drone_fix", "Drone fix"),
                ("drone_satellites", "Drone satellites"),
                ("drone_eph_epv", "Drone eph / epv"),
                ("drone_timestamp", "Drone GNSS timestamp"),
                ("p1_latitude", "P1 latitude"),
                ("p1_longitude", "P1 longitude"),
                ("p1_altitude", "P1 altitude"),
                ("p1_ellipsoid_height", "P1 ellipsoid height"),
                ("p1_fix", "P1 fix"),
                ("p1_satellites", "P1 satellites"),
                ("p1_eph_epv", "P1 eph / epv"),
                ("p1_timestamp", "P1 GNSS timestamp"),
                ("p2_latitude", "P2 latitude"),
                ("p2_longitude", "P2 longitude"),
                ("p2_altitude", "P2 altitude"),
                ("p2_ellipsoid_height", "P2 ellipsoid height"),
                ("p2_fix", "P2 fix"),
                ("p2_satellites", "P2 satellites"),
                ("p2_eph_epv", "P2 eph / epv"),
                ("p2_timestamp", "P2 GNSS timestamp"),
                ("left_frame_id", "Left frame ID"),
                ("left_frame_sha", "Left frame SHA prefix"),
                ("left_frame_timestamp", "Left frame timestamp"),
                ("left_frame_timestamp_reference", "Left timestamp reference"),
                ("right_frame_id", "Right frame ID"),
                ("right_frame_sha", "Right frame SHA prefix"),
                ("right_frame_timestamp", "Right frame timestamp"),
                ("right_frame_timestamp_reference", "Right timestamp reference"),
                ("sync_state", "Sync state"),
                ("sync_delta_left", "Sync delta L"),
                ("sync_delta_right", "Sync delta R"),
                ("sync_reason", "Sync reason"),
            )
        ):
            value = EngineeringValue(label)
            value.set_value("--")
            self.observation_detail_values[key] = value
            details_grid.addWidget(value, index // 2, index % 2)
        observations_layout.addLayout(details_grid)

        self._drone_rtk_observation_rows: dict[str, dict[str, object]] = {}
        self._drone_rtk_observation_order: list[str] = []
        self._drone_rtk_multiview_samples: list[dict[str, object]] = []
        self._drone_rtk_multiview_sample_by_observation: dict[str, dict[str, object]] = {}
        self._drone_rtk_multiview_sample_by_burst: dict[str, dict[str, object]] = {}
        self._drone_rtk_multiview_collected_frame_ids: dict[str, set[str]] = {
            "left": set(),
            "right": set(),
        }
        self.destroyed.connect(self.observations_group.deleteLater)

    def _selected_drone_rtk_observation_id(self) -> str:
        row = self.observation_history_table.currentRow()
        item = self.observation_history_table.item(row, 0) if row >= 0 else None
        return "" if item is None else str(item.data(QtCore.Qt.UserRole) or "")

    def set_drone_rtk_multiview_samples(self, samples: object) -> None:
        """Project frozen-target/RTK bindings without mutating capture rows."""

        rows = (
            [dict(value) for value in samples if isinstance(value, Mapping)]
            if isinstance(samples, (tuple, list))
            else []
        )
        self._drone_rtk_multiview_samples = rows
        self._drone_rtk_multiview_sample_by_observation = {}
        self._drone_rtk_multiview_sample_by_burst = {}
        self._drone_rtk_multiview_collected_frame_ids = {
            "left": set(),
            "right": set(),
        }
        for sample in rows:
            observation_id = str(sample.get("capture_observation_id") or "")
            burst_id = str(sample.get("burst_id") or "")
            if observation_id:
                self._drone_rtk_multiview_sample_by_observation[observation_id] = sample
            if burst_id:
                self._drone_rtk_multiview_sample_by_burst[burst_id] = sample
            if str(sample.get("state") or "").upper() != "READY":
                # A BLOCKED sample (e.g. GNSS sync not yet verified) is
                # recorded for auditability, but its frame must not be
                # treated as collected -- the operator's marks need to
                # survive so the pair can be retried on the next capture.
                continue
            annotations = sample.get("annotations")
            annotations = annotations if isinstance(annotations, Mapping) else {}
            for side in ("left", "right"):
                annotation = annotations.get(side)
                if isinstance(annotation, Mapping):
                    frame_id = str(annotation.get("frame_id") or "")
                    if frame_id:
                        self._drone_rtk_multiview_collected_frame_ids[side].add(frame_id)

        # A snapshot may update only the derived sample projection. Repaint
        # existing immutable observation rows so Target status stays current.
        if self._drone_rtk_observation_order:
            self.set_drone_rtk_observations(
                [
                    self._drone_rtk_observation_rows[observation_id]
                    for observation_id in self._drone_rtk_observation_order
                    if observation_id in self._drone_rtk_observation_rows
                ]
            )
        else:
            self.information_changed.emit()

    def _drone_rtk_target_state(self, observation: Mapping[str, object]) -> str:
        observation_id = str(observation.get("observation_id") or "")
        burst_id = str(observation.get("burst_id") or "")
        sample = self._drone_rtk_multiview_sample_by_observation.get(observation_id)
        if sample is None:
            sample = self._drone_rtk_multiview_sample_by_burst.get(burst_id)
        if sample is None:
            return "NOT ANNOTATED"
        return str(sample.get("state") or "ANNOTATED")

    def set_drone_rtk_observations(self, observations: object) -> None:
        """Refresh Observation History from an append-only, immutable list.

        Row identity is the observation_id (mirrors ``set_candidates``'
        candidate_id pattern) so a selected row survives a snapshot poll
        instead of resetting to row 0 or vanishing when a new observation is
        appended.
        """

        rows = (
            [dict(value) for value in observations if isinstance(value, Mapping)]
            if isinstance(observations, (tuple, list))
            else []
        )
        previously_selected = self._selected_drone_rtk_observation_id()
        self._drone_rtk_observation_rows = {}
        self._drone_rtk_observation_order = []
        table = self.observation_history_table
        table.blockSignals(True)
        try:
            table.setRowCount(len(rows))
            selected_row = -1
            for row_index, observation in enumerate(rows):
                observation_id = str(observation.get("observation_id") or "")
                if not observation_id:
                    observation_id = f"observation-{row_index + 1:03d}"
                self._drone_rtk_observation_rows[observation_id] = observation
                self._drone_rtk_observation_order.append(observation_id)
                gnss = observation.get("gnss")
                gnss = gnss if isinstance(gnss, Mapping) else {}
                sync = observation.get("synchronization")
                sync = sync if isinstance(sync, Mapping) else {}
                sequence = observation.get("sequence")
                display_index = (
                    f"{int(sequence) + 1:03d}" if isinstance(sequence, (int, float)) else f"{row_index + 1:03d}"
                )
                values = (
                    display_index,
                    str(observation.get("state") or "--"),
                    _drone_rtk_gnss_label(gnss.get("drone")),
                    _drone_rtk_gnss_label(gnss.get("left")),
                    _drone_rtk_gnss_label(gnss.get("right")),
                    _format_drone_rtk_sync_delta_ms(sync.get("left_frame_to_drone_gnss_s")),
                    _format_drone_rtk_sync_delta_ms(sync.get("right_frame_to_drone_gnss_s")),
                    self._drone_rtk_target_state(observation),
                )
                for column, value in enumerate(values):
                    item = QtWidgets.QTableWidgetItem(str(value))
                    item.setData(QtCore.Qt.UserRole, observation_id)
                    table.setItem(row_index, column, item)
                if observation_id == previously_selected:
                    selected_row = row_index
            if selected_row < 0 and rows:
                # Default to the most recent capture, not row 0, since this
                # history is append-only and the newest entry is what an
                # operator just produced.
                selected_row = len(rows) - 1
            if selected_row >= 0:
                table.selectRow(selected_row)
                table.setCurrentCell(selected_row, 0)
            else:
                table.clearSelection()
        finally:
            table.blockSignals(False)
        self._refresh_drone_rtk_observation_detail()
        self.information_changed.emit()

    def _refresh_drone_rtk_observation_detail(self) -> None:
        latest_id = self._drone_rtk_observation_order[-1] if self._drone_rtk_observation_order else ""
        self._apply_current_drone_rtk_observation_summary(self._drone_rtk_observation_rows.get(latest_id))
        selected_id = self._selected_drone_rtk_observation_id() or latest_id
        self._apply_selected_drone_rtk_observation_detail(self._drone_rtk_observation_rows.get(selected_id))

    def _apply_current_drone_rtk_observation_summary(self, observation: object) -> None:
        if not isinstance(observation, Mapping):
            self.current_observation_summary_label.setText("No observation yet")
            for value in self.current_observation_values.values():
                value.set_value("--")
            return
        sequence = observation.get("sequence")
        display_index = f"#{int(sequence) + 1:03d}" if isinstance(sequence, (int, float)) else "#--"
        state = str(observation.get("state") or "--")
        self.current_observation_summary_label.setText(f"{display_index} · {state}")
        gnss = observation.get("gnss")
        gnss = gnss if isinstance(gnss, Mapping) else {}
        sync = observation.get("synchronization")
        sync = sync if isinstance(sync, Mapping) else {}
        drone = gnss.get("drone")
        self.current_observation_values["drone_rtk"].set_value(_drone_rtk_gnss_label(drone))
        age_text = "--"
        if isinstance(drone, Mapping) and drone.get("timestamp_s") is not None:
            age_s = max(0.0, time.monotonic() - float(drone["timestamp_s"]))
            age_text = f"{age_s:.2f} s"
        self.current_observation_values["gnss_age"].set_value(age_text)
        left_delta = _format_drone_rtk_sync_delta_ms(sync.get("left_frame_to_drone_gnss_s"))
        right_delta = _format_drone_rtk_sync_delta_ms(sync.get("right_frame_to_drone_gnss_s"))
        self.current_observation_values["sync"].set_value(f"L {left_delta} / R {right_delta}")
        self.current_observation_values["target"].set_value(self._drone_rtk_target_state(observation))

    @staticmethod
    def _apply_gnss_detail_rows(
        values: dict[str, "EngineeringValue"],
        *,
        prefix: str,
        row: object,
    ) -> None:
        row = row if isinstance(row, Mapping) else {}
        values[f"{prefix}_latitude"].set_value(_format_drone_rtk_degrees(row.get("latitude_deg")))
        values[f"{prefix}_longitude"].set_value(_format_drone_rtk_degrees(row.get("longitude_deg")))
        values[f"{prefix}_altitude"].set_value(_format_drone_rtk_metres(row.get("altitude_m")))
        values[f"{prefix}_ellipsoid_height"].set_value(_format_drone_rtk_metres(row.get("altitude_ellipsoid_m")))
        values[f"{prefix}_fix"].set_value(_drone_rtk_gnss_label(row))
        satellites = row.get("satellites_used")
        values[f"{prefix}_satellites"].set_value("--" if satellites is None else str(satellites))
        eph = _format_drone_rtk_metres(row.get("eph_m"))
        epv = _format_drone_rtk_metres(row.get("epv_m"))
        values[f"{prefix}_eph_epv"].set_value("--" if eph == "--" and epv == "--" else f"{eph} / {epv}")
        values[f"{prefix}_timestamp"].set_value(_format_drone_rtk_seconds(row.get("timestamp_s")))

    @staticmethod
    def _apply_frame_detail_rows(
        values: dict[str, "EngineeringValue"],
        *,
        prefix: str,
        frame: object,
    ) -> None:
        frame = frame if isinstance(frame, Mapping) else {}
        values[f"{prefix}_frame_id"].set_value(frame.get("frame_id") or "--")
        sha = str(frame.get("sha256") or "")
        values[f"{prefix}_frame_sha"].set_value(sha[:12] if sha else "--")
        values[f"{prefix}_frame_timestamp"].set_value(_format_drone_rtk_seconds(frame.get("timestamp_s")))
        values[f"{prefix}_frame_timestamp_reference"].set_value(frame.get("timestamp_reference") or "--")

    def _apply_selected_drone_rtk_observation_detail(self, observation: object) -> None:
        values = self.observation_detail_values
        if not isinstance(observation, Mapping):
            for value in values.values():
                value.set_value("--")
            return
        values["observation_id"].set_value(observation.get("observation_id") or "--")
        values["observation_state"].set_value(observation.get("state") or "--")
        values["observation_burst_id"].set_value(observation.get("burst_id") or "--")
        gnss = observation.get("gnss")
        gnss = gnss if isinstance(gnss, Mapping) else {}
        self._apply_gnss_detail_rows(values, prefix="drone", row=gnss.get("drone"))
        self._apply_gnss_detail_rows(values, prefix="p1", row=gnss.get("left"))
        self._apply_gnss_detail_rows(values, prefix="p2", row=gnss.get("right"))
        frames = observation.get("frames")
        frames = frames if isinstance(frames, Mapping) else {}
        self._apply_frame_detail_rows(values, prefix="left", frame=frames.get("left"))
        self._apply_frame_detail_rows(values, prefix="right", frame=frames.get("right"))
        sync = observation.get("synchronization")
        sync = sync if isinstance(sync, Mapping) else {}
        values["sync_state"].set_value(sync.get("state") or "--")
        values["sync_delta_left"].set_value(_format_drone_rtk_sync_delta_ms(sync.get("left_frame_to_drone_gnss_s")))
        values["sync_delta_right"].set_value(_format_drone_rtk_sync_delta_ms(sync.get("right_frame_to_drone_gnss_s")))
        values["sync_reason"].set_value(sync.get("reason") or "--")

    def collected_frame_ids(self, side: str) -> frozenset[str]:
        return frozenset(self._drone_rtk_multiview_collected_frame_ids.get(side, ()))

    def information_snapshot(self):
        return len(self._drone_rtk_observation_order), tuple(dict(row) for row in self._drone_rtk_multiview_samples)
