from __future__ import annotations

import math
import time
from dataclasses import replace

from PyQt5 import QtWidgets

from Workstation.main_window import WorkstationMainWindow
from Workstation.tests.workstation_test_support import _FakeBackend
from Workstation.workstation_core.pixhawk_attitude import CameraExtrinsicState
from Workstation.workstation_core.rtk_geometry import EARTH_RADIUS_M, GpsPositionState


def _gps(
    side: str,
    *,
    north_m: float,
    east_m: float,
    down_m: float,
    timestamp_s: float,
) -> GpsPositionState:
    latitude_deg = 37.0
    return GpsPositionState(
        camera=side,
        namespace=side,
        timestamp_s=timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=latitude_deg + math.degrees(north_m / EARTH_RADIUS_M),
        longitude_deg=127.0
        + math.degrees(
            east_m / (EARTH_RADIUS_M * math.cos(math.radians(latitude_deg)))
        ),
        altitude_m=100.0 - down_m,
        fix_type=6,
        satellites_used=20,
    )


def _attitude(side: str, *, timestamp_s: float) -> CameraExtrinsicState:
    return CameraExtrinsicState(
        camera=side,
        namespace=side,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        timestamp_s=timestamp_s,
        topic="mavlink/ATTITUDE",
        q=(1.0, 0.0, 0.0, 0.0),
    )


def _close_window(window: WorkstationMainWindow) -> None:
    app = QtWidgets.QApplication.instance()
    window.close()
    if app is not None:
        app.processEvents()
    window.deleteLater()
    if app is not None:
        app.processEvents()


def _backend_with_geometry() -> _FakeBackend:
    now_s = time.monotonic()
    backend = _FakeBackend()
    backend.panel.latest_gps = {
        "left": _gps("left", north_m=0.0, east_m=0.0, down_m=0.0, timestamp_s=now_s),
        "right": _gps("right", north_m=10.0, east_m=0.0, down_m=2.0, timestamp_s=now_s),
        "drone": _gps("drone", north_m=0.0, east_m=10.0, down_m=-3.0, timestamp_s=now_s),
    }
    backend.panel._selected_rover_gps_state = lambda side: backend.panel.latest_gps.get(side)
    backend.panel._pixhawk_gnss_source = lambda _side: "GPS1"
    backend.panel.latest_extrinsics = {
        "left": _attitude("left", timestamp_s=now_s),
        "right": _attitude("right", timestamp_s=now_s),
    }
    return backend


def test_pixhawk_workspace_shows_dr_vectors_and_two_horizontal_angles() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_backend_with_geometry())
    try:
        window.pixhawk.refresh({})

        assert window.pixhawk.drone_geometry_group.title() == (
            "Drone Pixhawk geometry · F/R/D/Rg"
        )
        assert window.pixhawk.drone_geometry_model.rowCount() == 3
        assert window.pixhawk.drone_geometry_model.item(0, 1).text() == (
            "Dr from Baseline Center"
        )
        assert window.pixhawk.drone_geometry_model.item(1, 1).text() == "Dr from P1"
        assert window.pixhawk.drone_geometry_model.item(2, 1).text() == "Dr from P2"
        assert window.pixhawk.drone_geometry_model.item(0, 2).text() == "-10.000"
        assert window.pixhawk.drone_geometry_model.item(0, 3).text() == "-5.000"
        assert window.pixhawk.drone_geometry_model.item(1, 3).text() == "+10.000"
        assert window.pixhawk.drone_geometry_model.item(2, 2).text() == "-10.000"
        assert window.pixhawk.drone_geometry_angle_values["dr_p1_p2"].value.text() == "90.00"
        assert window.pixhawk.drone_geometry_angle_values["dr_p2_p1"].value.text() == "45.00"
        assert window.pixhawk.drone_geometry_angle_values["dr_p1_p2"].unit.text() == "°"
        assert window.pixhawk.drone_geometry_status_label.text().startswith("CURRENT")
    finally:
        _close_window(window)


def test_pixhawk_dr_geometry_keeps_layout_and_scroll_stable_on_live_refresh() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _backend_with_geometry()
    window = WorkstationMainWindow(backend=backend)
    try:
        window.show()
        window.navigation.select_context("devices.pixhawk")
        app.processEvents()
        container = window.workspace_host.currentWidget()
        scroll_bar = container.verticalScrollBar()
        scroll_bar.setValue(min(120, scroll_bar.maximum()))
        app.processEvents()
        before = {
            "group_height": window.pixhawk.drone_geometry_group.height(),
            "table_height": window.pixhawk.drone_geometry_table.height(),
            "scroll": scroll_bar.value(),
        }

        now_s = time.monotonic()
        backend.panel.latest_gps = {
            "left": _gps(
                "left",
                north_m=12_345.678,
                east_m=-23_456.789,
                down_m=34_567.891,
                timestamp_s=now_s,
            ),
            "right": _gps(
                "right",
                north_m=-45_678.901,
                east_m=56_789.012,
                down_m=-67_890.123,
                timestamp_s=now_s,
            ),
            "drone": _gps(
                "drone",
                north_m=78_901.234,
                east_m=-89_012.345,
                down_m=90_123.456,
                timestamp_s=now_s,
            ),
        }
        backend.panel.latest_extrinsics = {
            "left": _attitude("left", timestamp_s=now_s),
            "right": _attitude("right", timestamp_s=now_s),
        }
        window.pixhawk.refresh({})
        app.processEvents()

        assert window.pixhawk.drone_geometry_group.height() == before["group_height"]
        assert window.pixhawk.drone_geometry_table.height() == before["table_height"]
        assert scroll_bar.value() == before["scroll"]
        assert window.pixhawk.drone_geometry_model.item(0, 2).text() != "--"
    finally:
        _close_window(window)


def test_pixhawk_dr_geometry_shows_partial_and_stale_source_states() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _backend_with_geometry()
    window = WorkstationMainWindow(backend=backend)
    try:
        backend.panel.latest_extrinsics.pop("right")
        window.pixhawk.refresh({})
        app.processEvents()

        assert window.pixhawk.drone_geometry_status_label.text().startswith("PARTIAL")
        assert window.pixhawk.drone_geometry_model.item(1, 0).text() == "CURRENT"
        assert window.pixhawk.drone_geometry_model.item(2, 0).text() == "WAITING"
        assert window.pixhawk.drone_geometry_angle_values["dr_p1_p2"].value.text() == "90.00"

        now_s = time.monotonic()
        backend.panel.latest_extrinsics["right"] = replace(
            _attitude("right", timestamp_s=now_s - 3.0),
            timestamp_s=now_s - 3.0,
        )
        window.pixhawk.refresh({})
        app.processEvents()

        assert window.pixhawk.drone_geometry_status_label.text().startswith("PARTIAL")
        assert window.pixhawk.drone_geometry_model.item(2, 0).text() == "STALE"
        assert window.pixhawk.drone_geometry_model.item(2, 2).text() != "--"
    finally:
        _close_window(window)
