"""Pixhawk Baseline-P1/P2 read-only UI integration tests."""

from __future__ import annotations

import math
import time

import pytest
from PyQt5 import QtCore, QtWidgets

from Workstation.main_window import WorkstationMainWindow
from Workstation.telemetry_plots import BaselineYawAlignmentPlot
from Workstation.tests.workstation_test_support import _FakeBackend
from Workstation.workspaces import _display_compass_deg, _display_signed_deg
from Workstation.workstation_core.pixhawk_attitude import CameraExtrinsicState
from Workstation.workstation_core.rtk_geometry import GpsPositionState, RtkBaselineEstimate


def _attitude(side: str, *, yaw_deg: float, timestamp_s: float) -> CameraExtrinsicState:
    half_yaw = math.radians(yaw_deg) / 2.0
    return CameraExtrinsicState(
        camera=side,
        namespace=side,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=yaw_deg,
        timestamp_s=timestamp_s,
        topic="mavlink/ATTITUDE",
        q=(math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)),
    )


def _gps(side: str, *, timestamp_s: float, longitude_deg: float) -> GpsPositionState:
    return GpsPositionState(
        camera=side,
        namespace=side,
        timestamp_s=timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=37.0,
        longitude_deg=longitude_deg,
        altitude_m=100.0,
        fix_type=6,
        satellites_used=20,
    )


def _backend_with_alignment() -> tuple[_FakeBackend, float]:
    now_s = time.monotonic()
    backend = _FakeBackend()
    backend.panel.latest_rtk_baseline = RtkBaselineEstimate(
        baseline_m=1.0,
        north_m=0.0,
        east_m=1.0,
        down_m=0.0,
        left_age_s=0.02,
        right_age_s=0.01,
        fix_quality="GOOD",
        gps_pair_delta_s=0.01,
    )
    backend.panel.latest_gps = {
        "left": _gps("left", timestamp_s=now_s - 0.02, longitude_deg=127.0),
        "right": _gps("right", timestamp_s=now_s - 0.01, longitude_deg=127.00001),
    }
    backend.panel.latest_extrinsics = {
        "left": _attitude("left", yaw_deg=0.0, timestamp_s=now_s - 0.01),
        "right": _attitude("right", yaw_deg=100.0, timestamp_s=now_s - 0.02),
    }
    backend.panel._pixhawk_gnss_source = lambda _side: "GPS1"
    return backend, now_s


def _close_window(window: WorkstationMainWindow) -> None:
    app = QtWidgets.QApplication.instance()
    window.close()
    if app is not None:
        app.processEvents()
    window.deleteLater()
    if app is not None:
        app.processEvents()


def test_baseline_yaw_plot_maps_north_and_east_clockwise() -> None:
    center = QtCore.QPointF(100.0, 100.0)

    north = BaselineYawAlignmentPlot.heading_endpoint(center, 50.0, 0.0)
    east = BaselineYawAlignmentPlot.heading_endpoint(center, 50.0, 90.0)

    assert north.x() == pytest.approx(100.0)
    assert north.y() == pytest.approx(50.0)
    assert east.x() == pytest.approx(150.0)
    assert east.y() == pytest.approx(100.0)


def test_baseline_yaw_display_formats_keep_rollovers_inside_contract() -> None:
    assert _display_compass_deg(359.999) == "000.00"
    assert _display_compass_deg(-1.0) == "359.00"
    assert _display_signed_deg(179.999) == "-180.00"
    assert _display_signed_deg(-0.0001) == "+000.00"


def test_baseline_yaw_plot_renders_waiting_partial_and_stale_states() -> None:
    plot = BaselineYawAlignmentPlot()
    plot.resize(650, BaselineYawAlignmentPlot.HEIGHT_PX)
    snapshots = (
        {
            "state": "WAITING",
            "baseline_state": "WAITING",
            "p1_state": "WAITING",
            "p2_state": "WAITING",
            "reason": "P1→P2 NED baseline waiting",
        },
        {
            "state": "PARTIAL",
            "baseline_state": "CURRENT",
            "p1_state": "CURRENT",
            "p2_state": "WAITING",
            "baseline_azimuth_deg": 90.0,
            "p1_yaw_deg": 0.0,
        },
        {
            "state": "STALE",
            "baseline_state": "STALE",
            "p1_state": "STALE",
            "p2_state": "STALE",
            "baseline_azimuth_deg": 90.0,
            "p1_yaw_deg": 0.0,
            "p2_yaw_deg": 100.0,
        },
    )

    for snapshot in snapshots:
        plot.set_snapshot(snapshot)
        pixmap = plot.grab()
        assert not pixmap.isNull()
        assert plot.snapshot["state"] == snapshot["state"]


def test_pixhawk_workspace_shows_separate_baseline_p1_and_baseline_p2_degrees() -> None:
    backend, _now_s = _backend_with_alignment()
    window = WorkstationMainWindow(backend=backend)
    try:
        window.pixhawk.refresh({})

        assert window.pixhawk.baseline_yaw_status_label.text().startswith("CURRENT")
        assert window.pixhawk.baseline_yaw_values["baseline_azimuth"].value.text() == "090.00"
        assert window.pixhawk.baseline_yaw_values["p1_yaw"].value.text() == "000.00"
        assert window.pixhawk.baseline_yaw_values["baseline_p1"].value.text() == "+090.00"
        assert window.pixhawk.baseline_yaw_values["p2_yaw"].value.text() == "100.00"
        assert window.pixhawk.baseline_yaw_values["baseline_p2"].value.text() == "-010.00"
        assert window.pixhawk.baseline_yaw_values["p2_minus_p1"].value.text() == "+100.00"
        assert window.pixhawk.baseline_yaw_values["baseline_p1"].label.text() == "Baseline−P1"
        assert window.pixhawk.baseline_yaw_values["baseline_p2"].label.text() == "Baseline−P2"
        assert window.pixhawk.baseline_yaw_plot.snapshot["baseline_p1_deg"] == pytest.approx(90.0)
        assert window.pixhawk.baseline_yaw_plot.snapshot["baseline_p2_deg"] == pytest.approx(-10.0)
    finally:
        _close_window(window)


def test_pixhawk_workspace_keeps_p1_visible_when_p2_is_missing_or_stale() -> None:
    backend, now_s = _backend_with_alignment()
    window = WorkstationMainWindow(backend=backend)
    try:
        backend.panel.latest_extrinsics.pop("right")
        window.pixhawk.refresh({})

        assert window.pixhawk.baseline_yaw_status_label.text().startswith("PARTIAL")
        assert window.pixhawk.baseline_yaw_values["baseline_p1"].value.text() == "+090.00"
        assert window.pixhawk.baseline_yaw_values["baseline_p2"].value.text() == "--"
        assert window.pixhawk.baseline_yaw_plot.snapshot["p2_state"] == "WAITING"

        backend.panel.latest_extrinsics["right"] = _attitude(
            "right",
            yaw_deg=100.0,
            timestamp_s=now_s - 3.0,
        )
        window.pixhawk.refresh({})

        assert window.pixhawk.baseline_yaw_status_label.text().startswith("PARTIAL")
        assert window.pixhawk.baseline_yaw_values["baseline_p1"].value.text() == "+090.00"
        assert window.pixhawk.baseline_yaw_values["baseline_p2"].value.text() == "-010.00"
        assert window.pixhawk.baseline_yaw_plot.snapshot["p2_state"] == "STALE"
    finally:
        _close_window(window)


def test_baseline_yaw_live_updates_keep_reserved_geometry_and_scroll() -> None:
    backend, now_s = _backend_with_alignment()
    window = WorkstationMainWindow(backend=backend)
    app = QtWidgets.QApplication.instance()
    try:
        window.show()
        window.navigation.select_context("devices.pixhawk")
        assert app is not None
        app.processEvents()
        for width, height in ((1366, 768), (1600, 900), (1920, 1080)):
            window.resize(width, height)
            app.processEvents()
            group_rect = window.pixhawk.baseline_yaw_group.contentsRect()
            summary_widget = window.pixhawk.baseline_yaw_group.layout().itemAt(0).widget()
            assert summary_widget is not None
            assert summary_widget.geometry().left() >= group_rect.left()
            assert window.pixhawk.baseline_yaw_plot.geometry().right() <= group_rect.right()
            assert window.pixhawk.baseline_yaw_plot.width() >= 360
        window.resize(1366, 768)
        app.processEvents()
        container = window.workspace_host.currentWidget()
        assert isinstance(container, QtWidgets.QScrollArea)
        scroll_bar = container.verticalScrollBar()
        scroll_bar.setValue(min(100, scroll_bar.maximum()))
        scroll_before = scroll_bar.value()
        group_height = window.pixhawk.baseline_yaw_group.height()
        plot_height = window.pixhawk.baseline_yaw_plot.height()
        status_height = window.pixhawk.baseline_yaw_status_label.height()
        evidence_height = window.pixhawk.baseline_yaw_evidence_label.height()

        for index in range(40):
            yaw = 179.99 if index % 2 else -179.99
            backend.panel.latest_extrinsics["left"] = _attitude(
                "left",
                yaw_deg=yaw,
                timestamp_s=now_s + index * 0.001,
            )
            window.pixhawk.refresh({})
        app.processEvents()

        assert window.pixhawk.baseline_yaw_plot.minimumHeight() == BaselineYawAlignmentPlot.HEIGHT_PX
        assert window.pixhawk.baseline_yaw_plot.maximumHeight() == BaselineYawAlignmentPlot.HEIGHT_PX
        assert window.pixhawk.baseline_yaw_group.height() == group_height
        assert window.pixhawk.baseline_yaw_plot.height() == plot_height
        assert window.pixhawk.baseline_yaw_status_label.height() == status_height == 24
        assert window.pixhawk.baseline_yaw_evidence_label.height() == evidence_height == 42
        assert scroll_bar.value() == scroll_before
        assert window.pixhawk.baseline_yaw_values["baseline_p1"].value.text() in {
            "-089.99",
            "-090.01",
        }

        pixmap = window.pixhawk.baseline_yaw_plot.grab()
        assert not pixmap.isNull()
        assert pixmap.height() == BaselineYawAlignmentPlot.HEIGHT_PX
    finally:
        _close_window(window)
