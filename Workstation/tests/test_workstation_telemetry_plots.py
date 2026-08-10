from types import SimpleNamespace

import pytest
from PyQt5 import QtWidgets

from Workstation.telemetry_plots import (
    AttitudeHistoryPlot,
    AttitudeHistoryPoint,
    RtkAlignmentPlot,
    TelemetryHistoryPlot,
    TelemetryHistoryPoint,
)


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_plot_types_are_reexported_without_legacy_app_dependency(app):
    from Workstation import standalone_gui

    assert standalone_gui.AttitudeHistoryPlot is AttitudeHistoryPlot
    assert standalone_gui.TelemetryHistoryPlot is TelemetryHistoryPlot
    assert standalone_gui.RtkAlignmentPlot is RtkAlignmentPlot


def test_attitude_and_telemetry_history_keep_source_values(app):
    attitude = AttitudeHistoryPlot("Yaw")
    attitude_point = AttitudeHistoryPoint(1.0, left_deg=2.0, right_deg=-3.0)
    attitude.add_point(attitude_point)
    telemetry = TelemetryHistoryPlot("GPS", "m")
    telemetry_point = TelemetryHistoryPoint(2.0, "eph", 0.25)
    telemetry.add_point(telemetry_point)

    assert tuple(attitude.points) == (attitude_point,)
    assert tuple(telemetry.points) == (telemetry_point,)
    assert attitude.auto_y_bounds([2.0, 2.0])[0] < 2.0


def test_rtk_plot_retains_ned_baseline_and_optical_rdf_translation(app):
    baseline = SimpleNamespace(north_m=1.0, east_m=2.0, down_m=3.0)
    camera_translation_rdf_m = (4.0, 5.0, 6.0)
    plot = RtkAlignmentPlot()

    plot.set_alignment(baseline, camera_translation_rdf_m)

    assert plot.baseline is baseline
    assert plot.camera_translation_m == pytest.approx((4.0, 5.0, 6.0))
