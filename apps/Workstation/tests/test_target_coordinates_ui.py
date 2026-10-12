from collections import deque
from types import SimpleNamespace

from PyQt5 import QtCore, QtTest, QtWidgets

from Workstation.recording_controller import RecordingController
from Workstation.status_model import StatusModel
from Workstation.workspaces import MeasurementWorkspace
from Workstation.workstation_core.measurement_widgets import DistanceHistoryPoint
from Workstation.workstation_core.target_coordinates import TARGET_COORDINATE_FIELDS, target_coordinates_from_attempt
from Workstation.tests.test_target_coordinate_storage import georeferenced_attempt


def point(**changes):
    values = dict(timestamp_s=100., distance_m=123., quality="GOOD", method="pixhawk_rtk_live",
                  forward_m=120., right_m=5., down_m=-25., target_latitude_deg=37.123456789,
                  target_longitude_deg=127.987654321, target_altitude_ellipsoid_m=156.7892)
    values.update(changes)
    return DistanceHistoryPoint(**values)


def workspace():
    panel = SimpleNamespace(settings=SimpleNamespace(), distance_history=[])
    return MeasurementWorkspace(SimpleNamespace(panel=panel), StatusModel(), lambda _: True), panel


def test_live_values_update_together_and_clear_on_missing_or_empty(monkeypatch):
    import Workstation.workspace_views.measurement as module
    monkeypatch.setattr(module.time, "monotonic", lambda: 100.25)
    view, panel = workspace()
    panel.distance_history = [point()]
    view.refresh({})
    assert view.values["range"].value.text() == "123.0"
    assert view.target_coordinate_values["target_latitude_deg"].value.text() == "37.123456789"
    assert view.target_coordinate_values["target_longitude_deg"].value.text() == "127.987654321"
    assert view.target_coordinate_values["target_altitude_ellipsoid_m"].value.text() == "156.789"
    assert view.target_coordinate_values["target_altitude_msl_m"].value.text() == "--"
    assert "MSL 기준 없음" in view.target_coordinate_status.text()
    assert "0.2s 전" in view.target_coordinate_status.text()
    # A new result with missing geolocation replaces the old coordinates.
    panel.distance_history.append(point(distance_m=200., **dict.fromkeys(TARGET_COORDINATE_FIELDS)))
    view.refresh({})
    assert view.values["range"].value.text() == "200.0"
    assert all(v.value.text() == "--" for v in view.target_coordinate_values.values())
    panel.distance_history.clear()
    view.refresh({})
    assert view.target_coordinate_status.text() == "측정 결과 대기 중"
    view.deleteLater()


def test_last_result_age_advances_and_inspector_has_geographic_units(monkeypatch):
    import Workstation.workspace_views.measurement as module
    monkeypatch.setattr(module.time, "monotonic", lambda: 145.)
    view, panel = workspace()
    panel.distance_history = [point()]
    view.refresh({})
    assert "45.0s 전" in view.target_coordinate_status.text()
    emitted = []
    view.inspection_requested.connect(emitted.append)
    target = view.target_coordinate_values["target_latitude_deg"]
    QtTest.QTest.mouseClick(target, QtCore.Qt.LeftButton)
    assert emitted[-1]["value"] == "37.123456789 deg"
    assert "WGS84" in emitted[-1]["contract"]
    assert "baseline_center_frd" not in emitted[-1]["contract"]
    view.deleteLater()


def test_legacy_and_nonfinite_values_are_unavailable():
    view, panel = workspace()
    panel.distance_history = [SimpleNamespace(timestamp_s=100, distance_m=10)]
    view.refresh({})
    assert all(v.value.text() == "--" for v in view.target_coordinate_values.values())
    panel.distance_history = [point(target_latitude_deg=float("nan"), target_longitude_deg=True)]
    view.refresh({})
    assert view.target_coordinate_values["target_latitude_deg"].value.text() == "--"
    assert view.target_coordinate_values["target_longitude_deg"].value.text() == "--"
    view.deleteLater()


def test_recording_owner_attaches_frozen_coordinates_to_visible_history():
    a = georeferenced_attempt()
    source = (2., -3., 100.)
    drawn = []
    panel = SimpleNamespace(
        _last_distance_history_key=None, distance_recording_enabled=False,
        distance_history=deque(), distance_plot=SimpleNamespace(add_point=drawn.append),
        _distance_history_class_name=lambda _: "target",
    )
    owner = RecordingController(panel)
    estimate = SimpleNamespace(left=a.left_detection, right=a.right_detection, sync_quality="GOOD")
    expected = target_coordinates_from_attempt(a.canonical_payload(), source)
    result = owner._commit_distance_history(
        estimate=estimate, distance_m=100., method=a.measurement_method_id,
        baseline_source=a.baseline_source, baseline_m=a.baseline_m,
        point_m=(100., 1., -3.), raw_point_left_m=source,
        measurement_attempt=a, publish_history=True,
    )
    assert result.history_added
    assert drawn[0] is panel.distance_history[-1]
    for key in TARGET_COORDINATE_FIELDS:
        assert getattr(drawn[0], key) == expected[key]
    assert drawn[0].forward_m == 100.
