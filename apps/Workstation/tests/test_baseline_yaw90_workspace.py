from __future__ import annotations

from copy import deepcopy
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.baseline_yaw90_controller import baseline_yaw90_snapshot, reverse_editable
from Workstation.baseline_yaw90_pane import BaselineYaw90Pane
from Workstation.settings_controller import WorkstationSettingsController
from Workstation.status_model import StatusModel
from Workstation.workspaces import AlignmentWorkspace


def snapshot():
    return {
        "attitudes": [{"label": side, "roll_deg": 1.25, "pitch_deg": -2.5, "state": "CURRENT", "age_s": 0.1}
                      for side in ("P1", "P2")],
        "baseline": {"baseline_azimuth_deg": 90.0, "baseline_state": "CURRENT", "baseline_age_s": 0.2},
        "drone_geometry": {"angles": [
            {"key": "dr_p1_p2", "angle_deg": 45.0, "state": "CURRENT", "age_s": 0.3},
            {"key": "dr_p2_p1", "angle_deg": 90.0, "state": "CURRENT", "age_s": 0.3},
        ]},
        "branch_preview": {"selected_yaw_deg": 0.0, "candidate_minus90_deg": 0.0, "candidate_plus90_deg": 180.0, "mode": "auto"},
        "reverse": False, "reverse_editable": True,
    }


class Backend:
    def __init__(self):
        self.data = snapshot()
        self.calls = []

    def baseline_yaw90_snapshot(self):
        return deepcopy(self.data)

    def apply_baseline_yaw90_settings(self, payload, *, persist=False):
        self.calls.append((dict(payload), persist))
        self.data["reverse"] = payload["measurement_baseline_yaw_reverse"]
        return {"reverse": self.data["reverse"], "saved": persist}


def test_live_azimuth_does_not_require_drone_or_attitude():
    from test_drone_pixhawk_geometry_evidence import _panel

    panel = _panel()
    panel.latest_extrinsics = {}
    panel.latest_gps.pop("drone")
    panel._selected_rover_gps_state = lambda side: panel.latest_gps.get(side)
    panel._selected_baseline_gps_states = lambda: (panel.latest_gps["left"], panel.latest_gps["right"])
    data = baseline_yaw90_snapshot(panel)
    assert data["baseline"]["baseline_azimuth_deg"] == pytest.approx(0.0, abs=1e-4)
    assert data["drone_geometry"]["angles"][0]["angle_deg"] is None
    assert data["attitudes"][0]["state"] == "WAITING"
    assert data["reverse_editable"]


def test_snapshot_selects_branch_without_mutating_measurement_state():
    from test_drone_pixhawk_geometry_evidence import _panel

    panel = _panel()
    before_keys = set(vars(panel))
    before_attitudes = dict(panel.latest_extrinsics)
    data = baseline_yaw90_snapshot(panel)
    assert data["branch_preview"]["candidate_minus90_deg"] == pytest.approx(270.0, abs=1e-4)
    assert data["branch_preview"]["candidate_plus90_deg"] == pytest.approx(90.0, abs=1e-4)
    assert set(vars(panel)) == before_keys
    assert panel.latest_extrinsics == before_attitudes


def test_optional_telemetry_failure_does_not_hide_azimuth_or_attitude():
    from test_drone_pixhawk_geometry_evidence import _panel

    panel = _panel()
    panel.drone_telemetry_snapshot = lambda: (_ for _ in ()).throw(RuntimeError("unavailable"))
    data = baseline_yaw90_snapshot(panel)
    assert data["baseline"]["baseline_azimuth_deg"] == pytest.approx(0.0, abs=1e-4)
    assert data["attitudes"][0]["roll_deg"] == 0.0
    assert data["midpoint"]["state"] == "WAITING"


def test_live_snapshot_uses_selected_gps2_and_marks_future_time(monkeypatch):
    from dataclasses import replace
    from test_drone_pixhawk_geometry_evidence import _panel

    panel = _panel()
    p1, p2 = panel.latest_gps["left"], panel.latest_gps["right"]
    panel.latest_gps["left_GPS2"] = p2
    panel.latest_gps["right_GPS2"] = p1
    panel._pixhawk_gnss_source = lambda side: "GPS2"
    panel._selected_rover_gps_state = lambda side: panel.latest_gps.get(f"{side}_GPS2" if side != "drone" else "drone")
    monkeypatch.setattr("Workstation.baseline_yaw90_controller.time.monotonic", lambda: 100.0)
    data = baseline_yaw90_snapshot(panel)
    assert data["baseline"]["baseline_azimuth_deg"] == pytest.approx(180.0, abs=1e-4)
    assert data["baseline"]["baseline_state"] == "CURRENT"
    panel.latest_gps["left_GPS2"] = replace(p2, timestamp_s=101.0)
    assert baseline_yaw90_snapshot(panel)["baseline"]["baseline_state"] == "STALE"


def test_backend_preview_subscription_is_single_and_released(workstation_qapplication):
    from Workstation.standalone_backend import StandalonePanelBackend

    class Producer(QtCore.QObject):
        frame_changed = QtCore.pyqtSignal(str, QtGui.QImage, object)

    left, right = Producer(), Producer()
    backend = StandalonePanelBackend(SimpleNamespace(left_pane=left, right_pane=right))
    pane = BaselineYaw90Pane(backend)
    assert left.receivers(left.frame_changed) == 1
    image = QtGui.QImage(160, 90, QtGui.QImage.Format_RGB888)
    image.fill(QtCore.Qt.black)
    left.frame_changed.emit("left", image, {})
    assert pane.previews["left"]._image == image
    pane.deleteLater()
    workstation_qapplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
    assert left.receivers(left.frame_changed) == 0


@pytest.mark.parametrize("state", ["QUEUED", "STARTING", "RUNNING", "STOPPING"])
def test_reverse_guard_precedes_every_write(state):
    panel = SimpleNamespace(_measurement_run_state=state, _workstation_setting_overrides={"other": 7})
    owner = WorkstationSettingsController(panel)
    assert not reverse_editable(panel)
    with pytest.raises(RuntimeError, match="Stop Measurement"):
        owner.apply_baseline_yaw90_settings({"measurement_baseline_yaw_reverse": True}, persist=True)
    assert panel._workstation_setting_overrides == {"other": 7}


def test_reverse_apply_save_only_touch_existing_setting():
    saved = []
    panel = SimpleNamespace(_workstation_setting_overrides={"other": 7}, _save_current_settings=lambda: saved.append(True), _log=lambda text: None)
    owner = WorkstationSettingsController(panel)
    owner.apply_baseline_yaw90_settings({"measurement_baseline_yaw_reverse": True})
    assert saved == []
    assert panel._workstation_setting_overrides == {"other": 7, "measurement_baseline_yaw_reverse": True}
    assert owner.apply_baseline_yaw90_settings({"measurement_baseline_yaw_reverse": True}, persist=True)["saved"]
    assert saved == [True]
    with pytest.raises(ValueError):
        owner.apply_baseline_yaw90_settings({"measurement_baseline_yaw_reverse": "false"})


def test_preview_start_does_not_lock_reverse_but_pending_detection_does():
    panel = SimpleNamespace(_pending_camera_start=(False, False))
    assert reverse_editable(panel)
    panel._pending_camera_start = (True, False)
    assert not reverse_editable(panel)


def test_original_reverse_setting_round_trip(tmp_path):
    from Workstation.workstation_core.settings import AppSettings, load_settings, save_settings

    path = tmp_path / "settings.json"
    save_settings(AppSettings(measurement_baseline_yaw_reverse=True), path)
    assert load_settings(path).measurement_baseline_yaw_reverse is True
    assert AppSettings().measurement_baseline_yaw_reverse is False


def test_tables_preserve_state_and_dirty_reverse(workstation_qapplication):
    backend = Backend()
    pane = BaselineYaw90Pane(backend)
    pane.resize(1000, 650)
    pane.show()
    workstation_qapplication.processEvents()
    assert pane.angle_table.item(0, 1).text() == "90.00"
    assert pane.attitude_table.item(1, 2).text() == "-2.50"
    pane.angle_table.selectRow(2)
    widths = [pane.angle_table.columnWidth(i) for i in range(4)]
    pane.reverse_checkbox.setChecked(True)
    backend.data["baseline"]["baseline_azimuth_deg"] = 359.99
    backend.data["drone_geometry"]["drone_position_frozen"] = True
    pane.refresh()
    assert pane.reverse_checkbox.isChecked()
    assert pane.angle_table.item(0, 2).text() == "CURRENT"
    assert pane.angle_table.item(1, 2).text() == "FROZEN"
    assert pane.angle_table.currentRow() == 2
    assert widths == [pane.angle_table.columnWidth(i) for i in range(4)]
    pane.save_button.click()
    assert backend.calls == [({"measurement_baseline_yaw_reverse": True}, True)]
    assert "저장 완료" in pane.settings_status.toolTip()
    backend.data["reverse_editable"] = False
    pane.refresh()
    assert not pane.reverse_checkbox.isEnabled()
    pane.close()


def test_save_failure_is_retryable_and_does_not_claim_saved(workstation_qapplication):
    backend = Backend()
    pane = BaselineYaw90Pane(backend)
    pane.refresh()
    pane.reverse_checkbox.setChecked(True)
    original = backend.apply_baseline_yaw90_settings
    backend.apply_baseline_yaw90_settings = lambda *args, **kwargs: (_ for _ in ()).throw(OSError("disk full"))
    pane.save_button.click()
    assert pane._dirty
    assert "disk full" in pane.settings_status.toolTip()
    backend.apply_baseline_yaw90_settings = original
    pane.save_button.click()
    assert not pane._dirty
    pane.close()


def test_center_line_is_local_and_source_is_untouched(workstation_qapplication):
    pane = BaselineYaw90Pane(Backend())
    image = QtGui.QImage(160, 90, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    pane.set_frame("left", image, {})
    pane.center_line_checkbox.setChecked(True)
    assert all(preview.center_line_visible() for preview in pane.previews.values())
    preview = pane.previews["left"]
    preview.image.resize(160, 90)
    preview._render()
    rendered = preview.image.pixmap().toImage()
    assert rendered.pixelColor(rendered.width() // 2, rendered.height() // 2).red() == 255
    assert image.pixelColor(80, 45).red() == 0
    pane.clear_previews()
    assert preview._image is None
    pane.close()


def test_tab_keeps_optical_objects_and_annotation_mode(workstation_qapplication):
    workspace = AlignmentWorkspace(Backend(), StatusModel(), lambda: None)
    pane = workspace.optical_pose_pane
    workspace.operation_tabs.setCurrentIndex(0)
    old_views = dict(pane.annotation_views)
    mode = pane.annotation_mode_combo.currentData()
    workspace.open_baseline_yaw90()
    assert workspace.operation_tabs.currentIndex() == workspace.baseline_yaw90_index
    assert pane.preview_group.isHidden()
    assert pane.candidate_review_widget.isHidden()
    assert pane.annotation_mode_combo.currentData() == mode
    workspace.operation_tabs.setCurrentIndex(0)
    assert not pane.preview_group.isHidden()
    assert not pane.candidate_review_widget.isHidden()
    assert pane.annotation_views == old_views
    assert pane.annotation_mode_combo.currentData() == mode
    workspace.close()


def test_tab_restores_outer_scroll_and_splitter(workstation_qapplication):
    workspace = AlignmentWorkspace(Backend(), StatusModel(), lambda: None)
    scroll = QtWidgets.QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setWidget(workspace)
    scroll.resize(1000, 600)
    scroll.show()
    workstation_qapplication.processEvents()
    scroll.verticalScrollBar().setValue(200)
    before = scroll.verticalScrollBar().value()
    assert before > 0
    workspace.open_baseline_yaw90()
    workstation_qapplication.processEvents()
    assert scroll.verticalScrollBar().value() == 0
    workspace.operation_tabs.setCurrentIndex(0)
    workstation_qapplication.processEvents()
    assert scroll.verticalScrollBar().value() == before
    scroll.close()


@pytest.mark.parametrize("size", [(1366, 768), (1920, 1080)])
def test_main_window_setup_layout_and_navigation(workstation_qapplication, size):
    from test_workstation_calibration import _FakeBackend
    from Workstation.main_window import WorkstationMainWindow

    backend = _FakeBackend()
    backend.baseline_yaw90_snapshot = snapshot
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.resize(*size)
    window.show()
    window._open_baseline_yaw90()
    workstation_qapplication.processEvents()
    pane = window.alignment.baseline_yaw90_pane
    for side in ("left", "right"):
        image = QtGui.QImage(640, 360, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("#34495e"))
        painter = QtGui.QPainter(image)
        painter.fillRect(312, 170, 16, 20, QtGui.QColor("#d8e0e4"))
        painter.end()
        pane.set_frame(side, image, {"width": 640, "height": 360})
    pane.center_line_checkbox.setChecked(True)
    workstation_qapplication.processEvents()
    viewport = window.workspace_scrolls["alignment"].viewport()
    for widget in (pane.attitude_table, pane.angle_table):
        bottom = widget.mapTo(viewport, widget.rect().bottomRight())
        assert bottom.y() <= viewport.height(), (size, bottom.y(), viewport.height())
    assert not hasattr(window.telemetry, "gnss_baseline_center_group")
    assert not hasattr(window.calibration, "center_line_checkbox")
    assert "measurement_baseline_yaw_reverse" not in window.measurement.settings_payload()
    assert hasattr(window.pixhawk, "baseline_yaw_values")
    assert hasattr(window.pixhawk, "drone_geometry_angle_values")
    window.grab().save(f"/tmp/baseline-yaw90-{size[0]}.png")
    pane.measurement_button.click()
    assert window._current_context == "processing.measurement"
    window.close()


def test_leaving_setup_clears_only_its_inspector_details(workstation_qapplication):
    from test_workstation_calibration import _FakeBackend
    from Workstation.main_window import WorkstationMainWindow

    data = snapshot()
    reads = []
    backend = _FakeBackend()

    def read_setup():
        reads.append(True)
        return deepcopy(data)

    backend.baseline_yaw90_snapshot = read_setup
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.show()
    window._open_baseline_yaw90()
    workstation_qapplication.processEvents()
    pane = window.alignment.baseline_yaw90_pane
    pane.angle_table.selectRow(0)
    pane._inspect("angles", 0)
    assert window.inspector._selection["value"] == "90.00°"
    assert "CURRENT" in window.inspector._selection["freshness"]

    window.alignment.operation_tabs.setCurrentIndex(0)
    count = len(reads)
    data["baseline"].update(baseline_azimuth_deg=135.0, baseline_state="STALE", baseline_age_s=30.0)
    window._refresh_evidence_from_poll(window.status_adapter.model.snapshot())
    workstation_qapplication.processEvents()
    assert window.inspector._selection is None
    assert len(reads) == count  # No hidden setup polling to clear stale details.
    assert pane.angle_table.currentRow() == 0  # Preserve the table's own selection.

    window.alignment.open_baseline_yaw90()
    pane._inspect("angles", 0)
    assert window.inspector._selection["value"] == "135.00°"
    assert window.inspector._selection["freshness"] == "STALE · 30.00 s"
    window.inspector.inspect_selection(
        "processing.alignment", {"selection": "unrelated candidate", "value": "retained"},
        window.status_adapter.model,
    )
    window.alignment.operation_tabs.setCurrentIndex(1)
    assert window.inspector._selection["selection"] == "unrelated candidate"
    window.close()


@pytest.mark.parametrize("width", [1366, 1920])
def test_visible_setup_follows_viewport_height_changes(workstation_qapplication, width):
    from test_workstation_calibration import _FakeBackend
    from Workstation.main_window import WorkstationMainWindow

    backend = _FakeBackend()
    backend.baseline_yaw90_snapshot = snapshot
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window.resize(width, 1080)
    window.show()
    window._open_baseline_yaw90()
    pane = window.alignment.baseline_yaw90_pane
    viewport = window.workspace_scrolls["alignment"].viewport()

    def settle_and_check():
        for _ in range(4):
            workstation_qapplication.processEvents()
        for table in (pane.attitude_table, pane.angle_table):
            bottom = table.mapTo(viewport, table.rect().bottomRight()).y()
            assert bottom <= viewport.height(), (width, bottom, viewport.height())

    settle_and_check()
    tall_preview = pane.previews["left"].height()
    window.resize(width, 768)
    settle_and_check()
    assert pane.previews["left"].height() < tall_preview
    short_preview = pane.previews["left"].height()
    window.diagnostics_dock.hide()
    settle_and_check()
    assert pane.previews["left"].height() > short_preview
    window.diagnostics_dock.show()
    settle_and_check()
    window.resize(width, 1080)
    settle_and_check()
    window.resizeDocks([window.diagnostics_dock], [320], QtCore.Qt.Vertical)
    settle_and_check()

    # Stop coalesced resize work when another Method is shown.
    window.alignment.operation_tabs.setCurrentIndex(0)
    for _ in range(4):
        workstation_qapplication.processEvents()
    hidden_height = pane.previews["left"].height()
    window.resize(width, 768)
    for _ in range(4):
        workstation_qapplication.processEvents()
    assert pane.previews["left"].height() == hidden_height
    window.close()


def test_query_failure_keeps_display_values_and_advances_age(monkeypatch, workstation_qapplication):
    import time

    now = [100.0]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    backend = Backend()
    original = deepcopy(backend.data)
    pane = BaselineYaw90Pane(backend)
    pane.refresh()
    pane.attitude_table.selectRow(0)
    pane.reverse_checkbox.setChecked(True)  # Unsaved operator draft must survive.

    def unavailable():
        raise RuntimeError("snapshot source unavailable")

    backend.baseline_yaw90_snapshot = unavailable
    now[0] = 105.0
    pane.refresh()
    assert pane.attitude_table.item(0, 1).text() == "+1.25"
    assert pane.attitude_table.item(0, 3).text() == "조회 오류"
    assert pane.attitude_table.item(0, 4).text() == "5.10"
    assert pane.angle_table.item(0, 1).text() == "90.00"
    assert pane.angle_table.item(0, 2).text() == "조회 오류"
    assert pane.angle_table.item(1, 3).text() == "5.30"
    assert "마지막 확인값" in pane.status.toolTip()
    assert "snapshot source unavailable" in pane.status.toolTip()
    details = pane._selection_details("attitudes", 0)
    assert details["value"] == "Roll +1.25° / Pitch -2.50°"
    assert details["freshness"] == "조회 오류 · 5.10 s"
    assert "CURRENT" not in details["freshness"]
    assert pane.attitude_table.currentRow() == 0
    assert pane.reverse_checkbox.isChecked()
    assert not pane.apply_button.isEnabled()

    now[0] = 108.0
    pane.refresh()
    assert pane.attitude_table.item(0, 4).text() == "8.10"  # Not 13.10 from double aging.
    assert backend.data == original
    assert backend.calls == []
    backend.baseline_yaw90_snapshot = lambda: deepcopy(backend.data)
    backend.data["attitudes"][0].update(roll_deg=2.5, age_s=0.05)
    pane.refresh()
    assert pane.attitude_table.item(0, 1).text() == "+2.50"
    assert pane.attitude_table.item(0, 3).text() == "CURRENT"
    assert pane.attitude_table.item(0, 4).text() == "0.05"
    assert "조회 오류" not in pane.status.toolTip()
    assert pane.reverse_checkbox.isChecked()  # Recovery cannot replace the draft.
    assert pane.apply_button.isEnabled()
    pane.close()


def test_first_query_failure_and_empty_snapshot_do_not_leave_inspector_current(workstation_qapplication):
    backend = Backend()
    backend.baseline_yaw90_snapshot = lambda: (_ for _ in ()).throw(RuntimeError("read failed"))
    pane = BaselineYaw90Pane(backend)
    pane.refresh()
    details = pane._selection_details("attitudes", 0)
    assert details["value"] == "Roll --° / Pitch --°"
    assert details["freshness"] == "조회 오류 · -- s"
    assert pane.attitude_table.item(0, 1).text() == "--"
    pane.apply_snapshot(snapshot())
    pane.apply_snapshot({})  # A successful empty read is WAITING, not a query error.
    assert pane._selection_details("attitudes", 0)["freshness"] == "WAITING · -- s"
    assert pane._selection_details("attitudes", 0)["value"] == "Roll --° / Pitch --°"
    pane.close()


@pytest.mark.parametrize("bad_data", [None, [], {"attitudes": ["invalid row"]}, {"drone_geometry": {"angles": 42}}])
def test_malformed_snapshot_uses_last_confirmed_display(bad_data, workstation_qapplication):
    backend = Backend()
    pane = BaselineYaw90Pane(backend)
    pane.refresh()
    backend.baseline_yaw90_snapshot = lambda: bad_data
    pane.refresh()
    assert pane.attitude_table.item(0, 1).text() == "+1.25"
    assert pane.attitude_table.item(0, 3).text() == "조회 오류"
    assert pane._selection_details("attitudes", 0)["value"] == "Roll +1.25° / Pitch -2.50°"
    pane.close()


def test_query_error_updates_open_inspector_and_recovers(monkeypatch, workstation_qapplication):
    import time
    from test_workstation_calibration import _FakeBackend
    from Workstation.main_window import WorkstationMainWindow

    now = [100.0]
    monkeypatch.setattr(time, "monotonic", lambda: now[0])
    backend = _FakeBackend()
    backend.baseline_yaw90_snapshot = snapshot
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    window._open_baseline_yaw90()
    pane = window.alignment.baseline_yaw90_pane
    pane._inspect("attitudes", 0)
    before = window.inspector._selection["value"]
    backend.baseline_yaw90_snapshot = lambda: (_ for _ in ()).throw(RuntimeError())
    now[0] = 103.0
    window._refresh_evidence_from_poll(window.status_adapter.model.snapshot())
    assert window.inspector._selection["value"] == before
    assert window.inspector._selection["freshness"] == "조회 오류 · 3.10 s"
    assert "RuntimeError" in window.inspector._selection["verdict"]
    assert pane.attitude_table.item(0, 3).text() == "조회 오류"
    backend.baseline_yaw90_snapshot = snapshot
    window._refresh_evidence_from_poll(window.status_adapter.model.snapshot())
    assert window.inspector._selection["freshness"] == "CURRENT · 0.10 s"
    window.close()


def test_display_query_error_does_not_clear_disconnected_measurement_inputs(workstation_qapplication):
    from test_measurement_methods import _attitude_state, _gps_state, _correction_frame_panel
    from Workstation.measurement_view_controller import MeasurementViewController

    left_gps = _gps_state("left", timestamp_s=100.0)
    right_gps = _gps_state("right", timestamp_s=100.0, longitude_deg=127.00001)
    left_att = _attitude_state("left", timestamp_s=100.0)
    right_att = _attitude_state("right", timestamp_s=100.0)
    panel = _correction_frame_panel(
        identity_id="source-a", base_hash="base-1",
        latest_extrinsics={"left": left_att, "right": right_att},
        _selected_baseline_gps_states=lambda: (left_gps, right_gps),
    )
    controller = MeasurementViewController(panel)
    controller._cached_measurement_attitude_states()
    controller._cached_measurement_gps_states()
    backend = Backend()
    backend.panel = panel
    pane = BaselineYaw90Pane(backend)
    pane.refresh()
    panel.latest_extrinsics = {}
    panel._selected_baseline_gps_states = lambda: (None, None)
    backend.baseline_yaw90_snapshot = lambda: (_ for _ in ()).throw(RuntimeError("UI read failed"))
    pane.refresh()
    assert controller._cached_measurement_attitude_states()[:2] == (left_att, right_att)
    assert controller._cached_measurement_gps_states()[:2] == (left_gps, right_gps)
    assert backend.calls == []
    assert pane.attitude_table.item(0, 1).text() == "+1.25"
    pane.close()


def test_successful_apply_survives_following_query_failure(workstation_qapplication):
    backend = Backend()
    pane = BaselineYaw90Pane(backend)
    pane.refresh()
    pane.reverse_checkbox.setChecked(True)
    backend.baseline_yaw90_snapshot = lambda: (_ for _ in ()).throw(RuntimeError("read failed"))
    pane.apply_button.click()
    assert backend.data["reverse"] is True
    assert pane.reverse_checkbox.isChecked()
    assert "적용 완료" in pane.settings_status.toolTip()
    assert "조회 오류" in pane.status.toolTip()
    pane.close()


def test_query_failure_retains_frozen_reference_values_without_current_claim(workstation_qapplication):
    backend = Backend()
    backend.data["drone_geometry"]["drone_position_frozen"] = True
    backend.data["midpoint"] = {"state": "STALE", "forward_m": 3.0, "range_m": 4.0, "maximum_age_s": 9.0}
    pane = BaselineYaw90Pane(backend)
    pane.refresh()
    backend.baseline_yaw90_snapshot = lambda: (_ for _ in ()).throw(RuntimeError("read failed"))
    pane.refresh()
    assert pane.angle_table.item(1, 1).text() == "45.00"
    assert pane.angle_table.item(1, 2).text() == "조회 오류"
    assert pane.reference_table.item(0, 1).text() == "3.000"
    assert pane.reference_table.item(0, 2).text() == "조회 오류"
    pane.close()
