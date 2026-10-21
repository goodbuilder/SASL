"""Designer forms retain workspace wiring and remain the layout source."""
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
from PyQt5 import QtWidgets

from Workstation.ui import designer
from Workstation.workspace_views.base_gnss import BaseGnssWorkspace
from Workstation.workspace_views.telemetry import TelemetryWorkspace
from Workstation.workspace_views.time_sync import TimeWorkspace


@pytest.mark.parametrize("workspace_type", [TimeWorkspace, TelemetryWorkspace, BaseGnssWorkspace])
def test_designer_device_command_buttons_keep_command_identity(workspace_type):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    invoked = []
    page = workspace_type(SimpleNamespace(), invoked.append)
    try:
        for command, button in page.operation_buttons.items():
            assert button.property("commandId") == command
            button.setEnabled(True)
            button.click()
            assert invoked[-1] == command
        assert len(invoked) == len(page.operation_buttons)
    finally:
        page.close()
        page.deleteLater()
        app.sendPostedEvents()


def test_designer_time_form_edit_is_visible_and_changes_input_limits(tmp_path, monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    tree = ET.parse(designer.FORMS_ROOT / "time_sync.ui")
    button = tree.find(".//widget[@name='operation_buttons_time_check_sync']/property[@name='text']/string")
    button.text = "Designer edit proof"
    maximum = tree.find(".//widget[@name='settings_widgets_camera_sync_required_sr_count']/property[@name='maximum']/number")
    maximum.text = "25"
    tree.write(tmp_path / "time_sync.ui", encoding="utf-8")
    monkeypatch.setattr(designer, "FORMS_ROOT", tmp_path)
    page = TimeWorkspace(SimpleNamespace())
    try:
        assert page.operation_buttons["time.check_sync"].text() == "Designer edit proof"
        assert not page.operation_buttons["time.check_sync"].isEnabled()
        page.settings_widgets["camera_sync_required_sr_count"].setValue(24)
        assert page.settings_payload()["camera_sync_required_sr_count"] == 24
    finally:
        page.close()
        page.deleteLater()
        app.sendPostedEvents()


def test_designer_base_slots_preserve_editable_labels(tmp_path, monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    tree = ET.parse(designer.FORMS_ROOT / "base_gnss.ui")
    label = tree.find(".//widget[@name='survey_values_ecef']/property[@name='engineeringLabel']/string")
    label.text = "ECEF XYZ (m)"
    tree.write(tmp_path / "base_gnss.ui", encoding="utf-8")
    monkeypatch.setattr(designer, "FORMS_ROOT", tmp_path)
    page = BaseGnssWorkspace(SimpleNamespace())
    try:
        assert page.survey_values["ecef"].label.text() == "ECEF XYZ (m)"
        page.refresh({})
        assert not page.apply_candidate_button.isEnabled()
    finally:
        page.close()
        page.deleteLater()
        app.sendPostedEvents()


def test_designer_baseline_yaw90_form_edit_and_navigation(tmp_path, monkeypatch):
    from Workstation.ui.baseline_yaw90_pane import BaselineYaw90Pane

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    tree = ET.parse(designer.FORMS_ROOT / "baseline_yaw90.ui")
    text = tree.find(".//widget[@name='measurement_button']/property[@name='text']/string")
    text.text = "Open measurement (Designer)"
    header = tree.find(".//widget[@name='attitude_table']/column/property[@name='text']/string")
    header.text = "Device (Designer)"
    tree.write(tmp_path / "baseline_yaw90.ui", encoding="utf-8")
    monkeypatch.setattr(designer, "FORMS_ROOT", tmp_path)
    commands, navigation = [], []
    pane = BaselineYaw90Pane(SimpleNamespace(), commands.append)
    pane.open_measurement.connect(lambda: navigation.append(True))
    try:
        assert pane.measurement_button.text() == "Open measurement (Designer)"
        assert pane.attitude_table.horizontalHeaderItem(0).text() == "Device (Designer)"
        pane.preview_button.click()
        pane.measurement_button.click()
        assert commands == ["camera.toggle"]
        assert navigation == [True]
        assert set(pane.previews) == {"left", "right"}
        assert pane.reference_table.isHidden()
        pane.reference_toggle.click()
        assert not pane.reference_table.isHidden()
    finally:
        pane.close()
        pane.deleteLater()
        app.sendPostedEvents()
