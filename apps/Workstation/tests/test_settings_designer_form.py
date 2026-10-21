"""Settings form loading preserves editable presentation and runtime contracts."""
from __future__ import annotations

import xml.etree.ElementTree as ET

from PyQt5 import QtWidgets, uic

from Workstation.ui import settings_workspace
from Workstation.ui.designer import FORMS_ROOT


def test_settings_designer_edits_reach_runtime_without_rebuilding_python(tmp_path, monkeypatch):
    form = ET.parse(FORMS_ROOT / "settings_workspace.ui")
    names = [node.get("name") for node in form.iter() if node.tag in ("widget", "layout", "spacer")]
    assert len(names) == len(set(names)), "Designer object names must be unique"
    button = form.find(".//widget[@name='yolo_weights_browse_button']/property[@name='text']/string")
    button.text = "Designer edited model picker"
    tabs = form.find(".//widget[@name='tabs']")
    tabs.find("widget/attribute[@name='title']/string").text = "Designer edited camera tab"
    field = form.find(".//widget[@name='yolo_input_size']/property[@name='suffix']/string")
    field.text = " pixels"
    changed = tmp_path / "settings_workspace.ui"
    form.write(changed, encoding="utf-8")
    monkeypatch.setattr(settings_workspace, "load_form", lambda _name, target: uic.loadUi(str(changed), target))
    widget = settings_workspace.SettingsWorkspace(object())
    try:
        assert widget.yolo_weights_browse_button.text() == "Designer edited model picker"
        assert widget.tabs.tabText(0) == "Designer edited camera tab"
        assert widget.settings_widgets["yolo_input_size"].suffix() == " pixels"
        assert isinstance(widget.settings_payload()["yolo_input_size"], int)
    finally:
        widget.close()
        widget.deleteLater()


def test_settings_form_binds_commands_and_keeps_password_session_only():
    commands = []
    widget = settings_workspace.SettingsWorkspace(object(), invoke_command=lambda command: commands.append(command))
    try:
        widget.operation_buttons["settings.apply"].click()
        widget.operation_buttons["settings.save"].click()
        mode = widget.settings_widgets["gnss_operation_mode"]
        mode.setCurrentIndex(mode.findData("NETWORK_RTK_DIRECT"))
        widget.gnss_network_forget_password_button.click()
        assert commands == ["settings.apply", "settings.save", "gnss.ntrip_forget_password"]
        assert widget.gnss_network_password_edit.echoMode() == QtWidgets.QLineEdit.Password
        assert not widget._gnss_network_password_explicit
        widget.gnss_network_password_edit.setText("session-only-test-token")
        assert widget._gnss_network_password_explicit
        assert "session-only-test-token" not in repr(widget.settings_payload())
        assert "gnss_network_password" not in widget.settings_widgets
        # Editable form files never contain the deployment password or any persisted field value.
        form = ET.parse(FORMS_ROOT / "settings_workspace.ui")
        for field in form.findall(".//widget[@class='QLineEdit']"):
            assert field.find("property[@name='text']") is None
    finally:
        widget.close()
        widget.deleteLater()
