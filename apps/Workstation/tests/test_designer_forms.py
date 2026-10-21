"""Bundled forms are editable assets, not a parallel runtime layout."""
from __future__ import annotations

import io
from pathlib import Path
import shutil
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
from PyQt5 import QtWidgets, uic

from Workstation.ui import designer
from Workstation.application.status_model import StatusModel
from Workstation.workspace_views.camera import CameraWorkspace
from Workstation.workspace_views.experiment import ExperimentWorkspace
from Workstation.workspace_views.overview import OverviewWorkspace


@pytest.mark.parametrize("form", sorted(designer.FORMS_ROOT.glob("*.ui")), ids=lambda path: path.stem)
def test_bundled_form_has_unique_designer_names_and_compiles(form):
    tree = ET.parse(form)
    names = [node.get("name") for node in tree.iter()
             if node.tag in ("widget", "layout", "spacer", "action", "buttongroup")]
    assert len(names) == len(set(names)), f"Duplicate Designer object names: {form.name}"
    output = io.StringIO()
    uic.compileUi(str(form), output)
    compile(output.getvalue(), form.name, "exec")
    # Compiling cannot detect a property whose generated setter does not
    # exist (for example the read-only text property on a spin box).
    loaded = uic.loadUi(str(form))
    loaded.close()
    loaded.deleteLater()


def test_main_window_form_has_real_resizable_central_widget():
    window = QtWidgets.QMainWindow()
    designer.load_form("main_window.ui", window)
    try:
        window.show()
        for width, height in ((1366, 768), (1920, 1080)):
            window.resize(width, height)
            QtWidgets.QApplication.processEvents()
            assert window.centralWidget() is not None
            assert window.workspaceHost.height() > height / 2
            assert window.workspaceHost.parentWidget() is window.centralWidget()
        assert window.navigationDock.objectName() == "navigationDock"
        assert window.inspectorDock.objectName() == "inspectorDock"
        assert window.diagnosticsDock.objectName() == "diagnosticsDock"
    finally:
        window.close()
        window.deleteLater()


def test_edited_form_text_layout_and_navigation_survive_owner_initialization(tmp_path, monkeypatch):
    for name in ("overview.ui", "experiment.ui"):
        shutil.copyfile(designer.FORMS_ROOT / name, tmp_path / name)
    form = ET.parse(tmp_path / "overview.ui")
    form.find(".//widget[@name='camera_button']/property[@name='text']/string").text = "Designer camera link"
    # Change an existing layout property as well as a label: Python must not
    # silently restore the old placement after loading.
    margin = form.find(".//layout/property[@name='leftMargin']/number")
    margin.text = "23"
    form.write(tmp_path / "overview.ui", encoding="utf-8")
    monkeypatch.setattr(designer, "FORMS_ROOT", tmp_path)
    page = OverviewWorkspace(SimpleNamespace(), StatusModel())
    protocol = ExperimentWorkspace(SimpleNamespace())
    selected = []
    page.open_context.connect(selected.append)
    protocol.open_context.connect(selected.append)
    try:
        assert page.camera_button.text() == "Designer camera link"
        assert page.layout().contentsMargins().left() == 23
        page.camera_button.click()
        protocol.pre_run_calibration_button.click()
        protocol.step_action_buttons[1][1].click()
        assert selected == ["devices.cameras", "tasks.calibration", "devices.pixhawk"]
        assert not protocol.step_buttons[0].isEnabled()
        protocol.refresh({})  # Standalone Protocol has no attached recording owner.
    finally:
        page.close()
        protocol.close()
        page.deleteLater()
        protocol.deleteLater()


def test_acquisition_form_edit_preserves_camera_command_and_pair_identity(tmp_path, monkeypatch):
    for source in designer.FORMS_ROOT.glob("camera*.ui"):
        shutil.copyfile(source, tmp_path / source.name)
    path = tmp_path / "camera_connect.ui"
    text = path.read_text()
    assert "Check control" in text
    path.write_text(text.replace("Check control", "Designer camera check"))
    monkeypatch.setattr(designer, "FORMS_ROOT", tmp_path)
    commands = []
    page = CameraWorkspace(SimpleNamespace(), StatusModel(), commands.append)
    try:
        button = page.operation_buttons["camera.check_control"]
        assert button.text() == "Designer camera check"
        button.click()
        assert commands == ["camera.check_control"]
        assert page.preview_splitter.count() == 2
    finally:
        page.close()
        page.deleteLater()


def test_form_loader_uses_package_path_not_cwd_and_fails_for_missing_asset(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    widget = QtWidgets.QWidget()
    designer.load_form("overview.ui", widget)
    assert widget.camera_button.text() == "Connect cameras"
    with pytest.raises(ValueError):
        designer.load_form("../overview.ui", widget)
    with pytest.raises(FileNotFoundError):
        designer.load_form("missing.ui", QtWidgets.QWidget())
    widget.deleteLater()


def test_designer_diagnostics_tab_reorder_preserves_navigation_and_refresh(tmp_path, monkeypatch):
    from Workstation.workspace_views.diagnostics import DiagnosticsContent

    form = ET.parse(designer.FORMS_ROOT / "diagnostics.ui")
    tabs = form.find(".//widget[@name='tabs']")
    log_page = tabs.find("widget[@name='log_page']")
    tabs.remove(log_page)
    tabs.insert(0, log_page)
    form.write(tmp_path / "diagnostics.ui", encoding="utf-8")
    monkeypatch.setattr(designer, "FORMS_ROOT", tmp_path)
    page = DiagnosticsContent(SimpleNamespace(diagnostic_text=lambda: "log source sentinel"), StatusModel())
    try:
        page.select_tab_for_context("diagnostics.logs")
        assert page.tabs.currentIndex() == 0
        page.refresh({})
        assert page.log_view.toPlainText() == "log source sentinel"
        page.select_tab_for_context("diagnostics.system")
        assert page.tabs.currentWidget() is page.system_table
        page.select_tab_for_context("diagnostics.camera")
        assert page.tabs.currentWidget() is page.camera_view
    finally:
        page.close()
        page.deleteLater()
