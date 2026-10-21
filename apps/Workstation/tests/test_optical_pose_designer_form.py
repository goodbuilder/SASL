"""Optical-pose forms preserve component contracts and editable command presentation."""
from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET

from PyQt5 import QtWidgets

from Workstation.ui import designer
from Workstation.workspace_views.optical_pose import OpticalPosePane


def test_optical_pose_designer_edits_reach_factory_buttons_and_native_views(tmp_path, monkeypatch):
    for source in designer.FORMS_ROOT.glob("optical_pose*.ui"):
        shutil.copyfile(source, tmp_path / source.name)
    path = tmp_path / "optical_pose_methods.ui"
    tree = ET.parse(path)
    for widget in tree.findall(".//widget"):
        command = widget.find("property[@name='commandId']/string")
        if command is not None and command.text == "alignment.pose.solve_horizon":
            widget.find("property[@name='text']/string").text = "Designer Horizon action"
            font = ET.SubElement(ET.SubElement(widget, "property", name="font"), "font")
            ET.SubElement(font, "bold").text = "true"
            ET.SubElement(ET.SubElement(widget, "property", name="styleSheet"), "string").text = "color: rgb(13, 71, 161);"
            ET.SubElement(ET.SubElement(widget, "property", name="checkable"), "bool").text = "true"
    tree.write(path, encoding="utf-8")
    path = tmp_path / "optical_pose_annotations.ui"
    tree = ET.parse(path)
    tree.find("widget/property[@name='title']/string").text = "Designer frozen pair"
    tree.write(path, encoding="utf-8")
    monkeypatch.setattr(designer, "FORMS_ROOT", tmp_path)
    pane = OpticalPosePane()
    try:
        assert pane.preview_group.title() == "Designer frozen pair"
        button = pane.solver_buttons["alignment.pose.solve_horizon"]
        assert button.text() == "Designer Horizon action"
        assert button.font().bold()
        assert button.styleSheet() == "color: rgb(13, 71, 161);"
        assert button.isCheckable()
        commands = []
        pane.command_requested.connect(lambda command, payload: commands.append((command, payload)))
        button.click()
        assert commands[0][0] == "alignment.pose.solve_horizon"
        assert commands[0][1]["annotation_contract"].startswith("frozen native OpenCV pixels: u right, v down")
        assert pane.annotation_views["left"] is pane.annotations.annotation_views["left"]
    finally:
        pane.close()
        pane.deleteLater()


def test_optical_pose_forms_preserve_headers_units_and_unique_names():
    for path in designer.FORMS_ROOT.glob("optical_pose*.ui"):
        tree = ET.parse(path)
        names = [node.get("name") for node in tree.iter() if node.tag in ("widget", "layout", "spacer")]
        assert len(names) == len(set(names)), path.name
    pane = OpticalPosePane()
    try:
        assert tuple(pane.candidate_table.horizontalHeaderItem(i).text() for i in range(pane.candidate_table.columnCount())) == pane.CANDIDATE_HEADERS
        assert tuple(pane.observation_history_table.horizontalHeaderItem(i).text() for i in range(pane.observation_history_table.columnCount())) == pane.DRONE_RTK_OBSERVATION_HEADERS
        assert pane.live_rt_table.horizontalHeaderItem(2).text() == "t · RDF (m)"
        assert [pane.candidates.combine_dof_table.horizontalHeaderItem(i).text() for i in range(1, 7)] == ["Roll", "Pitch", "Yaw", "N", "E", "D"]
        pane.method_tabs.setCurrentIndex(1)
        assert pane.annotation_mode_combo.currentData() == "ridge"
        assert pane.mountain_hide_auto_lines_checkbox.isChecked()
        assert pane.frozen_preview_layout.count() == 2
    finally:
        pane.close()
        pane.deleteLater()
