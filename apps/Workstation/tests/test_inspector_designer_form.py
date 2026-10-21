"""Inspector Designer forms retain dynamic evidence and typed command payloads."""
from __future__ import annotations

import xml.etree.ElementTree as ET

from PyQt5 import uic

from Workstation.ui import inspector
from Workstation.ui.components import EngineeringValue, PropertySection
from Workstation.ui.designer import FORMS_ROOT
from Workstation.workstation_core.coordinate_contract import MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME


def test_inspector_designer_edits_and_promoted_components(tmp_path, monkeypatch):
    form = ET.parse(FORMS_ROOT / "inspector.ui")
    form.find(".//widget[@name='open_advanced_button']/property[@name='text']/string").text = "Edited advanced action"
    form.find(".//widget[@name='geometry_section']/property[@name='title']/string").text = "Edited geometry grouping"
    form.find(".//widget[@name='mountain_margin_combo']/item/property[@name='text']/string").text = "Edited 2 percent"
    names = [node.get("name") for node in form.iter() if node.tag in ("widget", "layout", "spacer")]
    assert len(names) == len(set(names))
    path = tmp_path / "inspector.ui"
    form.write(path, encoding="utf-8")
    monkeypatch.setattr(inspector, "load_form", lambda _name, target: uic.loadUi(str(path), target))
    calls = []
    widget = inspector.InspectorWidget(object(), lambda: calls.append("advanced"))
    try:
        assert widget.open_advanced_button.text() == "Edited advanced action"
        assert widget.geometry_section.title() == "Edited geometry grouping"
        assert widget.mountain_margin_combo.itemText(0) == "Edited 2 percent"
        assert widget.mountain_margin_combo.itemData(0) == 0.02
        assert isinstance(widget.context_section, PropertySection)
        assert isinstance(widget.context_values["primary"], EngineeringValue)
        assert widget.context_section.form.labelForField(widget.context_values["primary"]) is widget.context_row_labels["primary"]
        widget.open_advanced_button.click()
        assert calls == ["advanced"]
    finally:
        widget.close()
        widget.deleteLater()


def test_inspector_form_preserves_frame_contract_and_typed_commands():
    widget = inspector.InspectorWidget(object(), lambda: None)
    commands = []
    widget.command_requested.connect(lambda command, payload: commands.append((command, payload)))
    try:
        assert widget.output_contract_value.text() == f"{MEASUREMENT_FRAME}\n[F, R, D] · metres"
        assert widget.output_contract_value.toolTip() == MEASUREMENT_AXIS_CONTRACT
        widget.mountain_margin_combo.setCurrentIndex(2)
        widget.mountain_max_points_combo.setCurrentIndex(1)
        assert commands == [
            ("alignment.mountain_ridge_margin.set", {"ridge_margin_fraction": 0.06}),
            ("alignment.mountain_max_points.set", {"max_points": 4}),
        ]
        widget._refresh_geometry_candidates({"candidates": [{
            "candidate_id": "candidate-test", "method": "test", "applicable": True,
            "observable_dof": {"yaw": True}, "optimized_dof": {"yaw": True},
        }]})
        assert widget.geometry_apply_candidate_button.isEnabled()
        widget.geometry_apply_candidate_button.click()
        assert commands[-1] == ("alignment.pose.apply_candidate", {"candidate_id": "candidate-test"})
        widget._refresh_geometry_candidates({"candidates": []})
        assert not widget.geometry_apply_candidate_button.isEnabled()
    finally:
        widget.close()
        widget.deleteLater()
