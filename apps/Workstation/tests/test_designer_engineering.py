"""Designer edits are the runtime source for engineering controls."""
from pathlib import Path
from types import SimpleNamespace

from PyQt5 import QtWidgets

from Workstation.application.status_model import StatusModel
from Workstation.ui import designer
from Workstation.workspace_views.advanced import AdvancedWorkspace
from Workstation.workspace_views.pixhawk import PixhawkWorkspace


def test_advanced_designer_edit_preserves_command_and_semantic_data(monkeypatch, tmp_path):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    source = (designer.FORMS_ROOT / "advanced.ui").read_text()
    (tmp_path / "advanced.ui").write_text(source.replace("<string>Send</string>", "<string>Designer send</string>"))
    monkeypatch.setattr(designer, "FORMS_ROOT", tmp_path)
    commands = []
    workspace = AdvancedWorkspace(SimpleNamespace(panel=SimpleNamespace()), StatusModel(), commands.append)
    assert workspace.wiris_send_button.text() == "Designer send"
    workspace.wiris_send_button.click()
    assert commands == ["advanced.wiris.send"]
    assert [workspace.wiris_target_combo.itemData(i) for i in range(3)] == ["both", "left", "right"]
    assert workspace.wiris_terminator_combo.itemData(1) == "\0"
    assert workspace.px4_priority_payload() == {"BOARD": 0, "CAN1": 100, "CAN2": 90, "GPS1_MAG": 0, "GPS2_MAG": 0}
    assert (workspace.zoom_level_spin.minimum(), workspace.zoom_level_spin.maximum()) == (0, 100)
    workspace.deleteLater()
    app.processEvents()


def test_pixhawk_designer_transport_preserves_explicit_connect_commands():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls = []
    backend = SimpleNamespace(panel=SimpleNamespace(), connect_pixhawk_role=calls.append)
    workspace = PixhawkWorkspace(backend, StatusModel())
    assert calls == []
    workspace.role_connect_buttons["left"].click()
    assert calls == ["left"]
    assert [workspace.transport_combos["right"].itemData(i) for i in range(3)] == ["radio", "ethernet", "usb"]
    assert (workspace.radio_baud_spins["left"].minimum(), workspace.radio_baud_spins["left"].maximum()) == (1200, 4000000)
    workspace.deleteLater()
    app.processEvents()


def test_calibration_forms_preserve_validation_and_capture_contracts():
    from Workstation.workspace_views.calibration import CalibrationWorkspace
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    workspace = CalibrationWorkspace(SimpleNamespace(panel=SimpleNamespace()), lambda: None)
    validation = workspace._validation
    assert validation.checkerboard_validation_mode_combo.count() == 2
    assert [validation.checkerboard_validation_mode_combo.itemData(i) for i in range(2)] == ["pixhawk", "checkerboard"]
    assert validation.checkerboard_validation_random_runs_spin.value() == 5
    assert validation.checkerboard_validation_results_text.isReadOnly()
    capture = workspace._capture
    assert capture.auto_capture_settings_group.isCheckable()
    assert not capture.auto_capture_settings_group.isChecked()
    assert capture.capture_response_outer_group.isCheckable()
    assert all(log.isReadOnly() for log in capture.calibration_capture_response_edits.values())
    assert workspace.setup_widgets["pair_max_delta_ms"].suffix() == " ms"
    assert workspace.setup_widgets["pair_max_delta_ms"].value() == 100.0
    assert workspace.setup_widgets["square_size_mm"].suffix() == " mm"
    assert "X_right = R_left_to_right X_left" in workspace.extrinsics_bundle_contract_label.text()
    workspace.deleteLater()
    app.processEvents()
