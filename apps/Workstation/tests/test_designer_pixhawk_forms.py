"""Form edits and device control wiring across the Pixhawk presentation slots."""
from types import SimpleNamespace
import xml.etree.ElementTree as ET
from PyQt5 import QtWidgets
from Workstation.ui import designer
from Workstation.workspace_views.pixhawk_parts.gnss import GnssView
from Workstation.workspace_views.pixhawk_parts.magnetometers import MagnetometerView


def test_gnss_form_label_edit_preserves_ned_and_frd_bindings(tmp_path, monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    tree = ET.parse(designer.FORMS_ROOT / "pixhawk_gnss.ui")
    label = tree.find(".//widget[@name='baseline_values_north_m']/property[@name='engineeringLabel']/string")
    label.text = "North (Designer)"
    tree.write(tmp_path / "pixhawk_gnss.ui", encoding="utf-8")
    monkeypatch.setattr(designer, "FORMS_ROOT", tmp_path)
    selected = []
    view = GnssView(read_gps=lambda *a, **k: {}, read_selected_source=lambda side: "GPS1",
                    set_source=lambda *args: selected.append(args), read_baseline=lambda: {}, read_yaw=lambda: {})
    assert view.baseline_values["north_m"].label.text() == "North (Designer)"
    assert view.baseline_values["p1_body_frd_forward_m"].label.text() == "FRD Forward"
    view.gnss_source_combos["left"].setCurrentIndex(1)
    assert selected == [("left", "GPS2")]
    view._designer_form.close()
    view._designer_form.deleteLater()
    app.sendPostedEvents()


def test_mag_form_initial_priority_and_apply_pending_wiring():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    invoked = []
    def command(value):
        invoked.append(value)
        return True
    view = MagnetometerView(read_inputs=lambda: SimpleNamespace(status_cache={}),
                            read_cal_mag=lambda *a: {}, read_can=lambda *a: {},
                            read_legacy=lambda *a: {}, read_preview=lambda *a: ("TOPOLOGY WAITING", None),
                            invoke_command=command)
    assert view.mag_priority_payload("left") == {"CAL_MAG0": 0, "CAL_MAG1": 0, "CAL_MAG2": 0, "CAL_MAG3": 0}
    view.mag_priority_apply_buttons["left"].click()
    assert invoked == ["pixhawk.mag.apply.p1"]
    assert view._mag_priority_apply_pending["left"]
    assert not view.mag_priority_apply_buttons["left"].isEnabled()
    assert view.mag_priority_apply_buttons["right"].isEnabled()
    assert not view.px4_parameter_reset_buttons["left"]["board"].isEnabled()
    view._designer_form.close()
    view._designer_form.deleteLater()
    app.sendPostedEvents()


def test_pixhawk_form_keeps_direct_action_layout_and_commands():
    from Workstation.status_model import StatusModel
    from Workstation.workspace_views.pixhawk import PixhawkWorkspace
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    invoked = []
    workspace = PixhawkWorkspace(SimpleNamespace(panel=SimpleNamespace()), StatusModel(), invoked.append)
    actions = workspace.layout().itemAt(1).layout()
    assert [actions.itemAt(index).widget().text() for index in range(4)] == ["Connect", "Disconnect", "Find USB", "Find network"]
    actions.itemAt(2).widget().click()
    assert invoked == ["pixhawk.discover_usb"]
    workspace.close()
    workspace.deleteLater()
    app.sendPostedEvents()
