"""Pixhawk presentation ownership and unchanged identity/frame handoffs."""

import ast
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtWidgets

from Workstation.status_model import StatusModel
from Workstation.workspace_views.pixhawk import PixhawkWorkspace
from Workstation.workspace_views.pixhawk_parts.attitude import AttitudeView
from Workstation.workspace_views.pixhawk_parts.connections import ConnectionView
from Workstation.workspace_views.pixhawk_parts.contracts import (
    ConnectionCommands,
    ConnectionInputs,
    MagnetometerInputs,
    TransportDisplay,
)
from Workstation.workspace_views.pixhawk_parts.geometry import GeometryView
from Workstation.workspace_views.pixhawk_parts.gnss import GnssView
from Workstation.workspace_views.pixhawk_parts.magnetometers import MagnetometerView


@pytest.fixture
def host(workstation_qapplication):
    widget = QtWidgets.QWidget()
    widget.resize(440, 700)
    QtWidgets.QVBoxLayout(widget)
    yield widget
    widget.close()
    widget.deleteLater()
    workstation_qapplication.sendPostedEvents(widget, QtCore.QEvent.DeferredDelete)
    workstation_qapplication.processEvents()


def _connection_inputs(rows=(), *, revision=1):
    return ConnectionInputs(
        transports={side: TransportDisplay("radio", "", 57_600) for side in ("left", "right")},
        endpoints={"left": "udpin:0.0.0.0:14550", "right": "udpin:0.0.0.0:14551"},
        saved_serial_keys=(),
        connection={"candidates": tuple(rows), "revision": revision},
        recovery={"left": {}, "right": {}},
        open_sides=frozenset(),
        configured_sides=frozenset(("left", "right")),
        connection_in_progress=False,
        automatic_recording=False,
    )


def _mag_inputs(**overrides):
    return replace(
        MagnetometerInputs(
            status_cache={},
            incomplete_cache={},
            attitude_cache={},
            rotation_cache={},
            active_ids={},
            connected=frozenset(),
            verified=frozenset(),
            parameter_sides=frozenset(),
            shell_sides=frozenset(),
            armed_sides=frozenset(),
            parameter_errors={},
            reset_errors={},
            now_s=10.0,
        ),
        **overrides,
    )


def test_workspace_aliases_keep_one_real_owner_and_original_section_order(host):
    workspace = PixhawkWorkspace(SimpleNamespace(panel=SimpleNamespace()), StatusModel(), parent=host)
    owners = (
        (workspace.connections, "candidate_model", "candidate_table"),
        (workspace.attitude, "attitude_model", "attitude_table"),
        (workspace.gnss, "receiver_model", "receiver_table"),
        (workspace.magnetometers, "magnetometer_model", "magnetometer_table"),
        (workspace.geometry, "drone_geometry_model", "drone_geometry_table"),
    )
    for owner, model_name, table_name in owners:
        assert owner.parent() is workspace
        assert getattr(workspace, model_name) is getattr(owner, model_name)
        assert getattr(workspace, table_name) is getattr(owner, table_name)
        assert getattr(owner, model_name).parent() is owner
        assert model_name not in vars(workspace)
        assert table_name not in vars(workspace)
        assert not {"backend", "panel", "workspace"} & vars(owner).keys()
    assert workspace._mag_status_auto_refresh_attempts is workspace.magnetometers._mag_status_auto_refresh_attempts
    assert workspace.mag_priority_payload("left") == workspace.magnetometers.mag_priority_payload("left")
    groups = [workspace.layout().itemAt(index).widget() for index in range(workspace.layout().count())]
    assert [group.title() for group in groups if isinstance(group, QtWidgets.QGroupBox)] == [
        "Board transport · explicit Connect only",
        "Connection",
        "Network candidates and assignments",
        "Attitude (R/P/Y)",
        "P1/P2 GNSS receivers",
        "Magnetometer by CAL_MAG slot",
        "MAG Priority / active selection",
        "CAN magnetometer (dedicated MAVLink)",
        "P1→P2 baseline",
        "P1→P2 baseline / Pixhawk yaw",
        "Drone Pixhawk geometry · F/R/D/Rg",
        "ETH MAV",
        "GNSS used for Pixhawk calculation",
        "Pixhawk Parameters · calibration-owned groups",
    ]


def test_candidate_refresh_preserves_identity_and_uses_producer_source_index(host):
    usb = {
        "source": "mavlink",
        "source_row": 7,
        "kind": "USB",
        "value": "/dev/serial/by-id/p2",
        "system_id": 2,
        "component_id": 1,
        "status": "heartbeat verified",
    }
    network = {
        "source": "network",
        "kind": "NETWORK",
        "candidate_id": "nic-stable-a",
        "interface": "nic0",
        "carrier": "UP",
        "local_cidr": "10.0.0.10/24",
    }
    inputs = [_connection_inputs((network, usb))]
    applied = []
    commands = ConnectionCommands(
        assign_network=lambda identity, target: applied.append((identity, target)),
        apply_candidate=lambda source, row, target: applied.append((source, row, target)),
    )
    view = ConnectionView(
        read_inputs=lambda: inputs[0],
        commands=lambda: commands,
        transport_defaults={},
        parent=host,
    )
    host.layout().addWidget(view.candidate_group)
    view.refresh({})
    view.candidate_table.selectRow(1)
    view.candidate_table.setColumnWidth(1, 341)
    # Reordering does not turn the selected USB endpoint into the network row.
    inputs[0] = _connection_inputs((dict(usb, source_row=11), network))
    view.refresh({})
    assert view._selected_candidate_identity() == ("mavlink", "USB", "/dev/serial/by-id/p2")
    assert view._selected_candidate_row() == 0
    assert view.candidate_table.columnWidth(1) == 341
    assert view.candidate_model.item(0, 4).text() == "P2 (sysid 2)"
    view.candidate_buttons["right"].click()
    assert applied == [("mavlink", 11, "right")]
    view.candidate_table.selectRow(1)
    view.candidate_buttons["left"].click()
    assert applied[-1] == ("nic-stable-a", "left")
    # A new discovery deliberately invalidates selection, even for the same rows.
    inputs[0] = _connection_inputs((usb, network), revision=2)
    view.refresh({})
    assert view._selected_candidate_row() == -1
    assert not view.candidate_buttons["left"].isEnabled()


def test_attitude_updates_keep_selection_width_scroll_and_live_inspector(host, workstation_qapplication):
    rows = {
        "left": {"roll_deg": 1.25, "pitch_deg": -2.5, "yaw_deg": 30, "state": "LIVE"},
        "right": {"roll_deg": 4, "pitch_deg": 5, "yaw_deg": 60, "state": "LIVE"},
    }
    view = AttitudeView(read_attitude=lambda side: rows[side], parent=host)
    host.layout().addWidget(view.group)
    view.refresh()
    host.show()
    workstation_qapplication.processEvents()
    view.attitude_table.selectRow(1)
    view.attitude_table.setColumnWidth(0, 243)
    view.attitude_table.horizontalScrollBar().setValue(view.attitude_table.horizontalScrollBar().maximum())
    before_scroll = view.attitude_table.horizontalScrollBar().value()
    cell = view.attitude_model.item(1, 4)
    inspections = []
    view.inspection_requested.connect(inspections.append)
    view._inspect_attitude_row(view.attitude_model.index(1, 0))
    rows["right"] = dict(rows["right"], yaw_deg=-90.0)
    view.refresh()
    assert view.attitude_model.item(1, 4) is cell
    assert cell.text() == "-90.000"
    assert view.attitude_table.selectionModel().selectedRows()[0].row() == 1
    assert view.attitude_table.columnWidth(0) == 243
    assert view.attitude_table.horizontalScrollBar().value() == before_scroll
    assert "-90.000" in inspections[0]["_resolver"]()["value"]
    assert "body FRD" in inspections[0]["contract"]


def test_gnss_selection_and_baseline_keep_gps_height_and_frame_contracts(host):
    selected = {"left": "GPS2", "right": "GPS1"}
    calls = []

    def set_source(side, source):
        calls.append((side, source))
        selected[side] = source

    def gps(side, *, source):
        offset = {("left", "GPS1"): 100, ("left", "GPS2"): 200, ("right", "GPS1"): 300, ("right", "GPS2"): 400}[
            (side, source)
        ]
        return {"fix_type": 6, "altitude_m": offset, "altitude_ellipsoid_m": offset + 27.5}

    baseline = {
        "north_m": 1,
        "east_m": 2,
        "down_m": 3,
        "length_m": 4,
        "p1_body_frd_forward_m": 11,
        "p1_body_frd_right_m": 12,
        "p1_body_frd_down_m": 13,
        "state": "GOOD",
        "p1_body_frd_state": "READY",
    }
    view = GnssView(
        read_gps=gps,
        read_selected_source=selected.__getitem__,
        set_source=set_source,
        read_baseline=lambda: baseline,
        read_yaw=lambda: {},
        parent=host,
    )
    view.refresh_receivers()
    view.refresh_baseline()
    assert [view.receiver_model.item(row, 0).text() for row in range(4)] == [
        "P1 GPS1",
        "P1 GPS2 · selected",
        "P2 GPS1 · selected",
        "P2 GPS2",
    ]
    assert view.receiver_model.item(1, 4).text() == "200.000"
    assert view.receiver_model.item(1, 5).text() == "227.500"
    assert view.baseline_values["north_m"].value.text() == "1.000"
    assert view.baseline_values["p1_body_frd_forward_m"].value.text() == "11.000"
    assert "local NED" in view._baseline_inspection("north_m")["contract"]
    assert "P1 body FRD" in view._baseline_inspection("p1_body_frd_forward_m")["contract"]
    combo = view.gnss_source_combos["left"]
    combo.setCurrentIndex(combo.findData("GPS1"))
    assert calls == [("left", "GPS1")]
    assert view.gnss_source_summary.text() == "계산 source: P1 GPS1 → P2 GPS1"


def test_mag_readback_and_verification_remain_producer_owned(host):
    inputs = [
        _mag_inputs(
            status_cache={"left": SimpleNamespace(params={"CAL_MAG0_PRIO": 100})},
            connected=frozenset(("left", "right")),
            verified=frozenset(),
            parameter_sides=frozenset(("right",)),
            armed_sides=frozenset(("left",)),
        )
    ]
    verification = {
        "verification_started_s": 1.0,
        "state": "VERIFYING",
        "expected_device_id": 123,
        "actual_device_id": 456,
        "reason": "readback pending",
    }
    commands = []
    view = MagnetometerView(
        read_inputs=lambda: inputs[0],
        read_cal_mag=lambda side: (),
        read_can=lambda side: (),
        read_legacy=lambda side: (),
        read_preview=lambda side, priorities: ("TOPOLOGY WAITING", None),
        read_verification=lambda side: verification if side == "left" else {},
        invoke_command=lambda command: commands.append(command),
        parent=host,
    )
    view.refresh({})
    assert "VERIFYING" in view.mag_priority_status_labels["left"].text()
    assert "expected=123" in view.mag_priority_status_labels["left"].text()
    assert "actual=456" in view.mag_priority_status_labels["left"].text()
    assert view.mag_priority_payload("left")["CAL_MAG0"] == 100
    assert not view.mag_priority_apply_buttons["left"].isEnabled()  # Armed remains producer evidence.
    assert not view.mag_priority_refresh_buttons["right"].isEnabled()  # Readback already active.
    assert commands == []
    inputs[0] = replace(inputs[0], armed_sides=frozenset())
    verification.update(state="ACTIVE VERIFIED", actual_device_id=123)
    view.refresh({})
    assert "ACTIVE VERIFIED" in view.mag_priority_status_labels["left"].text()
    assert view.mag_priority_apply_buttons["left"].isEnabled()
    view.mag_priority_apply_buttons["left"].click()
    assert commands == ["pixhawk.mag.apply.p1"]


def test_geometry_renders_producer_frd_and_ned_angles_without_recalculation(host):
    data = {
        "state": "CURRENT",
        "vectors": [
            {
                "key": "dr-from-p1",
                "label": "Dr from P1",
                "state": "CURRENT",
                "forward_m": -7,
                "right_m": 8,
                "down_m": 9,
                "range_m": 123,
                "frame": "P1 body FRD [Forward,Right,Down]",
                "source": "producer fixture",
            }
        ],
        "angles": [{"key": "dr_p1_p2", "angle_deg": 17, "frame": "horizontal local NED"}],
    }
    view = GeometryView(read_geometry=lambda: data, parent=host)
    view.refresh()
    assert [view.drone_geometry_model.item(0, col).text() for col in (2, 3, 4, 5)] == [
        "-7.000",
        "+8.000",
        "+9.000",
        "123.000",
    ]
    assert view.drone_geometry_angle_values["dr_p1_p2"].value.text() == "17.00"
    assert "P1 body FRD" in view._drone_geometry_vector_inspection("dr-from-p1")["contract"]
    assert "horizontal local NED" in view._drone_geometry_angle_inspection("dr_p1_p2")["contract"]


def _assert_component_boundary(source):
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = [alias.name for alias in node.names]
            modules = names if isinstance(node, ast.Import) else [node.module or ""]
            assert not any(
                "workspaces" in name.split(".")
                or name.endswith(".pixhawk")
                or name == "pixhawk"
                or name.endswith("_controller")
                or "panel_" in name
                for name in modules + names
            )
            assert "*" not in names
        elif isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
            assert node.attr not in {"backend", "panel", "workspace"}
        elif isinstance(node, ast.FunctionDef):
            assert node.name != "__getattr__"


def test_parts_never_import_or_store_the_workspace_panel_or_controller():
    directory = Path(__file__).resolve().parents[1] / "workspace_views" / "pixhawk_parts"
    for path in directory.glob("*.py"):
        _assert_component_boundary(path.read_text())


def test_missing_required_projection_reader_fails_at_composition(host):
    with pytest.raises(TypeError, match="projection reader"):
        GeometryView(read_geometry=None, parent=host)


@pytest.mark.parametrize(
    "source",
    [
        "from Workstation.workspace_views.pixhawk import PixhawkWorkspace",
        "import Workstation.workspaces as workspace",
        "from Workstation import panel_diagnostics",
        "class Part:\n def bind(self, panel):\n  self.panel = panel",
    ],
)
def test_component_gate_rejects_broad_owner_dependencies(source):
    with pytest.raises(AssertionError):
        _assert_component_boundary(source)
