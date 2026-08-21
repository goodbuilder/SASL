from __future__ import annotations

import math
import time
from types import SimpleNamespace

from PyQt5 import QtWidgets

from Workstation.evidence import EvidenceAdapter
from Workstation.status_model import RuntimeStatus, StatusModel, StatusSeverity
from Workstation.workstation_core.px4_calibration_groups import PX4_CALIBRATION_GROUPS
from Workstation.workstation_core.px4_mag_manager import (
    mag_priority_display,
)
from Workstation.workspaces import PixhawkWorkspace
from Workstation.workstation_core.pixhawk_attitude import (
    _can_mag_sensor_state_from_mavlink,
    parse_px4_sensor_mag_shell_output,
)


def _mag_state(
    *,
    slot: str,
    topic: str,
    timestamp_s: float,
    x: float,
    y: float,
    z: float,
) -> SimpleNamespace:
    return SimpleNamespace(
        sensor_slot=slot,
        topic=topic,
        timestamp_s=timestamp_s,
        px4_timestamp_s=12.5,
        x=x,
        y=y,
        z=z,
        magnitude=math.sqrt(x * x + y * y + z * z),
        device_id=None,
    )


def _can_mag_state(
    *,
    slot: str,
    topic: str,
    timestamp_s: float,
    x: float,
    y: float,
    z: float,
    node_id: int = 0x7D,
    device_id: int = 8944899,
) -> SimpleNamespace:
    state = _mag_state(slot=slot, topic=topic, timestamp_s=timestamp_s, x=x, y=y, z=z)
    state.node_id = node_id
    state.device_id = device_id
    return state


def test_mavlink_magnetometer_evidence_keeps_three_imu_streams_and_axes() -> None:
    now = 100.0
    panel = SimpleNamespace(
        latest_mag_sensors={
            "left": {
                "CAN1": _mag_state(
                    slot="CAN1",
                    topic="mavlink/SCALED_IMU",
                    timestamp_s=99.5,
                    x=12.0,
                    y=-3.0,
                    z=4.0,
                ),
                "CAN3": _mag_state(
                    slot="CAN3",
                    topic="mavlink/SCALED_IMU3",
                    timestamp_s=99.75,
                    x=20.0,
                    y=21.0,
                    z=22.0,
                ),
            }
        }
    )

    rows = EvidenceAdapter(panel).magnetometers("left", now_s=now)

    assert [(row["sensor_slot"], row["imu"], row["topic"]) for row in rows] == [
        ("CAN1", "IMU stream 1", "mavlink/SCALED_IMU"),
        ("CAN2", "IMU stream 2", "mavlink/SCALED_IMU2"),
        ("CAN3", "IMU stream 3", "mavlink/SCALED_IMU3"),
    ]
    assert rows[0]["state"] == "LIVE"
    assert rows[0]["xmag"] == 12.0
    assert rows[0]["ymag"] == -3.0
    assert rows[0]["zmag"] == 4.0
    assert rows[0]["unit"] == "mG"
    assert rows[0]["age_s"] == 0.5
    assert rows[1]["state"] == "WAITING"
    assert rows[2]["state"] == "LIVE"
    assert "no coordinate conversion" in str(rows[0]["axis_contract"])


def test_px4_shell_sensor_mag_parser_keeps_only_known_can_device_ids() -> None:
    output = """
nsh> listener sensor_mag -i 0 -n 1
TOPIC: sensor_mag instance 0 #1
 timestamp: 123456789
 device_id: 8944899
 x: 0.012
 y: -0.003
 z: 0.004
nsh> listener sensor_mag -i 1 -n 1
TOPIC: sensor_mag instance 1 #1
 timestamp: 123456790
 device_id: 4395041
 x: 0.1
 y: 0.2
 z: 0.3
"""

    states = parse_px4_sensor_mag_shell_output(output, camera="left", timestamp_s=100.0)

    assert len(states) == 1
    assert states[0].sensor_slot == "CAN1"
    assert states[0].device_id == 8944899
    assert (states[0].x, states[0].y, states[0].z) == (0.012, -0.003, 0.004)
    assert states[0].px4_timestamp_s == 123.456789


def test_pixhawk_workspace_renders_p1_p2_imu_magnetometer_table() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    now = time.monotonic()
    backend = SimpleNamespace(
        panel=SimpleNamespace(
            latest_mag_sensors={
                "left": {
                    "CAN1": _mag_state(
                        slot="CAN1",
                        topic="mavlink/SCALED_IMU",
                        timestamp_s=now - 0.1,
                        x=1.25,
                        y=2.5,
                        z=3.75,
                    )
                },
                "right": {
                    "CAN2": _mag_state(
                        slot="CAN2",
                        topic="mavlink/SCALED_IMU2",
                        timestamp_s=now - 0.2,
                        x=-4.0,
                        y=5.0,
                        z=-6.0,
                    )
                },
            },
            latest_can_mag_sensors={
                "left": {
                    "CAN1": _can_mag_state(
                        slot="CAN1",
                        topic="mavlink/SASL_CAN_MAG",
                        timestamp_s=now - 0.1,
                        x=0.012,
                        y=0.003,
                        z=-0.004,
                    )
                }
            },
        )
    )
    workspace = PixhawkWorkspace(backend, StatusModel(), lambda _command_id: True)

    workspace.refresh({})

    assert workspace.magnetometer_model.horizontalHeaderItem(3).text() == "xmag"
    assert workspace.magnetometer_model.horizontalHeaderItem(5).text() == "zmag"
    assert workspace.magnetometer_model.rowCount() == 6
    assert [workspace.magnetometer_model.item(row, 0).text() for row in range(3)] == ["P1"] * 3
    assert [workspace.magnetometer_model.item(row, 0).text() for row in range(3, 6)] == ["P2"] * 3
    assert workspace.magnetometer_model.item(0, 1).text() == "IMU stream 1"
    assert workspace.magnetometer_model.item(0, 2).text() == "LIVE"
    assert workspace.magnetometer_model.item(0, 3).text() == "1.250"
    assert workspace.magnetometer_model.item(1, 2).text() == "WAITING"
    assert workspace.magnetometer_model.item(4, 1).text() == "IMU stream 2"
    assert workspace.magnetometer_model.item(4, 3).text() == "-4.000"
    assert workspace.can_magnetometer_model.horizontalHeaderItem(1).text() == "CAN port"
    assert workspace.can_magnetometer_model.rowCount() == 4
    assert workspace.can_magnetometer_model.item(0, 1).text() == "CAN1"
    assert workspace.can_magnetometer_model.item(0, 4).text() == "LIVE"
    assert workspace.can_magnetometer_model.item(0, 5).text() == "0.012"
    assert workspace.can_magnetometer_model.item(1, 4).text() == "WAITING"
    assert workspace.operation_buttons["pixhawk.connect_mavlink"].text() == "Connect"
    assert workspace.operation_buttons["pixhawk.disconnect_mavlink"].text() == "Disconnect"
    assert workspace.operation_buttons["pixhawk.connect_mavlink"].isEnabled()
    assert not workspace.operation_buttons["pixhawk.disconnect_mavlink"].isEnabled()
    workspace.refresh(
        {
            "pixhawk.left": RuntimeStatus(
                key="pixhawk.left",
                label="P1",
                state="LIVE",
                severity=StatusSeverity.OK,
                source="test",
            )
        }
    )
    assert not workspace.operation_buttons["pixhawk.connect_mavlink"].isEnabled()
    assert workspace.operation_buttons["pixhawk.disconnect_mavlink"].isEnabled()
    assert workspace.operation_buttons["pixhawk.open_shell.left"].text() == "P1 Shell"
    assert workspace.operation_buttons["pixhawk.open_shell.right"].text() == "P2 Shell"

    workspace.deleteLater()
    app.processEvents()


def test_px4_priority_labels_keep_px4_raw_values_and_custom_values() -> None:
    assert mag_priority_display(-1) == "Uninitialized (-1)"
    assert mag_priority_display(0) == "Disabled (0)"
    assert mag_priority_display(50) == "Medium (50)"
    assert mag_priority_display(90) == "Custom (90)"


def test_pixhawk_workspace_renders_qgc_calibration_parameter_group() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = SimpleNamespace(
        _px4_mag_status_cache={
            "left": SimpleNamespace(
                params={
                    "CAL_MAG0_PRIO": 75,
                    "CAL_MAG0_PITCH": -0.01,
                    "CAL_MAG0_XOFF": 0.125,
                    "CAL_MAG0_XSCALE": 1.015,
                }
            )
        },
        _px4_attitude_cal_status_cache={
            "left": SimpleNamespace(
                params={
                    "CAL_ACC0_ROT": 0,
                    "CAL_ACC0_XOFF": 0.25,
                    "SENS_BOARD_X_OFF": 1.5,
                }
            )
        },
        _px4_calibration_parameter_error={},
        pixhawk_parameter_threads={},
    )
    backend = SimpleNamespace(panel=panel)
    workspace = PixhawkWorkspace(backend, StatusModel())
    snapshot = {
        "pixhawk.left": SimpleNamespace(state="LIVE"),
        "pixhawk.right": SimpleNamespace(state="WAITING"),
    }

    workspace._refresh_px4_parameter_group(snapshot)

    assert set(workspace.px4_parameter_models["left"]) == {
        group.key for group in PX4_CALIBRATION_GROUPS
    }
    values: dict[str, str] = {}
    for calibration in PX4_CALIBRATION_GROUPS:
        model = workspace.px4_parameter_models["left"][calibration.key]
        assert model.rowCount() == len(calibration.parameter_names)
        values.update(
            {
                model.item(row, 0).text(): model.item(row, 1).text()
                for row in range(model.rowCount())
            }
        )
    assert values["CAL_MAG0_PRIO"] == "High (75)"
    assert values["CAL_MAG0_PITCH"] == "-0.010000"
    assert values["CAL_MAG0_XOFF"] == "0.125000"
    assert values["CAL_ACC0_XOFF"] == "0.250000"
    assert values["CAL_ACC0_ROT"] == "ROTATION_NONE (0)"
    assert values["SENS_BOARD_X_OFF"] == "1.500000"
    assert workspace.px4_parameter_reset_buttons["left"]["accelerometer"].text() == "Reset to PX4 default"
    assert not workspace.px4_parameter_reset_buttons["left"]["board"].isEnabled()
    assert "QGC calibration" in workspace.px4_parameter_status_labels["left"].text()

    workspace.deleteLater()
    app.processEvents()


def test_can_magnetometer_evidence_requires_dedicated_message_and_times_out() -> None:
    panel = SimpleNamespace(
        latest_can_mag_sensors={
            "left": {
                "CAN1": _can_mag_state(
                    slot="CAN1",
                    topic="mavlink/SASL_CAN_MAG",
                    timestamp_s=98.5,
                    x=0.012,
                    y=-0.003,
                    z=0.004,
                )
            }
        }
    )

    rows = EvidenceAdapter(panel).can_magnetometers("left", now_s=100.0)

    assert [row["can_port"] for row in rows] == ["CAN1", "CAN2"]
    assert rows[0]["state"] == "STALE"
    assert rows[0]["node_id"] == 0x7D
    assert rows[0]["device_id"] == 8944899
    assert rows[0]["unit"] == "G"
    assert rows[1]["state"] == "WAITING"
    assert "SCALED_IMU" not in str(rows[0]["source"])


def test_dedicated_can_mag_sample_wins_over_legacy_scaled_imu_regardless_of_arrival_order() -> None:
    """A SASL_CAN_MAG sample must keep winning the CAN2 label/plot/monitor.

    Once a dedicated sample exists for a slot, a stock SCALED_IMU2 message
    that happens to arrive afterwards (untouched firmware behaviour running
    in parallel to SASL_CAN_MAG) must not regress the label, plots, or the
    magnetometer_monitor calibration feed back to the legacy, port-ambiguous
    reading -- see sensor_presentation_controller._update_mag_sensor's
    ``display_state`` resolution via ``panel._mag_slot_state``.
    """

    from Workstation.sensor_presentation_controller import SensorPresentationController

    class _FakeMonitor:
        def __init__(self) -> None:
            self.calls: list[tuple[str, object]] = []

        def update(self, camera: str, state: object) -> None:
            self.calls.append((camera, state))

    class _FakePlot:
        def __init__(self) -> None:
            self.points: list[object] = []

        def add_point(self, point: object) -> None:
            self.points.append(point)

    class _FakePanel:
        def __init__(self) -> None:
            self.latest_mag_sensors = {"left": {}, "right": {}}
            self.latest_can_mag_sensors = {"left": {}, "right": {}}
            self.latest_vehicle_mag_device_ids = {"left": None, "right": None}
            self.mag_sensor_labels = {"left": {"CAN2": QtWidgets.QLabel()}, "right": {}}
            self.mag_sensor_plots = {"left": _FakePlot(), "right": _FakePlot()}
            self.rm3100_norm_plots = {"left": _FakePlot(), "right": _FakePlot()}
            self.magnetometer_monitor = _FakeMonitor()

        def _mag_state_display_device_id(self, camera, state):
            device_id = getattr(state, "device_id", None)
            return (device_id, "telemetry") if device_id is not None else (None, "unknown")

        def _magnetometer_device_type(self, slot, device_id):
            return "rm3100" if device_id is not None else "unknown"

        def _mag_slot_state(self, camera, slot):
            if slot in ("CAN1", "CAN2"):
                dedicated = self.latest_can_mag_sensors.get(camera, {}).get(slot)
                if dedicated is not None:
                    return dedicated
            return self.latest_mag_sensors.get(camera, {}).get(slot)

        def _record_external_timeline_row(self, *args, **kwargs) -> None:
            pass

        def refresh_px4_mag_manager(self, camera: str) -> None:
            pass

        def _fmt_record_float(self, value, digits):
            return "" if value is None else f"{value:.{digits}f}"

        def _refresh_pixhawk_sensor_health(self) -> None:
            pass

        def _refresh_alignment_workflow_status(self) -> None:
            pass

        def _refresh_alignment_magnetometer_ui(self) -> None:
            pass

        def _refresh_alignment_overview(self) -> None:
            pass

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = _FakePanel()
    controller = SensorPresentationController(panel)

    dedicated = _can_mag_state(
        slot="CAN2", topic="mavlink/SASL_CAN_MAG", timestamp_s=100.0, x=0.010, y=-0.020, z=0.030
    )
    controller._update_mag_sensor("left", dedicated)

    legacy = _mag_state(
        slot="CAN2", topic="mavlink/SCALED_IMU2", timestamp_s=100.5, x=-4.0, y=5.0, z=-6.0
    )
    controller._update_mag_sensor("left", legacy)

    assert panel.magnetometer_monitor.calls[-1] == ("left", dedicated)
    assert "0.010" in panel.mag_sensor_labels["left"]["CAN2"].text()
    assert "-4.000" not in panel.mag_sensor_labels["left"]["CAN2"].text()
    assert panel.mag_sensor_plots["left"].points[-1].value == dedicated.magnitude
    assert panel.rm3100_norm_plots["left"].points[-1].value == dedicated.magnitude

    app.processEvents()


def test_hidden_camera_parameter_surface_is_not_refreshed_for_each_telemetry_packet() -> None:
    from Workstation.sensor_presentation_controller import SensorPresentationController

    calls: list[str] = []
    panel = SimpleNamespace(
        isVisible=lambda: False,
        _refresh_camera_parameters_tab=lambda: calls.append("refresh"),
    )
    controller = SensorPresentationController(panel)

    controller._refresh_camera_parameters_if_needed()

    assert calls == []


def test_can_magnetometer_mavlink_decoder_keeps_port_and_device_identity() -> None:
    message = SimpleNamespace(
        can_port=2,
        node_id=0x7B,
        device_id=8944387,
        xmag=0.01,
        ymag=-0.02,
        zmag=0.03,
        time_usec=2_000_000,
    )

    state = _can_mag_sensor_state_from_mavlink(
        "left",
        "udp://pixhawk",
        message,
        "SASL_CAN_MAG",
        100.0,
    )

    assert state is not None
    assert state.sensor_slot == "CAN2"
    assert state.node_id == 0x7B
    assert state.device_id == 8944387
    assert state.px4_timestamp_s == 2.0
