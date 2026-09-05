from __future__ import annotations

import math
import time
from contextlib import contextmanager
from dataclasses import replace
from types import SimpleNamespace

import pytest
from PyQt5 import QtWidgets

from Workstation.evidence import EvidenceAdapter
from Workstation.status_model import RuntimeStatus, StatusModel, StatusSeverity
from Workstation.workstation_core.px4_calibration_groups import PX4_CALIBRATION_GROUPS
from Workstation.workstation_core.px4_mag_manager import (
    MAG_CAL_PARAM_NAMES,
    MAG_PARAM_NAMES,
    MAG_SLOTS,
    RM3100_ALTERNATE_PRIMARY_ID,
    RM3100_BACKUP_ID,
    build_mag_status,
    mag_priority_display,
)
from Workstation.workspaces import PixhawkWorkspace
from Workstation.workstation_core.pixhawk_attitude import (
    _can_mag_sensor_state_from_mavlink,
    _can_mag_sensor_state_from_debug_vect,
    parse_px4_sensor_mag_shell_output,
    parse_px4_vehicle_magnetometer_shell_device_id,
)


def _mag_state(
    *,
    slot: str,
    topic: str,
    timestamp_s: float,
    x: float,
    y: float,
    z: float,
    device_id: int | None = None,
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
        device_id=device_id,
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
    assert rows[0]["raw_yaw_deg"] == pytest.approx(math.degrees(math.atan2(3.0, 12.0)))
    assert rows[0]["px4_yaw_state"] == "WAITING"


def test_mavlink_imu_row_marks_cached_values_stale_by_app_receive_age() -> None:
    from Workstation.workstation_core.pixhawk_attitude import MAVLINK_IMU_STREAM_STALE_S

    now = 100.0
    panel = SimpleNamespace(
        latest_mag_sensors={
            "left": {
                "CAN1": _mag_state(
                    slot="CAN1",
                    topic="mavlink/SCALED_IMU",
                    timestamp_s=now - MAVLINK_IMU_STREAM_STALE_S - 0.001,
                    x=1.0,
                    y=2.0,
                    z=3.0,
                )
            }
        }
    )

    rows = EvidenceAdapter(panel).magnetometers("left", now_s=now)

    assert rows[0]["state"] == "STALE"
    assert rows[0]["xmag"] == 1.0
    assert rows[0]["age_s"] == pytest.approx(MAVLINK_IMU_STREAM_STALE_S + 0.001)
    assert rows[1]["state"] == "WAITING"


def test_imu_magnetometer_view_appends_debug_vect_rm3100_rows_without_relabeling_topic() -> None:
    now = 100.0
    panel = SimpleNamespace(
        latest_mag_sensors={
            "left": {
                "CAN1": _mag_state(
                    slot="CAN1",
                    topic="mavlink/SCALED_IMU",
                    timestamp_s=99.9,
                    x=12.0,
                    y=-3.0,
                    z=4.0,
                )
            }
        },
        latest_can_mag_sensors={
            "left": {
                "CAN1": _can_mag_state(
                    slot="CAN1",
                    topic="mavlink/DEBUG_VECT",
                    timestamp_s=99.9,
                    x=0.012,
                    y=0.003,
                    z=-0.004,
                )
            }
        },
    )

    rows = EvidenceAdapter(panel).magnetometers_for_imu_view("left", now_s=now)

    assert len(rows) == 5
    assert rows[0]["imu"] == "IMU stream 1"
    assert rows[3]["imu"] == "RM3100 CAN1"
    assert rows[3]["topic"] == "mavlink/DEBUG_VECT"
    assert rows[3]["unit"] == "G"
    assert rows[3]["raw_yaw_deg"] == pytest.approx(-math.degrees(math.atan2(0.003, 0.012)))
    assert "SCALED_IMU" not in str(rows[3]["source"])


def test_can_magnetometer_evidence_correlates_only_exact_active_device_with_px4_yaw() -> None:
    panel = SimpleNamespace(
        latest_can_mag_sensors={
            "left": {
                "CAN1": _can_mag_state(
                    slot="CAN1",
                    topic="mavlink/SASL_CAN_MAG",
                    timestamp_s=99.8,
                    x=0.012,
                    y=0.003,
                    z=-0.004,
                    device_id=8944899,
                )
            }
        },
        latest_vehicle_mag_device_ids={"left": 8944899},
        latest_extrinsics={
            "left": SimpleNamespace(
                yaw_deg=30.0,
                timestamp_s=99.9,
                topic="ATTITUDE",
            )
        },
    )

    can1, can2 = EvidenceAdapter(panel).can_magnetometers("left", now_s=100.0)

    assert can1["raw_yaw_deg"] == pytest.approx(-math.degrees(math.atan2(0.003, 0.012)))
    assert can1["px4_yaw_state"] == "SELECTED"
    assert can1["px4_yaw_deg"] == pytest.approx(30.0)
    assert can1["delta_yaw_deg"] == pytest.approx(30.0 + math.degrees(math.atan2(0.003, 0.012)))
    assert can2["px4_yaw_state"] == "WAITING"


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


def test_px4_shell_vehicle_magnetometer_parser_returns_selected_device_id() -> None:
    output = """
nsh> listener vehicle_magnetometer -n 1
TOPIC: vehicle_magnetometer
 timestamp: 123456800
 timestamp_sample: 123456790
 device_id: 8944899
 magnetometer_ga: [0.012, -0.003, 0.004]
 calibration_count: 2
"""

    assert parse_px4_vehicle_magnetometer_shell_device_id(output) == 8944899


def test_px4_shell_vehicle_magnetometer_parser_prefers_latest_uorb_timestamp() -> None:
    output = """
TOPIC: vehicle_magnetometer
 timestamp: 100
 device_id: 4395041

TOPIC: vehicle_magnetometer
 timestamp: 200
 device_id: 8944899
"""

    assert parse_px4_vehicle_magnetometer_shell_device_id(output) == 8944899


def test_stable_shell_poll_command_covers_all_mag_instances_and_selected_device() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    command = DeviceRuntimeController._px4_sensor_mag_shell_command()

    assert [f"listener sensor_mag -i {instance} -n 1" in command for instance in range(5)] == [
        True,
        True,
        True,
        True,
        True,
    ]
    assert "listener vehicle_magnetometer -n 1" in command


def test_stable_shell_poll_worker_frames_each_listener_command_separately() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController
    from Workstation.device_runtime_workers import PixhawkShellCommandWorker
    from Workstation.workstation_core.pixhawk_attitude import MavlinkShellCommandResult

    class _FakeSubscriber:
        def __init__(self) -> None:
            self.commands: list[str] = []

        @contextmanager
        def exclusive_command(self, _camera: str, **_kwargs):
            yield

        def send_shell_command_with_result(self, _camera: str, command: str, *, timeout_s: float):
            self.commands.append(command)
            return MavlinkShellCommandResult(command, f"out:{command}\n", "respond")

    subscriber = _FakeSubscriber()
    results: list[object] = []
    worker = PixhawkShellCommandWorker(
        subscriber,
        camera="left",
        command=DeviceRuntimeController._px4_sensor_mag_shell_commands(),
        operation="sensor_mag_shell_poll",
        timeout_s=1.0,
    )
    worker.completed.connect(results.append)
    worker.run()

    assert len(subscriber.commands) == 6
    assert all(";" not in command for command in subscriber.commands)
    assert results[0]["output"].count("out:") == 6


def test_stable_shell_poll_yields_to_manual_shell_on_same_pixhawk() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    calls: list[tuple[str, str, dict[str, object]]] = []
    panel = SimpleNamespace(
        _mavlink_open_sides=lambda: ("left", "right"),
        pixhawk_shell_threads={"right": object()},
        _start_pixhawk_shell_worker=lambda camera, command, **kwargs: calls.append(
            (camera, command, kwargs)
        ) or True,
    )
    controller = object.__new__(DeviceRuntimeController)
    controller.panel = panel

    controller._poll_px4_sensor_mag_shell()

    assert len(calls) == 1
    assert calls[0][0] == "left"
    assert calls[0][2]["operation"] == "sensor_mag_shell_poll"
    assert "listener vehicle_magnetometer -n 1" in calls[0][1]


def test_stable_shell_result_handoff_updates_active_id_and_two_rm3100_states() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    states: list[object] = []
    active_ids: dict[str, int | None] = {"left": None}
    panel = SimpleNamespace(
        latest_vehicle_mag_device_ids=active_ids,
        _log=lambda _text: None,
        refresh_px4_mag_manager=lambda _camera: None,
        _update_mag_sensor=lambda _camera, state: states.append(state),
    )
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
 device_id: 8944387
 x: -0.010
 y: 0.002
 z: 0.005
nsh> listener sensor_mag -i 2 -n 1
TOPIC: sensor_mag instance 2 #1
 timestamp: 123456791
 device_id: 4395041
 x: 0.011
 y: 0.002
 z: 0.004
nsh> listener sensor_mag -i 3 -n 1
TOPIC: sensor_mag instance 3 #1
 timestamp: 123456792
 device_id: 396809
 x: 0.010
 y: 0.003
 z: 0.005
nsh> listener sensor_mag -i 4 -n 1
TOPIC: sensor_mag instance 4 #1
 timestamp: 123456793
 device_id: 396817
 x: 0.009
 y: 0.004
 z: 0.006
nsh> listener vehicle_magnetometer -n 1
TOPIC: vehicle_magnetometer
 timestamp: 123456800
 device_id: 8944899
 magnetometer_ga: [0.012, -0.003, 0.004]
"""

    WirisStereoDistancePanel._apply_px4_sensor_mag_shell_result(
        panel,
        "left",
        {"operation": "sensor_mag_shell_poll", "output": output},
    )

    assert active_ids["left"] == 8944899
    assert [(state.sensor_slot, state.topic) for state in states] == [
        ("CAN1", "px4-shell/sensor_mag/0"),
        ("CAN2", "px4-shell/sensor_mag/1"),
        ("CAN1", "px4-shell/sensor_mag/2"),
        ("CAN2", "px4-shell/sensor_mag/3"),
        ("CAN3", "px4-shell/sensor_mag/4"),
    ]


def test_start_pixhawk_flag_selects_page_and_invokes_real_connect_owner() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    tabs = SimpleNamespace(calls=[])
    tabs.setCurrentIndex = lambda index: tabs.calls.append(index)
    alignment_tabs = SimpleNamespace(calls=[])
    alignment_tabs.setCurrentIndex = lambda index: alignment_tabs.calls.append(index)
    log_lines: list[str] = []
    connect_calls: list[object] = []
    panel = SimpleNamespace(
        tabs=tabs,
        alignment_tab_index=4,
        alignment_tabs=alignment_tabs,
        baseline_alignment_tab_index=2,
        _log=log_lines.append,
        connect_mavlink_pixhawk=lambda: connect_calls.append(True),
    )

    WirisStereoDistancePanel._auto_start_pixhawk(panel)

    assert tabs.calls == [4]
    assert alignment_tabs.calls == [2]
    assert connect_calls == [True]
    assert "configured P1/P2/Drone MAVLink" in log_lines[-1]


def test_pixhawk_workspace_renders_p1_p2_imu_magnetometer_table() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    now = time.monotonic()
    cal_params = {
        "CAL_MAG0_ID": 4395041,
        "CAL_MAG0_PRIO": 1,
        "CAL_MAG1_ID": 396809,
        "CAL_MAG1_PRIO": 1,
        "CAL_MAG2_ID": 8944899,
        "CAL_MAG2_PRIO": 100,
        "CAL_MAG3_ID": 8944387,
        "CAL_MAG3_PRIO": 100,
        "CAL_MAG2_XOFF": 0.0,
        "CAL_MAG2_YOFF": 0.0,
        "CAL_MAG2_ZOFF": 0.0,
        "CAL_MAG2_XSCALE": 1.0,
        "CAL_MAG2_YSCALE": 1.0,
        "CAL_MAG2_ZSCALE": 1.0,
        "CAL_MAG2_XODIAG": 0.0,
        "CAL_MAG2_YODIAG": 0.0,
        "CAL_MAG2_ZODIAG": 0.0,
        "CAL_MAG2_XCOMP": 0.0,
        "CAL_MAG2_YCOMP": 0.0,
        "CAL_MAG2_ZCOMP": 0.0,
        "CAL_MAG2_ROT": 0,
        "CAL_MAG2_ROLL": 0.0,
        "CAL_MAG2_PITCH": 0.0,
        "CAL_MAG2_YAW": 0.0,
    }
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
                        device_id=396809,
                    )
                },
            },
            latest_can_mag_sensors={
                "left": {
                    "CAN1": _can_mag_state(
                        slot="CAN1",
                        topic="px4-shell/sensor_mag/0",
                        timestamp_s=now - 0.1,
                        x=0.012,
                        y=0.003,
                        z=-0.004,
                    )
                }
            },
            latest_vehicle_mag_device_ids={"left": 8944899, "right": None},
            latest_sensor_mag_instances={
                "left": {
                    0: SimpleNamespace(device_id=4395041, topic="px4-shell/sensor_mag/0"),
                    1: SimpleNamespace(device_id=396809, topic="px4-shell/sensor_mag/1"),
                    2: SimpleNamespace(device_id=396817, topic="px4-shell/sensor_mag/2"),
                    3: SimpleNamespace(device_id=8944899, topic="px4-shell/sensor_mag/3"),
                    4: SimpleNamespace(device_id=8944387, topic="px4-shell/sensor_mag/4"),
                },
                "right": {},
            },
            _px4_mag_status_cache={
                "left": SimpleNamespace(params=cal_params),
                "right": SimpleNamespace(params=cal_params),
            },
            latest_extrinsics={
                "left": SimpleNamespace(
                    roll_deg=0.0,
                    pitch_deg=0.0,
                    yaw_deg=30.0,
                    timestamp_s=now - 0.1,
                    topic="ATTITUDE",
                )
            },
        )
    )
    workspace = PixhawkWorkspace(backend, StatusModel(), lambda _command_id: True)

    workspace.refresh({})

    assert workspace.magnetometer_model.horizontalHeaderItem(1).text() == "CAL_MAG"
    assert workspace.magnetometer_model.horizontalHeaderItem(2).text() == "Sensor"
    assert workspace.magnetometer_model.horizontalHeaderItem(5).text() == "xmag"
    assert workspace.magnetometer_model.horizontalHeaderItem(7).text() == "zmag"
    assert workspace.magnetometer_model.horizontalHeaderItem(9).text() == "RAW yaw°"
    assert workspace.magnetometer_model.horizontalHeaderItem(10).text() == "Corrected yaw°"
    assert workspace.magnetometer_model.horizontalHeaderItem(11).text() == "Δyaw°"
    assert not workspace.magnetometer_table.horizontalHeader().isSectionHidden(8)
    assert workspace.magnetometer_table.horizontalHeader().isSectionHidden(11)
    assert workspace.magnetometer_model.rowCount() == 8
    assert [workspace.magnetometer_model.item(row, 0).text() for row in range(4)] == ["P1"] * 4
    assert [workspace.magnetometer_model.item(row, 0).text() for row in range(4, 8)] == ["P2"] * 4
    assert [workspace.magnetometer_model.item(row, 1).text() for row in range(4)] == [
        "CAL_MAG0", "CAL_MAG1", "CAL_MAG2", "CAL_MAG3"
    ]
    assert workspace.magnetometer_model.item(0, 4).text() == "CONFIG"
    assert workspace.magnetometer_model.item(2, 2).text().startswith("RM3100")
    assert workspace.magnetometer_model.item(2, 4).text() == "LIVE"
    assert workspace.magnetometer_model.item(2, 5).text() == "0.012"
    assert workspace.magnetometer_model.item(2, 9).text() == "-014.04"
    assert workspace.magnetometer_model.item(2, 10).text() == "-014.04"
    assert workspace.magnetometer_model.item(2, 11).text() == "+044.04"
    assert workspace.magnetometer_model.item(2, 15).text() == "px4-shell/sensor_mag/0"
    assert workspace.magnetometer_model.item(5, 1).text() == "CAL_MAG1"
    assert workspace.magnetometer_model.item(5, 2).text().startswith("GPS1")
    assert workspace.magnetometer_model.item(5, 5).text() == "-4.000"
    inspection_payloads = []
    workspace.inspection_requested.connect(inspection_payloads.append)
    workspace._inspect_magnetometer_row(workspace.magnetometer_model.index(2, 1))
    assert "RM3100" in str(inspection_payloads[-1]["selection"])
    assert "node=125 device=8944899" in str(inspection_payloads[-1]["value"])
    assert "RAW=-014.04°" in str(inspection_payloads[-1]["value"])
    assert workspace.magnetometer_model.item(0, 9).text() == "--"
    assert workspace.magnetometer_model.item(0, 10).text() == "대기"
    assert workspace.magnetometer_model.item(0, 11).text() == "--"
    assert workspace.can_magnetometer_model.horizontalHeaderItem(1).text() == "CAN port"
    assert workspace.can_magnetometer_model.horizontalHeaderItem(9).text() == "RAW yaw°"
    assert workspace.can_magnetometer_model.horizontalHeaderItem(10).text() == "Corrected yaw°"
    assert workspace.can_magnetometer_model.horizontalHeaderItem(11).text() == "Δyaw°"
    assert not workspace.can_magnetometer_table.horizontalHeader().isSectionHidden(10)
    assert workspace.can_magnetometer_table.horizontalHeader().isSectionHidden(11)
    assert workspace.can_magnetometer_model.rowCount() == 4
    assert workspace.can_magnetometer_model.item(0, 1).text() == "CAN1"
    assert workspace.can_magnetometer_model.item(0, 4).text() == "LIVE"
    assert workspace.can_magnetometer_model.item(0, 5).text() == "0.012"
    assert workspace.can_magnetometer_model.item(0, 9).text() == "-014.04"
    assert workspace.can_magnetometer_model.item(0, 10).text() == "-014.04"
    assert workspace.can_magnetometer_model.item(0, 11).text() == "+044.04"
    inspection = workspace._can_magnetometer_inspection("left", 0)
    assert "RAW=-014.04°" in str(inspection["value"])
    assert "Corrected=-014.04" in str(inspection["value"])
    assert "Corrected CALIBRATED" in str(inspection["verdict"])
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
    # P1 is already live, but Connect remains available for the independently
    # disconnected P2 side.
    assert workspace.operation_buttons["pixhawk.connect_mavlink"].isEnabled()
    assert workspace.operation_buttons["pixhawk.disconnect_mavlink"].isEnabled()
    assert workspace.operation_buttons["pixhawk.open_shell.left"].text() == "P1 Shell"
    assert workspace.operation_buttons["pixhawk.open_shell.right"].text() == "P2 Shell"
    assert workspace.mag_priority_status_labels["left"].text().startswith("P1:")
    assert workspace.mag_priority_status_labels["right"].text().startswith("P2:")
    assert "i3:CAN1 id=8944899" in workspace.mag_priority_preview_labels["left"].text()
    assert "Priority candidate: CAN1 id=8944899 inst=3 CAL_MAG2 prio=100" in (
        workspace.mag_priority_preview_labels["left"].text()
    )
    assert "Actual PX4: CAN1 id=8944899" in workspace.mag_priority_preview_labels["left"].text()
    workspace.mag_priority_combos["left"]["CAL_MAG2"].setCurrentIndex(
        workspace.mag_priority_combos["left"]["CAL_MAG2"].findData(100)
    )
    workspace.mag_priority_combos["right"]["CAL_MAG2"].setCurrentIndex(
        workspace.mag_priority_combos["right"]["CAL_MAG2"].findData(75)
    )
    assert workspace.mag_priority_payload("left")["CAL_MAG2"] == 100
    assert workspace.mag_priority_payload("right")["CAL_MAG2"] == 75
    assert workspace.mag_priority_apply_buttons["left"].text() == "Apply & Verify"
    assert workspace.mag_priority_apply_buttons["right"].text() == "Apply & Verify"

    workspace.deleteLater()
    app.processEvents()


def test_px4_priority_labels_keep_px4_raw_values_and_custom_values() -> None:
    assert mag_priority_display(-1) == "Uninitialized (-1)"
    assert mag_priority_display(0) == "Disabled (0)"
    assert mag_priority_display(50) == "Medium (50)"
    assert mag_priority_display(90) == "Custom (90)"


def test_px4_v117_stock_magnetometer_refresh_uses_only_slots_zero_through_three() -> None:
    from Workstation.workstation_core.px4_attitude_cal_manager import (
        ATTITUDE_CAL_PARAM_NAMES,
    )

    assert MAG_SLOTS == (0, 1, 2, 3)
    assert len(MAG_CAL_PARAM_NAMES) == 72
    assert len(MAG_PARAM_NAMES) == 75
    assert len(MAG_PARAM_NAMES) + len(ATTITUDE_CAL_PARAM_NAMES) == 124
    assert "CAL_MAG3_ZCOMP" in MAG_PARAM_NAMES
    assert not any(name.startswith("CAL_MAG4_") for name in MAG_PARAM_NAMES)


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


def test_px4_shell_rm3100_evidence_is_live_and_keeps_shell_provenance() -> None:
    panel = SimpleNamespace(
        latest_can_mag_sensors={
            "left": {
                "CAN1": _can_mag_state(
                    slot="CAN1",
                    topic="px4-shell/sensor_mag/0",
                    timestamp_s=99.0,
                    x=0.012,
                    y=-0.003,
                    z=0.004,
                )
            }
        },
        latest_vehicle_mag_device_ids={"left": 8944899},
        latest_extrinsics={
            "left": SimpleNamespace(
                yaw_deg=30.0,
                timestamp_s=99.9,
                topic="ATTITUDE",
            )
        },
    )

    can1, can2 = EvidenceAdapter(panel).can_magnetometers("left", now_s=100.0)

    assert can1["state"] == "LIVE"
    assert can1["topic"] == "px4-shell/sensor_mag/0"
    assert can1["source"] == "PX4 Shell sensor_mag readback"
    assert can1["raw_yaw_deg"] == pytest.approx(math.degrees(math.atan2(0.003, 0.012)))
    assert can1["px4_yaw_state"] == "SELECTED"
    assert can1["px4_yaw_deg"] == pytest.approx(30.0)
    assert can2["state"] == "WAITING"


def test_can_magnetometer_evidence_falls_back_to_cal_mag_identity_without_live_message() -> None:
    """Node/Device ID come from the CAL_MAG* readout when DEBUG_VECT is silent.

    This is the operator-facing path for a Pixhawk reached over Ethernet/radio
    telemetry (or USB) whose firmware never sends the private DEBUG_VECT
    CAN_MAG compatibility message: the identity is still knowable from the
    standard MAVLink PARAM protocol, which works over any transport.
    """

    params = {
        "UAVCAN_ENABLE": 2,
        "UAVCAN_SUB_MAG": 1,
        "CAL_MAG0_ID": 4395041,
        "CAL_MAG0_PRIO": 75,
        "CAL_MAG1_ID": 396809,
        "CAL_MAG1_PRIO": 75,
        "CAL_MAG2_ID": RM3100_ALTERNATE_PRIMARY_ID,
        "CAL_MAG2_PRIO": 100,
        "CAL_MAG3_ID": RM3100_BACKUP_ID,
        "CAL_MAG3_PRIO": 0,
    }
    mag_status = build_mag_status(params, actual_selected_device_id=RM3100_ALTERNATE_PRIMARY_ID, armed=False)
    panel = SimpleNamespace(
        latest_can_mag_sensors={},
        _px4_mag_status_cache={"left": mag_status},
    )

    rows = EvidenceAdapter(panel).can_magnetometers("left", now_s=100.0)

    assert [row["can_port"] for row in rows] == ["CAN1", "CAN2"]
    can1, can2 = rows
    assert can1["state"] == "CONFIG"
    assert can1["device_id"] == RM3100_ALTERNATE_PRIMARY_ID
    assert can1["node_id"] == 0x7C
    assert can1["xmag"] is None and can1["ymag"] is None and can1["zmag"] is None
    assert "CAL_MAG" in str(can1["source"])
    assert can2["state"] == "CONFIG"
    assert can2["device_id"] == RM3100_BACKUP_ID
    assert can2["node_id"] == 0x7B


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


def test_px4_shell_sensor_mag_sample_routes_to_dedicated_rm3100_state() -> None:
    from Workstation.sensor_presentation_controller import SensorPresentationController

    class _FakeMonitor:
        def update(self, _camera: str, _state: object) -> None:
            pass

    class _FakePanel:
        def __init__(self) -> None:
            self.latest_mag_sensors = {"left": {}, "right": {}}
            self.latest_can_mag_sensors = {"left": {}, "right": {}}
            self.latest_vehicle_mag_device_ids = {"left": None, "right": None}
            self.mag_sensor_labels = {"left": {"CAN1": QtWidgets.QLabel()}, "right": {}}
            self.mag_sensor_plots = {"left": None, "right": None}
            self.rm3100_norm_plots = {"left": None, "right": None}
            self.magnetometer_monitor = _FakeMonitor()

        def _mag_state_display_device_id(self, _camera, state):
            device_id = getattr(state, "device_id", None)
            return (device_id, "PX4 Shell") if device_id is not None else (None, "unknown")

        def _magnetometer_device_type(self, _slot, _device_id):
            return "RM3100"

        def _mag_slot_state(self, camera, slot):
            dedicated = self.latest_can_mag_sensors.get(camera, {}).get(slot)
            return dedicated if dedicated is not None else self.latest_mag_sensors.get(camera, {}).get(slot)

        def _record_external_timeline_row(self, *args, **kwargs) -> None:
            pass

        def refresh_px4_mag_manager(self, _camera: str) -> None:
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
    state = _can_mag_state(
        slot="CAN1",
        topic="px4-shell/sensor_mag/0",
        timestamp_s=100.0,
        x=0.012,
        y=-0.003,
        z=0.004,
    )

    controller._update_mag_sensor("left", state)

    assert panel.latest_can_mag_sensors["left"]["CAN1"] is state
    assert panel.latest_mag_sensors["left"] == {}
    app.processEvents()


def test_px4_shell_standard_identity_survives_scaled_imu_refresh_for_px4_yaw() -> None:
    from Workstation.sensor_presentation_controller import SensorPresentationController
    from Workstation.workstation_core.pixhawk_attitude import MagSensorState

    class _FakeMonitor:
        def update(self, _camera: str, _state: object) -> None:
            pass

    class _FakePlot:
        def add_point(self, _point: object) -> None:
            pass

    class _FakePanel:
        def __init__(self) -> None:
            self.latest_mag_sensors = {"left": {}, "right": {}}
            self.latest_can_mag_sensors = {"left": {}, "right": {}}
            self.latest_vehicle_mag_device_ids = {"left": None, "right": None}
            self.mag_sensor_labels = {"left": {"CAN1": QtWidgets.QLabel()}, "right": {}}
            self.mag_sensor_plots = {"left": _FakePlot(), "right": _FakePlot()}
            self.rm3100_norm_plots = {"left": _FakePlot(), "right": _FakePlot()}
            self.magnetometer_monitor = _FakeMonitor()

        def _mag_state_display_device_id(self, _camera, state):
            device_id = getattr(state, "device_id", None)
            return (device_id, "PX4 Shell" if device_id is not None else "unknown")

        def _magnetometer_device_type(self, _slot, _device_id):
            return "mag"

        def _mag_slot_state(self, camera, slot):
            dedicated = self.latest_can_mag_sensors.get(camera, {}).get(slot)
            return dedicated if dedicated is not None else self.latest_mag_sensors.get(camera, {}).get(slot)

        def _record_external_timeline_row(self, *args, **kwargs) -> None:
            pass

        def refresh_px4_mag_manager(self, _camera: str) -> None:
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
    shell_state = MagSensorState(
        camera="left",
        namespace="px4_shell",
        sensor_slot="CAN1",
        timestamp_s=100.0,
        topic="px4-shell/sensor_mag/0",
        device_id=4395041,
        x=0.012,
        y=-0.003,
        z=0.004,
        magnitude=0.013,
    )
    scaled_state = MagSensorState(
        camera="left",
        namespace="mavlink",
        sensor_slot="CAN1",
        timestamp_s=100.5,
        topic="mavlink/SCALED_IMU",
        device_id=None,
        x=12.0,
        y=-3.0,
        z=4.0,
        magnitude=13.0,
    )

    controller._update_mag_sensor("left", shell_state)
    controller._update_mag_sensor("left", scaled_state)
    controller._update_mag_sensor(
        "left",
        replace(scaled_state, timestamp_s=101.0, device_id=None, x=11.0, y=-2.0),
    )

    stored = panel.latest_mag_sensors["left"]["CAN1"]
    assert stored.topic == "mavlink/SCALED_IMU"
    assert stored.device_id == 4395041
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


def test_debug_vect_decoder_maps_stable_can_mag_name_to_rm3100_identity() -> None:
    message = SimpleNamespace(
        name="CAN1_MAG",
        x=0.012,
        y=0.003,
        z=-0.004,
        time_usec=2_000_000,
    )

    state = _can_mag_sensor_state_from_debug_vect("left", "udp://pixhawk", message, 100.0)

    assert state is not None
    assert state.sensor_slot == "CAN1"
    assert state.device_id == 8944899
    assert state.node_id == 0x7D
    assert state.topic == "mavlink/DEBUG_VECT"
    assert state.px4_timestamp_s == 2.0
