"""Workstation pixhawk gnss regression tests."""

from __future__ import annotations

from PyQt5 import QtCore

from Workstation.tests.workstation_test_support import (
    BackendStatusAdapter,
    contextmanager,
    EvidenceAdapter,
    _FakeBackend,
    Path,
    pytest,
    QtWidgets,
    SimpleNamespace,
    StatusSeverity,
    time,
    WorkstationMainWindow,
)


def test_p1_body_frd_baseline_projection_keeps_raw_ned_contract_explicit() -> None:
    from math import cos, radians, sin

    from Workstation.workstation_core.rtk_geometry import project_ned_baseline_to_p1_body_frd

    identity = project_ned_baseline_to_p1_body_frd(
        (1.0, 2.0, 3.0),
        (1.0, 0.0, 0.0, 0.0),
        attitude_timestamp_s=10.0,
        attitude_age_s=0.25,
    )
    assert identity.vector_m == pytest.approx((1.0, 2.0, 3.0))
    assert identity.source_frame == "ned"
    assert identity.target_frame == "px4_frd"
    assert identity.direction == "P1->P2"
    assert identity.axis_contract == "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres"
    assert identity.attitude_age_s == pytest.approx(0.25)

    yaw_90 = (cos(radians(90.0) / 2.0), 0.0, 0.0, sin(radians(90.0) / 2.0))
    rotated = project_ned_baseline_to_p1_body_frd((1.0, 2.0, 3.0), yaw_90)
    assert rotated.vector_m == pytest.approx((2.0, -1.0, 3.0))


def test_param_read_defers_attitude_instead_of_dropping_telemetry() -> None:
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class Message:
        def __init__(self, message_type: str, **values) -> None:
            self._message_type = message_type
            self.__dict__.update(values)

        def get_type(self) -> str:
            return self._message_type

    class Mav:
        def param_request_read_send(self, *_args) -> None:
            return None

    class Connection:
        target_system = 1
        target_component = 1
        mav = Mav()

        def __init__(self) -> None:
            self.messages = [
                Message("ATTITUDE", roll=0.1, pitch=0.2, yaw=0.3, time_boot_ms=1),
                Message("PARAM_VALUE", param_id="TEST", param_value=3.5, param_type=9),
            ]

        def recv_match(self, **_kwargs):
            return self.messages.pop(0) if self.messages else None

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udp:fake", right_url="")
    subscriber.connections["left"] = Connection()
    attitudes: list[object] = []
    subscriber.extrinsic_changed.connect(lambda _side, state: attitudes.append(state))

    assert subscriber.read_param("left", "TEST", timeout_s=0.1) == pytest.approx(3.5)
    assert not attitudes
    subscriber.spin_once()

    assert len(attitudes) == 1
    assert attitudes[0].roll_deg == pytest.approx(5.72957795)


def test_generic_param_api_does_not_reject_explicit_non_stock_parameter_name() -> None:
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class ParamValue:
        param_id = "CAL_MAG4_ID"
        param_value = 9.0
        param_type = 9

        @staticmethod
        def get_type() -> str:
            return "PARAM_VALUE"

    class Mav:
        def __init__(self) -> None:
            self.requests: list[tuple[object, ...]] = []

        def param_request_read_send(self, *args: object) -> None:
            self.requests.append(args)

    class Connection:
        target_system = 1
        target_component = 1

        def __init__(self) -> None:
            self.mav = Mav()
            self.messages = [ParamValue()]

        def recv_match(self, **_kwargs):
            return self.messages.pop(0) if self.messages else None

    connection = Connection()
    subscriber = DualPixhawkMavlinkSubscriber(left_url="udp:fake", right_url="")
    subscriber.connections["left"] = connection

    assert subscriber.read_param("left", "CAL_MAG4_ID", timeout_s=0.1) == pytest.approx(9.0)
    assert connection.mav.requests == [(1, 1, b"CAL_MAG4_ID", -1)]


def test_common_param_receive_loop_preserves_integer_bits_and_unmatched_message() -> None:
    from Workstation.workstation_core import pixhawk_attitude

    class Message:
        def __init__(self, message_type: str, **values) -> None:
            self._message_type = message_type
            self.__dict__.update(values)

        def get_type(self) -> str:
            return self._message_type

    class Connection:
        def __init__(self) -> None:
            self.messages = [
                Message("ATTITUDE", roll=0.1),
                Message(
                    "PARAM_VALUE",
                    param_id="COUNT",
                    param_value=pixhawk_attitude._mavlink_int_to_param_float(123),
                    param_type=6,
                ),
            ]

        def recv_match(self, **_kwargs):
            return self.messages.pop(0) if self.messages else None

    unmatched: list[object] = []

    assert pixhawk_attitude._mavlink_recv_param_value(
        Connection(),
        "COUNT",
        timeout_s=0.1,
        on_unmatched=unmatched.append,
    ) == 123
    assert len(unmatched) == 1
    assert unmatched[0].get_type() == "ATTITUDE"


def test_duplicate_raw_mavlink_packet_is_ignored_before_signal_delivery() -> None:
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class Message:
        roll = 0.1
        pitch = 0.2
        yaw = 0.3
        time_boot_ms = 10

        def get_type(self):
            return "ATTITUDE"

        def get_msgbuf(self):
            return b"same-mavlink-packet"

        def get_srcSystem(self):
            return 1

        def get_srcComponent(self):
            return 1

        def get_seq(self):
            return 7

        def get_msgId(self):
            return 30

    class Connection:
        def __init__(self):
            self.messages = [Message(), Message()]

        def recv_match(self, **_kwargs):
            return self.messages.pop(0) if self.messages else None

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udp:fake", right_url="")
    subscriber.connections["left"] = Connection()
    attitudes = []
    subscriber.extrinsic_changed.connect(lambda _side, state: attitudes.append(state))

    subscriber.spin_once()

    assert len(attitudes) == 1


def test_repeated_mavlink_receive_error_is_rate_limited(monkeypatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    class Connection:
        def recv_match(self, **_kwargs):
            raise OSError("serial sentinel")

    now = {"value": 100.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: now["value"])
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udp:fake",
        right_url="",
    )
    subscriber.connections["left"] = Connection()
    statuses: list[str] = []
    subscriber.status_changed.connect(statuses.append)

    subscriber.spin_once()
    subscriber.spin_once()
    now["value"] += 1.0
    subscriber.spin_once()

    receive_errors = [status for status in statuses if "serial sentinel" in status]
    assert len(receive_errors) == 2


def test_successful_rtcm_status_is_bounded_per_side_during_frame_burst(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    clock = [100.0]
    sent: list[tuple[object, bytes]] = []
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(
        pixhawk_attitude,
        "_send_mavlink_rtcm_data",
        lambda connection, payload: sent.append((connection, payload)) or 2,
    )
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udp:left",
        right_url="udp:right",
    )
    subscriber.connections.update(left=object(), right=object())
    statuses: list[str] = []
    subscriber.status_changed.connect(statuses.append)

    for _index in range(100):
        subscriber.send_rtcm_data("left", b"left-frame")
        subscriber.send_rtcm_data("right", b"right-frame")

    assert len(sent) == 200
    assert len([text for text in statuses if "RTCM 주입" in text]) == 2
    assert any(text.startswith("왼쪽 RTCM 주입") for text in statuses)
    assert any(text.startswith("오른쪽 RTCM 주입") for text in statuses)

    clock[0] += pixhawk_attitude.MAVLINK_RTCM_STATUS_INTERVAL_S - 0.001
    subscriber.send_rtcm_data("left", b"still-throttled")
    subscriber.send_rtcm_data("right", b"still-throttled")
    assert len([text for text in statuses if "RTCM 주입" in text]) == 2

    clock[0] += 0.001
    subscriber.mavlink_health_snapshot = lambda _side, **_kwargs: SimpleNamespace(
        rtcm_eligible=True,
        connection_generation=7,
    )
    for _index in range(100):
        left = subscriber.send_rtcm_data_if_eligible(
            "left",
            b"next-window-left",
            expected_generation=7,
        )
        right = subscriber.send_rtcm_data_if_eligible(
            "right",
            b"next-window-right",
            expected_generation=7,
        )
        assert left["sent"] is True
        assert right["sent"] is True
    assert len([text for text in statuses if "RTCM 주입" in text]) == 4


def test_pixhawk_connect_disconnect_operations_do_not_overlap() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    logs: list[str] = []
    panel = SimpleNamespace(
        _session_recording_active=lambda: False,
        pixhawk_connection_thread=object(),
        pixhawk_disconnection_thread=object(),
        _log=logs.append,
    )
    controller = DeviceRuntimeController(panel)

    controller.connect_mavlink_pixhawk("left")
    controller.disconnect_mavlink_pixhawk("left")

    assert any("연결 해제가 끝난 뒤" in message for message in logs)
    assert any("연결 작업이 끝난 뒤" in message for message in logs)


def test_pixhawk_source_connection_is_allowed_only_for_automatic_measurement_recording() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    calls: list[str] = []
    panel = SimpleNamespace(
        _session_recording_active=lambda: True,
        _automatic_session_recording=False,
        _mavlink_open_sides=lambda: set(),
        connect_mavlink_pixhawk=lambda *_args: calls.append("connect"),
        disconnect_mavlink_pixhawk=lambda *_args: calls.append("disconnect"),
        _log=lambda message: calls.append(str(message)),
    )
    controller = DeviceRuntimeController(panel)

    assert controller.toggle_mavlink_connection("left") is False
    assert "connect" not in calls

    panel._automatic_session_recording = True
    assert controller.toggle_mavlink_connection("left") is None
    assert calls[-1] == "connect"


def test_all_rx_stall_does_not_cycle_host_socket_as_remote_recovery(monkeypatch) -> None:
    from Workstation import device_runtime_controller
    from Workstation.device_runtime_controller import DeviceRuntimeController
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    monkeypatch.setattr(device_runtime_controller.time, "monotonic", lambda: clock["value"])
    subscriber = DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
    )
    subscriber.connections["left"] = object()
    model = subscriber._ensure_liveness_listener("left")
    for message_type in ("HEARTBEAT", "ATTITUDE"):
        assert model.observe_packet(
            system_id=1,
            component_id=1,
            message_type=message_type,
            received_at_s=0.0,
            generation=model.connection_generation,
        )
    clock["value"] = 10.0

    logs: list[str] = []
    labels: list[str] = []
    refreshed: list[bool] = []
    panel = SimpleNamespace(
        mavlink_pixhawk=subscriber,
        pixhawk_disconnection_thread=None,
        _session_recording_active=lambda: False,
        _mavlink_target_sides=lambda _camera: ("left",),
        _mavlink_open_sides=lambda: {"left"},
        _px4_mag_camera_label=lambda side: "P1" if side == "left" else "P2",
        _log=logs.append,
        pixhawk_connection_label=SimpleNamespace(setText=labels.append),
        _refresh_mavlink_side_actions=lambda: refreshed.append(True),
    )
    controller = DeviceRuntimeController(panel)
    host_cycles: list[str] = []
    controller.disconnect_mavlink_pixhawk = lambda *_args, **_kwargs: host_cycles.append("disconnect")

    controller.connect_mavlink_pixhawk("left")

    assert host_cycles == []
    assert any("host UDP socket 재연결" in message for message in logs)
    assert labels == ["픽스호크 MAVLink: P1 ETH RX STALLED · RTCM/명령 차단"]
    assert refreshed == [True]


def _verified_usb_recovery_panel(*, now_s: float = 100.0) -> SimpleNamespace:
    stalled = SimpleNamespace(state=SimpleNamespace(value="ETH MAVLINK RX STALLED"))
    return SimpleNamespace(
        _px4_mag_camera_label=lambda side: "P1" if side == "left" else "P2",
        _session_recording_active=lambda: False,
        _measurement_run_state="IDLE",
        _session_save_thread=None,
        _measurement_package_finalize_requested=False,
        _session_save_retry_required=False,
        pixhawk_connection_thread=None,
        pixhawk_disconnection_thread=None,
        pixhawk_stream_setup_thread=None,
        pixhawk_usb_bootstrap_thread=None,
        pixhawk_persistent_config_thread=None,
        pixhawk_usb_discovery_thread=None,
        pixhawk_usb_recovery_thread=None,
        pixhawk_shell_threads={},
        pixhawk_parameter_threads={},
        mavlink_pixhawk=SimpleNamespace(
            mavlink_health_snapshot=lambda _side: stalled,
        ),
        pixhawk_usb_identity_evidence=[
            {
                "device": "/dev/serial/by-id/p1",
                "resolved_device": "/dev/ttyACM0",
                "baud": 115200,
                "system_id": 1,
                "component_id": 1,
                "observed_monotonic_s": now_s,
            }
        ],
        pixhawk_usb_recovery_status={},
        _log=lambda _message: None,
    )


def test_verified_usb_recovery_operator_no_sends_no_command(monkeypatch) -> None:
    from Workstation import device_runtime_controller
    from Workstation.device_runtime_controller import DeviceRuntimeController

    panel = _verified_usb_recovery_panel()
    logs: list[str] = []
    panel._log = logs.append
    monkeypatch.setattr(device_runtime_controller.time, "monotonic", lambda: 100.0)
    monkeypatch.setattr(
        device_runtime_controller.QtWidgets.QMessageBox,
        "question",
        lambda *_args, **_kwargs: device_runtime_controller.QtWidgets.QMessageBox.No,
    )

    started = DeviceRuntimeController(panel).recover_stalled_pixhawk_via_verified_usb("left")

    assert started is False
    assert panel.pixhawk_usb_recovery_thread is None
    assert panel.pixhawk_usb_recovery_status["left"]["state"] == "CANCELLED"
    assert any("취소" in message for message in logs)


@pytest.mark.parametrize("mode", ("duplicate", "stale", "missing"))
def test_verified_usb_recovery_ambiguous_or_stale_identity_fails_closed(monkeypatch, mode: str) -> None:
    from Workstation import device_runtime_controller
    from Workstation.device_runtime_controller import DeviceRuntimeController

    panel = _verified_usb_recovery_panel()
    if mode == "duplicate":
        panel.pixhawk_usb_identity_evidence.append(
            {
                **panel.pixhawk_usb_identity_evidence[0],
                "device": "/dev/serial/by-id/p1-duplicate",
                "resolved_device": "/dev/ttyACM1",
            }
        )
    elif mode == "stale":
        panel.pixhawk_usb_identity_evidence[0]["observed_monotonic_s"] = 60.0
    else:
        panel.pixhawk_usb_identity_evidence = []
    monkeypatch.setattr(device_runtime_controller.time, "monotonic", lambda: 100.0)
    monkeypatch.setattr(
        device_runtime_controller.QtWidgets.QMessageBox,
        "question",
        lambda *_args, **_kwargs: pytest.fail("confirmation must not open without unique fresh identity"),
    )

    assert DeviceRuntimeController(panel).recover_stalled_pixhawk_via_verified_usb("left") is False
    assert panel.pixhawk_usb_recovery_status["left"]["state"] == "ERROR"
    assert panel.pixhawk_usb_recovery_thread is None


@pytest.mark.parametrize(
    ("attribute", "value", "reason"),
    (
        ("_measurement_run_state", "RUNNING", "Measurement lifecycle"),
        ("_session_save_thread", object(), "save/finalization"),
        ("_measurement_package_finalize_requested", True, "finalization"),
        ("pixhawk_shell_threads", {"left": object()}, "shell/reboot"),
    ),
)
def test_verified_usb_recovery_lifecycle_guards_fail_before_confirmation(
    monkeypatch,
    attribute: str,
    value: object,
    reason: str,
) -> None:
    from Workstation import device_runtime_controller
    from Workstation.device_runtime_controller import DeviceRuntimeController

    panel = _verified_usb_recovery_panel()
    setattr(panel, attribute, value)
    logs: list[str] = []
    panel._log = logs.append
    monkeypatch.setattr(device_runtime_controller.time, "monotonic", lambda: 100.0)
    monkeypatch.setattr(
        device_runtime_controller.QtWidgets.QMessageBox,
        "question",
        lambda *_args, **_kwargs: pytest.fail("unsafe lifecycle must fail before confirmation"),
    )

    assert DeviceRuntimeController(panel).recover_stalled_pixhawk_via_verified_usb("left") is False
    assert any(reason in message for message in logs)


def test_usb_command_sent_is_not_recovery_or_live_evidence() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    panel = _verified_usb_recovery_panel()
    logs: list[str] = []
    panel._log = logs.append
    controller = DeviceRuntimeController(panel)

    controller._apply_pixhawk_usb_recovery_result(
        {
            "camera": "left",
            "device": "/dev/serial/by-id/p1",
            "system_id": 1,
            "component_id": 1,
            "command_sent": True,
        }
    )

    assert panel.pixhawk_usb_recovery_status["left"]["state"] == "COMMAND_SENT"
    assert panel.mavlink_pixhawk.mavlink_health_snapshot("left").state.value == "ETH MAVLINK RX STALLED"
    assert any("command send만으로 LIVE/READY로 처리하지 않습니다" in message for message in logs)


def test_stalled_ethernet_connection_is_never_used_for_reboot_command(monkeypatch) -> None:
    from Workstation import device_runtime_controller
    from Workstation.device_runtime_controller import DeviceRuntimeController

    logs: list[str] = []
    panel = SimpleNamespace(
        mavlink_pixhawk=SimpleNamespace(
            connections={"left": object()},
            mavlink_health_snapshot=lambda _side: SimpleNamespace(
                state=SimpleNamespace(value="ETH MAVLINK RX STALLED"),
                command_eligible=False,
            ),
        ),
        _log=logs.append,
    )
    monkeypatch.setattr(
        device_runtime_controller.QtWidgets.QMessageBox,
        "question",
        lambda *_args, **_kwargs: pytest.fail("stalled Ethernet command must fail before confirmation"),
    )

    DeviceRuntimeController(panel).reboot_pixhawk_from_mavlink("left")

    assert any("verified-USB recovery" in message for message in logs)


def test_alignment_refresh_never_overwrites_the_canonical_rtk_baseline() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    canonical = SimpleNamespace(
        baseline_m=1.0,
        north_m=0.0,
        east_m=1.0,
        down_m=0.0,
        fix_quality="GOOD",
        warnings=(),
    )
    vector = SimpleNamespace(
        east_m=1.0,
        north_m=0.0,
        up_m=0.0,
        length_m=1.0,
        azimuth_deg=90.0,
        elevation_deg=0.0,
        altitude_delta_m=0.0,
        timestamp_delta_ms=10.0,
    )
    left = SimpleNamespace(timestamp_s=10.0, fix_type=6)
    right = SimpleNamespace(timestamp_s=10.01, fix_type=6)
    plotted = []
    panel = SimpleNamespace(
        latest_rtk_baseline=canonical,
        latest_baseline_vector=None,
        latest_p1_body_frd_baseline=None,
        latest_extrinsics={},
        rtk_alignment_plot=SimpleNamespace(
            set_alignment=lambda baseline, *_args: plotted.append(baseline)
        ),
        rtk_alignment_detail_label=SimpleNamespace(setText=lambda _text: None),
        _refresh_baseline_gps_cards=lambda: None,
        _current_baseline_vector=lambda: (vector, left, right),
        _refresh_p1_body_frd_baseline=lambda: None,
        _alignment_view_camera_translation_right_in_left_m=lambda baseline: None,
        _baseline_camera_translation_in_side_rdf_m=lambda baseline, side: None,
        _baseline_forward_geometry_from_camera_translation=lambda translation: None,
        _baseline_forward_geometry_text=lambda translation: "waiting",
        _set_label_text_if_changed=lambda *args: None,
        _set_p1_body_frd_baseline_labels=lambda projection: None,
        _refresh_alignment_overview=lambda: None,
        _refresh_alignment_validation=lambda: None,
    )

    WirisStereoDistancePanel._refresh_baseline_alignment_ui(panel)

    assert panel.latest_rtk_baseline is canonical
    assert plotted == [canonical]


def test_gnss_source_change_resets_receiver_cache_and_fixed_duration() -> None:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    panel = SimpleNamespace(
        settings=SimpleNamespace(
            pixhawk_left_gnss_source="GPS1",
            pixhawk_right_gnss_source="GPS1",
        ),
        _workstation_setting_overrides={},
        gnss_monitor=SimpleNamespace(left_source="GPS1", right_source="GPS1"),
        _measurement_latest_valid_gps_source={"left": "GPS1"},
        gnss_rover_fixed_since={"left": 10.0, "right": 20.0, "drone": None},
    )

    selected = WirisStereoDistancePanel.set_pixhawk_gnss_source(
        panel, "left", "GPS2", persist=False
    )

    assert selected == "GPS2"
    assert panel.gnss_monitor.left_source == "GPS2"
    assert "left" not in panel._measurement_latest_valid_gps_source
    assert panel.gnss_rover_fixed_since["left"] is None

def test_workstation_mavlink_packet_provenance_reaches_state_and_timeline() -> None:
    from Workstation.workstation_core.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
        _mavlink_optional_timeline_row,
    )
    from Workstation.workstation_core.measurement_attempt import AttitudeSnapshot, GpsSnapshot

    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    attitudes = []
    gps_states = []
    subscriber.extrinsic_changed.connect(lambda _camera, state: attitudes.append(state))
    subscriber.gps_changed.connect(lambda _camera, state: gps_states.append(state))

    attitude_message = SimpleNamespace(
        get_type=lambda: "ATTITUDE",
        get_srcSystem=lambda: 11,
        get_srcComponent=lambda: 42,
        get_seq=lambda: 7,
        get_msgId=lambda: 30,
        roll=0.1,
        pitch=0.2,
        yaw=0.3,
        rollspeed=0.0,
        pitchspeed=0.0,
        yawspeed=0.0,
        time_boot_ms=123,
    )
    subscriber._handle_mavlink_message("left", attitude_message)
    assert len(attitudes) == 1
    assert attitudes[0].mavlink_source_system == 11
    assert attitudes[0].mavlink_source_component == 42
    assert attitudes[0].mavlink_sequence == 7
    assert attitudes[0].mavlink_message_id == 30
    attitude_snapshot = AttitudeSnapshot.from_state(attitudes[0])
    assert attitude_snapshot.mavlink_sequence == 7

    gps_message = SimpleNamespace(
        get_type=lambda: "GPS_RAW_INT",
        get_srcSystem=lambda: 11,
        get_srcComponent=lambda: 42,
        get_seq=lambda: 8,
        get_msgId=lambda: 24,
        lat=370000000,
        lon=1270000000,
        alt=10000,
        fix_type=6,
        satellites_visible=18,
        eph=50,
        epv=60,
        cog=9000,
        time_usec=1_800_000_000_000_000,
        yaw=0,
        hdg_acc=0,
    )
    subscriber._handle_mavlink_message("left", gps_message)
    assert len(gps_states) == 1
    assert gps_states[0].mavlink_source_system == 11
    assert gps_states[0].mavlink_source_component == 42
    assert gps_states[0].mavlink_sequence == 8
    assert gps_states[0].mavlink_message_id == 24
    gps_snapshot = GpsSnapshot.from_state(gps_states[0])
    assert gps_snapshot.mavlink_sequence == 8

    timeline_row = _mavlink_optional_timeline_row(
        "GLOBAL_POSITION_INT",
        SimpleNamespace(
            get_srcSystem=lambda: 11,
            get_srcComponent=lambda: 42,
            get_seq=lambda: 9,
            get_msgId=lambda: 33,
            lat=370000000,
            lon=1270000000,
            alt=10000,
            relative_alt=9000,
            vx=1,
            vy=2,
            vz=3,
            hdg=9000,
            time_boot_ms=123,
        ),
        10.0,
    )
    assert timeline_row is not None
    assert timeline_row["mavlink_source_system"] == 11
    assert timeline_row["mavlink_source_component"] == 42
    assert timeline_row["mavlink_sequence"] == 9
    assert timeline_row["mavlink_message_id"] == 33


def test_mavlink_dual_gps_decodes_source_specific_altitude_and_accuracy() -> None:
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    gps_states: list[tuple[str, object]] = []
    subscriber.gps_changed.connect(lambda camera, state: gps_states.append((camera, state)))

    def message(message_type: str, message_id: int):
        return SimpleNamespace(
            get_type=lambda: message_type,
            get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 1,
            get_seq=lambda: message_id,
            get_msgId=lambda: message_id,
            lat=370000000,
            lon=1270000000,
            alt=10000,
            alt_ellipsoid=30000,
            fix_type=5,
            satellites_visible=20,
            # PX4 stream semantics: eph/epv are HDOP/VDOP x100.
            eph=250,
            epv=300,
            h_acc=450,
            v_acc=800,
            cog=9000,
            time_usec=1_800_000_000_000_000,
            yaw=0,
            hdg_acc=0,
        )

    subscriber._handle_mavlink_message("left", message("GPS_RAW_INT", 24))
    subscriber._handle_mavlink_message("left", message("GPS2_RAW", 124))

    assert [camera for camera, _state in gps_states] == ["left", "left_GPS2"]
    primary = gps_states[0][1]
    secondary = gps_states[1][1]
    assert primary.hdop == pytest.approx(2.5)
    assert primary.vdop == pytest.approx(3.0)
    assert primary.eph_m == pytest.approx(0.45)
    assert primary.epv_m == pytest.approx(0.80)
    assert primary.altitude_ellipsoid_m == pytest.approx(30.0)
    assert secondary.topic == "mavlink/GPS2_RAW"


def test_gnss_monitor_selects_the_requested_receiver_source() -> None:
    from Workstation.workstation_core.alignment_managers import GNSSMonitor
    from Workstation.workstation_core.rtk_geometry import GpsPositionState

    def state(camera: str, altitude: float, topic: str) -> GpsPositionState:
        return GpsPositionState(
            camera=camera,
            namespace="p1" if camera == "left" else "p2",
            timestamp_s=10.0,
            topic=topic,
            latitude_deg=37.0,
            longitude_deg=127.0,
            altitude_m=altitude,
            fix_type=6,
        )

    monitor = GNSSMonitor()
    monitor.update_position("left", state("left", 100.0, "mavlink/GPS_RAW_INT"))
    monitor.update_position("left_GPS2", state("left", 110.0, "mavlink/GPS2_RAW"))
    monitor.update_position("right", state("right", 101.0, "mavlink/GPS_RAW_INT"))
    monitor.update_position("right_GPS2", state("right", 111.0, "mavlink/GPS2_RAW"))

    selected = monitor.current_baseline()
    assert selected is not None
    assert selected[1].altitude_m == pytest.approx(100.0)
    assert selected[2].altitude_m == pytest.approx(101.0)

    monitor.left_source = "GPS2"
    monitor.right_source = "GPS2"
    selected = monitor.current_baseline()
    assert selected is not None
    assert selected[1].altitude_m == pytest.approx(110.0)
    assert selected[2].altitude_m == pytest.approx(111.0)


def test_pixhawk_gnss_source_selection_persists(tmp_path) -> None:
    from Workstation.workstation_core.settings import AppSettings, load_settings, save_settings

    path = tmp_path / "settings.json"
    save_settings(
        AppSettings(pixhawk_left_gnss_source="GPS2", pixhawk_right_gnss_source="GPS1"),
        path,
    )
    loaded = load_settings(path)
    assert loaded.pixhawk_left_gnss_source == "GPS2"
    assert loaded.pixhawk_right_gnss_source == "GPS1"


def test_measurement_gps_cache_drops_previous_receiver_after_source_change() -> None:
    from Workstation.measurement_view_controller import MeasurementViewController
    from Workstation.workstation_core.rtk_geometry import GpsPositionState

    gps1 = GpsPositionState(
        camera="left",
        namespace="p1",
        timestamp_s=10.0,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_m=100.0,
        fix_type=6,
    )
    source = {"left": "GPS1", "right": "GPS1"}
    current = {"left": gps1, "right": gps1}
    panel = SimpleNamespace(
        _selected_baseline_gps_states=lambda: (current["left"], current["right"]),
        _pixhawk_gnss_source=lambda side: source[side],
        _measurement_started_monotonic_s=None,
    )
    controller = MeasurementViewController(panel)
    first_left, _first_right, _ = controller._cached_measurement_gps_states()
    assert first_left is gps1

    source["left"] = "GPS2"
    current["left"] = None
    selected_left, _selected_right, _ = controller._cached_measurement_gps_states()
    assert selected_left is None


def test_stock_stream_profile_is_exact_and_private_stream_is_not_retried(monkeypatch: pytest.MonkeyPatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
        MAVLINK_STOCK_STREAM_CONSUMERS,
        MAVLINK_STOCK_STREAM_REQUESTS,
    )

    clock = [100.0]
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock[0])

    class FakeMav:
        def __init__(self) -> None:
            self.calls: list[tuple[object, ...]] = []

        def command_long_send(self, *args: object) -> None:
            self.calls.append(args)

    mav = FakeMav()
    connection = SimpleNamespace(
        mav=mav,
        mavlink=SimpleNamespace(MAV_CMD_SET_MESSAGE_INTERVAL=511),
        target_system=1,
        target_component=1,
    )
    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    subscriber.connections["left"] = connection

    subscriber._request_mavlink_sensor_streams("left")
    initial_count = len(MAVLINK_STOCK_STREAM_REQUESTS)
    assert len(mav.calls) == initial_count
    requested_names = tuple(name for name, _rate in MAVLINK_STOCK_STREAM_REQUESTS)
    assert requested_names == (
        "ATTITUDE",
        "SYS_STATUS",
        "GPS_RAW_INT",
        "GPS2_RAW",
        "GPS_RTK",
        "GPS2_RTK",
        "SCALED_IMU",
        "SCALED_IMU2",
        "SCALED_IMU3",
        "DEBUG_VECT",
        "RAW_IMU",
        "LOCAL_POSITION_NED",
        "GLOBAL_POSITION_INT",
        "VFR_HUD",
        "BATTERY_STATUS",
    )
    assert set(MAVLINK_STOCK_STREAM_CONSUMERS) == set(requested_names)
    assert all(MAVLINK_STOCK_STREAM_CONSUMERS[name] for name in requested_names)
    assert "SASL_CAN_MAG" not in requested_names
    assert 180 not in {int(call[4]) for call in mav.calls}

    clock[0] += 10.0
    subscriber._request_mavlink_sensor_streams("left")
    assert len(mav.calls) == initial_count


def test_stock_stream_profile_burst_uses_the_side_send_lock() -> None:
    from Workstation.workstation_core.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
        MAVLINK_STOCK_STREAM_REQUESTS,
    )

    class Guard:
        def __init__(self) -> None:
            self.active = False
            self.entries = 0

        def __enter__(self):
            assert not self.active
            self.active = True
            self.entries += 1
            return self

        def __exit__(self, *_args: object) -> None:
            self.active = False

    guard = Guard()
    calls: list[tuple[object, ...]] = []

    class Mav:
        def command_long_send(self, *args: object) -> None:
            assert guard.active
            calls.append(args)

    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    subscriber.connections["left"] = SimpleNamespace(
        mav=Mav(),
        mavlink=SimpleNamespace(MAV_CMD_SET_MESSAGE_INTERVAL=511),
        target_system=1,
        target_component=1,
    )
    subscriber._mavlink_send_locks["left"] = guard

    subscriber._request_mavlink_sensor_streams("left")

    assert guard.entries == 1
    assert not guard.active
    assert len(calls) == len(MAVLINK_STOCK_STREAM_REQUESTS)


def test_remote_px4_reboot_rearms_only_affected_stock_profile_on_next_heartbeat(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.mavlink_liveness import MavlinkLivenessState
    from Workstation.workstation_core.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
        MAVLINK_STOCK_STREAM_REQUESTS,
    )

    clock = [100.0]
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock[0])

    class Message:
        def __init__(self, message_type: str, system_id: int, **values: object) -> None:
            self._message_type = message_type
            self._system_id = system_id
            self.__dict__.update(values)

        def get_type(self) -> str:
            return self._message_type

        def get_srcSystem(self) -> int:
            return self._system_id

        def get_srcComponent(self) -> int:
            return 1

    class Mav:
        def __init__(self) -> None:
            self.commands: list[tuple[object, ...]] = []

        def command_long_send(self, *args: object) -> None:
            self.commands.append(args)

    def connection(system_id: int) -> SimpleNamespace:
        return SimpleNamespace(
            mav=Mav(),
            mavlink=SimpleNamespace(MAV_CMD_SET_MESSAGE_INTERVAL=511),
            target_system=system_id,
            target_component=1,
        )

    subscriber = DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="udpin:0.0.0.0:14551",
    )
    subscriber.connections.update({"left": connection(1), "right": connection(2)})
    statuses: list[str] = []
    setup_requests: list[str] = []
    left_attitudes: list[object] = []
    subscriber.status_changed.connect(statuses.append)
    subscriber.stream_setup_required.connect(setup_requests.append)
    subscriber.extrinsic_changed.connect(
        lambda side, state: left_attitudes.append(state) if side == "left" else None
    )

    def deliver(side: str, message: Message) -> None:
        assert subscriber._observe_mavlink_transport_message(
            side,
            message,
            received_at_s=clock[0],
        )
        subscriber._handle_mavlink_message(side, message, received_at_s=clock[0])

    def heartbeat(side: str, system_id: int) -> None:
        deliver(side, Message("HEARTBEAT", system_id))

    def attitude(side: str, system_id: int, boot_ms: int) -> None:
        deliver(
            side,
            Message(
                "ATTITUDE",
                system_id,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                time_boot_ms=boot_ms,
            ),
        )

    heartbeat("left", 1)
    attitude("left", 1, 120_000)
    heartbeat("right", 2)
    attitude("right", 2, 220_000)
    for side in ("left", "right"):
        subscriber._mavlink_liveness[side].qualification_s = 1.0

    clock[0] = 101.1
    heartbeat("left", 1)
    attitude("left", 1, 121_100)
    heartbeat("right", 2)
    attitude("right", 2, 221_100)
    assert subscriber.mavlink_health_snapshot("left").state is MavlinkLivenessState.LIVE
    assert subscriber.mavlink_health_snapshot("right").state is MavlinkLivenessState.LIVE

    profile_size = len(MAVLINK_STOCK_STREAM_REQUESTS)
    left_mav = subscriber.connections["left"].mav
    right_mav = subscriber.connections["right"].mav
    assert len(left_mav.commands) == profile_size
    assert len(right_mav.commands) == profile_size
    left_generation = subscriber.mavlink_health_snapshot("left").connection_generation
    right_generation = subscriber.mavlink_health_snapshot("right").connection_generation
    subscriber._mavlink_stream_configured.update(("left", "right"))
    subscriber._stream_setup_requests_sent.update(("left", "right"))
    statuses.clear()

    # One low sample is only a candidate; the second increasing low sample is
    # authoritative reboot evidence. No profile command is sent from ATTITUDE.
    clock[0] = 102.0
    attitude("left", 1, 100)
    assert "left" in subscriber._mavlink_stream_requests_sent
    clock[0] = 102.05
    attitude("left", 1, 133)

    assert "left" not in subscriber._mavlink_stream_requests_sent
    assert "right" in subscriber._mavlink_stream_requests_sent
    assert len(left_mav.commands) == profile_size
    assert len(right_mav.commands) == profile_size
    left_health = subscriber.mavlink_health_snapshot("left")
    right_health = subscriber.mavlink_health_snapshot("right")
    # A remote boot invalidates PX4 evidence but does not replace the host UDP
    # listener, so its host connection generation remains stable.
    assert left_health.connection_generation == left_generation
    assert left_health.state is MavlinkLivenessState.CONNECTING
    assert right_health.connection_generation == right_generation
    assert right_health.state is MavlinkLivenessState.LIVE
    assert subscriber._mavlink_stream_configured == {"left", "right"}
    assert subscriber._stream_setup_requests_sent == {"left", "right"}
    assert setup_requests == []

    # The next expected heartbeat owns one re-request. A delayed packet from
    # the prior boot and repeated new-boot samples must not create a storm.
    clock[0] = 102.1
    heartbeat("left", 1)
    assert len(left_mav.commands) == profile_size * 2
    assert len(right_mav.commands) == profile_size
    assert subscriber.mavlink_health_snapshot("left").state is MavlinkLivenessState.RECOVERING

    clock[0] = 102.15
    attitude("left", 1, 121_050)
    clock[0] = 102.2
    attitude("left", 1, 166)
    clock[0] = 102.25
    attitude("left", 1, 199)
    clock[0] = 103.0
    heartbeat("left", 1)

    assert len(left_mav.commands) == profile_size * 2
    assert len(right_mav.commands) == profile_size
    reboot_statuses = [text for text in statuses if "PX4 재부팅 감지" in text]
    assert len(reboot_statuses) == 1
    assert all(int(call[2]) == 511 for call in left_mav.commands)
    assert len(left_attitudes) == 6

    # The latch belongs to one boot transition, not the lifetime of the host
    # socket. A later genuine reboot gets one new rearm as well.
    for observed_s, boot_ms in ((107.0, 4_200), (111.0, 8_200), (115.0, 12_200)):
        clock[0] = observed_s
        attitude("left", 1, boot_ms)
        clock[0] += 0.01
        heartbeat("left", 1)
    assert len(left_mav.commands) == profile_size * 2

    clock[0] = 116.0
    attitude("left", 1, 100)
    clock[0] = 116.05
    attitude("left", 1, 133)
    assert "left" not in subscriber._mavlink_stream_requests_sent
    clock[0] = 116.1
    heartbeat("left", 1)

    assert len(left_mav.commands) == profile_size * 3
    assert len(right_mav.commands) == profile_size
    assert len([text for text in statuses if "PX4 재부팅 감지" in text]) == 2
    assert subscriber.mavlink_health_snapshot("right").connection_generation == right_generation
    assert setup_requests == []


def test_rapid_remote_reboot_beyond_reorder_tolerance_rearms_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
        MAVLINK_STOCK_STREAM_REQUESTS,
    )

    clock = [10.0]
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock[0])

    class Message:
        def __init__(self, message_type: str, **values: object) -> None:
            self._message_type = message_type
            self.__dict__.update(values)

        def get_type(self) -> str:
            return self._message_type

        def get_srcSystem(self) -> int:
            return 1

        def get_srcComponent(self) -> int:
            return 1

    class Mav:
        def __init__(self) -> None:
            self.commands: list[tuple[object, ...]] = []

        def command_long_send(self, *args: object) -> None:
            self.commands.append(args)

    mav = Mav()
    subscriber = DualPixhawkMavlinkSubscriber(left_url="udp:test", right_url="")
    subscriber.connections["left"] = SimpleNamespace(
        mav=mav,
        mavlink=SimpleNamespace(MAV_CMD_SET_MESSAGE_INTERVAL=511),
        target_system=1,
        target_component=1,
    )
    statuses: list[str] = []
    subscriber.status_changed.connect(statuses.append)

    def deliver(message: Message, at_s: float) -> None:
        clock[0] = at_s
        assert subscriber._observe_mavlink_transport_message(
            "left", message, received_at_s=at_s
        )
        subscriber._handle_mavlink_message("left", message, received_at_s=at_s)

    def attitude(boot_ms: int, at_s: float) -> None:
        deliver(
            Message(
                "ATTITUDE",
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                time_boot_ms=boot_ms,
            ),
            at_s,
        )

    deliver(Message("HEARTBEAT"), 10.0)
    attitude(5_000, 10.01)
    profile_size = len(MAVLINK_STOCK_STREAM_REQUESTS)
    assert len(mav.commands) == profile_size
    statuses.clear()

    attitude(100, 10.1)
    attitude(133, 10.15)
    assert "left" not in subscriber._mavlink_stream_requests_sent
    assert len(mav.commands) == profile_size

    deliver(Message("HEARTBEAT"), 10.2)
    attitude(166, 10.25)
    attitude(199, 10.3)
    deliver(Message("HEARTBEAT"), 10.8)

    assert len(mav.commands) == profile_size * 2
    assert len([text for text in statuses if "PX4 재부팅 감지" in text]) == 1


@pytest.mark.parametrize(
    ("initial_boot_ms", "later_boot_ms"),
    (
        (100_000, (99_001, 100_033)),
        (100_000, (98_000, 100_033)),
        ((1 << 32) - 16, (25, 58)),
    ),
    ids=("small-out-of-order-regression", "exact-reorder-tolerance", "uint32-wrap"),
)
def test_attitude_boot_time_reorder_and_uint32_wrap_do_not_rearm_profile(
    monkeypatch: pytest.MonkeyPatch,
    initial_boot_ms: int,
    later_boot_ms: tuple[int, int],
) -> None:
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
        MAVLINK_STOCK_STREAM_REQUESTS,
    )

    clock = [10.0]
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock[0])

    class Message:
        def __init__(self, message_type: str, **values: object) -> None:
            self._message_type = message_type
            self.__dict__.update(values)

        def get_type(self) -> str:
            return self._message_type

        def get_srcSystem(self) -> int:
            return 1

        def get_srcComponent(self) -> int:
            return 1

    class Mav:
        def __init__(self) -> None:
            self.commands: list[tuple[object, ...]] = []

        def command_long_send(self, *args: object) -> None:
            self.commands.append(args)

    mav = Mav()
    subscriber = DualPixhawkMavlinkSubscriber(left_url="udp:test", right_url="")
    subscriber.connections["left"] = SimpleNamespace(
        mav=mav,
        mavlink=SimpleNamespace(MAV_CMD_SET_MESSAGE_INTERVAL=511),
        target_system=1,
        target_component=1,
    )
    statuses: list[str] = []
    subscriber.status_changed.connect(statuses.append)

    def deliver(message: Message) -> None:
        assert subscriber._observe_mavlink_transport_message(
            "left", message, received_at_s=clock[0]
        )
        subscriber._handle_mavlink_message("left", message, received_at_s=clock[0])

    deliver(Message("HEARTBEAT"))
    deliver(
        Message(
            "ATTITUDE",
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            time_boot_ms=initial_boot_ms,
        )
    )
    profile_size = len(MAVLINK_STOCK_STREAM_REQUESTS)
    assert len(mav.commands) == profile_size
    statuses.clear()

    for offset, boot_ms in enumerate(later_boot_ms, start=1):
        clock[0] += 0.05
        deliver(
            Message(
                "ATTITUDE",
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                time_boot_ms=boot_ms,
            )
        )
    clock[0] += 0.5
    deliver(Message("HEARTBEAT"))

    assert len(mav.commands) == profile_size
    assert "left" in subscriber._mavlink_stream_requests_sent
    assert not any("PX4 재부팅 감지" in text for text in statuses)


def test_boot_time_regression_after_uint32_half_range_is_still_a_reboot() -> None:
    from Workstation.workstation_core.pixhawk_attitude import _MavlinkBootTimeTracker

    tracker = _MavlinkBootTimeTracker()
    assert tracker.observe(3_000_000_000, received_at_s=10.0).reboot is None
    assert tracker.observe(100, received_at_s=10.05).reboot is None

    confirmed = tracker.observe(133, received_at_s=10.1)

    assert confirmed.reboot is not None
    assert confirmed.reboot.previous_time_boot_ms == 3_000_000_000
    assert confirmed.reboot.current_time_boot_ms == 133


def test_replaced_host_connection_starts_a_fresh_boot_time_baseline(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
        MAVLINK_STOCK_STREAM_REQUESTS,
    )

    clock = [10.0]
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock[0])

    class Message:
        def __init__(self, message_type: str, **values: object) -> None:
            self._message_type = message_type
            self.__dict__.update(values)

        def get_type(self) -> str:
            return self._message_type

        def get_srcSystem(self) -> int:
            return 1

        def get_srcComponent(self) -> int:
            return 1

    class Mav:
        def __init__(self) -> None:
            self.commands: list[tuple[object, ...]] = []

        def command_long_send(self, *args: object) -> None:
            self.commands.append(args)

    class Connection:
        target_system = 1
        target_component = 1

        def __init__(self) -> None:
            self.mav = Mav()
            self.mavlink = SimpleNamespace(MAV_CMD_SET_MESSAGE_INTERVAL=511)
            self.closed = False

        def close(self) -> None:
            self.closed = True

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udp:test", right_url="")
    statuses: list[str] = []
    subscriber.status_changed.connect(statuses.append)

    def deliver(message: Message, received_at_s: float) -> None:
        clock[0] = received_at_s
        assert subscriber._observe_mavlink_transport_message(
            "left", message, received_at_s=received_at_s
        )
        subscriber._handle_mavlink_message(
            "left", message, received_at_s=received_at_s
        )

    original = Connection()
    subscriber.connections["left"] = original
    deliver(Message("HEARTBEAT"), 10.0)
    deliver(
        Message(
            "ATTITUDE",
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            time_boot_ms=100_000,
        ),
        10.1,
    )
    original_generation = subscriber.mavlink_health_snapshot(
        "left", now_s=10.1
    ).connection_generation
    subscriber.stop("left")
    assert original.closed

    replacement = Connection()
    subscriber.connections["left"] = replacement
    statuses.clear()
    for received_at_s, boot_ms in ((20.0, 100), (20.05, 133)):
        deliver(
            Message(
                "ATTITUDE",
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
                time_boot_ms=boot_ms,
            ),
            received_at_s,
        )
    deliver(Message("HEARTBEAT"), 20.1)

    assert len(replacement.mav.commands) == len(MAVLINK_STOCK_STREAM_REQUESTS)
    assert not any("PX4 재부팅 감지" in text for text in statuses)
    assert subscriber.mavlink_health_snapshot(
        "left", now_s=20.1
    ).connection_generation == original_generation + 1


def test_device_runtime_passes_and_syncs_explicit_pixhawk_peer_hosts() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    panel = SimpleNamespace(
        mavlink_pixhawk=None,
        left_mavlink_url_edit=SimpleNamespace(text=lambda: "udpin:0.0.0.0:14550"),
        right_mavlink_url_edit=SimpleNamespace(text=lambda: "udpin:0.0.0.0:14551"),
        left_mavlink_baud_spin=SimpleNamespace(value=lambda: 57600),
        right_mavlink_baud_spin=SimpleNamespace(value=lambda: 57600),
        left_pixhawk_eth_ip_edit=SimpleNamespace(text=lambda: "10.0.0.2"),
        right_pixhawk_eth_ip_edit=SimpleNamespace(text=lambda: "10.0.0.3"),
        custom_watch_specs=[],
        _update_extrinsic=lambda *_args: None,
        _update_gps=lambda *_args: None,
        _update_mag_sensor=lambda *_args: None,
        _update_gps_sensor=lambda *_args: None,
        _update_gnss_relative=lambda *_args: None,
        _update_custom_watch=lambda *_args: None,
        _update_mavlink_optional_timeline=lambda *_args: None,
        _handle_mavlink_stream_setup_required=lambda *_args: None,
        _append_px4_shell_output=lambda *_args: None,
        _handle_mavlink_status_changed=lambda *_args: None,
    )
    controller = DeviceRuntimeController(panel)

    subscriber = controller._ensure_mavlink_subscriber()

    assert subscriber.peer_hosts == {"left": "10.0.0.2", "right": "10.0.0.3"}
    panel.left_pixhawk_eth_ip_edit = SimpleNamespace(text=lambda: "10.0.0.12")
    panel.right_pixhawk_eth_ip_edit = SimpleNamespace(text=lambda: "10.0.0.13")
    controller._sync_mavlink_subscriber_settings()
    assert subscriber.peer_hosts == {"left": "10.0.0.12", "right": "10.0.0.13"}


def test_automatic_calibration_param_refresh_waits_for_live_qualification() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    health = {"left": "QUALIFYING", "right": "MAVLINK INITIAL RX TIMEOUT"}

    class Subscriber:
        connections = {"left": object(), "right": object()}

        @staticmethod
        def mavlink_health_snapshot(side: str):
            return SimpleNamespace(state=SimpleNamespace(value=health[side]))

    started: list[tuple[str, str]] = []
    panel = SimpleNamespace(
        mavlink_pixhawk=Subscriber(),
        _px4_calibration_auto_refreshed_sides=set(),
        _start_pixhawk_parameter_worker=lambda side, operation: started.append((side, operation)),
    )
    controller = DeviceRuntimeController(panel)

    controller._auto_refresh_px4_calibration_params_when_live()
    assert started == []
    assert panel._px4_calibration_auto_refreshed_sides == set()

    health["left"] = "LIVE"
    controller._auto_refresh_px4_calibration_params_when_live()
    assert started == [("left", "calibration_refresh")]
    assert panel._px4_calibration_auto_refreshed_sides == {"left"}

    controller._auto_refresh_px4_calibration_params_when_live()
    assert started == [("left", "calibration_refresh")]


def test_all_rx_stall_blocks_initial_and_attitude_stream_requests(monkeypatch: pytest.MonkeyPatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    clock = [100.0]
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock[0])

    class FakeMav:
        def __init__(self) -> None:
            self.calls: list[tuple[object, ...]] = []

        def command_long_send(self, *args: object) -> None:
            self.calls.append(args)

    mav = FakeMav()
    connection = SimpleNamespace(
        mav=mav,
        mavlink=SimpleNamespace(MAV_CMD_SET_MESSAGE_INTERVAL=511),
        target_system=1,
        target_component=1,
    )
    subscriber = DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
    )
    subscriber.connections["left"] = connection
    model = subscriber._ensure_liveness_listener("left")
    generation = model.connection_generation
    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="HEARTBEAT",
        received_at_s=clock[0],
        generation=generation,
    )

    clock[0] += 10.0
    subscriber._request_mavlink_sensor_streams("left")
    subscriber._request_stale_attitude_stream("left")

    assert mav.calls == []


def test_udpin_exclusive_guard_rejects_an_existing_listener() -> None:
    import errno

    from Workstation.workstation_core.pixhawk_attitude import (
        _assert_mavlink_udpin_available,
    )

    class Probe:
        def __init__(self, *, fail: bool) -> None:
            self.fail = fail
            self.closed = False

        def bind(self, _endpoint) -> None:
            if self.fail:
                raise OSError(errno.EADDRINUSE, "already bound")

        def close(self) -> None:
            self.closed = True

    blocked = Probe(fail=True)
    with pytest.raises(RuntimeError, match="already in use"):
        _assert_mavlink_udpin_available(
            "udpin:127.0.0.1:14550",
            socket_factory=lambda *_args: blocked,
        )
    assert blocked.closed is True

    available = Probe(fail=False)
    _assert_mavlink_udpin_available(
        "udpin:127.0.0.1:14550",
        socket_factory=lambda *_args: available,
    )
    assert available.closed is True


def test_udpin_connection_disables_reuse_after_pymavlink_bind() -> None:
    import socket

    from Workstation.workstation_core.pixhawk_attitude import (
        _disable_mavlink_udpin_reuse,
    )

    calls: list[tuple[int, int, int]] = []
    connection = SimpleNamespace(
        port=SimpleNamespace(
            setsockopt=lambda level, option, value: calls.append(
                (int(level), int(option), int(value))
            )
        )
    )

    _disable_mavlink_udpin_reuse(connection, "udpin:0.0.0.0:14550")

    assert calls == [(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)]


def test_subscriber_start_guards_and_hardens_udpin_boundary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from pymavlink import mavutil

    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
    )

    calls: list[tuple[str, str]] = []

    class Connection:
        def close(self) -> None:
            calls.append(("close", "left"))

    connection = Connection()
    monkeypatch.setattr(
        pixhawk_attitude,
        "_assert_mavlink_udpin_available",
        lambda url: calls.append(("guard", url)),
    )
    monkeypatch.setattr(
        pixhawk_attitude,
        "_disable_mavlink_udpin_reuse",
        lambda actual, url: calls.append(("harden", url))
        if actual is connection
        else None,
    )
    monkeypatch.setattr(
        mavutil,
        "mavlink_connection",
        lambda *_args, **_kwargs: connection,
    )
    subscriber = DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
    )

    subscriber.start("left", start_timer=False)

    assert calls[:2] == [
        ("guard", "udpin:0.0.0.0:14550"),
        ("harden", "udpin:0.0.0.0:14550"),
    ]
    subscriber.stop("left")


def test_attitude_stream_is_re_requested_when_heartbeat_survives_a_stall(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
    )

    clock = [100.0]
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock[0])

    class FakeMav:
        def __init__(self) -> None:
            self.calls: list[tuple[object, ...]] = []

        def command_long_send(self, *args: object) -> None:
            self.calls.append(args)

    mav = FakeMav()
    connection = SimpleNamespace(
        mav=mav,
        mavlink=SimpleNamespace(MAV_CMD_SET_MESSAGE_INTERVAL=511),
        target_system=1,
        target_component=1,
    )
    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    subscriber.connections["left"] = connection
    heartbeat = SimpleNamespace(get_type=lambda: "HEARTBEAT")
    attitude = SimpleNamespace(
        get_type=lambda: "ATTITUDE",
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    )

    subscriber._handle_mavlink_message("left", heartbeat)
    subscriber._handle_mavlink_message("left", attitude)
    mav.calls.clear()

    clock[0] += 3.0
    subscriber._handle_mavlink_message("left", heartbeat)

    assert any(call[4] == pixhawk_attitude.MAVLINK_MESSAGE_IDS["ATTITUDE"] for call in mav.calls)

    subscriber._handle_mavlink_message("left", attitude)
    mav.calls.clear()
    clock[0] += 1.0
    subscriber._handle_mavlink_message("left", heartbeat)

    assert not any(call[4] == pixhawk_attitude.MAVLINK_MESSAGE_IDS["ATTITUDE"] for call in mav.calls)

def test_recording_protocol_includes_fix_alignment_and_final_sync_gates() -> None:
    from Workstation.protocol import evaluate_recording_protocol

    snapshot = {
        key: SimpleNamespace(state=state)
        for key, state in (
            ("camera.left", "LIVE"),
            ("camera.right", "LIVE"),
            ("synchronization", "SYNCED"),
            ("detection", "RUNNING"),
            ("pixhawk.left", "LIVE"),
            ("pixhawk.right", "LIVE"),
        )
    }
    calibration = {
        "active": True,
        "left": {"state": "AVAILABLE"},
        "right": {"state": "AVAILABLE"},
    }
    baseline = {
        "state": "GOOD",
        "length_m": 1.2,
        "left_fix_type": 3,
        "right_fix_type": 3,
    }
    readiness = evaluate_recording_protocol(
        snapshot,
        distance_mode="pixhawk",
        calibration=calibration,
        baseline=baseline,
        require_rtk_fixed=False,
        minimum_fix_type=3,
        physical_alignment={"state": "PASS"},
        final_sync_verified=True,
    )
    assert readiness.state == "READY"
    assert {check.key for check in readiness.checks} >= {
        "fix_type",
        "physical_alignment",
        "final_synchronization",
    }

    baseline["right_fix_type"] = 2
    blocked = evaluate_recording_protocol(
        snapshot,
        distance_mode="pixhawk",
        calibration=calibration,
        baseline=baseline,
        require_rtk_fixed=False,
        minimum_fix_type=3,
        physical_alignment={"state": "FAIL"},
        final_sync_verified=False,
    )
    assert {check.key for check in blocked.blocking} >= {
        "fix_type",
        "physical_alignment",
        "final_synchronization",
    }

def test_base_gnss_workspace_projects_survey_in_and_ntrip_without_relabeling_frames() -> None:
    from Workstation.workspaces import BaseGnssWorkspace
    from Workstation.workstation_core.gnss_operation import (
        CorrectionSourceIdentity,
        GnssOperationConfig,
        GnssOperationManager,
        GnssOperationMode,
    )

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    manager = GnssOperationManager(GnssOperationConfig())
    manager.configure(GnssOperationConfig())
    manager.set_source_identity(
        CorrectionSourceIdentity(source_kind="LOCAL_BASE", base_device="/dev/ttyACM2")
    )
    panel = SimpleNamespace(
        gnss_operation_manager=manager,
        gnss_base_serial_connected=True,
        gnss_base_detect_thread=None,
        workstation_gnss_base_discovery_status="FOUND 1 · UBX 1",
        _gnss_base_candidate_rows=[
            {
                "port": "/dev/ttyACM2",
                "target": "u-blox F9P",
                "baud": 115200,
                "status": "UBX detected",
                "messages": ("NAV-PVT", "NAV-SVIN"),
                "ubx": True,
            }
        ],
        gnss_network_connected=False,
        gnss_network_status_line="",
        latest_gnss_base_survey_in={
            "active": True,
            "valid": False,
            "duration_s": 75,
            "mean_accuracy_m": 0.0412,
            "observations": 812,
            "mean_x_m": -3041150.0,
            "mean_y_m": 4048200.0,
            "mean_z_m": 3867400.0,
        },
        gnss_base_survey_samples=(),
        gnss_coordinate_samples=(),
        gnss_fixed_base_profile=None,
        gnss_fixed_base_reconfigure_ack=False,
        gnss_fixed_base_reconfigure_readback=False,
        gnss_fixed_base_reconfigure_rtcm=False,
        gnss_base_rtcm_stats={
            "start_s": 5.0,
            "last_frame_s": 9.0,
            "frames": 50,
            "bytes": 4000,
            "injection": {
                "left": {"frames": 20, "packets": 30, "bytes": 1600, "fail": 0, "last_delivery_s": 9.0},
                "right": {"frames": 20, "packets": 30, "bytes": 1600, "fail": 1, "last_delivery_s": 8.5},
                "drone": {"frames": 18, "packets": 28, "bytes": 1440, "fail": 0, "last_delivery_s": 9.5},
            },
        },
    )
    adapter = EvidenceAdapter(panel)
    projected = adapter.gnss_operation(now_s=10.0)
    assert projected["mode"] == "LOCAL_SURVEY_IN"
    assert projected["survey_in"]["mean_ecef_m"] == (-3041150.0, 4048200.0, 3867400.0)
    assert "ECEF [X,Y,Z]" in projected["survey_in"]["axis_contract"]
    assert "NED" in projected["survey_in"]["axis_contract"]
    assert projected["routes"]["left"]["delivery_age_s"] == pytest.approx(1.0)
    assert projected["routes"]["drone"] == {
        "frames": 18,
        "bytes": 1440,
        "packets": 28,
        "fail": 0,
        "delivery_age_s": pytest.approx(0.5),
    }
    assert projected["base_candidates"][0]["port"] == "/dev/ttyACM2"
    assert projected["base_candidates"][0]["messages"] == "NAV-PVT, NAV-SVIN"
    assert projected["base_discovery_active"] is False
    assert projected["base_discovery_status"] == "FOUND 1 · UBX 1"

    applied: list[int] = []
    backend = SimpleNamespace(
        panel=panel,
        gnss_base_apply_candidate=lambda row: applied.append(row),
    )
    workspace = BaseGnssWorkspace(backend, lambda _command: True)
    workspace.refresh({})
    assert workspace.summary_values["mode"].value.text() == "Local Survey-in"
    assert workspace.survey_values["duration"].value.text().startswith("75 /")
    assert workspace.survey_values["observations"].value.text() == "812"
    assert workspace.survey_values["ecef"].value.text() == "-3041150.000/4048200.000/3867400.000"
    assert not workspace.survey_group.isHidden()
    assert workspace.ntrip_group.isHidden()
    assert workspace.candidate_model.rowCount() == 1
    assert workspace.discovery_status.value.text() == "FOUND 1 · UBX 1"
    assert workspace.candidate_model.item(0, 0).text() == "/dev/ttyACM2"
    assert workspace.candidate_model.item(0, 4).text() == "NAV-PVT, NAV-SVIN"
    assert workspace.route_model.rowCount() == 3
    assert workspace.route_model.item(2, 0).text() == "T"
    assert workspace.route_model.item(2, 1).text() == "18"
    assert workspace.route_model.item(2, 2).text() == "28"
    workspace.candidate_table.selectRow(0)
    assert workspace.apply_candidate_button.isEnabled()
    workspace.apply_candidate_button.click()
    assert applied == [0]
    assert workspace.candidate_feedback.text() == "Applied Base receiver row 1"

    network_config = GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT)
    manager.configure(network_config)
    manager.set_source_identity(
        CorrectionSourceIdentity(
            source_kind="NTRIP",
            caster_host="caster.example.org",
            caster_port=2101,
            mountpoint="BASE01",
        )
    )
    panel.gnss_network_connected = True
    panel.gnss_network_status_line = "ICY 200 OK"
    workspace.refresh({})
    assert workspace.summary_values["mode"].value.text() == "Network RTK Direct"
    assert workspace.ntrip_values["caster"].value.text().startswith("caster.example.org:2101")
    assert workspace.ntrip_values["mountpoint"].value.text() == "BASE01"
    assert workspace.ntrip_values["health"].value.text() == "CONNECTED_NO_DATA"
    assert "last=--" in workspace.ntrip_values["transport"].value.text()
    assert workspace.survey_group.isHidden()
    assert not workspace.ntrip_group.isHidden()
    workspace.deleteLater()
    app.processEvents()

def test_base_gnss_workspace_shows_gga_transmission_status() -> None:
    """A VRS caster withholds RTCM without periodic GGA while the transport
    itself keeps reporting "connected" -- the GGA transmission row is the
    only place that stuck-but-connected state becomes visible.
    """

    from Workstation.workspaces import BaseGnssWorkspace
    from Workstation.workstation_core.gnss_operation import (
        CorrectionSourceIdentity,
        GnssOperationConfig,
        GnssOperationManager,
        GnssOperationMode,
    )

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    manager = GnssOperationManager(GnssOperationConfig())
    manager.configure(GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT))
    manager.set_source_identity(
        CorrectionSourceIdentity(
            source_kind="NTRIP",
            caster_host="RTS2.ngii.go.kr",
            caster_port=2101,
            mountpoint="VRS-RTCM32",
        )
    )
    now_s = time.monotonic()
    panel = SimpleNamespace(
        gnss_operation_manager=manager,
        gnss_base_serial_connected=False,
        gnss_base_detect_thread=None,
        workstation_gnss_base_discovery_status="NOT RUN",
        _gnss_base_candidate_rows=[],
        gnss_network_connected=True,
        gnss_network_thread=object(),
        gnss_network_status_line="ICY 200 OK",
        gnss_network_stats=SimpleNamespace(
            connected_at_monotonic_s=now_s - 20.0,
            last_chunk_monotonic_s=now_s - 0.5,
            last_gga_monotonic_s=None,
            gga_sent_count=0,
            bytes_received=1000,
            chunks_received=5,
        ),
        latest_gnss_base_survey_in={},
        gnss_base_survey_samples=(),
        gnss_coordinate_samples=(),
        gnss_fixed_base_profile=None,
        gnss_fixed_base_reconfigure_ack=False,
        gnss_fixed_base_reconfigure_readback=False,
        gnss_fixed_base_reconfigure_rtcm=False,
        gnss_base_rtcm_stats={},
        _gnss_last_gga_skip_reason=(
            "[GNSS][NTRIP] GGA skipped: no left/right GNSS position available "
            "(policy=system_center, left=no message received, right=no message received)"
        ),
        _gnss_last_gga_skip_monotonic_s=now_s - 2.0,
    )
    backend = SimpleNamespace(panel=panel, gnss_base_apply_candidate=lambda row: None)
    workspace = BaseGnssWorkspace(backend, lambda _command: True)
    workspace.refresh({})

    gga_text = workspace.ntrip_values["gga_transmission"].value.text()
    assert "차단" in gga_text
    assert "no left/right GNSS position available" in gga_text
    assert "[GNSS][NTRIP] GGA skipped:" not in gga_text
    assert "INVALID" in workspace.ntrip_values["gga_transmission"].meta.text()

    # A rover fix arrives and GGA starts flowing again -- the skip reason
    # clears and the row reports normal transmission instead of a block.
    panel._gnss_last_gga_skip_reason = ""
    panel._gnss_last_gga_skip_monotonic_s = None
    panel.gnss_network_stats.last_gga_monotonic_s = now_s - 1.0
    panel.gnss_network_stats.gga_sent_count = 1
    workspace.refresh({})

    gga_text = workspace.ntrip_values["gga_transmission"].value.text()
    assert "전송 중" in gga_text
    assert "VALID" in workspace.ntrip_values["gga_transmission"].meta.text()

    workspace.deleteLater()
    app.processEvents()


def test_base_gnss_workspace_shows_non_gating_rts2_checklist() -> None:
    from Workstation.workspaces import BaseGnssWorkspace
    from Workstation.workstation_core.gnss_operation import (
        CorrectionSourceIdentity,
        GnssOperationConfig,
        GnssOperationManager,
    )

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    config = GnssOperationConfig.ngii_rts2_base_assisted()
    manager = GnssOperationManager(config)
    manager.configure(config)
    manager.set_source_identity(
        CorrectionSourceIdentity(
            source_kind="NTRIP",
            caster_host=config.network.caster_host,
            caster_port=config.network.caster_port,
            mountpoint=config.network.mountpoint,
            gga_policy=config.network.gga_policy,
        )
    )
    now = time.monotonic()
    gps = lambda side: SimpleNamespace(
        camera=side,
        namespace=side,
        timestamp_s=now,
        topic=f"gps/{side}",
        fix_type=6,
        rtk_label="RTK_FIXED",
        satellites_used=20,
        eph_m=0.01,
        epv_m=0.02,
    )
    panel = SimpleNamespace(
        gnss_operation_manager=manager,
        gnss_base_serial_connected=True,
        gnss_base_rtcm_stats={
            "start_s": now - 60.0,
            "last_frame_s": now - 1.0,
            "frames": 12,
            "bytes": 960,
            "message_types": {1005: 2, 1077: 10},
        },
        gnss_network_connected=True,
        gnss_network_status_line="ICY 200 OK",
        latest_gps={"left": gps("left"), "right": gps("right")},
        gnss_rover_fixed_since={"left": now - 45.0, "right": now - 50.0},
        gnss_coordinate_samples=(),
        gnss_fixed_base_profile=SimpleNamespace(to_mapping=lambda: {"profile_hash": "base-profile"}),
        gnss_fixed_base_reconfigure_ack=True,
        gnss_fixed_base_reconfigure_readback=True,
        gnss_fixed_base_reconfigure_rtcm=True,
    )
    backend = SimpleNamespace(panel=panel)
    workspace = BaseGnssWorkspace(backend, lambda _command: True)

    workspace.refresh({})

    assert not workspace.checklist_group.isHidden()
    assert all(value.value.text() == "SATISFIED" for value in workspace.checklist_values.values())
    assert manager.config.readiness_gate_enabled is False
    workspace.deleteLater()
    app.processEvents()

def test_camera_and_pixhawk_discovery_candidates_are_visible_and_assignable() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    interface = SimpleNamespace(name="enp4s0", address="192.168.0.10")
    backend.panel._camera_candidate_rows = [
        {
            "kind": "NIC",
            "address": "enp4s0",
            "port": "",
            "status": "candidate",
            "interface": interface,
            "host_ip": "192.168.0.10",
        },
        {
            "kind": "WIRIS",
            "address": "192.168.0.20",
            "host": "192.168.0.20",
            "port": "23",
            "status": "HIWS OK",
            "interface": interface,
            "host_ip": "192.168.0.10",
        },
    ]
    backend.panel._pixhawk_candidate_rows_by_source = {
        "network": [
            {
                "kind": "NETWORK",
                "candidate_id": "5:enp5s0:192.168.1.10/24",
                "interface": "enp5s0",
                "carrier": "UP",
                "local_cidr": "192.168.1.10/24",
                "local_ip": "192.168.1.10",
                "p1_route": "ROUTE OK · enp5s0 src=192.168.1.10",
                "p2_route": "ROUTE OK · enp5s0 src=192.168.1.10",
                "p1_route_state": "ROUTE OK",
                "p2_route_state": "ROUTE OK",
                "p1_endpoint": "udpin:0.0.0.0:14550",
                "p2_endpoint": "udpin:0.0.0.0:14551",
                "status": "LINK UP · P1 ROUTE OK · P2 ROUTE OK",
            }
        ],
        "pc": [
            {
                "kind": "PC_IP",
                "value": "192.168.1.10",
                "status": "enp5s0",
                "interface": "enp5s0",
            }
        ],
        "mavlink": [
            {
                "kind": "MAVLink",
                "value": "udpin:0.0.0.0:14550",
                "baud": 115200,
                "status": "Ethernet candidate",
            }
        ],
    }
    backend.panel.pixhawk_network_assignments = {
        "left": {"interface": "", "local_address": ""},
        "right": {"interface": "", "local_address": ""},
    }
    backend.panel.pixhawk_network_assignment_status = {
        "left": {"state": "LEGACY AUTO ROUTE", "detail": "no explicit NIC assignment"},
        "right": {"state": "LEGACY AUTO ROUTE", "detail": "no explicit NIC assignment"},
    }
    backend.panel.left_mavlink_url_edit = SimpleNamespace(
        text=lambda: "udpin:0.0.0.0:14550"
    )
    backend.panel.right_mavlink_url_edit = SimpleNamespace(
        text=lambda: "udpin:0.0.0.0:14551"
    )
    backend.panel.workstation_camera_discovery_status = "FOUND 2 ENDPOINTS"
    backend.panel.workstation_pixhawk_discovery_status = "FOUND 2 ENDPOINTS"
    window = WorkstationMainWindow(backend=backend)

    window.camera.refresh(window.status_adapter.model.snapshot())
    assert window.camera.candidate_model.rowCount() == 2
    assert window.camera.discovery_status.value.text() == "FOUND 2 ENDPOINTS"
    camera_inspections: list[dict[str, object]] = []
    window.camera.inspection_requested.connect(camera_inspections.append)
    window.camera.candidate_table.selectRow(0)
    window.camera.candidate_table.clicked.emit(window.camera.candidate_model.index(0, 0))
    assert camera_inspections[-1]["selection"] == "Camera endpoint · NIC"
    assert "no image pixels" in str(camera_inspections[-1]["contract"])
    assert window.camera.candidate_buttons["nic"].isEnabled()
    assert not window.camera.candidate_buttons["left"].isEnabled()
    window.camera.candidate_buttons["nic"].click()
    assert backend.calls[-1] == ("camera_apply_candidate", 0, "nic")
    assert window.camera.candidate_feedback.text() == "Applied row 1 to NIC"
    window.camera.candidate_table.selectRow(1)
    assert window.camera.candidate_buttons["left"].isEnabled()
    window.camera.candidate_buttons["left"].click()
    assert backend.calls[-1] == ("camera_apply_candidate", 1, "left")

    window.pixhawk.refresh(window.status_adapter.model.snapshot())
    assert window.pixhawk.candidate_model.rowCount() == 3
    assert window.pixhawk.candidate_model.columnCount() == 5
    assert [
        window.pixhawk.candidate_model.headerData(
            column, QtCore.Qt.Horizontal, QtCore.Qt.DisplayRole
        )
        for column in range(window.pixhawk.candidate_model.columnCount())
    ] == ["Source", "NIC / Device", "Link", "Local IPv4", "Assigned"]
    assert window.pixhawk.discovery_status.value.text() == "FOUND 2 ENDPOINTS"
    pixhawk_inspections: list[dict[str, object]] = []
    window.pixhawk.inspection_requested.connect(pixhawk_inspections.append)
    window.pixhawk.candidate_table.selectRow(0)
    window.pixhawk.candidate_table.clicked.emit(window.pixhawk.candidate_model.index(0, 0))
    assert pixhawk_inspections[-1]["selection"] == "Pixhawk endpoint · NETWORK"
    assert "no attitude" in str(pixhawk_inspections[-1]["contract"])
    assert window.pixhawk.candidate_buttons["left"].isEnabled()
    window.pixhawk.candidate_buttons["left"].click()
    assert backend.calls[-1] == (
        "pixhawk_assign_network_candidate",
        "5:enp5s0:192.168.1.10/24",
        "left",
    )
    assert "press Connect" in window.pixhawk.candidate_feedback.text()
    backend.panel.pixhawk_network_assignments["left"] = {
        "interface": "enp5s0",
        "local_address": "192.168.1.10",
    }
    backend.panel.pixhawk_network_assignment_status["left"] = {
        "state": "ASSIGNED",
        "detail": "ROUTE OK",
    }
    window.pixhawk.refresh(window.status_adapter.model.snapshot())
    assert window.pixhawk._selected_candidate_row() == 0
    assert window.pixhawk.candidate_model.item(0, 4).text() == "P1"
    assert window.pixhawk.network_assignment_values["left"]["state"].value.text() == "ASSIGNED"
    assert window.pixhawk.network_assignment_values["left"]["nic"].value.text() == "enp5s0"
    assert window.pixhawk.network_assignment_values["left"]["local"].value.text() == "192.168.1.10"
    assert (
        window.pixhawk.network_assignment_values["left"]["endpoint"].value.text()
        == "udpin:0.0.0.0:14550"
    )
    assert window.pixhawk.network_assignment_values["left"]["route"].value.text() == "ROUTE OK"
    assert window.pixhawk.network_assignment_values["right"]["state"].value.text() == "LEGACY AUTO ROUTE"
    backend.panel.pixhawk_network_assignments["right"] = {
        "interface": "enp5s0",
        "local_address": "192.168.1.10",
    }
    backend.panel.pixhawk_network_assignment_status["right"] = {
        "state": "ASSIGNED",
        "detail": "ROUTE OK",
    }
    window.pixhawk.refresh(window.status_adapter.model.snapshot())
    assert window.pixhawk.candidate_model.item(0, 4).text() == "P1,P2"
    assert (
        window.pixhawk.network_assignment_values["right"]["endpoint"].value.text()
        == "udpin:0.0.0.0:14551"
    )
    assert window.pixhawk.network_assignment_values["right"]["route"].value.text() == "ROUTE OK"
    assert "warning" not in window.pixhawk.candidate_feedback.text().lower()
    window.pixhawk.candidate_table.selectRow(1)
    assert not window.pixhawk.candidate_buttons["right"].isEnabled()
    window.pixhawk.candidate_table.selectRow(2)
    assert window.pixhawk.candidate_buttons["right"].isEnabled()
    window.pixhawk.candidate_buttons["right"].click()
    assert backend.calls[-1] == ("pixhawk_apply_candidate", "mavlink", 0, "right")

    backend.camera_apply_candidate = lambda _row, _target: False
    window.camera.candidate_table.selectRow(1)
    window.camera.candidate_buttons["right"].click()
    assert window.camera.candidate_feedback.text() == "ERROR: endpoint was not applied"

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_inspector_centralizes_active_geometry_and_applies_gnss_mountain_candidate() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    candidate_snapshot = {
        "state": "CANDIDATE",
        "active_candidate_id": "",
        "active_selection_id": "",
        "active_selected_dof": {},
        "candidates": [
            {
                "candidate_id": "gnss-mountain-1",
                "method": "gnss-constrained-mountain-uv",
                "state": "WARNING",
                "applicable": True,
                "observable_dof": {
                    "roll_deg": True,
                    "pitch_deg": True,
                    "yaw_deg": True,
                    "north_m": True,
                    "east_m": True,
                    "down_m": True,
                },
                "input_hashes": {
                    "operating_intrinsics_profile_sha256": "a" * 64,
                    "gnss_baseline_sha256": "b" * 64,
                },
                "quality_metrics": {"gnss_baseline_m": 1.2},
                "source_observation_ids": ["mountain-left", "mountain-right"],
                "warnings": ["camera optical center equals GNSS antenna"],
            }
        ],
    }
    backend.optical_pose_snapshot = lambda include_frames=True: candidate_snapshot
    backend.optical_pose_apply_candidate = lambda payload: backend.calls.append(
        ("optical_pose_apply_candidate", payload)
    ) or candidate_snapshot
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window.inspector.refresh("processing.alignment", window.status_adapter.model)
    assert window.inspector.geometry_section.title() == "Active Geometry"
    assert window.inspector.geometry_candidate_combo.currentData() == "gnss-mountain-1"
    assert window.inspector.geometry_apply_candidate_button.isEnabled()
    assert "K,D" in window.inspector.geometry_values["bundle"].value.text()
    assert "gnss-mountain-1" in window.inspector.geometry_candidate_combo.currentText()

    window.inspector.geometry_apply_candidate_button.click()
    assert backend.calls[-1] == (
        "optical_pose_apply_candidate",
        {"candidate_id": "gnss-mountain-1"},
    )

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_pixhawk_level_reset_only_targets_level_horizon_offsets(monkeypatch: pytest.MonkeyPatch) -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    shell_calls: list[tuple[str, str, dict[str, object]]] = []
    logs: list[str] = []
    panel = SimpleNamespace(
        _session_recording_active=lambda: False,
        _mavlink_open_sides=lambda: ("left", "right"),
        px4_mag_managers={"left": SimpleNamespace(armed=False), "right": SimpleNamespace(armed=False)},
        _px4_mag_camera_label=lambda side: "P1" if side == "left" else "P2",
        _log=logs.append,
        _start_pixhawk_shell_worker=lambda camera, command, **kwargs: shell_calls.append(
            (camera, command, kwargs)
        ) or True,
    )
    monkeypatch.setattr(
        QtWidgets.QMessageBox,
        "question",
        lambda *_args, **_kwargs: QtWidgets.QMessageBox.Yes,
    )

    assert DeviceRuntimeController(panel).reset_px4_level_calibration() is True
    assert [camera for camera, _command, _kwargs in shell_calls] == ["left", "right"]
    assert all(
        command == "param reset SENS_BOARD_X_OFF SENS_BOARD_Y_OFF"
        for _camera, command, _kwargs in shell_calls
    )
    assert all(kwargs["operation"] == "reset_level_calibration" for _camera, _command, kwargs in shell_calls)
    assert "SENS_BOARD_ROT" not in shell_calls[0][1]
    assert any("PARAM readback" in message for message in logs)


def test_pixhawk_usb_eth_bootstrap_persists_one_parameter_owned_instance() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    panel = SimpleNamespace(
        _persistent_pixhawk_eth_setup=lambda _camera: {
            "param_lines": ["param set MAV_2_CONFIG 1000"],
            "extras_lines": [
                "ifup eth0",
                "ifconfig eth0 10.0.0.2 netmask 255.255.255.0 up",
            ],
            "net_cfg_lines": [
                "DEVICE=eth0",
                "BOOTPROTO=static",
                "NETMASK=255.255.255.0",
                "IPADDR=10.0.0.2",
                "ROUTER=10.0.0.10",
            ],
        }
    )
    controller = DeviceRuntimeController(panel)

    normal = controller._ethernet_mavlink_bootstrap_command("left")
    onboard = controller._ethernet_mavlink_bootstrap_command("left", mode="onboard")

    assert normal.splitlines() == [
        "mkdir /fs/microsd/etc",
        "param set MAV_2_CONFIG 1000",
        "param save",
        "echo 'ifup eth0' > /fs/microsd/etc/extras.txt",
        "echo 'ifconfig eth0 10.0.0.2 netmask 255.255.255.0 up' >> /fs/microsd/etc/extras.txt",
        "echo 'DEVICE=eth0' > /fs/microsd/net.cfg",
        "echo 'BOOTPROTO=static' >> /fs/microsd/net.cfg",
        "echo 'NETMASK=255.255.255.0' >> /fs/microsd/net.cfg",
        "echo 'IPADDR=10.0.0.2' >> /fs/microsd/net.cfg",
        "echo 'ROUTER=10.0.0.10' >> /fs/microsd/net.cfg",
        "reboot",
    ]
    assert "mavlink stop" not in normal
    assert "mavlink start" not in normal
    assert "mavlink stop" not in onboard
    assert "mavlink start" not in onboard
    assert onboard == normal


def test_pixhawk_persistent_eth_has_one_parameter_owned_mavlink_instance() -> None:
    from Workstation.device_runtime_controller import DeviceRuntimeController

    panel = SimpleNamespace(
        left_pixhawk_eth_ip_edit=SimpleNamespace(text=lambda: "10.0.0.2"),
        right_pixhawk_eth_ip_edit=SimpleNamespace(text=lambda: "10.0.0.3"),
        left_mavlink_url_edit=SimpleNamespace(text=lambda: "udpin:0.0.0.0:14550"),
        right_mavlink_url_edit=SimpleNamespace(text=lambda: "udpin:0.0.0.0:14551"),
        _pixhawk_host_ip_for_mavlink=lambda: "10.0.0.10",
        _mavlink_stream_port=lambda _url, *, default: default,
        mavlink_stream_rate_spin=SimpleNamespace(value=lambda: 2_000_000),
    )

    setup = DeviceRuntimeController(panel)._persistent_pixhawk_eth_setup("left")

    assert setup["extras_lines"] == [
        "ifup eth0",
        "ifconfig eth0 10.0.0.2 netmask 255.255.255.0 up",
    ]
    assert setup["param_lines"] == [
        "param set MAV_SYS_ID 1",
        "param set MAV_2_CONFIG 1000",
        "param set MAV_2_MODE 0",
        "param set MAV_2_BROADCAST 1",
        "param set MAV_2_REMOTE_PRT 14550",
        "param set MAV_2_UDP_PRT 14550",
        "param set MAV_2_RATE 2000000",
    ]
    assert all("mavlink stop" not in line for line in setup["extras_lines"])
    assert all("mavlink start" not in line for line in setup["extras_lines"])


def test_pixhawk_stream_setup_never_creates_a_second_runtime_instance(monkeypatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    commands: list[str] = []
    monkeypatch.setattr(
        pixhawk_attitude,
        "_send_mavlink_shell_command",
        lambda _connection, command, **_kwargs: commands.append(command),
    )
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    subscriber.connections["left"] = object()

    subscriber.configure_mavlink_streams(
        host_ip="10.0.0.10",
        ports={"left": 14550},
        sysids={"left": 1},
        rate=2_000_000,
        cameras={"left"},
    )

    assert len(commands) == 1
    assert "param set MAV_2_CONFIG 1000" in commands[0]
    assert "param set MAV_2_BROADCAST 1" in commands[0]
    assert "param set MAV_2_UDP_PRT 14550" in commands[0]
    assert "param save" in commands[0]
    assert "mavlink start" not in commands[0]
    assert "mavlink stop" not in commands[0]


def test_pixhawk_shell_releases_serial_control_exclusive_flag() -> None:
    from Workstation.workstation_core.pixhawk_attitude import _send_mavlink_shell_command

    class FakeMav:
        def __init__(self) -> None:
            self.calls: list[tuple[int, int, int]] = []

        def serial_control_send(self, device, flags, _timeout, _baudrate, count, _data):
            self.calls.append((int(device), int(flags), int(count)))

    class FakeConnection:
        def __init__(self) -> None:
            self.mav = FakeMav()
            self.mavlink = SimpleNamespace(
                SERIAL_CONTROL_DEV_SHELL=10,
                SERIAL_CONTROL_FLAG_RESPOND=2,
                SERIAL_CONTROL_FLAG_EXCLUSIVE=4,
            )

        def recv_match(self, **_kwargs):
            return None

    connection = FakeConnection()
    result = _send_mavlink_shell_command(connection, "mavlink status", timeout_s=0.0)

    assert result.flag_label == "respond"
    assert any(flags & 4 for _device, flags, _count in connection.mav.calls)
    assert connection.mav.calls[-1] == (10, 0, 0)


def test_pixhawk_reset_commands_require_connected_disarmed_target() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel._mavlink_open_sides = lambda: ()
    backend.panel.px4_mag_managers = {}
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()
    command_id = "pixhawk.reset_calibration.p1.compass"

    assert window.commands.action(command_id).isEnabled() is False
    backend.panel._mavlink_open_sides = lambda: ("left",)
    backend.panel.px4_mag_managers = {"left": SimpleNamespace(armed=True)}
    window._refresh_from_status(window.status_adapter.model.snapshot())
    assert window.commands.action(command_id).isEnabled() is False
    backend.panel.px4_mag_managers["left"].armed = False
    window._refresh_from_status(window.status_adapter.model.snapshot())
    assert window.commands.action(command_id).isEnabled() is True

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_alignment_surface_reclaims_height_when_diagnostics_hidden() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.resize(1600, 980)
    window.diagnostics_dock.show()
    window.show()
    window.navigation.select_context("processing.alignment")
    app.processEvents()
    app.processEvents()

    alignment_scroll = window.workspace_scrolls["alignment"]
    shown_viewport_height = alignment_scroll.viewport().height()

    window.diagnostics_dock.hide()
    app.processEvents()
    app.processEvents()

    hidden_viewport_height = alignment_scroll.viewport().height()
    assert hidden_viewport_height > shown_viewport_height + 100

    window.diagnostics_dock.show()
    app.processEvents()
    app.processEvents()
    assert alignment_scroll.viewport().height() <= shown_viewport_height + 4

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_alignment_surface_shrinks_when_inspector_opens() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.resize(1600, 980)
    window.inspector_dock.hide()
    window.show()
    window.navigation.select_context("processing.alignment")
    app.processEvents()
    app.processEvents()

    alignment_scroll = window.workspace_scrolls["alignment"]
    hidden_viewport_width = alignment_scroll.viewport().width()
    assert window.alignment.width() == hidden_viewport_width

    window.inspector_dock.show()
    app.processEvents()
    app.processEvents()

    shown_viewport_width = alignment_scroll.viewport().width()
    assert shown_viewport_width < hidden_viewport_width - 200
    assert window.alignment.width() == shown_viewport_width
    assert window.alignment.optical_pose_pane.width() <= shown_viewport_width

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_wiris_lrf_worker_parses_meter_responses_and_reports_errors(monkeypatch) -> None:
    from Workstation import wiris_runtime_workers

    responses = {"10.0.0.11": "OK 20.4", "10.0.0.12": "ERR"}

    def fake_send(command) -> str:
        return responses[command.host]

    monkeypatch.setattr(wiris_runtime_workers, "send_tcp_line", fake_send)
    worker = wiris_runtime_workers.WirisLrfPollWorker(
        (
            ("left", "10.0.0.11", 2240, 1.0),
            ("right", "10.0.0.12", 2240, 1.0),
        )
    )

    samples = {row["side"]: row for row in worker.poll_once()}

    assert samples["left"]["distance_m"] == pytest.approx(20.4)
    assert samples["left"]["error"] == ""
    assert samples["right"]["distance_m"] is None
    assert samples["right"]["error"] == "ERR"

def test_status_adapter_projects_lrf_live_stale_and_error_states() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    now = time.monotonic()
    backend.panel.latest_wiris_lrf = {
        "left": {
            "distance_m": 20.4,
            "last_good_s": now,
            "last_attempt_s": now,
            "response": "20.4",
            "last_error": "",
        },
        "right": {
            "distance_m": 18.2,
            "last_good_s": now - 90.0,
            "last_attempt_s": now,
            "response": "ERR",
            "last_error": "ERR",
        },
    }
    adapter = BackendStatusAdapter(backend)

    adapter.refresh()

    left = adapter.model.record("lrf.left")
    right = adapter.model.record("lrf.right")
    assert left is not None and left.state == "LIVE" and left.severity == StatusSeverity.OK
    assert "20.400 m" in left.detail
    assert right is not None and right.state == "STALE" and right.severity == StatusSeverity.WARN
    assert "18.200 m" in right.detail
    _ = app

def test_pixhawk_usb_discovery_worker_keeps_serial_probe_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    monkeypatch.setattr(device_runtime_workers, "usb_mavlink_device_candidates", lambda: ["/dev/ttyACM0"])
    monkeypatch.setattr(
        device_runtime_workers,
        "probe_usb_mavlink_devices",
        lambda *, candidates, timeout_s: [(candidates[0], 115200)],
    )
    results: list[object] = []
    worker = standalone_gui.PixhawkUsbDiscoveryWorker(timeout_s=0.8)
    worker.discovered.connect(results.append)
    worker.run()

    assert results == [
        {
            "candidates": ["/dev/ttyACM0"],
            "found": [
                {
                    "device": "/dev/ttyACM0",
                    "resolved_device": "/dev/ttyACM0",
                    "baud": 115200,
                    "system_id": None,
                    "component_id": None,
                    "observed_monotonic_s": None,
                }
            ],
        }
    ]


def test_usb_probe_preserves_heartbeat_system_component_and_observation_time(monkeypatch) -> None:
    from pymavlink import mavutil

    from Workstation.workstation_core import pixhawk_attitude

    captured: dict[str, object] = {}

    class Heartbeat:
        def get_srcSystem(self):
            return 1

        def get_srcComponent(self):
            return 42

    class Connection:
        def __init__(self) -> None:
            self.mav = SimpleNamespace(heartbeat_send=lambda *args: captured.setdefault("heartbeat", args))
            self.closed = False

        def wait_heartbeat(self, *, timeout):
            captured["timeout"] = timeout
            return Heartbeat()

        def close(self):
            self.closed = True

    connection = Connection()

    def factory(device, **kwargs):
        captured["device"] = device
        captured.update(kwargs)
        return connection

    monkeypatch.setattr(mavutil, "mavlink_connection", factory)
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: 123.5)

    found = pixhawk_attitude.probe_usb_mavlink_devices(
        ["/dev/ttyACM0"],
        baudrates=(115200,),
        timeout_s=0.8,
    )

    assert len(found) == 1
    identity = found[0]
    assert identity.device == "/dev/ttyACM0"
    assert identity.baud == 115200
    assert identity.system_id == 1
    assert identity.component_id == 42
    assert identity.observed_monotonic_s == 123.5
    assert captured["source_system"] == 255
    assert captured["source_component"] == 190
    assert connection.closed


def test_verified_usb_recovery_worker_revalidates_identity_before_one_shot_reboot(monkeypatch) -> None:
    from pymavlink import mavutil

    from Workstation import device_runtime_workers

    commands: list[tuple[object, ...]] = []

    class Heartbeat:
        def get_srcSystem(self):
            return 1

        def get_srcComponent(self):
            return 1

    class Connection:
        def __init__(self) -> None:
            self.mav = SimpleNamespace(
                heartbeat_send=lambda *_args: None,
                command_long_send=lambda *args: commands.append(args),
            )
            self.closed = False

        def wait_heartbeat(self, *, timeout):
            assert timeout == 1.5
            return Heartbeat()

        def close(self):
            self.closed = True

    connection = Connection()
    monkeypatch.setattr(mavutil, "mavlink_connection", lambda *_args, **_kwargs: connection)
    results: list[dict[str, object]] = []
    failures: list[str] = []
    worker = device_runtime_workers.PixhawkUsbRecoveryWorker(
        camera="left",
        evidence={"device": "/dev/ttyACM0", "baud": 115200, "component_id": 1},
        expected_system_id=1,
    )
    worker.completed.connect(results.append)
    worker.failed.connect(failures.append)

    worker.run()

    assert failures == []
    assert results[0]["command_sent"] is True
    assert commands == [
        (
            1,
            1,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
    ]
    assert connection.closed


def test_verified_usb_recovery_worker_wrong_sysid_fails_without_reboot(monkeypatch) -> None:
    from pymavlink import mavutil

    from Workstation import device_runtime_workers

    commands: list[tuple[object, ...]] = []
    heartbeat = SimpleNamespace(get_srcSystem=lambda: 2, get_srcComponent=lambda: 1)
    connection = SimpleNamespace(
        mav=SimpleNamespace(
            heartbeat_send=lambda *_args: None,
            command_long_send=lambda *args: commands.append(args),
        ),
        wait_heartbeat=lambda **_kwargs: heartbeat,
        close=lambda: None,
    )
    monkeypatch.setattr(mavutil, "mavlink_connection", lambda *_args, **_kwargs: connection)
    failures: list[str] = []
    worker = device_runtime_workers.PixhawkUsbRecoveryWorker(
        camera="left",
        evidence={"device": "/dev/ttyACM0", "baud": 115200, "component_id": 1},
        expected_system_id=1,
    )
    worker.failed.connect(failures.append)

    worker.run()

    assert commands == []
    assert len(failures) == 1
    assert "expected 1, got 2" in failures[0]

def test_pixhawk_host_ip_discovery_worker_keeps_interface_scan_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    interfaces = (SimpleNamespace(name="enx-pixhawk", address="10.41.10.1/24"),)
    monkeypatch.setattr(device_runtime_workers, "local_ipv4_interfaces", lambda *, include_wireless: interfaces)
    results: list[object] = []
    worker = standalone_gui.PixhawkHostIpDiscoveryWorker()
    worker.discovered.connect(results.append)
    worker.run()

    assert results == [interfaces]

def test_pixhawk_shell_and_parameter_workers_use_exclusive_command_path() -> None:
    from contextlib import contextmanager

    from Workstation import standalone_gui
    from Workstation.workstation_core.pixhawk_attitude import MavlinkShellCommandResult
    from Workstation.workstation_core.px4_mag_manager import MAG_PARAM_NAMES

    class FakeSubscriber:
        def __init__(self) -> None:
            self.connections = {"left": object()}
            self.params = {name: None for name in MAG_PARAM_NAMES}
            self.params.update(
                {
                    "UAVCAN_ENABLE": 2,
                    "UAVCAN_SUB_MAG": 1,
                    "CAL_MAG0_ID": standalone_gui.BOARD_INTERNAL_MAG_ID,
                    "CAL_MAG0_PRIO": 75,
                    "CAL_MAG0_PITCH": -0.01,
                    "CAL_MAG0_XOFF": 0.125,
                    "CAL_MAG0_XSCALE": 1.015,
                }
            )
            self.active: list[str] = []

        @contextmanager
        def exclusive_command(self, camera: str):
            # Mirrors the real MavlinkPixhawkSubscriber.exclusive_command: a
            # second reservation for the same camera while one is already
            # held (even by the same worker/thread re-entering) is rejected,
            # not silently nested.
            if camera in self.active:
                raise RuntimeError(f"{camera} MAVLink 명령이 이미 진행 중입니다.")
            self.active.append(camera)
            try:
                yield
            finally:
                self.active.remove(camera)

        def send_shell_command_with_result(self, camera: str, command: str, *, timeout_s: float):
            assert self.active == [camera]
            return MavlinkShellCommandResult(command, "nsh> OK", "respond+exclusive")

        def read_param_int(self, camera: str, name: str, *, timeout_s: float):
            # Mirrors the real MavlinkPixhawkSubscriber: each PARAM read/write
            # reserves its own exclusive_command slice. PixhawkParameterWorker
            # must not also hold one around the whole operation, or this
            # reservation collides with itself (regression: it used to).
            with self.exclusive_command(camera):
                return self.params.get(name)

        def read_param(self, camera: str, name: str, *, timeout_s: float):
            with self.exclusive_command(camera):
                return self.params.get(name)

        def set_param_int(self, camera: str, name: str, value: int, *, timeout_s: float):
            with self.exclusive_command(camera):
                self.params[name] = int(value)
                return int(value)

        def reboot_autopilot(self, camera: str) -> None:
            assert self.active == [camera]

    subscriber = FakeSubscriber()
    shell_results: list[object] = []
    shell_worker = standalone_gui.PixhawkShellCommandWorker(
        subscriber,
        camera="left",
        command="param show",
    )
    shell_worker.completed.connect(shell_results.append)
    shell_worker.run()
    assert shell_results[0]["output"] == "nsh> OK"

    parameter_results: list[object] = []
    parameter_worker = standalone_gui.PixhawkParameterWorker(
        subscriber,
        camera="left",
        operation="mag_apply",
        priorities={"BOARD": 100},
    )
    parameter_worker.completed.connect(parameter_results.append)
    parameter_worker.run()
    assert parameter_results[0]["applied"] == ("BOARD:CAL_MAG0_PRIO=100",)
    assert subscriber.params["CAL_MAG0_PRIO"] == 100

    refresh_results: list[object] = []
    parameter_refresh_worker = standalone_gui.PixhawkParameterWorker(
        subscriber,
        camera="left",
        operation="mag_refresh",
    )
    parameter_refresh_worker.completed.connect(refresh_results.append)
    parameter_refresh_worker.run()
    assert refresh_results[0]["status"].params["CAL_MAG0_XOFF"] == pytest.approx(0.125)
    assert refresh_results[0]["status"].params["CAL_MAG0_XSCALE"] == pytest.approx(1.015)
    assert refresh_results[0]["status"].params["CAL_MAG0_PITCH"] == pytest.approx(-0.01)

    combined_results: list[object] = []
    combined_worker = standalone_gui.PixhawkParameterWorker(
        subscriber,
        camera="left",
        operation="calibration_refresh",
    )
    combined_worker.completed.connect(combined_results.append)
    combined_worker.run()
    assert combined_results[0]["mag_status"].params["CAL_MAG0_XOFF"] == pytest.approx(0.125)
    assert "CAL_ACC0_XOFF" in combined_results[0]["attitude_status"].params

def test_pixhawk_connection_and_stream_setup_workers_keep_transport_io_off_ui_path() -> None:
    from Workstation import standalone_gui

    class FakeSubscriber:
        def __init__(self) -> None:
            self.connections = {}
            self.start_args = None
            self.setup_args = None

        def start(self, camera, *, start_timer, apply_pending_setup):
            self.start_args = (camera, start_timer, apply_pending_setup)
            self.connections["left"] = object()

        def start_receive_timer(self):
            raise AssertionError("timer activation belongs to the UI completion handler")

        def configure_mavlink_streams(self, **kwargs):
            self.setup_args = kwargs

    subscriber = FakeSubscriber()
    connection_results: list[object] = []
    connection_worker = standalone_gui.PixhawkConnectionWorker(subscriber, "left")
    connection_worker.completed.connect(connection_results.append)
    connection_worker.run()

    setup_results: list[object] = []
    setup_worker = standalone_gui.PixhawkStreamSetupWorker(
        subscriber,
        host_ip="10.41.10.1",
        ports={"left": 14550, "right": 14551},
        sysids={"left": 1, "right": 2},
        rate=2_000_000,
        cameras=("left",),
    )
    setup_worker.completed.connect(setup_results.append)
    setup_worker.run()

    assert subscriber.start_args == ("left", False, False)
    assert subscriber.setup_args["host_ip"] == "10.41.10.1"
    assert setup_results[0]["cameras"] == ("left",)
    assert connection_results[0]["connections"] == ("left",)


def test_pixhawk_connect_status_precedes_gui_completion_without_precompletion_repolish_and_starts_timer(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    workstation_qapplication,
) -> None:
    import threading

    from PyQt5 import QtCore

    from Workstation import device_runtime_controller, standalone_gui
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_initial_status",
        lambda _self: None,
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda _self: None,
    )
    app = workstation_qapplication
    panel = standalone_gui.WirisStereoDistancePanel(
        app_root=tmp_path,
        settings_path=tmp_path / "settings.json",
    )
    gui_thread = panel.thread()
    worker_entered = threading.Event()
    release_worker = threading.Event()
    events: list[tuple[str, QtCore.QThread]] = []
    probe_threads: list[QtCore.QThread] = []
    receive_tick_threads: list[QtCore.QThread] = []

    class Connection:
        target_system = 1
        target_component = 1

        def recv_match(self, **_kwargs):
            return None

        def close(self) -> None:
            return None

    class GatedSubscriber(DualPixhawkMavlinkSubscriber):
        def start(
            self,
            camera=None,
            *,
            start_timer=True,
            apply_pending_setup=False,
        ) -> None:
            assert camera == "left"
            assert start_timer is False
            assert apply_pending_setup is False
            events.append(("worker", QtCore.QThread.currentThread()))
            self.connections["left"] = Connection()
            for index in range(32):
                self.status_changed.emit(f"connect-status-{index}")
            worker_entered.set()
            if not release_worker.wait(2.0):
                raise RuntimeError("test release timeout")

    subscriber = GatedSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
    )
    subscriber.timer.timeout.connect(
        lambda: receive_tick_threads.append(QtCore.QThread.currentThread())
    )
    monkeypatch.setattr(
        device_runtime_controller,
        "DualPixhawkMavlinkSubscriber",
        lambda **_kwargs: subscriber,
    )
    # Keep the real connect completion and receive-timer boundary while
    # excluding unrelated post-connect shell/PARAM workers.
    monkeypatch.setattr(panel, "_request_mavlink_eth0_status", lambda *_args: None)
    monkeypatch.setattr(panel, "refresh_px4_mag_manager", lambda *_args: None)
    monkeypatch.setattr(panel, "refresh_px4_attitude_cal_status", lambda *_args: None)
    monkeypatch.setattr(panel, "_refresh_operation_header", lambda *_args: None)

    real_status = panel._handle_mavlink_status_changed

    def tracked_status(text: str) -> None:
        events.append(("status", QtCore.QThread.currentThread()))
        real_status(text)

    monkeypatch.setattr(panel, "_handle_mavlink_status_changed", tracked_status)
    real_apply = panel._apply_pixhawk_connection_result

    def tracked_apply(camera, force_setup, target_sides) -> None:
        events.append(("completed", QtCore.QThread.currentThread()))
        real_apply(camera, force_setup, target_sides)

    monkeypatch.setattr(panel, "_apply_pixhawk_connection_result", tracked_apply)
    real_set_role = panel._set_button_role

    def tracked_set_role(button, role: str) -> None:
        if button.property("role") != role:
            events.append(("role-change", QtCore.QThread.currentThread()))
        real_set_role(button, role)

    monkeypatch.setattr(panel, "_set_button_role", tracked_set_role)
    real_refresh_actions = panel._refresh_mavlink_side_actions

    def tracked_refresh_actions() -> None:
        events.append(("action-refresh", QtCore.QThread.currentThread()))
        real_refresh_actions()

    monkeypatch.setattr(
        panel,
        "_refresh_mavlink_side_actions",
        tracked_refresh_actions,
    )

    probe = QtCore.QTimer(panel)
    probe.setInterval(1)
    probe.timeout.connect(
        lambda: probe_threads.append(QtCore.QThread.currentThread())
    )
    thread = None
    try:
        panel.connect_mavlink_pixhawk("left")
        thread = panel.pixhawk_connection_thread
        assert thread is not None
        assert worker_entered.wait(1.0)

        # The transport is open but completion is deliberately held. Status
        # delivery must stay cheap and must not force a synchronous QStyle
        # repolish before the GUI-owner completion can start the receive timer.
        probe.start()
        deadline = time.monotonic() + 1.0
        while (
            not probe_threads
            or len([event for event in events if event[0] == "status"]) < 32
        ) and time.monotonic() < deadline:
            app.processEvents(QtCore.QEventLoop.AllEvents, 10)
            time.sleep(0.001)

        assert probe_threads
        assert all(owner is gui_thread for owner in probe_threads)
        status_threads = [owner for name, owner in events if name == "status"]
        assert len(status_threads) == 32
        assert all(owner is gui_thread for owner in status_threads)
        assert not any(name == "completed" for name, _owner in events)
        assert not any(name == "role-change" for name, _owner in events)
        assert len([name for name, _owner in events if name == "action-refresh"]) <= 1

        release_worker.set()
        deadline = time.monotonic() + 2.0
        while (
            panel.pixhawk_connection_thread is not None
            or not subscriber.timer.isActive()
            or not receive_tick_threads
        ) and time.monotonic() < deadline:
            app.processEvents(QtCore.QEventLoop.AllEvents, 10)
            time.sleep(0.001)

        names = [name for name, _owner in events]
        assert names.index("status") < names.index("completed") < names.index("role-change")
        assert [owner for name, owner in events if name == "worker"] == [thread]
        assert [owner for name, owner in events if name == "completed"] == [gui_thread]
        assert subscriber.timer.thread() is gui_thread
        assert subscriber.timer.isActive()
        assert receive_tick_threads
        assert all(owner is gui_thread for owner in receive_tick_threads)
        assert panel.pixhawk_connection_thread is None
        assert panel.left_mavlink_button.text() == "Disconnect P1 MAVLink"
        assert panel.left_mavlink_button.property("role") == "danger"
        assert len([name for name, _owner in events if name == "action-refresh"]) <= 3
    finally:
        release_worker.set()
        probe.stop()
        deadline = time.monotonic() + 2.0
        try:
            while thread is not None and thread.isRunning() and time.monotonic() < deadline:
                app.processEvents(QtCore.QEventLoop.AllEvents, 10)
                time.sleep(0.001)
            if thread is not None and thread.isRunning():
                thread.quit()
                assert thread.wait(1_000)
        except RuntimeError:
            pass
        subscriber.timer.stop()
        subscriber.connections.clear()
        panel.mavlink_pixhawk = None
        panel._save_current_settings = lambda: None
        panel.close()
        panel.deleteLater()
        app.processEvents()


def test_dynamic_button_role_change_never_forces_synchronous_qstyle_repolish() -> None:
    from Workstation import standalone_gui

    class BlockingStyleSentinel:
        def unpolish(self, _button) -> None:
            raise AssertionError("button role must not synchronously unpolish QStyle")

        def polish(self, _button) -> None:
            raise AssertionError("button role must not synchronously polish QStyle")

    class Button:
        def __init__(self) -> None:
            self.role = "primary"
            self.update_count = 0

        def property(self, name: str):
            assert name == "role"
            return self.role

        def setProperty(self, name: str, value: str) -> None:
            assert name == "role"
            self.role = value

        def style(self) -> BlockingStyleSentinel:
            return BlockingStyleSentinel()

        def update(self) -> None:
            self.update_count += 1

    button = Button()

    standalone_gui.WirisStereoDistancePanel._set_button_role(
        SimpleNamespace(),
        button,
        "danger",
    )
    standalone_gui.WirisStereoDistancePanel._set_button_role(
        SimpleNamespace(),
        button,
        "danger",
    )

    assert button.role == "danger"
    assert button.update_count == 1


def test_pixhawk_usb_bootstrap_worker_keeps_usb_shell_and_cleanup_off_ui_path(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers
    from Workstation.workstation_core.pixhawk_attitude import MavlinkShellCommandResult

    instances: list[object] = []

    class FakeConnection:
        def wait_heartbeat(self, *, timeout):
            return object()

    class FakeSubscriber:
        def __init__(self, *args, **kwargs) -> None:
            self.connections = {}
            self.closed = False
            self.commands: list[str] = []
            instances.append(self)

        def start(self, camera, *, start_timer, apply_pending_setup):
            self.connections[camera] = FakeConnection()

        def stop(self, camera=None):
            self.closed = True
            self.connections.clear()

        @contextmanager
        def exclusive_command(self, camera):
            yield

        def send_shell_command_with_result(self, camera, command, *, timeout_s):
            self.commands.append(command)
            return MavlinkShellCommandResult(command, "OK", "respond+exclusive")

    monkeypatch.setattr(device_runtime_workers, "DualPixhawkMavlinkSubscriber", FakeSubscriber)
    results: list[object] = []
    worker = standalone_gui.PixhawkUsbBootstrapWorker(
        camera="left",
        usb_url="/dev/ttyACM0",
        baud_candidates=(115200,),
        command="ifup eth0",
        fallback_command="ifup eth0 -m onboard",
    )
    worker.completed.connect(results.append)
    worker.run()

    assert results[0]["camera"] == "left"
    assert results[0]["baud"] == 115200
    assert instances[0].commands == ["ifup eth0"]
    assert instances[0].closed is True

def test_pixhawk_persistent_config_worker_serializes_shell_and_ftp() -> None:
    from Workstation import standalone_gui
    from Workstation.workstation_core.pixhawk_attitude import MavlinkShellCommandResult

    class FakeSubscriber:
        connections = {"left": object()}

        def __init__(self) -> None:
            self.active = False
            self.calls: list[tuple[str, str]] = []

        @contextmanager
        def exclusive_command(self, camera):
            assert self.active is False
            self.active = True
            try:
                yield
            finally:
                self.active = False

        def send_shell_command_with_result(self, camera, command, *, timeout_s):
            assert self.active is True
            self.calls.append(("shell", command))
            return MavlinkShellCommandResult(command, "saved", "respond+exclusive")

        def write_remote_file_mavftp(self, camera, path, content):
            assert self.active is True
            self.calls.append((path, content))

    subscriber = FakeSubscriber()
    results: list[object] = []
    worker = standalone_gui.PixhawkPersistentConfigWorker(
        subscriber,
        camera="left",
        command="param save",
        extras_text="ifup eth0\n",
        net_cfg_text="IPADDR=10.41.10.2\n",
    )
    worker.completed.connect(results.append)
    worker.run()

    assert [name for name, _value in subscriber.calls] == [
        "shell",
        "/fs/microsd/etc/extras.txt",
        "/fs/microsd/net.cfg",
    ]
    assert results[0]["camera"] == "left"

def test_pixhawk_rtcm_worker_queues_injection_and_reports_partial_failure(monkeypatch) -> None:
    from Workstation import standalone_gui
    from Workstation import device_runtime_workers

    class FakeSubscriber:
        def __init__(self) -> None:
            self.sent: list[tuple[str, bytes]] = []

        @contextmanager
        def exclusive_command(self, camera):
            yield

        def send_rtcm_data(self, camera, payload):
            self.sent.append((camera, payload))
            if camera == "right":
                raise TimeoutError("right unavailable")
            return 3

    monkeypatch.setattr(device_runtime_workers.QtCore.QTimer, "singleShot", lambda *_args: None)
    subscriber = FakeSubscriber()
    results: list[object] = []
    worker = standalone_gui.PixhawkRtcmInjectionWorker(subscriber, max_queue=2)
    worker.injected.connect(results.append)
    worker.enqueue(("left", "right"), b"RTCM")

    assert subscriber.sent == [("left", b"RTCM"), ("right", b"RTCM")]
    assert results == [
        {
            "payload_bytes": 4,
            "sent": (("left", 3),),
            "failed": (("right", "right unavailable"),),
            "dropped": 0,
        }
    ]

def test_workstation_gnss_settings_preserve_drone_route_and_derive_field_base(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui
    from Workstation.workstation_core.settings import AppSettings

    monkeypatch.setattr(standalone_gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(standalone_gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel()
    operation = AppSettings().gnss_operation.to_mapping()
    operation.update(
        {
            "operation_mode": "NETWORK_RTK_DIRECT",
            "require_drone_rtk_fixed": True,
            "routing": {
                **operation["routing"],
                "drone_enabled": True,
                "field_base_enabled": True,
            },
        }
    )

    panel.apply_workstation_system_settings({"gnss_operation": operation})

    applied = panel._workstation_setting_overrides["gnss_operation"]
    assert applied["require_drone_rtk_fixed"] is True
    assert applied["routing"]["drone_enabled"] is True
    assert applied["routing"]["field_base_enabled"] is False
    built = panel._build_gnss_operation_config()
    assert built.require_drone_rtk_fixed is True
    assert built.routing.drone_enabled is True
    assert built.routing.field_base_enabled is False

    panel._save_current_settings = lambda: None
    panel.close()
    panel.deleteLater()
    app.processEvents()

def test_pixhawk_connection_cards_keep_fixed_width_when_px4_timestamp_changes() -> None:
    from Workstation.status_model import RuntimeStatus
    from Workstation.workspaces import PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = WorkstationMainWindow(backend=_FakeBackend())
    window.status_adapter.stop()
    window.show()
    app.processEvents()

    left = window.pixhawk.indicators["pixhawk.left"]
    right = window.pixhawk.indicators["pixhawk.right"]
    assert left.minimumWidth() == PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX
    assert left.maximumWidth() == PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX
    assert right.minimumWidth() == PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX
    assert right.maximumWidth() == PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX

    def set_detail(timestamp: str, *, state: str = "LIVE") -> None:
        left.set_status(
            RuntimeStatus(
                key="pixhawk.left",
                label="P1",
                state=state,
                severity=(
                    StatusSeverity.BAD
                    if state == "ETH MAVLINK RX STALLED"
                    else StatusSeverity.WARN
                    if state != "LIVE"
                    else StatusSeverity.OK
                ),
                detail=f"MAVLink attitude age 0.10 s; rpy=1.00/2.00/3.00 deg; PX4 t={timestamp}",
            )
        )
        app.processEvents()

    set_detail("9.99")
    first_geometry = (left.width(), right.width(), right.x())
    geometries = []
    for state in (
        "QUALIFYING",
        "LIVE",
        "ATTITUDE STALE",
        "ETH MAVLINK RX STALLED",
        "RECOVERING",
    ):
        set_detail("1234567.890123", state=state)
        geometries.append((left.width(), right.width(), right.x()))

    assert all(geometry == first_geometry for geometry in geometries)
    assert first_geometry[:2] == (
        PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX,
        PIXHAWK_CONNECTION_INDICATOR_WIDTH_PX,
    )

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_status_adapter_exposes_sync_and_ned_baseline_evidence() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.camera_sync_status = "verified"
    backend.panel.camera_sync_reason = "RTCP/NTP pair gate verified"
    backend.panel.selected_runtime_frame_pair_delta_s = 0.012
    backend.panel.selected_runtime_frame_pair_basis = "camera_rtc"
    backend.panel.selected_runtime_frame_pair_camera_rtc_delta_s = 0.004
    backend.panel.latest_gps = {
        "left": SimpleNamespace(
            timestamp_s=time.monotonic() - 0.1,
            rtk_label="RTK_FIXED",
            fix_type=6,
            satellites_used=18,
            eph_m=0.02,
            epv_m=0.03,
        ),
        "right": SimpleNamespace(
            timestamp_s=time.monotonic() - 0.2,
            rtk_label="RTK_FIXED",
            fix_type=6,
            satellites_used=17,
            eph_m=0.03,
            epv_m=0.04,
        ),
    }
    backend.panel.latest_rtk_baseline = SimpleNamespace(
        baseline_m=1.25,
        north_m=1.0,
        east_m=0.7,
        down_m=-0.1,
        left_age_s=0.1,
        right_age_s=0.2,
        fix_quality="GOOD",
    )
    backend.panel.latest_p1_body_frd_baseline = SimpleNamespace(
        forward_m=0.7,
        right_m=-1.0,
        down_m=-0.1,
        attitude_age_s=0.05,
    )

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()
    sync = adapter.model.record("synchronization")
    gnss = adapter.model.record("gnss")
    assert sync is not None and sync.state == "SYNCED"
    assert "RTCP/NTP" in sync.detail
    assert gnss is not None and gnss.state == "FIX"
    assert "baseline NED=+1.000/+0.700/-0.100 m" in gnss.detail
    assert "P1→P2 @ P1 body FRD F/R/D=+0.700/-1.000/-0.100 m" in gnss.detail
    assert gnss.age_s is not None and gnss.age_s >= 0.19
    _ = app

def test_status_adapter_never_promotes_manager_ready_without_rover_data() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.gnss_operation_manager = SimpleNamespace(
        state=SimpleNamespace(value="READY"),
        config=SimpleNamespace(
            readiness=SimpleNamespace(maximum_gps_sample_age_s=1.0),
        ),
    )
    backend.panel.latest_gps = {}
    backend.panel.latest_rtk_baseline = None

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()

    gnss = adapter.model.record("gnss")
    assert gnss is not None
    assert gnss.state == "NO DATA"
    assert gnss.severity == StatusSeverity.UNKNOWN
    assert "P1=NO DATA" in gnss.detail
    assert "P2=NO DATA" in gnss.detail
    _ = app

def test_status_adapter_marks_old_rover_pair_and_baseline_stale() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    now_s = time.monotonic()
    backend.panel.gnss_operation_manager = SimpleNamespace(
        state=SimpleNamespace(value="READY"),
        config=SimpleNamespace(
            readiness=SimpleNamespace(maximum_gps_sample_age_s=1.0),
        ),
    )
    backend.panel.latest_gps = {
        side: SimpleNamespace(
            timestamp_s=now_s - 120.0,
            rtk_label="RTK_FIXED",
            fix_type=6,
            satellites_used=18,
            eph_m=0.02,
            epv_m=0.03,
        )
        for side in ("left", "right")
    }
    backend.panel.latest_rtk_baseline = SimpleNamespace(
        baseline_m=1.0,
        north_m=1.0,
        east_m=0.0,
        down_m=0.0,
        left_age_s=0.01,
        right_age_s=0.01,
        fix_quality="GOOD",
    )

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()

    gnss = adapter.model.record("gnss")
    assert gnss is not None
    assert gnss.state == "STALE"
    assert gnss.severity == StatusSeverity.WARN
    assert gnss.age_s is not None and gnss.age_s >= 119.0
    assert "baseline NED=+1.000/+0.000/+0.000 m" in gnss.detail
    assert "freshness=STALE" in gnss.detail
    _ = app

def test_status_adapter_rejects_future_rover_clock_and_ghost_baseline() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    now_s = time.monotonic()
    backend.panel.gnss_operation_manager = SimpleNamespace(
        state=SimpleNamespace(value="READY"),
        config=SimpleNamespace(
            readiness=SimpleNamespace(maximum_gps_sample_age_s=1.0),
        ),
    )
    backend.panel.latest_gps = {
        side: SimpleNamespace(
            timestamp_s=now_s + 60.0,
            rtk_label="RTK_FIXED",
            fix_type=6,
            satellites_used=18,
            eph_m=0.02,
            epv_m=0.03,
        )
        for side in ("left", "right")
    }
    backend.panel.latest_rtk_baseline = SimpleNamespace(
        baseline_m=1.0,
        north_m=1.0,
        east_m=0.0,
        down_m=0.0,
        left_age_s=0.1,
        right_age_s=0.1,
        fix_quality="GOOD",
    )

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()
    invalid_clock = adapter.model.record("gnss")
    assert invalid_clock is not None and invalid_clock.state == "NO DATA"
    assert invalid_clock.age_s is None
    assert "NO TIMESTAMP" in invalid_clock.detail
    assert "freshness=UNKNOWN" in invalid_clock.detail

    backend.panel.latest_gps = {}
    adapter.refresh()
    ghost = adapter.model.record("gnss")
    assert ghost is not None and ghost.state == "NO DATA"
    assert ghost.age_s is None
    assert "freshness=UNKNOWN" in ghost.detail
    _ = app

def test_cached_good_baseline_cannot_override_current_float_receivers() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    now_s = time.monotonic()
    backend.panel.latest_gps = {
        side: SimpleNamespace(
            timestamp_s=now_s - 0.1,
            rtk_label="RTK_FLOAT",
            fix_type=5,
            satellites_used=18,
            eph_m=0.02,
            epv_m=0.03,
        )
        for side in ("left", "right")
    }
    backend.panel.latest_rtk_baseline = SimpleNamespace(
        baseline_m=1.0,
        north_m=1.0,
        east_m=0.0,
        down_m=0.0,
        left_age_s=0.1,
        right_age_s=0.1,
        fix_quality="GOOD",
    )

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()

    gnss = adapter.model.record("gnss")
    assert gnss is not None
    assert gnss.state == "FLOAT"
    assert gnss.severity == StatusSeverity.WARN
    _ = app

def test_status_adapter_separates_base_gnss_from_rover_baseline() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.gnss_operation_manager = SimpleNamespace(
        state=SimpleNamespace(value="BASE_SURVEYING"),
        config=SimpleNamespace(
            operation_mode=SimpleNamespace(value="LOCAL_SURVEY_IN"),
            readiness=SimpleNamespace(maximum_correction_age_s=2.0),
        ),
    )
    backend.panel.gnss_base_serial_connected = True
    backend.panel.gnss_network_connected = False
    backend.panel.latest_gnss_base_survey_in = {
        "active": True,
        "valid": False,
        "duration_s": 42,
        "mean_accuracy_m": 0.12,
        "observations": 540,
    }
    backend.panel.gnss_base_rtcm_stats = {
        "frames": 0,
        "last_frame_s": None,
    }
    backend.panel.latest_rtk_baseline = SimpleNamespace(
        baseline_m=1.2,
        north_m=1.0,
        east_m=0.5,
        down_m=0.0,
        left_age_s=0.1,
        right_age_s=0.1,
        fix_quality="GOOD",
    )

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()
    base = adapter.model.record("gnss.base")
    aggregate = adapter.model.record("gnss")
    assert base is not None and base.state == "SURVEYING"
    assert "Survey-in=active(42s, 0.12m, obs=540)" in base.detail
    assert "baseline NED" not in base.detail
    assert aggregate is not None and "baseline NED" in aggregate.detail
    _ = app

def test_status_adapter_reports_ntrip_transport_and_rtcm_health_separately() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    now = time.monotonic()
    backend.panel.gnss_operation_manager = SimpleNamespace(
        state=SimpleNamespace(value="NTRIP_ACTIVE"),
        config=SimpleNamespace(
            operation_mode=SimpleNamespace(value="NETWORK_RTK_DIRECT"),
            network=SimpleNamespace(read_timeout_s=10.0),
            readiness=SimpleNamespace(maximum_correction_age_s=3.0),
        ),
    )
    backend.panel.gnss_network_connected = True
    backend.panel.gnss_network_thread = object()
    backend.panel.gnss_network_phase = "STREAMING"
    backend.panel.gnss_network_stats = SimpleNamespace(
        connected_at_monotonic_s=now - 10.0,
        last_chunk_monotonic_s=now - 0.5,
        bytes_received=2400,
        chunks_received=12,
    )
    backend.panel.gnss_base_rtcm_stats = {
        "frames": 4,
        "last_frame_s": now - 1.0,
    }

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()
    ntrip = adapter.model.record("gnss.ntrip")
    assert ntrip is not None and ntrip.state == "RTCM_ACTIVE"
    assert "chunks=12" in ntrip.detail
    assert "RTCM age=" in ntrip.detail

    backend.panel.gnss_base_rtcm_stats["last_frame_s"] = now - 4.0
    adapter.refresh()
    ntrip = adapter.model.record("gnss.ntrip")
    assert ntrip is not None and ntrip.state == "RTCM_STALE"
    assert ntrip.severity == StatusSeverity.BAD

    backend.panel.gnss_network_connected = False
    backend.panel.gnss_network_phase = "RECONNECTING"
    adapter.refresh()
    ntrip = adapter.model.record("gnss.ntrip")
    assert ntrip is not None and ntrip.state == "RECONNECTING"
    _ = app

def test_pixhawk_workspace_keeps_raw_ned_and_p1_body_frd_values_separate() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel.latest_rtk_baseline = SimpleNamespace(
        baseline_m=3.741,
        north_m=1.0,
        east_m=2.0,
        down_m=3.0,
        left_age_s=0.1,
        right_age_s=0.1,
        fix_quality="GOOD",
        warnings=(),
    )
    backend.panel.latest_p1_body_frd_baseline = SimpleNamespace(
        forward_m=1.0,
        right_m=2.0,
        down_m=3.0,
        attitude_age_s=0.05,
        target_frame="px4_frd",
        axis_contract="P1->P2 in P1 body FRD: [Forward, Right, Down]; metres",
    )
    window = WorkstationMainWindow(backend=backend)
    window.pixhawk.refresh({})
    assert window.pixhawk.baseline_values["north_m"].value.text() == "1.000"
    assert window.pixhawk.baseline_values["north_m"].unit.text() == "m"
    assert window.pixhawk.baseline_values["p1_body_frd_forward_m"].value.text() == "1.000"
    assert window.pixhawk.baseline_values["p1_body_frd_forward_m"].unit.text() == "m"
    assert "frame=" not in window.pixhawk.baseline_values["north_m"].meta.text()
    assert "Frame: local NED [North,East,Down]" in window.pixhawk.baseline_values["north_m"].toolTip()
    assert "Frame: P1 body FRD" in window.pixhawk.baseline_values["p1_body_frd_forward_m"].toolTip()
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_base_gnss_disconnect_requests_stop_without_waiting_on_ui_thread() -> None:
    from Workstation import standalone_gui

    calls: list[str] = []

    class Worker:
        def request_stop(self) -> None:
            calls.append("request_stop")

    class Thread:
        def wait(self, *_args: object) -> bool:
            raise AssertionError("UI disconnect must not wait for the serial thread")

    panel = SimpleNamespace(
        gnss_base_serial_worker=Worker(),
        gnss_base_serial_thread=Thread(),
    )
    standalone_gui.WirisStereoDistancePanel.disconnect_gnss_base_serial(panel)

    assert calls == ["request_stop"]

def test_wiris_command_catalog_exposes_lrf_distance_query() -> None:
    from Workstation import standalone_gui
    from Workstation.wiris_control_controller import WIRIS_COMMAND_CATALOG as controller_catalog

    expected = (
        "LRF",
        "Get laser distance",
        "GLRD",
        standalone_gui.LINE_TERMINATOR,
        "LRF(Laser Range Finder)가 측정한 거리 값을 읽습니다.",
    )

    assert expected in controller_catalog
    assert expected in standalone_gui.WIRIS_COMMAND_CATALOG

def test_alignment_workspace_does_not_expose_lrf_ui() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.navigation.select_context("processing.alignment")
    window.status_adapter.refresh()
    app.processEvents()

    assert not hasattr(window.alignment, "lrf_group")
    assert not hasattr(window.alignment, "lrf_indicators")
    assert "alignment.query_lrf" not in window.commands._specs
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_evidence_adapter_preserves_attitude_gps_detection_and_recording_contracts() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    now = time.monotonic()
    panel = SimpleNamespace(
        latest_extrinsics={
            "left": SimpleNamespace(
                roll_deg=1.0,
                pitch_deg=2.0,
                yaw_deg=3.0,
                q=(1.0, 0.0, 0.0, 0.0),
                timestamp_s=now - 0.25,
                px4_timestamp_s=123.5,
                px4_timestamp_us=123500000,
                px4_timestamp_sample_us=123400000,
                namespace="px4_1",
                topic="/px4_1/fmu/out/vehicle_attitude",
            )
        },
        latest_gps={
            "left": SimpleNamespace(
                rtk_label="RTK_FIXED",
                fix_type=6,
                satellites_used=18,
                eph_m=0.02,
                epv_m=0.03,
                latitude_deg=37.0,
                longitude_deg=127.0,
                altitude_m=20.0,
                timestamp_s=now - 0.1,
                px4_timestamp_s=22.0,
                namespace="px4_1",
                topic="/px4_1/fmu/out/vehicle_gps_position",
                time_source="px4_ros2",
            )
        },
        latest_rtk_baseline=SimpleNamespace(
            fix_quality="GOOD",
            north_m=1.0,
            east_m=2.0,
            down_m=-0.2,
            baseline_m=2.25,
            left_age_s=0.1,
            right_age_s=0.2,
            warnings=(),
        ),
        latest_p1_body_frd_baseline=SimpleNamespace(
            forward_m=2.0,
            right_m=1.0,
            down_m=-0.2,
            attitude_age_s=0.04,
            target_frame="px4_frd",
            axis_contract="P1->P2 in P1 body FRD: [Forward, Right, Down]; metres",
        ),
        latest_left=SimpleNamespace(
            timestamp_s=now - 0.05,
            width=1920,
            height=1080,
            x1=10.0,
            y1=20.0,
            x2=110.0,
            y2=220.0,
            center=(60.0, 120.0),
            confidence=0.9,
            class_id=0,
            class_name="person",
            candidate_class_ids=(0,),
            timestamp_metadata={"source_timestamp_s": 100.0, "timestamp_source": "rtcp_ntp"},
        ),
        latest_right=None,
        camera_stream_stats={"left": {"inferred_hz": 15.0}, "right": {}},
        yolo_enabled=True,
        recording_session_id="session-1",
        distance_recording_enabled=False,
        video_recording_enabled=False,
        distance_recording_rows=[{"row": 1}],
        camera_recording_rows={},
        px4_recording_rows={},
        gnss_base_recording_rows=[],
        video_recording_paths={},
        _session_save_thread=None,
        _session_save_retry_required=False,
        _session_save_finalized=True,
        _recording_elapsed_s=lambda _now: 1.5,
        _recording_session_dir=lambda _session: "/tmp/session-1",
        recording_status_label=QtWidgets.QLabel("saved"),
    )
    evidence = EvidenceAdapter(panel)
    attitude = evidence.attitude("left", now_s=now)
    gps = evidence.gps("left", now_s=now)
    detection = evidence.detection("left", now_s=now)
    recording = evidence.recording(now_s=now)
    assert attitude["frame_contract"].startswith("PX4")
    assert attitude["px4_timestamp_s"] == 123.5
    assert gps["position_contract"].startswith("WGS84")
    assert evidence.baseline()["axis_contract"] == "NED = north / east / down; metres"
    assert evidence.baseline()["p1_body_frd_state"] == "READY"
    assert evidence.baseline()["p1_body_frd_forward_m"] == pytest.approx(2.0)
    assert evidence.baseline()["p1_body_frd_axis_contract"].startswith("P1->P2 in P1 body FRD")
    assert detection["pixel_contract"].startswith("bbox")
    assert detection["center_u_px"] == 60.0
    assert recording["state"] == "FINALIZED"
    assert recording["sample_counts"]["pairs"] == 1
    _ = app

def test_workstation_evidence_preserves_mavlink_link_provenance() -> None:
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    connection = SimpleNamespace(target_system=11, target_component=42)
    subscriber = SimpleNamespace(
        urls={"left": "udpin:0.0.0.0:14550"},
        connections={"left": connection},
        _heartbeat_logged={"left"},
    )
    panel = SimpleNamespace(
        mavlink_pixhawk=subscriber,
        latest_extrinsics={
            "left": CameraExtrinsicState(
                camera="left",
                namespace="",
                roll_deg=1.0,
                pitch_deg=2.0,
                yaw_deg=3.0,
                timestamp_s=10.0,
                topic="mavlink/ATTITUDE",
                q=(1.0, 0.0, 0.0, 0.0),
            )
        },
        latest_gps={
            "left": GpsPositionState(
                camera="left",
                namespace="",
                timestamp_s=10.0,
                topic="mavlink/GPS_RAW_INT",
                latitude_deg=37.0,
                longitude_deg=127.0,
                altitude_m=10.0,
                fix_type=6,
            )
        },
    )

    evidence = EvidenceAdapter(panel)
    attitude = evidence.attitude("left", now_s=10.1)
    gps = evidence.gps("left", now_s=10.1)

    for snapshot in (attitude, gps):
        assert snapshot["telemetry_transport"] == "MAVLink"
        assert snapshot["mavlink_endpoint"] == "udpin:0.0.0.0:14550"
        assert snapshot["mavlink_link_open"] is True
        assert snapshot["mavlink_heartbeat_seen"] is True
        assert snapshot["mavlink_target_system"] == 11
        assert snapshot["mavlink_target_component"] == 42

def test_workstation_px4_timeline_persists_mavlink_link_provenance(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(standalone_gui, "APP_ROOT", tmp_path)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel()
    now_s = time.monotonic()
    panel.recording_session_id = "provenance-session"
    panel.recording_session_started_monotonic_ns = int((now_s - 1.0) * 1_000_000_000)
    panel.recording_session_started_unix_ns = 1_800_000_000_000_000_000
    panel.recording_session_anchor_uncertainty_ns = 100
    panel.distance_recording_enabled = True
    panel.video_recording_enabled = False
    panel.mavlink_pixhawk = SimpleNamespace(
        urls={"left": "udpin:0.0.0.0:14550"},
        connections={"left": SimpleNamespace(target_system=11, target_component=42)},
        _heartbeat_logged={"left"},
    )

    panel._record_external_timeline_row(
        "left",
        "pixhawk_attitude",
        {
            "topic": "mavlink/ATTITUDE",
            "sample_app_timestamp_s": now_s,
        },
    )
    row = panel.px4_recording_rows["left"][0]
    assert row["mavlink_endpoint"] == "udpin:0.0.0.0:14550"
    assert row["mavlink_link_open"] is True
    assert row["mavlink_heartbeat_seen"] is True
    assert row["mavlink_target_system"] == 11
    assert row["mavlink_target_component"] == 42

    # This test verifies one timeline row, not recording finalization. Avoid
    # launching a session-save or fake-MAVLink disconnect worker from
    # closeEvent after the assertion.
    panel.distance_recording_enabled = False
    panel.mavlink_pixhawk = None
    panel.close()
    panel.deleteLater()
    app.processEvents()

def test_lrf_alignment_command_is_removed_from_workstation_ui() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.status_adapter.stop()
    backend.calls.clear()

    window._select_context("processing.alignment")
    assert "query_lrf_once" not in backend.calls

    window._select_context("devices.cameras")
    assert "query_lrf_once" not in backend.calls

    assert window.commands.invoke("alignment.query_lrf") is False
    assert "query_lrf_once" not in backend.calls

    window.close()
    window.deleteLater()
    app.processEvents()

def test_diagnostics_gnss_tab_surfaces_the_registered_ntrip_status() -> None:
    """Regression: the GNSS Diagnostics tab used to ask ``_status_text`` for
    a plain ``"gnss"`` key that was never registered in StatusModel (the
    real key is ``"gnss.ntrip"``, built by ``StatusModel._refresh_ntrip``
    with phase/GGA age/RTCM age/reconnects/error detail). The typo meant
    that detailed line silently rendered as "NO DATA" regardless of what
    NTRIP was actually doing.
    """

    from Workstation.status_model import BackendStatusAdapter
    from Workstation.workspaces import DiagnosticsContent

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    now = time.monotonic()
    backend.panel.gnss_operation_manager = SimpleNamespace(
        state=SimpleNamespace(value="NTRIP_ACTIVE"),
        config=SimpleNamespace(
            operation_mode=SimpleNamespace(value="NETWORK_RTK_DIRECT"),
            network=SimpleNamespace(read_timeout_s=10.0, caster_host="RTS1.ngii.go.kr", mountpoint="RTK-RTCM32"),
            readiness=SimpleNamespace(maximum_correction_age_s=3.0),
        ),
    )
    backend.panel.gnss_network_connected = True
    backend.panel.gnss_network_thread = object()
    backend.panel.gnss_network_phase = "STREAMING"
    backend.panel.gnss_network_stats = SimpleNamespace(
        connected_at_monotonic_s=now - 10.0,
        last_chunk_monotonic_s=now - 0.5,
        last_gga_monotonic_s=now - 2.0,
        gga_sent_count=3,
        bytes_received=2400,
        chunks_received=12,
    )
    backend.panel.gnss_base_rtcm_stats = {"frames": 4, "last_frame_s": now - 1.0}

    adapter = BackendStatusAdapter(backend)
    adapter.refresh()
    diagnostics = DiagnosticsContent(backend, adapter.model)
    diagnostics.tabs.setCurrentIndex(3)  # "GNSS" tab

    diagnostics.refresh(adapter.model.snapshot())

    text = diagnostics.gnss_view.toPlainText()
    assert "NO DATA" not in text.split("\n\n", 1)[0]
    assert "phase=STREAMING" in text
    assert "GGA sent=3" in text
    assert "caster=RTS1.ngii.go.kr/RTK-RTCM32" in text

def test_pixhawk_verified_usb_recovery_is_visible_and_stall_gated() -> None:
    from Workstation.status_model import RuntimeStatus

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    window.pixhawk.refresh(window.status_adapter.model.snapshot())
    pixhawk_layout = window.pixhawk.layout()
    assert [
        pixhawk_layout.itemAt(index).widget().title()
        for index in range(2, 12)
        if pixhawk_layout.itemAt(index).widget() is not None
    ] == [
        "Connection",
        "Network candidates and assignments",
        "Attitude (R/P/Y)",
        "P1/P2 GNSS receivers",
        "IMU magnetometer (MAVLink)",
        "CAN magnetometer (dedicated MAVLink)",
        "Pixhawk Parameters · calibration-owned groups",
        "P1→P2 baseline",
        "P1→P2 baseline / Pixhawk yaw",
        "ETH MAV",
    ]
    top_actions = pixhawk_layout.itemAt(1).layout()
    assert [top_actions.itemAt(index).widget().text() for index in range(4)] == [
        "Connect",
        "Disconnect",
        "Find USB",
        "Find network",
    ]
    assert window.pixhawk.operation_buttons["pixhawk.recover_eth_mavlink.p1"].text() == "P1 verified-USB reboot"
    assert window.pixhawk.operation_buttons["pixhawk.recover_eth_mavlink.p2"].text() == "P2 verified-USB reboot"
    assert window.pixhawk.receiver_model.rowCount() == 4
    assert window.pixhawk.gnss_source_combos["left"].currentData() == "GPS1"
    assert window.pixhawk.gnss_source_combos["right"].currentData() == "GPS1"
    assert window.pixhawk.gnss_source_summary.text() == "계산 source: P1 GPS1 → P2 GPS1"
    assert window.pixhawk.recovery_values["left"]["usb"].value.text() == "/dev/ttyACM0"
    assert window.pixhawk.recovery_values["right"]["baud"].value.text() == "115200"

    backend.panel.pixhawk_usb_recovery_status = {
        "left": {
        "state": "COMMAND_SENT",
        "detail": "USB reboot command sent; Ethernet telemetry 확인 중",
        "updated_s": 10.0,
        }
    }
    backend.panel.pixhawk_usb_identity_evidence = [
        {
            "device": "/dev/ttyACM0",
            "baud": 115200,
            "system_id": 1,
            "component_id": 1,
            "observed_monotonic_s": 10.0,
        }
    ]
    window.pixhawk.refresh(window.status_adapter.model.snapshot())
    assert window.pixhawk.recovery_values["left"]["recovery"].value.text() == "COMMAND_SENT"
    assert "HB NO LINK" in window.pixhawk.recovery_values["left"]["telemetry"].value.text()

    assert window.commands.action("pixhawk.recover_eth_mavlink.p1").isEnabled() is False
    snapshot = window.status_adapter.model.snapshot()
    snapshot["pixhawk.left"] = RuntimeStatus(
        key="pixhawk.left",
        label="P1",
        state="ETH MAVLINK RX STALLED",
        severity=StatusSeverity.BAD,
    )
    window._refresh_from_status(snapshot)
    assert window.commands.invoke("pixhawk.recover_eth_mavlink.p1") is True
    assert backend.calls[-1] == ("pixhawk_recover_eth_mavlink", "left")

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()

def test_pixhawk_recovery_evidence_separates_usb_control_and_attitude() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    left_baud = QtWidgets.QSpinBox()
    left_baud.setRange(1, 10_000_000)
    left_baud.setValue(115_200)
    panel = SimpleNamespace(
        left_pixhawk_bootstrap_url_edit=QtWidgets.QLineEdit("/dev/ttyACM0"),
        left_pixhawk_bootstrap_baud_spin=left_baud,
        pixhawk_usb_recovery_thread=None,
        pixhawk_usb_recovery_status={
            "left": {"state": "COMMAND_SENT", "detail": "reboot command sent", "updated_s": 99.0}
        },
        pixhawk_usb_identity_evidence=[
            {
                "device": "/dev/serial/by-id/p1",
                "resolved_device": "/dev/ttyACM0",
                "baud": 115200,
                "system_id": 1,
                "component_id": 1,
                "observed_monotonic_s": 99.0,
            }
        ],
        mavlink_pixhawk=SimpleNamespace(
            urls={"left": "udpin:0.0.0.0:14550"},
            connections={"left": object()},
            _heartbeat_logged={"left"},
            mavlink_health_snapshot=lambda _side, **_kwargs: SimpleNamespace(
                state=SimpleNamespace(value="LIVE"),
                listener_open=True,
                heartbeat_rx_age_s=0.1,
                attitude_rx_age_s=0.1,
                qualification_elapsed_s=600.0,
            ),
        ),
        latest_extrinsics={
            "left": SimpleNamespace(
                topic="mavlink/ATTITUDE",
                timestamp_s=99.5,
                roll_deg=1.0,
                pitch_deg=2.0,
                yaw_deg=3.0,
            )
        },
    )

    recovery = EvidenceAdapter(panel).pixhawk_recovery("left", now_s=100.0)

    assert recovery["usb_url"] == "/dev/serial/by-id/p1"
    assert recovery["usb_baud"] == 115_200
    assert recovery["state"] == "RECOVERED"
    assert recovery["heartbeat_state"] == "OK"
    assert recovery["attitude_state"] == "OK"
    assert recovery["mavlink_endpoint"] == "udpin:0.0.0.0:14550"
    assert recovery["usb_system_id"] == 1
    assert "10-minute qualification" in str(recovery["contract"])

    left_baud.deleteLater()
    panel.left_pixhawk_bootstrap_url_edit.deleteLater()
    app.processEvents()

def test_workstation_exposes_phase_one_camera_pixhawk_and_gnss_commands() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    backend.panel._mavlink_open_sides = lambda: ("left", "right")
    backend.panel.px4_mag_managers = {
        "left": SimpleNamespace(armed=False),
        "right": SimpleNamespace(armed=False),
    }
    window = WorkstationMainWindow(backend=backend)
    window.show()
    app.processEvents()

    camera_commands = (
        "camera.discover_host_ip",
        "camera.check_rtsp",
        "camera.check_control",
        "camera.enable_rtsp",
        "camera.reactivate_left",
        "camera.reactivate_right",
        "camera.swap",
        "camera.save_settings",
    )
    pixhawk_commands = (
        "pixhawk.toggle_mavlink",
        "pixhawk.reset_level_calibration",
        "alignment.swap_pixhawks",
    )
    gnss_commands = (
        "gnss.base_discover",
        "gnss.base_connect",
        "gnss.base_disconnect",
        "gnss.survey_start",
        "gnss.survey_stop",
        "gnss.ntrip_start",
        "gnss.ntrip_stop",
    )
    calibration_commands = (
        "calibration.capture_images",
        "calibration.save_intrinsics",
        "calibration.load_intrinsics",
    )

    assert all(window.camera.operation_buttons[command].isEnabled() for command in camera_commands)
    assert all(window.pixhawk.operation_buttons[command].isEnabled() for command in pixhawk_commands)
    assert all(window.base_gnss.operation_buttons[command].isEnabled() for command in gnss_commands)
    assert all(window.calibration.operation_buttons[command].isEnabled() for command in calibration_commands)
    assert not window.calibration.operation_buttons["calibration.run_intrinsic"].isEnabled()
    assert window.commands.invoke("calibration.run_intrinsic") is False
    for command in (*camera_commands, *pixhawk_commands, *gnss_commands, *calibration_commands):
        assert window.commands.invoke(command) is True

    expected_calls = [
        "camera_discover_host_ip",
        "camera_check_rtsp",
        "camera_check_control",
        "camera_enable_rtsp",
        "camera_reactivate",
        "camera_reactivate",
        "camera_swap",
        "camera_save_settings",
        "pixhawk_toggle_mavlink",
        "pixhawk_reset_level_calibration",
        "alignment_swap_pixhawks",
        "gnss_base_discover",
        "gnss_base_connect",
        "gnss_base_disconnect",
        "gnss_survey_start",
        "gnss_survey_stop",
        "gnss_ntrip_start",
        "gnss_ntrip_stop",
        "calibration_apply_setup",
        "calibration_capture_images",
        "calibration_save_intrinsics",
        "calibration_load_intrinsics",
    ]
    recent_calls = backend.calls[-len(expected_calls):]
    assert [call[0] if isinstance(call, tuple) else call for call in recent_calls] == expected_calls
    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()
