"""Independent threshold tests for per-side Ethernet MAVLink liveness."""

from __future__ import annotations

import threading

import pytest

from Workstation.workstation_core.mavlink_liveness import (
    MAVLINK_INITIAL_RX_TIMEOUT_S,
    MAVLINK_QUALIFICATION_S,
    MavlinkLivenessState,
    MavlinkSideLiveness,
)


def _opened(*, recovering: bool = False) -> tuple[MavlinkSideLiveness, int]:
    model = MavlinkSideLiveness(
        side="left",
        expected_system_id=1,
        endpoint="udpin:0.0.0.0:14550",
    )
    generation = model.open_listener(now_s=0.0, recovering=recovering)
    return model, generation


def _observe_pair(model: MavlinkSideLiveness, generation: int, at_s: float) -> None:
    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="HEARTBEAT",
        received_at_s=at_s,
        generation=generation,
    )
    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="ATTITUDE",
        received_at_s=at_s,
        generation=generation,
    )


def _keep_fresh_through(model: MavlinkSideLiveness, generation: int, end_s: int) -> None:
    for second in range(1, end_s + 1):
        _observe_pair(model, generation, float(second))


def test_listener_and_expected_identity_gate() -> None:
    model = MavlinkSideLiveness(side="left", expected_system_id=1)
    assert model.snapshot(now_s=0.0).state is MavlinkLivenessState.DISCONNECTED

    generation = model.open_listener(endpoint="udpin:0.0.0.0:14550", now_s=1.0)
    assert model.snapshot(now_s=1.0).state is MavlinkLivenessState.CONNECTING
    assert not model.observe_packet(
        system_id=2,
        component_id=1,
        message_type="HEARTBEAT",
        received_at_s=1.1,
        generation=generation,
    )
    wrong = model.snapshot(now_s=1.1)
    assert wrong.state is MavlinkLivenessState.CONNECTING
    assert wrong.last_any_rx_monotonic_s is None
    assert not wrong.identity_verified

    assert model.observe_packet(
        system_id=1,
        component_id=42,
        message_type="SYS_STATUS",
        received_at_s=1.15,
        generation=generation,
    )
    expected_non_heartbeat = model.snapshot(now_s=1.15)
    assert expected_non_heartbeat.state is MavlinkLivenessState.CONNECTING
    assert expected_non_heartbeat.last_any_rx_monotonic_s == pytest.approx(1.15)
    assert expected_non_heartbeat.observed_system_id == 1
    assert not expected_non_heartbeat.identity_verified

    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="HEARTBEAT",
        received_at_s=1.2,
        generation=generation,
    )
    verified = model.snapshot(now_s=1.2)
    assert verified.state is MavlinkLivenessState.QUALIFYING
    assert verified.identity_verified
    assert verified.observed_system_id == 1


def test_initial_expected_heartbeat_timeout_is_bounded_and_exact() -> None:
    model, generation = _opened()
    assert model.observe_packet(
        system_id=1,
        component_id=42,
        message_type="SYS_STATUS",
        received_at_s=1.0,
        generation=generation,
    )

    before = model.snapshot(now_s=MAVLINK_INITIAL_RX_TIMEOUT_S - 0.001)
    assert before.state is MavlinkLivenessState.CONNECTING
    assert before.stall_reason == "AWAITING_EXPECTED_HEARTBEAT"

    exact = model.snapshot(now_s=MAVLINK_INITIAL_RX_TIMEOUT_S)
    assert exact.state is MavlinkLivenessState.INITIAL_RX_TIMEOUT
    assert exact.stall_reason == "EXPECTED_HEARTBEAT_NOT_RECEIVED"
    assert exact.listener_age_s == pytest.approx(MAVLINK_INITIAL_RX_TIMEOUT_S)
    assert not exact.rtcm_eligible
    assert not exact.command_eligible

    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="HEARTBEAT",
        received_at_s=MAVLINK_INITIAL_RX_TIMEOUT_S + 0.1,
        generation=generation,
    )
    resumed = model.snapshot(now_s=MAVLINK_INITIAL_RX_TIMEOUT_S + 0.1)
    assert resumed.state is MavlinkLivenessState.QUALIFYING
    assert resumed.identity_verified
    assert resumed.qualification_elapsed_s == 0.0


def test_bad_data_and_local_echo_are_not_liveness_evidence() -> None:
    model, generation = _opened()
    assert not model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="BAD_DATA",
        received_at_s=1.0,
        generation=generation,
    )
    assert not model.observe_packet(
        system_id=255,
        component_id=190,
        message_type="HEARTBEAT",
        received_at_s=1.1,
        generation=generation,
    )
    assert model.snapshot(now_s=1.1).last_any_rx_monotonic_s is None


def test_qualification_boundary_is_exactly_600_continuous_seconds() -> None:
    model, generation = _opened()
    _observe_pair(model, generation, 0.0)
    _keep_fresh_through(model, generation, 599)
    _observe_pair(model, generation, 599.999)

    before = model.snapshot(now_s=599.999)
    assert before.state is MavlinkLivenessState.QUALIFYING
    assert before.qualification_elapsed_s == pytest.approx(599.999)

    _observe_pair(model, generation, MAVLINK_QUALIFICATION_S)
    exact = model.snapshot(now_s=MAVLINK_QUALIFICATION_S)
    assert exact.state is MavlinkLivenessState.LIVE
    assert exact.qualification_elapsed_s == pytest.approx(600.0)
    assert exact.rtcm_eligible
    assert exact.command_eligible


def test_attitude_stale_boundary_does_not_claim_all_rx_stall() -> None:
    model, generation = _opened()
    _observe_pair(model, generation, 0.0)
    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="HEARTBEAT",
        received_at_s=2.0,
        generation=generation,
    )
    assert model.snapshot(now_s=2.0).state is MavlinkLivenessState.QUALIFYING

    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="SYS_STATUS",
        received_at_s=2.001,
        generation=generation,
    )
    stale = model.snapshot(now_s=2.001)
    assert stale.state is MavlinkLivenessState.ATTITUDE_STALE
    assert stale.stall_reason == "ATTITUDE_STALE"
    assert stale.rtcm_eligible


def test_all_expected_rx_stalls_at_ten_seconds_not_before() -> None:
    model, generation = _opened()
    _observe_pair(model, generation, 0.0)

    before = model.snapshot(now_s=9.999)
    assert before.state is not MavlinkLivenessState.RX_STALLED
    assert before.rtcm_eligible

    exact = model.snapshot(now_s=10.0)
    assert exact.state is MavlinkLivenessState.RX_STALLED
    assert exact.stall_reason == "ALL_EXPECTED_RX_SILENT"
    assert not exact.rtcm_eligible
    assert not exact.command_eligible


def test_stale_heartbeat_with_other_rx_is_not_all_rx_stall() -> None:
    model, generation = _opened()
    _observe_pair(model, generation, 0.0)
    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="ATTITUDE",
        received_at_s=3.001,
        generation=generation,
    )
    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="SYS_STATUS",
        received_at_s=3.001,
        generation=generation,
    )

    snapshot = model.snapshot(now_s=3.001)
    assert snapshot.state is MavlinkLivenessState.QUALIFYING
    assert snapshot.stall_reason == "HEARTBEAT_STALE"
    assert snapshot.rtcm_eligible
    assert snapshot.qualification_elapsed_s == 0.0


def test_stalled_packet_resume_is_recovering_until_requalified() -> None:
    model, generation = _opened()
    _observe_pair(model, generation, 0.0)
    assert model.snapshot(now_s=10.0).state is MavlinkLivenessState.RX_STALLED

    _observe_pair(model, generation, 10.1)
    resumed = model.snapshot(now_s=10.1)
    assert resumed.state is MavlinkLivenessState.RECOVERING
    assert resumed.qualification_elapsed_s == 0.0

    for second in range(1, 601):
        _observe_pair(model, generation, 10.1 + second)
    qualified = model.snapshot(now_s=610.1)
    assert qualified.state is MavlinkLivenessState.LIVE
    assert qualified.qualification_elapsed_s == pytest.approx(600.0)


def test_remote_reboot_invalidates_identity_without_replacing_host_generation() -> None:
    model, generation = _opened()
    _observe_pair(model, generation, 0.0)
    _keep_fresh_through(model, generation, 600)
    assert model.snapshot(now_s=600.0).state is MavlinkLivenessState.LIVE

    assert model.note_remote_reboot(now_s=600.1)
    after_reset = model.snapshot(now_s=600.1)
    assert after_reset.connection_generation == generation
    assert after_reset.state is MavlinkLivenessState.CONNECTING
    assert not after_reset.identity_verified
    assert after_reset.qualification_elapsed_s == 0.0
    assert not after_reset.rtcm_eligible

    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="ATTITUDE",
        received_at_s=600.1,
        generation=generation,
    )
    assert model.snapshot(now_s=600.1).state is MavlinkLivenessState.CONNECTING
    assert model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="HEARTBEAT",
        received_at_s=600.2,
        generation=generation,
    )
    recovering = model.snapshot(now_s=600.2)
    assert recovering.state is MavlinkLivenessState.RECOVERING
    assert recovering.identity_verified
    assert recovering.qualification_elapsed_s == 0.0
    assert recovering.rtcm_eligible


def test_replaced_generation_rejects_late_packet() -> None:
    model, old_generation = _opened()
    new_generation = model.open_listener(now_s=1.0)
    assert new_generation == old_generation + 1
    assert not model.observe_packet(
        system_id=1,
        component_id=1,
        message_type="HEARTBEAT",
        received_at_s=1.1,
        generation=old_generation,
    )
    assert model.snapshot(now_s=1.1).state is MavlinkLivenessState.CONNECTING


def test_active_command_suppresses_false_stall_then_requires_fresh_evidence() -> None:
    model, generation = _opened()
    _observe_pair(model, generation, 0.0)
    model.begin_command("PARAM SYNC", now_s=1.0)

    during = model.snapshot(now_s=100.0)
    assert during.state is MavlinkLivenessState.QUALIFYING
    assert during.active_command == "PARAM SYNC"
    assert during.stall_reason == "ACTIVE_COMMAND"
    assert not during.command_eligible

    model.end_command(now_s=100.0)
    grace = model.snapshot(now_s=109.999)
    assert grace.state is MavlinkLivenessState.QUALIFYING
    assert grace.stall_reason == "AWAITING_POST_COMMAND_HEARTBEAT_ATTITUDE"

    stalled = model.snapshot(now_s=110.0)
    assert stalled.state is MavlinkLivenessState.RX_STALLED


def test_post_command_heartbeat_and_attitude_restart_qualification() -> None:
    model, generation = _opened()
    _observe_pair(model, generation, 0.0)
    model.begin_command("PARAM SYNC", now_s=1.0)
    model.end_command(now_s=5.0)
    _observe_pair(model, generation, 5.1)

    snapshot = model.snapshot(now_s=5.1)
    assert snapshot.state is MavlinkLivenessState.QUALIFYING
    assert snapshot.qualification_started_monotonic_s == pytest.approx(5.1)
    assert snapshot.qualification_elapsed_s == 0.0


class _TransportMessage:
    def __init__(
        self,
        message_type: str,
        *,
        system_id: int,
        component_id: int = 1,
        raw: bytes | None = None,
    ) -> None:
        self._message_type = message_type
        self._system_id = system_id
        self._component_id = component_id
        self._raw = raw or f"{message_type}:{system_id}".encode()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def get_type(self) -> str:
        return self._message_type

    def get_srcSystem(self) -> int:
        return self._system_id

    def get_srcComponent(self) -> int:
        return self._component_id

    def get_msgbuf(self) -> bytes:
        return self._raw


class _TransportMav:
    def __init__(self, *, heartbeat_error: Exception | None = None) -> None:
        self.heartbeats: list[tuple[object, ...]] = []
        self.commands: list[tuple[object, ...]] = []
        self.heartbeat_error = heartbeat_error

    def heartbeat_send(self, *args: object) -> None:
        self.heartbeats.append(args)
        if self.heartbeat_error is not None:
            raise self.heartbeat_error

    def command_long_send(self, *args: object) -> None:
        self.commands.append(args)


class _TransportConnection:
    target_system = 1
    target_component = 1

    def __init__(self, mav: _TransportMav | None = None) -> None:
        self.mav = mav or _TransportMav()
        self.udp_server = True
        self.clients: set[tuple[str, int]] = set()
        self.clients_last_alive: dict[tuple[str, int], float] = {}
        self.mavlink = type(
            "MavlinkConstants",
            (),
            {
                "MAV_TYPE_GCS": 6,
                "MAV_AUTOPILOT_INVALID": 8,
                "MAV_STATE_ACTIVE": 4,
                "MAV_CMD_SET_MESSAGE_INTERVAL": 511,
            },
        )()
        self.messages: list[object] = []
        self.closed = False

    def recv_match(self, **_kwargs):
        return self.messages.pop(0) if self.messages else None

    def close(self) -> None:
        self.closed = True


class _TrackedReentrantLock:
    """Expose lock depth so fake MAVLink writers can assert ownership."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.depth = 0
        self.max_depth = 0

    def acquire(self, blocking: bool = True) -> bool:
        acquired = self._lock.acquire(blocking=blocking)
        if acquired:
            self.depth += 1
            self.max_depth = max(self.max_depth, self.depth)
        return acquired

    def release(self) -> None:
        self.depth -= 1
        self._lock.release()

    def __enter__(self):
        assert self.acquire()
        return self

    def __exit__(self, *_args: object) -> None:
        self.release()


def test_operational_connection_declares_gcs_source_identity(monkeypatch) -> None:
    from pymavlink import mavutil

    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    connection = _TransportConnection()
    captured: dict[str, object] = {}

    def connection_factory(url: str, **kwargs):
        captured["url"] = url
        captured.update(kwargs)
        return connection

    monkeypatch.setattr(mavutil, "mavlink_connection", connection_factory)
    monkeypatch.setattr(pixhawk_attitude, "_assert_mavlink_udpin_available", lambda _url: None)
    monkeypatch.setattr(pixhawk_attitude, "_disable_mavlink_udpin_reuse", lambda *_args: None)
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
        peer_hosts={"left": "10.0.0.2"},
    )

    subscriber.start("left", start_timer=False)

    assert captured["source_system"] == 255
    assert captured["source_component"] == 190
    assert connection.clients == {("10.0.0.2", 14550)}
    assert len(connection.mav.heartbeats) == 1
    assert connection.mav.heartbeats[0] == (6, 8, 0, 0, 4)
    clock["value"] = 0.999
    subscriber.spin_once()
    assert len(connection.mav.heartbeats) == 1
    clock["value"] = 1.0
    subscriber.spin_once()
    assert len(connection.mav.heartbeats) == 2
    clock["value"] = 20.0
    subscriber.spin_once()
    assert len(connection.mav.heartbeats) == 3
    timed_out = subscriber.mavlink_health_snapshot("left", now_s=20.0)
    assert timed_out.state is MavlinkLivenessState.INITIAL_RX_TIMEOUT
    assert timed_out.last_gcs_heartbeat_tx_monotonic_s == 20.0
    assert subscriber.mavlink_health_snapshot("left", now_s=0.0).listener_open
    subscriber.stop("left")
    assert connection.closed
    assert subscriber.mavlink_health_snapshot("left", now_s=0.0).state is MavlinkLivenessState.DISCONNECTED


def test_gcs_heartbeat_without_explicit_udp_peer_starts_after_expected_heartbeat_and_never_catches_up(monkeypatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    connection = _TransportConnection()
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
    )
    subscriber.connections["left"] = connection

    connection.messages.append(_TransportMessage("HEARTBEAT", system_id=2))
    subscriber.spin_once()
    assert connection.mav.heartbeats == []

    connection.messages.append(_TransportMessage("HEARTBEAT", system_id=1))
    subscriber.spin_once()
    assert len(connection.mav.heartbeats) == 1
    assert connection.mav.heartbeats[0] == (6, 8, 0, 0, 4)

    clock["value"] = 0.999
    subscriber.spin_once()
    assert len(connection.mav.heartbeats) == 1

    clock["value"] = 1.0
    subscriber.spin_once()
    assert len(connection.mav.heartbeats) == 2

    clock["value"] = 20.0
    subscriber.spin_once()
    assert len(connection.mav.heartbeats) == 3
    assert subscriber.mavlink_health_snapshot("left").last_gcs_heartbeat_tx_monotonic_s == 20.0

    subscriber.stop("left")
    clock["value"] = 21.0
    subscriber.spin_once()
    assert len(connection.mav.heartbeats) == 3


def test_duplicate_transport_arrival_refreshes_liveness_before_sample_filter(monkeypatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    connection = _TransportConnection()
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
    )
    subscriber.connections["left"] = connection
    samples: list[object] = []
    subscriber.extrinsic_changed.connect(lambda _side, state: samples.append(state))
    raw = b"same-broadcast-and-unicast-packet"

    connection.messages.append(_TransportMessage("ATTITUDE", system_id=1, raw=raw))
    subscriber.spin_once()
    clock["value"] = 0.1
    connection.messages.append(_TransportMessage("ATTITUDE", system_id=1, raw=raw))
    subscriber.spin_once()

    snapshot = subscriber.mavlink_health_snapshot("left")
    assert snapshot.last_any_rx_monotonic_s == pytest.approx(0.1)
    assert snapshot.last_attitude_rx_monotonic_s == pytest.approx(0.1)
    assert len(samples) == 1


def test_gcs_heartbeat_failure_is_bounded_to_one_attempt_per_second(monkeypatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    mav = _TransportMav(heartbeat_error=OSError("tx sentinel"))
    connection = _TransportConnection(mav)
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
    )
    subscriber.connections["left"] = connection
    statuses: list[str] = []
    subscriber.status_changed.connect(statuses.append)
    connection.messages.append(_TransportMessage("HEARTBEAT", system_id=1))

    subscriber.spin_once()
    clock["value"] = 0.1
    subscriber.spin_once()
    clock["value"] = 1.0
    subscriber.spin_once()

    assert len(mav.heartbeats) == 2
    assert len([text for text in statuses if "tx sentinel" in text]) == 2


def test_liveness_transition_event_is_emitted_once_not_on_every_timer_tick(monkeypatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    connection = _TransportConnection()
    connection.messages.extend(
        (
            _TransportMessage("HEARTBEAT", system_id=1, raw=b"hb"),
            _TransportMessage("ATTITUDE", system_id=1, raw=b"att"),
        )
    )
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
    )
    subscriber.connections["left"] = connection
    statuses: list[str] = []
    subscriber.status_changed.connect(statuses.append)

    subscriber.spin_once()
    clock["value"] = 10.0
    subscriber.spin_once()
    clock["value"] = 11.0
    subscriber.spin_once()

    stalled_events = [text for text in statuses if "ETH MAVLINK RX STALLED" in text]
    assert len(stalled_events) == 1
    assert "RTCM/commands blocked" in stalled_events[0]


def test_long_param_reservation_keeps_one_hz_gcs_heartbeat_from_command_owner(
    monkeypatch,
) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    send_lock = _TrackedReentrantLock()

    class ParamValue:
        param_id = "TEST"
        param_value = 3.5
        param_type = 9

        @staticmethod
        def get_type() -> str:
            return "PARAM_VALUE"

    class Mav(_TransportMav):
        def __init__(self) -> None:
            super().__init__()
            self.heartbeat_times: list[float] = []
            self.param_requests: list[tuple[object, ...]] = []

        def heartbeat_send(self, *args: object) -> None:
            assert send_lock.depth == 2
            self.heartbeat_times.append(clock["value"])
            super().heartbeat_send(*args)

        def param_request_read_send(self, *args: object) -> None:
            assert send_lock.depth == 1
            self.param_requests.append(args)

    class Connection(_TransportConnection):
        def __init__(self, mav: Mav) -> None:
            super().__init__(mav)
            self.polls = 0

        def recv_match(self, **kwargs):
            assert send_lock.depth == 1
            clock["value"] += float(kwargs["timeout"])
            self.polls += 1
            if clock["value"] < 28.0:
                return _TransportMessage(
                    "SYS_STATUS",
                    system_id=1,
                    raw=f"telemetry-{self.polls}".encode(),
                )
            return ParamValue()

    mav = Mav()
    connection = Connection(mav)
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udp:fake",
        right_url="",
    )
    subscriber.connections["left"] = connection
    subscriber._mavlink_send_locks["left"] = send_lock

    result = subscriber.read_param("left", "TEST", timeout_s=30.0)

    assert result == pytest.approx(3.5)
    assert mav.param_requests == [(1, 1, b"TEST", -1)]
    assert send_lock.depth == 0
    assert send_lock.max_depth == 2
    assert clock["value"] >= 28.0
    assert len(mav.heartbeat_times) >= 27
    assert mav.heartbeat_times[0] == 0.0
    assert max(
        later - earlier
        for earlier, later in zip(mav.heartbeat_times, mav.heartbeat_times[1:])
    ) <= 1.05
    assert subscriber.mavlink_health_snapshot(
        "left", now_s=clock["value"]
    ).last_gcs_heartbeat_tx_monotonic_s == pytest.approx(mav.heartbeat_times[-1])


def test_shell_reservation_services_same_owner_heartbeat_and_releases_shell(
    monkeypatch,
) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    send_lock = _TrackedReentrantLock()

    class Mav(_TransportMav):
        def __init__(self) -> None:
            super().__init__()
            self.heartbeat_times: list[float] = []
            self.serial_calls: list[tuple[object, ...]] = []

        def heartbeat_send(self, *args: object) -> None:
            assert send_lock.depth == 2
            self.heartbeat_times.append(clock["value"])
            super().heartbeat_send(*args)

        def serial_control_send(self, *args: object) -> None:
            assert send_lock.depth == 1
            self.serial_calls.append(args)

    class Connection(_TransportConnection):
        def recv_match(self, **kwargs):
            assert send_lock.depth == 1
            clock["value"] += float(kwargs["timeout"])
            return None

    mav = Mav()
    connection = Connection(mav)
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udp:fake",
        right_url="",
    )
    subscriber.connections["left"] = connection
    subscriber._mavlink_send_locks["left"] = send_lock

    with subscriber.exclusive_command("left"):
        result = subscriber.send_shell_command_with_result(
            "left",
            "mavlink status",
            timeout_s=2.2,
        )

    assert result.command == "mavlink status"
    assert result.flag_label == "respond"
    assert send_lock.depth == 0
    assert send_lock.max_depth == 2
    assert len(mav.heartbeat_times) >= 3
    assert max(
        later - earlier
        for earlier, later in zip(mav.heartbeat_times, mav.heartbeat_times[1:])
    ) <= 1.05
    assert mav.serial_calls[-1][1] == 0


def test_gui_heartbeat_path_never_waits_for_busy_command_send_lock(monkeypatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 1.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])

    class BusyLock:
        def __init__(self) -> None:
            self.blocking_modes: list[bool] = []

        def acquire(self, blocking: bool = True) -> bool:
            self.blocking_modes.append(bool(blocking))
            return False

        def release(self) -> None:
            raise AssertionError("an unacquired busy lock must not be released")

    busy_lock = BusyLock()
    connection = _TransportConnection()
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udp:fake",
        right_url="",
    )
    subscriber.connections["left"] = connection
    subscriber._mavlink_send_locks["left"] = busy_lock

    subscriber.spin_once()

    assert busy_lock.blocking_modes == [False]
    assert connection.mav.heartbeats == []


def test_side_command_keeps_peer_receive_live_and_excludes_same_side_rtcm() -> None:
    from Workstation.workstation_core import pixhawk_attitude

    class LeftMav(_TransportMav):
        def __init__(self) -> None:
            super().__init__()
            self.rtcm_packets: list[tuple[object, ...]] = []

        def gps_rtcm_data_send(self, *args: object) -> None:
            self.rtcm_packets.append(args)

    left_mav = LeftMav()
    left = _TransportConnection(left_mav)
    right = _TransportConnection()
    right.messages.append(
        _TransportMessage("ATTITUDE", system_id=2, raw=b"healthy-right-attitude")
    )
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udp:left",
        right_url="udp:right",
    )
    subscriber.connections.update(left=left, right=right)
    right_attitudes: list[object] = []
    subscriber.extrinsic_changed.connect(
        lambda side, state: right_attitudes.append(state) if side == "right" else None
    )
    command_entered = threading.Event()
    release_command = threading.Event()

    def hold_command() -> None:
        with subscriber.exclusive_command("left"):
            command_entered.set()
            assert release_command.wait(2.0)

    command_thread = threading.Thread(target=hold_command)
    command_thread.start()
    assert command_entered.wait(1.0)

    subscriber.spin_once()
    assert len(right_attitudes) == 1

    rtcm_started = threading.Event()
    rtcm_done = threading.Event()

    def send_rtcm() -> None:
        rtcm_started.set()
        subscriber.send_rtcm_data("left", b"rtcm")
        rtcm_done.set()

    rtcm_thread = threading.Thread(target=send_rtcm)
    rtcm_thread.start()
    assert rtcm_started.wait(1.0)
    assert not rtcm_done.wait(0.05)
    assert left_mav.rtcm_packets == []

    release_command.set()
    command_thread.join(timeout=1.0)
    rtcm_thread.join(timeout=1.0)

    assert not command_thread.is_alive()
    assert not rtcm_thread.is_alive()
    assert rtcm_done.is_set()
    assert len(left_mav.rtcm_packets) == 1


def test_deferred_heartbeat_attitude_burst_replays_without_recovery_commands(
    monkeypatch,
) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 100.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    connection = _TransportConnection()
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udp:test",
        right_url="",
    )
    subscriber.connections["left"] = connection
    attitudes: list[object] = []
    subscriber.extrinsic_changed.connect(lambda _side, state: attitudes.append(state))

    def attitude(index: int, *, raw: bytes) -> _TransportMessage:
        message = _TransportMessage("ATTITUDE", system_id=1, raw=raw)
        message.time_boot_ms = 100_000 + index * 1_000
        return message

    def deliver_live(message: _TransportMessage, at_s: float) -> None:
        clock["value"] = at_s
        assert subscriber._observe_mavlink_transport_message(
            "left",
            message,
            received_at_s=at_s,
        )
        subscriber._handle_mavlink_message(
            "left",
            message,
            received_at_s=at_s,
        )

    deliver_live(
        _TransportMessage("HEARTBEAT", system_id=1, raw=b"initial-heartbeat"),
        100.0,
    )
    deliver_live(attitude(0, raw=b"initial-attitude"), 100.1)
    assert len(connection.mav.commands) == len(pixhawk_attitude.MAVLINK_STOCK_STREAM_REQUESTS)
    connection.mav.commands.clear()

    with subscriber.exclusive_command("left"):
        for index in range(1, 29):
            clock["value"] = 100.0 + index
            subscriber._defer_command_message(
                "left",
                _TransportMessage(
                    "HEARTBEAT",
                    system_id=1,
                    raw=f"deferred-heartbeat-{index}".encode(),
                ),
            )
            clock["value"] += 0.1
            subscriber._defer_command_message(
                "left",
                attitude(index, raw=f"deferred-attitude-{index}".encode()),
            )

    before_replay = subscriber.mavlink_health_snapshot("left", now_s=128.1)
    assert before_replay.last_any_rx_monotonic_s == pytest.approx(128.1)
    assert before_replay.last_heartbeat_rx_monotonic_s == pytest.approx(128.0)
    assert before_replay.last_attitude_rx_monotonic_s == pytest.approx(100.1)

    clock["value"] = 130.5
    while subscriber._deferred_command_messages["left"]:
        subscriber.spin_once()

    assert connection.mav.commands == []
    assert [state.timestamp_s for state in attitudes[-28:]] == pytest.approx(
        [100.1 + index for index in range(1, 29)]
    )
    after_replay = subscriber.mavlink_health_snapshot("left", now_s=130.5)
    assert after_replay.last_any_rx_monotonic_s == pytest.approx(128.1)
    assert after_replay.last_heartbeat_rx_monotonic_s == pytest.approx(128.0)
    assert after_replay.last_attitude_rx_monotonic_s == pytest.approx(128.1)

    deliver_live(
        _TransportMessage("HEARTBEAT", system_id=1, raw=b"new-live-heartbeat"),
        131.0,
    )

    assert len(connection.mav.commands) == 1
    assert connection.mav.commands[0][4] == pixhawk_attitude.MAVLINK_MESSAGE_IDS["ATTITUDE"]


def test_deferred_first_heartbeat_waits_for_live_profile_and_setup_owner(
    monkeypatch,
) -> None:
    from Workstation.workstation_core import pixhawk_attitude

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    connection = _TransportConnection()
    subscriber = pixhawk_attitude.DualPixhawkMavlinkSubscriber(
        left_url="udp:test",
        right_url="",
    )
    subscriber.connections["left"] = connection
    subscriber._mavlink_stream_config = {"pending": True}
    setup_sides: list[str] = []
    subscriber.stream_setup_required.connect(setup_sides.append)

    with subscriber.exclusive_command("left"):
        clock["value"] = 1.0
        subscriber._defer_command_message(
            "left",
            _TransportMessage("HEARTBEAT", system_id=1, raw=b"deferred-first-heartbeat"),
        )

    clock["value"] = 2.0
    subscriber.spin_once()

    assert setup_sides == []
    assert connection.mav.commands == []

    live = _TransportMessage("HEARTBEAT", system_id=1, raw=b"live-first-heartbeat")
    clock["value"] = 2.1
    assert subscriber._observe_mavlink_transport_message(
        "left", live, received_at_s=clock["value"]
    )
    subscriber._handle_mavlink_message(
        "left", live, received_at_s=clock["value"]
    )

    assert setup_sides == ["left"]
    assert len(connection.mav.commands) == len(pixhawk_attitude.MAVLINK_STOCK_STREAM_REQUESTS)

    repeated = _TransportMessage("HEARTBEAT", system_id=1, raw=b"live-second-heartbeat")
    clock["value"] = 2.2
    assert subscriber._observe_mavlink_transport_message(
        "left", repeated, received_at_s=clock["value"]
    )
    subscriber._handle_mavlink_message(
        "left", repeated, received_at_s=clock["value"]
    )

    assert setup_sides == ["left"]
    assert len(connection.mav.commands) == len(pixhawk_attitude.MAVLINK_STOCK_STREAM_REQUESTS)
