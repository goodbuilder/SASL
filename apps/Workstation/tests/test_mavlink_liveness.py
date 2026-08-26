"""Independent threshold tests for per-side Ethernet MAVLink liveness."""

from __future__ import annotations

import pytest

from Workstation.workstation_core.mavlink_liveness import (
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
        component_id=1,
        message_type="HEARTBEAT",
        received_at_s=1.2,
        generation=generation,
    )
    verified = model.snapshot(now_s=1.2)
    assert verified.state is MavlinkLivenessState.QUALIFYING
    assert verified.identity_verified
    assert verified.observed_system_id == 1


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


def test_operational_connection_declares_gcs_source_identity(monkeypatch) -> None:
    from pymavlink import mavutil

    from Workstation.workstation_core import pixhawk_attitude

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
    )

    subscriber.start("left", start_timer=False)

    assert captured["source_system"] == 255
    assert captured["source_component"] == 190
    assert subscriber.mavlink_health_snapshot("left", now_s=0.0).listener_open
    subscriber.stop("left")
    assert connection.closed
    assert subscriber.mavlink_health_snapshot("left", now_s=0.0).state is MavlinkLivenessState.DISCONNECTED


def test_gcs_heartbeat_starts_after_expected_peer_and_never_catches_up(monkeypatch) -> None:
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

    connection.messages.append(_TransportMessage("SYS_STATUS", system_id=1))
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
    connection.messages.append(_TransportMessage("SYS_STATUS", system_id=1))

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
