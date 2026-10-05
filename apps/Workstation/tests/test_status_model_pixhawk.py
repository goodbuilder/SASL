"""Focused Pixhawk status projection regressions."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from Workstation.status_model import BackendStatusAdapter, StatusSeverity
from Workstation.workstation_core.mavlink_liveness import (
    MAVLINK_ALL_RX_STALL_S,
    MAVLINK_INITIAL_RX_TIMEOUT_S,
)


@pytest.mark.parametrize(
    ("mavlink_sides", "attitude_times", "idle_state"),
    (
        (("left",), {"left": [90.0]}, "STALE"),
        ((), {"left": []}, "DISCONNECTED"),
    ),
)
def test_active_pixhawk_command_overrides_stale_or_disconnected_status(
    mavlink_sides: tuple[str, ...],
    attitude_times: dict[str, list[float]],
    idle_state: str,
) -> None:
    subscriber = SimpleNamespace(_active_command_cameras=set())
    panel = SimpleNamespace(
        _mavlink_open_sides=lambda: mavlink_sides,
        pixhawk_attitude_times=attitude_times,
        mavlink_pixhawk=subscriber,
        latest_extrinsics={},
    )
    adapter = BackendStatusAdapter(SimpleNamespace(panel=panel))

    adapter._refresh_pixhawk(panel, "left", "P1", now_s=100.0)
    assert adapter.model.record("pixhawk.left").state == idle_state

    subscriber._active_command_cameras.add("left")
    adapter._refresh_pixhawk(panel, "left", "P1", now_s=100.0)

    status = adapter.model.record("pixhawk.left")
    assert status is not None
    assert status.state == "PARAM SYNC"
    assert status.severity == StatusSeverity.OK
    assert status.state not in {"STALE", "DISCONNECTED"}
    assert status.detail == "PX4 파라미터 조회 중 (정상)"


def test_shell_poll_status_is_not_presented_as_parameter_sync() -> None:
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    subscriber.connections = {"left": object()}
    model = subscriber._ensure_liveness_listener("left")
    generation = model.connection_generation
    for message_type in ("HEARTBEAT", "ATTITUDE"):
        assert model.observe_packet(
            system_id=1,
            component_id=1,
            message_type=message_type,
            received_at_s=10.0,
            generation=generation,
        )
    panel = SimpleNamespace(
        _mavlink_open_sides=lambda: ("left",),
        pixhawk_attitude_times={"left": [10.0]},
        mavlink_pixhawk=subscriber,
        latest_extrinsics={},
    )
    adapter = BackendStatusAdapter(SimpleNamespace(panel=panel))

    with subscriber.exclusive_command("left", command_label="RM3100 SHELL POLL"):
        adapter._refresh_pixhawk(panel, "left", "P1", now_s=10.1)

    status = adapter.model.record("pixhawk.left")
    assert status is not None
    assert status.state == "SHELL SYNC"
    assert status.detail == "PX4 Shell sensor_mag 조회 중 (정상)"


def test_pixhawk_status_tracks_exclusive_command_lifecycle_per_side(monkeypatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    clock = {"value": 90.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    subscriber.connections = {"left": object(), "right": object()}
    for side, system_id in (("left", 1), ("right", 2)):
        model = subscriber._ensure_liveness_listener(side)
        generation = model.connection_generation
        for message_type in ("HEARTBEAT", "ATTITUDE"):
            assert model.observe_packet(
                system_id=system_id,
                component_id=1,
                message_type=message_type,
                received_at_s=90.0,
                generation=generation,
            )
    panel = SimpleNamespace(
        _mavlink_open_sides=lambda: tuple(subscriber.connections),
        pixhawk_attitude_times={"left": [90.0], "right": [90.0]},
        mavlink_pixhawk=subscriber,
        latest_extrinsics={},
    )
    adapter = BackendStatusAdapter(SimpleNamespace(panel=panel))

    stall_now_s = 90.0 + MAVLINK_ALL_RX_STALL_S
    clock["value"] = stall_now_s
    with subscriber.exclusive_command("left"):
        adapter._refresh_pixhawk(panel, "left", "P1", now_s=stall_now_s)
        adapter._refresh_pixhawk(panel, "right", "P2", now_s=stall_now_s)

        assert adapter.model.record("pixhawk.left").state == "PARAM SYNC"
        right = adapter.model.record("pixhawk.right")
        assert right is not None
        assert right.state == "ETH MAVLINK RX STALLED"
        assert right.severity == StatusSeverity.BAD

    clock["value"] = 100.1
    left_model = subscriber._mavlink_liveness["left"]
    for message_type in ("HEARTBEAT", "ATTITUDE"):
        assert left_model.observe_packet(
            system_id=1,
            component_id=1,
            message_type=message_type,
            received_at_s=100.1,
            generation=left_model.connection_generation,
        )
    adapter._refresh_pixhawk(panel, "left", "P1", now_s=100.2)

    recovered = adapter.model.record("pixhawk.left")
    assert recovered is not None
    assert recovered.state == "QUALIFYING"
    assert recovered.severity == StatusSeverity.WARN
    assert "00:00 / 00:10" in recovered.detail


def test_pixhawk_liveness_projection_distinguishes_qualification_attitude_stall_and_recovery(monkeypatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    subscriber.connections["left"] = object()
    model = subscriber._ensure_liveness_listener("left")
    generation = model.connection_generation

    def observe(message_type: str, at_s: float) -> None:
        assert model.observe_packet(
            system_id=1,
            component_id=1,
            message_type=message_type,
            received_at_s=at_s,
            generation=generation,
        )

    observe("HEARTBEAT", 0.0)
    observe("ATTITUDE", 0.0)
    panel = SimpleNamespace(
        _mavlink_open_sides=lambda: ("left",),
        pixhawk_attitude_times={"left": [0.0]},
        mavlink_pixhawk=subscriber,
        latest_extrinsics={},
    )
    adapter = BackendStatusAdapter(SimpleNamespace(panel=panel))

    adapter._refresh_pixhawk(panel, "left", "P1", now_s=0.0)
    qualifying = adapter.model.record("pixhawk.left")
    assert qualifying.state == "QUALIFYING"
    assert "qualification=00:00 / 00:10" in qualifying.detail

    for second in range(1, 61):
        observe("HEARTBEAT", float(second))
        observe("ATTITUDE", float(second))
    adapter._refresh_pixhawk(panel, "left", "P1", now_s=60.0)
    live = adapter.model.record("pixhawk.left")
    assert live.state == "LIVE"
    assert live.severity == StatusSeverity.OK

    observe("HEARTBEAT", 62.001)
    observe("SYS_STATUS", 62.001)
    adapter._refresh_pixhawk(panel, "left", "P1", now_s=62.001)
    attitude_stale = adapter.model.record("pixhawk.left")
    assert attitude_stale.state == "ATTITUDE STALE"
    assert "bounded ATTITUDE request only" in attitude_stale.detail

    stall_now_s = 62.001 + MAVLINK_ALL_RX_STALL_S
    adapter._refresh_pixhawk(panel, "left", "P1", now_s=stall_now_s)
    stalled = adapter.model.record("pixhawk.left")
    assert stalled.state == "ETH MAVLINK RX STALLED"
    assert stalled.severity == StatusSeverity.BAD
    assert "RTCM blocked" in stalled.detail
    assert "deadlock suspected" in stalled.detail

    recovery_now_s = stall_now_s + 0.1
    observe("HEARTBEAT", recovery_now_s)
    observe("ATTITUDE", recovery_now_s)
    adapter._refresh_pixhawk(panel, "left", "P1", now_s=recovery_now_s)
    recovering = adapter.model.record("pixhawk.left")
    assert recovering.state == "RECOVERING"
    assert "qualification=00:00 / 00:10" in recovering.detail


def test_pixhawk_status_projects_initial_heartbeat_timeout_with_peer_and_reason(monkeypatch) -> None:
    from Workstation.workstation_core import pixhawk_attitude
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    clock = {"value": 0.0}
    monkeypatch.setattr(pixhawk_attitude.time, "monotonic", lambda: clock["value"])
    subscriber = DualPixhawkMavlinkSubscriber(
        left_url="udpin:0.0.0.0:14550",
        right_url="",
        peer_hosts={"left": "10.0.0.2"},
    )
    subscriber.connections["left"] = object()
    subscriber._ensure_liveness_listener("left")
    panel = SimpleNamespace(
        _mavlink_open_sides=lambda: ("left",),
        pixhawk_attitude_times={"left": []},
        mavlink_pixhawk=subscriber,
        latest_extrinsics={},
    )
    adapter = BackendStatusAdapter(SimpleNamespace(panel=panel))

    clock["value"] = MAVLINK_INITIAL_RX_TIMEOUT_S
    adapter._refresh_pixhawk(
        panel,
        "left",
        "P1",
        now_s=MAVLINK_INITIAL_RX_TIMEOUT_S,
    )

    status = adapter.model.record("pixhawk.left")
    assert status is not None
    assert status.state == "MAVLINK INITIAL RX TIMEOUT"
    assert status.severity == StatusSeverity.BAD
    assert "peer=10.0.0.2" in status.detail
    assert f"listener={MAVLINK_INITIAL_RX_TIMEOUT_S:.1f}s" in status.detail
    assert "expected-sysid HEARTBEAT not received" in status.detail
