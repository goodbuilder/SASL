"""Focused Pixhawk status projection regressions."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from Workstation.status_model import BackendStatusAdapter, StatusSeverity


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

    clock["value"] = 100.0
    with subscriber.exclusive_command("left"):
        adapter._refresh_pixhawk(panel, "left", "P1", now_s=100.0)
        adapter._refresh_pixhawk(panel, "right", "P2", now_s=100.0)

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
    assert "00:00 / 10:00" in recovered.detail


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
    assert "qualification=00:00 / 10:00" in qualifying.detail

    for second in range(1, 601):
        observe("HEARTBEAT", float(second))
        observe("ATTITUDE", float(second))
    adapter._refresh_pixhawk(panel, "left", "P1", now_s=600.0)
    live = adapter.model.record("pixhawk.left")
    assert live.state == "LIVE"
    assert live.severity == StatusSeverity.OK

    observe("HEARTBEAT", 602.001)
    observe("SYS_STATUS", 602.001)
    adapter._refresh_pixhawk(panel, "left", "P1", now_s=602.001)
    attitude_stale = adapter.model.record("pixhawk.left")
    assert attitude_stale.state == "ATTITUDE STALE"
    assert "bounded ATTITUDE request only" in attitude_stale.detail

    adapter._refresh_pixhawk(panel, "left", "P1", now_s=612.001)
    stalled = adapter.model.record("pixhawk.left")
    assert stalled.state == "ETH MAVLINK RX STALLED"
    assert stalled.severity == StatusSeverity.BAD
    assert "RTCM blocked" in stalled.detail
    assert "deadlock suspected" in stalled.detail

    observe("HEARTBEAT", 612.1)
    observe("ATTITUDE", 612.1)
    adapter._refresh_pixhawk(panel, "left", "P1", now_s=612.1)
    recovering = adapter.model.record("pixhawk.left")
    assert recovering.state == "RECOVERING"
    assert "qualification=00:00 / 10:00" in recovering.detail
