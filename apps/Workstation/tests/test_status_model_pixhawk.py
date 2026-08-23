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


def test_pixhawk_status_tracks_exclusive_command_lifecycle_per_side() -> None:
    from Workstation.workstation_core.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    subscriber.connections = {"left": object(), "right": object()}
    panel = SimpleNamespace(
        _mavlink_open_sides=lambda: tuple(subscriber.connections),
        pixhawk_attitude_times={"left": [90.0], "right": [90.0]},
        mavlink_pixhawk=subscriber,
        latest_extrinsics={},
    )
    adapter = BackendStatusAdapter(SimpleNamespace(panel=panel))

    with subscriber.exclusive_command("left"):
        adapter._refresh_pixhawk(panel, "left", "P1", now_s=100.0)
        adapter._refresh_pixhawk(panel, "right", "P2", now_s=100.0)

        assert adapter.model.record("pixhawk.left").state == "PARAM SYNC"
        right = adapter.model.record("pixhawk.right")
        assert right is not None
        assert right.state == "STALE"
        assert right.severity == StatusSeverity.WARN

    panel.pixhawk_attitude_times["left"] = [100.1]
    adapter._refresh_pixhawk(panel, "left", "P1", now_s=100.2)

    recovered = adapter.model.record("pixhawk.left")
    assert recovered is not None
    assert recovered.state == "LIVE"
    assert recovered.severity == StatusSeverity.OK
