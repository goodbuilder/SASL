"""Entry lead and heading error boundaries without starting SITL."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.flight_controller import (
    _angle_error_deg,
    _heading_error_deg,
    _quaternion_yaw_deg_xyzw,
    bounded_approach_target,
)
from campaigns.stereo_v1_1.trajectory_contract import LocalNed


def test_entry_target_limits_position_lead_in_local_ned():
    current = LocalNed(north_m=0.0, east_m=1.0, down_m=-1.0)
    distant = LocalNed(north_m=-30.0, east_m=5.0, down_m=-141.0)
    command = bounded_approach_target(current, distant, max_lead_m=5.0)
    assert math.dist(command.as_xyz(), current.as_xyz()) == pytest.approx(5.0)
    assert command.north_m < 0.0
    assert command.east_m > 1.0
    assert command.down_m < -1.0
    near = LocalNed(north_m=-1.0, east_m=1.1, down_m=-2.0)
    assert bounded_approach_target(current, near) == near


def test_heading_error_wraps_north_to_east_yaw_at_180_degrees():
    assert _heading_error_deg(math.radians(-179.0), math.radians(179.0)) == pytest.approx(2.0)
    assert _heading_error_deg(math.pi, math.pi) == pytest.approx(0.0)


def test_spawn_yaw_check_reads_world_yaw_from_xyzw_quaternion():
    yaw_minus_90 = [0.0, 0.0, math.sin(math.radians(-45)), math.cos(math.radians(-45))]
    observed = _quaternion_yaw_deg_xyzw(yaw_minus_90)
    assert observed == pytest.approx(-90.0)
    assert _angle_error_deg(observed, 270.0) == pytest.approx(0.0)


def test_attempt_result_persists_missing_clock_cadence_evidence(tmp_path):
    import json
    from campaigns.stereo_v1_1.flight_controller import AttemptLog
    log = AttemptLog(tmp_path, "run", "attempt")
    log.science_send(None)
    log.finish("failed", failure_reason="telemetry stale")
    result = json.loads((tmp_path / "controller_result.json").read_text())
    timing = json.loads((tmp_path / "science_setpoint_timing.jsonl").read_text())
    assert timing["host_monotonic_ns"] > 0
    assert timing["simulation_timestamp_ns"] is None
    assert result["science_setpoint_cadence"]["unknown_simulation_time_count"] == 1
    assert not result["science_setpoint_cadence"]["sufficient_unique_simulation_cadence"]


def _liftoff_snapshot(rise=0.0, world_rise=None, time_ns=0, epoch=0, reset=0, speed=0.0):
    from types import SimpleNamespace
    from campaigns.stereo_v1_1.trajectory_contract import WorldEnu
    return SimpleNamespace(
        px4_local_ned_m=LocalNed(7.0, -3.0, -2.0-rise),
        gt_world_enu_m=WorldEnu(40.0, 415.0, 5.0+(rise if world_rise is None else world_rise)),
        simulation_time_ns=time_ns, simulation_epoch=epoch,
        xy_reset_counter=reset, z_reset_counter=0, speed_mps=speed)


def test_vertical_liftoff_holds_launch_north_east_before_distant_v0_transit():
    from campaigns.stereo_v1_1.entry_liftoff import VerticalLiftoffGate
    gate = VerticalLiftoffGate(_liftoff_snapshot())
    first = gate.target(LocalNed(7, -3, -2), 5)
    assert first == LocalNed(7, -3, -7)
    assert gate.target(LocalNed(8, -2, -10), 5) == LocalNed(7, -3, -12)
    assert not gate.observe(_liftoff_snapshot(rise=1, time_ns=1_000_000_000))


def test_vertical_liftoff_requires_both_local_and_world_height_and_stable_duration():
    from campaigns.stereo_v1_1.entry_liftoff import VerticalLiftoffGate
    gate = VerticalLiftoffGate(_liftoff_snapshot())
    for i in range(11):
        assert not gate.observe(_liftoff_snapshot(rise=10, world_rise=1, time_ns=i*100_000_000))
    for i in range(10):
        assert not gate.observe(_liftoff_snapshot(rise=10, time_ns=2_000_000_000+i*100_000_000))
    assert gate.observe(_liftoff_snapshot(rise=10, time_ns=3_000_000_000))


def test_vertical_liftoff_duplicate_clock_and_observation_gap_cannot_certify_settle():
    from campaigns.stereo_v1_1.entry_liftoff import VerticalLiftoffGate
    gate = VerticalLiftoffGate(_liftoff_snapshot())
    for _ in range(100):
        assert not gate.observe(_liftoff_snapshot(rise=10, time_ns=1_000_000_000))
    assert not gate.observe(_liftoff_snapshot(rise=10, time_ns=2_000_000_000))


@pytest.mark.parametrize('snapshot', [
    _liftoff_snapshot(rise=10, epoch=1),
    _liftoff_snapshot(rise=10, reset=1),
])
def test_vertical_liftoff_rejects_coordinate_generation_changes(snapshot):
    from campaigns.stereo_v1_1.entry_liftoff import VerticalLiftoffGate
    with pytest.raises(RuntimeError, match='reset'):
        VerticalLiftoffGate(_liftoff_snapshot()).observe(snapshot)


def test_vertical_liftoff_speed_and_horizontal_drift_reset_clearance_gate():
    from campaigns.stereo_v1_1.entry_liftoff import VerticalLiftoffGate
    gate = VerticalLiftoffGate(_liftoff_snapshot())
    for i in range(12):
        assert not gate.observe(_liftoff_snapshot(rise=10, time_ns=i*100_000_000, speed=2))
    snapshot = _liftoff_snapshot(rise=10, time_ns=2_000_000_000)
    snapshot.px4_local_ned_m = LocalNed(9, -3, -12)
    assert not gate.observe(snapshot)


def test_vertical_stage_executes_only_vertical_commands_until_gate(monkeypatch):
    from types import SimpleNamespace
    import campaigns.stereo_v1_1.flight_controller as controller
    launch = _liftoff_snapshot()
    samples = iter([_liftoff_snapshot(rise=10, time_ns=i*100_000_000) for i in range(11)])
    sent, events = [], []
    monkeypatch.setattr(controller, '_snapshot', lambda *_: next(samples))
    monkeypatch.setattr(controller, '_check_stop', lambda: None)
    monkeypatch.setattr(controller, '_tick_sleep', lambda tick: tick)
    monkeypatch.setattr(controller, 'send_position_yaw', lambda master, stamp, target, yaw: sent.append(target))
    monkeypatch.setattr(controller, 'now_ms', lambda boot: 0)
    log = SimpleNamespace(event=lambda event, *a, **k: events.append(event), state=lambda *a: None)
    snapshot, target = controller._vertical_liftoff(
        None, None, None, log, launch, None, 0, 0, .02, 5,
        controller.time.monotonic()+60)
    assert snapshot.simulation_time_ns == 1_000_000_000
    assert all(p.north_m == 7 and p.east_m == -3 for p in sent)
    assert events == ['vertical_liftoff_started', 'vertical_liftoff_complete']
    assert target.down_m == -12


def test_vertical_stage_stale_telemetry_aborts_before_horizontal_command(monkeypatch):
    from types import SimpleNamespace
    import itertools
    import campaigns.stereo_v1_1.flight_controller as controller
    ticks = itertools.count(0, .25)
    sent = []
    monkeypatch.setattr(controller.time, 'monotonic', lambda: next(ticks))
    monkeypatch.setattr(controller, '_snapshot', lambda *_: None)
    monkeypatch.setattr(controller, '_check_stop', lambda: None)
    monkeypatch.setattr(controller, '_tick_sleep', lambda tick: tick)
    monkeypatch.setattr(controller, 'send_position_yaw', lambda master, stamp, target, yaw: sent.append(target))
    monkeypatch.setattr(controller, 'now_ms', lambda boot: 0)
    log = SimpleNamespace(event=lambda *a, **k: None, state=lambda *a: None)
    with pytest.raises(RuntimeError, match='telemetry became stale'):
        controller._vertical_liftoff(None, None, None, log, _liftoff_snapshot(),
                                    None, 0, 0, .02, 5, 300)
    assert all(p.north_m == 7 and p.east_m == -3 for p in sent)
