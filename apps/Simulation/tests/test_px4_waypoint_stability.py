from __future__ import annotations

import importlib.util
import json
import sys
from types import SimpleNamespace
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


def _load_module():
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location(
        "px4_waypoint_mission_under_test", SCRIPTS / "px4_waypoint_mission.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_nearest_rank_p95_is_deterministic() -> None:
    module = _load_module()
    assert module._percentile_nearest_rank([0.2, 0.4, 0.1, 0.3], 0.95) == 0.4
    assert module._percentile_nearest_rank([], 0.95) != module._percentile_nearest_rank([], 0.95)


def test_event_log_is_create_then_append(tmp_path: Path) -> None:
    module = _load_module()
    path = tmp_path / "events.jsonl"
    module._append_event(path, {"event": "mission_started"}, create=True)
    module._append_event(path, {"event": "mission_completed"})
    events = [json.loads(line) for line in path.read_text().splitlines()]
    assert events == [{"event": "mission_started"}, {"event": "mission_completed"}]


def test_guidance_type_masks_match_active_setpoint_fields() -> None:
    module = _load_module()
    assert module._position_velocity_type_mask() == 3520
    assert module._position_only_type_mask() == 3576
    assert module._velocity_only_type_mask() == 3527
    assert module._waypoint_phase(7, "hold_analysis") == "waypoint_007_hold_analysis"


def test_hold_publishes_analysis_phase_and_records_position_only_trace(
    tmp_path: Path, monkeypatch
) -> None:
    module = _load_module()

    class Clock:
        value = 0.0

        def monotonic(self):
            return self.value

        def time(self):
            return 1000.0 + self.value

        def sleep(self, seconds):
            self.value += float(seconds)

    class Master:
        def recv_match(self, *, type, blocking=False):
            assert blocking is False
            if type == "LOCAL_POSITION_NED":
                return SimpleNamespace(x=1.0, y=2.0, z=-3.0, vx=0.0, vy=0.0, vz=0.0)
            if type == "ATTITUDE":
                return SimpleNamespace(roll=0.0, pitch=0.0)
            raise AssertionError(type)

    class PhaseMonitor:
        def __init__(self):
            self.phases = []

        def publish_phase(self, phase):
            self.phases.append(phase)

    clock = Clock()
    monkeypatch.setattr(module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(module.time, "time", clock.time)
    monkeypatch.setattr(module.time, "sleep", clock.sleep)
    monkeypatch.setattr(module, "send_position_setpoint", lambda *args: None)
    event_path = tmp_path / "events.jsonl"
    event_path.write_text("", encoding="utf-8")
    trace = module.GuidanceTrace(tmp_path / "guidance.jsonl")
    phase_monitor = PhaseMonitor()
    args = SimpleNamespace(
        guidance_mode="position_velocity_final_target",
        setpoint_hz=2.0,
        stability_window_s=1.0,
        stability_position_p95_max_m=None,
        stability_attitude_max_deg=None,
        event_log=event_path,
    )

    metrics = module.hold_waypoint(
        Master(),
        0.0,
        args,
        (1.0, 2.0, -3.0, None),
        2.0,
        4,
        8,
        phase_monitor=phase_monitor,
        guidance_trace=trace,
    )
    trace.close()

    assert metrics["stability_pass"] is True
    assert "waypoint_004_hold_settle" in phase_monitor.phases
    assert "waypoint_004_hold_analysis" in phase_monitor.phases
    events = [json.loads(line) for line in event_path.read_text().splitlines()]
    assert [event["event"] for event in events] == [
        "waypoint_hold_started",
        "waypoint_hold_analysis_started",
        "waypoint_hold_complete",
    ]
    trace_rows = [
        json.loads(line)
        for line in (tmp_path / "guidance.jsonl").read_text().splitlines()
    ]
    assert {row["mission_phase"] for row in trace_rows} == {
        "waypoint_004_hold_settle",
        "waypoint_004_hold_analysis",
    }
    assert {row["type_mask"] for row in trace_rows} == {3576}
