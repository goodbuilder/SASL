from __future__ import annotations

from decimal import Decimal
import importlib.util
from pathlib import Path
import signal
import subprocess
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_sweep import (
    CameraContract,
    build_sweep_plan,
    write_plan,
)


def _cli_module():
    path = Path(__file__).resolve().parents[1] / "scripts" / "run_correction_search_sweep.py"
    name = "s2_r3_cli_failure_test"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class FakeProcess:
    pid = 77

    def __init__(self, waits):
        self.waits = list(waits)
        self.events = []
        self.return_code = None
        self.closed = False

    def poll(self):
        return self.return_code

    def signal_owned(self, signal_number):
        self.events.append({"signal": signal_number.name})

    def wait(self, timeout):
        result = self.waits.pop(0)
        if result == "timeout":
            raise subprocess.TimeoutExpired("fake", timeout)
        self.return_code = result
        return result

    def close_log(self):
        self.closed = True

    def process_group_exists(self):
        return False


def _plan():
    return build_sweep_plan(
        sweep_id="s",
        commit_sha="a" * 40,
        branch="sim/test",
        repository_name="repo",
        attitude_mode="ofat",
        axis_ranges=(),
        range_values=(Decimal("500"),),
        bearing_values=(Decimal("0"),),
        elevation_values=(Decimal("0"),),
        repetitions=1,
        seed=0,
        case_order="canonical",
        invalid_spatial_policy="fail",
        max_cases=2,
        contract=CameraContract(),
    )


def test_owned_process_stops_with_sigint_only() -> None:
    module = _cli_module()
    process = FakeProcess((0,))
    events = module.stop_owned_process(process, shutdown_timeout_s=1)
    assert [event["signal"] for event in events] == [signal.SIGINT.name]


def test_owned_process_delivers_one_graceful_interrupt_then_group_escalation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _cli_module()
    delivered = []
    monkeypatch.setattr(module.os, "kill", lambda pid, value: delivered.append(("pid", pid, value)))
    monkeypatch.setattr(
        module.os, "killpg", lambda pid, value: delivered.append(("group", pid, value))
    )

    class PopenLike:
        pid = 91

    owned = module.OwnedProcess(PopenLike(), None)
    owned.signal_owned(signal.SIGINT)
    owned.signal_owned(signal.SIGTERM)
    assert delivered == [
        ("pid", 91, signal.SIGINT),
        ("group", 91, signal.SIGTERM),
    ]


def test_owned_process_escalates_int_term_kill_without_unrelated_pid() -> None:
    module = _cli_module()
    process = FakeProcess(("timeout", "timeout", 0))
    events = module.stop_owned_process(process, shutdown_timeout_s=1)
    assert [event["signal"] for event in events] == [
        signal.SIGINT.name,
        signal.SIGTERM.name,
        signal.SIGKILL.name,
    ]
    assert process.pid == 77


def test_owned_process_cleans_residual_gazebo_group_after_launch_exit() -> None:
    module = _cli_module()
    process = FakeProcess((0,))
    group_states = iter((True, False, False))
    process.process_group_exists = lambda: next(group_states)
    events = module.stop_owned_process(process, shutdown_timeout_s=1)
    assert [event["signal"] for event in events] == ["SIGINT", "SIGTERM"]


def test_failure_policy_stop_and_continue_are_explicit() -> None:
    module = _cli_module()
    assert module.should_launch_next(case_valid=True, failure_policy="stop") is True
    assert module.should_launch_next(case_valid=False, failure_policy="stop") is False
    assert module.should_launch_next(case_valid=False, failure_policy="continue") is True


def test_child_early_exit_preserves_attempt_failure_detail(tmp_path: Path) -> None:
    module = _cli_module()
    _plan_value, cases = _plan()
    process = FakeProcess(())
    process.return_code = 3

    class Launcher:
        def launch(self, *args, **kwargs):
            return process

    state = module.execute_case_attempt(
        case=cases[0],
        output_dir=tmp_path,
        repository_root=tmp_path,
        provenance={
            "repository_name": "repo",
            "branch": "sim/test",
            "commit_sha": "a" * 40,
        },
        sweep_id="s",
        gazebo_version="8",
        warmup_count=0,
        analysis_count=1,
        case_timeout_s=1,
        shutdown_timeout_s=1,
        launcher=Launcher(),
    )
    assert state["case_valid"] is False
    assert "exited early" in state["failure_reason"]
    assert state["failure_traceback"]
    assert process.closed is True
    assert (tmp_path / "cases" / cases[0]["case_id"] / "attempt-0001" / "attempt_state.json").is_file()


def test_timeout_attempt_escalates_and_remains_invalid(tmp_path: Path) -> None:
    module = _cli_module()
    _plan_value, cases = _plan()
    process = FakeProcess(("timeout", "timeout", 0))

    class Launcher:
        def launch(self, *args, **kwargs):
            return process

    moments = iter((0.0, 2.0))
    state = module.execute_case_attempt(
        case=cases[0],
        output_dir=tmp_path,
        repository_root=tmp_path,
        provenance={"repository_name": "repo", "branch": "sim/test", "commit_sha": "a" * 40},
        sweep_id="s",
        gazebo_version="8",
        warmup_count=0,
        analysis_count=1,
        case_timeout_s=1,
        shutdown_timeout_s=1,
        launcher=Launcher(),
        monotonic=lambda: next(moments),
        sleeper=lambda _seconds: None,
    )
    assert state["case_valid"] is False
    assert "timeout" in state["failure_reason"].lower()
    assert [event["signal"] for event in state["signal_escalation"]] == [
        "SIGINT",
        "SIGTERM",
        "SIGKILL",
    ]


def test_missing_case_row_finalizes_sweep_invalid(tmp_path: Path) -> None:
    module = _cli_module()
    plan, cases = _plan()
    write_plan(tmp_path, plan, cases)
    (tmp_path / "SWEEP_INCOMPLETE").touch()
    summary = module.finalize_sweep(
        output_dir=tmp_path,
        plan=plan,
        cases=cases,
        rows=[],
        counters={"launch_count": 0},
    )
    assert summary["run_valid"] is False
    assert (tmp_path / "SWEEP_FINALIZED_INVALID").is_file()
    assert not (tmp_path / "SWEEP_FINALIZED_VALID").exists()
