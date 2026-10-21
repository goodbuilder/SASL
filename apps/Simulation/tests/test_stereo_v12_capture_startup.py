"""The real runner must not arm until raw bags and source readiness pass."""
import io
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from campaigns.stereo_v1_1 import campaign_run as runner
from campaigns.stereo_v1_1.workbook_campaign import DEFAULT_WORKBOOK, build_workbook_plan, initialize_workbook_campaign


@pytest.mark.parametrize("development", [True, False])
@pytest.mark.parametrize("ready", [True, False])
def test_v12_startup_barrier_and_failure_cleanup(tmp_path, monkeypatch, ready, development):
    plan = build_workbook_plan(DEFAULT_WORKBOOK)
    initialize_workbook_campaign(tmp_path, DEFAULT_WORKBOOK)
    (tmp_path / "environment_report.json").write_text(json.dumps({
        "status": "PREFLIGHT_PASS", "workbook_sha256": plan["workbook_sha256"]}))
    calls = []
    controller_commands = []
    controller_environments = []
    operator_environments = []
    status_before = (tmp_path / "run_status.json").read_bytes()
    monkeypatch.setattr(runner, "check_owned_model", lambda: None)
    monkeypatch.setattr(runner, "_existing_sitl_processes", lambda: [])

    class Process:
        def __init__(self, name):
            self.name = name
            self.returncode = 0 if name == "flight_controller.py" else None

        def poll(self):
            return self.returncode

    def script(command):
        return next(Path(x).name for x in command if x.endswith((".py", ".sh")))

    def spawn(command, log_path, env=None):
        name = script(command)
        calls.append(name)
        if name == "run_operator_view.sh":
            operator_environments.append(env)
        if name == "flight_controller.py":
            controller_commands.append(command)
            controller_environments.append(env)
        return Process(name), io.StringIO()

    def checked(command, log_path, timeout_s=900):
        calls.append(script(command))
        if not ready:
            raise RuntimeError("GT publication did not reach the declared rate")

    def stop(process, timeout_s):
        if process is None:
            return None
        calls.append("stop:" + process.name)
        return 130 if process.name == "run_operator_view.sh" else 0

    monkeypatch.setattr(runner, "_spawn", spawn)
    monkeypatch.setattr(runner, "_wait_token", lambda process, path, token, timeout: calls.append(token))
    monkeypatch.setattr(runner, "_run_checked", checked)
    monkeypatch.setattr(runner, "_stop_process", stop)
    monkeypatch.setattr(runner, "_postprocess", lambda *args: None)
    monkeypatch.setattr(runner, "validate_attempt", lambda folder, run_id, attempt_id, **kw: {
        "run_id": run_id, "attempt_id": attempt_id, "status": "VALID",
        "infrastructure_errors": [], "protocol_errors": [], "warnings": [],
        "counted_in_campaign": True, "counted_in_39": True})
    result = runner._run_one(tmp_path, plan["planned_runs"][0], development=development)
    if development:
        assert (tmp_path / "run_status.json").read_bytes() == status_before
        assert result["development"] is True
        assert result["counted_in_campaign"] is False
        assert result["counted_in_39"] is False
    assert len(operator_environments) == 1
    console_path = Path(operator_environments[0]["PX4_CONSOLE_LOG"])
    assert console_path.is_relative_to(tmp_path) and console_path.name == "px4_console"
    if controller_commands:
        assert "--timing-diagnostics" in controller_commands[0]
        assert controller_environments[0]["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] == "python"
        assert controller_commands[0][controller_commands[0].index("--world-name")+1] == "stereo_ground_drone"
    assert calls.index("CAPTURE_READY") < calls.index("capture_readiness.py")
    assert "stop:capture_streams.py" in calls
    assert "stop:run_operator_view.sh" in calls
    if ready:
        assert calls.index("capture_readiness.py") < calls.index("state_recorder.py")
        assert calls.index("STATE_READY") < calls.index("flight_controller.py")
        assert result["status"] == "VALID"
    else:
        assert "state_recorder.py" not in calls
        assert "flight_controller.py" not in calls
        assert result["status"] == "INFRASTRUCTURE_FAILED"
