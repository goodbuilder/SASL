from __future__ import annotations

from types import SimpleNamespace

import pytest

from wiris_stereo_distance import xrce_agent


def test_micro_xrce_agent_command_uses_udp_port() -> None:
    assert xrce_agent.micro_xrce_agent_command(8888) == ["MicroXRCEAgent", "udp4", "-p", "8888"]


def test_micro_xrce_agent_command_rejects_invalid_port() -> None:
    with pytest.raises(ValueError):
        xrce_agent.micro_xrce_agent_command(70000)


def test_is_micro_xrce_agent_running_checks_port(monkeypatch) -> None:
    calls: list[list[str]] = []

    def fake_run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(xrce_agent.subprocess, "run", fake_run)

    assert xrce_agent.is_micro_xrce_agent_running(8888) is True
    assert calls == [["pgrep", "-f", r"MicroXRCEAgent.*udp4.*-p[ ]+8888"]]


def test_ensure_micro_xrce_agent_skips_when_running(monkeypatch, tmp_path) -> None:
    monkeypatch.setattr(xrce_agent, "is_micro_xrce_agent_running", lambda port: True)

    started, message = xrce_agent.ensure_micro_xrce_agent(8888, log_path=tmp_path / "agent.log")

    assert started is False
    assert message == "already running on UDP 8888"


def test_ensure_micro_xrce_agent_starts_process(monkeypatch, tmp_path) -> None:
    calls = []

    class FakeProcess:
        pid = 12345

    def fake_popen(command, **kwargs):
        calls.append((command, kwargs))
        return FakeProcess()

    monkeypatch.setattr(xrce_agent, "is_micro_xrce_agent_running", lambda port: False)
    monkeypatch.setattr(xrce_agent.subprocess, "Popen", fake_popen)

    started, message = xrce_agent.ensure_micro_xrce_agent(8888, log_path=tmp_path / "agent.log")

    assert started is True
    assert "started on UDP 8888" in message
    assert "pid 12345" in message
    assert calls[0][0] == ["MicroXRCEAgent", "udp4", "-p", "8888"]
    assert calls[0][1]["start_new_session"] is True
