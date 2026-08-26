from pathlib import Path
from types import SimpleNamespace

from Workstation.run_evidence_controller import RunEvidenceController


def _panel(root: Path, session_id: str) -> SimpleNamespace:
    return SimpleNamespace(recording_session_id=session_id, _recording_session_dir=lambda value: root / value)


def test_session_switch_rebinds_identity_without_run_setup(tmp_path: Path) -> None:
    panel = _panel(tmp_path, "A")
    controller = RunEvidenceController(panel)
    first = controller.ensure_workstation_run_evidence("started")
    panel.recording_session_id = "B"
    second = controller.ensure_workstation_run_evidence("started")
    assert (first["session_id"], second["session_id"]) == ("A", "B")
    assert not (tmp_path / "A" / "run_setup.json").exists()
    assert not (tmp_path / "B" / "run_setup.json").exists()


def test_controller_registers_owned_artifact_without_copy(tmp_path: Path) -> None:
    panel = _panel(tmp_path, "A")
    session = tmp_path / "A"
    session.mkdir()
    artifact = session / "capture.bin"
    artifact.write_bytes(b"capture")
    record = RunEvidenceController(panel).register_workstation_session_artifact(artifact, role="capture/raw")
    assert record["relative_path"] == "capture.bin"
    assert not (session / "assets").exists()


def test_run_evidence_oserror_becomes_recording_failure_not_callback_exception(
    monkeypatch,
    tmp_path: Path,
) -> None:
    from Workstation.workstation_core.run_evidence import RunEvidenceLedger

    panel = _panel(tmp_path, "A")
    panel.logs = []
    panel._log = panel.logs.append
    monkeypatch.setattr(
        RunEvidenceLedger,
        "append_event",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("read-only-sentinel")),
    )

    result = RunEvidenceController(panel).ensure_workstation_run_evidence("sample")

    assert result["status"] == "failed"
    assert panel._session_save_retry_required is True
    assert "read-only-sentinel" in panel._session_save_last_error
    assert panel.logs


def test_session_prepared_failure_stops_artifact_write(
    monkeypatch,
    tmp_path: Path,
) -> None:
    from Workstation.workstation_core.run_evidence import RunEvidenceLedger

    calls: list[str] = []

    def fail_setup(*_args, **_kwargs):
        calls.append("session_prepared")
        raise OSError("setup-sentinel")

    def unexpected_import(*_args, **_kwargs):
        calls.append("artifact_import")
        raise AssertionError("artifact write must not run after setup failure")

    monkeypatch.setattr(RunEvidenceLedger, "append_event", fail_setup)
    monkeypatch.setattr(RunEvidenceLedger, "import_artifact", unexpected_import)
    panel = _panel(tmp_path, "A")

    result = RunEvidenceController(panel).import_workstation_session_artifact(
        tmp_path / "source.bin",
        role="capture/raw",
    )

    assert result["status"] == "failed"
    assert result["error"] == "setup-sentinel"
    assert calls == ["session_prepared"]


def test_session_switch_keeps_old_ledger_when_flush_fails(tmp_path: Path) -> None:
    panel = _panel(tmp_path, "A")
    controller = RunEvidenceController(panel)
    controller.ensure_workstation_run_evidence("started")
    old_ledger = controller.ledger
    assert old_ledger is not None

    def fail_flush() -> None:
        raise OSError("flush-sentinel")

    old_ledger.flush = fail_flush  # type: ignore[method-assign]
    panel.recording_session_id = "B"

    result = controller.ensure_workstation_run_evidence("started")

    assert result["status"] == "failed"
    assert result["kind"] == "run_event_setup"
    assert controller.ledger is old_ledger
    assert controller.ledger.root == (tmp_path / "A").resolve()
