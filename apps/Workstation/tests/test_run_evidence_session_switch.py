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
