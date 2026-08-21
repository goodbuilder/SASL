from __future__ import annotations

import hashlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from Workstation.run_evidence_controller import RunEvidenceController


def test_locked_run_identity_does_not_leak_into_next_explicit_session(
    tmp_path: Path,
) -> None:
    panel = SimpleNamespace(
        recording_session_id="session-a",
        actual_visible_zoom={},
        _recording_session_dir=lambda session_id: tmp_path / session_id,
    )
    controller = RunEvidenceController(panel)
    controller.apply_workstation_run_setup(
        {
            "experiment_id": "exp",
            "run_id": "session-a",
            "condition_id": "condition",
            "mode": "drone",
        }
    )
    controller.ensure_workstation_run_evidence("measurement_started")
    assert controller.workstation_run_setup_snapshot()["locked"] is True

    panel.recording_session_id = "session-b"
    snapshot = controller.apply_workstation_run_setup(
        {
            "experiment_id": "exp",
            "run_id": "session-b",
            "condition_id": "condition",
            "mode": "drone",
        }
    )

    assert snapshot["run_id"] == "session-b"
    assert snapshot["locked"] is False
    assert controller.ledger is not None
    assert controller.ledger.root == (tmp_path / "session-b").resolve()


def test_locked_run_setup_rehydrates_idempotently_after_restart(tmp_path: Path) -> None:
    panel = SimpleNamespace(
        recording_session_id="session-restart",
        actual_visible_zoom={},
        _recording_session_dir=lambda session_id: tmp_path / session_id,
    )
    setup = {
        "experiment_id": "exp-restart",
        "run_id": "session-restart",
        "condition_id": "condition-a",
        "mode": "drone",
    }
    first = RunEvidenceController(panel)
    first.apply_workstation_run_setup(setup)
    first.ensure_workstation_run_evidence("measurement_started")

    restarted = RunEvidenceController(panel)
    snapshot = restarted.workstation_run_setup_snapshot()

    assert snapshot["locked"] is True
    assert snapshot["experiment_id"] == "exp-restart"
    assert restarted.apply_workstation_run_setup(setup)["locked"] is True
    event = restarted.append_workstation_run_event(
        "measurement_resumed",
        {"sentinel": 7},
        source="test",
        coordinate_contract="not_applicable",
    )
    assert event["run_id"] == "session-restart"
    with pytest.raises(RuntimeError, match="locked"):
        restarted.apply_workstation_run_setup(
            {**setup, "condition_id": "condition-b"}
        )


def test_controller_registers_session_owned_artifact_without_assets_copy(tmp_path: Path) -> None:
    root = tmp_path / "session-a"
    panel = SimpleNamespace(
        recording_session_id="session-a",
        actual_visible_zoom={},
        _recording_session_dir=lambda session_id: tmp_path / session_id,
    )
    controller = RunEvidenceController(panel)
    controller.apply_workstation_run_setup(
        {
            "experiment_id": "exp",
            "run_id": "session-a",
            "condition_id": "condition",
            "mode": "drone",
        }
    )
    artifact = root / "raw" / "target.jsonl"
    artifact.parent.mkdir(parents=True)
    artifact.write_bytes(b'{"target":1}\n')
    digest = hashlib.sha256(artifact.read_bytes()).hexdigest()

    record = controller.register_workstation_session_artifact(
        artifact,
        role="target_drone_raw_mavlink_jsonl",
        media_type="application/x-ndjson",
        expected_size_bytes=artifact.stat().st_size,
        expected_sha256=digest,
    )
    repeated = controller.register_workstation_session_artifact(
        artifact,
        role="target_drone_raw_mavlink_jsonl",
        media_type="application/x-ndjson",
        expected_size_bytes=artifact.stat().st_size,
        expected_sha256=digest,
    )

    assert record["relative_path"] == "raw/target.jsonl"
    assert record["sha256"] == digest
    assert repeated == record
    assert len((root / "artifacts.jsonl").read_text(encoding="utf-8").splitlines()) == 1
    assert not (root / "assets").exists()
