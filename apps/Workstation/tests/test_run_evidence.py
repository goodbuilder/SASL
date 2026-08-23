from __future__ import annotations

import json
from pathlib import Path

import pytest

from Workstation.workstation_core.run_evidence import RunEvidenceLedger, sha256_file


def test_event_uses_session_identity_without_run_setup(tmp_path: Path) -> None:
    root = tmp_path / "SESSION-1"
    ledger = RunEvidenceLedger(root)
    event = ledger.append_event("pose_sample", {"ned_m": [1.0, 2.0, -3.0]}, source="test", coordinate_contract="world NED [north,east,down] m")
    assert event["session_id"] == "SESSION-1"
    assert event["coordinate_contract"] == "world NED [north,east,down] m"
    assert not (root / "run_setup.json").exists()


def test_legacy_run_setup_is_ignored_and_not_required(tmp_path: Path) -> None:
    root = tmp_path / "SESSION-OLD"
    root.mkdir()
    (root / "run_setup.json").write_text('{"schema":"workstation-run-setup-v1"}\n')
    event = RunEvidenceLedger(root).append_event("opened", {}, source="test")
    assert event["session_id"] == "SESSION-OLD"


def test_import_artifact_hashes_and_records_content(tmp_path: Path) -> None:
    source = tmp_path / "source.bin"
    source.write_bytes(b"evidence")
    ledger = RunEvidenceLedger(tmp_path / "SESSION-2")
    record = ledger.import_artifact(source, role="camera/raw")
    stored = ledger.resolve_artifact(record.relative_path)
    assert stored.read_bytes() == b"evidence"
    assert record.sha256 == sha256_file(stored)


def test_register_session_artifact_rejects_hash_mismatch(tmp_path: Path) -> None:
    ledger = RunEvidenceLedger(tmp_path / "SESSION-3")
    owned = ledger.root / "video.bin"
    owned.write_bytes(b"video")
    assert ledger.register_session_artifact(owned, role="video/raw").relative_path == "video.bin"
    with pytest.raises(ValueError, match="SHA-256"):
        ledger.register_session_artifact(owned, role="bad", expected_sha256="0" * 64)


def test_manifest_and_canonical_event_have_no_run_setup(tmp_path: Path) -> None:
    ledger = RunEvidenceLedger(tmp_path / "SESSION-4")
    ledger.append_event("started", {}, source="test")
    manifest = ledger.manifest_snapshot()
    assert manifest["session_id"] == "SESSION-4"
    assert "run_setup" not in manifest
    row = json.loads((ledger.root / "run_events.jsonl").read_text().splitlines()[0])
    assert row["session_id"] == "SESSION-4"


def test_non_finite_payload_is_rejected_without_partial_line(tmp_path: Path) -> None:
    ledger = RunEvidenceLedger(tmp_path / "SESSION-5")
    with pytest.raises(ValueError, match="non-finite"):
        ledger.append_event("bad", {"value": float("nan")}, source="test")
    assert not (ledger.root / "run_events.jsonl").exists()
