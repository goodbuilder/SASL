from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from Workstation.evidence import EvidenceAdapter
from Workstation.workstation_core.measurement_session import (
    SESSION_MANIFEST_NAME,
    MeasurementSession,
    verify_measurement_session,
)
from Workstation.workstation_core.recording_session import save_session_files


def test_parent_manifest_indexes_child_generation_contexts_and_binary(
    tmp_path: Path,
) -> None:
    root = tmp_path / "recordings" / "session-catalog"
    session = MeasurementSession.create(root, session_id=root.name)
    package = session.begin_measurement()
    video = package.root / "video" / "raw_left.mp4"
    video.parent.mkdir(parents=True)
    video.write_bytes(b"exact-mp4-bytes")
    save_session_files(
        package.root,
        metadata={"session_id": root.name, "measurement_id": "m000001"},
        pair_rows=[{"pair_id": "pair-1"}],
    )
    package.register_artifact(
        "video/raw_left.mp4",
        role="video/left/raw",
        classification="raw",
        producer="test",
        source="camera.left",
        media_type="video/mp4",
    )
    package.mark_post_capture()
    package.seal({"result": "VALID"})
    session.refresh_measurement(package=package)
    manifest = session.seal()

    [row] = manifest["measurements"]
    assert row["measurement_id"] == "m000001"
    assert row["measurement_package_id"] == "m000001"
    assert row["sequence"] == 1
    assert row["recording_generation"]["generation_id"]
    assert row["recording_generation"]["current_pointer_target"].startswith(
        "generations/"
    )
    roles = {fragment["role"] for fragment in row["authoritative_jsonl_fragments"]}
    assert "raw/stereo_measurements" in roles
    assert "context/geometry_contexts" in roles
    assert "context/time_sync_epochs" in roles
    assert {fragment["role"] for fragment in row["context_fragments"]} == {
        "context/geometry_contexts",
        "context/time_sync_epochs",
    }
    assert any(
        artifact["relative_path"] == "video/raw_left.mp4"
        and artifact["sha256"]
        for artifact in row["binary_artifacts"]
    )
    assert not list(root.glob("*.jsonl"))
    assert verify_measurement_session(root).valid is True

    metadata = manifest["metadata"]
    assert metadata["identity"] == {
        "session_id": root.name,
        "application": "Workstation",
        "experiment_type": None,
    }
    assert metadata["software"]["commit"]
    assert isinstance(metadata["software"]["dirty"], bool)
    assert metadata["camera"]["requested_fps"] is None
    assert metadata["credentials"]["secret_values_persisted"] is False
    [catalog] = EvidenceAdapter(SimpleNamespace(app_root=tmp_path)).session_catalog()
    assert catalog["recording_generation"]["generation_id"] == row[
        "recording_generation"
    ]["generation_id"]
    assert catalog["authoritative_jsonl_fragments"] == row[
        "authoritative_jsonl_fragments"
    ]


def test_parent_verifier_detects_generation_inventory_tamper(tmp_path: Path) -> None:
    root = tmp_path / "session-tamper"
    session = MeasurementSession.create(root, session_id=root.name)
    package = session.begin_measurement()
    save_session_files(
        package.root,
        metadata={"session_id": root.name, "measurement_id": "m000001"},
        pair_rows=[{"pair_id": "pair-1"}],
    )
    package.mark_post_capture()
    package.seal({"result": "VALID"})
    session.refresh_measurement(package=package)
    session.seal()

    manifest_path = root / SESSION_MANIFEST_NAME
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["measurements"][0]["authoritative_jsonl_fragments"] = []
    from Workstation.workstation_core.measurement_package import canonical_json_bytes
    import hashlib

    payload = canonical_json_bytes(manifest) + b"\n"
    manifest_path.write_bytes(payload)
    (root / "measurement_session_manifest.sha256").write_text(
        f"{hashlib.sha256(payload).hexdigest()}  {SESSION_MANIFEST_NAME}\n",
        encoding="ascii",
    )

    result = verify_measurement_session(root)

    assert result.valid is False
    assert "measurement_recording_generation_mismatch" in {
        issue.code for issue in result.issues
    }
