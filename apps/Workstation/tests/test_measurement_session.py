from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
import Workstation.workstation_core.measurement_session as measurement_session_module

from Workstation.workstation_core.measurement_session import (
    SESSION_MANIFEST_NAME,
    SESSION_STATE_NAME,
    MeasurementSession,
    verify_measurement_session,
)


def _seal_measurement(session: MeasurementSession, content: bytes = b"result") -> Path:
    package = session.measurement_package()
    result_path = package.root / "results" / "result.json"
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_bytes(content)
    package.register_artifact(
        "results/result.json",
        role="measurement/result",
        classification="derived",
        producer="test",
        source="measurement",
        required=True,
        media_type="application/json",
    )
    package.set_source("measurement", required_roles=("measurement/result",))
    package.mark_post_capture()
    package.seal({"result": "VALID"})
    session.refresh_measurement(package=package)
    return result_path


def test_explicit_session_owns_ordered_independent_measurements(tmp_path: Path) -> None:
    root = tmp_path / "20260821_120000_123456789"
    session = MeasurementSession.create(
        root,
        session_id=root.name,
        app_run_id="run-001",
        metadata={"operator": "field"},
    )

    first = session.begin_measurement(metadata={"purpose": "baseline"})
    assert first.root == root / "measurements" / "m000001"
    assert first.session_id == root.name
    assert first.snapshot()["metadata"]["measurement_id"] == "m000001"
    assert first.snapshot()["metadata"]["measurement_package_id"] == "m000001"
    assert first.snapshot()["measurement_package_id"] == "m000001"
    with pytest.raises(RuntimeError, match="must be sealed"):
        session.begin_measurement()

    _seal_measurement(session, b"first")
    second = session.begin_measurement(metadata={"purpose": "repeat"})
    assert second.root == root / "measurements" / "m000002"
    assert second.root != first.root
    snapshot = session.snapshot()
    assert snapshot["active_measurement_id"] == "m000002"
    assert [row["measurement_id"] for row in snapshot["measurements"]] == [
        "m000001",
        "m000002",
    ]
    assert [row["measurement_package_id"] for row in snapshot["measurements"]] == [
        "m000001",
        "m000002",
    ]
    assert snapshot["app_runs"] == [
        {"app_run_id": "run-001", "joined_utc_ns": snapshot["app_runs"][0]["joined_utc_ns"]}
    ]


def test_reopen_recovers_active_measurement_and_appends_app_run(tmp_path: Path) -> None:
    root = tmp_path / "session"
    created = MeasurementSession.create(root, session_id="session", app_run_id="run-a")
    created.begin_measurement()

    reopened = MeasurementSession.open(root)
    assert reopened.active_measurement_id == "m000001"
    assert reopened.measurement_package().state == "COLLECTING"
    reopened.add_app_run("run-b")
    reopened.add_app_run("run-b")
    assert [row["app_run_id"] for row in reopened.snapshot()["app_runs"]] == [
        "run-a",
        "run-b",
    ]


def test_session_seal_hashes_child_manifests_and_detects_tampering(tmp_path: Path) -> None:
    root = tmp_path / "session"
    session = MeasurementSession.create(root, session_id="session")
    session.begin_measurement()
    result_path = _seal_measurement(session)
    manifest = session.seal()

    assert manifest["state"] == "SEALED"
    assert (root / SESSION_STATE_NAME).is_file()
    assert (root / SESSION_MANIFEST_NAME).is_file()
    assert manifest["measurements"][0]["manifest_relative_path"] == (
        "measurements/m000001/revisions/r0001/package_manifest.json"
    )
    verification = verify_measurement_session(root)
    assert verification.valid is True
    assert verification.verified_measurements == 1
    with pytest.raises(RuntimeError, match="not open"):
        session.begin_measurement()

    result_path.write_bytes(b"tampered")
    failed = verify_measurement_session(root)
    assert failed.valid is False
    assert "measurement_integrity_failed" in {issue.code for issue in failed.issues}


def test_session_cannot_close_with_mutable_measurement(tmp_path: Path) -> None:
    session = MeasurementSession.create(tmp_path / "session", session_id="session")
    session.begin_measurement()

    with pytest.raises(RuntimeError, match="must be sealed first"):
        session.seal()


def test_reconcile_adopts_child_created_before_session_index_persist(tmp_path: Path) -> None:
    root = tmp_path / "session"
    session = MeasurementSession.create(root, session_id="session")
    package_root = root / "measurements" / "m000001"
    from Workstation.workstation_core.measurement_package import MeasurementPackage

    MeasurementPackage.create(
        package_root,
        session_id="session",
        metadata={"measurement_id": "m000001", "measurement_sequence": 1},
    )

    reopened = MeasurementSession.open(root)
    assert reopened.active_measurement_id == "m000001"
    assert reopened.snapshot()["measurements"][0]["relative_path"] == (
        "measurements/m000001"
    )


def test_session_rejects_unsafe_identifiers(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="safe path component"):
        MeasurementSession.create(tmp_path / "bad", session_id="../bad")
    session = MeasurementSession.create(tmp_path / "safe", session_id="safe")
    with pytest.raises(ValueError, match="safe path component"):
        session.begin_measurement(measurement_id="../m1")


def test_session_manifest_is_portable_json(tmp_path: Path) -> None:
    root = tmp_path / "session"
    session = MeasurementSession.create(root, session_id="session")
    session.begin_measurement()
    _seal_measurement(session)
    session.seal()

    payload = json.loads((root / SESSION_MANIFEST_NAME).read_text(encoding="utf-8"))
    serialized = json.dumps(payload, ensure_ascii=False)
    assert str(tmp_path) not in serialized
    assert all(not Path(row["relative_path"]).is_absolute() for row in payload["measurements"])
    relocated = tmp_path / "relocated" / "session"
    relocated.parent.mkdir()
    shutil.copytree(root, relocated)
    shutil.rmtree(root)
    assert verify_measurement_session(relocated).valid


def test_measurement_allocation_skips_orphan_directory_and_protects_identity(
    tmp_path: Path,
) -> None:
    root = tmp_path / "session"
    session = MeasurementSession.create(root, session_id="session")
    (root / "measurements" / "m000007").mkdir(parents=True)

    package = session.begin_measurement(
        metadata={
            "measurement_id": "forged",
            "measurement_sequence": 99,
            "parent_session_id": "other",
        }
    )

    assert package.root.name == "m000008"
    assert package.snapshot()["metadata"] | {} == {
        "measurement_id": "m000008",
        "measurement_package_id": "m000008",
        "measurement_sequence": 8,
        "parent_session_id": "session",
    }
    _seal_measurement(session)
    with pytest.raises(RuntimeError, match="orphan measurement directory"):
        session.seal()


def test_session_close_retry_adopts_existing_parent_manifest(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    root = tmp_path / "session"
    session = MeasurementSession.create(root, session_id="session")
    session.begin_measurement()
    _seal_measurement(session)
    original = measurement_session_module._atomic_write_bytes
    failed = False

    def fail_sidecar(path: Path, payload: bytes, *, replace: bool = True) -> None:
        nonlocal failed
        if Path(path).name.endswith(".sha256") and not failed:
            failed = True
            raise OSError("injected parent sidecar failure")
        original(path, payload, replace=replace)

    monkeypatch.setattr(measurement_session_module, "_atomic_write_bytes", fail_sidecar)
    with pytest.raises(OSError, match="injected parent"):
        session.seal()
    before = session.manifest_path.read_bytes()

    reopened = MeasurementSession.open(root)
    manifest = reopened.seal()

    assert manifest["state"] == "SEALED"
    assert reopened.manifest_path.read_bytes() == before
    assert verify_measurement_session(root).valid


def test_session_manifest_hashes_shared_evidence(tmp_path: Path) -> None:
    root = tmp_path / "session"
    session = MeasurementSession.create(root, session_id="session")
    shared = root / "shared" / "run_setup.json"
    shared.parent.mkdir(parents=True)
    shared.write_text('{"run":"A"}\n', encoding="utf-8")
    session.begin_measurement()
    _seal_measurement(session)
    manifest = session.seal()

    assert manifest["shared_artifacts"][0]["relative_path"] == "shared/run_setup.json"
    assert verify_measurement_session(root).valid
    shared.write_text('{"run":"B"}\n', encoding="utf-8")
    failed = verify_measurement_session(root)
    assert "shared_artifact_hash_mismatch" in {issue.code for issue in failed.issues}


def test_session_rejects_windows_style_traversal_component(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="safe path component"):
        MeasurementSession.create(tmp_path / "bad", session_id="..\\bad")
