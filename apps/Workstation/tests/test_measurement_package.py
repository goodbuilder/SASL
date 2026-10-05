from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
import Workstation.workstation_core.measurement_package as measurement_package_module

from Workstation.workstation_core.measurement_package import (
    LATEST_REVISION_NAME,
    MANIFEST_NAME,
    MANIFEST_SIDECAR_NAME,
    PACKAGE_SCHEMA,
    WORKING_STATE_NAME,
    MeasurementPackage,
    canonical_json_bytes,
    verify_package,
)


def _write_artifact(root: Path, relative_path: str, content: bytes) -> Path:
    path = root / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return path


def _register_required_video(package: MeasurementPackage, relative_path: str = "raw/left.mp4") -> None:
    package.register_artifact(
        relative_path,
        role="camera/video/left",
        classification="raw",
        producer="camera-recorder",
        source="left_camera",
        required=True,
        media_type="video/mp4",
        schema="workstation-camera-video-v1",
        metadata={"frame_count": 3},
    )


def _rewrite_manifest(manifest_path: Path, value: dict[str, object]) -> None:
    payload = canonical_json_bytes(value) + b"\n"
    manifest_path.write_bytes(payload)
    digest = hashlib.sha256(payload).hexdigest()
    (manifest_path.parent / MANIFEST_SIDECAR_NAME).write_text(
        f"{digest}  {MANIFEST_NAME}\n", encoding="ascii"
    )


@pytest.mark.parametrize("failure_phase", ("sidecar", "latest", "working"))
def test_seal_retry_adopts_manifest_after_later_publish_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure_phase: str,
) -> None:
    root = tmp_path / failure_phase
    package = MeasurementPackage.create(root, session_id=f"session-{failure_phase}")
    _write_artifact(root, "raw/result.json", b"{}\n")
    package.register_artifact(
        "raw/result.json",
        role="measurement/result",
        classification="raw",
        producer="test",
        source="measurement",
        required=True,
    )
    original_bytes = measurement_package_module._atomic_write_bytes
    original_json = measurement_package_module._atomic_write_json
    failed = False

    def write_bytes(path: Path, payload: bytes, *, replace: bool = True) -> None:
        nonlocal failed
        if (
            failure_phase == "sidecar"
            and Path(path).name == MANIFEST_SIDECAR_NAME
            and not failed
        ):
            failed = True
            raise OSError("injected sidecar failure")
        original_bytes(path, payload, replace=replace)

    def write_json(
        path: Path,
        value: dict[str, object],
        *,
        replace: bool = True,
    ) -> None:
        nonlocal failed
        should_fail = (
            failure_phase == "latest" and Path(path).name == LATEST_REVISION_NAME
        ) or (
            failure_phase == "working"
            and Path(path).name == WORKING_STATE_NAME
            and str(value.get("state") or "").startswith("SEALED_")
        )
        if should_fail and not failed:
            failed = True
            raise OSError(f"injected {failure_phase} failure")
        original_json(path, value, replace=replace)

    monkeypatch.setattr(measurement_package_module, "_atomic_write_bytes", write_bytes)
    monkeypatch.setattr(measurement_package_module, "_atomic_write_json", write_json)
    with pytest.raises(OSError, match="injected"):
        package.seal({"result": "VALID"})
    manifest_path = root / "revisions" / "r0001" / MANIFEST_NAME
    before = manifest_path.read_bytes()

    reopened = MeasurementPackage.open(root)
    manifest = reopened.seal({"result": "VALID"})

    assert manifest["state"] == "SEALED_COMPLETE"
    assert manifest_path.read_bytes() == before
    assert verify_package(root).valid is True


def test_working_state_is_persisted_and_paths_are_session_owned(tmp_path: Path) -> None:
    root = tmp_path / "session"
    package = MeasurementPackage.create(
        root,
        session_id="measurement-001",
        metadata={"nested": {"threshold": 0.25}},
    )
    package.set_source(
        "left_camera",
        active=True,
        required_roles=("camera/video/left",),
        metadata={"serial": "LEFT-7"},
    )
    package.set_source("unused_gnss", active=False, required_roles=("gnss/raw",))
    target = _write_artifact(root, "raw/actual-left.mp4", b"ftyp-independent-fixture")
    alias = root / "raw" / "left-link.mp4"
    alias.symlink_to(target.name)

    entry = package.register_artifact(
        "camera/video/left",
        alias,
        classification="raw",
        producer="camera-recorder",
        source="left_camera",
        required=True,
        status="present",
        metadata={"gain": 1.5},
    )

    assert entry.relative_path == "raw/actual-left.mp4"
    assert entry.size_bytes == len(target.read_bytes())
    assert entry.sha256 == hashlib.sha256(target.read_bytes()).hexdigest()
    assert (root / WORKING_STATE_NAME).is_file()
    reopened = MeasurementPackage.open(root)
    assert reopened.session_id == "measurement-001"
    assert reopened.state == "COLLECTING"
    assert reopened.snapshot()["source_expectations"][0] == {
        "active": True,
        "metadata": {"serial": "LEFT-7"},
        "required_roles": ["camera/video/left"],
        "source_id": "left_camera",
        "status": "active",
    }
    assert reopened.snapshot()["evidence_completeness"]["complete"] is True

    outside = _write_artifact(tmp_path, "outside.bin", b"outside")
    escaping_link = root / "escaping-link.bin"
    escaping_link.symlink_to(outside)
    with pytest.raises(ValueError, match="escapes"):
        reopened.register_artifact(
            escaping_link,
            role="bad",
            classification="raw",
            producer="test",
            source="left_camera",
        )
    with pytest.raises(ValueError, match="non-finite"):
        reopened.set_measurement_outcome({"distance_m": float("nan")})


def test_active_source_cannot_be_downgraded_to_not_applicable(tmp_path: Path) -> None:
    package = MeasurementPackage.create(tmp_path / "session", session_id="active-source")
    package.set_source(
        "camera.left",
        active=True,
        required_roles=("video/left/raw",),
    )

    with pytest.raises(ValueError, match="cannot become not_applicable"):
        package.set_source(
            "camera.left",
            active=False,
            required_roles=("video/left/raw",),
        )

    [source] = package.snapshot()["source_expectations"]
    assert source["status"] == "active"


def test_late_present_artifact_satisfies_prior_failed_required_role(tmp_path: Path) -> None:
    root = tmp_path / "session"
    package = MeasurementPackage.create(root, session_id="late-evidence")
    package.set_source(
        "camera.left",
        active=True,
        required_roles=("captures/hr/left",),
    )
    package.register_failed(
        "captures/hr/left",
        classification="raw",
        producer="capture-worker",
        source="camera.left",
        required=True,
        reason="initial FTP timeout",
    )
    late = _write_artifact(root, "late/hr-left.jpg", b"late-exact-bytes")
    package.register_artifact(
        late,
        role="captures/hr/left",
        classification="raw",
        producer="manual-import",
        source="camera.left",
        required=True,
    )

    completeness = package.snapshot()["evidence_completeness"]
    assert completeness["complete"] is True
    assert completeness["missing_required"] == []


def test_seal_creates_complete_portable_manifest_and_auto_inventory(tmp_path: Path) -> None:
    root = tmp_path / "session"
    package = MeasurementPackage.create(root, session_id="measurement-002")
    package.set_source("left_camera", active=True, required_roles=("camera/video/left",))
    package.set_source("unused_gnss", active=False, required_roles=("gnss/raw",))
    video = _write_artifact(root, "raw/left.mp4", b"video-byte-fixture")
    _register_required_video(package)
    video.write_bytes(b"video-byte-fixture-after-drain")
    unregistered = _write_artifact(root, "logs/operator.log", b"operator evidence\n")
    empty_jsonl = _write_artifact(root, "raw/no-results.jsonl", b"")
    package.register_artifact(
        empty_jsonl,
        role="measurements/raw_pair_rows",
        classification="raw",
        producer="measurement-worker",
        source="measurement",
        media_type="application/x-ndjson",
    )
    _write_artifact(root, "logs/unfinished.part", b"temporary")
    package.mark_post_capture()

    manifest = package.seal({"status": "NO_VALID_RESULT", "reason": "no target"})

    manifest_path = root / "revisions" / "r0001" / MANIFEST_NAME
    sidecar_path = manifest_path.parent / MANIFEST_SIDECAR_NAME
    latest = json.loads((root / LATEST_REVISION_NAME).read_text(encoding="utf-8"))
    assert manifest["schema"] == PACKAGE_SCHEMA
    assert manifest["state"] == "SEALED_COMPLETE"
    assert manifest["evidence_completeness"] == {
        "status": "COMPLETE",
        "complete": True,
        "missing_required": [],
    }
    assert manifest["measurement_outcome"]["status"] == "NO_VALID_RESULT"
    assert latest["manifest_path"] == "revisions/r0001/package_manifest.json"
    assert latest["manifest_sha256"] == hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    assert sidecar_path.read_text(encoding="ascii").split()[0] == latest["manifest_sha256"]

    by_path = {item["relative_path"]: item for item in manifest["artifacts"]}
    assert by_path["raw/left.mp4"]["sha256"] == hashlib.sha256(video.read_bytes()).hexdigest()
    operator_path = "logs/operator.log"
    assert by_path[operator_path]["classification"] == "auto_inventory"
    assert by_path[operator_path]["metadata"]["storage_class"] == "sealed_immutable"
    assert by_path[operator_path]["metadata"]["snapshot_policy"] == "reference_by_hash"
    empty_entry = next(
        item for item in manifest["artifacts"] if item["role"] == "measurements/raw_pair_rows"
    )
    assert empty_entry["size_bytes"] == 0
    assert empty_entry["sha256"] == hashlib.sha256(b"").hexdigest()
    assert "logs/unfinished.part" not in by_path
    assert by_path["revisions/r0001/summary.md"]["media_type"] == "text/markdown"
    required_keys = {
        "role",
        "classification",
        "relative_path",
        "producer",
        "source",
        "required",
        "status",
        "media_type",
        "schema",
        "size_bytes",
        "sha256",
        "metadata",
    }
    assert required_keys <= set(by_path[operator_path])
    assert unregistered.read_bytes() == b"operator evidence\n"

    result = verify_package(root)
    assert result.valid is True
    assert result.state == "VERIFIED"
    assert result.verified_manifests == 1
    assert result.verified_artifacts == 4
    assert bool(result) is True
    with pytest.raises(RuntimeError, match="immutable"):
        package.register_missing("late", source="test")


def test_required_missing_and_failed_artifacts_seal_incomplete(tmp_path: Path) -> None:
    root = tmp_path / "session"
    package = MeasurementPackage.create(root, session_id="measurement-003")
    package.set_source(
        "left_camera",
        active=True,
        required_roles=("camera/video/left", "camera/frame/left"),
    )
    package.set_source("right_camera", status="not_applicable", required_roles=("camera/video/right",))
    _write_artifact(root, "raw/left.mp4", b"left-video")
    _register_required_video(package)
    missing = package.register_missing(
        "camera/frame/left",
        relative_path="frames/left.png",
        classification="raw",
        producer="frame-store",
        source="left_camera",
        reason="capture timeout",
    )
    failed = package.register_failed(
        "camera/video/right",
        classification="raw",
        producer="camera-recorder",
        source="right_camera",
        reason="source disabled",
    )

    assert missing.size_bytes is None and missing.sha256 is None
    assert failed.size_bytes is None and failed.sha256 is None
    manifest = package.seal({"status": "VALID", "distance_m": 123.0})
    assert manifest["state"] == "SEALED_INCOMPLETE"
    assert manifest["evidence_completeness"]["missing_required"] == [
        {"source": "left_camera", "role": "camera/frame/left", "status": "missing"}
    ]
    assert manifest["measurement_outcome"]["status"] == "VALID"
    assert verify_package(root).valid is True


@pytest.mark.parametrize(
    ("mutation", "expected_code"),
    [
        ("content", "artifact_size_mismatch"),
        ("missing", "artifact_missing"),
    ],
)
def test_verifier_detects_missing_or_tampered_artifact(
    tmp_path: Path, mutation: str, expected_code: str
) -> None:
    root = tmp_path / mutation
    package = MeasurementPackage.create(root, session_id=f"measurement-{mutation}")
    path = _write_artifact(root, "raw/left.mp4", b"original")
    _register_required_video(package)
    package.seal("VALID")

    if mutation == "content":
        path.write_bytes(b"tampered-longer")
    else:
        path.unlink()

    result = verify_package(root)
    assert result.valid is False
    assert result.state == "INTEGRITY_FAILED"
    assert expected_code in {issue.code for issue in result.issues}


def test_verifier_detects_manifest_sidecar_and_path_escape(tmp_path: Path) -> None:
    root = tmp_path / "session"
    package = MeasurementPackage.create(root, session_id="measurement-escape")
    _write_artifact(root, "raw/left.mp4", b"original")
    _register_required_video(package)
    package.seal("VALID")
    manifest_path = root / "revisions" / "r0001" / MANIFEST_NAME

    (manifest_path.parent / MANIFEST_SIDECAR_NAME).write_text("0" * 64, encoding="ascii")
    result = verify_package(root, revision=1)
    assert "manifest_sidecar_mismatch" in {issue.code for issue in result.issues}

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    artifact = next(item for item in manifest["artifacts"] if item["role"] == "camera/video/left")
    artifact["relative_path"] = "../../outside.bin"
    _rewrite_manifest(manifest_path, manifest)
    escaped = verify_package(root, revision="r0001")
    assert "artifact_path_escape" in {issue.code for issue in escaped.issues}


def test_new_revision_preserves_old_manifest_and_builds_hash_chain(tmp_path: Path) -> None:
    root = tmp_path / "session"
    package = MeasurementPackage.create(root, session_id="measurement-revisions")
    _write_artifact(root, "raw/left.mp4", b"revision-one")
    _register_required_video(package)
    first = package.seal("REJECTED")
    first_path = root / "revisions" / "r0001" / MANIFEST_NAME
    first_bytes = first_path.read_bytes()
    first_hash = hashlib.sha256(first_bytes).hexdigest()

    second_working = package.create_revision()
    assert second_working.revision == "r0002"
    assert second_working.state == "COLLECTING"
    _write_artifact(root, "post/diagnostic.json", b'{"reason":"blur"}\n')
    second_working.register_artifact(
        "post/diagnostic.json",
        role="measurement/diagnostic",
        classification="derived",
        producer="measurement-worker",
        source="measurement",
        media_type="application/json",
    )
    second = second_working.seal("REJECTED")

    assert first_path.read_bytes() == first_bytes
    assert second["previous_manifest"] == {
        "revision": "r0001",
        "revision_number": 1,
        "relative_path": "revisions/r0001/package_manifest.json",
        "sha256": first_hash,
    }
    result = verify_package(root)
    assert result.valid is True
    assert result.revision == "r0002"
    assert result.verified_manifests == 2
    assert MeasurementPackage.open_revision(root, 1).snapshot() == first

    first_path.write_bytes(first_bytes + b" ")
    broken = verify_package(root)
    assert "revision_chain_hash_mismatch" in {issue.code for issue in broken.issues}


def test_revision_snapshots_append_only_ledgers_before_later_rows_are_added(tmp_path: Path) -> None:
    root = tmp_path / "session"
    package = MeasurementPackage.create(root, session_id="measurement-ledger-revisions")
    ledger = _write_artifact(root, "run_events.jsonl", b'{"event":1}\n')
    final_rows = _write_artifact(root, "raw/final_pair_rows.jsonl", b'{"pair":1}\n')
    package.register_artifact(
        ledger,
        role="audit/run-events",
        classification="audit",
        producer="run-ledger",
        source="session",
        storage_class="appendable_ledger",
        snapshot_policy="copy_on_revision",
    )
    package.register_artifact(
        final_rows,
        role="measurements/raw_pair_rows",
        classification="raw",
        producer="session-writer",
        source="measurement",
    )
    first = package.seal("VALID")
    first_entry = next(item for item in first["artifacts"] if item["role"] == "audit/run-events")
    final_entry = next(
        item for item in first["artifacts"] if item["role"] == "measurements/raw_pair_rows"
    )
    assert first_entry["relative_path"] == (
        "revisions/r0001/snapshots/run_events.jsonl"
    )
    assert first_entry["metadata"]["storage_class"] == "appendable_ledger"
    assert first_entry["metadata"]["snapshot_policy"] == "copy_on_revision"
    assert final_entry["relative_path"] == "raw/final_pair_rows.jsonl"
    assert final_entry["metadata"]["storage_class"] == "sealed_immutable"
    assert final_entry["metadata"]["snapshot_policy"] == "reference_by_hash"
    assert not (root / "revisions/r0001/snapshots/raw/final_pair_rows.jsonl").exists()
    frozen_bytes = (root / first_entry["relative_path"]).read_bytes()

    second_working = package.create_revision()
    ledger.write_bytes(ledger.read_bytes() + b'{"event":2}\n')
    second_working.seal("VALID")

    assert (root / first_entry["relative_path"]).read_bytes() == frozen_bytes
    assert verify_package(root, revision=1).valid is True
    assert verify_package(root).valid is True


@pytest.mark.parametrize(
    "relative_path",
    ("audit/application_log.jsonl", "hr_imports.jsonl"),
)
def test_known_session_ledgers_are_explicitly_classified_appendable(
    tmp_path: Path,
    relative_path: str,
) -> None:
    root = tmp_path / "session"
    package = MeasurementPackage.create(root, session_id="measurement-known-ledger")
    ledger = _write_artifact(root, relative_path, b'{"event":1}\n')

    entry = package.register_artifact(
        ledger,
        role=f"audit/{ledger.name}",
        classification="audit",
        producer="session-ledger",
        source="session",
    )
    manifest = package.seal("VALID")
    frozen = next(item for item in manifest["artifacts"] if item["role"] == entry.role)

    assert entry.metadata["storage_class"] == "appendable_ledger"
    assert entry.metadata["snapshot_policy"] == "copy_on_revision"
    assert frozen["relative_path"] == f"revisions/r0001/snapshots/{relative_path}"


def test_latest_verification_rejects_evidence_added_without_a_revision(tmp_path: Path) -> None:
    root = tmp_path / "session"
    package = MeasurementPackage.create(root, session_id="measurement-unlisted")
    _write_artifact(root, "raw/left.mp4", b"sealed")
    _register_required_video(package)
    package.seal("VALID")

    _write_artifact(root, "post/unreviewed.json", b'{"late":true}\n')

    latest = verify_package(root)
    assert latest.valid is False
    assert "unlisted_evidence" in {issue.code for issue in latest.issues}
    # An explicit historical revision remains internally valid; the extra
    # file is a latest-package workflow violation, not a rewrite of r0001.
    assert verify_package(root, revision=1).valid is True
