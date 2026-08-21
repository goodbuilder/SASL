from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from Workstation.evidence import EvidenceAdapter
from Workstation.workstation_core.measurement_session import MeasurementSession


def _write_json(path: Path, payload: object) -> bytes:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded)
    return encoded


def _adapter(tmp_path: Path) -> EvidenceAdapter:
    return EvidenceAdapter(SimpleNamespace(app_root=tmp_path))


def test_nested_session_catalog_keeps_measurements_and_artifacts_isolated(
    tmp_path: Path,
) -> None:
    session_id = "20260821_120000_000000001"
    root = tmp_path / "recordings" / "sessions" / session_id
    session = MeasurementSession.create(root, session_id=session_id)
    for expected_id, value in (("m000001", "11.5"), ("m000002", "22.5")):
        package = session.begin_measurement()
        assert package.snapshot()["metadata"]["measurement_id"] == expected_id
        result = package.root / "results.csv"
        result.write_text(f"distance_m\n{value}\n", encoding="utf-8")
        package.register_artifact(
            "results.csv",
            role="measurement/result",
            classification="derived",
            producer="test",
            source="measurement",
            required=True,
            media_type="text/csv",
        )
        package.set_source("measurement", required_roles=("measurement/result",))
        package.mark_post_capture()
        package.seal({"result": "VALID"})
        session.refresh_measurement(package=package)

    records = _adapter(tmp_path).session_catalog()

    assert {(row["session_id"], row["measurement_id"]) for row in records} == {
        (session_id, "m000001"),
        (session_id, "m000002"),
    }
    assert len({row["catalog_key"] for row in records}) == 2
    first = next(row for row in records if row["measurement_id"] == "m000001")
    second = next(row for row in records if row["measurement_id"] == "m000002")
    assert Path(str(first["path"])).name == "m000001"
    assert Path(str(second["path"])).name == "m000002"
    first_preview = _adapter(tmp_path).session_artifact_rows(
        session_id,
        "results.csv",
        measurement_id="m000001",
    )
    second_preview = _adapter(tmp_path).session_artifact_rows(
        session_id,
        "results.csv",
        measurement_id="m000002",
    )
    assert first_preview["rows"] == ({"distance_m": "11.5"},)
    assert second_preview["rows"] == ({"distance_m": "22.5"},)


def test_package_preview_refuses_artifact_bytes_that_fail_manifest_hash(tmp_path: Path) -> None:
    session_id = "session-preview-tamper"
    root = tmp_path / "recordings" / "sessions" / session_id
    session = MeasurementSession.create(root, session_id=session_id)
    package = session.begin_measurement()
    result = package.root / "results.csv"
    result.write_text("distance_m\n11.5\n", encoding="utf-8")
    package.register_artifact(
        "results.csv",
        role="measurement/result",
        classification="derived",
        producer="test",
        source="measurement",
        required=True,
        media_type="text/csv",
    )
    package.set_source("measurement", required_roles=("measurement/result",))
    package.mark_post_capture()
    package.seal({"result": "VALID"})
    session.refresh_measurement(package=package)
    result.write_text("distance_m\n99.9\n", encoding="utf-8")

    preview = _adapter(tmp_path).session_artifact_rows(
        session_id,
        "results.csv",
        measurement_id="m000001",
    )

    assert preview["rows"] == ()
    assert "SHA-256" in str(preview["error"])


def test_latest_package_manifest_is_authoritative_and_projects_review_summary(
    tmp_path: Path,
) -> None:
    session = tmp_path / "recordings" / "sessions" / "session-package-list"
    result = session / "derived" / "results" / "measurements.csv"
    result.parent.mkdir(parents=True)
    result.write_text("distance_m\n12.5\n", encoding="utf-8")
    legacy = session / "legacy.csv"
    legacy.write_text("legacy\n1\n", encoding="utf-8")
    _write_json(
        session / "session_manifest.json",
        {"status": "finalized", "artifact_roles": {"legacy.csv": "legacy"}},
    )

    manifest = {
        "schema": "workstation-measurement-package-v1",
        "schema_version": 1,
        "session_id": session.name,
        "revision": "r0003",
        "revision_number": 3,
        "state": "SEALED_INCOMPLETE",
        "source_expectations": [
            {"source_id": "left-camera", "required": True},
            {"source_id": "right-camera", "required": True},
        ],
        "artifacts": [
            {
                "artifact_id": "distance-result",
                "role": "measurement/result",
                "classification": "derived",
                "relative_path": "derived/results/measurements.csv",
                "producer": "measurement",
                "source": "stereo",
                "required": True,
                "status": "present",
                "media_type": "text/csv",
                "size_bytes": result.stat().st_size,
                "sha256": hashlib.sha256(result.read_bytes()).hexdigest(),
            },
            {
                "artifact_id": "right-video",
                "role": "camera/right/video",
                "relative_path": None,
                "required": True,
                "status": "missing",
                "metadata": {"reason": "camera disconnected"},
            },
            {
                "artifact_id": "hr-left",
                "role": "camera/left/hr",
                "relative_path": "raw/camera/hr-left.png",
                "required": True,
                "status": "failed",
                "metadata": {"reason": "FTP timeout"},
            },
        ],
        "evidence_completeness": {
            "status": "INCOMPLETE",
            "complete": False,
            "missing_required": [
                {
                    "source": "right-camera",
                    "role": "camera/right/video",
                    "status": "missing",
                    "artifact_id": "right-video",
                }
            ],
        },
    }
    manifest_path = session / "revisions" / "r0003" / "package_manifest.json"
    manifest_bytes = _write_json(manifest_path, manifest)
    _write_json(
        session / "latest_revision.json",
        {
            "schema": "workstation-measurement-package-latest-v1",
            "session_id": session.name,
            "revision": "r0003",
            "revision_number": 3,
            "manifest_path": "revisions/r0003/package_manifest.json",
            "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
            "state": "SEALED_INCOMPLETE",
            "evidence_complete": False,
        },
    )

    [record] = _adapter(tmp_path).session_catalog()

    assert record["package_schema"] == "workstation-measurement-package-v1"
    assert record["package_schema_version"] == 1
    assert record["revision"] == "r0003"
    assert record["revision_number"] == 3
    assert record["status"] == "SEALED_INCOMPLETE"
    assert record["lifecycle_state"] == "SEALED_INCOMPLETE"
    assert record["completeness_state"] == "INCOMPLETE"
    assert record["evidence_complete"] is False
    assert record["total_bytes"] == result.stat().st_size
    assert record["artifacts"] == (
        "derived/results/measurements.csv",
        "raw/camera/hr-left.png",
    )
    assert record["hash_verification"]["manifest_status"] == "VERIFIED"
    assert record["hash_verification"]["artifact_hashes_declared"] == 1
    assert record["hash_verification"]["artifacts_not_run"] == 1
    assert "right-camera/camera/right/video: missing" in record["missing_reasons"]
    assert "right-video: camera disconnected" in record["missing_reasons"]
    assert "hr-left: FTP timeout" in record["failed_reasons"]
    assert len(record["source_expectations"]) == 2

    preview = _adapter(tmp_path).session_artifact_rows(
        session.name,
        "derived/results/measurements.csv",
    )
    assert preview["rows"] == ({"distance_m": "12.5"},)
    undeclared = _adapter(tmp_path).session_artifact_rows(session.name, "legacy.csv")
    assert "not declared by package manifest" in str(undeclared["error"])


def test_revision_scan_accepts_mapping_inventory_but_rejects_untrusted_paths(
    tmp_path: Path,
) -> None:
    session = tmp_path / "recordings" / "sessions" / "session-package-map"
    nested = session / "raw" / "nested" / "samples.jsonl"
    nested.parent.mkdir(parents=True)
    nested.write_text('{"sample":7}\n', encoding="utf-8")
    outside = tmp_path / "outside.jsonl"
    outside.write_text('{"secret":true}\n', encoding="utf-8")

    base = {
        "schema": {"name": "workstation-measurement-package-v1", "version": 1},
        "session_id": session.name,
        "state": "SEALED_COMPLETE",
        "evidence_completeness": "COMPLETE",
    }
    _write_json(
        session / "revisions" / "r0001" / "package_manifest.json",
        {**base, "revision": "r0001", "revision_number": 1, "artifacts": []},
    )
    _write_json(
        session / "revisions" / "r0002" / "package_manifest.json",
        {
            **base,
            "revision": "r0002",
            "revision_number": 2,
            "artifacts": {
                "raw/nested/samples.jsonl": {
                    "role": "telemetry/raw",
                    "status": "present",
                    "size": nested.stat().st_size,
                    "sha256": hashlib.sha256(nested.read_bytes()).hexdigest(),
                },
                "unsafe-relative": {
                    "artifact_id": "unsafe-relative",
                    "relative_path": "../outside.jsonl",
                    "status": "present",
                    "size_bytes": outside.stat().st_size,
                },
                "unsafe-absolute": {
                    "artifact_id": "unsafe-absolute",
                    "relative_path": str(outside),
                    "status": "present",
                },
                "missing-camera": {
                    "artifact_id": "missing-camera",
                    "status": "missing",
                    "metadata": {"reason": "source inactive"},
                },
            },
        },
    )

    [record] = _adapter(tmp_path).session_catalog()

    assert record["revision"] == "r0002"
    assert record["package_schema"] == "workstation-measurement-package-v1"
    assert record["artifacts"] == ("raw/nested/samples.jsonl",)
    assert record["total_bytes"] == nested.stat().st_size
    assert sum("unsafe artifact path rejected" in reason for reason in record["failed_reasons"]) == 2
    assert "missing-camera: source inactive" in record["missing_reasons"]
    assert all(not str(detail["relative_path"]).startswith(("/", "..")) for detail in record["artifact_details"])

    preview = _adapter(tmp_path).session_artifact_rows(
        session.name,
        "raw/nested/samples.jsonl",
    )
    assert preview["rows"] == ({"sample": 7},)
    traversal = _adapter(tmp_path).session_artifact_rows(session.name, "../outside.jsonl")
    absolute = _adapter(tmp_path).session_artifact_rows(session.name, str(outside))
    assert "unsafe" in str(traversal["error"])
    assert "unsafe" in str(absolute["error"])


def test_malformed_package_manifest_does_not_fall_back_to_legacy_inventory(
    tmp_path: Path,
) -> None:
    session = tmp_path / "recordings" / "sessions" / "session-malformed"
    manifest = session / "revisions" / "r0001" / "package_manifest.json"
    manifest.parent.mkdir(parents=True)
    malformed = b'{"schema":"workstation-measurement-package-v1",'
    manifest.write_bytes(malformed)
    legacy = session / "legacy.csv"
    legacy.write_text("value\n1\n", encoding="utf-8")
    _write_json(
        session / "session_manifest.json",
        {"status": "finalized", "artifact_roles": {"legacy.csv": "legacy"}},
    )
    _write_json(
        session / "latest_revision.json",
        {
            "revision": "r0001",
            "manifest_path": "revisions/r0001/package_manifest.json",
            "manifest_sha256": hashlib.sha256(malformed).hexdigest(),
        },
    )

    record = next(
        item
        for item in _adapter(tmp_path).session_catalog()
        if item["session_id"] == session.name
    )

    assert record["status"] == "INTEGRITY_FAILED"
    assert record["lifecycle_state"] == "UNKNOWN"
    assert record["completeness_state"] == "UNKNOWN"
    assert record["artifacts"] == ()
    assert record["legacy"] is False
    assert "malformed package manifest" in str(record["error"])
    preview = _adapter(tmp_path).session_artifact_rows(session.name, "legacy.csv")
    assert "package manifest is unreadable" in str(preview["error"])


def test_unsafe_latest_revision_pointer_is_reported_without_legacy_fallback(
    tmp_path: Path,
) -> None:
    session = tmp_path / "recordings" / "sessions" / "session-unsafe-pointer"
    session.mkdir(parents=True)
    _write_json(session / "session_manifest.json", {"status": "finalized"})
    _write_json(
        session / "latest_revision.json",
        {"revision": "r0001", "manifest_path": "../outside/package_manifest.json"},
    )
    _write_json(
        session.parent / "outside" / "package_manifest.json",
        {"schema": "workstation-measurement-package-v1", "artifacts": []},
    )

    record = next(
        item
        for item in _adapter(tmp_path).session_catalog()
        if item["session_id"] == session.name
    )

    assert record["status"] == "INTEGRITY_FAILED"
    assert record["artifacts"] == ()
    assert "unsafe manifest_path" in str(record["error"])
    assert record["legacy"] is False


def test_session_manifest_only_session_remains_readable_as_legacy_unverified(
    tmp_path: Path,
) -> None:
    session = tmp_path / "recordings" / "sessions" / "session-legacy"
    nested = session / "assets" / "telemetry" / "samples.csv"
    nested.parent.mkdir(parents=True)
    nested.write_text("sample\n9\n", encoding="utf-8")
    outside = tmp_path / "secret.csv"
    outside.write_text("secret\n1\n", encoding="utf-8")
    _write_json(
        session / "session_manifest.json",
        {
            "status": "finalized",
            "artifact_details": {
                "telemetry": {"path": "assets/telemetry/samples.csv"},
            },
            "artifact_roles": {str(outside): "must-not-be-trusted"},
        },
    )

    [record] = _adapter(tmp_path).session_catalog()

    assert record["status"] == "LEGACY_UNVERIFIED"
    assert record["lifecycle_state"] == "finalized"
    assert record["completeness_state"] == "LEGACY_UNVERIFIED"
    assert record["integrity_status"] == "LEGACY_UNVERIFIED"
    assert record["artifacts"] == ("assets/telemetry/samples.csv",)
    assert str(outside) not in record["artifacts"]
    preview = _adapter(tmp_path).session_artifact_rows(
        session.name,
        "assets/telemetry/samples.csv",
    )
    assert preview["rows"] == ({"sample": "9"},)
