from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from Workstation.workstation_core.run_evidence import RunEvidenceLedger, RunMode, RunSetup


def _setup(**overrides: object) -> RunSetup:
    values: dict[str, object] = {
        "experiment_id": "exp-001",
        "run_id": "run-007",
        "condition_id": "ridge-clear",
        "mode": RunMode.DRONE,
        "condition": (("baseline_m", 25.0), ("weather", "clear")),
        "requested_zoom_left": 8,
        "requested_zoom_right": 8,
    }
    values.update(overrides)
    return RunSetup(**values)  # type: ignore[arg-type]


def test_first_evidence_locks_join_identity_and_preserves_coordinate_contract(tmp_path: Path) -> None:
    ledger = RunEvidenceLedger(tmp_path / "session")
    setup = ledger.apply_setup(_setup())

    event = ledger.append_event(
        "target_rtk",
        {"north_m": 10.0, "east_m": 20.0, "down_m": -5.0},
        source="target_drone/GLOBAL_POSITION_INT",
        source_timestamp_ns=123,
        coordinate_contract="local NED [north,east,down] metres",
    )

    assert ledger.locked is True
    assert event["experiment_id"] == setup.experiment_id
    assert event["condition_signature_sha256"] == setup.condition_signature_sha256
    assert event["payload"] == {"north_m": 10.0, "east_m": 20.0, "down_m": -5.0}
    with pytest.raises(RuntimeError, match="locked"):
        ledger.apply_setup(_setup(run_id="run-008"))

    reopened = RunEvidenceLedger(tmp_path / "session")
    assert reopened.locked is True
    assert reopened.setup == setup


def test_condition_signature_is_order_independent_but_zoom_sensitive() -> None:
    first = _setup(condition=(("weather", "clear"), ("baseline_m", 25.0)))
    second = _setup(condition=(("baseline_m", 25.0), ("weather", "clear")))
    changed = _setup(requested_zoom_right=12)

    assert first.condition_signature_sha256 == second.condition_signature_sha256
    assert first.condition_signature_sha256 != changed.condition_signature_sha256


def test_import_artifact_copies_bytes_hashes_and_deduplicates_content(tmp_path: Path) -> None:
    source = tmp_path / "ridge DEM.tif"
    source.write_bytes(b"II*\x00fake-geotiff-elevation")
    ledger = RunEvidenceLedger(tmp_path / "session")
    ledger.apply_setup(_setup(mode="uam"))

    first = ledger.import_artifact(
        source,
        role="terrain/geotiff",
        media_type="image/tiff; application=geotiff",
        metadata={"crs": "EPSG:32652", "vertical_datum": "unknown"},
    )
    second = ledger.import_artifact(source, role="terrain/geotiff")

    expected = hashlib.sha256(source.read_bytes()).hexdigest()
    copied = ledger.resolve_artifact(first.relative_path)
    assert copied.read_bytes() == source.read_bytes()
    assert first.sha256 == expected
    assert first.size_bytes == len(source.read_bytes())
    assert first.relative_path == second.relative_path
    assert len(list((tmp_path / "session" / "assets" / "terrain_geotiff").glob("*.tif"))) == 1

    source.unlink()
    assert copied.exists(), "session evidence must not depend on the original external path"

    manifest = ledger.manifest_snapshot()
    assert manifest["schema"] == "session-manifest-v3"
    assert len(manifest["artifacts"]) == 2
    assert manifest["event_log"]["sha256"]


def test_resolve_artifact_rejects_path_traversal(tmp_path: Path) -> None:
    ledger = RunEvidenceLedger(tmp_path / "session")
    ledger.apply_setup(_setup())

    with pytest.raises(ValueError, match="escapes"):
        ledger.resolve_artifact("../../outside.ulg")


def test_non_finite_evidence_is_rejected_without_partial_line(tmp_path: Path) -> None:
    ledger = RunEvidenceLedger(tmp_path / "session")
    ledger.apply_setup(_setup())

    with pytest.raises(ValueError, match="non-finite"):
        ledger.append_event(
            "pose_candidate",
            {"residual_rad": float("nan")},
            source="test",
            coordinate_contract="OpenCV optical RDF",
        )

    events = tmp_path / "session" / "run_events.jsonl"
    assert not events.exists()


def test_persisted_json_is_canonical_and_contains_no_machine_source_path(tmp_path: Path) -> None:
    source = tmp_path / "hr_left.png"
    source.write_bytes(b"\x89PNG\r\n\x1a\nbytes")
    ledger = RunEvidenceLedger(tmp_path / "session")
    ledger.apply_setup(_setup())
    ledger.import_artifact(source, role="hr/imported/left", media_type="image/png")

    rows = [json.loads(line) for line in (tmp_path / "session" / "artifacts.jsonl").read_text().splitlines()]
    assert rows[0]["source_name"] == "hr_left.png"
    assert str(tmp_path) not in json.dumps(rows[0])

