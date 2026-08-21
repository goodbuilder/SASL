from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from Workstation.evidence import EvidenceAdapter, resolve_session_artifact_path
from Workstation.workstation_core.session_parquet import finalize_measurement_pairs


def test_nested_session_artifact_is_preserved_and_previewed(tmp_path: Path) -> None:
    root = tmp_path / "recordings" / "run-1"
    nested = root / "assets" / "target_drone"
    nested.mkdir(parents=True)
    artifact = nested / "raw.jsonl"
    artifact.write_text('{"source_id":"target_drone","latitude_deg":37.0}\n', encoding="utf-8")
    (root / "artifacts.jsonl").write_text(
        json.dumps({"relative_path": "assets/target_drone/raw.jsonl"}) + "\n",
        encoding="utf-8",
    )
    (root / "session_manifest.json").write_text('{"status":"finalized"}', encoding="utf-8")

    adapter = EvidenceAdapter(SimpleNamespace(app_root=tmp_path))
    catalog = adapter.session_catalog()
    preview = adapter.session_artifact_rows("run-1", "assets/target_drone/raw.jsonl")

    assert "assets/target_drone/raw.jsonl" in catalog[0]["artifacts"]
    assert preview["rows"] == ({"source_id": "target_drone", "latitude_deg": 37.0},)
    assert Path(str(preview["path"])) == artifact


def test_legacy_sessions_directory_remains_reviewable(tmp_path: Path) -> None:
    root = tmp_path / "recordings" / "sessions" / "legacy-run"
    root.mkdir(parents=True)
    artifact = root / "events.jsonl"
    artifact.write_text('{"event":"legacy"}\n', encoding="utf-8")
    (root / "session_manifest.json").write_text(
        '{"status":"finalized"}', encoding="utf-8"
    )

    adapter = EvidenceAdapter(SimpleNamespace(app_root=tmp_path))

    assert any(row["session_id"] == "legacy-run" for row in adapter.session_catalog())
    preview = adapter.session_artifact_rows("legacy-run", "events.jsonl")
    assert preview["rows"] == ({"event": "legacy"},)


def test_session_artifact_resolver_rejects_traversal_and_escaping_symlink(tmp_path: Path) -> None:
    root = tmp_path / "session"
    root.mkdir()
    outside = tmp_path / "secret.jsonl"
    outside.write_text("{}\n", encoding="utf-8")
    (root / "escape.jsonl").symlink_to(outside)

    assert resolve_session_artifact_path(root, "../secret.jsonl") is None
    assert resolve_session_artifact_path(root, str(outside)) is None
    assert resolve_session_artifact_path(root, "escape.jsonl") is None


def test_session_artifact_resolver_supports_published_generation(tmp_path: Path) -> None:
    root = tmp_path / "session"
    generation = root / ".generations" / "g1"
    generation.mkdir(parents=True)
    artifact = generation / "pairs.csv"
    artifact.write_text("a\n1\n", encoding="utf-8")
    (root / "published").symlink_to(Path(".generations") / "g1")

    assert resolve_session_artifact_path(root, "pairs.csv") == artifact.resolve()


def test_direct_session_parquet_rows_are_previewed_losslessly(tmp_path: Path) -> None:
    measurement = (
        tmp_path / "recordings" / "run-parquet" / "measurements" / "m000001"
    )
    data = measurement / "data"
    finalize_measurement_pairs(
        data / "measurement_pairs.parquet",
        [{"pair_id": "pair-1", "distance_m": 12.5, "future": {"kept": True}}],
    )
    adapter = EvidenceAdapter(SimpleNamespace(app_root=tmp_path))

    preview = adapter.session_artifact_rows(
        "run-parquet",
        "data/measurement_pairs.parquet",
        measurement_id="m000001",
    )

    assert preview["error"] == ""
    assert preview["rows"] == (
        {"pair_id": "pair-1", "distance_m": 12.5, "future": {"kept": True}},
    )
