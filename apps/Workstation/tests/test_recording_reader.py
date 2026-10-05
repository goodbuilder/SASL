from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from Workstation.workstation_core.recording_reader import read_measurement_stream
from Workstation.workstation_core.recording_session import (
    compact_distance_measurement_rows,
    save_session_files,
)
from Workstation.workstation_core.session_parquet import (
    finalize_camera_timeline,
    finalize_measurement_pairs,
)
from Workstation.evidence import EvidenceAdapter


def test_v4_compact_sync_delta_keeps_historical_selected_app_meaning() -> None:
    [row] = compact_distance_measurement_rows(
        [
            {
                "pair_id": "v4",
                "pairing_time_basis": "app_receive_monotonic",
                "pairing_time_basis_role": "authoritative_distance",
                "selected_pair_delta_ns": 20_000_000,
                "selected_pair_abs_delta_ns": 20_000_000,
                "app_receive_pair_delta_ns": 20_000_000,
                "rtcp_pair_delta_ns": 32_400_000_000_000,
                "ntp_evidence_role": "record_only",
            }
        ]
    )

    assert row["sync_delta_ns"] == 20_000_000
    assert row["selected_pair_delta_ns"] == 20_000_000
    assert row["rtcp_pair_delta_ns"] == 32_400_000_000_000
    assert row["ntp_evidence_role"] == "record_only"


def test_v5_compact_row_preserves_requested_effective_and_selected_rtcp_delta() -> None:
    [row] = compact_distance_measurement_rows(
        [
            {
                "pair_id": "v5",
                "pairing_time_basis_requested": "camera_rtcp",
                "pairing_time_basis": "camera_rtcp",
                "pairing_time_basis_role": "authoritative_distance",
                "pairing_fallback_active": False,
                "pairing_fallback_reason": "",
                "selected_pair_delta_ns": 10_000_000,
                "selected_pair_abs_delta_ns": 10_000_000,
                "app_receive_pair_delta_ns": 90_000_000,
                "rtcp_pair_delta_ns": 10_000_000,
                "ntp_evidence_role": "selected_distance_pairing",
            }
        ]
    )

    assert row["pairing_time_basis_requested"] == "camera_rtcp"
    assert row["pairing_time_basis"] == "camera_rtcp"
    assert row["pairing_fallback_active"] is False
    assert row["sync_delta_ns"] == 10_000_000
    assert row["app_receive_pair_delta_ns"] == 90_000_000
    assert row["ntp_evidence_role"] == "selected_distance_pairing"


def test_v3_compact_sync_delta_retains_legacy_rtcp_meaning() -> None:
    [row] = compact_distance_measurement_rows(
        [{"pair_id": "v3", "rtcp_pair_delta_ns": -10_000_000}]
    )

    assert row["sync_delta_ns"] == -10_000_000
    assert row["pairing_time_basis"] == ""


def test_v3_reader_prefers_verified_jsonl_even_if_derived_parquet_changes(
    tmp_path: Path,
) -> None:
    root = tmp_path / "measurement"
    saved = save_session_files(
        root,
        metadata={"session_id": root.name},
        pair_rows=[{"pair_id": "jsonl-source", "private": {"kept": True}}],
    )
    saved.measurement_pairs_parquet.write_bytes(b"broken-derived-view")

    stream = read_measurement_stream(root)

    assert stream.authoritative is True
    assert stream.legacy_fallback is False
    assert stream.path.name == "stereo_measurements.jsonl"
    assert stream.rows[0]["pair_id"] == "jsonl-source"
    assert stream.rows[0]["private"] == {"kept": True}


def test_v3_reader_never_falls_back_when_declared_jsonl_is_corrupt(
    tmp_path: Path,
) -> None:
    root = tmp_path / "measurement"
    saved = save_session_files(
        root,
        metadata={"session_id": root.name},
        pair_rows=[{"pair_id": "must-not-fallback"}],
    )
    saved.authoritative_jsonl_files[0].write_text(
        '{"pair_id":"tampered"}\n',
        encoding="utf-8",
    )

    with pytest.raises(OSError, match="does not match"):
        read_measurement_stream(root)


def test_legacy_parquet_only_reader_remains_available(tmp_path: Path) -> None:
    root = tmp_path / "legacy-measurement"
    data = root / "data"
    finalize_measurement_pairs(
        data / "measurement_pairs.parquet",
        [{"pair_id": "legacy", "recording_time_ns": 9_007_199_254_740_993}],
    )

    stream = read_measurement_stream(root)

    assert stream.authoritative is False
    assert stream.legacy_fallback is True
    assert stream.rows == (
        {"pair_id": "legacy", "recording_time_ns": 9_007_199_254_740_993},
    )


def test_legacy_partitioned_camera_reader_selects_requested_side(tmp_path: Path) -> None:
    root = tmp_path / "legacy-camera"
    data = root / "data"
    finalize_camera_timeline(
        data / "camera_timeline.parquet",
        {
            "left": [{"sequence": 1, "value": "L"}],
            "right": [{"sequence": 2, "value": "R"}],
        },
    )

    stream = read_measurement_stream(root, "raw/camera_right")

    assert stream.legacy_fallback is True
    assert stream.rows == ({"side": "right", "sequence": 2, "value": "R"},)


def test_context_stream_is_found_without_a_second_parent_copy(tmp_path: Path) -> None:
    root = tmp_path / "measurement"
    saved = save_session_files(
        root,
        metadata={"session_id": root.name},
        pair_rows=[{"pair_id": "context-fallback"}],
    )
    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))

    stream = read_measurement_stream(root, "context/geometry_contexts")

    assert stream.authoritative is True
    assert len(stream.rows) == 1
    assert stream.descriptor == manifest["artifact_details"]["geometry_contexts"]
    assert not (root.parent / "geometry_contexts.jsonl").exists()


def test_evidence_adapter_logical_role_is_jsonl_first_with_legacy_fallback(
    tmp_path: Path,
) -> None:
    new_root = (
        tmp_path / "recordings" / "new-session" / "measurements" / "m000001"
    )
    save_session_files(
        new_root,
        metadata={"session_id": "new-session", "measurement_id": "m000001"},
        pair_rows=[{"pair_id": "new-jsonl"}],
    )
    legacy_root = (
        tmp_path / "recordings" / "legacy-session" / "measurements" / "m000001"
    )
    finalize_measurement_pairs(
        legacy_root / "data" / "measurement_pairs.parquet",
        [{"pair_id": "legacy-parquet"}],
    )
    adapter = EvidenceAdapter(SimpleNamespace(app_root=tmp_path))

    new_preview = adapter.session_artifact_rows(
        "new-session",
        "measurement_pairs",
        measurement_id="m000001",
    )
    legacy_preview = adapter.session_artifact_rows(
        "legacy-session",
        "measurement_pairs",
        measurement_id="m000001",
    )

    assert new_preview["rows"][0]["pair_id"] == "new-jsonl"
    assert new_preview["authoritative"] is True
    assert new_preview["legacy_fallback"] is False
    assert legacy_preview["rows"] == ({"pair_id": "legacy-parquet"},)
    assert legacy_preview["authoritative"] is False
    assert legacy_preview["legacy_fallback"] is True
