from __future__ import annotations

import json

import pytest

from Workstation.workstation_core.recording_session import (
    SpoolRows,
    discover_retryable_sessions,
    resolve_session_dir,
    session_dir,
    write_recovery_metadata,
)


def test_open_existing_quarantines_only_a_truncated_final_record(tmp_path) -> None:
    path = tmp_path / "session" / ".spool" / "pair_rows.jsonl"
    path.parent.mkdir(parents=True)
    valid = json.dumps({"sample_index": 0, "value": "kept"}, sort_keys=True)
    torn = b'{"sample_index":1,"value":"torn"'
    path.write_bytes(valid.encode() + b"\n" + torn)

    rows = SpoolRows(supports_updates=True)
    rows.open_existing(path)

    assert list(rows) == [{"sample_index": 0, "value": "kept"}]
    assert rows.recovery_warnings
    assert len(rows.recovery_artifacts) == 1
    assert rows.recovery_artifacts[0].read_bytes() == torn
    assert path.read_bytes() == valid.encode() + b"\n"

    rows.append({"sample_index": 1, "value": "new"})
    rows.close()
    reopened = SpoolRows(supports_updates=True)
    reopened.open_existing(path)
    assert list(reopened) == [
        {"sample_index": 0, "value": "kept"},
        {"sample_index": 1, "value": "new"},
    ]


def test_open_existing_normalizes_a_complete_final_record_missing_only_newline(tmp_path) -> None:
    path = tmp_path / "camera.jsonl"
    path.write_text(json.dumps({"sequence": 1}), encoding="utf-8")

    rows = SpoolRows()
    rows.open_existing(path)

    assert list(rows) == [{"sequence": 1}]
    assert path.read_bytes().endswith(b"\n")
    assert rows.recovery_warnings == (
        "normalized valid unterminated final JSONL record",
    )


def test_open_existing_rejects_a_malformed_interior_record_without_rewriting(tmp_path) -> None:
    path = tmp_path / "px4.jsonl"
    original = b'{"sequence":0}\nnot-json\n{"sequence":2}\n'
    path.write_bytes(original)

    rows = SpoolRows()
    with pytest.raises(OSError, match="line=2"):
        rows.open_existing(path)

    assert path.read_bytes() == original
    assert rows.recovery_artifacts == ()


def test_retryable_discovery_scans_nested_measurements_without_returning_finalized_sibling(
    tmp_path,
) -> None:
    recordings = tmp_path / "recordings"
    session = recordings / "sessions" / "session-001"
    first = session / "measurements" / "m000001"
    second = session / "measurements" / "m000002"
    write_recovery_metadata(
        first,
        {"session_id": "session-001", "measurement_id": "m000001"},
    )
    write_recovery_metadata(
        second,
        {"session_id": "session-001", "measurement_id": "m000002"},
    )
    (first / "session_manifest.json").write_text(
        json.dumps({"status": "finalized"}), encoding="utf-8"
    )

    discovered = discover_retryable_sessions(recordings)

    assert discovered == [second]


def test_new_session_path_is_directly_below_recordings_root(tmp_path) -> None:
    recordings = tmp_path / "recordings"

    assert session_dir(recordings, "session-001") == recordings / "session-001"

    with pytest.raises(ValueError, match="non-reserved"):
        session_dir(recordings, "sessions")
    with pytest.raises(ValueError, match="non-reserved"):
        session_dir(recordings, ".hidden")


def test_retryable_discovery_prefers_new_direct_layout_and_keeps_legacy_reading(
    tmp_path,
) -> None:
    recordings = tmp_path / "recordings"
    direct = recordings / "session-direct" / "measurements" / "m000001"
    legacy = recordings / "sessions" / "session-legacy" / "measurements" / "m000001"
    write_recovery_metadata(direct, {"session_id": "session-direct"})
    write_recovery_metadata(legacy, {"session_id": "session-legacy"})

    discovered = discover_retryable_sessions(recordings)

    assert set(discovered) == {direct, legacy}


def test_session_resolver_rejects_duplicate_layout_and_symlink(tmp_path) -> None:
    recordings = tmp_path / "recordings"
    direct = recordings / "collision"
    legacy = recordings / "sessions" / "collision"
    direct.mkdir(parents=True)
    legacy.mkdir(parents=True)
    with pytest.raises(FileExistsError, match="collision"):
        resolve_session_dir(recordings, "collision")

    direct.rmdir()
    outside = tmp_path / "outside"
    outside.mkdir()
    direct.symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="symlinks"):
        resolve_session_dir(recordings, "collision")


def test_retry_discovery_rejects_same_id_in_direct_and_legacy_trees(tmp_path) -> None:
    recordings = tmp_path / "recordings"
    for root in (
        recordings / "duplicate" / "measurements" / "m000001",
        recordings / "sessions" / "duplicate" / "measurements" / "m000001",
    ):
        write_recovery_metadata(root, {"session_id": "duplicate"})

    with pytest.raises(FileExistsError, match="retry session ID collision"):
        discover_retryable_sessions(recordings)


def test_session_resolver_rejects_symlinked_legacy_container(tmp_path) -> None:
    recordings = tmp_path / "recordings"
    recordings.mkdir()
    outside = tmp_path / "outside-legacy"
    (outside / "legacy").mkdir(parents=True)
    (recordings / "sessions").symlink_to(outside, target_is_directory=True)

    with pytest.raises(ValueError, match="legacy container"):
        resolve_session_dir(recordings, "legacy")


def test_retry_discovery_ignores_spool_without_identity_metadata(tmp_path) -> None:
    recordings = tmp_path / "recordings"
    orphan = recordings / "orphan" / ".spool"
    orphan.mkdir(parents=True)
    (orphan / "pair_rows.jsonl").write_text('{"pair_id":"unknown"}\n')

    assert discover_retryable_sessions(recordings) == []


def test_spool_write_error_is_contained_and_marks_volatile_row(tmp_path) -> None:
    failures: list[str] = []
    rows = SpoolRows(on_write_error=lambda exc: failures.append(str(exc)))
    rows.bind(tmp_path / "spool.jsonl")

    class FailingHandle:
        def write(self, _payload: str) -> int:
            raise OSError("disk-full-sentinel")

        def close(self) -> None:
            return None

    rows._handle = FailingHandle()

    assert rows.append({"value": 7}) is False
    assert failures == ["disk-full-sentinel"]
    assert rows.write_error == "disk-full-sentinel"
    assert list(rows) == [{"value": 7}]
