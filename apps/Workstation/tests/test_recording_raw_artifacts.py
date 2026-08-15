from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path

import pytest

from Workstation.workstation_core import recording_session
from Workstation.workstation_core.recording_session import (
    SpoolRows,
    load_retryable_session,
    save_session_files,
    write_recovery_metadata,
)


RAW_RESOURCE_KEYS = (
    "raw_pair_rows_jsonl",
    "raw_camera_left_rows_jsonl",
    "raw_camera_right_rows_jsonl",
    "raw_px4_left_rows_jsonl",
    "raw_px4_right_rows_jsonl",
    "raw_gnss_base_rows_jsonl",
)


def _read_jsonl(path: Path) -> list[dict[str, object]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def _bound_source_spools(
    output_dir: Path,
) -> tuple[
    SpoolRows,
    dict[str, SpoolRows],
    dict[str, SpoolRows],
    SpoolRows,
]:
    spool_dir = output_dir / ".spool"
    pair_rows = SpoolRows(preview_limit=2, supports_updates=True)
    pair_rows.bind(spool_dir / "pair_rows.jsonl")
    camera_rows = {side: SpoolRows(preview_limit=2) for side in ("left", "right")}
    px4_rows = {side: SpoolRows(preview_limit=2) for side in ("left", "right")}
    for side in ("left", "right"):
        camera_rows[side].bind(spool_dir / f"camera_{side}.jsonl")
        px4_rows[side].bind(spool_dir / f"px4_{side}.jsonl")
    gnss_base_rows = SpoolRows(preview_limit=2)
    gnss_base_rows.bind(spool_dir / "gnss_base.jsonl")
    return pair_rows, camera_rows, px4_rows, gnss_base_rows


def _all_spools(
    pair_rows: SpoolRows,
    camera_rows: dict[str, SpoolRows],
    px4_rows: dict[str, SpoolRows],
    gnss_base_rows: SpoolRows,
) -> tuple[SpoolRows, ...]:
    return (
        pair_rows,
        camera_rows["left"],
        camera_rows["right"],
        px4_rows["left"],
        px4_rows["right"],
        gnss_base_rows,
    )


def test_save_publishes_verified_lossless_jsonl_before_discarding_spools(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "sessions" / "raw-session"
    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(output_dir)
    pair_rows.append(
        {
            "sample_index": 41,
            "session_elapsed_s": 1.25,
            "distance_m": 17.5,
            "odd_pair_private": {"nested": [1, "산"]},
            "odd_native_path": Path("native/left-0041.png"),
        }
    )
    camera_rows["left"].append(
        {
            "sequence": 5,
            "side": "left",
            "odd_camera_private": ["rtp", 77],
        }
    )
    camera_rows["right"].append(
        {"sequence": 6, "side": "right", "odd_camera_right": "보존"}
    )
    px4_rows["left"].append(
        {"sequence": 7, "side": "left", "odd_px4_private": {"boot": 123}}
    )
    px4_rows["right"].append(
        {"sequence": 8, "side": "right", "odd_px4_right": 9.25}
    )
    gnss_base_rows.append(
        {"sequence": 9, "side": "base", "odd_base_private": [37.1, 127.2]}
    )
    metadata = {"session_id": "raw-session", "session_started_unix_ns": 123}
    write_recovery_metadata(output_dir, metadata)

    expected_by_resource = {
        "raw_pair_rows_jsonl": list(pair_rows),
        "raw_camera_left_rows_jsonl": list(camera_rows["left"]),
        "raw_camera_right_rows_jsonl": list(camera_rows["right"]),
        "raw_px4_left_rows_jsonl": list(px4_rows["left"]),
        "raw_px4_right_rows_jsonl": list(px4_rows["right"]),
        "raw_gnss_base_rows_jsonl": list(gnss_base_rows),
    }
    spool_paths = tuple(
        spool.spool_path
        for spool in _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows)
    )

    saved = save_session_files(
        output_dir,
        metadata=metadata,
        pair_rows=pair_rows,
        camera_rows=camera_rows,
        px4_rows=px4_rows,
        gnss_base_rows=gnss_base_rows,
    )

    assert saved.finalized is True
    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    assert manifest["status"] == "finalized"
    assert manifest["authoritative_raw_artifacts_verified"] is True
    assert set(RAW_RESOURCE_KEYS) <= set(manifest["resource_files"])
    for resource_key, expected_rows in expected_by_resource.items():
        name = manifest["resource_files"][resource_key]
        artifact_path = output_dir / name
        assert artifact_path.is_symlink()
        assert _read_jsonl(artifact_path) == expected_rows
        detail = manifest["artifact_details"][name]
        assert detail["role"] == resource_key
        assert detail["path"] == name
        assert detail["size"] == artifact_path.stat().st_size
        assert detail["size_unit"] == "bytes"
        assert detail["row_count"] == len(expected_rows)
        assert detail["schema"]["version"] == (
            recording_session.AUTHORITATIVE_JSONL_SCHEMA_VERSION
        )
        assert detail["schema"]["format"] == "jsonl"
        assert detail["schema"]["fields"] == sorted(
            {key for row in expected_rows for key in row}
        )
        assert detail["sha256"] == hashlib.sha256(artifact_path.read_bytes()).hexdigest()
        assert detail["authoritative"] is True
        assert detail["verified"] is True
        assert manifest["artifact_roles"][name] == resource_key
        assert manifest["artifacts"][name] == "prepared"

    raw_pair_name = manifest["resource_files"]["raw_pair_rows_jsonl"]
    raw_pair_row = _read_jsonl(output_dir / raw_pair_name)[0]
    assert json.loads(raw_pair_row["odd_pair_private"]) == {"nested": [1, "산"]}
    assert raw_pair_row["odd_native_path"] == "native/left-0041.png"
    assert "odd_pair_private" not in saved.distance_measurements_csv.read_text(
        encoding="utf-8"
    ).splitlines()[0]
    generation_manifest = json.loads(
        (output_dir / "published" / "generation_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    assert generation_manifest["artifact_details"] == manifest["artifact_details"]

    assert all(path is not None and not path.exists() for path in spool_paths)
    assert all(
        spool.spool_path is None
        for spool in _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows)
    )
    assert not (output_dir / ".spool" / "session_metadata.json").exists()


@pytest.mark.parametrize("failure_mode", ("write", "validation"))
def test_raw_jsonl_failure_preserves_spools_and_previous_generation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure_mode: str,
) -> None:
    output_dir = tmp_path / "sessions" / "retry-session"
    first = save_session_files(
        output_dir,
        metadata={"session_id": "retry-session"},
        pair_rows=[
            {"sample_index": 0, "session_elapsed_s": 0.0, "distance_m": 1.0}
        ],
    )
    previous_pointer = (output_dir / "published").readlink()
    previous_raw_name = json.loads(first.manifest.read_text(encoding="utf-8"))[
        "resource_files"
    ]["raw_pair_rows_jsonl"]
    previous_raw_bytes = (output_dir / previous_raw_name).read_bytes()

    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(output_dir)
    pair_rows.append(
        {
            "sample_index": 1,
            "session_elapsed_s": 1.0,
            "distance_m": 99.0,
            "odd_retry_field": "must-survive",
        }
    )
    camera_rows["right"].append(
        {"sequence": 3, "side": "right", "odd_retry_camera": True}
    )
    metadata = {"session_id": "retry-session", "session_started_unix_ns": 456}
    write_recovery_metadata(output_dir, metadata)
    source_spools = _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows)
    source_paths = tuple(spool.spool_path for spool in source_spools)

    original_writer = recording_session._write_authoritative_jsonl

    if failure_mode == "write":

        def fail_writer(output: Path, rows) -> None:
            if output.name.endswith("_raw_camera_right_rows.jsonl"):
                raise OSError("raw JSONL write fault")
            original_writer(output, rows)

        monkeypatch.setattr(recording_session, "_write_authoritative_jsonl", fail_writer)
        expected_error = "raw JSONL write fault"
    else:

        def corrupt_writer(output: Path, rows) -> None:
            original_writer(output, rows)
            if output.name.endswith("_raw_camera_right_rows.jsonl"):
                with output.open("ab") as handle:
                    handle.write(b'{"unexpected":"row"}\n')
                    handle.flush()
                    os.fsync(handle.fileno())

        monkeypatch.setattr(recording_session, "_write_authoritative_jsonl", corrupt_writer)
        expected_error = "authoritative JSONL has an unexpected row"

    with pytest.raises(OSError, match=expected_error):
        save_session_files(
            output_dir,
            metadata=metadata,
            pair_rows=pair_rows,
            camera_rows=camera_rows,
            px4_rows=px4_rows,
            gnss_base_rows=gnss_base_rows,
        )

    assert (output_dir / "published").readlink() == previous_pointer
    assert (output_dir / previous_raw_name).read_bytes() == previous_raw_bytes
    assert (output_dir / ".generations" / previous_pointer.name).is_dir()
    assert all(path is not None and path.is_file() for path in source_paths)
    assert [row["odd_retry_field"] for row in pair_rows] == ["must-survive"]
    failed_manifest = json.loads(
        (output_dir / "session_manifest.json").read_text(encoding="utf-8")
    )
    assert failed_manifest["status"] == "failed"
    assert failed_manifest["phase"] == "prepare_resource_files"
    assert failed_manifest["retryable"] is True
    assert failed_manifest["authoritative_raw_artifacts_verified"] is False
    assert failed_manifest["artifact_details"] == {}
    for detail in failed_manifest["unpublished_artifact_details"].values():
        assert detail["published"] is False
        assert ".preparing/" in detail["path"]

    for spool in source_spools:
        spool.close()
    recovered = load_retryable_session(output_dir)
    try:
        assert len(recovered.pair_rows) == 1
        assert recovered.pair_rows[0]["odd_retry_field"] == "must-survive"
        assert recovered.camera_rows["right"][0]["odd_retry_camera"] is True
    finally:
        for spool in _all_spools(
            recovered.pair_rows,
            recovered.camera_rows,
            recovered.px4_rows,
            recovered.gnss_base_rows,
        ):
            spool.close()


def test_non_finite_json_value_is_rejected_without_discarding_spool(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "sessions" / "strict-json"
    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(output_dir)
    pair_rows.append(
        {
            "sample_index": 0,
            "session_elapsed_s": 0.0,
            "odd_non_finite": math.nan,
        }
    )
    metadata = {"session_id": "strict-json", "session_started_unix_ns": 789}
    write_recovery_metadata(output_dir, metadata)
    spool_path = pair_rows.spool_path

    with pytest.raises(ValueError, match="Out of range float values"):
        save_session_files(
            output_dir,
            metadata=metadata,
            pair_rows=pair_rows,
            camera_rows=camera_rows,
            px4_rows=px4_rows,
            gnss_base_rows=gnss_base_rows,
        )

    assert spool_path is not None and spool_path.is_file()
    assert math.isnan(pair_rows[0]["odd_non_finite"])
    manifest = json.loads(
        (output_dir / "session_manifest.json").read_text(encoding="utf-8")
    )
    assert manifest["status"] == "failed"
    assert manifest["phase"] == "prepare_resource_files"
    assert manifest["artifact_details"] == {}
    assert manifest["authoritative_raw_artifacts_verified"] is False
    for spool in _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows):
        spool.close()


def test_publish_failure_scopes_new_details_and_keeps_spool_and_old_generation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output_dir = tmp_path / "sessions" / "publish-retry"
    first = save_session_files(
        output_dir,
        metadata={"session_id": "publish-retry"},
        pair_rows=[
            {"sample_index": 0, "session_elapsed_s": 0.0, "distance_m": 2.0}
        ],
    )
    previous_pointer = (output_dir / "published").readlink()
    previous_csv = first.distance_measurements_csv.read_bytes()

    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(output_dir)
    pair_rows.append(
        {"sample_index": 1, "session_elapsed_s": 1.0, "odd_new_generation": 3}
    )
    write_recovery_metadata(output_dir, {"session_id": "publish-retry"})
    spool_path = pair_rows.spool_path

    def fail_publish(*_args, **_kwargs) -> None:
        raise OSError("published pointer fault")

    monkeypatch.setattr(recording_session, "_publish_generation", fail_publish)
    with pytest.raises(OSError, match="published pointer fault"):
        save_session_files(
            output_dir,
            metadata={"session_id": "publish-retry"},
            pair_rows=pair_rows,
            camera_rows=camera_rows,
            px4_rows=px4_rows,
            gnss_base_rows=gnss_base_rows,
        )

    assert (output_dir / "published").readlink() == previous_pointer
    assert first.distance_measurements_csv.read_bytes() == previous_csv
    assert spool_path is not None and spool_path.is_file()
    manifest = json.loads(
        (output_dir / "session_manifest.json").read_text(encoding="utf-8")
    )
    assert manifest["status"] == "failed"
    assert manifest["phase"] == "publish"
    assert manifest["artifact_details"] == {}
    assert manifest["authoritative_raw_artifacts_verified"] is False
    assert manifest["unpublished_artifact_details"]
    for detail in manifest["unpublished_artifact_details"].values():
        assert detail["published"] is False
        assert ".generations/" in detail["path"]
        assert ".preparing/" not in detail["path"]
    generations = list((output_dir / ".generations").iterdir())
    assert output_dir / ".generations" / previous_pointer.name in generations
    assert len(generations) == 2
    for spool in _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows):
        spool.close()


def test_retryable_loader_accepts_legacy_spool_without_raw_artifact_manifest(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "sessions" / "legacy-retry"
    pair_rows = SpoolRows(supports_updates=True)
    pair_rows.bind(output_dir / ".spool" / "pair_rows.jsonl")
    pair_rows.append({"sample_index": 2, "legacy_private": "still-readable"})
    pair_rows.close()
    write_recovery_metadata(output_dir, {"session_id": "legacy-retry"})

    recovered = load_retryable_session(output_dir)
    try:
        assert recovered.pair_rows[0]["legacy_private"] == "still-readable"
        assert len(recovered.camera_rows["left"]) == 0
        assert len(recovered.px4_rows["right"]) == 0
        assert len(recovered.gnss_base_rows) == 0
    finally:
        for spool in _all_spools(
            recovered.pair_rows,
            recovered.camera_rows,
            recovered.px4_rows,
            recovered.gnss_base_rows,
        ):
            spool.close()
