from __future__ import annotations

from Workstation.workstation_core.session_storage import canonical as _session_canonical

from Workstation.workstation_core.session_storage import generation as _session_generation
from Workstation.workstation_core.session_storage import manifest as _session_manifest
from Workstation.workstation_core.session_storage import review as _session_review

import hashlib
import json
import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from zipfile import ZipFile

import pytest
import pyarrow.parquet as pq

from Workstation.recording_controller import RecordingController
from Workstation.workstation_core import recording_session, session_parquet
from Workstation.workstation_core.measurement_widgets import DistanceHistoryPoint
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.recording_session import (
    SpoolRows,
    load_retryable_session,
    rebuild_session_review,
    save_session_files,
    write_recovery_metadata,
)
from Workstation.workstation_core.session_parquet import read_lossless_rows


def _bound_source_spools(
    output_dir: Path,
) -> tuple[SpoolRows, dict[str, SpoolRows], dict[str, SpoolRows], SpoolRows]:
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


def test_hiding_graph_point_never_mutates_append_only_pair_spool(tmp_path: Path) -> None:
    spool = SpoolRows(preview_limit=8, supports_updates=True)
    spool.bind(tmp_path / ".spool" / "pair_rows.jsonl")
    spool.append({"pair_id": "p1", "distance_m": 10.0})
    spool.append({"pair_id": "p2", "distance_m": 20.0})
    spool.flush()
    before = spool.spool_path.read_bytes()
    first = DistanceHistoryPoint(
        timestamp_s=1.0,
        distance_m=10.0,
        quality="GOOD",
        method="triangulation",
        sample_key=("p1",),
    )
    second = DistanceHistoryPoint(
        timestamp_s=2.0,
        distance_m=20.0,
        quality="GOOD",
        method="triangulation",
        sample_key=("p2",),
    )
    logs: list[str] = []
    panel = SimpleNamespace(
        distance_history=deque((first, second), maxlen=8),
        distance_recording_rows=spool,
        distance_recording_keys=[("p1",), ("p2",)],
        _recording_pair_row_index={("p1",): 0, ("p2",): 1},
        _last_distance_history_key=("p2",),
        _distance_history_key=lambda point: tuple(point.sample_key or ()),
        _refresh_recording_table=lambda: None,
        _refresh_recording_status_label=lambda: None,
        _refresh_operation_header=lambda: None,
        _log=logs.append,
    )

    RecordingController(panel)._remove_distance_history_point(first)

    assert list(panel.distance_history) == [second]
    assert spool.spool_path.read_bytes() == before
    assert [row["pair_id"] for row in spool] == ["p1", "p2"]
    assert panel.distance_recording_keys == [("p1",), ("p2",)]
    assert panel._recording_pair_row_index == {("p1",): 0, ("p2",): 1}
    assert any("original" in message.lower() or "원본" in message for message in logs)


def test_save_publishes_verified_parquet_and_one_session_review_before_discarding_spools(
    tmp_path: Path,
) -> None:
    session_root = tmp_path / "recordings" / "session-001"
    output_dir = session_root / "measurements" / "m000001"
    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(output_dir)
    pair_rows.append(
        {
            "sample_index": 41,
            "session_elapsed_s": 1.25,
            "session_elapsed_ns": 1_250_000_000,
            "recorded_unix_ns": 1_700_000_000_000_000_123,
            "pair_mid_utc_ns": 1_700_000_000_000_000_100,
            "rtcp_pair_delta_ns": 10_000_000,
            "distance_m": 17.5,
            "measurement_valid": True,
            "odd_pair_private": {"nested": [1, "산"]},
            "odd_native_path": Path("native/left-0041.png"),
        }
    )
    camera_rows["left"].append(
        {
            "sequence": 5,
            "session_elapsed_s": "1.500000000",
            "session_utc_ns": 1_700_000_000_500_000_000,
            "source_utc_ns": 1_700_000_000_490_000_000,
            "rtcp_sr_age_s": "0.025",
            "odd_camera": ["rtp", 77],
        }
    )
    camera_rows["right"].append({"sequence": 6, "odd_camera_right": "보존"})
    px4_rows["left"].append(
        {
            "sequence": 7,
            "session_elapsed_s": "1.750000000",
            "session_utc_ns": 1_700_000_000_750_000_000,
            "source_utc_ns": 1_700_000_000_700_000_000,
            "odd_px4": {"boot": 123},
        }
    )
    px4_rows["right"].append({"sequence": 8, "odd_px4_right": 9.25})
    gnss_base_rows.append({"sequence": 9, "odd_base": [37.1, 127.2]})
    metadata = {
        "session_id": "session-001",
        "measurement_id": "m000001",
        "session_started_unix_ns": 123,
    }
    write_recovery_metadata(output_dir, metadata)
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
    assert saved.review_error == ""
    assert saved.review_pending is True
    assert saved.review_xlsx == session_root / "review.xlsx"
    assert not saved.review_xlsx.exists()
    assert output_dir.joinpath("data").is_symlink()
    assert {path.name for path in output_dir.joinpath("data").iterdir()} == {
        "camera_timeline.parquet",
        "context",
        "context.json",
        "geometry_comparison.parquet",
        "gnss_base_timeline.parquet",
        "measurement_pairs.parquet",
        "pixhawk_timeline.parquet",
        "raw",
    }
    assert {path.name for path in (output_dir / "data" / "raw").iterdir()} == {
        "camera_left.jsonl",
        "camera_right.jsonl",
        "gnss_base.jsonl",
        "px4_left.jsonl",
        "px4_right.jsonl",
        "stereo_measurements.jsonl",
        "method_evaluation_bundles.jsonl",
        "method_results",
    }
    assert {
        path.name
        for path in (output_dir / "data" / "raw" / "method_results").iterdir()
    } == {f"{method_id}.jsonl" for method_id in MEASUREMENT_METHOD_IDS}
    raw_pair_path = output_dir / "data" / "raw" / "stereo_measurements.jsonl"
    raw_pair_rows = [
        json.loads(line)
        for line in raw_pair_path.read_text(encoding="utf-8").splitlines()
    ]
    assert len(raw_pair_rows) == 1
    terminal_record_id = raw_pair_rows[0].pop("terminal_record_id")
    geometry_context_id = raw_pair_rows[0].pop("geometry_context_id")
    assert terminal_record_id.startswith("legacy:")
    assert len(terminal_record_id) == len("legacy:") + 64
    assert geometry_context_id.startswith("gc:")
    assert raw_pair_rows == [
        {
            "measurement_id": "m000001",
            "measurement_package_id": "m000001",
            "time_sync_epoch_id": None,
            "left_time_sync_epoch_id": None,
            "right_time_sync_epoch_id": None,
            "sample_index": 41,
            "session_elapsed_s": 1.25,
            "session_elapsed_ns": 1_250_000_000,
            "recorded_unix_ns": 1_700_000_000_000_000_123,
            "pair_mid_utc_ns": 1_700_000_000_000_000_100,
            "rtcp_pair_delta_ns": 10_000_000,
            "distance_m": 17.5,
            "measurement_valid": True,
            "odd_pair_private": {"nested": [1, "산"]},
            "odd_native_path": "native/left-0041.png",
        }
    ]
    recovered = read_lossless_rows(saved.measurement_pairs_parquet, "measurement_pairs")
    assert recovered[0]["odd_pair_private"] == {"nested": [1, "산"]}
    assert recovered[0]["odd_native_path"] == "native/left-0041.png"
    assert read_lossless_rows(saved.camera_timeline_parquet, "camera_timeline")[0][
        "side"
    ] == "left"
    pair_table = pq.read_table(saved.measurement_pairs_parquet)
    assert pair_table.column("recording_time_ns").to_pylist() == [1_250_000_000]
    assert pair_table.column("estimated_utc_time_ns").to_pylist() == [
        1_700_000_000_000_000_123
    ]
    camera_table = pq.read_table(saved.camera_timeline_parquet)
    assert camera_table.column("recording_time_ns").to_pylist()[0] == 1_500_000_000
    assert camera_table.column("rtcp_sr_age_ns").to_pylist()[0] == 25_000_000
    px4_table = pq.read_table(saved.pixhawk_timeline_parquet)
    assert px4_table.column("recording_time_ns").to_pylist()[0] == 1_750_000_000
    assert px4_table.column("gps_utc_time_ns").to_pylist()[0] == (
        1_700_000_000_700_000_000
    )
    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    assert manifest["status"] == "finalized"
    assert manifest["authoritative_jsonl_artifacts_verified"] is True
    assert manifest["authoritative_parquet_artifacts_verified"] is False
    assert manifest["parquet_artifacts_verified"] is True
    raw_pair_detail = manifest["artifact_details"]["raw_stereo_measurements"]
    assert raw_pair_detail["authoritative"] is True
    assert raw_pair_detail["row_count"] == 1
    assert raw_pair_detail["size_bytes"] == raw_pair_path.stat().st_size
    assert raw_pair_detail["sha256"] == hashlib.sha256(
        raw_pair_path.read_bytes()
    ).hexdigest()
    assert raw_pair_detail["schema_version"] == (
        recording_session.AUTHORITATIVE_JSONL_SCHEMA_VERSION
    )
    assert manifest["measurement_package_id"] == manifest["measurement_id"]
    descriptor_fields = {
        "role",
        "media_type",
        "relative_path",
        "size_bytes",
        "sha256",
        "row_count",
        "schema_version",
        "authoritative",
        "derived_from",
    }
    assert descriptor_fields <= set(raw_pair_detail)
    for parquet_key in (
        "measurement_pairs",
        "camera_timeline",
        "pixhawk_timeline",
        "gnss_base_timeline",
        "geometry_comparison",
    ):
        assert manifest["artifact_details"][parquet_key]["authoritative"] is False
        assert descriptor_fields <= set(manifest["artifact_details"][parquet_key])
    assert manifest["artifact_details"]["measurement_pairs"]["derived_from"] == [
        raw_pair_detail
    ]
    assert manifest["artifact_details"]["camera_timeline"]["derived_from"] == [
        manifest["artifact_details"]["raw_camera_left"],
        manifest["artifact_details"]["raw_camera_right"],
    ]
    geometry_context_path = output_dir / manifest["resource_files"][
        "geometry_contexts"
    ]
    geometry_context_rows = [
        json.loads(line)
        for line in geometry_context_path.read_text(encoding="utf-8").splitlines()
    ]
    assert [row["geometry_context_id"] for row in geometry_context_rows] == [
        geometry_context_id
    ]
    assert manifest["artifact_details"]["geometry_contexts"]["role"] == (
        "context/geometry_contexts"
    )
    time_sync_path = output_dir / manifest["resource_files"]["time_sync_epochs"]
    assert time_sync_path.read_bytes() == b""
    assert manifest["artifact_details"]["time_sync_epochs"]["row_count"] == 0
    review_status = json.loads(
        (session_root / ".workstation" / "review.json").read_text(encoding="utf-8")
    )
    assert review_status["status"] == "pending_session_close"
    assert rebuild_session_review(session_root, "session-001") == saved.review_xlsx
    assert saved.review_xlsx.is_file()
    generated_review = json.loads(
        (session_root / ".workstation" / "review.json").read_text(
            encoding="utf-8"
        )
    )
    assert generated_review["status"] == "generated"
    assert generated_review["artifact"]["authoritative"] is False
    assert generated_review["artifact"]["relative_path"] == "review.xlsx"
    assert not list(output_dir.glob("*.csv"))
    assert not list(output_dir.glob("*_raw_*rows.jsonl"))
    assert not list(output_dir.glob("*_px4_*.xlsx"))
    assert all(path is not None and not path.exists() for path in spool_paths)
    assert not (output_dir / ".spool" / "session_metadata.json").exists()


def test_parquet_failure_preserves_spools_and_previous_generation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output_dir = tmp_path / "recordings" / "retry-session"
    first = save_session_files(
        output_dir,
        metadata={"session_id": "retry-session"},
        pair_rows=[{"sample_index": 0, "distance_m": 1.0}],
    )
    current = output_dir / ".workstation" / "publish" / "current"
    previous_pointer = current.readlink()
    previous_bytes = first.measurement_pairs_parquet.read_bytes()
    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(output_dir)
    pair_rows.append({"sample_index": 1, "odd_retry": "must-survive"})
    write_recovery_metadata(output_dir, {"session_id": "retry-session"})
    spool_path = pair_rows.spool_path

    def fail_camera(*_args, **_kwargs):
        raise OSError("camera Parquet write fault")

    monkeypatch.setattr(session_parquet, "finalize_camera_timeline", fail_camera)

    with pytest.raises(OSError, match="camera Parquet write fault"):
        save_session_files(
            output_dir,
            metadata={"session_id": "retry-session"},
            pair_rows=pair_rows,
            camera_rows=camera_rows,
            px4_rows=px4_rows,
            gnss_base_rows=gnss_base_rows,
        )

    assert current.readlink() == previous_pointer
    assert first.measurement_pairs_parquet.read_bytes() == previous_bytes
    assert spool_path is not None and spool_path.is_file()
    manifest = json.loads(
        (output_dir / ".workstation" / "save_failure.json").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["status"] == "failed"
    assert manifest["retryable"] is True
    for spool in _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows):
        spool.close()


@pytest.mark.parametrize(
    "fault_target",
    ("_write_authoritative_jsonl", "_validate_authoritative_jsonl"),
)
def test_jsonl_fault_preserves_spools_and_previous_generation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    fault_target: str,
) -> None:
    output_dir = tmp_path / "recordings" / f"jsonl-{fault_target}"
    first = save_session_files(
        output_dir,
        metadata={"session_id": output_dir.name},
        pair_rows=[{"sample_index": 0, "stable": "old"}],
    )
    current = output_dir / ".workstation" / "publish" / "current"
    previous_pointer = current.readlink()
    previous_manifest = first.manifest.read_bytes()
    previous_jsonl = first.authoritative_jsonl_files[0].read_bytes()
    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(
        output_dir
    )
    pair_rows.append({"sample_index": 1, "must_survive": {"retry": True}})
    write_recovery_metadata(output_dir, {"session_id": output_dir.name})
    spool_path = pair_rows.spool_path

    def fail_jsonl(*_args, **_kwargs):
        raise OSError(f"injected {fault_target} fault")

    monkeypatch.setattr(_session_canonical, fault_target, fail_jsonl)
    with pytest.raises(OSError, match=fault_target):
        save_session_files(
            output_dir,
            metadata={"session_id": output_dir.name},
            pair_rows=pair_rows,
            camera_rows=camera_rows,
            px4_rows=px4_rows,
            gnss_base_rows=gnss_base_rows,
        )

    assert current.readlink() == previous_pointer
    assert first.manifest.read_bytes() == previous_manifest
    assert first.authoritative_jsonl_files[0].read_bytes() == previous_jsonl
    assert spool_path is not None and spool_path.is_file()
    for spool in _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows):
        spool.close()


def test_session_review_accumulates_multiple_measurements(tmp_path: Path) -> None:
    session_root = tmp_path / "recordings" / "multi-session"
    first_manifest: Path | None = None
    first_manifest_bytes = b""
    for index, distance in ((1, 10.0), (2, 20.0)):
        measurement_id = f"m{index:06d}"
        saved = save_session_files(
            session_root / "measurements" / measurement_id,
            metadata={
                "session_id": "multi-session",
                "measurement_id": measurement_id,
            },
            pair_rows=[
                {
                    "sample_index": index,
                    "pair_id": f"pair-{index}",
                    "distance_m": distance,
                    "measurement_valid": True,
                }
            ],
        )
        if index == 1:
            first_manifest = saved.manifest
            first_manifest_bytes = saved.manifest.read_bytes()

    assert first_manifest is not None
    assert first_manifest.read_bytes() == first_manifest_bytes
    first_root = session_root / "measurements" / "m000001"
    video = first_root / "video" / "raw_left.mp4"
    video.parent.mkdir(parents=True)
    video.write_bytes(b"video")
    revision = first_root / "revisions" / "r0001"
    revision.mkdir(parents=True)
    package_manifest = revision / "package_manifest.json"
    package_manifest.write_text(
        json.dumps(
            {
                "artifacts": [
                    {
                        "role": "video/left/raw",
                        "relative_path": "video/raw_left.mp4",
                        "status": "present",
                        "required": True,
                        "size_bytes": video.stat().st_size,
                        "sha256": "a" * 64,
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    (first_root / "latest_revision.json").write_text(
        json.dumps({"manifest_path": "revisions/r0001/package_manifest.json"}),
        encoding="utf-8",
    )
    rebuild_session_review(session_root, "multi-session")
    with ZipFile(session_root / "review.xlsx") as archive:
        worksheet_bytes = b"".join(
            archive.read(name)
            for name in archive.namelist()
            if name.startswith("xl/worksheets/sheet") and name.endswith(".xml")
        )
    assert b"m000001" in worksheet_bytes
    assert b"m000002" in worksheet_bytes
    assert b"video/raw_left.mp4" in worksheet_bytes


def test_review_failure_does_not_invalidate_verified_parquet(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output_dir = tmp_path / "recordings" / "report-failure"

    def fail_review(*_args, **_kwargs):
        raise OSError("review generation fault")

    monkeypatch.setattr(_session_review, "_rebuild_session_review", fail_review)
    saved = save_session_files(
        output_dir,
        metadata={"session_id": "report-failure"},
        pair_rows=[{"pair_id": "pair-1", "measurement_valid": True}],
    )

    assert saved.finalized is True
    assert saved.measurement_pairs_parquet.is_file()
    assert saved.review_error == "review generation fault"
    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    assert manifest["status"] == "finalized"
    review_status = json.loads(
        (output_dir / ".workstation" / "review.json").read_text(encoding="utf-8")
    )
    assert review_status["status"] == "failed"
    assert review_status["regenerable"] is True


def test_review_rejects_internally_valid_parquet_that_disagrees_with_manifest(
    tmp_path: Path,
) -> None:
    session_root = tmp_path / "recordings" / "tamper-session"
    first = save_session_files(
        session_root / "measurements" / "m000001",
        metadata={"session_id": "tamper-session", "measurement_id": "m000001"},
        pair_rows=[{"pair_id": "original", "distance_m": 1.0}],
    )
    rebuild_session_review(session_root, "tamper-session")
    prior_review = first.review_xlsx.read_bytes()
    session_parquet.finalize_measurement_pairs(
        first.measurement_pairs_parquet.resolve(),
        [{"pair_id": "rewritten", "distance_m": 999.0}],
    )

    second = save_session_files(
        session_root / "measurements" / "m000002",
        metadata={"session_id": "tamper-session", "measurement_id": "m000002"},
        pair_rows=[{"pair_id": "second", "distance_m": 2.0}],
    )

    assert second.finalized is True
    with pytest.raises(OSError, match="disagrees with measurement manifest"):
        rebuild_session_review(session_root, "tamper-session")
    assert first.review_xlsx.read_bytes() == prior_review


def test_review_reports_finalized_measurement_missing_its_data(tmp_path: Path) -> None:
    session_root = tmp_path / "recordings" / "missing-session"
    session_root.mkdir(parents=True)
    (session_root / "session.json").write_text(
        json.dumps(
            {
                "session_id": "missing-session",
                "state": "OPEN",
                "measurements": [
                    {"measurement_id": "m000001", "state": "SEALED_COMPLETE"}
                ],
            }
        ),
        encoding="utf-8",
    )
    (session_root / "measurements" / "m000001").mkdir(parents=True)

    saved = save_session_files(
        session_root / "measurements" / "m000002",
        metadata={"session_id": "missing-session", "measurement_id": "m000002"},
        pair_rows=[{"pair_id": "second", "distance_m": 2.0}],
    )

    assert saved.finalized is True
    with pytest.raises(OSError, match="m000001"):
        rebuild_session_review(session_root, "missing-session")


def test_non_finite_value_is_rejected_without_discarding_spool(tmp_path: Path) -> None:
    output_dir = tmp_path / "recordings" / "strict-parquet"
    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(output_dir)
    pair_rows.append(
        {"sample_index": 0, "nested": {"odd_non_finite": math.nan}}
    )
    write_recovery_metadata(output_dir, {"session_id": "strict-parquet"})
    spool_path = pair_rows.spool_path

    with pytest.raises(ValueError, match="non-finite"):
        save_session_files(
            output_dir,
            metadata={"session_id": "strict-parquet"},
            pair_rows=pair_rows,
            camera_rows=camera_rows,
            px4_rows=px4_rows,
            gnss_base_rows=gnss_base_rows,
        )

    assert spool_path is not None and spool_path.is_file()
    manifest = json.loads(
        (output_dir / ".workstation" / "save_failure.json").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["status"] == "failed"
    for spool in _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows):
        spool.close()


def test_terminal_record_identity_is_retry_stable_and_duplicates_are_rejected(
    tmp_path: Path,
) -> None:
    output_dir = tmp_path / "recordings" / "terminal-identity"
    row = {"pair_id": "legacy-source", "future": {"kept": True}}

    first = save_session_files(
        output_dir,
        metadata={"session_id": output_dir.name},
        pair_rows=[row],
    )
    first_row = json.loads(
        first.authoritative_jsonl_files[0].read_text(encoding="utf-8")
    )
    second = save_session_files(
        output_dir,
        metadata={"session_id": output_dir.name},
        pair_rows=[row],
    )
    second_row = json.loads(
        second.authoritative_jsonl_files[0].read_text(encoding="utf-8")
    )

    assert first_row["terminal_record_id"] == second_row["terminal_record_id"]
    assert first_row["measurement_package_id"] == output_dir.name
    with pytest.raises(ValueError, match="duplicate terminal_record_id"):
        save_session_files(
            tmp_path / "recordings" / "terminal-duplicate",
            metadata={"session_id": "terminal-duplicate"},
            pair_rows=[row, row],
        )


@pytest.mark.parametrize(
    "metadata",
    (
        {"session_id": "wrong-session", "measurement_id": "m000001"},
        {"session_id": "owner-session", "measurement_id": "m999999"},
    ),
)
def test_save_rejects_metadata_identity_that_does_not_own_output_path(
    tmp_path: Path,
    metadata: dict[str, object],
) -> None:
    output = tmp_path / "recordings" / "owner-session" / "measurements" / "m000001"

    with pytest.raises(ValueError, match="does not own"):
        save_session_files(output, metadata=metadata, pair_rows=[])

    assert not output.exists()


def test_publish_failure_keeps_old_data_and_retry_spool(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output_dir = tmp_path / "recordings" / "publish-retry"
    first = save_session_files(
        output_dir,
        metadata={"session_id": "publish-retry"},
        pair_rows=[{"sample_index": 0, "distance_m": 2.0}],
    )
    current = output_dir / ".workstation" / "publish" / "current"
    previous_pointer = current.readlink()
    previous_bytes = first.measurement_pairs_parquet.read_bytes()
    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(output_dir)
    pair_rows.append({"sample_index": 1, "odd_new_generation": 3})
    write_recovery_metadata(output_dir, {"session_id": "publish-retry"})
    spool_path = pair_rows.spool_path

    def fail_publish(*_args, **_kwargs) -> None:
        raise OSError("current pointer fault")

    monkeypatch.setattr(_session_generation, "_publish_v2_generation", fail_publish)
    with pytest.raises(OSError, match="current pointer fault"):
        save_session_files(
            output_dir,
            metadata={"session_id": "publish-retry"},
            pair_rows=pair_rows,
            camera_rows=camera_rows,
            px4_rows=px4_rows,
            gnss_base_rows=gnss_base_rows,
        )

    assert current.readlink() == previous_pointer
    assert first.measurement_pairs_parquet.read_bytes() == previous_bytes
    assert spool_path is not None and spool_path.is_file()
    for spool in _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows):
        spool.close()


def test_published_generation_validation_failure_rolls_back_and_keeps_spool(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output_dir = tmp_path / "recordings" / "published-validation-retry"
    first = save_session_files(
        output_dir,
        metadata={"session_id": output_dir.name},
        pair_rows=[{"sample_index": 0, "stable": "old"}],
    )
    current = output_dir / ".workstation" / "publish" / "current"
    previous_pointer = current.readlink()
    previous_manifest = first.manifest.read_bytes()
    previous_jsonl = first.authoritative_jsonl_files[0].read_bytes()
    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(
        output_dir
    )
    pair_rows.append({"sample_index": 1, "must_survive": "validation"})
    write_recovery_metadata(output_dir, {"session_id": output_dir.name})
    spool_path = pair_rows.spool_path

    def fail_published_validation(*_args, **_kwargs) -> None:
        raise OSError("injected published validation fault")

    monkeypatch.setattr(
        _session_generation,
        "_validate_published_v2_generation",
        fail_published_validation,
    )
    with pytest.raises(OSError, match="published validation fault"):
        save_session_files(
            output_dir,
            metadata={"session_id": output_dir.name},
            pair_rows=pair_rows,
            camera_rows=camera_rows,
            px4_rows=px4_rows,
            gnss_base_rows=gnss_base_rows,
        )

    assert current.readlink() == previous_pointer
    assert first.manifest.read_bytes() == previous_manifest
    assert first.authoritative_jsonl_files[0].read_bytes() == previous_jsonl
    assert spool_path is not None and spool_path.is_file()
    for spool in _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows):
        spool.close()


def test_generation_manifest_failure_keeps_previous_manifest_data_and_spool(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output_dir = tmp_path / "recordings" / "manifest-retry"
    first = save_session_files(
        output_dir,
        metadata={"session_id": "manifest-retry"},
        pair_rows=[{"pair_id": "old", "distance_m": 1.0}],
    )
    previous_manifest = first.manifest.read_bytes()
    previous_data = first.measurement_pairs_parquet.read_bytes()
    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(output_dir)
    pair_rows.append({"pair_id": "new", "distance_m": 2.0})
    write_recovery_metadata(output_dir, {"session_id": "manifest-retry"})
    spool_path = pair_rows.spool_path
    original_write = recording_session._write_manifest

    def fail_generation_manifest(path: Path, payload: dict[str, object]) -> None:
        if path.name == "measurement.json" and path.parent.name.endswith(".preparing"):
            raise OSError("injected generation manifest fault")
        original_write(path, payload)

    monkeypatch.setattr(_session_manifest, "_write_manifest", fail_generation_manifest)
    with pytest.raises(OSError, match="generation manifest fault"):
        save_session_files(
            output_dir,
            metadata={"session_id": "manifest-retry"},
            pair_rows=pair_rows,
            camera_rows=camera_rows,
            px4_rows=px4_rows,
            gnss_base_rows=gnss_base_rows,
        )

    assert first.manifest.read_bytes() == previous_manifest
    assert first.measurement_pairs_parquet.read_bytes() == previous_data
    assert spool_path is not None and spool_path.is_file()
    assert (output_dir / ".workstation" / "save_failure.json").is_file()
    for spool in _all_spools(pair_rows, camera_rows, px4_rows, gnss_base_rows):
        spool.close()


def test_post_publish_spool_cleanup_failure_is_nonfatal_and_keeps_full_rows(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    output_dir = tmp_path / "recordings" / "cleanup-failure"
    pair_rows, camera_rows, px4_rows, gnss_base_rows = _bound_source_spools(output_dir)
    for sequence in range(300):
        camera_rows["left"].append({"sequence": sequence})
    protected = camera_rows["left"].spool_path
    assert protected is not None
    original_unlink = Path.unlink

    def guarded_unlink(path: Path, *args, **kwargs):
        if path == protected:
            raise PermissionError("injected cleanup denial")
        return original_unlink(path, *args, **kwargs)

    monkeypatch.setattr(Path, "unlink", guarded_unlink)
    saved = save_session_files(
        output_dir,
        metadata={"session_id": "cleanup-failure"},
        pair_rows=pair_rows,
        camera_rows=camera_rows,
        px4_rows=px4_rows,
        gnss_base_rows=gnss_base_rows,
    )

    assert saved.finalized is True
    assert camera_rows["left"].spool_path == protected
    assert protected.is_file()
    assert len(list(camera_rows["left"])) == 300


def test_retryable_loader_accepts_legacy_spool_without_manifest(tmp_path: Path) -> None:
    output_dir = tmp_path / "recordings" / "sessions" / "legacy-retry"
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


def test_v4_save_keeps_authoritative_app_delta_and_record_only_ntp_distinct(
    tmp_path: Path,
) -> None:
    output = tmp_path / "recordings" / "v4-app-time"
    saved = save_session_files(
        output,
        metadata={"session_id": output.name},
        pair_rows=[
            {
                "schema_version": "stereowiris-paired-frame-v4",
                "pair_id": "pair-v4",
                "pairing_time_basis": "app_receive_monotonic",
                "pairing_time_basis_role": "authoritative_distance",
                "left_app_receive_monotonic_ns": 1_000_000_000,
                "right_app_receive_monotonic_ns": 1_020_000_000,
                "app_receive_pair_delta_ns": 20_000_000,
                "selected_pair_delta_ns": 20_000_000,
                "selected_pair_abs_delta_ns": 20_000_000,
                "sync_delta_ns": 20_000_000,
                "left_rtcp_frame_ntp_unix_ns": 100_000_000_000,
                "right_rtcp_frame_ntp_unix_ns": 32_500_000_000_000,
                "rtcp_pair_delta_ns": 32_400_000_000_000,
                "ntp_evidence_role": "record_only",
                "measurement_valid": True,
            }
        ],
    )

    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    raw_path = output / "data" / "raw" / "stereo_measurements.jsonl"
    [raw] = [
        json.loads(line)
        for line in raw_path.read_text(encoding="utf-8").splitlines()
    ]
    table = pq.read_table(saved.measurement_pairs_parquet)

    assert manifest["schema"] == "workstation-measurement-storage-v4"
    assert manifest["schema_version"] == 4
    assert raw["selected_pair_delta_ns"] == 20_000_000
    assert raw["rtcp_pair_delta_ns"] == 32_400_000_000_000
    assert raw["ntp_evidence_role"] == "record_only"
    assert table.column("sync_delta_ns").to_pylist() == [20_000_000]
    assert table.column("rtcp_pair_delta_ns").to_pylist() == [
        32_400_000_000_000
    ]
