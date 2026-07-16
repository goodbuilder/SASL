"""Adversarial Phase A integrity and lifecycle tests."""

import json
import shutil
import subprocess
from pathlib import Path

import pyarrow.parquet as pq
import pytest

import wiris_stereo_distance.session_recording_parquet as recording_module
from wiris_stereo_distance.session_recording_parquet import (
    PixhawkSampleRecord,
    SchemaValidationError,
    SessionRecordingConfig,
    SessionRecorder,
    SourceRecord,
    TimeAlignmentRecord,
    TriangulationRecord,
    VideoArtifact,
    VideoMapping,
    validate_int64,
    intersection_angle_from_rays,
)
from wiris_stereo_distance.session_video import MkvWriterAdapter, MkvWriterError


def _fixture(tmp_path: Path, *, frames: int = 1) -> Path:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        pytest.skip("ffmpeg is unavailable")
    output = tmp_path / f"fixture-{frames}.mp4"
    command = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "lavfi",
        "-i",
        "testsrc=size=32x24:rate=2",
        "-frames:v",
        str(frames),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(output),
    ]
    try:
        subprocess.run(command, check=True, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.CalledProcessError) as exc:
        pytest.skip(f"H.264 fixture unavailable: {exc}")
    return output


def _source(session_id: str, side: str, frame: int = 0, *, status: str = "pending", drop_reason: str | None = None) -> SourceRecord:
    return SourceRecord(
        source_record_id=f"{side}-source-{frame}",
        session_id=session_id,
        camera_side=side,
        camera_stream_id=f"stream-{side}",
        frame_id=frame,
        camera_sequence=frame,
        pc_receive_monotonic_ns=1_000 + frame,
        width_px=32,
        height_px=24,
        video_mapping_status=status,
        drop_reason=drop_reason,
        stored_in_video=False,
    )


def _recorder(tmp_path: Path, session_id: str = "integrity") -> SessionRecorder:
    return SessionRecorder.open(SessionRecordingConfig(root=tmp_path / "recordings", session_id=session_id))


def _complete_session(tmp_path: Path, session_id: str = "complete") -> SessionRecorder:
    fixture = _fixture(tmp_path)
    recorder = _recorder(tmp_path, session_id)
    for side in ("left", "right"):
        source = _source(session_id, side)
        recorder.record_camera_source(source)
        recorder.mark_video_mapping(
            VideoMapping(
                source_record_id=source.source_record_id,
                camera_side=side,
                stored_in_video=True,
                video_mapping_status="stored",
                video_frame_index=0,
            )
        )
        writer = MkvWriterAdapter(side, recorder.paths.video_path(side))
        writer.remux_fixture(fixture)
        writer.close()
        recorder.register_video_artifact(VideoArtifact(side, frame_count=999))
    return recorder


def _attitude(session_id: str, sample_id: str, side: str, *, slot: str = "vehicle_attitude") -> PixhawkSampleRecord:
    return PixhawkSampleRecord(
        sample_id=sample_id,
        session_id=session_id,
        pixhawk_id=f"pixhawk_{side}",
        camera_side=side,
        transport="synthetic",
        topic=f"/{side}/{slot}",
        message_type="px4_msgs/msg/VehicleAttitude",
        source_slot=slot,
        sample_sequence=0,
        pc_receive_monotonic_ns=100,
        attitude_q_w=1.0,
        attitude_q_x=0.0,
        attitude_q_y=0.0,
        attitude_q_z=0.0,
    )


def test_strict_int64_rejects_float_bool_and_overflow_and_accepts_nullable_none() -> None:
    for value in (123.0, 123.9, True, "123", 1 << 63, -(1 << 63) - 1):
        with pytest.raises(SchemaValidationError):
            validate_int64("timestamp", value)
    assert validate_int64("optional_timestamp", None) is None
    assert validate_int64("timestamp", 123) == 123


def test_started_timestamps_are_strict_int64(tmp_path: Path) -> None:
    with pytest.raises(SchemaValidationError):
        SessionRecordingConfig(root=tmp_path, session_id="bad-start", started_monotonic_ns=1.0)
    with pytest.raises(SchemaValidationError):
        SessionRecordingConfig(root=tmp_path, session_id="bad-start-2", started_utc_ns=True)


@pytest.mark.parametrize("field_name", ["mavlink_time_usec", "px4_time_utc_usec", "px4_timestamp_us", "device_id"])
def test_pixhawk_int64_fields_do_not_allow_implicit_truncation(tmp_path: Path, field_name: str) -> None:
    recorder = _recorder(tmp_path, f"strict-{field_name}")
    kwargs = {field_name: 123.9}
    sample = PixhawkSampleRecord(
        sample_id="strict-sample",
        session_id=f"strict-{field_name}",
        pixhawk_id="pixhawk_left",
        camera_side="left",
        transport="synthetic",
        topic="/left/sample",
        message_type="px4_msgs/msg/SensorGps",
        source_slot="sensor_gps_0",
        sample_sequence=0,
        pc_receive_monotonic_ns=1,
        **kwargs,
    )
    with pytest.raises(SchemaValidationError):
        recorder.record_pixhawk_sample(sample)
    recorder.close()


def test_intersection_angle_uses_finite_nonzero_rays_and_clamped_dot() -> None:
    assert intersection_angle_from_rays((1.0, 0.0, 0.0), (1.0, 0.0, 0.0)) == pytest.approx(0.0)
    with pytest.raises(SchemaValidationError):
        intersection_angle_from_rays((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    with pytest.raises(SchemaValidationError):
        intersection_angle_from_rays((float("nan"), 0.0, 0.0), (1.0, 0.0, 0.0))


def test_missing_artifact_can_never_be_complete(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path, "missing-artifact")
    result = recorder.close()
    metadata = json.loads(result.paths.metadata.read_text(encoding="utf-8"))
    assert result.session_state == "incomplete"
    assert metadata["clean_shutdown"] is False
    assert len(result.paths.manifest) == 7
    assert not result.paths.left_video.exists()


def test_complete_session_has_exactly_seven_artifacts_and_authoritative_counts(tmp_path: Path) -> None:
    recorder = _complete_session(tmp_path)
    result = recorder.close()
    actual = sorted(path.relative_to(result.paths.session_dir).as_posix() for path in result.paths.session_dir.rglob("*") if path.is_file())
    assert result.session_state == "complete"
    assert actual == sorted(result.paths.manifest)
    assert json.loads(result.paths.metadata.read_text(encoding="utf-8"))["camera"]["left"]["frame_count"] == 1


def test_actual_mkv_frame_count_must_be_fully_explained_by_stored_mapping(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path, frames=2)
    recorder = _recorder(tmp_path, "frame-count-mismatch")
    for side in ("left", "right"):
        source = _source("frame-count-mismatch", side)
        recorder.record_camera_source(source)
        recorder.mark_video_mapping(VideoMapping(source.source_record_id, True, "stored", 0, camera_side=side))
        writer = MkvWriterAdapter(side, recorder.paths.video_path(side))
        writer.remux_fixture(fixture)
        writer.close()
        recorder.register_video_artifact(VideoArtifact(side, frame_count=1))
    result = recorder.close()
    assert result.session_state == "incomplete"


def test_orphan_pending_mapping_is_incomplete(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path, "orphan-mapping")
    recorder.mark_video_mapping(
        VideoMapping(
            source_record_id="never-recorded",
            camera_side="left",
            stored_in_video=True,
            video_mapping_status="stored",
            video_frame_index=0,
        )
    )
    result = recorder.close()
    assert result.session_state == "incomplete"


def test_dropped_and_writer_failure_contracts_are_explicit(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path, "mapping-contract")
    with pytest.raises(SchemaValidationError):
        recorder.record_camera_source(_source("mapping-contract", "left", status="dropped"))
    recorder.record_camera_source(_source("mapping-contract", "left", status="dropped", drop_reason="queue_full"))
    recorder.record_camera_source(_source("mapping-contract", "right", status="writer_failure", drop_reason="encoder_error"))
    result = recorder.close()
    assert result.session_state == "incomplete"


def test_wrong_side_duplicate_and_out_of_range_video_mappings_are_rejected(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path, "mapping-errors")
    left = _source("mapping-errors", "left")
    right = _source("mapping-errors", "right")
    right_second = _source("mapping-errors", "right", 1)
    recorder.record_camera_source(left)
    recorder.record_camera_source(right)
    recorder.record_camera_source(right_second)
    with pytest.raises(SchemaValidationError):
        recorder.mark_video_mapping(VideoMapping(left.source_record_id, True, "stored", 0, camera_side="right"))
    recorder.mark_video_mapping(VideoMapping(left.source_record_id, True, "stored", 0, camera_side="left"))
    recorder.mark_video_mapping(VideoMapping(right.source_record_id, True, "stored", 0, camera_side="right"))
    with pytest.raises(SchemaValidationError):
        recorder.mark_video_mapping(VideoMapping(right_second.source_record_id, True, "stored", 0, camera_side="right"))
    recorder.close()

    fixture = _fixture(tmp_path)
    writer = MkvWriterAdapter(
        "left",
        tmp_path / "out.mkv",
        source_record_resolver=lambda source_id: {"camera_side": "left", "frame_id": 0}
        if source_id == "left-source"
        else None,
    )
    writer.remux_fixture(fixture)
    with pytest.raises(MkvWriterError):
        writer.mark_mapping(VideoMapping("left-source", True, "stored", 1, camera_side="left"))
    writer.close()


def test_metadata_finalize_failure_is_retryable_without_second_writer_close(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    recorder = _recorder(tmp_path, "metadata-retry")
    original = recording_module._atomic_json_write
    calls = {"count": 0}

    def fail_once(path: Path, payload: dict) -> None:
        calls["count"] += 1
        if calls["count"] == 1:
            raise OSError("injected metadata failure")
        original(path, payload)

    monkeypatch.setattr(recording_module, "_atomic_json_write", fail_once)
    with pytest.raises(Exception):
        recorder.close()
    assert recorder.state == "metadata_finalize_failed"
    assert recorder.closed is False
    assert json.loads(recorder.paths.metadata.read_text(encoding="utf-8"))["session_state"] == "recording"
    result = recorder.close()
    assert result.session_state == "incomplete"
    assert result.clean_shutdown is False


def test_complete_metadata_failure_retains_index_and_retry_does_not_repeat_writes(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    remux_calls = {"count": 0}
    original_remux = MkvWriterAdapter.remux_fixture

    def tracked_remux(self: MkvWriterAdapter, fixture: Path):
        remux_calls["count"] += 1
        return original_remux(self, fixture)

    monkeypatch.setattr(MkvWriterAdapter, "remux_fixture", tracked_remux)
    recorder = _complete_session(tmp_path, "complete-metadata-retry")
    assert remux_calls["count"] == 2

    close_calls = {"count": 0}
    original_close = recorder._close_writers_locked

    def tracked_close() -> None:
        close_calls["count"] += 1
        original_close()

    monkeypatch.setattr(recorder, "_close_writers_locked", tracked_close)
    atomic_write = recording_module._atomic_json_write
    calls = {"count": 0}

    def fail_once(path: Path, payload: dict) -> None:
        calls["count"] += 1
        if calls["count"] == 1:
            raise OSError("injected complete metadata failure")
        atomic_write(path, payload)

    monkeypatch.setattr(recording_module, "_atomic_json_write", fail_once)
    with pytest.raises(Exception):
        recorder.close()
    assert recorder.state == "metadata_finalize_failed"
    assert recorder.closed is False
    assert recorder.paths.integrity_index.exists()
    counts_after_failure = dict(recorder._counts)
    assert close_calls["count"] == 1
    assert remux_calls["count"] == 2

    result = recorder.close()
    metadata = json.loads(result.paths.metadata.read_text(encoding="utf-8"))
    assert result.session_state == "complete"
    assert metadata["session_state"] == "complete"
    assert result.paths.integrity_index.exists() is False
    assert recorder._counts == counts_after_failure
    assert close_calls["count"] == 1
    assert remux_calls["count"] == 2


def test_integrity_index_cleanup_failure_is_not_reported_as_complete(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    recorder = _complete_session(tmp_path, "cleanup-failure")

    def fail_cleanup() -> bool:
        raise OSError("injected index cleanup failure")

    monkeypatch.setattr(recorder, "_remove_integrity_index_locked", fail_cleanup)
    result = recorder.close()
    metadata = json.loads(result.paths.metadata.read_text(encoding="utf-8"))
    assert result.session_state == "incomplete"
    assert result.clean_shutdown is False
    assert metadata["session_state"] == "incomplete"
    assert metadata["failure_stage"] == "integrity_index_cleanup"
    assert result.paths.integrity_index.exists()


def test_final_validation_uses_parquet_metadata_without_read_table(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    recorder = _complete_session(tmp_path, "parquet-metadata-validation")

    def fail_read_table(*_args: object, **_kwargs: object) -> None:
        raise AssertionError("final validation must not materialize a complete Parquet table")

    monkeypatch.setattr(recording_module.pq, "read_table", fail_read_table)
    result = recorder.close()
    assert result.session_state == "complete"
    assert recording_module.pq.ParquetFile(result.paths.source_records).metadata.num_rows == 2


def test_pending_source_flush_uses_bounded_fetchmany(tmp_path: Path) -> None:
    recorder = SessionRecorder.open(
        SessionRecordingConfig(root=tmp_path / "recordings", session_id="bounded-fetch", source_batch_size=256)
    )
    for index in range(1000):
        recorder.record_camera_source(_source("bounded-fetch", "left", index))

    real_connection = recorder._index
    assert real_connection is not None
    fetch_sizes: list[int] = []

    class CursorProxy:
        def __init__(self, cursor: object) -> None:
            self._cursor = cursor

        def fetchall(self) -> object:
            raise AssertionError("source flush must not call SQLite fetchall")

        def fetchmany(self, size: int) -> object:
            fetch_sizes.append(size)
            assert size <= 256
            return self._cursor.fetchmany(size)  # type: ignore[attr-defined]

        def __getattr__(self, name: str) -> object:
            return getattr(self._cursor, name)

    class ConnectionProxy:
        def execute(self, sql: str, *parameters: object) -> object:
            cursor = real_connection.execute(sql, *parameters)
            if sql.startswith("SELECT source_record_id,record_json"):
                return CursorProxy(cursor)
            return cursor

        def __getattr__(self, name: str) -> object:
            return getattr(real_connection, name)

    recorder._index = ConnectionProxy()  # type: ignore[assignment]
    result = recorder.close()
    assert result.session_state == "incomplete"
    assert len(fetch_sizes) >= 4
    assert max(fetch_sizes) <= 256
    assert pq.ParquetFile(result.paths.source_records).metadata.num_rows == 1000
    assert recorder._source_batch == []


def test_open_failure_closes_already_opened_writer_and_preserves_incomplete_metadata(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    original = recording_module.pq.ParquetWriter
    closed: list[str] = []
    calls = {"count": 0}

    class TrackingWriter:
        def __init__(self, path: str, schema: object, **_: object) -> None:
            self.schema = schema
            self.path = path

        def close(self) -> None:
            closed.append(self.path)

    def injected(path: str, schema: object, **kwargs: object) -> object:
        calls["count"] += 1
        if calls["count"] == 2:
            raise OSError("injected writer open failure")
        return TrackingWriter(path, schema, **kwargs)

    monkeypatch.setattr(recording_module.pq, "ParquetWriter", injected)
    with pytest.raises(Exception):
        SessionRecorder.open(SessionRecordingConfig(root=tmp_path, session_id="open-failure"))
    metadata_path = tmp_path / "session_open-failure" / "session_metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    assert metadata["session_state"] == "open_failed"
    assert metadata["incomplete"] is True
    assert metadata["failure_stage"] == "open_time_alignment"
    assert len(closed) == 1


def test_association_rejects_swapped_sources_frame_mismatch_and_wrong_pose(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path, "association")
    recorder.record_camera_source(_source("association", "left", 0))
    recorder.record_camera_source(_source("association", "right", 0))
    recorder.record_pixhawk_sample(_attitude("association", "pose-left", "left"))
    recorder.record_pixhawk_sample(_attitude("association", "pose-right", "right"))

    def tri(**kwargs: object) -> TriangulationRecord:
        return TriangulationRecord(
            triangulation_record_id=kwargs.pop("triangulation_record_id", "tri"),
            session_id="association",
            triangulation_valid=False,
            triangulation_status="invalid",
            left_source_record_id=kwargs.pop("left_source_record_id", "left-source-0"),
            right_source_record_id=kwargs.pop("right_source_record_id", "right-source-0"),
            left_frame_id=kwargs.pop("left_frame_id", 0),
            right_frame_id=kwargs.pop("right_frame_id", 0),
            left_pose_sample_id=kwargs.pop("left_pose_sample_id", "pose-left"),
            right_pose_sample_id=kwargs.pop("right_pose_sample_id", "pose-right"),
            **kwargs,
        )

    with pytest.raises(SchemaValidationError):
        recorder.record_triangulation(tri(left_source_record_id="right-source-0"))
    with pytest.raises(SchemaValidationError):
        recorder.record_triangulation(tri(left_frame_id=99))
    with pytest.raises(SchemaValidationError):
        recorder.record_triangulation(tri(left_pose_sample_id="pose-right"))
    with pytest.raises(SchemaValidationError):
        recorder.record_triangulation(tri(left_pose_sample_id="pose-left", right_pose_sample_id="pose-left"))
    recorder.close()


@pytest.mark.parametrize("kind", ["source", "alignment", "pixhawk", "triangulation"])
def test_each_primary_key_rejects_duplicate_and_keeps_one_row(tmp_path: Path, kind: str) -> None:
    session_id = f"duplicate-{kind}"
    recorder = _recorder(tmp_path, session_id)
    if kind == "source":
        row = _source(session_id, "left", status="dropped", drop_reason="test")
        recorder.record_camera_source(row)
        with pytest.raises(SchemaValidationError):
            recorder.record_camera_source(row)
        table_path = recorder.paths.source_records
    elif kind == "alignment":
        row = TimeAlignmentRecord("alignment-1", "camera", "source-1", "pc_receive_monotonic", "receive_time_fallback", 1, 1)
        recorder.record_time_alignment(row)
        with pytest.raises(SchemaValidationError):
            recorder.record_time_alignment(row)
        table_path = recorder.paths.time_alignment
    elif kind == "pixhawk":
        row = _attitude(session_id, "sample-1", "left")
        recorder.record_pixhawk_sample(row)
        with pytest.raises(SchemaValidationError):
            recorder.record_pixhawk_sample(row)
        table_path = recorder.paths.pixhawk_samples
    else:
        row = TriangulationRecord("tri-1", session_id, False, "invalid")
        recorder.record_triangulation(row)
        with pytest.raises(SchemaValidationError):
            recorder.record_triangulation(row)
        table_path = recorder.paths.triangulation
    recorder.close()
    assert pq.read_table(table_path).num_rows == 1


def test_source_rows_are_streamed_in_bounded_batches_and_partial_index_is_retained(tmp_path: Path) -> None:
    recorder = SessionRecorder.open(
        SessionRecordingConfig(root=tmp_path / "recordings", session_id="streaming", source_batch_size=2)
    )
    for index in range(5):
        recorder.record_camera_source(_source("streaming", "left", index, status="dropped", drop_reason="synthetic_drop"))
    assert len(recorder._source_batch) <= 1
    assert not hasattr(recorder, "_source_records")
    # Parquet readers require a footer, which ParquetWriter writes at close;
    # the flushed row groups are nevertheless already present on disk.
    assert recorder.paths.source_records.stat().st_size > 0
    result = recorder.close()
    assert pq.read_table(result.paths.source_records).num_rows == 5
    assert result.paths.integrity_index.exists()
