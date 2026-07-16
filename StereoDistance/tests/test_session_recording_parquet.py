import json
import shutil
import subprocess
from dataclasses import replace
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq
import pytest

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
    map_optical_rdf_to_stereo_frd,
)
from wiris_stereo_distance.session_video import MkvWriterAdapter


def _make_h264_fixture(tmp_path: Path) -> Path:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        pytest.skip("ffmpeg is unavailable; real MKV integration fixture cannot run")
    fixture = tmp_path / "fixture.mp4"
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
        "1",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(fixture),
    ]
    try:
        subprocess.run(command, check=True, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.CalledProcessError) as exc:
        pytest.skip(f"local H.264 fixture could not be generated: {exc}")
    return fixture


def _source(session_id: str, side: str, sequence: int) -> SourceRecord:
    return SourceRecord(
        source_record_id=f"{side}-source-{sequence}",
        session_id=session_id,
        camera_side=side,
        camera_stream_id=f"wiris-{side}",
        frame_id=sequence,
        camera_sequence=sequence,
        pc_receive_monotonic_ns=1_000_000_000 + sequence * 50_000_000,
        pc_receive_utc_ns=1_700_000_000_000_000_000 + sequence * 50_000_000,
        rtp_timestamp=90_000 + sequence * 4_500,
        rtp_ssrc=100 + sequence,
        rtp_marker_seq=sequence,
        rtcp_sr_ntp_raw=1234 + sequence,
        rtcp_frame_ntp_unix_ns=1_700_000_000_000_000_000 + sequence * 50_000_000,
        pts_ticks=sequence * 45_000,
        dts_ticks=sequence * 45_000,
        time_base_num=1,
        time_base_den=90_000,
        width_px=32,
        height_px=24,
    )


def _add_video(recorder: SessionRecorder, fixture: Path, side: str, source_id: str) -> None:
    writer = MkvWriterAdapter(
        side,
        recorder.paths.video_path(side),
        source_record_resolver=recorder.resolve_source_record,
    )
    result = writer.remux_fixture(fixture)
    writer.mark_mapping(
        VideoMapping(
            source_record_id=source_id,
            stored_in_video=True,
            video_mapping_status="stored",
            video_frame_index=0,
            video_packet_pts_ticks=result.packets[0].pts_ticks if result.packets else 0,
            video_packet_dts_ticks=result.packets[0].dts_ticks if result.packets else 0,
            source_frame_id=0,
        )
    )
    writer.close()
    recorder.register_video_artifact(VideoArtifact(side, frame_count=result.frame_count))


def test_session_recorder_creates_exactly_seven_artifacts_and_typed_schema(tmp_path: Path) -> None:
    fixture = _make_h264_fixture(tmp_path)
    session_id = "synthetic-0001"
    recorder = SessionRecorder.open(
        SessionRecordingConfig(root=tmp_path / "recordings", session_id=session_id)
    )
    left = _source(session_id, "left", 0)
    right = _source(session_id, "right", 0)
    recorder.record_camera_source(left)
    recorder.record_camera_source(right)
    recorder.mark_video_mapping(
        VideoMapping(
            source_record_id=left.source_record_id,
            stored_in_video=True,
            video_mapping_status="stored",
            video_frame_index=0,
            video_packet_pts_ticks=0,
            video_packet_dts_ticks=0,
        )
    )
    recorder.mark_video_mapping(
        VideoMapping(
            source_record_id=right.source_record_id,
            stored_in_video=True,
            video_mapping_status="stored",
            video_frame_index=0,
            video_packet_pts_ticks=0,
            video_packet_dts_ticks=0,
        )
    )
    recorder.record_time_alignment(
        TimeAlignmentRecord(
            alignment_id="align-left-0",
            record_type="camera",
            record_id=left.source_record_id,
            camera_side="left",
            alignment_reference_domain="pc_receive_monotonic",
            alignment_status="receive_time_fallback",
            aligned_pc_monotonic_ns=left.pc_receive_monotonic_ns,
            pc_receive_monotonic_ns=left.pc_receive_monotonic_ns,
            pc_receive_utc_ns=left.pc_receive_utc_ns,
            camera_rtp_timestamp=left.rtp_timestamp,
            camera_pts_ticks=left.pts_ticks,
            camera_dts_ticks=left.dts_ticks,
        )
    )
    recorder.record_pixhawk_sample(
        PixhawkSampleRecord(
            sample_id="left-attitude-0",
            session_id=session_id,
            pixhawk_id="pixhawk_left",
            camera_side="left",
            transport="ros2",
            namespace="/uav_left",
            topic="/uav_left/fmu/out/vehicle_attitude",
            message_type="px4_msgs/msg/VehicleAttitude",
            source_slot="vehicle_attitude",
            sample_sequence=0,
            pc_receive_monotonic_ns=left.pc_receive_monotonic_ns,
            px4_timestamp_us=500_000,
            px4_timestamp_sample_us=499_000,
            attitude_q_w=1.0,
            attitude_q_x=0.0,
            attitude_q_y=0.0,
            attitude_q_z=0.0,
            raw_fields={"timestamp": 500_000, "timestamp_sample": 499_000, "q": [1.0, 0.0, 0.0, 0.0]},
        )
    )
    recorder.record_triangulation(
        TriangulationRecord(
            triangulation_record_id="tri-0",
            session_id=session_id,
            triangulation_valid=True,
            triangulation_status="valid",
            left_source_record_id=left.source_record_id,
            right_source_record_id=right.source_record_id,
            left_frame_id=left.frame_id,
            right_frame_id=right.frame_id,
            pair_timestamp_basis="pc_receive_monotonic",
            pair_timestamp_delta_ns=1_000,
            left_pose_sample_id="left-attitude-0",
            stereo_extrinsics_source="checkerboard_stereo_calibration",
            stereo_extrinsics_frame="left_camera_opencv_optical",
            stereo_extrinsics_axis_contract="right_camera_in_left_camera_rdf:right,down,forward",
            left_bbox_center_u_px=10.0,
            left_bbox_center_v_px=12.0,
            right_bbox_center_u_px=9.0,
            right_bbox_center_v_px=12.0,
            left_confidence=0.9,
            right_confidence=0.88,
            class_id=1,
            class_name="target",
            stereo_center_optical_m=(2.0, 3.0, 10.0),
            stereo_range_m=10.63,
            ray_gap_m=0.001,
            intersection_angle_rad=0.1,
            baseline_m=0.2,
            triangulation_source="checkerboard_stereo_calibration",
            triangulation_method="closest-rays",
            warnings=(),
        )
    )
    _add_video(recorder, fixture, "left", left.source_record_id)
    _add_video(recorder, fixture, "right", right.source_record_id)

    result = recorder.close()

    assert result.session_state == "complete"
    assert result.clean_shutdown is True
    assert result.incomplete is False
    assert result.counts == {
        "source_records": 2,
        "time_alignment": 1,
        "pixhawk_samples": 1,
        "triangulation": 1,
    }
    actual_files = sorted(
        path.relative_to(result.paths.session_dir).as_posix()
        for path in result.paths.session_dir.rglob("*")
        if path.is_file()
    )
    assert actual_files == sorted(result.paths.manifest)

    metadata = json.loads(result.paths.metadata.read_text(encoding="utf-8"))
    assert metadata["file_manifest"] == list(result.paths.manifest)
    assert metadata["clean_shutdown"] is True
    assert metadata["incomplete"] is False
    assert metadata["camera"]["left"]["mapping_missing_count"] == 0
    assert metadata["camera"]["right"]["mapping_missing_count"] == 0
    assert metadata["camera"]["left"]["decoded_frame_count"] == 1
    assert metadata["camera"]["right"]["decoded_frame_count"] == 1
    assert metadata["camera"]["left"]["packet_count"] >= 1
    assert metadata["camera"]["right"]["packet_count"] >= 1

    source_table = pq.read_table(result.paths.source_records)
    assert source_table.schema.field("pc_receive_monotonic_ns").type == pa.int64()
    assert source_table.schema.field("pts_ticks").type == pa.int64()
    source_rows = source_table.to_pylist()
    assert {row["source_record_id"] for row in source_rows} == {left.source_record_id, right.source_record_id}
    assert all(row["stored_in_video"] is True for row in source_rows)

    tri_table = pq.read_table(result.paths.triangulation)
    tri_row = tri_table.to_pylist()[0]
    assert tri_row["stereo_center_forward_m"] == 10.0
    assert tri_row["stereo_center_right_m"] == 2.0
    assert tri_row["stereo_center_down_m"] == 3.0
    assert tri_row["ray_gap_m"] == 0.001
    assert tri_row["intersection_angle_rad"] == 0.1
    assert not any(field.startswith("corrected_") for field in tri_table.column_names)
    assert not any(field.startswith("reference_") for field in tri_table.column_names)
    assert "correction_profile_id" not in tri_table.column_names


def test_rdf_tuple_is_explicitly_mapped_to_frd_named_columns() -> None:
    assert map_optical_rdf_to_stereo_frd((2.0, 3.0, 10.0)) == (10.0, 2.0, 3.0)
    with pytest.raises(ValueError):
        map_optical_rdf_to_stereo_frd((float("nan"), 3.0, 10.0))


def test_close_is_idempotent_for_a_non_video_unit_recorder(tmp_path: Path) -> None:
    recorder = SessionRecorder.open(
        SessionRecordingConfig(
            root=tmp_path / "recordings",
            session_id="idempotent-0001",
            require_video_artifacts=False,
        )
    )
    provisional = json.loads(recorder.paths.metadata.read_text(encoding="utf-8"))
    assert provisional["session_state"] == "recording"
    assert provisional["clean_shutdown"] is False
    assert provisional["incomplete"] is True
    assert isinstance(provisional["started_monotonic_ns"], int)
    assert isinstance(provisional["started_utc_ns"], int)
    first = recorder.close()
    second = recorder.close()
    assert first == second
    assert first.session_state == "incomplete"


def test_failure_preserves_partial_parquet_artifacts_and_finalizes_incomplete(tmp_path: Path) -> None:
    session_id = "failure-0001"
    recorder = SessionRecorder.open(
        SessionRecordingConfig(
            root=tmp_path / "recordings",
            session_id=session_id,
            require_video_artifacts=False,
        )
    )
    recorder.record_camera_source(_source(session_id, "left", 0))
    recorder.mark_failure("synthetic_writer", "forced writer failure")
    result = recorder.close()

    assert result.session_state == "incomplete"
    assert result.clean_shutdown is False
    assert result.incomplete is True
    assert result.paths.source_records.is_file()
    metadata = json.loads(result.paths.metadata.read_text(encoding="utf-8"))
    assert metadata["failure_stage"] == "synthetic_writer"
    assert metadata["failure_message"] == "forced writer failure"
    assert metadata["session_state"] == "incomplete"


def test_finalize_failure_flushes_source_rows_and_duplicate_mappings_are_rejected(tmp_path: Path) -> None:
    session_id = "mapping-0001"
    recorder = SessionRecorder.open(
        SessionRecordingConfig(
            root=tmp_path / "recordings",
            session_id=session_id,
            require_video_artifacts=False,
        )
    )
    source = _source(session_id, "left", 0)
    recorder.record_camera_source(source)
    mapping = VideoMapping(
        source_record_id=source.source_record_id,
        stored_in_video=True,
        video_mapping_status="stored",
        video_frame_index=0,
    )
    recorder.mark_video_mapping(mapping)
    with pytest.raises(SchemaValidationError):
        recorder.mark_video_mapping(mapping)
    result = recorder.close()
    assert result.session_state == "incomplete"

    failure_id = "mapping-failure-0001"
    failed = SessionRecorder.open(
        SessionRecordingConfig(
            root=tmp_path / "recordings",
            session_id=failure_id,
            require_video_artifacts=False,
        )
    )
    pending_source = _source(failure_id, "right", 0)
    failed.record_camera_source(pending_source)
    incomplete = failed.close()
    assert incomplete.session_state == "incomplete"
    source_rows = pq.read_table(incomplete.paths.source_records).to_pylist()
    assert source_rows[0]["source_record_id"] == pending_source.source_record_id
    assert source_rows[0]["video_mapping_status"] == "pending"

    pending_id = "pending-duplicate-0001"
    pending_recorder = SessionRecorder.open(
        SessionRecordingConfig(
            root=tmp_path / "recordings",
            session_id=pending_id,
            require_video_artifacts=False,
        )
    )
    pending_mapping = VideoMapping(
        source_record_id="future-source",
        stored_in_video=True,
        video_mapping_status="stored",
        video_frame_index=0,
    )
    pending_recorder.mark_video_mapping(pending_mapping)
    with pytest.raises(SchemaValidationError):
        pending_recorder.mark_video_mapping(pending_mapping)
    pending_recorder.record_camera_source(replace(_source(pending_id, "left", 0), source_record_id="future-source"))
    pending_recorder.close()


def test_source_monotonic_timestamp_and_stored_mapping_index_are_validated(tmp_path: Path) -> None:
    session_id = "validation-0001"
    recorder = SessionRecorder.open(
        SessionRecordingConfig(
            root=tmp_path / "recordings",
            session_id=session_id,
            require_video_artifacts=False,
        )
    )
    invalid_source = _source(session_id, "left", 0)
    invalid_source = invalid_source.__class__(**{**invalid_source.__dict__, "pc_receive_monotonic_ns": -1})
    with pytest.raises(SchemaValidationError):
        recorder.record_camera_source(invalid_source)
    with pytest.raises(SchemaValidationError):
        recorder.mark_video_mapping(
            VideoMapping(
                source_record_id="missing-frame",
                stored_in_video=True,
                video_mapping_status="stored",
            )
        )
    recorder.close()
