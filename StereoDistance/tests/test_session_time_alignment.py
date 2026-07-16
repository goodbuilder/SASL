import pyarrow as pa
import pyarrow.parquet as pq

from wiris_stereo_distance.session_recording_parquet import (
    SessionRecordingConfig,
    SessionRecorder,
    TimeAlignmentRecord,
    time_alignment_schema,
)


def test_time_alignment_keeps_timestamp_domains_separate(tmp_path) -> None:
    recorder = SessionRecorder.open(
        SessionRecordingConfig(
            root=tmp_path / "recordings",
            session_id="time-0001",
            require_video_artifacts=False,
        )
    )
    recorder.record_time_alignment(
        TimeAlignmentRecord(
            alignment_id="camera-fallback-0",
            record_type="camera",
            record_id="left-source-0",
            camera_side="left",
            alignment_reference_domain="pc_receive_monotonic",
            alignment_status="receive_time_fallback",
            aligned_pc_monotonic_ns=100,
            pc_receive_monotonic_ns=100,
            pc_receive_utc_ns=None,
            camera_pts_ticks=90_000,
        )
    )
    recorder.record_time_alignment(
        TimeAlignmentRecord(
            alignment_id="pixhawk-utc-0",
            record_type="pixhawk",
            record_id="left-attitude-0",
            pixhawk_id="pixhawk_left",
            alignment_reference_domain="px4_utc",
            alignment_status="utc_aligned",
            aligned_pc_monotonic_ns=200,
            pc_receive_monotonic_ns=200,
            pc_receive_utc_ns=1_700_000_000_000_000_000,
            px4_timestamp_us=500,
            px4_time_utc_usec=1_700_000_000_000_000,
            px4_utc_to_pc_utc_delta_ns=0,
        )
    )
    result = recorder.close()

    table = pq.read_table(result.paths.time_alignment)
    assert table.schema.field("pc_receive_monotonic_ns").type == pa.int64()
    assert table.schema.field("pc_receive_utc_ns").type == pa.int64()
    assert table.schema.field("px4_timestamp_us").type == pa.int64()
    assert "source_timestamp_i64" not in table.column_names
    assert "px4_boot_to_pc_monotonic_delta_ns" not in table.column_names
    rows = {row["alignment_id"]: row for row in table.to_pylist()}
    assert rows["camera-fallback-0"]["aligned_pc_monotonic_ns"] == 100
    assert rows["camera-fallback-0"]["camera_pts_ticks"] == 90_000
    assert rows["pixhawk-utc-0"]["px4_timestamp_us"] == 500
    assert rows["pixhawk-utc-0"]["px4_utc_to_pc_utc_delta_ns"] == 0


def test_time_alignment_schema_timestamp_columns_are_int64() -> None:
    schema = time_alignment_schema()
    timestamp_names = {
        "aligned_pc_monotonic_ns",
        "pc_receive_monotonic_ns",
        "pc_receive_utc_ns",
        "camera_rtp_timestamp",
        "camera_pts_ticks",
        "camera_dts_ticks",
        "camera_rtcp_ntp_unix_ns",
        "px4_timestamp_us",
        "px4_timestamp_sample_us",
        "px4_time_utc_usec",
        "mavlink_time_usec",
        "mavlink_time_boot_ms",
        "camera_rtcp_to_pc_utc_delta_ns",
        "px4_utc_to_pc_utc_delta_ns",
    }
    assert all(schema.field(name).type == pa.int64() for name in timestamp_names)
