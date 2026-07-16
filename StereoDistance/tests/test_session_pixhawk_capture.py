import json

import pyarrow as pa
import pyarrow.parquet as pq
import pytest

from wiris_stereo_distance.session_recording_parquet import (
    PixhawkSampleRecord,
    SchemaValidationError,
    SessionRecordingConfig,
    SessionRecorder,
    pixhawk_samples_schema,
)


def _recorder(tmp_path):
    return SessionRecorder.open(
        SessionRecordingConfig(
            root=tmp_path / "recordings",
            session_id="pixhawk-0001",
            require_video_artifacts=False,
        )
    )


def _gps_sample(session_id: str, slot: str, sequence: int) -> PixhawkSampleRecord:
    return PixhawkSampleRecord(
        sample_id=f"{slot}-{sequence}",
        session_id=session_id,
        pixhawk_id="pixhawk_left",
        camera_side="left",
        transport="ros2",
        namespace="/uav_left",
        topic=f"/uav_left/fmu/out/{slot}",
        message_type="px4_msgs/msg/SensorGps",
        source_slot=slot,
        sample_sequence=sequence,
        pc_receive_monotonic_ns=1_000 + sequence,
        px4_timestamp_us=2_000 + sequence,
        px4_timestamp_sample_us=1_900 + sequence,
        px4_time_utc_usec=1_700_000_000_000_000 + sequence,
        device_id=42 + sequence,
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_msl_m=100.0,
        fix_type=3,
        satellites_used=12,
        jamming_state=2 + sequence,
        spoofing_state=1 + sequence,
        eph_m=0.5,
        epv_m=0.8,
        raw_fields={
            "timestamp": 2_000 + sequence,
            "timestamp_sample": 1_900 + sequence,
            "time_utc_usec": 1_700_000_000_000_000 + sequence,
            "device_id": 42 + sequence,
        },
    )


def test_pixhawk_raw_timestamps_quaternion_and_sensor_slots_are_preserved(tmp_path) -> None:
    recorder = _recorder(tmp_path)
    recorder.record_pixhawk_sample(
        PixhawkSampleRecord(
            sample_id="attitude-0",
            session_id="pixhawk-0001",
            pixhawk_id="pixhawk_left",
            camera_side="left",
            transport="ros2",
            namespace="/uav_left",
            topic="/uav_left/fmu/out/vehicle_attitude",
            message_type="px4_msgs/msg/VehicleAttitude",
            source_slot="vehicle_attitude",
            sample_sequence=0,
            pc_receive_monotonic_ns=900,
            px4_timestamp_us=1_000,
            px4_timestamp_sample_us=990,
            attitude_q_w=1.0,
            attitude_q_x=0.0,
            attitude_q_y=0.0,
            attitude_q_z=0.0,
            raw_fields={"timestamp": 1_000, "timestamp_sample": 990, "q": [1.0, 0.0, 0.0, 0.0]},
        )
    )
    recorder.record_pixhawk_sample(_gps_sample("pixhawk-0001", "sensor_gps_0", 0))
    recorder.record_pixhawk_sample(_gps_sample("pixhawk-0001", "sensor_gps_1", 1))
    result = recorder.close()

    table = pq.read_table(result.paths.pixhawk_samples)
    assert table.schema.field("px4_timestamp_us").type == pa.int64()
    assert table.schema.field("px4_timestamp_sample_us").type == pa.int64()
    assert table.schema.field("px4_time_utc_usec").type == pa.int64()
    assert table.schema.field("jamming_state").type == pa.int64()
    assert table.schema.field("spoofing_state").type == pa.int64()
    rows = table.to_pylist()
    attitude = next(row for row in rows if row["source_slot"] == "vehicle_attitude")
    assert (attitude["attitude_q_w"], attitude["attitude_q_x"], attitude["attitude_q_y"], attitude["attitude_q_z"]) == (
        1.0,
        0.0,
        0.0,
        0.0,
    )
    assert json.loads(attitude["raw_fields_json"])["timestamp_sample"] == 990
    assert {row["source_slot"] for row in rows if row["message_type"].endswith("SensorGps")} == {
        "sensor_gps_0",
        "sensor_gps_1",
    }
    gps_rows = {row["source_slot"]: row for row in rows if row["message_type"].endswith("SensorGps")}
    assert gps_rows["sensor_gps_0"]["jamming_state"] == 2
    assert gps_rows["sensor_gps_0"]["spoofing_state"] == 1
    assert gps_rows["sensor_gps_1"]["jamming_state"] == 3
    assert gps_rows["sensor_gps_1"]["spoofing_state"] == 2
    assert "gps_left" not in table.column_names
    assert "gps_right" not in table.column_names


def test_invalid_quaternion_and_non_finite_raw_field_are_rejected(tmp_path) -> None:
    recorder = _recorder(tmp_path)
    with pytest.raises(SchemaValidationError):
        recorder.record_pixhawk_sample(
            PixhawkSampleRecord(
                sample_id="invalid-q",
                session_id="pixhawk-0001",
                pixhawk_id="pixhawk_left",
                transport="ros2",
                topic="/uav_left/fmu/out/vehicle_attitude",
                message_type="px4_msgs/msg/VehicleAttitude",
                source_slot="vehicle_attitude",
                sample_sequence=0,
                pc_receive_monotonic_ns=1,
                attitude_q_w=1.0,
                attitude_q_x=None,
                attitude_q_y=0.0,
                attitude_q_z=0.0,
            )
        )
    with pytest.raises(SchemaValidationError):
        recorder.record_pixhawk_sample(
            PixhawkSampleRecord(
                sample_id="invalid-raw",
                session_id="pixhawk-0001",
                pixhawk_id="pixhawk_left",
                transport="ros2",
                topic="/uav_left/fmu/out/sensor_gps_0",
                message_type="px4_msgs/msg/SensorGps",
                source_slot="sensor_gps_0",
                sample_sequence=0,
                pc_receive_monotonic_ns=1,
                raw_fields={"eph": float("nan")},
            )
        )
    recorder.close()


def test_pixhawk_schema_keeps_original_time_fields_as_nullable_int64() -> None:
    schema = pixhawk_samples_schema()
    for name in (
        "px4_timestamp_us",
        "px4_timestamp_sample_us",
        "px4_time_utc_usec",
        "mavlink_time_usec",
        "mavlink_time_boot_ms",
    ):
        assert schema.field(name).type == pa.int64()
        assert schema.field(name).nullable is True


@pytest.mark.parametrize("value", [True, 1.5, "2", 1 << 63])
def test_sensor_gps_provenance_fields_require_nullable_int64(tmp_path, value) -> None:
    recorder = _recorder(tmp_path)
    sample = _gps_sample("pixhawk-0001", "sensor_gps_0", 0)
    field_name = "jamming_state"
    invalid = PixhawkSampleRecord(**{**sample.__dict__, field_name: value})
    with pytest.raises(SchemaValidationError):
        recorder.record_pixhawk_sample(invalid)
    recorder.close()


def test_sensor_gps_provenance_fields_allow_null(tmp_path) -> None:
    recorder = _recorder(tmp_path)
    sample = _gps_sample("pixhawk-0001", "sensor_gps_0", 0)
    recorder.record_pixhawk_sample(
        PixhawkSampleRecord(**{**sample.__dict__, "sample_id": "nullable", "jamming_state": None, "spoofing_state": None})
    )
    result = recorder.close()
    row = pq.read_table(result.paths.pixhawk_samples).to_pylist()[0]
    assert row["jamming_state"] is None
    assert row["spoofing_state"] is None
