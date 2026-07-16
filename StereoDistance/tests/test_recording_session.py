from pathlib import Path

import csv
import json

from wiris_stereo_distance.recording_session import (
    evidence_contract_metadata,
    existing_video_outputs,
    format_video_paths,
    raw_video_paths,
    save_session_files,
    session_dir,
)


def test_recording_session_paths_use_stable_session_layout() -> None:
    root = Path("/tmp/recordings")

    assert session_dir(root, "20260605_120000") == root / "sessions" / "20260605_120000"
    assert raw_video_paths(root, "20260605_120000") == {
        "left": root / "sessions" / "20260605_120000" / "video" / "raw_left.mp4",
        "right": root / "sessions" / "20260605_120000" / "video" / "raw_right.mp4",
    }


def test_format_video_paths_handles_empty_and_populated_paths() -> None:
    assert format_video_paths({}) == "저장 경로 없음"
    assert (
        format_video_paths({"left": Path("/tmp/session/video/raw_left.mp4")})
        == "left=/tmp/session/video/raw_left.mp4"
    )


def test_existing_video_outputs_ignores_missing_and_empty_files(tmp_path) -> None:
    paths = raw_video_paths(tmp_path, "20260605_120000")
    paths["left"].parent.mkdir(parents=True)
    paths["left"].write_bytes(b"mp4")
    paths["right"].touch()

    assert existing_video_outputs(paths) == {"left": paths["left"]}


def test_save_session_files_includes_existing_video_outputs(tmp_path) -> None:
    paths = raw_video_paths(tmp_path, "20260605_120003")
    paths["left"].parent.mkdir(parents=True)
    paths["left"].write_bytes(b"mp4")

    saved = save_session_files(
        tmp_path / "sessions" / "20260605_120003",
        metadata={"session_id": "20260605_120003"},
        video_paths=paths,
    )

    assert saved.video_outputs == {"left": paths["left"]}
    assert saved.outputs == [saved.metadata, paths["left"]]


def test_save_session_files_writes_metadata_and_available_csvs(tmp_path) -> None:
    saved = save_session_files(
        tmp_path / "sessions" / "20260605_120000",
        metadata={"session_id": "20260605_120000", "distance_sample_count": 1},
        distance_rows=[{"timestamp_s": "1.0", "distance_m": "5.4"}],
        external_parameter_rows=[{"timestamp_s": "1.0", "roll_deg": "0.1"}],
        pixhawk_mag_rows=[],
    )

    assert saved.outputs == [saved.metadata, saved.distance_history, saved.external_parameters]
    assert saved.metadata == tmp_path / "sessions" / "20260605_120000" / "metadata" / "session_metadata.json"
    assert saved.distance_history == (
        tmp_path / "sessions" / "20260605_120000" / "measurements" / "distance_history.csv"
    )
    assert saved.external_parameters == (
        tmp_path / "sessions" / "20260605_120000" / "alignment" / "external_parameters.csv"
    )
    metadata = json.loads(saved.metadata.read_text(encoding="utf-8"))
    assert metadata["session_id"] == "20260605_120000"
    assert metadata["session_schema_version"] == 2
    assert metadata["system_claim"] == evidence_contract_metadata()["system_claim"]
    assert metadata["coordinate_contract"]["measurement_frame"] == "stereo_center_opencv_optical"
    assert metadata["coordinate_contract"]["mixed_frame_policy"] == "diagnostic-only"
    assert metadata["mount_assumption"]["model"] == "nominal-shared-rig-v1"
    assert metadata["evidence_policy"]["simulation_pass_is_hardware_pass"] is False
    assert metadata["connection_policy"]["preferred_state"] == "mavlink_and_ros2_connected"
    assert metadata["connection_policy"]["readiness_authority"] == "mavlink"
    assert metadata["connection_policy"]["ros2_replaces_mavlink_control"] is False
    assert saved.distance_history is not None
    with saved.distance_history.open("r", newline="", encoding="utf-8") as handle:
        assert list(csv.DictReader(handle)) == [{"timestamp_s": "1.0", "distance_m": "5.4"}]
    assert saved.external_parameters is not None
    with saved.external_parameters.open("r", newline="", encoding="utf-8") as handle:
        assert list(csv.DictReader(handle)) == [{"timestamp_s": "1.0", "roll_deg": "0.1"}]
    assert saved.pixhawk_mag_status is None
    assert saved.uam_analysis is None


def test_save_session_files_writes_uam_analysis_when_available(tmp_path) -> None:
    saved = save_session_files(
        tmp_path / "sessions" / "20260605_120001",
        metadata={"session_id": "20260605_120001", "uam_analysis_sample_count": 1},
        uam_analysis_rows=[{"row_index": "1", "error_distance_m": "0.25"}],
    )

    assert saved.uam_analysis is not None
    assert saved.outputs == [saved.metadata, saved.uam_analysis]
    assert saved.uam_analysis == tmp_path / "sessions" / "20260605_120001" / "measurements" / "uam_analysis.csv"
    with saved.uam_analysis.open("r", newline="", encoding="utf-8") as handle:
        assert list(csv.DictReader(handle)) == [{"row_index": "1", "error_distance_m": "0.25"}]


def test_save_session_files_writes_gps_source_comparison(tmp_path) -> None:
    saved = save_session_files(
        tmp_path / "sessions" / "20260605_120002",
        metadata={"session_id": "20260605_120002"},
        gps_source_comparison_rows=[
            {
                "comparison_status": "OK",
                "sensor_gps_baseline_m": "25.0",
                "global_position_baseline_m": "25.1",
            }
        ],
    )

    assert saved.gps_source_comparison is not None
    assert saved.gps_source_comparison == (
        tmp_path / "sessions" / "20260605_120002" / "alignment" / "gps_source_comparison.csv"
    )
    with saved.gps_source_comparison.open("r", newline="", encoding="utf-8") as handle:
        assert list(csv.DictReader(handle)) == [
            {
                "comparison_status": "OK",
                "sensor_gps_baseline_m": "25.0",
                "global_position_baseline_m": "25.1",
            }
        ]
