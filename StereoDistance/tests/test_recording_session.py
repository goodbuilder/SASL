from pathlib import Path

import csv
import json

from wiris_stereo_distance.recording_session import format_video_paths, raw_video_paths, save_session_files, session_dir


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


def test_save_session_files_writes_metadata_and_available_csvs(tmp_path) -> None:
    saved = save_session_files(
        tmp_path / "sessions" / "20260605_120000",
        metadata={"session_id": "20260605_120000", "distance_sample_count": 1},
        distance_rows=[{"timestamp_s": "1.0", "distance_m": "5.4"}],
        external_parameter_rows=[{"timestamp_s": "1.0", "roll_deg": "0.1"}],
    )

    assert saved.outputs == [saved.metadata, saved.distance_history, saved.external_parameters]
    assert saved.metadata == tmp_path / "sessions" / "20260605_120000" / "metadata" / "session_metadata.json"
    assert saved.distance_history == (
        tmp_path / "sessions" / "20260605_120000" / "measurements" / "distance_history.csv"
    )
    assert saved.external_parameters == (
        tmp_path / "sessions" / "20260605_120000" / "alignment" / "external_parameters.csv"
    )
    assert json.loads(saved.metadata.read_text(encoding="utf-8"))["session_id"] == "20260605_120000"
    assert saved.distance_history is not None
    with saved.distance_history.open("r", newline="", encoding="utf-8") as handle:
        assert list(csv.DictReader(handle)) == [{"timestamp_s": "1.0", "distance_m": "5.4"}]
    assert saved.external_parameters is not None
    with saved.external_parameters.open("r", newline="", encoding="utf-8") as handle:
        assert list(csv.DictReader(handle)) == [{"timestamp_s": "1.0", "roll_deg": "0.1"}]
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
