from __future__ import annotations

import csv
import json
import sys
import threading
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.research_recording import (
    PAIR_RECORD_COLUMNS,
    RESEARCH_SCHEMA_VERSION,
    ResearchRecorder,
    empty_pair_record,
    sha256_file,
    validate_pair_record,
    validate_recording_configuration,
)


SHA = "a" * 40


def _config(tmp_path: Path, **changes):
    repository_root = tmp_path / "repository"
    repository_root.mkdir(parents=True, exist_ok=True)
    (repository_root / ".git").write_text("gitdir: elsewhere\n", encoding="utf-8")
    values = {
        "output_dir": str(tmp_path / "run"),
        "run_id": "run-1",
        "case_id": "case-1",
        "repository": "goodbuilder/SASL",
        "branch": "sim/test",
        "commit_sha": SHA,
        "git_status": "clean",
        "gazebo_version": "8.0",
        "px4_version": "not_applicable",
        "queue_capacity": 8,
        "repository_root": repository_root,
    }
    values.update(changes)
    return values


def _metadata():
    return {
        "run_id": "run-1",
        "case_id": "case-1",
        "repository": "goodbuilder/SASL",
        "branch": "sim/test",
        "commit_sha": SHA,
        "git_status": "clean",
        "gazebo_version": "8.0",
        "px4_version": "not_applicable",
        "observation_mode": "ideal_reference_point",
        "output_frame": "baseline_center_frd",
    }


def _record(pair_id=1, epoch=0, stamp=9_007_199_254_740_993):
    record = empty_pair_record()
    record.update(
        {
            "schema_version": RESEARCH_SCHEMA_VERSION,
            "run_id": "run-1",
            "case_id": "case-1",
            "record_sequence": None,
            "record_type": "pair_terminal",
            "pair_id": pair_id,
            "epoch": epoch,
            "left_stamp_ns": stamp,
            "right_stamp_ns": stamp + 2,
            "pair_midpoint_stamp_ns": stamp + 1,
            "pair_delta_ns": 2,
            "pairing_mode": "nearest",
            "terminal_status": "evaluated",
            "terminal_reason": "evaluated",
            "sample_eligible": True,
            "measurement_status": "processed",
            "evaluation_status": "evaluated",
            "output_frame": "baseline_center_frd",
        }
    )
    return record


def _final_state(**changes):
    state = {
        "run_id": "run-1",
        "case_id": "case-1",
        "shutdown_drain_status": "SUCCESS",
        "accepted": 1,
        "processed": 1,
        "discarded": 0,
        "measurement_queue_overflow_count": 0,
        "measurement_overrun_invalid": False,
        "measurement_rejected_after_overrun_count": 0,
        "measurement_processing_error_count": 0,
        "gt_evaluation_success_count": 1,
        "gt_evaluation_error_count": 0,
        "simulation_reset_count": 0,
        "remaining_measurement_queue": 0,
        "remaining_gt_queue": 0,
        "inflight_pair": None,
    }
    state.update(changes)
    return state


def _recorder(tmp_path: Path, *, capacity=8):
    output = validate_recording_configuration(**_config(tmp_path, queue_capacity=capacity))
    return ResearchRecorder(
        output_dir=output,
        run_metadata=_metadata(),
        queue_capacity=capacity,
    )


def test_disabled_contract_creates_no_files_or_thread(tmp_path: Path) -> None:
    output = tmp_path / "disabled"
    recorder = None
    assert recorder is None
    assert not output.exists()
    assert not any(thread.name == "simulation-research-recorder" for thread in threading.enumerate())


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("run_id", "", "missing"),
        ("commit_sha", "ABC", "40 lowercase"),
        ("git_status", "unknown", "clean or dirty"),
        ("queue_capacity", 0, "positive"),
    ),
)
def test_enabled_configuration_rejects_invalid_provenance(
    tmp_path: Path, field: str, value, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        validate_recording_configuration(**_config(tmp_path, **{field: value}))


def test_output_path_must_be_absolute_or_under_simulation_recordings_and_empty(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="absolute"):
        validate_recording_configuration(**_config(tmp_path, output_dir="relative"))
    repository = _config(tmp_path)["repository_root"]
    with pytest.raises(ValueError, match="outside"):
        validate_recording_configuration(
            **_config(tmp_path, output_dir=str(repository / "results"))
        )
    allowed = repository / "Simulation" / "recordings" / "case-1"
    output = validate_recording_configuration(
        **_config(tmp_path, output_dir=str(allowed))
    )
    assert output == allowed
    symlink = tmp_path / "symlink"
    symlink.symlink_to(repository, target_is_directory=True)
    with pytest.raises(ValueError, match="outside"):
        validate_recording_configuration(
            **_config(tmp_path, output_dir=str(symlink / "results"))
        )
    nonempty = tmp_path / "nonempty"
    nonempty.mkdir()
    (nonempty / "existing").write_text("preserve", encoding="utf-8")
    with pytest.raises(ValueError, match="new or empty"):
        validate_recording_configuration(
            **_config(tmp_path, output_dir=str(nonempty))
        )


def test_metadata_jsonl_csv_hashes_and_valid_marker_are_atomic_and_deterministic(
    tmp_path: Path,
) -> None:
    recorder = _recorder(tmp_path)
    output = recorder.output_dir
    assert (output / "RUN_INCOMPLETE").is_file()
    metadata = json.loads((output / "run_metadata.json").read_text(encoding="utf-8"))
    assert metadata["schema_version"] == RESEARCH_SCHEMA_VERSION
    assert metadata["commit_sha"] == SHA
    assert recorder.submit(_record(pair_id=1))
    assert recorder.submit(_record(pair_id=2))
    final_state = _final_state(accepted=2, processed=2, gt_evaluation_success_count=2)
    summary = recorder.close(final_state)
    assert recorder.close(final_state) == summary

    assert summary["run_valid"] is True
    assert summary["terminal_record_count"] == 2
    assert summary["eligible_sample_count"] == 2
    assert (output / "RUN_FINALIZED_VALID").is_file()
    assert not (output / "RUN_FINALIZED_INVALID").exists()
    assert not (output / "RUN_INCOMPLETE").exists()
    assert summary["pair_records_jsonl_sha256"] == sha256_file(output / "pair_records.jsonl")
    assert summary["pair_records_csv_sha256"] == sha256_file(output / "pair_records.csv")
    assert summary["run_metadata_sha256"] == sha256_file(output / "run_metadata.json")
    lines = [
        json.loads(line)
        for line in (output / "pair_records.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert [line["record_sequence"] for line in lines] == [1, 2]
    with (output / "pair_records.csv").open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert tuple(rows[0]) == PAIR_RECORD_COLUMNS
    assert len(rows) == 2
    assert rows[0]["left_stamp_ns"] == "9007199254740993"
    assert rows[0]["sample_eligible"] == "true"


def test_schema_accepts_nullable_diagnostic_ray_exports() -> None:
    record = _record()
    record["record_sequence"] = 1
    record.update(
        {
            "attitude_ray_computation_status": "computed",
            "attitude_ray_forward_m": 999.5,
            "attitude_ray_right_m": 6.8,
            "attitude_ray_down_m": 4.2,
            "attitude_ray_range_m": 999.532,
            "attitude_ray_delta_forward_m": -0.5,
            "attitude_ray_position_difference_m": 0.5745,
            "operational_ray_computation_status": "computed",
            "operational_ray_baseline_source": "px4_raw_vehicle_gps_position_baseline_only",
            "operational_ray_forward_m": 998.5,
            "operational_ray_range_m": 998.534,
            "ideal_position_error_m": 0.03,
            "ideal_forward_error_m": 0.01,
        }
    )

    normalized = validate_pair_record(record)

    assert normalized["schema_version"] == RESEARCH_SCHEMA_VERSION
    assert normalized["attitude_ray_delta_forward_m"] == -0.5
    assert normalized["operational_ray_baseline_source"] == "px4_raw_vehicle_gps_position_baseline_only"
    assert "attitude_ray_computation_status" in PAIR_RECORD_COLUMNS
    assert "operational_ray_gps_baseline_world_enu_x_m" in PAIR_RECORD_COLUMNS
    assert "operational_ray_position_difference_m" in PAIR_RECORD_COLUMNS
    assert "ideal_position_error_m" in PAIR_RECORD_COLUMNS
    assert "mission_phase" in PAIR_RECORD_COLUMNS
    assert "uam_raw_gps_reference_forward_m" in PAIR_RECORD_COLUMNS
    assert "uam_raw_gps_reference_world_enu_x_m" in PAIR_RECORD_COLUMNS
    assert "uam_raw_gps_reference_pair_skew_ms" in PAIR_RECORD_COLUMNS
    assert "session_id" in PAIR_RECORD_COLUMNS
    assert "attempt_id" in PAIR_RECORD_COLUMNS
    assert "effective_config_sha256" in PAIR_RECORD_COLUMNS
    assert "operational_ray_left_origin_x_m" in PAIR_RECORD_COLUMNS
    assert "operational_ray_right_direction_z" in PAIR_RECORD_COLUMNS
    assert "sample_id" in PAIR_RECORD_COLUMNS
    assert "left_bbox_xmin_px" in PAIR_RECORD_COLUMNS
    assert "left_operational_attitude_qw" in PAIR_RECORD_COLUMNS
    assert "operational_x_m" in PAIR_RECORD_COLUMNS
    assert "gt_timestamp_ns" in PAIR_RECORD_COLUMNS


def test_diagnostic_analysis_round_trip_preserves_precision_status_and_time_units(
    tmp_path: Path,
) -> None:
    metadata = {
        **_metadata(),
        "left_R_body_from_optical": [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        "right_R_body_from_optical": [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        "extrinsic_matrix_storage_order": "row_major",
        "extrinsic_source_file": "Simulation/stereo_ground_sim/worlds/stereo_ground_drone.sdf",
    }
    output = validate_recording_configuration(**_config(tmp_path))
    recorder = ResearchRecorder(
        output_dir=output,
        run_metadata=metadata,
        queue_capacity=8,
    )
    record = _record()
    record.update(
        {
            "attitude_ray_valid": True,
            "attitude_ray_computation_status": "computed",
            "attitude_ray_midpoint_x_m": 1000.123456789012,
            "attitude_ray_left_closest_y_m": -2.000000000001,
            "attitude_ray_right_closest_z_m": 3.000000000001,
            "attitude_ray_delta_valid": True,
            "attitude_ray_delta_forward_m": 0.000000000123,
            "attitude_ray_gt_valid": True,
            "attitude_ray_gt_position_error_m": 0.25,
            "operational_ray_valid": True,
            "operational_ray_computation_status": "computed",
            "operational_ray_gt_position_error_m": 0.5,
            "left_px4_q_w": 1.0,
            "left_px4_q_x": 0.0,
            "left_px4_q_y": 0.0,
            "left_px4_q_z": 0.0,
            "right_px4_q_w": 0.999999999999,
            "right_px4_q_z": 0.000001414214,
            "px4_quaternion_order": "wxyz",
            "px4_attitude_semantics": "body_frd_to_earth_ned",
            "left_attitude_timestamp_us": 9_007_199_254_740_991,
            "right_attitude_timestamp_us": 9_007_199_254_740_992,
            "left_attitude_receive_monotonic_ns": 9_007_199_254_740_993,
            "right_attitude_receive_monotonic_ns": 9_007_199_254_740_994,
            "left_attitude_reset_counter": 3,
            "right_attitude_reset_counter": 4,
            "left_gps_timestamp_us": 1_234_567,
            "right_gps_timestamp_us": 1_234_568,
            "baseline_anchor_policy": "nominal_left_optical_center_plus_px4_relative_enu_separation",
        }
    )
    assert recorder.submit(record)
    summary = recorder.close(_final_state())
    assert summary["run_valid"] is True

    stored = json.loads(recorder.jsonl_path.read_text(encoding="utf-8"))
    assert abs(stored["attitude_ray_midpoint_x_m"] - 1000.123456789012) <= 1.0e-12
    assert stored["operational_ray_computation_status"] == "computed"
    assert stored["px4_quaternion_order"] == "wxyz"
    assert stored["left_attitude_timestamp_us"] == 9_007_199_254_740_991
    assert stored["left_attitude_receive_monotonic_ns"] == 9_007_199_254_740_993
    assert stored["left_attitude_reset_counter"] == 3
    with recorder.csv_path.open(encoding="utf-8", newline="") as handle:
        csv_row = next(csv.DictReader(handle))
    assert csv_row["operational_ray_gt_position_error_m"] == "0.5"
    assert csv_row["left_attitude_timestamp_us"] == "9007199254740991"
    assert csv_row["left_attitude_receive_monotonic_ns"] == "9007199254740993"
    assert csv_row["right_attitude_reset_counter"] == "4"
    saved_metadata = json.loads(recorder.metadata_path.read_text(encoding="utf-8"))
    assert saved_metadata["left_R_body_from_optical"] == metadata["left_R_body_from_optical"]
    assert saved_metadata["extrinsic_matrix_storage_order"] == "row_major"


def test_legacy_v3_record_is_rejected_by_new_schema_only_reader() -> None:
    legacy = _record()
    legacy["record_sequence"] = 1
    legacy["schema_version"] = "s2_atomic_pair_v3"
    legacy = {
        key: value
        for key, value in legacy.items()
        if not key.endswith("_ray_valid") and not key.startswith("interaction_")
    }

    with pytest.raises(ValueError, match="pair record columns differ"):
        validate_pair_record(legacy)


def test_jsonl_rejects_nan_without_writing_a_terminal_line(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    record = _record()
    record["nominal_pose_forward_m"] = float("nan")
    assert recorder.submit(record) is True
    summary = recorder.close(_final_state())
    assert summary["run_valid"] is False
    assert summary["research_recording_error_count"] == 1
    assert summary["jsonl_line_count"] == 0
    assert (recorder.output_dir / "RUN_FINALIZED_INVALID").is_file()


def test_duplicate_pair_key_and_corrupt_jsonl_finalize_invalid(tmp_path: Path) -> None:
    duplicate = _recorder(tmp_path / "duplicate")
    assert duplicate.submit(_record(pair_id=5))
    assert duplicate.submit(_record(pair_id=5))
    summary = duplicate.close(_final_state(accepted=2, processed=2))
    assert summary["run_valid"] is False
    assert "research_recording_error" in summary["invalid_reasons"]

    corrupt = _recorder(tmp_path / "corrupt")
    corrupt._jsonl_handle.write("not-json\n")
    corrupt._jsonl_handle.flush()
    summary = corrupt.close(_final_state(accepted=0, processed=0))
    assert summary["run_valid"] is False
    assert (corrupt.output_dir / "RUN_FINALIZED_INVALID").is_file()


def test_csv_row_count_mismatch_is_invalid(tmp_path: Path, monkeypatch) -> None:
    recorder = _recorder(tmp_path)
    assert recorder.submit(_record())

    def header_only(_records):
        with recorder.csv_path.open("w", encoding="utf-8", newline="") as handle:
            csv.DictWriter(handle, fieldnames=PAIR_RECORD_COLUMNS).writeheader()

    monkeypatch.setattr(recorder, "_export_csv", header_only)
    summary = recorder.close(_final_state())
    assert summary["run_valid"] is False
    assert "jsonl_equals_csv_rows" in summary["invalid_reasons"]


def test_writer_exception_and_bounded_queue_overflow_finalize_invalid(
    tmp_path: Path,
) -> None:
    recorder = _recorder(tmp_path / "writer", capacity=1)
    entered = threading.Event()
    release = threading.Event()
    real_handle = recorder._jsonl_handle

    class BlockingFailure:
        def write(self, _value):
            entered.set()
            release.wait(2.0)
            raise OSError("injected write failure")

        def __getattr__(self, name):
            return getattr(real_handle, name)

    recorder._jsonl_handle = BlockingFailure()
    assert recorder.submit(_record(pair_id=1))
    assert entered.wait(1.0)
    assert recorder.submit(_record(pair_id=2))
    assert recorder.submit(_record(pair_id=3)) is False
    release.set()
    summary = recorder.close(_final_state(accepted=3, processed=3))
    assert summary["run_valid"] is False
    assert summary["research_recording_overrun_invalid"] is True
    assert summary["research_recording_error_count"] >= 2
    assert (recorder.output_dir / "RUN_FINALIZED_INVALID").is_file()
