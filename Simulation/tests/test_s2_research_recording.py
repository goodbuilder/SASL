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


def test_output_path_must_be_absolute_outside_repository_and_empty(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="absolute"):
        validate_recording_configuration(**_config(tmp_path, output_dir="relative"))
    repository = _config(tmp_path)["repository_root"]
    with pytest.raises(ValueError, match="outside"):
        validate_recording_configuration(
            **_config(tmp_path, output_dir=str(repository / "results"))
        )
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


def test_jsonl_rejects_nan_without_writing_a_terminal_line(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    record = _record()
    record["raw_forward_m"] = float("nan")
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
