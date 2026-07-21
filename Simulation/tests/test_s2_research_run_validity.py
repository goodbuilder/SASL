from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.research_recording import (
    RESEARCH_SCHEMA_VERSION,
    ResearchRecorder,
    empty_pair_record,
)


def _recorder(tmp_path: Path):
    return ResearchRecorder(
        output_dir=tmp_path / "run",
        run_metadata={
            "run_id": "run",
            "case_id": "case",
            "commit_sha": "b" * 40,
            "git_status": "clean",
        },
        queue_capacity=8,
    )


def _record(*, status="evaluated", eligible=True):
    record = empty_pair_record()
    record.update(
        {
            "schema_version": RESEARCH_SCHEMA_VERSION,
            "run_id": "run",
            "case_id": "case",
            "record_sequence": None,
            "record_type": "pair_terminal",
            "pair_id": 1,
            "epoch": 0,
            "left_stamp_ns": 10,
            "right_stamp_ns": 12,
            "pair_midpoint_stamp_ns": 11,
            "pair_delta_ns": 2,
            "pairing_mode": "nearest",
            "terminal_status": status,
            "terminal_reason": status,
            "sample_eligible": eligible,
            "measurement_status": "processed",
            "evaluation_status": "evaluated" if eligible else "unavailable",
            "output_frame": "baseline_center_frd",
        }
    )
    return record


def _state(**changes):
    result = {
        "run_id": "run",
        "case_id": "case",
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
    result.update(changes)
    return result


@pytest.mark.parametrize("terminal_status", ("gt_unavailable", "ideal_projection_rejected"))
def test_explicit_noneligible_terminal_alone_does_not_invalidate_run(
    tmp_path: Path, terminal_status: str
) -> None:
    recorder = _recorder(tmp_path)
    assert recorder.submit(_record(status=terminal_status, eligible=False))
    summary = recorder.close(_state(gt_evaluation_success_count=0))
    assert summary["run_valid"] is True
    assert summary["eligible_sample_count"] == 0


@pytest.mark.parametrize(
    ("changes", "reason"),
    (
        ({"measurement_overrun_invalid": True}, "measurement_queue_overrun"),
        ({"measurement_processing_error_count": 1}, "measurement_processing_error"),
        ({"gt_evaluation_error_count": 1}, "gt_evaluation_error"),
        ({"simulation_reset_count": 1}, "simulation_reset"),
        ({"shutdown_drain_status": "INCOMPLETE"}, "shutdown_drain_incomplete"),
        ({"remaining_measurement_queue": 1}, "measurement_queue_not_empty"),
        ({"remaining_gt_queue": 1}, "gt_queue_not_empty"),
        ({"inflight_pair": 1}, "inflight_pair_remaining"),
    ),
)
def test_runtime_faults_finalize_invalid(tmp_path: Path, changes, reason: str) -> None:
    recorder = _recorder(tmp_path)
    assert recorder.submit(_record())
    summary = recorder.close(_state(**changes))
    assert summary["run_valid"] is False
    assert reason in summary["invalid_reasons"]
    assert (recorder.output_dir / "RUN_FINALIZED_INVALID").is_file()
    assert not (recorder.output_dir / "RUN_FINALIZED_VALID").exists()


def test_accounting_mismatch_cannot_finalize_valid(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    assert recorder.submit(_record())
    summary = recorder.close(_state(accepted=2, processed=1))
    assert summary["run_valid"] is False
    assert "accepted_equals_processed_plus_discarded" in summary["invalid_reasons"]
    assert "terminal_records_equal_accepted" in summary["invalid_reasons"]


def test_summary_and_metadata_share_schema_and_identity(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    assert recorder.submit(_record())
    summary = recorder.close(_state())
    metadata = json.loads(recorder.metadata_path.read_text(encoding="utf-8"))
    assert summary["schema_version"] == metadata["schema_version"] == RESEARCH_SCHEMA_VERSION
    assert summary["run_id"] == metadata["run_id"] == "run"
    assert summary["case_id"] == metadata["case_id"] == "case"
