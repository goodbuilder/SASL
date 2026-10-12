"""Every manifest slot is represented before flight, without a false attempt."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.campaign_status import (
    REVALIDATABLE_QC_REGRESSION_ERRORS,
    begin_attempt,
    finish_attempt,
    initialize,
    revalidate_known_qc_regression,
)


def test_initial_status_has_39_planned_slots_and_no_attempts(tmp_path: Path):
    summary = initialize(tmp_path)
    status = json.loads((tmp_path / "run_status.json").read_text())
    actual = json.loads((tmp_path / "manifest_actual.json").read_text())
    assert summary["status_counts"] == {"PLANNED": 39}
    assert status["valid_run_count"] == 0
    assert len(status["run_slots"]) == 39
    assert len({row["run_id"] for row in status["run_slots"]}) == 39
    assert all(row["attempts"] == [] and row["seed_effective"] is None and row["recording_path"] is None
               for row in status["run_slots"])
    assert actual["actual_attempt_count"] == 0
    with pytest.raises(FileExistsError, match="already exist"):
        initialize(tmp_path)
    assert json.loads((tmp_path / "run_status.json").read_text()) == status


def test_first_valid_attempt_is_retained_and_cannot_be_rerun(tmp_path: Path):
    initialize(tmp_path)
    attempt, path = begin_attempt(tmp_path, "G0-P0-R01")
    assert attempt["attempt_id"] == "a01" and path.is_dir()
    qc = {"run_id": "G0-P0-R01", "attempt_id": "a01", "status": "VALID_WITH_WARNINGS",
          "infrastructure_errors": [], "protocol_errors": [], "warnings": ["tracking_position_p95_above_1m"]}
    row = finish_attempt(tmp_path, "G0-P0-R01", "a01", qc,
                         {"gazebo_cli": True, "independent_repetition_verified": False})
    assert row["status"] == "VALID_WITH_WARNINGS"
    status = json.loads((tmp_path / "run_status.json").read_text())
    assert status["valid_run_count"] == 1
    assert status["run_slots"][0]["attempt_count"] == 1
    with pytest.raises(ValueError, match="first valid attempt"):
        begin_attempt(tmp_path, "G0-P0-R01")
    assert path.is_dir()


def test_structural_failure_retries_same_seed_without_overwriting(tmp_path: Path):
    initialize(tmp_path)
    first, first_path = begin_attempt(tmp_path, "G0-P0-R01")
    qc1 = {"run_id": "G0-P0-R01", "attempt_id": "a01", "status": "INFRASTRUCTURE_FAILED",
           "infrastructure_errors": ["missing_ground_truth"], "protocol_errors": []}
    finish_attempt(tmp_path, "G0-P0-R01", "a01", qc1)
    second, second_path = begin_attempt(tmp_path, "G0-P0-R01")
    assert second["attempt_id"] == "a02"
    assert second["seed_requested"] == first["seed_requested"]
    assert second_path != first_path and first_path.is_dir()
    qc2 = {"run_id": "G0-P0-R01", "attempt_id": "a02", "status": "INFRASTRUCTURE_FAILED",
           "infrastructure_errors": ["missing_ground_truth"], "protocol_errors": []}
    finish_attempt(tmp_path, "G0-P0-R01", "a02", qc2)
    with pytest.raises(ValueError, match="same technical failure twice"):
        begin_attempt(tmp_path, "G0-P0-R01")


def test_valid_raw_attempt_preserves_controller_performance_failure_reason(tmp_path: Path):
    initialize(tmp_path)
    begin_attempt(tmp_path, "G0-V0-R01")
    qc = {
        "run_id": "G0-V0-R01", "attempt_id": "a01", "status": "VALID_WITH_WARNINGS",
        "infrastructure_errors": [], "protocol_errors": [],
        "warnings": ["science_not_completed_performance_outcome"],
        "evidence": {"controller_status": "failed",
                     "controller_failure_reason": "entry trajectory did not reach the science start"},
    }
    row = finish_attempt(tmp_path, "G0-V0-R01", "a01", qc)
    assert row["status"] == "VALID_WITH_WARNINGS"
    assert row["failure_reason"] == "entry trajectory did not reach the science start"
    assert row["attempts"][0]["failure_reason"] == row["failure_reason"]


def _known_qc_deviation(tmp_path: Path):
    plan = {
        "schema_version": "stereo_v1_1_workbook_campaign_plan_v2",
        "workbook_schema_version": "sweep_manifest_v2",
        "planned_run_count": 1,
        "planned_runs": [{"run_id": "G0-P0-R01", "seed_requested": 2026100501,
                           "status": "PLANNED", "attempt_count": 0, "recording_path": None}],
    }
    initialize(tmp_path, plan=plan)
    attempt, attempt_dir = begin_attempt(tmp_path, "G0-P0-R01")
    previous = {
        "run_id": "G0-P0-R01",
        "attempt_id": "a01",
        "status": "PROTOCOL_DEVIATION",
        "counted_in_campaign": False,
        "infrastructure_errors": [],
        "protocol_errors": sorted(REVALIDATABLE_QC_REGRESSION_ERRORS),
        "warnings": [],
        "evidence": {},
    }
    (attempt_dir / "source.json").write_text("preserved\n")
    (attempt_dir / "qc_report.json").write_text(json.dumps(previous, indent=2) + "\n")
    (attempt_dir / "runner_result.json").write_text(json.dumps({
        "run_id": "G0-P0-R01", "attempt_id": "a01", "qc_status": "PROTOCOL_DEVIATION",
        "controller_exit_code": 0, "runner_failure_reason": None,
    }, indent=2) + "\n")
    failed_qc = {
        **previous,
        "infrastructure_errors": [],
    }
    finish_attempt(tmp_path, "G0-P0-R01", "a01", failed_qc,
                   {"gazebo_cli": True, "independent_repetition_verified": False})
    return attempt, attempt_dir, previous


def _checksum_entries(attempt_dir: Path) -> dict[str, str]:
    import hashlib

    result = {}
    for line in (attempt_dir / "checksums.sha256").read_text().splitlines():
        digest, relative = line.split("  ", 1)
        result[relative] = digest
        assert hashlib.sha256((attempt_dir / relative).read_bytes()).hexdigest() == digest
    return result


def test_known_qc_regression_revalidates_same_attempt_and_preserves_prior_report(tmp_path: Path):
    attempt, attempt_dir, previous = _known_qc_deviation(tmp_path)
    fresh = {
        "run_id": "G0-P0-R01",
        "attempt_id": "a01",
        "status": "VALID_WITH_WARNINGS",
        "counted_in_campaign": True,
        "infrastructure_errors": [],
        "protocol_errors": [],
        "warnings": ["tracking_position_p95_above_configured_limit"],
        "evidence": {"controller_status": "science_completed"},
    }

    row = revalidate_known_qc_regression(tmp_path, "G0-P0-R01", "a01", fresh)

    status = json.loads((tmp_path / "run_status.json").read_text())
    actual = json.loads((tmp_path / "manifest_actual.json").read_text())
    report = json.loads((attempt_dir / "qc_report.json").read_text())
    prior = json.loads((attempt_dir / "qc_report_pre_revalidation.json").read_text())
    runner_result = json.loads((attempt_dir / "runner_result.json").read_text())
    assert row["status"] == "VALID_WITH_WARNINGS"
    assert status["valid_run_count"] == 1
    assert status["run_slots"][0]["attempt_count"] == 1
    assert actual["actual_attempt_count"] == 1
    assert actual["attempts"][0]["status"] == "VALID_WITH_WARNINGS"
    assert report == fresh and prior == previous
    assert runner_result["qc_revalidated"] is True
    assert runner_result["previous_qc_status"] == "PROTOCOL_DEVIATION"
    assert "qc_revalidation.json" in _checksum_entries(attempt_dir)
    events = [json.loads(line) for line in (tmp_path / "changes_log.jsonl").read_text().splitlines()]
    assert events[-1]["event"] == "attempt_qc_revalidated_after_validator_fix"
    assert events[-1]["attempt_id"] == "a01"
    assert events[-1]["previous_protocol_errors"] == sorted(REVALIDATABLE_QC_REGRESSION_ERRORS)
    with pytest.raises(ValueError, match="first valid attempt"):
        begin_attempt(tmp_path, "G0-P0-R01")


def test_known_qc_revalidation_rejects_unrelated_protocol_errors(tmp_path: Path):
    _attempt, _attempt_dir, previous = _known_qc_deviation(tmp_path)
    previous["protocol_errors"] = ["different_protocol_error"]
    attempt_dir = tmp_path / "원본_실행별/G0-P0-R01/a01"
    (attempt_dir / "qc_report.json").write_text(json.dumps(previous, indent=2) + "\n")
    fresh = {
        "run_id": "G0-P0-R01", "attempt_id": "a01", "status": "VALID",
        "counted_in_campaign": True, "infrastructure_errors": [], "protocol_errors": [],
    }
    with pytest.raises(ValueError, match="outside the known validator regression"):
        revalidate_known_qc_regression(tmp_path, "G0-P0-R01", "a01", fresh)
