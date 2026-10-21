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
    revalidate_cadence_qc_regression,
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
    mapping = json.loads((tmp_path / "schema_mapping.json").read_text())
    topics = {row["topic"]: row for row in mapping["topics"]}
    assert topics["/clock"]["output"] == "clock.jsonl.simulation_timestamp_ns"
    assert "FLU to ENU" in topics["/simulation/camera_pose_diagnostics/left"]["contract"]
    assert topics["/left_camera/camera_info"]["message"] == "sensor_msgs/msg/CameraInfo"
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
    with pytest.raises(ValueError, match="two consecutive technical failures"):
        begin_attempt(tmp_path, "G0-P0-R01")


def test_different_consecutive_technical_failures_also_require_review(tmp_path: Path):
    initialize(tmp_path)
    for reason in ("missing_ground_truth", "image_queue_overflow"):
        attempt, _ = begin_attempt(tmp_path, "G0-P0-R01")
        finish_attempt(tmp_path, "G0-P0-R01", attempt["attempt_id"], {
            "run_id": "G0-P0-R01", "attempt_id": attempt["attempt_id"],
            "status": "INFRASTRUCTURE_FAILED", "infrastructure_errors": [reason],
            "protocol_errors": [],
        })
    with pytest.raises(ValueError, match="two consecutive technical failures"):
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


def _cadence_false_positive(tmp_path):
    from campaigns.stereo_v1_1.campaign_status import _write_attempt_checksums
    attempt, folder, previous = _known_qc_deviation(tmp_path)
    for filename, collection in (("run_status.json", "run_slots"), ("manifest_actual.json", "attempts")):
        path = tmp_path / filename
        value = json.loads(path.read_text())
        row = value[collection][0]
        row["status"] = "INFRASTRUCTURE_FAILED"
        if collection == "run_slots":
            row["attempts"][0]["status"] = "INFRASTRUCTURE_FAILED"
        path.write_text(json.dumps(value))
    previous.update(status="INFRASTRUCTURE_FAILED", protocol_errors=[],
                    infrastructure_errors=["science_setpoint_cadence_below_v12_requirement"])
    (folder / "qc_report.json").write_text(json.dumps(previous))
    (folder / "runner_result.json").write_text(json.dumps({"qc_status": "INFRASTRUCTURE_FAILED", "runner_failure_reason": None}))
    (folder / "run_manifest.json").write_text(json.dumps({"protocol_version": "1.2"}))
    _write_attempt_checksums(folder)
    fresh = {**previous, "status": "VALID_WITH_WARNINGS", "counted_in_campaign": True,
             "infrastructure_errors": [], "evidence": {"science_setpoint_cadence_recomputed": {
                 "criterion": "reference_slot_coverage_v2", "missing_full_slot_count": 0,
                 "sufficient_unique_simulation_cadence": True}}}
    return folder, fresh


def test_cadence_revalidation_preserves_raw_and_original_checksums(tmp_path):
    folder, fresh = _cadence_false_positive(tmp_path)
    raw = (folder / "source.json").read_bytes()
    original_checksums = (folder / "checksums.sha256").read_bytes()
    revalidate_cadence_qc_regression(tmp_path, "G0-P0-R01", "a01", fresh)
    assert (folder / "source.json").read_bytes() == raw
    assert (folder / "checksums_pre_revalidation.sha256").read_bytes() == original_checksums
    assert json.loads((folder / "qc_report_pre_revalidation.json").read_text())["status"] == "INFRASTRUCTURE_FAILED"
    assert json.loads((tmp_path / "run_status.json").read_text())["valid_run_count"] == 1
    assert json.loads((folder / "qc_revalidation.json").read_text())["criterion"] == "reference_slot_coverage_v2"
    _checksum_entries(folder)
    with pytest.raises(ValueError):
        revalidate_cadence_qc_regression(tmp_path, "G0-P0-R01", "a01", fresh)


@pytest.mark.parametrize("defect", ["other_error", "missing_slot", "source_changed", "not_latest"])
def test_cadence_revalidation_rejects_unrelated_or_unproven_repair(tmp_path, defect):
    from campaigns.stereo_v1_1.campaign_status import _write_attempt_checksums
    folder, fresh = _cadence_false_positive(tmp_path)
    attempt_id = "a01"
    if defect == "other_error":
        p = folder / "qc_report.json"
        previous = json.loads(p.read_text())
        previous["infrastructure_errors"].append("missing_ground_truth")
        p.write_text(json.dumps(previous))
        _write_attempt_checksums(folder)
    elif defect == "missing_slot":
        fresh["evidence"]["science_setpoint_cadence_recomputed"]["missing_full_slot_count"] = 1
    elif defect == "source_changed":
        (folder / "source.json").write_text("changed")
    else:
        attempt_id = "a02"
    with pytest.raises(ValueError):
        revalidate_cadence_qc_regression(tmp_path, "G0-P0-R01", attempt_id, fresh)
    assert not (folder / "qc_revalidation.json").exists()


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


@pytest.mark.parametrize('tamper', [False, True])
def test_reviewed_user_retry_is_one_attempt_and_evidence_bound(tmp_path, tamper):
    from campaigns.stereo_v1_1.campaign_status import authorize_reviewed_retry
    initialize(tmp_path)
    for _ in range(2):
        attempt, _ = begin_attempt(tmp_path, 'G0-P0-R01')
        finish_attempt(tmp_path, 'G0-P0-R01', attempt['attempt_id'], {
            'run_id': 'G0-P0-R01', 'attempt_id': attempt['attempt_id'],
            'status': 'INFRASTRUCTURE_FAILED', 'infrastructure_errors': ['cadence'],
            'protocol_errors': []})
    evidence = tmp_path / 'review.json'
    evidence.write_text('{"repaired": false}')
    authorize_reviewed_retry(tmp_path, 'G0-P0-R01',
                            user_instruction='Resume after the diagnostic report',
                            evidence_paths=['review.json'])
    if tamper:
        evidence.write_text('changed')
        with pytest.raises(ValueError, match='two consecutive'):
            begin_attempt(tmp_path, 'G0-P0-R01')
        return
    with pytest.raises(ValueError, match='already recorded'):
        authorize_reviewed_retry(tmp_path, 'G0-P0-R01', user_instruction='Again',
                                 evidence_paths=['review.json'])
    attempt, _ = begin_attempt(tmp_path, 'G0-P0-R01')
    assert attempt['attempt_id'] == 'a03'
    assert attempt['reviewed_retry_authorization']['used']
    assert attempt['reviewed_retry_authorization']['cause_repaired'] is False
    finish_attempt(tmp_path, 'G0-P0-R01', 'a03', {
        'run_id': 'G0-P0-R01', 'attempt_id': 'a03',
        'status': 'INFRASTRUCTURE_FAILED', 'infrastructure_errors': ['cadence'],
        'protocol_errors': []})
    with pytest.raises(ValueError, match='two consecutive'):
        begin_attempt(tmp_path, 'G0-P0-R01')


@pytest.mark.parametrize('blocked_case', [None, 'tampered_raw', 'science_started', 'empty_checksums'])
def test_nonstart_reclassification_preserves_raw_and_allows_same_seed_retry(tmp_path, blocked_case):
    from campaigns.stereo_v1_1.campaign_status import invalidate_pre_science_attempt, _write_attempt_checksums
    initialize(tmp_path)
    attempt, directory = begin_attempt(tmp_path, 'G0-V0-R01')
    original = {'run_id': 'G0-V0-R01', 'attempt_id': 'a01', 'status': 'VALID_WITH_WARNINGS',
                'infrastructure_errors': [], 'protocol_errors': [], 'counted_in_campaign': True,
                'evidence': {'controller_status': 'failed', 'controller_failure_reason': 'entry timeout'}}
    finish_attempt(tmp_path, 'G0-V0-R01', 'a01', original)
    (directory / 'qc_report.json').write_text(json.dumps(original))
    (directory / 'runner_result.json').write_text(json.dumps({'qc_status': 'VALID_WITH_WARNINGS'}))
    (directory / 'controller_result.json').write_text(json.dumps({'controller_status': 'failed', 'failure_reason': 'entry timeout'}))
    (directory / 'run_manifest.json').write_text(json.dumps({'protocol_version': '1.2'}))
    (directory / 'events.jsonl').write_text(json.dumps({'event': 'science_start' if blocked_case == 'science_started' else 'entry_started'})+'\n')
    raw = directory / 'ground_truth.jsonl'
    raw.write_text('original raw\n')
    _write_attempt_checksums(directory)
    fresh = {**original, 'status': 'INFRASTRUCTURE_FAILED', 'counted_in_campaign': False,
             'counted_in_39': False, 'infrastructure_errors': ['pre_science_execution_failure']}
    if blocked_case:
        if blocked_case == 'tampered_raw':raw.write_text('tampered\n')
        if blocked_case == 'empty_checksums':(directory/'checksums.sha256').write_text('')
        with pytest.raises(ValueError):invalidate_pre_science_attempt(tmp_path, 'G0-V0-R01', 'a01', fresh)
        return
    row = invalidate_pre_science_attempt(tmp_path, 'G0-V0-R01', 'a01', fresh)
    assert row['status'] == 'INFRASTRUCTURE_FAILED'
    assert raw.read_text() == 'original raw\n'
    assert json.loads((directory / 'qc_report.json.before_entry_reclassification').read_text()) == original
    assert json.loads((tmp_path / 'run_status.json').read_text())['valid_run_count'] == 0
    next_attempt, _ = begin_attempt(tmp_path, 'G0-V0-R01')
    assert next_attempt['attempt_id'] == 'a02'
    assert next_attempt['seed_requested'] == attempt['seed_requested']
