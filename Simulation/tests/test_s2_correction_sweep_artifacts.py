from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_sweep import (
    compute_case_statistics,
    validate_r2_artifacts,
)
from stereo_ground_sim.research_recording import PAIR_RECORD_COLUMNS, RESEARCH_SCHEMA_VERSION


SHA = "a" * 40
BRANCH = "sim/test"
RUN = "sweep"
CASE = "case-000001-deadbeefdeadbeef"


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _record(sequence: int) -> dict:
    record = {name: None for name in PAIR_RECORD_COLUMNS}
    record.update(
        {
            "schema_version": RESEARCH_SCHEMA_VERSION,
            "run_id": RUN,
            "case_id": CASE,
            "record_sequence": sequence,
            "record_type": "pair_terminal",
            "pair_id": sequence,
            "epoch": 0,
            "left_stamp_ns": 1000 + sequence * 10,
            "right_stamp_ns": 1002 + sequence * 10,
            "pair_midpoint_stamp_ns": 1001 + sequence * 10,
            "pair_delta_ns": 2,
            "pairing_mode": "exact",
            "terminal_status": "evaluated",
            "terminal_reason": "evaluated",
            "sample_eligible": True,
            "measurement_status": "processed",
            "evaluation_status": "evaluated",
            "raw_position_error_m": float(sequence),
            "raw_forward_error_m": float(sequence),
            "raw_right_error_m": -float(sequence),
            "raw_down_error_m": float(sequence) / 2,
            "raw_range_error_m": -float(sequence) / 2,
            "raw_ray_gap_m": float(sequence) / 10,
            "raw_intersection_angle_deg": 10.0 + sequence,
            "corrected_position_error_m": float(sequence) / 10,
            "corrected_forward_error_m": float(sequence) / 10,
            "corrected_right_error_m": -float(sequence) / 10,
            "corrected_down_error_m": float(sequence) / 20,
            "corrected_range_error_m": -float(sequence) / 20,
            "corrected_ray_gap_m": float(sequence) / 100,
            "corrected_intersection_angle_deg": 10.0 + sequence,
            "output_frame": "baseline_center_frd",
            "output_axes": "forward_right_down",
            "position_unit": "m",
            "pixel_unit": "px",
            "angle_unit": "deg",
            "timestamp_unit": "ns",
        }
    )
    return record


def _write_artifact(root: Path, records=None) -> Path:
    records = list(records or (_record(1), _record(2), _record(3)))
    root.mkdir(parents=True)
    metadata = {
        "schema_version": RESEARCH_SCHEMA_VERSION,
        "commit_sha": SHA,
        "branch": BRANCH,
        "run_id": RUN,
        "case_id": CASE,
        "output_frame": "baseline_center_frd",
    }
    (root / "run_metadata.json").write_text(
        json.dumps(metadata, sort_keys=True) + "\n", encoding="utf-8"
    )
    (root / "pair_records.jsonl").write_text(
        "".join(json.dumps(record, sort_keys=True) + "\n" for record in records),
        encoding="utf-8",
    )
    with (root / "pair_records.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=PAIR_RECORD_COLUMNS)
        writer.writeheader()
        writer.writerows(records)
    summary = {
        "schema_version": RESEARCH_SCHEMA_VERSION,
        "run_id": RUN,
        "case_id": CASE,
        "run_valid": True,
        "shutdown_drain_status": "SUCCESS",
        "accepted": len(records),
        "processed": len(records),
        "discarded": 0,
        "terminal_record_count": len(records),
        "jsonl_line_count": len(records),
        "csv_data_row_count": len(records),
        "unique_pair_key_count": len(records),
        "pair_records_jsonl_sha256": _sha(root / "pair_records.jsonl"),
        "pair_records_csv_sha256": _sha(root / "pair_records.csv"),
        "run_metadata_sha256": _sha(root / "run_metadata.json"),
    }
    (root / "run_summary.json").write_text(
        json.dumps(summary, sort_keys=True) + "\n", encoding="utf-8"
    )
    (root / "RUN_FINALIZED_VALID").touch()
    return root


def _validate(path: Path, minimum=2):
    return validate_r2_artifacts(
        path,
        expected_sha=SHA,
        expected_branch=BRANCH,
        expected_run_id=RUN,
        expected_case_id=CASE,
        minimum_eligible=minimum,
    )


def _rewrite_rows(path: Path, records) -> None:
    (path / "pair_records.jsonl").write_text(
        "".join(json.dumps(record, sort_keys=True) + "\n" for record in records), encoding="utf-8"
    )
    with (path / "pair_records.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=PAIR_RECORD_COLUMNS)
        writer.writeheader()
        writer.writerows(records)
    summary = json.loads((path / "run_summary.json").read_text())
    summary["pair_records_jsonl_sha256"] = _sha(path / "pair_records.jsonl")
    summary["pair_records_csv_sha256"] = _sha(path / "pair_records.csv")
    (path / "run_summary.json").write_text(json.dumps(summary) + "\n")


def test_valid_r2_artifact_and_fixed_window_statistics(tmp_path: Path) -> None:
    path = _write_artifact(tmp_path / "recording")
    records, summary = _validate(path)
    assert summary["run_valid"] is True
    statistics = compute_case_statistics(records, warmup_count=1, analysis_count=2)
    assert statistics["analysis_sample_count"] == 2
    assert statistics["total_eligible_count"] == 3
    assert statistics["raw_position_error_median"] == 2.5
    assert statistics["raw_position_error_p95"] == pytest.approx(2.95)
    assert statistics["corrected_position_error_median"] == pytest.approx(0.25)
    assert statistics["corrected_raw_median_ratio"] == pytest.approx(0.1)
    assert statistics["corrected_improvement_percent"] == pytest.approx(90.0)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("sample_eligible", "true", "JSON bool"),
        ("terminal_status", "unknown", "terminal_status"),
        ("measurement_status", "unknown", "measurement_status"),
        ("evaluation_status", "unknown", "evaluation_status"),
        ("left_stamp_ns", 1.5, "integer"),
    ),
)
def test_strict_pair_semantics_reject_invalid_values(
    tmp_path: Path, field: str, value, message: str
) -> None:
    path = _write_artifact(tmp_path / field)
    records = [_record(1), _record(2), _record(3)]
    records[0][field] = value
    _rewrite_rows(path, records)
    with pytest.raises(ValueError, match=message):
        _validate(path)


def test_metadata_sha_branch_run_and_case_mismatch_rejected(tmp_path: Path) -> None:
    path = _write_artifact(tmp_path / "recording")
    metadata = json.loads((path / "run_metadata.json").read_text())
    metadata["commit_sha"] = "b" * 40
    (path / "run_metadata.json").write_text(json.dumps(metadata) + "\n")
    summary = json.loads((path / "run_summary.json").read_text())
    summary["run_metadata_sha256"] = _sha(path / "run_metadata.json")
    (path / "run_summary.json").write_text(json.dumps(summary) + "\n")
    with pytest.raises(ValueError, match="commit_sha"):
        _validate(path)


def test_duplicate_sequence_hash_row_and_marker_failures(tmp_path: Path) -> None:
    duplicate = _write_artifact(tmp_path / "duplicate")
    records = [_record(1), _record(1), _record(3)]
    _rewrite_rows(duplicate, records)
    with pytest.raises(ValueError, match="record_sequence|duplicate"):
        _validate(duplicate)

    hash_failure = _write_artifact(tmp_path / "hash")
    (hash_failure / "pair_records.jsonl").write_text("{}\n")
    with pytest.raises(ValueError, match="schema|hash"):
        _validate(hash_failure)

    incomplete = _write_artifact(tmp_path / "incomplete")
    (incomplete / "RUN_INCOMPLETE").touch()
    with pytest.raises(ValueError, match="RUN_INCOMPLETE"):
        _validate(incomplete)

    marker = _write_artifact(tmp_path / "marker")
    (marker / "RUN_FINALIZED_VALID").unlink()
    with pytest.raises(ValueError, match="incomplete"):
        _validate(marker)


def test_invalid_summary_and_insufficient_samples_rejected(tmp_path: Path) -> None:
    path = _write_artifact(tmp_path / "invalid")
    summary = json.loads((path / "run_summary.json").read_text())
    summary["run_valid"] = False
    (path / "run_summary.json").write_text(json.dumps(summary) + "\n")
    with pytest.raises(ValueError, match="invalid"):
        _validate(path)
    path2 = _write_artifact(tmp_path / "short")
    with pytest.raises(ValueError, match="insufficient"):
        _validate(path2, minimum=4)
