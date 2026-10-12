"""Missing raw streams and false-zero detections cannot qualify a run."""

from __future__ import annotations

import json
import sys
from pathlib import Path


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.qc_attempt import (
    OBSERVATION_REQUIRED,
    _workbook_run_manifest_errors,
    _yolo_rate_errors,
    validate,
)


def test_empty_manifest_slot_is_infrastructure_failed(tmp_path: Path):
    report = validate(tmp_path, "G0-P0-R01", "a01")
    assert report["status"] == "INFRASTRUCTURE_FAILED"
    assert not report["counted_in_39"]
    assert "missing_required_run_files" in report["infrastructure_errors"]
    assert "missing_raw_capture_summary.json" in report["infrastructure_errors"]
    assert "missing_state_capture_summary.json" in report["infrastructure_errors"]


def test_development_flight_never_counts_and_missing_bbox_must_be_null(tmp_path: Path):
    row = {name: None for name in OBSERVATION_REQUIRED}
    row.update({
        "run_id": "DEVELOPMENT-G0-P0", "attempt_id": "a01", "camera_id": "left",
        "frame_id": "left:0:100:000001", "detection_status": "NOT_DETECTED",
        "bbox_xyxy_px": [0.0, 0.0, 0.0, 0.0], "bbox_center_uv_px": [0.0, 0.0],
        "yolo_weights_sha256": json.loads((SIM_ROOT / "campaigns/stereo_v1_1/model_manifest.json").read_text())["sha256"],
    })
    (tmp_path / "observations_left.jsonl").write_text(json.dumps(row) + "\n")
    report = validate(tmp_path, "DEVELOPMENT-G0-P0", "a01")
    assert report["status"] == "DEVELOPMENT"
    assert not report["counted_in_39"]
    assert "left_missing_detection_encoded_as_point" in report["infrastructure_errors"]


def test_science_gt_gap_and_v0_reveal_are_independently_flagged(tmp_path: Path):
    (tmp_path / "gt_alignment_summary.json").write_text(json.dumps({
        "image_count_by_alignment_status": {"EXTRAPOLATION": 1},
        "science_image_count_by_alignment_status": {"EXTRAPOLATION": 1},
        "tracking_metrics_revealed": True,
        "tracking": None,
    }))
    report = validate(tmp_path, "G0-V0-R01", "a01")
    assert "science_images_lack_valid_gt" in report["infrastructure_errors"]
    assert "v0_tracking_metrics_opened_before_freeze" in report["protocol_errors"]
    assert not report["counted_in_39"]


def test_workbook_yolo_rate_qc_uses_per_camera_source_rate():
    capture = {"yolo_target_hz": 20.0}
    image_summary = {
        "left": {"observed_source_hz": 20.0},
        "right": {"observed_source_hz": 10.0},
    }
    rate_summary = {"actual_source_simulation_rate_hz": {"left": 20.0, "right": 10.0}}
    assert _yolo_rate_errors(rate_summary, capture, image_summary) == []

    rate_summary["actual_source_simulation_rate_hz"]["left"] = 10.0
    assert _yolo_rate_errors(rate_summary, capture, image_summary) == [
        "left_yolo_actual_rate_below_workbook_target",
    ]


def test_workbook_run_manifest_qc_reads_vehicle_identity_from_parameters():
    source = {
        "sequence": 1,
        "run_id": "G0-P0-R01",
        "repetition": 1,
        "role": "calibration",
        "seed_requested": 2026100501,
        "workbook_sha256": "workbook-hash",
        "use_for_calibration": True,
        "use_for_final_evaluation": False,
        "parameters": {"Vehicle": {
            "model_id": "x500_2m",
            "reference_link": "vehicle_reference_link",
        }},
    }
    actual = {
        "sequence": 1,
        "run_id": "G0-P0-R01",
        "repetition": 1,
        "role": "calibration",
        "seed_requested": 2026100501,
        "workbook_sha256": "workbook-hash",
        "use_for_calibration": True,
        "use_for_final_evaluation": False,
        "vehicle_model_id": "x500_2m",
        "vehicle_reference_link": "vehicle_reference_link",
    }

    assert _workbook_run_manifest_errors(actual, source) == []
    actual["vehicle_model_id"] = "unsupported_model"
    assert _workbook_run_manifest_errors(actual, source) == [
        "workbook_run_manifest_vehicle_model_id_mismatch",
    ]
