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


def _v12_fixture(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _sha256
    camera = {"fx_px": 100., "fy_px": 100., "cx_px": 32., "cy_px": 24.,
              "image_width_px": 64, "image_height_px": 48, "camera_rate_hz": 20.}
    source = {"protocol_version": "1.2", "parameters": {"CameraModel": camera}}
    def write(name, value):
        (tmp_path / name).write_text(json.dumps(value))
    write("image_index_summary.json", {"camera_info": {side: {
        "message_count": 1, "variants": [{"width": 64, "height": 48,
            "k": [100., 0, 32., 0, 100., 24., 0, 0, 1], "d": []}]}
        for side in ("left", "right")}})
    state_artifact = tmp_path / "state.db3"
    state_artifact.write_bytes(b"fixture")
    streams = {side: {"topic_counts": {f"/{side}_camera/image": 21,
                f"/{side}_camera/camera_info": 1}} for side in ("left", "right")}
    streams["state"] = {"topic_counts": {name: 1 for name in (
        "/clock", "/simulation/ground_truth/vehicle_reference_link",
        "/simulation/camera_pose_diagnostics/left", "/simulation/camera_pose_diagnostics/right")},
        "files": [{"path_from_attempt_root": "state.db3", "sha256": _sha256(state_artifact)}]}
    write("raw_capture_summary.json", {"streams": streams})
    write("controller_result.json", {"controller_status": "science_completed"})
    frames = [{"camera_id": side, "simulation_epoch": 2,
               "image_timestamp_ns": 1_000_000_000 + i*50_000_000,
               "image_width_px": 64, "image_height_px": 48}
              for side in ("left", "right") for i in range(21)]
    (tmp_path / "image_index.jsonl").write_text("".join(json.dumps(f)+"\n" for f in frames))
    events = [{"event": name, "simulation_epoch": 2, "simulation_timestamp_ns": stamp}
              for name, stamp in (("science_start", 1_000_000_000), ("science_end", 2_000_000_000))]
    (tmp_path / "events.jsonl").write_text("".join(json.dumps(f)+"\n" for f in events))
    state_rows = [{"topic": topic, "simulation_epoch": 2,
                   "simulation_timestamp_ns": 1_000_000_000+i*10_000_000}
                  for topic in ("/clock", "/simulation/ground_truth/vehicle_reference_link")
                  for i in range(101)]
    (tmp_path / "raw_state_index.jsonl").write_text("".join(json.dumps(r)+"\n" for r in state_rows))
    return source, frames, events


def test_v12_full_science_capture_and_camera_contract_pass(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_capture_errors
    source, _, _ = _v12_fixture(tmp_path)
    evidence = {}
    assert _v12_capture_errors(tmp_path, source, evidence) == []
    assert evidence["v12_science_capture"]["cameras"]["left"]["unique_frame_count"] == 21


def test_v12_low_camera_fps_cannot_be_hidden_by_yolo_rate(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_capture_errors
    source, frames, _ = _v12_fixture(tmp_path)
    frames = [r for r in frames if (r["image_timestamp_ns"]-1_000_000_000) % 200_000_000 == 0]
    (tmp_path / "image_index.jsonl").write_text("".join(json.dumps(f)+"\n" for f in frames))
    errors = _v12_capture_errors(tmp_path, source, {})
    assert "left_science_camera_rate_below_requested" in errors
    assert "right_science_image_gap" in errors


def test_v12_empty_science_or_wrong_epoch_fails_even_with_images(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_capture_errors
    source, frames, _ = _v12_fixture(tmp_path)
    for row in frames:
        row["simulation_epoch"] = 1
    (tmp_path / "image_index.jsonl").write_text("".join(json.dumps(f)+"\n" for f in frames))
    errors = _v12_capture_errors(tmp_path, source, {})
    assert "left_science_images_missing" in errors
    assert "right_science_images_missing" in errors


def test_v12_camera_info_and_state_bag_must_be_present(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_capture_errors
    source, _, _ = _v12_fixture(tmp_path)
    (tmp_path / "image_index_summary.json").write_text('{}')
    (tmp_path / "raw_capture_summary.json").write_text('{}')
    errors = _v12_capture_errors(tmp_path, source, {})
    assert "left_camera_info_missing" in errors
    assert "missing_v12_raw_topic:/clock" in errors
    assert "missing_v12_raw_topic:/left_camera/camera_info" in errors


def test_v12_camera_k_and_image_shape_must_match_workbook(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_capture_errors
    source, _, _ = _v12_fixture(tmp_path)
    source["parameters"]["CameraModel"]["fx_px"] = 105.
    source["parameters"]["CameraModel"]["image_width_px"] = 128
    errors = _v12_capture_errors(tmp_path, source, {})
    assert "left_camera_info_differs_from_workbook" in errors
    assert "right_raw_image_dimensions_differ_from_workbook" in errors


def test_v12_performance_abort_preserves_recorded_science_prefix(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_capture_errors
    source, frames, events = _v12_fixture(tmp_path)
    events[-1]["event"] = "emergency_landing_requested"
    events[-1]["simulation_timestamp_ns"] = 1_500_000_000
    (tmp_path / "events.jsonl").write_text("".join(json.dumps(f)+"\n" for f in events))
    (tmp_path / "controller_result.json").write_text(json.dumps({"controller_status": "failed"}))
    assert _v12_capture_errors(tmp_path, source, {}) == []


def test_v12_one_endpoint_frame_tolerance_does_not_allow_missing_boundary(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_capture_errors
    source, frames, _ = _v12_fixture(tmp_path)
    frames = [r for r in frames if r["image_timestamp_ns"] >= 1_100_000_000]
    (tmp_path / "image_index.jsonl").write_text("".join(json.dumps(f)+"\n" for f in frames))
    errors = _v12_capture_errors(tmp_path, source, {})
    assert "left_science_image_boundary_missing" in errors


def test_v12_cadence_recomputes_raw_prefix_even_when_controller_claims_success(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_cadence_errors
    _, _, events = _v12_fixture(tmp_path)
    controller = {"controller_status": "science_completed", "science_setpoint_cadence": {
        "required_hz": 50, "sufficient_unique_simulation_cadence": True, "science_window_covered": True}}
    assert "missing_science_setpoint_timing" in _v12_cadence_errors(tmp_path, controller, {})
    rows = [{"simulation_epoch": 2, "simulation_timestamp_ns": 1_000_000_000+i*20_000_000,
             "host_monotonic_ns": 10_000_000_000+i*20_000_000} for i in range(50)]
    timing = tmp_path / "science_setpoint_timing.jsonl"
    timing.write_text("".join(json.dumps(r)+"\n" for r in rows))
    assert _v12_cadence_errors(tmp_path, controller, {}) == []
    # Abort after half the interval: only the prefix is required, but low-rate
    # raw sends still fail even if the controller's saved boolean says success.
    events[-1].update(event="emergency_landing_requested", simulation_timestamp_ns=1_500_000_000)
    (tmp_path / "events.jsonl").write_text("".join(json.dumps(r)+"\n" for r in events))
    controller["controller_status"] = "performance_failure"
    rows = rows[:25]
    timing.write_text("".join(json.dumps(r)+"\n" for r in rows))
    assert _v12_cadence_errors(tmp_path, controller, {}) == []
    timing.write_text("".join(json.dumps(r)+"\n" for r in rows[::5]))
    assert "science_setpoint_cadence_below_v12_requirement" in _v12_cadence_errors(tmp_path, controller, {})


def test_v12_raw_bag_gt_outage_fails_despite_healthy_jsonl_gt(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_capture_errors
    source, _, _ = _v12_fixture(tmp_path)
    # Existing JSONL alignment status would pass; only the raw bag loses GT.
    (tmp_path / "gt_alignment_summary.json").write_text(json.dumps({
        "science_image_count_by_alignment_status": {"INTERPOLATED": 42}}))
    path = tmp_path / "raw_state_index.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows = [r for r in rows if r["topic"] == "/clock" or not
            1_400_000_000 < r["simulation_timestamp_ns"] < 1_600_000_000]
    path.write_text("".join(json.dumps(r)+"\n" for r in rows))
    evidence = {}
    assert "science_images_lack_raw_bag_gt_brackets" in _v12_capture_errors(tmp_path, source, evidence)
    assert evidence["v12_raw_state_coverage"]["images_without_raw_gt_brackets"]


def test_v12_raw_bag_clock_outage_fails(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_capture_errors
    source, _, _ = _v12_fixture(tmp_path)
    path = tmp_path / "raw_state_index.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows = [r for r in rows if r["topic"] != "/clock" or not
            1_400_000_000 < r["simulation_timestamp_ns"] < 1_600_000_000]
    path.write_text("".join(json.dumps(r)+"\n" for r in rows))
    assert "science_raw_bag_clock_coverage_missing" in _v12_capture_errors(tmp_path, source, {})


def test_v12_cadence_adjacent_slot_jitter_warns_but_missing_slot_fails(tmp_path):
    from campaigns.stereo_v1_1.qc_attempt import _v12_cadence_errors
    _v12_fixture(tmp_path)
    controller = {"controller_status": "science_completed"}
    # Slot 10 receives at its start; slot 11 receives 12ms into its interval.
    # Their 32ms gap is diagnostic jitter, with all 50 reference slots covered.
    rows = [{"simulation_epoch": 2,
             "simulation_timestamp_ns": 1_000_000_000+i*20_000_000+(12_000_000 if i == 11 else 0),
             "host_monotonic_ns": 10_000_000_000+i*20_000_000} for i in range(50)]
    timing = tmp_path / "science_setpoint_timing.jsonl"
    timing.write_text("".join(json.dumps(r)+"\n" for r in rows))
    warnings, evidence = [], {}
    assert _v12_cadence_errors(tmp_path, controller, evidence, warnings) == []
    assert warnings == ["science_setpoint_cadence_jitter_above_diagnostic_threshold"]
    assert evidence["science_setpoint_cadence_recomputed"]["missing_full_slot_count"] == 0
    timing.write_text("".join(json.dumps(r)+"\n" for i, r in enumerate(rows) if i != 11))
    warnings = []
    assert "science_setpoint_cadence_below_v12_requirement" in _v12_cadence_errors(
        tmp_path, controller, {}, warnings)
    assert warnings == []


def test_v12_nonstart_is_infrastructure_but_in_science_failure_remains_outcome(tmp_path):
    from campaigns.stereo_v1_1.workbook_campaign import build_workbook_plan, DEFAULT_WORKBOOK
    source = build_workbook_plan(DEFAULT_WORKBOOK)['planned_runs'][0]
    controller = tmp_path / 'controller_result.json'
    events = tmp_path / 'events.jsonl'
    controller.write_text(json.dumps({'controller_status': 'failed',
                                      'failure_reason': 'entry trajectory did not reach the science start'}))
    events.write_text(json.dumps({'event': 'entry_started'}) + '\n')
    report = validate(tmp_path, source['run_id'], 'a01', planned_source=source)
    assert 'pre_science_execution_failure' in report['infrastructure_errors']
    assert not report['counted_in_campaign']
    controller.write_text(json.dumps({'controller_status': 'failed',
                                      'failure_reason': 'Arm rejected by PX4'}))
    report = validate(tmp_path, source['run_id'], 'a01', planned_source=source)
    assert 'pre_science_execution_failure' in report['infrastructure_errors']
    events.write_text(json.dumps({'event': 'science_start'}) + '\n')
    controller.write_text(json.dumps({'controller_status': 'failed', 'failure_reason': 'tracking failed'}))
    report = validate(tmp_path, source['run_id'], 'a01', planned_source=source)
    assert 'pre_science_execution_failure' not in report['infrastructure_errors']
    assert 'science_not_completed_performance_outcome' in report['warnings']
