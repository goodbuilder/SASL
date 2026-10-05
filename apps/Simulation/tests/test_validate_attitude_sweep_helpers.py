from __future__ import annotations

import csv
import importlib.util
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


def _load_module():
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location(
        "validate_attitude_sweep_output_under_test",
        SCRIPTS / "validate_attitude_sweep_output.py",
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_probe_mp4_requires_decodable_positive_duration(tmp_path: Path, monkeypatch) -> None:
    module = _load_module()
    video = tmp_path / "left_overlay.mp4"
    video.write_bytes(b"not-empty-fixture")

    def fake_run(*_args, **_kwargs):
        return subprocess.CompletedProcess(
            args=[],
            returncode=0,
            stdout=(
                '{"streams":[{"codec_name":"mpeg4","width":1280,'
                '"height":720,"duration":"12.5","avg_frame_rate":"20/1",'
                '"r_frame_rate":"20/1","nb_frames":"250"}],'
                '"format":{"duration":"12.5"}}'
            ),
            stderr="",
        )

    monkeypatch.setattr(module.subprocess, "run", fake_run)
    validation = module.Validation()

    result = module._probe_mp4(video, validation)

    assert result["valid"] is True
    assert result["duration_s"] == 12.5
    assert result["frame_count"] == 250
    assert result["actual_average_frame_rate_hz"] == 20.0
    assert validation.errors == []


def test_operator_performance_gate_uses_wall_rate_unique_source_rate_and_telemetry(
    tmp_path: Path, monkeypatch
) -> None:
    module = _load_module()
    operator_dir = tmp_path / "operator-recording"
    operator_dir.mkdir()
    progress = {
        "state": "finalized",
        "mode": "full",
        "csv": "telemetry.csv",
        "video_streams": {
            side: {
                "frame_count": 161,
                "first_elapsed_sec": 15.0,
                "last_elapsed_sec": 25.0,
                "unique_source_frame_count": 161,
                "first_unique_elapsed_sec": 15.0,
                "last_unique_elapsed_sec": 25.0,
            }
            for side in ("left_overlay", "right_overlay")
        },
    }
    (operator_dir / "progress.json").write_text(json.dumps(progress), encoding="utf-8")
    (tmp_path / "operator.log").write_text(
        "YOLO process backend verified: requested=cuda:0, actual=cuda:0, precision=fp16.\n",
        encoding="utf-8",
    )
    with (operator_dir / "telemetry.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = __import__("csv").DictWriter(
            handle, fieldnames=("time_sec", "yolo_update_hz", "yolo_inference_ms")
        )
        writer.writeheader()
        for index in range(301):
            writer.writerow(
                {
                    "time_sec": 15.0 + index / 30.0,
                    "yolo_update_hz": 16.0,
                    "yolo_inference_ms": 40.0,
                }
            )
    monkeypatch.setattr(
        module,
        "_probe_mp4",
        lambda _path, _validation: {
            "valid": True,
            "frame_count": 161,
            "actual_average_frame_rate_hz": 16.0,
        },
    )
    config = {
        "experiment": {
            "require_operator_full_recording": True,
            "performance_acceptance": {
                "warmup_s": 15.0,
                "min_steady_duration_s": 10.0,
                "min_yolo_update_hz": 15.0,
                "max_yolo_inference_p95_ms": 66.7,
                "min_overlay_hz": 15.0,
            },
        }
    }
    validation = module.Validation()

    result = module._validate_operator_recording(tmp_path, config, validation)

    assert validation.errors == []
    assert result["performance"]["yolo_update_median_hz"] == 16.0
    assert result["performance"]["yolo_inference_p95_ms"] == 40.0
    assert result["video_rates"]["left"]["unique_source_observed_hz"] == 16.0
    assert "actual=cuda:0" in result["backend_evidence"]


def test_operator_performance_gate_rejects_duplicated_low_rate_camera_frames(
    tmp_path: Path, monkeypatch
) -> None:
    module = _load_module()
    operator_dir = tmp_path / "operator-recording"
    operator_dir.mkdir()
    progress = {
        "state": "finalized",
        "mode": "full",
        "csv": "telemetry.csv",
        "video_streams": {
            side: {
                "frame_count": 201,
                "first_elapsed_sec": 15.0,
                "last_elapsed_sec": 25.0,
                "unique_source_frame_count": 101,
                "first_unique_elapsed_sec": 15.0,
                "last_unique_elapsed_sec": 25.0,
            }
            for side in ("left_overlay", "right_overlay")
        },
    }
    (operator_dir / "progress.json").write_text(json.dumps(progress), encoding="utf-8")
    (tmp_path / "operator.log").write_text(
        "YOLO process backend verified: requested=cuda:0, actual=cuda:0, precision=fp16.\n",
        encoding="utf-8",
    )
    with (operator_dir / "telemetry.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = __import__("csv").DictWriter(
            handle, fieldnames=("time_sec", "yolo_update_hz", "yolo_inference_ms")
        )
        writer.writeheader()
        for index in range(301):
            writer.writerow(
                {"time_sec": 15.0 + index / 30.0, "yolo_update_hz": 16.0, "yolo_inference_ms": 40.0}
            )
    monkeypatch.setattr(
        module,
        "_probe_mp4",
        lambda _path, _validation: {
            "valid": True,
            "frame_count": 201,
            "actual_average_frame_rate_hz": 20.0,
        },
    )
    config = {
        "experiment": {
            "require_operator_full_recording": True,
            "performance_acceptance": {
                "warmup_s": 15.0,
                "min_steady_duration_s": 10.0,
                "min_yolo_update_hz": 15.0,
                "max_yolo_inference_p95_ms": 66.7,
                "min_overlay_hz": 15.0,
            },
        }
    }
    validation = module.Validation()

    module._validate_operator_recording(tmp_path, config, validation)

    assert any("unique camera-source rate" in error for error in validation.errors)


def test_probe_mp4_rejects_missing_file(tmp_path: Path) -> None:
    module = _load_module()
    validation = module.Validation()

    result = module._probe_mp4(tmp_path / "missing.mp4", validation)

    assert result["valid"] is False
    assert any("missing MP4" in error for error in validation.errors)


def test_waypoint_stability_requires_all_strict_hold_events(tmp_path: Path) -> None:
    module = _load_module()
    events = [
        {
            "schema_version": "px4_waypoint_stability_v1",
            "event": "waypoint_hold_complete",
            "waypoint_index": index,
            "stability_pass": True,
            "position_error_p95_m": 0.8,
            "max_abs_roll_deg": 10.0,
            "max_abs_pitch_deg": 11.0,
        }
        for index in range(1, 9)
    ]
    events.append(
        {
            "schema_version": "px4_waypoint_stability_v1",
            "event": "mission_completed",
        }
    )
    event_path = tmp_path / "waypoint_stability.jsonl"
    event_path.write_text(
        "".join(json.dumps(event) + "\n" for event in events),
        encoding="utf-8",
    )
    config = {
        "experiment": {
            "waypoint_count": 8,
            "stability_position_p95_max_m": 1.0,
            "stability_attitude_max_deg": 15.0,
        }
    }
    validation = module.Validation()

    result = module._validate_waypoint_stability(tmp_path, config, validation)

    assert result["hold_count"] == 8
    assert validation.errors == []


def test_waypoint_distance_analysis_uses_nominal_pose_minus_gt_per_hold() -> None:
    module = _load_module()
    config = {
        "experiment": {
            "waypoint_count": 2,
            "waypoint_distance_analysis": {
                "required": True,
                "phase_pattern": "waypoint_{index:03d}_hold_analysis",
                "primary_estimate_field": "nominal_pose_range_m",
                "ground_truth_field": "gt_range_m",
                "control_estimate_fields": ["corrected_range_m"],
            },
        }
    }
    rows = [
        {
            "mission_phase": "waypoint_001_hold_analysis",
            "nominal_pose_range_m": "101.0",
            "corrected_range_m": "100.2",
            "gt_range_m": "100.0",
        },
        {
            "mission_phase": "waypoint_001_hold_analysis",
            "nominal_pose_range_m": "102.0",
            "corrected_range_m": "100.4",
            "gt_range_m": "100.0",
        },
        {
            "mission_phase": "waypoint_002_hold_analysis",
            "nominal_pose_range_m": "48.0",
            "corrected_range_m": "49.5",
            "gt_range_m": "50.0",
        },
    ]
    validation = module.Validation()

    result = module._validate_waypoint_distance_analysis(config, rows, validation)

    assert validation.errors == []
    assert result["primary_metric"] == "nominal_pose_range_m - gt_range_m"
    first = result["waypoints"][0]["primary"]
    second = result["waypoints"][1]["primary"]
    assert first["sample_count"] == 2
    assert first["signed_error_median_m"] == 1.5
    assert first["absolute_error_p95_m"] == 1.95
    assert second["signed_error_median_m"] == -2.0
    assert round(
        result["waypoints"][0]["controls"]["corrected_range_m"][
            "signed_error_median_m"
        ],
        9,
    ) == 0.3


def test_waypoint_distance_analysis_rejects_missing_hold_phase() -> None:
    module = _load_module()
    config = {
        "experiment": {
            "waypoint_count": 1,
            "waypoint_distance_analysis": {
                "required": True,
                "phase_pattern": "waypoint_{index:03d}_hold_analysis",
                "primary_estimate_field": "nominal_pose_range_m",
                "ground_truth_field": "gt_range_m",
            },
        }
    }
    validation = module.Validation()

    module._validate_waypoint_distance_analysis(config, [], validation)

    assert any("waypoint 1 has no finite" in error for error in validation.errors)


def test_guidance_trace_requires_independent_masks_and_all_waypoints(
    tmp_path: Path,
) -> None:
    module = _load_module()
    events = []
    for index in (1, 2):
        events.extend(
            [
                {
                    "schema_version": "px4_waypoint_guidance_v1",
                    "event": "setpoint_sent",
                    "guidance_mode": "position_velocity_final_target",
                    "mission_phase": f"waypoint_{index:03d}_transit",
                    "waypoint_index": index,
                    "frame": "MAV_FRAME_LOCAL_NED",
                    "active_fields": ["position", "velocity"],
                    "type_mask": 3520,
                    "target_position_local_ned_m": [1.0, 2.0, -3.0],
                    "actual_position_local_ned_m": [0.0, 0.0, 0.0],
                },
                {
                    "schema_version": "px4_waypoint_guidance_v1",
                    "event": "setpoint_sent",
                    "guidance_mode": "position_velocity_final_target",
                    "mission_phase": f"waypoint_{index:03d}_hold_analysis",
                    "waypoint_index": index,
                    "frame": "MAV_FRAME_LOCAL_NED",
                    "active_fields": ["position"],
                    "type_mask": 3576,
                    "target_position_local_ned_m": [1.0, 2.0, -3.0],
                    "actual_position_local_ned_m": [1.0, 2.0, -3.0],
                },
            ]
        )
    (tmp_path / "guidance_trace.jsonl").write_text(
        "".join(json.dumps(event) + "\n" for event in events), encoding="utf-8"
    )
    config = {
        "experiment": {
            "waypoint_count": 2,
            "waypoint_distance_analysis": {"required": True},
        }
    }
    validation = module.Validation()

    result = module._validate_guidance_trace(tmp_path, config, validation)

    assert validation.errors == []
    assert result["setpoint_count"] == 4
    assert result["transit_waypoint_indices"] == [1, 2]
    assert result["hold_analysis_waypoint_indices"] == [1, 2]


def test_guidance_trace_accepts_position_only_diagnostic_mode(tmp_path: Path) -> None:
    module = _load_module()
    events = [
        {
            "schema_version": "px4_waypoint_guidance_v1",
            "event": "setpoint_sent",
            "guidance_mode": "position_only_final_target",
            "mission_phase": "waypoint_001_transit",
            "waypoint_index": 1,
            "frame": "MAV_FRAME_LOCAL_NED",
            "active_fields": ["position"],
            "type_mask": 3576,
            "target_position_local_ned_m": [1.0, 2.0, -3.0],
            "actual_position_local_ned_m": [0.0, 0.0, 0.0],
        },
        {
            "schema_version": "px4_waypoint_guidance_v1",
            "event": "setpoint_sent",
            "guidance_mode": "position_only_final_target",
            "mission_phase": "waypoint_001_hold_analysis",
            "waypoint_index": 1,
            "frame": "MAV_FRAME_LOCAL_NED",
            "active_fields": ["position"],
            "type_mask": 3576,
            "target_position_local_ned_m": [1.0, 2.0, -3.0],
            "actual_position_local_ned_m": [1.0, 2.0, -3.0],
        },
    ]
    (tmp_path / "guidance_trace.jsonl").write_text(
        "".join(json.dumps(event) + "\n" for event in events), encoding="utf-8"
    )
    config = {
        "experiment": {
            "waypoint_count": 1,
            "guidance_mode": "position_only_final_target",
            "waypoint_distance_analysis": {"required": True},
        }
    }
    validation = module.Validation()

    result = module._validate_guidance_trace(tmp_path, config, validation)

    assert validation.errors == []
    assert result["guidance_mode"] == "position_only_final_target"
    assert result["transit_position_only_setpoint_count"] == 1
    assert result["transit_position_velocity_setpoint_count"] == 0


def test_guidance_trace_accepts_velocity_only_mode(tmp_path: Path) -> None:
    module = _load_module()
    events = [
        {
            "schema_version": "px4_waypoint_guidance_v1",
            "event": "setpoint_sent",
            "guidance_mode": "velocity_only",
            "mission_phase": "waypoint_001_transit",
            "waypoint_index": 1,
            "frame": "MAV_FRAME_LOCAL_NED",
            "active_fields": ["velocity"],
            "type_mask": 3527,
            "target_position_local_ned_m": [1.0, 2.0, -3.0],
            "actual_position_local_ned_m": [0.0, 0.0, 0.0],
        },
        {
            "schema_version": "px4_waypoint_guidance_v1",
            "event": "setpoint_sent",
            "guidance_mode": "velocity_only",
            "mission_phase": "waypoint_001_hold_analysis",
            "waypoint_index": 1,
            "frame": "MAV_FRAME_LOCAL_NED",
            "active_fields": ["position"],
            "type_mask": 3576,
            "target_position_local_ned_m": [1.0, 2.0, -3.0],
            "actual_position_local_ned_m": [1.0, 2.0, -3.0],
        },
    ]
    (tmp_path / "guidance_trace.jsonl").write_text(
        "".join(json.dumps(event) + "\n" for event in events), encoding="utf-8"
    )
    config = {
        "experiment": {
            "waypoint_count": 1,
            "guidance_mode": "velocity_only",
            "waypoint_distance_analysis": {"required": True},
        }
    }
    validation = module.Validation()

    result = module._validate_guidance_trace(tmp_path, config, validation)

    assert validation.errors == []
    assert result["guidance_mode"] == "velocity_only"
    assert result["transit_velocity_only_setpoint_count"] == 1


def test_attitude_distance_comparison_subtracts_waypoint_baseline() -> None:
    module = _load_module()
    baseline = {
        "case_id": "baseline",
        "trajectory": "center",
        "attitude_condition": {"is_baseline": True},
        "waypoint_distance_error": {
            "required": True,
            "waypoints": [
                {
                    "waypoint_index": 1,
                    "primary": {
                        "signed_error_median_m": 0.25,
                        "absolute_error_p95_m": 0.3,
                        "sample_count": 4,
                    },
                }
            ],
        },
    }
    attitude_case = {
        "case_id": "roll-1",
        "trajectory": "center",
        "attitude_condition": {
            "is_baseline": False,
            "axis": "roll",
            "value_deg": 1.0,
        },
        "waypoint_distance_error": {
            "required": True,
            "waypoints": [
                {
                    "waypoint_index": 1,
                    "primary": {
                        "signed_error_median_m": 1.0,
                        "absolute_error_p95_m": 1.2,
                        "sample_count": 5,
                    },
                }
            ],
        },
    }
    errors = []

    comparisons = module._attitude_distance_comparisons(
        [baseline, attitude_case], errors
    )

    assert errors == []
    assert comparisons[0]["waypoints"][0]["attitude_induced_error_delta_m"] == 0.75

    result = {
        "cases": [baseline, attitude_case],
        "attitude_distance_comparisons": comparisons,
    }
    rows = module._waypoint_distance_csv_rows(result)
    assert len(rows) == 2
    assert rows[0]["is_baseline"] is True
    assert rows[0]["attitude_induced_error_delta_m"] == 0.0
    assert rows[1]["baseline_case_id"] == "baseline"
    assert rows[1]["attitude_induced_error_delta_m"] == 0.75


def test_waypoint_distance_csv_writes_stable_header(tmp_path: Path) -> None:
    module = _load_module()
    output = tmp_path / "waypoints.csv"
    result = {
        "case_id": "baseline",
        "trajectory": "center",
        "attitude_condition": {
            "axis": "baseline",
            "value_deg": 0.0,
            "is_baseline": True,
        },
        "waypoint_distance_error": {
            "required": True,
            "waypoints": [
                {
                    "waypoint_index": 1,
                    "primary": {
                        "estimate_field": "nominal_pose_range_m",
                        "ground_truth_field": "gt_range_m",
                        "sample_count": 3,
                        "signed_error_median_m": 0.25,
                    },
                }
            ],
        },
    }

    count = module._write_waypoint_distance_csv(output, result)

    with output.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert count == 1
    assert list(rows[0]) == list(module.WAYPOINT_DISTANCE_CSV_FIELDS)
    assert rows[0]["primary_estimate_field"] == "nominal_pose_range_m"
    assert rows[0]["attitude_induced_error_delta_m"] == "0.0"
