#!/usr/bin/env python3
"""Validate calibration-transfer provenance produced by an attitude sweep."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
import subprocess
import sys
from pathlib import Path
from typing import Any

from sdf_contract import sdf_contract


EXPECTED_RECORD_SCHEMA = "s2_atomic_pair_v11"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
WAYPOINT_DISTANCE_CSV_FIELDS = (
    "trajectory",
    "case_id",
    "axis",
    "value_deg",
    "is_baseline",
    "waypoint_index",
    "primary_estimate_field",
    "ground_truth_field",
    "sample_count",
    "ground_truth_median_m",
    "estimate_median_m",
    "signed_error_mean_m",
    "signed_error_median_m",
    "signed_error_p05_m",
    "signed_error_p95_m",
    "absolute_error_median_m",
    "absolute_error_p95_m",
    "absolute_error_max_m",
    "rmse_m",
    "baseline_case_id",
    "baseline_signed_error_median_m",
    "attitude_induced_error_delta_m",
)

ALWAYS_REQUIRED_COLUMNS = {
    "schema_version",
    "session_id",
    "run_id",
    "attempt_id",
    "case_id",
    "effective_config_sha256",
    "record_sequence",
    "record_type",
    "sample_id",
    "epoch",
    "pair_id",
    "simulation_timestamp_ns",
    "left_observation_timestamp_ns",
    "right_observation_timestamp_ns",
    "left_right_dt_s",
    "left_observation_source",
    "right_observation_source",
    "left_observation_valid",
    "right_observation_valid",
    "operational_position_frame",
    "left_operational_position_valid",
    "right_operational_position_valid",
    "left_operational_pose_valid",
    "right_operational_pose_valid",
    "operational_ray_generation_valid",
    "triangulation_valid",
    "triangulation_status",
    "sample_valid",
    "failure_stage",
    "failure_code",
    "failure_reason",
    "gt_timestamp_ns",
    "gt_valid",
    "gt_source",
    "gt_model_name",
    "gt_link_name",
    "gt_world_enu_x_m",
    "gt_world_enu_y_m",
    "gt_world_enu_z_m",
    "gt_forward_m",
    "gt_right_m",
    "gt_down_m",
    "left_injected_roll_deg",
    "left_injected_pitch_deg",
    "left_injected_yaw_deg",
    "right_injected_roll_deg",
    "right_injected_pitch_deg",
    "right_injected_yaw_deg",
    "sample_eligible",
    "terminal_status",
    "terminal_reason",
    "measurement_status",
    "evaluation_status",
    "observation_mode",
    "output_frame",
    "output_axes",
    "position_unit",
    "pixel_unit",
    "angle_unit",
    "timestamp_unit",
}

OBSERVATION_COLUMNS = {
    "left_u_px",
    "left_v_px",
    "right_u_px",
    "right_v_px",
    "left_confidence",
    "right_confidence",
}

BBOX_COLUMNS = {
    "left_bbox_xmin_px",
    "left_bbox_ymin_px",
    "left_bbox_xmax_px",
    "left_bbox_ymax_px",
    "right_bbox_xmin_px",
    "right_bbox_ymin_px",
    "right_bbox_xmax_px",
    "right_bbox_ymax_px",
}

OPERATIONAL_SOURCE_COLUMNS = {
    "left_operational_position_baseline_center_frd_x_m",
    "left_operational_position_baseline_center_frd_y_m",
    "left_operational_position_baseline_center_frd_z_m",
    "right_operational_position_baseline_center_frd_x_m",
    "right_operational_position_baseline_center_frd_y_m",
    "right_operational_position_baseline_center_frd_z_m",
    "operational_position_source",
    "operational_source_snapshot_timestamp_ns",
    "left_operational_position_timestamp_ns",
    "right_operational_position_timestamp_ns",
    "left_position_alignment_dt_s",
    "right_position_alignment_dt_s",
    "left_operational_attitude_qw",
    "left_operational_attitude_qx",
    "left_operational_attitude_qy",
    "left_operational_attitude_qz",
    "right_operational_attitude_qw",
    "right_operational_attitude_qx",
    "right_operational_attitude_qy",
    "right_operational_attitude_qz",
    "operational_attitude_source",
    "operational_attitude_parent_frame",
    "operational_attitude_child_frame",
    "operational_attitude_quaternion_order",
    "left_operational_pose_timestamp_ns",
    "right_operational_pose_timestamp_ns",
    "left_operational_attitude_timestamp_ns",
    "right_operational_attitude_timestamp_ns",
    "left_pose_alignment_dt_s",
    "right_pose_alignment_dt_s",
    "sync_valid",
}

OPERATIONAL_RESULT_COLUMNS = {
    "operational_x_m",
    "operational_y_m",
    "operational_z_m",
    "operational_range_m",
    "operational_azimuth_deg",
    "operational_elevation_deg",
    "operational_ray_valid",
    "operational_ray_left_origin_x_m",
    "operational_ray_left_origin_y_m",
    "operational_ray_left_origin_z_m",
    "operational_ray_left_direction_x",
    "operational_ray_left_direction_y",
    "operational_ray_left_direction_z",
    "operational_ray_right_origin_x_m",
    "operational_ray_right_origin_y_m",
    "operational_ray_right_origin_z_m",
    "operational_ray_right_direction_x",
    "operational_ray_right_direction_y",
    "operational_ray_right_direction_z",
    "operational_ray_gap_m",
    "operational_ray_intersection_angle_deg",
    "operational_ray_left_depth_m",
    "operational_ray_right_depth_m",
    "operational_ray_condition_number",
    "operational_ray_geometry_rank",
    "operational_ray_left_closest_x_m",
    "operational_ray_left_closest_y_m",
    "operational_ray_left_closest_z_m",
    "operational_ray_right_closest_x_m",
    "operational_ray_right_closest_y_m",
    "operational_ray_right_closest_z_m",
    "near_parallel",
    "negative_left_depth",
    "negative_right_depth",
    "nonfinite_input",
    "nonfinite_output",
    "zero_or_small_baseline",
    "condition_exceeded",
}


class Validation:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []

    def require(self, condition: bool, message: str) -> None:
        if not condition:
            self.errors.append(message)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _canonical_sha256(payload: dict[str, Any]) -> str:
    canonical = json.dumps(
        payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _load_json(path: Path, validation: Validation) -> dict[str, Any]:
    if not path.is_file():
        validation.errors.append(f"missing file: {path}")
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        validation.errors.append(f"invalid JSON {path}: {exc}")
        return {}
    if not isinstance(value, dict):
        validation.errors.append(f"JSON root must be an object: {path}")
        return {}
    return value


def _ratio_float(value: Any) -> float:
    text = str(value or "").strip()
    if "/" in text:
        numerator, denominator = text.split("/", 1)
        denominator_value = float(denominator)
        if denominator_value == 0.0:
            return math.nan
        return float(numerator) / denominator_value
    try:
        return float(text)
    except (TypeError, ValueError):
        return math.nan


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return math.nan
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, float(fraction))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _observed_rate_hz(count: Any, first_elapsed_sec: Any, last_elapsed_sec: Any) -> float:
    try:
        count_value = int(count)
        elapsed = float(last_elapsed_sec) - float(first_elapsed_sec)
    except (TypeError, ValueError):
        return math.nan
    if count_value < 2 or not math.isfinite(elapsed) or elapsed <= 0.0:
        return math.nan
    return float(count_value - 1) / elapsed


def _probe_mp4(path: Path, validation: Validation) -> dict[str, Any]:
    result: dict[str, Any] = {"path": str(path.resolve()), "valid": False}
    if not path.is_file():
        validation.errors.append(f"missing MP4: {path}")
        return result
    size_bytes = path.stat().st_size
    result["size_bytes"] = size_bytes
    validation.require(size_bytes > 0, f"empty MP4: {path}")
    try:
        completed = subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=codec_name,width,height,duration,avg_frame_rate,r_frame_rate,nb_frames:format=duration",
                "-of",
                "json",
                str(path),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=15.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        validation.errors.append(f"ffprobe failed for {path}: {exc}")
        return result
    if completed.returncode != 0:
        validation.errors.append(
            f"ffprobe rejected {path}: {completed.stderr.strip()}"
        )
        return result
    try:
        payload = json.loads(completed.stdout)
        stream = payload.get("streams", [])[0]
        width = int(stream.get("width"))
        height = int(stream.get("height"))
    except (IndexError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        validation.errors.append(f"invalid ffprobe result for {path}: {exc}")
        return result
    duration = math.nan
    for candidate in (stream.get("duration"), payload.get("format", {}).get("duration")):
        try:
            parsed = float(candidate)
        except (TypeError, ValueError):
            continue
        if math.isfinite(parsed):
            duration = parsed
            break
    validation.require(math.isfinite(duration) and duration > 0.0, f"MP4 duration is not positive: {path}")
    validation.require(width > 0 and height > 0, f"MP4 dimensions are invalid: {path}")
    encoded_average_hz = _ratio_float(stream.get("avg_frame_rate"))
    try:
        frame_count = int(stream.get("nb_frames"))
    except (TypeError, ValueError):
        frame_count = 0
    actual_average_hz = (
        float(frame_count) / duration
        if frame_count > 0 and math.isfinite(duration) and duration > 0.0
        else math.nan
    )
    validation.require(
        math.isfinite(encoded_average_hz) and encoded_average_hz > 0.0,
        f"MP4 average frame rate is invalid: {path}",
    )
    validation.require(frame_count > 0, f"MP4 frame count is not positive: {path}")
    validation.require(
        math.isfinite(actual_average_hz) and actual_average_hz > 0.0,
        f"MP4 actual average frame rate is invalid: {path}",
    )
    result.update(
        {
            "valid": (
                math.isfinite(duration)
                and duration > 0.0
                and width > 0
                and height > 0
                and math.isfinite(encoded_average_hz)
                and encoded_average_hz > 0.0
                and frame_count > 0
                and math.isfinite(actual_average_hz)
                and actual_average_hz > 0.0
            ),
            "codec_name": stream.get("codec_name", ""),
            "duration_s": duration,
            "width": width,
            "height": height,
            "encoded_average_frame_rate_hz": encoded_average_hz,
            "frame_count": frame_count,
            "actual_average_frame_rate_hz": actual_average_hz,
        }
    )
    return result


def _validate_steady_yolo_performance(
    operator_dir: Path,
    progress: dict[str, Any],
    acceptance: dict[str, Any],
    validation: Validation,
) -> dict[str, Any]:
    warmup_s = float(acceptance["warmup_s"])
    min_steady_s = float(acceptance["min_steady_duration_s"])
    min_update_hz = float(acceptance["min_yolo_update_hz"])
    max_inference_p95_ms = float(acceptance["max_yolo_inference_p95_ms"])
    csv_name = str(progress.get("csv", ""))
    csv_path = operator_dir / csv_name
    update_samples: list[float] = []
    inference_samples: list[float] = []
    steady_times: list[float] = []
    if not csv_path.is_file():
        validation.errors.append(f"missing operator telemetry CSV: {csv_path}")
    else:
        try:
            with csv_path.open(encoding="utf-8", newline="") as handle:
                for row in csv.DictReader(handle):
                    try:
                        time_sec = float(row.get("time_sec", ""))
                    except (TypeError, ValueError):
                        continue
                    if not math.isfinite(time_sec) or time_sec < warmup_s:
                        continue
                    steady_times.append(time_sec)
                    for field, target in (
                        ("yolo_update_hz", update_samples),
                        ("yolo_inference_ms", inference_samples),
                    ):
                        try:
                            value = float(row.get(field, ""))
                        except (TypeError, ValueError):
                            continue
                        if math.isfinite(value):
                            target.append(value)
        except (OSError, csv.Error) as exc:
            validation.errors.append(f"invalid operator telemetry CSV {csv_path}: {exc}")
    steady_duration_s = (
        max(steady_times) - warmup_s if steady_times else 0.0
    )
    update_median_hz = (
        statistics.median(update_samples) if update_samples else math.nan
    )
    inference_p95_ms = _percentile(inference_samples, 0.95)
    validation.require(
        steady_duration_s >= min_steady_s,
        "operator telemetry steady interval is shorter than the configured minimum",
    )
    validation.require(
        math.isfinite(update_median_hz) and update_median_hz >= min_update_hz,
        "steady YOLO update median is below the configured minimum",
    )
    validation.require(
        math.isfinite(inference_p95_ms)
        and inference_p95_ms <= max_inference_p95_ms,
        "steady YOLO inference p95 exceeds the configured maximum",
    )
    return {
        "warmup_s": warmup_s,
        "steady_duration_s": steady_duration_s,
        "update_sample_count": len(update_samples),
        "inference_sample_count": len(inference_samples),
        "yolo_update_median_hz": update_median_hz,
        "yolo_inference_p95_ms": inference_p95_ms,
        "min_yolo_update_hz": min_update_hz,
        "max_yolo_inference_p95_ms": max_inference_p95_ms,
    }


def _validate_video_runtime_rate(
    name: str,
    progress: dict[str, Any],
    probe: dict[str, Any],
    min_hz: float,
    validation: Validation,
) -> dict[str, Any]:
    stats = progress.get("video_streams", {}).get(name, {})
    frame_count = int(stats.get("frame_count", 0) or 0)
    unique_count = int(stats.get("unique_source_frame_count", 0) or 0)
    callback_hz = _observed_rate_hz(
        frame_count,
        stats.get("first_elapsed_sec"),
        stats.get("last_elapsed_sec"),
    )
    unique_source_hz = _observed_rate_hz(
        unique_count,
        stats.get("first_unique_elapsed_sec"),
        stats.get("last_unique_elapsed_sec"),
    )
    actual_mp4_hz = float(probe.get("actual_average_frame_rate_hz", math.nan))
    validation.require(
        frame_count == int(probe.get("frame_count", -1)),
        f"{name} recorder/MP4 frame counts differ",
    )
    validation.require(
        math.isfinite(callback_hz) and callback_hz >= min_hz,
        f"{name} observed callback rate is below the configured minimum",
    )
    validation.require(
        math.isfinite(unique_source_hz) and unique_source_hz >= min_hz,
        f"{name} unique camera-source rate is below the configured minimum",
    )
    validation.require(
        math.isfinite(actual_mp4_hz) and actual_mp4_hz >= min_hz,
        f"{name} ffprobe actual average frame rate is below the configured minimum",
    )
    return {
        "frame_count": frame_count,
        "unique_source_frame_count": unique_count,
        "observed_callback_hz": callback_hz,
        "unique_source_observed_hz": unique_source_hz,
        "ffprobe_actual_average_frame_rate_hz": actual_mp4_hz,
        "minimum_hz": min_hz,
    }


def _validate_operator_recording(
    case_dir: Path, config: dict[str, Any], validation: Validation
) -> dict[str, Any]:
    experiment = config.get("experiment", {})
    if experiment.get("require_operator_full_recording") is not True:
        return {"required": False}
    operator_dir = case_dir / "operator-recording"
    progress = _load_json(operator_dir / "progress.json", validation)
    validation.require(progress.get("state") == "finalized", "operator recorder did not finalize")
    validation.require(progress.get("mode") == "full", "operator recorder mode is not full")
    left = _probe_mp4(operator_dir / "left_overlay.mp4", validation)
    right = _probe_mp4(operator_dir / "right_overlay.mp4", validation)
    acceptance = experiment.get("performance_acceptance")
    performance: dict[str, Any] = {"required": False}
    video_rates: dict[str, Any] = {"required": False}
    backend_evidence = ""
    if isinstance(acceptance, dict):
        required_fields = {
            "warmup_s",
            "min_steady_duration_s",
            "min_yolo_update_hz",
            "max_yolo_inference_p95_ms",
            "min_overlay_hz",
        }
        validation.require(
            required_fields <= set(acceptance),
            "performance acceptance contract is incomplete",
        )
        if required_fields <= set(acceptance):
            performance = {
                "required": True,
                **_validate_steady_yolo_performance(
                    operator_dir, progress, acceptance, validation
                ),
            }
            min_overlay_hz = float(acceptance["min_overlay_hz"])
            video_rates = {
                "required": True,
                "left": _validate_video_runtime_rate(
                    "left_overlay", progress, left, min_overlay_hz, validation
                ),
                "right": _validate_video_runtime_rate(
                    "right_overlay", progress, right, min_overlay_hz, validation
                ),
            }
            operator_log = case_dir / "operator.log"
            if not operator_log.is_file():
                validation.errors.append(f"missing operator runtime log: {operator_log}")
            else:
                log_text = operator_log.read_text(encoding="utf-8", errors="replace")
                match = re.search(
                    r"YOLO (?:process |prediction )?backend verified: "
                    r"requested=cuda[^,]*, actual=(cuda[^,]*), precision=(fp16)",
                    log_text,
                )
                validation.require(
                    match is not None,
                    "operator log does not prove an actual CUDA FP16 YOLO backend",
                )
                if match is not None:
                    backend_evidence = match.group(0)
    return {
        "required": True,
        "progress_state": progress.get("state"),
        "left": left,
        "right": right,
        "performance": performance,
        "video_rates": video_rates,
        "backend_evidence": backend_evidence,
    }


def _validate_waypoint_stability(
    case_dir: Path, config: dict[str, Any], validation: Validation
) -> dict[str, Any]:
    experiment = config.get("experiment", {})
    position_limit = experiment.get("stability_position_p95_max_m")
    attitude_limit = experiment.get("stability_attitude_max_deg")
    if position_limit is None and attitude_limit is None:
        return {"required": False}
    path = case_dir / "waypoint_stability.jsonl"
    events: list[dict[str, Any]] = []
    if not path.is_file():
        validation.errors.append(f"missing waypoint stability event log: {path}")
        return {"required": True, "events": events}
    try:
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"line {line_number} is not an object")
            events.append(value)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        validation.errors.append(f"invalid waypoint stability event log: {exc}")
        return {"required": True, "events": events}

    holds = [event for event in events if event.get("event") == "waypoint_hold_complete"]
    expected_count = int(experiment.get("waypoint_count", 0))
    validation.require(
        len(holds) == expected_count,
        f"waypoint stability hold count mismatch: expected {expected_count}, got {len(holds)}",
    )
    validation.require(
        [event.get("waypoint_index") for event in holds]
        == list(range(1, expected_count + 1)),
        "waypoint stability indices are incomplete or out of order",
    )
    validation.require(
        any(event.get("event") == "mission_completed" for event in events),
        "waypoint stability log lacks mission_completed",
    )
    for event in holds:
        index = event.get("waypoint_index")
        validation.require(
            event.get("stability_pass") is True,
            f"waypoint {index} stability gate did not pass",
        )
        if position_limit is not None:
            value = event.get("position_error_p95_m")
            validation.require(
                isinstance(value, (int, float))
                and math.isfinite(float(value))
                and float(value) <= float(position_limit),
                f"waypoint {index} position-error p95 exceeds limit",
            )
        if attitude_limit is not None:
            for name in ("max_abs_roll_deg", "max_abs_pitch_deg"):
                value = event.get(name)
                validation.require(
                    isinstance(value, (int, float))
                    and math.isfinite(float(value))
                    and float(value) <= float(attitude_limit),
                    f"waypoint {index} {name} exceeds limit",
                )
    return {"required": True, "event_count": len(events), "hold_count": len(holds)}


def _range_error_statistics(
    rows: list[dict[str, str]], estimate_field: str, ground_truth_field: str
) -> dict[str, Any]:
    estimates: list[float] = []
    ground_truth: list[float] = []
    signed_errors: list[float] = []
    for row in rows:
        if not _finite(row.get(estimate_field, "")) or not _finite(
            row.get(ground_truth_field, "")
        ):
            continue
        estimate = float(row[estimate_field])
        truth = float(row[ground_truth_field])
        estimates.append(estimate)
        ground_truth.append(truth)
        signed_errors.append(estimate - truth)
    absolute_errors = [abs(value) for value in signed_errors]
    return {
        "estimate_field": estimate_field,
        "ground_truth_field": ground_truth_field,
        "signed_error_contract": "estimate_minus_ground_truth_m",
        "sample_count": len(signed_errors),
        "estimate_median_m": (
            statistics.median(estimates) if estimates else math.nan
        ),
        "ground_truth_median_m": (
            statistics.median(ground_truth) if ground_truth else math.nan
        ),
        "signed_error_mean_m": (
            statistics.mean(signed_errors) if signed_errors else math.nan
        ),
        "signed_error_median_m": (
            statistics.median(signed_errors) if signed_errors else math.nan
        ),
        "signed_error_p05_m": _percentile(signed_errors, 0.05),
        "signed_error_p95_m": _percentile(signed_errors, 0.95),
        "absolute_error_median_m": (
            statistics.median(absolute_errors) if absolute_errors else math.nan
        ),
        "absolute_error_p95_m": _percentile(absolute_errors, 0.95),
        "absolute_error_max_m": max(absolute_errors, default=math.nan),
        "rmse_m": (
            math.sqrt(statistics.mean(value * value for value in signed_errors))
            if signed_errors
            else math.nan
        ),
    }


def _validate_waypoint_distance_analysis(
    config: dict[str, Any],
    rows: list[dict[str, str]],
    validation: Validation,
) -> dict[str, Any]:
    experiment = config.get("experiment", {})
    contract = experiment.get("waypoint_distance_analysis")
    if not isinstance(contract, dict):
        return {"required": False}
    required = contract.get("required") is True
    if not required:
        return {"required": False, "contract": contract}

    expected_count = int(experiment.get("waypoint_count", 0))
    phase_pattern = str(
        contract.get("phase_pattern", "waypoint_{index:03d}_hold_analysis")
    )
    primary_field = str(contract.get("primary_estimate_field", ""))
    ground_truth_field = str(contract.get("ground_truth_field", ""))
    control_fields = [
        str(value) for value in contract.get("control_estimate_fields", [])
    ]
    validation.require(
        primary_field == "nominal_pose_range_m",
        "waypoint distance primary field is not nominal_pose_range_m",
    )
    validation.require(
        ground_truth_field == "gt_range_m",
        "waypoint distance ground-truth field is not gt_range_m",
    )

    waypoint_results: list[dict[str, Any]] = []
    for index in range(1, expected_count + 1):
        try:
            phase = phase_pattern.format(index=index)
        except (KeyError, ValueError) as exc:
            validation.errors.append(
                f"invalid waypoint distance phase pattern: {exc}"
            )
            break
        phase_rows = [row for row in rows if row.get("mission_phase") == phase]
        primary = _range_error_statistics(
            phase_rows, primary_field, ground_truth_field
        )
        validation.require(
            primary["sample_count"] > 0,
            f"waypoint {index} has no finite nominal-pose/GT hold-analysis samples",
        )
        controls = {
            field: _range_error_statistics(phase_rows, field, ground_truth_field)
            for field in control_fields
        }
        waypoint_results.append(
            {
                "waypoint_index": index,
                "mission_phase": phase,
                "phase_row_count": len(phase_rows),
                "primary": primary,
                "controls": controls,
            }
        )

    return {
        "required": True,
        "primary_metric": f"{primary_field} - {ground_truth_field}",
        "signed_error_contract": "estimate_minus_ground_truth_m",
        "expected_waypoint_count": expected_count,
        "waypoints": waypoint_results,
    }


def _validate_guidance_trace(
    case_dir: Path, config: dict[str, Any], validation: Validation
) -> dict[str, Any]:
    contract = config.get("experiment", {}).get("waypoint_distance_analysis")
    if not isinstance(contract, dict) or contract.get("required") is not True:
        return {"required": False}
    path = case_dir / "guidance_trace.jsonl"
    if not path.is_file():
        validation.errors.append(f"missing waypoint guidance trace: {path}")
        return {"required": True, "setpoint_count": 0}
    events: list[dict[str, Any]] = []
    try:
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            if not line.strip():
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"line {line_number} is not an object")
            events.append(value)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        validation.errors.append(f"invalid waypoint guidance trace: {exc}")
        return {"required": True, "setpoint_count": len(events)}

    expected_count = int(config.get("experiment", {}).get("waypoint_count", 0))
    expected_guidance_mode = str(
        config.get("experiment", {}).get(
            "guidance_mode", "position_velocity_final_target"
        )
    )
    validation.require(
        expected_guidance_mode
        in (
            "position_velocity_final_target",
            "position_only_final_target",
            "velocity_only",
        ),
        "unsupported waypoint guidance mode in effective config",
    )
    setpoints = [event for event in events if event.get("event") == "setpoint_sent"]
    validation.require(bool(setpoints), "waypoint guidance trace has no setpoints")
    transit_indices: set[int] = set()
    analysis_indices: set[int] = set()
    position_velocity_count = 0
    position_only_count = 0
    transit_position_velocity_count = 0
    transit_position_only_count = 0
    transit_velocity_only_count = 0
    readback_count = 0
    for event in setpoints:
        validation.require(
            event.get("schema_version") == "px4_waypoint_guidance_v1",
            "waypoint guidance trace schema mismatch",
        )
        validation.require(
            event.get("guidance_mode") == expected_guidance_mode,
            "waypoint guidance mode mismatch",
        )
        validation.require(
            event.get("frame") == "MAV_FRAME_LOCAL_NED",
            "waypoint guidance frame mismatch",
        )
        index = event.get("waypoint_index")
        phase = str(event.get("mission_phase", ""))
        if isinstance(index, int) and phase == f"waypoint_{index:03d}_transit":
            transit_indices.add(index)
        if isinstance(index, int) and phase == f"waypoint_{index:03d}_hold_analysis":
            analysis_indices.add(index)
        active_fields = event.get("active_fields")
        if active_fields == ["position", "velocity"]:
            position_velocity_count += 1
            if phase.endswith("_transit"):
                transit_position_velocity_count += 1
            validation.require(
                event.get("type_mask") == 3520,
                "position+velocity guidance type mask mismatch",
            )
        elif active_fields == ["position"]:
            position_only_count += 1
            if phase.endswith("_transit"):
                transit_position_only_count += 1
            validation.require(
                event.get("type_mask") == 3576,
                "position-only guidance type mask mismatch",
            )
        elif active_fields == ["velocity"]:
            if phase.endswith("_transit"):
                transit_velocity_only_count += 1
            validation.require(
                event.get("type_mask") == 3527,
                "velocity-only guidance type mask mismatch",
            )
        else:
            validation.errors.append("waypoint guidance active-fields contract mismatch")
        position = event.get("target_position_local_ned_m")
        validation.require(
            isinstance(position, list)
            and len(position) == 3
            and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in position),
            "waypoint guidance target position is invalid",
        )
        if event.get("actual_position_local_ned_m") is not None:
            readback_count += 1

    expected_indices = set(range(1, expected_count + 1))
    validation.require(
        transit_indices == expected_indices,
        "waypoint guidance transit indices are incomplete",
    )
    validation.require(
        analysis_indices == expected_indices,
        "waypoint guidance hold-analysis indices are incomplete",
    )
    if expected_guidance_mode == "position_velocity_final_target":
        validation.require(
            transit_position_velocity_count > 0,
            "waypoint guidance trace lacks position+velocity transit commands",
        )
    elif expected_guidance_mode == "position_only_final_target":
        validation.require(
            transit_position_only_count > 0,
            "waypoint guidance trace lacks position-only transit commands",
        )
        validation.require(
            transit_position_velocity_count == 0,
            "position-only mode unexpectedly sent velocity feed-forward",
        )
    elif expected_guidance_mode == "velocity_only":
        validation.require(
            transit_velocity_only_count > 0,
            "waypoint guidance trace lacks velocity-only transit commands",
        )
        validation.require(
            transit_position_velocity_count == 0,
            "velocity-only mode unexpectedly sent active position+velocity transit commands",
        )
    validation.require(
        position_only_count > 0,
        "waypoint guidance trace lacks position-only hold commands",
    )
    validation.require(
        readback_count > 0,
        "waypoint guidance trace lacks LOCAL_POSITION_NED readback",
    )
    return {
        "required": True,
        "schema_version": "px4_waypoint_guidance_v1",
        "guidance_mode": expected_guidance_mode,
        "setpoint_count": len(setpoints),
        "position_velocity_setpoint_count": position_velocity_count,
        "position_only_setpoint_count": position_only_count,
        "transit_position_velocity_setpoint_count": transit_position_velocity_count,
        "transit_position_only_setpoint_count": transit_position_only_count,
        "transit_velocity_only_setpoint_count": transit_velocity_only_count,
        "local_position_readback_count": readback_count,
        "transit_waypoint_indices": sorted(transit_indices),
        "hold_analysis_waypoint_indices": sorted(analysis_indices),
    }


def _bool(value: str) -> bool | None:
    normalized = str(value).strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    return None


def _finite(value: str) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _nonempty(row: dict[str, str], fields: set[str]) -> list[str]:
    return sorted(name for name in fields if row.get(name, "") == "")


def _unit_vector(row: dict[str, str], prefix: str) -> bool:
    try:
        norm = math.sqrt(
            sum(float(row[f"{prefix}_{axis}"]) ** 2 for axis in ("x", "y", "z"))
        )
    except (KeyError, TypeError, ValueError):
        return False
    return math.isfinite(norm) and abs(norm - 1.0) <= 1.0e-6


def _unit_quaternion(row: dict[str, str], side: str) -> bool:
    try:
        norm = math.sqrt(
            sum(
                float(row[f"{side}_operational_attitude_q{axis}"]) ** 2
                for axis in ("w", "x", "y", "z")
            )
        )
    except (KeyError, TypeError, ValueError):
        return False
    return math.isfinite(norm) and abs(norm - 1.0) <= 1.0e-6


def _validate_effective_config(
    case_dir: Path, validation: Validation
) -> tuple[dict[str, Any], str]:
    config_path = case_dir / "effective_config.json"
    config = _load_json(config_path, validation)
    stored_hash = str(config.get("effective_config_sha256", ""))
    validation.require(bool(SHA256_RE.fullmatch(stored_hash)), "invalid effective_config_sha256")
    if config:
        unhashed = dict(config)
        unhashed.pop("effective_config_sha256", None)
        validation.require(
            _canonical_sha256(unhashed) == stored_hash,
            "effective_config_sha256 does not match canonical effective config",
        )
    validation.require(config.get("hash_algorithm") == "sha256", "hash algorithm is not sha256")

    coordinate = config.get("coordinate_contract", {})
    expected_coordinate = {
        "gazebo_world": "ENU",
        "px4_local": "NED",
        "px4_body": "FRD",
        "camera_optical": "RDF",
        "output_frame": "baseline_center_frd",
        "output_axes": "forward_right_down",
        "length_unit": "m",
        "angle_unit": "deg",
        "px4_quaternion_order": "wxyz",
        "gazebo_quaternion_order": "xyzw",
    }
    for name, value in expected_coordinate.items():
        validation.require(coordinate.get(name) == value, f"coordinate contract mismatch: {name}")

    camera = config.get("camera", {})
    intrinsic_fields = (
        "image_width_px",
        "image_height_px",
        "horizontal_fov_rad",
        "fx_px",
        "fy_px",
        "cx_px",
        "cy_px",
        "skew_px",
        "distortion_model",
        "distortion_coefficients",
    )
    intrinsic = {name: camera.get(name) for name in intrinsic_fields}
    validation.require(
        _canonical_sha256(intrinsic) == camera.get("intrinsic_profile_sha256"),
        "intrinsic profile hash mismatch",
    )

    rig = config.get("rig", {})
    rig_fields = (
        "left_nominal_origin_baseline_center_frd_m",
        "right_nominal_origin_baseline_center_frd_m",
        "baseline_vector_baseline_center_frd_m",
        "baseline_length_m",
    )
    rig_contract = {name: rig.get(name) for name in rig_fields}
    validation.require(
        _canonical_sha256(rig_contract) == rig.get("profile_sha256"),
        "rig profile hash mismatch",
    )
    try:
        left = rig_contract["left_nominal_origin_baseline_center_frd_m"]
        right = rig_contract["right_nominal_origin_baseline_center_frd_m"]
        vector = [float(right[index]) - float(left[index]) for index in range(3)]
        length = math.sqrt(sum(value * value for value in vector))
        validation.require(
            all(abs(vector[index] - float(rig_contract["baseline_vector_baseline_center_frd_m"][index])) <= 1.0e-9 for index in range(3)),
            "rig baseline vector is inconsistent with optical centers",
        )
        validation.require(length > 0.0, "rig baseline is zero")
        validation.require(
            abs(length - float(rig_contract["baseline_length_m"])) <= 1.0e-9,
            "rig baseline length is inconsistent with optical centers",
        )
        validation.require(
            all(abs(float(left[index]) + float(right[index])) <= 1.0e-9 for index in range(3)),
            "baseline_center_frd origin is not the optical-center midpoint",
        )
    except (KeyError, TypeError, ValueError, IndexError):
        validation.errors.append("rig contract is incomplete or non-numeric")

    ground_truth = config.get("ground_truth", {})
    validation.require(ground_truth.get("source") == "gazebo", "effective config GT source is not Gazebo")
    validation.require(ground_truth.get("link") == "vehicle_reference_link", "effective config GT link mismatch")
    validation.require(ground_truth.get("runtime_feature_allowed") is False, "Gazebo GT is not marked label-only")

    provenance = config.get("provenance", {})
    sdf_path_value = provenance.get("target_model_sdf_path")
    expected_sdf_hash = provenance.get("target_model_sdf_sha256")
    expected_geometry_hash = provenance.get("target_model_protected_geometry_sha256")
    if sdf_path_value or expected_sdf_hash or expected_geometry_hash:
        validation.require(bool(sdf_path_value), "target model SDF path is missing")
        validation.require(
            bool(SHA256_RE.fullmatch(str(expected_sdf_hash or ""))),
            "target model SDF hash is invalid",
        )
        validation.require(
            bool(SHA256_RE.fullmatch(str(expected_geometry_hash or ""))),
            "target model protected geometry hash is invalid",
        )
        if sdf_path_value:
            try:
                actual_sdf = sdf_contract(Path(str(sdf_path_value)))
            except (OSError, ValueError) as exc:
                validation.errors.append(f"cannot validate target model SDF: {exc}")
            else:
                validation.require(
                    actual_sdf["sha256"] == expected_sdf_hash,
                    "target model SDF hash changed after effective config capture",
                )
                validation.require(
                    actual_sdf["protected_geometry_sha256"] == expected_geometry_hash,
                    "target model protected geometry hash mismatch",
                )
    return config, stored_hash


def validate_case(case_dir: Path) -> dict[str, Any]:
    validation = Validation()
    config, config_hash = _validate_effective_config(case_dir, validation)
    recording_dir = case_dir / "recording"
    csv_path = recording_dir / "pair_records.csv"
    jsonl_path = recording_dir / "pair_records.jsonl"
    metadata = _load_json(recording_dir / "run_metadata.json", validation)
    summary = _load_json(recording_dir / "run_summary.json", validation)
    validation.require((recording_dir / "RUN_FINALIZED_VALID").is_file(), "missing RUN_FINALIZED_VALID")
    validation.require(not (recording_dir / "RUN_FINALIZED_INVALID").exists(), "RUN_FINALIZED_INVALID exists")

    case_id = str(config.get("case_id", case_dir.name))
    session_id = str(config.get("session_id", ""))
    run_id = str(config.get("run_id", ""))
    for name, expected in (
        ("schema_version", EXPECTED_RECORD_SCHEMA),
        ("session_id", session_id),
        ("run_id", run_id),
        ("attempt_id", case_id),
        ("case_id", case_id),
        ("effective_config_sha256", config_hash),
    ):
        validation.require(metadata.get(name) == expected, f"run metadata mismatch: {name}")
    validation.require(metadata.get("mission_profile") == "takeoff_forward_land", "mission profile does not identify Auto trajectory")
    validation.require(metadata.get("trajectory_id") == "takeoff_forward_land", "trajectory_id mismatch")
    validation.require(metadata.get("ground_truth_source") == "gazebo_vehicle_reference_link_world_pose", "run metadata GT source mismatch")
    validation.require(metadata.get("ground_truth_link_name") == "vehicle_reference_link", "run metadata GT link mismatch")
    validation.require(metadata.get("ground_truth_fallback_allowed") is False, "GT fallback must be disabled")
    validation.require(
        metadata.get("operational_pose_alignment_method")
        == "latest_valid_px4_sensor_snapshot_at_image_pair_acceptance",
        "operational pose alignment method missing",
    )
    attitude_alignment_limit_s = metadata.get("px4_attitude_max_age_s")
    position_alignment_limit_s = metadata.get("gps_baseline_max_age_s")
    validation.require(
        isinstance(attitude_alignment_limit_s, (int, float))
        and math.isfinite(float(attitude_alignment_limit_s))
        and float(attitude_alignment_limit_s) > 0.0,
        "metadata PX4 attitude alignment limit is invalid",
    )
    validation.require(
        isinstance(position_alignment_limit_s, (int, float))
        and math.isfinite(float(position_alignment_limit_s))
        and float(position_alignment_limit_s) > 0.0,
        "metadata GPS position alignment limit is invalid",
    )
    attitude_alignment_limit_s = float(attitude_alignment_limit_s or 0.0)
    position_alignment_limit_s = float(position_alignment_limit_s or 0.0)
    validation.require(metadata.get("intrinsic_profile_sha256") == config.get("camera", {}).get("intrinsic_profile_sha256"), "metadata/config intrinsic hash mismatch")
    validation.require(metadata.get("rig_profile_sha256") == config.get("rig", {}).get("profile_sha256"), "metadata/config rig hash mismatch")

    validation.require(summary.get("schema_version") == EXPECTED_RECORD_SCHEMA, "summary schema mismatch")
    validation.require(summary.get("run_valid") is True, "run summary is not valid")
    validation.require(summary.get("finalization_status") == "VALID", "run was not finalized VALID")
    validation.require(summary.get("shutdown_drain_status") == "SUCCESS", "shutdown drain did not succeed")
    validation.require(summary.get("jsonl_equals_csv_rows") is True, "JSONL/CSV row counts differ")
    validation.require(summary.get("unique_pair_keys_equal_records") is True, "pair keys are not unique")
    validation.require(summary.get("measurement_queue_overflow_count") == 0, "measurement queue overflowed")
    validation.require(summary.get("research_recording_error_count") == 0, "research recorder reported errors")

    rows: list[dict[str, str]] = []
    header: list[str] = []
    if not csv_path.is_file():
        validation.errors.append(f"missing file: {csv_path}")
    else:
        try:
            with csv_path.open(encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                header = list(reader.fieldnames or ())
                rows = list(reader)
        except (OSError, csv.Error) as exc:
            validation.errors.append(f"invalid CSV {csv_path}: {exc}")
    missing_columns = sorted(
        (ALWAYS_REQUIRED_COLUMNS | OBSERVATION_COLUMNS | BBOX_COLUMNS | OPERATIONAL_SOURCE_COLUMNS | OPERATIONAL_RESULT_COLUMNS)
        - set(header)
    )
    validation.require(not missing_columns, f"missing CSV columns: {missing_columns}")
    validation.require(bool(rows), "pair_records.csv has no samples")
    if csv_path.is_file():
        validation.require(summary.get("pair_records_csv_sha256") == _sha256(csv_path), "CSV hash mismatch")
    if jsonl_path.is_file():
        validation.require(summary.get("pair_records_jsonl_sha256") == _sha256(jsonl_path), "JSONL hash mismatch")
    else:
        validation.errors.append(f"missing file: {jsonl_path}")
    validation.require(summary.get("csv_data_row_count") == len(rows), "summary CSV row count mismatch")
    validation.require(summary.get("jsonl_line_count") == len(rows), "summary JSONL row count mismatch")

    seen_samples: set[str] = set()
    seen_pairs: set[tuple[str, str]] = set()
    eligible_count = 0
    valid_count = 0
    operational_count = 0
    observation_count = 0
    first_dual_observation_world_front_m: float | None = None
    world_rfu_origin = config.get("coordinate_contract", {}).get(
        "world_rfu_origin_world_enu_m", [0.0, 0.0, 0.0]
    )
    try:
        world_rfu_origin_north_m = float(world_rfu_origin[1])
    except (TypeError, ValueError, IndexError):
        world_rfu_origin_north_m = math.nan
    expected_injection = config.get("attitude_injection", {})
    left_injected = expected_injection.get("left_applied_deg", {})
    right_injected = expected_injection.get("right_applied_deg", {})
    for index, row in enumerate(rows, start=1):
        prefix = f"row {index}"
        validation.require(row.get("record_sequence") == str(index), f"{prefix}: record_sequence mismatch")
        validation.require(row.get("schema_version") == EXPECTED_RECORD_SCHEMA, f"{prefix}: schema mismatch")
        validation.require(row.get("session_id") == session_id, f"{prefix}: session_id mismatch")
        validation.require(row.get("run_id") == run_id, f"{prefix}: run_id mismatch")
        validation.require(row.get("attempt_id") == case_id, f"{prefix}: attempt_id mismatch")
        validation.require(row.get("case_id") == case_id, f"{prefix}: case_id mismatch")
        validation.require(row.get("effective_config_sha256") == config_hash, f"{prefix}: config hash mismatch")
        sample_id = row.get("sample_id", "")
        pair_key = (row.get("epoch", ""), row.get("pair_id", ""))
        validation.require(sample_id == f"{pair_key[0]}:{pair_key[1]}", f"{prefix}: sample_id is not epoch:pair_id")
        validation.require(sample_id not in seen_samples, f"{prefix}: duplicate sample_id")
        validation.require(pair_key not in seen_pairs, f"{prefix}: duplicate epoch/pair_id")
        seen_samples.add(sample_id)
        seen_pairs.add(pair_key)

        validation.require(row.get("output_frame") == "baseline_center_frd", f"{prefix}: output frame mismatch")
        validation.require(row.get("output_axes") == "forward_right_down", f"{prefix}: output axes mismatch")
        validation.require(row.get("position_unit") == "m", f"{prefix}: position unit mismatch")
        validation.require(row.get("angle_unit") == "deg", f"{prefix}: angle unit mismatch")
        validation.require(row.get("timestamp_unit") == "ns", f"{prefix}: timestamp unit mismatch")
        validation.require(row.get("gt_valid") == "true", f"{prefix}: Gazebo GT is unavailable")
        validation.require(row.get("gt_source") == "gazebo_vehicle_reference_link_world_pose", f"{prefix}: GT source mismatch")
        validation.require(row.get("gt_link_name") == "vehicle_reference_link", f"{prefix}: GT link mismatch")
        validation.require(not _nonempty(row, {"gt_timestamp_ns", "gt_world_enu_x_m", "gt_world_enu_y_m", "gt_world_enu_z_m", "gt_forward_m", "gt_right_m", "gt_down_m"}), f"{prefix}: GT position/timestamp incomplete")
        validation.require(row.get("terminal_status", "") != "", f"{prefix}: terminal status missing")
        validation.require(row.get("measurement_status", "") != "", f"{prefix}: measurement status missing")
        validation.require(row.get("evaluation_status", "") != "", f"{prefix}: evaluation status missing")

        for side in ("left", "right"):
            for axis in ("roll", "pitch", "yaw"):
                key = f"{side}_injected_{axis}_deg"
                expected = (left_injected if side == "left" else right_injected).get(axis)
                validation.require(
                    _finite(row.get(key, "")) and abs(float(row[key]) - float(expected)) <= 1.0e-12,
                    f"{prefix}: applied injection mismatch in {key}",
                )

        left_observed = _bool(row.get("left_observation_valid", "")) is True
        right_observed = _bool(row.get("right_observation_valid", "")) is True
        if left_observed or right_observed:
            observation_count += 1
        if (
            first_dual_observation_world_front_m is None
            and left_observed
            and right_observed
            and row.get("gt_valid") == "true"
            and _finite(row.get("gt_world_enu_y_m", ""))
            and math.isfinite(world_rfu_origin_north_m)
        ):
            first_dual_observation_world_front_m = (
                float(row["gt_world_enu_y_m"]) - world_rfu_origin_north_m
            )
        if left_observed:
            validation.require(not _nonempty(row, {"left_u_px", "left_v_px", "left_confidence"}), f"{prefix}: left observation incomplete")
        if right_observed:
            validation.require(not _nonempty(row, {"right_u_px", "right_v_px", "right_confidence"}), f"{prefix}: right observation incomplete")
        if row.get("observation_mode") == "bbox_yolo" and left_observed:
            validation.require(
                not _nonempty(
                    row,
                    {
                        "left_bbox_xmin_px",
                        "left_bbox_ymin_px",
                        "left_bbox_xmax_px",
                        "left_bbox_ymax_px",
                    },
                ),
                f"{prefix}: left BBox corners incomplete",
            )
        if row.get("observation_mode") == "bbox_yolo" and right_observed:
            validation.require(
                not _nonempty(
                    row,
                    {
                        "right_bbox_xmin_px",
                        "right_bbox_ymin_px",
                        "right_bbox_xmax_px",
                        "right_bbox_ymax_px",
                    },
                ),
                f"{prefix}: right BBox corners incomplete",
            )

        operational_valid = _bool(row.get("operational_ray_valid", "")) is True
        if operational_valid:
            operational_count += 1
            missing = _nonempty(row, OPERATIONAL_SOURCE_COLUMNS | OPERATIONAL_RESULT_COLUMNS)
            validation.require(not missing, f"{prefix}: Operational source/result incomplete: {missing}")
            validation.require(row.get("operational_position_frame") == "baseline_center_frd", f"{prefix}: Operational position frame mismatch")
            validation.require(row.get("operational_attitude_parent_frame") == "earth_ned", f"{prefix}: Operational attitude parent mismatch")
            validation.require(row.get("operational_attitude_child_frame") == "body_frd", f"{prefix}: Operational attitude child mismatch")
            validation.require(row.get("operational_attitude_quaternion_order") == "wxyz", f"{prefix}: Operational quaternion order mismatch")
            validation.require(_unit_quaternion(row, "left"), f"{prefix}: left quaternion is not unit length")
            validation.require(_unit_quaternion(row, "right"), f"{prefix}: right quaternion is not unit length")
            validation.require(_unit_vector(row, "operational_ray_left_direction"), f"{prefix}: left Operational ray is not unit length")
            validation.require(_unit_vector(row, "operational_ray_right_direction"), f"{prefix}: right Operational ray is not unit length")
            validation.require(row.get("sync_valid") == "true", f"{prefix}: Operational sample is not time-aligned")
            for name, limit_s in (
                ("left_pose_alignment_dt_s", attitude_alignment_limit_s),
                ("right_pose_alignment_dt_s", attitude_alignment_limit_s),
                ("left_position_alignment_dt_s", position_alignment_limit_s),
                ("right_position_alignment_dt_s", position_alignment_limit_s),
            ):
                validation.require(
                    _finite(row.get(name, ""))
                    and abs(float(row[name])) <= limit_s,
                    f"{prefix}: {name} exceeds its recorded alignment limit",
                )
            validation.require(
                row.get("left_operational_attitude_timestamp_ns")
                == row.get("left_operational_pose_timestamp_ns"),
                f"{prefix}: left attitude/pose timestamp alias mismatch",
            )
            validation.require(
                row.get("right_operational_attitude_timestamp_ns")
                == row.get("right_operational_pose_timestamp_ns"),
                f"{prefix}: right attitude/pose timestamp alias mismatch",
            )
            validation.require(row.get("triangulation_valid") == "true", f"{prefix}: valid Operational ray lacks valid triangulation")

        sample_valid = _bool(row.get("sample_valid", "")) is True
        if sample_valid:
            valid_count += 1
            validation.require(operational_valid and left_observed and right_observed, f"{prefix}: sample_valid lacks complete runtime inputs")
        else:
            validation.require(row.get("failure_stage", "") != "", f"{prefix}: invalid sample lacks failure_stage")
            validation.require(row.get("failure_code", "") != "", f"{prefix}: invalid sample lacks failure_code")
            validation.require(row.get("failure_reason", "") != "", f"{prefix}: invalid sample lacks failure_reason")

        if _bool(row.get("sample_eligible", "")) is True:
            eligible_count += 1

    validation.require(eligible_count > 0, "run contains no calibration-eligible samples")
    validation.require(valid_count > 0, "run contains no valid Operational/GT samples")
    validation.require(operational_count > 0, "run contains no Operational rays")
    validation.require(observation_count > 0, "run contains no observations")
    validation.require(summary.get("eligible_sample_count") == eligible_count, "eligible sample count mismatch")

    entry_min_world_front_m = config.get("experiment", {}).get(
        "entry_min_world_front_m"
    )
    if entry_min_world_front_m is not None:
        validation.require(
            first_dual_observation_world_front_m is not None,
            "run contains no dual observation for the world Front entry gate",
        )
        if first_dual_observation_world_front_m is not None:
            validation.require(
                first_dual_observation_world_front_m + 1.0e-6
                >= float(entry_min_world_front_m),
                "first dual observation occurred below the minimum world Front",
            )

    operator_recording = _validate_operator_recording(case_dir, config, validation)
    waypoint_stability = _validate_waypoint_stability(case_dir, config, validation)
    waypoint_guidance = _validate_guidance_trace(case_dir, config, validation)
    waypoint_distance_error = _validate_waypoint_distance_analysis(
        config, rows, validation
    )

    case_result_path = case_dir / "case_result.json"
    if case_result_path.is_file():
        case_result = _load_json(case_result_path, validation)
        validation.require(case_result.get("status") == "PASS", "case_result status is not PASS")
        validation.require(case_result.get("case_id") == case_id, "case_result case_id mismatch")
        validation.require(
            case_result.get("effective_config_sha256") == config_hash,
            "case_result config hash mismatch",
        )
        # 0 is the expected exit status for a successful case:
        # run_px4_waypoint_mission.sh's px4_waypoint_mission.py completes the
        # shared waypoint path and its final hover, then returns normally --
        # it is a deterministic, camera/FOV-independent MAVLink follower with
        # no OFFBOARD hold to interrupt. 130 (128+SIGINT) remains accepted
        # for historical data recorded under the older FOV-guarded
        # px4_auto_mission.py flow, whose success path required SIGINT to end
        # an intentionally-infinite OFFBOARD hold.
        validation.require(
            case_result.get("auto_exit_status") in (0, 130),
            "Auto exit status is neither 0 nor 130 (clean SIGINT stop)",
        )
        validation.require(not case_result.get("failure_stage"), "successful case has a failure_stage")
        validation.require(not case_result.get("failure_reason"), "successful case has a failure_reason")

    return {
        "valid": not validation.errors,
        "case_id": case_id,
        "trajectory": config.get("experiment", {}).get("trajectory", ""),
        "attitude_condition": {
            "axis": config.get("experiment", {}).get("sweep_axis", ""),
            "value_deg": config.get("experiment", {}).get("sweep_value_deg"),
            "is_baseline": config.get("experiment", {}).get(
                "is_baseline_case", False
            ),
            "left_applied_deg": config.get("attitude_injection", {}).get(
                "left_applied_deg", {}
            ),
            "right_applied_deg": config.get("attitude_injection", {}).get(
                "right_applied_deg", {}
            ),
        },
        "record_schema": EXPECTED_RECORD_SCHEMA,
        "effective_config_sha256": config_hash,
        "sample_count": len(rows),
        "eligible_sample_count": eligible_count,
        "valid_sample_count": valid_count,
        "operational_ray_count": operational_count,
        "observation_count": observation_count,
        "first_dual_observation_world_front_m": first_dual_observation_world_front_m,
        "target_model_sdf_sha256": config.get("provenance", {}).get(
            "target_model_sdf_sha256", ""
        ),
        "target_model_protected_geometry_sha256": config.get("provenance", {}).get(
            "target_model_protected_geometry_sha256", ""
        ),
        "operator_recording": operator_recording,
        "waypoint_stability": waypoint_stability,
        "waypoint_guidance": waypoint_guidance,
        "waypoint_distance_error": waypoint_distance_error,
        "errors": validation.errors,
        "warnings": validation.warnings,
    }


def _condition_count(sweep_dir: Path) -> int | None:
    contract = sweep_dir / "sweep_contract.txt"
    if not contract.is_file():
        return None
    for line in contract.read_text(encoding="utf-8").splitlines():
        if line.startswith("condition_count="):
            try:
                return int(line.split("=", 1)[1])
            except ValueError:
                return None
    return None


def _contract_int(sweep_dir: Path, key: str) -> int | None:
    contract = sweep_dir / "sweep_contract.txt"
    if not contract.is_file():
        return None
    for line in contract.read_text(encoding="utf-8").splitlines():
        if line.startswith(f"{key}="):
            try:
                return int(line.split("=", 1)[1])
            except ValueError:
                return None
    return None


def _waypoint_primary_map(result: dict[str, Any]) -> dict[int, dict[str, Any]]:
    analysis = result.get("waypoint_distance_error", {})
    if analysis.get("required") is not True:
        return {}
    mapped: dict[int, dict[str, Any]] = {}
    for waypoint in analysis.get("waypoints", []):
        try:
            index = int(waypoint["waypoint_index"])
        except (KeyError, TypeError, ValueError):
            continue
        primary = waypoint.get("primary")
        if isinstance(primary, dict):
            mapped[index] = primary
    return mapped


def _attitude_distance_comparisons(
    results: list[dict[str, Any]], errors: list[str]
) -> list[dict[str, Any]]:
    comparisons: list[dict[str, Any]] = []
    trajectories = sorted(
        {str(result.get("trajectory", "")) for result in results if result.get("trajectory")}
    )
    for trajectory in trajectories:
        trajectory_results = [
            result for result in results if result.get("trajectory") == trajectory
        ]
        baselines = [
            result
            for result in trajectory_results
            if result.get("attitude_condition", {}).get("is_baseline") is True
        ]
        if len(baselines) != 1:
            errors.append(
                f"trajectory {trajectory} does not have exactly one attitude baseline"
            )
            continue
        baseline = baselines[0]
        baseline_waypoints = _waypoint_primary_map(baseline)
        for result in trajectory_results:
            condition = result.get("attitude_condition", {})
            if condition.get("is_baseline") is True:
                continue
            case_waypoints = _waypoint_primary_map(result)
            waypoint_comparisons: list[dict[str, Any]] = []
            for index in sorted(set(baseline_waypoints) | set(case_waypoints)):
                baseline_primary = baseline_waypoints.get(index)
                case_primary = case_waypoints.get(index)
                if baseline_primary is None or case_primary is None:
                    errors.append(
                        f"missing waypoint {index} distance evidence for attitude comparison: "
                        f"{result.get('case_id', '')}"
                    )
                    continue
                baseline_error = baseline_primary.get("signed_error_median_m")
                case_error = case_primary.get("signed_error_median_m")
                if not isinstance(baseline_error, (int, float)) or not isinstance(
                    case_error, (int, float)
                ) or not math.isfinite(float(baseline_error)) or not math.isfinite(
                    float(case_error)
                ):
                    errors.append(
                        f"nonfinite waypoint {index} median error in attitude comparison: "
                        f"{result.get('case_id', '')}"
                    )
                    continue
                waypoint_comparisons.append(
                    {
                        "waypoint_index": index,
                        "baseline_signed_error_median_m": float(baseline_error),
                        "case_signed_error_median_m": float(case_error),
                        "attitude_induced_error_delta_m": float(case_error)
                        - float(baseline_error),
                        "case_absolute_error_p95_m": case_primary.get(
                            "absolute_error_p95_m"
                        ),
                        "case_sample_count": case_primary.get("sample_count"),
                    }
                )
            comparisons.append(
                {
                    "trajectory": trajectory,
                    "baseline_case_id": baseline.get("case_id", ""),
                    "case_id": result.get("case_id", ""),
                    "axis": condition.get("axis", ""),
                    "value_deg": condition.get("value_deg"),
                    "left_applied_deg": condition.get("left_applied_deg", {}),
                    "right_applied_deg": condition.get("right_applied_deg", {}),
                    "waypoints": waypoint_comparisons,
                }
            )
    return comparisons


def _waypoint_distance_csv_rows(result: dict[str, Any]) -> list[dict[str, Any]]:
    cases = result.get("cases")
    if not isinstance(cases, list):
        cases = [result]
    comparison_lookup: dict[tuple[str, int], dict[str, Any]] = {}
    for comparison in result.get("attitude_distance_comparisons", []):
        case_id = str(comparison.get("case_id", ""))
        for waypoint in comparison.get("waypoints", []):
            try:
                index = int(waypoint["waypoint_index"])
            except (KeyError, TypeError, ValueError):
                continue
            comparison_lookup[(case_id, index)] = {
                **waypoint,
                "baseline_case_id": comparison.get("baseline_case_id", ""),
            }

    rows: list[dict[str, Any]] = []
    for case in cases:
        condition = case.get("attitude_condition", {})
        case_id = str(case.get("case_id", ""))
        is_baseline = condition.get("is_baseline") is True
        analysis = case.get("waypoint_distance_error", {})
        for waypoint in analysis.get("waypoints", []):
            primary = waypoint.get("primary", {})
            try:
                index = int(waypoint["waypoint_index"])
            except (KeyError, TypeError, ValueError):
                continue
            comparison = comparison_lookup.get((case_id, index), {})
            signed_median = primary.get("signed_error_median_m")
            rows.append(
                {
                    "trajectory": case.get("trajectory", ""),
                    "case_id": case_id,
                    "axis": condition.get("axis", ""),
                    "value_deg": condition.get("value_deg"),
                    "is_baseline": is_baseline,
                    "waypoint_index": index,
                    "primary_estimate_field": primary.get("estimate_field", ""),
                    "ground_truth_field": primary.get("ground_truth_field", ""),
                    "sample_count": primary.get("sample_count"),
                    "ground_truth_median_m": primary.get("ground_truth_median_m"),
                    "estimate_median_m": primary.get("estimate_median_m"),
                    "signed_error_mean_m": primary.get("signed_error_mean_m"),
                    "signed_error_median_m": signed_median,
                    "signed_error_p05_m": primary.get("signed_error_p05_m"),
                    "signed_error_p95_m": primary.get("signed_error_p95_m"),
                    "absolute_error_median_m": primary.get("absolute_error_median_m"),
                    "absolute_error_p95_m": primary.get("absolute_error_p95_m"),
                    "absolute_error_max_m": primary.get("absolute_error_max_m"),
                    "rmse_m": primary.get("rmse_m"),
                    "baseline_case_id": (
                        case_id if is_baseline else comparison.get("baseline_case_id", "")
                    ),
                    "baseline_signed_error_median_m": (
                        signed_median
                        if is_baseline
                        else comparison.get("baseline_signed_error_median_m")
                    ),
                    "attitude_induced_error_delta_m": (
                        0.0
                        if is_baseline
                        else comparison.get("attitude_induced_error_delta_m")
                    ),
                }
            )
    return rows


def _write_waypoint_distance_csv(path: Path, result: dict[str, Any]) -> int:
    rows = _waypoint_distance_csv_rows(result)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=WAYPOINT_DISTANCE_CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return len(rows)


def validate_sweep(sweep_dir: Path) -> dict[str, Any]:
    errors: list[str] = []
    manifest_path = sweep_dir / "sweep_manifest.tsv"
    manifest_rows: list[dict[str, str]] = []
    if not manifest_path.is_file():
        errors.append(f"missing file: {manifest_path}")
    else:
        with manifest_path.open(encoding="utf-8", newline="") as handle:
            manifest_rows = list(csv.DictReader(handle, delimiter="\t"))
    condition_count = _condition_count(sweep_dir)
    # A trajectory-specific baseline runs once up front, followed by one case
    # per non-zero condition for that trajectory. New sweeps record the exact
    # expected count; old one-trajectory sweeps retain the legacy fallback.
    expected_case_count = _contract_int(sweep_dir, "expected_case_count")
    if expected_case_count is None:
        expected_case_count = None if condition_count is None else condition_count + 1
    if expected_case_count is None:
        errors.append("condition_count is missing from sweep_contract.txt")
    elif len(manifest_rows) != expected_case_count:
        errors.append(
            f"manifest case count mismatch: expected {expected_case_count}, got {len(manifest_rows)}"
        )
    case_ids = [row.get("case_id", "") for row in manifest_rows]
    if len(set(case_ids)) != len(case_ids):
        errors.append("manifest contains duplicate case IDs")
    results = []
    for row in manifest_rows:
        case_id = row.get("case_id", "")
        if row.get("status") != "PASS":
            errors.append(f"manifest case is not PASS: {case_id}")
        result = validate_case(sweep_dir / case_id)
        results.append(result)
        if not result["valid"]:
            errors.append(f"case validation failed: {case_id}")
        manifest_trajectory = row.get("trajectory", "")
        if manifest_trajectory and manifest_trajectory != result.get("trajectory"):
            errors.append(f"manifest trajectory mismatch: {case_id}")
        if row.get("effective_config_sha256") != result.get("effective_config_sha256"):
            errors.append(f"manifest config hash mismatch: {case_id}")
        if row.get("target_model_sdf_sha256") and row.get(
            "target_model_sdf_sha256"
        ) != result.get("target_model_sdf_sha256"):
            errors.append(f"manifest target model SDF hash mismatch: {case_id}")
    sdf_hashes = {
        str(result.get("target_model_sdf_sha256"))
        for result in results
        if result.get("target_model_sdf_sha256")
    }
    if len(sdf_hashes) > 1:
        errors.append("cases were recorded with different target model SDF hashes")
    protected_geometry_hashes = {
        str(result.get("target_model_protected_geometry_sha256"))
        for result in results
        if result.get("target_model_protected_geometry_sha256")
    }
    if len(protected_geometry_hashes) > 1:
        errors.append("cases were recorded with different protected geometry hashes")
    attitude_distance_comparisons = _attitude_distance_comparisons(results, errors)
    return {
        "valid": not errors,
        "sweep_directory": str(sweep_dir.resolve()),
        "expected_case_count": expected_case_count,
        "case_count": len(manifest_rows),
        "passed_case_count": sum(row.get("status") == "PASS" for row in manifest_rows),
        "total_sample_count": sum(int(result.get("sample_count", 0)) for result in results),
        "total_eligible_sample_count": sum(int(result.get("eligible_sample_count", 0)) for result in results),
        "total_valid_sample_count": sum(int(result.get("valid_sample_count", 0)) for result in results),
        "target_model_sdf_sha256": next(iter(sdf_hashes), ""),
        "target_model_protected_geometry_sha256": next(
            iter(protected_geometry_hashes), ""
        ),
        "primary_waypoint_distance_metric": "nominal_pose_range_m - gt_range_m",
        "attitude_distance_comparisons": attitude_distance_comparisons,
        "errors": errors,
        "cases": results,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--case-dir", type=Path)
    group.add_argument("--sweep-dir", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--waypoint-report", type=Path)
    args = parser.parse_args()
    result = (
        validate_case(args.case_dir.resolve())
        if args.case_dir is not None
        else validate_sweep(args.sweep_dir.resolve())
    )
    if args.waypoint_report is not None:
        row_count = _write_waypoint_distance_csv(args.waypoint_report, result)
        result["waypoint_distance_csv"] = {
            "path": str(args.waypoint_report.resolve()),
            "row_count": row_count,
        }
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if result.get("valid") else 1


if __name__ == "__main__":
    sys.exit(main())
