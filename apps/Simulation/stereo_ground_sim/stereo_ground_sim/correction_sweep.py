"""Deterministic planning and validation for Simulation correction sweeps.

The module is intentionally ROS-free.  It reuses the production coordinate
and SO(3) helpers, while the repository CLI owns child processes and artifacts.
All public spatial values use ``baseline_center_frd`` (Forward, Right, Down).
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import hashlib
import json
import math
from pathlib import Path
import random
from typing import Any, Mapping, Sequence

from .coordinate_frames import (
    apply_optical_rpy_error,
    build_baseline_center_frd,
    effective_optical_rpy_errors_deg,
    nominal_camera_poses,
    project_world_point_to_pixel,
)
from .research_recording import PAIR_RECORD_COLUMNS, RESEARCH_SCHEMA_VERSION


S2_SWEEP_SCHEMA_VERSION = "s2_correction_sweep_v2"
ATTITUDE_AXES = (
    "cam_l_optical_roll_deg",
    "cam_l_optical_pitch_deg",
    "cam_l_optical_yaw_deg",
    "cam_r_optical_roll_deg",
    "cam_r_optical_pitch_deg",
    "cam_r_optical_yaw_deg",
    "common_optical_roll_error_deg",
    "relative_optical_roll_error_deg",
    "common_optical_pitch_error_deg",
    "relative_optical_pitch_error_deg",
    "common_optical_yaw_error_deg",
    "relative_optical_yaw_error_deg",
)
TERMINAL_STATUSES = frozenset(
    {
        "evaluated",
        "gt_unavailable",
        "evaluation_error",
        "measurement_processing_error",
        "measurement_queue_overflow",
        "image_decode_failure",
        "ideal_projection_rejected",
        "old_epoch_result_discarded",
        "simulation_reset_discarded",
        "target_respawned_discarded",
        "shutdown_discarded",
    }
)
MEASUREMENT_STATUSES = frozenset({"processed", "discarded"})
EVALUATION_STATUSES = frozenset(
    {"evaluated", "unavailable", "evaluation_error", "not_evaluated"}
)
CASE_CSV_COLUMNS = (
    "case_id",
    "attempt",
    "case_valid",
    "failure_reason",
    "range_m",
    "bearing_deg",
    "elevation_deg",
    *ATTITUDE_AXES,
    "effective_left_optical_roll_error_deg",
    "effective_left_optical_pitch_error_deg",
    "effective_left_optical_yaw_error_deg",
    "effective_right_optical_roll_error_deg",
    "effective_right_optical_pitch_error_deg",
    "effective_right_optical_yaw_error_deg",
    "analysis_sample_count",
    "total_eligible_count",
    "warmup_excluded_count",
    "invalid_record_count",
    "raw_position_error_median",
    "raw_position_error_p95",
    "raw_position_error_p99",
    "raw_position_error_max",
    "raw_forward_abs_error_median",
    "raw_right_abs_error_median",
    "raw_down_abs_error_median",
    "raw_range_abs_error_median",
    "raw_ray_gap_median",
    "raw_ray_gap_p99",
    "raw_intersection_angle_median_deg",
    "raw_intersection_angle_min_deg",
    "corrected_position_error_median",
    "corrected_position_error_p95",
    "corrected_position_error_p99",
    "corrected_position_error_max",
    "corrected_forward_abs_error_median",
    "corrected_right_abs_error_median",
    "corrected_down_abs_error_median",
    "corrected_range_abs_error_median",
    "corrected_ray_gap_median",
    "corrected_ray_gap_p99",
    "corrected_intersection_angle_median_deg",
    "corrected_intersection_angle_min_deg",
    "corrected_raw_median_ratio",
    "corrected_improvement_percent",
    "requested_range_m",
    "requested_bearing_deg",
    "requested_elevation_deg",
    "actual_range_m_median",
    "actual_bearing_deg_median",
    "actual_elevation_deg_median",
    "actual_range_m_min",
    "actual_range_m_max",
    "actual_bearing_deg_min",
    "actual_bearing_deg_max",
    "actual_elevation_deg_min",
    "actual_elevation_deg_max",
    "spatial_range_error_abs_max_m",
    "spatial_bearing_error_abs_max_deg",
    "spatial_elevation_error_abs_max_deg",
    "spatial_range_tolerance_m",
    "spatial_angle_tolerance_deg",
    "spatial_validation_pass",
    "spatial_validation_sample_count",
)


def canonical_json(payload: Any, *, indent: int | None = None) -> str:
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        allow_nan=False,
        separators=(",", ":") if indent is None else None,
        indent=indent,
    )


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write_json(path: Path, payload: Mapping[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    data = canonical_json(dict(payload), indent=2) + "\n"
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        handle.write(data)
        handle.flush()
    temporary.replace(path)


def _decimal(value: str | int | float | Decimal, name: str) -> Decimal:
    try:
        parsed = Decimal(str(value).strip())
    except (InvalidOperation, ValueError) as exc:
        raise ValueError(f"{name} must be a finite decimal") from exc
    if not parsed.is_finite():
        raise ValueError(f"{name} must be a finite decimal")
    return Decimal(0) if parsed == 0 else parsed.normalize()


def decimal_text(value: Decimal) -> str:
    value = Decimal(0) if value == 0 else value.normalize()
    text = format(value, "f")
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return "0" if text in {"", "-0"} else text


def decimal_range(start: Decimal, stop: Decimal, step: Decimal) -> tuple[Decimal, ...]:
    if step <= 0:
        raise ValueError("range step must be greater than zero")
    if start > stop:
        raise ValueError("range start must be less than or equal to stop")
    count = int((stop - start) // step)
    values = [start + step * index for index in range(count + 1)]
    if values[-1] != stop:
        raise ValueError("range endpoint must be exactly reachable by step")
    return tuple(Decimal(0) if value == 0 else value.normalize() for value in values)


def parse_numeric_values(specification: str, *, name: str) -> tuple[Decimal, ...]:
    specification = str(specification).strip()
    if not specification:
        raise ValueError(f"{name} must not be empty")
    if "," in specification:
        raw = [_decimal(item, name) for item in specification.split(",")]
    elif specification.count(":") == 2:
        start, stop, step = (_decimal(item, name) for item in specification.split(":"))
        raw = list(decimal_range(start, stop, step))
    elif ":" in specification:
        raise ValueError(f"{name} range must be START:STOP:STEP")
    else:
        raw = [_decimal(specification, name)]
    unique: list[Decimal] = []
    seen: set[str] = set()
    for value in raw:
        key = decimal_text(value)
        if key not in seen:
            seen.add(key)
            unique.append(value)
    return tuple(unique)


def parse_attitude_axis_range(specification: str) -> tuple[str, tuple[Decimal, ...]]:
    parts = str(specification).split(":")
    if len(parts) != 4:
        raise ValueError("attitude range must be AXIS:START:STOP:STEP")
    axis = parts[0].strip()
    if axis not in ATTITUDE_AXES:
        raise ValueError(f"unknown attitude axis: {axis}")
    return axis, decimal_range(
        _decimal(parts[1], axis), _decimal(parts[2], axis), _decimal(parts[3], axis)
    )


def _zero_attitude() -> dict[str, Decimal]:
    return {axis: Decimal(0) for axis in ATTITUDE_AXES}


def build_attitude_cases(
    axis_ranges: Sequence[tuple[str, Sequence[Decimal]]], *, mode: str
) -> tuple[dict[str, Decimal], ...]:
    if mode not in {"ofat", "cartesian"}:
        raise ValueError("attitude mode must be ofat or cartesian")
    ordered: list[tuple[str, tuple[Decimal, ...]]] = []
    seen_axes: set[str] = set()
    for axis, values in axis_ranges:
        if axis not in ATTITUDE_AXES:
            raise ValueError(f"unknown attitude axis: {axis}")
        if axis in seen_axes:
            raise ValueError(f"duplicate attitude axis range: {axis}")
        seen_axes.add(axis)
        ordered.append((axis, tuple(values)))
    cases: list[dict[str, Decimal]] = [_zero_attitude()]
    if mode == "ofat":
        for axis, values in ordered:
            for value in values:
                if value == 0:
                    continue
                item = _zero_attitude()
                item[axis] = value
                cases.append(item)
    else:
        products: list[dict[str, Decimal]] = [_zero_attitude()]
        for axis, values in ordered:
            next_products: list[dict[str, Decimal]] = []
            for current in products:
                for value in values:
                    item = dict(current)
                    item[axis] = value
                    next_products.append(item)
            products = next_products
        cases.extend(item for item in products if any(value != 0 for value in item.values()))
    unique: list[dict[str, Decimal]] = []
    seen: set[tuple[str, ...]] = set()
    for item in cases:
        key = tuple(decimal_text(item[axis]) for axis in ATTITUDE_AXES)
        if key not in seen:
            seen.add(key)
            unique.append(item)
    return tuple(unique)


def effective_rpy(attitude: Mapping[str, Decimal | float]) -> tuple[tuple[float, ...], tuple[float, ...]]:
    left = tuple(float(attitude[name]) for name in ATTITUDE_AXES[:3])
    right = tuple(float(attitude[name]) for name in ATTITUDE_AXES[3:6])
    common = tuple(
        float(attitude[name])
        for name in (
            "common_optical_roll_error_deg",
            "common_optical_pitch_error_deg",
            "common_optical_yaw_error_deg",
        )
    )
    relative = tuple(
        float(attitude[name])
        for name in (
            "relative_optical_roll_error_deg",
            "relative_optical_pitch_error_deg",
            "relative_optical_yaw_error_deg",
        )
    )
    return effective_optical_rpy_errors_deg(left, right, common, relative)


def spatial_frd(range_m: float, bearing_deg: float, elevation_deg: float) -> tuple[float, float, float]:
    values = (float(range_m), float(bearing_deg), float(elevation_deg))
    if not all(math.isfinite(value) for value in values) or values[0] <= 0.0:
        raise ValueError("spatial range must be positive and all values finite")
    bearing = math.radians(values[1])
    elevation = math.radians(values[2])
    return (
        values[0] * math.cos(elevation) * math.cos(bearing),
        values[0] * math.cos(elevation) * math.sin(bearing),
        -values[0] * math.sin(elevation),
    )


def spatial_from_frd(point: Sequence[float]) -> tuple[float, float, float]:
    forward, right, down = (float(value) for value in point)
    radius = math.sqrt(forward * forward + right * right + down * down)
    if radius <= 0.0 or not math.isfinite(radius):
        raise ValueError("FRD point must have positive finite range")
    return (
        radius,
        math.degrees(math.atan2(right, forward)),
        math.degrees(math.atan2(-down, math.hypot(forward, right))),
    )


def wrapped_angle_error_deg(actual_deg: float, requested_deg: float) -> float:
    if not math.isfinite(actual_deg) or not math.isfinite(requested_deg):
        raise ValueError("angles must be finite")
    return (actual_deg - requested_deg + 180.0) % 360.0 - 180.0


def validate_spatial_tolerances(
    range_tolerance_m: float, angle_tolerance_deg: float
) -> None:
    if not all(math.isfinite(value) for value in (range_tolerance_m, angle_tolerance_deg)):
        raise ValueError("spatial tolerances must be finite")
    if not 0.0 < range_tolerance_m <= 0.01:
        raise ValueError("spatial range tolerance must be within (0, 0.01] m")
    if not 0.0 < angle_tolerance_deg <= 0.001:
        raise ValueError("spatial angle tolerance must be within (0, 0.001] deg")


@dataclass(frozen=True)
class CameraContract:
    baseline_m: float = 25.0
    camera_height_m: float = 5.0
    horizontal_fov_rad: float = 0.154997710698
    image_width_px: int = 1280
    image_height_px: int = 720
    camera_update_rate_hz: float = 20.0
    camera_clip_near_m: float = 0.1
    camera_clip_far_m: float = 2500.0

    def validate(self) -> None:
        numeric = (
            self.baseline_m,
            self.camera_height_m,
            self.horizontal_fov_rad,
            self.camera_update_rate_hz,
            self.camera_clip_near_m,
            self.camera_clip_far_m,
        )
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("camera contract values must be finite")
        if self.baseline_m <= 0 or self.horizontal_fov_rad <= 0:
            raise ValueError("baseline and FOV must be positive")
        if self.image_width_px <= 0 or self.image_height_px <= 0:
            raise ValueError("image dimensions must be positive")
        if not 0 < self.camera_clip_near_m < self.camera_clip_far_m:
            raise ValueError("camera clip contract is invalid")

    def payload(self) -> dict[str, Any]:
        return {
            "baseline_m": self.baseline_m,
            "camera_height_m": self.camera_height_m,
            "horizontal_fov_rad": self.horizontal_fov_rad,
            "image_width_px": self.image_width_px,
            "image_height_px": self.image_height_px,
            "camera_update_rate_hz": self.camera_update_rate_hz,
            "camera_clip_near_m": self.camera_clip_near_m,
            "camera_clip_far_m": self.camera_clip_far_m,
        }


def _baseline(contract: CameraContract):
    nominal_left, nominal_right = nominal_camera_poses(
        contract.baseline_m, contract.camera_height_m
    )
    frame = build_baseline_center_frd(
        nominal_left.center_world_enu_m,
        nominal_right.center_world_enu_m,
        nominal_left.forward_world_enu,
        nominal_right.forward_world_enu,
    )
    return nominal_left, nominal_right, frame


def fov_preflight(
    *,
    contract: CameraContract,
    attitude: Mapping[str, Decimal | float],
    point_frd: Sequence[float],
) -> dict[str, Any]:
    contract.validate()
    nominal_left, nominal_right, frame = _baseline(contract)
    world = frame.point_to_world(point_frd)
    left_rpy, right_rpy = effective_rpy(attitude)
    left = apply_optical_rpy_error(
        nominal_left, *(math.radians(value) for value in left_rpy)
    )
    right = apply_optical_rpy_error(
        nominal_right, *(math.radians(value) for value in right_rpy)
    )
    fx = (contract.image_width_px * 0.5) / math.tan(contract.horizontal_fov_rad * 0.5)
    pixels = []
    depths = []
    reasons = []
    for side, camera in (("left", left), ("right", right)):
        rdf = camera.world_to_rdf(world)
        depth = float(rdf[2])
        pixel = project_world_point_to_pixel(
            world,
            camera,
            fx_px=fx,
            fy_px=fx,
            cx_px=contract.image_width_px * 0.5,
            cy_px=contract.image_height_px * 0.5,
        )
        depths.append(depth)
        pixels.append(pixel)
        if depth <= contract.camera_clip_near_m:
            reasons.append(f"{side}_behind_or_clip_near")
        if depth >= contract.camera_clip_far_m:
            reasons.append(f"{side}_clip_far")
        if pixel is None:
            reasons.append(f"{side}_projection_invalid")
        elif not (
            0.0 <= pixel[0] < contract.image_width_px
            and 0.0 <= pixel[1] < contract.image_height_px
        ):
            reasons.append(f"{side}_pixel_out_of_bounds")
    return {
        "valid": not reasons,
        "reasons": reasons,
        "target_world_enu_m": world,
        "left_pixel_px": pixels[0],
        "right_pixel_px": pixels[1],
        "left_forward_depth_m": depths[0],
        "right_forward_depth_m": depths[1],
        "effective_left_rpy_deg": left_rpy,
        "effective_right_rpy_deg": right_rpy,
        "baseline_origin_world_enu_m": frame.origin_world_enu_m,
        "baseline_rotation_baseline_from_world": frame.rotation_baseline_from_world,
    }


def _attitude_sort_key(attitude: Mapping[str, Decimal]) -> tuple[str, ...]:
    return tuple(decimal_text(attitude[name]) for name in ATTITUDE_AXES)


def _case_payload(
    *,
    attitude: Mapping[str, Decimal],
    range_value: Decimal,
    bearing_value: Decimal,
    elevation_value: Decimal,
    repetition_index: int,
    seed: int,
    contract: CameraContract,
    spatial_range_tolerance_m: float,
    spatial_angle_tolerance_deg: float,
) -> tuple[dict[str, Any], dict[str, Any]]:
    point_frd = spatial_frd(float(range_value), float(bearing_value), float(elevation_value))
    preflight = fov_preflight(contract=contract, attitude=attitude, point_frd=point_frd)
    world = preflight["target_world_enu_m"]
    left_rpy = preflight["effective_left_rpy_deg"]
    right_rpy = preflight["effective_right_rpy_deg"]
    payload = {
        "schema_version": S2_SWEEP_SCHEMA_VERSION,
        "observation_mode": "ideal_reference_point",
        "trajectory_mode": "static_gazebo_reference_point",
        "target_physics_mode": "static_model",
        "target_static_for_research": True,
        "spatial_range_tolerance_m": spatial_range_tolerance_m,
        "spatial_angle_tolerance_deg": spatial_angle_tolerance_deg,
        "attitude_source_deg": {
            name: {"decimal": decimal_text(attitude[name]), "value": float(attitude[name])}
            for name in ATTITUDE_AXES
        },
        "effective_left_rpy_deg": list(left_rpy),
        "effective_right_rpy_deg": list(right_rpy),
        "spatial": {
            "range_m": {"decimal": decimal_text(range_value), "value": float(range_value)},
            "bearing_deg": {"decimal": decimal_text(bearing_value), "value": float(bearing_value)},
            "elevation_deg": {"decimal": decimal_text(elevation_value), "value": float(elevation_value)},
            "target_forward_m": point_frd[0],
            "target_right_m": point_frd[1],
            "target_down_m": point_frd[2],
            "target_world_east_m": world[0],
            "target_world_north_m": world[1],
            "target_world_up_m": world[2],
        },
        "camera_contract": contract.payload(),
        "repetition_index": repetition_index,
        "seed": seed,
    }
    return payload, preflight


def build_sweep_plan(
    *,
    sweep_id: str,
    commit_sha: str,
    branch: str,
    repository_name: str,
    attitude_mode: str,
    axis_ranges: Sequence[tuple[str, Sequence[Decimal]]],
    range_values: Sequence[Decimal],
    bearing_values: Sequence[Decimal],
    elevation_values: Sequence[Decimal],
    repetitions: int,
    seed: int,
    case_order: str,
    invalid_spatial_policy: str,
    max_cases: int,
    contract: CameraContract,
    spatial_range_tolerance_m: float = 0.0001,
    spatial_angle_tolerance_deg: float = 0.00001,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    if not str(sweep_id).strip() or not str(repository_name).strip():
        raise ValueError("sweep_id and repository_name must be non-empty")
    if repetitions < 1:
        raise ValueError("repetitions must be at least one")
    if case_order not in {"canonical", "seeded"}:
        raise ValueError("case order must be canonical or seeded")
    if invalid_spatial_policy not in {"fail", "skip"}:
        raise ValueError("invalid spatial policy must be fail or skip")
    if max_cases <= 0:
        raise ValueError("max_cases must be positive")
    contract.validate()
    validate_spatial_tolerances(
        spatial_range_tolerance_m, spatial_angle_tolerance_deg
    )
    for value in range_values:
        if value <= 0:
            raise ValueError("range values must be positive")
        if value >= Decimal(str(contract.camera_clip_far_m)):
            raise ValueError("range values must be less than camera_clip_far_m")
    attitudes = build_attitude_cases(axis_ranges, mode=attitude_mode)
    spatial_count = len(range_values) * len(bearing_values) * len(elevation_values)
    total_requested = len(attitudes) * spatial_count * repetitions
    if total_requested > max_cases:
        raise ValueError(f"planned case count {total_requested} exceeds max_cases {max_cases}")
    raw_cases = []
    invalid_cases = []
    for attitude in sorted(attitudes, key=_attitude_sort_key):
        for range_value in sorted(range_values):
            for bearing_value in sorted(bearing_values):
                for elevation_value in sorted(elevation_values):
                    for repetition_index in range(repetitions):
                        payload, preflight = _case_payload(
                            attitude=attitude,
                            range_value=range_value,
                            bearing_value=bearing_value,
                            elevation_value=elevation_value,
                            repetition_index=repetition_index,
                            seed=seed,
                            contract=contract,
                            spatial_range_tolerance_m=spatial_range_tolerance_m,
                            spatial_angle_tolerance_deg=spatial_angle_tolerance_deg,
                        )
                        digest = sha256_bytes(canonical_json(payload).encode("utf-8"))[:16]
                        item = {
                            "case_hash": digest,
                            "payload": payload,
                            "fov_preflight": preflight,
                        }
                        if preflight["valid"]:
                            raw_cases.append(item)
                        else:
                            invalid_cases.append(item)
    if invalid_cases and invalid_spatial_policy == "fail":
        reasons = sorted({reason for item in invalid_cases for reason in item["fov_preflight"]["reasons"]})
        raise ValueError("requested plan contains invalid spatial cases: " + ", ".join(reasons))
    canonical_cases = raw_cases
    for ordinal, item in enumerate(canonical_cases, start=1):
        item["canonical_ordinal"] = ordinal
        item["case_id"] = f"case-{ordinal:06d}-{item['case_hash']}"
    execution = list(canonical_cases)
    if case_order == "seeded":
        random.Random(seed).shuffle(execution)
    for index, item in enumerate(execution, start=1):
        item["execution_ordinal"] = index
    plan_core = {
        "schema_version": S2_SWEEP_SCHEMA_VERSION,
        "sweep_id": sweep_id,
        "repository_name": repository_name,
        "branch": branch,
        "commit_sha": commit_sha,
        "attitude_mode": attitude_mode,
        "axis_ranges": [
            {"axis": axis, "values": [decimal_text(value) for value in values]}
            for axis, values in axis_ranges
        ],
        "range_values_m": [decimal_text(value) for value in range_values],
        "bearing_values_deg": [decimal_text(value) for value in bearing_values],
        "elevation_values_deg": [decimal_text(value) for value in elevation_values],
        "repetitions": repetitions,
        "seed": seed,
        "case_order": case_order,
        "invalid_spatial_policy": invalid_spatial_policy,
        "max_cases": max_cases,
        "camera_contract": contract.payload(),
        "observation_mode": "ideal_reference_point",
        "trajectory_mode": "static_gazebo_reference_point",
        "target_physics_mode": "static_model",
        "target_static_for_research": True,
        "spatial_range_tolerance_m": spatial_range_tolerance_m,
        "spatial_angle_tolerance_deg": spatial_angle_tolerance_deg,
        "attitude_case_count": len(attitudes),
        "spatial_cell_count": spatial_count,
        "total_planned": total_requested,
        "total_skipped_by_fov": len(invalid_cases),
        "total_runnable": len(execution),
        "skipped_cases": [
            {
                "case_hash": item["case_hash"],
                "reasons": item["fov_preflight"]["reasons"],
                "payload": item["payload"],
            }
            for item in invalid_cases
        ],
        "case_ids_in_execution_order": [item["case_id"] for item in execution],
    }
    plan_hash = sha256_bytes(canonical_json(plan_core).encode("utf-8"))
    plan = {**plan_core, "plan_hash": plan_hash}
    return plan, canonical_cases


def write_plan(output_dir: Path, plan: Mapping[str, Any], cases: Sequence[Mapping[str, Any]]) -> None:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    atomic_write_json(output / "sweep_plan.json", plan)
    temporary = output / "sweep_cases.jsonl.tmp"
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        for item in sorted(cases, key=lambda value: value["canonical_ordinal"]):
            handle.write(canonical_json(item) + "\n")
    temporary.replace(output / "sweep_cases.jsonl")


def percentile(values: Sequence[float], percentage: float) -> float:
    if not values:
        raise ValueError("percentile requires values")
    if not 0.0 <= percentage <= 100.0:
        raise ValueError("percentile must be within 0..100")
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * percentage / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _median(values: Sequence[float]) -> float:
    return percentile(values, 50.0)


def _finite_column(records: Sequence[Mapping[str, Any]], name: str, *, absolute: bool = False) -> list[float]:
    values = []
    source_name = (
        "nominal_pose_" + name.removeprefix("raw_")
        if name.startswith("raw_")
        else name
    )
    for record in records:
        value = record.get(source_name)
        if value is None and source_name != name:
            # Read-only compatibility for sealed artifacts produced before the
            # research schema renamed raw_* to nominal_pose_*.
            value = record.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise ValueError(f"analysis field {source_name} must be finite")
        values.append(abs(float(value)) if absolute else float(value))
    return values


def analysis_window(
    records: Sequence[Mapping[str, Any]], *, warmup_count: int, analysis_count: int
) -> tuple[list[Mapping[str, Any]], int]:
    if warmup_count < 0 or analysis_count <= 0:
        raise ValueError("sample window counts are invalid")
    eligible = sorted(
        (record for record in records if record.get("sample_eligible") is True),
        key=lambda record: int(record["record_sequence"]),
    )
    if len(eligible) < warmup_count + analysis_count:
        raise ValueError("insufficient eligible samples")
    return eligible[warmup_count : warmup_count + analysis_count], len(eligible)


def compute_case_statistics(
    records: Sequence[Mapping[str, Any]], *, warmup_count: int, analysis_count: int
) -> dict[str, Any]:
    window, eligible_count = analysis_window(
        records, warmup_count=warmup_count, analysis_count=analysis_count
    )
    result: dict[str, Any] = {
        "analysis_sample_count": len(window),
        "total_eligible_count": eligible_count,
        "warmup_excluded_count": warmup_count,
        "invalid_record_count": len(records) - eligible_count,
    }
    for prefix in ("raw", "corrected"):
        position = _finite_column(window, f"{prefix}_position_error_m")
        result.update(
            {
                f"{prefix}_position_error_median": _median(position),
                f"{prefix}_position_error_p95": percentile(position, 95.0),
                f"{prefix}_position_error_p99": percentile(position, 99.0),
                f"{prefix}_position_error_max": max(position),
                f"{prefix}_forward_abs_error_median": _median(_finite_column(window, f"{prefix}_forward_error_m", absolute=True)),
                f"{prefix}_right_abs_error_median": _median(_finite_column(window, f"{prefix}_right_error_m", absolute=True)),
                f"{prefix}_down_abs_error_median": _median(_finite_column(window, f"{prefix}_down_error_m", absolute=True)),
                f"{prefix}_range_abs_error_median": _median(_finite_column(window, f"{prefix}_range_error_m", absolute=True)),
                f"{prefix}_ray_gap_median": _median(_finite_column(window, f"{prefix}_ray_gap_m")),
                f"{prefix}_ray_gap_p99": percentile(_finite_column(window, f"{prefix}_ray_gap_m"), 99.0),
                f"{prefix}_intersection_angle_median_deg": _median(_finite_column(window, f"{prefix}_intersection_angle_deg")),
                f"{prefix}_intersection_angle_min_deg": min(_finite_column(window, f"{prefix}_intersection_angle_deg")),
            }
        )
    raw = result["raw_position_error_median"]
    corrected = result["corrected_position_error_median"]
    result["corrected_raw_median_ratio"] = corrected / raw if raw != 0.0 else 0.0
    result["corrected_improvement_percent"] = (
        (raw - corrected) * 100.0 / raw if raw != 0.0 else 0.0
    )
    return result


def compute_spatial_validation(
    records: Sequence[Mapping[str, Any]],
    *,
    warmup_count: int,
    analysis_count: int,
    requested_range_m: float,
    requested_bearing_deg: float,
    requested_elevation_deg: float,
    range_tolerance_m: float,
    angle_tolerance_deg: float,
) -> dict[str, Any]:
    validate_spatial_tolerances(range_tolerance_m, angle_tolerance_deg)
    requested = (
        float(requested_range_m),
        float(requested_bearing_deg),
        float(requested_elevation_deg),
    )
    if not all(math.isfinite(value) for value in requested) or requested[0] <= 0.0:
        raise ValueError("requested spatial cell must be finite with positive range")
    window, _eligible_count = analysis_window(
        records, warmup_count=warmup_count, analysis_count=analysis_count
    )
    actual_ranges: list[float] = []
    actual_bearings: list[float] = []
    actual_elevations: list[float] = []
    range_errors: list[float] = []
    bearing_errors: list[float] = []
    elevation_errors: list[float] = []
    for record in window:
        point = []
        for field in ("gt_forward_m", "gt_right_m", "gt_down_m"):
            value = record.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"spatial validation field {field} must be finite")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"spatial validation field {field} must be finite")
            point.append(value)
        actual_range, actual_bearing, actual_elevation = spatial_from_frd(point)
        actual_ranges.append(actual_range)
        actual_bearings.append(actual_bearing)
        actual_elevations.append(actual_elevation)
        range_errors.append(abs(actual_range - requested[0]))
        bearing_errors.append(abs(wrapped_angle_error_deg(actual_bearing, requested[1])))
        elevation_errors.append(abs(actual_elevation - requested[2]))
    range_max = max(range_errors)
    bearing_max = max(bearing_errors)
    elevation_max = max(elevation_errors)
    return {
        "requested_range_m": requested[0],
        "requested_bearing_deg": requested[1],
        "requested_elevation_deg": requested[2],
        "actual_range_m_median": _median(actual_ranges),
        "actual_bearing_deg_median": _median(actual_bearings),
        "actual_elevation_deg_median": _median(actual_elevations),
        "actual_range_m_min": min(actual_ranges),
        "actual_range_m_max": max(actual_ranges),
        "actual_bearing_deg_min": min(actual_bearings),
        "actual_bearing_deg_max": max(actual_bearings),
        "actual_elevation_deg_min": min(actual_elevations),
        "actual_elevation_deg_max": max(actual_elevations),
        "spatial_range_error_abs_max_m": range_max,
        "spatial_bearing_error_abs_max_deg": bearing_max,
        "spatial_elevation_error_abs_max_deg": elevation_max,
        "spatial_range_tolerance_m": range_tolerance_m,
        "spatial_angle_tolerance_deg": angle_tolerance_deg,
        "spatial_validation_pass": (
            len(window) == analysis_count
            and range_max <= range_tolerance_m
            and bearing_max <= angle_tolerance_deg
            and elevation_max <= angle_tolerance_deg
        ),
        "spatial_validation_sample_count": len(window),
    }


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    if not isinstance(value, dict):
        raise ValueError(f"{path.name} must contain an object")
    return value


def validate_r2_artifacts(
    recording_dir: Path,
    *,
    expected_sha: str,
    expected_branch: str,
    expected_run_id: str,
    expected_case_id: str,
    minimum_eligible: int,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    directory = Path(recording_dir)
    required = (
        "run_metadata.json",
        "pair_records.jsonl",
        "pair_records.csv",
        "run_summary.json",
        "RUN_FINALIZED_VALID",
    )
    missing = [name for name in required if not (directory / name).is_file()]
    if missing:
        raise ValueError("R2 artifact is incomplete: " + ", ".join(missing))
    if (directory / "RUN_INCOMPLETE").exists():
        raise ValueError("R2 RUN_INCOMPLETE marker remains")
    if (directory / "RUN_FINALIZED_INVALID").exists():
        raise ValueError("R2 VALID and INVALID markers conflict")
    metadata = _load_json(directory / "run_metadata.json")
    summary = _load_json(directory / "run_summary.json")
    expected_metadata = {
        "schema_version": RESEARCH_SCHEMA_VERSION,
        "commit_sha": expected_sha,
        "branch": expected_branch,
        "run_id": expected_run_id,
        "case_id": expected_case_id,
        "output_frame": "baseline_center_frd",
    }
    for key, value in expected_metadata.items():
        if metadata.get(key) != value:
            raise ValueError(f"R2 metadata mismatch: {key}")
    oracle_hash = metadata.get("camera_oracle_expected_sdf_sha256")
    if (
        metadata.get("camera_oracle_source") != "gazebo_observed_optical"
        or not isinstance(oracle_hash, str)
        or len(oracle_hash) != 64
        or any(character not in "0123456789abcdef" for character in oracle_hash)
    ):
        raise ValueError("R2 camera oracle metadata provenance is invalid")
    desired_poses = metadata.get("desired_camera_poses_from_env")
    if not isinstance(desired_poses, dict) or set(desired_poses) != {"left", "right"}:
        raise ValueError("R2 desired camera metadata is incomplete")
    if summary.get("run_valid") is not True:
        raise ValueError("R2 run is invalid")
    for key, value in (
        ("schema_version", RESEARCH_SCHEMA_VERSION),
        ("run_id", expected_run_id),
        ("case_id", expected_case_id),
    ):
        if summary.get(key) != value:
            raise ValueError(f"R2 summary mismatch: {key}")
    if summary.get("shutdown_drain_status") != "SUCCESS":
        raise ValueError("R2 shutdown did not drain successfully")
    lines = (directory / "pair_records.jsonl").read_text(encoding="utf-8").splitlines()
    records = [
        json.loads(line, parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
        for line in lines
    ]
    keys: set[tuple[int, int]] = set()
    for sequence, record in enumerate(records, start=1):
        if record.get("schema_version") != RESEARCH_SCHEMA_VERSION:
            raise ValueError("R2 pair schema mismatch")
        if type(record.get("sample_eligible")) is not bool:
            raise ValueError("sample_eligible must be a JSON bool")
        if record.get("terminal_status") not in TERMINAL_STATUSES:
            raise ValueError("unknown terminal_status")
        if record.get("measurement_status") not in MEASUREMENT_STATUSES:
            raise ValueError("unknown measurement_status")
        if record.get("evaluation_status") not in EVALUATION_STATUSES:
            raise ValueError("unknown evaluation_status")
        if record.get("record_sequence") != sequence:
            raise ValueError("record_sequence is not contiguous")
        for field in ("epoch", "pair_id", "left_stamp_ns", "right_stamp_ns", "pair_midpoint_stamp_ns"):
            if isinstance(record.get(field), bool) or not isinstance(record.get(field), int):
                raise ValueError(f"{field} must be an integer")
        if record.get("output_frame") != "baseline_center_frd":
            raise ValueError("pair output frame mismatch")
        if record.get("output_axes") != "forward_right_down":
            raise ValueError("pair output axes mismatch")
        if record.get("position_unit") != "m" or record.get("timestamp_unit") != "ns":
            raise ValueError("pair unit contract mismatch")
        key = (record["epoch"], record["pair_id"])
        if key in keys:
            raise ValueError("duplicate R2 pair key")
        keys.add(key)
        if record["sample_eligible"]:
            if (
                record.get("camera_oracle_source") != "gazebo_observed_optical"
                or record.get("correction_source") != "gazebo_observed_optical_pose"
                or record.get("camera_oracle_expected_sdf_sha256") != oracle_hash
            ):
                raise ValueError("eligible R2 record camera oracle provenance mismatch")
            for label in ("left", "right"):
                prefix = f"{label}_camera_oracle_"
                if (
                    not isinstance(record.get(prefix + "gazebo_stamp_ns"), int)
                    or isinstance(record.get(prefix + "gazebo_stamp_ns"), bool)
                    or record[prefix + "gazebo_stamp_ns"] <= 0
                    or record.get(prefix + "base_entity_name") != f"{label}_stereo_camera::base_link"
                    or record.get(prefix + "optical_entity_name") != f"{label}_stereo_camera::optical_link"
                ):
                    raise ValueError("eligible R2 record camera oracle entity/stamp mismatch")
                for field in ("base_entity_id", "optical_entity_id"):
                    if isinstance(record.get(prefix + field), bool) or not isinstance(record.get(prefix + field), int):
                        raise ValueError("eligible R2 record camera oracle entity ID mismatch")
                vector = tuple(
                    record.get(prefix + field)
                    for field in ("optical_x_m", "optical_y_m", "optical_z_m")
                )
                quaternion = tuple(
                    record.get(prefix + field)
                    for field in ("optical_qx", "optical_qy", "optical_qz", "optical_qw")
                )
                if not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in (*vector, *quaternion)):
                    raise ValueError("eligible R2 record camera oracle pose is invalid")
                if sum(float(value) * float(value) for value in quaternion) <= 0.0:
                    raise ValueError("eligible R2 record camera oracle quaternion is invalid")
    with (directory / "pair_records.csv").open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != PAIR_RECORD_COLUMNS:
            raise ValueError("R2 CSV schema mismatch")
        csv_rows = list(reader)
    if len(csv_rows) != len(records):
        raise ValueError("R2 JSONL and CSV row counts differ")
    if summary.get("pair_records_jsonl_sha256") != sha256_file(directory / "pair_records.jsonl"):
        raise ValueError("R2 JSONL hash mismatch")
    if summary.get("pair_records_csv_sha256") != sha256_file(directory / "pair_records.csv"):
        raise ValueError("R2 CSV hash mismatch")
    if summary.get("run_metadata_sha256") != sha256_file(directory / "run_metadata.json"):
        raise ValueError("R2 metadata hash mismatch")
    accepted = int(summary.get("accepted", -1))
    processed = int(summary.get("processed", -1))
    discarded = int(summary.get("discarded", -1))
    if not (accepted == processed + discarded == len(records)):
        raise ValueError("R2 terminal accounting mismatch")
    for field in (
        "terminal_record_count",
        "jsonl_line_count",
        "csv_data_row_count",
        "unique_pair_key_count",
    ):
        if int(summary.get(field, -1)) != len(records):
            raise ValueError(f"R2 summary count mismatch: {field}")
    eligible_count = sum(record["sample_eligible"] for record in records)
    if eligible_count < minimum_eligible:
        raise ValueError("insufficient eligible samples")
    return records, summary


def build_launch_command(
    *,
    repository_root: Path,
    case: Mapping[str, Any],
    attempt_dir: Path,
    sweep_id: str,
    repository_name: str,
    branch: str,
    commit_sha: str,
    gazebo_version: str,
    measurement_queue_capacity: int = 32,
    research_queue_capacity: int = 1024,
) -> list[str]:
    del repository_root
    payload = case["payload"]
    camera = payload["camera_contract"]
    spatial = payload["spatial"]
    command = [
        "ros2",
        "launch",
        "stereo_ground_sim",
        "stereo_ground_sim.launch.py",
        "observation_mode:=ideal_reference_point",
        "enable_research_recording:=true",
        "target_static_for_research:=true",
        f"research_output_dir:={Path(attempt_dir).resolve() / 'recording'}",
        f"research_run_id:={sweep_id}",
        f"research_case_id:={case['case_id']}",
        f"research_repository:={repository_name}",
        f"research_branch:={branch}",
        f"research_commit_sha:={commit_sha}",
        "research_git_status:=clean",
        f"research_gazebo_version:={gazebo_version}",
        "research_px4_version:=not_applicable",
        f"research_record_queue_capacity:={research_queue_capacity}",
        f"baseline_m:={camera['baseline_m']}",
        f"camera_height_m:={camera['camera_height_m']}",
        f"horizontal_fov_rad:={camera['horizontal_fov_rad']}",
        f"camera_image_width_px:={camera['image_width_px']}",
        f"camera_image_height_px:={camera['image_height_px']}",
        f"camera_update_rate_hz:={camera['camera_update_rate_hz']}",
        f"camera_clip_near_m:={camera['camera_clip_near_m']}",
        f"camera_clip_far_m:={camera['camera_clip_far_m']}",
        f"measurement_queue_capacity:={measurement_queue_capacity}",
        f"yolo_target_hz:={camera['camera_update_rate_hz']}",
        f"target_spawn_x_m:={spatial['target_world_east_m']}",
        f"target_spawn_y_m:={spatial['target_world_north_m']}",
        f"target_spawn_z_m:={spatial['target_world_up_m']}",
        f"approach_start_distance_m:={spatial['range_m']['value']}",
        f"approach_end_distance_m:={spatial['range_m']['value']}",
        "scenario_name:=s2-r3-static-ideal-sweep",
        "mission_profile:=manual",
        "fly_target_in_gazebo:=false",
        "gz_gui:=false",
        "launch_rviz:=false",
        "record_calibration_csv:=false",
        "enable_synthetic_detections:=false",
        "enable_external_bbox_detections:=false",
        "overlay_publish_hz:=0.0",
    ]
    for axis in ATTITUDE_AXES:
        command.append(f"{axis}:={payload['attitude_source_deg'][axis]['value']}")
    return command


def next_attempt_directory(case_dir: Path) -> tuple[int, Path]:
    case_dir = Path(case_dir)
    numbers = []
    if case_dir.exists():
        for child in case_dir.iterdir():
            if child.is_dir() and child.name.startswith("attempt-"):
                try:
                    numbers.append(int(child.name.split("-", 1)[1]))
                except ValueError:
                    continue
    number = max(numbers, default=0) + 1
    return number, case_dir / f"attempt-{number:04d}"


def write_case_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    temporary = Path(path).with_name(Path(path).name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CASE_CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, "") for name in CASE_CSV_COLUMNS})
    temporary.replace(path)


__all__ = [
    "ATTITUDE_AXES",
    "CASE_CSV_COLUMNS",
    "CameraContract",
    "EVALUATION_STATUSES",
    "MEASUREMENT_STATUSES",
    "S2_SWEEP_SCHEMA_VERSION",
    "TERMINAL_STATUSES",
    "atomic_write_json",
    "analysis_window",
    "build_attitude_cases",
    "build_launch_command",
    "build_sweep_plan",
    "canonical_json",
    "compute_case_statistics",
    "compute_spatial_validation",
    "decimal_range",
    "decimal_text",
    "effective_rpy",
    "fov_preflight",
    "next_attempt_directory",
    "parse_attitude_axis_range",
    "parse_numeric_values",
    "percentile",
    "sha256_bytes",
    "sha256_file",
    "spatial_frd",
    "spatial_from_frd",
    "validate_spatial_tolerances",
    "wrapped_angle_error_deg",
    "validate_r2_artifacts",
    "write_case_csv",
    "write_plan",
]
