"""Derive workbook-facing geometry and source evidence from one method outcome.

Everything here is recomputed from the frozen attempt payload with the same
functions the live solver used, so the canonical R,T, the actual ray inputs
and the public Forward/Right/Down/Range are exactly the values behind the
recorded result.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from types import SimpleNamespace
from typing import Any

from Workstation.workstation_core.canonical_transform import (
    flatten_rotation,
    flatten_translation,
)
from Workstation.workstation_core.triangulation import (
    normalized_ray_from_pixel,
    pixhawk_canonical_transform,
)

NOT_AVAILABLE = "N/A"

_UNIT_SUFFIXES: tuple[tuple[str, str], ...] = (
    ("_deg", "deg"),
    ("_rad_s", "rad/s"),
    ("_m_s", "m/s"),
    ("_px", "px"),
    ("_us", "us"),
    ("_ns", "ns"),
    ("_ms", "ms"),
    ("_s", "s"),
    ("_m", "m"),
)


def _unit_for(field: str) -> str:
    for suffix, unit in _UNIT_SUFFIXES:
        if field.endswith(suffix):
            return unit
    return ""


def _finite(value: object) -> float | None:
    try:
        parsed = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if math.isfinite(parsed) else None


def _vector(value: object) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    parsed = tuple(_finite(item) for item in value)
    if any(item is None for item in parsed):
        return None
    return parsed  # type: ignore[return-value]


def _quaternion(value: object) -> tuple[float, float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    parsed = tuple(_finite(item) for item in value)
    if any(item is None for item in parsed):
        return None
    return parsed  # type: ignore[return-value]


def _matrix(value: object) -> tuple[tuple[float, float, float], ...] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    rows = tuple(_vector(row) for row in value)
    if any(row is None for row in rows):
        return None
    return rows  # type: ignore[return-value]


def _intrinsics_adapter(payload: object) -> SimpleNamespace | None:
    if not isinstance(payload, Mapping):
        return None
    matrix = _matrix(payload.get("camera_matrix"))
    distortion = payload.get("distortion_coefficients")
    if matrix is None or not isinstance(distortion, (list, tuple)):
        return None
    return SimpleNamespace(
        camera_matrix=[list(row) for row in matrix],
        distortion_coefficients=tuple(float(value) for value in distortion),
        width=int(payload.get("width_px") or 0),
        height=int(payload.get("height_px") or 0),
    )


def canonical_transform_for_attempt(
    attempt: Mapping[str, Any],
) -> tuple[tuple[tuple[float, float, float], ...], tuple[float, float, float]] | None:
    """Return canonical (R_right_from_left, T_right_from_left_m) or None."""

    source = str(attempt.get("triangulation_source") or "")
    intrinsics = attempt.get("intrinsics")
    if source == "checkerboard":
        extrinsics = intrinsics.get("checkerboard_extrinsics") if isinstance(intrinsics, Mapping) else None
        if not isinstance(extrinsics, Mapping):
            return None
        rotation = _matrix(extrinsics.get("rotation_left_to_right"))
        translation = _vector(extrinsics.get("translation_left_to_right_m"))
        if rotation is None or translation is None:
            return None
        return rotation, translation
    left_attitude = attempt.get("left_attitude")
    right_attitude = attempt.get("right_attitude")
    left_q = _quaternion(attempt.get("geometry_left_quaternion_wxyz")) or (
        _quaternion(left_attitude.get("quaternion_wxyz")) if isinstance(left_attitude, Mapping) else None
    )
    right_q = _quaternion(attempt.get("geometry_right_quaternion_wxyz")) or (
        _quaternion(right_attitude.get("quaternion_wxyz")) if isinstance(right_attitude, Mapping) else None
    )
    baseline = attempt.get("baseline")
    origin = (
        _vector(baseline.get("translation_right_in_left_rdf_m"))
        if isinstance(baseline, Mapping)
        else None
    )
    if left_q is None or right_q is None or origin is None:
        return None
    try:
        rotation, translation = pixhawk_canonical_transform(
            left_body_to_world_q=left_q,
            right_body_to_world_q=right_q,
            translation_right_in_left_m=origin,
        )
    except (TypeError, ValueError, OverflowError):
        return None
    return tuple(tuple(row) for row in rotation), translation


def derive_method_result(outcome_record: Mapping[str, Any]) -> dict[str, Any]:
    """Flat workbook values for one outcome: R,T, u,v, rays, F/R/D/Range, diagnostics."""

    attempt = outcome_record.get("attempt_payload")
    attempt = attempt if isinstance(attempt, Mapping) else {}
    result = outcome_record.get("result")
    result = result if isinstance(result, Mapping) else {}
    derived: dict[str, Any] = {
        "status": str(outcome_record.get("status") or ""),
        "reason": "; ".join(str(item) for item in outcome_record.get("reasons", ()) or ()),
        "warnings": tuple(str(item) for item in outcome_record.get("warnings", ()) or ()),
        "method_instance_id": str(outcome_record.get("method_instance_id") or ""),
        "worker_started_monotonic_ns": outcome_record.get("worker_started_monotonic_ns"),
        "completed_monotonic_ns": outcome_record.get("completed_monotonic_ns"),
        "rotation": None,
        "translation": None,
        "left_uv": None,
        "right_uv": None,
        "left_ray": None,
        "right_ray": None,
        "forward_m": None,
        "right_m": None,
        "down_m": None,
        "range_m": None,
        "ray_gap_m": None,
        "ray_angle_deg": None,
        "left_depth_m": None,
        "right_depth_m": None,
        "triangulation_reason": str(result.get("triangulation_reason") or ""),
        "time_delta_s": None,
        "left_quaternion_wxyz": None,
        "right_quaternion_wxyz": None,
        "left_camera": None,
        "right_camera": None,
    }
    transform = canonical_transform_for_attempt(attempt)
    if transform is not None:
        derived["rotation"], derived["translation"] = transform
    intrinsics = attempt.get("intrinsics")
    for side in ("left", "right"):
        detection = attempt.get(f"{side}_detection")
        center = _vector_2(detection.get("center_px")) if isinstance(detection, Mapping) else None
        derived[f"{side}_uv"] = center
        camera = _intrinsics_adapter(intrinsics.get(side)) if isinstance(intrinsics, Mapping) else None
        if camera is not None:
            derived[f"{side}_camera"] = {
                "camera_matrix": tuple(tuple(row) for row in camera.camera_matrix),
                "distortion_coefficients": tuple(camera.distortion_coefficients),
                "width_px": camera.width,
                "height_px": camera.height,
            }
        if center is not None and camera is not None:
            try:
                derived[f"{side}_ray"] = normalized_ray_from_pixel(center[0], center[1], camera)
            except (TypeError, ValueError, OverflowError, ZeroDivisionError):
                derived[f"{side}_ray"] = None
        attitude = attempt.get(f"{side}_attitude")
        derived[f"{side}_quaternion_wxyz"] = _quaternion(
            attempt.get(f"geometry_{side}_quaternion_wxyz")
        ) or (_quaternion(attitude.get("quaternion_wxyz")) if isinstance(attitude, Mapping) else None)
    match = result.get("match")
    if isinstance(match, Mapping):
        derived["time_delta_s"] = _finite(match.get("time_delta_s"))
    triangulated = result.get("triangulated")
    if isinstance(triangulated, Mapping):
        point = _vector(triangulated.get("point_m"))
        if point is not None:
            derived["forward_m"], derived["right_m"], derived["down_m"] = point
        derived["range_m"] = _finite(triangulated.get("range_m"))
        derived["ray_gap_m"] = _finite(triangulated.get("closest_gap_m"))
        derived["ray_angle_deg"] = _finite(triangulated.get("ray_angle_deg"))
        derived["left_depth_m"] = _finite(triangulated.get("left_depth_m"))
        derived["right_depth_m"] = _finite(triangulated.get("right_depth_m"))
    return derived


def _vector_2(value: object) -> tuple[float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    parsed = tuple(_finite(item) for item in value)
    if any(item is None for item in parsed):
        return None
    return parsed  # type: ignore[return-value]


def flat_transform_cells(derived: Mapping[str, Any]) -> dict[str, float | None]:
    cells: dict[str, float | None] = {
        **{f"R{row}{column}": None for row in (1, 2, 3) for column in (1, 2, 3)},
        "T_R": None,
        "T_D": None,
        "T_F": None,
    }
    rotation = derived.get("rotation")
    translation = derived.get("translation")
    if rotation is not None:
        cells.update(flatten_rotation(rotation))
    if translation is not None:
        cells.update(flatten_translation(translation))
    return cells


def _scalar(value: object) -> object:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return ", ".join(str(_scalar(item)) for item in value)
    if isinstance(value, Mapping):
        return "; ".join(f"{key}={_scalar(item)}" for key, item in value.items())
    return str(value)


def _flatten_rows(
    prefix: str,
    payload: Mapping[str, Any],
    *,
    source: str,
    skip: frozenset[str] = frozenset(),
) -> list[tuple[str, object, str, str]]:
    rows: list[tuple[str, object, str, str]] = []
    for key, value in payload.items():
        if key in skip:
            continue
        field = f"{prefix}{key}"
        if isinstance(value, Mapping):
            rows.extend(_flatten_rows(f"{field}.", value, source=source))
            continue
        if isinstance(value, (list, tuple)) and value and all(
            isinstance(item, (int, float)) and not isinstance(item, bool) for item in value
        ):
            names = _component_names(key, len(value))
            for name, item in zip(names, value):
                rows.append((f"{field}.{name}", item, source, _unit_for(key)))
            continue
        rows.append((field, _scalar(value), source, _unit_for(key)))
    return rows


def _component_names(key: str, length: int) -> tuple[str, ...]:
    if "quaternion" in key and length == 4:
        return ("w", "x", "y", "z")
    if "ned" in key and length == 3:
        return ("N", "E", "D")
    if ("rdf" in key or "translation" in key) and length == 3:
        return ("R", "D", "F")
    if "center_px" in key and length == 2:
        return ("u", "v")
    if "bbox" in key and length == 4:
        return ("x1", "y1", "x2", "y2")
    if length == 3:
        return ("0", "1", "2")
    return tuple(str(index) for index in range(length))


_ATTEMPT_TOP_LEVEL_SKIP = frozenset(
    {
        "left_detection",
        "right_detection",
        "left_attitude",
        "right_attitude",
        "left_gps",
        "right_gps",
        "intrinsics",
        "baseline",
        "geometry_config",
        "detection_pair_key",
        "measurement_component_sources",
        "warnings",
        "hard_block_reasons",
        "advisory_reasons",
    }
)


def source_evidence_rows(
    outcome_record: Mapping[str, Any],
    method_evidence: Mapping[str, Any] | None = None,
) -> list[tuple[str, object, str, str]]:
    """Long-format (field, value, source, unit) rows behind this method's R,t."""

    attempt = outcome_record.get("attempt_payload")
    attempt = attempt if isinstance(attempt, Mapping) else {}
    rows: list[tuple[str, object, str, str]] = []
    component_sources: dict[str, str] = {}
    for item in attempt.get("measurement_component_sources", ()) or ():
        if isinstance(item, (list, tuple)) and len(item) == 2:
            component_sources[str(item[0])] = str(item[1])
    if method_evidence:
        for name, source in dict(method_evidence.get("component_sources", {}) or {}).items():
            component_sources.setdefault(str(name), str(source))
    for name, source in component_sources.items():
        rows.append((f"component_source.{name}", source, "method_status", ""))
    for side in ("left", "right"):
        attitude = attempt.get(f"{side}_attitude")
        if isinstance(attitude, Mapping):
            source = f"pixhawk:{attitude.get('namespace', '')}/{attitude.get('topic', '')}"
            rows.extend(_flatten_rows(f"{side}_attitude.", attitude, source=source))
            age = attempt.get(f"{side}_attitude_age_s")
            rows.append((f"{side}_attitude.age_s", age, "pairing", "s"))
        gps = attempt.get(f"{side}_gps")
        if isinstance(gps, Mapping):
            source = f"gnss:{gps.get('namespace', '')}/{gps.get('topic', '')}"
            rows.extend(_flatten_rows(f"{side}_gps.", gps, source=source))
    for side in ("left", "right"):
        geometry_q = attempt.get(f"geometry_{side}_quaternion_wxyz")
        if geometry_q is not None:
            rows.extend(
                _flatten_rows(
                    "",
                    {f"geometry_{side}_quaternion_wxyz": geometry_q},
                    source=str(attempt.get("measurement_yaw_source") or "geometry"),
                )
            )
    baseline = attempt.get("baseline")
    if isinstance(baseline, Mapping):
        rows.extend(
            _flatten_rows("baseline.", baseline, source=str(baseline.get("source") or "baseline"))
        )
    intrinsics = attempt.get("intrinsics")
    if isinstance(intrinsics, Mapping):
        rows.extend(
            _flatten_rows(
                "intrinsics.",
                intrinsics,
                source="calibration",
                skip=frozenset({"left", "right", "checkerboard_extrinsics"}),
            )
        )
        extrinsics = intrinsics.get("checkerboard_extrinsics")
        if isinstance(extrinsics, Mapping):
            rows.extend(
                _flatten_rows(
                    "applied_extrinsics.",
                    extrinsics,
                    source=str(attempt.get("measurement_method_instance_id") or "alignment"),
                )
            )
    geometry_config = attempt.get("geometry_config")
    if isinstance(geometry_config, Mapping):
        rows.extend(_flatten_rows("geometry_config.", geometry_config, source="settings"))
    rows.extend(
        _flatten_rows(
            "attempt.",
            attempt,
            source="attempt",
            skip=_ATTEMPT_TOP_LEVEL_SKIP,
        )
    )
    for warning in attempt.get("warnings", ()) or ():
        rows.append(("attempt.warning", str(warning), "attempt", ""))
    if method_evidence:
        rows.extend(
            _flatten_rows(
                "method.",
                method_evidence,
                source="method_status",
                skip=frozenset({"component_sources"}),
            )
        )
    return rows


__all__ = [
    "NOT_AVAILABLE",
    "canonical_transform_for_attempt",
    "derive_method_result",
    "flat_transform_cells",
    "source_evidence_rows",
]
