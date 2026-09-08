"""Additive, method-midpoint local-level FRD from frozen left optical RDF."""
from __future__ import annotations

import math
from collections.abc import Mapping

from Workstation.workstation_core.attitude_geometry import normalize_quaternion, rotate_vector_by_quaternion
from Workstation.workstation_core.triangulation import camera_vector_to_body_frd, invert_left_to_right_camera_transform

LEVEL_COORDINATE_FIELDS = (
    "target_level_forward_m", "target_level_right_m",
    "target_level_down_m", "target_level_range_m",
)
LEVEL_METADATA_FIELDS = ("target_level_frame", "target_level_status", "target_level_reason")
LEVEL_OUTPUT_FIELDS = LEVEL_COORDINATE_FIELDS + LEVEL_METADATA_FIELDS


def _finite_vector(value, size=3):
    if not isinstance(value, (tuple, list)) or len(value) != size:
        raise ValueError("missing_or_invalid_vector")
    if any(isinstance(v, bool) for v in value):
        raise ValueError("boolean_vector")
    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise ValueError("nonfinite_vector")
    return result


def target_level_from_attempt(attempt: Mapping, source_point_left_rdf_m) -> dict:
    """Keep the method's midpoint; use raw pair-frozen P1, never solver attitude.

    Input/translation: left optical [Right,Down,Forward] metres. Output:
    horizontal P1 forward, horizontal right, NED down, metres. No GNSS position
    is required. Translation is the same resolved baseline consumed by the
    method's triangulation; its half is the existing optical midpoint.
    """
    result = dict.fromkeys(LEVEL_COORDINATE_FIELDS)
    result.update(target_level_frame="baseline_center_local_level_frd",
                  target_level_status="UNAVAILABLE", target_level_reason="")
    evidence = {
        "schema": "target-level-coordinates-v1",
        "axis_contract": "baseline_center_local_level_frd:forward_right_down",
        "origin_model": "method_baseline_midpoint",
        "attitude_source": "pair_frozen_left_attitude",
        "mount_rotation_offset_rpy_deg": [0.0, 0.0, 0.0],
    }
    result["target_level_evidence"] = evidence
    try:
        point = _finite_vector(source_point_left_rdf_m)
        attitude = attempt.get("left_attitude")
        if not isinstance(attitude, Mapping):
            raise ValueError("left_attitude_unavailable")
        q = normalize_quaternion(_finite_vector(attitude.get("quaternion_wxyz"), 4))
        baseline = attempt.get("baseline") or {}
        if not isinstance(baseline, Mapping):
            raise ValueError("method_baseline_invalid")
        if attempt.get("triangulation_source") == "checkerboard":
            extrinsics = (attempt.get("intrinsics") or {}).get("checkerboard_extrinsics") or {}
            _, translation = invert_left_to_right_camera_transform(
                extrinsics.get("rotation_left_to_right"),
                extrinsics.get("translation_left_to_right_m"),
            )
            translation = _finite_vector(translation)
        else:
            translation = _finite_vector(baseline.get("translation_right_in_left_rdf_m"))
        if math.hypot(*translation) <= 0:
            raise ValueError("method_baseline_zero")
        origin = tuple(v / 2 for v in translation)
        centered = tuple(p - o for p, o in zip(point, origin))
        ned = rotate_vector_by_quaternion(camera_vector_to_body_frd(centered), q)
        forward_ned = rotate_vector_by_quaternion((1., 0., 0.), q)
        horizontal = math.hypot(forward_ned[0], forward_ned[1])
        if horizontal < 1e-8:
            raise ValueError("horizontal_forward_undefined")
        fn, fe = forward_ned[0] / horizontal, forward_ned[1] / horizontal
        values = (fn * ned[0] + fe * ned[1], -fe * ned[0] + fn * ned[1],
                  ned[2], math.hypot(*centered))
        if not all(math.isfinite(v) for v in values):
            raise ValueError("nonfinite_level_result")
        result.update(zip(LEVEL_COORDINATE_FIELDS, values))
        result["target_level_status"] = "AVAILABLE"
        evidence.update(
            source_point_left_rdf_m=list(point), origin_left_rdf_m=list(origin),
            body_to_ned_quaternion_wxyz=list(q), forward_axis_ned=[fn, fe, 0.],
            right_axis_ned=[-fe, fn, 0.], down_axis_ned=[0., 0., 1.],
            attitude_timestamp_s=attitude.get("app_receive_monotonic_s", attitude.get("timestamp_s")),
            attitude_age_s=attempt.get("left_attitude_age_s"),
            detection_midpoint_monotonic_s=attempt.get("detection_midpoint_monotonic_s"),
            baseline_source=baseline.get("source"),
        )
    except (TypeError, ValueError, OverflowError, AttributeError) as exc:
        result.update(dict.fromkeys(LEVEL_COORDINATE_FIELDS))
        result["target_level_status"] = "UNAVAILABLE"
        result["target_level_reason"] = str(exc)
    return result
