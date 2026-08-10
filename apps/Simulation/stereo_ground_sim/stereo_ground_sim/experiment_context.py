"""Simulation-owned experiment context construction and identity hashing.

This module is deliberately a producer-side boundary.  It uses the final
resolved launch values and the same nominal optical pose helper that builds
the generated SDF, but it has no dependency on PoseSimulation.
"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any, Mapping, Sequence

from .coordinate_frames import CameraOpticalPose, effective_optical_rpy_errors_deg


CONTEXT_SCHEMA = "simulation_experiment_context/v1"
SOURCE_APPLICATION = "Simulation"
WORLD_FRAME = "gazebo_world_enu"
CAMERA_ERROR_FRAME = "camera_optical_rdf"
CAMERA_ERROR_AXES = "right_down_forward"
ROTATION_COMPOSITION = "R_down_yaw@R_right_pitch@R_forward_roll"
QUATERNION_ORDER = "xyzw"


def _finite(value: Any, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _vector(values: Sequence[Any], name: str) -> list[float]:
    if len(values) != 3:
        raise ValueError(f"{name} must contain three values")
    return [_finite(value, name) for value in values]


def _quaternion(values: Sequence[Any], name: str) -> list[float]:
    if len(values) != 4:
        raise ValueError(f"{name} must contain four values")
    parsed = [_finite(value, name) for value in values]
    norm = math.sqrt(sum(value * value for value in parsed))
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError(f"{name} must have a non-zero finite norm")
    normalized = [value / norm for value in parsed]
    # Quaternion sign is not part of the represented rotation.  Canonicalize
    # it before hashing so q and -q cannot create two context identities.
    if normalized[3] < 0.0 or (
        abs(normalized[3]) <= 1.0e-15
        and next((value for value in normalized[:3] if abs(value) > 1.0e-15), 0.0) < 0.0
    ):
        normalized = [-value for value in normalized]
    return normalized


def _pose_payload(
    position_world_enu_m: Sequence[Any],
    orientation_xyzw: Sequence[Any],
) -> dict[str, list[float]]:
    return {
        "position_world_enu_m": _vector(position_world_enu_m, "pose.position_world_enu_m"),
        "orientation_xyzw": _quaternion(orientation_xyzw, "pose.orientation_xyzw"),
    }


def camera_pose_quaternion_xyzw(camera_pose: CameraOpticalPose) -> tuple[float, float, float, float]:
    """Convert a CameraOpticalPose's world RDF basis to ROS xyzw."""

    right = camera_pose.right_world_enu
    down = camera_pose.down_world_enu
    forward = camera_pose.forward_world_enu
    rotation = (
        (right[0], down[0], forward[0]),
        (right[1], down[1], forward[1]),
        (right[2], down[2], forward[2]),
    )
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        factor = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * factor
        x = (rotation[2][1] - rotation[1][2]) / factor
        y = (rotation[0][2] - rotation[2][0]) / factor
        z = (rotation[1][0] - rotation[0][1]) / factor
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        factor = 2.0 * math.sqrt(max(0.0, 1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]))
        x = 0.25 * factor
        y = (rotation[0][1] + rotation[1][0]) / factor
        z = (rotation[0][2] + rotation[2][0]) / factor
        w = (rotation[2][1] - rotation[1][2]) / factor
    elif rotation[1][1] > rotation[2][2]:
        factor = 2.0 * math.sqrt(max(0.0, 1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]))
        x = (rotation[0][1] + rotation[1][0]) / factor
        y = 0.25 * factor
        z = (rotation[1][2] + rotation[2][1]) / factor
        w = (rotation[0][2] - rotation[2][0]) / factor
    else:
        factor = 2.0 * math.sqrt(max(0.0, 1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]))
        x = (rotation[0][2] + rotation[2][0]) / factor
        y = (rotation[1][2] + rotation[2][1]) / factor
        z = 0.25 * factor
        w = (rotation[1][0] - rotation[0][1]) / factor
    quaternion = (x, y, z, w)
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def build_context_payload(
    *,
    source_application: str,
    run_id: str,
    case_id: str,
    scenario_name: str,
    simulation_epoch: int,
    simulation_epoch_valid: bool,
    simulation_epoch_source: str,
    repository: str,
    branch: str,
    commit_sha: str,
    git_status: str,
    generated_sdf_sha256: str,
    nominal_baseline_m: float,
    zoom_index: float,
    zoom_index_valid: bool,
    zoom_index_source: str,
    camera_image_width_px: int,
    camera_image_height_px: int,
    camera_update_rate_hz: float,
    camera_horizontal_fov_rad: float,
    observation_mode: str,
    nominal_left_position_world_enu_m: Sequence[Any],
    nominal_left_orientation_xyzw: Sequence[Any],
    nominal_right_position_world_enu_m: Sequence[Any],
    nominal_right_orientation_xyzw: Sequence[Any],
    left_direct_rpy_deg: Sequence[Any],
    right_direct_rpy_deg: Sequence[Any],
    common_rpy_deg: Sequence[Any],
    relative_rpy_deg: Sequence[Any],
    left_effective_rpy_deg: Sequence[Any] | None = None,
    right_effective_rpy_deg: Sequence[Any] | None = None,
    valid: bool = True,
    status: str = "VALID",
) -> dict[str, Any]:
    """Create the canonical, hashable context payload.

    The payload excludes the hash itself and the ROS header.  All angles are
    degrees, positions are metres, and nominal orientations are ROS xyzw.
    ``zoom_index`` is represented as zero when its source launch default is
    unresolved; ``zoom_index_valid`` preserves that provenance explicitly.
    """

    direct_left = _vector(left_direct_rpy_deg, "left_direct_rpy_deg")
    direct_right = _vector(right_direct_rpy_deg, "right_direct_rpy_deg")
    common = _vector(common_rpy_deg, "common_rpy_deg")
    relative = _vector(relative_rpy_deg, "relative_rpy_deg")
    expected_left, expected_right = effective_optical_rpy_errors_deg(
        direct_left, direct_right, common, relative
    )
    effective_left = _vector(
        expected_left if left_effective_rpy_deg is None else left_effective_rpy_deg,
        "left_effective_rpy_deg",
    )
    effective_right = _vector(
        expected_right if right_effective_rpy_deg is None else right_effective_rpy_deg,
        "right_effective_rpy_deg",
    )
    if any(abs(effective_left[i] - expected_left[i]) > 1.0e-12 for i in range(3)):
        raise ValueError("left effective RPY does not match direct/common/relative inputs")
    if any(abs(effective_right[i] - expected_right[i]) > 1.0e-12 for i in range(3)):
        raise ValueError("right effective RPY does not match direct/common/relative inputs")

    payload = {
        "schema": CONTEXT_SCHEMA,
        "valid": bool(valid),
        "status": str(status),
        "source_application": str(source_application),
        "run_id": str(run_id),
        "case_id": str(case_id),
        "scenario_name": str(scenario_name),
        "simulation_epoch": int(simulation_epoch),
        "simulation_epoch_valid": bool(simulation_epoch_valid),
        "simulation_epoch_source": str(simulation_epoch_source),
        "repository": str(repository),
        "branch": str(branch),
        "commit_sha": str(commit_sha),
        "git_status": str(git_status),
        "generated_sdf_sha256": str(generated_sdf_sha256),
        "world_frame": WORLD_FRAME,
        "camera_error_frame": CAMERA_ERROR_FRAME,
        "camera_error_axes": CAMERA_ERROR_AXES,
        "rotation_composition": ROTATION_COMPOSITION,
        "quaternion_order": QUATERNION_ORDER,
        "nominal_baseline_m": _finite(nominal_baseline_m, "nominal_baseline_m"),
        "zoom_index": _finite(zoom_index, "zoom_index"),
        "zoom_index_valid": bool(zoom_index_valid),
        "zoom_index_source": str(zoom_index_source),
        "camera_image_width_px": int(camera_image_width_px),
        "camera_image_height_px": int(camera_image_height_px),
        "camera_update_rate_hz": _finite(camera_update_rate_hz, "camera_update_rate_hz"),
        "camera_horizontal_fov_rad": _finite(camera_horizontal_fov_rad, "camera_horizontal_fov_rad"),
        "observation_mode": str(observation_mode),
        "nominal_left_optical_pose_world_enu": _pose_payload(
            nominal_left_position_world_enu_m, nominal_left_orientation_xyzw
        ),
        "nominal_right_optical_pose_world_enu": _pose_payload(
            nominal_right_position_world_enu_m, nominal_right_orientation_xyzw
        ),
        "left_direct_rpy_deg": direct_left,
        "right_direct_rpy_deg": direct_right,
        "common_rpy_deg": common,
        "relative_rpy_deg": relative,
        "left_effective_rpy_deg": effective_left,
        "right_effective_rpy_deg": effective_right,
    }
    if payload["nominal_baseline_m"] <= 0.0:
        raise ValueError("nominal_baseline_m must be positive")
    if payload["camera_image_width_px"] <= 0 or payload["camera_image_height_px"] <= 0:
        raise ValueError("camera image dimensions must be positive")
    if payload["camera_update_rate_hz"] <= 0.0:
        raise ValueError("camera_update_rate_hz must be positive")
    if payload["camera_horizontal_fov_rad"] <= 0.0:
        raise ValueError("camera_horizontal_fov_rad must be positive")
    return payload


def canonical_context_json(payload: Mapping[str, Any]) -> str:
    """Serialize a validated context payload deterministically."""

    def canonicalize(value: Any) -> Any:
        if isinstance(value, float):
            if not math.isfinite(value):
                raise ValueError("context payload contains a non-finite number")
            if value == 0.0:
                return 0.0
            return float(format(value, ".15g"))
        if isinstance(value, Mapping):
            return {key: canonicalize(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [canonicalize(item) for item in value]
        return value

    return json.dumps(
        canonicalize(payload),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )


def context_id_sha256(payload: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical_context_json(payload).encode("utf-8")).hexdigest()


__all__ = [
    "CAMERA_ERROR_AXES",
    "CAMERA_ERROR_FRAME",
    "CONTEXT_SCHEMA",
    "QUATERNION_ORDER",
    "ROTATION_COMPOSITION",
    "SOURCE_APPLICATION",
    "WORLD_FRAME",
    "build_context_payload",
    "camera_pose_quaternion_xyzw",
    "canonical_context_json",
    "context_id_sha256",
]
