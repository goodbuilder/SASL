"""Read-only adapter and validator for Simulation experiment provenance."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from .pose_math import (
    CAMERA_OPTICAL_FRAME,
    Matrix3,
    Pose3D,
    WORLD_FRAME,
    quaternion_xyzw_to_matrix,
)


CONTEXT_SCHEMA = "simulation_experiment_context/v1"
SOURCE_APPLICATION = "Simulation"
CAMERA_ERROR_FRAME = "camera_optical_rdf"
CAMERA_ERROR_AXES = "right_down_forward"
ROTATION_COMPOSITION = "R_down_yaw@R_right_pitch@R_forward_roll"
QUATERNION_ORDER = "xyzw"

WAITING_FOR_EXPERIMENT_CONTEXT = "WAITING_FOR_EXPERIMENT_CONTEXT"
CONTEXT_VALID = "CONTEXT_VALID"
CONTEXT_SCHEMA_MISMATCH = "CONTEXT_SCHEMA_MISMATCH"
CONTEXT_SOURCE_MISMATCH = "CONTEXT_SOURCE_MISMATCH"
CONTEXT_FRAME_MISMATCH = "CONTEXT_FRAME_MISMATCH"
CONTEXT_ROTATION_CONVENTION_MISMATCH = "CONTEXT_ROTATION_CONVENTION_MISMATCH"
CONTEXT_QUATERNION_ORDER_MISMATCH = "CONTEXT_QUATERNION_ORDER_MISMATCH"
CONTEXT_NONFINITE = "CONTEXT_NONFINITE"
CONTEXT_HASH_MISMATCH = "CONTEXT_HASH_MISMATCH"
CONTEXT_EFFECTIVE_RPY_MISMATCH = "CONTEXT_EFFECTIVE_RPY_MISMATCH"
CONTEXT_MUTATED_IN_RUN = "CONTEXT_MUTATED_IN_RUN"
CONTEXT_CAMERA_SDF_MISMATCH = "CONTEXT_CAMERA_SDF_MISMATCH"


def _field(value: Any, name: str) -> Any:
    if isinstance(value, Mapping):
        if name not in value:
            raise KeyError(name)
        return value[name]
    if not hasattr(value, name):
        raise KeyError(name)
    return getattr(value, name)


def _finite(value: Any, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _vector(value: Any, name: str) -> list[float]:
    if isinstance(value, Mapping):
        raw = [_field(value, "x"), _field(value, "y"), _field(value, "z")]
    else:
        raw = [getattr(value, axis) for axis in ("x", "y", "z")] if hasattr(value, "x") else list(value)
    if len(raw) != 3:
        raise ValueError(f"{name} must contain three values")
    return [_finite(item, name) for item in raw]


def _quaternion(value: Any, name: str) -> list[float]:
    if isinstance(value, Mapping):
        raw = [_field(value, axis) for axis in ("x", "y", "z", "w")]
    else:
        raw = [getattr(value, axis) for axis in ("x", "y", "z", "w")] if hasattr(value, "x") else list(value)
    if len(raw) != 4:
        raise ValueError(f"{name} must contain four values")
    parsed = [_finite(item, name) for item in raw]
    norm = math.sqrt(sum(item * item for item in parsed))
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError(f"{name} must have a non-zero finite norm")
    normalized = [item / norm for item in parsed]
    if normalized[3] < 0.0 or (
        abs(normalized[3]) <= 1.0e-15
        and next((item for item in normalized[:3] if abs(item) > 1.0e-15), 0.0) < 0.0
    ):
        normalized = [-item for item in normalized]
    # Verify the normalized boundary using the application-owned SO(3) code.
    quaternion_xyzw_to_matrix(normalized)
    return normalized


def _pose(value: Any, name: str) -> tuple[list[float], list[float], Matrix3]:
    position = _vector(_field(value, "position"), f"{name}.position")
    orientation = _quaternion(_field(value, "orientation"), f"{name}.orientation_xyzw")
    return position, orientation, quaternion_xyzw_to_matrix(orientation)


def _effective_rpy(
    left_direct: Sequence[float],
    right_direct: Sequence[float],
    common: Sequence[float],
    relative: Sequence[float],
) -> tuple[list[float], list[float]]:
    values = [_vector(item, name) for item, name in (
        (left_direct, "left_direct_rpy_deg"),
        (right_direct, "right_direct_rpy_deg"),
        (common, "common_rpy_deg"),
        (relative, "relative_rpy_deg"),
    )]
    left_direct_v, right_direct_v, common_v, relative_v = values
    left = [left_direct_v[i] + common_v[i] - 0.5 * relative_v[i] for i in range(3)]
    right = [right_direct_v[i] + common_v[i] + 0.5 * relative_v[i] for i in range(3)]
    return left, right


def context_payload_from_message(message: Any) -> dict[str, Any]:
    """Recreate the producer's canonical hash payload without importing it."""

    left_position, left_orientation, _ = _pose(
        _field(message, "nominal_left_optical_pose_world_enu"),
        "nominal_left_optical_pose_world_enu",
    )
    right_position, right_orientation, _ = _pose(
        _field(message, "nominal_right_optical_pose_world_enu"),
        "nominal_right_optical_pose_world_enu",
    )
    left_direct = _vector(_field(message, "left_direct_rpy_deg"), "left_direct_rpy_deg")
    right_direct = _vector(_field(message, "right_direct_rpy_deg"), "right_direct_rpy_deg")
    common = _vector(_field(message, "common_rpy_deg"), "common_rpy_deg")
    relative = _vector(_field(message, "relative_rpy_deg"), "relative_rpy_deg")
    effective_left = _vector(_field(message, "left_effective_rpy_deg"), "left_effective_rpy_deg")
    effective_right = _vector(_field(message, "right_effective_rpy_deg"), "right_effective_rpy_deg")
    return {
        "schema": str(_field(message, "schema")),
        "valid": bool(_field(message, "valid")),
        "status": str(_field(message, "status")),
        "source_application": str(_field(message, "source_application")),
        "run_id": str(_field(message, "run_id")),
        "case_id": str(_field(message, "case_id")),
        "scenario_name": str(_field(message, "scenario_name")),
        "simulation_epoch": int(_field(message, "simulation_epoch")),
        "simulation_epoch_valid": bool(_field(message, "simulation_epoch_valid")),
        "simulation_epoch_source": str(_field(message, "simulation_epoch_source")),
        "repository": str(_field(message, "repository")),
        "branch": str(_field(message, "branch")),
        "commit_sha": str(_field(message, "commit_sha")),
        "git_status": str(_field(message, "git_status")),
        "generated_sdf_sha256": str(_field(message, "generated_sdf_sha256")),
        "world_frame": str(_field(message, "world_frame")),
        "camera_error_frame": str(_field(message, "camera_error_frame")),
        "camera_error_axes": str(_field(message, "camera_error_axes")),
        "rotation_composition": str(_field(message, "rotation_composition")),
        "quaternion_order": str(_field(message, "quaternion_order")),
        "nominal_baseline_m": _finite(_field(message, "nominal_baseline_m"), "nominal_baseline_m"),
        "zoom_index": _finite(_field(message, "zoom_index"), "zoom_index"),
        "zoom_index_valid": bool(_field(message, "zoom_index_valid")),
        "zoom_index_source": str(_field(message, "zoom_index_source")),
        "camera_image_width_px": int(_field(message, "camera_image_width_px")),
        "camera_image_height_px": int(_field(message, "camera_image_height_px")),
        "camera_update_rate_hz": _finite(_field(message, "camera_update_rate_hz"), "camera_update_rate_hz"),
        "camera_horizontal_fov_rad": _finite(_field(message, "camera_horizontal_fov_rad"), "camera_horizontal_fov_rad"),
        "observation_mode": str(_field(message, "observation_mode")),
        "nominal_left_optical_pose_world_enu": {
            "position_world_enu_m": left_position,
            "orientation_xyzw": left_orientation,
        },
        "nominal_right_optical_pose_world_enu": {
            "position_world_enu_m": right_position,
            "orientation_xyzw": right_orientation,
        },
        "left_direct_rpy_deg": left_direct,
        "right_direct_rpy_deg": right_direct,
        "common_rpy_deg": common,
        "relative_rpy_deg": relative,
        "left_effective_rpy_deg": effective_left,
        "right_effective_rpy_deg": effective_right,
    }


def canonical_context_json(payload: Mapping[str, Any]) -> str:
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

    return json.dumps(canonicalize(payload), sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


def context_id_sha256(payload: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical_context_json(payload).encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class ValidatedSimulationExperimentContext:
    context_id_sha256: str
    run_id: str
    case_id: str
    scenario_name: str
    simulation_epoch: int
    simulation_epoch_valid: bool
    simulation_epoch_source: str
    generated_sdf_sha256: str
    nominal_left_pose_world: Pose3D
    nominal_right_pose_world: Pose3D
    left_direct_rpy_deg: tuple[float, float, float]
    right_direct_rpy_deg: tuple[float, float, float]
    common_rpy_deg: tuple[float, float, float]
    relative_rpy_deg: tuple[float, float, float]
    left_effective_rpy_deg: tuple[float, float, float]
    right_effective_rpy_deg: tuple[float, float, float]
    world_frame: str
    camera_error_frame: str
    camera_error_axes: str
    rotation_composition: str
    quaternion_order: str
    valid: bool = True
    invalid_reason: str = ""


@dataclass(frozen=True)
class ContextValidationResult:
    context: ValidatedSimulationExperimentContext | None
    valid: bool
    status: str
    reason: str


def validate_simulation_context(
    message: Any,
    *,
    expected_schema: str = CONTEXT_SCHEMA,
    require_hash_match: bool = True,
) -> ContextValidationResult:
    """Validate one producer context and return an immutable internal type."""

    try:
        if not bool(_field(message, "valid")):
            raise ValueError("producer marked the context invalid")
        if str(_field(message, "schema")) != expected_schema:
            return ContextValidationResult(None, False, CONTEXT_SCHEMA_MISMATCH, "context schema is not supported")
        if str(_field(message, "source_application")) != SOURCE_APPLICATION:
            return ContextValidationResult(None, False, CONTEXT_SOURCE_MISMATCH, "context source is not Simulation")
        if str(_field(message, "world_frame")) != WORLD_FRAME:
            return ContextValidationResult(None, False, CONTEXT_FRAME_MISMATCH, "world frame is not gazebo_world_enu")
        if str(_field(message, "camera_error_frame")) != CAMERA_ERROR_FRAME or str(_field(message, "camera_error_axes")) != CAMERA_ERROR_AXES:
            return ContextValidationResult(None, False, CONTEXT_FRAME_MISMATCH, "camera error frame/axes contract mismatch")
        if str(_field(message, "rotation_composition")) != ROTATION_COMPOSITION:
            return ContextValidationResult(None, False, CONTEXT_ROTATION_CONVENTION_MISMATCH, "optical rotation composition mismatch")
        if str(_field(message, "quaternion_order")) != QUATERNION_ORDER:
            return ContextValidationResult(None, False, CONTEXT_QUATERNION_ORDER_MISMATCH, "context quaternion order is not xyzw")

        payload = context_payload_from_message(message)
        if not payload["generated_sdf_sha256"]:
            return ContextValidationResult(
                None,
                False,
                CONTEXT_HASH_MISMATCH,
                "generated SDF identity is missing from the Simulation context",
            )
        if payload["nominal_baseline_m"] <= 0.0:
            raise ValueError("nominal baseline must be positive")
        if payload["camera_image_width_px"] <= 0 or payload["camera_image_height_px"] <= 0:
            raise ValueError("camera image dimensions must be positive")
        if payload["camera_update_rate_hz"] <= 0.0 or payload["camera_horizontal_fov_rad"] <= 0.0:
            raise ValueError("camera contract rates/FOV must be positive")
        expected_left, expected_right = _effective_rpy(
            payload["left_direct_rpy_deg"],
            payload["right_direct_rpy_deg"],
            payload["common_rpy_deg"],
            payload["relative_rpy_deg"],
        )
        if any(abs(expected_left[i] - payload["left_effective_rpy_deg"][i]) > 1.0e-12 for i in range(3)) or any(abs(expected_right[i] - payload["right_effective_rpy_deg"][i]) > 1.0e-12 for i in range(3)):
            return ContextValidationResult(None, False, CONTEXT_EFFECTIVE_RPY_MISMATCH, "published effective RPY does not match direct/common/relative RPY")
        computed_hash = context_id_sha256(payload)
        published_hash = str(_field(message, "context_id_sha256"))
        if require_hash_match and (not published_hash or published_hash != computed_hash):
            return ContextValidationResult(None, False, CONTEXT_HASH_MISMATCH, "context_id_sha256 does not match canonical payload")
        left_position, left_orientation, left_rotation = _pose(_field(message, "nominal_left_optical_pose_world_enu"), "nominal_left_optical_pose_world_enu")
        right_position, right_orientation, right_rotation = _pose(_field(message, "nominal_right_optical_pose_world_enu"), "nominal_right_optical_pose_world_enu")
        context = ValidatedSimulationExperimentContext(
            context_id_sha256=published_hash or computed_hash,
            run_id=payload["run_id"],
            case_id=payload["case_id"],
            scenario_name=payload["scenario_name"],
            simulation_epoch=payload["simulation_epoch"],
            simulation_epoch_valid=payload["simulation_epoch_valid"],
            simulation_epoch_source=payload["simulation_epoch_source"],
            generated_sdf_sha256=payload["generated_sdf_sha256"],
            nominal_left_pose_world=Pose3D(
                tuple(left_position), left_rotation, CAMERA_OPTICAL_FRAME
            ),
            nominal_right_pose_world=Pose3D(
                tuple(right_position), right_rotation, CAMERA_OPTICAL_FRAME
            ),
            left_direct_rpy_deg=tuple(payload["left_direct_rpy_deg"]),
            right_direct_rpy_deg=tuple(payload["right_direct_rpy_deg"]),
            common_rpy_deg=tuple(payload["common_rpy_deg"]),
            relative_rpy_deg=tuple(payload["relative_rpy_deg"]),
            left_effective_rpy_deg=tuple(payload["left_effective_rpy_deg"]),
            right_effective_rpy_deg=tuple(payload["right_effective_rpy_deg"]),
            world_frame=payload["world_frame"],
            camera_error_frame=payload["camera_error_frame"],
            camera_error_axes=payload["camera_error_axes"],
            rotation_composition=payload["rotation_composition"],
            quaternion_order=payload["quaternion_order"],
        )
        return ContextValidationResult(context, True, CONTEXT_VALID, "context validated")
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError, FloatingPointError) as exc:
        return ContextValidationResult(None, False, CONTEXT_NONFINITE, str(exc))


class SimulationContextStore:
    """Accept one immutable context per run and fail closed on mutation."""

    def __init__(self, *, expected_schema: str = CONTEXT_SCHEMA, require_hash_match: bool = True) -> None:
        self.expected_schema = expected_schema
        self.require_hash_match = require_hash_match
        self.context: ValidatedSimulationExperimentContext | None = None
        self.status = WAITING_FOR_EXPERIMENT_CONTEXT
        self.reason = "waiting for /simulation/experiment_context"

    def accept(self, message: Any) -> ContextValidationResult:
        result = validate_simulation_context(
            message,
            expected_schema=self.expected_schema,
            require_hash_match=self.require_hash_match,
        )
        if not result.valid or result.context is None:
            self.context = None
            self.status = result.status
            self.reason = result.reason
            return result
        if self.context is not None and self.context.run_id == result.context.run_id and self.context.context_id_sha256 != result.context.context_id_sha256:
            self.context = None
            self.status = CONTEXT_MUTATED_IN_RUN
            self.reason = "Simulation context identity changed within the same run"
            return ContextValidationResult(None, False, self.status, self.reason)
        self.context = result.context
        self.status = CONTEXT_VALID
        self.reason = result.reason
        return result


__all__ = [
    "CAMERA_ERROR_AXES",
    "CAMERA_ERROR_FRAME",
    "CONTEXT_CAMERA_SDF_MISMATCH",
    "CONTEXT_EFFECTIVE_RPY_MISMATCH",
    "CONTEXT_FRAME_MISMATCH",
    "CONTEXT_HASH_MISMATCH",
    "CONTEXT_MUTATED_IN_RUN",
    "CONTEXT_NONFINITE",
    "CONTEXT_QUATERNION_ORDER_MISMATCH",
    "CONTEXT_ROTATION_CONVENTION_MISMATCH",
    "CONTEXT_SCHEMA",
    "CONTEXT_SCHEMA_MISMATCH",
    "CONTEXT_SOURCE_MISMATCH",
    "CONTEXT_VALID",
    "ContextValidationResult",
    "QUATERNION_ORDER",
    "ROTATION_COMPOSITION",
    "SimulationContextStore",
    "ValidatedSimulationExperimentContext",
    "WAITING_FOR_EXPERIMENT_CONTEXT",
    "canonical_context_json",
    "context_id_sha256",
    "context_payload_from_message",
    "validate_simulation_context",
]
