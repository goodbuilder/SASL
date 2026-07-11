from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Sequence

import numpy as np


POSITION_ABS_TOL_M = 1.0e-12
ROTATION_ABS_TOL = 1.0e-12
UNIT_VECTOR_ABS_TOL = 1.0e-12


class FrameId(str, Enum):
    """Named Simulation coordinate frames and their axis contracts.

    Units are meters for positions and radians for internal angles.
    """

    GAZEBO_WORLD_ENU = "gazebo_world_enu"
    PX4_LOCAL_NED = "px4_local_ned"
    DRONE_BODY_FRD = "drone_body_frd"
    CAMERA_OPTICAL_RDF = "camera_optical_rdf"
    BASELINE_CENTER_FRD = "baseline_center_frd"


FRAME_AXIS_CONTRACTS: dict[FrameId, str] = {
    FrameId.GAZEBO_WORLD_ENU: "X East, Y North, Z Up",
    FrameId.PX4_LOCAL_NED: "X North, Y East, Z Down",
    FrameId.DRONE_BODY_FRD: "X Front, Y Right, Z Down",
    FrameId.CAMERA_OPTICAL_RDF: "X Right, Y Down, Z Forward",
    FrameId.BASELINE_CENTER_FRD: (
        "X Front, Y Right, Z Down; origin is midpoint of left/right optical centers"
    ),
}


def _coerce_frame(frame: FrameId | str) -> FrameId:
    try:
        return FrameId(frame)
    except ValueError as exc:
        raise ValueError(f"unknown frame id: {frame!r}") from exc


def _vector3(values: Sequence[float] | np.ndarray, *, field_name: str) -> tuple[float, float, float]:
    arr = np.asarray(values, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"{field_name} must contain exactly three values")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{field_name} must contain only finite values")
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _as_array3(values: Sequence[float] | np.ndarray, *, field_name: str) -> np.ndarray:
    return np.asarray(_vector3(values, field_name=field_name), dtype=float)


def _norm(values: Sequence[float] | np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(values, dtype=float)))


def _unit_tuple_from_vector(
    values: Sequence[float] | np.ndarray,
    *,
    field_name: str,
    normalize: bool,
    norm_abs_tol: float = UNIT_VECTOR_ABS_TOL,
) -> tuple[float, float, float]:
    arr = _as_array3(values, field_name=field_name)
    norm = _norm(arr)
    if norm <= 0.0:
        raise ValueError(f"{field_name} must not be a zero vector")
    if normalize:
        arr = arr / norm
    elif not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=norm_abs_tol):
        raise ValueError(f"{field_name} must be unit length")
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _matrix3x3(
    values: Sequence[Sequence[float]] | np.ndarray,
    *,
    field_name: str,
) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    if arr.shape != (3, 3):
        raise ValueError(f"{field_name} must be a 3x3 matrix")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{field_name} must contain only finite values")
    return arr


@dataclass(frozen=True)
class FramePoint:
    """A point in a named Simulation frame.

    `xyz_m` is a meter-valued position expressed in `frame`.
    """

    xyz_m: tuple[float, float, float]
    frame: FrameId

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame", _coerce_frame(self.frame))
        object.__setattr__(self, "xyz_m", _vector3(self.xyz_m, field_name="xyz_m"))


@dataclass(frozen=True)
class FrameDirection:
    """A unit direction in a named Simulation frame.

    `xyz_unit` is unitless and must already be unit length. Use
    `FrameDirection.from_vector` to construct a direction from a non-zero vector.
    """

    xyz_unit: tuple[float, float, float]
    frame: FrameId

    def __post_init__(self) -> None:
        object.__setattr__(self, "frame", _coerce_frame(self.frame))
        object.__setattr__(
            self,
            "xyz_unit",
            _unit_tuple_from_vector(self.xyz_unit, field_name="xyz_unit", normalize=False),
        )

    @classmethod
    def from_vector(cls, xyz: Sequence[float], frame: FrameId | str) -> "FrameDirection":
        return cls(
            _unit_tuple_from_vector(xyz, field_name="direction_vector", normalize=True),
            _coerce_frame(frame),
        )


@dataclass(frozen=True)
class Ray3D:
    """A 3D ray with meter-valued origin and unit direction."""

    origin: FramePoint
    direction: FrameDirection

    def __post_init__(self) -> None:
        if self.origin.frame != self.direction.frame:
            raise ValueError("Ray3D origin and direction frames must match")

    @property
    def frame(self) -> FrameId:
        return self.origin.frame


@dataclass(frozen=True)
class RigidTransform3D:
    """Rigid transform from `source_frame` to `target_frame`.

    `rotation_target_source` is R_B_A and converts coordinates expressed in
    source frame A into target frame B: `v_B = R_B_A @ v_A`.

    `origin_target_in_source_m` is the target-frame origin expressed in source
    frame coordinates, in meters. Points use
    `p_B = R_B_A @ (p_A - origin_target_in_source_m)`. Directions do not use
    translation.
    """

    source_frame: FrameId
    target_frame: FrameId
    rotation_target_source: tuple[tuple[float, float, float], ...]
    origin_target_in_source_m: tuple[float, float, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "source_frame", _coerce_frame(self.source_frame))
        object.__setattr__(self, "target_frame", _coerce_frame(self.target_frame))
        object.__setattr__(
            self,
            "rotation_target_source",
            validate_rotation_matrix(self.rotation_target_source),
        )
        object.__setattr__(
            self,
            "origin_target_in_source_m",
            _vector3(self.origin_target_in_source_m, field_name="origin_target_in_source_m"),
        )


def validate_rotation_matrix(
    rotation: Sequence[Sequence[float]] | np.ndarray,
    *,
    abs_tol: float = ROTATION_ABS_TOL,
) -> tuple[tuple[float, float, float], ...]:
    """Validate a proper 3x3 rotation matrix.

    The matrix must be finite, orthonormal, and have determinant +1. Reflection
    matrices with determinant -1 are rejected.
    """

    arr = _matrix3x3(rotation, field_name="rotation")
    identity = arr.T @ arr
    if not np.allclose(identity, np.eye(3), rtol=0.0, atol=abs_tol):
        raise ValueError("rotation matrix must be orthonormal")
    determinant = float(np.linalg.det(arr))
    if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=abs_tol):
        raise ValueError("rotation matrix determinant must be +1")
    return tuple(tuple(float(value) for value in row) for row in arr)


def transform_point(point: FramePoint, transform: RigidTransform3D) -> FramePoint:
    """Transform a meter-valued point with `p_B = R_B_A @ (p_A - o_A_B)`."""

    if point.frame != transform.source_frame:
        raise ValueError("point frame does not match transform source frame")
    rotation = np.asarray(transform.rotation_target_source, dtype=float)
    origin = np.asarray(transform.origin_target_in_source_m, dtype=float)
    transformed = rotation @ (np.asarray(point.xyz_m, dtype=float) - origin)
    return FramePoint(_vector3(transformed, field_name="transformed_point"), transform.target_frame)


def transform_direction(direction: FrameDirection, transform: RigidTransform3D) -> FrameDirection:
    """Transform a unit direction with `d_B = R_B_A @ d_A`.

    Translation is intentionally not applied to directions.
    """

    if direction.frame != transform.source_frame:
        raise ValueError("direction frame does not match transform source frame")
    rotation = np.asarray(transform.rotation_target_source, dtype=float)
    transformed = rotation @ np.asarray(direction.xyz_unit, dtype=float)
    norm = _norm(transformed)
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=UNIT_VECTOR_ABS_TOL):
        raise ValueError("transformed direction is not unit length")
    return FrameDirection(
        _unit_tuple_from_vector(transformed, field_name="transformed_direction", normalize=True),
        transform.target_frame,
    )


def transform_ray(ray: Ray3D, transform: RigidTransform3D) -> Ray3D:
    """Transform a ray origin as a point and ray direction as a direction."""

    if ray.frame != transform.source_frame:
        raise ValueError("ray frame does not match transform source frame")
    return Ray3D(
        origin=transform_point(ray.origin, transform),
        direction=transform_direction(ray.direction, transform),
    )


def _normalized_quaternion_wxyz(q_wxyz: Iterable[float]) -> tuple[float, float, float, float]:
    values = tuple(float(value) for value in q_wxyz)
    if len(values) != 4:
        raise ValueError("quaternion must contain four values")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain only finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        raise ValueError("quaternion must not be zero")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def quaternion_wxyz_to_xyzw(q_wxyz: Iterable[float]) -> tuple[float, float, float, float]:
    """Convert a Hamilton quaternion from PX4 `(w, x, y, z)` to SciPy `(x, y, z, w)`.

    The returned quaternion is normalized. Internal angle units are radians.
    """

    w, x, y, z = _normalized_quaternion_wxyz(q_wxyz)
    return (x, y, z, w)


def quaternion_xyzw_to_wxyz(q_xyzw: Iterable[float]) -> tuple[float, float, float, float]:
    """Convert a SciPy-order quaternion `(x, y, z, w)` to PX4 `(w, x, y, z)`.

    The returned quaternion is normalized. Internal angle units are radians.
    """

    values = tuple(float(value) for value in q_xyzw)
    if len(values) != 4:
        raise ValueError("quaternion must contain four values")
    x, y, z, w = values
    return _normalized_quaternion_wxyz((w, x, y, z))


def rotation_matrix_from_quaternion_wxyz(
    q_wxyz: Iterable[float],
) -> tuple[tuple[float, float, float], ...]:
    """Build a proper rotation matrix from PX4/Hamilton `(w, x, y, z)` order."""

    w, x, y, z = _normalized_quaternion_wxyz(q_wxyz)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    rotation = (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )
    return validate_rotation_matrix(rotation)


__all__ = [
    "FRAME_AXIS_CONTRACTS",
    "FrameDirection",
    "FrameId",
    "FramePoint",
    "POSITION_ABS_TOL_M",
    "ROTATION_ABS_TOL",
    "Ray3D",
    "RigidTransform3D",
    "UNIT_VECTOR_ABS_TOL",
    "quaternion_wxyz_to_xyzw",
    "quaternion_xyzw_to_wxyz",
    "rotation_matrix_from_quaternion_wxyz",
    "transform_direction",
    "transform_point",
    "transform_ray",
    "validate_rotation_matrix",
]
