"""Pure adapter from Camera Optical RDF attitude errors to Gazebo model rotation.

The adapter is rotation-only: it does not accept or return camera translation,
baseline length, ROS messages, Gazebo process state, files, or environment
state. It maps the Phase 3A Camera RDF attitude contract to the Gazebo camera
model-frame orientation boundary.

Frame notation follows the Simulation coordinate contract:

* W = gazebo_world_enu
* B = baseline_center_frd
* M = Gazebo camera model frame
* C = camera_optical_rdf

`R_B_A` converts A-frame components into B-frame components:
`v_B = R_B_A @ v_A`.

The fixed model-to-optical rotation is derived from nominal rotations:

    R_M_C_fixed = (R_W_M_nominal).T @ R_W_C_nominal

For an attitude error, Phase 3A supplies:

    R_delta_C = R_Y(yaw_C) @ R_X(pitch_C) @ R_Z(roll_C)

The authoritative adapter path is:

    R_B_C_actual = R_B_C_nominal @ R_delta_C
    R_W_C_actual = R_W_B @ R_B_C_actual
    R_W_M_actual = R_W_C_actual @ (R_M_C_fixed).T
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from stereo_ground_sim.attitude_error_model import (
    CameraAttitudeErrorDeg,
    StereoAttitudeErrorConfig,
    rotation_from_attitude_error_deg,
)
from stereo_ground_sim.coordinate_frames import (
    quaternion_xyzw_to_wxyz,
    rotation_matrix_from_quaternion_wxyz,
    validate_rotation_matrix,
)
from stereo_ground_sim.stereo_triangulation_adapter import R_FRD_RDF


SDF_RPY_STATUS_OK = "ok"
SDF_RPY_STATUS_GIMBAL_LOCK = "gimbal_lock"
SDF_RPY_STATUS_ROUND_TRIP_FAILED = "round_trip_failed"

R_B_W: tuple[tuple[float, float, float], ...] = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)


@dataclass(frozen=True)
class SdfRpyRotation:
    valid: bool
    status: str
    rpy_rad: tuple[float, float, float] | None
    round_trip_max_error: float | None

    def __post_init__(self) -> None:
        if not isinstance(self.valid, bool):
            raise TypeError("valid must be a bool")
        if self.status not in {
            SDF_RPY_STATUS_OK,
            SDF_RPY_STATUS_GIMBAL_LOCK,
            SDF_RPY_STATUS_ROUND_TRIP_FAILED,
        }:
            raise ValueError("status must be a known SDF RPY status")
        if self.valid != (self.status == SDF_RPY_STATUS_OK):
            raise ValueError("valid must match status == ok")
        if self.valid:
            if self.rpy_rad is None:
                raise ValueError("valid SDF RPY must include rpy_rad")
            object.__setattr__(self, "rpy_rad", _vector3(self.rpy_rad, field_name="rpy_rad"))
            if self.round_trip_max_error is None:
                raise ValueError("valid SDF RPY must include round_trip_max_error")
            object.__setattr__(
                self,
                "round_trip_max_error",
                _finite_float(self.round_trip_max_error, field_name="round_trip_max_error"),
            )
        else:
            if self.rpy_rad is not None:
                raise ValueError("invalid SDF RPY must not include rpy_rad")
            if self.status == SDF_RPY_STATUS_ROUND_TRIP_FAILED:
                if self.round_trip_max_error is None:
                    raise ValueError("round_trip_failed must include round_trip_max_error")
                object.__setattr__(
                    self,
                    "round_trip_max_error",
                    _finite_float(self.round_trip_max_error, field_name="round_trip_max_error"),
                )
            elif self.round_trip_max_error is not None:
                raise ValueError("gimbal_lock must not include round_trip_max_error")


@dataclass(frozen=True)
class GazeboCameraModelRotation:
    """Gazebo model-frame orientation result.

    rotation_world_model is R_W_M.
    quaternion_xyzw is the same R_W_M rotation in Gazebo/SciPy order.
    sdf_rpy is an SDF boundary representation using Rz(yaw) Ry(pitch) Rx(roll).
    """

    rotation_world_model: tuple[tuple[float, float, float], ...]
    quaternion_xyzw: tuple[float, float, float, float]
    sdf_rpy: SdfRpyRotation

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "rotation_world_model",
            _strict_rotation_matrix(self.rotation_world_model, field_name="rotation_world_model"),
        )
        object.__setattr__(
            self,
            "quaternion_xyzw",
            _strict_quaternion_xyzw(self.quaternion_xyzw, field_name="quaternion_xyzw"),
        )
        if not isinstance(self.sdf_rpy, SdfRpyRotation):
            raise TypeError("sdf_rpy must be an SdfRpyRotation")
        quaternion_rotation = rotation_matrix_from_quaternion_xyzw(self.quaternion_xyzw)
        if _matrix_max_abs_delta(self.rotation_world_model, quaternion_rotation) > 1.0e-12:
            raise ValueError("quaternion_xyzw does not match rotation_world_model")


@dataclass(frozen=True)
class GazeboStereoModelRotations:
    left: GazeboCameraModelRotation
    right: GazeboCameraModelRotation

    def __post_init__(self) -> None:
        if not isinstance(self.left, GazeboCameraModelRotation):
            raise TypeError("left must be a GazeboCameraModelRotation")
        if not isinstance(self.right, GazeboCameraModelRotation):
            raise TypeError("right must be a GazeboCameraModelRotation")


def _finite_float(value: object, *, field_name: str) -> float:
    if isinstance(value, (bool, np.bool_, str)):
        raise TypeError(f"{field_name} must be a finite float")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{field_name} must be a finite float") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{field_name} must be finite")
    return parsed


def _vector3(values: Sequence[object], *, field_name: str) -> tuple[float, float, float]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{field_name} must contain three finite floats")
    if len(values) != 3:
        raise ValueError(f"{field_name} must contain three values")
    return tuple(_finite_float(value, field_name=f"{field_name}[{idx}]") for idx, value in enumerate(values))  # type: ignore[return-value]


def _strict_matrix3x3(rotation: Sequence[Sequence[object]] | np.ndarray, *, field_name: str) -> np.ndarray:
    if isinstance(rotation, (str, bytes)):
        raise TypeError(f"{field_name} must be a 3x3 finite numeric matrix")
    arr = np.asarray(rotation, dtype=object)
    if arr.shape != (3, 3):
        raise ValueError(f"{field_name} must be 3x3")
    numeric = np.empty((3, 3), dtype=float)
    for row in range(3):
        for col in range(3):
            numeric[row, col] = _finite_float(arr[row, col], field_name=f"{field_name}[{row}][{col}]")
    return numeric


def _strict_rotation_matrix(
    rotation: Sequence[Sequence[object]] | np.ndarray,
    *,
    field_name: str,
) -> tuple[tuple[float, float, float], ...]:
    return validate_rotation_matrix(_strict_matrix3x3(rotation, field_name=field_name))


def _strict_quaternion_xyzw(values: Iterable[object], *, field_name: str) -> tuple[float, float, float, float]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{field_name} must contain four finite floats")
    parsed = tuple(values)
    if len(parsed) != 4:
        raise ValueError(f"{field_name} must contain four values")
    xyzw = tuple(_finite_float(value, field_name=f"{field_name}[{idx}]") for idx, value in enumerate(parsed))
    norm = math.sqrt(sum(value * value for value in xyzw))
    if norm <= 0.0:
        raise ValueError(f"{field_name} must not be zero")
    return tuple(value / norm for value in xyzw)  # type: ignore[return-value]


def _matrix_tuple(rotation: np.ndarray) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(float(value) for value in row) for row in rotation)


def _matrix_max_abs_delta(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> float:
    return float(np.max(np.abs(np.asarray(left, dtype=float) - np.asarray(right, dtype=float))))


def _r_x(angle_rad: float) -> np.ndarray:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.asarray(
        (
            (1.0, 0.0, 0.0),
            (0.0, cosine, -sine),
            (0.0, sine, cosine),
        ),
        dtype=float,
    )


def _r_y(angle_rad: float) -> np.ndarray:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.asarray(
        (
            (cosine, 0.0, sine),
            (0.0, 1.0, 0.0),
            (-sine, 0.0, cosine),
        ),
        dtype=float,
    )


def _r_z(angle_rad: float) -> np.ndarray:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return np.asarray(
        (
            (cosine, -sine, 0.0),
            (sine, cosine, 0.0),
            (0.0, 0.0, 1.0),
        ),
        dtype=float,
    )


def _r_w_b() -> np.ndarray:
    return np.asarray(R_B_W, dtype=float).T


def _r_b_c_nominal() -> np.ndarray:
    return np.asarray(R_FRD_RDF, dtype=float)


def _r_w_m_nominal() -> np.ndarray:
    return _r_z(math.pi / 2.0)


def _canonical_quaternion_xyzw(xyzw: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = xyzw
    tolerance = 1.0e-15
    if w < -tolerance:
        return (-x, -y, -z, -w)
    if abs(w) <= tolerance:
        for value in (x, y, z):
            if abs(value) > tolerance:
                return xyzw if value > 0.0 else (-x, -y, -z, -w)
    return xyzw


def derive_fixed_model_to_optical_rotation() -> tuple[tuple[float, float, float], ...]:
    """Return `R_M_C_fixed = R_W_M_nominal.T @ R_W_C_nominal`."""

    r_w_c_nominal = _r_w_b() @ _r_b_c_nominal()
    r_m_c_fixed = _r_w_m_nominal().T @ r_w_c_nominal
    return validate_rotation_matrix(r_m_c_fixed)


def rotation_matrix_from_sdf_rpy_rad(
    roll_rad: object,
    pitch_rad: object,
    yaw_rad: object,
) -> tuple[tuple[float, float, float], ...]:
    """Return SDF RPY rotation `R = R_Z(yaw) @ R_Y(pitch) @ R_X(roll)`."""

    roll = _finite_float(roll_rad, field_name="roll_rad")
    pitch = _finite_float(pitch_rad, field_name="pitch_rad")
    yaw = _finite_float(yaw_rad, field_name="yaw_rad")
    return validate_rotation_matrix(_r_z(yaw) @ _r_y(pitch) @ _r_x(roll))


def rotation_matrix_to_sdf_rpy(
    rotation_world_model: Sequence[Sequence[object]] | np.ndarray,
    *,
    gimbal_lock_cos_threshold: object = 1.0e-10,
    round_trip_abs_tol: object = 1.0e-10,
) -> SdfRpyRotation:
    """Decompose `R_W_M` to SDF RPY for `R_Z(yaw) @ R_Y(pitch) @ R_X(roll)`."""

    rotation = np.asarray(_strict_rotation_matrix(rotation_world_model, field_name="rotation_world_model"), dtype=float)
    threshold = _finite_float(gimbal_lock_cos_threshold, field_name="gimbal_lock_cos_threshold")
    tolerance = _finite_float(round_trip_abs_tol, field_name="round_trip_abs_tol")
    if threshold < 0.0:
        raise ValueError("gimbal_lock_cos_threshold must be non-negative")
    if tolerance < 0.0:
        raise ValueError("round_trip_abs_tol must be non-negative")

    sin_pitch = max(-1.0, min(1.0, -float(rotation[2, 0])))
    pitch = math.asin(sin_pitch)
    if abs(math.cos(pitch)) <= threshold:
        return SdfRpyRotation(
            valid=False,
            status=SDF_RPY_STATUS_GIMBAL_LOCK,
            rpy_rad=None,
            round_trip_max_error=None,
        )

    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    round_trip = rotation_matrix_from_sdf_rpy_rad(roll, pitch, yaw)
    residual = _matrix_max_abs_delta(rotation, round_trip)
    if residual > tolerance:
        return SdfRpyRotation(
            valid=False,
            status=SDF_RPY_STATUS_ROUND_TRIP_FAILED,
            rpy_rad=None,
            round_trip_max_error=residual,
        )
    return SdfRpyRotation(
        valid=True,
        status=SDF_RPY_STATUS_OK,
        rpy_rad=(roll, pitch, yaw),
        round_trip_max_error=residual,
    )


def rotation_matrix_to_quaternion_xyzw(
    rotation_world_model: Sequence[Sequence[object]] | np.ndarray,
) -> tuple[float, float, float, float]:
    """Convert a proper rotation matrix to deterministic normalized `(x,y,z,w)`."""

    rotation = np.asarray(_strict_rotation_matrix(rotation_world_model, field_name="rotation_world_model"), dtype=float)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = [float(rotation[0, 0]), float(rotation[1, 1]), float(rotation[2, 2])]
        axis = int(np.argmax(diagonal))
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
    return _canonical_quaternion_xyzw(_strict_quaternion_xyzw((x, y, z, w), field_name="quaternion_xyzw"))


def rotation_matrix_from_quaternion_xyzw(
    quaternion_xyzw: Iterable[object],
) -> tuple[tuple[float, float, float], ...]:
    """Build a rotation matrix from Gazebo/SciPy-order `(x,y,z,w)`."""

    normalized = _strict_quaternion_xyzw(quaternion_xyzw, field_name="quaternion_xyzw")
    return rotation_matrix_from_quaternion_wxyz(quaternion_xyzw_to_wxyz(normalized))


def gazebo_model_rotation_from_camera_attitude_error(
    error: CameraAttitudeErrorDeg,
) -> GazeboCameraModelRotation:
    """Return the Gazebo model rotation for one Camera RDF attitude error."""

    if not isinstance(error, CameraAttitudeErrorDeg):
        raise TypeError("error must be a CameraAttitudeErrorDeg")
    r_delta_c = np.asarray(rotation_from_attitude_error_deg(error), dtype=float)
    r_b_c_actual = _r_b_c_nominal() @ r_delta_c
    r_w_c_actual = _r_w_b() @ r_b_c_actual
    r_m_c_fixed = np.asarray(derive_fixed_model_to_optical_rotation(), dtype=float)
    r_w_m_actual = validate_rotation_matrix(r_w_c_actual @ r_m_c_fixed.T)
    return GazeboCameraModelRotation(
        rotation_world_model=r_w_m_actual,
        quaternion_xyzw=rotation_matrix_to_quaternion_xyzw(r_w_m_actual),
        sdf_rpy=rotation_matrix_to_sdf_rpy(r_w_m_actual),
    )


def gazebo_stereo_model_rotations_from_attitude_errors(
    config: StereoAttitudeErrorConfig,
) -> GazeboStereoModelRotations:
    """Return left/right Gazebo model rotations for a stereo attitude config."""

    if not isinstance(config, StereoAttitudeErrorConfig):
        raise TypeError("config must be a StereoAttitudeErrorConfig")
    return GazeboStereoModelRotations(
        left=gazebo_model_rotation_from_camera_attitude_error(config.left),
        right=gazebo_model_rotation_from_camera_attitude_error(config.right),
    )


__all__ = [
    "GazeboCameraModelRotation",
    "GazeboStereoModelRotations",
    "R_B_W",
    "SDF_RPY_STATUS_GIMBAL_LOCK",
    "SDF_RPY_STATUS_OK",
    "SDF_RPY_STATUS_ROUND_TRIP_FAILED",
    "SdfRpyRotation",
    "derive_fixed_model_to_optical_rotation",
    "gazebo_model_rotation_from_camera_attitude_error",
    "gazebo_stereo_model_rotations_from_attitude_errors",
    "rotation_matrix_from_quaternion_xyzw",
    "rotation_matrix_from_sdf_rpy_rad",
    "rotation_matrix_to_quaternion_xyzw",
    "rotation_matrix_to_sdf_rpy",
]
