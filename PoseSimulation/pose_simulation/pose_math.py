"""Pure 3-D pose mathematics for the independent PoseSimulation app.

The module intentionally has no ROS dependency.  Every rotation follows the
explicit convention ``R_B_A``: a vector expressed in frame A is multiplied by
``R_B_A`` to obtain its components in frame B.  Positions in this module are
metres in ``gazebo_world_enu`` unless a function name or parameter says
otherwise.

The physical FRD look-at construction uses ``Right = Front x WorldUp`` and
``Down = Front x Right``.  This gives a proper right-handed FRD rotation in an
ENU world.  The superficially similar ``WorldUp x Front`` expression would
reverse the Right axis and produce a reflection for FRD, so it is deliberately
not used here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]
QuaternionXYZW = tuple[float, float, float, float]

WORLD_UP_ENU: Vector3 = (0.0, 0.0, 1.0)
OPTICAL_FORWARD_RDF: Vector3 = (0.0, 0.0, 1.0)
# Current Gazebo camera ``optical_link`` is authored with SDF link axes
# FLU (Forward, Left, Up).  This fixed nominal extrinsic maps RDF optical
# components into that link frame: link=[optical Forward, -Right, -Down].
GAZEBO_LINK_FLU_FROM_OPTICAL_RDF: Matrix3 = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
)
IDENTITY_ROTATION: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


def _finite_float(value: float, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def as_vector3(values: Sequence[float], name: str = "vector") -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    result = tuple(_finite_float(value, name) for value in values)
    return result  # type: ignore[return-value]


def dot(left: Sequence[float], right: Sequence[float]) -> float:
    if len(left) != 3 or len(right) != 3:
        raise ValueError("dot product requires two 3-vectors")
    return sum(float(left[index]) * float(right[index]) for index in range(3))


def cross(left: Sequence[float], right: Sequence[float]) -> Vector3:
    a = as_vector3(left, "left")
    b = as_vector3(right, "right")
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(values: Sequence[float]) -> float:
    vector = as_vector3(values)
    return math.sqrt(dot(vector, vector))


def normalize(values: Sequence[float], name: str = "vector") -> Vector3:
    vector = as_vector3(values, name)
    magnitude = math.sqrt(dot(vector, vector))
    if magnitude <= 0.0:
        raise ValueError(f"{name} must have non-zero length")
    return tuple(value / magnitude for value in vector)  # type: ignore[return-value]


def add(left: Sequence[float], right: Sequence[float]) -> Vector3:
    a = as_vector3(left, "left")
    b = as_vector3(right, "right")
    return tuple(a[index] + b[index] for index in range(3))  # type: ignore[return-value]


def subtract(left: Sequence[float], right: Sequence[float]) -> Vector3:
    a = as_vector3(left, "left")
    b = as_vector3(right, "right")
    return tuple(a[index] - b[index] for index in range(3))  # type: ignore[return-value]


def scale(vector: Sequence[float], scalar: float) -> Vector3:
    values = as_vector3(vector)
    factor = _finite_float(scalar, "scalar")
    return tuple(factor * value for value in values)  # type: ignore[return-value]


def as_matrix3(values: Sequence[Sequence[float]], name: str = "rotation") -> Matrix3:
    if len(values) != 3 or any(len(row) != 3 for row in values):
        raise ValueError(f"{name} must be a 3x3 matrix")
    parsed = tuple(
        tuple(_finite_float(value, name) for value in row) for row in values
    )
    return parsed  # type: ignore[return-value]


def transpose(matrix: Sequence[Sequence[float]]) -> Matrix3:
    parsed = as_matrix3(matrix)
    return tuple(
        tuple(parsed[row][column] for row in range(3)) for column in range(3)
    )  # type: ignore[return-value]


def matrix_vector(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> Vector3:
    parsed = as_matrix3(matrix)
    values = as_vector3(vector)
    return tuple(dot(parsed[row], values) for row in range(3))  # type: ignore[return-value]


def matrix_multiply(
    left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]
) -> Matrix3:
    a = as_matrix3(left, "left")
    b = as_matrix3(right, "right")
    b_columns = transpose(b)
    return tuple(
        tuple(dot(a[row], b_columns[column]) for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def determinant(matrix: Sequence[Sequence[float]]) -> float:
    a, b, c = as_matrix3(matrix)
    return (
        a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
    )


def validate_rotation_matrix(
    matrix: Sequence[Sequence[float]], *, tolerance: float = 1.0e-12
) -> dict[str, float]:
    """Validate orthogonality and proper handedness of one rotation matrix."""

    parsed = as_matrix3(matrix)
    tolerance = _finite_float(tolerance, "tolerance")
    if tolerance < 0.0:
        raise ValueError("tolerance must be non-negative")
    candidate = matrix_multiply(transpose(parsed), parsed)
    orthogonality_error = max(
        abs(candidate[row][column] - (1.0 if row == column else 0.0))
        for row in range(3)
        for column in range(3)
    )
    det = determinant(parsed)
    determinant_error = abs(det - 1.0)
    if orthogonality_error > tolerance:
        raise ValueError(
            f"rotation is not orthogonal: error={orthogonality_error:.3e}"
        )
    if determinant_error > tolerance:
        raise ValueError(f"rotation determinant must be +1: det={det:.15g}")
    return {
        "orthogonality_error": orthogonality_error,
        "determinant": det,
        "determinant_error": determinant_error,
    }


def quaternion_xyzw_to_matrix(quaternion: Sequence[float]) -> Matrix3:
    """Convert a ROS/Gazebo/SciPy ``(x, y, z, w)`` quaternion to ``R``."""

    if len(quaternion) != 4:
        raise ValueError("quaternion_xyzw must contain exactly four values")
    q = tuple(_finite_float(value, "quaternion") for value in quaternion)
    magnitude = math.sqrt(sum(value * value for value in q))
    if magnitude <= 0.0:
        raise ValueError("quaternion must have non-zero norm")
    if abs(magnitude - 1.0) > 1.0e-6:
        raise ValueError(
            f"quaternion_xyzw must be normalized at the API boundary: norm={magnitude:.9g}"
        )
    x, y, z, w = q
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def matrix_to_quaternion_xyzw(matrix: Sequence[Sequence[float]]) -> QuaternionXYZW:
    """Convert a proper rotation matrix to a normalized ROS-order quaternion."""

    rotation = as_matrix3(matrix)
    validate_rotation_matrix(rotation, tolerance=1.0e-9)
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
    return _normalize_quaternion_xyzw((x, y, z, w))


def _normalize_quaternion_xyzw(quaternion: Sequence[float]) -> QuaternionXYZW:
    if len(quaternion) != 4:
        raise ValueError("quaternion must contain exactly four values")
    q = tuple(_finite_float(value, "quaternion") for value in quaternion)
    magnitude = math.sqrt(sum(value * value for value in q))
    if magnitude <= 0.0:
        raise ValueError("quaternion must have non-zero norm")
    return tuple(value / magnitude for value in q)  # type: ignore[return-value]


def quaternion_slerp_xyzw(
    left: Sequence[float], right: Sequence[float], alpha: float
) -> QuaternionXYZW:
    """Shortest-path spherical interpolation of two ROS-order quaternions."""

    q0 = _normalize_quaternion_xyzw(left)
    q1 = _normalize_quaternion_xyzw(right)
    t = _finite_float(alpha, "alpha")
    if t < 0.0 or t > 1.0:
        raise ValueError("alpha must be in [0, 1]")
    cosine = sum(q0[index] * q1[index] for index in range(4))
    if cosine < 0.0:
        q1 = tuple(-value for value in q1)  # type: ignore[assignment]
        cosine = -cosine
    cosine = max(-1.0, min(1.0, cosine))
    if cosine > 1.0 - 1.0e-12:
        return _normalize_quaternion_xyzw(
            tuple(q0[index] + t * (q1[index] - q0[index]) for index in range(4))
        )
    angle = math.acos(cosine)
    sine = math.sin(angle)
    first = math.sin((1.0 - t) * angle) / sine
    second = math.sin(t * angle) / sine
    return _normalize_quaternion_xyzw(
        tuple(first * q0[index] + second * q1[index] for index in range(4))
    )


def angle_between(left: Sequence[float], right: Sequence[float]) -> float:
    a = normalize(left, "left")
    b = normalize(right, "right")
    return math.acos(max(-1.0, min(1.0, dot(a, b))))


@dataclass(frozen=True)
class Pose3D:
    """A position and rotation ``R_world_frame`` with explicit world frame."""

    position_world_enu_m: Vector3
    rotation_world_from_frame: Matrix3
    frame_id: str
    source_timestamp_ns: int | None = None


@dataclass(frozen=True)
class LookAtRotationResult:
    rotation_world_from_rig: Matrix3 | None
    front_world_enu: Vector3 | None
    right_world_enu: Vector3 | None
    down_world_enu: Vector3 | None
    status: str
    valid: bool


def compute_look_at_rig_rotation(
    rig_center_world_enu_m: Sequence[float],
    target_world_enu_m: Sequence[float],
    world_up_world_enu: Sequence[float] = WORLD_UP_ENU,
    previous_right_axis_world_enu: Sequence[float] | None = None,
    *,
    near_vertical_threshold: float = 1.0e-9,
    fallback_horizontal_axis_world_enu: Sequence[float] = (1.0, 0.0, 0.0),
) -> LookAtRotationResult:
    """Build a rigid parallel FRD rig aimed at one target point.

    The third column is physical Down in ENU, so the returned matrix is a
    proper ``R_W_R`` mapping ``baseline_center_frd`` components to world ENU.
    Near-vertical lines of sight use the projected previous Right axis when
    available, otherwise the configured horizontal fallback axis.
    """

    center = as_vector3(rig_center_world_enu_m, "rig_center_world_enu_m")
    target = as_vector3(target_world_enu_m, "target_world_enu_m")
    up = normalize(world_up_world_enu, "world_up_world_enu")
    threshold = _finite_float(near_vertical_threshold, "near_vertical_threshold")
    if threshold < 0.0:
        raise ValueError("near_vertical_threshold must be non-negative")
    front_delta = subtract(target, center)
    front_length = norm(front_delta)
    if front_length <= 0.0:
        return LookAtRotationResult(None, None, None, None, "INVALID_ZERO_LOS", False)
    front = scale(front_delta, 1.0 / front_length)
    right_candidate = cross(front, up)
    right_length = norm(right_candidate)
    status = "VALID"
    if right_length <= threshold:
        candidates: list[tuple[str, Sequence[float]]] = []
        if previous_right_axis_world_enu is not None:
            candidates.append(("PREVIOUS_RIGHT", previous_right_axis_world_enu))
        candidates.append(("FALLBACK_HORIZONTAL_AXIS", fallback_horizontal_axis_world_enu))
        candidates.extend(
            (
                ("FALLBACK_EAST_AXIS", (1.0, 0.0, 0.0)),
                ("FALLBACK_NORTH_AXIS", (0.0, 1.0, 0.0)),
            )
        )
        selected: Vector3 | None = None
        selected_name = ""
        for name, candidate in candidates:
            try:
                candidate_vector = normalize(candidate, name)
            except ValueError:
                continue
            projection = subtract(candidate_vector, scale(front, dot(candidate_vector, front)))
            if norm(projection) > threshold:
                selected = normalize(projection, "projected_right")
                selected_name = name
                break
        if selected is None:
            return LookAtRotationResult(
                None, front, None, None, "INVALID_NEAR_VERTICAL_ROLL_REFERENCE", False
            )
        right = selected
        status = f"NEAR_VERTICAL_{selected_name}"
    else:
        right = normalize(right_candidate, "right_world_enu")
    down = normalize(cross(front, right), "down_world_enu")
    rotation = tuple(
        tuple(axis[column] for axis in (front, right, down)) for column in range(3)
    )  # type: ignore[assignment]
    validate_rotation_matrix(rotation)
    return LookAtRotationResult(rotation, front, right, down, status, True)


def derive_camera_optical_pose(
    rig_position_world_enu_m: Sequence[float],
    rig_rotation_world_from_rig: Sequence[Sequence[float]],
    camera_translation_rig_m: Sequence[float],
    camera_rotation_rig_from_optical: Sequence[Sequence[float]],
    *,
    source_timestamp_ns: int | None = None,
) -> Pose3D:
    """Derive one RDF optical pose from ``R_W_R`` and nominal rig extrinsics."""

    position = add(
        rig_position_world_enu_m,
        matrix_vector(rig_rotation_world_from_rig, camera_translation_rig_m),
    )
    rotation = matrix_multiply(
        rig_rotation_world_from_rig, camera_rotation_rig_from_optical
    )
    validate_rotation_matrix(rotation)
    return Pose3D(
        position_world_enu_m=position,
        rotation_world_from_frame=rotation,
        frame_id="camera_optical_rdf",
        source_timestamp_ns=source_timestamp_ns,
    )


def rotation_to_euler_zyx_rad(rotation: Sequence[Sequence[float]]) -> Vector3:
    """Return ``(roll, pitch, yaw)`` for ``R = Rz(yaw) Ry(pitch) Rx(roll)``.

    This is the fixed extrinsic Z-Y-X convention, equivalently intrinsic X-Y-Z
    in the named frame.  Axes are therefore ``+X, +Y, +Z`` of the compared
    frame; the result is for reporting only, never for internal composition.
    Output ranges are roll/yaw ``[-pi, pi]`` and pitch ``[-pi/2, pi/2]``.
    """

    r = as_matrix3(rotation)
    validate_rotation_matrix(r, tolerance=1.0e-9)
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    cosine_pitch = math.cos(pitch)
    if abs(cosine_pitch) > 1.0e-10:
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-r[0][1], r[1][1])
    return roll, pitch, yaw


@dataclass(frozen=True)
class RotationError:
    relative_rotation: Matrix3
    roll_error_deg: float
    pitch_error_deg: float
    yaw_error_deg: float
    rotation_error_deg: float
    valid: bool = True


def rotation_error_angle_rad(rotation: Sequence[Sequence[float]]) -> float:
    r = as_matrix3(rotation)
    validate_rotation_matrix(r, tolerance=1.0e-9)
    value = (r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, value)))


def compute_rotation_error(
    ideal_rotation_world_from_frame: Sequence[Sequence[float]],
    actual_rotation_world_from_frame: Sequence[Sequence[float]],
) -> RotationError:
    """Compute ``R_error = R_ideal.T @ R_actual`` and reporting angles."""

    ideal = as_matrix3(ideal_rotation_world_from_frame, "ideal_rotation")
    actual = as_matrix3(actual_rotation_world_from_frame, "actual_rotation")
    validate_rotation_matrix(ideal)
    validate_rotation_matrix(actual)
    relative = matrix_multiply(transpose(ideal), actual)
    roll, pitch, yaw = rotation_to_euler_zyx_rad(relative)
    return RotationError(
        relative_rotation=relative,
        roll_error_deg=math.degrees(roll),
        pitch_error_deg=math.degrees(pitch),
        yaw_error_deg=math.degrees(yaw),
        rotation_error_deg=math.degrees(rotation_error_angle_rad(relative)),
    )


@dataclass(frozen=True)
class CameraPoseError:
    rotation: RotationError
    position_error_world_enu_m: Vector3
    position_error_norm_m: float
    optical_axis_to_target_error_deg: float | None
    valid: bool


def compute_optical_axis_target_error(
    camera_position_world_enu_m: Sequence[float],
    camera_rotation_world_from_optical: Sequence[Sequence[float]],
    target_position_world_enu_m: Sequence[float],
) -> float:
    forward_world = matrix_vector(camera_rotation_world_from_optical, OPTICAL_FORWARD_RDF)
    target_ray = subtract(target_position_world_enu_m, camera_position_world_enu_m)
    return math.degrees(angle_between(forward_world, target_ray))


def compute_camera_pose_error(
    ideal_pose: Pose3D,
    actual_pose: Pose3D,
    target_position_world_enu_m: Sequence[float],
) -> CameraPoseError:
    rotation = compute_rotation_error(
        ideal_pose.rotation_world_from_frame,
        actual_pose.rotation_world_from_frame,
    )
    try:
        optical_axis_error = compute_optical_axis_target_error(
            actual_pose.position_world_enu_m,
            actual_pose.rotation_world_from_frame,
            target_position_world_enu_m,
        )
    except ValueError:
        optical_axis_error = None
    position_error = subtract(
        actual_pose.position_world_enu_m,
        ideal_pose.position_world_enu_m,
    )
    return CameraPoseError(
        rotation,
        position_error,
        norm(position_error),
        optical_axis_error,
        rotation.valid and optical_axis_error is not None,
    )


@dataclass(frozen=True)
class StereoRelativePoseError:
    actual_relative_rotation: Matrix3
    nominal_relative_rotation: Matrix3
    roll_error_deg: float
    pitch_error_deg: float
    yaw_error_deg: float
    rotation_error_deg: float
    optical_axis_angle_deg: float | None
    valid: bool


def compute_relative_camera_pose_error(
    left_actual_rotation_world_from_optical: Sequence[Sequence[float]],
    right_actual_rotation_world_from_optical: Sequence[Sequence[float]],
    nominal_left_rotation_rig_from_optical: Sequence[Sequence[float]],
    nominal_right_rotation_rig_from_optical: Sequence[Sequence[float]],
) -> StereoRelativePoseError:
    left_actual = as_matrix3(left_actual_rotation_world_from_optical)
    right_actual = as_matrix3(right_actual_rotation_world_from_optical)
    left_nominal = as_matrix3(nominal_left_rotation_rig_from_optical)
    right_nominal = as_matrix3(nominal_right_rotation_rig_from_optical)
    validate_rotation_matrix(left_actual)
    validate_rotation_matrix(right_actual)
    validate_rotation_matrix(left_nominal)
    validate_rotation_matrix(right_nominal)
    actual_relative = matrix_multiply(transpose(left_actual), right_actual)
    nominal_relative = matrix_multiply(transpose(left_nominal), right_nominal)
    error = compute_rotation_error(nominal_relative, actual_relative)
    left_forward = matrix_vector(left_actual, OPTICAL_FORWARD_RDF)
    right_forward = matrix_vector(right_actual, OPTICAL_FORWARD_RDF)
    optical_axis_angle = math.degrees(angle_between(left_forward, right_forward))
    return StereoRelativePoseError(
        actual_relative_rotation=actual_relative,
        nominal_relative_rotation=nominal_relative,
        roll_error_deg=error.roll_error_deg,
        pitch_error_deg=error.pitch_error_deg,
        yaw_error_deg=error.yaw_error_deg,
        rotation_error_deg=error.rotation_error_deg,
        optical_axis_angle_deg=optical_axis_angle,
        valid=True,
    )


@dataclass(frozen=True)
class BaselineGeometryResult:
    vector_world_enu_m: Vector3
    actual_length_m: float
    nominal_length_m: float
    length_error_m: float
    direction_error_deg: float | None
    midpoint_world_enu_m: Vector3
    midpoint_error_m: float | None
    valid: bool


def compute_baseline_geometry(
    left_actual_position_world_enu_m: Sequence[float],
    right_actual_position_world_enu_m: Sequence[float],
    nominal_baseline_vector_rig_m: Sequence[float],
    ideal_rotation_world_from_rig: Sequence[Sequence[float]],
    *,
    nominal_midpoint_world_enu_m: Sequence[float] | None = None,
) -> BaselineGeometryResult:
    actual_vector = subtract(right_actual_position_world_enu_m, left_actual_position_world_enu_m)
    actual_length = norm(actual_vector)
    nominal_vector = as_vector3(nominal_baseline_vector_rig_m, "nominal_baseline_vector_rig_m")
    nominal_length = norm(nominal_vector)
    ideal_vector = matrix_vector(ideal_rotation_world_from_rig, nominal_vector)
    direction_error: float | None
    if actual_length <= 0.0 or norm(ideal_vector) <= 0.0:
        direction_error = None
    else:
        direction_error = math.degrees(angle_between(actual_vector, ideal_vector))
    midpoint = scale(add(left_actual_position_world_enu_m, right_actual_position_world_enu_m), 0.5)
    midpoint_error: float | None = None
    if nominal_midpoint_world_enu_m is not None:
        midpoint_error = norm(subtract(midpoint, nominal_midpoint_world_enu_m))
    return BaselineGeometryResult(
        vector_world_enu_m=actual_vector,
        actual_length_m=actual_length,
        nominal_length_m=nominal_length,
        length_error_m=actual_length - nominal_length,
        direction_error_deg=direction_error,
        midpoint_world_enu_m=midpoint,
        midpoint_error_m=midpoint_error,
        valid=actual_length > 0.0 and nominal_length > 0.0 and direction_error is not None,
    )


def interpolate_pose(left: Pose3D, right: Pose3D, alpha: float) -> Pose3D:
    """Interpolate position linearly and orientation with shortest-path SLERP."""

    t = _finite_float(alpha, "alpha")
    if t < 0.0 or t > 1.0:
        raise ValueError("alpha must be in [0, 1]")
    if left.source_timestamp_ns is None or right.source_timestamp_ns is None:
        timestamp = None
    else:
        timestamp = int(round(left.source_timestamp_ns + t * (right.source_timestamp_ns - left.source_timestamp_ns)))
    q = quaternion_slerp_xyzw(
        matrix_to_quaternion_xyzw(left.rotation_world_from_frame),
        matrix_to_quaternion_xyzw(right.rotation_world_from_frame),
        t,
    )
    return Pose3D(
        position_world_enu_m=tuple(
            left.position_world_enu_m[index]
            + t * (right.position_world_enu_m[index] - left.position_world_enu_m[index])
            for index in range(3)
        ),  # type: ignore[arg-type]
        rotation_world_from_frame=quaternion_xyzw_to_matrix(q),
        frame_id=left.frame_id,
        source_timestamp_ns=timestamp,
    )


__all__ = [
    "BaselineGeometryResult",
    "CameraPoseError",
    "IDENTITY_ROTATION",
    "GAZEBO_LINK_FLU_FROM_OPTICAL_RDF",
    "LookAtRotationResult",
    "Matrix3",
    "OPTICAL_FORWARD_RDF",
    "Pose3D",
    "QuaternionXYZW",
    "RotationError",
    "StereoRelativePoseError",
    "Vector3",
    "WORLD_UP_ENU",
    "add",
    "angle_between",
    "as_matrix3",
    "as_vector3",
    "compute_baseline_geometry",
    "compute_camera_pose_error",
    "compute_look_at_rig_rotation",
    "compute_optical_axis_target_error",
    "compute_relative_camera_pose_error",
    "compute_rotation_error",
    "cross",
    "derive_camera_optical_pose",
    "determinant",
    "dot",
    "interpolate_pose",
    "matrix_multiply",
    "matrix_to_quaternion_xyzw",
    "matrix_vector",
    "norm",
    "normalize",
    "quaternion_slerp_xyzw",
    "quaternion_xyzw_to_matrix",
    "rotation_error_angle_rad",
    "rotation_to_euler_zyx_rad",
    "scale",
    "subtract",
    "transpose",
    "validate_rotation_matrix",
]
