"""Explicit coordinate contracts for the Simulation stereo rig.

The simulation world is Gazebo ENU.  Camera observations are OpenCV optical
RDF (Right, Down, Forward), while public positions are expressed in the
fixed ``baseline_center_frd`` frame (Forward, Right, Down).  No axis is
aliased by name: the camera poses below provide the world directions used by
the transform.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]

# The generated stereo-camera SDF fixes ``optical_link`` to ``base_link`` at
# identity pose.  Its optical RDF axes are link (-Y, -Z, +X); PX4 body FRD
# axes are the same physical (Forward, Right, Down) directions.  Therefore
# this maps optical [Right, Down, Forward] components to PX4 body [F,R,D].
PX4_BODY_FRD_FROM_OPTICAL_RDF: Matrix3 = (
    (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)
R_ENU_FROM_NED: Matrix3 = (
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
)

WORLD_FRAME = "gazebo_world_enu"
BASELINE_CENTER_FRD_FRAME = "baseline_center_frd"
VEHICLE_REFERENCE_LINK = "vehicle_reference_link"
VEHICLE_REFERENCE_DEFINITION = "project_defined_vehicle_reference_at_base_link_origin"
SOURCE_GAZEBO_ENTITY = "base_link"
REFERENCE_OFFSET_PARENT_M: Vector3 = (0.0, 0.0, 0.0)
REFERENCE_OFFSET_PARENT_RPY_RAD: Vector3 = (0.0, 0.0, 0.0)


def _finite_vector(values: Sequence[float], name: str) -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{name} must contain three values")
    parsed = (float(values[0]), float(values[1]), float(values[2]))
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vector3, scalar: float) -> Vector3:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def _cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: Vector3) -> float:
    return math.sqrt(_dot(a, a))


def normalize(values: Sequence[float], name: str = "vector") -> Vector3:
    parsed = _finite_vector(values, name)
    magnitude = _norm(parsed)
    if magnitude <= 0.0:
        raise ValueError(f"{name} must have non-zero length")
    return _scale(parsed, 1.0 / magnitude)


def _mat_vec(matrix: Matrix3, vector: Vector3) -> Vector3:
    return (_dot(matrix[0], vector), _dot(matrix[1], vector), _dot(matrix[2], vector))


def _mat_mul(left: Matrix3, right: Matrix3) -> Matrix3:
    right_columns = _transpose(right)
    return tuple(
        tuple(_dot(left[row], right_columns[column]) for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _transpose(matrix: Matrix3) -> Matrix3:
    return tuple(tuple(matrix[row][column] for row in range(3)) for column in range(3))  # type: ignore[return-value]


def _det(matrix: Matrix3) -> float:
    a, b, c = matrix
    return (
        a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
    )


def validate_rotation_matrix(
    matrix: Sequence[Sequence[float]], *, tolerance: float = 1.0e-12
) -> dict[str, float]:
    """Validate one explicit 3-D frame rotation and return audit metrics.

    The matrix follows the module convention ``R_B_A``: it maps components
    expressed in frame A into frame B.  Reflections are rejected even when
    they are otherwise orthogonal.
    """

    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("rotation matrix must be 3x3")
    parsed: Matrix3 = tuple(
        tuple(float(value) for value in row) for row in matrix
    )  # type: ignore[assignment]
    if not all(math.isfinite(value) for row in parsed for value in row):
        raise ValueError("rotation matrix must be finite")
    tolerance = float(tolerance)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("rotation tolerance must be finite and non-negative")
    identity_candidate = _mat_mul(_transpose(parsed), parsed)
    orthogonality_error = max(
        abs(identity_candidate[row][column] - (1.0 if row == column else 0.0))
        for row in range(3)
        for column in range(3)
    )
    determinant = _det(parsed)
    determinant_error = abs(determinant - 1.0)
    if orthogonality_error > tolerance:
        raise ValueError("rotation matrix must be orthogonal")
    if determinant_error > tolerance:
        raise ValueError("rotation matrix must have determinant +1")
    return {
        "orthogonality_error": orthogonality_error,
        "determinant": determinant,
        "determinant_error": determinant_error,
    }


@dataclass(frozen=True)
class BaselineCenterFRD:
    """The immutable nominal rig frame used for one simulation run."""

    origin_world_enu_m: Vector3
    rotation_baseline_from_world: Matrix3
    left_center_world_enu_m: Vector3
    right_center_world_enu_m: Vector3
    left_forward_world_enu: Vector3
    right_forward_world_enu: Vector3

    @property
    def frame_id(self) -> str:
        return BASELINE_CENTER_FRD_FRAME

    def point_from_world(self, point_world_enu_m: Sequence[float]) -> Vector3:
        point = _finite_vector(point_world_enu_m, "point_world_enu_m")
        return _mat_vec(self.rotation_baseline_from_world, _sub(point, self.origin_world_enu_m))

    def ray_from_world(self, ray_world_enu: Sequence[float]) -> Vector3:
        return _mat_vec(self.rotation_baseline_from_world, _finite_vector(ray_world_enu, "ray_world_enu"))

    def point_to_world(self, point_baseline_frd_m: Sequence[float]) -> Vector3:
        point = _finite_vector(point_baseline_frd_m, "point_baseline_frd_m")
        return _add(_mat_vec(_transpose(self.rotation_baseline_from_world), point), self.origin_world_enu_m)


@dataclass(frozen=True)
class CameraOpticalPose:
    """Nominal optical pose with RDF axes represented in world ENU."""

    center_world_enu_m: Vector3
    right_world_enu: Vector3
    down_world_enu: Vector3
    forward_world_enu: Vector3

    def ray_to_world(self, ray_rdf: Sequence[float]) -> Vector3:
        ray = _finite_vector(ray_rdf, "ray_rdf")
        return normalize(
            _add(
                _add(_scale(self.right_world_enu, ray[0]), _scale(self.down_world_enu, ray[1])),
                _scale(self.forward_world_enu, ray[2]),
            ),
            "ray_world_enu",
        )

    def world_to_rdf(self, point_world_enu_m: Sequence[float]) -> Vector3:
        delta = _sub(_finite_vector(point_world_enu_m, "point_world_enu_m"), self.center_world_enu_m)
        return (_dot(self.right_world_enu, delta), _dot(self.down_world_enu, delta), _dot(self.forward_world_enu, delta))


def _validated_camera_basis(camera_pose: CameraOpticalPose) -> Matrix3:
    right = _finite_vector(camera_pose.right_world_enu, "right_world_enu")
    down = _finite_vector(camera_pose.down_world_enu, "down_world_enu")
    forward = _finite_vector(camera_pose.forward_world_enu, "forward_world_enu")
    basis_columns = (right, down, forward)
    if any(abs(_norm(axis) - 1.0) > 1.0e-12 for axis in basis_columns):
        raise ValueError("camera optical basis axes must have unit norm")
    if any(
        abs(_dot(basis_columns[first], basis_columns[second])) > 1.0e-12
        for first, second in ((0, 1), (0, 2), (1, 2))
    ):
        raise ValueError("camera optical basis axes must be orthogonal")
    basis_rows = _transpose(basis_columns)
    if abs(_det(basis_rows) - 1.0) > 1.0e-12:
        raise ValueError("camera optical basis must be right-handed")
    return basis_rows


def optical_rpy_error_rotation(
    roll_error_rad: float,
    pitch_error_rad: float,
    yaw_error_rad: float,
) -> Matrix3:
    """Return ``R_Cnominal_Cactual`` for local optical RDF attitude error.

    Optical axes are Right, Down, Forward. Positive roll is about +Forward,
    positive pitch is about +Right, and positive yaw is about +Down. The
    composition is ``R_Down(yaw) @ R_Right(pitch) @ R_Forward(roll)``;
    column-vector application order is roll, then pitch, then yaw.
    """

    roll, pitch, yaw = _finite_vector(
        (roll_error_rad, pitch_error_rad, yaw_error_rad),
        "optical_rpy_error_rad",
    )
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation_forward: Matrix3 = (
        (cr, -sr, 0.0),
        (sr, cr, 0.0),
        (0.0, 0.0, 1.0),
    )
    rotation_right: Matrix3 = (
        (1.0, 0.0, 0.0),
        (0.0, cp, -sp),
        (0.0, sp, cp),
    )
    rotation_down: Matrix3 = (
        (cy, 0.0, sy),
        (0.0, 1.0, 0.0),
        (-sy, 0.0, cy),
    )
    rotation = _mat_mul(rotation_down, _mat_mul(rotation_right, rotation_forward))
    if abs(_det(rotation) - 1.0) > 1.0e-12:
        raise ValueError("optical RPY rotation must have determinant +1")
    return rotation


def apply_optical_rpy_error(
    nominal_camera_pose: CameraOpticalPose,
    roll_error_rad: float,
    pitch_error_rad: float,
    yaw_error_rad: float,
) -> CameraOpticalPose:
    """Apply a local optical RDF RPY error without moving the camera center."""

    nominal_basis = _validated_camera_basis(nominal_camera_pose)
    rotation_nominal_from_actual = optical_rpy_error_rotation(
        roll_error_rad,
        pitch_error_rad,
        yaw_error_rad,
    )
    actual_basis = _mat_mul(nominal_basis, rotation_nominal_from_actual)
    actual = CameraOpticalPose(
        center_world_enu_m=_finite_vector(
            nominal_camera_pose.center_world_enu_m,
            "center_world_enu_m",
        ),
        right_world_enu=tuple(row[0] for row in actual_basis),  # type: ignore[arg-type]
        down_world_enu=tuple(row[1] for row in actual_basis),  # type: ignore[arg-type]
        forward_world_enu=tuple(row[2] for row in actual_basis),  # type: ignore[arg-type]
    )
    _validated_camera_basis(actual)
    return actual


def apply_optical_yaw(
    nominal_camera_pose: CameraOpticalPose,
    yaw_error_rad: float,
) -> CameraOpticalPose:
    """Rotate an optical RDF pose about +Down using the right-hand rule.

    Positive yaw rotates optical Forward toward optical Right. The camera
    center and Down axis are unchanged.
    """

    return apply_optical_rpy_error(nominal_camera_pose, 0.0, 0.0, yaw_error_rad)


def effective_optical_rpy_errors_deg(
    left_direct_rpy_deg: Sequence[float],
    right_direct_rpy_deg: Sequence[float],
    common_rpy_deg: Sequence[float],
    relative_rpy_deg: Sequence[float],
) -> tuple[Vector3, Vector3]:
    """Combine direct and common/relative optical RDF RPY components.

    Each input and output is ``(roll, pitch, yaw)`` in degrees. For every
    component, left is ``direct + common - relative/2`` and right is
    ``direct + common + relative/2``.
    """

    left_direct = _finite_vector(left_direct_rpy_deg, "left_direct_rpy_deg")
    right_direct = _finite_vector(right_direct_rpy_deg, "right_direct_rpy_deg")
    common = _finite_vector(common_rpy_deg, "common_rpy_deg")
    relative = _finite_vector(relative_rpy_deg, "relative_rpy_deg")
    left = tuple(
        left_direct[index] + common[index] - relative[index] * 0.5
        for index in range(3)
    )
    right = tuple(
        right_direct[index] + common[index] + relative[index] * 0.5
        for index in range(3)
    )
    return left, right  # type: ignore[return-value]


def effective_optical_yaws_deg(
    common_optical_yaw_error_deg: float,
    relative_optical_yaw_error_deg: float,
) -> tuple[float, float]:
    """Return effective left/right optical yaw errors in degrees."""

    left, right = effective_optical_rpy_errors_deg(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, common_optical_yaw_error_deg),
        (0.0, 0.0, relative_optical_yaw_error_deg),
    )
    return left[2], right[2]


def sdf_link_rpy_from_optical_error(
    nominal_link_yaw_rad: float,
    optical_roll_error_rad: float,
    optical_pitch_error_rad: float,
    optical_yaw_error_rad: float,
) -> Vector3:
    """Map optical RDF error RPY to this world's generated SDF link RPY.

    Link +X is Optical +Forward, link +Y is Optical -Right, and link +Z is
    Optical -Down. Therefore link RPY is ``(roll, -pitch, nominal_yaw-yaw)``.
    This is equivalent to the full optical composition for composite RPY.
    """

    nominal_yaw, roll, pitch, yaw = (
        float(nominal_link_yaw_rad),
        float(optical_roll_error_rad),
        float(optical_pitch_error_rad),
        float(optical_yaw_error_rad),
    )
    if not all(math.isfinite(value) for value in (nominal_yaw, roll, pitch, yaw)):
        raise ValueError("SDF link and optical RPY values must be finite")
    return roll, -pitch, nominal_yaw - yaw


def sdf_link_rpy_from_px4_frd_error(
    nominal_link_yaw_rad: float,
    px4_roll_error_rad: float,
    px4_pitch_error_rad: float,
    px4_yaw_error_rad: float,
) -> Vector3:
    """Map a PX4 FRD body attitude error to Gazebo's FLU link RPY.

    PX4 body +X/+Y/+Z are Forward/Right/Down while the Gazebo model link is
    Forward/Left/Up.  The equivalent SDF error is therefore
    ``(roll, -pitch, -yaw)`` about the nominal model heading.
    """

    nominal_yaw, roll, pitch, yaw = (
        float(nominal_link_yaw_rad),
        float(px4_roll_error_rad),
        float(px4_pitch_error_rad),
        float(px4_yaw_error_rad),
    )
    if not all(math.isfinite(value) for value in (nominal_yaw, roll, pitch, yaw)):
        raise ValueError("SDF link and PX4 FRD RPY values must be finite")
    return roll, -pitch, nominal_yaw - yaw


def rpy_to_rotation_world_from_link(roll_rad: float, pitch_rad: float, yaw_rad: float) -> Matrix3:
    """Return the SDF ``roll pitch yaw`` rotation, link frame to world."""

    roll = float(roll_rad)
    pitch = float(pitch_rad)
    yaw = float(yaw_rad)
    if not all(math.isfinite(value) for value in (roll, pitch, yaw)):
        raise ValueError("rpy must be finite")
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def rotation_to_sdf_rpy(rotation: Matrix3) -> Vector3:
    """Return SDF ZYX roll/pitch/yaw for a proper rotation matrix."""

    if not all(math.isfinite(value) for row in rotation for value in row):
        raise ValueError("rotation must be finite")
    if abs(_det(rotation) - 1.0) > 1.0e-9:
        raise ValueError("rotation must have determinant +1")
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if abs(math.cos(pitch)) > 1.0e-12:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def rotation_world_from_quaternion_xyzw(quaternion_xyzw: Sequence[float]) -> Matrix3:
    """Return a world-from-link rotation from a finite, non-zero XYZW quaternion."""

    if len(quaternion_xyzw) != 4:
        raise ValueError("quaternion_xyzw must contain four values")
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ValueError("quaternion_xyzw must be finite")
    magnitude = math.sqrt(x * x + y * y + z * z + w * w)
    if magnitude <= 0.0:
        raise ValueError("quaternion_xyzw must have non-zero length")
    x, y, z, w = (value / magnitude for value in (x, y, z, w))
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def relative_sdf_rpy(
    parent_world_rpy_rad: Sequence[float],
    child_world_rpy_rad: Sequence[float],
) -> Vector3:
    """Return child RPY relative to parent while preserving its world attitude."""

    parent = _finite_vector(parent_world_rpy_rad, "parent_world_rpy_rad")
    child = _finite_vector(child_world_rpy_rad, "child_world_rpy_rad")
    rotation_world_from_parent = rpy_to_rotation_world_from_link(*parent)
    rotation_world_from_child = rpy_to_rotation_world_from_link(*child)
    rotation_parent_from_child = _mat_mul(
        _transpose(rotation_world_from_parent),
        rotation_world_from_child,
    )
    return rotation_to_sdf_rpy(rotation_parent_from_child)


def sdf_pose_rpy_from_camera_pose(camera_pose: "CameraOpticalPose") -> Vector3:
    """Inverse of ``camera_optical_pose_from_sdf_pose``.

    Given a camera's Right/Down/Forward world-ENU basis, return the SDF world
    ``roll pitch yaw`` for a link whose local ``+X, +Y, +Z`` are Optical
    ``Forward, -Right, -Down`` (the same convention already used for
    ``optical_link`` and, by the fixed zero-relative-pose mount, for
    ``base_link``). Round-trips exactly through
    ``camera_optical_pose_from_sdf_pose``.
    """

    forward = _finite_vector(camera_pose.forward_world_enu, "forward_world_enu")
    right = _finite_vector(camera_pose.right_world_enu, "right_world_enu")
    down = _finite_vector(camera_pose.down_world_enu, "down_world_enu")
    rotation = _transpose((forward, _scale(right, -1.0), _scale(down, -1.0)))
    return rotation_to_sdf_rpy(rotation)


def relative_rotation_angle_deg(pose_a: "CameraOpticalPose", pose_b: "CameraOpticalPose") -> float:
    """Return the angular magnitude, in degrees, of the rotation relating two
    optical bases (e.g. a PX4-reported reconstruction versus the Gazebo
    actual/true optical pose for the same camera).

    Uses the full 3-DOF relative rotation (``arccos((trace(R)-1)/2)``), not a
    per-axis Euler difference, so a pure-roll or pure-yaw residual is not
    missed by comparing only the Forward axis.
    """

    basis_a = _validated_camera_basis(pose_a)
    basis_b = _validated_camera_basis(pose_b)
    relative = _mat_mul(_transpose(basis_a), basis_b)
    trace = relative[0][0] + relative[1][1] + relative[2][2]
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cos_angle))


def camera_optical_pose_from_sdf_pose(
    center_world_enu_m: Sequence[float],
    rpy_rad: Sequence[float],
) -> CameraOpticalPose:
    """Map an SDF camera link pose to the optical RDF axes.

    The SDF camera optical convention used by this world is Forward=link X,
    Right=-link Y, Down=-link Z.  The mapping is applied through the supplied
    rotation matrix, so it remains valid for arbitrary rig yaw/translation.
    """

    center = _finite_vector(center_world_enu_m, "center_world_enu_m")
    rpy = _finite_vector(rpy_rad, "rpy_rad")
    rotation = rpy_to_rotation_world_from_link(*rpy)
    forward = _mat_vec(rotation, (1.0, 0.0, 0.0))
    right = _mat_vec(rotation, (0.0, -1.0, 0.0))
    down = _mat_vec(rotation, (0.0, 0.0, -1.0))
    return CameraOpticalPose(
        center_world_enu_m=center,
        right_world_enu=normalize(right, "right_world_enu"),
        down_world_enu=normalize(down, "down_world_enu"),
        forward_world_enu=normalize(forward, "forward_world_enu"),
    )


def camera_optical_pose_from_quaternion(
    center_world_enu_m: Sequence[float],
    orientation_world_from_link_xyzw: Sequence[float],
) -> CameraOpticalPose:
    """Map a Gazebo world-from-link quaternion to optical RDF axes."""

    center = _finite_vector(center_world_enu_m, "center_world_enu_m")
    rotation = rotation_world_from_quaternion_xyzw(orientation_world_from_link_xyzw)
    forward = _mat_vec(rotation, (1.0, 0.0, 0.0))
    right = _mat_vec(rotation, (0.0, -1.0, 0.0))
    down = _mat_vec(rotation, (0.0, 0.0, -1.0))
    return CameraOpticalPose(
        center_world_enu_m=center,
        right_world_enu=normalize(right, "right_world_enu"),
        down_world_enu=normalize(down, "down_world_enu"),
        forward_world_enu=normalize(forward, "forward_world_enu"),
    )


def camera_optical_pose_from_px4_attitude_wxyz(
    center_world_enu_m: Sequence[float],
    quaternion_wxyz: Sequence[float],
    *,
    rotation_body_from_optical: Sequence[Sequence[float]] = PX4_BODY_FRD_FROM_OPTICAL_RDF,
) -> CameraOpticalPose:
    """Build World-ENU optical RDF pose from PX4 FRD-to-NED attitude.

    ``quaternion_wxyz`` is PX4 VehicleAttitude.q, explicitly converted to the
    XYZW convention used by the existing quaternion rotation helper.
    """

    if len(quaternion_wxyz) != 4:
        raise ValueError("quaternion_wxyz must contain four values")
    w, x, y, z = (float(value) for value in quaternion_wxyz)
    validate_rotation_matrix(rotation_body_from_optical)
    parsed_body_from_optical: Matrix3 = tuple(
        tuple(float(value) for value in row) for row in rotation_body_from_optical
    )  # type: ignore[assignment]
    rotation_ned_from_body = rotation_world_from_quaternion_xyzw((x, y, z, w))
    rotation_enu_from_optical = _mat_mul(
        _mat_mul(R_ENU_FROM_NED, rotation_ned_from_body),
        parsed_body_from_optical,
    )
    return CameraOpticalPose(
        center_world_enu_m=_finite_vector(center_world_enu_m, "center_world_enu_m"),
        right_world_enu=normalize(_mat_vec(rotation_enu_from_optical, (1.0, 0.0, 0.0)), "right_world_enu"),
        down_world_enu=normalize(_mat_vec(rotation_enu_from_optical, (0.0, 1.0, 0.0)), "down_world_enu"),
        forward_world_enu=normalize(_mat_vec(rotation_enu_from_optical, (0.0, 0.0, 1.0)), "forward_world_enu"),
    )


def build_baseline_center_frd(
    left_center_world_enu_m: Sequence[float],
    right_center_world_enu_m: Sequence[float],
    left_forward_world_enu: Sequence[float],
    right_forward_world_enu: Sequence[float],
) -> BaselineCenterFRD:
    """Construct the fixed baseline-center FRD frame from nominal optical poses."""

    left_center = _finite_vector(left_center_world_enu_m, "left_center_world_enu_m")
    right_center = _finite_vector(right_center_world_enu_m, "right_center_world_enu_m")
    left_forward = normalize(left_forward_world_enu, "left_forward_world_enu")
    right_forward = normalize(right_forward_world_enu, "right_forward_world_enu")
    origin = _scale(_add(left_center, right_center), 0.5)
    right_axis = normalize(_sub(right_center, left_center), "baseline_right_world_enu")
    forward_candidate = normalize(_add(left_forward, right_forward), "forward_candidate_world_enu")
    forward = normalize(
        _sub(forward_candidate, _scale(right_axis, _dot(right_axis, forward_candidate))),
        "baseline_forward_world_enu",
    )
    down = normalize(_cross(forward, right_axis), "baseline_down_world_enu")
    rotation = (forward, right_axis, down)
    if abs(_det(rotation) - 1.0) > 1.0e-12:
        raise ValueError("baseline frame is not right-handed")
    return BaselineCenterFRD(
        origin_world_enu_m=origin,
        rotation_baseline_from_world=rotation,
        left_center_world_enu_m=left_center,
        right_center_world_enu_m=right_center,
        left_forward_world_enu=left_forward,
        right_forward_world_enu=right_forward,
    )


def nominal_camera_poses(baseline_m: float, camera_height_m: float) -> tuple[CameraOpticalPose, CameraOpticalPose]:
    """Return the nominal zero-error poses encoded by the generated world."""

    baseline = float(baseline_m)
    height = float(camera_height_m)
    if not math.isfinite(baseline) or baseline <= 0.0:
        raise ValueError("baseline_m must be positive and finite")
    if not math.isfinite(height):
        raise ValueError("camera_height_m must be finite")
    yaw = math.pi / 2.0
    left = camera_optical_pose_from_sdf_pose((-baseline * 0.5, 0.0, height), (0.0, 0.0, yaw))
    right = camera_optical_pose_from_sdf_pose((baseline * 0.5, 0.0, height), (0.0, 0.0, yaw))
    return left, right


def project_world_point_to_pixel(
    point_world_enu_m: Sequence[float],
    camera: CameraOpticalPose,
    *,
    fx_px: float,
    fy_px: float,
    cx_px: float,
    cy_px: float,
) -> tuple[float, float] | None:
    """Project a world point through a nominal optical RDF camera."""

    values = (float(fx_px), float(fy_px), float(cx_px), float(cy_px))
    if not all(math.isfinite(value) for value in values) or fx_px <= 0.0 or fy_px <= 0.0:
        return None
    right, down, forward = camera.world_to_rdf(point_world_enu_m)
    if not math.isfinite(forward) or forward <= 0.0:
        return None
    u = float(fx_px) * right / forward + float(cx_px)
    v = float(fy_px) * down / forward + float(cy_px)
    if not all(math.isfinite(value) for value in (u, v)):
        return None
    return u, v


def ground_truth_world_to_baseline(
    point_world_enu_m: Sequence[float],
    baseline: BaselineCenterFRD,
) -> dict[str, float | Vector3 | str]:
    """Convert a world ENU reference point to the public baseline-center FRD contract."""

    point = baseline.point_from_world(point_world_enu_m)
    forward_m, right_m, down_m = point
    range_m = math.sqrt(forward_m * forward_m + right_m * right_m + down_m * down_m)
    return {
        "forward_m": forward_m,
        "right_m": right_m,
        "down_m": down_m,
        "range_m": range_m,
        "ground_truth_forward_m": forward_m,
        "ground_truth_right_m": right_m,
        "ground_truth_down_m": down_m,
        "ground_truth_range_m": range_m,
        "midpoint_baseline_center_frd_m": point,
        "output_frame": BASELINE_CENTER_FRD_FRAME,
    }


__all__ = [
    "BASELINE_CENTER_FRD_FRAME",
    "BaselineCenterFRD",
    "CameraOpticalPose",
    "PX4_BODY_FRD_FROM_OPTICAL_RDF",
    "R_ENU_FROM_NED",
    "REFERENCE_OFFSET_PARENT_M",
    "REFERENCE_OFFSET_PARENT_RPY_RAD",
    "SOURCE_GAZEBO_ENTITY",
    "VEHICLE_REFERENCE_DEFINITION",
    "VEHICLE_REFERENCE_LINK",
    "WORLD_FRAME",
    "apply_optical_rpy_error",
    "apply_optical_yaw",
    "build_baseline_center_frd",
    "camera_optical_pose_from_quaternion",
    "camera_optical_pose_from_px4_attitude_wxyz",
    "camera_optical_pose_from_sdf_pose",
    "effective_optical_rpy_errors_deg",
    "effective_optical_yaws_deg",
    "ground_truth_world_to_baseline",
    "nominal_camera_poses",
    "normalize",
    "optical_rpy_error_rotation",
    "project_world_point_to_pixel",
    "relative_rotation_angle_deg",
    "rpy_to_rotation_world_from_link",
    "rotation_to_sdf_rpy",
    "relative_sdf_rpy",
    "rotation_world_from_quaternion_xyzw",
    "sdf_link_rpy_from_optical_error",
    "sdf_link_rpy_from_px4_frd_error",
    "sdf_pose_rpy_from_camera_pose",
    "validate_rotation_matrix",
]
