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


def _transpose(matrix: Matrix3) -> Matrix3:
    return tuple(tuple(matrix[row][column] for row in range(3)) for column in range(3))  # type: ignore[return-value]


def _det(matrix: Matrix3) -> float:
    a, b, c = matrix
    return (
        a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
    )


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


def apply_optical_yaw(
    nominal_camera_pose: CameraOpticalPose,
    yaw_error_rad: float,
) -> CameraOpticalPose:
    """Rotate an optical RDF pose about +Down using the right-hand rule.

    Positive yaw rotates optical Forward toward optical Right. The camera
    center and Down axis are unchanged.
    """

    yaw = float(yaw_error_rad)
    if not math.isfinite(yaw):
        raise ValueError("yaw_error_rad must be finite")
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    nominal_right = nominal_camera_pose.right_world_enu
    nominal_forward = nominal_camera_pose.forward_world_enu
    return CameraOpticalPose(
        center_world_enu_m=nominal_camera_pose.center_world_enu_m,
        right_world_enu=normalize(
            _sub(_scale(nominal_right, cosine), _scale(nominal_forward, sine)),
            "yawed_right_world_enu",
        ),
        down_world_enu=nominal_camera_pose.down_world_enu,
        forward_world_enu=normalize(
            _add(_scale(nominal_right, sine), _scale(nominal_forward, cosine)),
            "yawed_forward_world_enu",
        ),
    )


def effective_optical_yaws_deg(
    common_optical_yaw_error_deg: float,
    relative_optical_yaw_error_deg: float,
) -> tuple[float, float]:
    """Return effective left/right optical yaw errors in degrees."""

    common = float(common_optical_yaw_error_deg)
    relative = float(relative_optical_yaw_error_deg)
    if not math.isfinite(common) or not math.isfinite(relative):
        raise ValueError("optical yaw errors must be finite")
    return common - relative * 0.5, common + relative * 0.5


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
    "REFERENCE_OFFSET_PARENT_M",
    "REFERENCE_OFFSET_PARENT_RPY_RAD",
    "SOURCE_GAZEBO_ENTITY",
    "VEHICLE_REFERENCE_DEFINITION",
    "VEHICLE_REFERENCE_LINK",
    "WORLD_FRAME",
    "apply_optical_yaw",
    "build_baseline_center_frd",
    "camera_optical_pose_from_sdf_pose",
    "effective_optical_yaws_deg",
    "ground_truth_world_to_baseline",
    "nominal_camera_poses",
    "normalize",
    "project_world_point_to_pixel",
    "rpy_to_rotation_world_from_link",
]
