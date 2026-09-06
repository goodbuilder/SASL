"""Pure optical-pose geometry with explicit OpenCV/PX4 frame contracts.

This module deliberately keeps the following frames distinct:

* camera optical RDF: ``[Right, Down, Forward]``;
* camera virtual FRD: ``[Forward, Right, Down]`` at the same optical center;
* PX4 body FRD: ``[Forward, Right, Down]`` at the flight-controller body
  origin (connected to virtual FRD only by a separately calibrated mount);
* world NED: ``[North, East, Down]``.

Rotations use ``R_output_from_input`` and column vectors.  A camera pose is
``X_ned = R_ned_from_camera_rdf * X_camera_rdf + C_ned_m``.  OpenCV stereo
extrinsics remain ``X_right = R_right_from_left * X_left + T_right_from_left``.
No function in this module treats GNSS antenna positions as optical centers or
calibration-time reference data as a runtime measurement input.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, asin, atan2, cos, degrees, hypot, isfinite, radians, sin, sqrt
from typing import Any, Iterable, Sequence


from Workstation.workstation_core.triangulation import (
    CAMERA_TO_BODY_ROTATION as OPTICAL_RDF_TO_VIRTUAL_FRD,
    BODY_TO_CAMERA_ROTATION as VIRTUAL_FRD_TO_OPTICAL_RDF,
)

Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]

ROTATION_VALIDATION_TOLERANCE = 1.0e-6
DEFAULT_EARTH_RADIUS_M = 6_371_008.8

# ``v_virtual_frd = OPTICAL_RDF_TO_VIRTUAL_FRD * v_optical_rdf``.
# Single source of truth: this matrix (and its inverse) is defined once in
# ``triangulation.py`` as ``CAMERA_TO_BODY_ROTATION``/``BODY_TO_CAMERA_ROTATION``
# and re-exported here under the optical-pose module's own names, since the
# two modules describe the same physical rotation (optical RDF <-> body/virtual
# FRD) between the camera's optical frame and its FRD-convention counterpart.

POSE_DOF_NAMES = (
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "north_m",
    "east_m",
    "down_m",
)


def _finite_float(value: Any, *, name: str) -> float:
    parsed = float(value)
    if not isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _vector3(values: Iterable[Any], *, name: str) -> Vector3:
    parsed = tuple(_finite_float(value, name=name) for value in values)
    if len(parsed) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    return parsed  # type: ignore[return-value]


def _matrix3(values: Iterable[Iterable[Any]], *, name: str) -> Matrix3:
    rows = tuple(_vector3(row, name=f"{name} row") for row in values)
    if len(rows) != 3:
        raise ValueError(f"{name} must be a 3x3 matrix")
    return rows  # type: ignore[return-value]


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(a * b for a, b in zip(left, right))


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _norm(vector: Vector3) -> float:
    return sqrt(_dot(vector, vector))


def _normalize(vector: Iterable[Any], *, name: str) -> Vector3:
    parsed = _vector3(vector, name=name)
    magnitude = _norm(parsed)
    if magnitude <= 0.0:
        raise ValueError(f"{name} must have non-zero length")
    return tuple(value / magnitude for value in parsed)  # type: ignore[return-value]


def _scale(vector: Vector3, scalar: float) -> Vector3:
    return tuple(value * scalar for value in vector)  # type: ignore[return-value]


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(left, right))  # type: ignore[return-value]


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a - b for a, b in zip(left, right))  # type: ignore[return-value]


def _transpose(matrix: Matrix3) -> Matrix3:
    return tuple(
        tuple(matrix[row][column] for row in range(3)) for column in range(3)
    )  # type: ignore[return-value]


def _matvec(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(_dot(row, vector) for row in matrix)  # type: ignore[return-value]


def _matmul(left: Matrix3, right: Matrix3) -> Matrix3:
    right_transposed = _transpose(right)
    return tuple(
        tuple(_dot(row, column) for column in right_transposed) for row in left
    )  # type: ignore[return-value]


def _determinant(matrix: Matrix3) -> float:
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def _inverse_matrix3(matrix: Matrix3, *, name: str) -> Matrix3:
    determinant = _determinant(matrix)
    if abs(determinant) <= 1.0e-15:
        raise ValueError(f"{name} must be invertible")
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    inverse = (
        ((e * i - f * h) / determinant, (c * h - b * i) / determinant, (b * f - c * e) / determinant),
        ((f * g - d * i) / determinant, (a * i - c * g) / determinant, (c * d - a * f) / determinant),
        ((d * h - e * g) / determinant, (b * g - a * h) / determinant, (a * e - b * d) / determinant),
    )
    return _matrix3(inverse, name=f"inverse({name})")


def _validate_rotation(matrix: Matrix3, *, name: str) -> None:
    product = _matmul(matrix, _transpose(matrix))
    maximum_error = max(
        abs(product[row][column] - (1.0 if row == column else 0.0))
        for row in range(3)
        for column in range(3)
    )
    if maximum_error > ROTATION_VALIDATION_TOLERANCE:
        raise ValueError(f"{name} must be orthonormal within 1e-6")
    if abs(_determinant(matrix) - 1.0) > ROTATION_VALIDATION_TOLERANCE:
        raise ValueError(f"{name} determinant must equal +1 within 1e-6")


def _rotation_x(angle_rad: float) -> Matrix3:
    cosine, sine = cos(angle_rad), sin(angle_rad)
    return ((1.0, 0.0, 0.0), (0.0, cosine, -sine), (0.0, sine, cosine))


def _rotation_y(angle_rad: float) -> Matrix3:
    cosine, sine = cos(angle_rad), sin(angle_rad)
    return ((cosine, 0.0, sine), (0.0, 1.0, 0.0), (-sine, 0.0, cosine))


def _rotation_z(angle_rad: float) -> Matrix3:
    cosine, sine = cos(angle_rad), sin(angle_rad)
    return ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0))


def wrap_angle_deg(value: float) -> float:
    """Wrap an angle in degrees into the half-open interval ``[-180, 180)``."""

    wrapped = (_finite_float(value, name="angle_deg") + 180.0) % 360.0 - 180.0
    return -180.0 if wrapped == 180.0 else wrapped


def optical_rdf_to_virtual_frd(vector_rdf: Iterable[Any]) -> Vector3:
    """Reorder one direction/vector from optical ``[R,D,F]`` to ``[F,R,D]``."""

    return _matvec(OPTICAL_RDF_TO_VIRTUAL_FRD, _vector3(vector_rdf, name="vector_rdf"))


def virtual_frd_to_optical_rdf(vector_frd: Iterable[Any]) -> Vector3:
    """Reorder one direction/vector from virtual ``[F,R,D]`` to optical ``[R,D,F]``."""

    return _matvec(VIRTUAL_FRD_TO_OPTICAL_RDF, _vector3(vector_frd, name="vector_frd"))


def rotation_ned_from_virtual_frd_rpy_deg(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> Matrix3:
    """Return PX4-style 3-2-1 ``R_ned_from_virtual_frd``.

    The active right-handed rotation is ``Rz(yaw) * Ry(pitch) * Rx(roll)``.
    Yaw is clockwise from North towards East because world axes are NED.
    """

    roll = radians(_finite_float(roll_deg, name="roll_deg"))
    pitch = radians(_finite_float(pitch_deg, name="pitch_deg"))
    yaw = radians(_finite_float(yaw_deg, name="yaw_deg"))
    return _matmul(_matmul(_rotation_z(yaw), _rotation_y(pitch)), _rotation_x(roll))


def _virtual_frd_rpy_deg_from_rotation(rotation_ned_from_virtual_frd: Matrix3) -> Vector3:
    rotation = _matrix3(rotation_ned_from_virtual_frd, name="rotation_ned_from_virtual_frd")
    _validate_rotation(rotation, name="rotation_ned_from_virtual_frd")
    sine_pitch = max(-1.0, min(1.0, -rotation[2][0]))
    pitch = asin(sine_pitch)
    if abs(cos(pitch)) <= 1.0e-9:
        raise ValueError("virtual FRD roll/yaw are singular at pitch +/-90 degrees")
    roll = atan2(rotation[2][1], rotation[2][2])
    yaw = atan2(rotation[1][0], rotation[0][0])
    return degrees(roll), degrees(pitch), wrap_angle_deg(degrees(yaw))


@dataclass(frozen=True)
class CameraPoseNed:
    """Optical-center pose in world NED.

    ``rotation_ned_from_camera_rdf`` maps optical RDF directions into NED.
    ``center_ned_m`` is the optical center in ``[North, East, Down]`` metres.
    """

    rotation_ned_from_camera_rdf: Matrix3
    center_ned_m: Vector3

    def __post_init__(self) -> None:
        rotation = _matrix3(
            self.rotation_ned_from_camera_rdf,
            name="rotation_ned_from_camera_rdf",
        )
        _validate_rotation(rotation, name="rotation_ned_from_camera_rdf")
        object.__setattr__(self, "rotation_ned_from_camera_rdf", rotation)
        object.__setattr__(self, "center_ned_m", _vector3(self.center_ned_m, name="center_ned_m"))

    @classmethod
    def from_virtual_frd_rpy_deg(
        cls,
        *,
        center_ned_m: Iterable[Any],
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ) -> "CameraPoseNed":
        rotation_ned_from_virtual_frd = rotation_ned_from_virtual_frd_rpy_deg(
            roll_deg,
            pitch_deg,
            yaw_deg,
        )
        return cls(
            rotation_ned_from_camera_rdf=_matmul(
                rotation_ned_from_virtual_frd,
                OPTICAL_RDF_TO_VIRTUAL_FRD,
            ),
            center_ned_m=_vector3(center_ned_m, name="center_ned_m"),
        )

    @property
    def rotation_ned_from_virtual_frd(self) -> Matrix3:
        return _matmul(
            self.rotation_ned_from_camera_rdf,
            VIRTUAL_FRD_TO_OPTICAL_RDF,
        )

    @property
    def boresight_ned(self) -> Vector3:
        return _matvec(self.rotation_ned_from_camera_rdf, (0.0, 0.0, 1.0))

    def transform_point_to_ned(self, point_camera_rdf_m: Iterable[Any]) -> Vector3:
        point = _vector3(point_camera_rdf_m, name="point_camera_rdf_m")
        return _add(_matvec(self.rotation_ned_from_camera_rdf, point), self.center_ned_m)


@dataclass(frozen=True)
class StereoPoseRdf:
    """OpenCV left-to-right stereo transform in optical RDF metres.

    ``X_right = rotation_right_from_left_rdf * X_left
    + translation_right_from_left_rdf_m``.
    """

    rotation_right_from_left_rdf: Matrix3
    translation_right_from_left_rdf_m: Vector3

    def __post_init__(self) -> None:
        rotation = _matrix3(
            self.rotation_right_from_left_rdf,
            name="rotation_right_from_left_rdf",
        )
        _validate_rotation(rotation, name="rotation_right_from_left_rdf")
        object.__setattr__(self, "rotation_right_from_left_rdf", rotation)
        object.__setattr__(
            self,
            "translation_right_from_left_rdf_m",
            _vector3(
                self.translation_right_from_left_rdf_m,
                name="translation_right_from_left_rdf_m",
            ),
        )

    @property
    def right_origin_in_left_rdf_m(self) -> Vector3:
        return _scale(
            _matvec(
                _transpose(self.rotation_right_from_left_rdf),
                self.translation_right_from_left_rdf_m,
            ),
            -1.0,
        )

    @property
    def baseline_m(self) -> float:
        return _norm(self.right_origin_in_left_rdf_m)


def stereo_pose_from_absolute(left: CameraPoseNed, right: CameraPoseNed) -> StereoPoseRdf:
    """Derive OpenCV ``R_right_from_left,T_right_from_left`` from NED poses."""

    rotation_right_from_ned = _transpose(right.rotation_ned_from_camera_rdf)
    return StereoPoseRdf(
        rotation_right_from_left_rdf=_matmul(
            rotation_right_from_ned,
            left.rotation_ned_from_camera_rdf,
        ),
        translation_right_from_left_rdf_m=_matvec(
            rotation_right_from_ned,
            _subtract(left.center_ned_m, right.center_ned_m),
        ),
    )


def _boresight_yaw_deg(pose: CameraPoseNed, *, horizontal_epsilon: float) -> float:
    north, east, _down = pose.boresight_ned
    if hypot(north, east) <= horizontal_epsilon:
        raise ValueError("boresight yaw is unobservable for a vertical optical axis")
    return wrap_angle_deg(degrees(atan2(east, north)))


def boresight_relative_yaw_right_minus_left_deg(
    left: CameraPoseNed,
    right: CameraPoseNed,
    *,
    horizontal_epsilon: float = 1.0e-12,
) -> float:
    """Return ``wrap(yaw_right - yaw_left)`` from NED optical boresights.

    This diagnostic is intentionally not an Euler angle of OpenCV
    ``R_right_from_left_rdf``; optical Z is Forward, not NED Down.
    """

    epsilon = _finite_float(horizontal_epsilon, name="horizontal_epsilon")
    if epsilon < 0.0:
        raise ValueError("horizontal_epsilon must be non-negative")
    return wrap_angle_deg(
        _boresight_yaw_deg(right, horizontal_epsilon=epsilon)
        - _boresight_yaw_deg(left, horizontal_epsilon=epsilon)
    )


def _orient_gravity(
    gravity_camera_rdf: Vector3,
    expected_gravity_camera_rdf: Iterable[Any] | None,
) -> Vector3:
    gravity = _normalize(gravity_camera_rdf, name="gravity_camera_rdf")
    expected = (
        (0.0, 1.0, 0.0)
        if expected_gravity_camera_rdf is None
        else _normalize(expected_gravity_camera_rdf, name="expected_gravity_camera_rdf")
    )
    return _scale(gravity, -1.0) if _dot(gravity, expected) < 0.0 else gravity


def gravity_camera_rdf_from_normalized_horizon_line(
    normalized_line: Iterable[Any],
    *,
    expected_gravity_camera_rdf: Iterable[Any] | None = None,
) -> Vector3:
    """Recover camera gravity from a line fit in normalized ideal coordinates.

    ``normalized_line=(a,b,c)`` means ``a*x+b*y+c=0`` after
    ``cv2.undistortPoints(..., P omitted)``.  Do not multiply this result by
    ``K.T`` again.  The projective sign is selected using the expected gravity;
    the default assumes an upright image whose positive V axis points Down.
    """

    return _orient_gravity(
        _vector3(normalized_line, name="normalized_line"),
        expected_gravity_camera_rdf,
    )


def gravity_camera_rdf_from_pixel_horizon_line(
    pixel_line: Iterable[Any],
    camera_matrix: Iterable[Iterable[Any]],
    *,
    expected_gravity_camera_rdf: Iterable[Any] | None = None,
) -> Vector3:
    """Recover gravity as ``normalize(K.T * l_pixel)`` from ideal pixels.

    The input pixels must already be undistorted on the supplied ``K`` grid.
    Passing a raw distorted-image line or normalized coordinates is invalid.
    """

    line = _vector3(pixel_line, name="pixel_line")
    intrinsics = _matrix3(camera_matrix, name="camera_matrix")
    if abs(_determinant(intrinsics)) <= 1.0e-15:
        raise ValueError("camera_matrix must be invertible")
    return _orient_gravity(
        _matvec(_transpose(intrinsics), line),
        expected_gravity_camera_rdf,
    )


def normalized_horizon_line_from_gravity_camera_rdf(
    gravity_camera_rdf: Iterable[Any],
) -> Vector3:
    """Return the flat-horizon line in normalized ideal camera coordinates."""

    return _normalize(gravity_camera_rdf, name="gravity_camera_rdf")


def pixel_horizon_line_from_gravity_camera_rdf(
    gravity_camera_rdf: Iterable[Any],
    camera_matrix: Iterable[Iterable[Any]],
) -> Vector3:
    """Return an ideal-pixel line proportional to ``K^-T * gravity``."""

    gravity = _normalize(gravity_camera_rdf, name="gravity_camera_rdf")
    intrinsics = _matrix3(camera_matrix, name="camera_matrix")
    return _normalize(
        _matvec(_transpose(_inverse_matrix3(intrinsics, name="camera_matrix")), gravity),
        name="pixel_horizon_line",
    )


def horizon_roll_pitch_deg_from_gravity_camera_rdf(
    gravity_camera_rdf: Iterable[Any],
) -> tuple[float, float]:
    """Return virtual-FRD ``(roll,pitch)``; yaw is not observable.

    Gravity is world NED Down expressed in camera optical RDF.  The returned
    angles follow ``R_ned_from_virtual_frd = Rz(yaw) Ry(pitch) Rx(roll)``.
    """

    forward, right, down = optical_rdf_to_virtual_frd(
        _normalize(gravity_camera_rdf, name="gravity_camera_rdf")
    )
    roll = atan2(right, down)
    pitch = atan2(-forward, hypot(right, down))
    return degrees(roll), degrees(pitch)


def geometric_horizon_dip_rad(
    camera_height_m: float,
    *,
    earth_radius_m: float = DEFAULT_EARTH_RADIUS_M,
) -> float:
    """Return unrefracted spherical-Earth horizon dip below local level."""

    height = _finite_float(camera_height_m, name="camera_height_m")
    radius = _finite_float(earth_radius_m, name="earth_radius_m")
    if height < 0.0:
        raise ValueError("camera_height_m must be non-negative")
    if radius <= 0.0:
        raise ValueError("earth_radius_m must be positive")
    return acos(radius / (radius + height))


def geometric_horizon_dip_deg(
    camera_height_m: float,
    *,
    earth_radius_m: float = DEFAULT_EARTH_RADIUS_M,
) -> float:
    return degrees(geometric_horizon_dip_rad(camera_height_m, earth_radius_m=earth_radius_m))


def horizon_unit_ray_residual(
    gravity_camera_rdf: Iterable[Any],
    horizon_ray_camera_rdf: Iterable[Any],
    *,
    dip_rad: float = 0.0,
) -> float:
    """Return dimensionless spherical-horizon residual ``g.r - sin(dip)``."""

    dip = _finite_float(dip_rad, name="dip_rad")
    if not 0.0 <= dip < 0.5 * 3.141592653589793:
        raise ValueError("dip_rad must be in [0, pi/2)")
    return _dot(
        _normalize(gravity_camera_rdf, name="gravity_camera_rdf"),
        _normalize(horizon_ray_camera_rdf, name="horizon_ray_camera_rdf"),
    ) - sin(dip)


def angular_ray_residual_rad(
    rotation_ned_from_camera_rdf: Iterable[Iterable[Any]],
    observed_ray_camera_rdf: Iterable[Any],
    expected_los_ned: Iterable[Any],
) -> float:
    """Return the stable angle between an observed optical ray and NED LOS."""

    rotation = _matrix3(
        rotation_ned_from_camera_rdf,
        name="rotation_ned_from_camera_rdf",
    )
    _validate_rotation(rotation, name="rotation_ned_from_camera_rdf")
    observed_ned = _normalize(
        _matvec(rotation, _normalize(observed_ray_camera_rdf, name="observed_ray_camera_rdf")),
        name="observed_ray_ned",
    )
    expected = _normalize(expected_los_ned, name="expected_los_ned")
    return atan2(_norm(_cross(observed_ned, expected)), _dot(observed_ned, expected))


@dataclass(frozen=True)
class DroneYawEstimate:
    """Yaw-only result from one RTK LOS and one camera normalized ray."""

    yaw_deg: float | None
    observable: bool
    ray_horizontal_norm: float
    los_horizontal_norm: float
    angular_residual_rad: float | None
    reason: str

    def __post_init__(self) -> None:
        if type(self.observable) is not bool:
            raise TypeError("observable must be bool")
        yaw = None if self.yaw_deg is None else wrap_angle_deg(self.yaw_deg)
        ray_norm = _finite_float(self.ray_horizontal_norm, name="ray_horizontal_norm")
        los_norm = _finite_float(self.los_horizontal_norm, name="los_horizontal_norm")
        residual = (
            None
            if self.angular_residual_rad is None
            else _finite_float(self.angular_residual_rad, name="angular_residual_rad")
        )
        if ray_norm < 0.0 or los_norm < 0.0 or (residual is not None and residual < 0.0):
            raise ValueError("Drone yaw norms/residual must be non-negative")
        observable = self.observable
        if observable != (yaw is not None and residual is not None):
            raise ValueError("observable Drone yaw requires finite yaw and residual")
        object.__setattr__(self, "yaw_deg", yaw)
        object.__setattr__(self, "observable", observable)
        object.__setattr__(self, "ray_horizontal_norm", ray_norm)
        object.__setattr__(self, "los_horizontal_norm", los_norm)
        object.__setattr__(self, "angular_residual_rad", residual)
        object.__setattr__(self, "reason", str(self.reason))


def estimate_drone_yaw_deg(
    observed_ray_camera_rdf: Iterable[Any],
    expected_los_ned: Iterable[Any],
    *,
    roll_deg: float,
    pitch_deg: float,
    horizontal_epsilon: float = 1.0e-9,
) -> DroneYawEstimate:
    """Estimate absolute virtual-FRD yaw with roll/pitch and position fixed.

    The expected LOS must be formed from the same timestamp and from the
    camera optical center to the visual point corresponding to the marked
    pixel.  A GNSS antenna position is only a proxy until lever arms are
    applied.  A vertical ray/LOS makes yaw unobservable.
    """

    ray = _normalize(observed_ray_camera_rdf, name="observed_ray_camera_rdf")
    los = _normalize(expected_los_ned, name="expected_los_ned")
    roll = radians(_finite_float(roll_deg, name="roll_deg"))
    pitch = radians(_finite_float(pitch_deg, name="pitch_deg"))
    epsilon = _finite_float(horizontal_epsilon, name="horizontal_epsilon")
    if epsilon < 0.0:
        raise ValueError("horizontal_epsilon must be non-negative")

    ray_before_yaw = _matvec(
        _matmul(_rotation_y(pitch), _rotation_x(roll)),
        optical_rdf_to_virtual_frd(ray),
    )
    ray_horizontal = hypot(ray_before_yaw[0], ray_before_yaw[1])
    los_horizontal = hypot(los[0], los[1])
    if ray_horizontal <= epsilon or los_horizontal <= epsilon:
        return DroneYawEstimate(
            yaw_deg=None,
            observable=False,
            ray_horizontal_norm=ray_horizontal,
            los_horizontal_norm=los_horizontal,
            angular_residual_rad=None,
            reason="vertical_ray_or_los",
        )

    yaw_rad = atan2(los[1], los[0]) - atan2(ray_before_yaw[1], ray_before_yaw[0])
    yaw_deg = wrap_angle_deg(degrees(yaw_rad))
    pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0),
        roll_deg=degrees(roll),
        pitch_deg=degrees(pitch),
        yaw_deg=yaw_deg,
    )
    residual = angular_ray_residual_rad(
        pose.rotation_ned_from_camera_rdf,
        ray,
        los,
    )
    return DroneYawEstimate(
        yaw_deg=yaw_deg,
        observable=True,
        ray_horizontal_norm=ray_horizontal,
        los_horizontal_norm=los_horizontal,
        angular_residual_rad=residual,
        reason="ok",
    )


def _opencv_translation_vector(values: Iterable[Any]) -> Vector3:
    parsed: list[float] = []
    for value in values:
        if isinstance(value, (str, bytes)):
            raise TypeError("translation_camera_from_ned_m must contain numeric values")
        try:
            nested = tuple(value)  # type: ignore[arg-type]
        except TypeError:
            parsed.append(_finite_float(value, name="translation_camera_from_ned_m"))
        else:
            if len(nested) != 1:
                raise ValueError("translation_camera_from_ned_m must be shape (3,) or (3,1)")
            parsed.append(_finite_float(nested[0], name="translation_camera_from_ned_m"))
    return _vector3(parsed, name="translation_camera_from_ned_m")


def camera_pose_ned_from_opencv_pnp(
    rotation_camera_from_ned: Iterable[Iterable[Any]],
    translation_camera_from_ned_m: Iterable[Any],
) -> CameraPoseNed:
    """Invert OpenCV PnP ``X_camera = R_camera_from_ned*X_ned+t``.

    Object points must be world NED metres.  Convert an OpenCV Rodrigues rvec
    to its 3x3 matrix before calling this helper.
    """

    rotation_camera_from_ned_matrix = _matrix3(
        rotation_camera_from_ned,
        name="rotation_camera_from_ned",
    )
    _validate_rotation(rotation_camera_from_ned_matrix, name="rotation_camera_from_ned")
    translation = _opencv_translation_vector(translation_camera_from_ned_m)
    rotation_ned_from_camera = _transpose(rotation_camera_from_ned_matrix)
    center_ned = _scale(_matvec(rotation_ned_from_camera, translation), -1.0)
    return CameraPoseNed(rotation_ned_from_camera, center_ned)


def opencv_pnp_from_camera_pose_ned(pose: CameraPoseNed) -> tuple[Matrix3, Vector3]:
    """Return OpenCV PnP ``(R_camera_from_ned,t_camera_from_ned_m)``."""

    rotation_camera_from_ned = _transpose(pose.rotation_ned_from_camera_rdf)
    translation = _scale(_matvec(rotation_camera_from_ned, pose.center_ned_m), -1.0)
    return rotation_camera_from_ned, translation


@dataclass(frozen=True)
class SE3Transform:
    """Rigid transform ``X_output = R_output_from_input*X_input+t`` in metres."""

    output_frame: str
    input_frame: str
    rotation_output_from_input: Matrix3
    translation_output_from_input_m: Vector3

    def __post_init__(self) -> None:
        output_frame = str(self.output_frame).strip()
        input_frame = str(self.input_frame).strip()
        if not output_frame or not input_frame:
            raise ValueError("SE3 input/output frames must be non-empty")
        rotation = _matrix3(
            self.rotation_output_from_input,
            name="rotation_output_from_input",
        )
        _validate_rotation(rotation, name="rotation_output_from_input")
        object.__setattr__(self, "output_frame", output_frame)
        object.__setattr__(self, "input_frame", input_frame)
        object.__setattr__(self, "rotation_output_from_input", rotation)
        object.__setattr__(
            self,
            "translation_output_from_input_m",
            _vector3(
                self.translation_output_from_input_m,
                name="translation_output_from_input_m",
            ),
        )

    def transform_point(self, point_input_m: Iterable[Any]) -> Vector3:
        return _add(
            _matvec(
                self.rotation_output_from_input,
                _vector3(point_input_m, name="point_input_m"),
            ),
            self.translation_output_from_input_m,
        )

    def inverse(self) -> "SE3Transform":
        rotation = _transpose(self.rotation_output_from_input)
        return SE3Transform(
            output_frame=self.input_frame,
            input_frame=self.output_frame,
            rotation_output_from_input=rotation,
            translation_output_from_input_m=_scale(
                _matvec(rotation, self.translation_output_from_input_m),
                -1.0,
            ),
        )

    def compose(self, inner: "SE3Transform") -> "SE3Transform":
        return compose_se3(self, inner)


def compose_se3(outer: SE3Transform, inner: SE3Transform) -> SE3Transform:
    """Compose ``H_output<-middle * H_middle<-input`` with frame checking."""

    if outer.input_frame != inner.output_frame:
        raise ValueError(
            "SE3 frame mismatch: "
            f"outer input {outer.input_frame!r} != inner output {inner.output_frame!r}"
        )
    return SE3Transform(
        output_frame=outer.output_frame,
        input_frame=inner.input_frame,
        rotation_output_from_input=_matmul(
            outer.rotation_output_from_input,
            inner.rotation_output_from_input,
        ),
        translation_output_from_input_m=_add(
            _matvec(
                outer.rotation_output_from_input,
                inner.translation_output_from_input_m,
            ),
            outer.translation_output_from_input_m,
        ),
    )


def transfer_stereo_pose_through_bridges(
    source_stereo_pose: StereoPoseRdf,
    *,
    left_target_from_source: SE3Transform,
    right_target_from_source: SE3Transform,
) -> StereoPoseRdf:
    """Transfer stereo pose through full per-camera zoom/HR SE(3) bridges.

    ``left_target_from_source`` maps left source optical coordinates to left
    target optical coordinates; the right bridge has the analogous contract.
    The result is ``H_right_target<-left_target``.  A rotation-only bridge may
    use zero translation, but that zero is an explicit coincident-center
    assumption and does not estimate translation.
    """

    source = SE3Transform(
        output_frame=right_target_from_source.input_frame,
        input_frame=left_target_from_source.input_frame,
        rotation_output_from_input=source_stereo_pose.rotation_right_from_left_rdf,
        translation_output_from_input_m=source_stereo_pose.translation_right_from_left_rdf_m,
    )
    target = compose_se3(
        compose_se3(right_target_from_source, source),
        left_target_from_source.inverse(),
    )
    return StereoPoseRdf(
        rotation_right_from_left_rdf=target.rotation_output_from_input,
        translation_right_from_left_rdf_m=target.translation_output_from_input_m,
    )


@dataclass(frozen=True)
class PoseParametersNed:
    """Six scalar pose parameters: virtual-FRD RPY degrees and NED metres."""

    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    north_m: float
    east_m: float
    down_m: float

    def __post_init__(self) -> None:
        for name in POSE_DOF_NAMES:
            object.__setattr__(self, name, _finite_float(getattr(self, name), name=name))

    def to_camera_pose_ned(self) -> CameraPoseNed:
        return CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=(self.north_m, self.east_m, self.down_m),
            roll_deg=self.roll_deg,
            pitch_deg=self.pitch_deg,
            yaw_deg=self.yaw_deg,
        )


def pose_parameters_from_camera_pose(pose: CameraPoseNed) -> PoseParametersNed:
    """Extract non-singular virtual-FRD 3-2-1 RPY plus NED optical center."""

    roll, pitch, yaw = _virtual_frd_rpy_deg_from_rotation(
        pose.rotation_ned_from_virtual_frd
    )
    return PoseParametersNed(roll, pitch, yaw, *pose.center_ned_m)


@dataclass(frozen=True)
class PoseDofMask:
    """Select independently optimizable virtual-FRD/NED pose scalars."""

    roll_deg: bool = True
    pitch_deg: bool = True
    yaw_deg: bool = True
    north_m: bool = False
    east_m: bool = False
    down_m: bool = False

    def __post_init__(self) -> None:
        for name in POSE_DOF_NAMES:
            if type(getattr(self, name)) is not bool:
                raise TypeError(f"{name} mask must be bool")

    @classmethod
    def from_names(cls, names: Iterable[str]) -> "PoseDofMask":
        selected = tuple(str(name) for name in names)
        unknown = set(selected) - set(POSE_DOF_NAMES)
        if unknown:
            raise ValueError(f"unknown pose DOF names: {sorted(unknown)}")
        if len(selected) != len(set(selected)):
            raise ValueError("pose DOF names must be unique")
        return cls(**{name: name in selected for name in POSE_DOF_NAMES})

    @property
    def selected_names(self) -> tuple[str, ...]:
        return tuple(name for name in POSE_DOF_NAMES if getattr(self, name))


def pack_selected_dofs(parameters: PoseParametersNed, mask: PoseDofMask) -> tuple[float, ...]:
    """Pack solver variables in canonical ``POSE_DOF_NAMES`` order."""

    return tuple(getattr(parameters, name) for name in mask.selected_names)


def expand_selected_dofs(
    prior: PoseParametersNed,
    mask: PoseDofMask,
    selected_values: Sequence[Any],
) -> PoseParametersNed:
    """Expand solver variables while preserving unchecked prior values exactly."""

    values = tuple(
        _finite_float(value, name=f"selected_values[{index}]")
        for index, value in enumerate(selected_values)
    )
    if len(values) != len(mask.selected_names):
        raise ValueError(
            f"selected_values must contain {len(mask.selected_names)} values"
        )
    replacements = dict(zip(mask.selected_names, values))
    return PoseParametersNed(
        **{
            name: replacements[name] if name in replacements else getattr(prior, name)
            for name in POSE_DOF_NAMES
        }
    )


def apply_selected_dofs(
    prior: PoseParametersNed,
    candidate: PoseParametersNed,
    mask: PoseDofMask,
) -> PoseParametersNed:
    """Apply an already constrained candidate without editing rotation matrices.

    This is only an expansion/finalization helper.  It is not a substitute for
    re-solving the original observations with ``pack_selected_dofs`` and
    ``expand_selected_dofs`` under the same mask.
    """

    return expand_selected_dofs(prior, mask, pack_selected_dofs(candidate, mask))


def _immutable_strings(values: Iterable[Any], *, name: str) -> tuple[str, ...]:
    parsed = tuple(str(value) for value in values)
    if any(not value for value in parsed):
        raise ValueError(f"{name} values must be non-empty")
    if len(parsed) != len(set(parsed)):
        raise ValueError(f"{name} values must be unique")
    return parsed


def _immutable_text_pairs(values: Iterable[Sequence[Any]], *, name: str) -> tuple[tuple[str, str], ...]:
    parsed: list[tuple[str, str]] = []
    for value in values:
        if len(value) != 2:
            raise ValueError(f"each {name} item must have two values")
        key, item = str(value[0]), str(value[1])
        if not key or not item:
            raise ValueError(f"{name} keys/values must be non-empty")
        parsed.append((key, item))
    if len({key for key, _item in parsed}) != len(parsed):
        raise ValueError(f"{name} keys must be unique")
    return tuple(sorted(parsed))


def _immutable_float_pairs(
    values: Iterable[Sequence[Any]],
    *,
    name: str,
    non_negative: bool = False,
) -> tuple[tuple[str, float], ...]:
    parsed: list[tuple[str, float]] = []
    for value in values:
        if len(value) != 2:
            raise ValueError(f"each {name} item must have two values")
        key = str(value[0])
        item = _finite_float(value[1], name=f"{name}.{key}")
        if not key:
            raise ValueError(f"{name} keys must be non-empty")
        if non_negative and item < 0.0:
            raise ValueError(f"{name}.{key} must be non-negative")
        parsed.append((key, item))
    if len({key for key, _item in parsed}) != len(parsed):
        raise ValueError(f"{name} keys must be unique")
    return tuple(sorted(parsed))


def _immutable_observability(
    values: Iterable[Sequence[Any]],
) -> tuple[tuple[str, bool], ...]:
    parsed: dict[str, bool] = {}
    for value in values:
        if len(value) != 2:
            raise ValueError("each observability item must have two values")
        name, observable = str(value[0]), value[1]
        if name not in POSE_DOF_NAMES:
            raise ValueError(f"unknown observability DOF: {name}")
        if name in parsed:
            raise ValueError(f"duplicate observability DOF: {name}")
        if type(observable) is not bool:
            raise TypeError(f"observability.{name} must be bool")
        parsed[name] = observable
    missing = set(POSE_DOF_NAMES) - set(parsed)
    if missing:
        raise ValueError(f"observability must include all pose DOFs: {sorted(missing)}")
    return tuple((name, parsed[name]) for name in POSE_DOF_NAMES)


@dataclass(frozen=True)
class OpticalPoseCandidate:
    """Deep-immutable reference-assisted optical-pose candidate.

    Quality never silently turns the candidate into calibration truth.  The
    caller may display/apply it explicitly and records that action separately.
    """

    candidate_id: str
    method: str
    left_pose_ned: CameraPoseNed
    right_pose_ned: CameraPoseNed
    optimized_dofs: PoseDofMask
    observability: tuple[tuple[str, bool], ...]
    source_observation_ids: tuple[str, ...] = ()
    input_hashes: tuple[tuple[str, str], ...] = ()
    quality_metrics: tuple[tuple[str, float], ...] = ()
    uncertainty_metrics: tuple[tuple[str, float], ...] = ()
    warnings: tuple[str, ...] = ()
    created_utc_ns: int | None = None
    observability_by_side: tuple[
        tuple[str, tuple[tuple[str, bool], ...]], ...
    ] = ()

    def __post_init__(self) -> None:
        candidate_id = str(self.candidate_id).strip()
        method = str(self.method).strip()
        if not candidate_id or not method:
            raise ValueError("candidate_id and method must be non-empty")
        if not isinstance(self.left_pose_ned, CameraPoseNed) or not isinstance(
            self.right_pose_ned, CameraPoseNed
        ):
            raise TypeError("candidate poses must be CameraPoseNed")
        if not isinstance(self.optimized_dofs, PoseDofMask):
            raise TypeError("optimized_dofs must be PoseDofMask")
        created = None if self.created_utc_ns is None else int(self.created_utc_ns)
        if created is not None and created < 0:
            raise ValueError("created_utc_ns must be non-negative")
        object.__setattr__(self, "candidate_id", candidate_id)
        object.__setattr__(self, "method", method)
        object.__setattr__(self, "observability", _immutable_observability(self.observability))
        object.__setattr__(
            self,
            "source_observation_ids",
            _immutable_strings(self.source_observation_ids, name="source_observation_ids"),
        )
        object.__setattr__(self, "input_hashes", _immutable_text_pairs(self.input_hashes, name="input_hashes"))
        object.__setattr__(
            self,
            "quality_metrics",
            _immutable_float_pairs(self.quality_metrics, name="quality_metrics"),
        )
        object.__setattr__(
            self,
            "uncertainty_metrics",
            _immutable_float_pairs(
                self.uncertainty_metrics,
                name="uncertainty_metrics",
                non_negative=True,
            ),
        )
        object.__setattr__(self, "warnings", tuple(str(value) for value in self.warnings))
        object.__setattr__(self, "created_utc_ns", created)
        side_rows: list[tuple[str, tuple[tuple[str, bool], ...]]] = []
        if self.observability_by_side:
            seen: set[str] = set()
            for side, values in self.observability_by_side:
                side_name = str(side)
                if side_name not in {"left", "right"} or side_name in seen:
                    raise ValueError("observability_by_side must contain unique left/right rows")
                seen.add(side_name)
                side_rows.append((side_name, _immutable_observability(values)))
            if seen != {"left", "right"}:
                raise ValueError("observability_by_side must contain both left and right")
        else:
            side_rows = [
                ("left", self.observability),
                ("right", self.observability),
            ]
        object.__setattr__(self, "observability_by_side", tuple(side_rows))

    @property
    def stereo_pose_rdf(self) -> StereoPoseRdf:
        return stereo_pose_from_absolute(self.left_pose_ned, self.right_pose_ned)

    @property
    def relative_boresight_yaw_right_minus_left_deg(self) -> float:
        return boresight_relative_yaw_right_minus_left_deg(
            self.left_pose_ned,
            self.right_pose_ned,
        )

    def observable_dofs_for_side(self, side: str) -> tuple[str, ...]:
        rows = dict(self.observability_by_side)
        if side not in rows:
            raise ValueError(f"unknown camera side: {side}")
        observable = dict(rows[side])
        return tuple(
            name
            for name in self.optimized_dofs.selected_names
            if observable.get(name, False)
        )


@dataclass(frozen=True)
class OpticalPoseSelection:
    """Deep-immutable record of an operator-selected candidate/DOF subset."""

    selection_id: str
    candidate_id: str
    selected_dofs: PoseDofMask
    effective_from_pair_sequence: int
    applied_utc_ns: int
    previous_profile_id: str = ""
    quality_snapshot: tuple[tuple[str, float], ...] = ()

    def __post_init__(self) -> None:
        selection_id = str(self.selection_id).strip()
        candidate_id = str(self.candidate_id).strip()
        if not selection_id or not candidate_id:
            raise ValueError("selection_id and candidate_id must be non-empty")
        if not isinstance(self.selected_dofs, PoseDofMask):
            raise TypeError("selected_dofs must be PoseDofMask")
        pair_sequence = int(self.effective_from_pair_sequence)
        applied_utc_ns = int(self.applied_utc_ns)
        if pair_sequence < 0 or applied_utc_ns < 0:
            raise ValueError("selection pair sequence and UTC nanoseconds must be non-negative")
        object.__setattr__(self, "selection_id", selection_id)
        object.__setattr__(self, "candidate_id", candidate_id)
        object.__setattr__(self, "effective_from_pair_sequence", pair_sequence)
        object.__setattr__(self, "applied_utc_ns", applied_utc_ns)
        object.__setattr__(self, "previous_profile_id", str(self.previous_profile_id))
        object.__setattr__(
            self,
            "quality_snapshot",
            _immutable_float_pairs(self.quality_snapshot, name="quality_snapshot"),
        )
