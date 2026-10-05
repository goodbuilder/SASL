from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Sequence


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]

CAMERA_OPTICAL_RDF_FRAME = "camera_optical_rdf"
LEFT_CAMERA_RDF_FRAME = "left_camera_rdf"
LEFT_CAMERA_RDF_AXIS_CONTRACT = "left_camera_rdf:right_down_forward"
LEFT_CAMERA_RDF_VECTOR_CONTRACT = "left_camera_rdf:[right,down,forward,range]"
LEFT_CAMERA_FRD_AXIS_CONTRACT = "left_camera_frd:forward_right_down"
BASELINE_CENTER_FRD_FRAME = "baseline_center_frd"
BASELINE_CENTER_FRD_AXIS_CONTRACT = "baseline_center_frd:forward_right_down"
BASELINE_CENTER_FRD_VECTOR_CONTRACT = "baseline_center_frd:[forward,right,down,range]"
TRIANGULATION_METHOD = "two_ray_closest_midpoint"
GEOMETRY_POLICY_LEGACY_FORWARD_ONLY = "legacy_forward_only"
GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH = "diagnostic_signed_depth"

IDENTITY_ROTATION: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


@dataclass(frozen=True)
class BaselineCenterFrdTransform:
    """Attempt-frozen transform from left optical RDF into baseline-center FRD.

    Input points are ordered ``(Right, Down, Forward)`` in the left camera
    optical frame, in metres. Output points are ordered
    ``(Forward, Right, Down)`` from the midpoint between the left and right
    optical centers, in metres.
    """

    origin_left_rdf_m: Vector3
    forward_axis_left_rdf: Vector3
    right_axis_left_rdf: Vector3
    down_axis_left_rdf: Vector3
    frame: str = BASELINE_CENTER_FRD_FRAME
    axis_contract: str = BASELINE_CENTER_FRD_AXIS_CONTRACT
    vector_contract: str = BASELINE_CENTER_FRD_VECTOR_CONTRACT

    def point_from_left_rdf(self, point_left_rdf_m: Sequence[float]) -> Vector3:
        point = _vector3(point_left_rdf_m, name="point_left_rdf_m")
        delta = _subtract(point, self.origin_left_rdf_m)
        return (
            _dot(self.forward_axis_left_rdf, delta),
            _dot(self.right_axis_left_rdf, delta),
            _dot(self.down_axis_left_rdf, delta),
        )

    def range_from_left_rdf(self, point_left_rdf_m: Sequence[float]) -> float:
        point = _vector3(point_left_rdf_m, name="point_left_rdf_m")
        return _norm(_subtract(point, self.origin_left_rdf_m))


@dataclass(frozen=True)
class PinholeIntrinsics:
    """Pinhole intrinsics in pixels for an OpenCV optical RDF camera.

    ``u = fx * Right / Forward + cx`` and
    ``v = fy * Down / Forward + cy``.
    """

    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float

    def __post_init__(self) -> None:
        values = (self.fx_px, self.fy_px, self.cx_px, self.cy_px)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if self.fx_px <= 0.0 or self.fy_px <= 0.0:
            raise ValueError("focal lengths must be positive")


@dataclass(frozen=True)
class RayTriangulationResult:
    """Closest-ray midpoint expressed in the supplied common frame.

    ``point_frame``/``axis_contract``/``vector_contract`` describe the legacy
    ``*_rdf_m`` payload. ``output_point_m`` stores the public point when
    ``output_frame`` names a different frame, with its contracts recorded by
    ``output_axis_contract``/``output_vector_contract``. ``range_m`` is the
    Euclidean norm from the public output origin.
    Both camera origins and rays are expressed in the same common frame before
    this result is constructed. The ``*_rdf_m`` names are retained for legacy
    common-kernel compatibility. Results converted by
    ``result_as_baseline_center_frd`` keep those fields and ``point_frame`` in
    actual left-camera RDF; use ``point_m``/``midpoint_m`` for frame-neutral
    public output.
    """

    point_left_rdf_m: Vector3
    range_m: float
    closest_gap_m: float
    left_depth_m: float
    right_depth_m: float
    ray_angle_deg: float
    baseline_m: float
    point_on_left_ray_rdf_m: Vector3 | None = None
    point_on_right_ray_rdf_m: Vector3 | None = None
    midpoint_rdf_m: Vector3 | None = None
    geometry_rank: int = 2
    condition_number: float = 1.0
    computation_status: str = "computed"
    method: str = TRIANGULATION_METHOD
    point_frame: str = LEFT_CAMERA_RDF_FRAME
    output_frame: str = LEFT_CAMERA_RDF_FRAME
    axis_contract: str = LEFT_CAMERA_RDF_AXIS_CONTRACT
    vector_contract: str = LEFT_CAMERA_RDF_VECTOR_CONTRACT
    output_point_m: Vector3 | None = None
    output_point_on_left_ray_m: Vector3 | None = None
    output_point_on_right_ray_m: Vector3 | None = None
    output_midpoint_m: Vector3 | None = None
    output_axis_contract: str | None = None
    output_vector_contract: str | None = None
    source_point_left_rdf_m: Vector3 | None = None
    source_point_on_left_ray_left_rdf_m: Vector3 | None = None
    source_point_on_right_ray_left_rdf_m: Vector3 | None = None
    source_midpoint_left_rdf_m: Vector3 | None = None
    source_range_left_rdf_m: float | None = None

    @property
    def point_left_m(self) -> Vector3:
        """Legacy common-frame midpoint alias; inspect ``point_frame``."""

        return self.point_left_rdf_m

    @property
    def point_m(self) -> Vector3:
        """Frame-neutral public midpoint in ``output_frame``."""

        return self.output_point_m if self.output_point_m is not None else self.point_left_rdf_m

    @property
    def point_on_left_ray_m(self) -> Vector3 | None:
        """Frame-neutral left-ray point in ``output_frame``."""

        if self.output_point_on_left_ray_m is not None:
            return self.output_point_on_left_ray_m
        return self.point_on_left_ray_rdf_m

    @property
    def point_on_right_ray_m(self) -> Vector3 | None:
        """Frame-neutral right-ray point in ``output_frame``."""

        if self.output_point_on_right_ray_m is not None:
            return self.output_point_on_right_ray_m
        return self.point_on_right_ray_rdf_m

    @property
    def midpoint_m(self) -> Vector3 | None:
        """Frame-neutral midpoint in ``output_frame``."""

        if self.output_midpoint_m is not None:
            return self.output_midpoint_m
        return self.midpoint_rdf_m

    @property
    def distance_m(self) -> float:
        """Compatibility alias for ``range_m``."""

        return self.range_m


@dataclass(frozen=True)
class RayTriangulationAttempt:
    result: RayTriangulationResult | None
    reason: str
    diagnostics: dict[str, Any] = field(default_factory=dict)

    @property
    def valid(self) -> bool:
        return self.result is not None


def _vector3(values: Sequence[float], *, name: str) -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{name} must contain three values")
    result = (float(values[0]), float(values[1]), float(values[2]))
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must be finite")
    return result


def _matrix3(values: Sequence[Sequence[float]], *, name: str) -> Matrix3:
    if len(values) != 3 or any(len(row) != 3 for row in values):
        raise ValueError(f"{name} must be a 3x3 matrix")
    return tuple(_vector3(row, name=name) for row in values)  # type: ignore[return-value]


def _dot(left: Vector3, right: Vector3) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _add(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def _scale(vector: Vector3, scalar: float) -> Vector3:
    return (vector[0] * scalar, vector[1] * scalar, vector[2] * scalar)


def _norm(vector: Vector3) -> float:
    return math.sqrt(_dot(vector, vector))


def normalize_vector(vector: Sequence[float]) -> Vector3:
    parsed = _vector3(vector, name="vector")
    norm = _norm(parsed)
    if norm <= 0.0:
        raise ValueError("vector must have non-zero length")
    return _scale(parsed, 1.0 / norm)


def baseline_center_frd_transform_from_left_rdf(
    *,
    right_origin_left_rdf_m: Sequence[float],
    left_forward_left_rdf: Sequence[float] = (0.0, 0.0, 1.0),
    right_forward_left_rdf: Sequence[float] = (0.0, 0.0, 1.0),
) -> BaselineCenterFrdTransform:
    """Build the Simulation-compatible stereo-baseline center FRD transform.

    The right axis is the normalized vector from the left optical center to
    the right optical center. The forward axis is the normalized average of
    the two optical forward axes after projection orthogonal to Right. The
    down axis is ``Forward x Right``. All inputs and axes are expressed in
    left-camera optical RDF.
    """

    right_origin = _vector3(right_origin_left_rdf_m, name="right_origin_left_rdf_m")
    right_axis = normalize_vector(right_origin)
    left_forward = normalize_vector(left_forward_left_rdf)
    right_forward = normalize_vector(right_forward_left_rdf)
    forward_candidate = normalize_vector(_add(left_forward, right_forward))
    forward_orthogonal = _subtract(
        forward_candidate,
        _scale(right_axis, _dot(forward_candidate, right_axis)),
    )
    forward_axis = normalize_vector(forward_orthogonal)
    down_axis = normalize_vector(
        (
            forward_axis[1] * right_axis[2] - forward_axis[2] * right_axis[1],
            forward_axis[2] * right_axis[0] - forward_axis[0] * right_axis[2],
            forward_axis[0] * right_axis[1] - forward_axis[1] * right_axis[0],
        )
    )
    return BaselineCenterFrdTransform(
        origin_left_rdf_m=_scale(right_origin, 0.5),
        forward_axis_left_rdf=forward_axis,
        right_axis_left_rdf=right_axis,
        down_axis_left_rdf=down_axis,
    )


def result_as_baseline_center_frd(
    result: RayTriangulationResult,
    *,
    right_origin_left_rdf_m: Sequence[float],
    left_forward_left_rdf: Sequence[float] = (0.0, 0.0, 1.0),
    right_forward_left_rdf: Sequence[float] = (0.0, 0.0, 1.0),
) -> RayTriangulationResult:
    """Convert a left-camera RDF triangulation result into baseline-center FRD."""

    if (
        result.point_frame != LEFT_CAMERA_RDF_FRAME
        or result.output_frame != LEFT_CAMERA_RDF_FRAME
        or result.axis_contract != LEFT_CAMERA_RDF_AXIS_CONTRACT
        or result.vector_contract != LEFT_CAMERA_RDF_VECTOR_CONTRACT
    ):
        raise ValueError("baseline-center conversion requires a left_camera_rdf source result")
    transform = baseline_center_frd_transform_from_left_rdf(
        right_origin_left_rdf_m=right_origin_left_rdf_m,
        left_forward_left_rdf=left_forward_left_rdf,
        right_forward_left_rdf=right_forward_left_rdf,
    )
    point = transform.point_from_left_rdf(result.point_left_rdf_m)
    point_on_left = (
        None
        if result.point_on_left_ray_rdf_m is None
        else transform.point_from_left_rdf(result.point_on_left_ray_rdf_m)
    )
    point_on_right = (
        None
        if result.point_on_right_ray_rdf_m is None
        else transform.point_from_left_rdf(result.point_on_right_ray_rdf_m)
    )
    midpoint = None if result.midpoint_rdf_m is None else transform.point_from_left_rdf(result.midpoint_rdf_m)
    return RayTriangulationResult(
        point_left_rdf_m=result.point_left_rdf_m,
        range_m=transform.range_from_left_rdf(result.point_left_rdf_m),
        closest_gap_m=result.closest_gap_m,
        left_depth_m=result.left_depth_m,
        right_depth_m=result.right_depth_m,
        ray_angle_deg=result.ray_angle_deg,
        baseline_m=result.baseline_m,
        point_on_left_ray_rdf_m=result.point_on_left_ray_rdf_m,
        point_on_right_ray_rdf_m=result.point_on_right_ray_rdf_m,
        midpoint_rdf_m=result.midpoint_rdf_m,
        geometry_rank=result.geometry_rank,
        condition_number=result.condition_number,
        computation_status=result.computation_status,
        method=result.method,
        point_frame=LEFT_CAMERA_RDF_FRAME,
        output_frame=transform.frame,
        axis_contract=LEFT_CAMERA_RDF_AXIS_CONTRACT,
        vector_contract=LEFT_CAMERA_RDF_VECTOR_CONTRACT,
        output_point_m=point,
        output_point_on_left_ray_m=point_on_left,
        output_point_on_right_ray_m=point_on_right,
        output_midpoint_m=midpoint,
        output_axis_contract=transform.axis_contract,
        output_vector_contract=transform.vector_contract,
        source_point_left_rdf_m=result.point_left_rdf_m,
        source_point_on_left_ray_left_rdf_m=result.point_on_left_ray_rdf_m,
        source_point_on_right_ray_left_rdf_m=result.point_on_right_ray_rdf_m,
        source_midpoint_left_rdf_m=result.midpoint_rdf_m,
        source_range_left_rdf_m=result.range_m,
    )


def rotate_vector(rotation_output_from_input: Sequence[Sequence[float]], vector: Sequence[float]) -> Vector3:
    """Rotate a direction using ``R_output_from_input``."""

    rotation = _matrix3(rotation_output_from_input, name="rotation_output_from_input")
    parsed = _vector3(vector, name="vector")
    return (
        _dot(rotation[0], parsed),
        _dot(rotation[1], parsed),
        _dot(rotation[2], parsed),
    )


def transpose_rotation(rotation_output_from_input: Sequence[Sequence[float]]) -> Matrix3:
    rotation = _matrix3(rotation_output_from_input, name="rotation_output_from_input")
    return tuple(
        tuple(rotation[row][column] for row in range(3)) for column in range(3)
    )  # type: ignore[return-value]


def ray_from_pixel(u_px: float, v_px: float, intrinsics: PinholeIntrinsics) -> Vector3:
    """Return a unit ray in the source camera's optical RDF frame."""

    u_px = float(u_px)
    v_px = float(v_px)
    if not math.isfinite(u_px) or not math.isfinite(v_px):
        raise ValueError("pixel coordinates must be finite")
    return normalize_vector(
        (
            (u_px - intrinsics.cx_px) / intrinsics.fx_px,
            (v_px - intrinsics.cy_px) / intrinsics.fy_px,
            1.0,
        )
    )


def triangulate_rays(
    *,
    right_origin_left_rdf_m: Sequence[float],
    left_ray_left_rdf: Sequence[float],
    right_ray_left_rdf: Sequence[float],
    left_origin_left_rdf_m: Sequence[float] = (0.0, 0.0, 0.0),
    parallel_epsilon: float = 1.0e-15,
    common_frame: str = LEFT_CAMERA_RDF_FRAME,
    axis_contract: str = LEFT_CAMERA_RDF_AXIS_CONTRACT,
    vector_contract: str = LEFT_CAMERA_RDF_VECTOR_CONTRACT,
    geometry_policy: str = GEOMETRY_POLICY_LEGACY_FORWARD_ONLY,
) -> RayTriangulationAttempt:
    """Find the closest-ray midpoint in the supplied common frame.

    The default left optical center is ``(0, 0, 0)`` for compatibility.  A
    non-zero left origin is accepted for fixed world/rig frames; no baseline
    center translation or FRD axis reorder is performed here. ``common_frame``
    records provenance only. Geometry validity is selected independently by
    ``geometry_policy``; the default preserves the original StereoDistance
    forward-only behavior.
    """

    resolved_common_frame = str(common_frame)
    if geometry_policy not in {
        GEOMETRY_POLICY_LEGACY_FORWARD_ONLY,
        GEOMETRY_POLICY_DIAGNOSTIC_SIGNED_DEPTH,
    }:
        return RayTriangulationAttempt(
            None,
            "invalid_geometry_policy",
            {"geometry_policy": str(geometry_policy)},
        )

    try:
        left_origin = _vector3(left_origin_left_rdf_m, name="left_origin_left_rdf_m")
        right_origin = _vector3(right_origin_left_rdf_m, name="right_origin_left_rdf_m")
        left_ray = normalize_vector(left_ray_left_rdf)
        right_ray = normalize_vector(right_ray_left_rdf)
    except (TypeError, ValueError) as exc:
        return RayTriangulationAttempt(None, "invalid_input", {"error": str(exc)})
    baseline_vector = _subtract(right_origin, left_origin)
    baseline_m = _norm(baseline_vector)
    if baseline_m <= 0.0:
        return RayTriangulationAttempt(None, "zero_baseline", {"geometry_rank": 0})

    ray_dot = max(-1.0, min(1.0, _dot(left_ray, right_ray)))
    cross = (
        left_ray[1] * right_ray[2] - left_ray[2] * right_ray[1],
        left_ray[2] * right_ray[0] - left_ray[0] * right_ray[2],
        left_ray[0] * right_ray[1] - left_ray[1] * right_ray[0],
    )
    cross_norm = _norm(cross)
    if not math.isfinite(cross_norm):
        return RayTriangulationAttempt(
            None,
            "invalid_solution",
            {"geometry_rank": 2, "condition_number": math.inf, "ray_dot": ray_dot},
        )
    if cross_norm <= float(parallel_epsilon):
        return RayTriangulationAttempt(
            None,
            "near_parallel_rays"
            if geometry_policy == GEOMETRY_POLICY_LEGACY_FORWARD_ONLY
            else "rank_deficient",
            {
                "geometry_rank": 1,
                "condition_number": math.inf,
                "ray_dot": ray_dot,
            },
        )

    try:
        denominator = cross_norm * cross_norm
        condition_number = (1.0 + abs(ray_dot)) / cross_norm
        origin_delta = _subtract(left_origin, right_origin)
        left_projection = _dot(left_ray, origin_delta)
        right_projection = _dot(right_ray, origin_delta)
        left_depth_m = (ray_dot * right_projection - left_projection) / denominator
        right_depth_m = (right_projection - ray_dot * left_projection) / denominator
        left_closest = _add(left_origin, _scale(left_ray, left_depth_m))
        right_closest = _add(right_origin, _scale(right_ray, right_depth_m))
        midpoint = _scale(_add(left_closest, right_closest), 0.5)
        range_m = _norm(midpoint)
        ray_angle_deg = math.degrees(math.atan2(cross_norm, ray_dot))
    except (ArithmeticError, OverflowError, ValueError) as exc:
        return RayTriangulationAttempt(
            None,
            "invalid_solution",
            {
                "geometry_rank": 2,
                "condition_number": math.inf,
                "ray_dot": ray_dot,
                "error": str(exc),
            },
        )
    if not all(math.isfinite(value) for value in (*left_closest, *right_closest, *midpoint, range_m)):
        return RayTriangulationAttempt(
            None,
            "invalid_solution",
            {"geometry_rank": 2, "condition_number": condition_number},
        )

    if (
        geometry_policy == GEOMETRY_POLICY_LEGACY_FORWARD_ONLY
        and (left_depth_m <= 0.0 or right_depth_m <= 0.0)
    ):
        return RayTriangulationAttempt(
            None,
            "non_positive_depth",
            {
                "geometry_rank": 2,
                "condition_number": condition_number,
                "ray_dot": ray_dot,
                "left_depth_m": left_depth_m,
                "right_depth_m": right_depth_m,
            },
        )

    result = RayTriangulationResult(
        point_left_rdf_m=midpoint,
        range_m=range_m,
        closest_gap_m=_norm(_subtract(left_closest, right_closest)),
        left_depth_m=left_depth_m,
        right_depth_m=right_depth_m,
        ray_angle_deg=ray_angle_deg,
        baseline_m=baseline_m,
        point_on_left_ray_rdf_m=left_closest,
        point_on_right_ray_rdf_m=right_closest,
        midpoint_rdf_m=midpoint,
        geometry_rank=2,
        condition_number=condition_number,
        computation_status="computed",
        point_frame=resolved_common_frame,
        output_frame=resolved_common_frame,
        axis_contract=str(axis_contract),
        vector_contract=str(vector_contract),
        output_axis_contract=str(axis_contract),
        output_vector_contract=str(vector_contract),
    )
    return RayTriangulationAttempt(
        result,
        "ok",
        {
            "geometry_rank": 2,
            "condition_number": condition_number,
            "ray_dot": ray_dot,
        },
    )


def triangulate_pixels(
    *,
    left_pixel: tuple[float, float],
    right_pixel: tuple[float, float],
    left_intrinsics: PinholeIntrinsics,
    right_intrinsics: PinholeIntrinsics,
    right_origin_left_rdf_m: Sequence[float],
    rotation_left_from_right_rdf: Sequence[Sequence[float]] = IDENTITY_ROTATION,
    left_origin_left_rdf_m: Sequence[float] = (0.0, 0.0, 0.0),
    common_frame: str = LEFT_CAMERA_RDF_FRAME,
    axis_contract: str = LEFT_CAMERA_RDF_AXIS_CONTRACT,
    vector_contract: str = LEFT_CAMERA_RDF_VECTOR_CONTRACT,
    geometry_policy: str = GEOMETRY_POLICY_LEGACY_FORWARD_ONLY,
) -> RayTriangulationAttempt:
    """Triangulate pinhole pixels after expressing the right ray in the left frame."""

    try:
        left_ray = ray_from_pixel(left_pixel[0], left_pixel[1], left_intrinsics)
        right_ray = rotate_vector(
            rotation_left_from_right_rdf,
            ray_from_pixel(right_pixel[0], right_pixel[1], right_intrinsics),
        )
    except (TypeError, ValueError) as exc:
        return RayTriangulationAttempt(None, "invalid_input", {"error": str(exc)})
    return triangulate_rays(
        right_origin_left_rdf_m=right_origin_left_rdf_m,
        left_ray_left_rdf=left_ray,
        right_ray_left_rdf=right_ray,
        left_origin_left_rdf_m=left_origin_left_rdf_m,
        common_frame=common_frame,
        axis_contract=axis_contract,
        vector_contract=vector_contract,
        geometry_policy=geometry_policy,
    )
