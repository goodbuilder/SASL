"""Reproducible optical-pose solvers over explicitly framed saved evidence.

Public PnP object points are local world NED ``[North, East, Down]`` metres.
Image observations are either raw camera pixels paired with their exact K/D,
or already-undistorted normalized ideal coordinates paired with implicit
``K=I,D=0``.  Those input states are mutually exclusive so a caller cannot
silently undistort twice.  Returned poses use :class:`CameraPoseNed`, whose
rotation maps OpenCV optical RDF ``[Right, Down, Forward]`` into local NED.

OpenCV DLS and UPNP are deliberately not exposed: OpenCV documents both as
falling back to EPNP.  Ground truth is not consumed by any runtime solver in
this module; callers may compare immutable results offline.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import inf, isfinite, sqrt
from threading import Lock
from typing import Any, Callable, Iterable, Sequence

from Workstation.workstation_core.optical_pose import (
    POSE_DOF_NAMES,
    CameraPoseNed,
    PoseDofMask,
    PoseParametersNed,
    camera_pose_ned_from_opencv_pnp,
    expand_selected_dofs,
    gravity_camera_rdf_from_normalized_horizon_line,
    horizon_roll_pitch_deg_from_gravity_camera_rdf,
    opencv_pnp_from_camera_pose_ned,
    pack_selected_dofs,
    pose_parameters_from_camera_pose,
)


Vector2 = tuple[float, float]
Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]

RAW_CAMERA_PIXELS = "raw_camera_pixels"
NORMALIZED_IDEAL = "normalized_ideal"
SUPPORTED_PNP_METHODS = ("ITERATIVE", "EPNP", "SQPNP")
SUPPORTED_REFINEMENTS = ("LM", "VVS")
PNP_RANSAC_SEED_SEMANTICS = "opencv_rng_reset_before_each_solver_under_process_lock"

_OPENCV_RNG_LOCK = Lock()


class PoseSolverDependencyError(RuntimeError):
    """Raised when an explicitly requested numerical backend is unavailable."""


def _require_numpy():
    try:
        import numpy as np
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise PoseSolverDependencyError("NumPy is required for optical pose solvers") from exc
    return np


def _require_cv2():
    try:
        import cv2
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise PoseSolverDependencyError("OpenCV calib3d is required for PnP operations") from exc
    return cv2


def _require_least_squares():
    try:
        from scipy.optimize import least_squares
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise PoseSolverDependencyError("SciPy is required for constrained least_squares") from exc
    return least_squares


def _finite_float(value: Any, *, name: str) -> float:
    parsed = float(value)
    if not isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _vector2(values: Iterable[Any], *, name: str) -> Vector2:
    parsed = tuple(_finite_float(value, name=name) for value in values)
    if len(parsed) != 2:
        raise ValueError(f"{name} must contain exactly two values")
    return parsed  # type: ignore[return-value]


def _vector3(values: Iterable[Any], *, name: str) -> Vector3:
    parsed = tuple(_finite_float(value, name=name) for value in values)
    if len(parsed) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    return parsed  # type: ignore[return-value]


def _matrix3(values: Iterable[Iterable[Any]], *, name: str) -> Matrix3:
    rows = tuple(_vector3(row, name=f"{name} row") for row in values)
    if len(rows) != 3:
        raise ValueError(f"{name} must be 3x3")
    return rows  # type: ignore[return-value]


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(a * b for a, b in zip(left, right))


def _norm(vector: Vector3) -> float:
    return sqrt(_dot(vector, vector))


def _normalize(vector: Iterable[Any], *, name: str) -> Vector3:
    parsed = _vector3(vector, name=name)
    magnitude = _norm(parsed)
    if magnitude <= 0.0:
        raise ValueError(f"{name} must have non-zero length")
    return tuple(value / magnitude for value in parsed)  # type: ignore[return-value]


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a - b for a, b in zip(left, right))  # type: ignore[return-value]


def _matvec(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(_dot(row, vector) for row in matrix)  # type: ignore[return-value]


def _percentile(sorted_values: Sequence[float], percentile: float) -> float:
    if not sorted_values:
        raise ValueError("percentile requires at least one value")
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    position = (len(sorted_values) - 1) * percentile
    lower = int(position)
    upper = min(lower + 1, len(sorted_values) - 1)
    fraction = position - lower
    return float(sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction)


@dataclass(frozen=True)
class PnpCorrespondences:
    """Deep-immutable 3D/2D PnP evidence with one unambiguous image space."""

    object_points_ned_m: tuple[Vector3, ...]
    image_points: tuple[Vector2, ...]
    image_space: str
    camera_matrix: Matrix3 | None = None
    distortion_coefficients: tuple[float, ...] = ()
    object_frame: str = "local_ned"
    camera_frame: str = "camera_optical_rdf"

    def __post_init__(self) -> None:
        objects = tuple(
            _vector3(point, name="object_points_ned_m") for point in self.object_points_ned_m
        )
        images = tuple(_vector2(point, name="image_points") for point in self.image_points)
        if len(objects) != len(images):
            raise ValueError("PnP object/image point counts must match")
        if len(objects) < 4:
            raise ValueError("PnP ensemble requires at least four correspondences")
        image_space = str(self.image_space)
        distortion = tuple(
            _finite_float(value, name="distortion_coefficients")
            for value in self.distortion_coefficients
        )
        camera_matrix = (
            None
            if self.camera_matrix is None
            else _matrix3(self.camera_matrix, name="camera_matrix")
        )
        if image_space == RAW_CAMERA_PIXELS:
            if camera_matrix is None:
                raise ValueError("raw camera pixels require their exact camera_matrix")
            if camera_matrix[0][0] <= 0.0 or camera_matrix[1][1] <= 0.0:
                raise ValueError("camera_matrix focal lengths must be positive")
            if len(distortion) not in (0, 4, 5, 8, 12, 14):
                raise ValueError("OpenCV distortion must contain 0,4,5,8,12,or 14 values")
        elif image_space == NORMALIZED_IDEAL:
            if camera_matrix is not None or distortion:
                raise ValueError(
                    "normalized ideal points use implicit K=I,D=0; providing K/D risks double-undistort"
                )
        else:
            raise ValueError(f"unsupported PnP image_space: {image_space}")
        if str(self.object_frame) != "local_ned":
            raise ValueError("PnP object points must be local_ned [North,East,Down] metres")
        if str(self.camera_frame) != "camera_optical_rdf":
            raise ValueError("PnP camera frame must be OpenCV camera_optical_rdf")
        object.__setattr__(self, "object_points_ned_m", objects)
        object.__setattr__(self, "image_points", images)
        object.__setattr__(self, "image_space", image_space)
        object.__setattr__(self, "camera_matrix", camera_matrix)
        object.__setattr__(self, "distortion_coefficients", distortion)

    @classmethod
    def from_raw_camera_pixels(
        cls,
        *,
        object_points_ned_m: Iterable[Iterable[Any]],
        image_points_px: Iterable[Iterable[Any]],
        camera_matrix: Iterable[Iterable[Any]],
        distortion_coefficients: Iterable[Any] = (),
    ) -> "PnpCorrespondences":
        return cls(
            object_points_ned_m=tuple(tuple(point) for point in object_points_ned_m),
            image_points=tuple(tuple(point) for point in image_points_px),
            image_space=RAW_CAMERA_PIXELS,
            camera_matrix=tuple(tuple(row) for row in camera_matrix),
            distortion_coefficients=tuple(distortion_coefficients),
        )

    @classmethod
    def from_normalized_ideal(
        cls,
        *,
        object_points_ned_m: Iterable[Iterable[Any]],
        image_points_normalized: Iterable[Iterable[Any]],
    ) -> "PnpCorrespondences":
        return cls(
            object_points_ned_m=tuple(tuple(point) for point in object_points_ned_m),
            image_points=tuple(tuple(point) for point in image_points_normalized),
            image_space=NORMALIZED_IDEAL,
        )

    @property
    def residual_unit(self) -> str:
        return "px" if self.image_space == RAW_CAMERA_PIXELS else "normalized"


@dataclass(frozen=True)
class ObjectPointGeometry:
    """Centered local-NED 3D point-spread diagnostics."""

    classification: str
    rank: int
    singular_values_m: tuple[float, ...]
    rank_tolerance_m: float
    condition_number: float
    point_count: int


def analyze_object_point_geometry(
    object_points_ned_m: Iterable[Iterable[Any]],
    *,
    relative_rank_tolerance: float = 1.0e-6,
) -> ObjectPointGeometry:
    """Classify coincident/near-collinear/planar/volumetric NED evidence."""

    np = _require_numpy()
    points = tuple(_vector3(point, name="object_points_ned_m") for point in object_points_ned_m)
    if len(points) < 2:
        raise ValueError("object geometry requires at least two points")
    tolerance_ratio = _finite_float(relative_rank_tolerance, name="relative_rank_tolerance")
    if not 0.0 < tolerance_ratio < 1.0:
        raise ValueError("relative_rank_tolerance must be in (0,1)")
    array = np.asarray(points, dtype=np.float64)
    centered = array - array.mean(axis=0, keepdims=True)
    singular_values = np.linalg.svd(centered, compute_uv=False)
    maximum = float(singular_values[0]) if len(singular_values) else 0.0
    tolerance = maximum * tolerance_ratio
    rank = int(np.sum(singular_values > tolerance)) if maximum > 0.0 else 0
    if rank == 0:
        classification = "coincident"
    elif rank == 1:
        classification = "near_collinear"
    elif rank == 2:
        classification = "planar"
    else:
        classification = "volumetric"
    condition = (
        float(singular_values[0] / singular_values[-1])
        if rank == 3 and float(singular_values[-1]) > 0.0
        else inf
    )
    return ObjectPointGeometry(
        classification=classification,
        rank=rank,
        singular_values_m=tuple(float(value) for value in singular_values),
        rank_tolerance_m=tolerance,
        condition_number=condition,
        point_count=len(points),
    )


@dataclass(frozen=True)
class JacobianDiagnostics:
    """SVD rank, scaled condition, nullspace and optional native-unit covariance."""

    parameter_names: tuple[str, ...]
    parameter_scales: tuple[float, ...]
    residual_count: int
    parameter_count: int
    rank: int
    singular_values: tuple[float, ...]
    rank_tolerance: float
    condition_number: float
    covariance: tuple[tuple[float, ...], ...] | None
    nullspace_basis: tuple[tuple[float, ...], ...]

    @property
    def full_rank(self) -> bool:
        return self.rank == self.parameter_count


def analyze_jacobian(
    jacobian: Iterable[Iterable[Any]],
    *,
    parameter_names: Iterable[str] | None = None,
    parameter_scales: Iterable[Any] | None = None,
    residual_variance: float | None = None,
    relative_rank_tolerance: float = 1.0e-8,
) -> JacobianDiagnostics:
    """Analyze ``dr/dx`` after scaling ``dx = diag(scales)*d(x_scaled)``.

    Covariance is returned in the native parameter units only when the scaled
    Jacobian has full column rank and ``residual_variance`` is supplied.
    Rank-deficient covariance is ``None`` rather than a misleading pseudo-
    covariance; ``nullspace_basis`` exposes the weak scaled directions.
    """

    np = _require_numpy()
    rows = tuple(tuple(_finite_float(value, name="jacobian") for value in row) for row in jacobian)
    if not rows:
        raise ValueError("jacobian must contain at least one residual row")
    parameter_count = len(rows[0])
    if any(len(row) != parameter_count for row in rows):
        raise ValueError("jacobian rows must have equal length")
    names = (
        tuple(f"parameter_{index}" for index in range(parameter_count))
        if parameter_names is None
        else tuple(str(name) for name in parameter_names)
    )
    if len(names) != parameter_count or len(names) != len(set(names)) or any(not name for name in names):
        raise ValueError("parameter_names must be unique and match Jacobian columns")
    scales = (
        tuple(1.0 for _index in range(parameter_count))
        if parameter_scales is None
        else tuple(_finite_float(value, name="parameter_scales") for value in parameter_scales)
    )
    if len(scales) != parameter_count or any(value <= 0.0 for value in scales):
        raise ValueError("parameter_scales must be positive and match Jacobian columns")
    relative_tolerance = _finite_float(relative_rank_tolerance, name="relative_rank_tolerance")
    if not 0.0 < relative_tolerance < 1.0:
        raise ValueError("relative_rank_tolerance must be in (0,1)")
    variance = (
        None
        if residual_variance is None
        else _finite_float(residual_variance, name="residual_variance")
    )
    if variance is not None and variance < 0.0:
        raise ValueError("residual_variance must be non-negative")

    matrix = np.asarray(rows, dtype=np.float64).reshape(len(rows), parameter_count)
    if parameter_count == 0:
        return JacobianDiagnostics(
            parameter_names=names,
            parameter_scales=scales,
            residual_count=len(rows),
            parameter_count=0,
            rank=0,
            singular_values=(),
            rank_tolerance=0.0,
            condition_number=1.0,
            covariance=() if variance is not None else None,
            nullspace_basis=(),
        )
    scaled = matrix * np.asarray(scales, dtype=np.float64)[None, :]
    _u, singular_values, vh = np.linalg.svd(scaled, full_matrices=True)
    maximum = float(singular_values[0]) if len(singular_values) else 0.0
    numerical_tolerance = np.finfo(np.float64).eps * max(scaled.shape) * maximum
    tolerance = max(relative_tolerance * maximum, numerical_tolerance)
    rank = int(np.sum(singular_values > tolerance))
    condition = (
        float(singular_values[0] / singular_values[-1])
        if rank == parameter_count and float(singular_values[-1]) > 0.0
        else inf
    )
    nullspace = tuple(
        tuple(float(value) for value in row)
        for row in vh[rank:parameter_count]
    )
    covariance = None
    if variance is not None and rank == parameter_count:
        covariance_scaled = variance * np.linalg.inv(scaled.T @ scaled)
        scale_matrix = np.diag(np.asarray(scales, dtype=np.float64))
        covariance_native = scale_matrix @ covariance_scaled @ scale_matrix
        covariance = tuple(
            tuple(float(value) for value in row) for row in covariance_native
        )
    return JacobianDiagnostics(
        parameter_names=names,
        parameter_scales=scales,
        residual_count=len(rows),
        parameter_count=parameter_count,
        rank=rank,
        singular_values=tuple(float(value) for value in singular_values),
        rank_tolerance=tolerance,
        condition_number=condition,
        covariance=covariance,
        nullspace_basis=nullspace,
    )


@dataclass(frozen=True)
class HorizonLineFit:
    """Total-least-squares line fit in normalized ideal image coordinates."""

    normalized_line: Vector3
    gravity_camera_rdf: Vector3
    roll_deg: float
    pitch_deg: float
    rms_normalized: float
    max_abs_normalized: float
    point_count: int
    singular_values: tuple[float, ...]


def fit_normalized_horizon_line(
    points_normalized_ideal: Iterable[Iterable[Any]],
    *,
    expected_gravity_camera_rdf: Iterable[Any] | None = None,
) -> HorizonLineFit:
    """Fit ``a*x+b*y+c=0`` after exactly one undistortion to normalized space.

    This flat-horizon line observes roll/pitch but never yaw.  A real Earth
    horizon dip must be handled separately; terrain skylines are not valid
    inputs unless deliberately treated as relative visual evidence.
    """

    np = _require_numpy()
    points = tuple(_vector2(point, name="points_normalized_ideal") for point in points_normalized_ideal)
    if len(points) < 3:
        raise ValueError("horizon line fitting requires at least three normalized points")
    design = np.column_stack(
        (np.asarray(points, dtype=np.float64), np.ones(len(points), dtype=np.float64))
    )
    _u, singular_values, vh = np.linalg.svd(design, full_matrices=False)
    maximum = float(singular_values[0])
    rank = int(np.sum(singular_values > maximum * 1.0e-12)) if maximum > 0.0 else 0
    if rank < 2:
        raise ValueError("horizon points are coincident/degenerate")
    raw_line = tuple(float(value) for value in vh[-1])
    gravity = gravity_camera_rdf_from_normalized_horizon_line(
        raw_line,
        expected_gravity_camera_rdf=expected_gravity_camera_rdf,
    )
    line = gravity
    planar_normal = sqrt(line[0] * line[0] + line[1] * line[1])
    if planar_normal <= 1.0e-15:
        raise ValueError("horizon line has no finite image-plane normal")
    signed_distances = [
        (line[0] * x + line[1] * y + line[2]) / planar_normal for x, y in points
    ]
    roll, pitch = horizon_roll_pitch_deg_from_gravity_camera_rdf(gravity)
    return HorizonLineFit(
        normalized_line=line,
        gravity_camera_rdf=gravity,
        roll_deg=roll,
        pitch_deg=pitch,
        rms_normalized=sqrt(sum(value * value for value in signed_distances) / len(points)),
        max_abs_normalized=max(abs(value) for value in signed_distances),
        point_count=len(points),
        singular_values=tuple(float(value) for value in singular_values),
    )


def fixed_center_orientation_ray_residuals(
    roll_pitch_yaw_deg: Iterable[Any],
    *,
    camera_center_ned_m: Iterable[Any],
    observed_rays_camera_rdf: Iterable[Iterable[Any]],
    target_points_ned_m: Iterable[Iterable[Any]],
    weights: Iterable[Any] | None = None,
) -> tuple[float, ...]:
    """Return 3D unit-chord residuals for a fixed NED optical center.

    For each correspondence, the predicted optical ray rotated into NED is
    subtracted from the normalized camera-center-to-target NED LOS.  The three
    components retain two local angular constraints and distinguish parallel
    from anti-parallel rays.  Translation is not a variable in this function.
    """

    rpy = _vector3(roll_pitch_yaw_deg, name="roll_pitch_yaw_deg")
    center = _vector3(camera_center_ned_m, name="camera_center_ned_m")
    rays = tuple(_normalize(ray, name="observed_rays_camera_rdf") for ray in observed_rays_camera_rdf)
    targets = tuple(_vector3(point, name="target_points_ned_m") for point in target_points_ned_m)
    if not rays or len(rays) != len(targets):
        raise ValueError("orientation ray/target correspondences must be non-empty and matched")
    resolved_weights = (
        tuple(1.0 for _ray in rays)
        if weights is None
        else tuple(_finite_float(value, name="weights") for value in weights)
    )
    if len(resolved_weights) != len(rays) or any(value <= 0.0 for value in resolved_weights):
        raise ValueError("weights must be positive and match ray count")
    pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=center,
        roll_deg=rpy[0],
        pitch_deg=rpy[1],
        yaw_deg=rpy[2],
    )
    residuals: list[float] = []
    for ray, target, weight in zip(rays, targets, resolved_weights):
        expected = _normalize(_subtract(target, center), name="target_los_ned")
        predicted = _normalize(
            _matvec(pose.rotation_ned_from_camera_rdf, ray),
            name="predicted_ray_ned",
        )
        scale = sqrt(weight)
        residuals.extend((predicted[index] - expected[index]) * scale for index in range(3))
    return tuple(residuals)


def _opencv_correspondence_arrays(correspondences: PnpCorrespondences):
    np = _require_numpy()
    objects = np.asarray(correspondences.object_points_ned_m, dtype=np.float64)
    images = np.asarray(correspondences.image_points, dtype=np.float64)
    if correspondences.image_space == NORMALIZED_IDEAL:
        camera_matrix = np.eye(3, dtype=np.float64)
        distortion = None
    else:
        camera_matrix = np.asarray(correspondences.camera_matrix, dtype=np.float64)
        distortion = (
            None
            if not correspondences.distortion_coefficients
            else np.asarray(correspondences.distortion_coefficients, dtype=np.float64)
        )
    return objects, images, camera_matrix, distortion


def pose_reprojection_residuals(
    parameters: PoseParametersNed,
    correspondences: PnpCorrespondences,
) -> tuple[float, ...]:
    """Project a local-NED pose into the correspondence's single declared grid."""

    cv2 = _require_cv2()
    np = _require_numpy()
    objects, images, camera_matrix, distortion = _opencv_correspondence_arrays(correspondences)
    pose = parameters.to_camera_pose_ned()
    rotation, translation = opencv_pnp_from_camera_pose_ned(pose)
    rvec, _jacobian = cv2.Rodrigues(np.asarray(rotation, dtype=np.float64))
    projected, _projection_jacobian = cv2.projectPoints(
        objects,
        rvec,
        np.asarray(translation, dtype=np.float64).reshape(3, 1),
        camera_matrix,
        distortion,
    )
    delta = projected.reshape(-1, 2) - images
    return tuple(float(value) for value in delta.reshape(-1))


def numerical_pose_jacobian(
    parameters: PoseParametersNed,
    mask: PoseDofMask,
    residual_function: Callable[[PoseParametersNed], Sequence[Any]],
    *,
    step_by_dof: Iterable[Sequence[Any]] = (
        ("roll_deg", 1.0e-4),
        ("pitch_deg", 1.0e-4),
        ("yaw_deg", 1.0e-4),
        ("north_m", 1.0e-4),
        ("east_m", 1.0e-4),
        ("down_m", 1.0e-4),
    ),
) -> tuple[tuple[float, ...], ...]:
    """Central-difference residual Jacobian for selected native degree/metre DOFs."""

    steps: dict[str, float] = {}
    for item in step_by_dof:
        if len(item) != 2:
            raise ValueError("each step_by_dof item must have two values")
        name, value = str(item[0]), _finite_float(item[1], name="step_by_dof")
        if name not in POSE_DOF_NAMES or name in steps or value <= 0.0:
            raise ValueError("step_by_dof names must be unique known DOFs with positive steps")
        steps[name] = value
    if set(steps) != set(POSE_DOF_NAMES):
        raise ValueError("step_by_dof must include all six pose DOFs")
    baseline = tuple(
        _finite_float(value, name="residual_function") for value in residual_function(parameters)
    )
    if not baseline:
        raise ValueError("residual_function must return at least one residual")
    selected_names = mask.selected_names
    if not selected_names:
        return tuple(() for _value in baseline)
    baseline_selected = pack_selected_dofs(parameters, mask)
    columns: list[tuple[float, ...]] = []
    for column, name in enumerate(selected_names):
        step = steps[name]
        plus_values = list(baseline_selected)
        minus_values = list(baseline_selected)
        plus_values[column] += step
        minus_values[column] -= step
        plus = tuple(
            _finite_float(value, name="residual_function")
            for value in residual_function(expand_selected_dofs(parameters, mask, plus_values))
        )
        minus = tuple(
            _finite_float(value, name="residual_function")
            for value in residual_function(expand_selected_dofs(parameters, mask, minus_values))
        )
        if len(plus) != len(baseline) or len(minus) != len(baseline):
            raise ValueError("residual_function length must remain constant")
        columns.append(tuple((a - b) / (2.0 * step) for a, b in zip(plus, minus)))
    return tuple(
        tuple(columns[column][row] for column in range(len(columns)))
        for row in range(len(baseline))
    )


@dataclass(frozen=True)
class PoseLeastSquaresConfig:
    method: str = "trf"
    loss: str = "linear"
    f_scale: float = 1.0
    ftol: float = 1.0e-10
    xtol: float = 1.0e-10
    gtol: float = 1.0e-10
    max_nfev: int = 1000
    relative_rank_tolerance: float = 1.0e-8

    def __post_init__(self) -> None:
        if str(self.method) not in {"trf", "dogbox", "lm"}:
            raise ValueError("least_squares method must be trf, dogbox, or lm")
        if str(self.loss) not in {"linear", "soft_l1", "huber", "cauchy", "arctan"}:
            raise ValueError("unsupported least_squares loss")
        for name in ("f_scale", "ftol", "xtol", "gtol", "relative_rank_tolerance"):
            value = _finite_float(getattr(self, name), name=name)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)
        if int(self.max_nfev) <= 0:
            raise ValueError("max_nfev must be positive")
        object.__setattr__(self, "method", str(self.method))
        object.__setattr__(self, "loss", str(self.loss))
        object.__setattr__(self, "max_nfev", int(self.max_nfev))


@dataclass(frozen=True)
class ConstrainedPoseLeastSquaresResult:
    parameters: PoseParametersNed
    selected_dofs: tuple[str, ...]
    success: bool
    status: int
    message: str
    cost: float
    optimality: float
    nfev: int
    njev: int | None
    residuals: tuple[float, ...]
    jacobian: tuple[tuple[float, ...], ...]
    jacobian_diagnostics: JacobianDiagnostics


def solve_constrained_pose_least_squares(
    initial_parameters: PoseParametersNed,
    mask: PoseDofMask,
    residual_function: Callable[[PoseParametersNed], Sequence[Any]],
    *,
    parameter_scales: Iterable[Any] | None = None,
    config: PoseLeastSquaresConfig = PoseLeastSquaresConfig(),
) -> ConstrainedPoseLeastSquaresResult:
    """Optimize checked DOFs only; unchecked RPY/NED scalars remain bit-identical."""

    np = _require_numpy()
    selected_names = mask.selected_names
    scales = (
        tuple(1.0 for _name in selected_names)
        if parameter_scales is None
        else tuple(_finite_float(value, name="parameter_scales") for value in parameter_scales)
    )
    if len(scales) != len(selected_names) or any(value <= 0.0 for value in scales):
        raise ValueError("parameter_scales must be positive and match selected DOFs")

    def evaluate(selected_values: Sequence[Any]) -> Any:
        parameters = expand_selected_dofs(initial_parameters, mask, selected_values)
        residuals = tuple(
            _finite_float(value, name="residual_function") for value in residual_function(parameters)
        )
        if not residuals:
            raise ValueError("residual_function must return at least one residual")
        return np.asarray(residuals, dtype=np.float64)

    x0 = pack_selected_dofs(initial_parameters, mask)
    initial_residuals = evaluate(x0)
    if not selected_names:
        jacobian = tuple(() for _value in initial_residuals)
        diagnostics = analyze_jacobian(
            jacobian,
            parameter_names=(),
            parameter_scales=(),
            residual_variance=None,
            relative_rank_tolerance=config.relative_rank_tolerance,
        )
        return ConstrainedPoseLeastSquaresResult(
            parameters=initial_parameters,
            selected_dofs=(),
            success=True,
            status=0,
            message="no selected DOFs; residual evaluated without optimization",
            cost=0.5 * float(initial_residuals @ initial_residuals),
            optimality=0.0,
            nfev=1,
            njev=0,
            residuals=tuple(float(value) for value in initial_residuals),
            jacobian=jacobian,
            jacobian_diagnostics=diagnostics,
        )

    least_squares = _require_least_squares()
    result = least_squares(
        evaluate,
        np.asarray(x0, dtype=np.float64),
        method=config.method,
        loss=config.loss,
        f_scale=config.f_scale,
        ftol=config.ftol,
        xtol=config.xtol,
        gtol=config.gtol,
        max_nfev=config.max_nfev,
        x_scale=np.asarray(scales, dtype=np.float64),
    )
    parameters = expand_selected_dofs(initial_parameters, mask, result.x)
    residuals = tuple(float(value) for value in result.fun)
    jacobian_array = np.asarray(result.jac, dtype=np.float64)
    jacobian = tuple(tuple(float(value) for value in row) for row in jacobian_array)
    preliminary = analyze_jacobian(
        jacobian,
        parameter_names=selected_names,
        parameter_scales=scales,
        relative_rank_tolerance=config.relative_rank_tolerance,
    )
    residual_degrees_of_freedom = len(residuals) - preliminary.rank
    variance = (
        2.0 * float(result.cost) / residual_degrees_of_freedom
        if residual_degrees_of_freedom > 0
        else None
    )
    diagnostics = analyze_jacobian(
        jacobian,
        parameter_names=selected_names,
        parameter_scales=scales,
        residual_variance=variance,
        relative_rank_tolerance=config.relative_rank_tolerance,
    )
    return ConstrainedPoseLeastSquaresResult(
        parameters=parameters,
        selected_dofs=selected_names,
        success=bool(result.success),
        status=int(result.status),
        message=str(result.message),
        cost=float(result.cost),
        optimality=float(result.optimality),
        nfev=int(result.nfev),
        njev=None if result.njev is None else int(result.njev),
        residuals=residuals,
        jacobian=jacobian,
        jacobian_diagnostics=diagnostics,
    )


@dataclass(frozen=True)
class PnpEnsembleConfig:
    methods: tuple[str, ...] = SUPPORTED_PNP_METHODS
    refinements: tuple[str, ...] = SUPPORTED_REFINEMENTS
    use_ransac: bool = True
    ransac_seed: int = 20260813
    ransac_iterations: int = 1000
    ransac_reprojection_threshold_px: float = 4.0
    ransac_reprojection_threshold_normalized: float = 0.002
    ransac_confidence: float = 0.999
    object_rank_relative_tolerance: float = 1.0e-6
    jacobian_rank_relative_tolerance: float = 1.0e-8
    jacobian_parameter_scales: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

    def __post_init__(self) -> None:
        methods = tuple(str(value).upper() for value in self.methods)
        refinements = tuple(str(value).upper() for value in self.refinements)
        if not methods or len(methods) != len(set(methods)):
            raise ValueError("PnP methods must be non-empty and unique")
        unsupported = set(methods) - set(SUPPORTED_PNP_METHODS)
        if unsupported:
            raise ValueError(
                f"unsupported PnP methods {sorted(unsupported)}; DLS/UPNP are intentionally not exposed"
            )
        if len(refinements) != len(set(refinements)) or set(refinements) - set(SUPPORTED_REFINEMENTS):
            raise ValueError("PnP refinements must be unique LM/VVS values")
        if type(self.use_ransac) is not bool:
            raise TypeError("use_ransac must be bool")
        seed = int(self.ransac_seed)
        if not 0 <= seed <= 2_147_483_647:
            raise ValueError("ransac_seed must fit a non-negative signed 32-bit integer")
        if int(self.ransac_iterations) <= 0:
            raise ValueError("ransac_iterations must be positive")
        for name in (
            "ransac_reprojection_threshold_px",
            "ransac_reprojection_threshold_normalized",
            "object_rank_relative_tolerance",
            "jacobian_rank_relative_tolerance",
        ):
            value = _finite_float(getattr(self, name), name=name)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)
        confidence = _finite_float(self.ransac_confidence, name="ransac_confidence")
        if not 0.0 < confidence < 1.0:
            raise ValueError("ransac_confidence must be in (0,1)")
        scales = tuple(
            _finite_float(value, name="jacobian_parameter_scales")
            for value in self.jacobian_parameter_scales
        )
        if len(scales) != 6 or any(value <= 0.0 for value in scales):
            raise ValueError("jacobian_parameter_scales must contain six positive values")
        object.__setattr__(self, "methods", methods)
        object.__setattr__(self, "refinements", refinements)
        object.__setattr__(self, "ransac_seed", seed)
        object.__setattr__(self, "ransac_iterations", int(self.ransac_iterations))
        object.__setattr__(self, "ransac_confidence", confidence)
        object.__setattr__(self, "jacobian_parameter_scales", scales)


@dataclass(frozen=True)
class KnownBaselineRotationResult:
    """Relative rotation fitted from paired bearings and a known baseline.

    The baseline is the right-camera origin minus the left-camera origin in
    the left optical RDF frame.  The residual is the normalized epipolar
    coplanarity error; no terrain or 3-D object points are consumed.
    """

    rotation_right_from_left_rdf: Matrix3
    residuals: tuple[float, ...]
    jacobian_rank: int
    jacobian_condition: float
    iterations: int


def solve_known_baseline_rotation(
    *,
    left_bearings_rdf: Iterable[Iterable[Any]],
    right_bearings_rdf: Iterable[Iterable[Any]],
    baseline_right_minus_left_left_rdf_m: Iterable[Any],
    prior_rotation_right_from_left_rdf: Iterable[Iterable[Any]],
) -> KnownBaselineRotationResult:
    """Fit stereo rotation from paired normalized pixels and a known baseline.

    For a point pair ``x_l, x_r`` and left-frame baseline ``b``, the
    coplanarity constraint is ``x_rᵀ R [b]× x_l = 0``.  The known GNSS
    baseline supplies metric translation; the mountain clicks supply only
    same-order left/right ``(u,v)`` bearings.
    """

    cv2 = _require_cv2()
    np = _require_numpy()
    least_squares = _require_least_squares()

    left = np.asarray(
        tuple(_vector3(value, name="left_bearings_rdf") for value in left_bearings_rdf),
        dtype=np.float64,
    )
    right = np.asarray(
        tuple(_vector3(value, name="right_bearings_rdf") for value in right_bearings_rdf),
        dtype=np.float64,
    )
    if left.shape != right.shape or left.ndim != 2 or left.shape[1] != 3:
        raise ValueError("left/right bearing arrays must have equal shape (N,3)")
    if left.shape[0] < 4:
        raise ValueError("GNSS + Mountain requires at least four paired u,v observations")
    left_norms = np.linalg.norm(left, axis=1)
    right_norms = np.linalg.norm(right, axis=1)
    if np.any(left_norms <= 0.0) or np.any(right_norms <= 0.0):
        raise ValueError("bearing vectors must have non-zero length")
    left = left / left_norms[:, None]
    right = right / right_norms[:, None]
    baseline = np.asarray(
        _vector3(
            baseline_right_minus_left_left_rdf_m,
            name="baseline_right_minus_left_left_rdf_m",
        ),
        dtype=np.float64,
    )
    if float(np.linalg.norm(baseline)) <= 1.0e-9:
        raise ValueError("known baseline must be non-zero")
    prior = np.asarray(
        _matrix3(
            prior_rotation_right_from_left_rdf,
            name="prior_rotation_right_from_left_rdf",
        ),
        dtype=np.float64,
    )
    prior_rvec, _ = cv2.Rodrigues(prior)
    cross_terms = np.cross(np.broadcast_to(baseline, left.shape), left)

    def residuals(rvec: Any) -> Any:
        rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
        rotated = (rotation @ cross_terms.T).T
        return np.einsum("ij,ij->i", right, rotated)

    result = least_squares(
        residuals,
        prior_rvec.reshape(3),
        method="lm" if left.shape[0] >= 3 else "trf",
        max_nfev=2000,
        xtol=1.0e-13,
        ftol=1.0e-13,
        gtol=1.0e-13,
    )
    if not np.all(np.isfinite(result.x)) or not np.all(np.isfinite(result.fun)):
        raise ValueError("known-baseline rotation solve returned non-finite values")
    rotation, _ = cv2.Rodrigues(np.asarray(result.x, dtype=np.float64).reshape(3, 1))
    singular_values = np.linalg.svd(
        np.asarray(result.jac, dtype=np.float64),
        compute_uv=False,
    )
    maximum = float(singular_values[0]) if len(singular_values) else 0.0
    tolerance = max(1.0e-12, maximum * 1.0e-8)
    rank = int(np.sum(singular_values > tolerance))
    condition = (
        float(singular_values[0] / singular_values[-1])
        if len(singular_values) and float(singular_values[-1]) > tolerance
        else inf
    )
    return KnownBaselineRotationResult(
        rotation_right_from_left_rdf=tuple(
            tuple(float(value) for value in row) for row in rotation
        ),
        residuals=tuple(float(value) for value in result.fun),
        jacobian_rank=rank,
        jacobian_condition=condition,
        iterations=int(getattr(result, "nfev", 0)),
    )


@dataclass(frozen=True)
class PnpPoseCandidate:
    candidate_id: str
    solver: str
    refinement: str
    pose_ned: CameraPoseNed
    inlier_indices: tuple[int, ...]
    reprojection_rmse: float
    reprojection_median: float
    reprojection_p95: float
    residual_unit: str
    positive_depth_ratio: float
    jacobian_diagnostics: JacobianDiagnostics
    ransac_seed: int | None
    seed_semantics: str


@dataclass(frozen=True)
class PnpEnsembleResult:
    candidates: tuple[PnpPoseCandidate, ...]
    failures: tuple[tuple[str, str], ...]
    object_geometry: ObjectPointGeometry
    image_space: str
    ransac_seed: int | None
    seed_semantics: str

    @property
    def best_candidate(self) -> PnpPoseCandidate | None:
        return min(
            self.candidates,
            key=lambda candidate: (
                candidate.reprojection_p95,
                candidate.reprojection_rmse,
                candidate.candidate_id,
            ),
            default=None,
        )


def _pnp_flag(cv2: Any, method: str) -> int:
    attribute = f"SOLVEPNP_{method}"
    if not hasattr(cv2, attribute):
        raise PoseSolverDependencyError(f"installed OpenCV does not provide {attribute}")
    return int(getattr(cv2, attribute))


def _full_pose_mask() -> PoseDofMask:
    return PoseDofMask.from_names(POSE_DOF_NAMES)


def _pnp_candidate_from_rt(
    *,
    solver: str,
    refinement: str,
    rotation_vector: Any,
    translation_vector: Any,
    inlier_indices: tuple[int, ...],
    correspondences: PnpCorrespondences,
    config: PnpEnsembleConfig,
) -> PnpPoseCandidate:
    cv2 = _require_cv2()
    np = _require_numpy()
    objects, images, camera_matrix, distortion = _opencv_correspondence_arrays(correspondences)
    rotation, _jacobian = cv2.Rodrigues(np.asarray(rotation_vector, dtype=np.float64).reshape(3, 1))
    translation = np.asarray(translation_vector, dtype=np.float64).reshape(3, 1)
    pose = camera_pose_ned_from_opencv_pnp(rotation, translation)
    projected, _projection_jacobian = cv2.projectPoints(
        objects,
        np.asarray(rotation_vector, dtype=np.float64).reshape(3, 1),
        translation,
        camera_matrix,
        distortion,
    )
    deltas = projected.reshape(-1, 2) - images
    errors = sorted(float(value) for value in np.linalg.norm(deltas, axis=1))
    camera_points = (rotation @ objects.T + translation).T
    positive_depth_ratio = float(np.mean(camera_points[:, 2] > 0.0))
    parameters = pose_parameters_from_camera_pose(pose)
    residual_function = lambda value: pose_reprojection_residuals(value, correspondences)
    jacobian = numerical_pose_jacobian(parameters, _full_pose_mask(), residual_function)
    residual_vector = residual_function(parameters)
    preliminary = analyze_jacobian(
        jacobian,
        parameter_names=POSE_DOF_NAMES,
        parameter_scales=config.jacobian_parameter_scales,
        relative_rank_tolerance=config.jacobian_rank_relative_tolerance,
    )
    residual_dof = len(residual_vector) - preliminary.rank
    variance = (
        sum(value * value for value in residual_vector) / residual_dof
        if residual_dof > 0
        else None
    )
    diagnostics = analyze_jacobian(
        jacobian,
        parameter_names=POSE_DOF_NAMES,
        parameter_scales=config.jacobian_parameter_scales,
        residual_variance=variance,
        relative_rank_tolerance=config.jacobian_rank_relative_tolerance,
    )
    return PnpPoseCandidate(
        candidate_id=f"{solver}/{refinement}",
        solver=solver,
        refinement=refinement,
        pose_ned=pose,
        inlier_indices=inlier_indices,
        reprojection_rmse=sqrt(sum(value * value for value in errors) / len(errors)),
        reprojection_median=_percentile(errors, 0.5),
        reprojection_p95=_percentile(errors, 0.95),
        residual_unit=correspondences.residual_unit,
        positive_depth_ratio=positive_depth_ratio,
        jacobian_diagnostics=diagnostics,
        ransac_seed=config.ransac_seed if config.use_ransac else None,
        seed_semantics=PNP_RANSAC_SEED_SEMANTICS if config.use_ransac else "not_used",
    )


def solve_pnp_ensemble(
    correspondences: PnpCorrespondences,
    *,
    config: PnpEnsembleConfig = PnpEnsembleConfig(),
) -> PnpEnsembleResult:
    """Run reproducible ITERATIVE/EPNP/SQPNP candidates and LM/VVS refinements."""

    cv2 = _require_cv2()
    np = _require_numpy()
    objects, images, camera_matrix, distortion = _opencv_correspondence_arrays(correspondences)
    geometry = analyze_object_point_geometry(
        correspondences.object_points_ned_m,
        relative_rank_tolerance=config.object_rank_relative_tolerance,
    )
    threshold = (
        config.ransac_reprojection_threshold_px
        if correspondences.image_space == RAW_CAMERA_PIXELS
        else config.ransac_reprojection_threshold_normalized
    )
    candidates: list[PnpPoseCandidate] = []
    failures: list[tuple[str, str]] = []
    for method in config.methods:
        try:
            flag = _pnp_flag(cv2, method)
            if config.use_ransac:
                with _OPENCV_RNG_LOCK:
                    cv2.setRNGSeed(config.ransac_seed)
                    ok, rvec, tvec, raw_inliers = cv2.solvePnPRansac(
                        objects,
                        images,
                        camera_matrix,
                        distortion,
                        iterationsCount=config.ransac_iterations,
                        reprojectionError=threshold,
                        confidence=config.ransac_confidence,
                        flags=flag,
                    )
                inliers = (
                    ()
                    if raw_inliers is None
                    else tuple(int(value) for value in np.asarray(raw_inliers).reshape(-1))
                )
            else:
                ok, rvec, tvec = cv2.solvePnP(
                    objects,
                    images,
                    camera_matrix,
                    distortion,
                    flags=flag,
                )
                inliers = tuple(range(len(objects)))
            if not ok or len(inliers) < 4:
                raise ValueError(f"solver returned ok={ok} with {len(inliers)} inliers")
            candidates.append(
                _pnp_candidate_from_rt(
                    solver=method,
                    refinement="NONE",
                    rotation_vector=rvec,
                    translation_vector=tvec,
                    inlier_indices=inliers,
                    correspondences=correspondences,
                    config=config,
                )
            )
        except Exception as exc:
            failures.append((f"{method}/NONE", str(exc)))
            continue

        inlier_array = np.asarray(inliers, dtype=np.int32)
        refine_objects = objects[inlier_array]
        refine_images = images[inlier_array]
        for refinement in config.refinements:
            try:
                function_name = f"solvePnPRefine{refinement}"
                if not hasattr(cv2, function_name):
                    raise PoseSolverDependencyError(
                        f"installed OpenCV does not provide {function_name}"
                    )
                refine_function = getattr(cv2, function_name)
                refined_rvec, refined_tvec = refine_function(
                    refine_objects,
                    refine_images,
                    camera_matrix,
                    distortion,
                    np.asarray(rvec, dtype=np.float64).reshape(3, 1).copy(),
                    np.asarray(tvec, dtype=np.float64).reshape(3, 1).copy(),
                )
                candidates.append(
                    _pnp_candidate_from_rt(
                        solver=method,
                        refinement=refinement,
                        rotation_vector=refined_rvec,
                        translation_vector=refined_tvec,
                        inlier_indices=inliers,
                        correspondences=correspondences,
                        config=config,
                    )
                )
            except Exception as exc:
                failures.append((f"{method}/{refinement}", str(exc)))
    return PnpEnsembleResult(
        candidates=tuple(candidates),
        failures=tuple(failures),
        object_geometry=geometry,
        image_space=correspondences.image_space,
        ransac_seed=config.ransac_seed if config.use_ransac else None,
        seed_semantics=PNP_RANSAC_SEED_SEMANTICS if config.use_ransac else "not_used",
    )
