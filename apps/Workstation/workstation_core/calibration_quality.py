"""Pure checkerboard-view geometry and camera-calibration quality gates.

This module intentionally has no OpenCV or NumPy dependency.  Callers pass
refined checkerboard corners in row-major order and the decoded raster size.
The resulting frozen contracts contain only strings, numbers, tuples, and
other frozen contracts; :meth:`to_mapping` converts them to JSON-safe lists
and dictionaries at persistence/UI boundaries.

Normalized image coordinates use ``x / width`` and ``y / height``.  A board's
``coverage_fraction`` is the projected outer-corner quadrilateral area divided
by the image area, while ``scale`` is its square root (a linear-size proxy).
Perspective ratios are top-edge/bottom-edge and left-edge/right-edge lengths.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any


Point2 = tuple[float, float]
ImageSize = tuple[int, int]


def _finite(value: Any, *, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _positive(value: Any, *, name: str) -> float:
    parsed = _finite(value, name=name)
    if parsed <= 0.0:
        raise ValueError(f"{name} must be > 0")
    return parsed


def _non_negative(value: Any, *, name: str) -> float:
    parsed = _finite(value, name=name)
    if parsed < 0.0:
        raise ValueError(f"{name} must be >= 0")
    return parsed


def _pair(value: Iterable[Any], *, name: str) -> tuple[float, float]:
    items = tuple(value)
    if len(items) != 2:
        raise ValueError(f"{name} must contain two values")
    return (
        _finite(items[0], name=f"{name}[0]"),
        _finite(items[1], name=f"{name}[1]"),
    )


def _image_size(value: Iterable[Any]) -> ImageSize:
    items = tuple(value)
    if len(items) != 2:
        raise ValueError("image_size must contain width and height")
    width, height = int(items[0]), int(items[1])
    if width <= 0 or height <= 0:
        raise ValueError("image_size width and height must be > 0")
    return width, height


def _sequence(value: Any) -> tuple[Any, ...]:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (str, bytes)):
        raise ValueError("corner coordinates must be numeric sequences")
    try:
        return tuple(value)
    except TypeError as exc:
        raise ValueError("corner coordinates must be numeric sequences") from exc


def _point(value: Any) -> Point2:
    """Accept ``(x, y)`` and OpenCV's nested ``((x, y),)`` point shape."""

    items = _sequence(value)
    while len(items) == 1:
        items = _sequence(items[0])
    if len(items) != 2:
        raise ValueError("each checkerboard corner must contain x and y")
    return (
        _finite(items[0], name="corner x"),
        _finite(items[1], name="corner y"),
    )


def _flatten_points(value: Any) -> tuple[Point2, ...]:
    """Flatten arbitrary row/OpenCV nesting without importing NumPy."""

    try:
        return (_point(value),)
    except (TypeError, ValueError):
        pass
    result: list[Point2] = []
    for item in _sequence(value):
        result.extend(_flatten_points(item))
    return tuple(result)


def _distance(left: Point2, right: Point2) -> float:
    return math.hypot(right[0] - left[0], right[1] - left[1])


def _polygon_area(points: Sequence[Point2]) -> float:
    return abs(
        sum(
            left[0] * right[1] - right[0] * left[1]
            for left, right in zip(points, (*points[1:], points[0]))
        )
    ) * 0.5


def _axis_angle_deg(start: Point2, end: Point2) -> float:
    # A checkerboard detector may reverse the grid order.  Modulo 180 keeps
    # the physical board-axis orientation invariant under that reversal.
    angle = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
    return (angle + 90.0) % 180.0 - 90.0


@dataclass(frozen=True)
class CalibrationViewGeometry:
    """JSON-safe geometric evidence extracted from one checkerboard image."""

    view_id: str
    image_size: ImageSize
    corner_count: int
    center_normalized: tuple[float, float]
    bbox_normalized: tuple[float, float, float, float]
    bbox_extent_normalized: tuple[float, float]
    coverage_fraction: float
    roll_deg: float
    top_bottom_ratio: float
    left_right_ratio: float
    scale: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "view_id", str(self.view_id))
        object.__setattr__(self, "image_size", _image_size(self.image_size))
        corner_count = int(self.corner_count)
        if corner_count < 4:
            raise ValueError("corner_count must be >= 4")
        object.__setattr__(self, "corner_count", corner_count)
        object.__setattr__(
            self,
            "center_normalized",
            _pair(self.center_normalized, name="center_normalized"),
        )
        bbox = tuple(self.bbox_normalized)
        if len(bbox) != 4:
            raise ValueError("bbox_normalized must contain left, top, right, bottom")
        parsed_bbox = tuple(
            _finite(value, name=f"bbox_normalized[{index}]")
            for index, value in enumerate(bbox)
        )
        if parsed_bbox[2] < parsed_bbox[0] or parsed_bbox[3] < parsed_bbox[1]:
            raise ValueError("bbox_normalized extents must be ordered")
        object.__setattr__(self, "bbox_normalized", parsed_bbox)
        extents = _pair(self.bbox_extent_normalized, name="bbox_extent_normalized")
        if extents[0] < 0.0 or extents[1] < 0.0:
            raise ValueError("bbox_extent_normalized must be non-negative")
        object.__setattr__(self, "bbox_extent_normalized", extents)
        object.__setattr__(
            self,
            "coverage_fraction",
            _non_negative(self.coverage_fraction, name="coverage_fraction"),
        )
        object.__setattr__(self, "roll_deg", _finite(self.roll_deg, name="roll_deg"))
        object.__setattr__(
            self,
            "top_bottom_ratio",
            _positive(self.top_bottom_ratio, name="top_bottom_ratio"),
        )
        object.__setattr__(
            self,
            "left_right_ratio",
            _positive(self.left_right_ratio, name="left_right_ratio"),
        )
        object.__setattr__(self, "scale", _positive(self.scale, name="scale"))

    @property
    def center_x_normalized(self) -> float:
        return self.center_normalized[0]

    @property
    def center_y_normalized(self) -> float:
        return self.center_normalized[1]

    def to_mapping(self) -> dict[str, Any]:
        return {
            "view_id": self.view_id,
            "image_size": list(self.image_size),
            "corner_count": self.corner_count,
            "center_normalized": list(self.center_normalized),
            "bbox_normalized": list(self.bbox_normalized),
            "bbox_extent_normalized": list(self.bbox_extent_normalized),
            "coverage_fraction": self.coverage_fraction,
            "roll_deg": self.roll_deg,
            "top_bottom_ratio": self.top_bottom_ratio,
            "left_right_ratio": self.left_right_ratio,
            "scale": self.scale,
        }

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "CalibrationViewGeometry":
        return cls(
            view_id=str(payload.get("view_id", "")),
            image_size=tuple(payload["image_size"]),
            corner_count=int(payload["corner_count"]),
            center_normalized=tuple(payload["center_normalized"]),
            bbox_normalized=tuple(payload["bbox_normalized"]),
            bbox_extent_normalized=tuple(payload["bbox_extent_normalized"]),
            coverage_fraction=float(payload["coverage_fraction"]),
            roll_deg=float(payload["roll_deg"]),
            top_bottom_ratio=float(payload["top_bottom_ratio"]),
            left_right_ratio=float(payload["left_right_ratio"]),
            scale=float(payload["scale"]),
        )


def extract_view_geometry(
    corners: Any,
    image_size: Iterable[Any],
    *,
    pattern_size: Iterable[Any] | None = None,
    view_id: str = "",
) -> CalibrationViewGeometry:
    """Extract deterministic geometry from refined row-major board corners.

    ``pattern_size`` is ``(inner_columns, inner_rows)``.  It is required for a
    flat/OpenCV ``N x 1 x 2`` input.  It may be omitted only when ``corners``
    is already a rectangular ``rows x columns x 2`` Python-style grid.
    """

    width, height = _image_size(image_size)
    raw = corners.tolist() if hasattr(corners, "tolist") else corners
    if pattern_size is None:
        rows = _sequence(raw)
        parsed_rows: list[tuple[Point2, ...]] = []
        for row in rows:
            row_items = _sequence(row)
            parsed_rows.append(tuple(_point(item) for item in row_items))
        if len(parsed_rows) < 2 or len(parsed_rows[0]) < 2:
            raise ValueError("pattern_size is required for flat/OpenCV checkerboard corners")
        columns = len(parsed_rows[0])
        if any(len(row) != columns for row in parsed_rows):
            raise ValueError("checkerboard corner rows must have equal length")
        row_count = len(parsed_rows)
        points = tuple(point for row in parsed_rows for point in row)
    else:
        columns, row_count = _image_size(pattern_size)
        if columns < 2 or row_count < 2:
            raise ValueError("pattern_size must have at least 2 columns and 2 rows")
        points = _flatten_points(raw)
        if len(points) != columns * row_count:
            raise ValueError(
                "checkerboard corner count does not match pattern_size: "
                f"found {len(points)}, expected {columns * row_count}"
            )

    left = min(point[0] for point in points)
    top = min(point[1] for point in points)
    right = max(point[0] for point in points)
    bottom = max(point[1] for point in points)
    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)

    top_left = points[0]
    top_right = points[columns - 1]
    bottom_left = points[(row_count - 1) * columns]
    bottom_right = points[row_count * columns - 1]
    top_length = _distance(top_left, top_right)
    bottom_length = _distance(bottom_left, bottom_right)
    left_length = _distance(top_left, bottom_left)
    right_length = _distance(top_right, bottom_right)
    if min(top_length, bottom_length, left_length, right_length) <= 1.0e-12:
        raise ValueError("checkerboard outer edges must have non-zero projected length")

    quadrilateral = (top_left, top_right, bottom_right, bottom_left)
    coverage = _polygon_area(quadrilateral) / float(width * height)
    if coverage <= 0.0:
        raise ValueError("checkerboard projected coverage must be > 0")
    bbox = (left / width, top / height, right / width, bottom / height)
    return CalibrationViewGeometry(
        view_id=view_id,
        image_size=(width, height),
        corner_count=len(points),
        center_normalized=(center_x / width, center_y / height),
        bbox_normalized=bbox,
        bbox_extent_normalized=((right - left) / width, (bottom - top) / height),
        coverage_fraction=coverage,
        roll_deg=_axis_angle_deg(top_left, top_right),
        top_bottom_ratio=top_length / bottom_length,
        left_right_ratio=left_length / right_length,
        scale=math.sqrt(coverage),
    )


@dataclass(frozen=True)
class CenteredRoiPolicy:
    """Configurable principal-point expectation for a known raster ROI.

    Set :attr:`CalibrationQualityPolicy.centered_roi_policy` to ``None`` when
    the camera has a documented non-centred crop and no expected principal
    point is available.  Alternatively, set ``expected_normalized`` to the
    documented ROI-specific expectation.
    """

    expected_normalized: tuple[float, float] = (0.5, 0.5)
    warning_offset_fraction: tuple[float, float] = (0.05, 0.05)
    failure_offset_fraction: tuple[float, float] = (0.10, 0.10)

    def __post_init__(self) -> None:
        expected = _pair(self.expected_normalized, name="expected_normalized")
        warning = _pair(self.warning_offset_fraction, name="warning_offset_fraction")
        failure = _pair(self.failure_offset_fraction, name="failure_offset_fraction")
        if not all(0.0 <= value <= 1.0 for value in expected):
            raise ValueError("expected_normalized must be within [0, 1]")
        if any(value < 0.0 for value in warning + failure):
            raise ValueError("principal-point offset thresholds must be non-negative")
        if any(fail < warn for warn, fail in zip(warning, failure)):
            raise ValueError("failure offsets must be >= warning offsets")
        object.__setattr__(self, "expected_normalized", expected)
        object.__setattr__(self, "warning_offset_fraction", warning)
        object.__setattr__(self, "failure_offset_fraction", failure)

    def to_mapping(self) -> dict[str, Any]:
        return {
            "expected_normalized": list(self.expected_normalized),
            "warning_offset_fraction": list(self.warning_offset_fraction),
            "failure_offset_fraction": list(self.failure_offset_fraction),
        }


@dataclass(frozen=True)
class CalibrationQualityPolicy:
    """All tunable camera-calibration quality and pose-selection thresholds."""

    centered_roi_policy: CenteredRoiPolicy | None = field(default_factory=CenteredRoiPolicy)
    minimum_effective_poses: int = 12
    minimum_union_extent_fraction: tuple[float, float] = (0.60, 0.60)
    center_grid_shape: tuple[int, int] = (3, 3)
    minimum_center_grid_cells: int = 5
    minimum_top_bottom_log_span: float = 0.10
    minimum_left_right_log_span: float = 0.10
    minimum_scale_relative_span: float = 0.20
    minimum_subset_estimates: int = 3
    maximum_subset_principal_span_fraction: tuple[float, float] = (0.02, 0.02)
    maximum_subset_focal_relative_span: float = 0.03
    maximum_intrinsic_principal_stddev_fraction: tuple[float, float] = (0.02, 0.02)
    maximum_intrinsic_focal_stddev_fraction: float = 0.03
    maximum_per_view_error_px: float = 1.0
    pose_center_tolerance_fraction: tuple[float, float] = (0.035, 0.035)
    pose_roll_tolerance_deg: float = 5.0
    pose_log_scale_tolerance: float = 0.08
    pose_log_perspective_tolerance: float = 0.06

    def __post_init__(self) -> None:
        if self.centered_roi_policy is not None and not isinstance(
            self.centered_roi_policy, CenteredRoiPolicy
        ):
            raise TypeError("centered_roi_policy must be CenteredRoiPolicy or None")
        minimum_poses = int(self.minimum_effective_poses)
        minimum_subsets = int(self.minimum_subset_estimates)
        grid = tuple(int(value) for value in self.center_grid_shape)
        minimum_cells = int(self.minimum_center_grid_cells)
        if minimum_poses < 1:
            raise ValueError("minimum_effective_poses must be >= 1")
        if minimum_subsets < 2:
            raise ValueError("minimum_subset_estimates must be >= 2")
        if len(grid) != 2 or min(grid) < 1:
            raise ValueError("center_grid_shape must contain positive columns and rows")
        if not 1 <= minimum_cells <= grid[0] * grid[1]:
            raise ValueError("minimum_center_grid_cells must fit within center_grid_shape")
        union_extent = _pair(
            self.minimum_union_extent_fraction,
            name="minimum_union_extent_fraction",
        )
        principal_span = _pair(
            self.maximum_subset_principal_span_fraction,
            name="maximum_subset_principal_span_fraction",
        )
        principal_stddev = _pair(
            self.maximum_intrinsic_principal_stddev_fraction,
            name="maximum_intrinsic_principal_stddev_fraction",
        )
        pose_center = _pair(
            self.pose_center_tolerance_fraction,
            name="pose_center_tolerance_fraction",
        )
        if any(not 0.0 <= value <= 1.0 for value in union_extent):
            raise ValueError("minimum_union_extent_fraction must be within [0, 1]")
        positive_names = (
            ("pose_center_tolerance_fraction[0]", pose_center[0]),
            ("pose_center_tolerance_fraction[1]", pose_center[1]),
            ("pose_roll_tolerance_deg", self.pose_roll_tolerance_deg),
            ("pose_log_scale_tolerance", self.pose_log_scale_tolerance),
            ("pose_log_perspective_tolerance", self.pose_log_perspective_tolerance),
        )
        for name, value in positive_names:
            _positive(value, name=name)
        non_negative_names = (
            ("maximum_subset_principal_span_fraction[0]", principal_span[0]),
            ("maximum_subset_principal_span_fraction[1]", principal_span[1]),
            ("maximum_subset_focal_relative_span", self.maximum_subset_focal_relative_span),
            ("maximum_intrinsic_principal_stddev_fraction[0]", principal_stddev[0]),
            ("maximum_intrinsic_principal_stddev_fraction[1]", principal_stddev[1]),
            ("maximum_intrinsic_focal_stddev_fraction", self.maximum_intrinsic_focal_stddev_fraction),
            ("maximum_per_view_error_px", self.maximum_per_view_error_px),
            ("minimum_top_bottom_log_span", self.minimum_top_bottom_log_span),
            ("minimum_left_right_log_span", self.minimum_left_right_log_span),
            ("minimum_scale_relative_span", self.minimum_scale_relative_span),
        )
        for name, value in non_negative_names:
            _non_negative(value, name=name)

        object.__setattr__(self, "minimum_effective_poses", minimum_poses)
        object.__setattr__(self, "minimum_subset_estimates", minimum_subsets)
        object.__setattr__(self, "center_grid_shape", grid)
        object.__setattr__(self, "minimum_center_grid_cells", minimum_cells)
        object.__setattr__(self, "minimum_union_extent_fraction", union_extent)
        object.__setattr__(self, "maximum_subset_principal_span_fraction", principal_span)
        object.__setattr__(
            self,
            "maximum_intrinsic_principal_stddev_fraction",
            principal_stddev,
        )
        object.__setattr__(self, "pose_center_tolerance_fraction", pose_center)
        for name in (
            "maximum_subset_focal_relative_span",
            "maximum_intrinsic_focal_stddev_fraction",
            "maximum_per_view_error_px",
            "pose_roll_tolerance_deg",
            "pose_log_scale_tolerance",
            "pose_log_perspective_tolerance",
            "minimum_top_bottom_log_span",
            "minimum_left_right_log_span",
            "minimum_scale_relative_span",
        ):
            object.__setattr__(self, name, float(getattr(self, name)))

    def to_mapping(self) -> dict[str, Any]:
        return {
            "centered_roi_policy": (
                None if self.centered_roi_policy is None else self.centered_roi_policy.to_mapping()
            ),
            "minimum_effective_poses": self.minimum_effective_poses,
            "minimum_union_extent_fraction": list(self.minimum_union_extent_fraction),
            "center_grid_shape": list(self.center_grid_shape),
            "minimum_center_grid_cells": self.minimum_center_grid_cells,
            "minimum_top_bottom_log_span": self.minimum_top_bottom_log_span,
            "minimum_left_right_log_span": self.minimum_left_right_log_span,
            "minimum_scale_relative_span": self.minimum_scale_relative_span,
            "minimum_subset_estimates": self.minimum_subset_estimates,
            "maximum_subset_principal_span_fraction": list(
                self.maximum_subset_principal_span_fraction
            ),
            "maximum_subset_focal_relative_span": self.maximum_subset_focal_relative_span,
            "maximum_intrinsic_principal_stddev_fraction": list(
                self.maximum_intrinsic_principal_stddev_fraction
            ),
            "maximum_intrinsic_focal_stddev_fraction": (
                self.maximum_intrinsic_focal_stddev_fraction
            ),
            "maximum_per_view_error_px": self.maximum_per_view_error_px,
            "pose_center_tolerance_fraction": list(self.pose_center_tolerance_fraction),
            "pose_roll_tolerance_deg": self.pose_roll_tolerance_deg,
            "pose_log_scale_tolerance": self.pose_log_scale_tolerance,
            "pose_log_perspective_tolerance": self.pose_log_perspective_tolerance,
        }


DEFAULT_CALIBRATION_QUALITY_POLICY = CalibrationQualityPolicy()


def _quantize(value: float, step: float) -> int:
    return math.floor(value / step + 0.5)


def pose_signature(
    view: CalibrationViewGeometry | Mapping[str, Any],
    policy: CalibrationQualityPolicy = DEFAULT_CALIBRATION_QUALITY_POLICY,
) -> tuple[int, ...]:
    """Return a deterministic, JSON-safe coarse signature of a board pose."""

    geometry = _coerce_view(view)
    doubled_roll = math.radians(2.0 * geometry.roll_deg)
    roll_chord_step = 2.0 * math.sin(
        math.radians(min(policy.pose_roll_tolerance_deg, 90.0))
    )
    return (
        _quantize(geometry.center_normalized[0], policy.pose_center_tolerance_fraction[0]),
        _quantize(geometry.center_normalized[1], policy.pose_center_tolerance_fraction[1]),
        _quantize(math.log(geometry.scale), policy.pose_log_scale_tolerance),
        _quantize(math.sin(doubled_roll), roll_chord_step),
        _quantize(math.cos(doubled_roll), roll_chord_step),
        _quantize(
            math.log(geometry.top_bottom_ratio),
            policy.pose_log_perspective_tolerance,
        ),
        _quantize(
            math.log(geometry.left_right_ratio),
            policy.pose_log_perspective_tolerance,
        ),
    )


def _axis_angle_distance_deg(left: float, right: float) -> float:
    difference = abs(left - right) % 180.0
    return min(difference, 180.0 - difference)


def _pose_distance(
    left: CalibrationViewGeometry,
    right: CalibrationViewGeometry,
    policy: CalibrationQualityPolicy,
) -> float:
    return max(
        abs(left.center_normalized[0] - right.center_normalized[0])
        / policy.pose_center_tolerance_fraction[0],
        abs(left.center_normalized[1] - right.center_normalized[1])
        / policy.pose_center_tolerance_fraction[1],
        _axis_angle_distance_deg(left.roll_deg, right.roll_deg)
        / policy.pose_roll_tolerance_deg,
        abs(math.log(left.scale / right.scale)) / policy.pose_log_scale_tolerance,
        abs(math.log(left.top_bottom_ratio / right.top_bottom_ratio))
        / policy.pose_log_perspective_tolerance,
        abs(math.log(left.left_right_ratio / right.left_right_ratio))
        / policy.pose_log_perspective_tolerance,
    )


def pose_novelty(
    candidate: CalibrationViewGeometry | Mapping[str, Any],
    references: Iterable[CalibrationViewGeometry | Mapping[str, Any]],
    policy: CalibrationQualityPolicy = DEFAULT_CALIBRATION_QUALITY_POLICY,
) -> float:
    """Return distance to the nearest pose; values ``<= 1`` are duplicates.

    With no references, ``2.0`` is returned as a finite, JSON-safe sentinel
    that is unambiguously novel under the fixed duplicate boundary of 1.0.
    """

    candidate_view = _coerce_view(candidate)
    reference_views = tuple(_coerce_view(reference) for reference in references)
    if not reference_views:
        return 2.0
    return min(_pose_distance(candidate_view, reference, policy) for reference in reference_views)


def _representative_preference(view: CalibrationViewGeometry) -> tuple[Any, ...]:
    # Prefer the observation with the strongest pixel footprint, then make
    # every tie-break independent of caller iteration order.
    return (
        -view.coverage_fraction,
        -(view.bbox_extent_normalized[0] * view.bbox_extent_normalized[1]),
        view.view_id,
        view.center_normalized,
        view.roll_deg,
        view.top_bottom_ratio,
        view.left_right_ratio,
    )


def select_pose_representatives(
    views: Iterable[CalibrationViewGeometry | Mapping[str, Any]],
    policy: CalibrationQualityPolicy = DEFAULT_CALIBRATION_QUALITY_POLICY,
) -> tuple[CalibrationViewGeometry, ...]:
    """Greedily collapse near-identical poses using deterministic ordering."""

    candidates = sorted((_coerce_view(view) for view in views), key=_representative_preference)
    selected: list[CalibrationViewGeometry] = []
    for candidate in candidates:
        if pose_novelty(candidate, selected, policy) > 1.0:
            selected.append(candidate)
    return tuple(sorted(selected, key=lambda view: (view.view_id, pose_signature(view, policy))))


@dataclass(frozen=True)
class CalibrationSubsetEstimate:
    """One deterministic subset solve used to test intrinsic stability."""

    subset_id: str
    fx_px: float
    fy_px: float
    cx_px: float
    cy_px: float
    view_count: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "subset_id", str(self.subset_id))
        for name in ("fx_px", "fy_px", "cx_px", "cy_px"):
            object.__setattr__(self, name, _finite(getattr(self, name), name=name))
        count = int(self.view_count)
        if count < 0:
            raise ValueError("view_count must be non-negative")
        object.__setattr__(self, "view_count", count)

    def to_mapping(self) -> dict[str, Any]:
        return {
            "subset_id": self.subset_id,
            "fx_px": self.fx_px,
            "fy_px": self.fy_px,
            "cx_px": self.cx_px,
            "cy_px": self.cy_px,
            "view_count": self.view_count,
        }

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "CalibrationSubsetEstimate":
        return cls(
            subset_id=str(payload.get("subset_id", "")),
            fx_px=float(payload["fx_px"]),
            fy_px=float(payload["fy_px"]),
            cx_px=float(payload["cx_px"]),
            cy_px=float(payload["cy_px"]),
            view_count=int(payload.get("view_count", 0)),
        )


@dataclass(frozen=True)
class CalibrationQualityReason:
    severity: str
    code: str
    message: str

    def __post_init__(self) -> None:
        severity = str(self.severity).upper()
        if severity not in {"WARNING", "FAIL"}:
            raise ValueError("reason severity must be WARNING or FAIL")
        code = str(self.code).strip()
        message = str(self.message).strip()
        if not code or not message:
            raise ValueError("quality reason code and message must be non-empty")
        object.__setattr__(self, "severity", severity)
        object.__setattr__(self, "code", code)
        object.__setattr__(self, "message", message)

    def to_mapping(self) -> dict[str, str]:
        return {"severity": self.severity, "code": self.code, "message": self.message}


@dataclass(frozen=True)
class CameraCalibrationQuality:
    """Immutable, JSON-safe output of the camera-calibration quality policy."""

    state: str
    image_size: ImageSize
    fx_px: float | None
    fy_px: float | None
    cx_px: float | None
    cy_px: float | None
    principal_offset_fraction: tuple[float, float] | None
    view_count: int
    effective_pose_count: int
    representative_view_ids: tuple[str, ...]
    union_bbox_normalized: tuple[float, float, float, float] | None
    union_extent_normalized: tuple[float, float]
    center_grid_occupied: tuple[tuple[int, int], ...]
    center_grid_shape: tuple[int, int]
    top_bottom_log_span: float
    left_right_log_span: float
    scale_relative_span: float
    subset_count: int
    subset_principal_span_px: tuple[float, float] | None
    subset_focal_relative_span: float | None
    intrinsic_focal_stddev_relative: float | None
    intrinsic_principal_stddev_px: tuple[float, float] | None
    per_view_error_count: int
    per_view_error_max_px: float | None
    reasons: tuple[CalibrationQualityReason, ...] = ()

    def __post_init__(self) -> None:
        state = str(self.state).upper()
        if state not in {"PASS", "WARNING", "FAIL"}:
            raise ValueError("quality state must be PASS, WARNING, or FAIL")
        object.__setattr__(self, "state", state)
        object.__setattr__(self, "image_size", _image_size(self.image_size))
        for name in ("fx_px", "fy_px", "cx_px", "cy_px"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _finite(value, name=name))
        if self.principal_offset_fraction is not None:
            object.__setattr__(
                self,
                "principal_offset_fraction",
                _pair(self.principal_offset_fraction, name="principal_offset_fraction"),
            )
        for name in (
            "view_count",
            "effective_pose_count",
            "subset_count",
            "per_view_error_count",
        ):
            value = int(getattr(self, name))
            if value < 0:
                raise ValueError(f"{name} must be non-negative")
            object.__setattr__(self, name, value)
        object.__setattr__(
            self,
            "representative_view_ids",
            tuple(str(value) for value in self.representative_view_ids),
        )
        if self.union_bbox_normalized is not None:
            bbox = tuple(self.union_bbox_normalized)
            if len(bbox) != 4:
                raise ValueError("union_bbox_normalized must contain four values")
            object.__setattr__(
                self,
                "union_bbox_normalized",
                tuple(_finite(value, name="union bbox") for value in bbox),
            )
        object.__setattr__(
            self,
            "union_extent_normalized",
            _pair(self.union_extent_normalized, name="union_extent_normalized"),
        )
        occupied = tuple((int(column), int(row)) for column, row in self.center_grid_occupied)
        object.__setattr__(self, "center_grid_occupied", occupied)
        grid = tuple(int(value) for value in self.center_grid_shape)
        if len(grid) != 2 or min(grid) < 1:
            raise ValueError("center_grid_shape must contain positive columns and rows")
        object.__setattr__(self, "center_grid_shape", grid)
        for name in (
            "top_bottom_log_span",
            "left_right_log_span",
            "scale_relative_span",
        ):
            object.__setattr__(self, name, _non_negative(getattr(self, name), name=name))
        if self.subset_principal_span_px is not None:
            object.__setattr__(
                self,
                "subset_principal_span_px",
                _pair(self.subset_principal_span_px, name="subset_principal_span_px"),
            )
        if self.subset_focal_relative_span is not None:
            object.__setattr__(
                self,
                "subset_focal_relative_span",
                _non_negative(
                    self.subset_focal_relative_span,
                    name="subset_focal_relative_span",
                ),
            )
        if self.intrinsic_focal_stddev_relative is not None:
            object.__setattr__(
                self,
                "intrinsic_focal_stddev_relative",
                _non_negative(
                    self.intrinsic_focal_stddev_relative,
                    name="intrinsic_focal_stddev_relative",
                ),
            )
        if self.intrinsic_principal_stddev_px is not None:
            principal_stddev = _pair(
                self.intrinsic_principal_stddev_px,
                name="intrinsic_principal_stddev_px",
            )
            if any(value < 0.0 for value in principal_stddev):
                raise ValueError("intrinsic_principal_stddev_px must be non-negative")
            object.__setattr__(self, "intrinsic_principal_stddev_px", principal_stddev)
        if self.per_view_error_max_px is not None:
            object.__setattr__(
                self,
                "per_view_error_max_px",
                _non_negative(self.per_view_error_max_px, name="per_view_error_max_px"),
            )
        reasons = tuple(self.reasons)
        if any(not isinstance(reason, CalibrationQualityReason) for reason in reasons):
            raise TypeError("reasons must contain CalibrationQualityReason values")
        object.__setattr__(self, "reasons", reasons)

    @property
    def reason_codes(self) -> tuple[str, ...]:
        return tuple(reason.code for reason in self.reasons)

    @property
    def passed(self) -> bool:
        return self.state == "PASS"

    @property
    def center_grid_occupied_count(self) -> int:
        return len(self.center_grid_occupied)

    @property
    def center_grid_coverage_fraction(self) -> float:
        return len(self.center_grid_occupied) / (
            self.center_grid_shape[0] * self.center_grid_shape[1]
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "state": self.state,
            "image_size": list(self.image_size),
            "fx_px": self.fx_px,
            "fy_px": self.fy_px,
            "cx_px": self.cx_px,
            "cy_px": self.cy_px,
            "principal_offset_fraction": (
                None
                if self.principal_offset_fraction is None
                else list(self.principal_offset_fraction)
            ),
            "view_count": self.view_count,
            "effective_pose_count": self.effective_pose_count,
            "representative_view_ids": list(self.representative_view_ids),
            "union_bbox_normalized": (
                None if self.union_bbox_normalized is None else list(self.union_bbox_normalized)
            ),
            "union_extent_normalized": list(self.union_extent_normalized),
            "center_grid_occupied": [list(cell) for cell in self.center_grid_occupied],
            "center_grid_occupied_count": self.center_grid_occupied_count,
            "center_grid_coverage_fraction": self.center_grid_coverage_fraction,
            "center_grid_shape": list(self.center_grid_shape),
            "top_bottom_log_span": self.top_bottom_log_span,
            "left_right_log_span": self.left_right_log_span,
            "scale_relative_span": self.scale_relative_span,
            "subset_count": self.subset_count,
            "subset_principal_span_px": (
                None
                if self.subset_principal_span_px is None
                else list(self.subset_principal_span_px)
            ),
            "subset_focal_relative_span": self.subset_focal_relative_span,
            "intrinsic_focal_stddev_relative": self.intrinsic_focal_stddev_relative,
            "intrinsic_principal_stddev_px": (
                None
                if self.intrinsic_principal_stddev_px is None
                else list(self.intrinsic_principal_stddev_px)
            ),
            "per_view_error_count": self.per_view_error_count,
            "per_view_error_max_px": self.per_view_error_max_px,
            "reason_codes": list(self.reason_codes),
            "reasons": [reason.to_mapping() for reason in self.reasons],
        }


def _coerce_view(value: CalibrationViewGeometry | Mapping[str, Any]) -> CalibrationViewGeometry:
    if isinstance(value, CalibrationViewGeometry):
        return value
    if isinstance(value, Mapping):
        return CalibrationViewGeometry.from_mapping(value)
    raise TypeError("views must contain CalibrationViewGeometry values or mappings")


def _coerce_subset(
    value: CalibrationSubsetEstimate | Mapping[str, Any],
) -> CalibrationSubsetEstimate:
    if isinstance(value, CalibrationSubsetEstimate):
        return value
    if isinstance(value, Mapping):
        return CalibrationSubsetEstimate.from_mapping(value)
    raise TypeError("subset estimates must be CalibrationSubsetEstimate values or mappings")


def _matrix_values(camera_matrix: Any) -> tuple[tuple[float, float, float], ...]:
    if hasattr(camera_matrix, "camera_matrix"):
        camera_matrix = camera_matrix.camera_matrix
    elif isinstance(camera_matrix, Mapping) and "camera_matrix" in camera_matrix:
        camera_matrix = camera_matrix["camera_matrix"]
    rows = tuple(tuple(row) for row in camera_matrix)
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError("camera_matrix must be 3x3")
    return tuple(
        tuple(_finite(value, name="camera_matrix") for value in row)  # type: ignore[misc]
        for row in rows
    )


def _relative_span(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    average = sum(values) / len(values)
    if average <= 0.0:
        return 0.0
    return (max(values) - min(values)) / average


def _reason(severity: str, code: str, message: str) -> CalibrationQualityReason:
    return CalibrationQualityReason(severity=severity, code=code, message=message)


def evaluate_camera_calibration_quality(
    camera_matrix: Any,
    image_size: Iterable[Any],
    views: Iterable[CalibrationViewGeometry | Mapping[str, Any]],
    subset_estimates: Iterable[CalibrationSubsetEstimate | Mapping[str, Any]] = (),
    *,
    intrinsic_standard_deviations: Mapping[str, Any] | None = None,
    per_view_errors_px: Iterable[Any] | None = None,
    policy: CalibrationQualityPolicy = DEFAULT_CALIBRATION_QUALITY_POLICY,
) -> CameraCalibrationQuality:
    """Evaluate intrinsic validity, pose coverage, and subset stability."""

    width, height = _image_size(image_size)
    reasons: list[CalibrationQualityReason] = []
    fx: float | None = None
    fy: float | None = None
    cx: float | None = None
    cy: float | None = None
    principal_offset: tuple[float, float] | None = None
    focal_stddev_relative: float | None = None
    principal_stddev_px: tuple[float, float] | None = None
    parsed_view_errors: tuple[float, ...] = ()
    per_view_error_max_px: float | None = None
    try:
        matrix = _matrix_values(camera_matrix)
    except (TypeError, ValueError, KeyError) as exc:
        reasons.append(_reason("FAIL", "invalid_camera_matrix", str(exc)))
    else:
        fx, fy, cx, cy = matrix[0][0], matrix[1][1], matrix[0][2], matrix[1][2]
        if fx <= 0.0:
            reasons.append(_reason("FAIL", "non_positive_fx", f"fx must be > 0 px; got {fx:g}"))
        if fy <= 0.0:
            reasons.append(_reason("FAIL", "non_positive_fy", f"fy must be > 0 px; got {fy:g}"))
        if not 0.0 <= cx < width:
            reasons.append(
                _reason("FAIL", "cx_out_of_frame", f"cx={cx:g} px is outside [0, {width})")
            )
        if not 0.0 <= cy < height:
            reasons.append(
                _reason("FAIL", "cy_out_of_frame", f"cy={cy:g} px is outside [0, {height})")
            )

        roi_policy = policy.centered_roi_policy
        if roi_policy is not None:
            expected_x = roi_policy.expected_normalized[0] * (width - 1)
            expected_y = roi_policy.expected_normalized[1] * (height - 1)
            principal_offset = (abs(cx - expected_x) / width, abs(cy - expected_y) / height)
            for axis, value, warning_limit, failure_limit in zip(
                ("x", "y"),
                principal_offset,
                roi_policy.warning_offset_fraction,
                roi_policy.failure_offset_fraction,
            ):
                if value > failure_limit:
                    reasons.append(
                        _reason(
                            "FAIL",
                            f"principal_point_{axis}_offset_exceeds_failure_limit",
                            f"principal-point {axis} offset {value:.3%} exceeds "
                            f"{failure_limit:.3%}",
                        )
                    )
                elif value > warning_limit:
                    reasons.append(
                        _reason(
                            "WARNING",
                            f"principal_point_{axis}_offset_exceeds_warning_limit",
                            f"principal-point {axis} offset {value:.3%} exceeds "
                            f"{warning_limit:.3%}",
                        )
                    )

    if intrinsic_standard_deviations is not None:
        required_stddevs: dict[str, float] = {}
        missing_stddevs: list[str] = []
        for name in ("fx_px", "fy_px", "cx_px", "cy_px"):
            raw_value = intrinsic_standard_deviations.get(name)
            if raw_value is None:
                missing_stddevs.append(name)
                continue
            try:
                value = _non_negative(raw_value, name=f"intrinsic stddev {name}")
            except (TypeError, ValueError) as exc:
                reasons.append(_reason("FAIL", "invalid_intrinsic_standard_deviation", str(exc)))
                continue
            required_stddevs[name] = value
        if missing_stddevs:
            reasons.append(
                _reason(
                    "WARNING",
                    "intrinsic_uncertainty_not_evaluated",
                    "missing intrinsic standard deviation(s): " + ", ".join(missing_stddevs),
                )
            )
        if (
            fx is not None
            and fy is not None
            and fx > 0.0
            and fy > 0.0
            and all(name in required_stddevs for name in ("fx_px", "fy_px"))
        ):
            focal_stddev_relative = max(
                required_stddevs["fx_px"] / abs(fx),
                required_stddevs["fy_px"] / abs(fy),
            )
            if focal_stddev_relative > policy.maximum_intrinsic_focal_stddev_fraction:
                reasons.append(
                    _reason(
                        "FAIL",
                        "intrinsic_focal_stddev_exceeds_limit",
                        f"intrinsic focal standard deviation {focal_stddev_relative:.3%} "
                        f"exceeds {policy.maximum_intrinsic_focal_stddev_fraction:.3%}",
                    )
                )
        if all(name in required_stddevs for name in ("cx_px", "cy_px")):
            principal_stddev_px = (
                required_stddevs["cx_px"],
                required_stddevs["cy_px"],
            )
            for axis, value, axis_size, limit in zip(
                ("x", "y"),
                principal_stddev_px,
                (width, height),
                policy.maximum_intrinsic_principal_stddev_fraction,
            ):
                fraction = value / axis_size
                if fraction > limit:
                    reasons.append(
                        _reason(
                            "FAIL",
                            f"intrinsic_principal_{axis}_stddev_exceeds_limit",
                            f"intrinsic principal-point {axis} standard deviation "
                            f"{value:.3f} px ({fraction:.3%}) exceeds {limit:.3%}",
                        )
                    )

    if per_view_errors_px is not None:
        parsed_errors: list[float] = []
        try:
            for index, raw_value in enumerate(per_view_errors_px):
                parsed_errors.append(
                    _non_negative(raw_value, name=f"per_view_errors_px[{index}]")
                )
        except (TypeError, ValueError) as exc:
            reasons.append(_reason("FAIL", "invalid_per_view_reprojection_error", str(exc)))
        parsed_view_errors = tuple(parsed_errors)
        if not parsed_view_errors:
            reasons.append(
                _reason(
                    "WARNING",
                    "per_view_reprojection_error_not_evaluated",
                    "no per-view reprojection errors are available",
                )
            )
        else:
            per_view_error_max_px = max(parsed_view_errors)
            if per_view_error_max_px > policy.maximum_per_view_error_px:
                reasons.append(
                    _reason(
                        "FAIL",
                        "per_view_reprojection_error_exceeds_limit",
                        f"maximum per-view reprojection error {per_view_error_max_px:.3f} px "
                        f"exceeds {policy.maximum_per_view_error_px:.3f} px",
                    )
                )

    parsed_views: list[CalibrationViewGeometry] = []
    for index, raw_view in enumerate(views):
        try:
            parsed_views.append(_coerce_view(raw_view))
        except (TypeError, ValueError, KeyError) as exc:
            reasons.append(
                _reason(
                    "FAIL",
                    "invalid_view_geometry",
                    f"view {index} geometry is invalid: {exc}",
                )
            )
    mismatched = sorted(
        view.view_id for view in parsed_views if view.image_size != (width, height)
    )
    if mismatched:
        reasons.append(
            _reason(
                "FAIL",
                "view_image_size_mismatch",
                f"{len(mismatched)} view(s) do not match image size {width}x{height}",
            )
        )

    representatives = select_pose_representatives(parsed_views, policy)
    if len(representatives) < policy.minimum_effective_poses:
        reasons.append(
            _reason(
                "FAIL",
                "insufficient_effective_poses",
                f"effective pose count {len(representatives)} is below "
                f"{policy.minimum_effective_poses}",
            )
        )

    union_bbox: tuple[float, float, float, float] | None = None
    union_extent = (0.0, 0.0)
    occupied: tuple[tuple[int, int], ...] = ()
    top_bottom_span = 0.0
    left_right_span = 0.0
    scale_span = 0.0
    if parsed_views:
        union_bbox = (
            min(view.bbox_normalized[0] for view in parsed_views),
            min(view.bbox_normalized[1] for view in parsed_views),
            max(view.bbox_normalized[2] for view in parsed_views),
            max(view.bbox_normalized[3] for view in parsed_views),
        )
        union_extent = (
            union_bbox[2] - union_bbox[0],
            union_bbox[3] - union_bbox[1],
        )
        for axis, actual, minimum in zip(
            ("x", "y"), union_extent, policy.minimum_union_extent_fraction
        ):
            if actual < minimum:
                reasons.append(
                    _reason(
                        "FAIL",
                        f"insufficient_union_{axis}_extent",
                        f"union {axis} extent {actual:.3%} is below {minimum:.3%}",
                    )
                )

        grid_columns, grid_rows = policy.center_grid_shape
        cells = {
            (
                min(grid_columns - 1, max(0, int(math.floor(view.center_normalized[0] * grid_columns)))),
                min(grid_rows - 1, max(0, int(math.floor(view.center_normalized[1] * grid_rows)))),
            )
            for view in representatives
        }
        occupied = tuple(sorted(cells))
        if len(occupied) < policy.minimum_center_grid_cells:
            reasons.append(
                _reason(
                    "FAIL",
                    "insufficient_center_grid_coverage",
                    f"checkerboard centers occupy {len(occupied)} grid cells; "
                    f"need {policy.minimum_center_grid_cells}",
                )
            )

        top_bottom_values = [math.log(view.top_bottom_ratio) for view in representatives]
        left_right_values = [math.log(view.left_right_ratio) for view in representatives]
        scales = [view.scale for view in representatives]
        if top_bottom_values:
            top_bottom_span = max(top_bottom_values) - min(top_bottom_values)
            left_right_span = max(left_right_values) - min(left_right_values)
            scale_span = _relative_span(scales)
        if top_bottom_span < policy.minimum_top_bottom_log_span:
            reasons.append(
                _reason(
                    "FAIL",
                    "insufficient_top_bottom_perspective_diversity",
                    f"top/bottom log-ratio span {top_bottom_span:.4f} is below "
                    f"{policy.minimum_top_bottom_log_span:.4f}",
                )
            )
        if left_right_span < policy.minimum_left_right_log_span:
            reasons.append(
                _reason(
                    "FAIL",
                    "insufficient_left_right_perspective_diversity",
                    f"left/right log-ratio span {left_right_span:.4f} is below "
                    f"{policy.minimum_left_right_log_span:.4f}",
                )
            )
        if scale_span < policy.minimum_scale_relative_span:
            reasons.append(
                _reason(
                    "FAIL",
                    "insufficient_scale_diversity",
                    f"relative scale span {scale_span:.3%} is below "
                    f"{policy.minimum_scale_relative_span:.3%}",
                )
            )
    else:
        reasons.extend(
            (
                _reason("FAIL", "insufficient_union_x_extent", "no view geometry is available"),
                _reason("FAIL", "insufficient_union_y_extent", "no view geometry is available"),
                _reason(
                    "FAIL",
                    "insufficient_center_grid_coverage",
                    "no checkerboard centers are available",
                ),
                _reason(
                    "FAIL",
                    "insufficient_top_bottom_perspective_diversity",
                    "no checkerboard perspective evidence is available",
                ),
                _reason(
                    "FAIL",
                    "insufficient_left_right_perspective_diversity",
                    "no checkerboard perspective evidence is available",
                ),
                _reason("FAIL", "insufficient_scale_diversity", "no checkerboard scale evidence is available"),
            )
        )

    parsed_subsets: list[CalibrationSubsetEstimate] = []
    for index, raw_subset in enumerate(subset_estimates):
        try:
            subset = _coerce_subset(raw_subset)
        except (TypeError, ValueError, KeyError) as exc:
            reasons.append(
                _reason(
                    "FAIL",
                    "invalid_subset_estimate",
                    f"subset {index} estimate is invalid: {exc}",
                )
            )
            continue
        if subset.fx_px <= 0.0 or subset.fy_px <= 0.0:
            reasons.append(
                _reason(
                    "FAIL",
                    "invalid_subset_focal_length",
                    f"subset {subset.subset_id!r} has non-positive focal length",
                )
            )
        parsed_subsets.append(subset)

    subset_principal_span: tuple[float, float] | None = None
    subset_focal_span: float | None = None
    if len(parsed_subsets) < policy.minimum_subset_estimates:
        reasons.append(
            _reason(
                "WARNING",
                "subset_stability_not_evaluated",
                f"only {len(parsed_subsets)} subset estimate(s); "
                f"need {policy.minimum_subset_estimates}",
            )
        )
    if len(parsed_subsets) >= 2:
        cx_values = [subset.cx_px for subset in parsed_subsets]
        cy_values = [subset.cy_px for subset in parsed_subsets]
        subset_principal_span = (
            max(cx_values) - min(cx_values),
            max(cy_values) - min(cy_values),
        )
        for axis, actual_px, axis_size, limit_fraction in zip(
            ("x", "y"),
            subset_principal_span,
            (width, height),
            policy.maximum_subset_principal_span_fraction,
        ):
            actual_fraction = actual_px / axis_size
            if actual_fraction > limit_fraction:
                reasons.append(
                    _reason(
                        "FAIL",
                        f"subset_principal_{axis}_span_exceeds_limit",
                        f"subset principal-point {axis} span {actual_px:.3f} px "
                        f"({actual_fraction:.3%}) exceeds {limit_fraction:.3%}",
                    )
                )
        subset_focal_span = max(
            _relative_span([subset.fx_px for subset in parsed_subsets]),
            _relative_span([subset.fy_px for subset in parsed_subsets]),
        )
        if subset_focal_span > policy.maximum_subset_focal_relative_span:
            reasons.append(
                _reason(
                    "FAIL",
                    "subset_focal_span_exceeds_limit",
                    f"subset focal relative span {subset_focal_span:.3%} exceeds "
                    f"{policy.maximum_subset_focal_relative_span:.3%}",
                )
            )

    state = (
        "FAIL"
        if any(reason.severity == "FAIL" for reason in reasons)
        else "WARNING"
        if reasons
        else "PASS"
    )
    return CameraCalibrationQuality(
        state=state,
        image_size=(width, height),
        fx_px=fx,
        fy_px=fy,
        cx_px=cx,
        cy_px=cy,
        principal_offset_fraction=principal_offset,
        view_count=len(parsed_views),
        effective_pose_count=len(representatives),
        representative_view_ids=tuple(view.view_id for view in representatives),
        union_bbox_normalized=union_bbox,
        union_extent_normalized=union_extent,
        center_grid_occupied=occupied,
        center_grid_shape=policy.center_grid_shape,
        top_bottom_log_span=top_bottom_span,
        left_right_log_span=left_right_span,
        scale_relative_span=scale_span,
        subset_count=len(parsed_subsets),
        subset_principal_span_px=subset_principal_span,
        subset_focal_relative_span=subset_focal_span,
        intrinsic_focal_stddev_relative=focal_stddev_relative,
        intrinsic_principal_stddev_px=principal_stddev_px,
        per_view_error_count=len(parsed_view_errors),
        per_view_error_max_px=per_view_error_max_px,
        reasons=tuple(reasons),
    )


__all__ = [
    "CalibrationQualityPolicy",
    "CalibrationQualityReason",
    "CalibrationSubsetEstimate",
    "CalibrationViewGeometry",
    "CameraCalibrationQuality",
    "CenteredRoiPolicy",
    "DEFAULT_CALIBRATION_QUALITY_POLICY",
    "evaluate_camera_calibration_quality",
    "extract_view_geometry",
    "pose_novelty",
    "pose_signature",
    "select_pose_representatives",
]
