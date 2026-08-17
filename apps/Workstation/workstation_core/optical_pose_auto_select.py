"""OpenCV-only automatic optical-pose point suggestions.

The functions in this module deliberately stop at *evidence suggestion*.  They
never estimate or apply a pose.  Returned points are native OpenCV image
pixels, so a caller can put them through the same exact K,D path as manually
entered points after an operator has reviewed and accepted the suggestion.

No map, GeoTIFF, neural-network weight, or external service is used here.  The
Horizon path is a robust image-gradient/Hough line proposal.  The Mountain
path uses SIFT when the installed OpenCV build provides it and otherwise ORB,
then mutual ratio matching and RANSAC epipolar filtering.  A small skyline
gradient fallback is retained for low-texture mountain images, but it is never
labelled as a 3-D terrain correspondence.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Sequence


Point2 = tuple[float, float]


def _finite(value: Any, *, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _points(values: Iterable[Iterable[Any]]) -> tuple[Point2, ...]:
    result: list[Point2] = []
    for value in values:
        row = tuple(value)
        if len(row) != 2:
            raise ValueError("points must contain (u, v) pairs")
        result.append((_finite(row[0], name="u_px"), _finite(row[1], name="v_px")))
    return tuple(result)


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    return _finite(value, name="metric")


@dataclass(frozen=True)
class AutoSelectionResult:
    """One-side automatic suggestion and its auditable quality evidence."""

    mode: str
    state: str
    detection_method: str
    points: tuple[Point2, ...] = ()
    # Optional per-point match-quality score in [0, 1], index-aligned with
    # ``points``. Empty when the mode has no natural per-point score (e.g.
    # Horizon's single fitted line, or the skyline fallback's sampled
    # points) -- callers must treat empty as "no per-point data", not as
    # zero confidence.
    point_confidences: tuple[float, ...] = ()
    candidate_count: int = 0
    match_count: int = 0
    inlier_count: int = 0
    residual_px: float | None = None
    confidence: float = 0.0
    spatial_rank: int = 0
    eligible: bool = False
    failure_reason: str = ""
    warnings: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        mode = str(self.mode).strip().lower()
        state = str(self.state).strip().upper()
        if mode not in {"horizon", "mountain"}:
            raise ValueError("mode must be horizon or mountain")
        if state not in {"GOOD", "LOW_CONFIDENCE", "NO_DATA"}:
            raise ValueError("state must be GOOD, LOW_CONFIDENCE, or NO_DATA")
        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "state", state)
        object.__setattr__(self, "detection_method", str(self.detection_method or "unknown"))
        object.__setattr__(self, "points", _points(self.points))
        confidences = tuple(
            min(1.0, max(0.0, _finite(value, name="point_confidences"))) for value in self.point_confidences
        )
        if confidences and len(confidences) != len(self.points):
            raise ValueError("point_confidences must be empty or match points length")
        object.__setattr__(self, "point_confidences", confidences)
        object.__setattr__(self, "candidate_count", max(0, int(self.candidate_count)))
        object.__setattr__(self, "match_count", max(0, int(self.match_count)))
        object.__setattr__(self, "inlier_count", max(0, int(self.inlier_count)))
        residual = _optional_float(self.residual_px)
        object.__setattr__(self, "residual_px", residual)
        confidence = min(1.0, max(0.0, _finite(self.confidence, name="confidence")))
        object.__setattr__(self, "confidence", confidence)
        object.__setattr__(self, "spatial_rank", max(0, int(self.spatial_rank)))
        object.__setattr__(self, "eligible", bool(self.eligible))
        object.__setattr__(self, "failure_reason", str(self.failure_reason or ""))
        object.__setattr__(self, "warnings", tuple(str(value) for value in self.warnings))
        object.__setattr__(self, "metadata", dict(self.metadata))

    @property
    def accepted(self) -> bool:
        return self.state == "GOOD" and self.eligible

    @property
    def reason(self) -> str:
        return self.failure_reason or ("ok" if self.accepted else "low confidence")

    def to_mapping(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "state": self.state,
            "detection_method": self.detection_method,
            "points": [{"u_px": u, "v_px": v} for u, v in self.points],
            "point_confidences": list(self.point_confidences),
            "candidate_count": self.candidate_count,
            "match_count": self.match_count,
            "inlier_count": self.inlier_count,
            "residual_px": self.residual_px,
            "confidence": self.confidence,
            "spatial_rank": self.spatial_rank,
            "eligible": self.eligible,
            "failure_reason": self.failure_reason,
            "reason": self.reason,
            "warnings": list(self.warnings),
            "metadata": dict(self.metadata),
        }

    # A lightweight mapping-like surface keeps the result convenient in
    # controller code and in small integration tests without making the
    # dataclass mutable.
    def __getitem__(self, key: str) -> Any:
        return self.to_mapping()[key]

    def get(self, key: str, default: Any = None) -> Any:
        return self.to_mapping().get(key, default)


@dataclass(frozen=True)
class MountainAutoSelection:
    """Paired left/right mountain suggestion."""

    left: AutoSelectionResult
    right: AutoSelectionResult
    state: str
    detection_method: str
    match_count: int = 0
    inlier_count: int = 0
    residual_px: float | None = None
    confidence: float = 0.0
    eligible: bool = False
    failure_reason: str = ""
    warnings: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        state = str(self.state).strip().upper()
        if state not in {"GOOD", "LOW_CONFIDENCE", "NO_DATA"}:
            raise ValueError("state must be GOOD, LOW_CONFIDENCE, or NO_DATA")
        object.__setattr__(self, "state", state)
        object.__setattr__(self, "detection_method", str(self.detection_method or "unknown"))
        object.__setattr__(self, "match_count", max(0, int(self.match_count)))
        object.__setattr__(self, "inlier_count", max(0, int(self.inlier_count)))
        object.__setattr__(self, "residual_px", _optional_float(self.residual_px))
        object.__setattr__(
            self,
            "confidence",
            min(1.0, max(0.0, _finite(self.confidence, name="confidence"))),
        )
        object.__setattr__(self, "eligible", bool(self.eligible))
        object.__setattr__(self, "failure_reason", str(self.failure_reason or ""))
        object.__setattr__(self, "warnings", tuple(str(value) for value in self.warnings))
        object.__setattr__(self, "metadata", dict(self.metadata))

    @property
    def accepted(self) -> bool:
        return self.state == "GOOD" and self.eligible

    @property
    def reason(self) -> str:
        return self.failure_reason or ("ok" if self.accepted else "low confidence")

    def to_mapping(self) -> dict[str, Any]:
        return {
            "mode": "mountain",
            "state": self.state,
            "detection_method": self.detection_method,
            "left": self.left.to_mapping(),
            "right": self.right.to_mapping(),
            "match_count": self.match_count,
            "inlier_count": self.inlier_count,
            "residual_px": self.residual_px,
            "confidence": self.confidence,
            "eligible": self.eligible,
            "failure_reason": self.failure_reason,
            "reason": self.reason,
            "warnings": list(self.warnings),
            "metadata": dict(self.metadata),
        }

    def __getitem__(self, key: str) -> Any:
        return self.to_mapping()[key]

    def get(self, key: str, default: Any = None) -> Any:
        return self.to_mapping().get(key, default)


def _failure(
    mode: str,
    method: str,
    reason: str,
    *,
    warnings: Iterable[str] = (),
    metadata: Mapping[str, Any] | None = None,
) -> AutoSelectionResult:
    return AutoSelectionResult(
        mode=mode,
        state="NO_DATA",
        detection_method=method,
        failure_reason=reason,
        warnings=tuple(warnings),
        metadata=dict(metadata or {}),
    )


def _require_cv_numpy() -> tuple[Any, Any]:
    try:
        import cv2
        import numpy as np
    except Exception as exc:  # pragma: no cover - environment dependent.
        raise RuntimeError("OpenCV and NumPy are required for automatic point selection") from exc
    return cv2, np


def _as_gray(image: Any, cv2: Any, np: Any) -> Any:
    """Copy a QImage/array into a single-channel uint8 image."""

    if hasattr(image, "convertToFormat") and hasattr(image, "constBits"):
        # Avoid importing PyQt5 in the computational module.  QImage's public
        # methods are sufficient and the copied bytes do not alias the UI.
        rgba_enum = getattr(type(image), "Format_RGBA8888", None)
        if rgba_enum is None:
            from PyQt5 import QtGui  # pragma: no cover - only QImage path.

            rgba_enum = QtGui.QImage.Format_RGBA8888
        converted = image.convertToFormat(rgba_enum)
        width, height = int(converted.width()), int(converted.height())
        byte_count = int(converted.bytesPerLine()) * height
        bits = converted.constBits()
        bits.setsize(byte_count)
        rgba = np.frombuffer(bytes(bits), dtype=np.uint8).reshape(height, converted.bytesPerLine())
        rgba = rgba[:, : width * 4].reshape(height, width, 4).copy()
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2GRAY)

    array = np.asarray(image)
    if array.size == 0:
        raise ValueError("image is empty")
    if array.ndim == 2:
        gray = array
    elif array.ndim == 3 and array.shape[2] == 4:
        gray = cv2.cvtColor(array, cv2.COLOR_BGRA2GRAY)
    elif array.ndim == 3 and array.shape[2] == 3:
        gray = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)
    else:
        raise ValueError("image must be grayscale, BGR/BGRA, or QImage")
    if gray.dtype != np.uint8:
        scale = 255.0 / max(1.0, float(np.max(gray)))
        gray = np.clip(gray.astype(np.float32) * scale, 0.0, 255.0).astype(np.uint8)
    return np.ascontiguousarray(gray)


def _matrix(value: Any, np: Any) -> Any | None:
    if value is None:
        return None
    raw = getattr(value, "camera_matrix", value)
    try:
        matrix = np.asarray(raw, dtype=np.float64)
    except Exception:
        return None
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        return None
    return matrix


def _distortion(value: Any, np: Any) -> Any:
    raw = getattr(value, "distortion_coefficients", value)
    try:
        result = np.asarray(raw if raw is not None else (), dtype=np.float64).reshape(-1, 1)
    except Exception:
        return np.zeros((5, 1), dtype=np.float64)
    if result.size == 0:
        return np.zeros((5, 1), dtype=np.float64)
    return result


def _undistort_gray(gray: Any, camera_matrix: Any, distortion: Any, cv2: Any, np: Any) -> Any:
    matrix = _matrix(camera_matrix, np)
    if matrix is None:
        return gray
    try:
        return cv2.undistort(gray, matrix, _distortion(distortion, np), None, matrix)
    except Exception:
        # A suggestion must never make a frozen evidence workflow unusable
        # merely because a camera exposes an unusual OpenCV distortion length.
        return gray


def _raw_from_ideal(points: Sequence[Point2], camera_matrix: Any, distortion: Any, cv2: Any, np: Any) -> tuple[Point2, ...]:
    matrix = _matrix(camera_matrix, np)
    if matrix is None or not points:
        return tuple(points)
    ideal = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    normalized = np.empty_like(ideal)
    normalized[:, 0, 0] = (ideal[:, 0, 0] - cx) / fx
    normalized[:, 0, 1] = (ideal[:, 0, 1] - cy) / fy
    try:
        projected, _ = cv2.projectPoints(
            np.concatenate(
                (normalized, np.ones((len(points), 1, 1), dtype=np.float64)),
                axis=2,
            ),
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            matrix,
            _distortion(distortion, np),
        )
        return tuple((float(row[0, 0]), float(row[0, 1])) for row in projected)
    except Exception:
        return tuple(points)


def _line_endpoints(slope: float, intercept: float, width: int) -> tuple[Point2, Point2]:
    return ((0.0, float(intercept)), (float(max(0, width - 1)), float(slope * (width - 1) + intercept)))


def _fit_line(points: Any, np: Any) -> tuple[float, float, Any] | None:
    if len(points) < 3:
        return None
    work = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if len(work) < 3:
        return None
    mask = np.ones(len(work), dtype=bool)
    for _ in range(5):
        if int(mask.sum()) < 3:
            break
        slope, intercept = np.polyfit(work[mask, 0], work[mask, 1], 1)
        residuals = np.abs(work[:, 1] - (slope * work[:, 0] + intercept))
        median = float(np.median(residuals[mask]))
        threshold = max(2.0, 2.5 * median)
        next_mask = residuals <= threshold
        if bool(np.array_equal(mask, next_mask)):
            break
        mask = next_mask
    if int(mask.sum()) < 3:
        return None
    slope, intercept = np.polyfit(work[mask, 0], work[mask, 1], 1)
    residuals = np.abs(work[mask, 1] - (slope * work[mask, 0] + intercept))
    return float(slope), float(intercept), residuals


def _estimate_ground_mask(gray: Any, cv2: Any, np: Any) -> Any | None:
    """Per-pixel "likely ground/ridge, not sky/cloud" mask, reused only to
    bias Mountain's SIFT/ORB matches away from the sky.

    A richly-textured sky (clouds, especially at sunset) gives SIFT far more
    and stronger keypoints than a plain terrain silhouette, so unconstrained
    matching drifts onto the sky instead of the ridge. A single straight
    boundary line assumption (this module's first approach) fails whenever
    the ridge does not span the frame -- a lone foreground tree against a
    dramatic sky, for instance, pulls a robust line fit toward whichever
    edge dominates the column-wise gradient search, which is frequently a
    cloud or sun-glare edge, not the ridge.

    This instead classifies each neighbourhood as sky-like when it is both
    smoother (lower local intensity std) and brighter (higher local mean)
    than most of the frame: even backlit sunset cloud, despite being high
    *edge* contrast, is comparatively smooth and bright once averaged over a
    neighbourhood, while terrain/vegetation absorbs light and is comparatively
    dark and finely textured. This holds regardless of how much of the frame
    the ridge/silhouette occupies. Returning ``None`` (too small) means "no
    usable estimate" -- the caller must not restrict matches on a failed
    estimate, only on a successful one.
    """

    height, width = gray.shape[:2]
    if width < 32 or height < 24:
        return None
    ksize = max(9, (min(height, width) // 40) | 1)
    smoothed = gray.astype(np.float32)
    mean = cv2.boxFilter(smoothed, -1, (ksize, ksize))
    mean_sq = cv2.boxFilter(smoothed * smoothed, -1, (ksize, ksize))
    local_std = np.sqrt(np.clip(mean_sq - mean * mean, 0.0, None))
    texture_threshold = float(np.percentile(local_std, 65.0))
    brightness_threshold = float(np.percentile(mean, 45.0))
    sky_like = (local_std <= texture_threshold) & (mean >= brightness_threshold)
    return ~sky_like


def _estimate_ridge_boundary(mask: Any, np: Any) -> Any:
    """Per-column ridge/horizon row derived from :func:`_estimate_ground_mask`.

    ``_estimate_ground_mask`` alone tells Mountain's matcher "not sky", which
    is not the same as "on the ridge" -- it happily keeps matches anywhere on
    open foreground ground, scattered deep inside a silhouette's interior, as
    well as points right at its edge. This instead walks each column
    top-to-bottom and takes the first row where the mask turns "ground" and
    *stays* ground for a sustained run below it (a lone stray true/false
    pixel from sky/cloud misclassification does not count), giving a rough
    per-column boundary curve that follows the actual silhouette shape -- a
    bumpy treeline, a partial-width ridge, a hazy distant horizon -- without
    assuming it is a straight line or spans the full frame. Short isolated
    runs of "found something" (a misclassified cloud wisp with sky on both
    sides) are dropped since they are not part of the main boundary. Columns
    with no sustained ground below them at all are left at ``-1``, meaning
    "unknown" -- the caller must not restrict matches on those columns.
    """

    height, width = mask.shape[:2]
    sustain = max(4, int(round(height * 0.05)))
    if sustain >= height:
        return np.full(width, -1, dtype=np.int32)
    cumulative = np.cumsum(mask.astype(np.int32), axis=0)
    cumulative = np.vstack([np.zeros((1, width), dtype=np.int32), cumulative])
    windowed = cumulative[sustain:] - cumulative[: height - sustain + 1]
    threshold = sustain * 0.85
    satisfied = windowed >= threshold
    has_boundary = satisfied.any(axis=0)
    first_row = satisfied.argmax(axis=0)
    boundary = np.where(has_boundary, first_row, -1).astype(np.int32)

    min_segment = max(3, int(round(width * 0.03)))
    valid = boundary >= 0
    column = 0
    while column < width:
        if not bool(valid[column]):
            column += 1
            continue
        start = column
        while column < width and bool(valid[column]):
            column += 1
        if (column - start) < min_segment:
            boundary[start:column] = -1
    return boundary


def _line_support(gray: Any, slope: float, intercept: float, np: Any) -> tuple[float, float]:
    height, width = gray.shape[:2]
    gradient = np.abs(np.gradient(gray.astype(np.float32), axis=0))
    xs = np.linspace(0.0, float(max(0, width - 1)), 96)
    ys = slope * xs + intercept
    valid = (ys >= 1.0) & (ys < height - 1.0)
    if not bool(valid.any()):
        return 0.0, float("inf")
    x_indices = np.clip(np.rint(xs[valid]).astype(int), 0, width - 1)
    y_indices = np.clip(np.rint(ys[valid]).astype(int), 0, height - 1)
    values = gradient[y_indices, x_indices]
    reference = float(np.percentile(gradient, 85))
    threshold = max(4.0, reference * 0.35)
    support = float(np.mean(values >= threshold)) if len(values) else 0.0
    return support, float(np.mean(values)) if len(values) else 0.0


def _expected_line_score(
    slope: float,
    intercept: float,
    expected_gravity_camera_rdf: Sequence[float] | None,
    camera_matrix: Any,
    np: Any,
) -> float:
    if expected_gravity_camera_rdf is None:
        return 0.5
    vector = np.asarray(tuple(expected_gravity_camera_rdf), dtype=np.float64).reshape(-1)
    matrix = _matrix(camera_matrix, np)
    if vector.size != 3 or matrix is None or abs(float(vector[1])) < 1.0e-9:
        return 0.5
    # l_pixel = K^-T g.  Only compare the predicted line at the left edge and
    # slope; the sign is irrelevant for a geometric line.
    try:
        line = np.linalg.inv(matrix).T @ vector
        if abs(float(line[1])) < 1.0e-9:
            return 0.5
        expected_slope = -float(line[0]) / float(line[1])
        expected_intercept = -float(line[2]) / float(line[1])
        slope_delta = min(1.0, abs(slope - expected_slope) / 0.5)
        height = max(1.0, float(matrix[1, 2]) * 2.0)
        intercept_delta = min(1.0, abs(intercept - expected_intercept) / height)
        return max(0.0, 1.0 - 0.65 * slope_delta - 0.35 * intercept_delta)
    except Exception:
        return 0.5


def detect_horizon(
    image: Any,
    *,
    camera_matrix: Any = None,
    distortion_coefficients: Any = None,
    expected_gravity_camera_rdf: Sequence[float] | None = None,
) -> AutoSelectionResult:
    """Suggest a two-endpoint sky/terrain boundary on one frozen frame."""

    method = "undistort-gradient-hough-robust-line"
    try:
        cv2, np = _require_cv_numpy()
        gray = _as_gray(image, cv2, np)
        corrected = _undistort_gray(gray, camera_matrix, distortion_coefficients, cv2, np)
        height, width = corrected.shape[:2]
        if width < 32 or height < 24:
            return _failure("horizon", method, "image_too_small")
        blurred = cv2.GaussianBlur(corrected, (5, 5), 0)
        vertical_gradient = np.abs(cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=3))
        # Per-column maxima are effective for a clean synthetic horizon and
        # make the fallback independent of a particular Hough threshold.
        xs = np.linspace(0.0, float(width - 1), min(256, max(32, width // 2)))
        samples: list[Point2] = []
        lower = max(2, int(round(height * 0.06)))
        upper = min(height - 3, int(round(height * 0.90)))
        if upper <= lower:
            return _failure("horizon", method, "invalid_search_band")
        for x in xs:
            column = vertical_gradient[lower:upper + 1, int(round(x))]
            if len(column) == 0:
                continue
            y = lower + int(np.argmax(column))
            strength = float(column[y - lower])
            if strength >= 3.0:
                samples.append((float(x), float(y)))
        fitted = _fit_line(samples, np)
        candidates: list[tuple[float, float, float, float, int, str]] = []
        if fitted is not None:
            slope, intercept, residuals = fitted
            support, magnitude = _line_support(corrected, slope, intercept, np)
            expected = _expected_line_score(
                slope, intercept, expected_gravity_camera_rdf, camera_matrix, np
            )
            score = 0.42 * support + 0.24 * min(1.0, len(samples) / 96.0) + 0.20 * expected
            score += 0.14 * min(1.0, magnitude / 35.0)
            candidates.append((score, slope, intercept, float(np.sqrt(np.mean(residuals ** 2))), len(residuals), "gradient"))

        edges = cv2.Canny(blurred, 40, 120)
        raw_lines = cv2.HoughLinesP(
            edges,
            1.0,
            np.pi / 180.0,
            threshold=max(18, int(round(width * 0.08))),
            minLineLength=max(24, int(round(width * 0.20))),
            maxLineGap=max(8, int(round(width * 0.04))),
        )
        if raw_lines is not None:
            for row in raw_lines.reshape(-1, 4):
                x1, y1, x2, y2 = (float(value) for value in row)
                dx = x2 - x1
                if abs(dx) < max(8.0, width * 0.08):
                    continue
                slope = (y2 - y1) / dx
                intercept = y1 - slope * x1
                support, magnitude = _line_support(corrected, slope, intercept, np)
                expected = _expected_line_score(
                    slope, intercept, expected_gravity_camera_rdf, camera_matrix, np
                )
                length_fraction = math.hypot(dx, y2 - y1) / max(1.0, math.hypot(width, height))
                score = 0.48 * support + 0.25 * min(1.0, length_fraction * 2.0) + 0.17 * expected
                score += 0.10 * min(1.0, magnitude / 35.0)
                candidates.append((score, slope, intercept, 0.0, max(2, int(round(length_fraction * width))), "hough"))

        if not candidates:
            return _failure("horizon", method, "no_skyline_gradient_or_hough_line")
        candidates.sort(key=lambda row: (row[0], row[4]), reverse=True)
        score, slope, intercept, fit_residual, count, source = candidates[0]
        endpoints_ideal = _line_endpoints(slope, intercept, width)
        raw_endpoints = _raw_from_ideal(
            endpoints_ideal, camera_matrix, distortion_coefficients, cv2, np
        )
        # A fit with too little support is useful diagnostic evidence but not a
        # safe automatic annotation.  Keep the points visible only as a
        # low-confidence suggestion.
        confidence = min(1.0, max(0.0, float(score)))
        competing = False
        if len(candidates) > 1:
            second_score, second_slope, second_intercept = candidates[1][0:3]
            center_x = (width - 1) * 0.5
            line_separation = abs(
                (slope * center_x + intercept)
                - (second_slope * center_x + second_intercept)
            )
            competing = (
                float(score) - float(second_score) < 0.08
                and line_separation > max(8.0, height * 0.06)
            )
        state = "GOOD" if confidence >= 0.58 and count >= 4 and not competing else "LOW_CONFIDENCE"
        if competing:
            reason = "multiple_skyline_candidates_conflict"
        else:
            reason = "" if state == "GOOD" else "skyline_line_support_below_acceptance_threshold"
        if fit_residual <= 0.0:
            fit_residual = 1.0 / max(1.0, math.sqrt(float(count)))
        return AutoSelectionResult(
            mode="horizon",
            state=state,
            detection_method=f"{method}/{source}",
            points=raw_endpoints,
            candidate_count=len(candidates),
            inlier_count=count,
            residual_px=fit_residual,
            confidence=confidence,
            spatial_rank=2 if width > 1 else 1,
            eligible=state == "GOOD",
            failure_reason=reason,
            metadata={
                "search_band_v_px": [lower, upper],
                "ideal_line": [slope, 1.0, intercept],
                "gradient_sample_count": len(samples),
            },
        )
    except Exception as exc:
        return _failure("horizon", method, str(exc))


def _spatial_rank(points: Sequence[Point2], np: Any) -> tuple[int, float, float]:
    if len(points) < 2:
        return 0, 0.0, 0.0
    array = np.asarray(points, dtype=np.float64)
    centered = array - array.mean(axis=0, keepdims=True)
    singular = np.linalg.svd(centered, compute_uv=False)
    if len(singular) < 2 or float(singular[0]) <= 1.0e-9:
        return 0, 0.0, 0.0
    ratio = float(singular[1] / singular[0])
    rank = 2 if ratio >= 0.08 else 1
    area = float((np.max(array[:, 0]) - np.min(array[:, 0])) * (np.max(array[:, 1]) - np.min(array[:, 1])))
    return rank, ratio, area


def _select_spread_matches(
    left: Any,
    right: Any,
    distances: Sequence[float],
    *,
    width: int,
    height: int,
    np: Any,
    max_points: int = 12,
) -> tuple[list[Point2], list[Point2], list[float]]:
    order = sorted(range(len(distances)), key=lambda index: float(distances[index]))
    selected: list[int] = []
    cells: set[tuple[int, int]] = set()
    grid_x, grid_y = 4, 3
    # Once roughly two thirds of the budget is spent on distinct grid cells,
    # let the rest fill in from the same cells rather than staying empty --
    # preserves the original 8-of-12 ratio at any configured max_points.
    relax_after = max(1, (max_points * 2) // 3)
    for index in order:
        point = left[index]
        cell = (
            min(grid_x - 1, max(0, int(float(point[0]) / max(1.0, width) * grid_x))),
            min(grid_y - 1, max(0, int(float(point[1]) / max(1.0, height) * grid_y))),
        )
        if any(
            float(np.linalg.norm(np.asarray(point) - np.asarray(left[chosen]))) < 3.0
            or float(np.linalg.norm(np.asarray(right[index]) - np.asarray(right[chosen]))) < 3.0
            for chosen in selected
        ):
            continue
        if cell in cells and len(selected) < relax_after:
            continue
        selected.append(index)
        cells.add(cell)
        if len(selected) >= max_points:
            break
    # Fill remaining slots by descriptor quality when the image occupies only
    # a few grid cells.  This keeps the four-point minimum usable for a small
    # but textured crop while still preferring coverage.
    for index in order:
        if any(
            float(np.linalg.norm(np.asarray(left[index]) - np.asarray(left[chosen]))) < 3.0
            or float(np.linalg.norm(np.asarray(right[index]) - np.asarray(right[chosen]))) < 3.0
            for chosen in selected
        ):
            continue
        if index not in selected:
            selected.append(index)
        if len(selected) >= min(max_points, len(order)):
            break
    return (
        [(float(left[index][0]), float(left[index][1])) for index in selected],
        [(float(right[index][0]), float(right[index][1])) for index in selected],
        [float(distances[index]) for index in selected],
    )


def _empty_mountain(reason: str, method: str = "sift/ratio-mutual-ransac") -> MountainAutoSelection:
    empty = _failure("mountain", method, reason)
    return MountainAutoSelection(
        left=empty,
        right=empty,
        state="NO_DATA",
        detection_method=method,
        failure_reason=reason,
    )


def _sample_line_endpoints(points: Sequence[Point2], count: int = 8) -> tuple[Point2, ...]:
    if len(points) < 2:
        return ()
    first, second = points[0], points[-1]
    return tuple(
        (
            first[0] + (second[0] - first[0]) * index / float(count - 1),
            first[1] + (second[1] - first[1]) * index / float(count - 1),
        )
        for index in range(count)
    )


def _skyline_mountain_fallback(
    left_image: Any,
    right_image: Any,
    *,
    left_camera_matrix: Any,
    right_camera_matrix: Any,
    left_distortion_coefficients: Any,
    right_distortion_coefficients: Any,
    max_points: int = 8,
) -> MountainAutoSelection:
    """Return visible-but-unaccepted skyline samples for textureless scenes."""

    left_horizon = detect_horizon(
        left_image,
        camera_matrix=left_camera_matrix,
        distortion_coefficients=left_distortion_coefficients,
    )
    right_horizon = detect_horizon(
        right_image,
        camera_matrix=right_camera_matrix,
        distortion_coefficients=right_distortion_coefficients,
    )
    left_points = _sample_line_endpoints(left_horizon.points, count=max_points)
    right_points = _sample_line_endpoints(right_horizon.points, count=max_points)
    if len(left_points) < 4 or len(right_points) < 4:
        return _empty_mountain(
            "no_texture_matches_and_no_pairable_skyline_gradient",
            "skyline-gradient-fallback",
        )
    warning = (
        "skyline gradient fallback is a visual proposal only; operator review is required "
        "before any paired Mountain solve"
    )
    left_result = AutoSelectionResult(
        mode="mountain",
        state="LOW_CONFIDENCE",
        detection_method="skyline-gradient-fallback",
        points=left_points,
        candidate_count=2,
        inlier_count=0,
        residual_px=left_horizon.residual_px,
        confidence=min(0.45, left_horizon.confidence),
        spatial_rank=1,
        eligible=False,
        failure_reason="skyline_fallback_requires_operator_review",
        warnings=(warning,),
    )
    right_result = AutoSelectionResult(
        mode="mountain",
        state="LOW_CONFIDENCE",
        detection_method="skyline-gradient-fallback",
        points=right_points,
        candidate_count=2,
        inlier_count=0,
        residual_px=right_horizon.residual_px,
        confidence=min(0.45, right_horizon.confidence),
        spatial_rank=1,
        eligible=False,
        failure_reason="skyline_fallback_requires_operator_review",
        warnings=(warning,),
    )
    return MountainAutoSelection(
        left=left_result,
        right=right_result,
        state="LOW_CONFIDENCE",
        detection_method="skyline-gradient-fallback",
        match_count=0,
        inlier_count=0,
        residual_px=max(
            value for value in (left_horizon.residual_px, right_horizon.residual_px) if value is not None
        ),
        confidence=min(left_result.confidence, right_result.confidence),
        eligible=False,
        failure_reason="skyline_fallback_requires_operator_review",
        warnings=(warning,),
        metadata={"source": "horizon_gradient", "pair_count": len(left_points)},
    )


def detect_mountain(
    left_image: Any,
    right_image: Any,
    *,
    left_camera_matrix: Any = None,
    right_camera_matrix: Any = None,
    left_distortion_coefficients: Any = None,
    right_distortion_coefficients: Any = None,
    baseline_ned_m: Sequence[float] | None = None,
    prior_rotation_right_from_left_rdf: Sequence[Sequence[float]] | None = None,
    ridge_margin_fraction: float = 0.06,
    max_points: int = 12,
) -> MountainAutoSelection:
    """Suggest distributed left/right ridge correspondences without 3-D data.

    ``ridge_margin_fraction`` is the half-width of the boundary-curve match
    band, as a fraction of image height (default 0.06 = 6%). Wider tolerates
    a rougher/less certain boundary estimate at the cost of letting more
    off-ridge matches back in; narrower rejects more aggressively but can
    drop legitimate points on a very jagged silhouette.

    ``max_points`` caps how many spatially-spread correspondences are kept
    for the final suggestion (default 12); it never discards inliers beyond
    what is needed to build that spread selection.
    """

    method = "sift/ratio-mutual-ransac"
    max_points = max(4, int(max_points))
    try:
        cv2, np = _require_cv_numpy()
        left_gray = _undistort_gray(
            _as_gray(left_image, cv2, np),
            left_camera_matrix,
            left_distortion_coefficients,
            cv2,
            np,
        )
        right_gray = _undistort_gray(
            _as_gray(right_image, cv2, np),
            right_camera_matrix,
            right_distortion_coefficients,
            cv2,
            np,
        )
        height, width = left_gray.shape[:2]
        if width < 32 or height < 24 or right_gray.shape[:2] != left_gray.shape[:2]:
            return _empty_mountain("left_right_image_dimensions_are_incompatible", method)

        detector_name = "SIFT"
        if hasattr(cv2, "SIFT_create"):
            detector = cv2.SIFT_create(nfeatures=1800, contrastThreshold=0.01)
            norm = cv2.NORM_L2
        else:
            detector_name = "ORB"
            detector = cv2.ORB_create(nfeatures=1800, fastThreshold=7)
            norm = cv2.NORM_HAMMING
        key_left, descriptor_left = detector.detectAndCompute(left_gray, None)
        key_right, descriptor_right = detector.detectAndCompute(right_gray, None)
        if descriptor_left is None or descriptor_right is None:
            return _skyline_mountain_fallback(
                left_image,
                right_image,
                left_camera_matrix=left_camera_matrix,
                right_camera_matrix=right_camera_matrix,
                left_distortion_coefficients=left_distortion_coefficients,
                right_distortion_coefficients=right_distortion_coefficients,
                max_points=max_points,
            )
        matcher = cv2.BFMatcher(norm, crossCheck=False)
        forward = matcher.knnMatch(descriptor_left, descriptor_right, k=2)
        reverse = matcher.knnMatch(descriptor_right, descriptor_left, k=2)
        ratio = 0.76 if detector_name == "SIFT" else 0.82
        forward_good = {
            int(pair[0].queryIdx): pair[0]
            for pair in forward
            if len(pair) >= 2 and float(pair[0].distance) <= ratio * float(pair[1].distance)
        }
        reverse_good = {
            int(pair[0].queryIdx): pair[0]
            for pair in reverse
            if len(pair) >= 2 and float(pair[0].distance) <= ratio * float(pair[1].distance)
        }
        matches = [
            match
            for query_index, match in forward_good.items()
            if int(match.trainIdx) in reverse_good
            and int(reverse_good[int(match.trainIdx)].trainIdx) == query_index
        ]
        # Bias toward the ridge: SIFT/ORB has no notion of "near the
        # sky/terrain boundary", so a strongly-textured sky (sunset clouds)
        # can win over the ridge itself on plain keypoint strength, and even
        # once sky is excluded, open foreground ground and a silhouette's own
        # interior both still register as plain "not sky" keypoints, letting
        # matches scatter across a tree canopy's leaves or a nearby field
        # instead of tracing its edge. Prefer matches within a margin of each
        # side's own estimated boundary curve (the actual silhouette shape);
        # fall back to plain "not sky" and then to the unfiltered set as each
        # estimate becomes unavailable or leaves too few points, matching
        # this module's rule that a possibly-off-ridge suggestion beats none.
        left_ground_mask = _estimate_ground_mask(left_gray, cv2, np)
        right_ground_mask = _estimate_ground_mask(right_gray, cv2, np)
        if left_ground_mask is not None or right_ground_mask is not None:
            left_boundary = None if left_ground_mask is None else _estimate_ridge_boundary(left_ground_mask, np)
            right_boundary = None if right_ground_mask is None else _estimate_ridge_boundary(right_ground_mask, np)
            margin_fraction = min(0.25, max(0.01, float(ridge_margin_fraction)))
            boundary_margin_px = margin_fraction * max(left_gray.shape[0], right_gray.shape[0])

            def _on_ground(point: Point2, mask: Any | None) -> bool:
                if mask is None:
                    return True
                mask_height, mask_width = mask.shape[:2]
                x = int(round(point[0]))
                y = int(round(point[1]))
                if not (0 <= x < mask_width and 0 <= y < mask_height):
                    return True
                return bool(mask[y, x])

            def _near_boundary(point: Point2, boundary: Any | None) -> bool:
                if boundary is None:
                    return True
                width_columns = boundary.shape[0]
                x = min(width_columns - 1, max(0, int(round(point[0]))))
                row = int(boundary[x])
                if row < 0:
                    return True
                return abs(point[1] - row) <= boundary_margin_px

            # Proximity to the estimated boundary curve alone is not enough:
            # the curve itself can be locally wrong at a handful of columns
            # (an isolated cloud-edge misclassification, or an unstable
            # estimate near the image border), and a keypoint can happen to
            # sit within margin of that wrong value while still being sky at
            # its own pixel. Require both the point itself to be ground-like
            # and to sit near the boundary curve.
            boundary_matches = [
                match
                for match in matches
                if _near_boundary(key_left[match.queryIdx].pt, left_boundary)
                and _near_boundary(key_right[match.trainIdx].pt, right_boundary)
                and _on_ground(key_left[match.queryIdx].pt, left_ground_mask)
                and _on_ground(key_right[match.trainIdx].pt, right_ground_mask)
            ]
            if len(boundary_matches) >= 4:
                matches = boundary_matches
            else:
                ridge_matches = [
                    match
                    for match in matches
                    if _on_ground(key_left[match.queryIdx].pt, left_ground_mask)
                    and _on_ground(key_right[match.trainIdx].pt, right_ground_mask)
                ]
                if len(ridge_matches) >= 4:
                    matches = ridge_matches
        if len(matches) < 4:
            return _skyline_mountain_fallback(
                left_image,
                right_image,
                left_camera_matrix=left_camera_matrix,
                right_camera_matrix=right_camera_matrix,
                left_distortion_coefficients=left_distortion_coefficients,
                right_distortion_coefficients=right_distortion_coefficients,
                max_points=max_points,
            )
        left_points = np.asarray(
            [key_left[match.queryIdx].pt for match in matches], dtype=np.float64
        )
        right_points = np.asarray(
            [key_right[match.trainIdx].pt for match in matches], dtype=np.float64
        )
        left_matrix = _matrix(left_camera_matrix, np)
        right_matrix = _matrix(right_camera_matrix, np)
        left_distortion = _distortion(left_distortion_coefficients, np)
        right_distortion = _distortion(right_distortion_coefficients, np)
        if left_matrix is not None:
            left_norm = cv2.undistortPoints(left_points.reshape(-1, 1, 2), left_matrix, left_distortion).reshape(-1, 2)
        else:
            left_norm = left_points.copy()
        if right_matrix is not None:
            right_norm = cv2.undistortPoints(right_points.reshape(-1, 1, 2), right_matrix, right_distortion).reshape(-1, 2)
        else:
            right_norm = right_points.copy()

        mask = None
        essential = None
        if len(matches) >= 5:
            try:
                essential, mask = cv2.findEssentialMat(
                    left_norm,
                    right_norm,
                    np.eye(3, dtype=np.float64),
                    method=cv2.RANSAC,
                    prob=0.999,
                    threshold=0.004,
                )
            except Exception:
                mask = None
        if mask is None:
            try:
                _fundamental, mask = cv2.findFundamentalMat(
                    left_norm,
                    right_norm,
                    cv2.FM_RANSAC,
                    0.004,
                    0.999,
                )
            except Exception:
                mask = None
        if mask is None:
            inlier_indices = list(range(len(matches)))
        else:
            flat_mask = np.asarray(mask).reshape(-1)
            inlier_indices = [index for index, value in enumerate(flat_mask[: len(matches)]) if int(value) != 0]
        if len(inlier_indices) < 4:
            return _skyline_mountain_fallback(
                left_image,
                right_image,
                left_camera_matrix=left_camera_matrix,
                right_camera_matrix=right_camera_matrix,
                left_distortion_coefficients=left_distortion_coefficients,
                right_distortion_coefficients=right_distortion_coefficients,
                max_points=max_points,
            )

        inlier_left = left_points[inlier_indices]
        inlier_right = right_points[inlier_indices]
        inlier_distances = np.asarray(
            [float(matches[index].distance) for index in inlier_indices], dtype=np.float64
        )
        selected_left, selected_right, selected_distances = _select_spread_matches(
            inlier_left,
            inlier_right,
            inlier_distances,
            width=width,
            height=height,
            np=np,
            max_points=max_points,
        )
        # Lower SIFT/ORB descriptor distance means a stronger appearance
        # match. Normalize against the whole geometrically-consistent inlier
        # pool (not just the spread-selected subset) so a point's score
        # reflects how it compares to every RANSAC-accepted match, not only
        # to the handful of other points shown alongside it.
        if selected_distances:
            worst = float(np.max(inlier_distances)) if inlier_distances.size else float(max(selected_distances))
            best = float(np.min(inlier_distances)) if inlier_distances.size else float(min(selected_distances))
            spread = max(1.0e-6, worst - best)
            point_confidences = tuple(
                min(1.0, max(0.0, 1.0 - (float(distance) - best) / spread)) for distance in selected_distances
            )
        else:
            point_confidences = ()
        rank, rank_ratio, area = _spatial_rank(selected_left, np)
        # Epipolar residual in normalized coordinates, converted to a useful
        # approximate pixel scale for the UI.  Do not call this a PnP or
        # terrain reprojection error.
        selected_left_array = np.asarray(selected_left, dtype=np.float64)
        selected_right_array = np.asarray(selected_right, dtype=np.float64)
        selected_left_norm = (
            cv2.undistortPoints(selected_left_array.reshape(-1, 1, 2), left_matrix, left_distortion).reshape(-1, 2)
            if left_matrix is not None
            else selected_left_array
        )
        selected_right_norm = (
            cv2.undistortPoints(selected_right_array.reshape(-1, 1, 2), right_matrix, right_distortion).reshape(-1, 2)
            if right_matrix is not None
            else selected_right_array
        )
        disparity = np.linalg.norm(selected_left_norm - selected_right_norm, axis=1)
        focal = 1.0
        if left_matrix is not None and right_matrix is not None:
            focal = math.sqrt(
                max(1.0, float(left_matrix[0, 0]) * float(left_matrix[1, 1]))
                * max(1.0, float(right_matrix[0, 0]) * float(right_matrix[1, 1]))
            ) ** 0.25
        residual_px = float(np.sqrt(np.mean(disparity ** 2)) * focal)
        coverage = min(1.0, math.sqrt(max(0.0, area)) / max(1.0, math.hypot(width, height)))
        inlier_ratio = min(1.0, len(inlier_indices) / max(1.0, len(matches)))
        count_score = min(1.0, len(inlier_indices) / 12.0)
        residual_score = max(0.0, 1.0 - residual_px / 8.0)
        confidence = 0.35 * inlier_ratio + 0.25 * coverage + 0.20 * count_score + 0.20 * residual_score
        eligible = len(selected_left) >= 4 and rank >= 2
        if not eligible and rank < 2:
            reason = "selected_correspondences_are_near_collinear"
        elif len(selected_left) < 4:
            reason = "selected_correspondences_below_four"
        elif confidence < 0.52:
            reason = "match_confidence_below_acceptance_threshold"
        else:
            reason = ""
        state = "GOOD" if eligible and confidence >= 0.52 else "LOW_CONFIDENCE"
        warnings: list[str] = [
            "Mountain suggestion is image correspondence evidence only; no 3-D terrain/PnP is inferred",
        ]
        if baseline_ned_m is None:
            warnings.append("GNSS baseline was not supplied to auto-selection; solver still requires current GNSS baseline")
        if prior_rotation_right_from_left_rdf is None:
            warnings.append("relative prior rotation was not supplied; RANSAC used image epipolar geometry only")
        side_kwargs = {
            "detection_method": f"{detector_name}/{method.split('/', 1)[1]}",
            "candidate_count": len(matches),
            "match_count": len(matches),
            "inlier_count": len(inlier_indices),
            "residual_px": residual_px,
            "confidence": confidence,
            "spatial_rank": rank,
            "eligible": eligible and state == "GOOD",
            "failure_reason": reason,
            "warnings": warnings,
            "point_confidences": point_confidences,
            "metadata": {
                "inlier_ratio": inlier_ratio,
                "coverage_score": coverage,
                "spatial_rank_ratio": rank_ratio,
                "spatial_bbox_area_px2": area,
                "descriptor": detector_name,
                "baseline_supplied": baseline_ned_m is not None,
                "prior_rotation_supplied": prior_rotation_right_from_left_rdf is not None,
            },
        }
        left_result = AutoSelectionResult(
            mode="mountain", state=state, points=tuple(selected_left), **side_kwargs
        )
        right_result = AutoSelectionResult(
            mode="mountain", state=state, points=tuple(selected_right), **side_kwargs
        )
        return MountainAutoSelection(
            left=left_result,
            right=right_result,
            state=state,
            detection_method=f"{detector_name}/{method.split('/', 1)[1]}",
            match_count=len(matches),
            inlier_count=len(inlier_indices),
            residual_px=residual_px,
            confidence=confidence,
            eligible=eligible and state == "GOOD",
            failure_reason=reason,
            warnings=tuple(warnings),
            metadata={
                "descriptor": detector_name,
                "essential_matrix_found": essential is not None,
                "baseline_ned_m": None if baseline_ned_m is None else list(baseline_ned_m),
                "prior_rotation_supplied": prior_rotation_right_from_left_rdf is not None,
            },
        )
    except Exception as exc:
        return _empty_mountain(str(exc), method)


# Descriptive aliases used by callers that prefer the action wording.
auto_select_horizon = detect_horizon
auto_select_mountain = detect_mountain


__all__ = [
    "AutoSelectionResult",
    "MountainAutoSelection",
    "auto_select_horizon",
    "auto_select_mountain",
    "detect_horizon",
    "detect_mountain",
]
