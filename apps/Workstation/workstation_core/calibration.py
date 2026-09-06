from __future__ import annotations

import json
import hashlib
import math
import random
import statistics
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


CalibrationProgressCallback = Callable[[int, int, str], None]

STEREO_EXTRINSICS_FRAME_CONTRACT = (
    "opencv_optical:RDF(right,down,forward); "
    "X_right = R_left_to_right * X_left + T_left_to_right_m"
)


@dataclass(frozen=True)
class CheckerboardConfig:
    inner_cols: int
    inner_rows: int
    square_size_m: float

    @property
    def pattern_size(self) -> tuple[int, int]:
        return (self.inner_cols, self.inner_rows)


@dataclass(frozen=True)
class CheckerboardDetection:
    found: bool
    corner_count: int
    image_size: tuple[int, int] | None = None
    error: str | None = None


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    camera_matrix: list[list[float]]
    distortion_coefficients: list[float]
    reprojection_error_px: float
    image_count: int


@dataclass(frozen=True)
class CheckerboardObservation:
    """Serializable checkerboard corners cached before camera optimization.

    Image points use decoded-image pixels with ``x`` increasing right and
    ``y`` increasing down. Object points use the checkerboard plane in metres
    (``x`` across columns, ``y`` across rows, ``z=0``). No Qt image object or
    OpenCV/numpy array is retained.
    """

    view_id: str
    image_size: tuple[int, int]
    object_points_m: tuple[tuple[float, float, float], ...]
    image_points_px: tuple[tuple[float, float], ...]
    view_geometry: dict[str, Any]


@dataclass(frozen=True)
class CheckerboardObservationCache:
    """Detection output shared by the primary and deterministic subset solves."""

    image_size: tuple[int, int]
    pattern_size: tuple[int, int]
    observations: tuple[CheckerboardObservation, ...]
    input_count: int
    rejected_paths: tuple[str, ...] = ()


@dataclass(frozen=True)
class CameraCalibrationSolve:
    """Camera intrinsics plus OpenCV uncertainty and stability evidence.

    ``subset_estimates`` intentionally uses the generic mapping contract
    consumed by :mod:`calibration_quality`: ``subset_id``, ``fx_px``,
    ``fy_px``, ``cx_px``, ``cy_px``, and ``view_count``. Extra membership and
    failure evidence stays in separate fields so the quality layer need not
    depend on this solver module.
    """

    intrinsics: CameraIntrinsics
    observations: tuple[CheckerboardObservation, ...]
    intrinsic_standard_deviations: dict[str, float | None]
    extrinsic_standard_deviations: tuple[dict[str, Any], ...]
    per_view_errors_px: tuple[float, ...]
    view_geometry: tuple[dict[str, Any], ...]
    subset_estimates: tuple[dict[str, Any], ...] = ()
    subset_membership: tuple[dict[str, Any], ...] = ()
    subset_failures: tuple[dict[str, str], ...] = ()
    subset_summary: dict[str, float | int] = field(default_factory=dict)
    calibration_flags: int = 0
    effective_view_ids: tuple[str, ...] = ()

    @property
    def subset_results(self) -> tuple[dict[str, Any], ...]:
        """Compatibility-friendly alias for successful subset estimates."""

        return self.subset_estimates

    def quality_inputs(self) -> dict[str, Any]:
        """Return arguments accepted by ``evaluate_camera_calibration_quality``."""

        return {
            "camera_matrix": [list(row) for row in self.intrinsics.camera_matrix],
            "image_size": [self.intrinsics.width, self.intrinsics.height],
            "views": [dict(row) for row in self.view_geometry],
            "subset_estimates": [dict(row) for row in self.subset_estimates],
            "intrinsic_standard_deviations": dict(self.intrinsic_standard_deviations),
            "per_view_errors_px": list(self.per_view_errors_px),
        }


@dataclass(frozen=True)
class StereoExtrinsics:
    rotation_left_to_right: list[list[float]]
    translation_left_to_right_m: list[float]
    reprojection_error_px: float
    pair_count: int
    frame_contract: str = STEREO_EXTRINSICS_FRAME_CONTRACT


@dataclass(frozen=True)
class CalibrationImageDiagnostic:
    """Per-image checkerboard evidence used for review and outlier filtering."""

    side: str
    path: str
    capture_key: str
    found: bool
    corner_count: int
    reprojection_error_px: float | None
    width: int | None
    height: int | None
    board_center_x_px: float | None
    board_center_y_px: float | None
    board_coverage_fraction: float | None
    error: str = ""


@dataclass(frozen=True)
class StereoIntrinsicsResult:
    left: CameraIntrinsics
    right: CameraIntrinsics
    board: CheckerboardConfig
    extrinsics: StereoExtrinsics | None = None
    image_diagnostics: tuple[dict[str, Any], ...] = ()
    left_quality: dict[str, Any] | None = None
    right_quality: dict[str, Any] | None = None
    calibration_quality: dict[str, Any] | None = None
    solver_diagnostics: dict[str, Any] | None = None

    @property
    def quality_snapshot(self) -> dict[str, Any] | None:
        return None if self.calibration_quality is None else dict(self.calibration_quality)


def validate_intrinsics_result(result: StereoIntrinsicsResult) -> tuple[str, ...]:
    """Return hard K/D raster-contract errors independent of saved quality claims."""

    errors: list[str] = []
    for side, intrinsics in (("left", result.left), ("right", result.right)):
        if intrinsics.width <= 0 or intrinsics.height <= 0:
            errors.append(f"{side}: image size must be positive")
        matrix = intrinsics.camera_matrix
        if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
            errors.append(f"{side}: camera_matrix must be 3x3")
            continue
        values = [value for row in matrix for value in row]
        if not all(math.isfinite(float(value)) for value in values):
            errors.append(f"{side}: camera_matrix contains a non-finite value")
        elif float(matrix[0][0]) <= 0.0 or float(matrix[1][1]) <= 0.0:
            errors.append(f"{side}: fx and fy must be positive")
        else:
            cx = float(matrix[0][2])
            cy = float(matrix[1][2])
            if not 0.0 <= cx < float(intrinsics.width):
                errors.append(f"{side}: cx={cx:.3f}px is outside width={intrinsics.width}px")
            if not 0.0 <= cy < float(intrinsics.height):
                errors.append(f"{side}: cy={cy:.3f}px is outside height={intrinsics.height}px")
        if not all(
            math.isfinite(float(value)) for value in intrinsics.distortion_coefficients
        ):
            errors.append(f"{side}: distortion coefficients contain a non-finite value")
    return tuple(errors)


def _sha256_json_payload(payload: object) -> str:
    canonical = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def _validate_stereo_extrinsics(extrinsics: StereoExtrinsics) -> None:
    rotation = extrinsics.rotation_left_to_right
    translation = extrinsics.translation_left_to_right_m
    if len(rotation) != 3 or any(len(row) != 3 for row in rotation):
        raise ValueError("rotation_left_to_right must be 3x3")
    values = [float(value) for row in rotation for value in row]
    values.extend(float(value) for value in translation)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("stereo extrinsics must contain only finite values")
    if len(translation) != 3:
        raise ValueError("translation_left_to_right_m must contain 3 values")
    for left_column in range(3):
        for right_column in range(3):
            dot = sum(
                float(rotation[row][left_column]) * float(rotation[row][right_column])
                for row in range(3)
            )
            expected = 1.0 if left_column == right_column else 0.0
            if abs(dot - expected) > 1.0e-6:
                raise ValueError("rotation_left_to_right must be orthonormal within 1e-6")
    determinant = (
        float(rotation[0][0])
        * (float(rotation[1][1]) * float(rotation[2][2]) - float(rotation[1][2]) * float(rotation[2][1]))
        - float(rotation[0][1])
        * (float(rotation[1][0]) * float(rotation[2][2]) - float(rotation[1][2]) * float(rotation[2][0]))
        + float(rotation[0][2])
        * (float(rotation[1][0]) * float(rotation[2][1]) - float(rotation[1][1]) * float(rotation[2][0]))
    )
    if abs(determinant - 1.0) > 1.0e-6:
        raise ValueError("rotation_left_to_right determinant must equal +1 within 1e-6")
    if not math.isfinite(float(extrinsics.reprojection_error_px)):
        raise ValueError("stereo reprojection_error_px must be finite")
    if int(extrinsics.pair_count) < 0:
        raise ValueError("stereo pair_count must be non-negative")
    if str(extrinsics.frame_contract) != STEREO_EXTRINSICS_FRAME_CONTRACT:
        raise ValueError("stereo extrinsics frame_contract does not match OpenCV optical RDF left-to-right")


@dataclass(frozen=True)
class StereoExtrinsicsProfile:
    """Independent stereo R/T profile with explicit calibration provenance.

    ``source_zoom_level`` records where R/T was estimated.  It is metadata,
    not an application gate: callers may intentionally combine this profile
    with intrinsics from another zoom level.
    """

    extrinsics: StereoExtrinsics
    source_zoom_level: int | None = None
    method: str = "checkerboard_external_only"
    source_intrinsics_path: str = ""
    source_intrinsics_sha256: str = ""
    profile_id: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        _validate_stereo_extrinsics(self.extrinsics)
        if self.source_zoom_level is not None and int(self.source_zoom_level) < 0:
            raise ValueError("source_zoom_level must be non-negative")
        object.__setattr__(
            self,
            "source_zoom_level",
            None if self.source_zoom_level is None else int(self.source_zoom_level),
        )
        object.__setattr__(self, "method", str(self.method or "unknown"))
        object.__setattr__(self, "source_intrinsics_path", str(self.source_intrinsics_path))
        object.__setattr__(self, "source_intrinsics_sha256", str(self.source_intrinsics_sha256))
        object.__setattr__(self, "profile_id", str(self.profile_id))
        object.__setattr__(self, "metadata", dict(self.metadata))
        digest = self.source_intrinsics_sha256
        if digest and (
            len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise ValueError("source_intrinsics_sha256 must be a lowercase SHA-256 digest")

    @property
    def right_camera_origin_in_left_rdf_m(self) -> tuple[float, float, float]:
        rotation = self.extrinsics.rotation_left_to_right
        translation = self.extrinsics.translation_left_to_right_m
        return tuple(
            -sum(float(rotation[column][row]) * float(translation[column]) for column in range(3))
            for row in range(3)
        )

    @property
    def baseline_m(self) -> float:
        return math.sqrt(sum(value * value for value in self.right_camera_origin_in_left_rdf_m))

    @property
    def content_sha256(self) -> str:
        return _sha256_json_payload(asdict(self.extrinsics))


def intrinsics_content_sha256(result: StereoIntrinsicsResult) -> str:
    """Hash only K/D and board fields, deliberately excluding stereo R/T."""

    return _sha256_json_payload(
        {
            "left": asdict(result.left),
            "right": asdict(result.right),
            "board": asdict(result.board),
        }
    )


def independent_zoom_selection_mode(
    intrinsics_zoom: int | None,
    extrinsics_zoom: int | None,
) -> str:
    """Classify independently selected K/D and R/T zoom provenance."""

    if intrinsics_zoom is None or extrinsics_zoom is None:
        return "manual_independent_zoom"
    if int(intrinsics_zoom) == int(extrinsics_zoom):
        return "manual_same_zoom"
    return "manual_cross_zoom"


def compose_stereo_calibration(
    intrinsics: StereoIntrinsicsResult,
    extrinsics: StereoExtrinsics | StereoExtrinsicsProfile | None,
) -> StereoIntrinsicsResult:
    """Compose independently selected K/D and R/T without a zoom gate."""

    selected = extrinsics.extrinsics if isinstance(extrinsics, StereoExtrinsicsProfile) else extrinsics
    if selected is not None:
        _validate_stereo_extrinsics(selected)
    return StereoIntrinsicsResult(
        left=intrinsics.left,
        right=intrinsics.right,
        board=intrinsics.board,
        extrinsics=selected,
        image_diagnostics=tuple(intrinsics.image_diagnostics),
        left_quality=(
            None if intrinsics.left_quality is None else dict(intrinsics.left_quality)
        ),
        right_quality=(
            None if intrinsics.right_quality is None else dict(intrinsics.right_quality)
        ),
        calibration_quality=(
            None
            if intrinsics.calibration_quality is None
            else dict(intrinsics.calibration_quality)
        ),
        solver_diagnostics=(
            None
            if intrinsics.solver_diagnostics is None
            else dict(intrinsics.solver_diagnostics)
        ),
    )


def without_stereo_extrinsics(result: StereoIntrinsicsResult) -> StereoIntrinsicsResult:
    return compose_stereo_calibration(result, None)


def opencv_available() -> tuple[bool, str | None]:
    try:
        import cv2  # noqa: F401
        import numpy  # noqa: F401
    except Exception as exc:  # pragma: no cover - depends on runtime binary packages.
        return False, str(exc)
    return True, None


def _load_grayscale(path: Path):
    import cv2

    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"could not read image: {path}")
    return image


def detect_checkerboard(path: str | Path, config: CheckerboardConfig) -> CheckerboardDetection:
    try:
        import cv2

        image = _load_grayscale(Path(path))
        found, corners = cv2.findChessboardCorners(image, config.pattern_size, None)
        return CheckerboardDetection(
            found=bool(found),
            corner_count=0 if corners is None else int(len(corners)),
            image_size=(int(image.shape[1]), int(image.shape[0])),
        )
    except Exception as exc:
        return CheckerboardDetection(found=False, corner_count=0, error=str(exc))


def _object_points(config: CheckerboardConfig):
    import numpy as np

    points = np.zeros((config.inner_rows * config.inner_cols, 3), np.float32)
    grid = np.mgrid[0 : config.inner_cols, 0 : config.inner_rows].T.reshape(-1, 2)
    points[:, :2] = grid * float(config.square_size_m)
    return points


def _calibration_capture_key(path: Path, side: str) -> str:
    prefix = f"{side}_"
    return path.stem.removeprefix(prefix) if path.stem.startswith(prefix) else path.stem


def checkerboard_observation_from_points(
    *,
    view_id: str,
    image_size: tuple[int, int],
    pattern_size: tuple[int, int],
    object_points_m: Sequence[Sequence[float]],
    image_points_px: Sequence[Sequence[float]],
) -> CheckerboardObservation:
    """Build one reusable observation and its quality-layer geometry mapping.

    This constructor is useful for independently detected corners and for
    synthetic regression fixtures. It deliberately accepts plain sequences
    and stores plain tuples/lists so worker boundaries never carry a ``QImage``
    or mutable numpy view.
    """

    from .calibration_quality import extract_view_geometry

    width, height = (int(image_size[0]), int(image_size[1]))
    inner_cols, inner_rows = (int(pattern_size[0]), int(pattern_size[1]))
    if width <= 0 or height <= 0:
        raise ValueError("image_size must contain positive width and height")
    if inner_cols < 2 or inner_rows < 2:
        raise ValueError("pattern_size must contain at least 2 columns and 2 rows")

    object_points = tuple(
        tuple(float(value) for value in point)
        for point in object_points_m
    )
    image_points = tuple(
        tuple(float(value) for value in point)
        for point in image_points_px
    )
    expected_count = inner_cols * inner_rows
    if len(object_points) != expected_count or len(image_points) != expected_count:
        raise ValueError(
            "checkerboard point count mismatch: "
            f"expected {expected_count}, object={len(object_points)}, image={len(image_points)}"
        )
    if any(len(point) != 3 for point in object_points):
        raise ValueError("each checkerboard object point must contain x, y, z")
    if any(len(point) != 2 for point in image_points):
        raise ValueError("each checkerboard image point must contain x, y")
    if not all(math.isfinite(value) for point in object_points for value in point):
        raise ValueError("checkerboard object points must be finite")
    if not all(math.isfinite(value) for point in image_points for value in point):
        raise ValueError("checkerboard image points must be finite")
    geometry = extract_view_geometry(
        image_points,
        (width, height),
        pattern_size=(inner_cols, inner_rows),
        view_id=str(view_id),
    ).to_mapping()
    return CheckerboardObservation(
        view_id=str(view_id),
        image_size=(width, height),
        object_points_m=object_points,
        image_points_px=image_points,
        view_geometry=geometry,
    )


def cache_checkerboard_observations(
    paths: Sequence[str | Path],
    config: CheckerboardConfig,
    *,
    progress_callback: CalibrationProgressCallback | None = None,
) -> CheckerboardObservationCache:
    """Detect/refine checkerboards once and return reusable plain-data evidence."""

    import cv2
    import numpy as np

    observations: list[CheckerboardObservation] = []
    rejected_paths: list[str] = []
    objp = _object_points(config)
    image_size: tuple[int, int] | None = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    total_paths = len(paths)

    for index, raw_path in enumerate(paths, start=1):
        path = Path(raw_path)
        image = _load_grayscale(path)
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = current_size
        elif image_size != current_size:
            raise ValueError(f"image size mismatch: {path} has {current_size}, expected {image_size}")
        found, corners = cv2.findChessboardCorners(image, config.pattern_size, None)
        if found and corners is not None:
            refined = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), criteria)
            observations.append(
                checkerboard_observation_from_points(
                    view_id=str(path),
                    image_size=current_size,
                    pattern_size=config.pattern_size,
                    object_points_m=np.asarray(objp).reshape(-1, 3).tolist(),
                    image_points_px=np.asarray(refined).reshape(-1, 2).tolist(),
                )
            )
        else:
            rejected_paths.append(str(path))
        if progress_callback is not None:
            progress_callback(index, total_paths, "detect")

    if image_size is None:
        raise ValueError("no images supplied")
    return CheckerboardObservationCache(
        image_size=image_size,
        pattern_size=config.pattern_size,
        observations=tuple(observations),
        input_count=total_paths,
        rejected_paths=tuple(rejected_paths),
    )


def analyze_calibration_image(
    path: str | Path,
    config: CheckerboardConfig,
    intrinsics: CameraIntrinsics,
    *,
    side: str,
    observation: CheckerboardObservation | None = None,
) -> CalibrationImageDiagnostic:
    """Calculate an independent per-image checkerboard residual.

    The main calibration result intentionally keeps a global RMS. This review
    pass solves the board pose for each image using the calculated K,D and
    reports per-view RMS, board coverage, and detection state without changing
    the camera model.

    ``observation`` lets a caller that already ran checkerboard detection for
    this exact image (e.g. ``CameraCalibrationSolve.observations`` from the
    intrinsics solve) pass the cached corners through, so the image is not
    re-detected from scratch for this independent diagnostic pass. It is only
    used when its ``view_id`` matches ``path``; otherwise detection runs as
    before.
    """

    import cv2
    import numpy as np

    image_path = Path(path)
    capture_key = _calibration_capture_key(image_path, side)
    cached = observation if observation is not None and observation.view_id == str(image_path) else None
    try:
        if cached is not None:
            width, height = int(cached.image_size[0]), int(cached.image_size[1])
            corner_count = len(cached.image_points_px)
            points = np.asarray(cached.image_points_px, dtype=np.float64).reshape(-1, 2)
            solve_points = np.asarray(cached.image_points_px, dtype=np.float32).reshape(-1, 1, 2)
            object_points = np.asarray(cached.object_points_m, dtype=np.float32).reshape(-1, 3)
        else:
            image = _load_grayscale(image_path)
            height, width = int(image.shape[0]), int(image.shape[1])
            found, corners = cv2.findChessboardCorners(image, config.pattern_size, None)
            if not found or corners is None:
                return CalibrationImageDiagnostic(
                    side=str(side),
                    path=str(image_path),
                    capture_key=capture_key,
                    found=False,
                    corner_count=0 if corners is None else int(len(corners)),
                    reprojection_error_px=None,
                    width=width,
                    height=height,
                    board_center_x_px=None,
                    board_center_y_px=None,
                    board_coverage_fraction=None,
                    error="checkerboard not found",
                )

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            refined = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), criteria)
            corner_count = int(len(corners))
            points = np.asarray(refined, dtype=np.float64).reshape(-1, 2)
            solve_points = refined
            object_points = _object_points(config)

        board_center = points.mean(axis=0)
        min_xy = points.min(axis=0)
        max_xy = points.max(axis=0)
        coverage = max(
            0.0,
            float((max_xy[0] - min_xy[0]) * (max_xy[1] - min_xy[1]))
            / max(1.0, float(width * height)),
        )
        camera_matrix = np.asarray(intrinsics.camera_matrix, dtype=np.float64)
        distortion = np.asarray(intrinsics.distortion_coefficients, dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            solve_points,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return CalibrationImageDiagnostic(
                side=str(side),
                path=str(image_path),
                capture_key=capture_key,
                found=True,
                corner_count=corner_count,
                reprojection_error_px=None,
                width=width,
                height=height,
                board_center_x_px=float(board_center[0]),
                board_center_y_px=float(board_center[1]),
                board_coverage_fraction=coverage,
                error="solvePnP failed",
            )
        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            camera_matrix,
            distortion,
        )
        projected_points = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
        residuals = np.linalg.norm(points - projected_points, axis=1)
        rms = float(np.sqrt(np.mean(np.square(residuals))))
        return CalibrationImageDiagnostic(
            side=str(side),
            path=str(image_path),
            capture_key=capture_key,
            found=True,
            corner_count=corner_count,
            reprojection_error_px=rms,
            width=width,
            height=height,
            board_center_x_px=float(board_center[0]),
            board_center_y_px=float(board_center[1]),
            board_coverage_fraction=coverage,
        )
    except Exception as exc:
        return CalibrationImageDiagnostic(
            side=str(side),
            path=str(image_path),
            capture_key=capture_key,
            found=False,
            corner_count=0,
            reprojection_error_px=None,
            width=None,
            height=None,
            board_center_x_px=None,
            board_center_y_px=None,
            board_coverage_fraction=None,
            error=str(exc),
        )


def analyze_calibration_images(
    paths: list[str | Path],
    config: CheckerboardConfig,
    intrinsics: CameraIntrinsics,
    *,
    side: str,
    observations: Sequence[CheckerboardObservation] | None = None,
) -> tuple[dict[str, Any], ...]:
    """Run the independent per-image diagnostic, reusing cached detections.

    ``observations`` is typically ``CameraCalibrationSolve.observations`` from
    the intrinsics solve for this side; images already detected there are not
    re-detected here (see :func:`analyze_calibration_image`).
    """

    observation_by_view_id = (
        {observation.view_id: observation for observation in observations}
        if observations is not None
        else {}
    )
    return tuple(
        asdict(
            analyze_calibration_image(
                path,
                config,
                intrinsics,
                side=side,
                observation=observation_by_view_id.get(str(Path(path))),
            )
        )
        for path in paths
    )


def calibrate_camera_from_images(
    paths: list[str | Path],
    config: CheckerboardConfig,
    *,
    progress_callback: CalibrationProgressCallback | None = None,
) -> CameraIntrinsics:
    """Compatibility wrapper returning only the established intrinsics type."""

    return calibrate_camera_with_diagnostics(
        paths,
        config,
        progress_callback=progress_callback,
        subset_runs=0,
    ).intrinsics


def _run_extended_camera_calibration(
    observations: Sequence[CheckerboardObservation],
    image_size: tuple[int, int],
    *,
    calibration_flags: int,
):
    import cv2
    import numpy as np

    if len(observations) < 3:
        raise ValueError(f"need at least 3 valid checkerboard images, found {len(observations)}")
    width, height = (int(image_size[0]), int(image_size[1]))
    if width <= 0 or height <= 0:
        raise ValueError("image_size must contain positive width and height")

    object_points = []
    image_points = []
    for observation in observations:
        if tuple(observation.image_size) != (width, height):
            raise ValueError(
                f"image size mismatch: {observation.view_id} has {observation.image_size}, "
                f"expected {(width, height)}"
            )
        object_array = np.asarray(observation.object_points_m, dtype=np.float32)
        image_array = np.asarray(observation.image_points_px, dtype=np.float32)
        if object_array.ndim != 2 or object_array.shape[1:] != (3,):
            raise ValueError(f"invalid checkerboard object points: {observation.view_id}")
        if image_array.ndim != 2 or image_array.shape[1:] != (2,):
            raise ValueError(f"invalid checkerboard image points: {observation.view_id}")
        if len(object_array) != len(image_array):
            raise ValueError(f"checkerboard point count mismatch: {observation.view_id}")
        if not np.isfinite(object_array).all() or not np.isfinite(image_array).all():
            raise ValueError(f"checkerboard points must be finite: {observation.view_id}")
        object_points.append(object_array.reshape(-1, 3))
        image_points.append(image_array.reshape(-1, 1, 2))

    result = cv2.calibrateCameraExtended(
        object_points,
        image_points,
        (width, height),
        None,
        None,
        flags=int(calibration_flags),
    )
    if len(result) != 8:  # pragma: no cover - OpenCV API contract guard.
        raise RuntimeError(f"unexpected calibrateCameraExtended result length: {len(result)}")
    return result


def _intrinsics_and_view_errors_from_extended(
    observations: Sequence[CheckerboardObservation],
    image_size: tuple[int, int],
    extended_result,
) -> tuple[CameraIntrinsics, tuple[float, ...]]:
    import cv2
    import numpy as np

    ret, camera_matrix, distortion, rvecs, tvecs, _std_i, _std_e, raw_per_view = extended_result
    numeric_values = np.concatenate(
        (
            np.asarray(camera_matrix, dtype=np.float64).reshape(-1),
            np.asarray(distortion, dtype=np.float64).reshape(-1),
        )
    )
    if not np.isfinite(numeric_values).all():
        raise ValueError("camera calibration returned non-finite intrinsics")

    total_error = 0.0
    total_points = 0
    calculated_view_errors: list[float] = []
    for observation, rvec, tvec in zip(observations, rvecs, tvecs):
        obj = np.asarray(observation.object_points_m, dtype=np.float32).reshape(-1, 3)
        img = np.asarray(observation.image_points_px, dtype=np.float32).reshape(-1, 1, 2)
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        error = cv2.norm(img, projected, cv2.NORM_L2)
        total_error += error * error
        total_points += len(obj)
        calculated_view_errors.append(math.sqrt((error * error) / max(1, len(obj))))
    reprojection_error = math.sqrt(total_error / total_points) if total_points else float(ret)

    reported_view_errors = tuple(
        float(value) for value in np.asarray(raw_per_view, dtype=np.float64).reshape(-1)
    )
    per_view_errors = (
        reported_view_errors
        if len(reported_view_errors) == len(observations)
        else tuple(calculated_view_errors)
    )
    intrinsics = CameraIntrinsics(
        width=image_size[0],
        height=image_size[1],
        camera_matrix=[[float(value) for value in row] for row in camera_matrix.tolist()],
        distortion_coefficients=[float(value) for value in np.ravel(distortion).tolist()],
        reprojection_error_px=float(reprojection_error),
        image_count=len(observations),
    )
    return intrinsics, per_view_errors


_INTRINSIC_STDDEV_NAMES = (
    "fx_px",
    "fy_px",
    "cx_px",
    "cy_px",
    "k1",
    "k2",
    "p1",
    "p2",
    "k3",
    "k4",
    "k5",
    "k6",
    "s1",
    "s2",
    "s3",
    "s4",
    "tau_x",
    "tau_y",
)


def _intrinsic_standard_deviation_mapping(extended_result) -> dict[str, float | None]:
    import numpy as np

    values = np.asarray(extended_result[5], dtype=np.float64).reshape(-1)
    return {
        name: float(value) if math.isfinite(float(value)) else None
        for name, value in zip(_INTRINSIC_STDDEV_NAMES, values)
    }


def _extrinsic_standard_deviation_mappings(
    observations: Sequence[CheckerboardObservation],
    extended_result,
) -> tuple[dict[str, Any], ...]:
    import numpy as np

    values = np.asarray(extended_result[6], dtype=np.float64).reshape(-1)
    rows: list[dict[str, Any]] = []

    def finite_or_none(value: float) -> float | None:
        numeric = float(value)
        return numeric if math.isfinite(numeric) else None

    for index, observation in enumerate(observations):
        offset = index * 6
        if offset + 6 > len(values):
            break
        rows.append(
            {
                "view_id": observation.view_id,
                "rvec_x_rad_stddev": finite_or_none(values[offset]),
                "rvec_y_rad_stddev": finite_or_none(values[offset + 1]),
                "rvec_z_rad_stddev": finite_or_none(values[offset + 2]),
                "tvec_x_m_stddev": finite_or_none(values[offset + 3]),
                "tvec_y_m_stddev": finite_or_none(values[offset + 4]),
                "tvec_z_m_stddev": finite_or_none(values[offset + 5]),
                "axis_contract": (
                    "OpenCV optical camera RDF (x right, y down, z forward); "
                    "checkerboard object x across columns, y across rows, z=0"
                ),
            }
        )
    return tuple(rows)


def _normalized_observation_features(
    observations: Sequence[CheckerboardObservation],
) -> tuple[tuple[float, ...], ...]:
    raw_features: list[tuple[float, ...]] = []
    for observation in observations:
        geometry = observation.view_geometry
        center = geometry.get("center_normalized", (0.5, 0.5))
        top_bottom_ratio = max(float(geometry.get("top_bottom_ratio", 1.0)), 1.0e-12)
        left_right_ratio = max(float(geometry.get("left_right_ratio", 1.0)), 1.0e-12)
        raw_features.append(
            (
                float(center[0]),
                float(center[1]),
                float(geometry.get("scale", 0.0)),
                float(geometry.get("roll_deg", 0.0)) / 90.0,
                math.log(top_bottom_ratio),
                math.log(left_right_ratio),
            )
        )
    if not raw_features:
        return ()
    columns = tuple(zip(*raw_features))
    minima = tuple(min(column) for column in columns)
    spans = tuple(max(column) - minimum for column, minimum in zip(columns, minima))
    return tuple(
        tuple(
            0.0 if span <= 1.0e-12 else (value - minimum) / span
            for value, minimum, span in zip(feature, minima, spans)
        )
        for feature in raw_features
    )


def _coverage_preserving_subset_indices(
    observations: Sequence[CheckerboardObservation],
    *,
    runs: int,
    fraction: float,
    seed: int,
) -> tuple[tuple[int, ...], ...]:
    """Select deterministic, geometry-spread subsets via farthest-point sampling."""

    if runs < 0:
        raise ValueError("subset_runs must be non-negative")
    if not 0.0 < float(fraction) <= 1.0:
        raise ValueError("subset_fraction must be in (0, 1]")
    count = len(observations)
    if runs == 0 or count < 4:
        return ()
    subset_count = min(count, max(3, int(math.ceil(count * float(fraction)))))
    if subset_count == count:
        return (tuple(range(count)),)

    features = _normalized_observation_features(observations)
    generator = random.Random(int(seed))
    selected_subsets: list[tuple[int, ...]] = []
    seen: set[tuple[int, ...]] = set()
    maximum_attempts = max(runs * 30, count * 10)
    for _attempt in range(maximum_attempts):
        tie_order = list(range(count))
        generator.shuffle(tie_order)
        tie_rank = {index: rank for rank, index in enumerate(tie_order)}
        feature_weights = tuple(
            0.75 + 0.5 * generator.random()
            for _axis in range(len(features[0]))
        )
        selected = [tie_order[0]]
        while len(selected) < subset_count:
            remaining = (index for index in range(count) if index not in selected)

            def spread_score(candidate: int) -> tuple[float, int]:
                minimum_distance = min(
                    sum(
                        feature_weights[axis]
                        * (features[candidate][axis] - features[chosen][axis]) ** 2
                        for axis in range(len(features[candidate]))
                    )
                    for chosen in selected
                )
                return (minimum_distance, -tie_rank[candidate])

            selected.append(max(remaining, key=spread_score))
        subset = tuple(sorted(selected))
        if subset in seen:
            continue
        seen.add(subset)
        selected_subsets.append(subset)
        if len(selected_subsets) >= runs:
            break
    return tuple(selected_subsets)


def _subset_summary(
    estimates: Sequence[dict[str, Any]],
    failures: Sequence[dict[str, str]],
    image_size: tuple[int, int],
) -> dict[str, float | int]:
    summary: dict[str, float | int] = {
        "successful_subset_count": len(estimates),
        "failed_subset_count": len(failures),
    }
    if not estimates:
        return summary

    for key in ("fx_px", "fy_px", "cx_px", "cy_px"):
        values = [float(row[key]) for row in estimates]
        prefix = key.removesuffix("_px")
        summary[f"{prefix}_min_px"] = min(values)
        summary[f"{prefix}_max_px"] = max(values)
        summary[f"{prefix}_span_px"] = max(values) - min(values)
        summary[f"{prefix}_stddev_px"] = statistics.pstdev(values)

    fx_values = [float(row["fx_px"]) for row in estimates]
    fy_values = [float(row["fy_px"]) for row in estimates]
    fx_mean = abs(statistics.fmean(fx_values))
    fy_mean = abs(statistics.fmean(fy_values))
    summary["fx_relative_span"] = (
        float(summary["fx_span_px"]) / fx_mean if fx_mean > 0.0 else math.inf
    )
    summary["fy_relative_span"] = (
        float(summary["fy_span_px"]) / fy_mean if fy_mean > 0.0 else math.inf
    )
    summary["cx_span_fraction_of_width"] = float(summary["cx_span_px"]) / image_size[0]
    summary["cy_span_fraction_of_height"] = float(summary["cy_span_px"]) / image_size[1]
    return summary


def solve_camera_from_observations(
    cache: CheckerboardObservationCache,
    *,
    subset_runs: int = 12,
    subset_fraction: float = 0.75,
    subset_seed: int = 0,
    calibration_flags: int = 0,
    progress_callback: CalibrationProgressCallback | None = None,
) -> CameraCalibrationSolve:
    """Optimize one cached observation set and deterministic stability subsets.

    The default OpenCV flags leave the principal point free, matching the
    historical solver. Callers may explicitly provide other flags, but this
    function never enables ``CALIB_FIX_PRINCIPAL_POINT`` on their behalf.
    """

    import numpy as np

    observations = tuple(cache.observations)
    for observation in observations:
        if tuple(observation.image_size) != tuple(cache.image_size):
            raise ValueError(
                f"image size mismatch: {observation.view_id} has {observation.image_size}, "
                f"expected {cache.image_size}"
            )
    from .calibration_quality import (
        CalibrationViewGeometry,
        select_pose_representatives,
    )

    representative_views = select_pose_representatives(
        CalibrationViewGeometry.from_mapping(observation.view_geometry)
        for observation in observations
    )
    representative_ids = {view.view_id for view in representative_views}
    effective_observations = tuple(
        observation for observation in observations if observation.view_id in representative_ids
    )
    if len(effective_observations) < 3:
        raise ValueError(
            "need at least 3 geometrically distinct checkerboard poses, "
            f"found {len(effective_observations)} from {len(observations)} detections"
        )
    extended = _run_extended_camera_calibration(
        effective_observations,
        cache.image_size,
        calibration_flags=int(calibration_flags),
    )
    intrinsics, per_view_errors = _intrinsics_and_view_errors_from_extended(
        effective_observations,
        cache.image_size,
        extended,
    )

    subsets = _coverage_preserving_subset_indices(
        effective_observations,
        runs=int(subset_runs),
        fraction=float(subset_fraction),
        seed=int(subset_seed),
    )
    subset_estimates: list[dict[str, Any]] = []
    subset_membership: list[dict[str, Any]] = []
    subset_failures: list[dict[str, str]] = []
    for run_index, selected_indices in enumerate(subsets):
        subset_id = f"coverage-{run_index:03d}"
        selected_observations = tuple(effective_observations[index] for index in selected_indices)
        subset_membership.append(
            {
                "subset_id": subset_id,
                "strategy": "deterministic_geometry_farthest_point",
                "selected_indices": list(selected_indices),
                "selected_view_ids": [row.view_id for row in selected_observations],
            }
        )
        try:
            subset_extended = _run_extended_camera_calibration(
                selected_observations,
                cache.image_size,
                calibration_flags=int(calibration_flags),
            )
            subset_intrinsics, _errors = _intrinsics_and_view_errors_from_extended(
                selected_observations,
                cache.image_size,
                subset_extended,
            )
            matrix = np.asarray(subset_intrinsics.camera_matrix, dtype=np.float64)
            estimate_values = (
                matrix[0, 0],
                matrix[1, 1],
                matrix[0, 2],
                matrix[1, 2],
            )
            if not np.isfinite(estimate_values).all():
                raise ValueError("subset camera matrix contains non-finite values")
            subset_estimates.append(
                {
                    "subset_id": subset_id,
                    "fx_px": float(matrix[0, 0]),
                    "fy_px": float(matrix[1, 1]),
                    "cx_px": float(matrix[0, 2]),
                    "cy_px": float(matrix[1, 2]),
                    "view_count": len(selected_observations),
                }
            )
        except Exception as exc:
            subset_failures.append({"subset_id": subset_id, "error": str(exc)})

    if progress_callback is not None:
        progress_callback(cache.input_count, cache.input_count, "optimize")
    return CameraCalibrationSolve(
        intrinsics=intrinsics,
        observations=observations,
        intrinsic_standard_deviations=_intrinsic_standard_deviation_mapping(extended),
        extrinsic_standard_deviations=_extrinsic_standard_deviation_mappings(
            effective_observations,
            extended,
        ),
        per_view_errors_px=per_view_errors,
        view_geometry=tuple(dict(observation.view_geometry) for observation in observations),
        subset_estimates=tuple(subset_estimates),
        subset_membership=tuple(subset_membership),
        subset_failures=tuple(subset_failures),
        subset_summary=_subset_summary(subset_estimates, subset_failures, cache.image_size),
        calibration_flags=int(calibration_flags),
        effective_view_ids=tuple(observation.view_id for observation in effective_observations),
    )


def calibrate_camera_with_diagnostics(
    paths: Sequence[str | Path],
    config: CheckerboardConfig,
    *,
    progress_callback: CalibrationProgressCallback | None = None,
    subset_runs: int = 12,
    subset_fraction: float = 0.75,
    subset_seed: int = 0,
    calibration_flags: int = 0,
) -> CameraCalibrationSolve:
    """Detect once, solve with ``calibrateCameraExtended``, and report stability."""

    cache = cache_checkerboard_observations(
        paths,
        config,
        progress_callback=progress_callback,
    )
    return solve_camera_from_observations(
        cache,
        subset_runs=subset_runs,
        subset_fraction=subset_fraction,
        subset_seed=subset_seed,
        calibration_flags=calibration_flags,
        progress_callback=progress_callback,
    )


def calibrate_stereo_extrinsics_from_image_pairs(
    left_paths: list[str | Path],
    right_paths: list[str | Path],
    config: CheckerboardConfig,
    left_intrinsics: CameraIntrinsics,
    right_intrinsics: CameraIntrinsics,
    *,
    progress_callback: CalibrationProgressCallback | None = None,
    left_observations: Sequence[CheckerboardObservation] | None = None,
    right_observations: Sequence[CheckerboardObservation] | None = None,
) -> StereoExtrinsics:
    """Solve stereo R/T from matched checkerboard pairs.

    ``left_observations``/``right_observations`` are typically the
    ``CameraCalibrationSolve.observations`` already produced by the intrinsics
    solve for each side. When a pair's images both have a cached observation
    (matched by ``view_id`` against the resolved path), the cached corners are
    reused instead of re-running checkerboard detection on the image.
    """

    import cv2
    import numpy as np

    left_observation_by_view_id = (
        {observation.view_id: observation for observation in left_observations}
        if left_observations is not None
        else {}
    )
    right_observation_by_view_id = (
        {observation.view_id: observation for observation in right_observations}
        if right_observations is not None
        else {}
    )

    def paths_by_stem(paths: list[str | Path], *, side: str) -> dict[str, Path]:
        result = {}
        for raw_path in paths:
            path = Path(raw_path)
            key = _calibration_capture_key(path, side)
            result[key] = path
        return result

    left_by_stem = paths_by_stem(left_paths, side="left")
    right_by_stem = paths_by_stem(right_paths, side="right")
    pair_keys = sorted(set(left_by_stem) & set(right_by_stem))
    if not pair_keys and len(left_paths) == len(right_paths):
        # Explicit Compute selections are ordered left/right lists. Raw WIRIS
        # FTP names can contain different camera-local clock suffixes even
        # when they belong to the same trigger round, so matching by the
        # selected row is the only available pairing contract in that case.
        left_by_stem = {
            f"ordered-{index:06d}": Path(path)
            for index, path in enumerate(left_paths)
        }
        right_by_stem = {
            f"ordered-{index:06d}": Path(path)
            for index, path in enumerate(right_paths)
        }
        pair_keys = sorted(left_by_stem)
    if not pair_keys:
        raise ValueError("no left/right checkerboard image pairs with matching capture ids")

    object_points = []
    left_image_points = []
    right_image_points = []
    objp = _object_points(config)
    image_size: tuple[int, int] | None = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    total_pairs = len(pair_keys)
    for index, key in enumerate(pair_keys, start=1):
        left_path = left_by_stem[key]
        right_path = right_by_stem[key]
        left_observation = left_observation_by_view_id.get(str(left_path))
        right_observation = right_observation_by_view_id.get(str(right_path))
        if left_observation is not None and right_observation is not None:
            current_size = (int(left_observation.image_size[0]), int(left_observation.image_size[1]))
            right_size = (int(right_observation.image_size[0]), int(right_observation.image_size[1]))
            if current_size != right_size:
                raise ValueError(
                    f"image size mismatch for pair {key}: left={current_size}, right={right_size}"
                )
            if image_size is None:
                image_size = current_size
            elif image_size != current_size:
                raise ValueError(f"image size mismatch for pair {key}: {current_size}, expected {image_size}")
            object_points.append(
                np.asarray(left_observation.object_points_m, dtype=np.float32).reshape(-1, 3)
            )
            left_image_points.append(
                np.asarray(left_observation.image_points_px, dtype=np.float32).reshape(-1, 1, 2)
            )
            right_image_points.append(
                np.asarray(right_observation.image_points_px, dtype=np.float32).reshape(-1, 1, 2)
            )
        else:
            left_image = _load_grayscale(left_path)
            right_image = _load_grayscale(right_path)
            current_size = (int(left_image.shape[1]), int(left_image.shape[0]))
            right_size = (int(right_image.shape[1]), int(right_image.shape[0]))
            if current_size != right_size:
                raise ValueError(
                    f"image size mismatch for pair {key}: left={current_size}, right={right_size}"
                )
            if image_size is None:
                image_size = current_size
            elif image_size != current_size:
                raise ValueError(f"image size mismatch for pair {key}: {current_size}, expected {image_size}")

            left_found, left_corners = cv2.findChessboardCorners(left_image, config.pattern_size, None)
            right_found, right_corners = cv2.findChessboardCorners(right_image, config.pattern_size, None)
            if left_found and right_found and left_corners is not None and right_corners is not None:
                left_refined = cv2.cornerSubPix(left_image, left_corners, (11, 11), (-1, -1), criteria)
                right_refined = cv2.cornerSubPix(right_image, right_corners, (11, 11), (-1, -1), criteria)
                object_points.append(objp)
                left_image_points.append(left_refined)
                right_image_points.append(right_refined)
        if progress_callback is not None:
            progress_callback(index, total_pairs, "detect")

    if image_size is None:
        raise ValueError("no images supplied")
    if len(object_points) < 3:
        raise ValueError(f"need at least 3 valid stereo checkerboard pairs, found {len(object_points)}")

    left_matrix = np.array(left_intrinsics.camera_matrix, dtype=np.float64)
    right_matrix = np.array(right_intrinsics.camera_matrix, dtype=np.float64)
    left_distortion = np.array(left_intrinsics.distortion_coefficients, dtype=np.float64)
    right_distortion = np.array(right_intrinsics.distortion_coefficients, dtype=np.float64)
    rms, _cm1, _dc1, _cm2, _dc2, rotation, translation, _essential, _fundamental = cv2.stereoCalibrate(
        object_points,
        left_image_points,
        right_image_points,
        left_matrix,
        left_distortion,
        right_matrix,
        right_distortion,
        image_size,
        flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=criteria,
    )
    if progress_callback is not None:
        progress_callback(total_pairs, total_pairs, "optimize")
    return StereoExtrinsics(
        rotation_left_to_right=[[float(value) for value in row] for row in rotation.tolist()],
        translation_left_to_right_m=[float(value) for value in np.ravel(translation).tolist()],
        reprojection_error_px=float(rms),
        pair_count=len(object_points),
    )


def save_intrinsics_json(path: str | Path, result: StereoIntrinsicsResult, *, metadata: dict[str, Any] | None = None) -> None:
    saved_metadata = dict(metadata or {})
    if result.image_diagnostics:
        saved_metadata.setdefault("image_diagnostics", list(result.image_diagnostics))
    payload = {
        "left": result.left.__dict__,
        "right": result.right.__dict__,
        "board": result.board.__dict__,
        "metadata": saved_metadata,
    }
    if result.extrinsics is not None:
        payload["extrinsics"] = result.extrinsics.__dict__
    if result.left_quality is not None:
        payload["left_quality"] = dict(result.left_quality)
    if result.right_quality is not None:
        payload["right_quality"] = dict(result.right_quality)
    if result.calibration_quality is not None:
        payload["calibration_quality"] = dict(result.calibration_quality)
    if result.solver_diagnostics is not None:
        payload["solver_diagnostics"] = dict(result.solver_diagnostics)
    Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False))


def camera_intrinsics_from_dict(payload: dict[str, Any]) -> CameraIntrinsics:
    return CameraIntrinsics(
        width=int(payload["width"]),
        height=int(payload["height"]),
        camera_matrix=[[float(value) for value in row] for row in payload["camera_matrix"]],
        distortion_coefficients=[float(value) for value in payload.get("distortion_coefficients", [])],
        reprojection_error_px=float(payload.get("reprojection_error_px", 0.0)),
        image_count=int(payload.get("image_count", 0)),
    )


def stereo_extrinsics_from_dict(payload: dict[str, Any]) -> StereoExtrinsics:
    result = StereoExtrinsics(
        rotation_left_to_right=[
            [float(value) for value in row] for row in payload["rotation_left_to_right"]
        ],
        translation_left_to_right_m=[float(value) for value in payload["translation_left_to_right_m"]],
        reprojection_error_px=float(payload.get("reprojection_error_px", 0.0)),
        pair_count=int(payload.get("pair_count", 0)),
        frame_contract=str(
            payload.get(
                "frame_contract",
                STEREO_EXTRINSICS_FRAME_CONTRACT,
            )
        ),
    )
    _validate_stereo_extrinsics(result)
    return result


def save_stereo_extrinsics_json(
    path: str | Path,
    profile: StereoExtrinsicsProfile,
) -> None:
    output = Path(path)
    payload = {
        "schema_version": 1,
        "profile_type": "stereo_extrinsics",
        "profile_id": profile.profile_id or output.stem,
        "source_zoom_level": profile.source_zoom_level,
        "method": profile.method,
        "source_intrinsics_path": profile.source_intrinsics_path,
        "source_intrinsics_sha256": profile.source_intrinsics_sha256,
        "extrinsics": asdict(profile.extrinsics),
        "derived": {
            "right_camera_origin_in_left_rdf_m": list(profile.right_camera_origin_in_left_rdf_m),
            "baseline_m": profile.baseline_m,
        },
        "metadata": dict(profile.metadata),
    }
    output.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def load_stereo_extrinsics_json(path: str | Path) -> StereoExtrinsicsProfile:
    """Load a standalone profile or an older intrinsics JSON with embedded R/T."""

    input_path = Path(path)
    payload = json.loads(input_path.read_text(encoding="utf-8"))
    raw_extrinsics = payload.get("extrinsics")
    if not isinstance(raw_extrinsics, dict):
        raise ValueError("JSON does not contain stereo extrinsics")
    metadata = payload.get("metadata") if isinstance(payload.get("metadata"), dict) else {}
    source_zoom = payload.get("source_zoom_level", metadata.get("zoom_level"))
    return StereoExtrinsicsProfile(
        extrinsics=stereo_extrinsics_from_dict(raw_extrinsics),
        source_zoom_level=None if source_zoom is None else int(source_zoom),
        method=str(
            payload.get("method")
            or metadata.get("calculation_mode")
            or "legacy_embedded_checkerboard"
        ),
        source_intrinsics_path=str(
            payload.get("source_intrinsics_path")
            or (str(input_path) if "left" in payload and "right" in payload else "")
        ),
        source_intrinsics_sha256=str(payload.get("source_intrinsics_sha256") or ""),
        profile_id=str(payload.get("profile_id") or input_path.stem),
        metadata=dict(metadata),
    )


def load_intrinsics_json(path: str | Path) -> StereoIntrinsicsResult:
    payload = json.loads(Path(path).read_text())
    board_payload = payload.get("board", {})
    board = CheckerboardConfig(
        inner_cols=int(board_payload.get("inner_cols", 0)),
        inner_rows=int(board_payload.get("inner_rows", 0)),
        square_size_m=float(board_payload.get("square_size_m", 0.0)),
    )
    metadata = payload.get("metadata", {})
    raw_diagnostics = metadata.get("image_diagnostics", []) if isinstance(metadata, dict) else []
    diagnostics = tuple(
        dict(row)
        for row in raw_diagnostics
        if isinstance(row, dict)
    ) if isinstance(raw_diagnostics, (list, tuple)) else ()
    result = StereoIntrinsicsResult(
        left=camera_intrinsics_from_dict(payload["left"]),
        right=camera_intrinsics_from_dict(payload["right"]),
        board=board,
        extrinsics=stereo_extrinsics_from_dict(payload["extrinsics"]) if "extrinsics" in payload else None,
        image_diagnostics=diagnostics,
        left_quality=(
            dict(payload["left_quality"])
            if isinstance(payload.get("left_quality"), dict)
            else None
        ),
        right_quality=(
            dict(payload["right_quality"])
            if isinstance(payload.get("right_quality"), dict)
            else None
        ),
        calibration_quality=(
            dict(payload["calibration_quality"])
            if isinstance(payload.get("calibration_quality"), dict)
            else None
        ),
        solver_diagnostics=(
            dict(payload["solver_diagnostics"])
            if isinstance(payload.get("solver_diagnostics"), dict)
            else None
        ),
    )
    errors = validate_intrinsics_result(result)
    if errors:
        raise ValueError("invalid intrinsics profile: " + "; ".join(errors))
    return result
