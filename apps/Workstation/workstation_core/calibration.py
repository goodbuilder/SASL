from __future__ import annotations

import json
import hashlib
import math
from collections.abc import Callable
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
class StereoExtrinsics:
    rotation_left_to_right: list[list[float]]
    translation_left_to_right_m: list[float]
    reprojection_error_px: float
    pair_count: int
    frame_contract: str = STEREO_EXTRINSICS_FRAME_CONTRACT


@dataclass(frozen=True)
class StereoIntrinsicsResult:
    left: CameraIntrinsics
    right: CameraIntrinsics
    board: CheckerboardConfig
    extrinsics: StereoExtrinsics | None = None


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


def calibrate_camera_from_images(
    paths: list[str | Path],
    config: CheckerboardConfig,
    *,
    progress_callback: CalibrationProgressCallback | None = None,
) -> CameraIntrinsics:
    import cv2
    import numpy as np

    object_points = []
    image_points = []
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
            object_points.append(objp)
            image_points.append(refined)
        if progress_callback is not None:
            progress_callback(index, total_paths, "detect")

    if image_size is None:
        raise ValueError("no images supplied")
    if len(object_points) < 3:
        raise ValueError(f"need at least 3 valid checkerboard images, found {len(object_points)}")

    ret, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    if progress_callback is not None:
        progress_callback(total_paths, total_paths, "optimize")
    total_error = 0.0
    total_points = 0
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        error = cv2.norm(img, projected, cv2.NORM_L2)
        total_error += error * error
        total_points += len(obj)
    reprojection_error = math.sqrt(total_error / total_points) if total_points else float(ret)
    return CameraIntrinsics(
        width=image_size[0],
        height=image_size[1],
        camera_matrix=[[float(value) for value in row] for row in camera_matrix.tolist()],
        distortion_coefficients=[float(value) for value in np.ravel(distortion).tolist()],
        reprojection_error_px=float(reprojection_error),
        image_count=len(object_points),
    )


def calibrate_stereo_extrinsics_from_image_pairs(
    left_paths: list[str | Path],
    right_paths: list[str | Path],
    config: CheckerboardConfig,
    left_intrinsics: CameraIntrinsics,
    right_intrinsics: CameraIntrinsics,
    *,
    progress_callback: CalibrationProgressCallback | None = None,
) -> StereoExtrinsics:
    import cv2
    import numpy as np

    def paths_by_stem(paths: list[str | Path]) -> dict[str, Path]:
        result = {}
        for raw_path in paths:
            path = Path(raw_path)
            stem = path.stem
            key = stem.split("_")[-1]
            result[key] = path
        return result

    left_by_stem = paths_by_stem(left_paths)
    right_by_stem = paths_by_stem(right_paths)
    pair_keys = sorted(set(left_by_stem) & set(right_by_stem))
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
        left_image = _load_grayscale(left_by_stem[key])
        right_image = _load_grayscale(right_by_stem[key])
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
    payload = {
        "left": result.left.__dict__,
        "right": result.right.__dict__,
        "board": result.board.__dict__,
        "metadata": metadata or {},
    }
    if result.extrinsics is not None:
        payload["extrinsics"] = result.extrinsics.__dict__
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
    return StereoIntrinsicsResult(
        left=camera_intrinsics_from_dict(payload["left"]),
        right=camera_intrinsics_from_dict(payload["right"]),
        board=board,
        extrinsics=stereo_extrinsics_from_dict(payload["extrinsics"]) if "extrinsics" in payload else None,
    )
