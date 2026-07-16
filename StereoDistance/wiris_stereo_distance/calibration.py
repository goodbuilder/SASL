from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


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
    method: str | None = None
    error: str | None = None


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    camera_matrix: list[list[float]]
    distortion_coefficients: list[float]
    reprojection_error_px: float
    image_count: int
    image_results: list[dict[str, Any]] = field(default_factory=list)
    coverage_summary: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class StereoIntrinsicsResult:
    left: CameraIntrinsics
    right: CameraIntrinsics
    board: CheckerboardConfig
    stereo_calibration: dict[str, Any] | None = None


@dataclass(frozen=True)
class StereoCalibrationExtrinsics:
    rotation_right_to_left: list[list[float]]
    translation_right_in_left_m: tuple[float, float, float]
    baseline_m: float
    source: str
    pair_count: int
    rms_error_px: float | None
    epipolar_mean_error_px: float | None
    epipolar_max_error_px: float | None


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
        found, corners, method = _find_checkerboard(cv2, image, config.pattern_size)
        return CheckerboardDetection(
            found=bool(found),
            corner_count=0 if corners is None else int(len(corners)),
            image_size=(int(image.shape[1]), int(image.shape[0])),
            method=method,
        )
    except Exception as exc:
        return CheckerboardDetection(found=False, corner_count=0, error=str(exc))


def _find_checkerboard(cv2_module: Any, image: Any, pattern_size: tuple[int, int]) -> tuple[bool, Any, str]:
    if hasattr(cv2_module, "findChessboardCornersSB"):
        flags = int(getattr(cv2_module, "CALIB_CB_EXHAUSTIVE", 0)) | int(
            getattr(cv2_module, "CALIB_CB_ACCURACY", 0)
        )
        try:
            found, corners = cv2_module.findChessboardCornersSB(image, pattern_size, flags)
        except Exception:
            found, corners = False, None
        if found and corners is not None:
            return True, corners, "findChessboardCornersSB"

    found, corners = cv2_module.findChessboardCorners(image, pattern_size, None)
    return bool(found), corners, "findChessboardCorners"


def _object_points(config: CheckerboardConfig):
    import numpy as np

    points = np.zeros((config.inner_rows * config.inner_cols, 3), np.float32)
    grid = np.mgrid[0 : config.inner_cols, 0 : config.inner_rows].T.reshape(-1, 2)
    points[:, :2] = grid * float(config.square_size_m)
    return points


def _image_result(
    path: Path,
    *,
    accepted: bool,
    detector_method: str | None = None,
    corner_count: int = 0,
    image_size: tuple[int, int] | None = None,
    reprojection_error_px: float | None = None,
    reject_reason: str | None = None,
    board_coverage: dict[str, Any] | None = None,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "path": str(path),
        "accepted": bool(accepted),
        "detector_method": detector_method,
        "corner_count": int(corner_count),
        "image_size": None if image_size is None else {"width": int(image_size[0]), "height": int(image_size[1])},
        "reprojection_error_px": None if reprojection_error_px is None else float(reprojection_error_px),
        "reject_reason": reject_reason,
        "board_coverage": board_coverage,
    }
    return result


def _board_coverage_from_corners(corners: Any, image_size: tuple[int, int]) -> dict[str, Any]:
    import numpy as np

    width, height = image_size
    points = np.asarray(corners, dtype=float).reshape(-1, 2)
    min_xy = points.min(axis=0)
    max_xy = points.max(axis=0)
    center_xy = points.mean(axis=0)
    bbox_width = max(0.0, float(max_xy[0] - min_xy[0]))
    bbox_height = max(0.0, float(max_xy[1] - min_xy[1]))
    image_area = max(1.0, float(width * height))
    return {
        "center_norm": {
            "x": float(center_xy[0] / max(1, width)),
            "y": float(center_xy[1] / max(1, height)),
        },
        "bbox_norm": {
            "x_min": float(min_xy[0] / max(1, width)),
            "y_min": float(min_xy[1] / max(1, height)),
            "x_max": float(max_xy[0] / max(1, width)),
            "y_max": float(max_xy[1] / max(1, height)),
            "width": float(bbox_width / max(1, width)),
            "height": float(bbox_height / max(1, height)),
        },
        "area_fraction": float((bbox_width * bbox_height) / image_area),
    }


def _coverage_summary(image_results: list[dict[str, Any]]) -> dict[str, Any]:
    accepted = [entry for entry in image_results if entry.get("accepted") and entry.get("board_coverage")]
    warnings: list[str] = []
    if not accepted:
        return {
            "status": "insufficient",
            "accepted_count": 0,
            "center_span_norm": {"x": 0.0, "y": 0.0},
            "area_fraction_range": {"min": None, "max": None, "ratio": None},
            "warnings": ["no accepted checkerboard coverage metadata"],
        }

    centers_x = [float(entry["board_coverage"]["center_norm"]["x"]) for entry in accepted]
    centers_y = [float(entry["board_coverage"]["center_norm"]["y"]) for entry in accepted]
    areas = [float(entry["board_coverage"]["area_fraction"]) for entry in accepted]
    span_x = max(centers_x) - min(centers_x)
    span_y = max(centers_y) - min(centers_y)
    min_area = min(areas)
    max_area = max(areas)
    area_ratio = None if min_area <= 1e-12 else max_area / min_area

    if len(accepted) < 10:
        warnings.append("limited accepted checkerboard image count")
    if span_x < 0.35 or span_y < 0.35:
        warnings.append("insufficient screen-position coverage")
    if area_ratio is None or area_ratio < 1.5:
        warnings.append("insufficient checkerboard scale variation")
    if min(centers_x) > 0.25 or max(centers_x) < 0.75 or min(centers_y) > 0.25 or max(centers_y) < 0.75:
        warnings.append("checkerboard centers do not cover near-edge regions")

    return {
        "status": "ok" if not warnings else "warning",
        "accepted_count": len(accepted),
        "center_span_norm": {"x": float(span_x), "y": float(span_y)},
        "center_range_norm": {
            "x_min": float(min(centers_x)),
            "x_max": float(max(centers_x)),
            "y_min": float(min(centers_y)),
            "y_max": float(max(centers_y)),
        },
        "area_fraction_range": {
            "min": float(min_area),
            "max": float(max_area),
            "ratio": None if area_ratio is None else float(area_ratio),
        },
        "warnings": warnings,
    }


def _matrix_to_float_lists(matrix: Any) -> list[list[float]]:
    return [[float(value) for value in row] for row in matrix.tolist()]


def _vector_to_float_list(vector: Any) -> list[float]:
    import numpy as np

    return [float(value) for value in np.ravel(vector).tolist()]


def _as_3x3_matrix(value: Any, *, field_name: str) -> list[list[float]]:
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{field_name} must be a 3x3 matrix")
    matrix: list[list[float]] = []
    for row in value:
        if not isinstance(row, list | tuple) or len(row) != 3:
            raise ValueError(f"{field_name} must be a 3x3 matrix")
        matrix.append([float(row[0]), float(row[1]), float(row[2])])
    if not all(math.isfinite(component) for row in matrix for component in row):
        raise ValueError(f"{field_name} contains non-finite values")
    return matrix


def _as_3_vector(value: Any, *, field_name: str) -> tuple[float, float, float]:
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{field_name} must have 3 values")
    vector = (float(value[0]), float(value[1]), float(value[2]))
    if not all(math.isfinite(component) for component in vector):
        raise ValueError(f"{field_name} contains non-finite values")
    return vector


def stereo_calibration_extrinsics_from_result(result: StereoIntrinsicsResult) -> StereoCalibrationExtrinsics:
    """Convert OpenCV stereoCalibrate pose to the app's right-camera-in-left-camera contract."""
    stereo = result.stereo_calibration
    if not isinstance(stereo, dict):
        raise ValueError("stereo_calibration missing")
    rotation_left_to_right = _as_3x3_matrix(
        stereo.get("rotation_left_to_right"),
        field_name="rotation_left_to_right",
    )
    translation_left_to_right = _as_3_vector(
        stereo.get("translation_left_to_right_m"),
        field_name="translation_left_to_right_m",
    )
    rotation_right_to_left = [
        [rotation_left_to_right[col][row] for col in range(3)]
        for row in range(3)
    ]
    translation_right_in_left = tuple(
        -sum(rotation_right_to_left[row][col] * translation_left_to_right[col] for col in range(3))
        for row in range(3)
    )
    baseline_m = math.sqrt(sum(component * component for component in translation_right_in_left))
    if baseline_m <= 0.0:
        raise ValueError("checkerboard stereo baseline must be > 0")
    return StereoCalibrationExtrinsics(
        rotation_right_to_left=rotation_right_to_left,
        translation_right_in_left_m=translation_right_in_left,  # type: ignore[arg-type]
        baseline_m=float(baseline_m),
        source="checkerboard_stereo_calibration",
        pair_count=int(stereo.get("pair_count") or 0),
        rms_error_px=None if stereo.get("rms_error_px") is None else float(stereo["rms_error_px"]),
        epipolar_mean_error_px=(
            None if stereo.get("epipolar_mean_error_px") is None else float(stereo["epipolar_mean_error_px"])
        ),
        epipolar_max_error_px=(
            None if stereo.get("epipolar_max_error_px") is None else float(stereo["epipolar_max_error_px"])
        ),
    )


def calibrate_camera_from_images(paths: list[str | Path], config: CheckerboardConfig) -> CameraIntrinsics:
    import cv2
    import numpy as np

    object_points = []
    image_points = []
    image_results: list[dict[str, Any]] = []
    accepted_result_indexes: list[int] = []
    objp = _object_points(config)
    image_size: tuple[int, int] | None = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for raw_path in paths:
        path = Path(raw_path)
        image = _load_grayscale(path)
        current_size = (int(image.shape[1]), int(image.shape[0]))
        if image_size is None:
            image_size = current_size
        elif image_size != current_size:
            image_results.append(
                _image_result(
                    path,
                    accepted=False,
                    image_size=current_size,
                    reject_reason=f"image size mismatch: {current_size} != {image_size}",
                )
            )
            continue
        found, corners, method = _find_checkerboard(cv2, image, config.pattern_size)
        if not found or corners is None:
            image_results.append(
                _image_result(
                    path,
                    accepted=False,
                    detector_method=method,
                    corner_count=0 if corners is None else int(len(corners)),
                    image_size=current_size,
                    reject_reason="checkerboard not found",
                )
            )
            continue
        if method == "findChessboardCornersSB":
            refined = corners
        else:
            refined = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), criteria)
        object_points.append(objp)
        image_points.append(refined)
        accepted_result_indexes.append(len(image_results))
        image_results.append(
            _image_result(
                path,
                accepted=True,
                detector_method=method,
                corner_count=int(len(corners)),
                image_size=current_size,
                board_coverage=_board_coverage_from_corners(refined, current_size),
            )
        )

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
    total_error = 0.0
    total_points = 0
    for result_index, obj, img, rvec, tvec in zip(accepted_result_indexes, object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        error = cv2.norm(img, projected, cv2.NORM_L2)
        per_image_error = math.sqrt((error * error) / len(obj)) if len(obj) else 0.0
        image_results[result_index]["reprojection_error_px"] = float(per_image_error)
        total_error += error * error
        total_points += len(obj)
    reprojection_error = math.sqrt(total_error / total_points) if total_points else float(ret)
    return CameraIntrinsics(
        width=image_size[0],
        height=image_size[1],
        camera_matrix=_matrix_to_float_lists(camera_matrix),
        distortion_coefficients=_vector_to_float_list(distortion),
        reprojection_error_px=float(reprojection_error),
        image_count=len(object_points),
        image_results=image_results,
        coverage_summary=_coverage_summary(image_results),
    )


def _accepted_pair_result(
    left_path: Path,
    right_path: Path,
    *,
    detector_method_left: str,
    detector_method_right: str,
    corner_count_left: int,
    corner_count_right: int,
    image_size: tuple[int, int],
) -> dict[str, Any]:
    return {
        "left_path": str(left_path),
        "right_path": str(right_path),
        "accepted": True,
        "reject_reason": None,
        "left_detector_method": detector_method_left,
        "right_detector_method": detector_method_right,
        "left_corner_count": int(corner_count_left),
        "right_corner_count": int(corner_count_right),
        "image_size": {"width": int(image_size[0]), "height": int(image_size[1])},
        "epipolar_mean_error_px": None,
        "epipolar_max_error_px": None,
    }


def _rejected_pair_result(
    left_path: Path | None,
    right_path: Path | None,
    *,
    reject_reason: str,
    image_size: tuple[int, int] | None = None,
    left_detector_method: str | None = None,
    right_detector_method: str | None = None,
    left_corner_count: int = 0,
    right_corner_count: int = 0,
) -> dict[str, Any]:
    return {
        "left_path": None if left_path is None else str(left_path),
        "right_path": None if right_path is None else str(right_path),
        "accepted": False,
        "reject_reason": reject_reason,
        "left_detector_method": left_detector_method,
        "right_detector_method": right_detector_method,
        "left_corner_count": int(left_corner_count),
        "right_corner_count": int(right_corner_count),
        "image_size": None if image_size is None else {"width": int(image_size[0]), "height": int(image_size[1])},
        "epipolar_mean_error_px": None,
        "epipolar_max_error_px": None,
    }


def _stereo_pair_key(path: str | Path, side: str) -> str:
    stem = Path(path).stem
    for prefix in (f"{side}_", f"{side}-", side):
        if stem.startswith(prefix):
            return stem[len(prefix) :]
    return stem


def _stereo_image_pairs(
    left_paths: Sequence[str | Path],
    right_paths: Sequence[str | Path],
) -> list[tuple[Path | None, Path | None, str]]:
    left_by_key = {_stereo_pair_key(path, "left"): Path(path) for path in left_paths}
    right_by_key = {_stereo_pair_key(path, "right"): Path(path) for path in right_paths}
    pairs: list[tuple[Path | None, Path | None, str]] = []
    for key in sorted(set(left_by_key) | set(right_by_key)):
        pairs.append((left_by_key.get(key), right_by_key.get(key), key))
    return pairs


def _epipolar_errors_px(left_points: Any, right_points: Any, fundamental_matrix: Any) -> list[float]:
    import numpy as np

    left = np.asarray(left_points, dtype=float).reshape(-1, 2)
    right = np.asarray(right_points, dtype=float).reshape(-1, 2)
    count = min(len(left), len(right))
    if count <= 0:
        return []
    left_h = np.column_stack([left[:count], np.ones(count)])
    right_h = np.column_stack([right[:count], np.ones(count)])
    lines_right = (np.asarray(fundamental_matrix, dtype=float) @ left_h.T).T
    denominator = np.sqrt(lines_right[:, 0] ** 2 + lines_right[:, 1] ** 2)
    valid = denominator > 1e-12
    if not bool(np.any(valid)):
        return []
    numerator = np.abs(np.sum(right_h * lines_right, axis=1))
    return [float(value) for value in (numerator[valid] / denominator[valid]).tolist()]


def calibrate_stereo_from_image_pairs(
    left_paths: list[str | Path],
    right_paths: list[str | Path],
    config: CheckerboardConfig,
    left_intrinsics: CameraIntrinsics,
    right_intrinsics: CameraIntrinsics,
) -> dict[str, Any]:
    import cv2
    import numpy as np

    object_points = []
    left_image_points = []
    right_image_points = []
    pair_results: list[dict[str, Any]] = []
    accepted_result_indexes: list[int] = []
    objp = _object_points(config)
    image_size: tuple[int, int] | None = None
    expected_size = (int(left_intrinsics.width), int(left_intrinsics.height))
    right_expected_size = (int(right_intrinsics.width), int(right_intrinsics.height))
    if right_expected_size != expected_size:
        raise ValueError(f"left/right intrinsics image size mismatch: {expected_size} != {right_expected_size}")
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for left_path, right_path, _pair_key in _stereo_image_pairs(left_paths, right_paths):
        if left_path is None or right_path is None:
            pair_results.append(
                _rejected_pair_result(left_path, right_path, reject_reason="missing left/right pair image")
            )
            continue
        left_image = _load_grayscale(left_path)
        right_image = _load_grayscale(right_path)
        left_size = (int(left_image.shape[1]), int(left_image.shape[0]))
        right_size = (int(right_image.shape[1]), int(right_image.shape[0]))
        if left_size != right_size:
            pair_results.append(
                _rejected_pair_result(
                    left_path,
                    right_path,
                    reject_reason=f"left/right image size mismatch: {left_size} != {right_size}",
                    image_size=left_size,
                )
            )
            continue
        if left_size != expected_size:
            pair_results.append(
                _rejected_pair_result(
                    left_path,
                    right_path,
                    reject_reason=f"image size mismatch with intrinsics: {left_size} != {expected_size}",
                    image_size=left_size,
                )
            )
            continue
        if image_size is None:
            image_size = left_size

        left_found, left_corners, left_method = _find_checkerboard(cv2, left_image, config.pattern_size)
        right_found, right_corners, right_method = _find_checkerboard(cv2, right_image, config.pattern_size)
        if not left_found or left_corners is None or not right_found or right_corners is None:
            pair_results.append(
                _rejected_pair_result(
                    left_path,
                    right_path,
                    reject_reason="checkerboard not found in both pair images",
                    image_size=left_size,
                    left_detector_method=left_method,
                    right_detector_method=right_method,
                    left_corner_count=0 if left_corners is None else int(len(left_corners)),
                    right_corner_count=0 if right_corners is None else int(len(right_corners)),
                )
            )
            continue

        if left_method != "findChessboardCornersSB":
            left_corners = cv2.cornerSubPix(left_image, left_corners, (11, 11), (-1, -1), criteria)
        if right_method != "findChessboardCornersSB":
            right_corners = cv2.cornerSubPix(right_image, right_corners, (11, 11), (-1, -1), criteria)
        accepted_result_indexes.append(len(pair_results))
        pair_results.append(
            _accepted_pair_result(
                left_path,
                right_path,
                detector_method_left=left_method,
                detector_method_right=right_method,
                corner_count_left=int(len(left_corners)),
                corner_count_right=int(len(right_corners)),
                image_size=left_size,
            )
        )
        object_points.append(objp)
        left_image_points.append(left_corners)
        right_image_points.append(right_corners)

    if len(object_points) < 3:
        raise ValueError(f"need at least 3 valid stereo checkerboard pairs, found {len(object_points)}")
    assert image_size is not None

    flags = int(getattr(cv2, "CALIB_FIX_INTRINSIC", 0))
    ret, _left_camera, _left_distortion, _right_camera, _right_distortion, rotation, translation, essential, fundamental = (
        cv2.stereoCalibrate(
            object_points,
            left_image_points,
            right_image_points,
            np.asarray(left_intrinsics.camera_matrix, dtype=np.float64),
            np.asarray(left_intrinsics.distortion_coefficients, dtype=np.float64),
            np.asarray(right_intrinsics.camera_matrix, dtype=np.float64),
            np.asarray(right_intrinsics.distortion_coefficients, dtype=np.float64),
            image_size,
            flags=flags,
        )
    )

    all_epipolar_errors: list[float] = []
    for result_index, left_points, right_points in zip(accepted_result_indexes, left_image_points, right_image_points):
        errors = _epipolar_errors_px(left_points, right_points, fundamental)
        if errors:
            pair_results[result_index]["epipolar_mean_error_px"] = float(sum(errors) / len(errors))
            pair_results[result_index]["epipolar_max_error_px"] = float(max(errors))
            all_epipolar_errors.extend(errors)

    return {
        "method": "cv2.stereoCalibrate",
        "flags": ["CALIB_FIX_INTRINSIC"],
        "pair_count": len(object_points),
        "image_size": {"width": int(image_size[0]), "height": int(image_size[1])},
        "rms_error_px": float(ret),
        "rotation_left_to_right": _matrix_to_float_lists(rotation),
        "translation_left_to_right_m": _vector_to_float_list(translation),
        "essential_matrix": _matrix_to_float_lists(essential),
        "fundamental_matrix": _matrix_to_float_lists(fundamental),
        "epipolar_mean_error_px": None
        if not all_epipolar_errors
        else float(sum(all_epipolar_errors) / len(all_epipolar_errors)),
        "epipolar_max_error_px": None if not all_epipolar_errors else float(max(all_epipolar_errors)),
        "pair_results": pair_results,
    }


def save_intrinsics_json(path: str | Path, result: StereoIntrinsicsResult, *, metadata: dict[str, Any] | None = None) -> None:
    payload = {
        "left": result.left.__dict__,
        "right": result.right.__dict__,
        "board": result.board.__dict__,
        "stereo_calibration": result.stereo_calibration,
        "metadata": metadata or {},
    }
    Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False))


def camera_intrinsics_from_dict(payload: dict[str, Any]) -> CameraIntrinsics:
    return CameraIntrinsics(
        width=int(payload["width"]),
        height=int(payload["height"]),
        camera_matrix=[[float(value) for value in row] for row in payload["camera_matrix"]],
        distortion_coefficients=[float(value) for value in payload.get("distortion_coefficients", [])],
        reprojection_error_px=float(payload.get("reprojection_error_px", 0.0)),
        image_count=int(payload.get("image_count", 0)),
        image_results=list(payload.get("image_results", [])),
        coverage_summary=dict(payload.get("coverage_summary", {})),
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
        stereo_calibration=payload.get("stereo_calibration"),
    )
