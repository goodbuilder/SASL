from __future__ import annotations

import json
import math
from dataclasses import dataclass
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
    frame_contract: str = "opencv_optical:RDF(right,down,forward); X_right = R_left_to_right * X_left + T_left_to_right_m"


@dataclass(frozen=True)
class StereoIntrinsicsResult:
    left: CameraIntrinsics
    right: CameraIntrinsics
    board: CheckerboardConfig
    extrinsics: StereoExtrinsics | None = None


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


def calibrate_camera_from_images(paths: list[str | Path], config: CheckerboardConfig) -> CameraIntrinsics:
    import cv2
    import numpy as np

    object_points = []
    image_points = []
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
            raise ValueError(f"image size mismatch: {path} has {current_size}, expected {image_size}")
        found, corners = cv2.findChessboardCorners(image, config.pattern_size, None)
        if not found or corners is None:
            continue
        refined = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), criteria)
        object_points.append(objp)
        image_points.append(refined)

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

    for key in pair_keys:
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
        if not left_found or not right_found or left_corners is None or right_corners is None:
            continue
        left_refined = cv2.cornerSubPix(left_image, left_corners, (11, 11), (-1, -1), criteria)
        right_refined = cv2.cornerSubPix(right_image, right_corners, (11, 11), (-1, -1), criteria)
        object_points.append(objp)
        left_image_points.append(left_refined)
        right_image_points.append(right_refined)

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
    return StereoExtrinsics(
        rotation_left_to_right=[
            [float(value) for value in row] for row in payload["rotation_left_to_right"]
        ],
        translation_left_to_right_m=[float(value) for value in payload["translation_left_to_right_m"]],
        reprojection_error_px=float(payload.get("reprojection_error_px", 0.0)),
        pair_count=int(payload.get("pair_count", 0)),
        frame_contract=str(
            payload.get(
                "frame_contract",
                "opencv_optical:RDF(right,down,forward); X_right = R_left_to_right * X_left + T_left_to_right_m",
            )
        ),
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
