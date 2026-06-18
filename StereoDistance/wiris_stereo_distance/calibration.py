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
class StereoIntrinsicsResult:
    left: CameraIntrinsics
    right: CameraIntrinsics
    board: CheckerboardConfig


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


def save_intrinsics_json(path: str | Path, result: StereoIntrinsicsResult, *, metadata: dict[str, Any] | None = None) -> None:
    payload = {
        "left": result.left.__dict__,
        "right": result.right.__dict__,
        "board": result.board.__dict__,
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
    )
