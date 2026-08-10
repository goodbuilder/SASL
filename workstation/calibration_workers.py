"""Workstation checkerboard capture, calibration, validation, and preview workers."""

from __future__ import annotations

import json
import random
from collections.abc import Sequence
from pathlib import Path

from PyQt5 import QtCore, QtGui

from workstation_core.calibration import (
    CheckerboardConfig,
    StereoIntrinsicsResult,
    calibrate_camera_from_images,
    calibrate_stereo_extrinsics_from_image_pairs,
    detect_checkerboard,
)


class CheckerboardCaptureWorker(QtCore.QObject):
    """Encode and validate checkerboard captures away from the GUI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        root: Path,
        frames: dict[str, QtGui.QImage | None],
        stem: str,
        config: CheckerboardConfig | None,
        opencv_error: str | None,
        pair_timing_ok: bool,
        pair_time_basis: str,
        pair_delta_s: float | None,
        pair_timing_detail: str,
        metadata_payload: dict[str, object],
    ) -> None:
        super().__init__()
        self.root = Path(root)
        self.frames = dict(frames)
        self.stem = str(stem)
        self.config = config
        self.opencv_error = str(opencv_error or "")
        self.pair_timing_ok = bool(pair_timing_ok)
        self.pair_time_basis = str(pair_time_basis)
        self.pair_delta_s = pair_delta_s
        self.pair_timing_detail = str(pair_timing_detail)
        self.metadata_payload = dict(metadata_payload)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        saved_sides: list[str] = []
        status_parts: list[str] = []
        pair_candidate = self.frames.get("left") is not None and self.frames.get("right") is not None
        try:
            directories = {
                "left": self.root / "left",
                "right": self.root / "right",
            }
            for directory in directories.values():
                directory.mkdir(parents=True, exist_ok=True)
            if pair_candidate and not self.pair_timing_ok:
                status_parts.append(f"pair=rejected ({self.pair_timing_detail})")
            for side in ("left", "right"):
                image = self.frames.get(side)
                if image is None:
                    status_parts.append(f"{side}=no frame")
                    continue
                if pair_candidate and not self.pair_timing_ok:
                    status_parts.append(
                        f"{side}=not saved as stereo pair ({self.pair_timing_detail})"
                    )
                    continue
                path = directories[side] / f"{side}_{self.stem}.png"
                if not image.save(str(path), "PNG"):
                    status_parts.append(f"{side}=save FAIL")
                    continue
                if self.config is None:
                    saved_sides.append(side)
                    detail = f": {self.opencv_error}" if self.opencv_error else ""
                    status_parts.append(f"{side}=saved (OpenCV unavailable{detail})")
                    continue
                detection = detect_checkerboard(path, self.config)
                if detection.found:
                    saved_sides.append(side)
                    status_parts.append(f"{side}=OK saved ({detection.corner_count})")
                else:
                    path.unlink(missing_ok=True)
                    detail = (
                        f"error: {detection.error}"
                        if detection.error
                        else f"{detection.corner_count}"
                    )
                    status_parts.append(f"{side}=not found ({detail})")
            if saved_sides:
                metadata_dir = self.root / "capture_metadata"
                metadata_dir.mkdir(parents=True, exist_ok=True)
                payload = dict(self.metadata_payload)
                payload.update(
                    {
                        "capture_id": self.stem,
                        "saved_sides": saved_sides,
                        "stereo_pair_saved": set(saved_sides) == {"left", "right"},
                    }
                )
                (metadata_dir / f"capture_{self.stem}.json").write_text(
                    json.dumps(payload, indent=2, ensure_ascii=False),
                    encoding="utf-8",
                )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(
                {
                    "stem": self.stem,
                    "saved_sides": tuple(saved_sides),
                    "status_parts": tuple(status_parts),
                    "pair_timing_ok": self.pair_timing_ok,
                    "pair_time_basis": self.pair_time_basis,
                    "pair_delta_s": self.pair_delta_s,
                }
            )
        finally:
            self.finished.emit()


class CheckerboardCalibrationWorker(QtCore.QObject):
    """Run OpenCV intrinsic/stereo calibration away from the GUI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        mode: str,
        left_paths: Sequence[Path],
        right_paths: Sequence[Path],
        config: CheckerboardConfig,
    ) -> None:
        super().__init__()
        self.mode = str(mode)
        self.left_paths = tuple(Path(path) for path in left_paths)
        self.right_paths = tuple(Path(path) for path in right_paths)
        self.config = config

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            left = calibrate_camera_from_images(self.left_paths, self.config)
            right = calibrate_camera_from_images(self.right_paths, self.config)
            extrinsics = None
            if self.mode == "checkerboard":
                extrinsics = calibrate_stereo_extrinsics_from_image_pairs(
                    self.left_paths,
                    self.right_paths,
                    self.config,
                    left,
                    right,
                )
            result = StereoIntrinsicsResult(
                left=left,
                right=right,
                board=self.config,
                extrinsics=extrinsics,
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(result)
        finally:
            self.finished.emit()


class CheckerboardRandomValidationWorker(QtCore.QObject):
    """Run repeated checkerboard calibration subsets away from the GUI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        mode: str,
        pairs: Sequence[tuple[str, Path, Path]],
        config: CheckerboardConfig,
        runs: int,
        subset_count: int,
        seed: int,
    ) -> None:
        super().__init__()
        self.mode = str(mode)
        self.pairs = tuple((str(key), Path(left), Path(right)) for key, left, right in pairs)
        self.config = config
        self.runs = max(1, int(runs))
        self.subset_count = max(1, int(subset_count))
        self.seed = int(seed)

    @QtCore.pyqtSlot()
    def run(self) -> None:
        rng = random.Random(self.seed if self.seed > 0 else None)
        rows: list[dict[str, object]] = []
        try:
            for index in range(1, self.runs + 1):
                subset = rng.sample(self.pairs, self.subset_count)
                left_paths = tuple(left for _key, left, _right in subset)
                right_paths = tuple(right for _key, _left, right in subset)
                try:
                    left = calibrate_camera_from_images(left_paths, self.config)
                    right = calibrate_camera_from_images(right_paths, self.config)
                    extrinsics = None
                    if self.mode == "checkerboard":
                        extrinsics = calibrate_stereo_extrinsics_from_image_pairs(
                            left_paths,
                            right_paths,
                            self.config,
                            left,
                            right,
                        )
                    result = StereoIntrinsicsResult(
                        left=left,
                        right=right,
                        board=self.config,
                        extrinsics=extrinsics,
                    )
                except Exception as exc:
                    rows.append({"index": index, "error": str(exc)})
                else:
                    rows.append({"index": index, "result": result})
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(tuple(rows))
        finally:
            self.finished.emit()


class CheckerboardPreviewWorker(QtCore.QObject):
    result_ready = QtCore.pyqtSignal(object)

    def __init__(self) -> None:
        super().__init__()
        self._pending_payload: dict[str, object] | None = None
        self._busy = False
        self._process_scheduled = False
        self._cv_modules: tuple[object, object] | None = None
        self._cv_error: str | None = None

    @QtCore.pyqtSlot(object)
    def submit(self, payload: object) -> None:
        self._pending_payload = dict(payload or {}) if isinstance(payload, dict) else {}
        if not self._busy and not self._process_scheduled:
            self._process_scheduled = True
            QtCore.QTimer.singleShot(0, self._process_next)

    @QtCore.pyqtSlot()
    def _process_next(self) -> None:
        self._process_scheduled = False
        if self._busy or self._pending_payload is None:
            return
        payload = self._pending_payload
        self._pending_payload = None
        self._busy = True
        try:
            result = self._detect(payload)
        finally:
            self._busy = False
        self.result_ready.emit(result)
        if self._pending_payload is not None and not self._process_scheduled:
            self._process_scheduled = True
            QtCore.QTimer.singleShot(0, self._process_next)

    def _detect(self, payload: dict[str, object]) -> dict[str, object]:
        image = payload.get("image")
        result = {
            "camera": str(payload.get("camera") or ""),
            "request_id": int(payload.get("request_id") or 0),
            "image": image,
            "found": False,
            "corners": [],
            "status": "checkerboard: not found",
        }
        pattern_size = payload.get("pattern_size")
        if not isinstance(image, QtGui.QImage):
            result["status"] = "checkerboard: missing frame"
            return result
        if not (
            isinstance(pattern_size, tuple)
            and len(pattern_size) == 2
            and all(isinstance(value, int) and value > 0 for value in pattern_size)
        ):
            result["status"] = "checkerboard: invalid pattern"
            return result
        cv2, np, error = self._load_cv_modules()
        if error is not None:
            result["status"] = f"OpenCV unavailable: {error}"
            return result

        rgb_image = image.convertToFormat(QtGui.QImage.Format_RGB888).copy()
        width = rgb_image.width()
        height = rgb_image.height()
        ptr = rgb_image.bits()
        ptr.setsize(rgb_image.byteCount())
        rgb = np.frombuffer(ptr, dtype=np.uint8).reshape((height, rgb_image.bytesPerLine()))[:, : width * 3]
        rgb = rgb.reshape((height, width, 3))
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        flags = int(getattr(cv2, "CALIB_CB_FAST_CHECK", 0))
        found, raw_corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if not found or raw_corners is None:
            return result
        corners = [(float(point[0][0]), float(point[0][1])) for point in raw_corners]
        result["found"] = True
        result["corners"] = corners
        result["status"] = f"checkerboard: {len(corners)} corners"
        return result

    def _load_cv_modules(self):
        if self._cv_error is not None:
            return None, None, self._cv_error
        if self._cv_modules is not None:
            cv2, np = self._cv_modules
            return cv2, np, None
        try:
            import cv2
            import numpy as np
        except Exception as exc:
            self._cv_error = str(exc)
            return None, None, self._cv_error
        self._cv_modules = (cv2, np)
        return cv2, np, None
