"""Workstation checkerboard capture, calibration, validation, and preview workers."""

from __future__ import annotations

import inspect
import hashlib
import json
import random
from collections.abc import Sequence
from pathlib import Path

from PyQt5 import QtCore, QtGui

from Workstation.workstation_core.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoIntrinsicsResult,
    analyze_calibration_images,
    calibrate_camera_from_images,
    calibrate_camera_with_diagnostics,
    calibrate_stereo_extrinsics_from_image_pairs,
    detect_checkerboard,
)
from Workstation.workstation_core.calibration_quality import (
    CameraCalibrationQuality,
    evaluate_camera_calibration_quality,
)


def _solver_diagnostics_mapping(solve) -> dict[str, object]:
    return {
        "intrinsic_standard_deviations": dict(solve.intrinsic_standard_deviations),
        "extrinsic_standard_deviations": [
            dict(row) for row in solve.extrinsic_standard_deviations
        ],
        "per_view_errors_px": list(solve.per_view_errors_px),
        "view_geometry": [dict(row) for row in getattr(solve, "view_geometry", ())],
        "effective_view_ids": list(solve.effective_view_ids),
        "detected_view_count": len(solve.observations),
        "effective_view_count": len(solve.effective_view_ids),
        "subset_estimates": [dict(row) for row in solve.subset_estimates],
        "subset_membership": [dict(row) for row in solve.subset_membership],
        "subset_failures": [dict(row) for row in solve.subset_failures],
        "subset_summary": dict(solve.subset_summary),
        "calibration_flags": int(solve.calibration_flags),
    }


def _combined_quality_snapshot(
    left: CameraCalibrationQuality,
    right: CameraCalibrationQuality,
) -> dict[str, object]:
    state = (
        "FAIL"
        if "FAIL" in {left.state, right.state}
        else "WARNING"
        if "WARNING" in {left.state, right.state}
        else "PASS"
    )
    reasons = [
        {"side": side, **reason.to_mapping()}
        for side, quality in (("left", left), ("right", right))
        for reason in quality.reasons
    ]
    reason_codes = [f"{row['side']}.{row['code']}" for row in reasons]
    width = max(left.image_size[0], right.image_size[0])
    height = max(left.image_size[1], right.image_size[1])
    principal_offsets_px = {
        "left": [
            abs(float(left.cx_px or 0.0) - (left.image_size[0] - 1) * 0.5),
            abs(float(left.cy_px or 0.0) - (left.image_size[1] - 1) * 0.5),
        ],
        "right": [
            abs(float(right.cx_px or 0.0) - (right.image_size[0] - 1) * 0.5),
            abs(float(right.cy_px or 0.0) - (right.image_size[1] - 1) * 0.5),
        ],
    }
    left_cells = set(left.center_grid_occupied)
    right_cells = set(right.center_grid_occupied)
    coverage_grid = []
    for row in range(3):
        grid_row = []
        for column in range(3):
            left_present = (column, row) in left_cells
            right_present = (column, row) in right_cells
            grid_row.append(
                {
                    "label": f"L:{int(left_present)} R:{int(right_present)}",
                    "covered": left_present and right_present,
                }
            )
        coverage_grid.append(grid_row)
    return {
        "schema": "workstation-camera-calibration-quality-v1",
        "state": state,
        "source": "calibrateCameraExtended + deterministic pose/subset policy",
        "reason_codes": reason_codes,
        "reasons": reasons,
        "effective_pose_count": {
            "left": left.effective_pose_count,
            "right": right.effective_pose_count,
        },
        "coverage": {
            "left_union_extent": list(left.union_extent_normalized),
            "right_union_extent": list(right.union_extent_normalized),
        },
        "coverage_grid_3x3": coverage_grid,
        "principal_point_offsets_px": principal_offsets_px,
        "subset_stability": {
            "left_principal_span_px": (
                None
                if left.subset_principal_span_px is None
                else list(left.subset_principal_span_px)
            ),
            "right_principal_span_px": (
                None
                if right.subset_principal_span_px is None
                else list(right.subset_principal_span_px)
            ),
            "left_focal_relative_span": left.subset_focal_relative_span,
            "right_focal_relative_span": right.subset_focal_relative_span,
        },
        "uncertainty": {
            "left_focal_stddev_relative": getattr(
                left, "intrinsic_focal_stddev_relative", None
            ),
            "right_focal_stddev_relative": getattr(
                right, "intrinsic_focal_stddev_relative", None
            ),
            "left_principal_stddev_px": (
                None
                if getattr(left, "intrinsic_principal_stddev_px", None) is None
                else list(left.intrinsic_principal_stddev_px)
            ),
            "right_principal_stddev_px": (
                None
                if getattr(right, "intrinsic_principal_stddev_px", None) is None
                else list(right.intrinsic_principal_stddev_px)
            ),
            "left_max_per_view_error_px": getattr(left, "per_view_error_max_px", None),
            "right_max_per_view_error_px": getattr(right, "per_view_error_max_px", None),
        },
        "image_size": [width, height],
        "left_quality": left.to_mapping(),
        "right_quality": right.to_mapping(),
    }


def _call_with_progress(function, *args, progress_callback, **kwargs):
    """Keep worker test doubles and older integrations compatible."""

    try:
        supports_progress = "progress_callback" in inspect.signature(function).parameters
    except (TypeError, ValueError):
        supports_progress = True
    if supports_progress:
        kwargs["progress_callback"] = progress_callback
    return function(*args, **kwargs)


class CheckerboardCaptureWorker(QtCore.QObject):
    """Encode and validate checkerboard captures away from the GUI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()
    progress = QtCore.pyqtSignal(object)

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
        side_results: dict[str, dict[str, object]] = {}
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
                    side_results[side] = {
                        "state": "no_frame",
                        "response": "no frame",
                        "path": "",
                    }
                    self.progress.emit(
                        {
                            "source": "calibration",
                            "phase": "side_result",
                            "side": side,
                            "state": "no_frame",
                            "response": "no frame",
                        }
                    )
                    continue
                if pair_candidate and not self.pair_timing_ok:
                    detail = f"not saved as stereo pair ({self.pair_timing_detail})"
                    status_parts.append(
                        f"{side}={detail}"
                    )
                    side_results[side] = {
                        "state": "pair_rejected",
                        "response": detail,
                        "path": "",
                    }
                    self.progress.emit(
                        {
                            "source": "calibration",
                            "phase": "side_result",
                            "side": side,
                            "state": "pair_rejected",
                            "response": detail,
                        }
                    )
                    continue
                path = directories[side] / f"{side}_{self.stem}.png"
                if not image.save(str(path), "PNG"):
                    status_parts.append(f"{side}=save FAIL")
                    side_results[side] = {
                        "state": "save_failed",
                        "response": "PNG save failed",
                        "path": str(path),
                    }
                    self.progress.emit(
                        {
                            "source": "calibration",
                            "phase": "side_result",
                            "side": side,
                            "state": "save_failed",
                            "response": "PNG save failed",
                            "path": str(path),
                        }
                    )
                    continue
                if self.config is None:
                    saved_sides.append(side)
                    detail = f": {self.opencv_error}" if self.opencv_error else ""
                    status_parts.append(f"{side}=saved (OpenCV unavailable{detail})")
                    side_results[side] = {
                        "state": "saved",
                        "response": f"saved; OpenCV unavailable{detail}",
                        "path": str(path),
                        "checkerboard_found": None,
                    }
                    self.progress.emit(
                        {
                            "source": "calibration",
                            "phase": "side_result",
                            "side": side,
                            "state": "saved",
                            "response": f"saved; OpenCV unavailable{detail}",
                            "path": str(path),
                        }
                    )
                    continue
                detection = detect_checkerboard(path, self.config)
                encoded_bytes = path.read_bytes()
                if detection.found:
                    saved_sides.append(side)
                    status_parts.append(f"{side}=OK saved ({detection.corner_count})")
                    side_results[side] = {
                        "state": "saved",
                        "response": f"checkerboard found; corners={detection.corner_count}",
                        "path": str(path),
                        "checkerboard_found": True,
                        "corner_count": detection.corner_count,
                        "width": image.width(),
                        "height": image.height(),
                        "size_bytes": len(encoded_bytes),
                        "sha256": hashlib.sha256(encoded_bytes).hexdigest(),
                    }
                else:
                    path.unlink(missing_ok=True)
                    detail = (
                        f"error: {detection.error}"
                        if detection.error
                        else f"{detection.corner_count}"
                    )
                    status_parts.append(f"{side}=not found ({detail})")
                    side_results[side] = {
                        "state": "checkerboard_not_found",
                        "response": detail,
                        "path": "",
                        "checkerboard_found": False,
                        "corner_count": detection.corner_count,
                    }
                self.progress.emit(
                    {
                        "source": "calibration",
                        "phase": "side_result",
                        "side": side,
                        **side_results[side],
                    }
                )
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
                    "side_results": side_results,
                }
            )
        finally:
            self.finished.emit()


class CheckerboardCalibrationWorker(QtCore.QObject):
    """Run OpenCV intrinsic/stereo calibration away from the GUI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()
    progress = QtCore.pyqtSignal(object)

    def __init__(
        self,
        *,
        mode: str,
        config: CheckerboardConfig,
        left_intrinsic_paths: Sequence[Path] | None = None,
        right_intrinsic_paths: Sequence[Path] | None = None,
        stereo_pairs: Sequence[tuple[str, Path, Path]] | None = None,
        left_paths: Sequence[Path] | None = None,
        right_paths: Sequence[Path] | None = None,
    ) -> None:
        super().__init__()
        self.mode = str(mode)
        resolved_left = left_intrinsic_paths if left_intrinsic_paths is not None else left_paths
        resolved_right = right_intrinsic_paths if right_intrinsic_paths is not None else right_paths
        if resolved_left is None or resolved_right is None:
            raise TypeError(
                "left/right intrinsic paths are required; pass left_intrinsic_paths and "
                "right_intrinsic_paths (or legacy left_paths/right_paths)"
            )
        self.left_intrinsic_paths = tuple(Path(path) for path in resolved_left)
        self.right_intrinsic_paths = tuple(Path(path) for path in resolved_right)
        # Compatibility aliases for integrations which inspect the worker.
        self.left_paths = self.left_intrinsic_paths
        self.right_paths = self.right_intrinsic_paths
        if stereo_pairs is None:
            # Legacy callers supplied two parallel lists. Preserve their exact
            # ordering while keeping the new stereo boundary explicit inside
            # the worker.
            self.stereo_pairs = tuple(
                (f"legacy-{index:06d}", left, right)
                for index, (left, right) in enumerate(
                    zip(self.left_intrinsic_paths, self.right_intrinsic_paths, strict=False)
                )
            )
        else:
            self.stereo_pairs = tuple(
                (str(key), Path(left), Path(right))
                for key, left, right in stereo_pairs
            )
        self.config = config

    def _emit_progress(
        self,
        phase: str,
        processed: int,
        total: int,
        message: str,
    ) -> None:
        total = max(0, int(total))
        processed = min(max(0, int(processed)), total) if total else 0
        percent = 100.0 if total == 0 else (100.0 * processed / total)
        self.progress.emit(
            {
                "phase": phase,
                "processed": processed,
                "total": total,
                "percent": percent,
                "message": message,
            }
        )

    def _callback(self, phase: str, label: str):
        return lambda processed, total, _operation: self._emit_progress(
            phase,
            processed,
            total,
            f"{label} {processed}/{total}",
        )

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            self._emit_progress(
                "left_intrinsic",
                0,
                len(self.left_intrinsic_paths),
                f"왼쪽 내부 파라미터 검출 0/{len(self.left_intrinsic_paths)}",
            )
            left_solve = _call_with_progress(
                calibrate_camera_with_diagnostics,
                self.left_intrinsic_paths,
                self.config,
                progress_callback=self._callback("left_intrinsic", "왼쪽 내부 파라미터 검출"),
            )
            left = left_solve.intrinsics
            left_quality = evaluate_camera_calibration_quality(**left_solve.quality_inputs())
            self._emit_progress(
                "left_intrinsic",
                len(self.left_intrinsic_paths),
                len(self.left_intrinsic_paths),
                f"왼쪽 내부 파라미터 완료 "
                f"{len(self.left_intrinsic_paths)}/{len(self.left_intrinsic_paths)}",
            )
            self._emit_progress(
                "right_intrinsic",
                0,
                len(self.right_intrinsic_paths),
                f"오른쪽 내부 파라미터 검출 0/{len(self.right_intrinsic_paths)}",
            )
            right_solve = _call_with_progress(
                calibrate_camera_with_diagnostics,
                self.right_intrinsic_paths,
                self.config,
                progress_callback=self._callback("right_intrinsic", "오른쪽 내부 파라미터 검출"),
            )
            right = right_solve.intrinsics
            right_quality = evaluate_camera_calibration_quality(**right_solve.quality_inputs())
            self._emit_progress(
                "right_intrinsic",
                len(self.right_intrinsic_paths),
                len(self.right_intrinsic_paths),
                f"오른쪽 내부 파라미터 완료 "
                f"{len(self.right_intrinsic_paths)}/{len(self.right_intrinsic_paths)}",
            )
            extrinsics = None
            effective_stereo_pairs: tuple[tuple[str, Path, Path], ...] = ()
            if self.mode == "checkerboard":
                effective_left = set(left_solve.effective_view_ids)
                effective_right = set(right_solve.effective_view_ids)
                effective_stereo_pairs = tuple(
                    (key, left_path, right_path)
                    for key, left_path, right_path in self.stereo_pairs
                    if str(left_path) in effective_left and str(right_path) in effective_right
                )
                if len(effective_stereo_pairs) < 3:
                    raise ValueError(
                        "need at least 3 stereo pairs remaining after pose-duplicate filtering, "
                        f"found {len(effective_stereo_pairs)}"
                    )
                stereo_left_paths = tuple(
                    left_path for _key, left_path, _right in effective_stereo_pairs
                )
                stereo_right_paths = tuple(
                    right_path for _key, _left, right_path in effective_stereo_pairs
                )
                stereo_pair_count = len(effective_stereo_pairs)
                self._emit_progress(
                    "stereo_extrinsic",
                    0,
                    stereo_pair_count,
                    "stereo pair 검출 0개",
                )
                extrinsics = _call_with_progress(
                    calibrate_stereo_extrinsics_from_image_pairs,
                    stereo_left_paths,
                    stereo_right_paths,
                    self.config,
                    left,
                    right,
                    progress_callback=self._callback("stereo_extrinsic", "stereo pair 검출"),
                )
                self._emit_progress(
                    "stereo_extrinsic",
                    stereo_pair_count,
                    stereo_pair_count,
                    "stereo pair 계산 완료",
                )
            self._emit_progress(
                "image_diagnostics",
                0,
                len(self.left_intrinsic_paths) + len(self.right_intrinsic_paths),
                "사진별 reprojection error 계산 중",
            )
            diagnostics = (
                *analyze_calibration_images(
                    list(self.left_intrinsic_paths),
                    self.config,
                    left,
                    side="left",
                ),
                *analyze_calibration_images(
                    list(self.right_intrinsic_paths),
                    self.config,
                    right,
                    side="right",
                ),
            )
            self._emit_progress(
                "image_diagnostics",
                len(self.left_intrinsic_paths) + len(self.right_intrinsic_paths),
                len(self.left_intrinsic_paths) + len(self.right_intrinsic_paths),
                "사진별 reprojection error 완료",
            )
            result = StereoIntrinsicsResult(
                left=left,
                right=right,
                board=self.config,
                extrinsics=extrinsics,
                image_diagnostics=tuple(diagnostics),
                left_quality=left_quality.to_mapping(),
                right_quality=right_quality.to_mapping(),
                calibration_quality=_combined_quality_snapshot(left_quality, right_quality),
                solver_diagnostics={
                    "left": _solver_diagnostics_mapping(left_solve),
                    "right": _solver_diagnostics_mapping(right_solve),
                    "effective_selection": {
                        "left_intrinsic_paths": list(left_solve.effective_view_ids),
                        "right_intrinsic_paths": list(right_solve.effective_view_ids),
                        "stereo_pairs": [
                            [str(key), str(left_path), str(right_path)]
                            for key, left_path, right_path in effective_stereo_pairs
                        ],
                    },
                },
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(result)
        finally:
            self.finished.emit()


class CheckerboardExtrinsicsWorker(QtCore.QObject):
    """Estimate only stereo R/T while preserving the selected K/D exactly."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()
    progress = QtCore.pyqtSignal(object)

    def __init__(
        self,
        *,
        left_paths: Sequence[Path],
        right_paths: Sequence[Path],
        config: CheckerboardConfig,
        left_intrinsics: CameraIntrinsics,
        right_intrinsics: CameraIntrinsics,
    ) -> None:
        super().__init__()
        self.left_paths = tuple(Path(path) for path in left_paths)
        self.right_paths = tuple(Path(path) for path in right_paths)
        self.config = config
        self.left_intrinsics = left_intrinsics
        self.right_intrinsics = right_intrinsics

    @QtCore.pyqtSlot()
    def run(self) -> None:
        total = min(len(self.left_paths), len(self.right_paths))

        def progress(processed: int, count: int, _operation: str) -> None:
            percent = 100.0 if count <= 0 else 100.0 * int(processed) / int(count)
            self.progress.emit(
                {
                    "phase": "stereo_extrinsic_only",
                    "processed": int(processed),
                    "total": int(count),
                    "percent": percent,
                    "message": f"R,t only stereo pair {processed}/{count}",
                }
            )

        try:
            self.progress.emit(
                {
                    "phase": "stereo_extrinsic_only",
                    "processed": 0,
                    "total": total,
                    "percent": 0.0,
                    "message": f"R,t only stereo pair 0/{total}",
                }
            )
            extrinsics = _call_with_progress(
                calibrate_stereo_extrinsics_from_image_pairs,
                self.left_paths,
                self.right_paths,
                self.config,
                self.left_intrinsics,
                self.right_intrinsics,
                progress_callback=progress,
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        else:
            self.completed.emit(extrinsics)
        finally:
            self.finished.emit()


class CheckerboardRandomValidationWorker(QtCore.QObject):
    """Run repeated checkerboard calibration subsets away from the GUI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()
    progress = QtCore.pyqtSignal(object)

    def __init__(
        self,
        *,
        mode: str,
        config: CheckerboardConfig,
        runs: int,
        subset_count: int,
        seed: int,
        left_intrinsic_paths: Sequence[Path] | None = None,
        right_intrinsic_paths: Sequence[Path] | None = None,
        stereo_pairs: Sequence[tuple[str, Path, Path]] | None = None,
        pairs: Sequence[tuple[str, Path, Path]] | None = None,
    ) -> None:
        super().__init__()
        self.mode = str(mode)
        resolved_pairs = stereo_pairs if stereo_pairs is not None else pairs
        self.stereo_pairs = tuple(
            (str(key), Path(left), Path(right))
            for key, left, right in (resolved_pairs or ())
        )
        # Compatibility alias retained for code/tests that inspect ``pairs``.
        self.pairs = self.stereo_pairs
        self._independent_intrinsic_pools = (
            left_intrinsic_paths is not None or right_intrinsic_paths is not None
        )
        self.left_intrinsic_paths = tuple(
            Path(path)
            for path in (
                left_intrinsic_paths
                if left_intrinsic_paths is not None
                else (left for _key, left, _right in self.stereo_pairs)
            )
        )
        self.right_intrinsic_paths = tuple(
            Path(path)
            for path in (
                right_intrinsic_paths
                if right_intrinsic_paths is not None
                else (right for _key, _left, right in self.stereo_pairs)
            )
        )
        self.config = config
        self.runs = max(1, int(runs))
        self.subset_count = max(1, int(subset_count))
        self.seed = int(seed)

    def _emit_progress(
        self,
        run: int,
        phase: str,
        processed: int,
        total: int,
        message: str,
    ) -> None:
        total = max(0, int(total))
        processed = min(max(0, int(processed)), total) if total else 0
        percent = 100.0 if total == 0 else (100.0 * processed / total)
        self.progress.emit(
            {
                "run": run,
                "runs": self.runs,
                "phase": phase,
                "processed": processed,
                "total": total,
                "percent": percent,
                "message": message,
            }
        )

    def _callback(self, run: int, phase: str, label: str):
        return lambda processed, total, _operation: self._emit_progress(
            run,
            phase,
            processed,
            total,
            f"run {run}/{self.runs} · {label} {processed}/{total}",
        )

    @QtCore.pyqtSlot()
    def run(self) -> None:
        rng = random.Random(self.seed if self.seed > 0 else None)
        rows: list[dict[str, object]] = []
        try:
            for index in range(1, self.runs + 1):
                if self._independent_intrinsic_pools:
                    stereo_subset = (
                        rng.sample(self.stereo_pairs, self.subset_count)
                        if self.mode == "checkerboard"
                        else ()
                    )
                    left_count = min(self.subset_count, len(self.left_intrinsic_paths))
                    right_count = min(self.subset_count, len(self.right_intrinsic_paths))
                    left_paths = tuple(rng.sample(self.left_intrinsic_paths, left_count))
                    right_paths = tuple(rng.sample(self.right_intrinsic_paths, right_count))
                else:
                    legacy_subset = rng.sample(self.stereo_pairs, self.subset_count)
                    left_paths = tuple(left for _key, left, _right in legacy_subset)
                    right_paths = tuple(right for _key, _left, right in legacy_subset)
                    stereo_subset = legacy_subset if self.mode == "checkerboard" else ()
                try:
                    self._emit_progress(
                        index,
                        "validation_left",
                        0,
                        len(left_paths),
                        f"run {index}/{self.runs} · 왼쪽 검증 0/{len(left_paths)}",
                    )
                    left = _call_with_progress(
                        calibrate_camera_from_images,
                        left_paths,
                        self.config,
                        progress_callback=self._callback(index, "validation_left", "왼쪽 검증"),
                    )
                    self._emit_progress(
                        index,
                        "validation_right",
                        0,
                        len(right_paths),
                        f"run {index}/{self.runs} · 오른쪽 검증 0/{len(right_paths)}",
                    )
                    right = _call_with_progress(
                        calibrate_camera_from_images,
                        right_paths,
                        self.config,
                        progress_callback=self._callback(index, "validation_right", "오른쪽 검증"),
                    )
                    extrinsics = None
                    if self.mode == "checkerboard":
                        stereo_left_paths = tuple(
                            left_path for _key, left_path, _right in stereo_subset
                        )
                        stereo_right_paths = tuple(
                            right_path for _key, _left, right_path in stereo_subset
                        )
                        self._emit_progress(
                            index,
                            "validation_stereo",
                            0,
                            len(stereo_subset),
                            f"run {index}/{self.runs} · stereo 검증 0/{len(stereo_subset)}",
                        )
                        extrinsics = _call_with_progress(
                            calibrate_stereo_extrinsics_from_image_pairs,
                            stereo_left_paths,
                            stereo_right_paths,
                            self.config,
                            left,
                            right,
                            progress_callback=self._callback(index, "validation_stereo", "stereo 검증"),
                        )
                    result = StereoIntrinsicsResult(
                        left=left,
                        right=right,
                        board=self.config,
                        extrinsics=extrinsics,
                    )
                except Exception as exc:
                    rows.append(
                        {
                            "index": index,
                            "error": str(exc),
                            "left_paths": left_paths,
                            "right_paths": right_paths,
                            "stereo_pair_keys": tuple(key for key, _left, _right in stereo_subset),
                        }
                    )
                else:
                    rows.append(
                        {
                            "index": index,
                            "result": result,
                            "left_paths": left_paths,
                            "right_paths": right_paths,
                            "stereo_pair_keys": tuple(key for key, _left, _right in stereo_subset),
                        }
                    )
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
            "metadata": dict(payload.get("metadata") or {})
            if isinstance(payload.get("metadata"), dict)
            else {},
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
