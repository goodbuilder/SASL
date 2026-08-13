"""Workstation checkerboard calibration workflow controller.

Image observations and calibration matrices retain OpenCV optical RDF
(right, down, forward) conventions.  Stereo ``T`` follows
``X_right = R_left_to_right * X_left + T_left_to_right_m``; the right-camera
origin expressed in left RDF is the distinct value ``-R^T T``.  This module
performs no NED/ENU aliasing.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.calibration_workers import (
    CheckerboardCalibrationWorker,
    CheckerboardCaptureWorker,
    CheckerboardExtrinsicsWorker,
    CheckerboardRandomValidationWorker,
)
from Workstation.workstation_core.calibration import (
    CheckerboardConfig,
    StereoExtrinsics,
    StereoExtrinsicsProfile,
    StereoIntrinsicsResult,
    intrinsics_content_sha256,
    load_intrinsics_json,
    load_stereo_extrinsics_json,
    opencv_available,
    save_intrinsics_json,
    save_stereo_extrinsics_json,
)
from Workstation.workstation_core.settings import (
    DEFAULT_INTRINSICS_PATH,
    resolve_settings_path,
)

CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S = 0.100
CALIBRATION_AUTO_DETECTION_MAX_AGE_S = 2.0

CALIBRATION_WORKFLOW_METHODS = [
    "_app_relative_path_text",
    "_checkerboard_validation_mode",
    "_checkerboard_validation_root",
    "_calibration_image_paths_for_root",
    "_calibration_stereo_pair_paths_for_root",
    "_checkerboard_validation_scan_counts",
    "scan_checkerboard_validation_files",
    "choose_checkerboard_validation_root",
    "choose_checkerboard_validation_json",
    "_checkerboard_validation_set_result",
    "_checkerboard_result_baseline_m",
    "_rotation_delta_deg_from_matrices",
    "_format_checkerboard_validation_result_summary",
    "load_checkerboard_validation_json",
    "_checkerboard_random_subset_count",
    "_calibration_calculation_pair_limit",
    "_select_calibration_pairs",
    "_checkerboard_intrinsic_delta_px",
    "_checkerboard_validation_stddev",
    "run_checkerboard_random_validation",
    "_handle_random_validation_failed",
    "_handle_random_validation_completed",
    "_checkerboard_validation_result_row",
    "_checkerboard_validation_result_data",
    "save_checkerboard_validation_result",
    "apply_checkerboard_validation_result",
    "_checkerboard_config",
    "_calibration_dirs",
    "_intrinsics_storage_base_path",
    "_intrinsics_storage_path",
    "_extrinsics_storage_path",
    "_calibration_image_paths",
    "_next_calibration_capture_stem",
    "_checkerboard_capture_pair_timing",
    "capture_calibration_images",
    "toggle_auto_calibration_capture",
    "start_auto_calibration_capture",
    "stop_auto_calibration_capture",
    "_handle_checkerboard_capture_completed",
    "_handle_checkerboard_capture_failed",
    "run_intrinsic_calibration",
    "run_stereo_extrinsics_calibration",
    "_handle_stereo_extrinsics_completed",
    "save_stereo_extrinsics_profile",
    "load_stereo_extrinsics_profile",
    "apply_calibration_bundle",
    "_handle_calibration_progress",
    "_handle_intrinsic_calibration_completed",
    "_handle_intrinsic_calibration_failed",
    "_clear_checkerboard_calibration_worker"
]


class CalibrationWorkflowController:
    def __init__(self, panel):
        self.panel = panel
        timer_parent = panel if isinstance(panel, QtCore.QObject) else None
        self._auto_capture_timer = QtCore.QTimer(timer_parent)
        self._auto_capture_timer.timeout.connect(self._auto_calibration_capture_tick)
        self._auto_capture_active = False
        self._auto_capture_saved_pairs = 0
        self._auto_capture_last_pair_key: tuple[int, int] | None = None
        self._auto_capture_inflight_pair_key: tuple[int, int] | None = None
        self._auto_capture_last_attempt_s = 0.0
        self._capture_origin = "manual"

    def _app_relative_path_text(self, path: str | Path) -> str:
        resolved = Path(path).expanduser().resolve()
        try:
            return str(resolved.relative_to(self.panel.app_root))
        except ValueError:
            return str(resolved)

    def _checkerboard_validation_mode(self) -> str:
        if not hasattr(self.panel, "checkerboard_validation_mode_combo"):
            return "pixhawk"
        mode = str(self.panel.checkerboard_validation_mode_combo.currentData() or "pixhawk")
        return mode if mode in {"pixhawk", "checkerboard"} else "pixhawk"

    def _checkerboard_validation_root(self) -> Path:
        text = self.panel.checkerboard_validation_root_edit.text().strip() if hasattr(self.panel, "checkerboard_validation_root_edit") else ""
        if text:
            return resolve_settings_path(text, base_dir=self.panel.app_root)
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        return root

    def _calibration_image_paths_for_root(self, root: Path, side: str) -> list[Path]:
        return sorted((root / side).glob(f"{side}_*.png"))

    def _calibration_stereo_pair_paths_for_root(self, root: Path) -> list[tuple[str, Path, Path]]:
        def indexed(paths: list[Path], side: str) -> dict[str, Path]:
            result: dict[str, Path] = {}
            for path in paths:
                suffix = path.stem.replace(f"{side}_", "", 1)
                result[suffix] = path
            return result

        left_by_id = indexed(self.panel._calibration_image_paths_for_root(root, "left"), "left")
        right_by_id = indexed(self.panel._calibration_image_paths_for_root(root, "right"), "right")
        return [(key, left_by_id[key], right_by_id[key]) for key in sorted(set(left_by_id) & set(right_by_id))]

    def _checkerboard_validation_scan_counts(self) -> tuple[Path, list[Path], list[Path], list[tuple[str, Path, Path]]]:
        root = self.panel._checkerboard_validation_root()
        left_paths = self.panel._calibration_image_paths_for_root(root, "left")
        right_paths = self.panel._calibration_image_paths_for_root(root, "right")
        pairs = self.panel._calibration_stereo_pair_paths_for_root(root)
        return root, left_paths, right_paths, pairs

    def scan_checkerboard_validation_files(self) -> None:
        root, left_paths, right_paths, pairs = self.panel._checkerboard_validation_scan_counts()
        mode = self.panel._checkerboard_validation_mode()
        requirement = (
            "필수: left/right 이미지"
            if mode == "pixhawk"
            else "필수: matching stereo pair"
        )
        level = "good" if (pairs if mode == "checkerboard" else left_paths and right_paths) else "warn"
        text = (
            f"root={root} | left={len(left_paths)} right={len(right_paths)} "
            f"stereo_pair={len(pairs)} | {requirement}"
        )
        self.panel._set_health_label(self.panel.checkerboard_validation_scan_label, text, level)

    def choose_checkerboard_validation_root(self) -> None:
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self.panel,
            "체커보드 사진셋 root 선택",
            str(self.panel._checkerboard_validation_root()),
        )
        if not directory:
            return
        self.panel.checkerboard_validation_root_edit.setText(directory)
        self.panel.scan_checkerboard_validation_files()

    def choose_checkerboard_validation_json(self) -> None:
        path, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self.panel,
            "기존 intrinsics JSON 선택",
            str(self.panel._intrinsics_storage_path()),
            "Intrinsics JSON (*.json);;All Files (*)",
        )
        if path:
            self.panel.checkerboard_validation_json_edit.setText(path)

    def _checkerboard_validation_set_result(
        self,
        result: StereoIntrinsicsResult,
        *,
        mode: str,
        summary: dict[str, object],
        text: str,
    ) -> None:
        self.panel.checkerboard_validation_result = result
        self.panel.checkerboard_validation_result_mode = mode
        self.panel.checkerboard_validation_summary = dict(summary)
        extrinsics = getattr(result, "extrinsics", None)
        pair_count = int(
            summary.get("stereo_pair_count")
            or getattr(extrinsics, "pair_count", 0)
            or 0
        )
        self.panel.checkerboard_validation_run_results = [
            self.panel._checkerboard_validation_result_data(
                "Full",
                result,
                result,
                pair_count,
            )
        ]
        self.panel.checkerboard_validation_result_saved = False
        self.panel.last_calibration_result = result
        self.panel.last_calibration_zoom_level = self.panel.zoom_level_spin.value()
        self.panel._active_intrinsics_profile_path = (
            f"memory:checkerboard-validation:{mode}:zoom-{self.panel.last_calibration_zoom_level:02d}"
        )
        self.panel.calibration_result_text.setPlainText(text)
        extrinsics_state = "with checkerboard stereo R/T" if result.extrinsics is not None else "intrinsics only"
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"계산 결과 있음: {mode} | {extrinsics_state} | 저장 전",
            "good",
        )
        self.panel._refresh_camera_parameters_tab()
        self.panel._refresh_zoom_intrinsics_tab()

    def _checkerboard_result_baseline_m(self, result: StereoIntrinsicsResult) -> float | None:
        extrinsics = result.extrinsics
        if extrinsics is None:
            return None
        return math.sqrt(sum(value * value for value in extrinsics.translation_left_to_right_m))

    def _rotation_delta_deg_from_matrices(
        self,
        reference: list[list[float]] | None,
        current: list[list[float]] | None,
    ) -> float | None:
        if reference is None or current is None:
            return None
        try:
            trace = sum(
                float(current[row][col]) * float(reference[row][col])
                for row in range(3)
                for col in range(3)
            )
        except (TypeError, ValueError, IndexError):
            return None
        ratio = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
        return math.degrees(math.acos(ratio))

    def _format_checkerboard_validation_result_summary(
        self,
        result: StereoIntrinsicsResult,
        *,
        mode: str,
        title: str,
        root: Path | None = None,
        pair_count: int | None = None,
    ) -> str:
        baseline_m = self.panel._checkerboard_result_baseline_m(result)
        extrinsics = result.extrinsics
        lines = [
            title,
            f"mode: {mode}",
            f"root: {root}" if root is not None else "",
            f"Left: images={result.left.image_count}, error={result.left.reprojection_error_px:.3f}px, "
            f"fx={result.left.camera_matrix[0][0]:.3f}, fy={result.left.camera_matrix[1][1]:.3f}, "
            f"cx={result.left.camera_matrix[0][2]:.3f}, cy={result.left.camera_matrix[1][2]:.3f}",
            f"Right: images={result.right.image_count}, error={result.right.reprojection_error_px:.3f}px, "
            f"fx={result.right.camera_matrix[0][0]:.3f}, fy={result.right.camera_matrix[1][1]:.3f}, "
            f"cx={result.right.camera_matrix[0][2]:.3f}, cy={result.right.camera_matrix[1][2]:.3f}",
        ]
        if extrinsics is None:
            lines.append("Extrinsics: not computed (Pixhawk mode uses live attitude + GPS/RTK baseline)")
        else:
            lines.extend(
                [
                    f"Stereo: pairs={extrinsics.pair_count}, error={extrinsics.reprojection_error_px:.3f}px, "
                    f"|T|={baseline_m:.6f} m",
                    f"T_left_to_right_m={extrinsics.translation_left_to_right_m}",
                    f"R_left_to_right={extrinsics.rotation_left_to_right}",
                ]
            )
        if pair_count is not None:
            lines.append(f"selected stereo pairs: {pair_count}")
        return "\n".join(line for line in lines if line)

    def load_checkerboard_validation_json(self) -> None:
        path_text = self.panel.checkerboard_validation_json_edit.text().strip()
        if not path_text:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, "JSON 경로가 비어 있습니다", "bad")
            return
        path = resolve_settings_path(path_text, base_dir=self.panel.app_root)
        try:
            result = load_intrinsics_json(path)
        except Exception as exc:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, f"JSON 불러오기 실패: {exc}", "bad")
            return
        mode = "checkerboard" if result.extrinsics is not None else "pixhawk"
        if hasattr(self.panel, "checkerboard_validation_mode_combo"):
            self.panel._set_combo_data(self.panel.checkerboard_validation_mode_combo, mode)
        text = self.panel._format_checkerboard_validation_result_summary(
            result,
            mode=mode,
            title=f"Loaded intrinsics JSON: {path}",
        )
        self.panel._checkerboard_validation_set_result(
            result,
            mode=mode,
            summary={"source": "json", "path": str(path)},
            text=text,
        )
        self.panel.checkerboard_validation_result_saved = True
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"기존 JSON 불러옴: {mode} | {path}",
            "good",
        )

    def _checkerboard_random_subset_count(self, total_pairs: int) -> int:
        configured = self.panel.checkerboard_validation_random_pair_count_spin.value()
        if configured > 0:
            return min(total_pairs, max(3, configured))
        min_images = self.panel.calib_min_images_spin.value()
        return min(total_pairs, max(3, min_images, int(math.ceil(total_pairs * 0.7))))

    def _calibration_calculation_pair_limit(self) -> int:
        widget = getattr(self.panel, "calib_calculation_pairs_spin", None)
        try:
            return max(0, int(widget.value()))
        except (AttributeError, TypeError, ValueError):
            return 0

    def _select_calibration_pairs(
        self,
        pairs: list[tuple[str, Path, Path]],
    ) -> list[tuple[str, Path, Path]]:
        """Select evenly distributed capture pairs for calibration input.

        A deterministic spread keeps early/late board poses represented instead
        of silently taking the first N captures, while zero means all pairs.
        """

        candidates = list(pairs)
        limit = self._calibration_calculation_pair_limit()
        if limit <= 0 or limit >= len(candidates):
            return candidates
        limit = max(3, limit)
        if limit >= len(candidates):
            return candidates
        indices = sorted(
            {
                round(index * (len(candidates) - 1) / (limit - 1))
                for index in range(limit)
            }
        )
        return [candidates[index] for index in indices]

    def _checkerboard_intrinsic_delta_px(self, result: StereoIntrinsicsResult, reference: StereoIntrinsicsResult) -> tuple[float, float]:
        left_delta = max(
            abs(float(result.left.camera_matrix[row][col]) - float(reference.left.camera_matrix[row][col]))
            for row, col in ((0, 0), (1, 1), (0, 2), (1, 2))
        )
        right_delta = max(
            abs(float(result.right.camera_matrix[row][col]) - float(reference.right.camera_matrix[row][col]))
            for row, col in ((0, 0), (1, 1), (0, 2), (1, 2))
        )
        return left_delta, right_delta

    def _checkerboard_validation_stddev(self, values: list[float]) -> float:
        if len(values) < 2:
            return 0.0
        mean = sum(values) / len(values)
        return math.sqrt(sum((value - mean) ** 2 for value in values) / (len(values) - 1))

    def run_checkerboard_random_validation(self) -> None:
        if self.panel.checkerboard_calibration_thread is not None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "A calibration calculation is already running",
                "warn",
            )
            return
        available, error = opencv_available()
        if not available:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, f"OpenCV unavailable: {error}", "bad")
            return
        reference = self.panel.checkerboard_validation_result
        if reference is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "랜덤 검증 전에 전체 사진으로 계산을 먼저 실행하세요.",
                "warn",
            )
            return
        mode = self.panel._checkerboard_validation_mode()
        _root, _left_paths, _right_paths, pairs = self.panel._checkerboard_validation_scan_counts()
        if len(pairs) < 3:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"랜덤 검증 실패: stereo pair가 부족합니다 (pair={len(pairs)}, need>=3)",
                "bad",
            )
            return
        subset_count = self.panel._checkerboard_random_subset_count(len(pairs))
        runs = self.panel.checkerboard_validation_random_runs_spin.value()
        seed = self.panel.checkerboard_validation_seed_spin.value()
        worker = CheckerboardRandomValidationWorker(
            mode=mode,
            pairs=pairs,
            config=self.panel._checkerboard_config(),
            runs=runs,
            subset_count=subset_count,
            seed=seed,
        )
        thread = QtCore.QThread(self.panel)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(
            lambda results,
            reference=reference,
            mode=mode,
            pair_count=len(pairs),
            runs=runs,
            subset_count=subset_count,
            seed=seed: self.panel._handle_random_validation_completed(
                results,
                reference=reference,
                mode=mode,
                pair_count=pair_count,
                runs=runs,
                subset_count=subset_count,
                seed=seed,
            )
        )
        worker.progress.connect(self.panel._handle_calibration_progress)
        worker.failed.connect(self.panel._handle_random_validation_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_checkerboard_calibration_worker(thread))
        self.panel.checkerboard_calibration_thread = thread
        self.panel.checkerboard_calibration_worker = worker
        self.panel.capture_calibration_button.setEnabled(False)
        self.panel.run_calibration_button.setEnabled(False)
        self.panel.checkerboard_validation_random_button.setEnabled(False)
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"Random validation running: runs={runs}, subset_pairs={subset_count}, mode={mode}",
            "warn",
        )
        self.panel.calibration_progress = {
            "phase": "validation",
            "processed": 0,
            "total": subset_count,
            "percent": 0.0,
            "message": f"랜덤 검증 준비: {runs}회 × {subset_count} pair",
            "run": 0,
            "runs": runs,
        }
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(0)
            self.panel.calibration_progress_bar.setFormat("랜덤 검증 준비 · %p%")
        thread.start()

    def _handle_random_validation_failed(self, error: str) -> None:
        self.panel.calibration_progress = {
            "phase": "validation_failed",
            "processed": 0,
            "total": 0,
            "percent": 0.0,
            "message": str(error),
        }
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(0)
            self.panel.calibration_progress_bar.setFormat("랜덤 검증 실패 · %p%")
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"Random validation failed: {error}",
            "bad",
        )
        self.panel._log(f"[캘리브레이션] 랜덤 검증 실패: {error}")

    def _handle_random_validation_completed(
        self,
        results: object,
        *,
        reference: StereoIntrinsicsResult,
        mode: str,
        pair_count: int,
        runs: int,
        subset_count: int,
        seed: int,
    ) -> None:
        rows: list[str] = []
        baselines: list[float] = []
        rotations: list[float] = []
        left_deltas: list[float] = []
        right_deltas: list[float] = []
        reference_rotation = None if reference.extrinsics is None else reference.extrinsics.rotation_left_to_right
        run_results: list[dict[str, object]] = [
            self.panel._checkerboard_validation_result_data(
                "Full",
                reference,
                reference,
                pair_count,
            )
        ]
        rows.append(
            "Run       Pairs  Lerr   Rerr   Serr   Baseline(m)  dB(mm)   dR(deg)  max_dK_L(px)  max_dK_R(px)"
        )
        rows.append(self.panel._checkerboard_validation_result_row("Full", reference, reference, pair_count))
        result_rows = results if isinstance(results, (tuple, list)) else ()
        for row in result_rows:
            if not isinstance(row, dict):
                continue
            index = int(row.get("index") or 0)
            error = str(row.get("error") or "")
            if error:
                rows.append(f"Random {index:<2} FAILED: {error}")
                run_results.append(
                    {
                        "run": f"Random {index}",
                        "pairs": subset_count,
                        "state": "FAILED",
                        "error": error,
                    }
                )
                continue
            result = row.get("result")
            if not isinstance(result, StereoIntrinsicsResult):
                rows.append(f"Random {index:<2} FAILED: invalid worker result")
                run_results.append(
                    {
                        "run": f"Random {index}",
                        "pairs": subset_count,
                        "state": "FAILED",
                        "error": "invalid worker result",
                    }
                )
                continue
            run_results.append(
                self.panel._checkerboard_validation_result_data(
                    f"Random {index}",
                    result,
                    reference,
                    subset_count,
                )
            )
            rows.append(
                self.panel._checkerboard_validation_result_row(
                    f"Random {index}",
                    result,
                    reference,
                    subset_count,
                )
            )
            left_delta, right_delta = self.panel._checkerboard_intrinsic_delta_px(result, reference)
            left_deltas.append(left_delta)
            right_deltas.append(right_delta)
            baseline = self.panel._checkerboard_result_baseline_m(result)
            if baseline is not None:
                baselines.append(baseline)
            rotation = self.panel._rotation_delta_deg_from_matrices(
                reference_rotation,
                None if result.extrinsics is None else result.extrinsics.rotation_left_to_right,
            )
            if rotation is not None:
                rotations.append(rotation)
        baseline_std = self.panel._checkerboard_validation_stddev(baselines)
        rotation_std = self.panel._checkerboard_validation_stddev(rotations)
        rows.extend(
            [
                "",
                "Stability:",
                f"max_dK_L std={self.panel._checkerboard_validation_stddev(left_deltas):.3f} px",
                f"max_dK_R std={self.panel._checkerboard_validation_stddev(right_deltas):.3f} px",
                f"baseline std={baseline_std * 1000.0:.3f} mm" if baselines else "baseline std=n/a",
                f"rotation delta std={rotation_std:.6f} deg" if rotations else "rotation delta std=n/a",
            ]
        )
        summary = dict(self.panel.checkerboard_validation_summary)
        summary["random_validation"] = {
            "runs": runs,
            "subset_pair_count": subset_count,
            "seed": None if seed == 0 else seed,
            "baseline_std_m": baseline_std if baselines else None,
            "rotation_delta_std_deg": rotation_std if rotations else None,
            "left_intrinsics_delta_std_px": self.panel._checkerboard_validation_stddev(left_deltas),
            "right_intrinsics_delta_std_px": self.panel._checkerboard_validation_stddev(right_deltas),
        }
        self.panel.checkerboard_validation_summary = summary
        self.panel.checkerboard_validation_run_results = run_results
        full_text = self.panel.calibration_result_text.toPlainText().rstrip()
        self.panel.calibration_result_text.setPlainText(full_text + "\n\nRandom stereo-pair validation:\n" + "\n".join(rows))
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"랜덤 검증 완료: runs={runs}, subset_pairs={subset_count}, mode={mode}",
            "good",
        )
        self.panel.calibration_progress = {
            "phase": "validation_complete",
            "processed": runs,
            "total": runs,
            "percent": 100.0,
            "message": f"랜덤 검증 완료: {runs}회",
        }
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(100)
            self.panel.calibration_progress_bar.setFormat("랜덤 검증 완료 · %p%")

    def _checkerboard_validation_result_row(
        self,
        name: str,
        result: StereoIntrinsicsResult,
        reference: StereoIntrinsicsResult,
        pairs: int,
    ) -> str:
        baseline = self.panel._checkerboard_result_baseline_m(result)
        reference_baseline = self.panel._checkerboard_result_baseline_m(reference)
        baseline_text = "n/a" if baseline is None else f"{baseline:.6f}"
        baseline_delta_text = (
            "n/a"
            if baseline is None or reference_baseline is None
            else f"{(baseline - reference_baseline) * 1000.0:+.3f}"
        )
        rotation_delta = self.panel._rotation_delta_deg_from_matrices(
            None if reference.extrinsics is None else reference.extrinsics.rotation_left_to_right,
            None if result.extrinsics is None else result.extrinsics.rotation_left_to_right,
        )
        rotation_text = "n/a" if rotation_delta is None else f"{rotation_delta:.6f}"
        stereo_error = "n/a" if result.extrinsics is None else f"{result.extrinsics.reprojection_error_px:.3f}"
        left_delta, right_delta = self.panel._checkerboard_intrinsic_delta_px(result, reference)
        return (
            f"{name:<10} {pairs:>5}  {result.left.reprojection_error_px:>5.3f}  "
            f"{result.right.reprojection_error_px:>5.3f}  {stereo_error:>5}  "
            f"{baseline_text:>11}  {baseline_delta_text:>7}  {rotation_text:>8}  "
            f"{left_delta:>11.3f}  {right_delta:>11.3f}"
        )

    def _checkerboard_validation_result_data(
        self,
        name: str,
        result: StereoIntrinsicsResult,
        reference: StereoIntrinsicsResult,
        pairs: int,
    ) -> dict[str, object]:
        baseline = self.panel._checkerboard_result_baseline_m(result)
        reference_baseline = self.panel._checkerboard_result_baseline_m(reference)
        rotation_delta = self.panel._rotation_delta_deg_from_matrices(
            None if reference.extrinsics is None else reference.extrinsics.rotation_left_to_right,
            None if result.extrinsics is None else result.extrinsics.rotation_left_to_right,
        )
        left_delta, right_delta = self.panel._checkerboard_intrinsic_delta_px(result, reference)
        return {
            "run": name,
            "pairs": pairs,
            "left_reprojection_error_px": result.left.reprojection_error_px,
            "right_reprojection_error_px": result.right.reprojection_error_px,
            "stereo_reprojection_error_px": (
                None if result.extrinsics is None else result.extrinsics.reprojection_error_px
            ),
            "baseline_m": baseline,
            "baseline_delta_mm": (
                None
                if baseline is None or reference_baseline is None
                else (baseline - reference_baseline) * 1000.0
            ),
            "rotation_delta_deg": rotation_delta,
            "left_intrinsics_delta_px": left_delta,
            "right_intrinsics_delta_px": right_delta,
            "state": "OK",
            "error": "",
        }

    def save_checkerboard_validation_result(self) -> None:
        result = self.panel.checkerboard_validation_result
        if result is None:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, "저장할 계산 결과가 없습니다", "warn")
            return
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        root.mkdir(parents=True, exist_ok=True)
        zoom_level = self.panel.zoom_level_spin.value()
        zoom_output = self.panel._intrinsics_storage_path(zoom_level)
        latest_output = self.panel._intrinsics_storage_base_path()
        metadata = {
            "note": "Generated by StereoWIRIS checkerboard validation/calculation tab",
            "timestamp_s": time.time(),
            "zoom_level": zoom_level,
            "zoom_target": str(self.panel.zoom_target_combo.currentData() or "both"),
            "calculation_mode": self.panel.checkerboard_validation_result_mode,
            "validation": self.panel.checkerboard_validation_summary,
        }
        save_intrinsics_json(zoom_output, result, metadata=metadata)
        if zoom_output != latest_output:
            save_intrinsics_json(latest_output, result, metadata=metadata)
        self.panel.checkerboard_validation_result_saved = True
        self.panel.last_calibration_zoom_level = zoom_level
        self.panel._add_zoom_intrinsics_comparison_path(zoom_output)
        self.panel._save_current_settings()
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"계산 결과 저장 완료: zoom {zoom_level:02d} | {zoom_output}",
            "good",
        )
        self.panel._refresh_operation_header()
        self.panel._refresh_zoom_intrinsics_tab()

    def apply_checkerboard_validation_result(self) -> None:
        result = self.panel.checkerboard_validation_result
        if result is None:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, "적용할 계산 결과가 없습니다", "warn")
            return
        mode = self.panel.checkerboard_validation_result_mode
        if mode == "checkerboard" and result.extrinsics is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "체커보드 Stereo 모드는 stereo R/T가 있는 결과만 적용할 수 있습니다",
                "bad",
            )
            return
        self.panel.last_calibration_result = result
        self.panel.last_calibration_zoom_level = self.panel.zoom_level_spin.value()
        self.panel._set_combo_data(self.panel.distance_mode_combo, mode)
        self.panel.apply_intrinsics_to_distance()
        self.panel._save_current_settings()
        saved_text = "저장됨" if self.panel.checkerboard_validation_result_saved else "저장 전"
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"계산에 적용 완료: mode={mode} | {saved_text}",
            "good",
        )

    def _checkerboard_config(self) -> CheckerboardConfig:
        return CheckerboardConfig(
            inner_cols=self.panel.calib_cols_spin.value(),
            inner_rows=self.panel.calib_rows_spin.value(),
            square_size_m=self.panel.calib_square_spin.value() / 1000.0,
        )

    def _calibration_dirs(self) -> tuple[Path, Path, Path]:
        text = self.panel.calib_dir_edit.text().strip()
        if text:
            root = resolve_settings_path(text, base_dir=self.panel.app_root)
        else:
            root = resolve_settings_path(DEFAULT_INTRINSICS_PATH, base_dir=self.panel.app_root).parent
        left_dir = root / "left"
        right_dir = root / "right"
        return root, left_dir, right_dir

    def _intrinsics_storage_base_path(self) -> Path:
        if hasattr(self.panel, "calib_dir_edit"):
            root, _left_dir, _right_dir = self.panel._calibration_dirs()
            path = root / "intrinsics.json"
        else:
            path = resolve_settings_path(self.panel.settings.intrinsics_path, base_dir=self.panel.app_root)
        stem = path.stem
        if "_zoom_" in stem:
            base_stem, zoom_suffix = stem.rsplit("_zoom_", 1)
            if zoom_suffix.isdigit():
                return path.with_name(f"{base_stem}{path.suffix}")
        return path

    def _intrinsics_storage_path(self, zoom_level: int | None = None) -> Path:
        base_path = self.panel._intrinsics_storage_base_path()
        level = self.panel.zoom_level_spin.value() if zoom_level is None else int(zoom_level)
        return base_path.with_name(f"{base_path.stem}_zoom_{level:02d}{base_path.suffix}")

    def _extrinsics_storage_path(self, zoom_level: int | None = None) -> Path:
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        selected_zoom = getattr(self.panel, "last_calibration_zoom_level", None)
        level = (
            self.panel.zoom_level_spin.value()
            if zoom_level is None and selected_zoom is None
            else selected_zoom
            if zoom_level is None
            else int(zoom_level)
        )
        return root / f"extrinsics_zoom_{int(level):02d}_large_board.json"

    def _calibration_image_paths(self, side: str) -> list[Path]:
        _root, left_dir, right_dir = self.panel._calibration_dirs()
        directory = left_dir if side == "left" else right_dir
        return sorted(directory.glob(f"{side}_*.png"))

    def _next_calibration_capture_stem(self) -> str:
        max_index = self.panel.calibration_capture_index
        for side in ("left", "right"):
            for path in self.panel._calibration_image_paths(side):
                suffix = path.stem.replace(f"{side}_", "", 1)
                if suffix.isdigit():
                    max_index = max(max_index, int(suffix))
        self.panel.calibration_capture_index = max_index + 1
        return f"{self.panel.calibration_capture_index:03d}"

    def _checkerboard_capture_pair_timing(
        self,
        metadata_by_camera: dict[str, dict[str, object]] | None = None,
    ) -> tuple[bool, str, float | None, str]:
        metadata_by_camera = metadata_by_camera or self.panel.camera_frame_metadata
        left_metadata = metadata_by_camera.get("left") or {}
        right_metadata = metadata_by_camera.get("right") or {}
        left_timestamp = self.panel._camera_rtc_timestamp_s_from_metadata(left_metadata)
        right_timestamp = self.panel._camera_rtc_timestamp_s_from_metadata(right_metadata)
        if left_timestamp is None or right_timestamp is None:
            return (
                False,
                "rtcp-ntp",
                None,
                "missing RTCP/NTP camera timestamp for left or right frame",
            )
        delta_s = float(right_timestamp) - float(left_timestamp)
        if abs(delta_s) > CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S:
            return (
                False,
                "rtcp-ntp",
                delta_s,
                f"RTCP/NTP pair delta {delta_s * 1000.0:.1f} ms exceeds "
                f"{CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S * 1000.0:.1f} ms",
            )
        return True, "rtcp-ntp", delta_s, "ok"

    def capture_calibration_images(
        self,
        *,
        frames: dict[str, QtGui.QImage | None] | None = None,
        metadata_by_camera: dict[str, dict[str, object]] | None = None,
        capture_origin: str = "manual",
    ) -> bool:
        if self.panel.checkerboard_calibration_thread is not None:
            self.panel.calibration_status_label.setText("A calibration operation is already running.")
            return False
        frames = frames or {
            "left": self.panel.left_pane.current_image(),
            "right": self.panel.right_pane.current_image(),
        }
        if frames["left"] is None and frames["right"] is None:
            self.panel.calibration_status_label.setText("Start at least one camera stream before capturing calibration images.")
            self.panel._log("[캘리브레이션] 이미지 캡처 실패: 좌/우 카메라 프레임 중 하나 이상이 필요합니다.")
            return False
        frames = {
            side: None
            if image is None
            else image.convertToFormat(QtGui.QImage.Format_RGB888).copy()
            for side, image in frames.items()
        }
        metadata_by_camera = metadata_by_camera or self.panel.camera_frame_metadata
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        stem = self.panel._next_calibration_capture_stem()
        available, error = opencv_available()
        config = self.panel._checkerboard_config() if available else None
        pair_candidate = frames["left"] is not None and frames["right"] is not None
        pair_timing_ok = False
        pair_time_basis = "rtcp-ntp"
        pair_delta_s: float | None = None
        pair_timing_detail = "single-camera capture"
        if pair_candidate:
            pair_timing_ok, pair_time_basis, pair_delta_s, pair_timing_detail = self.panel._checkerboard_capture_pair_timing(
                metadata_by_camera
            )
        metadata_payload = {
            "capture_origin": str(capture_origin),
            "pair_time_basis": pair_time_basis,
            "pair_delta_s": pair_delta_s,
            "pair_max_delta_s": CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S,
            "pair_timing_status": pair_timing_detail,
            "left_rtcp_frame_ntp_unix_s": (metadata_by_camera.get("left") or {}).get("rtcp_frame_ntp_unix_s"),
            "right_rtcp_frame_ntp_unix_s": (metadata_by_camera.get("right") or {}).get("rtcp_frame_ntp_unix_s"),
            "frame_contract": "OpenCV image pixels; stereo timing uses RTCP/NTP camera timestamp before saving a left/right pair",
        }
        worker = CheckerboardCaptureWorker(
            root=root,
            frames=frames,
            stem=stem,
            config=config,
            opencv_error=error,
            pair_timing_ok=pair_timing_ok,
            pair_time_basis=pair_time_basis,
            pair_delta_s=pair_delta_s,
            pair_timing_detail=pair_timing_detail,
            metadata_payload=metadata_payload,
        )
        thread = QtCore.QThread(self.panel)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_checkerboard_capture_completed)
        worker.failed.connect(self.panel._handle_checkerboard_capture_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_checkerboard_calibration_worker(thread))
        self.panel.checkerboard_calibration_thread = thread
        self.panel.checkerboard_calibration_worker = worker
        self._capture_origin = str(capture_origin)
        self.panel.capture_calibration_button.setEnabled(False)
        self.panel.run_calibration_button.setEnabled(False)
        self.panel.checkerboard_validation_random_button.setEnabled(False)
        self.panel.calibration_status_label.setText(f"Capture {stem}: encoding and checkerboard validation running")
        thread.start()
        return True

    def toggle_auto_calibration_capture(self) -> None:
        if self._auto_capture_active:
            self.stop_auto_calibration_capture()
        else:
            self.start_auto_calibration_capture()

    def start_auto_calibration_capture(self) -> bool:
        if self._auto_capture_active:
            return True
        if self.panel.checkerboard_calibration_thread is not None:
            self.panel.calibration_status_label.setText("A calibration operation is already running.")
            return False
        interval_s = float(self.panel.calibration_auto_interval_spin.value())
        target_pairs = int(self.panel.calibration_auto_target_spin.value())
        self._auto_capture_active = True
        self._auto_capture_saved_pairs = 0
        self._auto_capture_last_pair_key = None
        self._auto_capture_inflight_pair_key = None
        self._auto_capture_last_attempt_s = 0.0
        self._auto_capture_timer.start(max(100, int(interval_s * 1000.0)))
        self.panel.capture_calibration_auto_button.setText("자동 촬영 중지")
        self.panel.calibration_status_label.setText(
            f"자동 촬영 대기: 유효 stereo pair 0/{target_pairs} | "
            f"간격={interval_s:.1f}s | 좌·우 보드를 움직여 주세요."
        )
        self._auto_calibration_capture_tick()
        return True

    def stop_auto_calibration_capture(self, message: str | None = None) -> bool:
        was_active = self._auto_capture_active or self._auto_capture_timer.isActive()
        self._auto_capture_timer.stop()
        self._auto_capture_active = False
        self._auto_capture_inflight_pair_key = None
        if hasattr(self.panel, "capture_calibration_auto_button"):
            self.panel.capture_calibration_auto_button.setText("자동 촬영 시작")
        if message is not None:
            self.panel.calibration_status_label.setText(message)
        elif was_active:
            self.panel.calibration_status_label.setText(
                f"자동 촬영 중지: 유효 stereo pair {self._auto_capture_saved_pairs}개 저장"
            )
        return True

    def _auto_calibration_capture_tick(self) -> None:
        if not self._auto_capture_active:
            return
        if self.panel.checkerboard_calibration_thread is not None:
            return
        target_pairs = int(self.panel.calibration_auto_target_spin.value())
        if self._auto_capture_saved_pairs >= target_pairs:
            self.stop_auto_calibration_capture(
                f"자동 촬영 완료: 유효 stereo pair {self._auto_capture_saved_pairs}/{target_pairs}개 저장"
            )
            return

        detections = getattr(self.panel, "calibration_preview_detection_results", {})
        left = detections.get("left") if isinstance(detections, dict) else None
        right = detections.get("right") if isinstance(detections, dict) else None
        if not isinstance(left, dict) or not isinstance(right, dict):
            self.panel.calibration_status_label.setText("자동 촬영 대기: 좌·우 체커보드 검출 결과를 기다리는 중")
            return
        if not bool(left.get("found")) or not bool(right.get("found")):
            self.panel.calibration_status_label.setText("자동 촬영 대기: 좌·우 화면에서 체커보드가 모두 검출되어야 합니다")
            return
        left_image = left.get("image")
        right_image = right.get("image")
        if not isinstance(left_image, QtGui.QImage) or not isinstance(right_image, QtGui.QImage):
            self.panel.calibration_status_label.setText("자동 촬영 대기: 좌·우 최신 프레임을 기다리는 중")
            return

        try:
            left_request_id = int(left.get("request_id") or 0)
            right_request_id = int(right.get("request_id") or 0)
        except (TypeError, ValueError):
            return
        if left_request_id <= 0 or right_request_id <= 0:
            return
        pair_key = (left_request_id, right_request_id)
        if pair_key == self._auto_capture_last_pair_key:
            return

        metadata_by_camera = {
            "left": dict(left.get("metadata") or {}) if isinstance(left.get("metadata"), dict) else {},
            "right": dict(right.get("metadata") or {}) if isinstance(right.get("metadata"), dict) else {},
        }
        now_s = time.monotonic()
        for camera in ("left", "right"):
            timestamp_s = metadata_by_camera[camera].get("timestamp_s")
            try:
                age_s = now_s - float(timestamp_s)
            except (TypeError, ValueError):
                self.panel.calibration_status_label.setText(
                    "자동 촬영 대기: 좌·우 프레임의 수신 시각이 없습니다"
                )
                return
            if age_s < 0.0 or age_s > CALIBRATION_AUTO_DETECTION_MAX_AGE_S:
                self.panel.calibration_status_label.setText(
                    "자동 촬영 대기: 체커보드 검출 프레임이 오래되어 새 프레임을 기다리는 중"
                )
                return

        pair_timing_ok, _pair_time_basis, _pair_delta_s, pair_timing_detail = (
            self.panel._checkerboard_capture_pair_timing(metadata_by_camera)
        )
        if not pair_timing_ok:
            self.panel.calibration_status_label.setText(
                f"자동 촬영 대기: 좌·우 동기 pair가 아님 ({pair_timing_detail})"
            )
            return

        self._auto_capture_last_pair_key = pair_key
        self._auto_capture_last_attempt_s = now_s
        self._auto_capture_inflight_pair_key = pair_key
        started = self.capture_calibration_images(
            frames={"left": left_image, "right": right_image},
            metadata_by_camera=metadata_by_camera,
            capture_origin="auto",
        )
        if not started:
            self._auto_capture_inflight_pair_key = None
            return
        self.panel.calibration_status_label.setText(
            f"자동 촬영 중: 유효 stereo pair {self._auto_capture_saved_pairs}/{target_pairs} | "
            f"Capture worker가 보드 검증 중"
        )

    def _handle_checkerboard_capture_completed(self, result: object) -> None:
        capture_origin = self._capture_origin
        if not isinstance(result, dict):
            self.panel._handle_checkerboard_capture_failed("worker returned an invalid capture result")
            return
        stem = str(result.get("stem") or "--")
        saved_sides = tuple(str(side) for side in result.get("saved_sides", ()) or ())
        status_parts = tuple(str(part) for part in result.get("status_parts", ()) or ())
        if not saved_sides:
            status = f"Capture {stem}: no calibration images saved; " + ", ".join(status_parts)
            if capture_origin == "auto" and self._auto_capture_active:
                status = f"자동 촬영 대기: Capture {stem} 실패; 다음 새 검출 pair를 기다리는 중"
            self.panel.calibration_status_label.setText(status)
            self.panel._log(f"[캘리브레이션] 이미지 캡처 실패 {stem}: {', '.join(status_parts)}")
            if capture_origin == "auto":
                self._auto_capture_inflight_pair_key = None
                self._capture_origin = "manual"
            return
        status = f"Captured {stem}: " + ", ".join(status_parts)
        pair_delta_s = result.get("pair_delta_s")
        if (
            bool(result.get("pair_timing_ok"))
            and isinstance(pair_delta_s, (int, float))
            and set(saved_sides) == {"left", "right"}
        ):
            status += (
                f"; pair_time_basis={result.get('pair_time_basis', 'rtcp-ntp')}, "
                f"pair_delta_s={float(pair_delta_s):.6f}"
            )
        self.panel.calibration_status_label.setText(status)
        self.panel._log(
            f"[캘리브레이션] 이미지 캡처 {stem}: "
            f"저장={','.join(saved_sides)}; {', '.join(status_parts)}"
        )
        if capture_origin == "auto":
            self._auto_capture_inflight_pair_key = None
            pair_saved = bool(result.get("pair_timing_ok")) and set(saved_sides) == {"left", "right"}
            if pair_saved:
                self._auto_capture_saved_pairs += 1
            self._capture_origin = "manual"
            if self._auto_capture_active:
                target_pairs = int(self.panel.calibration_auto_target_spin.value())
                if self._auto_capture_saved_pairs >= target_pairs:
                    self.stop_auto_calibration_capture(
                        f"자동 촬영 완료: 유효 stereo pair {self._auto_capture_saved_pairs}/{target_pairs}개 저장"
                    )
                else:
                    self.panel.calibration_status_label.setText(
                        f"자동 촬영 대기: 유효 stereo pair "
                        f"{self._auto_capture_saved_pairs}/{target_pairs}개 저장됨"
                    )

    def _handle_checkerboard_capture_failed(self, error: str) -> None:
        if self._capture_origin == "auto" and self._auto_capture_active:
            self.panel.calibration_status_label.setText(f"자동 촬영 대기: Capture worker 실패 ({error})")
        else:
            self.panel.calibration_status_label.setText(f"Calibration capture failed: {error}")
        self.panel._log(f"[캘리브레이션] 이미지 캡처 worker 실패: {error}")
        if self._capture_origin == "auto":
            self._auto_capture_inflight_pair_key = None
            self._capture_origin = "manual"

    def run_intrinsic_calibration(self) -> None:
        if self._auto_capture_active:
            self.stop_auto_calibration_capture()
        if self.panel.checkerboard_calibration_thread is not None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "Calibration is already running",
                "warn",
            )
            return
        available, error = opencv_available()
        if not available:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, f"OpenCV unavailable: {error}", "bad")
            self.panel.calibration_result_text.setPlainText(f"OpenCV unavailable:\n{error}")
            return
        mode = self.panel._checkerboard_validation_mode()
        root, left_paths, right_paths, pairs = self.panel._checkerboard_validation_scan_counts()
        source_pair_count = len(pairs)
        calculation_pair_limit = self._calibration_calculation_pair_limit()
        selected_pairs = self._select_calibration_pairs(pairs)
        if mode == "checkerboard":
            left_paths = [left for _key, left, _right in selected_pairs]
            right_paths = [right for _key, _left, right in selected_pairs]
        elif calculation_pair_limit > 0:
            left_paths = [left for _key, left, _right in selected_pairs]
            right_paths = [right for _key, _left, right in selected_pairs]
        pair_count = len(selected_pairs) if calculation_pair_limit > 0 or mode == "checkerboard" else len(pairs)
        min_images = self.panel.calib_min_images_spin.value()
        if not left_paths or not right_paths:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"계산 실패: left/right 이미지가 부족합니다 (left={len(left_paths)}, right={len(right_paths)})",
                "bad",
            )
            return
        if mode == "checkerboard" and pair_count < 3:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"계산 실패: stereo pair가 부족합니다 (pair={pair_count}, need>=3)",
                "bad",
            )
            return
        self.panel.scan_checkerboard_validation_files()
        worker = CheckerboardCalibrationWorker(
            mode=mode,
            left_paths=left_paths,
            right_paths=right_paths,
            config=self.panel._checkerboard_config(),
        )
        thread = QtCore.QThread(self.panel)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(
            lambda result,
            mode=mode,
            root=root,
            pair_count=pair_count,
            min_images=min_images,
            left_count=len(left_paths),
            right_count=len(right_paths): self.panel._handle_intrinsic_calibration_completed(
                result,
                mode=mode,
                root=root,
                pair_count=pair_count,
                min_images=min_images,
                left_count=left_count,
                right_count=right_count,
            )
        )
        worker.progress.connect(self.panel._handle_calibration_progress)
        worker.failed.connect(self.panel._handle_intrinsic_calibration_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_checkerboard_calibration_worker(thread))
        self.panel.checkerboard_calibration_thread = thread
        self.panel.checkerboard_calibration_worker = worker
        self.panel.capture_calibration_button.setEnabled(False)
        self.panel.run_calibration_button.setEnabled(False)
        self.panel.checkerboard_validation_random_button.setEnabled(False)
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"Calibration running: mode={mode}, calculation pairs={pair_count}/{source_pair_count}, "
            f"left={len(left_paths)}, right={len(right_paths)}",
            "warn",
        )
        self.panel.calibration_progress = {
            "phase": "queued",
            "processed": 0,
            "total": max(len(left_paths), len(right_paths), pair_count),
            "percent": 0.0,
            "message": (
                f"계산 준비: {pair_count}/{source_pair_count} pair"
                if calculation_pair_limit > 0 or mode == "checkerboard"
                else f"계산 준비: left={len(left_paths)}, right={len(right_paths)}"
            ),
        }
        thread.start()

    def run_stereo_extrinsics_calibration(self) -> None:
        """Run only OpenCV stereo R/T with the selected K/D held fixed."""

        if self._auto_capture_active:
            self.stop_auto_calibration_capture()
        if self.panel.checkerboard_calibration_thread is not None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "Calibration is already running",
                "warn",
            )
            return
        intrinsics = self.panel.last_calibration_result
        if intrinsics is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "R,t only 계산 전에 source K,D profile을 계산하거나 불러오세요",
                "bad",
            )
            return
        available, error = opencv_available()
        if not available:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"OpenCV unavailable: {error}",
                "bad",
            )
            return
        root, _left_paths, _right_paths, pairs = self.panel._checkerboard_validation_scan_counts()
        selected_pairs = self._select_calibration_pairs(pairs)
        if len(selected_pairs) < 3:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"R,t only 계산 실패: matching stereo pair={len(selected_pairs)}, need>=3",
                "bad",
            )
            return
        left_paths = [left for _key, left, _right in selected_pairs]
        right_paths = [right for _key, _left, right in selected_pairs]
        source_zoom = getattr(self.panel, "last_calibration_zoom_level", None)
        source_path = str(getattr(self.panel, "_active_intrinsics_profile_path", "") or "")
        source_hash = intrinsics_content_sha256(intrinsics)
        worker = CheckerboardExtrinsicsWorker(
            left_paths=left_paths,
            right_paths=right_paths,
            config=self.panel._checkerboard_config(),
            left_intrinsics=intrinsics.left,
            right_intrinsics=intrinsics.right,
        )
        thread = QtCore.QThread(self.panel)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(
            lambda result,
            source_zoom=source_zoom,
            source_path=source_path,
            source_hash=source_hash,
            root=root: self.panel._handle_stereo_extrinsics_completed(
                result,
                source_zoom=source_zoom,
                source_path=source_path,
                source_hash=source_hash,
                root=root,
            )
        )
        worker.progress.connect(self.panel._handle_calibration_progress)
        worker.failed.connect(self.panel._handle_intrinsic_calibration_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_checkerboard_calibration_worker(thread))
        self.panel.checkerboard_calibration_thread = thread
        self.panel.checkerboard_calibration_worker = worker
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"R,t only 계산 중: fixed K,D zoom={source_zoom}, pairs={len(selected_pairs)}",
            "warn",
        )
        thread.start()

    def _handle_stereo_extrinsics_completed(
        self,
        result: object,
        *,
        source_zoom: int | None,
        source_path: str,
        source_hash: str,
        root: Path,
    ) -> None:
        if not isinstance(result, StereoExtrinsics):
            self.panel._handle_intrinsic_calibration_failed(
                "worker returned invalid stereo extrinsics"
            )
            return
        profile = StereoExtrinsicsProfile(
            extrinsics=result,
            source_zoom_level=source_zoom,
            method="checkerboard_external_only",
            source_intrinsics_path=source_path,
            source_intrinsics_sha256=source_hash,
            profile_id=(
                f"extrinsics_zoom_{int(source_zoom):02d}_large_board"
                if source_zoom is not None
                else "extrinsics_large_board"
            ),
            metadata={
                "timestamp_s": time.time(),
                "calibration_root": str(root),
                "validation_managed_by_application": False,
            },
        )
        self.panel.selected_extrinsics_profile = profile
        self.panel._selected_extrinsics_profile_path = (
            f"memory:{profile.profile_id}"
        )
        self.panel._selected_extrinsics_profile_hash = profile.content_sha256
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"R,t only candidate 준비됨: source zoom={source_zoom}, "
            f"pairs={result.pair_count}, baseline={profile.baseline_m:.6f} m",
            "good",
        )
        self.panel._log(
            "[캘리브레이션] 고정 K,D 기반 R,t 계산 완료: "
            f"source_zoom={source_zoom}, frame={result.frame_contract}"
        )
        self.panel._refresh_camera_parameters_tab()
        self.panel._refresh_operation_header()

    def save_stereo_extrinsics_profile(self) -> None:
        profile = getattr(self.panel, "selected_extrinsics_profile", None)
        if not isinstance(profile, StereoExtrinsicsProfile):
            self.panel.calibration_status_label.setText(
                "Calculate or load an R,t profile before saving."
            )
            return
        output = self.panel._extrinsics_storage_path(profile.source_zoom_level)
        output.parent.mkdir(parents=True, exist_ok=True)
        save_stereo_extrinsics_json(output, profile)
        self.panel._selected_extrinsics_profile_path = str(output)
        self.panel._selected_extrinsics_profile_hash = profile.content_sha256
        self.panel.calibration_status_label.setText(f"Saved stereo R,t profile: {output}")
        self.panel._log(f"[캘리브레이션] 독립 R,t profile 저장: {output}")
        self.panel._refresh_operation_header()

    def load_stereo_extrinsics_profile(self) -> None:
        initial_path = self.panel._extrinsics_storage_path()
        selected, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self.panel,
            "Load stereo R,t profile",
            str(initial_path if initial_path.exists() else initial_path.parent),
            "Stereo extrinsics JSON (*.json);;All files (*)",
        )
        if not selected:
            return
        path = Path(selected)
        try:
            profile = load_stereo_extrinsics_json(path)
        except Exception as exc:
            self.panel.calibration_status_label.setText(f"Failed to load stereo R,t: {exc}")
            self.panel._log(f"[캘리브레이션] R,t profile 로드 실패: {path}: {exc}")
            return
        self.panel.selected_extrinsics_profile = profile
        self.panel._selected_extrinsics_profile_path = str(path)
        self.panel._selected_extrinsics_profile_hash = profile.content_sha256
        self.panel.calibration_status_label.setText(
            f"Loaded stereo R,t candidate: {path.name} (source zoom={profile.source_zoom_level})"
        )
        self.panel._log(
            f"[캘리브레이션] 독립 R,t profile 로드: {path}; "
            "cross-zoom 조합은 검증 gate 없이 허용"
        )
        self.panel._refresh_camera_parameters_tab()
        self.panel._refresh_operation_header()

    def apply_calibration_bundle(self) -> None:
        """Apply the independently selected K/D and R/T without validation gating."""

        self.panel.apply_intrinsics_to_distance()

    def _handle_calibration_progress(self, progress: object) -> None:
        if not isinstance(progress, dict):
            return
        payload = dict(progress)
        self.panel.calibration_progress = payload
        message = str(payload.get("message") or payload.get("phase") or "계산 중")
        try:
            percent = max(0.0, min(100.0, float(payload.get("percent", 0.0))))
        except (TypeError, ValueError):
            percent = 0.0
        text = f"계산 중: {message} ({percent:.0f}%)"
        if hasattr(self.panel, "checkerboard_validation_status_label"):
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                text,
                "warn",
            )
        if hasattr(self.panel, "calibration_status_label"):
            self.panel.calibration_status_label.setText(text)
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(round(percent))
            self.panel.calibration_progress_bar.setFormat(f"{message} · %p%")

    def _handle_intrinsic_calibration_completed(
        self,
        result: object,
        *,
        mode: str,
        root: Path,
        pair_count: int,
        min_images: int,
        left_count: int,
        right_count: int,
    ) -> None:
        if not isinstance(result, StereoIntrinsicsResult):
            self.panel._handle_intrinsic_calibration_failed("worker returned an invalid calibration result")
            return
        warning = (
            f"\nWARNING: valid images below requested minimum {min_images}."
            if result.left.image_count < min_images or result.right.image_count < min_images
            else ""
        )
        text = self.panel._format_checkerboard_validation_result_summary(
            result,
            mode=mode,
            title="Full checkerboard calibration result",
            root=root,
            pair_count=pair_count if mode == "checkerboard" else None,
        ) + warning
        self.panel._checkerboard_validation_set_result(
            result,
            mode=mode,
            summary={
                "source": "full",
                "root": str(root),
                "mode": mode,
                "left_image_count": left_count,
                "right_image_count": right_count,
                "stereo_pair_count": pair_count,
                "min_images": min_images,
            },
            text=text,
        )
        if mode == "checkerboard":
            self.panel._log("[캘리브레이션] 전체 사진 기반 intrinsic 및 체커보드 stereo 외부 파라미터 계산을 완료했습니다.")
        else:
            self.panel._log("[캘리브레이션] 전체 사진 기반 Pixhawk 모드용 intrinsic 계산을 완료했습니다.")
        self.panel._refresh_operation_header()
        self.panel.calibration_progress = {
            "phase": "complete",
            "processed": 1,
            "total": 1,
            "percent": 100.0,
            "message": "계산 완료",
        }
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(100)
            self.panel.calibration_progress_bar.setFormat("계산 완료 · %p%")

    def _handle_intrinsic_calibration_failed(self, error: str) -> None:
        self.panel.calibration_progress = {
            "phase": "failed",
            "processed": 0,
            "total": 0,
            "percent": 0.0,
            "message": str(error),
        }
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(0)
            self.panel.calibration_progress_bar.setFormat("계산 실패 · %p%")
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"Calibration failed: {error}",
            "bad",
        )
        self.panel.calibration_result_text.setPlainText(str(error))
        self.panel._log(f"[캘리브레이션] 검증/계산 실패: {error}")

    def _clear_checkerboard_calibration_worker(self, thread: QtCore.QThread) -> None:
        if self.panel.checkerboard_calibration_thread is thread:
            self.panel.checkerboard_calibration_thread = None
            self.panel.checkerboard_calibration_worker = None
            if hasattr(self.panel, "capture_calibration_button"):
                self.panel.capture_calibration_button.setEnabled(True)
            if hasattr(self.panel, "run_calibration_button"):
                self.panel.run_calibration_button.setEnabled(True)
            if hasattr(self.panel, "checkerboard_validation_random_button"):
                self.panel.checkerboard_validation_random_button.setEnabled(
                    self.panel.checkerboard_validation_result is not None
                )
        thread.deleteLater()
