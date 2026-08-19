"""Workstation checkerboard calibration workflow controller.

Image observations and calibration matrices retain OpenCV optical RDF
(right, down, forward) conventions.  Stereo ``T`` follows
``X_right = R_left_to_right * X_left + T_left_to_right_m``; the right-camera
origin expressed in left RDF is the distinct value ``-R^T T``.  This module
performs no NED/ENU aliasing.
"""

from __future__ import annotations

import json
import math
import time
import uuid
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
    compose_stereo_calibration,
    independent_zoom_selection_mode,
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
from Workstation.wiris_runtime_workers import (
    WirisFtpBatchDeleteWorker,
    WirisImageFormatWorker,
    WirisOzFtpSyncWorker,
    WirisOZPhotoCaptureWorker,
    WirisPnPPhotoCaptureWorker,
)

CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S = 0.100
CALIBRATION_AUTO_DETECTION_MAX_AGE_S = 2.0

CALIBRATION_WORKFLOW_METHODS = [
    "_app_relative_path_text",
    "_checkerboard_validation_mode",
    "_checkerboard_validation_root",
    "_capture_zoom_snapshot",
    "_capture_zoom_level_for_paths",
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
    "apply_intrinsics_to_distance",
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
    "_clear_checkerboard_calibration_worker",
    "workstation_checkerboard_validation_snapshot",
    "configure_workstation_checkerboard_validation",
    "load_workstation_checkerboard_validation_json",
    "run_workstation_checkerboard_validation",
    "save_workstation_checkerboard_validation_result",
    "apply_workstation_checkerboard_validation_result",
    "capture_calibration_images_wiris_ftp",
    "_handle_calibration_wiris_ftp_capture_completed",
    "_handle_calibration_wiris_ftp_capture_failed",
    "_clear_calibration_wiris_ftp_capture_worker",
    "capture_calibration_images_wiris_ftp_oz",
    "_handle_calibration_wiris_ftp_oz_capture_completed",
    "_handle_calibration_wiris_ftp_oz_capture_failed",
    "_clear_calibration_wiris_ftp_oz_capture_worker",
    "capture_calibration_images_wiris_oz_capture_only",
    "_handle_calibration_wiris_oz_format_preset_completed",
    "_handle_calibration_wiris_oz_format_preset_failed",
    "_clear_calibration_wiris_oz_format_worker",
    "_start_calibration_wiris_oz_capture",
    "_handle_calibration_wiris_oz_capture_only_completed",
    "_handle_calibration_wiris_oz_capture_only_failed",
    "_clear_calibration_wiris_oz_capture_only_worker",
    "sync_calibration_images_wiris_oz_ftp",
    "_handle_calibration_wiris_oz_gilc_completed",
    "_handle_calibration_wiris_oz_gilc_failed",
    "_clear_calibration_wiris_oz_gilc_worker",
    "_handle_calibration_wiris_oz_sync_completed",
    "_handle_calibration_wiris_oz_sync_failed",
    "_clear_calibration_wiris_oz_sync_worker",
    "_process_next_ftp_oz_sync_pair",
    "_start_calibration_wiris_oz_delete",
    "_handle_calibration_wiris_oz_delete_completed",
    "_handle_calibration_wiris_oz_delete_failed",
    "_clear_calibration_wiris_oz_delete_worker",
    "_calib_oz_ledger_path",
    "_load_calib_oz_ledger",
    "_save_calib_oz_ledger",
    "_calibration_pair_selection_path",
    "_load_calibration_pair_selection",
    "_choose_calibration_pairs",
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

    @staticmethod
    def _zoom_index(value: object) -> int | None:
        try:
            level = int(value)
        except (TypeError, ValueError):
            return None
        return level if level >= 0 else None

    def _capture_zoom_snapshot(self) -> dict[str, object]:
        """Capture the latest camera readback alongside every calibration image."""

        actual = getattr(self.panel, "actual_visible_zoom", {}) or {}
        values: dict[str, int | None] = {}
        readbacks: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            row = actual.get(side, {}) if isinstance(actual, dict) else {}
            readback = dict(row) if isinstance(row, dict) else {}
            readbacks[side] = readback
            values[side] = self._zoom_index(readback.get("index"))
        shared_level = (
            values["left"]
            if values["left"] is not None and values["left"] == values["right"]
            else None
        )
        requested = getattr(self.panel, "zoom_level_spin", None)
        try:
            requested_level = int(requested.value()) if requested is not None else None
        except (AttributeError, TypeError, ValueError):
            requested_level = None
        return {
            "capture_zoom_level": shared_level,
            "capture_left_zoom_level": values["left"],
            "capture_right_zoom_level": values["right"],
            "capture_zoom_source": "GZVV" if shared_level is not None else "unresolved",
            "capture_zoom_readback": readbacks,
            "requested_zoom_level": requested_level,
        }

    def _capture_zoom_level_for_paths(
        self,
        root: Path,
        paths: list[Path] | tuple[Path, ...],
    ) -> tuple[int | None, str]:
        """Resolve one common zoom from the capture metadata selected for a run."""

        capture_ids = {
            path.stem.split("_", 1)[1]
            for path in paths
            if "_" in path.stem
        }
        if not capture_ids:
            return None, "capture IDs are missing from the calibration image names"
        levels: dict[str, int] = {}
        missing: list[str] = []
        metadata_root = root / "capture_metadata"
        for capture_id in sorted(capture_ids):
            metadata_path = metadata_root / f"capture_{capture_id}.json"
            try:
                payload = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                missing.append(capture_id)
                continue
            level = self._zoom_index(payload.get("capture_zoom_level"))
            if level is None:
                left_level = self._zoom_index(payload.get("capture_left_zoom_level"))
                right_level = self._zoom_index(payload.get("capture_right_zoom_level"))
                if left_level is not None and left_level == right_level:
                    level = left_level
            if level is None:
                missing.append(capture_id)
            else:
                levels[capture_id] = level
        if missing:
            return (
                None,
                "capture-time zoom readback is missing for "
                + ", ".join(missing[:5])
                + (" …" if len(missing) > 5 else ""),
            )
        unique_levels = sorted(set(levels.values()))
        if len(unique_levels) != 1:
            return None, f"mixed capture zoom levels: {', '.join(str(level) for level in unique_levels)}"
        return unique_levels[0], f"capture-time zoom={unique_levels[0]}"

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
        zoom_level: int | None = None,
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
        if zoom_level is not None:
            self.panel.last_calibration_zoom_level = int(zoom_level)
        elif getattr(self.panel, "last_calibration_zoom_level", None) is None:
            self.panel.last_calibration_zoom_level = self.panel.zoom_level_spin.value()
        self.panel._active_intrinsics_profile_path = (
            f"memory:checkerboard-validation:{mode}:zoom-"
            f"{self.panel.last_calibration_zoom_level:02d}"
            if self.panel.last_calibration_zoom_level is not None
            else f"memory:checkerboard-validation:{mode}:zoom-unknown"
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
        zoom_level = getattr(self.panel, "last_calibration_zoom_level", None)
        if zoom_level is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "저장 실패: 계산 결과에 촬영 당시 줌 readback이 없습니다",
                "bad",
            )
            return
        zoom_output = self.panel._intrinsics_storage_path(zoom_level)
        latest_output = self.panel._intrinsics_storage_base_path()
        metadata = {
            "note": "Generated by StereoWIRIS calibration calculation",
            "timestamp_s": time.time(),
            "zoom_level": zoom_level,
            "capture_zoom_level": zoom_level,
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

    def workstation_checkerboard_validation_snapshot(self) -> dict:
        """Read-only projection of the validation workflow's current state.

        The action methods above (``load_checkerboard_validation_json`` etc.)
        read/write panel widget attributes directly and return ``None``; this
        gives a payload-in/result-out caller (the navigation UI, not the
        legacy panel's own hidden widgets) something to read after any
        action, or to poll while a random-validation run is in flight.
        """

        panel = self.panel
        return {
            "root": panel.checkerboard_validation_root_edit.text(),
            "mode": str(panel.checkerboard_validation_mode_combo.currentData() or "pixhawk"),
            "json_path": panel.checkerboard_validation_json_edit.text(),
            "scan_status": panel.checkerboard_validation_scan_label.text(),
            "status": panel.checkerboard_validation_status_label.text(),
            "random_pair_count": panel.checkerboard_validation_random_pair_count_spin.value(),
            "runs": panel.checkerboard_validation_random_runs_spin.value(),
            "seed": panel.checkerboard_validation_seed_spin.value(),
            "running": panel.checkerboard_calibration_thread is not None,
            "result_available": panel.checkerboard_validation_result is not None,
            "result_mode": str(getattr(panel, "checkerboard_validation_result_mode", "") or ""),
            "result_saved": bool(getattr(panel, "checkerboard_validation_result_saved", False)),
            "summary": dict(getattr(panel, "checkerboard_validation_summary", {}) or {}),
            "run_results": list(getattr(panel, "checkerboard_validation_run_results", []) or []),
        }

    def configure_workstation_checkerboard_validation(self, payload: dict) -> dict:
        """Apply operator-supplied settings, keeping the panel widgets as the
        single owner of this state (unchanged since before the navigation UI
        existed) rather than duplicating it. Only the keys present in
        ``payload`` are touched.
        """

        panel = self.panel
        if "root" in payload:
            panel.checkerboard_validation_root_edit.setText(str(payload["root"]))
        if "mode" in payload:
            panel._set_combo_data(panel.checkerboard_validation_mode_combo, str(payload["mode"]))
        if "json_path" in payload:
            panel.checkerboard_validation_json_edit.setText(str(payload["json_path"]))
        if "random_pair_count" in payload:
            panel.checkerboard_validation_random_pair_count_spin.setValue(int(payload["random_pair_count"]))
        if "runs" in payload:
            panel.checkerboard_validation_random_runs_spin.setValue(int(payload["runs"]))
        if "seed" in payload:
            panel.checkerboard_validation_seed_spin.setValue(int(payload["seed"]))
        if "root" in payload or "mode" in payload:
            panel.scan_checkerboard_validation_files()
        return self.panel.workstation_checkerboard_validation_snapshot()

    def load_workstation_checkerboard_validation_json(self, payload: dict) -> dict:
        self.panel.configure_workstation_checkerboard_validation(payload)
        self.panel.load_checkerboard_validation_json()
        return self.panel.workstation_checkerboard_validation_snapshot()

    def run_workstation_checkerboard_validation(self, payload: dict) -> dict:
        self.panel.configure_workstation_checkerboard_validation(payload)
        self.panel.run_checkerboard_random_validation()
        return self.panel.workstation_checkerboard_validation_snapshot()

    def save_workstation_checkerboard_validation_result(self) -> dict:
        self.panel.save_checkerboard_validation_result()
        return self.panel.workstation_checkerboard_validation_snapshot()

    def apply_workstation_checkerboard_validation_result(self) -> dict:
        self.panel.apply_checkerboard_validation_result()
        return self.panel.workstation_checkerboard_validation_snapshot()

    def apply_intrinsics_to_distance(self) -> None:
        panel = self.panel
        if panel.last_calibration_result is None:
            panel.calibration_status_label.setText("Run or load calibration before applying intrinsics.")
            return
        selected_profile = panel.selected_extrinsics_profile
        if selected_profile is None and panel.last_calibration_result.extrinsics is not None:
            selected_profile = StereoExtrinsicsProfile(
                extrinsics=panel.last_calibration_result.extrinsics,
                source_zoom_level=panel.last_calibration_zoom_level,
                method="legacy_embedded_checkerboard",
                source_intrinsics_path=panel._active_intrinsics_profile_path,
                source_intrinsics_sha256=intrinsics_content_sha256(
                    panel.last_calibration_result
                ),
                profile_id="legacy_embedded_extrinsics",
            )
            panel.selected_extrinsics_profile = selected_profile
            panel.checkerboard_extrinsics_profile = selected_profile
            panel._selected_extrinsics_profile_path = (
                f"{panel._active_intrinsics_profile_path}#embedded-extrinsics"
            )
            panel._selected_extrinsics_profile_hash = selected_profile.content_sha256
        panel.active_intrinsics = compose_stereo_calibration(
            panel.last_calibration_result,
            selected_profile,
        )
        panel._active_intrinsics_source_zoom_level = panel.last_calibration_zoom_level
        panel._active_intrinsics_profile_hash = intrinsics_content_sha256(
            panel.last_calibration_result
        )
        if selected_profile is None:
            panel._active_extrinsics_profile_path = ""
            panel._active_extrinsics_profile_hash = ""
            panel._active_extrinsics_source_zoom_level = None
            panel._active_calibration_selection_mode = "intrinsics_only"
        else:
            panel._active_extrinsics_profile_path = (
                panel._selected_extrinsics_profile_path
                or f"memory:{selected_profile.profile_id or 'stereo_extrinsics'}"
            )
            panel._active_extrinsics_profile_hash = selected_profile.content_sha256
            panel._active_extrinsics_source_zoom_level = selected_profile.source_zoom_level
            panel._active_calibration_selection_mode = independent_zoom_selection_mode(
                panel.last_calibration_zoom_level,
                selected_profile.source_zoom_level,
            )
            panel._set_combo_data(panel.distance_mode_combo, "checkerboard")
        left_fx = panel.active_intrinsics.left.camera_matrix[0][0]
        right_fx = panel.active_intrinsics.right.camera_matrix[0][0]
        panel.focal_spin.setValue((left_fx + right_fx) / 2.0)
        panel.manual_focal_checkbox.setChecked(False)
        selection_label = {
            "manual_cross_zoom": "cross-zoom/manual",
            "manual_same_zoom": "same-zoom/manual",
            "manual_independent_zoom": "independent/manual; source zoom unknown",
        }.get(panel._active_calibration_selection_mode, "")
        detail = f"K,D zoom={panel.last_calibration_zoom_level}; " + (
            f"R,t zoom={selected_profile.source_zoom_level}; "
            f"{selection_label}, validation not gated"
            if selected_profile is not None
            else "R,t not selected"
        )
        panel.calibration_status_label.setText(f"Applied calibration bundle: {detail}")
        panel._refresh_camera_parameters_tab()
        panel._refresh_distance()
        panel._refresh_optical_alignment_view()
        panel._refresh_operation_header()

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
        zoom_snapshot = self._capture_zoom_snapshot()
        self.panel.last_capture_zoom_level = zoom_snapshot.get("capture_zoom_level")
        metadata_payload = {
            "capture_origin": str(capture_origin),
            "pair_time_basis": pair_time_basis,
            "pair_delta_s": pair_delta_s,
            "pair_max_delta_s": CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S,
            "pair_timing_status": pair_timing_detail,
            "left_rtcp_frame_ntp_unix_s": (metadata_by_camera.get("left") or {}).get("rtcp_frame_ntp_unix_s"),
            "right_rtcp_frame_ntp_unix_s": (metadata_by_camera.get("right") or {}).get("rtcp_frame_ntp_unix_s"),
            "frame_contract": "OpenCV image pixels; stereo timing uses RTCP/NTP camera timestamp before saving a left/right pair",
            **zoom_snapshot,
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

    def capture_calibration_images_wiris_ftp(self) -> bool:
        """Capture a left/right pair via a WIRIS HR still + FTP download.

        A second source alongside ``capture_calibration_images``'s live-RTSP
        path, not a replacement: once the still is downloaded, it is handed
        to that exact same method (``frames=..., capture_origin="ftp"``), so
        save/naming/checkerboard-validation logic is entirely unchanged.
        """

        if self.panel.calibration_wiris_ftp_thread is not None:
            self.panel.calibration_status_label.setText(
                "A WIRIS FTP calibration capture is already in progress."
            )
            return False
        if self.panel.calibration_wiris_ftp_oz_thread is not None:
            self.panel.calibration_status_label.setText(
                "A WIRIS FTP (OZ) calibration capture is already in progress."
            )
            return False
        if self.panel.checkerboard_calibration_thread is not None:
            self.panel.calibration_status_label.setText("A calibration operation is already running.")
            return False
        settings = getattr(self.panel, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            self.panel.calibration_status_label.setText(
                "WIRIS FTP capture requires \"Enable FTP download\" in Settings."
            )
            return False
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            self.panel.calibration_status_label.setText(
                "WIRIS FTP capture failed: both WIRIS control hosts are required."
            )
            return False
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        capture_id = f"calib-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        kwargs = self.panel._wiris_ftp_worker_kwargs(output_dir=root)
        thread = QtCore.QThread(self.panel)
        worker = WirisPnPPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            capture_channel="calib",
            completion_timeout_s=10.0,
            **kwargs,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_ftp_capture_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_ftp_capture_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_ftp_capture_worker)
        self.panel.calibration_wiris_ftp_thread = thread
        self.panel.calibration_wiris_ftp_worker = worker
        self.panel.capture_calibration_button.setEnabled(False)
        self.panel.run_calibration_button.setEnabled(False)
        self.panel.calibration_status_label.setText(
            f"WIRIS FTP Capture {capture_id}: 좌/우 카메라 내부 저장 중"
        )
        thread.start()
        return True

    def _handle_calibration_wiris_ftp_capture_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        cameras = record.get("cameras")
        cameras = cameras if isinstance(cameras, dict) else {}
        capture_id = str(record.get("capture_id") or "")
        failures = [
            f"{side}: {item.get('error') or item.get('state', 'failed')}"
            for side, item in cameras.items()
            if item.get("state") != "completed" or not item.get("ftp_downloaded")
        ]
        if set(cameras) != {"left", "right"} or failures:
            self._handle_calibration_wiris_ftp_capture_failed(
                "; ".join(failures) or "camera result is missing left/right entries"
            )
            return
        frames: dict[str, QtGui.QImage | None] = {}
        for side, item in cameras.items():
            path = str(item.get("ftp_local_path") or "")
            image = QtGui.QImage(path) if path else QtGui.QImage()
            if image.isNull():
                self._handle_calibration_wiris_ftp_capture_failed(
                    f"{side}: downloaded file could not be decoded as an image ({path or 'no path'})"
                )
                return
            frames[side] = image
        self.panel._log(f"[캘리브레이션] WIRIS FTP Capture {capture_id}: 다운로드 완료, 캘리브레이션 저장 시작")
        self.panel.capture_calibration_images(frames=frames, capture_origin="ftp")

    def _handle_calibration_wiris_ftp_capture_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] WIRIS FTP Capture 실패: {error}")
        self.panel.calibration_status_label.setText(f"WIRIS FTP Capture failed: {error}")
        self.panel.capture_calibration_button.setEnabled(True)
        self.panel.run_calibration_button.setEnabled(True)

    def _clear_calibration_wiris_ftp_capture_worker(self) -> None:
        self.panel.calibration_wiris_ftp_thread = None
        self.panel.calibration_wiris_ftp_worker = None

    def capture_calibration_images_wiris_ftp_oz(self) -> bool:
        """Same as ``capture_calibration_images_wiris_ftp``, on the OZ channel.

        The WIRIS normal visible operating-zoom still (``SIVI TRUE``) instead
        of the HR still (``SIHR TRUE``) -- lower resolution than HR, but a
        faster capture with no camera reboot risk (unlike changing HR
        resolution via ``SHRR``). Downloads land in a separate
        ``calib_oz_capture`` subfolder so they never mix with HR downloads,
        and feed the exact same ``capture_calibration_images`` save path.
        """

        if self.panel.calibration_wiris_ftp_oz_thread is not None:
            self.panel.calibration_status_label.setText(
                "A WIRIS FTP (OZ) calibration capture is already in progress."
            )
            return False
        if self.panel.calibration_wiris_ftp_thread is not None:
            self.panel.calibration_status_label.setText(
                "A WIRIS FTP calibration capture is already in progress."
            )
            return False
        if self.panel.checkerboard_calibration_thread is not None:
            self.panel.calibration_status_label.setText("A calibration operation is already running.")
            return False
        settings = getattr(self.panel, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            self.panel.calibration_status_label.setText(
                "WIRIS FTP capture requires \"Enable FTP download\" in Settings."
            )
            return False
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            self.panel.calibration_status_label.setText(
                "WIRIS FTP capture failed: both WIRIS control hosts are required."
            )
            return False
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        capture_id = f"calib-oz-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        kwargs = self.panel._wiris_ftp_worker_kwargs(output_dir=root)
        thread = QtCore.QThread(self.panel)
        worker = WirisPnPPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            image_enable_command="SIVI TRUE",
            capture_channel="calib_oz",
            completion_timeout_s=10.0,
            **kwargs,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_ftp_oz_capture_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_ftp_oz_capture_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_ftp_oz_capture_worker)
        self.panel.calibration_wiris_ftp_oz_thread = thread
        self.panel.calibration_wiris_ftp_oz_worker = worker
        self.panel.capture_calibration_button.setEnabled(False)
        self.panel.run_calibration_button.setEnabled(False)
        self.panel.calibration_status_label.setText(
            f"WIRIS FTP (OZ) Capture {capture_id}: 좌/우 카메라 내부 저장 중"
        )
        thread.start()
        return True

    def _handle_calibration_wiris_ftp_oz_capture_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        cameras = record.get("cameras")
        cameras = cameras if isinstance(cameras, dict) else {}
        capture_id = str(record.get("capture_id") or "")
        failures = [
            f"{side}: {item.get('error') or item.get('state', 'failed')}"
            for side, item in cameras.items()
            if item.get("state") != "completed" or not item.get("ftp_downloaded")
        ]
        if set(cameras) != {"left", "right"} or failures:
            self._handle_calibration_wiris_ftp_oz_capture_failed(
                "; ".join(failures) or "camera result is missing left/right entries"
            )
            return
        frames: dict[str, QtGui.QImage | None] = {}
        for side, item in cameras.items():
            path = str(item.get("ftp_local_path") or "")
            image = QtGui.QImage(path) if path else QtGui.QImage()
            if image.isNull():
                self._handle_calibration_wiris_ftp_oz_capture_failed(
                    f"{side}: downloaded file could not be decoded as an image ({path or 'no path'})"
                )
                return
            frames[side] = image
        self.panel._log(f"[캘리브레이션] WIRIS FTP (OZ) Capture {capture_id}: 다운로드 완료, 캘리브레이션 저장 시작")
        self.panel.capture_calibration_images(frames=frames, capture_origin="ftp_oz")

    def _handle_calibration_wiris_ftp_oz_capture_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] WIRIS FTP (OZ) Capture 실패: {error}")
        self.panel.calibration_status_label.setText(f"WIRIS FTP (OZ) Capture failed: {error}")
        self.panel.capture_calibration_button.setEnabled(True)
        self.panel.run_calibration_button.setEnabled(True)

    def _clear_calibration_wiris_ftp_oz_capture_worker(self) -> None:
        self.panel.calibration_wiris_ftp_oz_thread = None
        self.panel.calibration_wiris_ftp_oz_worker = None

    # -- OZ capture-only + FTP sync -------------------------------------
    #
    # "Capture FTP OZ" above bundles a still trigger with an immediate FTP
    # download; every capture pays the full FTP round-trip before the next
    # one can start. These three pieces decouple that: capture repeatedly
    # (fast, camera-local only), then sync everything new in one pass, then
    # let the operator pick which synced pairs feed the calculation.
    #
    # Extrinsics need proof that a left/right pair came from the *same*
    # trigger. The two camera clocks can differ by hours, so remote
    # timestamps can't establish that once several stills have piled up
    # un-downloaded. Instead a local ledger (``calib_oz_capture/ledger.json``)
    # records each trigger round and whether *both* sides reported
    # ``stored_on_camera`` -- only such "synchronized" rounds are ever
    # eligible for sync/pairing, and syncing matches each camera's new
    # remote files to ledger rounds strictly in that camera's own upload
    # order (MDTM), never by cross-camera timestamp.

    def _calib_oz_ledger_path(self) -> Path:
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        return root / "calib_oz_capture" / "ledger.json"

    def _load_calib_oz_ledger(self) -> dict[str, object]:
        empty: dict[str, object] = {"rounds": [], "consumed_remote_names": {"left": [], "right": []}}
        path = self.panel._calib_oz_ledger_path()
        if not path.exists():
            return empty
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return empty
        if not isinstance(data, dict):
            return empty
        data.setdefault("rounds", [])
        data.setdefault("consumed_remote_names", {"left": [], "right": []})
        return data

    def _save_calib_oz_ledger(self, ledger: dict[str, object]) -> None:
        path = self.panel._calib_oz_ledger_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(ledger, indent=2), encoding="utf-8")

    def capture_calibration_images_wiris_oz_capture_only(self) -> bool:
        """Trigger an OZ still on both cameras and leave it on camera storage.

        No FTP download here -- see ``sync_calibration_images_wiris_oz_ftp``.
        Meant to be pressed repeatedly while watching the live view for board
        framing, accumulating stills quickly without paying an FTP round-trip
        per shot.
        """

        busy_threads = (
            self.panel.calibration_wiris_oz_format_thread,
            self.panel.calibration_wiris_oz_gilc_thread,
            self.panel.calibration_wiris_oz_capture_thread,
            self.panel.calibration_wiris_oz_sync_thread,
            self.panel.calibration_wiris_ftp_thread,
            self.panel.calibration_wiris_ftp_oz_thread,
            self.panel.checkerboard_calibration_thread,
        )
        if any(thread is not None for thread in busy_threads):
            self.panel.calibration_status_label.setText(
                "A WIRIS OZ capture/sync/calibration operation is already in progress."
            )
            return False
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            self.panel.calibration_status_label.setText(
                "OZ capture failed: both WIRIS control hosts are required."
            )
            return False
        # Force a known, single-file-per-round format state before every
        # trigger -- CPTR emits one file per enabled SIxx toggle, and the
        # two cameras were found with different toggles on (left had
        # radiometric on, right had HR on). Best-effort: proceed to capture
        # either way, since a failed format command shouldn't block the shot,
        # it'll just be caught later by the sync's per-name ``-visible.jpg``
        # filter and count check.
        self.panel.calibration_status_label.setText("OZ Capture: 이미지 포맷 강제 설정 중")
        thread = QtCore.QThread(self.panel)
        worker = WirisImageFormatWorker(
            target_infos,
            commands=("SIRJ FALSE", "SISR FALSE", "SIHR FALSE", "SIVI TRUE"),
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_oz_format_preset_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_oz_format_preset_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_oz_format_worker)
        self.panel.calibration_wiris_oz_format_thread = thread
        self.panel.calibration_wiris_oz_format_worker = worker
        self._pending_oz_capture_target_infos = target_infos
        thread.start()
        return True

    def _handle_calibration_wiris_oz_format_preset_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        for label, responses in record.items():
            self.panel._log(f"[캘리브레이션] OZ 포맷 강제 설정 {label}: {responses}")
        self._start_calibration_wiris_oz_capture()

    def _handle_calibration_wiris_oz_format_preset_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] OZ 포맷 강제 설정 실패 (촬영은 계속 진행): {error}")
        self._start_calibration_wiris_oz_capture()

    def _clear_calibration_wiris_oz_format_worker(self) -> None:
        self.panel.calibration_wiris_oz_format_thread = None
        self.panel.calibration_wiris_oz_format_worker = None

    def _start_calibration_wiris_oz_capture(self) -> None:
        target_infos = getattr(self, "_pending_oz_capture_target_infos", ())
        self._pending_oz_capture_target_infos = ()
        if len(target_infos) != 2:
            return
        capture_id = f"calib-ozcap-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        thread = QtCore.QThread(self.panel)
        worker = WirisOZPhotoCaptureWorker(target_infos, capture_id=capture_id, completion_timeout_s=10.0)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_oz_capture_only_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_oz_capture_only_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_oz_capture_only_worker)
        self.panel.calibration_wiris_oz_capture_thread = thread
        self.panel.calibration_wiris_oz_capture_worker = worker
        self.panel.calibration_status_label.setText(f"OZ Capture {capture_id}: 좌/우 카메라 내부 저장 중")
        thread.start()

    def _handle_calibration_wiris_oz_capture_only_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        cameras = record.get("cameras")
        cameras = cameras if isinstance(cameras, dict) else {}
        capture_id = str(record.get("capture_id") or "")
        left_ok = bool((cameras.get("left") or {}).get("stored_on_camera"))
        right_ok = bool((cameras.get("right") or {}).get("stored_on_camera"))
        synchronized = left_ok and right_ok
        ledger = self.panel._load_calib_oz_ledger()
        rounds = ledger["rounds"]
        rounds.append(
            {
                "capture_id": capture_id,
                "requested_at_unix_ns": time.time_ns(),
                "left": {"stored_on_camera": left_ok, "state": (cameras.get("left") or {}).get("state")},
                "right": {"stored_on_camera": right_ok, "state": (cameras.get("right") or {}).get("state")},
                "synchronized": synchronized,
                "downloaded": False,
                "left_remote_name": None,
                "right_remote_name": None,
            }
        )
        self.panel._save_calib_oz_ledger(ledger)
        pending = sum(1 for row in rounds if row.get("synchronized") and not row.get("downloaded"))
        if not synchronized:
            self.panel._log(
                f"[캘리브레이션] OZ Capture {capture_id}: 좌우 중 한쪽만 성공 -- 페어링 대상에서 제외 "
                f"(left={left_ok}, right={right_ok})"
            )
        self.panel.calibration_status_label.setText(
            f"OZ Capture {capture_id}: 완료 (동기화 대기 {pending}쌍)"
        )

    def _handle_calibration_wiris_oz_capture_only_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] OZ Capture 실패: {error}")
        self.panel.calibration_status_label.setText(f"OZ Capture failed: {error}")

    def _clear_calibration_wiris_oz_capture_only_worker(self) -> None:
        self.panel.calibration_wiris_oz_capture_thread = None
        self.panel.calibration_wiris_oz_capture_worker = None

    #: GILC's storage-location names, mapped to their FTP root folder name.
    IMAGE_LOCATION_FTP_ROOTS = {"SSD": "ssd", "SD_CARD": "sdcard", "FLASH_DRIVE": "flashd"}

    def sync_calibration_images_wiris_oz_ftp(self) -> bool:
        """Download every un-synced, both-sides-successful OZ round via FTP.

        WIRIS Enterprise has no fixed image directory -- stills land under
        /<GILC storage root>/<session folder>/img/, and a new session folder
        appears every time Ethernet Mode (re)starts or the camera reboots.
        The storage root is queried fresh via GILC before every sync (a
        camera-side ``SILC`` change should be picked up immediately, not
        cached); the sync worker itself walks every session folder under it.
        """

        busy_threads = (
            self.panel.calibration_wiris_oz_format_thread,
            self.panel.calibration_wiris_oz_gilc_thread,
            self.panel.calibration_wiris_oz_capture_thread,
            self.panel.calibration_wiris_oz_sync_thread,
            self.panel.calibration_wiris_ftp_thread,
            self.panel.calibration_wiris_ftp_oz_thread,
            self.panel.checkerboard_calibration_thread,
        )
        if any(thread is not None for thread in busy_threads):
            self.panel.calibration_status_label.setText(
                "A WIRIS OZ capture/sync/calibration operation is already in progress."
            )
            return False
        settings = getattr(self.panel, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            self.panel.calibration_status_label.setText(
                "WIRIS FTP sync requires \"Enable FTP download\" in Settings."
            )
            return False
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            self.panel.calibration_status_label.setText(
                "WIRIS FTP sync failed: both WIRIS control hosts are required."
            )
            return False
        ledger = self.panel._load_calib_oz_ledger()
        rounds = ledger["rounds"]
        pending_rounds = [row for row in rounds if row.get("synchronized") and not row.get("downloaded")]
        if not pending_rounds:
            self.panel.calibration_status_label.setText("동기화할 새 OZ 회차가 없습니다.")
            return False
        self.panel.calibration_status_label.setText("WIRIS FTP 동기화: 저장소 위치(GILC) 확인 중")
        thread = QtCore.QThread(self.panel)
        worker = WirisImageFormatWorker(target_infos, commands=("GILC",))
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_oz_gilc_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_oz_gilc_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_oz_gilc_worker)
        self.panel.calibration_wiris_oz_gilc_thread = thread
        self.panel.calibration_wiris_oz_gilc_worker = worker
        self._pending_ftp_oz_sync_target_infos = target_infos
        self._pending_ftp_oz_sync_rounds = pending_rounds
        thread.start()
        return True

    def _handle_calibration_wiris_oz_gilc_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        target_infos = getattr(self, "_pending_ftp_oz_sync_target_infos", ())
        pending_rounds = getattr(self, "_pending_ftp_oz_sync_rounds", [])
        self._pending_ftp_oz_sync_target_infos = ()
        self._pending_ftp_oz_sync_rounds = []
        storage_root_by_label: dict[str, str] = {}
        for label, responses in record.items():
            location = str((responses or {}).get("GILC", "")).strip().upper()
            root_name = self.IMAGE_LOCATION_FTP_ROOTS.get(location)
            if root_name is None:
                self.panel._log(
                    f"[캘리브레이션] WIRIS FTP 동기화 실패: {label} GILC 응답을 알 수 없습니다 ({location!r})."
                )
                self.panel.calibration_status_label.setText(
                    "WIRIS FTP sync failed: 저장소 위치(GILC)를 확인하지 못했습니다."
                )
                return
            storage_root_by_label[label] = root_name
        if len(storage_root_by_label) != 2:
            self.panel.calibration_status_label.setText(
                "WIRIS FTP sync failed: 좌우 저장소 위치를 모두 확인하지 못했습니다."
            )
            return
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        output_dir = root / "calib_oz_capture"
        kwargs = self.panel._wiris_ftp_worker_kwargs(output_dir=output_dir)
        ledger = self.panel._load_calib_oz_ledger()
        fetch_counts = {"left": len(pending_rounds), "right": len(pending_rounds)}
        thread = QtCore.QThread(self.panel)
        worker = WirisOzFtpSyncWorker(
            target_infos,
            ftp_settings_by_label=kwargs["ftp_settings_by_label"],
            storage_root_by_label=storage_root_by_label,
            ftp_output_dir=output_dir,
            fetch_counts=fetch_counts,
            downloaded_remote_names=ledger.get("consumed_remote_names") or {},
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_oz_sync_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_oz_sync_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_oz_sync_worker)
        self.panel.calibration_wiris_oz_sync_thread = thread
        self.panel.calibration_wiris_oz_sync_worker = worker
        self._pending_ftp_oz_sync_round_ids = [str(row.get("capture_id")) for row in pending_rounds]
        self.panel.calibration_status_label.setText(
            f"WIRIS FTP 동기화: {len(pending_rounds)}쌍 다운로드 중"
        )
        thread.start()

    def _handle_calibration_wiris_oz_gilc_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] WIRIS FTP 동기화: GILC 조회 실패 - {error}")
        self.panel.calibration_status_label.setText(f"WIRIS FTP sync failed: GILC query failed ({error})")
        self._pending_ftp_oz_sync_target_infos = ()
        self._pending_ftp_oz_sync_rounds = []

    def _clear_calibration_wiris_oz_gilc_worker(self) -> None:
        self.panel.calibration_wiris_oz_gilc_thread = None
        self.panel.calibration_wiris_oz_gilc_worker = None

    def _handle_calibration_wiris_oz_sync_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        left = record.get("left") if isinstance(record.get("left"), dict) else {}
        right = record.get("right") if isinstance(record.get("right"), dict) else {}
        for side, item in (("left", left), ("right", right)):
            if item.get("error"):
                self.panel._log(f"[캘리브레이션] WIRIS FTP 동기화: {side} 오류 - {item['error']}")
        left_items = list(left.get("items") or ())
        right_items = list(right.get("items") or ())
        round_ids = list(getattr(self, "_pending_ftp_oz_sync_round_ids", ()))
        self._pending_ftp_oz_sync_round_ids = []

        # Hard-fail the *whole* batch on any count mismatch or unreadable
        # file, rather than zipping via min(len(left), len(right)). A
        # mismatch anywhere except the tail (a round missing on one side
        # only, or a stray pre-existing remote file mixed in) would
        # otherwise get silently zipped against the wrong round -- exactly
        # the cross-camera mixup the ledger exists to prevent. Nothing here
        # is deleted from the camera; a failed batch can simply be retried
        # once the remote directory / ledger state is sorted out.
        left_invalid = [str(item.get("name")) for item in left_items if not item.get("valid")]
        right_invalid = [str(item.get("name")) for item in right_items if not item.get("valid")]
        if len(left_items) != len(round_ids) or len(right_items) != len(round_ids) or left_invalid or right_invalid:
            self.panel._log(
                "[캘리브레이션] WIRIS FTP 동기화 실패: 좌우 회차 수/유효성이 요청과 일치하지 않아 "
                f"배치 전체를 건너뜁니다 (요청={len(round_ids)}, left={len(left_items)}개"
                f"(무효 {left_invalid}), right={len(right_items)}개(무효 {right_invalid})). "
                "카메라 원본은 삭제하지 않았습니다 -- 다른 캡처 채널이 같은 원격 디렉토리에 "
                "파일을 남기지 않았는지 확인 후 다시 시도하세요."
            )
            self.panel.calibration_status_label.setText(
                "WIRIS FTP sync failed: 좌우 회차가 정확히 매칭되지 않아 중단했습니다 (로그 참고)."
            )
            return
        ledger = self.panel._load_calib_oz_ledger()
        rounds_by_id = {str(row.get("capture_id")): row for row in ledger["rounds"]}
        consumed = ledger.setdefault("consumed_remote_names", {"left": [], "right": []})
        queue: list[tuple[str, dict[str, object]]] = []
        delete_names: dict[str, list[str]] = {"left": [], "right": []}
        for index, capture_id in enumerate(round_ids):
            left_name, right_name = str(left_items[index]["name"]), str(right_items[index]["name"])
            row = rounds_by_id.get(capture_id)
            if row is not None:
                row["downloaded"] = True
                row["left_remote_name"] = left_name
                row["right_remote_name"] = right_name
            consumed.setdefault("left", []).append(left_name)
            consumed.setdefault("right", []).append(right_name)
            delete_names["left"].append(left_name)
            delete_names["right"].append(right_name)
            queue.append(
                (
                    capture_id,
                    {
                        "left": QtGui.QImage(str(left_items[index]["path"])),
                        "right": QtGui.QImage(str(right_items[index]["path"])),
                    },
                )
            )
        self.panel._save_calib_oz_ledger(ledger)
        self._pending_ftp_oz_sync_frames = queue
        self._ftp_oz_sync_saved_count = 0
        self.panel._process_next_ftp_oz_sync_pair()
        # Only ever asked for once every file in the batch downloaded clean
        # and every round paired 1:1 -- see the hard-fail check above.
        self.panel._start_calibration_wiris_oz_delete(delete_names)

    def _handle_calibration_wiris_oz_sync_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] WIRIS FTP 동기화 실패: {error}")
        self.panel.calibration_status_label.setText(f"WIRIS FTP sync failed: {error}")
        self._pending_ftp_oz_sync_round_ids = []

    def _clear_calibration_wiris_oz_sync_worker(self) -> None:
        self.panel.calibration_wiris_oz_sync_thread = None
        self.panel.calibration_wiris_oz_sync_worker = None

    def _start_calibration_wiris_oz_delete(self, names_by_label: dict[str, list[str]]) -> None:
        if not any(names_by_label.values()):
            return
        if self.panel.calibration_wiris_oz_delete_thread is not None:
            self.panel._log(
                "[캘리브레이션] WIRIS FTP 원본 삭제: 이전 삭제 작업이 아직 진행 중이라 이번 정리는 건너뜁니다 "
                "(로컬 사본은 이미 확보됨, 카메라에 파일이 남아있습니다)."
            )
            return
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            return
        kwargs = self.panel._wiris_ftp_worker_kwargs()
        thread = QtCore.QThread(self.panel)
        worker = WirisFtpBatchDeleteWorker(
            target_infos,
            ftp_settings_by_label=kwargs["ftp_settings_by_label"],
            names_by_label=names_by_label,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_oz_delete_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_oz_delete_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_oz_delete_worker)
        self.panel.calibration_wiris_oz_delete_thread = thread
        self.panel.calibration_wiris_oz_delete_worker = worker
        thread.start()

    def _handle_calibration_wiris_oz_delete_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        for side in ("left", "right"):
            item = record.get(side) if isinstance(record.get(side), dict) else {}
            if item.get("error"):
                self.panel._log(f"[캘리브레이션] WIRIS FTP 원본 삭제: {side} 오류 - {item['error']}")
            for name, error in (item.get("delete_errors") or {}).items():
                self.panel._log(
                    f"[캘리브레이션] WIRIS FTP 원본 삭제 실패 {side}/{name}: {error} "
                    "(로컬 사본은 이미 확보됨, 카메라에 파일이 남아있습니다)"
                )
            deleted = item.get("deleted_names") or []
            if deleted:
                self.panel._log(f"[캘리브레이션] WIRIS FTP 원본 삭제: {side} {len(deleted)}개 삭제됨")

    def _handle_calibration_wiris_oz_delete_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] WIRIS FTP 원본 삭제 실패: {error} (로컬 사본은 이미 확보됨)")

    def _clear_calibration_wiris_oz_delete_worker(self) -> None:
        self.panel.calibration_wiris_oz_delete_thread = None
        self.panel.calibration_wiris_oz_delete_worker = None

    def _process_next_ftp_oz_sync_pair(self) -> None:
        queue = getattr(self, "_pending_ftp_oz_sync_frames", None)
        if not queue:
            saved = getattr(self, "_ftp_oz_sync_saved_count", 0)
            self.panel._log(f"[캘리브레이션] WIRIS FTP 동기화 완료: {saved}쌍 저장")
            self.panel.calibration_status_label.setText(f"WIRIS FTP 동기화 완료: {saved}쌍 저장")
            return
        capture_id, frames = queue.pop(0)
        started = self.panel.capture_calibration_images(frames=frames, capture_origin="ftp_oz_sync")
        if not started:
            QtCore.QTimer.singleShot(200, self.panel._process_next_ftp_oz_sync_pair)
            return
        self._ftp_oz_sync_saved_count = getattr(self, "_ftp_oz_sync_saved_count", 0) + 1
        thread = self.panel.checkerboard_calibration_thread
        if thread is not None:
            thread.finished.connect(self.panel._process_next_ftp_oz_sync_pair)
        else:
            QtCore.QTimer.singleShot(200, self.panel._process_next_ftp_oz_sync_pair)

    # -- Manual calculation-pair selection --------------------------------

    def _calibration_pair_selection_path(self) -> Path:
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        return root / "calibration_pair_selection.json"

    def _load_calibration_pair_selection(self) -> dict[str, object]:
        empty: dict[str, object] = {"manual_mode": False, "included_keys": []}
        path = self.panel._calibration_pair_selection_path()
        if not path.exists():
            return empty
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return empty
        if not isinstance(data, dict):
            return empty
        return {
            "manual_mode": bool(data.get("manual_mode", False)),
            "included_keys": [str(key) for key in data.get("included_keys", []) or []],
        }

    def _choose_calibration_pairs(
        self,
        pairs: list[tuple[str, Path, Path]],
    ) -> list[tuple[str, Path, Path]]:
        """Manual picker override of ``_select_calibration_pairs``.

        Falls back to the existing even-distribution auto-selection unless
        the operator has switched on manual selection in the picker (which
        writes ``calibration_pair_selection.json`` -- the picker lives on the
        visible workspace panel, calculation runs on this hidden one, so a
        JSON sidecar is the handoff instead of a cross-panel command).
        """

        selection = self.panel._load_calibration_pair_selection()
        if not selection.get("manual_mode"):
            return self._select_calibration_pairs(pairs)
        included = set(selection.get("included_keys") or ())
        return [pair for pair in pairs if pair[0] in included]

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
        selected_pairs = self._choose_calibration_pairs(pairs)
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
        capture_zoom_level, zoom_detail = self._capture_zoom_level_for_paths(
            root,
            [*left_paths, *right_paths],
        )
        if capture_zoom_level is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"계산 실패: {zoom_detail}. 촬영 당시 양쪽 GZVV readback이 같은지 확인하고 다시 촬영하세요.",
                "bad",
            )
            self.panel._log(f"[캘리브레이션] 계산 중단: {zoom_detail}")
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
            capture_zoom_level=capture_zoom_level,
            left_count=len(left_paths),
            right_count=len(right_paths): self.panel._handle_intrinsic_calibration_completed(
                result,
                mode=mode,
                root=root,
                pair_count=pair_count,
                min_images=min_images,
                capture_zoom_level=capture_zoom_level,
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
        selected_pairs = self._choose_calibration_pairs(pairs)
        if len(selected_pairs) < 3:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"R,t only 계산 실패: matching stereo pair={len(selected_pairs)}, need>=3",
                "bad",
            )
            return
        left_paths = [left for _key, left, _right in selected_pairs]
        right_paths = [right for _key, _left, right in selected_pairs]
        source_zoom, zoom_detail = self._capture_zoom_level_for_paths(
            root,
            [*left_paths, *right_paths],
        )
        if source_zoom is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"R,t only 계산 실패: {zoom_detail}. 촬영 당시 양쪽 GZVV readback이 같은지 확인하고 다시 촬영하세요.",
                "bad",
            )
            self.panel._log(f"[캘리브레이션] R,t 계산 중단: {zoom_detail}")
            return
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
        # Kept separate from ``selected_extrinsics_profile`` (which a later
        # PnP apply would overwrite) so the geometry-comparison display can
        # always show the raw checkerboard stereoCalibrate result on its own.
        self.panel.checkerboard_extrinsics_profile = profile
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
        # Route into the geometry-comparison slot matching this profile's
        # origin so a loaded file doesn't silently blend checkerboard and
        # PnP provenance in the comparison display.
        if str(profile.method or "").startswith("optical_pose/"):
            self.panel.optical_pose_extrinsics_profile = profile
        else:
            self.panel.checkerboard_extrinsics_profile = profile
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
        capture_zoom_level: int,
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
                "capture_zoom_level": capture_zoom_level,
                "left_image_count": left_count,
                "right_image_count": right_count,
                "stereo_pair_count": pair_count,
                "min_images": min_images,
            },
            text=text,
            zoom_level=capture_zoom_level,
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
