"""Offline validation comparisons in native pixels and left-camera optical RDF.

Reference calibration is offline validation truth, never a runtime input to a
camera or measurement correction. Original formulas and pass/fail policy remain.
"""
from __future__ import annotations
import math
from pathlib import Path
from dataclasses import replace
from Workstation.workstation_core.calibration import StereoIntrinsicsResult


VALIDATION_FIELDS = ('checkerboard_validation_result', 'checkerboard_validation_result_mode', 'checkerboard_validation_summary', 'checkerboard_validation_run_results', 'checkerboard_validation_result_saved', 'last_calibration_result', 'calibration_quality', 'last_calibration_quality', 'checkerboard_calibration_selection_snapshot', 'calibration_image_diagnostics', 'calibration_diagnostics_root', 'last_calibration_zoom_level', '_candidate_intrinsics_profile_path', 'calibration_progress')


class CalibrationValidation:
    def __init__(self):
        self.checkerboard_validation_result = None
        self.checkerboard_validation_result_mode = "pixhawk"
        self.checkerboard_validation_summary = {}
        self.checkerboard_validation_run_results = []
        self.checkerboard_validation_result_saved = False
        self.last_calibration_result = None
        self.calibration_quality = None
        self.last_calibration_quality = None
        self.checkerboard_calibration_selection_snapshot = None
        self.calibration_image_diagnostics = ()
        self.calibration_diagnostics_root = ""
        self.last_calibration_zoom_level = None
        self._candidate_intrinsics_profile_path = ""
        self.calibration_progress = {"phase": "idle", "processed": 0, "total": 0, "percent": 0.0, "message": "대기"}

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
        baseline_m = self._checkerboard_result_baseline_m(result)
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
        quality = result.calibration_quality
        if isinstance(quality, dict):
            lines.extend(
                (
                    f"Calibration quality: {quality.get('state', 'UNVERIFIED')}",
                    "Quality reasons: "
                    + ", ".join(str(code) for code in quality.get("reason_codes", ()) or ())
                    if quality.get("reason_codes")
                    else "Quality reasons: none",
                    f"Effective poses: {quality.get('effective_pose_count', '--')}",
                    f"Principal offsets: {quality.get('principal_point_offsets_px', '--')}",
                    f"Subset stability: {quality.get('subset_stability', '--')}",
                )
            )
        return "\n".join(line for line in lines if line)


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


    def _checkerboard_validation_result_row(
        self,
        name: str,
        result: StereoIntrinsicsResult,
        reference: StereoIntrinsicsResult,
        pairs: int,
    ) -> str:
        baseline = self._checkerboard_result_baseline_m(result)
        reference_baseline = self._checkerboard_result_baseline_m(reference)
        baseline_text = "n/a" if baseline is None else f"{baseline:.6f}"
        baseline_delta_text = (
            "n/a"
            if baseline is None or reference_baseline is None
            else f"{(baseline - reference_baseline) * 1000.0:+.3f}"
        )
        rotation_delta = self._rotation_delta_deg_from_matrices(
            None if reference.extrinsics is None else reference.extrinsics.rotation_left_to_right,
            None if result.extrinsics is None else result.extrinsics.rotation_left_to_right,
        )
        rotation_text = "n/a" if rotation_delta is None else f"{rotation_delta:.6f}"
        stereo_error = "n/a" if result.extrinsics is None else f"{result.extrinsics.reprojection_error_px:.3f}"
        left_delta, right_delta = self._checkerboard_intrinsic_delta_px(result, reference)
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
        baseline = self._checkerboard_result_baseline_m(result)
        reference_baseline = self._checkerboard_result_baseline_m(reference)
        rotation_delta = self._rotation_delta_deg_from_matrices(
            None if reference.extrinsics is None else reference.extrinsics.rotation_left_to_right,
            None if result.extrinsics is None else result.extrinsics.rotation_left_to_right,
        )
        left_delta, right_delta = self._checkerboard_intrinsic_delta_px(result, reference)
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



    def accept_full_result(
        self,
        result: StereoIntrinsicsResult,
        *,
        mode: str,
        summary: dict[str, object],
        root: Path,
        publish_diagnostics=lambda _rows: None,
        zoom_level: int | None = None,
        selection_snapshot: dict[str, object] | None = None,
    ) -> StereoIntrinsicsResult:
        quality_snapshot = (
            dict(result.calibration_quality)
            if isinstance(result.calibration_quality, dict)
            else {
                "schema": "workstation-camera-calibration-quality-v1",
                "state": "UNVERIFIED",
                "source": "legacy/no quality evidence",
                "reason_codes": ["quality_snapshot_unavailable"],
                "reasons": ["This result predates source-backed calibration quality checks."],
            }
        )
        if result.calibration_quality is None:
            result = replace(result, calibration_quality=quality_snapshot)
        self.calibration_quality = quality_snapshot
        self.last_calibration_quality = quality_snapshot
        self.checkerboard_validation_result = result
        self.checkerboard_validation_result_mode = mode
        self.checkerboard_validation_summary = {
            **dict(summary),
            "calibration_quality": quality_snapshot,
        }
        extrinsics = getattr(result, "extrinsics", None)
        pair_count = int(
            summary.get("stereo_pair_count")
            or getattr(extrinsics, "pair_count", 0)
            or 0
        )
        self.checkerboard_validation_run_results = [
            self._checkerboard_validation_result_data(
                "Full",
                result,
                result,
                pair_count,
            )
        ]
        self.checkerboard_validation_result_saved = False
        self.last_calibration_result = result
        if selection_snapshot is None:
            self.checkerboard_calibration_selection_snapshot = None
        else:
            effective_selection = dict(selection_snapshot)
            effective_selection["reference_result"] = result
            self.checkerboard_calibration_selection_snapshot = effective_selection
        self.calibration_image_diagnostics = tuple(result.image_diagnostics)
        self.calibration_diagnostics_root = str(
            root
        )
        publish_diagnostics(tuple(result.image_diagnostics))
        if zoom_level is not None:
            self.last_calibration_zoom_level = int(zoom_level)
        else:
            # A newly loaded/calculated result owns its zoom provenance.  Do
            # not inherit either the current spinner or a previous result's
            # capture zoom when this result has no readback evidence.
            self.last_calibration_zoom_level = None
        self._candidate_intrinsics_profile_path = (
            f"memory:checkerboard-validation:{mode}:zoom-"
            f"{self.last_calibration_zoom_level:02d}"
            if self.last_calibration_zoom_level is not None
            else f"memory:checkerboard-validation:{mode}:zoom-unknown"
        )
        return result

    def complete_random_validation(
        self,
        results: object,
        *,
        reference: StereoIntrinsicsResult,
        mode: str,
        pair_count: int,
        runs: int,
        subset_count: int,
        seed: int,
        quality: tuple[str, tuple[str, ...]],
    ) -> tuple[str, str]:
        rows: list[str] = []
        baselines: list[float] = []
        rotations: list[float] = []
        left_deltas: list[float] = []
        right_deltas: list[float] = []
        reference_rotation = None if reference.extrinsics is None else reference.extrinsics.rotation_left_to_right
        run_results: list[dict[str, object]] = [
            self._checkerboard_validation_result_data(
                "Full",
                reference,
                reference,
                pair_count,
            )
        ]
        rows.append(
            "Run       Pairs  Lerr   Rerr   Serr   Baseline(m)  dB(mm)   dR(deg)  max_dK_L(px)  max_dK_R(px)"
        )
        rows.append(self._checkerboard_validation_result_row("Full", reference, reference, pair_count))
        result_rows = results if isinstance(results, (tuple, list)) else ()
        failed_runs = 0
        successful_runs = 0
        for row in result_rows:
            if not isinstance(row, dict):
                continue
            index = int(row.get("index") or 0)
            error = str(row.get("error") or "")
            if error:
                failed_runs += 1
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
                failed_runs += 1
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
            successful_runs += 1
            run_results.append(
                self._checkerboard_validation_result_data(
                    f"Random {index}",
                    result,
                    reference,
                    subset_count,
                )
            )
            rows.append(
                self._checkerboard_validation_result_row(
                    f"Random {index}",
                    result,
                    reference,
                    subset_count,
                )
            )
            left_delta, right_delta = self._checkerboard_intrinsic_delta_px(result, reference)
            left_deltas.append(left_delta)
            right_deltas.append(right_delta)
            baseline = self._checkerboard_result_baseline_m(result)
            if baseline is not None:
                baselines.append(baseline)
            rotation = self._rotation_delta_deg_from_matrices(
                reference_rotation,
                None if result.extrinsics is None else result.extrinsics.rotation_left_to_right,
            )
            if rotation is not None:
                rotations.append(rotation)
        baseline_std = self._checkerboard_validation_stddev(baselines)
        rotation_std = self._checkerboard_validation_stddev(rotations)
        quality_state, quality_reasons = quality
        if successful_runs == 0:
            quality_state = "FAIL"
            quality_reasons = (*quality_reasons, "all_random_validation_runs_failed")
        elif failed_runs:
            if quality_state != "FAIL":
                quality_state = "WARNING"
            quality_reasons = (*quality_reasons, "random_validation_run_failed")
        quality_reasons = tuple(dict.fromkeys(quality_reasons))
        rows.extend(
            [
                "",
                "Stability:",
                f"max_dK_L std={self._checkerboard_validation_stddev(left_deltas):.3f} px",
                f"max_dK_R std={self._checkerboard_validation_stddev(right_deltas):.3f} px",
                f"baseline std={baseline_std * 1000.0:.3f} mm" if baselines else "baseline std=n/a",
                f"rotation delta std={rotation_std:.6f} deg" if rotations else "rotation delta std=n/a",
                f"quality state={quality_state}",
            ]
        )
        summary = dict(self.checkerboard_validation_summary)
        summary["random_validation"] = {
            "runs": runs,
            "subset_pair_count": subset_count,
            "seed": None if seed == 0 else seed,
            "baseline_std_m": baseline_std if baselines else None,
            "rotation_delta_std_deg": rotation_std if rotations else None,
            "left_intrinsics_delta_std_px": self._checkerboard_validation_stddev(left_deltas),
            "right_intrinsics_delta_std_px": self._checkerboard_validation_stddev(right_deltas),
            "successful_runs": successful_runs,
            "failed_runs": failed_runs,
            "quality_state": quality_state,
            "quality_reason_codes": list(quality_reasons),
        }
        self.checkerboard_validation_summary = summary
        self.checkerboard_validation_run_results = run_results
        self.calibration_progress = {
            "phase": "validation_complete",
            "processed": runs,
            "total": runs,
            "percent": 100.0,
            "message": f"랜덤 검증 완료: {runs}회 · quality={quality_state}",
        }
        return "\n".join(rows), quality_state
