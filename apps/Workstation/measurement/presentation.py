"""Qt presentation of measurement, optical alignment and quality.

Attitudes: body FRD -> earth NED, wxyz. GPS baselines: NED metres.
Optical R,t: X_right_RDF = R_right_from_left X_left_RDF + t_m.
Public point_m: baseline-center FRD; source_point_left_rdf_m: left optical RDF.
"""

from __future__ import annotations
import html as html_lib
import time
from collections.abc import Mapping
from dataclasses import dataclass
from math import isfinite
from PyQt5 import QtCore
from Workstation.workstation_core.attitude_geometry import relative_attitude
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_LABELS
from Workstation.workstation_core.gnss_operation import GnssOperationState
from Workstation.workstation_core.measurement_attempt import MeasurementAttempt
from Workstation.workstation_core.method_evaluation import MethodEvaluationOutcome
from Workstation.workstation_core.detection_pair_coordinator import PairTimingDecision
from Workstation.workstation_core.preview_measurement_worker import (
    PreviewMeasurementResult,
)
from Workstation.workstation_core.measurement_widgets import (
    triangulation_3d_scene_points,
)
from Workstation.workstation_core.rtk_geometry import (
    RtkBaselineEstimate,
    estimate_rtk_baseline,
)
from Workstation.workstation_core.stereo import DetectionObservation
from Workstation.workstation_core.sync import classify_attitude_timing
from Workstation.workstation_core.theme import (
    ERROR,
    ERROR_TEXT,
    PANEL_ALT_BG,
    PANEL_BORDER,
    SUCCESS,
    SUCCESS_TEXT,
    TEXT_MUTED,
    WARNING,
    WARNING_TEXT,
)
from Workstation.workstation_core.triangulation import RayTriangulationResult
from typing import Any, Callable
from Workstation.measurement import limits
from Workstation.measurement.ports import ValueInput
from Workstation.measurement.state import (
    UNSET,
    read,
    PairCaptureState,
    MeasurementPresentationState,
    WorkerRuntimeState,
)


@dataclass(frozen=True)
class MeasurementPresentationInputs:
    _selected_baseline_gps_states: ValueInput
    latest_rtk_baseline: ValueInput
    _refresh_p1_body_frd_baseline: ValueInput
    rtk_fixed_status_label: ValueInput
    rtk_baseline_label: ValueInput
    _refresh_operation_header: ValueInput
    _refresh_alignment_workflow_status: ValueInput
    latest_p1_body_frd_baseline: ValueInput
    rtk_summary_labels: ValueInput
    _set_health_label: ValueInput
    baseline_attitude_labels: ValueInput
    _set_label_text_if_changed: ValueInput
    rtk_alignment_plot: ValueInput
    latest_extrinsics: ValueInput
    rtk_alignment_detail_label: ValueInput
    optical_alignment_labels: ValueInput
    _matched_detection_pair: ValueInput
    optical_alignment_status_label: ValueInput
    optical_target_distance_spin_value: Callable[..., Any]
    _distance_mode_label: ValueInput
    _rtsp_urls_are_same_stream: ValueInput
    _measurement_display_mode: ValueInput
    gnss_operation_gate_active: ValueInput
    gnss_operation_manager: ValueInput
    _record_unresolved_pair: ValueInput
    _triangulation_extrinsics_source_label: ValueInput
    _append_distance_history: ValueInput
    left_pane: ValueInput
    right_pane: ValueInput
    _update_runtime_camera_stats_labels: ValueInput
    selected_runtime_frame_metadata: ValueInput
    _yolo_match_timestamp_s: ValueInput
    _effective_yolo_time_basis: ValueInput
    _yolo_time_basis: ValueInput
    _selected_pair_sync_summary: ValueInput
    detection_buffers: ValueInput
    camera_stream_stats: ValueInput
    _yolo_time_delta_gate_enabled: ValueInput
    _yolo_pair_max_delta_s: ValueInput
    triangulation_quality_status_label: ValueInput
    triangulation_quality_gap_label: ValueInput
    triangulation_quality_distance_label: ValueInput
    triangulation_quality_mode_label: ValueInput
    triangulation_quality_attitude_label: ValueInput
    triangulation_quality_raw_point_label: ValueInput
    triangulation_quality_plot: ValueInput
    distance_plot: ValueInput
    distance_label: ValueInput


@dataclass(frozen=True)
class MeasurementPresentationActions:
    panel__rtk_fixed_status_text: ValueInput
    panel__set_rtk_summary_chip: ValueInput
    panel__refresh_rtk_alignment_view: ValueInput
    panel__gps_baseline_usable_for_translation: ValueInput
    panel__baseline_forward_angle_level: ValueInput
    panel__baseline_forward_geometry_from_camera_translation: ValueInput
    panel__alignment_view_camera_translation_right_in_left_m: ValueInput
    panel__baseline_camera_translation_in_side_rdf_m: ValueInput
    panel__set_baseline_forward_metric_labels: ValueInput
    panel__baseline_forward_geometry_text: ValueInput
    panel__optical_alignment_expected_dy_px: ValueInput
    panel__format_optical_px: ValueInput
    panel__optical_alignment_mean_focal_px: ValueInput
    panel__build_measurement_attempt: ValueInput
    _preview_measurement_result: Callable[..., Any]
    panel__optical_alignment_level_from_mrad: ValueInput
    panel__optical_alignment_overall_level: ValueInput
    panel__format_optical_mrad: ValueInput
    panel__attitude_timing_health_level: ValueInput
    panel__optical_alignment_level_text: ValueInput
    panel__set_distance_text: ValueInput
    panel__clear_measurement_pair_display: ValueInput
    panel__clear_triangulation_quality: ValueInput
    panel__detection_wait_status: ValueInput
    panel__detection_wait_detail: ValueInput
    panel__geometry_comparison_rows: ValueInput
    _multi_method_measurement_active: Callable[..., Any]
    _select_measurement_method_for_attempt: Callable[..., Any]
    panel__show_measurement_pair: ValueInput
    _submit_method_evaluations: Callable[..., Any]
    _record_no_ready_measurement_pair: Callable[..., Any]
    panel__capture_measurement_gnss_contract: ValueInput
    panel__stereo_geometry_wait_detail: ValueInput
    panel__triangulation_wait_detail: ValueInput
    panel__update_triangulation_quality: ValueInput
    panel__clear_selected_runtime_measurement_pair: ValueInput
    _pair_timing_decision: Callable[..., Any]
    panel__format_attitude_timing_age: ValueInput
    panel__format_attitude_timing_text: ValueInput
    panel__format_quality_point: ValueInput
    panel__format_distance_label_text: ValueInput
    panel__distance_style: ValueInput


class MeasurementPresentationService:
    """Qt presentation of measurement, optical alignment and quality."""

    def __init__(
        self,
        *,
        state: MeasurementPresentationState,
        inputs: MeasurementPresentationInputs,
        actions: MeasurementPresentationActions,
        capture_state: PairCaptureState,
        runtime_state: WorkerRuntimeState,
    ) -> None:
        self.state = state
        self.inputs = inputs
        self.actions = actions
        self.capture_state = capture_state
        self.runtime_state = runtime_state

    def _refresh_rtk_baseline(self) -> RtkBaselineEstimate | None:
        left_gps, right_gps = self.inputs._selected_baseline_gps_states.get()()
        baseline = estimate_rtk_baseline(
            left_gps, right_gps, now_s=time.monotonic(), require_fixed=False
        )
        self.inputs.latest_rtk_baseline.set(baseline)
        self.inputs._refresh_p1_body_frd_baseline.get()()
        reference_text = " | 거리 계산 필수 입력"
        if self.inputs.rtk_fixed_status_label.exists():
            self.inputs.rtk_fixed_status_label.get().setText(
                self.actions.panel__rtk_fixed_status_text.get(None)(baseline)
            )
        if baseline is None:
            self.actions.panel__set_rtk_summary_chip.get(None)(
                "baseline", "Baseline: 대기 중", "idle"
            )
            self.actions.panel__set_rtk_summary_chip.get(None)(
                "camera", "Camera: 자세 대기 중", "idle"
            )
            self.actions.panel__set_rtk_summary_chip.get(None)(
                "status", "Status: GPS 대기 중", "idle"
            )
            self.inputs.rtk_baseline_label.get().setText(
                f"GPS baseline: 좌/우 GPS 대기 중{reference_text}"
            )
            self.actions.panel__refresh_rtk_alignment_view.get(None)()
            self.inputs._refresh_operation_header.get()()
            self.inputs._refresh_alignment_workflow_status.get()()
            return None
        warning_text = f" | {'; '.join(baseline.warnings)}" if baseline.warnings else ""
        usable, reason = self.actions.panel__gps_baseline_usable_for_translation.get(
            None
        )(baseline)
        baseline_level = (
            "good"
            if baseline.fix_quality == "GOOD"
            else "warn" if baseline.fix_quality == "DEGRADED" else "bad"
        )
        status_level = "good" if usable else "bad"
        self.actions.panel__set_rtk_summary_chip.get(None)(
            "baseline", f"Baseline {baseline.baseline_m:.3f} m", baseline_level
        )
        self.actions.panel__set_rtk_summary_chip.get(None)(
            "status",
            "Status: 사용 가능" if usable else f"Status: {reason}",
            status_level,
        )
        self.inputs.rtk_baseline_label.get().setText(
            f"GPS baseline: {baseline.baseline_m:.3f} m N/E/D={baseline.north_m:.3f}/{baseline.east_m:.3f}/{baseline.down_m:.3f} m "
            + (
                "P1 FRD=대기 중 "
                if self.inputs.latest_p1_body_frd_baseline.get() is None
                else f"P1 FRD F/R/D={self.inputs.latest_p1_body_frd_baseline.get().forward_m:.3f}/{self.inputs.latest_p1_body_frd_baseline.get().right_m:.3f}/{self.inputs.latest_p1_body_frd_baseline.get().down_m:.3f} m "
            )
            + f"age={baseline.left_age_s * 1000:.0f}/{baseline.right_age_s * 1000:.0f}ms quality={baseline.fix_quality}{reference_text}{warning_text}"
        )
        self.actions.panel__refresh_rtk_alignment_view.get(None)()
        self.inputs._refresh_operation_header.get()()
        self.inputs._refresh_alignment_workflow_status.get()()
        return baseline

    def _set_rtk_summary_chip(self, key: str, text: str, level: str) -> None:
        labels = self.inputs.rtk_summary_labels.get({})
        label = labels.get(key)
        if label is not None:
            self.inputs._set_health_label.get()(label, text, level)

    def _set_baseline_forward_metric_labels(
        self,
        *,
        prefix: str,
        geometry: tuple[float, float, float, float] | None,
        missing_text: str = "자세 대기 중",
    ) -> None:
        if not self.inputs.baseline_attitude_labels.exists():
            return
        angle_key = (
            "rdf_forward_angle" if prefix == "left" else "right_rdf_forward_angle"
        )
        effective_key = (
            "effective_baseline" if prefix == "left" else "right_effective_baseline"
        )
        forward_key = (
            "forward_component" if prefix == "left" else "right_forward_component"
        )
        if geometry is None:
            self.inputs._set_health_label.get()(
                self.inputs.baseline_attitude_labels.get()[angle_key],
                missing_text,
                "warn",
            )
            self.inputs._set_health_label.get()(
                self.inputs.baseline_attitude_labels.get()[effective_key],
                missing_text,
                "warn",
            )
            self.inputs._set_health_label.get()(
                self.inputs.baseline_attitude_labels.get()[forward_key],
                missing_text,
                "warn",
            )
            return
        angle_deg, effective_m, length_m, forward_m = geometry
        self.inputs._set_health_label.get()(
            self.inputs.baseline_attitude_labels.get()[angle_key],
            f"{angle_deg:.2f} deg ({('P1' if prefix == 'left' else 'P2')} RDF +Z vs P1->P2 baseline)",
            self.actions.panel__baseline_forward_angle_level.get(None)(angle_deg),
        )
        self.inputs._set_label_text_if_changed.get()(
            self.inputs.baseline_attitude_labels.get()[effective_key],
            f"{effective_m:.4f} m / {length_m:.4f} m",
        )
        self.inputs._set_label_text_if_changed.get()(
            self.inputs.baseline_attitude_labels.get()[forward_key],
            f"{forward_m:+.4f} m",
        )

    def _baseline_forward_geometry_text(
        self, camera_translation: tuple[float, float, float] | None
    ) -> str:
        geometry = (
            self.actions.panel__baseline_forward_geometry_from_camera_translation.get(
                None
            )(camera_translation)
        )
        if geometry is None:
            return "P1 RDF Forward 각도: 자세 대기 중"
        angle_deg, effective_m, length_m, forward_m = geometry
        return f"P1 RDF Forward vs baseline angle={angle_deg:.2f}°, effective={effective_m:.3f}/{length_m:.3f} m, forward={forward_m:+.3f} m"

    def _refresh_rtk_alignment_view(self) -> None:
        self.inputs._refresh_p1_body_frd_baseline.get()()
        if not self.inputs.rtk_alignment_plot.exists():
            return
        baseline = self.inputs.latest_rtk_baseline.get()
        if baseline is None:
            self.inputs.rtk_alignment_plot.get().set_alignment(
                None,
                left_attitude=self.inputs.latest_extrinsics.get().get("left"),
                right_attitude=self.inputs.latest_extrinsics.get().get("right"),
            )
            self.inputs.rtk_alignment_detail_label.get().setText(
                "정렬 성분: 좌/우 GPS 위치 대기 중. 안테나 baseline은 카메라 광축 수직 baseline과 다를 수 있습니다."
            )
            return
        camera_translation = (
            self.actions.panel__alignment_view_camera_translation_right_in_left_m.get(
                None
            )(baseline)
        )
        left_ext = self.inputs.latest_extrinsics.get().get("left")
        right_ext = self.inputs.latest_extrinsics.get().get("right")
        self.inputs.rtk_alignment_plot.get().set_alignment(
            baseline, camera_translation, left_ext, right_ext
        )
        antenna_text = f"안테나 |T|={baseline.baseline_m:.3f} m, N/E/D={baseline.north_m:.3f}/{baseline.east_m:.3f}/{baseline.down_m:.3f} m"
        if camera_translation is None:
            self.actions.panel__set_rtk_summary_chip.get(None)(
                "camera", "Camera: 자세 대기 중", "warn"
            )
            body_text = "P1 body FRD=자세 대기 중"
            if self.inputs.latest_p1_body_frd_baseline.get() is not None:
                projection = self.inputs.latest_p1_body_frd_baseline.get()
                body_text = f"P1 body FRD F/R/D={projection.forward_m:+.3f}/{projection.right_m:+.3f}/{projection.down_m:+.3f} m"
            self.inputs.rtk_alignment_detail_label.get().setText(
                f"정렬 성분: {antenna_text}. GPS-only 모드: N/E/D 기준선을 표시합니다. {body_text}. 카메라 기준 Right/Down/Forward 변환은 픽스호크 자세 수신 후 추가됩니다."
            )
            return
        attitude_text = ""
        if left_ext is not None and right_ext is not None:
            relative = relative_attitude(left_ext.q, right_ext.q)
            attitude_text = f" 상대 자세 ΔR/P/Y={relative.roll_deg:+.2f}/{relative.pitch_deg:+.2f}/{relative.yaw_deg:+.2f}°."
        x_m, y_m, z_m = camera_translation
        right_camera_translation = (
            self.actions.panel__baseline_camera_translation_in_side_rdf_m.get(None)(
                baseline, "right"
            )
        )
        left_geometry = (
            self.actions.panel__baseline_forward_geometry_from_camera_translation.get(
                None
            )(camera_translation)
        )
        right_geometry = (
            self.actions.panel__baseline_forward_geometry_from_camera_translation.get(
                None
            )(right_camera_translation)
        )
        self.actions.panel__set_baseline_forward_metric_labels.get(None)(
            prefix="left", geometry=left_geometry
        )
        self.actions.panel__set_baseline_forward_metric_labels.get(None)(
            prefix="right", geometry=right_geometry
        )
        right_geometry_text = self.actions.panel__baseline_forward_geometry_text.get(
            None
        )(right_camera_translation).replace("P1 RDF Forward", "P2 RDF Forward")
        forward_warn = abs(z_m) > max(0.2, abs(x_m) * 0.5)
        down_warn = abs(y_m) > max(0.1, abs(x_m) * 0.25)
        self.actions.panel__set_rtk_summary_chip.get(None)(
            "camera",
            f"P1 RDF R/D/F {x_m:+.3f}/{y_m:+.3f}/{z_m:+.3f} m",
            "warn" if forward_warn or down_warn else "good",
        )
        self.inputs.rtk_alignment_detail_label.get().setText(
            f"정렬 성분: {antenna_text}. "
            + (
                "P1 body FRD 대기 중. "
                if self.inputs.latest_p1_body_frd_baseline.get() is None
                else f"P1 body FRD F/R/D={self.inputs.latest_p1_body_frd_baseline.get().forward_m:+.3f}/{self.inputs.latest_p1_body_frd_baseline.get().right_m:+.3f}/{self.inputs.latest_p1_body_frd_baseline.get().down_m:+.3f} m. "
            )
            + f"P1 RDF Right={x_m:.3f} m, Down={y_m:.3f} m, Forward={z_m:.3f} m. P1 RDF 좌우 성분 |Right|={abs(x_m):.3f} m입니다. {self.actions.panel__baseline_forward_geometry_text.get(None)(camera_translation)} | {right_geometry_text}.{attitude_text}"
        )

    def _optical_alignment_level_text(self, level: str) -> str:
        return {"good": "GOOD", "warn": "WARN", "bad": "BAD", "idle": "WAITING"}.get(
            level, "WAITING"
        )

    def _format_optical_px(self, value: float | None) -> str:
        return "--" if value is None or not isfinite(value) else f"{value:+.3f} px"

    def _format_optical_mrad(self, value: float | None) -> str:
        return "--" if value is None or not isfinite(value) else f"{value:+.3f} mrad"

    def _refresh_optical_alignment_view(self) -> None:
        if not self.inputs.optical_alignment_labels.exists():
            return
        labels = self.inputs.optical_alignment_labels.get()
        pair = self.inputs._matched_detection_pair.get()()
        if pair is None:
            self.inputs._set_health_label.get()(
                self.inputs.optical_alignment_status_label.get(),
                "Optical alignment: YOLO target pair 대기 중",
                "idle",
            )
            for label in labels.values():
                self.inputs._set_health_label.get()(label, "대기 중", "idle")
            expected_dy, expected_source, expected_level = (
                self.actions.panel__optical_alignment_expected_dy_px.get(None)()
            )
            self.inputs._set_health_label.get()(
                labels["expected_dy"],
                f"{self.actions.panel__format_optical_px.get(None)(expected_dy)} | {expected_source}",
                expected_level,
            )
            return
        left_center_x, left_center_y = pair.left.center
        right_center_x, right_center_y = pair.right.center
        expected_dy_px, expected_dy_source, expected_dy_level = (
            self.actions.panel__optical_alignment_expected_dy_px.get(None)()
        )
        mean_fy_px, fy_source = self.actions.panel__optical_alignment_mean_focal_px.get(
            None
        )("y")
        mean_fx_px, fx_source = self.actions.panel__optical_alignment_mean_focal_px.get(
            None
        )("x")
        target_distance_m = self.inputs.optical_target_distance_spin_value()
        current_identity = (
            str(pair.left.timestamp_metadata.get("frame_key") or pair.left.timestamp_s),
            str(
                pair.right.timestamp_metadata.get("frame_key") or pair.right.timestamp_s
            ),
        )
        preview_result = read(
            self.capture_state._latest_preview_measurement_result, None
        )
        if (
            not isinstance(preview_result, PreviewMeasurementResult)
            or read(self.capture_state._latest_preview_pair_identity, None)
            != current_identity
        ):
            try:
                optical_attempt = self.actions.panel__build_measurement_attempt.get(
                    None
                )(pair)
            except (TypeError, ValueError):
                optical_attempt = None
            if optical_attempt is not None:
                self.actions._preview_measurement_result(optical_attempt, pair)
            self.inputs._set_health_label.get()(
                self.inputs.optical_alignment_status_label.get(),
                "Optical alignment: measurement worker 계산 중",
                "idle",
            )
            return
        metrics = dict(preview_result.optical_metrics)
        observed_dx_px = metrics.get("observed_dx_px")
        observed_dy_px = metrics.get("observed_dy_px")
        dy_residual_px = metrics.get("dy_residual_px")
        vertical_error_mrad = metrics.get("vertical_error_mrad")
        expected_disparity_px = metrics.get("expected_disparity_px")
        yaw_residual_px = metrics.get("yaw_residual_px")
        yaw_error_mrad = metrics.get("yaw_error_mrad")
        camera_delta_s = metrics.get("camera_delta_s")
        vertical_level = self.actions.panel__optical_alignment_level_from_mrad.get(
            None
        )(vertical_error_mrad)
        attempt = preview_result.attempt
        geometry_result = preview_result.geometry_result
        geometry_error = preview_result.error
        baseline_m = 0.0 if attempt is None else attempt.baseline_m
        baseline_source = "waiting" if attempt is None else attempt.baseline_source
        yaw_level = self.actions.panel__optical_alignment_level_from_mrad.get(None)(
            yaw_error_mrad
        )
        overall_level = self.actions.panel__optical_alignment_overall_level.get(None)(
            (vertical_level, yaw_level)
        )
        class_text = pair.left.class_name or str(pair.left.class_id)
        confidence_text = f"L={pair.left.confidence:.2f}, R={pair.right.confidence:.2f}"
        self.inputs._set_health_label.get()(
            labels["left_center"],
            f"x={left_center_x:.1f}, y={left_center_y:.1f} | {pair.left.width}x{pair.left.height}",
            "good",
        )
        self.inputs._set_health_label.get()(
            labels["right_center"],
            f"x={right_center_x:.1f}, y={right_center_y:.1f} | {pair.right.width}x{pair.right.height}",
            "good",
        )
        self.inputs._set_health_label.get()(
            labels["bbox_delta"],
            f"dx={observed_dx_px:+.3f} px, dy={observed_dy_px:+.3f} px",
            "good",
        )
        self.inputs._set_health_label.get()(
            labels["bbox_quality"],
            f"class={class_text} | conf {confidence_text}",
            "good",
        )
        camera_delta_text = (
            "--" if camera_delta_s is None else f"{camera_delta_s * 1000.0:.1f} ms"
        )
        self.inputs._set_health_label.get()(
            labels["camera_time"],
            camera_delta_text,
            "good" if camera_delta_s is not None else "warn",
        )
        self.inputs._set_health_label.get()(
            labels["expected_dy"],
            f"{self.actions.panel__format_optical_px.get(None)(expected_dy_px)} | {expected_dy_source}",
            expected_dy_level,
        )
        self.inputs._set_health_label.get()(
            labels["observed_dy"],
            self.actions.panel__format_optical_px.get(None)(observed_dy_px),
            "good",
        )
        self.inputs._set_health_label.get()(
            labels["dy_residual"],
            self.actions.panel__format_optical_px.get(None)(dy_residual_px),
            vertical_level,
        )
        self.inputs._set_health_label.get()(
            labels["vertical_error"],
            (
                f"{self.actions.panel__format_optical_mrad.get(None)(vertical_error_mrad)} | fy={mean_fy_px:.1f}px"
                if mean_fy_px is not None
                else f"-- | {fy_source}"
            ),
            vertical_level,
        )
        self.inputs._set_health_label.get()(
            labels["observed_dx"],
            self.actions.panel__format_optical_px.get(None)(observed_dx_px),
            "good",
        )
        self.inputs._set_health_label.get()(
            labels["expected_disparity"],
            f"{self.actions.panel__format_optical_px.get(None)(expected_disparity_px)} | target={target_distance_m:.2f} m",
            "good" if expected_disparity_px is not None else "warn",
        )
        self.inputs._set_health_label.get()(
            labels["yaw_residual"],
            self.actions.panel__format_optical_px.get(None)(yaw_residual_px),
            yaw_level,
        )
        self.inputs._set_health_label.get()(
            labels["yaw_error"],
            (
                f"{self.actions.panel__format_optical_mrad.get(None)(yaw_error_mrad)} | fx={mean_fx_px:.1f}px"
                if mean_fx_px is not None
                else f"-- | {fx_source}"
            ),
            yaw_level,
        )
        triangulated = None if geometry_result is None else geometry_result.triangulated
        estimate = None if geometry_result is None else geometry_result.estimate
        if triangulated is None:
            reason = geometry_error or (
                "" if geometry_result is None else geometry_result.triangulation_reason
            )
            self.inputs._set_health_label.get()(labels["range"], "대기 중", "idle")
            self.inputs._set_health_label.get()(
                labels["ray_gap"], reason or "geometry 대기 중", "warn"
            )
        else:
            self.inputs._set_health_label.get()(
                labels["range"], f"{triangulated.range_m:.3f} m", "good"
            )
            self.inputs._set_health_label.get()(
                labels["ray_gap"], f"{triangulated.closest_gap_m:.3f} m", "good"
            )
        self.inputs._set_health_label.get()(
            labels["baseline"],
            f"{baseline_source}:{baseline_m:.3f} m",
            "good" if baseline_m > 0.0 else "warn",
        )
        triangulation_text = (
            "대기 중"
            if attempt is None
            else f"{attempt.triangulation_source} | {(geometry_result.triangulation_reason if geometry_result else geometry_error)}"
        )
        self.inputs._set_health_label.get()(
            labels["triangulation"],
            triangulation_text,
            "good" if triangulated is not None else "warn",
        )
        attitude_status = "WAITING"
        if attempt is not None:
            uses_live_pixhawk = (
                "live_pixhawk" in dict(attempt.measurement_component_sources).values()
            )
            attitude_status = classify_attitude_timing(
                uses_pixhawk_attitude=uses_live_pixhawk,
                left_age_s=None if estimate is None else estimate.left_attitude_age_s,
                right_age_s=None if estimate is None else estimate.right_attitude_age_s,
                pair_delta_s=attempt.attitude_pair_delta_s,
            )
        self.inputs._set_health_label.get()(
            labels["attitude"],
            attitude_status,
            self.actions.panel__attitude_timing_health_level.get(None)(attitude_status),
        )
        self.inputs._set_health_label.get()(
            self.inputs.optical_alignment_status_label.get(),
            f"Optical alignment: {self.actions.panel__optical_alignment_level_text.get(None)(overall_level)} | vertical={self.actions.panel__format_optical_mrad.get(None)(vertical_error_mrad)} | yaw={self.actions.panel__format_optical_mrad.get(None)(yaw_error_mrad)} | mode={self.inputs._distance_mode_label.get()()}",
            overall_level,
        )

    def _display_primary_method_outcome(
        self,
        outcome: MethodEvaluationOutcome,
        *,
        waiting_for_barrier: bool,
        persistence_state: str = "NOT_REQUESTED",
    ) -> None:
        suffix = ""
        persistence_rejected = persistence_state == "REJECTED"
        if persistence_rejected:
            suffix = " · EVIDENCE DEGRADED · 대표 기록 background 재시도 중"
            if waiting_for_barrier:
                suffix += " · comparison barrier 대기"
        elif waiting_for_barrier:
            if persistence_state == "QUEUED":
                suffix = " · 대표 기록 QUEUED · comparison barrier 대기"
            else:
                suffix = " · comparison barrier 대기"
        result = outcome.result if isinstance(outcome.result, Mapping) else {}
        triangulated = result.get("triangulated")
        if outcome.status in {"VALID", "ESTIMATED"} and isinstance(
            triangulated, Mapping
        ):
            distance_m = triangulated.get("range_m")
            try:
                distance_text = f"{float(distance_m):.2f} m"
            except (TypeError, ValueError):
                distance_text = "--"
            reused = " · REUSED" if outcome.reused_inputs else ""
            method_label = MEASUREMENT_METHOD_LABELS.get(
                outcome.method_id, outcome.method_id
            )
            self.actions.panel__set_distance_text.get(None)(
                f"거리: {distance_text} ({method_label} worker){reused}{suffix}",
                quality=(
                    "WARNING"
                    if persistence_rejected
                    else "ESTIMATED" if outcome.status == "ESTIMATED" else "GOOD"
                ),
            )
            return
        reason = outcome.reasons[0] if outcome.reasons else outcome.status
        self.actions.panel__set_distance_text.get(None)(
            f"거리: Pixhawk + GNSS/RTK {outcome.status} · {reason}{suffix}",
            quality="ERROR" if outcome.status == "FAILED" else "WARNING",
        )

    def _refresh_distance(self) -> None:
        self.state.geometry_comparison_rows = ()
        if self.inputs._rtsp_urls_are_same_stream.get()():
            self.actions.panel__clear_measurement_pair_display.get(None)()
            self.actions.panel__clear_triangulation_quality.get(None)(
                "좌/우 RTSP URL이 동일해 3D ray 품질을 표시할 수 없습니다."
            )
            self.actions.panel__set_distance_text.get(None)(
                "거리: 좌/우 RTSP URL이 동일해 측정 차단",
                quality="ERROR",
                detail="그래프 미표시: 좌/우가 같은 스트림이라 유효한 매칭 쌍이 없습니다.",
            )
            return
        pair = self.inputs._matched_detection_pair.get()()
        if pair is None:
            self.actions.panel__clear_measurement_pair_display.get(None)()
            wait_status = self.actions.panel__detection_wait_status.get(None)()
            self.actions.panel__clear_triangulation_quality.get(None)(
                wait_status.removeprefix("거리: ")
            )
            self.actions.panel__set_distance_text.get(None)(
                wait_status,
                quality="WARNING",
                detail=self.actions.panel__detection_wait_detail.get(None)(),
            )
            return
        try:
            self.state.geometry_comparison_rows = (
                self.actions.panel__geometry_comparison_rows.get(None)(pair)
            )
        except (TypeError, ValueError):
            self.state.geometry_comparison_rows = ()
        if self.actions._multi_method_measurement_active():
            selected_method_status = (
                self.actions._select_measurement_method_for_attempt()
            )
            if selected_method_status is not None and (
                not str(
                    read(self.runtime_state._measurement_recording_method_id, "") or ""
                )
            ):
                self.runtime_state._measurement_recording_method_id = (
                    selected_method_status.method_id
                )
            if self.inputs._measurement_display_mode.get()() == "detection":
                self.actions.panel__show_measurement_pair.get(None)(
                    pair.left, pair.right
                )
            else:
                self.actions.panel__clear_measurement_pair_display.get(None)()
            try:
                self.actions._submit_method_evaluations(pair)
            except (
                AttributeError,
                KeyError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                self.actions.panel__clear_triangulation_quality.get(None)(
                    "method snapshot 생성 실패"
                )
                self.actions.panel__set_distance_text.get(None)(
                    f"거리: method snapshot 생성 실패 | {exc}", quality="ERROR"
                )
            return
        selected_method_status = self.actions._select_measurement_method_for_attempt()
        if selected_method_status is None:
            record_detail = "해당 matched pair를 geometry 미준비 상태로 기록했습니다."
            try:
                self.actions._record_no_ready_measurement_pair(pair)
            except (
                AttributeError,
                KeyError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                record_detail = f"blocked-pair 기록 실패(계속 진행): {exc}"
            self.actions.panel__clear_triangulation_quality.get(None)(
                "준비된 측정 방식 없음"
            )
            self.actions.panel__set_distance_text.get(None)(
                "거리: 준비된 측정 방식 없음", quality="WARNING", detail=record_detail
            )
            return
        gnss_measurement_advisories: tuple[str, ...] = ()
        if (
            selected_method_status is not None
            and selected_method_status.requires_live_gnss
            and self.inputs.gnss_operation_gate_active.get()
        ):
            readiness = self.inputs.gnss_operation_manager.get().readiness()
            config = self.inputs.gnss_operation_manager.get().config
            if config.readiness_gate_enabled and (
                readiness is None or not readiness.ready
            ):
                reasons = (
                    ("GNSS readiness snapshot 대기",)
                    if readiness is None
                    else readiness.reasons
                )
                reason_text = (
                    "; ".join(reasons[:3])
                    or self.inputs.gnss_operation_manager.get().state.value
                )
                gnss_measurement_advisories = tuple(reasons) or (reason_text,)
            if readiness is not None:
                try:
                    contract = (
                        self.actions.panel__capture_measurement_gnss_contract.get(None)(
                            readiness
                        )
                    )
                    if (
                        self.inputs.gnss_operation_manager.get().state
                        == GnssOperationState.READY
                    ):
                        self.inputs.gnss_operation_manager.get().begin_measurement(
                            contract, readiness
                        )
                    elif (
                        self.inputs.gnss_operation_manager.get().state
                        == GnssOperationState.MEASURING
                    ):
                        if not self.inputs.gnss_operation_manager.get().validate_measurement_contract(
                            contract, readiness
                        ):
                            raise RuntimeError("GNSS measurement contract changed")
                except (RuntimeError, ValueError) as exc:
                    gnss_measurement_advisories = tuple(
                        dict.fromkeys((*gnss_measurement_advisories, str(exc)))
                    )
        if self.inputs._measurement_display_mode.get()() == "detection":
            self.actions.panel__show_measurement_pair.get(None)(pair.left, pair.right)
        else:
            self.actions.panel__clear_measurement_pair_display.get(None)()
        try:
            attempt = self.actions.panel__build_measurement_attempt.get(None)(pair)
        except (TypeError, ValueError) as exc:
            self.actions.panel__clear_triangulation_quality.get(None)(
                "측정 입력 snapshot 생성 실패"
            )
            self.actions.panel__set_distance_text.get(None)(
                f"거리: 측정 입력 snapshot 생성 실패 | {exc}",
                quality="WARNING",
                detail="그래프 미표시: 동일 입력 provenance를 고정할 수 없습니다.",
            )
            return
        if gnss_measurement_advisories:
            attempt = MeasurementAttempt.from_canonical_payload(
                {
                    **attempt.canonical_payload(),
                    "warnings": list(
                        dict.fromkeys((*attempt.warnings, *gnss_measurement_advisories))
                    ),
                }
            )
        preview_result = self.actions._preview_measurement_result(attempt, pair)
        if preview_result is None:
            self.actions.panel__clear_triangulation_quality.get(None)(
                "measurement worker 계산 중"
            )
            self.actions.panel__set_distance_text.get(None)(
                "거리: measurement worker 계산 중",
                quality="WARNING",
                detail="GUI는 계산하지 않고 latest worker 결과를 기다립니다.",
            )
            return
        if preview_result.error or preview_result.geometry_result is None:
            self.actions.panel__clear_triangulation_quality.get(None)(
                "measurement worker 실패"
            )
            self.actions.panel__set_distance_text.get(None)(
                f"거리: measurement worker 실패 | {preview_result.error}",
                quality="ERROR",
            )
            return
        attempt = preview_result.attempt or attempt
        self.capture_state._latest_preview_measurement_result = preview_result
        self.capture_state._latest_preview_pair_identity = (
            str(pair.left.timestamp_metadata.get("frame_key") or pair.left.timestamp_s),
            str(
                pair.right.timestamp_metadata.get("frame_key") or pair.right.timestamp_s
            ),
        )
        baseline_m = attempt.baseline_m
        baseline_source = attempt.baseline_source
        geometry_result = preview_result.geometry_result
        match = geometry_result.match
        triangulated = geometry_result.triangulated
        estimate = geometry_result.estimate
        combined_warnings = geometry_result.warnings
        if match is None:
            self.inputs._record_unresolved_pair.get()(
                attempt,
                status="matched_stereo_geometry_invalid",
                warnings=combined_warnings,
            )
            self.actions.panel__clear_triangulation_quality.get(None)(
                "유효한 스테레오 기하 대기 중"
            )
            self.actions.panel__set_distance_text.get(None)(
                "거리: 유효한 스테레오 기하 대기 중",
                quality="WARNING",
                detail=self.actions.panel__stereo_geometry_wait_detail.get(None)(pair),
            )
            return
        if triangulated is None:
            self.inputs._record_unresolved_pair.get()(
                attempt,
                status="matched_triangulation_pending",
                warnings=combined_warnings,
            )
            warning_text = (
                f" | {'; '.join(combined_warnings)}" if combined_warnings else ""
            )
            self.actions.panel__clear_triangulation_quality.get(None)(
                "3D ray 기하 대기 중"
            )
            self.actions.panel__set_distance_text.get(None)(
                f"거리: 3D ray 기하 대기 중 ({self.inputs._triangulation_extrinsics_source_label.get()()}){warning_text}",
                quality="WARNING",
                detail=self.actions.panel__triangulation_wait_detail.get(None)(
                    combined_warnings
                ),
            )
            return
        if estimate is None:
            self.inputs._record_unresolved_pair.get()(
                attempt, status="matched_distance_invalid", warnings=combined_warnings
            )
            self.actions.panel__clear_triangulation_quality.get(None)(
                "유효한 거리 계산 대기 중"
            )
            self.actions.panel__set_distance_text.get(None)(
                "거리: 유효한 거리 계산 대기 중",
                quality="WARNING",
                detail="그래프 미표시: 현재 입력으로 거리를 만들 수 없습니다.",
            )
            return
        if tuple(estimate.warnings) != tuple(attempt.warnings):
            attempt = MeasurementAttempt.from_canonical_payload(
                {
                    **attempt.canonical_payload(),
                    "warnings": list(estimate.warnings),
                    "advisory_reasons": list(estimate.warnings),
                }
            )
        public_point_m = triangulated.point_m
        source_point_left_rdf_m = getattr(triangulated, "source_point_left_rdf_m", None)
        display_point_m = public_point_m
        display_distance_m = triangulated.range_m
        method_text = f"{triangulated.method}-{attempt.measurement_method_id}"
        gap_text = f" gap={triangulated.closest_gap_m:.3f}m"
        display_baseline_source = baseline_source
        display_baseline_m = baseline_m
        attitude_timing_status = classify_attitude_timing(
            uses_pixhawk_attitude="live_pixhawk"
            in dict(attempt.measurement_component_sources).values(),
            left_age_s=estimate.left_attitude_age_s,
            right_age_s=estimate.right_attitude_age_s,
            pair_delta_s=estimate.attitude_pair_delta_s,
        )
        self.actions.panel__update_triangulation_quality.get(None)(
            triangulated=triangulated,
            method_text=method_text,
            display_distance_m=display_distance_m,
            display_baseline_source=display_baseline_source,
            display_baseline_m=display_baseline_m,
            public_point_m=public_point_m,
            source_point_left_rdf_m=source_point_left_rdf_m,
            attitude_timing_status=attitude_timing_status,
            left_attitude_age_s=estimate.left_attitude_age_s,
            right_attitude_age_s=estimate.right_attitude_age_s,
            attitude_pair_delta_s=estimate.attitude_pair_delta_s,
        )
        attitude_badge = (
            "N/A" if attitude_timing_status == "NOT_USED" else attitude_timing_status
        )
        display_quality = estimate.sync_quality
        if (
            attitude_timing_status in {"STALE", "VERY_STALE"}
            and display_quality == "GOOD"
        ):
            display_quality = "DEGRADED"
        measurement_validity = "ESTIMATED" if estimate.warnings else "VALID"
        if measurement_validity == "ESTIMATED":
            display_quality = "ESTIMATED"
        relative_text = "relRPY=대기"
        if estimate.relative_roll_deg is not None:
            relative_text = f"relRPY={estimate.relative_roll_deg:.2f}/{estimate.relative_pitch_deg:.2f}/{estimate.relative_yaw_deg:.2f}°"
        warning_text = f" | {'; '.join(estimate.warnings)}" if estimate.warnings else ""
        pair_input_unverified = any(
            (
                bool(
                    detection.timestamp_metadata.get(
                        "media_integrity_unverified", False
                    )
                )
                for detection in (attempt.left_detection, attempt.right_detection)
            )
        )
        input_integrity_text = (
            " input=UNVERIFIED" if pair_input_unverified else " input=VERIFIED"
        )
        self.actions.panel__set_distance_text.get(None)(
            f"거리: {display_distance_m:.2f} m ({method_text}{gap_text}) | bboxΔx={estimate.disparity_px:.1f}px camΔt={estimate.time_delta_s * 1000:.0f}ms Δy={estimate.vertical_delta_px:.1f}px baseline={display_baseline_source}:{display_baseline_m:.3f}m ATT:{attitude_badge} {relative_text} validity={measurement_validity} quality={display_quality}{input_integrity_text}{warning_text}",
            quality=display_quality,
        )
        appended = self.inputs._append_distance_history.get()(
            estimate=estimate,
            distance_m=display_distance_m,
            method=method_text,
            baseline_source=display_baseline_source,
            baseline_m=display_baseline_m,
            point_m=display_point_m,
            raw_point_m=public_point_m,
            raw_point_left_m=source_point_left_rdf_m,
            measurement_attempt=attempt,
        )
        if not appended:
            self.actions.panel__set_distance_text.get(None)(
                read(self.state.last_distance_status),
                quality=display_quality,
                detail="그래프 미표시: 직전과 동일한 샘플이라 중복 추가를 건너뛰었습니다.",
            )

    def _show_measurement_pair(
        self, left: DetectionObservation, right: DetectionObservation
    ) -> None:
        left_shown = self.inputs.left_pane.get().show_measurement_frame_for_detection(
            left
        )
        right_shown = self.inputs.right_pane.get().show_measurement_frame_for_detection(
            right
        )
        if not (left_shown and right_shown):
            self.inputs._update_runtime_camera_stats_labels.get()()
            return
        self.inputs.selected_runtime_frame_metadata.get()["left"] = dict(
            left.timestamp_metadata
        )
        self.inputs.selected_runtime_frame_metadata.get()["right"] = dict(
            right.timestamp_metadata
        )
        result = read(self.capture_state._coordinated_pairing_result, None)
        result_pair = getattr(result, "pair", None)
        timing = getattr(result, "timing", None)
        if (
            isinstance(timing, PairTimingDecision)
            and result_pair is not None
            and (result_pair.left is left)
            and (result_pair.right is right)
        ):
            self.state.selected_runtime_frame_pair_delta_s = timing.abs_delta_ns * 1e-09
            self.state.selected_runtime_frame_pair_basis = timing.effective_basis
            self.state.selected_runtime_frame_pair_requested_basis = (
                timing.requested_basis
            )
            self.state.selected_runtime_frame_pair_fallback_reason = (
                timing.fallback_reason
            )
        else:
            left_timestamp_s = self.inputs._yolo_match_timestamp_s.get()(left)
            right_timestamp_s = self.inputs._yolo_match_timestamp_s.get()(right)
            self.state.selected_runtime_frame_pair_delta_s = (
                None
                if left_timestamp_s is None or right_timestamp_s is None
                else abs(left_timestamp_s - right_timestamp_s)
            )
            self.state.selected_runtime_frame_pair_basis = (
                self.inputs._effective_yolo_time_basis.get()()
            )
            self.state.selected_runtime_frame_pair_requested_basis = (
                self.inputs._yolo_time_basis.get()()
            )
            self.state.selected_runtime_frame_pair_fallback_reason = ""
        _, self.state.selected_runtime_frame_pair_camera_rtc_delta_s = (
            self.inputs._selected_pair_sync_summary.get()(left, right)
        )
        self.inputs._update_runtime_camera_stats_labels.get()()

    def _clear_measurement_pair_display(self) -> None:
        self.inputs.left_pane.get().clear_measurement_pair_display()
        self.inputs.right_pane.get().clear_measurement_pair_display()
        self.actions.panel__clear_selected_runtime_measurement_pair.get(None)(
            update_labels=True
        )

    def _clear_selected_runtime_measurement_pair(
        self, *, update_labels: bool = True
    ) -> None:
        self.inputs.selected_runtime_frame_metadata.get()["left"] = {}
        self.inputs.selected_runtime_frame_metadata.get()["right"] = {}
        self.state.selected_runtime_frame_pair_delta_s = None
        self.state.selected_runtime_frame_pair_basis = ""
        self.state.selected_runtime_frame_pair_camera_rtc_delta_s = None
        self.state.selected_runtime_frame_pair_requested_basis = ""
        self.state.selected_runtime_frame_pair_fallback_reason = ""
        if update_labels:
            self.inputs._update_runtime_camera_stats_labels.get()()

    def _detection_wait_detail(self) -> str:
        left_count = len(self.inputs.detection_buffers.get()["left"])
        right_count = len(self.inputs.detection_buffers.get()["right"])
        left_stats = self.inputs.camera_stream_stats.get().get("left") or {}
        right_stats = self.inputs.camera_stream_stats.get().get("right") or {}
        left_yolo_hz = float(left_stats.get("inferred_hz", 0.0))
        right_yolo_hz = float(right_stats.get("inferred_hz", 0.0))
        rejection_reason = str(
            read(self.capture_state._last_correspondence_rejection_reason) or ""
        )
        if rejection_reason.startswith(("APP_", "RTCP_", "CLOCK_", "FRAME_")):
            result = read(self.capture_state._coordinated_pairing_result, None)
            primary_reason = str(getattr(result, "primary_rejection_reason", "") or "")
            primary_detail = f", preferred={primary_reason}" if primary_reason else ""
            return f"그래프 미표시: 선택 timing과 app-receive fallback에서 유효한 pair를 만들지 못했습니다 (effective={rejection_reason}{primary_detail})."
        if rejection_reason and (not rejection_reason.startswith("missing_")):
            return f"그래프 미표시: correspondence 검증 실패 ({rejection_reason})."
        if left_count == 0 or right_count == 0:
            if left_yolo_hz <= 0.0 or right_yolo_hz <= 0.0:
                return f"그래프 미표시: YOLO 추론 대기 중 (left={left_yolo_hz:.1f} Hz, right={right_yolo_hz:.1f} Hz)."
            return f"그래프 미표시: bbox 검출 대기 중 (left={left_count}, right={right_count})."
        return (
            f"그래프 미표시: 좌/우 bbox timestamp가 `최대 Δt` 안에서 아직 맞지 않습니다 (left={left_count}, right={right_count})."
            if self.inputs._yolo_time_delta_gate_enabled.get()()
            else f"그래프 미표시: 좌/우 bbox 매칭 후보 대기 중 (left={left_count}, right={right_count})."
        )

    def _detection_wait_status(self) -> str:
        rejection_reason = str(
            read(self.capture_state._last_correspondence_rejection_reason) or ""
        )
        if rejection_reason.startswith(("APP_", "RTCP_", "CLOCK_", "FRAME_")):
            return f"거리: timing pair 대기 ({rejection_reason})"
        if rejection_reason and (not rejection_reason.startswith("missing_")):
            return f"거리: 검출 대응 거부 ({rejection_reason})"
        if (
            self.inputs._yolo_time_delta_gate_enabled.get()()
            or self.inputs._yolo_time_basis.get()() != "detection"
        ):
            return "거리: timestamp가 맞는 좌/우 검출 대기 중"
        return "거리: 좌/우 검출 쌍 대기 중"

    def _stereo_geometry_wait_detail(self, pair) -> str:
        left = pair.left
        right = pair.right
        left_center = getattr(left, "center", (0.0, 0.0))
        right_center = getattr(right, "center", (0.0, 0.0))
        disparity_px = abs(float(left_center[0]) - float(right_center[0]))
        timing = self.actions._pair_timing_decision(pair)
        time_delta_ms = timing.abs_delta_ns * 1e-06
        time_limit_ms = self.inputs._yolo_pair_max_delta_s.get()() * 1000.0
        if not isfinite(disparity_px) or disparity_px <= 1e-06:
            return f"그래프 미표시: 좌/우 bbox 중심의 x 차이가 0에 가까워 disparity를 만들 수 없습니다. left_x={float(left_center[0]):.1f}px, right_x={float(right_center[0]):.1f}px."
        if time_delta_ms > time_limit_ms:
            return f"그래프 미표시: 좌/우 bbox timestamp 차이가 `최대 Δt`보다 큽니다. camΔt={time_delta_ms:.0f}ms, limit={time_limit_ms:.0f}ms."
        return f"그래프 미표시: 좌/우 bbox는 선택됐지만 거리 계산용 stereo match가 생성되지 않았습니다. disparity={disparity_px:.1f}px, camΔt={time_delta_ms:.0f}ms."

    def _triangulation_wait_detail(self, warnings: tuple[str, ...]) -> str:
        warning_text = "; ".join(warnings)
        if any(("waiting for intrinsics" in warning for warning in warnings)):
            return f"그래프 미표시: bbox 매칭은 됐지만 내부 파라미터가 아직 적용되지 않아 3D ray triangulation을 계산할 수 없습니다. `줌 내부 파라미터`에서 intrinsics JSON을 불러온 뒤 `계산에 적용`을 누르세요. {warning_text}"
        if any(("waiting for Pixhawk R" in warning for warning in warnings)):
            return f"그래프 미표시: bbox 매칭은 됐지만 Pixhawk 자세 샘플이 아직 없어 3D ray 방향을 만들 수 없습니다. {warning_text}"
        if any(
            (
                "waiting for checkerboard stereo extrinsics" in warning
                for warning in warnings
            )
        ):
            return f"그래프 미표시: bbox 매칭은 됐지만 적용된 체커보드 calibration에 stereo 외부 파라미터가 없습니다. 좌/우 체커보드 쌍을 최소 3개 이상 보정한 뒤 결과를 계산에 적용하세요. {warning_text}"
        if any(("waiting for RTK T" in warning for warning in warnings)):
            return f"그래프 미표시: bbox 매칭은 됐지만 좌/우 카메라 translation을 만들 RTK/GPS baseline이 아직 없습니다. {warning_text}"
        if any(("triangulation error" in warning for warning in warnings)):
            return f"그래프 미표시: triangulation 계산 중 오류가 발생했습니다. {warning_text}"
        if warning_text:
            return f"그래프 미표시: 3D ray triangulation 결과가 아직 없습니다. {warning_text}"
        return "그래프 미표시: bbox 매칭은 됐지만 3D ray triangulation 결과가 아직 없습니다."

    def _format_distance_label_text(self, text: str, detail: str | None = None) -> str:
        if not detail:
            return text
        return f"<div style='line-height: 1.1;'>{html_lib.escape(text)}<br><span style='font-size: 9px; font-weight: 400;'>{html_lib.escape(detail)}</span></div>"

    def _format_quality_point(self, point: tuple[float, float, float] | None) -> str:
        if point is None:
            return "--"
        return f"Right={point[0]:+.3f} m, Down={point[1]:+.3f} m, Forward={point[2]:+.3f} m"

    def _format_attitude_timing_age(self, value_s: float | None) -> str:
        return "--" if value_s is None else f"{value_s:.3f} s"

    def _attitude_timing_health_level(self, status: str) -> str:
        return {
            "FRESH": "good",
            "STALE": "warn",
            "VERY_STALE": "bad",
            "WAITING": "warn",
            "NOT_USED": "idle",
        }.get(status, "idle")

    def _format_attitude_timing_text(
        self,
        *,
        status: str,
        left_age_s: float | None,
        right_age_s: float | None,
        pair_delta_s: float | None,
        baseline_source: str,
        baseline_m: float,
    ) -> str:
        baseline_text = f"Baseline {baseline_source}:{baseline_m:.3f} m"
        if status == "NOT_USED":
            return f"Attitude timing [N/A] | {baseline_text} | PIXHAWK ATTITUDE NOT USED (checkerboard)"
        return f"Attitude timing [{status}] | {baseline_text} | Left age {self.actions.panel__format_attitude_timing_age.get(None)(left_age_s)} | Right age {self.actions.panel__format_attitude_timing_age.get(None)(right_age_s)} | Pair Δt {self.actions.panel__format_attitude_timing_age.get(None)(pair_delta_s)} | display-only; measurement continues"

    def _clear_triangulation_quality(self, reason: str) -> None:
        if not self.inputs.triangulation_quality_status_label.exists():
            return
        self.inputs.triangulation_quality_status_label.get().setText(reason)
        self.inputs.triangulation_quality_gap_label.get().setText("gap: --")
        self.inputs.triangulation_quality_distance_label.get().setText("distance: --")
        self.inputs.triangulation_quality_mode_label.get().setText("mode: --")
        self.inputs._set_health_label.get()(
            self.inputs.triangulation_quality_attitude_label.get(),
            "Attitude timing: --",
            "idle",
        )
        self.inputs.triangulation_quality_raw_point_label.get().setText(
            "Raw baseline_center_frd point: --"
        )
        self.inputs.triangulation_quality_plot.get().set_quality(None)

    def _update_triangulation_quality(
        self,
        *,
        triangulated: RayTriangulationResult,
        method_text: str,
        display_distance_m: float,
        display_baseline_source: str,
        display_baseline_m: float,
        public_point_m: tuple[float, float, float] | None,
        source_point_left_rdf_m: tuple[float, float, float] | None,
        attitude_timing_status: str,
        left_attitude_age_s: float | None,
        right_attitude_age_s: float | None,
        attitude_pair_delta_s: float | None,
    ) -> None:
        if not self.inputs.triangulation_quality_status_label.exists():
            return
        self.inputs.triangulation_quality_status_label.get().setText(
            "표시 전용: 현재 좌/우 bbox ray의 최단거리 gap과 3D midpoint를 보여줍니다."
        )
        self.inputs.triangulation_quality_gap_label.get().setText(
            f"gap: {triangulated.closest_gap_m:.3f} m"
        )
        self.inputs.triangulation_quality_distance_label.get().setText(
            f"distance: {display_distance_m:.3f} m"
        )
        self.inputs.triangulation_quality_mode_label.get().setText(
            f"mode: {method_text}"
        )
        self.inputs._set_health_label.get()(
            self.inputs.triangulation_quality_attitude_label.get(),
            self.actions.panel__format_attitude_timing_text.get(None)(
                status=attitude_timing_status,
                left_age_s=left_attitude_age_s,
                right_age_s=right_attitude_age_s,
                pair_delta_s=attitude_pair_delta_s,
                baseline_source=display_baseline_source,
                baseline_m=display_baseline_m,
            ),
            self.actions.panel__attitude_timing_health_level.get(None)(
                attitude_timing_status
            ),
        )
        self.inputs.triangulation_quality_raw_point_label.get().setText(
            f"Raw baseline_center_frd point [F/R/D]: {self.actions.panel__format_quality_point.get(None)(public_point_m)} | origin=baseline midpoint | source left_camera_rdf [R/D/F]: {self.actions.panel__format_quality_point.get(None)(source_point_left_rdf_m)}"
        )
        self.inputs.triangulation_quality_plot.get().set_quality(
            {
                "closest_gap_m": triangulated.closest_gap_m,
                "distance_m": display_distance_m,
                "method": method_text,
                "scene": triangulation_3d_scene_points(
                    triangulated, baseline_m=display_baseline_m
                ),
            }
        )

    def _set_distance_text(
        self, text: str, *, quality: str = "unknown", detail: str | None = None
    ) -> None:
        self.state.last_distance_status = text
        self.state.last_distance_quality = quality
        if self.inputs.distance_plot.exists():
            self.inputs.distance_plot.get().set_empty_message(detail or text)
        if detail:
            self.inputs.distance_label.get().setTextFormat(QtCore.Qt.RichText)
        else:
            self.inputs.distance_label.get().setTextFormat(QtCore.Qt.PlainText)
        self.inputs.distance_label.get().setText(
            self.actions.panel__format_distance_label_text.get(None)(text, detail)
        )
        style = self.actions.panel__distance_style.get(None)(quality)
        self.inputs.distance_label.get().setStyleSheet(
            f"font-size: 14px; font-weight: 700; padding: 8px 10px; border: 1px solid {PANEL_BORDER}; border-radius: 0px; "
            + style
        )
        self.inputs._refresh_operation_header.get()()

    def _distance_style(self, quality: str) -> str:
        if quality == "GOOD":
            return f"background: {PANEL_ALT_BG}; color: {SUCCESS_TEXT}; border-color: {SUCCESS};"
        if quality in {"DEGRADED", "WARNING", "WARN", "ESTIMATED"}:
            return f"background: {PANEL_ALT_BG}; color: {WARNING_TEXT}; border-color: {WARNING};"
        if quality in {"BAD", "ERROR"}:
            return f"background: {PANEL_ALT_BG}; color: {ERROR_TEXT}; border-color: {ERROR};"
        return f"background: {PANEL_ALT_BG}; color: {TEXT_MUTED}; border-color: {PANEL_BORDER};"
