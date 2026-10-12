"""Explicit composition root; no service receives the complete composition."""

from __future__ import annotations
from dataclasses import dataclass
from Workstation.measurement.boundary import MeasurementBoundary
from Workstation.measurement.methods import (
    MethodSelectionService,
    MethodSelectionInputs,
    MethodSelectionActions,
)
from Workstation.measurement.capture import (
    PairCaptureService,
    PairCaptureInputs,
    PairCaptureActions,
)
from Workstation.measurement.geometry import (
    GeometryInputAdapter,
    GeometryInputs,
    GeometryActions,
)
from Workstation.measurement.planning import (
    MethodPlanningService,
    MethodPlanningInputs,
    MethodPlanningActions,
)
from Workstation.measurement.runtime import (
    WorkerRuntimeService,
    WorkerRuntimeInputs,
    WorkerRuntimeActions,
)
from Workstation.measurement.commit import (
    ResultCommitService,
    ResultCommitInputs,
    ResultCommitActions,
)
from Workstation.measurement.projection import (
    ResultProjectionService,
    ResultProjectionInputs,
    ResultProjectionActions,
)
from Workstation.measurement.presentation import (
    MeasurementPresentationService,
    MeasurementPresentationInputs,
    MeasurementPresentationActions,
)


@dataclass(frozen=True)
class MeasurementServices:
    methods: MethodSelectionService
    capture: PairCaptureService
    geometry: GeometryInputAdapter
    planning: MethodPlanningService
    runtime: WorkerRuntimeService
    commit: ResultCommitService
    projection: ResultProjectionService
    presentation: MeasurementPresentationService


def compose_measurement(boundary: MeasurementBoundary) -> MeasurementServices:
    state = boundary.state
    methods = MethodSelectionService(
        state=state.methods,
        inputs=MethodSelectionInputs(
            _measurement_id=boundary.value("_measurement_id"),
            _measurement_package=boundary.value("_measurement_package"),
            _measurement_run_metadata=boundary.value("_measurement_run_metadata"),
            settings=boundary.value("settings"),
            _workstation_setting=boundary.value("_workstation_setting"),
            optical=boundary.optical_inputs(),
            active_intrinsics=boundary.value("active_intrinsics"),
            _active_intrinsics_profile_path=boundary.value(
                "_active_intrinsics_profile_path"
            ),
            checkerboard_extrinsics_candidate_profile=boundary.value(
                "checkerboard_extrinsics_candidate_profile"
            ),
            latest_rtk_baseline=boundary.value("latest_rtk_baseline"),
            gnss_operation_gate_active=boundary.value("gnss_operation_gate_active"),
            gnss_operation_manager=boundary.value("gnss_operation_manager"),
            checkerboard_extrinsics_profile=boundary.value(
                "checkerboard_extrinsics_profile"
            ),
            optical_pose_extrinsics_profile=boundary.value(
                "optical_pose_extrinsics_profile"
            ),
            _measurement_package_state=boundary.value("_measurement_package_state"),
            _measurement_package_detection_started=boundary.value(
                "_measurement_package_detection_started"
            ),
            append_workstation_run_event=boundary.value("append_workstation_run_event"),
            latest_extrinsics=boundary.value("latest_extrinsics"),
        ),
        actions=MethodSelectionActions(
            _configured_time_basis=lambda *a, **kw: capture._configured_time_basis(
                *a, **kw
            ),
            _cached_measurement_attitude_states=lambda *a, **kw: capture._cached_measurement_attitude_states(
                *a, **kw
            ),
            _cached_measurement_gps_states=lambda *a, **kw: capture._cached_measurement_gps_states(
                *a, **kw
            ),
            panel__gps_baseline_usable_for_translation=boundary.optional_action(
                "_gps_baseline_usable_for_translation",
                lambda *a, **kw: geometry._gps_baseline_usable_for_translation(
                    *a, **kw
                ),
            ),
            _gnss_readiness_config=lambda *a, **kw: capture._gnss_readiness_config(
                *a, **kw
            ),
            panel__measurement_baseline_yaw_reverse=boundary.optional_action(
                "_measurement_baseline_yaw_reverse",
                lambda *a, **kw: methods._measurement_baseline_yaw_reverse(*a, **kw),
            ),
            _baseline_yaw_selection_preview=lambda *a, **kw: geometry._baseline_yaw_selection_preview(
                *a, **kw
            ),
            _geometry_evidence_sha256=lambda *a, **kw: geometry._geometry_evidence_sha256(
                *a, **kw
            ),
        ),
        capture_state=state.capture,
        commit_state=state.commit,
        geometry_state=state.geometry,
    )
    capture = PairCaptureService(
        state=state.capture,
        inputs=PairCaptureInputs(
            camera_pane=boundary.camera_pane,
            qt_parent=boundary.qt_parent(),
            yolo_gate_time_basis_combo=boundary.value("yolo_gate_time_basis_combo"),
            settings=boundary.value("settings"),
            _workstation_setting=boundary.value("_workstation_setting"),
            _measurement_id=boundary.value("_measurement_id"),
            camera_sync_status=boundary.value("camera_sync_status"),
            frame_timing_status=boundary.value("frame_timing_status"),
            _yolo_pair_max_delta_s=boundary.value("_yolo_pair_max_delta_s"),
            _yolo_vertical_max_delta_px=boundary.value("_yolo_vertical_max_delta_px"),
            optical_target_distance_spin_value=boundary.widget_value(
                "optical_target_distance_spin", "value"
            ),
            _decorate_clock_epoch_metadata=boundary.value(
                "_decorate_clock_epoch_metadata"
            ),
            latest_left=boundary.value("latest_left", writable=True),
            latest_right=boundary.value("latest_right", writable=True),
            detection_buffers=boundary.value("detection_buffers"),
            gnss_operation_manager=boundary.value("gnss_operation_manager"),
            gnss_fixed_base_profile=boundary.value("gnss_fixed_base_profile"),
            latest_rtcm_reference_station=boundary.value(
                "latest_rtcm_reference_station"
            ),
            latest_extrinsics=boundary.value("latest_extrinsics"),
            _measurement_started_monotonic_s=boundary.value(
                "_measurement_started_monotonic_s"
            ),
            _selected_baseline_gps_states=boundary.value(
                "_selected_baseline_gps_states"
            ),
            _pixhawk_gnss_source=boundary.value("_pixhawk_gnss_source"),
            attitude_buffers=boundary.value("attitude_buffers"),
            _persist_native_detection_pair=boundary.value(
                "_persist_native_detection_pair"
            ),
        ),
        actions=PairCaptureActions(
            freeze_geometry_inputs=lambda frozen: geometry.freeze_geometry_inputs(
                frozen
            ),
            _measurement_method_state=lambda *a, **kw: methods._measurement_method_state(
                *a, **kw
            ),
            panel__refresh_distance=boundary.optional_action(
                "_refresh_distance",
                lambda *a, **kw: presentation._refresh_distance(*a, **kw),
            ),
            panel__refresh_optical_alignment_view=boundary.optional_action(
                "_refresh_optical_alignment_view",
                lambda *a, **kw: presentation._refresh_optical_alignment_view(*a, **kw),
            ),
            panel__optical_alignment_expected_dy_px=boundary.optional_action(
                "_optical_alignment_expected_dy_px",
                lambda *a, **kw: geometry._optical_alignment_expected_dy_px(*a, **kw),
            ),
            panel__optical_alignment_mean_focal_px=boundary.optional_action(
                "_optical_alignment_mean_focal_px",
                lambda *a, **kw: geometry._optical_alignment_mean_focal_px(*a, **kw),
            ),
            _measurement_registry=lambda *a, **kw: commit._measurement_registry(
                *a, **kw
            ),
        ),
        commit_state=state.commit,
        presentation_state=state.presentation,
        runtime_state=state.runtime,
    )
    geometry = GeometryInputAdapter(
        state=state.geometry,
        inputs=GeometryInputs(
            checkerboard_extrinsics_profile=boundary.value(
                "checkerboard_extrinsics_profile"
            ),
            optical=boundary.optical_inputs(),
            _active_optical_pose_profile_hash=boundary.value(
                "_active_optical_pose_profile_hash"
            ),
            _selected_baseline_gps_states=boundary.value(
                "_selected_baseline_gps_states"
            ),
            latest_gps=boundary.value("latest_gps"),
            active_intrinsics=boundary.value("active_intrinsics"),
            optical_pose_extrinsics_profile=boundary.value(
                "optical_pose_extrinsics_profile"
            ),
            _active_intrinsics_source_zoom_level=boundary.value(
                "_active_intrinsics_source_zoom_level"
            ),
            _active_intrinsics_profile_path=boundary.value(
                "_active_intrinsics_profile_path"
            ),
            _distance_mode=boundary.value("_distance_mode"),
            latest_rtk_baseline=boundary.value("latest_rtk_baseline"),
            gnss_operation_manager=boundary.value("gnss_operation_manager"),
            settings=boundary.value("settings"),
            _pixhawk_gnss_source=boundary.value("_pixhawk_gnss_source"),
            latest_extrinsics=boundary.value("latest_extrinsics"),
            baseline_scale_m=boundary.widget_value("baseline_spin", "value"),
            last_calibration_result=boundary.value("last_calibration_result"),
            optical_expected_dy_combo_currentData=boundary.widget_value(
                "optical_expected_dy_combo", "currentData"
            ),
            optical_expected_dy_spin_value=boundary.widget_value(
                "optical_expected_dy_spin", "value"
            ),
            active_zoom=boundary.widget_value("zoom_level_spin", "value"),
            _active_extrinsics_source_zoom_level=boundary.value(
                "_active_extrinsics_source_zoom_level"
            ),
            _yolo_pair_max_delta_s=boundary.value("_yolo_pair_max_delta_s"),
            _yolo_vertical_max_delta_px=boundary.value("_yolo_vertical_max_delta_px"),
            focal_length_px=boundary.widget_value("focal_spin", "value"),
            manual_focal_override=boundary.widget_value(
                "manual_focal_checkbox", "isChecked"
            ),
            _workstation_setting=boundary.value("_workstation_setting"),
            gnss_fixed_base_profile=boundary.value("gnss_fixed_base_profile"),
            latest_baseline_vector=boundary.value("latest_baseline_vector"),
            _gnss_base_rtcm_target_sides=boundary.value("_gnss_base_rtcm_target_sides"),
            _selected_rover_gps_state=boundary.value("_selected_rover_gps_state"),
            _fmt_record_float=boundary.value("_fmt_record_float"),
            workstation_optical_pose_candidate_poses=boundary.value(
                "workstation_optical_pose_candidate_poses"
            ),
        ),
        actions=GeometryActions(
            _measurement_method_statuses=lambda: methods._measurement_method_statuses(),
            _cached_measurement_attitude_states=lambda *a, **kw: capture._cached_measurement_attitude_states(
                *a, **kw
            ),
            _measurement_baseline_yaw_reverse=lambda *a, **kw: methods._measurement_baseline_yaw_reverse(
                *a, **kw
            ),
            _measurement_candidate=lambda *a, **kw: methods._measurement_candidate(
                *a, **kw
            ),
            _candidate_observed_dofs=lambda *a, **kw: methods._candidate_observed_dofs(
                *a, **kw
            ),
            panel__checkerboard_translation_right_in_left_m=boundary.optional_action(
                "_checkerboard_translation_right_in_left_m",
                lambda *a, **kw: geometry._checkerboard_translation_right_in_left_m(
                    *a, **kw
                ),
            ),
            panel__gps_baseline_usable_for_translation=boundary.optional_action(
                "_gps_baseline_usable_for_translation",
                lambda *a, **kw: geometry._gps_baseline_usable_for_translation(
                    *a, **kw
                ),
            ),
            panel__optical_alignment_intrinsics_result=boundary.optional_action(
                "_optical_alignment_intrinsics_result",
                lambda *a, **kw: geometry._optical_alignment_intrinsics_result(
                    *a, **kw
                ),
            ),
            _select_measurement_method_for_attempt=lambda *a, **kw: methods._select_measurement_method_for_attempt(
                *a, **kw
            ),
            _freeze_measurement_pair_inputs=lambda *a, **kw: capture._freeze_measurement_pair_inputs(
                *a, **kw
            ),
        ),
        capture_state=state.capture,
        methods_state=state.methods,
    )
    planning = MethodPlanningService(
        state=state.planning,
        inputs=MethodPlanningInputs(),
        actions=MethodPlanningActions(
            _freeze_measurement_pair_inputs=lambda *a, **kw: capture._freeze_measurement_pair_inputs(
                *a, **kw
            ),
            _measurement_method_statuses=lambda *a, **kw: methods._measurement_method_statuses(
                *a, **kw
            ),
            _build_measurement_attempt=lambda *a, **kw: geometry._build_measurement_attempt(
                *a, **kw
            ),
        ),
    )
    runtime = WorkerRuntimeService(
        state=state.runtime,
        inputs=WorkerRuntimeInputs(
            qt_parent=boundary.qt_parent(),
            append_workstation_data=boundary.value("append_workstation_data"),
            _log=boundary.value("_log"),
            _shutdown_in_progress=boundary.value("_shutdown_in_progress"),
            _measurement_id=boundary.value("_measurement_id"),
            distance_recording_enabled=boundary.value("distance_recording_enabled"),
        ),
        actions=WorkerRuntimeActions(
            _retry_pending_primary_measurements=lambda *a, **kw: commit._retry_pending_primary_measurements(
                *a, **kw
            ),
            _retry_pending_method_bundles=lambda *a, **kw: commit._retry_pending_method_bundles(
                *a, **kw
            ),
            _measurement_registry=lambda *a, **kw: commit._measurement_registry(
                *a, **kw
            ),
            _terminalize_shutdown_pending_retries=lambda *a, **kw: commit._terminalize_shutdown_pending_retries(
                *a, **kw
            ),
            _sync_persistence_pending_status=lambda *a, **kw: commit._sync_persistence_pending_status(
                *a, **kw
            ),
            _release_detection_pair_reservation=lambda *a, **kw: capture._release_detection_pair_reservation(
                *a, **kw
            ),
            _plan_method_evaluations=lambda *a, **kw: planning._plan_method_evaluations(
                *a, **kw
            ),
            panel__set_distance_text=boundary.optional_action(
                "_set_distance_text",
                lambda *a, **kw: presentation._set_distance_text(*a, **kw),
            ),
            _selected_method_id_for=lambda *a, **kw: commit._selected_method_id_for(
                *a, **kw
            ),
            _commit_primary_method_outcome=lambda *a, **kw: commit._commit_primary_method_outcome(
                *a, **kw
            ),
            _primary_outcome_key=lambda *a, **kw: projection._primary_outcome_key(
                *a, **kw
            ),
            _display_primary_method_outcome=lambda *a, **kw: presentation._display_primary_method_outcome(
                *a, **kw
            ),
            _commit_method_evaluation_bundle=lambda *a, **kw: commit._commit_method_evaluation_bundle(
                *a, **kw
            ),
            _method_bundle_with_primary_failure=lambda *a, **kw: projection._method_bundle_with_primary_failure(
                *a, **kw
            ),
        ),
        capture_state=state.capture,
        commit_state=state.commit,
        presentation_state=state.presentation,
    )
    commit = ResultCommitService(
        state=state.commit,
        inputs=ResultCommitInputs(
            append_workstation_data=boundary.value("append_workstation_data"),
            _log=boundary.value("_log"),
            release_primary_retry=boundary.release_primary_retry(),
            persist_compact_primary=boundary.persist_compact_primary(),
            measurement_registry=boundary.value("measurement_registry"),
            _record_blocked_measurement_pair=boundary.value(
                "_record_blocked_measurement_pair"
            ),
            distance_recording_enabled=boundary.value("distance_recording_enabled"),
            _measurement_session_hierarchy_enabled=boundary.value(
                "_measurement_session_hierarchy_enabled"
            ),
            _measurement_id=boundary.value("_measurement_id"),
            _measurement_run_generation=boundary.value("_measurement_run_generation"),
            _measurement_run_state=boundary.value("_measurement_run_state"),
            _commit_primary_distance_outcome=boundary.value(
                "_commit_primary_distance_outcome"
            ),
            _append_distance_history=boundary.value("_append_distance_history"),
            method_evaluation_bundle_rows=boundary.value(
                "method_evaluation_bundle_rows"
            ),
            method_result_rows=boundary.value("method_result_rows"),
            _recording_pair_row_index=boundary.value("_recording_pair_row_index"),
            _upsert_pair_recording_row=boundary.value("_upsert_pair_recording_row"),
        ),
        actions=ResultCommitActions(
            _pair_timing_decision=lambda *a, **kw: capture._pair_timing_decision(
                *a, **kw
            ),
            _measurement_method_statuses=lambda *a, **kw: methods._measurement_method_statuses(
                *a, **kw
            ),
            _measurement_fallback_order=lambda *a, **kw: methods._measurement_fallback_order(
                *a, **kw
            ),
            _primary_outcome_key=lambda *a, **kw: projection._primary_outcome_key(
                *a, **kw
            ),
            _failed_primary_outcome=lambda *a, **kw: projection._failed_primary_outcome(
                *a, **kw
            ),
            _method_history_payload=lambda *a, **kw: projection._method_history_payload(
                *a, **kw
            ),
            _ensure_method_evaluation_timer=lambda *a, **kw: runtime._ensure_method_evaluation_timer(
                *a, **kw
            ),
            _stop_method_evaluation_timer=lambda *a, **kw: runtime._stop_method_evaluation_timer(
                *a, **kw
            ),
            _method_bundle_key=lambda *a, **kw: projection._method_bundle_key(*a, **kw),
            _method_result_row=lambda *a, **kw: projection._method_result_row(*a, **kw),
            _bundle_flat_fields=lambda *a, **kw: projection._bundle_flat_fields(
                *a, **kw
            ),
            _optical_relative_rotation_fields=lambda *a, **kw: geometry._optical_relative_rotation_fields(
                *a, **kw
            ),
            _display_primary_method_outcome=lambda *a, **kw: presentation._display_primary_method_outcome(
                *a, **kw
            ),
        ),
        methods_state=state.methods,
        presentation_state=state.presentation,
        runtime_state=state.runtime,
    )
    projection = ResultProjectionService(
        state=state.projection,
        inputs=ResultProjectionInputs(),
        actions=ResultProjectionActions(),
    )
    presentation = MeasurementPresentationService(
        state=state.presentation,
        inputs=MeasurementPresentationInputs(
            _selected_baseline_gps_states=boundary.value(
                "_selected_baseline_gps_states"
            ),
            latest_rtk_baseline=boundary.value("latest_rtk_baseline"),
            _refresh_p1_body_frd_baseline=boundary.value(
                "_refresh_p1_body_frd_baseline"
            ),
            rtk_fixed_status_label=boundary.value("rtk_fixed_status_label"),
            rtk_baseline_label=boundary.value("rtk_baseline_label"),
            _refresh_operation_header=boundary.value("_refresh_operation_header"),
            _refresh_alignment_workflow_status=boundary.value(
                "_refresh_alignment_workflow_status"
            ),
            latest_p1_body_frd_baseline=boundary.value("latest_p1_body_frd_baseline"),
            rtk_summary_labels=boundary.value("rtk_summary_labels"),
            _set_health_label=boundary.value("_set_health_label"),
            baseline_attitude_labels=boundary.value("baseline_attitude_labels"),
            _set_label_text_if_changed=boundary.value("_set_label_text_if_changed"),
            rtk_alignment_plot=boundary.value("rtk_alignment_plot"),
            latest_extrinsics=boundary.value("latest_extrinsics"),
            rtk_alignment_detail_label=boundary.value("rtk_alignment_detail_label"),
            optical_alignment_labels=boundary.value("optical_alignment_labels"),
            _matched_detection_pair=boundary.value("_matched_detection_pair"),
            optical_alignment_status_label=boundary.value(
                "optical_alignment_status_label"
            ),
            optical_target_distance_spin_value=boundary.widget_value(
                "optical_target_distance_spin", "value"
            ),
            _distance_mode_label=boundary.value("_distance_mode_label"),
            _rtsp_urls_are_same_stream=boundary.value("_rtsp_urls_are_same_stream"),
            _measurement_display_mode=boundary.value("_measurement_display_mode"),
            gnss_operation_gate_active=boundary.value("gnss_operation_gate_active"),
            gnss_operation_manager=boundary.value("gnss_operation_manager"),
            _record_unresolved_pair=boundary.value("_record_unresolved_pair"),
            _triangulation_extrinsics_source_label=boundary.value(
                "_triangulation_extrinsics_source_label"
            ),
            _append_distance_history=boundary.value("_append_distance_history"),
            left_pane=boundary.value("left_pane"),
            right_pane=boundary.value("right_pane"),
            _update_runtime_camera_stats_labels=boundary.value(
                "_update_runtime_camera_stats_labels"
            ),
            selected_runtime_frame_metadata=boundary.value(
                "selected_runtime_frame_metadata"
            ),
            _yolo_match_timestamp_s=boundary.value("_yolo_match_timestamp_s"),
            _effective_yolo_time_basis=boundary.value("_effective_yolo_time_basis"),
            _yolo_time_basis=boundary.value("_yolo_time_basis"),
            _selected_pair_sync_summary=boundary.value("_selected_pair_sync_summary"),
            detection_buffers=boundary.value("detection_buffers"),
            camera_stream_stats=boundary.value("camera_stream_stats"),
            _yolo_time_delta_gate_enabled=boundary.value(
                "_yolo_time_delta_gate_enabled"
            ),
            _yolo_pair_max_delta_s=boundary.value("_yolo_pair_max_delta_s"),
            triangulation_quality_status_label=boundary.value(
                "triangulation_quality_status_label"
            ),
            triangulation_quality_gap_label=boundary.value(
                "triangulation_quality_gap_label"
            ),
            triangulation_quality_distance_label=boundary.value(
                "triangulation_quality_distance_label"
            ),
            triangulation_quality_mode_label=boundary.value(
                "triangulation_quality_mode_label"
            ),
            triangulation_quality_attitude_label=boundary.value(
                "triangulation_quality_attitude_label"
            ),
            triangulation_quality_raw_point_label=boundary.value(
                "triangulation_quality_raw_point_label"
            ),
            triangulation_quality_plot=boundary.value("triangulation_quality_plot"),
            distance_plot=boundary.value("distance_plot"),
            distance_label=boundary.value("distance_label"),
        ),
        actions=MeasurementPresentationActions(
            panel__rtk_fixed_status_text=boundary.optional_action(
                "_rtk_fixed_status_text",
                lambda *a, **kw: geometry._rtk_fixed_status_text(*a, **kw),
            ),
            panel__set_rtk_summary_chip=boundary.optional_action(
                "_set_rtk_summary_chip",
                lambda *a, **kw: presentation._set_rtk_summary_chip(*a, **kw),
            ),
            panel__refresh_rtk_alignment_view=boundary.optional_action(
                "_refresh_rtk_alignment_view",
                lambda *a, **kw: presentation._refresh_rtk_alignment_view(*a, **kw),
            ),
            panel__gps_baseline_usable_for_translation=boundary.optional_action(
                "_gps_baseline_usable_for_translation",
                lambda *a, **kw: geometry._gps_baseline_usable_for_translation(
                    *a, **kw
                ),
            ),
            panel__baseline_forward_angle_level=boundary.optional_action(
                "_baseline_forward_angle_level",
                lambda *a, **kw: geometry._baseline_forward_angle_level(*a, **kw),
            ),
            panel__baseline_forward_geometry_from_camera_translation=boundary.optional_action(
                "_baseline_forward_geometry_from_camera_translation",
                lambda *a, **kw: geometry._baseline_forward_geometry_from_camera_translation(
                    *a, **kw
                ),
            ),
            panel__alignment_view_camera_translation_right_in_left_m=boundary.optional_action(
                "_alignment_view_camera_translation_right_in_left_m",
                lambda *a, **kw: geometry._alignment_view_camera_translation_right_in_left_m(
                    *a, **kw
                ),
            ),
            panel__baseline_camera_translation_in_side_rdf_m=boundary.optional_action(
                "_baseline_camera_translation_in_side_rdf_m",
                lambda *a, **kw: geometry._baseline_camera_translation_in_side_rdf_m(
                    *a, **kw
                ),
            ),
            panel__set_baseline_forward_metric_labels=boundary.optional_action(
                "_set_baseline_forward_metric_labels",
                lambda *a, **kw: presentation._set_baseline_forward_metric_labels(
                    *a, **kw
                ),
            ),
            panel__baseline_forward_geometry_text=boundary.optional_action(
                "_baseline_forward_geometry_text",
                lambda *a, **kw: presentation._baseline_forward_geometry_text(*a, **kw),
            ),
            panel__optical_alignment_expected_dy_px=boundary.optional_action(
                "_optical_alignment_expected_dy_px",
                lambda *a, **kw: geometry._optical_alignment_expected_dy_px(*a, **kw),
            ),
            panel__format_optical_px=boundary.optional_action(
                "_format_optical_px",
                lambda *a, **kw: presentation._format_optical_px(*a, **kw),
            ),
            panel__optical_alignment_mean_focal_px=boundary.optional_action(
                "_optical_alignment_mean_focal_px",
                lambda *a, **kw: geometry._optical_alignment_mean_focal_px(*a, **kw),
            ),
            panel__build_measurement_attempt=boundary.optional_action(
                "_build_measurement_attempt",
                lambda *a, **kw: geometry._build_measurement_attempt(*a, **kw),
            ),
            _preview_measurement_result=lambda *a, **kw: capture._preview_measurement_result(
                *a, **kw
            ),
            panel__optical_alignment_level_from_mrad=boundary.optional_action(
                "_optical_alignment_level_from_mrad",
                lambda *a, **kw: geometry._optical_alignment_level_from_mrad(*a, **kw),
            ),
            panel__optical_alignment_overall_level=boundary.optional_action(
                "_optical_alignment_overall_level",
                lambda *a, **kw: geometry._optical_alignment_overall_level(*a, **kw),
            ),
            panel__format_optical_mrad=boundary.optional_action(
                "_format_optical_mrad",
                lambda *a, **kw: presentation._format_optical_mrad(*a, **kw),
            ),
            panel__attitude_timing_health_level=boundary.optional_action(
                "_attitude_timing_health_level",
                lambda *a, **kw: presentation._attitude_timing_health_level(*a, **kw),
            ),
            panel__optical_alignment_level_text=boundary.optional_action(
                "_optical_alignment_level_text",
                lambda *a, **kw: presentation._optical_alignment_level_text(*a, **kw),
            ),
            panel__set_distance_text=boundary.optional_action(
                "_set_distance_text",
                lambda *a, **kw: presentation._set_distance_text(*a, **kw),
            ),
            panel__clear_measurement_pair_display=boundary.optional_action(
                "_clear_measurement_pair_display",
                lambda *a, **kw: presentation._clear_measurement_pair_display(*a, **kw),
            ),
            panel__clear_triangulation_quality=boundary.optional_action(
                "_clear_triangulation_quality",
                lambda *a, **kw: presentation._clear_triangulation_quality(*a, **kw),
            ),
            panel__detection_wait_status=boundary.optional_action(
                "_detection_wait_status",
                lambda *a, **kw: presentation._detection_wait_status(*a, **kw),
            ),
            panel__detection_wait_detail=boundary.optional_action(
                "_detection_wait_detail",
                lambda *a, **kw: presentation._detection_wait_detail(*a, **kw),
            ),
            panel__geometry_comparison_rows=boundary.optional_action(
                "_geometry_comparison_rows",
                lambda *a, **kw: geometry._geometry_comparison_rows(*a, **kw),
            ),
            _multi_method_measurement_active=lambda *a, **kw: runtime._multi_method_measurement_active(
                *a, **kw
            ),
            _select_measurement_method_for_attempt=lambda *a, **kw: methods._select_measurement_method_for_attempt(
                *a, **kw
            ),
            panel__show_measurement_pair=boundary.optional_action(
                "_show_measurement_pair",
                lambda *a, **kw: presentation._show_measurement_pair(*a, **kw),
            ),
            _submit_method_evaluations=lambda *a, **kw: runtime._submit_method_evaluations(
                *a, **kw
            ),
            _record_no_ready_measurement_pair=lambda *a, **kw: commit._record_no_ready_measurement_pair(
                *a, **kw
            ),
            panel__capture_measurement_gnss_contract=boundary.optional_action(
                "_capture_measurement_gnss_contract",
                lambda *a, **kw: geometry._capture_measurement_gnss_contract(*a, **kw),
            ),
            panel__stereo_geometry_wait_detail=boundary.optional_action(
                "_stereo_geometry_wait_detail",
                lambda *a, **kw: presentation._stereo_geometry_wait_detail(*a, **kw),
            ),
            panel__triangulation_wait_detail=boundary.optional_action(
                "_triangulation_wait_detail",
                lambda *a, **kw: presentation._triangulation_wait_detail(*a, **kw),
            ),
            panel__update_triangulation_quality=boundary.optional_action(
                "_update_triangulation_quality",
                lambda *a, **kw: presentation._update_triangulation_quality(*a, **kw),
            ),
            panel__clear_selected_runtime_measurement_pair=boundary.optional_action(
                "_clear_selected_runtime_measurement_pair",
                lambda *a, **kw: presentation._clear_selected_runtime_measurement_pair(
                    *a, **kw
                ),
            ),
            _pair_timing_decision=lambda *a, **kw: capture._pair_timing_decision(
                *a, **kw
            ),
            panel__format_attitude_timing_age=boundary.optional_action(
                "_format_attitude_timing_age",
                lambda *a, **kw: presentation._format_attitude_timing_age(*a, **kw),
            ),
            panel__format_attitude_timing_text=boundary.optional_action(
                "_format_attitude_timing_text",
                lambda *a, **kw: presentation._format_attitude_timing_text(*a, **kw),
            ),
            panel__format_quality_point=boundary.optional_action(
                "_format_quality_point",
                lambda *a, **kw: presentation._format_quality_point(*a, **kw),
            ),
            panel__format_distance_label_text=boundary.optional_action(
                "_format_distance_label_text",
                lambda *a, **kw: presentation._format_distance_label_text(*a, **kw),
            ),
            panel__distance_style=boundary.optional_action(
                "_distance_style",
                lambda *a, **kw: presentation._distance_style(*a, **kw),
            ),
        ),
        capture_state=state.capture,
        runtime_state=state.runtime,
    )
    commit._sync_persistence_pending_status()
    boundary.publish_legacy_state()
    return MeasurementServices(
        methods, capture, geometry, planning, runtime, commit, projection, presentation
    )
