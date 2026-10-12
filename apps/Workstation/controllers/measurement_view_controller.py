"""Compatibility entrypoints for the explicitly composed Measurement feature.

State and behavior live in Workstation.measurement services. This outer adapter
keeps existing panel bindings and small embedders stable without exposing their
panel to any service. Public points remain baseline-center FRD; optical values RDF.
"""

from __future__ import annotations
import time  # Existing embedders patch the shared clock module here.
from Workstation.measurement.boundary import (
    MeasurementBoundary,
    service_method,
    service_state,
)
from Workstation.measurement.composition import compose_measurement
from Workstation.measurement.methods import MethodSelectionService
from Workstation.measurement.capture import PairCaptureService
from Workstation.measurement.geometry import GeometryInputAdapter
from Workstation.measurement.planning import MethodPlanningService
from Workstation.measurement.runtime import WorkerRuntimeService
from Workstation.measurement.commit import ResultCommitService
from Workstation.measurement.projection import ResultProjectionService
from Workstation.measurement.presentation import MeasurementPresentationService

MEASUREMENT_VIEW_CONTROLLER_METHODS = [
    "workstation_measurement_methods_snapshot",
    "select_workstation_measurement_method",
    "configure_workstation_measurement_method_fill",
    "_measurement_method_statuses",
    "_select_measurement_method_for_attempt",
    "_record_no_ready_measurement_pair",
    "_update_detection",
    "_checkerboard_translation_right_in_left_m",
    "_active_baseline_m",
    "_gps_fix_ready_for_baseline",
    "_gps_baseline_usable_for_translation",
    "_rtk_fixed_status_text",
    "_refresh_rtk_baseline",
    "_set_rtk_summary_chip",
    "_baseline_forward_geometry_from_camera_translation",
    "_baseline_camera_translation_in_side_rdf_m",
    "_set_baseline_forward_metric_labels",
    "_baseline_forward_angle_level",
    "_baseline_forward_geometry_text",
    "_refresh_rtk_alignment_view",
    "_optical_alignment_intrinsics_result",
    "_optical_alignment_mean_focal_px",
    "_optical_alignment_expected_dy_px",
    "_optical_alignment_level_from_mrad",
    "_optical_alignment_overall_level",
    "_optical_alignment_level_text",
    "_format_optical_px",
    "_format_optical_mrad",
    "_refresh_optical_alignment_view",
    "_build_measurement_attempt",
    "_translation_right_in_left_m",
    "_latest_translation_right_in_left_m_for_diagnostics",
    "_alignment_view_camera_translation_right_in_left_m",
    "_capture_measurement_gnss_contract",
    "_measurement_gnss_record_metadata",
    "_refresh_distance",
    "_geometry_comparison_rows",
    "_show_measurement_pair",
    "_clear_measurement_pair_display",
    "_clear_selected_runtime_measurement_pair",
    "_detection_wait_detail",
    "_detection_wait_status",
    "_stereo_geometry_wait_detail",
    "_triangulation_wait_detail",
    "_format_distance_label_text",
    "_format_quality_point",
    "_format_attitude_timing_age",
    "_attitude_timing_health_level",
    "_format_attitude_timing_text",
    "_clear_triangulation_quality",
    "_update_triangulation_quality",
    "_set_distance_text",
    "_distance_style",
]


class MeasurementViewController:
    def __init__(self, panel: object) -> None:
        self.boundary = MeasurementBoundary(panel)
        self.services = compose_measurement(self.boundary)

    @property
    def panel(self) -> object:
        """Historical outer-edge host access; never supplied to a service."""
        return self.boundary._host

    _pending_record_size = service_method(
        "commit", "_pending_record_size", ResultCommitService
    )
    _sync_persistence_pending_status = service_method(
        "commit", "_sync_persistence_pending_status", ResultCommitService
    )
    _record_persistence_pending_overflow = service_method(
        "commit", "_record_persistence_pending_overflow", ResultCommitService
    )
    _release_recording_primary_retry = service_method(
        "commit", "_release_recording_primary_retry", ResultCommitService
    )
    _remove_pending_primary_measurement = service_method(
        "commit", "_remove_pending_primary_measurement", ResultCommitService
    )
    _retain_pending_primary_measurement = service_method(
        "commit", "_retain_pending_primary_measurement", ResultCommitService
    )
    _remove_pending_method_bundle = service_method(
        "commit", "_remove_pending_method_bundle", ResultCommitService
    )
    _retain_pending_method_bundle = service_method(
        "commit", "_retain_pending_method_bundle", ResultCommitService
    )
    _terminalize_shutdown_pending_retries = service_method(
        "commit", "_terminalize_shutdown_pending_retries", ResultCommitService
    )
    _ensure_method_evaluation_coordinator = service_method(
        "runtime", "_ensure_method_evaluation_coordinator", WorkerRuntimeService
    )
    measurement_worker_snapshot = service_method(
        "runtime", "measurement_worker_snapshot", WorkerRuntimeService
    )
    _record_method_worker_recovery = service_method(
        "runtime", "_record_method_worker_recovery", WorkerRuntimeService
    )
    restart_measurement_method_worker = service_method(
        "runtime", "restart_measurement_method_worker", WorkerRuntimeService
    )
    _measurement_registry = service_method(
        "commit", "_measurement_registry", ResultCommitService
    )
    _registry_is_current = service_method(
        "commit", "_registry_is_current", ResultCommitService
    )
    _selected_method_id_for = service_method(
        "commit", "_selected_method_id_for", ResultCommitService
    )
    measurement_calibration_snapshot = service_method(
        "methods", "measurement_calibration_snapshot", MethodSelectionService
    )
    measurement_alignment_rows = service_method(
        "geometry", "measurement_alignment_rows", GeometryInputAdapter
    )
    close_method_evaluation_workers = service_method(
        "runtime", "close_method_evaluation_workers", WorkerRuntimeService
    )
    _measurement_method_state = service_method(
        "methods", "_measurement_method_state", MethodSelectionService
    )
    measurement_method_config_for_new_measurement = service_method(
        "methods",
        "measurement_method_config_for_new_measurement",
        MethodSelectionService,
    )
    _measurement_fill_policy = service_method(
        "methods", "_measurement_fill_policy", MethodSelectionService
    )
    _measurement_fill_source = service_method(
        "methods", "_measurement_fill_source", MethodSelectionService
    )
    _measurement_live_readiness = service_method(
        "methods", "_measurement_live_readiness", MethodSelectionService
    )
    _candidate_observed_dofs = service_method(
        "methods", "_candidate_observed_dofs", MethodSelectionService
    )
    _geometry_evidence_sha256 = service_method(
        "geometry", "_geometry_evidence_sha256", GeometryInputAdapter
    )
    _operating_intrinsics_readiness = service_method(
        "methods", "_operating_intrinsics_readiness", MethodSelectionService
    )
    _measurement_baseline_yaw_reverse = service_method(
        "methods", "_measurement_baseline_yaw_reverse", MethodSelectionService
    )
    _baseline_yaw_selection_preview = service_method(
        "geometry", "_baseline_yaw_selection_preview", GeometryInputAdapter
    )
    _measurement_method_statuses = service_method(
        "methods", "_measurement_method_statuses", MethodSelectionService
    )
    _measurement_fallback_order = service_method(
        "methods", "_measurement_fallback_order", MethodSelectionService
    )
    _measurement_fill_policy_locked = service_method(
        "methods", "_measurement_fill_policy_locked", MethodSelectionService
    )
    _measurement_component_controls = service_method(
        "methods", "_measurement_component_controls", MethodSelectionService
    )
    configure_workstation_measurement_method_fill = service_method(
        "methods",
        "configure_workstation_measurement_method_fill",
        MethodSelectionService,
    )
    _set_selected_measurement_method = service_method(
        "methods", "_set_selected_measurement_method", MethodSelectionService
    )
    _clear_selected_measurement_method = service_method(
        "methods", "_clear_selected_measurement_method", MethodSelectionService
    )
    _select_measurement_method_for_attempt = service_method(
        "methods", "_select_measurement_method_for_attempt", MethodSelectionService
    )
    _measurement_method_input_snapshot = service_method(
        "methods", "_measurement_method_input_snapshot", MethodSelectionService
    )
    workstation_measurement_methods_snapshot = service_method(
        "methods", "workstation_measurement_methods_snapshot", MethodSelectionService
    )
    select_workstation_measurement_method = service_method(
        "methods", "select_workstation_measurement_method", MethodSelectionService
    )
    _measurement_candidate = service_method(
        "methods", "_measurement_candidate", MethodSelectionService
    )
    _measurement_intrinsics_result_for_method = service_method(
        "geometry", "_measurement_intrinsics_result_for_method", GeometryInputAdapter
    )
    _measurement_geometry_hash = service_method(
        "geometry", "_measurement_geometry_hash", GeometryInputAdapter
    )
    _record_no_ready_measurement_pair = service_method(
        "commit", "_record_no_ready_measurement_pair", ResultCommitService
    )
    _release_detection_pair_reservation = service_method(
        "capture", "_release_detection_pair_reservation", PairCaptureService
    )
    _exact_pair_reservation_supported = service_method(
        "capture", "_exact_pair_reservation_supported", PairCaptureService
    )
    _reserve_detection_pair = service_method(
        "capture", "_reserve_detection_pair", PairCaptureService
    )
    _ensure_detection_pair_coordinator = service_method(
        "capture", "_ensure_detection_pair_coordinator", PairCaptureService
    )
    _configured_time_basis = service_method(
        "capture", "_configured_time_basis", PairCaptureService
    )
    requested_time_basis = service_method(
        "capture", "requested_time_basis", PairCaptureService
    )
    _rtcp_pairing_health = service_method(
        "capture", "_rtcp_pairing_health", PairCaptureService
    )
    _pairing_policy_snapshot = service_method(
        "capture", "_pairing_policy_snapshot", PairCaptureService
    )
    _poll_detection_pairs = service_method(
        "capture", "_poll_detection_pairs", PairCaptureService
    )
    reset_detection_pair_coordinator = service_method(
        "capture", "reset_detection_pair_coordinator", PairCaptureService
    )
    begin_pairing_generation = service_method(
        "capture", "begin_pairing_generation", PairCaptureService
    )
    _ensure_preview_measurement_coordinator = service_method(
        "capture", "_ensure_preview_measurement_coordinator", PairCaptureService
    )
    _poll_preview_measurements = service_method(
        "capture", "_poll_preview_measurements", PairCaptureService
    )
    _preview_optical_payload = service_method(
        "capture", "_preview_optical_payload", PairCaptureService
    )
    _preview_measurement_result = service_method(
        "capture", "_preview_measurement_result", PairCaptureService
    )
    _update_detection = service_method(
        "capture", "_update_detection", PairCaptureService
    )
    _checkerboard_translation_right_in_left_m = service_method(
        "geometry", "_checkerboard_translation_right_in_left_m", GeometryInputAdapter
    )
    _active_baseline_m = service_method(
        "geometry", "_active_baseline_m", GeometryInputAdapter
    )
    _gps_fix_ready_for_baseline = service_method(
        "geometry", "_gps_fix_ready_for_baseline", GeometryInputAdapter
    )
    _gps_pair_readiness = service_method(
        "geometry", "_gps_pair_readiness", GeometryInputAdapter
    )
    _gps_baseline_usable_for_translation = service_method(
        "geometry", "_gps_baseline_usable_for_translation", GeometryInputAdapter
    )
    _rtk_fixed_status_text = service_method(
        "geometry", "_rtk_fixed_status_text", GeometryInputAdapter
    )
    _refresh_rtk_baseline = service_method(
        "presentation", "_refresh_rtk_baseline", MeasurementPresentationService
    )
    _set_rtk_summary_chip = service_method(
        "presentation", "_set_rtk_summary_chip", MeasurementPresentationService
    )
    _baseline_forward_geometry_from_camera_translation = service_method(
        "geometry",
        "_baseline_forward_geometry_from_camera_translation",
        GeometryInputAdapter,
    )
    _baseline_camera_translation_in_side_rdf_m = service_method(
        "geometry", "_baseline_camera_translation_in_side_rdf_m", GeometryInputAdapter
    )
    _set_baseline_forward_metric_labels = service_method(
        "presentation",
        "_set_baseline_forward_metric_labels",
        MeasurementPresentationService,
    )
    _baseline_forward_angle_level = service_method(
        "geometry", "_baseline_forward_angle_level", GeometryInputAdapter
    )
    _baseline_forward_geometry_text = service_method(
        "presentation",
        "_baseline_forward_geometry_text",
        MeasurementPresentationService,
    )
    _refresh_rtk_alignment_view = service_method(
        "presentation", "_refresh_rtk_alignment_view", MeasurementPresentationService
    )
    _optical_alignment_intrinsics_result = service_method(
        "geometry", "_optical_alignment_intrinsics_result", GeometryInputAdapter
    )
    _optical_alignment_mean_focal_px = service_method(
        "geometry", "_optical_alignment_mean_focal_px", GeometryInputAdapter
    )
    _optical_alignment_expected_dy_px = service_method(
        "geometry", "_optical_alignment_expected_dy_px", GeometryInputAdapter
    )
    _optical_alignment_level_from_mrad = service_method(
        "geometry", "_optical_alignment_level_from_mrad", GeometryInputAdapter
    )
    _optical_alignment_overall_level = service_method(
        "geometry", "_optical_alignment_overall_level", GeometryInputAdapter
    )
    _optical_alignment_level_text = service_method(
        "presentation", "_optical_alignment_level_text", MeasurementPresentationService
    )
    _format_optical_px = service_method(
        "presentation", "_format_optical_px", MeasurementPresentationService
    )
    _format_optical_mrad = service_method(
        "presentation", "_format_optical_mrad", MeasurementPresentationService
    )
    _refresh_optical_alignment_view = service_method(
        "presentation",
        "_refresh_optical_alignment_view",
        MeasurementPresentationService,
    )
    _gnss_readiness_config = service_method(
        "capture", "_gnss_readiness_config", PairCaptureService
    )
    _measurement_correction_frame_token = service_method(
        "capture", "_measurement_correction_frame_token", PairCaptureService
    )
    _attitude_state_is_valid_for_measurement_cache = service_method(
        "capture", "_attitude_state_is_valid_for_measurement_cache", PairCaptureService
    )
    _cached_measurement_attitude_states = service_method(
        "capture", "_cached_measurement_attitude_states", PairCaptureService
    )
    _gps_state_is_valid_for_measurement_cache = service_method(
        "capture", "_gps_state_is_valid_for_measurement_cache", PairCaptureService
    )
    _cached_measurement_gps_states = service_method(
        "capture", "_cached_measurement_gps_states", PairCaptureService
    )
    _ftp_command_round_sync = service_method(
        "capture", "_ftp_command_round_sync", PairCaptureService
    )
    _pair_timing_decision = service_method(
        "capture", "_pair_timing_decision", PairCaptureService
    )
    _freeze_measurement_pair_inputs = service_method(
        "capture", "_freeze_measurement_pair_inputs", PairCaptureService
    )
    _build_measurement_attempt = service_method(
        "geometry", "_build_measurement_attempt", GeometryInputAdapter
    )
    _translation_right_in_left_m = service_method(
        "geometry", "_translation_right_in_left_m", GeometryInputAdapter
    )
    _latest_translation_right_in_left_m_for_diagnostics = service_method(
        "geometry",
        "_latest_translation_right_in_left_m_for_diagnostics",
        GeometryInputAdapter,
    )
    _alignment_view_camera_translation_right_in_left_m = service_method(
        "geometry",
        "_alignment_view_camera_translation_right_in_left_m",
        GeometryInputAdapter,
    )
    _capture_measurement_gnss_contract = service_method(
        "geometry", "_capture_measurement_gnss_contract", GeometryInputAdapter
    )
    _measurement_gnss_record_metadata = service_method(
        "geometry", "_measurement_gnss_record_metadata", GeometryInputAdapter
    )
    _geometry_comparison_rows = service_method(
        "geometry", "_geometry_comparison_rows", GeometryInputAdapter
    )
    _multi_method_measurement_active = service_method(
        "runtime", "_multi_method_measurement_active", WorkerRuntimeService
    )
    method_evaluation_pending_count = service_method(
        "runtime", "method_evaluation_pending_count", WorkerRuntimeService
    )
    _plan_method_evaluations = service_method(
        "planning", "_plan_method_evaluations", MethodPlanningService
    )
    _method_evaluation_requests_for_pair = service_method(
        "planning", "_method_evaluation_requests_for_pair", MethodPlanningService
    )
    _submit_method_evaluations = service_method(
        "runtime", "_submit_method_evaluations", WorkerRuntimeService
    )
    _primary_outcome_key = service_method(
        "projection", "_primary_outcome_key", ResultProjectionService
    )
    _failed_primary_outcome = service_method(
        "projection", "_failed_primary_outcome", ResultProjectionService
    )
    _primary_outcome_is_current = service_method(
        "commit", "_primary_outcome_is_current", ResultCommitService
    )
    _normalize_primary_outcome = service_method(
        "commit", "_normalize_primary_outcome", ResultCommitService
    )
    _commit_primary_method_outcome = service_method(
        "commit", "_commit_primary_method_outcome", ResultCommitService
    )
    _persist_late_primary_measurement = service_method(
        "commit", "_persist_late_primary_measurement", ResultCommitService
    )
    _retry_pending_primary_measurements = service_method(
        "commit", "_retry_pending_primary_measurements", ResultCommitService
    )
    _display_primary_method_outcome = service_method(
        "presentation",
        "_display_primary_method_outcome",
        MeasurementPresentationService,
    )
    _update_method_evaluation_counters = service_method(
        "commit", "_update_method_evaluation_counters", ResultCommitService
    )
    _method_bundle_key = service_method(
        "projection", "_method_bundle_key", ResultProjectionService
    )
    _method_bundle_is_current = service_method(
        "commit", "_method_bundle_is_current", ResultCommitService
    )
    _method_bundle_record = service_method(
        "commit", "_method_bundle_record", ResultCommitService
    )
    _write_admitted = service_method("commit", "_write_admitted", ResultCommitService)
    _method_bundle_row_owner_usable = service_method(
        "commit", "_method_bundle_row_owner_usable", ResultCommitService
    )
    _persist_method_evaluation_bundle = service_method(
        "commit", "_persist_method_evaluation_bundle", ResultCommitService
    )
    _retry_pending_method_bundles = service_method(
        "commit", "_retry_pending_method_bundles", ResultCommitService
    )
    _row_scalar = service_method("projection", "_row_scalar", ResultProjectionService)
    _method_row_prefix = service_method(
        "projection", "_method_row_prefix", ResultProjectionService
    )
    _optical_relative_rotation_fields = service_method(
        "geometry", "_optical_relative_rotation_fields", GeometryInputAdapter
    )
    _method_result_row = service_method(
        "projection", "_method_result_row", ResultProjectionService
    )
    _persist_method_result_rows = service_method(
        "commit", "_persist_method_result_rows", ResultCommitService
    )
    _bundle_flat_fields = service_method(
        "projection", "_bundle_flat_fields", ResultProjectionService
    )
    _flatten_bundle_into_measurement_row = service_method(
        "commit", "_flatten_bundle_into_measurement_row", ResultCommitService
    )
    _commit_method_evaluation_bundle = service_method(
        "commit", "_commit_method_evaluation_bundle", ResultCommitService
    )
    _method_bundle_with_primary_failure = service_method(
        "projection", "_method_bundle_with_primary_failure", ResultProjectionService
    )
    _method_history_payload = service_method(
        "projection", "_method_history_payload", ResultProjectionService
    )
    _poll_method_evaluations = service_method(
        "runtime", "_poll_method_evaluations", WorkerRuntimeService
    )
    _refresh_distance = service_method(
        "presentation", "_refresh_distance", MeasurementPresentationService
    )
    _show_measurement_pair = service_method(
        "presentation", "_show_measurement_pair", MeasurementPresentationService
    )
    _clear_measurement_pair_display = service_method(
        "presentation",
        "_clear_measurement_pair_display",
        MeasurementPresentationService,
    )
    _clear_selected_runtime_measurement_pair = service_method(
        "presentation",
        "_clear_selected_runtime_measurement_pair",
        MeasurementPresentationService,
    )
    _detection_wait_detail = service_method(
        "presentation", "_detection_wait_detail", MeasurementPresentationService
    )
    _detection_wait_status = service_method(
        "presentation", "_detection_wait_status", MeasurementPresentationService
    )
    _stereo_geometry_wait_detail = service_method(
        "presentation", "_stereo_geometry_wait_detail", MeasurementPresentationService
    )
    _triangulation_wait_detail = service_method(
        "presentation", "_triangulation_wait_detail", MeasurementPresentationService
    )
    _format_distance_label_text = service_method(
        "presentation", "_format_distance_label_text", MeasurementPresentationService
    )
    _format_quality_point = service_method(
        "presentation", "_format_quality_point", MeasurementPresentationService
    )
    _format_attitude_timing_age = service_method(
        "presentation", "_format_attitude_timing_age", MeasurementPresentationService
    )
    _attitude_timing_health_level = service_method(
        "presentation", "_attitude_timing_health_level", MeasurementPresentationService
    )
    _format_attitude_timing_text = service_method(
        "presentation", "_format_attitude_timing_text", MeasurementPresentationService
    )
    _clear_triangulation_quality = service_method(
        "presentation", "_clear_triangulation_quality", MeasurementPresentationService
    )
    _update_triangulation_quality = service_method(
        "presentation", "_update_triangulation_quality", MeasurementPresentationService
    )
    _set_distance_text = service_method(
        "presentation", "_set_distance_text", MeasurementPresentationService
    )
    _distance_style = service_method(
        "presentation", "_distance_style", MeasurementPresentationService
    )
    _ensure_method_evaluation_timer = service_method(
        "runtime", "_ensure_method_evaluation_timer", WorkerRuntimeService
    )
    _stop_method_evaluation_timer = service_method(
        "runtime", "_stop_method_evaluation_timer", WorkerRuntimeService
    )

    def resource_inventory(self) -> dict[str, object]:
        return self.services.runtime.resource_inventory()

    # Finite compatibility state access for existing recovery/diagnostic clients.
    _method_evaluation_coordinator = service_state(
        "runtime", "_method_evaluation_coordinator"
    )
    _method_evaluation_timer = service_state("runtime", "_method_evaluation_timer")
    _detection_pair_coordinator = service_state(
        "capture", "_detection_pair_coordinator"
    )
    _detection_pair_timer = service_state("capture", "_detection_pair_timer")
    _pairing_policy_token = service_state("capture", "_pairing_policy_token")
    _pairing_policy_revision = service_state("capture", "_pairing_policy_revision")
    _preview_measurement_coordinator = service_state(
        "capture", "_preview_measurement_coordinator"
    )
    _preview_measurement_timer = service_state("capture", "_preview_measurement_timer")
    _preview_measurement_results = service_state(
        "capture", "_preview_measurement_results"
    )
    _preview_measurement_pending = service_state(
        "capture", "_preview_measurement_pending"
    )
    _primary_outcome_hash_by_key = service_state(
        "commit", "_primary_outcome_hash_by_key"
    )
    _primary_normalized_outcome_by_key = service_state(
        "commit", "_primary_normalized_outcome_by_key"
    )
    _primary_measurement_persisted = service_state(
        "commit", "_primary_measurement_persisted"
    )
    _pending_primary_measurement_commits = service_state(
        "commit", "_pending_primary_measurement_commits"
    )
    _pending_primary_measurement_sizes = service_state(
        "commit", "_pending_primary_measurement_sizes"
    )
    _pending_primary_measurement_bytes = service_state(
        "commit", "_pending_primary_measurement_bytes"
    )
    _primary_history_projected = service_state("commit", "_primary_history_projected")
    _primary_display_high_water = service_state(
        "presentation", "_primary_display_high_water"
    )
    _primary_displayed_key = service_state("presentation", "_primary_displayed_key")
    _pending_method_bundle_commits = service_state(
        "commit", "_pending_method_bundle_commits"
    )
    _pending_method_bundle_sizes = service_state(
        "commit", "_pending_method_bundle_sizes"
    )
    _pending_method_bundle_bytes = service_state(
        "commit", "_pending_method_bundle_bytes"
    )
    _persisted_method_bundle_keys = service_state(
        "commit", "_persisted_method_bundle_keys"
    )
    _persistence_pending_overflow_count = service_state(
        "commit", "_persistence_pending_overflow_count"
    )
    _persistence_pending_overflow_evidence_failures = service_state(
        "commit", "_persistence_pending_overflow_evidence_failures"
    )
    _last_persistence_pending_overflow = service_state(
        "commit", "_last_persistence_pending_overflow"
    )
    _method_evaluation_shutdown_close_retry_calls = service_state(
        "runtime", "_method_evaluation_shutdown_close_retry_calls"
    )
