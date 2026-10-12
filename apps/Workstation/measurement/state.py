"""Single ownership for Measurement feature state.

Producer contracts are unchanged: attitudes body FRD -> NED; GNSS deltas NED;
optical transforms RDF; public points baseline-center FRD. These fields hold
runtime state, never a second persistent settings format.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
    from PyQt5 import QtCore
    from Workstation.workstation_core.measurement_attempt import MeasurementAttempt
    from Workstation.workstation_core.calibration import StereoIntrinsicsResult
    from Workstation.workstation_core.method_evaluation import (
        MethodEvaluationOutcome,
        MethodEvaluationBundle,
    )
    from Workstation.workstation_core.method_evaluation_workers import (
        MethodEvaluationCoordinator,
    )
    from Workstation.workstation_core.detection_pair_coordinator import (
        DetectionPairCoordinator,
    )
    from Workstation.workstation_core.preview_measurement_worker import (
        PreviewMeasurementCoordinator,
        PreviewMeasurementResult,
    )
    from Workstation.measurement.pending_commit import PendingMethodBundleCommit
    from Workstation.measurement.frozen_inputs import FrozenPairInputs


class _Unset:
    def __repr__(self) -> str:
        return "UNSET"


UNSET = _Unset()


def read(value: Any, default: Any = UNSET) -> Any:
    """Preserve optional legacy fields without manufacturing a sensor value."""
    if value is UNSET:
        if default is UNSET:
            raise AttributeError("Measurement field is not initialized")
        return default
    return value


@dataclass
class MethodSelectionState:
    """State owned by MethodSelectionService."""

    _measurement_preferred_method_id: str | _Unset = UNSET
    _measurement_active_fallback_order: tuple[str, ...] | _Unset = UNSET
    _measurement_baseline_yaw_reverse: bool | _Unset = UNSET
    _measurement_method_fill_policy: dict[str, dict[str, str]] | _Unset = UNSET
    _measurement_frozen_comparison_inputs_declared: bool | _Unset = UNSET
    _measurement_frozen_comparison_inputs: dict[str, object] | _Unset = UNSET
    _measurement_method_config_token: tuple[str, int] | _Unset = UNSET
    _measurement_selected_method_id: str | _Unset = UNSET
    _measurement_selected_instance_id: str | _Unset = UNSET
    _measurement_selection_reason: str | _Unset = UNSET
    _measurement_method_transition_sequence: int | _Unset = UNSET
    _measurement_method_ready_since: dict[str, float] | _Unset = UNSET
    _pending_measurement_frozen_intrinsics_result: (
        StereoIntrinsicsResult | None | _Unset
    ) = UNSET
    _pending_measurement_frozen_intrinsics_path: str | _Unset = UNSET
    _pending_measurement_frozen_comparison_objects: dict[str, object] | _Unset = UNSET


@dataclass
class PairCaptureState:
    """State owned by PairCaptureService."""

    _coordinated_detection_pair_reservation: Any = UNSET
    _measurement_active_time_basis_requested: str | _Unset = UNSET
    _coordinated_detection_pair: Any = UNSET
    _coordinated_pairing_result: Any = UNSET
    _last_frozen_measurement_pair: FrozenPairInputs | None | _Unset = UNSET
    _detection_pair_coordinator_active: bool | _Unset = UNSET
    _last_correspondence_rejection_reason: str | _Unset = UNSET
    _last_pair_advisories: tuple[str, ...] | _Unset = UNSET
    _runtime_pair_uses_app_fallback: bool | _Unset = UNSET
    _detection_pair_metrics: dict[str, object] | _Unset = UNSET
    _latest_preview_measurement_result: PreviewMeasurementResult | None | _Unset = UNSET
    _latest_preview_pair_identity: str | _Unset = UNSET
    _preview_measurement_metrics: dict[str, object] | _Unset = UNSET
    _measurement_latest_valid_attitude: dict[str, object] | _Unset = UNSET
    _measurement_gps_correction_frame_dropped: tuple[str, ...] | _Unset = UNSET
    _measurement_latest_valid_gps: dict[str, object] | _Unset = UNSET
    _measurement_latest_valid_gps_source: dict[str, str] | _Unset = UNSET
    _measurement_latest_valid_gps_correction_frame: dict[str, str] | _Unset = UNSET
    _measurement_gps_correction_frame_barrier_s: float | _Unset = UNSET
    _measurement_attempt_counter: int | _Unset = UNSET
    _measurement_attempt_generation_id: str | _Unset = UNSET
    _detection_pair_coordinator: DetectionPairCoordinator | None = None
    _detection_pair_timer: QtCore.QTimer | None = None
    _pairing_policy_token: tuple[object, ...] | None = None
    _pairing_policy_revision: int = 0
    _preview_measurement_coordinator: PreviewMeasurementCoordinator | None = None
    _preview_measurement_timer: QtCore.QTimer | None = None
    _preview_measurement_results: dict[str, PreviewMeasurementResult] = field(
        default_factory=dict
    )
    _preview_measurement_pending: set[str] = field(default_factory=set)


@dataclass
class GeometryState:
    """Cache owned by GeometryInputAdapter."""

    _last_measurement_attempt_key: tuple[object, ...] | None | _Unset = UNSET
    _last_measurement_attempt: MeasurementAttempt | None | _Unset = UNSET


@dataclass
class MethodPlanningState:
    """State owned by MethodPlanningService."""

    pass


@dataclass
class WorkerRuntimeState:
    """State owned by WorkerRuntimeService."""

    _measurement_recording_method_id: str | _Unset = UNSET
    _method_evaluation_submitted: set[tuple[str, int]] | _Unset = UNSET
    _method_evaluation_accepting: bool | _Unset = UNSET
    _method_evaluation_stop_retry_scheduled: bool | _Unset = UNSET
    _method_evaluation_coordinator: MethodEvaluationCoordinator | None = None
    _method_evaluation_timer: QtCore.QTimer | None = None
    _method_evaluation_shutdown_close_retry_calls: int = 0


@dataclass
class ResultCommitState:
    """State owned by ResultCommitService."""

    _last_method_evaluation_bundle: MethodEvaluationBundle | None | _Unset = UNSET
    _primary_measurement_retry_status: dict[str, object] | _Unset = UNSET
    _measurement_method_result_counters: dict[str, dict[str, int]] | _Unset = UNSET
    _method_evaluation_committed: set[tuple[str, int]] | _Unset = UNSET
    _primary_outcome_hash_by_key: dict[tuple[str, str, int], str] = field(
        default_factory=dict
    )
    _primary_normalized_outcome_by_key: dict[
        tuple[str, str, int], MethodEvaluationOutcome
    ] = field(default_factory=dict)
    _primary_measurement_persisted: set[tuple[str, str, int]] = field(
        default_factory=set
    )
    _pending_primary_measurement_commits: dict[
        tuple[str, str, int], MethodEvaluationOutcome
    ] = field(default_factory=dict)
    _pending_primary_measurement_sizes: dict[tuple[str, str, int], int] = field(
        default_factory=dict
    )
    _pending_primary_measurement_bytes: int = 0
    _primary_history_projected: set[tuple[str, str, int]] = field(default_factory=set)
    _pending_method_bundle_commits: dict[
        tuple[str, str, int], PendingMethodBundleCommit
    ] = field(default_factory=dict)
    _pending_method_bundle_sizes: dict[tuple[str, str, int], int] = field(
        default_factory=dict
    )
    _pending_method_bundle_bytes: int = 0
    _persisted_method_bundle_keys: set[tuple[str, str, int]] = field(
        default_factory=set
    )
    _persistence_pending_overflow_count: int = 0
    _persistence_pending_overflow_evidence_failures: int = 0
    _last_persistence_pending_overflow: dict[str, object] = field(default_factory=dict)


@dataclass
class ResultProjectionState:
    """State owned by ResultProjectionService."""

    pass


@dataclass
class MeasurementPresentationState:
    """State owned by MeasurementPresentationService."""

    selected_runtime_frame_pair_basis: str | _Unset = UNSET
    selected_runtime_frame_pair_camera_rtc_delta_s: float | None | _Unset = UNSET
    selected_runtime_frame_pair_delta_s: float | None | _Unset = UNSET
    selected_runtime_frame_pair_fallback_reason: str | _Unset = UNSET
    selected_runtime_frame_pair_requested_basis: str | _Unset = UNSET
    last_distance_quality: str | _Unset = UNSET
    last_distance_status: str | _Unset = UNSET
    geometry_comparison_rows: tuple[dict[str, object], ...] | _Unset = UNSET
    _primary_display_high_water: int = -1
    _primary_displayed_key: tuple[str, str, int] | None = None


@dataclass
class MeasurementState:
    methods: MethodSelectionState = field(default_factory=MethodSelectionState)
    capture: PairCaptureState = field(default_factory=PairCaptureState)
    geometry: GeometryState = field(default_factory=GeometryState)
    planning: MethodPlanningState = field(default_factory=MethodPlanningState)
    runtime: WorkerRuntimeState = field(default_factory=WorkerRuntimeState)
    commit: ResultCommitState = field(default_factory=ResultCommitState)
    projection: ResultProjectionState = field(default_factory=ResultProjectionState)
    presentation: MeasurementPresentationState = field(
        default_factory=MeasurementPresentationState
    )


MEASUREMENT_PANEL_FIELDS = {
    "_last_method_evaluation_bundle": "commit",
    "_primary_measurement_retry_status": "commit",
    "_coordinated_detection_pair_reservation": "capture",
    "_measurement_preferred_method_id": "methods",
    "_measurement_active_fallback_order": "methods",
    "_measurement_baseline_yaw_reverse": "methods",
    "_measurement_active_time_basis_requested": "capture",
    "_measurement_method_fill_policy": "methods",
    "_measurement_frozen_comparison_inputs_declared": "methods",
    "_measurement_frozen_comparison_inputs": "methods",
    "_measurement_method_config_token": "methods",
    "_measurement_selected_method_id": "methods",
    "_measurement_selected_instance_id": "methods",
    "_measurement_selection_reason": "methods",
    "_measurement_method_transition_sequence": "methods",
    "_measurement_method_ready_since": "methods",
    "_pending_measurement_frozen_intrinsics_result": "methods",
    "_pending_measurement_frozen_intrinsics_path": "methods",
    "_pending_measurement_frozen_comparison_objects": "methods",
    "_last_measurement_attempt_key": "geometry",
    "_last_measurement_attempt": "geometry",
    "_coordinated_detection_pair": "capture",
    "_coordinated_pairing_result": "capture",
    "_last_frozen_measurement_pair": "capture",
    "_detection_pair_coordinator_active": "capture",
    "_last_correspondence_rejection_reason": "capture",
    "_last_pair_advisories": "capture",
    "_runtime_pair_uses_app_fallback": "capture",
    "_detection_pair_metrics": "capture",
    "_latest_preview_measurement_result": "capture",
    "_latest_preview_pair_identity": "capture",
    "_measurement_recording_method_id": "runtime",
    "_preview_measurement_metrics": "capture",
    "_measurement_latest_valid_attitude": "capture",
    "_measurement_gps_correction_frame_dropped": "capture",
    "_measurement_latest_valid_gps": "capture",
    "_measurement_latest_valid_gps_source": "capture",
    "_measurement_latest_valid_gps_correction_frame": "capture",
    "_measurement_gps_correction_frame_barrier_s": "capture",
    "_measurement_attempt_counter": "capture",
    "_method_evaluation_submitted": "runtime",
    "_measurement_method_result_counters": "commit",
    "_method_evaluation_committed": "commit",
    "_measurement_attempt_generation_id": "capture",
    "_method_evaluation_accepting": "runtime",
    "_method_evaluation_stop_retry_scheduled": "runtime",
    "selected_runtime_frame_pair_basis": "presentation",
    "selected_runtime_frame_pair_camera_rtc_delta_s": "presentation",
    "selected_runtime_frame_pair_delta_s": "presentation",
    "selected_runtime_frame_pair_fallback_reason": "presentation",
    "selected_runtime_frame_pair_requested_basis": "presentation",
    "last_distance_quality": "presentation",
    "last_distance_status": "presentation",
    "geometry_comparison_rows": "presentation",
}


def measurement_state_for(host: object) -> MeasurementState:
    """Allocate one feature state; called only at the outer compatibility edge."""
    state = getattr(host, "_measurement_feature_state", None)
    if state is None:
        state = MeasurementState()
        setattr(host, "_measurement_feature_state", state)
    return state


def measurement_state_property(name: str) -> property:
    """A finite, named compatibility property; it stores no copy on the panel."""
    domain = MEASUREMENT_PANEL_FIELDS[name]

    def get(host: object) -> Any:
        return read(getattr(getattr(measurement_state_for(host), domain), name))

    def set_value(host: object, value: Any) -> None:
        setattr(getattr(measurement_state_for(host), domain), name, value)

    return property(get, set_value)
